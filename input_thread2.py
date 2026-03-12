#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
input_thread.py — RHex Rover serial reader + consumer (Raspberry Pi 3B+)

Architecture
------------
INPUT thread   opens /dev/ttyUSB0 at 115200 baud, reads one 20-column CSV
               line per frame from the Arduino Nano, parses it, and pushes
               the latest frame into a single-slot keep-latest queue.

CONSUMER thread pulls frames from the queue, normalises the UpsideDown remap
               when needed (and clears the flag afterwards), calls
               RHexSystem.update(), and prints the 10-field output array at
               a rate-limited pace to avoid stdout bottlenecks on the Pi.

Watchdog       if no fresh frame arrives within WATCHDOG_MS the consumer
               injects a safe dummy frame with watchdog_ok=False, which
               forces the fusion FSM into STOP_SAFE.

CSV format produced by rhex_logger.ino (20 columns, 0-indexed):
  [0]    timestamp_ms
  [1-8]  FDL, FCF, FCD, FDR, RDL, RCF, RCD, RDR   (cm; -1 = very-near)
  [9-12] QuatW, QuatX, QuatY, QuatZ
  [13-15]AccelX, AccelY, AccelZ                     (m/s²)
  [16-18]GyroX, GyroY, GyroZ                        (rad/s)
  [19]   UpsideDown                                  (0 / 1)

UpsideDown remap (preserve body-frame L/R):
  Input:  [FDL, FCF, FCD, FDR,  RDL, RCF, RCD, RDR]
  Output: [RDL, RCF, RCD, RDR,  FDL, FCF, FCD, FDR]
  After remap upside_down is set to 0 so fusion computes quality correctly.

Requirements:
  pip install pyserial
"""

import json
import queue
import signal
import sys
import threading
import time
from typing import Dict, List, Optional

try:
    import serial
    import serial.serialutil
except ImportError:
    print("[FATAL] pyserial is not installed.  Run:  pip install pyserial")
    sys.exit(1)

from fusion2 import (
    CLASS_NAMES,
    FINAL_FIELDS,
    STATE_NAMES,
    DecisionConfig,
    FusionConfig,
    RHexSystem,
    remap_preserve_left_right,
)

# ================================================================
# CONFIGURATION
# ================================================================

SERIAL_PORT  = "/dev/ttyUSB0"    # change to "/dev/ttyACM0" for some boards
BAUDRATE     = 115200
READ_TIMEOUT = 0.12              # seconds — keeps the loop responsive

WATCHDOG_MS  = 300               # ms without a frame → inject safe dummy

# Minimum seconds between consecutive output prints (~5 Hz)
PRINT_RATE_S = 0.20

# Debug: print every N raw input frames (0 = disabled)
INPUT_DEBUG_EVERY = 0

# Pi-side distance clamp (second safety net; Nano already clamps)
CLAMP_DISTANCES = True
DIST_MIN_CM     = -1.0            # preserve -1 sentinel
DIST_MAX_CM     = 450.0

# Serial reconnection back-off
_BACKOFF_INIT = 2.0
_BACKOFF_MULT = 1.5
_BACKOFF_MAX  = 30.0

# ================================================================
# SHARED STATE
# ================================================================

_stop_evt: threading.Event = threading.Event()
_frame_q:  queue.Queue     = queue.Queue(maxsize=1)   # keep-latest policy

# ================================================================
# HELPERS
# ================================================================

def _safe_dummy_frame() -> Dict:
    """Watchdog safety frame: all distances unknown, rover level."""
    return {
        "timestamp_ms": int(time.time() * 1000),
        "distances":    [-1.0] * 8,
        "upside_down":  0,
        "quat":         {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
    }


def _push_latest(frame: Dict) -> None:
    """Keep-latest enqueue: silently discard the stale entry if full."""
    try:
        _frame_q.put_nowait(frame)
    except queue.Full:
        try:
            _frame_q.get_nowait()
        except queue.Empty:
            pass
        try:
            _frame_q.put_nowait(frame)
        except queue.Full:
            pass   # consumer is dead; drop silently


# ================================================================
# INPUT THREAD
# ================================================================

def _input_thread() -> None:
    """
    Read CSV lines from the Nano, parse them, push to _frame_q.
    Reconnects automatically on serial errors with exponential back-off.
    """
    ser:      Optional[serial.Serial] = None
    backoff   = _BACKOFF_INIT
    dbg_count = 0

    while not _stop_evt.is_set():
        try:
            # (Re)open the serial port if needed
            if ser is None or not ser.is_open:
                ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=READ_TIMEOUT)
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                print(f"[INPUT] Connected → {SERIAL_PORT} @ {BAUDRATE} baud")
                backoff = _BACKOFF_INIT     # reset back-off on success

            raw = ser.readline()
            if not raw:
                continue                    # timeout, no data this cycle

            line = raw.decode("utf-8", errors="replace").strip()
            if not line or line.startswith("timestamp_ms"):
                continue                    # skip empty lines and CSV header

            parts = line.split(",")
            if len(parts) != 20:
                continue                    # malformed row — skip quietly

            # Parse all 20 fields; skip row on any conversion error
            try:
                ts_ms      = int(parts[0])
                distances  = [float(p) for p in parts[1:9]]
                qw, qx     = float(parts[9]),  float(parts[10])
                qy, qz     = float(parts[11]), float(parts[12])
                ax, ay, az = float(parts[13]), float(parts[14]), float(parts[15])
                gx, gy, gz = float(parts[16]), float(parts[17]), float(parts[18])
                upside     = int(parts[19])
            except (ValueError, IndexError):
                continue

            # Pi-side distance clamp (second safety net)
            if CLAMP_DISTANCES:
                distances = [
                    max(DIST_MIN_CM, min(DIST_MAX_CM, d)) for d in distances
                ]

            frame: Dict = {
                "timestamp_ms": ts_ms,
                "distances":    distances,
                "quat":         {"w": qw, "x": qx, "y": qy, "z": qz},
                "accel":        {"x": ax, "y": ay, "z": az},
                "gyro":         {"x": gx, "y": gy, "z": gz},
                "upside_down":  1 if upside else 0,
            }

            if INPUT_DEBUG_EVERY > 0:
                dbg_count += 1
                if dbg_count % INPUT_DEBUG_EVERY == 0:
                    print("[INPUT]", json.dumps(frame, ensure_ascii=False))

            _push_latest(frame)

        except serial.SerialException as exc:
            print(f"[INPUT] Serial error: {exc!r}  — retry in {backoff:.1f} s")
            try:
                if ser and ser.is_open:
                    ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(backoff)
            backoff = min(backoff * _BACKOFF_MULT, _BACKOFF_MAX)

        except Exception as exc:
            print(f"[INPUT] Unexpected error: {exc!r}")
            time.sleep(0.02)

    # Clean shutdown
    try:
        if ser and ser.is_open:
            ser.close()
    except Exception:
        pass
    print("[INPUT] Thread stopped.")


# ================================================================
# CONSUMER THREAD
# ================================================================

def _consumer_thread() -> None:
    """
    Pull frames, apply UpsideDown remap if needed, run fusion+decision,
    and print the 10-field output array at a rate-limited pace.

    Key design points
    -----------------
    - After the front/rear remap, upside_down is set to 0 so that
      FusionEngine computes upright_quality from the pitch angle normally
      (the remapped frame is treated as a normalised body-frame view).
    - Watchdog: injects a dummy safe frame after WATCHDOG_MS with
      watchdog_ok=False, which forces STOP_SAFE in the FSM.
    - FPS counter refreshes every ~2 s for display.
    """
    system = RHexSystem(
        fcfg=FusionConfig(APPLY_SWAP=False),   # remap handled here, not inside fusion
        dcfg=DecisionConfig(),
    )

    print(f"[CONSUMER] Output fields : {FINAL_FIELDS}")

    last_frame_at  = time.monotonic()
    last_print_at  = 0.0
    frames_window  = 0
    window_start   = time.monotonic()

    while not _stop_evt.is_set():
        try:
            frame = _frame_q.get(timeout=0.05)
            last_frame_at = time.monotonic()
            watchdog_ok   = True

        except queue.Empty:
            elapsed_ms = (time.monotonic() - last_frame_at) * 1000.0
            if elapsed_ms < WATCHDOG_MS:
                continue
            # Watchdog expired: force STOP_SAFE this cycle
            frame       = _safe_dummy_frame()
            watchdog_ok = False

        # Save the raw Nano frame before any remap so we can print it faithfully.
        nano_frame = frame

        # UpsideDown remap: swap front/rear while preserving body-frame L/R.
        # Set upside_down=0 afterwards so fusion treats the frame as normalised.
        if frame.get("upside_down", 0) and len(frame.get("distances", [])) == 8:
            frame = dict(frame)                             # shallow copy
            frame["distances"]   = remap_preserve_left_right(frame["distances"])
            frame["upside_down"] = 0                        # mark as normalised

        try:
            arr = system.update(frame, watchdog_ok=watchdog_ok)
        except Exception as exc:
            print(f"[CONSUMER] update() error: {exc!r}")
            continue

        frames_window += 1
        now = time.monotonic()

        if now - last_print_at >= PRINT_RATE_S:
            # Approximate FPS for the current window
            window_s = now - window_start
            fps = frames_window / window_s if window_s > 0 else 0.0
            if window_s >= 2.0:
                frames_window = 0
                window_start  = now

            state_name = STATE_NAMES.get(arr[1], f"STATE_{arr[1]}")
            f_name     = CLASS_NAMES.get(arr[5], "?")
            l_name     = CLASS_NAMES.get(arr[6], "?")
            r_name     = CLASS_NAMES.get(arr[7], "?")
            wdog_tag   = " [WATCHDOG]" if not watchdog_ok else ""

            # ── Raw data from the Nano ─────────────────────────────────────
            _SENSOR_LABELS = ("FDL", "FCF", "FCD", "FDR", "RDL", "RCF", "RCD", "RDR")
            nd   = nano_frame.get("distances", [-1.0] * 8)
            nq   = nano_frame.get("quat",  {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})
            na   = nano_frame.get("accel", {"x": 0.0, "y": 0.0, "z": 0.0})
            ng   = nano_frame.get("gyro",  {"x": 0.0, "y": 0.0, "z": 0.0})
            dist_str = "  ".join(
                f"{lbl}={v:6.1f}" for lbl, v in zip(_SENSOR_LABELS, nd)
            )
            print(
                f"[NANO{wdog_tag}] "
                f"ts={nano_frame.get('timestamp_ms', 0):>8}ms  "
                f"{dist_str}  "
                f"quat=[{nq['w']:+.3f},{nq['x']:+.3f},{nq['y']:+.3f},{nq['z']:+.3f}]  "
                f"acc=[{na['x']:+.2f},{na['y']:+.2f},{na['z']:+.2f}]  "
                f"gyro=[{ng['x']:+.3f},{ng['y']:+.3f},{ng['z']:+.3f}]  "
                f"up={nano_frame.get('upside_down', 0)}"
            )
            # ── Fusion decision output ─────────────────────────────────────
            print(
                f"[OUT {fps:4.1f}FPS{wdog_tag}] "
                f"ts={arr[0]:>8}ms  "
                f"state={state_name:<20}  "
                f"turn={arr[2]:+.3f}  speed={arr[3]:.2f}  tripod={arr[4]:.2f}  "
                f"F={f_name:<8} L={l_name:<8} R={r_name:<8}  "
                f"cliff={arr[8]}  upr={arr[9]:.3f}"
            )
            last_print_at = now

    print("[CONSUMER] Thread stopped.")


# ================================================================
# MAIN
# ================================================================

def main() -> None:
    """Start INPUT and CONSUMER threads; wait for SIGINT / SIGTERM."""

    def _handle_signal(sig, _frame):
        print(f"\n[MAIN] Signal {sig} received → shutting down…")
        _stop_evt.set()

    signal.signal(signal.SIGINT,  _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    t_input    = threading.Thread(
        target=_input_thread,    name="INPUT",    daemon=True
    )
    t_consumer = threading.Thread(
        target=_consumer_thread, name="CONSUMER", daemon=True
    )

    t_input.start()
    t_consumer.start()

    try:
        while not _stop_evt.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        _stop_evt.set()
    finally:
        t_input.join(timeout=3.0)
        t_consumer.join(timeout=3.0)
        print("[MAIN] Closed.")


if __name__ == "__main__":
    main()