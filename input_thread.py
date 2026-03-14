
import threading
import time
import serial
import queue
import json
import sys

# Import your hybrid system (fusion + decision)
from fusion import RHexSystem, FusionConfig, DecisionConfig, FINAL_FIELDS

# ----------------------------- Configuration -----------------------------

SERIAL_PORT_NANO = "/dev/ttyUSB0"
BAUDRATE_NANO    = 115200
READ_TIMEOUT_S   = 0.1

# Queue: keep only the latest frame to minimize latency
MAX_QUEUE_SIZE   = 1

# Input debug printing (raw frame dict) every N frames (0 = disable)
INPUT_PRINT_EVERY = 0   # set to 10 if to see a frame dict every 10 frames

# Consumer print rate limit (seconds). Use 0 to print every frame (not recommended on Pi 3B)
PRINT_PERIOD_S   = 0.2  # 5 Hz

# Watchdog threshold in milliseconds (no fresh frame for this time => STOP_SAFE)
WATCHDOG_MS      = 300

# Optional: clamp distances on Pi side (a second safety net; Nano already clamps)
CLAMP_DISTANCES  = True
DIST_MIN_CM      = -1.0     # keep -1 for unknown/no-echo
DIST_MAX_CM      = 450.0

# ----------------------------- Globals -----------------------------------

stop_event = threading.Event()
frames_q = queue.Queue(maxsize=MAX_QUEUE_SIZE)


# ----------------------------- Input Thread ------------------------------

def input_thread():
    """
    INPUT THREAD
      - Open/reopen the serial port to the Nano (auto-reconnect)
      - Read one CSV line per frame (20 columns)
      - Parse → build 'frame' dict → push into frames_q (keep-latest policy)
    """
    ser = None
    printed = 0

    while not stop_event.is_set():
        try:
            # (Re)connect if needed
            if ser is None or not ser.is_open:
                ser = serial.Serial(
                    port=SERIAL_PORT_NANO,
                    baudrate=BAUDRATE_NANO,
                    timeout=READ_TIMEOUT_S
                )
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                print(f"[INPUT] Connected {SERIAL_PORT_NANO} @ {BAUDRATE_NANO}")

            # Read one line (bytes)
            raw = ser.readline()
            if not raw:
                # No data this cycle (timeout); loop to check stop_event
                continue

            # Decode and strip newline/whitespace
            line = raw.decode("utf-8", errors="replace").strip()

            # Skip header and empty lines
            if not line or line.startswith("timestamp_ms"):
                continue

            # Parse CSV: timestamp, 8 distances, 4 quat, 3 accel, 3 gyro, upside
            parts = line.split(",")
            if len(parts) != 20:
                # Malformed row; show occasionally and skip
                # print(f"[INPUT] Bad column count: {len(parts)} in: {line!r}")
                continue

            try:
                timestamp_ms = int(parts[0])
                distances = [float(x) for x in parts[1:9]]
                qw = float(parts[9]);  qx = float(parts[10])
                qy = float(parts[11]); qz = float(parts[12])
                ax = float(parts[13]); ay = float(parts[14]); az = float(parts[15])
                gx = float(parts[16]); gy = float(parts[17]); gz = float(parts[18])
                upside_down = int(parts[19])
            except ValueError as e:
                # Bad numeric field; log and skip
                # Include the offending line for diagnosis
                print(f"[INPUT] Parse ValueError: {e!r} on line: {line!r}")
                continue

            # Optional: clamp distances (keeps -1 for unknown)
            if CLAMP_DISTANCES:
                distances = [min(max(d, DIST_MIN_CM), DIST_MAX_CM) for d in distances]

            frame = {
                "timestamp_ms": timestamp_ms,
                "distances": distances,  # list of 8 floats (cm); -1 means unknown/no-echo
                "quat": {"w": qw, "x": qx, "y": qy, "z": qz},
                "accel": {"x": ax, "y": ay, "z": az},
                "gyro":  {"x": gx, "y": gy, "z": gz},
                "upside_down": 1 if upside_down else 0
            }

            # Optional input debug print
            if INPUT_PRINT_EVERY and INPUT_PRINT_EVERY > 0:
                printed += 1
                if printed % INPUT_PRINT_EVERY == 0:
                    print("[INPUT] frame:", json.dumps(frame, ensure_ascii=False))

            # Push into queue with "keep latest" policy
            try:
                frames_q.put(frame, timeout=0.002)
            except queue.Full:
                # Discard one old and retry
                try:
                    _ = frames_q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    frames_q.put(frame, timeout=0.002)
                except queue.Full:
                    # If consumer is dead, skip
                    pass

        except serial.SerialException as e:
            print(f"[INPUT] Serial error: {e}. Reconnecting in 2s...")
            try:
                if ser and ser.is_open:
                    ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(2.0)

        except Exception as e:
            # Log unexpected errors with context; do not swallow silently
            print(f"[INPUT] Unexpected error: {e!r}")
            # If you want context, you can also print the last line read (if available)
            # print(f"[INPUT] Last line: {line!r}" if 'line' in locals() else "[INPUT] No line context")
            time.sleep(0.01)

    # Graceful close on exit
    if ser and ser.is_open:
        try:
            ser.close()
        except Exception:
            pass


# ----------------------------- Consumer Loop -----------------------------

def consumer_loop():
    """
    CONSUMER (hybrid system: fusion + decision -> array)
      - Pull frames from queue
      - Apply 300 ms watchdog
      - Call RHexSystem.update() to get the 10-number array per frame
      - Print array at a limited rate to avoid stdout bottleneck
    """
    last_ts_monotonic = time.monotonic()
    last_print = 0.0

    # Instantiate the hybrid system (APPLY_SWAP=False because we swap here)
    system = RHexSystem(
        fcfg=FusionConfig(APPLY_SWAP=False),
        dcfg=DecisionConfig()
    )

    # Print array field names once for reference
    print("[FIELDS]", FINAL_FIELDS)

    while not stop_event.is_set():
        try:
            # Wait up to 50 ms for a new frame
            frame = frames_q.get(timeout=0.05)
            last_ts_monotonic = time.monotonic()

            # Perform upside-down swap here (keep APPLY_SWAP=False in FusionConfig)
            distances = frame["distances"][:]
            upside = 1 if frame.get("upside_down", 0) else 0
            if upside == 1 and len(distances) == 8:
                front = distances[0:4]
                rear  = distances[4:8]
                frame = dict(frame)
                frame["distances"] = rear + front

            # Produce final 10-number array (state + continuous modulation + classes + safety)
            arr = system.update(frame, watchdog_ok=True)

            # Rate-limited print (avoid console bottleneck)
            now = time.monotonic()
            if PRINT_PERIOD_S <= 0.0 or (now - last_print) >= PRINT_PERIOD_S:
                print(arr)
                last_print = now

        except queue.Empty:
            # Watchdog: no fresh frame within WATCHDOG_MS => force STOP_SAFE for this cycle
            elapsed_ms = (time.monotonic() - last_ts_monotonic) * 1000.0
            if elapsed_ms > WATCHDOG_MS:
                dummy_frame = {
                    "timestamp_ms": int(time.time() * 1000),
                    "distances": [-1.0] * 8,  # unknowns so fusion stays conservative
                    "upside_down": 0,
                    "quat": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
                }
                arr = system.update(dummy_frame, watchdog_ok=False)
                now = time.monotonic()
                if PRINT_PERIOD_S <= 0.0 or (now - last_print) >= PRINT_PERIOD_S:
                    print(arr)
                    last_print = now
                time.sleep(0.05)  # small pause to avoid busy loop

        except Exception as e:
            print(f"[CONSUMER] Unexpected error: {e!r}")
            time.sleep(0.01)


# ----------------------------- Main --------------------------------------

def main():
    t_input = threading.Thread(target=input_thread,   name="INPUT",    daemon=False)
    t_cons  = threading.Thread(target=consumer_loop,  name="CONSUMER", daemon=False)
    t_input.start()
    t_cons.start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nMAIN Ctrl+C -> shutting down...")
    finally:
        stop_event.set()
        t_input.join(timeout=2.0)
        t_cons.join(timeout=2.0)
        print("MAIN Closed.")


if __name__ == "__main__":
    main()