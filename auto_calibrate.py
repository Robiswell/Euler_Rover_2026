#!/usr/bin/env python3
"""
auto_calibrate.py -- Gravity-drop HOME_POSITIONS calibration

For each servo: disables torque so the leg hangs freely under gravity,
waits for it to settle, reads the encoder position. That position IS
the vertical home. Repeats 4 times (alternating +/- to cancel friction
bias) and uses circular averaging for stability.

Usage:
    sudo python3 auto_calibrate.py

Requires: scservo_sdk, /dev/ttyUSB1 accessible, robot elevated so legs swing freely.
"""

import math
import sys
import time
from scservo_sdk import PortHandler, PacketHandler

# ---------- Hardware config (must match final_full_gait_test.py) ----------
PORT_NAME      = "/dev/ttyUSB1"
BAUDRATE       = 1000000
SERVO_PROTOCOL = 0

# Register addresses (STS3215 datasheet)
ADDR_MODE             = 33   # 1 byte: 0=position, 1=velocity
ADDR_TORQUE_ENABLE    = 40   # 1 byte: 0=off, 1=on
ADDR_ACCEL            = 41   # 1 byte: acceleration profile
ADDR_GOAL_POSITION    = 42   # 2 bytes: target position (position mode)
ADDR_GOAL_SPEED       = 46   # 2 bytes: movement speed (position mode)
ADDR_TORQUE_LIMIT     = 16   # 2 bytes: max torque (0-1000) -- RAM register
ADDR_PRESENT_POSITION = 56   # 2 bytes: current encoder value (0-4095)

ENCODER_RESOLUTION = 4096    # ticks per revolution

ALL_SERVOS = [1, 2, 3, 4, 5, 6]

SERVO_LABELS = {
    1: "Right Front",
    2: "Left Front",
    3: "Left Middle",
    4: "Left Rear",
    5: "Right Rear",
    6: "Right Middle",
}

# Mirror pairs: (left_id, right_id, label) -- for symmetry reporting only
PAIRS = [
    (2, 1, "Front"),
    (3, 6, "Middle"),
    (4, 5, "Rear"),
]

# Current HOME_POSITIONS from final_full_gait_test.py
HOME_POSITIONS = {
    1: 3474, 2: 954, 3: 1423,
    4: 1613, 5: 3238, 6: 3201,
}

# ---------- Calibration parameters ----------
SETTLE_TIME    = 3.0   # seconds to wait for leg to stop swinging after release
NUM_SAMPLES    = 5     # readings to average after settling
SAMPLE_GAP     = 0.1   # seconds between readings
NUM_TRIALS     = 4     # even count so +/- drops cancel friction bias
TRIAL_LIFT_DEG = 30    # degrees to lift leg before each drop trial
MOVE_SPEED     = 200   # raw speed for repositioning
TORQUE_LIMIT   = 400   # safe torque limit for repositioning
ACCEL_PROFILE  = 20    # acceleration profile


def read_position(pkt, port, sid):
    """Read current encoder position (0-4095)."""
    pos, result, error = pkt.read2ByteTxRx(port, sid, ADDR_PRESENT_POSITION)
    if result != 0:
        return None
    return pos


def move_to(pkt, port, sid, target):
    """Move servo to target position in position mode and wait for arrival."""
    target = max(0, min(4095, target))
    # Enable position mode
    pkt.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, 0)
    pkt.write1ByteTxRx(port, sid, ADDR_MODE, 0)  # position mode
    pkt.write1ByteTxRx(port, sid, ADDR_ACCEL, ACCEL_PROFILE)
    pkt.write2ByteTxRx(port, sid, ADDR_TORQUE_LIMIT, TORQUE_LIMIT)
    pkt.write2ByteTxRx(port, sid, ADDR_GOAL_SPEED, MOVE_SPEED)
    pkt.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, 1)
    pkt.write2ByteTxRx(port, sid, ADDR_GOAL_POSITION, target)
    # Wait for arrival
    deadline = time.time() + 3.0
    while time.time() < deadline:
        time.sleep(0.05)
        pos = read_position(pkt, port, sid)
        if pos is not None and abs(pos - target) <= 5:
            break
    time.sleep(0.2)  # vibration settle


def release_servo(pkt, port, sid):
    """Disable torque so leg hangs freely."""
    pkt.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, 0)


def circular_avg(readings):
    """Average encoder readings using circular mean (handles 0/4095 wrap)."""
    angles = [r * 2 * math.pi / ENCODER_RESOLUTION for r in readings]
    avg_angle = math.atan2(
        sum(math.sin(a) for a in angles),
        sum(math.cos(a) for a in angles)
    )
    if avg_angle < 0:
        avg_angle += 2 * math.pi
    return round(avg_angle * ENCODER_RESOLUTION / (2 * math.pi))


def read_settled_position(pkt, port, sid, n_samples=NUM_SAMPLES):
    """Read position multiple times and return circular average (filters jitter + wrap)."""
    readings = []
    for _ in range(n_samples):
        pos = read_position(pkt, port, sid)
        if pos is not None:
            readings.append(pos)
        time.sleep(SAMPLE_GAP)
    if not readings:
        return None
    return circular_avg(readings)


def calibrate_servo(pkt, port, sid):
    """
    Calibrate one servo using gravity-drop method:
    1. Lift leg 30 degrees from current HOME
    2. Release (disable torque)
    3. Wait for leg to settle under gravity
    4. Read position = vertical
    5. Repeat from opposite side
    6. Average all readings
    """
    center = HOME_POSITIONS[sid]
    lift_ticks = int(TRIAL_LIFT_DEG / 360.0 * ENCODER_RESOLUTION)
    label = SERVO_LABELS.get(sid, f"Servo {sid}")

    print(f"\n  {'='*50}")
    print(f"  Servo {sid} ({label})")
    print(f"  Current HOME: {center}")
    print(f"  {'='*50}")

    all_readings = []

    for trial in range(NUM_TRIALS):
        # Alternate lifting direction: +, -, +
        direction = 1 if trial % 2 == 0 else -1
        lift_target = center + (direction * lift_ticks)
        lift_target = max(0, min(4095, lift_target))

        dir_label = "+" if direction > 0 else "-"
        print(f"  Trial {trial+1}/{NUM_TRIALS}: lifting {dir_label}{TRIAL_LIFT_DEG} deg to {lift_target}...")

        # Move to lifted position
        move_to(pkt, port, sid, lift_target)

        # Release and let gravity pull to vertical
        release_servo(pkt, port, sid)
        print(f"    Released. Waiting {SETTLE_TIME}s for settle...", end="", flush=True)
        time.sleep(SETTLE_TIME)
        print(" reading...", end="", flush=True)

        # Read settled position
        settled = read_settled_position(pkt, port, sid)
        if settled is None:
            print(" READ ERROR")
            continue

        all_readings.append(settled)
        delta = settled - center
        deg = delta * 360.0 / ENCODER_RESOLUTION
        print(f" pos={settled} (delta={delta:+d}, {deg:+.1f} deg)")

    if not all_readings:
        print(f"  [ERROR] No valid readings for servo {sid}!")
        return center

    # Average all trials (circular mean handles wrap near 0/4095)
    avg_pos = circular_avg(all_readings)
    spread = max(all_readings) - min(all_readings)
    spread_deg = spread * 360.0 / ENCODER_RESOLUTION

    delta = avg_pos - center
    delta_deg = delta * 360.0 / ENCODER_RESOLUTION

    print(f"\n  Result: {avg_pos} (delta={delta:+d}, {delta_deg:+.1f} deg)")
    print(f"  Spread: {spread} ticks ({spread_deg:.1f} deg)")

    if spread > 50:
        print(f"  WARNING: High spread ({spread} ticks) -- leg may be swinging or obstructed")

    return avg_pos


def run_auto_calibration():
    """Main calibration routine."""
    port = PortHandler(PORT_NAME)
    pkt = PacketHandler(SERVO_PROTOCOL)

    if not port.openPort():
        print("[ERROR] Failed to open port. Is /dev/ttyUSB1 connected?")
        sys.exit(1)
    if not port.setBaudRate(BAUDRATE):
        print("[ERROR] Failed to set baud rate.")
        sys.exit(1)

    print("\n" + "=" * 60)
    print("  HEXAPOD AUTO-CALIBRATION (Gravity Drop)")
    print("=" * 60)
    print()
    print("METHOD: Lift each leg, release, let gravity find vertical.")
    print()
    print("PREREQUISITES:")
    print("  1. Robot must be ELEVATED (legs hanging freely)")
    print("  2. No obstructions around legs")
    print("  3. Power supply connected and stable")
    print()
    print("  Starting in 5 seconds... (Ctrl+C to abort)")

    try:
        time.sleep(5)
    except KeyboardInterrupt:
        print("\nAborted.")
        port.closePort()
        sys.exit(0)

    try:
        # Verify all servos respond
        print("\nPinging servos...")
        for sid in ALL_SERVOS:
            pos = read_position(pkt, port, sid)
            if pos is None:
                print(f"  [ERROR] Servo {sid} ({SERVO_LABELS[sid]}) not responding!")
                print("  Check wiring and power. Aborting.")
                sys.exit(1)
            print(f"  Servo {sid} ({SERVO_LABELS[sid]}): OK (pos={pos})")

        new_home = {}

        for sid in ALL_SERVOS:
            best_pos = calibrate_servo(pkt, port, sid)
            new_home[sid] = best_pos

        # ---------- Symmetry report (informational only, no correction) ----------
        # Encoder zero varies per servo -- L+R does NOT need to sum to 4096.
        # Gravity readings are ground truth. Report deltas for sanity check only.
        print("\n" + "=" * 60)
        print("  SYMMETRY REPORT (informational -- no correction applied)")
        print("=" * 60)
        for left_id, right_id, name in PAIRS:
            old_sum = HOME_POSITIONS[left_id] + HOME_POSITIONS[right_id]
            new_sum = new_home[left_id] + new_home[right_id]
            delta_l = new_home[left_id] - HOME_POSITIONS[left_id]
            delta_r = new_home[right_id] - HOME_POSITIONS[right_id]
            deg_l = delta_l * 360.0 / ENCODER_RESOLUTION
            deg_r = delta_r * 360.0 / ENCODER_RESOLUTION
            print(f"  {name:7s}  S{left_id}: {delta_l:+d} ({deg_l:+.1f} deg)  "
                  f"S{right_id}: {delta_r:+d} ({deg_r:+.1f} deg)")
            if abs(abs(deg_l) - abs(deg_r)) > 5.0:
                print(f"           NOTE: asymmetric drift > 5 deg -- inspect mounting")

        # ---------- Final summary ----------
        print("\n" + "=" * 60)
        print("  CALIBRATION COMPLETE (gravity-drop, no artificial correction)")
        print("=" * 60)

        print("\nComparison (old vs new):")
        print(f"  {'Servo':<8} {'Label':<15} {'Old':>6} {'New':>6} {'Delta':>7} {'Degrees':>8}")
        print(f"  {'-'*52}")
        for sid in ALL_SERVOS:
            old = HOME_POSITIONS[sid]
            new = new_home[sid]
            delta = new - old
            deg = (delta / ENCODER_RESOLUTION) * 360.0
            print(f"  {sid:<8} {SERVO_LABELS[sid]:<15} {old:>6} {new:>6} {delta:>+7} {deg:>+7.1f} deg")

        print("\n" + "-" * 60)
        print("Copy this block into final_full_gait_test.py (lines ~171-174):")
        print("-" * 60)
        print("HOME_POSITIONS = {")
        row1 = [f"{sid}: {new_home[sid]}" for sid in [1, 2, 3]]
        row2 = [f"{sid}: {new_home[sid]}" for sid in [4, 5, 6]]
        print(f"    {', '.join(row1)},")
        print(f"    {', '.join(row2)},")
        print("}")
        print("-" * 60)

        # Move all servos to new home
        print("\nMoving all servos to new HOME positions...")
        for sid in ALL_SERVOS:
            move_to(pkt, port, sid, new_home[sid])
        time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n\nAborted.")
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
    finally:
        # Always disable torque and close port
        print("\nDisabling all servos...")
        for sid in ALL_SERVOS:
            try:
                release_servo(pkt, port, sid)
            except Exception:
                pass
        port.closePort()
        print("Servos are limp. Port closed. Power off safely.")


if __name__ == "__main__":
    run_auto_calibration()
