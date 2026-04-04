#!/usr/bin/env python3
"""
auto_calibrate.py -- Hexapod HOME_POSITIONS Calibration Tool

Two calibration methods:

  interactive   Move each leg to HOME, then nudge with keyboard until
                visually straight down. Best accuracy -- you are the sensor.

  gravity       Lift-and-drop method. Works OK for rough alignment but
                limited by gear friction and curved-leg center of mass.
                Use interactive mode to refine after gravity.

Usage:
    sudo python3 auto_calibrate.py                  # interactive (default)
    sudo python3 auto_calibrate.py interactive       # same
    sudo python3 auto_calibrate.py gravity           # gravity-drop method
    sudo python3 auto_calibrate.py interactive 3     # calibrate only servo 3
    sudo python3 auto_calibrate.py interactive 1,4,5 # calibrate servos 1,4,5

Requires: scservo_sdk, /dev/ttyUSB1, robot on flat surface (interactive)
          or elevated with legs free (gravity).
"""

import math
import os
import sys
import time

try:
    from scservo_sdk import PortHandler, PacketHandler
except ImportError:
    print("[!] scservo_sdk not found. Install it or run on the Pi.")
    sys.exit(1)

# Try to import terminal raw input (Linux/Pi)
try:
    import termios
    import tty
    HAS_TERMIOS = True
except ImportError:
    HAS_TERMIOS = False

# ---------- Hardware config (must match final_full_gait_test.py) ----------
PORT_NAME      = "/dev/ttyUSB1"
BAUDRATE       = 1000000
SERVO_PROTOCOL = 0

ADDR_MODE             = 33
ADDR_TORQUE_ENABLE    = 40
ADDR_ACCEL            = 41
ADDR_GOAL_POSITION    = 42
ADDR_GOAL_SPEED       = 46
ADDR_TORQUE_LIMIT     = 16
ADDR_PRESENT_POSITION = 56

ENCODER_RESOLUTION = 4096
DEGREES_PER_TICK   = 360.0 / ENCODER_RESOLUTION

ALL_SERVOS = [1, 2, 3, 4, 5, 6]

SERVO_LABELS = {
    1: "Right Front",
    2: "Left Front",
    3: "Left Middle",
    4: "Left Rear",
    5: "Right Rear",
    6: "Right Middle",
}

DIRECTION_MAP = {
    1: 1, 2: -1, 3: -1,
    4: -1, 5: 1, 6: 1,
}

PAIRS = [
    (2, 1, "Front"),
    (3, 6, "Middle"),
    (4, 5, "Rear"),
]

# ---------- Read current HOME_POSITIONS from gait file ----------
GAIT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "final_full_gait_test.py")


def load_home_positions():
    """Parse HOME_POSITIONS from the gait engine file."""
    import re
    if not os.path.exists(GAIT_FILE):
        print(f"[!] Gait file not found: {GAIT_FILE}")
        return {1: 2048, 2: 2048, 3: 2048, 4: 2048, 5: 2048, 6: 2048}
    with open(GAIT_FILE, "r") as f:
        content = f.read()
    match = re.search(r'HOME_POSITIONS\s*=\s*\{([^}]+)\}', content)
    if not match:
        print("[!] Could not parse HOME_POSITIONS from gait file")
        return {1: 2048, 2: 2048, 3: 2048, 4: 2048, 5: 2048, 6: 2048}
    positions = {}
    for pair in re.finditer(r'(\d+)\s*:\s*(\d+)', match.group(1)):
        positions[int(pair.group(1))] = int(pair.group(2))
    return positions


HOME_POSITIONS = load_home_positions()

# ---------- Calibration parameters ----------
MOVE_SPEED     = 200
TORQUE_LIMIT   = 400
ACCEL_PROFILE  = 20

# Interactive mode
NUDGE_SMALL    = 5     # ticks per small step (~0.44 deg)
NUDGE_BIG      = 25    # ticks per big step (~2.2 deg)
NUDGE_HUGE     = 100   # ticks per huge step (~8.8 deg)

# Gravity mode
SETTLE_TIME    = 3.0
NUM_SAMPLES    = 5
SAMPLE_GAP     = 0.1
NUM_TRIALS     = 4
TRIAL_LIFT_DEG = 30


# ===================================================================
# Low-level servo helpers
# ===================================================================

def read_position(pkt, port, sid):
    """Read encoder position (0-4095). Returns int or None."""
    pos, result, error = pkt.read2ByteTxRx(port, sid, ADDR_PRESENT_POSITION)
    return pos if result == 0 else None


def move_to(pkt, port, sid, target, timeout=5.0):
    """Move servo to target in position mode and wait for arrival."""
    target = max(0, min(4095, target))
    pkt.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, 0)
    pkt.write1ByteTxRx(port, sid, ADDR_MODE, 0)
    pkt.write1ByteTxRx(port, sid, ADDR_ACCEL, ACCEL_PROFILE)
    pkt.write2ByteTxRx(port, sid, ADDR_TORQUE_LIMIT, TORQUE_LIMIT)
    pkt.write2ByteTxRx(port, sid, ADDR_GOAL_SPEED, MOVE_SPEED)
    pkt.write2ByteTxRx(port, sid, ADDR_GOAL_POSITION, target)
    pkt.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, 1)
    deadline = time.time() + timeout
    while time.time() < deadline:
        time.sleep(0.05)
        pos = read_position(pkt, port, sid)
        if pos is not None and abs(pos - target) <= 5:
            break
    time.sleep(0.2)


def release_servo(pkt, port, sid):
    """Disable torque (leg goes limp)."""
    pkt.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, 0)


def hold_position(pkt, port, sid, target):
    """Move to target and keep holding (torque stays on)."""
    move_to(pkt, port, sid, target)


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


def read_averaged(pkt, port, sid, n=5, gap=0.1):
    """Read position n times and return circular average."""
    readings = []
    for _ in range(n):
        pos = read_position(pkt, port, sid)
        if pos is not None:
            readings.append(pos)
        time.sleep(gap)
    return circular_avg(readings) if readings else None


# ===================================================================
# Terminal input helpers
# ===================================================================

def get_single_key():
    """Read a single keypress without requiring Enter (Linux/Pi).
    Falls back to input() on Windows."""
    if HAS_TERMIOS:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
            # Handle escape sequences (arrow keys)
            if ch == '\x1b':
                ch2 = sys.stdin.read(1)
                if ch2 == '[':
                    ch3 = sys.stdin.read(1)
                    if ch3 == 'C':    # right arrow
                        return 'right'
                    elif ch3 == 'D':  # left arrow
                        return 'left'
                    elif ch3 == 'A':  # up arrow
                        return 'up'
                    elif ch3 == 'B':  # down arrow
                        return 'down'
                return 'esc'
            return ch
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
    else:
        # Fallback: require Enter after input
        return input().strip()


def print_interactive_help():
    """Print key bindings for interactive mode."""
    print()
    print("  CONTROLS:")
    print("  -----------------------------------------------")
    print("  d / right arrow  = nudge clockwise (small, ~0.4 deg)")
    print("  a / left arrow   = nudge counter-clockwise (small)")
    print("  D (shift+d)      = nudge clockwise (big, ~2.2 deg)")
    print("  A (shift+a)      = nudge counter-clockwise (big)")
    print("  w / up arrow     = nudge clockwise (huge, ~8.8 deg)")
    print("  s / down arrow   = nudge counter-clockwise (huge)")
    print("  r                = read current position (no move)")
    print("  h                = return to original HOME")
    print("  z                = release torque (leg goes limp)")
    print("  t                = re-enable torque at current pos")
    print("  Enter / space    = ACCEPT this position, next servo")
    print("  q                = quit (skip remaining servos)")
    print("  ?                = show this help")
    print("  -----------------------------------------------")


# ===================================================================
# INTERACTIVE MODE
# ===================================================================

def interactive_calibrate_servo(pkt, port, sid, original_home):
    """
    Interactive calibration for one servo.
    Moves to HOME, then lets user nudge until satisfied.
    Returns the accepted position.
    """
    label = SERVO_LABELS.get(sid, f"Servo {sid}")
    current_target = original_home

    print(f"\n{'='*56}")
    print(f"  Servo {sid} -- {label}")
    print(f"  Current HOME: {original_home} "
          f"(side: {'LEFT' if DIRECTION_MAP[sid] == -1 else 'RIGHT'})")
    print(f"{'='*56}")

    print(f"  Moving to HOME position {original_home}...")
    move_to(pkt, port, sid, current_target)

    print(f"  Look at the leg. Is it pointing straight down?")
    print_interactive_help()

    while True:
        # Show current state
        actual = read_position(pkt, port, sid)
        delta = current_target - original_home
        deg = delta * DEGREES_PER_TICK
        actual_str = str(actual) if actual is not None else "?"

        sys.stdout.write(
            f"\r  [{sid}] target={current_target} actual={actual_str} "
            f"delta={delta:+d} ({deg:+.1f} deg)  > "
        )
        sys.stdout.flush()

        key = get_single_key()

        if key in ('d', 'right'):
            current_target = min(4095, current_target + NUDGE_SMALL)
            hold_position(pkt, port, sid, current_target)
        elif key in ('a', 'left'):
            current_target = max(0, current_target - NUDGE_SMALL)
            hold_position(pkt, port, sid, current_target)
        elif key == 'D':
            current_target = min(4095, current_target + NUDGE_BIG)
            hold_position(pkt, port, sid, current_target)
        elif key == 'A':
            current_target = max(0, current_target - NUDGE_BIG)
            hold_position(pkt, port, sid, current_target)
        elif key in ('w', 'up'):
            current_target = min(4095, current_target + NUDGE_HUGE)
            hold_position(pkt, port, sid, current_target)
        elif key in ('s', 'down'):
            current_target = max(0, current_target - NUDGE_HUGE)
            hold_position(pkt, port, sid, current_target)
        elif key == 'r':
            pos = read_position(pkt, port, sid)
            print(f"\n  Raw read: {pos}")
        elif key == 'h':
            current_target = original_home
            move_to(pkt, port, sid, current_target)
            print(f"\n  Reset to original HOME: {original_home}")
        elif key == 'z':
            release_servo(pkt, port, sid)
            print(f"\n  Torque OFF -- leg is limp. Press 't' to re-engage.")
        elif key == 't':
            hold_position(pkt, port, sid, current_target)
            print(f"\n  Torque ON at {current_target}")
        elif key in ('\r', '\n', ' '):
            # Accept
            actual = read_averaged(pkt, port, sid)
            if actual is not None:
                current_target = actual
            delta = current_target - original_home
            deg = delta * DEGREES_PER_TICK
            print(f"\n  ACCEPTED: {current_target} "
                  f"(delta={delta:+d}, {deg:+.1f} deg from old HOME)")
            return current_target
        elif key == 'q':
            print(f"\n  Skipped.")
            return None  # signal to quit
        elif key == '?':
            print_interactive_help()
        else:
            pass  # ignore unknown keys

    return current_target


def run_interactive(servo_list=None):
    """Interactive calibration for selected servos."""
    if servo_list is None:
        servo_list = ALL_SERVOS

    port = PortHandler(PORT_NAME)
    pkt = PacketHandler(SERVO_PROTOCOL)

    if not port.openPort():
        print(f"[!] Cannot open {PORT_NAME}. Is USB connected?")
        sys.exit(1)
    if not port.setBaudRate(BAUDRATE):
        print("[!] Cannot set baud rate.")
        sys.exit(1)

    print()
    print("=" * 56)
    print("  INTERACTIVE CALIBRATION")
    print("=" * 56)
    print()
    print("  Robot should be on a flat surface, powered on.")
    print("  You will visually align each leg to point straight down.")
    print(f"  Servos to calibrate: {servo_list}")
    print()
    print(f"  HOME_POSITIONS loaded from gait file:")
    for sid in servo_list:
        print(f"    Servo {sid} ({SERVO_LABELS[sid]}): {HOME_POSITIONS[sid]}")

    # Verify all servos respond
    print("\n  Checking servo communication...")
    for sid in servo_list:
        pos = read_position(pkt, port, sid)
        if pos is None:
            print(f"  [!] Servo {sid} ({SERVO_LABELS[sid]}) not responding!")
            print("      Check wiring. Aborting.")
            port.closePort()
            sys.exit(1)
        print(f"    Servo {sid}: OK (current pos={pos})")

    print("\n  Starting in 3 seconds... (Ctrl+C to abort)")
    try:
        time.sleep(3)
    except KeyboardInterrupt:
        print("\n  Aborted.")
        port.closePort()
        sys.exit(0)

    new_home = dict(HOME_POSITIONS)  # start with current values
    calibrated = []

    try:
        for sid in servo_list:
            result = interactive_calibrate_servo(pkt, port, sid,
                                                 HOME_POSITIONS[sid])
            if result is None:
                # User pressed 'q' to quit
                break
            new_home[sid] = result
            calibrated.append(sid)

    except KeyboardInterrupt:
        print("\n\n  Interrupted.")

    # Show results
    if calibrated:
        print_results(new_home, calibrated)
        # Hold all calibrated servos at new positions
        print("\n  Holding legs at new positions. Press Ctrl+C to release.")
        try:
            for sid in calibrated:
                hold_position(pkt, port, sid, new_home[sid])
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass

    # Cleanup
    print("\n  Releasing all servos...")
    for sid in ALL_SERVOS:
        try:
            release_servo(pkt, port, sid)
        except Exception:
            pass
    port.closePort()
    print("  Done.")


# ===================================================================
# GRAVITY DROP MODE (improved)
# ===================================================================

def gravity_calibrate_servo(pkt, port, sid):
    """Gravity-drop calibration for one servo with stability check."""
    center = HOME_POSITIONS[sid]
    lift_ticks = int(TRIAL_LIFT_DEG / 360.0 * ENCODER_RESOLUTION)
    label = SERVO_LABELS.get(sid, f"Servo {sid}")

    print(f"\n  {'='*50}")
    print(f"  Servo {sid} ({label}) -- Current HOME: {center}")
    print(f"  {'='*50}")

    all_readings = []

    for trial in range(NUM_TRIALS):
        direction = 1 if trial % 2 == 0 else -1
        lift_target = max(0, min(4095, center + direction * lift_ticks))
        dir_label = "+" if direction > 0 else "-"

        print(f"  Trial {trial+1}/{NUM_TRIALS}: "
              f"lift {dir_label}{TRIAL_LIFT_DEG} deg to {lift_target}...",
              end="", flush=True)

        move_to(pkt, port, sid, lift_target)
        release_servo(pkt, port, sid)
        print(f" released...", end="", flush=True)

        # Wait with stability check: read every 0.5s, stop early if stable
        prev = None
        stable_count = 0
        for _ in range(int(SETTLE_TIME * 2)):
            time.sleep(0.5)
            pos = read_position(pkt, port, sid)
            if pos is not None and prev is not None:
                if abs(pos - prev) <= 2:
                    stable_count += 1
                    if stable_count >= 3:
                        break
                else:
                    stable_count = 0
            prev = pos

        settled = read_averaged(pkt, port, sid, NUM_SAMPLES, SAMPLE_GAP)
        if settled is None:
            print(" READ ERROR")
            continue

        all_readings.append(settled)
        delta = settled - center
        deg = delta * DEGREES_PER_TICK
        print(f" settled={settled} ({delta:+d}, {deg:+.1f} deg)")

    if not all_readings:
        print(f"  [!] No valid readings for servo {sid}")
        return center

    avg_pos = circular_avg(all_readings)
    spread = max(all_readings) - min(all_readings)
    spread_deg = spread * DEGREES_PER_TICK
    delta = avg_pos - center
    delta_deg = delta * DEGREES_PER_TICK

    print(f"\n  Result: {avg_pos} ({delta:+d}, {delta_deg:+.1f} deg)")
    print(f"  Spread: {spread} ticks ({spread_deg:.1f} deg)")

    if spread > 50:
        print(f"  [!] High spread -- leg may be swinging or obstructed")
        print(f"      Consider using 'interactive' mode for this servo")

    return avg_pos


def run_gravity(servo_list=None):
    """Gravity-drop calibration for selected servos."""
    if servo_list is None:
        servo_list = ALL_SERVOS

    port = PortHandler(PORT_NAME)
    pkt = PacketHandler(SERVO_PROTOCOL)

    if not port.openPort():
        print(f"[!] Cannot open {PORT_NAME}")
        sys.exit(1)
    if not port.setBaudRate(BAUDRATE):
        print("[!] Cannot set baud rate")
        sys.exit(1)

    print()
    print("=" * 56)
    print("  GRAVITY-DROP CALIBRATION")
    print("=" * 56)
    print()
    print("  Robot must be ELEVATED (legs hanging freely).")
    print("  No obstructions around legs.")
    print()
    print("  NOTE: Curved legs + gear friction can bias results.")
    print("  Use 'interactive' mode after this for fine-tuning.")
    print()
    print(f"  Servos: {servo_list}")
    print()

    # Verify servos
    print("  Checking servos...")
    for sid in servo_list:
        pos = read_position(pkt, port, sid)
        if pos is None:
            print(f"  [!] Servo {sid} not responding. Aborting.")
            port.closePort()
            sys.exit(1)
        print(f"    Servo {sid} ({SERVO_LABELS[sid]}): OK (pos={pos})")

    print("\n  Starting in 5 seconds... (Ctrl+C to abort)")
    try:
        time.sleep(5)
    except KeyboardInterrupt:
        print("\n  Aborted.")
        port.closePort()
        sys.exit(0)

    new_home = dict(HOME_POSITIONS)
    calibrated = []

    try:
        for sid in servo_list:
            result = gravity_calibrate_servo(pkt, port, sid)
            new_home[sid] = result
            calibrated.append(sid)
    except KeyboardInterrupt:
        print("\n\n  Interrupted.")

    if calibrated:
        print_results(new_home, calibrated)

        # Hold at new positions
        print("\n  Moving to calibrated positions and holding...")
        for sid in calibrated:
            move_to(pkt, port, sid, new_home[sid])
        print("  Press Ctrl+C to release.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass

    print("\n  Releasing all servos...")
    for sid in ALL_SERVOS:
        try:
            release_servo(pkt, port, sid)
        except Exception:
            pass
    port.closePort()
    print("  Done.")


# ===================================================================
# Shared output
# ===================================================================

def print_results(new_home, calibrated):
    """Print calibration results, comparison, and copy-paste block."""

    # Comparison table
    print()
    print("=" * 60)
    print("  CALIBRATION RESULTS")
    print("=" * 60)
    print(f"  {'Servo':<8} {'Label':<15} {'Old':>6} {'New':>6} "
          f"{'Delta':>7} {'Degrees':>8}")
    print(f"  {'-'*54}")

    for sid in ALL_SERVOS:
        old = HOME_POSITIONS[sid]
        new = new_home[sid]
        delta = new - old
        deg = delta * DEGREES_PER_TICK
        marker = " *" if sid in calibrated else ""
        print(f"  {sid:<8} {SERVO_LABELS[sid]:<15} {old:>6} {new:>6} "
              f"{delta:>+7} {deg:>+7.1f} deg{marker}")

    # Symmetry report
    print()
    print("=" * 60)
    print("  SYMMETRY CHECK (L vs R same pair)")
    print("=" * 60)
    for left_id, right_id, name in PAIRS:
        delta_l = new_home[left_id] - HOME_POSITIONS[left_id]
        delta_r = new_home[right_id] - HOME_POSITIONS[right_id]
        deg_l = delta_l * DEGREES_PER_TICK
        deg_r = delta_r * DEGREES_PER_TICK
        diff = abs(abs(deg_l) - abs(deg_r))
        flag = " <-- check!" if diff > 5.0 else ""
        print(f"  {name:7s}  S{left_id}: {deg_l:+.1f} deg  "
              f"S{right_id}: {deg_r:+.1f} deg  (asymmetry: {diff:.1f} deg){flag}")

    # Copy-paste block
    print()
    print("=" * 60)
    print("  COPY INTO final_full_gait_test.py (lines ~180-183):")
    print("=" * 60)
    print("HOME_POSITIONS = {")
    row1 = [f"{sid}: {new_home[sid]}" for sid in [1, 2, 3]]
    row2 = [f"{sid}: {new_home[sid]}" for sid in [4, 5, 6]]
    print(f"    {', '.join(row1)},")
    print(f"    {', '.join(row2)},")
    print("}")
    print("=" * 60)

    # Also print single-line for quick reference
    one_liner = ", ".join(f"{sid}: {new_home[sid]}" for sid in ALL_SERVOS)
    print(f"\n  One-liner: {{{one_liner}}}")


# ===================================================================
# Main
# ===================================================================

def parse_servo_list(arg):
    """Parse '1,4,5' or '3' into list of servo IDs."""
    try:
        ids = [int(x.strip()) for x in arg.split(",")]
        for sid in ids:
            if sid not in ALL_SERVOS:
                print(f"[!] Invalid servo ID: {sid}. Valid: {ALL_SERVOS}")
                sys.exit(1)
        return ids
    except ValueError:
        print(f"[!] Cannot parse servo list: {arg}")
        sys.exit(1)


def print_usage():
    print("Hexapod Auto-Calibration Tool")
    print("-" * 40)
    print()
    print("Usage: sudo python3 auto_calibrate.py [mode] [servos]")
    print()
    print("Modes:")
    print("  interactive  Nudge each leg with keyboard until aligned (default)")
    print("  gravity      Lift-and-drop, let gravity find vertical")
    print()
    print("Servos (optional):")
    print("  3       calibrate only servo 3")
    print("  1,4,5   calibrate servos 1, 4, and 5")
    print("  (omit)  calibrate all 6 servos")
    print()
    print("Controls (interactive mode):")
    print("  a/d or arrows = small nudge (~0.4 deg)")
    print("  A/D           = medium nudge (~2.2 deg)")
    print("  w/s or up/dn  = large nudge (~8.8 deg)")
    print("  z/t           = release/re-enable torque")
    print("  h             = reset to original HOME")
    print("  Enter/space   = accept, next servo")
    print("  q             = quit")
    print()
    print("Examples:")
    print("  sudo python3 auto_calibrate.py")
    print("  sudo python3 auto_calibrate.py interactive 2,3")
    print("  sudo python3 auto_calibrate.py gravity")


if __name__ == "__main__":
    args = sys.argv[1:]

    if not args:
        # Default: interactive, all servos
        run_interactive()
        sys.exit(0)

    if args[0] in ("-h", "--help", "help"):
        print_usage()
        sys.exit(0)

    mode = args[0].lower()
    servo_list = None
    if len(args) > 1:
        servo_list = parse_servo_list(args[1])

    if mode == "interactive":
        run_interactive(servo_list)
    elif mode == "gravity":
        run_gravity(servo_list)
    else:
        # Maybe they passed a servo list without mode
        try:
            servo_list = parse_servo_list(args[0])
            run_interactive(servo_list)
        except SystemExit:
            print(f"[!] Unknown mode: {args[0]}")
            print_usage()
            sys.exit(1)
