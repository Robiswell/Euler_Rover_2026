#!/usr/bin/env python3
"""
Hexapod Servo Calibration Tool
-------------------------------
Three modes:
  check     - Read current positions, compare to HOME_POSITIONS in gait engine
  calibrate - Disable torque, wait for manual positioning, average N readings
  analyze   - Detailed deviation report in ticks and degrees

Usage:
  sudo python3 calibrate_legs.py check
  sudo python3 calibrate_legs.py calibrate
  sudo python3 calibrate_legs.py analyze
  sudo python3 calibrate_legs.py check --samples 20
"""
import sys
import time
import statistics
import re
import os

# ---------------------------------------------------------------------------
# Hardware constants (must match final_full_gait_test.py)
# ---------------------------------------------------------------------------
PORT_NAME      = "/dev/ttyUSB1"
BAUDRATE       = 1000000
SERVO_PROTOCOL = 0

ADDR_TORQUE           = 40
ADDR_PRESENT_POSITION = 56

ENCODER_RESOLUTION = 4096.0
DEGREES_PER_TICK   = 360.0 / ENCODER_RESOLUTION  # ~0.0879 deg/tick

ALL_SERVOS = [1, 2, 3, 4, 5, 6]

SERVO_NAMES = {
    1: "Right Front",
    2: "Left Front",
    3: "Left Middle",
    4: "Left Rear",
    5: "Right Rear",
    6: "Right Middle",
}

SERVO_SIDES = {
    1: "RIGHT", 2: "LEFT", 3: "LEFT",
    4: "LEFT",  5: "RIGHT", 6: "RIGHT",
}

# ---------------------------------------------------------------------------
# Path to gait engine (auto-detected relative to this script)
# ---------------------------------------------------------------------------
GAIT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "final_full_gait_test.py")


def read_home_positions_from_gait():
    """Parse HOME_POSITIONS dict from the gait engine file."""
    if not os.path.exists(GAIT_FILE):
        print(f"[!] Gait file not found: {GAIT_FILE}")
        return None

    with open(GAIT_FILE, "r") as f:
        content = f.read()

    # Match the HOME_POSITIONS = { ... } block
    match = re.search(
        r'HOME_POSITIONS\s*=\s*\{([^}]+)\}', content
    )
    if not match:
        print("[!] Could not find HOME_POSITIONS in gait file")
        return None

    block = match.group(1)
    positions = {}
    for pair in re.finditer(r'(\d+)\s*:\s*(\d+)', block):
        positions[int(pair.group(1))] = int(pair.group(2))

    return positions


def open_port():
    """Open serial port and return (port_handler, packet_handler) or None."""
    try:
        from scservo_sdk import PortHandler, PacketHandler
    except ImportError:
        print("[!] scservo_sdk not found. Install it or run on the Pi.")
        return None, None

    port_handler = PortHandler(PORT_NAME)
    packet_handler = PacketHandler(SERVO_PROTOCOL)

    if not port_handler.openPort():
        print(f"[!] Failed to open port {PORT_NAME}")
        return None, None
    if not port_handler.setBaudRate(BAUDRATE):
        print(f"[!] Failed to set baud rate {BAUDRATE}")
        port_handler.closePort()
        return None, None

    return port_handler, packet_handler


def read_position(packet_handler, port_handler, sid):
    """Read current encoder position for one servo. Returns int or None."""
    pos, result, error = packet_handler.read2ByteTxRx(
        port_handler, sid, ADDR_PRESENT_POSITION
    )
    if result != 0:
        return None
    return pos


def read_all_positions(packet_handler, port_handler, num_samples=10,
                       delay=0.05):
    """
    Read positions for all servos, averaging over num_samples readings.
    Returns dict {sid: avg_position} and {sid: stdev}.
    """
    readings = {sid: [] for sid in ALL_SERVOS}

    for i in range(num_samples):
        for sid in ALL_SERVOS:
            pos = read_position(packet_handler, port_handler, sid)
            if pos is not None:
                readings[sid].append(pos)
        if i < num_samples - 1:
            time.sleep(delay)

    averages = {}
    stdevs = {}
    for sid in ALL_SERVOS:
        if len(readings[sid]) >= 2:
            averages[sid] = round(statistics.mean(readings[sid]))
            stdevs[sid] = round(statistics.stdev(readings[sid]), 1)
        elif len(readings[sid]) == 1:
            averages[sid] = readings[sid][0]
            stdevs[sid] = 0.0
        else:
            averages[sid] = None
            stdevs[sid] = None

    return averages, stdevs


def disable_torque(packet_handler, port_handler):
    """Disable torque on all servos (legs go limp)."""
    for sid in ALL_SERVOS:
        packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE, 0)


def ticks_to_degrees(ticks):
    """Convert encoder ticks to degrees."""
    return ticks * DEGREES_PER_TICK


def print_separator(char="-", width=72):
    print(char * width)


# ===========================================================================
# MODE: check
# ===========================================================================
def mode_check(num_samples=10):
    """Read current positions and compare to HOME_POSITIONS from gait file."""
    home = read_home_positions_from_gait()
    if home is None:
        print("[!] Cannot compare without HOME_POSITIONS. Falling back to raw read.")
        home = {}

    port_handler, packet_handler = open_port()
    if port_handler is None:
        return

    print(f"\nReading positions ({num_samples} samples per servo)...")
    positions, stdevs = read_all_positions(packet_handler, port_handler,
                                           num_samples)

    print_separator("=")
    print("POSITION CHECK -- Current vs HOME_POSITIONS")
    print_separator("=")
    print(f"{'Servo':<8} {'Name':<14} {'Current':>8} {'Home':>8} "
          f"{'Diff':>8} {'Degrees':>8} {'StdDev':>7} {'Status'}")
    print_separator()

    issues = []
    for sid in ALL_SERVOS:
        name = SERVO_NAMES.get(sid, f"Servo {sid}")
        cur = positions.get(sid)
        h = home.get(sid)
        sd = stdevs.get(sid, 0)

        if cur is None:
            print(f"  {sid:<6} {name:<14} {'ERROR':>8} {h or '?':>8} "
                  f"{'--':>8} {'--':>8} {'--':>7} COMM FAIL")
            issues.append(f"Servo {sid}: communication error")
            continue

        if h is not None:
            diff = cur - h
            deg = ticks_to_degrees(abs(diff))
            if abs(diff) > 100:
                status = "** BAD **"
                issues.append(f"Servo {sid} ({name}): {abs(diff)} ticks "
                              f"({deg:.1f} deg) off")
            elif abs(diff) > 30:
                status = "* DRIFT *"
                issues.append(f"Servo {sid} ({name}): minor drift "
                              f"{abs(diff)} ticks ({deg:.1f} deg)")
            else:
                status = "OK"
            print(f"  {sid:<6} {name:<14} {cur:>8} {h:>8} "
                  f"{diff:>+8} {deg:>7.1f}° {sd:>7} {status}")
        else:
            print(f"  {sid:<6} {name:<14} {cur:>8} {'N/A':>8} "
                  f"{'--':>8} {'--':>8} {sd:>7} NO HOME")

    print_separator("=")

    if issues:
        print("\nISSUES FOUND:")
        for issue in issues:
            print(f"  - {issue}")
        print(f"\nRun 'sudo python3 calibrate_legs.py analyze' for detailed report")
    else:
        print("\nAll servos within tolerance. HOME_POSITIONS look good.")

    if home:
        print(f"\nHOME_POSITIONS from gait file:")
        print(f"  {home}")

    port_handler.closePort()


# ===========================================================================
# MODE: calibrate
# ===========================================================================
def mode_calibrate(num_samples=20):
    """Full calibration: disable torque, wait, read averaged positions."""
    port_handler, packet_handler = open_port()
    if port_handler is None:
        return

    home = read_home_positions_from_gait()

    print_separator("=")
    print("CALIBRATION MODE")
    print_separator("=")
    print("\nStep 1: Disabling torque on all servos (legs go limp)...")
    disable_torque(packet_handler, port_handler)

    print("\nStep 2: Position the robot")
    print("  - Place robot on a flat surface")
    print("  - Push ALL legs straight down, pointing at the ground")
    print("  - Each leg should be perpendicular to the body")
    print("  - Hold steady until readings begin")
    print()

    # Countdown
    wait_time = 20
    print(f"You have {wait_time} seconds to position legs.")
    print("Press Ctrl+C to skip countdown if already positioned.\n")
    try:
        for i in range(wait_time, 0, -1):
            print(f"  Reading in {i}s...", end="\r")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n  Countdown skipped.                ")

    print(f"\nStep 3: Reading positions ({num_samples} samples, averaging)...")

    # Take 3 rounds to check stability
    print("  Round 1/3...")
    pos1, _ = read_all_positions(packet_handler, port_handler, num_samples)
    time.sleep(0.5)
    print("  Round 2/3...")
    pos2, _ = read_all_positions(packet_handler, port_handler, num_samples)
    time.sleep(0.5)
    print("  Round 3/3...")
    pos3, _ = read_all_positions(packet_handler, port_handler, num_samples)

    # Average across rounds and check stability
    final_positions = {}
    stability = {}
    for sid in ALL_SERVOS:
        vals = [v for v in [pos1.get(sid), pos2.get(sid), pos3.get(sid)]
                if v is not None]
        if vals:
            final_positions[sid] = round(statistics.mean(vals))
            stability[sid] = max(vals) - min(vals)
        else:
            final_positions[sid] = None
            stability[sid] = None

    print_separator("=")
    print("CALIBRATION RESULTS")
    print_separator("=")
    print(f"{'Servo':<8} {'Name':<14} {'New Home':>9} {'Old Home':>9} "
          f"{'Change':>8} {'Stability':>10}")
    print_separator()

    unstable = []
    for sid in ALL_SERVOS:
        name = SERVO_NAMES.get(sid, f"Servo {sid}")
        new = final_positions.get(sid)
        old = home.get(sid) if home else None
        stab = stability.get(sid)

        if new is None:
            print(f"  {sid:<6} {name:<14} {'ERROR':>9}")
            continue

        stab_str = f"{stab} ticks" if stab is not None else "?"
        if stab is not None and stab > 10:
            stab_str += " !"
            unstable.append(sid)

        if old is not None:
            change = new - old
            print(f"  {sid:<6} {name:<14} {new:>9} {old:>9} "
                  f"{change:>+8} {stab_str:>10}")
        else:
            print(f"  {sid:<6} {name:<14} {new:>9} {'N/A':>9} "
                  f"{'--':>8} {stab_str:>10}")

    if unstable:
        print(f"\n[!] WARNING: Servos {unstable} showed instability "
              f"(>10 tick spread between rounds).")
        print("    The robot may have been moving. Consider re-running.")

    # Output copy-paste block
    print_separator("=")
    print("COPY THIS INTO final_full_gait_test.py:")
    print_separator("=")
    valid = {sid: pos for sid, pos in final_positions.items() if pos is not None}
    print("\nHOME_POSITIONS = {")
    items = sorted(valid.items())
    # Print in pairs for readability (matching gait file style)
    for i in range(0, len(items), 3):
        chunk = items[i:i+3]
        parts = [f"{sid}: {pos}" for sid, pos in chunk]
        line = ", ".join(parts) + ","
        print(f"    {line}")
    print("}")

    # Also output as a one-liner for quick copy
    one_liner = ", ".join(f"{sid}: {pos}" for sid, pos in sorted(valid.items()))
    print(f"\n# One-liner: {{{one_liner}}}")
    print_separator("=")

    port_handler.closePort()


# ===========================================================================
# MODE: analyze
# ===========================================================================
def mode_analyze(num_samples=20):
    """Detailed deviation analysis with degree conversions and recommendations."""
    home = read_home_positions_from_gait()
    if home is None:
        print("[!] Need HOME_POSITIONS in gait file for analysis.")
        return

    port_handler, packet_handler = open_port()
    if port_handler is None:
        return

    print(f"\nReading positions ({num_samples} samples per servo)...")
    positions, stdevs = read_all_positions(packet_handler, port_handler,
                                           num_samples)

    print_separator("=")
    print("DETAILED DEVIATION ANALYSIS")
    print_separator("=")

    total_error_deg = 0.0
    max_error_deg = 0.0
    max_error_servo = None
    servo_count = 0

    for sid in ALL_SERVOS:
        name = SERVO_NAMES.get(sid, f"Servo {sid}")
        side = SERVO_SIDES.get(sid, "?")
        cur = positions.get(sid)
        h = home.get(sid)
        sd = stdevs.get(sid, 0)

        print(f"\n  Servo {sid} -- {name} ({side})")
        print(f"  {'':4}Current position : {cur if cur else 'READ ERROR'}")
        print(f"  {'':4}Home position    : {h}")

        if cur is None or h is None:
            print(f"  {'':4}Status: CANNOT ANALYZE")
            continue

        diff = cur - h
        deg = ticks_to_degrees(diff)
        abs_deg = abs(deg)

        print(f"  {'':4}Deviation        : {diff:+d} ticks ({deg:+.2f} deg)")
        print(f"  {'':4}Read noise (1sd) : {sd} ticks ({ticks_to_degrees(sd):.2f} deg)")

        total_error_deg += abs_deg
        servo_count += 1
        if abs_deg > max_error_deg:
            max_error_deg = abs_deg
            max_error_servo = sid

        # Severity assessment
        if abs_deg < 3.0:
            print(f"  {'':4}Verdict          : GOOD (< 3 deg)")
        elif abs_deg < 8.0:
            print(f"  {'':4}Verdict          : ACCEPTABLE (< 8 deg)")
        elif abs_deg < 15.0:
            print(f"  {'':4}Verdict          : NEEDS ATTENTION (< 15 deg)")
            print(f"  {'':4}Recommendation   : Recalibrate this leg")
        else:
            print(f"  {'':4}Verdict          : CRITICAL (>= 15 deg)")
            print(f"  {'':4}Recommendation   : Recalibrate immediately -- "
                  f"gait will be unstable")

    # Summary
    print_separator("=")
    print("SUMMARY")
    print_separator()
    if servo_count > 0:
        avg_error = total_error_deg / servo_count
        print(f"  Average deviation : {avg_error:.2f} deg across {servo_count} servos")
        print(f"  Worst servo       : {max_error_servo} ({SERVO_NAMES.get(max_error_servo, '?')}) "
              f"at {max_error_deg:.2f} deg")

        if avg_error < 3.0:
            print(f"\n  OVERALL: GOOD -- calibration is solid")
        elif avg_error < 8.0:
            print(f"\n  OVERALL: ACCEPTABLE -- minor drift, monitor it")
        else:
            print(f"\n  OVERALL: RECALIBRATE -- run 'sudo python3 calibrate_legs.py calibrate'")

    print_separator("=")
    print(f"\nHOME_POSITIONS in gait file: {home}")

    port_handler.closePort()


# ===========================================================================
# Main
# ===========================================================================
def print_usage():
    print("Hexapod Servo Calibration Tool")
    print_separator()
    print("Usage: sudo python3 calibrate_legs.py <mode> [options]")
    print()
    print("Modes:")
    print("  check      Quick position check vs HOME_POSITIONS")
    print("  calibrate  Full recalibration (disables torque, waits for positioning)")
    print("  analyze    Detailed deviation report with degree analysis")
    print()
    print("Options:")
    print("  --samples N   Number of samples per reading (default: 10 for check, 20 for calibrate/analyze)")
    print()
    print("Examples:")
    print("  sudo python3 calibrate_legs.py check")
    print("  sudo python3 calibrate_legs.py calibrate")
    print("  sudo python3 calibrate_legs.py analyze --samples 30")


if __name__ == "__main__":
    args = sys.argv[1:]

    if not args or args[0] in ("-h", "--help", "help"):
        print_usage()
        sys.exit(0)

    mode = args[0].lower()

    # Parse --samples
    num_samples = None
    if "--samples" in args:
        idx = args.index("--samples")
        if idx + 1 < len(args):
            try:
                num_samples = int(args[idx + 1])
            except ValueError:
                print("[!] --samples requires an integer")
                sys.exit(1)

    if mode == "check":
        mode_check(num_samples or 10)
    elif mode == "calibrate":
        mode_calibrate(num_samples or 20)
    elif mode == "analyze":
        mode_analyze(num_samples or 20)
    else:
        print(f"[!] Unknown mode: {mode}")
        print_usage()
        sys.exit(1)
