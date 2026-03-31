#!/usr/bin/env python3
"""
HOME_POSITIONS Calibration Script
==================================
1. Disable torque on all 6 servos (legs go limp)
2. Manually position each leg pointing straight down
3. Press Enter to read all 6 encoder positions
4. Prints new HOME_POSITIONS dict ready to paste into gait file

Run on Pi:  sudo python3 calibrate_homes.py
"""

import sys
try:
    from scservo_sdk import PortHandler, PacketHandler
except ImportError:
    print("ERROR: scservo_sdk not installed. Run: pip install scservo_sdk")
    sys.exit(1)

# --- Config (must match gait file) ---
PORT_NAME  = "/dev/ttyUSB1"
BAUDRATE   = 1000000
PROTOCOL   = 0
SERVO_IDS  = [1, 2, 3, 4, 5, 6]

# STS3215 registers
ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_POSITION = 56

# Servo labels for reference
LABELS = {
    1: "Right Front",
    2: "Left Front",
    3: "Left Middle",
    4: "Left Rear",
    5: "Right Rear",
    6: "Right Middle",
}

# Mirror pairs for symmetry check
PAIRS = [(2, 1, "Front"), (3, 6, "Middle"), (4, 5, "Rear")]


def main():
    # Connect to servo bus
    port = PortHandler(PORT_NAME)
    if not port.openPort():
        print(f"ERROR: Cannot open {PORT_NAME}")
        print("Check: is the USB cable connected? Try /dev/ttyUSB0 if needed.")
        sys.exit(1)
    port.setBaudRate(BAUDRATE)
    pkt = PacketHandler(PROTOCOL)

    # Step 1: Disable torque on all servos so legs go limp
    print("\n=== HOME CALIBRATION ===\n")
    print("Step 1: Disabling torque on all servos...")
    for sid in SERVO_IDS:
        result, error = pkt.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, 0)
        if result != 0:
            print(f"  WARNING: Servo {sid} ({LABELS[sid]}) -- comm error: {pkt.getTxRxResult(result)}")
        else:
            print(f"  Servo {sid} ({LABELS[sid]}) -- torque OFF")

    # Step 2: User positions legs
    print("\nStep 2: Manually position ALL 6 legs pointing straight down.")
    print("         Use a flat surface or visual alignment.")
    print("         Each leg should hang perfectly vertical.\n")
    input("Press ENTER when all legs are positioned... ")

    # Step 3: Read all positions
    print("\nStep 3: Reading encoder positions...\n")
    positions = {}
    for sid in SERVO_IDS:
        pos, result, error = pkt.read2ByteTxRx(port, sid, ADDR_PRESENT_POSITION)
        if result != 0:
            print(f"  ERROR: Servo {sid} ({LABELS[sid]}) -- read failed: {pkt.getTxRxResult(result)}")
            positions[sid] = None
        else:
            positions[sid] = pos
            print(f"  Servo {sid} ({LABELS[sid]}): {pos}")

    # Check all reads succeeded
    if any(v is None for v in positions.values()):
        print("\nSome servos failed to read. Fix connections and retry.")
        port.closePort()
        sys.exit(1)

    # Step 4: Print results
    print("\n" + "=" * 50)
    print("NEW HOME_POSITIONS (paste into gait file):")
    print("=" * 50)
    print("HOME_POSITIONS = {")
    print(f"    1: {positions[1]}, 2: {positions[2]}, 3: {positions[3]},")
    print(f"    4: {positions[4]}, 5: {positions[5]}, 6: {positions[6]},")
    print("}")

    # Step 5: Symmetry check
    print("\n" + "=" * 50)
    print("SYMMETRY CHECK (L+R should be close to 4096):")
    print("=" * 50)
    all_good = True
    for left_id, right_id, name in PAIRS:
        total = positions[left_id] + positions[right_id]
        offset = total - 4096
        deg = offset / 4096.0 * 360.0
        status = "OK" if abs(offset) < 50 else "BAD"
        if status == "BAD":
            all_good = False
        print(f"  {name:7s}  S{left_id}({positions[left_id]}) + S{right_id}({positions[right_id]}) = {total}  "
              f"offset={offset:+d} ticks ({deg:+.1f} deg)  [{status}]")

    if all_good:
        print("\n  All pairs within 50 ticks -- looks good!")
    else:
        print("\n  Some pairs are off. Re-check those legs are truly vertical")
        print("  and retry. Tolerance: +/- 50 ticks (~4.4 degrees).")

    # Compare with current values
    print("\n" + "=" * 50)
    print("COMPARISON WITH CURRENT HOMES:")
    print("=" * 50)
    current = {1: 1727, 2: 2769, 3: 1431, 4: 2899, 5: 1188, 6: 3200}
    for sid in SERVO_IDS:
        diff = positions[sid] - current[sid]
        deg = diff / 4096.0 * 360.0
        flag = " <-- significant" if abs(diff) > 30 else ""
        print(f"  S{sid} ({LABELS[sid]:12s}): {current[sid]:4d} -> {positions[sid]:4d}  "
              f"delta={diff:+4d} ticks ({deg:+.1f} deg){flag}")

    port.closePort()
    print("\nDone. Copy the HOME_POSITIONS block above into final_full_gait_test.py lines 171-174.")


if __name__ == "__main__":
    main()
