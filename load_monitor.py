#!/usr/bin/env python3
"""Live load monitor — reads servo loads every 500ms and prints them.
No motion, no gait. Just raw load readings from all 6 servos.
Run on Pi: sudo python3 load_monitor.py
Ctrl+C to stop.
"""
import time
import sys
from scservo_sdk import PortHandler, PacketHandler, GroupSyncRead

PORT_NAME = "/dev/ttyUSB1"
BAUDRATE = 1000000
SERVO_IDS = [1, 2, 3, 4, 5, 6]
ADDR_PRESENT_LOAD = 60   # 2 bytes: 60-61 (STS3215)
ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_POSITION = 56

port = PortHandler(PORT_NAME)
packet = PacketHandler(0)

if not port.openPort():
    print("ERROR: could not open port"); sys.exit(1)
if not port.setBaudRate(BAUDRATE):
    print("ERROR: could not set baudrate"); sys.exit(1)

# Enable torque so servos hold position (required for non-zero Present Load readings)
print("Enabling torque on all servos...")
for sid in SERVO_IDS:
    result, err = packet.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, 1)
    if result != 0 or err != 0:
        print(f"  servo {sid}: FAILED (result={result} err={err})")
    else:
        # Also read current position to confirm servo is alive
        pos, result, err = packet.read2ByteTxRx(port, sid, ADDR_PRESENT_POSITION)
        print(f"  servo {sid}: torque ON, position={pos}")

reader = GroupSyncRead(port, packet, ADDR_PRESENT_LOAD, 2)
for sid in SERVO_IDS:
    reader.addParam(sid)

print("LOAD MONITOR  (Ctrl+C to stop)")
print(f"{'time':>6} | {'s1':>5} {'s2':>5} {'s3':>5} {'s4':>5} {'s5':>5} {'s6':>5} | {'avg':>5} {'max':>5} {'min':>5}")
print("-" * 72)

t0 = time.monotonic()
try:
    while True:
        result = reader.txRxPacket()
        loads = []
        for sid in SERVO_IDS:
            if reader.isAvailable(sid, ADDR_PRESENT_LOAD, 2):
                raw = reader.getData(sid, ADDR_PRESENT_LOAD, 2)
                mag = raw & 0x3FF
                loads.append(mag)
            else:
                loads.append(-1)
        avg = sum(loads) / 6
        mx = max(loads)
        mn = min(loads)
        t = time.monotonic() - t0
        print(f"{t:6.1f} | {loads[0]:5d} {loads[1]:5d} {loads[2]:5d} {loads[3]:5d} {loads[4]:5d} {loads[5]:5d} | {avg:5.0f} {mx:5d} {mn:5d}")
        time.sleep(0.5)
except KeyboardInterrupt:
    print("\nDisabling torque...")
    for sid in SERVO_IDS:
        packet.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, 0)
    print("stopped")
    port.closePort()
