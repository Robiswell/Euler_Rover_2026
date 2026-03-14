#!/usr/bin/env python3
import time
from scservo_sdk import PortHandler, PacketHandler

PORT_NAME      = "/dev/ttyUSB0"  
BAUDRATE       = 1000000       
SERVO_PROTOCOL = 0               

ADDR_TORQUE           = 40  
ADDR_PRESENT_POSITION = 56  # Register where the servo stores its current angle (2 bytes)

ALL_SERVOS = [1, 2, 3, 4, 5, 6]

def run_calibration():
    port_handler = PortHandler(PORT_NAME)
    packet_handler = PacketHandler(SERVO_PROTOCOL)

    if not port_handler.openPort() or not port_handler.setBaudRate(BAUDRATE):
        print("Failed to open port!")
        return

    print("\n--- ROBOT INDIVIDUAL CALIBRATION MODE ---")
    print("1. Turning off all torque (motors will go limp)...")
    for sid in ALL_SERVOS:
        packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE, 0)
        
    print("\n2. You have 30 seconds!")
    print("   Physically push the robot down so ALL legs are pointing perfectly straight at the ground.")
    
    for i in range(30, 0, -1):
        print(f"   Reading in {i} seconds...")
        time.sleep(1)
        
    print("\n3. Reading Absolute Encoders...")
    positions = {}
    for sid in ALL_SERVOS:
        # Read 2 bytes from the present position register
        pos, result, error = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_POSITION)
        if result == 0:
            positions[sid] = pos
        else:
            print(f"   [!] Error reading Servo {sid}")

    print("\n========================================================")
    print("CALIBRATION RESULTS:")
    print("========================================================")
    print("Copy and paste this EXACT block into your gait_test_pro.py")
    print("replacing the old 'HOME_POSITION = 2048' line:\n")
    
    print("HOME_POSITIONS = {")
    for sid in sorted(positions.keys()):
        print(f"    {sid}: {positions[sid]},")
    print("}")
    print("========================================================\n")
    
    port_handler.closePort()

if __name__ == "__main__":
    run_calibration()