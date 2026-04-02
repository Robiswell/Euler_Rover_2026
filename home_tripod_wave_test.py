#!/usr/bin/env python3
import time
import sys
import signal
import multiprocessing as mp
from scservo_sdk import PortHandler, PacketHandler, GroupSyncWrite

# =================================================================
# HARDWARE CONFIGURATION
# =================================================================
PORT_NAME      = "/dev/ttyUSB1"
BAUDRATE       = 1000000       
SERVO_PROTOCOL = 0               

ADDR_MODE        = 33  
ADDR_TORQUE      = 40  
ADDR_ACCEL       = 41  
ADDR_GOAL_POSITION = 42  
ADDR_GOAL_SPEED  = 46  
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD = 60  
LEN_GOAL_SPEED   = 2   

LEFT_SERVOS  = [2, 3, 4]
RIGHT_SERVOS = [1, 6, 5]
ALL_SERVOS = LEFT_SERVOS + RIGHT_SERVOS

HOME_POSITIONS = {
    1: 1727,
    2: 2769,
    3: 1431,
    4: 2899,
    5: 1188,
    6: 3200,
}

KP_PHASE = 12.0 # Slightly lowered as Feed-Forward is now highly accurate
STALL_THRESHOLD = 750

# --- TRUE KINEMATIC GAIT DICTIONARIES (synced with final_full_gait_test.py) ---
GAITS = {
    0: {  # TRIPOD
        'duty': 0.55,
        'offsets': {2: 0.0, 6: 0.0, 4: 0.0,  1: 0.5, 3: 0.5, 5: 0.5}
    },
    1: {  # WAVE
        'duty': 0.80,
        'offsets': {2: 0.0, 6: 0.167, 4: 0.333, 1: 0.5, 3: 0.667, 5: 0.833}
    },
    2: {  # QUADRUPED
        'duty': 0.7,
        'offsets': {2: 0.0, 6: 0.0,  4: 0.333, 1: 0.333,  3: 0.666, 5: 0.666}
    }
}

# Flawless Buehler Clock mapping
def get_buehler_angle(t_norm, duty_cycle, start_ang, end_ang, is_reversed):
    # Find shortest angular distance for stance phase
    stance_sweep = (end_ang - start_ang + 180) % 360 - 180
    
    if is_reversed:
        start_ang, end_ang = end_ang, start_ang
        stance_sweep = -stance_sweep

    if t_norm <= duty_cycle:
        # Ground Stroke
        progress = t_norm / duty_cycle
        angle = start_ang + (stance_sweep * progress)
    else:
        # Air Stroke
        progress = (t_norm - duty_cycle) / (1.0 - duty_cycle)
        air_sweep = 360.0 - abs(stance_sweep)
        if stance_sweep < 0: 
            air_sweep = -air_sweep
        angle = end_ang + (air_sweep * progress)
        
    return angle % 360

# =================================================================
# PROCESS 2: THE HEART (CLOSED-LOOP GAIT THREAD)
# =================================================================
def gait_worker(shared_speed, shared_x_flip, shared_z_flip, shared_turn_bias, shared_gait_id, shared_impact_start, shared_impact_end, shared_servo_loads, is_running):
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    port_handler = PortHandler(PORT_NAME)
    packet_handler = PacketHandler(SERVO_PROTOCOL)
    group_sync_write = GroupSyncWrite(port_handler, packet_handler, ADDR_GOAL_SPEED, LEN_GOAL_SPEED)
    
    is_stalled = {id: False for id in ALL_SERVOS}
    
    def init_and_align_servos():
        if not port_handler.openPort() or not port_handler.setBaudRate(BAUDRATE):
            print("[Heart] Failed to open port! Exiting Gait Thread.")
            is_running.value = False
            return
            
        print("[Heart] Snapping to absolute calibration positions...")
        for sid in ALL_SERVOS:
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 0) # Position Mode
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE, 1)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 800)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_POSITION, HOME_POSITIONS[sid])
        
        time.sleep(3.0) 
        
        print("[Heart] Alignment complete. Switching to continuous kinematics.")
        for sid in ALL_SERVOS:
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 1) # Velocity Mode
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_ACCEL, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE, 1)

    init_and_align_servos()
    
    master_time_L = 0.0
    master_time_R = 0.0
    target_dt = 1.0 / 50.0  
    
    try:
        while is_running.value:
            loop_start = time.perf_counter()

            # 1. READ PHYSICAL SENSORS 
            actual_phases = {}
            for sid in ALL_SERVOS:
                pos, res1, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_POSITION)
                load, res2, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_LOAD)
                
                if res1 == 0 and res2 == 0:
                    load_mag = load & 0x3FF 
                    shared_servo_loads[sid] = load_mag
                    is_stalled[sid] = (load_mag > STALL_THRESHOLD)

                    diff = pos - HOME_POSITIONS[sid]
                    if sid in LEFT_SERVOS:
                        diff = -diff # Left encoders decrease when moving forward (CCW)
                    
                    angle = (diff / 4096.0) * 360.0
                    actual_phases[sid] = angle % 360
                else:
                    actual_phases[sid] = 0 

            # 2. GET BRAIN COMMANDS
            cmd_speed = shared_speed.value
            cmd_x     = shared_x_flip.value
            cmd_z     = shared_z_flip.value
            cmd_turn  = shared_turn_bias.value
            cmd_gait  = shared_gait_id.value
            imp_start = shared_impact_start.value
            imp_end   = shared_impact_end.value
            
            # 3. ADVANCE MASTER TIME CLOCKS
            # 1000 speed = 1.0 Full Gait Cycle per second
            cycle_hz = (abs(cmd_speed) / 1000.0)
            delta = cycle_hz * target_dt
            
            master_time_L = (master_time_L + delta * (1.0 + cmd_turn)) % 1.0
            master_time_R = (master_time_R + delta * (1.0 - cmd_turn)) % 1.0
            
            gait_params = GAITS.get(cmd_gait, GAITS[0])
            duty = gait_params['duty']
            offsets = gait_params['offsets']
            
            is_rev = (cmd_speed * cmd_x * cmd_z) < 0
            
            # 4. KINEMATIC CONTROLLER (Direction-Aware Anti-Wrap)
            for sid in ALL_SERVOS:
                if is_stalled[sid] or cmd_speed == 0:
                    final_speed = 0
                else:
                    master_time = master_time_L if sid in LEFT_SERVOS else master_time_R
                    current_phase = actual_phases.get(sid, 0)
                    t_leg = (master_time + offsets[sid]) % 1.0
                    
                    # A. Target Phase & Sweep Math
                    target_phase = get_buehler_angle(t_leg, duty, imp_start, imp_end, is_rev)
                    
                    stance_sweep = (imp_end - imp_start + 180) % 360 - 180
                    air_sweep = 360.0 - abs(stance_sweep)
                    
                    # B. Accurate Feed-Forward Speed Calculation (Deg/Sec converted to Servo Units)
                    if t_leg <= duty:
                        deg_per_sec = (abs(stance_sweep) * cycle_hz) / duty
                    else:
                        deg_per_sec = (air_sweep * cycle_hz) / (1.0 - duty)
                        
                    ff_speed = deg_per_sec * 1.85 # ~1.85 servo units per degree/sec
                    
                    # C. Proportional Error with ANTI-WRAP logic
                    error = target_phase - current_phase
                    short_error = (error + 180) % 360 - 180
                    
                    # Prevent the motor from braking/reversing if the target sprints too far ahead
                    if not is_rev and short_error < -90:
                        short_error += 360
                    elif is_rev and short_error > 90:
                        short_error -= 360
                        
                    # D. Final Speed Compilation
                    raw_speed = ff_speed + (short_error * KP_PHASE)
                    
                    if is_rev: raw_speed = -raw_speed
                    if sid in LEFT_SERVOS: raw_speed = -raw_speed 

                    final_speed = max(-3000, min(3000, int(raw_speed)))
                
                abs_speed = int(abs(final_speed))
                speed_val = (abs_speed | 0x8000) if final_speed < 0 else abs_speed
                group_sync_write.addParam(sid, [speed_val & 0xff, (speed_val >> 8) & 0xff])
            
            group_sync_write.txPacket()
            group_sync_write.clearParam()
            
            elapsed_time = time.perf_counter() - loop_start
            sleep_time = target_dt - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            
    except Exception as e:
        print(f"[Heart] Error encountered: {e}")
    finally:
        print("\n[Heart] Safely shutting down servos...")
        port_handler.clearPort()
        for sid in ALL_SERVOS:
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE, 0)
        port_handler.closePort()
        print("[Heart] Servos disabled and port closed.")


# =================================================================
# PROCESS 1: THE BRAIN (AUTONOMOUS FSM SIMULATION)
# =================================================================
if __name__ == "__main__":
    shared_speed        = mp.Value('i', 0)  
    shared_x_flip       = mp.Value('i', 1)  
    shared_z_flip       = mp.Value('i', 1)  
    shared_turn_bias    = mp.Value('d', 0.0) 
    shared_gait_id      = mp.Value('i', 0)   
    
    shared_impact_start = mp.Value('i', 320) 
    shared_impact_end   = mp.Value('i', 40)  
    shared_servo_loads  = mp.Array('i', 7) 
    is_running          = mp.Value('b', True) 

    gait_process = mp.Process(target=gait_worker, args=(
        shared_speed, shared_x_flip, shared_z_flip, shared_turn_bias, 
        shared_gait_id, shared_impact_start, shared_impact_end, 
        shared_servo_loads, is_running
    ))
    gait_process.start()

    try:
        print("\n[Brain] Waiting for hardware calibration...")
        time.sleep(4) 
        
        print("[Brain FSM] State: EXPLORE. Tripod Gait.")
        shared_gait_id.value = 0
        # Speed 500 = 0.5 cycles per sec. Safe, steady walking speed.
        shared_speed.value = 500  
        time.sleep(18) 

        print("[Brain FSM] Action: Increasing Ride Height (Walking Tall)")
        shared_impact_start.value = 340 
        shared_impact_end.value = 20    
        time.sleep(18) 
        
        print("[Brain FSM] State: CLIMB. Wave Gait. Legs physically reorganizing...")
        shared_gait_id.value = 1
        # Wave gait air-stroke is violent. We MUST lower the cycle speed to 
        # keep the servos within their physical RPM limits!
        shared_speed.value = 350 
        time.sleep(24) 

        print("[Brain FSM] Goal Reached. Stopping.")
        shared_speed.value = 0
        time.sleep(4) 

    except KeyboardInterrupt:
        print("\n[Brain] Ctrl+C Detected.")
        
    finally:
        is_running.value = False
        gait_process.join() 
        print("System shutdown complete.")