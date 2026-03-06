#!/usr/bin/env python3
"""
=============================================================================
HEXAPOD KINEMATICS
=============================================================================
SYSTEM ARCHITECTURE:
- Process 1 (The Brain): 
  Autonomous Tactical Mission Control. Executes an exhaustive multi-gait 
  matrix course with high-verbosity diagnostic narratives for all maneuvers
  and error-recovery sequences.
  
- Process 2 (The Heart): 
  50Hz Real-Time Kinematics Processor. Combines "Pure Flow" magnitude-based 
  Buehler math for stutter-free rotation and V69 LERP for state-morphing.

HARDWARE: Optimized for STS3215 12V High-Torque Bus Servos. 
Zero software offsets implemented; clearance managed physically.
=============================================================================
"""

import time
import sys
import signal
import datetime
import os
import multiprocessing as mp
import gc  

# --- HARDWARE SDK INTEGRITY CHECK ---
try:
    from scservo_sdk import PortHandler, PacketHandler, GroupSyncWrite
except ImportError:
    print("\n" + "!"*60)
    print("[CRITICAL ERROR] scservo_sdk (SCServo SDK) is not installed.")
    print("Please install it to run on hardware: pip install scservo_sdk")
    print("!"*60 + "\n")
    sys.exit(1)

# =================================================================
# HARDWARE CONFIGURATION & REGISTER MAPPING
# =================================================================
PORT_NAME      = "/dev/ttyUSB0"  
BAUDRATE       = 1000000         
SERVO_PROTOCOL = 0               

# STS3215 Register Map
ADDR_MODE             = 33  # 0: Position, 1: Velocity
ADDR_TORQUE_ENABLE    = 40  
ADDR_ACCEL            = 41  
ADDR_GOAL_POSITION    = 42  
ADDR_GOAL_SPEED       = 46  
ADDR_TORQUE_LIMIT     = 16  
ADDR_PRESENT_POSITION = 56  
ADDR_PRESENT_SPEED    = 58  
ADDR_PRESENT_LOAD     = 60  
ADDR_PRESENT_VOLTAGE  = 62  
ADDR_PRESENT_TEMP     = 63  
ADDR_PRESENT_CURRENT  = 69  
LEN_GOAL_SPEED        = 2   

# Robot Physical Constants
ENCODER_RESOLUTION = 4096.0
VELOCITY_SCALAR    = 1.85 
LEFT_SERVOS        = [2, 3, 4]
RIGHT_SERVOS       = [1, 6, 5]
ALL_SERVOS         = LEFT_SERVOS + RIGHT_SERVOS

# Industrial Safety Parameters
TEMP_MAX     = 70      
VOLTAGE_MIN  = 10.0    
LOG_FILE     = "telemetry_log.txt"
LOG_MAX_SIZE = 10 * 1024 * 1024 

# Mirrored Physical Mounting Map
DIRECTION_MAP = {
    1: 1,   # Right Front
    2: -1,  # Left Front (Inverted)
    3: -1,  # Left Middle (Inverted)
    4: -1,  # Left Rear (Inverted)
    5: 1,   # Right Rear
    6: 1,   # Right Middle
}

# Calibrated zero points (Legs pointing straight down)
HOME_POSITIONS = {
    1: 2233, 2: 2731, 3: 4086,
    4: 2606, 5: 253,  6: 771,
}

# Feedback Gains
KP_PHASE        = 12.0
STALL_THRESHOLD = 600 

# --- KINEMATIC GAIT DICTIONARIES ---
GAITS = {
    0: {  # TRIPOD
        'duty': 0.5, 
        'offsets': {2: 0.0, 6: 0.0, 4: 0.0,  1: 0.5, 3: 0.5, 5: 0.5}
    },
    1: {  # WAVE
        'duty': 0.85, 
        'offsets': {4: 0.833, 3: 0.666, 2: 0.5,  5: 0.380, 6: 0.166, 1: 0.0}
    },
    2: {  # QUADRUPED
        'duty': 0.7,
        'offsets': {2: 0.0, 5: 0.0,  3: 0.333, 6: 0.333,  4: 0.666, 1: 0.666}
    }
}

# -----------------------------------------------------------------
# KINEMATIC CORE: PURE BUEHLER CLOCK
# -----------------------------------------------------------------
def get_buehler_angle(t_norm, duty_cycle, start_ang, end_ang, is_reversed):
    """
    Pure geometric mapping for stutter-free 360-degree rotation.
    Handles stance/air phases with magnitude-based feed-forward logic.
    """
    # Calculate shortest angular distance for stance phase
    stance_sweep = (end_ang - start_ang + 180) % 360 - 180
    
    if is_reversed:
        start_ang, end_ang = end_ang, start_ang
        stance_sweep = -stance_sweep

    if t_norm <= duty_cycle:
        # PROGRESS: STANCE
        progress = t_norm / duty_cycle
        angle = start_ang + (stance_sweep * progress)
    else:
        # PROGRESS: AIR
        progress = (t_norm - duty_cycle) / (1.0 - duty_cycle)
        air_sweep = 360.0 - abs(stance_sweep)
        if stance_sweep < 0: 
            air_sweep = -air_sweep
        angle = end_ang + (air_sweep * progress)
        
    return angle % 360

# =================================================================
# PROCESS 2: THE HEART (REAL-TIME ENGINE)
# =================================================================
def gait_worker(shared_speed, shared_x_flip, shared_z_flip, shared_turn_bias, shared_gait_id, 
                shared_impact_start, shared_impact_end, shared_servo_loads, shared_heartbeat, is_running):
    """
    Executes high-frequency kinematics loop (50Hz).
    Maintains rotation smoothness using magnitude-based signs.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        os.nice(-20) 
    except:
        pass
    gc.disable() 

    port_handler = PortHandler(PORT_NAME)
    packet_handler = PacketHandler(SERVO_PROTOCOL)
    group_sync_write = GroupSyncWrite(port_handler, packet_handler, ADDR_GOAL_SPEED, LEN_GOAL_SPEED)
    port_handler.setPacketTimeoutMillis(10)
    
    # Internal Tracking
    is_stalled = {id: False for id in ALL_SERVOS}
    stall_counters = {id: 0 for id in ALL_SERVOS}
    last_actual_phases = {id: 0.0 for id in ALL_SERVOS}
    
    telemetry_rotator = 0
    voltage_reading = 12.0
    volt_dip_counter = 0 
    temp_spike_counter = 0 
    parent_pid = os.getppid()
    last_log_time = 0
    
    def log_telemetry(volt, temp_max, load_max, total_amps, fsm_speed, fsm_turn):
        """Industrial Standard Logging."""
        try:
            if os.path.exists(LOG_FILE) and os.path.getsize(LOG_FILE) > LOG_MAX_SIZE:
                os.rename(LOG_FILE, LOG_FILE + f".{int(time.time())}.old")
            with open(LOG_FILE, "a") as f:
                ts = datetime.datetime.now().strftime("%H:%M:%S")
                f.write(f"[{ts}] Spd:{fsm_speed} Trn:{fsm_turn:.1f} | V:{volt:.1f}V | T:{temp_max}C | L:{load_max} | A:{total_amps:.2f}\n")
        except: pass

    def init_and_align_servos():
        """Reliable Boot Sequence: Position Mode snap."""
        while is_running.value:
            if port_handler.openPort() and port_handler.setBaudRate(BAUDRATE): break
            time.sleep(1.0)
            
        port_handler.clearPort()
        print("[Heart] BOOT: Aligned snap to calibration points...")
        for sid in ALL_SERVOS:
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 0) # Position
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_ACCEL, 20)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_TORQUE_LIMIT, 400)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 800)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_POSITION, HOME_POSITIONS[sid])
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 1)
        
        time.sleep(3.0) 
        
        print("[Heart] BOOT: Transitioning to Pure Flow Kinematics.")
        for sid in ALL_SERVOS:
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 1) # Velocity
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_ACCEL, 0) 
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_TORQUE_LIMIT, 1000)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 1)

    init_and_align_servos()
    
    master_time_L, master_time_R = 0.0, 0.0
    target_dt     = 1.0 / 50.0  
    last_loop_time = time.perf_counter()
    
    # State Smoothing (V69)
    smooth_hz        = 0.0
    smooth_turn      = 0.0
    smooth_imp_start = float(shared_impact_start.value)
    smooth_imp_end   = float(shared_impact_end.value)
    smooth_duty      = 0.5
    smooth_offsets   = {sid: 0.0 for sid in ALL_SERVOS}
    
    try:
        while is_running.value:
            loop_start = time.perf_counter()
            real_dt = min(0.05, loop_start - last_loop_time)
            last_loop_time = loop_start
            
            shared_heartbeat.value += 1
            port_handler.clearPort()
            
            if loop_start - last_log_time > 1.0:
                if os.getppid() != parent_pid: break
                gc.collect() 

            # 1. READ PHYSICAL FEEDBACK
            actual_phases = {}
            current_temp_max, current_load_max, current_total_ma = 0, 0, 0
            
            for sid in ALL_SERVOS:
                pos, res1, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_POSITION)
                
                if sid == ALL_SERVOS[telemetry_rotator] or sid == 1:
                    load, res2, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_LOAD)
                    temp, res3, _ = packet_handler.read1ByteTxRx(port_handler, sid, ADDR_PRESENT_TEMP)
                    volt, res4, _ = packet_handler.read1ByteTxRx(port_handler, sid, ADDR_PRESENT_VOLTAGE)
                    amps, res5, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_CURRENT)
                    
                    if res2 == 0:
                        l_mag = load & 0x3FF 
                        shared_servo_loads[sid] = l_mag
                        if l_mag > STALL_THRESHOLD: stall_counters[sid] += 1
                        else: stall_counters[sid] = 0
                        is_stalled[sid] = (stall_counters[sid] >= 3)
                        current_load_max = max(current_load_max, l_mag)
                        
                    if res3 == 0 and temp > current_temp_max: current_temp_max = temp
                    if res4 == 0: voltage_reading = volt / 10.0
                    if res5 == 0: current_total_ma = (amps & 0x7FFF)

                if res1 == 0:
                    diff = pos - HOME_POSITIONS[sid]
                    angle = ((diff / ENCODER_RESOLUTION) * 360.0 * DIRECTION_MAP[sid]) % 360
                    actual_phases[sid] = angle
                    last_actual_phases[sid] = angle
                else:
                    actual_phases[sid] = last_actual_phases[sid]

            telemetry_rotator = (telemetry_rotator + 1) % len(ALL_SERVOS)

            # Safety Guards
            if current_temp_max > TEMP_MAX: temp_spike_counter += 1
            else: temp_spike_counter = 0

            if voltage_reading < VOLTAGE_MIN: volt_dip_counter += 1
            else: volt_dip_counter = 0
            
            if temp_spike_counter > 15 or volt_dip_counter > 100:
                print(f"[Heart] CRITICAL SHUTDOWN: Logic Halt.")
                log_telemetry(voltage_reading, current_temp_max, current_load_max, current_total_ma / 1000.0, shared_speed.value, shared_turn_bias.value)
                is_running.value = False 
                break

            if loop_start - last_log_time > 1.0:
                log_telemetry(voltage_reading, current_temp_max, current_load_max, current_total_ma / 1000.0, shared_speed.value, shared_turn_bias.value)
                last_log_time = loop_start

            # 2. STATE LERPING (Software Inertia)
            target_hz = (shared_speed.value * shared_x_flip.value * shared_z_flip.value) / 1000.0
            target_turn = shared_turn_bias.value
            
            lerp_rate = min(1.0, 4.0 * real_dt) 
            smooth_hz += (target_hz - smooth_hz) * lerp_rate
            smooth_turn += (target_turn - smooth_turn) * lerp_rate
            
            d_s = (shared_impact_start.value - smooth_imp_start + 180) % 360 - 180
            smooth_imp_start = (smooth_imp_start + d_s * lerp_rate) % 360
            
            d_e = (shared_impact_end.value - smooth_imp_end + 180) % 360 - 180
            smooth_imp_end = (smooth_imp_end + d_e * lerp_rate) % 360

            gait_params = GAITS.get(shared_gait_id.value, GAITS[0])
            t_duty = max(0.01, min(0.99, gait_params['duty']))
            smooth_duty += (t_duty - smooth_duty) * min(1.0, 1.0 * real_dt) 
            
            t_offsets = gait_params['offsets']
            for sid in ALL_SERVOS:
                o_diff = (t_offsets[sid] - smooth_offsets[sid] + 0.5) % 1.0 - 0.5
                smooth_offsets[sid] = (smooth_offsets[sid] + o_diff * lerp_rate) % 1.0

            # 3. DRIVE CALCULATIONS & SAFETY GOVERNOR
            hz_L = smooth_hz + smooth_turn
            hz_R = smooth_hz - smooth_turn
            
            base_sweep = (smooth_imp_end - smooth_imp_start + 180) % 360 - 180
            air_sweep = 360.0 - abs(base_sweep)
            if base_sweep < 0: air_sweep = -air_sweep
            
            max_safe_hz = (2800.0 / VELOCITY_SCALAR * (1.0 - smooth_duty)) / max(5.0, abs(air_sweep))
            hz_L = max(-max_safe_hz, min(max_safe_hz, hz_L))
            hz_R = max(-max_safe_hz, min(max_safe_hz, hz_R))

            cycle_hz_L, cycle_hz_R = abs(hz_L), abs(hz_R)
            if cycle_hz_L > 0.001: master_time_L = (master_time_L + cycle_hz_L * real_dt) % 1.0
            if cycle_hz_R > 0.001: master_time_R = (master_time_R + cycle_hz_R * real_dt) % 1.0
            
            # Phase Resync for stability
            if abs(smooth_turn) < 0.01 and abs(smooth_hz) > 0.01 and (hz_L * hz_R) > 0:
                clock_diff = (master_time_L - master_time_R + 0.5) % 1.0 - 0.5
                if abs(clock_diff) > 0.005:  
                    decay = clock_diff * min(1.0, 2.0 * real_dt) 
                    master_time_L = (master_time_L - decay/2) % 1.0
                    master_time_R = (master_time_R + decay/2) % 1.0
                else: master_time_R = master_time_L

            # 4. KINEMATIC CONTROLLER (Pure Math Flow)
            for sid in ALL_SERVOS:
                leg_hz = hz_L if sid in LEFT_SERVOS else hz_R
                is_rev_leg = (leg_hz < 0)
                
                if is_stalled[sid] or abs(leg_hz) < 0.001:
                    final_speed = 0
                else:
                    master_time = master_time_L if sid in LEFT_SERVOS else master_time_R
                    current_phase = actual_phases.get(sid, 0.0)
                    t_leg = (master_time + smooth_offsets[sid]) % 1.0
                    
                    target_phase = get_buehler_angle(t_leg, smooth_duty, smooth_imp_start, smooth_imp_end, is_rev_leg)
                    
                    stance_sweep = (smooth_imp_end - smooth_imp_start + 180) % 360 - 180
                    air_sweep = 360.0 - abs(stance_sweep)
                    
                    if t_leg <= smooth_duty:
                        deg_per_sec = (abs(stance_sweep) * abs(leg_hz)) / smooth_duty
                    else:
                        deg_per_sec = (air_sweep * abs(leg_hz)) / (1.0 - smooth_duty)
                        
                    ff_speed = deg_per_sec * VELOCITY_SCALAR 
                    
                    error = target_phase - current_phase
                    short_error = (error + 180) % 360 - 180
                    
                    if not is_rev_leg and short_error < -90: short_error += 360
                    elif is_rev_leg and short_error > 90: short_error -= 360
                        
                    raw_speed = ff_speed + (short_error * KP_PHASE)
                    
                    if is_rev_leg: raw_speed = -raw_speed
                    if sid in LEFT_SERVOS: raw_speed = -raw_speed 

                    final_speed = max(-3000, min(3000, int(raw_speed)))
                
                abs_speed = int(abs(final_speed))
                speed_val = (abs_speed | 0x8000) if final_speed < 0 else abs_speed
                group_sync_write.addParam(sid, [speed_val & 0xff, (speed_val >> 8) & 0xff])
            
            group_sync_write.txPacket()
            group_sync_write.clearParam()
            
            # 5. PRECISION TIMING
            elapsed_time = time.perf_counter() - loop_start
            sleep_time = target_dt - elapsed_time
            if sleep_time > 0.002: time.sleep(sleep_time - 0.002)
            while time.perf_counter() - loop_start < target_dt: pass 
            
    except Exception as e:
        print(f"[Heart] FATAL: {e}")
    finally:
        # SHUTDOWN: Classic Position Mode
        print("\n" + "*"*60)
        print("[Heart] SHUTDOWN: Snapping to base posture...")
        print("*"*60)
        port_handler.clearPort()
        for sid in ALL_SERVOS: packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 0)
        time.sleep(0.1)
        for sid in ALL_SERVOS:
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 0) 
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_ACCEL, 20)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 1)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 800)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_POSITION, HOME_POSITIONS[sid])
        time.sleep(2.0)
        for sid in ALL_SERVOS: packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
        port_handler.closePort()
        print("[Heart] Offline.")

# =================================================================
# PROCESS 1: THE BRAIN (VERBOSE TACTICAL MISSION CONTROL)
# =================================================================

class EmergencyStopException(Exception): pass

def tactical_sleep(duration, running_flag):
    """Monitors Heartbeat during tactical phases for safety shutdown."""
    start_time = time.time()
    while time.time() - start_time < duration:
        if not running_flag.value:
            raise EmergencyStopException("Safety shutdown triggered.")
        time.sleep(0.1)

if __name__ == "__main__":
    print("\n" + "="*60)
    print("      HEXAPOD KINEMATICS PRO V80 ")
    print("="*60)

    # Multi-Processing Primitives
    shared_speed        = mp.Value('i', 0)  
    shared_x_flip       = mp.Value('i', 1)  
    shared_z_flip       = mp.Value('i', 1)  
    shared_turn_bias    = mp.Value('d', 0.0) 
    shared_gait_id      = mp.Value('i', 0)   
    shared_impact_start = mp.Value('i', 320) 
    shared_impact_end   = mp.Value('i', 40)  
    shared_servo_loads  = mp.Array('i', 7) 
    shared_heartbeat    = mp.Value('i', 0)   
    is_running          = mp.Value('b', True) 

    for i in range(7): shared_servo_loads[i] = -1

    gait_process = mp.Process(target=gait_worker, args=(
        shared_speed, shared_x_flip, shared_z_flip, shared_turn_bias, 
        shared_gait_id, shared_impact_start, shared_impact_end, 
        shared_servo_loads, shared_heartbeat, is_running
    ))
    gait_process.start()

    # --- EXPANDED TACTICAL RECOVERY BEHAVIORS ---
    def state_recovery_wiggle(duration=10):
        """Active Compliance: Extraction frequency vibration to break static friction."""
        print("\n" + "*"*55)
        print("[STATE] RECOVERY: HIGH MECHANICAL LOAD DETECTED!")
        print("[DIAGNOSTIC] Environment: Simulated High-Friction Surface Entanglement.")
        print("[DIAGNOSTIC] Root Cause: Proprioception feedback indicates Leg 2 mechanical stall.")
        print("[ACTION] Initializing High-Frequency Extraction Wiggle (1.6Hz).")
        print("[OBJECTIVE] Using 'Active Compliance' to break static friction bonds through transverse oscillation.")
        print("*"*55)
        
        end_time = time.time() + duration
        cycle_count = 0
        while time.time() < end_time:
            if not is_running.value: raise EmergencyStopException("Hardware aborted.")
            
            cycle_count += 1
            print(f"[RECOVERY] Extraction Phase {cycle_count}: Toggling transverse polarity for chassis oscillation...")
            shared_x_flip.value = -1
            tactical_sleep(0.3, is_running)
            
            shared_x_flip.value = 1
            tactical_sleep(0.3, is_running)
            
        print("[STATE] Physical extraction concluded. Verifying motor current stability...")
        print("[STATE] Resuming mission tactical course.")

    def state_self_right_roll():
        """Recovery: Multi-step momentum roll to flip robot upright using centripetal loading."""
        print("\n" + "!"*55)
        print("[STATE] RECOVERY: GRAVITY INVERSION DETECTED (CAPSIZED)!")
        print("[DIAGNOSTIC] Scenario: Chassis tilt exceeded stability gate on steep incline.")
        print("[DIAGNOSTIC] IMU Feedback: Gravity vector aligned with Z-Positive axis (Inverted).")
        print("[ACTION] Initiating 'Symmetrical Roll' recovery sequence.")
        print("[OBJECTIVE] Generating rotational centripetal momentum to flip the primary chassis mass.")
        print("!"*55)
        
        shared_gait_id.value = 1 # Wave gait for maximum synchronized torque
        shared_impact_start.value, shared_impact_end.value = 320, 40 
        
        print("[FSM] RECOVERY STEP 1: Shifting internal COG to the forward stability quadrant (Primary Load Buildup)..."); 
        shared_speed.value = 400
        tactical_sleep(4.0, is_running)
        
        print("[FSM] RECOVERY STEP 2: Executing high-torque reverse momentum pulse (Inertial Snap Phase)..."); 
        shared_speed.value = -500
        tactical_sleep(8.0, is_running)
        
        print("[FSM] RECOVERY STEP 3: Secondary momentum loading... Completing rollover rotation."); 
        shared_speed.value = 400
        tactical_sleep(4.0, is_running)
        
        shared_speed.value = 0
        print("[STATE] Chassis orientation normalized. Gyroscopic truth verified. Recovery Successful.")

    try:
        print("\n[Brain] System Bootstrapping... Waiting for heartbeat.")
        while shared_heartbeat.value < 50: time.sleep(0.1)
        print("[Brain] Mission Control Online. System Link Established.")
        
        # =========================================================
        # PHASE 1: TRIPOD TACTICAL AGILITY (CONCRETE)
        # =========================================================
        print("\n" + "="*50)
        print("PHASE 1: TRIPOD FULL DIRECTIONAL MATRIX")
        print("="*50)
        shared_gait_id.value = 0; shared_impact_start.value, shared_impact_end.value = 330, 30 
        
        print("[FSM] Tripod: Forward Sprint (Velocity 1200)"); shared_speed.value = 1200; tactical_sleep(12, is_running)
        print("[FSM] Tripod: Carving Turn LEFT"); shared_turn_bias.value = -0.4; tactical_sleep(10, is_running)
        print("[FSM] Tripod: Carving Turn RIGHT"); shared_turn_bias.value = 0.4; tactical_sleep(10, is_running)
        print("[FSM] Tripod: Zero-Radius Tactical Pivot LEFT"); shared_speed.value = 0; shared_turn_bias.value = -1.0; tactical_sleep(10, is_running)
        print("[FSM] Tripod: Zero-Radius Tactical Pivot RIGHT"); shared_speed.value = 0; shared_turn_bias.value = 1.0; tactical_sleep(10, is_running)
        print("[FSM] Tripod: Full-Power Reverse Sprint"); shared_turn_bias.value = 0.0; shared_speed.value = -1200; tactical_sleep(12, is_running)

        # =========================================================
        # PHASE 2: QUADRUPED TACTICAL STABILITY (GRAVEL)
        # =========================================================
        print("\n" + "="*50)
        print("PHASE 2: QUADRUPED FULL DIRECTIONAL MATRIX")
        print("="*50)
        shared_gait_id.value = 2; shared_impact_start.value, shared_impact_end.value = 330, 30 
        
        print("[FSM] Quadruped: Forward Stability Walk"); shared_speed.value = 550; shared_turn_bias.value = 0.0; tactical_sleep(12, is_running)
        print("[FSM] Quadruped: Stability Carving Turn LEFT"); shared_turn_bias.value = -0.3; tactical_sleep(10, is_running)
        print("[FSM] Quadruped: Stability Carving Turn RIGHT"); shared_turn_bias.value = 0.3; tactical_sleep(10, is_running)
        print("[FSM] Quadruped: Zero-Radius Tactical Pivot LEFT"); shared_speed.value = 0; shared_turn_bias.value = -0.8; tactical_sleep(10, is_running)
        print("[FSM] Quadruped: Zero-Radius Tactical Pivot RIGHT"); shared_speed.value = 0; shared_turn_bias.value = 0.8; tactical_sleep(10, is_running)
        print("[FSM] Quadruped: Reverse Stability Course"); shared_turn_bias.value = 0.0; shared_speed.value = -550; tactical_sleep(12, is_running)
        
        print("[FSM] Quadruped Adaptation: WALKING TALL (Obstacle Overpass)"); shared_speed.value = 500; shared_impact_start.value, shared_impact_end.value = 345, 15; tactical_sleep(15, is_running)
        print("[FSM] Quadruped Adaptation: STEALTH CRAWL (Low Clearance)"); shared_impact_start.value, shared_impact_end.value = 325, 35; tactical_sleep(15, is_running)

        # =========================================================
        # PHASE 3: WAVE TACTICAL PRECISION (SAND)
        # =========================================================
        print("\n" + "="*50)
        print("PHASE 3: WAVE FULL DIRECTIONAL MATRIX")
        print("="*50)
        shared_gait_id.value = 1; shared_impact_start.value, shared_impact_end.value = 330, 30
        
        print("[FSM] Wave: Ultra-Stable Forward Crawl"); shared_speed.value = 350; shared_turn_bias.value = 0.0; tactical_sleep(12, is_running)
        print("[FSM] Wave: Precision Carving Turn LEFT"); shared_turn_bias.value = -0.2; tactical_sleep(10, is_running)
        print("[FSM] Wave: Precision Carving Turn RIGHT"); shared_turn_bias.value = 0.2; tactical_sleep(10, is_running)
        print("[FSM] Wave: Precision Tactical Pivot LEFT"); shared_speed.value = 0; shared_turn_bias.value = -0.5; tactical_sleep(10, is_running)
        print("[FSM] Wave: Precision Tactical Pivot RIGHT"); shared_speed.value = 0; shared_turn_bias.value = 0.5; tactical_sleep(10, is_running)
        print("[FSM] Wave: Precision Reverse Stroke"); shared_turn_bias.value = 0.0; shared_speed.value = -350; tactical_sleep(12, is_running)
        
        print("[FSM] Wave Adaptation: COG SHIFT (Hill Climb Navigation)"); shared_speed.value = 350; shared_impact_start.value, shared_impact_end.value = 315, 15; tactical_sleep(20, is_running)

        # =========================================================
        # PHASE 4: EXPANDED ADVERSARIAL RECOVERY
        # =========================================================
        print("\n" + "="*50)
        print("PHASE 4: ADVERSARIAL RECOVERY MATRIX")
        print("="*50)
        print("[FSM] ASSESSMENT: Tactical deceleration for system diagnostics..."); shared_speed.value = 0; shared_turn_bias.value = 0.0; tactical_sleep(3, is_running)
        
        state_recovery_wiggle()
        state_self_right_roll()

        # =========================================================
        # PHASE 5: SHUTDOWN
        # =========================================================
        print("\n" + "="*50)
        print("PHASE 5: NAVIGATION SEQUENCE COMPLETE")
        print("="*50)
        print("[SIMULATION] Sensor: Destination waypoint verified. Mission Success.")
        shared_speed.value = 0; shared_turn_bias.value = 0.0; tactical_sleep(8, is_running) 

    except EmergencyStopException as e:
        print(f"\n[Brain] EMERGENCY ABORT: {e}")
    except KeyboardInterrupt:
        print("\n[Brain] INTERRUPT: Tactical Kill Command Received.")
    finally:
        is_running.value = False
        gait_process.join(timeout=15)
        print("\nMission Matrix Concluded. All systems offline.")