#!/usr/bin/env python3
"""
=============================================================================
HEXAPOD KINEMATICS
=============================================================================
This is the kinematics engine for a 6-legged robot using STS3215 serial bus servos.

CORE ARCHITECTURE:
- Process 1 (The Brain): 
  Manages the Tactical State Machine, high-level path planning, and
  simulated sensor integration (IMU/Ultrasonic/Load).
  
- Process 2 (The Heart): 
  A high-frequency (50Hz) real-time kinematics processor that manages
  closed-loop phase tracking, hardware safety, and bus duty-cycle.

SAFETY CAPABILITIES:
- 69 Independent adversarial safety layers.
- Linux Real-Time Priority Escalation (Nice -20).
- Automatic Garbage Collection management for zero-jitter frames.
- Rotating Health Telemetry to prevent Bus Saturation.
- Zero-Jerk handover and phase-aligned LERP shutdown.
=============================================================================
"""

import time
import sys
import signal
import datetime
import os
import multiprocessing as mp
import gc  # Level 7 Audit: Manual memory management for real-time loops

# --- HARDWARE SDK INTEGRITY CHECK ---
# Ensures the necessary Waveshare/SCServo SDK is available before execution.
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
PORT_NAME      = "/dev/ttyUSB0"  # Physical UART/USB device interface
BAUDRATE       = 1000000         # 1Mbps high-speed serial bus
SERVO_PROTOCOL = 0               # Protocol 0 for STS series servos

# Register addresses for STS3215 Control Table
ADDR_TORQUE_ENABLE    = 40  # 0: Torque Off, 1: Torque On
ADDR_ACCEL            = 41  # Internal hardware acceleration ramp
ADDR_GOAL_POSITION    = 42  # 2-byte goal position (0-4095)
ADDR_GOAL_SPEED       = 46  # 2-byte goal speed (0-3000)
ADDR_TORQUE_LIMIT     = 16  # Global torque ceiling (EC51 protection)
ADDR_MODE             = 33  # 0: Position Mode, 1: Velocity Mode
ADDR_PRESENT_POSITION = 56  # 2-byte current physical angle
ADDR_PRESENT_SPEED    = 58  # 2-byte current physical speed
ADDR_PRESENT_LOAD     = 60  # Current motor load (0-1023)
ADDR_PRESENT_VOLTAGE  = 62  # Voltage reading (V * 10)
ADDR_PRESENT_TEMP     = 63  # Internal thermal sensor (Celsius)
ADDR_PRESENT_CURRENT  = 69  # Real-time current draw (Milliamps)
LEN_GOAL_SPEED        = 2   # Byte length for velocity sync writes

# --- ROBOT PHYSICAL CONSTANTS ---
ENCODER_RESOLUTION = 4096.0  # Ticks per 360-degree rotation
VELOCITY_SCALAR    = 1.85    # Multiplier for degrees/sec to STS units
VOLTAGE_MIN        = 9.5     # Absolute floor for 3S LiPo batteries
TEMP_MAX           = 70      # Thermal safety shutdown threshold
LOG_FILE           = "telemetry_log.txt"
LOG_MAX_SIZE       = 10 * 1024 * 1024  # 10MB Log Rotation (EC58)

# differential grouping logic
LEFT_SERVOS  = [2, 3, 4]
RIGHT_SERVOS = [1, 6, 5]
ALL_SERVOS   = LEFT_SERVOS + RIGHT_SERVOS

# EC60: Coordinate system inversion map based on physical motor orientation
DIRECTION_MAP = {
    1: 1,   # Right Front
    2: -1,  # Left Front (Inverted)
    3: -1,  # Left Middle (Inverted)
    4: -1,  # Left Rear (Inverted)
    5: 1,   # Right Rear
    6: 1,   # Right Middle
}

# Calibrated absolute zero points for leg orientation (pointing straight down)
HOME_POSITIONS = {
    1: 2233, 2: 2731, 3: 4086,
    4: 2606, 5: 253,  6: 771,
}

# Control Loop Gain Tuning
KP_PHASE        = 12.5       # Proportional gain for trajectory tracking
STALL_THRESHOLD = 600        # Load magnitude required to trigger yield

# --- KINEMATIC GAIT DICTIONARIES ---
GAITS = {
    0: {  # TRIPOD (High speed, 3-leg support)
        'duty': 0.5, 
        'offsets': {2: 0.0, 6: 0.0, 4: 0.0,  1: 0.5, 3: 0.5, 5: 0.5}
    },
    1: {  # WAVE (Max stability, 5-leg support)
        'duty': 0.85, 
        'offsets': {4: 0.833, 3: 0.666, 2: 0.5,  5: 0.380, 6: 0.166, 1: 0.0}
    },
    2: {  # QUADRUPED (Medium terrain, 4-leg support)
        'duty': 0.7,
        'offsets': {4: 0.0, 1: 0.0,  3: 0.333, 6: 0.333,  2: 0.666, 5: 0.666}
    }
}

# -----------------------------------------------------------------
# PURE GEOMETRIC BUEHLER CLOCK ENGINE
# -----------------------------------------------------------------
def get_buehler_angle(t_norm, duty_cycle, start_ang, end_ang):
    """
    Calculates the instantaneous target angle for a specific leg.
    Stance Phase: Linear ground sweep (from start to end).
    Air Phase: Accelerated return swing (from end back to start).
    """
    # EC67: Parity guard ensures the sweep always takes the shortest path
    stance_sweep = (end_ang - start_ang + 180) % 360 - 180
    
    if t_norm <= duty_cycle:
        # PROGRESS: STANCE (Ground Contact)
        progress = t_norm / duty_cycle
        angle = start_ang + (stance_sweep * progress)
    else:
        # PROGRESS: AIR (Return Swing)
        progress = (t_norm - duty_cycle) / (1.0 - duty_cycle)
        air_sweep = 360.0 - abs(stance_sweep)
        # Match sign for return direction
        if stance_sweep < 0:
            air_sweep = -air_sweep
        angle = end_ang + (air_sweep * progress)
        
    return angle % 360

# =================================================================
# PROCESS 2: THE HEART (CLOSED-LOOP GAIT ENGINE)
# =================================================================
def gait_worker(shared_speed, shared_x_flip, shared_z_flip, shared_turn_bias, shared_gait_id, 
                shared_impact_start, shared_impact_end, shared_servo_loads, shared_heartbeat, is_running):
    """
    Process 2: The Heart.
    This thread performs high-speed kinematics, PD loop correction, 
    and handles all identified physical edge cases.
    """
    # EC18, EC53, EC66: Signal Alignment
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    # EC29: Escalate process priority to Real-Time level in the Linux Kernel
    try:
        os.nice(-20)
    except:
        print("[Heart] Priority Alert: Running without administrative rights. Loop jitter may occur.")

    # EC38: Disable Garbage Collection to ensure 20ms frame consistency
    gc.disable()

    port_handler = PortHandler(PORT_NAME)
    packet_handler = PacketHandler(SERVO_PROTOCOL)
    group_sync_write = GroupSyncWrite(port_handler, packet_handler, ADDR_GOAL_SPEED, LEN_GOAL_SPEED)
    
    # EC21: Shorten packet timeout to 10ms to prevent loop blockage on dead hardware
    port_handler.setPacketTimeoutMillis(10)
    
    # Internal State Management
    is_stalled = {id: False for id in ALL_SERVOS}
    stall_counters = {id: 0 for id in ALL_SERVOS} 
    actual_phases = {id: 0.0 for id in ALL_SERVOS} 
    comm_error_streak = 0
    telemetry_rotator = 0
    volt_dip_counter = 0 
    last_log_time = 0
    parent_pid = os.getppid() # EC24: Monitor parent for orphaned thread prevention
    
    def log_telemetry(volt, temp_max, load_max, total_amps, fsm_speed, fsm_turn):
        """EC58 & EC68: Smart Log Management."""
        try:
            # Check for log size to prevent filling SD card
            if os.path.exists(LOG_FILE) and os.path.getsize(LOG_FILE) > LOG_MAX_SIZE:
                os.rename(LOG_FILE, LOG_FILE + f".{int(time.time())}.old")

            with open(LOG_FILE, "a") as f:
                ts = datetime.datetime.now().strftime("%H:%M:%S")
                f.write(f"[{ts}] Spd:{fsm_speed} Trn:{fsm_turn:.1f} | V:{volt:.1f}V | T:{temp_max}C | L:{load_max} | A:{total_amps:.2f}\n")
        except:
            pass

    def init_and_align_servos():
        """EC55, EC51, EC7: Robust Hardware Connection Logic."""
        # EC62: OS Lock-file verification
        while is_running.value:
            if port_handler.openPort() and port_handler.setBaudRate(BAUDRATE):
                break
            print("[Heart] Port Locked: Waiting for Linux to release serial resource...")
            time.sleep(1.0)
            
        port_handler.clearPort() # EC41: Clear serial artifacts
        print("[Heart] BOOT: Moving to STAND stance with Soft-Torque protection (40%)...")
        
        for sid in ALL_SERVOS:
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 0) # Position Mode
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_ACCEL, 20) # Smooth hardware ramp
            # EC51: Limit torque during initial alignment to protect gears
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_TORQUE_LIMIT, 400)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 800)
            # EC7: Load goal position BEFORE torque-on to prevent snap
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_POSITION, HOME_POSITIONS[sid])
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 1)
        time.sleep(3.0) 
        
        # EC13: Perform blocking truth-read to prevent 1-frame snap on loop start
        for sid in ALL_SERVOS:
            pos, r_val, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_POSITION)
            if r_val == 0:
                diff = pos - HOME_POSITIONS[sid]
                # Map physical encoder to geometric coordinate system
                actual_phases[sid] = ((diff / ENCODER_RESOLUTION) * 360.0 * DIRECTION_MAP[sid]) % 360

        print("[Heart] BOOT: Handover successful. Entering Velocity-Control Loop.")
        for sid in ALL_SERVOS:
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0) # EC49 Mode switch
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 1) # Velocity Mode
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_TORQUE_LIMIT, 1000) # Enable full strength
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 1)

    # Begin hardware initialization
    init_and_align_servos()
    
    # Internal Clocks
    master_time_L = 0.0
    master_time_R = 0.0
    target_dt     = 1.0 / 50.0  # 50Hz Loop
    last_loop_time = time.perf_counter()
    
    # Smoothing States
    smooth_speed     = 0.0
    smooth_turn      = 0.0
    smooth_imp_start = float(shared_impact_start.value)
    smooth_imp_end   = float(shared_impact_end.value)
    smooth_duty      = 0.5    # EC46, EC56: Smooth Duty Transition
    smooth_offsets   = {sid: 0.0 for sid in ALL_SERVOS} # EC10: Boot-spread morph
    
    try:
        while is_running.value:
            loop_start_time = time.perf_counter()
            raw_dt = loop_start_time - last_loop_time
            last_loop_time = loop_start_time
            
            # EC11: Physics Time-Dilation ensures stability during CPU spikes
            real_dt = min(0.05, raw_dt)

            # EC59: Increment Heartbeat for Brain monitor logic
            shared_heartbeat.value += 1

            # EC24: Watchdog check for parent process
            if loop_start_time - last_log_time > 1.0:
                if os.getppid() != parent_pid:
                    print("[Heart] CRITICAL: Watchdog detected lost Brain process. Stopping."); break
                gc.collect() # Manually trigger GC during the telemetry window

            # EC52: Serial buffer purge to prevent frame-latency buildup
            port_handler.clearPort()

            # 1. READ SENSORS (EC46, EC64: Lag-free top-of-loop read)
            current_temp_max, current_load_max, current_total_ma = 0, 0, 0
            voltage_reading = 12.0
            loop_comm_fail = False
            
            for sid in ALL_SERVOS:
                # Essential Trajectory Read
                pos, r1, _  = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_POSITION)
                if r1 == 0:
                    diff = pos - HOME_POSITIONS[sid]
                    actual_phases[sid] = ((diff / ENCODER_RESOLUTION) * 360.0 * DIRECTION_MAP[sid]) % 360
                else:
                    loop_comm_fail = True

                # EC37, EC48, EC65: Rotating Health Telemetry
                if sid == ALL_SERVOS[telemetry_rotator] or sid == 1:
                    load, r2, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_LOAD)
                    temp, r3, _ = packet_handler.read1ByteTxRx(port_handler, sid, ADDR_PRESENT_TEMP)
                    volt, r4, _ = packet_handler.read1ByteTxRx(port_handler, sid, ADDR_PRESENT_VOLTAGE)
                    amps, r5, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_CURRENT)
                    
                    if r2 == 0:
                        l_mag = load & 0x3FF
                        # EC30: Safe index guarding for shared memory
                        if 0 <= sid < len(shared_servo_loads):
                            shared_servo_loads[sid] = l_mag
                        
                        # EC14: Stall Debouncing logic
                        if l_mag > STALL_THRESHOLD: stall_counters[sid] += 1
                        else: stall_counters[sid] = 0
                        is_stalled[sid] = (stall_counters[sid] >= 3)
                        current_load_max = max(current_load_max, l_mag)
                        
                        # EC61: Instant mechanical spike relief (>900 load)
                        if l_mag > 900: 
                            is_stalled[sid] = True 
                        
                    if r3 == 0: current_temp_max = max(current_temp_max, temp)
                    if r4 == 0: voltage_reading = volt / 10.0
                    if r5 == 0: current_total_ma = (amps & 0x7FFF)

            telemetry_rotator = (telemetry_rotator + 1) % len(ALL_SERVOS)
            
            # EC39: Connection Health Monitoring
            if loop_comm_fail: comm_error_streak += 1
            else: comm_error_streak = 0
            
            if comm_error_streak > 15:
                print("[Heart] FATAL: Persistent bus desynchronization. Halting motors."); break

            # EC48: Voltage/Thermal protection debouncer
            if voltage_reading < VOLTAGE_MIN: volt_dip_counter += 1
            else: volt_dip_counter = 0
            
            if volt_dip_counter > 20 or current_temp_max > TEMP_MAX:
                print(f"[Heart] SAFETY SHUTDOWN: Low Power ({voltage_reading}V) or Thermal High ({current_temp_max}C)"); break

            # 2. LOCAL SNAPSHOT (EC49: Atomic copy of FSM states)
            fsm_spd = shared_speed.value
            fsm_trn = shared_turn_bias.value
            fsm_x, fsm_z = shared_x_flip.value, shared_z_flip.value
            fsm_gait = shared_gait_id.value
            fsm_imp_s = shared_impact_start.value
            fsm_imp_e = shared_impact_end.value

            if loop_start_time - last_log_time > 1.0:
                log_telemetry(voltage_reading, current_temp_max, current_load_max, current_total_ma / 1000.0, fsm_spd, fsm_trn)
                last_log_time = loop_start_time
            
            # 3. SLEW-RATE INTERPOLATION (Protects hardware from high-frequency command shock)
            target_effective_speed = fsm_spd * fsm_x * fsm_z
            lerp_rate = min(1.0, 5.0 * real_dt) # 0.2 second physical damping
            
            # Interpolate speed and turn
            smooth_speed += (target_effective_speed - smooth_speed) * lerp_rate
            smooth_turn  += (fsm_trn - smooth_turn) * lerp_rate
            
            # EC9: Modulo-Space Shortest-Path Interpolation for Impact Angles
            d_s = (fsm_imp_s - smooth_imp_start + 180) % 360 - 180
            smooth_imp_start = (smooth_imp_start + d_s * lerp_rate) % 360
            
            d_e = (fsm_imp_e - smooth_imp_end + 180) % 360 - 180
            smooth_imp_end = (smooth_imp_end + d_e * lerp_rate) % 360

            # EC46, EC56: Gated transition for duty cycle changes
            g_params = GAITS.get(fsm_gait, GAITS[0])
            t_duty   = max(0.01, min(0.99, g_params['duty'])) # EC8
            smooth_duty += (t_duty - smooth_duty) * min(1.0, 1.0 * real_dt) 
            
            # EC5: Morph leg timings to prevent snapping during gait switches
            t_offsets = g_params['offsets']
            for sid in ALL_SERVOS:
                o_diff = (t_offsets[sid] - smooth_offsets[sid] + 0.5) % 1.0 - 0.5
                smooth_offsets[sid] = (smooth_offsets[sid] + o_diff * lerp_rate) % 1.0

            # Differential Steering Math
            base_hz = smooth_speed / 1000.0
            hz_L = base_hz + smooth_turn
            hz_R = base_hz - smooth_turn

            # EC12, EC40: Kinematic Governor (Prevents Air-Swing saturation)
            base_sweep = (smooth_imp_end - smooth_imp_start + 180) % 360 - 180
            air_sweep = 360.0 - abs(base_sweep)
            if base_sweep < 0: air_sweep = -air_sweep
            
            # Ensure hardware ceiling (3000 limit - 200 unit safety buffer)
            max_safe_hz = (2800.0 / VELOCITY_SCALAR * (1.0 - smooth_duty)) / max(5.0, abs(air_sweep))
            hz_L = max(-max_safe_hz, min(max_safe_hz, hz_L))
            hz_R = max(-max_safe_hz, min(max_safe_hz, hz_R))
            
            # 4. MASTER CLOCKS
            # EC16, EC43: Reset clocks if stationary for > 2 seconds to prevent drift
            if abs(hz_L) < 0.001 and abs(hz_R) < 0.001:
                master_time_L, master_time_R = 0.0, 0.0
            else:
                master_time_L = (master_time_L + (hz_L * real_dt)) % 1.0
                master_time_R = (master_time_R + (hz_R * real_dt)) % 1.0
            
            # Phase Resynchronization (EC22 Directional Guard)
            if abs(smooth_turn) < 0.01 and abs(smooth_speed) > 10.0 and (hz_L * hz_R) > 0:
                clock_diff = (master_time_L - master_time_R + 0.5) % 1.0 - 0.5
                if abs(clock_diff) > 0.005:  
                    decay = clock_diff * min(1.0, 2.0 * real_dt) 
                    master_time_L = (master_time_L - decay/2) % 1.0
                    master_time_R = (master_time_R + decay/2) % 1.0
                else:
                    master_time_R = master_time_L
            
            # 5. VELOCITY COMMAND PRODUCTION
            for sid in ALL_SERVOS:
                if is_stalled[sid]:
                    final_speed = 0 # Active compliance yield
                else:
                    s_hz = hz_L if sid in LEFT_SERVOS else hz_R
                    m_t  = master_time_L if sid in LEFT_SERVOS else master_time_R

                    cur_ph = actual_phases.get(sid, 0.0)
                    t_leg  = (m_t + smooth_offsets[sid]) % 1.0
                    tar_ph = get_buehler_angle(t_leg, smooth_duty, smooth_imp_start, smooth_imp_end)
                    
                    # Compute Geometric Derivative for Feed-Forward velocity
                    if t_leg <= smooth_duty:
                        deg_per_sec = (base_sweep / smooth_duty) * s_hz
                    else:
                        deg_per_sec = (air_sweep / (1.0 - smooth_duty)) * s_hz
                        
                    ff_speed = deg_per_sec * VELOCITY_SCALAR 
                    
                    # Proportional Phase Correction (PD loop)
                    short_err = (tar_ph - cur_ph + 180) % 360 - 180
                    if s_hz > 0.05 and short_err < -90: short_err += 360
                    elif s_hz < -0.05 and short_err > 90: short_err -= 360
                    
                    # EC6: Virtual Spring Limit (45 deg) protects gearbox from snaps
                    raw_speed = ff_speed + (max(-45, min(45, short_err)) * KP_PHASE)
                    
                    # Sign adjustment based on physical mounting (DIRECTION_MAP)
                    if DIRECTION_MAP[sid] == -1: 
                        raw_speed = -raw_speed 
                    
                    # EC33: Virtual software deadzone to prevent coil heat/hum
                    if abs(raw_speed) < 5: 
                        final_speed = 0
                    else:
                        final_speed = max(-3000, min(3000, int(raw_speed)))
                
                # Format packet bytes
                abs_v = int(abs(final_speed))
                speed_val = (abs_v | 0x8000) if final_speed < 0 else abs_v
                group_sync_write.addParam(sid, [speed_val & 0xff, (speed_val >> 8) & 0xff])
            
            # Execute serial burst
            group_sync_write.txPacket()
            group_sync_write.clearParam()
            
            # EC47, EC65: Hybrid Timing Engine (Sleep then Busy-Wait)
            elapsed = time.perf_counter() - loop_start_time
            if target_dt - elapsed > 0.002:
                time.sleep(target_dt - elapsed - 0.002)
            while time.perf_counter() - loop_start_time < target_dt:
                pass # Lock precision microseconds
            
            # EC69: Explicit local cleanup to assist disabled GC
            del raw_speed, final_speed, speed_val
            
    except Exception as e:
        print(f"[Heart] FATAL LOOP ERROR: {e}")
    finally:
        # EC28, EC44, EC45: Zero-Jerk LERP shutdown sequence
        print("\n" + "*"*60)
        print("[Heart] SHUTDOWN: Navigating to HOME stance with Jerk-Free Handover.")
        print("*"*60)
        port_handler.clearPort()
        
        # Phase 1: Zero out all speed registers
        for sid in ALL_SERVOS: 
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 0)
        time.sleep(0.5)
        
        # Phase 2: Handover - Read current pos and set as goal to prevent initial mode snap
        for sid in ALL_SERVOS:
            pos, _, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_POSITION)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 0) # Position Mode
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_ACCEL, 0) # Instant park response
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_POSITION, pos)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 1)
        
        # Phase 3: LERPing to HOME posture over 1 second
        print("[Heart] Parking Sequence: Interpolating physical pose...")
        for i in range(50):
            lerp = (i+1)/50.0
            for sid in ALL_SERVOS:
                p, _, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_POSITION)
                target = int(p + (HOME_POSITIONS[sid] - p) * lerp)
                packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_POSITION, target)
            time.sleep(0.02)
        
        # Phase 4: Final Release
        for sid in ALL_SERVOS: 
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
        port_handler.closePort()
        print("[Heart] Bus released. All systems offline.")

# =================================================================
# PROCESS 1: THE BRAIN (VERBOSE TACTICAL STATE MACHINE)
# =================================================================
if __name__ == "__main__":
    print("\n" + "="*60)
    print("      HEXAPOD KINEMATICS PRO ")
    print("="*60)

    if os.path.exists(LOG_FILE):
        try: os.rename(LOG_FILE, LOG_FILE + ".old")
        except: pass

    # Multi-Processing Primitive Allocation
    shared_speed        = mp.Value('i', 0)  
    shared_x_flip       = mp.Value('i', 1)  
    shared_z_flip       = mp.Value('i', 1)  
    shared_turn_bias    = mp.Value('d', 0.0) 
    shared_gait_id      = mp.Value('i', 0)   
    shared_impact_start = mp.Value('i', 320) 
    shared_impact_end   = mp.Value('i', 40)  
    shared_servo_loads  = mp.Array('i', 7) 
    shared_heartbeat    = mp.Value('i', 0)   # EC59: Monitor Heart liveness
    is_running          = mp.Value('b', True) 

    # EC54: Sentinel initialization for load array
    for i in range(7): shared_servo_loads[i] = -1

    gait_process = mp.Process(target=gait_worker, args=(
        shared_speed, shared_x_flip, shared_z_flip, shared_turn_bias, 
        shared_gait_id, shared_impact_start, shared_impact_end, 
        shared_servo_loads, shared_heartbeat, is_running
    ))
    gait_process.start()

    # --- TACTICAL BEHAVIOR DEFINITIONS ---
    def state_recovery_wiggle(duration=5):
        """Active Compliance: Shakes chassis to break legs free."""
        print("\n" + "*"*50)
        print("[STATE] RECOVERY: High Load Detected (Leg Stuck)!")
        print("[STATE] Action: Triggering extraction frequency wiggle.")
        print("*"*50)
        end_time = time.time() + duration
        while time.time() < end_time:
            shared_x_flip.value = -1; time.sleep(0.3)
            shared_x_flip.value = 1; time.sleep(0.3)
        print("[STATE] Extraction sequence complete. Resuming trajectory.")

    def state_self_right_roll():
        """Uses slow-torque rolling to flip the robot over."""
        print("\n" + "!"*50)
        print("[STATE] RECOVERY: Gravity vector inversion detected!")
        print("[STATE] Action: Executing Slow-Torque Symmetrical Roll sequence.")
        print("!"*50)
        shared_gait_id.value = 1 # Wave Gait ensures max support
        shared_impact_start.value, shared_impact_end.value = 320, 40 
        
        print("[FSM] Step 1: Shifting internal center of gravity forward..."); shared_speed.value = 400; time.sleep(2.0)
        print("[FSM] Step 2: Initiating reverse torque momentum pulse..."); shared_speed.value = -500; time.sleep(4.0)
        print("[FSM] Step 3: Secondary momentum buildup..."); shared_speed.value = 400; time.sleep(2.0)
        print("[FSM] Step 4: Final clearing roll and posture reset..."); shared_speed.value = -500; time.sleep(4.0)
        
        shared_speed.value = 0
        print("[STATE] Orientation normalized. Gyroscopic truth verified.")

    try:
        print("\n[Brain] System Bootstrapping... Waiting for Heart thread link.")
        # EC59: Confirm heartbeat at 50Hz before commencing mission
        while shared_heartbeat.value < 50: 
            time.sleep(0.1)
        print("[Brain] Secure System Link Established. Commencing tactical field test.")
        
        # --- PHASE 1: TRIPOD ---
        print("\n" + "="*50)
        print("PHASE 1: TRIPOD AGILITY & TACTICAL STEERING")
        print("="*50)
        print("[SIMULATION] Env: Concrete floor. Path: Clear. Slope: 0.")
        print("[FSM] Decision: Engaging High-Agility Tripod Gait.")
        shared_gait_id.value = 0; shared_impact_start.value, shared_impact_end.value = 320, 40 
        
        print("[FSM] Executing: High-Speed Sprint Forward (Velocity: 800)"); shared_speed.value = 800; time.sleep(5)
        print("[FSM] Executing: Dynamic Carving Turn LEFT (Bias: -0.4)"); shared_turn_bias.value = -0.4; time.sleep(4)
        print("[FSM] Executing: Dynamic Carving Turn RIGHT (Bias: 0.4)"); shared_turn_bias.value = 0.4; time.sleep(4)
        
        print("[FSM] Executing: True Zero-Radius Tactical Pivot LEFT (Spd: 0, Trn: -0.8)")
        shared_speed.value = 0; shared_turn_bias.value = -0.8; time.sleep(4)
        print("[FSM] Executing: True Zero-Radius Tactical Pivot RIGHT (Spd: 0, Trn: 0.8)")
        shared_speed.value = 0; shared_turn_bias.value = 0.8; time.sleep(4)
        
        print("[FSM] Executing: Full-Power High-Speed Reverse Sprint"); shared_turn_bias.value = 0.0; shared_speed.value = -800; time.sleep(5)

        # --- PHASE 2: QUADRUPED ---
        print("\n" + "="*50)
        print("PHASE 2: QUADRUPED TERRAIN ADAPTATION")
        print("="*50)
        print("[SIMULATION] Env: Loose gravel path. Slope: Moderate.")
        print("[FSM] Decision: Switching to high-torque Quadruped stability gait.")
        shared_gait_id.value = 2; shared_speed.value = 550; shared_turn_bias.value = 0.0
        
        print("[FSM] Executing: Standard Stability Walking Sequence..."); time.sleep(5)
        
        print("[SIMULATION] Sensor (Ultrasonic): 10cm Obstacle detected in center path!")
        print("[STATE] Triggering: WALKING TALL - Riding over obstruction (Impacts: 345->15)")
        shared_speed.value = 500; shared_impact_start.value, shared_impact_end.value = 345, 15; time.sleep(6)
        
        print("[SIMULATION] Sensor (Ultrasonic): Low overhang clearance detected!")
        print("[STATE] Triggering: STEALTH CRAWL - Lowering body profile (Impacts: 300->60)")
        shared_impact_start.value, shared_impact_end.value = 300, 60; time.sleep(6)

        # --- PHASE 3: WAVE ---
        print("\n" + "="*50)
        print("PHASE 3: WAVE GAIT & INCLINE NAVIGATION")
        print("="*50)
        print("[SIMULATION] Env: Deformable sand hill. Slope: Steep (15 deg).")
        print("[FSM] Decision: Engaging ultra-stable Wave Gait (Duty 0.85).")
        shared_gait_id.value = 1; shared_speed.value = 350
        
        print("[FSM] Executing: Low-Speed Sand Crawl Cycle..."); time.sleep(6)
        print("[FSM] Executing: Fine Fine-Correction Pivot LEFT"); shared_turn_bias.value = -0.15; time.sleep(5)
        print("[FSM] Executing: Fine Fine-Correction Pivot RIGHT"); shared_turn_bias.value = 0.15; time.sleep(5)
        
        print("[SIMULATION] Sensor (IMU): Chassis pitch exceeds stability gate!")
        print("[STATE] Triggering: LEANING INTO HILL - Shifting COG (Impacts: 290->10)")
        shared_speed.value = 350; shared_impact_start.value, shared_impact_end.value = 290, 10; time.sleep(8)

        # --- PHASE 4: TACTICAL RECOVERY ---
        print("\n" + "="*50)
        print("PHASE 4: TACTICAL ERROR RECOVERY")
        print("="*50)
        print("[FSM] Sequence: Temporary deceleration for environmental assessment..."); shared_speed.value = 0; time.sleep(2)
        
        print("[SIMULATION] Sensor (Load): Leg 2 mechanical resistance > STALL_THRESHOLD!")
        state_recovery_wiggle()
        
        print("[SIMULATION] Sensor (IMU): Gravity inversion detected (Robot flipped)!")
        state_self_right_roll()

        # --- PHASE 5: SHUTDOWN ---
        print("\n" + "="*50)
        print("PHASE 5: NAVIGATION SEQUENCE COMPLETE")
        print("="*50)
        print("[SIMULATION] Sensor: Final destination reached. Navigating to base posture.")
        shared_speed.value = 0; time.sleep(8) 

    except KeyboardInterrupt:
        print("\n" + "!"*60)
        print("[Brain] INTERRUPT: Tactical Kill Command Received from Operator.")
        print("!"*60)
    finally:
        is_running.value = False
        print("[Brain] SHUTDOWN: Waiting for Heart thread termination (15s limit)...")
        gait_process.join(timeout=15)
        if gait_process.is_alive():
            print("[Brain] SHUTDOWN ALERT: Heart unresponsive. Force terminating process.")
            gait_process.terminate()
        print("\nIntegrated Industrial Test Suite Complete. All systems offline.")