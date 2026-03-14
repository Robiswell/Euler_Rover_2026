#!/usr/bin/env python3
"""
Hexapod rover kinematics - STS3215 servos, Raspberry Pi 3B+

Two processes:
  Brain  - mission sequencer, runs gait phases and recovery behaviors
  Heart  - 50Hz kinematics loop, Buehler clock math + LERP smoothing

Hardware: 6x STS3215 12V servos on /dev/ttyUSB0 @ 1Mbaud
No software angle offsets - clearance is managed physically.
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
    from scservo_sdk import (PortHandler, PacketHandler,
                             GroupSyncWrite, GroupSyncRead)
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
ADDR_MODE             = 33
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
LEN_PRESENT_POSITION  = 2
LEN_PRESENT_LOAD      = 2
LEN_PRESENT_TEMP      = 1
LEN_PRESENT_VOLTAGE   = 1
LEN_PRESENT_CURRENT   = 2

# Robot Physical Constants
ENCODER_RESOLUTION = 4096.0
VELOCITY_SCALAR    = 1.85
LEFT_SERVOS        = [2, 3, 4]
RIGHT_SERVOS       = [1, 6, 5]
ALL_SERVOS         = LEFT_SERVOS + RIGHT_SERVOS

# Industrial Safety Parameters
TEMP_MAX     = 65  # lowered from 70 — altitude reduces convective cooling ~25%
VOLTAGE_MIN  = 10.5  # 10.5V = 3.5V/cell floor — raised from 10.0 for cold-weather cell protection (no hardware BMS)
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

# Servo ID -> shared_servo_loads array index (0-based, clean mapping)
SERVO_LOAD_INDEX = {sid: i for i, sid in enumerate(ALL_SERVOS)}

# Feedback Gains
KP_PHASE        = 12.0
STALL_THRESHOLD = 750  # raised from 600 for wet sand operation — tune from telemetry after first run
GHOST_TEMP      = 125  # STS3215 EMI artifact - bus noise returns flat 125°C (not a real reading)

# --- KINEMATIC GAIT DICTIONARIES ---
GAITS = {
    0: {  # TRIPOD
        'duty': 0.5,
        'offsets': {2: 0.0, 6: 0.0, 4: 0.0,  1: 0.5, 3: 0.5, 5: 0.5}
    },
    1: {  # WAVE
        'duty': 0.85,
        # Offsets are spaced 1/6 ≈ 0.167 apart so exactly one leg is in air
        # at any moment (air window = 0.15 cycles each, no overlap possible).
        # servo 5 was previously 0.380 (typo/drift from 0.333), which caused
        # a 0.030-cycle overlap with servo 2's air window — two legs airborne
        # simultaneously at T ≈ 0.47-0.50, violating the wave gait contract.
        'offsets': {4: 0.833, 3: 0.666, 2: 0.5,  5: 0.333, 6: 0.166, 1: 0.0}
    },
    2: {  # QUADRUPED
        'duty': 0.7,
        'offsets': {2: 0.0, 5: 0.0,  3: 0.333, 6: 0.333,  4: 0.666, 1: 0.666}
    }
}

# -----------------------------------------------------------------
# KINEMATIC CORE HELPERS
# -----------------------------------------------------------------
def get_air_sweep(stance_sweep):
    """
    Authoritative air-phase sweep calculation.
    Extracted into a single helper to avoid duplicated sign logic.
    """
    air = 360.0 - abs(stance_sweep)
    return -air if stance_sweep < 0 else air


def get_buehler_angle(t_norm, duty_cycle, start_ang, end_ang, is_reversed):
    """
    Pure geometric mapping for stutter-free 360-degree rotation.
    Handles stance/air phases with magnitude-based feed-forward logic.
    """
    stance_sweep = (end_ang - start_ang + 180) % 360 - 180

    if is_reversed:
        start_ang, end_ang = end_ang, start_ang
        stance_sweep = -stance_sweep

    if t_norm <= duty_cycle:
        progress = t_norm / duty_cycle
        angle = start_ang + (stance_sweep * progress)
    else:
        progress = (t_norm - duty_cycle) / (1.0 - duty_cycle)
        air_sweep = get_air_sweep(stance_sweep)
        angle = end_ang + (air_sweep * progress)

    return angle % 360


# =================================================================
# PROCESS 2: THE HEART (REAL-TIME ENGINE)
# =================================================================
def gait_worker(shared_speed, shared_x_flip, shared_z_flip, shared_turn_bias, shared_gait_id,
                shared_impact_start, shared_impact_end, shared_servo_loads, shared_heartbeat,
                shared_stall_override, is_running):
    """
    50Hz kinematics loop. Runs as a separate process.
    GroupSyncRead for position and load every tick. Temp/voltage/current
    rotate one servo per tick to avoid unnecessary bus traffic.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        os.nice(-20)
    except:
        pass

    # Drain heap during boot before disabling GC for the live loop
    gc.collect()
    gc.disable()

    port_handler   = PortHandler(PORT_NAME)
    packet_handler = PacketHandler(SERVO_PROTOCOL)

    # Sync write: goal speeds (sent every loop)
    group_sync_write = GroupSyncWrite(
        port_handler, packet_handler, ADDR_GOAL_SPEED, LEN_GOAL_SPEED)

    # Sync read: present position (every loop)
    gsread_pos = GroupSyncRead(
        port_handler, packet_handler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

    # Sync read: load - all servos every tick (required for stall detection)
    gsread_load = GroupSyncRead(port_handler, packet_handler, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD)

    # Temp / voltage / current use individual reads on the single rotating telemetry_sid.
    # GroupSyncRead for these would register all 6 servos and fire a 6-servo transaction
    # every tick, tripling bus traffic for registers we only need from one servo per tick.

    port_handler.setPacketTimeoutMillis(10)

    # Internal Tracking
    is_stalled     = {sid: False for sid in ALL_SERVOS}
    stall_counters = {sid: 0     for sid in ALL_SERVOS}
    last_actual_phases = {sid: 0.0 for sid in ALL_SERVOS}

    telemetry_sid   = ALL_SERVOS[0]   # Which single servo gets full telemetry this tick
    telemetry_index = 0
    voltage_reading = 12.0
    volt_dip_counter   = 0
    temp_spike_counter = 0
    temp_per_servo     = {sid: 0 for sid in ALL_SERVOS}   # Last known temp per servo (°C)
    current_per_servo  = {sid: 0 for sid in ALL_SERVOS}   # Last known current per servo (raw mA units)
    parent_pid     = os.getppid()
    last_log_time  = 0

    GAIT_NAMES = {0: "tripod", 1: "wave", 2: "quad"}

    def log_telemetry(volt, total_amps, fsm_speed, fsm_turn,
                      volt_dip_ctr, temp_spike_ctr, loop_ms, ghost_event):
        """Extended telemetry log — per-servo loads, stall IDs, safety counters, loop timing.
        Fields: timestamp, gait, speed, turn | voltage + volt_dip_counter | per-servo temps +
        temp_spike_counter | per-servo loads | stall servo IDs | amps | loop_ms | event tags.
        [GHOST] = 125C EMI artifact rejected this tick.
        [V_WARN:N] = volt_dip_counter > 100 (shutdown triggers at > 150).
        [T_WARN:N] = temp_spike_counter > 10 (shutdown triggers at > 15).
        """
        try:
            if os.path.exists(LOG_FILE) and os.path.getsize(LOG_FILE) > LOG_MAX_SIZE:
                os.rename(LOG_FILE, LOG_FILE + f".{int(time.time())}.old")
            with open(LOG_FILE, "a") as f:
                ts        = datetime.datetime.now().strftime("%H:%M:%S")
                gait_name = GAIT_NAMES.get(shared_gait_id.value, str(shared_gait_id.value))
                # Per-servo temps and loads in ALL_SERVOS order (2,3,4,1,6,5)
                temps_str = ",".join(str(temp_per_servo[sid]) for sid in ALL_SERVOS)
                loads_str = ",".join(str(shared_servo_loads[SERVO_LOAD_INDEX[sid]]) for sid in ALL_SERVOS)
                stall_ids = ",".join(f"s{sid}" for sid in ALL_SERVOS if is_stalled[sid]) or "none"
                ghost_tag = " [GHOST]" if ghost_event else ""
                vwarn_tag = f" [V_WARN:{volt_dip_ctr}]" if volt_dip_ctr > 100 else ""
                twarn_tag = f" [T_WARN:{temp_spike_ctr}]" if temp_spike_ctr > 10 else ""
                f.write(f"[{ts}] G:{gait_name} Spd:{fsm_speed} Trn:{fsm_turn:.2f} | "
                        f"V:{volt:.2f}V Vc:{volt_dip_ctr:03d} | "
                        f"T:{temps_str} Tc:{temp_spike_ctr:02d} | "
                        f"L:{loads_str} | Stall:{stall_ids} | "
                        f"A:{total_amps:.2f} | Loop:{loop_ms:.1f}ms"
                        f"{ghost_tag}{vwarn_tag}{twarn_tag}\n")
        except:
            pass

    def log_stall_event(sid, entering, load):
        """Event-triggered stall log — fires immediately on state transition, not on 1Hz heartbeat.
        Captures brief stall events (as short as 60ms) that the 1Hz log would miss.
        entering=True → stall declared (counter hit 3). entering=False → stall cleared (counter hit 0).
        """
        try:
            with open(LOG_FILE, "a") as f:
                ts    = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                event = "ENTRY" if entering else "EXIT "
                total_stalled = sum(1 for s in is_stalled.values() if s)
                f.write(f"[{ts}] [STALL-{event}] servo=s{sid} load={load} "
                        f"thresh={STALL_THRESHOLD} concurrent_stalls={total_stalled}\n")
        except:
            pass

    def init_and_align_servos():
        """Boot: snap all legs to home in position mode, then switch to velocity mode."""
        while is_running.value:
            if port_handler.openPort() and port_handler.setBaudRate(BAUDRATE):
                break
            time.sleep(1.0)

        port_handler.clearPort()
        print("[heart] snapping legs to home...")
        for sid in ALL_SERVOS:
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 0)  # Position
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_ACCEL, 20)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_TORQUE_LIMIT, 400)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 800)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_POSITION, HOME_POSITIONS[sid])
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 1)

        time.sleep(3.0)

        print("[heart] switching to velocity mode")
        for sid in ALL_SERVOS:
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 1)  # Velocity
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_ACCEL, 0)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_TORQUE_LIMIT, 1000)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 1)

        # Pre-register all servos for position and load sync reads
        for sid in ALL_SERVOS:
            gsread_pos.addParam(sid)
            gsread_load.addParam(sid)

    init_and_align_servos()

    master_time_L, master_time_R = 0.0, 0.0
    target_dt      = 1.0 / 50.0
    last_loop_time = time.perf_counter()

    # State Smoothing (V69)
    smooth_hz        = 0.0
    smooth_turn      = 0.0
    smooth_imp_start = float(shared_impact_start.value)
    smooth_imp_end   = float(shared_impact_end.value)
    smooth_duty      = 0.5
    # Seed offsets from default gait (TRIPOD) so legs are correctly phased
    # from tick 1 - avoids all-6-legs-in-phase condition during cold start.
    smooth_offsets   = dict(GAITS[0]['offsets'])

    prev_loop_ms    = 0.0    # last frame's measured elapsed time — logged in 1Hz heartbeat
    ghost_event_flag = False  # latched True when 125C artifact seen; reset after each log tick

    try:
        while is_running.value:
            loop_start     = time.perf_counter()
            real_dt        = min(0.05, loop_start - last_loop_time)
            last_loop_time = loop_start

            shared_heartbeat.value += 1

            if loop_start - last_log_time > 1.0:
                if os.getppid() != parent_pid:
                    break

            # ----------------------------------------------------------
            # 1. READ PHYSICAL FEEDBACK via GroupSyncRead
            # ----------------------------------------------------------
            # Position read - all servos, every loop
            gsread_pos.txRxPacket()

            actual_phases = {}
            for sid in ALL_SERVOS:
                if gsread_pos.isAvailable(sid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                    pos  = gsread_pos.getData(sid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                    diff = pos - HOME_POSITIONS[sid]
                    angle = ((diff / ENCODER_RESOLUTION) * 360.0 * DIRECTION_MAP[sid]) % 360
                    actual_phases[sid]      = angle
                    last_actual_phases[sid] = angle
                else:
                    actual_phases[sid] = last_actual_phases[sid]

            # Telemetry read - rotate one servo per tick for full telemetry,
            # read all servos for load (needed for stall detection every tick)
            gsread_load.txRxPacket()
            current_load_max  = 0
            stall_active = not shared_stall_override.value
            for sid in ALL_SERVOS:
                if gsread_load.isAvailable(sid, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD):
                    load  = gsread_load.getData(sid, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD)
                    l_mag = load & 0x3FF
                    shared_servo_loads[SERVO_LOAD_INDEX[sid]] = l_mag
                    if stall_active:
                        # Latching stall logic — symmetric 3-frame entry AND exit.
                        # Wet sand produces brief stance-phase load spikes from sand
                        # resistance; a single high frame must NOT freeze the leg.
                        # Entry: counter must reach 3 before stall is declared.
                        # Exit: counter must fall to 0 before stall is cleared.
                        # During exit, a new spike increments the counter back up —
                        # stall re-latches immediately when counter hits 3 again.
                        if l_mag > STALL_THRESHOLD:
                            stall_counters[sid] = min(stall_counters[sid] + 1, 3)
                        else:
                            stall_counters[sid] = max(0, stall_counters[sid] - 1)
                        prev_stalled = is_stalled[sid]
                        if stall_counters[sid] >= 3:
                            is_stalled[sid] = True
                        elif stall_counters[sid] == 0:
                            is_stalled[sid] = False
                        # counter between 1-2: maintain current stall state (no change)
                        if is_stalled[sid] != prev_stalled:
                            log_stall_event(sid, is_stalled[sid], l_mag)
                    else:
                        # Stall override active (e.g. inverted roll): high load is
                        # intentional floor contact - do not cut leg speed.
                        stall_counters[sid] = 0
                        is_stalled[sid]     = False
                    current_load_max = max(current_load_max, l_mag)

            # Rotate which servo supplies temp/voltage/current each tick -
            # individual reads on one servo, not a 6-servo batch transaction.
            telemetry_sid   = ALL_SERVOS[telemetry_index]
            telemetry_index = (telemetry_index + 1) % len(ALL_SERVOS)

            # STS3215 "Ghost Sensor" artifact: bus noise / EMI can cause the servo
            # to return a flat 125°C instead of a real reading. Treat 125 the same
            # as a comms failure - leave spike counter unchanged rather than
            # triggering a false shutdown or masking a real overheat.
            temp_read_ok = False
            current_temp_max = 0
            temp, res_t, _ = packet_handler.read1ByteTxRx(port_handler, telemetry_sid, ADDR_PRESENT_TEMP)
            if res_t == 0 and temp != GHOST_TEMP:
                current_temp_max = temp
                temp_per_servo[telemetry_sid] = temp
                temp_read_ok = True
            elif res_t == 0 and temp == GHOST_TEMP:
                ghost_event_flag = True  # latch — cleared after next 1Hz log tick

            volt, res_v, _ = packet_handler.read1ByteTxRx(port_handler, telemetry_sid, ADDR_PRESENT_VOLTAGE)
            if res_v == 0:
                voltage_reading = volt / 10.0

            amps, res_a, _ = packet_handler.read2ByteTxRx(port_handler, telemetry_sid, ADDR_PRESENT_CURRENT)
            if res_a == 0:
                current_per_servo[telemetry_sid] = amps & 0x7FFF

            # ----------------------------------------------------------
            # Safety Guards
            # ----------------------------------------------------------
            # Only reset spike counter when we have a confirmed safe reading.
            # Failed reads leave the counter unchanged - a string of failures
            # cannot mask a genuine overheat that was already accumulating.
            if temp_read_ok:
                if current_temp_max > TEMP_MAX:
                    temp_spike_counter += 1
                else:
                    temp_spike_counter = 0

            if voltage_reading < VOLTAGE_MIN:
                volt_dip_counter += 1
            else:
                volt_dip_counter  = 0

            if temp_spike_counter > 15 or volt_dip_counter > 150:  # 150 ticks = 3s debounce — extended for cold-weather sag during heavy terrain
                print(f"[heart] safety shutdown - temp={max(temp_per_servo.values())}C volt={voltage_reading:.1f}V")
                log_telemetry(voltage_reading,
                              sum(current_per_servo.values()) / 1000.0,
                              shared_speed.value, shared_turn_bias.value,
                              volt_dip_counter, temp_spike_counter, prev_loop_ms, ghost_event_flag)
                is_running.value = False
                break

            if loop_start - last_log_time > 1.0:
                log_telemetry(voltage_reading,
                              sum(current_per_servo.values()) / 1000.0,
                              shared_speed.value, shared_turn_bias.value,
                              volt_dip_counter, temp_spike_counter, prev_loop_ms, ghost_event_flag)
                ghost_event_flag = False  # reset after logging — new ghost events latch fresh
                last_log_time = loop_start

            # ----------------------------------------------------------
            # 2. STATE LERPING (Software Inertia)
            # ----------------------------------------------------------
            target_hz   = (shared_speed.value * shared_x_flip.value * shared_z_flip.value) / 1000.0
            target_turn = shared_turn_bias.value

            lerp_rate    = min(1.0, 4.0 * real_dt)
            smooth_hz   += (target_hz   - smooth_hz)   * lerp_rate
            smooth_turn += (target_turn - smooth_turn) * lerp_rate

            d_s = (shared_impact_start.value - smooth_imp_start + 180) % 360 - 180
            smooth_imp_start = (smooth_imp_start + d_s * lerp_rate) % 360

            d_e = (shared_impact_end.value - smooth_imp_end + 180) % 360 - 180
            smooth_imp_end = (smooth_imp_end + d_e * lerp_rate) % 360

            gait_params = GAITS.get(shared_gait_id.value, GAITS[0])
            t_duty      = max(0.01, min(0.99, gait_params['duty']))

            # Duty LERP rate matched to hz/turn (was 1.0x, now 4.0x)
            smooth_duty += (t_duty - smooth_duty) * lerp_rate

            t_offsets = gait_params['offsets']
            for sid in ALL_SERVOS:
                o_diff = (t_offsets[sid] - smooth_offsets[sid] + 0.5) % 1.0 - 0.5
                smooth_offsets[sid] = (smooth_offsets[sid] + o_diff * lerp_rate) % 1.0

            # ----------------------------------------------------------
            # 3. DRIVE CALCULATIONS & SAFETY GOVERNOR
            # ----------------------------------------------------------
            hz_L = smooth_hz + smooth_turn
            hz_R = smooth_hz - smooth_turn

            stance_sweep = (smooth_imp_end - smooth_imp_start + 180) % 360 - 180
            air_sweep    = get_air_sweep(stance_sweep)

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
                    master_time_L = (master_time_L - decay / 2) % 1.0
                    master_time_R = (master_time_R + decay / 2) % 1.0
                # else: clocks are within 0.005 cycles - close enough, let decay converge naturally

            # ----------------------------------------------------------
            # 4. KINEMATIC CONTROLLER (Pure Math Flow)
            # ----------------------------------------------------------
            for sid in ALL_SERVOS:
                leg_hz    = hz_L if sid in LEFT_SERVOS else hz_R
                is_rev_leg = (leg_hz < 0)

                if is_stalled[sid] or abs(leg_hz) < 0.001:
                    final_speed = 0
                else:
                    master_time   = master_time_L if sid in LEFT_SERVOS else master_time_R
                    current_phase = actual_phases.get(sid, 0.0)
                    t_leg         = (master_time + smooth_offsets[sid]) % 1.0

                    target_phase = get_buehler_angle(
                        t_leg, smooth_duty, smooth_imp_start, smooth_imp_end, is_rev_leg)

                    # Reuse stance_sweep / air_sweep already computed in section 3
                    if t_leg <= smooth_duty:
                        deg_per_sec = (abs(stance_sweep) * abs(leg_hz)) / smooth_duty
                    else:
                        deg_per_sec = (abs(air_sweep) * abs(leg_hz)) / (1.0 - smooth_duty)

                    ff_speed = deg_per_sec * VELOCITY_SCALAR

                    error       = target_phase - current_phase
                    short_error = (error + 180) % 360 - 180

                    if t_leg > smooth_duty:
                        # Air phase: guard prevents backward-error signal during fast
                        # forward return sweep.  Fires only in air phase to avoid
                        # runaway when the leg overshoots during stance.
                        if not is_rev_leg and short_error < -90: short_error += 360
                        elif is_rev_leg and short_error > 90:   short_error -= 360

                    raw_speed = ff_speed + (short_error * KP_PHASE)

                    if is_rev_leg:            raw_speed = -raw_speed
                    if sid in LEFT_SERVOS:    raw_speed = -raw_speed

                    final_speed = max(-3000, min(3000, int(raw_speed)))

                abs_speed = int(abs(final_speed))
                speed_val = (abs_speed | 0x8000) if final_speed < 0 else abs_speed
                group_sync_write.addParam(sid, [speed_val & 0xff, (speed_val >> 8) & 0xff])

            group_sync_write.txPacket()
            group_sync_write.clearParam()

            # ----------------------------------------------------------
            # 5. PRECISION TIMING
            # ----------------------------------------------------------
            elapsed_time = time.perf_counter() - loop_start
            prev_loop_ms = elapsed_time * 1000  # stored for next 1Hz log tick
            sleep_time   = target_dt - elapsed_time
            if sleep_time > 0.002:
                time.sleep(sleep_time - 0.002)
            while time.perf_counter() - loop_start < target_dt:
                pass

    except Exception as e:
        print(f"[heart] fatal error: {e}")
    finally:
        print("[heart] shutting down, returning legs to home...")
        port_handler.clearPort()
        for sid in ALL_SERVOS:
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 0)
        time.sleep(0.1)
        for sid in ALL_SERVOS:
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_ACCEL, 20)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 1)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 800)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_POSITION, HOME_POSITIONS[sid])
        time.sleep(2.0)
        for sid in ALL_SERVOS:
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
        port_handler.closePort()
        print("[heart] offline")


# =================================================================
# PROCESS 1: THE BRAIN (MISSION SEQUENCER)
# =================================================================

class EmergencyStopException(Exception):
    pass


def tactical_sleep(duration, running_flag, heart_process):
    """
    Monitors is_running flag AND gait_process.is_alive() during tactical
    phases so a Heart crash triggers a clean Brain abort.
    """
    start_time = time.time()
    while time.time() - start_time < duration:
        if not running_flag.value:
            raise EmergencyStopException("Safety shutdown triggered by hardware.")
        if not heart_process.is_alive():
            raise EmergencyStopException("Heart process has died unexpectedly.")
        time.sleep(0.1)


if __name__ == "__main__":
    print("hexapod starting up...")

    # Multi-Processing Primitives
    shared_speed        = mp.Value('i', 0)
    shared_x_flip       = mp.Value('i', 1)
    shared_z_flip       = mp.Value('i', 1)   # NOTE: never written after init - always 1.
                                              #
                                              # Intended for capsized mode in state_self_right_roll.
                                              # Setting -1 would flip effective forward/backward for
                                              # all legs, to correct for the chassis axis reversing
                                              # when upside down.
                                              #
                                              # Unresolved because the correct value can only be
                                              # determined by physically running Step 1 of the roll
                                              # (1-2s) on a capsized robot and observing which way
                                              # the chassis shifts:
                                              #   - Shifts in the useful roll direction → keep 1
                                              #   - Shifts the wrong way → set -1 at the top of
                                              #     state_self_right_roll and reset to 1 in finally
    shared_turn_bias    = mp.Value('f', 0.0)  # c_float (4B) — atomic on ARMv7; c_double (8B) was non-atomic
    shared_gait_id      = mp.Value('i', 0)
    shared_impact_start = mp.Value('i', 320)
    shared_impact_end   = mp.Value('i', 40)
    shared_servo_loads  = mp.Array('i', len(ALL_SERVOS))  # Clean 0-5 indexing
    shared_heartbeat    = mp.Value('i', 0)
    shared_stall_override = mp.Value('b', False)  # When True, stall detection is suppressed.
                                                   # Used during inverted roll to allow legs to
                                                   # push hard against floor without being cut.
    is_running          = mp.Value('b', True)

    for i in range(len(ALL_SERVOS)):
        shared_servo_loads[i] = -1

    gait_process = mp.Process(target=gait_worker, args=(
        shared_speed, shared_x_flip, shared_z_flip, shared_turn_bias,
        shared_gait_id, shared_impact_start, shared_impact_end,
        shared_servo_loads, shared_heartbeat, shared_stall_override, is_running
    ))
    gait_process.start()

    # Convenience wrapper so tactical calls don't repeat the process arg
    def tsleep(duration):
        tactical_sleep(duration, is_running, gait_process)

    # --- EXPANDED TACTICAL RECOVERY BEHAVIORS ---
    def state_recovery_wiggle(duration=10):
        """Oscillate direction rapidly to break static friction / unstick a jammed leg."""
        print("[recovery] wiggling to break friction...")

        # Reset to standard window - avoids inheriting stale state (e.g. COG Shift 315/15)
        shared_impact_start.value, shared_impact_end.value = 330, 30

        # Phase 4 zeros speed before calling this, so we own the speed here
        shared_speed.value = 350

        end_time    = time.time() + duration
        try:
            while time.time() < end_time:
                if not is_running.value:
                    raise EmergencyStopException("Hardware aborted.")
                if not gait_process.is_alive():
                    raise EmergencyStopException("Heart process died during recovery wiggle.")

                shared_x_flip.value = -1
                tsleep(0.3)
                shared_x_flip.value = 1
                tsleep(0.3)
        finally:
            shared_speed.value  = 0
            shared_x_flip.value = 1

        print("[recovery] wiggle done")

    def stall_tsleep(duration, wiggle_threshold=4):
        """
        Break long tactical sleeps into 3-second chunks.
        After each chunk, count legs with load > STALL_THRESHOLD.
        If >= wiggle_threshold legs are stalled, fire an immediate recovery
        wiggle then resume at the same speed.  Closes the 89-second gap
        before Phase 4 on deep-sand entrapment.
        """
        chunk = 3.0
        slept = 0.0
        while slept < duration:
            tsleep(min(chunk, duration - slept))
            slept += chunk
            if slept < duration:  # more remaining — check stalls
                stalled = sum(
                    1 for i in range(len(ALL_SERVOS))
                    if shared_servo_loads[i] > STALL_THRESHOLD
                )
                if stalled >= wiggle_threshold:
                    print(f"[brain] stall_tsleep: {stalled} legs stalled — early wiggle")
                    saved_speed = shared_speed.value
                    state_recovery_wiggle()
                    shared_speed.value = saved_speed

    def state_self_right_roll():
        """Momentum roll to flip robot upright when capsized.

        !! UNVALIDATED - never tested on hardware, but geometrically grounded !!

        Geometry (from design specs):
          Leg radius = 75mm, arc = 210°. Chassis 510x280x75mm.
          Servos were designed for 90mm chassis at centre = 45mm from each face.
          15mm cut from TOP only → servos now 45mm from bottom, 30mm from top.

          Upright:  servo axis 45mm above ground, leg reach 75mm → 30mm clearance
          Inverted: rests on top face, servo axis 30mm up, leg reach 75mm → 45mm clearance
          The 15mm extra clearance inverted is a direct result of only cutting the top.

          Home (0°) = legs straight down upright = straight UP when inverted.
          Legs must be at ~180° to reach the floor when inverted.
          Normal walking window (320/40) sweeps ±40° around 0° - no floor contact inverted.
          Corrected inverted window: 140/220 (±40° around 180°). 210° arc includes this. ✓

        Open questions (needs physical testing):
          - Does +400 shift weight in the useful direction when inverted?
          - Is 4s/8s/4s the right timing for a 3kg robot?
          - z_flip strategy (Scenario A vs B) - see shared_z_flip comment.

        First test: run Step 1 only (1-2s) inverted on flat surface.
          Confirm legs hit floor and chassis shifts, then tune from there.
        """
        print("[recovery] upside-down - attempting momentum roll to self-right")

        ROLL_LOAD_SPEED   =  400   # step 1 - direction unverified inverted
        ROLL_SNAP_SPEED   = -600   # step 2 - direction unverified inverted
        ROLL_SETTLE_SPEED =  400   # step 3 - timing unverified

        # Corrected window centered on 180° (floor-facing when inverted).
        # Old value (320/40) was centered on 0° - legs pointing at ceiling, no contact.
        ROLL_IMPACT_START = 140
        ROLL_IMPACT_END   = 220

        shared_gait_id.value   = 1    # wave gait - maximum sequential torque
        shared_turn_bias.value = 0.0
        shared_impact_start.value, shared_impact_end.value = ROLL_IMPACT_START, ROLL_IMPACT_END

        # Suppress stall detection - floor contact produces high load intentionally.
        # Without this the legs get cut every 60ms and generate no sustained momentum.
        shared_stall_override.value = True
        shared_z_flip.value         = -1    # invert gait direction: legs sweep toward floor when inverted
        try:
            # Wait for LERP to converge on the new window before engaging speed.
            # 330/30 → 140/220 crosses 0° on both endpoints, briefly collapsing
            # stance_sweep to ~0° and zeroing feed-forward. 1s = ~98% convergence.
            tsleep(1.0)

            # Timing extended from 4/8/4 to 5/10/5 for wet sand (Alamosa CO competition)
            print("[recovery] roll step 1 - loading weight")
            shared_speed.value = ROLL_LOAD_SPEED
            tsleep(5.0)

            print("[recovery] roll step 2 - inertial snap")
            shared_speed.value = ROLL_SNAP_SPEED
            tsleep(10.0)  # step 2: inertial snap — wet sand requires longer push

            print("[recovery] roll step 3 - settling")
            shared_speed.value = ROLL_SETTLE_SPEED
            tsleep(5.0)
        finally:
            shared_speed.value          = 0
            shared_z_flip.value         = 1     # restore normal orientation
            shared_stall_override.value = False

        print("[recovery] roll complete")

    try:
        print("[brain] waiting for heart...")
        # Use != 0 sentinel rather than < 50 threshold: a signed 32-bit counter
        # overflows to negative after ~497 days at 50 Hz, making the < 50 check
        # pass immediately on the next boot and starting the mission before Heart
        # has actually initialised.  Any nonzero value means Heart has ticked at
        # least once — sufficient proof of life for the boot gate.
        while shared_heartbeat.value == 0:
            if not gait_process.is_alive():
                raise EmergencyStopException("Heart process died before boot heartbeat.")
            time.sleep(0.1)
        print("[brain] heart is running, starting mission")

        # Speeds reduced from test values for wet-sand obstacle navigation (Alamosa CO, April)
        # =========================================================
        # PHASE 1: TRIPOD
        # =========================================================
        print("\n-- phase 1: tripod --")
        shared_gait_id.value = 0
        shared_impact_start.value, shared_impact_end.value = 330, 30

        print("forward");                    shared_speed.value = 900;                           stall_tsleep(12)
        print("carve left");                 shared_turn_bias.value = -0.3;                      stall_tsleep(10)
        print("carve right");                shared_turn_bias.value =  0.3;                      stall_tsleep(10)
        print("pivot left");                 shared_speed.value = 0; shared_turn_bias.value = -0.8; stall_tsleep(10)
        print("pivot right");                shared_speed.value = 0; shared_turn_bias.value =  0.8; stall_tsleep(10)
        print("reverse");                    shared_turn_bias.value = 0.0; shared_speed.value = -900; stall_tsleep(12)

        # Decel pause - smooth_hz bleeds ~720ms from -1.2Hz without this,
        # meaning the robot reverses briefly after Phase 2 starts.
        shared_speed.value = 0; tsleep(2)

        # =========================================================
        # PHASE 2: QUADRUPED
        # =========================================================
        print("\n-- phase 2: quadruped --")
        shared_gait_id.value = 2
        shared_impact_start.value, shared_impact_end.value = 330, 30

        print("forward");                    shared_speed.value = 400; shared_turn_bias.value = 0.0; stall_tsleep(12)
        print("carve left");                 shared_turn_bias.value = -0.25;                    stall_tsleep(10)
        print("carve right");                shared_turn_bias.value =  0.25;                    stall_tsleep(10)
        print("pivot left");                 shared_speed.value = 0; shared_turn_bias.value = -0.6; stall_tsleep(10)
        print("pivot right");                shared_speed.value = 0; shared_turn_bias.value =  0.6; stall_tsleep(10)
        print("reverse");                    shared_turn_bias.value = 0.0; shared_speed.value = -400; stall_tsleep(12)

        # Decel before walking tall - avoids ~720ms of backward motion bleed into forward mode
        shared_speed.value = 0; tsleep(2)

        print("walking tall");               shared_speed.value = 380; shared_impact_start.value, shared_impact_end.value = 345, 15; stall_tsleep(15)
        print("stealth crawl");              shared_impact_start.value, shared_impact_end.value = 325, 35; stall_tsleep(15)

        # =========================================================
        # PHASE 3: WAVE
        # =========================================================
        print("\n-- phase 3: wave --")
        shared_gait_id.value = 1
        shared_impact_start.value, shared_impact_end.value = 330, 30

        print("forward");                    shared_speed.value = 280; shared_turn_bias.value = 0.0; stall_tsleep(12)
        print("carve left");                 shared_turn_bias.value = -0.2;                     stall_tsleep(10)
        print("carve right");                shared_turn_bias.value =  0.2;                     stall_tsleep(10)
        print("pivot left");                 shared_speed.value = 0; shared_turn_bias.value = -0.5; stall_tsleep(10)
        print("pivot right");                shared_speed.value = 0; shared_turn_bias.value =  0.5; stall_tsleep(10)
        print("reverse");                    shared_turn_bias.value = 0.0; shared_speed.value = -280; stall_tsleep(12)

        # Decel pause - matches Phase 1/2 pattern, bleeds ~720ms of reverse motion before COG shift
        shared_speed.value = 0; tsleep(2)

        print("cog shift (hill climb)");     shared_speed.value = 280; shared_impact_start.value, shared_impact_end.value = 315, 15; stall_tsleep(20)

        # =========================================================
        # PHASE 4: RECOVERY
        # =========================================================
        print("\n-- phase 4: recovery --")
        shared_speed.value = 0; shared_turn_bias.value = 0.0; shared_gait_id.value = 0; tsleep(3)

        state_recovery_wiggle()
        state_self_right_roll()

        # =========================================================
        # PHASE 5: DONE
        # =========================================================
        print("\n-- phase 5: shutdown --")
        shared_speed.value = 0; shared_turn_bias.value = 0.0; tsleep(8)

    except EmergencyStopException as e:
        print(f"[brain] emergency stop: {e}")
    except KeyboardInterrupt:
        print("\n[brain] interrupted")
    finally:
        is_running.value = False
        gait_process.join(timeout=15)
        print("done")
