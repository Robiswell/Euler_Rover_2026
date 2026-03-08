#!/usr/bin/env python3
"""
Hexapod gait engine for 6 STS3215 serial-bus servos.

Architecture:
  Brain (main process) — state machine that sequences gaits, speeds, and turns.
  Heart (child process) — 50 Hz closed-loop kinematics, servo writes, and safety checks.

Safety:
  - Raised-cosine air phase with matched feedforward derivative
  - Per-servo stall detection with active compliance yield
  - Thermal and voltage debounce shutdown
  - Anti-collision leg splay bias on front and rear legs
  - Velocity-mode boot and park (avoids encoder 0-boundary wrap)
  - Rotating health telemetry to stay within bus bandwidth
  - 10 MB log rotation
"""

import time
import sys
import signal
import datetime
import os
import multiprocessing as mp
import gc
import math
import csv

try:
    from scservo_sdk import PortHandler, PacketHandler, GroupSyncWrite
except ImportError:
    print("\nERROR: scservo_sdk not installed. Run: pip install scservo_sdk\n")
    sys.exit(1)

# =============================================================
# HARDWARE CONFIGURATION
# =============================================================
PORT_NAME      = "/dev/ttyUSB0"
BAUDRATE       = 1000000
SERVO_PROTOCOL = 0

# STS3215 control table registers
ADDR_TORQUE_ENABLE    = 40
ADDR_ACCEL            = 41
ADDR_GOAL_POSITION    = 42
ADDR_GOAL_SPEED       = 46
ADDR_TORQUE_LIMIT     = 16
ADDR_MODE             = 33
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED    = 58
ADDR_PRESENT_LOAD     = 60
ADDR_PRESENT_VOLTAGE  = 62
ADDR_PRESENT_TEMP     = 63
ADDR_PRESENT_CURRENT  = 69
LEN_GOAL_SPEED        = 2

# Robot physical constants
ENCODER_RESOLUTION = 4096.0  # ticks per full rotation
VELOCITY_SCALAR    = 1.85    # deg/s to STS speed units
VOLTAGE_MIN        = 10.0    # low-voltage cutoff (3S LiPo under load)
TEMP_MAX           = 70      # thermal shutdown threshold (°C)
TEMP_ERROR_VAL     = 125     # STS3215 ghost value returned on sensor glitch
LOG_FILE           = "telemetry_log.txt"
LOG_MAX_SIZE       = 10 * 1024 * 1024  # rotate at 10 MB

LEFT_SERVOS  = [2, 3, 4]
RIGHT_SERVOS = [1, 6, 5]
ALL_SERVOS   = LEFT_SERVOS + RIGHT_SERVOS

# Left-side servos are mounted inverted relative to right-side
DIRECTION_MAP = {
    1:  1,   # right front
    2: -1,   # left front  (inverted)
    3: -1,   # left middle (inverted)
    4: -1,   # left rear   (inverted)
    5:  1,   # right rear
    6:  1,   # right middle
}

# Encoder counts where each leg points straight down
HOME_POSITIONS = {
    1: 2233, 2: 2731, 3: 4086,
    4: 2606, 5: 253,  6: 771,
}

KP_PHASE        = 12.5  # proportional gain for phase tracking
STALL_THRESHOLD     = 600   # load magnitude that triggers stall yield
RECURRENCE_WINDOW    = 150   # frames (~3 s at 50 Hz) for sand-trap rolling window
SAND_TRAP_THRESHOLD  = 120   # stall servo-frames in window to declare sand trap
RECURRENCE_THRESHOLD = SAND_TRAP_THRESHOLD  # audit alias

# Anti-collision splay: permanently offsets front/rear legs to prevent
# toe contact during the tripod air swing.
# Negative = biased forward, positive = biased backward.
# At ±35° (70° total separation) the 150 mm arc legs clear by ~2.6 mm.
LEG_SPLAY = {
    1: -35, 2: -35,  # front legs splayed forward
    6:   0, 3:   0,  # middle legs centered
    5:  35, 4:  35,  # rear legs splayed backward
}

# Gaits — duty cycle and per-leg phase offsets.
# Tripod:    two groups of three, 180° apart. Fast, moderate stability.
# Wave:      sequential single-leg lifts, back-to-front. Slowest, highest stability.
# Quadruped: diagonal pairs. Mid-speed, good terrain compliance.
GAITS = {
    0: {  # Tripod
        'duty': 0.5,
        'offsets': {2: 0.0, 6: 0.0, 4: 0.0,  1: 0.5, 3: 0.5, 5: 0.5}
    },
    1: {  # Wave
        'duty': 0.85,
        'offsets': {1: 0.833, 6: 0.666, 5: 0.5,  2: 0.333, 3: 0.166, 4: 0.0}
    },
    2: {  # Quadruped
        # Servo 6 offset changed from 0.333 → 0.833 to prevent collision with
        # servo 5. With offset 0.333, servo 6 was at 359° while servo 5 touched
        # down at 5° — only 6° angular separation, insufficient for crooked legs.
        # At 0.833, servo 6 is in its air phase (105°) when servo 5 lands → 149°
        # separation. Minimum across full cycle rises from 6° to 49°.
        # Servo 3 (left middle) intentionally kept at 0.333 — it runs on the
        # left clock and does not interact with the servo 5/6 collision zone.
        'duty': 0.7,
        'offsets': {1: 0.666, 4: 0.666,  6: 0.833, 3: 0.333,  5: 0.0, 2: 0.0}
    },
}


def get_buehler_angle(t_norm, duty_cycle, start_ang, end_ang, is_reversed=False):
    """
    Returns the target phase angle for a leg at normalized clock position t_norm.

    Stance (t <= duty): linear sweep from start to end — constant body velocity.
    Air   (t >  duty): raised-cosine return swing — velocity is zero at liftoff
                       and touchdown, peaks at mid-swing.

    is_reversed=True swaps start/end so the stance sweep runs backward while the
    clock always advances forward.  This avoids the backward-clock issues (low-KP
    freeze during speed transition, 300-degree air sweep direction error) that
    caused all legs to hang near 180° during reverse quadruped.
    """
    stance_sweep = (end_ang - start_ang + 180) % 360 - 180

    if is_reversed:
        start_ang, end_ang = end_ang, start_ang
        stance_sweep = -stance_sweep

    if t_norm <= duty_cycle:
        progress = t_norm / duty_cycle
        angle = start_ang + (stance_sweep * progress)
    else:
        progress     = (t_norm - duty_cycle) / (1.0 - duty_cycle)
        air_sweep    = 360.0 - abs(stance_sweep)
        if stance_sweep < 0:
            air_sweep = -air_sweep
        smooth_progress = (1.0 - math.cos(math.pi * progress)) / 2.0
        angle = end_ang + (air_sweep * smooth_progress)

    return angle % 360


# =============================================================
# HEART — 50 Hz closed-loop kinematics
# =============================================================
def gait_worker(shared_speed, shared_x_flip, shared_z_flip, shared_turn_bias,
                shared_gait_id, shared_impact_start, shared_impact_end,
                shared_servo_loads, shared_heartbeat, is_running,
                shared_sand_trap):

    signal.signal(signal.SIGINT, signal.SIG_IGN)

    try:
        os.nice(-20)
    except:
        print("[heart] running without elevated priority — loop jitter may occur")

    gc.disable()

    port_handler     = PortHandler(PORT_NAME)
    packet_handler   = PacketHandler(SERVO_PROTOCOL)
    group_sync_write = GroupSyncWrite(port_handler, packet_handler, ADDR_GOAL_SPEED, LEN_GOAL_SPEED)

    # Short packet timeout so a dead servo doesn't stall the whole loop
    port_handler.setPacketTimeoutMillis(10)

    # Internal state
    is_stalled        = {id: False for id in ALL_SERVOS}
    prev_stalled      = {id: False for id in ALL_SERVOS}
    stall_counters    = {id: 0     for id in ALL_SERVOS}
    stall_exit_ramp   = {id: 0     for id in ALL_SERVOS}
    actual_phases     = {id: 0.0   for id in ALL_SERVOS}
    stall_window      = [0] * RECURRENCE_WINDOW   # circular buffer: stalled-servo count per frame
    window_total      = 0                          # running sum of stall_window
    window_idx        = 0                          # next write position
    comm_error_streak = 0
    telemetry_rotator = 0
    last_log_time      = 0
    last_jitter_log_t  = 0
    parent_pid         = os.getppid()

    # CSV telemetry state
    frame_counter = 0
    last_load     = {id: 0                for id in ALL_SERVOS}
    last_pos      = {id: HOME_POSITIONS[id] for id in ALL_SERVOS}

    # Persistent sensor readings — rotating telemetry only polls one servo per
    # frame, so we hold each servo's last-known value to avoid false resets.
    voltage_reading    = 12.0
    volt_dip_counter   = 0
    temp_spike_counter = 0
    hot_servo_id       = -1
    last_temp_readings = {sid: 0 for sid in ALL_SERVOS}

    def log_telemetry(volt, temp_max, load_max, total_amps, fsm_speed, fsm_turn, gait=0, hot_srv=-1):
        """Append one telemetry line; rotate file at 10 MB."""
        try:
            if os.path.exists(LOG_FILE) and os.path.getsize(LOG_FILE) > LOG_MAX_SIZE:
                ts_r = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                os.rename(LOG_FILE, LOG_FILE + f".{ts_r}")
                import glob
                for old in sorted(glob.glob(LOG_FILE + ".*"), key=os.path.getmtime)[:-20]:
                    try: os.remove(old)
                    except: pass
            with open(LOG_FILE, "a") as f:
                ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                f.write(
                    f"[{ts}] Spd:{fsm_speed} Trn:{fsm_turn:.1f} Gait:{gait} | "
                    f"V:{volt:.1f}V | T:{temp_max}C(S{hot_srv}) | L:{load_max} | A:{total_amps:.2f}\n"
                )
        except:
            pass

    def init_and_align_servos():
        """Open serial port, set velocity mode, and LERP all legs to the park splay."""
        while is_running.value:
            if port_handler.openPort() and port_handler.setBaudRate(BAUDRATE):
                break
            print("[heart] waiting for serial port...")
            time.sleep(1.0)

        port_handler.clearPort()
        print("[heart] boot: moving to park stance")

        for sid in ALL_SERVOS:
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 1)  # velocity mode
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_ACCEL, 0)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_TORQUE_LIMIT, 1000)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 1)
        time.sleep(0.5)

        # Read current phase of each servo before interpolating
        start_phases = {}
        for sid in ALL_SERVOS:
            pos, r1, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_POSITION)
            if r1 == 0:
                start_phases[sid] = (((pos - HOME_POSITIONS[sid]) / ENCODER_RESOLUTION) * 360.0 * DIRECTION_MAP[sid]) % 360
            else:
                start_phases[sid] = 0.0

        # 70-frame sequence: LERP over first 50 frames, hold on target for 20
        for i in range(70):
            if not is_running.value:
                break
            lerp = min(1.0, (i + 1) / 50.0)
            for sid in ALL_SERVOS:
                target_ph  = LEG_SPLAY.get(sid, 0) % 360
                diff       = (target_ph - start_phases[sid] + 180) % 360 - 180
                cur_target = (start_phases[sid] + diff * lerp) % 360

                pos, r1, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_POSITION)
                if r1 == 0:
                    cur_ph    = (((pos - HOME_POSITIONS[sid]) / ENCODER_RESOLUTION) * 360.0 * DIRECTION_MAP[sid]) % 360
                    short_err = (cur_target - cur_ph + 180) % 360 - 180
                    pd_term   = short_err * (KP_PHASE * 1.5)
                    if abs(short_err) > 0.5:
                        pd_term += 40 if short_err > 0 else -40

                    raw_speed   = pd_term * DIRECTION_MAP[sid]
                    final_speed = max(-500, min(500, int(raw_speed)))
                    abs_v       = int(abs(final_speed))
                    speed_val   = (abs_v | 0x8000) if final_speed < 0 else abs_v
                    group_sync_write.addParam(sid, [speed_val & 0xff, (speed_val >> 8) & 0xff])

            group_sync_write.txPacket()
            group_sync_write.clearParam()
            time.sleep(0.02)

        for sid in ALL_SERVOS:
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 0)
        print("[heart] boot: park complete, entering control loop")

    init_and_align_servos()

    _csv_path   = os.path.expanduser(
        f"~/hexapod_run_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    _csv_file   = open(_csv_path, 'w', newline='', buffering=1)
    _csv_writer = csv.writer(_csv_file)
    _csv_writer.writerow([
        'frame', 'master_time_L', 'master_time_R', 'sid',
        'phase', 'cmd_dps', 'actual_pos', 'load',
        'stall_active', 'stall_counter', 'zone', 'hz', 'gait',
    ])
    print(f"[heart] CSV telemetry: {_csv_path}")

    master_time_L  = 0.0
    master_time_R  = 0.0
    target_dt      = 1.0 / 50.0
    last_loop_time = time.perf_counter()

    smooth_speed     = 0.0
    smooth_turn      = 0.0
    smooth_imp_start = float(shared_impact_start.value)
    smooth_imp_end   = float(shared_impact_end.value)
    smooth_duty      = 0.5
    # Seed from the initial gait so the slew starts from the correct side.
    # Initializing to 0.0 would cause legs whose target offset is 0.5 to converge
    # backwards (0.0 → 0.98 → 0.97 → ... → 0.5) for the first ~25 frames.
    smooth_offsets   = {sid: GAITS[0]['offsets'][sid] for sid in ALL_SERVOS}

    try:
        while is_running.value:
            loop_start_time = time.perf_counter()
            raw_dt          = loop_start_time - last_loop_time
            last_loop_time  = loop_start_time

            # Clamp dt so a CPU spike doesn't cause a huge position jump
            real_dt = min(0.05, raw_dt)

            # Log frames that ran significantly late, throttled to once per second
            # (un-throttled file I/O here would make the next frame late too)
            if raw_dt > target_dt * 2.0 and loop_start_time - last_jitter_log_t > 1.0:
                last_jitter_log_t = loop_start_time
                jitter_ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                try:
                    with open(LOG_FILE, "a") as f:
                        f.write(f"[{jitter_ts}] WARN: loop jitter {raw_dt*1000:.1f}ms (target {target_dt*1000:.0f}ms)\n")
                except:
                    pass

            shared_heartbeat.value += 1

            # Stop if the Brain process has gone away
            if loop_start_time - last_log_time > 1.0:
                if os.getppid() != parent_pid:
                    print("[heart] brain process gone, stopping")
                    break

            # Clear serial buffer to prevent frame-latency buildup
            port_handler.clearPort()

            # --- READ SENSORS ---
            current_load_max = 0
            current_total_ma = 0
            loop_comm_fail   = False

            for sid in ALL_SERVOS:
                pos, r1, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_POSITION)
                if r1 == 0:
                    last_pos[sid]      = pos
                    diff               = pos - HOME_POSITIONS[sid]
                    actual_phases[sid] = ((diff / ENCODER_RESOLUTION) * 360.0 * DIRECTION_MAP[sid]) % 360
                else:
                    loop_comm_fail = True

                # Load read: ALL servos every frame for correct 3-frame stall latency.
                # Rotating-only load (v1 original) only read each servo every 6 frames,
                # making effective stall latency 18 frames (360ms) instead of 3 (60ms).
                # Temp/voltage/current remain rotating (only one servo per frame needed).
                load, r2, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_LOAD)

                # Stall detection on every servo every frame (load read moved outside rotate guard)
                if r2 == 0:
                    l_mag = load & 0x3FF
                    last_load[sid] = l_mag
                    if 0 <= sid < len(shared_servo_loads):
                        shared_servo_loads[sid] = l_mag

                    if l_mag > STALL_THRESHOLD:
                        stall_counters[sid] += 1
                    else:
                        stall_counters[sid] = max(0, stall_counters[sid] - 1)

                    is_stalled[sid]  = (stall_counters[sid] >= 3)
                    current_load_max = max(current_load_max, l_mag)

                    if l_mag > 900:
                        is_stalled[sid] = True

                    if is_stalled[sid] and not prev_stalled[sid]:
                        ev_ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        print(f"[heart] servo {sid} stalled (load={l_mag})")
                        try:
                            with open(LOG_FILE, "a") as f:
                                f.write(f"[{ev_ts}] EVENT: servo {sid} STALLED (load={l_mag})\n")
                        except:
                            pass
                    elif not is_stalled[sid] and prev_stalled[sid]:
                        ev_ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        print(f"[heart] servo {sid} stall cleared")
                        try:
                            with open(LOG_FILE, "a") as f:
                                f.write(f"[{ev_ts}] EVENT: servo {sid} STALL CLEARED\n")
                        except:
                            pass
                    prev_stalled[sid] = is_stalled[sid]

                # Rotating health: temp/voltage/current for one servo per frame
                if sid == ALL_SERVOS[telemetry_rotator] or sid == 1:
                    temp, r3, _ = packet_handler.read1ByteTxRx(port_handler, sid, ADDR_PRESENT_TEMP)
                    volt, r4, _ = packet_handler.read1ByteTxRx(port_handler, sid, ADDR_PRESENT_VOLTAGE)
                    amps, r5, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_CURRENT)

                    if r3 == 0:
                        if temp != TEMP_ERROR_VAL:  # filter STS3215 ghost reads
                            last_temp_readings[sid] = temp

                    if r4 == 0:
                        voltage_reading = volt / 10.0

                    if r5 == 0:
                        current_total_ma = (amps & 0x7FFF)

            telemetry_rotator = (telemetry_rotator + 1) % len(ALL_SERVOS)

            # --- SAND TRAP ROLLING WINDOW (per frame) ---
            n_trapped = sum(1 for s in ALL_SERVOS if is_stalled[s])
            n_stalled = n_trapped  # alias used by telemetry
            window_total -= stall_window[window_idx]
            stall_window[window_idx] = n_trapped
            window_total += n_trapped
            window_idx = (window_idx + 1) % RECURRENCE_WINDOW
            if window_total >= SAND_TRAP_THRESHOLD:
                shared_sand_trap.value = True

            # Derive max temp from all servos' persistent readings
            current_temp_max = max(last_temp_readings.values())
            hot_servo_id     = max(last_temp_readings, key=last_temp_readings.get)

            # --- COMM HEALTH ---
            if loop_comm_fail:
                comm_error_streak += 1
            else:
                comm_error_streak = 0

            if comm_error_streak > 15:
                print("[heart] serial bus error — stopping")
                log_telemetry(voltage_reading, current_temp_max, current_load_max,
                              current_total_ma / 1000.0, shared_speed.value, shared_turn_bias.value,
                              gait=shared_gait_id.value, hot_srv=hot_servo_id)
                is_running.value = False
                break

            # --- THERMAL & VOLTAGE DEBOUNCE ---
            if current_temp_max > TEMP_MAX:
                temp_spike_counter += 1
            else:
                temp_spike_counter = 0

            if voltage_reading < VOLTAGE_MIN:
                volt_dip_counter += 1
            else:
                volt_dip_counter = 0

            if temp_spike_counter > 15 or volt_dip_counter > 150:  # 150 ticks = 3s debounce — covers transient sag during max-torque extraction
                reason = "over temperature" if temp_spike_counter > 15 else "low voltage"
                val    = f"{current_temp_max}C servo {hot_servo_id}" if temp_spike_counter > 15 else f"{voltage_reading:.1f}V"
                print(f"[heart] safety shutdown: {reason} ({val})")
                log_telemetry(voltage_reading, current_temp_max, current_load_max,
                              current_total_ma / 1000.0, shared_speed.value, shared_turn_bias.value,
                              gait=shared_gait_id.value, hot_srv=hot_servo_id)
                is_running.value = False
                break

            # --- SNAPSHOT FSM STATE ---
            fsm_spd   = shared_speed.value
            fsm_trn   = shared_turn_bias.value
            fsm_x     = shared_x_flip.value
            fsm_z     = shared_z_flip.value
            fsm_gait  = shared_gait_id.value
            fsm_imp_s = shared_impact_start.value
            fsm_imp_e = shared_impact_end.value

            if loop_start_time - last_log_time > 1.0:
                log_telemetry(voltage_reading, current_temp_max, current_load_max,
                              current_total_ma / 1000.0, fsm_spd, fsm_trn,
                              gait=fsm_gait, hot_srv=hot_servo_id)
                last_log_time = loop_start_time

            # --- SLEW-RATE SMOOTHING ---
            target_effective_speed = fsm_spd * fsm_x * fsm_z
            lerp_rate = min(1.0, 2.0 * real_dt)  # ~0.5 second ramp

            smooth_speed += (target_effective_speed - smooth_speed) * lerp_rate
            smooth_turn  += (fsm_trn - smooth_turn) * lerp_rate

            # Shortest-path interpolation for impact angles
            d_s = (fsm_imp_s - smooth_imp_start + 180) % 360 - 180
            smooth_imp_start = (smooth_imp_start + d_s * lerp_rate) % 360

            d_e = (fsm_imp_e - smooth_imp_end + 180) % 360 - 180
            smooth_imp_end = (smooth_imp_end + d_e * lerp_rate) % 360

            # Smooth duty cycle transitions so gait switches don't snap
            g_params = GAITS.get(fsm_gait, GAITS[0])
            t_duty   = max(0.01, min(0.99, g_params['duty']))
            smooth_duty += (t_duty - smooth_duty) * min(1.0, 1.0 * real_dt)

            t_offsets = g_params['offsets']
            for sid in ALL_SERVOS:
                o_diff = (t_offsets[sid] - smooth_offsets[sid] + 0.5) % 1.0 - 0.5
                smooth_offsets[sid] = (smooth_offsets[sid] + o_diff * lerp_rate) % 1.0

            # --- CLOCKS ---
            base_hz = smooth_speed / 1000.0
            hz_L    = base_hz + smooth_turn
            hz_R    = base_hz - smooth_turn

            # Governor: prevent air-swing from saturating the 3000-unit speed limit.
            # Cosine peaks at π/2 × the mean speed, so the denominator accounts for that.
            base_sweep = (smooth_imp_end - smooth_imp_start + 180) % 360 - 180
            air_sweep  = 360.0 - abs(base_sweep)
            if base_sweep < 0:
                air_sweep = -air_sweep
            max_safe_hz = (2800.0 / VELOCITY_SCALAR * (1.0 - smooth_duty)) / max(5.0, abs(air_sweep) * math.pi / 2.0)
            hz_L = max(-max_safe_hz, min(max_safe_hz, hz_L))
            hz_R = max(-max_safe_hz, min(max_safe_hz, hz_R))

            # Advance clocks using ABSOLUTE hz so they always count forward.
            # Direction is encoded in is_reversed passed to get_buehler_angle —
            # not in clock direction.  Backward-running clocks caused legs to
            # hang near 180° during reverse: the 300° air sweep traversed in
            # reverse combined with the near-zero KP during the speed lerp
            # transition left legs stranded at the air-phase midpoint.
            master_time_L = (master_time_L + abs(hz_L) * real_dt) % 1.0
            master_time_R = (master_time_R + abs(hz_R) * real_dt) % 1.0

            # Keep left/right clocks synchronized during straight-line walking.
            # Cap the per-frame correction to 0.002 cycles so a large post-pivot
            # divergence decays gradually rather than stepping ff_air by >50 STS.
            # (Uncapped: clock_diff=0.5 after long pivot → 0.01-cycle step → 131 STS.
            #  At 0.002 cycles/frame: max step = 0.002×7107×1.85 = 26 STS. ✓)
            if abs(smooth_turn) < 0.01 and abs(smooth_speed) > 10.0 and (hz_L * hz_R) > 0:
                clock_diff = (master_time_L - master_time_R + 0.5) % 1.0 - 0.5
                raw_decay  = clock_diff * min(1.0, 2.0 * real_dt)
                decay      = max(-0.002, min(0.002, raw_decay))
                master_time_L = (master_time_L - decay / 2) % 1.0
                master_time_R = (master_time_R + decay / 2) % 1.0

            # --- VELOCITY COMMANDS ---
            # Blend window width in normalized cycle units.
            # At 1.2 Hz gait, 0.03 cycles ≈ 25 ms — wide enough to smooth the
            # feedforward step at liftoff and touchdown without distorting gait geometry.
            BLEND = 0.03
            raw_speed = 0.0  # pre-init so the del below is always valid
            STALL_EXIT_FRAMES = 10  # 200 ms ramp after stall clears
            for sid in ALL_SERVOS:
                # CSV telemetry capture — populated in whichever branch runs below
                _csv_zone  = 0     # 0=stalled, 1-5=feedforward zone
                _csv_dps   = 0.0   # feedforward deg/s
                _csv_t_leg = -1.0  # normalized leg phase (−1 when stalled)
                _csv_s_hz  = 0.0   # per-side clock frequency

                if is_stalled[sid]:
                    stall_exit_ramp[sid] = 0  # reset while stalled
                    final_speed = 0  # active compliance yield
                else:
                    s_hz     = hz_L if sid in LEFT_SERVOS else hz_R
                    m_t      = master_time_L if sid in LEFT_SERVOS else master_time_R
                    is_rev   = (s_hz < 0)   # direction flag — clock always advances
                    _csv_s_hz = s_hz

                    cur_ph = actual_phases.get(sid, 0.0)
                    t_leg  = (m_t + smooth_offsets[sid]) % 1.0
                    _csv_t_leg = t_leg
                    tar_ph = get_buehler_angle(t_leg, smooth_duty, smooth_imp_start, smooth_imp_end,
                                               is_reversed=is_rev)

                    # Apply anti-collision splay offset
                    tar_ph = (tar_ph + LEG_SPLAY.get(sid, 0)) % 360

                    # Feedforward: derivative of the raised-cosine trajectory.
                    # Two blend zones prevent velocity steps at the phase boundaries:
                    #   Touchdown (t_leg near 0): air ff ends at 0, stance ff starts at full speed.
                    #                             Ramp up over BLEND cycles to avoid the step.
                    #   Liftoff   (t_leg near duty): stance ff is full speed, air ff starts at 0.
                    #                               Blend across the boundary over BLEND cycles.
                    ff_stance = (base_sweep / smooth_duty) * s_hz
                    if t_leg < BLEND:
                        # Post-touchdown ramp: ramp from 0 up to full stance ff over
                        # BLEND cycles. Cosine envelope gives zero slope at both ends,
                        # making the ramp C1-continuous with zone 2 (no click at exit).
                        env         = (1.0 - math.cos(math.pi * t_leg / BLEND)) / 2.0
                        deg_per_sec = ff_stance * env
                        _csv_zone   = 1
                    elif t_leg < smooth_duty:
                        # Deep stance: full feedforward until the leg actually lifts off.
                        # Keeping this at full value all the way to t=duty prevents the
                        # 50% feedforward dip that occurred when the blend started early.
                        deg_per_sec = ff_stance
                        _csv_zone   = 2
                    elif t_leg < smooth_duty + 2.0 * BLEND:
                        # Liftoff blend: leg just left the ground (t >= duty).
                        # Blend from ff_stance into the rising air cosine over 2*BLEND
                        # cycles. Starting at t=duty keeps stance speed at full value
                        # until liftoff, eliminating the pre-liftoff slowdown.
                        blend_frac         = (t_leg - smooth_duty) / (2.0 * BLEND)
                        air_progress_blend = (t_leg - smooth_duty) / (1.0 - smooth_duty)
                        ff_air = (air_sweep * math.pi / (2.0 * (1.0 - smooth_duty))) \
                                 * math.sin(math.pi * air_progress_blend) * s_hz
                        deg_per_sec = ff_stance * (1.0 - blend_frac) + ff_air * blend_frac
                        _csv_zone   = 3
                    elif t_leg > 1.0 - BLEND:
                        # Pre-touchdown blend: fade air ff to 0 before landing.
                        # The guard t_leg > smooth_duty + BLEND is implicit here because
                        # smooth_duty + 2*BLEND < 1 - BLEND for all valid duty cycles.
                        # Cosine envelope: zero slope at both ends → C1-continuous with
                        # zone 5 at entry and with zone 1 (zero) at exit. Eliminates the
                        # 450 deg/s/frame derivative jump caused by the old linear fade.
                        air_progress   = (t_leg - smooth_duty) / (1.0 - smooth_duty)
                        ff_air_full    = (air_sweep * math.pi / (2.0 * (1.0 - smooth_duty))) \
                                         * math.sin(math.pi * air_progress) * s_hz
                        blend_frac_pre = (t_leg - (1.0 - BLEND)) / BLEND  # 0→1
                        env            = (1.0 + math.cos(math.pi * blend_frac_pre)) / 2.0
                        deg_per_sec    = ff_air_full * env
                        _csv_zone      = 4
                    else:
                        # Deep air: full sinusoidal feedforward.
                        air_progress = (t_leg - smooth_duty) / (1.0 - smooth_duty)
                        deg_per_sec  = (air_sweep * math.pi / (2.0 * (1.0 - smooth_duty))) \
                                       * math.sin(math.pi * air_progress) * s_hz
                        _csv_zone    = 5
                    _csv_dps = deg_per_sec

                    ff_speed = deg_per_sec * VELOCITY_SCALAR

                    # PD correction
                    short_err = (tar_ph - cur_ph + 180) % 360 - 180
                    # Only unwrap if the error direction contradicts the walking direction —
                    # unconditional unwrap would corrupt large genuine lag corrections.
                    if s_hz > 0.05 and short_err < -90:
                        short_err += 360
                    elif s_hz < -0.05 and short_err > 90:
                        short_err -= 360

                    # Blend gain from 15% (stopped) to 100% (0.1 Hz and above) to
                    # avoid rubber-band overcorrection at low speed without a hard
                    # velocity step when s_hz crosses the threshold.
                    kp_scale   = min(1.0, abs(s_hz) / 0.1)
                    current_kp = KP_PHASE * (0.15 + 0.85 * kp_scale)
                    pd_term    = max(-45, min(45, short_err)) * current_kp

                    raw_speed = (ff_speed + pd_term) * DIRECTION_MAP[sid]

                    # Relax when effectively stopped and close to target
                    if abs(s_hz) < 0.01 and abs(short_err) < 15:
                        final_speed = 0
                    else:
                        final_speed = max(-3000, min(3000, int(raw_speed)))

                    # Post-stall soft ramp: blend from 0 to full over 200 ms so
                    # the servo doesn't snap from compliance yield to full air-swing
                    # feedforward in a single frame (up to ~2600 STS step otherwise).
                    if stall_exit_ramp[sid] < STALL_EXIT_FRAMES:
                        stall_exit_ramp[sid] += 1
                        final_speed = int(final_speed * stall_exit_ramp[sid] / STALL_EXIT_FRAMES)

                # CSV: write stance-phase rows and all stalled rows (load is stance-relevant;
                # skipping pure air rows saves ~50% I/O at tripod duty, ~85% at wave duty).
                _in_stance = (0.0 <= _csv_t_leg <= smooth_duty)
                if _in_stance or is_stalled[sid]:
                    _csv_writer.writerow([
                        frame_counter,
                        f"{master_time_L:.6f}", f"{master_time_R:.6f}",
                        sid,
                        f"{actual_phases.get(sid, 0.0):.2f}",
                        f"{_csv_dps:.3f}",
                        last_pos[sid],
                        last_load[sid],
                        1 if is_stalled[sid] else 0,
                        stall_counters[sid],
                        _csv_zone,
                        f"{_csv_s_hz:.4f}",
                        fsm_gait,
                    ])

                abs_v     = int(abs(final_speed))
                speed_val = (abs_v | 0x8000) if final_speed < 0 else abs_v
                group_sync_write.addParam(sid, [speed_val & 0xff, (speed_val >> 8) & 0xff])

            group_sync_write.txPacket()
            group_sync_write.clearParam()

            frame_counter += 1

            # Hybrid timing: sleep then busy-wait for sub-millisecond precision
            elapsed = time.perf_counter() - loop_start_time
            if target_dt - elapsed > 0.002:
                time.sleep(target_dt - elapsed - 0.002)
            while time.perf_counter() - loop_start_time < target_dt:
                pass

            del raw_speed, final_speed, speed_val

    except Exception as e:
        print(f"[heart] loop error: {e}")
    finally:
        try:
            _csv_file.close()
            print(f"[heart] CSV closed: {_csv_path}")
        except Exception:
            pass
        print("\n[heart] shutdown: parking legs")
        group_sync_write.clearParam()
        port_handler.clearPort()

        # Stop all servos immediately
        for sid in ALL_SERVOS:
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 0)
        time.sleep(0.1)

        # Read current phases for the park LERP
        start_phases = {}
        for sid in ALL_SERVOS:
            pos, r1, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_POSITION)
            if r1 == 0:
                start_phases[sid] = (((pos - HOME_POSITIONS[sid]) / ENCODER_RESOLUTION) * 360.0 * DIRECTION_MAP[sid]) % 360
            else:
                start_phases[sid] = LEG_SPLAY.get(sid, 0)

        # LERP to splay park stance
        for i in range(70):
            lerp = min(1.0, (i + 1) / 50.0)
            for sid in ALL_SERVOS:
                target_ph  = LEG_SPLAY.get(sid, 0) % 360
                diff       = (target_ph - start_phases[sid] + 180) % 360 - 180
                cur_target = (start_phases[sid] + diff * lerp) % 360

                pos, r1, _ = packet_handler.read2ByteTxRx(port_handler, sid, ADDR_PRESENT_POSITION)
                if r1 == 0:
                    cur_ph    = (((pos - HOME_POSITIONS[sid]) / ENCODER_RESOLUTION) * 360.0 * DIRECTION_MAP[sid]) % 360
                    short_err = (cur_target - cur_ph + 180) % 360 - 180
                    pd_term   = short_err * (KP_PHASE * 1.5)
                    if abs(short_err) > 0.5:
                        pd_term += 40 if short_err > 0 else -40

                    raw_speed   = pd_term * DIRECTION_MAP[sid]
                    final_speed = max(-500, min(500, int(raw_speed)))
                    abs_v       = int(abs(final_speed))
                    speed_val   = (abs_v | 0x8000) if final_speed < 0 else abs_v
                    group_sync_write.addParam(sid, [speed_val & 0xff, (speed_val >> 8) & 0xff])

            group_sync_write.txPacket()
            group_sync_write.clearParam()
            time.sleep(0.02)

        for sid in ALL_SERVOS:
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 0)
        time.sleep(0.1)
        for sid in ALL_SERVOS:
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
        port_handler.closePort()
        print("[heart] offline")


# =============================================================
# BRAIN — tactical state machine
# =============================================================
class EmergencyStopException(Exception):
    pass


class SandTrapException(Exception):
    pass


_sand_trap_ref    = None   # set to shared_sand_trap before tactical sequence
_shared_speed     = None   # module-level refs so state_sand_extraction() can be top-level
_shared_turn_bias = None
_shared_gait_id   = None
_is_running       = None
_heart_process_ref = None  # set to gait_process before tactical sequence; checked for liveness


def state_sand_extraction():
    """S4 strategy: oscillate 3 cycles to loosen compacted sand, then Wave crawl out."""
    print("\n[brain] SAND TRAP -- executing extraction (S4: oscillate x3 + Wave crawl)")

    # Settle
    _shared_speed.value = 0
    tactical_sleep(1.5, _is_running)

    # 3 oscillation cycles (reverse then forward) — speed cap <= 600
    for cycle in range(3):
        print(f"[brain] sand osc cycle {cycle + 1}/3")
        _shared_speed.value = -400
        tactical_sleep(0.4, _is_running)
        _shared_speed.value = 400
        tactical_sleep(0.3, _is_running)

    # Wave crawl to exit
    print("[brain] sand crawl: Wave gait speed=200 for 8 s")
    _shared_speed.value      = 0
    _shared_turn_bias.value  = 0.0
    _shared_gait_id.value    = 1   # Wave
    _shared_speed.value      = 200
    tactical_sleep(8.0, _is_running)

    # Settle before handing back to Brain exception handler
    print("[brain] extraction complete")
    _shared_speed.value   = 0
    _shared_gait_id.value = 0


def tactical_sleep(duration, running_flag):
    """Sleep for duration seconds, waking every 100 ms to check safety and sand trap."""
    start = time.time()
    while time.time() - start < duration:
        if not running_flag.value:
            raise EmergencyStopException("safety halt from heart process")
        if _heart_process_ref is not None and not _heart_process_ref.is_alive():
            raise EmergencyStopException("heart process died unexpectedly")
        if _sand_trap_ref is not None and _sand_trap_ref.value:
            _sand_trap_ref.value = False   # acknowledge — Heart will re-arm if still trapped
            raise SandTrapException("sand trap detected by Heart")
        time.sleep(0.1)


if __name__ == "__main__":
    print("\n" + "="*60)
    print("      HEXAPOD GAIT TEST")
    print("="*60)

    if os.path.exists(LOG_FILE):
        try:
            ts  = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            os.rename(LOG_FILE, LOG_FILE + f".{ts}")
            # Keep only the 5 most recent archived logs
            import glob
            archives = sorted(glob.glob(LOG_FILE + ".*"), key=os.path.getmtime)
            for old in archives[:-20]:
                try:
                    os.remove(old)
                except:
                    pass
        except:
            pass

    try:
        with open(LOG_FILE, "a") as f:
            f.write(f"\n{'='*60}\n")
            f.write(f"=== SESSION START {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===\n")
            f.write(f"{'='*60}\n")
    except:
        pass

    shared_speed        = mp.Value('i', 0)
    shared_x_flip       = mp.Value('i', 1)
    shared_z_flip       = mp.Value('i', 1)
    shared_turn_bias    = mp.Value('f', 0.0)    # c_float (4B) — atomic on ARMv7; c_double (8B) was not
    shared_gait_id      = mp.Value('i', 0)
    shared_impact_start = mp.Value('i', 320)
    shared_impact_end   = mp.Value('i', 40)
    shared_servo_loads  = mp.Array('i', 7)
    shared_heartbeat    = mp.Value('i', 0)
    is_running          = mp.Value('b', True)
    shared_sand_trap    = mp.Value('b', False)

    for i in range(7):
        shared_servo_loads[i] = -1

    gait_process = mp.Process(target=gait_worker, args=(
        shared_speed, shared_x_flip, shared_z_flip, shared_turn_bias,
        shared_gait_id, shared_impact_start, shared_impact_end,
        shared_servo_loads, shared_heartbeat, is_running,
        shared_sand_trap
    ))
    gait_process.start()

    def state_recovery_wiggle(duration=10):
        """Oscillate forward/backward to free a stuck leg."""
        print("\n[brain] stuck leg detected — running extraction wiggle")
        shared_speed.value = 600
        end_time = time.time() + duration
        while time.time() < end_time:
            if not is_running.value:
                raise EmergencyStopException("safety halt during wiggle")
            shared_x_flip.value = -1
            tactical_sleep(0.3, is_running)
            shared_x_flip.value = 1
            tactical_sleep(0.3, is_running)
        shared_speed.value  = 0
        shared_x_flip.value = 1  # restore forward direction
        print("[brain] wiggle complete")

    def state_self_right_roll():
        """Slow-torque symmetrical roll to recover from a flip."""
        print("\n[brain] upside-down detected — executing self-right roll")
        shared_z_flip.value       = -1   # inverted: negate velocity commands
        shared_gait_id.value      = 1
        shared_impact_start.value = 320
        shared_impact_end.value   = 40

        print("[brain] step 1: shift COG forward")
        shared_speed.value = 400
        tactical_sleep(4.0, is_running)

        print("[brain] step 2: reverse momentum pulse")
        shared_speed.value = -500
        tactical_sleep(8.0, is_running)

        print("[brain] step 3: secondary buildup")
        shared_speed.value = 400
        tactical_sleep(4.0, is_running)

        print("[brain] step 4: final clearing roll")
        shared_speed.value = -500
        tactical_sleep(8.0, is_running)

        shared_speed.value  = 0
        shared_z_flip.value = 1    # restore normal orientation
        print("[brain] self-right complete")

    try:
        print("\n[brain] waiting for heart link...")
        while shared_heartbeat.value < 50:
            if not is_running.value:
                raise EmergencyStopException("heart failed to start")
            time.sleep(0.1)
        print("[brain] link established — starting test sequence")

        # Arm module-level refs so top-level functions can access shared state
        _sand_trap_ref     = shared_sand_trap
        _shared_speed      = shared_speed
        _shared_turn_bias  = shared_turn_bias
        _shared_gait_id    = shared_gait_id
        _is_running        = is_running
        _heart_process_ref = gait_process

        # --- Phase 1: Tripod ---
        print("\n--- Phase 1: Tripod ---")
        shared_gait_id.value      = 0
        shared_impact_start.value = 330
        shared_impact_end.value   = 30

        print("[brain] forward sprint (speed 1200)")
        shared_speed.value = 1200
        tactical_sleep(20, is_running)

        print("[brain] carving turn left")
        shared_turn_bias.value = -0.4
        tactical_sleep(16, is_running)

        print("[brain] carving turn right")
        shared_turn_bias.value = 0.4
        tactical_sleep(16, is_running)

        print("[brain] zero-radius pivot left")
        shared_speed.value     = 0
        shared_turn_bias.value = -1.0
        tactical_sleep(16, is_running)

        print("[brain] zero-radius pivot right")
        shared_turn_bias.value = 1.0
        tactical_sleep(16, is_running)

        print("[brain] reverse sprint")
        shared_turn_bias.value = 0.0
        shared_speed.value     = -1200
        tactical_sleep(20, is_running)

        # --- Phase 2: Quadruped ---
        print("\n--- Phase 2: Quadruped ---")
        shared_gait_id.value      = 2
        shared_speed.value        = 550
        shared_turn_bias.value    = 0.0
        shared_impact_start.value = 330
        shared_impact_end.value   = 30
        tactical_sleep(12, is_running)

        print("[brain] carving turn left")
        shared_turn_bias.value = -0.3
        tactical_sleep(10, is_running)

        print("[brain] carving turn right")
        shared_turn_bias.value = 0.3
        tactical_sleep(10, is_running)

        print("[brain] zero-radius pivot left")
        shared_speed.value     = 0
        shared_turn_bias.value = -0.8
        tactical_sleep(10, is_running)

        print("[brain] zero-radius pivot right")
        shared_turn_bias.value = 0.8
        tactical_sleep(10, is_running)

        print("[brain] reverse")
        shared_turn_bias.value = 0.0
        shared_speed.value     = -550
        tactical_sleep(12, is_running)

        shared_speed.value = 0
        tactical_sleep(2, is_running)

        print("[brain] obstacle — stepping over (impacts 345->15)")
        shared_speed.value        = 500
        shared_impact_start.value = 345
        shared_impact_end.value   = 15
        tactical_sleep(15, is_running)

        print("[brain] low clearance — lowering profile (impacts 325->35)")
        shared_impact_start.value = 325
        shared_impact_end.value   = 35
        tactical_sleep(15, is_running)

        # --- Phase 3: Wave ---
        print("\n--- Phase 3: Wave ---")
        shared_gait_id.value      = 1
        shared_speed.value        = 350
        shared_turn_bias.value    = 0.0
        shared_impact_start.value = 330
        shared_impact_end.value   = 30
        tactical_sleep(12, is_running)

        print("[brain] carving turn left")
        shared_turn_bias.value = -0.12
        tactical_sleep(10, is_running)

        print("[brain] carving turn right")
        shared_turn_bias.value = 0.12
        tactical_sleep(10, is_running)

        print("[brain] zero-radius pivot left")
        shared_speed.value     = 0
        shared_turn_bias.value = -0.48
        tactical_sleep(10, is_running)

        print("[brain] zero-radius pivot right")
        shared_turn_bias.value = 0.48
        tactical_sleep(10, is_running)

        print("[brain] reverse")
        shared_turn_bias.value = 0.0
        shared_speed.value     = -350
        tactical_sleep(12, is_running)

        shared_speed.value = 0
        tactical_sleep(2, is_running)

        print("[brain] leaning into slope (impacts 315->15)")
        shared_speed.value        = 350
        shared_turn_bias.value    = 0.0
        shared_impact_start.value = 315
        shared_impact_end.value   = 15
        tactical_sleep(20, is_running)

        # --- Phase 4: Recovery ---
        print("\n--- Phase 4: Recovery ---")
        shared_speed.value     = 0
        shared_turn_bias.value = 0.0
        shared_gait_id.value   = 0
        tactical_sleep(3, is_running)

        state_recovery_wiggle()
        state_self_right_roll()

        # --- Phase 5: Shutdown ---
        print("\n--- Phase 5: Shutdown ---")
        shared_speed.value     = 0
        shared_turn_bias.value = 0.0
        tactical_sleep(8, is_running)

    except SandTrapException:
        print("\n[brain] sand trap interrupt — running extraction then resuming")
        try:
            state_sand_extraction()
            # After extraction, continue with tripod forward
            shared_gait_id.value   = 0
            shared_speed.value     = 600
            shared_turn_bias.value = 0.0
            tactical_sleep(10, is_running)
            shared_speed.value = 0
        except (EmergencyStopException, SandTrapException, KeyboardInterrupt):
            pass
    except EmergencyStopException as e:
        print(f"\n[brain] emergency stop: {e}")
    except KeyboardInterrupt:
        print("\n[brain] operator interrupt")
    finally:
        is_running.value = False
        print("[brain] waiting for heart to stop...")
        gait_process.join(timeout=15)
        if gait_process.is_alive():
            print("[brain] heart unresponsive — force terminating")
            gait_process.terminate()
        print("done")
