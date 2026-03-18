#!/usr/bin/env python3
"""
Hexapod rover kinematics - STS3215 servos, Raspberry Pi 3B+

Two processes:
  Brain  - mission sequencer, runs gait phases and recovery behaviors
  Heart  - 50Hz kinematics loop, Buehler clock math + LERP smoothing

Hardware: 6x STS3215 12V servos on /dev/ttyUSB1 @ 1Mbaud
No software angle offsets - clearance is managed physically.

Usage:
  sudo python3 final_full_gait_test.py [MODE]

Modes:
  (no args)              Demo mode - full maneuver showcase (all gaits + stances)
  --competition          Autonomous nav - 8-state FSM, Arduino sensors, terrain adaptation
  --competition-dry-run  Autonomous nav with speed=0 - sensors active, logs transitions only
  --test-competition     Timed fallback - quad@400 45s then wave@350 30s (no sensors)
  --test-tripod          Tripod gait only - forward, carve, pivot, reverse
  --test-quad            Quadruped gait only - forward, carve, pivot, reverse, walking tall, stealth crawl
  --test-wave            Wave gait only - forward, carve, pivot, reverse, COG shift hill climb
  --test-recovery        Recovery behaviors only - wiggle + self-right roll
  --test-arduino         Serial reader test - print 20 Arduino frames, report health (no Heart)
  --test-sensors         Sensor processing test - classify sectors, cliff, IMU, turn intensity
  --test-nav             Full nav pipeline test - Arduino -> parse -> classify -> FSM transitions
"""

import time
import sys
import signal
import datetime
import os
import multiprocessing as mp
import gc
import math
import collections
import socket
import threading

# --- OPTIONAL: pyserial for autonomous nav (Arduino sensor hub) ---
try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

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
PORT_NAME      = "/dev/ttyUSB1"
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
    1: 3447, 2: 955, 3: 1420,
    4: 1569, 5: 3197, 6: 3175,
}

# Servo ID -> shared_servo_loads array index (0-based, clean mapping)
SERVO_LOAD_INDEX = {sid: i for i, sid in enumerate(ALL_SERVOS)}

# Feedback Gains
KP_PHASE        = 12.0
KAPPA_TRANSITION = 12.0   # exponential ramp decay rate for gait transitions (paper Eq. 20-21, kappa < 0 convention inverted)
STALL_THRESHOLD = 750  # raised from 600 for wet sand operation — tune from telemetry after first run
GHOST_TEMP      = 125  # STS3215 EMI artifact - bus noise returns flat 125°C (not a real reading)
OVERLOAD_PREVENTION_TIME = 1.5  # seconds — clear overload flag before 2s hardware cutoff (time-based, loop-rate invariant)
OVERLOAD_MAX_CYCLES = 10  # max TE cycles per servo per session — limits EPROM wear

# --- KINEMATIC GAIT DICTIONARIES ---
GAITS = {
    0: {  # TRIPOD — do NOT use on slopes >10°: peak servo torque exceeds rated 10 kg·cm.
        #         Use Wave with COG shift for ramps (see Phase 3 hill climb).
        'duty': 0.5,
        'offsets': {2: 0.0, 6: 0.0, 4: 0.0,  1: 0.5, 3: 0.5, 5: 0.5}
    },
    1: {  # WAVE
        'duty': 0.75,
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
def cpg_exp_ramp(current, target, kappa, dt):
    """Exponential ramp toward target (paper Eq. 20-21). Returns new value."""
    return current + (target - current) * (1.0 - math.exp(-kappa * dt))

def cpg_exp_ramp_circular(current, target, kappa, dt, period=1.0):
    """Exponential ramp with circular wrapping. Returns new value."""
    half = period / 2.0
    diff = (target - current + half) % period - half
    new_target = current + diff
    result = cpg_exp_ramp(current, new_target, kappa, dt)
    return result % period

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
                shared_stall_override, shared_roll_mode, shared_voltage, is_running):
    """
    50Hz kinematics loop. Runs as a separate process.
    GroupSyncRead for position and load every tick. Temp/voltage/current
    rotate one servo per tick to avoid unnecessary bus traffic.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    def _sigterm_handler(signum, frame):
        """Allow finally block to run on SIGTERM from Brain."""
        is_running.value = False

    signal.signal(signal.SIGTERM, _sigterm_handler)

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
    stall_sustained_start = {sid: 0.0 for sid in ALL_SERVOS}  # timestamp of sustained stall onset (0.0 = not timing)
    overload_cycle_count = {sid: 0 for sid in ALL_SERVOS}
    te_dwell_remaining = {sid: 0 for sid in ALL_SERVOS}
    last_actual_phases = {sid: 0.0 for sid in ALL_SERVOS}

    telemetry_sid   = ALL_SERVOS[0]   # Which single servo gets full telemetry this tick
    telemetry_index = 0
    voltage_reading = 12.0
    volt_dip_counter   = 0
    temp_spike_counters = {sid: 0 for sid in ALL_SERVOS}  # per-servo overtemp tracking
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
                # Tier 2 diagnostic fields (appended to existing format)
                jit_max = max(dt_samples) if dt_samples else 0.0
                jit_mean = sum(dt_samples) / len(dt_samples) if dt_samples else 0.0
                jit_std = (sum((x - jit_mean) ** 2 for x in dt_samples) / len(dt_samples)) ** 0.5 if dt_samples else 0.0
                gov_pct = int(round(gov_clamp_count * 100 / max(1, len(dt_samples)))) if dt_samples else 0
                te_str = ",".join(str(te_cycle_counts[sid]) for sid in ALL_SERVOS)
                stall_dur_parts = []
                for s in ALL_SERVOS:
                    if stall_entry_time[s] > 0:
                        sd = time.perf_counter() - stall_entry_time[s]
                        stall_dur_parts.append(f"s{s}={sd:.1f}s")
                stall_dur_str = ",".join(stall_dur_parts) if stall_dur_parts else "none"
                f.write(f"[{ts}] G:{gait_name} Spd:{fsm_speed} Trn:{fsm_turn:.2f} | "
                        f"V:{volt:.2f}V Vc:{volt_dip_ctr:03d} | "
                        f"T:{temps_str} Tc:{temp_spike_ctr:02d} | "
                        f"L:{loads_str} | Stall:{stall_ids} | "
                        f"A:{total_amps:.2f} | Loop:{loop_ms:.1f}ms"
                        f"{ghost_tag}{vwarn_tag}{twarn_tag}"
                        f" | Pdelta:{pos_delta_accum} Jit:{jit_max:.1f}/{jit_std:.1f}ms"
                        f" Gov:{gov_pct:02d}% TE:{te_str} PhErr:{ref_phase_error:+.1f}°"
                        f" Comm:{comm_fail_streak:02d} StDur:{stall_dur_str}\n")
        except:
            pass

    _stall_log_buffer = []

    def log_stall_event(sid, entering, load):
        """Buffer stall events in memory — flushed in Heart finally block.
        entering=True → stall declared (counter hit 3). entering=False → stall cleared (counter hit 0).
        """
        try:
            ts    = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            event = "ENTRY" if entering else "EXIT "
            total_stalled = sum(1 for s in is_stalled.values() if s)
            pos = raw_positions.get(sid, 0)
            base = (f"[{ts}] [STALL-{event}] servo=s{sid} load={load} "
                    f"thresh={STALL_THRESHOLD} concurrent_stalls={total_stalled} "
                    f"pos={pos} cmd_spd={shared_speed.value} ff_spd={ref_ff_speed:.1f}")
            if not entering:
                dur = (time.perf_counter() - stall_entry_time[sid]) * 1000 if stall_entry_time[sid] > 0 else 0
                te_fired = te_during_stall.get(sid, 0) > 0
                _stall_log_buffer.append(f"{base} dur={dur:.0f}ms te_fired={te_fired}")
            else:
                _stall_log_buffer.append(base)
        except:
            pass

    def flush_stall_log():
        """Write buffered stall events to disk. Called once at shutdown."""
        if not _stall_log_buffer:
            return
        try:
            with open(LOG_FILE, "a") as f:
                for line in _stall_log_buffer:
                    f.write(line + "\n")
            _stall_log_buffer.clear()
        except:
            pass

    def flush_ring_buffer(tag, count=50, trigger_desc=""):
        """Dump last `count` ring buffer entries to LOG_FILE with [tag] header."""
        try:
            entries = list(ring_buffer)[-count:]
            if not entries:
                return
            lines = [f"[{tag}] {len(entries)} ticks, trigger: {trigger_desc}\n"]
            for e in entries:
                (t_off, p1, p2, p3, p4, p5, p6,
                 l1, l2, l3, l4, l5, l6,
                 cmd_spd, cmd_trn, ff_spd, ph_ang, gid,
                 st_bm, ldt, gov, xf, zf) = e
                lines.append(
                    f"T+{t_off:08.3f} P:{p1:04d},{p2:04d},{p3:04d},{p4:04d},{p5:04d},{p6:04d} "
                    f"L:{l1:03d},{l2:03d},{l3:03d},{l4:03d},{l5:03d},{l6:03d} "
                    f"Cmd:{cmd_spd:04d}/{cmd_trn:+.2f} FF:{ff_spd:05.1f} Ph:{ph_ang:05.1f} "
                    f"G:{gid} St:{st_bm:02d} dt:{ldt:04.1f} Gov:{int(gov)} Dir:{xf:+d}/{zf:+d}\n"
                )
            with open(LOG_FILE, "a") as f:
                f.write("".join(lines))
        except:
            pass

    def init_and_align_servos():
        """Boot: snap all legs to home in position mode, then switch to velocity mode."""
        port_retries = 0
        while is_running.value:
            if port_handler.openPort() and port_handler.setBaudRate(BAUDRATE):
                print(f"[heart] port {PORT_NAME} opened at {BAUDRATE} baud")
                break
            port_retries += 1
            if port_retries % 5 == 0:
                print(f"[heart] waiting for {PORT_NAME}... ({port_retries} retries)")
            if port_retries >= 30:
                print(f"[heart] FATAL: {PORT_NAME} not available after 30s")
                is_running.value = False
                return
            time.sleep(1.0)

        port_handler.clearPort()
        print("[heart] snapping legs to home...")
        for sid in ALL_SERVOS:
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
            dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 0)  # Position
            if dxl_comm_result != 0:
                print(f"[init] WARNING: servo {sid} mode write failed (err={dxl_comm_result})")
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_ACCEL, 20)
            dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, sid, ADDR_TORQUE_LIMIT, 400)
            if dxl_comm_result != 0:
                print(f"[init] WARNING: servo {sid} torque_limit write failed (err={dxl_comm_result})")
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 800)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_POSITION, HOME_POSITIONS[sid])
            dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 1)
            if dxl_comm_result != 0:
                print(f"[init] WARNING: servo {sid} torque_enable write failed (err={dxl_comm_result})")

        time.sleep(3.0)

        print("[heart] switching to velocity mode")
        for sid in ALL_SERVOS:
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
            dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 1)  # Velocity
            if dxl_comm_result != 0:
                print(f"[init] WARNING: servo {sid} velocity mode write failed (err={dxl_comm_result})")
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_ACCEL, 0)
            dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, sid, ADDR_TORQUE_LIMIT, 1000)
            if dxl_comm_result != 0:
                print(f"[init] WARNING: servo {sid} torque_limit write failed (err={dxl_comm_result})")
            dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 1)
            if dxl_comm_result != 0:
                print(f"[init] WARNING: servo {sid} torque_enable write failed (err={dxl_comm_result})")

        # Pre-register all servos for position and load sync reads
        for sid in ALL_SERVOS:
            gsread_pos.addParam(sid)
            gsread_load.addParam(sid)

    # --- DIAGNOSTIC LOGGING INFRASTRUCTURE (Tier 1 + accumulators) ---
    heart_start_mono = time.monotonic()
    ring_buffer = collections.deque(maxlen=100)
    raw_positions = {sid: HOME_POSITIONS[sid] for sid in ALL_SERVOS}
    pos_prev = {sid: 0 for sid in ALL_SERVOS}
    pos_delta_accum = 0
    dt_samples = collections.deque(maxlen=50)
    gov_clamp_count = 0
    te_cycle_counts = {sid: 0 for sid in ALL_SERVOS}
    te_during_stall = {sid: 0 for sid in ALL_SERVOS}
    stall_entry_time = {sid: 0.0 for sid in ALL_SERVOS}
    overrun_streak = 0
    servo_comm_fails = {sid: 0 for sid in ALL_SERVOS}
    servo_disabled = {sid: False for sid in ALL_SERVOS}
    ref_ff_speed = 0.0
    ref_phase_angle = 0.0
    ref_phase_error = 0.0
    gov_active = False

    # --- Tier 4: Session Header ---
    try:
        with open(LOG_FILE, "a") as f:
            f.write("=" * 50 + "\n")
            f.write(f"=== HEXAPOD SESSION {datetime.datetime.now().strftime('%Y-%m-%dT%H:%M:%S')} ===\n")
            f.write(f"Pi: {socket.gethostname()} | Py: {sys.version.split()[0]} | PID: {os.getpid()}\n")
            f.write(f"Constants: STALL_THRESH={STALL_THRESHOLD} V_SCALAR={VELOCITY_SCALAR} "
                    f"KP_PHASE={KP_PHASE} OVERLOAD_TIME={OVERLOAD_PREVENTION_TIME}s\n")
            f.write(f"Voltage: {voltage_reading:.1f}V | Home: {HOME_POSITIONS}\n")
            f.write(f"Bus: {PORT_NAME} @ {BAUDRATE} | Servos: {ALL_SERVOS}\n")
            f.write("=" * 50 + "\n")
    except:
        pass

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
    prev_gait_id     = 0  # track gait switches for phase snap (Fix 66)

    prev_loop_ms    = 0.0    # last frame's measured elapsed time — logged in 1Hz heartbeat
    ghost_event_flag = False  # latched True when 125C artifact seen; reset after each log tick
    comm_fail_streak = 0     # consecutive ticks with zero position reads — bus disconnect detection

    try:
        init_and_align_servos()

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
            pos_read_count = 0
            for sid in ALL_SERVOS:
                if gsread_pos.isAvailable(sid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                    pos_read_count += 1
                    servo_comm_fails[sid] = 0
                    pos  = gsread_pos.getData(sid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                    raw_positions[sid] = pos
                    diff = pos - HOME_POSITIONS[sid]
                    angle = ((diff / ENCODER_RESOLUTION) * 360.0 * DIRECTION_MAP[sid]) % 360
                    actual_phases[sid]      = angle
                    last_actual_phases[sid] = angle
                else:
                    actual_phases[sid] = last_actual_phases[sid]
                    servo_comm_fails[sid] += 1
                    if servo_comm_fails[sid] == 50:
                        print(f"[heart] servo {sid} unresponsive for 1s — zeroing speed")
                        servo_disabled[sid] = True

            # Bus disconnect detection — if no servo responds for 10 consecutive
            # ticks (200ms at 50Hz), the USB link is dead.  STS3215 servos in
            # velocity mode keep spinning at last speed, so detect fast.
            if pos_read_count == 0:
                comm_fail_streak += 1
            else:
                comm_fail_streak = 0
            if comm_fail_streak >= 10:
                print("[heart] serial bus error — no servo responding for 200ms")
                # Tier 3d: log bus disconnect to file (was stdout only)
                try:
                    with open(LOG_FILE, "a") as f:
                        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        last_pos = " ".join(f"s{s}={raw_positions.get(s, 0)}" for s in ALL_SERVOS)
                        f.write(f"[{ts}] [BUS-DISCONNECT] streak={comm_fail_streak} last_pos=[{last_pos}]\n")
                except:
                    pass
                flush_ring_buffer("BUS-DISCONNECT-CONTEXT", 100, "serial bus lost for 200ms")
                is_running.value = False
                break

            # Telemetry read - rotate one servo per tick for full telemetry,
            # read all servos for load (needed for stall detection every tick)
            gsread_load.txRxPacket()
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
                            if is_stalled[sid]:
                                stall_entry_time[sid] = time.perf_counter()
                            else:
                                te_during_stall[sid] = 0
                                stall_entry_time[sid] = 0.0
                    else:
                        # Stall override active (e.g. inverted roll): high load is
                        # intentional floor contact - do not cut leg speed.
                        stall_counters[sid] = 0
                        is_stalled[sid]     = False

            # Overload prevention: cycle TE to clear hardware overload flag before 2s cutoff.
            # Decoupled from stall_active — runs on raw load magnitude so it still fires
            # during stall_override (e.g. self-right roll) where stall detection is suppressed
            # but hardware overcurrent protection can still trip.
            # Time-based (not frame-based) so margin is invariant to loop rate jitter.
            now = time.perf_counter()
            for sid in ALL_SERVOS:
                # TE dwell: space off/on cycle across 5 frames (100ms) instead of 2
                if te_dwell_remaining[sid] > 0:
                    te_dwell_remaining[sid] -= 1
                    if te_dwell_remaining[sid] == 0:
                        packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 1)
                    continue  # skip normal TE logic while dwelling

                load_mag = shared_servo_loads[SERVO_LOAD_INDEX[sid]]
                if load_mag > STALL_THRESHOLD:
                    if stall_sustained_start[sid] == 0.0:
                        stall_sustained_start[sid] = now
                    elif now - stall_sustained_start[sid] >= OVERLOAD_PREVENTION_TIME:
                        if overload_cycle_count[sid] < OVERLOAD_MAX_CYCLES:
                            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
                            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 1)
                            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_TORQUE_LIMIT, 1000)
                            te_dwell_remaining[sid] = 5  # 100ms dwell before re-enable
                            overload_cycle_count[sid] += 1
                            te_cycle_counts[sid] += 1
                            te_during_stall[sid] += 1
                            try:
                                stall_dur = now - stall_sustained_start[sid]
                                with open(LOG_FILE, "a") as f:
                                    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                                    f.write(f"[{ts}] [TE-CYCLE] servo=s{sid} stall_dur={stall_dur:.2f}s "
                                            f"load={load_mag} cycle={overload_cycle_count[sid]}/{OVERLOAD_MAX_CYCLES}\n")
                            except:
                                pass
                        stall_sustained_start[sid] = 0.0
                else:
                    stall_sustained_start[sid] = 0.0

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
                shared_voltage.value = voltage_reading

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
                    temp_spike_counters[telemetry_sid] += 1
                else:
                    temp_spike_counters[telemetry_sid] = max(0, temp_spike_counters[telemetry_sid] - 1)
            temp_spike_counter = max(temp_spike_counters.values())

            if voltage_reading < VOLTAGE_MIN:
                volt_dip_counter += 1
            else:
                volt_dip_counter = max(0, volt_dip_counter - 1)

            if temp_spike_counter > 15 or volt_dip_counter > 150:  # 150 ticks = 3s debounce — extended for cold-weather sag during heavy terrain
                if temp_spike_counter > 15:
                    hot_sid = max(temp_spike_counters, key=temp_spike_counters.get)
                    reason = f"overtemp servo={hot_sid} temp={temp_per_servo[hot_sid]}C"
                else:
                    reason = "undervolt"
                print(f"[heart] safety shutdown - {reason} volt={voltage_reading:.1f}V")
                log_telemetry(voltage_reading,
                              sum(current_per_servo.values()) / 1000.0,
                              shared_speed.value, shared_turn_bias.value,
                              volt_dip_counter, temp_spike_counter, prev_loop_ms, ghost_event_flag)
                # Tier 3c: flush full ring buffer on safety shutdown
                try:
                    with open(LOG_FILE, "a") as f:
                        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        temps_at_shutdown = " ".join(f"s{s}={temp_per_servo[s]}C" for s in ALL_SERVOS)
                        loads_at_shutdown = " ".join(f"s{s}={shared_servo_loads[SERVO_LOAD_INDEX[s]]}" for s in ALL_SERVOS)
                        f.write(f"[{ts}] [SHUTDOWN] trigger={reason} temps=[{temps_at_shutdown}] loads=[{loads_at_shutdown}]\n")
                except:
                    pass
                flush_ring_buffer("SHUTDOWN-CONTEXT", 100, reason)
                is_running.value = False
                break

            if loop_start - last_log_time > 1.0:
                log_telemetry(voltage_reading,
                              sum(current_per_servo.values()) / 1000.0,
                              shared_speed.value, shared_turn_bias.value,
                              volt_dip_counter, temp_spike_counter, prev_loop_ms, ghost_event_flag)
                ghost_event_flag = False  # reset after logging — new ghost events latch fresh
                last_log_time = loop_start
                # Reset per-1Hz accumulators
                pos_delta_accum = 0
                gov_clamp_count = 0
                for _sid in ALL_SERVOS:
                    te_cycle_counts[_sid] = 0

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
            current_gait_id = shared_gait_id.value
            t_duty      = max(0.01, min(0.99, gait_params['duty']))

            t_offsets = gait_params['offsets']

            # Snap on gait switch -- eliminate LERP convergence lag (Fix 66)
            if current_gait_id != prev_gait_id:
                smooth_duty = t_duty
                smooth_imp_start = shared_impact_start.value
                smooth_imp_end   = shared_impact_end.value
                for sid in ALL_SERVOS:
                    smooth_offsets[sid] = t_offsets[sid]
                prev_gait_id = current_gait_id
            else:
                # Exponential ramp for smooth convergence (paper Eq. 20-21)
                smooth_duty = cpg_exp_ramp(smooth_duty, t_duty, KAPPA_TRANSITION, real_dt)
                for sid in ALL_SERVOS:
                    smooth_offsets[sid] = cpg_exp_ramp_circular(
                        smooth_offsets[sid], t_offsets[sid], KAPPA_TRANSITION, real_dt)

            # ----------------------------------------------------------
            # 3. DRIVE CALCULATIONS & SAFETY GOVERNOR
            # ----------------------------------------------------------
            hz_L = smooth_hz + smooth_turn
            hz_R = smooth_hz - smooth_turn

            stance_sweep = (smooth_imp_end - smooth_imp_start + 180) % 360 - 180
            air_sweep    = get_air_sweep(stance_sweep)

            max_safe_hz = (2800.0 / VELOCITY_SCALAR * (1.0 - smooth_duty)) / max(5.0, abs(air_sweep))
            gov_active = (abs(hz_L) > max_safe_hz + 0.001 or abs(hz_R) > max_safe_hz + 0.001)
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

                if is_stalled[sid] or servo_disabled[sid] or abs(leg_hz) < 0.001:
                    final_speed = 0
                    if sid == 1:
                        ref_ff_speed = 0.0
                        ref_phase_angle = actual_phases.get(sid, 0.0)
                        ref_phase_error = 0.0
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

                    ff_speed = min(499.0, deg_per_sec * VELOCITY_SCALAR)

                    if sid == 1:
                        ref_ff_speed = ff_speed
                        ref_phase_angle = target_phase
                        ref_phase_error = (target_phase - current_phase + 180) % 360 - 180

                    error       = target_phase - current_phase
                    short_error = (error + 180) % 360 - 180

                    if t_leg > smooth_duty:
                        # Air phase: guard prevents backward-error signal during fast
                        # forward return sweep.  Fires only in air phase to avoid
                        # runaway when the leg overshoots during stance.
                        # Linear ramp over 10° transition zone eliminates discontinuity
                        # at the ±90° boundary (was a hard re-wrap).
                        transition_deg = 10.0
                        if not is_rev_leg and short_error < -80:
                            depth = -80.0 - short_error  # 0 at -80, 10 at -90, >10 beyond
                            blend = min(1.0, depth / transition_deg)
                            short_error += 360 * blend
                        elif is_rev_leg and short_error > 80:
                            depth = short_error - 80.0   # 0 at +80, 10 at +90, >10 beyond
                            blend = min(1.0, depth / transition_deg)
                            short_error -= 360 * blend

                    raw_speed = ff_speed + (short_error * KP_PHASE)

                    if is_rev_leg:            raw_speed = -raw_speed
                    if sid in LEFT_SERVOS and not shared_roll_mode.value:
                        raw_speed = -raw_speed

                    final_speed = max(-3000, min(3000, int(raw_speed)))

                abs_speed = int(abs(final_speed))
                speed_val = (abs_speed | 0x8000) if final_speed < 0 else abs_speed
                group_sync_write.addParam(sid, [speed_val & 0xff, (speed_val >> 8) & 0xff])

            group_sync_write.txPacket()
            group_sync_write.clearParam()

            # ----------------------------------------------------------
            # 5. RING BUFFER + ACCUMULATOR UPDATES (no disk I/O)
            # ----------------------------------------------------------
            tick_mono = time.monotonic() - heart_start_mono
            loop_dt_ms = (time.perf_counter() - loop_start) * 1000
            stall_bitmap = sum((1 << i) for i, sid in enumerate(ALL_SERVOS) if is_stalled[sid])
            ring_buffer.append((
                tick_mono,
                raw_positions.get(2, 0), raw_positions.get(3, 0), raw_positions.get(4, 0),
                raw_positions.get(1, 0), raw_positions.get(6, 0), raw_positions.get(5, 0),
                shared_servo_loads[SERVO_LOAD_INDEX[2]], shared_servo_loads[SERVO_LOAD_INDEX[3]],
                shared_servo_loads[SERVO_LOAD_INDEX[4]], shared_servo_loads[SERVO_LOAD_INDEX[1]],
                shared_servo_loads[SERVO_LOAD_INDEX[6]], shared_servo_loads[SERVO_LOAD_INDEX[5]],
                shared_speed.value, shared_turn_bias.value,
                ref_ff_speed, ref_phase_angle, shared_gait_id.value,
                stall_bitmap, loop_dt_ms, gov_active,
                shared_x_flip.value, shared_z_flip.value
            ))

            # Position delta accumulator (handles encoder wraparound)
            for sid in ALL_SERVOS:
                rp = raw_positions.get(sid, 0)
                if pos_prev[sid] > 0:
                    delta = abs(rp - pos_prev[sid])
                    if delta > 2048:
                        delta = 4096 - delta
                    pos_delta_accum += delta
                pos_prev[sid] = rp

            dt_samples.append(loop_dt_ms)
            if gov_active:
                gov_clamp_count += 1

            # ----------------------------------------------------------
            # 6. PRECISION TIMING
            # ----------------------------------------------------------
            elapsed_time = time.perf_counter() - loop_start
            prev_loop_ms = elapsed_time * 1000  # stored for next 1Hz log tick
            sleep_time   = target_dt - elapsed_time
            if sleep_time > 0.002:
                time.sleep(sleep_time - 0.002)
            while time.perf_counter() - loop_start < target_dt:
                pass

            # Loop overrun detection
            if prev_loop_ms > 18.0:
                overrun_streak += 1
                try:
                    with open(LOG_FILE, "a") as f:
                        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        f.write(f"[{ts}] [OVERRUN] dt={prev_loop_ms:.1f}ms at T+{tick_mono:.3f}\n")
                except:
                    pass
                if overrun_streak >= 3:
                    flush_ring_buffer("OVERRUN-CONTEXT", 50, f"{overrun_streak} consecutive overruns")
            else:
                overrun_streak = 0

    except Exception as e:
        print(f"[heart] fatal error: {e}")
    finally:
        flush_stall_log()
        print("[heart] shutting down, returning legs to home...")
        port_handler.clearPort()
        for sid in ALL_SERVOS:
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 0)
        time.sleep(0.1)
        for sid in ALL_SERVOS:
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 0)
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_ACCEL, 20)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_TORQUE_LIMIT, 400)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 800)
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_POSITION, HOME_POSITIONS[sid])
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 1)
        time.sleep(3.0)
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
    shared_roll_mode    = mp.Value('b', False)     # When True, LEFT_SERVOS negation is bypassed so
                                                   # all legs spin same physical direction for rolling.
    shared_voltage      = mp.Value('f', 12.0)      # Latest battery voltage from Heart telemetry,
                                                   # readable by Brain for pre-roll voltage check.
    is_running          = mp.Value('b', True)

    for i in range(len(ALL_SERVOS)):
        shared_servo_loads[i] = -1

    gait_process = mp.Process(target=gait_worker, args=(
        shared_speed, shared_x_flip, shared_z_flip, shared_turn_bias,
        shared_gait_id, shared_impact_start, shared_impact_end,
        shared_servo_loads, shared_heartbeat, shared_stall_override,
        shared_roll_mode, shared_voltage, is_running
    ))
    gait_process.start()

    # Convenience wrapper so tactical calls don't repeat the process arg
    def tsleep(duration):
        tactical_sleep(duration, is_running, gait_process)

    # --- BRAIN-SIDE DIAGNOSTIC LOGGING ---
    brain_start_mono = time.monotonic()

    def brain_log(msg):
        """Append a [BRAIN]-tagged message to LOG_FILE with ms-precision timestamp."""
        try:
            t_off = time.monotonic() - brain_start_mono
            with open(LOG_FILE, "a") as f:
                f.write(f"[BRAIN] T+{t_off:.3f} {msg}\n")
        except:
            pass

    def set_gait_state(speed=None, turn=None, gait=None, impact_start=None,
                       impact_end=None, x_flip=None, z_flip=None, step_name=""):
        """Log before→after diff then write changed shared variables. Behavior is identical
        to direct writes but every state change is now recorded in the log file."""
        parts = []
        if speed is not None and speed != shared_speed.value:
            parts.append(f"speed:{shared_speed.value}→{speed}")
            shared_speed.value = speed
        if turn is not None and abs(turn - shared_turn_bias.value) > 0.001:
            parts.append(f"turn:{shared_turn_bias.value:.2f}→{turn:.2f}")
            shared_turn_bias.value = turn
        if gait is not None and gait != shared_gait_id.value:
            parts.append(f"gait:{shared_gait_id.value}→{gait}")
            shared_gait_id.value = gait
        if impact_start is not None and impact_start != shared_impact_start.value:
            parts.append(f"imp_s:{shared_impact_start.value}→{impact_start}")
            shared_impact_start.value = impact_start
        if impact_end is not None and impact_end != shared_impact_end.value:
            parts.append(f"imp_e:{shared_impact_end.value}→{impact_end}")
            shared_impact_end.value = impact_end
        if x_flip is not None and x_flip != shared_x_flip.value:
            parts.append(f"x_flip:{shared_x_flip.value}→{x_flip}")
            shared_x_flip.value = x_flip
        if z_flip is not None and z_flip != shared_z_flip.value:
            parts.append(f"z_flip:{shared_z_flip.value}→{z_flip}")
            shared_z_flip.value = z_flip
        if parts:
            gait_name = {0: "tripod", 1: "wave", 2: "quad"}.get(shared_gait_id.value, "?")
            brain_log(f"step={step_name} {' '.join(parts)} | "
                      f"gait:{gait_name} impact:{shared_impact_start.value}/{shared_impact_end.value} "
                      f"dir:{shared_x_flip.value:+d}/{shared_z_flip.value:+d}")

    # --- EXPANDED TACTICAL RECOVERY BEHAVIORS ---
    def state_recovery_wiggle(duration=10):
        """Oscillate direction rapidly to break static friction / unstick a jammed leg."""
        print("[recovery] wiggling to break friction...")

        # Tier 3g: log wiggle start with stall bitmap and positions
        stall_sids = [f"s{sid}" for sid in ALL_SERVOS
                      if shared_servo_loads[SERVO_LOAD_INDEX[sid]] > STALL_THRESHOLD]
        start_loads = [str(shared_servo_loads[i]) for i in range(len(ALL_SERVOS))]
        brain_log(f"[WIGGLE-START] stalled={','.join(stall_sids) or 'none'} ({len(stall_sids)}/6) "
                  f"loads:{','.join(start_loads)}")
        wiggle_start_time = time.time()

        # Suppress stall detection — Heart zeros stalled legs' speed, making wiggle a no-op
        shared_stall_override.value = True
        saved_gait_id = shared_gait_id.value
        shared_gait_id.value = 0   # Tripod — 50% duty, best wiggle effectiveness

        # Reset to standard window - avoids inheriting stale state (e.g. COG Shift 315/15)
        set_gait_state(impact_start=330, impact_end=30, step_name="wiggle_window_reset")

        # Phase 4 zeros speed before calling this, so we own the speed here
        set_gait_state(speed=350, step_name="wiggle_engage")

        end_time    = time.time() + duration
        try:
            while time.time() < end_time:
                if not is_running.value:
                    raise EmergencyStopException("Hardware aborted.")
                if not gait_process.is_alive():
                    raise EmergencyStopException("Heart process died during recovery wiggle.")

                set_gait_state(x_flip=-1, step_name="wiggle_flip_neg")
                tsleep(0.3)
                set_gait_state(x_flip=1, step_name="wiggle_flip_pos")
                tsleep(0.3)
        finally:
            set_gait_state(speed=0, x_flip=1, step_name="wiggle_stop")
            shared_stall_override.value = False
            shared_gait_id.value = saved_gait_id

        # Tier 3g: log wiggle end with outcome
        wiggle_dur = (time.time() - wiggle_start_time) * 1000
        end_stall_sids = [f"s{sid}" for sid in ALL_SERVOS
                          if shared_servo_loads[SERVO_LOAD_INDEX[sid]] > STALL_THRESHOLD]
        cleared = [s for s in stall_sids if s not in end_stall_sids]
        remaining = [s for s in stall_sids if s in end_stall_sids]
        end_loads = [str(shared_servo_loads[i]) for i in range(len(ALL_SERVOS))]
        brain_log(f"[WIGGLE-END] dur={wiggle_dur:.0f}ms cleared={','.join(cleared) or 'none'} "
                  f"remaining={','.join(remaining) or 'none'} loads:{','.join(end_loads)}")
        print("[recovery] wiggle done")

    def stall_tsleep(duration, wiggle_threshold=3):
        """
        Break long tactical sleeps into 3-second chunks.
        After each chunk, count legs with load > STALL_THRESHOLD.
        If >= wiggle_threshold legs are stalled, fire an immediate recovery
        wiggle then resume the manoeuvre at the saved speed/turn/window.
        Closes the 89-second gap before Phase 4 on deep-sand entrapment.
        """
        chunk = 3.0
        slept = 0.0
        while slept < duration:
            tsleep(min(chunk, duration - slept))
            slept += chunk
            stalled = sum(
                1 for i in range(len(ALL_SERVOS))
                if shared_servo_loads[i] > STALL_THRESHOLD
            )
            is_pivot = (abs(shared_speed.value) < 1 and abs(shared_turn_bias.value) > 0.1)
            if stalled >= wiggle_threshold and not is_pivot:
                print(f"[brain] stall_tsleep: {stalled} legs stalled — early wiggle")
                saved_speed  = shared_speed.value
                saved_turn   = shared_turn_bias.value
                saved_imp_s  = shared_impact_start.value
                saved_imp_e  = shared_impact_end.value
                saved_x_flip = shared_x_flip.value
                set_gait_state(speed=0, turn=0.0, step_name="stall_tsleep_pre_wiggle")
                state_recovery_wiggle()
                set_gait_state(speed=saved_speed, turn=saved_turn,
                               impact_start=saved_imp_s, impact_end=saved_imp_e,
                               x_flip=saved_x_flip, step_name="stall_tsleep_restore")
                # Post-wiggle re-check — one retry max
                time.sleep(0.5)
                recheck = sum(
                    1 for i in range(len(ALL_SERVOS))
                    if shared_servo_loads[i] > STALL_THRESHOLD
                )
                if recheck >= wiggle_threshold and not is_pivot:
                    print("[stall_tsleep] still stalled after wiggle — retrying")
                    set_gait_state(speed=0, turn=0.0, step_name="stall_tsleep_pre_wiggle_retry")
                    state_recovery_wiggle()
                    set_gait_state(speed=saved_speed, turn=saved_turn,
                                   impact_start=saved_imp_s, impact_end=saved_imp_e,
                                   x_flip=saved_x_flip, step_name="stall_tsleep_restore_retry")

    # =====================================================================
    #  AUTONOMOUS NAVIGATION — Serial Reader (Phase 1)
    # =====================================================================

    # --- Arduino serial config ---
    ARDUINO_PORT = "/dev/ttyACM0"
    ARDUINO_BAUD = 115200

    # --- Nav tunable constants ---
    CRUISE_SPEED = 400
    TRIPOD_CRUISE_SPEED = 450
    SLOW_SPEED = 200
    BACKWARD_SPEED = 200
    MAX_TURN_BIAS = 0.25
    PIVOT_TURN_BIAS = 0.5
    HEADING_CORRECTION_BIAS = 0.1
    STALL_LOAD_THRESHOLD_NAV = 500
    STALL_SUSTAIN_S = 1.5
    SLOPE_PITCH_DEG = 12
    HEAVY_TERRAIN_LOAD = 400
    LIGHT_TERRAIN_LOAD = 200
    TERRAIN_SUSTAIN_S = 2.0
    LOAD_ASYMMETRY_THRESHOLD = 200
    FLICKER_WINDOW_S = 2.0
    FLICKER_COUNT_THRESHOLD = 3
    MISSION_TIMEOUT_S = 90
    FINISH_WALL_DIST_CM = 10
    FINISH_WALL_SUSTAIN_S = 2.0

    # --- CSV column indices ---
    CSV_COLS = 20
    IDX_TS = 0
    IDX_FDL, IDX_FCF, IDX_FCD, IDX_FDR = 1, 2, 3, 4
    IDX_RDL, IDX_RCF, IDX_RCD, IDX_RDR = 5, 6, 7, 8
    IDX_QW, IDX_QX, IDX_QY, IDX_QZ = 9, 10, 11, 12
    IDX_AX, IDX_AY, IDX_AZ = 13, 14, 15
    IDX_GX, IDX_GY, IDX_GZ = 16, 17, 18
    IDX_UPSIDE = 19

    def _parse_arduino_csv(line):
        """Parse a 20-column CSV line from Arduino sensor hub into a frame dict.
        Returns dict or None on parse failure."""
        if not line:
            return None
        line = line.strip()
        if not line or line.startswith("timestamp_ms"):
            return None
        parts = line.split(",")
        if len(parts) != CSV_COLS:
            return None
        try:
            ts = int(parts[IDX_TS])
            dists = []
            for i in range(IDX_FDL, IDX_RDR + 1):
                v = float(parts[i])
                dists.append(v)
            qw = float(parts[IDX_QW])
            qx = float(parts[IDX_QX])
            qy = float(parts[IDX_QY])
            qz = float(parts[IDX_QZ])
            ax = float(parts[IDX_AX])
            ay = float(parts[IDX_AY])
            az = float(parts[IDX_AZ])
            gx = float(parts[IDX_GX])
            gy = float(parts[IDX_GY])
            gz = float(parts[IDX_GZ])
            upside = int(float(parts[IDX_UPSIDE]))
        except (ValueError, IndexError):
            return None

        # Upside-down remap: swap front/rear, preserve body-frame left/right
        if upside == 1:
            # Original: [FDL, FCF, FCD, FDR, RDL, RCF, RCD, RDR]
            # Remapped: [RDL, RCF, RCD, RDR, FDL, FCF, FCD, FDR]
            dists = dists[4:8] + dists[0:4]
            upside = 0

        # Clamp distances: preserve -1 sentinel, cap max at 450
        for i in range(len(dists)):
            if dists[i] == -1:
                continue
            if dists[i] > 450:
                dists[i] = 450.0

        return {
            "timestamp_ms": ts,
            "FDL": dists[0], "FCF": dists[1], "FCD": dists[2], "FDR": dists[3],
            "RDL": dists[4], "RCF": dists[5], "RCD": dists[6], "RDR": dists[7],
            "qw": qw, "qx": qx, "qy": qy, "qz": qz,
            "ax": ax, "ay": ay, "az": az,
            "gx": gx, "gy": gy, "gz": gz,
            "upside_down": upside,
        }

    class ArduinoReader:
        """Threaded reader for Arduino sensor hub CSV over serial."""

        def __init__(self, port=ARDUINO_PORT, baud=ARDUINO_BAUD):
            self._port = port
            self._baud = baud
            self._lock = threading.Lock()
            self._latest = None
            self._last_frame_time = 0.0
            self._running = True
            self._consecutive_failures = 0
            self._ser = None
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

        def _open_serial(self):
            """Open serial port. Returns True on success."""
            try:
                self._ser = serial.Serial(self._port, self._baud, timeout=0.1)
                self._consecutive_failures = 0
                return True
            except (serial.SerialException, OSError) as e:
                brain_log(f"[SERIAL] open failed: {e}")
                self._ser = None
                return False

        def _run(self):
            """Thread loop: read lines, parse, store latest frame."""
            backoff = 2.0
            max_backoff = 30.0

            while self._running:
                # Open port if needed
                if self._ser is None:
                    if not self._open_serial():
                        time.sleep(backoff)
                        backoff = min(backoff * 1.5, max_backoff)
                        continue
                    backoff = 2.0  # reset on successful open

                try:
                    raw = self._ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("ascii", errors="replace")
                    frame = _parse_arduino_csv(line)
                    if frame is not None:
                        with self._lock:
                            self._latest = frame
                            self._last_frame_time = time.monotonic()
                        self._consecutive_failures = 0
                    else:
                        self._consecutive_failures += 1
                except (serial.SerialException, OSError) as e:
                    brain_log(f"[SERIAL] read error: {e}")
                    try:
                        if self._ser:
                            self._ser.close()
                    except:
                        pass
                    self._ser = None
                    # Will reconnect on next loop iteration with backoff
                except Exception:
                    self._consecutive_failures += 1

        def get_latest(self):
            """Return the latest parsed frame dict, or None."""
            with self._lock:
                return self._latest

        @property
        def healthy(self):
            """True if a valid frame was received within the last 300ms."""
            with self._lock:
                t = self._last_frame_time
            return (time.monotonic() - t) < 0.3

        def stale_seconds(self):
            """Seconds since last valid frame. Returns inf if never received."""
            with self._lock:
                t = self._last_frame_time
            if t == 0.0:
                return float("inf")
            return time.monotonic() - t

        def stop(self):
            """Signal thread to exit and close serial port."""
            self._running = False
            try:
                if self._ser:
                    self._ser.close()
            except:
                pass

    # =====================================================================
    #  AUTONOMOUS NAVIGATION — Sensor Processing (Phase 2)
    # =====================================================================

    # Distance classification constants
    DIST_DANGER = 3   # severity 3 (highest)
    DIST_NEAR = 2
    DIST_CAUTION = 1
    DIST_CLEAR = 0
    DIST_UNKNOWN = 1  # treat unknown same as caution

    def classify_distance(cm):
        """Classify a single distance reading into severity level."""
        if cm is None:
            return DIST_UNKNOWN
        if cm == -1:
            return DIST_DANGER  # blind zone = very close
        if cm < 0:
            return DIST_UNKNOWN  # invalid
        if cm <= 20:
            return DIST_DANGER
        if cm <= 30:
            return DIST_NEAR
        if cm <= 50:
            return DIST_CAUTION
        return DIST_CLEAR

    def classify_sectors(frame):
        """Classify front, left, right sectors from frame dict.
        Returns (front_class, left_class, right_class) as severity levels."""
        front = max(classify_distance(frame["FDL"]),
                    classify_distance(frame["FCF"]),
                    classify_distance(frame["FDR"]))
        left = max(classify_distance(frame["FDL"]),
                   classify_distance(frame["RDL"]))
        right = max(classify_distance(frame["FDR"]),
                    classify_distance(frame["RDR"]))
        return front, left, right

    def speed_scale_from_front(front_class):
        """Map front sector classification to speed multiplier."""
        if front_class == DIST_CLEAR:
            return 1.0
        if front_class == DIST_CAUTION:
            return 0.7
        if front_class == DIST_NEAR:
            return 0.45
        if front_class == DIST_DANGER:
            return 0.0
        return 0.7  # UNKNOWN

    CLIFF_WARMUP = 5  # frames before cliff detection active (sensor settle)

    class CliffDetector:
        """Detects cliffs using ground EMA + delta + consecutive frame confirmation."""

        def __init__(self, alpha=0.2):
            self._alpha = alpha
            self._ground_ema = 25.0  # initial guess: ground ~25cm from sensor
            self._consecutive_front = 0
            self._consecutive_rear = 0
            self._warmup_frames = 0

        def update(self, fcd, rcd):
            """Update cliff detection with new FCD and RCD readings.
            Returns (front_cliff, rear_cliff) booleans."""
            front_cliff = self._check_cliff(fcd, is_front=True)
            rear_cliff = self._check_cliff(rcd, is_front=False)
            return front_cliff, rear_cliff

        def _check_cliff(self, reading, is_front):
            """Check single cliff sensor. Returns True if cliff confirmed."""
            self._warmup_frames += 1
            if self._warmup_frames <= CLIFF_WARMUP:
                return False

            counter_attr = "_consecutive_front" if is_front else "_consecutive_rear"
            count = getattr(self, counter_attr)

            # 300.0 = max range timeout (acoustic scatter on sand) -- treat as blind zone
            if reading >= 300.0:
                setattr(self, counter_attr, 0)
                return False

            if reading == -1:
                # Ground in blind zone (very close) — NOT a cliff. Reset.
                setattr(self, counter_attr, 0)
                return False

            if reading is None:
                # Defensive: possible cliff, increment
                setattr(self, counter_attr, count + 1)
                return count + 1 >= 2

            # Update ground EMA with valid low readings
            if 0 < reading <= 40:
                self._ground_ema = self._alpha * reading + (1 - self._alpha) * self._ground_ema

            # Cliff candidate: absolute > 30 OR delta > EMA + 10
            is_candidate = (reading > 30) or (reading > self._ground_ema + 10)

            if is_candidate:
                setattr(self, counter_attr, count + 1)
                return count + 1 >= 2
            else:
                setattr(self, counter_attr, 0)
                return False

    def compute_imu(frame):
        """Extract orientation, vibration, angular rate from frame.
        Returns dict with: pitch_deg, roll_deg, yaw_deg, upright_quality,
        accel_mag, angular_rate."""
        w, x, y, z = frame["qw"], frame["qx"], frame["qy"], frame["qz"]

        # Check for invalid quaternion (all zeros = IMU dead)
        qmag = w*w + x*x + y*y + z*z
        if qmag < 0.5:
            # IMU dead — return safe defaults
            return {
                "pitch_deg": 0.0, "roll_deg": 0.0, "yaw_deg": 0.0,
                "pitch_rad": 0.0, "roll_rad": 0.0, "yaw_rad": 0.0,
                "upright_quality": 0.0,  # dead IMU → trigger P1 STOP_SAFE
                "accel_mag": 9.81, "angular_rate": 0.0,
            }

        # Pitch and roll from quaternion
        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch_rad = math.asin(sinp)

        roll_rad = math.atan2(2.0 * (w * x + y * z),
                              1.0 - 2.0 * (x * x + y * y))

        yaw_rad = math.atan2(2.0 * (w * z + x * y),
                             1.0 - 2.0 * (y * y + z * z))

        upright_quality = min(math.cos(abs(pitch_rad)),
                              math.cos(abs(roll_rad)))

        # Vibration magnitude from accelerometer
        accel_mag = math.sqrt(frame["ax"]**2 + frame["ay"]**2 + frame["az"]**2)

        # Angular rate from gyroscope
        angular_rate = math.sqrt(frame["gx"]**2 + frame["gy"]**2 + frame["gz"]**2)

        return {
            "pitch_deg": math.degrees(pitch_rad),
            "roll_deg": math.degrees(roll_rad),
            "yaw_deg": math.degrees(yaw_rad),
            "pitch_rad": pitch_rad,
            "roll_rad": roll_rad,
            "yaw_rad": yaw_rad,
            "upright_quality": upright_quality,
            "accel_mag": accel_mag,
            "angular_rate": angular_rate,
        }

    def compute_turn_intensity(frame):
        """Compute turn intensity from diagonal distances.
        Positive = left is freer = should turn left.
        Range [-1, +1]."""
        fdl = frame["FDL"]
        fdr = frame["FDR"]
        # Replace -1 and None with 25 cm (effective distance for blind zone)
        fdl_eff = 25.0 if (fdl is None or fdl == -1) else fdl
        fdr_eff = 25.0 if (fdr is None or fdr == -1) else fdr
        return math.tanh((fdl_eff - fdr_eff) / 50.0)

    def compute_battery_mult(voltage_value):
        """Compute battery speed multiplier from shared_voltage.
        Returns 1.0 normally, 0.7 if low battery."""
        VOLTAGE_MIN = 10.5  # 3S LiPo minimum
        if voltage_value < VOLTAGE_MIN:
            return 0.0  # critical — STOP_SAFE should handle this
        if voltage_value < VOLTAGE_MIN + 0.5:
            return 0.7
        return 1.0

    def compute_servo_loads(loads_snapshot):
        """Analyze servo load array snapshot.
        Returns dict with avg_load, load_asymmetry, left_avg, right_avg."""
        if not loads_snapshot or all(v < 0 for v in loads_snapshot):
            return {"avg_load": 0, "load_asymmetry": 0, "left_avg": 0, "right_avg": 0}
        # Filter out uninitialized (-1) values
        valid = [v for v in loads_snapshot if v >= 0]
        if not valid:
            return {"avg_load": 0, "load_asymmetry": 0, "left_avg": 0, "right_avg": 0}
        avg_load = sum(valid) / len(valid)
        load_asymmetry = max(valid) - min(valid)
        # Left servos: indices 0,1,2 (servo 2,3,4); Right: indices 3,4,5 (servo 1,6,5)
        left_indices = [0, 1, 2]   # servo 2, 3, 4 (LEFT side)
        right_indices = [3, 4, 5]  # servo 1, 6, 5 (RIGHT side)
        left_vals = [loads_snapshot[i] for i in left_indices if loads_snapshot[i] >= 0]
        right_vals = [loads_snapshot[i] for i in right_indices if loads_snapshot[i] >= 0]
        left_avg = sum(left_vals) / len(left_vals) if left_vals else 0
        right_avg = sum(right_vals) / len(right_vals) if right_vals else 0
        return {
            "avg_load": avg_load,
            "load_asymmetry": load_asymmetry,
            "left_avg": left_avg,
            "right_avg": right_avg,
        }

    class FlickerTracker:
        """Track front classification flicker over a sliding window."""

        def __init__(self, window_s=FLICKER_WINDOW_S):
            self._window_s = window_s
            self._history = []  # list of (time, front_class)
            self._prev_class = None

        def update(self, front_class):
            """Record new front classification. Returns flicker count."""
            now = time.monotonic()
            self._history.append((now, front_class))
            # Prune old entries
            cutoff = now - self._window_s
            self._history = [(t, c) for t, c in self._history if t >= cutoff]
            # Count transitions
            transitions = 0
            for i in range(1, len(self._history)):
                if self._history[i][1] != self._history[i-1][1]:
                    transitions += 1
            self._prev_class = front_class
            return transitions

    # =====================================================================
    #  AUTONOMOUS NAVIGATION — State Machine (Phase 3)
    # =====================================================================

    # Nav states
    NAV_FORWARD = 0
    NAV_SLOW_FORWARD = 1
    NAV_ARC_LEFT = 2
    NAV_ARC_RIGHT = 3
    NAV_BACKWARD = 4
    NAV_PIVOT_TURN = 5
    NAV_WIGGLE = 6
    NAV_STOP_SAFE = 7

    NAV_STATE_NAMES = {
        0: "FORWARD", 1: "SLOW_FWD", 2: "ARC_L", 3: "ARC_R",
        4: "BACKWARD", 5: "PIVOT", 6: "WIGGLE", 7: "STOP_SAFE",
    }

    def is_rear_safe(frame):
        """Check if rear is clear enough for reversing.
        Returns False if any rear sensor < 20cm, == -1, or rear cliff detected."""
        if frame is None:
            return False
        rcf = frame["RCF"]
        rdl = frame["RDL"]
        rdr = frame["RDR"]
        for v in (rcf, rdl, rdr):
            if v is None or v == -1:
                return False
            if v < 20:
                return False
        return True

    class NavStateMachine:
        """8-state FSM for autonomous obstacle course navigation."""

        def __init__(self):
            self.state = NAV_FORWARD
            self.prev_state = NAV_FORWARD
            self.dwell_start = 0.0
            self.dwell_duration = 0.0
            self.hold_position_count = 0
            self.consecutive_pivot_count = 0
            self.pivot_direction = -1  # -1=left, +1=right
            self.stall_start_time = 0.0
            self.stall_count_30s = 0
            self.last_stall_time = 0.0
            self.stall_speed_mult = 1.0
            self.mission_start = time.monotonic()
            self.initial_yaw = None
            self.finished = False
            self.finish_wall_start = 0.0
            self.front_danger_frames = 0
            # Sustained condition trackers
            self._freefall_start = 0.0
            self._high_vibe_start = 0.0
            self._steep_up_start = 0.0
            self._steep_down_start = 0.0
            self._heavy_load_start = 0.0
            self._light_load_start = 0.0
            self._roll_sustained_start = 0.0
            self._asymmetry_start = 0.0
            self._impact_cooldown_until = 0.0
            self._last_stall_clear_time = time.monotonic()
            # Terrain state (held during non-FORWARD/SLOW states)
            self.terrain_gait = 2  # default quad
            self.terrain_impact_start = 330
            self.terrain_impact_end = 30
            self.terrain_mult = 1.0
            self.terrain_is_tripod = False
            self._gait_transition_until = 0.0

        def _dwell_active(self):
            """True if current dwell timer hasn't expired."""
            return (time.monotonic() - self.dwell_start) < self.dwell_duration

        def _start_dwell(self, duration):
            self.dwell_start = time.monotonic()
            self.dwell_duration = duration

        def _refresh_dwell(self, duration):
            """Extend dwell if still in danger."""
            self.dwell_start = time.monotonic()
            self.dwell_duration = duration

        def _transition(self, new_state):
            """Record state transition."""
            if new_state != self.state:
                self.prev_state = self.state
                self.state = new_state
                brain_log(f"[NAV] {NAV_STATE_NAMES.get(self.prev_state)}→{NAV_STATE_NAMES.get(new_state)}")

        def update(self, frame, imu, front_class, left_class, right_class,
                   front_cliff, rear_cliff, turn_intensity, avg_load,
                   load_asymmetry, angular_rate, accel_mag, voltage,
                   flicker_count):
            """Evaluate priorities and return (state, speed, turn, x_flip, step_name).
            Also updates terrain overlay (gait, impact_start, impact_end, terrain_mult)."""
            now = time.monotonic()
            elapsed = now - self.mission_start

            # --- Mission timeout ---
            if elapsed >= MISSION_TIMEOUT_S:
                self.finished = True
                return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_timeout")

            # --- Finish wall detection (only while moving forward) ---
            fcf = frame["FCF"] if frame else 999
            rcf_val = frame["RCF"] if frame else 999
            if (fcf != -1 and fcf is not None and 0 < fcf < FINISH_WALL_DIST_CM
                    and self.state in (NAV_FORWARD, NAV_SLOW_FORWARD)):
                if self.finish_wall_start == 0.0:
                    self.finish_wall_start = now
                elif (now - self.finish_wall_start) >= FINISH_WALL_SUSTAIN_S:
                    # Check rear is clear (it's the end wall, not boxed in)
                    if rcf_val is not None and rcf_val != -1 and rcf_val > 50:
                        self.finished = True
                        brain_log("[NAV] finish wall detected")
                        return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_finish")
                    else:
                        # Rear blocked too — boxed in, not finish wall
                        self.finish_wall_start = 0.0
            else:
                self.finish_wall_start = 0.0

            upright = imu["upright_quality"]
            pitch_deg = imu["pitch_deg"]
            roll_deg = imu["roll_deg"]

            # --- Track front danger frames for P10 (2+ consecutive) ---
            if front_class >= DIST_DANGER:
                self.front_danger_frames += 1
            else:
                self.front_danger_frames = 0

            # === LAYER 1: State transitions (priority order) ===

            # P1: Tipover
            if upright < 0.15:
                self._transition(NAV_STOP_SAFE)
                return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_stop_safe")

            # P2: Unexpected rapid rotation (not during pivot)
            if angular_rate > 1.5 and self.state != NAV_PIVOT_TURN:
                self._transition(NAV_STOP_SAFE)
                brain_log(f"[NAV] rapid rotation {angular_rate:.2f} rad/s")
                return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_stop_safe")

            # P3: Critical battery
            if voltage < 10.5:  # VOLTAGE_MIN for 3S
                self._transition(NAV_STOP_SAFE)
                brain_log(f"[NAV] critical voltage {voltage:.1f}V")
                return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_stop_safe")

            # P4: Arduino watchdog (caller handles stale detection, passes frame=None)
            # Handled at integration level — if stale > 5s, caller forces STOP_SAFE

            # P5: Freefall (accel < 8 sustained 0.5s AND upright < 0.5)
            if accel_mag < 8.0 and upright < 0.5:
                if self._freefall_start == 0.0:
                    self._freefall_start = now
                elif (now - self._freefall_start) >= 0.5:
                    self._transition(NAV_STOP_SAFE)
                    brain_log("[NAV] freefall detected")
                    return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_stop_safe")
            else:
                self._freefall_start = 0.0

            # P6: Front cliff
            if front_cliff:
                if self.state != NAV_BACKWARD:
                    self.hold_position_count = 0
                self._transition(NAV_BACKWARD)
                self._start_dwell(0.8)
                return self._backward_action(frame)

            # P7: Rear cliff
            if rear_cliff:
                self._transition(NAV_SLOW_FORWARD)
                speed = int(SLOW_SPEED * self.terrain_mult * self.stall_speed_mult)
                return (NAV_SLOW_FORWARD, speed, 0.0, 1, "nav_slow_fwd")

            # P8: Stall detection
            if avg_load > STALL_LOAD_THRESHOLD_NAV:
                if self.stall_start_time == 0.0:
                    self.stall_start_time = now
                elif (now - self.stall_start_time) >= STALL_SUSTAIN_S:
                    self.stall_start_time = 0.0
                    self.stall_count_30s += 1
                    self.last_stall_time = now
                    self._transition(NAV_WIGGLE)
                    return (NAV_WIGGLE, 0, 0.0, 1, "nav_wiggle")
            else:
                self.stall_start_time = 0.0

            # Reset stall count after 60s stall-free
            if self.stall_count_30s > 0 and (now - self.last_stall_time) > 60:
                self.stall_count_30s = 0
                self.stall_speed_mult = 1.0
                self._last_stall_clear_time = now

            # P9: Dead end (front DANGER + both sides DANGER + came from BACKWARD)
            if (front_class >= DIST_DANGER and left_class >= DIST_DANGER
                    and right_class >= DIST_DANGER
                    and self.prev_state == NAV_BACKWARD):
                if self.consecutive_pivot_count >= 2:
                    self._transition(NAV_STOP_SAFE)
                    brain_log("[NAV] 2 pivots failed — stop")
                    return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_stop_safe")
                self._pick_pivot_direction(frame)
                self._transition(NAV_PIVOT_TURN)
                self._start_dwell(1.5)
                self.consecutive_pivot_count += 1
                turn = self.pivot_direction * PIVOT_TURN_BIAS
                step = "nav_pivot_L" if self.pivot_direction < 0 else "nav_pivot_R"
                return (NAV_PIVOT_TURN, 0, turn, 1, step)

            # P10: Front DANGER (2+ frames, not dead-end)
            if self.front_danger_frames >= 2:
                if self.state != NAV_BACKWARD:
                    self.hold_position_count = 0
                self._transition(NAV_BACKWARD)
                self._start_dwell(0.8)
                return self._backward_action(frame)

            # Reset pivot count when front clears
            if front_class < DIST_DANGER:
                self.consecutive_pivot_count = 0

            # P11: Lateral obstacle, one side freer
            if (left_class >= DIST_NEAR or right_class >= DIST_NEAR):
                if left_class != right_class:
                    if left_class < right_class:
                        self._transition(NAV_ARC_LEFT)
                        self._start_dwell(0.6)
                        turn = -abs(turn_intensity) * MAX_TURN_BIAS
                        speed = int(SLOW_SPEED * self.terrain_mult * self.stall_speed_mult)
                        return (NAV_ARC_LEFT, speed, turn, 1, "nav_arc_L")
                    else:
                        self._transition(NAV_ARC_RIGHT)
                        self._start_dwell(0.6)
                        turn = abs(turn_intensity) * MAX_TURN_BIAS
                        speed = int(SLOW_SPEED * self.terrain_mult * self.stall_speed_mult)
                        return (NAV_ARC_RIGHT, speed, turn, 1, "nav_arc_R")

            # P12: Narrow corridor (both sides DANGER, front OK)
            if left_class >= DIST_DANGER and right_class >= DIST_DANGER and front_class < DIST_DANGER:
                self._transition(NAV_SLOW_FORWARD)
                speed_s = speed_scale_from_front(front_class)
                speed = int(SLOW_SPEED * speed_s * self.terrain_mult * self.stall_speed_mult)
                return (NAV_SLOW_FORWARD, speed, 0.0, 1, "nav_slow_fwd")

            # P13: Front CAUTION or NEAR
            if front_class >= DIST_CAUTION:
                self._transition(NAV_SLOW_FORWARD)
                speed_s = speed_scale_from_front(front_class)
                turn = -turn_intensity * MAX_TURN_BIAS * 0.5
                speed = int(SLOW_SPEED * speed_s * self.terrain_mult * self.stall_speed_mult)
                return (NAV_SLOW_FORWARD, speed, turn, 1, "nav_slow_fwd")

            # P14: All clear
            self._transition(NAV_FORWARD)

            # --- Dwell re-evaluation for active states ---
            # (If we reach here, no higher-priority trigger fired)
            if self.state in (NAV_ARC_LEFT, NAV_ARC_RIGHT) and self._dwell_active():
                # Extend if arc-side still blocked
                if self.state == NAV_ARC_LEFT and left_class >= DIST_NEAR:
                    self._refresh_dwell(0.4)
                elif self.state == NAV_ARC_RIGHT and right_class >= DIST_NEAR:
                    self._refresh_dwell(0.4)

            # --- FORWARD with heading correction ---
            base_speed = CRUISE_SPEED
            if self.terrain_is_tripod:
                base_speed = TRIPOD_CRUISE_SPEED
            speed_s = speed_scale_from_front(front_class)
            speed = int(base_speed * speed_s * self.terrain_mult * self.stall_speed_mult)

            # Heading drift correction
            turn = 0.0
            if self.initial_yaw is not None:
                yaw_error = math.atan2(
                    math.sin(imu["yaw_rad"] - self.initial_yaw),
                    math.cos(imu["yaw_rad"] - self.initial_yaw))
                if abs(yaw_error) > math.radians(30):
                    if yaw_error > 0:
                        turn = -HEADING_CORRECTION_BIAS  # drifted right → correct left
                    else:
                        turn = HEADING_CORRECTION_BIAS   # drifted left → correct right

            return (NAV_FORWARD, speed, turn, 1, "nav_forward")

        def _backward_action(self, frame):
            """Compute backward recovery action with rear safety check."""
            if is_rear_safe(frame):
                self.hold_position_count = 0
                return (NAV_BACKWARD, BACKWARD_SPEED, 0.0, -1, "nav_backward")
            else:
                self.hold_position_count += 1
                brain_log(f"[NAV] rear unsafe, holding (count={self.hold_position_count})")
                if self.hold_position_count >= 2:
                    # Stuck — try pivot
                    self._pick_pivot_direction(frame)
                    self._transition(NAV_PIVOT_TURN)
                    self._start_dwell(1.5)
                    self.consecutive_pivot_count += 1
                    turn = self.pivot_direction * PIVOT_TURN_BIAS
                    step = "nav_pivot_L" if self.pivot_direction < 0 else "nav_pivot_R"
                    return (NAV_PIVOT_TURN, 0, turn, 1, step)
                return (NAV_BACKWARD, 0, 0.0, 1, "nav_backward")  # hold position

        def _pick_pivot_direction(self, frame):
            """Pick pivot direction toward freer rear diagonal."""
            if frame is None:
                self.pivot_direction = -1  # default left
                return
            rdl = frame["RDL"]
            rdr = frame["RDR"]
            rdl_eff = 0 if (rdl is None or rdl == -1) else rdl
            rdr_eff = 0 if (rdr is None or rdr == -1) else rdr
            if rdl_eff >= rdr_eff:
                self.pivot_direction = -1  # left
            else:
                self.pivot_direction = 1   # right

        def update_terrain(self, imu, avg_load, angular_rate, accel_mag,
                           front_class, flicker_count, roll_deg):
            """Unified Terrain Decision Table — updates terrain_gait, terrain_impact_start/end, terrain_mult.
            Only applies in FORWARD or SLOW_FORWARD states."""
            now = time.monotonic()
            pitch_deg = imu["pitch_deg"]
            upright = imu["upright_quality"]

            # Don't change terrain during active maneuvers
            if self.state not in (NAV_FORWARD, NAV_SLOW_FORWARD):
                return

            prev_gait = self.terrain_gait

            # T1: Steep climb
            if pitch_deg > 15:
                if self._steep_up_start == 0.0:
                    self._steep_up_start = now
                if (now - self._steep_up_start) >= 1.0:
                    self.terrain_gait = 1  # wave
                    self.terrain_impact_start = 315
                    self.terrain_impact_end = 15
                    self.terrain_mult = 0.7
                    self.terrain_is_tripod = False
                    self._apply_gait_transition(prev_gait)
                    return
            else:
                self._steep_up_start = 0.0

            # T2: Steep descent
            if pitch_deg < -15:
                if self._steep_down_start == 0.0:
                    self._steep_down_start = now
                if (now - self._steep_down_start) >= 1.0:
                    self.terrain_gait = 1  # wave
                    self.terrain_impact_start = 325
                    self.terrain_impact_end = 35
                    self.terrain_mult = 0.5
                    self.terrain_is_tripod = False
                    self._apply_gait_transition(prev_gait)
                    return
            else:
                self._steep_down_start = 0.0

            # T3: Moderate slope (uphill or downhill) or tilted
            if abs(pitch_deg) > SLOPE_PITCH_DEG or (0.15 <= upright <= 0.5):
                self.terrain_gait = 1  # wave
                self.terrain_impact_start = 325
                self.terrain_impact_end = 35
                self.terrain_mult = 0.6
                self.terrain_is_tripod = False
                self._apply_gait_transition(prev_gait)
                return

            # T4: Heavy terrain (soft sand)
            if avg_load > HEAVY_TERRAIN_LOAD:
                if self._heavy_load_start == 0.0:
                    self._heavy_load_start = now
                if (now - self._heavy_load_start) >= TERRAIN_SUSTAIN_S:
                    self.terrain_gait = 1  # wave
                    self.terrain_impact_start = 330
                    self.terrain_impact_end = 30
                    self.terrain_mult = 0.5
                    self.terrain_is_tripod = False
                    self._apply_gait_transition(prev_gait)
                    return
            else:
                self._heavy_load_start = 0.0

            # T5: Excessive wobble
            if angular_rate > 0.3:
                self.terrain_impact_start = 325
                self.terrain_impact_end = 35
                self.terrain_mult = 0.7
                self.terrain_is_tripod = False
                return  # keep current gait

            # T6: Rocky terrain (flicker + moderate load)
            if flicker_count >= FLICKER_COUNT_THRESHOLD and avg_load > 300:
                self.terrain_impact_start = 345
                self.terrain_impact_end = 15
                self.terrain_mult = 0.8
                self.terrain_is_tripod = False
                return  # keep current gait

            # T7: High vibration
            if accel_mag > 14:
                if self._high_vibe_start == 0.0:
                    self._high_vibe_start = now
                if (now - self._high_vibe_start) >= 0.5:
                    self.terrain_impact_start = 345
                    self.terrain_impact_end = 15
                    self.terrain_mult = 0.8
                    self.terrain_is_tripod = False
                    return  # keep current gait
            else:
                self._high_vibe_start = 0.0

            # T8: Hard flat ground — sprint with tripod
            no_recent_stalls = (now - self._last_stall_clear_time) > 30 or self.stall_count_30s == 0
            if (avg_load < LIGHT_TERRAIN_LOAD
                    and front_class == DIST_CLEAR
                    and abs(pitch_deg) < 5
                    and abs(roll_deg) < 5
                    and no_recent_stalls):
                if self._light_load_start == 0.0:
                    self._light_load_start = now
                if (now - self._light_load_start) >= TERRAIN_SUSTAIN_S:
                    self.terrain_gait = 0  # tripod
                    self.terrain_impact_start = 330
                    self.terrain_impact_end = 30
                    self.terrain_mult = 1.0
                    self.terrain_is_tripod = True
                    self._apply_gait_transition(prev_gait)
                    return
            else:
                self._light_load_start = 0.0
                # Tripod safety gate: immediate fallback
                if self.terrain_is_tripod:
                    self.terrain_gait = 2  # back to quad
                    self.terrain_is_tripod = False
                    self._apply_gait_transition(0)  # force transition from tripod
                    # fall through to T9

            # T9: Default — quadruped
            self.terrain_gait = 2
            self.terrain_impact_start = 330
            self.terrain_impact_end = 30
            self.terrain_mult = 1.0
            self.terrain_is_tripod = False
            self._apply_gait_transition(prev_gait)

        def _apply_gait_transition(self, prev_gait):
            """Track gait transition timing for smooth speed ramp."""
            if prev_gait != self.terrain_gait:
                self._gait_transition_until = time.monotonic() + 0.5

        def apply_modifiers(self, speed, imu, accel_mag, voltage, angular_rate,
                            load_asymmetry, stale_seconds, roll_deg):
            """Layer 2: speed/stance modifiers. Returns modified speed."""
            now = time.monotonic()

            # M1: Impact event
            if accel_mag > 20:
                self._impact_cooldown_until = now + 1.0
            if now < self._impact_cooldown_until:
                speed = int(speed * 0.5)

            # M2: Low battery
            battery_mult = compute_battery_mult(voltage)
            if battery_mult < 1.0:
                speed = int(speed * battery_mult)

            # M3: Brief data gap (300ms-5s)
            if 0.3 < stale_seconds < 5.0:
                speed = int(speed * 0.5)

            # M4: Lateral slope
            if abs(roll_deg) > 10:
                if self._roll_sustained_start == 0.0:
                    self._roll_sustained_start = now
                if (now - self._roll_sustained_start) >= 0.5:
                    speed = int(speed * 0.7)
                    # Override to stealth crawl stance
                    self.terrain_impact_start = 325
                    self.terrain_impact_end = 35
            else:
                self._roll_sustained_start = 0.0

            # M5: Asymmetric loading
            if load_asymmetry > LOAD_ASYMMETRY_THRESHOLD:
                if self._asymmetry_start == 0.0:
                    self._asymmetry_start = now
                if (now - self._asymmetry_start) >= 1.0:
                    speed = int(speed * 0.8)
            else:
                self._asymmetry_start = 0.0

            # Gait transition smoothing
            if now < self._gait_transition_until:
                speed = min(speed, SLOW_SPEED)

            return max(0, speed)

    def state_self_right_roll():
        """Momentum roll to flip robot upright when capsized.

        !! UNVALIDATED - never tested on hardware !!

        Safety guards (audit fixes 21-26):
          - C2: Load-based orientation check — skips roll if robot is upright
          - H5: Voltage pre-check — skips roll if battery too low for high-current draw
          - C3: shared_roll_mode bypasses LEFT_SERVOS negation so all legs spin
                same physical direction, producing actual rolling moment
          - C1: stall_override suppresses stall speed-zeroing but overload prevention
                (TE cycling) still fires independently via load magnitude check

        Geometry (from design specs):
          Leg radius = 75mm, arc = 210°. Chassis 510x280x75mm.
          Inverted window: 140/220 (±40° around 180°). Legs reach floor when inverted. ✓

        Physics caveat: even with DIRECTION_MAP bypass, lateral friction on wet sand
        is ~4x insufficient to roll a 2.5kg robot. May work on hard surfaces only.
        """
        # --- C2: Orientation guard ---
        # Switch to wave gait for reliable upright detection (5 legs in stance vs tripod's 3)
        saved_gait_for_check = shared_gait_id.value
        shared_gait_id.value = 1   # WAVE — duty 0.85, 5 legs in stance
        shared_speed.value = 0
        time.sleep(1.5)             # Wait for LERP convergence to wave offsets

        UPRIGHT_LOAD_THRESHOLD = 200
        roll_loads = [str(shared_servo_loads[i]) for i in range(len(ALL_SERVOS))]
        upright_count = sum(1 for i in range(len(ALL_SERVOS))
                           if shared_servo_loads[i] > UPRIGHT_LOAD_THRESHOLD)

        # Tier 3g: log orientation check
        current_voltage = shared_voltage.value
        brain_log(f"[ROLL-CHECK] loads:{','.join(roll_loads)} above{UPRIGHT_LOAD_THRESHOLD}="
                  f"{upright_count}/4needed volt={current_voltage:.1f}V")

        if upright_count >= 4:
            brain_log(f"[ROLL-CHECK] → SKIP_UPRIGHT")
            print(f"[recovery] roll skipped — robot appears upright "
                  f"({upright_count}/6 legs loaded > {UPRIGHT_LOAD_THRESHOLD})")
            shared_gait_id.value = saved_gait_for_check  # restore gait
            return

        # --- H5: Voltage pre-check ---
        # Self-right draws ~4.9A total. On cold/discharged battery, voltage sag
        # could drop below VOLTAGE_MIN (10.5V) and trigger Heart safety shutdown
        # mid-roll. Require 11.0V minimum before attempting.
        ROLL_MIN_VOLTAGE = 11.0
        if current_voltage < ROLL_MIN_VOLTAGE:
            brain_log(f"[ROLL-CHECK] → SKIP_LOW_VOLTAGE ({current_voltage:.1f}V < {ROLL_MIN_VOLTAGE}V)")
            print(f"[recovery] roll skipped — battery too low "
                  f"({current_voltage:.1f}V < {ROLL_MIN_VOLTAGE}V)")
            return

        brain_log("[ROLL-CHECK] → ATTEMPT")
        print("[recovery] attempting momentum roll to self-right")

        ROLL_LOAD_SPEED   =  400   # step 1 - direction unverified inverted
        ROLL_SNAP_SPEED   = -600   # step 2 - direction unverified inverted
        ROLL_SETTLE_SPEED =  400   # step 3 - timing unverified

        # Corrected window centered on 180° (floor-facing when inverted).
        ROLL_IMPACT_START = 140
        ROLL_IMPACT_END   = 220

        set_gait_state(gait=1, turn=0.0, impact_start=ROLL_IMPACT_START,
                       impact_end=ROLL_IMPACT_END, z_flip=-1, step_name="roll_init")

        # Suppress stall detection speed-zeroing — floor contact produces high load.
        # Overload prevention TE cycling still fires (decoupled, checks load directly).
        shared_stall_override.value = True
        # C3: all legs spin same physical direction for actual rolling moment.
        shared_roll_mode.value      = True
        try:
            # Wait for LERP to converge on the new window before engaging speed.
            tsleep(1.0)

            print("[recovery] roll step 1 - loading weight")
            set_gait_state(speed=ROLL_LOAD_SPEED, step_name="roll_load_weight")
            tsleep(5.0)

            print("[recovery] roll step 2 - inertial snap")
            set_gait_state(speed=ROLL_SNAP_SPEED, step_name="roll_inertial_snap")
            tsleep(10.0)

            print("[recovery] roll step 3 - settling")
            set_gait_state(speed=ROLL_SETTLE_SPEED, step_name="roll_settle")
            tsleep(5.0)
        finally:
            set_gait_state(speed=0, z_flip=1, step_name="roll_cleanup")
            shared_stall_override.value = False
            shared_roll_mode.value      = False  # restore LEFT_SERVOS negation

        print("[recovery] roll complete")

    try:
        print("[brain] waiting for heart...")
        # Use != 0 sentinel rather than < 50 threshold: a signed 32-bit counter
        # overflows to negative after ~497 days at 50 Hz, making the < 50 check
        # pass immediately on the next boot and starting the mission before Heart
        # has actually initialised.  Any nonzero value means Heart has ticked at
        # least once — sufficient proof of life for the boot gate.
        heartbeat_start = time.time()
        while shared_heartbeat.value == 0:
            if not gait_process.is_alive():
                raise EmergencyStopException("[brain] Heart process died during startup")
            if time.time() - heartbeat_start > 15:
                raise EmergencyStopException("[brain] Heart failed to start within 15s")
            time.sleep(0.1)
        print("[brain] heart is running, starting mission")

        # Tier 4: Brain session header
        try:
            with open(LOG_FILE, "a") as f:
                f.write("=" * 50 + "\n")
                f.write(f"=== BRAIN ONLINE {datetime.datetime.now().strftime('%Y-%m-%dT%H:%M:%S')} ===\n")
                f.write("Mission: Phase1(tripod) → Phase2(quad) → Phase3(wave) → Phase4(recovery) → Phase5(shutdown)\n")
                f.write("=" * 50 + "\n")
        except:
            pass

        if "--competition" in sys.argv or "--competition-dry-run" in sys.argv:
            # === COMPETITION MODE: Autonomous sensor-driven navigation ===
            dry_run = "--competition-dry-run" in sys.argv
            if dry_run:
                print("=== COMPETITION DRY-RUN (speed=0, sensors active, logging only) ===")
            else:
                print("=== COMPETITION MODE (autonomous nav) ===")

            # --- Timed fallback sequence (used when sensors unavailable) ---
            def run_timed_fallback():
                brain_log("FALLBACK: timed sequence — no sensor data")
                set_gait_state(gait=2, impact_start=330, impact_end=30,
                               step_name="fallback_quad_init")
                set_gait_state(speed=400, turn=0.0, step_name="fallback_quad_fwd")
                stall_tsleep(45)
                set_gait_state(gait=1, speed=350, step_name="fallback_wave_fwd")
                stall_tsleep(30)
                set_gait_state(speed=0, turn=0.0, step_name="fallback_shutdown")
                tsleep(3)

            # --- Try to start Arduino sensor reader ---
            reader = None
            if not HAS_SERIAL:
                print("[NAV] pyserial not installed — timed fallback")
                if dry_run:
                    brain_log("[NAV][DRY-RUN] fallback suppressed — no servo motion")
                else:
                    run_timed_fallback()
            else:
                try:
                    reader = ArduinoReader()
                    brain_log("[NAV] Arduino reader started")
                except Exception as e:
                    brain_log(f"[NAV] Arduino init failed: {e}")
                    print(f"[NAV] Arduino init failed: {e} — timed fallback")
                    if dry_run:
                        brain_log("[NAV][DRY-RUN] fallback suppressed — no servo motion")
                    else:
                        run_timed_fallback()
                    reader = None

            if reader is not None:
                try:
                    # Wait up to 3s for first valid frame
                    init_wait = time.monotonic()
                    while reader.stale_seconds() == float("inf"):
                        if time.monotonic() - init_wait > 3.0:
                            break
                        time.sleep(0.1)

                    if reader.stale_seconds() == float("inf"):
                        brain_log("[NAV] no Arduino data after 3s — timed fallback")
                        print("[NAV] no Arduino data — timed fallback")
                        if dry_run:
                            brain_log("[NAV][DRY-RUN] fallback suppressed — no servo motion")
                        else:
                            run_timed_fallback()
                    else:
                        # --- Initialize nav components ---
                        nav = NavStateMachine()
                        cliff = CliffDetector()
                        flicker = FlickerTracker()

                        # Set initial gait: quadruped, normal stance
                        set_gait_state(gait=2, impact_start=330, impact_end=30,
                                       speed=0, turn=0.0, x_flip=1,
                                       step_name="nav_init")

                        # Capture initial yaw for heading correction
                        first_frame = reader.get_latest()
                        if first_frame:
                            first_imu = compute_imu(first_frame)
                            nav.initial_yaw = first_imu["yaw_rad"]
                            brain_log(f"[NAV] initial yaw={math.degrees(nav.initial_yaw):.1f}deg")

                        brain_log("[NAV] autonomous nav loop starting")
                        fallen_back = False

                        # === MAIN NAV LOOP (~10 Hz) ===
                        while is_running.value and not nav.finished:
                            loop_start = time.monotonic()

                            # --- Check for Arduino death → timed fallback ---
                            stale = reader.stale_seconds()
                            if stale > 5.0:
                                brain_log(f"[NAV] Arduino dead {stale:.1f}s — timed fallback")
                                print("[NAV] Arduino dead — switching to timed fallback")
                                if dry_run:
                                    brain_log("[NAV][DRY-RUN] mid-mission fallback suppressed — no servo motion")
                                else:
                                    remaining = MISSION_TIMEOUT_S - (time.monotonic() - nav.mission_start)
                                    if remaining > 5:
                                        set_gait_state(gait=2, impact_start=330, impact_end=30,
                                                       speed=400, turn=0.0, x_flip=1,
                                                       step_name="fallback_mid_quad")
                                        stall_tsleep(min(remaining * 0.6, 45))
                                        set_gait_state(gait=1, speed=350,
                                                       step_name="fallback_mid_wave")
                                        stall_tsleep(min(remaining * 0.3, 30))
                                    set_gait_state(speed=0, turn=0.0,
                                                   step_name="fallback_mid_stop")
                                    tsleep(3)
                                fallen_back = True
                                break

                            # --- Get latest sensor frame ---
                            frame = reader.get_latest()
                            if frame is None:
                                time.sleep(0.1)
                                continue

                            # --- Sensor processing ---
                            imu = compute_imu(frame)
                            front_class, left_class, right_class = classify_sectors(frame)
                            front_cliff, rear_cliff = cliff.update(frame["FCD"], frame["RCD"])
                            turn_intensity = compute_turn_intensity(frame)
                            flicker_count = flicker.update(front_class)

                            # Servo loads snapshot
                            loads = list(shared_servo_loads)
                            load_info = compute_servo_loads(loads)
                            avg_load = load_info["avg_load"]
                            load_asymmetry = load_info["load_asymmetry"]

                            # Voltage
                            voltage = shared_voltage.value

                            # --- Update terrain overlay (FORWARD/SLOW only) ---
                            nav.update_terrain(imu, avg_load, imu["angular_rate"],
                                               imu["accel_mag"], front_class,
                                               flicker_count, imu["roll_deg"])

                            # --- FSM update ---
                            state, speed, turn, x_flip, step_name = nav.update(
                                frame, imu, front_class, left_class, right_class,
                                front_cliff, rear_cliff, turn_intensity, avg_load,
                                load_asymmetry, imu["angular_rate"], imu["accel_mag"],
                                voltage, flicker_count)

                            # --- Apply Layer 2 modifiers ---
                            speed = nav.apply_modifiers(
                                speed, imu, imu["accel_mag"], voltage,
                                imu["angular_rate"], load_asymmetry,
                                stale, imu["roll_deg"])

                            # --- Handle special states ---
                            if state == NAV_WIGGLE:
                                if dry_run:
                                    brain_log("[NAV][DRY-RUN] would wiggle")
                                    nav._transition(NAV_FORWARD)
                                    continue
                                # Blocking wiggle recovery
                                set_gait_state(speed=0, turn=0.0, x_flip=1,
                                               step_name="nav_pre_wiggle")
                                state_recovery_wiggle()
                                nav.stall_speed_mult *= 0.75
                                if nav.stall_count_30s >= 3:
                                    # Switch to wave gait, halve speed
                                    nav.terrain_gait = 1
                                    nav.stall_speed_mult = 0.5
                                    brain_log("[NAV] 3 stalls in 30s — wave gait, half speed")
                                nav._transition(NAV_FORWARD)
                                continue

                            if state == NAV_STOP_SAFE and imu["upright_quality"] < 0.15:
                                if dry_run:
                                    brain_log("[NAV][DRY-RUN] would self-right")
                                    nav._transition(NAV_FORWARD)
                                    continue
                                # Severely tilted — try self-right
                                set_gait_state(speed=0, turn=0.0, x_flip=1,
                                               step_name="nav_stop_safe")
                                brain_log("[NAV] tipover — attempting self-right")
                                state_self_right_roll()
                                # Fresh frame after recovery
                                time.sleep(0.2)
                                continue

                            # --- Apply gait state ---
                            # Always ensure x_flip=1 when not in BACKWARD
                            if state != NAV_BACKWARD:
                                x_flip = 1

                            if dry_run:
                                brain_log(f"[NAV][DRY-RUN] state={step_name} speed={speed} turn={turn:.2f} x_flip={x_flip} gait={nav.terrain_gait}")
                                speed = 0
                            set_gait_state(speed=speed, turn=turn, x_flip=x_flip,
                                           gait=nav.terrain_gait,
                                           impact_start=nav.terrain_impact_start,
                                           impact_end=nav.terrain_impact_end,
                                           step_name=step_name)

                            # --- Loop timing ---
                            elapsed = time.monotonic() - loop_start
                            sleep_time = max(0.0, 0.1 - elapsed)
                            if sleep_time > 0:
                                time.sleep(sleep_time)

                        # --- Nav loop exit ---
                        if not fallen_back:
                            set_gait_state(speed=0, turn=0.0, x_flip=1,
                                           step_name="nav_mission_end")
                            brain_log(f"[NAV] mission complete, elapsed={time.monotonic()-nav.mission_start:.1f}s")
                            tsleep(3)

                finally:
                    if reader is not None:
                        reader.stop()
                        brain_log("[NAV] Arduino reader stopped")
        elif "--test-tripod" in sys.argv:
            # === TEST: Tripod phase only ===
            print("=== TEST MODE: TRIPOD ===")
            set_gait_state(gait=0, impact_start=330, impact_end=30, step_name="test_tripod_init")

            set_gait_state(speed=600, step_name="forward");                                    stall_tsleep(12)
            set_gait_state(turn=-0.2, step_name="carve_left");                                 stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(turn=0.2, step_name="carve_right");                                 stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(speed=0, turn=-0.5, step_name="pivot_left");                        stall_tsleep(10)
            set_gait_state(turn=0.0, speed=600, step_name="straight");                         stall_tsleep(3)
            set_gait_state(speed=0, turn=0.5, step_name="pivot_right");                        stall_tsleep(10)
            set_gait_state(turn=0.0, speed=600, step_name="straight");                         stall_tsleep(3)
            set_gait_state(speed=0, step_name="decel_pre_reverse"); tsleep(0.5)
            set_gait_state(speed=-600, turn=0.0, step_name="reverse");                         stall_tsleep(12)
            set_gait_state(speed=0, step_name="decel"); tsleep(2)

        elif "--test-quad" in sys.argv:
            # === TEST: Quadruped phase only ===
            print("=== TEST MODE: QUADRUPED ===")
            set_gait_state(gait=2, impact_start=330, impact_end=30, step_name="test_quad_init")

            set_gait_state(speed=300, turn=0.0, step_name="forward");                          stall_tsleep(12)
            set_gait_state(turn=-0.15, step_name="carve_left");                                stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(turn=0.15, step_name="carve_right");                                stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(speed=0, turn=-0.4, step_name="pivot_left");                        stall_tsleep(10)
            set_gait_state(turn=0.0, speed=300, step_name="straight");                         stall_tsleep(3)
            set_gait_state(speed=0, turn=0.4, step_name="pivot_right");                        stall_tsleep(10)
            set_gait_state(turn=0.0, speed=300, step_name="straight");                         stall_tsleep(3)
            set_gait_state(speed=0, step_name="decel_pre_reverse"); tsleep(0.5)
            set_gait_state(speed=-300, turn=0.0, step_name="reverse");                         stall_tsleep(12)
            set_gait_state(speed=0, step_name="decel"); tsleep(2)
            set_gait_state(speed=300, impact_start=345, impact_end=15, step_name="walking_tall");   stall_tsleep(15)
            set_gait_state(impact_start=325, impact_end=35, step_name="stealth_crawl");              stall_tsleep(15)

        elif "--test-wave" in sys.argv:
            # === TEST: Wave phase only ===
            print("=== TEST MODE: WAVE ===")
            set_gait_state(gait=1, impact_start=330, impact_end=30, step_name="test_wave_init")

            set_gait_state(speed=180, turn=0.0, step_name="forward");                          stall_tsleep(12)
            set_gait_state(turn=-0.1, step_name="carve_left");                                 stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(turn=0.1, step_name="carve_right");                                 stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(speed=0, turn=-0.3, step_name="pivot_left");                        stall_tsleep(10)
            set_gait_state(turn=0.0, speed=180, step_name="straight");                         stall_tsleep(3)
            set_gait_state(speed=0, turn=0.3, step_name="pivot_right");                        stall_tsleep(10)
            set_gait_state(turn=0.0, speed=180, step_name="straight");                         stall_tsleep(3)
            set_gait_state(speed=0, step_name="decel_pre_reverse"); tsleep(0.5)
            set_gait_state(speed=-180, turn=0.0, step_name="reverse");                         stall_tsleep(12)
            set_gait_state(speed=0, step_name="decel"); tsleep(2)
            set_gait_state(speed=180, impact_start=315, impact_end=15, step_name="cog_shift_hill_climb"); stall_tsleep(20)

        elif "--test-recovery" in sys.argv:
            # === TEST: Recovery phase only ===
            print("=== TEST MODE: RECOVERY ===")
            set_gait_state(speed=0, turn=0.0, gait=0, step_name="test_recovery_init"); tsleep(3)
            state_recovery_wiggle()
            state_self_right_roll()

        elif "--test-arduino" in sys.argv:
            # === TEST: Arduino serial reader — no Heart needed ===
            print("=== ARDUINO SERIAL TEST ===")
            if not HAS_SERIAL:
                print("[ERROR] pyserial not installed. pip install pyserial")
            else:
                reader = ArduinoReader()
                try:
                    for i in range(20):
                        frame = reader.get_latest()
                        if frame:
                            print(f"[{i}] FCF={frame['FCF']:.0f} FDL={frame['FDL']:.0f} "
                                  f"FDR={frame['FDR']:.0f} FCD={frame['FCD']:.0f} "
                                  f"healthy={reader.healthy} stale={reader.stale_seconds():.2f}s")
                        else:
                            print(f"[{i}] no frame yet, stale={reader.stale_seconds():.2f}s")
                        time.sleep(0.5)
                finally:
                    reader.stop()
                print("=== DONE ===")

        elif "--test-competition" in sys.argv:
            # === TEST: Competition sequence only ===
            print("=== TEST MODE: COMPETITION ===")

            # Phase 1: Quadruped forward (best stability + speed balance on sand)
            set_gait_state(gait=2, impact_start=330, impact_end=30, step_name="comp_quad_init")
            set_gait_state(speed=400, turn=0.0, step_name="comp_quad_fwd")
            stall_tsleep(45)

            # Phase 2: Wave fallback (max ground contact if quad didn't finish)
            set_gait_state(gait=1, speed=350, step_name="comp_wave_fwd")
            stall_tsleep(30)

            # Decel and stop
            set_gait_state(speed=0, turn=0.0, step_name="comp_shutdown")
            tsleep(3)

        else:
            # === DEMO MODE: Full maneuver showcase ===
            # Speeds reduced from test values for wet-sand obstacle navigation (Alamosa CO, April)
            # =========================================================
            # PHASE 1: TRIPOD
            # =========================================================
            print("\n-- phase 1: tripod --")
            set_gait_state(gait=0, impact_start=330, impact_end=30, step_name="phase1_init")

            set_gait_state(speed=600, step_name="forward");                                    stall_tsleep(12)
            set_gait_state(turn=-0.2, step_name="carve_left");                                 stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(turn=0.2, step_name="carve_right");                                 stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(speed=0, turn=-0.5, step_name="pivot_left");                        stall_tsleep(10)
            set_gait_state(turn=0.0, speed=600, step_name="straight");                         stall_tsleep(3)
            set_gait_state(speed=0, turn=0.5, step_name="pivot_right");                        stall_tsleep(10)
            set_gait_state(turn=0.0, speed=600, step_name="straight");                         stall_tsleep(3)
            set_gait_state(speed=0, step_name="decel_pre_reverse_p1"); tsleep(0.5)
            set_gait_state(speed=-600, turn=0.0, step_name="reverse");                         stall_tsleep(12)

            # Decel pause - smooth_hz bleeds ~720ms from -1.2Hz without this,
            # meaning the robot reverses briefly after Phase 2 starts.
            set_gait_state(speed=0, step_name="decel_p1"); tsleep(2)

            # =========================================================
            # PHASE 2: QUADRUPED
            # =========================================================
            print("\n-- phase 2: quadruped --")
            set_gait_state(gait=2, impact_start=330, impact_end=30, step_name="phase2_init")

            set_gait_state(speed=300, turn=0.0, step_name="forward");                          stall_tsleep(12)
            set_gait_state(turn=-0.15, step_name="carve_left");                                stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(turn=0.15, step_name="carve_right");                                stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(speed=0, turn=-0.4, step_name="pivot_left");                        stall_tsleep(10)
            set_gait_state(turn=0.0, speed=300, step_name="straight");                         stall_tsleep(3)
            set_gait_state(speed=0, turn=0.4, step_name="pivot_right");                        stall_tsleep(10)
            set_gait_state(turn=0.0, speed=300, step_name="straight");                         stall_tsleep(3)
            set_gait_state(speed=0, step_name="decel_pre_reverse_p2"); tsleep(0.5)
            set_gait_state(speed=-300, turn=0.0, step_name="reverse");                         stall_tsleep(12)

            # Decel before walking tall - avoids ~720ms of backward motion bleed into forward mode
            set_gait_state(speed=0, step_name="decel_p2"); tsleep(2)

            set_gait_state(speed=300, impact_start=345, impact_end=15, step_name="walking_tall");   stall_tsleep(15)
            set_gait_state(impact_start=325, impact_end=35, step_name="stealth_crawl");              stall_tsleep(15)

            # =========================================================
            # PHASE 3: WAVE
            # =========================================================
            print("\n-- phase 3: wave --")
            set_gait_state(gait=1, impact_start=330, impact_end=30, step_name="phase3_init")

            set_gait_state(speed=180, turn=0.0, step_name="forward");                          stall_tsleep(12)
            set_gait_state(turn=-0.1, step_name="carve_left");                                 stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(turn=0.1, step_name="carve_right");                                 stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(speed=0, turn=-0.3, step_name="pivot_left");                        stall_tsleep(10)
            set_gait_state(turn=0.0, speed=180, step_name="straight");                         stall_tsleep(3)
            set_gait_state(speed=0, turn=0.3, step_name="pivot_right");                        stall_tsleep(10)
            set_gait_state(turn=0.0, speed=180, step_name="straight");                         stall_tsleep(3)
            set_gait_state(speed=0, step_name="decel_pre_reverse_p3"); tsleep(0.5)
            set_gait_state(speed=-180, turn=0.0, step_name="reverse");                         stall_tsleep(12)

            # Decel pause - matches Phase 1/2 pattern, bleeds ~720ms of reverse motion before COG shift
            set_gait_state(speed=0, step_name="decel_p3"); tsleep(2)

            set_gait_state(speed=180, impact_start=315, impact_end=15, step_name="cog_shift_hill_climb"); stall_tsleep(20)

            # =========================================================
            # PHASE 4: RECOVERY
            # =========================================================
            print("\n-- phase 4: recovery --")
            set_gait_state(speed=0, turn=0.0, gait=0, step_name="phase4_init"); tsleep(3)

            state_recovery_wiggle()
            state_self_right_roll()

            # =========================================================
            # PHASE 5: DONE
            # =========================================================
            print("\n-- phase 5: shutdown --")
            set_gait_state(speed=0, turn=0.0, step_name="phase5_shutdown"); tsleep(8)

    except EmergencyStopException as e:
        print(f"[brain] emergency stop: {e}")
    except KeyboardInterrupt:
        print("\n[brain] interrupted")
    finally:
        is_running.value = False
        gait_process.join(timeout=5)
        if gait_process.is_alive():
            gait_process.terminate()
        print("done")