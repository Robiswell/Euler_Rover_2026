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
  --competition          Autonomous nav - 8-state FSM, Arduino sensors, terrain adaptation (prints real-time FSM state/speed/turn/gait/sensors to terminal)
  --competition-dry-run  Autonomous nav with speed=0 - sensors active, logs transitions only (same FSM terminal output, servos stay still)
  --test-competition     Timed fallback - quad@400 45s then wave@350 30s (no sensors)
  --test-tripod          Tripod gait only - forward, carve, pivot, reverse
  --test-quad            Quadruped gait only - forward, carve, pivot, reverse, walking tall, stealth crawl
  --test-wave            Wave gait only - forward, carve, pivot, reverse, COG shift hill climb
  --test-recovery        Recovery behaviors only - wiggle + self-right roll
  --test-arduino         Serial reader test - print 20 Arduino frames, report health (no Heart)
  --test-sensors         Sensor processing test - classify sectors, cliff, IMU, turn intensity
  --test-nav             Full nav pipeline test - Arduino -> parse -> classify -> FSM transitions
  --no-verbose-telemetry Disable extended telemetry ([H5] servo detail, [BS] sensor, [BN] nav)
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
import unittest

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
ROLL_FRONT_SERVOS        = {1, 2}   # Fix 76: counter-rotating front pair for self-right
ROLL_REAR_SERVOS         = {4, 5}   # Fix 76: counter-rotating rear pair for self-right
SERVO_SPEED_GOVERNOR_CAP = 499     # Speed ceiling for roll/self-right mode (walking uses GOVERNOR_FF_BUDGET)

# Body Geometry (measured from physical robot — verified from physical robot measurements)
# These values are the foundation for all ground-clearance calculations.
#   h(θ) = LEG_EFFECTIVE_RADIUS × cos(θ)  — chassis height above ground at leg angle θ
#   clearance = h(θ) − SHAFT_TO_CHASSIS_BOTTOM  — must stay > MIN_GROUND_CLEARANCE
LEG_EFFECTIVE_RADIUS    = 125.0    # mm — C-leg geometry: servo shaft at arc endpoint, not center (full diameter = radius, corrected 2026-03-19)
LEG_DIAMETER            = 125.0    # mm — outer tip-to-tip across leg arc (12.5 cm measured); for C-legs, effective radius = diameter (shaft at endpoint)
LEG_ARC_DEGREES         = 190.0    # degrees — physical arc of the curved leg (190° measured, constrains max stance sweep)
SHAFT_TO_CHASSIS_BOTTOM = 47.0     # mm — shaft center to chassis bottom (servo mounting block height 4.7 cm measured)
MIN_GROUND_CLEARANCE    = 15.0     # mm — minimum safe clearance (restored: r=125mm gives 78mm static clearance)
GOVERNOR_CLEARANCE_MARGIN = 5.0    # mm — extra safety buffer in clearance governor (restored: r=125mm has ample headroom)
FEEDFORWARD_CAP         = 499.0    # STS raw units — max open-loop speed to prevent servo overshoot
GOVERNOR_FF_BUDGET      = 2800.0   # STS raw units — max total speed budget (ff + KP correction) per leg
DEFAULT_IMPACT_START    = 345      # walking stance start angle (30 deg sweep)
DEFAULT_IMPACT_END      = 15       # walking stance end angle

# Body Dimensions (final mechanical design appendix — measured from physical robot)
BODY_LENGTH         = 511.0   # mm — total front-to-back (51.1 cm)
BODY_WIDTH          = 220.0   # mm — total side-to-side (22.0 cm)
CHASSIS_HEIGHT      = 80.0    # mm — total chassis height side view (8.0 cm)
SERVO_SPACING       = 175.0   # mm — shaft center-to-center, adjacent same side (17.5 cm)
CORNER_CHAMFER      = 40.0    # mm — front chamfer vertical side (4.0 cm)
CHAMFER_HORIZONTAL  = 69.0    # mm — front chamfer horizontal side (6.9 cm)
FRONT_SERVO_OFFSET  = 40.0    # mm — front chassis edge to front servo shaft center (4.0 cm)
REAR_SERVO_OFFSET   = 121.0   # mm — rear chassis edge to rear servo shaft center (511 - 40 - 2×175)
SHAFT_TO_TOP        = 33.0    # mm — shaft center to top face (CHASSIS_HEIGHT - SHAFT_TO_CHASSIS_BOTTOM)

# Roll-aware clearance constants (for turn governor -- see hardware.md)
# During turns, differential leg speeds create body roll. The chassis corner
# on the inside of the turn drops closer to the ground. These constants let
# the governor compute the extra clearance loss from roll.
W_SHAFT             = 160.0   # mm — left-right servo shaft plane distance (BODY_WIDTH - 2*30mm servo Y-offset)
CORNER_OVERHANG     = 30.0    # mm — chassis corner extends beyond servo shaft plane (servo Y-offset)
ROLL_SAFETY_FACTOR  = 2.0     # multiplier — accounts for dynamic bounce during walking turns

# Phase-Error Governor — throttles gait clock when servos can't track commanded phase
# Exponential ramp from CPG paper (Sensors 2019, 19(17), 3705, Eqs. 20-21)
PHERR_ENGAGE_DEG   = 30.0   # deg — engage governor when max servo phase error exceeds this
PHERR_RELEASE_DEG  = 20.0   # deg — release governor (hysteresis band prevents toggle)
PHERR_FLOOR_SCALE  = 0.35   # minimum Hz multiplier at full throttle (35% of max_safe_hz)
PHERR_RAMP_WIDTH   = 120.0  # deg — exponential ramp width (gentle near threshold, aggressive at high error)
PHERR_STUCK_TIMEOUT = 5.0   # sec — if governor stays at floor for this long, escalate to stall/wiggle
KAPPA_GOVERNOR     = 3.0    # exponential decay rate (gentler than KAPPA_TRANSITION=12.0 for gait switches)

# Industrial Safety Parameters
TEMP_MAX     = 65  # lowered from 70 — altitude reduces convective cooling ~25%
VOLTAGE_MIN  = 10.5  # 10.5V = 3.5V/cell floor — raised from 10.0 for cold-weather cell protection (no hardware BMS)
LOG_FILE     = "telemetry_log.txt"
LOG_MAX_SIZE = 10 * 1024 * 1024

# Phase Error Governor — breaks the PhErr positive feedback loop
# (see Hexapod Phase Error Accumulation Root Cause Analysis, 2026-03-17)
# When PhErr is below PHERR_DEADBAND, no correction is applied (noise floor).
PHERR_DEADBAND          = 6.0     # degrees — no phase correction below this (noise floor)

# Mirrored Physical Mounting Map
DIRECTION_MAP = {
    1: 1,   # Right Front
    2: -1,  # Left Front (Inverted)
    3: -1,  # Left Middle (Inverted)
    4: -1,  # Left Rear (Inverted)
    5: 1,   # Right Rear
    6: 1,   # Right Middle
}

# V0.5.01 New Constants for calibrated home positions and stall detection tuning for wet sand operation.
# Calibrated zero points (Legs pointing straight down)
HOME_POSITIONS = {
    1: 3447, 2: 955, 3: 1420,
    4: 1569, 5: 3197, 6: 3175,
}

# Servo ID -> shared_servo_loads array index (0-based, clean mapping)
SERVO_LOAD_INDEX = {sid: i for i, sid in enumerate(ALL_SERVOS)}

# Feedback Gains
KP_PHASE        = 12.0
STALL_THRESHOLD = 750  # raised from 600 for wet sand operation — tune from telemetry after first run
GHOST_TEMP      = 125  # STS3215 EMI artifact - bus noise returns flat 125°C (not a real reading)
OVERLOAD_PREVENTION_TIME = 1.5  # seconds — clear overload flag before 2s hardware cutoff (time-based, loop-rate invariant)
OVERLOAD_MAX_CYCLES = 10  # max TE cycles per servo per session — limits EPROM wear

# Verbose Telemetry — ON by default. Adds [H5] servo detail (5Hz), [BS] sensor (2Hz), [BN] nav decision lines.
# Disable with --no-verbose-telemetry. Total overhead: ~1.6 KB/s, <1 MB for a 10-minute run.
VERBOSE_TELEMETRY = "--no-verbose-telemetry" not in sys.argv

# --- KINEMATIC GAIT DICTIONARIES ---
GAITS = {
    0: {  # TRIPOD — do NOT use on slopes >10°: peak servo torque exceeds rated 10 kg·cm.
        #         Use Wave with COG shift for ramps (see Phase 3 hill climb).
        'duty': 0.5,
        'offsets': {2: 0.0, 6: 0.0, 4: 0.0,  1: 0.5, 3: 0.5, 5: 0.5}
    },
    1: {  # WAVE — metachronal wave, rear-to-front, alternating sides (R-L-R-L-R-L)
        'duty': 0.75,
        # Lift order: 5(RR)→3(LM)→1(RF)→4(LR)→6(RM)→2(LF)
        # Each consecutive pair in the air is on OPPOSITE sides AND different
        # columns, guaranteeing the support polygon always spans the full body.
        #
        # Previous offsets ({5:0,4:0.166,...}) had same-COLUMN overlaps:
        # legs 5(RR)+4(LR) both rear, 6(RM)+3(LM) both middle, 1(RF)+2(LF)
        # both front -- these pairs are simultaneously in air (old duty=0.75,
        # air=0.25 > spacing=0.166), leaving entire rows unsupported and
        # causing the chassis edge to hit the ground.
        #
        # New pattern interleaves columns: rear->middle->front per side.
        # Spacing 0.167; with duty=0.75, same-column legs are >=0.5 apart
        # (air=0.25), so no same-column overlap is possible.
        'offsets': {5: 0.0, 3: 0.167, 1: 0.333, 4: 0.5, 6: 0.667, 2: 0.833}
    },
    2: {  # QUADRUPED
        'duty': 0.70,
        'offsets': {2: 0.0, 5: 0.0,  3: 0.333, 6: 0.333,  4: 0.666, 1: 0.666}
    }
}

# -----------------------------------------------------------------
# CPG / HOPF OSCILLATOR PARAMETERS (Sensors-19-03705)
# -----------------------------------------------------------------
# Exponential ramp rate κ for smooth gait transitions.
# Controls convergence speed of duty (ε) and phase offset (φ) ramps.
# κ·dt ≈ 0.16 at 50 Hz → 95% settled in ~0.6 s (paper recommends 0.5–1.0 s).
KAPPA_TRANSITION = 8.0  # 1/s — exponential decay rate for φ/ε ramps

# Adjacent-leg pairs: no two adjacent legs may be in swing (air) simultaneously.
# Layout: RIGHT front→back = 1,6,5 | LEFT front→back = 2,3,4
# Adjacency includes ipsilateral neighbours AND contralateral front/rear pairs.
ADJACENT_LEGS = {
    1: [6, 2],    # R-front  ↔ R-mid, L-front
    6: [1, 5],    # R-mid    ↔ R-front, R-rear
    5: [6, 4],    # R-rear   ↔ R-mid, L-rear
    2: [3, 1],    # L-front  ↔ L-mid, R-front
    3: [2, 4],    # L-mid    ↔ L-front, L-rear
    4: [3, 5],    # L-rear   ↔ L-mid, R-rear
}

# -----------------------------------------------------------------
# KINEMATIC CORE HELPERS
# -----------------------------------------------------------------
def compute_max_safe_speed(impact_start, impact_end, duty, ff_cap=FEEDFORWARD_CAP):
    """
    Maximum cycle frequency (Hz) where air-phase feedforward stays below
    the cap, avoiding phase lag that reduces chassis ground clearance.

    Physics:
      Air-phase angular velocity = air_sweep × freq / (1 − duty)
      Feedforward = angular_velocity × VELOCITY_SCALAR
      For ff ≤ cap:  freq ≤ cap / (VELOCITY_SCALAR × air_sweep / (1 − duty))

    Returns (max_hz, max_speed_int) where max_speed_int = int(max_hz × 1000).
    """
    stance_sweep = (impact_end - impact_start + 180) % 360 - 180
    air_sweep_deg = 360.0 - abs(stance_sweep)
    if air_sweep_deg < 1.0:
        return 10.0, 9999  # degenerate — near-zero air sweep, no practical limit
    max_deg_per_sec = ff_cap / VELOCITY_SCALAR   # 269.7 °/s at cap=499
    max_hz = max_deg_per_sec * (1.0 - duty) / air_sweep_deg
    return max_hz, int(max_hz * 1000)


def _max_angle_from_vertical(impact_start, impact_end):
    """
    Maximum angle (degrees) from vertical (0°/360°) that occurs during
    the stance sweep from impact_start to impact_end.

    For symmetric sweeps (e.g. 330°/30°) this equals half_sweep.
    For asymmetric sweeps (e.g. 315°/15°) the center is offset from
    vertical, so the worst angle = offset + half_sweep > half_sweep.

    Physics (Sensors-19-03705 §3):
      h(θ) = r·cos(θ) — chassis height depends on angle from vertical.
      Minimum clearance occurs at the maximum |θ| during stance.
      For a sweep centered at angle c from vertical with half-width w,
      the worst angle is |c| + w.
    """
    # Deviation from vertical for each endpoint
    s = impact_start % 360
    e = impact_end % 360
    dev_s = min(s, 360.0 - s)
    dev_e = min(e, 360.0 - e)
    return max(dev_s, dev_e)


def compute_min_clearance(impact_start, impact_end, phase_lag_deg=0.0):
    """
    Minimum chassis ground clearance (mm) during stance phase,
    accounting for phase-tracking lag at stance start.

    Physics (Sensors-19-03705 §3, adapted):
      At the worst stance angle θ_max, the leg deviates furthest from vertical.
      For asymmetric sweeps, θ_max ≠ half_sweep — we compute the actual
      maximum angle from vertical across both endpoints.
      With phase lag, the effective angle is θ_max + lag.
      h(θ) = LEG_EFFECTIVE_RADIUS × cos(θ)
      clearance = h(θ) − SHAFT_TO_CHASSIS_BOTTOM
    """
    worst_angle = _max_angle_from_vertical(impact_start, impact_end) + phase_lag_deg
    h = LEG_EFFECTIVE_RADIUS * math.cos(math.radians(worst_angle))
    return h - SHAFT_TO_CHASSIS_BOTTOM


def compute_max_clearance_hz(impact_start, impact_end, duty,
                             min_clearance=MIN_GROUND_CLEARANCE,
                             ff_cap=FEEDFORWARD_CAP, kp=KP_PHASE,
                             turn_bias=0.0):
    """
    Maximum cycle frequency (Hz) that maintains minimum ground clearance,
    accounting for phase lag from feedforward cap clipping and body roll
    during turns.

    Physics (Sensors-19-03705 S3, adapted for servo dynamics):
      When air-phase feedforward exceeds ff_cap, it is clipped.
      The remaining error is corrected by KP_PHASE at ~kp deg/STS,
      creating a steady-state phase lag = (ff_needed - ff_cap) / kp.
      At stance entry, the leg angle = worst_static_angle + phase_lag.
      For asymmetric sweeps, worst_static_angle > half_sweep.
      clearance = LEG_EFFECTIVE_RADIUS * cos(angle) - SHAFT_TO_CHASSIS_BOTTOM
      During turns, roll_drop from compute_roll_corner_drop() is added to
      the clearance requirement, tightening the Hz limit.
      Solving for max_hz where clearance = min_clearance + roll_drop.

    Returns max_hz (float).  Caller converts to speed via int(max_hz * 1000).
    """
    stance_sweep = (impact_end - impact_start + 180) % 360 - 180
    worst_static = _max_angle_from_vertical(impact_start, impact_end)
    air_sweep = 360.0 - abs(stance_sweep)

    if air_sweep < 1.0 or duty >= 0.99:
        return 10.0  # degenerate -- near-zero air sweep, no practical limit

    # Roll-aware: add corner drop from body tilt during turns
    roll_drop = compute_roll_corner_drop(turn_bias, worst_static, duty)
    effective_clearance = min_clearance + roll_drop

    # Max angle from vertical that still gives effective_clearance
    ratio = (SHAFT_TO_CHASSIS_BOTTOM + effective_clearance) / LEG_EFFECTIVE_RADIUS
    if ratio >= 1.0:
        return 0.0  # geometry cannot provide clearance even at rest
    max_total_angle = math.degrees(math.acos(ratio))

    max_lag = max(0.0, max_total_angle - worst_static)
    if max_lag <= 0.0:
        return 0.0  # impact angles already too wide for clearance at rest

    # max_ff = cap + allowable lag * kp  (invert lag = (ff - cap) / kp)
    max_ff = ff_cap + max_lag * kp
    max_deg_per_sec = max_ff / VELOCITY_SCALAR
    max_hz = max_deg_per_sec * (1.0 - duty) / air_sweep
    return max_hz


def compute_roll_corner_drop(turn_bias, worst_angle, duty):
    """
    Extra clearance loss (mm) at the inside chassis corner from body roll
    during a turn.

    Physics:
      Differential leg speeds (hz_L != hz_R) create asymmetric stance heights.
      Inside legs run faster -> larger worst_angle -> lower h(theta).
      The height difference across W_SHAFT tilts the chassis, and the corner
      overhang amplifies the drop:
        delta_h = r * |cos(theta_outside) - cos(theta_inside)|
        corner_drop = CORNER_OVERHANG * delta_h / W_SHAFT * ROLL_SAFETY_FACTOR

      The inside leg's worst angle is increased by the turn_bias fraction of
      the air sweep, modeling faster rotation on the inside.
    """
    if abs(turn_bias) < 0.001:
        return 0.0

    # Inside leg sees a larger effective angle due to faster rotation.
    # Turn bias shifts the inside leg's phase, widening its worst angle.
    # The 0.5 is a linearization coefficient: actual phase lag depends on
    # feedforward dynamics, but 0.5 * ROLL_SAFETY_FACTOR(2.0) ~= 1.0 net,
    # giving a first-order approximation. Validate on hardware.
    # duty is accepted but not yet used -- reserved for future duty-dependent
    # roll modeling (higher duty = more stance time = more constrained roll).
    bias_angle_shift = abs(turn_bias) * worst_angle * 0.5
    theta_inside = math.radians(worst_angle + bias_angle_shift)
    theta_outside = math.radians(max(0.0, worst_angle - bias_angle_shift))

    h_inside = LEG_EFFECTIVE_RADIUS * math.cos(theta_inside)
    h_outside = LEG_EFFECTIVE_RADIUS * math.cos(theta_outside)
    delta_h = abs(h_outside - h_inside)

    # Corner overhang amplifies the tilt
    corner_drop = CORNER_OVERHANG * delta_h / W_SHAFT * ROLL_SAFETY_FACTOR
    return corner_drop


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


def cpg_asymmetric_omega(base_hz, duty, t_leg):
    """
    Hopf-CPG asymmetric frequency (Sensors-19-03705 Eq. 8).

    During stance (t_leg <= duty) the oscillator advances slower,
    during swing (t_leg > duty) it advances faster, so that the
    total cycle period is preserved while the duty split is honoured.

    Returns the phase-rate multiplier to apply to base_hz for this tick.

    Math:  ω_stance = 1 / duty          (normalised so stance·ω_s + swing·ω_sw = 1)
           ω_swing  = 1 / (1 − duty)
    The caller already integrates master_time += |hz| * real_dt, so we
    return a dimensionless scale factor.

    NOTE: Not called in the Heart loop because the Buehler clock's
    stance/air split already implements this asymmetry geometrically
    (different angular sweeps in different time fractions). Exposed
    for Brain-level analysis, testing, and future CPG extensions.
    """
    if duty <= 0.01 or duty >= 0.99:
        return 1.0  # degenerate — fall back to uniform rate
    if t_leg <= duty:
        return 1.0 / duty        # stance: slower phase advance
    else:
        return 1.0 / (1.0 - duty)  # swing: faster phase advance


def cpg_exp_ramp(current, target, kappa, dt):
    """
    Exponential parameter ramp (Sensors-19-03705 Eq. 12/13).

    φ(t) = φ_target + (φ_current − φ_target) · e^(−κ·dt)

    Replaces linear LERP for smoother, monotonic convergence.
    At κ=8, dt=0.02: factor ≈ 0.85 → 95% settled in ~0.6 s.
    """
    decay = math.exp(-kappa * dt)
    return target + (current - target) * decay


def cpg_exp_ramp_circular(current, target, kappa, dt, period=1.0):
    """
    Circular-aware exponential ramp for phase offsets in [0, period).

    Computes shortest-arc delta, applies exponential decay, wraps result.
    """
    half = period / 2.0
    delta = (target - current + half) % period - half  # shortest signed delta
    new_target = current + delta  # unwrapped target
    result = cpg_exp_ramp(current, new_target, kappa, dt)
    return result % period


def cpg_check_adjacent_swing(smooth_offsets, smooth_duty, master_time_L, master_time_R):
    """
    Adjacent-swing stability check (Sensors-19-03705 Section 4.2).

    Returns True if no two adjacent legs are simultaneously in swing phase.
    Used as a diagnostic — does NOT block commands (safety is via offsets).
    """
    in_swing = {}
    for sid in ALL_SERVOS:
        mt = master_time_L if sid in LEFT_SERVOS else master_time_R
        t_leg = (mt + smooth_offsets[sid]) % 1.0
        in_swing[sid] = (t_leg > smooth_duty)

    for sid, neighbours in ADJACENT_LEGS.items():
        if in_swing.get(sid, False):
            for nbr in neighbours:
                if in_swing.get(nbr, False):
                    return False  # violation: two adjacent legs both in swing
    return True  # stable


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
    # UDP telemetry broadcast (fire-and-forget, non-blocking)
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    udp_sock.setblocking(False)
    # UDP command listener (receives STOP from laptop command sender)
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    cmd_sock.bind(("0.0.0.0", 9877))
    cmd_sock.setblocking(False)
    last_log_time  = 0

    # Verbose telemetry: [H5] servo detail at 5Hz (every 10th tick of 50Hz loop)
    h5_buffer  = []       # buffered [H5] lines, drained at 1Hz with log_telemetry
    h5_counter = 0        # tick counter for 5Hz decimation
    cmd_speeds     = {sid: 0 for sid in ALL_SERVOS}    # per-servo commanded speed (raw units)
    phase_angles   = {sid: 0.0 for sid in ALL_SERVOS}  # per-servo actual phase from encoders (degrees)
    target_angles  = {sid: 0.0 for sid in ALL_SERVOS}  # per-servo Buehler target phase (degrees)

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
                phgov_tag = f" [PhGov:{int(round((1.0 - ph_scale) * 100))}%]" if pherr_gov_active else ""
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
                _hb_line = (f"[{ts}] G:{gait_name} Spd:{fsm_speed} Trn:{fsm_turn:.2f} | "
                        f"V:{volt:.2f}V Vc:{volt_dip_ctr:03d} | "
                        f"T:{temps_str} Tc:{temp_spike_ctr:02d} | "
                        f"L:{loads_str} | Stall:{stall_ids} | "
                        f"A:{total_amps:.2f} | Loop:{loop_ms:.1f}ms"
                        f"{ghost_tag}{vwarn_tag}{twarn_tag}{phgov_tag}"
                        f" | Pdelta:{pos_delta_accum} Jit:{jit_max:.1f}/{jit_std:.1f}ms"
                        f" Gov:{gov_pct:02d}% TE:{te_str} PhErr:{ref_phase_error:+.1f}° MaxPhErr:{max_phase_error_prev:.1f}° PG:{int(pherr_gov_active)}"
                        f" Comm:{comm_fail_streak:02d} StDur:{stall_dur_str}\n")
                f.write(_hb_line)
                # UDP broadcast heartbeat (fire-and-forget for live analyst)
                try:
                    udp_sock.sendto(_hb_line.encode("utf-8"), ("255.255.255.255", 9876))
                except Exception:
                    pass
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
    overrun_buffer = []   # buffered overrun lines, drained at 1 Hz
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
                    f"KP_PHASE={KP_PHASE} OVERLOAD_TIME={OVERLOAD_PREVENTION_TIME}s "
                    f"KAPPA={KAPPA_TRANSITION}\n")
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

    # PhErr governor state (CPG paper Eqs. 20-21 exponential ramp)
    max_phase_error_prev = 0.0   # worst-case servo phase error from previous tick (degrees)
    pherr_gov_active     = False # True when governor is engaged (hysteresis)
    pherr_low_scale_start = 0.0  # monotonic time when governor first hit floor — stuck timeout
    ph_scale             = 1.0   # current governor throttle (1.0 = no throttle)

    prev_loop_ms    = 0.0    # last frame's measured elapsed time — logged in 1Hz heartbeat
    ghost_event_flag = False  # latched True when 125C artifact seen; reset after each log tick
    comm_fail_streak = 0     # consecutive ticks with zero position reads — bus disconnect detection
    prev_gait_id    = shared_gait_id.value  # Fix 65: track gait ID to detect switches for CPG snap

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
                # Check for remote STOP command (UDP from laptop)
                try:
                    cmd_data, _ = cmd_sock.recvfrom(64)
                    if cmd_data.strip() == b"STOP":
                        print("[HEART] Remote STOP received via UDP")
                        is_running.value = False
                        break
                except BlockingIOError:
                    pass
                except Exception:
                    pass

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
                            comm1, _ = packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
                            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_MODE, 1)
                            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_TORQUE_LIMIT, 1000)
                            comm2, _ = packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 1)
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
                            if comm1 == 0 and comm2 == 0:  # COMM_SUCCESS
                                stall_sustained_start[sid] = 0.0
                            # else: leave timer running -- retry on next tick
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
                # Flush pending [H5] verbose telemetry before ring buffer dump
                if h5_buffer:
                    try:
                        with open(LOG_FILE, "a") as f:
                            f.write("".join(h5_buffer))
                        h5_buffer.clear()
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
                # Drain [H5] verbose telemetry buffer (rides the same 1Hz file open)
                if h5_buffer:
                    try:
                        with open(LOG_FILE, "a") as f:
                            f.write("".join(h5_buffer))
                    except:
                        pass
                    # UDP broadcast [H5] lines (fire-and-forget for live analyst)
                    try:
                        for _udp_line in h5_buffer:
                            udp_sock.sendto(_udp_line.encode("utf-8"), ("255.255.255.255", 9876))
                    except Exception:
                        pass
                    h5_buffer.clear()
                # Drain overrun buffer at 1 Hz
                if overrun_buffer:
                    try:
                        with open(LOG_FILE, "a") as f:
                            f.writelines(overrun_buffer)
                    except:
                        pass
                    overrun_buffer.clear()

            # ----------------------------------------------------------
            # 2. STATE SMOOTHING (CPG exponential ramps — Sensors-19-03705)
            # ----------------------------------------------------------
            target_hz   = (shared_speed.value * shared_x_flip.value * shared_z_flip.value) / 1000.0
            target_turn = shared_turn_bias.value

            # Speed/turn: keep linear LERP (command inputs, not CPG parameters)
            lerp_rate    = min(1.0, 4.0 * real_dt)
            smooth_hz   += (target_hz   - smooth_hz)   * lerp_rate
            smooth_turn += (target_turn - smooth_turn) * lerp_rate

            # Impact angles: keep linear circular LERP (geometric, not CPG)
            d_s = (shared_impact_start.value - smooth_imp_start + 180) % 360 - 180
            smooth_imp_start = (smooth_imp_start + d_s * lerp_rate) % 360

            d_e = (shared_impact_end.value - smooth_imp_end + 180) % 360 - 180
            smooth_imp_end = (smooth_imp_end + d_e * lerp_rate) % 360

            current_gait_id = shared_gait_id.value  # single read -- eliminates TOCTOU between gait_params fetch and snap block
            gait_params = GAITS.get(current_gait_id, GAITS[0])
            t_duty      = max(0.01, min(0.99, gait_params['duty']))

            # Phase offsets target (needed by both snap block and CPG ramp below)
            t_offsets = gait_params['offsets']

            # Fix 65: Snap phase offsets on gait switch to avoid CPG ramp lag
            # KAPPA=8 takes ~0.6s to converge; during nav tripod→quad the phase
            # error hits 100-170°. Snap eliminates this instantly. Servos don't
            # see the offset snap because feedforward+KP_PHASE smooth it mechanically.
            if current_gait_id != prev_gait_id:
                smooth_duty = t_duty
                for sid in ALL_SERVOS:
                    smooth_offsets[sid] = t_offsets[sid]
                prev_gait_id = current_gait_id

            # Duty (ε): CPG exponential ramp (Eq. 13 — ε(t) = ε⁺ + (ε⁻−ε⁺)e^(−κΔt))
            smooth_duty = cpg_exp_ramp(smooth_duty, t_duty, KAPPA_TRANSITION, real_dt)

            # Phase offsets (φ): CPG circular exponential ramp (Eq. 12)
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

            max_safe_hz = (GOVERNOR_FF_BUDGET / VELOCITY_SCALAR * (1.0 - smooth_duty)) / max(5.0, abs(air_sweep))

            # Clearance governor: limit Hz so chassis stays above MIN_GROUND_CLEARANCE.
            # Roll-aware: during turns, body tilt drops inside chassis corner closer
            # to ground. compute_max_clearance_hz internally adds roll_drop to the
            # clearance requirement, tightening the Hz limit proportionally to turn_bias.
            # (Sensors-19-03705 Section 3 -- bounded swing velocity for stability)
            max_clr_hz = compute_max_clearance_hz(smooth_imp_start, smooth_imp_end, smooth_duty,
                                                  min_clearance=MIN_GROUND_CLEARANCE + GOVERNOR_CLEARANCE_MARGIN,
                                                  turn_bias=smooth_turn)
            max_safe_hz = min(max_safe_hz, max_clr_hz)

            # PhErr governor: throttle Hz when servos can't track commanded phase
            # Exponential ramp (CPG paper Eqs. 20-21) — gentle near threshold, aggressive at high error
            if max_phase_error_prev >= PHERR_ENGAGE_DEG:
                pherr_gov_active = True
            elif max_phase_error_prev < PHERR_RELEASE_DEG:
                pherr_gov_active = False
            if pherr_gov_active:
                ph_scale = PHERR_FLOOR_SCALE + (1.0 - PHERR_FLOOR_SCALE) * math.exp(
                    -KAPPA_GOVERNOR * (max_phase_error_prev - PHERR_ENGAGE_DEG) / PHERR_RAMP_WIDTH)
                max_safe_hz *= ph_scale
                # Stuck timeout: if at floor scale too long, escalate to stall recovery
                if ph_scale <= PHERR_FLOOR_SCALE + 0.01:
                    if pherr_low_scale_start == 0.0:
                        pherr_low_scale_start = time.monotonic()
                    elif (time.monotonic() - pherr_low_scale_start) > PHERR_STUCK_TIMEOUT:
                        for sid in ALL_SERVOS:
                            is_stalled[sid] = True  # triggers existing stall/wiggle recovery
                        pherr_low_scale_start = 0.0
                else:
                    pherr_low_scale_start = 0.0
            else:
                ph_scale = 1.0  # governor inactive -- no throttle
                pherr_low_scale_start = 0.0

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
            max_phase_error_frame = 0.0  # worst-case servo error this tick
            tick_max_pherr = 0.0        # redundant tracker (used in feedback block)
            for sid in ALL_SERVOS:
                target_phase = 0.0  # safe default -- overwritten by Buehler calc or roll mode
                leg_hz    = hz_L if sid in LEFT_SERVOS else hz_R
                is_rev_leg = (leg_hz < 0)

                if shared_roll_mode.value:
                    # Counter-rotating roll: bypass Buehler clock entirely.
                    # Front/rear pairs spin opposite directions; sin(±35° splay)
                    # creates additive rolling torque about the body long axis.
                    base_speed = shared_speed.value
                    if sid in ROLL_FRONT_SERVOS:
                        raw_speed = float(base_speed)
                    elif sid in ROLL_REAR_SERVOS:
                        raw_speed = float(-base_speed)
                    else:  # middle servos (3, 6) — zero splay, no roll torque but
                        raw_speed = float(base_speed)  # spin to prevent kickstand effect
                    # Normal direction corrections for physical servo mounting
                    if DIRECTION_MAP[sid] < 0:
                        raw_speed = -raw_speed
                    if sid in LEFT_SERVOS:
                        raw_speed = -raw_speed
                    final_speed = max(-SERVO_SPEED_GOVERNOR_CAP, min(SERVO_SPEED_GOVERNOR_CAP, int(raw_speed)))
                elif is_stalled[sid] or servo_disabled[sid] or abs(leg_hz) < 0.001:
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

                    ff_speed = min(FEEDFORWARD_CAP, deg_per_sec * VELOCITY_SCALAR)

                    if sid == 1:
                        ref_ff_speed = ff_speed
                        ref_phase_angle = target_phase
                        ref_phase_error = (target_phase - current_phase + 180) % 360 - 180

                    error       = target_phase - current_phase
                    short_error = (error + 180) % 360 - 180
                    max_phase_error_frame = max(max_phase_error_frame, abs(short_error))

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

                    correction = short_error * KP_PHASE if abs(short_error) >= PHERR_DEADBAND else 0.0
                    directed_ff = -ff_speed if is_rev_leg else ff_speed  # only flip feedforward, not correction
                    raw_speed = directed_ff + correction
                    tick_max_pherr = max(tick_max_pherr, abs(short_error))
                    if sid in LEFT_SERVOS and not shared_roll_mode.value:
                        raw_speed = -raw_speed

                    final_speed = max(-1023, min(1023, int(raw_speed)))

                # Capture for [H5] verbose telemetry (no overhead when disabled -- just dict writes)
                cmd_speeds[sid] = final_speed
                phase_angles[sid] = actual_phases.get(sid, 0.0)
                target_angles[sid] = target_phase if not (is_stalled[sid] or servo_disabled[sid] or abs(leg_hz) < 0.001) else phase_angles[sid]

                abs_speed = int(abs(final_speed))
                speed_val = (abs_speed | 0x8000) if final_speed < 0 else abs_speed
                group_sync_write.addParam(sid, [speed_val & 0xff, (speed_val >> 8) & 0xff])

            group_sync_write.txPacket()
            group_sync_write.clearParam()

            max_phase_error_prev = max_phase_error_frame  # carry to next tick for governor

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

            # [H5] verbose telemetry: servo positions + commanded speeds + phase angles at 5Hz
            h5_counter += 1
            if VERBOSE_TELEMETRY and h5_counter % 10 == 0:
                t_off = time.monotonic() - heart_start_mono
                pos_str = ",".join(str(raw_positions.get(sid, 0)) for sid in ALL_SERVOS)
                spd_str = ",".join(str(cmd_speeds[sid]) for sid in ALL_SERVOS)
                pha_str = ",".join(f"{phase_angles[sid]:.1f}" for sid in ALL_SERVOS)
                tgt_str = ",".join(f"{target_angles[sid]:.1f}" for sid in ALL_SERVOS)
                h5_buffer.append(
                    f"[H5] T+{t_off:.3f} P:{pos_str} S:{spd_str} "
                    f"Ph:{pha_str} PhT:{tgt_str} FF:{ref_ff_speed:.0f} G:{int(gov_active)} D:{smooth_duty:.2f}\n")

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
                if overrun_streak >= 3 and (overrun_streak == 3 or overrun_streak % 50 == 0):
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
            # V0.5.01 safety addition: explicitly zero speed in case mode switch back to velocity failed, 
            # which would leave the legs spinning at last speed instead of holding position.
            # packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 0)  # safety: zero velocity in case mode switch faile
            packet_handler.write2ByteTxRx(port_handler, sid, ADDR_GOAL_SPEED, 0)  # safety: zero velocity before torque disable to prevent runaway motion
            packet_handler.write1ByteTxRx(port_handler, sid, ADDR_TORQUE_ENABLE, 0)
        port_handler.closePort()
        print("[heart] offline")


# =================================================================
# PROCESS 1: THE BRAIN (MISSION SEQUENCER)
# =================================================================

class EmergencyStopException(Exception):
    pass

# V0.5.01 addition: tactical sleep function that monitors both the is_running flag and the heart process's liveness,
#  to ensure that if the heart crashes during a critical phase (e.g. self-righting roll), the brain detects it and triggers 
# a clean shutdown instead of continuing blindly.
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



# ===================================================================

if __name__ == "__main__":
    print("hexapod starting up...")

    # Multi-Processing Primitives
    shared_speed        = mp.Value('i', 0)
    shared_x_flip       = mp.Value('i', 1)
    # V0.5.01 update: change shared_z_flip from constant to variable so it can be set to -1 during inverted roll,
    #shared_z_flip       = mp.Value('i', 1)   # NOTE: never written after init - always 1.
    shared_z_flip       = mp.Value('i', 1)   # Written to -1 by state_self_right_roll when inverted,
                                              #
                                              # Setting -1 flips effective forward/backward for
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
    shared_impact_start = mp.Value('i', DEFAULT_IMPACT_START)  # 345/15 = 30° sweep
    shared_impact_end   = mp.Value('i', DEFAULT_IMPACT_END)
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

        # Reset to standard window - avoids inheriting stale state (e.g. COG Shift 330/15)
        set_gait_state(impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END, step_name="wiggle_window_reset")

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
                tsleep(0.5)
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
    ARDUINO_PORT = "/dev/ttyUSB0"
    ARDUINO_BAUD = 115200

    # --- Nav tunable constants ---
    CRUISE_SPEED = 350          # under governor limit at 30° sweep with duty 0.75
    TRIPOD_CRUISE_SPEED = 420   # under FF governor limit at 30° sweep
    SLOW_SPEED = 200
    BACKWARD_SPEED = 300
    BACKWARD_MIN_DWELL = 0.8          # seconds in BACKWARD before allowing pivot escalation
    CLIFF_BACKUP_DURATION = 5.0       # seconds of forced backward on front cliff before escape
    OBSTACLE_BACKUP_DURATION = 3.0    # seconds of forced backward when front blocked + both sides NEAR
    MAX_TURN_BIAS = 0.25              # restored: r=125mm has ample roll clearance headroom
    PIVOT_TURN_BIAS = 0.28            # reduced from 0.35 -- stays within roll-aware clearance governor
    PIVOT_IMPACT_START = 345          # ° — narrowed 30° stance sweep for safe pivot clearance
    PIVOT_IMPACT_END   = 15           # ° — same as default 345/15, explicit for pivot clarity
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
    RAPID_ROTATION_THRESHOLD = 3.5     # Fix 73: walking oscillation peaks ~2.0 rad/s
    NAV_SENSOR_KEYS = ("FDL", "FCF", "FCD", "FDR", "RDL", "RCF", "RCD", "RDR")  # Fix A7: hoisted from inline loop
    CLIFF_WARMUP = 5  # Fix 72: frames to skip during sensor settle

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

    def classify_sectors_voted(frame):
        """Classify sectors with majority voting to reject outlier sensors.
        Front (3 sensors): median severity rejects one outlier.
        Left/Right (2 sensors): if readings disagree by 2+ levels,
        downgrade to (max - 1) instead of trusting the worst reading.
        Blind-zone (-1.0) is hard DANGER -- not outvotable (confirmed by
        double-ping in firmware, so physically real)."""
        front_blind = any(frame[k] == -1 for k in ("FDL", "FCF", "FDR"))
        left_blind = any(frame[k] == -1 for k in ("FDL", "RDL"))
        right_blind = any(frame[k] == -1 for k in ("FDR", "RDR"))
        fdl = classify_distance(frame["FDL"])
        fcf = classify_distance(frame["FCF"])
        fdr = classify_distance(frame["FDR"])
        rdl = classify_distance(frame["RDL"])
        rdr = classify_distance(frame["RDR"])
        front = sorted([fdl, fcf, fdr])[1]
        if front_blind:
            front = DIST_DANGER
        left_max = max(fdl, rdl)
        if left_blind:
            left = DIST_DANGER
        elif left_max - min(fdl, rdl) >= 2:
            left = left_max - 1
        else:
            left = left_max
        right_max = max(fdr, rdr)
        if right_blind:
            right = DIST_DANGER
        elif right_max - min(fdr, rdr) >= 2:
            right = right_max - 1
        else:
            right = right_max
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

    class CliffDetector:
        """Detects cliffs using ground EMA + delta + consecutive frame confirmation."""

        def __init__(self, alpha=0.2):
            self._alpha = alpha
            self._ground_ema = 25.0       # Fix 74: raised from 15.0 to prevent false cliff on startup
            self._consecutive_front = 0
            self._consecutive_rear = 0
            self._warmup_frames = 0       # Fix 72: skip detection during sensor settle

        def update(self, fcd, rcd):
            """Update cliff detection with new FCD and RCD readings.
            Returns (front_cliff, rear_cliff) booleans."""
            self._warmup_frames += 1
            front_cliff = self._check_cliff(fcd, is_front=True)
            rear_cliff = self._check_cliff(rcd, is_front=False)
            return front_cliff, rear_cliff

        def _check_cliff(self, reading, is_front):
            """Check single cliff sensor. Returns True if cliff confirmed."""
            counter_attr = "_consecutive_front" if is_front else "_consecutive_rear"
            count = getattr(self, counter_attr)

            if reading == -1:
                # Ground in blind zone (very close) — NOT a cliff. Reset.
                setattr(self, counter_attr, 0)
                return False

            # Fix 74: acoustic scatter at max range reads 300cm — not a cliff
            if reading >= 300.0:
                setattr(self, counter_attr, 0)
                return False

            if reading is None:
                # Defensive: possible cliff, increment
                setattr(self, counter_attr, count + 1)
                return count + 1 >= 2

            # Update ground EMA with valid low readings (runs during warmup so baseline converges)
            if 0 < reading <= 40:
                self._ground_ema = self._alpha * reading + (1 - self._alpha) * self._ground_ema

            # Fix 72: suppress detection during sensor settle
            if self._warmup_frames <= CLIFF_WARMUP:
                return False

            # Cliff candidate: absolute > 30 OR delta > EMA + 10
            is_candidate = (reading > 30) or (reading > self._ground_ema + 10)

            if is_candidate:
                setattr(self, counter_attr, count + 1)
                return count + 1 >= 2
            else:
                setattr(self, counter_attr, 0)
                return False

    # IMU processing constants
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

        IMU_GRACE_PERIOD_S = 2.0  # seconds — ignore IMU safety checks during startup (BNO085 UART-RVC init)

        def __init__(self):
            self.state = NAV_FORWARD
            self.prev_state = NAV_FORWARD
            self.dwell_start = 0.0
            self.dwell_duration = 0.0
            self.hold_position_count = 0
            self.backward_entry_time = 0.0  # monotonic time when BACKWARD was entered
            self.cliff_backup_until = 0.0   # monotonic deadline for cliff backup lockout
            self.obstacle_backup_until = 0.0  # monotonic deadline for obstacle backup lockout
            self.consecutive_pivot_count = 0
            self.pivot_direction = -1  # -1=left, +1=right
            self.stall_start_time = 0.0
            self.stall_count_30s = 0
            self.last_stall_time = 0.0
            self.stall_speed_mult = 1.0
            self.mission_start = time.monotonic()
            self._nav_start = time.monotonic()  # for IMU grace period
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
            self.terrain_impact_start = DEFAULT_IMPACT_START
            self.terrain_impact_end = DEFAULT_IMPACT_END
            self.terrain_mult = 1.0
            self.terrain_is_tripod = False
            self._gait_transition_until = 0.0
            self.sensor_ema = {}  # per-sensor EMA state, keyed by sensor index 0-7

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
                self.dwell_duration = 0  # Fix 68 (A4): reset stale dwell on state change
                brain_log(f"[NAV] {NAV_STATE_NAMES.get(self.prev_state)}→{NAV_STATE_NAMES.get(new_state)}")

        def smooth_sensor(self, idx, raw_cm):
            """EMA smoothing for ultrasonic sensors. alpha=0.3, passthrough for -1/None."""
            if raw_cm is None or raw_cm == -1:
                return raw_cm
            if idx not in self.sensor_ema:
                self.sensor_ema[idx] = raw_cm
                return raw_cm
            alpha = 0.3
            self.sensor_ema[idx] = alpha * raw_cm + (1.0 - alpha) * self.sensor_ema[idx]
            return self.sensor_ema[idx]

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

            # IMU grace period: BNO085 UART-RVC mode needs ~1-2s to produce valid
            # quaternions.  During this window the IMU may report all-zeros, which
            # compute_imu() maps to upright_quality=0.0.  Skipping IMU-dependent
            # safety checks (P1/P2/P5) avoids a false STOP_SAFE on startup.
            imu_ready = (now - self._nav_start) >= self.IMU_GRACE_PERIOD_S

            # P1: Tipover (skip during IMU grace period)
            if imu_ready and upright < 0.15:
                self._transition(NAV_STOP_SAFE)
                brain_log(f"[NAV] tipover upright={upright:.2f}")
                return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_stop_safe")

            # P2: Unexpected rapid rotation (not during pivot; skip during IMU grace)
            if imu_ready and angular_rate > RAPID_ROTATION_THRESHOLD and self.state not in (NAV_PIVOT_TURN, NAV_ARC_LEFT, NAV_ARC_RIGHT, NAV_BACKWARD, NAV_WIGGLE):
                self._transition(NAV_STOP_SAFE)
                brain_log(f"[NAV] rapid rotation {angular_rate:.2f} rad/s")
                return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_stop_safe")

            # P3: Critical battery
            if voltage < VOLTAGE_MIN:
                self._transition(NAV_STOP_SAFE)
                brain_log(f"[NAV] critical voltage {voltage:.1f}V")
                return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_stop_safe")

            # P4: Arduino watchdog (caller handles stale detection, passes frame=None)
            # Handled at integration level — if stale > 5s, caller forces STOP_SAFE

            # P5: Freefall (accel < 8 sustained 0.5s AND upright < 0.5; skip during IMU grace)
            if imu_ready and accel_mag < 8.0 and upright < 0.5:
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
                    self.backward_entry_time = time.monotonic()
                    self.cliff_backup_until = now + CLIFF_BACKUP_DURATION
                    self.obstacle_backup_until = 0.0  # mutual exclusion with obstacle lockout
                self._transition(NAV_BACKWARD)
                self._start_dwell(0.8)  # Fix A5a: refreshes every frame; dwell counts from last cliff
                return self._backward_action(frame)

            # P7: Rear cliff (overrides cliff lockout -- don't back into a drop-off)
            if rear_cliff:
                self.cliff_backup_until = 0.0  # clear lockout on rear cliff
                self.obstacle_backup_until = 0.0  # clear obstacle lockout on rear cliff
                self._transition(NAV_SLOW_FORWARD)
                speed = int(SLOW_SPEED * self.terrain_mult * self.stall_speed_mult)
                return (NAV_SLOW_FORWARD, speed, 0.0, 1, "nav_slow_fwd")

            # P6b: Cliff backup lockout -- forced BACKWARD until duration expires, then escape
            if self.cliff_backup_until > 0.0:
                if now < self.cliff_backup_until:
                    # Lockout active -- stay BACKWARD regardless of other sensors
                    self._transition(NAV_BACKWARD)
                    return self._backward_action(frame)
                else:
                    # Lockout expired -- clear and escape toward free side
                    self.cliff_backup_until = 0.0
                    if left_class < DIST_DANGER or right_class < DIST_DANGER:
                        if left_class != right_class:
                            escape_dir = -1 if left_class < right_class else 1
                        else:
                            l_cm = max(frame.get("FDL") or 0, frame.get("RDL") or 0)
                            r_cm = max(frame.get("FDR") or 0, frame.get("RDR") or 0)
                            escape_dir = -1 if l_cm >= r_cm else 1
                        if escape_dir < 0:
                            self._transition(NAV_ARC_LEFT)
                        else:
                            self._transition(NAV_ARC_RIGHT)
                        self._start_dwell(0.8)
                        turn = escape_dir * abs(turn_intensity) * MAX_TURN_BIAS
                        speed = int(SLOW_SPEED * self.terrain_mult * self.stall_speed_mult)
                        step = "nav_cliff_escape_L" if escape_dir < 0 else "nav_cliff_escape_R"
                        state = NAV_ARC_LEFT if escape_dir < 0 else NAV_ARC_RIGHT
                        return (state, speed, turn, 1, step)
                    else:
                        # Both sides blocked -- pivot away from cliff
                        self._pick_pivot_direction(frame)
                        self._transition(NAV_PIVOT_TURN)
                        self._start_dwell(1.5)
                        self.consecutive_pivot_count += 1
                        self.terrain_impact_start = PIVOT_IMPACT_START
                        self.terrain_impact_end = PIVOT_IMPACT_END
                        turn = self.pivot_direction * PIVOT_TURN_BIAS
                        step = "nav_cliff_pivot_L" if self.pivot_direction < 0 else "nav_cliff_pivot_R"
                        return (NAV_PIVOT_TURN, 0, turn, 1, step)

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
                self.obstacle_backup_until = 0.0  # clear so pivot isn't recaptured by P10b
                # Narrow stance sweep for safe clearance during zero-speed turns
                self.terrain_impact_start = PIVOT_IMPACT_START
                self.terrain_impact_end = PIVOT_IMPACT_END
                turn = self.pivot_direction * PIVOT_TURN_BIAS
                step = "nav_pivot_L" if self.pivot_direction < 0 else "nav_pivot_R"
                return (NAV_PIVOT_TURN, 0, turn, 1, step)

            # P10: Front DANGER (2+ frames, not dead-end) -- escape-route awareness
            if self.front_danger_frames >= 2:
                # Prefer arc escape over backward when a side is clear
                if left_class < DIST_NEAR or right_class < DIST_NEAR:
                    # Pick the freer side; tie-break with raw cm (most clearance)
                    if left_class != right_class:
                        escape_dir = -1 if left_class < right_class else 1
                    else:
                        l_cm = max(frame.get("FDL") or 0, frame.get("RDL") or 0)
                        r_cm = max(frame.get("FDR") or 0, frame.get("RDR") or 0)
                        escape_dir = -1 if l_cm >= r_cm else 1
                    if escape_dir < 0:
                        self._transition(NAV_ARC_LEFT)
                    else:
                        self._transition(NAV_ARC_RIGHT)
                    self._start_dwell(0.8)
                    turn = escape_dir * abs(turn_intensity) * MAX_TURN_BIAS
                    speed = int(SLOW_SPEED * self.terrain_mult * self.stall_speed_mult)
                    step = "nav_escape_L" if escape_dir < 0 else "nav_escape_R"
                    state = NAV_ARC_LEFT if escape_dir < 0 else NAV_ARC_RIGHT
                    return (state, speed, turn, 1, step)
                else:
                    # Both sides NEAR or worse -- timed BACKWARD + escape turn
                    if self.state != NAV_BACKWARD:
                        self.hold_position_count = 0
                        self.backward_entry_time = time.monotonic()
                        self.obstacle_backup_until = now + OBSTACLE_BACKUP_DURATION
                    self._transition(NAV_BACKWARD)
                    self._start_dwell(0.8)
                    return self._backward_action(frame)

            # P10b: Obstacle backup lockout -- forced BACKWARD until duration expires, then escape
            if self.obstacle_backup_until > 0.0:
                if now < self.obstacle_backup_until:
                    # Lockout active -- stay BACKWARD regardless of front sensor clearing
                    self._transition(NAV_BACKWARD)
                    return self._backward_action(frame)
                else:
                    # Lockout expired -- clear and escape toward free side (never straight FORWARD)
                    self.obstacle_backup_until = 0.0
                    if left_class < DIST_DANGER or right_class < DIST_DANGER:
                        if left_class != right_class:
                            escape_dir = -1 if left_class < right_class else 1
                        else:
                            l_cm = max(frame.get("FDL") or 0, frame.get("RDL") or 0)
                            r_cm = max(frame.get("FDR") or 0, frame.get("RDR") or 0)
                            escape_dir = -1 if l_cm >= r_cm else 1
                        if escape_dir < 0:
                            self._transition(NAV_ARC_LEFT)
                        else:
                            self._transition(NAV_ARC_RIGHT)
                        self._start_dwell(0.8)
                        turn = escape_dir * abs(turn_intensity) * MAX_TURN_BIAS
                        speed = int(SLOW_SPEED * self.terrain_mult * self.stall_speed_mult)
                        step = "nav_obs_escape_L" if escape_dir < 0 else "nav_obs_escape_R"
                        state = NAV_ARC_LEFT if escape_dir < 0 else NAV_ARC_RIGHT
                        return (state, speed, turn, 1, step)
                    else:
                        # Both sides still blocked -- pivot away
                        self._pick_pivot_direction(frame)
                        self._transition(NAV_PIVOT_TURN)
                        self._start_dwell(1.5)
                        self.consecutive_pivot_count += 1
                        self.terrain_impact_start = PIVOT_IMPACT_START
                        self.terrain_impact_end = PIVOT_IMPACT_END
                        turn = self.pivot_direction * PIVOT_TURN_BIAS
                        step = "nav_obs_pivot_L" if self.pivot_direction < 0 else "nav_obs_pivot_R"
                        return (NAV_PIVOT_TURN, 0, turn, 1, step)

            # Reset pivot count when front clears
            if front_class < DIST_DANGER:
                self.consecutive_pivot_count = 0

            # P11: Lateral obstacle, one side freer
            # Arc dwell hold: if already arcing with active dwell, hold the arc.
            # If obstacle persists, refresh the dwell. If obstacle cleared, let dwell
            # expire naturally but don't snap to FORWARD mid-dwell.
            if self.state in (NAV_ARC_LEFT, NAV_ARC_RIGHT) and self._dwell_active():
                if self.state == NAV_ARC_LEFT and left_class >= DIST_NEAR:
                    self._refresh_dwell(0.4)
                elif self.state == NAV_ARC_RIGHT and right_class >= DIST_NEAR:
                    self._refresh_dwell(0.4)
                # Hold arc during dwell regardless of obstacle state
                turn = (-1 if self.state == NAV_ARC_LEFT else 1) * abs(turn_intensity) * MAX_TURN_BIAS
                speed = int(SLOW_SPEED * self.terrain_mult * self.stall_speed_mult)
                step = "nav_arc_L_hold" if self.state == NAV_ARC_LEFT else "nav_arc_R_hold"
                return (self.state, speed, turn, 1, step)

            if (left_class >= DIST_NEAR or right_class >= DIST_NEAR):
                if left_class != right_class:
                    if left_class < right_class:
                        self._transition(NAV_ARC_LEFT)
                        self._start_dwell(0.6)  # Fix A5c: reordered; dwell set after transition
                        turn = -abs(turn_intensity) * MAX_TURN_BIAS
                        speed = int(SLOW_SPEED * self.terrain_mult * self.stall_speed_mult)
                        return (NAV_ARC_LEFT, speed, turn, 1, "nav_arc_L")
                    else:
                        self._transition(NAV_ARC_RIGHT)
                        self._start_dwell(0.6)  # Fix A5d: reordered; dwell set after transition
                        turn = abs(turn_intensity) * MAX_TURN_BIAS
                        speed = int(SLOW_SPEED * self.terrain_mult * self.stall_speed_mult)
                        return (NAV_ARC_RIGHT, speed, turn, 1, "nav_arc_R")

                # V0.5.01 update — if both sides equally blocked, don't prefer one arc over the other,
                # just slow down and go straight to reassess.  This mitigates oscillation when both sides are borderline.
                else:
                    # Both sides equally blocked — slow down, drive straight
                    self._transition(NAV_SLOW_FORWARD)
                    speed = int(SLOW_SPEED * self.terrain_mult * self.stall_speed_mult)
                    return (NAV_SLOW_FORWARD, speed, 0.0, 1, "nav_slow_fwd")

            # NOTE: P12 is structurally unreachable (P11 fires first when both sides >= NEAR). Kept as defense-in-depth.
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
            """Compute backward recovery action with rear safety check and dwell guard."""
            if is_rear_safe(frame):
                self.hold_position_count = 0
                return (NAV_BACKWARD, BACKWARD_SPEED, 0.0, -1, "nav_backward")
            else:
                self.hold_position_count += 1
                brain_log(f"[NAV] rear unsafe, holding (count={self.hold_position_count})")
                backward_elapsed = time.monotonic() - self.backward_entry_time
                if self.hold_position_count >= 2 and backward_elapsed >= BACKWARD_MIN_DWELL:
                    # Stuck long enough — try pivot
                    self._pick_pivot_direction(frame)
                    self._transition(NAV_PIVOT_TURN)
                    self.cliff_backup_until = 0.0  # clear lockout so pivot can complete
                    self.obstacle_backup_until = 0.0  # clear obstacle lockout so pivot can complete
                    self._start_dwell(1.5)
                    self.consecutive_pivot_count += 1
                    # Narrow stance sweep for safe clearance during zero-speed turns
                    self.terrain_impact_start = PIVOT_IMPACT_START
                    self.terrain_impact_end = PIVOT_IMPACT_END
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

            # T1: Steep climb — symmetric narrow sweep (345°/15°)
            # 30° total sweep, centered on vertical.
            # Worst angle from vertical = 15° → clearance ~73.7mm (safe).
            # Old 330°/15° had worst angle 30° → clearance 61.2mm (< 70mm effective with margin).
            if pitch_deg > 15:
                if self._steep_up_start == 0.0:
                    self._steep_up_start = now
                if (now - self._steep_up_start) >= 1.0:
                    self.terrain_gait = 1  # wave
                    self.terrain_impact_start = DEFAULT_IMPACT_START
                    self.terrain_impact_end = DEFAULT_IMPACT_END
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
                    self.terrain_impact_start = DEFAULT_IMPACT_START
                    self.terrain_impact_end = DEFAULT_IMPACT_END
                    self.terrain_mult = 0.5
                    self.terrain_is_tripod = False
                    self._apply_gait_transition(prev_gait)
                    return
            else:
                self._steep_down_start = 0.0

            # T3: Moderate slope (uphill or downhill) or tilted
            if abs(pitch_deg) > SLOPE_PITCH_DEG or (0.15 <= upright <= 0.5):
                self.terrain_gait = 1  # wave
                self.terrain_impact_start = DEFAULT_IMPACT_START
                self.terrain_impact_end = DEFAULT_IMPACT_END
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
                    self.terrain_impact_start = DEFAULT_IMPACT_START
                    self.terrain_impact_end = DEFAULT_IMPACT_END
                    self.terrain_mult = 0.5
                    self.terrain_is_tripod = False
                    self._apply_gait_transition(prev_gait)
                    return
            else:
                self._heavy_load_start = 0.0

            # T5: Excessive wobble
            if angular_rate > 0.3:
                self.terrain_impact_start = DEFAULT_IMPACT_START
                self.terrain_impact_end = DEFAULT_IMPACT_END
                self.terrain_mult = 0.7
                self.terrain_is_tripod = False
                return  # keep current gait

            # T6: Rocky terrain (flicker + moderate load)
            if flicker_count >= FLICKER_COUNT_THRESHOLD and avg_load > 300:
                self.terrain_impact_start = DEFAULT_IMPACT_START
                self.terrain_impact_end = DEFAULT_IMPACT_END
                self.terrain_mult = 0.8
                self.terrain_is_tripod = False
                return  # keep current gait

            # T7: High vibration
            if accel_mag > 14:
                if self._high_vibe_start == 0.0:
                    self._high_vibe_start = now
                if (now - self._high_vibe_start) >= 0.5:
                    self.terrain_impact_start = DEFAULT_IMPACT_START
                    self.terrain_impact_end = DEFAULT_IMPACT_END
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
                    self.terrain_impact_start = DEFAULT_IMPACT_START
                    self.terrain_impact_end = DEFAULT_IMPACT_END
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
            self.terrain_impact_start = DEFAULT_IMPACT_START
            self.terrain_impact_end = DEFAULT_IMPACT_END
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
                    self.terrain_impact_start = DEFAULT_IMPACT_START
                    self.terrain_impact_end = DEFAULT_IMPACT_END
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
          Leg radius = 125mm, arc = 190°. Chassis 510x280x80mm.
          Inverted window: 140/220 (±40° around 180°). Legs reach floor when inverted. ✓

        Physics caveat: even with DIRECTION_MAP bypass, lateral friction on wet sand
        is ~4x insufficient to roll a 2.5kg robot. May work on hard surfaces only.
        """
        # --- C2: Orientation guard ---
        # Switch to wave gait for reliable upright detection (5 legs in stance vs tripod's 3)
        saved_gait_for_check = shared_gait_id.value
        #shared_gait_id.value = 1   # WAVE — duty 0.85, 5 legs in stance
        # V0.5.01
        shared_gait_id.value = 1   # WAVE — duty 0.70, max legs in stance for load detection
        shared_speed.value = 0
        tsleep(1.5)                 # Wait for LERP convergence to wave offsets (tsleep detects Heart crash)

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
            shared_gait_id.value = saved_gait_for_check  # restore gait before early return
            return

        brain_log("[ROLL-CHECK] → ATTEMPT")
        print("[recovery] attempting momentum roll to self-right")

        ROLL_LOAD_SPEED   =  400   # step 1 - direction unverified inverted
        ROLL_SNAP_SPEED   = -499   # step 2 - capped to SERVO_SPEED_GOVERNOR_CAP (was -600, silently clamped)
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
            # Confirm clearance governor integration
            default_duty = GAITS[2]['duty']  # quad is default gait
            default_clr_hz = compute_max_clearance_hz(DEFAULT_IMPACT_START, DEFAULT_IMPACT_END, default_duty,
                                                    min_clearance=MIN_GROUND_CLEARANCE + GOVERNOR_CLEARANCE_MARGIN)
            print(f"  Clearance governor: ACTIVE (Sensors-19-03705)")
            print(f"    default config: {DEFAULT_IMPACT_START}°/{DEFAULT_IMPACT_END}° quad(duty={default_duty}) → "
                  f"max_speed={int(default_clr_hz * 1000)}")
            print(f"    body: radius={LEG_EFFECTIVE_RADIUS}mm, "
                  f"shaft_to_bottom={SHAFT_TO_CHASSIS_BOTTOM}mm, "
                  f"min_clearance={MIN_GROUND_CLEARANCE}mm")

            # --- Timed fallback sequence (used when sensors unavailable) ---
            def run_timed_fallback():
                brain_log("FALLBACK: timed sequence — no sensor data")
                set_gait_state(gait=2, impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END,
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
                        # V0.5.01
                        # Reset stall count on mission start — if we have sensor data, we can track stalls accurately from the beginning
                        roll_attempts = 0
                        MAX_ROLL_ATTEMPTS = 2
                        # Set initial gait: quadruped, normal stance
                        set_gait_state(gait=2, impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END,
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

                        # V0.5.01 - Add state name logging for better visibility into FSM behavior in logs
                        nav_tick = 0
                        prev_state_name_comp = "FORWARD"
                        SEVERITY_LABELS_COMP = {0: "CLEAR", 1: "CAUTION", 2: "NEAR", 3: "DANGER"}
                        GAIT_LABELS_COMP = {0: "TRIPOD", 1: "WAVE", 2: "QUAD"}

                        # Verbose telemetry buffers for [BS] sensor and [BN] nav decision lines
                        bs_buffer = []       # [BS] sensor lines, flushed 1Hz
                        bn_buffer = []       # [BN] nav decision lines, flushed 1Hz
                        bs_counter = 0       # tick counter for 2Hz sensor decimation
                        last_brain_flush = time.monotonic()
                        # [BN] change detection: only log when state/speed/turn changes or 1Hz heartbeat
                        last_bn_state = ""
                        last_bn_speed = -1
                        last_bn_turn = -999.0
                        last_bn_time = 0.0

                        # UDP socket for Brain telemetry broadcast (independent of Heart's socket)
                        brain_udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                        brain_udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                        brain_udp_sock.setblocking(False)

                        def brain_flush_buffers():
                            """Drain [BS] and [BN] buffers to telemetry log."""
                            if not bs_buffer and not bn_buffer:
                                return
                            try:
                                with open(LOG_FILE, "a") as f:
                                    f.write("".join(bs_buffer) + "".join(bn_buffer))
                            except:
                                pass
                            # UDP broadcast [BS]/[BN] lines (fire-and-forget for live analyst)
                            try:
                                for _udp_line in bs_buffer + bn_buffer:
                                    brain_udp_sock.sendto(_udp_line.encode("utf-8"), ("255.255.255.255", 9876))
                            except Exception:
                                pass
                            bs_buffer.clear()
                            bn_buffer.clear()

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
                                        set_gait_state(gait=2, impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END,
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
                            # Save raw cliff sensor readings BEFORE EMA -- cliff detector has
                            # its own 2-frame confirmation and ground EMA; double-smoothing adds
                            # 3-4 frames of latency to cliff detection (safety concern).
                            raw_fcd = frame.get("FCD")
                            raw_rcd = frame.get("RCD")
                            # EMA smooth all 8 ultrasonic readings before classification
                            # (-1 and None pass through unmodified; alpha=0.3)
                            _sensor_keys = NAV_SENSOR_KEYS
                            frame = dict(frame)  # shallow copy — do not mutate shared reader state
                            for _si, _sk in enumerate(_sensor_keys):
                                frame[_sk] = nav.smooth_sensor(_si, frame[_sk])
                            front_class, left_class, right_class = classify_sectors_voted(frame)
                            front_cliff, rear_cliff = cliff.update(raw_fcd, raw_rcd)
                            turn_intensity = compute_turn_intensity(frame)
                            flicker_count = flicker.update(front_class)

                            # [BS] verbose telemetry: sensor snapshot at 2Hz (every 5th nav tick)
                            bs_counter += 1
                            if VERBOSE_TELEMETRY and bs_counter % 5 == 0:
                                t_off = time.monotonic() - brain_start_mono
                                bs_buffer.append(
                                    f"[BS] T+{t_off:.3f} "
                                    f"F:{frame.get('FDL',0)},{frame.get('FCF',0)},{frame.get('FCD',0)},{frame.get('FDR',0)} "
                                    f"R:{frame.get('RDL',0)},{frame.get('RCF',0)},{frame.get('RCD',0)},{frame.get('RDR',0)} "
                                    f"P:{imu['pitch_deg']:+.1f} Ro:{imu['roll_deg']:+.1f} Y:{imu['yaw_deg']:+.1f} "
                                    f"Ac:{imu['accel_mag']:.1f} Gy:{imu['angular_rate']:.2f} U:{imu['upright_quality']:.2f}\n")

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
                            pre_mod_speed = speed  # capture before modifiers for [BN] telemetry
                            speed = nav.apply_modifiers(
                                speed, imu, imu["accel_mag"], voltage,
                                imu["angular_rate"], load_asymmetry,
                                stale, imu["roll_deg"])

                            # [BN] verbose telemetry: nav decisions on state/speed/turn change or 1Hz heartbeat
                            if VERBOSE_TELEMETRY:
                                state_name_bn = NAV_STATE_NAMES.get(state, "?")
                                now_mono = time.monotonic()
                                bn_changed = (state_name_bn != last_bn_state or
                                              abs(speed - last_bn_speed) > 50 or
                                              abs(turn - last_bn_turn) > 0.05 or
                                              now_mono - last_bn_time > 1.0)
                                if bn_changed:
                                    t_off = now_mono - brain_start_mono
                                    bn_buffer.append(
                                        f"[BN] T+{t_off:.3f} St:{state_name_bn} "
                                        f"Sec:{front_class}/{left_class}/{right_class} "
                                        f"Clf:{int(front_cliff)}/{int(rear_cliff)} "
                                        f"Spd:{speed} BSpd:{pre_mod_speed} Trn:{turn:+.3f} TI:{turn_intensity:.2f} "
                                        f"Gait:{nav.terrain_gait} TM:{nav.terrain_mult:.2f} "
                                        f"SM:{nav.stall_speed_mult:.2f} "
                                        f"Imp:{nav.terrain_impact_start}/{nav.terrain_impact_end} "
                                        f"Flk:{flicker_count}\n")
                                    last_bn_state = state_name_bn
                                    last_bn_speed = speed
                                    last_bn_turn = turn
                                    last_bn_time = now_mono

                            # V0.5.01 - Add terminal FSM visualization for better real-time insight into nav behavior during testing
                            # --- Terminal FSM visualization (mirrors --test-nav output) ---
                            state_name_comp = NAV_STATE_NAMES.get(state, "?")
                            gait_name_comp = GAIT_LABELS_COMP.get(nav.terrain_gait, "?")
                            changed_comp = "  <<<" if state_name_comp != prev_state_name_comp else ""
                            elapsed_mission = time.monotonic() - nav.mission_start
                            # Clearance governor: compute Brain-side limit for display
                            cur_duty = GAITS[nav.terrain_gait]['duty']
                            clr_hz = compute_max_clearance_hz(
                                nav.terrain_impact_start, nav.terrain_impact_end, cur_duty,
                                min_clearance=MIN_GROUND_CLEARANCE + GOVERNOR_CLEARANCE_MARGIN)
                            clr_speed = int(clr_hz * 1000)
                            outer_hz = abs(speed / 1000.0) + abs(turn)
                            gov_tag = " GOV" if outer_hz > clr_hz + 0.001 else ""
                            print(f"\n[{nav_tick:03d} T+{elapsed_mission:.1f}s] ─── Competition FSM ─────────────────")
                            print(f"  STATE: {state_name_comp:>10s}{changed_comp}")
                            print(f"  speed={speed:4d}  turn={turn:+.3f}  x_flip={x_flip}")
                            print(f"  gait={gait_name_comp}  terrain_mult={nav.terrain_mult:.2f}  "
                                  f"tripod={nav.terrain_is_tripod}")
                            print(f"  impact=[{nav.terrain_impact_start}°,{nav.terrain_impact_end}°]  "
                                  f"stall_mult={nav.stall_speed_mult:.2f}")
                            print(f"  clearance_gov: max_speed={clr_speed}  "
                                  f"outer_hz={outer_hz:.3f}/{clr_hz:.3f}{gov_tag}")
                            print(f"  sectors: F={SEVERITY_LABELS_COMP[front_class]} "
                                  f"L={SEVERITY_LABELS_COMP[left_class]} "
                                  f"R={SEVERITY_LABELS_COMP[right_class]}  "
                                  f"cliff: F={'YES' if front_cliff else 'no'} "
                                  f"R={'YES' if rear_cliff else 'no'}")
                            print(f"  pitch={imu['pitch_deg']:+5.1f}°  "
                                  f"roll={imu['roll_deg']:+5.1f}°  "
                                  f"upright={imu['upright_quality']:.2f}  "
                                  f"accel={imu['accel_mag']:.1f}  "
                                  f"gyro={imu['angular_rate']:.2f}")
                            print(f"  load_avg={avg_load:.0f}  load_asym={load_asymmetry:.0f}  "
                                  f"voltage={voltage:.1f}V  stale={stale:.2f}s")
                            print(f"  step={step_name}")
                            prev_state_name_comp = state_name_comp
                            nav_tick += 1

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
                                # V0.5.01 - add roll attempt counter and limit to prevent infinite loop if self-right fails
                                if roll_attempts >= MAX_ROLL_ATTEMPTS:
                                    brain_log(f"[NAV] roll limit reached ({MAX_ROLL_ATTEMPTS}) — staying in STOP_SAFE")
                                    set_gait_state(speed=0, turn=0.0, x_flip=1,
                                                   step_name="nav_stop_safe_final")
                                    continue
                                # Severely tilted — try self-right
                                set_gait_state(speed=0, turn=0.0, x_flip=1,
                                               step_name="nav_stop_safe")
                                #brain_log("[NAV] tipover — attempting self-right")
                                # V0.5.01 - add roll attempt counter and limit to prevent infinite loop if self-right fails
                                roll_attempts += 1
                                brain_log(f"[NAV] tipover — attempting self-right ({roll_attempts}/{MAX_ROLL_ATTEMPTS})")
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

                            # --- Flush verbose telemetry buffers (1Hz) ---
                            if VERBOSE_TELEMETRY and time.monotonic() - last_brain_flush > 1.0:
                                brain_flush_buffers()
                                last_brain_flush = time.monotonic()

                            # --- Loop timing ---
                            elapsed = time.monotonic() - loop_start
                            sleep_time = max(0.0, 0.1 - elapsed)
                            if sleep_time > 0:
                                time.sleep(sleep_time)

                        # --- Nav loop exit: flush remaining buffers ---
                        if VERBOSE_TELEMETRY:
                            brain_flush_buffers()

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
            # Ground clearance derivation (measured dimensions):
            #   h(θ) = LEG_EFFECTIVE_RADIUS × cos(θ)  = 125.0 × cos(θ) mm
            #   clearance(θ) = h(θ) − SHAFT_TO_CHASSIS_BOTTOM = h(θ) − 47 mm
            #   At 345/15 (30° sweep): clearance at 15° = 125*cos(15°)-47 = 73.7mm
            #   Air sweep = 330°, governor limits Hz to keep servo within budget
            tripod_impact_start = DEFAULT_IMPACT_START  # 30° sweep
            tripod_impact_end   = DEFAULT_IMPACT_END    # clearance at 15°: 73.7mm
            tripod_duty = GAITS[0]['duty']  # 0.5
            max_hz, max_speed = compute_max_safe_speed(tripod_impact_start, tripod_impact_end, tripod_duty)
            tripod_speed = min(350, max_speed)  # reduced from 450 — less phase lag → more clearance
            print("=== TEST MODE: TRIPOD ===")
            print(f"    Body geometry: leg_radius={LEG_EFFECTIVE_RADIUS}mm, "
                  f"shaft_to_bottom={SHAFT_TO_CHASSIS_BOTTOM}mm")
            print(f"    Impact angles: {tripod_impact_start}°/{tripod_impact_end}° "
                  f"({abs((tripod_impact_end - tripod_impact_start + 180) % 360 - 180)}° sweep), "
                  f"max safe speed={max_speed} (ff not capped up to {max_hz:.2f} Hz)")
            print(f"    Min clearance at stance extremes: "
                  f"{compute_min_clearance(tripod_impact_start, tripod_impact_end):.1f}mm (static), "
                  f"{compute_min_clearance(tripod_impact_start, tripod_impact_end, phase_lag_deg=3.0):.1f}mm (with ~3° lag)")
            set_gait_state(gait=0, impact_start=tripod_impact_start, impact_end=tripod_impact_end, step_name="test_tripod_init")

            set_gait_state(speed=tripod_speed, step_name="forward");                               stall_tsleep(12)
            set_gait_state(turn=-0.15, step_name="carve_left");                                    stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                        stall_tsleep(3)
            set_gait_state(turn=0.15, step_name="carve_right");                                    stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                        stall_tsleep(3)
            set_gait_state(speed=0, turn=-0.35, impact_start=345, impact_end=15, step_name="pivot_left");  stall_tsleep(10)
            set_gait_state(turn=0.0, speed=tripod_speed, impact_start=tripod_impact_start, impact_end=tripod_impact_end, step_name="straight"); stall_tsleep(3)
            set_gait_state(speed=0, turn=0.35, impact_start=345, impact_end=15, step_name="pivot_right");  stall_tsleep(10)
            set_gait_state(turn=0.0, speed=tripod_speed, impact_start=tripod_impact_start, impact_end=tripod_impact_end, step_name="straight"); stall_tsleep(3)
            set_gait_state(speed=0, step_name="decel_pre_reverse"); tsleep(0.5)
            set_gait_state(speed=-tripod_speed, turn=0.0, step_name="reverse");                    stall_tsleep(12)
            set_gait_state(speed=0, step_name="decel"); tsleep(2)

        elif "--test-quad" in sys.argv:
            # === TEST: Quadruped phase only ===
            # Clearance analysis (same framework as test-tripod):
            #   Quad duty=0.70 → air phase is 30% of cycle → moderate ff demand.
            #   345/15 (30° sweep): clearance at 15° = 73.7mm.
            #   Governor dynamically limits Hz to maintain MIN_GROUND_CLEARANCE.
            quad_impact_start = DEFAULT_IMPACT_START  # 30° sweep
            quad_impact_end   = DEFAULT_IMPACT_END    # clearance at 15°: 73.7mm
            quad_duty = GAITS[2]['duty']  # 0.70
            max_hz_q, max_speed_q = compute_max_safe_speed(quad_impact_start, quad_impact_end, quad_duty)
            max_clr_hz_q = compute_max_clearance_hz(quad_impact_start, quad_impact_end, quad_duty,
                                                    min_clearance=MIN_GROUND_CLEARANCE + GOVERNOR_CLEARANCE_MARGIN)
            max_clr_speed_q = int(max_clr_hz_q * 1000)
            quad_speed = min(250, max_clr_speed_q)  # reduced from 300 — less phase lag
            print("=== TEST MODE: QUADRUPED ===")
            print(f"    Body geometry: leg_radius={LEG_EFFECTIVE_RADIUS}mm, "
                  f"shaft_to_bottom={SHAFT_TO_CHASSIS_BOTTOM}mm")
            print(f"    Impact angles: {quad_impact_start}°/{quad_impact_end}° "
                  f"({abs((quad_impact_end - quad_impact_start + 180) % 360 - 180)}° sweep), "
                  f"max safe speed={max_clr_speed_q} (clearance governor)")
            print(f"    Min clearance at stance extremes: "
                  f"{compute_min_clearance(quad_impact_start, quad_impact_end):.1f}mm (static), "
                  f"{compute_min_clearance(quad_impact_start, quad_impact_end, phase_lag_deg=3.0):.1f}mm (with ~3° lag)")
            set_gait_state(gait=2, impact_start=quad_impact_start, impact_end=quad_impact_end, step_name="test_quad_init")

            set_gait_state(speed=quad_speed, turn=0.0, step_name="forward");                          stall_tsleep(12)
            set_gait_state(turn=-0.15, step_name="carve_left");                                stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(turn=0.15, step_name="carve_right");                                stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(speed=0, turn=-0.35, impact_start=345, impact_end=15, step_name="pivot_left");  stall_tsleep(10)
            set_gait_state(turn=0.0, speed=quad_speed, impact_start=quad_impact_start, impact_end=quad_impact_end, step_name="straight"); stall_tsleep(3)
            set_gait_state(speed=0, turn=0.35, impact_start=345, impact_end=15, step_name="pivot_right");  stall_tsleep(10)
            set_gait_state(turn=0.0, speed=quad_speed, impact_start=quad_impact_start, impact_end=quad_impact_end, step_name="straight"); stall_tsleep(3)
            set_gait_state(speed=0, step_name="decel_pre_reverse"); tsleep(0.5)
            set_gait_state(speed=-quad_speed, turn=0.0, step_name="reverse");                  stall_tsleep(12)
            set_gait_state(speed=0, step_name="decel"); tsleep(2)
            # Walking tall: DEFAULT stance (30° sweep) — standard clearance with r=125mm
            wt_clr = int(compute_max_clearance_hz(DEFAULT_IMPACT_START, DEFAULT_IMPACT_END, quad_duty,
                                                  min_clearance=MIN_GROUND_CLEARANCE + GOVERNOR_CLEARANCE_MARGIN) * 1000)
            set_gait_state(speed=min(300, wt_clr), impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END, step_name="walking_tall");   stall_tsleep(15)
            # Stealth crawl: DEFAULT stance (30° sweep) — same geometry as walking_tall
            sc_clr = int(compute_max_clearance_hz(DEFAULT_IMPACT_START, DEFAULT_IMPACT_END, quad_duty,
                                                  min_clearance=MIN_GROUND_CLEARANCE + GOVERNOR_CLEARANCE_MARGIN) * 1000)
            set_gait_state(speed=min(300, sc_clr), impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END, step_name="stealth_crawl");  stall_tsleep(15)

        elif "--test-wave" in sys.argv:
            # === TEST: Wave phase only ===
            # Clearance analysis:
            #   Wave duty=0.75 → air phase is 25% of cycle → higher ff demand.
            #   345/15 (30° sweep): clearance at 15° = 73.7mm.
            #   Governor dynamically limits Hz to maintain MIN_GROUND_CLEARANCE.
            wave_impact_start = DEFAULT_IMPACT_START  # 30° sweep
            wave_impact_end   = DEFAULT_IMPACT_END    # clearance at 15°: 73.7mm
            wave_duty = GAITS[1]['duty']  # 0.75
            max_hz_w, max_speed_w = compute_max_safe_speed(wave_impact_start, wave_impact_end, wave_duty)
            max_clr_hz_w = compute_max_clearance_hz(wave_impact_start, wave_impact_end, wave_duty,
                                                    min_clearance=MIN_GROUND_CLEARANCE + GOVERNOR_CLEARANCE_MARGIN)
            max_clr_speed_w = int(max_clr_hz_w * 1000)
            wave_speed = min(150, max_clr_speed_w)  # reduced from 180 — less phase lag
            print("=== TEST MODE: WAVE ===")
            print(f"    Body geometry: leg_radius={LEG_EFFECTIVE_RADIUS}mm, "
                  f"shaft_to_bottom={SHAFT_TO_CHASSIS_BOTTOM}mm")
            print(f"    Impact angles: {wave_impact_start}°/{wave_impact_end}° "
                  f"({abs((wave_impact_end - wave_impact_start + 180) % 360 - 180)}° sweep), "
                  f"max safe speed={max_clr_speed_w} (clearance governor)")
            print(f"    Min clearance at stance extremes: "
                  f"{compute_min_clearance(wave_impact_start, wave_impact_end):.1f}mm (static), "
                  f"{compute_min_clearance(wave_impact_start, wave_impact_end, phase_lag_deg=3.0):.1f}mm (with ~3° lag)")
            set_gait_state(gait=1, impact_start=wave_impact_start, impact_end=wave_impact_end, step_name="test_wave_init")

            set_gait_state(speed=wave_speed, turn=0.0, step_name="forward");                   stall_tsleep(12)
            set_gait_state(turn=-0.1, step_name="carve_left");                                 stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(turn=0.1, step_name="carve_right");                                 stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(speed=0, turn=-0.25, impact_start=345, impact_end=15, step_name="pivot_left");  stall_tsleep(10)
            set_gait_state(turn=0.0, speed=wave_speed, impact_start=wave_impact_start, impact_end=wave_impact_end, step_name="straight"); stall_tsleep(3)
            set_gait_state(speed=0, turn=0.25, impact_start=345, impact_end=15, step_name="pivot_right");  stall_tsleep(10)
            set_gait_state(turn=0.0, speed=wave_speed, impact_start=wave_impact_start, impact_end=wave_impact_end, step_name="straight"); stall_tsleep(3)
            set_gait_state(speed=0, step_name="decel_pre_reverse"); tsleep(0.5)
            set_gait_state(speed=-wave_speed, turn=0.0, step_name="reverse");                  stall_tsleep(12)
            set_gait_state(speed=0, step_name="decel"); tsleep(2)
            # COG shift hill climb: 340/15 (35° asymmetric sweep, forward bias) — governor limits
            hc_clr = int(compute_max_clearance_hz(330, 15, wave_duty,
                                                  min_clearance=MIN_GROUND_CLEARANCE + GOVERNOR_CLEARANCE_MARGIN) * 1000)
            set_gait_state(speed=min(180, hc_clr), impact_start=340, impact_end=15, step_name="cog_shift_hill_climb"); stall_tsleep(20)

        elif "--test-recovery" in sys.argv:
            # === TEST: Recovery phase only ===
            print("=== TEST MODE: RECOVERY ===")
            set_gait_state(speed=0, turn=0.0, gait=0, step_name="test_recovery_init"); tsleep(3)
            state_recovery_wiggle()
            state_self_right_roll()

        # V0.5.01: Prints raw sensor values and processed outputs side by side for easy comparison.
        elif "--test-sensors" in sys.argv:
            # === TEST: Sensor processing — full 20-column frame + classify/cliff/IMU/turn ===
            # No Heart needed.  Reads live Arduino frames over serial, prints every
            # column the Pi receives, then runs the full sensor-processing pipeline
            # (classify_sectors, CliffDetector, compute_imu, compute_turn_intensity)
            # so you can validate wiring, orientation, and parsing in one shot.
            #
            # Arduino sends CSV at ~5 Hz (self-throttled by ultrasonic measurement
            # time).  We sample every 0.5 s (every ~2–3 Arduino frames) so the
            # terminal stays readable — 30 iterations = ~15 s total.
            print("=== SENSOR PROCESSING TEST ===")
            if not HAS_SERIAL:
                print("[ERROR] pyserial not installed. pip install pyserial")
            else:
                reader = ArduinoReader()
                cliff = CliffDetector()
                SEVERITY_LABELS = {0: "CLEAR", 1: "CAUTION", 2: "NEAR", 3: "DANGER"}
                try:
                    for i in range(30):
                        frame = reader.get_latest()
                        if frame is None:
                            print(f"\n[{i:02d}] no frame yet  "
                                  f"(stale={reader.stale_seconds():.2f}s, "
                                  f"healthy={reader.healthy})")
                            time.sleep(0.5)
                            continue

                        # ── Raw 20-column frame ──────────────────────────────
                        print(f"\n{'='*72}")
                        print(f"  Frame {i:02d}  |  healthy={reader.healthy}  "
                              f"stale={reader.stale_seconds():.2f}s")
                        print(f"{'='*72}")

                        print(f"  {'Timestamp':>12s}: {frame['timestamp_ms']} ms")
                        print(f"  {'--- Distances (cm) ---':^50s}")
                        print(f"  {'FDL':>5s}  {'FCF':>5s}  {'FCD':>5s}  {'FDR':>5s}  "
                              f"{'RDL':>5s}  {'RCF':>5s}  {'RCD':>5s}  {'RDR':>5s}")
                        print(f"  {frame['FDL']:5.1f}  {frame['FCF']:5.1f}  "
                              f"{frame['FCD']:5.1f}  {frame['FDR']:5.1f}  "
                              f"{frame['RDL']:5.1f}  {frame['RCF']:5.1f}  "
                              f"{frame['RCD']:5.1f}  {frame['RDR']:5.1f}")

                        print(f"  {'--- Quaternion ---':^50s}")
                        print(f"  {'qw':>8s}  {'qx':>8s}  {'qy':>8s}  {'qz':>8s}")
                        print(f"  {frame['qw']:8.4f}  {frame['qx']:8.4f}  "
                              f"{frame['qy']:8.4f}  {frame['qz']:8.4f}")

                        print(f"  {'--- Accelerometer (m/s²) ---':^50s}")
                        print(f"  {'ax':>8s}  {'ay':>8s}  {'az':>8s}")
                        print(f"  {frame['ax']:8.4f}  {frame['ay']:8.4f}  "
                              f"{frame['az']:8.4f}")

                        print(f"  {'--- Gyroscope (rad/s) ---':^50s}")
                        print(f"  {'gx':>8s}  {'gy':>8s}  {'gz':>8s}")
                        print(f"  {frame['gx']:8.4f}  {frame['gy']:8.4f}  "
                              f"{frame['gz']:8.4f}")

                        print(f"  {'UpsideDown':>12s}: {frame['upside_down']}")

                        # ── Processed sensor pipeline ────────────────────────
                        front_cls, left_cls, right_cls = classify_sectors_voted(frame)
                        front_cliff, rear_cliff = cliff.update(
                            frame["FCD"], frame["RCD"])
                        imu = compute_imu(frame)
                        turn = compute_turn_intensity(frame)

                        print(f"  {'--- Sensor Pipeline ---':^50s}")
                        print(f"  Sectors  : front={SEVERITY_LABELS[front_cls]}  "
                              f"left={SEVERITY_LABELS[left_cls]}  "
                              f"right={SEVERITY_LABELS[right_cls]}")
                        print(f"  Cliff    : front={'YES' if front_cliff else 'no'}  "
                              f"rear={'YES' if rear_cliff else 'no'}")
                        print(f"  IMU      : pitch={imu['pitch_deg']:+6.1f}°  "
                              f"roll={imu['roll_deg']:+6.1f}°  "
                              f"yaw={imu['yaw_deg']:+6.1f}°  "
                              f"upright={imu['upright_quality']:.2f}")
                        print(f"  Vibration: accel={imu['accel_mag']:.2f} m/s²  "
                              f"gyro={imu['angular_rate']:.2f} rad/s")
                        print(f"  Turn     : intensity={turn:+.3f}")

                        time.sleep(0.5)
                finally:
                    reader.stop()
                print("=== DONE ===")

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

        # V0.5.01: Runs the full NavStateMachine on live Arduino data, 
        # printing every FSM decision and its sensor inputs so you can verify that the robot would react correctly to different terrains and obstacles 
        # — all without needing Heart or actual servo motion.
        elif "--test-nav" in sys.argv:
            # === TEST: Full FSM pipeline visualisation — NO servos, NO Heart needed ===
            # Reads live Arduino frames, runs the complete sensor-processing +
            # NavStateMachine + terrain overlay pipeline, and prints every
            # decision to the terminal so you can verify that state transitions,
            # gait selection, speed, and turn are correct and fully autonomous
            # (driven by sensor data, not hard-coded sequences).
            #
            # Usage on the Pi:
            #   sudo python3 final_full_gait_test.py --test-nav
            #
            # 60 iterations at 0.2 s ≈ 12 s of observation.
            print("=== NAV FSM TEST (no servos) ===")
            if not HAS_SERIAL:
                print("[ERROR] pyserial not installed. pip install pyserial")
            else:
                reader = ArduinoReader()
                nav = NavStateMachine()
                cliff_det = CliffDetector()
                flicker = FlickerTracker()
                SEVERITY_LABELS = {0: "CLEAR", 1: "CAUTION", 2: "NEAR", 3: "DANGER"}
                GAIT_LABELS = {0: "TRIPOD", 1: "WAVE", 2: "QUAD"}
                prev_state_name = "FORWARD"
                try:
                    # Capture initial yaw from first valid frame
                    init_wait = time.monotonic()
                    while reader.stale_seconds() == float("inf"):
                        if time.monotonic() - init_wait > 3.0:
                            break
                        time.sleep(0.1)
                    first_frame = reader.get_latest()
                    if first_frame:
                        first_imu = compute_imu(first_frame)
                        nav.initial_yaw = first_imu["yaw_rad"]
                        print(f"  Initial yaw: {math.degrees(nav.initial_yaw):.1f}°")
                    for i in range(60):
                        frame = reader.get_latest()
                        if frame is None:
                            print(f"[{i:02d}] no frame (stale={reader.stale_seconds():.2f}s)")
                            time.sleep(0.2)
                            continue
                        # --- Sensor processing (identical to competition loop) ---
                        imu = compute_imu(frame)
                        front_cls, left_cls, right_cls = classify_sectors_voted(frame)
                        front_cliff, rear_cliff = cliff_det.update(
                            frame["FCD"], frame["RCD"])
                        turn_intensity = compute_turn_intensity(frame)
                        flicker_count = flicker.update(front_cls)
                        # Simulate zero servo loads and nominal voltage (no Heart)
                        avg_load = 0
                        load_asymmetry = 0
                        voltage = 12.0
                        # --- Terrain overlay ---
                        prev_gait = nav.terrain_gait
                        nav.update_terrain(imu, avg_load, imu["angular_rate"],
                                           imu["accel_mag"], front_cls,
                                           flicker_count, imu["roll_deg"])
                        # --- FSM update ---
                        state, speed, turn, x_flip, step_name = nav.update(
                            frame, imu, front_cls, left_cls, right_cls,
                            front_cliff, rear_cliff, turn_intensity, avg_load,
                            load_asymmetry, imu["angular_rate"], imu["accel_mag"],
                            voltage, flicker_count)
                        # --- Layer 2 modifiers ---
                        stale = reader.stale_seconds()
                        speed = nav.apply_modifiers(
                            speed, imu, imu["accel_mag"], voltage,
                            imu["angular_rate"], load_asymmetry,
                            stale, imu["roll_deg"])
                        state_name = NAV_STATE_NAMES.get(state, "?")
                        gait_name = GAIT_LABELS.get(nav.terrain_gait, "?")
                        changed = "  <<<" if state_name != prev_state_name else ""
                        gait_changed = " GAIT-CHG" if nav.terrain_gait != prev_gait else ""
                        # --- Print decision ---
                        print(f"\n[{i:02d}] ─── FSM Decision ───────────────────────────")
                        print(f"  STATE: {state_name:>10s}{changed}")
                        print(f"  speed={speed:4d}  turn={turn:+.3f}  x_flip={x_flip}")
                        print(f"  gait={gait_name}  terrain_mult={nav.terrain_mult:.2f}  "
                              f"tripod={nav.terrain_is_tripod}{gait_changed}")
                        print(f"  impact_window=[{nav.terrain_impact_start}°,{nav.terrain_impact_end}°]  "
                              f"stall_mult={nav.stall_speed_mult:.2f}")
                        # Clearance governor: compute limit for display
                        cur_duty_tn = GAITS[nav.terrain_gait]['duty']
                        clr_hz_tn = compute_max_clearance_hz(
                            nav.terrain_impact_start, nav.terrain_impact_end, cur_duty_tn,
                            min_clearance=MIN_GROUND_CLEARANCE + GOVERNOR_CLEARANCE_MARGIN)
                        clr_speed_tn = int(clr_hz_tn * 1000)
                        outer_hz_tn = abs(speed / 1000.0) + abs(turn)
                        gov_tag_tn = " GOV" if outer_hz_tn > clr_hz_tn + 0.001 else ""
                        print(f"  clearance_gov: max_speed={clr_speed_tn}  "
                              f"outer_hz={outer_hz_tn:.3f}/{clr_hz_tn:.3f}{gov_tag_tn}")
                        print(f"  ── Sensor inputs ──")
                        print(f"  sectors: F={SEVERITY_LABELS[front_cls]} "
                              f"L={SEVERITY_LABELS[left_cls]} "
                              f"R={SEVERITY_LABELS[right_cls]}  "
                              f"cliff: F={'YES' if front_cliff else 'no'} "
                              f"R={'YES' if rear_cliff else 'no'}")
                        print(f"  turn_intensity={turn_intensity:+.3f}  "
                              f"flicker={flicker_count}")
                        print(f"  pitch={imu['pitch_deg']:+5.1f}°  "
                              f"roll={imu['roll_deg']:+5.1f}°  "
                              f"upright={imu['upright_quality']:.2f}  "
                              f"accel={imu['accel_mag']:.1f}  "
                              f"gyro={imu['angular_rate']:.2f}")
                        print(f"  FCF={frame['FCF']:.0f}  FDL={frame['FDL']:.0f}  "
                              f"FDR={frame['FDR']:.0f}  FCD={frame['FCD']:.0f}  "
                              f"RCF={frame['RCF']:.0f}")
                        print(f"  step_name={step_name}")
                        prev_state_name = state_name
                        time.sleep(0.2)
                finally:
                    reader.stop()
                print("=== DONE ===")

        elif "--test-competition" in sys.argv:
            # === TEST: Competition sequence only ===
            print("=== TEST MODE: COMPETITION ===")

            # Phase 1: Quadruped forward (best stability + speed balance on sand)
            set_gait_state(gait=2, impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END, step_name="comp_quad_init")
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
            set_gait_state(gait=0, impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END, step_name="phase1_init")

            set_gait_state(speed=900, step_name="forward");                                    stall_tsleep(12)
            set_gait_state(turn=-0.2, step_name="carve_left");                                 stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(turn=0.2, step_name="carve_right");                                 stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(speed=0, turn=-0.35, impact_start=345, impact_end=15, step_name="pivot_left");  stall_tsleep(10)
            set_gait_state(turn=0.0, speed=900, impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END, step_name="straight"); stall_tsleep(3)
            set_gait_state(speed=0, turn=0.35, impact_start=345, impact_end=15, step_name="pivot_right");  stall_tsleep(10)
            set_gait_state(turn=0.0, speed=900, impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END, step_name="straight"); stall_tsleep(3)
            set_gait_state(speed=0, step_name="decel_pre_reverse_p1"); tsleep(0.5)
            set_gait_state(speed=-900, turn=0.0, step_name="reverse");                         stall_tsleep(12)

            # Decel pause - smooth_hz bleeds ~720ms from -1.2Hz without this,
            # meaning the robot reverses briefly after Phase 2 starts.
            set_gait_state(speed=0, step_name="decel_p1"); tsleep(2)

            # =========================================================
            # PHASE 2: QUADRUPED
            # =========================================================
            print("\n-- phase 2: quadruped --")
            set_gait_state(gait=2, impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END, step_name="phase2_init")

            set_gait_state(speed=900, turn=0.0, step_name="forward");                          stall_tsleep(12)
            set_gait_state(turn=-0.15, step_name="carve_left");                                stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(turn=0.15, step_name="carve_right");                                stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(speed=0, turn=-0.35, impact_start=345, impact_end=15, step_name="pivot_left");  stall_tsleep(10)
            set_gait_state(turn=0.0, speed=900, impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END, step_name="straight"); stall_tsleep(3)
            set_gait_state(speed=0, turn=0.35, impact_start=345, impact_end=15, step_name="pivot_right");  stall_tsleep(10)
            set_gait_state(turn=0.0, speed=900, impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END, step_name="straight"); stall_tsleep(3)
            set_gait_state(speed=0, step_name="decel_pre_reverse_p2"); tsleep(0.5)
            set_gait_state(speed=-900, turn=0.0, step_name="reverse");                         stall_tsleep(12)

            # Decel before walking tall - avoids ~720ms of backward motion bleed into forward mode
            set_gait_state(speed=0, step_name="decel_p2"); tsleep(2)

            set_gait_state(speed=300, impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END, step_name="walking_tall");   stall_tsleep(15)
            set_gait_state(impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END, step_name="stealth_crawl");              stall_tsleep(15)

            # =========================================================
            # PHASE 3: WAVE
            # =========================================================
            print("\n-- phase 3: wave --")
            set_gait_state(gait=1, impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END, step_name="phase3_init")

            set_gait_state(speed=900, turn=0.0, step_name="forward");                          stall_tsleep(12)
            set_gait_state(turn=-0.1, step_name="carve_left");                                 stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(turn=0.1, step_name="carve_right");                                 stall_tsleep(10)
            set_gait_state(turn=0.0, step_name="straight");                                    stall_tsleep(3)
            set_gait_state(speed=0, turn=-0.25, impact_start=345, impact_end=15, step_name="pivot_left");  stall_tsleep(10)
            set_gait_state(turn=0.0, speed=900, impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END, step_name="straight"); stall_tsleep(3)
            set_gait_state(speed=0, turn=0.25, impact_start=345, impact_end=15, step_name="pivot_right");  stall_tsleep(10)
            set_gait_state(turn=0.0, speed=900, impact_start=DEFAULT_IMPACT_START, impact_end=DEFAULT_IMPACT_END, step_name="straight"); stall_tsleep(3)
            set_gait_state(speed=0, step_name="decel_pre_reverse_p3"); tsleep(0.5)
            set_gait_state(speed=-900, turn=0.0, step_name="reverse");                         stall_tsleep(12)

            # Decel pause - matches Phase 1/2 pattern, bleeds ~720ms of reverse motion before COG shift
            set_gait_state(speed=0, step_name="decel_p3"); tsleep(2)

            set_gait_state(speed=180, impact_start=340, impact_end=15, step_name="cog_shift_hill_climb"); stall_tsleep(20)

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