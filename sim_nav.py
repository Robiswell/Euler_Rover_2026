#!/usr/bin/env python3
"""
sim_nav.py — Brain-level autonomous navigation test suite

Tests the 8-state FSM, serial parser, sensor processing, cliff detection,
terrain overlay, and graceful degradation using synthetic sensor frames.
No hardware dependencies (no serial, no GPIO, no scservo_sdk).

Usage:
    python sim_nav.py

Output: Named checks (N1-N11), PASS/FAIL per check, diagnostics on failure,
overall summary at bottom.
"""

import sys
import io
import math
import time

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

# =========================================================================
#  REPLICATED NAV CODE
#  Replicated from final_full_gait_test.py -- keep in sync
# =========================================================================

# --- Nav tunable constants ---
CRUISE_SPEED = 450
TRIPOD_CRUISE_SPEED = 600
SLOW_SPEED = 200
BACKWARD_SPEED = 300
BACKWARD_MIN_DWELL = 0.8          # seconds in BACKWARD before allowing pivot escalation
CLIFF_BACKUP_DURATION = 5.0       # seconds of forced backward on front cliff before escape
MAX_TURN_BIAS = 0.25              # restored -- safe with r=125mm clearance headroom
PIVOT_TURN_BIAS = 0.35            # restored -- safe with r=125mm roll-aware clearance governor
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

# Distance classification constants
DIST_DANGER = 3
DIST_NEAR = 2
DIST_CAUTION = 1
DIST_CLEAR = 0
DIST_UNKNOWN = 1  # treat unknown same as caution

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

NAV_IMU_SETTLE_TICKS = 5  # ignore IMU safety checks for first 0.5s (BNO085 convergence)
CLIFF_WARMUP = 5  # frames before cliff detection active (sensor settle)
RAPID_ROTATION_THRESHOLD = 3.5  # rad/s (~200 deg/s) -- walking oscillation peaks ~2.0

# Suppress brain_log in test context
def brain_log(msg):
    pass


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


def classify_distance(cm):
    """Classify a single distance reading into severity level."""
    if cm is None:
        return DIST_UNKNOWN
    if cm == -1:
        return DIST_DANGER
    if cm < 0:
        return DIST_UNKNOWN
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


class CliffDetector:
    """Detects cliffs using ground EMA + delta + consecutive frame confirmation."""

    def __init__(self, alpha=0.2):
        self._alpha = alpha
        self._ground_ema = 25.0
        self._consecutive_front = 0
        self._consecutive_rear = 0
        self._warmup_frames = 0

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
            setattr(self, counter_attr, 0)
            return False

        # 300.0 = max range timeout (acoustic scatter on sand) -- treat as blind zone
        if reading >= 300.0:
            setattr(self, counter_attr, 0)
            return False

        if reading is None:
            setattr(self, counter_attr, count + 1)
            return count + 1 >= 2

        # Update ground EMA with valid low readings (runs during warmup so baseline converges)
        if 0 < reading <= 40:
            self._ground_ema = self._alpha * reading + (1 - self._alpha) * self._ground_ema

        # Warmup: suppress detection but EMA already updated above
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


def compute_imu(frame):
    """Extract orientation, vibration, angular rate from frame."""
    w, x, y, z = frame["qw"], frame["qx"], frame["qy"], frame["qz"]

    qmag = w*w + x*x + y*y + z*z
    if qmag < 0.5:
        return {
            "pitch_deg": 0.0, "roll_deg": 0.0, "yaw_deg": 0.0,
            "pitch_rad": 0.0, "roll_rad": 0.0, "yaw_rad": 0.0,
            "upright_quality": 0.0,
            "accel_mag": 9.81, "angular_rate": 0.0,
        }

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch_rad = math.asin(sinp)

    roll_rad = math.atan2(2.0 * (w * x + y * z),
                          1.0 - 2.0 * (x * x + y * y))

    yaw_rad = math.atan2(2.0 * (w * z + x * y),
                         1.0 - 2.0 * (y * y + z * z))

    upright_quality = min(math.cos(abs(pitch_rad)),
                          math.cos(abs(roll_rad)))

    accel_mag = math.sqrt(frame["ax"]**2 + frame["ay"]**2 + frame["az"]**2)
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
    fdl_eff = 25.0 if (fdl is None or fdl == -1) else fdl
    fdr_eff = 25.0 if (fdr is None or fdr == -1) else fdr
    return math.tanh((fdl_eff - fdr_eff) / 50.0)


def compute_battery_mult(voltage_value):
    """Compute battery speed multiplier from voltage."""
    VOLTAGE_MIN = 10.5
    if voltage_value < VOLTAGE_MIN:
        return 0.0
    if voltage_value < VOLTAGE_MIN + 0.5:
        return 0.7
    return 1.0


def compute_servo_loads(loads_snapshot):
    """Analyze servo load array snapshot."""
    if not loads_snapshot or all(v < 0 for v in loads_snapshot):
        return {"avg_load": 0, "load_asymmetry": 0, "left_avg": 0, "right_avg": 0}
    valid = [v for v in loads_snapshot if v >= 0]
    if not valid:
        return {"avg_load": 0, "load_asymmetry": 0, "left_avg": 0, "right_avg": 0}
    avg_load = sum(valid) / len(valid)
    load_asymmetry = max(valid) - min(valid)
    left_indices = [0, 1, 2]
    right_indices = [3, 4, 5]
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
        self._history = []
        self._prev_class = None

    def update(self, front_class):
        """Record new front classification. Returns flicker count."""
        now = time.monotonic()
        self._history.append((now, front_class))
        cutoff = now - self._window_s
        self._history = [(t, c) for t, c in self._history if t >= cutoff]
        transitions = 0
        for i in range(1, len(self._history)):
            if self._history[i][1] != self._history[i-1][1]:
                transitions += 1
        self._prev_class = front_class
        return transitions


def is_rear_safe(frame):
    """Check if rear is clear enough for reversing."""
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
        self.backward_entry_time = 0.0
        self.cliff_backup_until = 0.0   # monotonic deadline for cliff backup lockout
        self.consecutive_pivot_count = 0
        self.pivot_direction = -1
        self.stall_start_time = 0.0
        self.stall_count_30s = 0
        self.last_stall_time = 0.0
        self.stall_speed_mult = 1.0
        self.mission_start = time.monotonic()
        self.initial_yaw = None
        self.finished = False
        self.finish_wall_start = 0.0
        self.front_danger_frames = 0
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
        self.terrain_gait = 2
        self.terrain_impact_start = 340     # Fix 75: matches production default (belly clearance)
        self.terrain_impact_end = 20
        self.terrain_mult = 1.0
        self.terrain_is_tripod = False
        self._gait_transition_until = 0.0
        self._tick_count = 0
        self.sensor_ema = {}

    def _dwell_active(self):
        return (time.monotonic() - self.dwell_start) < self.dwell_duration

    def _start_dwell(self, duration):
        self.dwell_start = time.monotonic()
        self.dwell_duration = duration

    def _refresh_dwell(self, duration):
        self.dwell_start = time.monotonic()
        self.dwell_duration = duration

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

    def _transition(self, new_state):
        if new_state != self.state:
            self.prev_state = self.state
            self.state = new_state
            self.dwell_duration = 0
            brain_log(f"[NAV] {NAV_STATE_NAMES.get(self.prev_state)}->{NAV_STATE_NAMES.get(new_state)}")

    def update(self, frame, imu, front_class, left_class, right_class,
               front_cliff, rear_cliff, turn_intensity, avg_load,
               load_asymmetry, angular_rate, accel_mag, voltage,
               flicker_count):
        """Evaluate priorities and return (state, speed, turn, x_flip, step_name)."""
        now = time.monotonic()
        elapsed = now - self.mission_start

        # --- Mission timeout ---
        if elapsed >= MISSION_TIMEOUT_S:
            self.finished = True
            return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_timeout")

        # --- Finish wall detection ---
        fcf = frame["FCF"] if frame else 999
        rcf_val = frame["RCF"] if frame else 999
        if (fcf != -1 and fcf is not None and 0 < fcf < FINISH_WALL_DIST_CM
                and self.state in (NAV_FORWARD, NAV_SLOW_FORWARD)):
            if self.finish_wall_start == 0.0:
                self.finish_wall_start = now
            elif (now - self.finish_wall_start) >= FINISH_WALL_SUSTAIN_S:
                if rcf_val is not None and rcf_val != -1 and rcf_val > 50:
                    self.finished = True
                    brain_log("[NAV] finish wall detected")
                    return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_finish")
                else:
                    self.finish_wall_start = 0.0
        else:
            self.finish_wall_start = 0.0

        upright = imu["upright_quality"]
        pitch_deg = imu["pitch_deg"]
        roll_deg = imu["roll_deg"]

        # --- Track front danger frames ---
        if front_class >= DIST_DANGER:
            self.front_danger_frames += 1
        else:
            self.front_danger_frames = 0

        self._tick_count += 1

        if self._tick_count >= NAV_IMU_SETTLE_TICKS:
            # P1: Tipover
            if upright < 0.15:
                self._transition(NAV_STOP_SAFE)
                return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_stop_safe")

            # P2: Unexpected rapid rotation
            if angular_rate > RAPID_ROTATION_THRESHOLD and self.state not in (NAV_PIVOT_TURN, NAV_ARC_LEFT, NAV_ARC_RIGHT, NAV_BACKWARD, NAV_WIGGLE):
                self._transition(NAV_STOP_SAFE)
                return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_stop_safe")

        # P3: Critical battery
        if voltage < 10.5:
            self._transition(NAV_STOP_SAFE)
            return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_stop_safe")

        # P5: Freefall
        if accel_mag < 8.0 and upright < 0.5:
            if self._freefall_start == 0.0:
                self._freefall_start = now
            elif (now - self._freefall_start) >= 0.5:
                self._transition(NAV_STOP_SAFE)
                return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_stop_safe")
        else:
            self._freefall_start = 0.0

        # P6: Front cliff
        if front_cliff:
            if self.state != NAV_BACKWARD:
                self.hold_position_count = 0
                self.backward_entry_time = time.monotonic()
                self.cliff_backup_until = now + CLIFF_BACKUP_DURATION
                self._start_dwell(0.8)  # Fix 68: only on first transition
            self._transition(NAV_BACKWARD)
            return self._backward_action(frame)

        # P7: Rear cliff (overrides cliff lockout -- don't back into a drop-off)
        if rear_cliff:
            self.cliff_backup_until = 0.0  # clear lockout on rear cliff
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

        # P9: Dead end
        if (front_class >= DIST_DANGER and left_class >= DIST_DANGER
                and right_class >= DIST_DANGER
                and self.prev_state == NAV_BACKWARD):
            if self.consecutive_pivot_count >= 2:
                self._transition(NAV_STOP_SAFE)
                return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_stop_safe")
            self._pick_pivot_direction(frame)
            self._transition(NAV_PIVOT_TURN)
            self._start_dwell(1.5)
            self.consecutive_pivot_count += 1
            turn = self.pivot_direction * PIVOT_TURN_BIAS
            step = "nav_pivot_L" if self.pivot_direction < 0 else "nav_pivot_R"
            return (NAV_PIVOT_TURN, 0, turn, 1, step)

        # P10: Front DANGER (2+ frames) -- escape-route awareness
        if self.front_danger_frames >= 2:
            # Prefer arc escape over backward when a side is clear
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
                step = "nav_escape_L" if escape_dir < 0 else "nav_escape_R"
                state = NAV_ARC_LEFT if escape_dir < 0 else NAV_ARC_RIGHT
                return (state, speed, turn, 1, step)
            else:
                if self.state != NAV_BACKWARD:
                    self.hold_position_count = 0
                    self.backward_entry_time = time.monotonic()
                    self._start_dwell(0.8)
                self._transition(NAV_BACKWARD)
                return self._backward_action(frame)

        # Reset pivot count when front clears
        if front_class < DIST_DANGER:
            self.consecutive_pivot_count = 0

        # P11: Lateral obstacle
        # Arc dwell hold: if already arcing with active dwell, hold the arc.
        # If obstacle persists, refresh the dwell. If cleared, let dwell expire naturally.
        if self.state in (NAV_ARC_LEFT, NAV_ARC_RIGHT) and self._dwell_active():
            if self.state == NAV_ARC_LEFT and left_class >= DIST_NEAR:
                self._refresh_dwell(0.4)
            elif self.state == NAV_ARC_RIGHT and right_class >= DIST_NEAR:
                self._refresh_dwell(0.4)
            turn = (-1 if self.state == NAV_ARC_LEFT else 1) * abs(turn_intensity) * MAX_TURN_BIAS
            speed = int(SLOW_SPEED * self.terrain_mult * self.stall_speed_mult)
            step = "nav_arc_L_hold" if self.state == NAV_ARC_LEFT else "nav_arc_R_hold"
            return (self.state, speed, turn, 1, step)

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

        # P12: Narrow corridor
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

        # FORWARD with heading correction
        base_speed = CRUISE_SPEED
        if self.terrain_is_tripod:
            base_speed = TRIPOD_CRUISE_SPEED
        speed_s = speed_scale_from_front(front_class)
        speed = int(base_speed * speed_s * self.terrain_mult * self.stall_speed_mult)

        turn = 0.0
        if self.initial_yaw is not None:
            yaw_error = math.atan2(
                math.sin(imu["yaw_rad"] - self.initial_yaw),
                math.cos(imu["yaw_rad"] - self.initial_yaw))
            if abs(yaw_error) > math.radians(30):
                if yaw_error > 0:
                    turn = -HEADING_CORRECTION_BIAS
                else:
                    turn = HEADING_CORRECTION_BIAS

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
                self._pick_pivot_direction(frame)
                self._transition(NAV_PIVOT_TURN)
                self.cliff_backup_until = 0.0  # clear lockout so pivot can complete
                self._start_dwell(1.5)
                self.consecutive_pivot_count += 1
                turn = self.pivot_direction * PIVOT_TURN_BIAS
                step = "nav_pivot_L" if self.pivot_direction < 0 else "nav_pivot_R"
                return (NAV_PIVOT_TURN, 0, turn, 1, step)
            return (NAV_BACKWARD, 0, 0.0, 1, "nav_backward")

    def _pick_pivot_direction(self, frame):
        """Pick pivot direction toward freer rear diagonal."""
        if frame is None:
            self.pivot_direction = -1
            return
        rdl = frame["RDL"]
        rdr = frame["RDR"]
        rdl_eff = 0 if (rdl is None or rdl == -1) else rdl
        rdr_eff = 0 if (rdr is None or rdr == -1) else rdr
        if rdl_eff >= rdr_eff:
            self.pivot_direction = -1
        else:
            self.pivot_direction = 1

    def update_terrain(self, imu, avg_load, angular_rate, accel_mag,
                       front_class, flicker_count, roll_deg):
        """Unified Terrain Decision Table."""
        now = time.monotonic()
        pitch_deg = imu["pitch_deg"]
        upright = imu["upright_quality"]

        if self.state not in (NAV_FORWARD, NAV_SLOW_FORWARD):
            return

        prev_gait = self.terrain_gait

        # T1: Steep climb
        if pitch_deg > 15:
            if self._steep_up_start == 0.0:
                self._steep_up_start = now
            if (now - self._steep_up_start) >= 1.0:
                self.terrain_gait = 1
                self.terrain_impact_start = 325
                self.terrain_impact_end = 20
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
                self.terrain_gait = 1
                self.terrain_impact_start = 325
                self.terrain_impact_end = 35
                self.terrain_mult = 0.5
                self.terrain_is_tripod = False
                self._apply_gait_transition(prev_gait)
                return
        else:
            self._steep_down_start = 0.0

        # T3: Moderate slope or tilted
        if abs(pitch_deg) > SLOPE_PITCH_DEG or (0.15 <= upright <= 0.5):
            self.terrain_gait = 1
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
                self.terrain_gait = 1
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
            return

        # T6: Rocky terrain (flicker + moderate load)
        if flicker_count >= FLICKER_COUNT_THRESHOLD and avg_load > 300:
            self.terrain_impact_start = 345
            self.terrain_impact_end = 15
            self.terrain_mult = 0.8
            self.terrain_is_tripod = False
            return

        # T7: High vibration
        if accel_mag > 14:
            if self._high_vibe_start == 0.0:
                self._high_vibe_start = now
            if (now - self._high_vibe_start) >= 0.5:
                self.terrain_impact_start = 345
                self.terrain_impact_end = 15
                self.terrain_mult = 0.8
                self.terrain_is_tripod = False
                return
        else:
            self._high_vibe_start = 0.0

        # T8: Hard flat ground
        no_recent_stalls = (now - self._last_stall_clear_time) > 30 or self.stall_count_30s == 0
        if (avg_load < LIGHT_TERRAIN_LOAD
                and front_class == DIST_CLEAR
                and abs(pitch_deg) < 5
                and abs(roll_deg) < 5
                and no_recent_stalls):
            if self._light_load_start == 0.0:
                self._light_load_start = now
            if (now - self._light_load_start) >= TERRAIN_SUSTAIN_S:
                self.terrain_gait = 0
                self.terrain_impact_start = 340
                self.terrain_impact_end = 20
                self.terrain_mult = 1.0
                self.terrain_is_tripod = True
                self._apply_gait_transition(prev_gait)
                return
        else:
            self._light_load_start = 0.0
            if self.terrain_is_tripod:
                self.terrain_gait = 2
                self.terrain_is_tripod = False
                self._apply_gait_transition(0)

        # T9: Default quad
        self.terrain_gait = 2
        self.terrain_impact_start = 340
        self.terrain_impact_end = 20
        self.terrain_mult = 1.0
        self.terrain_is_tripod = False
        self._apply_gait_transition(prev_gait)

    def _apply_gait_transition(self, prev_gait):
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

        # M3: Brief data gap
        if 0.3 < stale_seconds < 5.0:
            speed = int(speed * 0.5)

        # M4: Lateral slope
        if abs(roll_deg) > 10:
            if self._roll_sustained_start == 0.0:
                self._roll_sustained_start = now
            if (now - self._roll_sustained_start) >= 0.5:
                speed = int(speed * 0.7)
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


# =========================================================================
#  TEST HELPERS
# =========================================================================

def make_csv_line(ts=1000, fdl=100, fcf=100, fcd=15, fdr=100,
                  rdl=100, rcf=100, rcd=15, rdr=100,
                  qw=1.0, qx=0.0, qy=0.0, qz=0.0,
                  ax=0.0, ay=0.0, az=9.81,
                  gx=0.0, gy=0.0, gz=0.0, upside=0):
    """Build a 20-column CSV line string."""
    return (f"{ts},{fdl},{fcf},{fcd},{fdr},"
            f"{rdl},{rcf},{rcd},{rdr},"
            f"{qw},{qx},{qy},{qz},"
            f"{ax},{ay},{az},"
            f"{gx},{gy},{gz},{upside}")


def make_frame(fdl=100, fcf=100, fcd=15, fdr=100,
               rdl=100, rcf=100, rcd=15, rdr=100,
               qw=1.0, qx=0.0, qy=0.0, qz=0.0,
               ax=0.0, ay=0.0, az=9.81,
               gx=0.0, gy=0.0, gz=0.0, upside=0,
               ts=1000):
    """Build a frame dict directly (bypassing CSV parsing)."""
    return {
        "timestamp_ms": ts,
        "FDL": fdl, "FCF": fcf, "FCD": fcd, "FDR": fdr,
        "RDL": rdl, "RCF": rcf, "RCD": rcd, "RDR": rdr,
        "qw": qw, "qx": qx, "qy": qy, "qz": qz,
        "ax": ax, "ay": ay, "az": az,
        "gx": gx, "gy": gy, "gz": gz,
        "upside_down": upside,
    }


def make_imu_level():
    """IMU dict for a level, stable robot."""
    return {
        "pitch_deg": 0.0, "roll_deg": 0.0, "yaw_deg": 0.0,
        "pitch_rad": 0.0, "roll_rad": 0.0, "yaw_rad": 0.0,
        "upright_quality": 1.0,
        "accel_mag": 9.81, "angular_rate": 0.0,
    }


def make_imu(pitch_deg=0.0, roll_deg=0.0, yaw_deg=0.0,
             upright_quality=None, accel_mag=9.81, angular_rate=0.0):
    """Build an IMU dict with specified values. Auto-computes upright_quality if not given."""
    pr = math.radians(pitch_deg)
    rr = math.radians(roll_deg)
    yr = math.radians(yaw_deg)
    if upright_quality is None:
        upright_quality = min(math.cos(abs(pr)), math.cos(abs(rr)))
    return {
        "pitch_deg": pitch_deg, "roll_deg": roll_deg, "yaw_deg": yaw_deg,
        "pitch_rad": pr, "roll_rad": rr, "yaw_rad": yr,
        "upright_quality": upright_quality,
        "accel_mag": accel_mag, "angular_rate": angular_rate,
    }


def fresh_nav():
    """Create a fresh NavStateMachine for testing."""
    return NavStateMachine()


def fsm_update_simple(nav, frame=None, imu=None, front_cliff=False, rear_cliff=False,
                      avg_load=100, load_asymmetry=50, voltage=12.0, flicker_count=0):
    """Convenience wrapper: run one FSM update with sensible defaults."""
    if frame is None:
        frame = make_frame()
    if imu is None:
        imu = make_imu_level()
    front_class, left_class, right_class = classify_sectors(frame)
    turn_intensity = compute_turn_intensity(frame)
    return nav.update(frame, imu, front_class, left_class, right_class,
                      front_cliff, rear_cliff, turn_intensity, avg_load,
                      load_asymmetry, imu["angular_rate"], imu["accel_mag"],
                      voltage, flicker_count)


# =========================================================================
#  RESULT TRACKING
# =========================================================================

_results = {}   # check_id -> (passed: bool, details: str)
_diags = {}     # check_id -> list of diagnostic strings


def check(check_id, condition, detail=""):
    """Record a sub-check result."""
    if check_id not in _results:
        _results[check_id] = True
        _diags[check_id] = []
    if not condition:
        _results[check_id] = False
        if detail:
            _diags[check_id].append(detail)


def pf(check_id):
    return "PASS" if _results.get(check_id, True) else "FAIL"


# =========================================================================
#  N1: SERIAL PARSER ROBUSTNESS
# =========================================================================

def run_n1():
    cid = "N1"

    # Valid 20-column CSV
    line = make_csv_line()
    f = _parse_arduino_csv(line)
    check(cid, f is not None, "valid CSV returned None")
    if f:
        check(cid, f["timestamp_ms"] == 1000, f"timestamp={f['timestamp_ms']} expected 1000")
        check(cid, f["FDL"] == 100, f"FDL={f['FDL']} expected 100")
        check(cid, f["FCF"] == 100, f"FCF={f['FCF']} expected 100")
        check(cid, f["FCD"] == 15, f"FCD={f['FCD']} expected 15")
        check(cid, f["FDR"] == 100, f"FDR={f['FDR']} expected 100")
        check(cid, f["RDL"] == 100, f"RDL={f['RDL']} expected 100")
        check(cid, f["RCF"] == 100, f"RCF={f['RCF']} expected 100")
        check(cid, f["RCD"] == 15, f"RCD={f['RCD']} expected 15")
        check(cid, f["RDR"] == 100, f"RDR={f['RDR']} expected 100")
        check(cid, abs(f["qw"] - 1.0) < 1e-9, f"qw={f['qw']} expected 1.0")
        check(cid, abs(f["az"] - 9.81) < 1e-6, f"az={f['az']} expected 9.81")

    # Short line (< 20 columns) -> rejected
    short = "1000,50,50,15,50,80,80,15,80"
    check(cid, _parse_arduino_csv(short) is None, "short line not rejected")

    # Non-numeric field -> rejected
    bad_num = make_csv_line().replace("100", "abc", 1)
    check(cid, _parse_arduino_csv(bad_num) is None, "non-numeric field not rejected")

    # Empty line / whitespace only -> rejected
    check(cid, _parse_arduino_csv("") is None, "empty string not rejected")
    check(cid, _parse_arduino_csv("   ") is None, "whitespace not rejected")
    check(cid, _parse_arduino_csv(None) is None, "None not rejected")

    # Very long line (> 500 chars) -> either rejected or parsed (no crash)
    long_line = "1," * 500 + "1"
    try:
        result = _parse_arduino_csv(long_line)
        # It should reject (column count != 20), but no crash is the real test
        check(cid, result is None, "long line should be rejected (column count mismatch)")
    except Exception as e:
        check(cid, False, f"long line caused crash: {e}")

    # Header line rejection
    check(cid, _parse_arduino_csv("timestamp_ms,FDL,FCF,...") is None,
          "header line not rejected")

    # Column mapping: verify CSV column 0=timestamp, 1=FDL, 2=FCF, 3=FCD, 4=FDR
    line2 = "5000,11,22,33,44,55,66,77,88,1.0,0.0,0.0,0.0,0.0,0.0,9.81,0.0,0.0,0.0,0"
    f2 = _parse_arduino_csv(line2)
    check(cid, f2 is not None, "mapping test CSV returned None")
    if f2:
        check(cid, f2["timestamp_ms"] == 5000, f"ts mapping: {f2['timestamp_ms']} != 5000")
        check(cid, f2["FDL"] == 11, f"FDL mapping: {f2['FDL']} != 11")
        check(cid, f2["FCF"] == 22, f"FCF mapping: {f2['FCF']} != 22")
        check(cid, f2["FCD"] == 33, f"FCD mapping: {f2['FCD']} != 33")
        check(cid, f2["FDR"] == 44, f"FDR mapping: {f2['FDR']} != 44")
        check(cid, f2["RDL"] == 55, f"RDL mapping: {f2['RDL']} != 55")
        check(cid, f2["RCF"] == 66, f"RCF mapping: {f2['RCF']} != 66")
        check(cid, f2["RCD"] == 77, f"RCD mapping: {f2['RCD']} != 77")
        check(cid, f2["RDR"] == 88, f"RDR mapping: {f2['RDR']} != 88")

    # Distance clamping: >450 capped, -1 preserved
    line3 = make_csv_line(fdl=600, fcf=-1)
    f3 = _parse_arduino_csv(line3)
    check(cid, f3 is not None, "clamping test CSV returned None")
    if f3:
        check(cid, f3["FDL"] == 450.0, f"FDL clamping: {f3['FDL']} != 450")
        check(cid, f3["FCF"] == -1, f"FCF sentinel: {f3['FCF']} != -1")

    # Trailing whitespace / newline handling
    line4 = make_csv_line() + "\r\n"
    f4 = _parse_arduino_csv(line4)
    check(cid, f4 is not None, "trailing newline caused rejection")


# =========================================================================
#  N2: SENSOR CLASSIFICATION
# =========================================================================

def run_n2():
    cid = "N2"

    # Single-sensor classification thresholds
    check(cid, classify_distance(-1) == DIST_DANGER, "-1 should be DANGER")
    check(cid, classify_distance(0) == DIST_DANGER, "0 should be DANGER")
    check(cid, classify_distance(10) == DIST_DANGER, "10 should be DANGER")
    check(cid, classify_distance(20) == DIST_DANGER, "20 should be DANGER")
    check(cid, classify_distance(20.1) == DIST_NEAR, "20.1 should be NEAR")
    check(cid, classify_distance(25) == DIST_NEAR, "25 should be NEAR")
    check(cid, classify_distance(30) == DIST_NEAR, "30 should be NEAR")
    check(cid, classify_distance(30.1) == DIST_CAUTION, "30.1 should be CAUTION")
    check(cid, classify_distance(40) == DIST_CAUTION, "40 should be CAUTION")
    check(cid, classify_distance(50) == DIST_CAUTION, "50 should be CAUTION")
    check(cid, classify_distance(50.1) == DIST_CLEAR, "50.1 should be CLEAR")
    check(cid, classify_distance(100) == DIST_CLEAR, "100 should be CLEAR")
    check(cid, classify_distance(None) == DIST_UNKNOWN, "None should be UNKNOWN")
    check(cid, classify_distance(-5) == DIST_UNKNOWN, "-5 should be UNKNOWN")

    # Severity ordering
    check(cid, DIST_DANGER > DIST_NEAR > DIST_CAUTION > DIST_CLEAR,
          f"severity ordering wrong: D={DIST_DANGER} N={DIST_NEAR} C={DIST_CAUTION} CL={DIST_CLEAR}")

    # Sector classification: front = worst of (FDL, FCF, FDR)
    f1 = make_frame(fdl=100, fcf=100, fdr=15)  # fdr=15 -> DANGER
    fc, lc, rc = classify_sectors(f1)
    check(cid, fc == DIST_DANGER, f"front should be DANGER (one sensor 15cm): got {fc}")
    check(cid, rc == DIST_DANGER, f"right should include FDR=15: got {rc}")

    # Left = worst of (FDL, RDL)
    f2 = make_frame(fdl=25, rdl=100)  # fdl=25 -> NEAR
    fc2, lc2, rc2 = classify_sectors(f2)
    check(cid, lc2 == DIST_NEAR, f"left should be NEAR (FDL=25): got {lc2}")

    # Right = worst of (FDR, RDR)
    f3 = make_frame(fdr=100, rdr=25)  # rdr=25 -> NEAR
    fc3, lc3, rc3 = classify_sectors(f3)
    check(cid, rc3 == DIST_NEAR, f"right should be NEAR (RDR=25): got {rc3}")

    # All clear
    f4 = make_frame(fdl=100, fcf=100, fdr=100, rdl=100, rdr=100)
    fc4, lc4, rc4 = classify_sectors(f4)
    check(cid, fc4 == DIST_CLEAR, f"all-clear front should be CLEAR: got {fc4}")
    check(cid, lc4 == DIST_CLEAR, f"all-clear left should be CLEAR: got {lc4}")
    check(cid, rc4 == DIST_CLEAR, f"all-clear right should be CLEAR: got {rc4}")

    # Speed scale mapping
    check(cid, speed_scale_from_front(DIST_CLEAR) == 1.0, "CLEAR speed scale should be 1.0")
    check(cid, speed_scale_from_front(DIST_CAUTION) == 0.7, "CAUTION speed scale should be 0.7")
    check(cid, speed_scale_from_front(DIST_NEAR) == 0.45, "NEAR speed scale should be 0.45")
    check(cid, speed_scale_from_front(DIST_DANGER) == 0.0, "DANGER speed scale should be 0.0")


# =========================================================================
#  N3: FSM STATE TRANSITIONS — PRIORITIES P1-P14 + MODIFIERS M1-M5
# =========================================================================

def run_n3():
    cid = "N3"

    # --- P1: Tipover (upright < 0.15) ---
    # Fix 70: P1 suppressed for first NAV_IMU_SETTLE_TICKS ticks (BNO085 convergence).
    # Warm up 4 safe ticks so tick 5 (_tick_count=5 >= 5) fires P1.
    nav = fresh_nav()
    imu_tip = make_imu(upright_quality=0.1)
    for _ in range(4):
        fsm_update_simple(nav)
    state, speed, turn, x_flip, step = fsm_update_simple(nav, imu=imu_tip)
    check(cid, state == NAV_STOP_SAFE, f"P1 tipover: state={state} expected {NAV_STOP_SAFE}")
    check(cid, speed == 0, f"P1 tipover: speed={speed} expected 0")

    # --- P2: Rapid rotation (angular_rate > RAPID_ROTATION_THRESHOLD=3.5, Fix 72) ---
    # Fix 70: P2 also suppressed during settle. Fix 72: threshold raised 1.5->3.5.
    nav = fresh_nav()
    imu_spin = make_imu(angular_rate=4.0)
    for _ in range(4):
        fsm_update_simple(nav)
    state, speed, turn, x_flip, step = fsm_update_simple(nav, imu=imu_spin)
    check(cid, state == NAV_STOP_SAFE, f"P2 rotation: state={state} expected {NAV_STOP_SAFE}")

    # P2 should NOT fire during pivot -- set state to PIVOT, use all-danger frame
    # to prevent P14 from overriding state, so we stay in PIVOT through the check
    nav2 = fresh_nav()
    nav2.state = NAV_PIVOT_TURN
    nav2.prev_state = NAV_BACKWARD
    nav2.consecutive_pivot_count = 0
    nav2.front_danger_frames = 2
    nav2._tick_count = NAV_IMU_SETTLE_TICKS  # past settle guard
    imu_spin2 = make_imu(angular_rate=4.0)
    # All-danger frame with rear safe so P9 fires (pivot), not P2 STOP_SAFE
    frame_deadend2 = make_frame(fdl=10, fcf=10, fdr=10, rdl=100, rdr=100, rcf=100)
    state2, _, _, _, step2 = fsm_update_simple(nav2, frame=frame_deadend2, imu=imu_spin2)
    # P9 triggers PIVOT_TURN; P2 is skipped because state is PIVOT_TURN
    check(cid, state2 == NAV_PIVOT_TURN,
          f"P2 during pivot should be exempted: state={NAV_STATE_NAMES.get(state2)} step={step2}")

    # --- N3.A: P1 suppressed during IMU settle period (Fix 70) ---
    # Physical failure caught: BNO085 reports garbage upright_quality on startup
    # (reads near 0 during quaternion convergence) -> robot stops before moving.
    nav_a = fresh_nav()
    imu_tip_a = make_imu(upright_quality=0.05)
    for tick in range(1, 4):  # ticks 1-3: settle guard active (_tick_count < 5)
        s_a, _, _, _, _ = fsm_update_simple(nav_a, imu=imu_tip_a)
        check(cid, s_a != NAV_STOP_SAFE,
              f"N3.A: P1 should be suppressed during IMU settle tick {tick}: got {NAV_STATE_NAMES.get(s_a)}")

    # --- N3.B: P2 suppressed during IMU settle period (Fix 70) ---
    # Physical failure caught: gyro spins up noisily on boot -> spurious STOP_SAFE.
    nav_b = fresh_nav()
    imu_spin_b = make_imu(angular_rate=4.0)
    for tick in range(1, 4):  # ticks 1-3: settle guard active
        s_b, _, _, _, _ = fsm_update_simple(nav_b, imu=imu_spin_b)
        check(cid, s_b != NAV_STOP_SAFE,
              f"N3.B: P2 should be suppressed during IMU settle tick {tick}: got {NAV_STATE_NAMES.get(s_b)}")

    # angular_rate=2.0 no longer triggers P2 (threshold raised 1.5->3.5, Fix 72)
    # Catches regression: old threshold would stop robot during normal walking arcs.
    nav_b2 = fresh_nav()
    nav_b2._tick_count = NAV_IMU_SETTLE_TICKS  # past settle
    s_b2, _, _, _, _ = fsm_update_simple(nav_b2, imu=make_imu(angular_rate=2.0))
    check(cid, s_b2 != NAV_STOP_SAFE,
          f"N3.B: angular_rate=2.0 should NOT trigger P2 (threshold now 3.5): got {NAV_STATE_NAMES.get(s_b2)}")

    # --- N3.C: P2 suppressed in ARC_LEFT, ARC_RIGHT, BACKWARD, WIGGLE (Fix 72) ---
    # Physical failure caught: walking rotation oscillation (~2-3 rad/s) during
    # arcs/reversals was triggering STOP_SAFE mid-maneuver.
    for exempt_state in (NAV_ARC_LEFT, NAV_ARC_RIGHT, NAV_BACKWARD, NAV_WIGGLE):
        nav_c = fresh_nav()
        nav_c.state = exempt_state
        nav_c._tick_count = NAV_IMU_SETTLE_TICKS  # past settle
        s_c, _, _, _, step_c = fsm_update_simple(nav_c, imu=make_imu(angular_rate=5.0))
        check(cid, s_c != NAV_STOP_SAFE,
              f"N3.C: P2 should be suppressed in {NAV_STATE_NAMES.get(exempt_state)}: "
              f"got {NAV_STATE_NAMES.get(s_c)} step={step_c}")

    # --- P3: Critical battery (voltage < 10.5) ---
    nav = fresh_nav()
    state, speed, turn, x_flip, step = fsm_update_simple(nav, voltage=10.0)
    check(cid, state == NAV_STOP_SAFE, f"P3 battery: state={state} expected {NAV_STOP_SAFE}")

    # --- P6: Front cliff ---
    nav = fresh_nav()
    frame = make_frame(rdl=100, rcf=100, rdr=100)  # rear safe
    state, speed, turn, x_flip, step = fsm_update_simple(nav, frame=frame, front_cliff=True)
    check(cid, state == NAV_BACKWARD, f"P6 front cliff: state={state} expected {NAV_BACKWARD}")
    check(cid, x_flip == -1, f"P6 front cliff: x_flip={x_flip} expected -1 (reversing)")

    # --- P7: Rear cliff ---
    nav = fresh_nav()
    state, speed, turn, x_flip, step = fsm_update_simple(nav, rear_cliff=True)
    check(cid, state == NAV_SLOW_FORWARD, f"P7 rear cliff: state={state} expected {NAV_SLOW_FORWARD}")
    check(cid, x_flip == 1, f"P7 rear cliff: x_flip={x_flip} expected 1 (forward)")

    # --- N3.P6b_lockout: Cliff lockout keeps BACKWARD after cliff clears ---
    nav_p6b = fresh_nav()
    frame_rear_safe = make_frame(rdl=100, rcf=100, rdr=100)
    # Trigger cliff to set lockout
    fsm_update_simple(nav_p6b, frame=frame_rear_safe, front_cliff=True)
    check(cid, nav_p6b.cliff_backup_until > 0.0,
          f"N3.P6b_lockout: cliff_backup_until should be set: got {nav_p6b.cliff_backup_until}")
    # Cliff clears -- lockout should keep BACKWARD (cliff_backup_until still in future)
    state_p6b, _, _, x_flip_p6b, _ = fsm_update_simple(nav_p6b, frame=frame_rear_safe, front_cliff=False)
    check(cid, state_p6b == NAV_BACKWARD,
          f"N3.P6b_lockout: should stay BACKWARD during lockout: got {NAV_STATE_NAMES.get(state_p6b)}")
    check(cid, x_flip_p6b == -1,
          f"N3.P6b_lockout: x_flip should be -1 during lockout: got {x_flip_p6b}")

    # --- N3.P6b_escape: After lockout expires, robot escapes (arc toward free side) ---
    nav_esc = fresh_nav()
    frame_left_clear = make_frame(fdl=100, fcf=10, fdr=10, rdl=100, rcf=100, rdr=100)
    # Trigger cliff
    fsm_update_simple(nav_esc, frame=frame_left_clear, front_cliff=True)
    # Expire the lockout manually
    nav_esc.cliff_backup_until = time.monotonic() - 0.1
    # Update with cliff cleared and left side clear
    state_esc, _, turn_esc, x_flip_esc, step_esc = fsm_update_simple(
        nav_esc, frame=frame_left_clear, front_cliff=False)
    check(cid, state_esc == NAV_ARC_LEFT,
          f"N3.P6b_escape: should escape left (clear side): got {NAV_STATE_NAMES.get(state_esc)}")
    check(cid, "cliff_escape" in step_esc,
          f"N3.P6b_escape: step should be nav_cliff_escape_*: got {step_esc}")
    check(cid, x_flip_esc == 1,
          f"N3.P6b_escape: x_flip should be 1 (forward escape): got {x_flip_esc}")

    # --- N3.P6b_override: P7 rear cliff overrides lockout ---
    nav_ovr = fresh_nav()
    fsm_update_simple(nav_ovr, frame=frame_rear_safe, front_cliff=True)
    check(cid, nav_ovr.cliff_backup_until > 0.0, "N3.P6b_override: lockout should be set")
    # Rear cliff while lockout active -- P7 should win
    state_ovr, _, _, x_flip_ovr, _ = fsm_update_simple(nav_ovr, rear_cliff=True)
    check(cid, state_ovr == NAV_SLOW_FORWARD,
          f"N3.P6b_override: P7 should override lockout: got {NAV_STATE_NAMES.get(state_ovr)}")
    check(cid, nav_ovr.cliff_backup_until == 0.0,
          f"N3.P6b_override: lockout should be cleared by P7: got {nav_ovr.cliff_backup_until}")

    # --- N3.P6b_escape_blocked: Both sides blocked -> pivot after lockout ---
    nav_blk = fresh_nav()
    frame_boxed = make_frame(fdl=10, fcf=10, fdr=10, rdl=10, rdr=10, rcf=100)
    fsm_update_simple(nav_blk, frame=frame_boxed, front_cliff=True)
    nav_blk.cliff_backup_until = time.monotonic() - 0.1  # expire lockout
    state_blk, _, _, _, step_blk = fsm_update_simple(nav_blk, frame=frame_boxed, front_cliff=False)
    check(cid, state_blk == NAV_PIVOT_TURN,
          f"N3.P6b_escape_blocked: both sides blocked should pivot: got {NAV_STATE_NAMES.get(state_blk)}")
    check(cid, "cliff_pivot" in step_blk,
          f"N3.P6b_escape_blocked: step should be nav_cliff_pivot_*: got {step_blk}")

    # --- P8: Stall detection (sustained load > 500 for STALL_SUSTAIN_S) ---
    nav = fresh_nav()
    # First frame starts the timer
    fsm_update_simple(nav, avg_load=600)
    # Simulate time passing beyond STALL_SUSTAIN_S
    nav.stall_start_time = time.monotonic() - STALL_SUSTAIN_S - 0.1
    state, _, _, _, step = fsm_update_simple(nav, avg_load=600)
    check(cid, state == NAV_WIGGLE, f"P8 stall: state={state} expected {NAV_WIGGLE}")

    # --- P9: Dead end (all DANGER + came from BACKWARD) ---
    nav = fresh_nav()
    nav.prev_state = NAV_BACKWARD
    frame_deadend = make_frame(fdl=10, fcf=10, fdr=10, rdl=10, rdr=10, rcf=100)
    # Need 2+ front danger frames for P10 to be checked too, but P9 has higher priority
    # when prev_state is BACKWARD
    nav.front_danger_frames = 2  # simulate prior danger frames
    state, speed, turn, x_flip, step = fsm_update_simple(nav, frame=frame_deadend)
    check(cid, state == NAV_PIVOT_TURN, f"P9 dead end: state={state} expected {NAV_PIVOT_TURN}")
    check(cid, nav.consecutive_pivot_count == 1,
          f"P9 pivot count={nav.consecutive_pivot_count} expected 1")

    # P9 requires prev_state == BACKWARD
    nav3 = fresh_nav()
    nav3.prev_state = NAV_FORWARD  # NOT backward
    nav3.front_danger_frames = 2
    state3, _, _, _, _ = fsm_update_simple(nav3, frame=frame_deadend)
    check(cid, state3 != NAV_PIVOT_TURN,
          f"P9 should NOT fire without prev_state=BACKWARD: got {state3}")

    # --- P10: Front DANGER (2+ frames) -- escape-route awareness ---

    # P10a: Both sides clear -- escape-arc, not backward
    nav = fresh_nav()
    frame_front_danger = make_frame(fcf=10, rdl=100, rcf=100, rdr=100)
    fsm_update_simple(nav, frame=frame_front_danger)  # 1st frame: counter++
    state, _, _, _, step = fsm_update_simple(nav, frame=frame_front_danger)  # 2nd: P10 fires
    check(cid, state in (NAV_ARC_LEFT, NAV_ARC_RIGHT),
          f"N3.P10a: both sides clear should escape-arc: state={NAV_STATE_NAMES.get(state)} step={step}")
    check(cid, "nav_escape" in step,
          f"N3.P10a: step should be nav_escape_*: got {step}")

    # P10b: One side clear, other DANGER -- escape toward clear side
    nav = fresh_nav()
    frame_right_blocked = make_frame(fcf=10, fdr=10, rdl=100, rdr=10)
    fsm_update_simple(nav, frame=frame_right_blocked)
    state, _, turn, _, step = fsm_update_simple(nav, frame=frame_right_blocked)
    check(cid, state == NAV_ARC_LEFT,
          f"N3.P10b: right blocked should escape left: state={NAV_STATE_NAMES.get(state)} step={step}")
    check(cid, turn < 0, f"N3.P10b: escape left turn should be negative: turn={turn}")

    # P10c: Both sides DANGER -- BACKWARD (last resort)
    nav = fresh_nav()
    frame_all_danger = make_frame(fcf=10, fdl=10, fdr=10, rdl=10, rdr=10, rcf=100)
    fsm_update_simple(nav, frame=frame_all_danger)
    state, _, _, _, step = fsm_update_simple(nav, frame=frame_all_danger)
    check(cid, state == NAV_BACKWARD,
          f"N3.P10c: both sides blocked should backward: state={NAV_STATE_NAMES.get(state)} step={step}")

    # P10d: Tie-break -- same classification, pick side with more raw cm
    nav = fresh_nav()
    frame_tie = make_frame(fcf=10, fdl=35, rdl=35, fdr=45, rdr=45)
    fsm_update_simple(nav, frame=frame_tie)
    state, _, turn, _, step = fsm_update_simple(nav, frame=frame_tie)
    check(cid, state == NAV_ARC_RIGHT,
          f"N3.P10d: right has more cm should escape right: state={NAV_STATE_NAMES.get(state)} step={step}")
    check(cid, turn > 0, f"N3.P10d: escape right turn should be positive: turn={turn}")

    # --- P11: Lateral obstacle (one side NEAR, other freer) ---
    nav = fresh_nav()
    frame_left_near = make_frame(fdl=25, rdl=25, fdr=100, rdr=100)  # left NEAR, right CLEAR
    state, _, turn, _, step = fsm_update_simple(nav, frame=frame_left_near)
    # left_class=NEAR, right_class=CLEAR -> left < right severity (wait, max is worst)
    # left = max(classify(25=NEAR), classify(25=NEAR)) = NEAR
    # right = max(classify(100=CLEAR), classify(100=CLEAR)) = CLEAR
    # Since left_class(NEAR=2) > right_class(CLEAR=0), BUT left_class != right_class
    # and left_class < right_class is FALSE (2 > 0), so it goes to ARC_RIGHT
    # Wait: the code checks: if left_class < right_class => ARC_LEFT
    # left_class=2 < right_class=0? NO. So else: ARC_RIGHT? That's wrong...
    # Re-reading: left_class=NEAR=2, right_class=CLEAR=0.
    # "left_class < right_class" => 2 < 0 => False => else: ARC_RIGHT
    # This makes sense: right side is freer (CLEAR), so arc toward right
    # Actually the HIGHER severity number means MORE dangerous.
    # Left is more dangerous => turn RIGHT (away from danger)
    check(cid, state == NAV_ARC_RIGHT,
          f"P11 left near, right clear: state={state} expected {NAV_ARC_RIGHT} (turn away from danger)")

    # Reverse: right near, left clear
    nav = fresh_nav()
    frame_right_near = make_frame(fdr=25, rdr=25, fdl=100, rdl=100)
    state, _, turn, _, step = fsm_update_simple(nav, frame=frame_right_near)
    check(cid, state == NAV_ARC_LEFT,
          f"P11 right near, left clear: state={state} expected {NAV_ARC_LEFT}")

    # --- P12: Narrow corridor (both sides DANGER, front clear) ---
    nav = fresh_nav()
    frame_corridor = make_frame(fdl=10, fdr=10, rdl=10, rdr=10, fcf=100)
    state, _, _, _, step = fsm_update_simple(nav, frame=frame_corridor)
    check(cid, state == NAV_SLOW_FORWARD,
          f"P12 narrow corridor: state={state} expected {NAV_SLOW_FORWARD}")

    # --- P13: Front CAUTION ---
    nav = fresh_nav()
    frame_caution = make_frame(fcf=40)  # 40cm = CAUTION
    state, speed, turn, _, step = fsm_update_simple(nav, frame=frame_caution)
    check(cid, state == NAV_SLOW_FORWARD,
          f"P13 front caution: state={state} expected {NAV_SLOW_FORWARD}")
    check(cid, speed > 0, f"P13 speed should be > 0: got {speed}")

    # --- P14: All clear ---
    nav = fresh_nav()
    frame_clear = make_frame()
    state, speed, turn, _, step = fsm_update_simple(nav, frame=frame_clear)
    check(cid, state == NAV_FORWARD,
          f"P14 all clear: state={state} expected {NAV_FORWARD}")
    check(cid, speed > 0, f"P14 speed should be > 0: got {speed}")

    # --- Modifier M1: Impact spike -> speed x0.5 for 1s ---
    nav = fresh_nav()
    imu_impact = make_imu(accel_mag=25)
    # Run update to get base speed
    state, base_speed, _, _, _ = fsm_update_simple(nav)
    # Trigger impact
    nav._impact_cooldown_until = time.monotonic() + 1.0
    modified = nav.apply_modifiers(base_speed, make_imu_level(), 25, 12.0, 0.0, 50, 0.0, 0.0)
    check(cid, modified == int(base_speed * 0.5),
          f"M1 impact: modified={modified} expected {int(base_speed * 0.5)}")

    # --- Modifier M2: Low battery -> speed x0.7 ---
    nav = fresh_nav()
    state, base_speed, _, _, _ = fsm_update_simple(nav)
    modified = nav.apply_modifiers(base_speed, make_imu_level(), 9.81, 10.8, 0.0, 50, 0.0, 0.0)
    check(cid, modified == int(base_speed * 0.7),
          f"M2 low battery: modified={modified} expected {int(base_speed * 0.7)}")

    # --- Modifier M3: Data gap 300ms-5s -> speed x0.5 ---
    nav = fresh_nav()
    state, base_speed, _, _, _ = fsm_update_simple(nav)
    modified = nav.apply_modifiers(base_speed, make_imu_level(), 9.81, 12.0, 0.0, 50, 1.0, 0.0)
    check(cid, modified == int(base_speed * 0.5),
          f"M3 data gap: modified={modified} expected {int(base_speed * 0.5)}")

    # --- Modifier M4: Lateral roll > 10deg sustained -> speed x0.7 ---
    nav = fresh_nav()
    nav._roll_sustained_start = time.monotonic() - 1.0  # sustained > 0.5s
    state, base_speed, _, _, _ = fsm_update_simple(nav)
    modified = nav.apply_modifiers(base_speed, make_imu(roll_deg=15), 9.81, 12.0, 0.0, 50, 0.0, 15.0)
    check(cid, modified == int(base_speed * 0.7),
          f"M4 roll: modified={modified} expected {int(base_speed * 0.7)}")
    check(cid, nav.terrain_impact_start == 325,
          f"M4 stealth crawl impact_start={nav.terrain_impact_start} expected 325")

    # --- Modifier M5: Load asymmetry -> speed x0.8 ---
    nav = fresh_nav()
    nav._asymmetry_start = time.monotonic() - 2.0  # sustained > 1s
    state, base_speed, _, _, _ = fsm_update_simple(nav)
    modified = nav.apply_modifiers(base_speed, make_imu_level(), 9.81, 12.0, 0.0, 300, 0.0, 0.0)
    check(cid, modified == int(base_speed * 0.8),
          f"M5 asymmetry: modified={modified} expected {int(base_speed * 0.8)}")


# =========================================================================
#  N4: FSM STUCK-STATE PREVENTION
# =========================================================================

def run_n4():
    cid = "N4"

    # ARC dwell blocks rapid re-entry
    nav = fresh_nav()
    frame_right_near = make_frame(fdr=25, rdr=25, fdl=100, rdl=100)
    # Trigger ARC_LEFT
    state, _, _, _, _ = fsm_update_simple(nav, frame=frame_right_near)
    check(cid, state == NAV_ARC_LEFT, f"initial ARC_LEFT: state={state}")
    # Dwell should be active
    check(cid, nav._dwell_active(), "dwell should be active after ARC transition")

    # BACKWARD escalation: 2+ holds with rear unsafe -> pivot (after dwell expires)
    nav = fresh_nav()
    frame_rear_blocked = make_frame(fcf=10, fdl=10, fdr=10, rdl=10, rcf=10, rdr=10)
    nav.front_danger_frames = 2  # force P10
    # First update: BACKWARD with rear unsafe -> hold (count=1)
    state1, speed1, _, _, _ = fsm_update_simple(nav, frame=frame_rear_blocked)
    check(cid, state1 == NAV_BACKWARD, f"backward first: state={state1}")
    check(cid, speed1 == 0, f"rear unsafe hold: speed={speed1} expected 0")

    # Dwell guard: premature escalation should be blocked
    nav.front_danger_frames = 2
    state_early, _, _, _, _ = fsm_update_simple(nav, frame=frame_rear_blocked)
    check(cid, state_early == NAV_BACKWARD,
          f"dwell guard: premature escalation blocked: state={NAV_STATE_NAMES.get(state_early)}")

    # Simulate dwell elapsed, then escalate
    nav.backward_entry_time = time.monotonic() - 1.0  # 1s ago > BACKWARD_MIN_DWELL
    nav.front_danger_frames = 2
    state2, _, _, _, step2 = fsm_update_simple(nav, frame=frame_rear_blocked)
    check(cid, state2 == NAV_PIVOT_TURN,
          f"backward escalation to pivot: state={state2} expected {NAV_PIVOT_TURN}")

    # Max 2 consecutive pivots -> STOP_SAFE
    nav = fresh_nav()
    nav.prev_state = NAV_BACKWARD
    nav.consecutive_pivot_count = 2
    nav.front_danger_frames = 2
    frame_deadend = make_frame(fdl=10, fcf=10, fdr=10, rdl=10, rdr=10, rcf=100)
    state, _, _, _, _ = fsm_update_simple(nav, frame=frame_deadend)
    check(cid, state == NAV_STOP_SAFE,
          f"max pivots: state={state} expected {NAV_STOP_SAFE}")

    # All sensors UNKNOWN -> should not crash and should produce valid state
    nav = fresh_nav()
    frame_unknown = make_frame(fdl=-5, fcf=-5, fdr=-5, rdl=-5, rdr=-5)
    try:
        state, _, _, _, _ = fsm_update_simple(nav, frame=frame_unknown)
        check(cid, state in range(8), f"unknown sensors: got invalid state {state}")
    except Exception as e:
        check(cid, False, f"unknown sensors caused crash: {e}")

    # Mission timeout -> STOP_SAFE
    nav = fresh_nav()
    nav.mission_start = time.monotonic() - MISSION_TIMEOUT_S - 1
    state, speed, _, _, step = fsm_update_simple(nav)
    check(cid, state == NAV_STOP_SAFE, f"timeout: state={state} expected {NAV_STOP_SAFE}")
    check(cid, nav.finished, "timeout: finished flag should be True")
    check(cid, step == "nav_timeout", f"timeout step: '{step}' expected 'nav_timeout'")


# =========================================================================
#  N5: CLIFF DETECTION
# =========================================================================

def run_n5():
    cid = "N5"

    # FCD > 30 absolute -> cliff candidate
    cliff = CliffDetector()
    for _ in range(CLIFF_WARMUP):  # warmup past CLIFF_WARMUP
        cliff.update(15, 15)
    # Frame 1: candidate, not confirmed yet
    fc1, rc1 = cliff.update(35, 15)
    check(cid, fc1 is False, f"single frame should not confirm cliff: got {fc1}")
    # Frame 2: confirmed
    fc2, rc2 = cliff.update(35, 15)
    check(cid, fc2 is True, f"2nd frame should confirm front cliff: got {fc2}")

    # FCD == -1 -> NOT a cliff (reset)
    cliff2 = CliffDetector()
    cliff2.update(35, 15)  # candidate
    fc3, _ = cliff2.update(-1, 15)  # -1 resets
    check(cid, fc3 is False, "FCD=-1 should reset cliff counter")

    # Single-frame spike -> not triggered
    cliff3 = CliffDetector()
    fc4, _ = cliff3.update(50, 15)  # candidate
    fc5, _ = cliff3.update(15, 15)  # back to normal
    check(cid, fc4 is False, "single spike should not trigger cliff")
    check(cid, fc5 is False, "post-spike normal should not trigger cliff")

    # RCD rear cliff
    cliff4 = CliffDetector()
    for _ in range(CLIFF_WARMUP):  # warmup past CLIFF_WARMUP
        cliff4.update(15, 15)
    _, rc1 = cliff4.update(15, 35)  # rear candidate
    _, rc2 = cliff4.update(15, 35)  # rear confirmed
    check(cid, rc2 is True, f"rear cliff should confirm after 2 frames: got {rc2}")

    # Ground EMA updates from valid readings in (0, 40]
    # update(fcd=20, rcd=15): both are non-candidates (< 30 and < EMA+10)
    #   EMA_init=25.0, fcd=20: EMA = 0.2*20 + 0.8*25.0 = 24.0
    #   rcd=15: EMA = 0.2*15 + 0.8*24.0 = 22.2
    cliff5 = CliffDetector()
    initial_ema = cliff5._ground_ema
    cliff5.update(20, 15)  # valid non-candidate readings
    check(cid, cliff5._ground_ema != initial_ema,
          f"EMA should update: was {initial_ema}, now {cliff5._ground_ema}")
    expected_ema = 0.2 * 15 + 0.8 * (0.2 * 20 + 0.8 * initial_ema)
    check(cid, abs(cliff5._ground_ema - expected_ema) < 0.01,
          f"EMA={cliff5._ground_ema} expected {expected_ema}")

    # EMA does NOT update for reading > 40 or <= 0
    cliff6 = CliffDetector()
    ema_before = cliff6._ground_ema
    cliff6._check_cliff(50, True)  # >40, should not update EMA
    check(cid, cliff6._ground_ema == ema_before,
          f"EMA should not update for reading>40: was {ema_before}, now {cliff6._ground_ema}")

    # Delta-based cliff: reading > EMA + 10 (after EMA update)
    # With production EMA ordering: EMA updates BEFORE candidate check, so
    # reading must exceed (updated_EMA + 10). Use reading=28, EMA_init=10,
    # RCD=-1 to avoid rear reading polluting the shared EMA.
    cliff7 = CliffDetector()
    cliff7._ground_ema = 10.0
    cliff7._warmup_frames = CLIFF_WARMUP + 1  # bypass warmup for delta test
    fc_d1, _ = cliff7.update(28, -1)
    fc_d2, _ = cliff7.update(28, -1)
    check(cid, fc_d2 is True, f"delta-based cliff should confirm: reading=28 > EMA(~10)+10")

    # FCD=-1 sentinel specifically resets front counter
    cliff8 = CliffDetector()
    cliff8.update(35, 15)  # candidate -> count=1
    cliff8.update(-1, 15)  # reset
    fc_reset, _ = cliff8.update(35, 15)  # candidate again, count=1
    check(cid, fc_reset is False,
          "after -1 reset, single candidate should not confirm cliff")

    # --- N5.A: 300.0 reading treated as blind zone, not cliff (Fix 71) ---
    # Physical failure caught: acoustic scatter on sand returns HC-SR04 max-range
    # timeout (300cm). Without this fix it triggers spurious cliff stops.
    cliff_a = CliffDetector()
    for _ in range(CLIFF_WARMUP):  # warm past warmup counter
        cliff_a.update(15, 15)
    fc_300, _ = cliff_a.update(300.0, 15)   # 1st frame with 300
    fc_300b, _ = cliff_a.update(300.0, 15)  # 2nd frame -- would confirm if treated as cliff
    check(cid, fc_300b is False,
          f"N5.A: 300.0 reading should NOT trigger cliff (blind zone sentinel): got {fc_300b}")

    # --- N5.B: Warmup suppression for cliff detection (Fix 71) ---
    # Physical failure caught: HC-SR04 returns invalid readings on first frames ->
    # spurious cliff detection before sensor has settled.
    # _warmup_frames increments once per update() call. CLIFF_WARMUP=5.
    # Updates 1-5: suppressed. Update 6: first active (candidate, count=1).
    # Update 7: confirmed (count=2).
    cliff_b = CliffDetector()
    for i in range(CLIFF_WARMUP):  # updates 1-5: cliff still in warmup
        fc_b, _ = cliff_b.update(50.0, 15)  # fcd=50 > 30 = cliff candidate if active
        check(cid, fc_b is False,
              f"N5.B: front cliff suppressed during warmup update {i + 1}: got {fc_b}")
    # Update 6: first active check -- candidate (count=1), not yet confirmed
    fc_b4, _ = cliff_b.update(50.0, 15)
    check(cid, fc_b4 is False,
          f"N5.B: single active candidate should not confirm cliff: got {fc_b4}")
    # Update 5: second consecutive active candidate -- confirmed
    fc_b5, _ = cliff_b.update(50.0, 15)
    check(cid, fc_b5 is True,
          f"N5.B: cliff should confirm after warmup + 2 consecutive frames: got {fc_b5}")


# =========================================================================
#  N6: TURN SIGN CONVENTIONS
# =========================================================================

def run_n6():
    cid = "N6"

    # turn_intensity > 0 means left is freer
    frame_left_free = make_frame(fdl=100, fdr=20)
    ti = compute_turn_intensity(frame_left_free)
    check(cid, ti > 0, f"left freer: turn_intensity={ti:.4f} should be > 0")

    # turn_intensity < 0 means right is freer
    frame_right_free = make_frame(fdl=20, fdr=100)
    ti2 = compute_turn_intensity(frame_right_free)
    check(cid, ti2 < 0, f"right freer: turn_intensity={ti2:.4f} should be < 0")

    # ARC_LEFT: turn_bias always negative
    nav = fresh_nav()
    frame_right_near = make_frame(fdr=25, rdr=25, fdl=100, rdl=100)
    state, speed, turn, _, step = fsm_update_simple(nav, frame=frame_right_near)
    check(cid, state == NAV_ARC_LEFT,
          f"should be ARC_LEFT for right-side obstacle: got {NAV_STATE_NAMES.get(state)}")
    check(cid, turn <= 0, f"ARC_LEFT turn should be <= 0: got {turn:.4f}")

    # ARC_RIGHT: turn_bias always positive
    nav = fresh_nav()
    frame_left_near = make_frame(fdl=25, rdl=25, fdr=100, rdr=100)
    state, speed, turn, _, step = fsm_update_simple(nav, frame=frame_left_near)
    check(cid, state == NAV_ARC_RIGHT,
          f"should be ARC_RIGHT for left-side obstacle: got {NAV_STATE_NAMES.get(state)}")
    check(cid, turn >= 0, f"ARC_RIGHT turn should be >= 0: got {turn:.4f}")

    # PIVOT_TURN left: turn = -PIVOT_TURN_BIAS
    nav = fresh_nav()
    nav.prev_state = NAV_BACKWARD
    nav.front_danger_frames = 2
    frame_deadend = make_frame(fdl=10, fcf=10, fdr=10, rdl=100, rdr=10, rcf=100)
    # RDL=100 > RDR=10 -> pivot_direction = -1 (left)
    state, speed, turn, _, step = fsm_update_simple(nav, frame=frame_deadend)
    check(cid, state == NAV_PIVOT_TURN, f"pivot setup: state={NAV_STATE_NAMES.get(state)}")
    check(cid, turn == -PIVOT_TURN_BIAS,
          f"PIVOT left: turn={turn} expected {-PIVOT_TURN_BIAS}")

    # PIVOT_TURN right: turn = +PIVOT_TURN_BIAS
    nav = fresh_nav()
    nav.prev_state = NAV_BACKWARD
    nav.front_danger_frames = 2
    frame_deadend_r = make_frame(fdl=10, fcf=10, fdr=10, rdl=10, rdr=100, rcf=100)
    # RDR=100 > RDL=10 -> pivot_direction = +1 (right)
    state, speed, turn, _, step = fsm_update_simple(nav, frame=frame_deadend_r)
    check(cid, state == NAV_PIVOT_TURN, f"pivot right setup: state={NAV_STATE_NAMES.get(state)}")
    check(cid, turn == PIVOT_TURN_BIAS,
          f"PIVOT right: turn={turn} expected {PIVOT_TURN_BIAS}")

    # Equal diagonals -> -1 sentinel
    frame_eq = make_frame(fdl=10, fcf=10, fdr=10, rdl=50, rdr=50, rcf=100)
    ti_eq = compute_turn_intensity(frame_eq)
    check(cid, abs(ti_eq) < 0.01,
          f"equal diagonals: turn_intensity={ti_eq:.4f} should be ~0")

    # Blind zone handling: -1 distances replaced with 25cm
    frame_blind = make_frame(fdl=-1, fdr=50)
    ti_blind = compute_turn_intensity(frame_blind)
    # fdl_eff=25, fdr_eff=50 -> tanh((25-50)/50) = tanh(-0.5) < 0
    check(cid, ti_blind < 0,
          f"blind left + clear right: turn_intensity={ti_blind:.4f} should be < 0")


# =========================================================================
#  N7: TERRAIN OVERLAY (T1-T9)
# =========================================================================

def run_n7():
    cid = "N7"

    # T1: Steep climb (pitch > 15, sustained 1s)
    nav = fresh_nav()
    nav.state = NAV_FORWARD
    nav._steep_up_start = time.monotonic() - 1.5  # sustained > 1s
    imu_steep = make_imu(pitch_deg=20)
    nav.update_terrain(imu_steep, 100, 0.0, 9.81, DIST_CLEAR, 0, 0.0)
    check(cid, nav.terrain_gait == 1, f"T1: gait={nav.terrain_gait} expected 1 (wave)")
    check(cid, nav.terrain_impact_start == 325, f"T1: impact_start={nav.terrain_impact_start} expected 325")
    check(cid, nav.terrain_impact_end == 20, f"T1: impact_end={nav.terrain_impact_end} expected 20")
    check(cid, abs(nav.terrain_mult - 0.7) < 0.01, f"T1: terrain_mult={nav.terrain_mult} expected 0.7")

    # T2: Steep descent (pitch < -15, sustained 1s)
    nav = fresh_nav()
    nav.state = NAV_FORWARD
    nav._steep_down_start = time.monotonic() - 1.5
    imu_down = make_imu(pitch_deg=-20)
    nav.update_terrain(imu_down, 100, 0.0, 9.81, DIST_CLEAR, 0, 0.0)
    check(cid, nav.terrain_gait == 1, f"T2: gait={nav.terrain_gait} expected 1 (wave)")
    check(cid, nav.terrain_impact_start == 325, f"T2: impact_start={nav.terrain_impact_start} expected 325")
    check(cid, nav.terrain_impact_end == 35, f"T2: impact_end={nav.terrain_impact_end} expected 35")
    check(cid, abs(nav.terrain_mult - 0.5) < 0.01, f"T2: terrain_mult={nav.terrain_mult} expected 0.5")

    # T3: Moderate slope (abs(pitch) > 12)
    nav = fresh_nav()
    nav.state = NAV_FORWARD
    imu_mod = make_imu(pitch_deg=14)  # > 12 but <= 15 (T1 won't fire)
    nav.update_terrain(imu_mod, 100, 0.0, 9.81, DIST_CLEAR, 0, 0.0)
    check(cid, nav.terrain_gait == 1, f"T3 pitch: gait={nav.terrain_gait} expected 1 (wave)")
    check(cid, abs(nav.terrain_mult - 0.6) < 0.01, f"T3 pitch: terrain_mult={nav.terrain_mult} expected 0.6")

    # T3 via upright_quality (0.15-0.5)
    nav = fresh_nav()
    nav.state = NAV_FORWARD
    imu_tilt = make_imu(upright_quality=0.3)
    nav.update_terrain(imu_tilt, 100, 0.0, 9.81, DIST_CLEAR, 0, 0.0)
    check(cid, nav.terrain_gait == 1, f"T3 tilt: gait={nav.terrain_gait} expected 1 (wave)")
    check(cid, abs(nav.terrain_mult - 0.6) < 0.01, f"T3 tilt: terrain_mult={nav.terrain_mult} expected 0.6")

    # T4: Heavy terrain (avg_load > 400, sustained 2s)
    nav = fresh_nav()
    nav.state = NAV_FORWARD
    nav._heavy_load_start = time.monotonic() - TERRAIN_SUSTAIN_S - 0.5
    imu_level = make_imu_level()
    nav.update_terrain(imu_level, 500, 0.0, 9.81, DIST_CLEAR, 0, 0.0)
    check(cid, nav.terrain_gait == 1, f"T4: gait={nav.terrain_gait} expected 1 (wave)")
    check(cid, abs(nav.terrain_mult - 0.5) < 0.01, f"T4: terrain_mult={nav.terrain_mult} expected 0.5")

    # T5: Excessive wobble (angular_rate > 0.3)
    nav = fresh_nav()
    nav.state = NAV_FORWARD
    nav.update_terrain(make_imu_level(), 100, 0.5, 9.81, DIST_CLEAR, 0, 0.0)
    check(cid, nav.terrain_impact_start == 325, f"T5: impact_start={nav.terrain_impact_start} expected 325")
    check(cid, nav.terrain_impact_end == 35, f"T5: impact_end={nav.terrain_impact_end} expected 35")
    check(cid, abs(nav.terrain_mult - 0.7) < 0.01, f"T5: terrain_mult={nav.terrain_mult} expected 0.7")

    # T6: Rocky terrain (flicker >= 3 + avg_load > 300)
    nav = fresh_nav()
    nav.state = NAV_FORWARD
    nav.update_terrain(make_imu_level(), 350, 0.0, 9.81, DIST_CLEAR, 3, 0.0)
    check(cid, nav.terrain_impact_start == 345, f"T6: impact_start={nav.terrain_impact_start} expected 345")
    check(cid, nav.terrain_impact_end == 15, f"T6: impact_end={nav.terrain_impact_end} expected 15")
    check(cid, abs(nav.terrain_mult - 0.8) < 0.01, f"T6: terrain_mult={nav.terrain_mult} expected 0.8")

    # T7: High vibration (accel_mag > 14, sustained 0.5s)
    nav = fresh_nav()
    nav.state = NAV_FORWARD
    nav._high_vibe_start = time.monotonic() - 1.0
    nav.update_terrain(make_imu_level(), 100, 0.0, 16.0, DIST_CLEAR, 0, 0.0)
    check(cid, nav.terrain_impact_start == 345, f"T7: impact_start={nav.terrain_impact_start} expected 345")
    check(cid, nav.terrain_impact_end == 15, f"T7: impact_end={nav.terrain_impact_end} expected 15")
    check(cid, abs(nav.terrain_mult - 0.8) < 0.01, f"T7: terrain_mult={nav.terrain_mult} expected 0.8")

    # T8: Hard flat (all conditions met, sustained 2s)
    nav = fresh_nav()
    nav.state = NAV_FORWARD
    nav._light_load_start = time.monotonic() - TERRAIN_SUSTAIN_S - 0.5
    nav.update_terrain(make_imu_level(), 100, 0.0, 9.81, DIST_CLEAR, 0, 0.0)
    check(cid, nav.terrain_gait == 0, f"T8: gait={nav.terrain_gait} expected 0 (tripod)")
    check(cid, abs(nav.terrain_mult - 1.0) < 0.01, f"T8: terrain_mult={nav.terrain_mult} expected 1.0")
    check(cid, nav.terrain_is_tripod is True, "T8: terrain_is_tripod should be True")

    # T8 safety gate: tripod fallback on condition drop
    nav.update_terrain(make_imu(pitch_deg=8), 100, 0.0, 9.81, DIST_CLEAR, 0, 0.0)
    # pitch=8 doesn't trigger T1/T2/T3, but we need to fail one T8 condition
    # pitch=8 > 5 fails T8 condition -> tripod safety gate
    check(cid, nav.terrain_is_tripod is False,
          "T8 safety gate: tripod should fallback when conditions no longer met")
    check(cid, nav.terrain_gait == 2, f"T8 safety gate: gait={nav.terrain_gait} expected 2 (quad)")

    # T9: Default quad
    nav = fresh_nav()
    nav.state = NAV_FORWARD
    nav.update_terrain(make_imu_level(), 100, 0.0, 9.81, DIST_CLEAR, 0, 0.0)
    # With _light_load_start=0 (fresh), T8 won't sustain -> falls to T9
    check(cid, nav.terrain_gait == 2, f"T9: gait={nav.terrain_gait} expected 2 (quad)")
    check(cid, abs(nav.terrain_mult - 1.0) < 0.01, f"T9: terrain_mult={nav.terrain_mult} expected 1.0")

    # Terrain held during non-FORWARD states
    nav = fresh_nav()
    nav.state = NAV_BACKWARD
    nav.terrain_gait = 1
    nav.terrain_mult = 0.7
    nav.update_terrain(make_imu_level(), 100, 0.0, 9.81, DIST_CLEAR, 0, 0.0)
    check(cid, nav.terrain_gait == 1,
          f"terrain held during BACKWARD: gait={nav.terrain_gait} expected 1 (unchanged)")
    check(cid, abs(nav.terrain_mult - 0.7) < 0.01,
          f"terrain held during BACKWARD: mult={nav.terrain_mult} expected 0.7")


# =========================================================================
#  N8: UPSIDE-DOWN SENSOR REMAP + SELF-RIGHT
# =========================================================================

def run_n8():
    cid = "N8"

    # UpsideDown=1 -> sensor remap
    line = make_csv_line(fdl=11, fcf=22, fcd=33, fdr=44,
                         rdl=55, rcf=66, rcd=77, rdr=88,
                         upside=1)
    f = _parse_arduino_csv(line)
    check(cid, f is not None, "upside-down CSV returned None")
    if f:
        # After remap: [FDL,FCF,FCD,FDR,RDL,RCF,RCD,RDR] becomes
        #              [RDL,RCF,RCD,RDR,FDL,FCF,FCD,FDR]
        # Original dists = [11,22,33,44, 55,66,77,88]
        # Remapped dists = [55,66,77,88, 11,22,33,44]
        check(cid, f["FDL"] == 55, f"remap FDL: {f['FDL']} expected 55 (was RDL)")
        check(cid, f["FCF"] == 66, f"remap FCF: {f['FCF']} expected 66 (was RCF)")
        check(cid, f["FCD"] == 77, f"remap FCD: {f['FCD']} expected 77 (was RCD)")
        check(cid, f["FDR"] == 88, f"remap FDR: {f['FDR']} expected 88 (was RDR)")
        check(cid, f["RDL"] == 11, f"remap RDL: {f['RDL']} expected 11 (was FDL)")
        check(cid, f["RCF"] == 22, f"remap RCF: {f['RCF']} expected 22 (was FCF)")
        check(cid, f["RCD"] == 33, f"remap RCD: {f['RCD']} expected 33 (was FCD)")
        check(cid, f["RDR"] == 44, f"remap RDR: {f['RDR']} expected 44 (was FDR)")

        # upside_down flag cleared after remap
        check(cid, f["upside_down"] == 0,
              f"upside_down should be cleared after remap: got {f['upside_down']}")

    # No remap when upside=0
    line2 = make_csv_line(fdl=11, fcf=22, fcd=33, fdr=44,
                          rdl=55, rcf=66, rcd=77, rdr=88,
                          upside=0)
    f2 = _parse_arduino_csv(line2)
    if f2:
        check(cid, f2["FDL"] == 11, f"no remap: FDL={f2['FDL']} expected 11")
        check(cid, f2["RDL"] == 55, f"no remap: RDL={f2['RDL']} expected 55")

    # upright_quality uses BOTH pitch AND roll
    frame_tilted = make_frame(qw=1.0, qx=0.0, qy=0.0, qz=0.0)
    imu = compute_imu(frame_tilted)
    # For identity quaternion: pitch=0, roll=0, upright = min(cos(0),cos(0)) = 1.0
    check(cid, abs(imu["upright_quality"] - 1.0) < 0.01,
          f"level upright_quality={imu['upright_quality']} expected ~1.0")

    # IMU dead (qmag < 0.5) -> upright_quality = 0.0
    frame_dead = make_frame(qw=0.0, qx=0.0, qy=0.0, qz=0.0)
    imu_dead = compute_imu(frame_dead)
    check(cid, imu_dead["upright_quality"] == 0.0,
          f"dead IMU upright_quality={imu_dead['upright_quality']} expected 0.0")


# =========================================================================
#  N9: END-TO-END SCENARIO TRACES
# =========================================================================

def run_n9():
    cid = "N9"

    # Scenario 1: Clear run - 50 frames all-clear -> stays FORWARD
    nav = fresh_nav()
    frame_clear = make_frame()
    imu_level = make_imu_level()
    stayed_forward = True
    for i in range(50):
        state, speed, turn, x_flip, step = fsm_update_simple(nav, frame=frame_clear, imu=imu_level)
        if state != NAV_FORWARD:
            stayed_forward = False
            break
    check(cid, stayed_forward, f"clear run: left FORWARD at frame {i}, state={state}")
    check(cid, speed > 0, f"clear run: final speed={speed} should be > 0")

    # Scenario 2: Left obstacle approach -> ARC_RIGHT -> clears -> FORWARD
    nav = fresh_nav()
    # Phase 1: approach (left getting closer)
    distances = [100, 60, 40, 25, 25, 25]
    states_seen = []
    for d in distances:
        frame = make_frame(fdl=d, rdl=d, fdr=100, rdr=100)
        state, _, _, _, _ = fsm_update_simple(nav, frame=frame)
        states_seen.append(state)
    check(cid, NAV_ARC_RIGHT in states_seen,
          f"left obstacle: expected ARC_RIGHT in sequence: {[NAV_STATE_NAMES.get(s) for s in states_seen]}")
    # Fix 67: ARC dwell keeps robot in ARC_RIGHT; wait for dwell to expire, then clear
    time.sleep(0.7)  # P11 entry dwell is 0.6s, last obstacle frame restarts it
    for _ in range(4):
        frame = make_frame(fdl=100, rdl=100, fdr=100, rdr=100)
        state, _, _, _, _ = fsm_update_simple(nav, frame=frame)
        states_seen.append(state)
    check(cid, states_seen[-1] == NAV_FORWARD,
          f"left obstacle: final state should be FORWARD after dwell expiry: got {NAV_STATE_NAMES.get(states_seen[-1])}")

    # Scenario 3: Dead end -> BACKWARD -> PIVOT -> clears -> FORWARD
    nav = fresh_nav()
    # Step 1: FORWARD with all clear
    fsm_update_simple(nav)
    # Step 2: Front DANGER (2 frames needed for P10)
    frame_blocked = make_frame(fcf=10, fdl=10, fdr=10, rdl=100, rcf=100, rdr=100)
    fsm_update_simple(nav, frame=frame_blocked)  # frame 1 danger
    state, _, _, x_flip, _ = fsm_update_simple(nav, frame=frame_blocked)  # frame 2 -> BACKWARD
    check(cid, state == NAV_BACKWARD, f"dead end step1: state={NAV_STATE_NAMES.get(state)} expected BACKWARD")

    # Step 3: From BACKWARD, all sides danger -> PIVOT (P9 requires prev_state=BACKWARD)
    # nav.prev_state should now be FORWARD (before it transitioned to BACKWARD)
    # We need nav.prev_state == NAV_BACKWARD for P9
    # Actually after transition to BACKWARD, prev_state = FORWARD
    # P9 checks prev_state == BACKWARD, so we need to come FROM backward
    # Let's simulate: BACKWARD is current, we call update again with all danger
    # _transition to BACKWARD again won't change prev_state (same state)
    # We need to force prev_state
    nav.prev_state = NAV_BACKWARD
    nav.front_danger_frames = 2
    frame_all_blocked = make_frame(fdl=10, fcf=10, fdr=10, rdl=10, rdr=10, rcf=100)
    state, _, _, _, _ = fsm_update_simple(nav, frame=frame_all_blocked)
    check(cid, state == NAV_PIVOT_TURN,
          f"dead end step2: state={NAV_STATE_NAMES.get(state)} expected PIVOT_TURN")

    # Step 4: After pivot, front clears
    frame_clear_after = make_frame()
    state, _, _, _, _ = fsm_update_simple(nav, frame=frame_clear_after)
    check(cid, state == NAV_FORWARD,
          f"dead end step3: state={NAV_STATE_NAMES.get(state)} expected FORWARD after clearing")

    # Scenario 4: Cliff at speed -> BACKWARD -> clears -> FORWARD
    nav = fresh_nav()
    cliff = CliffDetector()
    for _ in range(CLIFF_WARMUP):  # warmup past CLIFF_WARMUP
        cliff.update(15, 15)
    # 2 cliff frames
    cliff.update(50, 15)  # candidate
    fc, _ = cliff.update(50, 15)  # confirmed
    check(cid, fc is True, "cliff scenario: cliff should be confirmed")
    frame_cliff = make_frame(rdl=100, rcf=100, rdr=100)
    state, _, _, x_flip, _ = fsm_update_simple(nav, frame=frame_cliff, front_cliff=True)
    check(cid, state == NAV_BACKWARD, f"cliff scenario: state={NAV_STATE_NAMES.get(state)} expected BACKWARD")
    check(cid, x_flip == -1, f"cliff scenario: x_flip={x_flip} expected -1")
    # Cliff clears -- lockout keeps BACKWARD
    state2, _, _, _, _ = fsm_update_simple(nav, front_cliff=False)
    check(cid, state2 == NAV_BACKWARD,
          f"cliff clears (lockout active): state={NAV_STATE_NAMES.get(state2)} expected BACKWARD")
    # After lockout expires -- escape arc away from cliff (not back toward it)
    nav.cliff_backup_until = time.monotonic() - 0.1  # expire lockout
    state3, _, _, _, step3 = fsm_update_simple(nav, front_cliff=False)
    check(cid, state3 in (NAV_ARC_LEFT, NAV_ARC_RIGHT),
          f"cliff lockout expired: should escape arc, got {NAV_STATE_NAMES.get(state3)}")
    check(cid, "cliff_escape" in step3,
          f"cliff lockout expired: step should contain cliff_escape, got {step3}")

    # Scenario 5: Rear cliff during BACKWARD -> FORWARD @ SLOW_SPEED (P7)
    nav = fresh_nav()
    nav.state = NAV_BACKWARD
    state, speed, _, x_flip, _ = fsm_update_simple(nav, rear_cliff=True)
    check(cid, state == NAV_SLOW_FORWARD,
          f"rear cliff: state={NAV_STATE_NAMES.get(state)} expected SLOW_FORWARD")
    check(cid, x_flip == 1, f"rear cliff: x_flip={x_flip} expected 1")


# =========================================================================
#  N10: REVERSE SAFETY + FINISH DETECTION
# =========================================================================

def run_n10():
    cid = "N10"

    # BACKWARD checks rear sensors before reversing
    # Note: fdl=10, fdr=10 ensure both sides DANGER so P10 escape-route falls through to BACKWARD
    nav = fresh_nav()
    frame_rear_safe = make_frame(fcf=10, fdl=10, fdr=10, rdl=100, rcf=100, rdr=100)
    nav.front_danger_frames = 2
    state, speed, _, x_flip, _ = fsm_update_simple(nav, frame=frame_rear_safe)
    check(cid, state == NAV_BACKWARD, f"rear safe: state={NAV_STATE_NAMES.get(state)}")
    check(cid, x_flip == -1, f"rear safe: x_flip={x_flip} expected -1 (reversing)")
    check(cid, speed == BACKWARD_SPEED, f"rear safe: speed={speed} expected {BACKWARD_SPEED}")

    # Rear distance < 20 or -1 -> hold (speed=0)
    nav = fresh_nav()
    frame_rear_blocked = make_frame(fcf=10, fdl=10, fdr=10, rdl=10, rcf=100, rdr=100)
    nav.front_danger_frames = 2
    state, speed, _, x_flip, _ = fsm_update_simple(nav, frame=frame_rear_blocked)
    check(cid, state == NAV_BACKWARD, f"rear blocked: state={NAV_STATE_NAMES.get(state)}")
    check(cid, speed == 0, f"rear blocked: speed={speed} expected 0 (hold position)")

    # Rear with -1 sentinel -> hold
    nav = fresh_nav()
    frame_rear_blind = make_frame(fcf=10, fdl=10, fdr=10, rdl=-1, rcf=100, rdr=100)
    nav.front_danger_frames = 2
    state, speed, _, _, _ = fsm_update_simple(nav, frame=frame_rear_blind)
    check(cid, speed == 0, f"rear -1: speed={speed} expected 0 (hold)")

    # Reverse uses x_flip=-1, NOT negative speed
    nav = fresh_nav()
    frame_rev = make_frame(fcf=10, fdl=10, fdr=10, rdl=100, rcf=100, rdr=100)
    nav.front_danger_frames = 2
    state, speed, _, x_flip, _ = fsm_update_simple(nav, frame=frame_rev)
    check(cid, x_flip == -1, "reverse should use x_flip=-1")
    check(cid, speed >= 0, f"speed should not be negative: got {speed}")

    # Transition OUT of BACKWARD resets x_flip=1 (integration level check)
    nav = fresh_nav()
    nav.state = NAV_BACKWARD
    frame_clear = make_frame()
    state, _, _, x_flip, _ = fsm_update_simple(nav, frame=frame_clear)
    check(cid, state == NAV_FORWARD, f"exit backward: state={NAV_STATE_NAMES.get(state)}")
    check(cid, x_flip == 1, f"exit backward: x_flip={x_flip} expected 1")

    # Finish detection: front < 10cm sustained 2s AND rear clear
    nav = fresh_nav()
    nav.state = NAV_FORWARD
    frame_finish = make_frame(fcf=5, rcf=100)
    nav.finish_wall_start = time.monotonic() - FINISH_WALL_SUSTAIN_S - 0.1
    imu = make_imu_level()
    # Need to call update directly to trigger finish wall logic
    fc, lc, rc = classify_sectors(frame_finish)
    ti = compute_turn_intensity(frame_finish)
    state, speed, _, _, step = nav.update(
        frame_finish, imu, fc, lc, rc,
        False, False, ti, 100, 50, 0.0, 9.81, 12.0, 0)
    check(cid, state == NAV_STOP_SAFE, f"finish: state={NAV_STATE_NAMES.get(state)} expected STOP_SAFE")
    check(cid, nav.finished, "finish: finished flag should be True")
    check(cid, step == "nav_finish", f"finish: step='{step}' expected 'nav_finish'")

    # Mission timeout
    nav = fresh_nav()
    nav.mission_start = time.monotonic() - MISSION_TIMEOUT_S - 1
    state, speed, _, _, step = fsm_update_simple(nav)
    check(cid, state == NAV_STOP_SAFE, f"timeout: state={NAV_STATE_NAMES.get(state)}")
    check(cid, speed == 0, f"timeout: speed={speed} expected 0")
    check(cid, step == "nav_timeout", f"timeout step='{step}' expected 'nav_timeout'")

    # Finish NOT triggered if rear is blocked (boxed in)
    nav = fresh_nav()
    nav.state = NAV_FORWARD
    frame_boxed = make_frame(fcf=5, rcf=10)  # rear blocked
    nav.finish_wall_start = time.monotonic() - FINISH_WALL_SUSTAIN_S - 0.1
    imu = make_imu_level()
    fc, lc, rc = classify_sectors(frame_boxed)
    ti = compute_turn_intensity(frame_boxed)
    state, _, _, _, step = nav.update(
        frame_boxed, imu, fc, lc, rc,
        False, False, ti, 100, 50, 0.0, 9.81, 12.0, 0)
    check(cid, step != "nav_finish",
          f"boxed in: should NOT detect finish wall when rear blocked: step='{step}'")


# =========================================================================
#  N11: GRACEFUL DEGRADATION
# =========================================================================

def run_n11():
    cid = "N11"

    # Timed fallback constants exist (pyserial fallback would use these)
    check(cid, CRUISE_SPEED > 0, "CRUISE_SPEED should exist and be > 0")
    check(cid, SLOW_SPEED > 0, "SLOW_SPEED should exist and be > 0")
    check(cid, BACKWARD_SPEED > 0, "BACKWARD_SPEED should exist and be > 0")
    check(cid, MISSION_TIMEOUT_S > 0, "MISSION_TIMEOUT_S should exist and be > 0")

    # 20 consecutive parse failures (simulated)
    fail_count = 0
    for i in range(25):
        if _parse_arduino_csv("garbage_data") is None:
            fail_count += 1
    check(cid, fail_count == 25, f"all garbage lines rejected: {fail_count}/25")
    # The STOP_SAFE logic for 20 failures is in ArduinoReader._run() which
    # we don't test here (it's a thread), but we verify the parser rejects them

    # No data stale > 5s -> caller should force STOP_SAFE (tested via timeout check)
    # We verify that mission_timeout handles the eventual stop
    nav = fresh_nav()
    nav.mission_start = time.monotonic() - MISSION_TIMEOUT_S - 1
    state, _, _, _, _ = fsm_update_simple(nav)
    check(cid, state == NAV_STOP_SAFE, "timeout safety net: STOP_SAFE on mission timeout")

    # M3: Data gap 300ms-5s -> speed x0.5
    nav = fresh_nav()
    state, base_speed, _, _, _ = fsm_update_simple(nav)
    modified = nav.apply_modifiers(base_speed, make_imu_level(), 9.81, 12.0, 0.0, 50, 2.0, 0.0)
    check(cid, modified == int(base_speed * 0.5),
          f"M3 data gap: modified={modified} expected {int(base_speed * 0.5)}")

    # Data gap >= 5s should NOT apply M3 (caller handles STOP_SAFE instead)
    nav = fresh_nav()
    state, base_speed, _, _, _ = fsm_update_simple(nav)
    modified2 = nav.apply_modifiers(base_speed, make_imu_level(), 9.81, 12.0, 0.0, 50, 6.0, 0.0)
    check(cid, modified2 == base_speed,
          f"stale >= 5s: M3 should NOT apply: modified={modified2} expected {base_speed}")

    # Data gap <= 0.3s should NOT apply M3
    nav = fresh_nav()
    state, base_speed, _, _, _ = fsm_update_simple(nav)
    modified3 = nav.apply_modifiers(base_speed, make_imu_level(), 9.81, 12.0, 0.0, 50, 0.1, 0.0)
    check(cid, modified3 == base_speed,
          f"stale <= 0.3s: M3 should NOT apply: modified={modified3} expected {base_speed}")

    # IMU dead (qmag < 0.5) -> upright_quality=0.0 -> triggers P1 STOP_SAFE
    nav = fresh_nav()
    for _ in range(NAV_IMU_SETTLE_TICKS - 1):  # warmup past IMU settle guard
        fsm_update_simple(nav)
    frame_dead_imu = make_frame(qw=0.0, qx=0.0, qy=0.0, qz=0.0)
    imu_dead = compute_imu(frame_dead_imu)
    check(cid, imu_dead["upright_quality"] == 0.0,
          f"dead IMU upright={imu_dead['upright_quality']} expected 0.0")
    state, _, _, _, _ = fsm_update_simple(nav, frame=frame_dead_imu, imu=imu_dead)
    check(cid, state == NAV_STOP_SAFE,
          f"dead IMU -> STOP_SAFE: state={NAV_STATE_NAMES.get(state)}")

    # Battery voltage edge cases
    check(cid, compute_battery_mult(12.0) == 1.0, "12V -> mult=1.0")
    check(cid, compute_battery_mult(10.8) == 0.7, "10.8V -> mult=0.7")
    check(cid, compute_battery_mult(10.0) == 0.0, "10.0V -> mult=0.0")
    check(cid, compute_battery_mult(10.5) == 0.7, "10.5V -> mult=0.7 (edge)")
    check(cid, compute_battery_mult(11.0) == 1.0, "11.0V -> mult=1.0 (above 10.5+0.5)")

    # compute_servo_loads with all-negative inputs
    loads_empty = compute_servo_loads([-1, -1, -1, -1, -1, -1])
    check(cid, loads_empty["avg_load"] == 0, f"all-negative loads: avg={loads_empty['avg_load']}")

    # compute_servo_loads with None/empty
    loads_none = compute_servo_loads([])
    check(cid, loads_none["avg_load"] == 0, "empty loads: avg=0")
    loads_none2 = compute_servo_loads(None)
    check(cid, loads_none2["avg_load"] == 0, "None loads: avg=0")

    # is_rear_safe edge cases
    check(cid, is_rear_safe(None) is False, "is_rear_safe(None) should be False")
    check(cid, is_rear_safe(make_frame(rcf=100, rdl=100, rdr=100)) is True,
          "rear all clear should be safe")
    check(cid, is_rear_safe(make_frame(rcf=-1, rdl=100, rdr=100)) is False,
          "rear RCF=-1 should be unsafe")
    check(cid, is_rear_safe(make_frame(rcf=100, rdl=15, rdr=100)) is False,
          "rear RDL=15 (<20) should be unsafe")



# =========================================================================
#  NC: ARC DWELL TIMEOUT + FIX 67/68 REGRESSION
# =========================================================================

def run_nc():
    """Test C -- ARC dwell timeout and Fix 67/68 correctness.

    Fix 67: early return in ARC dwell block prevents fall-through to FORWARD
    logic. When the FSM is in NAV_ARC_LEFT/RIGHT and _dwell_active(), decide()
    must return an arc action (not FORWARD), with the correct turn sign.
    After the 0.6s dwell expires naturally the FSM is free to transition.

    Fix 68: _start_dwell(0.8) only fires on the FIRST frame of a P6/P10
    transition. Repeated frames in the same BACKWARD state must NOT reset
    dwell_start. dwell_duration persists across _transition() by design --
    the arc dwell must survive a momentary P14 re-evaluation.
    """
    cid = "NC"

    # ------------------------------------------------------------------
    # C1: ARC_LEFT action returned while dwell active (Fix 67)
    # Scenario: right-side obstacle triggered ARC_LEFT. Next frame the
    # obstacle is gone (all-clear), but dwell is still live. The FSM must
    # NOT return FORWARD -- it must hold the arc action.
    # ------------------------------------------------------------------
    nav = fresh_nav()
    frame_right_near = make_frame(fdr=25, rdr=25, fdl=100, rdl=100)  # right NEAR, left CLEAR
    frame_clear = make_frame()

    state0, _, _, _, _ = fsm_update_simple(nav, frame=frame_right_near)
    check(cid, state0 == NAV_ARC_LEFT,
          f"C1 setup: expected ARC_LEFT for right-side obstacle, got {NAV_STATE_NAMES.get(state0)}")
    check(cid, nav._dwell_active(),
          "C1 setup: dwell must be active immediately after ARC_LEFT entry")

    # All-clear frame while dwell is still active.
    # Fix 67: returns arc action. Without fix: falls through to NAV_FORWARD.
    state1, speed1, turn1, x_flip1, step1 = fsm_update_simple(nav, frame=frame_clear)
    check(cid, state1 == NAV_ARC_LEFT,
          f"C1 Fix67: all-clear during ARC dwell must stay ARC_LEFT, got {NAV_STATE_NAMES.get(state1)}")
    check(cid, turn1 <= 0,
          f"C1 Fix67: ARC_LEFT turn must be <= 0 during dwell, got {turn1:.4f}")
    check(cid, speed1 > 0,
          f"C1 Fix67: ARC_LEFT speed must be > 0 during dwell, got {speed1}")
    check(cid, x_flip1 == 1,
          f"C1 Fix67: x_flip must be 1 (forward) during ARC, got {x_flip1}")
    check(cid, isinstance(step1, str) and len(step1) > 0,
          f"C1 Fix67: step_name must be non-empty string, got {step1!r}")

    # ------------------------------------------------------------------
    # C2: ARC_RIGHT action returned while dwell active, turn sign positive
    # ------------------------------------------------------------------
    nav = fresh_nav()
    frame_left_near = make_frame(fdl=25, rdl=25, fdr=100, rdr=100)  # left NEAR, right CLEAR

    state_r0, _, _, _, _ = fsm_update_simple(nav, frame=frame_left_near)
    check(cid, state_r0 == NAV_ARC_RIGHT,
          f"C2 setup: expected ARC_RIGHT for left-side obstacle, got {NAV_STATE_NAMES.get(state_r0)}")
    check(cid, nav._dwell_active(), "C2 setup: dwell must be active after ARC_RIGHT entry")

    state_r1, _, turn_r1, _, _ = fsm_update_simple(nav, frame=frame_clear)
    check(cid, state_r1 == NAV_ARC_RIGHT,
          f"C2 Fix67: all-clear during ARC_RIGHT dwell must stay ARC_RIGHT, got {NAV_STATE_NAMES.get(state_r1)}")
    check(cid, turn_r1 >= 0,
          f"C2 Fix67: ARC_RIGHT turn must be >= 0 during dwell, got {turn_r1:.4f}")

    # ------------------------------------------------------------------
    # C3: Dwell expires naturally -- FSM transitions to FORWARD
    # After the 0.6s dwell timer expires, an all-clear frame must yield FORWARD.
    # ------------------------------------------------------------------
    nav = fresh_nav()
    fsm_update_simple(nav, frame=frame_right_near)  # enter ARC_LEFT
    check(cid, nav.state == NAV_ARC_LEFT, "C3 setup: must be in ARC_LEFT")

    ARC_DWELL_S = 0.6  # matches P11 _start_dwell(0.6) in production code
    nav.dwell_start = nav.dwell_start - ARC_DWELL_S - 0.05  # backdate to force expiry

    check(cid, not nav._dwell_active(),
          "C3 setup: dwell must be expired after backdating dwell_start")

    state3, _, _, _, _ = fsm_update_simple(nav, frame=frame_clear)
    check(cid, state3 == NAV_FORWARD,
          f"C3: after ARC dwell expiry + all-clear must go FORWARD, got {NAV_STATE_NAMES.get(state3)}")

    # ------------------------------------------------------------------
    # C4: 5-tuple return format is correct during ARC dwell (Fix 67 shape)
    # ------------------------------------------------------------------
    nav = fresh_nav()
    fsm_update_simple(nav, frame=frame_right_near)  # enter ARC_LEFT
    result = fsm_update_simple(nav, frame=frame_clear)
    check(cid, len(result) == 5,
          f"C4: update() must return 5-tuple during ARC dwell, got {len(result)} elements")
    st4, sp4, tr4, xf4, step4 = result
    check(cid, isinstance(st4, int),   f"C4: element[0] state must be int, got {type(st4)}")
    check(cid, isinstance(sp4, int),   f"C4: element[1] speed must be int, got {type(sp4)}")
    check(cid, isinstance(tr4, float), f"C4: element[2] turn must be float, got {type(tr4)}")
    check(cid, xf4 in (1, -1),         f"C4: element[3] x_flip must be 1 or -1, got {xf4}")
    check(cid, isinstance(step4, str), f"C4: element[4] step_name must be str, got {type(step4)}")

    # ------------------------------------------------------------------
    # C5: Fix 68 -- _start_dwell fires only on FIRST P6 frame (front cliff)
    # dwell_start must NOT advance on repeated frames in the same BACKWARD state.
    # Without Fix 68 each frame resets dwell_start, making the dwell never expire.
    # ------------------------------------------------------------------
    import time as _time

    nav = fresh_nav()
    frame_rear_safe = make_frame(rdl=100, rcf=100, rdr=100)

    state_c1, _, _, _, _ = fsm_update_simple(nav, frame=frame_rear_safe, front_cliff=True)
    check(cid, state_c1 == NAV_BACKWARD,
          f"C5 setup: expected BACKWARD on front cliff, got {NAV_STATE_NAMES.get(state_c1)}")
    dwell_start_frame1 = nav.dwell_start

    _time.sleep(0.02)  # ensure clock advances so a reset would be detectable
    state_c2, _, _, _, _ = fsm_update_simple(nav, frame=frame_rear_safe, front_cliff=True)
    check(cid, state_c2 == NAV_BACKWARD,
          f"C5: second cliff frame must stay BACKWARD, got {NAV_STATE_NAMES.get(state_c2)}")
    check(cid, nav.dwell_start == dwell_start_frame1,
          f"C5 Fix68: dwell_start must not reset on repeated P6 frames "
          f"(frame1={dwell_start_frame1:.6f}, frame2={nav.dwell_start:.6f})")

    # ------------------------------------------------------------------
    # C6: Fix 68 same guard for P10 (front DANGER 2+ frames)
    # Note: fdl=10, fdr=10 ensure both sides DANGER so P10 escape-route falls through to BACKWARD
    # ------------------------------------------------------------------
    nav = fresh_nav()
    frame_front_danger = make_frame(fcf=10, fdl=10, fdr=10, rdl=100, rcf=100, rdr=100)

    fsm_update_simple(nav, frame=frame_front_danger)  # frame 1: counter -> 1, not BACKWARD yet
    state_p10_1, _, _, _, _ = fsm_update_simple(nav, frame=frame_front_danger)  # frame 2 -> BACKWARD
    check(cid, state_p10_1 == NAV_BACKWARD,
          f"C6 setup: expected BACKWARD from P10, got {NAV_STATE_NAMES.get(state_p10_1)}")
    dwell_p10_frame2 = nav.dwell_start

    _time.sleep(0.02)
    state_p10_2, _, _, _, _ = fsm_update_simple(nav, frame=frame_front_danger)
    check(cid, state_p10_2 == NAV_BACKWARD,
          f"C6: third danger frame must stay BACKWARD, got {NAV_STATE_NAMES.get(state_p10_2)}")
    check(cid, nav.dwell_start == dwell_p10_frame2,
          f"C6 Fix68: dwell_start must not reset on repeated P10 frames "
          f"(frame2={dwell_p10_frame2:.6f}, frame3={nav.dwell_start:.6f})")

    # ------------------------------------------------------------------
    # C7: Fix 68 -- _transition() resets dwell_duration to 0 to prevent
    # stale dwells from carrying into the next state.
    # ------------------------------------------------------------------
    nav = fresh_nav()
    fsm_update_simple(nav, frame=frame_right_near)  # enter ARC_LEFT
    check(cid, nav.dwell_duration > 0,
          f"C7: dwell_duration must be > 0 after ARC entry, got {nav.dwell_duration}")
    nav._transition(NAV_FORWARD)  # manually call _transition; must zero dwell
    check(cid, nav.dwell_duration == 0,
          f"C7: _transition() must zero dwell_duration (Fix 68), "
          f"got {nav.dwell_duration}")

# =========================================================================
#  MAIN — RUN ALL CHECKS
# =========================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("      sim_nav.py — Brain-Level Nav Test Suite")
    print("=" * 60)
    print()

    # Run all test categories
    categories = [
        ("N1", "Serial parser robustness", run_n1),
        ("N2", "Sensor classification", run_n2),
        ("N3", "FSM state transitions (P1-P14 + M1-M5)", run_n3),
        ("N4", "FSM stuck-state prevention", run_n4),
        ("N5", "Cliff detection", run_n5),
        ("N6", "Turn sign conventions", run_n6),
        ("N7", "Terrain overlay (T1-T9)", run_n7),
        ("N8", "Upside-down remap + self-right", run_n8),
        ("N9", "End-to-end scenario traces", run_n9),
        ("N10", "Reverse safety + finish detection", run_n10),
        ("N11", "Graceful degradation", run_n11),
        ("NC",  "ARC dwell timeout + Fix 67/68", run_nc),
    ]

    for check_id, name, fn in categories:
        try:
            fn()
        except Exception as e:
            check(check_id, False, f"UNCAUGHT EXCEPTION: {e}")

    # ---- Report ----
    print("PASS/FAIL per check:")
    for check_id, name, _ in categories:
        tag = pf(check_id)
        diag_list = _diags.get(check_id, [])
        diag_count = len(diag_list)
        if tag == "PASS":
            print(f"  {check_id:4s} {name:45s} PASS")
        else:
            print(f"  {check_id:4s} {name:45s} FAIL  ({diag_count} sub-check{'s' if diag_count != 1 else ''} failed)")

    # Detailed diagnostics
    any_fail = False
    diag_lines = []
    for check_id, name, _ in categories:
        diag_list = _diags.get(check_id, [])
        if diag_list:
            any_fail = True
            diag_lines.append(f"\n{check_id} failures ({len(diag_list)}):")
            for d in diag_list:
                diag_lines.append(f"  - {d}")

    if diag_lines:
        print()
        print("=" * 60)
        print("DETAILED DIAGNOSTICS")
        print("=" * 60)
        for line in diag_lines:
            print(line)

    # Overall
    print()
    print("=" * 60)
    all_pass = all(_results.get(c, True) for c, _, _ in categories)
    total_checks = len(categories)
    pass_count = sum(1 for c, _, _ in categories if _results.get(c, True))
    fail_count = total_checks - pass_count

    if all_pass:
        print(f"NAV SIMULATION COMPLETE - all {total_checks} categories verified.")
        print("Brain-level nav logic cleared for integration testing.")
    else:
        fails = [c for c, _, _ in categories if not _results.get(c, True)]
        print(f"NAV SIMULATION FAILED - {fail_count} categor{'y' if fail_count == 1 else 'ies'} failed: {', '.join(fails)}")
        print("Resolve the above before deploying autonomous nav.")
    print("=" * 60)
