"""
test_nav_logic.py — Synthetic-frame unit tests for Phase 2 (sensor processing)
and Phase 3 (NavStateMachine) nav components.

All functions under test live inside `if __name__ == "__main__":` in
final_full_gait_test.py and cannot be imported directly. They are
mirrored verbatim here. When the gait file changes any of these functions,
the corresponding mirror must be updated in sync.

This follows the same pattern established in test_nav_serial.py (ADR-001).

Source line ranges (as of 2026-03-13):
  Phase 2 sensor processing : lines 1280–1492
  Nav constants              : lines 1104–1123
  Phase 3 NavStateMachine    : lines 1494–1816

Run:
    pytest C:/Users/rgane/Downloads/test_nav_logic.py -v
"""

import math
import time

# ---------------------------------------------------------------------------
# Stub for brain_log — NavStateMachine calls this on every transition.
# In the test environment there is no log file, so we no-op it.
# ---------------------------------------------------------------------------

def brain_log(msg):
    """No-op stub replacing the Brain-process log writer."""
    pass


# ---------------------------------------------------------------------------
# Nav constants (mirrors lines 1104–1123)
# Keep in sync with gait file.
# ---------------------------------------------------------------------------

CRUISE_SPEED = 500
TRIPOD_CRUISE_SPEED = 420
SLOW_SPEED = 200
BACKWARD_SPEED = 200
MAX_TURN_BIAS = 0.25
PIVOT_TURN_BIAS = 0.35
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


# ---------------------------------------------------------------------------
# Distance classification constants (mirrors lines 1284–1289)
# ---------------------------------------------------------------------------

DIST_DANGER  = 3
DIST_NEAR    = 2
DIST_CAUTION = 1
DIST_CLEAR   = 0
DIST_UNKNOWN = 1   # same numeric value as CAUTION — treat unknown as caution


# ---------------------------------------------------------------------------
# Nav state constants (mirrors lines 1498–1506)
# ---------------------------------------------------------------------------

NAV_FORWARD      = 0
NAV_SLOW_FORWARD = 1
NAV_ARC_LEFT     = 2
NAV_ARC_RIGHT    = 3
NAV_BACKWARD     = 4
NAV_PIVOT_TURN   = 5
NAV_WIGGLE       = 6
NAV_STOP_SAFE    = 7

NAV_STATE_NAMES = {
    0: "FORWARD", 1: "SLOW_FWD", 2: "ARC_L", 3: "ARC_R",
    4: "BACKWARD", 5: "PIVOT", 6: "WIGGLE", 7: "STOP_SAFE",
}


# ---------------------------------------------------------------------------
# Phase 2 mirrors (lines 1291–1469)
# ---------------------------------------------------------------------------

def classify_distance(cm):
    """Mirror of classify_distance (gait file line 1291)."""
    if cm is None:
        return DIST_UNKNOWN
    if cm == -1:
        return DIST_DANGER  # blind zone = very close
    if cm < 0:
        return DIST_UNKNOWN  # invalid
    if cm <= 20:
        return DIST_DANGER
    if cm <= 40:
        return DIST_NEAR
    if cm <= 60:
        return DIST_CAUTION
    return DIST_CLEAR


def classify_sectors(frame):
    """Mirror of classify_sectors_voted (gait file line 1818).
    Uses majority voting: median for 3-sensor front, (max-1) downgrade
    for 2-sensor sides when readings disagree by 2+ levels.
    Blind-zone (-1.0) is hard DANGER -- not outvotable."""
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
    """Mirror of speed_scale_from_front (gait file line 1319)."""
    if front_class == DIST_CLEAR:
        return 1.0
    if front_class == DIST_CAUTION:
        return 0.85
    if front_class == DIST_NEAR:
        return 0.50
    if front_class == DIST_DANGER:
        return 0.0
    return 0.7  # UNKNOWN


class CliffDetector:
    """Mirror of CliffDetector (gait file line 1331)."""

    def __init__(self, alpha=0.2):
        self._alpha = alpha
        self._ground_ema = 15.0  # initial guess: ground ~15cm from sensor
        self._consecutive_front = 0
        self._consecutive_rear = 0

    def update(self, fcd, rcd):
        """Update cliff detection with new FCD and RCD readings.
        Returns (front_cliff, rear_cliff) booleans."""
        front_cliff = self._check_cliff(fcd, is_front=True)
        rear_cliff   = self._check_cliff(rcd, is_front=False)
        return front_cliff, rear_cliff

    def _check_cliff(self, reading, is_front):
        """Check single cliff sensor. Returns True if cliff confirmed."""
        counter_attr = "_consecutive_front" if is_front else "_consecutive_rear"
        count = getattr(self, counter_attr)

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
    """Mirror of compute_imu (gait file line 1376)."""
    w, x, y, z = frame["qw"], frame["qx"], frame["qy"], frame["qz"]

    # Check for invalid quaternion (all zeros = IMU dead)
    qmag = w*w + x*x + y*y + z*z
    if qmag < 0.5:
        return {
            "pitch_deg": 0.0, "roll_deg": 0.0, "yaw_deg": 0.0,
            "pitch_rad": 0.0, "roll_rad": 0.0, "yaw_rad": 0.0,
            "upright_quality": 0.0,  # dead IMU → trigger P1 STOP_SAFE
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

    accel_mag    = math.sqrt(frame["ax"]**2 + frame["ay"]**2 + frame["az"]**2)
    angular_rate = math.sqrt(frame["gx"]**2 + frame["gy"]**2 + frame["gz"]**2)

    return {
        "pitch_deg":      math.degrees(pitch_rad),
        "roll_deg":       math.degrees(roll_rad),
        "yaw_deg":        math.degrees(yaw_rad),
        "pitch_rad":      pitch_rad,
        "roll_rad":       roll_rad,
        "yaw_rad":        yaw_rad,
        "upright_quality": upright_quality,
        "accel_mag":      accel_mag,
        "angular_rate":   angular_rate,
    }


def compute_turn_intensity(frame):
    """Mirror of compute_turn_intensity (gait file line 1425)."""
    fdl = frame["FDL"]
    fdr = frame["FDR"]
    fdl_eff = 25.0 if (fdl is None or fdl == -1) else fdl
    fdr_eff = 25.0 if (fdr is None or fdr == -1) else fdr
    return math.tanh((fdl_eff - fdr_eff) / 50.0)


def compute_battery_mult(voltage_value):
    """Mirror of compute_battery_mult (gait file line 1436)."""
    VOLTAGE_MIN = 10.5
    if voltage_value < VOLTAGE_MIN:
        return 0.0
    if voltage_value < VOLTAGE_MIN + 0.5:
        return 0.7
    return 1.0


def compute_servo_loads(loads_snapshot):
    """Mirror of compute_servo_loads (gait file line 1446)."""
    if not loads_snapshot or all(v < 0 for v in loads_snapshot):
        return {"avg_load": 0, "load_asymmetry": 0, "left_avg": 0, "right_avg": 0}
    valid = [v for v in loads_snapshot if v >= 0]
    if not valid:
        return {"avg_load": 0, "load_asymmetry": 0, "left_avg": 0, "right_avg": 0}
    avg_load      = sum(valid) / len(valid)
    load_asymmetry = max(valid) - min(valid)
    left_indices  = [1, 2, 3]
    right_indices = [0, 4, 5]
    left_vals  = [loads_snapshot[i] for i in left_indices if loads_snapshot[i] >= 0]
    right_vals = [loads_snapshot[i] for i in right_indices if loads_snapshot[i] >= 0]
    left_avg  = sum(left_vals)  / len(left_vals)  if left_vals  else 0
    right_avg = sum(right_vals) / len(right_vals) if right_vals else 0
    return {
        "avg_load":       avg_load,
        "load_asymmetry": load_asymmetry,
        "left_avg":       left_avg,
        "right_avg":      right_avg,
    }


# ---------------------------------------------------------------------------
# Phase 3 mirrors (lines 1513–1816)
# ---------------------------------------------------------------------------

def is_rear_safe(frame):
    """Mirror of is_rear_safe (gait file line 1513)."""
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
    """Mirror of NavStateMachine (gait file line 1528)."""

    def __init__(self):
        self.state = NAV_FORWARD
        self.prev_state = NAV_FORWARD
        self.dwell_start = 0.0
        self.dwell_duration = 0.0
        self.hold_position_count = 0
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
        self.terrain_impact_start = 330
        self.terrain_impact_end = 30
        self.terrain_mult = 1.0
        self.terrain_is_tripod = False
        self._gait_transition_until = 0.0

    def _dwell_active(self):
        return (time.monotonic() - self.dwell_start) < self.dwell_duration

    def _start_dwell(self, duration):
        self.dwell_start = time.monotonic()
        self.dwell_duration = duration

    def _refresh_dwell(self, duration):
        self.dwell_start = time.monotonic()
        self.dwell_duration = duration

    def _transition(self, new_state):
        if new_state != self.state:
            self.prev_state = self.state
            self.state = new_state
            brain_log(f"[NAV] {NAV_STATE_NAMES.get(self.prev_state)}"
                      f"→{NAV_STATE_NAMES.get(new_state)}")

    def update(self, frame, imu, front_class, left_class, right_class,
               front_cliff, rear_cliff, turn_intensity, avg_load,
               load_asymmetry, angular_rate, accel_mag, voltage,
               flicker_count):
        """Evaluate priorities and return (state, speed, turn, x_flip, step_name)."""
        now     = time.monotonic()
        elapsed = now - self.mission_start

        # Mission timeout
        if elapsed >= MISSION_TIMEOUT_S:
            self.finished = True
            return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_timeout")

        # Finish wall detection
        fcf     = frame["FCF"] if frame else 999
        rcf_val = frame["RCF"] if frame else 999
        if fcf != -1 and fcf is not None and 0 < fcf < FINISH_WALL_DIST_CM:
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

        upright   = imu["upright_quality"]
        pitch_deg = imu["pitch_deg"]
        roll_deg  = imu["roll_deg"]

        # Track front danger frames for P10
        if front_class >= DIST_DANGER:
            self.front_danger_frames += 1
        else:
            self.front_danger_frames = 0

        # P1: Tipover
        if upright < 0.15:
            self._transition(NAV_STOP_SAFE)
            return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_stop_safe")

        # P2: Rapid rotation (not during pivot)
        if angular_rate > 1.5 and self.state != NAV_PIVOT_TURN:
            self._transition(NAV_STOP_SAFE)
            brain_log(f"[NAV] rapid rotation {angular_rate:.2f} rad/s")
            return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_stop_safe")

        # P3: Critical battery
        if voltage < 10.5:
            self._transition(NAV_STOP_SAFE)
            brain_log(f"[NAV] critical voltage {voltage:.1f}V")
            return (NAV_STOP_SAFE, 0, 0.0, 1, "nav_stop_safe")

        # P5: Freefall
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

        if front_class < DIST_DANGER:
            self.consecutive_pivot_count = 0

        # P11: Lateral obstacle
        if (left_class >= DIST_NEAR or right_class >= DIST_NEAR):
            if left_class != right_class:
                if left_class < right_class:
                    self._transition(NAV_ARC_LEFT)
                    self._start_dwell(0.6)
                    turn  = -abs(turn_intensity) * MAX_TURN_BIAS
                    speed = int(SLOW_SPEED * self.terrain_mult * self.stall_speed_mult)
                    return (NAV_ARC_LEFT, speed, turn, 1, "nav_arc_L")
                else:
                    self._transition(NAV_ARC_RIGHT)
                    self._start_dwell(0.6)
                    turn  = abs(turn_intensity) * MAX_TURN_BIAS
                    speed = int(SLOW_SPEED * self.terrain_mult * self.stall_speed_mult)
                    return (NAV_ARC_RIGHT, speed, turn, 1, "nav_arc_R")

        # P12: Narrow corridor
        if left_class >= DIST_DANGER and right_class >= DIST_DANGER and front_class < DIST_DANGER:
            self._transition(NAV_SLOW_FORWARD)
            speed_s = speed_scale_from_front(front_class)
            speed   = int(SLOW_SPEED * speed_s * self.terrain_mult * self.stall_speed_mult)
            return (NAV_SLOW_FORWARD, speed, 0.0, 1, "nav_slow_fwd")

        # P13: Front CAUTION or NEAR
        if front_class >= DIST_CAUTION:
            self._transition(NAV_SLOW_FORWARD)
            speed_s = speed_scale_from_front(front_class)
            turn    = -turn_intensity * MAX_TURN_BIAS * 0.5
            speed   = int(SLOW_SPEED * speed_s * self.terrain_mult * self.stall_speed_mult)
            return (NAV_SLOW_FORWARD, speed, turn, 1, "nav_slow_fwd")

        # P14: All clear
        self._transition(NAV_FORWARD)

        if self.state in (NAV_ARC_LEFT, NAV_ARC_RIGHT) and self._dwell_active():
            if self.state == NAV_ARC_LEFT and left_class >= DIST_NEAR:
                self._refresh_dwell(0.4)
            elif self.state == NAV_ARC_RIGHT and right_class >= DIST_NEAR:
                self._refresh_dwell(0.4)

        base_speed = CRUISE_SPEED
        if self.terrain_is_tripod:
            base_speed = TRIPOD_CRUISE_SPEED
        speed_s = speed_scale_from_front(front_class)
        speed   = int(base_speed * speed_s * self.terrain_mult * self.stall_speed_mult)

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
        """Compute backward recovery action with rear safety check."""
        if is_rear_safe(frame):
            self.hold_position_count = 0
            return (NAV_BACKWARD, BACKWARD_SPEED, 0.0, -1, "nav_backward")
        else:
            self.hold_position_count += 1
            brain_log(f"[NAV] rear unsafe, holding (count={self.hold_position_count})")
            if self.hold_position_count >= 2:
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
            self.pivot_direction = -1
            return
        rdl = frame["RDL"]
        rdr = frame["RDR"]
        rdl_eff = 0 if (rdl is None or rdl == -1) else rdl
        rdr_eff = 0 if (rdr is None or rdr == -1) else rdr
        if rdl_eff >= rdr_eff:
            self.pivot_direction = -1   # left
        else:
            self.pivot_direction = 1    # right

    # update_terrain is not tested here (real-time sustained timers make it
    # deterministic only with time injection; a dedicated time-mock suite should
    # cover it separately). The method signature is mirrored for completeness.
    def update_terrain(self, imu, avg_load, angular_rate, accel_mag,
                       front_class, flicker_count, roll_deg):
        pass  # not exercised by these tests

    def _apply_gait_transition(self, prev_gait):
        pass  # no-op in test mirror


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------

def make_frame(
    fdl=100.0, fcf=100.0, fcd=15.0, fdr=100.0,
    rdl=100.0, rcf=100.0, rcd=15.0, rdr=100.0,
    qw=1.0, qx=0.0, qy=0.0, qz=0.0,
    ax=0.0,  ay=0.0,  az=9.8,
    gx=0.0,  gy=0.0,  gz=0.0,
    upside_down=0,
):
    """Synthetic sensor frame with all-clear defaults and identity quaternion.

    Override only the fields relevant to each test.
    Key layout matches the dict produced by _parse_arduino_csv in the gait file
    (note: IMU keys are lowercase qw/qx/... ax/ay/... gx/gy/...).
    """
    return {
        "FDL": fdl, "FCF": fcf, "FCD": fcd, "FDR": fdr,
        "RDL": rdl, "RCF": rcf, "RCD": rcd, "RDR": rdr,
        "qw": qw, "qx": qx, "qy": qy, "qz": qz,
        "ax": ax, "ay": ay, "az": az,
        "gx": gx, "gy": gy, "gz": gz,
        "upside_down": upside_down,
    }


def make_imu(
    pitch_deg=0.0, roll_deg=0.0, yaw_deg=0.0,
    upright_quality=1.0,
    accel_mag=9.81, angular_rate=0.0,
):
    """Build a pre-computed IMU output dict with safe defaults.

    Use this instead of compute_imu() when you want to inject specific
    upright_quality or angular_rate values without constructing a quaternion.
    """
    pitch_rad = math.radians(pitch_deg)
    roll_rad  = math.radians(roll_deg)
    yaw_rad   = math.radians(yaw_deg)
    return {
        "pitch_deg":       pitch_deg,
        "roll_deg":        roll_deg,
        "yaw_deg":         yaw_deg,
        "pitch_rad":       pitch_rad,
        "roll_rad":        roll_rad,
        "yaw_rad":         yaw_rad,
        "upright_quality": upright_quality,
        "accel_mag":       accel_mag,
        "angular_rate":    angular_rate,
    }


def nav_update_all_clear(fsm):
    """Call fsm.update() with a fully-clear environment (no obstacles, safe IMU)."""
    frame = make_frame()
    imu   = make_imu()
    return fsm.update(
        frame=frame, imu=imu,
        front_class=DIST_CLEAR, left_class=DIST_CLEAR, right_class=DIST_CLEAR,
        front_cliff=False, rear_cliff=False,
        turn_intensity=0.0, avg_load=100, load_asymmetry=0,
        angular_rate=0.0, accel_mag=9.81, voltage=12.0,
        flicker_count=0,
    )


# ============================================================================
# SECTION 1 — classify_distance boundary tests
# ============================================================================

def test_classify_distance_sentinel_minus1_is_danger():
    """FDL==-1 (HC-SR04 blind zone, sensor too close) must map to DANGER, not UNKNOWN.

    Physical failure caught: treating blind-zone as 'clear' would allow the robot
    to drive into a wall that the sensor reports as -1.
    """
    assert classify_distance(-1) == DIST_DANGER


def test_classify_distance_none_is_unknown():
    """None reading (missing data frame) must map to DIST_UNKNOWN (treated as caution).

    Physical failure caught: a None propagating to max() in classify_sectors would
    raise TypeError.
    """
    assert classify_distance(None) == DIST_UNKNOWN


def test_classify_distance_negative_other_is_unknown():
    """Values < -1 (unexpected negative, e.g. -5) must map to DIST_UNKNOWN.

    Physical failure caught: distinguishes from -1 sentinel; prevents treating
    sensor glitches as guaranteed danger.
    """
    assert classify_distance(-5) == DIST_UNKNOWN
    assert classify_distance(-100) == DIST_UNKNOWN


def test_classify_distance_zero_is_danger():
    """Distance=0 (object touching sensor face) must be DANGER."""
    assert classify_distance(0) == DIST_DANGER


def test_classify_distance_boundary_20_is_danger():
    """Upper boundary of DANGER zone (0–20 inclusive): 20 → DANGER."""
    assert classify_distance(20) == DIST_DANGER


def test_classify_distance_boundary_21_is_near():
    """First value above DANGER boundary: 21 → NEAR."""
    assert classify_distance(21) == DIST_NEAR


def test_classify_distance_boundary_30_is_near():
    """Upper boundary of NEAR zone (21–30 inclusive): 30 → NEAR."""
    assert classify_distance(30) == DIST_NEAR


def test_classify_distance_boundary_31_is_caution():
    """First value above NEAR boundary: 31 → CAUTION."""
    assert classify_distance(31) == DIST_CAUTION


def test_classify_distance_boundary_60_is_caution():
    """Upper boundary of CAUTION zone (31-60 inclusive): 60 -> CAUTION."""
    assert classify_distance(60) == DIST_CAUTION


def test_classify_distance_boundary_61_is_clear():
    """First value above CAUTION boundary: 61 → CLEAR."""
    assert classify_distance(61) == DIST_CLEAR


def test_classify_distance_large_value_is_clear():
    """Distance well past any obstacle (e.g. 400 cm) → CLEAR."""
    assert classify_distance(400) == DIST_CLEAR


# ============================================================================
# SECTION 2 — classify_sectors worst-case logic
# ============================================================================

def test_classify_sectors_all_clear_returns_zeros():
    """All sensors at 100 cm → (CLEAR, CLEAR, CLEAR) == (0, 0, 0).

    Physical failure caught: non-zero sectors at startup would cause premature
    obstacle avoidance maneuvers.
    """
    f, l, r = classify_sectors(make_frame())
    assert (f, l, r) == (0, 0, 0)


def test_classify_sectors_front_uses_worst_of_fdl_fcf_fdr():
    """Front sector uses median voting. FCF=15 (DANGER) alone is outvoted by 2 CLEAR."""
    f, l, r = classify_sectors(make_frame(fcf=15))
    assert f == DIST_CLEAR  # 1-of-3 outlier rejected by median


def test_classify_sectors_front_fdl_danger_dominates():
    """FDL=15 (DANGER), FCF+FDR clear → front=CLEAR (1-of-3 outlier rejected)."""
    f, _, _ = classify_sectors(make_frame(fdl=15, fcf=100, fdr=100))
    assert f == DIST_CLEAR


def test_classify_sectors_front_fdr_danger_dominates():
    """FDR=15 (DANGER), FCF+FDL clear → front=CLEAR (1-of-3 outlier rejected)."""
    f, _, _ = classify_sectors(make_frame(fdl=100, fcf=100, fdr=15))
    assert f == DIST_CLEAR


def test_classify_sectors_left_uses_fdl_and_rdl_only():
    """Left sector = worst of FDL and RDL. FCF danger must NOT pollute left sector.

    Physical failure caught: if FCF leaked into left classification, the robot
    would arc when it should reverse.
    """
    # FCF=15 (DANGER) but not in left sector; FDL=60 (CLEAR), RDL=100 (CLEAR)
    _, l, _ = classify_sectors(make_frame(fdl=60, fcf=15, rdl=100))
    assert l == DIST_CLEAR


def test_classify_sectors_right_uses_fdr_and_rdr_only():
    """Right sector = worst of FDR and RDR only. FCF must not pollute right."""
    _, _, r = classify_sectors(make_frame(fdr=60, fcf=15, rdr=100))
    assert r == DIST_CLEAR


def test_classify_sectors_left_danger_from_rdl():
    """RDL=15 (DANGER), FDL=100 (CLEAR) → left=NEAR (disagreement downgrade, diff=3 >= 2)."""
    _, l, _ = classify_sectors(make_frame(fdl=100, rdl=15))
    assert l == DIST_NEAR  # max(3,0)-1 = 2 = NEAR


def test_classify_sectors_mixed_front_caution_left_danger_right_clear():
    """Composite: front=CLEAR (median of [CLEAR,CAUTION,CLEAR]), left=NEAR (downgrade), right=CLEAR."""
    f, l, r = classify_sectors(make_frame(
        fdl=100, fcf=35, fdr=100,   # front median: sorted([0,1,0])[1] = 0 = CLEAR
        rdl=10,  rcf=100, rdr=100   # left: FDL=CLEAR(0), RDL=DANGER(3), diff=3 → max-1=2=NEAR
    ))
    assert f == DIST_CLEAR  # median rejects single CAUTION
    assert l == DIST_NEAR   # disagreement downgrade
    assert r == DIST_CLEAR


def test_classify_sectors_sentinel_minus1_in_fdl_makes_front_danger():
    """FDL==-1 (blind zone) → front=DANGER via blind-zone override (not outvotable)."""
    f, _, _ = classify_sectors(make_frame(fdl=-1, fcf=100, fdr=100))
    assert f == DIST_DANGER


# --- Voting-specific tests (added with sector voting implementation) ---

def test_voting_front_2of3_danger_stays_danger():
    """2 of 3 front sensors at DANGER → median = DANGER."""
    f, _, _ = classify_sectors(make_frame(fdl=10, fcf=10, fdr=100))
    assert f == DIST_DANGER


def test_voting_front_1of3_danger_rejected():
    """1 of 3 front sensors at DANGER → median rejects it."""
    f, _, _ = classify_sectors(make_frame(fdl=10, fcf=100, fdr=100))
    assert f == DIST_CLEAR


def test_voting_front_multipath_bounce_rejected():
    """Exact scenario from telemetry: FCF=5.83cm multipath bounce, others clear."""
    f, _, _ = classify_sectors(make_frame(fdl=100, fcf=5.83, fdr=100))
    assert f == DIST_CLEAR  # single bad reading rejected


def test_voting_front_2of3_near_stays_near():
    """2 of 3 front sensors at NEAR → median = NEAR."""
    f, _, _ = classify_sectors(make_frame(fdl=25, fcf=25, fdr=100))
    assert f == DIST_NEAR


def test_voting_front_all_3_agree_danger():
    """All 3 front sensors at DANGER → unanimous DANGER."""
    f, _, _ = classify_sectors(make_frame(fdl=10, fcf=10, fdr=10))
    assert f == DIST_DANGER


def test_voting_blind_zone_not_outvotable():
    """Blind zone (-1) in front cannot be outvoted even with 2 CLEAR sensors."""
    f, _, _ = classify_sectors(make_frame(fdl=-1, fcf=100, fdr=100))
    assert f == DIST_DANGER


def test_voting_blind_zone_in_left_sector():
    """Blind zone in left sector (FDL=-1) → left=DANGER."""
    _, l, _ = classify_sectors(make_frame(fdl=-1, rdl=100))
    assert l == DIST_DANGER


def test_voting_blind_zone_in_right_sector():
    """Blind zone in right sector (FDR=-1) → right=DANGER."""
    _, _, r = classify_sectors(make_frame(fdr=-1, rdr=100))
    assert r == DIST_DANGER


def test_voting_left_2sensor_agree_danger():
    """Both left sensors at DANGER → left=DANGER (no disagreement)."""
    _, l, _ = classify_sectors(make_frame(fdl=10, rdl=10))
    assert l == DIST_DANGER


def test_voting_left_2sensor_disagree_downgrade():
    """Left sensors disagree by 3 levels (DANGER vs CLEAR) → downgrade to NEAR."""
    _, l, _ = classify_sectors(make_frame(fdl=100, rdl=10))
    assert l == DIST_NEAR  # max(3,0)-1 = 2 = NEAR


def test_voting_right_2sensor_disagree_downgrade():
    """Right sensors disagree by 3 levels → downgrade to NEAR."""
    _, _, r = classify_sectors(make_frame(fdr=100, rdr=10))
    assert r == DIST_NEAR


def test_voting_left_small_disagreement_no_downgrade():
    """Left sensors disagree by 1 level (NEAR vs CAUTION) → no downgrade, use max."""
    _, l, _ = classify_sectors(make_frame(fdl=25, rdl=35))
    assert l == DIST_NEAR  # diff=1 < 2, so max(2,1)=2=NEAR


def test_voting_fault_sensor_treated_as_caution_not_danger():
    """FAULT (-2) → classify_distance returns DIST_UNKNOWN=1 (CAUTION).
    Median of [1,0,0] = 0 = CLEAR. Fault doesn't trigger emergency."""
    f, _, _ = classify_sectors(make_frame(fdl=-2, fcf=100, fdr=100))
    assert f == DIST_CLEAR  # fault treated as CAUTION, outvoted by 2 CLEAR


def test_voting_fault_not_blind_zone():
    """FAULT (-2) is NOT blind zone (-1). No blind-zone override triggers."""
    f, _, _ = classify_sectors(make_frame(fdl=-2, fcf=100, fdr=100))
    assert f != DIST_DANGER  # would be DANGER if mistaken for blind zone


def test_voting_left_near_vs_clear_downgrade():
    """Left: FDL=NEAR(25cm), RDL=CLEAR(100cm) → diff=2, downgrade to CAUTION."""
    _, l, _ = classify_sectors(make_frame(fdl=25, rdl=100))
    assert l == DIST_CAUTION  # max(2,0)-1 = 1 = CAUTION


# ============================================================================
# SECTION 3 — speed_scale_from_front
# ============================================================================

def test_speed_scale_clear_is_1():
    """DIST_CLEAR → full speed (multiplier 1.0)."""
    assert speed_scale_from_front(DIST_CLEAR) == 1.0


def test_speed_scale_caution_is_0_7():
    """DIST_CAUTION → 70% speed."""
    assert speed_scale_from_front(DIST_CAUTION) == 0.7


def test_speed_scale_near_is_0_45():
    """DIST_NEAR → 45% speed."""
    assert speed_scale_from_front(DIST_NEAR) == 0.45


def test_speed_scale_danger_is_0():
    """DIST_DANGER → zero speed (stop forward motion)."""
    assert speed_scale_from_front(DIST_DANGER) == 0.0


def test_speed_scale_unknown_is_0_7():
    """DIST_UNKNOWN (value==1) → conservative 70% speed, same as CAUTION."""
    assert speed_scale_from_front(DIST_UNKNOWN) == 0.7


# ============================================================================
# SECTION 4 — CliffDetector
# ============================================================================

def test_cliff_fcd_minus1_is_not_cliff():
    """FCD==-1 means ground is in blind zone (very close). NOT a cliff — counter resets.

    Physical failure caught: if -1 were treated as a large distance, the robot
    would stop at ground level (tight sand grain, ramp base) and never move.
    """
    cd = CliffDetector()
    fc, rc = cd.update(fcd=-1, rcd=15)
    assert not fc
    assert cd._consecutive_front == 0


def test_cliff_fcd_minus1_resets_partial_counter():
    """FCD==-1 after one candidate frame must reset the counter, not confirm cliff."""
    cd = CliffDetector()
    # First frame: large reading (candidate)
    cd.update(fcd=60, rcd=15)
    assert cd._consecutive_front == 1
    # Second frame: blind zone — NOT a cliff, counter resets
    fc, _ = cd.update(fcd=-1, rcd=15)
    assert not fc
    assert cd._consecutive_front == 0


def test_cliff_confirmed_after_two_consecutive_frames():
    """FCD > 30 on two consecutive frames → confirmed cliff.

    Physical failure caught: single-frame glitch (sensor echo from dust, air)
    would cause spurious backward reversals.
    """
    cd = CliffDetector()
    fc1, _ = cd.update(fcd=60, rcd=15)  # frame 1: candidate, not yet confirmed
    assert not fc1
    fc2, _ = cd.update(fcd=60, rcd=15)  # frame 2: confirmed
    assert fc2


def test_cliff_single_candidate_then_normal_resets():
    """One candidate frame followed by a normal reading must not trigger cliff."""
    cd = CliffDetector()
    cd.update(fcd=60, rcd=15)          # candidate
    fc, _ = cd.update(fcd=15, rcd=15)  # normal — resets
    assert not fc
    assert cd._consecutive_front == 0


def test_cliff_delta_detection_triggers_after_two_frames():
    """FCD jumps more than EMA+10 → delta-based cliff candidate, confirmed on 2nd frame.

    Physical failure caught: an absolute threshold alone would miss slow-onset
    drop-offs where the ground gradually descends.
    """
    cd = CliffDetector()
    # Warm up EMA to ~15 cm
    for _ in range(10):
        cd.update(fcd=15, rcd=15)
    # FCD=30 > ema(~15) + 10 → candidate
    fc1, _ = cd.update(fcd=30, rcd=15)
    assert not fc1
    fc2, _ = cd.update(fcd=30, rcd=15)
    assert fc2


def test_cliff_none_fcd_increments_counter():
    """FCD==None (lost frame) treated as possible cliff — increments counter."""
    cd = CliffDetector()
    fc1, _ = cd.update(fcd=None, rcd=15)
    assert not fc1
    assert cd._consecutive_front == 1


def test_cliff_none_fcd_two_frames_confirmed():
    """FCD==None on two consecutive frames → cliff confirmed.

    Physical failure caught: if None were silently ignored, a dead sensor over
    a cliff edge would never stop the robot.
    """
    cd = CliffDetector()
    cd.update(fcd=None, rcd=15)
    fc, _ = cd.update(fcd=None, rcd=15)
    assert fc


def test_cliff_rear_sensor_independent_from_front():
    """Rear cliff (RCD) detector operates independently of front (FCD).

    Physical failure caught: a shared counter would cause a rear cliff reading
    to accidentally confirm a front cliff event already at count=1.
    """
    cd = CliffDetector()
    # Front gets one candidate
    cd.update(fcd=60, rcd=15)
    # Now only rear spikes — front should NOT be confirmed
    fc, rc1 = cd.update(fcd=15, rcd=60)   # front resets, rear first candidate
    assert not fc
    assert not rc1
    fc2, rc2 = cd.update(fcd=15, rcd=60)  # rear second candidate → confirmed
    assert not fc2
    assert rc2


def test_cliff_ema_updated_only_for_valid_low_readings():
    """EMA should not update when reading > 40 (ceiling/wall echo range).

    Physical failure caught: a false wall echo at 200 cm would push EMA up,
    making threshold EMA+10 too high to catch a real cliff at 35 cm.
    """
    cd = CliffDetector()
    initial_ema = cd._ground_ema
    # Feed a large reading; EMA must stay unchanged
    cd.update(fcd=200, rcd=15)
    assert cd._ground_ema == initial_ema  # 200 > 40, no EMA update


# ============================================================================
# SECTION 5 — compute_imu
# ============================================================================

def test_imu_identity_quaternion_zero_angles():
    """Identity quaternion (w=1, x=y=z=0) → pitch=0, roll=0, yaw=0."""
    result = compute_imu(make_frame(qw=1.0, qx=0.0, qy=0.0, qz=0.0))
    assert abs(result["pitch_deg"]) < 1e-6
    assert abs(result["roll_deg"])  < 1e-6
    assert abs(result["yaw_deg"])   < 1e-6


def test_imu_identity_quaternion_upright_quality_one():
    """Identity quaternion → upright_quality = 1.0 (perfectly level)."""
    result = compute_imu(make_frame())
    assert abs(result["upright_quality"] - 1.0) < 1e-6


def test_imu_all_zero_quaternion_safe_defaults():
    """All-zero quaternion (|q|²<0.5) → IMU-dead safe defaults, upright_quality=0.0.

    Physical failure caught: a dead IMU triggers P1 STOP_SAFE immediately.
    """
    result = compute_imu(make_frame(qw=0.0, qx=0.0, qy=0.0, qz=0.0))
    assert result["upright_quality"] == 0.0
    assert result["pitch_deg"] == 0.0
    assert result["roll_deg"]  == 0.0
    assert result["yaw_deg"]   == 0.0
    assert result["accel_mag"]    == 9.81
    assert result["angular_rate"] == 0.0


def test_imu_nose_up_pitch_positive():
    """Nose-up tilt produces positive pitch_deg.

    Quaternion for 30° pitch (rotation about Y-axis):
      w = cos(15°), x=0, y=sin(15°), z=0
    """
    angle = math.radians(30.0)
    qw = math.cos(angle / 2)
    qy = math.sin(angle / 2)
    result = compute_imu(make_frame(qw=qw, qx=0.0, qy=qy, qz=0.0))
    assert result["pitch_deg"] > 0
    assert abs(result["pitch_deg"] - 30.0) < 0.5


def test_imu_roll_right_positive():
    """Roll right produces positive roll_deg.

    Quaternion for 20° roll (rotation about X-axis):
      w = cos(10°), x=sin(10°), y=0, z=0
    """
    angle = math.radians(20.0)
    qw = math.cos(angle / 2)
    qx = math.sin(angle / 2)
    result = compute_imu(make_frame(qw=qw, qx=qx, qy=0.0, qz=0.0))
    assert result["roll_deg"] > 0
    assert abs(result["roll_deg"] - 20.0) < 0.5


def test_imu_upright_quality_decreases_with_tilt():
    """upright_quality < 1.0 when tilted, and decreases as tilt increases."""
    angle_small = math.radians(10.0)
    angle_large = math.radians(45.0)
    r_small = compute_imu(make_frame(
        qw=math.cos(angle_small / 2), qy=math.sin(angle_small / 2)))
    r_large = compute_imu(make_frame(
        qw=math.cos(angle_large / 2), qy=math.sin(angle_large / 2)))
    assert r_small["upright_quality"] < 1.0
    assert r_large["upright_quality"] < r_small["upright_quality"]


def test_imu_upright_quality_is_min_of_cos_pitch_and_cos_roll():
    """upright_quality = min(cos(|pitch_rad|), cos(|roll_rad|)).

    A 45° pitch combined with 10° roll → quality is limited by the larger
    tilt (pitch), not the smaller one.
    """
    pitch_angle = math.radians(45.0)
    roll_angle  = math.radians(10.0)
    # Build a quaternion with pitch=45° and roll=10° by composing rotations
    qw_p = math.cos(pitch_angle / 2);  qy_p = math.sin(pitch_angle / 2)
    qw_r = math.cos(roll_angle  / 2);  qx_r = math.sin(roll_angle  / 2)
    # Compose: Q = Q_pitch * Q_roll (simple approximation for small roll)
    qw = qw_p * qw_r;  qx = qw_p * qx_r;  qy = qy_p * qw_r;  qz = -qy_p * qx_r
    result = compute_imu(make_frame(qw=qw, qx=qx, qy=qy, qz=qz))
    expected = min(math.cos(abs(result["pitch_rad"])),
                   math.cos(abs(result["roll_rad"])))
    assert abs(result["upright_quality"] - expected) < 1e-6


def test_imu_accel_mag_computed_correctly():
    """accel_mag = sqrt(ax² + ay² + az²)."""
    result = compute_imu(make_frame(ax=3.0, ay=4.0, az=0.0))
    assert abs(result["accel_mag"] - 5.0) < 1e-6


def test_imu_angular_rate_computed_correctly():
    """angular_rate = sqrt(gx² + gy² + gz²)."""
    result = compute_imu(make_frame(gx=0.0, gy=3.0, gz=4.0))
    assert abs(result["angular_rate"] - 5.0) < 1e-6


# ============================================================================
# SECTION 6 — compute_turn_intensity
# ============================================================================

def test_turn_intensity_left_freer_positive():
    """FDL > FDR → left is freer → turn_intensity > 0 (must turn left).

    Physical failure caught: if sign is inverted, the robot turns away from the
    open space and into the obstacle.
    """
    ti = compute_turn_intensity(make_frame(fdl=100, fdr=50))
    assert ti > 0


def test_turn_intensity_right_freer_negative():
    """FDR > FDL → right is freer → turn_intensity < 0 (must turn right)."""
    ti = compute_turn_intensity(make_frame(fdl=50, fdr=100))
    assert ti < 0


def test_turn_intensity_equal_is_zero():
    """FDL == FDR → turn_intensity == 0."""
    ti = compute_turn_intensity(make_frame(fdl=100, fdr=100))
    assert abs(ti) < 1e-9


def test_turn_intensity_sentinel_both_minus1_is_zero():
    """FDL==-1 and FDR==-1 → both replaced with 25 cm → difference=0 → intensity=0.

    Physical failure caught: if -1 were used directly in tanh((fdl-fdr)/50),
    the result would be zero by coincidence but for the wrong reason; the
    effective replacement must be explicit.
    """
    ti = compute_turn_intensity(make_frame(fdl=-1, fdr=-1))
    assert abs(ti) < 1e-9


def test_turn_intensity_fdl_sentinel_rdr_clear():
    """FDL==-1 (replaced with 25 cm), FDR=100 → right is freer → ti < 0."""
    ti = compute_turn_intensity(make_frame(fdl=-1, fdr=100))
    assert ti < 0


def test_turn_intensity_range_bounded_minus1_to_plus1():
    """tanh output is always within (-1, +1) regardless of extreme inputs."""
    ti_large = compute_turn_intensity(make_frame(fdl=450, fdr=0))
    ti_small = compute_turn_intensity(make_frame(fdl=0,   fdr=450))
    assert -1.0 < ti_large <= 1.0
    assert -1.0 <= ti_small < 1.0


# ============================================================================
# SECTION 7 — compute_battery_mult
# ============================================================================

def test_battery_mult_nominal_12v_is_1():
    """12.0V → full speed multiplier (1.0)."""
    assert compute_battery_mult(12.0) == 1.0


def test_battery_mult_above_threshold_is_1():
    """Just above low-battery threshold (11.1V) → 1.0."""
    assert compute_battery_mult(11.1) == 1.0


def test_battery_mult_low_battery_is_0_7():
    """10.8V (between 10.5 and 11.0) → reduced multiplier 0.7."""
    assert compute_battery_mult(10.8) == 0.7


def test_battery_mult_critical_is_0():
    """Below VOLTAGE_MIN (10.3V) → 0.0 (STOP_SAFE territory)."""
    assert compute_battery_mult(10.3) == 0.0


def test_battery_mult_boundary_exactly_10_5_is_warning():
    """Exactly at VOLTAGE_MIN (10.5) → low band → 0.7 (strict less-than guard)."""
    assert compute_battery_mult(10.5) == 0.7


def test_battery_mult_boundary_just_above_10_5_is_0_7():
    """Just above VOLTAGE_MIN (10.51) → low band → 0.7."""
    assert compute_battery_mult(10.51) == 0.7


# ============================================================================
# SECTION 8 — compute_servo_loads
# ============================================================================

def test_servo_loads_all_minus1_returns_zeros():
    """All -1 (uninitialized servos) → zero defaults.

    Physical failure caught: if -1 were averaged in, avg_load would be ~-1
    and never exceed STALL_LOAD_THRESHOLD_NAV, masking a real stall.
    """
    result = compute_servo_loads([-1, -1, -1, -1, -1, -1])
    assert result["avg_load"]       == 0
    assert result["load_asymmetry"] == 0
    assert result["left_avg"]       == 0
    assert result["right_avg"]      == 0


def test_servo_loads_empty_list_returns_zeros():
    """Empty list → zero defaults without raising."""
    result = compute_servo_loads([])
    assert result["avg_load"] == 0


def test_servo_loads_normal_average():
    """Six equal loads → avg = that load, asymmetry = 0."""
    loads = [200, 200, 200, 200, 200, 200]
    result = compute_servo_loads(loads)
    assert result["avg_load"] == 200
    assert result["load_asymmetry"] == 0


def test_servo_loads_asymmetry_is_max_minus_min():
    """Asymmetry = max(valid) - min(valid)."""
    loads = [100, 300, 200, 250, 150, 350]
    result = compute_servo_loads(loads)
    assert result["load_asymmetry"] == 250  # 350 - 100


def test_servo_loads_left_right_split():
    """Left indices [1,2,3] and right indices [0,4,5] must be split correctly.

    Physical failure caught: a wrong index mapping would mis-identify which
    side is stalled, causing the robot to arc the wrong way.
    """
    # Left servos (indices 1,2,3): 300, 300, 300
    # Right servos (indices 0,4,5): 100, 100, 100
    loads = [100, 300, 300, 300, 100, 100]
    result = compute_servo_loads(loads)
    assert result["left_avg"]  == 300.0
    assert result["right_avg"] == 100.0


def test_servo_loads_mixed_with_minus1():
    """Valid loads interspersed with -1 sentinels → -1 excluded from averages."""
    # Indices 0=100, 1=-1, 2=200, 3=300, 4=-1, 5=400
    loads = [100, -1, 200, 300, -1, 400]
    result = compute_servo_loads(loads)
    # Valid values: 100, 200, 300, 400 → avg=250
    assert result["avg_load"] == 250.0
    # Left (indices 1,2,3): index 1=-1 excluded, 2=200, 3=300 → avg=250
    assert result["left_avg"] == 250.0
    # Right (indices 0,4,5): 100, -1 excluded, 400 → avg=250
    assert result["right_avg"] == 250.0


# ============================================================================
# SECTION 9 — is_rear_safe
# ============================================================================

def test_is_rear_safe_all_clear_returns_true():
    """All rear sensors > 20 cm → safe to reverse."""
    assert is_rear_safe(make_frame(rdl=100, rcf=100, rdr=100)) is True


def test_is_rear_safe_rcf_blocked_returns_false():
    """RCF=15 (< 20) → rear unsafe, cannot reverse."""
    assert is_rear_safe(make_frame(rcf=15)) is False


def test_is_rear_safe_rdl_sentinel_returns_false():
    """RDL==-1 (blind zone) → treat as unsafe (could be wall at ankle)."""
    assert is_rear_safe(make_frame(rdl=-1)) is False


def test_is_rear_safe_rdr_none_returns_false():
    """RDR==None (no reading) → treat as unsafe."""
    assert is_rear_safe(make_frame(rdr=None)) is False


def test_is_rear_safe_none_frame_returns_false():
    """frame==None → False without raising AttributeError."""
    assert is_rear_safe(None) is False


def test_is_rear_safe_boundary_exactly_20_is_unsafe():
    """RCF==20 is the boundary; the check is v < 20, so 20 is SAFE (not blocked)."""
    assert is_rear_safe(make_frame(rcf=20)) is True


def test_is_rear_safe_boundary_19_is_unsafe():
    """RCF==19 < 20 → unsafe."""
    assert is_rear_safe(make_frame(rcf=19)) is False


# ============================================================================
# SECTION 10 — NavStateMachine FSM priority tests
# ============================================================================

def _run_nav(fsm, frame=None, front_class=DIST_CLEAR, left_class=DIST_CLEAR,
             right_class=DIST_CLEAR, front_cliff=False, rear_cliff=False,
             turn_intensity=0.0, avg_load=100, load_asymmetry=0,
             angular_rate=0.0, accel_mag=9.81, voltage=12.0,
             upright_quality=1.0, flicker_count=0):
    """Thin wrapper: build imu dict from upright_quality and call fsm.update()."""
    if frame is None:
        frame = make_frame()
    imu = make_imu(upright_quality=upright_quality,
                   accel_mag=accel_mag,
                   angular_rate=angular_rate)
    return fsm.update(
        frame=frame, imu=imu,
        front_class=front_class, left_class=left_class, right_class=right_class,
        front_cliff=front_cliff, rear_cliff=rear_cliff,
        turn_intensity=turn_intensity, avg_load=avg_load,
        load_asymmetry=load_asymmetry, angular_rate=angular_rate,
        accel_mag=accel_mag, voltage=voltage,
        flicker_count=flicker_count,
    )


def test_fsm_p14_all_clear_is_forward():
    """P14: No obstacles, safe IMU → NAV_FORWARD."""
    fsm = NavStateMachine()
    state, speed, turn, x_flip, _ = _run_nav(fsm)
    assert state == NAV_FORWARD
    assert x_flip == 1
    assert speed > 0


def test_fsm_p14_forward_x_flip_is_1():
    """FORWARD: x_flip must always be 1 (not reverse direction).

    Physical failure caught: if x_flip were -1 in FORWARD, the robot would
    drive backward while reporting NAV_FORWARD.
    """
    fsm = NavStateMachine()
    _, _, _, x_flip, _ = _run_nav(fsm)
    assert x_flip == 1


def test_fsm_p13_front_caution_is_slow_forward():
    """P13: Front CAUTION → NAV_SLOW_FORWARD."""
    fsm = NavStateMachine()
    state, _, _, _, _ = _run_nav(fsm, front_class=DIST_CAUTION)
    assert state == NAV_SLOW_FORWARD


def test_fsm_p13_front_near_is_slow_forward():
    """P13: Front NEAR → NAV_SLOW_FORWARD (same priority slot)."""
    fsm = NavStateMachine()
    state, _, _, _, _ = _run_nav(fsm, front_class=DIST_NEAR)
    assert state == NAV_SLOW_FORWARD


def test_fsm_p10_front_danger_two_frames_is_backward():
    """P10: Front DANGER on 2+ consecutive frames → NAV_BACKWARD.

    Physical failure caught: single-frame transient (shadow, dust) must not
    trigger backward recovery.
    """
    fsm = NavStateMachine()
    _run_nav(fsm, front_class=DIST_DANGER)   # frame 1: front_danger_frames=1
    state, _, _, _, _ = _run_nav(fsm, front_class=DIST_DANGER)  # frame 2: >=2
    assert state == NAV_BACKWARD


def test_fsm_p10_front_danger_single_frame_not_backward():
    """P10 requires 2+ frames. Single DANGER frame must not immediately back up."""
    fsm = NavStateMachine()
    state, _, _, _, _ = _run_nav(fsm, front_class=DIST_DANGER)
    # First frame: front_danger_frames=1, threshold is >=2, so not yet BACKWARD
    # It may be FORWARD (no lateral data) — what matters is NOT BACKWARD
    assert state != NAV_BACKWARD


def test_fsm_p10_backward_x_flip_minus1_when_rear_safe():
    """When NAV_BACKWARD fires and rear is safe, x_flip=-1 (reverse).

    Physical failure caught: if x_flip stays at 1, the robot drives forward
    while believing it is reversing.
    """
    fsm = NavStateMachine()
    # Two DANGER frames with clear rear
    frame = make_frame(rdl=100, rcf=100, rdr=100)
    _run_nav(fsm, frame=frame, front_class=DIST_DANGER)
    _, _, _, x_flip, _ = _run_nav(fsm, frame=frame, front_class=DIST_DANGER)
    assert x_flip == -1


def test_fsm_p10_backward_hold_when_rear_unsafe():
    """NAV_BACKWARD: rear blocked → hold (x_flip=1, speed=0).

    Physical failure caught: reversing into a wall behind would be as bad as
    the obstacle in front.
    """
    fsm = NavStateMachine()
    # Rear blocked (RCF=10 < 20)
    frame = make_frame(rdl=100, rcf=10, rdr=100)
    _run_nav(fsm, frame=frame, front_class=DIST_DANGER)
    state, speed, _, x_flip, _ = _run_nav(fsm, frame=frame, front_class=DIST_DANGER)
    assert state == NAV_BACKWARD
    assert x_flip == 1   # no reverse
    assert speed == 0    # hold position


def test_fsm_p11_right_near_left_clear_arcs_left():
    """P11: Right NEAR, left CLEAR → NAV_ARC_LEFT (turn toward open side).

    Physical failure caught: arc in wrong direction drives robot into the obstacle.
    """
    fsm = NavStateMachine()
    state, _, _, _, _ = _run_nav(fsm, left_class=DIST_CLEAR, right_class=DIST_NEAR,
                                  turn_intensity=-0.3)
    assert state == NAV_ARC_LEFT


def test_fsm_p11_left_near_right_clear_arcs_right():
    """P11: Left NEAR, right CLEAR → NAV_ARC_RIGHT."""
    fsm = NavStateMachine()
    state, _, _, _, _ = _run_nav(fsm, left_class=DIST_NEAR, right_class=DIST_CLEAR,
                                  turn_intensity=0.3)
    assert state == NAV_ARC_RIGHT


def test_fsm_p11_arc_left_turn_is_negative():
    """ARC_LEFT: turn value must be ≤ 0 (negative bias turns robot left).

    Physical failure caught: a positive turn in ARC_LEFT would turn the robot
    toward the obstacle it is trying to avoid.
    """
    fsm = NavStateMachine()
    _, _, turn, _, _ = _run_nav(fsm, left_class=DIST_CLEAR, right_class=DIST_NEAR,
                                  turn_intensity=-0.5)
    assert turn <= 0


def test_fsm_p11_arc_right_turn_is_positive():
    """ARC_RIGHT: turn value must be ≥ 0 (positive bias turns robot right)."""
    fsm = NavStateMachine()
    _, _, turn, _, _ = _run_nav(fsm, left_class=DIST_NEAR, right_class=DIST_CLEAR,
                                  turn_intensity=0.5)
    assert turn >= 0


def test_fsm_p1_tipover_stop_safe():
    """P1: upright_quality < 0.15 → NAV_STOP_SAFE regardless of other inputs.

    Physical failure caught: attempting forward motion while tipped would drag
    the chassis and destroy the servos.
    """
    fsm = NavStateMachine()
    state, speed, _, _, _ = _run_nav(fsm, front_class=DIST_CLEAR,
                                      upright_quality=0.10)
    assert state == NAV_STOP_SAFE
    assert speed == 0


def test_fsm_p1_tipover_preempts_forward_obstacle():
    """P1 fires even with a clear front — priority 1 is the highest."""
    fsm = NavStateMachine()
    state, _, _, _, _ = _run_nav(fsm, upright_quality=0.05)
    assert state == NAV_STOP_SAFE


def test_fsm_p2_rapid_rotation_stop_safe():
    """P2: angular_rate > 1.5 rad/s outside PIVOT → NAV_STOP_SAFE."""
    fsm = NavStateMachine()
    state, _, _, _, _ = _run_nav(fsm, angular_rate=2.0)
    assert state == NAV_STOP_SAFE


def test_fsm_p2_rapid_rotation_ok_during_pivot():
    """P2: rapid rotation is allowed during NAV_PIVOT_TURN (expected behavior)."""
    fsm = NavStateMachine()
    # Force state into PIVOT manually
    fsm.state = NAV_PIVOT_TURN
    fsm.prev_state = NAV_BACKWARD
    state, _, _, _, _ = _run_nav(fsm, angular_rate=2.0,
                                  front_class=DIST_CLEAR)
    # Should NOT be STOP_SAFE due to angular rate alone
    assert state != NAV_STOP_SAFE


def test_fsm_p3_critical_voltage_stop_safe():
    """P3: voltage < 10.5 V → NAV_STOP_SAFE."""
    fsm = NavStateMachine()
    state, speed, _, _, _ = _run_nav(fsm, voltage=10.0)
    assert state == NAV_STOP_SAFE
    assert speed == 0


def test_fsm_p6_front_cliff_backward():
    """P6: front_cliff=True → NAV_BACKWARD (retreat from cliff).

    Physical failure caught: if FORWARD were returned, the robot would drive
    off the edge.
    """
    fsm = NavStateMachine()
    state, _, _, _, _ = _run_nav(fsm, front_cliff=True,
                                  frame=make_frame(rdl=100, rcf=100, rdr=100))
    assert state == NAV_BACKWARD


def test_fsm_p6_front_cliff_with_safe_rear_x_flip_minus1():
    """P6 front cliff + safe rear → x_flip=-1 (reverse away from cliff)."""
    fsm = NavStateMachine()
    frame = make_frame(rdl=100, rcf=100, rdr=100)
    _, _, _, x_flip, _ = _run_nav(fsm, frame=frame, front_cliff=True)
    assert x_flip == -1


def test_fsm_p7_rear_cliff_slow_forward():
    """P7: rear_cliff=True → NAV_SLOW_FORWARD (move away from rear drop-off).

    Physical failure caught: if BACKWARD were returned with a rear cliff, the
    robot would reverse directly off the edge.
    """
    fsm = NavStateMachine()
    state, _, _, x_flip, _ = _run_nav(fsm, rear_cliff=True)
    assert state == NAV_SLOW_FORWARD
    assert x_flip == 1   # forward, not reverse


def test_fsm_p9_dead_end_pivot_after_backward():
    """P9: front+left+right all DANGER + prev_state==BACKWARD → NAV_PIVOT_TURN.

    Physical failure caught: without the prev_state guard, any three-sided
    box-in (e.g. narrow corridor with NEAR sides) would trigger a pivot instead
    of a careful arc.
    """
    fsm = NavStateMachine()
    fsm.prev_state = NAV_BACKWARD   # simulate came from backward

    frame = make_frame(rdl=100, rcf=100, rdr=100)
    state, _, _, _, _ = _run_nav(
        fsm, frame=frame,
        front_class=DIST_DANGER, left_class=DIST_DANGER, right_class=DIST_DANGER,
    )
    assert state == NAV_PIVOT_TURN


def test_fsm_p9_pivot_not_triggered_without_prior_backward():
    """P9 dead-end pivot must NOT fire unless prev_state is NAV_BACKWARD.

    Physical failure caught: oscillating between pivot and forward on first
    contact with a wall corner.
    """
    fsm = NavStateMachine()
    fsm.prev_state = NAV_FORWARD   # not from backward

    state, _, _, _, _ = _run_nav(
        fsm,
        front_class=DIST_DANGER, left_class=DIST_DANGER, right_class=DIST_DANGER,
    )
    assert state != NAV_PIVOT_TURN


def test_fsm_p9_after_two_pivots_stop_safe():
    """P9: two consecutive pivots failed → NAV_STOP_SAFE (avoid infinite loop).

    Physical failure caught: without the 2-pivot cap, the robot would spin
    forever in a tight corner.
    """
    fsm = NavStateMachine()
    fsm.prev_state = NAV_BACKWARD
    fsm.consecutive_pivot_count = 2   # already used 2 pivots

    frame = make_frame(rdl=100, rcf=100, rdr=100)
    state, _, _, _, _ = _run_nav(
        fsm, frame=frame,
        front_class=DIST_DANGER, left_class=DIST_DANGER, right_class=DIST_DANGER,
    )
    assert state == NAV_STOP_SAFE


# ============================================================================
# SECTION 11 — Turn sign convention regression tests
# ============================================================================

def test_turn_convention_left_freer_arc_bias_negative():
    """Left freer (turn_intensity>0) → ARC_LEFT turn bias < 0 (turns left).

    Convention: turn_bias = -abs(turn_intensity) * MAX_TURN_BIAS
    Positive turn_bias = turn right; negative = turn left.

    Physical failure caught: an inverted sign would turn toward the blocked side.
    """
    fsm = NavStateMachine()
    # Right NEAR (obstacle right), left CLEAR → ARC_LEFT, turn negative
    _, _, turn, _, _ = _run_nav(
        fsm,
        left_class=DIST_CLEAR, right_class=DIST_NEAR,
        turn_intensity=0.5,   # left freer
    )
    assert turn <= 0


def test_turn_convention_right_freer_arc_bias_positive():
    """Right freer (turn_intensity<0) → ARC_RIGHT turn bias > 0 (turns right)."""
    fsm = NavStateMachine()
    _, _, turn, _, _ = _run_nav(
        fsm,
        left_class=DIST_NEAR, right_class=DIST_CLEAR,
        turn_intensity=-0.5,   # right freer
    )
    assert turn >= 0


def test_turn_convention_heading_correction_yaw_error_right_turns_left():
    """Heading correction: yaw drifted right (yaw_error > 30°) → turn negative (correct left).

    Physical failure caught: heading drift correction in the wrong direction
    would amplify steering errors instead of dampening them.
    """
    fsm = NavStateMachine()
    fsm.initial_yaw = 0.0
    # Inject a yaw_rad that is +35° from initial
    imu = make_imu(yaw_deg=35.0, upright_quality=1.0)
    state, _, turn, _, _ = fsm.update(
        frame=make_frame(), imu=imu,
        front_class=DIST_CLEAR, left_class=DIST_CLEAR, right_class=DIST_CLEAR,
        front_cliff=False, rear_cliff=False,
        turn_intensity=0.0, avg_load=100, load_asymmetry=0,
        angular_rate=0.0, accel_mag=9.81, voltage=12.0, flicker_count=0,
    )
    assert state == NAV_FORWARD
    assert turn < 0   # correct left


def test_turn_convention_heading_correction_yaw_error_left_turns_right():
    """Heading drifted left (yaw_error < -30°) → turn positive (correct right)."""
    fsm = NavStateMachine()
    fsm.initial_yaw = 0.0
    imu = make_imu(yaw_deg=-35.0, upright_quality=1.0)
    _, _, turn, _, _ = fsm.update(
        frame=make_frame(), imu=imu,
        front_class=DIST_CLEAR, left_class=DIST_CLEAR, right_class=DIST_CLEAR,
        front_cliff=False, rear_cliff=False,
        turn_intensity=0.0, avg_load=100, load_asymmetry=0,
        angular_rate=0.0, accel_mag=9.81, voltage=12.0, flicker_count=0,
    )
    assert turn > 0   # correct right


def test_turn_convention_small_yaw_error_no_correction():
    """Heading error < 30° must not trigger heading correction (hysteresis band)."""
    fsm = NavStateMachine()
    fsm.initial_yaw = 0.0
    imu = make_imu(yaw_deg=10.0, upright_quality=1.0)
    _, _, turn, _, _ = fsm.update(
        frame=make_frame(), imu=imu,
        front_class=DIST_CLEAR, left_class=DIST_CLEAR, right_class=DIST_CLEAR,
        front_cliff=False, rear_cliff=False,
        turn_intensity=0.0, avg_load=100, load_asymmetry=0,
        angular_rate=0.0, accel_mag=9.81, voltage=12.0, flicker_count=0,
    )
    assert turn == 0.0


# ============================================================================
# SECTION 12 — Rear safety in BACKWARD state
# ============================================================================

def test_rear_safe_backward_fires_x_flip_minus1():
    """Rear clear → backward recovery x_flip=-1 (true reverse)."""
    fsm = NavStateMachine()
    frame = make_frame(rdl=100, rcf=100, rdr=100)
    _run_nav(fsm, frame=frame, front_class=DIST_DANGER)
    _, _, _, x_flip, _ = _run_nav(fsm, frame=frame, front_class=DIST_DANGER)
    assert x_flip == -1


def test_rear_unsafe_first_hold_not_pivot():
    """Rear unsafe on first hold cycle (hold_count=1) → hold position, not pivot yet."""
    fsm = NavStateMachine()
    frame_blocked_rear = make_frame(rcf=10)   # rear blocked
    _run_nav(fsm, frame=frame_blocked_rear, front_class=DIST_DANGER)
    state, speed, _, x_flip, _ = _run_nav(fsm, frame=frame_blocked_rear,
                                           front_class=DIST_DANGER)
    # First hold: should be NAV_BACKWARD hold, not pivot
    assert state == NAV_BACKWARD
    assert speed == 0
    assert x_flip == 1


def test_rear_unsafe_two_holds_escalates_to_pivot():
    """Rear unsafe for 2+ cycles (hold_count >= 2) → escalates to PIVOT_TURN.

    Physical failure caught: if hold looped forever, the robot would be stuck
    with both front and rear blocked and no escape.
    """
    fsm = NavStateMachine()
    frame_blocked = make_frame(rcf=10)
    # Frame 1: sets front_danger_frames=1
    _run_nav(fsm, frame=frame_blocked, front_class=DIST_DANGER)
    # Frame 2: front_danger_frames=2 → P10 fires, _backward_action → hold_count=1
    _run_nav(fsm, frame=frame_blocked, front_class=DIST_DANGER)
    # Frame 3: still DANGER + in BACKWARD, _backward_action → hold_count=2 → pivot
    state, _, _, _, _ = _run_nav(fsm, frame=frame_blocked, front_class=DIST_DANGER)
    assert state == NAV_PIVOT_TURN


# ============================================================================
# SECTION 13 — Finish detection
# ============================================================================

def test_finish_detection_not_triggered_on_first_close_reading():
    """Single FCF < FINISH_WALL_DIST_CM frame must not immediately finish mission.

    Physical failure caught: a narrow gap midcourse would prematurely end the run.
    """
    fsm = NavStateMachine()
    # FCF=5 (< 10) but rear is clear
    frame = make_frame(fcf=5, rcf=100)
    state, _, _, _, step = _run_nav(fsm, frame=frame)
    assert step != "nav_finish"
    assert not fsm.finished


def test_finish_detection_requires_rear_clear():
    """Finish detection must not trigger when rear is also blocked (boxed in, not goal).

    Physical failure caught: if the robot is boxed in with walls on both sides,
    it should not declare victory.
    """
    fsm = NavStateMachine()
    # Simulate sustained FCF < 10, but RCF is also blocked
    fsm.finish_wall_start = time.monotonic() - (FINISH_WALL_SUSTAIN_S + 1.0)
    frame = make_frame(fcf=5, rcf=15)  # rear < 50
    state, _, _, _, step = _run_nav(fsm, frame=frame)
    assert step != "nav_finish"
    assert not fsm.finished


def test_finish_detection_triggers_after_sustained_wall_and_clear_rear():
    """Finish wall sustained >= FINISH_WALL_SUSTAIN_S + rear clear → finished=True.

    Physical failure caught: if dwell timer were missing, the finish line would
    never be declared after the robot reaches it.
    """
    fsm = NavStateMachine()
    # Set finish_wall_start in the past to simulate sustained detection
    fsm.finish_wall_start = time.monotonic() - (FINISH_WALL_SUSTAIN_S + 0.1)
    # FCF=5 < 10, RCF=100 (> 50) — genuine finish wall
    frame = make_frame(fcf=5, rcf=100)
    state, speed, _, _, step = _run_nav(fsm, frame=frame)
    assert fsm.finished is True
    assert state == NAV_STOP_SAFE
    assert speed == 0
    assert step == "nav_finish"


def test_finish_detection_resets_on_sensor_clearing():
    """If FCF goes back above FINISH_WALL_DIST_CM, finish_wall_start resets to 0."""
    fsm = NavStateMachine()
    # Briefly see close wall
    fsm.finish_wall_start = time.monotonic() - 0.5
    # Now FCF is back to 100 (moved away or it was a shadow)
    frame = make_frame(fcf=100, rcf=100)
    _run_nav(fsm, frame=frame)
    assert fsm.finish_wall_start == 0.0


# ============================================================================
# SECTION 14 — Mission timeout
# ============================================================================

def test_mission_timeout_returns_stop_safe():
    """After MISSION_TIMEOUT_S elapsed → NAV_STOP_SAFE, speed=0, finished=True.

    Physical failure caught: without timeout, a robot stuck in a loop would run
    the battery flat after the course window ends.
    """
    fsm = NavStateMachine()
    # Backdate mission_start to simulate timeout
    fsm.mission_start = time.monotonic() - (MISSION_TIMEOUT_S + 1.0)
    state, speed, _, _, step = _run_nav(fsm)
    assert state == NAV_STOP_SAFE
    assert speed == 0
    assert fsm.finished is True
    assert step == "nav_timeout"
