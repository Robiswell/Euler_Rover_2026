# fusion.py
# -----------------------------------------------------------------------------
# RHex Rover – Sensor Fusion and hybrid Decision (array-only output)
#
#   Given one sensor frame (parsed from your 20-column CSV), compute:
#     - Continuous modulation:
#         * turn_intensity  [-1..1], + means turn left (right side is freer)
#         * speed_scale     [0..1],  forward speed scaling based on clearance
#         * tripod_strength [0.4..1.0], gait amplitude scaling based on clearance
#     - Minimal classes/safety cues for analysis (numeric only)
#
# INPUT FRAME (dict)
#   {
#     "timestamp_ms": int,
#     "distances": [FDL, FCF, FCD, FDR, RDL, RCF, RCD, RDR],  # cm; -1 => unknown/no-echo
#     "quat": {"w": float, "x": float, "y": float, "z": float}, # optional, for pitch
#     "upside_down": 0 or 1
#   }
#
# OUTPUT ARRAY (fixed-length numeric list, 10 columns):
#   [0]  ts_ms (int)
#   [1]  state_id (int): 0=FORWARD, 1=SLOW_FORWARD, 2=ARC_LEFT,
#                        3=ARC_RIGHT, 4=BACKWARD_RECOVERY, 5=STOP_SAFE
#   [2]  turn_intensity (float, -1..1)
#   [3]  speed_scale (float, 0..1)
#   [4]  tripod_strength (float, 0.4..1.0)
#   [5]  front_class (int): 0=UNK,1=CLEAR,2=CAUTION,3=NEAR,4=DANGER
#   [6]  left_class  (int): same classes
#   [7]  right_class (int): same classes
#   [8]  cliff (int, 0/1): confirmed cliff (large FCD or -1 vs ground EMA)
#   [9]  upright_quality (float, 0..1): ~cos(|pitch|), 1=upright, ~0=upside
#
# DESIGN
#   - Correct cliff model: down sensor sees big distance (or -1) over a void.
#   - Ground EMA stabilizes cliff detection (relative drop).
#   - Classes follow your rules: >50 Clear, 30–50 Caution, 20–30 Near, <20 Danger.
#   - Hybrid decision:
#       STOP_SAFE > BACKWARD_RECOVERY > ARC_(L/R) > SLOW_FORWARD > FORWARD.
#     With short dwell for BACKWARD & ARC to avoid jitter.
#
# -----------------------------------------------------------------------------

from dataclasses import dataclass
from typing import Dict, Any, List, Optional
import time
import math


# ---------------------- Compact schema helpers ----------------------

FINAL_FIELDS: List[str] = [
    "ts_ms",
    "state_id",          # 0..5 (see STATE_ID)
    "turn_intensity",
    "speed_scale",
    "tripod_strength",
    "front_class",       # 0=UNK,1=CLEAR,2=CAUTION,3=NEAR,4=DANGER
    "left_class",
    "right_class",
    "cliff",             # 0/1
    "upright_quality",   # 0..1
]

# Classes
CLASS_UNKNOWN = 0
CLASS_CLEAR   = 1
CLASS_CAUTION = 2
CLASS_NEAR    = 3
CLASS_DANGER  = 4

# States
STATE_ID = {
    "FORWARD":            0,
    "SLOW_FORWARD":       1,
    "ARC_LEFT":           2,
    "ARC_RIGHT":          3,
    "BACKWARD_RECOVERY":  4,
    "STOP_SAFE":          5,
}


# ---------------------- Low-level utilities ----------------------

def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def _valid(x: float) -> bool:
    return (x is not None) and (x >= 0.0)

def _min_valid(a: float, b: float) -> Optional[float]:
    """Return min(a,b) if both valid; else return the valid one; else None."""
    va, vb = _valid(a), _valid(b)
    if va and vb: return min(a, b)
    if va: return a
    if vb: return b
    return None

def _quat_to_pitch_deg(w: float, x: float, y: float, z: float) -> float:
    """Pitch (deg) from quaternion (w,x,y,z) using pitch = asin(2*(w*y - z*x))."""
    s = 2.0 * (w * y - z * x)
    s = _clamp(s, -1.0, 1.0)
    return math.degrees(math.asin(s))


# ---------------------- Configurations ----------------------

@dataclass
class FusionConfig:
    # Front classification thresholds (cm)
    T_CLEAR: float = 50.0
    T_CAUTION: float = 30.0
    T_NEAR: float = 20.0
    T_DANGER: float = 20.0

    # Cliff detection (large FCD or -1 relative to ground estimate)
    T_CLIFF_ABS: float = 30.0     # absolute large value means likely drop
    T_CLIFF_DELTA: float = 10.0   # relative over ground_est (EMA)

    # Temporal confirmation
    N_CONFIRM: int = 2            # cliff and front_danger

    # Ground estimation (EMA) from FCD when floor-like
    GROUND_MAX_ACCEPT: float = 40.0
    GROUND_EMA_ALPHA: float = 0.2

    # Lateral scoring for turn_intensity
    W_DIAG: float = 1.0
    W_CENTER: float = 0.5
    SCORE_DEFAULT: float = 35.0
    TURN_SCALE: float = 50.0      # tanh(diff/scale)

    # Continuous modulation
    SPEED_CLEAR: float = 1.0
    SPEED_CAUTION: float = 0.7
    SPEED_NEAR: float = 0.5
    SPEED_DANGER: float = 0.0

    TRIPOD_MIN: float = 0.4
    TRIPOD_MAX: float = 1.0
    TRIPOD_FCF_DEN: float = 80.0  # tripod_strength ~ clamp(FCF/80, 0.4..1)

    # Swap here only if upstream does NOT swap
    APPLY_SWAP: bool = False


@dataclass
class DecisionConfig:
    # Dwell times to avoid jitter (seconds)
    T_BACKWARD: float = 0.8
    T_ARC_MIN: float = 0.6

    # Side selection hysteresis (cm advantage to switch)
    DELTA_SIDE_CM: float = 10.0

    # Allow SLOW_FORWARD only when upright
    MIN_UPRIGHT_4_SLOW: float = 0.5  # if upright_quality < this, skip SLOW

    # If front is UNKNOWN, be conservative (use CAUTION speed)
    UNKNOWN_AS_CAUTION: bool = True


# ---------------------- Fusion Engine (compact) ----------------------

class FusionEngine:
    """
    Sensor fusion with compact outputs used by the decision layer.
    Produces: front/left/right classes, cliff, turn_intensity, speed_scale, tripod_strength, upright_quality.
    """

    def __init__(self, cfg: Optional[FusionConfig] = None):
        self.cfg = cfg or FusionConfig()
        self._ground_est: float = -1.0
        self._cliff_run: int = 0
        self._cliff_clear_run: int = 0
        self._front_danger_run: int = 0
        self._front_clear_run: int = 0

    def reset(self) -> None:
        self._ground_est = -1.0
        self._cliff_run = self._cliff_clear_run = 0
        self._front_danger_run = self._front_clear_run = 0

    def classify(self, d: Optional[float]) -> int:
        """Return class id for a distance."""
        c = self.cfg
        if d is None or not _valid(d):
            return CLASS_UNKNOWN
        if d > c.T_CLEAR:
            return CLASS_CLEAR
        if c.T_CAUTION <= d <= c.T_CLEAR:
            return CLASS_CAUTION
        if c.T_NEAR <= d < c.T_CAUTION:
            return CLASS_NEAR
        return CLASS_DANGER  # < 20

    def update(self, frame: Dict[str, Any]) -> Dict[str, Any]:
        """
        Compute fusion features. Returns a dict:
          {
            "ts": int,
            "front_class": int,
            "left_class": int,
            "right_class": int,
            "cliff": int,
            "turn_intensity": float,
            "speed_scale": float,
            "tripod_strength": float,
            "upright_quality": float,
          }
        """
        c = self.cfg

        ts = int(frame.get("timestamp_ms", 0))
        distances = frame.get("distances", [])
        upside = int(frame.get("upside_down", 0))
        quat = frame.get("quat", {})
        qw = float(quat.get("w", 1.0))
        qx = float(quat.get("x", 0.0))
        qy = float(quat.get("y", 0.0))
        qz = float(quat.get("z", 0.0))

        # Validate distances
        if not isinstance(distances, list) or len(distances) != 8:
            return {
                "ts": ts,
                "front_class": CLASS_UNKNOWN,
                "left_class": CLASS_UNKNOWN,
                "right_class": CLASS_UNKNOWN,
                "cliff": 0,
                "turn_intensity": 0.0,
                "speed_scale": 0.0,
                "tripod_strength": c.TRIPOD_MIN,
                "upright_quality": 0.0,
            }

        # Optional internal swap (use only if upstream doesn't do it)
        if c.APPLY_SWAP and upside == 1:
            distances = distances[4:8] + distances[0:4]

        # Unpack front-active distances
        FDL, FCF, FCD, FDR, RDL, RCF, RCD, RDR = distances

        # Ground EMA (only integrate floor-like values)
        if _valid(FCD) and FCD < c.GROUND_MAX_ACCEPT:
            if self._ground_est < 0:
                self._ground_est = FCD
            else:
                a = c.GROUND_EMA_ALPHA
                self._ground_est = (1.0 - a) * self._ground_est + a * FCD

        # Front class
        front_class = self.classify(FCF)

        # Front danger confirmation
        if front_class == CLASS_DANGER:
            self._front_danger_run += 1
            self._front_clear_run = 0
        elif front_class == CLASS_CLEAR:
            self._front_danger_run = 0
            self._front_clear_run += 1
        else:
            self._front_danger_run = 0
            self._front_clear_run = 0

        if self._front_danger_run >= c.N_CONFIRM:
            front_class_out = CLASS_DANGER
        else:
            front_class_out = front_class

        # Lateral classes (min(diagonal, forward) with unknown handling)
        left_zone = _min_valid(FDL, FCF)
        right_zone = _min_valid(FDR, FCF)
        left_class = self.classify(left_zone)
        right_class = self.classify(right_zone)

        # Cliff detection (large FCD or -1 vs ground estimate)
        cliff_now = False
        if not _valid(FCD):
            cliff_now = True
        else:
            if FCD > c.T_CLIFF_ABS:
                cliff_now = True
            elif self._ground_est > 0 and FCD > (self._ground_est + c.T_CLIFF_DELTA):
                cliff_now = True

        if cliff_now:
            self._cliff_run += 1
            self._cliff_clear_run = 0
        else:
            self._cliff_run = 0
            self._cliff_clear_run += 1

        cliff = 1 if self._cliff_run >= c.N_CONFIRM else 0

        # Scores and continuous modulation
        # Effective values for unknowns
        FDL_eff = FDL if _valid(FDL) else c.SCORE_DEFAULT
        FDR_eff = FDR if _valid(FDR) else c.SCORE_DEFAULT
        FCF_eff = FCF if _valid(FCF) else c.SCORE_DEFAULT

        left_score = c.W_DIAG * FDL_eff + c.W_CENTER * FCF_eff
        right_score = c.W_DIAG * FDR_eff + c.W_CENTER * FCF_eff

        diff = right_score - left_score
        turn_intensity = math.tanh(diff / max(1.0, c.TURN_SCALE))  # [-1..1], smooth

        if _valid(FCF):
            if FCF > c.T_CLEAR:
                speed_scale = c.SPEED_CLEAR
            elif FCF > c.T_CAUTION:
                speed_scale = c.SPEED_CAUTION
            elif FCF > c.T_NEAR:
                speed_scale = c.SPEED_NEAR
            else:
                speed_scale = c.SPEED_DANGER
        else:
            speed_scale = c.SPEED_CAUTION if True else 0.0  # conservative

        if _valid(FCF):
            tripod_strength = _clamp(FCF / max(1.0, c.TRIPOD_FCF_DEN), c.TRIPOD_MIN, c.TRIPOD_MAX)
        else:
            tripod_strength = 0.6

        # IMU pitch & upright quality
        pitch_deg = _quat_to_pitch_deg(qw, qx, qy, qz)
        upright_quality = math.cos(math.radians(abs(pitch_deg)))
        upright_quality = _clamp(upright_quality, 0.0, 1.0)

        return {
            "ts": ts,
            "front_class": front_class_out,
            "left_class": left_class,
            "right_class": right_class,
            "cliff": cliff,
            "turn_intensity": float(turn_intensity),
            "speed_scale": float(speed_scale),
            "tripod_strength": float(tripod_strength),
            "upright_quality": float(upright_quality),
        }

# ---------------------- Hybrid Decision (FSM-lite) ----------------------

class DecisionEngine:
    """
    Hybrid decision layer:
      - Uses compact fusion outputs to pick a discrete locomotion state.
      - Keeps short dwell times for BACKWARD and ARC to avoid oscillations.
      - Applies a minimal hysteresis on ARC side selection.

    States (numeric):
      0=FORWARD, 1=SLOW_FORWARD, 2=ARC_LEFT, 3=ARC_RIGHT, 4=BACKWARD_RECOVERY, 5=STOP_SAFE
    """

    def __init__(self, dcfg: Optional[DecisionConfig] = None):
        self.cfg = dcfg or DecisionConfig()
        self.state_id: int = STATE_ID["FORWARD"]
        self.state_since: float = time.monotonic()
        self.last_arc: Optional[int] = None  # STATE_ID["ARC_LEFT"] or ["ARC_RIGHT"]

        # Timing control
        self.t_until: Optional[float] = None

    def _enter(self, new_state_id: int, duration_s: Optional[float] = None):
        self.state_id = new_state_id
        self.state_since = time.monotonic()
        self.t_until = (self.state_since + duration_s) if duration_s else None
        if new_state_id in (STATE_ID["ARC_LEFT"], STATE_ID["ARC_RIGHT"]):
            self.last_arc = new_state_id

    def _timed_out(self) -> bool:
        return (self.t_until is not None) and (time.monotonic() >= self.t_until)

    def update(self, fusion: Dict[str, Any], *, watchdog_ok: bool = True) -> int:
        """
        Decide the locomotion state from fusion features.
        Returns state_id (int).
        """
        if not watchdog_ok:
            self._enter(STATE_ID["STOP_SAFE"], None)
            return self.state_id

        cfg = self.cfg
        front_class = int(fusion["front_class"])
        left_class  = int(fusion["left_class"])
        right_class = int(fusion["right_class"])
        cliff       = int(fusion["cliff"])
        turn_intensity = float(fusion["turn_intensity"])
        upright_quality = float(fusion["upright_quality"])

        # 1) Safety hard-override
        if cliff == 1:
            # Stop first, then go to backward recovery with dwell
            self._enter(STATE_ID["BACKWARD_RECOVERY"], cfg.T_BACKWARD)
            return self.state_id

        # 2) Handle timed states (BACKWARD, ARC)
        if self.state_id == STATE_ID["BACKWARD_RECOVERY"]:
            if not self._timed_out():
                return self.state_id
            # After backward, pick ARC side by sign of turn_intensity
            if turn_intensity >= 0.0:
                self._enter(STATE_ID["ARC_LEFT"], cfg.T_ARC_MIN)
            else:
                self._enter(STATE_ID["ARC_RIGHT"], cfg.T_ARC_MIN)
            return self.state_id

        if self.state_id in (STATE_ID["ARC_LEFT"], STATE_ID["ARC_RIGHT"]):
            if not self._timed_out():
                # During ARC dwell, if danger gets worse (front becomes DANGER), re-enter BACKWARD
                if front_class == CLASS_DANGER:
                    self._enter(STATE_ID["BACKWARD_RECOVERY"], cfg.T_BACKWARD)
                return self.state_id
            # ARC dwell complete: decide whether to continue ARC or go FORWARD/SLOW
            # If sides are still tight (NEAR/DANGER), keep ARC (extend dwell).
            if (left_class in (CLASS_NEAR, CLASS_DANGER)) or (right_class in (CLASS_NEAR, CLASS_DANGER)):
                # Minimal hysteresis: keep same side unless opposite is clearly better
                if self.last_arc == STATE_ID["ARC_LEFT"] and turn_intensity < -0.2:
                    self._enter(STATE_ID["ARC_RIGHT"], cfg.T_ARC_MIN)
                elif self.last_arc == STATE_ID["ARC_RIGHT"] and turn_intensity > 0.2:
                    self._enter(STATE_ID["ARC_LEFT"], cfg.T_ARC_MIN)
                else:
                    self._enter(self.state_id, cfg.T_ARC_MIN)
                return self.state_id
            # Otherwise, leave ARC
            # If front is NEAR (but not DANGER) and upright, go SLOW; else FORWARD
            if front_class == CLASS_NEAR and upright_quality >= cfg.MIN_UPRIGHT_4_SLOW:
                self._enter(STATE_ID["SLOW_FORWARD"], None)
            else:
                self._enter(STATE_ID["FORWARD"], None)
            return self.state_id

        # 3) Baseline transitions (FORWARD / SLOW_FORWARD)
        # If front is DANGER: go BACKWARD
        if front_class == CLASS_DANGER:
            self._enter(STATE_ID["BACKWARD_RECOVERY"], cfg.T_BACKWARD)
            return self.state_id

        # Lateral hazards: if left/right is DANGER or NEAR, go ARC with dwell
        if (left_class in (CLASS_NEAR, CLASS_DANGER)) or (right_class in (CLASS_NEAR, CLASS_DANGER)):
            if turn_intensity >= 0.0:
                self._enter(STATE_ID["ARC_LEFT"], cfg.T_ARC_MIN)
            else:
                self._enter(STATE_ID["ARC_RIGHT"], cfg.T_ARC_MIN)
            return self.state_id

        # If front is NEAR and upright enough -> SLOW_FORWARD; else FORWARD
        if front_class == CLASS_NEAR and upright_quality >= cfg.MIN_UPRIGHT_4_SLOW:
            self._enter(STATE_ID["SLOW_FORWARD"], None)
            return self.state_id

        self._enter(STATE_ID["FORWARD"], None)
        return self.state_id


# ---------------------- Top-level system wrapper ----------------------

class RHexSystem:
    """
    Compose FusionEngine and DecisionEngine to produce the final array output.
    - Call update(frame, watchdog_ok=True) per incoming frame.
    - Returns fixed-length numeric list (see FINAL_FIELDS).
    """

    def __init__(self, fcfg: Optional[FusionConfig] = None, dcfg: Optional[DecisionConfig] = None):
        self.fusion = FusionEngine(fcfg)
        self.decision = DecisionEngine(dcfg)

    def update(self, frame: Dict[str, Any], *, watchdog_ok: bool = True) -> List[float]:
        # 1) Fusion
        f = self.fusion.update(frame)
        # 2) Decision
        state_id = self.decision.update(f, watchdog_ok=watchdog_ok)
        # 3) Pack final numeric array
        return [
            int(f["ts"]),
            int(state_id),
            float(f["turn_intensity"]),
            float(f["speed_scale"]),
            float(f["tripod_strength"]),
            int(f["front_class"]),
            int(f["left_class"]),
            int(f["right_class"]),
            int(f["cliff"]),
            float(f["upright_quality"]),
        ]


