#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fusion2.py — RHex Rover sensor fusion + hybrid decision FSM (corrected).

INPUT FRAME (dict):
  {
    "timestamp_ms": int,
    "distances": [FDL, FCF, FCD, FDR, RDL, RCF, RCD, RDR],  # cm; -1 = very-near
    "quat": {"w": float, "x": float, "y": float, "z": float},
    "upside_down": 0 or 1,
  }

OUTPUT ARRAY (10 fields — FINAL_FIELDS):
  [ts_ms, state_id, turn_intensity, speed_scale, tripod_strength,
   front_class, left_class, right_class, cliff, upright_quality]

State IDs:
  0 = FORWARD           all clear, full speed
  1 = SLOW_FORWARD      front caution or near
  2 = ARC_LEFT          arc toward the free left side
  3 = ARC_RIGHT         arc toward the free right side
  4 = BACKWARD_RECOVERY front danger or cliff confirmed (dwell 0.8 s)
  5 = STOP_SAFE         watchdog / critical tilt / persistent danger

Distance classes:
  0 = UNKNOWN  (None or other invalid reading)
  1 = CLEAR    (> 60 cm)
  2 = CAUTION  (30-60 cm)
  3 = NEAR     (20–30 cm)
  4 = DANGER   (< 20 cm  OR  -1 very-near sentinel)

Turn-intensity convention  (BUG FIX 1 — sign was reversed in draft):
  diff = left_score − right_score
    positive when LEFT is freer  → right is more blocked → turn_intensity > 0 → turn LEFT ✓
    negative when RIGHT is freer → left  is more blocked → turn_intensity < 0 → turn RIGHT ✓

ARC direction rule  (BUG FIX 2 — was sign-based, now class-based):
  right_class >= left_class  →  ARC_LEFT   (arc toward the freer left  side)
  left_class  >  right_class →  ARC_RIGHT  (arc toward the freer right side)

Cliff sensor (FCD) — BUG FIX 3:
  FCD = -1 (very-near sentinel) means the floor is in the blind zone → NOT a cliff.
  Cliff fires only when FCD is a large positive value above the ground EMA.

Lateral classification — BUG FIX 4:
  Uses diagonal pairs only  (FDL+RDL for left, FDR+RDR for right).
  FCF is NOT mixed into lateral scores (it would cancel out and add no information).

Dwell refresh — BUG FIX 5:
  _enter(..., force_dwell=True) re-extends the timer on every frame where
  cliff or front-DANGER persists, preventing premature exit from safety states.

Thread safety — BUG FIX 6:
  RHexSystem.update() is protected by threading.Lock; safe to call from the
  consumer thread in input_thread.py while other threads read shared state.
"""

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

# ================================================================
# OUTPUT FIELD NAMES
# ================================================================

FINAL_FIELDS: List[str] = [
    "ts_ms",
    "state_id",
    "turn_intensity",
    "speed_scale",
    "tripod_strength",
    "front_class",
    "left_class",
    "right_class",
    "cliff",
    "upright_quality",
]

# ================================================================
# CLASS CONSTANTS
# ================================================================

CLASS_UNKNOWN = 0
CLASS_CLEAR   = 1
CLASS_CAUTION = 2
CLASS_NEAR    = 3
CLASS_DANGER  = 4

CLASS_NAMES: Dict[int, str] = {
    CLASS_UNKNOWN: "UNK",
    CLASS_CLEAR:   "CLEAR",
    CLASS_CAUTION: "CAUTION",
    CLASS_NEAR:    "NEAR",
    CLASS_DANGER:  "DANGER",
}

# ================================================================
# STATE CONSTANTS
# ================================================================

ST_FORWARD           = 0
ST_SLOW_FORWARD      = 1
ST_ARC_LEFT          = 2
ST_ARC_RIGHT         = 3
ST_BACKWARD_RECOVERY = 4
ST_STOP_SAFE         = 5

STATE_NAMES: Dict[int, str] = {
    ST_FORWARD:           "FORWARD",
    ST_SLOW_FORWARD:      "SLOW_FORWARD",
    ST_ARC_LEFT:          "ARC_LEFT",
    ST_ARC_RIGHT:         "ARC_RIGHT",
    ST_BACKWARD_RECOVERY: "BACKWARD_RECOVERY",
    ST_STOP_SAFE:         "STOP_SAFE",
}

# ================================================================
# SENSOR ARRAY INDICES
# ================================================================

_FDL = 0   # front-diagonal-left
_FCF = 1   # front-center-forward
_FCD = 2   # front-center-down  (cliff / ground sensor)
_FDR = 3   # front-diagonal-right
_RDL = 4   # rear-diagonal-left
_RCF = 5   # rear-center-forward
_RCD = 6   # rear-center-down
_RDR = 7   # rear-diagonal-right

# Sentinel produced by the Nano firmware when the echo is too short
# (obstacle in blind zone / no echo / too-early return).
VERY_NEAR_SENTINEL: float = -1.0


# ================================================================
# CONFIGURATION DATACLASSES
# ================================================================

@dataclass
class FusionConfig:
    """Tunable parameters for the sensor-fusion layer."""

    # Distance classification thresholds (cm)
    T_CLEAR:   float = 60.0
    T_CAUTION: float = 30.0
    T_NEAR:    float = 20.0

    # Cliff detection
    T_CLIFF_ABS:       float = 30.0   # absolute FCD threshold for a cliff candidate (cm)
    T_CLIFF_DELTA:     float = 10.0   # FCD delta above ground EMA to declare a candidate (cm)
    N_CONFIRM:         int   = 2      # consecutive candidates required to confirm cliff
    GROUND_MAX_ACCEPT: float = 40.0   # max FCD accepted into the ground EMA (cm)
    GROUND_EMA_ALPHA:  float = 0.2    # ground EMA smoothing factor

    # Turn intensity: tanh(diff / TURN_SCALE); larger → gentler curve
    TURN_SCALE:   float = 50.0
    DEFAULT_DIST: float = 25.0   # assumed distance (cm) when a sensor reads invalid

    # Speed scale by front class
    SPEED_CLEAR:   float = 1.00
    SPEED_CAUTION: float = 0.70
    SPEED_NEAR:    float = 0.45
    SPEED_DANGER:  float = 0.00

    # Tripod strength by locomotion state
    TRIPOD_FORWARD:   float = 0.60
    TRIPOD_SLOW:      float = 0.75
    TRIPOD_ARC:       float = 1.00
    TRIPOD_BACKWARD:  float = 1.00
    TRIPOD_STOP:      float = 0.00

    # Set True to apply front/rear swap inside fusion when upside_down=1.
    # Keep False (default) when input_thread.py already applies the remap.
    APPLY_SWAP: bool = False


@dataclass
class DecisionConfig:
    """Tunable parameters for the hybrid decision FSM."""

    DWELL_BACKWARD:   float = 0.80   # seconds for BACKWARD_RECOVERY
    DWELL_ARC_MIN:    float = 0.60   # minimum seconds for ARC states
    DWELL_ARC_EXTEND: float = 0.40   # extra seconds when ARC side is still blocked

    # Upright quality below this threshold → STOP_SAFE
    UPRIGHT_MIN: float = 0.15


# ================================================================
# PRIVATE HELPERS
# ================================================================

def _is_very_near(d) -> bool:
    """Return True if d is the -1 VERY_NEAR sentinel."""
    return d is not None and float(d) == VERY_NEAR_SENTINEL


def _valid_positive(d) -> bool:
    """Return True only for a finite, non-negative reading (not the -1 sentinel)."""
    return d is not None and float(d) >= 0.0


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


def _classify(d, cfg: "FusionConfig") -> int:
    """
    Classify one distance reading.

    -1.0 (VERY_NEAR)   → CLASS_DANGER   obstacle is in the sensor blind zone
    None / other < 0   → CLASS_UNKNOWN  missing or invalid reading
    0 – T_NEAR         → CLASS_DANGER
    T_NEAR – T_CAUTION → CLASS_NEAR
    T_CAUTION – T_CLEAR→ CLASS_CAUTION
    > T_CLEAR          → CLASS_CLEAR    (includes 300 = very far)
    """
    if d is None:
        return CLASS_UNKNOWN
    d = float(d)
    if d == VERY_NEAR_SENTINEL:   # -1.0 = blind zone = obstacle very close
        return CLASS_DANGER
    if d < 0.0:
        return CLASS_UNKNOWN      # other invalid negatives
    if d > cfg.T_CLEAR:
        return CLASS_CLEAR
    if d > cfg.T_CAUTION:
        return CLASS_CAUTION
    if d > cfg.T_NEAR:
        return CLASS_NEAR
    return CLASS_DANGER


def _sector_worst(readings: list, cfg: "FusionConfig") -> int:
    """Return the worst (highest) class across a list of distance readings."""
    if not readings:
        return CLASS_UNKNOWN
    return max(_classify(d, cfg) for d in readings)


def _eff(d, default: float) -> float:
    """
    Return the distance value for scoring purposes.
    The -1 sentinel and None both map to `default` (treated as near-caution).
    """
    if d is None:
        return default
    d = float(d)
    return default if d < 0.0 else d   # covers -1 sentinel


def _normalize_quat(quat) -> tuple:
    """Return (w, x, y, z) from a dict {'w':..} or a tuple/list."""
    if isinstance(quat, dict):
        return (
            float(quat.get("w", 1.0)),
            float(quat.get("x", 0.0)),
            float(quat.get("y", 0.0)),
            float(quat.get("z", 0.0)),
        )
    if quat is None:
        return (1.0, 0.0, 0.0, 0.0)
    q = list(quat)
    while len(q) < 4:
        q.append(0.0)
    return tuple(float(v) for v in q[:4])


def _pitch_deg(w: float, x: float, y: float, z: float) -> float:
    """Pitch in degrees from unit quaternion: pitch = asin(clamp(2*(w*y − z*x)))."""
    sinp = _clamp(2.0 * (w * y - z * x), -1.0, 1.0)
    return math.degrees(math.asin(sinp))


def remap_preserve_left_right(distances: List[float]) -> List[float]:
    """
    Front/rear swap that preserves body-frame left/right laterality.

    Used when the rover is operating inverted (belly-up): the rear sensors
    become the active front while left stays left and right stays right.

    Input:  [FDL, FCF, FCD, FDR,  RDL, RCF, RCD, RDR]
    Output: [RDL, RCF, RCD, RDR,  FDL, FCF, FCD, FDR]

    Raises ValueError if the list does not contain exactly 8 elements.
    """
    if len(distances) != 8:
        raise ValueError(f"Expected 8 distances, got {len(distances)}.")
    FDL, FCF, FCD, FDR, RDL, RCF, RCD, RDR = distances
    return [RDL, RCF, RCD, RDR, FDL, FCF, FCD, FDR]


# ================================================================
# FUSION ENGINE
# ================================================================

class FusionEngine:
    """
    Stateful sensor fusion.

    Produces per-frame:
      front_class, left_class, right_class  — obstacle classes
      cliff                                  — 0 / 1 confirmed cliff
      turn_intensity                         — [-1, +1]
      speed_scale                            — [0, 1]
      upright_quality                        — [0, 1]
    """

    def __init__(self, cfg: Optional[FusionConfig] = None):
        self._cfg = cfg or FusionConfig()
        self._ground_ema:       Optional[float] = None
        self._cliff_run:        int = 0
        self._front_danger_run: int = 0

    def reset(self) -> None:
        """Reset all stateful components."""
        self._ground_ema       = None
        self._cliff_run        = 0
        self._front_danger_run = 0

    def classify(self, d) -> int:
        """Public single-value classifier."""
        return _classify(d, self._cfg)

    def update(self, frame: Dict[str, Any]) -> Dict[str, Any]:
        cfg  = self._cfg
        ts   = int(frame.get("timestamp_ms", 0))
        raw  = frame.get("distances", [])
        upsd = bool(frame.get("upside_down", 0))
        quat = _normalize_quat(frame.get("quat"))

        dist = list(raw)
        while len(dist) < 8:
            dist.append(None)
        dist = dist[:8]

        if cfg.APPLY_SWAP and upsd:
            dist = dist[4:8] + dist[0:4]

        FDL, FCF, FCD, FDR, RDL, RCF, RCD, RDR = dist

        # ---- 1. GROUND EMA ------------------------------------------------
        if _valid_positive(FCD) and float(FCD) <= cfg.GROUND_MAX_ACCEPT:
            v = float(FCD)
            self._ground_ema = (
                v if self._ground_ema is None
                else cfg.GROUND_EMA_ALPHA * v + (1.0 - cfg.GROUND_EMA_ALPHA) * self._ground_ema
            )

        # ---- 2. CLIFF DETECTION -------------------------------------------
        if _is_very_near(FCD):
            self._cliff_run = 0
        elif not _valid_positive(FCD):
            if self._ground_ema is not None:
                self._cliff_run += 1
        else:
            fcd_v     = float(FCD)
            candidate = (
                fcd_v > cfg.T_CLIFF_ABS
                or (self._ground_ema is not None
                    and fcd_v > self._ground_ema + cfg.T_CLIFF_DELTA)
            )
            if candidate:
                self._cliff_run += 1
            else:
                self._cliff_run = 0

        cliff = 1 if self._cliff_run >= cfg.N_CONFIRM else 0

        # ---- 3. SECTOR CLASSIFICATION -------------------------------------
        front_raw = _sector_worst([FDL, FCF, FDR], cfg)

        if front_raw == CLASS_DANGER:
            self._front_danger_run += 1
        else:
            self._front_danger_run = 0

        front_class = (
            CLASS_DANGER if self._front_danger_run >= cfg.N_CONFIRM else front_raw
        )

        left_class  = _sector_worst([FDL, RDL], cfg)
        right_class = _sector_worst([FDR, RDR], cfg)

        # ---- 4. TURN INTENSITY --------------------------------------------
        dfl  = _eff(FDL, cfg.DEFAULT_DIST)
        dfr  = _eff(FDR, cfg.DEFAULT_DIST)
        diff = dfl - dfr
        turn_intensity = math.tanh(diff / max(1.0, cfg.TURN_SCALE))

        # ---- 5. SPEED SCALE -----------------------------------------------
        _speed = {
            CLASS_CLEAR:   cfg.SPEED_CLEAR,
            CLASS_CAUTION: cfg.SPEED_CAUTION,
            CLASS_NEAR:    cfg.SPEED_NEAR,
            CLASS_DANGER:  cfg.SPEED_DANGER,
            CLASS_UNKNOWN: cfg.SPEED_CAUTION,
        }
        speed_scale = _speed.get(front_class, cfg.SPEED_CAUTION)

        # ---- 6. UPRIGHT QUALITY ------------------------------------------
        w, x, y, z = quat
        pitch        = _pitch_deg(w, x, y, z)
        upright_quality = _clamp(math.cos(math.radians(abs(pitch))), 0.0, 1.0)

        return {
            "ts":              ts,
            "front_class":     front_class,
            "left_class":      left_class,
            "right_class":     right_class,
            "cliff":           cliff,
            "turn_intensity":  float(turn_intensity),
            "speed_scale":     float(speed_scale),
            "upright_quality": float(upright_quality),
        }


# ================================================================
# DECISION ENGINE
# ================================================================

class DecisionEngine:
    """
    Hybrid FSM that selects the rover's locomotion state.

    Safety hierarchy (highest → lowest priority):
      watchdog failure / critical tilt  →  STOP_SAFE
      cliff OR front DANGER             →  BACKWARD_RECOVERY  (dwell refreshed)
      lateral NEAR / DANGER             →  ARC_LEFT / ARC_RIGHT  (class-based)
      front NEAR or CAUTION             →  SLOW_FORWARD
      all clear                         →  FORWARD

    ARC direction is always CLASS-BASED (BUG FIX 2):
      right_class >= left_class  →  ARC_LEFT   (left side is at least as free)
      left_class  >  right_class →  ARC_RIGHT  (right side is freer)

    Dwell refresh (BUG FIX 5):
      _enter(..., force_dwell=True) re-extends the backward/arc timer on
      every frame where the triggering condition is still present, preventing
      the rover from leaving BACKWARD_RECOVERY while danger persists.
    """

    def __init__(self, dcfg: Optional[DecisionConfig] = None):
        self._cfg       = dcfg or DecisionConfig()
        self._state     = ST_FORWARD
        self._dwell_end = 0.0
        self._last_arc  = ST_ARC_LEFT

    def reset(self) -> None:
        self._state     = ST_FORWARD
        self._dwell_end = 0.0

    def update(
        self,
        fusion: Dict[str, Any],
        *,
        watchdog_ok: bool = True,
    ) -> int:
        cfg = self._cfg

        front_class     = int(fusion["front_class"])
        left_class      = int(fusion["left_class"])
        right_class     = int(fusion["right_class"])
        cliff           = int(fusion["cliff"])
        upright_quality = float(fusion["upright_quality"])

        now      = time.monotonic()
        in_dwell = now < self._dwell_end

        if not watchdog_ok:
            return self._enter(ST_STOP_SAFE)

        if upright_quality < cfg.UPRIGHT_MIN:
            return self._enter(ST_STOP_SAFE)

        if cliff or front_class == CLASS_DANGER:
            return self._enter(
                ST_BACKWARD_RECOVERY, cfg.DWELL_BACKWARD, force_dwell=True
            )

        if in_dwell:
            if self._state == ST_BACKWARD_RECOVERY:
                return self._state

            if self._state in (ST_ARC_LEFT, ST_ARC_RIGHT):
                if not self._arc_is_correct(left_class, right_class):
                    return self._enter_arc(
                        left_class, right_class, cfg.DWELL_ARC_EXTEND
                    )
                return self._state

        if self._state == ST_BACKWARD_RECOVERY and not in_dwell:
            return self._enter_arc(left_class, right_class, cfg.DWELL_ARC_MIN)

        if self._state in (ST_ARC_LEFT, ST_ARC_RIGHT) and not in_dwell:
            lateral_blocked = (
                left_class  in (CLASS_NEAR, CLASS_DANGER)
                or right_class in (CLASS_NEAR, CLASS_DANGER)
            )
            if lateral_blocked:
                if not self._arc_is_correct(left_class, right_class):
                    return self._enter_arc(
                        left_class, right_class, cfg.DWELL_ARC_EXTEND
                    )
                return self._enter(self._state, cfg.DWELL_ARC_EXTEND)

            if front_class in (CLASS_NEAR, CLASS_CAUTION):
                return self._enter(ST_SLOW_FORWARD)
            return self._enter(ST_FORWARD)

        if (left_class  in (CLASS_NEAR, CLASS_DANGER)
                or right_class in (CLASS_NEAR, CLASS_DANGER)):
            return self._enter_arc(left_class, right_class, cfg.DWELL_ARC_MIN)

        if front_class in (CLASS_NEAR, CLASS_CAUTION):
            return self._enter(ST_SLOW_FORWARD)

        return self._enter(ST_FORWARD)

    def _enter(
        self,
        new_state: int,
        dwell_secs: float = 0.0,
        *,
        force_dwell: bool = False,
    ) -> int:
        changing = new_state != self._state
        if changing:
            self._state = new_state
        if (changing or force_dwell) and dwell_secs > 0:
            self._dwell_end = time.monotonic() + dwell_secs
        elif changing:
            self._dwell_end = 0.0
        if new_state in (ST_ARC_LEFT, ST_ARC_RIGHT):
            self._last_arc = new_state
        return self._state

    def _enter_arc(self, left_class: int, right_class: int, dwell: float) -> int:
        arc = ST_ARC_LEFT if right_class >= left_class else ST_ARC_RIGHT
        return self._enter(arc, dwell)

    def _arc_is_correct(self, left_class: int, right_class: int) -> bool:
        if self._state == ST_ARC_LEFT:
            return right_class >= left_class
        if self._state == ST_ARC_RIGHT:
            return left_class > right_class
        return True


# ================================================================
# SYSTEM WRAPPER  (thread-safe)
# ================================================================

class RHexSystem:
    """
    Thread-safe wrapper combining FusionEngine + DecisionEngine.

    Usage
    -----
    system = RHexSystem(fcfg=FusionConfig(APPLY_SWAP=False), dcfg=DecisionConfig())
    arr    = system.update(frame, watchdog_ok=True)   # returns list[10]
    """

    def __init__(
        self,
        fcfg: Optional[FusionConfig] = None,
        dcfg: Optional[DecisionConfig] = None,
    ):
        self.fusion   = FusionEngine(fcfg)
        self.decision = DecisionEngine(dcfg)
        self._lock    = threading.Lock()

    def update(
        self,
        frame: Dict[str, Any],
        *,
        watchdog_ok: bool = True,
    ) -> List:
        with self._lock:
            f        = self.fusion.update(frame)
            state_id = self.decision.update(f, watchdog_ok=watchdog_ok)
            cfg      = self.fusion._cfg

            _tripod = {
                ST_FORWARD:           cfg.TRIPOD_FORWARD,
                ST_SLOW_FORWARD:      cfg.TRIPOD_SLOW,
                ST_ARC_LEFT:          cfg.TRIPOD_ARC,
                ST_ARC_RIGHT:         cfg.TRIPOD_ARC,
                ST_BACKWARD_RECOVERY: cfg.TRIPOD_BACKWARD,
                ST_STOP_SAFE:         cfg.TRIPOD_STOP,
            }
            tripod = _tripod.get(state_id, 0.6)

        return [
            int(f["ts"]),
            int(state_id),
            float(f["turn_intensity"]),
            float(f["speed_scale"]),
            float(tripod),
            int(f["front_class"]),
            int(f["left_class"]),
            int(f["right_class"]),
            int(f["cliff"]),
            float(f["upright_quality"]),
        ]

    def reset(self) -> None:
        """Reset all internal state (EMA, counters, FSM dwell)."""
        with self._lock:
            self.fusion.reset()
            self.decision.reset()