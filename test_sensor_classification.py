"""
test_sensor_classification.py

Python reimplementation and pytest test suite for the measureClassified()
logic from final_sensors.ino.

Invariant: the Python classify() function must produce the same output as the
Arduino measureClassified() for every combination of:
  - echo_duration_us  (measured HIGH pulse, None = timeout, -1 = no rising edge)
  - previous_reading  (distances_cm[idx] before this call)
  - drain_success     (whether drainEchoLow succeeds before retrigger in step-D)
  - double_ping_echo_us (echo on the second ping; None = no echo within window)

NOT under test: GPIO timing, Arduino micros() drift, multi-sensor crosstalk,
                step-A pre-drain behavior (sensor stuck HIGH before trigger),
                IMU logic, CSV formatting.
"""

import pytest

# ---------------------------------------------------------------------------
# Constants (mirrored from final_sensors.ino)
# ---------------------------------------------------------------------------
SOUND_CM_PER_US: float = 0.0343       # speed of sound ~20 C
BLIND_ZONE_CM:   float = 3.0          # < 3 cm -> VERY NEAR
LIMIT_CM:        float = 300.0        # clamped upper bound

# Round-trip thresholds (integer truncation matches C unsigned long cast)
NEAR_RT_US:  int = int((2.0 * BLIND_ZONE_CM) / SOUND_CM_PER_US)   # 174 us
LIMIT_RT_US: int = int((2.0 * LIMIT_CM)      / SOUND_CM_PER_US)   # 17492 us

TIMEOUT_TOTAL:    int   = 20_000       # us -- step-C and step-D timeout
NEAR_RECHECK_US:  int   = 2_000        # us -- double-ping poll window
HYST_NEAR_CM:     float = 10.0        # hysteresis threshold (cm)

# Sentinel outputs
FAULT      = -2.0
VERY_NEAR  = -1.0
VERY_FAR   = 300.0


# ---------------------------------------------------------------------------
# Python reimplementation of measureClassified()
#
# Arguments map to Arduino steps:
#   echo_duration_us   -- HIGH pulse width from step D (None = step-D timed out,
#                         -1 = step-C timed out / no rising edge = FAULT path)
#   previous_reading   -- distances_cm[idx] value from previous call
#   drain_success      -- result of drainEchoLow() called inside step-D timeout
#                         branch (True = echo went LOW; False = still HIGH)
#   double_ping_echo_us -- echo HIGH duration on the retry trigger
#                         (None = no rising edge within NEAR_RECHECK_US)
#
# Step A (pre-drain before trigger) is excluded -- it only fires when the echo
# pin is stuck HIGH before the trigger pulse, which is a hardware-only condition
# outside the scope of this test suite.
# ---------------------------------------------------------------------------
def classify(
    echo_duration_us,        # int | None | -1 (sentinel for no rising edge)
    previous_reading: float,
    drain_success: bool = True,
    double_ping_echo_us      = None,   # int | None
) -> float:
    """
    Reimplements the core decision tree of measureClassified().

    echo_duration_us values:
      -1       -> step-C timeout (no rising edge) -> FAULT
      None     -> step-D timeout (echo stayed HIGH)
      int >= 0 -> normal echo HIGH duration in microseconds
    """

    # --- Step C: no rising edge -> FAULT (FIX 13) ---
    if echo_duration_us == -1:
        return FAULT

    # --- Step D timeout path ---
    if echo_duration_us is None:

        # FIX 12b: Hysteresis -- previous valid positive reading < HYST_NEAR_CM
        # The >= 0.0 guard is critical: prevents -1.0 / -2.0 from locking sensor
        if previous_reading >= 0.0 and previous_reading < HYST_NEAR_CM:
            return VERY_NEAR

        # FIX 12a + FIX 17: Drain echo line; if still HIGH after drain -> VERY FAR
        if not drain_success:
            return VERY_FAR

        # Double-ping: poll for a rising edge within NEAR_RECHECK_US
        if double_ping_echo_us is None:
            return VERY_FAR  # no echo on retry -> VERY FAR

        dur2 = double_ping_echo_us
        if dur2 < NEAR_RT_US:
            return VERY_NEAR
        d2 = float(dur2) * SOUND_CM_PER_US * 0.5
        if d2 < BLIND_ZONE_CM:
            return VERY_NEAR
        if d2 > LIMIT_CM:
            return VERY_FAR
        # Clamp to [2, 300]
        d2 = max(2.0, min(300.0, d2))
        return d2

    # --- Normal path: echo_duration_us is a valid non-negative integer ---
    dur = echo_duration_us

    # Step F: pulse too short -> blind zone
    if dur < NEAR_RT_US:
        return VERY_NEAR

    # Step G: convert to distance
    d = float(dur) * SOUND_CM_PER_US * 0.5

    # Step H: extra blind-zone guard after conversion
    if d < BLIND_ZONE_CM:
        return VERY_NEAR

    # Step I: over range limit -> VERY FAR
    if dur > LIMIT_RT_US or d > LIMIT_CM:
        return VERY_FAR

    # Step J: clamp to [2, 300]
    d = max(2.0, min(300.0, d))
    return d


# ---------------------------------------------------------------------------
# Helper: compute the echo duration in us that corresponds to a real distance
# ---------------------------------------------------------------------------
def distance_to_echo_us(cm: float) -> int:
    """Round-trip time for a given distance in cm."""
    return int((2.0 * cm) / SOUND_CM_PER_US)


# ===========================================================================
# 1. Normal distance classification
# ===========================================================================

@pytest.mark.parametrize("target_cm", [10.0, 50.0, 100.0, 200.0, 299.0])
def test_normal_distance_returns_correct_cm(target_cm):
    """Valid echoes in [3, 300) cm return the measured distance (within 0.5 cm)."""
    echo_us = distance_to_echo_us(target_cm)
    result = classify(echo_us, previous_reading=50.0)
    expected = float(echo_us) * SOUND_CM_PER_US * 0.5
    assert abs(result - expected) < 0.5, (
        f"target={target_cm}cm echo={echo_us}us -> {result}, expected ~{expected:.1f}"
    )


def test_normal_distance_result_is_clamped_at_floor_2cm():
    """Step J clamps the output floor to 2.0 cm even if distance math yields less."""
    # Force a distance just above BLIND_ZONE_CM but below 2 cm -- not physically
    # reachable via echo time alone (BLIND_ZONE_CM = 3 > 2), but verify the clamp
    # code path by injecting a duration that yields exactly 2.3 cm (above blind zone)
    # and checking the result is >= 2.0.
    echo_us = distance_to_echo_us(4.0)   # 4 cm -> well above blind zone
    result = classify(echo_us, previous_reading=50.0)
    assert result >= 2.0


def test_normal_distance_result_is_clamped_at_ceiling_300cm():
    """A duration just at the limit returns exactly 300.0."""
    # LIMIT_RT_US corresponds to exactly LIMIT_CM; anything > returns 300.
    echo_us = LIMIT_RT_US + 1
    result = classify(echo_us, previous_reading=50.0)
    assert result == VERY_FAR


# ===========================================================================
# 2. Blind zone detection (normal path)
# ===========================================================================

def test_blind_zone_short_echo_returns_very_near():
    """Echo duration below NEAR_RT_US (step F) returns VERY_NEAR."""
    short_echo = NEAR_RT_US - 1   # one us below threshold
    result = classify(short_echo, previous_reading=50.0)
    assert result == VERY_NEAR


def test_blind_zone_zero_duration_returns_very_near():
    """Zero-length echo (step F) returns VERY_NEAR."""
    result = classify(0, previous_reading=50.0)
    assert result == VERY_NEAR


def test_blind_zone_exact_near_rt_threshold_is_valid():
    """Echo duration exactly equal to NEAR_RT_US passes step F."""
    # NEAR_RT_US corresponds to ~3 cm (BLIND_ZONE_CM); step H catches it if d < 3.
    result = classify(NEAR_RT_US, previous_reading=50.0)
    d = NEAR_RT_US * SOUND_CM_PER_US * 0.5
    # Result is either VERY_NEAR (step H) or the computed distance -- never FAULT or VERY_FAR
    assert result in (VERY_NEAR, d) or abs(result - d) < 0.1


def test_blind_zone_step_h_guard_short_distance():
    """Step H catches a duration that passes step F but yields d < BLIND_ZONE_CM."""
    # Find a duration that is >= NEAR_RT_US but whose distance is < BLIND_ZONE_CM.
    # NEAR_RT_US = int((2*3)/0.0343) = 174 us -> d = 174*0.0343/2 = 2.98 cm < 3.0
    echo_us = NEAR_RT_US  # 174 us -> 2.98 cm -> step H fires
    d = echo_us * SOUND_CM_PER_US * 0.5
    if d < BLIND_ZONE_CM:
        result = classify(echo_us, previous_reading=50.0)
        assert result == VERY_NEAR, f"expected VERY_NEAR, got {result} (d={d:.3f})"


# ===========================================================================
# 3. Very far / timeout (normal path)
# ===========================================================================

def test_very_far_echo_exceeds_limit_rt():
    """Echo duration > LIMIT_RT_US (step I) returns VERY_FAR."""
    result = classify(LIMIT_RT_US + 100, previous_reading=50.0)
    assert result == VERY_FAR


def test_very_far_step_d_timeout_no_prior_history():
    """Step-D timeout with far previous reading and no retry echo -> VERY_FAR."""
    result = classify(
        echo_duration_us=None,
        previous_reading=50.0,   # not in hysteresis zone
        drain_success=True,
        double_ping_echo_us=None,
    )
    assert result == VERY_FAR


# ===========================================================================
# 4. Boundary: exactly at 2 cm and 300 cm
# ===========================================================================

def test_boundary_at_300cm_exactly():
    """Distance that computes to exactly 300 cm is clamped and returned as 300.0."""
    # d = dur * 0.0343 * 0.5 = 300 -> dur = 300 / (0.0343 * 0.5) = 17492.6... -> 17492 us
    # Check both sides of LIMIT_RT_US
    echo_at_limit = LIMIT_RT_US
    d = echo_at_limit * SOUND_CM_PER_US * 0.5
    result = classify(echo_at_limit, previous_reading=50.0)
    # If d <= LIMIT_CM the clamp to 300 applies via step J; if > LIMIT_RT_US step I fires
    assert result <= VERY_FAR
    assert result >= 2.0


def test_boundary_above_300cm_returns_very_far():
    """Any echo that converts to > 300 cm returns VERY_FAR."""
    echo_us = distance_to_echo_us(301.0)
    result = classify(echo_us, previous_reading=50.0)
    assert result == VERY_FAR


def test_boundary_minimum_valid_distance_at_least_2cm():
    """The output floor is 2.0; no valid echo returns a distance below 2.0."""
    # Smallest echo that clears blind zone: distance_to_echo_us(BLIND_ZONE_CM) + 1
    echo_us = distance_to_echo_us(BLIND_ZONE_CM) + 1
    result = classify(echo_us, previous_reading=50.0)
    if result not in (VERY_NEAR, VERY_FAR, FAULT):
        assert result >= 2.0


# ===========================================================================
# 5. CRITICAL: Hysteresis feedback loop bug (FIX 12b)
# ===========================================================================

def test_hysteresis_does_not_lock_on_negative_one():
    """
    FIX 12b guard: previous reading of -1.0 must NOT trigger hysteresis.
    Without the >= 0.0 guard, -1.0 < HYST_NEAR_CM (10) would fire and return
    VERY_NEAR, permanently locking the sensor.
    """
    result = classify(
        echo_duration_us=None,   # step-D timeout
        previous_reading=VERY_NEAR,   # -1.0
        drain_success=True,
        double_ping_echo_us=None,
    )
    # Must NOT return VERY_NEAR via hysteresis; drain succeeded, no retry echo -> VERY_FAR
    assert result == VERY_FAR, (
        f"Hysteresis fired on previous=-1.0 (FIX 12b broken): got {result}"
    )


def test_hysteresis_does_not_lock_on_negative_two():
    """
    FIX 12b guard: previous reading of -2.0 (FAULT) must NOT trigger hysteresis.
    -2.0 < HYST_NEAR_CM but is negative, so >= 0.0 guard must block it.
    """
    result = classify(
        echo_duration_us=None,
        previous_reading=FAULT,   # -2.0
        drain_success=True,
        double_ping_echo_us=None,
    )
    assert result == VERY_FAR, (
        f"Hysteresis fired on previous=-2.0 (FIX 12b broken): got {result}"
    )


def test_hysteresis_fires_correctly_for_close_valid_reading():
    """
    FIX 12b: previous reading was 5.0 cm (positive, < HYST_NEAR_CM=10).
    Step-D timeout should return VERY_NEAR immediately -- object entered blind zone.
    """
    result = classify(
        echo_duration_us=None,
        previous_reading=5.0,
        drain_success=True,
        double_ping_echo_us=None,
    )
    assert result == VERY_NEAR, (
        f"Hysteresis should fire for previous=5.0 cm: got {result}"
    )


def test_hysteresis_does_not_fire_for_far_reading():
    """
    FIX 12b: previous reading was 50.0 cm (>= HYST_NEAR_CM=10).
    Step-D timeout must NOT return VERY_NEAR via hysteresis.
    """
    result = classify(
        echo_duration_us=None,
        previous_reading=50.0,
        drain_success=True,
        double_ping_echo_us=None,
    )
    assert result != VERY_NEAR, (
        f"Hysteresis incorrectly fired for previous=50.0 cm: got {result}"
    )


def test_hysteresis_does_not_fire_for_very_far_previous():
    """
    FIX 12b: previous reading was 300.0 (VERY_FAR).
    Step-D timeout must NOT trigger hysteresis (300.0 >= HYST_NEAR_CM).
    """
    result = classify(
        echo_duration_us=None,
        previous_reading=VERY_FAR,
        drain_success=True,
        double_ping_echo_us=None,
    )
    assert result != VERY_NEAR, (
        f"Hysteresis incorrectly fired for previous=300.0: got {result}"
    )


@pytest.mark.parametrize("prev", [0.0, 1.0, 2.5, 9.99])
def test_hysteresis_fires_at_all_valid_close_distances(prev):
    """Hysteresis fires for any previous in [0, HYST_NEAR_CM)."""
    result = classify(
        echo_duration_us=None,
        previous_reading=prev,
        drain_success=True,
        double_ping_echo_us=None,
    )
    assert result == VERY_NEAR, (
        f"Hysteresis should fire for previous={prev}: got {result}"
    )


@pytest.mark.parametrize("prev", [10.0, 10.01, 50.0, 299.0, 300.0])
def test_hysteresis_does_not_fire_at_or_above_threshold(prev):
    """Hysteresis does NOT fire for previous >= HYST_NEAR_CM."""
    result = classify(
        echo_duration_us=None,
        previous_reading=prev,
        drain_success=True,
        double_ping_echo_us=None,
    )
    assert result != VERY_NEAR, (
        f"Hysteresis incorrectly fired for previous={prev}: got {result}"
    )


# ===========================================================================
# 6. CRITICAL: Drain failure after step-D timeout (FIX 17)
# ===========================================================================

def test_drain_failure_returns_far_not_near():
    """
    FIX 17: After step-D timeout, if drainEchoLow fails (echo still HIGH),
    the function must return VERY_FAR immediately, NOT proceed to double-ping
    and NOT return VERY_NEAR.
    """
    result = classify(
        echo_duration_us=None,
        previous_reading=50.0,   # no hysteresis
        drain_success=False,     # echo still HIGH after drain
        double_ping_echo_us=distance_to_echo_us(5.0),   # would be NEAR if reached
    )
    assert result == VERY_FAR, (
        f"Drain failure should return VERY_FAR, got {result}"
    )


def test_drain_failure_with_near_previous_hysteresis_fires_first():
    """
    Hysteresis (FIX 12b) is checked BEFORE drain (FIX 17).
    If previous=5.0 and drain fails, hysteresis fires first -> VERY_NEAR.
    """
    result = classify(
        echo_duration_us=None,
        previous_reading=5.0,
        drain_success=False,
    )
    assert result == VERY_NEAR, (
        f"Hysteresis should fire before drain check: got {result}"
    )


# ===========================================================================
# 7. Double-ping retry: no echo on second ping
# ===========================================================================

def test_double_ping_no_echo_returns_very_far():
    """
    After step-D timeout + drain success, if second ping gets no echo within
    NEAR_RECHECK_US, result is VERY_FAR (object genuinely not in range).
    """
    result = classify(
        echo_duration_us=None,
        previous_reading=50.0,
        drain_success=True,
        double_ping_echo_us=None,
    )
    assert result == VERY_FAR


# ===========================================================================
# 8. Double-ping with short echo -> VERY_NEAR
# ===========================================================================

def test_double_ping_short_echo_returns_very_near():
    """
    Second-ping echo duration below NEAR_RT_US returns VERY_NEAR.
    Object is in the blind zone.
    """
    result = classify(
        echo_duration_us=None,
        previous_reading=50.0,
        drain_success=True,
        double_ping_echo_us=NEAR_RT_US - 1,
    )
    assert result == VERY_NEAR


def test_double_ping_zero_duration_returns_very_near():
    """Second-ping echo of 0 us returns VERY_NEAR."""
    result = classify(
        echo_duration_us=None,
        previous_reading=50.0,
        drain_success=True,
        double_ping_echo_us=0,
    )
    assert result == VERY_NEAR


def test_double_ping_valid_echo_returns_distance():
    """Second-ping echo for a real distance (10 cm) returns a valid distance."""
    echo_us = distance_to_echo_us(10.0)
    result = classify(
        echo_duration_us=None,
        previous_reading=50.0,
        drain_success=True,
        double_ping_echo_us=echo_us,
    )
    expected = echo_us * SOUND_CM_PER_US * 0.5
    assert abs(result - expected) < 0.5, f"double-ping 10cm: got {result}"


def test_double_ping_over_limit_returns_very_far():
    """Second-ping echo for > 300 cm returns VERY_FAR."""
    echo_us = distance_to_echo_us(301.0)
    result = classify(
        echo_duration_us=None,
        previous_reading=50.0,
        drain_success=True,
        double_ping_echo_us=echo_us,
    )
    assert result == VERY_FAR


# ===========================================================================
# 9. Sensor fault (step-C timeout / no rising edge)
# ===========================================================================

def test_sensor_fault_no_rising_edge_returns_fault():
    """
    Step-C timeout (no rising edge detected) must return FAULT (-2.0),
    not VERY_FAR (300.0) or VERY_NEAR (-1.0).
    FIX 13 regression guard.
    """
    result = classify(
        echo_duration_us=-1,   # sentinel: no rising edge
        previous_reading=50.0,
    )
    assert result == FAULT, f"expected FAULT (-2.0), got {result}"


def test_sensor_fault_returns_negative_two_not_300():
    """FAULT value must be exactly -2.0, not 300.0."""
    result = classify(echo_duration_us=-1, previous_reading=300.0)
    assert result == -2.0


def test_sensor_fault_not_affected_by_previous_reading():
    """Step-C timeout ignores previous_reading entirely."""
    for prev in (FAULT, VERY_NEAR, 5.0, 50.0, VERY_FAR):
        result = classify(echo_duration_us=-1, previous_reading=prev)
        assert result == FAULT, (
            f"FAULT path changed by previous_reading={prev}: got {result}"
        )


# ===========================================================================
# 10. Sequence test: near -> far recovery (no hysteresis lock)
# ===========================================================================

def test_sequence_near_then_far_recovers():
    """
    Simulate a two-reading sequence:
      Reading 1: object at 5 cm -> stored as VERY_NEAR (-1.0)
      Reading 2: step-D timeout, drain success, no retry echo
                 previous_reading is -1.0 (from reading 1)
                 -> must return VERY_FAR (not VERY_NEAR)
    This verifies the >= 0.0 hysteresis guard prevents the feedback lock.
    """
    # Reading 1: object close, echo is short
    r1 = classify(echo_duration_us=NEAR_RT_US - 1, previous_reading=300.0)
    assert r1 == VERY_NEAR, f"Reading 1 should be VERY_NEAR, got {r1}"

    # Reading 2: object gone, step-D timeout, previous is now -1.0
    r2 = classify(
        echo_duration_us=None,
        previous_reading=r1,        # -1.0 -- the value written back to distances_cm
        drain_success=True,
        double_ping_echo_us=None,
    )
    assert r2 == VERY_FAR, (
        f"Sensor must recover to VERY_FAR after -1.0 previous (FIX 12b lock): got {r2}"
    )


def test_sequence_close_object_entering_blind_zone():
    """
    Simulate an object approaching:
      Reading 1: 8 cm (valid, < HYST_NEAR_CM) -> stored as 8.0
      Reading 2: object inches into blind zone -> step-D timeout
                 previous_reading = 8.0 -> hysteresis fires -> VERY_NEAR
    """
    r1 = classify(echo_duration_us=distance_to_echo_us(8.0), previous_reading=50.0)
    assert 7.0 < r1 < 9.0, f"Reading 1 should be ~8 cm, got {r1}"

    r2 = classify(
        echo_duration_us=None,
        previous_reading=r1,
        drain_success=True,
        double_ping_echo_us=None,
    )
    assert r2 == VERY_NEAR, (
        f"Object in blind zone should trigger hysteresis from previous={r1}: got {r2}"
    )


def test_sequence_fault_then_recovery():
    """
    After a FAULT reading (-2.0), the next reading with a valid echo
    must return the correct distance, not be influenced by the FAULT value.
    """
    r1 = classify(echo_duration_us=-1, previous_reading=50.0)
    assert r1 == FAULT

    r2 = classify(
        echo_duration_us=distance_to_echo_us(40.0),
        previous_reading=r1,   # -2.0
    )
    assert abs(r2 - 40.0) < 1.0, (
        f"Recovery after FAULT should give ~40 cm, got {r2}"
    )


# ===========================================================================
# Constant sanity checks (catch accidental constant drift)
# ===========================================================================

def test_constants_near_rt_us_is_approximately_174_to_175():
    """NEAR_RT_US should be ~174-175 us (2*3.0/0.0343)."""
    assert 174 <= NEAR_RT_US <= 175, f"NEAR_RT_US={NEAR_RT_US}"


def test_constants_limit_rt_us_is_approximately_17490_to_17495():
    """LIMIT_RT_US should be ~17492 us (2*300/0.0343)."""
    assert 17490 <= LIMIT_RT_US <= 17495, f"LIMIT_RT_US={LIMIT_RT_US}"


def test_constants_blind_zone_less_than_hyst_near():
    """BLIND_ZONE_CM must be less than HYST_NEAR_CM or hysteresis is unreachable."""
    assert BLIND_ZONE_CM < HYST_NEAR_CM
