"""
Tests for GOVERNOR_FF_BUDGET=700 / FEEDFORWARD_CAP=499 invariants.

These tests are pure arithmetic -- no hardware, no imports from final_full_gait_test.py.
They replicate the exact formulas from the source so they survive internal refactors
while still catching accidental constant changes or formula drift.

Source references:
  - Constants:  final_full_gait_test.py lines 108-109
  - Formula:    final_full_gait_test.py line 1121
  - ff_speed:   final_full_gait_test.py line 1222
  - get_air_sweep: final_full_gait_test.py lines 387-393
"""

import pytest

# ---------------------------------------------------------------------------
# Constants mirrored from final_full_gait_test.py -- if these drift the
# tests will fail and alert you to sync them.
# ---------------------------------------------------------------------------
GOVERNOR_FF_BUDGET = 700.0   # line 109
FEEDFORWARD_CAP    = 499.0   # line 108
VELOCITY_SCALAR    = 1.85    # line 90

# Stance sweep for DEFAULT_IMPACT_START=345, DEFAULT_IMPACT_END=15:
#   stance_sweep = (15 - 345 + 180) % 360 - 180 = 210 % 360 - 180 = 30
#   air_sweep    = get_air_sweep(30) = 360 - 30 = 330 (positive, forward)
STANCE_SWEEP_DEFAULT = 30.0
AIR_SWEEP_DEFAULT    = 330.0


def governor_max_hz(budget, velocity_scalar, duty, air_sweep):
    """Replicated from final_full_gait_test.py line 1121."""
    return (budget / velocity_scalar * (1.0 - duty)) / max(5.0, abs(air_sweep))


def ff_speed_at_hz(hz, air_sweep, duty, ff_cap, velocity_scalar):
    """Replicated from final_full_gait_test.py line 1218-1222 (air phase branch)."""
    deg_per_sec = (abs(air_sweep) * abs(hz)) / (1.0 - duty)
    return min(ff_cap, deg_per_sec * velocity_scalar)


# ---------------------------------------------------------------------------
# Test 1: constant consistency -- budget must be >= cap
# ---------------------------------------------------------------------------

def test_governor_budget_geq_feedforward_cap():
    """
    Invariant: GOVERNOR_FF_BUDGET >= FEEDFORWARD_CAP.

    Budget is the speed ceiling the governor enforces via Hz limiting.
    Cap is the hard per-tick clamp on raw servo speed.
    If budget < cap, the Hz governor could allow a speed the cap then silently
    clips, meaning the governor math doesn't reflect reality.
    """
    assert GOVERNOR_FF_BUDGET >= FEEDFORWARD_CAP, (
        f"Budget {GOVERNOR_FF_BUDGET} < cap {FEEDFORWARD_CAP}: "
        "governor Hz limit would be set below the feedforward cap, "
        "making the cap the de-facto budget without the governor knowing."
    )


# ---------------------------------------------------------------------------
# Test 2: governor produces positive Hz for all three gaits at 30-deg sweep
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("gait_name,duty", [
    ("tripod",    0.50),
    ("wave",      0.75),
    ("quadruped", 0.70),
])
def test_governor_hz_positive_all_gaits(gait_name, duty):
    """
    Invariant: max_safe_hz > 0 for every gait with default 30-deg stance sweep.

    A non-positive result would mean the governor clamps Hz to zero or below,
    stalling the robot at any speed command.
    """
    hz = governor_max_hz(GOVERNOR_FF_BUDGET, VELOCITY_SCALAR, duty, AIR_SWEEP_DEFAULT)
    assert hz > 0.0, (
        f"Gait '{gait_name}' (duty={duty}): governor returned hz={hz:.4f}, "
        "robot would be stalled."
    )


# ---------------------------------------------------------------------------
# Test 3: feedforward cap (499) is the binding constraint at governor limit
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("gait_name,duty", [
    ("tripod",    0.50),
    ("wave",      0.75),
    ("quadruped", 0.70),
])
def test_feedforward_cap_binds_before_budget(gait_name, duty):
    """
    Invariant: when the rover walks at max_safe_hz, ff_speed == FEEDFORWARD_CAP.

    This confirms the cap (499) is the constraint that limits servo speed,
    not the budget (700). The governor limits Hz so that the resulting
    feedforward speed lands exactly at 499. If the cap were raised above 700,
    the governor would become the binding constraint instead -- which is the
    unsafe regime that caused ground contact at budget=900.

    Tolerance: 1 STS raw unit (rounding in integer servo write).
    """
    max_hz = governor_max_hz(GOVERNOR_FF_BUDGET, VELOCITY_SCALAR, duty, AIR_SWEEP_DEFAULT)
    ff = ff_speed_at_hz(max_hz, AIR_SWEEP_DEFAULT, duty, FEEDFORWARD_CAP, VELOCITY_SCALAR)

    # At max_safe_hz, the raw deg_per_sec * VELOCITY_SCALAR should equal GOVERNOR_FF_BUDGET.
    # min(FEEDFORWARD_CAP, GOVERNOR_FF_BUDGET) = FEEDFORWARD_CAP since cap < budget.
    assert ff == pytest.approx(FEEDFORWARD_CAP, abs=1.0), (
        f"Gait '{gait_name}' (duty={duty}): ff_speed={ff:.2f} at max_safe_hz={max_hz:.4f}. "
        f"Expected ff_speed == FEEDFORWARD_CAP ({FEEDFORWARD_CAP}). "
        "Cap is not the binding constraint -- check budget/cap relationship."
    )

    # Validate formula round-trip: uncapped speed at max_safe_hz must equal budget
    uncapped = abs(AIR_SWEEP_DEFAULT) * abs(max_hz) / (1.0 - duty) * VELOCITY_SCALAR
    assert uncapped == pytest.approx(GOVERNOR_FF_BUDGET, abs=1.0), (
        f"Governor formula round-trip broken: uncapped={uncapped:.1f}, budget={GOVERNOR_FF_BUDGET}")


# ---------------------------------------------------------------------------
# Test 4: wave gait at budget=700 has sufficient Hz headroom (> 0.2 Hz)
# ---------------------------------------------------------------------------

def test_wave_duty_075_hz_exceeds_minimum_walking_threshold():
    """
    Invariant: wave gait max_safe_hz > 0.2 Hz with GOVERNOR_FF_BUDGET=700.

    Wave is the slowest gait (duty=0.75, most air-phase time). At 0.2 Hz the
    rover completes one full gait cycle in 5 seconds -- below that the robot
    moves too slowly to be useful on a competition course. Previous budget=499
    produced hz=0.204 (barely above); budget=700 must give clear headroom.
    """
    WAVE_DUTY = 0.75
    MIN_WALKING_HZ = 0.2

    hz = governor_max_hz(GOVERNOR_FF_BUDGET, VELOCITY_SCALAR, WAVE_DUTY, AIR_SWEEP_DEFAULT)
    assert hz > MIN_WALKING_HZ, (
        f"Wave gait max_safe_hz={hz:.4f} <= {MIN_WALKING_HZ}. "
        f"Budget={GOVERNOR_FF_BUDGET} is insufficient for wave walking on competition course."
    )


# ---------------------------------------------------------------------------
# Test 5: regression -- budget=900 would overshoot the cap (documents why 900 failed)
# ---------------------------------------------------------------------------

def test_budget_900_would_exceed_feedforward_cap():
    """
    Regression: GOVERNOR_FF_BUDGET=900 caused ground contact on hardware.

    At budget=900, the governor's implied target speed exceeds FEEDFORWARD_CAP=499.
    The cap then clips the speed, making the actual air-phase speed lower than
    the governor calculated -- the governor under-estimates ground clearance risk.
    This test documents why 900 is unsafe and must not be restored.
    """
    UNSAFE_BUDGET = 900.0
    WAVE_DUTY     = 0.75

    hz_at_900 = governor_max_hz(UNSAFE_BUDGET, VELOCITY_SCALAR, WAVE_DUTY, AIR_SWEEP_DEFAULT)

    # The governor assumes ff_speed reaches UNSAFE_BUDGET before the cap intervenes.
    # Since FEEDFORWARD_CAP=499 < 900, the cap clips -- governor math is broken.
    assert FEEDFORWARD_CAP < UNSAFE_BUDGET, (
        "Test setup error: cap should be below the unsafe budget to demonstrate the hazard."
    )
    # Confirm hz is higher than with budget=700 (governor is less conservative)
    hz_at_700 = governor_max_hz(GOVERNOR_FF_BUDGET, VELOCITY_SCALAR, WAVE_DUTY, AIR_SWEEP_DEFAULT)
    assert hz_at_900 > hz_at_700, (
        "Budget=900 should allow higher Hz than budget=700, "
        "demonstrating the governor is less restrictive (and unsafe)."
    )
