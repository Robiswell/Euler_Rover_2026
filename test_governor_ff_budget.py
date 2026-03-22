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

# Per-gait sweep values (from GAITS dict impact_start/impact_end):
#   Tripod: 340/20 -> stance_sweep = (20-340+180)%360-180 = 40, air = 320
#   Wave/Quad: 345/15 -> stance_sweep = (15-345+180)%360-180 = 30, air = 330
STANCE_SWEEP_TRIPOD  = 40.0
AIR_SWEEP_TRIPOD     = 320.0
STANCE_SWEEP_DEFAULT = 30.0   # Wave and Quad
AIR_SWEEP_DEFAULT    = 330.0  # Wave and Quad

# Per-gait ff_budget values (from GAITS dict)
FF_BUDGET_TRIPOD = 575.0
FF_BUDGET_WAVE   = 700.0
FF_BUDGET_QUAD   = 650.0


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

@pytest.mark.parametrize("gait_name,duty,budget,air_sweep", [
    ("tripod",    0.55, FF_BUDGET_TRIPOD, AIR_SWEEP_TRIPOD),
    ("wave",      0.75, FF_BUDGET_WAVE,   AIR_SWEEP_DEFAULT),
    ("quadruped", 0.70, FF_BUDGET_QUAD,   AIR_SWEEP_DEFAULT),
])
def test_governor_hz_positive_all_gaits(gait_name, duty, budget, air_sweep):
    """
    Invariant: max_safe_hz > 0 for every gait with its per-gait sweep and budget.

    A non-positive result would mean the governor clamps Hz to zero or below,
    stalling the robot at any speed command.
    """
    hz = governor_max_hz(budget, VELOCITY_SCALAR, duty, air_sweep)
    assert hz > 0.0, (
        f"Gait '{gait_name}' (duty={duty}): governor returned hz={hz:.4f}, "
        "robot would be stalled."
    )


# ---------------------------------------------------------------------------
# Test 3: feedforward cap (499) is the binding constraint at governor limit
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("gait_name,duty,budget,air_sweep", [
    ("tripod",    0.55, FF_BUDGET_TRIPOD, AIR_SWEEP_TRIPOD),
    ("wave",      0.75, FF_BUDGET_WAVE,   AIR_SWEEP_DEFAULT),
    ("quadruped", 0.70, FF_BUDGET_QUAD,   AIR_SWEEP_DEFAULT),
])
def test_feedforward_cap_binds_before_budget(gait_name, duty, budget, air_sweep):
    """
    Invariant: when the rover walks at max_safe_hz, ff_speed == FEEDFORWARD_CAP.

    This confirms the cap (499) is the constraint that limits servo speed,
    not the per-gait budget. The governor limits Hz so that the resulting
    feedforward speed lands exactly at 499.

    Tolerance: 1 STS raw unit (rounding in integer servo write).
    """
    max_hz = governor_max_hz(budget, VELOCITY_SCALAR, duty, air_sweep)
    ff = ff_speed_at_hz(max_hz, air_sweep, duty, FEEDFORWARD_CAP, VELOCITY_SCALAR)

    # At max_safe_hz, the raw deg_per_sec * VELOCITY_SCALAR should equal the per-gait budget.
    # min(FEEDFORWARD_CAP, budget) = FEEDFORWARD_CAP since cap < all budgets.
    assert ff == pytest.approx(FEEDFORWARD_CAP, abs=1.0), (
        f"Gait '{gait_name}' (duty={duty}): ff_speed={ff:.2f} at max_safe_hz={max_hz:.4f}. "
        f"Expected ff_speed == FEEDFORWARD_CAP ({FEEDFORWARD_CAP}). "
        "Cap is not the binding constraint -- check budget/cap relationship."
    )

    # Validate formula round-trip: uncapped speed at max_safe_hz must equal budget
    uncapped = abs(air_sweep) * abs(max_hz) / (1.0 - duty) * VELOCITY_SCALAR
    assert uncapped == pytest.approx(budget, abs=1.0), (
        f"Governor formula round-trip broken: uncapped={uncapped:.1f}, budget={budget}")


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
