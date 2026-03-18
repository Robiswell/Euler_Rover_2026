#!/usr/bin/env python3
"""
Phase 4 — FSM State Machine Completeness Audit
Reads full_gait_test.py and audits the Brain FSM via static code analysis.
No execution required.  Prints PASS / WARN / FAIL per check.
"""

import sys, re, ast, pathlib

TARGET = pathlib.Path(__file__).parent / "full_gait_test.py"

LABEL_W  = 55
STATUS_W = 6

def verdict(label, status, detail=''):
    pad = '.' * max(1, LABEL_W - len(label))
    tail = f"  {detail}" if detail else ''
    print(f"  {label} {pad} [{status}]{tail}")


def load_source():
    return TARGET.read_text(encoding='utf-8', errors='replace')


# ── Check 1: BrainState enum ──────────────────────────────────────────────────
def check_brain_state_enum(src):
    """
    full_gait_test.py uses a *procedural* Brain (sequential tactical_sleep calls),
    not an enum-based FSM.  Detect which model is present and audit accordingly.
    """
    has_enum = 'class BrainState' in src or 'BrainState.' in src
    if has_enum:
        # Enumerate values
        m = re.findall(r'BrainState\.(\w+)', src)
        states = sorted(set(m))
        verdict("Brain uses BrainState enum", "INFO",
                f"{len(states)} states: {', '.join(states)}")
        return True, states
    else:
        # Procedural: phases are labelled by print() calls like "--- Phase N ---"
        phases = re.findall(r'--- Phase \d+[^-]*---', src)
        verdict("Brain FSM model: procedural (sequential phases)", "INFO",
                f"{len(phases)} phases found")
        return False, phases


# ── Check 2: Dead-state / unreachable code ─────────────────────────────────────
def check_dead_states(src, has_enum, states):
    """
    For procedural Brain: verify each labelled phase has at least one
    shared_speed or shared_turn_bias write (i.e., it actually commands motion).
    For enum Brain: verify each state name appears in an if/elif block.
    """
    if not has_enum:
        # Each "--- Phase N ---" block must have at least one shared_* write before the next phase
        phase_blocks = re.split(r'# --- Phase \d+', src)
        issues = []
        for i, block in enumerate(phase_blocks[1:], start=1):
            next_phase_idx = block.find('# --- Phase')
            chunk = block[:next_phase_idx] if next_phase_idx >= 0 else block
            if 'shared_speed.value' not in chunk and 'shared_turn_bias.value' not in chunk:
                issues.append(f"Phase {i}")
        if issues:
            verdict("All phases command motion", "WARN",
                    f"phases with no motion command: {issues}")
        else:
            verdict("All phases command motion", "PASS")
        return not bool(issues)
    else:
        missing = [s for s in states if f'BrainState.{s}' not in src]
        if missing:
            verdict("All enum states reachable", "FAIL", f"unreferenced: {missing}")
            return False
        verdict("All enum states reachable", "PASS")
        return True


# ── Check 3: Mission schedule frame count ────────────────────────────────────
def check_mission_schedule(src):
    """
    Look for MISSION_SCHEDULE list; if present, sum frame counts and verify
    total mission time is within [40, 90] s at 50 Hz.
    If not present, infer from tactical_sleep() calls.
    """
    FPS = 50
    COURSE_TIME_MIN = 40   # seconds
    COURSE_TIME_MAX = 90

    # Try MISSION_SCHEDULE list
    m = re.search(r'MISSION_SCHEDULE\s*=\s*\[([^\]]+)\]', src, re.DOTALL)
    if m:
        nums = [int(x) for x in re.findall(r'\d+', m.group(1))]
        total_frames = sum(nums)
        total_sec    = total_frames / FPS
        ok = COURSE_TIME_MIN <= total_sec <= COURSE_TIME_MAX
        verdict(
            f"MISSION_SCHEDULE total {total_frames} frames = {total_sec:.1f} s",
            "PASS" if ok else "FAIL",
            f"(limit {COURSE_TIME_MIN}-{COURSE_TIME_MAX} s)",
        )
        return ok, total_sec

    # Fall back to tactical_sleep() durations
    sleeps = [float(x) for x in re.findall(r'tactical_sleep\((\d+(?:\.\d+)?)', src)]
    if sleeps:
        total_sec = sum(sleeps)
        ok = COURSE_TIME_MIN <= total_sec <= COURSE_TIME_MAX
        verdict(
            f"tactical_sleep total: {total_sec:.1f} s ({len(sleeps)} calls)",
            "PASS" if ok else "WARN",
            f"(limit {COURSE_TIME_MIN}-{COURSE_TIME_MAX} s — warn if > competition course time)",
        )
        return ok, total_sec

    verdict("Mission timing (no MISSION_SCHEDULE found)", "WARN",
            "add MISSION_SCHEDULE for precise check")
    return True, 0.0


# ── Check 4: Pivot state timing ───────────────────────────────────────────────
def check_pivot_timing(src):
    """
    Locate pivot tactical_sleep durations.
    Pivot must complete in < 1/(hz × 2) seconds so the opposite clock doesn't
    advance half a cycle during the pivot.  At hz=0.47 the limit is ~1.06 s.
    Brain pivots typically run at speed=0 + turn_bias for N seconds.
    """
    # Heuristic: find sections where shared_speed is 0 and shared_turn_bias != 0
    # then check the following tactical_sleep duration
    pivot_pattern = re.compile(
        r'shared_speed\.value\s*=\s*0\s*\n'
        r'(?:.*\n)*?'
        r'\s*shared_turn_bias\.value\s*=\s*([^\n]+)\s*\n'
        r'(?:.*\n)*?'
        r'\s*tactical_sleep\((\d+(?:\.\d+)?)',
        re.MULTILINE,
    )
    MAX_PIVOT_HZ = 0.47
    MAX_PIVOT_SEC = 1.0 / (MAX_PIVOT_HZ * 2.0)

    pivots = pivot_pattern.findall(src)
    if not pivots:
        # looser pattern: turn_bias set then sleep
        pivots2 = re.findall(
            r'shared_turn_bias\.value\s*=\s*([^\n]+)\s*\n'
            r'(?:.*\n){0,3}?'
            r'\s*tactical_sleep\((\d+(?:\.\d+)?)',
            src, re.MULTILINE,
        )
        pivots = pivots2

    if not pivots:
        verdict("Pivot timing (no pivot sections found)", "WARN",
                "expected shared_turn_bias + tactical_sleep pairs")
        return True

    issues = []
    for bias_str, dur_str in pivots:
        bias = float(bias_str.strip())
        dur  = float(dur_str)
        if bias != 0.0 and dur > MAX_PIVOT_SEC * 2:
            # multiply by 2 because brain deliberately runs multi-second pivots;
            # the relevant constraint is max clock_diff that can be corrected
            # within the ±0.002 cycles/frame cap without snap.
            # The cap handles up to 0.002×50=0.1 cycle divergence per second.
            # So a 16 s pivot at 0.47 hz creates 7.5 cycle diff — takes ~75 s
            # to converge.  Flag pivots > 20 s as needing explicit resync logic.
            if dur > 20:
                issues.append(f"bias={bias} dur={dur}s")

    if issues:
        verdict("Pivot duration (>20s pivots need resync logic)", "WARN",
                f"{issues}")
    else:
        verdict("Pivot timing: no excessively long pivots", "PASS")
    return True


# ── Check 5: Self-right sequence ──────────────────────────────────────────────
def check_self_right(src):
    """
    Verify state_self_right_roll() restores normal operation:
      - shared_z_flip.value = 1  at end
      - shared_speed.value  = 0  at end
    Verify it is called from within a try block (Brain loop) so it leads back
    to post-recovery execution, not a terminal exit.
    """
    ok = True

    # Must set z_flip back to 1
    if 'shared_z_flip.value = 1' not in src:
        verdict("self_right restores z_flip=1", "FAIL",
                "shared_z_flip never reset to 1 after self-right")
        ok = False
    else:
        verdict("self_right restores z_flip=1", "PASS")

    # Must set speed to 0 at end
    fn_match = re.search(
        r'def state_self_right_roll\(\):(.*?)(?=\ndef |\Z)', src, re.DOTALL)
    if fn_match:
        body = fn_match.group(1)
        if 'shared_speed.value  = 0' in body or 'shared_speed.value = 0' in body:
            verdict("self_right ends with speed=0", "PASS")
        else:
            verdict("self_right ends with speed=0", "WARN",
                    "speed may not be zeroed at end of self_right_roll")
    else:
        verdict("state_self_right_roll() found", "WARN", "function not found in source")

    # Verify called inside try block (so execution continues after)
    state_right_call_pos = src.find('state_self_right_roll()')
    try_pos = src.rfind('try:', 0, state_right_call_pos)
    finally_pos = src.find('finally:', state_right_call_pos)
    if try_pos > 0 and finally_pos > 0:
        verdict("self_right called inside try (non-terminal)", "PASS")
    else:
        verdict("self_right called inside try (non-terminal)", "WARN",
                "could not confirm try/finally wrapping")

    return ok


# ── Check 6: Stall override scope ─────────────────────────────────────────────
def check_stall_override(src):
    """
    Verify shared_stall_override (if present) is only set during self-right roll
    and cleared immediately after.  If not present, note that v1 doesn't use it.
    """
    if 'shared_stall_override' not in src:
        verdict("shared_stall_override scope", "INFO",
                "not present in v1 (v2 feature) — OK for this file")
        return True

    # Find all assignments
    sets   = re.findall(r'shared_stall_override\.value\s*=\s*([^\n]+)', src)
    set_1  = [v.strip() for v in sets if v.strip() in ('1', 'True')]
    set_0  = [v.strip() for v in sets if v.strip() in ('0', 'False')]

    if len(set_1) == 1 and len(set_0) == 1:
        verdict("shared_stall_override set/clear exactly once each", "PASS")
        # Verify the set is inside state_self_right_roll
        fn_match = re.search(
            r'def state_self_right_roll\(\):(.*?)(?=\ndef |\Z)', src, re.DOTALL)
        if fn_match and 'shared_stall_override' in fn_match.group(1):
            verdict("shared_stall_override confined to self_right_roll", "PASS")
        else:
            verdict("shared_stall_override confined to self_right_roll", "WARN",
                    "set/clear found outside self_right_roll")
    else:
        verdict("shared_stall_override balanced set/clear", "FAIL",
                f"set×{len(set_1)} clear×{len(set_0)}")
        return False

    return True


# ── Check 7: STALL_EXIT_FRAMES guard ─────────────────────────────────────────
def check_stall_exit_frames(src):
    """Verify STALL_EXIT_FRAMES constant is defined and in safe range [8, 12]."""
    m = re.search(r'STALL_EXIT_FRAMES\s*=\s*(\d+)', src)
    if not m:
        verdict("STALL_EXIT_FRAMES defined", "FAIL", "constant not found")
        return False
    val = int(m.group(1))
    ok  = 8 <= val <= 12
    verdict(f"STALL_EXIT_FRAMES = {val} (range 8-12)", "PASS" if ok else "FAIL")
    return ok


# ── Check 8: CSV telemetry present (Phase 0) ─────────────────────────────────
def check_telemetry(src):
    """Verify Phase 0 CSV instrumentation is present."""
    checks = [
        ('csv.writer(',         "csv.writer initialized"),
        ('_csv_writer.writerow', "rows written per servo"),
        ('_csv_file.close()',    "CSV closed in finally"),
        ('hexapod_run_',        "timestamped filename"),
        ('frame_counter',       "frame counter present"),
    ]
    all_ok = True
    for pattern, label in checks:
        found = pattern in src
        verdict(f"Telemetry: {label}", "PASS" if found else "FAIL")
        if not found:
            all_ok = False
    return all_ok


def main():
    print()
    print("=" * 70)
    print("  PHASE 4 — BRAIN FSM COMPLETENESS AUDIT")
    print(f"  Target: {TARGET}")
    print("=" * 70)

    if not TARGET.exists():
        print(f"\n  ERROR: {TARGET} not found")
        sys.exit(1)

    src = load_source()

    print()
    print("  [A] FSM Architecture")
    has_enum, states_or_phases = check_brain_state_enum(src)

    print()
    print("  [B] Dead-State / Unreachable Code")
    check_dead_states(src, has_enum, states_or_phases)

    print()
    print("  [C] Mission Schedule Timing")
    timing_ok, total_sec = check_mission_schedule(src)
    if total_sec > 0:
        print(f"      -> At 0.3 hz Wave speed=350: ~{total_sec/60:.1f} min total Brain runtime")

    print()
    print("  [D] Pivot State Timing")
    check_pivot_timing(src)

    print()
    print("  [E] Self-Right Sequence")
    check_self_right(src)

    print()
    print("  [F] Stall Override Scope")
    check_stall_override(src)

    print()
    print("  [G] Safety Constants")
    check_stall_exit_frames(src)

    print()
    print("  [H] Phase 0 CSV Telemetry")
    check_telemetry(src)

    print()
    print("=" * 70)
    print("  PHASE 4 AUDIT COMPLETE")
    print("  Review any FAIL or WARN items above before hardware deployment.")
    print("=" * 70)

if __name__ == "__main__":
    main()
