#!/usr/bin/env python3
"""
Phase 5 — Log Analysis Tool
Reads a CSV produced by Phase 0 telemetry (full_gait_test.py)
and computes the same metrics as sim_terrain.py.

Usage:
    python analyze_run_log.py ~/hexapod_run_<timestamp>.csv
"""

import sys, csv, math, pathlib, collections

# ── Limits (mirrors sim_terrain.py pass criteria) ────────────────────────────
STALL_THRESHOLD   = 750
STALL_HYSTERESIS  = 3      # frames to set stall
STALL_EXIT_FRAMES = 10
MAX_EXIT_SNAP_STS = 600
MAX_CLOCK_DRIFT   = 0.05   # cycles
MAX_STALL_FRACTION = 0.20  # 20%
VELOCITY_SCALAR   = 1.85
KP_PHASE_RANGE    = (10.0, 14.0)
BLEND             = 0.03

ALL_SERVOS = [1, 2, 3, 4, 5, 6]
LEFT_SERVOS  = [2, 3, 4]
RIGHT_SERVOS = [1, 5, 6]


def load_csv(path):
    rows = []
    with open(path, newline='', encoding='utf-8', errors='replace') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def parse_float(v, default=0.0):
    try: return float(v)
    except: return default

def parse_int(v, default=0):
    try: return int(v)
    except: return default


def compute_metrics(rows):
    """
    Returns a dict of all metrics computed from telemetry rows.
    """
    # Group by (frame, sid)
    by_frame_sid = collections.defaultdict(dict)
    frames_seen  = set()
    sids_seen    = set()

    for row in rows:
        frame = parse_int(row.get('frame', -1))
        sid   = parse_int(row.get('sid',   -1))
        by_frame_sid[frame][sid] = row
        frames_seen.add(frame)
        sids_seen.add(sid)

    frames_sorted = sorted(frames_seen)
    n_frames      = len(frames_sorted)

    # ── Per-servo accumulators ──
    stall_frames   = {s: 0   for s in ALL_SERVOS}   # total frames with stall_active=1
    stall_events   = {s: 0   for s in ALL_SERVOS}   # stall entry events
    exit_snap_max  = {s: 0.0 for s in ALL_SERVOS}
    prev_stalled   = {s: False for s in ALL_SERVOS}
    hysteresis_lag = {s: []  for s in ALL_SERVOS}    # lag in frames from load > thresh to stall_active

    # For stall hysteresis verification: track when load first crossed threshold
    load_cross_frame = {s: None for s in ALL_SERVOS}

    # Zone boundary delta
    prev_dps   = {s: None for s in ALL_SERVOS}
    prev_zone  = {s: None for s in ALL_SERVOS}
    zone_bd    = {s: []   for s in ALL_SERVOS}  # deltas at zone transitions

    # Clock drift
    clock_drifts = []

    # Governor headroom
    gov_headrooms = []

    # Gait hz tracking
    hz_by_sid = {s: [] for s in ALL_SERVOS}

    for frame in frames_sorted:
        frame_data = by_frame_sid[frame]

        # Clock drift: read from any servo row (all share same master_time)
        if frame_data:
            any_row = next(iter(frame_data.values()))
            mtL = parse_float(any_row.get('master_time_L', '0'))
            mtR = parse_float(any_row.get('master_time_R', '0'))
            drift = abs((mtL - mtR + 0.5) % 1.0 - 0.5)
            clock_drifts.append(drift)

        for sid in ALL_SERVOS:
            row = frame_data.get(sid)
            if row is None:
                continue

            stall_active  = parse_int(row.get('stall_active', '0'))
            stall_counter = parse_int(row.get('stall_counter', '0'))
            load          = parse_int(row.get('load', '0'))
            cmd_dps       = parse_float(row.get('cmd_dps', '0'))
            zone          = parse_int(row.get('zone', '0'))
            hz            = parse_float(row.get('hz', '0'))
            phase         = parse_float(row.get('phase', '0'))

            # Stall fraction
            if stall_active:
                stall_frames[sid] += 1

            # Stall events + exit snap
            currently_stalled = stall_active == 1
            was_stalled       = prev_stalled[sid]

            if currently_stalled and not was_stalled:
                stall_events[sid] += 1

            if was_stalled and not currently_stalled:
                # First non-stalled frame: cmd_dps × VELOCITY_SCALAR ≈ STS speed
                snap_sts = abs(cmd_dps) * VELOCITY_SCALAR
                if snap_sts > exit_snap_max[sid]:
                    exit_snap_max[sid] = snap_sts

            # Stall hysteresis verification:
            # count frames from load first crossing threshold to stall_active going 1
            if load > STALL_THRESHOLD and load_cross_frame[sid] is None and not currently_stalled:
                load_cross_frame[sid] = frame
            if currently_stalled and not was_stalled and load_cross_frame[sid] is not None:
                lag = frame - load_cross_frame[sid]
                hysteresis_lag[sid].append(lag)
                load_cross_frame[sid] = None
            if load <= STALL_THRESHOLD:
                load_cross_frame[sid] = None

            # Zone boundary delta (deg/s per frame, at zone transitions only)
            if prev_dps[sid] is not None and prev_zone[sid] is not None:
                if zone != prev_zone[sid]:   # zone changed this frame
                    delta = abs(cmd_dps - prev_dps[sid])
                    zone_bd[sid].append(delta)

            prev_dps[sid]     = cmd_dps
            prev_zone[sid]    = zone
            prev_stalled[sid] = currently_stalled
            hz_by_sid[sid].append(hz)

    # ── Governor headroom ──
    # Estimate from first row with hz data
    # gov formula: (2800/1.85 * (1-duty)) / max(5, |air_sweep| * pi/2)
    # We don't have duty/sweep logged directly, but we can bound hz vs governor
    # by checking if any hz > 0.47 appears (pre-clip)
    all_hz = [h for lst in hz_by_sid.values() for h in lst if h > 0]
    max_hz = max(all_hz) if all_hz else 0.0
    # Conservative: if max_hz <= 0.47, governor was respected
    gov_ok = max_hz <= 0.47

    # ── Aggregate ──
    total_servo_frames = n_frames * len(ALL_SERVOS)
    # Actual logged frames only cover stance/stall rows (air rows skipped per Phase 0 spec).
    # Use stall_frames directly; denominator is approximate.
    total_stall_f   = sum(stall_frames.values())
    stall_fraction  = total_stall_f / max(1, total_servo_frames)

    max_snap   = max(exit_snap_max.values())
    max_drift  = max(clock_drifts) if clock_drifts else 0.0
    mean_drift = sum(clock_drifts) / len(clock_drifts) if clock_drifts else 0.0

    # Hysteresis check: all observed lags must be exactly 3 frames
    all_lags    = [l for lst in hysteresis_lag.values() for l in lst]
    hyst_ok     = all(l == STALL_HYSTERESIS for l in all_lags) if all_lags else True
    bad_lags    = [l for l in all_lags if l != STALL_HYSTERESIS]

    # Zone boundary deltas
    all_zbd  = [d for lst in zone_bd.values() for d in lst]
    max_zbd  = max(all_zbd) if all_zbd else 0.0
    mean_zbd = sum(all_zbd) / len(all_zbd) if all_zbd else 0.0

    return {
        'n_frames':         n_frames,
        'n_rows':           len(rows),
        'sids_found':       sorted(sids_seen),
        'stall_fraction':   stall_fraction,
        'stall_events':     dict(stall_events),
        'stall_by_servo':   dict(stall_frames),
        'max_exit_snap_sts': max_snap,
        'exit_snap_by_servo': {s: f"{exit_snap_max[s]:.1f}" for s in ALL_SERVOS},
        'max_clock_drift':  max_drift,
        'mean_clock_drift': mean_drift,
        'gov_ok':           gov_ok,
        'max_hz_observed':  max_hz,
        'hyst_ok':          hyst_ok,
        'hyst_lags':        sorted(set(all_lags)),
        'bad_hyst_lags':    sorted(set(bad_lags)),
        'max_zone_delta_dps': max_zbd,
        'mean_zone_delta_dps': mean_zbd,
        'n_zone_transitions': len(all_zbd),
    }


def pass_fail(m):
    """Map metric dict to pass/fail table (same format as sim_terrain.evaluate())."""
    checks = []

    # Stall fraction
    sf_ok = m['stall_fraction'] <= MAX_STALL_FRACTION
    checks.append(('Stall fraction',
                   f"{m['stall_fraction']*100:.2f}% (limit 20%)",
                   sf_ok))

    # Exit snap
    snap_ok = m['max_exit_snap_sts'] <= MAX_EXIT_SNAP_STS
    checks.append(('Max exit snap',
                   f"{m['max_exit_snap_sts']:.1f} STS (limit {MAX_EXIT_SNAP_STS})",
                   snap_ok))

    # Clock drift
    drift_ok = m['max_clock_drift'] <= MAX_CLOCK_DRIFT
    checks.append(('Max clock drift',
                   f"{m['max_clock_drift']:.5f} cycles (limit {MAX_CLOCK_DRIFT})",
                   drift_ok))

    # Governor
    checks.append(('Governor (max hz <= 0.47)',
                   f"max observed hz = {m['max_hz_observed']:.4f}",
                   m['gov_ok']))

    # Stall hysteresis
    if m['hyst_lags']:
        hyst_detail = f"lags seen: {m['hyst_lags']} (expected [3])"
    else:
        hyst_detail = "no stall events to verify"
    checks.append(('Stall hysteresis (3-frame set)',
                   hyst_detail,
                   m['hyst_ok']))

    # Zone boundary smoothness
    zbd_ok = m['max_zone_delta_dps'] < 200.0 or m['n_zone_transitions'] == 0
    checks.append(('Zone boundary delta < 200 deg/s',
                   f"max={m['max_zone_delta_dps']:.1f}, mean={m['mean_zone_delta_dps']:.1f}, "
                   f"n={m['n_zone_transitions']}",
                   zbd_ok))

    return checks


def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_run_log.py <hexapod_run_*.csv>")
        sys.exit(1)

    path = pathlib.Path(sys.argv[1]).expanduser()
    if not path.exists():
        print(f"ERROR: {path} not found")
        sys.exit(1)

    print()
    print("=" * 70)
    print("  PHASE 5 — RUN LOG ANALYSIS")
    print(f"  File: {path}")
    print("=" * 70)

    rows = load_csv(path)
    if not rows:
        print("  ERROR: CSV is empty")
        sys.exit(1)

    print(f"\n  Loaded {len(rows)} rows from {path.name}")
    m = compute_metrics(rows)

    print(f"\n  Run summary:")
    print(f"    Frames logged:      {m['n_frames']}")
    print(f"    CSV rows:           {m['n_rows']}")
    print(f"    Servo IDs found:    {m['sids_found']}")
    print(f"    Total stall events: {sum(m['stall_events'].values())}")
    print(f"    Stall by servo:     " +
          " ".join(f"S{k}={v}" for k,v in sorted(m['stall_events'].items())))

    print(f"\n  PASS/FAIL TABLE:")
    print("-" * 70)
    all_ok = True
    for label, detail, ok in pass_fail(m):
        status = "PASS" if ok else "FAIL"
        if not ok:
            all_ok = False
        pad = '.' * max(1, 40 - len(label))
        print(f"  {label} {pad} [{status}]  {detail}")

    print()
    print("  STALL EXIT SNAP BY SERVO:")
    for sid in ALL_SERVOS:
        snap = float(m['exit_snap_by_servo'][sid])
        flag = " <-- EXCEEDS LIMIT" if snap > MAX_EXIT_SNAP_STS else ""
        print(f"    S{sid}: {snap:.1f} STS{flag}")

    if m['bad_hyst_lags']:
        print(f"\n  WARNING: non-3-frame stall hysteresis lags detected: {m['bad_hyst_lags']}")
        print("    This indicates a stall was triggered with wrong counter timing.")
        print("    Check that stall_counters decrement by 1 (not reset to 0) on clear.")

    print()
    print("=" * 70)
    print(f"  PHASE 5 RESULT: {'ALL METRICS WITHIN SIM-VALIDATED BOUNDS' if all_ok else 'METRICS OUT OF BOUNDS — REVIEW REQUIRED'}")
    print("=" * 70)

    # Sim comparison hint
    print()
    print("  Sim reference (sim_terrain.py T8 worst-case):")
    print("    stall_fraction <= 0.20  |  max_exit_snap <= 600 STS")
    print("    gov_headroom >= 0  |  stall hysteresis = 3 frames")
    print("  Compare hardware CSV metrics above against these bounds.")


if __name__ == "__main__":
    main()
