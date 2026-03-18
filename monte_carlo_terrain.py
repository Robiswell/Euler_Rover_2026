#!/usr/bin/env python3
"""
Phase 3 — Extended Terrain Monte Carlo
Runs sim_terrain scenarios T1-T10 across 500 seeds each using multiprocessing.
Reports pass rate, 95th-percentile stall_fraction, 95th-percentile max_exit_snap,
and worst-case gov_headroom_hz per scenario.
"""

import sys, csv, math, datetime, importlib.util, pathlib, multiprocessing as mp

_SPEC = importlib.util.spec_from_file_location(
    "sim_terrain",
    str(pathlib.Path(__file__).parent / "sim_terrain.py"),
)

N_SEEDS  = 500
PARALLEL = max(1, mp.cpu_count() - 1)

# Pass-rate thresholds by scenario group
PASS_RATE_THRESHOLDS = {
    'T1':  0.99,
    'T2':  0.95,
    'T3':  0.90,
    'T4':  0.90,
    'T5':  0.90,
    'T6':  0.90,
    'T7':  0.85,
    'T8':  0.85,
    'T9':  0.85,
    'T10': 0.85,
}


def _worker_init():
    """Re-import sim_terrain in the subprocess so constants are fresh."""
    global sim
    spec = importlib.util.spec_from_file_location(
        "sim_terrain",
        str(pathlib.Path(__file__).parent / "sim_terrain.py"),
    )
    sim = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sim)


def _run_one(args):
    """Run a single (scenario_kwargs, seed) pair and return (pass, stall_fraction, max_exit_snap, gov_headroom)."""
    sc_kwargs, seed = args
    kw = dict(sc_kwargs)
    kw['seed'] = seed
    name = kw.get('name', '')

    if name.startswith("T9"):
        r = sim.run_t9_hysteresis()
    else:
        r = sim.run_scenario(**kw)

    fails = sim.evaluate(r)
    passed = len(fails) == 0
    return (
        passed,
        r.get('stall_fraction', 0.0),
        r.get('max_exit_snap', 0.0),
        r.get('gov_headroom_hz', float('inf')),
    )


def percentile(data, p):
    """Return the p-th percentile of sorted data (0-100)."""
    if not data:
        return float('nan')
    s = sorted(data)
    idx = (len(s) - 1) * p / 100.0
    lo  = int(idx)
    hi  = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (idx - lo)


def main():
    # Load sim_terrain in main process for scenario definitions
    spec = importlib.util.spec_from_file_location(
        "sim_terrain",
        str(pathlib.Path(__file__).parent / "sim_terrain.py"),
    )
    sim_main = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sim_main)

    scenarios = sim_main.define_scenarios()
    date_str  = datetime.date.today().strftime('%Y%m%d')
    out_path  = f"monte_carlo_{date_str}.csv"

    total_cells = len(scenarios) * N_SEEDS
    print()
    print("=" * 70)
    print(f"  PHASE 3 — TERRAIN MONTE CARLO  ({N_SEEDS} seeds × {len(scenarios)} scenarios)")
    print(f"  {total_cells} total runs  |  {PARALLEL} parallel workers")
    print("=" * 70)

    # Build work list
    work = []
    for sc in scenarios:
        for seed in range(N_SEEDS):
            work.append((sc, seed))

    # Run in parallel pool
    results_flat = []
    with mp.Pool(processes=PARALLEL, initializer=_worker_init) as pool:
        for i, res in enumerate(pool.imap_unordered(_run_one, work, chunksize=20)):
            results_flat.append(res)
            if (i + 1) % 500 == 0 or (i + 1) == total_cells:
                print(f"  [{i+1}/{total_cells}] completed", flush=True)

    # Group results by scenario (order may be scrambled by imap_unordered,
    # so we re-run with ordered imap to get stable per-scenario grouping)
    print("  Collecting ordered results ...", flush=True)
    results_ordered = []
    with mp.Pool(processes=PARALLEL, initializer=_worker_init) as pool:
        for res in pool.imap(_run_one, work, chunksize=20):
            results_ordered.append(res)

    csv_rows    = []
    summary     = []
    overall_ok  = True

    for sc_idx, sc in enumerate(scenarios):
        sc_name = sc['name']
        prefix  = sc_name[:2]  # e.g. 'T1'

        # Pull this scenario's results
        offset  = sc_idx * N_SEEDS
        sc_res  = results_ordered[offset: offset + N_SEEDS]

        passes        = [r[0] for r in sc_res]
        stall_fracs   = [r[1] for r in sc_res]
        exit_snaps    = [r[2] for r in sc_res]
        gov_headrooms = [r[3] for r in sc_res]

        pass_rate     = sum(passes) / N_SEEDS
        p95_stall     = percentile(stall_fracs, 95)
        p95_snap      = percentile(exit_snaps,  95)
        worst_gov     = min(h for h in gov_headrooms if h != float('inf')) \
                        if any(h != float('inf') for h in gov_headrooms) else float('inf')

        threshold = PASS_RATE_THRESHOLDS.get(prefix, 0.85)
        sc_ok     = pass_rate >= threshold
        if not sc_ok:
            overall_ok = False

        summary.append({
            'scenario':       sc_name,
            'pass_rate_pct':  f"{pass_rate * 100:.1f}",
            'threshold_pct':  f"{threshold * 100:.0f}",
            'p95_stall_frac': f"{p95_stall:.4f}",
            'p95_exit_snap':  f"{p95_snap:.1f}",
            'worst_gov_hz':   f"{worst_gov:.5f}" if worst_gov != float('inf') else 'inf',
            'result':         'PASS' if sc_ok else 'FAIL',
        })

        # Per-seed CSV rows
        for seed in range(N_SEEDS):
            r = sc_res[seed]
            csv_rows.append({
                'scenario': sc_name,
                'seed':     seed,
                'pass':     'PASS' if r[0] else 'FAIL',
                'stall_fraction': f"{r[1]:.5f}",
                'max_exit_snap':  f"{r[2]:.1f}",
                'gov_headroom_hz': f"{r[3]:.5f}" if r[3] != float('inf') else 'inf',
            })

    # Write CSV
    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=csv_rows[0].keys())
        writer.writeheader()
        writer.writerows(csv_rows)
    print(f"\nCSV written: {out_path}")

    # Print summary table
    print()
    print("=" * 70)
    print("  MONTE CARLO SUMMARY")
    print(f"  {'Scenario':<35} {'Pass%':>6} {'Need%':>6} {'P95Stall':>10} {'P95Snap':>9} {'WorstGov':>10}  Result")
    print("-" * 90)
    for s in summary:
        print(f"  {s['scenario']:<35} {s['pass_rate_pct']:>6} {s['threshold_pct']:>6} "
              f"{s['p95_stall_frac']:>10} {s['p95_exit_snap']:>9} {s['worst_gov_hz']:>10}  {s['result']}")

    print()
    print("=" * 70)
    print(f"  PHASE 3 RESULT: {'ALL SCENARIOS PASS' if overall_ok else 'SOME SCENARIOS FAILED'}")
    print("=" * 70)

if __name__ == "__main__":
    main()
