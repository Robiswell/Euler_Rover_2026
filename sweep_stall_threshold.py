#!/usr/bin/env python3
"""
Phase 1 — Parameter Sweep: STALL_THRESHOLD Sensitivity
Calls sim_terrain.run_scenario() across a grid of threshold values,
terrain types, gaits, and seeds.  Writes results to CSV and prints summary.
"""

import sys, io, csv, math, datetime, random, importlib, types

# ── Import sim_terrain without executing main() ───────────────────────────────
import importlib.util, pathlib
_spec = importlib.util.spec_from_file_location(
    "sim_terrain",
    str(pathlib.Path(__file__).parent / "sim_terrain.py"),
)
sim = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sim)

# ── Grid definition ──────────────────────────────────────────────────────────
THRESHOLDS   = [400, 450, 500, 550, 600, 650, 700]
TERRAIN_TYPES = ['wet_sand', 'sand_rock', 'worst_case']
GAIT_IDS     = {0: 'tripod', 1: 'wave', 2: 'quadruped'}
SEEDS        = list(range(5))   # 5 seeds per cell
FRAMES       = 2000             # 40 s of simulation per cell
SPEED        = 350

# Speed by gait (wave is slower by design)
GAIT_SPEED   = {0: 800, 1: 350, 2: 500}

# Per-scenario pass criteria (mirrors sim_terrain.evaluate() spirit)
def cell_pass(r, threshold):
    fails = []
    if not r['gov_ok']:
        fails.append('governor exceeded')
    if r['max_speed_sts'] > 3000:
        fails.append('speed limit')
    if r['max_exit_snap'] > 600:
        fails.append(f"exit snap {r['max_exit_snap']:.0f} STS")
    return fails

def main():
    date_str = datetime.date.today().strftime('%Y%m%d')
    out_path = f"sweep_stall_threshold_{date_str}.csv"

    print()
    print("=" * 70)
    print("  PHASE 1 — STALL_THRESHOLD SENSITIVITY SWEEP")
    print(f"  Grid: {len(THRESHOLDS)} thresholds × {len(TERRAIN_TYPES)} terrains × "
          f"{len(GAIT_IDS)} gaits × {len(SEEDS)} seeds = "
          f"{len(THRESHOLDS)*len(TERRAIN_TYPES)*len(GAIT_IDS)*len(SEEDS)} cells")
    print("=" * 70)

    rows = []
    total = len(THRESHOLDS) * len(TERRAIN_TYPES) * len(GAIT_IDS) * len(SEEDS)
    done  = 0

    for threshold in THRESHOLDS:
        # Monkey-patch the threshold into sim_terrain for this sweep pass
        sim.STALL_THRESHOLD = threshold
        sim.INSTANT_STALL   = int(threshold * 1.5)  # maintain same ratio as 600/900

        for terrain in TERRAIN_TYPES:
            for gait_id, gait_name in GAIT_IDS.items():
                for seed in SEEDS:
                    r = sim.run_scenario(
                        name=f"sweep",
                        gait_id=gait_id,
                        speed=GAIT_SPEED[gait_id],
                        turn_bias=0.0,
                        frames=FRAMES,
                        terrain_type=terrain,
                        ramp_deg=0.0,
                        seed=seed,
                    )
                    fails = cell_pass(r, threshold)
                    rows.append({
                        'threshold':      threshold,
                        'terrain':        terrain,
                        'gait':           gait_name,
                        'seed':           seed,
                        'stall_fraction': f"{r['stall_fraction']:.5f}",
                        'max_exit_snap':  f"{r['max_exit_snap']:.1f}",
                        'gov_headroom_hz': f"{r['gov_headroom_hz']:.5f}",
                        'pass':           'PASS' if not fails else 'FAIL',
                        'fail_reason':    '; '.join(fails) if fails else '',
                    })
                    done += 1
                    if done % 20 == 0 or done == total:
                        print(f"  [{done}/{total}] thr={threshold} {terrain} {gait_name} ...", flush=True)

    # Write CSV
    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nCSV written: {out_path}")

    # ── Summary table ──
    print()
    print("=" * 70)
    print("  SUMMARY: pass rate by threshold (all terrains & gaits)")
    print(f"  {'Threshold':>10}  {'wet_sand':>10}  {'sand_rock':>10}  {'worst_case':>12}  {'OVERALL':>8}")
    print("-" * 70)

    for threshold in THRESHOLDS:
        cell_rows = [r for r in rows if int(r['threshold']) == threshold]
        def pct(subset):
            if not subset: return '-'
            n = sum(1 for r in subset if r['pass'] == 'PASS')
            return f"{100*n//len(subset)}%"

        wet  = [r for r in cell_rows if r['terrain'] == 'wet_sand']
        srk  = [r for r in cell_rows if r['terrain'] == 'sand_rock']
        wc   = [r for r in cell_rows if r['terrain'] == 'worst_case']
        print(f"  {threshold:>10}  {pct(wet):>10}  {pct(srk):>10}  {pct(wc):>12}  {pct(cell_rows):>8}")

    # ── Key acceptance checks ──
    print()
    print("  ACCEPTANCE CRITERIA:")
    def pass_rate(threshold, terrain):
        subset = [r for r in rows
                  if int(r['threshold']) == threshold and r['terrain'] == terrain]
        if not subset: return 0.0
        return sum(1 for r in subset if r['pass'] == 'PASS') / len(subset)

    # Restore original threshold
    sim.STALL_THRESHOLD = 600
    sim.INSTANT_STALL   = 900

    ok600  = pass_rate(600, 'wet_sand') == 1.0 and \
             pass_rate(600, 'sand_rock') == 1.0 and \
             pass_rate(600, 'worst_case') == 1.0

    # At 500: stall_fraction <= 0.05 on wet_sand
    wet500 = [r for r in rows if int(r['threshold'])==500 and r['terrain']=='wet_sand']
    ok500  = all(float(r['stall_fraction']) <= 0.05 for r in wet500)

    # At 650: zero stalls on wet_sand
    wet650 = [r for r in rows if int(r['threshold'])==650 and r['terrain']=='wet_sand']
    ok650  = all(float(r['stall_fraction']) == 0.0 for r in wet650)

    print(f"  [{'PASS' if ok600 else 'FAIL'}] threshold=600: all terrains/gaits pass")
    print(f"  [{'PASS' if ok500 else 'FAIL'}] threshold=500: wet_sand stall_fraction <= 5%")
    print(f"  [{'PASS' if ok650 else 'FAIL'}] threshold=650: wet_sand stall_fraction = 0%")

    all_ok = ok600 and ok500 and ok650
    print()
    print("=" * 70)
    print(f"  PHASE 1 RESULT: {'ALL CRITERIA PASS' if all_ok else 'SOME CRITERIA FAILED'}")
    print("=" * 70)

if __name__ == "__main__":
    main()
