#!/usr/bin/env python3
"""
Phase 2 — Parameter Sweep: Hz Governor Across Gaits
Tests hz values × gaits × terrain combinations.
Key metrics: gov_headroom_hz, max_feedforward_dps, zone_boundary_delta, stall_fraction.
"""

import sys, csv, math, datetime, importlib.util, pathlib

_spec = importlib.util.spec_from_file_location(
    "sim_terrain",
    str(pathlib.Path(__file__).parent / "sim_terrain.py"),
)
sim = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sim)

# ── Grid ─────────────────────────────────────────────────────────────────────
HZ_VALUES    = [0.1, 0.2, 0.3, 0.4, 0.47, 0.5, 0.6]
GAIT_IDS     = {0: 'tripod', 1: 'wave', 2: 'quadruped'}
TERRAIN_TYPES = ['smooth', 'wet_sand', 'worst_case']
FRAMES       = 1500
BLEND        = sim.BLEND

# Duty cycles per gait
DUTIES = {0: 0.5, 1: 0.85, 2: 0.7}


def compute_zone_boundary_delta(hz, gait_id, n_samples=500):
    """
    Finite-difference derivative of feedforward at zone boundaries.
    Samples across t_leg in [0,1] and returns max |dps[i+1] - dps[i]|
    at zone transitions (t_leg near 0, near duty, near duty+2*BLEND, near 1-BLEND).
    """
    duty = DUTIES[gait_id]
    imp_start = 330.0
    imp_end   = 30.0
    base_sweep = (imp_end - imp_start + 180) % 360 - 180
    air_sweep  = 360.0 - abs(base_sweep)
    if base_sweep < 0:
        air_sweep = -air_sweep

    dt = 1.0 / n_samples
    prev_dps = None
    max_delta = 0.0

    for i in range(n_samples + 1):
        t = i * dt
        ff_stance = (base_sweep / duty) * hz

        if t < BLEND:
            env = (1.0 - math.cos(math.pi * t / BLEND)) / 2.0
            dps = ff_stance * env
        elif t < duty:
            dps = ff_stance
        elif t < duty + 2.0 * BLEND:
            bf  = (t - duty) / (2.0 * BLEND)
            ap  = (t - duty) / (1.0 - duty)
            fa  = (air_sweep * math.pi / (2.0 * (1.0 - duty))) * math.sin(math.pi * ap) * hz
            dps = ff_stance * (1.0 - bf) + fa * bf
        elif t > 1.0 - BLEND:
            ap  = (t - duty) / (1.0 - duty)
            faf = (air_sweep * math.pi / (2.0 * (1.0 - duty))) * math.sin(math.pi * ap) * hz
            bfp = (t - (1.0 - BLEND)) / BLEND
            env = (1.0 + math.cos(math.pi * bfp)) / 2.0
            dps = faf * env
        else:
            ap  = (t - duty) / (1.0 - duty)
            dps = (air_sweep * math.pi / (2.0 * (1.0 - duty))) * math.sin(math.pi * ap) * hz

        if prev_dps is not None:
            delta = abs(dps - prev_dps) / dt  # deg/s per unit t
            # Only flag near zone boundaries
            near_boundary = (
                abs(t) < 2 * BLEND or
                abs(t - duty) < 2 * BLEND or
                abs(t - (duty + 2 * BLEND)) < 2 * BLEND or
                abs(t - (1.0 - BLEND)) < 2 * BLEND
            )
            if near_boundary and delta > max_delta:
                max_delta = delta
        prev_dps = dps

    # Convert from per-unit-t to per-frame (at 50 Hz, one frame = hz * 0.02 in t units)
    frame_dt_t = hz * 0.02  # cycles per frame
    return max_delta * frame_dt_t  # deg/s per frame


def hz_to_speed(hz, gait_id):
    """Convert target gait hz to the speed integer used by run_scenario()."""
    return int(hz * 1000.0)


def main():
    date_str = datetime.date.today().strftime('%Y%m%d')
    out_path = f"sweep_hz_{date_str}.csv"

    total = len(HZ_VALUES) * len(GAIT_IDS) * len(TERRAIN_TYPES)
    print()
    print("=" * 70)
    print("  PHASE 2 — HZ GOVERNOR SWEEP")
    print(f"  Grid: {len(HZ_VALUES)} hz × {len(GAIT_IDS)} gaits × "
          f"{len(TERRAIN_TYPES)} terrains = {total} cells")
    print("=" * 70)

    rows = []
    done = 0

    for hz in HZ_VALUES:
        for gait_id, gait_name in GAIT_IDS.items():
            speed = hz_to_speed(hz, gait_id)
            zbd   = compute_zone_boundary_delta(hz, gait_id)

            for terrain in TERRAIN_TYPES:
                r = sim.run_scenario(
                    name='hz_sweep',
                    gait_id=gait_id,
                    speed=speed,
                    turn_bias=0.0,
                    frames=FRAMES,
                    terrain_type=terrain,
                    ramp_deg=0.0,
                    seed=42,
                )

                # Max feedforward dps — compute analytically at mid-air (sin peak)
                duty = DUTIES[gait_id]
                imp_start = 330.0; imp_end = 30.0
                base_sweep = (imp_end - imp_start + 180) % 360 - 180
                air_sweep  = 360.0 - abs(base_sweep)
                max_ff_dps = abs(air_sweep * math.pi / (2.0 * (1.0 - duty))) * hz

                gov_hd = r['gov_headroom_hz']
                gov_ok = gov_hd >= 0
                stall_ok_flat = (terrain == 'smooth' and r['stall_fraction'] == 0.0)
                zbd_ok = (zbd < 200.0) if hz <= 0.3 else True

                fails = []
                if not gov_ok:
                    fails.append(f"gov {gov_hd:.4f}")
                if terrain == 'smooth' and r['stall_fraction'] > 0:
                    fails.append(f"stall on flat ({r['stall_fraction']*100:.1f}%)")
                if hz <= 0.3 and zbd > 200.0:
                    fails.append(f"zone delta {zbd:.1f} deg/s/frame")

                rows.append({
                    'hz':                hz,
                    'gait':              gait_name,
                    'terrain':           terrain,
                    'gov_headroom_hz':   f"{gov_hd:.5f}",
                    'max_ff_dps':        f"{max_ff_dps:.1f}",
                    'zone_boundary_delta': f"{zbd:.2f}",
                    'stall_fraction':    f"{r['stall_fraction']:.5f}",
                    'pass':              'PASS' if not fails else 'FAIL',
                    'fail_reason':       '; '.join(fails),
                })
                done += 1

            if done % 9 == 0:
                print(f"  [{done}/{total}] hz={hz} {gait_name} ...", flush=True)

    # Write CSV
    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nCSV written: {out_path}")

    # ── Per-gait max safe hz table ──
    print()
    print("=" * 70)
    print("  MAX SAFE HZ BY GAIT (gov_headroom_hz >= 0, all terrains)")
    print(f"  {'Gait':<12} {'Max safe hz':>12}  {'Governor margin at 0.47':>24}")
    print("-" * 55)
    for gait_id, gait_name in GAIT_IDS.items():
        gait_rows = [r for r in rows if r['gait'] == gait_name]
        # highest hz where all terrains pass gov
        safe_hz = 0.0
        for hz in sorted(HZ_VALUES):
            hz_rows = [r for r in gait_rows if float(r['hz']) == hz]
            all_gov_ok = all(float(r['gov_headroom_hz']) >= 0 for r in hz_rows)
            if all_gov_ok:
                safe_hz = hz

        at47 = [r for r in gait_rows if abs(float(r['hz']) - 0.47) < 0.01]
        min_hd = min((float(r['gov_headroom_hz']) for r in at47), default=float('nan'))
        print(f"  {gait_name:<12} {safe_hz:>12.2f}  margin@0.47 = {min_hd:+.5f} hz")

    # ── Acceptance criteria ──
    print()
    print("  ACCEPTANCE CRITERIA:")
    # C1: gov_headroom >= 0 for all hz <= 0.47
    c1_rows = [r for r in rows if float(r['hz']) <= 0.47]
    c1 = all(float(r['gov_headroom_hz']) >= 0 for r in c1_rows)

    # C2: zone_boundary_delta < 200 at hz <= 0.3
    c2_rows = [r for r in rows if float(r['hz']) <= 0.3]
    c2 = all(float(r['zone_boundary_delta']) < 200.0 for r in c2_rows)

    # C3: no stalls on flat terrain at any hz
    c3_rows = [r for r in rows if r['terrain'] == 'smooth']
    c3 = all(float(r['stall_fraction']) == 0.0 for r in c3_rows)

    print(f"  [{'PASS' if c1 else 'FAIL'}] gov_headroom_hz >= 0 at all hz <= 0.47")
    print(f"  [{'PASS' if c2 else 'FAIL'}] zone_boundary_delta < 200 deg/s/frame at hz <= 0.3")
    print(f"  [{'PASS' if c3 else 'FAIL'}] zero stalls on flat terrain at any hz")

    all_ok = c1 and c2 and c3
    print()
    print("=" * 70)
    print(f"  PHASE 2 RESULT: {'ALL CRITERIA PASS' if all_ok else 'SOME CRITERIA FAILED'}")
    print("=" * 70)

if __name__ == "__main__":
    main()
