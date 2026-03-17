#!/usr/bin/env python3
"""
param_sweep.py — Hexapod parameter sweep runner.

Edits a single constant in final_full_gait_test.py, runs sim_verify.py
and sim_terrain.py, reports pass/fail + key metrics, then restores the
original file.

Usage:
  python param_sweep.py CONSTANT_NAME value1 value2 value3 ...

Example:
  python param_sweep.py CRUISE_SPEED 400 450 500 550 600
  python param_sweep.py STALL_THRESHOLD 600 700 750 800
  python param_sweep.py OVERLOAD_PREVENTION_TIME 1.0 1.25 1.5 1.75 2.0

Output: table of results per value with pass/fail and metric summaries.
"""

import sys
import os
import re
import subprocess
import shutil
from pathlib import Path

GAIT_FILE = Path(r"C:\Users\rgane\Downloads\final_full_gait_test.py")
SIM_VERIFY = Path(r"C:\Users\rgane\Downloads\sim_verify.py")
SIM_TERRAIN = Path(r"C:\Users\rgane\Downloads\sim_terrain.py")
SIM_NAV = Path(r"C:\Users\rgane\Downloads\sim_nav.py")
BACKUP_SUFFIX = ".sweep_backup"


def backup_file():
    backup = GAIT_FILE.with_suffix(GAIT_FILE.suffix + BACKUP_SUFFIX)
    shutil.copy2(GAIT_FILE, backup)
    return backup


def restore_file(backup):
    shutil.copy2(backup, GAIT_FILE)
    backup.unlink()


def set_constant(name, value):
    """Replace a constant assignment in the gait file. Returns True if found."""
    content = GAIT_FILE.read_text(encoding="utf-8")
    # Match patterns like: NAME = 750   or   NAME = 1.5
    pattern = rf"^(\s*{re.escape(name)}\s*=\s*)([^\s#]+)"
    new_content, count = re.subn(pattern, rf"\g<1>{value}", content, flags=re.MULTILINE)
    if count == 0:
        return False
    GAIT_FILE.write_text(new_content, encoding="utf-8")
    return True


def run_sim(sim_path):
    """Run a sim script, return (pass_count, fail_count, total, output_summary)."""
    try:
        result = subprocess.run(
            [sys.executable, str(sim_path)],
            capture_output=True, text=True, timeout=120,
            cwd=str(sim_path.parent)
        )
        output = result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        return (0, 0, 0, "TIMEOUT")

    passes = len(re.findall(r"\bPASS\b", output))
    fails = len(re.findall(r"\bFAIL\b", output))
    infos = len(re.findall(r"\bINFO\b", output))

    # Extract key metrics if present
    metrics = []
    for pattern, label in [
        (r"carve headroom[:\s]+([\d.]+)%", "carve_headroom"),
        (r"max speed[:\s]+([\d.]+)", "max_speed"),
        (r"governor[:\s]+([\d.]+)%", "governor"),
        (r"gap56[:\s]+([\d.]+)", "gap56"),
    ]:
        m = re.search(pattern, output, re.IGNORECASE)
        if m:
            metrics.append(f"{label}={m.group(1)}")

    summary = f"{passes}P/{fails}F/{infos}I"
    if metrics:
        summary += " | " + ", ".join(metrics)

    return (passes, fails, infos, summary)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    const_name = sys.argv[1]
    values = sys.argv[2:]

    print(f"{'='*70}")
    print(f"Parameter Sweep: {const_name}")
    print(f"Values to test: {', '.join(values)}")
    print(f"Gait file: {GAIT_FILE}")
    print(f"{'='*70}\n")

    backup = backup_file()
    results = []

    try:
        for val in values:
            print(f"\n--- Testing {const_name} = {val} ---")

            if not set_constant(const_name, val):
                print(f"  ERROR: Constant '{const_name}' not found in {GAIT_FILE.name}")
                results.append((val, "NOT_FOUND", "", ""))
                continue

            # Run sims
            v_pass, v_fail, v_info, v_summary = run_sim(SIM_VERIFY)
            t_pass, t_fail, t_info, t_summary = run_sim(SIM_TERRAIN)

            verdict = "PASS" if v_fail == 0 and t_fail == 0 else "FAIL"
            results.append((val, verdict, f"verify: {v_summary}", f"terrain: {t_summary}"))
            print(f"  sim_verify:  {v_summary}")
            print(f"  sim_terrain: {t_summary}")
            print(f"  VERDICT: {verdict}")

            # Restore original before next iteration
            restore_file(backup)
            backup = backup_file()

    finally:
        restore_file(backup)

    # Print summary table
    print(f"\n{'='*70}")
    print(f"SWEEP RESULTS: {const_name}")
    print(f"{'='*70}")
    print(f"{'Value':<12} {'Verdict':<8} {'sim_verify':<30} {'sim_terrain':<30}")
    print(f"{'-'*12} {'-'*8} {'-'*30} {'-'*30}")
    for val, verdict, v_sum, t_sum in results:
        marker = ">>>" if verdict == "PASS" else "   "
        print(f"{marker} {val:<9} {verdict:<8} {v_sum:<30} {t_sum:<30}")

    passing = [r for r in results if r[1] == "PASS"]
    if passing:
        print(f"\nPassing values: {', '.join(r[0] for r in passing)}")
    else:
        print(f"\nNo values passed all sims.")


if __name__ == "__main__":
    main()
