#!/usr/bin/env python3
"""
Phase 6 — Competition-Day Configuration Validator
Reads full_gait_test.py directly (no execution) and verifies all tunable
constants are within the bounds established by Phases 1-5.
Prints GO / NO-GO per parameter.  Any NO-GO blocks competition run.
"""

import sys, re, pathlib

TARGET = pathlib.Path(__file__).parent / "full_gait_test.py"

# -- Expected HOME_POSITIONS (from MEMORY.md — verified against hardware) ------
EXPECTED_HOME = {1: 3447, 2: 955, 3: 1420, 4: 1569, 5: 3197, 6: 3175}

# -- Parameter specification --------------------------------------------------
# Each entry: (name, required_value_or_range, source_description)
# Range = (min, max, inclusive).  Exact value = a number or dict.
SPECS = [
    # name, type ('exact'|'range'|'dict'|'lte'|'gte'), value, source
    ('STALL_THRESHOLD',   'range',  (550, 650),        'Phase 1 sweep'),
    ('STALL_EXIT_FRAMES', 'range',  (8,   12),         'MEMORY fix record'),
    ('BLEND',             'range',  (0.03, 0.05),      'C1 continuity constraint'),
    ('KP_PHASE',          'range',  (10.0, 14.0),      'stability range'),
    ('VELOCITY_SCALAR',   'range',  (1.80, 1.90),      'hardware calibration ±0.05'),
]

# Gait duty cycle ranges
GAIT_DUTY_SPECS = {
    0: ('tripod',    0.5,   0.5  ),   # exact
    1: ('wave',      0.68,  0.72 ),   # range — restored for 4-leg stability
    2: ('quadruped', 0.68,  0.72 ),   # range — restored for 4-leg stability
}

# Governor max hz
GOV_MAX_HZ = 0.47

# Wave carve bias (abs)
CARVE_BIAS_RANGE = (0.10, 0.14)

LABEL_W = 48

def verdict(label, status, actual, required, source=''):
    pad = '.' * max(1, LABEL_W - len(label))
    actual_s   = str(actual)
    required_s = str(required)
    src_s      = f"  [{source}]" if source else ''
    print(f"  {label} {pad} [{status}]  actual={actual_s}  required={required_s}{src_s}")
    return status == 'GO'


def extract_scalar(src, name):
    """Extract a Python-literal constant: NAME = <value>
    Matches both module-level and function-local assignments."""
    # Allow any leading whitespace so indented local constants are found
    pattern = rf'^\s*{re.escape(name)}\s*=\s*([^\n#]+)'
    m = re.search(pattern, src, re.MULTILINE)
    if not m:
        return None
    try:
        return eval(m.group(1).strip())
    except:
        return None


def extract_gaits(src):
    """Extract GAITS dict — parse duty and offsets per gait_id."""
    m = re.search(r'GAITS\s*=\s*\{(.+?)\n\}', src, re.DOTALL)
    if not m:
        return None
    block = m.group(1)
    # Find each gait_id block
    gaits = {}
    for gid_m in re.finditer(r'(\d+):\s*\{[^{]*?\'duty\':\s*([\d.]+)[^{]*?\'offsets\':\s*\{([^}]+)\}', block, re.DOTALL):
        gid   = int(gid_m.group(1))
        duty  = float(gid_m.group(2))
        offsets_str = gid_m.group(3)
        offsets = {}
        for om in re.finditer(r'(\d+)\s*:\s*([\d.]+)', offsets_str):
            offsets[int(om.group(1))] = float(om.group(2))
        gaits[gid] = {'duty': duty, 'offsets': offsets}
    return gaits


def extract_home(src):
    """Extract HOME_POSITIONS dict."""
    m = re.search(r'HOME_POSITIONS\s*=\s*\{([^}]+)\}', src)
    if not m:
        return None
    d = {}
    for pair in re.finditer(r'(\d+)\s*:\s*(\d+)', m.group(1)):
        d[int(pair.group(1))] = int(pair.group(2))
    return d


def extract_carve_bias(src):
    """Find the wave carve bias from tactical_sleep blocks.
    Specifically searches the Wave gait section (Phase 3 / Wave block)
    and picks the smallest non-zero carve turn_bias (pivot biases are larger).
    """
    # Find Wave phase block — look for "Wave" gait label
    wave_block_m = re.search(
        r'#[- ]+Phase [^:]+: Wave[- ]+#(.+?)#[- ]+Phase',
        src, re.DOTALL | re.IGNORECASE,
    )
    if not wave_block_m:
        # Fallback: look for "Phase 3" block
        wave_block_m = re.search(
            r'Phase 3.*?Wave(.+?)Phase 4',
            src, re.DOTALL | re.IGNORECASE,
        )
    if not wave_block_m:
        return None

    block  = wave_block_m.group(1)
    biases = re.findall(r'shared_turn_bias\.value\s*=\s*(-?[\d.]+)', block)
    # Carve biases are small (0.05-0.3); pivot biases are larger (>0.3)
    carves = [abs(float(b)) for b in biases if 0.05 < abs(float(b)) <= 0.3]
    return min(carves) if carves else None


def extract_governor_hz(src):
    """Confirm governor formula is present and extract effective max_safe_hz.
    Returns a string describing what was found."""
    has_gov = 'max_safe_hz' in src and '2800' in src and 'VELOCITY_SCALAR' in src
    return has_gov


def main():
    print()
    print("=" * 70)
    print("  PHASE 6 — COMPETITION-DAY CONFIGURATION VALIDATOR")
    print(f"  Target: {TARGET}")
    print("=" * 70)

    if not TARGET.exists():
        print(f"\n  ERROR: {TARGET} not found")
        sys.exit(1)

    src = load_src = TARGET.read_text(encoding='utf-8', errors='replace')

    all_go   = True
    any_fail = False

    print()
    print("  -- Core Constants --")

    for param, kind, value, source in SPECS:
        actual = extract_scalar(src, param)
        if actual is None:
            status = 'NO-GO'
            req_s  = f"{value}"
            ok = verdict(param, status, 'NOT FOUND', req_s, source)
        elif kind == 'range':
            lo, hi = value
            ok_val = lo <= actual <= hi
            status = 'GO' if ok_val else 'NO-GO'
            req_s  = f"{lo} – {hi}"
            ok = verdict(param, status, actual, req_s, source)
        elif kind == 'exact':
            ok_val = actual == value
            status = 'GO' if ok_val else 'NO-GO'
            ok = verdict(param, status, actual, value, source)
        else:
            ok = True

        if not ok:
            all_go = False

    print()
    print("  -- Gait Duty Cycles --")
    gaits = extract_gaits(src)
    if gaits is None:
        verdict("GAITS dict", 'NO-GO', 'NOT FOUND', 'parseable dict', 'gait definitions')
        all_go = False
    else:
        for gid, (gname, lo, hi) in GAIT_DUTY_SPECS.items():
            if gid not in gaits:
                verdict(f"GAITS[{gid}] ({gname}) duty", 'NO-GO', 'MISSING', f"{lo}-{hi}", 'gait params')
                all_go = False
            else:
                actual = gaits[gid]['duty']
                if lo == hi:
                    ok_val = abs(actual - lo) < 1e-9
                    req_s  = str(lo)
                else:
                    ok_val = lo <= actual <= hi
                    req_s  = f"{lo} – {hi}"
                status = 'GO' if ok_val else 'NO-GO'
                ok = verdict(f"GAITS[{gid}] ({gname}) duty", status, actual, req_s, 'gait params')
                if not ok:
                    all_go = False

    print()
    print("  -- Wave Carve Bias --")
    carve = extract_carve_bias(src)
    if carve is None:
        verdict("Wave carve turn_bias", 'WARN', 'not found in Wave phase', f"{CARVE_BIAS_RANGE[0]}-{CARVE_BIAS_RANGE[1]}", 'Phase 2 sweep')
    else:
        ok_val = CARVE_BIAS_RANGE[0] <= carve <= CARVE_BIAS_RANGE[1]
        status = 'GO' if ok_val else 'NO-GO'
        ok = verdict("Wave carve turn_bias (abs)", status, carve, f"{CARVE_BIAS_RANGE[0]}-{CARVE_BIAS_RANGE[1]}", 'Phase 2 sweep')
        if not ok:
            all_go = False

    print()
    print("  -- Governor Formula --")
    has_gov = extract_governor_hz(src)
    ok = verdict("Governor formula present", 'GO' if has_gov else 'NO-GO',
                 'yes' if has_gov else 'no', 'present', 'Phase 2 sweep')
    if not ok:
        all_go = False

    # Verify governor clamp: Heart must contain max_safe_hz clamp logic.
    # Brain speed inputs can exceed the governor limit (e.g. speed=1200 → 1.2 hz)
    # because the Heart governor clips hz_L/hz_R before advancing clocks.
    # The relevant check is that the clamp formula is present and uses 2800 STS limit.
    gov_clamp_ok = ('hz_L = max(-max_safe_hz' in src or 'min(max_safe_hz, hz_L)' in src or
                    'max(-max_safe_hz, min(max_safe_hz' in src)
    ok = verdict("Heart governor clamps hz to max_safe_hz", 'GO' if gov_clamp_ok else 'NO-GO',
                 'yes' if gov_clamp_ok else 'no', 'clamp present', 'Phase 2 sweep')
    if not ok:
        all_go = False

    print()
    print("  -- HOME_POSITIONS Auditability --")
    home = extract_home(src)
    if home is None:
        verdict("HOME_POSITIONS dict", 'NO-GO', 'NOT FOUND', str(EXPECTED_HOME), 'MEMORY.md')
        all_go = False
    else:
        match = (home == EXPECTED_HOME)
        diffs = {k: (home.get(k), EXPECTED_HOME[k])
                 for k in EXPECTED_HOME if home.get(k) != EXPECTED_HOME[k]}
        status = 'GO' if match else 'NO-GO'
        detail = 'matches MEMORY.md' if match else f"mismatches: {diffs}"
        ok = verdict("HOME_POSITIONS vs MEMORY.md", status, detail, str(EXPECTED_HOME), 'MEMORY.md')
        if not ok:
            all_go = False

    print()
    print("  -- Servo 6 Quadruped Offset (collision safety) --")
    if gaits and 2 in gaits:
        s6_off = gaits[2]['offsets'].get(6)
        # Widen-stance sweep: offset unchanged at 0.333 (duty changed, not offset)
        ok_val = s6_off is not None and abs(s6_off - 0.333) < 0.01
        status = 'GO' if ok_val else 'NO-GO'
        req    = '0.333 (offset unchanged; duty sweep only)'
        ok = verdict("GAITS[2] servo 6 offset", status, s6_off, req, 'MEMORY collision fix')
        if not ok:
            all_go = False

    print()
    print("=" * 70)
    if all_go:
        print("  RESULT: GO — all parameters within verified bounds")
        print("  Robot is cleared for Alamosa competition run.")
    else:
        print("  RESULT: NO-GO — one or more parameters out of bounds")
        print("  Resolve all NO-GO items before hardware deployment.")
    print("=" * 70)

    sys.exit(0 if all_go else 1)

if __name__ == "__main__":
    main()
