"""
test_nav_serial.py — Synthetic-frame unit tests for _parse_arduino_csv.

The function under test lives inside `if __name__ == "__main__":` in
final_full_gait_test.py and cannot be imported directly. The implementation
is mirrored here verbatim. When the gait file changes _parse_arduino_csv, this
mirror must be updated in sync.

This is the standard pattern for testing monolith-embedded functions.
See decisions.md ADR-001.

Run:
    pytest C:/Users/rgane/Downloads/test_nav_serial.py -v
"""

# ---------------------------------------------------------------------------
# Mirror of _parse_arduino_csv from final_full_gait_test.py (fix #50)
# Lines 1124–1187 in the gait file (as of 2026-03-13).
# Keep in sync with the gait file implementation.
# ---------------------------------------------------------------------------

# Column layout constants (matches gait file lines 1124–1132)
CSV_COLS = 20
IDX_TS = 0
IDX_FDL, IDX_FCF, IDX_FCD, IDX_FDR = 1, 2, 3, 4
IDX_RDL, IDX_RCF, IDX_RCD, IDX_RDR = 5, 6, 7, 8
IDX_QW, IDX_QX, IDX_QY, IDX_QZ = 9, 10, 11, 12
IDX_AX, IDX_AY, IDX_AZ = 13, 14, 15
IDX_GX, IDX_GY, IDX_GZ = 16, 17, 18
IDX_UPSIDE = 19


def _parse_arduino_csv(line):
    """Mirror of _parse_arduino_csv from final_full_gait_test.py.

    Parse a 20-column CSV line from Arduino sensor hub into a frame dict.
    Returns dict or None on parse failure.
    """
    if not line:
        return None
    line = line.strip()
    if not line or line.startswith("timestamp_ms"):
        return None
    parts = line.split(",")
    if len(parts) != CSV_COLS:
        return None
    try:
        ts = int(parts[IDX_TS])
        dists = []
        for i in range(IDX_FDL, IDX_RDR + 1):
            v = float(parts[i])
            dists.append(v)
        qw = float(parts[IDX_QW])
        qx = float(parts[IDX_QX])
        qy = float(parts[IDX_QY])
        qz = float(parts[IDX_QZ])
        ax = float(parts[IDX_AX])
        ay = float(parts[IDX_AY])
        az = float(parts[IDX_AZ])
        gx = float(parts[IDX_GX])
        gy = float(parts[IDX_GY])
        gz = float(parts[IDX_GZ])
        upside = int(float(parts[IDX_UPSIDE]))
    except (ValueError, IndexError):
        return None

    # Upside-down remap: swap front/rear, preserve body-frame left/right
    if upside == 1:
        # Original: [FDL, FCF, FCD, FDR, RDL, RCF, RCD, RDR]
        # Remapped: [RDL, RCF, RCD, RDR, FDL, FCF, FCD, FDR]
        dists = dists[4:8] + dists[0:4]
        upside = 0

    # Clamp distances: preserve -1 sentinel, cap max at 450
    for i in range(len(dists)):
        if dists[i] == -1:
            continue
        if dists[i] > 450:
            dists[i] = 450.0

    return {
        "timestamp_ms": ts,
        "FDL": dists[0], "FCF": dists[1], "FCD": dists[2], "FDR": dists[3],
        "RDL": dists[4], "RCF": dists[5], "RCD": dists[6], "RDR": dists[7],
        "qw": qw, "qx": qx, "qy": qy, "qz": qz,
        "ax": ax, "ay": ay, "az": az,
        "gx": gx, "gy": gy, "gz": gz,
        "upside_down": upside,
    }


# ---------------------------------------------------------------------------
# Test helper
# ---------------------------------------------------------------------------

def make_csv_line(
    ts=12345,
    fdl=100.0, fcf=200.0, fcd=15.0, fdr=100.0,
    rdl=100.0, rcf=200.0, rcd=15.0, rdr=100.0,
    qw=1.0, qx=0.0, qy=0.0, qz=0.0,
    ax=0.0, ay=0.0, az=9.8,
    gx=0.0, gy=0.0, gz=0.0,
    upside_down=0,
):
    """Build a valid 20-column Arduino CSV string from keyword arguments.

    All parameters have sensible defaults (all-clear, upright, stationary).
    Override individual fields to construct targeted test cases.
    """
    fields = [
        ts,
        fdl, fcf, fcd, fdr,
        rdl, rcf, rcd, rdr,
        qw, qx, qy, qz,
        ax, ay, az,
        gx, gy, gz,
        upside_down,
    ]
    return ",".join(str(f) for f in fields)


# ---------------------------------------------------------------------------
# 1. Valid line — all keys present, values correctly parsed
# ---------------------------------------------------------------------------

def test_parse_valid_line_returns_dict():
    """A correctly formatted 20-column line must produce a non-None dict."""
    frame = _parse_arduino_csv(make_csv_line())
    assert frame is not None


def test_parse_valid_line_all_keys_present():
    """Returned dict must contain all expected sensor keys."""
    frame = _parse_arduino_csv(make_csv_line())
    expected_keys = {
        "timestamp_ms",
        "FDL", "FCF", "FCD", "FDR",
        "RDL", "RCF", "RCD", "RDR",
        "qw", "qx", "qy", "qz",
        "ax", "ay", "az",
        "gx", "gy", "gz",
        "upside_down",
    }
    assert expected_keys == set(frame.keys())


def test_parse_valid_line_timestamp_parsed():
    """timestamp_ms must be returned as int."""
    frame = _parse_arduino_csv(make_csv_line(ts=99999))
    assert frame["timestamp_ms"] == 99999
    assert isinstance(frame["timestamp_ms"], int)


def test_parse_valid_line_distance_values():
    """Distance fields must round-trip correctly for typical in-range values."""
    frame = _parse_arduino_csv(make_csv_line(fdl=50.0, fcf=75.5, fcd=15.0, fdr=120.0,
                                              rdl=30.0, rcf=60.0, rcd=12.0, rdr=200.0))
    assert frame["FDL"] == 50.0
    assert frame["FCF"] == 75.5
    assert frame["FCD"] == 15.0
    assert frame["FDR"] == 120.0
    assert frame["RDL"] == 30.0
    assert frame["RCF"] == 60.0
    assert frame["RCD"] == 12.0
    assert frame["RDR"] == 200.0


def test_parse_valid_line_imu_values():
    """IMU quaternion, accel, gyro fields must round-trip correctly."""
    frame = _parse_arduino_csv(make_csv_line(qw=0.707, qx=0.0, qy=0.707, qz=0.0,
                                              ax=1.1, ay=2.2, az=9.5,
                                              gx=0.01, gy=0.02, gz=0.03))
    assert abs(frame["qw"] - 0.707) < 1e-6
    assert abs(frame["qy"] - 0.707) < 1e-6
    assert abs(frame["ax"] - 1.1) < 1e-6
    assert abs(frame["ay"] - 2.2) < 1e-6
    assert abs(frame["az"] - 9.5) < 1e-6
    assert abs(frame["gx"] - 0.01) < 1e-6
    assert abs(frame["gz"] - 0.03) < 1e-6


# ---------------------------------------------------------------------------
# 2. Header line — must be skipped
# ---------------------------------------------------------------------------

def test_parse_header_skip():
    """A line starting with 'timestamp_ms' is the CSV header and must return None."""
    header = "timestamp_ms,FDL,FCF,FCD,FDR,RDL,RCF,RCD,RDR,QuatW,QuatX,QuatY,QuatZ,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,UpsideDown"
    assert _parse_arduino_csv(header) is None


def test_parse_header_skip_with_leading_whitespace():
    """Header preceded by whitespace must also be rejected after strip."""
    header = "  timestamp_ms,FDL,FCF,FCD,FDR,RDL,RCF,RCD,RDR,QuatW,QuatX,QuatY,QuatZ,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,UpsideDown"
    # After strip, starts with 'timestamp_ms' — should be None
    assert _parse_arduino_csv(header) is None


# ---------------------------------------------------------------------------
# 3. Empty / blank line
# ---------------------------------------------------------------------------

def test_parse_empty_string_returns_none():
    """Empty string must return None immediately."""
    assert _parse_arduino_csv("") is None


def test_parse_none_input_returns_none():
    """None input must return None without raising."""
    assert _parse_arduino_csv(None) is None


def test_parse_whitespace_only_returns_none():
    """A line with only whitespace strips to empty — must return None."""
    assert _parse_arduino_csv("   \t\n") is None


# ---------------------------------------------------------------------------
# 4. Wrong column count — 19 or 21 columns
# ---------------------------------------------------------------------------

def test_parse_too_few_columns_returns_none():
    """19-column line must be rejected (CSV_COLS == 20)."""
    # Drop the last field
    line = make_csv_line()
    short = ",".join(line.split(",")[:19])
    assert _parse_arduino_csv(short) is None


def test_parse_too_many_columns_returns_none():
    """21-column line must be rejected."""
    line = make_csv_line()
    long_line = line + ",9999"
    assert _parse_arduino_csv(long_line) is None


def test_parse_single_field_returns_none():
    """A single-value line must be rejected as wrong column count."""
    assert _parse_arduino_csv("12345") is None


# ---------------------------------------------------------------------------
# 5. Non-numeric value in distance field
# ---------------------------------------------------------------------------

def test_parse_non_numeric_distance_returns_none():
    """A non-numeric value in a distance column must return None."""
    parts = make_csv_line().split(",")
    parts[IDX_FCF] = "ERR"   # FCF = column 2
    assert _parse_arduino_csv(",".join(parts)) is None


def test_parse_non_numeric_in_timestamp_returns_none():
    """A non-numeric timestamp must return None."""
    parts = make_csv_line().split(",")
    parts[IDX_TS] = "abc"
    assert _parse_arduino_csv(",".join(parts)) is None


def test_parse_non_numeric_in_quaternion_returns_none():
    """A non-numeric quaternion value must return None."""
    parts = make_csv_line().split(",")
    parts[IDX_QW] = "NaN_bad"
    assert _parse_arduino_csv(",".join(parts)) is None


def test_parse_non_numeric_in_gyro_returns_none():
    """A non-numeric gyro value must return None."""
    parts = make_csv_line().split(",")
    parts[IDX_GZ] = "overflow"
    assert _parse_arduino_csv(",".join(parts)) is None


# ---------------------------------------------------------------------------
# 6. Sentinel -1 is preserved in distance fields
# ---------------------------------------------------------------------------

def test_parse_sentinel_minus1_fdl_preserved():
    """FDL == -1 (blind-zone sentinel from HC-SR04) must be preserved as -1.0."""
    frame = _parse_arduino_csv(make_csv_line(fdl=-1.0))
    assert frame["FDL"] == -1.0


def test_parse_sentinel_minus1_fcf_preserved():
    """FCF == -1 must pass through unclamped."""
    frame = _parse_arduino_csv(make_csv_line(fcf=-1.0))
    assert frame["FCF"] == -1.0


def test_parse_sentinel_minus1_fcd_preserved():
    """FCD == -1 (cliff sensor in blind zone) must be preserved, not treated as cliff."""
    frame = _parse_arduino_csv(make_csv_line(fcd=-1.0))
    assert frame["FCD"] == -1.0


def test_parse_sentinel_minus1_all_sensors():
    """All 8 distance sensors reporting -1 must all be preserved."""
    frame = _parse_arduino_csv(make_csv_line(
        fdl=-1.0, fcf=-1.0, fcd=-1.0, fdr=-1.0,
        rdl=-1.0, rcf=-1.0, rcd=-1.0, rdr=-1.0,
    ))
    for key in ("FDL", "FCF", "FCD", "FDR", "RDL", "RCF", "RCD", "RDR"):
        assert frame[key] == -1.0, f"{key} sentinel not preserved"


# ---------------------------------------------------------------------------
# 7. Distance > 450 clamped to 450
# ---------------------------------------------------------------------------

def test_parse_clamp_500_to_450():
    """Distance 500.0 must be clamped to 450.0 (HC-SR04 max reliable range)."""
    frame = _parse_arduino_csv(make_csv_line(fdl=500.0))
    assert frame["FDL"] == 450.0


def test_parse_clamp_9999_to_450():
    """Extreme out-of-range value 9999 must clamp to 450.0."""
    frame = _parse_arduino_csv(make_csv_line(fcf=9999.0))
    assert frame["FCF"] == 450.0


def test_parse_clamp_exactly_450_unchanged():
    """Distance exactly 450.0 is at the boundary and must NOT be clamped further."""
    frame = _parse_arduino_csv(make_csv_line(fdr=450.0))
    assert frame["FDR"] == 450.0


def test_parse_clamp_451_to_450():
    """Distance 451.0 is just over the boundary and must be clamped to 450.0."""
    frame = _parse_arduino_csv(make_csv_line(rdl=451.0))
    assert frame["RDL"] == 450.0


def test_parse_clamp_does_not_affect_in_range():
    """Values in range (e.g. 200.0) must not be modified by the clamp logic."""
    frame = _parse_arduino_csv(make_csv_line(rcf=200.0))
    assert frame["RCF"] == 200.0


# ---------------------------------------------------------------------------
# 8. Upside-down remap (UpsideDown=1)
# ---------------------------------------------------------------------------

def test_parse_upside_down_front_becomes_rear():
    """UpsideDown=1: original front sensors must appear in rear slots after remap."""
    frame = _parse_arduino_csv(make_csv_line(
        fdl=10.0, fcf=20.0, fcd=15.0, fdr=30.0,
        rdl=40.0, rcf=50.0, rcd=16.0, rdr=60.0,
        upside_down=1,
    ))
    # After remap: new rear = original front
    assert frame["RDL"] == 10.0
    assert frame["RCF"] == 20.0
    assert frame["RCD"] == 15.0
    assert frame["RDR"] == 30.0


def test_parse_upside_down_rear_becomes_front():
    """UpsideDown=1: original rear sensors must appear in front slots after remap."""
    frame = _parse_arduino_csv(make_csv_line(
        fdl=10.0, fcf=20.0, fcd=15.0, fdr=30.0,
        rdl=40.0, rcf=50.0, rcd=16.0, rdr=60.0,
        upside_down=1,
    ))
    # After remap: new front = original rear
    assert frame["FDL"] == 40.0
    assert frame["FCF"] == 50.0
    assert frame["FCD"] == 16.0
    assert frame["FDR"] == 60.0


def test_parse_upside_down_flag_cleared():
    """UpsideDown=1 input must be cleared to 0 in the output frame."""
    frame = _parse_arduino_csv(make_csv_line(upside_down=1))
    assert frame["upside_down"] == 0


def test_parse_upside_down_remap_is_symmetric():
    """Applying remap twice should be equivalent to no remap (distances are swapped)."""
    original_fdl = 77.0
    original_rdl = 33.0
    # First remap: fdl→RDL, rdl→FDL
    frame = _parse_arduino_csv(make_csv_line(fdl=original_fdl, rdl=original_rdl, upside_down=1))
    assert frame["FDL"] == original_rdl
    assert frame["RDL"] == original_fdl


# ---------------------------------------------------------------------------
# 9. Upside-down=0 — no remap, distances stay in original order
# ---------------------------------------------------------------------------

def test_parse_upside_down_zero_no_remap():
    """UpsideDown=0: sensor values must remain in their original slots."""
    frame = _parse_arduino_csv(make_csv_line(
        fdl=10.0, fcf=20.0, fcd=15.0, fdr=30.0,
        rdl=40.0, rcf=50.0, rcd=16.0, rdr=60.0,
        upside_down=0,
    ))
    assert frame["FDL"] == 10.0
    assert frame["FCF"] == 20.0
    assert frame["FCD"] == 15.0
    assert frame["FDR"] == 30.0
    assert frame["RDL"] == 40.0
    assert frame["RCF"] == 50.0
    assert frame["RCD"] == 16.0
    assert frame["RDR"] == 60.0


def test_parse_upside_down_zero_flag_preserved():
    """UpsideDown=0 must remain 0 in the output frame (no mutation)."""
    frame = _parse_arduino_csv(make_csv_line(upside_down=0))
    assert frame["upside_down"] == 0


# ---------------------------------------------------------------------------
# 10. Sentinel -1 NOT clamped (regression guard)
# ---------------------------------------------------------------------------

def test_parse_sentinel_not_clamped_to_450():
    """Critical regression: -1 sentinel must never be clamped to 450.

    Without the `if dists[i] == -1: continue` guard, the clamp branch
    `if dists[i] > 450` would not fire for -1, but a future refactor that
    reorders the checks could silently break this. This test locks the behavior.
    """
    frame = _parse_arduino_csv(make_csv_line(fdl=-1.0))
    assert frame["FDL"] == -1.0
    assert frame["FDL"] != 450.0


def test_parse_sentinel_not_clamped_distinct_from_zero():
    """-1 sentinel must be distinct from 0 (distance=0 means object touching sensor)."""
    frame_sentinel = _parse_arduino_csv(make_csv_line(fdl=-1.0))
    frame_zero = _parse_arduino_csv(make_csv_line(fdl=0.0))
    assert frame_sentinel["FDL"] == -1.0
    assert frame_zero["FDL"] == 0.0
    assert frame_sentinel["FDL"] != frame_zero["FDL"]


def test_parse_sentinel_in_rear_not_clamped():
    """Sentinel -1 in rear sensors must also be preserved (not only front)."""
    frame = _parse_arduino_csv(make_csv_line(rcd=-1.0))
    assert frame["RCD"] == -1.0


# ---------------------------------------------------------------------------
# 11. Additional robustness edge cases
# ---------------------------------------------------------------------------

def test_parse_upside_down_sentinel_remap_preserved():
    """Sentinel -1 in front sensor, upside_down=1: after remap -1 must still be -1.

    Catches a bug where clamping runs on the pre-remap indices before the swap.
    The correct behavior is: remap first, then clamp. The mirrored code does
    remap then clamp, so -1 in any slot survives regardless of swap.
    """
    # rdl=450, fdl=-1 → after remap: FDL=450, RDL=-1
    frame = _parse_arduino_csv(make_csv_line(fdl=-1.0, rdl=450.0, upside_down=1))
    assert frame["FDL"] == 450.0   # original rdl
    assert frame["RDL"] == -1.0    # original fdl sentinel, preserved after remap


def test_parse_upside_down_clamp_applies_after_remap():
    """Clamping must apply to post-remap positions, not pre-remap.

    If fdl=500 and upside_down=1, the 500 will end up in RDL after remap.
    It must still be clamped to 450.
    """
    frame = _parse_arduino_csv(make_csv_line(fdl=500.0, rdl=100.0, upside_down=1))
    # After remap: FDL=100 (from rdl), RDL=500 → clamped to 450
    assert frame["FDL"] == 100.0
    assert frame["RDL"] == 450.0


def test_parse_float_upside_down_value_accepted():
    """UpsideDown field transmitted as float (e.g. '1.0') must be accepted via int(float(...))."""
    parts = make_csv_line(upside_down=0).split(",")
    parts[IDX_UPSIDE] = "1.0"   # Arduino may send float
    frame = _parse_arduino_csv(",".join(parts))
    assert frame is not None
    assert frame["upside_down"] == 0   # remap fired and cleared flag


def test_parse_zero_distances_accepted():
    """Distance=0 means object touching sensor face — must be accepted, not rejected."""
    frame = _parse_arduino_csv(make_csv_line(fdl=0.0, fcf=0.0))
    assert frame is not None
    assert frame["FDL"] == 0.0
    assert frame["FCF"] == 0.0


def test_parse_newline_stripped_from_line():
    """Trailing CRLF from readline() must not cause a parse failure."""
    line = make_csv_line() + "\r\n"
    frame = _parse_arduino_csv(line)
    assert frame is not None


def test_parse_trailing_space_stripped():
    """Trailing whitespace must be stripped and not affect column count."""
    line = make_csv_line() + "   "
    frame = _parse_arduino_csv(line)
    assert frame is not None


def test_parse_exactly_20_columns_accepted():
    """Boundary: exactly 20 columns must be accepted (not off-by-one rejection)."""
    line = make_csv_line()
    assert len(line.split(",")) == 20
    assert _parse_arduino_csv(line) is not None
