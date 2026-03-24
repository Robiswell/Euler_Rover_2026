#!/usr/bin/env python3
"""
parse_telemetry.py — Diagnostic visualization for hexapod hardware telemetry.

Parses real telemetry data from:
  - Heart telemetry log (pipe-separated fields, ~1 Hz)
  - Arduino sensor CSV (20 columns, ~5 Hz)

Generates diagnostic PNG plots similar to gait_viz.py but for REAL data.

Usage:
  python parse_telemetry.py <heart_log> [--arduino <csv>] [--output-dir <dir>]

Plots generated:
  1. voltage_timeline.png     — Battery voltage over time with dip threshold
  2. servo_loads.png          — Per-servo load over time with stall threshold
  3. servo_temps.png          — Per-servo temperature over time with shutdown threshold
  4. governor_phase.png       — Governor clamp rate + phase error over time
  5. loop_timing.png          — Loop cycle time + jitter over time
  6. stall_events.png         — Stall timeline (which servos, duration)
  7. sensor_distances.png     — Ultrasonic distances over time (if Arduino data)
  8. imu_orientation.png      — Quaternion-derived pitch/roll/yaw (if Arduino data)
  9. run_summary.png          — Combined overview dashboard
 10. servo_positions.png      — Per-servo encoder positions at 5Hz (if [H5] data)
 11. servo_cmd_vs_actual.png  — Commanded vs actual speed per servo (if [H5] data)
 12. nav_timeline.png         — FSM state + speed/turn + sector severity (if [BN]/[BS] data)
"""

import argparse
import os
import re
import sys
from pathlib import Path

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
except ImportError:
    print("matplotlib required: pip install matplotlib")
    sys.exit(1)

import math


# ── Heart telemetry parser ──────────────────────────────────────────────

def parse_heart_log(filepath):
    """Parse Heart telemetry log into list of dicts."""
    records = []
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if not line or not line.startswith('['):
                continue
            rec = parse_heart_line(line)
            if rec:
                records.append(rec)
    return records


def parse_heart_line(line):
    """Parse a single Heart telemetry line into a dict."""
    rec = {}

    # Timestamp [HH:MM:SS]
    ts_match = re.match(r'\[(\d{2}):(\d{2}):(\d{2})\]', line)
    if not ts_match:
        return None
    h, m, s = int(ts_match.group(1)), int(ts_match.group(2)), int(ts_match.group(3))
    rec['time_s'] = h * 3600 + m * 60 + s

    # Key:value pairs
    for key, pattern in [
        ('gait',       r'G:(\w+)'),
        ('speed',      r'Spd:(\d+)'),
        ('turn_bias',  r'Trn:([-\d.]+)'),
        ('voltage',    r'V:([\d.]+)V?'),
        ('volt_dips',  r'Vc:(\d+)'),
        ('amps',       r'A:([\d.]+)'),
        ('loop_ms',    r'Loop:([\d.]+)ms'),
        ('governor',   r'Gov:([\d.]+)%'),
        ('phase_err',  r'PhErr:([-\d.]+)'),
        ('comm_fail',  r'Comm:(\d+)'),
    ]:
        m = re.search(pattern, line)
        if m:
            val = m.group(1)
            if key in ('voltage', 'turn_bias', 'amps', 'loop_ms', 'governor', 'phase_err'):
                rec[key] = float(val)
            elif key in ('speed', 'volt_dips', 'comm_fail'):
                rec[key] = int(val)
            else:
                rec[key] = val

    # Per-servo temperatures T:t1,t2,...
    t_match = re.search(r'T:([\d,]+)\s', line)
    if t_match:
        rec['temps'] = [int(x) for x in t_match.group(1).split(',')]

    # Per-servo loads L:l1,l2,...
    l_match = re.search(r'L:([\d,]+)', line)
    if l_match:
        rec['loads'] = [int(x) for x in l_match.group(1).split(',')]

    # Stall status
    stall_match = re.search(r'Stall:(\S+)', line)
    if stall_match:
        val = stall_match.group(1)
        rec['stall'] = [] if val == 'none' else val.split(',')

    # Jitter
    jit_match = re.search(r'Jit:([\d.]+)/([\d.]+)ms', line)
    if jit_match:
        rec['jitter_max'] = float(jit_match.group(1))
        rec['jitter_std'] = float(jit_match.group(2))

    # Torque-enable cycles
    te_match = re.search(r'TE:([\d,]+)', line)
    if te_match:
        rec['te_cycles'] = [int(x) for x in te_match.group(1).split(',')]

    return rec


# ── Brain event parser ──────────────────────────────────────────────────

def parse_brain_events(filepath):
    """Extract Brain event log entries from telemetry file."""
    events = []
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if '[BRAIN]' not in line and '[STALL]' not in line:
                continue
            ev = {}
            t_match = re.search(r'T\+([\d.]+)', line)
            if t_match:
                ev['time_s'] = float(t_match.group(1))
            tag_match = re.search(r'\[(NAV|WIGGLE-START|WIGGLE-END|ROLL-CHECK|STALL)\]', line)
            if tag_match:
                ev['tag'] = tag_match.group(1)
            ev['raw'] = line
            events.append(ev)
    return events


# ── Arduino CSV parser ──────────────────────────────────────────────────

ARDUINO_COLS = [
    'timestamp_ms', 'FDL', 'FCF', 'FCD', 'FDR',
    'RDL', 'RCF', 'RCD', 'RDR',
    'QuatW', 'QuatX', 'QuatY', 'QuatZ',
    'AccelX', 'AccelY', 'AccelZ',
    'GyroX', 'GyroY', 'GyroZ', 'UpsideDown'
]

ULTRASONIC_COLS = ['FDL', 'FCF', 'FCD', 'FDR', 'RDL', 'RCF', 'RCD', 'RDR']


def parse_arduino_csv(filepath):
    """Parse Arduino sensor CSV into list of dicts."""
    records = []
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or line.startswith('timestamp'):
                continue
            parts = line.split(',')
            if len(parts) < 20:
                continue
            try:
                rec = {}
                for i, col in enumerate(ARDUINO_COLS):
                    if col == 'timestamp_ms':
                        rec[col] = int(parts[i])
                    elif col == 'UpsideDown':
                        rec[col] = int(float(parts[i]))
                    else:
                        rec[col] = float(parts[i])
                rec['time_s'] = rec['timestamp_ms'] / 1000.0
                records.append(rec)
            except (ValueError, IndexError):
                continue
    return records


# ── Verbose telemetry parsers ([H5], [BS], [BN]) ──────────────────────

def parse_h5_lines(filepath):
    """Parse [H5] servo detail lines (5Hz) into list of dicts.
    Format: [H5] T+offset P:pos1,...,pos6 S:spd1,...,spd6 Ph:ph1,...,ph6 FF:ff G:gov D:duty
    ALL_SERVOS order: 2,3,4,1,6,5
    """
    records = []
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            if not line.startswith('[H5]'):
                continue
            rec = {}
            t_match = re.search(r'T\+([\d.]+)', line)
            if t_match:
                rec['time_s'] = float(t_match.group(1))

            p_match = re.search(r'P:([\d,]+)', line)
            if p_match:
                rec['positions'] = [int(x) for x in p_match.group(1).split(',')]

            s_match = re.search(r'S:([-\d,]+)', line)
            if s_match:
                rec['speeds'] = [int(x) for x in s_match.group(1).split(',')]

            ph_match = re.search(r'Ph:([\d.,]+)', line)
            if ph_match:
                rec['phases'] = [float(x) for x in ph_match.group(1).split(',')]

            pht_match = re.search(r'PhT:([\d.,]+)', line)
            if pht_match:
                rec['target_phases'] = [float(x) for x in pht_match.group(1).split(',')]

            ff_match = re.search(r'FF:([\d.]+)', line)
            if ff_match:
                rec['ff_speed'] = float(ff_match.group(1))

            g_match = re.search(r'G:([01])', line)
            if g_match:
                rec['gov_active'] = int(g_match.group(1))

            d_match = re.search(r'D:([\d.]+)', line)
            if d_match:
                rec['duty'] = float(d_match.group(1))

            if 'time_s' in rec:
                records.append(rec)
    return records


def parse_bs_lines(filepath):
    """Parse [BS] sensor snapshot lines (2Hz) into list of dicts.
    Format: [BS] T+offset F:fdl,fcf,fcd,fdr R:rdl,rcf,rcd,rdr P:pitch Ro:roll Y:yaw Ac:accel Gy:gyro U:upright
    """
    records = []
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            if not line.startswith('[BS]'):
                continue
            rec = {}
            t_match = re.search(r'T\+([\d.]+)', line)
            if t_match:
                rec['time_s'] = float(t_match.group(1))

            f_match = re.search(r'F:([\d.,-]+)', line)
            if f_match:
                vals = [float(x) for x in f_match.group(1).split(',')]
                if len(vals) == 4:
                    rec['FDL'], rec['FCF'], rec['FCD'], rec['FDR'] = vals

            r_match = re.search(r' R:([\d.,-]+)', line)
            if r_match:
                vals = [float(x) for x in r_match.group(1).split(',')]
                if len(vals) == 4:
                    rec['RDL'], rec['RCF'], rec['RCD'], rec['RDR'] = vals

            for key, pat in [('pitch', r'P:([-+\d.]+)'), ('roll', r'Ro:([-+\d.]+)'),
                             ('yaw', r'Y:([-+\d.]+)'), ('accel', r'Ac:([\d.]+)'),
                             ('gyro', r'Gy:([\d.]+)'), ('upright', r'U:([\d.]+)')]:
                m = re.search(pat, line)
                if m:
                    rec[key] = float(m.group(1))

            if 'time_s' in rec:
                records.append(rec)
    return records


def parse_bn_lines(filepath):
    """Parse [BN] nav decision lines into list of dicts.
    Format: [BN] T+offset St:state Sec:f/l/r Clf:fc/rc Spd:speed Trn:turn TI:ti Gait:g TM:tm SM:sm Imp:is/ie Flk:f
    """
    records = []
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            if not line.startswith('[BN]'):
                continue
            rec = {}
            t_match = re.search(r'T\+([\d.]+)', line)
            if t_match:
                rec['time_s'] = float(t_match.group(1))

            st_match = re.search(r'St:(\S+)', line)
            if st_match:
                rec['state'] = st_match.group(1)

            sec_match = re.search(r'Sec:(\d)/(\d)/(\d)', line)
            if sec_match:
                rec['front_sev'] = int(sec_match.group(1))
                rec['left_sev'] = int(sec_match.group(2))
                rec['right_sev'] = int(sec_match.group(3))

            clf_match = re.search(r'Clf:(\d)/(\d)', line)
            if clf_match:
                rec['front_cliff'] = int(clf_match.group(1))
                rec['rear_cliff'] = int(clf_match.group(2))

            for key, pat in [('speed', r'Spd:(-?\d+)'), ('base_speed', r'BSpd:(-?\d+)'),
                             ('turn', r'Trn:([-+\d.]+)'),
                             ('turn_intensity', r'TI:([\d.]+)'), ('gait', r'Gait:(\d)'),
                             ('terrain_mult', r'TM:([\d.]+)'), ('stall_mult', r'SM:([\d.]+)'),
                             ('flicker', r'Flk:(\d+)')]:
                m = re.search(pat, line)
                if m:
                    val = m.group(1)
                    rec[key] = float(val) if '.' in val else int(val)

            imp_match = re.search(r'Imp:(\d+)/(\d+)', line)
            if imp_match:
                rec['impact_start'] = int(imp_match.group(1))
                rec['impact_end'] = int(imp_match.group(2))

            if 'time_s' in rec:
                records.append(rec)
    return records


# ── Quaternion to Euler ─────────────────────────────────────────────────

def quat_to_euler(w, x, y, z):
    """Convert quaternion to roll, pitch, yaw in degrees."""
    # Roll (x-axis)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis)
    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Yaw (z-axis)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


# ── Plotting ────────────────────────────────────────────────────────────

# Data order in telemetry is ALL_SERVOS: [2, 3, 4, 1, 6, 5] (left then right)
SERVO_NAMES = ['S2', 'S3', 'S4', 'S1', 'S6', 'S5']  # matches ALL_SERVOS order in gait engine
SERVO_COLORS = ['#377eb8', '#4daf4a', '#984ea3', '#e41a1c', '#a65628', '#ff7f00']
STALL_THRESHOLD = 750
TEMP_MAX = 65
VOLTAGE_MIN = 10.5

# DUPLICATED FROM final_full_gait_test.py -- update both files when changing
ENCODER_RESOLUTION = 4096.0
HOME_POSITIONS = {1: 3447, 2: 955, 3: 1420, 4: 1569, 5: 3197, 6: 3175}
DIRECTION_MAP = {1: 1, 2: -1, 3: -1, 4: -1, 5: 1, 6: 1}
LEG_SPLAY = {1: 35.0, 2: 35.0, 3: 0.0, 4: 0.0, 5: -35.0, 6: -35.0}  # degrees
GAITS = {
    0: {'name': 'Tripod', 'duty': 0.5, 'offsets': {2: 0.0, 6: 0.0, 4: 0.0, 1: 0.5, 3: 0.5, 5: 0.5}},
    1: {'name': 'Wave', 'duty': 0.75, 'offsets': {4: 0.833, 3: 0.666, 2: 0.5, 5: 0.333, 6: 0.166, 1: 0.0}},
    2: {'name': 'Quadruped', 'duty': 0.7, 'offsets': {2: 0.0, 5: 0.0, 3: 0.333, 6: 0.333, 4: 0.666, 1: 0.666}},
}
ALL_SERVOS = [2, 3, 4, 1, 6, 5]
KAPPA_TRANSITION = 8.0
RAMP_95_PCT = 0.375  # 3/KAPPA = 3/8.0 = 95% settled time


def normalize_time(records):
    """Normalize timestamps to start at 0."""
    if not records:
        return records
    t0 = records[0].get('time_s', 0)
    for r in records:
        r['t'] = r.get('time_s', 0) - t0
    return records


def plot_voltage(records, output_dir):
    """Plot battery voltage over time."""
    times = [r['t'] for r in records if 'voltage' in r]
    volts = [r['voltage'] for r in records if 'voltage' in r]
    if not times:
        return

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(times, volts, 'b-', linewidth=1.5, label='Voltage')
    ax.axhline(y=VOLTAGE_MIN, color='r', linestyle='--', alpha=0.7, label=f'Shutdown ({VOLTAGE_MIN}V)')
    ax.axhline(y=11.0, color='orange', linestyle='--', alpha=0.5, label='Warning (11.0V)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Voltage (V)')
    ax.set_title('Battery Voltage Timeline')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'voltage_timeline.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] voltage_timeline.png")


def plot_servo_loads(records, output_dir):
    """Plot per-servo load over time with stall threshold."""
    times = [r['t'] for r in records if 'loads' in r]
    loads_by_servo = [[] for _ in range(6)]
    for r in records:
        if 'loads' in r and len(r['loads']) == 6:
            for i in range(6):
                loads_by_servo[i].append(r['loads'][i])

    if not times:
        return

    fig, ax = plt.subplots(figsize=(12, 5))
    for i in range(6):
        if len(loads_by_servo[i]) == len(times):
            ax.plot(times, loads_by_servo[i], color=SERVO_COLORS[i],
                    linewidth=0.8, alpha=0.8, label=SERVO_NAMES[i])
    ax.axhline(y=STALL_THRESHOLD, color='r', linestyle='--', alpha=0.7,
               label=f'Stall threshold ({STALL_THRESHOLD})')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Load (raw STS units)')
    ax.set_title('Per-Servo Load')
    ax.legend(ncol=7, fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'servo_loads.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] servo_loads.png")


def plot_servo_temps(records, output_dir):
    """Plot per-servo temperature over time."""
    times = [r['t'] for r in records if 'temps' in r]
    temps_by_servo = [[] for _ in range(6)]
    for r in records:
        if 'temps' in r and len(r['temps']) == 6:
            for i in range(6):
                t = r['temps'][i]
                if t == 125:  # Ghost temperature — EMI artifact
                    t = None
                temps_by_servo[i].append(t)

    if not times:
        return

    fig, ax = plt.subplots(figsize=(12, 4))
    for i in range(6):
        valid_times = [times[j] for j in range(len(temps_by_servo[i]))
                       if temps_by_servo[i][j] is not None]
        valid_temps = [t for t in temps_by_servo[i] if t is not None]
        if valid_temps:
            ax.plot(valid_times, valid_temps, color=SERVO_COLORS[i],
                    linewidth=0.8, alpha=0.8, label=SERVO_NAMES[i])
    ax.axhline(y=TEMP_MAX, color='r', linestyle='--', alpha=0.7,
               label=f'Shutdown ({TEMP_MAX}°C)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Temperature (°C)')
    ax.set_title('Per-Servo Temperature (125°C ghost readings filtered)')
    ax.legend(ncol=7, fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'servo_temps.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] servo_temps.png")


def plot_governor_phase(records, output_dir):
    """Plot governor clamp rate and phase error."""
    times = [r['t'] for r in records if 'governor' in r]
    gov = [r['governor'] for r in records if 'governor' in r]
    pe_times = [r['t'] for r in records if 'phase_err' in r]
    phase_err = [abs(r['phase_err']) for r in records if 'phase_err' in r]

    if not times:
        return

    fig, ax1 = plt.subplots(figsize=(12, 4))
    ax1.plot(times, gov, 'b-', linewidth=1, alpha=0.8, label='Governor %')
    ax1.axhline(y=50, color='b', linestyle='--', alpha=0.3, label='50% (speed-limited)')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Governor Clamp Rate (%)', color='b')
    ax1.tick_params(axis='y', labelcolor='b')

    ax2 = ax1.twinx()
    if phase_err:
        ax2.plot(pe_times, phase_err, 'r-', linewidth=1, alpha=0.8, label='|Phase Error|')
        ax2.axhline(y=15, color='r', linestyle='--', alpha=0.3, label='15° (desync)')
    ax2.set_ylabel('Phase Error (°)', color='r')
    ax2.tick_params(axis='y', labelcolor='r')

    ax1.set_title('Governor Clamp Rate + Phase Error')
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=8)
    ax1.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'governor_phase.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] governor_phase.png")


def plot_loop_timing(records, output_dir):
    """Plot loop cycle time and jitter."""
    times = [r['t'] for r in records if 'loop_ms' in r]
    loop = [r['loop_ms'] for r in records if 'loop_ms' in r]
    jit_times = [r['t'] for r in records if 'jitter_max' in r]
    jitter = [r['jitter_max'] for r in records if 'jitter_max' in r]

    if not times:
        return

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(times, loop, 'b-', linewidth=1, alpha=0.8, label='Loop time')
    if jitter:
        ax.plot(jit_times, jitter, 'r-', linewidth=0.8, alpha=0.6, label='Jitter max')
    ax.axhline(y=20, color='g', linestyle='--', alpha=0.5, label='Target (20ms)')
    ax.axhline(y=25, color='orange', linestyle='--', alpha=0.5, label='Overrun (25ms)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Time (ms)')
    ax.set_title('Heart Loop Timing')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'loop_timing.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] loop_timing.png")


def plot_stall_events(records, output_dir):
    """Plot stall events as a timeline."""
    stall_data = []
    for r in records:
        if 'stall' in r and r['stall']:
            for sid in r['stall']:
                stall_data.append((r['t'], sid))

    times = [r['t'] for r in records if 'loads' in r]
    if not times:
        return

    fig, ax = plt.subplots(figsize=(12, 3))
    servo_ids = ['s1', 's2', 's3', 's4', 's5', 's6']
    for i, sid in enumerate(servo_ids):
        stall_times = [t for t, s in stall_data if s == sid]
        if stall_times:
            ax.scatter(stall_times, [i + 1] * len(stall_times),
                       color=SERVO_COLORS[i], s=20, marker='s', label=SERVO_NAMES[i])

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Servo')
    ax.set_yticks(range(1, 7))
    ax.set_yticklabels(SERVO_NAMES)
    ax.set_title('Stall Events Timeline')
    ax.set_xlim(times[0], times[-1])
    ax.grid(True, alpha=0.3, axis='x')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'stall_events.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] stall_events.png")


def plot_sensor_distances(arduino_records, output_dir):
    """Plot ultrasonic sensor distances over time."""
    if not arduino_records:
        return

    times = [r['time_s'] for r in arduino_records]
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    front_cols = ['FDL', 'FCF', 'FCD', 'FDR']
    rear_cols = ['RDL', 'RCF', 'RCD', 'RDR']
    colors_front = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3']
    colors_rear = ['#ff7f00', '#a65628', '#f781bf', '#999999']

    for i, col in enumerate(front_cols):
        vals = [min(r.get(col, 300), 300) for r in arduino_records]
        axes[0].plot(times, vals, color=colors_front[i], linewidth=0.8, alpha=0.8, label=col)
    axes[0].axhline(y=15, color='r', linestyle='--', alpha=0.3, label='DANGER (15cm)')
    axes[0].axhline(y=40, color='orange', linestyle='--', alpha=0.3, label='NEAR (40cm)')
    axes[0].set_ylabel('Distance (cm)')
    axes[0].set_title('Front Ultrasonics')
    axes[0].legend(ncol=6, fontsize=7)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_ylim(-5, 310)

    for i, col in enumerate(rear_cols):
        vals = [min(r.get(col, 300), 300) for r in arduino_records]
        axes[1].plot(times, vals, color=colors_rear[i], linewidth=0.8, alpha=0.8, label=col)
    axes[1].axhline(y=15, color='r', linestyle='--', alpha=0.3, label='DANGER')
    axes[1].axhline(y=40, color='orange', linestyle='--', alpha=0.3, label='NEAR')
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Distance (cm)')
    axes[1].set_title('Rear Ultrasonics')
    axes[1].legend(ncol=6, fontsize=7)
    axes[1].grid(True, alpha=0.3)
    axes[1].set_ylim(-5, 310)

    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'sensor_distances.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] sensor_distances.png")


def plot_imu_orientation(arduino_records, output_dir):
    """Plot IMU-derived roll, pitch, yaw over time."""
    if not arduino_records:
        return

    times, rolls, pitches, yaws = [], [], [], []
    for r in arduino_records:
        if all(k in r for k in ('QuatW', 'QuatX', 'QuatY', 'QuatZ')):
            roll, pitch, yaw = quat_to_euler(r['QuatW'], r['QuatX'], r['QuatY'], r['QuatZ'])
            times.append(r['time_s'])
            rolls.append(roll)
            pitches.append(pitch)
            yaws.append(yaw)

    if not times:
        return

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(times, rolls, 'r-', linewidth=0.8)
    axes[0].set_ylabel('Roll (°)')
    axes[0].set_title('IMU Orientation (Game Rotation Vector — boot-relative)')
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times, pitches, 'g-', linewidth=0.8)
    axes[1].set_ylabel('Pitch (°)')
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(times, yaws, 'b-', linewidth=0.8)
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Yaw (°)')
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'imu_orientation.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] imu_orientation.png")


def plot_run_summary(records, brain_events, arduino_records, output_dir):
    """Combined overview dashboard."""
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)

    times = [r['t'] for r in records]
    t_max = times[-1] if times else 60

    # Panel 1: Voltage + Current
    v_times = [r['t'] for r in records if 'voltage' in r]
    volts = [r['voltage'] for r in records if 'voltage' in r]
    if volts:
        axes[0].plot(v_times, volts, 'b-', linewidth=1, label='Voltage')
        axes[0].axhline(y=VOLTAGE_MIN, color='r', linestyle='--', alpha=0.5)
    a_times = [r['t'] for r in records if 'amps' in r]
    amps = [r['amps'] for r in records if 'amps' in r]
    if amps:
        ax0b = axes[0].twinx()
        ax0b.plot(a_times, amps, 'orange', linewidth=0.8, alpha=0.7, label='Current')
        ax0b.set_ylabel('Amps', color='orange')
    axes[0].set_ylabel('Voltage (V)')
    axes[0].set_title('Run Overview Dashboard')
    axes[0].grid(True, alpha=0.3)

    # Panel 2: Max load across all servos
    max_loads = []
    load_times = []
    for r in records:
        if 'loads' in r and len(r['loads']) == 6:
            load_times.append(r['t'])
            max_loads.append(max(r['loads']))
    if max_loads:
        axes[1].plot(load_times, max_loads, 'purple', linewidth=0.8)
        axes[1].axhline(y=STALL_THRESHOLD, color='r', linestyle='--', alpha=0.5,
                         label=f'Stall ({STALL_THRESHOLD})')
        axes[1].set_ylabel('Max Servo Load')
        axes[1].legend(fontsize=8)
        axes[1].grid(True, alpha=0.3)

    # Panel 3: Governor + Phase error
    gov_times = [r['t'] for r in records if 'governor' in r]
    gov = [r['governor'] for r in records if 'governor' in r]
    if gov:
        axes[2].plot(gov_times, gov, 'b-', linewidth=0.8, label='Governor %')
    pe_times = [r['t'] for r in records if 'phase_err' in r]
    pe = [abs(r['phase_err']) for r in records if 'phase_err' in r]
    if pe:
        ax2b = axes[2].twinx()
        ax2b.plot(pe_times, pe, 'r-', linewidth=0.8, alpha=0.7, label='|PhErr|')
        ax2b.set_ylabel('Phase Error (°)', color='r')
    axes[2].set_ylabel('Governor %')
    axes[2].grid(True, alpha=0.3)

    # Panel 4: Brain events as markers
    if brain_events:
        event_times = [e.get('time_s', 0) for e in brain_events if 'time_s' in e]
        event_tags = [e.get('tag', '?') for e in brain_events if 'time_s' in e]
        tag_colors = {
            'NAV': 'blue', 'WIGGLE-START': 'red', 'WIGGLE-END': 'green',
            'ROLL-CHECK': 'orange', 'STALL': 'purple'
        }
        for t, tag in zip(event_times, event_tags):
            c = tag_colors.get(tag, 'gray')
            axes[3].axvline(x=t, color=c, alpha=0.6, linewidth=1.5)
        patches = [mpatches.Patch(color=c, label=t) for t, c in tag_colors.items()]
        axes[3].legend(handles=patches, ncol=5, fontsize=7)
    axes[3].set_xlabel('Time (s)')
    axes[3].set_ylabel('Events')
    axes[3].set_title('Brain Events')
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'run_summary.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] run_summary.png")


# ── Verbose telemetry plots ([H5], [BS], [BN]) ────────────────────────

def plot_servo_positions(h5_records, output_dir):
    """Plot per-servo encoder positions over time from [H5] data."""
    if not h5_records or 'positions' not in h5_records[0]:
        return
    times = [r['time_s'] for r in h5_records if 'positions' in r]
    pos_by_servo = [[] for _ in range(6)]
    for r in h5_records:
        if 'positions' in r and len(r['positions']) == 6:
            for i in range(6):
                pos_by_servo[i].append(r['positions'][i])

    fig, ax = plt.subplots(figsize=(12, 5))
    for i in range(6):
        if len(pos_by_servo[i]) == len(times):
            ax.plot(times, pos_by_servo[i], color=SERVO_COLORS[i],
                    linewidth=0.8, alpha=0.8, label=SERVO_NAMES[i])
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Encoder Position (ticks)')
    ax.set_title('Per-Servo Encoder Positions (5Hz [H5])')
    ax.legend(ncol=6, fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'servo_positions.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] servo_positions.png")


def plot_servo_cmd_vs_actual(h5_records, output_dir):
    """Plot commanded speed vs estimated actual velocity from position derivative."""
    if not h5_records or len(h5_records) < 3:
        return
    times = [r['time_s'] for r in h5_records if 'positions' in r and 'speeds' in r]
    if len(times) < 3:
        return

    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    axes = axes.flatten()

    for servo_idx in range(6):
        cmd_times = []
        cmd_speeds = []
        est_times = []
        est_speeds = []
        for j in range(len(h5_records)):
            r = h5_records[j]
            if 'speeds' not in r or 'positions' not in r:
                continue
            if len(r['speeds']) > servo_idx:
                cmd_times.append(r['time_s'])
                cmd_speeds.append(r['speeds'][servo_idx])
            # Estimate actual velocity from position derivative
            if j > 0 and 'positions' in h5_records[j-1]:
                dt = r['time_s'] - h5_records[j-1]['time_s']
                if dt > 0.01:
                    dp = r['positions'][servo_idx] - h5_records[j-1]['positions'][servo_idx]
                    # Handle encoder wraparound
                    if abs(dp) > 2048:
                        dp = dp - 4096 if dp > 0 else dp + 4096
                    est_speeds.append(dp / dt)  # ticks/sec
                    est_times.append(r['time_s'])

        ax = axes[servo_idx]
        if cmd_speeds:
            ax.plot(cmd_times, cmd_speeds, color=SERVO_COLORS[servo_idx],
                    linewidth=0.8, alpha=0.8, label='Commanded')
        if est_speeds:
            ax.plot(est_times, est_speeds, color='gray',
                    linewidth=0.5, alpha=0.5, label='Actual (est.)')
        ax.set_title(SERVO_NAMES[servo_idx], fontsize=10)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    axes[4].set_xlabel('Time (s)')
    axes[5].set_xlabel('Time (s)')
    fig.suptitle('Commanded vs Actual Speed per Servo (5Hz [H5])', fontsize=12)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'servo_cmd_vs_actual.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] servo_cmd_vs_actual.png")


def plot_nav_timeline(bn_records, bs_records, output_dir):
    """Plot FSM state timeline with speed/turn overlays and sector severity."""
    if not bn_records:
        return

    times = [r['time_s'] for r in bn_records]
    states = [r.get('state', '?') for r in bn_records]
    speeds = [r.get('speed', 0) for r in bn_records]
    turns = [r.get('turn', 0) for r in bn_records]

    # Assign unique colors to states
    unique_states = list(dict.fromkeys(states))
    state_cmap = plt.cm.get_cmap('tab10', max(len(unique_states), 1))
    state_colors = {s: state_cmap(i) for i, s in enumerate(unique_states)}

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)

    # Panel 1: FSM state as colored bands
    for i in range(len(times) - 1):
        axes[0].axvspan(times[i], times[i+1], color=state_colors[states[i]], alpha=0.6)
    if times:
        axes[0].axvspan(times[-1], times[-1] + 0.5, color=state_colors[states[-1]], alpha=0.6)
    patches = [mpatches.Patch(color=state_colors[s], label=s) for s in unique_states]
    axes[0].legend(handles=patches, ncol=min(len(unique_states), 5), fontsize=7, loc='upper right')
    axes[0].set_ylabel('FSM State')
    axes[0].set_title('Navigation Timeline ([BN] decisions)')
    axes[0].set_yticks([])

    # Panel 2: Speed + Turn
    axes[1].plot(times, speeds, 'b-', linewidth=1, label='Speed')
    axes[1].set_ylabel('Speed', color='b')
    ax1b = axes[1].twinx()
    ax1b.plot(times, turns, 'r-', linewidth=0.8, alpha=0.7, label='Turn')
    ax1b.set_ylabel('Turn', color='r')
    axes[1].grid(True, alpha=0.3)

    # Panel 3: Sector severity from [BN]
    front_sev = [r.get('front_sev', 0) for r in bn_records]
    left_sev = [r.get('left_sev', 0) for r in bn_records]
    right_sev = [r.get('right_sev', 0) for r in bn_records]
    axes[2].plot(times, front_sev, 'r-', linewidth=1, label='Front')
    axes[2].plot(times, left_sev, 'b-', linewidth=1, alpha=0.7, label='Left')
    axes[2].plot(times, right_sev, 'g-', linewidth=1, alpha=0.7, label='Right')
    axes[2].set_ylabel('Severity (0-3)')
    axes[2].set_yticks([0, 1, 2, 3])
    axes[2].set_yticklabels(['CLEAR', 'CAUTION', 'NEAR', 'DANGER'])
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.3)

    # Panel 4: Sensor distances from [BS] if available
    if bs_records:
        bs_times = [r['time_s'] for r in bs_records]
        for col, color, label in [('FCF', '#e41a1c', 'FCF'), ('FDL', '#377eb8', 'FDL'),
                                   ('FDR', '#4daf4a', 'FDR'), ('RCF', '#984ea3', 'RCF')]:
            vals = [min(r.get(col, 300), 300) for r in bs_records]
            axes[3].plot(bs_times, vals, color=color, linewidth=0.8, alpha=0.8, label=label)
        axes[3].axhline(y=15, color='r', linestyle='--', alpha=0.3, label='DANGER')
        axes[3].axhline(y=40, color='orange', linestyle='--', alpha=0.3, label='NEAR')
        axes[3].legend(ncol=6, fontsize=7)
        axes[3].set_ylabel('Distance (cm)')
    else:
        axes[3].text(0.5, 0.5, 'No [BS] sensor data', ha='center', va='center',
                     transform=axes[3].transAxes, fontsize=12, color='gray')
    axes[3].set_xlabel('Time (s)')
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'nav_timeline.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] nav_timeline.png")


def get_buehler_angle(phase_fraction, duty, impact_start_deg, impact_end_deg, is_reversed=False):
    """Compute Buehler clock angle -- matches gait engine formula exactly."""
    start = impact_end_deg if is_reversed else impact_start_deg
    end = impact_start_deg if is_reversed else impact_end_deg
    stance_sweep = (end - start + 180) % 360 - 180
    if is_reversed:
        stance_sweep = -stance_sweep
    air_sign = -1 if stance_sweep >= 0 else 1
    air_sweep = air_sign * (360 - abs(stance_sweep))
    phase = phase_fraction % 1.0
    if phase < duty:
        stance_frac = phase / duty
        angle = start + stance_frac * stance_sweep
    else:
        swing_frac = (phase - duty) / (1.0 - duty)
        angle = end + swing_frac * air_sweep
    return angle % 360


def plot_phase_timeline(h5_records, output_dir):
    if not h5_records or 'phases' not in h5_records[0]:
        return
    times = []
    phases_by_servo = [[] for _ in range(6)]
    for r in h5_records:
        if 'phases' in r and len(r['phases']) == 6:
            times.append(r['time_s'])
            for i in range(6):
                phases_by_servo[i].append(r['phases'][i])

    fig, ax = plt.subplots(figsize=(12, 5))
    for i in range(6):
        if len(phases_by_servo[i]) == len(times):
            ax.plot(times, phases_by_servo[i], color=SERVO_COLORS[i],
                    linewidth=0.8, alpha=0.8, label=SERVO_NAMES[i])

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Buehler Angle (degrees)')
    ax.set_title('Phase Timeline (Buehler Angles)')
    ax.legend(ncol=7, fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'phase_timeline.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] phase_timeline.png")


def plot_phase_error(h5_records, output_dir):
    """Per-servo phase error (target - actual) over time. Requires PhT: field."""
    recs = [r for r in h5_records
            if 'phases' in r and 'target_phases' in r
            and len(r['phases']) == 6 and len(r['target_phases']) == 6]
    if not recs:
        return
    times = [r['time_s'] for r in recs]
    errors_by_servo = [[] for _ in range(6)]
    for r in recs:
        for i in range(6):
            err = r['target_phases'][i] - r['phases'][i]
            # Wrap to [-180, 180]
            err = (err + 180) % 360 - 180
            errors_by_servo[i].append(err)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

    # Top: per-servo phase error
    for i in range(6):
        ax1.plot(times, errors_by_servo[i], color=SERVO_COLORS[i],
                 linewidth=0.6, alpha=0.7, label=SERVO_NAMES[i])
    ax1.axhline(y=0, color='black', linewidth=0.5, linestyle='--')
    ax1.set_ylabel('Phase Error (degrees)')
    ax1.set_title('Per-Servo Phase Error (target - actual)')
    ax1.legend(ncol=6, fontsize=8)
    ax1.grid(True, alpha=0.3)

    # Bottom: max absolute error across all servos (what governor sees)
    max_errors = [max(abs(errors_by_servo[i][j]) for i in range(6))
                  for j in range(len(times))]
    ax2.plot(times, max_errors, color='#e41a1c', linewidth=0.8)
    ax2.axhline(y=30, color='orange', linewidth=1, linestyle='--',
                label='Governor engage (30 deg)')
    ax2.axhline(y=20, color='green', linewidth=1, linestyle='--',
                label='Governor release (20 deg)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Max |Phase Error| (degrees)')
    ax2.set_title('Max Phase Error vs Governor Thresholds')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'phase_error.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] phase_error.png")


def plot_gait_transitions(bn_records, output_dir):
    if not bn_records:
        return
    times = [r['time_s'] for r in bn_records if 'gait' in r]
    gaits = [r['gait'] for r in bn_records if 'gait' in r]
    if not times:
        return

    gait_names = {0: 'Tripod', 1: 'Wave', 2: 'Quadruped'}

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.step(times, gaits, where='post', color='#377eb8', linewidth=1.5)

    # Draw transition bands where gait changes
    for i in range(1, len(gaits)):
        if gaits[i] != gaits[i - 1]:
            t = times[i]
            ax.axvspan(t, t + RAMP_95_PCT, color='orange', alpha=0.3)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Gait')
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels([gait_names[k] for k in [0, 1, 2]])
    ax.set_title('Gait Transitions with CPG Ramp Windows')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'gait_transitions.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] gait_transitions.png")


def plot_real_vs_theoretical(h5_records, bn_records, output_dir):
    if not h5_records or not bn_records:
        return

    # Build time-indexed gait schedule from bn_records
    gait_schedule = []
    for r in bn_records:
        if 'gait' in r and 'time_s' in r:
            gait_schedule.append({
                'time_s': r['time_s'],
                'gait': r['gait'],
                'impact_start': r.get('impact_start', 0),
                'impact_end': r.get('impact_end', 180),
            })
    if not gait_schedule:
        return

    def lookup_gait(t):
        result = gait_schedule[0]
        for gs in gait_schedule:
            if gs['time_s'] <= t:
                result = gs
            else:
                break
        return result

    # Find gait transition times for vertical lines
    transition_times = []
    for i in range(1, len(gait_schedule)):
        if gait_schedule[i]['gait'] != gait_schedule[i - 1]['gait']:
            transition_times.append(gait_schedule[i]['time_s'])

    fig, axes = plt.subplots(2, 3, figsize=(16, 8), sharex=True)
    axes = axes.flatten()

    for idx in range(6):
        sid = ALL_SERVOS[idx]
        times_real = []
        real_deg = []
        theo_deg = []

        for r in h5_records:
            if 'positions' not in r or len(r['positions']) <= idx:
                continue
            if 'phases' not in r or len(r['phases']) <= idx:
                continue
            t = r['time_s']
            times_real.append(t)

            # Real position converted to degrees
            pos = r['positions'][idx]
            deg = (((pos - HOME_POSITIONS[sid]) * DIRECTION_MAP[sid] / ENCODER_RESOLUTION) * 360.0) % 360
            real_deg.append(deg)

            # Theoretical Buehler angle
            gs = lookup_gait(t)
            gait_info = GAITS.get(gs['gait'], GAITS[0])
            offset = gait_info['offsets'].get(sid, 0.0)
            # Use the phase from h5 record to derive phase fraction
            # phases are Buehler angles; compute theoretical from gait params
            phase_frac = (r['phases'][idx] / 360.0) if r['phases'][idx] > 0 else 0.0
            # Instead, compute theoretical from offset + elapsed cycles
            # Use raw phase fraction: offset is the starting offset for this servo
            theo_angle = get_buehler_angle(
                phase_frac, gait_info['duty'],
                float(gs['impact_start']), float(gs['impact_end'])
            )
            theo_deg.append(theo_angle)

        ax = axes[idx]
        if times_real:
            ax.plot(times_real, real_deg, color=SERVO_COLORS[idx],
                    linewidth=0.8, alpha=0.8, label='Encoder (raw)')
            ax.plot(times_real, theo_deg, color=SERVO_COLORS[idx],
                    linewidth=0.8, alpha=0.6, linestyle='--', label='Buehler (from phase)')
        for tt in transition_times:
            ax.axvline(x=tt, color='gray', linestyle=':', alpha=0.5)
        ax.set_title(SERVO_NAMES[idx], fontsize=10)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    axes[3].set_xlabel('Time (s)')
    axes[4].set_xlabel('Time (s)')
    axes[5].set_xlabel('Time (s)')
    fig.suptitle('Encoder Position vs Buehler Mapping', fontsize=12)
    fig.text(0.5, 0.01, 'Note: Buehler trace derived from recorded phase data, not independent theoretical prediction',
             ha='center', fontsize=8, color='gray')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'encoder_vs_buehler.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] encoder_vs_buehler.png")


# ── Run stats extraction ────────────────────────────────────────────────

def extract_run_stats(records):
    """Extract summary statistics from a parsed telemetry run.

    Returns a dict of key metrics for comparison across runs.
    """
    stats = {}
    if not records:
        return stats

    # Duration
    stats['duration_s'] = records[-1].get('t', records[-1].get('time_s', 0))

    # Voltage
    volts = [r['voltage'] for r in records if 'voltage' in r]
    if volts:
        stats['voltage_min'] = min(volts)
        stats['voltage_max'] = max(volts)
        stats['voltage_mean'] = sum(volts) / len(volts)
        stats['voltage_drop'] = volts[0] - volts[-1] if len(volts) > 1 else 0.0

    # Loads
    all_max_loads = []
    per_servo_loads = {i: [] for i in range(6)}
    for r in records:
        if 'loads' in r and len(r['loads']) == 6:
            all_max_loads.append(max(r['loads']))
            for i, load in enumerate(r['loads']):
                per_servo_loads[i].append(load)
    if all_max_loads:
        stats['load_max'] = max(all_max_loads)
        stats['load_mean'] = sum(all_max_loads) / len(all_max_loads)
        stats['load_p95'] = sorted(all_max_loads)[int(len(all_max_loads) * 0.95)]
    for i in range(6):
        if per_servo_loads[i]:
            stats[f'load_{SERVO_NAMES[i]}_max'] = max(per_servo_loads[i])
            stats[f'load_{SERVO_NAMES[i]}_mean'] = sum(per_servo_loads[i]) / len(per_servo_loads[i])

    # Stalls
    stall_frames = [r for r in records if 'stall' in r and r['stall']]
    stats['stall_count'] = len(stall_frames)
    stall_servos = {}
    for r in stall_frames:
        for sid in r['stall']:
            stall_servos[sid] = stall_servos.get(sid, 0) + 1
    stats['stall_by_servo'] = stall_servos

    # Governor
    gov = [r['governor'] for r in records if 'governor' in r]
    if gov:
        stats['governor_max'] = max(gov)
        stats['governor_mean'] = sum(gov) / len(gov)
        stats['governor_above_50'] = sum(1 for g in gov if g > 50) / len(gov) * 100

    # Phase error
    pe = [abs(r['phase_err']) for r in records if 'phase_err' in r]
    if pe:
        stats['phase_err_max'] = max(pe)
        stats['phase_err_mean'] = sum(pe) / len(pe)

    # Temperatures
    max_temps = []
    for r in records:
        if 'temps' in r and len(r['temps']) == 6:
            real_temps = [t for t in r['temps'] if t != 125]  # filter EMI ghosts
            if real_temps:
                max_temps.append(max(real_temps))
    if max_temps:
        stats['temp_max'] = max(max_temps)
        stats['temp_mean'] = sum(max_temps) / len(max_temps)

    # Loop timing
    loops = [r['loop_ms'] for r in records if 'loop_ms' in r]
    if loops:
        stats['loop_max_ms'] = max(loops)
        stats['loop_mean_ms'] = sum(loops) / len(loops)
        stats['loop_overruns'] = sum(1 for l in loops if l > 22)

    # Gaits used
    gaits = [r['gait'] for r in records if 'gait' in r]
    if gaits:
        stats['gaits_used'] = list(set(gaits))

    # TE cycles
    te_total = 0
    for r in records:
        if 'te_cycles' in r:
            te_total += sum(r['te_cycles'])
    stats['te_cycles_total'] = te_total

    return stats


def format_comparison_table(current_stats, baseline_stats):
    """Format a comparison table between current run and baseline run.

    Returns a list of lines (strings) for the delta table.
    """
    lines = []
    lines.append("=" * 78)
    lines.append("  TELEMETRY COMPARISON: CURRENT vs BASELINE")
    lines.append("=" * 78)
    lines.append(f"{'Metric':<30} {'Current':>12} {'Baseline':>12} {'Delta':>12} {'Verdict':>8}")
    lines.append("-" * 78)

    def row(label, key, fmt=".1f", lower_better=True, threshold=None):
        cur = current_stats.get(key)
        base = baseline_stats.get(key)
        if cur is None and base is None:
            return
        cur_s = f"{cur:{fmt}}" if cur is not None else "N/A"
        base_s = f"{base:{fmt}}" if base is not None else "N/A"
        if cur is not None and base is not None:
            delta = cur - base
            delta_s = f"{delta:+{fmt}}"
            if threshold is not None:
                if abs(delta) < threshold:
                    verdict = "  ~"
                elif (lower_better and delta > 0) or (not lower_better and delta < 0):
                    verdict = " WORSE"
                else:
                    verdict = " BETTER"
            else:
                if (lower_better and delta > 0) or (not lower_better and delta < 0):
                    verdict = " WORSE"
                elif delta == 0:
                    verdict = "  ~"
                else:
                    verdict = " BETTER"
        else:
            delta_s = "N/A"
            verdict = "  ?"
        lines.append(f"{label:<30} {cur_s:>12} {base_s:>12} {delta_s:>12} {verdict:>8}")

    row("Duration (s)", "duration_s", ".0f", lower_better=False, threshold=5)
    row("Voltage min (V)", "voltage_min", ".2f", lower_better=False)
    row("Voltage mean (V)", "voltage_mean", ".2f", lower_better=False)
    row("Voltage drop (V)", "voltage_drop", ".2f", lower_better=True)
    lines.append("-" * 78)
    row("Load max (raw)", "load_max", ".0f", lower_better=True)
    row("Load mean (raw)", "load_mean", ".0f", lower_better=True)
    row("Load P95 (raw)", "load_p95", ".0f", lower_better=True)
    for name in SERVO_NAMES:
        row(f"  {name} load max", f"load_{name}_max", ".0f", lower_better=True)
    lines.append("-" * 78)
    row("Stall events", "stall_count", ".0f", lower_better=True)
    row("Governor max (%)", "governor_max", ".1f", lower_better=True)
    row("Governor mean (%)", "governor_mean", ".1f", lower_better=True)
    row("Governor >50% (%frames)", "governor_above_50", ".1f", lower_better=True)
    lines.append("-" * 78)
    row("Phase error max (deg)", "phase_err_max", ".1f", lower_better=True)
    row("Phase error mean (deg)", "phase_err_mean", ".1f", lower_better=True)
    row("Temp max (C)", "temp_max", ".1f", lower_better=True)
    row("Temp mean (C)", "temp_mean", ".1f", lower_better=True)
    lines.append("-" * 78)
    row("Loop max (ms)", "loop_max_ms", ".1f", lower_better=True)
    row("Loop mean (ms)", "loop_mean_ms", ".1f", lower_better=True)
    row("Loop overruns (>22ms)", "loop_overruns", ".0f", lower_better=True)
    row("TE recovery cycles", "te_cycles_total", ".0f", lower_better=True)
    lines.append("=" * 78)

    # Stall breakdown comparison
    cur_stalls = current_stats.get('stall_by_servo', {})
    base_stalls = baseline_stats.get('stall_by_servo', {})
    if cur_stalls or base_stalls:
        lines.append("\nStall breakdown by servo:")
        all_sids = sorted(set(list(cur_stalls.keys()) + list(base_stalls.keys())))
        for sid in all_sids:
            c = cur_stalls.get(sid, 0)
            b = base_stalls.get(sid, 0)
            delta = c - b
            lines.append(f"  {sid}: {c} (was {b}, {delta:+d})")

    # Gaits comparison
    cur_gaits = current_stats.get('gaits_used', [])
    base_gaits = baseline_stats.get('gaits_used', [])
    if cur_gaits or base_gaits:
        lines.append(f"\nGaits used: {', '.join(sorted(cur_gaits))} (baseline: {', '.join(sorted(base_gaits))})")

    return lines


def plot_comparison_overlay(current_records, baseline_records, output_dir):
    """Generate overlay plots comparing current vs baseline run."""
    fig, axes = plt.subplots(4, 1, figsize=(14, 14), sharex=False)

    # Normalize baseline times
    if baseline_records and 't' not in baseline_records[0]:
        baseline_records = normalize_time(baseline_records)

    # Panel 1: Voltage overlay
    for recs, label, color, alpha in [
        (current_records, 'Current', '#2196F3', 1.0),
        (baseline_records, 'Baseline', '#9E9E9E', 0.6),
    ]:
        vt = [r['t'] for r in recs if 'voltage' in r]
        vv = [r['voltage'] for r in recs if 'voltage' in r]
        if vv:
            axes[0].plot(vt, vv, color=color, linewidth=1.2, alpha=alpha, label=label)
    axes[0].axhline(y=VOLTAGE_MIN, color='r', linestyle='--', alpha=0.3)
    axes[0].set_ylabel('Voltage (V)')
    axes[0].set_title('Current vs Baseline Comparison')
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    # Panel 2: Max load overlay
    for recs, label, color, alpha in [
        (current_records, 'Current', '#E91E63', 1.0),
        (baseline_records, 'Baseline', '#9E9E9E', 0.6),
    ]:
        lt = []
        ll = []
        for r in recs:
            if 'loads' in r and len(r['loads']) == 6:
                lt.append(r['t'])
                ll.append(max(r['loads']))
        if ll:
            axes[1].plot(lt, ll, color=color, linewidth=0.8, alpha=alpha, label=label)
    axes[1].axhline(y=STALL_THRESHOLD, color='r', linestyle='--', alpha=0.3,
                     label=f'Stall ({STALL_THRESHOLD})')
    axes[1].set_ylabel('Max Servo Load')
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    # Panel 3: Governor overlay
    for recs, label, color, alpha in [
        (current_records, 'Current', '#4CAF50', 1.0),
        (baseline_records, 'Baseline', '#9E9E9E', 0.6),
    ]:
        gt = [r['t'] for r in recs if 'governor' in r]
        gg = [r['governor'] for r in recs if 'governor' in r]
        if gg:
            axes[2].plot(gt, gg, color=color, linewidth=0.8, alpha=alpha, label=label)
    axes[2].set_ylabel('Governor %')
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.3)

    # Panel 4: Phase error overlay
    for recs, label, color, alpha in [
        (current_records, 'Current', '#FF5722', 1.0),
        (baseline_records, 'Baseline', '#9E9E9E', 0.6),
    ]:
        pt = [r['t'] for r in recs if 'phase_err' in r]
        pp = [abs(r['phase_err']) for r in recs if 'phase_err' in r]
        if pp:
            axes[3].plot(pt, pp, color=color, linewidth=0.8, alpha=alpha, label=label)
    axes[3].set_ylabel('|Phase Error| (deg)')
    axes[3].set_xlabel('Time (s)')
    axes[3].legend(fontsize=8)
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'comparison_overlay.png'), dpi=150)
    plt.close(fig)
    print(f"  [+] comparison_overlay.png")


def parse_sim_baseline(filepath):
    """Parse sim_verify.py output to extract predicted thresholds.

    Returns a dict of metric predictions from simulation.
    """
    sim = {}
    if not filepath or not os.path.exists(filepath):
        return sim

    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        text = f.read()

    # Extract governor headroom predictions
    gov_match = re.search(r'governor.*?(\d+\.?\d*)%', text, re.IGNORECASE)
    if gov_match:
        sim['governor_predicted'] = float(gov_match.group(1))

    # Extract phase error predictions
    pe_match = re.search(r'phase.?err.*?(\d+\.?\d*)', text, re.IGNORECASE)
    if pe_match:
        sim['phase_err_predicted'] = float(pe_match.group(1))

    # Extract speed/duty info
    duty_match = re.search(r'duty.*?(\d+\.?\d*)', text, re.IGNORECASE)
    if duty_match:
        sim['duty_predicted'] = float(duty_match.group(1))

    return sim


def format_sim_comparison(run_stats, sim_stats):
    """Compare real telemetry against simulation predictions.

    Returns a list of lines showing where reality diverges from the model.
    """
    lines = []
    if not sim_stats:
        return lines

    lines.append("")
    lines.append("=" * 78)
    lines.append("  SIM vs REALITY: Model Prediction Accuracy")
    lines.append("=" * 78)
    lines.append(f"{'Metric':<30} {'Real':>12} {'Sim Pred':>12} {'Gap':>12} {'Status':>8}")
    lines.append("-" * 78)

    def sim_row(label, real_key, sim_key, fmt=".1f"):
        real = run_stats.get(real_key)
        pred = sim_stats.get(sim_key)
        if real is None or pred is None:
            return
        gap = real - pred
        pct = abs(gap / pred * 100) if pred != 0 else 0
        if pct < 15:
            status = "  OK"
        elif pct < 30:
            status = " DRIFT"
        else:
            status = "DIVERGE"
        lines.append(f"{label:<30} {real:{fmt}:>12} {pred:{fmt}:>12} {gap:+{fmt}:>12} {status:>8}")

    sim_row("Governor max (%)", "governor_max", "governor_predicted")
    sim_row("Phase error max (deg)", "phase_err_max", "phase_err_predicted")

    lines.append("=" * 78)
    lines.append("")
    lines.append("Status: OK = within 15%, DRIFT = 15-30% off, DIVERGE = >30% off")
    lines.append("DIVERGE means the sim model may need recalibration for this terrain/load.")
    return lines


# ── Main ────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Parse and visualize hexapod telemetry data')
    parser.add_argument('heart_log', help='Path to Heart telemetry_log.txt')
    parser.add_argument('--arduino', help='Path to Arduino sensor CSV', default=None)
    parser.add_argument('--output-dir', help='Output directory for PNGs',
                        default='telemetry_output')
    parser.add_argument('--overlay', action='store_true',
                        help='Generate real vs theoretical overlay plot')
    parser.add_argument('--baseline', default=None,
                        help='Path to baseline Heart telemetry log for comparison')
    parser.add_argument('--sim-baseline', default=None,
                        help='Path to sim_verify.py output (text) to compare against')
    args = parser.parse_args()

    # Validate input
    if not os.path.exists(args.heart_log):
        print(f"Error: {args.heart_log} not found")
        sys.exit(1)

    os.makedirs(args.output_dir, exist_ok=True)
    print(f"Parsing telemetry from: {args.heart_log}")

    # Parse Heart data
    records = parse_heart_log(args.heart_log)
    brain_events = parse_brain_events(args.heart_log)
    h5_records = parse_h5_lines(args.heart_log)
    bs_records = parse_bs_lines(args.heart_log)
    bn_records = parse_bn_lines(args.heart_log)
    print(f"  Heart records: {len(records)}")
    print(f"  Brain events: {len(brain_events)}")
    print(f"  [H5] servo detail: {len(h5_records)}")
    print(f"  [BS] sensor snapshots: {len(bs_records)}")
    print(f"  [BN] nav decisions: {len(bn_records)}")

    if not records:
        print("Error: No telemetry records found in file")
        sys.exit(1)

    records = normalize_time(records)

    # Parse Arduino data if provided
    arduino_records = []
    if args.arduino:
        if not os.path.exists(args.arduino):
            print(f"Warning: Arduino CSV {args.arduino} not found, skipping")
        else:
            arduino_records = parse_arduino_csv(args.arduino)
            print(f"  Arduino records: {len(arduino_records)}")

    # Generate plots
    print(f"\nGenerating plots in {args.output_dir}/")
    plot_voltage(records, args.output_dir)
    plot_servo_loads(records, args.output_dir)
    plot_servo_temps(records, args.output_dir)
    plot_governor_phase(records, args.output_dir)
    plot_loop_timing(records, args.output_dir)
    plot_stall_events(records, args.output_dir)

    if arduino_records:
        plot_sensor_distances(arduino_records, args.output_dir)
        plot_imu_orientation(arduino_records, args.output_dir)

    # Verbose telemetry plots (from [H5], [BS], [BN] lines)
    if h5_records:
        plot_servo_positions(h5_records, args.output_dir)
        plot_servo_cmd_vs_actual(h5_records, args.output_dir)
        plot_phase_timeline(h5_records, args.output_dir)
        plot_phase_error(h5_records, args.output_dir)
    if bn_records:
        plot_gait_transitions(bn_records, args.output_dir)
    if bn_records or bs_records:
        plot_nav_timeline(bn_records, bs_records, args.output_dir)

    plot_run_summary(records, brain_events, arduino_records, args.output_dir)

    if args.overlay and h5_records and bn_records:
        plot_real_vs_theoretical(h5_records, bn_records, args.output_dir)

    # Extract and print run stats (always)
    run_stats = extract_run_stats(records)
    print("\n" + "=" * 50)
    print("  RUN SUMMARY STATS")
    print("=" * 50)
    stat_labels = [
        ("Duration", "duration_s", ".0f", "s"),
        ("Voltage", "voltage_min", ".2f", "V min"),
        ("Load max", "load_max", ".0f", "raw"),
        ("Load P95", "load_p95", ".0f", "raw"),
        ("Stalls", "stall_count", ".0f", "events"),
        ("Governor max", "governor_max", ".1f", "%"),
        ("Governor mean", "governor_mean", ".1f", "%"),
        ("Phase err max", "phase_err_max", ".1f", "deg"),
        ("Phase err mean", "phase_err_mean", ".1f", "deg"),
        ("Temp max", "temp_max", ".1f", "C"),
        ("Loop max", "loop_max_ms", ".1f", "ms"),
        ("Loop overruns", "loop_overruns", ".0f", ">22ms"),
        ("TE cycles", "te_cycles_total", ".0f", "total"),
    ]
    for label, key, fmt, unit in stat_labels:
        val = run_stats.get(key)
        if val is not None:
            print(f"  {label:<20} {val:{fmt}} {unit}")
    stalls = run_stats.get('stall_by_servo', {})
    if stalls:
        print(f"  {'Stall breakdown':<20} {', '.join(f'{k}:{v}' for k, v in sorted(stalls.items()))}")
    gaits = run_stats.get('gaits_used', [])
    if gaits:
        print(f"  {'Gaits used':<20} {', '.join(sorted(gaits))}")
    print("=" * 50)

    # Baseline comparison (--baseline flag)
    if args.baseline:
        if not os.path.exists(args.baseline):
            print(f"Warning: Baseline file {args.baseline} not found, skipping comparison")
        else:
            print(f"\nComparing against baseline: {args.baseline}")
            baseline_records = parse_heart_log(args.baseline)
            if baseline_records:
                baseline_records = normalize_time(baseline_records)
                baseline_stats = extract_run_stats(baseline_records)

                # Print delta table
                comparison_lines = format_comparison_table(run_stats, baseline_stats)
                for line in comparison_lines:
                    print(line)

                # Generate overlay plot
                plot_comparison_overlay(records, baseline_records, args.output_dir)

                # Save delta table to file
                delta_path = os.path.join(args.output_dir, 'comparison_delta.txt')
                with open(delta_path, 'w') as f:
                    f.write('\n'.join(comparison_lines))
                print(f"  [+] comparison_delta.txt")
            else:
                print("Warning: No records found in baseline file")

    # Sim baseline comparison (--sim-baseline flag)
    if args.sim_baseline:
        sim_stats = parse_sim_baseline(args.sim_baseline)
        if sim_stats:
            sim_lines = format_sim_comparison(run_stats, sim_stats)
            for line in sim_lines:
                print(line)
            sim_path = os.path.join(args.output_dir, 'sim_vs_reality.txt')
            with open(sim_path, 'w') as f:
                f.write('\n'.join(sim_lines))
            print(f"  [+] sim_vs_reality.txt")
        else:
            print(f"Warning: Could not parse sim baseline from {args.sim_baseline}")

    print(f"\nDone. {len(os.listdir(args.output_dir))} files generated.")


if __name__ == '__main__':
    main()
