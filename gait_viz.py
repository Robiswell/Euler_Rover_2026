#!/usr/bin/env python3
"""Hexapod Gait Visualization Utility

Generates diagnostic plots for the hexapod gait engine.
Each subcommand produces a PNG file and prints the file path to stdout.

Usage:
    python gait_viz.py phase --gait 0 --speed 500 --cycles 3
    python gait_viz.py contact --gait 1 --speed 350 --cycles 2
    python gait_viz.py feedforward --gait 0 --speed 500
    python gait_viz.py gap56 --gait 0
    python gait_viz.py trajectory --gait 0 --speed 500
    python gait_viz.py stability --gait 0
    python gait_viz.py transition --gait-from 0 --gait-to 1
    python gait_viz.py startup --gait 0 --speed 500
    python gait_viz.py self_right
    python gait_viz.py wiggle --duration 5.0
    python gait_viz.py all --all-gaits
"""

import argparse
import math
import os
import sys

try:
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend — no display needed
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
except ImportError:
    print("ERROR: matplotlib and numpy required. Install with: pip install matplotlib numpy")
    sys.exit(1)

# ── Gait Constants (v2 defaults, from final_full_gait_test.py) ──────────

ENCODER_RESOLUTION = 4096.0
VELOCITY_SCALAR = 1.85
LEFT_SERVOS = [2, 3, 4]
RIGHT_SERVOS = [1, 6, 5]
ALL_SERVOS = [1, 2, 3, 4, 5, 6]
DIRECTION_MAP = {1: 1, 2: -1, 3: -1, 4: -1, 5: 1, 6: 1}
HOME_POSITIONS = {1: 3447, 2: 955, 3: 1420, 4: 1569, 5: 3197, 6: 3175}
KP_PHASE = 12.0
LEG_SPLAY = {1: -35, 2: -35, 6: 0, 3: 0, 5: 35, 4: 35}

GAITS = {
    0: {'name': 'Tripod',    'duty': 0.5,
        'offsets': {2: 0.0, 6: 0.0, 4: 0.0, 1: 0.5, 3: 0.5, 5: 0.5}},
    1: {'name': 'Wave',      'duty': 0.60,
        'offsets': {4: 0.833, 3: 0.666, 2: 0.5, 5: 0.333, 6: 0.166, 1: 0.0}},
    2: {'name': 'Quadruped', 'duty': 0.55,
        'offsets': {2: 0.0, 5: 0.0, 3: 0.333, 6: 0.333, 4: 0.666, 1: 0.666}},
}

DEFAULT_IMPACT_START = 320.0
DEFAULT_IMPACT_END = 40.0

# CPG transition
KAPPA_TRANSITION = 8.0
HEART_DT = 0.02  # 50Hz control loop

# Self-right sequence (counter-rotating roll from state_self_right_roll)
ROLL_SPEED = 500          # Raw STS units for counter-rotate attempts
ROLL_DURATION = 1.5       # seconds per attempt (under OVERLOAD_PREVENTION_TIME)
ROLL_SETTLE_TIME = 0.5    # seconds between attempts
ROLL_FRONT_SERVOS = {1, 2}
ROLL_REAR_SERVOS = {4, 5}

# Wiggle recovery (from state_recovery_wiggle)
WIGGLE_SPEED = 350
WIGGLE_IMPACT_START = 330
WIGGLE_IMPACT_END = 30
WIGGLE_FLIP_INTERVAL = 0.3

# Servo display names (position: RF=Right Front, etc.)
SERVO_NAMES = {
    1: 'S1 (RF)', 2: 'S2 (LF)', 3: 'S3 (LM)',
    4: 'S4 (LR)', 5: 'S5 (RR)', 6: 'S6 (RM)'
}

# Distinct colors per servo for consistent identification across plots
SERVO_COLORS = {
    1: '#e74c3c', 2: '#3498db', 3: '#2ecc71',
    4: '#9b59b6', 5: '#f39c12', 6: '#1abc9c'
}

# Physical display order: front to back, right then left
SERVO_DISPLAY_ORDER = [1, 2, 6, 3, 5, 4]


# ── Core Gait Math ───────────────────────────────────────────────────────────

def get_buehler_angle(t_norm, duty_cycle, start_ang, end_ang):
    """Buehler clock: maps normalized phase [0,1] to leg angle [0,360].

    Stance (t_norm <= duty): linear sweep from start_ang to end_ang.
    Air (t_norm > duty): cosine-smoothed return sweep.
    """
    stance_sweep = (end_ang - start_ang + 180) % 360 - 180
    if t_norm <= duty_cycle:
        progress = t_norm / duty_cycle if duty_cycle > 0 else 0
        angle = start_ang + stance_sweep * progress
    else:
        air_dur = 1.0 - duty_cycle
        progress = (t_norm - duty_cycle) / air_dur if air_dur > 0 else 0
        air_sweep = 360.0 - abs(stance_sweep)
        if stance_sweep < 0:
            air_sweep = -air_sweep
        smooth_progress = (1.0 - math.cos(math.pi * progress)) / 2.0
        angle = end_ang + air_sweep * smooth_progress
    return angle % 360


def compute_governor_limit(duty, air_sweep):
    """v2 governor: max safe cycle rate (Hz). No pi/2 factor."""
    return (660.0 / VELOCITY_SCALAR * (1.0 - duty)) / max(5.0, abs(air_sweep))


def compute_feedforward_v2(t_leg, duty, base_sweep, air_sweep, hz):
    """v2 feedforward: constant stance rate, cosine-derivative air profile.

    Returns speed in deg/s (not raw STS — multiply by VELOCITY_SCALAR for raw).
    """
    if duty <= 0 or (1.0 - duty) <= 0:
        return 0.0
    if t_leg <= duty:
        return (base_sweep / duty) * hz
    else:
        ap = (t_leg - duty) / (1.0 - duty)
        return (air_sweep * math.pi / (2.0 * (1.0 - duty))) * math.sin(math.pi * ap) * hz


# ── Plot Functions ───────────────────────────────────────────────────────────

def plot_phase(gait_id, cycles, speed, output_dir, imp_start, imp_end):
    """Phase diagram: t_leg for all 6 legs over multiple cycles."""
    g = GAITS[gait_id]
    duty = g['duty']
    offsets = g['offsets']
    hz = speed / 1000.0
    if hz == 0:
        hz = 0.5  # fallback for display

    cycle_period = 1.0 / abs(hz)
    total_time = cycle_period * cycles
    n_points = max(500, int(total_time / 0.002))
    times = np.linspace(0, total_time, n_points)

    fig, ax = plt.subplots(figsize=(14, 6))

    for sid in ALL_SERVOS:
        master_phases = (hz * times) % 1.0
        leg_phases = (master_phases + offsets[sid]) % 1.0
        ax.scatter(times, leg_phases, c=SERVO_COLORS[sid], s=0.3,
                   label=SERVO_NAMES[sid], alpha=0.7)

    ax.axhline(y=duty, color='gray', linestyle='--', linewidth=1.5,
               label=f'Duty = {duty}')
    ax.fill_between([0, total_time], 0, duty, alpha=0.04, color='green')
    ax.fill_between([0, total_time], duty, 1.0, alpha=0.04, color='blue')
    ax.text(total_time * 0.01, duty / 2, 'STANCE', fontsize=9, alpha=0.4, va='center')
    ax.text(total_time * 0.01, (1 + duty) / 2, 'AIR', fontsize=9, alpha=0.4, va='center')

    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Phase (t_leg)', fontsize=11)
    ax.set_title(f'{g["name"]} Gait — Phase Diagram  |  speed={speed}  hz={hz:.2f}  duty={duty}',
                 fontsize=12, fontweight='bold')
    ax.legend(loc='upper right', fontsize=8, ncol=2, markerscale=8)
    ax.set_ylim(-0.05, 1.05)
    ax.grid(True, alpha=0.3)

    path = os.path.join(output_dir, f'phase_{g["name"].lower()}.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return path


def plot_contact(gait_id, cycles, speed, output_dir, imp_start, imp_end):
    """Hildebrand contact pattern: horizontal bars showing stance vs air per leg."""
    g = GAITS[gait_id]
    duty = g['duty']
    offsets = g['offsets']
    hz = speed / 1000.0
    if hz == 0:
        hz = 0.5

    cycle_period = 1.0 / abs(hz)
    total_time = cycle_period * cycles
    dt = 0.001  # 1 ms resolution
    times = np.arange(0, total_time, dt)
    master_phases = (hz * times) % 1.0

    fig, ax = plt.subplots(figsize=(14, 4))

    bar_height = 0.6
    for row_idx, sid in enumerate(SERVO_DISPLAY_ORDER):
        leg_phases = (master_phases + offsets[sid]) % 1.0
        in_stance = leg_phases < duty

        # Find stance start/end transitions for drawing rectangles
        diff = np.diff(in_stance.astype(int))
        # Stance starts: transition from 0→1 (or starts in stance)
        starts = list(np.where(diff == 1)[0] + 1)
        ends = list(np.where(diff == -1)[0] + 1)

        if in_stance[0]:
            starts.insert(0, 0)
        if in_stance[-1]:
            ends.append(len(times) - 1)

        for s, e in zip(starts, ends):
            t_start = times[s]
            t_end = times[min(e, len(times) - 1)]
            rect = Rectangle((t_start, row_idx - bar_height / 2),
                              t_end - t_start, bar_height,
                              facecolor=SERVO_COLORS[sid], alpha=0.8,
                              edgecolor='black', linewidth=0.5)
            ax.add_patch(rect)

    ax.set_yticks(range(len(SERVO_DISPLAY_ORDER)))
    ax.set_yticklabels([SERVO_NAMES[s] for s in SERVO_DISPLAY_ORDER], fontsize=10)
    ax.set_xlim(0, total_time)
    ax.set_ylim(-0.5, len(SERVO_DISPLAY_ORDER) - 0.5)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_title(f'{g["name"]} Gait — Contact Pattern  |  duty={duty}  speed={speed}',
                 fontsize=12, fontweight='bold')
    ax.grid(True, axis='x', alpha=0.3)
    ax.invert_yaxis()

    # Add legend
    from matplotlib.patches import Patch
    ax.legend([Patch(facecolor='gray', alpha=0.8, edgecolor='black'),
               Patch(facecolor='white', edgecolor='gray', linestyle='--')],
              ['Stance (ground contact)', 'Air (swing)'],
              loc='upper right', fontsize=9)

    path = os.path.join(output_dir, f'contact_{g["name"].lower()}.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return path


def plot_feedforward(gait_id, speed, output_dir, imp_start, imp_end):
    """Feedforward speed profile: raw STS speed vs phase for one cycle."""
    g = GAITS[gait_id]
    duty = g['duty']
    hz = speed / 1000.0
    if hz == 0:
        hz = 0.5

    base_sweep = (imp_end - imp_start + 180) % 360 - 180
    air_sweep = 360.0 - abs(base_sweep)
    if base_sweep < 0:
        air_sweep = -air_sweep

    governor_hz = compute_governor_limit(duty, air_sweep)

    fig, ax = plt.subplots(figsize=(14, 6))

    n_points = 500
    phases = np.linspace(0, 1.0, n_points)

    for sid in ALL_SERVOS:
        ff_raw = []
        for t in phases:
            ff_dps = compute_feedforward_v2(t, duty, base_sweep, air_sweep, hz)
            ff_raw.append(abs(ff_dps) * VELOCITY_SCALAR)
        ax.plot(phases, ff_raw, color=SERVO_COLORS[sid],
                label=SERVO_NAMES[sid], linewidth=1.5)

    # Reference lines
    ax.axhline(y=500, color='red', linestyle='--', linewidth=1.5,
               label='Max servo speed (~500 raw @ no-load)')
    ax.axhline(y=3000, color='darkred', linestyle=':', linewidth=1,
               label='Code clamp (3000 raw)', alpha=0.5)
    ax.axvline(x=duty, color='gray', linestyle='--', linewidth=1.5,
               alpha=0.6, label=f'Duty boundary ({duty})')

    # Annotate governor info
    ax.text(0.02, 0.95, f'Governor max: {governor_hz:.2f} Hz\nActual: {abs(hz):.2f} Hz\n'
            f'Headroom: {((governor_hz - abs(hz)) / governor_hz * 100):.1f}%',
            transform=ax.transAxes, fontsize=9, va='top',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    ax.set_xlabel('Phase (t_leg)', fontsize=11)
    ax.set_ylabel('Feedforward Speed (raw STS units)', fontsize=11)
    ax.set_title(f'{g["name"]} Gait — Feedforward Profile  |  speed={speed}  hz={hz:.2f}',
                 fontsize=12, fontweight='bold')
    ax.legend(loc='upper right', fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 1)

    path = os.path.join(output_dir, f'feedforward_{g["name"].lower()}.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return path


def plot_gap56(gait_id, speed, output_dir, imp_start, imp_end):
    """Angular gap between servos 5 and 6 through one cycle."""
    g = GAITS[gait_id]
    duty = g['duty']
    offsets = g['offsets']

    fig, ax = plt.subplots(figsize=(14, 5))

    n_points = 1000
    phases = np.linspace(0, 1.0, n_points)
    gaps = []
    min_gap = 360.0
    min_gap_phase = 0.0
    s5_angles = []
    s6_angles = []

    for master_t in phases:
        t5 = (master_t + offsets[5]) % 1.0
        t6 = (master_t + offsets[6]) % 1.0

        angle5 = get_buehler_angle(t5, duty, imp_start, imp_end)
        angle6 = get_buehler_angle(t6, duty, imp_start, imp_end)

        # Add splay
        angle5 = (angle5 + LEG_SPLAY[5]) % 360
        angle6 = (angle6 + LEG_SPLAY[6]) % 360

        gap = abs((angle5 - angle6 + 180) % 360 - 180)
        gaps.append(gap)
        s5_angles.append(angle5)
        s6_angles.append(angle6)

        if gap < min_gap:
            min_gap = gap
            min_gap_phase = master_t

    ax.plot(phases, gaps, color='#e74c3c', linewidth=2.5, label='Angular gap')
    ax.axhline(y=min_gap, color='orange', linestyle='--', linewidth=1,
               label=f'Min gap: {min_gap:.1f}° @ phase {min_gap_phase:.3f}')
    ax.axhline(y=0, color='red', linestyle='-', linewidth=2,
               alpha=0.3, label='Collision (0°)')

    # Mark minimum point
    ax.plot(min_gap_phase, min_gap, 'ro', markersize=10, zorder=5)
    ax.annotate(f'{min_gap:.1f}°', (min_gap_phase, min_gap),
                textcoords="offset points", xytext=(12, 12), fontsize=11,
                fontweight='bold',
                arrowprops=dict(arrowstyle='->', color='red', lw=1.5))

    # Secondary axis: individual servo angles
    ax2 = ax.twinx()
    ax2.plot(phases, s5_angles, color=SERVO_COLORS[5], linewidth=1, alpha=0.3,
             linestyle=':', label='S5 angle')
    ax2.plot(phases, s6_angles, color=SERVO_COLORS[6], linewidth=1, alpha=0.3,
             linestyle=':', label='S6 angle')
    ax2.set_ylabel('Individual Servo Angle (°)', fontsize=10, alpha=0.5)
    ax2.legend(loc='lower right', fontsize=8, framealpha=0.5)

    ax.set_xlabel('Master Clock Phase', fontsize=11)
    ax.set_ylabel('Angular Gap (degrees)', fontsize=11)
    ax.set_title(f'{g["name"]} Gait — Servo 5/6 Gap  |  duty={duty}  '
                 f'splay: S5=+{LEG_SPLAY[5]}° S6={LEG_SPLAY[6]}°',
                 fontsize=12, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9)
    ax.set_ylim(-5, max(gaps) * 1.15)
    ax.set_xlim(0, 1)
    ax.grid(True, alpha=0.3)

    path = os.path.join(output_dir, f'gap56_{g["name"].lower()}.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return path


def plot_trajectory(gait_id, speed, output_dir, imp_start, imp_end):
    """Buehler angle (degrees) vs phase for all 6 legs through one cycle.

    Shows the actual angular position of each leg, with stance and air regions
    clearly distinguished.
    """
    g = GAITS[gait_id]
    duty = g['duty']
    offsets = g['offsets']

    fig, ax = plt.subplots(figsize=(14, 6))

    n_points = 500
    master_phases = np.linspace(0, 1.0, n_points)

    for sid in ALL_SERVOS:
        angles = []
        in_stance = []
        for mt in master_phases:
            t_leg = (mt + offsets[sid]) % 1.0
            angle = get_buehler_angle(t_leg, duty, imp_start, imp_end)
            angle = (angle + LEG_SPLAY[sid]) % 360
            angles.append(angle)
            in_stance.append(t_leg <= duty)

        angles = np.array(angles)
        in_stance = np.array(in_stance)

        # Plot stance portions solid, air portions dashed
        # Use masking to separate
        stance_angles = np.where(in_stance, angles, np.nan)
        air_angles = np.where(~in_stance, angles, np.nan)

        ax.plot(master_phases, stance_angles, color=SERVO_COLORS[sid],
                linewidth=2.0, alpha=0.9, label=f'{SERVO_NAMES[sid]} stance')
        ax.plot(master_phases, air_angles, color=SERVO_COLORS[sid],
                linewidth=1.0, alpha=0.4, linestyle='--')

    # Impact angle reference lines
    ax.axhline(y=imp_start, color='green', linestyle=':', linewidth=1,
               alpha=0.5, label=f'Impact start ({imp_start}°)')
    ax.axhline(y=imp_end, color='blue', linestyle=':', linewidth=1,
               alpha=0.5, label=f'Impact end ({imp_end}°)')

    ax.set_xlabel('Master Clock Phase', fontsize=11)
    ax.set_ylabel('Leg Angle (degrees)', fontsize=11)
    ax.set_title(f'{g["name"]} Gait — Leg Trajectories  |  '
                 f'impact: {imp_start}°→{imp_end}°  duty={duty}',
                 fontsize=12, fontweight='bold')
    ax.legend(loc='upper right', fontsize=7, ncol=3)
    ax.set_xlim(0, 1)
    ax.grid(True, alpha=0.3)

    path = os.path.join(output_dir, f'trajectory_{g["name"].lower()}.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return path


def plot_stability(gait_id, speed, output_dir, imp_start, imp_end):
    """Number of legs in stance at each point in one cycle.

    Critical for verifying static stability — the robot needs at least 3 legs
    in stance at all times to avoid tipover.
    """
    g = GAITS[gait_id]
    duty = g['duty']
    offsets = g['offsets']

    fig, ax = plt.subplots(figsize=(14, 4))

    n_points = 1000
    master_phases = np.linspace(0, 1.0, n_points)
    legs_in_stance = np.zeros(n_points)

    per_leg_stance = {}
    for sid in ALL_SERVOS:
        stance = []
        for mt in master_phases:
            t_leg = (mt + offsets[sid]) % 1.0
            stance.append(1 if t_leg <= duty else 0)
        per_leg_stance[sid] = np.array(stance)
        legs_in_stance += per_leg_stance[sid]

    # Main line
    ax.fill_between(master_phases, legs_in_stance, alpha=0.3, color='#2ecc71')
    ax.plot(master_phases, legs_in_stance, color='#27ae60', linewidth=2.5,
            label='Legs in stance')

    # Stability threshold
    ax.axhline(y=3, color='red', linestyle='--', linewidth=1.5,
               label='Min stable (3 legs)', alpha=0.7)
    ax.axhline(y=6, color='gray', linestyle=':', linewidth=1,
               label='All legs down', alpha=0.3)

    # Annotate min and max
    min_legs = int(np.min(legs_in_stance))
    max_legs = int(np.max(legs_in_stance))
    ax.text(0.02, 0.95, f'Min: {min_legs} legs  |  Max: {max_legs} legs  |  '
            f'Avg: {np.mean(legs_in_stance):.1f} legs',
            transform=ax.transAxes, fontsize=10, va='top',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    # Color dangerous zones
    below_3 = legs_in_stance < 3
    if np.any(below_3):
        ax.fill_between(master_phases, 0, legs_in_stance,
                        where=below_3, alpha=0.3, color='red',
                        label='UNSTABLE (<3 legs)')

    ax.set_xlabel('Master Clock Phase', fontsize=11)
    ax.set_ylabel('Number of Legs in Stance', fontsize=11)
    ax.set_title(f'{g["name"]} Gait — Stability  |  duty={duty}',
                 fontsize=12, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9)
    ax.set_ylim(-0.2, 6.5)
    ax.set_xlim(0, 1)
    ax.set_yticks(range(7))
    ax.grid(True, alpha=0.3)

    path = os.path.join(output_dir, f'stability_{g["name"].lower()}.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return path


def cpg_exp_ramp(current, target, kappa, dt):
    """Exponential ramp toward target. Returns new value."""
    return current + (target - current) * (1.0 - math.exp(-kappa * dt))


def cpg_exp_ramp_circular(current, target, kappa, dt, period=1.0):
    """Exponential ramp with circular wrapping. Returns new value."""
    half = period / 2.0
    diff = (target - current + half) % period - half
    return (current + diff * (1.0 - math.exp(-kappa * dt))) % period


def plot_wiggle(output_dir, duration=3.0):
    g = GAITS[0]
    duty = g['duty']
    offsets = g['offsets']
    n_steps = int(duration / HEART_DT)
    times = [i * HEART_DT for i in range(n_steps)]

    x_flips = []
    x_flip = 1
    for t in times:
        flip_index = int(t / WIGGLE_FLIP_INTERVAL)
        x_flip = 1 if flip_index % 2 == 0 else -1
        x_flips.append(x_flip)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    ax1.step(times, x_flips, where='post', color='#2c3e50', linewidth=2)
    ax1.set_ylabel('x_flip')
    ax1.set_ylim(-1.5, 1.5)
    ax1.set_yticks([-1, 1])
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Wiggle Recovery Sequence', fontsize=12, fontweight='bold')

    servo_angles = {sid: [] for sid in ALL_SERVOS}
    phases = {sid: offsets[sid] for sid in ALL_SERVOS}
    speed = WIGGLE_SPEED

    for i, t in enumerate(times):
        xf = x_flips[i]
        effective_speed = speed * xf
        hz = effective_speed / 1000.0
        for sid in ALL_SERVOS:
            phases[sid] = (phases[sid] + hz * HEART_DT) % 1.0
            t_leg = phases[sid]
            angle = get_buehler_angle(t_leg, duty, WIGGLE_IMPACT_START, WIGGLE_IMPACT_END)
            servo_angles[sid].append(angle)

    for sid in ALL_SERVOS:
        ax2.plot(times, servo_angles[sid], color=SERVO_COLORS[sid],
                 label=SERVO_NAMES[sid], linewidth=1.5)

    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_ylabel('Buehler Angle (deg)', fontsize=11)
    ax2.legend(loc='upper right', fontsize=8, ncol=2)
    ax2.grid(True, alpha=0.3)

    path = os.path.join(output_dir, 'wiggle_sequence.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return path


def plot_self_right(output_dir):
    """Visualize counter-rotating self-right sequence.

    Panel 1: Speed command profile (base speed vs time)
    Panel 2: Per-servo effective speed (front/rear/middle differentiation)
    """
    total_time = 2 * ROLL_DURATION + 2 * ROLL_SETTLE_TIME  # ~4s
    n_steps = int(total_time / HEART_DT)
    times = [i * HEART_DT for i in range(n_steps)]

    # Build base speed timeline
    t1 = ROLL_DURATION                          # end of counter_A
    t2 = t1 + ROLL_SETTLE_TIME                  # end of settle_1
    t3 = t2 + ROLL_DURATION                     # end of counter_B
    t4 = t3 + ROLL_SETTLE_TIME                  # end of settle_2

    base_speeds = []
    for t in times:
        if t < t1:
            base_speeds.append(ROLL_SPEED)
        elif t < t2:
            base_speeds.append(0)
        elif t < t3:
            base_speeds.append(-ROLL_SPEED)
        else:
            base_speeds.append(0)

    # Per-servo speeds (counter-rotating: front vs rear vs middle)
    servo_speeds = {sid: [] for sid in ALL_SERVOS}
    for base in base_speeds:
        for sid in ALL_SERVOS:
            if sid in ROLL_FRONT_SERVOS:
                servo_speeds[sid].append(base)
            elif sid in ROLL_REAR_SERVOS:
                servo_speeds[sid].append(-base)
            else:  # middle — spin same as front to prevent kickstand
                servo_speeds[sid].append(base)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 9), sharex=True)

    # Panel 1: Base speed command
    ax1.plot(times, base_speeds, color='#2c3e50', linewidth=2)
    ax1.set_ylabel('Base Speed Command (STS)', fontsize=11)
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Counter-Rotating Self-Right Sequence', fontsize=12, fontweight='bold')
    ax1.axhline(y=0, color='gray', linewidth=0.5)

    stages = [
        (0, t1, 'Counter-rotate A'),
        (t1, t2, 'Settle'),
        (t2, t3, 'Counter-rotate B'),
        (t3, t4, 'Settle'),
    ]
    for t_start, t_end, label in stages:
        ax1.axvline(x=t_start, color='gray', linestyle='--', linewidth=1, alpha=0.6)
        y_pos = ROLL_SPEED * 0.85 if 'A' in label else (-ROLL_SPEED * 0.85 if 'B' in label else 0)
        ax1.text((t_start + t_end) / 2, y_pos, label,
                 ha='center', fontsize=9, alpha=0.7)

    # Panel 2: Per-servo effective speed
    for sid in ALL_SERVOS:
        ax2.plot(times, servo_speeds[sid], color=SERVO_COLORS[sid],
                 label=SERVO_NAMES[sid], linewidth=1.5)

    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_ylabel('Effective Servo Speed (STS)', fontsize=11)
    ax2.legend(loc='upper right', fontsize=8, ncol=2)
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='gray', linewidth=0.5)

    # Annotate servo groups
    ax2.text(t1 / 2, ROLL_SPEED * 0.7, 'Front (1,2) + Middle (3,6)',
             ha='center', fontsize=8, color='#2980b9', alpha=0.8)
    ax2.text(t1 / 2, -ROLL_SPEED * 0.7, 'Rear (4,5) opposite',
             ha='center', fontsize=8, color='#e74c3c', alpha=0.8)

    for t_start, t_end, label in stages:
        ax2.axvline(x=t_start, color='gray', linestyle='--', linewidth=1, alpha=0.6)

    path = os.path.join(output_dir, 'self_right_sequence.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return path


def plot_transition(gait_from, gait_to, speed, output_dir, imp_start, imp_end):
    g_from = GAITS[gait_from]
    g_to = GAITS[gait_to]
    hz = speed / 1000.0
    duration = 2.0
    n_steps = int(duration / HEART_DT)
    times = [i * HEART_DT for i in range(n_steps)]

    duty_trace = []
    phase_traces = {sid: [] for sid in ALL_SERVOS}
    angle_traces = {sid: [] for sid in ALL_SERVOS}

    cur_duty = g_from['duty']
    cur_offsets = {sid: g_from['offsets'][sid] for sid in ALL_SERVOS}
    cur_imp_start = imp_start
    cur_imp_end = imp_end
    master_phase = 0.0

    for i, t in enumerate(times):
        cur_duty = cpg_exp_ramp(cur_duty, g_to['duty'], KAPPA_TRANSITION, HEART_DT)
        for sid in ALL_SERVOS:
            cur_offsets[sid] = cpg_exp_ramp_circular(
                cur_offsets[sid], g_to['offsets'][sid], KAPPA_TRANSITION, HEART_DT)

        duty_trace.append(cur_duty)
        master_phase = (master_phase + hz * HEART_DT) % 1.0

        for sid in ALL_SERVOS:
            phase_traces[sid].append(cur_offsets[sid])
            t_leg = (master_phase + cur_offsets[sid]) % 1.0
            angle = get_buehler_angle(t_leg, cur_duty, imp_start, imp_end)
            angle_traces[sid].append(angle)

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 11), sharex=True)

    ax1.plot(times, duty_trace, color='#2c3e50', linewidth=2)
    ax1.axvline(x=0, color='red', linestyle='--', linewidth=1, alpha=0.6)
    ax1.set_ylabel('Duty Cycle', fontsize=11)
    ax1.grid(True, alpha=0.3)
    ax1.set_title(f"Gait Transition: {g_from['name']} -> {g_to['name']}",
                  fontsize=12, fontweight='bold')

    for sid in ALL_SERVOS:
        ax2.plot(times, phase_traces[sid], color=SERVO_COLORS[sid],
                 label=SERVO_NAMES[sid], linewidth=1.5)
    ax2.axvline(x=0, color='red', linestyle='--', linewidth=1, alpha=0.6)
    ax2.set_ylabel('Phase Offset', fontsize=11)
    ax2.legend(loc='upper right', fontsize=8, ncol=2)
    ax2.grid(True, alpha=0.3)

    for sid in ALL_SERVOS:
        ax3.plot(times, angle_traces[sid], color=SERVO_COLORS[sid],
                 label=SERVO_NAMES[sid], linewidth=1.5)
    ax3.axvline(x=0, color='red', linestyle='--', linewidth=1, alpha=0.6)
    ax3.set_xlabel('Time (s)', fontsize=11)
    ax3.set_ylabel('Buehler Angle (deg)', fontsize=11)
    ax3.legend(loc='upper right', fontsize=8, ncol=2)
    ax3.grid(True, alpha=0.3)

    path = os.path.join(output_dir, f'transition_{gait_from}_{gait_to}.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return path


def plot_startup(gait_id, speed, output_dir, imp_start, imp_end):
    g = GAITS[gait_id]
    duty = g['duty']
    offsets = g['offsets']
    hz = speed / 1000.0
    duration = 2.0
    n_steps = int(duration / HEART_DT)
    times = [i * HEART_DT for i in range(n_steps)]

    home_angles = {sid: (HOME_POSITIONS[sid] / ENCODER_RESOLUTION) * 360.0 for sid in ALL_SERVOS}

    angle_traces = {sid: [] for sid in ALL_SERVOS}
    master_phase = 0.0
    ramp_factor = 0.0

    for i, t in enumerate(times):
        ramp_factor = cpg_exp_ramp(ramp_factor, 1.0, KAPPA_TRANSITION, HEART_DT)
        master_phase = (master_phase + hz * HEART_DT) % 1.0

        for sid in ALL_SERVOS:
            t_leg = (master_phase + offsets[sid]) % 1.0
            walk_angle = get_buehler_angle(t_leg, duty, imp_start, imp_end)
            blended = home_angles[sid] + (walk_angle - home_angles[sid]) * ramp_factor
            angle_traces[sid].append(blended % 360)

    fig, ax = plt.subplots(figsize=(14, 6))

    for sid in ALL_SERVOS:
        ax.plot(times, angle_traces[sid], color=SERVO_COLORS[sid],
                label=SERVO_NAMES[sid], linewidth=1.5)

    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Servo Angle (deg)', fontsize=11)
    ax.set_title(f"Startup Sequence: {g['name']}", fontsize=12, fontweight='bold')
    ax.legend(loc='upper right', fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)
    ax.text(0.02, 0.02, 'Theoretical approximation -- actual startup uses position-mode servos',
            transform=ax.transAxes, fontsize=9, style='italic', alpha=0.6,
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    path = os.path.join(output_dir, f'startup_{gait_id}.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return path


# ── CLI ──────────────────────────────────────────────────────────────────────

PLOT_FUNCS = {
    'phase':       lambda a, gid: plot_phase(gid, a.cycles, a.speed, a.output_dir, a.impact_start, a.impact_end),
    'contact':     lambda a, gid: plot_contact(gid, a.cycles, a.speed, a.output_dir, a.impact_start, a.impact_end),
    'feedforward': lambda a, gid: plot_feedforward(gid, a.speed, a.output_dir, a.impact_start, a.impact_end),
    'gap56':       lambda a, gid: plot_gap56(gid, a.speed, a.output_dir, a.impact_start, a.impact_end),
    'trajectory':  lambda a, gid: plot_trajectory(gid, a.speed, a.output_dir, a.impact_start, a.impact_end),
    'stability':   lambda a, gid: plot_stability(gid, a.speed, a.output_dir, a.impact_start, a.impact_end),
    'transition':  lambda a, gid: plot_transition(a.gait_from, a.gait_to, a.speed, a.output_dir, a.impact_start, a.impact_end),
    'startup':     lambda a, gid: plot_startup(gid, a.speed, a.output_dir, a.impact_start, a.impact_end),
    'self_right':  lambda a, gid: plot_self_right(a.output_dir),
    'wiggle':      lambda a, gid: plot_wiggle(a.output_dir, a.duration),
}


def main():
    parser = argparse.ArgumentParser(
        description='Hexapod Gait Visualization Utility',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Plot types:
  phase        Phase fraction (t_leg) for all 6 legs over time
  contact      Hildebrand diagram: stance (filled) vs air bars per leg
  feedforward  Feedforward speed (raw STS units) vs phase
  gap56        Angular gap between servos 5 and 6 over one cycle
  trajectory   Buehler angle (degrees) vs phase for all legs
  stability    Number of legs in stance at each phase point
  transition   CPG exponential ramp between two gaits
  startup      HOME to steady-state walking ramp
  self_right   Self-right recovery sequence (speed + angle timeline)
  wiggle       Wiggle recovery sequence (flip + angle timeline)
  all          Generate all plots (excludes transition)

Examples:
  python gait_viz.py phase --gait 0 --speed 500 --cycles 3
  python gait_viz.py all --all-gaits
  python gait_viz.py feedforward --gait 1 --speed 350
  python gait_viz.py gap56 --gait 2
  python gait_viz.py transition --gait-from 0 --gait-to 1
  python gait_viz.py startup --gait 0 --speed 500
  python gait_viz.py wiggle --duration 5.0
""")

    parser.add_argument('plot',
                        choices=['phase', 'contact', 'feedforward', 'gap56',
                                 'trajectory', 'stability', 'transition',
                                 'startup', 'self_right', 'wiggle', 'all'],
                        help='Type of plot to generate')
    parser.add_argument('--gait', type=int, default=0, choices=[0, 1, 2],
                        help='Gait: 0=Tripod, 1=Wave, 2=Quadruped (default: 0)')
    parser.add_argument('--speed', type=int, default=500,
                        help='Speed value (default: 500)')
    parser.add_argument('--cycles', type=int, default=3,
                        help='Number of cycles for phase/contact plots (default: 3)')
    parser.add_argument('--impact-start', type=float, default=DEFAULT_IMPACT_START,
                        help=f'Impact start angle in degrees (default: {DEFAULT_IMPACT_START})')
    parser.add_argument('--impact-end', type=float, default=DEFAULT_IMPACT_END,
                        help=f'Impact end angle in degrees (default: {DEFAULT_IMPACT_END})')
    parser.add_argument('--output-dir', type=str, default=None,
                        help='Output directory (default: ./gait_viz_output)')
    parser.add_argument('--all-gaits', action='store_true',
                        help='Generate the plot(s) for all 3 gaits')
    parser.add_argument('--gait-from', type=int, default=0,
                        help='Source gait for transition plot (default: 0)')
    parser.add_argument('--gait-to', type=int, default=1,
                        help='Target gait for transition plot (default: 1)')
    parser.add_argument('--duration', type=float, default=3.0,
                        help='Duration in seconds for wiggle plot (default: 3.0)')

    args = parser.parse_args()

    if args.output_dir is None:
        args.output_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'gait_viz_output')

    os.makedirs(args.output_dir, exist_ok=True)

    gait_ids = [0, 1, 2] if args.all_gaits else [args.gait]
    ALL_PLOTS = [k for k in PLOT_FUNCS.keys() if k != 'transition']
    plots_to_run = ALL_PLOTS if args.plot == 'all' else [args.plot]

    ONCE_PLOTS = {'self_right', 'wiggle'}
    generated = []
    for i, gid in enumerate(gait_ids):
        gait_name = GAITS[gid]['name']
        for plot_name in plots_to_run:
            if plot_name in ONCE_PLOTS and i > 0:
                continue
            try:
                path = PLOT_FUNCS[plot_name](args, gid)
                generated.append(path)
                print(path)
            except Exception as e:
                print(f"ERROR generating {plot_name} for {gait_name}: {e}",
                      file=sys.stderr)

    print(f"\nGenerated {len(generated)} plot(s) in {args.output_dir}")


if __name__ == '__main__':
    main()
