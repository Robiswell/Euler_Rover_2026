#!/usr/bin/env python3
"""
rover_statics.py -- Hexapod rover statics analysis tool.

Analyzes self-right roll torques, kickstand effects, and slope stability.
Uses hexagonal cross-section geometry and Buehler-leg servo layout.

Usage:
    python rover_statics.py self_right
    python rover_statics.py kickstand
    python rover_statics.py slope ANGLE
"""

import argparse
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ──────────────────────────────────────────────────────────────
# Constants (from hardware.md -- authoritative)
# ──────────────────────────────────────────────────────────────

MASS_KG = 3.0
G = 9.81  # m/s^2
WEIGHT_N = MASS_KG * G  # 29.43 N

# Chassis hexagonal cross-section vertices (Y, Z) in mm from body center.
# Hexagon: flat top/bottom (82 mm), pointed sides (220 mm width at mid-height).
# Vertices CCW viewed from front, starting top-left.
CHASSIS_VERTICES_YZ = np.array([
    [-41.0, +40.0],   # 0: top-left
    [+41.0, +40.0],   # 1: top-right
    [+110.0,  0.0],   # 2: right (widest point at mid-height)
    [+41.0, -40.0],   # 3: bottom-right
    [-41.0, -40.0],   # 4: bottom-left
    [-110.0,  0.0],   # 5: left (widest point at mid-height)
])

# Servo positions in body frame (origin=center, +X=forward, +Y=right, +Z=up)
SERVO_POSITIONS = {
    1: (216.7, 80.0, 0.0),    # right-front
    2: (216.7, -80.0, 0.0),   # left-front
    3: (41.7, -80.0, 0.0),    # left-middle
    4: (-133.3, -80.0, 0.0),  # left-rear
    5: (-133.3, 80.0, 0.0),   # right-rear
    6: (41.7, 80.0, 0.0),     # right-middle
}

# Splay angles: positive = toes-out on right side convention
SERVO_SPLAY_DEG = {
    1: 35.0, 2: 35.0, 3: 0.0, 4: -35.0, 5: -35.0, 6: 0.0
}

LEG_RADIUS_MM = 125.0       # shaft center to outer tip
LEG_ARC_DEG = 195.0          # total ground-contact arc
LEG_GAP_DEG = 165.0          # C-gap
STALL_TORQUE_NM = 2.94       # STS3215 stall torque
SHAFT_HEIGHT_INVERTED_MM = 36.0  # shaft center above ground when inverted (lid thickness)

# ──────────────────────────────────────────────────────────────
# Strategy definitions
# Each strategy maps servo_id -> direction multiplier.
# +1 = spin direction that produces +Y roll torque via splay.
# -1 = opposite.  0 = stopped.
# ──────────────────────────────────────────────────────────────

STRATEGIES = {
    "counter_rotate_middle_stop": {
        "label": "counter_rotate (middle stop)",
        "dirs": {1: +1, 2: +1, 3: 0, 4: -1, 5: -1, 6: 0},
        "kickstand_risk": True,
    },
    "counter_rotate_middle_spin": {
        "label": "counter_rotate (middle spin)",
        "dirs": {1: +1, 2: +1, 3: +1, 4: -1, 5: -1, 6: +1},
        "kickstand_risk": False,
    },
    "all_same": {
        "label": "all_same_direction",
        "dirs": {1: +1, 2: +1, 3: +1, 4: +1, 5: +1, 6: +1},
        "kickstand_risk": False,
    },
    "front_only": {
        "label": "front_only (1,2)",
        "dirs": {1: +1, 2: +1, 3: 0, 4: 0, 5: 0, 6: 0},
        "kickstand_risk": True,
    },
    "rear_only": {
        "label": "rear_only (4,5)",
        "dirs": {1: 0, 2: 0, 3: 0, 4: -1, 5: -1, 6: 0},
        "kickstand_risk": True,
    },
    "one_side": {
        "label": "one_side (right: 1,5,6)",
        "dirs": {1: +1, 2: 0, 3: 0, 4: 0, 5: +1, 6: +1},
        "kickstand_risk": False,
    },
}

# ──────────────────────────────────────────────────────────────
# Core geometry functions
# ──────────────────────────────────────────────────────────────

def _rot2d(angle_rad):
    """2D rotation matrix (CCW positive)."""
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, -s], [s, c]])


def compute_vertex_exterior_angles(vertices):
    """Compute the exterior angle at each vertex of a closed polygon.

    The exterior angle at vertex i is pi minus the interior angle.
    For rolling, this is the angle the body must rotate to transition
    from the face before vertex i to the face after vertex i.

    Returns array of exterior angles in radians, one per vertex.
    """
    n = len(vertices)
    exterior = np.zeros(n)
    for i in range(n):
        prev_v = vertices[(i - 1) % n]
        curr_v = vertices[i]
        next_v = vertices[(i + 1) % n]
        # Vectors along edges meeting at vertex i
        edge_in = curr_v - prev_v
        edge_out = next_v - curr_v
        # Interior angle via dot product
        cos_int = np.dot(edge_in, edge_out) / (
            np.linalg.norm(edge_in) * np.linalg.norm(edge_out)
        )
        cos_int = np.clip(cos_int, -1.0, 1.0)
        interior = np.arccos(cos_int)
        exterior[i] = np.pi - interior
    return exterior


def cross_section_pivot(roll_deg, vertices):
    """For a given roll angle (inverted start), return pivot info.

    Convention: robot starts INVERTED with top face (vertices 0-1) on ground.
    Rolling RIGHT (positive phi) goes toward vertex 1, then 2, etc.

    Returns:
        pivot_yz: (y, z) of the current pivot vertex in the WORLD frame
        face_idx: index of the face currently in contact
        accumulated_angle: total angle rolled through completed transitions
    """
    roll_rad = np.radians(roll_deg)
    n = len(vertices)
    ext_angles = compute_vertex_exterior_angles(vertices)

    # When inverted, top face (edge 0->1) is on ground.
    # The body center (CoG) is above this face.
    # Rolling right: first pivot is vertex 1 (right end of top face).
    # After rotating ext_angles[1] around vertex 1, we transition to
    # face 1->2, and the new pivot becomes vertex 2, etc.

    # Initial pivot: vertex 1 in world frame.
    # At phi=0, vertex 1 is on the ground at position determined by
    # the top face geometry.

    # Place the body so that the top face midpoint is at the origin on the
    # ground (z=0), with the body center above.
    face_start = vertices[0]
    face_end = vertices[1]
    face_mid = (face_start + face_end) / 2.0

    # Body center relative to face midpoint (in body frame, inverted)
    # In inverted orientation, body-frame Z is flipped: body center is
    # at (0,0) and face midpoint is at (0, +40) in body frame.
    # In world frame (inverted), the face midpoint is on the ground,
    # body center is ABOVE it.
    # cog_world_initial = (0, face_height_above_ground)
    # But we need the pivot (vertex 1) position in world frame at phi=0.

    # At phi=0 (inverted, top face flat on ground):
    # The ground plane is at z=0. The top face lies along z=0.
    # Vertex 0 is at world position (-41 - (-41), 0) relative to face...
    # Simpler: place body center at world position.
    # Top face is at body z=+40. Inverted means body z=+40 maps to world z=0.
    # So body center is at world z = 0 - 40 = ... wait, let's be careful.

    # In body frame: vertex 0 = (-41, +40), vertex 1 = (+41, +40).
    # When inverted (flipped upside down), body y stays same but body z flips.
    # Inverted body frame: vertex 0 = (-41, -40), vertex 1 = (+41, -40).
    # Place so that top face (now at body_z = -40) touches ground (world z=0):
    # world_z = body_z_inverted + offset, where offset = +40.
    # So body center at world (0, 40). Vertex 1 at world (+41, 0). Correct.

    # For inversion: reflect Z (multiply z by -1), then translate up
    # so the lowest point touches ground.
    inv_vertices = vertices.copy()
    inv_vertices[:, 1] *= -1  # flip Z for inversion
    # Translate so min Z = 0 (ground)
    min_z = inv_vertices[:, 1].min()
    inv_vertices[:, 1] -= min_z

    # Body center in world frame at phi=0
    cog_world = np.array([0.0, -min_z])  # (0, 40) for our hexagon

    # Current pivot vertex index (start with vertex 1 for rightward roll)
    pivot_idx = 1
    pivot_pos = inv_vertices[pivot_idx].copy()
    accumulated = 0.0

    if roll_rad <= 0:
        return pivot_pos, 0, 0.0

    # Walk through vertex transitions
    while True:
        ext = ext_angles[pivot_idx]
        if accumulated + ext > roll_rad:
            # Still on current face (or transitioning)
            break
        accumulated += ext
        # Rotate everything (body + CoG) around current pivot by ext
        rot = _rot2d(-ext)  # negative = clockwise roll to the right
        cog_world = pivot_pos + rot @ (cog_world - pivot_pos)
        # Update all vertex positions
        for i in range(n):
            inv_vertices[i] = pivot_pos + rot @ (inv_vertices[i] - pivot_pos)
        # Advance to next pivot vertex
        pivot_idx = (pivot_idx + 1) % n
        pivot_pos = inv_vertices[pivot_idx].copy()

    # Apply remaining rotation within current face
    remaining = roll_rad - accumulated
    if remaining > 0:
        rot = _rot2d(-remaining)
        cog_world = pivot_pos + rot @ (cog_world - pivot_pos)

    return pivot_pos, pivot_idx, accumulated


def gravity_torque(roll_deg, vertices, mass_kg=MASS_KG):
    """Gravity torque about the current pivot at given roll angle.

    Positive = pro-roll (helps the roll continue rightward).
    Negative = restoring (opposes the roll).

    The torque is m*g times the horizontal distance from CoG to
    the vertical line through the pivot.
    """
    roll_rad = np.radians(roll_deg)
    n = len(vertices)
    ext_angles = compute_vertex_exterior_angles(vertices)

    # Set up inverted geometry (same as cross_section_pivot)
    inv_vertices = vertices.copy()
    inv_vertices[:, 1] *= -1
    min_z = inv_vertices[:, 1].min()
    inv_vertices[:, 1] -= min_z

    cog_world = np.array([0.0, -min_z])

    pivot_idx = 1
    pivot_pos = inv_vertices[pivot_idx].copy()
    accumulated = 0.0

    if roll_rad > 0:
        while True:
            ext = ext_angles[pivot_idx]
            if accumulated + ext > roll_rad:
                break
            accumulated += ext
            rot = _rot2d(-ext)
            cog_world = pivot_pos + rot @ (cog_world - pivot_pos)
            for i in range(n):
                inv_vertices[i] = pivot_pos + rot @ (inv_vertices[i] - pivot_pos)
            pivot_idx = (pivot_idx + 1) % n
            pivot_pos = inv_vertices[pivot_idx].copy()

        remaining = roll_rad - accumulated
        if remaining > 0:
            rot = _rot2d(-remaining)
            cog_world = pivot_pos + rot @ (cog_world - pivot_pos)

    # Torque = m * g * horizontal_offset (CoG_y - pivot_y)
    # Positive horizontal_offset = CoG is to the right of pivot = pro-roll
    horizontal_mm = cog_world[0] - pivot_pos[0]
    torque_nm = mass_kg * G * (horizontal_mm / 1000.0)
    return torque_nm


def servo_roll_torque(strategy_name):
    """Net servo reaction torque about the roll axis for a given strategy.

    Each servo contributes: direction * stall_torque * sin(splay_angle).
    Middle servos (splay=0) always contribute 0.
    """
    strat = STRATEGIES[strategy_name]
    dirs = strat["dirs"]
    total = 0.0
    for sid, direction in dirs.items():
        splay_rad = np.radians(SERVO_SPLAY_DEG[sid])
        total += direction * STALL_TORQUE_NM * np.sin(splay_rad)
    return total


def kickstand_gravity_torque(roll_deg, kickstand_y_mm=80.0,
                             vertices=CHASSIS_VERTICES_YZ, mass_kg=MASS_KG):
    """Gravity torque when a kickstand leg creates a new pivot point.

    The kickstand leg (middle servo, stopped) touches ground at y=kickstand_y_mm.
    This shifts the effective pivot outward, increasing required torque.

    For simplicity, at phi=0 the kickstand pivot is at (kickstand_y_mm, 0) in
    world frame (leg tip on ground). We compute torque about this point.
    """
    roll_rad = np.radians(roll_deg)

    # CoG at phi=0 inverted: (0, 40) in world frame
    inv_vertices = vertices.copy()
    inv_vertices[:, 1] *= -1
    min_z = inv_vertices[:, 1].min()
    cog_initial = np.array([0.0, -min_z])  # (0, 40)

    # Kickstand pivot on ground
    pivot = np.array([kickstand_y_mm, 0.0])

    # Rotate CoG around kickstand pivot
    if roll_rad > 0:
        rot = _rot2d(-roll_rad)
        cog_rotated = pivot + rot @ (cog_initial - pivot)
    else:
        cog_rotated = cog_initial

    horizontal_mm = cog_rotated[0] - pivot[0]
    torque_nm = mass_kg * G * (horizontal_mm / 1000.0)
    return torque_nm


# ──────────────────────────────────────────────────────────────
# Analyzers
# ──────────────────────────────────────────────────────────────

def analyze_self_right(strategies=None, n_points=361):
    """Sweep roll angle 0-180 deg, compute torques for each strategy.

    Returns dict with roll_angles, gravity_torques, and per-strategy results.
    """
    if strategies is None:
        strategies = list(STRATEGIES.keys())

    roll_angles = np.linspace(0, 180, n_points)
    grav_torques = np.array([gravity_torque(phi, CHASSIS_VERTICES_YZ) for phi in roll_angles])

    results = {
        "roll_angles": roll_angles,
        "gravity_torques": grav_torques,
        "strategies": {},
    }

    for sname in strategies:
        s_torque = servo_roll_torque(sname)
        net = s_torque + grav_torques  # servo (constant) + gravity (varies)

        # Find critical angle where net crosses zero (pro-roll -> anti-roll)
        # At phi=0, net should be positive (servo > gravity restoring)
        # for good strategies. We want where net first goes positive
        # or where it crosses zero.
        zero_crossings = []
        for i in range(len(net) - 1):
            if net[i] * net[i + 1] < 0:
                # Linear interpolation
                frac = -net[i] / (net[i + 1] - net[i])
                cross_angle = roll_angles[i] + frac * (roll_angles[i + 1] - roll_angles[i])
                zero_crossings.append(cross_angle)

        # Margin at phi=0: servo_torque / abs(gravity_torque_at_0)
        grav_at_0 = abs(grav_torques[0]) if abs(grav_torques[0]) > 1e-9 else 1e-9
        margin = abs(s_torque) / grav_at_0

        results["strategies"][sname] = {
            "servo_torque": s_torque,
            "net_torque": net,
            "zero_crossings": zero_crossings,
            "margin": margin,
            "kickstand_risk": STRATEGIES[sname]["kickstand_risk"],
            "label": STRATEGIES[sname]["label"],
        }

    return results


def analyze_kickstand(n_points=181):
    """Compare no-kickstand vs kickstand at various leg angles.

    Returns dict with roll angles, gravity torques for each scenario.
    """
    roll_angles = np.linspace(0, 90, n_points)

    # No kickstand: pivot at vertex 1 (y=41 mm)
    no_kick = np.array([gravity_torque(phi, CHASSIS_VERTICES_YZ) for phi in roll_angles])

    # Kickstand at 180 deg (straight down): pivot at y=80 mm
    kick_180 = np.array([
        kickstand_gravity_torque(phi, kickstand_y_mm=80.0) for phi in roll_angles
    ])

    # Kickstand at 150 deg: leg angled inward, effective y ~ 80 - 125*sin(30) = 17.5
    # Actually the leg tip position depends on the leg angle from vertical.
    # At 180 deg (straight down from shaft in inverted frame): tip at y=80
    # At 150 deg: tip at y = 80 + 125*sin(30 deg in body frame)
    # In inverted frame, 150 deg means the leg is 30 deg from straight-down toward body
    # tip_y = shaft_y - leg_radius * sin(30 deg) = 80 - 125*0.5 = 17.5 mm
    kick_150 = np.array([
        kickstand_gravity_torque(phi, kickstand_y_mm=17.5) for phi in roll_angles
    ])

    # Kickstand at 210 deg: leg angled outward
    # tip_y = 80 + 125*sin(30 deg) = 80 + 62.5 = 142.5 mm
    kick_210 = np.array([
        kickstand_gravity_torque(phi, kickstand_y_mm=142.5) for phi in roll_angles
    ])

    servo_torque = servo_roll_torque("counter_rotate_middle_stop")

    results = {
        "roll_angles": roll_angles,
        "servo_torque": servo_torque,
        "scenarios": {
            "no_kickstand (pivot y=41mm)": {
                "gravity": no_kick, "pivot_y": 41.0,
            },
            "kickstand 180 deg (pivot y=80mm)": {
                "gravity": kick_180, "pivot_y": 80.0,
            },
            "kickstand 150 deg (pivot y=17.5mm)": {
                "gravity": kick_150, "pivot_y": 17.5,
            },
            "kickstand 210 deg (pivot y=142.5mm)": {
                "gravity": kick_210, "pivot_y": 142.5,
            },
        },
    }
    return results


def analyze_slope(slope_deg, direction="lateral"):
    """Slope stability analysis.

    For a lateral slope, gravity has a component that shifts the CoG
    sideways relative to the support polygon.
    """
    strategies = list(STRATEGIES.keys())
    slope_rad = np.radians(slope_deg)

    # On a lateral slope, the effective gravity component trying to
    # tip the robot is m*g*sin(slope). The restoring torque from the
    # support polygon depends on the distance from CoG projection to
    # the support edge.

    # For an upright hexapod, the support polygon is defined by the
    # ground-contact points of the legs. On a slope, we check if
    # the CoG projection (shifted by slope) stays within the polygon.

    # Simplified: the critical metric is whether the servo torque can
    # overcome gravity on the slope for self-righting.
    grav_component = MASS_KG * G * np.sin(slope_rad)  # lateral force (N)
    normal_component = MASS_KG * G * np.cos(slope_rad)

    print(f"\n=== Slope Stability Analysis ({slope_deg} deg, {direction}) ===")
    print(f"Gravity lateral component: {grav_component:.2f} N")
    print(f"Gravity normal component:  {normal_component:.2f} N")
    print()

    # For self-righting on a slope: the gravity torque at phi=0 changes
    # because the effective g is tilted.
    # Worst case: rolling UPHILL. The slope adds to the restoring torque.
    # Best case: rolling DOWNHILL. The slope helps.
    pivot_y_mm = 41.0  # vertex 1
    cog_height_mm = 40.0  # body center height above ground when inverted

    # Uphill roll: extra restoring torque from slope
    extra_torque_uphill = MASS_KG * G * np.sin(slope_rad) * (cog_height_mm / 1000.0)
    base_restoring = abs(gravity_torque(0, CHASSIS_VERTICES_YZ))

    print(f"{'Strategy':<35} {'Servo':>10} {'Base Req':>10} {'Uphill Req':>12} {'Uphill Margin':>14}")
    print("-" * 85)
    for sname in strategies:
        s_torque = servo_roll_torque(sname)
        uphill_req = base_restoring + extra_torque_uphill
        margin = abs(s_torque) / uphill_req if uphill_req > 1e-9 else float("inf")
        print(f"{STRATEGIES[sname]['label']:<35} {s_torque:>9.2f}  {base_restoring:>9.2f}  "
              f"{uphill_req:>11.2f}  {margin:>12.2f}x")

    # Downhill: slope assists
    downhill_req = max(0, base_restoring - extra_torque_uphill)
    print(f"\nDownhill required torque: {downhill_req:.2f} N-m (slope assists)")
    print(f"Max slope for counter_rotate to self-right uphill: ", end="")

    # Find max slope where margin >= 1.0
    best_torque = servo_roll_torque("counter_rotate_middle_stop")
    # best_torque >= base_restoring + m*g*sin(theta)*h/1000
    # sin(theta) <= (best_torque - base_restoring) / (m*g*h/1000)
    max_sin = (best_torque - base_restoring) / (MASS_KG * G * cog_height_mm / 1000.0)
    max_sin = np.clip(max_sin, -1, 1)
    max_slope = np.degrees(np.arcsin(max_sin))
    print(f"{max_slope:.1f} deg")

    return {"slope_deg": slope_deg, "max_slope_self_right": max_slope}


# ──────────────────────────────────────────────────────────────
# Plotters
# ──────────────────────────────────────────────────────────────

COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]


def plot_self_right(results, save_path="self_right_analysis.png"):
    """Torque vs roll angle for all strategies."""
    fig, ax = plt.subplots(figsize=(12, 7))

    roll = results["roll_angles"]
    grav = results["gravity_torques"]

    # Gravity torque (dashed red)
    ax.plot(roll, grav, "r--", linewidth=2, label="Gravity torque", zorder=3)

    # Zero line
    ax.axhline(y=0, color="black", linewidth=0.8, zorder=2)

    # Per-strategy net torque
    for i, (sname, sdata) in enumerate(results["strategies"].items()):
        color = COLORS[i % len(COLORS)]
        margin_str = f"{sdata['margin']:.2f}x"
        label = f"{sdata['label']} (margin {margin_str})"
        ax.plot(roll, sdata["net_torque"], color=color, linewidth=1.8,
                label=label, zorder=4)

        # Mark zero crossings
        for zc in sdata["zero_crossings"]:
            ax.axvline(x=zc, color=color, linestyle=":", alpha=0.5)
            ax.plot(zc, 0, "o", color=color, markersize=8, zorder=5)

    # Shaded regions for the best strategy (counter_rotate_middle_stop)
    best = results["strategies"].get("counter_rotate_middle_stop")
    if best is not None:
        net = best["net_torque"]
        ax.fill_between(roll, 0, net, where=(net > 0),
                        alpha=0.08, color="green", label="Pro-roll region")
        ax.fill_between(roll, 0, net, where=(net < 0),
                        alpha=0.08, color="red", label="Anti-roll region")

    ax.set_xlabel("Roll Angle (deg) -- 0=inverted, 180=upright", fontsize=12)
    ax.set_ylabel("Torque (N-m)", fontsize=12)
    ax.set_title("Self-Right Roll: Torque vs Roll Angle", fontsize=14)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 180)

    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"Plot saved: {save_path}")


def plot_kickstand(results, save_path="kickstand_analysis.png"):
    """Kickstand penalty visualization."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    roll = results["roll_angles"]
    servo_t = results["servo_torque"]

    # Left plot: gravity torque for each scenario
    for i, (label, sdata) in enumerate(results["scenarios"].items()):
        color = COLORS[i % len(COLORS)]
        ax1.plot(roll, -sdata["gravity"], color=color, linewidth=2, label=label)

    ax1.axhline(y=servo_t, color="black", linestyle="--", linewidth=1.5,
                label=f"Servo torque = {servo_t:.2f} N-m")
    ax1.set_xlabel("Roll Angle (deg)", fontsize=12)
    ax1.set_ylabel("Required Torque (N-m, absolute)", fontsize=12)
    ax1.set_title("Gravity Restoring Torque by Kickstand Config", fontsize=13)
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    # Right plot: pivot positions
    scenarios = results["scenarios"]
    labels = list(scenarios.keys())
    pivot_ys = [scenarios[k]["pivot_y"] for k in labels]
    short_labels = [k.split("(")[0].strip() for k in labels]

    bars = ax2.barh(range(len(labels)), pivot_ys, color=COLORS[:len(labels)], alpha=0.7)
    ax2.set_yticks(range(len(labels)))
    ax2.set_yticklabels(short_labels, fontsize=9)
    ax2.set_xlabel("Pivot Y position (mm from center)", fontsize=12)
    ax2.set_title("Effective Pivot Location", fontsize=13)
    ax2.grid(True, alpha=0.3, axis="x")

    # Annotate with torque at phi=0
    for i, (label, sdata) in enumerate(scenarios.items()):
        torque_0 = abs(sdata["gravity"][0])
        ax2.text(sdata["pivot_y"] + 2, i, f"{torque_0:.2f} N-m",
                 va="center", fontsize=9, fontweight="bold")

    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"Plot saved: {save_path}")


# ──────────────────────────────────────────────────────────────
# CLI output
# ──────────────────────────────────────────────────────────────

def print_self_right_table(results):
    """Print self-right analysis results table."""
    print("\n=== Self-Right Roll Analysis ===")
    print(f"{'Strategy':<35} {'Servo Torque':>13} {'Required':>10} "
          f"{'Margin':>8} {'Kickstand Risk':>16}")
    print("-" * 86)

    grav_at_0 = abs(results["gravity_torques"][0])

    for sname, sdata in results["strategies"].items():
        st = sdata["servo_torque"]
        risk = "YES - middle stopped" if sdata["kickstand_risk"] else "no"
        print(f"{sdata['label']:<35} {st:>10.2f} N-m {grav_at_0:>7.2f} N-m "
              f"{sdata['margin']:>7.2f}x {risk:>16}")

    # Find critical angle for best strategy
    best = results["strategies"].get("counter_rotate_middle_stop")
    if best and best["zero_crossings"]:
        # The first crossing where gravity starts helping
        print(f"\nCritical angle (counter_rotate): {best['zero_crossings'][0]:.1f} deg "
              "(gravity assists beyond this)")
    else:
        print("\nNet torque stays positive across full range (gravity never dominates)")


def print_kickstand_table(results):
    """Print kickstand analysis results table."""
    print("\n=== Kickstand Analysis ===")
    servo_t = results["servo_torque"]

    base_torque = None
    for label, sdata in results["scenarios"].items():
        torque_0 = abs(sdata["gravity"][0])
        margin = servo_t / torque_0 if torque_0 > 1e-9 else float("inf")

        if base_torque is None:
            base_torque = torque_0
            multiplier_str = "(baseline)"
        else:
            mult = torque_0 / base_torque if base_torque > 1e-9 else 0
            multiplier_str = f"({mult:.2f}x {'harder' if mult > 1 else 'easier'})"

        print(f"{label:<40} pivot y={sdata['pivot_y']:>6.1f}mm, "
              f"required = {torque_0:.2f} N-m {multiplier_str}, "
              f"margin = {margin:.2f}x")


# ──────────────────────────────────────────────────────────────
# Main CLI
# ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Hexapod rover statics analysis tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Examples:\n"
               "  python rover_statics.py self_right\n"
               "  python rover_statics.py kickstand\n"
               "  python rover_statics.py slope 15\n",
    )
    subparsers = parser.add_subparsers(dest="command", help="Analysis mode")

    # self_right
    sp_sr = subparsers.add_parser("self_right", help="Self-right roll torque analysis")
    sp_sr.add_argument("--no-plot", action="store_true", help="Skip plot generation")
    sp_sr.add_argument("-o", "--output", default="self_right_analysis.png",
                       help="Output plot path (default: self_right_analysis.png)")

    # kickstand
    sp_ks = subparsers.add_parser("kickstand", help="Kickstand effect analysis")
    sp_ks.add_argument("--no-plot", action="store_true", help="Skip plot generation")
    sp_ks.add_argument("-o", "--output", default="kickstand_analysis.png",
                       help="Output plot path (default: kickstand_analysis.png)")

    # slope
    sp_sl = subparsers.add_parser("slope", help="Slope stability analysis")
    sp_sl.add_argument("angle", type=float, help="Slope angle in degrees")
    sp_sl.add_argument("--direction", default="lateral",
                       choices=["lateral", "longitudinal"],
                       help="Slope direction (default: lateral)")

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        sys.exit(1)

    if args.command == "self_right":
        results = analyze_self_right()
        print_self_right_table(results)
        if not args.no_plot:
            plot_self_right(results, save_path=args.output)

    elif args.command == "kickstand":
        results = analyze_kickstand()
        print_kickstand_table(results)
        if not args.no_plot:
            plot_kickstand(results, save_path=args.output)

    elif args.command == "slope":
        analyze_slope(args.angle, direction=args.direction)


if __name__ == "__main__":
    main()
