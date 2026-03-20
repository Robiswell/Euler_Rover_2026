#!/usr/bin/env python3
"""
sim_terrain.py  --  Alamosa terrain robustness simulation
NASA Colorado Space Grant competition | April 11, Alamosa CO

Verifies final_full_gait_test.py gait engine against:
  - Wet sand drag (load +100-200 per stance frame)
  - Loose rock spikes (0.5% per stance frame, 2-4 frames, +150-350 load)
  - Fixed rock obstruction (sustained 950+ load -> stall + recovery)
  - Hole encounter (edge spike 750-900, drop 30-50, far wall 700-850)
  - Ramp grades 10 / 15 / 20 deg (load increase + governor check)
  - Worst-case combined: wet sand + loose rocks + 15 deg ramp

Key pass criteria:
  T1  Sand only         -> ZERO stall events (load 250-370, well below 750)
  T2  Sand + rocks      -> stalls only from 3-frame spikes, all <15 frames
  T3  Fixed rock        -> stall within 3 frames, clean exit, no snap
  T4  Hole              -> no false stall on drop phase, spike handled
  T5  Ramp 10deg        -> governor headroom > 0, stall rate same as T2
  T6  Ramp 15deg        -> governor headroom > 0, stall rate same as T2
  T7  Ramp 20deg Quad   -> governor headroom > 0 (Quad governs more easily)
  T8  Worst case        -> forward progress maintained, stall recovery <20 frames
  T9  2-frame hysteresis -> zero stalls from sub-threshold spike patterns
  T10 Wave carve sand   -> governor margin positive, minimal stalls
  T11 Gait transition   -> Quad->Wave mid-scenario on sand, no governor violation
  T12 Timed fallback    -> full fallback sequence on sand, <10% stall fraction
  T13 Overload timing   -> TE cycle fires at 1.5s; capped at 10; no fire at 1.0s
  T14 PhErr governor    -> hz scaled down when error high; no cascade; released on recovery
"""

import sys, io, math, random
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

# ── Constants (must match final_full_gait_test.py) ───────────────────────
ENCODER_RESOLUTION = 4096.0
VELOCITY_SCALAR    = 1.85
LEFT_SERVOS  = [2, 3, 4]
RIGHT_SERVOS = [1, 6, 5]
ALL_SERVOS   = LEFT_SERVOS + RIGHT_SERVOS
LEG_SPLAY      = {1:-35, 2:-35, 6:0, 3:0, 5:35, 4:35}
KP_PHASE        = 12.0
STALL_THRESHOLD = 750
real_dt           = 0.02
GAITS = {
    0: {'duty': 0.5,  'offsets': {2:0.0, 6:0.0, 4:0.0,  1:0.5, 3:0.5, 5:0.5}},
    1: {'duty': 0.60, 'offsets': {5:0.0, 3:0.167, 1:0.333, 4:0.5, 6:0.667, 2:0.833}},
    2: {'duty': 0.67, 'offsets': {2:0.0, 5:0.0, 3:0.333, 6:0.333, 4:0.666, 1:0.666}},
}

# Overload prevention constants (must match final_full_gait_test.py)
OVERLOAD_PREVENTION_TIME = 1.5   # seconds: stall duration that triggers TE cycle
OVERLOAD_MAX_CYCLES      = 10    # max TE cycles per stall event before cap

# PhErr governor constants (must match final_full_gait_test.py)
PHERR_ENGAGE_DEG  = 30.0    # deg: engage governor when max phase error exceeds this
PHERR_RELEASE_DEG = 20.0    # deg: release governor when error drops below this
PHERR_FLOOR_SCALE = 0.35    # minimum hz scale when governor is fully engaged
PHERR_RAMP_WIDTH  = 120.0   # deg: error range over which scale ramps from 1.0 to floor
KAPPA_GOVERNOR    = 3.0     # exponential decay rate for governor scaling

# ── Kinematics (sync: final_full_gait_test.py lines 159-178) ─────────────
def get_buehler_angle(t_norm, duty_cycle, start_ang, end_ang, is_reversed=False):
    stance_sweep = (end_ang - start_ang + 180) % 360 - 180
    if is_reversed:
        start_ang, end_ang = end_ang, start_ang
        stance_sweep = -stance_sweep
    if t_norm <= duty_cycle:
        progress = t_norm / duty_cycle
        angle = start_ang + (stance_sweep * progress)
    else:
        progress  = (t_norm - duty_cycle) / (1.0 - duty_cycle)
        air_sweep = 360.0 - abs(stance_sweep)
        if stance_sweep < 0:
            air_sweep = -air_sweep
        angle = end_ang + (air_sweep * progress)
    return angle % 360

def get_air_sweep(stance_sweep):
    air_sweep = 360.0 - abs(stance_sweep)
    if stance_sweep < 0: air_sweep = -air_sweep
    return air_sweep

# ── Terrain load model ────────────────────────────────────────────────────────
class TerrainGen:
    """
    Stateful per-scenario terrain load generator.
    get_load(sid, in_stance) -> synthetic load register value [0..1023].

    terrain_type choices:
      'smooth'           base load only
      'wet_sand'         +100-200 per stance frame
      'sand_rock'        wet_sand + 0.5% Poisson loose rock spikes
      'worst_case'       sand_rock + ramp load
      'fixed_rock'       arm_fixed_rock() needed; injects 950+ for N frames
      'hole'             arm_hole() needed; 3-phase load profile on one leg
      'spike_2on_2off'   deterministic 2-frame-above / 2-frame-below pattern at 800/200
      'sustained_stall'  every stance frame on the target servo returns fixed_load
    """
    def __init__(self, terrain_type, ramp_deg=0.0, seed=42,
                 sustained_sid=None, sustained_load=800):
        self.terrain  = terrain_type
        self.ramp_sin = math.sin(math.radians(ramp_deg))
        self.rng      = random.Random(seed)
        self._rock_cd  = {s: 0 for s in ALL_SERVOS}
        self._hole_cd  = {s: (0, 0) for s in ALL_SERVOS}
        self._fr_sid   = None
        self._fr_cd    = 0
        # sustained_stall terrain: one servo held above threshold indefinitely
        self._sus_sid  = sustained_sid
        self._sus_load = sustained_load

    def arm_fixed_rock(self, sid, duration=20):
        self._fr_sid = sid
        self._fr_cd  = duration

    def arm_hole(self, sid):
        self._hole_cd[sid] = (1, 3)

    def get_load(self, sid, in_stance, tick=0):
        # spike_2on_2off: deterministic pattern, ignores stance phase
        if self.terrain == 'spike_2on_2off':
            return 800 if (tick // 2) % 2 == 0 else 200

        # sustained_stall: target servo always above threshold while in stance
        if self.terrain == 'sustained_stall' and sid == self._sus_sid and in_stance:
            return self._sus_load

        if not in_stance:
            return self.rng.randint(50, 90)

        load = self.rng.randint(130, 170)

        # ── fixed rock (overrides everything) ──
        if self._fr_cd > 0 and sid == self._fr_sid:
            self._fr_cd -= 1
            return self.rng.randint(950, 1000)

        # ── hole scenario for this leg ──
        phase, cd = self._hole_cd.get(sid, (0, 0))
        if phase > 0:
            cd -= 1
            if phase == 1:   load = self.rng.randint(750, 900)
            elif phase == 2: load = self.rng.randint(30,   50)
            elif phase == 3: load = self.rng.randint(700, 850)
            if cd <= 0:
                next_phase = phase + 1 if phase < 3 else 0
                self._hole_cd[sid] = (next_phase, 3) if next_phase else (0, 0)
            else:
                self._hole_cd[sid] = (phase, cd)
            return min(1023, max(0, int(load)))

        # ── wet sand drag ──
        if self.terrain in ('wet_sand', 'sand_rock', 'worst_case'):
            load += self.rng.randint(100, 200)

        # ── ramp torque increase ──
        if self.ramp_sin > 0:
            load += int(load * self.ramp_sin * 0.6)

        # ── loose rock spike (Poisson) ──
        if self.terrain in ('sand_rock', 'worst_case'):
            if self._rock_cd[sid] > 0:
                load += self.rng.randint(150, 350)
                self._rock_cd[sid] -= 1
            elif self.rng.random() < 0.005:
                self._rock_cd[sid] = self.rng.randint(2, 4)
                load += self.rng.randint(150, 350)

        return min(1023, max(0, int(load)))


# ── Scenario runner ───────────────────────────────────────────────────────────
def run_scenario(name, gait_id, speed, turn_bias, frames,
                 terrain_type='smooth', ramp_deg=0.0, seed=42,
                 fixed_rock_at=None, fixed_rock_sid=None, fixed_rock_dur=20,
                 hole_at=None, hole_sid=None,
                 gait_schedule=None):
    """
    Run N frames of the v2 gait engine with terrain-modulated stall injection.
    Returns a result dict with all metrics.

    gait_schedule: optional list of (frame_number, params_dict) tuples.
        At frame_number, update gait_id/speed/turn_bias targets.
        LERP smooths the transition -- matches production engine behavior.
    """
    rng = random.Random(seed)
    tgen = TerrainGen(terrain_type, ramp_deg=ramp_deg, seed=seed+1)

    # Gait state
    g = GAITS[gait_id]
    smooth_duty     = g['duty']
    smooth_offsets  = {s: g['offsets'][s] for s in ALL_SERVOS}
    smooth_speed    = speed * 1.0
    smooth_turn     = turn_bias * 1.0
    imp_start       = 320.0
    imp_end         = 40.0

    master_L = 0.0; master_R = 0.0
    actual_phases = {s: 0.0 for s in ALL_SERVOS}

    # Stall state (v2 hysteresis: 3-frame set, 3-frame clear, no INSTANT_STALL)
    is_stalled     = {s: False for s in ALL_SERVOS}
    stall_ctr      = {s: 0     for s in ALL_SERVOS}

    # Metrics
    stall_events   = {s: 0 for s in ALL_SERVOS}
    stall_dur_max  = {s: 0 for s in ALL_SERVOS}
    stall_cur_dur  = {s: 0 for s in ALL_SERVOS}
    prev_stalled   = {s: False for s in ALL_SERVOS}
    speed_max        = {s: 0    for s in ALL_SERVOS}
    exit_snap_max    = {s: 0.0  for s in ALL_SERVOS}
    prev_final_spd   = {s: 0    for s in ALL_SERVOS}
    prev_was_stalled = {s: False for s in ALL_SERVOS}
    gov_headroom_min = float('inf')
    total_stall_frames = 0
    gap56_min      = 360.0
    prev_dps_ff      = {s: None for s in ALL_SERVOS}
    prev_in_stance_ff = {s: True for s in ALL_SERVOS}
    max_ff_jump      = 0.0

    for tick in range(frames):
        # Arm terrain events at specified tick
        if tick == fixed_rock_at and fixed_rock_sid is not None:
            tgen.arm_fixed_rock(fixed_rock_sid, fixed_rock_dur)
        if tick == hole_at and hole_sid is not None:
            tgen.arm_hole(hole_sid)

        # Apply gait schedule (mid-scenario gait/speed changes)
        if gait_schedule:
            for sched_frame, sched_params in gait_schedule:
                if tick == sched_frame:
                    if 'gait_id' in sched_params:
                        gait_id = sched_params['gait_id']
                        g = GAITS[gait_id]
                    if 'speed' in sched_params:
                        speed = sched_params['speed']
                    if 'turn_bias' in sched_params:
                        turn_bias = sched_params['turn_bias']

        # v2: single LERP rate for all smoothing: min(1.0, 4.0 * real_dt)
        lr = min(1.0, 4.0 * real_dt)

        smooth_speed  += (speed    - smooth_speed)  * lr
        smooth_turn   += (turn_bias - smooth_turn)  * lr
        smooth_duty   += (g['duty'] - smooth_duty)  * lr
        for s in ALL_SERVOS:
            od = (g['offsets'][s] - smooth_offsets[s] + 0.5) % 1.0 - 0.5
            smooth_offsets[s] = (smooth_offsets[s] + od * lr) % 1.0

        base_hz  = smooth_speed / 1000.0
        hz_L_raw = base_hz + smooth_turn
        hz_R_raw = base_hz - smooth_turn

        base_sweep = (imp_end - imp_start + 180) % 360 - 180
        air_sweep_val = get_air_sweep(base_sweep)
        # v2 governor: NO pi/2 factor
        max_safe = (660.0 / VELOCITY_SCALAR * (1.0 - smooth_duty)) / max(5.0, abs(air_sweep_val))

        # governor headroom (outer wheel)
        outer_hz = max(abs(hz_L_raw), abs(hz_R_raw))
        headroom = max_safe - outer_hz
        if headroom < gov_headroom_min:
            gov_headroom_min = headroom

        hz_L = max(-max_safe, min(max_safe, hz_L_raw))
        hz_R = max(-max_safe, min(max_safe, hz_R_raw))

        # v2 master clock: abs(hz)
        master_L = (master_L + abs(hz_L) * real_dt) % 1.0
        master_R = (master_R + abs(hz_R) * real_dt) % 1.0

        # v2 clock sync: 0.005 deadband, unclamped decay at 2x rate (not 4x)
        if abs(smooth_turn) < 0.01 and abs(smooth_speed) > 10.0 and hz_L * hz_R > 0:
            cd  = (master_L - master_R + 0.5) % 1.0 - 0.5
            if abs(cd) > 0.005:
                dec = cd * min(1.0, 2.0 * real_dt)
                master_L = (master_L - dec/2) % 1.0
                master_R = (master_R + dec/2) % 1.0

        # Per-servo kinematics + stall detection
        tar_ph = {}
        for sid in ALL_SERVOS:
            s_hz = hz_L if sid in LEFT_SERVOS else hz_R
            m_t  = master_L if sid in LEFT_SERVOS else master_R
            t_leg = (m_t + smooth_offsets[sid]) % 1.0
            in_stance = (t_leg <= smooth_duty)

            # v2 is_reversed flag (matches production: leg_hz < 0)
            is_rev = (s_hz < 0)

            # ── Terrain load -> stall detection (v2: no INSTANT_STALL) ──
            load = tgen.get_load(sid, in_stance, tick=tick)

            if load > STALL_THRESHOLD:
                stall_ctr[sid] = min(3, stall_ctr[sid] + 1)
                if stall_ctr[sid] >= 3 and not is_stalled[sid]:
                    stall_events[sid] += 1
                    is_stalled[sid]    = True
            else:
                stall_ctr[sid] = max(0, stall_ctr[sid] - 1)
                if stall_ctr[sid] == 0 and is_stalled[sid]:
                    is_stalled[sid] = False

            # Stall duration tracking
            if is_stalled[sid]:
                stall_cur_dur[sid] += 1
            else:
                if stall_cur_dur[sid] > stall_dur_max[sid]:
                    stall_dur_max[sid] = stall_cur_dur[sid]
                stall_cur_dur[sid] = 0

            prev_stalled[sid] = is_stalled[sid]

            # ── Kinematic command ──
            if is_stalled[sid]:
                final_speed = 0
                dps = 0.0
            else:
                cur_ph = actual_phases[sid]
                tp = get_buehler_angle(t_leg, smooth_duty, imp_start, imp_end, is_rev)
                tp = (tp + LEG_SPLAY.get(sid, 0)) % 360
                tar_ph[sid] = tp

                # v2 feedforward: constant-per-phase (absolute values, two zones)
                if in_stance:
                    dps = (abs(base_sweep) * abs(s_hz)) / smooth_duty
                else:
                    dps = (abs(air_sweep_val) * abs(s_hz)) / (1.0 - smooth_duty)

                ff_sp = dps * VELOCITY_SCALAR

                # v2 phase error
                se = (tp - cur_ph + 180) % 360 - 180
                if not is_rev and se < -90: se += 360
                elif is_rev and se > 90: se -= 360

                # v2: flat KP_PHASE, no scaling, no error clamping
                pd = se * KP_PHASE

                # v2 sign chain: two independent if statements (cumulative)
                raw = ff_sp + pd
                if is_rev:       raw = -raw
                if sid in LEFT_SERVOS: raw = -raw

                # v2: no stall exit ramp, no low-speed deadband
                final_speed = max(-3000, min(3000, int(raw)))

            # Metrics
            if abs(final_speed) > speed_max[sid]: speed_max[sid] = abs(final_speed)
            if prev_was_stalled[sid] and not is_stalled[sid]:
                esnap = abs(final_speed)
                if esnap > exit_snap_max[sid]: exit_snap_max[sid] = esnap
            prev_was_stalled[sid] = is_stalled[sid]
            prev_final_spd[sid] = final_speed
            if is_stalled[sid]: total_stall_frames += 1

            # Feedforward discontinuity tracking (exempt boundary crossings and stall transitions)
            if not is_stalled[sid]:
                if prev_dps_ff[sid] is not None and prev_in_stance_ff[sid] == in_stance:
                    ff_jump = abs(dps - prev_dps_ff[sid])
                    if ff_jump > max_ff_jump:
                        max_ff_jump = ff_jump
                prev_dps_ff[sid] = dps
                prev_in_stance_ff[sid] = in_stance
            else:
                prev_dps_ff[sid] = None  # reset on stall

            actual_phases[sid] = (actual_phases[sid] + final_speed / VELOCITY_SCALAR * real_dt) % 360

        # gap56 informational
        if 5 in tar_ph and 6 in tar_ph:
            gap = abs((tar_ph[5] - tar_ph[6] + 180) % 360 - 180)
            if gap < gap56_min:
                gap56_min = gap

    # flush remaining stall streaks
    for sid in ALL_SERVOS:
        if stall_cur_dur[sid] > stall_dur_max[sid]:
            stall_dur_max[sid] = stall_cur_dur[sid]

    total_stalls   = sum(stall_events.values())
    max_dur        = max(stall_dur_max.values())
    stall_fraction = total_stall_frames / max(1, frames * len(ALL_SERVOS))
    gov_ok         = gov_headroom_min >= 0
    max_exit_snap  = max(exit_snap_max.values())

    return {
        'name':            name,
        'frames':          frames,
        'total_stalls':    total_stalls,
        'stall_by_servo':  stall_events,
        'max_stall_dur':   max_dur,
        'stall_fraction':  stall_fraction,
        'max_speed_sts':   max(speed_max.values()),
        'max_exit_snap':   max_exit_snap,
        'gov_headroom_hz': gov_headroom_min,
        'gov_ok':          gov_ok,
        'gap56_min_deg':   gap56_min,
        'max_ff_jump':     max_ff_jump,
    }


# ── Scenario definitions ──────────────────────────────────────────────────────
def define_scenarios():
    sc = []

    # T1: Wet sand drag only, Wave gait 60 s
    sc.append(dict(name="T1 Wet sand Wave",
        gait_id=1, speed=350, turn_bias=0.0, frames=3000,
        terrain_type='wet_sand', ramp_deg=0.0))

    # T2: Sand + loose rock spikes, Wave gait 60 s
    sc.append(dict(name="T2 Sand+rocks Wave",
        gait_id=1, speed=350, turn_bias=0.0, frames=3000,
        terrain_type='sand_rock', ramp_deg=0.0))

    # T3: Fixed rock obstruction on servo 1, Wave gait
    sc.append(dict(name="T3 Fixed rock stall",
        gait_id=1, speed=350, turn_bias=0.0, frames=500,
        terrain_type='wet_sand',
        fixed_rock_at=100, fixed_rock_sid=1, fixed_rock_dur=20))

    # T4: Hole on servo 2, Wave gait
    sc.append(dict(name="T4 Hole encounter",
        gait_id=1, speed=350, turn_bias=0.0, frames=400,
        terrain_type='wet_sand',
        hole_at=50, hole_sid=2))

    # T5: Ramp 10 deg, Wave gait
    sc.append(dict(name="T5 Ramp 10deg Wave",
        gait_id=1, speed=350, turn_bias=0.0, frames=2000,
        terrain_type='wet_sand', ramp_deg=10.0))

    # T6: Ramp 15 deg, Wave gait
    sc.append(dict(name="T6 Ramp 15deg Wave",
        gait_id=1, speed=350, turn_bias=0.0, frames=2000,
        terrain_type='wet_sand', ramp_deg=15.0))

    # T7: Ramp 20 deg, Quadruped gait
    sc.append(dict(name="T7 Ramp 20deg Quad",
        gait_id=2, speed=400, turn_bias=0.0, frames=2000,
        terrain_type='wet_sand', ramp_deg=20.0))

    # T8: Worst case -- wet sand + loose rocks + 15 deg ramp, Wave gait
    sc.append(dict(name="T8 Worst case Wave",
        gait_id=1, speed=350, turn_bias=0.0, frames=3000,
        terrain_type='worst_case', ramp_deg=15.0))

    # T9: Hysteresis verification -- 2-frame spikes should NOT cause stall
    #      spike_2on_2off terrain: load 800 for 2 frames, 200 for 2 frames, repeat
    sc.append(dict(name="T9 2-frame spike hysteresis",
        gait_id=0, speed=800, turn_bias=0.0, frames=1000,
        terrain_type='spike_2on_2off'))

    # T10: Wave carve turn during sand -- governor margin check
    sc.append(dict(name="T10 Wave carve left sand",
        gait_id=1, speed=350, turn_bias=-0.12, frames=1500,
        terrain_type='wet_sand', ramp_deg=0.0))

    # T11: Gait transition under terrain load (Quad@400 -> Wave@350 at frame 500)
    sc.append(dict(name="T11 Gait transition wet sand",
        gait_id=2, speed=400, turn_bias=0.0, frames=1500,
        terrain_type='wet_sand',
        gait_schedule=[(500, {'gait_id': 1, 'speed': 350})]))

    # T12: Timed fallback sequence on wet sand (Quad@400 45s -> Wave@350 30s -> decel)
    sc.append(dict(name="T12 Timed fallback wet sand",
        gait_id=2, speed=400, turn_bias=0.0, frames=3900,
        terrain_type='wet_sand', seed=42,
        gait_schedule=[(2250, {'gait_id': 1, 'speed': 350}),
                       (3750, {'speed': 0})]))

    return sc


# ── T13: Overload prevention timing ──────────────────────────────────────────
def run_t13_overload_timing():
    """
    Property: The overload prevention timer fires a TE cycle when a servo is
    stalled continuously for >= OVERLOAD_PREVENTION_TIME (1.5s), the cycle
    counter is capped at OVERLOAD_MAX_CYCLES (10), and a 1.0s stall does NOT
    fire a cycle.

    Failure means: On hardware, a servo held in stall for > 1.5s without a TE
    cycle would draw 2.7A sustained until overcurrent lockout (~2s window).
    A missing cap means the counter overflows and the TE dwell logic misbehaves.
    A 1.0s stall triggering a cycle wastes 0.5s of motion on sand for no reason.

    Known false positive risk: None. Timing is deterministic -- stall_time
    increments by real_dt each frame while load > STALL_THRESHOLD.

    Sub-checks:
      T13.A: 2.0s sustained stall -> stall_time exceeds OVERLOAD_PREVENTION_TIME
      T13.B: overload_cycle_count increments on each threshold crossing
      T13.C: cycle count is capped at OVERLOAD_MAX_CYCLES after 10 crossings
      T13.D: 1.0s stall does NOT trigger an overload cycle
    """
    # T13.A + T13.B: simulate servo 1 stalled for 2.0 s then released, repeated
    # 12 times (to exercise cap at 10). Each repetition: 2.0s stall + 0.5s clear.
    STALL_DURATION_S = 2.0     # seconds to hold above threshold
    CLEAR_DURATION_S = 0.5     # seconds below threshold between events
    REPETITIONS      = 12      # intentionally exceeds OVERLOAD_MAX_CYCLES=10

    overload_cycle_count = 0
    stall_time_s = 0.0         # seconds servo has been continuously stalled
    stall_start  = None        # wall-clock time when stall began (None = not stalled)

    fails = []

    # Drive deterministic load sequence: REPETITIONS x (stall then clear)
    total_frames = int((STALL_DURATION_S + CLEAR_DURATION_S) * REPETITIONS / real_dt)
    cycle_period_frames = int((STALL_DURATION_S + CLEAR_DURATION_S) / real_dt)
    stall_frames        = int(STALL_DURATION_S / real_dt)

    current_time = 0.0
    for tick in range(total_frames):
        phase_in_cycle = tick % cycle_period_frames
        load = 800 if phase_in_cycle < stall_frames else 200  # deterministic

        # Overload prevention timer -- direct threshold check, no hysteresis (matches production)
        if load > STALL_THRESHOLD:
            if stall_start is None:
                stall_start = current_time
            elif (current_time - stall_start) >= OVERLOAD_PREVENTION_TIME:
                if overload_cycle_count < OVERLOAD_MAX_CYCLES:
                    overload_cycle_count += 1
                stall_start = current_time  # reset timer for next cycle
        else:
            stall_start = None

        current_time += real_dt

    # T13.A: stall duration of 2.0s must exceed OVERLOAD_PREVENTION_TIME (1.5s)
    if STALL_DURATION_S <= OVERLOAD_PREVENTION_TIME:
        fails.append("T13.A: TEST SETUP ERROR -- STALL_DURATION_S must exceed "
                     f"OVERLOAD_PREVENTION_TIME ({OVERLOAD_PREVENTION_TIME}s)")

    # T13.B: cycle count must be > 0 (threshold was crossed at least once)
    if overload_cycle_count == 0:
        fails.append("T13.B: OVERLOAD CYCLE NOT FIRED -- no TE cycle triggered "
                     f"after {STALL_DURATION_S}s stall (threshold {OVERLOAD_PREVENTION_TIME}s)")

    # T13.C: cycle count must be capped at OVERLOAD_MAX_CYCLES despite 12 repetitions
    if overload_cycle_count > OVERLOAD_MAX_CYCLES:
        fails.append(f"T13.C: OVERLOAD CAP VIOLATED -- count={overload_cycle_count} "
                     f"exceeds OVERLOAD_MAX_CYCLES={OVERLOAD_MAX_CYCLES}")

    # T13.D: a 1.0s stall must NOT trigger a cycle (below 1.5s threshold)
    SHORT_STALL_S = 1.0
    short_stall_frames = int(SHORT_STALL_S / real_dt)
    short_cycle_count  = 0
    s_stall_start  = None
    s_current_time = 0.0
    for tick in range(short_stall_frames + 10):  # +10 frames tail
        load = 800 if tick < short_stall_frames else 200
        if load > STALL_THRESHOLD:
            if s_stall_start is None:
                s_stall_start = s_current_time
            elif (s_current_time - s_stall_start) >= OVERLOAD_PREVENTION_TIME:
                short_cycle_count += 1
                s_stall_start = s_current_time
        else:
            s_stall_start = None
        s_current_time += real_dt
    if short_cycle_count > 0:
        fails.append(f"T13.D: FALSE OVERLOAD CYCLE -- 1.0s stall triggered "
                     f"{short_cycle_count} cycle(s); threshold is {OVERLOAD_PREVENTION_TIME}s")

    return {
        'name':                'T13 Overload prevention timing',
        'passed':              len(fails) == 0,
        'fails':               fails,
        'overload_cycles_fired': overload_cycle_count,
        'overload_cap':        OVERLOAD_MAX_CYCLES,
        'stall_duration_s':    STALL_DURATION_S,
    }


# ── T14: PhErr governor under terrain load ────────────────────────────────────
def run_t14_pherr_governor():
    """
    Property: The PhErr governor scales down hz proportionally when max phase
    error exceeds PHERR_ENGAGE_DEG (30 deg), does not cause a stall cascade
    while throttling, and releases (restores hz scale to 1.0) when error drops
    below PHERR_RELEASE_DEG (20 deg).

    Failure means: On hardware, a governor that fails to scale down hz when
    phase error is high will command speeds the servo cannot track, growing
    phase error further until the stuck-timeout stall-escalation fires. A
    governor that never releases will permanently throttle the robot to 35% of
    commanded speed, preventing it from completing the Alamosa course.

    Known false positive risk: None. Phase error is injected deterministically
    and ph_scale is computed analytically -- no floating point accumulation.

    Sub-checks:
      T14.A: inject phase error > PHERR_ENGAGE_DEG, verify ph_scale < 1.0
      T14.B: while governor is active, stall count from governor-limited speed
             must be <= 2 (no cascade on wet sand within 500 frames)
      T14.C: reduce injected error below PHERR_RELEASE_DEG, verify governor
             releases (ph_scale returns to 1.0)
    """
    # Governor logic (mirrors final_full_gait_test.py lines 997-1020)
    def compute_ph_scale(max_phase_error_deg, gov_active_prev):
        """Stateless per-tick governor computation using hysteresis engage/release."""
        if gov_active_prev:
            # Release when error drops below release threshold
            if max_phase_error_deg < PHERR_RELEASE_DEG:
                return 1.0, False
        else:
            # Engage when error exceeds engage threshold
            if max_phase_error_deg < PHERR_ENGAGE_DEG:
                return 1.0, False

        # Governor is active: exponential decay matching production formula
        raw_scale = PHERR_FLOOR_SCALE + (1.0 - PHERR_FLOOR_SCALE) * math.exp(
            -KAPPA_GOVERNOR * (max_phase_error_deg - PHERR_ENGAGE_DEG) / PHERR_RAMP_WIDTH)
        ph_scale = max(PHERR_FLOOR_SCALE, raw_scale)
        return ph_scale, True

    # Wave gait on wet sand, 500 frames
    gait_id = 1
    speed   = 350
    g = GAITS[gait_id]
    tgen = TerrainGen('wet_sand', seed=42)

    smooth_duty    = g['duty']
    smooth_offsets = {s: g['offsets'][s] for s in ALL_SERVOS}
    smooth_speed   = speed * 1.0
    imp_start = 320.0; imp_end = 40.0
    master_L = 0.0; master_R = 0.0
    actual_phases = {s: 0.0 for s in ALL_SERVOS}
    is_stalled = {s: False for s in ALL_SERVOS}
    stall_ctr  = {s: 0 for s in ALL_SERVOS}
    stall_count = 0

    gov_active = False
    ph_scale   = 1.0
    fails      = []

    # T14.A: inject high error for first 300 frames, check scale is reduced
    HIGH_ERROR_DEG = 45.0    # above PHERR_ENGAGE_DEG (30 deg)
    LOW_ERROR_DEG  = 10.0    # below PHERR_RELEASE_DEG (20 deg)
    PHASE_A_FRAMES = 300
    PHASE_B_FRAMES = 200

    scale_when_high = []     # collect ph_scale while error is injected high
    scale_when_low  = []     # collect ph_scale while error is injected low
    gov_active_when_low = [] # collect governor state after error drops

    lr = min(1.0, 4.0 * real_dt)

    for tick in range(PHASE_A_FRAMES + PHASE_B_FRAMES):
        smooth_speed += (speed - smooth_speed) * lr
        smooth_duty  += (g['duty'] - smooth_duty) * lr
        for s in ALL_SERVOS:
            od = (g['offsets'][s] - smooth_offsets[s] + 0.5) % 1.0 - 0.5
            smooth_offsets[s] = (smooth_offsets[s] + od * lr) % 1.0

        base_hz = smooth_speed / 1000.0

        base_sweep    = (imp_end - imp_start + 180) % 360 - 180
        air_sweep_val = get_air_sweep(base_sweep)
        max_safe = (660.0 / VELOCITY_SCALAR * (1.0 - smooth_duty)) / max(5.0, abs(air_sweep_val))

        # Inject synthetic phase error (replaces the real per-servo accumulation)
        injected_error = HIGH_ERROR_DEG if tick < PHASE_A_FRAMES else LOW_ERROR_DEG
        ph_scale, gov_active = compute_ph_scale(injected_error, gov_active)

        # Apply governor scaling to hz (matches production: hz_L and hz_R scaled)
        effective_hz = base_hz * ph_scale
        hz_L = max(-max_safe, min(max_safe, effective_hz))
        hz_R = max(-max_safe, min(max_safe, effective_hz))

        master_L = (master_L + abs(hz_L) * real_dt) % 1.0
        master_R = (master_R + abs(hz_R) * real_dt) % 1.0

        for sid in ALL_SERVOS:
            m_t = master_L if sid in LEFT_SERVOS else master_R
            t_leg = (m_t + smooth_offsets[sid]) % 1.0
            in_stance = (t_leg <= smooth_duty)
            load = tgen.get_load(sid, in_stance, tick=tick)

            if load > STALL_THRESHOLD:
                stall_ctr[sid] = min(3, stall_ctr[sid] + 1)
                if stall_ctr[sid] >= 3 and not is_stalled[sid]:
                    is_stalled[sid] = True
                    stall_count += 1
            else:
                stall_ctr[sid] = max(0, stall_ctr[sid] - 1)
                if stall_ctr[sid] == 0:
                    is_stalled[sid] = False

        if tick < PHASE_A_FRAMES:
            scale_when_high.append(ph_scale)
        else:
            scale_when_low.append(ph_scale)
            gov_active_when_low.append(gov_active)

    # T14.A: ph_scale must be < 1.0 while error is above engage threshold
    max_scale_during_high = max(scale_when_high) if scale_when_high else 1.0
    # Allow the first few frames before hysteresis engages (engage takes 1 tick)
    # Check that scale is reduced for the bulk of the high-error window
    throttled_frames = sum(1 for s in scale_when_high if s < 1.0)
    if throttled_frames < PHASE_A_FRAMES - 2:
        fails.append(
            f"T14.A: GOVERNOR DID NOT THROTTLE -- only {throttled_frames}/{PHASE_A_FRAMES} "
            f"frames had ph_scale < 1.0 during {HIGH_ERROR_DEG} deg error injection "
            f"(engage threshold {PHERR_ENGAGE_DEG} deg)")
    # Also verify the floor is respected
    min_scale_during_high = min(scale_when_high) if scale_when_high else 1.0
    if min_scale_during_high < PHERR_FLOOR_SCALE - 0.001:
        fails.append(
            f"T14.A: FLOOR VIOLATED -- ph_scale reached {min_scale_during_high:.3f}, "
            f"below PHERR_FLOOR_SCALE={PHERR_FLOOR_SCALE}")

    # T14.B: stall count during governor-throttled run must stay low
    # Wet sand base load is ~230-370, well below STALL_THRESHOLD=750, so only
    # the rare rock spike stalls; governor throttle should not cause cascade.
    STALL_CASCADE_LIMIT = 2
    if stall_count > STALL_CASCADE_LIMIT:
        fails.append(
            f"T14.B: STALL CASCADE -- {stall_count} stall events during governor-throttled "
            f"wet sand run (limit {STALL_CASCADE_LIMIT}); governor may be over-throttling")

    # T14.C: governor must release after error drops below PHERR_RELEASE_DEG
    # After the transition at frame PHASE_A_FRAMES, governor should disengage
    # within 1-2 ticks (hysteresis is single-tick). Allow 3 frames grace.
    RELEASE_GRACE_FRAMES = 3
    still_active_late = [gov_active_when_low[i]
                         for i in range(RELEASE_GRACE_FRAMES, len(gov_active_when_low))]
    if any(still_active_late):
        active_count = sum(1 for v in still_active_late if v)
        fails.append(
            f"T14.C: GOVERNOR NOT RELEASED -- active for {active_count} frames after "
            f"error dropped to {LOW_ERROR_DEG} deg (release threshold {PHERR_RELEASE_DEG} deg)")

    return {
        'name':                  'T14 PhErr governor terrain load',
        'passed':                len(fails) == 0,
        'fails':                 fails,
        'min_ph_scale_high_err': min(scale_when_high) if scale_when_high else 1.0,
        'max_ph_scale_high_err': max(scale_when_high) if scale_when_high else 1.0,
        'stall_count':           stall_count,
        'governor_released':     not any(still_active_late) if 'still_active_late' in dir() else None,
    }


# ── Pass/fail criteria ────────────────────────────────────────────────────────
def evaluate(r):
    name = r['name']
    fails = []

    if not r['gov_ok']:
        fails.append(f"GOVERNOR EXCEEDED (headroom={r['gov_headroom_hz']:.4f} Hz)")

    if r['max_speed_sts'] > 3000:
        fails.append(f"SPEED LIMIT (max={r['max_speed_sts']})")

    if name.startswith("T1"):
        if r['total_stalls'] > 0:
            fails.append(f"FALSE STALL on sand only (count={r['total_stalls']})")

    elif name.startswith("T2"):
        if r['max_stall_dur'] > 10:
            fails.append(f"STALL TOO LONG (max_dur={r['max_stall_dur']} frames)")
        if r['total_stalls'] > 40:
            fails.append(f"TOO MANY STALLS (count={r['total_stalls']})")
        if r['max_exit_snap'] > 2000:
            fails.append(f"EXIT SNAP too high (exit_snap={r['max_exit_snap']} STS, limit 2000)")

    elif name.startswith("T3"):
        if r['total_stalls'] == 0:
            fails.append("STALL NOT DETECTED - fixed rock obstruction missed")
        if r['max_exit_snap'] > 2000:
            fails.append(f"EXIT SNAP on stall recovery (exit_snap={r['max_exit_snap']} STS, limit 2000)")

    elif name.startswith("T4"):
        if r['max_exit_snap'] > 2000:
            fails.append(f"EXIT SNAP on hole recovery (exit_snap={r['max_exit_snap']} STS, limit 2000)")

    elif name.startswith("T5") or name.startswith("T6"):
        if r['total_stalls'] > 15:
            fails.append(f"EXCESSIVE STALLS on ramp (count={r['total_stalls']})")
        if r['max_exit_snap'] > 2000:
            fails.append(f"EXIT SNAP on ramp stall recovery (exit_snap={r['max_exit_snap']} STS)")

    elif name.startswith("T7"):
        if r['total_stalls'] > 30:
            fails.append(f"EXCESSIVE STALLS on steep ramp (count={r['total_stalls']})")

    elif name.startswith("T8"):
        if r['stall_fraction'] > 0.20:
            fails.append(f"STALL FRACTION too high ({r['stall_fraction']*100:.1f}% stalled, limit 20%)")
        if r['max_stall_dur'] > 15:
            fails.append(f"STALL TOO LONG (max_dur={r['max_stall_dur']} frames)")
        if r['max_exit_snap'] > 2000:
            fails.append(f"EXIT SNAP in worst-case (exit_snap={r['max_exit_snap']} STS)")

    elif name.startswith("T9"):
        # T9 uses intentionally high speed (800) to generate load spikes for hysteresis testing.
        # Governor headroom is expected to be negative -- exempt from gov_ok check.
        fails = [f for f in fails if "GOVERNOR" not in f]
        if r['total_stalls'] > 0:
            fails.append(f"HYSTERESIS FAIL: 2-frame spikes caused stall (count={r['total_stalls']})")

    elif name.startswith("T10"):
        if r['gov_headroom_hz'] < 0.001:
            fails.append(f"THIN GOVERNOR MARGIN ({r['gov_headroom_hz']:.5f} Hz)")
        if r['total_stalls'] > 5:
            fails.append(f"STALLS DURING CARVE (count={r['total_stalls']})")

    elif name.startswith("T11"):
        if r['max_exit_snap'] > 2000:
            fails.append(f"EXIT SNAP ({r['max_exit_snap']:.0f} STS, limit 2000)")
        if r.get('max_ff_jump', 0) > 300:
            fails.append(f"FF DISCONTINUITY ({r.get('max_ff_jump', 0):.1f} deg/s, limit 300)")
        # LERP convergence: analytical (duty 0.67->0.60, lr=0.08)
        lr = min(1.0, 4.0 * real_dt)
        delta = abs(0.60 - 0.67)
        thresh = 0.01 * 0.60
        lerp_frames = int(math.ceil(math.log(thresh / delta) / math.log(1.0 - lr)))
        if lerp_frames > 150:
            fails.append(f"LERP TOO SLOW ({lerp_frames} frames, limit 150)")

    elif name.startswith("T12"):
        if r['stall_fraction'] > 0.10:
            fails.append(f"STALL FRACTION too high ({r['stall_fraction']*100:.1f}%, limit 10%)")
        if r['max_stall_dur'] > 15:
            fails.append(f"STALL TOO LONG (max_dur={r['max_stall_dur']} frames, limit 15)")
        if r['max_exit_snap'] > 2000:
            fails.append(f"EXIT SNAP ({r['max_exit_snap']:.0f} STS, limit 2000)")
        if r.get('max_ff_jump', 0) > 300:
            fails.append(f"FF DISCONTINUITY ({r.get('max_ff_jump', 0):.1f} deg/s, limit 300)")
        # LERP convergence: analytical (duty 0.67->0.60, lr=0.08)
        lr = min(1.0, 4.0 * real_dt)
        delta = abs(0.60 - 0.67)
        thresh = 0.01 * 0.60
        lerp_frames = int(math.ceil(math.log(thresh / delta) / math.log(1.0 - lr)))
        if lerp_frames > 150:
            fails.append(f"LERP TOO SLOW ({lerp_frames} frames, limit 150)")

    return fails


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    print()
    print("=" * 65)
    print("   TERRAIN ROBUSTNESS SIMULATION  --  Alamosa CO, April 11")
    print("   NASA Colorado Space Grant | final_full_gait_test.py")
    print("=" * 65)
    print(f"  Gait engine: VELOCITY_SCALAR={VELOCITY_SCALAR}, "
          f"STALL_THRESHOLD={STALL_THRESHOLD}")
    print(f"  Stall hysteresis: 3-frame set, 3-frame clear (no INSTANT_STALL)")
    print(f"  Air phase: LINEAR | Feedforward: constant-per-phase")
    print(f"  Governor: no pi/2 factor | LERP: 4x rate | KP_PHASE: {KP_PHASE}")
    print()

    all_results  = []
    overall_pass = True

    scenarios = define_scenarios()
    for sc_kwargs in scenarios:
        r = run_scenario(**sc_kwargs)
        all_results.append(r)

    # Print per-scenario results
    print(f"{'Scenario':<35} {'Stalls':>6} {'MaxDur':>7} {'GovHd Hz':>10} {'ExSnap':>7} {'Stall%':>7}  Result")
    print("-" * 87)
    for r in all_results:
        fails = evaluate(r)
        status = "PASS" if not fails else "FAIL"
        if fails: overall_pass = False
        gov_s  = f"{r['gov_headroom_hz']:+.4f}" if r['gov_headroom_hz'] != float('inf') else "  inf "
        esnap_s = f"{r['max_exit_snap']:.0f}" if r['max_exit_snap'] > 0 else "   0"
        sfrac_s = f"{r.get('stall_fraction',0)*100:.1f}%"
        print(f"  {r['name']:<33} {r['total_stalls']:>6} {r['max_stall_dur']:>7} "
              f"{gov_s:>10} {esnap_s:>7} {sfrac_s:>7}  {status}")
        for f in fails:
            print(f"    !! {f}")

    # T13 and T14 are standalone (not run_scenario-based); run and print separately
    t13 = run_t13_overload_timing()
    t14 = run_t14_pherr_governor()
    for tr in (t13, t14):
        status = "PASS" if tr['passed'] else "FAIL"
        if not tr['passed']: overall_pass = False
        print(f"  {tr['name']:<33} {'--':>6} {'--':>7} {'--':>10} {'--':>7} {'--':>7}  {status}")
        for f in tr['fails']:
            print(f"    !! {f}")

    print()
    print("=" * 65)
    print("DETAILED TERRAIN ANALYSIS")
    print("=" * 65)

    # T1 analysis
    t1 = next(r for r in all_results if r['name'].startswith('T1'))
    print(f"\nT1  Wet sand drag:")
    print(f"    Expected load range: 230-370 (threshold {STALL_THRESHOLD})")
    print(f"    Stall events: {t1['total_stalls']} (expected 0)")
    print(f"    Max stall dur: {t1['max_stall_dur']} frames")
    print(f"    Result: {'Sand drag safe - no false stalls' if t1['total_stalls']==0 else 'UNEXPECTED STALLS - load model error'}")

    # T2 analysis
    t2 = next(r for r in all_results if r['name'].startswith('T2'))
    print(f"\nT2  Sand + loose rocks (0.5% per stance frame, 2-4 frame duration):")
    print(f"    Stall events: {t2['total_stalls']} | max_dur: {t2['max_stall_dur']} frames")
    print(f"    Stall by servo: " + " ".join(f"S{k}={v}" for k,v in sorted(t2['stall_by_servo'].items())))
    print(f"    Max exit snap:  {t2['max_exit_snap']:.0f} STS (limit 2000)")

    # T3 analysis
    t3 = next(r for r in all_results if r['name'].startswith('T3'))
    print(f"\nT3  Fixed rock obstruction (servo 1, 20 frames at tick 100):")
    print(f"    Stall detected: {'YES' if t3['total_stalls']>0 else 'NO - PROBLEM'}")
    print(f"    Stall dur: {t3['max_stall_dur']} frames (rock 20f + hysteresis exit)")
    print(f"    Max exit snap: {t3['max_exit_snap']:.0f} STS (limit 2000)")
    print(f"    Stall fraction: {t3['stall_fraction']*100:.1f}% of scenario")

    # T4 analysis
    t4 = next(r for r in all_results if r['name'].startswith('T4'))
    print(f"\nT4  Hole encounter (servo 2, edge/drop/wall at tick 50):")
    print(f"    Total stalls: {t4['total_stalls']} | max_dur: {t4['max_stall_dur']} frames")
    print(f"    Max exit snap:  {t4['max_exit_snap']:.0f} STS (limit 2000)")
    print(f"    Note: drop phase load ~40 (below threshold) - no stall on drop")

    # Ramp analysis
    for tn, t in [(5,10),(6,15),(7,20)]:
        tr = next(r for r in all_results if r['name'].startswith(f'T{tn}'))
        gname = "Wave" if tn < 7 else "Quad"
        print(f"\nT{tn}  Ramp {t} deg ({gname} gait):")
        print(f"    Governor headroom: {tr['gov_headroom_hz']:+.4f} Hz")
        print(f"    Stalls: {tr['total_stalls']} | max_dur: {tr['max_stall_dur']} frames")
        base = 150; sand = 150; ramp_extra = int((base+sand)*math.sin(math.radians(t))*0.6)
        est_load = base + sand + ramp_extra
        print(f"    Est. stance load: {est_load} (base 150 + sand 150 + ramp {ramp_extra})")
        margin = STALL_THRESHOLD - est_load
        print(f"    Margin to stall threshold: {margin} units {'OK' if margin>0 else 'EXCEEDED'}")

    # T8 worst-case
    t8 = next(r for r in all_results if r['name'].startswith('T8'))
    print(f"\nT8  Worst case (sand + rocks + 15 deg ramp, Wave):")
    print(f"    Stalls: {t8['total_stalls']} | max_dur: {t8['max_stall_dur']} frames")
    print(f"    Stall fraction: {t8['stall_fraction']*100:.1f}% of servo-frames")
    print(f"    Forward progress (uptime): {'YES (>80%)' if t8['stall_fraction']<0.20 else 'MARGINAL'}")
    print(f"    Governor headroom: {t8['gov_headroom_hz']:+.4f} Hz")

    # T9 hysteresis
    t9 = next(r for r in all_results if r['name'].startswith('T9'))
    print(f"\nT9  2-frame spike hysteresis (1000 frames, 2-on/2-off at {800} load):")
    print(f"    Stall events: {t9['total_stalls']} (expected 0)")
    print(f"    Counter trace: 0->1->2->1->0 repeating, never reaches 3")
    print(f"    Hysteresis fix: {'WORKING' if t9['total_stalls']==0 else 'BROKEN'}")

    # T10 carve
    t10 = next(r for r in all_results if r['name'].startswith('T10'))
    print(f"\nT10 Wave carve left on sand (turn_bias=-0.12):")
    print(f"    Governor headroom: {t10['gov_headroom_hz']:+.4f} Hz")
    print(f"    Stalls: {t10['total_stalls']} | max_dur: {t10['max_stall_dur']}")

    # T11 gait transition
    t11 = next((r for r in all_results if r['name'].startswith('T11')), None)
    if t11:
        lr = min(1.0, 4.0 * real_dt)
        delta = abs(0.60 - 0.67)
        thresh = 0.01 * 0.60
        lerp_frames = int(math.ceil(math.log(thresh / delta) / math.log(1.0 - lr)))
        print(f"\nT11 Gait transition under load (Quad@400 -> Wave@350 at frame 500):")
        print(f"    Governor headroom: {t11['gov_headroom_hz']:+.4f} Hz")
        print(f"    Stalls: {t11['total_stalls']} | max_dur: {t11['max_stall_dur']} frames")
        print(f"    Max exit snap: {t11['max_exit_snap']:.0f} STS (limit 2000)")
        print(f"    Max FF jump: {t11.get('max_ff_jump', 0):.1f} deg/s (limit 300)")
        print(f"    LERP convergence: {lerp_frames} frames (duty 0.67->0.60, limit 150)")

    # T12 timed fallback
    t12 = next((r for r in all_results if r['name'].startswith('T12')), None)
    if t12:
        print(f"\nT12 Timed fallback on wet sand (Quad@400 45s -> Wave@350 30s -> decel):")
        print(f"    Governor headroom: {t12['gov_headroom_hz']:+.4f} Hz")
        print(f"    Stalls: {t12['total_stalls']} | max_dur: {t12['max_stall_dur']} frames")
        print(f"    Stall fraction: {t12['stall_fraction']*100:.1f}% (limit 10%)")
        print(f"    Max exit snap: {t12['max_exit_snap']:.0f} STS (limit 2000)")
        print(f"    Max FF jump: {t12.get('max_ff_jump', 0):.1f} deg/s (limit 300)")

    # T13 overload prevention
    print(f"\nT13 Overload prevention timing (servo 1 sustained stall):")
    print(f"    OVERLOAD_PREVENTION_TIME: {OVERLOAD_PREVENTION_TIME}s | "
          f"OVERLOAD_MAX_CYCLES: {OVERLOAD_MAX_CYCLES}")
    print(f"    Cycles fired (12 reps, 2.0s stall each): "
          f"{t13['overload_cycles_fired']} (cap={t13['overload_cap']})")
    print(f"    T13.A (2.0s > 1.5s threshold): "
          f"{'PASS' if t13['stall_duration_s'] > OVERLOAD_PREVENTION_TIME else 'FAIL'}")
    print(f"    T13.B (cycle fires after 2.0s stall): "
          f"{'PASS' if t13['overload_cycles_fired'] > 0 else 'FAIL'}")
    print(f"    T13.C (cap at {OVERLOAD_MAX_CYCLES} despite 12 reps): "
          f"{'PASS' if t13['overload_cycles_fired'] <= OVERLOAD_MAX_CYCLES else 'FAIL'}")
    t13_d_pass = not any('T13.D' in f for f in t13['fails'])
    print(f"    T13.D (1.0s stall does NOT fire cycle): "
          f"{'PASS' if t13_d_pass else 'FAIL'}")
    print(f"    Result: {'PASS' if t13['passed'] else 'FAIL -- ' + '; '.join(t13['fails'])}")

    # T14 PhErr governor
    print(f"\nT14 PhErr governor under terrain load (Wave@350, wet sand):")
    print(f"    PHERR_ENGAGE_DEG={PHERR_ENGAGE_DEG} | PHERR_RELEASE_DEG={PHERR_RELEASE_DEG} | "
          f"PHERR_FLOOR_SCALE={PHERR_FLOOR_SCALE}")
    print(f"    Min ph_scale during {45.0} deg error: "
          f"{t14['min_ph_scale_high_err']:.3f} (floor={PHERR_FLOOR_SCALE})")
    print(f"    Max ph_scale during {45.0} deg error: "
          f"{t14['max_ph_scale_high_err']:.3f} (expected <1.0 after engage)")
    print(f"    Stall count during governor-throttled run: {t14['stall_count']} (limit 2)")
    print(f"    Governor released after error drop: "
          f"{'YES' if t14['governor_released'] else 'NO'}")
    print(f"    Result: {'PASS' if t14['passed'] else 'FAIL -- ' + '; '.join(t14['fails'])}")

    # Clearance note
    print()
    print("=" * 65)
    print("COMPETITION TERRAIN CLEARANCE NOTES")
    print("=" * 65)
    chassis_bottom_mm = 74.058 - 45.0
    print(f"  Body ground clearance (calculated from CAD):")
    print(f"    Leg outer reach at 0 splay: 122.058 mm")
    print(f"    Shaft center -> chassis bottom: 45 mm")
    print(f"    Ground clearance (shaft to ground): 122.058 - 48.0 = 74.058 mm")
    print(f"    Chassis bottom from ground: 74.058 - 45.0 = {chassis_bottom_mm:.1f} mm")
    print(f"    Rocks < {chassis_bottom_mm-3:.0f} mm: pass under body safely")
    print(f"    Rocks {chassis_bottom_mm-3:.0f}-{chassis_bottom_mm+20:.0f} mm: contact legs only (not body)")
    print(f"    Rocks > {chassis_bottom_mm+20:.0f} mm: potential chassis contact")
    print()
    print(f"  Course: ~15-20 ft (4.6-6.1 m) | Battery: 3000 mAh")
    print(f"  Wave gait speed=350: ~45-60 s to complete course (estimated)")
    print(f"  Tripod gait speed=1200: ~30-40 s to complete course (estimated)")
    print()

    print("=" * 65)
    if overall_pass:
        print("TERRAIN SIMULATION COMPLETE - all scenarios passed.")
        print("Gait engine cleared for Alamosa terrain deployment.")
    else:
        # For run_scenario results
        failing = [r['name'] for r in all_results if evaluate(r)]
        # For standalone results
        for tr in (t13, t14):
            if not tr['passed']:
                failing.append(tr['name'])
        print(f"TERRAIN SIMULATION FAILED - {len(failing)} scenario(s) failed:")
        for n in failing:
            print(f"  - {n}")
        print("Resolve failures before hardware deployment.")
    print("=" * 65)


if __name__ == "__main__":
    main()
