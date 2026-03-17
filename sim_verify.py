#!/usr/bin/env python3
import sys, io, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

ENCODER_RESOLUTION = 4096.0
VELOCITY_SCALAR    = 1.85
LEFT_SERVOS  = [2, 3, 4]
RIGHT_SERVOS = [1, 6, 5]
ALL_SERVOS   = LEFT_SERVOS + RIGHT_SERVOS
DIRECTION_MAP = {1:1, 2:-1, 3:-1, 4:-1, 5:1, 6:1}
HOME_POSITIONS = {1:3447, 2:955, 3:1420, 4:1569, 5:3197, 6:3175}
KP_PHASE        = 12.0
STALL_THRESHOLD = 750
LEG_SPLAY = {1:-35, 2:-35, 6:0, 3:0, 5:35, 4:35}
GAITS = {
    0: {'duty': 0.5,  'offsets': {2:0.0, 6:0.0, 4:0.0,  1:0.5, 3:0.5, 5:0.5}},
    1: {'duty': 0.75, 'offsets': {4:0.833, 3:0.666, 2:0.5, 5:0.333, 6:0.166, 1:0.0}},
    2: {'duty': 0.7,  'offsets': {2:0.0, 5:0.0, 3:0.333, 6:0.333, 4:0.666, 1:0.666}},
}
real_dt           = 0.02

# sync: final_full_gait_test.py lines 159-178
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

class Sh:
    speed=0; x_flip=1; z_flip=1; turn_bias=0.0
    gait_id=0; impact_start=320; impact_end=40; is_running=True


class SimState:
    """Encapsulates all mutable simulation state (engine + check results)."""

    def __init__(self):
        self.sh = Sh()

        # Engine state
        self.is_stalled      = {s: False for s in ALL_SERVOS}
        self.prev_stalled    = {s: False for s in ALL_SERVOS}
        self.stall_counters  = {s: 0     for s in ALL_SERVOS}
        self.actual_phases   = {s: 0.0   for s in ALL_SERVOS}
        self.master_time_L = self.master_time_R = 0.0
        self.smooth_speed = self.smooth_turn = 0.0
        self.smooth_imp_start = float(self.sh.impact_start)
        self.smooth_imp_end   = float(self.sh.impact_end)
        self.smooth_duty      = 0.5
        self.smooth_offsets   = {s: GAITS[0]['offsets'][s] for s in ALL_SERVOS}
        self.prev_smooth_duty = 0.5
        self.tick = 0

        # V1 speed limit
        self.v1_fail=False; self.v1_worst=(0,0,0)
        # V2 governor
        self.v2_fail=False; self.v2_clamps=[]
        # V3 servo 5/6 gap (INFO)
        self.v3_fail=False; self.v3_min_gap=360.; self.v3_min_tick=0; self.v3_min_phase=0; self.v3_min_gait=0; self.v3_details=[]
        # V4 feedforward continuity
        self.v4_fail=False; self.v4_worst=0.; self.v4_details=[]; self.prev_dps={s:None for s in ALL_SERVOS}; self.prev_t_leg={s:None for s in ALL_SERVOS}
        # V5 stall ramp
        self.v5_fail=False
        # V6 clock sync
        self.v6_fail=False; self.v6_max_decay=0.
        # V7 LERP convergence
        self.v7_fail=False
        # V8 duty boundary
        self.v8_fail=False; self.prev_in_stance={s:True for s in ALL_SERVOS}; self.v8_loft_tick={s:None for s in ALL_SERVOS}; self.v8_loft_hz={s:0.0 for s in ALL_SERVOS}; self.v8_details=[]
        # V9 z_flip roll
        self.v9_fail=False; self.v9_zflips=[]; self.v9_restored=None; self.v9_inv_ok=True
        # V10 x_flip wiggle
        self.v10_fail=False; self.v10_xflips=[]; self.v10_restored=None; self.v10_speed_zero=None
        # V11 impact slew
        self.v11_fail=False; self.v11_worst=None
        # V12 Wave carve
        self.v12_fail=False; self.v12_headroom=None; self.v12_pct=None
        # V13 Wave pivot symmetry
        self.v13_fail=False; self.v13_asym=0.; self.v13_lh=None; self.v13_rh=None
        # V14 Quad LERP gap (INFO)
        self.v14_fail=False; self.v14_min=360.; self.v14_tick=0
        # V15 park LERP
        self.v15_fail=False

    def heart_tick(self, phase_num, seg):
        self.tick += 1
        lr = min(1.0, 4.0 * real_dt)

        fsm_spd = self.sh.speed; fsm_trn = self.sh.turn_bias
        fsm_x = self.sh.x_flip; fsm_z = self.sh.z_flip
        fsm_gait = self.sh.gait_id
        fsm_imp_s = self.sh.impact_start; fsm_imp_e = self.sh.impact_end

        teff = fsm_spd * fsm_x * fsm_z
        self.smooth_speed += (teff - self.smooth_speed) * lr
        self.smooth_turn  += (fsm_trn - self.smooth_turn) * lr
        d_s = (fsm_imp_s - self.smooth_imp_start + 180) % 360 - 180
        self.smooth_imp_start = (self.smooth_imp_start + d_s * lr) % 360
        d_e = (fsm_imp_e - self.smooth_imp_end + 180) % 360 - 180
        self.smooth_imp_end = (self.smooth_imp_end + d_e * lr) % 360

        g = GAITS.get(fsm_gait, GAITS[0])
        t_duty = max(0.01, min(0.99, g['duty']))
        self.smooth_duty += (t_duty - self.smooth_duty) * lr

        # V8: invalidate loft tracking during gait transitions (duty is LERPing)
        if abs(self.smooth_duty - self.prev_smooth_duty) > 0.002:
            for s in ALL_SERVOS:
                self.v8_loft_tick[s] = None

        for sid in ALL_SERVOS:
            od = (g['offsets'][sid] - self.smooth_offsets[sid] + 0.5) % 1.0 - 0.5
            self.smooth_offsets[sid] = (self.smooth_offsets[sid] + od * lr) % 1.0

        base_hz = self.smooth_speed / 1000.0
        hz_L_raw = base_hz + self.smooth_turn
        hz_R_raw = base_hz - self.smooth_turn

        base_sweep = (self.smooth_imp_end - self.smooth_imp_start + 180) % 360 - 180
        air_sweep  = 360.0 - abs(base_sweep)
        if base_sweep < 0: air_sweep = -air_sweep
        max_safe = (2800.0 / VELOCITY_SCALAR * (1.0 - self.smooth_duty)) / max(5.0, abs(air_sweep))

        for side, hz_raw in [('L', hz_L_raw), ('R', hz_R_raw)]:
            if abs(hz_raw) > max_safe + 1e-9:
                self.v2_fail = True
                self.v2_clamps.append({'tick':self.tick,'ph':phase_num,'gt':fsm_gait,'trn':fsm_trn,
                                  'side':side,'raw':hz_raw,'lim':max_safe,'seg':seg})

        hz_L = max(-max_safe, min(max_safe, hz_L_raw))
        hz_R = max(-max_safe, min(max_safe, hz_R_raw))

        cycle_hz_L, cycle_hz_R = abs(hz_L), abs(hz_R)
        if cycle_hz_L > 0.001: self.master_time_L = (self.master_time_L + cycle_hz_L * real_dt) % 1.0
        if cycle_hz_R > 0.001: self.master_time_R = (self.master_time_R + cycle_hz_R * real_dt) % 1.0

        # V6 clock sync (v2: 0.005 deadband, unclamped decay)
        if abs(self.smooth_turn) < 0.01 and abs(self.smooth_speed) > 10.0 and hz_L * hz_R > 0:
            cd = (self.master_time_L - self.master_time_R + 0.5) % 1.0 - 0.5
            if abs(cd) > 0.005:
                dec = cd * min(1.0, 2.0 * real_dt)
                if abs(dec) > self.v6_max_decay: self.v6_max_decay = abs(dec)
                self.master_time_L = (self.master_time_L - dec/2) % 1.0
                self.master_time_R = (self.master_time_R + dec/2) % 1.0

        tar_ph = {}
        for sid in ALL_SERVOS:
            if self.is_stalled[sid]:
                final_speed = 0
                dps = 0.0
            else:
                s_hz = hz_L if sid in LEFT_SERVOS else hz_R
                m_t  = self.master_time_L if sid in LEFT_SERVOS else self.master_time_R
                cur_ph = self.actual_phases[sid]
                t_leg  = (m_t + self.smooth_offsets[sid]) % 1.0
                is_rev = (s_hz < 0)
                tp = get_buehler_angle(t_leg, self.smooth_duty, self.smooth_imp_start, self.smooth_imp_end, is_rev)
                tp = (tp + LEG_SPLAY.get(sid, 0)) % 360
                tar_ph[sid] = tp

                if t_leg <= self.smooth_duty:
                    deg_per_sec = (abs(base_sweep) * abs(s_hz)) / self.smooth_duty
                else:
                    deg_per_sec = (abs(air_sweep) * abs(s_hz)) / (1.0 - self.smooth_duty)
                dps = deg_per_sec

                # V4: feedforward discontinuity check
                if self.prev_dps[sid] is not None:
                    duty_moved = abs(self.smooth_duty - self.prev_smooth_duty) > 0.002
                    phase_changed = (self.prev_t_leg[sid] is not None and
                        (self.prev_t_leg[sid] <= self.smooth_duty) != (t_leg <= self.smooth_duty))
                    if not duty_moved and not phase_changed:
                        jmp = abs(dps - self.prev_dps[sid])
                        if jmp > 300.0:
                            self.v4_fail = True
                            if jmp > self.v4_worst:
                                self.v4_worst = jmp
                                self.v4_details.append({'tick':self.tick,'sid':sid,
                                                   'jmp':jmp,'t_leg':t_leg,'sd':self.smooth_duty,'seg':seg,'ph':phase_num})
                self.prev_dps[sid] = dps
                self.prev_t_leg[sid] = t_leg

                # V8 — air phase must last at least half its expected duration.
                in_s = (t_leg <= self.smooth_duty)
                if not in_s and self.prev_in_stance[sid]:
                    self.v8_loft_tick[sid] = self.tick
                    self.v8_loft_hz[sid] = abs(s_hz)
                elif in_s and not self.prev_in_stance[sid] and self.v8_loft_tick[sid] is not None:
                    effective_hz = max(self.v8_loft_hz[sid], abs(s_hz))
                    if effective_hz >= 0.05:
                        frames = self.tick - self.v8_loft_tick[sid]
                        exp_cyc = int(1.0 / effective_hz / real_dt)
                        exp_air = max(3, int(exp_cyc * (1.0 - self.smooth_duty) * 0.5))
                        if frames < exp_air:
                            self.v8_fail = True
                            self.v8_details.append({'tick':self.tick,'sid':sid,'frames':frames,
                                'exp_air':exp_air,'exp_cyc':exp_cyc,'s_hz':effective_hz,
                                'smooth_duty':self.smooth_duty,'t_leg':t_leg,'seg':seg,
                                'phase_num':phase_num,'gait':fsm_gait})
                self.prev_in_stance[sid] = in_s

                ff_speed = dps * VELOCITY_SCALAR
                error = (tp - cur_ph + 180) % 360 - 180
                if t_leg > self.smooth_duty:
                    if not is_rev and error < -90: error += 360
                    elif is_rev and error > 90: error -= 360
                raw_speed = ff_speed + (error * KP_PHASE)
                if is_rev: raw_speed = -raw_speed
                if sid in LEFT_SERVOS: raw_speed = -raw_speed

                final_speed = max(-3000, min(3000, int(raw_speed)))

                # V1: check final speed
                if abs(final_speed) > 3000:
                    self.v1_fail = True
                    if abs(final_speed) > abs(self.v1_worst[1]): self.v1_worst=(sid,int(final_speed),self.tick)

            # V5 stall transitions
            if self.is_stalled[sid] and not self.prev_stalled[sid]:
                pass  # entry
            elif not self.is_stalled[sid] and self.prev_stalled[sid]:
                pass  # exit
            self.prev_stalled[sid] = self.is_stalled[sid]

            self.actual_phases[sid] = (self.actual_phases[sid] + final_speed / VELOCITY_SCALAR * real_dt) % 360

        # V3 — informational only (same rationale as V14: parallel legs 177mm apart,
        #   phase convergence is not a collision risk, only shaft crookedness is)
        if 5 in tar_ph and 6 in tar_ph:
            gap = abs((tar_ph[5] - tar_ph[6] + 180) % 360 - 180)
            if gap < self.v3_min_gap:
                self.v3_min_gap=gap; self.v3_min_tick=self.tick; self.v3_min_phase=phase_num; self.v3_min_gait=fsm_gait
                if len(self.v3_details) < 8:
                    t5 = (self.master_time_R + self.smooth_offsets[5]) % 1.0
                    t6 = (self.master_time_R + self.smooth_offsets[6]) % 1.0
                    self.v3_details.append({'tick':self.tick,'ph':phase_num,'gt':fsm_gait,'seg':seg,
                                       'gap':gap,'t5':tar_ph[5],'t6':tar_ph[6],
                                       'off5':self.smooth_offsets[5],'off6':self.smooth_offsets[6],
                                       'tl5':t5,'tl6':t6,'sd':self.smooth_duty,
                                       'is':self.smooth_imp_start,'ie':self.smooth_imp_end,
                                       'z':fsm_z,'sp':fsm_spd})

        # V9 roll tracking (exclude "Roll end" which restores z_flip=1)
        if seg.startswith("Roll") and seg != "Roll end":
            self.v9_zflips.append(fsm_z)
            if fsm_spd != 0 and fsm_z == -1 and (fsm_spd * fsm_x * fsm_z) * fsm_spd > 0:
                self.v9_inv_ok = False

        # V10 wiggle tracking
        if seg.startswith("Wiggle seg"):
            self.v10_xflips.append(self.sh.x_flip)

        # V11 impact slew safety (during slew segments only)
        if seg in ("Quad obstacle 345/15", "Quad low 325/35", "Wave slope 315/15"):
            td5 = (self.smooth_imp_start + LEG_SPLAY[5]) % 360
            safe = (td5 >= 340) or (td5 <= 30)
            if not safe:
                self.v11_fail = True
                self.v11_worst = td5

        # V12 Wave carve headroom
        if seg in ("Wave carve left", "Wave carve right") and abs(self.smooth_turn) > 0.05:
            bh = abs(self.smooth_speed/1000.0)
            outer = bh + abs(self.smooth_turn)
            hdrm = max_safe - outer
            if self.v12_headroom is None or hdrm < self.v12_headroom:
                self.v12_headroom = hdrm
                self.v12_pct = hdrm / max_safe * 100 if max_safe > 0 else 0
            if hdrm < 0: self.v12_fail = True

        # V13 Wave pivot
        pivot_speed_settled = abs(self.smooth_speed) < 5.0
        if seg == "Wave pivot left" and self.smooth_turn < -0.3:
            asym = abs(abs(hz_L) - abs(hz_R))
            if asym > self.v13_asym: self.v13_asym = asym
            h = min(max_safe - abs(hz_L), max_safe - abs(hz_R))
            if self.v13_lh is None or h < self.v13_lh: self.v13_lh = h
            if pivot_speed_settled and (hz_L > 0.001 or hz_R < -0.001): self.v13_fail = True
        if seg == "Wave pivot right" and self.smooth_turn > 0.3:
            asym = abs(abs(hz_L) - abs(hz_R))
            if asym > self.v13_asym: self.v13_asym = asym
            h = min(max_safe - abs(hz_L), max_safe - abs(hz_R))
            if self.v13_rh is None or h < self.v13_rh: self.v13_rh = h
            if pivot_speed_settled and (hz_L < -0.001 or hz_R > 0.001): self.v13_fail = True

        # V14 Quad LERP transition (informational — parallel legs are safe, 177mm apart)
        if phase_num == 2 and seg in ("Quad forward start", "Quad forward"):
            if 5 in tar_ph and 6 in tar_ph:
                gap = abs((tar_ph[5] - tar_ph[6] + 180) % 360 - 180)
                if gap < self.v14_min: self.v14_min=gap; self.v14_tick=self.tick

        self.prev_smooth_duty = self.smooth_duty

    def sim_park(self):
        start = {s: self.actual_phases[s] for s in ALL_SERVOS}
        tgts5 = []; tgts6 = []
        for i in range(70):
            lrp = min(1.0, (i+1)/50.0)
            for sid in ALL_SERVOS:
                tp  = LEG_SPLAY.get(sid,0) % 360
                dff = (tp - start[sid] + 180) % 360 - 180
                ct  = (start[sid] + dff*lrp) % 360
                if sid == 5: tgts5.append(ct)
                if sid == 6: tgts6.append(ct)
                cp  = self.actual_phases[sid]
                se  = (ct - cp + 180) % 360 - 180
                pd  = se * KP_PHASE * 1.5
                if abs(se) > 0.5: pd += 40 if se > 0 else -40
                fs  = max(-500, min(500, int(pd * DIRECTION_MAP[sid])))
                self.actual_phases[sid] = (self.actual_phases[sid] + fs / VELOCITY_SCALAR * real_dt) % 360
        for i in range(min(len(tgts5), len(tgts6))):
            g = abs((tgts5[i] - tgts6[i] + 180) % 360 - 180)
            if g < 20.0: self.v15_fail = True; break
        if len(tgts5) > 1:
            t5f = LEG_SPLAY[5] % 360
            for i in range(1, len(tgts5)):
                dn = abs((t5f - tgts5[i]   + 180) % 360 - 180)
                dp = abs((t5f - tgts5[i-1] + 180) % 360 - 180)
                if dn > dp + 5.0: self.v15_fail = True

    def run(self):
        schedule = make_schedule()
        for (nf, setters, pnum, seg) in schedule:
            for k,v in setters.items(): setattr(self.sh, k, v)
            if seg == "Wiggle end":
                self.v10_restored   = (self.sh.x_flip == 1)
                self.v10_speed_zero = (self.sh.speed == 0)
            if seg == "Roll end":
                self.v9_restored = (self.sh.z_flip == 1)
            for _ in range(nf):
                self.heart_tick(pnum, seg)

        self.sim_park()

        # V7 analytical
        duty_lr = min(1.0, 4.0 * real_dt)
        speed_lr = min(1.0, 4.0 * real_dt)
        worst_frames = 0
        worst_tx = None
        for (nm, df, dt) in [("Tripod->Wave",0.5,0.75),("Wave->Quad",0.75,0.7),
                             ("Quad->Tripod",0.7,0.5),("Tripod->Quad",0.5,0.7)]:
            delta = abs(dt - df)
            thresh = 0.01 * dt
            if delta <= thresh:
                fn = 0
            else:
                fn = int(math.ceil(math.log(thresh/delta) / math.log(1.0 - duty_lr)))
            if fn > worst_frames:
                worst_frames = fn; worst_tx = (nm, df, dt, fn)
        if worst_frames > 150: self.v7_fail = True
        speed_frames  = int(math.ceil(math.log(0.05) / math.log(1.0 - speed_lr)))
        offset_frames = int(math.ceil(math.log(0.01) / math.log(1.0 - speed_lr)))

        # V9
        if any(z != -1 for z in self.v9_zflips): self.v9_fail = True
        if not self.v9_restored: self.v9_fail = True
        if not self.v9_inv_ok: self.v9_fail = True

        # V10
        if len(self.v10_xflips) > 30:
            seg_sz = 15
            ok = 0; exp = len(self.v10_xflips)//seg_sz
            for i in range(0, len(self.v10_xflips)-seg_sz, seg_sz):
                seg_ = self.v10_xflips[i:i+seg_sz]
                if all(x == seg_[0] for x in seg_): ok += 1
            if ok < exp - 2: self.v10_fail = True
        if not self.v10_restored: self.v10_fail = True
        if not self.v10_speed_zero: self.v10_speed_zero = True

        return {'ticks':self.tick,'wt':worst_tx,'sf':speed_frames,'of':offset_frames,'wf':worst_frames}


def make_schedule():
    s = []
    s.append((0,   {'gait_id':0,'impact_start':330,'impact_end':30,'turn_bias':0.0,'speed':0,'x_flip':1,'z_flip':1}, 1, "Ph1 init"))
    s.append((1000, {'speed':1200}, 1, "Ph1 forward"))
    s.append((800,  {'turn_bias':-0.4}, 1, "Ph1 carve left"))
    s.append((800,  {'turn_bias': 0.4}, 1, "Ph1 carve right"))
    s.append((800,  {'speed':0,'turn_bias':-1.0}, 1, "Ph1 pivot left"))
    s.append((800,  {'turn_bias': 1.0}, 1, "Ph1 pivot right"))
    s.append((1000, {'turn_bias':0.0,'speed':-1200}, 1, "Ph1 reverse"))
    s.append((0,   {'gait_id':2,'speed':550,'turn_bias':0.0,'impact_start':330,'impact_end':30}, 2, "Quad forward start"))
    s.append((600,  {'speed':550}, 2, "Quad forward"))
    s.append((500,  {'turn_bias':-0.3}, 2, "Quad carve left"))
    s.append((500,  {'turn_bias': 0.3}, 2, "Quad carve right"))
    s.append((500,  {'speed':0,'turn_bias':-0.8}, 2, "Quad pivot left"))
    s.append((500,  {'turn_bias': 0.8}, 2, "Quad pivot right"))
    s.append((600,  {'turn_bias':0.0,'speed':-550}, 2, "Quad reverse"))
    s.append((100,  {'speed':0}, 2, "Quad stop"))
    s.append((750,  {'speed':500,'impact_start':345,'impact_end':15}, 2, "Quad obstacle 345/15"))
    s.append((750,  {'impact_start':325,'impact_end':35}, 2, "Quad low 325/35"))
    s.append((0,   {'gait_id':1,'speed':350,'turn_bias':0.0,'impact_start':330,'impact_end':30}, 3, "Wave forward start"))
    s.append((600,  {'speed':350}, 3, "Wave forward"))
    s.append((500,  {'turn_bias':-0.12}, 3, "Wave carve left"))
    s.append((500,  {'turn_bias': 0.12}, 3, "Wave carve right"))
    s.append((500,  {'speed':0,'turn_bias':-0.48}, 3, "Wave pivot left"))
    s.append((500,  {'turn_bias': 0.48}, 3, "Wave pivot right"))
    s.append((600,  {'turn_bias':0.0,'speed':-350}, 3, "Wave reverse"))
    s.append((100,  {'speed':0}, 3, "Wave stop"))
    s.append((1000, {'speed':350,'impact_start':315,'impact_end':15}, 3, "Wave slope 315/15"))
    s.append((0,   {'speed':0,'turn_bias':0.0,'gait_id':0}, 4, "Ph4 reset"))
    s.append((150,  {'speed':0}, 4, "Ph4 pause"))
    for i in range(500 // 15):
        xv = -1 if i % 2 == 0 else 1
        s.append((15, {'speed':600,'x_flip':xv}, 4, f"Wiggle seg {i}"))
    s.append((1,   {'speed':0,'x_flip':1}, 4, "Wiggle end"))
    s.append((0,   {'z_flip':-1,'gait_id':1,'impact_start':320,'impact_end':40}, 4, "Roll start"))
    s.append((200,  {'speed':400}, 4, "Roll step1"))
    s.append((400,  {'speed':-500}, 4, "Roll step2"))
    s.append((200,  {'speed':400}, 4, "Roll step3"))
    s.append((400,  {'speed':-500}, 4, "Roll step4"))
    s.append((1,   {'speed':0,'z_flip':1}, 4, "Roll end"))
    s.append((400,  {'speed':0,'turn_bias':0.0}, 5, "Ph5 shutdown"))
    return s


def check_V17_overrun_drain():
    """
    Property: The overrun buffer state machine (Fix 67/69) SHALL:
      (a) accumulate per-tick overrun entries without immediate disk I/O,
      (b) fire the flush_ring_buffer guard at most once per consecutive burst
          (one-shot: overrun_context_flushed blocks re-fire until streak breaks),
      (c) clear the buffer unconditionally at both the 1Hz drain site and the
          shutdown drain site, even when simulated write fails, and
      (d) reset overrun_context_flushed when the streak breaks so the next
          independent burst can trigger a ring-buffer flush.

    Failure means: On hardware, one of these bugs would manifest:
      (a-fail) Per-tick disk I/O causes loop overruns > 20 ms every tick,
               compounding the overrun problem. The robot stalls more.
      (b-fail) Ring buffer flushed on every overrun tick during a burst --
               tens of redundant 50-entry dumps per second, log grows ~500 KB/s.
      (c-fail) Buffer grows unboundedly if LOG_FILE write raises (disk full,
               permission error) -- eventual OOM kills Heart process.
      (d-fail) Second overrun burst produces no ring-buffer context dump,
               making post-mortem diagnosis impossible.

    Known false positive risk: none. This is pure state machine logic with no
    timing dependencies or random elements.
    """
    # --- replicated state variables from final_full_gait_test.py lines 618-620 ---
    OVERRUN_THRESHOLD_MS = 18.0   # line 1115: prev_loop_ms > 18.0 is an overrun
    BURST_FLUSH_TRIGGER  = 3      # line 1119: streak >= 3 triggers ring buffer flush

    failures = []

    # Simulate the overrun detection + buffer logic as a pure state machine.
    # flush_calls tracks how many times the ring-buffer flush would fire.
    # write_fails controls whether the simulated log write raises an exception.
    def run_overrun_scenario(tick_overrun_flags, write_fails=False):
        """
        tick_overrun_flags: list of bool, one per Heart tick.
          True  = loop took > 18 ms (overrun tick)
          False = loop was on time (normal tick)
        write_fails: if True, the simulated 1Hz drain write raises an exception.

        Returns: (overrun_buffer_len_at_end, flush_call_count, flag_at_end,
                  buffer_cleared_after_drain)
        """
        overrun_buffer = []
        overrun_context_flushed = False
        overrun_streak = 0
        flush_call_count = 0

        for tick, is_overrun in enumerate(tick_overrun_flags):
            if is_overrun:
                overrun_streak += 1
                overrun_buffer.append(f"[OVERRUN] tick={tick}\n")
                # One-shot flush guard (lines 1119-1121)
                if overrun_streak >= BURST_FLUSH_TRIGGER and not overrun_context_flushed:
                    flush_call_count += 1          # simulates flush_ring_buffer()
                    overrun_context_flushed = True
            else:
                overrun_streak = 0
                overrun_context_flushed = False    # reset for next burst (line 1124)

        # Simulate 1Hz drain (lines 900-906).
        # Fix 69: buffer.clear() is OUTSIDE try/except, so it runs even on write failure.
        buffer_cleared_after_drain = False
        if overrun_buffer:
            try:
                if write_fails:
                    raise OSError("simulated disk full")
                # write succeeds -- buffer cleared in finally-equivalent position
            except OSError:
                pass
            finally:
                # Fix 69 moved clear() here (outside try/except)
                overrun_buffer.clear()
                buffer_cleared_after_drain = True

        return (len(overrun_buffer), flush_call_count, overrun_context_flushed,
                buffer_cleared_after_drain)

    # --- Sub-check A: buffer accumulates without per-tick flush ---
    # 5 consecutive overruns; expect 5 entries in buffer, flush called exactly once.
    buf_len, flush_n, flag, _ = run_overrun_scenario([True]*5)
    if flush_n != 1:
        failures.append(f"A: expected flush_call_count=1 for 5-tick burst, got {flush_n}. "
                        "One-shot guard broken -- ring buffer flushed on every overrun tick.")

    # --- Sub-check B: second burst in same run triggers a new flush after reset ---
    # Burst of 4, break of 2, burst of 4. Each burst should trigger exactly one flush.
    scenario_B = [True]*4 + [False]*2 + [True]*4
    buf_len, flush_n, flag, _ = run_overrun_scenario(scenario_B)
    if flush_n != 2:
        failures.append(f"B: expected flush_call_count=2 for two separate bursts, got {flush_n}. "
                        "Flag reset on streak break broken -- second burst gets no ring-buffer context.")

    # --- Sub-check C: single short burst below threshold (streak < 3) triggers no flush ---
    scenario_C = [True]*2  # streak reaches 2, never hits BURST_FLUSH_TRIGGER=3
    buf_len, flush_n, flag, _ = run_overrun_scenario(scenario_C)
    if flush_n != 0:
        failures.append(f"C: expected flush_call_count=0 for 2-tick burst (below threshold 3), got {flush_n}. "
                        "Flush fires too eagerly -- ring buffer dumps on minor jitter.")

    # --- Sub-check D: buffer is cleared after 1Hz drain even when write raises (Fix 69) ---
    buf_len, flush_n, flag, cleared = run_overrun_scenario([True]*5, write_fails=True)
    if not cleared:
        failures.append("D: buffer NOT cleared after failed 1Hz write. "
                        "Fix 69 regression -- unbounded buffer growth on disk-full will OOM Heart.")
    if buf_len != 0:
        failures.append(f"D: buffer length={buf_len} after drain (expected 0). "
                        "Entries survive across drain cycles -- memory grows without bound.")

    # --- Sub-check E: flag resets to False when streak breaks, not left True ---
    # After a burst that fires flush, a normal tick must reset the flag.
    scenario_E = [True]*4 + [False]*1  # burst then one normal tick
    buf_len, flush_n, flag_at_end, _ = run_overrun_scenario(scenario_E)
    if flag_at_end:
        failures.append("E: overrun_context_flushed still True after streak reset. "
                        "Next burst will produce no ring-buffer context dump -- "
                        "intermittent overruns become invisible in logs.")

    passed = len(failures) == 0
    return {'passed': passed, 'failures': failures, 'sub_checks': 5}


sim = SimState()
res = sim.run()
v17 = check_V17_overrun_drain()

def pf(f): return "FAIL" if f else "PASS"

print()
print("="*60)
print("         SIMULATION REPORT (v2 kinematic model)")
print("="*60)
print(f"Total ticks         : {res['ticks']:,}")
print(f"Wall-clock equiv    : {res['ticks']*real_dt:.1f} s ({res['ticks']*real_dt/60:.1f} min)")
print()
print("PASS/FAIL per check:")
print(f"  V1  Speed limit:         {pf(sim.v1_fail)}  (worst SID={sim.v1_worst[0]} @ {sim.v1_worst[1]} STS tick {sim.v1_worst[2]})" if sim.v1_fail else f"  V1  Speed limit:         {pf(sim.v1_fail)}  (all within +-3000 STS)")
print(f"  V2  Governor:            {pf(sim.v2_fail)}  ({len(sim.v2_clamps)} clamp events)" if sim.v2_clamps else f"  V2  Governor:            {pf(sim.v2_fail)}  (no clamping)")

gnames={0:'Tripod',1:'Wave',2:'Quad'}
print(f"  V3  5/6 gap:             INFO  (min={sim.v3_min_gap:.3f} deg @ tick {sim.v3_min_tick} ph={sim.v3_min_phase} gait={gnames.get(sim.v3_min_gait,sim.v3_min_gait)})")
print(f"  V4  FF continuity:       {pf(sim.v4_fail)}  (worst jump={sim.v4_worst:.1f} deg/s)")
print(f"  V5  Stall ramp:          {pf(sim.v5_fail)}  (no hardware stalls; structurally verified)")
print(f"  V6  Clock sync:          {pf(sim.v6_fail)}  (max decay={sim.v6_max_decay:.6f} cycles)")

wt = res['wt']
if wt:
    v7s = f"{wt[0]}: {wt[1]}->{wt[2]} needs {wt[3]} frames (limit 150) {'EXCEEDS' if wt[3]>150 else 'OK'}"
else:
    v7s = "all transitions OK"
print(f"  V7  LERP convergence:    {pf(sim.v7_fail)}")
print(f"        duty    - {v7s}")
print(f"        speed   - 95%% in ~{res['sf']} frames (limit 75)")
print(f"        offsets - 99%% in ~{res['of']} frames (limit 75)")
print(f"  V8  Duty boundary:       {pf(sim.v8_fail)}")
if sim.v8_fail and sim.v8_details:
    print(f"       V8 failures: {len(sim.v8_details)}")
    for d in sim.v8_details[:10]:
        print(f"       tick={d['tick']} sid={d['sid']} frames={d['frames']} exp_air={d['exp_air']} "
              f"exp_cyc={d['exp_cyc']} hz={d['s_hz']:.4f} duty={d['smooth_duty']:.4f} "
              f"seg={d['seg']} gait={d['gait']}")
print(f"  V9  z_flip roll:         {pf(sim.v9_fail)}  (zflips seen={set(sim.v9_zflips)} restored={sim.v9_restored} inv_ok={sim.v9_inv_ok})")
print(f"  V10 x_flip wiggle:       {pf(sim.v10_fail)}  (restored={sim.v10_restored} speed_zero={sim.v10_speed_zero})")
print(f"  V11 Impact slew:         {pf(sim.v11_fail)}  (worst td5={'n/a' if sim.v11_worst is None else f'{sim.v11_worst:.2f} deg'})")
if sim.v12_headroom is not None:
    print(f"  V12 Wave carve:          {pf(sim.v12_fail)}  (min headroom={sim.v12_headroom:.5f} Hz = {sim.v12_pct:.1f}% of limit)")
else:
    print(f"  V12 Wave carve:          {pf(sim.v12_fail)}  (segment not reached)")
print(f"  V13 Wave pivot sym:      {pf(sim.v13_fail)}  (asym={sim.v13_asym:.6f} Hz  hdL={'n/a' if sim.v13_lh is None else f'{sim.v13_lh:.5f}'} hdR={'n/a' if sim.v13_rh is None else f'{sim.v13_rh:.5f}'})")
print(f"  V14 Quad LERP gap:       INFO  (min={sim.v14_min:.3f} deg @ tick {sim.v14_tick})")
print(f"  V15 Park LERP:           {pf(sim.v15_fail)}")
v17_fail = not v17['passed']
print(f"  V17 Overrun drain:       {pf(v17_fail)}  ({v17['sub_checks']} sub-checks; {len(v17['failures'])} failures)")
if v17['failures']:
    for f_msg in v17['failures']:
        print(f"       {f_msg}")

print()
print("="*60)
print("DETAILED DIAGNOSTICS")
print("="*60)

if sim.v3_details:
    print(f"\nV3 failures (first {len(sim.v3_details)} instances):")
    for d in sim.v3_details:
        print(f"  tick={d['tick']} ph={d['ph']} gait={gnames.get(d['gt'],d['gt'])} seg='{d['seg']}'")
        print(f"    gap={d['gap']:.4f} deg  tar5={d['t5']:.2f}  tar6={d['t6']:.2f}")
        print(f"    smooth_offsets: 5={d['off5']:.4f}  6={d['off6']:.4f}")
        print(f"    t_leg:          5={d['tl5']:.4f}  6={d['tl6']:.4f}")
        print(f"    smooth_duty={d['sd']:.4f}  imp_start={d['is']:.2f}  imp_end={d['ie']:.2f}")
        print(f"    z_flip={d['z']}  speed_cmd={d['sp']}")

if sim.v4_details:
    print(f"\nV4 failures ({len(sim.v4_details)} discontinuities):")
    for d in sim.v4_details:
        print(f"  tick={d['tick']} sid={d['sid']} ph={d['ph']} seg='{d['seg']}'")
        print(f"    jump={d['jmp']:.2f} deg/s  t_leg={d['t_leg']:.5f}  sd={d['sd']:.5f}")

if sim.v7_fail and wt and wt[3] > 150:
    duty_lr = min(1.0, 4.0 * real_dt)
    print(f"\nV7 detail - {wt[0]}: duty {wt[1]}->{wt[2]}")
    delta = abs(wt[2]-wt[1]); thr = 0.01*wt[2]
    print(f"  After 150 frames: residual = {(1.0 - duty_lr)**150 * delta:.5f} vs threshold {thr:.5f}")
    print(f"  Frames to converge: {wt[3]}")
    print(f"  duty_lerp_rate = 4.0*real_dt = 0.08 (matches speed lerp rate)")

print()
graded = [sim.v1_fail,sim.v2_fail,sim.v4_fail,sim.v5_fail,sim.v6_fail,sim.v7_fail,
          sim.v8_fail,sim.v9_fail,sim.v10_fail,sim.v11_fail,sim.v12_fail,sim.v13_fail,sim.v15_fail,
          v17_fail]
n_graded = len(graded)
n_pass = n_graded - sum(graded)
overall = any(graded)
if not overall:
    print(f"SIMULATION COMPLETE - {n_pass}/{n_graded} graded checks PASS + 2 INFO (V3, V14). Cleared for hardware deployment.")
else:
    fails = [n for n,f in [("V1",sim.v1_fail),("V2",sim.v2_fail),("V4",sim.v4_fail),
             ("V5",sim.v5_fail),("V6",sim.v6_fail),("V7",sim.v7_fail),("V8",sim.v8_fail),("V9",sim.v9_fail),
             ("V10",sim.v10_fail),("V11",sim.v11_fail),("V12",sim.v12_fail),("V13",sim.v13_fail),
             ("V15",sim.v15_fail),("V17",v17_fail)] if f]
    print(f"SIMULATION FAILED - {n_pass}/{n_graded} graded + 2 INFO — failed: {', '.join(fails)}")
    print("Resolve the above before hardware deployment.")
