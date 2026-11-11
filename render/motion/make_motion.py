"""Build render/motion/motion.json: the physically driven motion of the GrappStar D50 base and the KEDGE dart, in REAL
TIME 1:1 with one hard cut (render/SPEC.md v2 direction).

    ../../kedge/.venv/bin/python make_motion.py [--rerun-sim] --node-path <dir with @dimforge/rapier3d-compat 0.14.0>

Base: the base's own joints, kin() and line paths (base_motion.BaseKin) driven by a real-time motor schedule
(base_motion.RealSchedule: wiper as a DC motor against the springs / trains, selector drum at the N20's 8.1 deg/s, the
firmware C1 order). The shot: release table, base/physics launcher.py trace, carriage stop flexure model. Flight:
dart_motion.Flight3D (kedge/flight34.py physics in 3D). Landing / bounce / righting / anchoring / tow: kedge/sim_righting
(sim_dump.mjs) STARTED FROM THE FLIGHT'S END STATE, launch inputs PICK from search_landing.py. Tether: rope.py on the base
line path, routed over the muzzle lip roller after the launch, line length conserved. Dart build: dart_motion.OUT
(KEDGE_OUT, else kedge/out35 once READY, else kedge/out34). Writes motion.json, checks.json and plots/*.png.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)

import base_motion as BM  # noqa: E402
import dart_motion as DM  # noqa: E402
import landing as LD  # noqa: E402
from rope import Rope  # noqa: E402

FPS = 30
# launch inputs picked by search_landing.py (seed 8, run 28; rechecked with the reachable rail pitch): plausible rail pitch,
# muzzle speed, cradle tip-off, yaw disturbance and a light cross-wind; its own bounce ends inverted, the leg alone rights
# it (no reel tug), both drills go in and it holds 55 N. Rerun the search for a new dart build (search_landing.py).
PICKS = {"out34": dict(rail_deg=33.987, v0=6.783, q0=-0.908, dth=0.16, r0=0.368, dpsi=3.18, wind_y=0.645, slope_deg=0.0,
                       source="search_landing.py seed 8 run 28 (search/search_results_recheck_search_results_s8.json)"),
         # v35 (pods hinged at x 50, folding aft, drills raked back): its own bounce ends inverted, the leg flips it, one pod
         # roll finishes it (no reel tug), both drills in, held 45 N in the sim
         "out35": dict(rail_deg=33.712, v0=6.888, q0=-0.818, dth=1.79, r0=-0.127, dpsi=1.14, wind_y=-0.319, slope_deg=0.0,
                       source="search_landing.py seed 11 run 118 (search/search_results_s11.json)")}
DRILL_RPM = 45.0                  # drill motor under load on 2S (100 rpm 12 V listing)
EA_LINE = 30000.0                 # N, 1 mm Dyneema braid (stretch under the tow)
T_FLIGHT, T_TIDY, T_TOW_MIN = 0.3, 5.0, 5.0     # N: line tension in flight (base physics 0.25 N drag, 0.37 N peak), FIRM take-up, tow floor
W_LINE = 0.006                    # N/m, 1 mm Dyneema braid (0.6 g/m)
W_DRAG_FLIGHT = 0.010             # N/m, air drag on the line sweeping sideways while the dart arcs down
SPOOL_DECEL = 0.25 / 0.035        # m/s2: LIGHT 0.25 N on the spool's effective line mass (I / r^2, base physics payout)
OVERRUN_M = 0.6                   # spool overrun after landing: base physics payout 0.20 m at 0.75 N brake, x 3 at LIGHT 0.25 N
V_OVERRUN = math.sqrt(2 * SPOOL_DECEL * OVERRUN_M)
R_SPOOL = 0.0274                  # m, storage spool pack radius (base physics payout)
TIDY_S = 3.0                      # s, firmware tidy by time (default 3000 ms)
LANDED_DELAY = 0.8                # s after first contact: the Jetson's landing verdict reaches the base
TOW_SIM = 6.5                     # s of the sim tow shown: 0 -> 15 N in 3 s, hold, step to 20 N, hold
HOLD_S = 4.0                      # s of tension holding on cam:B
K_FIXED, K_FREE = 12, 36          # line points per frame: base path (spool -> exit) and the free rope (exit -> knot)
K_WRAPS = 64
TOL_LOC, TOL_Q, TOL_DEG, TOL_PT = 1e-4, 1e-5, 0.02, 1e-3


def interp_rows(t, T, A):
    return np.array([np.interp(t, T, A[:, j]) for j in range(A.shape[1])])


def decimate(frames, vals, tol, always=()):
    """indices of keys so linear interpolation between kept keys stays within tol of every value (greedy, per channel max)"""
    vals = np.asarray(vals, float).reshape(len(frames), -1)
    n = len(frames)
    if n <= 2:
        return list(range(n))
    keep = [0]
    i = 0
    alw = set(always)
    while i < n - 1:
        j = i + 1
        best = j
        while j < n:
            if j - i >= 2:
                f0, f1 = frames[i], frames[j]
                u = (np.asarray(frames[i + 1:j]) - f0) / (f1 - f0)
                lin = vals[i] + u[:, None] * (vals[j] - vals[i])
                if np.abs(lin - vals[i + 1:j]).max() > tol:
                    break
                if any(k in alw for k in range(i + 1, j)):
                    break
            best = j
            j += 1
        keep.append(best)
        i = best
    return keep


def resample_poly(p, k, keep_idx=()):
    """k points along polyline p: the kept vertices plus evenly spaced arc-length samples"""
    p = np.asarray(p, float)
    if len(p) == 1:
        return np.repeat(p, k, axis=0)
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    fixed = sorted(set([0, len(p) - 1] + [i for i in keep_idx if 0 <= i < len(p)]))
    if len(fixed) >= k:
        sel = np.linspace(0, len(fixed) - 1, k).round().astype(int)
        return p[[fixed[i] for i in sel]]
    q = list(s[fixed])
    while len(q) < k:
        q.sort()
        gaps = np.diff(q)
        g = int(np.argmax(gaps))
        q.insert(g + 1, 0.5 * (q[g] + q[g + 1]))
    q = np.array(sorted(q))
    return np.stack([np.interp(q, s, p[:, j]) for j in range(3)], axis=1)


def dp_keep(p, k):
    """indices of the k most significant vertices (Douglas-Peucker order) of polyline p"""
    n = len(p)
    if n <= k:
        return list(range(n))
    keep = {0, n - 1}
    segs = [(0, n - 1)]
    while len(keep) < k and segs:
        best = None
        for a, b in segs:
            if b - a < 2:
                continue
            A, Bp = p[a], p[b]
            d = Bp - A
            L = np.linalg.norm(d)
            q = p[a + 1:b] - A
            if L < 1e-12:
                dist = np.linalg.norm(q, axis=1)
            else:
                dist = np.linalg.norm(np.cross(q, d / L), axis=1)
            i = int(np.argmax(dist))
            if best is None or dist[i] > best[0]:
                best = (dist[i], a, b, a + 1 + i)
        if best is None:
            break
        _, a, b, m = best
        keep.add(m)
        segs.remove((a, b))
        segs += [(a, m), (m, b)]
    return sorted(keep)


def line_slack(d, T, w=W_LINE):
    """extra length of a line of span d under a transverse load w (N/m) at tension T over the straight chord (shallow
    catenary: sag s = w d^2 / 8 T, extra 8 s^2 / 3 d)"""
    s = w * d * d / (8.0 * max(T, 1e-3))
    return 8.0 * s * s / (3.0 * max(d, 1e-6))


def tangent_angle(c, r, q, side):
    dx, dz = q[0] - c[0], q[1] - c[1]
    d = math.hypot(dx, dz)
    base = math.degrees(math.atan2(dz, dx))
    return base + side * math.degrees(math.acos(min(1.0, r / max(d, 1e-9))))


# ------------------------------------------------------------------ main build
def build(args):
    t_start = time.time()
    log = []

    def say(*a):
        s = " ".join(str(x) for x in a)
        print(s, flush=True)
        log.append(s)

    out_name = os.path.basename(os.path.normpath(DM.OUT))
    if out_name not in PICKS:
        raise SystemExit(f"no launch inputs for {out_name}: run search_landing.py and add its pick to PICKS")
    PICK = PICKS[out_name]
    SIM_JOB = DM.sim_job()
    JM = DM.joint_map()
    say("dart build", DM.OUT, "| sim layout", SIM_JOB["layout"], "| joint map", JM)

    # ---------------- base and its real-time schedule up to the shot
    say("loading base kinematics ...")
    B = BM.BaseKin(aim_deg=PICK["rail_deg"])
    SCH = BM.RealSchedule(B)
    aim_p = float(B.aim_end_pitch)
    say("aim: wiper", round(B.W34, 2), "deg backward -> pitch", round(aim_p, 4), "in", round(SCH.events["aimed"] - SCH.events["aim_start"], 2),
        "s; cock 425 deg in", round(SCH.events["cocked"] - SCH.events["cock_start"], 2), "s")
    sys.path.insert(0, os.path.join(ROOT, "kedge"))
    import dart_model as dmod
    _, lres, ltr, _ = dmod.launch_trace(round(aim_p, 2))       # base/physics launcher.py chain-spring model (279 g dart)
    trace = ltr[:, :2].copy()
    v_carr = float(np.max(ltr[:, 2]))
    i_end = int(np.argmax(trace[:, 1] >= trace[:, 1].max() - 1e-9))
    trace[i_end, 0] = trace[i_end - 1, 0] + (trace[i_end, 1] - trace[i_end - 1, 1]) / ltr[i_end - 1, 2]
    trace = trace[: i_end + 1]
    B.stop_model = BM.carriage_stop_model(v_carr)
    st = B.stop_model
    T0 = SCH.t_fire0
    S_cocked = SCH.state(T0 - 1e-6, aim_p, None)
    wprof = (SCH.fire_t, SCH.fire_w)
    _, info1 = B.fire_state(S_cocked, SCH.t_rel + 0.001, trace, wprof=wprof)
    t_rel, T_st = SCH.t_rel, info1["T_stroke"]
    tau_home = t_rel + T_st
    say("shot: release", round(t_rel * 1000, 1), "ms after the wiper restarts; stroke", round(T_st * 1000, 1), "ms; stop flexure",
        round(-st["umin"], 1), "mm, rebound", round(st["umax"], 1), "mm, settled", round(st["t_settle"] * 1000), "ms")
    place_dart = [np.array(p["place"], float) for p in B.asm["parts"] if p["name"] == "launch_dart"][0]

    def dart_on_carriage(S):
        return BM.to_world_m(B.joint_world(S)["launch_dart_rot"] @ place_dart)

    # ---------------- launch and free flight from the picked inputs, to 20 mm above the ground
    CGX = SIM_JOB["D"]["hull"]["cgX"] / 1000.0
    geom = LD.LaunchGeom(B, S_cocked, CGX)
    inp = {k: v for k, v in PICK.items() if k != "source"}
    inp["rail_deg"] = round(aim_p, 6)
    cloud = DM.dart_cloud()
    case = LD.Case(geom, inp, cloud)
    V0 = inp["v0"]
    tau_sep = tau_home + case.slide / V0
    tau_h = tau_sep + case.t_fl
    say("flight", case.summary())
    import search_landing as SL
    reel_w = SL.reel_point(B)
    init = case.sim_init(CGX, reel_w)

    # ---------------- the righting sim FROM the flight's end state
    sim_path = os.path.join(HERE, "sim", f"sim_pick_{out_name}.json")
    os.makedirs(os.path.dirname(sim_path), exist_ok=True)
    keys = ("p", "q", "v", "w", "cg", "reel")
    stale = True
    if os.path.exists(sim_path) and not args.rerun_sim:
        old = json.load(open(sim_path))["meta"].get("init")
        stale = old is None or np.abs(np.concatenate([old[k] for k in keys]) - np.concatenate([init[k] for k in keys])).max() > 1e-9
    if stale:
        job = dict(SIM_JOB, out=sim_path, seed=0, init=init, dump=True, dtFine=0.002, tFine=3.0, dtCoarse=0.01)
        env = dict(os.environ)
        if args.node_path:
            env["NODE_PATH"] = args.node_path
        subprocess.run(["node", os.path.join(HERE, "sim_dump.mjs"), json.dumps(job)], check=True, env=env, cwd=HERE)
    sim = DM.SimTrace(sim_path)
    sm = sim.summary
    say("sim from the flight end: righted", sm["righted"], "rightT", round(sm["rightT"] or -1, 2), "anchored", sm["anchored"],
        "held", sm["held"], "moves", [(m["move"], round(m["t"], 2), round(m["tilt"])) for m in sm["moves"]])
    if not (sm["righted"] and sm["anchored"] == 2 and sm["held"] >= SL.HELD_MIN and any(m["move"] in SL.LEG_MOVES for m in sm["moves"])):
        raise SystemExit("the picked landing does not right with the leg, plant both drills and hold: re-run search_landing.py")
    ph_sim = dict((p["p"], p["t"]) for p in sm["phases"])
    SMAP = DM.SimMap(0.0, np.zeros(3))
    t_contact = None
    for i in np.where(sim.t <= 0.5)[0]:
        p_s, q_s, *_ = sim.at(float(sim.t[i]))
        rp, Rr_ = SMAP.root(p_s, q_s, CGX)
        if float((cloud @ Rr_.T + rp)[:, 2].min()) <= 0.002:
            t_contact = float(sim.t[i])
            break
    t_contact = t_contact if t_contact is not None else ph_sim["landing"]
    t_right_sim = ph_sim["landing"] + sm["rightT"]
    t_anch_sim = ph_sim["tow"]
    t_settle_sim = t_anch_sim + 1.0                   # the sim's tow starts with 1 s on the light drag
    tS = sim.t
    dL, dR, aL, aR = sim.col("drillL"), sim.col("drillR"), sim.col("anchorL"), sim.col("anchorR")
    run = ((dL > 0.5) & (aL < 0.5)) | ((dR > 0.5) & (aR < 0.5))
    t_drill_sim = float(tS[run][0])
    e_ = case.end_state()
    p0_sim, q0_sim, *_ = sim.at(0.0)
    root0, R0 = SMAP.root(p0_sim, q0_sim, CGX)
    hand = dict(pos_mm=round(float(np.linalg.norm(root0 - e_["root"])) * 1000, 4),
                rot_deg=round(math.degrees(math.acos(min(1.0, (np.trace(R0.T @ e_["R"]) - 1) / 2))), 4),
                v_flight=np.round(e_["v"], 4).tolist(), w_flight=np.round(e_["w"], 4).tolist(),
                w_sim=np.round(DM.C_SIM @ np.array([sim.col("wx")[0], sim.col("wy")[0], sim.col("wz")[0]]), 4).tolist())
    say("handover (no blend):", hand)

    # ---------------- the real-time clock: base after LANDED, dart events
    t_hand = T0 + tau_h
    t_imp = t_hand + t_contact
    t_anch = t_hand + t_anch_sim
    t_tidy0 = SCH.post_landing_head(t_imp + LANDED_DELAY, t_anch)
    ev = SCH.events
    # the one cut: cam:A until the drills have run a few seconds, cam:B from the last turns of the wrap
    cut_f = int(math.ceil((t_hand + t_drill_sim + 4.0) * FPS))
    F_cut = cut_f / FPS
    t_cut_from = F_cut
    t_cut_to = ev["base_wrap_done"] - 3.0
    TT = dict(tow=1e9)                                  # set once the tail of the schedule is known

    def t_of_frame(f):
        return f / FPS if f < cut_f else t_cut_to + (f - cut_f) / FPS

    def ts_of(t):
        """sim time shown at real time t: live until the tow-settle end, still until the base tows, then the sim tow"""
        if t < t_hand + t_settle_sim:
            return t - t_hand
        if t < TT["tow"]:
            return t_settle_sim
        return t_settle_sim + min(TOW_SIM, t - TT["tow"])

    knot_d = np.array(DM.dart_parts()["P"]["TIE"]["knot"], float) / 1000.0
    Tcol = sim.col("T")
    sp = JM["augers"]
    w_dr = DRILL_RPM * 6.0
    spin = np.zeros_like(tS)
    om = 0.0
    for i in range(1, len(tS)):
        dt = tS[i] - tS[i - 1]
        om += np.clip((w_dr if run[i] else 0.0) - om, -w_dr * dt / 0.15, w_dr * dt / 0.15)
        spin[i] = spin[i - 1] + om * dt

    def base_state(t):
        if T0 <= t < SCH.t_fire_end:
            S, info = B.fire_state(S_cocked, t - T0, trace, wprof=wprof)
            S["t"] = t
            return S
        return SCH.state(t, aim_p, S_cocked)

    def dart_pose(t, S):
        """(kind, root p, R, leg deg, pod deg, auger spin deg, sim time or None)"""
        if t < T0 + tau_home:
            Wd = dart_on_carriage(S)
            return "carriage", Wd[:3, 3], Wd[:3, :3], 0.0, 0.0, 0.0, None
        if t < T0 + tau_sep:
            s_ = V0 * (t - T0 - tau_home)
            f_ = DM.smooth(s_ / case.slide)
            R0f = DM.R_flight(math.radians(inp["rail_deg"] + inp["dth"]), 0.0, math.radians(inp["dpsi"]))
            R_ = DM.q2R(DM.slerp(DM.R2q(case.R_rail), DM.R2q(R0f), f_))
            cg_ = case.Wh[:3, 3] + case.R_rail @ case.cg_d + case.u * s_
            return "slide", cg_ - R_ @ case.cg_d, R_, 0.0, 0.0, 0.0, None
        if t < t_hand:
            p, R = case.pose(t - T0 - tau_sep)
            return "flight", p, R, 0.0, 0.0, 0.0, None
        ts = ts_of(t)
        p_s, q_s, leg, pl, pr, _ = sim.at(ts)
        p, R = SMAP.root(p_s, q_s, CGX)
        return ("sim", p, R, max(0.0, JM["k_leg"] * leg), max(0.0, JM["k_pod"] * 0.5 * (pl + pr)),
                float(np.interp(ts, tS, spin)), ts)

    def make_frame(f):
        t = t_of_frame(f)
        S = base_state(t)
        kind, p, R, leg, pod, aug, ts = dart_pose(t, S)
        return dict(f=f, t=t, S=S, dart=(kind, p, R), leg=leg, pod=pod, aug=aug, ts=ts, knot=p + R @ knot_d)

    # ---------------- tether rope over real time (continuous through the cut), sampled at the frames
    decks = []
    for nm, j, pl in B.parts:
        if nm.startswith("frame_deck"):
            bb = [p_["bbox"] for p_ in B.asm["parts"] if p_["name"] == nm][0]
            c = np.array([[x / 1000.0, y / 1000.0, z / 1000.0, 1.0] for x in (bb[0][0], bb[1][0]) for y in (bb[0][1], bb[1][1])
                          for z in (bb[0][2], bb[1][2])])
            w_ = (c @ BM.to_world_m(pl).T)[:, :3]
            decks.append((w_[:, 0].min(), w_[:, 0].max(), w_[:, 1].min(), w_[:, 1].max(), w_[:, 2].max()))

    def ground_fn(x, y):
        """surface under the line: the base deck tiles (their tops) where they are, else the board / ground at z 0"""
        z = np.zeros_like(x)
        for x0, x1, y0, y1, zt in decks:
            m = (x > x0 - 0.002) & (x < x1 + 0.002) & (y > y0 - 0.002) & (y < y1 + 0.002)
            z = np.where(m, np.maximum(z, zt), z)
        return z
    Rope.ground_fn = staticmethod(ground_fn)

    def fixed_and_exit(S, toward_w, launched):
        """base path (world m) spool -> pusher, the wrap index range, and after the launch the line along the cradle and
        over the muzzle lip roller (wrapping it whenever the rope leaves below the pusher -> roller-top line): the exit"""
        pts, wr = B.line_fixed(S)
        if not launched:
            return BM.pts_world_m(pts), wr, False
        v = B.values(S)
        Wr_ = B.rail_W(v["launch_rail"])
        Wi = np.linalg.inv(Wr_)
        pusher = (Wi @ np.r_[pts[-1], 1.0])[:3]
        q = (Wi @ np.r_[(np.linalg.inv(BM.BASE_TO_WORLD) @ np.r_[toward_w, 1.0])[:3] * 1000.0, 1.0])[:3]
        ur_, wr2, rp2 = B.roller(S, v["launch_carriage"])
        extra, used = [], False
        # wrap when the rope point lies below the line from the pusher hole to the roller top (it would otherwise run
        # through the cradle), or behind the pusher; a dart still on the axis in front of the pusher pulls it straight
        top_w = wr2 + rp2
        if q[0] <= pusher[0] + 1.0:
            below = True
        else:                                           # the pusher -> q ray where it reaches the roller (base rule)
            w_at_r = pusher[2] + (ur_ - pusher[0]) * (q[2] - pusher[2]) / (q[0] - pusher[0])
            below = w_at_r < top_w + 1.5
        if not below and q[0] > ur_ + 1.0:
            # the line runs inside the cradle along the dart axis to the funnel mouth over the lip roller, then free
            extra.append((Wr_ @ np.array([ur_, 0.0, pusher[2], 1.0]))[:3])
        if below:
            a_d = tangent_angle((ur_, wr2), rp2, (q[0], q[2]), 1.0)
            a_d = min(90.0, a_d)
            for a in np.linspace(90.0, max(a_d, -170.0), 7):
                extra.append((Wr_ @ np.array([ur_ + rp2 * math.cos(math.radians(a)), 0.0, wr2 + rp2 * math.sin(math.radians(a)), 1.0]))[:3])
            used = True
        if extra:
            pts = np.vstack([pts, np.array(extra)])
        return BM.pts_world_m(pts), wr, used

    RS = dict(rope=None, total=None, total0=None, spool_v=0.0, t=0.0, knot=None, exit=None, base=None, taut=None)
    lens, line_frames, wraps_frames, roller_used = [], [], [], []
    t_launch_free = T0 + tau_sep

    def rope_advance(t, S, knot, fr=None):
        rope = RS["rope"]
        toward = knot if rope is None or rope.n() < 3 else rope.x[-2]
        fixed_w, wr, used = fixed_and_exit(S, toward, t >= t_launch_free)
        exit_w = fixed_w[-1]
        base_len = float(np.linalg.norm(np.diff(fixed_w, axis=0), axis=1).sum())
        if rope is None:
            rope = RS["rope"] = Rope(knot, exit_w)
            RS["total"] = RS["total0"] = base_len + rope.L
        else:
            dt_tot = t - RS["t"]
            fine = t <= t_imp + 1.0
            nsub = max(1, int(math.ceil(dt_tot / (0.001 if fine else 0.002))))
            for k in range(1, nsub + 1):
                a = k / nsub
                kn = RS["knot"] + (knot - RS["knot"]) * a
                ex = RS["exit"] + (exit_w - RS["exit"]) * a
                bl = RS["base"] + (base_len - RS["base"]) * a
                dts = dt_tot / nsub
                tt = RS["t"] + dt_tot * a
                dd = float(np.linalg.norm(kn - ex))
                L_free = RS["total"] - bl
                if tt < t_launch_free:
                    L_free = dd
                    RS["spool_v"] = V0
                elif tt < t_imp:
                    need = dd + line_slack(dd, T_FLIGHT, W_LINE + W_DRAG_FLIGHT)
                    v_free = max(0.0, RS["spool_v"] - SPOOL_DECEL * dts)
                    if L_free + v_free * dts >= need:
                        RS["spool_v"] = v_free
                        L_free += v_free * dts
                    else:
                        RS["spool_v"] = (need - L_free) / dts
                        L_free = need
                elif tt < ev["tidy_start"]:
                    RS["spool_v"] = max(0.0, min(RS["spool_v"], V_OVERRUN) - SPOOL_DECEL * dts)
                    L_free = max(L_free + RS["spool_v"] * dts, dd * 1.0005)    # pays out (LIGHT drag) only when pulled
                elif tt < ev.get("tidy_done", 1e9):
                    wr_rate = math.radians(max(0.0, (SCH.wiper.at(tt + 0.01) - SCH.wiper.at(tt - 0.01)) / 0.02))
                    L_free = max(dd + line_slack(dd, T_TIDY), L_free - 2.5 * wr_rate * R_SPOOL * dts)
                elif tt < TT["tow"]:
                    L_free = dd + line_slack(dd, T_TIDY)
                else:
                    Tn = float(np.interp(ts_of(tt), tS, Tcol))
                    L_free = dd + line_slack(dd, max(T_TOW_MIN, Tn))
                RS["total"] = bl + L_free
                rope.step(dts, kn, ex, L_free, iters=6)
        RS["t"], RS["knot"], RS["exit"], RS["base"] = t, knot, exit_w, base_len
        if fr is None:
            return
        L_free = RS["total"] - base_len
        d_ee = float(np.linalg.norm(knot - exit_w))
        if RS["taut"] is None and t >= ev["tidy_start"] and L_free <= d_ee + 2.0 * line_slack(d_ee, T_TIDY):
            RS["taut"] = fr["f"]
        lens.append((fr["f"], L_free, d_ee, base_len, rope.n()))
        roller_used.append(used)
        if wr is not None:
            i0, i1 = wr
            wraps_frames.append((fr["f"], resample_poly(fixed_w[i0:i1 + 1], K_WRAPS)))
            fixed_vis = np.vstack([fixed_w[:i0 + 1], fixed_w[i1:]])
        else:
            fixed_vis = fixed_w
        ki = dp_keep(fixed_vis, K_FIXED)
        fixed_pts = fixed_vis[ki] if len(ki) == K_FIXED else resample_poly(fixed_vis, K_FIXED, ki)
        line_frames.append(np.vstack([fixed_pts, rope.points(K_FREE + 1)[1:]]))
        fr["line_out_m"] = RS["total"] - RS["total0"]
        fr["fixed_w"] = fixed_w
        fr["rope_x"] = rope.x.copy()

    # quarter-frame keys where things move fast (the shot and the slide off the cradle; impact and bounce), so the
    # renderer's motion blur sees the real motion between frames
    sub_windows = [(int(math.floor((T0 + t_rel) * FPS)) - 1, int(math.ceil((T0 + tau_sep) * FPS)) + 9),
                   (int(math.floor(t_imp * FPS)) - 3, int(math.ceil(t_imp * FPS)) + 24)]

    def frame_list(f0, f1):
        out = []
        for f in range(f0, f1):
            out.append(f)
            if any(a <= f < b for a, b in sub_windows) and f + 1 < cut_f:
                out += [f + 0.25, f + 0.5, f + 0.75]
        return out

    def run_frames(f0, f1, frames):
        """frames f0..f1-1 (made and roped in order), quarter frames in the fast windows, and the hidden stretch of the
        cut at 20 ms"""
        for f in frame_list(f0, f1):
            if f == cut_f:
                for th in np.arange(t_cut_from, t_cut_to, 0.02)[1:]:
                    S = base_state(th)
                    kind, p, R, *_ = dart_pose(th, S)
                    rope_advance(th, S, p + R @ knot_d)
            fr = make_frame(f)
            rope_advance(fr["t"], fr["S"], fr["knot"], fr)
            frames.append(fr)

    # stage 1: every frame before the tidy starts
    frames = []
    f_tidy = cut_f + int(math.ceil((t_tidy0 - t_cut_to) * FPS))
    run_frames(0, f_tidy, frames)
    slack0 = lens[-1][1] - (lens[-1][2] + line_slack(lens[-1][2], T_TIDY))
    tidy_dur = max(1.5, slack0 / SCH.takeup_speed() + 0.6)
    t_tow = SCH.post_landing_tail(t_tidy0, tidy_dur)
    TT["tow"] = t_tow
    t_tow_end = t_tow + TOW_SIM
    t_end = t_tow_end + HOLD_S + 1.0
    say("slack at the tidy start", round(slack0, 3), "m -> tidy", round(tidy_dur, 2), "s at", round(SCH.takeup_speed(), 3), "m/s")

    # the capstan take-in during the tow (line stretch + the dart's give in the soil, from the sim)
    def knot_at_sim(ts):
        p_s, q_s, *_ = sim.at(ts)
        rp, Rr_ = SMAP.root(p_s, q_s, CGX)
        return rp + Rr_ @ knot_d
    exit_tow = RS["exit"]
    d0 = float(np.linalg.norm(knot_at_sim(t_settle_sim) - exit_tow))
    T0n = float(np.interp(t_settle_sim, tS, Tcol))
    r_cap = B.RL.LINE_R_CAP / 1000.0
    gear_cap = B.RL.Z_MAIN / B.RL.Z_CAPG

    def take_deg(t):
        ts = ts_of(t)
        Tn = float(np.interp(ts, tS, Tcol))
        d = float(np.linalg.norm(knot_at_sim(ts) - exit_tow))
        take = (Tn - T0n) * d / EA_LINE + (d0 - d) + (line_slack(d0, T_TIDY) - line_slack(d, max(T_TOW_MIN, Tn)))
        return math.degrees(take / r_cap) / gear_cap
    SCH.tow(t_tow, take_deg, t_tow_end)
    th_tow = [float(np.interp(ts_of(t), tS, 0.5 * (sim.col("podL_deg") + sim.col("podR_deg")))) for t in np.linspace(t_tow, t_tow_end, 14)]
    say("pods under the tow (deg, stop", SIM_JOB["D"]["pod"]["stopDeg"], "):", [round(x, 2) for x in th_tow])
    say("tow: capstan take-in", round(take_deg(t_tow_end) * gear_cap * math.radians(1) * r_cap * 1000, 1), "mm to",
        round(float(np.interp(ts_of(t_tow_end), tS, Tcol)), 1), "N")
    F_total = F_cut + (t_end - t_cut_to)
    NF = int(round(F_total * FPS)) + 1
    say("real time: fire", round(T0 + t_rel, 2), "impact", round(t_imp, 2), "righted", round(t_hand + t_right_sim, 2), "drills",
        round(t_hand + t_drill_sim, 2), "anchored", round(t_anch, 2), "| base: rail low", round(ev["base_rail_low_start"], 2),
        "wrap", round(ev["base_wrap_start"], 2), "-", round(ev["base_wrap_done"], 2), "tidy", round(ev["tidy_start"], 2),
        "tow", round(t_tow, 2), "end", round(t_end, 2))
    say("cut: frame", cut_f, "(film", round(F_cut, 2), "s) jumps real", round(t_cut_from, 2), "->", round(t_cut_to, 2), "(",
        round(t_cut_to - t_cut_from, 1), "s hidden ); film", round(F_total, 2), "s,", NF, "frames")
    # stage 2: the rest
    run_frames(f_tidy, NF, frames)
    taut_frame = RS["taut"]
    say("frames + rope done", round(time.time() - t_start, 1), "s; max free length", round(max(l[1] for l in lens), 3), "m; max nodes",
        max(l[4] for l in lens))
    for fr in frames:
        fr["S"] = dict(fr["S"], line_out_m=float(fr["line_out_m"]))
    is_int = np.array([float(fr["f"]).is_integer() for fr in frames])

    # ---------------- base world transforms per frame
    say("posing base parts ...")
    rest = {nm: BM.to_world_m(pl) for nm, _, pl in B.parts}
    base_tracks = {nm: [] for nm, _, _ in B.parts}
    springs = {"l": [], "r": []}
    gear_check = []
    for fr in frames:
        S = fr["S"]
        JW = B.joint_world(S)
        for nm, M in B.world(S, JW).items():
            base_tracks[nm].append(BM.to_world_m(M))
        se = B.spring_ends(S, JW)
        for side in ("l", "r"):
            springs[side].append((BM.pts_world_m(se[side][0]), BM.pts_world_m(se[side][1])))
        v = B.values(S)
        gear_check.append((fr["f"], v["launch_carriage"], v["launch_big_gear"], v["launch_drive"], v["launch_rail"]))
    say("posed", round(time.time() - t_start, 1), "s")

    # ---------------- dart tracks
    dart_loc = np.array([fr["dart"][1] for fr in frames])
    dart_R = [fr["dart"][2] for fr in frames]
    dart_q = []
    for R in dart_R:
        q = DM.R2q(R)
        if dart_q and float(q @ dart_q[-1]) < 0:
            q = -q
        dart_q.append(q)
    dart_q = np.array(dart_q)
    leg = np.array([fr["leg"] for fr in frames])
    pod = np.array([fr["pod"] for fr in frames])
    aug = np.array([fr["aug"] for fr in frames])

    # ---------------- events
    def fnum(t):
        if t < t_cut_from:
            return min(cut_f - 1, int(round(t * FPS)))
        if t < t_cut_to:
            return None
        return cut_f + int(round((t - t_cut_to) * FPS))

    evl = [("aim_start", ev["aim_start"]), ("aimed", ev["aimed"]), ("cock_start", ev["cock_start"]), ("cocked", ev["cocked"]),
           ("fire", T0 + t_rel), ("carriage_home", T0 + tau_home), ("muzzle_exit", T0 + tau_sep), ("impact", t_imp),
           ("base_rail_low_start", ev["base_rail_low_start"]), ("base_drum_to_neutral", ev["base_drum_to_neutral"]),
           ("base_wrap_start", ev["base_wrap_start"])]
    evl += [("righting_" + m["move"].replace(":", "_"), t_hand + m["t"]) for m in sm["moves"]]
    evl += [("righted", t_hand + t_right_sim), ("pods_deploy_start", t_hand + ph_sim["anchor"]),
            ("drills_start", t_hand + t_drill_sim), ("anchored", t_anch), ("base_wrap_done", ev["base_wrap_done"]),
            ("base_wrapped_park", ev["base_wrapped_park"]), ("tidy_start", ev["tidy_start"]),
            ("reel_engage", ev["reel_engage"]), ("tow_start", t_tow), ("tension_hold", t_tow_end), ("end", t_of_frame(NF - 1))]
    events, hidden_ev = [], []
    for nm, t in evl:
        f = fnum(t)
        (hidden_ev if f is None else events).append((f, nm, round(t, 3)))
    if taut_frame is not None:
        events.append((taut_frame, "reel_taut", round(t_of_frame(taut_frame), 3)))
    events.append((cut_f, "cut", round(t_cut_to, 3)))
    events.sort()
    say("events:", [(f, nm) for f, nm, _ in events], "| hidden by the cut:", [(nm, t) for _, nm, t in hidden_ev])

    # ---------------- assemble motion.json (sparse keys; the cut frame is always a key)
    def fnums(idx):
        return [int(fidx[i]) if float(fidx[i]).is_integer() else float(fidx[i]) for i in idx]

    fidx = [fr["f"] for fr in frames]
    pos = {f: i for i, f in enumerate(fidx)}
    always = sorted(set(pos[f] for f in [e[0] for e in events] + [cut_f - 1, cut_f] if f in pos))
    objects = {}
    n_static_moved = 0
    for nm, tr_ in base_tracks.items():
        M = np.array(tr_)
        loc = M[:, :3, 3]
        qs = []
        for Mi in M:
            q = DM.R2q(Mi[:3, :3])
            if qs and float(q @ qs[-1]) < 0:
                q = -q
            qs.append(q)
        qs = np.array(qs)
        if (np.abs(M - M[0]).max() < 1e-7) and np.abs(M[0] - rest[nm]).max() < 1e-7:
            continue
        if np.abs(M - M[0]).max() < 1e-7:
            keep = [0]
            n_static_moved += 1
        else:
            keep = sorted(set(decimate(fidx, np.c_[loc / TOL_LOC, qs / TOL_Q], 1.0, always)))
        objects["base:" + nm] = dict(frames=fnums(keep), loc=np.round(loc[keep], 4).tolist(), quat=np.round(qs[keep], 5).tolist())
    keep = sorted(set(decimate(fidx, np.c_[dart_loc / TOL_LOC, dart_q / TOL_Q], 1.0, always)))
    objects["dart:root"] = dict(frames=fnums(keep), loc=np.round(dart_loc[keep], 4).tolist(), quat=np.round(dart_q[keep], 5).tolist())

    J34 = DM.dart_parts()
    PP = {p["name"]: p for p in J34["parts"]}

    def jkeys(vals):
        return sorted(set(decimate(fidx, np.asarray(vals)[:, None] / TOL_DEG, 1.0, always)))
    joints = {}
    k = jkeys(leg)
    joints["dart:righting_leg"] = dict(frames=fnums(k), parent="dart:root", axis=[0.0, float(PP["righting_leg"].get("sgn", 1)), 0.0],
                                       pivot=np.round(np.array(PP["righting_leg"]["pos"]) / 1000, 5).tolist(), deg=np.round(leg[k], 3).tolist())
    k = jkeys(pod)
    for nm in ("drill_leg_L", "drill_leg_R"):
        joints["dart:" + nm] = dict(frames=fnums(k), parent="dart:root", axis=[0.0, float(PP[nm].get("sgn", 1)), 0.0],
                                    pivot=np.round(np.array(PP[nm]["pos"]) / 1000, 5).tolist(), deg=np.round(pod[k], 3).tolist())
    k = jkeys(aug)
    for side in ("L", "R"):
        nm = f"drill_leg_{side}_auger"
        joints["dart:" + nm] = dict(frames=fnums(k), parent="dart:" + PP[nm]["parent"], axis=sp[side]["axis"], pivot=[0.0, 0.0, 0.0],
                                    pivot_dart_frame_folded=np.round(np.array(PP[PP[nm]["parent"]]["pos"]) / 1000, 5).tolist(),
                                    deg=np.round(sp[side]["drill_in_sign"] * aug[k], 2).tolist())

    LF = np.array(line_frames)
    lk = sorted(set(decimate(fidx, LF.reshape(len(LF), -1) / TOL_PT, 1.0, always)))
    line = dict(frames=fnums(lk), points=np.round(LF[lk], 4).tolist(), k_base=K_FIXED, k_free=K_FREE)
    if wraps_frames:
        wf = [w[0] for w in wraps_frames]
        WP = np.array([w[1] for w in wraps_frames])
        wk = sorted(set(decimate(list(range(len(wf))), WP.reshape(len(wf), -1) / TOL_PT, 1.0)))
        line_wraps = dict(frames=[wf[i] for i in wk], points=np.round(WP[wk], 4).tolist(),
                          note="capstan turns of the tether while the wrap cup has laid them (the main line skips them with a chord inside the drum)")
    else:
        line_wraps = None
    sp_out = {}
    for side in ("l", "r"):
        A_ = np.array([s[0] for s in springs[side]])
        C_ = np.array([s[1] for s in springs[side]])
        kk = sorted(set(decimate(fidx, np.c_[A_, C_] / TOL_LOC, 1.0, always)))
        sp_out[f"launch_spring_{side}"] = dict(frames=fnums(kk), anchor_pin=np.round(A_[kk], 4).tolist(), carriage_pin=np.round(C_[kk], 4).tolist(),
                                               length=np.round(np.linalg.norm(A_[kk] - C_[kk], axis=1), 4).tolist(),
                                               hook_l=0.012, coil_od=0.016, free_length_pins=round(B.n["spring_free"] / 1000, 4))

    film_note = (f"30 fps, {NF} frames ({F_total:.1f} s), REAL TIME 1:1 (no slow motion): frame f shows real time f / 30 s until "
                 f"the cut at frame {cut_f} ({F_cut:.2f} s); from there real time {t_cut_to:.2f} + (f / 30 - {F_cut:.2f}) s "
                 f"({t_cut_to - t_cut_from:.1f} s of drilling and the base's wrap are skipped). cam:A (static wide) up to the cut, "
                 f"cam:B (static, side-on to the line) after it. Keys are sparse: interpolate LINEARLY between keys, never "
                 f"across the cut (frames {cut_f - 1} and {cut_f} are both keys). Around the shot and the impact some keys sit "
                 f"on quarter frames (e.g. 320.25) so motion blur sees the motion between frames: key them at those "
                 f"fractional frames. Tolerances {TOL_LOC} m, {TOL_Q} quat, "
                 f"{TOL_DEG} deg, line {TOL_PT} m.")
    outj = dict(
        fps=FPS, frames=NF, film_time_note=film_note,
        units="metres, world: Z up, +X launch direction, origin = centre of the board top (z 0)",
        cut=dict(frame=cut_f, real_time_before=round(t_cut_from, 3), real_time_after=round(t_cut_to, 3), cam_before="cam:A", cam_after="cam:B"),
        real_time_of_frame="t = f / fps for f < cut.frame, else cut.real_time_after + (f - cut.frame) / fps",
        base_to_world=BM.BASE_TO_WORLD.tolist(),
        base_to_world_note=("world_m = base_to_world @ S(0.001) @ M_base_mm, where M_base_mm is a part's base-frame transform: "
                            "asm.json 'place' composed with its joint chain, JW(joint) @ place (tools.assembly.world). objects[base:*] "
                            "give exactly that per frame (set it as matrix_world). Frame 0 is the load state = models.py's rest pose; "
                            "parts that never move and sit at base_to_world @ place are omitted. Base frame (BASE_SPEC section 2): origin "
                            "on the wiper / main shaft axis, deck top z -165 mm, deck underside -173 mm: base_to_world puts the deck "
                            "underside on the board top (z 0) and centres the deck footprint on the origin; no rotation."),
        objects=objects, joints=joints, line=line, line_wraps=line_wraps, springs=sp_out,
        events=[dict(frame=int(f), name=nm, real_time=t) for f, nm, t in events],
        events_hidden_by_cut=[dict(name=nm, real_time=t) for _, nm, t in hidden_ev],
        sources=dict(base="base/cad/parts (drum, frame, launch, reel, sensors: joints, kin, line_path) posed with base/out/base/asm.json; "
                          "real-time motor schedule base_motion.RealSchedule (wiper DC motor 45 rpm / 15 N m against the springs and trains, "
                          "drum N20 8.1 deg/s, firmware C1 order)",
                     shot="base/physics launcher.py chain-spring trace via kedge/dart_model.launch_trace; carriage stop flexure model",
                     flight="kedge/flight34.py strip aero in 3D (dart_motion.Flight3D), 0.25 N payout drag on the tail tie, cross-wind",
                     landing="kedge/sim_righting simCore + v34_check/run34_shared.mjs SimV34 (rigid axle, tail tie), loam, started from the flight's end state, no blend",
                     launch_inputs=PICK["source"], dart_build=os.path.relpath(DM.OUT, ROOT),
                     line="render/motion/rope.py on the base line_path (reel.line_spans + launch.line_path) and over the muzzle lip roller"),
        dart=dict(launch_inputs=dict(inp, rail_deg_commanded=PICK["rail_deg"]), handover=hand, knot_dart_frame=knot_d.tolist(),
                  cg_station_m=CGX, auger_rpm=DRILL_RPM, sim_job=SIM_JOB,
                  auger_note="auger joints: axis = the auger's own axis toward its tip in the pod frame; deg already carries the "
                             "drill-in sign (right-hand flight turns + about the tip direction): both screw in",
                  sim_to_world="world = C_SIM @ sim (x, y up, z right) -> (X, -z, y), same origin"),
    )
    path = os.path.join(HERE, "motion.json")
    with open(path, "w") as fh:
        json.dump(outj, fh, separators=(",", ":"))
    say("wrote", path, round(os.path.getsize(path) / 1e6, 2), "MB;", len(objects), "objects (", n_static_moved, "static off rest )")

    # ---------------- checks and plots
    tf_all = np.arange(NF) / FPS
    ii = np.where(is_int)[0]
    frames_i = [frames[i] for i in ii]
    base_tracks_i = {nm: [tr_[i] for i in ii] for nm, tr_ in base_tracks.items()}
    springs_i = {k_: [v_[i] for i in ii] for k_, v_ in springs.items()}
    ctx = dict(B=B, frames=frames_i, tf=tf_all, dart_loc=dart_loc[ii], dart_R=[dart_R[i] for i in ii], leg=leg[ii], pod=pod[ii],
               aug=aug[ii], lens=[lens[i] for i in ii], gear=np.array(gear_check)[ii], fl=case.rows, sim=sim, SMAP=SMAP, ev=[(f, nm) for f, nm, _ in events],
               springs=springs_i, LF=LF[ii], mism=hand, roller=[roller_used[i] for i in ii], J34=J34, knot_d=knot_d, T_FIRE=T0,
               cut_f=cut_f, tau_sep=tau_sep, CGX=CGX, base_tracks=base_tracks_i, wraps=wraps_frames, t_of_frame=t_of_frame,
               frames_all=frames, fl_sub=sub_windows)
    import motion_checks
    res = motion_checks.run(ctx, say)
    res["log"] = log
    json.dump(res, open(os.path.join(HERE, "checks.json"), "w"), indent=1, default=float)
    if not args.no_plots:
        import motion_plots
        motion_plots.run(ctx, os.path.join(HERE, "plots"))
        motion_plots.scene(ctx, os.path.join(HERE, "plots"))
    say("done", round(time.time() - t_start, 1), "s")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--rerun-sim", action="store_true")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--node-path", default=os.environ.get("NODE_PATH", ""),
                    help="folder holding @dimforge/rapier3d-compat@0.14.0 (node_modules)")
    build(ap.parse_args())


if __name__ == "__main__":
    main()
