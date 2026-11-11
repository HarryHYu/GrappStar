"""Build render/motion/motion.json: the physically driven motion of the GrappStar D50 base and the KEDGE v34 dart.

    ../../kedge/.venv/bin/python make_motion.py [--rerun-sim] [--no-plots]

Parts (README.md): base kinematics (base_motion.py: the base's own joints, kin() and line paths, with its timeline),
the shot (base/physics launcher.py trace), free flight (dart_motion.Flight: kedge/flight34.py physics), landing /
righting / anchoring (kedge/sim_righting through sim_dump.mjs, started from the flight's end state, v34 tail tie; launch
inputs PICK from search_landing.py), the tether rope (rope.py), and
the film clock (slow motion baked in). Writes motion.json, plots/*.png and checks.json next to this file.
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
# it (no reel tug), both drills go in and it holds 55 N
PICK = dict(rail_deg=33.987, v0=6.783, q0=-0.908, dth=0.16, r0=0.368, dpsi=3.18, wind_y=0.645, slope_deg=0.0)
SIM_JOB = {"cfg": {"axle": "rigid", "tie": True, "legLen": 3},
           "D": {"hull": {"mass": 0.1905, "cgX": 83.3},
                 "pod": {"mass": 0.026, "pivotX": 132, "pivotY": 9.5, "stopDeg": 46.75, "lat": 31.15},
                 "leg": {"mass": 0.0055, "len": 113, "padW": 37, "pad": 3.5, "hingeX": 33, "hingeY": 14.5, "stopDeg": 178}},
           "LAYOUTS": {"tail": {"pivotX": 132}}, "cgUp": -2.6, "soil": "loam"}      # = v34_tail_tie_worm.json T_rigid_tie
DRILL_RPM = 45.0                  # drill motor under load on 2S (100 rpm 12 V listing)
EA_LINE = 30000.0                 # N, 1 mm Dyneema braid (stretch under the tow)
T_FLIGHT, T_TIDY, T_TOW_MIN = 0.3, 5.0, 5.0     # N: line tension in flight (base physics: 0.25 N payout drag, 0.37 N peak at the dart), FIRM take-up, tow floor
W_LINE = 0.006                    # N/m, 1 mm Dyneema braid (0.6 g/m)
W_DRAG_FLIGHT = 0.010             # N/m, air drag on the line sweeping sideways at about 3-4 m/s while the dart arcs down (Cd 1.2, d 1 mm)
SPOOL_DECEL = 0.25 / 0.035        # m/s2: LIGHT 0.25 N on the spool's effective line mass (I / r^2, base physics payout)
OVERRUN_M = 0.6                   # spool overrun after landing: base physics payout 0.20 m at 0.75 N brake, x 3 at LIGHT 0.25 N
V_OVERRUN = math.sqrt(2 * SPOOL_DECEL * OVERRUN_M)
TAKEUP_V = 0.30                   # m/s, tidy take-up line speed (FIRM clutch slipping once tight)
K_FIXED, K_FREE = 12, 36          # line points per frame: base path (spool -> exit) and the free rope (exit -> knot)
K_WRAPS = 64
TOL_LOC, TOL_Q, TOL_DEG, TOL_PT = 1e-4, 1e-5, 0.02, 1e-3


# ------------------------------------------------------------------ helpers
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

    # ---------------- base
    say("loading base kinematics ...")
    B = BM.BaseKin(aim_deg=PICK["rail_deg"])
    say("aim: wiper", round(B.W34, 2), "deg backward from LOW -> pitch", round(B.aim_end_pitch, 4), "(asked", PICK["rail_deg"], ")")
    sys.path.insert(0, os.path.join(ROOT, "kedge"))
    import dart_model as dmod
    _, lres, ltr, _ = dmod.launch_trace(round(B.aim_end_pitch, 2))   # base/physics launcher.py chain-spring model (279 g dart)
    trace = ltr[:, :2].copy()
    v_carr = float(np.max(ltr[:, 2]))
    i_end = int(np.argmax(trace[:, 1] >= trace[:, 1].max() - 1e-9))     # the trace clamps at its 1 ms step: put the
    trace[i_end, 0] = trace[i_end - 1, 0] + (trace[i_end, 1] - trace[i_end - 1, 1]) / ltr[i_end - 1, 2]   # stop where it happens
    trace = trace[: i_end + 1]
    S_cocked = B.state(B.t0["flight"] - 1e-6)
    _, info0 = B.fire_state(S_cocked, 0.0, trace)
    t_rel = info0["t_rel"]
    Sx, info1 = B.fire_state(S_cocked, t_rel + 0.001, trace)
    T_st = info1["T_stroke"]
    tau_home = t_rel + T_st
    say("shot: release", round(t_rel * 1000, 1), "ms after full cock; stroke", round(T_st * 1000, 1), "ms (launcher.py, carriage",
        round(v_carr, 2), "m/s)")
    stop = BM.carriage_stop_model(v_carr)
    B.stop_model = stop
    say("carriage stop: flexure", round(-stop["umin"], 1), "mm, rebound", round(stop["umax"], 1), "mm, settled", round(stop["t_settle"] * 1000), "ms")

    place_dart = [np.array(p["place"], float) for p in B.asm["parts"] if p["name"] == "launch_dart"][0]

    def dart_on_carriage(S):
        JW = B.joint_world(S)
        return JW["launch_dart_rot"] @ place_dart        # dart frame (mm) in the base frame

    # ---------------- launch and free flight from the picked inputs (search_landing.py), to 20 mm above the ground
    CGX = SIM_JOB["D"]["hull"]["cgX"] / 1000.0           # sim body origin: hull axis at the CG station
    geom = LD.LaunchGeom(B, S_cocked, CGX)
    inp = dict(PICK, rail_deg=round(float(B.aim_end_pitch), 6))
    cloud = DM.dart_cloud()
    case = LD.Case(geom, inp, cloud)
    V0 = inp["v0"]
    Wd_home = case.Wh
    u_dir = case.u
    R_rail = case.R_rail
    slide = case.slide
    tau_sep = tau_home + slide / V0
    fl = case.rows
    t_fl = case.t_fl
    tau_h = tau_sep + t_fl
    say("flight", case.summary())
    import search_landing as SL
    reel_w = SL.reel_point(B)
    init = case.sim_init(CGX, reel_w)

    # ---------------- the righting sim FROM the flight's end state (sim world = C^T world, same origin)
    sim_path = os.path.join(HERE, "sim", "sim_pick.json")
    os.makedirs(os.path.dirname(sim_path), exist_ok=True)
    stale = True
    if os.path.exists(sim_path) and not args.rerun_sim:
        old = json.load(open(sim_path))["meta"].get("init")
        stale = old is None or np.abs(np.r_[np.concatenate([old[k] for k in ("p", "q", "v", "w", "cg", "reel")])]
                                      - np.r_[np.concatenate([init[k] for k in ("p", "q", "v", "w", "cg", "reel")])]).max() > 1e-9
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
    if not (sm["righted"] and sm["anchored"] == 2 and sm["held"] >= 55 and any(m["move"] in SL.LEG_MOVES for m in sm["moves"])):
        raise SystemExit("the picked landing no longer rights with the leg, plants both drills and holds: re-run search_landing.py")
    ph_sim = dict((p["p"], p["t"]) for p in sm["phases"])
    t_land_firmware = ph_sim["landing"]                  # the sim firmware's landing flag (hull cylinder low)
    SMAP = DM.SimMap(0.0, np.zeros(3))
    t_land_sim = None                                   # first ground contact of the real dart shape (folded cloud)
    for i in np.where(sim.t <= 0.5)[0]:
        p_s, q_s, *_ = sim.at(float(sim.t[i]))
        rp, Rr_ = SMAP.root(p_s, q_s, CGX)
        if float((cloud @ Rr_.T + rp)[:, 2].min()) <= 0.002:
            t_land_sim = float(sim.t[i])
            break
    t_land_sim = t_land_sim if t_land_sim is not None else t_land_firmware
    t_right_abs = t_land_firmware + sm["rightT"]
    t_anch_sim = ph_sim["tow"]                         # both drills at their stops, anchors set
    t_settle_end = t_anch_sim + 1.0                    # the sim's tow starts with 1 s on the light drag
    say("first ground contact", round(t_land_sim * 1000, 1), "ms after the handover; righted at sim", round(t_right_abs, 2), "s; anchored", round(t_anch_sim, 2))
    # continuity at the handover (must be zero: the sim starts from the flight's own state)
    e = case.end_state()
    p0_sim, q0_sim, *_ = sim.at(0.0)
    root0, R0 = SMAP.root(p0_sim, q0_sim, CGX)
    v0s = DM.C_SIM @ np.array([sim.col("vx")[0], sim.col("vy")[0], sim.col("vz")[0]])
    w0s = DM.C_SIM @ np.array([sim.col("wx")[0], sim.col("wy")[0], sim.col("wz")[0]])
    hand = dict(pos_mm=round(float(np.linalg.norm(root0 - e["root"])) * 1000, 4),
                rot_deg=round(math.degrees(math.acos(min(1.0, (np.trace(R0.T @ e["R"]) - 1) / 2))), 4),
                v_flight=np.round(e["v"], 4).tolist(), v_sim_com=np.round(v0s, 4).tolist(),
                w_flight=np.round(e["w"], 4).tolist(), w_sim=np.round(w0s, 4).tolist(),
                roll_deg=0.0, note="sim hull COM velocity differs from the flight CG velocity only by w x (COM offset)")
    say("handover (no blend):", hand)
    mism = hand

    def flight_pose(tf_):
        return case.pose(tf_)

    # ---------------- dart clock and film clock
    T_FIRE = B.t0["flight"]                  # dart clock (s) at full cock = start of the shot
    t_d_hand = T_FIRE + tau_h                # sim t = 0
    t_d_land = t_d_hand + t_land_sim
    t_d_settle = t_d_hand + t_settle_end
    w_fire_end = B.wc0 + 553.8
    w_rel = B.wc0 + (B.g0 - B.g_rel) * B.ratio
    tau_fire_end = t_rel + (w_fire_end - w_rel) / 270.0

    knots = [  # (dart clock, film rate) control points; rate = film s per real s, smooth between
        (T_FIRE - 0.12, 1.0), (T_FIRE - 0.005, 20.0), (T_FIRE + tau_sep + 0.05, 20.0), (T_FIRE + tau_sep + 0.20, 3.0),
        (t_d_hand - 0.14, 3.0), (t_d_hand - 0.05, 6.0), (t_d_land + 0.35, 6.0), (t_d_land + 1.2, 1.0)]

    def rate(td):
        if td <= knots[0][0]:
            return 1.0
        for (a, ra), (b, rb) in zip(knots[:-1], knots[1:]):
            if td <= b:
                return ra + (rb - ra) * DM.smooth((td - a) / (b - a))
        return 1.0

    tgrid = np.arange(0.0, t_d_settle + 1e-9, 1e-4)
    rg = np.array([rate(t) for t in tgrid])
    film_of_td = np.r_[0.0, np.cumsum(0.5 * (rg[1:] + rg[:-1]) * np.diff(tgrid))]
    F_settle = film_of_td[-1]
    # reel segment (film seconds): tidy, engage, tow, end hold
    DUR_TIDY, DUR_ENG, DUR_TOW, DUR_END = 2.25, 2.5, 3.0, 2.5
    F_tidy = F_settle
    F_eng = F_tidy + DUR_TIDY
    F_tow = F_eng + DUR_ENG
    F_end = F_tow + DUR_TOW
    F_total = F_end + DUR_END
    NF = int(round(F_total * FPS)) + 1
    say("film: launch at", round(float(np.interp(T_FIRE, tgrid, film_of_td)), 2), "s, handover",
        round(float(np.interp(t_d_hand, tgrid, film_of_td)), 2), "s, anchored+settle", round(F_settle, 2), "s, total",
        round(F_total, 2), "s,", NF, "frames")
    TOW_SIM = 6.0                                       # sim seconds of the tow shown (ramp to 15 N, hold, step to 20 N)

    # base clock in the post-landing stretch: rail_low .. wrapped (63.5 base s) inside [landing + 0.8, anchored]
    td_b0 = t_d_land + 0.8
    td_b1 = t_d_hand + t_anch_sim
    tb_b0, tb_b1 = B.t0["rail_low"], B.t0["anchor"]
    k_base = (tb_b1 - tb_b0) / (td_b1 - td_b0)
    # per phase: the rail swing (4 s) and the drum shift (8 s) less compressed than the 49 s wrap
    share = [("rail_low", 3.0), ("neutral", 3.0), ("wrap", 11.0), ("wrapped", 0.85)]
    tot = sum(w for _, w in share)
    bk_td, bk_tb = [td_b0], [tb_b0]
    for nm, wgt in share:
        bk_td.append(bk_td[-1] + (td_b1 - td_b0) * wgt / tot)
        bk_tb.append(B.t0[nm] + dict(B.phases_d)[nm])
    base_comp = {nm: round(dict(B.phases_d)[nm] / ((td_b1 - td_b0) * wgt / tot), 2) for nm, wgt in share}
    say("base drum sequence after landing, x faster than the dart:", base_comp)

    def tb_post(td):
        return float(np.interp(td, bk_td, bk_tb))

    # ---------------- per-frame evaluation
    tf_all = np.arange(NF) / FPS
    frames = []
    for f, tfilm in enumerate(tf_all):
        if tfilm <= F_settle:
            td = float(np.interp(tfilm, film_of_td, tgrid))
            seg = "main"
        else:
            td = t_d_settle
            seg = "reel"
        fr = dict(f=f, tf=tfilm, td=td, seg=seg)
        # base state
        if seg == "main":
            if td < T_FIRE:
                S = B.state(td)
                fr["base"] = "timeline"
            elif td < T_FIRE + tau_fire_end:
                S, info = B.fire_state(S_cocked, td - T_FIRE, trace)
                fr["base"] = "fire:" + info["stage"]
            elif td < td_b0:
                S = B.state(tb_b0)
                fr["base"] = "flight"
            elif td < td_b1:
                S = B.state(tb_post(td))
                fr["base"] = "post-landing"
            else:
                S = B.state(tb_b1 + min(1.0, td - td_b1))
                fr["base"] = "anchor"
        else:
            u = tfilm - F_settle
            if tfilm < F_eng:
                tb = B.t0["tidy"] + 3.0 * (tfilm - F_tidy) / DUR_TIDY
            elif tfilm < F_tow:
                tb = B.t0["reel_engage"] + 6.5 * (tfilm - F_eng) / DUR_ENG
            else:
                tb = B.t0["tow"] + 6.0 * min(1.0, (tfilm - F_tow) / DUR_TOW)
            S = B.state(tb)
            fr["base"] = S["phase"]
            fr["tb"] = tb
        fr["S"] = S
        # dart clock -> dart pose
        if seg == "main":
            if td < T_FIRE:
                Wd = BM.to_world_m(dart_on_carriage(S))
                fr["dart"] = ("carriage", Wd[:3, 3], Wd[:3, :3])
            elif td < T_FIRE + tau_home:
                Wd = BM.to_world_m(dart_on_carriage(S))
                fr["dart"] = ("carriage", Wd[:3, 3], Wd[:3, :3])
            elif td < T_FIRE + tau_sep:
                # sliding off the cradle at the muzzle speed; the tip-off attitude offset (flight34's initial pitch / yaw
                # disturbance) builds up over the slide, about the CG, so the flight starts from the same pose
                s_ = V0 * (td - T_FIRE - tau_home)
                f_ = DM.smooth(s_ / slide)
                R0f = DM.R_flight(math.radians(inp["rail_deg"] + inp["dth"]), 0.0, math.radians(inp["dpsi"]))
                R_ = DM.q2R(DM.slerp(DM.R2q(R_rail), DM.R2q(R0f), f_))
                cg_ = Wd_home[:3, 3] + R_rail @ case.cg_d + u_dir * s_
                fr["dart"] = ("slide", cg_ - R_ @ case.cg_d, R_)
            elif td < t_d_hand:
                p, R = flight_pose(td - T_FIRE - tau_sep)
                fr["dart"] = ("flight", p, R)
            else:
                ts = td - t_d_hand
                fr["ts"] = ts
                p_s, q_s, leg, pl, pr, _ = sim.at(ts)
                p, R = SMAP.root(p_s, q_s, CGX)
                fr["dart"] = ("sim", p, R)
                fr["leg"], fr["pod"] = leg, 0.5 * (pl + pr)
        else:
            if tfilm < F_tow:
                ts = t_settle_end
            else:
                ts = t_settle_end + TOW_SIM * min(1.0, (tfilm - F_tow) / DUR_TOW)
            fr["ts"] = ts
            p_s, q_s, leg, pl, pr, _ = sim.at(ts)
            p, R = SMAP.root(p_s, q_s, CGX)
            fr["dart"] = ("sim", p, R)
            fr["leg"], fr["pod"] = leg, 0.5 * (pl + pr)
            fr["T_sim"] = float(np.interp(ts, sim.t, sim.col("T")))
        frames.append(fr)
    say("frames evaluated", round(time.time() - t_start, 1), "s")

    # ---------------- auger spin (sim drilling flags), joints
    tS = sim.t
    dL, dR, aL, aR = sim.col("drillL"), sim.col("drillR"), sim.col("anchorL"), sim.col("anchorR")
    run = ((dL > 0.5) & (aL < 0.5)) | ((dR > 0.5) & (aR < 0.5))
    w = DRILL_RPM * 6.0                                 # deg/s
    ramp = 0.15
    spin = np.zeros_like(tS)
    om = 0.0
    for i in range(1, len(tS)):
        dt = tS[i] - tS[i - 1]
        target = w if run[i] else 0.0
        om += np.clip(target - om, -w * dt / ramp, w * dt / ramp)
        spin[i] = spin[i - 1] + om * dt
    say("auger spin: drilling", round(float(tS[run][0]) if run.any() else -1, 2), "to", round(float(tS[run][-1]) if run.any() else -1, 2),
        "sim s,", round(spin[-1] / 360, 1), "turns at", DRILL_RPM, "rpm")

    # ---------------- tether rope
    J34 = DM.dart_parts()
    knot_d = np.array(J34["P"]["TIE"]["knot"], float) / 1000.0
    rope = None
    line_frames = []
    wraps_frames = []
    L_free = None
    spool_v = 0.0
    taut_reel_frame = None
    lens = []
    base_len0 = None
    roller_used = []
    t_prev = None

    def fixed_and_exit(S, knot_w, toward_w):
        """base path (world m) spool -> exit, the wrap index range, and the exit point; the line goes over the lip roller
        when the straight run from the pusher toward the rope would pass under its top (launch.line_path rule)"""
        pts, wr = B.line_fixed(S)
        v = B.values(S)
        pitch, travel = v["launch_rail"], v["launch_carriage"]
        Wr = B.rail_W(pitch)
        Wi = np.linalg.inv(Wr)
        pusher_rail = (Wi @ np.r_[pts[-1], 1.0])[:3]
        q = (Wi @ np.r_[np.asarray(toward_w) / 1.0, 1.0])[:3]
        ur, wr_, rp = B.roller(S, travel)
        extra = []
        used = False
        if q[0] > ur + rp:
            fz = (ur - pusher_rail[0]) / (q[0] - pusher_rail[0])
            if pusher_rail[2] + fz * (q[2] - pusher_rail[2]) < wr_ + rp + 1.5:
                a_d = min(90.0, tangent_angle((ur, wr_), rp, (q[0], q[2]), 1.0))
                for a in np.linspace(90.0, a_d, 5):
                    extra.append((Wr @ np.array([ur + rp * math.cos(math.radians(a)), 0.0, wr_ + rp * math.sin(math.radians(a)), 1.0]))[:3])
                used = True
        if extra:
            pts = np.vstack([pts, np.array(extra)])
        return BM.pts_world_m(pts), wr, used

    knot_prev = None
    for fr in frames:
        S = fr["S"]
        kind, p, R = fr["dart"]
        knot = p + R @ knot_d
        fr["knot"] = knot
        # the free rope starts at the base exit; aim the roller test at the rope's node next to the exit
        toward = knot if rope is None or rope.n() < 3 else rope.x[-2]
        toward_mm = (np.linalg.inv(BM.BASE_TO_WORLD) @ np.r_[toward, 1.0])[:3] * 1000.0
        fixed_w, wr, used = fixed_and_exit(S, knot, toward_mm)
        exit_w = fixed_w[-1]
        roller_used.append(used)
        base_len = float(np.linalg.norm(np.diff(fixed_w, axis=0), axis=1).sum())
        if base_len0 is None:
            base_len0 = base_len
        # rope clock: dart clock in the main segment, film-scaled base time in the reel segment
        if fr["seg"] == "main":
            t_now = fr["td"]
        else:
            t_now = t_d_settle + (fr["tb"] - B.t0["tidy"])          # the base's own clock (tidy 3 s, engage 6.5 s, tow 6 s)
        d_ee = float(np.linalg.norm(knot - exit_w))
        if rope is None:
            rope = Rope(knot, exit_w)
            L_free = rope.L
        else:
            dt_tot = t_now - t_prev
            nsub = max(1, int(math.ceil(dt_tot / 0.002)))
            for k in range(1, nsub + 1):
                a = k / nsub
                kn = knot_prev + (knot - knot_prev) * a
                ex = exit_prev + (exit_w - exit_prev) * a
                dts = dt_tot / nsub
                dd = float(np.linalg.norm(kn - ex))
                flying = fr["td"] < t_d_land if fr["seg"] == "main" else False
                if fr["seg"] == "main" and fr["td"] < T_FIRE + tau_sep:
                    L_new = dd                                   # on the carriage: knot to pusher hole
                    spool_v = 0.0
                elif flying:
                    # storage spool: pulled round by the line when it is tight, free to coast against the LIGHT drag
                    # (0.25 N on its 35 g effective line mass) when the dart slows faster: slack then runs out of the base
                    if spool_v <= 0.0:
                        spool_v = V0
                    need = dd + line_slack(dd, T_FLIGHT, W_LINE + W_DRAG_FLIGHT)
                    v_free = max(0.0, spool_v - SPOOL_DECEL * dts)
                    if L_free + v_free * dts >= need:
                        spool_v = v_free
                        L_new = L_free + v_free * dts
                    else:
                        spool_v = (need - L_free) / dts
                        L_new = need
                elif fr["seg"] == "main":
                    spool_v = min(spool_v, V_OVERRUN)
                    spool_v = max(0.0, spool_v - SPOOL_DECEL * dts)        # spool overruns against the LIGHT drag
                    L_new = max(L_free + spool_v * dts, dd * 1.0005)
                else:
                    if fr["tf"] < F_eng:                                     # tidy: the take-up pulls the slack in
                        L_new = max(dd + line_slack(dd, T_TIDY), L_free - TAKEUP_V * dts)
                    else:                                                     # REEL / tow: tight at the sim's tension
                        L_new = dd + line_slack(dd, max(T_TOW_MIN, fr.get("T_sim", T_TOW_MIN)))
                L_free = L_new
                rope.step(dts, kn, ex, L_free, iters=30 if nsub > 3 else 50)
        if fr["seg"] == "reel" and taut_reel_frame is None and L_free <= d_ee + 2.0 * line_slack(d_ee, T_TIDY):
            taut_reel_frame = fr["f"]
        t_prev, knot_prev, exit_prev = t_now, knot, exit_w
        lens.append((fr["f"], L_free, d_ee, base_len, rope.n()))
        # line points: base path (wraps replaced by their chord) + the free rope
        if wr is not None:
            i0, i1 = wr
            wraps_frames.append((fr["f"], resample_poly(fixed_w[i0:i1 + 1], K_WRAPS)))
            fixed_vis = np.vstack([fixed_w[:i0 + 1], fixed_w[i1:]])
        else:
            fixed_vis = fixed_w
        ki = dp_keep(fixed_vis, K_FIXED)
        fixed_pts = fixed_vis[ki] if len(ki) == K_FIXED else resample_poly(fixed_vis, K_FIXED, ki)
        free_pts = rope.points(K_FREE + 1)[1:]
        line_frames.append(np.vstack([fixed_pts, free_pts]))
        fr["line_out_m"] = (base_len + L_free) - (base_len0 + lens[0][1])
    say("rope done", round(time.time() - t_start, 1), "s; max free length", round(max(l[1] for l in lens), 3), "m; max nodes", max(l[4] for l in lens))

    # the paid-out length drives the storage spool (reel kin: spool angle from line_out_m)
    for fr in frames:
        fr["S"] = dict(fr["S"], line_out_m=float(fr["line_out_m"]))

    # ---------------- capstan take-in during the tow (line stretch + the dart's give in the soil, from the sim)
    tow_frames = [fr for fr in frames if fr["seg"] == "reel" and fr["tf"] >= F_tow]
    if tow_frames:
        Tcol = sim.col("T")
        d0 = None
        r_cap = (B.RL.LINE_R_CAP) / 1000.0
        for fr in tow_frames:
            Tn = float(np.interp(fr["ts"], sim.t, Tcol))
            d_ee = float(np.linalg.norm(fr["knot"] - line_frames[fr["f"]][K_FIXED - 1]))
            if d0 is None:
                d0, T0 = d_ee, Tn
            take = (Tn - T0) * d_ee / EA_LINE + (d0 - d_ee)
            cap_deg = math.degrees(take / r_cap)
            reel = cap_deg / (B.RL.Z_MAIN / B.RL.Z_CAPG)
            S = fr["S"]
            fr["S"] = dict(S, reel_deg=float(S.get("reel_deg", 0.0)) + reel, wiper_deg=float(S["wiper_deg"]) + reel)
            fr["tension"] = Tn
            fr["capstan_take_mm"] = take * 1000
        say("tow: tension", round(float(np.interp(tow_frames[-1]["ts"], sim.t, Tcol)), 1), "N at the end; capstan take-in",
            round(tow_frames[-1]["capstan_take_mm"], 1), "mm")

    # ---------------- base world transforms per frame
    say("posing base parts ...")
    rest = {nm: BM.to_world_m(pl) for nm, _, pl in B.parts}
    base_tracks = {nm: [] for nm, _, _ in B.parts}
    springs = {"l": [], "r": []}
    gear_check = []
    for fr in frames:
        S = fr["S"]
        JW = B.joint_world(S)
        Wp = B.world(S, JW)
        for nm, M in Wp.items():
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
    leg = np.array([max(0.0, fr.get("leg", 0.0)) for fr in frames])
    pod = np.array([max(0.0, fr.get("pod", 0.0)) for fr in frames])
    aug = np.array([float(np.interp(fr["ts"], tS, spin)) if "ts" in fr else 0.0 for fr in frames])

    # ---------------- events
    def film_at_td(td):
        return float(np.interp(td, tgrid, film_of_td))

    def fnum(tfilm):
        return int(round(tfilm * FPS))

    mv = sm["moves"]
    ev = [
        (fnum(B.t0["aim"]), "aim_start"), (fnum(B.t0["arm"]), "aimed"), (fnum(B.t0["cock"]), "cock_start"),
        (fnum(B.t0["cocked"]), "cocked"), (fnum(film_at_td(T_FIRE + t_rel)), "fire"),
        (fnum(film_at_td(T_FIRE + tau_home)), "carriage_home"), (fnum(film_at_td(T_FIRE + tau_sep)), "muzzle_exit"),
        (fnum(film_at_td(t_d_land)), "impact"),
        (fnum(film_at_td(td_b0)), "base_rail_low_start"),
        (fnum(film_at_td(bk_td[1])), "base_drum_to_neutral"),
        (fnum(film_at_td(bk_td[2])), "base_wrap_start"),
        (fnum(film_at_td(bk_td[3])), "base_wrap_done"),
        (fnum(film_at_td(bk_td[4])), "base_wrapped_park"),
    ]
    for m in mv:
        ev.append((fnum(film_at_td(t_d_hand + m["t"])), "righting_" + m["move"].replace(":", "_")))
    ev += [(fnum(film_at_td(t_d_hand + t_right_abs)), "righted"),
           (fnum(film_at_td(t_d_hand + ph_sim["anchor"])), "pods_deploy_start")]
    if run.any():
        ev.append((fnum(film_at_td(t_d_hand + float(tS[run][0]))), "drills_start"))
    ev += [(fnum(film_at_td(t_d_hand + t_anch_sim)), "anchored"), (fnum(F_tidy), "tidy_start"),
           (taut_reel_frame if taut_reel_frame is not None else fnum(F_eng), "reel_taut"),
           (fnum(F_eng), "reel_engage"), (fnum(F_tow), "tow_start"), (fnum(F_end), "tension_hold"), (NF - 1, "end")]
    ev.sort()

    # ---------------- assemble motion.json (sparse keys)
    fidx = list(range(NF))
    always = [e[0] for e in ev]
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
        r0 = rest[nm]
        if (np.abs(M - M[0]).max() < 1e-7) and np.abs(M[0] - r0).max() < 1e-7:
            continue                                        # never leaves its rest pose
        if np.abs(M - M[0]).max() < 1e-7:
            keep = [0]
            n_static_moved += 1
        else:
            keep = sorted(set(decimate(fidx, np.c_[loc / TOL_LOC, qs / TOL_Q], 1.0)))
        objects["base:" + nm] = dict(frames=[int(i) for i in keep], loc=np.round(loc[keep], 4).tolist(),
                                     quat=np.round(qs[keep], 5).tolist())
    keep = sorted(set(decimate(fidx, np.c_[dart_loc / TOL_LOC, dart_q / TOL_Q], 1.0, always)))
    objects["dart:root"] = dict(frames=keep, loc=np.round(dart_loc[keep], 4).tolist(), quat=np.round(dart_q[keep], 5).tolist())

    joints = {}
    PP = {p["name"]: p for p in J34["parts"]}

    def jkeys(vals):
        return sorted(set(decimate(fidx, np.asarray(vals)[:, None] / TOL_DEG, 1.0)))

    k = jkeys(leg)
    joints["dart:righting_leg"] = dict(frames=k, parent="dart:root", axis=[0.0, -1.0, 0.0],
                                       pivot=np.round(np.array(PP["righting_leg"]["pos"]) / 1000, 5).tolist(),
                                       deg=np.round(leg[k], 3).tolist())
    k = jkeys(pod)
    for nm in ("drill_leg_L", "drill_leg_R"):          # coupler_L/R, deploy_shaft, pod caps, motors follow the same deploy angle
        joints["dart:" + nm] = dict(frames=k, parent="dart:root", axis=[0.0, -1.0, 0.0],
                                    pivot=np.round(np.array(PP[nm]["pos"]) / 1000, 5).tolist(), deg=np.round(pod[k], 3).tolist())
    k = jkeys(aug)
    for nm, sg in (("drill_leg_L_auger", 1.0), ("drill_leg_R_auger", -1.0)):
        piv = PP[PP[nm]["parent"]]["pos"]
        joints["dart:" + nm] = dict(frames=k, parent="dart:" + PP[nm]["parent"], axis=[-1.0, 0.0, 0.0], pivot=[0.0, 0.0, 0.0],
                                    pivot_dart_frame_folded=np.round(np.array(piv) / 1000, 5).tolist(),
                                    deg=np.round(sg * aug[k], 2).tolist())

    # line (every frame where it moves) and the capstan wraps
    LF = np.array(line_frames)
    lk = sorted(set(decimate(fidx, LF.reshape(NF, -1) / TOL_PT, 1.0, always)))
    line = dict(frames=lk, points=np.round(LF[lk], 4).tolist(), k_base=K_FIXED, k_free=K_FREE)
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
        kk = sorted(set(decimate(fidx, np.c_[A_, C_] / TOL_LOC, 1.0)))
        sp_out[f"launch_spring_{side}"] = dict(frames=kk, anchor_pin=np.round(A_[kk], 4).tolist(), carriage_pin=np.round(C_[kk], 4).tolist(),
                                               length=np.round(np.linalg.norm(A_[kk] - C_[kk], axis=1), 4).tolist(),
                                               hook_l=0.012, coil_od=0.016, free_length_pins=round((B.n["spring_free"]) / 1000, 4))

    film_note = (f"30 fps, {NF} frames ({F_total:.1f} s). Slow motion is baked in: real time until 0.12 s before the shot, then "
                 f"ramping to 20x for the release, the {T_st * 1000:.0f} ms stroke and the slide off the cradle, easing to 3x in "
                 f"flight, 6x for the impact and bounce, back to real time 1.2 s after impact. The dart's righting and anchoring are "
                 f"real time; the base's drum sequence after landing is compressed to fit inside them (rail low over the top of the "
                 f"aim cycle {base_comp['rail_low']}x, drum to NEUTRAL {base_comp['neutral']}x, the 7-turn wrap {base_comp['wrap']}x, "
                 f"the park {base_comp['wrapped']}x). The reel segment runs the base tidy "
                 f"({3.0 / DUR_TIDY:.2f}x), the REEL engage ({6.5 / DUR_ENG:.2f}x) and the tow ({6.0 / DUR_TOW:.2f}x, sim tow "
                 f"{TOW_SIM:.0f} s: tension to 15 N, hold, step to 20 N), then holds. Keys are sparse: interpolate LINEARLY "
                 f"(loc, quaternion components, joint degrees, line points) between keys; tolerances {TOL_LOC} m, {TOL_Q} quat, "
                 f"{TOL_DEG} deg, line {TOL_PT} m.")
    out = dict(
        fps=FPS, frames=NF, film_time_note=film_note,
        units="metres, world: Z up, +X launch direction, origin = centre of the board top (z 0)",
        base_to_world=BM.BASE_TO_WORLD.tolist(),
        base_to_world_note=("world_m = base_to_world @ S(0.001) @ M_base_mm, where M_base_mm is a part's base-frame transform: "
                            "asm.json 'place' (the part's local mesh frame in the base frame with every joint at 0) composed with "
                            "its joint chain, JW(joint) @ place (tools.assembly.world). objects[base:*] give exactly that per frame "
                            "(set it as matrix_world). Frame 0 is the load state (tools.timeline state_at(P, 0): rail 6.02 deg, wiper "
                            "0, drum LAUNCH) = models.py's rest pose; parts that never move and sit at base_to_world @ place are "
                            "omitted, parts that never move but sit off it (jointed, e.g. the rail at 6 deg) have one key. Base frame "
                            "(BASE_SPEC section 2): origin on the wiper / main shaft axis, deck top z -165 mm, deck underside -173 mm: "
                            "base_to_world puts the deck underside on the board top (z 0) and centres the deck footprint (with the "
                            "D50 extension tile) on the origin; no rotation (+X launch azimuth)."),
        objects=objects, joints=joints, line=line, line_wraps=line_wraps, springs=sp_out,
        events=[dict(frame=int(f), name=nm) for f, nm in ev],
        sources=dict(base="base/cad/parts (drum, frame, launch, reel, sensors: joints, kin, line_path) posed with base/out/base/asm.json",
                     shot="base/physics launcher.py chain-spring trace at 34 deg via kedge/dart_model.launch_trace",
                     flight="kedge/flight34.py strip aero + 0.25 N payout drag on the tail tie, tip-off 0.4 rad/s nose down",
                     landing="kedge/sim_righting simCore + v34_check/run34_shared.mjs SimV34 (rigid axle, tail tie), loam, started from the flight's end state (pose, velocity, angular velocity), no blend",
                     launch_inputs="search_landing.py seed 8 run 28 (search/search_results_recheck_search_results_s8.json)",
                     line="render/motion/rope.py on the base line_path (reel.line_spans + launch.line_path)"),
        dart=dict(launch_inputs=dict(PICK, rail_deg_achieved=round(float(B.aim_end_pitch), 4)), handover=mism, knot_dart_frame=knot_d.tolist(), cg_station_m=CGX,
                  auger_rpm=DRILL_RPM, auger_note="drill_leg_L_auger (right-hand flight) turns + about its tip direction (local -x), "
                  "drill_leg_R_auger (left-hand) the other way: both screw in", sim_to_world="world = C_SIM @ sim (x, y up, z right) -> (X, -z, y), same origin"),
    )
    path = os.path.join(HERE, "motion.json")
    with open(path, "w") as fh:
        json.dump(out, fh, separators=(",", ":"))
    say("wrote", path, round(os.path.getsize(path) / 1e6, 2), "MB;", len(objects), "objects (", n_static_moved, "static off rest )")

    # ---------------- checks and plots
    ctx = dict(B=B, frames=frames, tf=tf_all, dart_loc=dart_loc, dart_R=dart_R, leg=leg, pod=pod, aug=aug, lens=lens,
               gear=np.array(gear_check), fl=fl, sim=sim, SMAP=SMAP, ev=ev, t_d_hand=t_d_hand, tgrid=tgrid, film_of_td=film_of_td,
               springs=springs, LF=LF, mism=mism, roller=roller_used, J34=J34, knot_d=knot_d, rate=rate, T_FIRE=T_FIRE,
               tau_sep=tau_sep, k_base=base_comp, CGX=CGX, base_tracks=base_tracks, wraps=wraps_frames)
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
