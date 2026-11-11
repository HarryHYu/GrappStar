"""Sanity checks of the motion: gear senses for the carriage motion, nose-first flight, drills down into the ground,
no part below the ground at rest / landing, base parts above the board, the line above the ground."""
from __future__ import annotations

import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))


def _RotY(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def dart_meshes(J):
    out = {}
    for p in J["parts"]:
        z = np.load(os.path.join(ROOT, "kedge", "out34", p["name"] + ".npz"))
        v = np.asarray(z["v"], float) / 1000.0
        if len(v) > 4000:
            v = v[:: max(1, len(v) // 4000)]
        out[p["name"]] = v
    return out


def pose_dart(J, meshes, root_p, root_R, leg, pod):
    PP = {p["name"]: p for p in J["parts"]}

    def T(name):
        p = PP[name]
        pos = np.array(p["pos"], float) / 1000.0
        if p.get("parent"):
            Rp, tp = T(p["parent"])
            return Rp, tp + Rp @ pos
        R = np.eye(3)
        if p.get("axis"):
            deg = leg if p.get("joint") == "leg" else pod
            R = _RotY(math.radians(p.get("sgn", 1) * deg))
        return R, pos
    out = {}
    for n, v in meshes.items():
        R, t = T(n)
        out[n] = (v @ R.T + t) @ root_R.T + root_p
    return out


def run(ctx, say):
    res = {}
    frames, gear = ctx["frames"], ctx["gear"]
    B = ctx["B"]
    # ---- gear senses while cocking: rack rolls on the big gear, drive gear meshes it the other way
    ck = np.array([fr["S"].get("phase") == "cock" for fr in frames])
    g = gear[ck]
    d_tr, d_g = np.diff(g[:, 1]), np.diff(g[:, 2])
    d_drive_rel = np.diff(g[:, 3] + g[:, 4])
    mm = np.abs(d_tr) > 1e-6
    roll = float(np.median(np.radians(d_g[mm]) * B.g["r_big"] / d_tr[mm]))
    mesh = float(np.median(d_drive_rel[mm] / d_g[mm]))
    wip = np.array([fr["S"]["wiper_deg"] for fr in frames])[ck]
    res["gears"] = dict(rack_rolls_on_big_gear=round(roll, 4), expect_rack=-1.0, drive_per_big_gear=round(mesh, 4),
                        expect_drive=-round(B.ratio, 4), wiper_forward_while_cocking=bool(np.all(np.diff(wip) >= -1e-9)),
                        carriage_back_while_cocking=bool(np.all(d_tr >= -1e-9)),
                        ok=bool(abs(roll + 1.0) < 0.03 and abs(mesh + B.ratio) < 0.03))
    say("check gears:", res["gears"])
    # ---- flight: nose first (angle of attack under 45 deg all the way)
    fl = ctx["fl"]
    aoa = np.degrees(fl[:, 3] - np.arctan2(fl[:, 5], fl[:, 4]))
    res["flight"] = dict(aoa_max_deg=round(float(np.abs(aoa).max()), 1), aoa_at_handover_deg=round(float(aoa[-1]), 1),
                         nose_first=bool(np.abs(aoa).max() < 45), time_s=round(float(fl[-1, 0]), 3))
    say("check flight:", res["flight"])
    # ---- impact / rest positions
    J = ctx["J34"]
    ev = dict((nm, f) for f, nm in ctx["ev"])
    loc = ctx["dart_loc"]
    fi = ev["impact"]
    res["impact_nose_xyz"] = np.round(loc[fi], 3).tolist()
    res["rest_nose_xyz"] = np.round(loc[-1], 3).tolist()
    say("check impact nose at", res["impact_nose_xyz"], "rest", res["rest_nose_xyz"])
    # ---- drills: tips below the ground, deploy moves the tips down
    PP = {p["name"]: p for p in J["parts"]}
    tip_local = np.array([-J["P"]["POD_TIP"], 0.0, 0.0]) / 1000.0
    out = {}
    for nm in ("drill_leg_L", "drill_leg_R"):
        piv = np.array(PP[nm]["pos"]) / 1000.0
        zs = []
        for f in (ev["pods_deploy_start"], ev["anchored"], len(frames) - 1):
            p, R = frames[f]["dart"][1], frames[f]["dart"][2]
            deg = ctx["pod"][f]
            tip = p + R @ (piv + _RotY(math.radians(-deg)) @ tip_local)
            zs.append(round(float(tip[2]) * 1000, 1))
        out[nm] = dict(tip_z_mm_deploy_start_anchored_end=zs, into_ground=bool(zs[1] < -20 and zs[2] < -20))
    res["drills"] = out
    say("check drills:", out)
    # ---- dart meshes vs the ground after landing (augers / motors / shafts excluded once drilling)
    meshes = dart_meshes(J)
    drill_parts = {n for n in meshes if n.startswith("drill_leg_") and n not in ("drill_leg_L", "drill_leg_R")}
    worst = (1.0, None, None)
    rest_min = None
    t_imp = ev["impact"]
    sel = list(range(t_imp + 20, len(frames), 3))
    for f in sel:
        fr = frames[f]
        p, R = fr["dart"][1], fr["dart"][2]
        posed = pose_dart(J, meshes, p, R, ctx["leg"][f], ctx["pod"][f])
        for n, v in posed.items():
            if ctx["pod"][f] > 12.0 and n in drill_parts:
                continue
            z = float(v[:, 2].min())
            if z < worst[0]:
                worst = (z, n, f)
        if f >= len(frames) - 3:
            rest_min = min(float(v[:, 2].min()) for n, v in posed.items() if n not in drill_parts)
    res["dart_vs_ground"] = dict(min_z_mm=round(worst[0] * 1000, 2), part=worst[1], frame=worst[2],
                                 rest_min_z_mm=round(rest_min * 1000, 2) if rest_min is not None else None,
                                 note="sim contact is soft and its colliders are a capsule hull, cone nose and capsule pods/leg: a few mm of "
                                      "sink into the soil is expected; drills / motors / augers excluded once the pods are past 12 deg")
    say("check dart vs ground:", res["dart_vs_ground"])
    # ---- base parts above the board (z 0), moving parts, every 6th frame
    base_min = (1.0, None, None)
    mdir = os.path.join(ROOT, "base", "out", "base", "meshes")
    tr = ctx["base_tracks"]
    for nm, track in tr.items():
        M = np.array(track)
        if np.abs(M - M[0]).max() < 1e-7:
            continue
        fp = os.path.join(mdir, nm + ".npz")
        if not os.path.exists(fp):
            continue
        v = np.asarray(np.load(fp)["v"], float) / 1000.0
        if len(v) > 3000:
            v = v[:: len(v) // 3000]
        # meshes are in the part's local frame (asm.export): world = M @ v (M already in metres)
        for f in range(0, len(M), 6):
            z = float((v @ M[f][:3, :3].T + M[f][:3, 3])[:, 2].min())
            if z < base_min[0]:
                base_min = (z, nm, f)
    res["base_moving_parts_min_z_mm"] = dict(z=round(base_min[0] * 1000, 2), part=base_min[1], frame=base_min[2])
    say("check base moving parts min z:", res["base_moving_parts_min_z_mm"])
    # ---- line above ground, dart in flight clear of the base
    LF = ctx["LF"]
    res["line_min_z_mm"] = round(float(LF[:, :, 2].min()) * 1000, 2)
    say("check line min z mm:", res["line_min_z_mm"])
    res["handover_mismatch"] = ctx["mism"]
    return res
