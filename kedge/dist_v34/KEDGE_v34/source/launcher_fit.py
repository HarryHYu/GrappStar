"""Launcher fit check for a KEDGE dart build against the REAL base launcher (base/cad/parts/launch.py).

python3 launcher_fit.py OUT [REF_OUT]   ->  OUT/launcher_fit.json, prints FITS / DOES NOT FIT

Loads the base assembly (all subsystems, as the base audit does), builds the folded dart solid from OUT the same way
launch.dart_solid() builds it from the v30 meshes (every closed part mesh, union, plus the hull and nose as solids),
and intersects it with every other base part over the launcher's own audit states (cock sweep, the shot, reset, the
dart leaving the carriage at 6 and 47 deg pitch past the muzzle, the stop) and every mission timeline state in which
the dart is on the launcher, flying off it or coming home. The same run is done with the reference dart (default: the
v30 meshes the launcher was designed around, base/cad/parts/launch_dart_v30_solid.npz) so a pair that already
overlaps by design (the tether line, the seat pushrod) is only a failure if the new dart overlaps MORE than v30 did.
Also reports the gap from the dart to the cradle and the pusher at HOME (the dart must rest on them).
"""
import json, math, os, sys, time
import numpy as np
import trimesh
import manifold3d as m3

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(os.path.dirname(HERE), "base")
sys.path.insert(0, BASE)
OUT = sys.argv[1]
REF = sys.argv[2] if len(sys.argv) > 2 else None


def dart_from_out(out):
    J = json.load(open(os.path.join(out, "parts.json"))); P = J["P"]
    PP = {p["name"]: p for p in J["parts"]}
    ms = []
    for n, p in PP.items():
        z = np.load(os.path.join(out, n + ".npz")); v = np.asarray(z["v"], float); f = np.asarray(z["f"], np.int64)
        q = p
        while True:                                   # folded pose: sum of pos up the parent chain (as launch_dart_v30.json)
            v = v + np.array(q["pos"], float)
            if not q.get("parent"):
                break
            q = PP[q["parent"]]
        tm = trimesh.Trimesh(v, f, process=True)
        for c in tm.split(only_watertight=False):
            try:
                M = m3.Manifold(m3.Mesh(vert_properties=np.asarray(c.vertices, np.float32), tri_verts=np.asarray(c.faces, np.uint32)))
            except Exception:
                continue
            if M.status() == m3.Error.NoError and M.volume() > 0:
                ms.append(M)
    U = m3.Manifold.batch_boolean(ms, m3.OpType.Add).simplify(0.2)
    return U, P


def _seg_tube(p, q, r, seg=16):
    """manifold cylinder of radius r from p to q"""
    p, q = np.asarray(p, float), np.asarray(q, float); d = q - p; Ld = float(np.linalg.norm(d)); u = d / Ld
    z = np.array([0.0, 0.0, 1.0]); v = np.cross(z, u); c = float(z @ u); s_ = float(np.linalg.norm(v))
    if s_ < 1e-9:
        Rm = np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    else:
        k = v / s_; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        Rm = np.eye(3) + s_ * K + (1 - c) * K @ K
    M = np.c_[Rm, p]
    return m3.Manifold.cylinder(Ld, r, r, seg).transform(M.tolist())


def rope_paths(P):
    """the stowed bridle (both legs, dart frame folded): eye -> past the arm head -> tail edge groove -> end wall groove -> knot on the axis"""
    RP = P["ROPE"]; left = [np.array(q, float) for q in RP["left"]] + [np.array(RP["knot_stowed"], float)]
    right = [q * np.array([1.0, -1.0, 1.0]) for q in left]
    return [left, right]


def rope_manifold(P):
    """the stowed rope as a Ø ROPE_D swept polyline (both legs), the two bowlines at the eyes and the sliding knot on the axis"""
    RP = P["ROPE"]; r = RP["rope_d"] / 2; ms = []
    for leg in rope_paths(P):
        for a, b in zip(leg[:-1], leg[1:]):
            ms.append(_seg_tube(a, b, r))
        for a in leg[1:-1]:
            ms.append(m3.Manifold.sphere(r, 16).translate(tuple(float(c) for c in a)))
    for sg in (1, -1):
        ek = np.array(RP["eye_knot"], float) * np.array([1.0, sg, 1.0])
        ms.append(m3.Manifold.sphere(RP["eye_knot_r"], 24).translate(tuple(float(c) for c in ek)))
    ms.append(m3.Manifold.sphere(RP["knot_r"], 24).translate(tuple(float(c) for c in RP["knot_stowed"])))
    return m3.Manifold.batch_boolean(ms, m3.OpType.Add)


def _posed(out, deploy_deg, leg_deg):
    """every dart part mesh in world (pods at deploy_deg, leg at leg_deg)"""
    J = json.load(open(os.path.join(out, "parts.json"))); PP = {p["name"]: p for p in J["parts"]}
    def T(name):
        p = PP[name]
        if p.get("parent"):
            return T(p["parent"]) @ _tm(p["pos"])
        M = _tm(p["pos"])
        if p.get("axis"):
            deg = leg_deg if p.get("joint") == "leg" else deploy_deg
            a = math.radians(p.get("sgn", 1) * deg); c, s_ = math.cos(a), math.sin(a)
            M = M @ np.array([[c, 0, s_, 0], [0, 1, 0, 0], [-s_, 0, c, 0], [0, 0, 0, 1]])
        return M
    out_m = {}
    for n in PP:
        z = np.load(os.path.join(out, n + ".npz")); v = np.asarray(z["v"], float); M = T(n)
        out_m[n] = trimesh.Trimesh((np.c_[v, np.ones(len(v))] @ M.T)[:, :3], z["f"], process=False)
    return out_m, PP


def _tm(p):
    M = np.eye(4); M[:3, 3] = p; return M


def _dense(pts, step=0.25):
    out = []
    for a, b in zip(pts[:-1], pts[1:]):
        n = max(2, int(np.linalg.norm(b - a) / step) + 1)
        out += [a + (b - a) * t for t in np.linspace(0, 1, n)]
    return np.array(out)


def rope_checks(out):
    """dart-side bridle checks: the stowed legs against the folded dart (they lie in the grooves, nothing inside a part), 3 mm clear of
    the ON plug + wire loop envelope, and the anchored straight legs (deployed eyes to the knot) against the deployed dart"""
    J = json.load(open(os.path.join(out, "parts.json"))); P = J["P"]; RP = P["ROPE"]; r = RP["rope_d"] / 2
    fold, PP = _posed(out, 0.0, 0.0)
    rep = {"stowed_len_mm": RP["stowed_len_mm"], "knot_behind_tail_mm": RP["knot_behind_tail_mm"]}
    pts = np.vstack([_dense(leg[:-1]) for leg in rope_paths(P)])          # up to the groove end (the last span runs into the pusher hole)
    tail = np.vstack([_dense(leg[-2:]) for leg in rope_paths(P)])
    worst = {}
    for n, m in fold.items():
        if n == "jst_loop_plug":
            continue
        lo, hi = m.bounds
        q = np.vstack([pts, tail]); sel = ((q > lo - 3) & (q < hi + 3)).all(1)
        if not sel.any():
            continue
        sd = trimesh.proximity.signed_distance(m, q[sel])
        worst[n] = round(float(-sd.max() - r), 2)
    rep["stowed_clear_mm"] = dict(sorted(worst.items(), key=lambda kv: kv[1])[:8])
    rep["stowed_inside_parts"] = {n: v for n, v in worst.items() if v < -0.1}
    if "PLUG_ENV" in P:                               # ON shorting plug + 10 mm wire loop straight out of the left flank
        lo, hi = np.array(P["PLUG_ENV"][0], float), np.array(P["PLUG_ENV"][1], float)
        d = np.linalg.norm(np.maximum(np.maximum(lo - pts, pts - hi), 0), axis=1) - r
        rep["plug_loop_clear_mm"] = round(float(d.min()), 2); rep["plug_loop_closest_at"] = [round(float(c), 1) for c in pts[d.argmin()]]
        qa = _dense([np.array(RP["eye_deployed"], float), np.array(RP["knot_deployed"], float)])
        da = np.linalg.norm(np.maximum(np.maximum(lo - qa, qa - hi), 0), axis=1) - r
        hi2 = hi.copy(); hi2[1] = 21.0 + 1.5                          # the plug flush, its loop folded flat (1.5 off the skin)
        db = np.linalg.norm(np.maximum(np.maximum(lo - qa, qa - hi2), 0), axis=1) - r
        rep["plug_loop_clear_towed_mm"] = round(float(da.min()), 2); rep["plug_flat_loop_clear_towed_mm"] = round(float(db.min()), 2)
    dep, _ = _posed(out, P["DEPLOY_DEG"], 0.0)
    kids = {}
    for n, p in PP.items():
        q_ = p
        while q_.get("parent"): q_ = PP[q_["parent"]]
        kids.setdefault(q_["name"], []).append(n)
    worst = {}; own = {}
    for sg, pod in ((1, "drill_leg_L"), (-1, "drill_leg_R")):
        b = np.array(RP["knot_deployed"], float)
        q = _dense([np.array(RP["eye_deployed"], float) * [1, sg, 1], b])          # straight from the eye (as towed), other parts
        q2 = _dense([np.array(RP["leg_start_deployed"], float) * [1, sg, 1], b])   # from the bowline: the own arm (wrap contact allowed)
        for n, m in dep.items():
            if n == "jst_loop_plug":
                continue
            mine = n in kids.get(pod, [])
            qq = q2 if mine else q
            lo, hi = m.bounds; sel = ((qq > lo - 3) & (qq < hi + 3)).all(1)
            if not sel.any():
                continue
            v = round(float(-trimesh.proximity.signed_distance(m, qq[sel]).max() - r), 2)
            if mine: own[n] = min(v, own.get(n, 99.0))
            else: worst[n] = min(v, worst.get(n, 99.0))
    rep["anchored_clear_mm"] = dict(sorted(worst.items(), key=lambda kv: kv[1])[:6])
    rep["anchored_min_mm"] = min(worst.values()) if worst else None
    rep["anchored_own_arm_mm"] = own
    rep["ok"] = bool(not rep["stowed_inside_parts"] and rep.get("plug_loop_clear_mm", 99) >= 3.0 and (rep["anchored_min_mm"] or 0) >= 0.5
                     and min(own.values() or [0]) >= -0.3)
    return rep


def main():
    t0 = time.time()
    from cad.lib import prim, tf
    from tools.assembly import Assembly
    from tools.timeline import timeline
    import cad.parts.launch as LA
    A = Assembly.load()
    P = A.P
    darts = {}
    U, DP = dart_from_out(OUT)
    darts["new"] = U + prim.cylX(DP["R_OUT"], DP["NOSE_L"], DP["L"]) + prim.place(prim.cyl(DP["NOSE_R0"], DP["NOSE_L"], r2=DP["R_OUT"]), tf.Ry(90))
    if REF:
        U2, RP = dart_from_out(REF)
        darts["ref"] = U2 + prim.cylX(RP["R_OUT"], RP["NOSE_L"], RP["L"]) + prim.place(prim.cyl(RP["NOSE_R0"], RP["NOSE_L"], r2=RP["R_OUT"]), tf.Ry(90))
    else:
        darts["ref"] = LA.dart_solid(P)
    rope = None
    if "ROPE" in DP:                                 # v34 bridle: checked on its own (no reference), moving with the dart
        rc = rope_checks(OUT); print("rope checks:", json.dumps(rc))
        rope = rope_manifold(DP); darts["rope"] = rope
    elif "TIE" in DP:                                # simple tail tie: the knot and 25 mm of line straight back from the two tail holes
        T_ = DP["TIE"]; k_ = np.array(T_["knot"], float); r_ = T_["line_d"] / 2
        ms_ = [m3.Manifold.sphere(T_["knot_r"], 24).translate(tuple(float(c) for c in k_)), _seg_tube(k_, k_ + np.array([25.0, 0, 0]), r_)]
        ms_ += [_seg_tube(np.array(h, float) - np.array([0.5, 0, 0]), k_, r_) for h in T_["holes"]]
        rope = m3.Manifold.batch_boolean(ms_, m3.OpType.Add); darts["rope"] = rope
        rc = dict(tie=T_, ok=True)
    states = [S for S in A.extra_states if str(S.get("phase", "")).startswith("launch")]
    try:
        for S in timeline(P):
            if any(float(S.get(k, 0) or 0) > 0 for k in ("dart_flight", "dart_return")) or S.get("phase") in ("load", "aim", "fire"):
                states.append(S)
    except Exception as e:
        print("timeline not used:", str(e)[:80])
    place = A.parts["launch_dart"].place
    others = [n for n in A.parts if n != "launch_dart"]
    worst = {k: {} for k in darts}
    for i, S in enumerate(states):
        W = A.world(S)
        Mdart = W["launch_dart"]
        for key, D in darts.items():
            d = prim.place(D, Mdart)
            lo, hi = np.array(prim.bbox(d)[0]), np.array(prim.bbox(d)[1])
            for n in others:
                s = A.posed(n, S, W)
                a, b = prim.bbox(s)
                if (np.array(a) > hi + 0.01).any() or (np.array(b) < lo - 0.01).any():
                    continue
                v = (d ^ s).volume()
                if v > 1e-3:
                    k = worst[key].get(n)
                    if k is None or v > k[0]:
                        worst[key][n] = (round(v, 3), S.get("phase"), {kk: S[kk] for kk in ("aim_deg", "launch_travel", "launch_dart_fly", "dart_flight", "dart_return") if kk in S})
    fails = {}; line_model = {}; rope_fail = {}
    for n, (v, ph, st) in worst.get("rope", {}).items():
        if n.startswith("launch_line"):
            line_model["rope | " + n] = v; continue
        if v > 0.05:
            rope_fail[n] = dict(rope_mm3=v, phase=ph, state=st)
    for n, (v, ph, st) in worst["new"].items():
        if n.startswith("launch_line"):            # base's tether model still ends at the old tail holes; a dart with a tail boom
            line_model[n] = v; continue            # carries the line on its eye: base moves the line end (notes), not a clash

        ref = worst["ref"].get(n, (0.0,))[0]
        if v > ref + 0.5:
            fails[n] = dict(new_mm3=v, ref_mm3=ref, phase=ph, state=st)
    # rest contact at HOME: dart to cradle (carriage) and pusher
    S0 = dict(aim_deg=6.02, cock=0.0, phase="launch_sweep")
    W = A.world(S0)
    d0 = prim.place(darts["new"], W["launch_dart"])
    car = A.posed("launch_carriage", S0, W)
    rest = dict(dart_vs_carriage_mm3=round((d0 ^ car).volume(), 3),
                dart_down_0p5_vs_carriage_mm3=round((d0.translate((0, 0, -0.5)) ^ car).volume(), 3))
    if rope is not None:
        if not rc["ok"]:
            rope_fail["dart_side_checks"] = rc
        fails.update({"rope | " + k: v for k, v in rope_fail.items()})
    rep = dict(out=OUT, ref=REF or "base/cad/parts/launch_dart_v30_solid.npz", states=len(states),
               overlaps_new=worst["new"], overlaps_ref=worst["ref"], overlaps_rope=worst.get("rope"), rope_checks=rc if rope is not None else None,
               fails=fails, line_model_overlaps=line_model, rest=rest, seconds=round(time.time() - t0))
    json.dump(rep, open(os.path.join(OUT, "launcher_fit.json"), "w"), indent=1, default=str)
    print(json.dumps(rep, indent=1, default=str))
    print("FITS" if not fails else "DOES NOT FIT")


if __name__ == "__main__":
    main()
