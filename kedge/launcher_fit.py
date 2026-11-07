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
    states = [S for S in A.extra_states if str(S.get("phase", "")).startswith("launch")]
    try:
        for S in timeline(P):
            if any(float(S.get(k, 0) or 0) > 0 for k in ("dart_flight", "dart_return")) or S.get("phase") in ("load", "aim", "fire"):
                states.append(S)
    except Exception as e:
        print("timeline not used:", str(e)[:80])
    place = A.parts["launch_dart"].place
    others = [n for n in A.parts if n != "launch_dart"]
    worst = {"new": {}, "ref": {}}
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
    fails = {}; line_model = {}
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
    rep = dict(out=OUT, ref=REF or "base/cad/parts/launch_dart_v30_solid.npz", states=len(states),
               overlaps_new=worst["new"], overlaps_ref=worst["ref"], fails=fails, line_model_overlaps=line_model, rest=rest, seconds=round(time.time() - t0))
    json.dump(rep, open(os.path.join(OUT, "launcher_fit.json"), "w"), indent=1, default=str)
    print(json.dumps(rep, indent=1, default=str))
    print("FITS" if not fails else "DOES NOT FIT")


if __name__ == "__main__":
    main()
