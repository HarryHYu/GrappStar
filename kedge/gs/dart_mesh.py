"""KEDGE v30 for the ground station sim: body + movers (kick, pods, couplers) relative to their pivots.
python dart_mesh.py ../out30 out"""
import json, sys, numpy as np, trimesh
OUT, DST = sys.argv[1], sys.argv[2]
J = json.load(open(OUT + "/parts.json")); PP = {p["name"]: p for p in J["parts"]}
def root(n):
    while PP[n].get("parent"): n = PP[n]["parent"]
    return n
GROUPS = {"kick_leg": "dart_kick", "drill_leg_L": "dart_drillL", "drill_leg_R": "dart_drillR", "coupler_L": "dart_cpl", "coupler_R": "dart_cpl", "deploy_shaft": "dart_cpl"}
acc, piv = {}, {}
for n, p in PP.items():
    z = np.load(f"{OUT}/{n}.npz"); v = z["v"].astype(float); f = z["f"]
    r = root(n); g = GROUPS.get(r)
    if g is None:
        g = "dart_body_" + ("hull" if p["group"] == "hull" else "elec")
        if PP[r]["axis"]: v = v + np.array(PP[r]["pos"])
    else:
        piv[g] = PP[r]["pos"]; g = g + ("_aug" if n.endswith("_auger") else "")
        if n.endswith("_auger"): piv[g] = PP[r]["pos"]
    acc.setdefault(g, []).append(trimesh.Trimesh(v, f, process=False))
meta = {}
for g, ms in acc.items():
    m = trimesh.util.concatenate(ms)
    np.savez_compressed(f"{DST}/{g}.npz", v=m.vertices.astype(np.float32), f=m.faces.astype(np.int32))
    meta[g] = dict(pivot=piv.get(g), nverts=len(m.vertices))
P = J["P"]
meta["_P"] = dict(KICK_DEG=P["KICK_DEG"], DEPLOY_DEG=P["DEPLOY_DEG"], L=P["L"], ground=J["ground"])
json.dump(meta, open(f"{DST}/dart_groups.json", "w"), indent=1)
print(json.dumps(meta, indent=0))
