"""Debug: overlap volume and its bounding box between a posed part family and fixed parts.
python3 ovl.py OUT ROOT DEG [fixed1,fixed2,...] [child]"""
import json, math, sys, numpy as np, manifold3d as mf
from trimesh.transformations import rotation_matrix as RM, translation_matrix as TM
OUT, ROOT, DEG = sys.argv[1], sys.argv[2], float(sys.argv[3])
J = json.load(open(OUT + "/parts.json")); PP = {p["name"]: p for p in J["parts"]}
fixed = sys.argv[4].split(",") if len(sys.argv) > 4 else [n for n, p in PP.items() if not p["axis"] and not p.get("parent")]
def load(n, T=np.eye(4)):
    z = np.load(f"{OUT}/{n}.npz"); v = np.c_[z["v"], np.ones(len(z["v"]))] @ T.T
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(v[:, :3], np.float32), tri_verts=np.asarray(z["f"], np.uint32)))
kids = {}
for n, p in PP.items():
    if p.get("parent"): kids.setdefault(p["parent"], []).append(n)
def fam(r):
    o = [r]
    for k in kids.get(r, []): o += fam(k)
    return o
p = PP[ROOT]
T = TM(p["pos"]) @ RM(math.radians(p.get("sgn", 1) * DEG), [0, 1, 0]) if p.get("axis") else np.eye(4)
for c in fam(ROOT):
    if len(sys.argv) > 5 and c != sys.argv[5]: continue
    a = load(c, T)
    for f in fixed:
        if f == c: continue
        i = a ^ load(f)
        v = i.volume()
        if v > 1e-3:
            bb = i.bounding_box()
            print(f"{c:22s} x {f:14s} {v:8.2f} mm3  bbox {np.round(bb[:3],1)} .. {np.round(bb[3:],1)}")
