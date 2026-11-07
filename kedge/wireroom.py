"""Wire room check: free cross-section inside the hull bay along x (bay circle minus every part, legs folded) in the two
harness passages, against the bundle the v31 cut list puts through each station.
  KEEL passage: |y| <= 4.6, z -17 .. -8.6 (under the cells, discs 2..4, saddles, board-bay floor, sled)
  TOP passage : |y| <= 11, z >= 9 (over the VTX, the disc top notches, the board bay roof)
python3 wireroom.py out31 [pack_v31]  -> OUT/wireroom.json, prints the worst fill per passage (bundle area / free area).
A fill under 0.5 is comfortable for loose PVC/silicone hook-up wire; 0.5..0.7 tight; above 0.7 it will not go in."""
import json, sys, math, numpy as np, trimesh
from shapely.geometry import Polygon, box, Point
from shapely.ops import unary_union, polygonize
OUT = sys.argv[1]; PK = sys.argv[2] if len(sys.argv) > 2 else OUT.replace("out", "pack_v")
J = json.load(open(OUT + "/parts.json")); P = J["P"]; PP = {p["name"]: p for p in J["parts"]}
DIA = {"22 AWG": 1.6, "24 AWG": 1.4, "26 AWG": 1.2, "28 AWG": 1.0}      # insulated outside diameter, PVC hook-up wire
def wire_area(g, silicone=False): d = DIA[g] + (0.15 if silicone else 0); return math.pi * d * d / 4
# every part in the folded pose
mesh = {}
for n, p in PP.items():
    z = np.load(f"{OUT}/{n}.npz"); v = np.asarray(z["v"], float); q = p
    while True:
        v = v + np.array(q["pos"], float)
        if not q.get("parent"): break
        q = PP[q["parent"]]
    mesh[n] = trimesh.Trimesh(v, z["f"], process=False)
RB = P["R_BAY"]; KZ = P["KEEL_Z"]
bay = Point(0, 0).buffer(RB, 128)
KEEL = box(-4.6, KZ, 4.6, KZ + 8.4) & bay
TOP = box(-11, 9.0, 11, RB) & bay
def section(m, x):
    if not (m.bounds[0][0] < x < m.bounds[1][0]): return None
    sec = m.section(plane_normal=[1, 0, 0], plane_origin=[x, 0, 0])
    if sec is None: return None
    T = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, -x], [0, 0, 0, 1]], float)   # (x, y, z) -> 2D (y, z)
    try:
        p2, _ = sec.to_planar(to_2D=T, check=False)
        polys = [pg for pg in p2.polygons_full if pg is not None and pg.area > 1e-4]
    except Exception:
        return None
    return unary_union(polys).buffer(0) if polys else None
xs = np.arange(P["NOSE_L"] + 1, P["X_TAILSP"][0] - 0.5, 1.0)
free = {}
for x in xs:
    occ = [s for n in mesh if (s := section(mesh[n], x)) is not None and not s.is_empty]
    U = unary_union(occ) if occ else Polygon()
    f = bay.difference(U)
    free[float(x)] = (round(f.intersection(KEEL).area, 1), round(f.intersection(TOP).area, 1))
# bundle per station from the cut list routes
W = json.load(open(f"{PK}/wires.json"))
pts = {k: np.array(v) for k, v in W["points"].items()}
KEELN = {"keel_d2", "keel_d3", "keel_d4", "sled", "chase", "bno"}
TOPN = {"top_d1", "top_d2", "top_d3", "vtx", "flankL", "flankR", "foot_hole", "cam", "drv2"}
load = {float(x): [0.0, 0.0] for x in xs}; count = {float(x): [0, 0] for x in xs}
for w in W["wires"]:
    r = [n for n in w["route"] if isinstance(n, str)]
    a_ = wire_area(w["g"], "silicone" in w["note"] or "Foot" in w["to"])
    for a, b in zip(r[:-1], r[1:]):
        k = 0 if (a in KEELN and b in KEELN) else (1 if (a in TOPN and b in TOPN) else None)
        if k is None: continue
        x0, x1 = sorted((pts[a][0], pts[b][0]))
        for x in xs:
            if x0 <= x <= x1: load[float(x)][k] += a_; count[float(x)][k] += 1
rows = []; worst = [(0, None), (0, None)]
for x in xs:
    x = float(x); fk, ft = free[x]; lk, lt = load[x]
    rk = lk / fk if fk > 0 else (9 if lk > 0 else 0); rt = lt / ft if ft > 0 else (9 if lt > 0 else 0)
    rows.append(dict(x=x, keel_free=fk, keel_bundle=round(lk, 1), keel_wires=count[x][0], top_free=ft, top_bundle=round(lt, 1), top_wires=count[x][1]))
    if rk > worst[0][0]: worst[0] = (round(rk, 2), x)
    if rt > worst[1][0]: worst[1] = (round(rt, 2), x)
rep = dict(note=__doc__.split("\n")[0], worst_keel_fill=worst[0], worst_top_fill=worst[1], stations=rows)
json.dump(rep, open(OUT + "/wireroom.json", "w"), indent=1)
for r in rows[::4]: print(r)
print("worst keel fill", worst[0], " worst top fill", worst[1])
print("WIRE ROOM OK" if worst[0][0] <= 0.6 and worst[1][0] <= 0.6 else "WIRE ROOM TIGHT")
