"""Launcher interface: the folded dart's silhouette seen along its flight axis (what the barrel must clear),
the cradle contact band, and a barrel keep-out solid.  python3 barrel.py out29 pack_v29"""
import json, sys, math, numpy as np, trimesh
from shapely.geometry import Polygon, MultiPolygon, box as sbox
from shapely.ops import unary_union
OUT, PK = sys.argv[1], sys.argv[2]
J = json.load(open(OUT + "/parts.json")); P = J["P"]; PP = {p["name"]: p for p in J["parts"]}
kids = {}
for n, p in PP.items():
    if p.get("parent"): kids.setdefault(p["parent"], []).append(n)
def fam(r): o = [r]; [o.extend(fam(k)) for k in kids.get(r, [])]; return o
from trimesh.transformations import translation_matrix as TM
polys = []; segs = {}
for n, p in PP.items():
    if p.get("parent"): continue
    names = fam(n) if p["axis"] else [n]
    for c in names:
        z = np.load(f"{OUT}/{c}.npz"); v = z["v"].copy(); f = z["f"]
        if p["axis"]: v = v + np.array(p["pos"])
        if PP[c]["group"] == "electronics" and c not in ("n20_kick", "n20_foot", "kick_shaft", "deploy_shaft", "foot_shaft", "n20_leg", "leg_shaft"): continue
        tri = v[f][:, :, 1:3]
        area = 0.5 * np.abs((tri[:, 1, 0] - tri[:, 0, 0]) * (tri[:, 2, 1] - tri[:, 0, 1]) - (tri[:, 2, 0] - tri[:, 0, 0]) * (tri[:, 1, 1] - tri[:, 0, 1]))
        tp = [Polygon(t) for t in tri[area > 1e-4]]
        u = unary_union(tp).buffer(0.01)
        polys.append(u); segs[c] = (float(v[:, 0].min()), float(v[:, 0].max()))
sil = unary_union(polys).buffer(0.05).buffer(-0.05)
CLEAR = 1.0
chan = sil.buffer(CLEAR, join_style=2)
# keep-out solid along the barrel
L = max(b for a, b in segs.values()) + 5
geoms = chan.geoms if isinstance(chan, MultiPolygon) else [chan]
solids = []
for g in geoms:
    m = trimesh.creation.extrude_polygon(Polygon(g.exterior.coords), L + 10)
    m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))      # extrusion z -> x
    m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]) @ np.eye(4)) if False else None
    solids.append(m)
ko = trimesh.util.concatenate(solids)
# polygon coords were (y, z); after the y-rotation the extrusion runs along +x but the axes are (x<-z, y->y, z->-x...). rebuild explicitly instead:
V = []; F = []
for g in geoms:
    ring = np.array(g.exterior.coords)[:-1]
    m2 = trimesh.creation.extrude_polygon(Polygon(ring), L + 10)        # 2D (a=y, b=z), height along +Z
    vv = m2.vertices.copy(); x = vv[:, 2] - 5; y = vv[:, 0]; zz = vv[:, 1]
    m2.vertices = np.c_[x, y, zz]; m2.fix_normals(); V.append(m2)
ko = trimesh.util.concatenate(V)
ko.export(f"{PK}/barrel_keepout.stl")
# profile drawing (SVG + PNG) and numbers
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, ax = plt.subplots(figsize=(7, 7))
for g in (sil.geoms if isinstance(sil, MultiPolygon) else [sil]):
    xs, ys = g.exterior.xy; ax.fill(xs, ys, color="#bbbbbb", lw=0)
for g in geoms:
    xs, ys = g.exterior.xy; ax.plot(xs, ys, color="black", lw=1.2)
t = np.linspace(0, 2 * math.pi, 200); ax.plot(P["R_OUT"] * np.cos(t), P["R_OUT"] * np.sin(t), "k--", lw=0.6)
ax.axhline(0, color="k", lw=0.4); ax.axvline(0, color="k", lw=0.4)
b = chan.bounds
ax.set_title(f"KEDGE {J.get('version','')} folded, seen along the flight axis (from the nose)\nGrey = dart, black = barrel channel with {CLEAR} mm clearance, dashed = Ø{2*P['R_OUT']:.0f} hull", fontsize=9, fontname="DejaVu Sans")
ax.set_aspect("equal"); ax.grid(True, lw=0.3); ax.set_xlabel("y (mm, + = left side)"); ax.set_ylabel("z (mm, up)")
fig.savefig(f"{PK}/barrel_profile.png", dpi=160, bbox_inches="tight"); fig.savefig(f"{PK}/barrel_profile.svg", bbox_inches="tight")
# DXF of the channel outline
with open(f"{PK}/barrel_profile.dxf", "w") as f:
    f.write("0\nSECTION\n2\nENTITIES\n")
    for g in geoms:
        c = list(g.exterior.coords)
        for (a1, b1), (a2, b2) in zip(c, c[1:]):
            f.write(f"0\nLINE\n8\nCHANNEL\n10\n{a1:.3f}\n20\n{b1:.3f}\n11\n{a2:.3f}\n21\n{b2:.3f}\n")
    f.write("0\nENDSEC\n0\nEOF\n")
below = sil.intersection(sbox(-60, -60, 60, 0))
out = dict(channel_bounds_yz=[round(v, 1) for v in b], clearance_mm=CLEAR, silhouette_area_mm2=round(sil.area), hull_area_mm2=round(math.pi * P["R_OUT"] ** 2),
           below_centreline_outside_hull=round(below.difference(Polygon(np.c_[P["R_OUT"] * np.cos(t), P["R_OUT"] * np.sin(t)])).area, 1),
           cradle="the belly (z < 0) is the plain Ø42 cylinder over x %.0f..%.0f: cradle it there; the barrel's lower half can be a plain Ø%.0f half-pipe" % (P["NOSE_L"], P.get("X_TAILSP", [0, P["L"]])[1], 2 * P["R_OUT"] + 2 * CLEAR),
           pusher=("v34: push on the flat tail end face of the two hull halves (no tail cap, no boom); the tether is a bridle to the pod eye lugs, see the engineering report" if "LEG_DEG" in P else "push on the tail cap end face (flat ring Ø42, tether holes in the middle): keep the pusher face a ring so the tether exits through its centre"),
           length_folded=round(L - 5, 1))
json.dump(out, open(f"{PK}/barrel.json", "w"), indent=1); print(json.dumps(out, indent=1))
