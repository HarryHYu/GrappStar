"""Mass properties for dart-physics: every part's volume, area, centroid and inertia (unit density, about its own
centroid, dart axes) in the FOLDED pose and the DEPLOYED pose, plus the printed mass at the house print settings.
python3 massprops.py out31  -> out31/mass_props.json
Bought parts carry no mass here (dart-physics owns BOUGHT_G); printed parts: shell (4 walls x 0.42 = 1.7) solid +
40 percent infill, PLA 1.24 g/cm3, same rule as physics.py."""
import json, math, sys, numpy as np, trimesh
from trimesh.transformations import rotation_matrix as RM, translation_matrix as TM
OUT = sys.argv[1]
J = json.load(open(OUT + "/parts.json")); P = J["P"]; PP = {p["name"]: p for p in J["parts"]}
RHO, WALL, INFILL = 1.24e-3, 1.7, 0.40
kids = {}
for n, p in PP.items():
    if p.get("parent"): kids.setdefault(p["parent"], []).append(n)
def root(n):
    while PP[n].get("parent"): n = PP[n]["parent"]
    return n
def T(n, pose):
    r = root(n); p = PP[r]
    if not p.get("axis"): return np.eye(4)
    d = 0.0 if pose == "folded" else (P["KICK_DEG"] if r == "kick_leg" else P["DEPLOY_DEG"])
    return TM(p["pos"]) @ RM(math.radians((-1 if r == "kick_leg" else 1) * d), [0, 1, 0])
out = {}
for n, p in PP.items():
    z = np.load(f"{OUT}/{n}.npz"); m = trimesh.Trimesh(np.asarray(z["v"], float), z["f"], process=False)
    row = dict(printable=p["printable"], group=p["group"], parent=p.get("parent"), root=root(n),
               volume_mm3=round(float(m.volume), 1), area_mm2=round(float(m.area), 1), watertight=bool(m.is_watertight))
    if p["printable"]:
        V = m.volume; shell = min(V, m.area * WALL)
        row["printed_g"] = round(RHO * (shell + INFILL * max(0.0, V - shell)), 2)
    for pose in ("folded", "deployed"):
        mm = m.copy(); mm.apply_transform(T(n, pose))
        c = mm.center_mass if mm.is_watertight and mm.volume > 1 else mm.bounds.mean(0)
        row[pose] = dict(centroid_mm=[round(float(x), 2) for x in c],
                         inertia_unit_density_mm5=[[round(float(x), 1) for x in r_] for r_ in (mm.moment_inertia if mm.is_watertight else np.zeros((3, 3)))])
    out[n] = row
json.dump(dict(frame="dart: nose tip x 0, +x to the tail, +y left (kick side), +z up, belly z -21 lying flat",
               poses=dict(folded=dict(kick=0, deploy=0), deployed=dict(kick=P["KICK_DEG"], deploy=P["DEPLOY_DEG"])),
               print_rule=dict(rho_g_mm3=RHO, wall_mm=WALL, infill=INFILL), ballast_g=P.get("BALLAST_G"), parts=out),
          open(OUT + "/mass_props.json", "w"), indent=1)
pg = sum(r.get("printed_g", 0) for r in out.values())
print(len(out), "parts, printed", round(pg, 1), "g at 4 walls / 40 percent")
