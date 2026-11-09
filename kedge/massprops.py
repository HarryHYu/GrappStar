"""Mass properties for dart-physics and the righting sim: every part's volume, area, centroid and inertia (unit density, about its
own centroid, dart axes) in the FOLDED, DEPLOYED (anchored) and, when the build has a righting leg, LEG_OUT poses, plus the printed
mass at the house print settings and (v34) the whole-dart mass and centre of mass per pose with the catalogue bought masses.
python3 massprops.py out34  -> out34/mass_props.json
Printed parts: shell (4 walls x 0.42 = 1.7) solid + 40 percent infill, PLA 1.24 g/cm3 (PPA-CF parts the same rule, density 1.3);
bought parts: a part's "mass_g" in parts.json, else dart_model.BOUGHT_G (dart-physics' catalogue table), plus its EXTRA items."""
import json, math, sys, os, numpy as np, trimesh
from trimesh.transformations import rotation_matrix as RM, translation_matrix as TM
OUT = sys.argv[1]
J = json.load(open(OUT + "/parts.json")); P = J["P"]; PP = {p["name"]: p for p in J["parts"]}
RHO, WALL, INFILL = 1.24e-3, 1.7, 0.40
PPA = ("coupler", "leg_coupler")
try:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import dart_model as dm
    BOUGHT, ZERO, EXTRA = dict(dm.BOUGHT_G), set(dm.ZERO_G), list(dm.EXTRA)
except Exception:
    BOUGHT, ZERO, EXTRA = {}, set(), []
BOUGHT.update({"n20_leg": BOUGHT.get("n20_kick", 10.5), "contacts_front_spring": 0.5, "contacts_mid_strip": 0.3, "contacts_rear_plate": 0.3,
               "leg_grub_L": 0.3, "leg_grub_R": 0.3})
ZERO |= {"leg_shaft"}
if "LEG_DEG" in P:     # v34: the harness runs in the lower corridors beside the in-line cells; the line is on the pod eyes
    EXTRA = [("wiring + solder (v34 routes: corridors beside the cells, up to deck 1)", 12.0, (90.0, 0.0, -4.0)) if "wiring" in e[0] else e for e in EXTRA if "tether" not in e[0]]
    EXTRA.append(("tether bridle and ring at the pod eyes (outside, about 0.5 g)", 0.5, (P.get("DRILL_X", 132.0) + 10.0, 0.0, -4.0)))
kids = {}
for n, p in PP.items():
    if p.get("parent"): kids.setdefault(p["parent"], []).append(n)
def root(n):
    while PP[n].get("parent"): n = PP[n]["parent"]
    return n
ISLEG = lambda r: r == "kick_leg" or PP[r].get("joint") == "leg"
POSES = dict(folded=dict(kick=0.0, deploy=0.0), deployed=dict(kick=0.0 if "LEG_DEG" in P else P["KICK_DEG"], deploy=P["DEPLOY_DEG"]))
if "LEG_DEG" in P:
    POSES["leg_out"] = dict(kick=P["LEG_DEG"], deploy=0.0)
def T(n, pose):
    r = root(n); p = PP[r]
    if not p.get("axis"): return np.eye(4)
    d = POSES[pose]["kick"] if ISLEG(r) else POSES[pose]["deploy"]
    return TM(p["pos"]) @ RM(math.radians(p.get("sgn", -1 if r == "kick_leg" else 1) * d), [0, 1, 0])
def part_g(n, m):
    p = PP[n]
    if p.get("mass_g") is not None: return float(p["mass_g"])
    if p["printable"]:
        V = m.volume; shell = min(V, m.area * WALL); rho = 1.30e-3 if n.startswith(PPA) else RHO
        return rho * (shell + INFILL * max(0.0, V - shell))
    if n in ZERO or n == "ballast": return 0.0
    return BOUGHT.get(n, 0.0)
out = {}; tot = {k: [0.0, np.zeros(3)] for k in POSES}
for n, p in PP.items():
    z = np.load(f"{OUT}/{n}.npz"); m = trimesh.Trimesh(np.asarray(z["v"], float), z["f"], process=False)
    row = dict(printable=p["printable"], group=p["group"], parent=p.get("parent"), root=root(n),
               volume_mm3=round(float(m.volume), 1), area_mm2=round(float(m.area), 1), watertight=bool(m.is_watertight))
    g = part_g(n, m)
    if p["printable"]: row["printed_g"] = round(g, 2)
    else: row["bought_g"] = round(g, 2)
    for pose in POSES:
        mm = m.copy(); mm.apply_transform(T(n, pose))
        c = mm.center_mass if mm.is_watertight and mm.volume > 1 else mm.bounds.mean(0)
        row[pose] = dict(centroid_mm=[round(float(x), 2) for x in c],
                         inertia_unit_density_mm5=[[round(float(x), 1) for x in r_] for r_ in (mm.moment_inertia if mm.is_watertight else np.zeros((3, 3)))])
        tot[pose][0] += g; tot[pose][1] += g * np.asarray(c)
    out[n] = row
totals = {}
for pose, (M, mc) in tot.items():
    for nm, g, c in EXTRA:
        M += g; mc = mc + g * np.asarray(c)
    totals[pose] = dict(mass_g=round(float(M), 1), cg_mm=[round(float(x), 2) for x in mc / M])
pg = sum(r.get("printed_g", 0) for r in out.values()); bg = sum(r.get("bought_g", 0) for r in out.values()) + sum(e[1] for e in EXTRA)
json.dump(dict(frame="dart: nose tip x 0, +x to the tail, +y left, +z up (hull axis z 0), belly z -21 lying flat",
               poses={k: dict(leg_or_kick_deg=v["kick"], deploy_deg=v["deploy"]) for k, v in POSES.items()},
               print_rule=dict(rho_g_mm3=RHO, wall_mm=WALL, infill=INFILL), ballast_g=P.get("BALLAST_G"),
               totals=totals, printed_g=round(pg, 1), bought_g=round(bg, 1), extras=[[e[0], e[1], list(e[2])] for e in EXTRA], parts=out),
          open(OUT + "/mass_props.json", "w"), indent=1)
print(len(out), "parts, printed", round(pg, 1), "g at 4 walls / 40 percent, bought + extras", round(bg, 1), "g")
for k, v in totals.items(): print(k, v)
