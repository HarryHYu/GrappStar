"""KEDGE system physics: mass/CG/inertia, flight stability, launch + impact loads, anchoring, tip-over, kick/deploy torque,
battery, part strength.  python3 physics.py out28  -> out28/physics.json + printed report.
Every assumption is a named constant at the top so the numbers can be redone when something is measured."""
import json, math, sys, numpy as np, trimesh
from trimesh.transformations import rotation_matrix as RM, translation_matrix as TM
OUT = sys.argv[1]
J = json.load(open(OUT + "/parts.json")); P = J["P"]; PP = {p["name"]: p for p in J["parts"]}
g = 9.81
# ---------------- assumptions
RHO_PLA = 1.24e-3            # g/mm3
WALL = 1.7                   # 4 loops x 0.42
INFILL = 0.40
BOUGHT_G = {                 # grams, typical catalogue / bench values (MEASURE on the day: kitchen scale)
    "n20_kick": 10.5, "n20_deploy": 10.5, "n20_foot": 10.5, "drill_leg_L_motor": 11.0, "drill_leg_R_motor": 11.0,
    "drill_leg_L_shaft": 0.0, "drill_leg_R_shaft": 0.0, "kick_shaft": 0.0, "deploy_shaft": 0.0, "foot_shaft": 0.0,   # included in motor mass
    "cell_1": 22.0, "cell_2": 22.0, "xiao": 3.0, "drv8833_1": 1.8, "drv8833_2": 1.8, "drv8833_3": 1.8, "ina219": 2.2,
    "bno085": 2.6, "mp1584": 2.0, "vtx": 3.6, "bms_2s": 1.5, "camera": 2.2,
    "drill_leg_L_brg1": 0.8, "drill_leg_L_brg2": 0.8, "drill_leg_R_brg1": 0.8, "drill_leg_R_brg2": 0.8,
    "drill_leg_L_nut": 0.5, "drill_leg_R_nut": 0.5, "drill_leg_L_nyloc": 0.6, "drill_leg_R_nyloc": 0.6,
}
EXTRA = [("wiring + solder", 12.0, (85, 0, 0)), ("screws, washers, kick bearing, grub", 7.0, (70, 0, 5)), ("contacts, JST, antenna, foam tape", 6.0, (120, 0, 0))]
RANGE_M = (15.0, 20.0)       # launcher target
DRAG_FACTOR = 1.3            # launch speed margin for drag + tether payout
STROKE_M = 0.20              # launcher acceleration stroke (assumed; replace with the launcher's)
IMPACT_STOP_M = 0.03         # stopping distance on landing (soil 3 cm; hard ground much less)
SOIL = {"dry sand": dict(gamma=16e3, phi=32, cu=0), "moist loam": dict(gamma=18e3, phi=28, cu=10e3), "firm clay": dict(gamma=19e3, phi=0, cu=30e3)}
PLA_SIGMA = 45e6             # Pa, printed PLA tensile (layer-aligned parts lower: use 25e6 across layers)
PLA_TAU = 25e6
CELL_MAH = 700; CELL_V = 7.4

def load(n):
    z = np.load(f"{OUT}/{n}.npz"); return trimesh.Trimesh(z["v"], z["f"], process=False)
kids = {}
for n, p in PP.items():
    if p.get("parent"): kids.setdefault(p["parent"], []).append(n)
def fam(r): o = [r]; [o.extend(fam(k)) for k in kids.get(r, [])]; return o
def T(n, d): return TM(PP[n]["pos"]) @ RM(math.radians((-1 if n == "kick_leg" else 1) * d), [0, 1, 0])

def part_mass(n, m):
    if PP[n]["printable"]:
        V = m.volume; A = m.area; shell = min(V, A * WALL)
        return RHO_PLA * (shell + INFILL * max(0.0, V - shell))
    return BOUGHT_G.get(n, 0.0)

def assembly(kick_deg, dep_deg):
    items = []
    for n, p in PP.items():
        if p.get("parent"): continue
        if p["axis"]:
            Tm = T(n, kick_deg if n == "kick_leg" else dep_deg)
            for c in fam(n): items.append((c, load(c).apply_transform(Tm)))
        else:
            items.append((n, load(n)))
    return items

def mass_props(items):
    M = 0; S = np.zeros(3); rows = []
    for n, m in items:
        mm = part_mass(n, m)
        c = m.center_mass if m.is_watertight and m.volume > 1 else m.bounds.mean(0)
        M += mm; S += mm * c; rows.append((n, round(mm, 1), [round(float(x), 1) for x in c]))
    for nm, mm, c in EXTRA: M += mm; S += mm * np.array(c); rows.append((nm, mm, list(c)))
    cg = S / M
    I = np.zeros((3, 3))
    for n, m in items:
        mm = part_mass(n, m); c = m.center_mass if m.is_watertight and m.volume > 1 else m.bounds.mean(0)
        d = c - cg; I += mm * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return M, cg, I, rows

R = {}
fold = assembly(0, 0)
M, cg, I, rows = mass_props(fold)
R["mass_g"] = round(M, 1); R["cg_mm"] = [round(float(x), 1) for x in cg]
R["printed_g"] = round(sum(part_mass(n, m) for n, m in fold if PP.get(n, {}).get("printable")), 1)
R["bought_g"] = round(sum(part_mass(n, m) for n, m in fold if not PP.get(n, {}).get("printable")) + sum(e[1] for e in EXTRA), 1)
R["heaviest"] = sorted(rows, key=lambda r: -r[1])[:8]

# ---------------- flight stability (folded): projected-area centroid as the centre of pressure (cardboard-cutout method,
# conservative for a slender body); the trailing tether adds drag at the tail which moves CP further aft.
xs = np.arange(-5, P['L'] + 40, 1.0)
side = np.zeros_like(xs)
# union spans across parts in z by occupancy grid
grid = np.zeros((len(xs), 200), bool); z0 = -60
for n, m in fold:
    if PP.get(n, {}).get("group") == "electronics" and n not in ("n20_kick", "n20_foot"): continue
    pts = m.sample(6000) if m.area > 50 else m.vertices
    ix = np.clip(np.round(pts[:, 0] - xs[0]).astype(int), 0, len(xs) - 1); iz = np.clip(np.round(pts[:, 2] - z0).astype(int), 0, 199)
    grid[ix, iz] = True
# fill between min and max per column per connected run is overkill: take column fill = count of occupied 1 mm cells after closing gaps of 3 mm
from scipy.ndimage import binary_closing
gs = binary_closing(grid, structure=np.ones((3, 5)))
side = gs.sum(1).astype(float)
grid2 = np.zeros((len(xs), 200), bool); y0 = -100
for n, m in fold:
    if PP.get(n, {}).get("group") == "electronics" and n not in ("n20_kick", "n20_foot"): continue
    pts = m.sample(6000) if m.area > 50 else m.vertices
    ix = np.clip(np.round(pts[:, 0] - xs[0]).astype(int), 0, len(xs) - 1); iy = np.clip(np.round(pts[:, 1] - y0).astype(int), 0, 199)
    grid2[ix, iy] = True
top = binary_closing(grid2, structure=np.ones((3, 5))).sum(1).astype(float)
# body tube: fill the hull section solid (inside the skin counts as area)
body = np.where((xs >= 0) & (xs <= P["L"]), 2 * P["R_OUT"], 0)
side = np.maximum(side, body); top = np.maximum(top, body)
cp_side = float((xs * side).sum() / side.sum()); cp_top = float((xs * top).sum() / top.sum())
D = 2 * P["R_OUT"]
R["flight"] = dict(direction="nose first (toward -x)", cg_x=round(float(cg[0]), 1), cp_side_x=round(cp_side, 1), cp_top_x=round(cp_top, 1),
                   static_margin_calibers_side=round((cp_side - cg[0]) / D, 2), static_margin_calibers_top=round((cp_top - cg[0]) / D, 2),
                   note="positive = CP behind CG = stable nose first. Cardboard-cutout CP; real CP sits further aft because the folded legs, augers and the trailing tether all sit aft.")

# ---------------- launch
v_need = [math.sqrt(g * r) * DRAG_FACTOR for r in RANGE_M]
a_launch = [v * v / (2 * STROKE_M) for v in v_need]
v_imp = v_need[1] / DRAG_FACTOR * 0.95
a_imp = v_imp ** 2 / (2 * IMPACT_STOP_M)
R["launch"] = dict(v_needed_m_s=[round(v, 1) for v in v_need], stroke_m=STROKE_M, accel_g=[round(a / g) for a in a_launch],
                   launch_force_N=round(M / 1000 * a_launch[1], 0), spring_energy_J=round(0.5 * M / 1000 * v_need[1] ** 2, 1),
                   impact_speed_m_s=round(v_imp, 1), impact_stop_m=IMPACT_STOP_M, impact_g=round(a_imp / g))
G_L = a_launch[1] / g; G_I = a_imp / g
def gload(mass_g, gg): return mass_g / 1000 * gg * g
loads = []
cellF = gload(22, G_L)
loads.append(["cells pushed aft at launch (each)", round(cellF, 1), "N", "rear contacts are RIGID plates on the back disc of the cell bay and the springs sit on the front disc, so launch presses the cells onto the plates and contact never breaks"])
loads.append(["cells forward at landing (both onto the front cell-bay disc)", round(2 * gload(22, G_I), 0), "N", "that disc is 3 mm thick and carries the load through the spine to the 4 disc-1 screws"])
pod_m = sum(part_mass(n, load(n)) for n in fam("drill_leg_L"))
arm = 9.0e-3
Mb = gload(pod_m, G_I) * arm; Wsec = math.pi * (P["CPL_D"] * 1e-3) ** 3 / 32
loads.append(["coupler bending at landing (pod %.0f g, %d g)" % (pod_m, G_I), round(Mb / Wsec / 1e6, 1), "MPa", "PLA ~45 MPa: margin %.1f" % (PLA_SIGMA / (Mb / Wsec))])
kick_m = sum(part_mass(n, load(n)) for n in fam("kick_leg"))
loads.append(["kick leg along its axis at landing (%.0f g)" % kick_m, round(gload(kick_m, G_I), 0), "N", "taken by the Ø3 steel shaft in shear + ear bearing; worm sees almost no torque because the line of action runs through the pivot"])
tray_m = sum(part_mass(n, m) for n, m in fold if n in ("tray", "xiao", "drv8833_1", "drv8833_2", "drv8833_3", "ina219", "bno085", "mp1584", "vtx", "bms_2s", "cell_1", "cell_2", "n20_deploy"))
loads.append(["tray + contents on 4 disc-1 screws at landing (%.0f g)" % tray_m, round(gload(tray_m, G_I) / 4, 0), "N per screw", "M3 self-tapped into PLA shear ~250-400 N each"])
loads.append(["launcher push through the tail cap", round(M / 1000 * a_launch[1], 0), "N", "4 radial M3 into the spigot ring + the hull wall in compression"])
R["loads"] = loads

# ---------------- anchoring (very small augers, shallow): two mechanisms per auger
dep = assembly(P["KICK_DEG"], P["DEPLOY_DEG"])
Gz = J["ground"]
def depth(n): return max(0.0, Gz - min(m.vertices[:, 2].min() for nm, m in dep if nm == n))
th = math.radians(P["DEPLOY_DEG"])                       # drill auger axis below horizontal
L_d = depth("drill_leg_L_auger") / 1000 / math.sin(th)       # embedded LENGTH along the raked auger
L_f = depth("foot_auger") / 1000; Dau = P["AUGER_D"] / 1000
def broms_clay_free(cu, D, L):
    """Broms short rigid pile in clay, free head, load at ground: solve L = 1.5D + f + g, f = H/(9 cu D), H (1.5D + f/2) = 2.25 cu D g^2"""
    lo, hi = 0.0, 9 * cu * D * L
    for _ in range(80):
        H = (lo + hi) / 2; f = H / (9 * cu * D); g_ = L - 1.5 * D - f
        if g_ <= 0 or H * (1.5 * D + f / 2) > 2.25 * cu * D * g_ ** 2: hi = H
        else: lo = H
    return lo
anch = {}
for sname, s_ in SOIL.items():
    Kp = math.tan(math.radians(45 + s_["phi"] / 2)) ** 2 if s_["phi"] else 1
    def lat(L):          # sideways capacity: sand term (Broms, e = 0) + clay term (Broms free head)
        return (0.5 * s_["gamma"] * Dau * L ** 2 * Kp if s_["phi"] else 0) + (broms_clay_free(s_["cu"], Dau, L) if s_["cu"] else 0)
    def ax(L):           # end bearing on the flight disc: shallow Nc ~ 6.5, Nq (Reissner)
        A_ = math.pi * (Dau / 2) ** 2
        Nq = math.exp(math.pi * math.tan(math.radians(s_["phi"]))) * math.tan(math.radians(45 + s_["phi"] / 2)) ** 2 if s_["phi"] else 1
        return A_ * (s_["gamma"] * L * math.sin(th) * Nq + 6.5 * s_["cu"])
    # tether pull is horizontal toward the tail; along the auger that is cos(th) (pushes it in), across it sin(th): the weaker limit governs
    drill_each = min(ax(L_d) / max(math.cos(th), 1e-3), lat(L_d) / max(math.sin(th), 1e-3))
    foot = lat(L_f)
    weight_fr = M / 1000 * g * 0.5
    anch[sname] = dict(drill_auger_each_N=round(drill_each, 1), foot_auger_N=round(foot, 1), friction_N=round(weight_fr, 1),
                       total_N=round(2 * drill_each + foot + weight_fr, 1))
R["anchoring"] = dict(depth_mm=dict(drill=round(depth("drill_leg_L_auger"), 1), foot=round(L_f * 1000, 1)), embedded_length_mm=dict(drill=round(L_d * 1000, 1), foot=round(L_f * 1000, 1)), by_soil=anch,
                      note="Short rigid piles (Broms). Holding scales with embedded length: about L^2 in sand. A reel that pulls harder than this drags the dart: the reel side should limit tether tension.")

# ---------------- tip-over: tether horizontal at the tail eye height (axis, 21 above ground), nose would lift about the rear auger line
F_ref = anch["moist loam"]["total_N"]
h = -Gz / 1000
def entry_x(n):        # where an auger crosses the ground plane (mean x of its vertices within 1 mm of ground)
    for nm, m in dep:
        if nm == n:
            v = m.vertices; sel = np.abs(v[:, 2] - Gz) < 1.0
            return float(v[sel, 0].mean()) / 1000 if sel.any() else float(v[:, 0].mean()) / 1000
x_drill = entry_x("drill_leg_L_auger"); foot_x = entry_x("foot_auger")
M_tip = F_ref * h
F_foot_needed = M_tip / (x_drill - foot_x)
A_f = math.pi * (P["AUGER_D"] / 2000) ** 2
sl = SOIL["moist loam"]
Nq_l = math.exp(math.pi * math.tan(math.radians(sl["phi"]))) * math.tan(math.radians(45 + sl["phi"] / 2)) ** 2
foot_uplift = A_f * (sl["gamma"] * L_f * Nq_l * 0.5 + 6.5 * sl["cu"])     # flights bearing upward on the soil above (half Nq for uplift)
R["tip_over_foot_uplift_capacity_loam_N"] = round(foot_uplift, 1)
R["tip_over"] = dict(tether_height_mm=round(h * 1000), pull_N=F_ref, moment_Nm=round(M_tip, 3), foot_lever_mm=round((x_drill - foot_x) * 1000),
                     foot_hold_down_needed_N=round(F_foot_needed, 2), dart_weight_N=round(M / 1000 * g, 2),
                     note="the kick foot ahead of the nose holds the nose down; weight alone resists %.2f N" % (M / 1000 * g * (x_drill - cg[0] / 1000) / (x_drill - foot_x)))

# ---------------- kick torque over the swing (gravity) and deploy torque
kfam = fam("kick_leg"); kms = {n: load(n) for n in kfam}
km = sum(part_mass(n, kms[n]) for n in kfam)
kc = sum(part_mass(n, kms[n]) * (kms[n].center_mass if kms[n].is_watertight else kms[n].bounds.mean(0)) for n in kfam) / km
tq = []
for d in range(0, int(P["KICK_DEG"]) + 1, 10):
    c = (RM(math.radians(-d), [0, 1, 0]) @ np.r_[kc, 1])[:3]
    tq.append((d, round(km / 1000 * abs(c[0]) / 10, 3)))   # kg*cm
R["kick"] = dict(leg_mass_g=round(km, 1), cg_radius_mm=round(float(np.hypot(kc[0], kc[2])), 1), max_gravity_torque_kgcm=max(t for _, t in tq),
                 profile_kgcm=tq, note="worm N20s of this size are typically rated 1-4 kg.cm; the swing is light. Planting the foot is done by spinning the foot auger WHILE the kick drives 180 -> stop")
pod_c = sum(part_mass(n, load(n)) * load(n).center_mass for n in fam("drill_leg_L") if load(n).is_watertight) / pod_m
R["deploy"] = dict(pod_mass_g=round(pod_m, 1), gravity_torque_both_kgcm=round(2 * pod_m / 1000 * abs(pod_c[0]) / 10, 3),
                   soil_cut_note="the augers are radial on the pods, so depth comes only from the swing: they cut sideways through the soil. Spin the drills WHILE deploying; the deploy worm stalls first in firm soil.")
# ---------------- battery
idle_W = 5.0 * (0.10 + 0.20 + 0.10) / 0.9    # XIAO WiFi AP, VTX 25 mW, camera
R["battery"] = dict(capacity_Wh=round(CELL_MAH / 1000 * CELL_V, 2), idle_W_video_on=round(idle_W, 2), hours_video_on=round(CELL_MAH / 1000 * CELL_V / idle_W, 1),
                    anchor_sequence_mAh=round((0.2 * 15 + 0.6 * 10 + 0.2 * 3 + 0.7 * 20) / 3.6, 1), worst_current_A=2.0)
# ---------------- part strength
# kick pole: real cross-section (round pole + outboard keel, wire channel) sliced from the mesh at mid-pole
km_ = load("kick_leg"); sec = km_.section(plane_origin=[P["KICK_HUB_D"] / 2 + P["KICK_POLE_L"] / 2, 0, 0], plane_normal=[1, 0, 0])
pl, _ = sec.to_2D(); poly = pl.polygons_full[0] if len(pl.polygons_full) else None
def sec_modulus(poly, axis):          # bending about the in-plane axis, from the polygon's second moment
    from shapely.geometry import Polygon as SP
    import shapely
    c = np.array(poly.centroid.coords[0]); pts = np.array(poly.exterior.coords) - c
    def I(ring):
        x, y = ring[:, 0], ring[:, 1]; cr = x[:-1] * y[1:] - x[1:] * y[:-1]
        return (np.sum((y[:-1] ** 2 + y[:-1] * y[1:] + y[1:] ** 2) * cr) / 12, np.sum((x[:-1] ** 2 + x[:-1] * x[1:] + x[1:] ** 2) * cr) / 12)
    Ix, Iy = I(pts); Ix, Iy = abs(Ix), abs(Iy)
    for hole in poly.interiors:
        h_ = np.array(hole.coords) - c; hx, hy = I(h_); Ix -= abs(hx); Iy -= abs(hy)
    cmax = np.abs(pts).max(0)
    return min(Ix / cmax[1], Iy / cmax[0]) * 1e-9
Wp = sec_modulus(poly, 0) if poly is not None else 44.8e-9
F_foot_side = 20.0
Mk = F_foot_side * P["KICK_POLE_L"] / 1000
R["strength"] = dict(kick_pole_section_mm3=round(Wp * 1e9, 1), kick_pole_stress_MPa_at_20N_side_load=round(Mk / Wp / 1e6, 1),
                     kick_pole_side_load_limit_N=round(PLA_SIGMA / 2 * Wp / (P["KICK_POLE_L"] / 1000), 1),
                     coupler_torque_capacity_Nm=round(PLA_TAU * math.pi * (P["CPL_D"] * 1e-3) ** 3 / 16 * 0.6, 2),
                     note="coupler capacity derated 0.6 for the D flat and layer lines; deploy worm output is far below it")
json.dump(R, open(OUT + "/physics.json", "w"), indent=1)
print(json.dumps(R, indent=1))
