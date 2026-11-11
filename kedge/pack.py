"""Build-day pack: oriented STLs, two print plates (3MF), STEP set, fit coupon.  python3 pack.py out27 pack27"""
import json, math, os, sys, zipfile, shutil
import numpy as np, trimesh
from trimesh.transformations import rotation_matrix as RM, translation_matrix as TM
OUT, PK = sys.argv[1], sys.argv[2]
os.makedirs(PK, exist_ok=True)
J = json.load(open(OUT + "/parts.json")); PP = {p["name"]: p for p in J["parts"]}
X = [1, 0, 0]; Y = [0, 1, 0]
ORIENT = {   # rotation that puts each part in its print pose (z up from the bed)
    "hull_top": np.eye(4), "hull_bottom": RM(math.pi, X), "nose": RM(-math.pi / 2, Y), "tail_cap": RM(math.pi / 2, Y),
    "tray": np.eye(4), "kick_leg": RM(-math.pi / 2, X), "drill_leg_L": np.eye(4), "drill_leg_R": np.eye(4),
    "drill_leg_L_auger": RM(-math.pi / 2, Y), "drill_leg_R_auger": RM(-math.pi / 2, Y), "foot_auger": np.eye(4),
    "coupler_L": RM(math.pi, Y), "coupler_R": RM(math.pi, Y),      # lying on the D flat: layers run along the axle (bending)
    "pod_cap_L": RM(math.pi / 2, X), "pod_cap_R": RM(-math.pi / 2, X),
    "tail_boom": RM(math.pi / 2, X), "boom_anchor": RM(math.pi / 2, Y), "drill_leg_L_tip": RM(math.pi / 2, Y), "drill_leg_R_tip": RM(math.pi / 2, Y),
}
if "tail_boom" in PP:                     # v32: arms stand on the head's flat front face, sleeves on their base
    ORIENT.update({"drill_leg_L": RM(-math.pi / 2, Y), "drill_leg_R": RM(-math.pi / 2, Y)})
if J["P"].get("FOOT_TILT"):               # v32c: the foot auger is stored tilted with its housing; stand it back on its base
    ORIENT["foot_auger"] = RM(-math.radians(J["P"]["FOOT_TILT"]), Y)
if "righting_leg" in PP:                  # v34: rear pods folded forward (mirrored arms and augers), righting leg, skin plugs
    ORIENT.update({"drill_leg_L": RM(math.pi / 2, Y), "drill_leg_R": RM(math.pi / 2, Y),          # stand on the head's flat rear (stop) face
                   "drill_leg_L_auger": RM(math.pi / 2, Y), "drill_leg_R_auger": RM(math.pi / 2, Y),  # point up
                   "righting_leg": RM(math.pi, X),                                                  # upside down: the crown strip on the bed, cheeks up
                   "leg_coupler_L": RM(math.pi, X), "leg_coupler_R": RM(math.pi, X),                 # lying on the D flat
                   "leg_plug_L": RM(math.pi / 2, X), "leg_plug_R": RM(-math.pi / 2, X)})             # flat inner face down
if J["P"].get("POD_DIR") == 1:           # v35: pods hinge at the front and point aft (arms and augers no longer mirrored)
    ORIENT.update({k: RM(-math.pi / 2, Y) for k in ["drill_leg_L", "drill_leg_R", "drill_leg_L_auger", "drill_leg_R_auger"]})
ORIENT = {k: v for k, v in ORIENT.items() if k in PP}          # older builds have no pod caps
SUPPORT = {   # what the slicer must do (Bambu Studio, PLA, 0.4 nozzle, 0.2 layers)
    "hull_top": "tree supports ON, build plate only: the inner roof of the half shell (about 28 mm wide strip); pivot blocks, saddle and ear print without (arc outlines, 45 deg undersides)",
    "hull_bottom": "tree supports ON, build plate only: the inner roof of the half shell",
    "nose": "none (tip down, cone walls under 45 deg)", "tail_cap": "none (end face down; the socket windows bridge 8 mm)",
    "tray": "none (decks bridge 26 mm between discs; enable 'thick bridges')", "kick_leg": "none (hub face and foot housing outboard face on the bed)",
    "drill_leg_L": "none (flat-bottomed head, D-bore roof bridges 5 mm)", "drill_leg_R": "none (flat-bottomed head, D-bore roof bridges 5 mm)",
    "drill_leg_L_auger": "none (point up: 45 deg flight load faces, 45 deg roof inside the hub); brim 5 mm, the base is a thin ring", "drill_leg_R_auger": "none (point up: 45 deg flight load faces, 45 deg roof inside the hub); brim 5 mm, the base is a thin ring",
    "foot_auger": "none; brim 5 mm", "coupler_L": "PPA-CF (hardened nozzle, dried filament), lying on the D flat, no supports; PLA only as a stopgap", "coupler_R": "PPA-CF (hardened nozzle, dried filament), lying on the D flat, no supports; PLA only as a stopgap",
    "pod_cap_L": "none", "pod_cap_R": "none",
    "drill_leg_L_tip": "none (spigot down)", "drill_leg_R_tip": "none (spigot down)",
    "tail_boom": "PPA-CF, lying on its side, no supports (the shank bridges 12 mm)", "boom_anchor": "PPA-CF, flat, no supports",
}
SUPPORT.update({"righting_leg": "upside down (the blade crown on the bed, cheeks up); BRIM 5 mm (the crown touches the bed along a 5 mm strip only); tree supports under the two ends of the pad arch only; 4 walls",
                "leg_coupler_L": "PPA-CF, lying on the D flat, no supports", "leg_coupler_R": "PPA-CF, lying on the D flat, no supports",
                "leg_plug_L": "none (flat inner face down)", "leg_plug_R": "none (flat inner face down)",
                "tray": "none (flat underside down: U saddle legs, disc bottoms, cradle tubes, the tail key's web); deck 1 and deck 2 bridge, enable 'thick bridges'"})
if "righting_leg" in PP:
    SUPPORT["nose"] = "none (tip down: cone walls under 45 deg, the putty collar has a 45 deg underside)"
    SUPPORT["hull_top"] = "tree supports ON, build plate only: the inner roof of the half shell and under the knuckle and groove floors; the pivot blocks print without (arc outlines)"
QTY = {k: 1 for k in ORIENT}
def load(n):
    z = np.load(f"{OUT}/{n}.npz"); return trimesh.Trimesh(z["v"], z["f"], process=False)
oriented = {}
for n, R in ORIENT.items():
    m = load(n); m.apply_transform(R); b = m.bounds
    m.apply_transform(TM([-(b[0][0] + b[1][0]) / 2, -(b[0][1] + b[1][1]) / 2, -b[0][2]]))
    oriented[n] = m
    m.export(f"{PK}/{n}.stl")
    # bed contact check: area of faces within 0.05 of z=0 facing down
    down = (m.face_normals[:, 2] < -0.99) & (m.triangles[:, :, 2].max(1) < 0.05)
    PP[n]["bed_contact_mm2"] = round(float(m.area_faces[down].sum()), 1)
    PP[n]["print_bbox"] = [round(float(e), 1) for e in m.extents]
# overhang report: downward faces steeper than 45 deg from vertical that are not on the bed
def overhang(m):
    n = m.face_normals; z = m.triangles[:, :, 2].min(1)
    bad = (n[:, 2] < -0.7072) & (z > 0.3)
    return float(m.area_faces[bad].sum())
for n, m in oriented.items():
    PP[n]["overhang_mm2_gt45"] = round(overhang(m), 1)
# plates (X2D bed: use a 240 x 240 safe area)
PLATES = {"plate_A_hull": [n for n in ["hull_top", "hull_bottom", "nose", "tail_cap", "pod_cap_L", "pod_cap_R", "drill_leg_L_tip", "drill_leg_R_tip"] + ([] if "pod_cap_L" in ORIENT else ["coupler_L", "coupler_R"]) if n in ORIENT],
          **({"plate_C_couplers_PPA-CF": [n for n in ["coupler_L", "coupler_R", "tail_boom", "boom_anchor"] if n in ORIENT]} if "pod_cap_L" in ORIENT else {}),
          "plate_B_legs_tray": [n for n in ["tray", "kick_leg", "righting_leg", "drill_leg_L", "drill_leg_R", "drill_leg_L_auger", "drill_leg_R_auger", "foot_auger", "leg_plug_L", "leg_plug_R"] if n in ORIENT]}
if "righting_leg" in PP:
    PLATES["plate_C_couplers_PPA-CF"] = [n for n in ["coupler_L", "coupler_R", "leg_coupler_L", "leg_coupler_R"] if n in ORIENT]
plate_info = {}
for pn, names in PLATES.items():
    sc = trimesh.Scene(); x = y = 0; rowh = 0; placed = []
    for n in sorted(names, key=lambda k: -oriented[k].extents[1]):
        m = oriented[n].copy(); ex, ey = m.extents[0], m.extents[1]
        if x + ex > 230: x = 0; y += rowh + 8; rowh = 0
        m.apply_transform(TM([x + ex / 2 - 115, y + ey / 2 - 115, 0])); x += ex + 8; rowh = max(rowh, ey)
        sc.add_geometry(m, node_name=n, geom_name=n); placed.append(n)
    assert y + rowh <= 235, (pn, y + rowh)
    sc.export(f"{PK}/{pn}.3mf")
    plate_info[pn] = dict(parts=placed, extent_mm=[230, round(y + rowh, 1)])
# STEP set
for n in ORIENT:
    if os.path.exists(f"{OUT}/{n}.step"): shutil.copy(f"{OUT}/{n}.step", f"{PK}/{n}.step")
for n in ORIENT: PP[n]["supports"] = SUPPORT.get(n, "")
json.dump(dict(parts={n: {k: PP[n].get(k) for k in ("print_bbox", "bed_contact_mm2", "overhang_mm2_gt45", "orient", "volume_cm3", "supports")} for n in ORIENT},
               plates=plate_info), open(f"{PK}/print_report.json", "w"), indent=1)
print(json.dumps(plate_info, indent=1))
for n in ORIENT: print(f"{n:20s} bbox {PP[n]['print_bbox']}  bed {PP[n]['bed_contact_mm2']:7.1f} mm2  overhang>45 {PP[n]['overhang_mm2_gt45']:7.1f} mm2")
