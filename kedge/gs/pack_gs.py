"""Print pack for the ground station: splits, print orientation (least overhang), plates (3MF), report.
python pack_gs.py out ../pack_gs"""
import json, math, os, sys, itertools
import numpy as np
import trimesh
import manifold3d as m3
from manifold3d import Manifold, Mesh

OUT, PK = sys.argv[1], sys.argv[2]
os.makedirs(f"{PK}/print", exist_ok=True); os.makedirs(f"{PK}/plates", exist_ok=True)
import glob as _g
for _f in _g.glob(f"{PK}/print/*.stl"): os.remove(_f)      # no stale parts from earlier runs
J = json.load(open(f"{OUT}/gs_parts.json")); P = J["P"]
BED = 250.0

MAT = {  # material / infill per part (anything not listed: PLA, 4 walls, 40 %)
    "program_gear": ("PPA-CF", "100%"), "drive_pinion": ("PPA-CF", "100%"), "rack": ("PPA-CF", "100%"),
    "reel_main_gear": ("PPA-CF", "100%"), "reel_stub_gear": ("PPA-CF", "100%"), "lift_cam": ("PPA-CF", "100%"),
    "sear": ("PPA-CF", "100%"), "sear_spring_post": ("PPA-CF", "100%"), "pickup_pin": ("PPA-CF", "100%"), "drag_disc": ("PLA", "100%"), "reel_cover": ("PPA-CF", "60%"),
    "cam_roller": ("PLA", "100%"), "wiper_coupler": ("PPA-CF", "100%"), "sled": ("PPA-CF", "100%"), "rotor": ("PPA-CF", "100%"),
    "cheek_L": ("PPA-CF", "60%"), "cheek_R": ("PPA-CF", "60%"), "spring_anchor": ("PPA-CF", "60%"), "trip_anchor": ("PPA-CF", "60%"),
}
NOTES = {
    "cheek_L": "left cheek, inner face down: tree support only inside the 2.9 mm sear pocket. Pivot + PG bearings, backstop boss, cord arc slot, reel pad inserts.",
    "cheek_R": "right cheek, inner face down. Pivot + PG bearings, backstop boss.",
    "rack": "m3 rack, y up: support only under the side face of the teeth (block it off the flanks).",
    "sled": "carriage on its side (y up): keel, shoes and stem all bend along their layers. Tree supports under the pusher disc.",
    "rail_beam_A": "rear piece: rack channel. Screws onto the spring anchor post (2 x M3 csk).",
    "rail_beam_B": "middle piece: cradle, shelves, sear window side, SW_SEATED hole, bolts to C with 7 x M4 at the flange, screws onto the front tie.",
    "rail_beam_C": "front piece: muzzle, pulley forks, carriage stop blocks.",
}


def load(name):
    z = np.load(f"{OUT}/{name}.npz")
    return Manifold(Mesh(vert_properties=z["v"].astype(np.float32), tri_verts=z["f"].astype(np.uint32)))


def to_tm(s):
    m = s.to_mesh(); return trimesh.Trimesh(np.asarray(m.vert_properties)[:, :3], np.asarray(m.tri_verts), process=False)


def overhang(tm):
    n = tm.face_normals; c = tm.triangles_center; zmin = tm.bounds[0, 2]
    mask = (n[:, 2] < -0.72) & (c[:, 2] > zmin + 0.4)
    return float(tm.area_faces[mask].sum())


ROTS = []
for ax in ([1, 0, 0], [0, 1, 0], [0, 0, 1]):
    for k in range(4):
        ROTS.append(trimesh.transformations.rotation_matrix(k * math.pi / 2, ax))
R24 = []
for a, b in itertools.product(ROTS, ROTS):
    M = a @ b; key = tuple(np.round(M[:3, :3]).astype(int).flatten())
    if key not in [k for k, _ in R24]:
        R24.append((key, M))


PROFILE_Z = {"spring_anchor"}
PROFILE_Y = {"rack", "program_gear", "drive_pinion", "reel_main_gear", "reel_stub_gear", "lift_cam", "sear", "motor_plate", "cheek_L", "cheek_R", "sled"}   # tooth / plate profile in XY


def orient(tm, name):
    best = None
    for key, M in R24:
        if name in PROFILE_Y and abs(M[2, 1]) < 0.99:      # original y (gear / plate axis) must end up vertical
            continue
        if name in PROFILE_Z and abs(M[2, 2]) < 0.99:      # original z must end up vertical (layers carry the bending)
            continue
        t = tm.copy(); t.apply_transform(M)
        ext = t.extents
        fits = (ext[0] <= BED and ext[1] <= BED) or (ext[1] <= BED and ext[0] <= BED)
        if not fits or ext[2] > BED:
            continue
        oh = overhang(t)
        bed_area = float(t.area_faces[(t.face_normals[:, 2] < -0.99) & (t.triangles_center[:, 2] < t.bounds[0, 2] + 0.05)].sum())
        score = oh + 1.5 * ext[2] - 0.05 * bed_area
        if best is None or score < best[0]:
            best = (score, M, oh, ext.copy(), bed_area)
    if best is None:
        raise RuntimeError("does not fit the bed: " + name)
    t = tm.copy(); t.apply_transform(best[1])
    t.apply_translation(-t.bounds[0])
    if t.extents[0] < t.extents[1]:          # long side along X
        t.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 0, 1])); t.apply_translation(-t.bounds[0])
    return t, best[2], best[4]


items = []
for p in J["parts"]:
    if not p["printed"]:
        continue
    s = load(p["name"])
    if p["name"] == "rail_beam":
        cuts = P["RAIL_SPLITS"]
        bounds = [(-1e4, cuts[0]), (cuts[0], cuts[1]), (cuts[1], 1e4)]
        for tag, (u0, u1) in zip("ABC", bounds):
            from manifold3d import Manifold as Mf
            piece = s ^ Mf.cube((min(u1, 2000) - max(u0, -2000), 400, 400)).translate((max(u0, -2000), -200, -200))
            items.append(("rail_beam_" + tag, piece, p))
    else:
        items.append((p["name"], s, p))

report = []
for name, s, p in items:
    tm = to_tm(s)
    if len(tm.faces) == 0:
        continue
    t, oh, bed = orient(tm, name)
    t.export(f"{PK}/print/{name}.stl")
    mat, infill = MAT.get(name, ("PLA", "40%"))
    dens = 1.24 if mat == "PLA" else 1.20
    fill = 1.0 if infill == "100%" else (0.75 if infill == "60%" else 0.55)
    vol = tm.volume / 1000
    nb = len(tm.split(only_watertight=False))
    report.append(dict(name=name, dims=[round(x, 1) for x in t.extents], volume_cm3=round(vol, 1), mass_g=round(vol * dens * fill, 1),
                       material=mat, infill=infill, walls=4, supports="yes (tree, build plate only)" if oh > 150 else "no",
                       overhang_mm2=round(oh), bodies=nb, watertight=bool(tm.is_watertight),
                       note=NOTES.get(name, p["note"]), qty=1))
report.sort(key=lambda r: (r["material"], -r["volume_cm3"]))
json.dump(report, open(f"{PK}/print_report.json", "w"), indent=1)

import glob
for f_ in glob.glob(f"{PK}/plates/*.3mf"):
    os.remove(f_)            # no stale plates from earlier runs
# plates: max-rects packing on 250 x 250 (rotation allowed), PLA and PPA-CF separately
import rectpack
plates = []
for mat in ("PLA", "PPA-CF"):
    rs = [r for r in report if r["material"] == mat]
    pk = rectpack.newPacker(rotation=True, pack_algo=rectpack.MaxRectsBssf, sort_algo=rectpack.SORT_AREA)
    for i, r in enumerate(rs):
        pk.add_rect(int(math.ceil(r["dims"][0] + 6)), int(math.ceil(r["dims"][1] + 6)), rid=i)
    for _ in range(12):
        pk.add_bin(int(BED), int(BED))
    pk.pack()
    for b_ in pk:
        if not len(b_): continue
        items = []
        for rc in b_:
            r = rs[rc.rid]; w0 = int(math.ceil(r["dims"][0] + 6))
            rot = (rc.width != w0)
            items.append((r["name"], rc.x, rc.y, rot))
        plates.append(dict(mat=mat, items=items))
for i, pl in enumerate(plates):
    sc = trimesh.Scene()
    for nm, x, y, rot in pl["items"]:
        m = trimesh.load(f"{PK}/print/{nm}.stl")
        if rot:
            m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 0, 1])); m.apply_translation(-m.bounds[0])
        m.apply_translation([x + 3, y + 3, 0]); sc.add_geometry(m, node_name=nm)
    fn = f"{PK}/plates/plate_{i + 1:02d}_{pl['mat']}.3mf"
    sc.export(fn)
    pl["file"] = os.path.basename(fn)
json.dump([dict(file=pl["file"], material=pl["mat"], parts=[it[0] for it in pl["items"]]) for pl in plates], open(f"{PK}/plates.json", "w"), indent=1)

# plate preview: filled 2D shadow of each part + overlap check between footprints
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from shapely.geometry import MultiPoint
from shapely.ops import unary_union as UU
n = len(plates); cols = 3; rows = (n + cols - 1) // cols
fig, axs = plt.subplots(rows, cols, figsize=(5 * cols, 5 * rows))
overl = []
for k, ax in enumerate(np.array(axs).flatten()):
    ax.set_xlim(-2, 252); ax.set_ylim(-2, 252); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
    if k >= n: ax.axis("off"); continue
    pl = plates[k]; ax.plot([0, 250, 250, 0, 0], [0, 0, 250, 250, 0], "k-", lw=1)
    shapes = []
    for nm, x, y, rot in pl["items"]:
        m = trimesh.load(f"{PK}/print/{nm}.stl")
        if rot:
            m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 0, 1])); m.apply_translation(-m.bounds[0])
        tri = m.vertices[m.faces][:, :, :2] + [x + 3, y + 3]
        from shapely.geometry import Polygon as SP
        sh = UU([SP(t) for t in tri[::max(1, len(tri) // 4000)] if SP(t).area > 1e-3]).buffer(0.6)
        shapes.append((nm, sh))
        for g in (sh.geoms if hasattr(sh, "geoms") else [sh]):
            xs, ys = g.exterior.xy; ax.fill(xs, ys, color="#c8c8c8", ec="k", lw=0.5)
        c = sh.centroid; ax.text(c.x, c.y, nm, fontsize=7, ha="center", va="center", fontname="Arial")
    for i in range(len(shapes)):
        for j in range(i + 1, len(shapes)):
            if shapes[i][1].intersection(shapes[j][1]).area > 1.0:
                overl.append((k + 1, shapes[i][0], shapes[j][0]))
    ax.set_title("plate %d  %s" % (k + 1, pl["mat"]), fontsize=10, fontname="Arial")
fig.savefig(f"{PK}/plates.png", dpi=110, bbox_inches="tight")
print("plate overlaps:", overl if overl else "none")
tot = sum(r["mass_g"] for r in report)
print("parts", len(report), "plates", n, "printed mass %.0f g" % tot)
for r in report: print("  %-18s %-6s %5s %-4s dims %-22s %6.1f cm3 %6.1f g  supports %s bodies %d wt %s" % (r["name"], r["material"], r["infill"], "", r["dims"], r["volume_cm3"], r["mass_g"], r["supports"][:3], r["bodies"], r["watertight"]))
