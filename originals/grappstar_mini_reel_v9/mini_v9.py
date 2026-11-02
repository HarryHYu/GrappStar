"""GrappStar mini reel prototype v9: same mechanism and dimensions as v8, every rotating joint on a proper axle. The wrapping element is a second reel on the capstan's nose:
a disc that rides on a peg in the capstan's front face, carries the line eyelet on its front, turns to wrap, and is
locked by a bolt for the tow. No external ring or frame.
World: x launch, y up (base top y=0), z across; drum axis y=32; drum body z -12..11, wrap reel z 12..17, eyelet hole z 19.
CadQuery frame for world geometry: (X, Y, Z) = (x, -z, y). Parts are built in print orientation with a transform to world.
"""
import math, os
import cadquery as cq

OUT = "/home/claude/mini_v9"
os.makedirs(OUT, exist_ok=True)
PARTS = {}

def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))
def cylz(r, z0, z1, x=0, y=0):
    return cq.Workplane("XY").workplane(offset=z0).center(x, y).circle(r).extrude(z1 - z0)
def cone(r1, r2, z0, h):
    return cq.Workplane("XY").add(cq.Solid.makeCone(r1, r2, h, pnt=cq.Vector(0, 0, z0)))
def dshaft(r, z0, z1, flat=1.0):
    return cylz(r, z0, z1).cut(box(r - flat, r + 1, -r - 1, r + 1, z0 - 1, z1 + 1))
def cyl_along(r, p0, p1):
    v = cq.Vector(*p1) - cq.Vector(*p0)
    return cq.Workplane("XY").add(cq.Solid.makeCylinder(r, v.Length, cq.Vector(*p0), v))
def cone_along(r1, r2, p0, p1):
    v = cq.Vector(*p1) - cq.Vector(*p0)
    return cq.Workplane("XY").add(cq.Solid.makeCone(r1, r2, v.Length, cq.Vector(*p0), v))
def rotX(shape, deg):
    return shape.rotate((0, 0, 0), (1, 0, 0), deg)
def add(name, print_shape, world_shape, color):
    PARTS[name] = (print_shape, world_shape, color)

DR, DL = 12.0, 23.0
AXY = 32.0
SHAFT_R = 3.0
PEG_R, PEG_H, HOLE_R = 2.3, 5.0, 2.42
ER, EZ = 14.6, 20.0             # eyelet hole radius from the axis and its z (world)
LINE_Y = AXY + ER               # 48.5, height of the guide and exit eye holes
PEGS = {"reel": [(-68, -38), (-48, -38)], "guide": [(-34, -18.5), (-34, -8.5)], "capstan": [(-10, -19), (10, -19)],
        "lock": [(21, 10), (21, 20)], "eye": [(50, 15), (50, 25)]}

# ---------------------------------------------------------------- base
base = box(-82, 66, -30, 54, -4, 0)
for pts in PEGS.values():
    for (x, z) in pts:
        base = base.union(cylz(PEG_R, 0, PEG_H, x, -z)).union(cone(PEG_R, PEG_R - 0.8, PEG_H, 0.8).translate((x, -z, 0)))
add("base", base.translate((0, 0, 4)), base, "#d9d9d9")

def foot(x0, x1, z0, z1):
    return box(x0, x1, -z1, -z0, 0, 4)
def peg_holes(shape, pegs):
    for (x, z) in pegs:
        shape = shape.cut(cylz(HOLE_R, -1, 6.2, x, -z))
    return shape

# ---------------------------------------------------------------- capstan bracket (world frame)
cb = foot(-15, 15, -33, -5).union(box(-15, 15, 15, 23, 0, 46))
cb = peg_holes(cb, PEGS["capstan"]).cut(cyl_along(3.2, (0, 10, AXY), (0, 30, AXY)))
add("capstan_bracket", cb, cb, "#bdbdbd")

# ---------------------------------------------------------------- drum (print: base flange face down). D-bore through, 1 mm thrust boss on the nose
# print coords: Z_p = world z + 14.5 (rear flange face at z -14.5 on the bed); cone flange 28 -> 24 over 2.5, body 23, boss 1
drum = cone(DR + 2, DR, 0, 2.5).union(cylz(DR, 2.5, 2.5 + DL)).union(cylz(4.5, 2.5 + DL, 3.3 + DL))
drum = drum.cut(dshaft(3.1, -1, 30, flat=1.05))                      # D-bore 6.2, keyed to the axle
drum = drum.edges(">Z").chamfer(0.5)
drum_w = rotX(drum, 90).translate((0, 14.5, AXY))                     # Z_p -> +z world; rear face at z -14.5
add("drum", drum, drum_w, "#2e9c86")

# ---------------------------------------------------------------- capstan axle (print lying on its flat): 6 D-rod, z -29 .. +18, M3 hole in the front end
AXL = 47.3
axle = dshaft(3.0, 0, AXL, flat=1.0).faces("<Z").chamfer(0.5).faces(">Z").chamfer(0.5).cut(cylz(1.25, AXL - 8, AXL + 1))
axle_print = axle.rotate((0, 0, 0), (0, 1, 0), 90).translate((0, 0, 2.0))   # lying down, flat on the bed
axle_w = rotX(axle, 90).translate((0, 29, AXY))                      # Z_p -> +z world: z -29 .. +18.3
add("axle_capstan", axle_print, axle_w, "#555555")

# ---------------------------------------------------------------- cap (print flat): 9 mm washer, countersunk M3 x 8 into the axle end, holds the wrap reel on
cap = cylz(4.5, 0, 1.6).cut(cylz(1.7, -1, 3)).cut(cone(1.7, 3.2, 0.1, 1.5))
cap_w = rotX(cap, -90).translate((0, -19.9, AXY))                    # world z 18.3 .. 19.9, countersink facing front
add("cap", cap, cap_w, "#9a94e8")

# ---------------------------------------------------------------- wrap reel (print: rear face down). Round 6.4 bore, rides on the axle between the drum's nose boss and the cap
wr = cylz(15, 0, 6)
wr = wr.edges(">Z").chamfer(1.0)
wr = wr.cut(cylz(3.2, -1, 7))
lug = cylz(2.7, 0, 10.4, 0, ER).faces(">Z").fillet(1.3)
wr = wr.union(lug)
wr = wr.cut(cyl_along(1.3, (-5, ER, 8), (5, ER, 8)))
wr = wr.cut(cone_along(1.9, 1.3, (-2.71, ER, 8), (-2.1, ER, 8))).cut(cone_along(1.3, 1.9, (2.1, ER, 8), (2.71, ER, 8)))
wr = wr.cut(cyl_along(1.7, (16, 0, 3), (9, 0, 3)))
wr_w = rotX(wr, 90).translate((0, -12, AXY))
add("wrap_reel", wr, wr_w, "#7f77dd")

# ---------------------------------------------------------------- lock bracket (world frame): M3 bolt through it into the wrap reel rim
lb = foot(13, 29, 7, 23).union(box(21, 27, -18, -12, 0, 36))
lb = peg_holes(lb, PEGS["lock"]).cut(cyl_along(1.25, (18, -15, AXY), (30, -15, AXY)))
add("lock_bracket", lb, lb, "#bdbdbd")

# ---------------------------------------------------------------- crank x2 (print flat)
crank = cylz(11, 0, 3).union(cylz(2.5, 3, 13, 8, 0)).cut(dshaft(3.2, -1, 4, flat=1.05))
add("crank", crank, rotX(crank, -90).translate((0, 23.5, AXY)), "#9a94e8")
PARTS["crank_2"] = (None, rotX(crank, -90).translate((-58, 42.5, AXY)), "#9a94e8")

# ---------------------------------------------------------------- reel bracket + spool (base end)
rb = foot(-73, -43, -52, -24).union(box(-73, -43, 34, 42, 0, 46))
rb = peg_holes(rb, PEGS["reel"]).cut(cyl_along(3.2, (-58, 28, AXY), (-58, 48, AXY)))
add("reel_bracket", rb, rb, "#bdbdbd")
# spool print: front flange face down. Z_p = world (-10 - z): front flange z -10..-12, barrel -12..-28, cone flange -28..-32, boss -32..-33.5
spool = cylz(16, 0, 2).union(cylz(12, 2, 18)).union(cone(12, 16, 18, 4)).union(cylz(6, 22, 23.5))
spool = spool.cut(dshaft(3.1, -1, 30, flat=1.05))
add("spool", spool, rotX(spool, -90).translate((-58, 10, AXY)), "#7f77dd")
AXS = 38.0
axs = dshaft(3.0, 0, AXS, flat=1.0).faces("<Z").chamfer(0.5).faces(">Z").chamfer(0.5)
axs_print = axs.rotate((0, 0, 0), (0, 1, 0), 90).translate((0, 0, 2.0))
axs_w = rotX(axs, 90).translate((-58, 48, AXY))                      # world z -48 .. -10
add("axle_spool", axs_print, axs_w, "#555555")

# ---------------------------------------------------------------- eye posts x2 (world frame): guide at the base end, exit eye in the eyelet plane
def eye_post(x, z):
    p = foot(x - 8, x + 8, z - 8, z + 8)
    p = p.union(box(x - 3, x + 3, -z - 3, -z + 3, 0, 40)).union(box(x - 2, x + 2, -z - 3, -z + 3, 40, 52))
    p = peg_holes(p, [(x, z - 5), (x, z + 5)])
    p = p.cut(cyl_along(1.7, (x - 5, -z, LINE_Y), (x + 5, -z, LINE_Y)))
    p = p.cut(cone_along(2.4, 1.7, (x - 2.01, -z, LINE_Y), (x - 1.3, -z, LINE_Y))).cut(cone_along(1.7, 2.4, (x + 1.3, -z, LINE_Y), (x + 2.01, -z, LINE_Y)))
    return p
guide = eye_post(-34, -13.5)
eye = eye_post(50, 20)
add("eye_post", guide, guide, "#bdbdbd")
PARTS["eye_post_2"] = (None, eye, "#bdbdbd")

# ---------------------------------------------------------------- line for the render only
pts = [(-58, AXY + 12.75, -13.5), (-34, LINE_Y, -13.5), (0, LINE_Y, EZ), (50, LINE_Y, EZ), (66, LINE_Y, EZ)]
line = None
for a, b in zip(pts[:-1], pts[1:]):
    seg = cyl_along(0.6, (a[0], -a[2], a[1]), (b[0], -b[2], b[1]))
    line = seg if line is None else line.union(seg)
PARTS["line"] = (None, line, "#e8892b")

for name, (p, w, c) in PARTS.items():
    if p is not None:
        cq.exporters.export(p, f"{OUT}/{name}.stl", tolerance=0.02, angularTolerance=0.15)
        print("wrote", name)
    cq.exporters.export(w, f"{OUT}/_world_{name}.stl", tolerance=0.03, angularTolerance=0.25)
print("done")
