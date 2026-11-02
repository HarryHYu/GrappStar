"""GrappStar reel, full size v5 (21 Sep 2026). The line hole is a 4 mm ring on the cup's face edge (0.5 mm in front of the face), every visible hole round, spool held by a second C-clip. Sized for 20 m of 1 mm Dyneema: dia 40 capstan, spool that holds 25 m, 8 mm axles, all snap ends.
Same mechanism as the mini v18: storage spool, capstan drum, wrap cup with a tilted lug and a helical line groove, lock pin, one-piece frame.
World: x launch, y up (frame top y=0), z across; axles at y=44. Drum body z -17..23, cup z 16..30, lug hole z 35.6, exit eye z 62.
CadQuery frame for world geometry: (X, Y, Z) = (x, -z, y). Moving parts are built in print orientation with a transform to world.
"""
import math, os
import cadquery as cq

OUT = "/home/claude/full_v7"
os.makedirs(OUT, exist_ok=True)
PARTS = {}

def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))
def cylz(r, z0, z1, x=0, y=0):
    return cq.Workplane("XY").workplane(offset=z0).center(x, y).circle(r).extrude(z1 - z0)
def cone(r1, r2, z0, h, x=0, y=0):
    return cq.Workplane("XY").add(cq.Solid.makeCone(r1, r2, h, pnt=cq.Vector(x, y, z0)))
def dshaft(r, z0, z1, flat):
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
def try_fillet(wp, selector, r):
    try:
        return wp.edges(selector).fillet(r)
    except Exception as e:
        print("  fillet skipped:", selector, r, e)
        return wp
def fillet_box(wp, lo, hi, r):
    """fillet every edge whose bounding box lies inside lo..hi"""
    try:
        sel = [e for e in wp.edges().vals() if all(lo[i] - 1e-3 <= v <= hi[i] + 1e-3 for bb in [e.BoundingBox()] for i, v in enumerate((bb.xmin, bb.ymin, bb.zmin))) and all(lo[i] - 1e-3 <= v <= hi[i] + 1e-3 for bb in [e.BoundingBox()] for i, v in enumerate((bb.xmax, bb.ymax, bb.zmax)))]
        if not sel: print("  fillet_box: no edges", lo, hi); return wp
        return wp.newObject(sel).fillet(r)
    except Exception as e:
        print("  fillet_box skipped:", lo, hi, r, e)
        return wp
def wedge(plane, origin, pts, depth, r=1.0):
    """triangular prism with its sloped edges rounded"""
    w = cq.Workplane(plane, origin=origin).polyline(pts).close().extrude(depth)
    try:
        sel = [e for e in w.edges().vals() if not any(abs(abs(e.tangentAt(0.5).normalized().dot(cq.Vector(*ax))) - 1) < 1e-6 for ax in ((1, 0, 0), (0, 1, 0), (0, 0, 1)))]
        w = w.newObject(sel).fillet(r)
    except Exception as e:
        print("  wedge fillet skipped:", e)
    return w

# ---------------------------------------------------------------- sizes
LINE = 1.0                     # Dyneema
BT = 4.0                       # base plate
AXY = 44.0                     # axle height above the plate
DR, DL = 20.0, 40.0            # capstan root radius / body length (dia 40: 8 Nm stall -> 400 N)
AR = 4.0                       # axle radius (8 mm D rod)
AFLAT = 1.3                    # flat depth on the axle (flat plane 2.7 from the axis)
CUP_IN, CUP_OUT, CUP_D = 20.6, 24.0, 14.0     # wrap cup: bore over the drum, outside, depth
ER = 23.5                      # lug hole radius = line centre radius in the groove (groove floor 23.0)
EZP = 14.5                     # lug hole height above the cup's rim plane: 0.5 in front of the face, the least that clears the dart line
RING_L = 2.0                   # the ring is 4 mm long: this far either side of the lug position, tangentially
RING_R = 3.4                   # ring outside radius (2 mm wall round the dia 2.8 hole)
RING_AT = ER + 1.0             # 24.5: ring axis 1 mm outside the cup wall, so the line from the guide reaches the rear mouth without touching the cup's edge
CUP_Z0 = 16.0                  # cup rim plane, world z
EZ = CUP_Z0 + EZP              # 35.6, lug hole world z
LINE_Y = AXY + ER              # 67.5, guide and exit eye height
EYE_Z = 62.0                   # exit eye, 28 mm past the ring
GUIDE_Y = AXY + CUP_OUT + 2.0  # 70: guide eye centre, 2 mm above the top of the cup
ALPHA = 30.0                   # groove helix angle from the rim plane
SPOOL_X, SPOOL_ZC = -85.0, -21.0
GUIDE_X = -45.0
pitch = 2 * math.pi * CUP_OUT * math.tan(math.radians(ALPHA))   # 87.1 mm per turn on the cup surface
dpm = 360.0 / pitch                                            # 4.13 deg of wrap per mm of rise
TH_TOP = 90 + math.degrees(RING_L / ER)                        # groove starts under the ring's rear mouth, on the face edge
TH_RIM = TH_TOP + dpm * CUP_D                                  # and crosses the rim here

# ---------------------------------------------------------------- one-piece frame (world frame; printed on its underside)
def pad(x0, x1, z0, z1):
    p = box(x0, x1, -z1, -z0, -BT, 0).edges("|Z").fillet(6)
    return try_fillet(p, ">Z", 1.5)
def upright(x0, x1, z0, z1, h):
    u = box(x0, x1, -z1, -z0, 0, h).edges("|Z").fillet(2)
    return try_fillet(u, ">Z", 2)
def gusset(x0, x1, zface, direction, h, d):
    prof = [(-zface, 0), (-zface - direction * d, 0), (-zface, h)]
    return wedge("YZ", (x0, 0, 0), prof, x1 - x0)
frame = pad(-118, 85, -80, 72)
def foot(fr, x0, x1, y0, y1, r=1.5):
    """round the junction of a feature's footprint with the plate (uprights and posts only; gusset feet stay sharp, OCC will not fillet them cleanly)"""
    if r < 1.5: return fr
    return fillet_box(fr, (x0 - 0.05, y0 - 0.05, -0.01), (x1 + 0.05, y1 + 0.05, 0.01), r)
frame = foot(frame.union(upright(-16, 16, -31, -21, 56)), -16, 16, 21, 31)                # capstan bracket
for gx in ((-16, -11), (11, 16)):
    frame = foot(frame.union(gusset(gx[0], gx[1], -21, 1, 18, 11)), gx[0], gx[1], 10, 21, 1.0)
    frame = foot(frame.union(gusset(gx[0], gx[1], -31, -1, 18, 11)), gx[0], gx[1], 31, 42, 1.0)
frame = foot(frame.union(upright(-103, -67, -58, -48, 56)), -103, -67, 48, 58)             # reel bracket
for gx in ((-103, -98), (-72, -67)):
    frame = foot(frame.union(gusset(gx[0], gx[1], -48, 1, 18, 11)), gx[0], gx[1], 37, 48, 1.0)
    frame = foot(frame.union(gusset(gx[0], gx[1], -58, -1, 18, 11)), gx[0], gx[1], 58, 69, 1.0)
frame = foot(frame.union(upright(30, 40, 14, 26, 52)), 30, 40, -26, -14)                    # lock post
frame = foot(frame.union(gusset(30, 40, 26, 1, 16, 8)), 30, 40, -34, -26, 1.0)
frame = foot(frame.union(gusset(30, 40, 14, -1, 16, 8)), 30, 40, -14, -6, 1.0)
def post(fr, x, z, ey):
    """post with a plain round eye at the top: dia 4.4 hole along x through a 6 mm wall, coned out to dia 6.4 at both faces"""
    p = box(x - 5, x + 5, -z - 5, -z + 5, 0, 50).edges("|Z").fillet(2)
    p = try_fillet(p, ">Z", 1.5)
    top = box(x - 3, x + 3, -z - 5, -z + 5, 50, ey + 6).edges("|Z").fillet(1.5)
    top = try_fillet(top, ">Z", 2.5)
    p = p.union(top)
    p = p.cut(cyl_along(2.2, (x - 6, -z, ey), (x + 6, -z, ey)))
    p = p.cut(cone_along(3.2, 2.2, (x - 3.01, -z, ey), (x - 1.5, -z, ey))).cut(cone_along(2.2, 3.2, (x + 1.5, -z, ey), (x + 3.01, -z, ey)))
    fr = foot(fr.union(p), x - 5, x + 5, -z - 5, -z + 5)
    fr = foot(fr.union(wedge("XZ", (0, -z - 5, 0), [(x - 5, 0), (x - 13, 0), (x - 5, 20)], -10)), x - 13, x - 5, -z - 5, -z + 5, 1.0)
    fr = foot(fr.union(wedge("XZ", (0, -z - 5, 0), [(x + 5, 0), (x + 13, 0), (x + 5, 20)], -10)), x + 5, x + 13, -z - 5, -z + 5, 1.0)
    return fr
frame = post(frame, GUIDE_X, SPOOL_ZC, GUIDE_Y)                                           # guide eye, 2 mm above the top of the cup
frame = post(frame, 70, EYE_Z, LINE_Y)                                                    # exit eye past the cup, on the lug hole's height
# ---- windows: the plate becomes a frame. Keep a 10 mm border, 8 mm round every foot, a spar down the capstan line and one across the guide line.
def keep(x0, x1, z0, z1):
    return box(x0, x1, -z1, -z0, -BT - 1, 1)
cutter = box(-108, 75, -62, 70, -BT - 1, 1)
for k in (keep(-24, 24, -50, -2), keep(-111, -59, -77, -29), keep(22, 48, -2, 42), keep(-66, -24, -34, -8), keep(49, 91, 49, 75),
          keep(-8, 8, -80, 72), keep(-118, 85, -10, 6)):
    cutter = cutter.cut(k)
try:
    cutter = cutter.edges("|Z").fillet(4)
except Exception as e:
    print("  window corner fillet skipped:", e)
frame = frame.cut(cutter)
frame = (frame.cut(cyl_along(AR + 0.25, (0, 15, AXY), (0, 35, AXY)))                  # capstan axle hole
              .cut(cyl_along(AR + 0.25, (SPOOL_X, 44, AXY), (SPOOL_X, 62, AXY)))       # spool axle hole
              .cut(cyl_along(2.15, (26, -20, AXY), (44, -20, AXY))))                   # lock pin hole
add("frame", frame.translate((0, 0, BT)), frame, "#d0d0d0")

# ---------------------------------------------------------------- drum (print: base flange face down). Keyed bore, recess in the nose for the cup hub
drum = cone(DR + 3, DR, 0, 3).union(cylz(DR, 3, 3 + DL))
drum = try_fillet(drum, ">Z", 1.2).faces("<Z").chamfer(0.8)
drum = drum.cut(dshaft(AR + 0.2, -1, 3 + DL + 1, flat=AFLAT + 0.05))
drum = drum.cut(cylz(7.6, 3 + DL - 7.3, 3 + DL + 1)).cut(cone(8.4, 7.6, 3 + DL - 0.8, 0.81))       # 7.3 deep: cup hub stops 0.3 short of the floor
drum = drum.cut(cone(5.0, 4.2, -0.01, 0.81).cut(box(AR + 0.2 - AFLAT - 0.05, 7, -7, 7, -1, 2)))   # bore mouth at the flange, follows the D
add("drum", drum, rotX(drum, 90).translate((0, 20, AXY)), "#2e9c86")

# ---------------------------------------------------------------- axles (print lying on the flat). Rear: 8.2 head + 7.6 neck the crank snaps over.
HEAD, NECK = 2.5, 3.0
def snap_axle(D_len, front, round_len=0.0):
    a = dshaft(AR + 0.1, 0, HEAD, flat=AFLAT + 0.1).faces("<Z").chamfer(0.6)          # head 8.2, flat plane 2.7 like the rod
    a = a.union(dshaft(AR - 0.2, HEAD, HEAD + NECK, flat=AFLAT - 0.2))                 # neck 7.6
    a = a.union(dshaft(AR, HEAD + NECK, HEAD + NECK + D_len, flat=AFLAT))              # D rod 8.0
    e = HEAD + NECK + D_len
    if round_len:                                                                     # plain round 8.0 section in front of the D
        a = a.union(cylz(AR, e, e + round_len)); e += round_len
    if front == "snap":                                                               # spool: same head and neck at the front
        a = a.union(dshaft(AR - 0.2, e, e + NECK, flat=AFLAT - 0.2)).union(dshaft(AR + 0.1, e + NECK, e + NECK + HEAD, flat=AFLAT + 0.1).faces(">Z").chamfer(0.6))
    if front == "stub":                                                               # capstan: 4.5 neck + 5.6 head for the push-nut cap
        a = a.union(cylz(2.25, e, e + 2.6)).union(cylz(2.8, e + 2.6, e + 4.1).faces(">Z").chamfer(0.5))
    return a
CAP_D = 69.8
axle = snap_axle(CAP_D, "stub")                                                       # world: head -45.3..-42.8, neck ..-39.8, rod ..30.0, neck ..32.6, head ..34.1
add("axle_capstan", axle.rotate((0, 0, 0), (0, 1, 0), 90).translate((0, 0, AR - AFLAT)), rotX(axle, 90).translate((0, 45.3, AXY)), "#555555")

# ---------------------------------------------------------------- cap (print flat): C-clip, dia 11 x 2, dia 4.6 bore, 3.8 mm opening. Pushed sideways onto the 4.5 neck, the 5.6 head keeps it there.
cap = cylz(5.5, 0, 2.0).cut(cylz(2.3, -1, 3)).cut(box(-1.9, 1.9, -7, 0, -1, 3))
cap = try_fillet(cap, ">Z", 0.5)
cap = try_fillet(cap, "<Z", 0.5)
add("cap", cap, rotX(cap, -90).translate((0, -32.3, AXY)), "#9a94e8")                # world z 30.3 .. 32.3, in the neck, opening upward
PARTS["cap_2"] = (None, rotX(cap, -90).translate((SPOOL_X, 2.7, AXY)), "#9a94e8")       # spool clip, z -4.7 .. -2.7

# ---------------------------------------------------------------- wrap cup (print: open end down, lug up)
cup = cylz(CUP_OUT, 0, CUP_D)
cup = try_fillet(cup, ">Z", 1.0)                                                      # rounded face edge
cup = cup.cut(cylz(CUP_IN, -1, CUP_D - 2.0))
cup = cup.edges("<Z").chamfer(1.0)                                                    # rim: both edges, flared mouth over the drum
cup = cup.union(cylz(7.0, 0, CUP_D))
# ring: a short eyelet on the cup's face edge, axis tangential, 0.5 mm in front of the face. A 45 deg block under it keeps it printable.
ring = cyl_along(RING_R, (-RING_L, RING_AT, EZP), (RING_L, RING_AT, EZP))
block = box(-RING_L, RING_L, 22.0, RING_AT + 2.0, 5.5, EZP).intersect(cone(24.0, 40.0, 5.5, 16.0))
lug = ring.union(block)
boss = box(20.0, 28.0, -4.0, 4.0, 0, 8.0).edges("|Z").fillet(1.5)
boss = try_fillet(boss, ">Z", 1.5)
cup = cup.union(lug).union(boss)
cup = cup.cut(cylz(CUP_IN, -1, CUP_D - 2.0).cut(cylz(7.0, -2, CUP_D - 1)))            # clear the cavity again (lug and boss roots)
cup = cup.cut(cylz(AR + 0.3, -1, CUP_D + 1))                                          # round 8.6 bore: the cup turns on the D rod
cup = cup.cut(cone(5.0, AR + 0.3, -0.01, 0.71)).cut(cone(AR + 0.3, 5.0, CUP_D - 0.7, 0.71))   # bore mouths
# line hole: dia 2.8 tangential through the ring at the groove's line radius, both mouths coned out on the ring's flat ends
cup = cup.cut(cyl_along(1.4, (-RING_L - 1, RING_AT, EZP), (RING_L + 1, RING_AT, EZP)))
cup = cup.cut(cone_along(2.0, 1.4, (-RING_L - 0.01, RING_AT, EZP), (-RING_L + 0.6, RING_AT, EZP))).cut(cone_along(1.4, 2.0, (RING_L - 0.6, RING_AT, EZP), (RING_L + 0.01, RING_AT, EZP)))
cup = cup.cut(cyl_along(2.15, (29, 0, 4.0), (21.2, 0, 4.0)))                          # lock pin socket, 0.6 of wall left
# helical line groove, U section: 1.2 wide across the line, 1.0 deep with a rounded floor, 30 deg helix, about 75 deg of wrap.
# The line leaves the rear mouth of the lug, runs 6.3 mm in free flight down to the face edge, drops into the groove and runs out through the rim.
# Both ends of the groove are funnelled.
GW = 1.2                                                                              # width across the line
helix = cq.Wire.makeHelix(pitch, 20, 24.3)
grv = (cq.Workplane("XZ").center(24.3, 0).slot2D(2.6, GW / math.cos(math.radians(ALPHA)), 0)
       .sweep(cq.Workplane().add(helix), isFrenet=True).mirror("YZ").translate((0, 0, -4)))     # floor at r 23.0, open past the surface
grv = grv.rotate((0, 0, 0), (0, 0, 1), (TH_TOP + dpm * (CUP_D + 4)) - 180).intersect(box(-30, 30, -30, 30, -3, CUP_D + 0.5))
def helix_pt_dir(th_deg, z):
    """point on the line's path (r 23.6) and its unit tangent heading down the helix, print frame"""
    th = math.radians(th_deg); ca, sa = math.cos(math.radians(ALPHA)), math.sin(math.radians(ALPHA))
    return (23.6 * math.cos(th), 23.6 * math.sin(th), z), (-math.sin(th) * ca, math.cos(th) * ca, -sa)
for th_deg, z, r0, r1 in ((TH_TOP, CUP_D, 1.6, 0.6), (TH_RIM, 0.0, 0.6, 1.8)):
    p, d = helix_pt_dir(th_deg, z)
    cup = cup.cut(cone_along(r0, r1, tuple(p[i] - 3.0 * d[i] for i in range(3)), tuple(p[i] + 3.0 * d[i] for i in range(3))))
cup = cup.cut(grv)
add("wrap_cup", cup, rotX(cup, 90).translate((0, -CUP_Z0, AXY)), "#7f77dd")

# ---------------------------------------------------------------- lock pin (print head down): 4.15 shaft in 4.3 holes, snug
pin = cylz(4.0, 0, 3).union(cylz(2.075, 3, 21.8))
pin = try_fillet(pin, ">Z", 0.8).faces("<Z").chamfer(0.6)
add("lock_pin", pin, pin.rotate((0, 0, 0), (0, 1, 0), -90).translate((43, -20, AXY)), "#333333")

# ---------------------------------------------------------------- cranks x2 (print flat, hub up). Round bore; the D flat is in the 7.9 lip, which sits in the axle's neck and snaps over its 8.2 head
crank = cylz(16, 0, 4)
crank = try_fillet(crank, ">Z", 1.0).faces("<Z").chamfer(0.6)
crank = crank.union(cylz(6.2, 3, 13)).union(cylz(3.5, 3, 18, 12, 0))
crank = try_fillet(crank, ">Z", 1.2)
crank = fillet_box(crank, (-6.3, -6.3, 12.9), (6.3, 6.3, 13.1), 1.0)
crank = crank.cut(cylz(AR + 0.25, -1, 9.3))                                           # plain round bore
crank = crank.cut(dshaft(3.95, 8.9, 11.5, flat=1.1)).cut(cylz(4.5, 11.5, 14.5))        # 7.9 lip with the D flat (this is the key), then open chamber for the 8.2 head
for a in (0, 90):
    crank = crank.cut(box(-0.45, 0.45, -8, 8, 4.0, 15).rotate((0, 0, 0), (0, 0, 1), a))    # four slots, from the disc top
add("crank", crank, rotX(crank, -90).translate((0, 31.3, AXY)), "#9a94e8")            # disc z -31.3..-35.3, hub to -44.3
PARTS["crank_2"] = (None, rotX(crank, -90).translate((SPOOL_X, 58.3, AXY)), "#9a94e8") # disc z -58.3..-62.3, hub to -71.3

# ---------------------------------------------------------------- spool (print front flange down): barrel dia 28 x 26, flanges dia 52, holds ~25 m of 1 mm line. Trapped between the bracket and a C-clip
spool = cylz(26, 0, 3)
spool = try_fillet(spool, ">Z", 1.0).faces("<Z").chamfer(0.8)
spool = spool.union(cylz(14, 2, 29)).union(cone(14, 26, 29, 8)).union(cylz(26, 37, 40))
spool = fillet_box(spool, (-26.1, -26.1, 36.9), (26.1, 26.1, 37.1), 1.5)
spool = try_fillet(spool, ">Z", 1.0)                                                  # rim top edge
spool = spool.union(cylz(8, 40, 42))
spool = try_fillet(spool, ">Z", 1.0)                                                  # hub top edge
spool = spool.cut(cylz(AR + 0.25, -1, 20)).cut(dshaft(AR + 0.25, 20, 43, flat=AFLAT + 0.1))   # round bore at the front, keyed D at the back
spool = spool.cut(cone(5.0, AR + 0.25, -0.01, 0.76))                                   # round mouth at the front face
add("spool", spool, rotX(spool, -90).translate((SPOOL_X, 5, AXY)), "#7f77dd")        # front flange at z -5, rear hub to -47, bracket at -48
SP_D = 41.8
axs = snap_axle(SP_D, "stub", round_len=20.3)                                         # world z -72.3 .. -0.6: crank head at the back, D under the spool's rear half, round under the front, clip stub in front
add("axle_spool", axs.rotate((0, 0, 0), (0, 1, 0), 90).translate((0, 0, AR - AFLAT)), rotX(axs, 90).translate((SPOOL_X, 72.3, AXY)), "#555555")

# ---------------------------------------------------------------- line for the render only
pts = [(SPOOL_X, AXY + 17, SPOOL_ZC), (GUIDE_X, GUIDE_Y, SPOOL_ZC), (-RING_L, AXY + RING_AT, EZ), (RING_L, AXY + RING_AT, EZ), (70, LINE_Y, EYE_Z), (95, LINE_Y, EYE_Z)]
line = None
for a, b in zip(pts[:-1], pts[1:]):
    seg = cyl_along(0.5, (a[0], -a[2], a[1]), (b[0], -b[2], b[1]))
    line = seg if line is None else line.union(seg)
PARTS["line"] = (None, line, "#e8892b")

import trimesh
def clean(path):
    m = trimesh.load(path)
    parts = [q for q in m.split(only_watertight=False) if abs(q.volume) > 1.0]      # drop sliver triangles the tessellator leaves at fillet corners
    if len(parts) != len(m.split(only_watertight=False)):
        m = trimesh.util.concatenate(parts) if len(parts) > 1 else parts[0]
        m.export(path)
    return m
for name, (p, w, c) in PARTS.items():
    if p is not None:
        cq.exporters.export(p, f"{OUT}/{name}.stl", tolerance=0.02, angularTolerance=0.15)
        m = clean(f"{OUT}/{name}.stl")
        print("wrote", name, "watertight" if m.is_watertight else "NOT WATERTIGHT", len(m.split(only_watertight=False)), "body")
    cq.exporters.export(w, f"{OUT}/_world_{name}.stl", tolerance=0.03, angularTolerance=0.25)
    if name != "line": clean(f"{OUT}/_world_{name}.stl")
print("TH_TOP %.1f TH_RIM %.1f wrap %.1f deg" % (TH_TOP, TH_RIM, TH_RIM - TH_TOP))
print("done")
