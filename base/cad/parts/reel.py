"""GrappStar base, reel system (worker cad-reel): storage reel + slip clutch + capstan (backstopped on sprag #2) +
Harry's v7 wrap cup, all on one printed reel wall. Module contract: base/cad/README.md (parts, joints, kin, ...).

Reel frame R (reel_NOTES.md section 2): origin on the capstan axis at the front face of the reel wall, +Z along the
capstan axis toward the cup, +X forward (launch direction), +Y up. R = Harry's v7 world frame shifted by
(0, -44, +42), so every v7 line rule carries over unchanged. R is placed in the world by joint reel_base
(rotation Rx(90): R +Y -> world +Z, R +Z -> world -Y), origin from P.LAYOUT["reel"] when the spec gives it.

Senses (rotation about +Z of R): capstan wind-in +, storage reel take-up +, cup WRAP -, cup UNWRAP +.
Numbers that other workers rely on are in reel_NOTES.md; physics in reel_calc.py.
"""
from __future__ import annotations

import math

import numpy as np

from cad.lib import Part, Joint, Allow, Clearance, prim, tf, fits, gears, bearings, shafts, clips, line
from cad.parts import reel_calc, reel_v7 as v7

# ------------------------------------------------------------------------------------------------ layout (R frame)
WALL_T = 16.0                 # wall Z -16 .. 0
FOOT_T = 5.0
H_AXIS = 45.0                 # capstan axis above the deck top
Y_DECK = -H_AXIS              # deck top in R
X_WALL = (-122.0, 30.0)
Y_WALL_TOP = 44.0
Z_FOOT = (-WALL_T, 42.0)

Z_GUIDE = 21.0                # guide eye plane = 1 mm behind the drum flange back (Harry v7)
Z_DRUM = Z_GUIDE + 1.0        # drum flange back face
Z_BLOCK = Z_GUIDE             # capstan block front face = sprag seat mouth
Z_CUP = Z_DRUM + v7.FLANGE_H + v7.DL - 7.0      # cup rim plane (cup skirt 7 mm over the drum, v7)
GUIDE = np.array((-45.0, v7.CUP_OUT + 2.0, Z_GUIDE))            # guide eye centre
EXIT = np.array((70.0, v7.ER, Z_CUP + v7.EZP + 31.5))           # exit eye centre, 31.5 past the ring (v7)

X_REEL = -85.0                # storage reel axis (Y 0)
Z_T = 3.9                     # thrust ring T rear face
Z_SPOOL = 6.0                 # spool rear face (rear 608 mouth)
SPOOL_LEN = 26.0              # rear face .. front flange face
ANNULUS = (13.0, 19.0, 0.3)   # clutch friction annulus on the spool front face: r_in, r_out, raised
Z_D = Z_SPOOL + SPOOL_LEN + ANNULUS[2]   # clutch disk D (head of S) rear face
D_T, D_R = 3.0, 20.0

ROCKER_PIVOT = np.array((X_REEL, 37.0, 3.0))
ROCKER_Z = (0.5, 3.5)
PUSH_Y = -35.0                # pushrod axis height (R)
PUSH_D = 5.0
BRAKE_STROKE = 4.0            # drum pushes the rod +Z this far for REEL
PUSH_COLLAR_R, PUSH_COLLAR_T, PUSH_CB = 4.5, 1.5, 5.0   # collar on the rod, counterbore depth in the wall front

GEAR_Z = (-27.0, -17.0)       # take-up plane (take-up gear on C, idler, reel gear), face 10
GC_Z = (-43.0, -29.0)         # capstan gear plane (meshes the drum's m3 24T REEL output gear), face 14
CUPGEAR_Z = (-56.5, -48.5)    # cup ring gear plane (meshes the drum's m1.5 48T wrap idler), face 8
M_CAP, Z_CAPG, X_CAPG = 3.0, 12, 0.3                    # capstan gear
M_TAKE, Z_TAKE, Z_IDL, Z_REELG = 2.0, 24, 24, 15         # take-up train
M_CUPG, Z_CUPG = 1.5, 24

C_D, K_D, S_D = 20.0, 10.0, 8.0
C_BORE = K_D + fits.SHAFT_IN_BORE
C_Z = (-47.6, None)           # C rear end; front end set from the drum web
K_Z = (-60.0, None)
S_Z0 = -30.5

COL = dict(wall="#b9c2cc", ppa="#3a3f45", gear="#d97a00", drum="#2e9c86", cup="#7f77dd", spool="#7f77dd",
           clip="#9a94e8", brg="#8a8f96", rocker="#e0b040", post="#4a5058")


def _d(P):
    return reel_calc.get(P)


def idler_xy():
    """Idler centre: meshes the capstan gear (at 0, 0) and the reel gear (at X_REEL, 0), below the line between them."""
    c1 = gears.center_distance(M_TAKE, Z_TAKE, Z_IDL)
    c2 = gears.center_distance(M_TAKE, Z_IDL, Z_REELG)
    d = abs(X_REEL)
    x = (c1 ** 2 - c2 ** 2 + d ** 2) / (2 * d)
    y = -math.sqrt(max(c1 ** 2 - x ** 2, 0.0))
    return np.array((-x, y))


def drum_web_z():
    return Z_DRUM + v7.drum_info()["z_web0"]


def c_front():
    return drum_web_z() - 0.3


def k_front():
    return Z_CUP + v7.CUP_D       # cup face plane: K's 10 mm body ends here, the clip neck follows


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def guide_axis():
    """Guide eye axis: bisects the line from the reel (+X) and the launch line to the ring (v7 keeps the eye along X;
    the launch line then leaves 50 deg off the axis and grazes the post, so the plain round eye is turned 25 deg)."""
    ring = np.array((-v7.RING_L, v7.RING_AT, Z_CUP + v7.EZP))
    return _unit(_unit((1.0, 0.0, 0.0)) + _unit(ring - GUIDE))


def exit_axis():
    """Exit eye axis: bisects the mean incoming dart line (from the cup face centre) and the outgoing +X."""
    c = np.array((0.0, 0.0, Z_CUP + v7.EZP))
    return _unit(_unit(EXIT - c) + _unit((1.0, 0.0, 0.0)))


def _eye_cut(centre, axis, wall):
    """Plain round v7 eye (d 4.4, coned to 6.4) through a wall of thickness `wall` along X, axis tilted to `axis`."""
    L = wall / abs(axis[0])
    return prim.place(line.eye_cutter(L), tf.frame(tuple(np.asarray(centre) - axis * L / 2), tuple(axis)))


# ------------------------------------------------------------------------------------------------ solids
def _wall():
    x0, x1 = X_WALL
    foot = prim.box(x0, x1, Y_DECK, Y_DECK + FOOT_T, Z_FOOT[0], Z_FOOT[1])
    wall = prim.box(x0, x1, Y_DECK, Y_WALL_TOP, -WALL_T, 0.0)
    # capstan block round the sprag seat
    rb = bearings.CSK20_OD / 2 + bearings.BRG_RIB_BORE / 2 + 4.0
    block = prim.union([prim.cyl(rb, Z_BLOCK, seg=128), prim.box(-rb, rb, Y_DECK, 0.0, 0.0, Z_BLOCK)])
    # guide eye post (v7 style: 10 x 10 column, 6 mm thick top with the eye along X)
    gx, gy, gz = GUIDE
    post = prim.union([prim.box(gx - 5, gx + 5, Y_DECK, gy - 8, gz - 5, gz + 5),
                       prim.box(gx - 3, gx + 3, gy - 8.01, gy + 6, gz - 5, gz + 5)])
    # rocker pivot ears (M3 pin along X)
    px, py, pz = ROCKER_PIVOT
    ears = [prim.box(px + s * 5.4, px + s * 9.4, py - 6, py + 6, 0.0, 5.7) for s in (-1, 1)]
    # arm and pad on the deck carrying the exit eye post (the tow load stays inside the reel frame)
    ex, _, ez = EXIT
    arm = prim.union([prim.box(8.0, 36.0, Y_DECK, Y_DECK + FOOT_T, Z_FOOT[1] - 1.0, ez - 9.0),
                      prim.box(ex - 17, ex + 17, Y_DECK, Y_DECK + FOOT_T, ez - 12, ez + 12),
                      prim.box(8.0, ex - 16, Y_DECK, Y_DECK + FOOT_T, ez - 12, ez - 2)])
    body = prim.union([foot, wall, block, post, arm] + ears)
    cut = []
    # sprag seat (outer key rib on +Y), mouth at the block front face
    cut.append(prim.place(bearings.seat_csk20(key_deg=90.0), tf.T(0, 0, Z_BLOCK)))
    # capstan tube bushing through the wall (printed horizontal: extra for the sagging roof)
    cut.append(prim.cyl((C_D + fits.SHAFT_IN_BORE + fits.HOLE_HORIZ_EXTRA) / 2, WALL_T + 10, z=-WALL_T - 1))
    # reel shaft bushing
    cut.append(prim.cyl((S_D + fits.SHAFT_IN_BORE + fits.HOLE_HORIZ_EXTRA) / 2, WALL_T + 2, x=X_REEL, z=-WALL_T - 1))
    # idler pin socket (blind, from the rear face)
    ix, iy = idler_xy()
    cut.append(prim.cyl((8.0 + fits.HOLE_PRESS + fits.HOLE_HORIZ_EXTRA) / 2, 10.0, x=ix, y=iy, z=-WALL_T - 1))
    # pushrod bore
    cut.append(prim.cyl((PUSH_D + fits.HOLE_SLIDE + fits.HOLE_HORIZ_EXTRA) / 2, WALL_T + 2, x=X_REEL, y=PUSH_Y,
                        z=-WALL_T - 1))
    # counterbore for the pushrod collar (its rear face is the LAUNCH back-stop)
    cut.append(prim.cyl(PUSH_COLLAR_R + 0.3, PUSH_CB + 1.0, x=X_REEL, y=PUSH_Y, z=-PUSH_CB))
    # guide eye along X
    cut.append(_eye_cut(GUIDE, guide_axis(), 6.0))
    # rocker pivot hole (M3 clearance along X through both ears)
    cut.append(prim.cylX(fits.M3_CLEAR_D / 2, px - 12, px + 12, y=py, z=pz))
    # mount holes: M3 clearance down through the foot, counterbored from the top
    for x in (ex - 11, ex + 11):        # the exit post's two M3 go through the pad into the deck
        cut.append(prim.cylY(fits.M3_CLEAR_D / 2, Y_DECK - 1, Y_DECK + FOOT_T + 1, x=x, z=ez))
    for (x, z) in MOUNT_HOLES:
        cut.append(prim.cylY(fits.M3_CLEAR_D / 2, Y_DECK - 1, Y_DECK + FOOT_T + 1, x=x, z=z))
        cut.append(prim.cylY(fits.M3_HEAD_D / 2, Y_DECK + FOOT_T - fits.M3_HEAD_H - 0.2, Y_DECK + FOOT_T + 30,
                             x=x, z=z))
    return body - prim.union(cut)


MOUNT_HOLES = [(-116.0, 10.0), (-116.0, 36.0), (-62.0, 36.0), (-34.0, 10.0), (-34.0, 36.0), (24.0, 36.0)]


def _c_tube():
    """Capstan tube C (PPA-CF), local frame = R (axis Z). Rear D flat for the capstan gear, clip groove behind it,
    round through the wall bushing, key rib from the sprag rear face to the front, collar between the sprag inner
    race and the drum, 10.3 bore for K."""
    z0, z1 = C_Z[0], c_front()
    body = shafts.keyed_shaft(z1 - z0, d=C_D, key_w=shafts.KEY20_W, key_h=shafts.KEY20_H,
                              key_z=(Z_BLOCK - bearings.CSK20_W - z0, z1 - z0 - 0.8), chamfer=0.8)
    body = body.translate((0, 0, z0))
    # collar: bears on the sprag inner race (rotates with it), 54 deg cone underneath so it prints standing
    collar = prim.cyl(12.6, Z_DRUM - Z_BLOCK, z=Z_BLOCK, r2=14.0)
    body = prim.union([body, collar])
    g = clips.groove_dims(C_D)
    flat = prim.box(C_D / 2 - 2.0, C_D, -C_D, C_D, GC_Z[0] - 0.5, GEAR_Z[1] + 0.6)
    groove = clips.clip_groove_cutter(C_D, z=GC_Z[0] - 0.3 - g["width"])
    bore = prim.cyl(C_BORE / 2, z1 - z0 + 2, z=z0 - 1)
    return body - prim.union([flat, groove, bore])


def _k_shaft():
    """Cup shaft K (PPA-CF): 10 mm, D flat (on +X) under the cup's rear half and under the cup gear, clip groove
    behind the cup gear, v7 front end (4.5 neck + chamfered 5.6 head) for Harry's C-clip in front of the cup."""
    z0, zf = K_Z[0], k_front()
    body = prim.cyl(K_D / 2, zf - z0, z=z0)
    body = prim.union([body, prim.cyl(2.25, 2.6 + 0.01, z=zf - 0.01), prim.cyl(2.8, 1.5, z=zf + 2.6, r2=2.3)])
    flat_cup = prim.box(K_D / 2 - 1.2, K_D, -K_D, K_D, Z_CUP - 0.5, Z_CUP + v7.CUP_D / 2)
    flat_g = prim.box(K_D / 2 - 1.2, K_D, -K_D, K_D, z0 - 1, CUPGEAR_Z[1] + 0.5)
    g = clips.groove_dims(K_D)
    groove = clips.clip_groove_cutter(K_D, z=CUPGEAR_Z[0] - 0.3 - g["width"])
    return body - prim.union([flat_cup, flat_g, groove])


def _s_shaft():
    """Reel shaft S (PPA-CF): 8 mm, D flat (+X) under the reel gear, clip groove behind, head = clutch disk D
    (r 20 x 3) at the front, grooved face toward the spool."""
    body = prim.cyl(S_D / 2, Z_D - S_Z0 + 0.01, z=S_Z0)
    head = prim.cyl(D_R, D_T, z=Z_D)
    body = prim.union([body, head])
    flat = prim.box(S_D / 2 - 1.0, S_D, -S_D, S_D, S_Z0 - 1, GEAR_Z[1] + 0.5)
    g = clips.groove_dims(S_D)
    groove = clips.clip_groove_cutter(S_D, z=GEAR_Z[0] - 0.3 - g["width"])
    return body - prim.union([flat, groove])


def _gear(m, z, face, bore_d, flat, hub=None, shift=0.0):
    g = gears.spur_gear(m, z, face, hub=hub, shift=shift)
    rb = (bore_d + fits.SHAFT_IN_BORE) / 2
    cut = prim.cyl(rb, 100, z=-10) - prim.box(rb - flat, rb + 5, -rb - 5, rb + 5, -20, 120)
    return g - cut


def _spool(d):
    """Storage reel (PLA), local frame: axis Z, rear face z = 0. Rear flange is a 56 deg cone (prints with no
    support, front face down), barrel r 20, front flange flat, clutch friction annulus raised on the front face,
    608 seats at both faces, two d 2 tie holes into the barrel for the line end."""
    L = SPOOL_LEN
    rc, rf = d["core_r"], d["flange_r"]
    prof = [(4.6, 0), (rf - 0.6, 0), (rf, 0.6), (rf, 2.0), (rf - 3.0, 2.0), (rc, 8.0), (rc, L - 2.0), (rf, L - 2.0),
            (rf, L - 0.6), (rf - 0.6, L), (ANNULUS[1], L), (ANNULUS[1], L + ANNULUS[2]),
            (ANNULUS[0], L + ANNULUS[2]), (ANNULUS[0], L), (4.6, L)]
    body = prim.revolve_pts(prof, seg=180)
    seat_r = bearings.B608_OD / 2 + fits.BRG_RIB_BORE / 2
    s_rear = prim.place(bearings.seat_608(bottom="lip", lip_d=14.0, below=0.0), tf.T(0, 0, 0) @ tf.Rx(180))
    s_front = prim.place(bearings.seat_608(bottom="lip", lip_d=14.0, below=0.0), tf.T(0, 0, L))
    holes = [prim.cylY(1.0, 15.0, 25.0, x=x, z=11.5) for x in (-3.0, 3.0)]
    return body - prim.union([s_rear, s_front] + holes), seat_r


def _thrust_ring():
    """Thrust ring T (PLA), local frame: rear face z = 0. Big rear part takes the rocker bumps, small front boss
    presses only the rear 608 inner race. Slides on S."""
    a = prim.tube(11.0, (S_D + fits.HOLE_CLEAR) / 2, Z_SPOOL - Z_T - 0.65)
    b = prim.tube(6.0, (S_D + fits.HOLE_CLEAR) / 2, 0.6, z=Z_SPOOL - Z_T - 0.66)
    return prim.union([a, b])


def _rocker():
    """Brake rocker (PETG, flexible lever), local frame = R. Pivots on an M3 pin at the top, two bumps press T at
    Y 0, the drum's pushrod pushes the bottom. With T blocked by the clutch stack the plate bends: it IS the
    clutch spring (reel_calc.rocker)."""
    x, y, z = ROCKER_PIVOT
    z0, z1 = ROCKER_Z
    w = 5.0
    plate = prim.box(x - w, x + w, PUSH_Y - 4.5, y, z0, z1)
    ring = prim.cyl(11.0, z1 - z0, x=x, z=z0)
    hole = prim.cyl(6.0, 10, x=x, z=z0 - 2)
    bumps = [prim.box(x + s * 8.5 - 1.2, x + s * 8.5 + 1.2, -2.0, 2.0, z1 - 0.01, Z_T) for s in (-1, 1)]
    boss = prim.cylX(2.6, x - w, x + w, y=y, z=z) ^ prim.box(x - w, x + w, y - 3, y + 3, z0, z + 3)
    pin = prim.cylX(fits.M3_CLEAR_D / 2, x - 10, x + 10, y=y, z=z)
    return prim.union([plate, ring, boss] + bumps) - hole - pin


def _pushrod():
    """Brake pushrod (PPA-CF), local frame = R at the LAUNCH position. Rear end face = the drum's contact."""
    z_front = ROCKER_Z[0] - 0.1
    rod = prim.cyl(PUSH_D / 2, z_front + 24.0, x=X_REEL, y=PUSH_Y, z=-24.0)
    collar = prim.cyl(PUSH_COLLAR_R, PUSH_COLLAR_T, x=X_REEL, y=PUSH_Y, z=-PUSH_CB + 0.05)
    return prim.union([rod, collar])


def _exit_post():
    """Exit eye post (PPA-CF): own foot with 2 x M3, v7 column 10 x 10, 6 mm top with the eye along X at the line
    height, 45 deg gussets fore and aft."""
    ex, ey, ez = EXIT
    y0 = Y_DECK + FOOT_T                 # sits on the wall's pad
    foot = prim.box(ex - 15, ex + 15, y0, y0 + FOOT_T, ez - 10, ez + 10)
    col = prim.box(ex - 5, ex + 5, y0, ey - 8, ez - 5, ez + 5)
    top = prim.box(ex - 3, ex + 3, ey - 8.01, ey + 6, ez - 5, ez + 5)
    gus = []
    for s in (-1, 1):
        pts = [(ex + s * 5, y0), (ex + s * 13, y0), (ex + s * 5, y0 + 20)]
        cs = prim.poly_cs(pts)
        gus.append(prim.extrude(cs, 10.0, z=ez - 5))
    body = prim.union([foot, col, top] + gus)
    cut = [_eye_cut(EXIT, exit_axis(), 6.0)]
    for x in (ex - 11, ex + 11):
        cut.append(prim.cylY(fits.M3_CLEAR_D / 2, y0 - 1, y0 + FOOT_T + 1, x=x, z=ez))
    return body - prim.union(cut)


def _cap_clip():
    """Harry's v7 C-clip: dia 11 x 2, 4.6 bore, 3.8 opening (toward -Y), for the 4.5 neck on K."""
    c = prim.tube(5.5, 2.3, 2.0) - prim.box(-1.9, 1.9, -7, 0, -1, 3)
    return c


# ------------------------------------------------------------------------------------------------ contract
def _bed(solid, R=None):
    """print_tf: rotate by R (4x4, default none) and move the result onto the bed corner (min x, y, z = 0)."""
    R = np.eye(4) if R is None else R
    (x0, y0, z0), _ = prim.bbox(prim.place(solid, R))
    return tf.T(-x0, -y0, -z0) @ R


def _clip_z(d):
    """z of a clip's lower face in its groove behind a gear on GEAR_Z / CUPGEAR_Z (shaft diameter d)."""
    return -0.3 - clips.groove_dims(d)["width"] + clips.CLIP_AX_CLEAR / 2


def parts(P):
    d = _d(P)
    ix, iy = idler_xy()
    out = []

    def add(name, solid, mat, R=None, **k):
        pt = _bed(solid, R) if mat not in ("bought", "steel") else None
        out.append(Part(name, solid, mat, print_tf=pt, **k))

    add("reel_wall", _wall(), "PLA", tf.Rx(90), joint="reel_base", color=COL["wall"], step=1,
        notes="Reel wall: foot, sprag housing, bushings, guide eye, rocker ears. Print on its foot")
    add("reel_sprag", bearings.bearing_csk20(okey_deg=90.0, ikey_deg=0.0), "bought", joint="reel_base",
        place=tf.T(0, 0, Z_BLOCK), color=COL["brg"], bought_ref="CSK20PP one-way bearing 20x47x14", step=2,
        notes="Sprag #2: capstan backstop, lets the capstan turn only in the wind-in sense")
    # capstan group
    add("reel_c", _c_tube(), "PPA-CF", joint="reel_capstan", color=COL["ppa"], step=3,
        notes="Capstan tube: key rib in the sprag and drum, D flat for the capstan gear. Print standing")
    add("reel_gt", _gear(M_TAKE, Z_TAKE, GEAR_Z[1] - GEAR_Z[0], C_D, 2.0, hub=(30.0, 0.9)), "PPA-CF",
        joint="reel_capstan", place=tf.T(0, 0, GEAR_Z[0]), color=COL["gear"], step=4,
        notes=f"Take-up gear m{M_TAKE:g} z{Z_TAKE}: drives the storage reel clutch through the idler")
    add("reel_gc", _gear(M_CAP, Z_CAPG, GC_Z[1] - GC_Z[0], C_D, 2.0, shift=X_CAPG, hub=(28.0, GEAR_Z[0] - GC_Z[1] - 0.2)),
        "PPA-CF", joint="reel_capstan", place=tf.T(0, 0, GC_Z[0]), color=COL["gear"], step=4,
        notes=f"Capstan gear m{M_CAP:g} z{Z_CAPG} x+{X_CAPG}: driven by the drum's REEL output gear")
    add("reel_c_clip", clips.c_clip(C_D), "PLA", joint="reel_capstan",
        place=tf.T(0, 0, GC_Z[0] + _clip_z(C_D)), color=COL["clip"], step=4,
        notes="C-clip holding both gears on the capstan tube")
    add("reel_drum", v7.drum(), "PLA", joint="reel_capstan", place=tf.T(0, 0, Z_DRUM), color=COL["drum"], step=5,
        notes="Capstan drum dia 40 x 40 (Harry v7), keyed on the tube. Print flange down")
    # cup group
    add("reel_k", _k_shaft(), "PPA-CF", joint="reel_cup", color=COL["ppa"], step=6,
        notes="Cup shaft: runs inside the capstan tube, cup in front, cup gear behind. Print standing")
    add("reel_cup", v7.cup(), "PLA", joint="reel_cup", place=tf.T(0, 0, Z_CUP), color=COL["cup"], step=6,
        notes="Wrap cup (Harry v7): ring and groove lay the wraps on the capstan. Print open end down")
    add("reel_cup_clip", _cap_clip(), "PLA", joint="reel_cup", place=tf.T(0, 0, k_front() + 0.3),
        color=COL["clip"], step=6, notes="Harry's v7 C-clip on the cup shaft neck in front of the cup")
    add("reel_gk", _gear(M_CUPG, Z_CUPG, CUPGEAR_Z[1] - CUPGEAR_Z[0], K_D, 1.2,
                         hub=(16.0, C_Z[0] - CUPGEAR_Z[1] - 0.3)), "PPA-CF",
        joint="reel_cup", place=tf.T(0, 0, CUPGEAR_Z[0]), color=COL["gear"], step=7,
        notes=f"Cup gear m{M_CUPG:g} z{Z_CUPG}: driven by the drum's wrap pinion")
    add("reel_k_clip", clips.c_clip(K_D), "PLA", joint="reel_cup",
        place=tf.T(0, 0, CUPGEAR_Z[0] + _clip_z(K_D)), color=COL["clip"], step=7,
        notes="C-clip holding the cup gear on the cup shaft")
    # idler
    gz = GEAR_Z[0] + _clip_z(8.0) - clips.CLIP_AX_CLEAR / 2
    pin_z0 = gz - clips.groove_dims(8.0)["land"] - 0.6
    pin = prim.cyl(4.0, -WALL_T + 10.0 - pin_z0, z=pin_z0) - clips.clip_groove_cutter(8.0, z=gz)
    add("reel_idler_pin", pin, "PPA-CF", joint="reel_base", place=tf.T(ix, iy, 0), color=COL["ppa"], step=8,
        notes="Idler pin, pressed into the wall from behind. Print standing")
    add("reel_gi", gears.spur_gear(M_TAKE, Z_IDL, GEAR_Z[1] - GEAR_Z[0], bore=8.0 + fits.SHAFT_IN_BORE,
                                   hub=(16.0, 0.9)), "PPA-CF",
        joint="reel_idler", place=tf.T(0, 0, GEAR_Z[0]), color=COL["gear"], step=8,
        notes=f"Idler m{M_TAKE:g} z{Z_IDL}: capstan gear to reel gear, keeps the take-up sense")
    add("reel_gi_clip", clips.c_clip(8.0), "PLA", joint="reel_base",
        place=tf.T(ix, iy, GEAR_Z[0] + _clip_z(8.0)), color=COL["clip"], step=8,
        notes="C-clip holding the idler on its pin")
    # reel shaft, gear, spool, clutch, brake
    add("reel_s", _s_shaft(), "PPA-CF", tf.Rx(180), joint="reel_shaft", color=COL["ppa"], step=9,
        notes="Reel shaft: 608 journals, head = clutch disk, D flat for the reel gear. Print head down")
    add("reel_gr", _gear(M_TAKE, Z_REELG, GEAR_Z[1] - GEAR_Z[0], S_D, 1.0, hub=(14.0, 0.9)), "PPA-CF",
        joint="reel_shaft", place=tf.T(0, 0, GEAR_Z[0]), color=COL["gear"], step=10,
        notes=f"Reel gear m{M_TAKE:g} z{Z_REELG}: turns the clutch disk 1.6 x capstan speed")
    add("reel_s_clip", clips.c_clip(S_D), "PLA", joint="reel_shaft",
        place=tf.T(0, 0, GEAR_Z[0] + _clip_z(S_D)), color=COL["clip"], step=10, notes="C-clip holding the reel gear")
    sp, _ = _spool(d)
    add("reel_spool", sp, "PLA", tf.Rx(180), joint="reel_spool", place=tf.T(0, 0, Z_SPOOL), color=COL["spool"],
        infill=0.25, step=9, notes="Storage reel: 20 m of 1 mm Dyneema on a dia 40 core, two 608s, clutch face "
                                   "in front. Print front face down")
    for i, zb in ((1, Z_SPOOL + bearings.B608_W), (2, Z_SPOOL + SPOOL_LEN)):
        add(f"reel_brg_{i}", bearings.bearing_608(), "bought", joint="reel_spool", place=tf.T(0, 0, zb),
            color=COL["brg"], bought_ref="608ZZ 8x22x7", step=9, notes="608 bearing in the storage reel")
    add("reel_t", _thrust_ring(), "PLA", joint="reel_t", place=tf.T(0, 0, Z_T), color=COL["clip"], step=9,
        notes="Thrust ring: the rocker pushes it onto the rear 608 inner race")
    add("reel_rocker", _rocker(), "PETG", joint="reel_base", color=COL["rocker"], step=11,
        notes="Brake rocker: flexible lever = clutch spring, pushed by the drum. Print flat")
    add("reel_rocker_pin", fasteners_m3(24.0), "bought", joint="reel_base",
        place=tf.T(ROCKER_PIVOT[0] - 12.0, ROCKER_PIVOT[1], ROCKER_PIVOT[2]) @ tf.Ry(90), color=COL["brg"],
        bought_ref="M3x25 SHCS + M3 nut", step=11, notes="Rocker pivot pin")
    add("reel_pushrod", _pushrod(), "PPA-CF", joint="reel_push", color=COL["ppa"], step=11,
        notes="Brake pushrod: the drum pushes it 4 mm for REEL. Print standing")
    add("reel_exit_post", _exit_post(), "PPA-CF", tf.Rx(90), joint="reel_base", color=COL["post"], step=12,
        notes="Exit eye post: carries the full tow load, so PPA-CF. Print on its foot")
    return out


def fasteners_m3(L):
    """M3 SHCS model along +Z: head z = -3 .. 0, shank 0 .. L."""
    return prim.union([prim.cyl(2.75, 3.0, z=-3.0), prim.cyl(1.5, L)])


def _placement(P):
    lay = getattr(P, "LAYOUT", None)
    if isinstance(lay, dict) and isinstance(lay.get("reel"), dict):
        r = lay["reel"]
        return tuple(r.get("origin", (0, 0, 0))), r.get("rot_z_deg", 0.0)
    return (-40.0, 140.0, H_AXIS), 0.0


def joints(P):
    org, rz = _placement(P)
    base = tf.Rz(rz) @ tf.Rx(90)
    ix, iy = idler_xy()
    return [
        Joint("reel_base", None, "fixed", org, (0, 0, 1), base=base),
        Joint("reel_capstan", "reel_base", "rev", (0, 0, 0), (0, 0, 1)),
        Joint("reel_cup", "reel_base", "rev", (0, 0, 0), (0, 0, 1)),
        Joint("reel_idler", "reel_base", "rev", (ix, iy, 0), (0, 0, 1)),
        Joint("reel_shaft", "reel_base", "rev", (X_REEL, 0, 0), (0, 0, 1)),
        Joint("reel_spool", "reel_base", "rev", (X_REEL, 0, 0), (0, 0, 1)),
        Joint("reel_t", "reel_base", "slide", (X_REEL, 0, 0), (0, 0, 1), limits=(0.0, 0.3)),
        Joint("reel_push", "reel_base", "slide", (0, 0, 0), (0, 0, 1), limits=(0.0, BRAKE_STROKE)),
    ]


def mesh_offset(alpha_deg, z1, z2):
    """Phase offset (deg) of gear 2 for gear 1 at angle 0 (tooth 0 on +X), gear 2 centre at direction alpha_deg from
    gear 1: theta2 = -(z1 / z2) theta1 + offset puts a gap of gear 2 on each tooth of gear 1 at the contact line."""
    return alpha_deg + 180.0 + 180.0 / z2 + (z1 / z2) * alpha_deg


def gear_angles(cap_deg):
    """Capstan, idler and reel gear angles (deg) for a capstan angle (gear tooth 0 on +X at angle 0)."""
    ix, iy = idler_xy()
    a1 = math.degrees(math.atan2(iy, ix))
    th_i = -(Z_TAKE / Z_IDL) * cap_deg + mesh_offset(a1, Z_TAKE, Z_IDL)
    a2 = math.degrees(math.atan2(-iy, X_REEL - ix))
    # the idler plays gear 1 for the second mesh: its tooth 0 sits at th_i, so shift by th_i
    th_r = -(Z_IDL / Z_REELG) * th_i + mesh_offset(a2, Z_IDL, Z_REELG)
    return cap_deg, th_i, th_r


def brake_mm(S):
    if "reel_brake_mm" in S:
        return float(S["reel_brake_mm"])
    mode = str(S.get("mode", "LAUNCH")).upper()
    return BRAKE_STROKE if mode in ("REEL", "WRAPPED", "WRAP", "TOW") else 0.0


def kin(P, S):
    d = _d(P)
    cap = float(S.get("capstan_deg", 0.0))
    _, th_i, th_r = gear_angles(cap)
    wraps = float(S.get("wraps", 0.0))
    if "spool_deg" in S:
        spool = float(S["spool_deg"])
    else:
        r_mid = 0.5 * (d["core_r"] + reel_calc.pack_r(d, d["line_len_m"]))
        spool = -math.degrees(float(S.get("line_out_m", 0.0)) * 1000.0 / r_mid)
    return {"reel_capstan": cap, "reel_cup": -360.0 * wraps, "reel_idler": th_i, "reel_shaft": th_r,
            "reel_spool": spool, "reel_t": 0.0, "reel_push": brake_mm(S)}


def allowed(P):
    return [
        Allow("reel_sprag", "reel_wall", "CSK20PP pressed in its crush-rib seat, outer key rib"),
        Allow("reel_sprag", "reel_c", "C rib in the sprag inner keyway, inner race on C (model is one solid)"),
        Allow("reel_c", "reel_gc", "D-flat fit"),
        Allow("reel_c", "reel_gt", "D-flat fit"),
        Allow("reel_c", "reel_c_clip", "clip in the groove"),
        Allow("reel_c", "reel_drum", "keyed fit"),
        Allow("reel_k", "reel_cup", "D fit"),
        Allow("reel_k", "reel_cup_clip", "clip on the neck"),
        Allow("reel_k", "reel_gk", "D fit"),
        Allow("reel_k", "reel_k_clip", "clip in the groove"),
        Allow("reel_idler_pin", "reel_wall", "press fit"),
        Allow("reel_idler_pin", "reel_gi", "idler turns on the pin"),
        Allow("reel_idler_pin", "reel_gi_clip", "clip in the groove"),
        Allow("reel_s", "reel_gr", "D-flat fit"),
        Allow("reel_s", "reel_s_clip", "clip in the groove"),
        Allow("reel_brg_*", "reel_spool", "608 pressed in the reel"),
        Allow("reel_brg_*", "reel_s", "journal in the inner race"),
        Allow("reel_rocker_pin", "reel_wall", "M3 pin through the ears"),
        Allow("reel_rocker_pin", "reel_rocker", "M3 pin through the rocker"),
        Allow("reel_pushrod", "reel_rocker", "the pushrod bends the flexible rocker 4 mm at REEL (elastic)",
              max_vol=120.0),
    ]


def clearances(P):
    return [
        Clearance("reel_cup", "reel_drum", 0.25, "cup skirt over the drum (v7 0.6 radial)"),
        Clearance("reel_drum", "reel_wall", 0.5, "drum flange in front of the block"),
        Clearance("reel_gt", "reel_gi", 0.0, "mesh"),
        Clearance("reel_gi", "reel_gr", 0.0, "mesh"),
        Clearance("reel_spool", "reel_wall", 0.3, "spool spins in front of the wall"),
        Clearance("reel_spool", "reel_rocker", 0.3, "rocker behind the spool"),
    ]


def grounded(P):
    return ["reel_wall"]


def audit_states(P):
    out = []
    for k in range(6):                          # one capstan tooth pitch, gear mesh sweep
        out.append(dict(phase="reel_mesh", t=0.0, mode="REEL", capstan_deg=k * 360.0 / Z_TAKE / 5))
    for w in (0.0, 0.25, 0.5, 0.75, 1.0, 3.5, 7.0):
        out.append(dict(phase="reel_wrap", t=0.0, mode="REEL", wraps=w))
    out.append(dict(phase="reel_launch", t=0.0, mode="LAUNCH", line_out_m=3.0))
    return out


# ------------------------------------------------------------------------------------------------ line path
LINE_R_CAP = v7.DR + 0.5          # line centre radius on the capstan


def _tangent_angle(p, r, upper=True):
    """Angle (deg) of the tangent point on a circle (centre origin, radius r) seen from point p (XY)."""
    ang = math.degrees(math.atan2(p[1], p[0]))
    a = math.degrees(math.acos(r / math.hypot(p[0], p[1])))
    return ang - a if upper else ang + a


def _rz(p, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array((c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]))


def line_path(P, wraps=0.0, pack_r=None, z_spool=None):
    """Line centre polyline in the R frame as a list of (points (N x 3), allowed part names) spans:
    storage reel -> guide eye -> (capstan turns -> groove) -> ring -> exit eye -> 20 mm toward the dart.
    wraps = cup turns from home (cup angle -360 wraps). Spans with allowed parts may touch those parts only."""
    d = _d(P)
    rp = reel_calc.pack_r(d, 3.0) if pack_r is None else pack_r
    zs = Z_GUIDE if z_spool is None else z_spool
    g = GUIDE
    # storage reel: line leaves the pack top toward the guide (take-up sense +Z)
    gl = np.array((g[0] - X_REEL, g[1]))
    th = _tangent_angle(gl, rp, upper=False)
    sp = np.array((X_REEL + rp * math.cos(math.radians(th)), rp * math.sin(math.radians(th)), zs))
    spans = [(np.array([sp, g]), {"reel_spool", "reel_wall"})]
    psi = -360.0 * wraps
    cp = v7.cup_line_points()
    off = np.array((0, 0, Z_CUP))
    ring_r = _rz(cp["ring_rear"], psi) + off
    ring_f = _rz(cp["ring_front"], psi) + off
    if wraps <= 1e-9:
        spans.append((np.array([g, ring_r]), {"reel_wall", "reel_cup"}))
    else:
        th0 = _tangent_angle(g[:2], LINE_R_CAP, upper=True)
        th_end = v7.TH_RIM + psi
        A = th0 - th_end
        if A <= 0:
            raise ValueError("wraps too small for the turn model (transition inside the first 70 deg)")
        n = max(8, int(A / 10))
        z0, z1 = Z_DRUM + v7.FLANGE_H + 0.8, Z_CUP - 0.6
        turns = []
        for k in range(n + 1):
            t = math.radians(th0 - A * k / n)
            turns.append((LINE_R_CAP * math.cos(t), LINE_R_CAP * math.sin(t), z0 + (z1 - z0) * k / n))
        turns = np.array(turns)
        spans.append((np.array([g, turns[0]]), {"reel_wall", "reel_drum"}))
        spans.append((turns, {"reel_drum"}))
        grv = []
        for k in range(9):
            tdeg = v7.TH_RIM - (v7.TH_RIM - v7.TH_TOP) * k / 8 + psi
            z = Z_CUP + v7.CUP_D * k / 8
            grv.append((v7.ER * math.cos(math.radians(tdeg)), v7.ER * math.sin(math.radians(tdeg)), z))
        grv = np.array(grv)
        spans.append((np.vstack([turns[-1:], grv, [ring_r]]), {"reel_drum", "reel_cup"}))
    spans.append((np.array([ring_r, ring_f]), {"reel_cup"}))
    spans.append((np.array([ring_f, EXIT]), {"reel_cup", "reel_exit_post"}))
    spans.append((np.array([EXIT, EXIT + (20.0, 0, 0)]), {"reel_exit_post"}))
    return spans


def _trim(pts, a, b):
    """Polyline with a mm cut off the start and b mm off the end (straight two-point spans only)."""
    p0, p1 = pts[0], pts[-1]
    v = p1 - p0
    L = np.linalg.norm(v)
    u = v / L
    return np.array([p0 + u * a, p1 - u * b])


def line_check(P, wraps_list=None, local=None):
    """Exact check of the line tube (d 1.0) against every reel part. Free spans must clear everything except the
    contact ends (2.5 mm at a ring or eye); contact spans may touch only their listed parts.
    local: {part name: Manifold in the R frame} for the posed parts (built here when None).
    Returns list of failure strings."""
    if wraps_list is None:
        wraps_list = [0.0] + [1.0 + k / 24.0 for k in range(24)] + [8.0]
    fails = []
    for w in wraps_list:
        S = dict(wraps=w, mode="REEL")
        sol = local(S) if callable(local) else _local_posed(P, S)
        for pts, allow in line_path(P, w):
            if len(pts) == 2 and np.linalg.norm(pts[-1] - EXIT) < 1e-6:
                # dart line: rides round the outside of the ring for half of each turn (v7 README) and enters the
                # exit eye at an angle; it must clear the cup face and everything else
                q = _trim(pts, 0.0, 0.0)
                psi = -360.0 * w
                M = tf.T(0, 0, Z_CUP) @ tf.Rz(psi)
                rz = prim.place(prim.cylX(v7.RING_R + 0.3, -v7.RING_L - 0.5, v7.RING_L + 0.5, y=v7.RING_AT, z=v7.EZP)
                                + prim.box(-2.5, 2.5, 21, 28, 5, 15), M)
                ez = prim.cyl(4.5, 12, z=-6).rotate((0, 90, 0)).translate(tuple(EXIT))
                sol2 = dict(sol)
                sol2["reel_cup"] = sol["reel_cup"] - rz
                sol2["reel_exit_post"] = sol["reel_exit_post"] - ez
                for name, v in line.line_hits(q, sol2).items():
                    if v > 1e-3:
                        fails.append(f"line at {w:.3f} wraps: dart span hits {name} ({v:.3f} mm3)")
            elif len(pts) == 2:
                ends = (4.0 if np.linalg.norm(pts[0] - GUIDE) < 1e-6 or np.linalg.norm(pts[0] - EXIT) < 1e-6 else 2.5,
                        4.0 if np.linalg.norm(pts[-1] - GUIDE) < 1e-6 or np.linalg.norm(pts[-1] - EXIT) < 1e-6 else 2.5)
                q = _trim(pts, *ends)
                hits = line.line_hits(q, sol)
                for name, v in hits.items():
                    if v > 1e-3:
                        fails.append(f"line at {w:.3f} wraps: free span {np.round(pts[0], 1)} -> "
                                     f"{np.round(pts[-1], 1)} hits {name} ({v:.3f} mm3)")
            else:
                hits = line.line_hits(pts, {k: s for k, s in sol.items() if k not in allow})
                for name, v in hits.items():
                    if v > 1e-3:
                        fails.append(f"line at {w:.3f} wraps: contact span hits {name} ({v:.3f} mm3)")
    return fails


_PARTS_CACHE = {}


def _local_posed(P, S):
    """Every reel part posed in the R frame for state S (no world placement)."""
    vals = kin(P, S)
    js = {j.name: j for j in joints(P)}
    out = {}
    key = id(P)
    if key not in _PARTS_CACHE:
        _PARTS_CACHE.clear()
        _PARTS_CACHE[key] = parts(P)
    for p in _PARTS_CACHE[key]:
        M = np.eye(4)
        j = p.joint
        chain = []
        while j and j != "reel_base":
            chain.append(js[j]); j = js[j].parent
        for jj in reversed(chain):
            M = M @ jj.local(vals.get(jj.name, 0.0))
        out[p.name] = prim.place(p.solid, M @ p.place)
    return out


def checks(P, asm=None):
    return line_check(P)
