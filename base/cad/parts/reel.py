"""GrappStar base, reel system (worker cad-reel), D49 RAIL REEL: storage spool, capstan with Harry's v7 drum and cup,
spool clutch and the two drives across the pivot (the rail bridge). Module contract: base/cad/README.md.

Everything rides cad-launch's rail (joint launch_rail) and tilts with it. Rail frame (launch.py): u along the rail
(muzzle +), yl = world y - 140 (rail centre plane), w normal to the rail (up at pitch 0), origin on the main axis.
Shared numbers: params RAIL_REEL (positions in u, WORLD y, w), REEL, DRUM. Physics: reel_calc.py.

Line (D49): one straight segment from the spool pack top (u -224, y 140, w 84.6) through the AXIAL eyelet of the cup
(u -187 .. -183, 26 below the capstan axis) to the dart tail, at every pitch and carriage position. With wraps on, the
line goes from the pack onto the capstan, round it, up the cup groove into the eyelet.

Frames used for parts on round axes (local Z = the axis):
- F_U(y, w): axis along +u through (y, w): local X = -yl, Y = -w, Z = u (so local z IS the rail u).
- F_Y(u, w): axis along +y through (u, w): local X = u, Y = -w, Z = yl (z = world y - 140). At pitch 0 this is cad-drum's
  main-axis frame L (x forward, y down, z = world y), so main-axis gear angles carry over plus the pitch.
Senses: capstan wind-in + about +u; cup WRAP - about +u (v7 handedness kept: positive drum still wraps, firmware sign
unchanged); spool payout + about +y; take-up turns the spool shaft - about +y on forward wiper turns.
"""
from __future__ import annotations

import math

import numpy as np
from manifold3d import Manifold

from cad.lib import Part, Joint, Allow, Clearance, Mesh, prim, tf, fits, gears, bearings, shafts, clips, line, switch
from cad.parts import reel_calc, reel_v7 as v7

Y0 = 140.0                    # rail centre plane (world y)


def _rr(P):
    r = (getattr(P, "RAIL_REEL", None) if P is not None else None) or {}
    return r


try:
    from tools.params import load_params as _lp
    _P0 = _lp()
except Exception:                          # pragma: no cover
    _P0 = None
RR = _rr(_P0)


def _g(d, path, default):
    for k in path:
        if not isinstance(d, dict) or k not in d:
            return default
        d = d[k]
    return d


# ------------------------------------------------------------------------------------------------ layout (rail frame)
LINE_W = _g(RR, ("line", "w"), 84.6)
CAP_YW = tuple(_g(RR, ("capstan", "axis_yw"), (140.0, 110.6)))
CAP_W = CAP_YW[1]
Z_CUP = _g(RR, ("capstan", "cup_rim_u"), -201.0)          # cup rim plane (u)
Z_DRUM = _g(RR, ("capstan", "drum_back_u"), -237.0)       # v7 drum flange back face (u)
SP_U, SP_W = _g(RR, ("spool", "axis_uw"), (-224.0, 57.6))
CORE_R = _g(RR, ("spool", "core_d"), 40.0) / 2
FLANGE_R = _g(RR, ("spool", "flange_d"), 62.0) / 2
PACK_Y = tuple(_g(RR, ("spool", "pack_y"), (129.0, 149.0)))
CT = _g(RR, ("capstan_train",), {})
TT = _g(RR, ("take_up_train",), {})
C1_U, C1_W = _g(CT, ("c1", "axis_uw"), (0.0, 99.1))
UC_Y, UC_W = _g(CT, ("u_shaft_yw",), (76.5, 99.1))
B1_APEX_Y = _g(CT, ("b1", "apex_y"), 76.5)
C1P_U, C1P_W = _g(TT, ("c1p", "axis_uw"), (-72.1, -2.5))
T1_U, T1_W = _g(TT, ("t1", "axis_uw"), (-110.0, 57.6))
UT_Y, UT_W = _g(TT, ("u_shaft_yw",), (64.0, 57.6))
B2_APEX_Y = _g(TT, ("b2", "apex_y"), 64.0)
B3_APEX_U = _g(TT, ("b3", "apex_u"), -224.0)
U_FLANGE = _g(RR, ("head_flange_u",), -178.3)
WRAPS = int((getattr(_P0, "REEL", None) or {}).get("wraps", 7)) if _P0 is not None else 7

# axial planes (u) of the head, front to rear (study section 1.2, reordered so the drum lock and bracket B fit)
U_FRONT = (-187.2, -181.2)    # head front plate (bolts to cad-launch's rail flange at u -181.2)
U_P1 = (-248.0, -241.0)       # P1: capstan tube, u-shaft C rear, G_b stub, lever post, spool outboard leg
U_G = (-256.0, -249.0)        # G_a / G_b / G_c (face 7)
U_P1B = (-264.0, -258.0)      # P1b: capstan tube rear bushing, SW_ARM
U_GK = (-284.5, -277.5)       # cup gear G_k (and cad-drum's W output 49T in the same plane)
U_P2 = (-311.0, -305.0)       # P2: cup shaft and W rear journals, SW_ARM on its front face
U_LEVER = (-237.0, -229.0)    # clutch lever plane (= cad-drum's brake cam plane)
Y_SPINE = (-22.0, -16.0)      # inboard spine wall (world y)
Y_MID = (82.0, 88.0)          # mid plate (spool shaft inboard bushing)
Y_LEG = (155.0, 161.0)        # P1 outboard leg (spool shaft outboard bushing)
W_LOW = 24.0                  # head front plate bottom
DECK_A, DECK_B, DECK_LIM = 0.7314, 0.682, -160.0   # deck rule at 47.06 deg: 0.7314 u + 0.682 w >= -160

# gears
M_CAP = 1.5
Z_GA, Z_GB, Z_GC = 26, 32, 23
Z_C1 = 49
Z_MAIN, Z_BUS = 50, 50
Z_C1P, Z_IDL, Z_T1 = 46, 30, 20
Z_GK, Z_WOUT = 28, 49
M_TAKE = 1.5
SWING_OUT_DEG = 12.0
C_D, K_D, S_D = 20.0, 10.0, 8.0
UC_D, UT_D = 10.0, 8.0

COL = dict(plate="#b9c2cc", ppa="#3a3f45", gear="#d97a00", drum="#2e9c86", cup="#7f77dd", spool="#7f77dd",
           brg="#8a8f96", clip="#e0c060", lever="#c86f00", bevel="#c9a227")


def Y(y):
    """World y -> rail-local y."""
    return y - Y0


def Bw(u0, u1, y0, y1, w0, w1):
    """Box in (u, WORLD y, w)."""
    return prim.box(u0, u1, Y(y0), Y(y1), w0, w1)


def F_U(y, w):
    """Axis-along-u frame through (world y, w): local X = -yl, Y = -w, Z = u."""
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = (0, -1, 0), (0, 0, -1), (1, 0, 0)
    M[:3, 3] = (0.0, Y(y), w)
    return M


def F_Y(u, w):
    """Axis-along-y frame through (u, w): local X = u, Y = -w, Z = yl."""
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = (1, 0, 0), (0, 0, -1), (0, 1, 0)
    M[:3, 3] = (u, 0.0, w)
    return M


def zU(u):
    return u


def zY(y):
    return Y(y)


def _cd(m, z1, z2):
    return gears.center_distance(m, z1, z2)


def _circ2(p0, r0, p1, r1, side):
    d = math.dist(p0, p1)
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(0.0, r0 * r0 - a * a))
    ux, uy = (p1[0] - p0[0]) / d, (p1[1] - p0[1]) / d
    return (p0[0] + a * ux + side * h * uy, p0[1] + a * uy - side * h * ux)


def gb_yw():
    """G_b idler axis (world y, w): meshes G_a (u-shaft C) and G_c (capstan tube), on the upper side."""
    a = _circ2((UC_Y, UC_W), _cd(M_CAP, Z_GA, Z_GB), (CAP_YW[0], CAP_W), _cd(M_CAP, Z_GB, Z_GC), -1)
    b = _circ2((UC_Y, UC_W), _cd(M_CAP, Z_GA, Z_GB), (CAP_YW[0], CAP_W), _cd(M_CAP, Z_GB, Z_GC), 1)
    return a if a[1] > b[1] else b


def idler_uw(swing=0.0):
    """Swing idler axis (u, w) engaged (swing 0) or turned by swing deg about the c1' axis."""
    a = _circ2((C1P_U, C1P_W), _cd(M_TAKE, Z_C1P, Z_IDL), (T1_U, T1_W), _cd(M_TAKE, Z_IDL, Z_T1), 1)
    b = _circ2((C1P_U, C1P_W), _cd(M_TAKE, Z_C1P, Z_IDL), (T1_U, T1_W), _cd(M_TAKE, Z_IDL, Z_T1), -1)
    p = a if a[0] < b[0] else b             # the rear solution (the study's (-123.3, 22.5))
    if swing:
        r = math.dist(p, (C1P_U, C1P_W))
        t = math.atan2(p[1] - C1P_W, p[0] - C1P_U) - math.radians(swing)   # joint +swing turns u toward -w
        p = (C1P_U + r * math.cos(t), C1P_W + r * math.sin(t))
    return p


def swing_sign():
    """Sign of the arm turn (about +w... about +y in F_Y) that moves the idler away from T1."""
    p0 = idler_uw(0.0)
    pp = idler_uw(SWING_OUT_DEG)
    return 1.0 if math.dist(pp, (T1_U, T1_W)) > math.dist(p0, (T1_U, T1_W)) else -1.0   # swing = sign x SWING_OUT_DEG


# ------------------------------------------------------------------------------------------------ generic solids
def fasteners_m3(L):
    """M3 SHCS model along +Z: head z = -3 .. 0, shank 0 .. L."""
    return prim.union([prim.cyl(2.75, 3.0, z=-3.0), prim.cyl(1.5, L)])


def _gear(m, z, face, bore_d, flat=1.0, hub=None, shift=0.0, phase=0.0, z0=0.0):
    """Spur gear along +Z from z0, D bore (flat on +X before the phase turn)."""
    g = gears.spur_gear(m, z, face, hub=hub, shift=shift)
    rb = (bore_d + fits.SHAFT_IN_BORE) / 2
    cut = prim.cyl(rb, 100, z=-10)
    if flat:
        cut = cut - prim.box(rb - flat, rb + 5, -rb - 5, rb + 5, -20, 120)
    g = g - cut
    if phase:
        g = prim.place(g, tf.Rz(phase))
    return g.translate((0, 0, z0))


def bevel(m, z, z_mate, b, bore_d, hub=None, flat=1.0):
    """Straight bevel gear (approximate: the spur profile of the large end extruded toward the apex with the cone
    scale), axis +Z, large end at z = 0, teeth toward +z (the apex side). b = face width along the cone. hub = (r, h)
    boss on the -z side (away from the apex). Returns (solid, info)."""
    d = math.atan2(z, z_mate)                              # pitch cone angle
    r0 = m * z / 2
    rc = r0 / math.sin(d)                                  # cone distance
    h = b * math.cos(d)
    sc = (rc - b) / rc
    prof = gears.spur_profile(m, z, backlash=fits.GEAR_BACKLASH + 0.15)
    body = prim.extrude(prim.shapely_cs(prof), h, scale_top=sc)
    parts = [body]
    if hub is not None:
        parts.append(prim.cyl(hub[0], hub[1] + 0.01, z=-hub[1]))
    g = prim.union(parts)
    rb = (bore_d + fits.SHAFT_IN_BORE) / 2
    cut = prim.cyl(rb, 200, z=-100)
    if flat:
        cut = cut - prim.box(rb - flat, rb + 5, -rb - 5, rb + 5, -120, 120)
    return g - cut, dict(delta=math.degrees(d), apex=r0 / math.tan(d), h=h, rc=rc, r_tip=r0 + m)


def _bed(solid, R=None):
    """print_tf: rotate by R (4x4, default none) and move the result onto the bed corner (min x, y, z = 0)."""
    R = np.eye(4) if R is None else R
    (x0, y0, z0), _ = prim.bbox(prim.place(solid, R))
    return tf.T(-x0, -y0, -z0) @ R


def _clip_z(d, face0):
    """z of a clip's lower face in its groove behind a gear whose rear face is at face0 (shaft diameter d)."""
    return face0 - 0.3 - clips.groove_dims(d)["width"] + clips.CLIP_AX_CLEAR / 2


def _hole_u(r, y, w, u0, u1):
    return prim.cylX(r, u0, u1, y=Y(y), z=w)


def _hole_y(r, u, w, y0, y1):
    return prim.cylY(r, Y(y0), Y(y1), x=u, z=w)


def _hole_w(r, u, y, w0, w1):
    return prim.cyl(r, w1 - w0, x=u, y=Y(y), z=w0)


def _deck_cut(margin=0.0):
    """Half space below the 47 deg deck plane (rail frame): parts may not reach into it."""
    n = np.array((DECK_A, DECK_B))
    n = n / np.linalg.norm(n)
    big = 2000.0
    c = prim.box(-big, big, -big, big, -big, 0.0)           # z <= 0 half space, then turned: local z = n
    M = np.eye(4)
    M[:3, 2] = (n[0], 0.0, n[1])
    M[:3, 0] = (n[1], 0.0, -n[0])
    M[:3, 1] = (0.0, 1.0, 0.0)
    off = (DECK_LIM + margin) / math.hypot(DECK_A, DECK_B)
    M[:3, 3] = (n[0] * off, 0.0, n[1] * off)
    return prim.place(c, M)


# ------------------------------------------------------------------------------------------------ head plates
def _front_plate():
    """Head front plate (PLA), u -187.2 .. -181.2: a low bar under the cocked carriage group (w 24 .. 62, y -14 .. 170)
    bolted to cad-launch's rail flange (4 x M3 x 16 along u into its inserts), and an inboard wing (y -14 .. 70) up to
    w 200 joining the spine; bushing for u-shaft T, window for cad-drum's link."""
    u0, u1 = U_FRONT
    body = prim.union([Bw(u0, u1, Y_SPINE[1], 170.0, W_LOW, 62.0), Bw(u0, u1, Y_SPINE[1], 68.0, W_LOW, 212.0),
                       _hole_u(10.0, UT_Y, UT_W, u0, u1)])
    cuts = [_hole_u((UT_D + fits.SHAFT_IN_BORE) / 2, UT_Y, UT_W, u0 - 1, u1 + 1)]
    for (y, w) in FRONT_BOLTS:
        cuts.append(_hole_u(fits.M3_CLEAR_D / 2, y, w, u0 - 1, u1 + 1))
        cuts.append(_hole_u(3.0, y, w, u0 - 1, u0 + 3.0))                       # head counterbore from behind
    lw = link_window()
    cuts.append(Bw(u0 - 1, u1 + 1, lw[0], lw[1], lw[2], lw[3]))
    try:
        from cad.parts import drum as _dm
        cuts += _dm.head_front_cuts(_P0)
    except Exception:
        pass
    for (y, w) in SPINE_SCREWS_FRONT:
        cuts.append(_hole_y(fits.M3_TAP_D / 2, 0.5 * (u0 + u1), w, Y_SPINE[1] - 1.0, Y_SPINE[1] + 10.0))
    return prim.diff(body, cuts)


FRONT_BOLTS = [(115.0, 36.0), (115.0, 52.0), (165.0, 36.0), (165.0, 52.0)]     # into the rail flange inserts (world y, w)
LINK_WIN = (-11.0, 3.0, 94.0, 142.0)            # (y0, y1, w0, w1): cad-drum's link passes the front plate here
def link_window(margin=5.0):
    """(y0, y1, w0, w1) of the window cad-drum's link passes through the front plate, over its +-7 mm travel."""
    try:
        from cad.parts import drum as _dm
        g = _dm.G(_P0)
        A, B = _dm.link_ends(g)
        ys, ws = [], []
        for du in (-7.5, 7.5):
            for u in U_FRONT:
                t = (u - (A[0] + du)) / (B[0] - A[0])
                p = A + t * (B - A)
                ys.append(p[1] + Y0)
                ws.append(p[2])
        r = g["link_d"] / 2 + margin
        return (min(ys) - r, max(ys) + r, min(ws) - r, max(ws) + r)
    except Exception:
        return LINK_WIN


SPINE_SCREWS_FRONT = [(0.0, 40.0), (0.0, 70.0), (0.0, 200.0)]


def spine_outline():
    """(u, w) polygon of the spine: front plate to P2, top w 205, bottom edge on the deck rule (+6 mm)."""
    u_f, u_r = U_FRONT[1] + 1.0, U_P2[0]          # 1 mm past the front plate: the screw counterbores stay inside

    def w_min(u):
        return max(W_LOW, (DECK_LIM + 6.0 - DECK_A * u) / DECK_B)
    return [(u_f, w_min(u_f)), (u_f, 212.0), (u_r, 212.0), (u_r, w_min(u_r))]


def _spine():
    """Inboard spine wall (PLA), y -14 .. -8: front plate, P1 strip and P2 screw to it; cad-drum's brackets A and B
    and the guide rod blocks screw to its inner face."""
    from shapely.geometry import Polygon
    poly = Polygon(spine_outline())
    s = prim.extrude(prim.shapely_cs(poly), Y_SPINE[1] - Y_SPINE[0])         # in (u, w) at local z = y
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = (1, 0, 0), (0, 0, 1), (0, -1, 0)          # x = u, y = w, z = -y
    M[:3, 3] = (0.0, Y(Y_SPINE[1]), 0.0)
    s = prim.place(s, M)
    cuts = []
    for (u, w) in spine_holes():
        cuts.append(_hole_y(fits.M3_CLEAR_D / 2, u, w, Y_SPINE[0] - 1, Y_SPINE[1] + 1))
        cuts.append(_hole_y(3.0, u, w, Y_SPINE[0] - 1, Y_SPINE[0] + 3.0))
    # lightening windows clear of every screw (keeps the print time down)
    cuts.append(Bw(-235.0, -200.0, Y_SPINE[0] - 1, Y_SPINE[1] + 1, 60.0, 92.0))
    return prim.diff(s, cuts)


def spine_holes():
    """(u, w) of the screws through the spine: front plate, P1 strip, P2, and cad-drum's brackets."""
    uf = 0.5 * sum(U_FRONT)
    up1 = 0.5 * sum(U_P1)
    up2 = 0.5 * sum(U_P2)
    out = [(uf, w) for _, w in SPINE_SCREWS_FRONT] + [(up1, 96.0), (up1, 110.0), (up2, 100.0), (up2, 190.0)]
    try:
        from cad.parts import drum as _dm
        out += [tuple(p) for p in _dm.head_spine_holes(_P0)]
    except Exception:
        pass
    return out


def _p1():
    """P1 (PLA), u -248 .. -241: strip under the drum (y -8 .. 85, w 86 .. 118) joining the spine, main plate
    (y 85 .. 161, w 91 .. 140) with the capstan tube bushing, the u-shaft C rear bushing and the G_b stub, the clutch
    lever post on its front face, the outboard leg down to the spool shaft (y 155 .. 161) and two bosses back to P1b.
    Cut round the spool and shroud (y 124 .. 154, w below 91)."""
    u0, u1 = U_P1
    gy, gw = gb_yw()
    body = [Bw(u0, u1, Y_SPINE[1], 85.0, 86.0, 118.0), Bw(u0, u1, 85.0, 161.0, 91.0, 140.0),
            Bw(u0, u1, 85.0, 124.0, 80.0, 92.0),
            Bw(u0, u1, Y_LEG[0], Y_LEG[1], 40.0, 92.0), Bw(u0, SP_U + 12.0, Y_LEG[0], Y_LEG[1], 44.0, 72.0),
            Bw(u1 - 0.01, U_LEVER[1], LIGHT_STOP[0] - 5.0, LIGHT_STOP[0] + 5.0, LIGHT_BLOCK_W[0], LIGHT_BLOCK_W[1]),
            _hole_y(13.0, SP_U, SP_W, Y_LEG[0], Y_LEG[1]),
            _hole_u(CAP_BOSS_R, CAP_YW[0], CAP_W, u0, u1), _hole_u(9.0, UC_Y, UC_W, u0, u1),
            _hole_u(LEVER_POST_R, LEVER_PIVOT[0], LEVER_PIVOT[1], u1 - 0.01, LEVER_POST_U)]
    b = prim.union(body)
    cuts = [_hole_u((C_D + fits.SHAFT_IN_BORE) / 2, CAP_YW[0], CAP_W, u0 - 1, u1 + 1),
            _hole_u((UC_D + fits.SHAFT_IN_BORE) / 2, UC_Y, UC_W, u0 - 1, u1 + 1),
            _hole_u((GB_AXLE_D + fits.HOLE_PRESS) / 2, gy, gw, u0 - 1, u1 + 1),
            _hole_y((S_D + fits.SHAFT_IN_BORE) / 2, SP_U, SP_W, Y_LEG[0] - 1, Y_LEG[1] + 1),
            _hole_u(fits.M3_TAP_D / 2, LEVER_PIVOT[0], LEVER_PIVOT[1], LEVER_POST_U - 9.0, LEVER_POST_U + 1),
            _hole_w(fits.M3_TAP_D / 2, LIGHT_STOP_U, LIGHT_STOP[0], LIGHT_BLOCK_W[0] - 1, LIGHT_BLOCK_W[1] + 1)]
    for (y, w) in P1B_POSTS:                         # P1b's posts screw into P1's rear face
        cuts.append(_hole_u(fits.M3_TAP_D / 2, y, w, u0 - 1, u0 + 6.0))
    for (u, w) in [(0.5 * (u0 + u1), 96.0), (0.5 * (u0 + u1), 110.0)]:
        cuts.append(_hole_y(fits.M3_TAP_D / 2, u, w, Y_SPINE[1] - 1, Y_SPINE[1] + 10.0))
    # mid plate screws into the strip's lower edge and the main plate (M3 along u from the mid plate's rear tab)
    for w in (88.5,):
        cuts.append(_hole_u(fits.M3_TAP_D / 2, 0.5 * sum(Y_MID), w, u1 - 9.0, u1 + 1))
    # clear the spool, the shroud and the drum: no plate within r 36.5 of the spool axis, nothing round the drum
    cuts.append(_hole_y(36.5, SP_U, SP_W, 122.0, 154.5))
    cuts.append(_drum_keepout(3.0))
    return prim.diff(b, [c for c in cuts if c is not None])


CAP_BOSS_R = 14.0
GB_AXLE_D = 8.0
P1B_POSTS = [(162.0, 98.0), (162.0, 124.0)]


def _drum_keepout(margin):
    """cad-drum's rail drum and its code switch block, as a keep-out (rail frame), when drum.py is there."""
    try:
        from cad.parts import drum as _dm
        return _dm.head_keepout(_P0, margin)
    except Exception:
        return None


def _p1b():
    """P1b (PLA), u -264 .. -258: rear bushing of the capstan tube, on two posts from P1; SW_ARM base on its rear
    face."""
    u0, u1 = U_P1B
    b = prim.union([Bw(u0, u1, 122.0, 166.0, 92.0, 130.0), _hole_u(CAP_BOSS_R, CAP_YW[0], CAP_W, u0, u1)]
                   + [_hole_u(4.5, y, w, u1 - 0.01, U_P1[0]) for (y, w) in P1B_POSTS])     # posts forward to P1
    cuts = [_hole_u((C_D + fits.SHAFT_IN_BORE) / 2, CAP_YW[0], CAP_W, u0 - 1, u1 + 1)]
    for (y, w) in P1B_POSTS:
        cuts.append(_hole_u(fits.M3_CLEAR_D / 2, y, w, u0 - 1, U_P1[0] + 1))
        cuts.append(_hole_u(3.0, y, w, u0 - 1, u0 + 3.0))
    return prim.diff(b, cuts)


def _p2():
    """P2 (PLA), u -291.5 .. -285.5: cup shaft and W idler rear journals; screws to the spine."""
    u0, u1 = U_P2
    wmin = (DECK_LIM + 6.0 - DECK_A * u0) / DECK_B
    b = prim.union([Bw(u0, u1, Y_SPINE[1], 166.0, max(wmin, 86.0), 212.0)])
    cuts = [_hole_u((K_D + fits.SHAFT_IN_BORE) / 2, CAP_YW[0], CAP_W, u0 - 1, u1 + 1)]
    for (x, y) in sw_base_holes():                  # SW_ARM base screws, tapped into P2's front face
        p = sw_frame() @ np.array([x, y, 0.0, 1.0])
        cuts.append(prim.cylX(fits.M3_TAP_D / 2, u1 - 8.0, u1 + 1, y=p[1], z=p[2]))
    try:
        from cad.parts import drum as _dm
        cuts += [c for c in _dm.head_p2_cuts(_P0, u0, u1)]
    except Exception:
        pass
    for (u, w) in [(0.5 * (u0 + u1), 100.0), (0.5 * (u0 + u1), 190.0)]:
        cuts.append(_hole_y(fits.M3_TAP_D / 2, u, w, Y_SPINE[1] - 1, Y_SPINE[1] + 10.0))
    # lightening window between the journals and the top
    cuts.append(Bw(u0 - 1, u1 + 1, 100.0, 150.0, 140.0, 196.0))
    return prim.diff(b, cuts)


def _mid():
    """Mid plate (PLA), y 82 .. 88: spool shaft inboard bushing; screwed to the front plate and P1."""
    y0, y1 = Y_MID
    b = prim.union([Bw(U_FRONT[0] - 0.01, U_P1[1] + 0.01, y0, y1, 30.0, 88.5), _hole_y(12.0, SP_U, SP_W, y0, y1)])
    cuts = [_hole_y((S_D + fits.SHAFT_IN_BORE) / 2, SP_U, SP_W, y0 - 1, y1 + 1),
            _hole_u(fits.M3_CLEAR_D / 2, 0.5 * (y0 + y1), 88.5 - 4.0, -400, 0)]
    return prim.diff(b, cuts)


SHROUD_R = (33.5, 35.0)
SHROUD_DEG = (118.0, 370.0)   # covered arc about the spool axis, deg from +u toward +w (open at the top front)
SHROUD_Y = (125.5, 152.5)


def _shroud():
    """Spool shroud (PLA): arc shell r 33.5 .. 35 round the spool, open at the top front where the line leaves, with
    a tab screwed to the P1 leg. Keeps loose loops on the spool."""
    a0, a1 = SHROUD_DEG
    pts = [(SHROUD_R[1] * math.cos(math.radians(t)), SHROUD_R[1] * math.sin(math.radians(t))) for t in np.linspace(a0, a1, 72)]
    pts += [(SHROUD_R[0] * math.cos(math.radians(t)), SHROUD_R[0] * math.sin(math.radians(t))) for t in np.linspace(a1, a0, 72)]
    L = SHROUD_Y[1] - SHROUD_Y[0]
    sh = prim.extrude(prim.poly_cs(pts), L)                     # local x = u, y = w (about the spool axis), z = -yl
    t0, t1 = math.radians(a0), math.radians(a0 + 10.0)
    tab = prim.extrude(prim.poly_cs([(SHROUD_R[0] * math.cos(t0), SHROUD_R[0] * math.sin(t0)),
                                     (SHROUD_R[1] * math.cos(t0), SHROUD_R[1] * math.sin(t0)),
                                     (SHROUD_R[1] * math.cos(t1), SHROUD_R[1] * math.sin(t1)),
                                     (SHROUD_R[0] * math.cos(t1), SHROUD_R[0] * math.sin(t1))]),
                       Y_LEG[0] - SHROUD_Y[1] + 0.01, z=-(Y_LEG[0] - SHROUD_Y[1]))
    sh = prim.union([sh, tab])
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = (1, 0, 0), (0, 0, 1), (0, -1, 0)
    M[:3, 3] = (SP_U, Y(SHROUD_Y[1]), SP_W)
    return prim.place(sh, M)


# ------------------------------------------------------------------------------------------------ capstan group
C_U = (-262.5, -204.0)        # capstan tube: from behind P1b to the drum web
K_U = (-291.5, -183.0 + 0.0)  # cup shaft K: P2 journal to the cup neck


def _c_tube():
    """Capstan tube C (PPA-CF) in F_U: key rib under the drum (v7 key on +X), D flat under G_c, clip groove behind
    G_c, journals in P1 and P1b, 10.3 bore for K."""
    z0, z1 = C_U
    body = prim.cyl(C_D / 2, z1 - z0, z=z0)
    key = prim.box(C_D / 2 - 1.0, C_D / 2 + 2.6, -3.0, 3.0, Z_DRUM - 0.5, z1 - 0.8)
    body = prim.union([body, key])
    flat = prim.box(C_D / 2 - 2.0, C_D, -C_D, C_D, U_G[0] - 0.5, U_G[1] + 0.5)
    g = clips.groove_dims(C_D)
    groove = clips.clip_groove_cutter(C_D, z=U_G[1] + 0.3)               # clip in front of G_c (between G_c and P1)
    bore = prim.cyl((K_D + fits.SHAFT_IN_BORE) / 2, z1 - z0 + 2, z=z0 - 1)
    return body - prim.union([flat, bore])


def _k_shaft():
    """Cup shaft K (PPA-CF), F_U: 10 mm, D flat under the cup's rear half and under G_k, v7 neck and head in front of the
    cup for Harry's C-clip, journal in P2."""
    z0 = K_U[0]
    zf = Z_CUP + v7.CUP_D
    body = prim.cyl(K_D / 2, zf - z0, z=z0)
    body = prim.union([body, prim.cyl(2.25, 2.6 + 0.01, z=zf - 0.01), prim.cyl(2.8, 1.5, z=zf + 2.6, r2=2.3)])
    flat_cup = prim.box(K_D / 2 - 1.2, K_D, -K_D, K_D, Z_CUP - 0.5, zf + 0.01)
    flat_g = prim.box(K_D / 2 - 1.2, K_D, -K_D, K_D, U_GK[0] - 0.5, U_GK[1] + 2.0)
    return body - prim.union([flat_cup, flat_g])


def k_front():
    return Z_CUP + v7.CUP_D


# ------------------------------------------------------------------------------------------------ SW_ARM (as before)
SW_BUMP_R = 16.5
SW_BUMP_H = 2.1
SW_BOSS_H = 3.3               # D49: the 28T cup gear is bigger, so the track stands on a boss (switch screw clear of the teeth)
SW_BUMP_DEG = 12.0
SW_LEVER = 28.0
SW_TIP_X = -3.0


def _sw():
    return switch.contact_switch(SW_LEVER)


def sw_tip_deg():
    """Angle (deg about the cup axis, cup frame) of the SW_ARM lever tip = the home notch centre."""
    return math.degrees(math.atan2(-math.sqrt(SW_BUMP_R ** 2 - SW_TIP_X ** 2), SW_TIP_X))


def sw_frame():
    """Switch frame -> rail frame (via F_U at the capstan axis): switch +X = cup +Y (lever tip toward the axis),
    switch +Z = cup +Z = +u (toward G_k's rear face from P2's front face)."""
    i = _sw()["info"]
    z_top = i.get("z_lever_top", 16.5)
    tip_y = -math.sqrt(SW_BUMP_R ** 2 - SW_TIP_X ** 2)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = (0, 1, 0), (-1, 0, 0), (0, 0, 1)
    z_rest = U_GK[0] - SW_BOSS_H - 0.4           # free lever top 0.4 behind the notch floor
    M[:3, 3] = (SW_TIP_X, tip_y - i["x_act"], z_rest - z_top)
    return F_U(*CAP_YW) @ M


def _cup_gear_track():
    """Raised track on G_k's REAR face (gear frame z 0 = rear face, protruding -z) on a boss, r 13 .. 16, with a notch
    at home. The SW_ARM lever rides it (contact CLOSED) and drops into the notch at home (contact OPEN)."""
    boss = prim.tube(17.0, 7.0, SW_BOSS_H + 0.01, z=-SW_BOSS_H)
    ring = prim.tube(16.0, 13.0, SW_BUMP_H + 0.02, z=-SW_BOSS_H - SW_BUMP_H)
    half = math.radians(SW_BUMP_DEG)
    a_top = half + SW_BUMP_H / SW_BUMP_R

    def wedge(a, z0, h):
        pts = [(0.0, 0.0)] + [(20.0 * math.cos(t - math.pi / 2), 20.0 * math.sin(t - math.pi / 2))
                              for t in np.linspace(-a, a, 24)]
        return prim.extrude(prim.poly_cs(pts), h, z=z0)
    notch = Manifold.batch_hull([wedge(a_top + 0.5 / SW_BUMP_R, -SW_BOSS_H - SW_BUMP_H - 0.5, 0.5),
                                 wedge(half, -SW_BOSS_H - 0.01, 0.02)])
    notch = prim.place(notch, tf.Rz(sw_tip_deg() + 90.0))
    return boss + (ring - notch)


SW_PLATE_T = 4.0


def sw_base_holes():
    sw = _sw()
    lo, hi = prim.bbox(sw["base"])
    bw, rl = hi[1] - lo[1], -lo[0] - 2.0
    return [(x, s * (bw / 2 - 3.5)) for x in (-rl / 2, sw["info"]["x_flex"] / 2) for s in (-1, 1)]


# ------------------------------------------------------------------------------------------------ spool and clutch
Y_T = (123.0, 128.6)          # thrust ring (presses the inboard 608 inner race at the spool face y 128.8)
Y_SPOOL = (128.8, 151.2)      # spool body faces (flanges 2.0 + pack 20 between 129 and 149... + 0.2 each side)
Y_D = (151.5, 154.5)          # clutch disk D = head of the spool shaft, presses the spool's outboard face
ANNULUS = (18.0, 26.0)
S_Y = (62.0, 161.0)           # spool shaft S from the B3 bevel to the P1 leg


def _spool():
    """Spool (PLA), F_Y at the spool axis (z = yl): core r 20, flanges r 31, two 608 seats from both ends, tie hole."""
    z0, z1 = zY(Y_SPOOL[0]), zY(Y_SPOOL[1])
    rf, rc = FLANGE_R, CORE_R
    ft = 2.0
    ri = 7.5                                   # clears the 608 inner races
    prof = [(ri, z0), (rf - 0.6, z0), (rf, z0 + 0.6), (rf, z0 + ft), (rc + 1.5, z0 + ft), (rc, z0 + ft + 1.5),
            (rc, z1 - ft - 1.5), (rc + 1.5, z1 - ft), (rf, z1 - ft), (rf, z1 - 0.6), (rf - 0.6, z1), (ri, z1)]
    body = prim.revolve_pts(prof, seg=180)
    s_in = prim.place(bearings.seat_608(bottom="lip", lip_d=15.0, below=0.0), tf.T(0, 0, z0 - 0.02) @ tf.Rx(180))
    s_out = prim.place(bearings.seat_608(bottom="lip", lip_d=15.0, below=0.0), tf.T(0, 0, z1 + 0.02))
    tie = prim.cylY(1.0, 12.0, rc + 1.0, z=0.5 * (z0 + z1))
    return body - prim.union([s_in, s_out, tie])


def _s_shaft():
    """Spool shaft S (PPA-CF), F_Y (z = yl): 8 mm, D flat for the B3 bevel at the inboard end, head = clutch disk D
    with its friction annulus facing the spool, journal past D into the P1 leg."""
    z0, z1 = zY(S_Y[0]), zY(S_Y[1])
    body = prim.cyl(S_D / 2, z1 - z0, z=z0)
    flat = prim.box(S_D / 2 - 1.0, S_D, -S_D, S_D, z0 - 1, zY(B3_BEV_Y[1]) + 0.5)          # B3 bevel
    flat_d = prim.box(S_D / 2 - 1.0, S_D, -S_D, S_D, zY(Y_D[0]) - 0.5, zY(Y_D[1]) + 0.5)  # clutch disk
    pflat = prim.box(-S_D, -S_D / 2 + 1.0, -S_D, S_D, z0 - 1, z1 + 1)                      # print flat (-X)
    return body - prim.union([flat, flat_d, pflat])


def _d_disk():
    """Clutch disk D (PPA-CF), F_Y: D bore on the spool shaft flat, friction ring toward the spool."""
    z0, z1 = zY(Y_D[0]), zY(Y_D[1])
    d = prim.union([prim.cyl(ANNULUS[1] + 0.5, z1 - z0 - 0.4, z=z0 + 0.4), prim.tube(ANNULUS[1], ANNULUS[0], 0.41, z=z0)])
    rb = (S_D + fits.HOLE_SLIDE) / 2
    return d - (prim.cyl(rb, 10, z=z0 - 2) - prim.box(rb - 1.0 + 0.1, rb + 5, -rb - 5, rb + 5, z0 - 3, z1 + 3))


B3_BEV_Y = (66.0, 76.0)       # bevel on S: teeth 66 .. 70 (large end 70), hub 70 .. 76


def _thrust_ring():
    """Thrust ring T (PPA-CF), F_Y: slides on S, its boss presses the inboard 608's inner race; the clutch lever's
    FIRM screw bears on its inboard face."""
    z0, z1 = zY(Y_T[0]), zY(Y_T[1])
    return prim.union([prim.tube(9.0, (S_D + fits.HOLE_SLIDE) / 2, 4.0, z=z0),
                       prim.tube(6.0, (S_D + fits.HOLE_SLIDE) / 2, z1 - z0 - 4.0 + 0.01, z=z0 + 4.0 - 0.01)])


# clutch lever (in the cam plane U_LEVER, pivot parallel to u)
LEVER_PIVOT = (105.0, 118.0)          # (world y, w)
LEVER_PIVOT_U = 0.5 * sum(U_LEVER)
LEVER_POST_U = U_LEVER[0] - 0.5       # post end (lever bears on it)
LEVER_POST_R = 6.0
LEVER_FOLLOW = (52.0, 119.8)          # follower nose contact (world y, w): cad-drum's cam bottom, LIGHT
LEVER_TIP = (121.5, 62.0)             # FIRM screw axis (world y, w) at the thrust ring
FIRM_BOSS_BACK = 8.5                  # boss centre this far inboard of LEVER_TIP: its face stays 4.5 short of the ring
FIRM_TIP_Y = 122.9                    # FIRM screw tip at LIGHT (0.1 off the thrust ring face)
LIGHT_STOP = (95.0, 118.0)            # LIGHT stop screw (world y, w at the tip): presses the upper arm down
LIGHT_BLOCK_W = (124.0, 134.0)        # its tapped block above the arm, on P1's front face
LIGHT_STOP_U = 0.5 * (U_LEVER[0] + U_LEVER[1])


def lever_ratio():
    a = math.dist(LEVER_PIVOT, LEVER_FOLLOW)
    b = abs(LEVER_TIP[1] - LEVER_PIVOT[1])
    return b / a


def _lever():
    """Clutch lever (PPA-CF) in the rail frame at LIGHT: hub on the M3 pivot screw, upper arm under cad-drum's brake cam
    (rounded nose), lower arm down beside the capstan drum to the thrust ring with the FIRM screw boss. The lower arm
    is the clutch spring (4.6 thick PPA-CF leaf, reel_calc leaf_t): at FIRM it bends about 3 mm."""
    from shapely.geometry import LineString, Point
    from shapely.ops import unary_union
    py, pw = LEVER_PIVOT
    poly = unary_union([Point(Y(py), pw).buffer(6.0, 48),
                        LineString([(Y(py), pw - 5.0), (Y(LEVER_FOLLOW[0]) + 4.0, pw - 5.0)]).buffer(3.0, cap_style=2),
                        Point(Y(LEVER_FOLLOW[0]) + 3.0, LEVER_FOLLOW[1] - 3.0).buffer(3.0, 32),
                        LineString([(Y(py), pw), (Y(LEVER_TIP[0]) - FIRM_BOSS_BACK, LEVER_TIP[1])]).buffer(2.3, cap_style=2),
                        Point(Y(LEVER_TIP[0]) - FIRM_BOSS_BACK, LEVER_TIP[1]).buffer(5.0, 32)])
    s = prim.extrude(prim.shapely_cs(poly), U_LEVER[1] - U_LEVER[0])           # x = yl, y = w, z = u
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = (0, 1, 0), (0, 0, 1), (1, 0, 0)
    M[:3, 3] = (U_LEVER[0], 0.0, 0.0)
    s = prim.place(s, M)
    cuts = [_hole_u(fits.M3_CLEAR_D / 2, py, pw, U_LEVER[0] - 1, U_LEVER[1] + 1),
            _hole_y(fits.M3_TAP_D / 2, LEVER_PIVOT_U, LEVER_TIP[1], LEVER_TIP[0] - 18.0, LEVER_TIP[0] + 1.0)]
    return prim.diff(s, cuts)


# ------------------------------------------------------------------------------------------------ bridge
Y_BR = (30.0, 35.0)           # bridge plate (world y)
Y_SPRAG = (35.0, 49.0)        # sprag #2 hub
CSC_Y = (30.0, 70.5)          # countershaft C
CST_Y = (30.0, 54.5)          # countershaft T
C1P_Y = (40.0, 46.0)
T1_Y = (40.0, 46.0)
ARM_U, ARM_W = (60.0, 76.0), (17.0, 29.5)   # box arm to rail cheek A (web inner face y 118, under the rail wall)
UC_BLOCK_U = (-36.0, -28.0)                 # u-shaft C front bushing block (y 70 .. 83)
UT_BLOCK_U = (-138.0, -130.0)               # u-shaft T front bushing pillar


def bridge_outline():
    """(u, w) outline of the bridge plate: hulls of boss groups (countershaft C with the arm foot; C with c1', T and the
    u-shaft pillars; c1' with cad-drum's shift rail boss), minus r 41 round the main axis. Not one convex hull: the
    frame-fixed SW_INDEX block sits forward and below the axis (rail angles -101 .. +11 deg within r 78)."""
    from shapely.geometry import Point, MultiPoint
    from shapely.ops import unary_union

    def hull(ps):
        return MultiPoint([c for p in ps for c in p.exterior.coords]).convex_hull
    c = Point(C1_U, C1_W).buffer(15.0, 48)
    c1p = Point(C1P_U, C1P_W).buffer(10.0, 32)
    t = Point(T1_U, T1_W).buffer(11.0, 32)
    ut = Point(0.5 * sum(UT_BLOCK_U), UT_W).buffer(9.0, 32)
    uc = Point(-59.0, UC_W).buffer(8.0, 32)
    arm = Point(0.5 * sum(ARM_U), 0.5 * sum(ARM_W)).buffer(9.0, 32)
    groups = [hull([c, arm]), hull([c, c1p, t, ut, uc])]
    try:
        from cad.parts import drum as _dm
        groups.append(hull([c1p] + _dm.bridge_bosses(_P0)))
    except Exception:
        pass
    return unary_union(groups).difference(Point(0.0, 0.0).buffer(41.0, 96))


def _bridge():
    """Bridge (PLA), rail-fixed: the plate y 30 .. 35 inboard of the main-axis stack carries countershafts C and T, the
    c1' stub, cad-drum's shift rail and bell crank shelf; pillars to the u-shaft bushings; the box arm out to rail cheek A
    (2 x M3 x 16 into the cheek web at y 118). Prints on its inboard face."""
    from shapely.geometry import Polygon
    o = bridge_outline()
    if o.geom_type != "Polygon":
        o = max(o.geoms, key=lambda q: q.area)
    s = prim.extrude(prim.shapely_cs(o), Y_BR[1] - Y_BR[0])                 # x = u, y = w, z = -yl... build then turn
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = (1, 0, 0), (0, 0, 1), (0, -1, 0)
    M[:3, 3] = (0.0, Y(Y_BR[1]), 0.0)
    plate = prim.place(s, M)
    parts = [plate,
             _hole_y(14.0, C1_U, C1_W, Y_BR[0], Y_SPRAG[0] - 0.01),
             _hole_y(10.0, C1P_U, C1P_W, Y_BR[0], C1P_Y[0] - 0.3),
             _hole_y(11.0, T1_U, T1_W, Y_BR[0], T1_Y[0] - 0.3),
             # u-shaft T pillar: from the plate to a bushing round y 64
             Bw(UT_BLOCK_U[0], UT_BLOCK_U[1], Y_BR[1] - 0.01, UT_Y + 7.0, UT_W - 9.0, UT_W + 9.0),
             # u-shaft C L-bracket: pillar at u -62 .. -56 up to y 83, arm along u to the bushing block
             Bw(-62.0, -56.0, Y_BR[1] - 0.01, 83.0, UC_W - 9.0, UC_W + 9.0),
             Bw(-62.0, UC_BLOCK_U[1], 70.0, 83.0, UC_W - 9.0, UC_W + 9.0),
             # box arm to cheek A
             Bw(ARM_U[0], ARM_U[1], Y_BR[1] - 0.01, 118.0, ARM_W[0], ARM_W[1])]
    try:
        from cad.parts import drum as _dm
        parts += _dm.bridge_adds(_P0)
    except Exception:
        pass
    b = prim.union(parts)
    cuts = [_hole_y((20.0 + fits.SHAFT_IN_BORE) / 2, C1_U, C1_W, Y_BR[0] - 1, Y_SPRAG[0] + 1),
            _hole_y((8.0 + fits.HOLE_PRESS) / 2, C1P_U, C1P_W, Y_BR[0] - 1, C1P_Y[0]),
            _hole_y((12.0 + fits.SHAFT_IN_BORE) / 2, T1_U, T1_W, Y_BR[0] - 1, T1_Y[0]),
            _hole_u((UT_D + fits.SHAFT_IN_BORE) / 2, UT_Y, UT_W, UT_BLOCK_U[0] - 1, UT_BLOCK_U[1] + 1),
            _hole_u((UC_D + fits.SHAFT_IN_BORE) / 2, UC_Y, UC_W, UC_BLOCK_U[0] - 1, UC_BLOCK_U[1] + 1),
            Bw(UC_BLOCK_U[1], -20.0, 60.0, 90.0, UC_W - 12.0, UC_W + 12.0) if False else None]
    for w in (21.0, 26.0):
        cuts.append(_hole_y(fits.M3_CLEAR_D / 2, 0.5 * sum(ARM_U), w, 100.0, 119.0))
        cuts.append(_hole_y(3.0, 0.5 * sum(ARM_U), w, 100.0, 113.0))                # screwdriver access to the heads
    for (u, w) in SPRAG_HUB_SCREWS:
        cuts.append(_hole_y(fits.M3_TAP_D / 2, u, w, Y_BR[0] - 1, Y_BR[1] + 0.5))
    try:
        from cad.parts import drum as _dm
        cuts += _dm.bridge_cuts(_P0)
    except Exception:
        pass
    return prim.diff(b, [c for c in cuts if c is not None])


SPRAG_HUB_R = 28.0
SPRAG_HUB_SCREWS = [(C1_U + 23.0 * math.cos(math.radians(a)), C1_W + 23.0 * math.sin(math.radians(a)))
                    for a in (45.0, 135.0, 225.0, 315.0)]


def _sprag_hub():
    """Sprag #2 hub (PPA-CF), F_Y at countershaft C: CSK20PP crush-rib seat with the outer key rib, flange screwed to
    the bridge plate (4 x M3 x 16); reacts the capstan backstop torque into the bridge (the rail)."""
    z0, z1 = zY(Y_SPRAG[0]), zY(Y_SPRAG[1])
    body = prim.cyl(SPRAG_HUB_R, z1 - z0, z=z0)
    seat = prim.place(bearings.seat_csk20(depth=bearings.CSK20_W, bottom="through", key_deg=90.0),
                      tf.T(0, 0, z1))
    cuts = [seat]
    for (u, w) in SPRAG_HUB_SCREWS:
        cuts.append(prim.cyl(fits.M3_CLEAR_D / 2, z1 - z0 + 2, x=u - C1_U, y=-(w - C1_W), z=z0 - 1))
        cuts.append(prim.cyl(3.0, 4.0, x=u - C1_U, y=-(w - C1_W), z=z1 - 3.0))
    return prim.diff(body, cuts)


def _cs_c():
    """Countershaft C (PPA-CF), F_Y: d20 with the key rib in the sprag, d16 D flat for C1, d14 D flat for the B1 bevel."""
    z0, z1 = zY(CSC_Y[0]), zY(CSC_Y[1])
    zs1 = zY(Y_SPRAG[1])
    a = prim.cyl(C_D / 2, zs1 + 1.0 - z0, z=z0)
    key = prim.box(C_D / 2 - 1.0, C_D / 2 + 2.6, -3.0, 3.0, zY(Y_SPRAG[0]), zs1)
    b = prim.cyl(8.0, zY(C1_Y[1]) + 0.5 - zs1 - 1.0 + 0.01, z=zs1 + 1.0 - 0.01)
    c = prim.cyl(7.0, z1 - zY(C1_Y[1]) - 0.5 + 0.01, z=zY(C1_Y[1]) + 0.5 - 0.01)
    body = prim.union([a, key, b, c])
    flats = prim.union([prim.box(8.0 - 1.2, 20, -20, 20, zY(C1_Y[0]) - 0.5, zY(C1_Y[1]) + 0.5),
                        prim.box(7.0 - 1.0, 20, -20, 20, zY(C1_Y[1]) + 0.5, z1 + 1)])
    return body - flats


C1_Y = (50.5, 62.5)


def _cs_t():
    """Countershaft T (PPA-CF), F_Y: d12 journal in the plate, D flat d10 for T1 and the B2 bevel."""
    z0, z1 = zY(CST_Y[0]), zY(CST_Y[1])
    a = prim.cyl(6.0, zY(T1_Y[0]) - z0, z=z0)
    b = prim.cyl(5.0, z1 - zY(T1_Y[0]) + 0.01, z=zY(T1_Y[0]) - 0.01)
    body = prim.union([a, b])
    return body - prim.box(5.0 - 1.0, 20, -20, 20, zY(T1_Y[0]), z1 + 1)


def _stub():
    """c1' stub axle (PPA-CF d8), F_Y: pressed into the bridge plate, c1' and the swing arm turn on it."""
    z0, z1 = zY(Y_BR[0]), zY(C1P_Y[1]) + 4.0
    return prim.cyl(4.0, z1 - z0, z=z0) - prim.box(4.0 - 1.2, 10, -10, 10, z0 - 1, z1 + 1)


def _swing_arm():
    """Swing arm (PPA-CF), F_Y at c1': split ring on c1's hub (drag makes it swing), plate to the idler pin."""
    ix, iw = idler_uw(0.0)
    r = math.dist((ix, iw), (C1P_U, C1P_W))
    z0, z1 = zY(C1P_Y[1]) + 0.3, zY(C1P_Y[1]) + 3.3
    ringp = prim.tube(9.5, 6.6, z1 - z0, z=z0)
    arm = prim.box(0.0, r, -5.0, 5.0, z0, z1)
    pin = prim.cyl(3.85, z0 + 0.01 - zY(C1P_Y[0]) + 0.3, x=r, z=zY(C1P_Y[0]) - 0.3)
    hub = prim.cyl(6.0, z1 - z0, x=r, z=z0)
    return prim.union([ringp, arm, hub, pin]) - prim.box(-12, -6.0, -0.6, 0.6, z0 - 1, z1 + 1)


def _u_shaft(d, u0, u1, flat_front=None, flat_rear=None):
    s = prim.cyl(d / 2, u1 - u0, z=u0)
    cuts = []
    for f in (flat_front, flat_rear):
        if f:
            cuts.append(prim.box(d / 2 - 1.0, d, -d, d, f[0], f[1]))
    return prim.diff(s, cuts)


UC_U = (-256.5, -13.0)        # u-shaft C: B1 pinion (front) to behind G_a
UT_U = (-218.5, -114.0)       # u-shaft T: B2 pinion (front) to the B3 bevel
GA_HUB = 4.0


# ------------------------------------------------------------------------------------------------ parts
def parts(P):
    out = []

    def add(name, solid, mat, R=None, **k):
        if mat not in ("bought", "steel"):
            solid = prim.clean(solid)
        pt = _bed(solid, R) if mat not in ("bought", "steel") else None
        out.append(Part(name, solid, mat, print_tf=pt, **k))

    J = "launch_rail"
    # head structure
    add("reel_head_front", _front_plate(), "PLA", tf.Ry(90), joint=J, color=COL["plate"],
        bought_ref="4 x M3x16 SHCS (into the rail flange inserts); 3 x M3x12 SHCS (spine)",
        notes="Head front plate: bolts the reel head to the rail rear flange, u-shaft T rear bushing, link window. "
              "Print flat")
    add("reel_head_spine", _spine(), "PLA", tf.Rx(-90), joint=J, color=COL["plate"],
        notes="Head spine: inboard wall from the front plate to P2; the drum brackets and plates screw to it. Print flat")
    add("reel_head_p1", _p1(), "PLA", tf.Ry(-90), joint=J, color=COL["plate"],
        bought_ref="2 x M3x12 SHCS (spine)",
        notes="Head plate P1: capstan tube, u-shaft C and G_b bushings, clutch lever post, spool shaft leg. Print flat")
    add("reel_head_p1b", _p1b(), "PLA", tf.Ry(-90), joint=J, color=COL["plate"],
        bought_ref="2 x M3x12 SHCS (posts)", notes="Head plate P1b: capstan tube rear bushing, SW_ARM. Print flat")
    add("reel_head_p2", _p2(), "PLA", tf.Ry(90), joint=J, color=COL["plate"], bought_ref="2 x M3x12 SHCS (spine)",
        notes="Head plate P2: cup shaft and wrap idler rear journals. Print flat")
    add("reel_head_mid", _mid(), "PLA", tf.Rx(90), joint=J, color=COL["plate"],
        bought_ref="2 x M3x12 SHCS", notes="Head mid plate: spool shaft inboard bushing. Print flat")
    # capstan
    fu = F_U(*CAP_YW)
    add("reel_c", _c_tube(), "PPA-CF", joint="reel_capstan", color=COL["ppa"],
        notes="Capstan tube C (PPA-CF d20): key rib in the drum, G_c on its flat, journals in P1 and P1b. Print standing on its rear end")
    add("reel_drum", v7.drum(), "PPA-CF", joint="reel_capstan", place=tf.T(0, 0, Z_DRUM), color=COL["drum"],
        notes="Capstan drum dia 40 x 40 (Harry v7), keyed on the tube. Print flange down")
    add("reel_k", _k_shaft(), "PPA-CF", joint="reel_cup", color=COL["ppa"],
        notes="Cup shaft K (PPA-CF d10): runs inside the tube, cup in front, G_k behind P1b, journal in P2. Print standing on its rear end")
    add("reel_cup", v7.cup(), "PLA", joint="reel_cup", place=tf.T(0, 0, Z_CUP), color=COL["cup"],
        notes="Wrap cup (Harry v7, D49 axial eyelet at r 26): the line runs straight through the eyelet. Print open end down")
    add("reel_cup_clip", _cap_clip(), "PLA", joint="reel_cup", place=tf.T(0, 0, k_front() + 0.3), color=COL["clip"],
        notes="Harry's v7 C-clip on the cup shaft neck in front of the cup")
    gk = prim.union([_gear(M_CAP, Z_GK, U_GK[1] - U_GK[0], K_D, 1.2, phase=GK_PHASE),
                     _cup_gear_track()])
    add("reel_gk", gk, "PPA-CF", joint="reel_cup", place=tf.T(0, 0, U_GK[0]), color=COL["gear"],
        notes=f"Cup gear G_k m1.5 {Z_GK}T: driven by the drum's wrap idler 49T (7 cup turns per wrap); the raised track "
              "on its rear face holds SW_ARM closed, the notch at home lets it open. Print track face down")
    add("reel_gc", _gear(M_CAP, Z_GC, U_G[1] - U_G[0], C_D, 2.0, phase=GC_PHASE), "PPA-CF", joint="reel_capstan",
        place=tf.T(0, 0, U_G[0]), color=COL["gear"], notes=f"Capstan gear G_c m1.5 {Z_GC}T on the tube flat")
    add("reel_ga", _gear(M_CAP, Z_GA, U_G[1] - U_G[0], UC_D, 1.0, phase=GA_PHASE), "PPA-CF", joint="reel_uc",
        place=tf.T(0, 0, U_G[0]), color=COL["gear"], notes=f"G_a m1.5 {Z_GA}T on the rear end of u-shaft C")
    add("reel_gb", _gear(M_CAP, Z_GB, U_G[1] - U_G[0], GB_AXLE_D, 0.0, phase=GB_PHASE), "PPA-CF", joint="reel_gb",
        place=tf.T(0, 0, U_G[0]), color=COL["gear"], notes=f"G_b m1.5 {Z_GB}T idler on its stub in P1")
    gy, gw = gb_yw()
    add("reel_gb_axle", prim.cyl(GB_AXLE_D / 2, U_P1[1] - U_G[0] + 1.0, z=U_G[0] - 1.0), "steel",
        joint=J, place=F_U(gy, gw), color=COL["brg"], bought_ref="8 mm steel rod, 26 mm long",
        notes="G_b axle: 8 mm steel rod pressed into P1")
    add("reel_uc", _u_shaft(UC_D, UC_U[0], UC_U[1], flat_front=(UC_U[1] - 12.0, UC_U[1] + 1.0),
                            flat_rear=(UC_U[0] - 1.0, U_G[1] + 0.5)), "PPA-CF", joint="reel_uc",
        color=COL["ppa"], notes="u-shaft C (PPA-CF d10, 244 long): along the rail from the B1 bevel to G_a. Print standing "
                                "(244 of the 260 mm height); an 8 mm steel rod with printed sleeves is the stiff fallback")
    add("reel_ut", _u_shaft(UT_D, UT_U[0], UT_U[1], flat_front=(UT_U[1] - 10.0, UT_U[1] + 1.0),
                            flat_rear=(UT_U[0] - 1.0, UT_U[0] + 10.0)), "PPA-CF", joint="reel_ut",
        color=COL["ppa"], notes="u-shaft T (PPA-CF d8): along the rail from the B2 bevel to the B3 bevel. Print standing")
    # spool
    add("reel_spool", _spool(), "PLA", tf.Rx(180), joint="reel_spool", color=COL["spool"], infill=0.25,
        notes="Spool d40 core, d62 flanges, 20 wide: 20 m of 1 mm Dyneema, turns free on two 608s. Print flange down")
    for i, (yb, R) in enumerate(((Y_SPOOL[0] + bearings.B608_W, tf.I4), (Y_SPOOL[1], tf.I4)), 1):
        add(f"reel_brg_{i}", bearings.bearing_608(), "bought", joint="reel_spool", place=tf.T(0, 0, zY(yb)),
            color=COL["brg"], bought_ref="608ZZ 8x22x7", notes="608 bearing in the spool")
    add("reel_s", _s_shaft(), "PPA-CF", tf.Ry(-90), joint="reel_s", color=COL["ppa"],
        notes="Spool shaft S (PPA-CF d8): B3 bevel and the clutch disk on its flats, journals in the mid plate and the "
              "P1 leg. Print lying on its 1 mm flat")
    add("reel_d", _d_disk(), "PPA-CF", joint="reel_s", color=COL["ppa"],
        notes="Clutch disk D on the spool shaft flat: its friction ring presses the spool's outboard face. Print ring up")
    add("reel_t", _thrust_ring(), "PPA-CF", joint="reel_tring", color=COL["clip"],
        notes="Thrust ring: the lever's FIRM screw pushes it onto the inboard 608 inner race (clutch squeeze)")
    add("reel_shroud", _shroud(), "PLA", tf.Rx(90), joint=J, color=COL["plate"],
        bought_ref="1 x M3x8 SHCS (tab to the P1 leg)",
        notes="Spool shroud: keeps loose loops on the spool, open at the top front where the line leaves")
    LV = tf.T(0.0, -Y(LEVER_PIVOT[0]), -LEVER_PIVOT[1])          # rail frame -> lever joint frame
    add("reel_lever", _lever(), "PPA-CF", tf.Ry(90), joint="reel_lever", place=LV, color=COL["lever"],
        notes="Clutch lever (PPA-CF): the drum's brake cam presses its upper arm; the lower arm is the clutch spring "
              "leaf, its FIRM screw squeezes the spool. Print flat")
    add("reel_lever_pin", fasteners_m3(16.0), "bought", joint=J,
        place=tf.T(U_LEVER[1] + 3.0, Y(LEVER_PIVOT[0]), LEVER_PIVOT[1]) @ tf.Ry(-90), color=COL["brg"],
        bought_ref="M3x16 SHCS + washer", notes="Lever pivot screw, tapped into the P1 post")
    add("reel_firm_screw", fasteners_m3(10.0), "bought", joint="reel_lever",
        place=LV @ tf.T(LEVER_PIVOT_U, Y(FIRM_TIP_Y - 10.0), LEVER_TIP[1]) @ tf.Rx(-90), color=COL["brg"],
        bought_ref="M3x10 SHCS + M3 nut", notes="FIRM screw: tapped through the lever tip, locked by its nut")
    add("reel_light_screw", fasteners_m3(16.0), "bought", joint=J,
        place=tf.T(LIGHT_STOP_U, Y(LIGHT_STOP[0]), LIGHT_STOP[1] + 16.2) @ tf.Rx(180), color=COL["brg"],
        bought_ref="M3x16 SHCS + M3 nut", notes="LIGHT stop screw: tapped down through its block on P1, its tip holds the "
                                                 "lever's upper arm (sets LIGHT 0.25 N), lock nut on top")
    # SW_ARM
    sw = _sw()
    Msw = sw_frame()
    add("reel_sw_base", sw["base"], "PLA", joint=J, place=Msw, color=COL["clip"],
        bought_ref="4 x M3x10 SHCS (base to P2)", notes="SW_ARM contact base on P2's front face")
    add("reel_sw_lever", sw["lever"], "PLA", tf.Rx(90), joint=J, place=Msw, color=COL["lever"],
        bought_ref=f"2 x M3x{sw['info']['clamp_screw_len']} self tap (lever clamp)",
        notes="SW_ARM lever: rides the cup gear's raised track (contact closed), drops into its home notch (open)")
    add("reel_sw_hw", prim.union([sw["screw"], sw["nut"], sw["jam_nut"]]), "bought", joint=J, place=Msw,
        color=COL["brg"], bought_ref="M3x10 SHCS + 2 M3 nuts", notes="SW_ARM contact screw and nuts")
    # bridge
    add("reel_bridge", _bridge(), "PLA", tf.Rx(90), joint=J, color=COL["plate"],
        bought_ref="2 x M3x16 SHCS (arm to cheek A)",
        notes="Rail bridge: plate inboard of the main-axis stack with countershafts C and T, the c1' stub, pillars to the "
              "u-shaft bushings, cad-drum's shift rail; box arm bolted to rail cheek A. Print on its inboard face")
    add("reel_sprag_hub", _sprag_hub(), "PPA-CF", tf.Rx(180), joint=J, place=F_Y(C1_U, C1_W), color=COL["ppa"],
        bought_ref="4 x M3x16 SHCS", notes="Sprag #2 hub on the bridge: holds the CSK20PP capstan backstop")
    add("reel_sprag", bearings.bearing_csk20(okey_deg=90.0, ikey_deg=0.0), "bought", joint=J,
        place=F_Y(C1_U, C1_W) @ tf.T(0, 0, zY(Y_SPRAG[1])), color=COL["brg"], bought_ref="CSK20PP one-way bearing 20x47x14",
        notes="Sprag #2: countershaft C turns only in the capstan wind-in sense")
    add("reel_cs_c", _cs_c(), "PPA-CF", tf.Rx(180), joint="reel_csc", color=COL["ppa"],
        notes="Countershaft C (PPA-CF): sprag #2, C1 and the B1 bevel. Print standing")
    add("reel_c1", _gear(2.0, Z_C1, C1_Y[1] - C1_Y[0], 16.0, 1.2, phase=C1_PHASE, z0=zY(C1_Y[0])), "PPA-CF",
        joint="reel_csc", color=COL["gear"], notes=f"C1 m2 {Z_C1}T: meshes the REEL output gear 50T across the pivot")
    b1g, _ = bevel(1.5, 24, 12, 6.0, 14.0, hub=(10.0, 3.5))
    add("reel_b1g", b1g.translate((0, 0, zY(B1G_Y))), "PPA-CF", joint="reel_csc", color=COL["bevel"],
        notes="Bevel B1 gear m1.5 24T (2:1 up to u-shaft C)")
    b1p, _ = bevel(1.5, 12, 24, 6.0, UC_D, hub=(7.0, 6.0))
    add("reel_b1p", b1p.translate((0, 0, B1P_U)), "PPA-CF", joint="reel_uc", color=COL["bevel"],
        notes="Bevel B1 pinion m1.5 12T on the front end of u-shaft C")
    add("reel_c1p", _gear(M_TAKE, Z_C1P, C1P_Y[1] - C1P_Y[0], 8.0, 0.0, hub=(14.0, 3.6), phase=C1P_PHASE,
                          z0=zY(C1P_Y[0])), "PPA-CF", joint="reel_c1p", color=COL["gear"],
        notes=f"c1' m1.5 {Z_C1P}T, free on its stub: driven by the bus gear 50T, drives the swing idler")
    add("reel_stub", _stub(), "PPA-CF", tf.Rx(90), joint=J, place=F_Y(C1P_U, C1P_W), color=COL["ppa"],
        notes="c1' stub axle (PPA-CF d8, pressed into the bridge)")
    add("reel_swing_arm", _swing_arm(), "PPA-CF", tf.Rx(180), joint="reel_swing", color=COL["ppa"],
        notes="Swing arm on c1's hub: carries the idler into mesh with T1 on forward wiper turns")
    add("reel_gi", _gear(M_TAKE, Z_IDL, C1P_Y[1] - C1P_Y[0], 7.7, 0.0, z0=zY(C1P_Y[0])), "PPA-CF", joint="reel_idler",
        color=COL["gear"], notes=f"Swing idler m1.5 {Z_IDL}T")
    add("reel_cs_t", _cs_t(), "PPA-CF", tf.Rx(180), joint="reel_cst", color=COL["ppa"],
        notes="Countershaft T (PPA-CF): T1 and the B2 bevel")
    add("reel_t1", _gear(M_TAKE, Z_T1, T1_Y[1] - T1_Y[0], 10.0, 1.0, phase=T1_PHASE, z0=zY(T1_Y[0])), "PPA-CF",
        joint="reel_cst", color=COL["gear"], notes=f"T1 m1.5 {Z_T1}T: take-up at 2.5 x wiper")
    b2g, _ = bevel(1.0, 20, 20, 4.0, 10.0, hub=(7.0, 4.0))
    add("reel_b2g", b2g.translate((0, 0, zY(B2G_Y))), "PPA-CF", joint="reel_cst", color=COL["bevel"],
        notes="Bevel B2 m1 20T on countershaft T")
    b2p, _ = bevel(1.0, 20, 20, 4.0, UT_D, hub=(7.0, 5.0))
    add("reel_b2p", prim.place(b2p, tf.T(0, 0, B2P_U) @ tf.Rx(0)), "PPA-CF", joint="reel_ut", color=COL["bevel"],
        notes="Bevel B2 m1 20T on the front end of u-shaft T")
    b3a, _ = bevel(1.0, 20, 20, 4.0, UT_D, hub=(7.0, 5.0))
    add("reel_b3a", prim.place(b3a, tf.T(0, 0, B3A_U) @ tf.Rx(180)), "PPA-CF", joint="reel_ut", color=COL["bevel"],
        notes="Bevel B3 m1 20T on the rear end of u-shaft T")
    b3b, _ = bevel(1.0, 20, 20, 4.0, S_D, hub=(7.0, 6.0))
    add("reel_b3b", prim.place(b3b, tf.T(0, 0, zY(B3_BEV_Y[0] + 4.0)) @ tf.Rx(180)), "PPA-CF", joint="reel_s",
        color=COL["bevel"], notes="Bevel B3 m1 20T on the spool shaft")
    return out


B1G_Y = 67.5          # B1 gear large end (teeth toward the apex at y 76.5)
B1P_U = -18.0         # B1 pinion large end (teeth toward the apex at u 0)
B2G_Y = 50.0          # B2 gear large end (apex y 60)
B2P_U = -120.0        # B2 pinion large end (apex u -110)
B3A_U = -214.0        # B3 on u-shaft T: large end, teeth toward the apex at u -224 (built turned)
# tooth phases (deg, set by mesh_phase_check)
GA_PHASE = GB_PHASE = GC_PHASE = GK_PHASE = 0.0
C1_PHASE = C1P_PHASE = T1_PHASE = 0.0


def _cap_clip():
    """Harry's v7 cup C-clip: flat ring on the neck (r 2.25), opening toward -Y."""
    ri, ro, t = 2.3, 5.0, 2.4
    body = prim.tube(ro, ri, t)
    return body - prim.box(-1.8, 1.8, -10.0, 0.0, -1, t + 1)


# ------------------------------------------------------------------------------------------------ joints and kin
def joints(P):
    gy, gw = gb_yw()
    ix, iw = idler_uw(0.0)
    J = "launch_rail"
    fy = F_Y(0, 0)[:3, :3]
    base_y = np.eye(4)
    base_y[:3, :3] = fy
    base_u = np.eye(4)
    base_u[:3, :3] = F_U(0, 0)[:3, :3]
    sw0 = math.degrees(math.atan2(-(iw - C1P_W), ix - C1P_U))       # arm direction in F_Y (X = u, Y = -w)
    lo, hi = sorted((0.0, swing_sign() * SWING_OUT_DEG))
    return [
        Joint("reel_capstan", J, "rev", (0.0, Y(CAP_YW[0]), CAP_W), (0, 0, 1), base=base_u),
        Joint("reel_cup", J, "rev", (0.0, Y(CAP_YW[0]), CAP_W), (0, 0, 1), base=base_u),
        Joint("reel_uc", J, "rev", (0.0, Y(UC_Y), UC_W), (0, 0, 1), base=base_u),
        Joint("reel_gb", J, "rev", (0.0, Y(gy), gw), (0, 0, 1), base=base_u),
        Joint("reel_ut", J, "rev", (0.0, Y(UT_Y), UT_W), (0, 0, 1), base=base_u),
        Joint("reel_csc", J, "rev", (C1_U, 0.0, C1_W), (0, 0, 1), base=base_y),
        Joint("reel_c1p", J, "rev", (C1P_U, 0.0, C1P_W), (0, 0, 1), base=base_y),
        Joint("reel_swing", J, "rev", (C1P_U, 0.0, C1P_W), (0, 0, 1), base=base_y @ tf.Rz(sw0), limits=(lo, hi)),
        Joint("reel_idler", "reel_swing", "rev", (math.dist((ix, iw), (C1P_U, C1P_W)), 0, 0), (0, 0, 1)),
        Joint("reel_cst", J, "rev", (T1_U, 0.0, T1_W), (0, 0, 1), base=base_y),
        Joint("reel_s", J, "rev", (SP_U, 0.0, SP_W), (0, 0, 1), base=base_y),
        Joint("reel_spool", J, "rev", (SP_U, 0.0, SP_W), (0, 0, 1), base=base_y),
        Joint("reel_tring", J, "slide", (SP_U, 0.0, SP_W), (0, 0, 1), base=base_y, limits=(0.0, 0.5)),
        Joint("reel_lever", J, "rev", (0.0, Y(LEVER_PIVOT[0]), LEVER_PIVOT[1]), (1, 0, 0), limits=(-0.5, 8.0)),
    ]


def _num(S, k, default=0.0):
    v = S.get(k, default)
    return float(default if v is None else v)


def _mods():
    out = {}
    for n in ("drum", "launch"):
        try:
            import importlib
            out[n] = importlib.import_module(f"cad.parts.{n}")
        except Exception:
            out[n] = None
    return out


_MODS = {}


def _mod(n):
    if n not in _MODS:
        _MODS.update(_mods())
    return _MODS.get(n)


def mesh_offset(alpha_deg, z1, z2):
    """Phase offset (deg) of gear 2 for gear 1 at angle 0 (tooth 0 on +X), gear 2 centre at direction alpha_deg from
    gear 1, both about the same axis sense: theta2 = -(z1 / z2) theta1 + offset."""
    return alpha_deg + 180.0 + 180.0 / z2 + (z1 / z2) * alpha_deg


def _alpha_y(p1, p2):
    """Direction (deg) from gear 1 to gear 2 in an F_Y frame (X = u, Y = -w); p = (u, w)."""
    return math.degrees(math.atan2(-(p2[1] - p1[1]), p2[0] - p1[0]))


def _alpha_u(p1, p2):
    """Direction (deg) from gear 1 to gear 2 in an F_U frame (X = -yl, Y = -w); p = (world y, w)."""
    return math.degrees(math.atan2(-(p2[1] - p1[1]), -(p2[0] - p1[0])))


def pitch_deg(P, S):
    L = _mod("launch")
    if L is not None:
        try:
            return float(L.kin(P, S)["launch_rail"])
        except Exception:
            pass
    return _num(S, "aim_deg", 6.0)


def _drum_vals(P, S):
    D = _mod("drum")
    if D is None:
        return None
    try:
        return D.kin(P, S)
    except Exception:
        return None


def kin(P, S):
    p = pitch_deg(P, S)
    dv = None if ("bus_deg" in S or "capstan_deg" in S) else _drum_vals(P, S)
    wiper = _num(S, "wiper_deg")
    reel = _num(S, "reel_deg")
    if dv is not None:
        wiper = float(dv.get("drum_main", wiper))
        reel = float(dv.get("drum_out_reel", reel))
    # bridge: main-axis gears seen from the rail turn by (angle + pitch)
    b = wiper + p
    r = reel + p
    a_c1 = _alpha_y((0.0, 0.0), (C1_U, C1_W))
    th_c = -(Z_MAIN / Z_C1) * r + mesh_offset(a_c1, Z_MAIN, Z_C1) + C1_PHASE
    th_uc = -2.0 * th_c
    gy, gw = gb_yw()
    th_gb = -(Z_GA / Z_GB) * th_uc + mesh_offset(_alpha_u((UC_Y, UC_W), (gy, gw)), Z_GA, Z_GB)
    cap = -(Z_GB / Z_GC) * th_gb + mesh_offset(_alpha_u((gy, gw), CAP_YW), Z_GB, Z_GC)
    if "capstan_deg" in S:
        cap = float(S["capstan_deg"])
    th_c1p = -(Z_BUS / Z_C1P) * b + mesh_offset(_alpha_y((0.0, 0.0), (C1P_U, C1P_W)), Z_BUS, Z_C1P)
    if "bus_deg" in S:
        th_c1p = float(S["bus_deg"])
    swing = 0.0 if _num(S, "wiper_dir", 1.0) >= 0 else swing_sign() * SWING_OUT_DEG
    ip = idler_uw(swing)
    sw0 = math.degrees(math.atan2(-(idler_uw(0.0)[1] - C1P_W), idler_uw(0.0)[0] - C1P_U))
    a1 = _alpha_y((C1P_U, C1P_W), ip)
    th_i_world = -(Z_C1P / Z_IDL) * th_c1p + mesh_offset(a1, Z_C1P, Z_IDL)
    a2 = _alpha_y(ip, (T1_U, T1_W))
    th_t = -(Z_IDL / Z_T1) * th_i_world + mesh_offset(a2, Z_IDL, Z_T1) + T1_PHASE
    th_ut = -th_t
    th_s = th_t
    # cup: cad-drum's W output 49T (joint drum_idler about -u) meshes G_k 28T on K (about +u)
    cup = -360.0 * _num(S, "wraps")
    if dv is not None and "drum_idler" in dv and ("wraps" not in S or "drum_deg" in S):
        cup = (Z_WOUT / Z_GK) * (float(dv["drum_idler"]) - _idler_home(P)) + CUP_HOME
    if "spool_deg" in S:
        spool = float(S["spool_deg"])
    else:
        d = reel_calc.get(P)
        r_mid = 0.5 * (d["core_r"] + reel_calc.pack_r(d, d["line_len_m"]))
        spool = math.degrees(_num(S, "line_out_m") * 1000.0 / r_mid)
    return {"reel_capstan": cap, "reel_cup": cup, "reel_uc": th_uc, "reel_gb": th_gb, "reel_ut": th_ut,
            "reel_csc": th_c, "reel_c1p": th_c1p, "reel_swing": swing, "reel_idler": th_i_world - (sw0 + swing),
            "reel_cst": th_t, "reel_s": th_s, "reel_spool": spool, "reel_tring": 0.0,
            "reel_lever": lever_deg(P, S)}


CUP_HOME = 0.0
_IDL0 = {}


def _idler_home(P):
    key = id(P)
    if key not in _IDL0:
        dv = _drum_vals(P, dict(drum_deg=0.0))
        _IDL0[key] = float(dv["drum_idler"]) if dv and "drum_idler" in dv else 0.0
    return _IDL0[key]


def brake_lift(P, S):
    """Cam lift at the follower (0 LIGHT .. 4 FIRM) from cad-drum's track 2 profile."""
    if "reel_brake_mm" in S:
        return float(S["reel_brake_mm"])
    try:
        from cad.parts import _drum_kin as K
        D = _mod("drum")
        th = D.drum_theta(S, K.cfg(P)) if D is not None else _num(S, "drum_deg")
        return float(K.brake_lift(th, K.cfg(P)))
    except Exception:
        return 0.0


LEVER_GAP = 0.3               # follower nose gap to the cam base circle at LIGHT


_LEVER_CACHE = {}
NOSE_C = (55.0, 116.8)                # nose circle centre at LIGHT (world y, w), r 3


def lever_deg(P, S):
    """Lever turn (deg about +u at the pivot) that keeps the round nose LEVER_GAP clear of cad-drum's real cam profile
    (the nose touches the ramps off the follower line); positive presses the FIRM screw toward the spool."""
    a_ = math.dist(LEVER_PIVOT, LEVER_FOLLOW)
    if "reel_brake_mm" in S:
        return math.degrees(math.asin(min(max(0.0, float(S["reel_brake_mm"]) - LEVER_GAP) / a_, 1.0)))
    try:
        from cad.parts import _drum_kin as K
        D = _mod("drum")
        g, c = D.G(P), K.cfg(P)
        th = D.drum_theta(S, c)
    except Exception:
        return math.degrees(math.asin(min(max(0.0, brake_lift(P, S) - LEVER_GAP) / a_, 1.0)))
    key = (id(P), round(th, 4))
    if key in _LEVER_CACHE:
        return _LEVER_CACHE[key]
    o = g["head_o"]
    dx, dy = D.drum_xy(g)

    def ld(yw):                                       # world (y, w) -> Ld (x, y)
        return o[2] - yw[1], o[1] - (yw[0] - Y0)
    piv, n0 = ld(LEVER_PIVOT), ld(NOSE_C)

    def clear(a):                                     # min gap of the nose circle to the cam, lever turned by a
        t = math.radians(a)
        rx, ry = n0[0] - piv[0], n0[1] - piv[1]
        best = 1e9
        for sgn in (-1.0,):                           # +a about +u = -a about Ld z (z = -u)
            nx = piv[0] + rx * math.cos(sgn * t) - ry * math.sin(sgn * t)
            ny = piv[1] + rx * math.sin(sgn * t) + ry * math.cos(sgn * t)
            for k in range(-30, 31):
                psi = math.degrees(math.atan2(ny - dy, nx - dx)) + 0.5 * k
                r = D.cam_radius(g, c, psi - th)
                px, py = dx + r * math.cos(math.radians(psi)), dy + r * math.sin(math.radians(psi))
                best = min(best, math.hypot(nx - px, ny - py) - 3.0)
        return best
    if clear(0.0) >= LEVER_GAP - 0.05:
        v = 0.0
    else:
        lo, hi = 0.0, 8.0
        for _ in range(30):
            m = 0.5 * (lo + hi)
            if clear(m) < LEVER_GAP - 0.05:
                lo = m
            else:
                hi = m
        v = hi
    _LEVER_CACHE[key] = v
    return v


def allowed(P):
    return [
        Allow("reel_c", "reel_drum", "keyed fit"),
        Allow("reel_c", "reel_gc", "D-flat fit"),
        Allow("reel_c", "reel_head_p1*", "tube journals in P1 and P1b"),
        Allow("reel_k", "reel_cup", "D fit"),
        Allow("reel_k", "reel_cup_clip", "clip on the neck"),
        Allow("reel_k", "reel_gk", "D fit"),
        Allow("reel_k", "reel_c", "cup shaft runs inside the tube"),
        Allow("reel_k", "reel_head_p2", "journal in P2"),
        Allow("reel_uc", "reel_ga", "D-flat fit"),
        Allow("reel_uc", "reel_b1p", "D-flat fit"),
        Allow("reel_uc", "reel_head_p1", "journal in P1"),
        Allow("reel_uc", "reel_bridge", "journal in the bridge block"),
        Allow("reel_gb_axle", "reel_head_p1", "axle pressed into P1"),
        Allow("reel_gb_axle", "reel_gb", "idler turns on its axle"),
        Allow("reel_ut", "reel_b2p", "D-flat fit"),
        Allow("reel_ut", "reel_b3a", "D-flat fit"),
        Allow("reel_ut", "reel_head_front", "journal in the front plate"),
        Allow("reel_ut", "reel_bridge", "journal in the bridge pillar"),
        Allow("reel_brg_*", "reel_spool", "608 pressed in the spool"),
        Allow("reel_brg_*", "reel_s", "journal in the inner race"),
        Allow("reel_s", "reel_b3b", "D-flat fit"),
        Allow("reel_s", "reel_head_mid", "journal in the mid plate"),
        Allow("reel_s", "reel_head_p1", "journal in the P1 leg"),
        Allow("reel_s", "reel_t", "thrust ring slides on the shaft"),
        Allow("reel_t", "reel_brg_1", "boss on the inboard 608 inner race (clutch squeeze)", max_vol=1.0),
        Allow("reel_d", "reel_spool", "clutch disk on the spool's outboard face (friction annulus)", max_vol=2.0),
        Allow("reel_s", "reel_d", "D-flat fit"),
        Allow("reel_lever_pin", "reel_lever", "pivot screw through the lever"),
        Allow("reel_lever_pin", "reel_head_p1", "pivot screw tapped into the post"),
        Allow("reel_firm_screw", "reel_lever", "FIRM screw tapped through the lever tip"),
        Allow("reel_firm_screw", "reel_t", "FIRM screw tip on the thrust ring: the leaf bends about 3 mm at FIRM "
              "(static lever leaf in the model)", max_vol=40.0),
        Allow("reel_light_screw", "reel_head_p1", "screw tapped into P1"),
        Allow("reel_sw_hw", "reel_sw_*", "contact screw in the lever, nut in the base trap"),
        Allow("reel_sw_lever", "reel_sw_base", "lever clamped on the base pad"),
        Allow("reel_sw_lever", "reel_gk", "the track holds the lever down (contact closed) except at home", max_vol=60.0),
        Allow("reel_sw_base", "reel_head_p2", "base screwed to P2", max_vol=1.0),
        Allow("reel_sprag", "reel_sprag_hub", "CSK20PP pressed in its crush-rib seat, outer key rib"),
        Allow("reel_sprag", "reel_cs_c", "key rib in the sprag inner keyway (model is one solid)"),
        Allow("reel_cs_c", "reel_c1", "D-flat fit"),
        Allow("reel_cs_c", "reel_b1g", "D-flat fit"),
        Allow("reel_cs_c", "reel_bridge", "journal in the bridge plate"),
        Allow("reel_cs_c", "reel_sprag_hub", "shaft through the hub"),
        Allow("reel_b1g", "reel_b1p", "bevel mesh (approximate printed bevel model)"),
        Allow("reel_b2g", "reel_b2p", "bevel mesh (approximate printed bevel model)"),
        Allow("reel_b3a", "reel_b3b", "bevel mesh (approximate printed bevel model)"),
        Allow("reel_stub", "reel_bridge", "stub pressed into the plate"),
        Allow("reel_stub", "reel_c1p", "c1' turns on the stub"),
        Allow("reel_stub", "reel_swing_arm", "arm ring on the stub end"),
        Allow("reel_swing_arm", "reel_gi", "idler turns on the arm pin"),
        Allow("reel_swing_arm", "reel_c1p", "split ring grips c1's hub (drag)"),
        Allow("reel_cs_t", "reel_t1", "D-flat fit"),
        Allow("reel_cs_t", "reel_b2g", "D-flat fit"),
        Allow("reel_cs_t", "reel_bridge", "journal in the bridge plate"),
        Allow("reel_shroud", "reel_head_p1", "tab screwed to the P1 leg", max_vol=1.0),
        Allow("launch_line_a*", "reel_cup", "the line leaves the eyelet's front mouth"),
    ]


def clearances(P):
    return [
        Clearance("reel_cup", "reel_drum", 0.25, "cup skirt over the drum (v7 0.6 radial)"),
        Clearance("reel_spool", "reel_shroud", 1.0, "spool turns inside the shroud"),
        Clearance("reel_ga", "reel_gb", 0.0, "mesh"),
        Clearance("reel_gb", "reel_gc", 0.0, "mesh"),
        Clearance("reel_c1p", "reel_gi", 0.0, "mesh"),
        Clearance("reel_gi", "reel_t1", 0.0, "mesh"),
    ]


def meshes(P):
    T = tf.T
    f = U_G[1] - U_G[0]
    out = [Mesh("reel_ga", "reel_gb", M_CAP, Z_GA, Z_GB, face_a=f, face_b=f, frame_a=tf.I4, frame_b=tf.I4,
                why="G_a 26T to G_b 32T"),
           Mesh("reel_gb", "reel_gc", M_CAP, Z_GB, Z_GC, face_a=f, face_b=f, frame_a=tf.I4, frame_b=tf.I4,
                why="G_b 32T to G_c 23T"),
           Mesh("reel_c1p", "reel_gi", M_TAKE, Z_C1P, Z_IDL, face_a=6.0, face_b=6.0, frame_a=T(0, 0, zY(C1P_Y[0])),
                frame_b=T(0, 0, zY(C1P_Y[0])), why="c1' 46T to the swing idler 30T")]
    m = Mesh("reel_gi", "reel_t1", M_TAKE, Z_IDL, Z_T1, face_a=6.0, face_b=6.0, frame_a=T(0, 0, zY(C1P_Y[0])),
             frame_b=T(0, 0, zY(T1_Y[0])), why="swing idler 30T to T1 20T (forward turns)")
    m.when = lambda S: _num(S, "wiper_dir", 1.0) >= 0
    out.append(m)
    return out


def grounded(P):
    return []


def audit_states(P):
    out = []
    for k in range(6):
        out.append(dict(phase="reel_mesh", t=0.0, drum_deg=347.0, wiper_deg=k * 360.0 / Z_BUS / 5,
                        reel_deg=k * 360.0 / Z_MAIN / 5))
    for k in range(3):
        out.append(dict(phase="reel_swing_out", t=0.0, drum_deg=0.0, wiper_dir=-1, wiper_deg=-k * 5.0))
    for w in (0.0, 0.25, 0.5, 0.75, 1.0, 3.5, float(WRAPS)):
        out.append(dict(phase="reel_wrap", t=0.0, wraps=w))
    out.append(dict(phase="reel_firm", t=0.0, drum_deg=41.0, line_out_m=3.0))
    for a in (6.0, 47.0):
        out.append(dict(phase="reel_pitch", t=0.0, aim_deg=a, drum_deg=41.0))
    return out


# ------------------------------------------------------------------------------------------------ line path
LINE_R_CAP = v7.DR + 0.5          # line centre radius on the capstan


def _tangent_angle(p, r, upper=True):
    ang = math.degrees(math.atan2(p[1], p[0]))
    a = math.degrees(math.acos(min(1.0, r / math.hypot(p[0], p[1]))))
    return ang - a if upper else ang + a


def _rz(p, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array((c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]))


def pack_top(P, stored_m=None):
    """Spool tangent point (cup frame of the capstan axis: X = -yl, Y = -w, Z = u): the line leaves the pack top."""
    d = reel_calc.get(P)
    rp = reel_calc.pack_r(d, d["line_len_m"] - 2.0 if stored_m is None else stored_m)
    w = SP_W + rp + d["line_d"] / 2 - 0.5        # line centre: the design pack (r 27) puts it on the dart axis
    return np.array((0.0, -(w - CAP_W), SP_U))


def _line_points(P, wraps=0.0, stored_m=None):
    """Line centre polyline in the cup frame (X = -yl, Y = -w, Z = u about the capstan axis) as spans
    [(points N x 3, allowed reel parts, label)]: spool -> (capstan turns -> groove) -> eyelet (rear .. front mouth)."""
    g = pack_top(P, stored_m)
    psi = -360.0 * wraps
    cp = v7.cup_line_points()
    off = np.array((0, 0, Z_CUP))
    ring_r = _rz(cp["ring_rear"], psi) + off
    ring_f = _rz(cp["ring_front"], psi) + off
    spans = []
    if wraps <= 1e-9:
        spans.append((np.array([g, ring_r]), {"reel_spool", "reel_cup"}, "spool -> eyelet"))
    else:
        th0 = _tangent_angle(g[:2], LINE_R_CAP, upper=True)
        th_end = v7.TH_RIM + psi
        A = th0 - th_end
        if A <= 0:
            raise ValueError("wraps too small for the turn model")
        n = max(8, int(A / 10))
        z0, z1 = Z_DRUM + v7.FLANGE_H + 0.8, Z_CUP - 0.6
        turns = np.array([(LINE_R_CAP * math.cos(math.radians(th0 - A * k / n)),
                           LINE_R_CAP * math.sin(math.radians(th0 - A * k / n)), z0 + (z1 - z0) * k / n)
                          for k in range(n + 1)])
        spans.append((np.array([g, turns[0]]), {"reel_spool", "reel_drum"}, "spool -> capstan"))
        spans.append((turns, {"reel_drum"}, "capstan turns"))
        grv = []
        for k in range(9):
            tdeg = v7.TH_RIM - (v7.TH_RIM - v7.TH_TOP) * k / 8 + psi
            z = Z_CUP + v7.CUP_D * k / 8
            grv.append((v7.ER * math.cos(math.radians(tdeg)), v7.ER * math.sin(math.radians(tdeg)), z))
        spans.append((np.vstack([turns[-1:], np.array(grv), [ring_r]]), {"reel_drum", "reel_cup"},
                      "capstan -> cup groove -> eyelet"))
    spans.append((np.array([ring_r, ring_f]), {"reel_cup"}, "eyelet"))
    return spans


def _cup_to_rail():
    return F_U(*CAP_YW)


def _rail_to_world(P, S):
    p = pitch_deg(P, S)
    return tf.mul(tf.T(0.0, Y0, 0.0), tf.R((0, -1, 0), p))


def line_spans(P, S):
    """The reel's part of the tether for state S as WORLD spans [(points N x 3, allowed part names, label)]: spool
    pack top -> (capstan, groove) -> eyelet front mouth, with the cup as kin(P, S) poses it. In the first quarter turn
    the line is drawn straight (the turn model starts after 0.19 turns)."""
    d = reel_calc.get(P)
    stored = max(0.0, d["line_len_m"] - _num(S, "line_out_m"))
    k = kin(P, S)
    w = -k["reel_cup"] / 360.0
    if 0.0 < w < 0.32:                # the turn model needs the first tangent ahead of the groove (about 0.27 turn)
        w = 0.0
    M = _rail_to_world(P, S) @ _cup_to_rail()
    out = []
    for pts, allow, label in _line_points(P, w, stored_m=stored):
        out.append((tf.apply(M, np.asarray(pts)), set(allow), label))
    return out


def line_exit(P, S):
    """World point where the reel hands the line to cad-launch: the eyelet's front mouth (posed)."""
    return line_spans(P, S)[-1][0][-1]


def line_path(P, S):
    """Tether polyline in WORLD mm for state S: spool -> eyelet front mouth. cad-launch continues to the dart."""
    pts = []
    for span, _, _ in line_spans(P, S):
        for p in span:
            if not pts or np.linalg.norm(np.asarray(p) - pts[-1]) > 1e-6:
                pts.append(np.asarray(p, float))
    return [tuple(float(v) for v in p) for p in pts]


def _trim(pts, a, b):
    p0, p1 = pts[0], pts[-1]
    v = p1 - p0
    L = np.linalg.norm(v)
    u = v / L
    return np.array([p0 + u * a, p1 - u * b])


_PARTS_CACHE = {}


def _local_posed(P, S):
    """Every reel part posed in the RAIL frame for state S."""
    vals = kin(P, S)
    js = {j.name: j for j in joints(P)}
    key = id(P)
    if key not in _PARTS_CACHE:
        _PARTS_CACHE.clear()
        _PARTS_CACHE[key] = parts(P)
    out = {}
    for p in _PARTS_CACHE[key]:
        M = np.eye(4)
        j = p.joint
        chain = []
        while j and j != "launch_rail":
            chain.append(js[j]); j = js[j].parent
        for jj in reversed(chain):
            M = M @ jj.local(vals.get(jj.name, 0.0))
        out[p.name] = prim.place(p.solid, M @ p.place)
    return out


def line_check(P, wraps_list=None):
    """Exact check of the line tube (d 1.0) against every reel part (rail frame). Free spans clear everything except
    2.5 mm at the eyelet mouths; contact spans may touch only their listed parts."""
    if wraps_list is None:
        wraps_list = [0.0] + [1.0 + k / 12.0 for k in range(12)] + [float(WRAPS)]
    fails = []
    Mc = _cup_to_rail()
    for w in wraps_list:
        S = dict(wraps=w, mode="REEL")
        sol = _local_posed(P, S)
        for pts, allow, lab in _line_points(P, w):
            q = tf.apply(Mc, np.asarray(pts))
            if len(q) == 2 and lab in ("spool -> eyelet", "spool -> capstan"):
                q = _trim(q, 1.0, 2.5)
                hits = line.line_hits(q, {k: s for k, s in sol.items()})
            else:
                hits = line.line_hits(q, {k: s for k, s in sol.items() if k not in allow})
            for name, v in hits.items():
                if v > 1e-3:
                    fails.append(f"line at {w:.3f} wraps: {lab} hits {name} ({v:.3f} mm3)")
    return fails


# ---- whole-assembly tether check: the posed line against every posed part of every subsystem
TETHER_DT = 1.0
TETHER_HIT = 1e-3
TETHER_ALWAYS = ("launch_line_*",)            # cad-launch's model of the line itself
TETHER_EXTRA = {}


def tether_states(P):
    from tools.timeline import phases, state_at
    out, seen = [], set()
    for ph in phases(P):
        n = max(2, int(math.ceil((ph["t1"] - ph["t0"]) / TETHER_DT - 1e-9)))
        for k in range(n + 1):
            t = round(ph["t0"] + (ph["t1"] - ph["t0"]) * k / n, 6)
            if t not in seen:
                seen.add(t)
                out.append(state_at(P, t))
    out += [dict(phase="tether_wraps_0", t=0.0, wraps=0.0),
            dict(phase="tether_wraps_full", t=0.0, wraps=float(WRAPS)),
            dict(phase="tether_pitch_low", t=0.0, aim_deg=6.0, cock=1.0),
            dict(phase="tether_pitch_high", t=0.0, aim_deg=47.0, cock=1.0)]
    return out


def _module_spans(obj, sub):
    items = list(obj or [])
    if items and all(isinstance(x, (tuple, list)) and len(x) == 2 and isinstance(x[1], (set, frozenset, list, tuple))
                     and np.asarray(x[0], float).ndim == 2 for x in items):
        return [(np.asarray(p, float), set(a), f"{sub} span {k + 1} ({', '.join(sorted(a))})")
                for k, (p, a) in enumerate(items)]
    return [(np.asarray(items, float).reshape(-1, 3) if items else np.zeros((0, 3)), set(), f"{sub} line")]


def tether_check(P, asm, states=None):
    """Whole-assembly tether check: every span of the posed line (line_spans, then cad-launch's line_path spans) at
    every state, as the d 1.0 line tube, against every posed part except the parts that span is meant to touch.
    Returns one failure string per (span, part); an allowed name of a loaded subsystem that matches no part fails."""
    import inspect
    from fnmatch import fnmatchcase
    states = tether_states(P) if states is None else states
    names = list(asm.parts)
    box = {}
    for n, p in asm.parts.items():
        lo, hi = prim.bbox(p.solid)
        box[n] = np.array([[x, y, z, 1.0] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    others = [(sub, m) for sub, m in asm.modules.items() if sub != "reel" and callable(getattr(m, "line_path", None))
              and list(inspect.signature(m.line_path).parameters)[:2] == ["P", "S"]]
    pad = line.LINE_D / 2 + 0.01
    vols, hits, named, tubes = {}, {}, {}, {}
    for S in states:
        W = asm.world(S)
        spans = line_spans(P, S)
        for sub, m in others:
            spans += _module_spans(m.line_path(P, S), sub)
        prev, tubes = tubes, {}
        for pts, allow, label in spans:
            if len(pts) < 2:
                continue
            ok = set(allow) | set(TETHER_ALWAYS) | set(TETHER_EXTRA.get(label, ()))
            for a in ok:
                named.setdefault(a, label)
            lo, hi = pts.min(axis=0) - pad, pts.max(axis=0) + pad
            kp = (label, np.round(pts, 4).tobytes())
            for n in names:
                if any(fnmatchcase(n, a) for a in ok):
                    continue
                cw = (box[n] @ W[n].T)[:, :3]
                if np.any(cw.max(axis=0) < lo) or np.any(cw.min(axis=0) > hi):
                    continue
                key = (n, np.round(W[n], 4).tobytes(), kp)
                if key not in vols:
                    if kp not in tubes:
                        tubes[kp] = prev[kp] if kp in prev else line.line_tube(pts)
                    x = tubes[kp] ^ prim.place(asm.parts[n].solid, W[n])
                    v = x.volume()
                    at = np.asarray(x.bounding_box(), float).reshape(2, 3).mean(axis=0) if v > TETHER_HIT else None
                    vols[key] = (v, at)
                v, at = vols[key]
                if v > TETHER_HIT:
                    h = hits.setdefault((label, n), dict(v=0.0, n=0))
                    h["n"] += 1
                    if v > h["v"]:
                        h.update(v=v, at=at, phase=S.get("phase", ""), t=float(S.get("t", 0.0)))
    out = []
    for (label, n), h in sorted(hits.items()):
        x, y, z = (round(float(c), 1) for c in h["at"])
        out.append(f"tether: {label} hits {n} ({h['v']:.2f} mm3) in {h['n']} of {len(states)} states, worst "
                   f"{h['phase']} t {h['t']:.1f} near world ({x}, {y}, {z})")
    for a, label in sorted(named.items()):
        if a.split("_")[0] in asm.modules and not any(fnmatchcase(n, a) for n in names):
            out.append(f"tether: allowed contact {a} ({label}) names no part")
    return out


def deck_check(P):
    """Every reel part (rail frame, rest pose) above the 47 deg deck plane (study 4.2); returns failure strings."""
    sol = _local_posed(P, dict(drum_deg=0.0))
    cut = _deck_cut()
    out = []
    for n, s in sol.items():
        v = (s ^ cut).volume()
        if v > 1e-3:
            out.append(f"{n} reaches below the deck plane at 47 deg ({v:.1f} mm3)")
    return out


def checks(P, asm=None):
    """Reel line check and the deck rule; with the assembly (tools.audit passes it) the whole-assembly tether check."""
    out = line_check(P) + deck_check(P)
    if asm is not None:
        out += tether_check(P, asm)
    return out


# ------------------------------------------------------------------------------------------------ steps
def steps(P):
    """Assembly steps (plain language, hardware named per step). Bridge 220..224, head 225..239 (params)."""
    si = _sw()["info"]
    return [
        dict(title="Reel bridge on rail cheek A", stage=220,
             text="Press the 8 mm c1' stub into its hole in the bridge plate. Press the CSK20PP into the sprag hub (key "
                  "slot on the printed rib; seen from the outboard side the inner ring must turn freely counter-clockwise "
                  "and lock clockwise) and screw the hub to the plate's outboard face with four M3 x 16. Hold the bridge "
                  "inboard of the main shaft stack, below the REEL gear, and bolt its box arm to the inner face of rail "
                  "cheek A with two M3 x 16 (heads reached through the two holes in the arm).",
             parts=["reel_bridge", "reel_stub", "reel_sprag_hub", "reel_sprag"]),
        dict(title="Countershaft C, C1 and bevel B1", stage=221,
             text="Slide countershaft C into the sprag from the outboard side, key rib in the keyway, until its collar "
                  "meets the plate. Slide C1 (49 teeth) onto the D-flat so it meshes the REEL gear, then the 24-tooth "
                  "bevel, teeth outboard. Turn the REEL gear by hand: C1 turns one way only (the sprag holds the other).",
             parts=["reel_cs_c", "reel_c1", "reel_b1g"]),
        dict(title="Take-up gears on the bridge", stage=222,
             text="Slide c1' (46 teeth) onto the stub, hub outboard, meshing the bus gear. Clip the swing arm's split "
                  "ring over c1's hub and put the 30-tooth swing idler on the arm pin. Push countershaft T into its "
                  "bushing and slide T1 (20 teeth) and the B2 bevel onto its flat. Turn the wiper forward by hand: the "
                  "idler swings into T1; backward it swings out.",
             parts=["reel_c1p", "reel_swing_arm", "reel_gi", "reel_cs_t", "reel_t1", "reel_b2g"]),
        dict(title="Shafts along the rail", stage=223,
             text="Slide the 12-tooth B1 pinion onto the front flat of u-shaft C and feed the shaft rearward through the "
                  "bridge block until the pinion meshes the B1 bevel. Slide the B2 pinion onto the front flat of u-shaft T "
                  "and feed it rearward through the bridge pillar until the pinion meshes B2. Both shafts reach back to "
                  "the reel head.",
             parts=["reel_b1p", "reel_uc", "reel_b2p", "reel_ut"]),
        dict(title="Head frame on the rail", stage=225,
             text="Screw the front plate to the spine (three M3 x 12 through the spine into the plate). Slide the front "
                  "plate's bushing over the rear end of u-shaft T and bolt the plate to the rail's rear flange with four "
                  "M3 x 16 into the flange inserts. Screw the mid plate to the front plate (M3 x 12).",
             parts=["reel_head_front", "reel_head_spine", "reel_head_mid"]),
        dict(title="Spool, shaft and clutch", stage=226,
             text="Press a 608 into each end of the spool. Push the line end through the small hole near the inboard "
                  "flange, knot it inside (figure eight) and wind 20 m on (line over the top, leaving toward the rail). "
                  "Slide the clutch disk onto the outboard flat of the spool shaft (friction ring inboard), then from the "
                  "other end the spool until its outboard face meets the disk, then the thrust ring, boss toward the "
                  "spool. Feed the inboard end through the mid plate bushing "
                  "and slide the B3 bevel onto its flat. Slide the B3 bevel for u-shaft T onto the shaft's rear flat; the "
                  "two bevels mesh.",
             parts=["reel_spool", "reel_brg_1", "reel_brg_2", "reel_s", "reel_d", "reel_t", "reel_b3a", "reel_b3b"]),
        dict(title="Capstan tube, drum, cup shaft and cup", stage=227,
             text="Slide the capstan drum onto the front end of the capstan tube, key slot on the rib. Push the cup shaft "
                  "through the tube from the front (neck end in front), slide the cup onto it (flat in the bore onto the "
                  "flat, eyelet lug at the bottom, in line with the spool) and push Harry's small C-clip onto the neck.",
             parts=["reel_c", "reel_drum", "reel_k", "reel_cup", "reel_cup_clip"]),
        dict(title="P1, the capstan gears and the clutch lever", stage=228,
             text="Press the G_b axle (8 mm rod, 26 long) into P1. Slide P1 over the capstan tube, the rear end of u-shaft "
                  "C and the spool shaft end (its leg), and screw it to the spine (two M3 x 12) and to the mid plate. Put "
                  "G_c (23 teeth) on the tube flat, G_a (26) on u-shaft C's flat and G_b (32) on its axle. Hang the clutch "
                  "lever on the P1 post with the M3 x 16 pivot screw (it must swing), its upper arm under the drum's brake "
                  "cam, the FIRM screw toward the thrust ring. Screw the LIGHT stop screw into P1 under the upper arm.",
             parts=["reel_head_p1", "reel_gb_axle", "reel_gc", "reel_ga", "reel_gb", "reel_lever", "reel_lever_pin",
                    "reel_firm_screw", "reel_light_screw"]),
        dict(title="P1b, cup gear and P2", stage=232,
             text="Fit P1b over the rear end of the capstan tube and screw it to the two P1 posts (M3 x 12). Slide the cup "
                  "gear G_k (28 teeth, raised track forward) onto the cup shaft flat so it meshes the drum's wrap idler "
                  "output with the cup at home (eyelet at the bottom). Screw P2 to the spine over the cup shaft and wrap "
                  "idler ends (two M3 x 12). The cup must turn freely both ways.",
             parts=["reel_head_p1b", "reel_gk", "reel_head_p2"]),
        dict(title="SW_ARM contact", stage=233,
             text="Battery unplugged. Build the printed contact (see the contact switch page): nut pressed in the base "
                  f"trap, lever screwed onto the pad with 2 x M3 x {si['clamp_screw_len']} self tapping screws, "
                  f"M3 x {si['screw_len']} contact screw with its jam nut in the lever. Screw the base to P1b's rear face "
                  "with four M3 x 10. SW_ARM is closed while the cup is away from home and opens at home: the lever rides "
                  "the raised track on the front of the cup gear and drops into its notch at home. Cup at home: turn the "
                  f"contact screw until its head just touches the nut, back it off {si['gap'] * 2:.1f} turns (gap about "
                  f"{si['gap']:.1f} mm) and lock the jam nut. Check with the meter: about 15 degrees of cup turn either way "
                  "closes it; at home it must be open.",
             parts=["reel_sw_base", "reel_sw_lever", "reel_sw_hw"]),
        dict(title="Shroud, line and clutch setting", stage=234,
             text="Put the shroud round the spool, open side at the top toward the rail, and screw its tab to the P1 leg "
                  "(M3 x 8). Lead the line from the top of the spool straight forward under the capstan drum and through "
                  "the cup eyelet from behind, then forward through the pusher hole to the dart's tail cap. Wind 20 m "
                  "through the eyelet once on the bench before the first shot (D49: the pack mounds in the middle). Set "
                  "LIGHT with the drum at LAUNCH: turn the LIGHT stop screw until a 25 g weight hung on the line end just "
                  "pulls line off slowly (0.25 N). Set FIRM with the drum at NEUTRAL: turn the FIRM screw until the line "
                  "slips at 5.0 N on the luggage scale, then lock its nut.",
             parts=["reel_shroud", "launch_line_p"] + [f"launch_line_a{k}" for k in range(21)]),
    ]
