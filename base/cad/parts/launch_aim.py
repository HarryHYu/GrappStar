"""GrappStar base: AIM TRAIN of the launcher (worker cad-launch). Hooked into launch.py (parts_aim, joints_aim, kin_aim,
allowed_aim, clearances_aim, audit_states_aim).

Driven only by BACKWARD wiper turns (launch sleeve turning negative about +y):
  (1) splitter sun m1.5 z30 keyed on the launch sleeve
  (2) swing arm on the sleeve (friction drag: O-ring in a groove of its hub bore) carrying an idler m1.5 z18 on a
      printed pin; backward the arm swings the idler into mesh with
  (3) the output gear m1.5 z30 (one part with its d8 countershaft), forward it swings out against a stop post
  (4) bevel z30 on the countershaft (D flat) drives a bevel z15 (one part with the worm and its shaft), 2:1 up
  (5) worm m2 pd24 single start (self-locking) drives the worm wheel m2 z20 = crank gear, one PPA-CF part with the
      crank arm and the crank pin, on two 608 bearings on a fixed M8 bolt
  (6) conrod 156 between the crank pin and
  (7) the lug pin on the lug bracket bolted to the rail wall
  (8) aim housing (one PLA part on the deck z = 0) with the sleeve end bushing, countershaft bushings, worm saddles
      (cap on top), the M8 wheel bolt and the swing arm stop post.

World: x forward, y left, z up. Main axis along +y at (0, *, AXIS_Z). Plane order outward (world y):
  splitter 174..182 | arm 182.5..186.5 | housing inner wall 187..193 (behind x 22), conrod 187..193 (in front) |
  crank arm 193.6..199.6 | bevel z30 200.3..210 | wheel 208.2..220.2, worm axis 214.2 | outer wall 221..229.
The lug bracket stays inside y_rel 28..46.5 below the springs (w < 49) and puts its pin into the conrod plane.

Ratio: wiper / crank = worm_wheel_z / worm_starts / bevel_up / split_ratio = 20 / 1 / 2 / 1 = 10 (launch_kin).
Kinematics: the crank angle psi comes from the pitch (rising branch of launch_kin.pitch_of_psi); every gear angle,
the sleeve angle included, follows from psi through the train so all meshes stay phased in every state. S["aim_psi_from_low"]
(crank degrees from the LOW dead point) selects any crank angle, the falling stroke included. With
S["aim_split"] == "out" the arm sits on its stop and the sleeve follows S["wiper_deg"] (forward turns).
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from cad.lib import Part, Joint, Allow, Clearance, prim, tf   # noqa: E402
import launch_kin as K                                         # noqa: E402
import launch_gears as G                                       # noqa: E402

# ------------------------------------------------------------------------------------------------ numbers
FIT = 0.35            # diametral clearance pin in bore (gives real contact for the floating check)
M_S = 1.5             # splitter module
Z_SUN, Z_IDL, Z_OUT = 30, 18, 30
FACE_S = 8.0
M_B, Z_B1, Z_B2, FACE_B = 1.5, 30, 15, 8.0   # bevel pair (z30 on the countershaft, z15 on the worm shaft)
WORM_L = 32.0
BEV_PLAY = 0.2       # z15 moved back along its axis (backlash: the library pair overlaps up to ~1 mm3)
WHEEL_FACE = 12.0
R_CRANK_HUB = 16.5
SHAFT_R = 4.0         # countershaft and worm shaft d8
PIN_R = 4.0           # crank pin and lug pin d8
BRG = (22.0, 8.0, 7.0)   # 608

Y_SUN = 174.0
Y_ARM0, Y_ARM1 = 182.5, 186.5
Y_W1 = (187.0, 193.0)          # housing inner wall
Y_ROD = (187.0, 193.0)         # conrod plane
Y_CRANK = (193.6, 199.6)       # crank arm plane
Y_W2 = (221.0, 229.0)          # housing outer wall


def bed(solid, R=None, rz=0.0):
    """print_tf: rotate by R (4x4), then about z by rz deg, then translate so the part sits at z = 0, centred in x y."""
    M = tf.mul(tf.Rz(rz), R if R is not None else tf.I4)
    lo, hi = prim.bbox(prim.place(solid, M))
    return tf.mul(tf.T(-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]), M)


def A(P):
    """All aim geometry numbers (world mm / deg)."""
    d = K.lp(P)
    g = K.aim_geom(P)
    axis_z = 165.0
    try:
        import launch as LN
        axis_z = LN.L(P)["axis_z"]
        Y0 = LN.L(P)["Y0"]
    except Exception:                                     # pragma: no cover
        Y0 = 140.0
    n = dict(axis_z=axis_z, Y0=Y0, m_w=d["m_worm"], z_w=d["worm_wheel_z"], starts=d["worm_starts"],
             pd_w=d["worm_pd"], bevel_up=d["bevel_up"], split=d["split_ratio"])
    n["crank_x"], n["crank_z"] = g["cu"], axis_z + g["cw"]
    n["rc"], n["rod"], n["rl"] = g["rc"], g["rod"], g["rl"]
    n["a_ww"] = G.worm_center_distance(n["m_w"], n["z_w"], n["pd_w"])           # 32.1
    n["worm_z"] = n["crank_z"] + n["a_ww"]
    n["y_wheel_out"] = Y_W2[0] - 0.8                                            # wheel outer face
    n["y_w"] = n["y_wheel_out"] - WHEEL_FACE / 2                                # worm axis / wheel mid plane
    # countershaft: parallel to y, its axis at the worm height (bevel axes intersect)
    n["cs_x"] = 11.7
    n["cs_z"] = n["worm_z"]
    _, _, bi = G.bevel_pair(M_B, Z_B1, Z_B2, FACE_B)
    n["bev"] = bi
    n["y_b"] = n["y_w"] - bi["apex_z"][0]                                       # bevel z30 back face (inboard)
    # splitter in the alpha frame: X = world x, Y = -(world z - axis_z); alpha = rotation about +y
    n["cd_si"] = G.center_distance(M_S, Z_SUN, Z_IDL)
    n["cd_io"] = G.center_distance(M_S, Z_IDL, Z_OUT)
    ox, oy = n["cs_x"], -(n["cs_z"] - axis_z)
    D = math.hypot(ox, oy)
    n["D_so"] = D
    a_o = math.degrees(math.atan2(oy, ox))
    off = math.degrees(math.acos((D ** 2 + n["cd_si"] ** 2 - n["cd_io"] ** 2) / (2 * D * n["cd_si"])))
    n["alpha_out"], n["alpha_off"] = a_o, off
    n["alpha_e"] = a_o + off              # engaged: arm came from the + side (backward drags the arm -)
    n["alpha_d"] = n["alpha_e"] + 20.0    # disengaged: on the stop post
    ia = math.radians(n["alpha_e"])
    xi, yi = n["cd_si"] * math.cos(ia), n["cd_si"] * math.sin(ia)
    n["beta"] = math.degrees(math.atan2(oy - yi, ox - xi))                      # idler -> output direction
    # lug
    n["lug_u"] = n["rl"]
    return n


# ------------------------------------------------------------------------------------------------ helpers
def _ygear(solid, y0):
    """Solid built along +z (face from z = 0) -> axis along +y from world y0 (joint frames are world oriented)."""
    return tf.mul(tf.T(0, y0, 0), tf.Rx(-90))


def _alpha_xy(alpha, r):
    a = math.radians(alpha)
    return r * math.cos(a), r * math.sin(a)


# ------------------------------------------------------------------------------------------------ solids
def sun_solid(P):
    """Sun m1.5 z30 face 8 with a keyway in its bore for the sleeve rib (local +x)."""
    g = G.spur(M_S, Z_SUN, FACE_S)
    g = g - prim.cyl(10.0 + FIT / 2, FACE_S + 2, z=-1)
    g = g - prim.box(9.0, 13.2, -3.2, 3.2, -1, FACE_S + 1)           # keyway for the sleeve's integral rib (D13)
    return g


def arm_solid(P):
    """Swing arm: hub on the sleeve (O-ring groove in the bore), bar to the idler pin at alpha 0 (local +x), pin
    towards -z (inboard) through the idler, clip groove at the end."""
    n = A(P)
    t = Y_ARM1 - Y_ARM0
    r = n["cd_si"]
    s = prim.cyl(16.0, t) + prim.box(10.0, r, -6.0, 6.0, 0, t) + prim.cyl(7.0, t, x=r)
    s = s + prim.cyl(PIN_R, t + (Y_ARM0 - (Y_SUN - 1.5)), x=r, z=-(Y_ARM0 - (Y_SUN - 1.5)))
    s = s - prim.cyl(10.0 + FIT / 2, t + 2, z=-1)
    s = s - prim.tube(11.3, 10.0, 1.6, z=t / 2 - 0.8)                         # O-ring groove (drag on the sleeve)
    s = s - prim.tube(PIN_R + 1, PIN_R - 0.5, 0.8, z=-(Y_ARM0 - Y_SUN) - 1.4).translate((r, 0, 0))   # clip groove
    return s


def idler_solid(P):
    return G.spur(M_S, Z_IDL, FACE_S) - prim.cyl(PIN_R + FIT / 2, FACE_S + 2, z=-1)


def output_solid(P):
    """Output gear z30 + collar + d8 countershaft (local z = world y - Y_SUN), D flat for the bevel."""
    z_end = Y_W2[1] + 0.5 - Y_SUN
    s = G.spur(M_S, Z_OUT, FACE_S) + prim.cyl(8.0, Y_W1[0] - 0.1 - Y_SUN - FACE_S + 0.01, z=FACE_S - 0.01)
    s = s + prim.cyl(SHAFT_R, z_end - 1, z=1)
    s = s - prim.box(3.2, 5, -5, 5, 199.0 - Y_SUN, 211.0 - Y_SUN)            # D flat under the bevel
    return s


def bevel1_solid(P):
    """Bevel z30 (back face at local z 0 = world y_b, teeth towards +z) with a backing disc and a hub, D bore."""
    n = A(P)
    g1, _, bi = G.bevel_pair(M_B, Z_B1, Z_B2, FACE_B)
    back = n["y_b"] - (Y_CRANK[1] + 0.7)
    s = g1 + prim.cyl(18.0, back + 0.01, z=-back) + prim.cyl(8.0, 210.0 - n["y_b"], z=0)
    bore = prim.cyl(SHAFT_R + FIT / 2, 40, z=-20) - prim.box(3.2 + FIT / 2, 9, -9, 9, -21, 21)
    return s - bore


def worm_solid(P):
    """Worm + d8 shaft + bevel z15, in the worm frame of G.worm (axis +z, worm centred on z = 0); the bevel placement
    in this frame comes from _g2_local (meshes the bevel z30 at countershaft angle 0, worm angle 0)."""
    n = A(P)
    w = G.worm(n["m_w"], n["pd_w"], WORM_L, starts=n["starts"]).translate((0, 0, -WORM_L / 2))
    x0 = n["cs_x"] + n["bev"]["apex_z"][1] - n["bev"]["axial_height"][1]      # small end of the z15 (world x)
    z0 = x0 - n["crank_x"]
    z1 = 80.5 + 1.0 - n["crank_x"]
    s = w + prim.cyl(SHAFT_R, z1 - z0, z=z0)
    _, g2, _ = G.bevel_pair(M_B, Z_B1, Z_B2, FACE_B)
    s = s + prim.place(g2, _g2_local(P))
    return s


def wheel_solid(P):
    """Worm wheel z20 (lib frame: axis +z, face 0..12, z = 0 is the outboard face) + hub + crank arm + crank pin.
    Pocket for two 608 from the outboard face. Built pre-rotated by -mesh phase (place adds it back)."""
    n = A(P)
    yo = n["y_wheel_out"]
    zl = lambda y: yo - y                                                       # world y -> lib z
    g = G.worm_wheel(n["m_w"], n["z_w"], WHEEL_FACE, n["pd_w"], starts=n["starts"])
    s = g + prim.cyl(R_CRANK_HUB, zl(Y_CRANK[0]) - WHEEL_FACE + 0.01, z=WHEEL_FACE - 0.01)
    mp = G.mesh_phase(n["z_w"], 90.0)
    arm = prim.box(0, n["rc"], -7.0, 7.0, zl(Y_CRANK[1]), zl(Y_CRANK[0])) + \
        prim.cyl(8.0, Y_CRANK[1] - Y_CRANK[0], x=n["rc"], z=zl(Y_CRANK[1]))
    arm = arm + prim.cyl(PIN_R, Y_CRANK[0] - 186.0 + 0.01, x=n["rc"], z=zl(Y_CRANK[0]) - 0.01)
    arm = arm - prim.tube(PIN_R + 1, PIN_R - 0.5, 0.8, z=zl(186.0) - 1.6).translate((n["rc"], 0, 0))
    s = s + prim.place(arm, tf.Rz(-mp))
    s = s - prim.cyl(BRG[0] / 2 + 0.05, zl(206.2) + 1, z=-1)                   # 608 pocket
    s = s - prim.cyl(5.0, 2.0, z=zl(206.2) - 0.01)                              # bolt end
    return s


def conrod_solid(P):
    """Conrod in its joint frame: crank pin at the origin, lug pin at (+rod, 0), plane y Y_ROD."""
    n = A(P)
    L = n["rod"]
    y0, y1 = Y_ROD
    s = prim.box(0, L, y0, y1, -5.5, 5.5) + prim.cylY(8.5, y0, y1) + prim.cylY(8.5, y0, y1, x=L)
    s = s - prim.cylY(PIN_R + FIT / 2, y0 - 1, y1 + 1) - prim.cylY(PIN_R + FIT / 2, y0 - 1, y1 + 1, x=L)
    return s


def lug_solid(P):
    """Lug bracket in the rail frame (u, y_rel, w): bolted (2 x M3) to the rail wall outer face y_rel 28 at
    u 100..130, w 32..48 (below the springs), outer plate y_rel 40.5..46.5 down to the pin at (rl, w 0)."""
    n = A(P)
    import launch as LN
    ln = LN.L(P)
    yw = ln["y_wall1"] + 0.02
    u = n["lug_u"]
    yo = Y_ROD[0] - 0.5 - ln["Y0"]                                              # outer face (46.5)
    s = prim.box(100, 130, yw, yo, 34.0, 48.0)                                  # shelf on the wall
    s = s + prim.box(100, 130, yw, yw + 6.0, 32.0, 48.0)                        # bolting plate
    s = s + prim.box(103, 127, yo - 6.0, yo, -2.0, 48.0)                        # outer plate
    s = s + prim.cylY(10.0, yo - 6.0, yo, x=u, z=0.0)
    s = s + prim.cylY(PIN_R, yo - 0.01, Y_ROD[1] + 1.0 - ln["Y0"], x=u, z=0.0)
    for uu in (107.0, 123.0):
        s = s - prim.cylY(1.7, yw - 1, yo + 1, x=uu, z=41.0)
        s = s - prim.cylY(3.2, yw + 6.0, yo + 1, x=uu, z=41.0)                  # screw head / driver access
    return s


def housing_solid(P):
    """Aim housing (world frame), standing on the deck z = 0."""
    n = A(P)
    cx, cz = n["crank_x"], n["crank_z"]
    sx, sz = n["cs_x"], n["cs_z"]
    az = n["axis_z"]
    yw = n["y_w"]
    s = []
    a, b = Y_W1
    s.append(prim.cylY(18.0, a, b, 0.0, az))
    s.append(prim.box(-30, 14, a, b, 0, az))
    s.append(prim.cylY(10.0, a, b, sx, sz))
    s.append(prim.box(0, sx, a, b, sz - 10, sz + 10))
    a, b = Y_W2
    s.append(prim.box(-30, 86, a, b, 0, 80))
    s.append(prim.box(-30, sx, a, b, 80, sz + 10))
    s.append(prim.cylY(10.0, a, b, sx, sz))
    s.append(prim.cylY(7.0, 220.42, b, cx, cz))                                 # washer face on the bearing race
    s.append(prim.box(-30, 86, Y_W1[0], Y_W2[1], 0, 8))                         # foot
    s.append(prim.box(-30, -16, Y_W1[0], Y_W2[1], 0, 140))                      # spine
    for x0, x1 in ((35.0, 41.5), (74.5, 80.5)):                                  # worm saddles (lower halves)
        s.append(prim.box(x0, x1, yw - 7.2, Y_W2[1], 80.3, sz))
        s.append(prim.box(x0, x1, Y_W2[0], Y_W2[1], 79.0, 81.0))                 # join to the outer wall
    # swing arm stop post (touches the arm's leading face at alpha_d)
    px, py = _stop_post_xy(P)
    s.append(prim.cylY(4.0, Y_ARM0 + 0.1, Y_W1[0] + 0.01, px, az - py))
    h = prim.union(s)
    h = h - prim.cylY(10.0 + FIT / 2, Y_W1[0] - 1, Y_W1[1] + 1, 0.0, az)
    h = h - prim.cylY(SHAFT_R + FIT / 2, Y_W1[0] - 1, Y_W2[1] + 1, sx, sz)
    h = h - prim.cylY(4.1, 220, Y_W2[1] + 1, cx, cz)
    h = h - prim.cylX(SHAFT_R + FIT / 2, 30, 90, yw, sz)
    for x in (38.25, 77.5):
        for yy in (yw - 5.0, yw + 5.0):
            h = h - prim.cyl(1.25, 12, x=x, y=yy, z=sz - 11)                   # M3 self tap for the cap
    # deck screws
    for x, y in ((-24, 190.5), (80, 190.5), (-24, 225.5), (80, 225.5)):
        h = h - prim.cyl(1.7, 10, x=x, y=y, z=-1)
    return h


def cap_solid(P):
    n = A(P)
    yw, sz = n["y_w"], n["cs_z"]
    s = prim.union([prim.box(35.0, 41.5, yw - 7.2, yw + 6.8, sz + 0.05, sz + 19.5),
                    prim.box(74.5, 80.5, yw - 7.2, yw + 6.8, sz + 0.05, sz + 19.5),
                    prim.box(35.0, 80.5, yw - 7.2, yw + 6.8, sz + 15.5, sz + 19.5)])
    s = s - prim.cylX(SHAFT_R + FIT / 2, 30, 90, yw, sz)
    for x in (38.25, 77.5):
        for yy in (yw - 5.0, yw + 5.0):
            s = s - prim.cyl(1.7, 30, x=x, y=yy, z=sz - 1)
    return s


def bolt_solid(P):
    """M8 x 25 hex bolt along +z: shank z 0..(229 - 205.2), head on top."""
    Ls = Y_W2[1] - 205.2
    return prim.cyl(3.95, Ls + 0.1) + prim.cyl(7.5, 5.3, z=Ls + 0.02, seg=6)


def brg_solid(P):
    return prim.tube(BRG[0] / 2, BRG[1] / 2, BRG[2])


# ------------------------------------------------------------------------------------------------ frames
def _wheel_place(P):
    n = A(P)
    return tf.mul(tf.T(0, n["y_wheel_out"], 0), tf.Rx(90), tf.Rz(G.mesh_phase(n["z_w"], 90.0)))


def _worm_place(P):
    phi0 = -90.0 - 360.0 * (WORM_L / 2) / (math.pi * A(P)["m_w"] * A(P)["starts"])
    return tf.mul(tf.Rx(90), tf.Ry(90), tf.Rz(phi0))


def _worm_origin(P):
    n = A(P)
    return (n["crank_x"], n["y_w"], n["worm_z"])


def _g2_local(P):
    """Bevel z15 pose in the worm part frame, from the reference mesh: countershaft angle 0, worm angle 0."""
    n = A(P)
    bi = n["bev"]
    Ob = tf.T(n["cs_x"], n["y_b"], n["cs_z"])
    G2 = tf.mul(Ob, tf.Rx(-90), tf.T(bi["apex_z"][1] + BEV_PLAY, 0, bi["apex_z"][0]), tf.Ry(-90),
                tf.Rz(G.mesh_phase(Z_B2, 180.0)))
    Mw = tf.mul(tf.T(*_worm_origin(P)), _worm_place(P))
    return tf.mul(tf.inv(Mw), G2)


def _stop_post_xy(P):
    """Stop post centre (alpha frame X, Y) relative to the main axis."""
    n = A(P)
    rp, ro = 4.0, 26.0
    a = math.radians(n["alpha_d"])
    lx, ly = ro, 6.0 + rp + 0.1
    return lx * math.cos(a) - ly * math.sin(a), lx * math.sin(a) + ly * math.cos(a)


# ------------------------------------------------------------------------------------------------ module hooks
def parts_aim(P):
    n = A(P)
    sun, arm, idl, out, b1 = sun_solid(P), arm_solid(P), idler_solid(P), output_solid(P), bevel1_solid(P)
    wm, wh, rod, lug = worm_solid(P), wheel_solid(P), conrod_solid(P), lug_solid(P)
    hs, cap = housing_solid(P), cap_solid(P)
    bolt, brg = bolt_solid(P), brg_solid(P)
    Y0 = n["Y0"]
    yo = n["y_wheel_out"]
    return [
        Part("launch_aim_sun", sun, "PPA-CF", joint="launch_sleeve", place=_ygear(sun, Y_SUN - Y0),
             print_tf=bed(sun), color="#2f6db5", notes="Splitter sun m1.5 z30, keyed on the launch sleeve"),
        Part("launch_aim_arm", arm, "PPA-CF", joint="launch_aim_arm", place=_ygear(arm, Y_ARM0),
             print_tf=bed(arm, tf.Rx(180)), color="#5a6a7a",
             notes="Swing arm on the sleeve, O-ring 20x1.5 in the hub drags it; printed idler pin, e-clip groove"),
        Part("launch_aim_idler", idl, "PPA-CF", joint="launch_aim_idler", place=_ygear(idl, Y_SUN),
             print_tf=bed(idl), color="#3d7cc9", notes="Splitter idler m1.5 z18 on the arm pin"),
        Part("launch_aim_output", out, "PPA-CF", joint="launch_aim_cshaft", place=_ygear(out, Y_SUN),
             print_tf=bed(out), color="#2f6db5", notes="Splitter output m1.5 z30 with its d8 countershaft (D flat)"),
        Part("launch_aim_bevel30", b1, "PPA-CF", joint="launch_aim_cshaft", place=_ygear(b1, n["y_b"]),
             print_tf=bed(b1), color="#3d7cc9", notes="Bevel m1.5 z30 on the countershaft D flat, drives the z15 2:1"),
        Part("launch_aim_worm", wm, "PPA-CF", joint="launch_aim_worm", place=_worm_place(P),
             print_tf=bed(wm), color="#2f6db5",
             notes="Worm m2 pd24 single start (lead angle 4.8 deg, self-locking) + d8 shaft + bevel z15"),
        Part("launch_aim_crank", wh, "PPA-CF", joint="launch_aim_crank", place=_wheel_place(P),
             print_tf=bed(wh), color="#3d7cc9",
             notes="Worm wheel m2 z20 = crank gear, crank arm r30 and d8 crank pin in one part, two 608 inside"),
        Part("launch_aim_conrod", rod, "PPA-CF", joint="launch_aim_conrod", place=tf.I4,
             print_tf=bed(rod, tf.Rx(90)), color="#d97a00", notes="Conrod 156 between pin centres, e-clips"),
        Part("launch_aim_lug", lug, "PPA-CF", joint="launch_rail", place=tf.I4,
             print_tf=bed(lug, tf.Rx(-90)), color="#d97a00",
             notes="Lug bracket, 2 x M3 into the rail wall, d8 lug pin at (u 115.2, w 0)"),
        Part("launch_aim_housing", hs, "PLA", joint=None, place=tf.I4, print_tf=bed(hs), color="#9aa7b4",
             notes="Aim housing on the deck (4 x M3): sleeve end bushing, countershaft bushings, worm saddles, "
                   "M8 wheel bolt (captive nut), swing arm stop post"),
        Part("launch_aim_cap", cap, "PLA", joint=None, place=tf.I4, print_tf=bed(cap, tf.Rx(180)), color="#8796a5",
             notes="Worm shaft saddle cap, 4 x M3"),
        Part("launch_aim_bolt", bolt, "bought", joint=None,
             place=tf.mul(tf.T(n["crank_x"], 205.2, n["crank_z"]), tf.Rx(-90)), color="#777c84",
             bought_ref="M8x25 hex bolt ISO 4017 + M8 nut", notes="Fixed axle of the worm wheel"),
        Part("launch_aim_brg_1", brg, "bought", joint=None,
             place=tf.mul(tf.T(n["crank_x"], 213.4, n["crank_z"]), tf.Rx(-90)), color="#b0b5bb",
             bought_ref="608ZZ 8x22x7", notes="Worm wheel bearing (outboard)"),
        Part("launch_aim_brg_2", brg, "bought", joint=None,
             place=tf.mul(tf.T(n["crank_x"], 206.4, n["crank_z"]), tf.Rx(-90)), color="#b0b5bb",
             bought_ref="608ZZ 8x22x7", notes="Worm wheel bearing (inboard)"),
    ]


def joints_aim(P):
    n = A(P)
    az = n["axis_z"]
    wpc = K.wiper_per_crank(P)
    return [
        Joint("launch_aim_arm", None, "rev", (0.0, 0.0, az), (0, 1, 0)),
        Joint("launch_aim_idler", "launch_aim_arm", "rev", (n["cd_si"], 0.0, 0.0), (0, 1, 0)),
        Joint("launch_aim_cshaft", None, "rev", (n["cs_x"], 0.0, n["cs_z"]), (0, 1, 0),
              gear=("launch_aim_crank", -wpc / n["split"], 0.0)),
        Joint("launch_aim_worm", None, "rev", _worm_origin(P), (1, 0, 0),
              gear=("launch_aim_crank", -n["z_w"] / n["starts"], 0.0)),
        Joint("launch_aim_crank", None, "rev", (n["crank_x"], 0.0, n["crank_z"]), (0, 1, 0)),
        Joint("launch_aim_conrod", "launch_aim_crank", "rev", (n["rc"], 0.0, 0.0), (0, 1, 0)),
    ]


_PSI_CACHE = {}


def psi_of_pitch(P, pitch):
    """Crank angle psi (deg, launch_kin convention) on the rising branch for a pitch (deg), plus dpsi from LOW."""
    key = id(P)
    if key not in _PSI_CACHE:
        t = K.pitch_table(P)
        rise = t["crank_rise_deg"]
        dp = np.linspace(0.0, rise, 4001)
        pv = K.pitch_of_psi(P, t["psi_low"] - dp, +1)
        pv = np.maximum.accumulate(pv)
        _PSI_CACHE[key] = (t["psi_low"], dp, pv)
    psi_lo, dp, pv = _PSI_CACHE[key]
    d = float(np.interp(pitch, pv, dp))
    return psi_lo - d, d


def kin_aim(P, S, pitch):
    n = A(P)
    if "aim_psi_from_low" in S:                                # any crank angle (falling stroke too)
        psi = K.pitch_table(P)["psi_low"] - float(S["aim_psi_from_low"]) % 360.0
    else:
        psi, _ = psi_of_pitch(P, float(pitch))
    c = -psi                                                   # wheel / crank joint (about +y): pin at psi = -c
    w = -c * n["z_w"] / n["starts"]                            # worm (about +x)
    o = w / n["bevel_up"]                                      # countershaft (about +y)
    # conrod: absolute direction crank pin -> lug (alpha sense: about +y)
    cxp, czp = K.crank_pin(P, psi)
    lx, lz = K.lug_pos(P, pitch)
    a_rod = math.degrees(math.atan2(-(lz - czp), lx - cxp))
    out = {"launch_aim_crank": c, "launch_aim_worm": w, "launch_aim_cshaft": o, "launch_aim_conrod": a_rod - c}
    k_io, k_si = Z_OUT / Z_IDL, Z_IDL / Z_SUN
    if S.get("aim_split", "in") == "out":
        th = n["alpha_d"]
        s = float(S.get("wiper_deg", 0.0))
        I = G.mesh_phase(Z_IDL, th + 180.0) - (s - th) * Z_SUN / Z_IDL
    else:
        th = n["alpha_e"]
        be = n["beta"]
        I = be + (G.mesh_phase(Z_OUT, be + 180.0) - o) * k_io
        s = th + (G.mesh_phase(Z_IDL, th + 180.0) - I) * k_si
        out["launch_sleeve"] = s
    out["launch_aim_arm"] = th
    out["launch_aim_idler"] = I - th
    return out


def allowed_aim(P):
    return []


def clearances_aim(P):
    return [
        Clearance("launch_aim_conrod", "launch_aim_housing", 1.0, "conrod sweeps past the housing inner wall"),
        Clearance("launch_aim_crank", "launch_aim_housing", 0.5, "crank arm / wheel vs housing"),
        Clearance("launch_aim_lug", "launch_spring_*", 1.0, "lug bracket under the springs"),
    ]


def audit_states_aim(P):
    S = []
    for p in np.arange(6.02, 47.01, 2.0):
        S.append(dict(aim_deg=float(p), cock=0.0, phase="aim_sweep"))
    S.append(dict(aim_deg=47.0, cock=0.0, phase="aim_sweep"))
    t = K.pitch_table(P)
    for d in np.arange(t["crank_rise_deg"] + 10.0, 360.0, 15.0):              # falling stroke (worm-controlled)
        pv = float(K.pitch_of_psi(P, t["psi_low"] - d, +1))
        S.append(dict(aim_deg=pv, aim_psi_from_low=float(d), cock=0.0, phase="aim_fall"))
    for p, wd in ((6.02, 0.0), (20.0, 37.0), (35.0, 101.0), (47.0, 233.0)):
        S.append(dict(aim_deg=p, cock=0.5, wiper_deg=wd, aim_split="out", phase="aim_split_out"))
    return S
