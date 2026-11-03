"""GrappStar base: SENSORS subsystem (worker cad-sensors). Parts the firmware relies on that had no proper CAD:

1. AS5600 angle sensor (Adafruit PID 6357, STEMMA QT) facing the diametric magnet in drum_magnet_cap (magnet face
   y 201.5, params DRUM.magnet_face_y). Printed carrier screwed to the aim housing spine, 4 printed pegs through the
   board's corner holes, 2 printed clamp strips with M3 screws (Harry has no M2.5 screws).
2. SW_RAIL_LOW (GPIO 40): cad.lib.switch.contact_switch on a printed bracket on the aim housing foot, closed by the
   cam lobe on the aim crank (launch_aim.swl_cam_joint) around the LOW dead point (pitch 6 deg).
3. SW_GAP (GPIO 39): cad.lib.switch.contact_switch on a printed bracket screwed to the right rail cheek, closed by
   the cam bump on the big gear (launch.big_gear_solid) at its rest angle (gear in its toothless gap).

The spring guard (D33) is cad-frame's (frame_guards.py), not part of this module.

Board geometry is from Adafruit's EAGLE file (Adafruit-AS5600-Magnetic-Angle-Sensor-PCB): 25.4 x 17.78 x 1.6, four
plated 2.5 mm holes at (2.54, 2.54), (22.86, 2.54), (2.54, 15.24), (22.86, 15.24), the AS5600 (SOIC-8) at the board
centre (12.7, 8.89), JST SH 4 pin connectors at (3.048, 8.89) and (22.352, 8.89) opening to the short edges.

World frame: x forward, y left, z up, origin on the main axis (base/cad/README.md).
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from cad.lib import Part, Joint, Allow, Clearance, prim, tf, switch  # noqa: E402,F401
from cad.lib import fits  # noqa: E402

PREFIX = "sensors"

# ------------------------------------------------------------------------------------------------ AS5600 numbers
PCB_L, PCB_W, PCB_T = 25.4, 17.78, 1.6
PCB_HOLES = [(2.54, 2.54), (22.86, 2.54), (2.54, 15.24), (22.86, 15.24)]   # board coordinates (bx, by)
PCB_HOLE_D = 2.5
CHIP = (4.9, 3.9, 1.5)            # SOIC-8 body (x, z, height); datasheet A max 1.75
CHIP_H_MAX = 1.75
JST = dict(bx=(3.048, 22.352), along=7.0, deep=4.25, h=2.95)   # SM04B-SRSS-TB, height 2.95
PLUG_OUT = 6.0                    # JST SH plug + cable bend room beyond the board edge
CAP_R = 10.0                      # drum_magnet_cap radius (cad-drum magnet_holder)
JST_CAP_CLR = 0.55                # JST body to the cap end face (the cap turns)
PEG_D = 2.3
STRIP_BAND = 3.2                  # strip covers the board 3.2 mm in from each long edge (clear of parts)
STRIP_FRONT_GAP = 0.3             # strip thick part stops this short of the plate: the band clamps the board
STRIP_SCREW_Z = 14.0              # M3 clamp screws at z = +-14: heads 1.25 mm outside the cap radius
STRIP_SCREW_L = 12.0
PLATE_T = 8.0                     # carrier plate behind the board (nut pockets in its back)
PLATE_HALF_X, PLATE_HALF_Z = 17.0, 17.5
COL_X = (-16.0, -6.0)             # carrier column (on the housing spine's +x face at x -16)
COL_Z0 = -50.0
COL_LOW_Y0 = 204.6                # column below the plate (z < -18) is 10 thick in y, round the bolt holes
SPINE_X = -16.0                   # launch_aim housing spine +x face (box x -30..-16, y 187..229, z up to -25)
SPINE_TOP = -25.0
COL_BOLT_Z = (-33.0, -44.0)       # 2 x M3x16 along -x into the spine (pilots d2.6 cut by cad-launch at y 209.1)
COL_BOLT_Y = 209.1


def _p(P, name, default):
    v = getattr(P, name, default)
    return default if v is None else v


def magnet_face_y(P):
    d = _p(P, "DRUM", {}) or {}
    return float(d.get("magnet_face_y", 201.5))


def as_geom(P):
    """AS5600 stack along world y (all world mm). The chip-to-magnet gap is set by the JST connector height: the
    connectors sit at radius 7.5..11.8, inside the cap radius 10, so they must clear the turning cap face."""
    yf = magnet_face_y(P)
    y_board = yf + JST_CAP_CLR + JST["h"]               # board chip face (205.0 with the defaults)
    g = dict(y_mag=yf, y_board=y_board, y_board_back=y_board + PCB_T, y_chip=y_board - CHIP[2],
             gap=y_board - CHIP[2] - yf, gap_min=y_board - CHIP_H_MAX - yf)
    g["y_strip"] = y_board - 1.8                        # strip front face (band 1.8 on the chip face)
    g["y_peg_tip"] = y_board - 1.4
    g["y_plate"] = (g["y_board_back"], g["y_board_back"] + PLATE_T)
    g["axis_z"] = 0.0
    return g


def _b2w(bx, by, az=0.0):
    """Board coordinates -> world (x, z)."""
    return bx - PCB_L / 2, az + by - PCB_W / 2


def board_solid(P):
    """Adafruit AS5600 STEMMA QT (bought), world frame, chip facing -y (the magnet)."""
    g = as_geom(P)
    yb, yk = g["y_board"], g["y_board_back"]
    s = [prim.box(-PCB_L / 2, PCB_L / 2, yb, yk, -PCB_W / 2, PCB_W / 2),
         prim.box(-CHIP[0] / 2, CHIP[0] / 2, yb - CHIP[2], yb + 0.01, -CHIP[1] / 2, CHIP[1] / 2)]
    for bx in JST["bx"]:
        x, _ = _b2w(bx, 0)
        s.append(prim.box(x - JST["deep"] / 2, x + JST["deep"] / 2, yb - JST["h"], yb + 0.01,
                          -JST["along"] / 2, JST["along"] / 2))
    b = prim.union(s)
    for bx, by in PCB_HOLES:
        x, z = _b2w(bx, by)
        b = b - prim.cylY(PCB_HOLE_D / 2, yb - 1, yk + 1, x=x, z=z)
    return b


def carrier_solid(P):
    """Printed carrier: plate behind the board with 4 locating pegs and 2 hex nut pockets in its back for the strip
    screws, column down to the housing spine (2 x M3 along -x)."""
    g = as_geom(P)
    y0, y1 = g["y_plate"]
    s = [prim.box(-PLATE_HALF_X, PLATE_HALF_X, y0, y1, -PLATE_HALF_Z, PLATE_HALF_Z),
         prim.box(COL_X[0], COL_X[1], y0, y1, COL_Z0, PLATE_HALF_Z),
         prim.box(COL_X[0], COL_X[1], COL_LOW_Y0, y1, COL_Z0, -PLATE_HALF_Z - 0.5)]
    for bx, by in PCB_HOLES:
        x, z = _b2w(bx, by)
        s.append(prim.cylY(PEG_D / 2, g["y_peg_tip"], y0 + 0.01, x=x, z=z, seg=24))
    c = prim.union(s)
    nut = prim.place(prim.cyl(switch.M3_NUT_S / math.sqrt(3) + 0.08, 2.7, seg=6), tf.Rx(90))   # +z -> -y
    for zz in (STRIP_SCREW_Z, -STRIP_SCREW_Z):
        c = c - prim.cylY(fits.M3_CLEAR_D / 2, y0 - 1, y1 + 1, x=0.0, z=zz)
        c = c - nut.translate((0.0, y1 + 0.01, zz))
    for zb in COL_BOLT_Z:
        c = c - prim.cylX(fits.M3_CLEAR_D / 2, COL_X[0] - 1, COL_X[1] + 1.0, y=COL_BOLT_Y, z=zb)
    return c


def strip_solid(P, sg):
    """Clamp strip along one long board edge (sg +1 top, -1 bottom): band on the chip face over the corner holes, thick
    part past the board edge reaching to 0.3 mm short of the plate (so the band clamps the board), M3 clearance hole
    (screw head on the strip front, nut in the carrier back)."""
    g = as_geom(P)
    zin = PCB_W / 2 - STRIP_BAND                      # inner edge of the band (|z|)
    zout = STRIP_SCREW_Z + 3.5
    yf = g["y_strip"]
    a = prim.box(-PCB_L / 2, PCB_L / 2, yf, g["y_board"], *sorted((sg * zin, sg * (PCB_W / 2 + 0.3))))
    b = prim.box(-PCB_L / 2, PCB_L / 2, yf, g["y_plate"][0] - STRIP_FRONT_GAP,
                 *sorted((sg * (PCB_W / 2 + 0.3), sg * zout)))
    s = a + b
    for bx, by in PCB_HOLES:
        x, z = _b2w(bx, by)
        if z * sg > 0:
            s = s - prim.cylY(PEG_D / 2 + 0.15, yf - 1, g["y_board"] + 1, x=x, z=z)
    s = s - prim.cylY(fits.M3_CLEAR_D / 2, yf - 1, g["y_plate"][0] + 1, x=0.0, z=sg * STRIP_SCREW_Z)
    return s


def strip_nut(P, sg):
    g = as_geom(P)
    n = prim.place(switch._nut(0.0), tf.Rx(-90))                       # nut along +y from y 0
    return n.translate((0.0, g["y_plate"][1] - 0.1 - switch.M3_NUT_M, sg * STRIP_SCREW_Z))


def strip_screw(P, sg, L=STRIP_SCREW_L):
    """M3 x L SHCS, head on the strip front face, into the nut in the carrier back."""
    g = as_geom(P)
    yf = g["y_strip"]
    z = sg * STRIP_SCREW_Z
    return prim.cylY(1.5, yf - 0.01, yf + L, x=0.0, z=z, seg=20) + prim.cylY(2.75, yf - 3.0, yf, x=0.0, z=z, seg=24)


def column_screws(P, L=16.0):
    out = []
    xh = COL_X[1]                                          # head on the column's +x face
    for zb in COL_BOLT_Z:
        out.append(prim.cylX(1.5, xh - L, xh + 0.01, y=COL_BOLT_Y, z=zb, seg=20) +
                   prim.cylX(2.75, xh, xh + 3.0, y=COL_BOLT_Y, z=zb, seg=24))
    return prim.union(out)


def spine_pilots(P):
    """Spec for cad-launch: M3 self tap pilots in the aim housing spine +x face (world)."""
    return [dict(x=SPINE_X, y=COL_BOLT_Y, z=zb, d=fits.M3_TAP_D, depth=11.0, axis="-x") for zb in COL_BOLT_Z]


# ------------------------------------------------------------------------------------------------ switches
SW_LEVER = 30.0                   # cad.lib.switch defaults = the coupon 2 geometry (gap 0.93, strain 0.78 percent)
SW_PRESS = 0.3                    # actuator push past first contact (overtravel allowance 0.6)
RISER = 4.0                       # SW_GAP: pad on the lever tip so the jam nut and screw end stay below the gear web
_SW = {}


def sw_build(riser=0.0):
    """cad.lib.switch.contact_switch with the default (coupon 2) geometry. riser > 0 adds a printed pad of that height
    on the rigid lever tip beyond the jam nut (the actuator then touches the pad top); info["top"] = actuator face z."""
    key = float(riser)
    if key not in _SW:
        d = dict(switch.contact_switch(SW_LEVER))
        i = dict(d["info"])
        top = i["z_lever"] + 5.0                                    # boss_h 5 (contact_switch default)
        if riser > 0:
            b = i["b"]
            x0 = i["x_screw"] + switch.M3_NUT_S / math.sqrt(3) + 0.6
            d["lever"] = d["lever"] + prim.box(x0, SW_LEVER, -b / 2, b / 2, top - 0.01, top + riser)
            top += riser
        i["top"] = top
        i["riser"] = riser
        d["info"] = i
        _SW[key] = d
    return _SW[key]


def _sw_frame(origin_pt, x_dir, z_dir, i):
    """4x4 switch local -> carrier frame: local X -> x_dir, local Z -> z_dir, the actuator point
    (x_act, 0, lever top) lands on origin_pt (rest, unpressed)."""
    X = np.asarray(x_dir, float)
    Z = np.asarray(z_dir, float)
    Y = np.cross(Z, X)
    R = np.eye(4)
    R[:3, 0], R[:3, 1], R[:3, 2] = X, Y, Z
    loc = np.array([i["x_act"], 0.0, i["top"]])
    R[:3, 3] = np.asarray(origin_pt, float) - R[:3, :3] @ loc
    return R


def swl_geom(P):
    """SW_RAIL_LOW: cam lobe on the crank (launch_aim), dwell radius SWL_RT at psi 0 (world +x from the crank centre),
    cam plane y SWL_CAM_Y. Lever along world -y (root outboard), actuated towards +x."""
    import launch_aim as LA
    n = LA.A(P)
    i = sw_build()["info"]
    y_act = 0.5 * (LA.SWL_CAM_Y[0] + LA.SWL_CAM_Y[1]) - 0.2
    x_rest = n["crank_x"] + LA.SWL_RT - (i["pretravel"] + SW_PRESS)
    M = _sw_frame((x_rest, y_act, n["crank_z"]), (0, -1, 0), (-1, 0, 0), i)
    return dict(M=M, x_rest=x_rest, y_act=y_act, crank=(n["crank_x"], n["crank_z"]), deck_z=n["deck_z"], info=i)


SWL_FOOT = dict(x=(103.0, 130.4), y=(193.8, 201.8))   # = cad-launch housing foot extension x 80..130.9 (top at deck+8)
SWL_FOOT_PILOTS = ((107.0, 197.8),)   # M3 self tap d2.6 x 7 deep into the foot extension top (cad-launch also cut
                                      # one at x 116: it sits under the bracket column, unused); the second fixing
                                      # is the housing 4th deck screw, M3x20 from above through the foot


def _local_box_world(M, lo, hi):
    c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    w = tf.apply(M, c)
    return w.min(axis=0), w.max(axis=0)


def swl_bracket_solid(P):
    """Printed bracket (world): plate behind the switch base (world x beyond the base bottom), a column down to the
    housing foot extension and a foot on it (1 x M3 down + the housing 4th deck screw from above through the foot)."""
    g = swl_geom(P)
    d = sw_build()
    M = g["M"]
    lo, hi = (np.array(v) for v in prim.bbox(d["base"]))
    wlo, whi = _local_box_world(M, lo, hi)
    xb = whi[0]                                           # base bottom face (world x)
    z_top_foot = g["deck_z"] + 8.0
    s = [prim.box(xb, xb + 5.0, wlo[1], whi[1], wlo[2], whi[2])]                           # backing plate
    xdk, ydk = swl_deck_screw(P)
    s.append(prim.box(xb, min(xb + 10.0, xdk - 3.4), SWL_FOOT["y"][0], SWL_FOOT["y"][1], z_top_foot,
                      wlo[2] + 0.01))                                                         # column (clear of the
    #                                                                                           deck screw head)
    s.append(prim.box(SWL_FOOT["x"][0], SWL_FOOT["x"][1], SWL_FOOT["y"][0], SWL_FOOT["y"][1], z_top_foot,
                      z_top_foot + 6.0))                                                      # foot
    s.append(prim.box(xb - 6.0, xb + 0.01, SWL_FOOT["y"][0], SWL_FOOT["y"][1], z_top_foot + 5.99,
                      z_top_foot + 14.0))                                                     # gusset
    # web (about 48 deg) under the cantilevered plate (stiffness, no overhang when printed foot down)
    y0, y1 = SWL_FOOT["y"][1] - 0.01, whi[1] - 4.0
    zt = wlo[2] + 0.01
    web = prim.poly_cs([(y0, z_top_foot), (y0, zt), (y1, zt)])
    web = prim.place(prim.extrude(web, 5.0), np.array([[0, 0, 1.0, xb], [1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 0, 1.0]]))
    s.append(web)
    b = prim.union(s)
    for x, y in SWL_FOOT_PILOTS:
        b = b - prim.cyl(fits.M3_CLEAR_D / 2, 10.0, x=x, y=y, z=z_top_foot - 1)
        b = b - prim.cyl(3.0, 20.0, x=x, y=y, z=z_top_foot + 6.0)                           # head clearance up
    xdk, ydk = swl_deck_screw(P)
    b = b - prim.cyl(fits.M3_CLEAR_D / 2, 10.0, x=xdk, y=ydk, z=z_top_foot - 1)            # deck screw (from above)
    b = b - prim.cyl(3.2, 30.0, x=xdk, y=ydk, z=z_top_foot + 6.0)                          # its head and driver
    b = b - _mount_pilots(M, d["info"])
    return b


def swl_deck_screw(P):
    """cad-launch's 4th housing deck screw (x, y): from above through the bracket foot and the housing foot extension
    into the deck's heat-set insert."""
    try:
        import launch_aim as LA
        return LA.deck_holes(P)[3]
    except Exception:                                    # pragma: no cover
        return (125.9, 197.8)


def _mount_pilots(M, i, depth=5.5):
    """Self tap pilots in the backing plate under the switch base's 4 mounting holes (switch local -> M)."""
    d = sw_build(i.get("riser", 0.0))
    lo, hi = prim.bbox(d["base"])
    base_w = hi[1] - lo[1]
    root_len = -lo[0] - 2.0
    c = []
    for x in (-root_len / 2, i["x_flex"] / 2):
        for y in (-1, 1):
            c.append(prim.cyl(fits.M3_TAP_D / 2, depth + 0.5, x=x, y=y * (base_w / 2 - 3.5), z=-depth))
    return prim.place(prim.union(c), M)


def gap_geom(P):
    """SW_GAP on the rail (joint launch_rail, frame (u, y_rel, w)): bump top on the big gear web at y_rel
    y_sec0 - BUMP_H, bump centre at launch.home_bump_point. Lever runs from the bump outwards along the 225 deg ray
    from the gear pin; actuated towards -y."""
    import launch as LN
    n = LN.L(P)
    i = sw_build(RISER)["info"]
    u_t, w_t = LN.home_bump_point(P)
    y_bump = n["y_sec"][0] - LN.BUMP_H
    y_rest = y_bump + i["pretravel"] + SW_PRESS
    a = math.radians(GAP_DIR)
    xdir = (-math.cos(a), 0.0, -math.sin(a))             # root -> tip: from the root towards the bump
    M = _sw_frame((u_t, y_rest, w_t), xdir, (0, 1, 0), i)
    return dict(M=M, u=u_t, w=w_t, y_bump=y_bump, y_rest=y_rest, cd=n["cd"], y_chA=n["y_chA"], info=i)


GAP_DIR = 217.5               # direction (rail u-w, deg) from the bump to the lever root; within 29 deg of the radial
                              # 225 so the contact screw stays outside the bump's circle
GAP_BASE_Y = (-17.5, 7.8)     # switch local y kept of the SW_GAP base (no mounting wings needed: it has its arm);
                              # +y (down-forward, towards cad-reel's p2 post) trimmed 1.7 past the wire channel
GAP_TAB_PILOTS = ((48.0, 2.0), (64.0, 2.0))   # (u, w) M3 self tap d2.6 into the right cheek outer face, 7 deep along +y


def gap_arm_rail(P):
    """Arm of the SW_GAP base (rail frame): a web in the base plane from the base edge up to a tab on the right cheek's
    outer face (y_rel = cheek A outer face), 2 x M3x12 along +y into the cheek, heads in counterbores. The arm is
    printed in one piece with the switch base (no backing plate: cad-drum's dog post sits at world y < 111.5)."""
    g = gap_geom(P)
    M = g["M"]
    yo = g["y_chA"][0]                                    # cheek outer face (-17)
    yb = float(tf.apply(M, np.array([[0.0, 0.0, 0.0]]))[0, 1])                # base bottom (rail y)
    t = sw_build(RISER)["info"]
    base_t = 3.0
    u0, u1 = GAP_TAB_PILOTS[0][0] - 7.0, GAP_TAB_PILOTS[1][0] + 7.0
    tab = prim.box(u0, u1, yb, yo, -4.0, 8.0)
    web = prim.box(u0, u1, yb, yb + base_t, -24.0, -3.99)
    s = tab + web
    for u, w in GAP_TAB_PILOTS:
        s = s - prim.cylY(fits.M3_CLEAR_D / 2, yb - 1.0, yo + 1.0, x=u, z=w)
        s = s - prim.cylY(3.25, yb - 1.0, yb + 3.2, x=u, z=w)                 # head counterbore
    _ = t
    return s


def gap_base_solid(P):
    """SW_GAP printed base (switch local frame): cad.lib switch base + the arm to the cheek."""
    g = gap_geom(P)
    d = sw_build(RISER)
    lo, hi = prim.bbox(d["base"])
    keep = prim.box(lo[0] - 1, hi[0] + 1, GAP_BASE_Y[0], GAP_BASE_Y[1], lo[2] - 1, hi[2] + 1)
    arm = prim.place(gap_arm_rail(P), tf.inv(g["M"]))
    return (d["base"] ^ keep) + arm


def gap_cheek_pilots(P):
    g = gap_geom(P)
    return [dict(u=u, y_rel=g["y_chA"][0], w=w, d=fits.M3_TAP_D, depth=7.0, axis="+y (into the cheek)")
            for u, w in GAP_TAB_PILOTS]


# ------------------------------------------------------------------------------------------------ contract
def _bed(solid, R=None):
    import launch as LN
    if R is None:
        return LN.best_bed(solid)
    return LN.bed(solid, R)


def _switch_parts(tag, M, joint, where, riser=0.0, base=None, base_ref="4 x M3x8 self tap (base to bracket)"):
    d = sw_build(riser)
    i = d["info"]
    nm = f"{PREFIX}_sw_{tag}"
    up = f"SW_{tag.upper()}"
    base = d["base"] if base is None else base
    return [
        Part(nm, base, "PLA", joint=joint, place=M, print_tf=_bed(base, tf.I4), color="#5fae5f",
             bought_ref=base_ref,
             notes=f"{up} printed contact base (cad.lib.switch, coupon 2 geometry): {where}"),
        Part(nm + "_lever", d["lever"], "PLA", joint=joint, place=M, print_tf=_bed(d["lever"], tf.Rx(90)),
             color="#7cc47c", bought_ref=f"2 x M3x{i['clamp_screw_len']} self tap (lever clamp)",
             notes=f"{up} lever, t {i['t']:g} mm, print on its side so layers run along the beam"),
        Part(nm + "_screw", d["screw"], "bought", joint=joint, place=M, color="#444444",
             bought_ref=f"M3x{i['screw_len']} SHCS; M3 nut (jam)", notes=f"{up} contact screw, head down, jam nut"),
        Part(nm + "_jam", d["jam_nut"], "bought", joint=joint, place=M, color="#555555",
             notes=f"{up} jam nut on the contact screw (wire ring lug under it)"),
        Part(nm + "_nut", d["nut"], "bought", joint=joint, place=M, color="#444444", bought_ref="M3 nut",
             notes=f"{up} contact nut in its trap (wire soldered in the side slot)"),
    ]


def parts(P):
    g = as_geom(P)
    car = carrier_solid(P)
    out = [
        Part(f"{PREFIX}_as5600", board_solid(P), "bought", joint=None, place=tf.I4, color="#1a5fb4",
             bought_ref="Adafruit AS5600 STEMMA QT PID 6357; STEMMA QT JST SH 4 pin cable",
             notes=f"Main shaft angle sensor, chip on the axis {g['gap']:.1f} mm from the magnet face "
                   f"(1.75 to 2.0 with the SOIC height range); cable in the +x connector"),
        Part(f"{PREFIX}_as5600_carrier", car, "PLA", joint=None, place=tf.I4, print_tf=_bed(car, tf.Rx(-90)),
             color="#9aa7b4", bought_ref="2 x M3x16 SHCS (into the housing spine)",
             notes="AS5600 carrier: plate with 4 pegs through the board holes, column screwed to the aim housing spine"),
    ]
    for sg, tag in ((1, "top"), (-1, "bot")):
        st = strip_solid(P, sg)
        out.append(Part(f"{PREFIX}_as5600_strip_{tag}", st, "PLA", joint=None, place=tf.I4,
                        print_tf=_bed(st, tf.Rx(90)), color="#b8c4cc",
                        bought_ref=f"M3x{STRIP_SCREW_L:g} SHCS; M3 nut",
                        notes="AS5600 clamp strip on the board edge over the pegs, M3 from the strip front into a "
                              "nut in the carrier back"))
        out.append(Part(f"{PREFIX}_as5600_nut_{tag}", strip_nut(P, sg), "bought", joint=None, place=tf.I4,
                        color="#444444", notes="Clamp strip nut in the carrier back (counted on the strip)"))
        out.append(Part(f"{PREFIX}_as5600_screw_{tag}", strip_screw(P, sg), "bought", joint=None, place=tf.I4,
                        color="#444444", notes=f"Clamp strip screw M3x{STRIP_SCREW_L:g} (counted on the strip)"))
    out.append(Part(f"{PREFIX}_as5600_col_screws", column_screws(P), "bought", joint=None, place=tf.I4,
                    color="#444444", notes="2 x M3x16 carrier to spine (counted on the carrier)"))
    # SW_RAIL_LOW
    gl = swl_geom(P)
    br = swl_bracket_solid(P)
    out += _switch_parts("rail_low", gl["M"], None,
                         "on its bracket on the aim housing foot, closed by the crank cam at the LOW dead point")
    out.append(Part(f"{PREFIX}_sw_rail_low_bracket", br, "PLA", joint=None, place=tf.I4, print_tf=_bed(br, tf.I4),
                    color="#9aa7b4", bought_ref="M3x10 SHCS (foot into the housing); M3x20 SHCS (housing deck screw)",
                    notes="SW_RAIL_LOW bracket on the aim housing foot extension; the housing's 4th deck screw (M3x20 "
                          "from above) goes through its foot into the deck insert"))
    # SW_GAP
    gg = gap_geom(P)
    out += _switch_parts("gap", gg["M"], "launch_rail",
                         "with its arm screwed to the right rail cheek outer face below the big gear, closed by the "
                         "big gear cam bump (gear in its gap)", riser=RISER, base=gap_base_solid(P),
                         base_ref="2 x M3x12 SHCS (arm into the right cheek)")
    return out


def joints(P):
    return []                      # every part rides on world or on cad-launch's launch_rail


def kin(P, S):
    return {}


def allowed(P):
    out = []
    for tag in ("rail_low", "gap"):
        nm = f"{PREFIX}_sw_{tag}"
        out += [Allow(nm + "_screw", nm + "_lever", "contact screw threaded into the lever boss"),
                Allow(nm + "_jam", nm + "_screw", "jam nut on the screw thread"),
                Allow(nm + "_nut", nm, "contact nut pressed into its trap", max_vol=5.0)]
    out += [
        Allow(f"{PREFIX}_sw_rail_low_lever", "launch_aim_crank",
              "crank cam presses the lever tip at the LOW dead point (the lever bends; modelled rigid at rest)",
              max_vol=60.0),
        Allow(f"{PREFIX}_sw_gap_lever", "launch_big_gear",
              "big gear cam bump presses the lever tip at rest (the lever bends; modelled rigid at rest)",
              max_vol=60.0),
        Allow(f"{PREFIX}_as5600_screw_*", f"{PREFIX}_as5600_nut_*", "clamp screw in its nut"),
        Allow(f"{PREFIX}_as5600_col_screws", "launch_aim_housing", "M3 self tapping into the spine pilots"),
    ]
    return out


def clearances(P):
    return [
        Clearance(f"{PREFIX}_as5600", "drum_magnet_cap", 0.4, "JST connectors and chip vs the turning magnet cap"),
        Clearance(f"{PREFIX}_as5600_strip_*", "drum_magnet_cap", 0.4, "clamp strips vs the turning magnet cap"),
        Clearance(f"{PREFIX}_as5600_screw_*", "drum_magnet_cap", 0.4, "clamp screw tips vs the magnet cap"),
    ]


def audit_states(P):
    """SW_GAP: the big gear through the start of the cock (bump leaving the lever) and the whole cock; SW_RAIL_LOW is
    covered by launch's aim_sw_low and aim sweeps."""
    S = []
    for c in (0.0, 0.005, 0.01, 0.02, 0.03, 0.05, 0.08, 0.12, 0.2, 0.35, 0.5, 0.65, 0.8, 0.9, 1.0):
        for a in (6.0, 47.0):
            S.append(dict(aim_deg=a, cock=c, phase="sensors_gap_cock"))
    return S


def checks(P, asm=None):
    f = []
    i = sw_build()["info"]
    if not i["strain_ok"]:
        f.append(f"sensors: switch lever strain {i['strain_max']:.4f} over the limit")
    if SW_PRESS >= i["overtravel"]:
        f.append("sensors: SW_PRESS must stay below the switch overtravel (hard stop)")
    g = as_geom(P)
    if not (1.0 <= g["gap_min"] and g["gap"] <= 2.0 + 1e-6):
        f.append(f"sensors: AS5600 gap {g['gap_min']:.2f}..{g['gap']:.2f} outside 1..2 mm")
    return f


def steps(P):
    i = sw_build()["info"]
    g = as_geom(P)
    return [
        dict(title="Sensors 1: build the two contact switches (SW_RAIL_LOW, SW_GAP)",
             text=f"For each switch: press an M3 nut into the hex trap on the base pillar (a drop of glue), solder a "
                  f"wire to the nut through the side slot and lay it in the groove. Screw the lever onto the pad with "
                  f"2 x M3x{i['clamp_screw_len']} self tapping screws. Screw an M3x{i['screw_len']} through the "
                  f"lever boss from the top, head down, with a ring lug and the second wire under a jam nut on top. "
                  f"Turn the screw until the head touches the nut, then back it off {i['gap'] * 2:.1f} turns "
                  f"(gap about {i['gap']:.1f} mm, 0.5 mm per turn) and lock the jam nut.",
             parts=[f"{PREFIX}_sw_rail_low", f"{PREFIX}_sw_rail_low_lever", f"{PREFIX}_sw_rail_low_screw",
                    f"{PREFIX}_sw_rail_low_jam", f"{PREFIX}_sw_rail_low_nut"]),
        dict(title="Sensors 2: SW_RAIL_LOW on the aim housing",
             text="Screw the switch base to its bracket (4 x M3x8 self tapping), lever pointing to the crank. Set the "
                  "bracket foot on the housing foot extension in front of the crank. Screw it down with an M3x10 "
                  "in the hole nearest the crank (self taps into the housing), and put the housing's 4th deck screw "
                  "(M3x20 instead of the shorter one) down through the outer hole, the foot extension and into its "
                  "deck insert. Turn the crank by hand (wiper backwards) to the lowest rail position: the cam lobe "
                  "presses the lever tip and the meter shows the switch closed. Turn the crank about 20 deg either "
                  "side: the switch must open.",
             parts=[f"{PREFIX}_sw_rail_low_bracket"]),
        dict(title="Sensors 3: SW_GAP on the right rail cheek",
             text="Build SW_GAP as in Sensors 1 (its base has its own arm). Hold the arm tab on the outer face of the right rail cheek, below "
                  "the big gear, and screw it on with 2 x M3x12 (heads in the counterbores). With the big gear "
                  "at rest (toothless gap at the rack) the cam bump presses the lever: switch closed. Turn the gear "
                  "20 deg either way: switch open.",
             parts=[f"{PREFIX}_sw_gap", f"{PREFIX}_sw_gap_lever", f"{PREFIX}_sw_gap_screw", f"{PREFIX}_sw_gap_jam",
                    f"{PREFIX}_sw_gap_nut"]),
        dict(title="Sensors 4: AS5600 board",
             text="Do this after the magnet cap is on the main shaft end (drum steps): the board sits 2 mm in front of "
                  "it. Battery unplugged. Plug the STEMMA QT cable into the board's connector on the +x end (the end "
                  "away from the housing spine). Push the board onto the 4 pegs of the carrier, chip facing out. Drop "
                  "an M3 nut into each hex pocket in the carrier back, lay the strips over the board edges (pegs "
                  f"through their holes) and screw each from the strip front with an M3x{STRIP_SCREW_L:g} into its "
                  "nut. Screw the carrier "
                  "column to the aim housing spine with 2 x M3x16. Check by eye: the chip sits on the shaft axis, "
                  f"about {g['gap']:.1f} mm from the magnet face, and the cap turns free.",
             parts=[f"{PREFIX}_as5600", f"{PREFIX}_as5600_carrier", f"{PREFIX}_as5600_strip_top",
                    f"{PREFIX}_as5600_strip_bot", f"{PREFIX}_as5600_nut_top", f"{PREFIX}_as5600_nut_bot",
                    f"{PREFIX}_as5600_screw_top", f"{PREFIX}_as5600_screw_bot", f"{PREFIX}_as5600_col_screws"]),
    ]
