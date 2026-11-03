"""GrappStar base: AIM TRAIN of the launcher (worker cad-launch). Hooked into launch.py (parts_aim, joints_aim, kin_aim,
allowed_aim, clearances_aim, audit_states_aim).

Driven only by BACKWARD wiper turns (launch sleeve turning negative about +y):
  (1) splitter sun m1.5 z30 face 12 keyed on the launch sleeve
  (2) swing arm on the sleeve (friction drag: a printed PLA flexure pad, a cantilever finger in the arm hub bore
      pressing on the sleeve d20, preload set by a printed shim behind its root block) carrying an idler m1.5 z18 on
      a printed pin (printed C clip); backward the arm swings the idler into mesh with
  (3) the output gear m1.5 z30 (one part with its d8 countershaft) and comes to rest on the engaged stop post, which
      holds the idler at its design centre distance (the tooth forces pull the arm on towards the output, so without
      that post the idler would wedge into the output gear); forward it swings out against the other stop post
  (4) bevel z30 on the countershaft (D flat) drives a bevel z15 (one part with the worm and its shaft), 2:1 up
  (5) worm m2 pd24 single start (self-locking) drives the worm wheel m2 z20 face 16 = crank gear, one PPA-CF part
      with the crank arm and the crank pin, on two 608 bearings on a fixed M8 bolt
  (6) conrod 156 (PPA-CF) between the crank pin and
  (7) the lug pin on the lug bracket (PPA-CF) bolted to the rail wall; both pins carry printed C clips
Gear strength at the 5 A aim trip: see the LEWIS block below the numbers.
  (8) aim housing (one PLA part on the deck z = 0) with the sleeve end bushing, countershaft bushings, worm saddles
      (cap on top), the M8 wheel bolt and the two swing arm stop posts (engaged and swung out); 4 x M3 x 10 from the
      top through counterbored foot flanges into cad-frame's deck inserts (deck_holes).

World: x forward, y left, z up. Main axis along +y at (0, *, AXIS_Z). Plane order outward (world y):
  splitter 170..182 | arm 182.5..186.5 | housing inner wall 187..193 (behind x 22), conrod 187..193 (in front) |
  crank arm 193.6..199.6 | bevel z30 200.3..210 | wheel 206.2..222.2, worm axis 214.2 | outer wall 223..229.
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
from cad.lib import clips, fits, fasteners                     # noqa: E402
import launch_kin as K                                         # noqa: E402
import launch_gears as G                                       # noqa: E402
import launch_switch as SW                                     # noqa: E402

# ------------------------------------------------------------------------------------------------ numbers
FIT = 0.35            # diametral clearance pin in bore (gives real contact for the floating check)
M_S = 1.5             # splitter module
Z_SUN, Z_IDL, Z_OUT = 30, 18, 30
FACE_S = 12.0         # splitter face (sun, idler, output): 12 for Lewis SF >= 1.5 at the trip (LEWIS below)
M_B, Z_B1, Z_B2, FACE_B = 1.5, 30, 15, 8.0   # bevel pair (z30 on the countershaft, z15 on the worm shaft)
WORM_L = 32.0
BEV_PLAY = 0.2       # z15 moved back along its axis (backlash: the library pair overlaps up to ~1 mm3)
WHEEL_FACE = 16.0     # = 2/3 of the worm pd 24, the most face a worm wheel can use (LEWIS below)
R_CRANK_HUB = 16.5
SHAFT_R = 4.0         # countershaft and worm shaft d8
PIN_R = 4.0           # crank pin and lug pin d8
BRG = (22.0, 8.0, 7.0)   # 608

Y_SUN = 170.0                  # splitter inboard face (rail cheek B ends at y 168); sleeve rib under y 170..182
Y_ARM0, Y_ARM1 = 182.5, 186.5
Y_W1 = (187.0, 193.0)          # housing inner wall
Y_ROD = (187.0, 193.0)         # conrod plane
Y_CRANK = (193.6, 199.6)       # crank arm plane
Y_W2 = (223.0, 229.0)          # housing outer wall (6 thick: its inner face moved out 2 for the face 16 wheel)
# SW_RAIL_LOW: printed contact on a housing pedestal in front of the crank (screw along world +x at the crank centre
# height), pressed by a cam lobe on the crank only around the LOW dead point psi_low.
SWL_RT = 41.5                  # tip (pressed position) radius from the crank centre; free tip at SWL_RT - TRAVEL
SWL_Y = 197.8                  # switch mid plane (world y), width 8: 193.8..201.8 (crank plane, clear of the conrod)
SWL_ZS = 9.0                   # screw axis height above the switch base
SWL_SCREW = 6.0
SWL_DWELL = 4.0                # cam dwell half angle at SWL_RT (+ tip half width 2.1 deg = closed window ~ +-6 deg)
SWL_RAMP = 6.0                 # ramp from SWL_RT down to SWL_RT - 2 (beyond the leaf travel)
SWL_CAM_Y = (193.8, 199.4)
# cad-sensors' SW_RAIL_LOW bracket stands on the foot extension (x 80..SWL_EXT_X1, y SWL_Y +- 4, top at deck + 8). Its
# own deck screw (sensors.deck_holes) is an M3 x 16 from the top through the bracket foot (counterbored) and a
# clearance hole in the extension into a deck insert, in front of the bracket column (x <= 124.5).
SWL_DECK_X = 128.3
SWL_EXT_X1 = SWL_DECK_X + 6.2
# Deck fixing of the housing: the deck is on the rover long before Aim 3, so every screw goes in from the TOP:
# 4 x M3 x 10 SHCS through counterbored holes (d6.1 x 3.2, 4.8 of flange under the head, 5.2 into the insert) in foot
# flanges outside the spine and the walls, into cad-frame's M3 heat-set inserts (the frame cuts one under every
# deck_holes point; launch.deck_holes re-exports it). Each screw has a free vertical path for a hex key.
FOOT_T = 8.0                   # foot and flanges, z deck .. deck + 8
DECK_FLANGES = ((-42.0, -29.9, 187.0, 229.0),   # behind the spine: both rear screws
                (85.9, 98.0, 217.0, 229.0),     # in front of the outer wall (wheel x <= 80, SW_RAIL_LOW lever x >= 98)
                (33.0, 47.0, 174.0, 187.1))     # inboard tab between the inner wall and the conrod
DECK_SCREWS = ((-36.0, 193.0), (-36.0, 223.0), (92.0, 223.0), (40.0, 180.0))
# Main shaft spigot d12 (cad-drum) runs to y 198.5, its AS5600 magnet cap is y 195..201.5 (magnet face 201.5)
SPIGOT_CLR = 12.5
SPIGOT_END = 198.5
Y_AX_END = 194.5               # housing ends here in the axis region (cap from 195)
AS_CHIP_Y = 202.6              # chip face 1.1 beyond the magnet face
AS_BOARD_Y = 203.6             # board inboard face (1.6 thick)
AS_PLATE = (205.22, 208.2)     # bracket plate behind the board
AS_HOLES_X = (-10.15, 10.15)   # 2 x M2.5, 20.3 apart (assumed)

# LEWIS (root bending at the 5 A aim trip). Assumptions: wiper torque at the trip kt 0.909 x (5 - 1.5) = 3.18 N m
# on the sleeve, no losses credited in the splitter (conservative), static (Kv 1), one tooth carries the load, Shigley
# Table 14-2 Y (20 deg full depth), all aim gears printed PPA-CF with an allowable of 40 MPa (25 MPa in
# physics/common.py is the conservative figure; SF quoted at both, 40 / 25).
#   splitter m1.5 (pitch r 22.5): Wt = 3180 / 22.5 = 141 N
#     face 8 (before):  sun z30 Y 0.359: 32.8 MPa, SF 1.22 / 0.76; idler z18 Y 0.309: 38.1 MPa, SF 1.05 / 0.66
#     face 12 (now):    sun z30: 21.9 MPa, SF 1.83 / 1.14;          idler z18: 25.4 MPa, SF 1.57 / 0.98
#   worm wheel m2 z20: worm torque 3.18 / 2 x 0.95 = 1.51 N m, thrust = 1.51 / (0.012 x (cos20 tan4.77 + mu) /
#     (cos20 - mu tan4.77)) = 352 N with mu 0.25 (printed on printed, dry). Shigley worm gear Lewis:
#     sigma = Wt / (pn Fe y), pn = pi m cos(lambda), y 0.125 (20 deg), Fe = min(face, 2/3 worm pd = 16)
#     face 12 (before): 37.5 MPa, SF 1.07 / 0.67 (spur Y 0.322: 45.5 MPa, SF 0.88)
#     face 16 (now):    28.1 MPa, SF 1.42 / 0.89 (spur Y 0.322: 34.2 MPa, SF 1.17). Fe is capped at 16 by the
#     worm pd 24 (launch_kin), so more face buys nothing; the outer wall inner face moved out 2 for it.
#   bevel m1.5 z30 / z15 face 8 (unchanged): face 8 is already the cone distance / 3 limit (25.2 / 3) and a bigger
#     module meets the housing spine (x <= -16) and the worm saddles, so the bevel stays: z15 at the large end,
#     equivalent z 16.8 (Y 0.301), 39.1 MPa (SF 1.02 / 0.64), with the bevel factor A / (A - b) 1.47: 57.4 MPa
#     (SF 0.70 / 0.44). The bevel pair stays the weak link at the trip.


def bed(solid, R=None, rz=0.0):
    """print_tf: rotate by R (4x4), then about z by rz deg, then translate so the part sits at z = 0, centred in x y."""
    M = tf.mul(tf.Rz(rz), R if R is not None else tf.I4)
    lo, hi = prim.bbox(prim.place(solid, M))
    return tf.mul(tf.T(-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]), M)


_A_CACHE = {}


def A(P):
    """All aim geometry numbers (world mm / deg), cached per params object and version (kin calls it per state)."""
    key = (id(P), getattr(P, "VERSION", None))
    if key not in _A_CACHE:
        _A_CACHE[key] = _A(P)
    return _A_CACHE[key]


def _A(P):
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
    lay = getattr(P, "LAYOUT", None) if P is not None else None
    dz = lay.get("deck_top_z") if isinstance(lay, dict) else None
    n["deck_z"] = float(dz) if dz is not None else axis_z - 165.0              # deck top (housing foot)
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


# ---- swing arm drag: printed PLA flexure pad (replaces the O-ring) ---------------------------------------------
# A curved cantilever finger lies in a recess of the arm hub bore; its root block sits in a pocket of the hub (pushed
# in axially, held by the sun face and the housing bushing face 0.5 away) and a bump at its free tip bears on the
# sleeve d20. The bump stands DRAG_INT proud of the sleeve surface as printed, so the finger tip deflects outward by
# DRAG_INT when the sleeve is in: that deflection times the finger stiffness is the preload. The printed shim behind
# the root block sets it: a shim DRAG_SHIM_ADJ thicker pushes the root block (and the bump) that much further in.
# Finger: PLA (E 3500 MPa), radial thickness t 2.0, axial width b 4.0, mean radius 11.3 over 90 deg: L = 17.7 mm.
#   strain at the root, eps = 3 t d / (2 L^2):
#     nominal d = DRAG_INT 0.45:            3 x 2.0 x 0.45 / (2 x 17.7^2) = 0.43 %
#     thickest shim d = 0.45 + 0.25 = 0.70: 3 x 2.0 x 0.70 / (2 x 17.7^2) = 0.67 %  (limit 0.8 %)
#   tip force F = 3 E I d / L^3, I = b t^3 / 12 = 2.67 mm4: 2.3 N nominal (3.5 N thickest shim); drag torque on the
#   sleeve mu F r = 0.3 x 2.3 x 10 = 7 N mm (about twice what swings the arm and idler up against gravity).
#   The gap behind the finger (1.2) is larger than the deflection, so the finger never bottoms in its recess.
DRAG_T, DRAG_W_RB = 2.0, 10.3          # finger radial thickness, finger inner radius (0.3 clear of the sleeve)
DRAG_TIP, DRAG_ROOT, DRAG_END = 130.0, 220.2, 250.0   # arm local deg: finger tip, root block from .. to
DRAG_INT = 0.45                        # bump interference with the sleeve d20 (= working tip deflection)
DRAG_SHIM_ADJ = 0.25                   # thickest shim over nominal (more preload)
DRAG_BLOCK_R = 16.0                    # root block outer radius; shim DRAG_BLOCK_R + 0.05 .. DRAG_POCKET_R - 0.05
DRAG_POCKET_R = 18.6                    # hub pocket outer radius (shim 2.5 thick, hub wall 1.9)
ARM_HUB_R = 20.5


def _sector(r0, r1, a0, a1, h, z=0.0, seg=None):
    """Annular sector r0..r1 between a0 and a1 deg (counterclockwise), extruded along +z from z by h."""
    k = seg or max(4, int(abs(a1 - a0) / 2.0))
    aa = np.linspace(math.radians(a0), math.radians(a1), k + 1)
    pts = [(r1 * math.cos(a), r1 * math.sin(a)) for a in aa] + [(r0 * math.cos(a), r0 * math.sin(a)) for a in aa[::-1]]
    return prim.extrude(prim.poly_cs(pts), h, z=z)


def drag_pad_solid(P):
    """Printed PLA flexure pad in the arm frame (axis +z, z 0..arm thickness): finger, root block and tip bump as one
    outline (one shell, clean STL)."""
    t = Y_ARM1 - Y_ARM0
    rf, ro, rb = DRAG_W_RB, DRAG_W_RB + DRAG_T, 10.0 - DRAG_INT                # finger in / out, bump crest radius

    def P2(r, a):
        return (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))

    def arc(r, a0, a1):
        k = max(4, int(abs(a1 - a0) / 2.0))
        return [P2(r, a) for a in np.linspace(a0, a1, k + 1)]

    tip = DRAG_TIP
    pts = arc(ro, tip, DRAG_ROOT) + arc(DRAG_BLOCK_R, DRAG_ROOT, DRAG_END) + arc(rf, DRAG_END, tip + 8.0)[:-1]
    pts += [P2(rf, tip + 8.0), P2(rb, tip + 5.5), P2(rb, tip + 1.5), P2(rf, tip)]
    return prim.extrude(prim.poly_cs(pts), t)


def drag_shim_solid(P):
    """Printed preload shim (nominal) behind the pad root block, arm frame. Print one DRAG_SHIM_ADJ thicker for more
    preload (it then pushes the root block in by that much)."""
    return _sector(DRAG_BLOCK_R + 0.05, DRAG_POCKET_R - 0.05, DRAG_ROOT + 0.2, DRAG_END - 0.2, Y_ARM1 - Y_ARM0)


def arm_solid(P):
    """Swing arm: hub on the sleeve (recess and pocket for the printed flexure pad and its shim), bar to the idler
    pin at alpha 0 (local +x), pin towards -z (inboard) through the idler, printed C clip groove at the end."""
    n = A(P)
    t = Y_ARM1 - Y_ARM0
    r = n["cd_si"]
    g = clips.groove_dims(2 * PIN_R)
    z_end = -(Y_ARM0 - Y_SUN) - clips.PIN_PLAY - g["width"] - g["land"]       # pin end (arm local z)
    s = prim.cyl(ARM_HUB_R, t) + prim.box(10.0, r, -6.0, 6.0, 0, t) + prim.cyl(7.0, t, x=r)
    s = s + prim.cyl(PIN_R, t - z_end, x=r, z=z_end)
    s = s - prim.cyl(10.0 + FIT / 2, t + 2, z=-1)
    s = s - _sector(9.0, DRAG_W_RB + DRAG_T + 1.2, DRAG_TIP - 6.0, DRAG_ROOT - 0.2, t + 2, z=-1)   # finger recess
    s = s - _sector(9.0, DRAG_POCKET_R, DRAG_ROOT - 0.2, DRAG_END + 0.2, t + 2, z=-1)             # block + shim
    zg = -(Y_ARM0 - Y_SUN) - clips.PIN_PLAY - g["width"]
    s = s - clips.clip_groove_cutter(2 * PIN_R, z=zg).translate((r, 0, 0))
    return s


def pin_clip_solid(P):
    """Printed C clip for the d8 printed pins (idler pin, crank pin, lug pin): cad.lib.clips v10 style."""
    return clips.c_clip(2 * PIN_R)


def _clip_y0(face_y, inboard=True):
    """World y of the clip's low face for a clip seated in the groove next to a part face at face_y. inboard: the
    clip sits on the -y side of that face (groove below it), else on the +y side."""
    g = clips.groove_dims(2 * PIN_R)
    if inboard:
        return face_y - clips.PIN_PLAY - g["width"] + clips.CLIP_AX_CLEAR / 2
    return face_y + clips.PIN_PLAY + clips.CLIP_AX_CLEAR / 2


def idler_solid(P):
    return G.spur(M_S, Z_IDL, FACE_S) - prim.cyl(PIN_R + FIT / 2, FACE_S + 2, z=-1)


def output_solid(P):
    """Output gear z30 + collar + d8 countershaft (local z = world y - Y_SUN), D flat for the bevel."""
    z_end = Y_W2[1] + 0.5 - Y_SUN
    s = G.spur(M_S, Z_OUT, FACE_S) + prim.cyl(8.0, Y_W1[0] - 0.1 - Y_SUN - FACE_S + 0.01, z=FACE_S - 0.01)
    s = s + prim.cyl(SHAFT_R, z_end - 1, z=1)
    s = s - prim.box(3.2, 5, -5, 5, 199.0 - Y_SUN, z_end + 1)                # D flat from the bevel to the end (bevel
    #                                                                            slides on from the outboard end)
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
    """Worm wheel z20 (lib frame: axis +z, face 0..WHEEL_FACE, z = 0 is the outboard face) + hub + crank arm + crank
    pin (printed C clip groove next to the conrod). Pocket for two 608 from the outboard face. Built pre-rotated by
    -mesh phase (place adds it back)."""
    n = A(P)
    yo = n["y_wheel_out"]
    zl = lambda y: yo - y                                                       # world y -> lib z
    g = G.worm_wheel(n["m_w"], n["z_w"], WHEEL_FACE, n["pd_w"], starts=n["starts"])
    s = g + prim.cyl(R_CRANK_HUB, zl(Y_CRANK[0]) - WHEEL_FACE + 0.01, z=WHEEL_FACE - 0.01)
    mp = G.mesh_phase(n["z_w"], 90.0)
    arm = prim.box(0, n["rc"], -7.0, 7.0, zl(Y_CRANK[1]), zl(Y_CRANK[0])) + \
        prim.cyl(8.0, Y_CRANK[1] - Y_CRANK[0], x=n["rc"], z=zl(Y_CRANK[1]))
    g = clips.groove_dims(2 * PIN_R)
    y_g1 = Y_ROD[0] - clips.PIN_PLAY                                            # groove outboard edge (world y)
    y_end = y_g1 - g["width"] - g["land"]                                       # crank pin end
    arm = arm + prim.cyl(PIN_R, Y_CRANK[0] - y_end + 0.01, x=n["rc"], z=zl(Y_CRANK[0]) - 0.01)
    arm = arm - clips.clip_groove_cutter(2 * PIN_R, z=zl(y_g1)).translate((n["rc"], 0, 0))
    s = s + prim.place(arm, tf.Rz(-mp))
    s = s + prim.place(swl_cam_joint(P), tf.inv(_wheel_place(P)))              # SW_RAIL_LOW cam lobe
    s = s - prim.cyl(BRG[0] / 2 + 0.05, zl(_brg_y(P)[1] - 0.2) + 1, z=-1)       # 608 pocket (two, from outboard)
    s = s - prim.cyl(5.0, zl(BOLT_Y0 - 1.0) - zl(_brg_y(P)[1] - 0.2) + 0.01,
                     z=zl(_brg_y(P)[1] - 0.2) - 0.01)                           # bolt end
    return s


# M8 wheel axle: M8 x 25 hex bolt (ISO 4017) from outboard, head on a boss of the outer wall (y Y_W2[1] .. BOLT_HEAD_Y),
# M8 nut (13 AF x 6.5) in a hex pocket (13.2 AF x 6.6 deep) open to the outer wall inner face: the bolt clamps the
# wall between head and nut, and the nut's inboard face is the shoulder for the outboard 608 inner race (the race
# face stands 0.08 proud of the wall face, in a relief r 11.6 x 0.7 so the outer race runs free). The shank carries
# both 608 and ends 1.1 past the inboard one, in a d10 pocket of the crank hub.
BOLT_L = 25.0
BOLT_HEAD_Y = Y_W2[1] + 4.0                     # outer face of the bolt boss (head seat), y 233
BOLT_Y0 = BOLT_HEAD_Y - BOLT_L                  # bolt end (world y), 208
NUT_Y = (Y_W2[0] + 0.09, Y_W2[0] + 6.59)        # nut in its pocket (pocket Y_W2[0] - 0.1 .. Y_W2[0] + 6.6)


def _brg_y(P):
    """World y of the inboard faces of the outboard and inboard 608 (outboard one on the nut face, 0.02 clear)."""
    y1 = NUT_Y[0] - 0.02 - BRG[2]
    return y1, y1 - BRG[2]


def swl_cam_joint(P):
    """Cam lobe for SW_RAIL_LOW in the crank joint frame (world oriented at crank value 0, origin on the crank centre,
    world y). Centred on psi_sw - psi_low + 0 (pin at psi 0 for value 0), so it faces the switch at psi = psi_low."""
    psi_lo = K.pitch_table(P)["psi_low"]
    phc = SWL_PSI - psi_lo
    rt = SWL_RT
    pts = []
    half = SWL_DWELL + SWL_RAMP + 2.0
    for d in np.linspace(-half, half, 121):
        ad = abs(d)
        if ad <= SWL_DWELL:
            r = rt
        elif ad <= SWL_DWELL + SWL_RAMP:
            r = rt - 2.0 * (ad - SWL_DWELL) / SWL_RAMP
        else:
            r = rt - 2.0 - 3.0 * (ad - SWL_DWELL - SWL_RAMP) / 2.0
        a = math.radians(phc + d)
        pts.append((r * math.cos(a), -r * math.sin(a)))                      # 2D (x, -z)
    for d in (half, -half):
        a = math.radians(phc + d)
        pts.append((14.0 * math.cos(a), -14.0 * math.sin(a)))
    cs = prim.poly_cs(pts)
    y0, y1 = SWL_CAM_Y
    return prim.place(prim.extrude(cs, y1 - y0), tf.mul(tf.T(0, y0, 0), tf.Rx(-90)))


SWL_PSI = 0.0                  # switch direction from the crank centre (psi sense): straight forward (+x)


def swl_frame(P):
    """SW_RAIL_LOW local -> world (local x = world +x towards the switch, z up)."""
    n = A(P)
    tip_local = SW.LEAF_T - SWL_SCREW
    a = math.radians(SWL_PSI)
    tx = n["crank_x"] + SWL_RT * math.cos(a) - tip_local
    tz = n["crank_z"] + SWL_RT * math.sin(a) - SWL_ZS
    return tf.T(tx, SWL_Y, tz)


def swl_body(P):
    return SW.body(SWL_ZS, SWL_SCREW, -3.0, width=8.0, top=3.5)


def conrod_solid(P):
    """Conrod (PPA-CF) in its joint frame: crank pin at the origin, lug pin at (+rod, 0), plane y Y_ROD. Held on both
    pins by printed C clips (crank pin: inboard of the rod; lug pin: outboard)."""
    n = A(P)
    L = n["rod"]
    y0, y1 = Y_ROD
    s = prim.box(0, L, y0, y1, -5.5, 5.5) + prim.cylY(8.5, y0, y1) + prim.cylY(8.5, y0, y1, x=L)
    s = s - prim.cylY(PIN_R + FIT / 2, y0 - 1, y1 + 1) - prim.cylY(PIN_R + FIT / 2, y0 - 1, y1 + 1, x=L)
    return s


def lug_solid(P):
    """Lug bracket (PPA-CF) in the rail frame (u, y_rel, w): bolted (2 x M3) to the rail wall outer face y_rel 28 at
    u 100..130, w 32..48 (below the springs), outer plate y_rel 40.5..46.5 down to the pin at (rl, w 0); the pin
    carries a printed C clip outboard of the conrod."""
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
    g = clips.groove_dims(2 * PIN_R)
    yg = Y_ROD[1] + clips.PIN_PLAY - ln["Y0"]                                   # printed C clip groove (rail y)
    s = s + prim.cylY(PIN_R, yo - 0.01, yg + g["width"] + g["land"], x=u, z=0.0)
    s = s - prim.place(clips.clip_groove_cutter(2 * PIN_R), tf.mul(tf.T(u, yg, 0.0), tf.Rx(-90)))
    for uu in (107.0, 123.0):
        s = s - prim.cylY(1.7, yw - 1, yo + 1, x=uu, z=41.0)
        s = s - prim.cylY(3.2, yw + 6.0, yo + 1, x=uu, z=41.0)                  # screw head / driver access
    return s


def housing_solid(P):
    """Aim housing (world frame), standing on the deck (z = deck_z). Literal heights below are above the deck."""
    n = A(P)
    cx, cz = n["crank_x"], n["crank_z"]
    sx, sz = n["cs_x"], n["cs_z"]
    az = n["axis_z"]
    yw = n["y_w"]
    d0 = n["deck_z"]
    s = []
    a, b = Y_W1
    s.append(prim.cylY(18.0, a, Y_AX_END, 0.0, az))                             # sleeve bushing + thrust face boss
    s.append(prim.box(-30, 14, a, b, d0, az))
    s.append(prim.cylY(10.0, a, b, sx, sz))
    s.append(prim.box(0, sx, a, b, sz - 10, sz + 10))
    a, b = Y_W2
    s.append(prim.box(-30, 86, a, b, d0, d0 + 80))
    s.append(prim.box(-30, sx, a, b, d0 + 80, sz + 10))
    s.append(prim.cylY(10.0, a, b, sx, sz))
    s.append(prim.cylY(12.0, b - 0.01, BOLT_HEAD_Y, cx, cz))                     # M8 bolt head boss (outboard)
    s.append(prim.box(-30, 86, Y_W1[0], Y_W2[1], d0, d0 + FOOT_T))              # foot
    for x0, x1, y0, y1 in DECK_FLANGES:                                          # deck screw flanges (top access)
        s.append(prim.box(x0, x1, y0, y1, d0, d0 + FOOT_T))
    # (SW_RAIL_LOW pedestal tower removed: cad-sensors' bracket stands on the foot extension below)
    s.append(prim.box(80, SWL_EXT_X1, SWL_Y - 4.0, SWL_Y + 4.0, d0, d0 + FOOT_T))
    s.append(prim.box(-30, -16, Y_W1[0], Y_W2[1], d0, d0 + 140))                # spine
    # (AS5600 arms and plate removed: cad-sensors carrier screws to the spine +x face)
    for x0, x1 in ((35.0, 41.5), (74.5, 80.5)):                                  # worm saddles (lower halves)
        s.append(prim.box(x0, x1, yw - 7.2, Y_W2[1], d0 + 80.3, sz))
        s.append(prim.box(x0, x1, Y_W2[0], Y_W2[1], d0 + 79.0, d0 + 81.0))       # join to the outer wall
    # swing arm stop posts: swung out (the arm's leading face at alpha_d) and engaged (its trailing face at alpha_e:
    # the backward tooth forces pull the arm on towards the output, this post holds the idler at cd_io)
    for px, py in (_stop_post_xy(P), _stop_post_e_xy(P)):
        s.append(prim.cylY(4.0, Y_ARM0 + 0.1, Y_W1[0] + 0.01, px, az - py))
    h = prim.union(s)
    h = h - prim.cylY(10.0 + FIT / 2, Y_W1[0] - 1, Y_W1[1], 0.0, az)          # sleeve bushing, thrust face y 193
    h = h - prim.cylY(SPIGOT_CLR / 2, Y_W1[0] - 1, SPIGOT_END + 1, 0.0, az)     # main shaft spigot clearance
    h = h - prim.cylY(SHAFT_R + FIT / 2, Y_W1[0] - 1, Y_W2[1] + 1, sx, sz)
    h = h - prim.cylY(4.1, Y_W2[0] - 1, BOLT_HEAD_Y + 1, cx, cz)              # M8 bolt
    h = h - prim.cylY(11.6, Y_W2[0] - 1, Y_W2[0] + 0.6, cx, cz)                 # relief: 608 outer race runs free
    h = h - prim.place(prim.cyl(13.2 / math.sqrt(3), 6.7, seg=6),
                       tf.mul(tf.T(cx, Y_W2[0] - 0.1, cz), tf.Rx(-90)))         # M8 nut pocket 13.2 AF x 6.6 deep
    h = h - prim.cylX(SHAFT_R + FIT / 2, 30, 90, yw, sz)
    for x in (38.25, 77.5):
        for yy in (yw - 5.0, yw + 5.0):
            h = h - prim.cyl(1.25, 14, x=x, y=yy, z=sz - 13)                   # M3 self tap for the cap, 13 deep
    for x, y in ((107.0, 197.8), (116.0, 197.8)):                                # cad-sensors SW_RAIL_LOW bracket
        h = h - prim.cyl(1.3, 8.0, x=x, y=y, z=d0 + 1.0)                        # M3 self tap d2.6, 7 deep from the top
    h = h - prim.cyl(fits.M3_CLEAR_D / 2, FOOT_T + 2.0, x=SWL_DECK_X, y=SWL_Y, z=d0 - 1.0)   # bracket deck screw
    for zb in (-33.0, -44.0):                                                    # cad-sensors AS5600 carrier column
        h = h - prim.cylX(1.3, -27.0, -15.0, y=209.1, z=zb)                     # M3 self tap d2.6, 11 deep along -x
    for x, y in deck_holes(P):                                                   # M3 x 10 from the top, counterbored
        h = h - fasteners.m3_counterbore(FOOT_T + 1.0).translate((x, y, d0 + FOOT_T))
    return h


def deck_holes(P):
    """4 x M3 deck screws of the housing (world x, y): M3 x 10 SHCS from the top through the counterbored foot flange
    holes into M3 heat-set inserts that cad-frame puts in the deck under these points (launch.deck_holes re-exports
    this for _frame_others.deck_contacts). The SW_RAIL_LOW bracket's own deck screw is cad-sensors'
    (sensors.deck_holes)."""
    return [tuple(xy) for xy in DECK_SCREWS]


def as5600_solid(P):
    """Adafruit AS5600 STEMMA QT board as a box (world oriented, centred on the main axis), chip on its inboard face."""
    n = A(P)
    az = n["axis_z"]
    b = prim.box(-12.7, 12.7, AS_BOARD_Y, AS_BOARD_Y + 1.6, az - 8.9, az + 8.9)
    b = b + prim.box(-2.0, 2.0, AS_CHIP_Y, AS_BOARD_Y + 0.01, az - 2.5, az + 2.5)
    for hx in AS_HOLES_X:
        b = b - prim.cylY(1.35, AS_BOARD_Y - 1, AS_BOARD_Y + 3, hx, az)
    return b


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
    """M8 x 25 hex bolt along +z: shank z 0..25, head on top (seat on the outer wall boss)."""
    Ls = BOLT_L
    return prim.cyl(3.95, Ls + 0.1) + prim.cyl(7.5, 5.3, z=Ls + 0.02, seg=6)


def nut_solid(P):
    """M8 nut 13 AF x 6.5 along +z (same hex orientation as its pocket)."""
    return prim.cyl(13.0 / math.sqrt(3), NUT_Y[1] - NUT_Y[0], seg=6) - prim.cyl(4.0, 10, z=-1)


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
    """Swung-out stop post centre (alpha frame X, Y) relative to the main axis: the arm's leading face at alpha_d."""
    n = A(P)
    rp, ro = 4.0, 26.0
    a = math.radians(n["alpha_d"])
    lx, ly = ro, 6.0 + rp + 0.1
    return lx * math.cos(a) - ly * math.sin(a), lx * math.sin(a) + ly * math.cos(a)


def _stop_post_e_xy(P):
    """Engaged stop post centre (alpha frame X, Y): the arm's trailing face at alpha_e, 0.1 clear, so the idler sits at
    cd_io (the 0.1 lets the arm on 0.22 deg, 0.07 off the centre distance, inside its +0.10 extra)."""
    n = A(P)
    rp, ro = 4.0, 26.0
    a = math.radians(n["alpha_e"])
    lx, ly = ro, -(6.0 + rp + 0.1)
    return lx * math.cos(a) - ly * math.sin(a), lx * math.sin(a) + ly * math.cos(a)


# ------------------------------------------------------------------------------------------------ module hooks
def parts_aim(P):
    n = A(P)
    sun, arm, idl, out, b1 = sun_solid(P), arm_solid(P), idler_solid(P), output_solid(P), bevel1_solid(P)
    wm, wh, rod, lug = worm_solid(P), wheel_solid(P), conrod_solid(P), lug_solid(P)
    hs, cap = housing_solid(P), cap_solid(P)
    bolt, brg = bolt_solid(P), brg_solid(P)
    pad, shim, clip = drag_pad_solid(P), drag_shim_solid(P), pin_clip_solid(P)
    Y0 = n["Y0"]
    f = f"face {FACE_S:g}"
    return [
        Part("launch_aim_sun", sun, "PPA-CF", joint="launch_sleeve", place=_ygear(sun, Y_SUN - Y0),
             print_tf=bed(sun), color="#2f6db5", notes=f"Splitter sun m1.5 z30 {f}, keyed on the launch sleeve"),
        Part("launch_aim_arm", arm, "PPA-CF", joint="launch_aim_arm", place=_ygear(arm, Y_ARM0),
             print_tf=bed(arm, tf.Rx(180)), color="#5a6a7a",
             notes="Swing arm on the sleeve, dragged by the printed PLA flexure pad in its hub; printed idler pin "
                   "with a printed C clip groove"),
        Part("launch_aim_drag_pad", pad, "PLA", joint="launch_aim_arm", place=_ygear(pad, Y_ARM0),
             print_tf=bed(pad), color="#e0c050",
             notes=f"Flexure drag pad: cantilever finger t {DRAG_T:g}, tip bump {DRAG_INT:g} into the sleeve d20 "
                   "(strain 0.43 %, about 2.3 N), root block pushed axially into the arm hub pocket"),
        Part("launch_aim_drag_shim", shim, "PLA", joint="launch_aim_arm", place=_ygear(shim, Y_ARM0),
             print_tf=bed(shim), color="#c0a040",
             notes=f"Preload shim behind the drag pad root block (nominal; print up to {DRAG_SHIM_ADJ:g} thicker for "
                   "more drag, strain stays under 0.7 %)"),
        Part("launch_aim_idler", idl, "PPA-CF", joint="launch_aim_idler", place=_ygear(idl, Y_SUN),
             print_tf=bed(idl), color="#3d7cc9", notes=f"Splitter idler m1.5 z18 {f} on the arm pin"),
        Part("launch_aim_idler_clip", clip, "PLA", joint="launch_aim_arm",
             place=tf.mul(_ygear(clip, Y_ARM0), tf.T(n["cd_si"], 0.0, _clip_y0(Y_SUN) - Y_ARM0)),
             print_tf=bed(clip), color="#e0c050", notes="Printed C clip on the arm pin, holds the idler"),
        Part("launch_aim_output", out, "PPA-CF", joint="launch_aim_cshaft", place=_ygear(out, Y_SUN),
             print_tf=bed(out), color="#2f6db5",
             notes=f"Splitter output m1.5 z30 {f} with its d8 countershaft (D flat)"),
        Part("launch_aim_bevel30", b1, "PPA-CF", joint="launch_aim_cshaft", place=_ygear(b1, n["y_b"]),
             print_tf=bed(b1), color="#3d7cc9", notes="Bevel m1.5 z30 on the countershaft D flat, drives the z15 2:1"),
        Part("launch_aim_worm", wm, "PPA-CF", joint="launch_aim_worm", place=_worm_place(P),
             print_tf=bed(wm), color="#2f6db5",
             notes="Worm m2 pd24 single start (lead angle 4.8 deg, self-locking) + d8 shaft + bevel z15"),
        Part("launch_aim_crank", wh, "PPA-CF", joint="launch_aim_crank", place=_wheel_place(P),
             print_tf=bed(wh), color="#3d7cc9",
             notes=f"Worm wheel m2 z20 face {WHEEL_FACE:g} = crank gear, crank arm r30 and d8 crank pin in one "
                   "part, two 608 inside"),
        Part("launch_aim_conrod", rod, "PPA-CF", joint="launch_aim_conrod", place=tf.I4,
             print_tf=bed(rod, tf.Rx(90)), color="#d97a00",
             notes="Conrod 156 between pin centres (PPA-CF), printed C clips on both pins"),
        Part("launch_aim_crankpin_clip", clip, "PLA", joint="launch_aim_conrod",
             place=tf.mul(tf.T(0.0, _clip_y0(Y_ROD[0]), 0.0), tf.Rx(-90)),
             print_tf=bed(clip), color="#e0c050", notes="Printed C clip on the crank pin, inboard of the conrod"),
        Part("launch_aim_lug", lug, "PPA-CF", joint="launch_rail", place=tf.I4,
             print_tf=bed(lug, tf.Rx(90)), color="#d97a00",
             notes="Lug bracket (PPA-CF), 2 x M3 into the rail wall, d8 lug pin at (u 115.2, w 0)"),
        Part("launch_aim_lugpin_clip", clip, "PLA", joint="launch_rail",
             place=tf.mul(tf.T(n["lug_u"], _clip_y0(Y_ROD[1], inboard=False) - Y0, 0.0), tf.Rx(-90)),
             print_tf=bed(clip), color="#e0c050", notes="Printed C clip on the lug pin, outboard of the conrod"),
        Part("launch_aim_housing", hs, "PLA", joint=None, place=tf.I4, print_tf=bed(hs), color="#9aa7b4",
             notes="Aim housing on the deck (4 x M3 x 10 from the top through its counterbored foot flanges into the "
                   "deck inserts): sleeve end bushing, countershaft bushings, worm saddles, M8 wheel bolt (head on "
                   "the outer boss, nut in the inner face hex pocket), engaged and swung-out swing arm stop posts"),
        Part("launch_aim_cap", cap, "PLA", joint=None, place=tf.I4, print_tf=bed(cap, tf.Rx(180)), color="#8796a5",
             notes="Worm shaft saddle cap, 4 x M3x30 through the legs into 13 deep pilots in the saddles"),
        Part("launch_aim_bolt", bolt, "bought", joint=None,
             place=tf.mul(tf.T(n["crank_x"], BOLT_Y0, n["crank_z"]), tf.Rx(-90)), color="#777c84",
             bought_ref="M8x25 hex bolt ISO 4017",
             notes="Fixed axle of the worm wheel, from outboard into the nut, clamps the outer wall"),
        Part("launch_aim_nut", nut_solid(P), "bought", joint=None,
             place=tf.mul(tf.T(n["crank_x"], NUT_Y[0], n["crank_z"]), tf.Rx(-90)), color="#777c84",
             bought_ref="M8 hex nut ISO 4032", notes="Captive in the outer wall hex pocket, shoulder for the 608"),
        Part("launch_aim_brg_1", brg, "bought", joint=None,
             place=tf.mul(tf.T(n["crank_x"], _brg_y(P)[0], n["crank_z"]), tf.Rx(-90)), color="#b0b5bb",
             bought_ref="608ZZ 8x22x7", notes="Worm wheel bearing (outboard)"),
        Part("launch_aim_brg_2", brg, "bought", joint=None,
             place=tf.mul(tf.T(n["crank_x"], _brg_y(P)[1], n["crank_z"]), tf.Rx(-90)), color="#b0b5bb",
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
    if S.get("aim_psi_from_low") is not None:                  # any crank angle (falling stroke too)
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
    if S.get("aim_split") == "out":
        th = n["alpha_d"]
        s = float(S.get("wiper_deg") or 0.0)
        I = G.mesh_phase(Z_IDL, th + 180.0) - (s - th) * Z_SUN / Z_IDL
    else:
        th = n["alpha_e"]
        be = n["beta"]
        I = be + (G.mesh_phase(Z_OUT, be + 180.0) - o) * k_io
        s = th + (G.mesh_phase(Z_IDL, th + 180.0) - I) * k_si
    out["launch_sleeve"] = s                                   # split out: the sleeve follows wiper_deg
    out["launch_aim_arm"] = th
    out["launch_aim_idler"] = I - th
    return out


def allowed_aim(P):
    return [Allow("launch_aim_drag_pad", "launch_sleeve",
                  f"flexure pad tip bump bears on the sleeve d20 ({DRAG_INT:g} preload interference)")]


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
    for d in np.arange(-10.0, 10.01, 2.0):                                     # SW_RAIL_LOW cam passing the tip
        pv = float(K.pitch_of_psi(P, t["psi_low"] - d, +1))
        S.append(dict(aim_deg=pv, aim_psi_from_low=float(d), cock=0.0, phase="aim_sw_low"))
    for p, wd in ((6.02, 0.0), (20.0, 37.0), (35.0, 101.0), (47.0, 233.0)):
        S.append(dict(aim_deg=p, cock=0.5, wiper_deg=wd, aim_split="out", phase="aim_split_out"))
    return S
