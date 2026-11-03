"""GrappStar base, reel system (worker cad-reel): storage reel + slip clutch + capstan (backstopped on sprag #2) +
Harry's v7 wrap cup + the take-up bus (D15) with a forward-only swing idler. Module contract: base/cad/README.md.

Reel frame R (reel_NOTES.md section 2): origin on the capstan axis at the front face of the reel wall, +Z along the
capstan axis toward the cup, +X forward (launch direction), +Y up. params.LAYOUT["reel_frame_R"] places it in the
world (R +X = world +X, R +Y = world +Z, R +Z = world -Y). R is a pure translation of Harry's v7 world frame, so
the v7 line rules (ring, groove, guide and exit eye positions) carry over unchanged.

Senses about R +Z: capstan wind-in +, storage reel take-up +, bus gear under a FORWARD wiper +, cup WRAP -,
cup UNWRAP +. Wiper FORWARD turns the main shaft - about R +Z.
Every shared number comes from params (LAYOUT, REEL, DRUM); local geometry is named here; physics in reel_calc.py.
"""
from __future__ import annotations

import math

import numpy as np
from manifold3d import Manifold

from cad.lib import Part, Joint, Allow, Clearance, prim, tf, fits, gears, bearings, shafts, clips, line, switch
from cad.parts import reel_calc, reel_v7 as v7


# ------------------------------------------------------------------------------------------------ spec numbers
def _spec(P):
    """Shared numbers from params v1.0 with the v1.0 values as fallbacks (for the stub)."""
    lay = (getattr(P, "LAYOUT", None) if P is not None else None) or {}
    reel = (getattr(P, "REEL", None) if P is not None else None) or {}
    drum = (getattr(P, "DRUM", None) if P is not None else None) or {}
    fr = lay.get("reel_frame_R", {})
    main = lay.get("axes", {}).get("main", {})
    org = tuple(fr.get("origin", (-18.661, 33.5, -69.643)))
    deck = lay.get("deck_top_z", -165.0)
    bus = drum.get("bus", {})
    ro = drum.get("reel_output", {})
    capg = reel.get("capstan", {}).get("gear", {})
    st = reel.get("storage", {})
    y0 = org[1]
    mo = main.get("origin", (0.0, 0.0, 0.0))
    return dict(
        origin=org, axes=(tuple(fr.get("x_axis", (1, 0, 0))), tuple(fr.get("y_axis", (0, 0, 1))),
                          tuple(fr.get("z_axis", (0, -1, 0)))),
        h_axis=org[2] - deck, main_xy=(mo[0] - org[0], mo[2] - org[2]),
        bus_m=bus.get("module", 1.5), bus_z=tuple(sorted(y0 - y for y in bus.get("plane_y", (40.0, 48.0)))),
        reel_m=ro.get("module", 2.0), reel_teeth=ro.get("teeth", 50),
        reel_z=tuple(sorted(y0 - y for y in ro.get("plane_y", (50.5, 62.5)))),
        cap_m=capg.get("module", 2.0), cap_teeth=capg.get("teeth", 22), cap_face=capg.get("face", 12.0),
        wraps=reel.get("wraps", 7), core_d=st.get("core_d", 50.0), flange_d=st.get("flange_d", 66.0),
        width=st.get("width", 22.0), cup_m=reel.get("cup_gear", {}).get("module", 1.5),
        cup_teeth=reel.get("cup_gear", {}).get("teeth", 24), overdrive=bus.get("overdrive_total", 2.5),
    )


try:                                       # module constants follow the spec at import time
    from tools.params import load_params as _lp
    SP = _spec(_lp())
except Exception:                          # pragma: no cover
    SP = _spec(None)

# ------------------------------------------------------------------------------------------------ layout (R frame)
WALL_T = 6.0                  # wall Z -6 .. 0 (the bus plane starts at -6.5 behind it)
FOOT_T = 5.0
H_AXIS = SP["h_axis"]         # capstan axis above the deck top (95.4 in v1.0)
Y_DECK = -H_AXIS
X_WALL = (-134.0, 30.0)       # D44: 6 mm longer on the left (free space, checked against frame and drum)
Y_WALL_TOP = 40.0             # keeps 29 mm from the main shaft coupler (main axis at R (18.7, 69.6))
Z_FOOT = (-40.0, 42.0)

WIDTH = SP["width"]           # storage reel line zone
CORE_R = SP["core_d"] / 2
Z_SPOOL = 12.0                # reel rear face (rear 608 mouth); room for the rocker to bend 4 mm in front of it
FL_T = 1.2                    # rear flange thickness (thin: the flanges carry most of the reel's inertia)
FL_F = 1.6                    # front flange disc thickness (it takes the clutch squeeze)
Z_GUIDE = Z_SPOOL + FL_T + WIDTH / 2          # guide eye plane = middle of the line zone (19)
Z_DRUM = Z_GUIDE + 1.0        # drum flange back face (v7: guide 1 mm behind it)
Z_BLOCK_GAP = 1.0             # collar flat between the sprag inner race and the drum
Z_BLOCK = Z_DRUM - 1.0        # sprag hub front face = seat mouth (collar flat 1 mm, drum flange 1 mm clear)
Z_CUP = Z_DRUM + v7.FLANGE_H + v7.DL - 7.0      # cup rim plane (cup skirt 7 mm over the drum, v7)
GUIDE = np.array((-45.0, v7.CUP_OUT + 2.0, Z_GUIDE))            # guide eye centre (v7)
EXIT = np.array((70.0, v7.ER, Z_CUP + v7.EZP + 31.5))           # exit eye centre, 31.5 past the ring (v7)

X_REEL = -94.0                # storage reel axis (Y 0); D44: moved 6 left for the r 40 flange (guide post clear)
SPOOL_LEN = FL_T + WIDTH + FL_F                 # rear face .. front flange disc face
ANNULUS = (20.0, 30.0, 0.3)   # clutch friction annulus on the front flange disc: r_in, r_out, raised (r_eff 25)
Z_D = Z_SPOOL + SPOOL_LEN + ANNULUS[2]          # clutch disk D (head of S) rear face
D_T, D_R = 3.0, 31.0
Z_T = 7.0                     # thrust ring T rear face (the rocker's two M3 trim screw tips press it)
TRIM_R = 10.0                 # trim screws at X +-10 from the reel axis
SHROUD_R = (43.5, 45.5)       # shroud shell round the storage reel (flange r 40, D44)
SHROUD_DEG = (140.0, 318.0)   # covered arc (the line leaves at the top, the guide post stands at the right)

ROCKER_PIVOT = np.array((X_REEL, 37.0, 3.0))
ROCKER_Z = (0.5, 3.5)         # spring plate (PPA-CF, t 3.0, reel_calc.rocker_section)
BAR_LUG = (19.2, 25.0, 4.0)   # bar end lug for crank arm A: Z from .. to at the end face, 45 deg chamfer below
BRAKE_STROKE = 4.0            # cad-drum's track 2 cam lift (radial, base r 30, follower at L angle 180 = R -X)
# Brake input: the cam pushes a sliding bar -X (in front of the wall, under the shroud); the bar turns a bell crank
# (axis R Y) left of the reel; the crank's arm B pushes the rocker's stiff leg +Z from behind. Fail-safe LIGHT: the
# rocker's own spring pushes everything back.
CAM_R0 = 30.0
BAR_GAP = 0.2                 # bar nose to the cam base circle at LIGHT
BAR_Z = (9.2, 15.2)           # bar thickness: rides the cam at world y 18.3 .. 24.3 (cam s 18 .. 29)
BAR_H = 6.0                   # bar height (R Y)
BAR_YL = -55.0                # left run: under the shroud and under the guide post
BAR_DOG = (-30.0, -10.0)      # dogleg up to the cam height between these X
LEG_X = (-92.0, -83.0)        # rocker leg (stiff, Z 0.5 .. 6), from the spring plate down past the shroud
LEG_Y0, LEG_Z1 = -51.7, 6.0
CRANK_X, CRANK_Z = -97.8, 2.0                 # crank pivot (axis R +Y)
CRANK_Y = (-58.1, -45.7)
CRANK_LA, CRANK_RT = 20.0, 2.5                # arm A: tip circle centre above the pivot, radius (bar lug contact)
CRANK_B, CRANK_RB = (10.8, -3.05), 1.5        # arm B: tip circle centre from the pivot (leg contact), radius
CRANK_HUB_R, CRANK_BOSS_R = 4.3, 2.5
CRANK_COL_TOP, CRANK_BOSS_TOP = -58.4, -45.4  # column under the crank, pivot boss top (washer + M3 x 10 on it)
WASHER_T, WASHER_R = 1.2, 4.5
POCKET_D = 3.0                # wall front recess for arm B and the hub
BAR_GUIDES = ((-71.5, BAR_YL), (17.0, None))  # (x, y) of the two U guides (y None = right run)
GUIDE_W, GUIDE_FRONT, GUIDE_SCREW_Z = 10.0, 20.0, 17.2
POST_Y0 = -51.0               # guide post bottom (the bar passes under it)
MAIN_CLEAR_R = 37.5           # wall cut round the main axis (cad-drum's coupler reaches r 35 in the wall plane)

# gear planes behind the wall (from params)
BUS_Z = (SP["bus_z"][1] - 6.0, SP["bus_z"][1])  # my bus gear c1 (face 6) inside cad-drum's bus plane (-14.5..-6.5)
HUB1_Z = (BUS_Z[0] - 3.5, BUS_Z[0])             # c1 rear hub: the swing arm rides on it
ARM_Z = (HUB1_Z[0] + 0.3, BUS_Z[0] - 0.3)       # swing arm plate
GC_Z = SP["reel_z"]                             # capstan gear plane = cad-drum's REEL gear plane (-29..-17)
CUPGEAR_Z = (-43.0, -35.0)                      # cup ring gear plane (coordinator, world y 68.5..76.5)
M_CAP, Z_MAIN, Z_CAPG = SP["cap_m"], SP["reel_teeth"], SP["cap_teeth"]
M_BUS, Z_BUSMAIN, Z_C1, Z_IDL = SP["bus_m"], 50, 46, 30
Z_REELG = int(round(Z_BUSMAIN / SP["overdrive"]))           # 20: clutch disk = 50 / 20 = 2.5 x wiper
M_CUPG, Z_CUPG = SP["cup_m"], SP["cup_teeth"]
# Cross-owner mesh phases (numeric search against cad-drum's gears, zero overlap). With cad-drum's kin present my
# gears follow its joint values: mine (about R +Z = world -Y) = +ratio x theirs (about world +Y).
BUS_PHASE_D = 3.78           # deg, c1 = (50/46) drum_main + this
CAP_PHASE_D = 7.91           # deg, capstan = (50/22) drum_out_reel + this
GK_PHASE = 8.67              # deg, cup gear teeth in the cup frame (cup = (42/24) (drum_idler - idler at home))
BUS_PHASE = 1.37             # fallbacks without cad-drum's module (S keys only)
CAP_PHASE = 3.27
SWING_OUT_DEG = 12.0          # swing arm travel from engaged to parked (tips clear by about 2 mm)
WRAPS = SP["wraps"]
HUB1_R = 13.0

C_D, K_D, S_D = 20.0, 10.0, 8.0
C_BORE = K_D + fits.SHAFT_IN_BORE
C_Z0 = GC_Z[0] - 0.3 - clips.groove_dims(C_D)["width"] - 1.05      # C rear end
K_Z0 = CUPGEAR_Z[0] - 0.3 - clips.groove_dims(K_D)["width"] - 1.6
S_Z0 = BUS_Z[0] - 0.3 - clips.groove_dims(S_D)["width"] - 1.0

COL = dict(wall="#b9c2cc", ppa="#3a3f45", gear="#d97a00", drum="#2e9c86", cup="#7f77dd", spool="#7f77dd",
           clip="#9a94e8", brg="#8a8f96", rocker="#e0b040", post="#4a5058")


FLANGE_R_MIN = 40.0           # D44: no level wind; a mounded 20 m pack reaches r 35 .. 36


def flange_r(P=None):
    """Flange radius: the spec value, raised (physics) until 20 m of line packs with 4 mm to spare."""
    d = reel_calc.get(P)
    d["core_r"], d["width"] = CORE_R, WIDTH
    need = reel_calc.pack_r(d, d["line_len_m"]) + 4.0
    return max(SP["flange_d"] / 2, math.ceil(need), FLANGE_R_MIN)


def _d(P):
    d = reel_calc.get(P)
    d["core_r"] = CORE_R
    d["width"] = WIDTH
    d["flange_r"] = flange_r(P)
    return d


def idler_xy():
    """Swing idler centre (engaged): meshes the bus gear c1 (capstan axis) and the reel gear (X_REEL, 0), ABOVE
    the line between them so the arm swings + (the forward sense) into mesh."""
    c1 = gears.center_distance(M_BUS, Z_C1, Z_IDL)
    c2 = gears.center_distance(M_BUS, Z_IDL, Z_REELG)
    d = abs(X_REEL)
    x = (c1 ** 2 - c2 ** 2 + d ** 2) / (2 * d)
    y = math.sqrt(max(c1 ** 2 - x ** 2, 0.0))
    return np.array((-x, y))


def arm_r():
    return gears.center_distance(M_BUS, Z_C1, Z_IDL)


def arm_deg():
    ix, iy = idler_xy()
    return math.degrees(math.atan2(iy, ix))


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


def _teardrop_z(r, z0, z1, x=0.0, y=0.0):
    """Round hole along Z with a 45 deg roof (+Y) so it prints without support when the part stands on its foot."""
    cs = prim.circle_cs(r) + prim.poly_cs([(-r * 0.7071, r * 0.7071), (r * 0.7071, r * 0.7071), (0.0, r * 1.4142)])
    return prim.extrude(cs, z1 - z0, z=z0).translate((x, y, 0))


def _gusset(x0, x1, z_wall, z_end, y0, h):
    """Triangular gusset on the deck plane y0 against a wall face at z_wall, reaching z_end on the deck, height h,
    thickness x0..x1."""
    pts = [(z_wall, y0), (z_end, y0), (z_wall, y0 + h)]
    cs = prim.poly_cs(pts)                                   # (u = R z, v = R y)
    s = prim.extrude(cs, x1 - x0)                            # local frame: x = R z, y = R y, z = R x
    M = np.array([[0, 0, -1, x1], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]], float)   # proper rotation
    return prim.place(s, M)


# ------------------------------------------------------------------------------------------------ solids
COLLAR_CONE = 4.0             # 45 deg cone on the capstan collar (prints front end down), sits in the drum mouth
HUB_SPIGOT_R = 14.0
HUB_SCREW_R = 29.5
HUB_FL_R = 33.5
HUB_TOP_Y = 28.6             # flange cut flat here: cad-frame's cheek r starts at R Y 29.6 (base-v1 5356f65)
STOP_R, STOP_RAD = 2.5, 40.0
ARM_W = 16.0
STOP_OFF = 27.0               # stop pins sit this far (deg) off the idler: outside its tip circle where they cross it


def hub_screw_deg():
    """Hub screws (deg about the capstan axis), clear of the swing arm and its stops behind the wall."""
    a = arm_deg()
    band = (a - SWING_OUT_DEG - STOP_OFF - 12.0, a + STOP_OFF + 12.0)
    out = []
    for c in (10.0, 250.0, 310.0, 205.0, 280.0, 0.0):          # none near 75 deg: the main axis side
        if not (band[0] <= c <= band[1]) and len(out) < 4:
            out.append(c)
    return tuple(out)


def stop_angles():
    """Swing-arm stop pins at r STOP_RAD: (engaged stop, parked stop), deg about the capstan axis."""
    return (arm_deg() + STOP_OFF, arm_deg() - SWING_OUT_DEG - STOP_OFF)


FOOT_FRONT_X1 = -57.0         # front foot ends here (cad-frame's cheek r stands at world x > -75)
FOOT_REAR_X1 = 3.0            # rear strip ends here (cad-drum's idler bracket at R X 4.9 .. 46.9 behind the wall)
MOUNT_HOLES = [(-120.0, 14.0), (-120.0, 36.0), (-64.0, 36.0), (-64.0, 14.0), (-120.0, -32.0), (-60.0, -32.0),
               (-8.0, -32.0)]
SHROUD_SCREW_R = SHROUD_R[1] + 3.5
SHROUD_SCREW_DEG = (145.0, 240.0, 305.0)


def _wall():
    x0, x1 = X_WALL
    # foot: under the storage reel (left of cad-frame's cheek r, world x < -75) and a strip behind the wall (left of
    # cad-drum's idler bracket, which stands just behind the wall at R X 5 .. 47)
    foot = prim.union([prim.box(x0, FOOT_FRONT_X1, Y_DECK, Y_DECK + FOOT_T, Z_FOOT[0], Z_FOOT[1]),
                       prim.box(x0, FOOT_REAR_X1, Y_DECK, Y_DECK + FOOT_T, Z_FOOT[0], 0.0)])
    wall = prim.box(x0, x1, Y_DECK, Y_WALL_TOP, -WALL_T, 0.0)
    # guide eye post (v7 top: 6 mm thick with the eye), 12 x 12 column, a web back to the wall below the line
    gx, gy, gz = GUIDE
    # it hangs from the wall (cad-frame's cheek r has a 6 mm foot flange on the deck under it; the brake bar passes
    # under its bottom end)
    yb = POST_Y0
    post = prim.union([prim.box(gx - 6, gx + 6, yb, gy - 8, gz - 6, gz + 6),
                       prim.box(gx - 3, gx + 3, gy - 8.01, gy + 6, gz - 7.5, gz + 7.5),   # eye flares stay inside
                       prim.box(gx - 2.5, gx + 2.5, yb, -2.0, -0.01, gz)])
    # rocker pivot ears (M3 pin along X) and the S bushing boss through the rocker's hole
    px, py, pz = ROCKER_PIVOT
    ears = [prim.box(px + s * 5.4, px + s * 9.4, py - 6, py + 6, -0.01, 5.7) for s in (-1, 1)]
    sboss = prim.cyl(5.5, Z_T - 0.3, x=X_REEL, z=-0.01)        # S bushing 12.7 long (through the rocker's hole)
    ex, _, ez = EXIT
    # swing-arm stop pins from the wall rear face
    stops = [prim.cyl(STOP_R, -ARM_Z[0] - WALL_T + 0.01, x=STOP_RAD * math.cos(math.radians(a)),
                      y=STOP_RAD * math.sin(math.radians(a)), z=ARM_Z[0]) for a in stop_angles()]
    # gussets: front below the reel, rear below the gears
    yg = Y_DECK + FOOT_T - 0.01
    gus = []
    for x in (-126.0, -64.0):
        gus.append(_gusset(x, x + 4.0, 0.0, Z_FOOT[1] - 2.0, yg, min(40.0, -44.0 - yg)))
    for x in (-126.0, -64.0, -20.0):
        gus.append(_gusset(x, x + 4.0, -WALL_T, Z_FOOT[0] + 2.0, yg, min(40.0, -50.0 - yg)))
    body = prim.union([foot, wall, post, sboss] + ears + stops + gus)
    cut = [_teardrop_z(HUB_SPIGOT_R + 0.2, -WALL_T - 1, 1.0)]
    for a in hub_screw_deg():                      # hub screws tap into the wall from the front (blind, 5 deep)
        x, y = HUB_SCREW_R * math.cos(math.radians(a)), HUB_SCREW_R * math.sin(math.radians(a))
        cut.append(prim.cyl(fits.M3_TAP_D / 2, 5.0 + 0.01, x=x, y=y, z=-5.0))
    cut.append(prim.cyl((S_D + fits.SHAFT_IN_BORE + fits.HOLE_HORIZ_EXTRA) / 2, WALL_T + Z_T + 2, x=X_REEL,
                        z=-WALL_T - 1))
    cut += _brake_wall_cuts()
    cut.append(_eye_cut(GUIDE, guide_axis(), 6.0))
    for s_ in (-1, 1):                             # hex key access to the rocker's trim screws
        cut.append(prim.cyl(3.6, WALL_T + 2, x=X_REEL + s_ * TRIM_R, z=-WALL_T - 1))
    cut.append(prim.cylX(fits.M3_CLEAR_D / 2, px - 12, px + 12, y=py, z=pz))
    for sx in (-12.0, 12.0):                       # SW_ARM bracket screws tap into the wall rear face
        cut.append(prim.cyl(fits.M3_TAP_D / 2, 5.0, x=sx, y=_sw_bracket()[1] - 13.0, z=-WALL_T - 0.01))
    for a in SHROUD_SCREW_DEG:                     # shroud screws tap into the wall
        x = X_REEL + SHROUD_SCREW_R * math.cos(math.radians(a))
        y = SHROUD_SCREW_R * math.sin(math.radians(a))
        cut.append(prim.cyl(fits.M3_TAP_D / 2, WALL_T - 1.0, x=x, y=y, z=-WALL_T + 1.0))
    for (x, z) in MOUNT_HOLES:
        cut.append(prim.cylY(fits.M3_CLEAR_D / 2, Y_DECK - 1, Y_DECK + FOOT_T + 1, x=x, z=z))
        cut.append(prim.cylY(fits.M3_HEAD_D / 2, Y_DECK + FOOT_T - fits.M3_HEAD_H - 0.2, Y_DECK + FOOT_T + 60,
                             x=x, z=z))
    mx, my = SP["main_xy"]                         # wall top cut round cad-drum's coupler (bus gear, SW_INDEX cam)
    cut.append(prim.cyl(MAIN_CLEAR_R, WALL_T + 2, x=mx, y=my, z=-WALL_T - 1, seg=192))
    body = body - prim.union(cut)
    add_, cut2 = _brake_wall_adds()
    return prim.union([body] + add_) - prim.union(cut2)


def cam_contact(P=None):
    """R (x, y) of cad-drum's clutch cam base circle on the follower line (L angle 180 from the drum axis)."""
    sp = _spec(P) if P is not None else SP
    lay = (getattr(P, "LAYOUT", None) if P is not None else None) or {}
    do = lay.get("axes", {}).get("drum", {}).get("origin", (60.0, 0.0, -112.0))
    return do[0] - CAM_R0 - sp["origin"][0], do[2] - sp["origin"][2]


def _bar_y_right():
    return cam_contact()[1]


def _guide_ys():
    """(y_lo, y_hi) of the bar run at each guide."""
    out = []
    for gx, gy in BAR_GUIDES:
        yc = gy if gy is not None else _bar_y_right()
        out.append((gx, yc - BAR_H / 2, yc + BAR_H / 2))
    return out


GUIDE_JAW = 4.6               # lower jaw and cap thickness (R Y)
GUIDE_CLEAR = 0.4             # bar clearance in the guides (each side)


def _wedge_down(x0, x1, y_top, depth, z0=0.0):
    """45 deg support wedge under a ledge that sticks out of the wall front (prints standing on the foot)."""
    cs = prim.poly_cs([(z0, y_top), (z0 + depth, y_top), (z0, y_top - depth)])
    s = prim.extrude(cs, x1 - x0)
    M = np.array([[0, 0, -1, x1], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]], float)
    return prim.place(s, M)


def _brake_wall_cuts():
    """Recess in the wall front for the crank hub and arm B (behind the rocker leg), washer and screw head."""
    x0 = CRANK_X - CRANK_HUB_R - 0.9
    return [prim.box(x0, LEG_X[1] - 1.5, CRANK_Y[0] - 0.4, CRANK_Y[1] + 0.4, -POCKET_D, 0.01),
            prim.box(x0, CRANK_X + CRANK_HUB_R + 0.9, CRANK_Y[0] - 0.4, CRANK_BOSS_TOP + WASHER_T + 3.6, -POCKET_D,
                     0.01)]


def _brake_wall_adds():
    """Crank column and pivot boss, two U guides for the bar (lower jaw + back; the cap is a separate part).
    Returns (solids to add, holes to cut after)."""
    adds, cuts = [], []
    yf = Y_DECK + FOOT_T - 0.01
    adds.append(prim.box(CRANK_X - 3.0, CRANK_X + 3.0, yf, CRANK_COL_TOP, -0.01, CRANK_Z + 3.0))
    adds.append(prim.cylY(CRANK_BOSS_R, CRANK_COL_TOP - 0.01, CRANK_BOSS_TOP, x=CRANK_X, z=CRANK_Z))
    cuts.append(prim.cylY(fits.M3_TAP_D / 2, CRANK_BOSS_TOP - 9.0, CRANK_BOSS_TOP + 1.0, x=CRANK_X, z=CRANK_Z))
    for gx, ylo, yhi in _guide_ys():
        x0, x1 = gx - GUIDE_W / 2, gx + GUIDE_W / 2
        s0, s1 = ylo - GUIDE_CLEAR, yhi + GUIDE_CLEAR          # slot
        zb = BAR_Z[0] - GUIDE_CLEAR                            # slot back face
        adds.append(prim.box(x0, x1, s0 - GUIDE_JAW, s0, -0.01, GUIDE_FRONT))          # lower jaw
        adds.append(prim.box(x0, x1, s0 - 0.01, s1, -0.01, zb))                       # back pillar
        adds.append(_wedge_down(x0, x1, s0 - GUIDE_JAW + 0.01, GUIDE_FRONT, z0=-0.01))
        for z in (zb / 2, GUIDE_SCREW_Z):                     # cap screws: tapped into the pillar and the jaw
            cuts.append(prim.cylY(fits.M3_TAP_D / 2, s0 - GUIDE_JAW - 0.5, s1 + 0.01, x=gx, z=z))
    return adds, cuts


def _bar():
    """Brake bar (PPA-CF), R frame at LIGHT. Nose (round, r 3) on cad-drum's cam at the right end, left end face
    on the crank's arm A tip. A flat dogleg takes it from the cam height down under the shroud."""
    cx, cy = cam_contact()
    zr = BAR_Z
    xl = CRANK_X + CRANK_RT + 0.05
    xn = cx - BAR_GAP - NOSE_R
    yl, yr, h = BAR_YL, cy, BAR_H / 2
    lz0, lz1, lstep = BAR_LUG
    left = prim.box(xl + lstep, BAR_DOG[0], yl - h, yl + h, zr[0], zr[1])
    lug_cs = prim.poly_cs([(xl, lz0), (xl + lstep, zr[1] - 0.01), (xl + lstep, zr[0] + 1.0), (xl + lstep + 8.0, zr[0] + 1.0),
                           (xl + lstep + 8.0, lz1), (xl, lz1)])           # (R x, R z) profile, extruded along R Y
    lug = prim.place(prim.extrude(lug_cs, 2 * h), tf.T(0, yl + h, 0) @ tf.Rx(90))
    right = prim.box(BAR_DOG[1], xn, yr - h, yr + h, zr[0], zr[1])
    dog = Manifold.batch_hull([prim.box(BAR_DOG[0] - 4.0, BAR_DOG[0], yl - h, yl + h, zr[0], zr[1]),
                               prim.box(BAR_DOG[1], BAR_DOG[1] + 4.0, yr - h, yr + h, zr[0], zr[1])])
    nose = prim.cyl(NOSE_R, zr[1] - zr[0], x=xn, y=yr, z=zr[0], seg=96)
    return prim.union([left, lug, dog, right, nose])


def _guide_cap(gx, ylo, yhi):
    """Bar guide cap (PLA), R frame: closes the U guide over the bar; 2 x M3 x 16 down into the pillar and jaw."""
    s1 = yhi + GUIDE_CLEAR
    cap = prim.box(gx - GUIDE_W / 2, gx + GUIDE_W / 2, s1 + 0.05, s1 + 0.05 + GUIDE_JAW, 0.0, GUIDE_FRONT)
    zb = BAR_Z[0] - GUIDE_CLEAR
    holes = [prim.cylY(fits.M3_CLEAR_D / 2, s1 - 1, s1 + GUIDE_JAW + 1, x=gx, z=z) for z in (zb / 2, GUIDE_SCREW_Z)]
    return cap - prim.union(holes)


def _crank():
    """Bell crank (PPA-CF), local frame: pivot at the origin (R orientation, R Y absolute), axis +Y. One extrusion
    along Y: hub on the wall boss, arm A up (+Z) to the bar, arm B along +X in the wall recess to the rocker leg."""
    bx, bz = CRANK_B
    z0, z1 = -CRANK_Z - 0.25 - 1.95, -CRANK_Z - 0.25          # arm B strip (R Z -2.2 .. -0.25)
    cs = (prim.circle_cs(CRANK_HUB_R) + prim.poly_cs([(-2.2, 0.0), (2.2, 0.0), (1.2, 7.0), (1.2, CRANK_LA), (-1.2, CRANK_LA), (-2.2, 7.0)])
          + prim.circle_cs(CRANK_RT, 0.0, CRANK_LA)
          + prim.poly_cs([(0.0, z0), (bx, z0), (bx, z1), (0.0, z1)]) + prim.circle_cs(CRANK_RB, bx, bz))
    s = prim.extrude(cs, CRANK_Y[1] - CRANK_Y[0])            # profile (u = R x, v = R z), extruded along w
    s = prim.place(s, tf.T(0, CRANK_Y[1], 0) @ tf.Rx(90))     # (u, v, w) -> (u, -w, v): R y = y1 - w
    return s - prim.cylY(CRANK_BOSS_R + fits.HOLE_CLEAR / 2 + 0.15, CRANK_Y[0] - 1, CRANK_Y[1] + 1)


def _washer():
    """Printed washer (PPA-CF) on the crank boss top, under the M3 x 10 head; it keeps the crank on the boss."""
    return prim.tube(WASHER_R, fits.M3_CLEAR_D / 2, WASHER_T)


def _sprag_hub():
    """Sprag #2 hub (PPA-CF), R frame: CSK20PP crush-rib seat (outer key rib at +Y, mouth at the front face),
    flange on the wall front (45 deg cone back to the block so it prints face down), spigot through the wall,
    capstan tube bushing 20.3 through. 4 M3 x 8 from the front through the flange, self tapping into the wall."""
    rb = bearings.CSK20_OD / 2 + bearings.BRG_RIB_BORE / 2 + 4.0
    fl_t = 3.0
    block = prim.cyl(rb, Z_BLOCK, seg=128)
    flange = prim.revolve_pts([(0, 0), (HUB_FL_R, 0), (HUB_FL_R, fl_t), (rb, fl_t + HUB_FL_R - rb),
                               (0, fl_t + HUB_FL_R - rb)])
    spig = prim.cyl(HUB_SPIGOT_R, WALL_T + 0.01, z=-WALL_T)
    # the flange is cut back to the block on the side facing the main axis (cad-frame's motor adapter plate)
    ma = math.degrees(math.atan2(SP["main_xy"][1], SP["main_xy"][0]))
    flange = flange - prim.place(prim.box(-HUB_FL_R - 5, HUB_FL_R + 5, rb + 0.4, HUB_FL_R + 5, -1, 20), tf.Rz(ma - 90.0))
    flange = flange - prim.box(-HUB_FL_R - 5, HUB_FL_R + 5, HUB_TOP_Y, HUB_FL_R + 5, -1, 20)   # cad-frame's cheek r above
    body = prim.union([block, flange, spig])
    cut = [prim.place(bearings.seat_csk20(key_deg=90.0, below=2.0), tf.T(0, 0, Z_BLOCK)),
           prim.cyl((C_D + fits.SHAFT_IN_BORE) / 2, Z_BLOCK + 30, z=-WALL_T - 1)]
    for a in hub_screw_deg():                      # M3 x 8 from the front through the flange into the wall
        x, y = HUB_SCREW_R * math.cos(math.radians(a)), HUB_SCREW_R * math.sin(math.radians(a))
        cut.append(prim.cyl(fits.M3_CLEAR_D / 2, 20.0, x=x, y=y, z=-1.0))
        cut.append(prim.cyl(fits.M3_HEAD_D / 2, 30.0, x=x, y=y, z=3.0))
    return body - prim.union(cut)


def _c_tube():
    """Capstan tube C (PPA-CF), local frame = R. D flat for the capstan gear, clip groove behind it, round through
    the bus gear and the hub bushing, key rib from the sprag rear face to the front, collar on the sprag inner race,
    10.3 bore for K."""
    z0, z1 = C_Z0, c_front()
    body = shafts.keyed_shaft(z1 - z0, d=C_D, key_w=shafts.KEY20_W, key_h=shafts.KEY20_H,
                              key_z=(Z_BLOCK - bearings.CSK20_W - z0, z1 - z0 - 0.8), chamfer=0.8)
    body = body.translate((0, 0, z0))
    collar = prim.union([prim.cyl(14.0, 1.0, z=Z_BLOCK), prim.cyl(14.0, COLLAR_CONE, z=Z_BLOCK + 0.99, r2=C_D / 2)])
    body = prim.union([body, collar])
    g = clips.groove_dims(C_D)
    flat = prim.box(C_D / 2 - 2.0, C_D, -C_D, C_D, z0 - 1.0, GC_Z[1] + 0.6)    # runs out the rear end
    groove = clips.clip_groove_cutter(C_D, z=GC_Z[0] - 0.3 - g["width"])
    bore = prim.cyl(C_BORE / 2, z1 - z0 + 2, z=z0 - 1)
    return body - prim.union([flat, groove, bore])


def _k_shaft():
    """Cup shaft K (PPA-CF): 10 mm, D flat (+X) under the cup's rear half and under the cup gear, clip groove behind
    the cup gear, v7 front end (4.5 neck + chamfered 5.6 head) for Harry's C-clip in front of the cup."""
    z0, zf = K_Z0, k_front()
    body = prim.cyl(K_D / 2, zf - z0, z=z0)
    body = prim.union([body, prim.cyl(2.25, 2.6 + 0.01, z=zf - 0.01), prim.cyl(2.8, 1.5, z=zf + 2.6, r2=2.3)])
    flat_cup = prim.box(K_D / 2 - 1.2, K_D, -K_D, K_D, Z_CUP - 0.5, zf + 0.01)        # runs out at the cup face
    flat_g = prim.box(K_D / 2 - 1.2, K_D, -K_D, K_D, z0 - 1, C_Z0 + 1.0)              # covers the cup gear hub
    g = clips.groove_dims(K_D)
    groove = clips.clip_groove_cutter(K_D, z=CUPGEAR_Z[0] - 0.3 - g["width"])
    return body - prim.union([flat_cup, flat_g, groove])


def _s_shaft():
    """Reel shaft S (PPA-CF): 8 mm, D flat (+X) under the reel gear, clip groove behind it, head = clutch disk D
    (r 20 x 3) at the front."""
    body = prim.cyl(S_D / 2, Z_D - S_Z0 + 0.01, z=S_Z0)
    head = prim.cyl(D_R, D_T, z=Z_D)
    body = prim.union([body, head])
    flat = prim.box(S_D / 2 - 1.0, S_D, -S_D, S_D, S_Z0 - 1, BUS_Z[1] + 0.5)
    g = clips.groove_dims(S_D)
    groove = clips.clip_groove_cutter(S_D, z=BUS_Z[0] - 0.3 - g["width"])
    return body - prim.union([flat, groove])


def _gear(m, z, face, bore_d, flat, hub=None, shift=0.0, phase=0.0):
    g = gears.spur_gear(m, z, face, hub=hub, shift=shift)
    if phase:
        g = prim.place(g, tf.Rz(phase))
    rb = (bore_d + fits.SHAFT_IN_BORE) / 2
    cut = prim.cyl(rb, 100, z=-10) - prim.box(rb - flat, rb + 5, -rb - 5, rb + 5, -20, 120)
    return g - cut


def _spool_body(rf):
    """Storage reel body (PLA), local frame: axis Z, rear face z = 0. Rear flange flat on the bed, core r CORE_R up
    to the front flange disc, 608 seats at both ends, 3 drive lugs for the disc, two d 2 tie holes."""
    rc = CORE_R
    L = FL_T + WIDTH
    prof = [(4.6, 0), (rf - 0.6, 0), (rf, 0.6), (rf, FL_T), (rc + 1.5, FL_T), (rc, FL_T + 1.5), (rc, L), (4.6, L)]
    body = prim.revolve_pts(prof, seg=180)
    lugs = [prim.place(prim.box(rc - 4.0, rc - 1.0, -3.0, 3.0, L - 0.01, L + 1.5), tf.Rz(a))
            for a in (0.0, 120.0, 240.0)]
    body = prim.union([body] + lugs)
    s_rear = prim.place(bearings.seat_608(bottom="lip", lip_d=14.0, below=0.0), tf.Rx(180))
    s_front = prim.place(bearings.seat_608(bottom="lip", lip_d=14.0, below=0.0), tf.T(0, 0, L + 0.02))   # 0.02: avoids a
    # coincident-vertex seam that breaks the STL after vertex merge
    # open pocket under the front disc (lighter reel, prints with no support); the line end goes in through a radial
    # hole near the rear flange and is knotted inside the pocket before the disc goes on
    pocket = prim.tube(20.4, 13.0, L - 3.0 + 1.0, z=3.0)
    holes = [pocket, prim.cylY(1.0, 20.0, rc + 1.0, z=FL_T + 3.5)]
    return body - prim.union([s_rear, s_front] + holes)


def _spool_flange(rf):
    """Front flange disc (PLA), local frame z 0 = rear face: flat disc r 9.5 .. rf, slots for the body's lugs, the
    raised friction annulus on its front face. It keeps the front 608 in and takes the clutch squeeze."""
    disc = prim.tube(rf, 9.5, FL_F)
    ann = prim.tube(ANNULUS[1], ANNULUS[0], ANNULUS[2] + 0.01, z=FL_F - 0.01)
    body = prim.union([disc, ann])
    slots = [prim.place(prim.box(CORE_R - 4.3, CORE_R - 0.7, -3.3, 3.3, -1.0, 1.6), tf.Rz(a))
             for a in (0.0, 120.0, 240.0)]
    return body - prim.union(slots)


def _shroud():
    """Shroud (PLA), R frame: arc shell round the storage reel (SHROUD_DEG) with a rear flange screwed to the wall.
    It keeps loose loops on the reel. Print standing on its flange."""
    a0, a1 = SHROUD_DEG
    ang = np.radians(np.linspace(a0, a1, 48))

    def ring(r_in, r_out, inset=0.0):
        an = np.radians(np.linspace(a0 + inset, a1 - inset, 48))
        pts = [(X_REEL + r_out * math.cos(t), r_out * math.sin(t)) for t in an] + \
              [(X_REEL + r_in * math.cos(t), r_in * math.sin(t)) for t in an[::-1]]
        return prim.poly_cs(pts)
    shell = prim.extrude(ring(*SHROUD_R), Z_D + D_T + 1.0 - 1.0, z=1.0)
    flange = prim.extrude(ring(SHROUD_R[0] + 0.5, SHROUD_R[1] + 1.5, inset=1.5), 3.0, z=0.02)
    tabs = []
    for a in SHROUD_SCREW_DEG:                    # screw tabs only (a full flange would reach into the rail sweep)
        x = X_REEL + SHROUD_SCREW_R * math.cos(math.radians(a))
        y = SHROUD_SCREW_R * math.sin(math.radians(a))
        tabs.append(prim.cyl(4.5, 3.0, x=x, y=y, z=0.02))
        tabs.append(prim.place(prim.box(-1.0, SHROUD_SCREW_R - SHROUD_R[0] - 1.0, -4.5, 4.5, 0.02, 3.02),
                               tf.T(X_REEL + (SHROUD_R[0] + 0.5) * math.cos(math.radians(a)),
                                    (SHROUD_R[0] + 0.5) * math.sin(math.radians(a)), 0) @ tf.Rz(a)))
    body = prim.union([shell, flange] + tabs)
    cut = [prim.box(LEG_X[0] - 1.5, LEG_X[1] + 1.5, -50.0, -38.0, -1.0, 11.5),    # window: the rocker leg (FIRM +2.5)
           prim.cylY(WASHER_R + 0.7, -50.0, -38.0, x=CRANK_X, z=CRANK_Z)]          # crank washer and screw head
    for a in SHROUD_SCREW_DEG:
        x = X_REEL + SHROUD_SCREW_R * math.cos(math.radians(a))
        y = SHROUD_SCREW_R * math.sin(math.radians(a))
        cut.append(prim.cyl(fits.M3_CLEAR_D / 2, 10, x=x, y=y, z=-1))
    return body - prim.union(cut)


def _swing_arm():
    """Swing arm (PPA-CF), joint frame: idler centre on +X at arm_r(). The ring rides on the bus gear's rear hub
    (its friction swings the arm with the bus), the plate runs out to the idler pin, an arc tab meets the stop pins."""
    r = arm_r()
    z0, z1 = ARM_Z
    ring = prim.tube(HUB1_R + 6.0, HUB1_R + fits.SHAFT_IN_BORE / 2, z1 - z0, z=z0)
    # drag: the ring is a split C (slot opposite the plate) bored 0.2 under the hub, so it grips the bus gear hub
    # and the arm follows the bus. Too stiff to swing: sand the bore; too slack: close the slot with a cable tie.
    ring = prim.tube(HUB1_R + 6.0, HUB1_R - 0.1, z1 - z0, z=z0)
    ring = ring - prim.box(-HUB1_R - 8.0, -HUB1_R + 1.0, -0.6, 0.6, z0 - 1, z1 + 1)
    plate = prim.box(HUB1_R + 3.0, r, -ARM_W / 2, ARM_W / 2, z0, z1) + prim.cyl(ARM_W / 2, z1 - z0, x=r, z=z0)
    tab_deg = STOP_OFF - math.degrees((STOP_R + 0.05) / STOP_RAD)
    ang = np.radians(np.linspace(-tab_deg, tab_deg, 40))
    pts = [((STOP_RAD + 3.0) * math.cos(t), (STOP_RAD + 3.0) * math.sin(t)) for t in ang] + \
          [((STOP_RAD - 3.0) * math.cos(t), (STOP_RAD - 3.0) * math.sin(t)) for t in ang[::-1]]
    tabs = prim.extrude(prim.poly_cs(pts), z1 - z0, z=z0)
    pin = prim.cyl(4.0, BUS_Z[1] - 0.6 - z1 + 0.01, x=r, z=z1 - 0.01)       # idler pin, ends 0.6 short of the wall
    return prim.union([ring, plate, tabs, pin])


def _bus_gear():
    """Bus gear c1 (PPA-CF), free on C: m1.5 46T face 6 in cad-drum's bus plane, rear hub r 13 for the swing arm.
    Prints gear face down, hub up."""
    g = gears.spur_gear(M_BUS, Z_C1, BUS_Z[1] - BUS_Z[0]).translate((0, 0, BUS_Z[0]))
    hub = prim.cyl(HUB1_R, HUB1_Z[1] - HUB1_Z[0] + 0.01, z=HUB1_Z[0])
    body = prim.union([g, hub])
    return body - prim.cyl((C_D + fits.SHAFT_IN_BORE) / 2, 100, z=HUB1_Z[0] - 10)


def _thrust_ring():
    """Thrust ring T (PLA), local frame: rear face z = 0. Big rear part takes the rocker bumps, small front boss
    presses only the rear 608 inner race. Slides on S."""
    a = prim.tube(13.0, (S_D + fits.HOLE_CLEAR) / 2, 2.5)
    b = prim.tube(6.0, (S_D + fits.HOLE_CLEAR) / 2, Z_SPOOL - Z_T - 0.05 - 2.49, z=2.49)
    return prim.union([a, b])


def _rocker():
    """Brake rocker (PPA-CF, flexible lever), local frame = R. Pivots on an M3 pin at the top, two trim screw tips press
    T at Y 0, the crank pushes the stiff leg at the bottom. With T blocked by the clutch stack the plate bends: it IS
    the clutch spring (reel_calc.rocker_section, plate t 3.0)."""
    x, y, z = ROCKER_PIVOT
    z0, z1 = ROCKER_Z
    w = 7.5
    plate = prim.box(x - 5.0, x + 5.0, y - 6.0, y, z0, z1) + prim.box(x - w, x + w, -30.0, y - 6.0, z0, z1)
    plate = plate + prim.box(LEG_X[0], LEG_X[1], LEG_Y0, -28.0, z0, LEG_Z1)          # stiff leg
    ring = prim.cyl(14.5, z1 - z0, x=x, z=z0)
    hole = prim.cyl(6.0, 10, x=x, z=z0 - 2)
    bosses = [prim.cyl(4.0, Z_T - 0.5 - z0, x=x + s * TRIM_R, z=z0) for s in (-1, 1)]
    taps = [prim.cyl(fits.M3_TAP_D / 2, 20, x=x + s * TRIM_R, z=z0 - 5) for s in (-1, 1)]
    boss = prim.cylX(2.6, x - 5.0, x + 5.0, y=y, z=z) ^ prim.box(x - 5.0, x + 5.0, y - 3, y + 3, z0, z + 3)
    pin = prim.cylX(fits.M3_CLEAR_D / 2, x - 10, x + 10, y=y, z=z)
    return prim.union([plate, ring, boss] + bosses) - prim.union([hole, pin] + taps)


P1_R = 7.0                    # line radius on pulley 1 (groove root 6.5 + half the line)
P1_DX = 14.0                  # pulley 1 centre this far past the exit eye along +X
P1_T = 6.0                    # pulley rim width (along R Y)


P2_R = P1_R
P2_WORLD_Y = 119.0            # cad-launch's pivot pulley plane (D34): the line leaves pulley 2 in it, heading -X
P2_WORLD_Z = -60.0            # feed height (coordinator: exit about (10, 119, -60))
P2_POST_X = 156.7             # R X of pulley 2's post (world x 138: clear of the drum and its switch block)


def p2_centre():
    """Pulley 2 centre (R frame), axis R +Y (world Z): the +Y run arrives on its +X side, leaves along world -X in
    the y 119 plane."""
    pc1 = p1_centre()
    x = pc1[0] + P1_R - P2_R
    y = P2_WORLD_Z - SP["origin"][2]
    z = SP["origin"][1] - (P2_WORLD_Y - P2_R)
    return np.array((x, y, z))


def p1_centre():
    """Pulley 1 (693ZZ) centre: turns the line from +X (out of the exit eye) to R -Z (world +Y), axis R +Y."""
    return np.array((EXIT[0] + P1_DX, EXIT[1], EXIT[2] - P1_R))


def exit_post_screws():
    ex, _, ez = EXIT
    return [(ex + sx * 17.0, ez + sz * 12.0) for sx in (-1, 1) for sz in (-1, 1)]   # clear of all four gussets


def _exit_post():
    """Exit eye post (PPA-CF): own foot on the wall's pad (4 x M3 off the centre lines), 16 x 16 column, gussets on
    all four sides (the line pulls it toward -Z and -X), 6 mm top with the plain eye at the line height, and a fork
    carrying pulley 1 on an M3 axle."""
    ex, ey, ez = EXIT
    px, py, pz = p1_centre()
    y0 = Y_DECK                                       # stands on the deck on its own foot (4 x M3 into inserts)
    foot = prim.box(ex - 20, ex + 20, y0, y0 + FOOT_T + 1.0, ez - 16, ez + 16)
    col = prim.box(ex - 8, ex + 8, y0, ey - 9, ez - 8, ez + 8)
    taper = prim.box(ex - 3, ex + 3, ey - 9.01, ey + 10.5, ez - 6, ez + 6)
    gus = []
    for s_ in (-1, 1):
        yg = y0 + FOOT_T - 1.0                     # gussets start inside the foot, not on its bottom face
        pts = [(ex + s_ * 7.5, yg), (ex + s_ * 16, yg), (ex + s_ * 7.5, yg + 36)]
        gus.append(prim.extrude(prim.poly_cs(pts), 16.0, z=ez - 8))
        gus.append(_gusset(ex - 4.0, ex + 4.0, ez + s_ * 7.5, ez + s_ * 13.5, yg, 36.0))
    # pulley fork: two cheeks above and below the pulley rim, from the post top out past the pulley
    gap = P1_T / 2 + 0.4
    cheeks = [prim.box(ex + 2.0, px + 7.0, py + s_ * gap, py + s_ * (gap + 3.5), pz - 8.0, ez + 1.5) for s_ in (-1, 1)]
    web = prim.box(ex - 2.5, ex + 2.5, ey - 9.5, ey + gap + 3.5, ez - 5.5, ez + 1.5)
    body = prim.union([foot, col, taper, web] + gus + cheeks + _race_bosses(px, py, pz, gap))
    cut = [_eye_cut(EXIT, exit_axis(), 6.0), prim.cylY(fits.M3_CLEAR_D / 2, py - 10, py + 10, x=px, z=pz)]
    for (x, z) in exit_post_screws():
        cut.append(prim.cylY(fits.M3_CLEAR_D / 2, y0 - 1, y0 + FOOT_T + 1, x=x, z=z))
    return body - prim.union(cut)


def _p2_post():
    """Pulley 2 post (PPA-CF), R frame: foot on the deck (4 x M3), 16 x 16 column at P2_POST_X beside the drum, an
    arm with a deep web above the pulley (the drum sits below) and two cheeks round the pulley on an M3 axle."""
    px, py, pz = p2_centre()
    gap = P1_T / 2 + 0.4
    xc = P2_POST_X
    foot = prim.box(xc - 16, xc + 16, Y_DECK, Y_DECK + FOOT_T, pz - 14, pz + 14)
    col = prim.box(xc - 8, xc + 8, Y_DECK, py + gap + 22.0, pz - 8, pz + 8)
    upper = prim.box(px - 8.0, xc, py + gap, py + gap + 3.5, pz - 6, pz + 6)
    web = prim.box(px - 8.0, xc, py + gap + 3.49, py + gap + 22.0, pz - 4, pz + 4)
    lower = prim.box(px - 8.0, xc, py - gap - 3.5, py - gap, pz - 6, pz + 6)
    gus = [_gusset(xc - 4, xc + 4, pz + s_ * 7.5, pz + s_ * 14.0, Y_DECK + FOOT_T - 1.0, 40.0) for s_ in (-1, 1)]
    body = prim.union([foot, col, upper, web, lower] + gus + _race_bosses(px, py, pz, gap))
    cut = [prim.cylY(fits.M3_CLEAR_D / 2, py - 15, py + 15, x=px, z=pz)]
    for sx in (-1, 1):
        for sz in (-1, 1):
            cut.append(prim.cylY(fits.M3_CLEAR_D / 2, Y_DECK - 1, Y_DECK + FOOT_T + 1, x=xc + sx * 11.5, z=pz + sz * 9.5))
    return body - prim.union(cut)


RACE_BOSS_R = 2.2             # cheek bosses clamp the 693 inner race (OD about 4.6), not the pulley rim


def _race_bosses(px, py, pz, gap):
    """Two bosses, one on each cheek's inner face, reaching the 693 inner race faces (1 mm inside the pulley faces)."""
    h = gap - 2.0 - 0.05                             # cheek face to the race face (693 half width 2), 0.05 shy
    return [prim.cylY(RACE_BOSS_R, py + gap - h, py + gap + 0.01, x=px, z=pz),
            prim.cylY(RACE_BOSS_R, py - gap - 0.01, py - gap + h, x=px, z=pz)]


def _pulley():
    """Pulley 1 (PLA): rim d 2 x (P1_R + 1.5), U groove for the 1 mm line (root r P1_R - 0.5), crush-rib seat for
    a 693ZZ (3 x 8 x 4). Local frame: axis Z, z 0 .. P1_T."""
    ro = P1_R + 2.0
    rr = P1_R - 0.5
    ri = RACE_BOSS_R + 0.4                            # face bore: the cheek bosses enter it freely
    prof = [(ri, 0), (ro, 0), (ro, 1.2), (rr + 0.6, P1_T / 2 - 0.7), (rr, P1_T / 2), (rr + 0.6, P1_T / 2 + 0.7),
            (ro, P1_T - 1.2), (ro, P1_T), (ri, P1_T)]
    body = prim.revolve_pts(prof, seg=96)
    seat = prim.place(bearings.seat_693(depth=4.0, bottom="through", below=3.0), tf.T(0, 0, P1_T / 2 + 2.0))
    return body - seat


def _cap_clip():
    """Harry's v7 C-clip (4.6 bore, 3.8 opening toward -Y, 2 thick) on the 4.5 neck of K; dia 14 so it bears on
    the cup hub face (the cup's 10.3 bore mouth is wider than v7's)."""
    return prim.tube(7.0, 2.3, 2.0) - prim.box(-1.9, 1.9, -8, 0, -1, 3)


# ---- SW_ARM: printed contact behind the cup gear (BASE_SPEC section 4, GPIO 42)
SW_BUMP_R = 16.5              # bump radius on the cup gear's rear face (screw boss stays outside the gear tips)
SW_BUMP_H = 2.1               # track height (gap 0.4 + pretravel 1.3 + 0.4 overtravel)
SW_BUMP_DEG = 12.0            # contact closed +- this about home
SW_LEVER = 28.0


def _sw():
    return switch.contact_switch(SW_LEVER)


SW_TIP_X = -3.0              # lever tip off the vertical (R -X): its nut and jam nut clear cad-drum's wrap idler


def sw_tip_deg():
    """Angle (deg about the cup axis, R frame) of the SW_ARM lever tip = the home notch centre."""
    return math.degrees(math.atan2(-math.sqrt(SW_BUMP_R ** 2 - SW_TIP_X ** 2), SW_TIP_X))


def sw_frame():
    """Switch frame -> R: switch +X = R +Y (lever tip toward the axis), switch +Z = R +Z (toward the gear)."""
    i = _sw()["info"]
    z_top = 16.5 if "z_lever_top" not in i else i["z_lever_top"]
    tip_y = -math.sqrt(SW_BUMP_R ** 2 - SW_TIP_X ** 2)
    z_rest = CUPGEAR_Z[0] - 0.4                                # free lever top 0.4 behind the gear face (in the notch)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = (0, 1, 0), (-1, 0, 0), (0, 0, 1)
    M[:3, 3] = (SW_TIP_X, tip_y - i["x_act"], z_rest - z_top)
    return M


def _cup_gear_track():
    """Raised track on the cup gear's rear face (gear frame z 0 = rear face, protruding -z), r 13 .. 16, with a notch
    at home (local 270 deg, +-SW_BUMP_DEG, 45 deg ramps). The SW_ARM lever rides the track (contact CLOSED) and drops
    into the notch at home (contact OPEN): one open pulse per cup turn. Prints rear face down: the track is the bed
    face."""
    ring = prim.tube(16.0, 13.0, SW_BUMP_H, z=-SW_BUMP_H)
    half = math.radians(SW_BUMP_DEG)
    a_top = half + SW_BUMP_H / SW_BUMP_R                   # 45 deg ramps measured at the lever radius
    n = 24
    ang = np.linspace(-a_top, a_top, n)
    pts = [(0.0, 0.0)]
    for t in ang:                                          # wedge-shaped notch cutter in the (angle, z) sense
        pts.append((20.0 * math.cos(t - math.pi / 2), 20.0 * math.sin(t - math.pi / 2)))
    notch = prim.extrude(prim.poly_cs(pts), SW_BUMP_H + 1.0, z=-SW_BUMP_H - 0.5)
    notch = prim.place(notch, tf.Rz(sw_tip_deg() + 90.0))       # centred on the lever tip
    # 45 deg ramps: subtract a slightly smaller notch at full depth and a wider shallow one is approximated by
    # stepping the notch in three depths
    out = ring - notch
    return out


def _sw_bracket():
    """SW_ARM bracket (PLA), R frame: flange screwed to the wall rear face below the gears, an arm back under the
    gears, a plate the switch base screws onto."""
    M = sw_frame()
    (x0, y0, z0), (x1, y1, z1) = prim.bbox(prim.place(_sw()["base"], M))
    zb = z0
    plate = prim.box(x0 - 3.0, x1 + 1.5, y0 - 9.0, y1, zb - 4.0, zb)      # +X edge 2 mm off the wrap idler
    arm = prim.box(-11.5, 8.5, y0 - 9.0, y0 - 1.0, zb - 4.0, -WALL_T)        # below the switch base, clear of the wrap idler
    flange = prim.box(-16.0, 16.0, y0 - 17.0, y0 - 1.0, -WALL_T - 4.0, -WALL_T)
    body = prim.union([plate, arm, flange])
    cut = [prim.cyl(fits.M3_CLEAR_D / 2, 10, x=sx, y=y0 - 13.0, z=-WALL_T - 6.0) for sx in (-12.0, 12.0)]
    return body - prim.union(cut), y0


def fasteners_m3(L):
    """M3 SHCS model along +Z: head z = -3 .. 0, shank 0 .. L."""
    return prim.union([prim.cyl(2.75, 3.0, z=-3.0), prim.cyl(1.5, L)])


# ------------------------------------------------------------------------------------------------ contract
def _bed(solid, R=None):
    """print_tf: rotate by R (4x4, default none) and move the result onto the bed corner (min x, y, z = 0)."""
    R = np.eye(4) if R is None else R
    (x0, y0, z0), _ = prim.bbox(prim.place(solid, R))
    return tf.T(-x0, -y0, -z0) @ R


def _clip_z(d, face0):
    """z of a clip's lower face in its groove behind a gear whose rear face is at face0 (shaft diameter d)."""
    return face0 - 0.3 - clips.groove_dims(d)["width"] + clips.CLIP_AX_CLEAR / 2


def parts(P):
    d = _d(P)
    rf = d["flange_r"]
    out = []

    def add(name, solid, mat, R=None, **k):
        if mat not in ("bought", "steel"):
            solid = prim.clean(solid)          # merge near-duplicate vertices so the STL stays watertight
        pt = _bed(solid, R) if mat not in ("bought", "steel") else None
        out.append(Part(name, solid, mat, print_tf=pt, **k))

    add("reel_wall", _wall(), "PLA", tf.Rx(90), joint="reel_base", color=COL["wall"], step=1,
        notes="Reel wall: foot, guide eye post, rocker ears, stop pins, exit post pad. Print on its foot")
    add("reel_sprag_hub", _sprag_hub(), "PPA-CF", tf.Rx(180), joint="reel_base", color=COL["post"], step=2,
        notes="Sprag hub: holds the CSK20PP backstop and bushes the capstan tube. Print front face down")
    add("reel_hub_screws", prim.union([prim.place(fasteners_m3(8.0), tf.T(HUB_SCREW_R * math.cos(math.radians(a)),
                                                                     HUB_SCREW_R * math.sin(math.radians(a)),
                                                                     3.0) @ tf.Rx(180))
                                       for a in hub_screw_deg()]), "bought", joint="reel_base", color=COL["brg"],
        bought_ref="4 x M3x8 SHCS", step=2, notes="Hub screws from the front, tapped into the wall")
    add("reel_sprag", bearings.bearing_csk20(okey_deg=90.0, ikey_deg=0.0), "bought", joint="reel_base",
        place=tf.T(0, 0, Z_BLOCK), color=COL["brg"], bought_ref="CSK20PP one-way bearing 20x47x14", step=2,
        notes="Sprag #2: capstan backstop, lets the capstan turn only in the wind-in sense")
    add("reel_c", _c_tube(), "PPA-CF", tf.Rx(180), joint="reel_capstan", color=COL["ppa"], step=3,
        notes="Capstan tube: key rib in the sprag and drum, D flat for the capstan gear. Print standing, front end down")
    add("reel_bus", _bus_gear(), "PPA-CF", tf.Rx(180), joint="reel_bus", color=COL["gear"], step=4,
        notes=f"Bus gear m{M_BUS:g} z{Z_C1} (free on the capstan tube): driven by the drum's bus gear, drives the "
              "swing idler. Print face down")
    add("reel_swing_arm", _swing_arm(), "PPA-CF", joint="reel_swing", color=COL["ppa"], step=4,
        notes="Swing arm: rides on the bus gear hub, carries the idler into mesh on forward wiper turns. Print flat")
    add("reel_gi", gears.spur_gear(M_BUS, Z_IDL, BUS_Z[1] - BUS_Z[0], bore=8.0 + fits.SHAFT_IN_BORE), "PPA-CF",
        joint="reel_idler", place=tf.T(0, 0, BUS_Z[0]), color=COL["gear"], step=4,
        notes=f"Swing idler m{M_BUS:g} z{Z_IDL}: meshes the reel gear only on forward wiper turns")
    add("reel_gc", _gear(M_CAP, Z_CAPG, GC_Z[1] - GC_Z[0], C_D, 2.0, hub=(28.0, HUB1_Z[0] - GC_Z[1] - 0.2)),
        "PPA-CF", joint="reel_capstan", place=tf.T(0, 0, GC_Z[0]), color=COL["gear"], step=5,
        notes=f"Capstan gear m{M_CAP:g} z{Z_CAPG}: driven by the drum's REEL output gear ({Z_MAIN}T)")
    add("reel_c_clip", clips.c_clip(C_D), "PLA", joint="reel_capstan", place=tf.T(0, 0, _clip_z(C_D, GC_Z[0])),
        color=COL["clip"], step=5, notes="C-clip holding the capstan gear on the tube")
    add("reel_drum", v7.drum(mouth=(14.2, COLLAR_CONE + 0.2)), "PLA", joint="reel_capstan", place=tf.T(0, 0, Z_DRUM), color=COL["drum"], step=6,
        notes="Capstan drum dia 40 x 40 (Harry v7), keyed on the tube. Print flange down")
    add("reel_k", _k_shaft(), "PPA-CF", joint="reel_cup", color=COL["ppa"], step=7,
        notes="Cup shaft: runs inside the capstan tube, cup in front, cup gear behind. Print standing")
    add("reel_cup", v7.cup(), "PLA", joint="reel_cup", place=tf.T(0, 0, Z_CUP), color=COL["cup"], step=7,
        notes="Wrap cup (Harry v7): ring and groove lay the wraps on the capstan. Print open end down")
    add("reel_cup_clip", _cap_clip(), "PLA", joint="reel_cup", place=tf.T(0, 0, k_front() + 0.3),
        color=COL["clip"], step=7, notes="Harry's v7 C-clip on the cup shaft neck in front of the cup")
    add("reel_gk", prim.union([_gear(M_CUPG, Z_CUPG, CUPGEAR_Z[1] - CUPGEAR_Z[0], K_D, 1.2,
                                     hub=(16.0, C_Z0 - CUPGEAR_Z[1] - 0.3), phase=GK_PHASE), _cup_gear_track()]),
        "PPA-CF",
        joint="reel_cup", place=tf.T(0, 0, CUPGEAR_Z[0]), color=COL["gear"], step=8,
        notes=f"Cup gear m{M_CUPG:g} z{Z_CUPG}: driven by the drum's wrap idler; the bump on its back trips SW_ARM")
    add("reel_k_clip", clips.c_clip(K_D), "PLA", joint="reel_cup", place=tf.T(0, 0, _clip_z(K_D, CUPGEAR_Z[0])),
        color=COL["clip"], step=8, notes="C-clip holding the cup gear on the cup shaft")
    add("reel_s", _s_shaft(), "PPA-CF", tf.Rx(180), joint="reel_shaft", color=COL["ppa"], step=9,
        notes="Reel shaft: 608 journals, head = clutch disk, D flat for the reel gear. Print head down")
    add("reel_spool", _spool_body(rf), "PLA", joint="reel_spool", place=tf.T(0, 0, Z_SPOOL), color=COL["spool"],
        infill=0.25, step=9, notes=f"Storage reel body: 20 m of 1 mm Dyneema on a dia {2 * CORE_R:g} core, "
                                   "two 608s. Print rear flange down")
    add("reel_spool_flange", _spool_flange(rf), "PLA", joint="reel_spool",
        place=tf.T(0, 0, Z_SPOOL + FL_T + WIDTH), color=COL["spool"], step=9,
        notes="Front flange disc: keeps the front 608 in, carries the clutch friction ring. Print flat")
    for i, zb in ((1, Z_SPOOL + bearings.B608_W), (2, Z_SPOOL + FL_T + WIDTH)):
        add(f"reel_brg_{i}", bearings.bearing_608(), "bought", joint="reel_spool", place=tf.T(0, 0, zb),
            color=COL["brg"], bought_ref="608ZZ 8x22x7", step=9, notes="608 bearing in the storage reel")
    add("reel_t", _thrust_ring(), "PLA", joint="reel_t", place=tf.T(0, 0, Z_T), color=COL["clip"], step=9,
        notes="Thrust ring: the rocker pushes it onto the rear 608 inner race")
    add("reel_rocker", _rocker(), "PPA-CF", joint="reel_base", color=COL["rocker"], step=10,
        notes="Brake rocker: flexible PPA-CF lever = clutch spring, pushed by the crank. Print flat")
    add("reel_rocker_pin", fasteners_m3(24.0), "bought", joint="reel_base",
        place=tf.T(ROCKER_PIVOT[0] - 12.0, ROCKER_PIVOT[1], ROCKER_PIVOT[2]) @ tf.Ry(90), color=COL["brg"],
        bought_ref="M3x25 SHCS + M3 nut", step=10, notes="Rocker pivot pin")
    add("reel_trim_screws", prim.union([prim.place(fasteners_m3(8.0), tf.T(X_REEL + s_ * TRIM_R, 0, Z_T - 8.0))
                                        for s_ in (-1, 1)]), "bought", joint="reel_base", color=COL["brg"],
        bought_ref="2 x M3x8 SHCS", step=10, notes="LIGHT trim screws: their tips press the thrust ring; 1/8 turn = 0.4 N")
    add("reel_crank", _crank(), "PPA-CF", tf.Rx(-90), joint="reel_crank", color=COL["ppa"], step=10,
        notes="Brake bell crank: the bar turns it, arm B pushes the rocker leg. Print on its flat side")
    add("reel_crank_washer", _washer(), "PPA-CF", joint="reel_base",
        place=tf.T(CRANK_X, CRANK_BOSS_TOP, CRANK_Z) @ tf.Rx(-90), color=COL["ppa"], step=10,
        notes="Printed washer: keeps the crank on its boss")
    add("reel_crank_screw", fasteners_m3(10.0), "bought", joint="reel_base",
        place=tf.T(CRANK_X, CRANK_BOSS_TOP + WASHER_T, CRANK_Z) @ tf.Rx(90), color=COL["brg"],
        bought_ref="M3x10 SHCS", step=10, notes="Crank retaining screw, into the boss")
    add("reel_bar", _bar(), "PPA-CF", joint="reel_bar", color=COL["ppa"], step=10,
        notes="Brake bar: cad-drum's track 2 cam pushes its nose, its left end turns the crank. Print flat")
    for k, (gx, ylo, yhi) in enumerate(_guide_ys()):
        add(f"reel_bar_cap_{k + 1}", _guide_cap(gx, ylo, yhi), "PLA", tf.Rx(-90), joint="reel_base",
            color=COL["wall"], step=10, notes="Bar guide cap: closes the U guide over the bar. Print flat")
    add("reel_bar_screws", prim.union([prim.place(fasteners_m3(16.0), tf.T(gx, yhi + GUIDE_CLEAR + 0.05 + GUIDE_JAW,
                                                                           z) @ tf.Rx(90))
                                       for gx, ylo, yhi in _guide_ys()
                                       for z in ((BAR_Z[0] - GUIDE_CLEAR) / 2, GUIDE_SCREW_Z)]),
        "bought", joint="reel_base", color=COL["brg"], bought_ref="4 x M3x16 SHCS", step=10,
        notes="Guide cap screws (the front one crosses the slot mouth in front of the bar)")
    gr = _gear(M_BUS, Z_REELG, BUS_Z[1] - BUS_Z[0], S_D, 1.0, hub=(14.0, -WALL_T - BUS_Z[1] - 0.1))
    gr = gr - prim.union([prim.cyl(2.0, 40, x=s_ * TRIM_R, z=-10) for s_ in (-1, 1)])   # hex key to the trim screws
    add("reel_gr", gr,
        "PPA-CF", joint="reel_shaft", place=tf.T(0, 0, BUS_Z[0]), color=COL["gear"], step=11,
        notes=f"Reel gear m{M_BUS:g} z{Z_REELG}: turns the clutch disk {Z_BUSMAIN / Z_REELG:g} x wiper speed")
    add("reel_s_clip", clips.c_clip(S_D), "PLA", joint="reel_shaft", place=tf.T(0, 0, _clip_z(S_D, BUS_Z[0])),
        color=COL["clip"], step=11, notes="C-clip holding the reel gear")
    sw = _sw()
    Msw = sw_frame()
    brk, _ = _sw_bracket()
    add("reel_sw_bracket", brk, "PLA", joint="reel_base", color=COL["wall"], step=14,
        notes="SW_ARM bracket: screwed to the wall below the gears. Print flat")
    add("reel_sw_base", sw["base"], "PLA", joint="reel_base", place=Msw, color=COL["clip"], step=14,
        notes="SW_ARM contact base (printed switch, M3 head on M3 nut)")
    add("reel_sw_lever", sw["lever"], "PLA", tf.Rx(90), joint="reel_base", place=Msw, color=COL["rocker"], step=14,
        notes="SW_ARM lever: the cup gear's bump presses its tip at home. Print on its side")
    add("reel_sw_hw", prim.union([sw["screw"], sw["nut"], sw["jam_nut"]]), "bought", joint="reel_base", place=Msw,
        color=COL["brg"], bought_ref="M3x10 SHCS + 2 M3 nuts", step=14, notes="SW_ARM contact screw and nuts")
    add("reel_shroud", _shroud(), "PLA", joint="reel_base", color=COL["wall"], step=12,
        notes="Shroud round the storage reel: keeps loose loops on it. Print standing on its flange")
    add("reel_exit_post", _exit_post(), "PPA-CF", tf.Rx(90), joint="reel_base", color=COL["post"], step=13,
        notes="Exit eye post with the pulley 1 fork: carries the full tow load, so PPA-CF. Print on its foot")
    pc = p1_centre()
    add("reel_pulley1", _pulley(), "PLA", joint="reel_base", place=tf.T(*pc) @ tf.Rx(-90) @ tf.T(0, 0, -P1_T / 2),
        color=COL["clip"], step=13, notes="Pulley 1: turns the line from the exit eye toward the rail (D14)")
    add("reel_p1_brg", bearings.bearing_693(), "bought", joint="reel_base",
        place=tf.T(*pc) @ tf.Rx(-90) @ tf.T(0, 0, 2.0), color=COL["brg"], bought_ref="693ZZ 3x8x4", step=13,
        notes="Bearing in pulley 1")
    p2 = p2_centre()
    add("reel_p2_post", _p2_post(), "PPA-CF", tf.Rx(90), joint="reel_base", color=COL["post"], step=15,
        notes="Pulley 2 post: stands on the deck beside the drum, turns the line into the rail pivot pulley's plane")
    add("reel_pulley2", _pulley(), "PLA", joint="reel_base", place=tf.T(*p2) @ tf.Rx(-90) @ tf.T(0, 0, -P1_T / 2),
        color=COL["clip"], step=15, notes="Pulley 2: feeds the line to cad-launch's rail pivot pulley (D34)")
    add("reel_p2_brg", bearings.bearing_693(), "bought", joint="reel_base",
        place=tf.T(*p2) @ tf.Rx(-90) @ tf.T(0, 0, 2.0), color=COL["brg"], bought_ref="693ZZ 3x8x4", step=15,
        notes="Bearing in pulley 2")
    add("reel_p2_axle", fasteners_m3(20.0), "bought", joint="reel_base",
        place=tf.T(p2[0], p2[1] + P1_T / 2 + 0.4 + 3.5 + 3.0, p2[2]) @ tf.Rx(90), color=COL["brg"],
        bought_ref="M3x20 SHCS + nyloc nut", step=15, notes="Pulley 2 axle")
    add("reel_p1_axle", fasteners_m3(20.0), "bought", joint="reel_base",
        place=tf.T(pc[0], pc[1] + P1_T / 2 + 0.4 + 3.5 + 3.0, pc[2]) @ tf.Rx(90), color=COL["brg"],
        bought_ref="M3x20 SHCS + nyloc nut", step=13, notes="Pulley 1 axle")
    return out


def joints(P):
    sp = _spec(P) if P is not None else SP
    base = np.eye(4)
    base[:3, 0], base[:3, 1], base[:3, 2] = [np.asarray(a, float) for a in sp["axes"]]
    return [
        Joint("reel_base", None, "fixed", sp["origin"], (0, 0, 1), base=base),
        Joint("reel_capstan", "reel_base", "rev", (0, 0, 0), (0, 0, 1)),
        Joint("reel_cup", "reel_base", "rev", (0, 0, 0), (0, 0, 1)),
        Joint("reel_bus", "reel_base", "rev", (0, 0, 0), (0, 0, 1)),
        Joint("reel_swing", "reel_base", "rev", (0, 0, 0), (0, 0, 1), base=tf.Rz(arm_deg()),
              limits=(-SWING_OUT_DEG, 0.0)),
        Joint("reel_idler", "reel_swing", "rev", (arm_r(), 0, 0), (0, 0, 1)),
        Joint("reel_shaft", "reel_base", "rev", (X_REEL, 0, 0), (0, 0, 1)),
        Joint("reel_spool", "reel_base", "rev", (X_REEL, 0, 0), (0, 0, 1)),
        Joint("reel_t", "reel_base", "slide", (X_REEL, 0, 0), (0, 0, 1), limits=(0.0, 0.3)),
        Joint("reel_bar", "reel_base", "slide", (0, 0, 0), (-1, 0, 0), limits=(0.0, BRAKE_STROKE)),
        Joint("reel_crank", "reel_base", "rev", (CRANK_X, 0, CRANK_Z), (0, 1, 0), limits=(-25.0, 0.0)),
    ]


def mesh_offset(alpha_deg, z1, z2):
    """Phase offset (deg) of gear 2 for gear 1 at angle 0 (tooth 0 on +X), gear 2 centre at direction alpha_deg from
    gear 1: theta2 = -(z1 / z2) theta1 + offset puts a gap of gear 2 on each tooth of gear 1 at the contact line."""
    return alpha_deg + 180.0 + 180.0 / z2 + (z1 / z2) * alpha_deg


def gear_angles(bus_deg, swing_deg=0.0):
    """Bus gear, idler (in its arm frame) and reel gear angles (deg). Gear tooth 0 on +X at angle 0.
    The idler is a planet on the arm: its joint angle is its world angle minus the arm angle."""
    ix, iy = idler_xy()
    a1 = arm_deg() + swing_deg
    th_i = -(Z_C1 / Z_IDL) * bus_deg + mesh_offset(a1, Z_C1, Z_IDL)
    a2 = math.degrees(math.atan2(-iy, X_REEL - ix))
    th_r = -(Z_IDL / Z_REELG) * th_i + mesh_offset(a2, Z_IDL, Z_REELG)
    return bus_deg, th_i - a1, th_r


def _zone(P, drum_deg):
    drum = (getattr(P, "DRUM", None) if P is not None else None) or {}
    for z in drum.get("zones", []):
        if z["deg"][0] <= drum_deg <= z["deg"][1]:
            return z
    return None


def brake_mm(P, S):
    """Cam lift at the follower (0 LIGHT .. BRAKE_STROKE FIRM): cad-drum's own track 2 profile when its module is
    there, else linear on the ramps of the params DRUM zones."""
    if "reel_brake_mm" in S:
        return float(S["reel_brake_mm"])
    if "drum_deg" in S:
        dd = float(S["drum_deg"])
        try:
            from cad.parts import _drum_kin as K
            return float(K.brake_lift(dd, K.cfg(P)))
        except Exception:
            pass
        z = _zone(P, dd)
        if z is not None:
            c = z.get("clutch", "LIGHT")
            if c == "FIRM":
                return BRAKE_STROKE
            if c == "LIGHT":
                return 0.0
            lo, hi = z["deg"]
            f = min(max((dd - lo) / max(hi - lo, 1e-9), 0.0), 1.0)
            return BRAKE_STROKE * (f if c.startswith("LIGHT to") else 1.0 - f)
    mode = str(S.get("mode", "LAUNCH")).upper()
    return BRAKE_STROKE if mode in ("REEL", "WRAPPED", "TOW", "NEUTRAL") else 0.0


NOSE_R = 3.0


def bar_mm(P, S):
    """Bar slide (-X). With cad-drum's module: the round nose's envelope on the real cam profile (on a ramp the
    nose touches off the follower line, higher up the ramp). Else the cam lift less the LIGHT gap."""
    if "reel_brake_mm" not in S:
        try:
            from cad.parts import drum as _dm
            g, c = _dm.G(P), _dm.C(P)
            th = _dm.drum_theta(S, c)
            d0 = g["cam_r0"] + BAR_GAP + NOSE_R
            dreq = d0
            for k in range(-40, 41):
                dl = 0.2 * k                                   # deg off the follower line
                r = _dm.cam_radius(g, c, g["phi_brake"] + dl - th)
                y = r * math.sin(math.radians(dl))
                if abs(y) < NOSE_R:
                    dreq = max(dreq, r * math.cos(math.radians(dl)) + math.sqrt(NOSE_R ** 2 - y * y))
            return min(max(0.0, dreq - d0), BRAKE_STROKE)
        except Exception:
            pass
    return max(0.0, brake_mm(P, S) - BAR_GAP)


def crank_deg(bar):
    """Crank angle about R +Y for a bar slide: arm A's tip circle stays on the bar's end face."""
    return -math.degrees(math.asin(min(bar / CRANK_LA, 1.0)))


def leg_push_mm(bar):
    """Arm B tip rise (+Z) at the rocker leg for a bar slide."""
    p = math.radians(crank_deg(bar))
    bx, bz = CRANK_B
    return -bx * math.sin(p) + bz * (math.cos(p) - 1.0)


def _num(S, k, default=0.0):
    v = S.get(k, default)
    return float(default if v is None else v)


_IDLER_HOME = {}


def _drum_vals(P, S):
    """cad-drum's joint values for S, plus its wrap idler angle at drum home, or None when drum.py is absent."""
    try:
        from cad.parts import drum as _drum
        v = _drum.kin(P, S)
        key = id(P)
        if key not in _IDLER_HOME:
            _IDLER_HOME[key] = _drum.kin(P, dict(drum_deg=0.0))["drum_idler"]
        return v, _IDLER_HOME[key]
    except Exception:
        return None


def kin(P, S):
    d = _d(P)
    wiper = _num(S, "wiper_deg")
    # wiper FORWARD turns the main shaft negative about R +Z, so a gear meshing it turns positive (wind-in sense)
    bus = float(S.get("bus_deg", (Z_BUSMAIN / Z_C1) * wiper)) + BUS_PHASE
    if "capstan_deg" in S:
        cap = float(S["capstan_deg"])
    else:
        # params 1.2.3: reel_deg = REEL output gear angle (changes only with the collar in REEL), wind-in sense
        cap = (Z_MAIN / Z_CAPG) * _num(S, "reel_deg") + CAP_PHASE
    swing = 0.0 if _num(S, "wiper_dir", 1.0) >= 0 else -SWING_OUT_DEG
    dv = None if ("bus_deg" in S or "capstan_deg" in S) else _drum_vals(P, S)
    cup = None
    if dv is not None:
        v, idl0 = dv
        if "drum_main" in v:
            bus = (Z_BUSMAIN / Z_C1) * float(v["drum_main"]) + BUS_PHASE_D
        if "drum_out_reel" in v:
            cap = (Z_MAIN / Z_CAPG) * float(v["drum_out_reel"]) + CAP_PHASE_D
        if "drum_idler" in v and ("wraps" not in S or "drum_deg" in S):
            cup = (42.0 / Z_CUPG) * (float(v["drum_idler"]) - idl0)
    _, th_i, th_r = gear_angles(bus, swing)
    wraps = _num(S, "wraps") if cup is None else -cup / 360.0
    if "spool_deg" in S:
        spool = float(S["spool_deg"])
    else:
        r_mid = 0.5 * (d["core_r"] + reel_calc.pack_r(d, d["line_len_m"]))
        spool = -math.degrees(_num(S, "line_out_m") * 1000.0 / r_mid)
    bar = bar_mm(P, S)
    return {"reel_capstan": cap, "reel_cup": -360.0 * wraps, "reel_bus": bus, "reel_swing": swing,
            "reel_idler": th_i, "reel_shaft": th_r, "reel_spool": spool, "reel_t": 0.0, "reel_bar": bar,
            "reel_crank": crank_deg(bar)}


def allowed(P):
    return [
        Allow("reel_sprag", "reel_sprag_hub", "CSK20PP pressed in its crush-rib seat, outer key rib"),
        Allow("reel_sprag", "reel_c", "C rib in the sprag inner keyway, inner race on C (model is one solid)"),
        Allow("reel_hub_screws", "reel_wall", "screws through the wall"),
        Allow("reel_hub_screws", "reel_sprag_hub", "screws tapped into the hub flange"),
        Allow("reel_sprag_hub", "reel_c", "capstan tube in the hub bushing"),
        Allow("reel_sprag_hub", "reel_wall", "hub spigot in the wall hole"),
        Allow("reel_c", "reel_gc", "D-flat fit"),
        Allow("reel_c", "reel_c_clip", "clip in the groove"),
        Allow("reel_c", "reel_drum", "keyed fit"),
        Allow("reel_c", "reel_bus", "bus gear turns free on the capstan tube"),
        Allow("reel_k", "reel_cup", "D fit"),
        Allow("reel_k", "reel_cup_clip", "clip on the neck"),
        Allow("reel_k", "reel_gk", "D fit"),
        Allow("reel_k", "reel_k_clip", "clip in the groove"),
        Allow("reel_swing_arm", "reel_gi", "idler turns on the arm pin"),
        Allow("reel_swing_arm", "reel_bus", "split arm ring grips the bus gear hub (drag)"),
        Allow("reel_s", "reel_gr", "D-flat fit"),
        Allow("reel_s", "reel_s_clip", "clip in the groove"),
        Allow("reel_brg_*", "reel_spool", "608 pressed in the reel"),
        Allow("reel_brg_*", "reel_s", "journal in the inner race"),
        Allow("reel_spool", "reel_spool_flange", "lugs in the flange slots"),
        Allow("reel_rocker_pin", "reel_wall", "M3 pin through the ears"),
        Allow("reel_rocker_pin", "reel_rocker", "M3 pin through the rocker"),
        Allow("reel_trim_screws", "reel_rocker", "trim screws tapped into the rocker bosses"),
        Allow("reel_p1_brg", "reel_pulley1", "693ZZ pressed in the pulley"),
        Allow("reel_sw_hw", "reel_sw_*", "contact screw in the lever, nut in the base trap"),
        Allow("reel_sw_lever", "reel_sw_base", "lever clamped on the base pad"),
        Allow("reel_sw_lever", "reel_gk", "the track holds the lever down (contact closed) except at home",
              max_vol=60.0),
        Allow("reel_p1_axle", "reel_p1_brg", "axle through the inner race"),
        Allow("reel_p1_axle", "reel_exit_post", "axle through the fork cheeks"),
        Allow("reel_p2_brg", "reel_pulley2", "693ZZ pressed in the pulley"),
        Allow("reel_p2_axle", "reel_p2_brg", "axle through the inner race"),
        Allow("reel_p2_axle", "reel_p2_post", "axle through the cheeks"),
        Allow("reel_trim_screws", "reel_t", "screw tips on the thrust ring (preload)", max_vol=0.5),
        Allow("reel_crank", "reel_rocker", "arm B bends the flexible rocker (static in the model; 5 mm at FIRM)",
              max_vol=140.0),
        Allow("reel_crank_washer", "reel_crank_screw", "screw through the washer"),
        Allow("reel_crank_screw", "reel_wall", "screw tapped into the crank boss"),
        Allow("reel_bar_screws", "reel_wall", "cap screws tapped into the guides"),
        Allow("reel_bar_screws", "reel_bar_cap_*", "cap screws through the caps"),
    ]


def clearances(P):
    return [
        Clearance("reel_cup", "reel_drum", 0.25, "cup skirt over the drum (v7 0.6 radial)"),
        Clearance("reel_drum", "reel_sprag_hub", 0.5, "drum flange in front of the hub"),
        Clearance("reel_bus", "reel_gi", 0.0, "mesh"),
        Clearance("reel_gi", "reel_gr", 0.0, "mesh"),
        Clearance("reel_spool", "reel_wall", 0.3, "reel spins in front of the wall"),
        Clearance("reel_spool*", "reel_shroud", 1.0, "reel spins inside the shroud"),
        Clearance("reel_spool", "reel_rocker", 0.3, "rocker behind the reel"),
    ]


def grounded(P):
    return ["reel_wall", "reel_p2_post", "reel_exit_post"]


def audit_states(P):
    out = []
    for k in range(6):                          # one tooth pitch of each mesh
        out.append(dict(phase="reel_mesh", t=0.0, drum_deg=347.0, wiper_deg=k * 360.0 / Z_BUSMAIN / 5,
                        reel_deg=k * 360.0 / Z_MAIN / 5))
    for k in range(3):
        out.append(dict(phase="reel_swing_out", t=0.0, drum_deg=0.0, wiper_dir=-1, wiper_deg=-k * 5.0))
    for w in (0.0, 0.25, 0.5, 0.75, 1.0, 3.5, float(WRAPS)):
        out.append(dict(phase="reel_wrap", t=0.0, wraps=w))
    out.append(dict(phase="reel_firm", t=0.0, drum_deg=41.0, line_out_m=3.0))
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


def _line_points(P, wraps=0.0, pack_r=None, z_spool=None):
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
    pc = p1_centre()
    top = np.array((pc[0], pc[1], pc[2] + P1_R))
    spans.append((np.array([EXIT, top]), {"reel_exit_post", "reel_pulley1"}))
    arc = np.array([(pc[0] + P1_R * math.sin(t), pc[1], pc[2] + P1_R * math.cos(t))
                    for t in np.linspace(0.0, math.pi / 2, 7)])
    spans.append((arc, {"reel_pulley1"}))
    p2 = p2_centre()
    in2 = np.array((p2[0] + P2_R, p2[1], p2[2]))
    spans.append((np.array([arc[-1], in2]), {"reel_pulley1", "reel_pulley2"}))
    arc2 = np.array([(p2[0] + P2_R * math.cos(t), p2[1], p2[2] - P2_R * math.sin(t))
                     for t in np.linspace(0.0, math.pi / 2, 7)])
    spans.append((arc2, {"reel_pulley2"}))
    spans.append((np.array([arc2[-1], arc2[-1] + (-RUN_CHECK, 0.0, 0.0)]), {"reel_pulley2"}))
    return spans


RUN_CHECK = 30.0              # length of the stub after pulley 2 checked against my own parts


def line_path(P, S):
    """Tether polyline in WORLD mm for state S (cad-infra contract): storage reel pack -> guide eye -> capstan turns
    and cup groove (S["wraps"]) -> ring -> exit eye -> pulley 1 -> along world +Y -> pulley 2 -> into the y 119 plane
    -> round the bottom of cad-launch's pivot pulley to launch.line_entry(P, S). cad-launch continues from there."""
    d = _d(P)
    stored = max(0.0, d["line_len_m"] - _num(S, "line_out_m"))
    rp = reel_calc.pack_r(d, stored)
    w = _num(S, "wraps")
    if 0.0 < w < 0.25:              # the turn model starts after the first quarter turn; show the launch path before
        w = 0.0
    pts = []
    for span, _ in _line_points(P, w, pack_r=rp):
        for p in span:
            if not pts or np.linalg.norm(np.asarray(p) - pts[-1]) > 1e-6:
                pts.append(np.asarray(p, float))
    sp = _spec(P)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = [np.asarray(a, float) for a in sp["axes"]]
    M[:3, 3] = sp["origin"]
    pts = pts[:-1]                                           # drop the checking stub after pulley 2
    W = [np.asarray((M @ np.r_[p, 1.0])[:3]) for p in pts]
    W += _to_pivot(P, S, W[-1])
    return [tuple(float(v) for v in p) for p in W]


PIV_R = 6.5                   # cad-launch's pivot pulley line radius


def _pivot(P, S):
    """(entry point, pulley centre, rail direction u) of cad-launch's pivot pulley, world. Uses launch.line_entry
    when the launch module is present, else the coordinator's numbers (centre (44 sin p, 119, -44 cos p))."""
    p = math.radians(_num(S, "aim_deg", 6.0))
    u = np.array((math.cos(p), 0.0, math.sin(p)))
    try:
        from cad.parts import launch as _L
        e = np.asarray(_L.line_entry(P, S), float)
        return e, e + PIV_R * u, u
    except Exception:
        c = np.array((44.0 * math.sin(p), P2_WORLD_Y, -44.0 * math.cos(p)))
        return c - PIV_R * u, c, u


def _to_pivot(P, S, f):
    """From pulley 2's exit f: tangent onto the bottom of the pivot pulley, then round its bottom to the entry point
    (its rear side), all in the y 119 plane."""
    e, c, u = _pivot(P, S)
    fx, fz = f[0] - c[0], f[2] - c[2]
    d = math.hypot(fx, fz)
    if d <= PIV_R + 1e-6:
        return [e]
    a0 = math.atan2(fz, fx)
    b = math.acos(PIV_R / d)
    cands = [a0 + b, a0 - b]
    tz = [c[2] + PIV_R * math.sin(a) for a in cands]
    at = cands[int(np.argmin(tz))]                           # tangent on the lower side
    ae = math.atan2(e[2] - c[2], e[0] - c[0])
    # go from the tangent point round the bottom (decreasing angle through -90 deg) to the entry
    while ae > at:
        ae -= 2 * math.pi
    out = []
    for a in np.linspace(at, ae, 9):
        out.append(np.array((c[0] + PIV_R * math.cos(a), c[1], c[2] + PIV_R * math.sin(a))))
    return out


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
        wraps_list = [0.0] + [1.0 + k / 24.0 for k in range(24)] + [float(WRAPS)]
    fails = []
    for w in wraps_list:
        S = dict(wraps=w, mode="REEL")
        sol = local(S) if callable(local) else _local_posed(P, S)
        for pts, allow in _line_points(P, w):
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
                ends = (6.0 if np.linalg.norm(pts[0] - GUIDE) < 1e-6 or np.linalg.norm(pts[0] - EXIT) < 1e-6 else 2.5,
                        6.0 if np.linalg.norm(pts[-1] - GUIDE) < 1e-6 or np.linalg.norm(pts[-1] - EXIT) < 1e-6 else 2.5)
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
    return line_check(P) + main_axis_check(P)


def steps(P):
    """Assembly steps, in order, plain language, hardware named per step."""
    return [
        dict(title="Screw the reel wall to the deck",
             text="Stand the reel wall on the deck at its marked place. Put eight M3 screws down through the "
                  "counterbored foot holes into the deck inserts and tighten. Do this first: the storage reel and "
                  "the gears later cover some of the holes.",
             parts=["reel_wall"]),
        dict(title="Sprag hub and sprag (CSK20PP)",
             text="Push the sprag hub's round spigot into the big hole in the wall from the front, flange flat on "
                  "the wall, and screw it on from the front with four M3 x 8 screws (they tap into the wall). Find "
                  "the key slot on the outside of the CSK20PP and line it up with the printed rib at the top of the "
                  "hub's seat. Check the arrow on the bearing: seen from the front, the inner ring must turn freely "
                  "counter-clockwise and lock clockwise. Press it in flat until it sits on the lip.",
             parts=["reel_sprag_hub", "reel_hub_screws", "reel_sprag"]),
        dict(title="Capstan tube",
             text="Slide the capstan tube in from the FRONT, D-flat end first, its key rib lined up with the key "
                  "slot inside the sprag. Push until the collar sits on the sprag. It turns one way only "
                  "(counter-clockwise seen from the front).",
             parts=["reel_c"]),
        dict(title="Bus gear, swing arm and swing idler",
             text="From behind the wall slide the 46-tooth bus gear onto the tube, gear face first, hub toward you. "
                  "Slip the swing arm's ring over the bus gear's hub, idler pin pointing at the wall, and put the "
                  "30-tooth swing idler on the pin. The idler is held between the arm and the wall. The arm swings "
                  "12 degrees between its two stop pins.",
             parts=["reel_bus", "reel_swing_arm", "reel_gi"]),
        dict(title="Capstan gear",
             text="Slide the 22-tooth capstan gear onto the D-flat behind the bus gear, hub toward the bus gear, and "
                  "snap the large C-clip into the groove behind it. The bus gear must still turn freely.",
             parts=["reel_gc", "reel_c_clip"]),
        dict(title="Capstan drum",
             text="Slide the green drum onto the front of the tube, flange first, key slot on the rib, until it "
                  "sits on the collar.",
             parts=["reel_drum"]),
        dict(title="Cup shaft and wrap cup",
             text="Push the cup shaft through the tube from the FRONT, plain end first (the neck end stays in front), "
                  "until its back end comes out "
                  "behind the capstan gear. Slide the wrap cup onto the front end (flat in the bore onto the flat of "
                  "the shaft, ring at the top), over the drum. Push Harry's small C-clip sideways onto the neck in "
                  "front of the cup.",
             parts=["reel_k", "reel_cup", "reel_cup_clip"]),
        dict(title="Cup gear",
             text="From behind, slide the 24-tooth cup gear onto the D-flat at the back of the cup shaft, hub "
                  "forward, and snap its C-clip on. The cup must turn freely both ways.",
             parts=["reel_gk", "reel_k_clip"]),
        dict(title="Storage reel",
             text="Press one 608 bearing into each end of the reel body. Push the Dyneema end through the small hole in the "
                  "core near the back flange into the open pocket, tie a stopper knot (figure eight) and pull it tight. Lay the flat flange disc on the front, its slots on "
                  "the three lugs, raised ring outward. Slide the reel onto the reel shaft from the shaft's thin end "
                  "until the disc meets the shaft's round head. Slide the thrust ring onto the shaft behind the "
                  "reel, small boss toward the bearing.",
             parts=["reel_spool", "reel_spool_flange", "reel_brg_1", "reel_brg_2", "reel_s", "reel_t"]),
        dict(title="Brake crank, rocker and bar",
             text="Grease the crank boss low on the left of the wall front. Slide the black bell crank onto it, "
                  "the long arm into the recess pointing right, the short arm up. Put the printed washer on the boss "
                  "and screw the M3 x 10 into the boss top: the crank must swing freely. Hang the yellow rocker "
                  "between the two ears at the top with an M3 x 25 screw and nut (snug, it must swing); its round "
                  "hole sits over the small boss and its leg hangs in front of the crank's long arm. Screw two "
                  "M3 x 8 screws into the rocker's two bosses from behind, until their tips stand 0.5 mm proud of "
                  "the front. Lay the brake bar in the two U guides, left end against the crank's short arm, nose "
                  "toward the drum. Close each guide with its cap and two M3 x 16 screws. Push the nose: the crank "
                  "swings and the rocker leg moves forward; let go and it all springs back.",
             parts=["reel_crank", "reel_crank_washer", "reel_crank_screw", "reel_rocker", "reel_rocker_pin",
                    "reel_trim_screws", "reel_bar", "reel_bar_cap_1", "reel_bar_cap_2", "reel_bar_screws"]),
        dict(title="Reel shaft into the wall",
             text="Feed the reel shaft's thin end through the rocker's hole and the wall bushing from the front. "
                  "From behind, slide the 20-tooth reel gear onto the D-flat, hub toward the wall, and snap its "
                  "C-clip on. Swing the arm toward the reel gear (to its stop): the idler meshes it. Spin the reel "
                  "by hand: it should turn with a light, even drag. Set LIGHT (after the line is wound on, drum at "
                  "LAUNCH): turn the reel until the reel gear's two small holes line up with the wall holes, reach "
                  "through with a 2.5 mm hex key and turn both trim screws the same amount until a 50 g weight hung "
                  "on the line end just pulls line off slowly (0.5 N). 1/8 turn changes it by about 0.4 N of squeeze.",
             parts=["reel_gr", "reel_s_clip"]),
        dict(title="SW_ARM contact",
             text="Battery unplugged. Build the printed contact (see the contact switch page): nut pressed in the base "
                  "trap, lever screwed on, M3 x 10 contact screw with its jam nut in the lever. Screw the switch base to "
                  "the bracket, and the bracket to the back of the wall below the gears with two M3 x 8. Turn the cup "
                  "to home (ring at the top): the bump on the cup gear presses the lever; set the contact screw so it "
                  "just closes there and opens 15 degrees either side. Lock the jam nut.",
             parts=["reel_sw_bracket", "reel_sw_base", "reel_sw_lever", "reel_sw_hw"]),
        dict(title="Pulley 2",
             text="Screw the pulley 2 post to the deck beside the drum with four M3 screws. Press a 693ZZ into "
                  "pulley 2, set it between the post's cheeks and push the M3 x 20 axle down through (nyloc nut "
                  "under). The line from pulley 1 runs along the base to it and leaves toward the rail pivot pulley.",
             parts=["reel_p2_post", "reel_pulley2", "reel_p2_brg", "reel_p2_axle"]),
        dict(title="Shroud",
             text="Put the shroud round the reel, its flange on the wall, opening at the top right, and screw it on "
                  "with three M3 x 8 screws into the wall. The reel must not touch it anywhere.",
             parts=["reel_shroud"]),
        dict(title="Exit eye post and line",
             text="Stand the exit eye post on the pad at the front of the wall foot and screw it down with four M3 "
                  "screws into the deck. Wind the line onto the reel (counter-clockwise seen from the front, line over "
                  "the top), thread it through the guide eye, over the top of the cup, through the cup ring from the "
                  "back, then through the exit eye. Press the 693ZZ into pulley 1, set the pulley between the fork "
                  "cheeks with the line in its groove and push the M3 x 20 axle down through (nyloc nut under). Lead "
                  "the line on toward the rail pivot pulley.",
             parts=["reel_exit_post", "reel_pulley1", "reel_p1_brg", "reel_p1_axle"]),
    ]


# ------------------------------------------------------------------------------------------------ neighbours
WRAP_IDLER_R = (34.9, -35.2)  # cad-drum's 42T wrap idler axis in R (tip r 33), meshing my cup gear


def main_axis_keepouts(P):
    """Keep-out solids (world) of the main-axis stack and the drum, from params and the coordinator's list, with
    2 mm added. Each: (name, Manifold, reel parts that legitimately reach in (gear meshes))."""
    lay = (getattr(P, "LAYOUT", None) or {})
    drum_ax = lay.get("axes", {}).get("drum", {}).get("origin", (60.0, 0.0, -112.0))
    def cyl_y(r, y0, y1, x=0.0, z=0.0):
        return prim.cylY(r, y0, y1, x=x, z=z)
    sp = _spec(P)
    wi = (sp["origin"][0] + WRAP_IDLER_R[0], sp["origin"][2] + WRAP_IDLER_R[1])
    return [
        ("coupler", cyl_y(35.0 + 2, 0.5, 46.0), ("reel_bus",)),   # bus gear meshes cad-drum's (audit: 0 overlap)            # base-v1 1d668d6: r 35 in the wall plane
        ("bus_gear", cyl_y(39.0 + 2, 40.0, 48.0), ("reel_bus",)),
        ("reel_gear", cyl_y(52.0 + 2, 50.5, 62.5), ("reel_gc",)),
        ("collar", cyl_y(30.0 + 2, 65.0, 105.0), ()),
        ("dog_ring", cyl_y(31.0 + 2, 100.5, 111.0), ()),
        ("main_shaft", cyl_y(13.0 + 2, 0.5, 200.0), ()),
        ("drum", cyl_y(42.0 + 2, 1.0, 142.0, x=drum_ax[0], z=drum_ax[2]), ("reel_gk", "reel_bar", "reel_p2_axle")),   # bar nose on the cam; axle tip > 10 from the real drum
        ("drum_gear", cyl_y(48.6 + 2, 8.0, 16.0, x=drum_ax[0], z=drum_ax[2]), ("reel_bar",)),
        ("wrap_idler", cyl_y(33.0 + 2, 68.5, 76.5, x=wi[0], z=wi[1]), ("reel_gk",)),   # cad-drum's 42T idler
    ]


def main_axis_check(P, states=None):
    """Every reel part (world pose) against the main-axis keep-outs; returns failure strings."""
    from cad.lib import tf as _tf  # noqa: F401
    sp = _spec(P)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = [np.asarray(a, float) for a in sp["axes"]]
    M[:3, 3] = sp["origin"]
    fails = []
    for S in (states or [dict(), dict(wiper_dir=-1), dict(wraps=0.5)]):
        sol = _local_posed(P, S)
        for kname, ko, ok in main_axis_keepouts(P):
            for pname, s_ in sol.items():
                if pname in ok:
                    continue
                v = (prim.place(s_, M) ^ ko).volume()
                if v > 1e-3:
                    fails.append(f"{pname} enters the {kname} keep-out ({v:.1f} mm3)")
    return sorted(set(fails))
