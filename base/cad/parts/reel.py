"""GrappStar base, reel system (worker cad-reel), D50 layout. Module contract: base/cad/README.md.

Two groups:
- FRONT GEARS (frame R, on the reel wall under the main axis, as before): the take-up bus gear c1 (D15) with its
  forward-only swing idler and the switched slip clutch (rocker, bell crank and bar on cad-drum's brake cam), the cup
  gear meshing cad-drum's wrap idler (with SW_ARM on its track), and three take-offs: countershaft C (22T m2 driven
  by the REEL gear through a 41T idler), K2 (the cup drive, raised by a 1:1 riser from the cup shaft) and T2 (40T on
  the 40T clutch hub). Each ends in a bevel pair that turns the drive into a shaft along world X (lanes C, K, T).
- REEL UNIT (world frame, on the bolt-on deck extension behind the launcher, params REEL_UNIT): Harry's v7 capstan
  drum and wrap cup on a capstan axis along +X in the rail plane y 140 (cup and its AXIAL eyelet in front, ring at the
  bottom at home), sprag #2 on the capstan tube in P1, the rear trains (C plane, K plane), and the free storage spool
  (d40 / d62 x 20, axis along world Y) between two cheeks, driven by lane T through a bevel and a 45T pair.

The tether: spool pack top -> axial eyelet (straight, on the low-aim line) -> floor segment to cad-launch's one
693ZZ pulley at the rail rear -> along the rail to the dart (launch.line_path). In tow (6 deg) the floor segment is
straight; at higher pitch the rail-rear pulley drops and the line bends there and at the eyelet mouth (params
REEL_UNIT bends).

Reel frame R (front group): origin on the old capstan axis (now the cup shaft axis A0) at the front face of the reel
wall, +Z toward world -Y, +X forward, +Y up (params LAYOUT reel_frame_R). Senses about R +Z: bus gear under a FORWARD
wiper +, clutch input (reel gear) take-up +, cup WRAP -. Wiper FORWARD turns the main shaft - about R +Z.
Every shared number comes from params (LAYOUT, REEL, DRUM, REEL_UNIT); local geometry is named here.
"""
from __future__ import annotations

import math

import numpy as np
from manifold3d import Manifold
from shapely import affinity
from shapely.geometry import Point

from cad.lib import Part, Joint, Allow, Clearance, Mesh, prim, tf, fits, gears, bearings, shafts, clips, line, switch
from cad.parts import reel_calc, reel_v7 as v7


# ------------------------------------------------------------------------------------------------ spec numbers
def _spec(P):
    """Shared numbers from params with the as-built values as fallbacks."""
    lay = (getattr(P, "LAYOUT", None) if P is not None else None) or {}
    reel = (getattr(P, "REEL", None) if P is not None else None) or {}
    drum = (getattr(P, "DRUM", None) if P is not None else None) or {}
    ru = (getattr(P, "REEL_UNIT", None) if P is not None else None) or {}
    fr = lay.get("reel_frame_R", {})
    main = lay.get("axes", {}).get("main", {})
    org = tuple(fr.get("origin", (-18.661, 33.5, -69.643)))
    deck = lay.get("deck_top_z", -165.0)
    bus = drum.get("bus", {})
    ro = drum.get("reel_output", {})
    st = reel.get("storage", {})
    y0 = org[1]
    mo = main.get("origin", (0.0, 0.0, 0.0))
    lanes = ru.get("lanes", {})
    front = ru.get("front", {})
    return dict(
        origin=org, axes=(tuple(fr.get("x_axis", (1, 0, 0))), tuple(fr.get("y_axis", (0, 0, 1))),
                          tuple(fr.get("z_axis", (0, -1, 0)))),
        h_axis=org[2] - deck, deck=deck, main_xy=(mo[0] - org[0], mo[2] - org[2]),
        bus_m=bus.get("module", 1.5), bus_z=tuple(sorted(y0 - y for y in bus.get("plane_y", (40.0, 48.0)))),
        reel_m=ro.get("module", 2.0), reel_teeth=ro.get("teeth", 50),
        reel_z=tuple(sorted(y0 - y for y in ro.get("plane_y", (50.5, 62.5)))),
        wraps=reel.get("wraps", 7), core_d=st.get("core_d", 40.0), flange_d=st.get("flange_d", 62.0),
        width=st.get("width", 20.0), cup_m=reel.get("cup_gear", {}).get("module", 1.5),
        cup_teeth=reel.get("cup_gear", {}).get("teeth", 24), overdrive=bus.get("overdrive_total", 2.5),
        low_pitch=ru.get("low_pitch", 6.01), eyelet_u=ru.get("eyelet_u", -233.0), line_w=ru.get("line_w", 84.6),
        line_y=ru.get("line_y", 140.0),
        lane_c=tuple(lanes.get("c", (7.0, 7.0))), lane_k=tuple(lanes.get("k", (5.0, 36.0))),
        lane_t=tuple(lanes.get("t", (0.0, -49.09))),
        c_axis=tuple(front.get("c_axis", (-150.0, 7.0))), c_idler=front.get("c_idler_teeth", 41),
        k2_axis=tuple(front.get("k2_axis", (-100.0, 36.0))), t2_deg=front.get("t2_deg", 160.0),
    )


try:                                       # module constants follow the spec at import time
    from tools.params import load_params as _lp
    SP = _spec(_lp())
except Exception:                          # pragma: no cover
    SP = _spec(None)

A0 = np.array((SP["origin"][0], SP["origin"][2]))      # world (x, z) of the frame R axis (cup shaft K, c1)
Y0 = SP["origin"][1]                                   # world y of the wall front face (R Z = 0)


def W2R(x, z):
    """World (x, z) -> R (X, Y)."""
    return (x - A0[0], z - A0[1])


def YtoZ(y):
    """World y -> R Z."""
    return Y0 - y


# ------------------------------------------------------------------------------------------------ front group (R)
WALL_T = 6.0                  # wall Z -6 .. 0 (the bus plane starts at -6.5 behind it)
FOOT_T = 5.0
H_AXIS = SP["h_axis"]
Y_DECK = -H_AXIS
X_WALL = (-139.5, 30.0)       # old wall
X_WALL0 = -171.3              # D50: the wall runs back to world x -190 (T2, lane C tab)
Y_WALL_TOP = 40.0
Z_FOOT = (-40.0, 42.0)

Z_SPOOL = 12.0                # clutch hub rear face (rear 608 mouth); room for the rocker to bend 4 mm in front of it
HUB_L = 16.0                  # hub body: two 608 seats and a 2 mm lip
HUB_R = 16.0                  # hub body radius under the 28T teeth
FL_F = 1.6                    # front flange disc thickness (it takes the clutch squeeze)
X_REEL = -94.0                # clutch hub / reel shaft S axis (R Y 0)
ANNULUS = (20.0, 30.0, 0.3)   # clutch friction annulus on the front flange disc: r_in, r_out, raised (r_eff 25)
Z_D = Z_SPOOL + HUB_L + FL_F + ANNULUS[2]       # clutch disk D (head of S) rear face
D_T, D_R = 3.0, 31.0
Z_T = 7.0                     # thrust ring T rear face (the rocker's two M3 trim screw tips press it)
TRIM_R = 10.0                 # trim screws at X +-10 from the hub axis
HUBG_Z = (Z_SPOOL + 0.5, Z_SPOOL + 6.5)         # hub teeth 28T m1.5 and the T2 gear (world y 21 .. 15)
Z_HUBG = 28
M_T = 1.5

ROCKER_PIVOT = np.array((X_REEL, 37.0, 3.5))  # pin hole floor 1.3 above the plate's back
ROCKER_Z = (0.5, 3.5)         # spring plate (PPA-CF, t 3.0, reel_calc.rocker_section)
BAR_LUG = (19.2, 25.0, 4.0)   # bar end lug for crank arm A: Z from .. to at the end face, 45 deg chamfer below
BRAKE_STROKE = 4.0            # cad-drum's track 2 cam lift (radial, base r 30, follower at L angle 180 = R -X)
CAM_R0 = 30.0
BAR_GAP = 0.2                 # bar nose to the cam base circle at LIGHT
BAR_Z = (9.2, 15.2)           # bar thickness: rides the cam at world y 18.3 .. 24.3 (cam s 18 .. 29)
BAR_H = 6.0                   # bar height (R Y)
BAR_YL = -55.0                # left run under the hub
BAR_DOG = (-30.0, -10.0)      # dogleg up to the cam height between these X
LEG_X = (-98.5, -88.5)        # rocker leg (stiff, Z 0.5 .. 6), from the spring plate down
LEG_Y0, LEG_Z1 = -51.7, 6.0
CRANK_X, CRANK_Z = -107.6, 2.0                # crank pivot (axis R +Y)
CRANK_Y = (-59.6, -47.2)
CRANK_LA, CRANK_RT = 20.0, 2.5                # arm A: tip circle centre above the pivot, radius (bar lug contact)
CRANK_B, CRANK_RB = (14.1, -4.0), 1.5         # arm B: tip circle centre from the pivot (FIRM screw contact)
FIRM_SCREW = (-93.5, -48.7)   # M3 x 10 through the rocker leg (R X, Y): its tip on arm B; lost motion g sets FIRM
FIRM_GAP = 0.64               # nominal lost motion g at LIGHT
LIGHT_PAD = (-34.0, -30.0)    # 0.45 mm pad on the wall front under the leg: the rocker rests on it at LIGHT
CRANK_HUB_R, CRANK_BOSS_R = 5.5, 3.5
CRANK_COL_TOP, CRANK_BOSS_TOP = -59.9, -46.9  # column under the crank, pivot boss top (washer + M3 x 20 on it)
WASHER_T, WASHER_R = 1.2, 6.0
POCKET_D = 4.0                # wall front recess for arm B and the hub
BAR_GUIDES = ((-71.5, BAR_YL), (17.0, None))  # (x, y) of the two U guides (y None = right run)
GUIDE_W, GUIDE_FRONT, GUIDE_SCREW_Z = 10.0, 20.0, 17.2
MAIN_CLEAR_R = 37.5           # wall cut round the main axis (cad-drum's coupler reaches r 35 in the wall plane)
EXT_CLEAR_R = 46.0            # D50 wall extension keeps this far from the main axis (coupler r 40)

# gear planes behind the wall (from params)
BUS_Z = (SP["bus_z"][1] - 6.0, SP["bus_z"][1])  # my bus gear c1 (face 6) inside cad-drum's bus plane (-14.5..-6.5)
HUB1_Z = (BUS_Z[0] - 3.5, BUS_Z[0])             # c1 rear hub: the swing arm rides on it
ARM_Z = (HUB1_Z[0] + 0.3, BUS_Z[0] - 0.3)       # swing arm plate
GC_Z = SP["reel_z"]                             # REEL gear plane (-29..-17): C's 22T and its idler
CUPGEAR_Z = (-43.0, -35.0)                      # cup gear plane (world y 68.5..76.5)
M_CAP, Z_MAIN, Z_CAPG = SP["reel_m"], SP["reel_teeth"], 22
Z_IC = SP["c_idler"]
M_BUS, Z_BUSMAIN, Z_C1, Z_IDL = SP["bus_m"], 50, 46, 30
Z_REELG = int(round(Z_BUSMAIN / SP["overdrive"]))           # 20: clutch disk = 50 / 20 = 2.5 x wiper
M_CUPG, Z_CUPG = SP["cup_m"], SP["cup_teeth"]
BUS_PHASE_D = 3.78           # deg, c1 = (50/46) drum_main + this (numeric, against cad-drum's bus gear)
GK_PHASE = 8.67              # deg, cup gear teeth in the cup frame
BUS_PHASE = 1.37             # fallback without cad-drum's module
SWING_OUT_DEG = 12.0          # swing arm travel from engaged to parked (tips clear by about 2 mm)
WRAPS = SP["wraps"]
HUB1_R = 13.0

SLEEVE_D, K_D, S_D = 20.0, 10.0, 8.0
C_D = SLEEVE_D                                  # c1's bore (it turned on the old capstan tube)
SLEEVE_Z = (HUB1_Z[0] - 0.3 - clips.groove_dims(20.0)['width'] - 1.65, 0.0)   # stationary sleeve at A0, clip behind c1's hub
SLEEVE_FL = (17.0, 2.5)                         # flange on the wall front (r, thickness)
K_Z0 = CUPGEAR_Z[0] - 0.3 - clips.groove_dims(K_D)["width"] - 1.6
S_Z0 = BUS_Z[0] - 0.3 - clips.groove_dims(S_D)["width"] - 1.0

# K riser (cup drive up to K2), world y 23.5 .. 28.5 in front of the wall
RISER_Z = (YtoZ(28.5), YtoZ(23.5))              # R Z 5 .. 10
M_R, Z_R = 1.5, (30, 34, 30, 30)                # cup shaft gear, idler 1, idler 2, K2 gear (1:1)
K2W = np.array(SP["k2_axis"])
CW = np.array(SP["c_axis"])
I1_HINT = np.array((-61.03, -46.88))            # world, from the layout search (margins >= 7 mm, I2 clear of the I_C axle)
I2_HINT = np.array((-73.34, -0.38))
LANE_C, LANE_K, LANE_T = SP["lane_c"], SP["lane_k"], SP["lane_t"]
# bevels (1:1, 45 deg): (module, teeth, face width along the cone, bore)
BEV_C = (2.0, 20, 6.0)
BEV_K = (1.5, 18, 5.0)
BEV_T = (1.0, 20, 4.0)
CS_D, LC_D, LK_D, LT_D = 16.0, 12.0, 8.0, 8.0   # C countershaft, lanes C, K, T


def _bev_info(b):
    m, z, w = b
    r0 = m * z / 2
    return dict(r0=r0, h=w * math.cos(math.radians(45.0)), tip=r0 + m)


def _cd(m, z1, z2):
    return gears.center_distance(m, z1, z2)


def _circ_x(c1, r1, c2, r2, pick):
    """Intersection of two circles (2D); pick(a, b) chooses one of the two points."""
    c1, c2 = np.asarray(c1, float), np.asarray(c2, float)
    d = np.linalg.norm(c2 - c1)
    ex = (c2 - c1) / d
    ey = np.array((-ex[1], ex[0]))
    x = (r1 ** 2 - r2 ** 2 + d ** 2) / (2 * d)
    y = math.sqrt(max(r1 ** 2 - x ** 2, 0.0))
    return pick(c1 + x * ex + y * ey, c1 + x * ex - y * ey)


def front_layout():
    """World (x, z) of the front take-off axes: I1, I2 (K riser idlers), K2, C, I_C (REEL idler), T2."""
    l1 = _cd(M_R, Z_R[0], Z_R[1])
    i1 = A0 + l1 * (I1_HINT - A0) / np.linalg.norm(I1_HINT - A0)
    i2 = _circ_x(i1, _cd(M_R, Z_R[1], Z_R[2]), K2W, _cd(M_R, Z_R[2], Z_R[3]),
                 lambda a, b: a if np.linalg.norm(a - I2_HINT) < np.linalg.norm(b - I2_HINT) else b)
    main = np.array((0.0, 0.0))
    ic = _circ_x(main, _cd(M_CAP, Z_MAIN, Z_IC), CW, _cd(M_CAP, Z_IC, Z_CAPG), lambda a, b: a if a[1] < b[1] else b)
    s = A0 + np.array((X_REEL, 0.0))
    t2 = s + _cd(M_T, Z_HUBG, Z_HUBG) * np.array((math.cos(math.radians(SP["t2_deg"])), math.sin(math.radians(SP["t2_deg"]))))
    return dict(i1=i1, i2=i2, k2=K2W, c=CW, ic=ic, t2=t2, s=s)


FL = front_layout()


def R_of(key):
    p = FL[key]
    return np.array(W2R(p[0], p[1]))


# bevel planes (R Z): Y-gear large end, teeth toward the apex (R +Z, world -y)
BC_Z = YtoZ(LANE_C[0] + _bev_info(BEV_C)["r0"])     # C: large end at world y lane + r0
BK_Z = YtoZ(LANE_K[0] + _bev_info(BEV_K)["r0"])
BT_Z = YtoZ(LANE_T[0] + _bev_info(BEV_T)["r0"])

# ------------------------------------------------------------------------------------------------ reel unit (world)
P_LOW = math.radians(SP["low_pitch"])
_c, _s = math.cos(P_LOW), math.sin(P_LOW)
EYE = np.array((SP["eyelet_u"] * _c - SP["line_w"] * _s, SP["line_y"], SP["eyelet_u"] * _s + SP["line_w"] * _c))
CAP_YZ = np.array((SP["line_y"], EYE[2] + v7.RING_AT))     # capstan axis (y, z), ring at the cup bottom
X_RIM = EYE[0] - v7.CUP_D - 2 * v7.RING_L                  # cup rim plane (cup frame origin)
X_D0 = X_RIM - (v7.FLANGE_H + v7.DL - 7.0)                 # drum flange back face
COLLAR_CONE = 4.0
X_P1 = (X_D0 - 1.0 - 6.0, X_D0 - 1.0)                      # P1 plate (front face = sprag seat mouth, 1 mm collar flat)
X_P1B = X_P1[1] - 14.0                                     # P1 bushing bosses' rear face
X_CPL = (X_P1B - 15.0, X_P1B - 7.0)                        # C plane (m2, face 8); the I_c hub reaches 6 forward
SPRAG_X = (X_CPL[0], X_CPL[0] + bearings.CSK20_W)          # sprag #2 inside the I_c idler (backstops the capstan train)
X_KPL = (X_CPL[0] - 4.5 - 5.0, X_CPL[0] - 4.5)             # K plane (m1.5, face 5); 4.5 gap for the C-plane clips
X_P2 = (X_KPL[0] - 1.0 - 6.0, X_KPL[0] - 1.0)              # P2 plate
M_RC, Z_RC = 2.0, (20, 58, 20)                             # C plane: lane gear, idler, capstan gear
M_RK, Z_RK = 1.5, (20, 76, 20)                             # K plane: lane gear, idler, cup gear
SPOOL_PACK = 27.0                                          # design pack radius (about 2 m out)
M_TS, Z_TS, Z_TI = 1.5, 16, 34                             # T3 16T, idler 34T, spool shaft 16T (light: it all spins in payout)
Z_SS = 18.51                                               # spool axis height (params REEL_UNIT spool)
CORE_R2 = SP["core_d"] / 2
FLANGE_R2 = SP["flange_d"] / 2
PACK_Y = (SP["line_y"] - SP["width"] / 2, SP["line_y"] + SP["width"] / 2)
FL2_T = 1.6                                                # spool flange thickness
CHEEK_I, CHEEK_O = (112.0, 120.0), (160.0, 168.0)          # reel unit cheeks (world y)
T_BR_Y = (21.0, 27.0)                                      # T bracket (T3 and the spool shaft)
TSP_Y = (14.0, 20.0)                                       # rear take-up gear plane
X_CHEEK1 = X_P2[0]                                         # cheeks start behind P2


def spool_x():
    """Spool axis world x: the pack top (r SPOOL_PACK + line) is tangent to the low-aim line through the eyelet."""
    R = SPOOL_PACK + 0.5
    tn = math.tan(P_LOW)
    return EYE[0] - (EYE[2] - Z_SS - R * _c) / tn + R * _s


X_SS = spool_x()
X_CHEEK0 = X_SS - FLANGE_R2 - 4.0                          # cheek rear edge


def rear_layout():
    """Idler centres (y, z) of the two rear trains."""
    lc, lk, cap = np.array(LANE_C), np.array(LANE_K), CAP_YZ
    pc = _circ_x(lc, _cd(M_RC, Z_RC[0], Z_RC[1]), cap, _cd(M_RC, Z_RC[1], Z_RC[2]), lambda a, b: a if a[1] > b[1] else b)
    ti = _circ_x((X_SS, LANE_T[1]), _cd(M_TS, Z_TS, Z_TI), (X_SS, Z_SS), _cd(M_TS, Z_TI, Z_TS), lambda a, b: a if a[0] < b[0] else b)
    pk = _circ_x(lk, _cd(M_RK, Z_RK[0], Z_RK[1]), cap, _cd(M_RK, Z_RK[1], Z_RK[2]), lambda a, b: a if a[1] > b[1] else b)
    return dict(pc=pc, pk=pk, ti=ti)


RL = rear_layout()
FX = np.array([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])   # local Z = world X
FYN = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])  # local Z = world -Y
FYP = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])  # local Z = world +Y

COL = dict(wall="#b9c2cc", ppa="#3a3f45", gear="#d97a00", drum="#2e9c86", cup="#7f77dd", spool="#7f77dd",
           clip="#9a94e8", brg="#8a8f96", rocker="#e0b040", post="#4a5058", bevel="#c9a227", frame="#a7b0ba")


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


ARM_W = 16.0
STOP_R, STOP_RAD = 2.5, 40.0
STOP_OFF = 27.0               # stop pins sit this far (deg) off the idler: outside its tip circle where they cross it


def stop_angles():
    """Swing-arm stop pins at r STOP_RAD: (engaged stop, parked stop), deg about the capstan axis."""
    return (arm_deg() + STOP_OFF, arm_deg() - SWING_OUT_DEG - STOP_OFF)


FOOT_FRONT_X1 = -57.0         # front foot ends here (cad-frame's cheek r stands at world x > -75)
FOOT_REAR_X1 = 3.0            # rear strip ends here (cad-drum's idler bracket at R X 4.9 .. 46.9 behind the wall)
MOUNT_HOLES = [(-120.0, 14.0), (-120.0, 36.0), (-64.0, 36.0), (-64.0, 14.0), (-120.0, -32.0), (-60.0, -32.0),
               (-8.0, -32.0), (-160.0, 14.0), (-160.0, 36.0)]
SLEEVE_SCREW_R = 13.5


def _ext_outline():
    """D50 wall extension (R X, R Y): hull of the take-off bosses and the tab roots, minus the main-axis clearance and
    cad-frame's guard hood (world x > -60, z > 42)."""
    pts = []
    def circ(key, r):
        c = R_of(key)
        return [(c[0] + r * math.cos(t), c[1] + r * math.sin(t)) for t in np.linspace(0, 2 * math.pi, 40)]
    pts += circ("t2", 10.0) + circ("c", 14.0) + circ("k2", 10.0) + circ("ic", 9.0) + circ("i2", 8.0)
    for x, z in ((-190.0, -100.0), (-190.0, 24.0), (-137.0, 48.0), (-127.0, 48.0)):
        pts.append(W2R(x, z))
    pts.append((X_WALL[0], Y_WALL_TOP - 1.0))
    pts.append((-40.0, Y_WALL_TOP - 1.0))
    from shapely.geometry import MultiPoint, box as sbox
    h = MultiPoint(pts).convex_hull
    mx, my = SP["main_xy"]
    h = h.difference(Point(mx, my).buffer(EXT_CLEAR_R, 96))
    hx, hz = W2R(-60.0, 42.0)
    h = h.difference(sbox(hx, hz, 1e3, 1e3))
    if h.geom_type != "Polygon":
        h = max(h.geoms, key=lambda g: g.area)
    return h


def _wall():
    x0, x1 = X_WALL
    foot = prim.union([prim.box(X_WALL0, FOOT_FRONT_X1, Y_DECK, Y_DECK + FOOT_T, Z_FOOT[0], Z_FOOT[1]),
                       prim.box(X_WALL0, FOOT_REAR_X1, Y_DECK, Y_DECK + FOOT_T, Z_FOOT[0], 0.0)])
    wall = prim.box(X_WALL0, x1, Y_DECK, Y_WALL_TOP, -WALL_T, 0.0)
    ext = prim.extrude(prim.shapely_cs(_ext_outline()), WALL_T, z=-WALL_T)
    px, py, pz = ROCKER_PIVOT
    ears = [prim.box(px + s * 5.4, px + s * 9.4, py - 6, py + 6, -0.01, 6.5) for s in (-1, 1)]
    sboss = prim.cyl(5.5, Z_T - 0.3, x=X_REEL, z=-0.01)        # S bushing through the rocker's hole
    stops = [prim.cyl(STOP_R, -ARM_Z[0] - WALL_T + 0.01, x=STOP_RAD * math.cos(math.radians(a)),
                      y=STOP_RAD * math.sin(math.radians(a)), z=ARM_Z[0]) for a in stop_angles()]
    # back bosses for the long take-off shafts (bus plane free there)
    bosses = []
    for key, r, z0 in (("c", 13.0, GC_Z[1] + 0.5), ("k2", 9.0, -14.0), ("ic", 8.0, GC_Z[1] + 0.5), ("t2", 9.0, -12.0)):
        c = R_of(key)
        bosses.append(prim.cyl(r, -WALL_T - z0 + 0.01, x=c[0], y=c[1], z=z0))
    yg = Y_DECK + FOOT_T - 0.01
    gus = []
    for x in (-126.0, -64.0, -160.0):
        gus.append(_gusset(x, x + 4.0, 0.0, Z_FOOT[1] - 2.0, yg, min(40.0, -44.0 - yg)))
    for x in (-126.0, -64.0, -20.0, -160.0):
        gus.append(_gusset(x, x + 4.0, -WALL_T, Z_FOOT[0] + 2.0, yg, min(40.0, -50.0 - yg)))
    body = prim.union([foot, wall, ext, sboss] + ears + stops + bosses + gus)
    cut = [prim.cyl((SLEEVE_D + fits.SHAFT_IN_BORE) / 2, WALL_T + 2, z=-WALL_T - 1)]
    cut += [prim.box(SLEEVE_D / 2 - 1.5, SLEEVE_D / 2 + 3.0, -3.0, 3.0, -WALL_T - 1, 1.0)]     # key slot for the sleeve rib
    for a in (90.0, 210.0, 330.0):
        cut.append(prim.cyl(fits.M3_TAP_D / 2, WALL_T + 0.02, x=SLEEVE_SCREW_R * math.cos(math.radians(a)),
                            y=SLEEVE_SCREW_R * math.sin(math.radians(a)), z=-WALL_T - 0.01))
    cut.append(prim.cyl((S_D + fits.SHAFT_IN_BORE + fits.HOLE_HORIZ_EXTRA) / 2, WALL_T + Z_T + 2, x=X_REEL,
                        z=-WALL_T - 1))
    for key, d, z0 in (("c", CS_D, GC_Z[1] - 1.0), ("k2", LK_D, -16.0), ("t2", LT_D, -14.0)):
        c = R_of(key)
        cut.append(prim.cyl((d + fits.SHAFT_IN_BORE + fits.HOLE_HORIZ_EXTRA) / 2, 40.0, x=c[0], y=c[1], z=z0))
    for key, z0 in (("ic", GC_Z[1] - 1.0), ("i1", -WALL_T - 1.0), ("i2", -WALL_T - 1.0)):
        c = R_of(key)            # idler axles: press fit (no bore extra)
        cut.append(prim.cyl(AXLE_D / 2 - 0.05, 40.0, x=c[0], y=c[1], z=z0))
    cut += _brake_wall_cuts()
    for s_ in (-1, 1):                             # hex key access to the rocker's trim screws
        cut.append(prim.cyl(3.6, WALL_T + 2, x=X_REEL + s_ * TRIM_R, z=-WALL_T - 1))
    cut.append(prim.cylX(fits.M3_CLEAR_D / 2, px - 12, px + 12, y=py, z=pz))
    for sx in (-12.0, 12.0):                       # SW_ARM bracket screws tap into the wall rear face
        cut.append(prim.cyl(fits.M3_TAP_D / 2, 5.0, x=sx, y=_sw_bracket()[1] - 13.0, z=-WALL_T - 0.01))
    for (tx, tz) in TAB_SCREWS:                    # lane tab flanges: M3 tapped into the wall front
        x, y = W2R(tx, tz)
        cut.append(prim.cyl(fits.M3_TAP_D / 2, WALL_T + 0.02, x=x, y=y, z=-WALL_T - 0.01))
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
    adds.append(prim.box(CRANK_X - 4.0, CRANK_X + 4.0, yf, CRANK_COL_TOP, -0.01, CRANK_Z + 4.0))
    adds.append(prim.box(LEG_X[0], LEG_X[1], LIGHT_PAD[0], LIGHT_PAD[1], -0.01, ROCKER_Z[0] - 0.05))   # LIGHT stop
    adds.append(prim.cylY(CRANK_BOSS_R, CRANK_COL_TOP - 0.01, CRANK_BOSS_TOP, x=CRANK_X, z=CRANK_Z))
    cuts.append(prim.cylY(fits.M3_TAP_D / 2, CRANK_BOSS_TOP - 19.0, CRANK_BOSS_TOP + 1.0, x=CRANK_X, z=CRANK_Z))
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
    lug = prim.place(prim.extrude(lug_cs, 2 * h - 0.04), tf.T(0, yl + h - 0.02, 0) @ tf.Rx(90))   # inset: no coplanar sides
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
    z0, z1 = -CRANK_Z - 2.9, -CRANK_Z - 1.0                  # arm B strip (R Z -2.9 .. -1.0)
    cs = (prim.circle_cs(CRANK_HUB_R) + prim.poly_cs([(-2.2, 0.0), (2.2, 0.0), (1.2, 7.0), (1.2, CRANK_LA), (-1.2, CRANK_LA), (-2.2, 7.0)])
          + prim.circle_cs(CRANK_RT, 0.0, CRANK_LA)
          + prim.poly_cs([(0.0, z0), (bx, z0), (bx, z1), (0.0, z1)]) + prim.circle_cs(CRANK_RB, bx, bz))
    s = prim.extrude(cs, CRANK_Y[1] - CRANK_Y[0])            # profile (u = R x, v = R z), extruded along w
    s = prim.place(s, tf.T(0, CRANK_Y[1], 0) @ tf.Rx(90))     # (u, v, w) -> (u, -w, v): R y = y1 - w
    return s - prim.cylY(CRANK_BOSS_R + fits.HOLE_CLEAR / 2, CRANK_Y[0] - 1, CRANK_Y[1] + 1)   # running fit


def _washer():
    """Printed washer (PPA-CF) on the crank boss top, under the M3 x 10 head; it keeps the crank on the boss."""
    return prim.tube(WASHER_R, fits.M3_CLEAR_D / 2, WASHER_T)


def _sleeve():
    """Stationary sleeve at A0 (PPA-CF), R frame: d20 through the wall (key rib in the wall slot) and c1, flange on the
    wall front with 3 x M3, clip groove behind c1's hub; bore 10.3 for the cup shaft K."""
    z0, z1 = SLEEVE_Z
    body = prim.union([prim.cyl(SLEEVE_D / 2, z1 - z0, z=z0), prim.cyl(SLEEVE_FL[0], SLEEVE_FL[1], z=0.0),
                       prim.box(SLEEVE_D / 2 - 1.6, SLEEVE_D / 2 + 2.6, -2.6, 2.6, -WALL_T, 0.01)])
    g = clips.groove_dims(SLEEVE_D)
    cut = [prim.cyl((K_D + fits.SHAFT_IN_BORE) / 2, z1 - z0 + 10, z=z0 - 1),
           clips.clip_groove_cutter(SLEEVE_D, z=HUB1_Z[0] - 0.3 - g['width'])]
    for a in (90.0, 210.0, 330.0):
        x, y = SLEEVE_SCREW_R * math.cos(math.radians(a)), SLEEVE_SCREW_R * math.sin(math.radians(a))
        cut.append(prim.cyl(fits.M3_CLEAR_D / 2, 10.0, x=x, y=y, z=-1.0))
    return body - prim.union(cut)


K_FRONT = RISER_Z[1] + 2.2                     # cup shaft K front end (clip in front of the riser gear)


def _k_shaft():
    """Cup shaft K (PPA-CF): 10 mm from behind the cup gear through the sleeve to the riser gear in front of the wall;
    D flats (+X) under the cup gear and under the riser gear, clip grooves behind the cup gear and in front of the riser
    gear."""
    z0, zf = K_Z0, K_FRONT
    body = prim.cyl(K_D / 2, zf - z0, z=z0)
    flat_g = prim.box(K_D / 2 - 1.2, K_D, -K_D, K_D, z0 - 1, CUPGEAR_Z[1] + 0.5)
    flat_r = prim.box(K_D / 2 - 1.2, K_D, -K_D, K_D, RISER_Z[0] - 0.5, zf + 1)
    gd = clips.groove_dims(K_D)
    grooves = [clips.clip_groove_cutter(K_D, z=CUPGEAR_Z[0] - 0.3 - gd["width"]),
               clips.clip_groove_cutter(K_D, z=RISER_Z[1] + 0.3)]
    return body - prim.union([flat_g, flat_r] + grooves)


def _s_shaft():
    """Reel shaft S (PPA-CF): 8 mm, D flat (+X) under the reel gear, clip groove behind it, head = clutch disk D
    (r 31 x 3) at the front."""
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


def _hub():
    """Clutch hub (PPA-CF), local frame: axis Z, rear face z = 0. Two 608 seats, 40T m1.5 teeth (T2 take-off) on the
    rear half, 3 drive lugs for the friction disc at the front. It replaces the old storage reel: the friction disc on
    its front takes the clutch squeeze, the spool in the reel unit follows it 1:1."""
    L = HUB_L
    body = prim.cyl(HUB_R, L)
    g = gears.spur_gear(M_T, Z_HUBG, HUBG_Z[1] - HUBG_Z[0]).translate((0, 0, HUBG_Z[0] - Z_SPOOL))
    lugs = [prim.place(prim.box(HUB_R - 4.0, HUB_R - 1.0, -3.0, 3.0, L - 0.01, L + 0.9), tf.Rz(a))
            for a in (0.0, 120.0, 240.0)]
    body = prim.union([body, g] + lugs)
    s_rear = prim.place(bearings.seat_608(bottom="lip", lip_d=14.0, below=0.0), tf.Rx(180))
    s_front = prim.place(bearings.seat_608(bottom="lip", lip_d=14.0, below=0.0), tf.T(0, 0, L + 0.02))
    return body - prim.union([s_rear, s_front, prim.cyl(7.0, L + 4, z=-2)])


def _hub_flange():
    """Front friction disc (PPA-CF), local frame z 0 = rear face: flat disc r 9.5 .. 31, slots for the hub's lugs, the
    raised friction annulus on its front face. It keeps the front 608 in and takes the clutch squeeze."""
    disc = prim.tube(D_R, 9.5, FL_F)
    ann = prim.tube(ANNULUS[1], ANNULUS[0], ANNULUS[2] + 0.01, z=FL_F - 0.01)
    body = prim.union([disc, ann])
    slots = [prim.place(prim.box(HUB_R - 4.3, HUB_R - 0.7, -3.3, 3.3, -1.0, 1.0), tf.Rz(a)) for a in (0.0, 120.0, 240.0)]
    disc = disc - prim.union([prim.place(prim.cyl(4.0, 4.0, x=13.0, z=-1.0), tf.Rz(a + 60.0)) for a in (0.0, 120.0, 240.0)])
    return body - prim.union(slots)


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
    boss = prim.cylX(3.2, x - 5.0, x + 5.0, y=y, z=z) ^ prim.box(x - 5.0, x + 5.0, y - 3, y + 3, z0, z + 3)
    pin = prim.cylX(fits.M3_CLEAR_D / 2, x - 10, x + 10, y=y, z=z)
    taps.append(prim.cyl(fits.M3_TAP_D / 2, 20, x=FIRM_SCREW[0], y=FIRM_SCREW[1], z=z0 - 5))   # FIRM screw
    return prim.union([plate, ring, boss] + bosses) - prim.union([hole, pin] + taps)


def _cap_clip():
    """Harry's v7 C-clip (4.6 bore, 3.8 opening toward -Y, 2 thick) on the 4.5 neck of K; dia 14 so it bears on
    the cup hub face (the cup's 10.3 bore mouth is wider than v7's)."""
    return prim.tube(7.0, 2.3, 2.0) - prim.box(-1.9, 1.9, -8, 0, -1, 3)


# ---- SW_ARM: printed contact behind the cup gear (BASE_SPEC section 4, GPIO 42). Polarity (params SWITCHES.SW_ARM):
# CLOSED while the lever rides the raised track on the cup gear's rear face, OPEN in the track's notch at home.
SW_BUMP_R = 16.5
SW_BUMP_H = 2.1
SW_BUMP_DEG = 12.0
SW_LEVER = 28.0
SW_TIP_X = -3.0
SW_PLATE_T = 4.0


def _sw():
    return switch.contact_switch(SW_LEVER)


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

    def wedge(a, z0, h):
        pts = [(0.0, 0.0)] + [(20.0 * math.cos(t - math.pi / 2), 20.0 * math.sin(t - math.pi / 2))
                              for t in np.linspace(-a, a, 24)]
        return prim.extrude(prim.poly_cs(pts), h, z=z0)
    # ramps: wide at the track's outer face (z -SW_BUMP_H), the plain notch width at the gear face (z 0)
    notch = Manifold.batch_hull([wedge(a_top + 0.5 / SW_BUMP_R, -SW_BUMP_H - 0.5, 0.5), wedge(half, -0.01, 0.02)])
    notch = prim.place(notch, tf.Rz(sw_tip_deg() + 90.0))       # centred on the lever tip
    out = ring - notch
    return out


def sw_base_holes():
    """Switch-frame (x, y) of the switch base's four mounting holes (cad.lib.switch: x = -root_len / 2 and x_flex / 2,
    y = +-(base_w / 2 - 3.5)), read off the base itself as sensors._mount_pilots does."""
    sw = _sw()
    lo, hi = prim.bbox(sw["base"])
    bw, rl = hi[1] - lo[1], -lo[0] - 2.0
    return [(x, s * (bw / 2 - 3.5)) for x in (-rl / 2, sw["info"]["x_flex"] / 2) for s in (-1, 1)]


def _sw_bracket():
    """SW_ARM bracket (PLA), R frame: flange screwed to the wall rear face below the gears, an arm back under the
    gears, a plate the switch base screws onto (4 x M3x8 self tap into d2.6 pilots through the plate)."""
    M = sw_frame()
    (x0, y0, z0), (x1, y1, z1) = prim.bbox(prim.place(_sw()["base"], M))
    zb = z0
    plate = prim.box(x0 - 3.0, x1 + 1.5, y0 - 9.0, y1, zb - SW_PLATE_T, zb)      # +X edge 2 mm off the wrap idler
    arm = prim.box(-11.5, 8.5, y0 - 9.0, y0 - 1.0, zb - SW_PLATE_T, -WALL_T)   # below the switch base, clear of the wrap idler
    flange = prim.box(-16.0, 16.0, y0 - 17.0, y0 - 1.0, -WALL_T - 4.0, -WALL_T)
    body = prim.union([plate, arm, flange])
    cut = [prim.cyl(fits.M3_CLEAR_D / 2, 10, x=sx, y=y0 - 13.0, z=-WALL_T - 6.0) for sx in (-12.0, 12.0)]
    # self tap pilots under the base holes, through the plate (an M3x8 through the 3 mm base ends 1 mm behind it,
    # where nothing lies within 5 mm)
    cut += [prim.place(prim.cyl(fits.M3_TAP_D / 2, SW_PLATE_T + 1.0, x=x, y=y, z=-SW_PLATE_T - 0.5), M)
            for x, y in sw_base_holes()]
    return body - prim.union(cut), y0


def fasteners_m3(L):
    """M3 SHCS model along +Z: head z = -3 .. 0, shank 0 .. L."""
    return prim.union([prim.cyl(2.75, 3.0, z=-3.0), prim.cyl(1.5, L)])


def _bed(solid, R=None):
    """print_tf: rotate by R (4x4, default none) and move the result onto the bed corner (min x, y, z = 0)."""
    R = np.eye(4) if R is None else R
    (x0, y0, z0), _ = prim.bbox(prim.place(solid, R))
    return tf.T(-x0, -y0, -z0) @ R


def _clip_z(d, face0):
    """z of a clip's lower face in its groove behind a gear whose rear face is at face0 (shaft diameter d)."""
    return face0 - 0.3 - clips.groove_dims(d)["width"] + clips.CLIP_AX_CLEAR / 2


# ------------------------------------------------------------------------------------------------ D50 solids
AXLE_D = 8.0                  # idler axles (PPA-CF pins, press fit in the wall / plates)


def bevel(m, z, z_mate, b, bore_d, hub=None, flat=1.0):
    """Straight bevel gear (approximate: the spur profile of the large end extruded toward the apex with the cone
    scale), axis +Z, large end at z = 0, teeth toward +z (the apex side). b = face width along the cone. hub = (r, h)
    boss on the -z side (away from the apex). Returns (solid, info)."""
    d = math.atan2(z, z_mate)
    r0 = m * z / 2
    rc = r0 / math.sin(d)
    h = b * math.cos(d)
    sc = (rc - b) / rc
    prof = gears.spur_profile(m, z, backlash=fits.GEAR_BACKLASH + 0.15)
    body = prim.extrude(prim.shapely_cs(prof), h, scale_top=sc)
    parts_ = [body]
    if hub is not None:
        parts_.append(prim.cyl(hub[0], hub[1] + 0.01, z=-hub[1]))
    g = prim.union(parts_)
    rb = (bore_d + fits.SHAFT_IN_BORE) / 2
    cut = prim.cyl(rb, 200, z=-100)
    if flat:
        cut = cut - prim.box(rb - flat, rb + 5, -rb - 5, rb + 5, -120, 120)
    return g - cut, dict(delta=math.degrees(d), apex=r0 / math.tan(d), h=h, rc=rc, r_tip=r0 + m)


def _bev(b, bore, hub_h=3.5, hub_r=None):
    m, z, w = b
    hr = hub_r if hub_r is not None else max(bore / 2 + 3.0, m * z / 2 - m * 1.6)
    return bevel(m, z, z, w, bore, hub=(hr, hub_h))[0]


def _stub(d, z0, z1, flats=(), grooves=()):
    """Plain printed shaft along local Z with D flats (z0, z1 ranges, +X) and clip grooves at z."""
    body = prim.cyl(d / 2, z1 - z0, z=z0)
    cut = [prim.box(d / 2 - (1.0 if d <= 8 else 1.5), d, -d, d, a, b) for a, b in flats]
    cut += [clips.clip_groove_cutter(d, z=z) for z in grooves]
    return body - prim.union(cut) if cut else body


def _axle(z0, z1, head_at_end=True):
    """Idler axle pin (PPA-CF) along local Z from z0 to z1, head r 6 x 1.5 at z1 (or at z0)."""
    pin = prim.cyl(AXLE_D / 2, z1 - z0, z=z0)
    hz = z1 if head_at_end else z0 - 1.5
    return prim.union([pin, prim.cyl(6.0, 1.5, z=hz)])


def _rear_plate(x0, x1, outline):
    """Plate in a world YZ plane between x0 and x1 from a shapely outline in (y, z)."""
    s = prim.extrude(prim.shapely_cs(outline), x1 - x0, z=x0)
    return prim.place(s, FX)


def _yz_disc(c, r, n=48):
    return Point(float(c[0]), float(c[1])).buffer(r, n // 4)


LINE_NOTCH = (SP["line_y"] - 9.0, SP["line_y"] + 9.0)


def _p1_outline():
    from shapely.geometry import MultiPoint, box as sbox
    pts = []
    for c, r in ((CAP_YZ, 31.0), (LANE_C, 12.0), (LANE_K, 9.0), (RL["pc"], 9.0), ((162.0, 115.0), 6.0),
                 ((165.0, 60.0), 6.0), ((-8.0, -30.0), 6.0)):
        pts += list(_yz_disc(c, r).exterior.coords)
    pts += [(-14.0, -40.0), (22.0, -40.0)]
    h = MultiPoint(pts).convex_hull
    h = h.difference(sbox(LINE_NOTCH[0], -100.0, LINE_NOTCH[1], CAP_YZ[1] - 16.0))
    return h


def _p2_outline():
    from shapely.geometry import MultiPoint, box as sbox
    pts = []
    for c, r in ((CAP_YZ, 14.0), (LANE_K, 9.0), (RL["pk"], 9.0), ((162.0, 115.0), 6.0), ((165.0, 60.0), 6.0),
                 ((-8.0, -30.0), 6.0)):
        pts += list(_yz_disc(c, r).exterior.coords)
    pts += [(-14.0, -40.0), (12.0, -40.0), (CHEEK_I[0], 20.0), (CHEEK_O[1], 20.0)]
    h = MultiPoint(pts).convex_hull
    h = h.difference(sbox(CHEEK_I[1] - 1.0, -100.0, CHEEK_O[0] + 1.0, Z_SS + FLANGE_R2 + 3.0))      # spool
    h = h.difference(sbox(LINE_NOTCH[0], -100.0, LINE_NOTCH[1], CAP_YZ[1] - 14.0))
    h = h.difference(sbox(TSP_Y[0] - 1.0, -100.0, T_BR_Y[1] + 1.0, Z_SS + 36.0))                     # 45T pair
    if h.geom_type != "Polygon":
        h = max(h.geoms, key=lambda g: g.area)
    return h


SPACERS = ((162.0, 115.0), (165.0, 60.0), (-8.0, -30.0))


def _p1():
    """P1 (PPA-CF): front plate of the reel unit, world YZ plane; boss on its rear face round the capstan holds sprag
    #2 (crush-rib seat, mouth at the front) and the capstan tube bushing. Bushings for lane C and lane K, the I_c axle
    (press fit), screw holes for the spacers and the leg. Print front face down."""
    x0, x1 = X_P1
    body = _rear_plate(x0, x1, _p1_outline())
    boss = [prim.place(prim.cyl(14.0, x0 - X_P1B + 0.01, x=c[0], y=c[1], z=X_P1B), FX) for c in (CAP_YZ, RL["pc"])]
    body = prim.union([body] + boss)
    cut = [prim.place(prim.cyl((20.0 + fits.SHAFT_IN_BORE) / 2, 60, x=CAP_YZ[0], y=CAP_YZ[1], z=X_P1B - 1), FX)]
    cut.append(prim.place(prim.cyl(10.0 - 0.05, 60, x=RL["pc"][0], y=RL["pc"][1], z=X_P1B - 1)
                          - prim.box(RL["pc"][0] + 8.5, RL["pc"][0] + 20, RL["pc"][1] - 20, RL["pc"][1] + 20, X_P1B - 2, x1 + 2), FX))
    for c, d in ((LANE_C, LC_D), (LANE_K, LK_D)):
        cut.append(prim.place(prim.cyl((d + fits.SHAFT_IN_BORE + fits.HOLE_HORIZ_EXTRA) / 2, 30, x=c[0], y=c[1], z=x0 - 1), FX))
    for c in SPACERS + ((4.0, -36.0),):
        cut.append(prim.place(prim.cyl(fits.M3_CLEAR_D / 2, 30, x=c[0], y=c[1], z=x0 - 1), FX))
    return body - prim.union(cut)


def _icr():
    """I_c idler (PPA-CF) on its joint (local z = world x): 58T m2 in the C plane with a hub forward; sprag #2's
    crush-rib seat through hub and gear (the outer race turns with the idler, the inner race sits on the fixed keyed
    axle). Print gear face down."""
    g = gears.spur_gear(M_RC, Z_RC[1], X_CPL[1] - X_CPL[0]).translate((0, 0, X_CPL[0]))
    hub = prim.cyl(bearings.CSK20_OD / 2 + bearings.BRG_RIB_BORE / 2 + 4.0, SPRAG_X[1] - X_CPL[1] + 0.01, z=X_CPL[1] - 0.01)
    body = prim.union([g, hub])
    # show build: no sprag #2: a plain bore, the idler turns free on the plain fixed axle (the hub fills the old seat)
    return body - prim.cyl((20.0 + fits.SHAFT_IN_BORE) / 2, 80, z=X_CPL[0] - 20)


def _icr_axle():
    """Fixed I_c axle (PPA-CF d20) on the I_c joint frame without its rotation: D flat pressed in P1's boss, key rib in
    the sprag's inner keyway, head on P1's front face, clip groove behind the idler."""
    z0, z1 = X_CPL[0] - 2.6, X_P1[1]
    body = prim.cyl(10.0, z1 - z0, z=z0)            # show build: plain d20 (no sprag key rib)
    body = prim.union([body, prim.cyl(13.0, 1.5, z=z1)])
    flat = prim.box(8.5, 20.0, -20.0, 20.0, X_P1B - 1.0, z1 + 0.01)
    return body - prim.union([flat, clips.clip_groove_cutter(20.0, z=X_CPL[0] - 0.3 - clips.groove_dims(20.0)["width"])])


def _p2():
    """P2 (PPA-CF): rear plate, world YZ plane: bushings for the cup shaft K' and lane K, the I_k axle (press fit),
    spacer screws, bolted onto the cheeks' front edges. Print flat."""
    x0, x1 = X_P2
    body = _rear_plate(x0, x1, _p2_outline())
    cut = []
    for c, d in ((CAP_YZ, K_D), (LANE_K, LK_D)):
        cut.append(prim.place(prim.cyl((d + fits.SHAFT_IN_BORE + fits.HOLE_HORIZ_EXTRA) / 2, 30, x=c[0], y=c[1], z=x0 - 1), FX))
    cut.append(prim.place(prim.cyl(AXLE_D / 2 - 0.05, 30, x=RL["pk"][0], y=RL["pk"][1], z=x0 - 1), FX))
    for c in SPACERS + ((4.0, -36.0), (116.0, 30.0), (164.0, 30.0)):
        cut.append(prim.place(prim.cyl(fits.M3_CLEAR_D / 2, 30, x=c[0], y=c[1], z=x0 - 1), FX))
    return body - prim.union(cut)


def _leg(x0, x1, y0, y1, z_top, lane_t=True):
    """Leg (PLA) under a reel-unit plate: post from the deck with a foot flange; lane T runs through its bushing."""
    dz = SP["deck"]
    post = prim.box(x0, x1, y0, y1, dz, z_top)
    foot = prim.box(x0 - 10.0, x1 + 10.0, y0, y1, dz, dz + FOOT_T)
    body = prim.union([post, foot])
    cut = []
    for xs in (x0 - 5.0, x1 + 5.0):
        cut.append(prim.cyl(fits.M3_CLEAR_D / 2, FOOT_T + 2, x=xs, y=(y0 + y1) / 2, z=dz - 1))
    if lane_t:
        cut.append(prim.cylX((LT_D + fits.SHAFT_IN_BORE + fits.HOLE_HORIZ_EXTRA) / 2, x0 - 1, x1 + 1,
                             y=LANE_T[0], z=LANE_T[1]))
    cut.append(prim.cylX(fits.M3_TAP_D / 2, x0 - 1, x1 + 1, y=4.0, z=-36.0))                 # plate screw
    return body - prim.union(cut)


def _spacer(x0, x1, c):
    return prim.place(prim.tube(5.0, fits.M3_CLEAR_D / 2, x0 - x1, z=x1), FX @ tf.T(c[0], c[1], 0.0))


def _cheek(y0, y1):
    """Reel unit cheek (PLA), world XZ plane between y0 and y1: carries the spool shaft (608 seat) and P2 (front edge),
    foot flange on the deck."""
    dz = SP["deck"]
    xa, xb = X_CHEEK0, X_CHEEK1
    top = Z_SS + 42.0
    plate = prim.box(xa, xb, y0, y1, dz, top)
    out = 1.0 if y0 > 140 else -1.0
    fy0, fy1 = (y1, y1 + 14.0) if out > 0 else (y0 - 14.0, y0)
    foot = prim.box(xa, xb, fy0, fy1, dz, dz + FOOT_T)
    body = prim.union([plate, foot])
    seat = prim.place(bearings.seat_608(bottom="lip", lip_d=14.0, below=0.0),
                      tf.T(X_SS, y0 if out > 0 else y1, Z_SS) @ (FYN if out > 0 else FYP))
    cut = [seat, prim.cylY((S_D + 2.0) / 2, y0 - 1, y1 + 1, x=X_SS, z=Z_SS)]
    for xs in (xa + 10.0, (xa + xb) / 2, xb - 10.0):
        cut.append(prim.cyl(fits.M3_CLEAR_D / 2, FOOT_T + 2, x=xs, y=(fy0 + fy1) / 2, z=dz - 1))
    for zs in (30.0,):
        cut.append(prim.cylX(fits.M3_TAP_D / 2, xb - 12.0, xb + 1.0, y=(y0 + y1) / 2, z=zs))
    return body - prim.union(cut)


def _t_bracket():
    """T bracket (PLA), world XZ plate y 21..27: bushings for T3 and the spool shaft behind the 45T pair."""
    dz = SP["deck"]
    y0, y1 = T_BR_Y
    xa, xb = X_SS - 24.0, X_SS + 24.0
    plate = prim.box(xa, xb, y0, y1, dz, Z_SS + 12.0)
    foot = prim.box(xa, xb, y1, y1 + 14.0, dz, dz + FOOT_T)
    body = prim.union([plate, foot])
    cut = [prim.cylY((S_D + fits.SHAFT_IN_BORE + fits.HOLE_HORIZ_EXTRA) / 2, y0 - 1, y1 + 1, x=X_SS, z=z)
           for z in (Z_SS, LANE_T[1])]
    cut.append(prim.cylY(AXLE_D / 2 - 0.05, y0 - 1, y1 + 1, x=RL["ti"][0], z=RL["ti"][1]))
    for xs in (xa + 8.0, xb - 8.0):
        cut.append(prim.cyl(fits.M3_CLEAR_D / 2, FOOT_T + 2, x=xs, y=y1 + 7.0, z=dz - 1))
    return body - prim.union(cut)


def _post(xc, yc, z_top, d_bore):
    """Small post (PLA) from the deck carrying a lane bushing along X at (yc, z_top - 10)."""
    dz = SP["deck"]
    body = prim.union([prim.box(xc - 3.0, xc + 3.0, yc - 7.0, yc + 7.0, dz, z_top),
                       prim.box(xc - 13.0, xc + 13.0, yc - 7.0, yc + 7.0, dz, dz + FOOT_T)])
    cut = [prim.cylX((d_bore + fits.SHAFT_IN_BORE + fits.HOLE_HORIZ_EXTRA) / 2, xc - 4, xc + 4, y=yc, z=z_top - 10.0)]
    for xs in (xc - 8.0, xc + 8.0):
        cut.append(prim.cyl(fits.M3_CLEAR_D / 2, FOOT_T + 2, x=xs, y=yc, z=dz - 1))
    return body - prim.union(cut)


TAB_C_X = (-186.0, -180.0)
TAB_K_X = (-135.0, -129.0)
TAB_SCREWS = ((-189.0, 2.0), (-177.0, 2.0), (-138.0, 40.0), (-126.0, 40.0))


def _tab(x0, x1, lane, d, zr):
    """Lane bushing tab (PLA): plate in a world YZ plane from the wall front forward round the lane, with a flange on
    the wall front face (2 x M3 into the wall). World frame."""
    yw = Y0
    plate = prim.box(x0, x1, lane[0] - 8.0, yw - 0.01, zr[0], zr[1])
    fl = prim.box(x0 - 5.0, x1 + 5.0, yw - 3.0, yw - 0.01, zr[0], zr[1])
    body = prim.union([plate, fl])
    cut = [prim.cylX((d + fits.SHAFT_IN_BORE + fits.HOLE_HORIZ_EXTRA) / 2, x0 - 1, x1 + 1, y=lane[0], z=lane[1])]
    for (tx, tz) in TAB_SCREWS:
        if x0 - 6 <= tx <= x1 + 6:
            cut.append(prim.cylY(fits.M3_CLEAR_D / 2, yw - 5.0, yw + 1.0, x=tx, z=tz))
    return body - prim.union(cut)


def _capstan_tube():
    """Capstan tube C (PPA-CF) on the reel_capstan joint (local Z = world X, z = world x): key rib from the sprag's
    rear face to the drum web, collar on the sprag inner race, round bushing in P1's boss, D flat for G_c, clip groove
    behind it; 10.3 bore for the cup shaft K'."""
    z1 = X_D0 + v7.drum_info()["z_web0"] - 0.3
    z0 = X_CPL[0] - 2.4
    body = shafts.keyed_shaft(z1 - z0, d=20.0, key_w=shafts.KEY20_W, key_h=shafts.KEY20_H,
                              key_z=(X_D0 - z0, z1 - z0 - 0.8), chamfer=0.8).translate((0, 0, z0))
    collar = prim.union([prim.cyl(14.0, 1.0, z=X_P1[1]), prim.cyl(14.0, COLLAR_CONE, z=X_P1[1] + 0.99, r2=10.0)])
    body = prim.union([body, collar])
    g = clips.groove_dims(20.0)
    flat = prim.box(10.0 - 2.0, 20.0, -20.0, 20.0, z0 - 1.0, X_CPL[1] + 0.6)
    groove = clips.clip_groove_cutter(20.0, z=X_CPL[0] - 0.3 - g["width"])
    bore = prim.cyl((K_D + fits.SHAFT_IN_BORE) / 2, z1 - z0 + 2, z=z0 - 1)
    return body - prim.union([flat, groove, bore])


def _kc_shaft():
    """Rear cup shaft K' (PPA-CF): from the v7 neck in front of the cup back through the drum and the tube to G_k and
    the P2 journal; D flats under the cup's rear half and under G_k, clip behind P2."""
    zf = X_RIM + v7.CUP_D
    z0 = X_P2[0] - 2.6
    body = prim.cyl(K_D / 2, zf - z0, z=z0)
    body = prim.union([body, prim.cyl(2.25, 2.6 + 0.01, z=zf - 0.01), prim.cyl(2.8, 1.5, z=zf + 2.6, r2=2.3)])
    flat_cup = prim.box(K_D / 2 - 1.2, K_D, -K_D, K_D, X_RIM - 0.5, zf + 0.01)
    flat_g = prim.box(K_D / 2 - 1.2, K_D, -K_D, K_D, X_KPL[0] - 0.5, X_KPL[1] + 0.5)
    groove = clips.clip_groove_cutter(K_D, z=X_P2[0] - 0.3 - clips.groove_dims(K_D)["width"])
    return body - prim.union([flat_cup, flat_g, groove])


def _spool():
    """Free storage spool (PLA), world frame on its joint (local Z = world +Y, origin on the axis at y 0): core d40, two
    d62 flanges (1.6), pack 20 wide at y 130..150, hub d16 with a D-flat bore on the spool shaft; a radial hole near
    the inboard flange takes the line end. Print on its outboard flange."""
    y0, y1 = PACK_Y
    prof = [(S_D / 2 + 0.2, CHEEK_I[1] + 1.0), (8.0, CHEEK_I[1] + 1.0), (8.0, y0 - FL2_T), (FLANGE_R2, y0 - FL2_T),
            (FLANGE_R2, y0), (CORE_R2, y0), (CORE_R2, y1), (FLANGE_R2, y1), (FLANGE_R2, y1 + FL2_T), (8.0, y1 + FL2_T),
            (8.0, CHEEK_O[0] - 1.0), (S_D / 2 + 0.2, CHEEK_O[0] - 1.0)]
    body = prim.revolve_pts(prof, seg=160)
    flat = prim.box(S_D / 2 - 1.0 + 0.1, 8.0, -8.0, 8.0, CHEEK_I[1] + 1.0, CHEEK_O[0] - 1.0)
    hole = prim.cylX(1.0, 0.0, CORE_R2 + 1.0, y=0.0, z=y0 + 3.0)
    return prim.union([body, flat]) - hole


# ------------------------------------------------------------------------------------------------ phasing
def _prof(m, z, ang, c, frame=None):
    p = gears.spur_profile(m, z)
    p = affinity.rotate(p, ang, origin=(0, 0))
    return affinity.translate(p, c[0], c[1])


def _phase(ga, gb, ratio_sign=-1.0):
    """Static tooth phase (deg) of gear b so that it meshes gear a at joint angle 0 of both (2D, common plane frame):
    ga, gb = (m, z, centre, phase_a). Middle of the zero-overlap window."""
    m, za, ca, pa_ = ga
    _, zb, cb, _ = gb
    A = _prof(m, za, pa_, ca)
    step = 360.0 / zb
    ok = []
    for k in range(240):
        ph = k * step / 240.0
        if A.intersection(_prof(m, zb, ph, cb)).area < 1e-4:
            ok.append(ph)
    if not ok:
        return 0.0
    # widest run of consecutive ok samples (cyclic)
    best, run, start = (0, 0), 0, None
    sset = set(round(o / (step / 240.0)) for o in ok)
    for k in range(480):
        if (k % 240) in sset:
            run += 1
            if run > best[0]:
                best = (run, k)
        else:
            run = 0
    n, kend = best
    kmid = kend - (n - 1) / 2.0
    return (kmid % 240) * step / 240.0


_PH = {}
# as computed by phases(recompute=True) for params 1.5.0 (2D profile overlap search, middle of the zero window); the
# search takes minutes, so the module uses these. Recompute after moving any D50 gear axis (D50_PHASES=1).
PHASES_D50 = {'gk1': 0.0, 'ik1': 5.073529411764706, 'ik2': 7.9, 'gk2': 10.55, 'ic': 5.085365853658536,
              'gc': 2.965909090909091, 'hub': 0.0, 'gt2': 5.008928571428571, 'ga': 0.0, 'icr': 5.806034482758621,
              'gc2': 7.2, 'ga2': 0.0, 'ikr': 4.381578947368421, 'gkc': 16.875, 'gt3': 0.0, 'gs': 19.59375,
              'ti': 5.977941176470589}


def phases(recompute=None):
    """Mesh phases of the D50 gear pairs: each gear's static rotation (deg about its own axis in its joint frame) so
    that the meshes interleave with the joint values kin() gives (zero at state 0)."""
    import os
    if recompute is None:
        recompute = os.environ.get("D50_PHASES") == "1"
    if not recompute:
        return PHASES_D50
    if _PH:
        return _PH
    r = {k: R_of(k) for k in ("i1", "i2", "k2", "c", "ic", "t2")}
    r["a0"] = np.zeros(2)
    r["main"] = np.array(SP["main_xy"])
    r["s"] = np.array((X_REEL, 0.0))
    # K riser (R plane): cup shaft gear at A0 with phase 0
    _PH["gk1"] = 0.0
    _PH["ik1"] = _phase((M_R, Z_R[0], r["a0"], 0.0), (M_R, Z_R[1], r["i1"], 0.0))
    _PH["ik2"] = _phase((M_R, Z_R[1], r["i1"], _PH["ik1"]), (M_R, Z_R[2], r["i2"], 0.0))
    _PH["gk2"] = _phase((M_R, Z_R[2], r["i2"], _PH["ik2"]), (M_R, Z_R[3], r["k2"], 0.0))
    # REEL gear (cad-drum: tooth 0 on world +x at joint 0, in R the same X) -> I_C -> C 22T
    _PH["ic"] = _phase((M_CAP, Z_MAIN, r["main"], 0.0), (M_CAP, Z_IC, r["ic"], 0.0))
    _PH["gc"] = _phase((M_CAP, Z_IC, r["ic"], _PH["ic"]), (M_CAP, Z_CAPG, r["c"], 0.0))
    # clutch hub 40T -> T2 40T
    _PH["hub"] = 0.0
    _PH["gt2"] = _phase((M_T, Z_HUBG, r["s"], 0.0), (M_T, Z_HUBG, r["t2"], 0.0))
    # rear trains (world (y, z) plane frame of FX)
    _PH["ga"] = 0.0
    _PH["icr"] = _phase((M_RC, Z_RC[0], np.array(LANE_C), 0.0), (M_RC, Z_RC[1], RL["pc"], 0.0))
    _PH["gc2"] = _phase((M_RC, Z_RC[1], RL["pc"], _PH["icr"]), (M_RC, Z_RC[2], CAP_YZ, 0.0))
    _PH["ga2"] = 0.0
    _PH["ikr"] = _phase((M_RK, Z_RK[0], np.array(LANE_K), 0.0), (M_RK, Z_RK[1], RL["pk"], 0.0))
    _PH["gkc"] = _phase((M_RK, Z_RK[1], RL["pk"], _PH["ikr"]), (M_RK, Z_RK[2], CAP_YZ, 0.0))
    # 45T pair, world (x, z) plane seen along -Y (T3 frame FYN: local X = world x, local Y = world z)
    _PH["gt3"] = 0.0
    ti = np.asarray(RL["ti"])
    _PH["ti"] = _phase((M_TS, Z_TS, np.array((X_SS, LANE_T[1])), 0.0), (M_TS, Z_TI, ti, 0.0))
    _PH["gs"] = _phase((M_TS, Z_TI, ti, _PH["ti"]), (M_TS, Z_TS, np.array((X_SS, Z_SS)), 0.0))
    return _PH


# ------------------------------------------------------------------------------------------------ contract
def parts(P):
    out = []
    ph = phases()
    fl = FL

    def add(name, solid, mat, R=None, **k):
        if mat not in ("bought", "steel"):
            solid = prim.clean(solid)          # merge near-duplicate vertices so the STL stays watertight
        pt = _bed(solid, R) if mat not in ("bought", "steel") else None
        out.append(Part(name, solid, mat, print_tf=pt, **k))

    def rz(deg):
        return tf.Rz(deg)

    # ---- front group (frame R)
    add("reel_wall", _wall(), "PLA", tf.Rx(90), joint="reel_base", color=COL["wall"], step=1,
        notes="Reel wall: foot, take-off bosses (C, K2, T2, idlers), rocker ears, stop pins, brake guides and crank boss. Print on its foot")
    add("reel_sleeve", _sleeve(), "PPA-CF", tf.Rx(180), joint="reel_base", color=COL["ppa"], step=2,
        bought_ref="3 x M3x8 SHCS", notes="Stationary sleeve at the old capstan axis: c1 turns on it, the cup shaft inside. Print flange down")
    add("reel_sleeve_clip", clips.c_clip(SLEEVE_D), "PLA", joint="reel_base", place=tf.T(0, 0, _clip_z(SLEEVE_D, HUB1_Z[0])),
        color=COL["clip"], step=4, notes="C-clip on the sleeve behind the bus gear hub")
    add("reel_bus", _bus_gear(), "PPA-CF", tf.Rx(180), joint="reel_bus", color=COL["gear"], step=4,
        notes=f"Bus gear m{M_BUS:g} z{Z_C1} (free on the sleeve): driven by the drum's bus gear, drives the swing idler. Print face down")
    add("reel_swing_arm", _swing_arm(), "PPA-CF", joint="reel_swing", color=COL["ppa"], step=4,
        notes="Swing arm: rides on the bus gear hub, carries the idler into mesh on forward wiper turns. Print flat")
    add("reel_gi", gears.spur_gear(M_BUS, Z_IDL, BUS_Z[1] - BUS_Z[0], bore=8.0 + fits.SHAFT_IN_BORE), "PPA-CF",
        joint="reel_idler", place=tf.T(0, 0, BUS_Z[0]), color=COL["gear"], step=4,
        notes=f"Swing idler m{M_BUS:g} z{Z_IDL}: meshes the reel gear only on forward wiper turns")
    add("reel_k", _k_shaft(), "PPA-CF", tf.Rx(180), joint="reel_cup", color=COL["ppa"], step=5,
        notes="Cup shaft K: through the sleeve, cup gear behind the wall, riser gear in front. Print standing")
    add("reel_gk", prim.union([_gear(M_CUPG, Z_CUPG, CUPGEAR_Z[1] - CUPGEAR_Z[0], K_D, 1.2,
                                     hub=(16.0, SLEEVE_Z[0] - 0.6 - CUPGEAR_Z[1]), phase=GK_PHASE),
                               _cup_gear_track()]),
        "PPA-CF", joint="reel_cup", place=tf.T(0, 0, CUPGEAR_Z[0]), color=COL["gear"], step=5,
        notes=f"Cup gear m{M_CUPG:g} z{Z_CUPG}: driven by the drum's wrap idler; the raised track on its back holds "
              "SW_ARM closed, the notch at home lets it open")
    add("reel_k_clip", clips.c_clip(K_D), "PLA", joint="reel_cup", place=tf.T(0, 0, _clip_z(K_D, CUPGEAR_Z[0])),
        color=COL["clip"], step=5, notes="C-clip holding the cup gear on the cup shaft")
    add("reel_gk1", _gear(M_R, Z_R[0], RISER_Z[1] - RISER_Z[0], K_D, 1.2, phase=ph["gk1"]), "PPA-CF", joint="reel_cup",
        place=tf.T(0, 0, RISER_Z[0]), color=COL["gear"], step=6,
        notes=f"Cup riser gear m{M_R:g} z{Z_R[0]} on the cup shaft in front of the wall (1:1 riser to K2)")
    add("reel_k_clip2", clips.c_clip(K_D), "PLA", joint="reel_cup",
        place=tf.T(0, 0, RISER_Z[1] + 0.3 + clips.CLIP_AX_CLEAR / 2), color=COL["clip"], step=6,
        notes="C-clip in front of the riser gear")
    for key, jn, z, nm in (("i1", "reel_ik1", Z_R[1], "reel_ik1"), ("i2", "reel_ik2", Z_R[2], "reel_ik2")):
        add(nm, gears.spur_gear(M_R, z, RISER_Z[1] - RISER_Z[0], bore=AXLE_D + fits.HOLE_CLEAR), "PPA-CF", joint=jn,
            place=tf.T(0, 0, RISER_Z[0]) @ rz(ph["ik1" if key == "i1" else "ik2"]), color=COL["gear"], step=6,
            notes=f"Cup riser idler m{M_R:g} z{z} on its axle pin in the wall")
        c = R_of(key)
        add(nm + "_axle", _axle(-WALL_T, RISER_Z[1] + 0.3), "PPA-CF", joint="reel_base", place=tf.T(c[0], c[1], 0),
            color=COL["ppa"], step=6, notes="Idler axle pin (PPA-CF d8): pressed into the wall from the front, head in front of the idler")
    c = R_of("k2")
    bk = _bev_info(BEV_K)
    add("reel_k2", _stub(LK_D, -14.5, BK_Z + bk["h"] + 0.2, flats=((RISER_Z[0] - 0.5, BK_Z + bk["h"] + 1),),
                         grooves=(-14.0 + 0.2,)), "PPA-CF", tf.Rx(180), joint="reel_k2", color=COL["ppa"], step=7,
        notes="K2 stub (PPA-CF d8): wall bushing, riser gear and the K bevel")
    add("reel_gk2", _gear(M_R, Z_R[3], RISER_Z[1] - RISER_Z[0], LK_D, 1.0, phase=ph["gk2"]), "PPA-CF", joint="reel_k2",
        place=tf.T(0, 0, RISER_Z[0]), color=COL["gear"], step=7, notes=f"K2 riser gear m{M_R:g} z{Z_R[3]}")
    add("reel_bk2", _bev(BEV_K, LK_D, hub_h=BK_Z - RISER_Z[1] - 1.0), "PPA-CF", joint="reel_k2", place=tf.T(0, 0, BK_Z),
        color=COL["bevel"], step=7, notes=f"K bevel m{BEV_K[0]:g} z{BEV_K[1]} on K2 (1:1 into lane K)")
    bc = _bev_info(BEV_C)
    add("reel_cs", _stub(CS_D, GC_Z[0] - 2.6, BC_Z + bc["h"] + 0.2,
                         flats=((GC_Z[0] - 3.0, GC_Z[1] + 0.5), (BC_Z - 4.0, BC_Z + bc["h"] + 1)),
                         grooves=(GC_Z[0] - 0.3 - clips.groove_dims(CS_D)["width"],)),
        "PPA-CF", tf.Rx(180), joint="reel_cs", color=COL["ppa"], step=8,
        notes="Countershaft C (PPA-CF d16): bevel in front of the wall, wall bushing, 22T gear behind")
    add("reel_gc", _gear(M_CAP, Z_CAPG, GC_Z[1] - GC_Z[0], CS_D, 1.5, phase=ph["gc"]), "PPA-CF", joint="reel_cs",
        place=tf.T(0, 0, GC_Z[0]), color=COL["gear"], step=8,
        notes=f"C gear m{M_CAP:g} z{Z_CAPG}: driven by the REEL gear ({Z_MAIN}T) through the {Z_IC}T idler")
    add("reel_cs_clip", clips.c_clip(CS_D), "PLA", joint="reel_cs", place=tf.T(0, 0, _clip_z(CS_D, GC_Z[0])),
        color=COL["clip"], step=8, notes="C-clip behind the C gear")
    add("reel_bc", _bev(BEV_C, CS_D, hub_h=BC_Z - 2.5), "PPA-CF", joint="reel_cs", place=tf.T(0, 0, BC_Z),
        color=COL["bevel"], step=8, notes=f"C bevel m{BEV_C[0]:g} z{BEV_C[1]} (1:1 into lane C)")
    add("reel_ic", gears.spur_gear(M_CAP, Z_IC, GC_Z[1] - GC_Z[0], bore=AXLE_D + fits.HOLE_CLEAR), "PPA-CF",
        joint="reel_ic", place=tf.T(0, 0, GC_Z[0]) @ rz(ph["ic"]), color=COL["gear"], step=8,
        notes=f"REEL idler m{M_CAP:g} z{Z_IC}: from the REEL gear to the C gear")
    c = R_of("ic")
    add("reel_ic_axle", _axle(GC_Z[0] - 0.3 - 1.5 + 1.5, 0.0, head_at_end=False), "PPA-CF", joint="reel_base",
        place=tf.T(c[0], c[1], 0), color=COL["ppa"], step=8,
        notes="REEL idler axle pin (PPA-CF d8): pressed into the wall from behind, head behind the idler")
    # clutch (old storage-reel position)
    add("reel_s", _s_shaft(), "PPA-CF", tf.Rx(180), joint="reel_shaft", color=COL["ppa"], step=9,
        notes="Reel shaft: 608 journals, head = clutch disk, D flat for the reel gear. Print head down")
    add("reel_hub", prim.place(_hub(), rz(ph["hub"])), "PPA-CF", joint="reel_spool", place=tf.T(0, 0, Z_SPOOL), color=COL["spool"],
        step=9, notes="Clutch hub: two 608s on the reel shaft, 40T teeth drive T2 (and the spool 1:1). Print rear face down")
    add("reel_hub_flange", _hub_flange(), "PPA-CF", joint="reel_spool", place=tf.T(0, 0, Z_SPOOL + HUB_L), color=COL["spool"],
        step=9, notes="Friction disc: keeps the front 608 in, carries the clutch friction ring. Print flat")
    for i, zb in ((1, Z_SPOOL + bearings.B608_W), (2, Z_SPOOL + HUB_L)):
        add(f"reel_brg_{i}", bearings.bearing_608(), "bought", joint="reel_spool", place=tf.T(0, 0, zb),
            color=COL["brg"], bought_ref="608ZZ 8x22x7", step=9, notes="608 bearing in the clutch hub")
    add("reel_t", _thrust_ring(), "PLA", joint="reel_t", place=tf.T(0, 0, Z_T), color=COL["clip"], step=9,
        notes="Thrust ring: the rocker pushes it onto the rear 608 inner race")
    add("reel_rocker", _rocker(), "PPA-CF", joint="reel_base", color=COL["rocker"], step=10,
        notes="Brake rocker: flexible PPA-CF lever = clutch spring, pushed by the crank. Print flat")
    add("reel_rocker_pin", fasteners_m3(24.0), "bought", joint="reel_base",
        place=tf.T(ROCKER_PIVOT[0] - 12.0, ROCKER_PIVOT[1], ROCKER_PIVOT[2]) @ tf.Ry(90), color=COL["brg"],
        bought_ref="M3x25 SHCS + M3 nut", step=10, notes="Rocker pivot pin")
    add("reel_trim_screws", prim.union([prim.place(fasteners_m3(8.0), tf.T(X_REEL + s_ * TRIM_R, 0, Z_T - 8.0))
                                        for s_ in (-1, 1)]), "bought", joint="reel_base", color=COL["brg"],
        bought_ref="2 x M3x8 SHCS", step=10, notes="LIGHT trim screws: their tips press the thrust ring")
    add("reel_crank", _crank(), "PPA-CF", tf.Rx(-90), joint="reel_crank", color=COL["ppa"], step=10,
        notes="Brake bell crank: the bar turns it, arm B pushes the rocker leg. Print on its flat side")
    add("reel_crank_washer", _washer(), "PPA-CF", joint="reel_base",
        place=tf.T(CRANK_X, CRANK_BOSS_TOP, CRANK_Z) @ tf.Rx(-90), color=COL["ppa"], step=10,
        notes="Printed washer: keeps the crank on its boss")
    add("reel_crank_screw", fasteners_m3(20.0), "bought", joint="reel_base",
        place=tf.T(CRANK_X, CRANK_BOSS_TOP + WASHER_T, CRANK_Z) @ tf.Rx(90), color=COL["brg"],
        bought_ref="M3x20 SHCS", step=10, notes="Crank retaining screw")
    tip_z = CRANK_Z + CRANK_B[1] + CRANK_RB + FIRM_GAP
    fs = prim.place(fasteners_m3(10.0), tf.T(FIRM_SCREW[0], FIRM_SCREW[1], tip_z + 10.0) @ tf.Rx(180))
    nut = prim.cyl(3.2, 2.4, x=FIRM_SCREW[0], y=FIRM_SCREW[1], z=LEG_Z1, seg=6)
    add("reel_firm_screw", fs + nut, "bought", joint="reel_base", color=COL["brg"], bought_ref="M3x10 SHCS + M3 nut",
        step=10, notes="FIRM set screw: through the rocker leg from the front, its tip on the crank's arm B (sets FIRM)")
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
        notes="Guide cap screws")
    gr = _gear(M_BUS, Z_REELG, BUS_Z[1] - BUS_Z[0], S_D, 1.0, hub=(14.0, -WALL_T - BUS_Z[1] - 0.1))
    gr = gr - prim.union([prim.cyl(2.0, 40, x=s_ * TRIM_R, z=-10) for s_ in (-1, 1)])   # hex key to the trim screws
    add("reel_gr", gr, "PPA-CF", joint="reel_shaft", place=tf.T(0, 0, BUS_Z[0]), color=COL["gear"], step=11,
        notes=f"Reel gear m{M_BUS:g} z{Z_REELG}: turns the clutch disk {Z_BUSMAIN / Z_REELG:g} x wiper speed")
    add("reel_s_clip", clips.c_clip(S_D), "PLA", joint="reel_shaft", place=tf.T(0, 0, _clip_z(S_D, BUS_Z[0])),
        color=COL["clip"], step=11, notes="C-clip holding the reel gear")
    bt = _bev_info(BEV_T)
    add("reel_t2", _stub(LT_D, -12.5, BT_Z + bt["h"] + 0.2,
                         flats=((HUBG_Z[0] - 0.5, BT_Z + bt["h"] + 1),), grooves=(-12.0 + 0.2,)),
        "PPA-CF", tf.Rx(180), joint="reel_t2", color=COL["ppa"], step=11, notes="T2 stub (PPA-CF d8): wall bushing, 40T gear and the T bevel")
    add("reel_gt2", _gear(M_T, Z_HUBG, HUBG_Z[1] - HUBG_Z[0], LT_D, 1.0, phase=ph["gt2"]), "PPA-CF", joint="reel_t2",
        place=tf.T(0, 0, HUBG_Z[0]), color=COL["gear"], step=11, notes=f"T2 gear m{M_T:g} z{Z_HUBG}: meshes the clutch hub 1:1")
    add("reel_bt2", _bev(BEV_T, LT_D, hub_h=BT_Z - HUBG_Z[1] - 1.0), "PPA-CF", joint="reel_t2", place=tf.T(0, 0, BT_Z),
        color=COL["bevel"], step=11, notes=f"T bevel m{BEV_T[0]:g} z{BEV_T[1]} on T2 (1:1 into lane T)")
    sw = _sw()
    Msw = sw_frame()
    brk, _ = _sw_bracket()
    add("reel_sw_bracket", brk, "PLA", joint="reel_base", color=COL["wall"], step=14,
        bought_ref="2 x M3x8 SHCS", notes="SW_ARM bracket: screwed to the wall below the gears. Print flat")
    add("reel_sw_base", sw["base"], "PLA", joint="reel_base", place=Msw, color=COL["clip"], step=14,
        bought_ref="4 x M3x8 self tap (base to bracket)", notes="SW_ARM contact base (printed switch, M3 head on M3 nut)")
    add("reel_sw_lever", sw["lever"], "PLA", tf.Rx(90), joint="reel_base", place=Msw, color=COL["rocker"], step=14,
        bought_ref=f"2 x M3x{sw['info']['clamp_screw_len']} self tap (lever clamp)",
        notes="SW_ARM lever: rides the cup gear's raised track (contact closed) and drops into its home notch (open). Print on its side")
    add("reel_sw_hw", prim.union([sw["screw"], sw["nut"], sw["jam_nut"]]), "bought", joint="reel_base", place=Msw,
        color=COL["brg"], bought_ref="M3x10 SHCS + 2 M3 nuts", step=14, notes="SW_ARM contact screw and nuts")
    # ---- lanes (world): local z = world x
    add("reel_tab_c", _tab(*TAB_C_X, LANE_C, LC_D, (LANE_C[1] - 13.0, LANE_C[1] + 13.0)), "PLA", tf.Rx(-90),
        color=COL["wall"], step=15, bought_ref="2 x M3x10 SHCS", notes="Lane C bushing tab: screwed to the wall front. Print on its flange")
    add("reel_tab_k", _tab(*TAB_K_X, LANE_K, LK_D, (LANE_K[1] - 10.0, LANE_K[1] + 10.0)), "PLA", tf.Rx(-90),
        color=COL["wall"], step=15, bought_ref="2 x M3x10 SHCS", notes="Lane K bushing tab: screwed to the wall front. Print on its flange")
    add("reel_post_t", _post(-205.0, LANE_T[0], LANE_T[1] + 10.0, LT_D), "PLA", color=COL["wall"], step=15,
        bought_ref="2 x M3x10 SHCS", notes="Lane T front post on the deck extension. Print on its foot")
    bkx = _bev(BEV_K, LK_D, hub_h=3.5)
    xk = FL["k2"][0] - bk["r0"]
    add("reel_bkx", bkx, "PPA-CF", joint="reel_lk", place=tf.T(0, 0, xk), color=COL["bevel"], step=16,
        notes=f"Lane K bevel m{BEV_K[0]:g} z{BEV_K[1]} (meshes the K2 bevel)")
    add("reel_lane_k", _stub(LK_D, X_P2[0] - 2.6, xk + bk["h"] - 0.2, flats=((xk - 4.0, xk + bk["h"]), (X_KPL[0] - 0.5, X_KPL[1] + 0.5)),
                             grooves=(X_P2[0] - 0.3 - clips.groove_dims(LK_D)["width"],)),
        "PPA-CF", None, joint="reel_lk", color=COL["ppa"], step=16,
        notes="Lane K (PPA-CF d8): the cup drive along the deck to the reel unit. Print standing")
    bcx = _bev(BEV_C, LC_D, hub_h=3.5)
    xc = FL["c"][0] - bc["r0"]
    add("reel_bcx", bcx, "PPA-CF", joint="reel_lc", place=tf.T(0, 0, xc), color=COL["bevel"], step=16,
        notes=f"Lane C bevel m{BEV_C[0]:g} z{BEV_C[1]} (meshes the C bevel)")
    add("reel_lane_c", _stub(LC_D, X_CPL[0] - 2.6, xc + bc["h"] - 0.2, flats=((xc - 4.0, xc + bc["h"]), (X_CPL[0] - 0.5, X_CPL[1] + 0.5)),
                             grooves=(X_CPL[0] - 0.3 - clips.groove_dims(LC_D)["width"],)),
        "PPA-CF", None, joint="reel_lc", color=COL["ppa"], step=16,
        notes="Lane C (PPA-CF d12): the capstan drive along the deck to the reel unit. Print standing")
    btx = _bev(BEV_T, LT_D, hub_h=3.5)
    xt = FL["t2"][0] - bt["r0"]
    xt3 = X_SS + bt["r0"]                       # in front of T3's axis, teeth toward -x
    add("reel_btx", btx, "PPA-CF", joint="reel_lt", place=tf.T(0, 0, xt), color=COL["bevel"], step=16,
        notes=f"Lane T front bevel m{BEV_T[0]:g} z{BEV_T[1]} (meshes the T2 bevel)")
    add("reel_bt3x", _bev(BEV_T, LT_D, hub_h=3.5), "PPA-CF", joint="reel_lt", place=tf.T(0, 0, xt3) @ tf.Rx(180),
        color=COL["bevel"], step=16, notes=f"Lane T rear bevel m{BEV_T[0]:g} z{BEV_T[1]} (meshes T3 in front of its axis)")
    add("reel_lane_t", _stub(LT_D, xt3 - bt["h"] + 0.2, xt + bt["h"] - 0.2, flats=((xt - 4.0, xt + bt["h"]), (xt3 - bt["h"], xt3 + 4.0))),
        "PPA-CF", None, joint="reel_lt", color=COL["ppa"], step=16,
        notes="Lane T (PPA-CF d8): the take-up drive along the deck to the spool. Print standing")
    # ---- reel unit (world)
    add("reel_p1", _p1(), "PPA-CF", tf.Ry(90), color=COL["frame"], step=17,
        bought_ref="5 x M3x16 SHCS; 5 x M3 nut", notes="Reel unit front plate P1 with the sprag #2 seat and the capstan bushing. Print front face down")
    add("reel_p1_leg", _leg(X_P1[0], X_P1[1], -14.0, 22.0, -40.0), "PLA", color=COL["frame"], step=17,
        bought_ref="2 x M3x10 SHCS", notes="P1 leg: from the deck up to P1; lane T runs through its bushing. Print on its side")
    add("reel_p2", _p2(), "PPA-CF", tf.Ry(90), color=COL["frame"], step=20,
        bought_ref="5 x M3x16 SHCS; 2 x M3x12 SHCS", notes="Reel unit rear plate P2: cup shaft and lane K journals, I_k axle. Print flat")
    add("reel_p2_leg", _leg(X_P2[0], X_P2[1], -14.0, 12.0, -40.0), "PLA", color=COL["frame"], step=20,
        bought_ref="2 x M3x10 SHCS", notes="P2 leg: from the deck up to P2; lane T runs through its bushing. Print on its side")
    for i, c in enumerate(SPACERS):
        add(f"reel_spacer_{i + 1}", _spacer(X_P1[0], X_P2[1], c), "PLA", tf.Ry(90), color=COL["frame"], step=20,
            notes="Spacer tube between P1 and P2 (M3 through)")
    add("reel_cheek_i", _cheek(*CHEEK_I), "PLA", tf.Rx(90), color=COL["frame"], step=18,
        bought_ref="3 x M3x10 SHCS", notes="Inboard cheek: spool shaft 608, P2 bolts to its front edge. Print flat")
    add("reel_cheek_o", _cheek(*CHEEK_O), "PLA", tf.Rx(90), color=COL["frame"], step=18,
        bought_ref="3 x M3x10 SHCS", notes="Outboard cheek: spool shaft 608, P2 bolts to its front edge. Print flat")
    add("reel_t_bracket", _t_bracket(), "PLA", tf.Rx(90), color=COL["frame"], step=18,
        bought_ref="2 x M3x10 SHCS", notes="T bracket: bushings for T3 and the spool shaft behind the 45T pair. Print flat")
    for i, yb in ((3, CHEEK_I[1]), (4, CHEEK_O[0])):
        M = tf.T(X_SS, yb, Z_SS) @ (FYP if i == 3 else FYN)
        add(f"reel_brg_{i}", bearings.bearing_608(), "bought", place=M, color=COL["brg"], bought_ref="608ZZ 8x22x7",
            step=18, notes="608 bearing in a cheek (spool shaft)")
    add("reel_sprag", bearings.bearing_csk20(okey_deg=90.0, ikey_deg=0.0), "bought",
        place=FX @ tf.T(RL["pc"][0], RL["pc"][1], SPRAG_X[1]), color=COL["brg"], bought_ref="CSK20PP one-way bearing 20x47x14",
        step=19, notes="Sprag #2 inside the I_c idler: the capstan train turns only in the wind-in sense")
    add("reel_c", _capstan_tube(), "PPA-CF", None, joint="reel_capstan", color=COL["ppa"], step=17,
        notes="Capstan tube C: key rib in the sprag and the drum, D flat for G_c. Print standing")
    add("reel_drum", v7.drum(mouth=(14.2, COLLAR_CONE + 0.2)), "PPA-CF", joint="reel_capstan", place=tf.T(0, 0, X_D0),
        color=COL["drum"], step=19, notes="Capstan drum dia 40 x 40 (Harry v7), keyed on the tube. Print flange down")
    add("reel_gc2", _gear(M_RC, Z_RC[2], X_CPL[1] - X_CPL[0], 20.0, 2.0, phase=ph["gc2"]), "PPA-CF", joint="reel_capstan",
        place=tf.T(0, 0, X_CPL[0]), color=COL["gear"], step=19, notes=f"G_c m{M_RC:g} z{Z_RC[2]} on the capstan tube")
    add("reel_c_clip", clips.c_clip(20.0), "PLA", joint="reel_capstan", place=tf.T(0, 0, _clip_z(20.0, X_CPL[0])),
        color=COL["clip"], step=19, notes="C-clip behind G_c")
    kc = _kc_shaft()
    kc = prim.place(kc, tf.Rz(180))                           # flats toward local -X (the cup is turned 180 deg)
    add("reel_kc", kc, "PPA-CF", None, joint="reel_cupr", color=COL["ppa"], step=19,
        notes="Rear cup shaft K': cup in front, G_k behind, journal in P2. Print standing")
    add("reel_cup", v7.cup(), "PLA", joint="reel_cupr", place=tf.T(0, 0, X_RIM) @ tf.Rz(180), color=COL["cup"], step=19,
        notes="Wrap cup (Harry v7, ring hole turned axial): ring at the bottom at home, the line passes straight through it. Print open end down")
    add("reel_cup_clip", _cap_clip(), "PLA", joint="reel_cupr", place=tf.T(0, 0, X_RIM + v7.CUP_D + 0.3) @ tf.Rz(180),
        color=COL["clip"], step=19, notes="Harry's v7 C-clip on the cup shaft neck in front of the cup")
    add("reel_gkc", prim.place(_gear(M_RK, Z_RK[2], X_KPL[1] - X_KPL[0], K_D, 1.2, phase=ph["gkc"]), tf.Rz(180)),
        "PPA-CF", joint="reel_cupr", place=tf.T(0, 0, X_KPL[0]), color=COL["gear"], step=20,
        notes=f"G_k m{M_RK:g} z{Z_RK[2]} on the cup shaft K'")
    add("reel_kc_clip", clips.c_clip(K_D), "PLA", joint="reel_cupr", place=tf.T(0, 0, _clip_z(K_D, X_P2[0])),
        color=COL["clip"], step=20, notes="C-clip behind P2")
    add("reel_ga", _gear(M_RC, Z_RC[0], X_CPL[1] - X_CPL[0], LC_D, 1.5, phase=ph["ga"]), "PPA-CF", joint="reel_lc",
        place=tf.T(0, 0, X_CPL[0]), color=COL["gear"], step=19, notes=f"G_a m{M_RC:g} z{Z_RC[0]} on lane C")
    add("reel_icr", prim.place(_icr(), rz(ph["icr"])), "PPA-CF", tf.Rx(180), joint="reel_icr", color=COL["gear"], step=19,
        notes=f"I_c m{M_RC:g} z{Z_RC[1]}: idler from lane C to the capstan; show build: plain bore, turns free on its axle (no sprag #2)")
    add("reel_icr_axle", _icr_axle(), "PPA-CF", None, place=FX @ tf.T(RL["pc"][0], RL["pc"][1], 0.0),
        color=COL["ppa"], step=19, notes="I_c axle (PPA-CF d20, plain in the show build): fixed in P1 (D flat), the I_c idler turns on it")
    add("reel_ga2", _gear(M_RK, Z_RK[0], X_KPL[1] - X_KPL[0], LK_D, 1.0, phase=ph["ga2"]), "PPA-CF", joint="reel_lk",
        place=tf.T(0, 0, X_KPL[0]), color=COL["gear"], step=20, notes=f"G_a' m{M_RK:g} z{Z_RK[0]} on lane K")
    add("reel_ikr", gears.spur_gear(M_RK, Z_RK[1], X_KPL[1] - X_KPL[0], bore=AXLE_D + fits.HOLE_CLEAR), "PPA-CF",
        joint="reel_ikr", place=tf.T(0, 0, X_KPL[0]) @ rz(ph["ikr"]), color=COL["gear"], step=20,
        notes=f"I_k m{M_RK:g} z{Z_RK[1]}: idler from lane K to the cup shaft")
    add("reel_ikr_axle", _axle(X_P2[1], X_KPL[1] + 0.3), "PPA-CF", None,
        place=FX @ tf.T(RL["pk"][0], RL["pk"][1], 0.0), color=COL["ppa"], step=20,
        notes="I_k axle pin (PPA-CF d8): pressed into P2, head in front of the idler")
    # T3 and the spool (FYN joints: local z = -world y)
    add("reel_t3", _stub(LT_D, -T_BR_Y[1] - 0.5, -(LANE_T[0] + bt["r0"]) + bt["h"] + 0.2,
                         flats=((-TSP_Y[1] - 0.5, 0.0),)), "PPA-CF", joint="reel_t3", color=COL["ppa"], step=18,
        notes="T3 stub (PPA-CF d8): rear T bevel and the 45T gear")
    add("reel_bt3", _bev(BEV_T, LT_D, hub_h=3.0), "PPA-CF", joint="reel_t3", place=tf.T(0, 0, -(LANE_T[0] + bt["r0"])),
        color=COL["bevel"], step=18, notes=f"T3 bevel m{BEV_T[0]:g} z{BEV_T[1]}")
    add("reel_gt3", _gear(M_TS, Z_TS, TSP_Y[1] - TSP_Y[0], LT_D, 1.0, phase=ph["gt3"]), "PPA-CF", joint="reel_t3",
        place=tf.T(0, 0, -TSP_Y[1]), color=COL["gear"], step=18, notes=f"T3 gear m{M_TS:g} z{Z_TS}")
    add("reel_ss", _stub(S_D, -CHEEK_O[1] - 2.6, -TSP_Y[0] + 0.5, flats=((-TSP_Y[1] - 0.5, -TSP_Y[0] + 1.0),
                                                                          (-CHEEK_O[0], -CHEEK_I[1]))),
        "PPA-CF", None, joint="reel_spool_r", color=COL["ppa"], step=18,
        notes="Spool shaft (PPA-CF d8): 45T gear, bushing in the T bracket, 608s in both cheeks, the spool on its flat")
    add("reel_ti", gears.spur_gear(M_TS, Z_TI, TSP_Y[1] - TSP_Y[0], bore=AXLE_D + fits.HOLE_CLEAR), "PPA-CF", joint="reel_ti",
        place=tf.T(0, 0, -TSP_Y[1]) @ rz(ph["ti"]), color=COL["gear"], step=18, notes=f"Take-up idler m{M_TS:g} z{Z_TI}")
    add("reel_ti_axle", _axle(-T_BR_Y[1], -TSP_Y[0] + 0.3), "PPA-CF", None,
        place=tf.T(RL["ti"][0], 0.0, RL["ti"][1]) @ FYN, color=COL["ppa"], step=18,
        notes="Take-up idler axle pin (PPA-CF d8): pressed into the T bracket, head on the inboard side of the idler")
    add("reel_gs", _gear(M_TS, Z_TS, TSP_Y[1] - TSP_Y[0], S_D, 1.0, phase=ph["gs"]), "PPA-CF", joint="reel_spool_r",
        place=tf.T(0, 0, -TSP_Y[1]), color=COL["gear"], step=18, notes=f"Spool shaft gear m{M_TS:g} z{Z_TS}")
    add("reel_spool", _spool(), "PLA", tf.Rx(180), joint="reel_spool_r", place=tf.Rx(180), color=COL["spool"], infill=0.25,
        step=18, notes=f"Free storage spool: 20 m of 1 mm Dyneema on a dia {2 * CORE_R2:g} core, d{2 * FLANGE_R2:g} flanges, keyed on the spool shaft")
    return out


def joints(P):
    sp = _spec(P) if P is not None else SP
    base = np.eye(4)
    base[:3, 0], base[:3, 1], base[:3, 2] = [np.asarray(a, float) for a in sp["axes"]]
    def rj(name, key):
        c = R_of(key)
        return Joint(name, "reel_base", "rev", (c[0], c[1], 0.0), (0, 0, 1))
    return [
        Joint("reel_base", None, "fixed", sp["origin"], (0, 0, 1), base=base),
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
        rj("reel_ik1", "i1"), rj("reel_ik2", "i2"), rj("reel_k2", "k2"), rj("reel_cs", "c"), rj("reel_ic", "ic"),
        rj("reel_t2", "t2"),
        Joint("reel_lk", None, "rev", (0.0, LANE_K[0], LANE_K[1]), (0, 0, 1), base=FX),
        Joint("reel_lc", None, "rev", (0.0, LANE_C[0], LANE_C[1]), (0, 0, 1), base=FX),
        Joint("reel_lt", None, "rev", (0.0, LANE_T[0], LANE_T[1]), (0, 0, 1), base=FX),
        Joint("reel_capstan", None, "rev", (0.0, CAP_YZ[0], CAP_YZ[1]), (0, 0, 1), base=FX),
        Joint("reel_cupr", None, "rev", (0.0, CAP_YZ[0], CAP_YZ[1]), (0, 0, 1), base=FX),
        Joint("reel_icr", None, "rev", (0.0, RL["pc"][0], RL["pc"][1]), (0, 0, 1), base=FX),
        Joint("reel_ikr", None, "rev", (0.0, RL["pk"][0], RL["pk"][1]), (0, 0, 1), base=FX),
        Joint("reel_t3", None, "rev", (X_SS, 0.0, LANE_T[1]), (0, 0, 1), base=FYN),
        Joint("reel_spool_r", None, "rev", (X_SS, 0.0, Z_SS), (0, 0, 1), base=FYN),
        Joint("reel_ti", None, "rev", (RL["ti"][0], 0.0, RL["ti"][1]), (0, 0, 1), base=FYN),
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
    """Arm B tip rise (+Z) under the FIRM screw for a bar slide (the leg moves this less the lost motion g)."""
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


def _pack_r(stored_m):
    d = reel_calc.get(None)
    d["core_r"], d["width"] = CORE_R2, SP["width"]
    return reel_calc.pack_r(d, stored_m)


def kin(P, S):
    wiper = _num(S, "wiper_deg")
    bus = float(S.get("bus_deg", (Z_BUSMAIN / Z_C1) * wiper)) + BUS_PHASE
    dor = _num(S, "reel_deg")                              # REEL gear angle (about world +Y, forward +)
    swing = 0.0 if _num(S, "wiper_dir", 1.0) >= 0 else -SWING_OUT_DEG
    dv = None if ("bus_deg" in S or "capstan_deg" in S) else _drum_vals(P, S)
    cup = None
    if dv is not None:
        v, idl0 = dv
        if "drum_main" in v:
            bus = (Z_BUSMAIN / Z_C1) * float(v["drum_main"]) + BUS_PHASE_D
        if "drum_out_reel" in v:
            dor = float(v["drum_out_reel"])
        if "drum_idler" in v and ("wraps" not in S or "drum_deg" in S):
            cup = (42.0 / Z_CUPG) * (float(v["drum_idler"]) - idl0)
    _, th_i, th_r = gear_angles(bus, swing)
    wraps = _num(S, "wraps") if cup is None else -cup / 360.0
    cup_f = -360.0 * wraps
    lane_c = (Z_MAIN / Z_CAPG) * dor if "capstan_deg" not in S else float(S["capstan_deg"])
    if "spool_deg" in S:
        spool = float(S["spool_deg"])                      # about world +Y, payout +
    else:
        stored = max(0.0, 20.0 - _num(S, "line_out_m"))
        r_mid = 0.5 * (_pack_r(20.0) + _pack_r(stored))
        spool = math.degrees(_num(S, "line_out_m") * 1000.0 / r_mid)
    hub = -spool                                           # hub (R +Z) = - spool (+Y): 1:1 chain
    bar = bar_mm(P, S)
    return {"reel_cup": cup_f, "reel_bus": bus, "reel_swing": swing, "reel_idler": th_i, "reel_shaft": th_r,
            "reel_spool": hub, "reel_t": 0.0, "reel_bar": bar, "reel_crank": crank_deg(bar),
            "reel_ik1": -(Z_R[0] / Z_R[1]) * cup_f, "reel_ik2": cup_f, "reel_k2": -cup_f,
            "reel_ic": (Z_MAIN / Z_IC) * dor, "reel_cs": -(Z_MAIN / Z_CAPG) * dor, "reel_t2": -hub,
            "reel_lk": cup_f, "reel_lc": lane_c, "reel_lt": hub,
            "reel_capstan": lane_c, "reel_icr": -(Z_RC[0] / Z_RC[1]) * lane_c,
            "reel_cupr": cup_f, "reel_ikr": -(Z_RK[0] / Z_RK[1]) * cup_f,
            "reel_t3": hub, "reel_ti": -(Z_TS / Z_TI) * hub, "reel_spool_r": hub}


def allowed(P):
    pairs = [
        ("reel_sleeve", "reel_wall", "sleeve pressed in the wall, rib in the slot"),
        ("reel_sleeve", "reel_bus", "bus gear turns free on the sleeve"),
        ("reel_sleeve", "reel_k", "cup shaft turns inside the sleeve"),
        ("reel_sleeve", "reel_sleeve_clip", "clip in the groove"),
        ("reel_k", "reel_gk", "D fit"), ("reel_k", "reel_k_clip", "clip in the groove"),
        ("reel_k", "reel_gk1", "D fit"), ("reel_k", "reel_k_clip2", "clip in the groove"),
        ("reel_swing_arm", "reel_gi", "idler turns on the arm pin"),
        ("reel_swing_arm", "reel_bus", "split arm ring grips the bus gear hub (drag)"),
        ("reel_s", "reel_gr", "D-flat fit"), ("reel_s", "reel_s_clip", "clip in the groove"),
        ("reel_brg_1", "reel_hub", "608 pressed in the hub"), ("reel_brg_2", "reel_hub", "608 pressed in the hub"),
        ("reel_brg_1", "reel_s", "journal in the inner race"), ("reel_brg_2", "reel_s", "journal in the inner race"),
        ("reel_hub", "reel_hub_flange", "lugs in the disc slots"),
        ("reel_rocker_pin", "reel_wall", "M3 pin through the ears"), ("reel_rocker_pin", "reel_rocker", "M3 pin through the rocker"),
        ("reel_trim_screws", "reel_rocker", "trim screws tapped into the rocker bosses"),
        ("reel_sw_hw", "reel_sw_*", "contact screw in the lever, nut in the base trap"),
        ("reel_sw_lever", "reel_sw_base", "lever clamped on the base pad"),
        ("reel_firm_screw", "reel_rocker", "FIRM screw tapped through the leg, lock nut on its front"),
        ("reel_crank_washer", "reel_crank_screw", "screw through the washer"),
        ("reel_crank_screw", "reel_wall", "screw tapped into the crank boss"),
        ("reel_bar_screws", "reel_wall", "cap screws tapped into the guides"),
        ("reel_bar_screws", "reel_bar_cap_*", "cap screws through the caps"),
        ("reel_ik1", "reel_ik1_axle", "idler turns on its axle"), ("reel_ik1_axle", "reel_wall", "axle pressed in the wall"),
        ("reel_ik2", "reel_ik2_axle", "idler turns on its axle"), ("reel_ik2_axle", "reel_wall", "axle pressed in the wall"),
        ("reel_k2", "reel_wall", "stub in the wall bushing"), ("reel_k2", "reel_gk2", "D fit"), ("reel_k2", "reel_bk2", "D fit"),
        ("reel_cs", "reel_wall", "countershaft in the wall bushing"), ("reel_cs", "reel_gc", "D fit"),
        ("reel_cs", "reel_cs_clip", "clip in the groove"), ("reel_cs", "reel_bc", "D fit"),
        ("reel_ic", "reel_ic_axle", "idler turns on its axle"), ("reel_ic_axle", "reel_wall", "axle pressed in the wall"),
        ("reel_t2", "reel_wall", "stub in the wall bushing"), ("reel_t2", "reel_gt2", "D fit"), ("reel_t2", "reel_bt2", "D fit"),
        ("reel_bk2", "reel_bkx", "bevel mesh (approximate printed bevel model)"),
        ("reel_bc", "reel_bcx", "bevel mesh (approximate printed bevel model)"),
        ("reel_bt2", "reel_btx", "bevel mesh (approximate printed bevel model)"),
        ("reel_bt3x", "reel_bt3", "bevel mesh (approximate printed bevel model)"),
        ("reel_lane_k", "reel_bkx", "D fit"), ("reel_lane_k", "reel_tab_k", "lane in the tab bushing"),
        ("reel_lane_k", "reel_p1", "lane in the P1 bushing"), ("reel_lane_k", "reel_p2", "lane journal in P2"),
        ("reel_lane_k", "reel_ga2", "D fit"),
        ("reel_lane_c", "reel_bcx", "D fit"), ("reel_lane_c", "reel_tab_c", "lane in the tab bushing"),
        ("reel_lane_c", "reel_p1", "lane in the P1 bushing"), ("reel_lane_c", "reel_ga", "D fit"),
        ("reel_lane_t", "reel_btx", "D fit"), ("reel_lane_t", "reel_bt3x", "D fit"),
        ("reel_lane_t", "reel_post_t", "lane in the post bushing"),
        ("reel_ti", "reel_ti_axle", "idler turns on its axle"), ("reel_ti_axle", "reel_t_bracket", "axle pressed in the bracket"),
        ("reel_lane_t", "reel_p1_leg", "lane in the leg bushing"), ("reel_lane_t", "reel_p2_leg", "lane in the leg bushing"),
        ("reel_t3", "reel_bt3", "D fit"), ("reel_t3", "reel_gt3", "D fit"), ("reel_t3", "reel_t_bracket", "stub in the bracket bushing"),
        ("reel_ss", "reel_gs", "D fit"), ("reel_ss", "reel_t_bracket", "shaft in the bracket bushing"),
        ("reel_ss", "reel_brg_3", "journal in the inner race"), ("reel_ss", "reel_brg_4", "journal in the inner race"),
        ("reel_ss", "reel_spool", "spool keyed on the shaft flat"),
        ("reel_brg_3", "reel_cheek_i", "608 pressed in the cheek"), ("reel_brg_4", "reel_cheek_o", "608 pressed in the cheek"),
        ("reel_sprag", "reel_icr", "CSK20PP pressed in the idler's crush-rib seat, outer key rib"),
        ("reel_sprag", "reel_icr_axle", "axle rib in the sprag inner keyway, inner race on the axle (model is one solid)"),
        ("reel_c", "reel_p1", "capstan tube in the P1 bushing"), ("reel_c", "reel_drum", "keyed fit"),
        ("reel_c", "reel_gc2", "D-flat fit"), ("reel_c", "reel_c_clip", "clip in the groove"),
        ("reel_kc", "reel_c", "cup shaft runs inside the tube"), ("reel_kc", "reel_cup", "D fit"),
        ("reel_kc", "reel_cup_clip", "clip on the neck"), ("reel_kc", "reel_gkc", "D fit"),
        ("reel_kc", "reel_kc_clip", "clip in the groove"), ("reel_kc", "reel_p2", "journal in P2"),
        ("reel_icr_axle", "reel_p1", "axle pressed in P1 (D flat)"), ("reel_icr", "reel_icr_axle", "idler turns on the plain axle (show build)"),
        ("reel_ikr", "reel_ikr_axle", "idler turns on its axle"), ("reel_ikr_axle", "reel_p2", "axle pressed in P2"),
        ("launch_line_*", "reel_cup", "the line leaves the eyelet's front mouth"),
    ]
    out = [Allow(a, b, w) for a, b, w in pairs]
    out += [Allow("reel_trim_screws", "reel_t", "screw tips on the thrust ring (preload)", max_vol=0.5),
            Allow("reel_crank", "reel_rocker", "arm B rises under the static leg (the real leg moves 2.1 mm away first)", max_vol=40.0),
            Allow("reel_firm_screw", "reel_crank", "arm B lifts the screw tip: the leg bends 2.1 mm at FIRM (static rocker in the model)", max_vol=20.0),
            Allow("reel_sw_lever", "reel_gk", "the track holds the lever down (contact closed) except at home", max_vol=60.0),
            Allow("reel_bar", "drum_drum_a", "the bar nose rides cad-drum's brake cam (round nose on the sampled cam envelope)", max_vol=0.1)]
    for a, b in (("reel_tab_c", "reel_wall"), ("reel_tab_k", "reel_wall"), ("reel_p1", "reel_p1_leg"), ("reel_p2", "reel_p2_leg"),
                 ("reel_p2", "reel_cheek_i"), ("reel_p2", "reel_cheek_o"), ("reel_spacer_*", "reel_p1"), ("reel_spacer_*", "reel_p2"),
                 ("reel_sleeve", "reel_wall")):
        out.append(Allow(a, b, "bolted faces touch", max_vol=1.0))
    return out


def clearances(P):
    return [
        Clearance("reel_cup", "reel_drum", 0.25, "cup skirt over the drum (v7 0.6 radial)"),
        Clearance("reel_drum", "reel_p1", 0.3, "drum flange in front of P1"),
        Clearance("reel_spool", "reel_cheek_*", 0.5, "spool turns between the cheeks"),
        Clearance("reel_hub*", "reel_wall", 0.3, "clutch hub spins in front of the wall"),
        Clearance("reel_hub", "reel_rocker", 0.3, "rocker behind the hub"),
    ]


def meshes(P):
    T = tf.T
    f = lambda z: (z[1] - z[0])
    out = [Mesh("reel_gk1", "reel_ik1", M_R, Z_R[0], Z_R[1], face_a=f(RISER_Z), face_b=f(RISER_Z), why="cup riser 30T to idler 34T"),
           Mesh("reel_ik1", "reel_ik2", M_R, Z_R[1], Z_R[2], face_a=f(RISER_Z), face_b=f(RISER_Z), why="riser idlers 34T / 30T"),
           Mesh("reel_ik2", "reel_gk2", M_R, Z_R[2], Z_R[3], face_a=f(RISER_Z), face_b=f(RISER_Z), why="riser idler 30T to K2 30T"),
           Mesh("reel_ic", "reel_gc", M_CAP, Z_IC, Z_CAPG, face_a=f(GC_Z), face_b=f(GC_Z), why="REEL idler 41T to the C gear 22T"),
           Mesh("reel_hub", "reel_gt2", M_T, Z_HUBG, Z_HUBG, face_a=f(HUBG_Z), face_b=f(HUBG_Z),
                frame_a=T(0, 0, HUBG_Z[0] - Z_SPOOL), why="clutch hub 40T to T2 40T"),
           Mesh("reel_ga", "reel_icr", M_RC, Z_RC[0], Z_RC[1], face_a=f(X_CPL), face_b=f(X_CPL),
                frame_b=T(0, 0, X_CPL[0]), why="lane C 20T to I_c 58T"),
           Mesh("reel_icr", "reel_gc2", M_RC, Z_RC[1], Z_RC[2], face_a=f(X_CPL), face_b=f(X_CPL),
                frame_a=T(0, 0, X_CPL[0]), why="I_c 58T to G_c 20T"),
           Mesh("reel_ga2", "reel_ikr", M_RK, Z_RK[0], Z_RK[1], face_a=f(X_KPL), face_b=f(X_KPL), why="lane K 20T to I_k 76T"),
           Mesh("reel_ikr", "reel_gkc", M_RK, Z_RK[1], Z_RK[2], face_a=f(X_KPL), face_b=f(X_KPL), why="I_k 76T to G_k 20T"),
           Mesh("reel_gt3", "reel_ti", M_TS, Z_TS, Z_TI, face_a=f(TSP_Y), face_b=f(TSP_Y), why="T3 16T to the take-up idler 34T"),
           Mesh("reel_ti", "reel_gs", M_TS, Z_TI, Z_TS, face_a=f(TSP_Y), face_b=f(TSP_Y), why="take-up idler 34T to the spool shaft 16T")]
    return out


def grounded(P):
    return ["reel_wall", "reel_cheek_*", "reel_p1_leg", "reel_p2_leg", "reel_post_*", "reel_t_bracket"]


def audit_states(P):
    out = []
    for k in range(6):                          # one tooth pitch of the REEL / C chain
        out.append(dict(phase="reel_mesh", t=0.0, drum_deg=347.0, wiper_deg=k * 360.0 / Z_BUSMAIN / 5,
                        reel_deg=k * 360.0 / Z_MAIN / 5))
    for k in range(3):
        out.append(dict(phase="reel_swing_out", t=0.0, drum_deg=0.0, wiper_dir=-1, wiper_deg=-k * 5.0))
    for w in (0.0, 0.25, 0.5, 0.75, 1.0, 3.5, float(WRAPS)):
        out.append(dict(phase="reel_wrap", t=0.0, wraps=w))
    for k in range(5):                          # one tooth pitch of the take-up chain
        out.append(dict(phase="reel_spool", t=0.0, line_out_m=k * 0.0015))
    out.append(dict(phase="reel_firm", t=0.0, drum_deg=41.0, line_out_m=3.0))
    return out


# ------------------------------------------------------------------------------------------------ line path
LINE_R_CAP = v7.DR + 0.5          # line centre radius on the capstan
QF = FX @ tf.Rz(180)              # cup frame axes (ring at +Y = world -Z), origin added below


def _Q():
    """Cup frame -> world: z = world x - X_RIM along +X, ring at +Y (world -Z)."""
    return tf.T(X_RIM, CAP_YZ[0], CAP_YZ[1]) @ QF


def _tangent_upper(c, r, p):
    """Tangent point on circle (c, r) in the world (x, z) plane seen from p, on the upper side."""
    dx, dz = p[0] - c[0], p[1] - c[1]
    d = math.hypot(dx, dz)
    a0 = math.atan2(dz, dx)
    b = math.acos(min(1.0, r / d))
    cands = [a0 + b, a0 - b]
    tz = [c[1] + r * math.sin(a) for a in cands]
    a = cands[int(np.argmax(tz))]
    return np.array((c[0] + r * math.cos(a), c[1] + r * math.sin(a)))


def _ring_points(psi):
    cp = v7.cup_line_points()
    Q = _Q()
    rz = tf.Rz(psi)
    return (tf.apply(Q @ rz, np.array([cp["ring_rear"]]))[0], tf.apply(Q @ rz, np.array([cp["ring_front"]]))[0])


def _rz2(p, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array((c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]))


def line_spans(P, S):
    """The reel's part of the tether for state S as WORLD spans [(points N x 3, allowed part names, label)]: spool pack
    top -> (capstan turns and cup groove when wrapped) -> axial eyelet -> floor segment to the tangent on cad-launch's
    rail-rear pulley (launch.line_entry). The cup turns are the ones kin(P, S) poses."""
    stored = max(0.0, 20.0 - _num(S, "line_out_m"))
    R = _pack_r(stored) + 0.5
    k = kin(P, S)
    w = -k["reel_cupr"] / 360.0
    if 0.0 < w < 0.25:              # the turn model starts after the first quarter turn
        w = 0.0
    psi = -360.0 * w
    ring_r, ring_f = _ring_points(psi)
    spans = []
    if w <= 1e-9:
        t = _tangent_upper((X_SS, Z_SS), R, (ring_r[0], ring_r[2]))
        sp = np.array((t[0], SP["line_y"], t[1]))
        spans.append((np.array([sp, ring_r]), {"reel_spool", "reel_cup"}, "spool -> cup eyelet"))
    else:
        Q = _Q()
        Qi = tf.inv(Q)
        # spool tangent toward the drum's first contact (approximate: the line comes in along the axis from behind)
        z0, z1 = (X_D0 - X_RIM) + v7.FLANGE_H + 0.8, -0.6
        th0 = math.degrees(math.atan2(1.0, 0.0)) + 180.0 * 0.0
        # first contact: the line arrives from below the drum (cup frame +Y = world down): at the drum's +Y side
        th0 = 90.0
        th_end = v7.TH_RIM + psi
        while th0 - th_end <= 60.0:
            th0 += 360.0
        A = th0 - th_end
        n = max(8, int(A / 10))
        turns = []
        for kk in range(n + 1):
            tt = math.radians(th0 - A * kk / n)
            turns.append((LINE_R_CAP * math.cos(tt), LINE_R_CAP * math.sin(tt), z0 + (z1 - z0) * kk / n))
        turns = tf.apply(Q, np.array(turns))
        t = _tangent_upper((X_SS, Z_SS), R, (turns[0][0], turns[0][2]))
        sp = np.array((t[0], SP["line_y"], t[1]))
        spans.append((np.array([sp, turns[0]]), {"reel_spool", "reel_drum"}, "spool -> capstan"))
        spans.append((turns, {"reel_drum"}, "capstan turns"))
        grv = []
        for kk in range(9):
            tdeg = v7.TH_RIM - (v7.TH_RIM - v7.TH_TOP) * kk / 8 + psi
            grv.append((v7.ER * math.cos(math.radians(tdeg)), v7.ER * math.sin(math.radians(tdeg)), v7.CUP_D * kk / 8))
        grv = tf.apply(Q, np.array(grv))
        spans.append((np.vstack([turns[-1:], grv, [ring_r]]), {"reel_drum", "reel_cup"}, "capstan -> cup groove -> ring"))
    spans.append((np.array([ring_r, ring_f]), {"reel_cup"}, "cup eyelet"))
    e = _launch_entry(P, S)
    spans.append((np.array([ring_f, e]), {"reel_cup", "launch_pulley_top"}, "eyelet -> rail-rear pulley"))
    return spans


def _launch_entry(P, S):
    try:
        from cad.parts import launch as _L
        return np.asarray(_L.line_entry(P, S), float)
    except Exception:
        p = math.radians(_num(S, "aim_deg", 6.0))
        u, wv = -200.87, 84.6
        return np.array((u * math.cos(p) - wv * math.sin(p), SP["line_y"], u * math.sin(p) + wv * math.cos(p)))


def line_path(P, S):
    """Tether polyline in WORLD mm for state S (cad-infra contract): spool -> eyelet -> rail-rear pulley tangent
    (launch.line_entry). cad-launch continues from there."""
    pts = []
    for span, _, _ in line_spans(P, S):
        for p in span:
            if not pts or np.linalg.norm(np.asarray(p) - pts[-1]) > 1e-6:
                pts.append(np.asarray(p, float))
    return [tuple(float(v) for v in p) for p in pts]


_PARTS_CACHE = {}


def _posed_world(P, S):
    """Every reel part posed in the WORLD for state S (reel joints only)."""
    vals = kin(P, S)
    js = {j.name: j for j in joints(P)}
    key = id(P)
    if key not in _PARTS_CACHE:
        _PARTS_CACHE.clear()
        _PARTS_CACHE[key] = parts(P)
    Mj = {}

    def M(name):
        if name in Mj:
            return Mj[name]
        j = js[name]
        Mp = np.eye(4) if j.parent is None else M(j.parent)
        Mj[name] = Mp @ j.local(vals.get(name, 0.0))
        return Mj[name]
    out = {}
    for p in _PARTS_CACHE[key]:
        W = (np.eye(4) if p.joint is None else M(p.joint)) @ p.place
        out[p.name] = prim.place(p.solid, W)
    return out


def line_check(P, wraps_list=None):
    """The reel's own line spans (d 1.0 tube) against every reel part except the parts each span may touch, at 0
    wraps, mid wrap and full wraps (spool full). Returns failure strings."""
    if wraps_list is None:
        wraps_list = [0.0, 1.5, float(WRAPS)]
    fails = []
    for wv in wraps_list:
        S = dict(wraps=wv, mode="REEL" if wv else "LAUNCH")
        sol = _posed_world(P, S)
        for pts, allow, label in line_spans(P, S):
            if label == "eyelet -> rail-rear pulley":
                pts = np.array([pts[0] + 2.5 * (pts[1] - pts[0]) / np.linalg.norm(pts[1] - pts[0]), pts[1]])
            hits = line.line_hits(pts, {k: s for k, s in sol.items() if k not in allow})
            for name, v in hits.items():
                if v > 1e-3:
                    fails.append(f"line at {wv:.2f} wraps: {label} hits {name} ({v:.3f} mm3)")
    return fails


# ---- whole-assembly tether check: the posed line against every posed part of every subsystem
TETHER_DT = 1.0
TETHER_HIT = 1e-3
TETHER_ALWAYS = ("launch_line_*",)
TETHER_EXTRA = {"eyelet -> rail-rear pulley": ("launch_pulley_top_*", "launch_rail_rear")}
# mid-wrap the ring orbits the capstan (r 26) and the slack dart-side line comes into the rail-rear pulley at a slant
# (D22): it may brush the pulley's bearing and washers and the fork plates beside it


def tether_states(P):
    """States of the tether check: every TIMELINE phase at its start, its end and at least every TETHER_DT seconds
    between, plus the line at 0 wraps and at full wraps on their own."""
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
            dict(phase="tether_wraps_full", t=0.0, wraps=float(WRAPS))]
    return out


def _module_spans(obj, sub):
    """Another subsystem's line_path result as spans [(points, allowed, label)]: (points, allowed parts) pairs keep
    their parts (cad-launch's contract); plain points are one span that may touch nothing."""
    items = list(obj or [])
    if items and all(isinstance(x, (tuple, list)) and len(x) == 2 and isinstance(x[1], (set, frozenset, list, tuple))
                     and np.asarray(x[0], float).ndim == 2 for x in items):
        return [(np.asarray(p, float), set(a), f"{sub} span {k + 1} ({', '.join(sorted(a))})")
                for k, (p, a) in enumerate(items)]
    return [(np.asarray(items, float).reshape(-1, 3) if items else np.zeros((0, 3)), set(), f"{sub} line")]


def tether_check(P, asm, states=None):
    """Whole-assembly tether check. Every span of the posed line (line_spans, then the line_path spans of the other
    subsystems: cad-launch's along the rail) at every state (tether_states by default), as the d 1.0 line tube,
    against every posed part of every subsystem in asm except the parts that span is meant to touch: its own list
    (guide eye in the wall, capstan, cup ring, pulleys, exit post, storage reel, the pivot pulley, the rail pulleys,
    carriage and dart), TETHER_ALWAYS and TETHER_EXTRA. Exact manifold volumes after a box cull, cached per (part
    pose, span) so the static stretches are computed once. Returns one failure string per (span, part); an allowed
    name of a loaded subsystem that matches no part fails too (a renamed part must not allow nothing silently)."""
    import inspect
    from fnmatch import fnmatchcase
    states = tether_states(P) if states is None else states
    names = list(asm.parts)
    box = {}
    for n, p in asm.parts.items():
        lo, hi = prim.bbox(p.solid)
        box[n] = np.array([[x, y, z, 1.0] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    others = [(sub, m) for sub, m in asm.modules.items() if sub != "reel" and callable(getattr(m, "line_path", None))
              and list(inspect.signature(m.line_path).parameters)[:2] == ["P", "S"]]     # the asm.line hook rule
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


def checks(P, asm=None):
    """Reel-only line check and main-axis keep-outs; with the assembly (tools.audit passes it) also the
    whole-assembly tether check."""
    out = line_check(P) + main_axis_check(P)
    if asm is not None:
        out += tether_check(P, asm)
    return out


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
        ("coupler", cyl_y(35.0 + 2, 0.5, 46.0), ("reel_bus",)),
        ("bus_gear", cyl_y(39.0 + 2, 40.0, 48.0), ("reel_bus",)),
        ("reel_gear", cyl_y(52.0 + 2, 50.5, 62.5), ("reel_ic",)),
        ("collar", cyl_y(30.0 + 2, 65.0, 105.0), ()),
        ("dog_ring", cyl_y(31.0 + 2, 100.5, 111.0), ()),
        ("main_shaft", cyl_y(13.0 + 2, 0.5, 200.0), ()),
        ("drum", cyl_y(42.0 + 2, 1.0, 142.0, x=drum_ax[0], z=drum_ax[2]), ("reel_gk", "reel_bar")),
        ("drum_gear", cyl_y(48.6 + 2, 8.0, 16.0, x=drum_ax[0], z=drum_ax[2]), ("reel_bar",)),
        ("wrap_idler", cyl_y(33.0 + 2, 68.5, 76.5, x=wi[0], z=wi[1]), ("reel_gk",)),
    ]


def main_axis_check(P, states=None):
    """Every reel part (world pose) against the main-axis keep-outs; returns failure strings."""
    fails = []
    for S in (states or [dict(), dict(wiper_dir=-1), dict(wraps=0.5)]):
        sol = _posed_world(P, S)
        for kname, ko, ok in main_axis_keepouts(P):
            for pname, s_ in sol.items():
                if pname in ok:
                    continue
                v = (s_ ^ ko).volume()
                if v > 1e-3:
                    fails.append(f"{pname} enters the {kname} keep-out ({v:.1f} mm3)")
    return sorted(set(fails))


# ------------------------------------------------------------------------------------------------ steps
def steps(P):
    """Assembly steps, in order, plain language, hardware named per step."""
    si = _sw()["info"]
    return [
        dict(title="Screw the reel wall to the deck",
             text="Stand the reel wall on the deck at its marked place (its long foot reaches back onto the deck "
                  "extension tile). Put nine M3 screws down through the counterbored foot holes into the deck inserts "
                  "and tighten. Do this first: the gears later cover some of the holes.",
             parts=["reel_wall"]),
        dict(title="Sleeve, bus gear, swing arm and swing idler",
             text="Push the sleeve into the big hole in the wall from the front, its rib in the slot, flange flat on the "
                  "wall, and screw it on with three M3 x 8 screws (they tap into the wall). From behind slide the "
                  "46-tooth bus gear onto the sleeve, gear face first, hub toward you, and snap the large C-clip into "
                  "the groove behind the hub. Slip the swing arm's ring over the bus gear's hub, idler pin pointing at "
                  "the wall, and put the 30-tooth swing idler on the pin. The arm swings 12 degrees between its two "
                  "stop pins.",
             parts=["reel_sleeve", "reel_sleeve_clip", "reel_bus", "reel_swing_arm", "reel_gi"]),
        dict(title="Cup shaft, cup gear and riser gear",
             text="Push the cup shaft through the sleeve from BEHIND, flat end first, until it comes out in front of "
                  "the wall. Behind: slide the 24-tooth cup gear onto the flat (hub forward, raised track toward you) "
                  "and snap its C-clip on. In front: slide the 30-tooth riser gear onto the flat and snap its C-clip "
                  "on. The cup shaft must turn freely.",
             parts=["reel_k", "reel_gk", "reel_k_clip", "reel_gk1", "reel_k_clip2"]),
        dict(title="Riser idlers and K2",
             text="Push the two idler axle pins through the 34-tooth and the 30-tooth riser idlers and press them into "
                  "their holes in the wall front (34 teeth next to the cup shaft gear). Push the K2 stub through its "
                  "wall bushing from the front with its 30-tooth gear and the 18-tooth bevel on the flats, the bevel "
                  "teeth facing you, and snap its C-clip on behind the wall. Turn the cup gear: the whole riser turns.",
             parts=["reel_ik1", "reel_ik1_axle", "reel_ik2", "reel_ik2_axle", "reel_k2", "reel_gk2", "reel_bk2"]),
        dict(title="Countershaft C and the REEL idler",
             text="Slide the 20-tooth C bevel onto the front of countershaft C (teeth facing away from the shaft), "
                  "feed the shaft through its wall bushing from the front, and from behind slide the 22-tooth C gear "
                  "onto the flat and snap its C-clip on. Push the REEL idler axle through the 41-tooth idler and press "
                  "it into the wall from behind: the idler meshes the drum's REEL gear and the C gear.",
             parts=["reel_cs", "reel_bc", "reel_gc", "reel_cs_clip", "reel_ic", "reel_ic_axle"]),
        dict(title="Clutch hub",
             text="Press one 608 bearing into each end of the clutch hub. Lay the friction disc on the front, its slots "
                  "on the three lugs, raised ring outward. Slide the hub onto the reel shaft from the shaft's thin end "
                  "until the disc meets the shaft's round head. Slide the thrust ring onto the shaft behind the hub, "
                  "small boss toward the bearing.",
             parts=["reel_hub", "reel_hub_flange", "reel_brg_1", "reel_brg_2", "reel_s", "reel_t"]),
        dict(title="Brake crank, rocker and bar",
             text="Grease the crank boss low on the left of the wall front. Slide the black bell crank onto it: the "
                  "short arm lies in the recess in the wall and points right, the long arm sticks straight out from "
                  "the wall. Put the printed washer on the boss and screw the M3 x 20 into it: the crank must swing "
                  "freely. Screw the M3 x 10 FIRM screw into the rocker's leg from the front with its nut on the front "
                  "face, until 0.35 mm of its tip shows at the back. Hang the yellow rocker between the two ears at the "
                  "top with an M3 x 25 screw and nut (snug, it must swing); its round hole goes over the small boss, "
                  "its leg rests on the small pad on the wall, the FIRM screw tip just above the crank's short arm. "
                  "Screw two M3 x 8 screws into the rocker's two bosses from behind, until their tips stand 0.5 mm "
                  "proud of the front. Lay the brake bar in the two U guides, its raised lug against the crank's long "
                  "arm, nose toward the drum. Close each guide with its cap and two M3 x 16 screws. Push the nose: "
                  "the crank swings and the rocker leg moves away from the wall; let go and it all springs back.",
             parts=["reel_crank", "reel_crank_washer", "reel_crank_screw", "reel_firm_screw", "reel_rocker",
                    "reel_rocker_pin", "reel_trim_screws", "reel_bar", "reel_bar_cap_1", "reel_bar_cap_2",
                    "reel_bar_screws"]),
        dict(title="Reel shaft into the wall",
             text="Feed the reel shaft's thin end through the rocker's hole and the wall bushing from the front. "
                  "From behind, slide the 20-tooth reel gear onto the D-flat, hub toward the wall, and snap its "
                  "C-clip on. Swing the arm toward the reel gear (to its stop): the idler meshes it.",
             parts=["reel_gr", "reel_s_clip"]),
        dict(title="T2 take-off",
             text="Push the T2 stub through its wall bushing from the front with the 40-tooth T2 gear and the 20-tooth "
                  "T bevel on its flats (bevel teeth facing you): the T2 gear meshes the clutch hub's teeth. Snap its "
                  "C-clip on behind the wall. Turn the clutch hub by hand: T2 turns the same speed.",
             parts=["reel_t2", "reel_gt2", "reel_bt2"]),
        dict(title="SW_ARM contact",
             text="Battery unplugged. Build the printed contact (see the contact switch page): nut pressed in the base "
                  f"trap, lever screwed onto the pad with 2 x M3 x {si['clamp_screw_len']} self tapping screws, "
                  f"M3 x {si['screw_len']} contact screw with its jam nut in the lever. Screw the switch base to the "
                  "bracket with 4 x M3 x 8 self tapping screws, and the bracket to the back of the wall below the gears "
                  "with two M3 x 8. SW_ARM is closed while the cup is away from home and opens at home: the lever rides "
                  "a raised track on the back of the cup gear and drops into a notch in it at home. Turn the cup to "
                  "home (the rear cup's eyelet at the bottom): the lever tip sits free in the notch. Turn the contact "
                  "screw until its head just touches the nut, then back it off "
                  f"{si['gap'] * 2:.1f} turns (gap about {si['gap']:.1f} mm, 0.5 mm per turn) and lock the jam nut.",
             parts=["reel_sw_bracket", "reel_sw_base", "reel_sw_lever", "reel_sw_hw"]),
        dict(title="Lane tabs and the lane T front post",
             text="Screw the lane C tab and the lane K tab to the wall front, each with two M3 x 10 (they tap into "
                  "the wall). Screw the lane T front post to the deck extension with two M3 screws into the inserts.",
             parts=["reel_tab_c", "reel_tab_k", "reel_post_t"]),
        dict(title="Reel unit: P1, sprag #2, capstan tube",
             text="Screw the P1 leg to the deck extension and bolt P1 onto it (one M3 x 16). Slide the capstan tube in "
                  "from the FRONT, D-flat end first, until its collar sits on P1's bushing.",
             parts=["reel_p1", "reel_p1_leg", "reel_c"]),
        dict(title="Lanes along the deck",
             text="Slide each bevel onto its lane's front end (teeth toward the front). Feed lane C (d12) from the "
                  "front through the lane C tab and P1, its bevel meshing the C bevel; lane K (d8) through the lane K "
                  "tab and P1, its bevel meshing the K2 bevel; lane T (d8) through the front post and the P1 leg, its "
                  "bevel meshing the T2 bevel. Slide the lane T rear bevel onto lane T (teeth toward the front).",
             parts=["reel_lane_c", "reel_bcx", "reel_lane_k", "reel_bkx", "reel_lane_t", "reel_btx", "reel_bt3x"]),
        dict(title="Cheeks, T bracket, spool and spool shaft",
             text="Press a 608 into each cheek. Screw both cheeks and the T bracket to the deck extension (M3 into the "
                  "inserts). Hold the spool between the cheeks and push the spool shaft from the inboard side through "
                  "the T bracket, the inboard 608, the spool (flat in the flat) and the outboard 608; slide the "
                  "16-tooth gear onto its inboard end. Put T3 with its bevel and 16-tooth gear into the T bracket: its "
                  "bevel meshes the lane T rear bevel. Press the idler axle through the 34-tooth take-up idler into "
                  "the bracket so it meshes both 16-tooth gears.",
             parts=["reel_cheek_i", "reel_cheek_o", "reel_brg_3", "reel_brg_4", "reel_t_bracket",
                    "reel_spool", "reel_ss", "reel_gs", "reel_t3", "reel_bt3", "reel_gt3", "reel_ti", "reel_ti_axle"]),
        dict(title="Drum, cup and the C plane",
             text="Slide the green drum onto the front of the capstan tube, flange first, key slot on the rib. Push "
                  "the rear cup shaft through the tube from the FRONT, plain end first. Slide the wrap cup onto its "
                  "front end, flat on the flat, eyelet at the BOTTOM, and push Harry's small C-clip onto the neck. "
                  "Behind P1: slide G_c (20 teeth, m2) onto the tube's flat and clip it; slide G_a onto lane C's flat. "
                  "Press sprag #2 (CSK20PP) into the 58-tooth idler's hub, key slot on the printed rib. Check the arrow: "
                  "the idler must turn freely the way it turns when the capstan winds in and lock the other way (turn "
                  "the capstan by hand once assembled: wind-in free, payout locked). Push the keyed I_c axle from the "
                  "front through P1 and the sprag (key in the keyway, flat in P1's flat) and clip it behind the idler.",
             parts=["reel_drum", "reel_kc", "reel_cup", "reel_cup_clip", "reel_gc2", "reel_c_clip", "reel_ga",
                    "reel_sprag", "reel_icr", "reel_icr_axle"]),
        dict(title="K plane, P2 and the line",
             text="Slide G_k (20 teeth) onto the rear cup shaft and G_a' onto lane K. Press the I_k axle through the "
                  "76-tooth idler into P2's front face. Put the three spacer tubes on P1's screws, offer P2 up so the "
                  "cup shaft and lane K enter its journals and I_k meshes both gears, and bolt it to the spacers, its "
                  "leg and the two cheeks; clip the cup shaft behind P2. Turn the cup to home (eyelet at the bottom). "
                  "Wind the line onto the spool (line over the TOP, leaving toward the front), lead it under the "
                  "capstan and through the cup eyelet from behind, then straight forward to the pulley at the rail's "
                  "rear end. Set LIGHT with the drum at LAUNCH: turn both trim screws the same amount until 25 g hung "
                  "on the line end just pulls line off slowly (0.25 N) and 20 g does not. Set FIRM last, with the drum "
                  "in REEL: the line must slip at 5.0 N on the luggage scale (FIRM screw in = more).",
             parts=["reel_gkc", "reel_ga2", "reel_ikr", "reel_ikr_axle", "reel_spacer_1", "reel_spacer_2",
                    "reel_spacer_3", "reel_p2", "reel_p2_leg", "reel_kc_clip"]),
    ]
