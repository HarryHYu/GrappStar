"""GrappStar base: launcher + aim train (worker cad-launch). Module contract: base/cad/README.md.

Harry's v10 launcher sized to the KEDGE v30 dart:
- rail pivots on the main axis; drive gear (sprag #1 inside) on the LAUNCH SLEEVE turns the big gear, which rides on
  the rail on its own pin and meshes the rack under the carriage; one big gear turn = cock + fire + reset;
- springs pull the carriage forward (direct, 1:1), the dart rides in a half pipe cradle on the carriage;
- aim: v10 crank-rocker (scaled), see launch_aim.py.

Frames
- World: x forward, y left, z up (cad-infra). Main axis along +y at (0, *, AXIS_Z).
- Rail frame (joint launch_rail): origin on the main axis at the dart plane y = Y0. Local x = u (along the rail,
  muzzle +), y = y - Y0, z = w (up, perpendicular to the rail). Joint value = pitch (deg, muzzle up +).
- All rail geometry below is written in (u, y, w).

Numbers: launch_kin.py (sizing), BASE_SPEC via P.LAUNCH / P.AIM / P.LAYOUT when present.
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
import launch_switch as SW                                     # noqa: E402

try:
    import launch_gears as G                                   # noqa: E402
except Exception:                                              # pragma: no cover
    G = None
try:
    import launch_aim as AIM                                   # noqa: E402  aim train (parts_aim, joints_aim, ...)
except ImportError:                                            # pragma: no cover
    AIM = None


def _aim(fn, P, *a, default=()):
    f = getattr(AIM, fn, None) if AIM is not None else None
    return list(f(P, *a)) if f is not None else list(default)

# ------------------------------------------------------------------------------------------------ layout numbers


def _get(P, block, key, default):
    b = getattr(P, block, None) if P is not None else None
    if isinstance(b, dict) and key in b and b[key] is not None:
        return b[key]
    return default


def L(P):
    """All launcher geometry numbers in one dict (derived from P where the spec has them)."""
    d = K.lp(P)
    g = K.gears(P)
    sp = K.spring(P)
    main = None
    lay = getattr(P, "LAYOUT", None) if P is not None else None
    if isinstance(lay, dict):
        main = lay.get("main") or lay.get("MAIN")
    axis_z = _get(P, "LAUNCH", "axis_z", 165.0)
    if isinstance(main, dict) and main.get("origin") is not None:
        axis_z = float(main["origin"][2])
    n = dict(
        axis_z=axis_z,
        y_flange=_get(P, "LAUNCH", "sleeve_y0", 101.0),      # world y of the sleeve flange face (drum_NOTES 3b)
        Y0=_get(P, "LAUNCH", "rail_y", 140.0),               # world y of the rail / dart centre plane
        m=g["m"], rb=g["r_big"], rd=g["r_drive"], cd=g["cd"], face=d["face"],
        z_big=d["z_big"], z_drive=d["z_drive"], n_present=g["n_present"],
        stroke=K.stroke(P), preload=d["preload"],
        spring_od=sp["od"] if "od" in sp else 16.0, spring_wire=sp.get("wire", 1.4),
        spring_free=sp.get("free_len", 165.1),
    )
    m = n["m"]
    n["w_rp"] = n["rb"] + 0.1                                  # rack pitch line height
    # tooth 0 sits N_LEAD pitches before the mesh centre at rest so that it is the FIRST tooth to touch the rack
    # (rack tooth 0) while the carriage is held on its stop (a rotating gear cannot enter a stationary rack at the
    # mesh centre: the teeth within about 25 deg of it would cut the rack). Checked by the reset audit states.
    n["n_lead"] = d["n_lead"]
    n["fta"] = -90.0 + n["n_lead"] * 360.0 / n["z_big"]           # gear-local angle of tooth 0 at rest
    n["w_rtip"] = n["w_rp"] - m                                # rack tooth tips (towards the gear)
    n["w_rroot"] = n["w_rp"] + 1.45 * m
    n["w_deck0"], n["w_deck1"] = n["w_rroot"], n["w_rroot"] + 7.0
    n["w_dart"] = 85.0                                          # dart axis
    n["r_hull"] = 21.0
    n["w_belly"] = n["w_dart"] - n["r_hull"]
    # track (rail) section
    n["shelf_top"] = n["w_deck0"] - 0.6                        # carriage shoes rest on this (contact in rest)
    n["shoe_w0"], n["shoe_w1"] = n["shelf_top"], n["shelf_top"] + 6.6
    n["lip_w0"] = n["shoe_w1"] + 0.4
    n["lip_w1"] = n["lip_w0"] + 4.0
    n["y_shoe0"], n["y_shoe1"] = 15.5, 21.5
    n["y_lip0"], n["y_wall0"], n["y_wall1"] = 18.0, 22.0, 28.0
    n["w_floor0"] = 30.0
    # carriage along u (home position of its rear face)
    n["pusher_t"] = 8.0
    n["u_mesh"] = n["cd"]
    n["c_len"] = n["stroke"] + 22.0                            # carriage length (rack + ends)
    n["u_c_home"] = n["u_mesh"] - 12.0                        # carriage rear face at HOME
    n["u_tail_home"] = n["u_c_home"] + n["pusher_t"]           # dart tail at HOME
    n["u_track0"] = n["u_c_home"] - n["stroke"] - 8.0          # rail rear end
    n["u_track1"] = 190.0                                      # shoe track front end
    n["y_spring"], n["w_spring"] = 38.0, 60.0
    n["u_peg_c"] = 6.0                                         # spring peg position on the carriage (from rear face)
    n["L_home"] = n["spring_free"] + n["preload"]              # hook pin distance at HOME
    n["u_anchor"] = n["u_c_home"] + n["u_peg_c"] + n["L_home"]
    n["u_rail1"] = n["u_anchor"] + 10.0                        # rail front end
    n["split"] = (-30.0, 140.0)                                # rail piece splits (u)
    # stop
    n["stop_y0"], n["stop_y1"] = 9.5, 15.0
    n["tab_w0"] = n["w_floor0"] + 14.0
    return n


def switch_frames(P):
    """Printed contact switches: name -> dict(M = switch local -> carrier joint frame, carrier joint, z_s, screw)."""
    n = L(P)
    out = {}
    # SW_HOME on the right rail lip, pressed by the right spring arm when the carriage is on its stop
    u_tip = n["u_c_home"] + n["pusher_t"]                    # stop tab front face at HOME
    w_base = n["w_floor0"] + 1.0
    zs = (n["tab_w0"] + 5.0) - w_base
    out["home"] = dict(M=tf.T(u_tip - SW.LEAF_T + 6.0, n["y_shoe0"] - 0.5 - 4.0, w_base),
                       joint="launch_rail", z_s=zs, screw=6.0, x0=-3.0, width=8.0, top=3.5,
                       where="rail mid piece, left of the big gear, pressed by the carriage stop tab at HOME")
    # SW_SEATED on the carriage shelf behind the pusher, tip flush with the pusher face, pressed by the dart tail
    zs2 = 70.0 - n["w_deck1"]
    M2 = tf.mul(tf.T(n["u_c_home"] - 2.8 + 0.0, 0, n["w_deck1"]), tf.Rz(180))
    out["seated"] = dict(M=M2, joint="launch_carriage", z_s=zs2, screw=12.0, x0=-2.8, width=None, top=5.0,
                         where="carriage shelf behind the pusher, pressed by the dart tail cap")
    return out


# ------------------------------------------------------------------------------------------------ helpers
def bed(solid, R=None, rz=0.0):
    """print_tf: rotate by R (4x4), then about z by rz deg, then translate so the part sits at z = 0 centred."""
    M = tf.mul(tf.Rz(rz), R if R is not None else tf.I4)
    lo, hi = prim.bbox(prim.place(solid, M))
    return tf.mul(tf.T(-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]), M)


def B(u0, u1, y0, y1, w0, w1):
    return prim.box(u0, u1, y0, y1, w0, w1)


def Bs(u0, u1, y0, y1, w0, w1):
    """Box mirrored in y (both sides), y0 < y1 > 0."""
    return B(u0, u1, y0, y1, w0, w1) + B(u0, u1, -y1, -y0, w0, w1)


def cyl_y(r, y0, y1, u=0.0, w=0.0, seg=None):
    return prim.cylY(r, y0, y1, x=u, z=w, seg=seg)


def cyl_u(r, u0, u1, y=0.0, w=0.0, seg=None):
    return prim.cylX(r, u0, u1, y=y, z=w, seg=seg)


def cyl_w(r, w0, w1, u=0.0, y=0.0, seg=None):
    return prim.cyl(r, w1 - w0, x=u, y=y, z=w0, seg=seg)


def _gear_y(solid_z, y0):
    """Gear built along +z (face 0..h) -> axis along +y starting at y0 (rail frame)."""
    return prim.place(solid_z, tf.mul(tf.T(0, y0, 0), tf.Rx(-90)))


def _spur(m, z, face, bore=None, key=None):
    if G is not None:
        return G.spur(m, z, face, bore=bore, key=key)
    s = prim.cyl(m * z / 2 + m, face)
    if bore:
        s = s - prim.cyl(bore / 2, face + 2, z=-1)
    return s


# ------------------------------------------------------------------------------------------------ parts
def rail_solid(P):
    n = L(P)
    u0, u1 = n["u_track0"], n["u_rail1"]
    t1 = n["u_track1"]
    s = []
    # side walls + lips + shelves along the track
    s.append(Bs(u0, t1, n["y_wall0"], n["y_wall1"], n["w_floor0"], n["lip_w1"]))
    s.append(Bs(u0, t1, n["y_lip0"], n["y_wall0"], n["lip_w0"], n["lip_w1"]))
    s.append(Bs(u0, t1, n["y_shoe0"] - 0.5, n["y_wall0"], n["w_floor0"], n["shelf_top"]))
    # cross ties under the carriage path (clear of the drive gear and big gear)
    for uc in (u0 + 6, -110.0, -60.0, 150.0, t1 - 6):
        s.append(B(uc - 6, uc + 6, -n["y_wall1"], n["y_wall1"], n["w_floor0"] - 6, n["w_floor0"]))
    # cheeks: pivot boss on the sleeve + big gear pin boss, joined to the shelves
    for sg in (1, -1):
        ya, yb = (9.0, 17.0) if sg > 0 else (-17.0, -9.0)
        boss_p = cyl_y(21.0, ya, yb, 0.0, 0.0)
        boss_g = cyl_y(14.0, ya, yb, n["cd"], 0.0)
        web = prim.union([B(-21, n["cd"], ya, yb, -8, 8), B(-21, 21, ya, yb, 0, n["w_floor0"] + 1),
                          B(n["cd"] - 14, n["cd"] + 14, ya, yb, 0, n["w_floor0"] + 1),
                          B(-21, n["cd"] + 14, ya, yb, 18, n["w_floor0"] + 1)])
        s.append(boss_p + boss_g + web)
    # front anchor posts and the tie under the muzzle
    ua = n["u_anchor"]
    ys, wsp = n["y_spring"], n["w_spring"]
    for sg in (1, -1):
        s.append(B(t1 - 1, ua + 8, sg * 22.0 if sg > 0 else -46.0, 46.0 if sg > 0 else -22.0,
                   n["w_floor0"], n["w_floor0"] + 10))
        s.append(B(ua - 8, ua + 8, sg * 30.0 if sg > 0 else -46.0, 46.0 if sg > 0 else -30.0,
                   n["w_floor0"], wsp - 12))
    s.append(B(ua - 8, ua + 8, -46, 46, n["w_floor0"] - 6, n["w_floor0"]))
    # carriage front stop blocks (the carriage stop tab hits their rear face at HOME)
    ust = n["u_c_home"] + 8.0
    s.append(B(ust, ust + 27.0, -n["y_shoe0"], -(n["stop_y0"] - 0.5), n["w_floor0"], n["tab_w0"] + 6))
    s.append(B(n["cd"], 111.0, 9.0, 17.0, 18.0, n["w_floor0"] + 1.0))          # SW_HOME seat (left)
    # mid piece cross tie (behind the drive gear, under the carriage tab path)
    s.append(B(n["split"][0] + 1.0, n["split"][0] + 10.0, -n["y_wall1"], n["y_wall1"], n["w_floor0"] + 6, n["tab_w0"] - 0.5))
    # split flanges: plates on the outer wall faces either side of each split, 2 x M3 x 16 along u (nut on the far side)
    for us in n["split"]:
        s.append(Bs(us - 7.0, us + 7.0, n["y_wall1"] - 0.01, n["y_wall1"] + 6.0, n["w_floor0"] + 2.0, n["w_floor0"] + 17.5))
    solid = prim.union(s)
    for us in n["split"]:
        for sg in (1, -1):
            for wb in FLANGE_W:
                solid = solid - cyl_u(1.7, us - 8.0, us + 8.0, sg * (n["y_wall1"] + 3.0), wb)
    # bores: sleeve journals (d20 + running clearance) and the big gear pin
    solid = solid - cyl_y(10.0 + 0.2, -30, 30, 0.0, 0.0) - cyl_y(5.0 + 0.2, -30, 30, n["cd"], 0.0)
    # slot in both cheek bores so the sleeve's key rib passes when the sleeve is pushed in (plain bearing otherwise)
    solid = solid - B(-(RIB_W + 0.6) / 2, (RIB_W + 0.6) / 2, -30, 30, -(RIB_R + 0.5), -8.0)
    # anchor pins (steel 6 mm or printed): vertical holes through the posts
    for sg in (1, -1):
        solid = solid - cyl_w(3.1, n["w_floor0"] - 1, wsp, ua, sg * ys)
    # SW_HOME mounting pilots (M3 self tap)
    f = switch_frames(P)["home"]
    for hx in SW.mount_holes(f["x0"], f["z_s"]):
        q = tf.apply(f["M"], np.array([[hx, 0.0, 0.0]]))[0]
        solid = solid - cyl_w(1.3, q[2] - 10.0, q[2] + 1.0, q[0], q[1])
    # stop pad pockets
    solid = solid - B(ust - 1.0, ust + PAD_L, -(n["y_shoe0"] - 0.5), -n["stop_y0"], n["tab_w0"] - 8, n["tab_w0"] + 6.1)
    return solid


def rail_pieces(P):
    """Split the rail at n['split'] into rear / mid / front printable pieces (bolted flanges: TODO M3)."""
    n = L(P)
    r = rail_solid(P)
    a, b = n["split"]
    big = 2000
    rear = r ^ B(-big, a, -big, big, -big, big)
    mid = r ^ B(a, b, -big, big, -big, big)
    front = r ^ B(b, big, -big, big, -big, big)
    return dict(rear=rear, mid=mid, front=front)


def carriage_solid(P):
    n = L(P)
    m = n["m"]
    Lc = n["c_len"]
    s = []
    # rack spine with teeth (pitch line at w_rp, teeth down), length = stroke + margin
    rack_n = n["n_present"]
    u_first = n["u_mesh"] - n["u_c_home"] + (n["n_lead"] - 0.5) * math.pi * m   # local u of rack tooth 0 (rearmost)
    if G is not None:
        rk = G.rack_2d(m, rack_n, back=n["w_deck1"] - n["w_rroot"], dedendum=1.45)
        rk = prim.extrude(rk, 12.0)                                   # 2D (x, y) -> (u, -w)? place below
        # rack_2d: pitch line y=0, teeth -y. Map 2D x -> u, 2D y -> w - w_rp, extrude z -> y
        rk = prim.place(rk, tf.mul(tf.T(u_first, 6.0, n["w_rp"]), tf.Rx(90)))
        s.append(rk)
    else:
        s.append(B(u_first - 4, u_first + rack_n * math.pi * m, -6, 6, n["w_rtip"], n["w_deck1"]))
    # deck + shoes (full length)
    s.append(B(0, Lc, -n["y_lip0"] + 0.5, n["y_lip0"] - 0.5, n["w_deck0"], n["w_deck1"]))
    s.append(Bs(0, Lc, n["y_shoe0"], n["y_shoe1"], n["shoe_w0"], n["shoe_w1"]))
    # cradle (half pipe under the belly, up to 5 below the dart axis) over the dart length
    rc_in, rc_out = n["r_hull"] + 0.4, n["r_hull"] + 3.4
    cr = cyl_u(rc_out, 0, 150, 0, n["w_dart"]) - cyl_u(rc_in, -1, 151, 0, n["w_dart"])
    cr = cr ^ B(-1, 151, -40, 40, n["w_deck0"], n["w_dart"] - 5)
    s.append(cr)
    s.append(B(0, 150, -12, 12, n["w_deck1"] - 0.5, n["w_belly"] - 0.4) - cyl_u(rc_in, -1, 151, 0, n["w_dart"]))
    # pusher at the rear (dart tail bears on its front face), tether hole d14
    pu = cyl_u(n["r_hull"] + 1.0, -n["pusher_t"], 0, 0, n["w_dart"]) ^ B(-20, 1, -30, 30, n["w_deck0"], 200)
    pu = pu + B(-n["pusher_t"], 0, -12, 12, n["w_deck0"], n["w_dart"])
    pu = pu - cyl_u(7.0, -20, 5, 0, n["w_dart"])
    s.append(pu.translate((n["pusher_t"], 0, 0)))
    # spring arms and pegs (pegs vertical at y +-y_spring)
    up = n["u_peg_c"]
    for sg in (1, -1):
        ya, yb = (12.0, n["y_spring"] + 5) if sg > 0 else (-n["y_spring"] - 5, -12.0)
        s.append(B(0, n["pusher_t"], ya, yb, n["lip_w1"] + 0.5, n["lip_w1"] + 14.0))   # behind the dart tail only
        s.append(cyl_w(3.0, n["w_spring"] - 9, n["lip_w1"] + 1, up, sg * n["y_spring"]))
        s.append(cyl_w(4.5, n["w_spring"] - 11, n["w_spring"] - 9, up, sg * n["y_spring"]))   # keeper flange
        # web from deck up to the arm at the rear (clears the lip: |y| < y_lip0)
        s.append(B(0, n["pusher_t"], 12.0 if sg > 0 else -n["y_lip0"] + 0.5, n["y_lip0"] - 0.5 if sg > 0 else -12.0,
                   n["w_deck0"], n["lip_w1"] + 14.0))
    # SW_SEATED shelf behind the pusher, shank hole through the pusher (below the tether hole), pilots
    s.append(B(-30.0, 0.5, -6.0, 6.0, n["w_deck0"], n["w_deck1"]))
    # stop tab under the rear (hits the rail stop pads at HOME)
    s.append(Bs(0, 8, n["stop_y0"], n["y_shoe0"] - 0.7, n["tab_w0"], n["w_deck0"] + 0.1))
    c = prim.union(s)
    c = c - cyl_u(1.8, -1.0, n["pusher_t"] + 1.0, 0.0, 70.0)
    f = switch_frames(P)["seated"]
    Mloc = tf.mul(tf.T(-n["u_c_home"], 0, 0), f["M"])
    for hx in SW.mount_holes(f["x0"], f["z_s"]):
        q = tf.apply(Mloc, np.array([[hx, 0.0, 0.0]]))[0]
        c = c - cyl_w(1.3, q[2] - 6.5, q[2] + 1.0, q[0], q[1])
    return c


def big_gear_solid(P):
    n = L(P)
    m = n["m"]
    if G is not None:
        cs = G.sector_gear_2d(m, n["z_big"], n["n_present"], first_tooth_angle=n["fta"])
        g = prim.extrude(cs, n["face"])
    else:
        g = prim.cyl(n["rb"] + m, n["face"])
    hub = prim.cyl(10.0, 17.0, z=-(17.0 - n["face"]) / 2)
    g = (g + hub) - prim.cyl(5.0 + 0.175, 40, z=-20)
    return g


def drive_gear_solid(P):
    n = L(P)
    sp = getattr(P, "SPRAG", None) or dict(od=47.0, w=14.0, key=6.0)
    g = _spur(n["m"], n["z_drive"], n["face"])
    hub = prim.cyl(sp["od"] / 2 + 4.0, sp["w"] + 2.0, z=-(sp["w"] + 2.0 - n["face"]) / 2)
    g = g + hub
    g = g - prim.cyl(sp["od"] / 2 + 0.05, 40, z=-20)
    g = g - prim.box(sp["od"] / 2 - 1.0, sp["od"] / 2 + 3.0, -sp["key"] / 2 - 0.1, sp["key"] / 2 + 0.1, -20, 20)
    return g


def sprag_solid(P):
    sp = getattr(P, "SPRAG", None) or dict(od=47.0, bore=20.0, w=14.0)
    return prim.tube(sp["od"] / 2, sp["bore"] / 2, sp["w"], z=-sp["w"] / 2)


def gear_pin_solid(P):
    """Snap pin for the big gear: head one end, clip groove the other (along +z, centred)."""
    pin = prim.cyl(5.0, 40.0, z=-20.0) + prim.cyl(8.0, 3.0, z=-23.0)
    pin = pin - prim.tube(6.0, 4.0, 1.6, z=18.5)
    return pin


RIB_W, RIB_R = 6.0, 12.8      # integral key rib on the sleeve (D13): 6 wide, top at r 12.8 (sprag keyway depth 2.8)
SPIGOT_BORE = 12.35           # cad-drum d12 spigot runs in this bore the whole sleeve length


def sleeve_solid(P):
    """Launch sleeve along +z from z = 0 (flange face towards the motor, world y_flange) to the outboard end.
    d20 journals, integral key rib (no keyway cut: D13) for sprag #1 and the splitter sun, flange for the drum's
    LAUNCH dog ring (drum_NOTES 3b: OD 64 x 8, 3 x M3 at r 28 with nut traps outboard, 3 lug slots 6.4 x 3.2)."""
    n = L(P)
    z_end = n["Y0"] + 52.0 - n["y_flange"]
    s = prim.cyl(10.0, z_end) + prim.cyl(32.0, 8.0)
    zg = n["Y0"] - n["y_flange"]
    z_sun1 = n["Y0"] + 42.0 - n["y_flange"]
    # two rib lengths: under sprag #1 (between the rail cheeks) and under the sun; never inside a cheek bore
    s = s + prim.box(9.0, RIB_R, -RIB_W / 2, RIB_W / 2, zg - 7.5, zg + 7.5)
    s = s + prim.box(9.0, RIB_R, -RIB_W / 2, RIB_W / 2, z_sun1 - 8.0, z_sun1)
    s = s - prim.cyl(SPIGOT_BORE / 2, z_end + 2, z=-1)
    for k in range(3):
        a = math.radians(45 + 120 * k)
        x, y = 28 * math.cos(a), 28 * math.sin(a)
        s = s - prim.cyl(1.7, 10, x=x, y=y, z=-1)
        hexr = 5.8 / 2 / 0.866
        s = s - prim.cyl(hexr, 3.0, x=x, y=y, z=8.0 - 3.0 + 0.01, seg=6)
        b = math.radians(60 + 120 * k)
        slot = prim.box(15.0, 21.5, -3.2, 3.2, -0.01, 3.2)
        s = s - prim.place(slot, tf.Rz(math.degrees(b)))
    return s


def spring_segment_solid(P, length):
    n = L(P)
    ro = n["spring_od"] / 2
    return prim.cylX(ro, 0, length) - prim.cylX(ro - n["spring_wire"], -1, length + 1)


HOOK_L = 12.0     # pin centre to the first coil


def hook_solid(P, sign=1):
    """Spring hook loop round a d6 pin (ring about w) with a neck to the coils (towards +u for sign 1)."""
    n = L(P)
    r = n["spring_wire"]
    ring = prim.tube(3.0 + 2 * r, 3.0 + 0.05, r * 2, z=-r)
    neck = prim.box(3.0, HOOK_L + 0.01, -r, r, -r, r)
    neck = neck + prim.cylX(n["spring_od"] / 2, HOOK_L - 2.0, HOOK_L) - prim.cylX(n["spring_od"] / 2 - r, HOOK_L - 3, HOOK_L + 1)
    h = ring + neck
    return h if sign > 0 else prim.place(h, tf.Rz(180))


def anchor_pin_solid(P):
    """Printed PPA-CF pin d6 (vertical) with a head on top; the spring hook sits on it."""
    n = L(P)
    h = n["w_spring"] + 6.0 - (n["w_floor0"] - 1.0)
    return prim.cyl(3.0, h) + prim.cyl(5.0, 2.5, z=h)


N_SEG = 5
FLANGE_W = (36.0, 43.5)  # w of the M3 bolts through the rail split flanges
PAD_L = 18.0          # carriage stop pad length (physics F3: 15 to 20 mm of TPU or foam)
_DART = {}


def dart_solid(P=None):
    """KEDGE v30 folded, dart frame (nose x = 0, tail x = 151.97, belly z = -21): legs and bought parts from the
    out30 meshes (launch_dart_v30_solid.npz, built by launch_dartmesh.py) plus the hull and nose as solids."""
    if "s" in _DART:
        return _DART["s"]
    from manifold3d import Manifold, Mesh
    d = np.load(os.path.join(_HERE, "launch_dart_v30_solid.npz"))
    U = Manifold(Mesh(vert_properties=np.asarray(d["v"], np.float32), tri_verts=np.asarray(d["f"], np.uint32)))
    hull = prim.cylX(21.0, 20.0, 151.97)
    nose = prim.place(prim.cyl(11.0, 20.0, r2=21.0), tf.Ry(90))
    _DART["s"] = U + hull + nose
    return _DART["s"]


def parts(P):
    n = L(P)
    out = []
    pcs = rail_pieces(P)
    cols = dict(rear="#9aa7b4", mid="#8796a5", front="#9aa7b4")
    for k, sol in pcs.items():
        out.append(Part(f"launch_rail_{k}", sol, "PLA", joint="launch_rail", place=tf.I4,
                        print_tf=bed(sol, tf.Rx(90)), color=cols[k],
                        notes=f"Rail {k} piece: shoe track, walls" + (", cheeks on the sleeve, big gear pin bosses" if k == "mid" else "")))
    cs = carriage_solid(P)
    out.append(Part("launch_carriage", cs, "PLA", joint="launch_carriage", place=tf.T(n["u_c_home"], 0, 0),
                    print_tf=bed(cs, tf.Rx(90), rz=45.0),
                    color="#d97a00", notes="Carriage: rack, shoes, dart cradle, pusher with tether hole, spring pegs, stop tab"))
    out.append(Part("launch_big_gear", big_gear_solid(P), "PPA-CF", joint="launch_big_gear",
                    place=tf.mul(tf.Rx(-90), tf.T(0, 0, -n["face"] / 2)), print_tf=tf.T(0, 0, (17.0 - n["face"]) / 2),
                    color="#3d7cc9", notes="Big gear m2.5 40T, 33 teeth present, relieved last tooth, rides on the rail"))
    out.append(Part("launch_gear_pin", gear_pin_solid(P), "PPA-CF", joint="launch_rail",
                    place=tf.mul(tf.T(n["cd"], 0, 0), tf.Rx(-90)), print_tf=bed(gear_pin_solid(P)), color="#3a3f45",
                    notes="Snap pin d10 for the big gear, C clip"))
    out.append(Part("launch_drive_gear", drive_gear_solid(P), "PPA-CF", joint="launch_drive",
                    place=tf.mul(tf.Rx(-90), tf.T(0, 0, -n["face"] / 2)), print_tf=bed(drive_gear_solid(P)), color="#2f6db5",
                    notes="Drive gear m2.5 26T, sprag #1 keyed inside"))
    out.append(Part("launch_sprag", sprag_solid(P), "bought", joint="launch_drive", place=tf.Rx(-90),
                    color="#777c84", bought_ref="CSK20PP 20x47x14 sprag #1", notes="Sprag #1: forward wiper cocks"))
    out.append(Part("launch_sleeve", sleeve_solid(P), "PPA-CF", joint="launch_sleeve",
                    place=tf.mul(tf.T(0, n["y_flange"] - n["Y0"], 0), tf.Rx(-90)), print_tf=bed(sleeve_solid(P)), color="#3a3f45",
                    notes="Launch sleeve d20 keyed: dog ring flange, sprag #1, splitter sun, journals"))
    # springs: hooks + coil segments that spread as the carriage moves
    seg_len = (n["L_home"] - 2 * HOOK_L) / N_SEG
    for sg, side in ((1, "l"), (-1, "r")):
        y = sg * n["y_spring"]
        out.append(Part(f"launch_anchor_pin_{side}", anchor_pin_solid(P), "PPA-CF", joint="launch_rail",
                        place=tf.T(n["u_anchor"], y, n["w_floor0"] - 1.0), print_tf=bed(anchor_pin_solid(P), tf.Rx(180)),
                        color="#3a3f45", notes="Spring anchor pin d6, pushed down through the front post"))
        out.append(Part(f"launch_spring_{side}_hook_a", hook_solid(P, -1), "bought", joint="launch_rail",
                        place=tf.T(n["u_anchor"], y, n["w_spring"]), color="#b0b5bb",
                        bought_ref="Century 16 x 165.1 x 1.4 extension spring", notes="Spring hook on the anchor pin"))
        out.append(Part(f"launch_spring_{side}_hook_c", hook_solid(P, 1), "bought", joint="launch_carriage",
                        place=tf.T(n["u_c_home"] + n["u_peg_c"], y, n["w_spring"]), color="#b0b5bb",
                        notes="Spring hook on the carriage peg"))
        for i in range(N_SEG):
            u_seg = n["u_c_home"] + n["u_peg_c"] + HOOK_L + i * seg_len
            out.append(Part(f"launch_spring_{side}_{i}", spring_segment_solid(P, seg_len), "bought",
                            joint=f"launch_spring_seg_{i}", place=tf.T(u_seg, y, n["w_spring"]), color="#c2c7cc",
                            notes="Spring coils (shown in segments that spread when stretched)"))
    out.append(Part("launch_stop_pad_r", B(0, PAD_L, -5.0, 0, 0, 14), "TPU", joint="launch_rail",
                    place=tf.T(n["u_c_home"] + 8.0, -n["stop_y0"], n["tab_w0"] - 8),
                    print_tf=tf.T(0, 5, 0), color="#222222", notes="Carriage stop pad"))
    for nm, f in switch_frames(P).items():
        bod = SW.body(f["z_s"], f["screw"], f["x0"], width=f["width"], top=f["top"])
        out.append(Part(f"launch_sw_{nm}", bod, "PLA", joint=f["joint"], place=f["M"], print_tf=bed(bod),
                        color="#5fae5f", notes=f"SW_{nm.upper()} printed contact: {f['where']}"))
        out.append(Part(f"launch_sw_{nm}_screw", SW.screw(f["screw"], f["z_s"]), "bought", joint=f["joint"], place=f["M"],
                        color="#444444", bought_ref=f"M3x{f['screw']:g} SHCS", notes="Contact screw, threaded through the leaf"))
        out.append(Part(f"launch_sw_{nm}_nut", SW.nut(f["z_s"]), "bought", joint=f["joint"], place=f["M"],
                        color="#444444", bought_ref="M3 nut", notes="Contact nut in its trap, wire under it"))
    out.append(Part("launch_dart", dart_solid(P), "bought", joint="launch_dart",
                    place=tf.mul(tf.T(n["u_tail_home"] + 152.0, 0, n["w_dart"] - 0.4), tf.Rz(180)),
                    color="#e8e8e8", bought_ref="KEDGE v30 dart (reference, built separately)",
                    notes="The dart, folded, belly on the cradle, tail on the pusher"))
    out += _aim("parts_aim", P)
    return out


def joints(P):
    n = L(P)
    g = K.gears(P)
    out = [
        Joint("launch_rail", None, "rev", (0.0, n["Y0"], n["axis_z"]), (0, -1, 0), limits=(5.5, 47.5)),
        Joint("launch_sleeve", None, "rev", (0.0, n["Y0"], n["axis_z"]), (0, 1, 0)),
        Joint("launch_drive", None, "rev", (0.0, n["Y0"], n["axis_z"]), (0, 1, 0)),
        Joint("launch_carriage", "launch_rail", "slide", (0, 0, 0), (-1, 0, 0), limits=(0.0, n["stroke"] + 0.5)),
        Joint("launch_big_gear", "launch_rail", "rev", (n["cd"], 0, 0), (0, 1, 0)),
        Joint("launch_dart", "launch_carriage", "slide", (0, 0, 0), (1, 0, 0)),
    ]
    for i in range(N_SEG):
        out.append(Joint(f"launch_spring_seg_{i}", "launch_rail", "slide", (0, 0, 0), (-1, 0, 0),
                         gear=("launch_carriage", (N_SEG - i - 0.5) / N_SEG, 0.0)))
    out += _aim("joints_aim", P)
    return out


_REL_CACHE = {}


def release_table(P):
    """Carriage travel (mm) vs big gear angle (deg, negative = cocking) from the real 2D teeth: the springs press the
    rack forward against the gear, so the travel is the smallest rack position with no tooth overlap. Release = the
    first angle at which the rack can run all the way home without touching the gear. Returns dict with arrays."""
    n = L(P)
    key = (n["m"], n["z_big"], n["n_present"], round(n["stroke"], 3))
    if key in _REL_CACHE:
        return _REL_CACHE[key]
    m, rb, p = n["m"], n["rb"], math.pi * n["m"]
    gear = G.sector_gear_2d(m, n["z_big"], n["n_present"], first_tooth_angle=n["fta"])
    rack = G.rack_2d(m, n["n_present"], back=7.0, dedendum=1.45).scale((1.0, -1.0))
    u0 = -p / 2 + n["n_lead"] * p

    def ov(th, xu):
        return (gear.rotate(th) ^ rack.translate((u0 - xu, -n["w_rp"]))).area()

    def follow(th, prev):
        x = max(0.0, prev - 1.0)
        while x < prev + 4.0:
            if ov(th, x) < 1e-5:
                return float(x)
            x += 0.03
        return float(prev)

    ths, trs = [0.0], [0.0]
    th_rel = None
    th = -1.0
    th_fine = -math.degrees((n["n_present"] + n["n_lead"] - 3) * p / rb)
    prev = 0.0
    while th > -359.0:
        tr = follow(th, prev)
        if th <= th_fine and all(ov(th, x) < 1e-5 for x in np.arange(0.0, tr, 0.5)):
            th_rel = th
            break
        ths.append(th)
        trs.append(tr)
        prev = tr
        th += -1.0 if th > th_fine else -0.25
    d_mid = float(np.median(-np.radians(np.array(ths[5:-20])) * rb - np.array(trs[5:-20])))
    out = dict(theta=np.array(ths), travel=np.array(trs), theta_release=th_rel,
               travel_release=float(trs[-1]), d_mid=d_mid)
    _REL_CACHE[key] = out
    return out


def gear_angle_for_travel(P, travel):
    """Big gear angle (deg, about rail +y) for a carriage travel back from HOME (mm), from the real contact table."""
    if G is None:
        return -math.degrees(travel / L(P)["rb"])
    t = release_table(P)
    tr = np.maximum.accumulate(t["travel"])
    travel = min(float(travel), float(tr[-1]))
    return float(np.interp(travel, tr, t["theta"]))


def gear_release_angle(P):
    return release_table(P)["theta_release"] if G is not None else -math.degrees(L(P)["stroke"] / L(P)["rb"])


def _drive_phase(P):
    n = L(P)
    return G.mesh_phase(n["z_drive"], 0.0) if G is not None else 0.0


def kin(P, S):
    n = L(P)
    g = K.gears(P)
    pitch = float(np.clip(S.get("aim_deg", 6.02), 6.02, 47.0))
    cock = float(np.clip(S.get("cock", 0.0), 0.0, 1.0))
    travel = cock * n["stroke"]
    if "launch_travel" in S:
        travel = float(S["launch_travel"])
    if "launch_gear_deg" in S:
        gdeg = float(S["launch_gear_deg"])
    elif S.get("fired") and cock < 1.0:
        gdeg = gear_release_angle(P)                                # gear stays at the release angle in the shot
    else:
        gdeg = gear_angle_for_travel(P, travel)
    out = {"launch_rail": pitch, "launch_carriage": travel, "launch_big_gear": gdeg,
           "launch_drive": -gdeg * g["ratio"] - pitch + _drive_phase(P), "launch_sleeve": S.get("wiper_deg", 0.0)}
    flown = S.get("fired") and travel <= 0.0 or S.get("phase") in ("flight", "neutral", "reel_mode", "wrap", "winch")
    out["launch_dart"] = float(S["launch_dart_fly"]) if "launch_dart_fly" in S else (3000.0 if flown else 0.0)
    if AIM is not None and hasattr(AIM, "kin_aim"):
        out.update(AIM.kin_aim(P, S, pitch))
    return out


def allowed(P):
    return [
        Allow("launch_sprag", "launch_drive_gear", "sprag outer race keyed in the drive gear (press)"),
        Allow("launch_sprag", "launch_sleeve", "sprag inner race on the keyed sleeve"),
        Allow("launch_spring_*_hook_a", "launch_anchor_pin_*", "hook loop round the anchor pin"),
        Allow("launch_anchor_pin_*", "launch_rail_front", "pin in its hole"),
        Allow("launch_spring_*_hook_c", "launch_carriage", "hook loop round the carriage peg"),
        Allow("launch_stop_pad_*", "launch_rail_mid", "pad in its pocket"),
        Allow("launch_sw_*_screw", "launch_sw_home", "screw threaded into the leaf"),
        Allow("launch_sw_*_screw", "launch_sw_seated", "screw threaded into the leaf"),
    ] + _aim("allowed_aim", P)


def clearances(P):
    return [Clearance("launch_carriage", "launch_rail_*", 0.0, "carriage slides in the track"),
            Clearance("launch_big_gear", "launch_drive_gear", 0.0, "mesh")] + _aim("clearances_aim", P)


def audit_states(P):
    n = L(P)
    S = []
    for p in (6.02, 25.0, 47.0):
        for c in (0.0, 0.5, 1.0):
            S.append(dict(aim_deg=p, cock=c, phase="launch_sweep"))
    # the shot: gear at the release angle, carriage flying home
    for f in np.linspace(0, 1, 11):            # the wiper keeps turning about 16 deg (gear about 10) during the shot
        for lag in (0.0, 12.0):
            S.append(dict(aim_deg=25.0, launch_travel=n["stroke"] * (1 - f), fired=True, cock=0.5,
                          launch_gear_deg=gear_release_angle(P) - lag * f, phase="launch_shot"))
    # reset: gear turns through the gap with the carriage at HOME
    a0 = gear_release_angle(P)
    for a in np.linspace(a0, -360.0, 9):
        S.append(dict(aim_deg=25.0, launch_travel=0.0, launch_gear_deg=float(a), phase="launch_reset"))
    # the dart leaving the carriage at HOME and passing the muzzle (pitch low and high)
    for p in (6.02, 47.0):
        for d in np.arange(0.0, 320.0, 20.0):
            S.append(dict(aim_deg=p, launch_travel=0.0, launch_dart_fly=float(d), phase="launch_flight"))
    S += _aim("audit_states_aim", P)
    return S


def grounded(P):
    """The aim housing stands on the deck; everything else must connect to it (or to other subsystems' grounded
    parts) through contacts: rail cheeks on the sleeve, sleeve in the housing bushing, conrod to the crank."""
    return ["launch_aim_housing"]


def steps(P):
    """Assembly steps for the launcher and aim train (plain language, hardware per step)."""
    n = L(P)
    return [
        dict(title="Launcher 1: print the parts",
             text="PPA-CF: big gear (flat), drive gear (flat), launch sleeve (standing on its flange), big gear pin and "
                  "spring anchor pins (standing on the head), aim sun, idler, output gear, bevel, worm, crank. "
                  "PLA: rail rear, mid and front pieces (on their side), carriage (on its side, turned 45 deg on the bed), "
                  "the three switch bodies (flat), conrod, lug, aim housing and cap. TPU 95A: the carriage stop pad "
                  "(if you have no TPU, print it in PLA at 15 percent gyroid and replace it when it cracks). "
                  "0.2 mm layers, 4 walls on every loaded part.",
             parts=[]),
        dict(title="Launcher 2: join the rail",
             text="Lay the rear, mid and front rail pieces in a line on a flat table, wall faces down. Push each joint "
                  "together and fit 2 x M3 x 16 bolts with nuts through each flange pair (4 per joint, 8 in total). "
                  "Tighten evenly; the inside of the track must be smooth where the pieces meet (file any step).",
             parts=["launch_rail_rear", "launch_rail_mid", "launch_rail_front"]),
        dict(title="Launcher 3: carriage stop pad and SW_HOME",
             text="Press the stop pad into its pocket on the right side of the mid piece, just in front of the big gear "
                  "pin. Build SW_HOME: drop an M3 nut into the nut trap of the switch body, slide the stripped end of one "
                  "wire into the slot under it. Screw an M3 x 6 through the thin leaf from the tip side until the head "
                  "just touches the nut, then back it off half a turn (0.25 mm gap). Wrap the second wire under the "
                  "screw head side before tightening. Screw the switch onto the mid piece left of the gear with 2 x M3 x 10 "
                  "self-tapping into the pilot holes, leaf facing back.",
             parts=["launch_stop_pad_r", "launch_sw_home", "launch_sw_home_screw", "launch_sw_home_nut"]),
        dict(title="Launcher 4: carriage and SW_SEATED",
             text="Build SW_SEATED the same way with an M3 x 12 (its tip passes through the hole in the pusher and sits "
                  "flush with the pusher face when pressed). Screw it onto the shelf behind the pusher with 2 x M3 x 10. "
                  "Run its two wires (thin silicone wire) forward along the carriage and leave a 300 mm loop: the carriage "
                  "moves 241 mm. Slide the carriage into the rail track from the rear end, rack teeth down.",
             parts=["launch_carriage", "launch_sw_seated", "launch_sw_seated_screw", "launch_sw_seated_nut"]),
        dict(title="Launcher 5: big gear",
             text="Hold the big gear between the rail cheeks, long tooth and short (cut) tooth together, and push the "
                  "big gear pin through cheek, gear, cheek. Snap the C clip on the groove. Turn the gear by hand: it must "
                  "spin freely. Do not mesh it with the rack yet.",
             parts=["launch_big_gear", "launch_gear_pin"]),
        dict(title="Launcher 6: drive gear, sprag #1 and the launch sleeve",
             text="Press sprag #1 into the drive gear with its outer key in the gear slot. Check the direction: holding the "
                  "gear, the inner race must turn freely one way only, the way that is BACKWARD for the wiper. Hold the "
                  "drive gear between the rail cheeks, the rib slots in both cheek bores pointing down. Push the launch "
                  "sleeve in from the motor side, outboard end first: line up its short rib with the cheek slots and the "
                  "sprag keyway, push until the rib sits inside the sprag. The drive gear must mesh with the big gear.",
             parts=["launch_drive_gear", "launch_sprag", "launch_sleeve"]),
        dict(title="Launcher 7: springs",
             text="Push each spring anchor pin down through its front post. Hook the front end of each spring over its "
                  "anchor pin and the back end over the carriage peg (the carriage sits on its stop pad). The springs are "
                  "slightly stretched (preload) and pull the carriage onto the pad.",
             parts=["launch_anchor_pin_l", "launch_anchor_pin_r", "launch_spring_l_hook_a", "launch_spring_r_hook_a",
                    "launch_spring_l_hook_c", "launch_spring_r_hook_c"]
                   + [f"launch_spring_{sd}_{i}" for sd in "lr" for i in range(N_SEG)]),
        dict(title="Launcher 8: time the big gear",
             text="With the carriage on its pad, turn the drive gear FORWARD by hand (cocking direction) until the first "
                  "big gear tooth (the one after the toothless gap) just touches the rearmost rack tooth. This is HOME. "
                  "Turning on must pull the carriage back smoothly; at full cock the cut last tooth lets go. Never put "
                  "fingers in front of the carriage: hold the drive gear from the side and let the carriage run home.",
             parts=[]),
        dict(title="Aim 1: aim housing, worm, wheel, crank",
             text="Screw the aim housing to the deck (4 x M3). Build SW_RAIL_LOW like SW_HOME and screw it to the "
                  "housing where the crank bump presses it. Fit the two 608 bearings into the crank (worm wheel) hub "
                  "and slide it onto the M8 bolt through the housing outer wall; fit the nut. Lay the worm shaft with the "
                  "small bevel in its saddles and screw the cap on. Fit the countershaft with the output gear and the big "
                  "bevel; the bevels must mesh with a little play.",
             parts=["launch_aim_housing", "launch_aim_cap", "launch_aim_worm", "launch_aim_crank", "launch_aim_bolt",
                    "launch_aim_brg_1", "launch_aim_brg_2", "launch_aim_output", "launch_aim_bevel30",
                    "launch_sw_rail_low", "launch_sw_rail_low_screw", "launch_sw_rail_low_nut"]),
        dict(title="Aim 2: splitter",
             text="Slide the sun onto the sleeve rib. Put the O-ring in the swing arm hub groove, slide the arm onto the "
                  "sleeve next to the sun, fit the idler on the arm pin with its e-clip. Turn the sleeve backward by hand: "
                  "the arm must swing the idler into the output gear and drive it; forward, the arm must swing out onto "
                  "its stop and the output gear must stay still.",
             parts=["launch_aim_sun", "launch_aim_arm", "launch_aim_idler"]),
        dict(title="Aim 3: lug and conrod, rail onto the frame",
             text="Bolt the lug to the right rail wall (M3). Lower the rail assembly so the launch sleeve's outboard end "
                  "enters the aim housing bushing and the flange meets the drum's LAUNCH dog ring (3 x M3 from the drum "
                  "side, nuts in the sleeve flange traps). Join the conrod to the crank pin and the lug pin with C clips. "
                  "Turn the wiper backward slowly: the rail rises to 47 deg, then lowers to 6 deg.",
             parts=["launch_aim_lug", "launch_aim_conrod"]),
        dict(title="Launcher 9: wiring (battery unplugged)",
             text="Battery unplugged. Run the SW_HOME and SW_SEATED wires along the rail to the pivot and leave a 150 mm "
                  "loop at the main axis before they go to the frame. SW_SEATED's loop on the carriage must not reach the "
                  "rack or the gears. Each switch: one wire to its ESP32 pin, the other to GND.",
             parts=[]),
    ]
