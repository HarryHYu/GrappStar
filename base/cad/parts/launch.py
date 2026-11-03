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
        y_flange=_get(P, "LAUNCH", "sleeve_y0", 96.0),       # world y of the sleeve flange (inboard face)
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
    n["split"] = (-30.0, 112.0)                                # rail piece splits (u)
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
    solid = prim.union(s)
    # bores: sleeve journals (d20 + running clearance) and the big gear pin
    solid = solid - cyl_y(10.0 + 0.2, -30, 30, 0.0, 0.0) - cyl_y(5.0 + 0.2, -30, 30, n["cd"], 0.0)
    # anchor pins (steel 6 mm or printed): vertical holes through the posts
    for sg in (1, -1):
        solid = solid - cyl_w(3.1, n["w_floor0"] - 1, wsp, ua, sg * ys)
    # SW_HOME mounting pilots (M3 self tap)
    f = switch_frames(P)["home"]
    for hx in SW.mount_holes(f["x0"], f["z_s"]):
        q = tf.apply(f["M"], np.array([[hx, 0.0, 0.0]]))[0]
        solid = solid - cyl_w(1.3, q[2] - 10.0, q[2] + 1.0, q[0], q[1])
    # stop pad pockets
    solid = solid - B(ust - 1.0, ust + 10.0, -(n["y_shoe0"] - 0.5), -n["stop_y0"], n["tab_w0"] - 8, n["tab_w0"] + 6.1)
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


def sleeve_solid(P):
    """Launch sleeve along +z from z = 0 (flange face, world y_flange) to the outboard end. d20 with keyways."""
    n = L(P)
    z_end = n["Y0"] + 52.0 - n["y_flange"]
    s = prim.cyl(10.0, z_end) + prim.cyl(20.0, 4.0)
    s = s - prim.cyl(6.2, 24.0, z=-1)                           # main shaft d12 spigot bushing bore
    zg = n["Y0"] - n["y_flange"]
    s = s - prim.box(10.0 - 3.0, 11, -3.1, 3.1, zg - 8.0, zg + 8.0)             # sprag key
    s = s - prim.box(10.0 - 3.0, 11, -3.1, 3.1, zg + 33.0, zg + 43.0)           # sun gear key
    for k in range(3):
        a = math.radians(120 * k + 60)
        s = s - prim.cyl(1.7, 6, x=15 * math.cos(a), y=15 * math.sin(a), z=-1)
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
    out.append(Part("launch_stop_pad_r", B(0, 10, -5.0, 0, 0, 14), "TPU", joint="launch_rail",
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

    def follow(th, nom):
        xs = np.arange(max(0.0, nom - 4.0), nom + 4.0, 0.05)
        for x in xs:
            if ov(th, x) < 1e-5:
                return float(x)
        return float(nom)

    # mid-stroke offset (springs take up the backlash) measured once
    d_mid = -math.radians(-150.0) * rb - follow(-150.0, -math.radians(-150.0) * rb)
    ths, trs = [0.0], [0.0]
    th_rel = None
    th = -1.0
    th_start_fine = -math.degrees((n["n_present"] - 3) * p / rb)
    while th > -359.0:
        nom = -math.radians(th) * rb
        if th > th_start_fine:
            tr = max(0.0, nom - d_mid)
            step = -2.0
        else:
            tr = follow(th, nom)
            free = all(ov(th, x) < 1e-5 for x in np.arange(0.0, tr, 0.5))
            if free:
                th_rel = th
                break
            step = -0.25
        ths.append(th)
        trs.append(tr)
        th += step
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
    return []
