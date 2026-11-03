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
import launch_line as LN                                       # noqa: E402

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
    lay = getattr(P, "LAYOUT", None) if P is not None else None
    lay = lay if isinstance(lay, dict) else {}
    axes = lay.get("axes") or {}
    main = axes.get("main") or lay.get("main") or {}
    axis_z = float(main["origin"][2]) if main.get("origin") is not None else 165.0
    y_flange = 111.0
    for st in lay.get("stack") or []:
        if "LAUNCH sleeve" in str(st.get("part", "")):
            y_flange = float(st["y"][0])
    n = dict(
        axis_z=axis_z,
        y_flange=y_flange,                                    # world y of the sleeve flange face (meets the dog ring)
        Y0=float(lay.get("rail_plane_y", 140.0)),            # world y of the rail / dart centre plane
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
    n["u_peg_c"] = 4.0                                         # spring peg position on the carriage (from rear face)
    n["L_home"] = n["spring_free"] + n["preload"]              # hook pin distance at HOME
    n["u_anchor"] = n["u_c_home"] + n["u_peg_c"] + n["L_home"]
    n["u_rail1"] = n["u_anchor"] + 10.0                        # rail front end
    n["split"] = (-30.0, 140.0)                                # rail piece splits (u)
    # planes across the rail (y): big gear sector layer (rack), full layer (drive gear), hub, cheeks
    n["y_sec"] = (-6.0, 6.0)
    n["y_full"] = (6.3, 14.3)
    n["hub_y"] = (-8.5, 19.0)
    n["y_chA"] = (-17.0, -9.0)
    n["y_chB"] = (19.5, 27.5)
    # spring arms (they hit the stop pads) and pads
    n["arm_w0"], n["arm_w1"] = 72.0, 80.0
    n["u_stop"] = n["u_c_home"] + n["pusher_t"]               # arm front faces at HOME
    n["pad_y"] = (33.0, 41.0)
    n["pad_w"] = (69.0, 77.5)
    return n


SEAT_TIP = (15.0, 86.0)       # SW_SEATED tip (y, w) on the pusher face


def arm_top(P):
    n = L(P)
    return n["lip_w1"] + 14.0


def switch_frames(P):
    """Generic printed contact switches (launch_switch.py): name -> dict(M = switch local -> carrier joint frame, ...).
    SW_GAP reads the BIG GEAR at its rest angle: a 2 mm cam bump on the gear web (right face, y -6) presses the tip
    when the gear is within about +-5 deg of rest. At rest the gear sits in its toothless gap, so the springs hold the
    carriage on its stop: gear in its gap."""
    n = L(P)
    out = {}
    u_t, w_t = home_bump_point(P)
    zs = 7.0
    w_base = w_t - zs
    y_leaf = n["y_sec"][0] - BUMP_H - 4.8                     # tip (x -4.8) pressed at the bump top, y = -8
    M = tf.mul(tf.T(u_t, y_leaf, w_base), tf.Rz(-90))
    out["gap"] = dict(M=M, joint="launch_rail", z_s=zs, screw=6.0, x0=-3.0, width=None, top=5.0,
                       where="right rail cheek, pressed by a cam bump on the big gear at its rest angle (gear in its gap)")
    return out


BUMP_H = 2.0                  # SW_GAP cam bump height on the big gear web
BUMP_R = 29.7                 # its radius from the gear pin
BUMP_DEG = 5.0                # half width (deg)


def home_bump_point(P):
    """Rail (u, w) of the SW_GAP tip: below and behind the gear pin, on the bump circle."""
    n = L(P)
    a = math.radians(225.0)
    return n["cd"] + BUMP_R * math.cos(a), BUMP_R * math.sin(a)


def best_bed(solid, rz=0.0):
    """print_tf with the largest flat face on the bed among the six axis orientations."""
    cands = [tf.I4, tf.Rx(90), tf.Rx(-90), tf.Rx(180), tf.Ry(90), tf.Ry(-90)]
    best, area = None, -1.0
    for R in cands:
        M = tf.mul(tf.Rz(rz), R)
        v, f = prim.mesh_of(prim.place(solid, M))
        z0 = v[:, 2].min()
        tri = v[f]
        nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        flat = (np.abs(tri[:, :, 2] - z0).max(axis=1) < 0.01) & (nrm[:, 2] < 0)
        a = 0.5 * np.linalg.norm(nrm[flat], axis=1).sum()
        lo, hi = prim.bbox(prim.place(solid, M))
        if hi[0] - lo[0] > 256 or hi[1] - lo[1] > 256 or hi[2] - lo[2] > 260:
            continue
        if a > area:
            best, area = R, a
    return bed(solid, best if best is not None else tf.I4, rz)


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
    u0 = n["u_track0"]
    t1 = n["u_track1"]
    s = []
    # side walls + lips + shelves along the track
    s.append(Bs(u0, t1, n["y_wall0"], n["y_wall1"], n["w_floor0"], n["lip_w1"]))
    s.append(Bs(u0, t1, n["y_lip0"], n["y_wall0"], n["lip_w0"], n["lip_w1"]))
    s.append(Bs(u0, t1, n["y_shoe0"] - 0.5, n["y_wall0"], n["w_floor0"], n["shelf_top"]))
    # cross ties under the carriage path (clear of the drive gear and big gear)
    for uc in (u0 + 6, -110.0, -60.0, 150.0, t1 - 6):
        s.append(B(uc - 6, uc + 6, -n["y_wall1"], n["y_wall1"], n["w_floor0"] - 6, n["w_floor0"]))
    # cheeks: pivot boss on the sleeve + big gear pin boss; webs run out to the shelf/wall and up to w 45 (review 1)
    for (ya, yb), y_out in ((n["y_chA"], -n["y_wall0"]), (n["y_chB"], n["y_wall0"])):
        boss_p = cyl_y(21.0, ya, yb, 0.0, 0.0)
        boss_g = cyl_y(14.0, ya, yb, n["cd"], 0.0)
        web = prim.union([B(-21, n["cd"], ya, yb, -8, 8), B(-21, 21, ya, yb, 0, 45.0),
                          B(n["cd"] - 14, n["cd"] + 14, ya, yb, 0, 45.0),
                          B(-21, n["cd"] + 14, ya, yb, 18, 45.0),
                          B(34.0, n["cd"] + 14, min(ya, y_out), max(yb, y_out), 18, 45.0)])     # clear of the sleeve flange r 32
        s.append(boss_p + boss_g + web)
    # SW_GAP ledge and web on the right cheek
    u_t, w_t = home_bump_point(P)
    yA0 = n["y_chA"][0]
    s.append(B(u_t - 13, u_t + 13, yA0, n["y_chA"][1], w_t - 13.0, -7.9))
    s.append(B(u_t - 6.5, u_t + 6.5, -40.0, n["y_chA"][1], w_t - 13.0, w_t - 7.0))
    # front anchor posts (raised to the hook, with a top strap: the pin is held in double shear) and the muzzle tie
    ua = n["u_anchor"]
    ys, wsp = n["y_spring"], n["w_spring"]
    yout = Y_SPRING2 + 8.0                       # posts reach the second-pair provision (D5)
    for sg in (1, -1):
        yi, yo = sorted((sg * 22.0, sg * yout))
        s.append(B(t1 - 1, ua + 8, yi, yo, n["w_floor0"], n["w_floor0"] + 10))
        yi2, yo2 = sorted((sg * 30.0, sg * yout))
        s.append(B(ua - 8, ua + 8, yi2, yo2, n["w_floor0"], wsp - 5.0))            # lower post up to under the hook
        s.append(B(ua - 8, ua + 8, yi2, yo2, wsp + 5.0, wsp + 11.0))               # top strap over the hook
        for yc in (0.5 * (n["y_spring"] + Y_SPRING2), yout - 1.5):                 # strap side walls between hooks
            s.append(B(ua + 2, ua + 8, *sorted((sg * (yc - 1.5), sg * (yc + 1.5))), n["w_floor0"], wsp + 11.0))
    s.append(B(ua - 8, ua + 8, -yout, yout, n["w_floor0"] - 6, n["w_floor0"]))
    # carriage stop: TPU pads on brackets over the springs, hit by both spring arms (no load on any switch)
    us = n["u_stop"]
    for sg in (1, -1):
        y0, y1 = sorted((sg * 25.0, sg * n["pad_y"][1] + sg * 2.0))
        s.append(B(us + PAD_L, us + PAD_L + 14.0, y0, y1, n["pad_w"][0] - 1.0, n["pad_w"][1] + 2.5))
        s.append(B(us + PAD_L, us + PAD_L + 14.0, *sorted((sg * 25.0, sg * n["y_wall1"])), n["lip_w1"] - 0.01, n["pad_w"][1] + 2.5))
    # mid piece cross tie (behind the drive gear)
    s.append(B(n["split"][0] + 1.0, n["split"][0] + 10.0, -n["y_wall1"], n["y_wall1"], n["w_floor0"] + 6, 43.5))
    # split flanges: plates on the outer wall faces either side of each split, 2 x M3 x 16 along u (nut on the far side)
    for usp in n["split"]:
        s.append(Bs(usp - 7.0, usp + 7.0, n["y_wall1"] - 0.01, n["y_wall1"] + 6.0, n["w_floor0"] + 2.0, n["w_floor0"] + 17.5))
    s.append(LN.rail_add(n))
    solid = prim.union(s)
    for usp in n["split"]:
        for sg in (1, -1):
            for wb in FLANGE_W:
                solid = solid - cyl_u(1.7, usp - 8.0, usp + 8.0, sg * (n["y_wall1"] + 3.0), wb)
    # bores: sleeve journals (d20 + running clearance) and the big gear pin
    solid = solid - cyl_y(10.0 + 0.2, -30, 30, 0.0, 0.0) - cyl_y(5.0 + 0.2, -30, 30, n["cd"], 0.0)
    # slot in both cheek bores so the sleeve's key ribs pass when the sleeve is pushed in (plain bearing otherwise)
    solid = solid - B(-(RIB_W + 0.6) / 2, (RIB_W + 0.6) / 2, -30, 30, -(RIB_R + 0.5), -8.0)
    # anchor pins: vertical holes through post and strap
    for sg in (1, -1):
        for yy in (ys, Y_SPRING2):                                                 # second set: provision, not fitted
            solid = solid - cyl_w(3.1, n["w_floor0"] - 1, wsp + 12.0, ua, sg * yy)
    # stop pad pockets in the brackets (4 deep)
    for sg in (1, -1):
        y0, y1 = sorted((sg * n["pad_y"][0], sg * n["pad_y"][1]))
        solid = solid - B(us + PAD_L - 0.01, us + PAD_L + 4.0, y0, y1, n["pad_w"][0], n["pad_w"][1])
    # clear the sleeve flange (r 32, world y 111..119) and the dog ring: nothing of the rail within r 33.5 there
    solid = solid - cyl_y(33.5, -60.0, n["y_flange"] + 8.5 - n["Y0"], 0.0, 0.0)
    # SW_GAP pocket in the right cheek + mounting pilots
    f = switch_frames(P)["gap"]
    solid = solid - B(u_t - 6.0, u_t + 6.0, -30.0, n["y_chA"][1] + 0.01, w_t - 7.01, w_t + 6.0)
    for hx in SW.mount_holes(f["x0"], f["z_s"]):
        q = tf.apply(f["M"], np.array([[hx, 0.0, 0.0]]))[0]
        solid = solid - cyl_w(1.3, q[2] - 6.5, q[2] + 1.0, q[0], q[1])
    # SW_SEATED (left lip) and SW_HOME (right lip) lever seats: head pocket, nut pocket with an outboard slot, root pilot
    for side in ("seated", "home"):
        st = seat_geom(P, side)
        sg = st["sg"]
        solid = solid - cyl_w(3.4, st["w_head0"] - 0.3, n["lip_w1"] + 1.0, st["u_tip"], st["y"])
        hexr = 5.8 / 2 / 0.866
        solid = solid - prim.cyl(hexr, SW.NUT_T + 0.4, x=st["u_tip"], y=st["y"], z=st["w_nut0"] - 0.2, seg=6)
        solid = solid - B(st["u_tip"] - 2.9, st["u_tip"] + 2.9, *sorted((st["y"], sg * (n["y_wall1"] + 1))),
                          st["w_nut0"] - 0.2, st["w_nut0"] + SW.NUT_T + 0.2)
        solid = solid - cyl_w(1.3, n["lip_w1"] - 9.0, n["lip_w1"] + 1.0, st["u_root"] + 2.5, st["y"])
    # aim lug: 2 x M3 self tap pilots through the left wall into the shelf (lug bolting plate at u 100..130)
    for uu in LUG_BOLTS_U:
        solid = solid - cyl_y(1.3, n["y_shoe0"] + 1.0, n["y_wall1"] + 0.5, uu, 41.0)
    solid = solid - LN.rail_cut(n)
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
    # rack spine with teeth (pitch line at w_rp, teeth down)
    rack_n = n["n_present"]
    u_first = n["u_mesh"] - n["u_c_home"] + (n["n_lead"] - 0.5) * math.pi * m   # local u of rack tooth 0 (rearmost)
    if G is not None:
        rk = G.rack_2d(m, rack_n, back=RACK_BACK, dedendum=1.45)
        rk = prim.extrude(rk, 12.0)
        rk = prim.place(rk, tf.mul(tf.T(u_first, 6.0, n["w_rp"]), tf.Rx(90)))       # 2D x -> u, 2D y -> w, z -> -y
        s.append(rk)
    else:
        s.append(B(u_first - 4, u_first + rack_n * math.pi * m, -6, 6, n["w_rtip"], n["w_deck1"]))
    s.append(B(0, u_first, -6, 6, n["w_deck0"], n["w_rroot"] + RACK_BACK))           # spine behind the rack teeth
    # shoes (full length: the carriage is guided over its whole length) and a light deck: a 3 mm floor under the
    # cradle, cross ribs every 30 mm in front of it
    s.append(Bs(0, Lc, n["y_lip0"] - 0.5, n["y_shoe1"], n["shoe_w0"], n["shoe_w1"]))
    for uc in np.arange(160.0, Lc - 3.0, 30.0):
        s.append(B(uc, uc + 3.0, -n["y_lip0"] + 0.5, n["y_lip0"] - 0.5, n["w_deck0"], n["w_rroot"] + RACK_BACK))
    s.append(B(Lc - 3.0, Lc, -n["y_lip0"] + 0.5, n["y_lip0"] - 0.5, n["w_deck0"], n["w_rroot"] + RACK_BACK))
    # cradle: five rings (full half pipe up to 5 below the dart axis) and two longitudinal strips 20..50 deg either side
    # of the bottom, so the belly sits in a V of two lines (light, self-centring). Ribs from each ring to the shoes.
    rc_in, rc_out = n["r_hull"] + 0.4, n["r_hull"] + 2.4
    pipe = (cyl_u(rc_out, 0, 152, 0, n["w_dart"]) - cyl_u(rc_in, -1, 153, 0, n["w_dart"])) ^ \
        B(-1, 153, -40, 40, n["w_deck0"], n["w_dart"] - 5)
    rings = prim.union([B(uc, uc + 6.0, -40, 40, 0, 200) for uc in RING_U])
    def wedge(a0, a1):
        pts = [(0.0, 0.0)] + [(40 * math.sin(math.radians(a)), -40 * math.cos(math.radians(a))) for a in np.linspace(a0, a1, 8)]
        cs = prim.poly_cs(pts)
        return prim.place(prim.extrude(cs, 145.0, z=n["pusher_t"] + 1.0), tf.mul(tf.T(0, 0, n["w_dart"]), tf.Ry(90)))
    strips = wedge(20.0, 50.0) + wedge(-50.0, -20.0)
    s.append(pipe ^ (rings + strips))
    for uc in RING_U:
        rib = B(uc, uc + 6.0, -n["y_lip0"] + 0.5, n["y_lip0"] - 0.5, n["w_deck0"], n["w_dart"] - 12.0)
        s.append(rib - cyl_u(rc_in, uc - 1, uc + 7, 0, n["w_dart"]))
    for sg in (1, -1):                                   # strip-to-shoe webs between the rings
        for uc in RING_U[:-1]:
            s.append(B(uc + 6.0, uc + 36.0, *sorted((sg * 13.0, sg * 15.0)), n["w_deck0"], n["w_dart"] - 17.5)
                     - cyl_u(rc_in, uc, uc + 40, 0, n["w_dart"]))
    # cradle lead-in at the front (dart reeled in tail first): 2 mm chamfer
    s.append((cyl_u(rc_out, 148.0, 152.0, 0, n["w_dart"]) - prim.place(prim.cyl(rc_in, 4.0, r2=rc_in + 2.0),
              tf.mul(tf.T(148.0, 0, n["w_dart"]), tf.Ry(90)))) ^ B(147, 153, -40, 40, n["w_deck0"], n["w_dart"] - 5))
    # pusher at the rear (dart tail bears on its front face), tether hole d14
    pu = cyl_u(n["r_hull"] + 1.0, -n["pusher_t"], 0, 0, n["w_dart"]) ^ B(-20, 1, -11, 11, n["w_deck0"], 200)
    pu = pu + B(-n["pusher_t"], 0, -6, 6, n["w_deck0"], n["w_dart"] - 15.0)
    pu = pu - cyl_u(7.0, -20, 5, 0, n["w_dart"])
    s.append(pu.translate((n["pusher_t"], 0, 0)))
    # arm stubs (lower half of a half-lap joint with the bolted SPRING ARMS, see carriage_arm_solid) and webs
    for sg in (1, -1):
        s.append(B(0, n["pusher_t"], *sorted((sg * 12.0, sg * Y_BODY)), n["arm_w0"], ARM_LAP_W))
        yc, yd = sorted((sg * 12.0, sg * (n["y_lip0"] - 0.5)))
        s.append(B(0, n["pusher_t"], yc, yd, n["w_deck0"], ARM_LAP_W))                 # web deck -> stub (inside the lips)
    # SW_SEATED block behind the left arm: pushrod guide and plunger guide
    st = seat_geom(P)
    s.append(B(-16.0, 0.0, 10.0, 20.0, n["arm_w0"], st["w_push"] + 4.0))
    c = prim.union(s)
    c = c ^ B(-100, 400, -Y_BODY, Y_BODY, -100, 300)        # body inside the shoe faces: prints on its side, flat
    c = c - _carriage_cuts(P)
    for sg in (1, -1):                                       # arm bolt pilots (M3 self tap, from above)
        c = c - cyl_w(1.3, n["arm_w0"] - 6.0, ARM_LAP_W + 0.5, ARM_BOLT_U, sg * ARM_BOLT_Y)
    return c


def _carriage_cuts(P):
    n = L(P)
    st = seat_geom(P)
    c = cyl_u(1.85, st["u_plunger_local"] - 6.0, n["pusher_t"] + 1.0, st["y_push"], st["w_push"])
    c = c + cyl_w(2.25, n["arm_w0"] - 2.0, st["w_push"], st["u_plunger_local"], st["y_push"])
    c = c + B(st["u_plunger_local"] - 10.5, st["u_plunger_local"] + 10.5, 12.5, 28.5, n["arm_w0"] - 2.0, n["arm_w0"] + 1.4)
    return c


def carriage_arm_solid(P, sg):
    """Bolted SPRING ARM (one per side, carriage local frame): outer arm y 21.5..61 (hits the stop, carries the spring
    pegs at y 38 and the D5 provision peg at 56), upper half-lap tongue over the body stub (2 x M3 down into it).
    Right arm also carries the rigid SW_HOME foot; left arm guides the SW_SEATED pushrod."""
    n = L(P)
    up = n["u_peg_c"]
    s = [B(0, n["pusher_t"], *sorted((sg * Y_BODY, sg * (Y_SPRING2 + 5))), n["arm_w0"], n["arm_w1"]),
         B(0, n["pusher_t"], *sorted((sg * 13.0, sg * Y_BODY)), ARM_LAP_W, n["arm_w1"])]
    for yy in (n["y_spring"], Y_SPRING2):
        s.append(cyl_w(3.0, n["w_spring"] - 9, n["arm_w0"] + 1, up, sg * yy))
        s.append(cyl_w(4.5, n["w_spring"] - 11, n["w_spring"] - 9, up, sg * yy))         # keeper flange
    st = seat_geom(P)
    if sg > 0:
        s.append(B(0, n["pusher_t"], 11.2, 20.0, n["arm_w1"] - 0.01, st["w_push"] + 4.0))   # pushrod guide lug
    else:
        s.append(B(-16.0, 0.01, -28.0, -Y_BODY, n["arm_w0"], ARM_LAP_W))                  # foot carrier
        s.append(home_foot_solid(P) ^ B(-50, 50, -40, -Y_BODY, -100, 300))
    a = prim.union(s) - _carriage_cuts(P)
    a = a - cyl_w(1.7, ARM_LAP_W - 1.0, n["arm_w1"] + 1.0, ARM_BOLT_U, sg * ARM_BOLT_Y)
    a = a - cyl_w(2.9, n["arm_w1"] - 2.5, n["arm_w1"] + 12.0, ARM_BOLT_U, sg * ARM_BOLT_Y)   # head counterbore
    return a


def big_gear_solid(P):
    """Two layers on one hub: the SECTOR layer (rack, y -6..6, teeth present only over n_present, v10 release teeth) and
    a FULL 40T layer (y 6..14) that the drive gear meshes all the way round (review 1: with one sector layer the drive
    would sit in the toothless gap for part of every turn). SW_HOME cam bump on the sector web's right face.
    Local frame: axis +z, z = 0 is the rail plane y = -6 (sector right face), so local z = y + 6."""
    n = L(P)
    m = n["m"]
    ys0, ys1 = n["y_sec"]
    yf0, yf1 = n["y_full"]
    if G is not None:
        cs = G.sector_gear_2d(m, n["z_big"], n["n_present"], first_tooth_angle=n["fta"])
        g = prim.extrude(cs, ys1 - ys0)
        g = g + G.spur(m, n["z_big"], yf1 - yf0).translate((0, 0, yf0 - ys0))
        # root disk bridging the 0.3 mm gap between the layers (no coincident tooth faces: watertight STL)
        g = g + prim.cyl(G.root_r(m, n["z_big"]) - 0.2, (yf0 - ys1) + 0.2, z=(ys1 - ys0) - 0.1)
    else:
        g = prim.cyl(n["rb"] + m, yf1 - ys0)
    hub = prim.cyl(10.0, n["hub_y"][1] - n["hub_y"][0], z=n["hub_y"][0] - ys0)
    g = g + hub
    # SW_HOME bump (rail angle 225 deg at rest: gear local x = u, y = -w)
    a0 = math.degrees(math.atan2(-math.sin(math.radians(225.0)), math.cos(math.radians(225.0))))
    arc = prim.place(prim.revolve_pts([(BUMP_R - 3.0, -BUMP_H), (BUMP_R + 3.0, -BUMP_H), (BUMP_R + 3.0, 0.01),
                                       (BUMP_R - 3.0, 0.01)], deg=2 * BUMP_DEG), tf.Rz(a0 - BUMP_DEG))
    g = g + arc                                                           # on the z = 0 face, standing out to -z
    return g - prim.cyl(5.0 + 0.175, 80, z=-40)


def drive_gear_solid(P):
    """Drive gear 26T in the FULL-layer plane (face = full layer width), hub holding sprag #1 with an integral outer
    key rib (no loose key). Local z = 0 at the face centre plane."""
    n = L(P)
    sp = dict(od=47.0, w=14.0, key=6.0)
    sp.update(getattr(P, "SPRAG", None) or {})
    fw = n["y_full"][1] - n["y_full"][0]
    g = _spur(n["m"], n["z_drive"], fw).translate((0, 0, -fw / 2))
    hub = prim.cyl(sp["od"] / 2 + 4.0, sp["w"] + 2.0, z=-(sp["w"] + 2.0) / 2)
    g = g + hub
    g = g - prim.cyl(sp["od"] / 2 + 0.05, 40, z=-20)
    g = g + prim.box(sp["od"] / 2 - 2.8, sp["od"] / 2 + 0.1, -sp["key"] / 2 + 0.1, sp["key"] / 2 - 0.1, -sp["w"] / 2, sp["w"] / 2)
    return g


def sprag_solid(P):
    sp = getattr(P, "SPRAG", None) or dict(od=47.0, bore=20.0, w=14.0)
    return prim.tube(sp["od"] / 2, sp["bore"] / 2, sp["w"], z=-sp["w"] / 2)


def gear_pin_solid(P):
    """Snap pin d10 for the big gear (axis +z = rail +y): head at the right cheek outer face, C clip groove beyond the
    left cheek with a 3 mm shoulder after the groove (review 1)."""
    n = L(P)
    y0 = n["y_chA"][0]
    y1 = n["y_chB"][1]
    pin = prim.cyl(5.0, (y1 + 5.0) - y0, z=y0) + prim.cyl(8.0, 3.0, z=y0 - 3.0)
    return pin - prim.tube(6.0, 4.0, 1.6, z=y1 + 0.4)


RIB_W, RIB_R = 6.0, 12.8      # integral key rib on the sleeve (D13): 6 wide, top at r 12.8 (sprag keyway depth 2.8)
SPIGOT_BORE = 12.35           # cad-drum d12 spigot runs in this bore the whole sleeve length


def sleeve_solid(P):
    """Launch sleeve along +z from z = 0 (flange face towards the motor, world y_flange) to the outboard end.
    d20 journals, integral key rib (no keyway cut: D13) for sprag #1 and the splitter sun, flange for the drum's
    LAUNCH dog ring (drum_NOTES 3b: OD 64 x 8, 3 x M3 at r 28 with nut traps outboard, 3 lug slots 6.4 x 3.2)."""
    n = L(P)
    z_end = n["Y0"] + 53.0 - n["y_flange"]
    s = prim.cyl(10.0, z_end) + prim.cyl(32.0, 8.0)
    zg = n["Y0"] + 0.5 * (n["y_full"][0] + n["y_full"][1]) - n["y_flange"]
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
    """6 mm steel pin (vertical) through post, hook and strap (double shear), head on top."""
    n = L(P)
    h = n["w_spring"] + 12.0 - (n["w_floor0"] - 1.0)
    return prim.cyl(3.0, h) + prim.cyl(5.0, 2.5, z=h)


N_SEG = 5
Y_BODY = 21.5         # carriage body half width (shoe faces): body prints on its side on them
ARM_LAP_W = 76.0      # half-lap joint plane between the body stub and the bolted spring arm
ARM_BOLT_U, ARM_BOLT_Y = 4.0, 17.5
Y_SPRING2 = 56.0      # second spring pair provision (D5): pegs, anchor holes and clear space at y +-56, w 60
LUG_BOLTS_U = (107.0, 123.0)   # launch_aim lug_solid bolt positions (w 41)
RACK_BACK = 4.0       # rack body above the tooth roots (carriage mass, review 1)
RING_U = (9.0, 42.0, 76.0, 110.0, 146.0)   # cradle ring positions (carriage local u)
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
    U = U.simplify(0.2)                    # 113k -> about 20k triangles (audit speed), shape within 0.2 mm
    hull = prim.cylX(21.0, 20.0, 151.97)
    nose = prim.place(prim.cyl(11.0, 20.0, r2=21.0), tf.Ry(90))
    _DART["s"] = U + hull + nose
    return _DART["s"]


def seat_geom(P, side="seated"):
    """SW_SEATED (dart seated) geometry. No wires on the carriage: the dart tail pushes a PUSHROD (through the pusher at
    y 15, w 86) back 1.5 mm; its 34 deg wedge end pushes a PLUNGER down 1.0 mm; the plunger's FOOT (under the carriage
    left arm, 20 mm long so the stop overshoot just slides) presses the screw of a printed flex lever on the left rail
    lip at HOME. The lever's leaf spring lifts foot, plunger and pushrod back when the dart leaves. Modelled pressed.
    side "home": the same lever on the RIGHT lip, pressed by a RIGID foot under the right spring arm whenever the
    carriage is within +-10 mm of its stop (overshoot into the pads just slides the flat foot over the screw)."""
    n = L(P)
    sg = 1.0 if side == "seated" else -1.0
    g = dict(sg=sg, y=sg * 25.0, u_plunger_local=-6.0, y_push=15.0, w_push=86.0)
    g["u_tip"] = n["u_c_home"] + g["u_plunger_local"]
    g["w_tip"] = n["arm_w0"] - 1.6                              # pressed foot bottom = screw tip
    g["w_head1"] = g["w_tip"] - 6.0                            # M3 x 6 shank down to the head
    g["w_head0"] = g["w_head1"] - SW.HEAD_H
    g["w_nut0"] = g["w_head0"] - SW.NUT_T
    g["leaf_w0"] = g["w_head1"]
    g["leaf_w1"] = g["w_head1"] + 1.2
    g["u_root"] = g["u_tip"] - 24.0
    return g


def seat_lever_solid(P, side="seated"):
    """Printed flex lever (PLA, flat): root block on the lip (one M3 self tap) and a 1.2 thick leaf to the screw."""
    n = L(P)
    st = seat_geom(P, side)
    root = B(st["u_root"], st["u_root"] + 5.0, st["y"] - 3.0, st["y"] + 3.0, n["lip_w1"], st["leaf_w1"] + 2.0)
    leaf = B(st["u_root"] + 4.99, st["u_tip"] + 4.0, st["y"] - 3.0, st["y"] + 3.0, st["leaf_w0"], st["leaf_w1"])
    s = root + leaf
    s = s - cyl_w(1.3, st["leaf_w0"] - 1, st["leaf_w1"] + 1, st["u_tip"], st["y"])          # M3 tap in the leaf
    s = s - cyl_w(1.7, n["lip_w1"] - 1, st["leaf_w1"] + 3, st["u_root"] + 2.5, st["y"])      # root screw
    return s


def seat_screw_solid(P, side="seated"):
    st = seat_geom(P, side)
    return cyl_w(SW.HEAD_D / 2, st["w_head0"], st["w_head1"], st["u_tip"], st["y"]) + \
        cyl_w(1.5, st["w_head1"] - 0.01, st["w_tip"], st["u_tip"], st["y"])


def seat_nut_solid(P, side="seated"):
    st = seat_geom(P, side)
    hexr = SW.NUT_AF / 2 / 0.866
    nut = prim.cyl(hexr, SW.NUT_T, x=st["u_tip"], y=st["y"], z=st["w_nut0"], seg=6)
    return nut - cyl_w(1.5, st["w_nut0"] - 1, st["w_nut0"] + SW.NUT_T + 1, st["u_tip"], st["y"])


def seat_plunger_solid(P):
    """Plunger + foot (carriage local frame, pressed position)."""
    n = L(P)
    st = seat_geom(P)
    up = st["u_plunger_local"]
    foot = B(up - 10.0, up + 10.0, 13.0, 28.0, st["w_tip"], st["w_tip"] + 1.6)
    foot = foot - B(up + 8.0, up + 10.1, 12.9, 28.1, st["w_tip"] - 0.1, st["w_tip"] + 0.8)       # front lead-in step
    foot = foot - B(up - 10.1, up - 8.0, 12.9, 28.1, st["w_tip"] - 0.1, st["w_tip"] + 0.8)       # rear lead-in step
    stem = cyl_w(2.0, st["w_tip"] + 1.59, st["w_push"] - 1.6, up, st["y_push"])
    return foot + stem


def seat_pushrod_solid(P):
    """Pushrod d3.2 along u (carriage local), front flush with the pusher face (pressed), rear end over the plunger."""
    n = L(P)
    st = seat_geom(P)
    r = cyl_u(1.6, st["u_plunger_local"] - 4.5, n["pusher_t"], st["y_push"], st["w_push"])
    return r - B(-50, 50, -100, 100, -100, st["w_push"] - 1.2)            # flat underside: prints lying on it


def home_foot_solid(P):
    """Rigid foot under the right spring arm (carriage local), with lead-in steps; part of the carriage."""
    st = seat_geom(P, "home")
    up = st["u_plunger_local"]
    foot = B(up - 10.0, up + 10.0, -28.0, -13.0, st["w_tip"], st["w_tip"] + 1.61)
    foot = foot - B(up + 8.0, up + 10.1, -28.1, -12.9, st["w_tip"] - 0.1, st["w_tip"] + 0.8)
    foot = foot - B(up - 10.1, up - 8.0, -28.1, -12.9, st["w_tip"] - 0.1, st["w_tip"] + 0.8)
    return foot


def parts(P):
    n = L(P)
    out = []
    pcs = rail_pieces(P)
    cols = dict(rear="#9aa7b4", mid="#8796a5", front="#9aa7b4")
    notes = dict(rear="shoe track, walls, tether fork and two pulleys behind the rear end",
                 mid="shoe track, cheeks on the sleeve, big gear pin bosses, SW_GAP ledge, lever seats, pad brackets",
                 front="shoe track end, spring anchor posts with straps, muzzle tie")
    for k, sol in pcs.items():
        out.append(Part(f"launch_rail_{k}", sol, "PLA", joint="launch_rail", place=tf.I4,
                        print_tf=best_bed(sol), color=cols[k], notes=f"Rail {k} piece: {notes[k]}"))
    cs = carriage_solid(P)
    out.append(Part("launch_carriage", cs, "PLA", joint="launch_carriage", place=tf.T(n["u_c_home"], 0, 0),
                    print_tf=bed(cs, tf.Rx(90), rz=45.0), color="#d97a00",
                    notes="Carriage: rack, shoes, dart cradle, pusher with tether hole, spring arms and pegs, SW feet"))
    for sg, side in ((1, "l"), (-1, "r")):
        arm = carriage_arm_solid(P, sg)
        out.append(Part(f"launch_carriage_arm_{side}", arm, "PLA", joint="launch_carriage",
                        place=tf.T(n["u_c_home"], 0, 0), print_tf=best_bed(arm), color="#c86f00",
                        notes="Bolted spring arm: hits the stop, carries the spring pegs" +
                              (" and the SW_SEATED pushrod guide" if sg > 0 else " and the rigid SW_HOME foot")))
    bg = big_gear_solid(P)
    out.append(Part("launch_big_gear", bg, "PPA-CF", joint="launch_big_gear",
                    place=tf.mul(tf.T(0, n["y_sec"][0], 0), tf.Rx(-90)), print_tf=bed(bg, tf.Rx(180)), color="#3d7cc9",
                    notes=f"Big gear m2.5: sector layer {n['n_present']} of 40 teeth (rack, v10 release teeth) + full 40T "
                          f"layer (drive), SW_GAP cam bump, on the rail pin"))
    gp = gear_pin_solid(P)
    out.append(Part("launch_gear_pin", gp, "PPA-CF", joint="launch_rail", place=tf.mul(tf.T(n["cd"], 0, 0), tf.Rx(-90)),
                    print_tf=bed(gp), color="#3a3f45", notes="Snap pin d10 for the big gear, C clip"))
    out.append(Part("launch_gear_clip", clip_solid(P), "PLA", joint="launch_rail",
                    place=tf.mul(tf.T(n["cd"], 0, 0), tf.Rx(-90)), print_tf=bed(clip_solid(P)), color="#5a5f65",
                    notes="C clip on the big gear pin groove"))
    ym = 0.5 * (n["y_full"][0] + n["y_full"][1])
    dg = drive_gear_solid(P)
    out.append(Part("launch_drive_gear", dg, "PPA-CF", joint="launch_drive", place=tf.mul(tf.T(0, ym, 0), tf.Rx(-90)),
                    print_tf=bed(dg), color="#2f6db5", notes="Drive gear m2.5 26T, sprag #1 inside on an integral key rib"))
    out.append(Part("launch_sprag", sprag_solid(P), "bought", joint="launch_drive", place=tf.mul(tf.T(0, ym, 0), tf.Rx(-90)),
                    color="#777c84", bought_ref="CSK20PP 20x47x14 sprag #1", notes="Sprag #1: forward wiper cocks"))
    sl = sleeve_solid(P)
    out.append(Part("launch_sleeve", sl, "PPA-CF", joint="launch_sleeve",
                    place=tf.mul(tf.T(0, n["y_flange"] - n["Y0"], 0), tf.Rx(-90)), print_tf=bed(sl), color="#3a3f45",
                    notes="Launch sleeve d20: dog ring flange, integral key ribs (sprag #1, splitter sun), journals"))
    # springs: hooks + coil segments that spread as the carriage moves
    seg_len = (n["L_home"] - 2 * HOOK_L) / N_SEG
    spref = (getattr(P, "SPRING", None) or {}).get("name", "extension spring") + " extension spring"
    for sg, side in ((1, "l"), (-1, "r")):
        y = sg * n["y_spring"]
        ap = anchor_pin_solid(P)
        out.append(Part(f"launch_anchor_pin_{side}", ap, "steel", joint="launch_rail",
                        place=tf.T(n["u_anchor"], y, n["w_floor0"] - 1.0), color="#3a3f45",
                        bought_ref="6 mm steel dowel or M6 bolt shank, 45 long (printed PPA-CF only as a stopgap)",
                        notes="Spring anchor pin through post and strap (double shear)"))
        out.append(Part(f"launch_spring_{side}_hook_a", hook_solid(P, -1), "bought", joint="launch_rail",
                        place=tf.T(n["u_anchor"], y, n["w_spring"]), color="#b0b5bb", bought_ref=spref,
                        notes="Spring hook on the anchor pin"))
        out.append(Part(f"launch_spring_{side}_hook_c", hook_solid(P, 1), "bought", joint="launch_carriage",
                        place=tf.T(n["u_c_home"] + n["u_peg_c"], y, n["w_spring"]), color="#b0b5bb",
                        notes="Spring hook on the carriage peg"))
        for i in range(N_SEG):
            u_seg = n["u_c_home"] + n["u_peg_c"] + HOOK_L + i * seg_len
            out.append(Part(f"launch_spring_{side}_{i}", spring_segment_solid(P, seg_len), "bought",
                            joint=f"launch_spring_seg_{i}", place=tf.T(u_seg, y, n["w_spring"]), color="#c2c7cc",
                            notes="Spring coils (shown in segments that spread when stretched)"))
        y0, y1 = sorted((sg * n["pad_y"][0], sg * n["pad_y"][1]))
        pad = B(0, PAD_L + 4.0, y0, y1, n["pad_w"][0], n["pad_w"][1])
        out.append(Part(f"launch_stop_pad_{side}", pad, "TPU", joint="launch_rail", place=tf.T(n["u_stop"], 0, 0),
                        print_tf=bed(pad), color="#222222",
                        notes="Carriage stop pad TPU 95A, 18 free + 4 in the pocket (or EVA foam block)"))
    # generic contact switch: SW_GAP
    for nm, f in switch_frames(P).items():
        bod = SW.body(f["z_s"], f["screw"], f["x0"], width=f["width"], top=f["top"])
        out.append(Part(f"launch_sw_{nm}", bod, "PLA", joint=f["joint"], place=f["M"], print_tf=bed(bod),
                        color="#5fae5f", notes=f"SW_{nm.upper()} printed contact: {f['where']}"))
        out.append(Part(f"launch_sw_{nm}_screw", SW.screw(f["screw"], f["z_s"]), "bought", joint=f["joint"], place=f["M"],
                        color="#444444", bought_ref=f"M3x{f['screw']:g} SHCS", notes="Contact screw, threaded through the leaf"))
        out.append(Part(f"launch_sw_{nm}_nut", SW.nut(f["z_s"]), "bought", joint=f["joint"], place=f["M"],
                        color="#444444", bought_ref="M3 nut", notes="Contact nut in its trap, wire under it"))
    # lever contacts on the lips: SW_SEATED (left) and SW_HOME (right)
    for side in ("seated", "home"):
        lv = seat_lever_solid(P, side)
        out.append(Part(f"launch_sw_{side}", lv, "PLA", joint="launch_rail", place=tf.I4, print_tf=bed(lv),
                        color="#5fae5f", notes=f"SW_{side.upper()} printed flex lever on the {'left' if side == 'seated' else 'right'} rail lip"))
        out.append(Part(f"launch_sw_{side}_screw", seat_screw_solid(P, side), "bought", joint="launch_rail",
                        color="#444444", bought_ref="M3x6 SHCS", notes="Contact screw, head down onto the nut"))
        out.append(Part(f"launch_sw_{side}_nut", seat_nut_solid(P, side), "bought", joint="launch_rail",
                        color="#444444", bought_ref="M3 nut", notes="Contact nut in the wall, wire under it"))
    pl = seat_plunger_solid(P)
    out.append(Part("launch_seat_plunger", pl, "PPA-CF", joint="launch_carriage", place=tf.T(n["u_c_home"], 0, 0),
                    print_tf=bed(pl), color="#3a3f45", notes="SW_SEATED plunger with foot (wedge top, pressed by the pushrod)"))
    pr = seat_pushrod_solid(P)
    out.append(Part("launch_seat_pushrod", pr, "PPA-CF", joint="launch_carriage", place=tf.T(n["u_c_home"], 0, 0),
                    print_tf=bed(pr), color="#3a3f45", notes="SW_SEATED pushrod d3.2 with a flat, pressed by the dart tail"))
    out += LN.parts_line(n, bed)
    out.append(Part("launch_dart", dart_solid(P), "bought", joint="launch_dart_rot",
                    place=tf.mul(tf.T(-(n["u_tail_home"] + 76.0), 0, -n["w_dart"]),
                                 tf.T(n["u_tail_home"] + 152.0, 0, n["w_dart"] - 0.4), tf.Rz(180)),
                    color="#e8e8e8", bought_ref="KEDGE v30 dart (reference, built separately)",
                    notes="The dart, folded, belly on the cradle, tail on the pusher"))
    out += _aim("parts_aim", P)
    return out


def clip_solid(P):
    """Printed C clip for the d10 pin groove (core d8): opening 0.84 of the core (v10 snap fit)."""
    c = prim.tube(9.0, 4.0, 1.4, z=0.0)
    c = c - prim.box(0.0, 20.0, -0.84 * 4.0, 0.84 * 4.0, -1, 3)
    return c.translate((0, 0, n_clip_z(P)))


def n_clip_z(P):
    n = L(P)
    return n["y_chB"][1] + 0.5


def joints(P):
    n = L(P)
    out = [
        Joint("launch_rail", None, "rev", (0.0, n["Y0"], n["axis_z"]), (0, -1, 0), limits=(5.5, 47.5)),
        Joint("launch_sleeve", None, "rev", (0.0, n["Y0"], n["axis_z"]), (0, 1, 0)),
        Joint("launch_drive", None, "rev", (0.0, n["Y0"], n["axis_z"]), (0, 1, 0)),
        Joint("launch_carriage", "launch_rail", "slide", (0, 0, 0), (-1, 0, 0), limits=(0.0, n["stroke"] + 0.5)),
        Joint("launch_big_gear", "launch_rail", "rev", (n["cd"], 0, 0), (0, 1, 0)),
        Joint("launch_dart", "launch_carriage", "slide", (0, 0, 0), (1, 0, 0)),
        Joint("launch_dart_w", "launch_dart", "slide", (0, 0, 0), (0, 0, 1)),
        Joint("launch_dart_rot", "launch_dart_w", "rev", (n["u_tail_home"] + 76.0, 0, n["w_dart"]), (0, -1, 0)),
    ]
    for i in range(N_SEG):
        out.append(Joint(f"launch_spring_seg_{i}", "launch_rail", "slide", (0, 0, 0), (-1, 0, 0),
                         gear=("launch_carriage", (N_SEG - i - 0.5) / N_SEG, 0.0)))
    out += LN.joints_line(n)
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


def _f(S, key, default):
    v = S.get(key, default)
    try:
        return float(default if v is None else v)
    except (TypeError, ValueError):
        return float(default)


POST_FIRE = ("flight", "neutral", "tidy", "wrap", "reel_engage", "tow", "release", "reel_in", "unload", "unwrap",
             "approach", "to_launch", "lower")


def _lower_progress(S):
    """'lower' phase: the timeline eases aim_deg down; the crank really goes on over the top and down the falling
    stroke. Returns (progress 0..1, start pitch) or None."""
    if S.get("phase") != "lower":
        return None
    a0 = 35.4
    a = _f(S, "aim_deg", a0)
    return (min(1.0, max(0.0, (a0 - a) / max(1e-6, a0 - 6.0))), a0)


def kin(P, S):
    n = L(P)
    g = K.gears(P)
    t = K.pitch_table(P)
    pmin, pmax = t["pitch_min"] + 0.01, t["pitch_max"] - 0.01
    if S.get("aim_psi_from_low") is not None and S.get("aim_deg") is None:
        pitch = float(K.pitch_of_psi(P, (t["psi_low"] - _f(S, "aim_psi_from_low", 0.0)) % 360.0))
    else:
        pitch = float(np.clip(_f(S, "aim_deg", pmin), pmin, pmax))
    lp = _lower_progress(S)
    S_aim = dict(S)
    sleeve_cmd = S.get("sleeve_deg")
    if sleeve_cmd is not None:
        lp = None
    if lp is not None:                                   # over the top and down the falling stroke
        u, a0 = lp
        psi_up = AIM.psi_of_pitch(P, a0) if AIM is not None and hasattr(AIM, "psi_of_pitch") else None
        if psi_up is not None:
            d_start = float(psi_up[1])
            d = d_start + u * (360.0 - d_start)
            S_aim = {k: v for k, v in S.items() if k != "aim_deg"}
            S_aim["aim_psi_from_low"] = d % 360.0
            pitch = float(np.clip(K.pitch_of_psi(P, (t["psi_low"] - d) % 360.0), pmin, pmax))
    cock = float(np.clip(_f(S, "cock", 0.0), 0.0, 1.0))
    fired = bool(_f(S, "fired", 0.0))
    phase = S.get("phase", "")
    travel = cock * n["stroke"]
    if S.get("launch_travel") is not None:
        travel = _f(S, "launch_travel", travel)
    if S.get("launch_gear_deg") is not None:
        gdeg = _f(S, "launch_gear_deg", 0.0)
    elif fired and travel <= 0.0:
        gdeg = -360.0                                     # the forward run carried the gear through its gap to rest
    elif fired:
        gdeg = gear_release_angle(P)                      # shot in progress: gear just past the release
    else:
        gdeg = gear_angle_for_travel(P, travel)
    drive_rel = -gdeg * g["ratio"]
    if sleeve_cmd is not None and AIM is not None and hasattr(AIM, "kin_aim"):
        # spec v1.2: the sleeve angle is given. Backward sleeve travel from the LOW dead point (sleeve 0 at LOW) =
        # -sleeve + the forward travel spent cocking; the crank turns 1/10 of it. Lock the crank to the sun teeth
        # (12 deg pitch) so the splitter mesh stays phased, then the pitch follows from the crank.
        sl = float(sleeve_cmd)
        wb = max(0.0, -sl + drive_rel)
        d = wb / K.wiper_per_crank(P)
        def s_eng(dd):
            return AIM.kin_aim(P, {"aim_psi_from_low": dd % 360.0}, float(K.pitch_of_psi(P, (t["psi_low"] - dd) % 360.0)))["launch_sleeve"]
        K0 = s_eng(0.0)
        kk = round((K0 - sl - 10.0 * d) / 12.0)
        d = max(0.0, (K0 - sl - 12.0 * kk) / 10.0)
        pitch = float(np.clip(K.pitch_of_psi(P, (t["psi_low"] - d) % 360.0), pmin, pmax))
        S_aim = {k: v for k, v in S.items() if k != "aim_deg"}
        S_aim["aim_psi_from_low"] = d % 360.0
    out = {"launch_rail": pitch, "launch_carriage": travel, "launch_big_gear": gdeg,
           "launch_drive": drive_rel - pitch + _drive_phase(P)}
    # dart: on the sled, in flight (dart_flight 0..1 along the LAUNCH.flight arc), coming home (dart_return 0..1)
    du, dw, dr = dart_offsets(P, S, pitch, travel)
    out["launch_dart"] = du
    out["launch_dart_w"] = dw
    out["launch_dart_rot"] = dr
    # aim train and sleeve: engaged (sleeve from the crank) except while cocking, when sprag #1 locks the sleeve to the
    # drive gear and the splitter arm swings out onto its stop
    if AIM is not None and hasattr(AIM, "kin_aim"):
        eng = AIM.kin_aim(P, S_aim, pitch)
        out.update(eng)
        s_eng = eng.get("launch_sleeve", 0.0)
        cocking = cock > 0.0 or (fired and phase in ("fire",))
        if sleeve_cmd is not None:
            if cocking:
                out.update(AIM.kin_aim(P, dict(S_aim, aim_split="out", wiper_deg=float(sleeve_cmd)), pitch))
            out["launch_sleeve"] = float(sleeve_cmd) if cocking else s_eng
        elif cocking:
            S_out = dict(S_aim, aim_split="out", wiper_deg=s_eng + drive_rel)
            out.update(AIM.kin_aim(P, S_out, pitch))
            out["launch_sleeve"] = s_eng + drive_rel
        elif fired:
            out["launch_sleeve"] = s_eng + 12.0 * round(drive_rel / 12.0)      # keeps the sun phased with the idler
    else:
        out["launch_sleeve"] = _f(S, "wiper_deg", 0.0)
    return out


def dart_offsets(P, S, pitch, travel):
    """(u, w, rot) of the dart relative to its seat on the carriage, from launch_dart_fly (audit), dart_flight and
    dart_return (timeline)."""
    if S.get("launch_dart_fly") is not None:
        return _f(S, "launch_dart_fly", 0.0), 0.0, 0.0
    fl = _f(S, "dart_flight", 0.0)
    rt = _f(S, "dart_return", 0.0)
    if fl <= 0.0 or travel > 0.0:
        return 0.0, 0.0, 0.0
    lf = dict(range_m=4.5, apex_m=1.0, t_flight_s=1.0)
    lf.update((getattr(P, "LAUNCH", None) or {}).get("flight", {}) or {})
    R, H = lf["range_m"] * 1000.0, lf["apex_m"] * 1000.0
    if rt > 0.0:
        s = 1.0 - rt                                         # straight back along the line from the landing point
        X, Z, ang = R * s, -165.0 * s, 0.0
    else:
        X = R * fl
        Z = 4.0 * H * fl * (1.0 - fl) - 165.0 * fl * fl       # parabola to the ground (deck level) at the landing
        ang = math.degrees(math.atan2(4.0 * H * (1 - 2 * fl) - 330.0 * fl, R)) - pitch
    c, s_ = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    return X * c + Z * s_, -X * s_ + Z * c, ang


def allowed(P):
    return [
        Allow("launch_sprag", "launch_drive_gear", "sprag outer race keyed in the drive gear (press)"),
        Allow("launch_sprag", "launch_sleeve", "sprag inner race on the keyed sleeve"),
        Allow("launch_spring_*_hook_a", "launch_anchor_pin_*", "hook loop round the anchor pin"),
        Allow("launch_anchor_pin_*", "launch_rail_front", "pin in its hole"),
        Allow("launch_spring_*_hook_c", "launch_carriage_arm_*", "hook loop round the arm peg"),
        Allow("launch_stop_pad_*", "launch_rail_mid", "pad in its pocket"),
        Allow("launch_sw_gap_screw", "launch_sw_gap", "screw threaded into the leaf"),
        Allow("launch_sw_home_screw", "launch_sw_home", "screw threaded into the lever"),
        Allow("launch_sw_seated_screw", "launch_sw_seated", "screw threaded into the lever"),
        Allow("launch_sw_rail_low_screw", "launch_sw_rail_low", "screw threaded into the leaf"),
    ] + LN.allowed_line() + _aim("allowed_aim", P)


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


def _rail_to_world(P, pitch):
    n = L(P)
    return tf.mul(tf.T(0.0, n["Y0"], n["axis_z"]), tf.R((0, -1, 0), pitch))


def line_entry(P, S=None):
    """World point where the tether arrives at the rail: the far side of the pivot pulley (cad-reel's line ends here).
    The pulley is 24 mm below the main axis, so this point moves only a little with pitch."""
    S = S or {}
    n = L(P)
    pitch = kin(P, S)["launch_rail"]
    g = LN.geom(n)
    pv = g["piv"]
    p = (pv[0] - LN.R_PATH, pv[1], pv[2])                      # rear side of the pivot pulley, rail frame
    return tf.apply(_rail_to_world(P, pitch), np.array([p]))[0]


def line_path(P, S=None):
    """Tether centre polyline on the launcher, WORLD frame, as [(points N x 3, allowed part names)] spans:
    pivot pulley -> under the rail -> bottom and top pulleys behind the rail -> forward along the axis to the pusher
    -> dart tail (on the sled, in flight or coming home). Continues cad-reel's path from line_entry(P, S)."""
    S = S or {}
    n = L(P)
    v = kin(P, S)
    pitch, travel = v["launch_rail"], v["launch_carriage"]
    g = LN.geom(n)
    W = _rail_to_world(P, pitch)
    pv = g["piv"]
    arc = []
    for a in np.linspace(180.0, 90.0, 7):                         # wrap over the pivot pulley (rear to top)
        t = math.radians(a)
        arc.append((pv[0] + LN.R_PATH * math.cos(t), pv[1], pv[2] + LN.R_PATH * math.sin(t)))
    bt, tp = g["bot"], g["top"]
    bot = [(bt[0] + LN.R_PATH * math.cos(math.radians(a)), 0.0, bt[2] + LN.R_PATH * math.sin(math.radians(a)))
           for a in np.linspace(270.0, 180.0, 5)]
    top = [(tp[0] + LN.R_PATH * math.cos(math.radians(a)), 0.0, tp[2] + LN.R_PATH * math.sin(math.radians(a)))
           for a in np.linspace(180.0, 90.0, 5)]
    pusher = (n["u_c_home"] - travel, 0.0, g["w_line"])
    tail_seat = np.array((n["u_tail_home"] - travel, 0.0, g["w_line"]))
    du, dw = v.get("launch_dart", 0.0), v.get("launch_dart_w", 0.0)
    tail = tail_seat + np.array((du, 0.0, dw))
    rail_pts = np.array(arc + [g["p_bot_low"]] + bot + top + [pusher])
    spans = [(tf.apply(W, rail_pts), {"launch_pulley_piv", "launch_pulley_bot", "launch_pulley_top", "launch_rail_rear"}),
             (tf.apply(W, np.array([pusher, tail])), {"launch_carriage", "launch_dart", "launch_line_p"})]
    return spans
