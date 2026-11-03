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
    n["y_full"] = (7.5, 14.5)                                  # 1.5 clear of the rack face, 0.5 of the shelf
    n["hub_y"] = (-8.8, 14.5)                                  # hub flush with the full layer face; washer to cheek B
    n["y_chA"] = (-17.0, -9.0)
    n["y_chB"] = (21.0, 28.0)
    # spring arms (they hit the stop pads) and pads
    n["arm_w0"], n["arm_w1"] = 73.2, 80.0             # arm bottom 73.2: the SW feet (1.6) stay above the stop ledges
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
    # SW_GAP moved to cad-sensors (sensors.py: cad.lib contact switch on its own bracket); geometry kept for reference
    _ = dict(M=M, joint="launch_rail", z_s=zs, screw=6.0, x0=-3.0, width=None, top=5.0)
    return out


BUMP_H = 2.0                  # SW_GAP cam bump height on the big gear web
BUMP_R = 29.7                 # its radius from the gear pin
BUMP_DEG = 5.0                # half width (deg) of the flat top
BUMP_RAMP_DEG = 10.0          # ramp length (deg) each side: about 22 deg slope at r 29.7


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
    # (SW_GAP ledge removed: cad-sensors' bracket screws to the cheek outer face, pilots below)
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
        # fallback hole set F, ANCHOR_F rearward (params SPRING fallback asks 15; the stop frame front wall ends at
        # u 248.15 so 8.5 is the most there is room for): the coils of a hook on the main hole pass here, so the slot
        # is the coil size (w +-8.5) and the strap sits higher
        s.append(B(248.3, ua - 7.99, yi2, yo2, n["w_floor0"] + 10.0 - 0.01, wsp - 8.5))
        s.append(B(248.3, ua - 7.99, yi2, yo2, wsp + 8.5, wsp + 11.0 + 1.5))
    s.append(B(ua - 8, ua + 8, -yout, yout, n["w_floor0"] - 6, n["w_floor0"]))
    # carriage stop: printed PLA flexure units (launch_stop.py): rail ledges and hard stops
    s.append(ST.stop_rail_add(n))
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
            for uu in (ua, ua - ANCHOR_F):
                solid = solid - cyl_w(ANCHOR_D / 2 + 0.1, n["w_floor0"] - 1, wsp + 14.0, uu, sg * yy)
        # letter F on the fallback strap top (0.6 deep)
        uf, yf, wt = ua - ANCHOR_F, sg * (ys + 9.0), wsp + 12.5
        for (du0, du1, dy0, dy1) in ((-3.0, 3.0, -2.0, -0.8), (2.2, 3.0, -2.0, 2.0), (-0.4, 0.4, -2.0, 1.2)):
            solid = solid - B(uf + du0, uf + du1, yf + dy0, yf + dy1, wt - 0.6, wt + 1)
    solid = solid - ST.stop_rail_cut(n, P)
    # clear the sleeve flange (r 32, world y 111..119) and the dog ring: nothing of the rail within r 33.5 there
    solid = solid - cyl_y(33.5, -60.0, n["y_flange"] + 8.5 - n["Y0"], 0.0, 0.0)
    # cad-sensors SW_GAP bracket: 2 x M3 self tap pilots d2.6, 7 deep, along +y into the right cheek outer face
    for (uu, ww) in ((48.0, 2.0), (64.0, 2.0)):
        solid = solid - cyl_y(1.3, n["y_chA"][0] - 1.0, n["y_chA"][0] + 7.0, uu, ww)
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
    # top splice straps across the front joint: 2 x M3 self tap pilots each side of the split, into the walls
    for sg in (1, -1):
        for du in (-8.0, -2.5, 2.5, 8.0):
            solid = solid - cyl_w(1.3, n["lip_w1"] - 12.0, n["lip_w1"] + 1.0, n["split"][1] + du, sg * SPLICE_Y)
    # aim lug: 2 x M3 self tap pilots through the left wall into the shelf (lug bolting plate at u 100..130)
    for uu in LUG_BOLTS_U:
        solid = solid - cyl_y(1.3, n["y_shoe0"] + 1.0, n["y_wall1"] + 0.5, uu, 41.0)
    solid = solid - LN.rail_cut(n)
    solid = solid - GD.guard_rail_cut(n)
    return solid


def splice_solid(P, sg):
    """Strap on the wall top across the front rail split: keeps the joint from opening at the lips (the springs pull
    at w 60, the flange bolts sit lower)."""
    n = L(P)
    u0 = n["split"][1]
    st = B(u0 - 11.0, u0 + 11.0, *sorted((sg * 23.0, sg * n["y_wall1"])), n["lip_w1"], n["lip_w1"] + 3.0)
    for du in (-8.0, -2.5, 2.5, 8.0):
        st = st - cyl_w(1.7, n["lip_w1"] - 1.0, n["lip_w1"] + 4.0, u0 + du, sg * SPLICE_Y)
    return st


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


def _truss_windows(a, b, qa, qb, z0, z1, t, plane="uw"):
    """Warren truss lightening windows (carriage / yoke): the band a..b (first plane axis) x qa..qb (second) minus
    diagonals and end posts t wide, as a prism z0..z1 along the third axis. plane "uw": windows in the (u, w) plane
    through y; "yu": in the (y, u) plane through w. Diagonals about 40 deg off the chords."""
    from manifold3d import CrossSection
    a, b = a + t, b - t
    h = qb - qa
    if b - a < 2.0 * t or h < 1.0:
        from manifold3d import Manifold
        return Manifold()
    nd = max(1, int(round((b - a) / (1.2 * h))))
    xs = np.linspace(a, b, nd + 1)
    cs = CrossSection.square((b - a, h)).translate((a, qa))
    for k in range(nd):
        p0, p1 = xs[k], xs[k + 1]
        q0, q1 = (qa, qb) if k % 2 == 0 else (qb, qa)
        hh = 0.5 * t * math.hypot(p1 - p0, h) / h                          # horizontal half width of the diagonal
        lo, hi = qa - 1.0, qb + 1.0
        pl = p0 + (lo - q0) * (p1 - p0) / (q1 - q0)
        ph = p0 + (hi - q0) * (p1 - p0) / (q1 - q0)
        cs = cs - prim.poly_cs([(pl - hh, lo), (pl + hh, lo), (ph + hh, hi), (ph - hh, hi)])
    sol = prim.extrude(cs, z1 - z0)
    if plane == "uw":
        return prim.place(sol, tf.mul(tf.T(0, z1, 0), tf.Rx(90)))          # x = u, y = w, z -> -y: spans z1 .. z0
    M = np.array([[0.0, 1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, -1.0, z1], [0.0, 0.0, 0.0, 1.0]])
    return prim.place(sol, M)                                              # x = y, y = u, z -> -w: spans z1 .. z0


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
    # Rack back stress (RACK_BACK 3.0, review: carriage mass): tooth force 80 N + 30 % shock = 104 N tangential at the
    # pitch line, 38 N radial (20 deg). Rack back 12 x 3 (teeth ignored): Z = 12 * 3^2 / 6 = 18 mm3, A = 36 mm2. Worst
    # clear span between deck ribs 29 mm, fixed-fixed: radial 38 * 29 / 8 = 138 Nmm; tangential force 5.1 mm below the
    # back centroid, couple 104 * 5.1 = 530 Nmm at mid span -> 265 Nmm; axial 104 / 36 = 2.9 MPa. Total
    # (138 + 265) / 18 + 2.9 = 25 MPa < 30 MPa (bending along u, in the layers: the body prints on its side), and the
    # teeth (6 mm deep over half the length) make the real section stiffer. Tooth root (Lewis, Y 0.3): 104 / (12 * 2.5
    # * 0.3) = 12 MPa.
    rib_top = n["w_rroot"] + DECK_T
    yo = n["y_lip0"] - 0.5
    s.append(B(0, u_first, -6, 6, n["w_deck0"], rib_top))                              # spine behind the rack teeth
    # light deck: cross ribs (under the cradle between the ring ribs, every 30 mm in front of it): rack spans <= 29 mm
    deck_u = [46.0, 103.0, 125.0] + [float(uc) for uc in np.arange(160.0, Lc - 3.0, 30.0)] + [Lc - 3.0]
    for uc in deck_u:
        s.append(B(uc, uc + 3.0, -yo, yo, n["w_deck0"], rib_top))
    # shoes (full length: the carriage is guided over its whole length) with windows through them (y, vertical as
    # printed): 1.6 mm chords bear on the shelf and under the lips, posts 2 mm, solid where a rib or web meets the shoe.
    # Chord check: radial rack force 38 N split over both shoes plus about 5 N per side from the spring / rack couple,
    # say 25 N on one 4 x 1.6 chord over one window (span 12, fixed-fixed): 25 * 12 / 8 / (4 * 1.6^2 / 6) = 22 MPa
    # (along u, in the layers). The lip / shelf contact spreads it over several windows in practice.
    keep = sorted([(0.0, n["pusher_t"])] + [(uc, uc + RIB_T) for uc in RING_U] + [(uc, uc + 3.0) for uc in deck_u])
    shoe = Bs(0, Lc, yo, n["y_shoe1"], n["shoe_w0"], n["shoe_w1"])
    for (_, a), (b, _) in zip(keep[:-1], keep[1:]):
        a, b = a + 2.0, b - 2.0
        nw = int(math.ceil((b - a + 2.0) / 14.0))
        lw = (b - a - 2.0 * (nw - 1)) / max(nw, 1)
        if nw < 1 or lw < 3.0:
            continue
        for k in range(nw):
            u0 = a + k * (lw + 2.0)
            shoe = shoe - Bs(u0, u0 + lw, yo - 1.0, n["y_shoe1"] + 1.0, n["shoe_w0"] + 1.6, n["shoe_w1"] - 1.6)
    s.append(shoe)
    # cradle: five rings (full half pipe up to 5 below the dart axis) and two longitudinal strips 20..50 deg either side
    # of the bottom, so the belly sits in a V of two lines (light, self-centring). Ribs from each ring to the shoes.
    rc_in, rc_out = n["r_hull"] + 0.4, n["r_hull"] + 2.4
    pipe = (cyl_u(rc_out, 0, 152, 0, n["w_dart"]) - cyl_u(rc_in, -1, 153, 0, n["w_dart"])) ^ \
        B(-1, 153, -40, 40, n["w_deck0"], n["w_dart"] - 5)
    rings = prim.union([B(uc, uc + RING_T, -40, 40, 0, 200) for uc in RING_U])
    def wedge(a0, a1):
        pts = [(0.0, 0.0)] + [(40 * math.sin(math.radians(a)), -40 * math.cos(math.radians(a))) for a in np.linspace(a0, a1, 8)]
        cs = prim.poly_cs(pts)
        return prim.place(prim.extrude(cs, 140.0, z=14.0), tf.mul(tf.T(0, 0, n["w_dart"]), tf.Ry(90)))
    strips = wedge(20.0, 50.0) + wedge(-50.0, -20.0)
    s.append(pipe ^ (rings + strips))
    for uc in RING_U:
        rib = B(uc, uc + RIB_T, -yo, yo, n["w_deck0"], n["w_dart"] - 12.0)
        s.append(rib - cyl_u(rc_in, uc - 1, uc + 7, 0, n["w_dart"]))
    for sg in (1, -1):                                   # strip-to-shoe webs between the rings: Warren truss windows
        for uc, uc1 in zip(RING_U[:-1], RING_U[1:]):
            web = B(uc + RIB_T - 0.01, uc1 + 0.01, *sorted((sg * 13.0, sg * 15.0)), rib_top - 0.5, n["w_dart"] - 17.5)
            web = web - _truss_windows(uc + RIB_T, uc1, rib_top + 1.1, n["w_dart"] - 19.1, -16.0, 16.0, 1.6)
            s.append(web - cyl_u(rc_in, uc, uc + 40, 0, n["w_dart"]))
    # cradle lead-in at the front (dart reeled in tail first): 2 mm chamfer
    s.append((cyl_u(rc_out, 148.0, 152.0, 0, n["w_dart"]) - prim.place(prim.cyl(rc_in, 4.0, r2=rc_in + 2.0),
              tf.mul(tf.T(148.0, 0, n["w_dart"]), tf.Ry(90)))) ^ B(147, 153, -40, 40, n["w_deck0"], n["w_dart"] - 5))
    # pusher at the rear (dart tail bears on its front face), tether hole d14
    pu = cyl_u(n["r_hull"] + 1.0, -n["pusher_t"], 0, 0, n["w_dart"]) ^ B(-20, 1, -11, 11, n["w_deck0"], 200)
    pu = pu + B(-n["pusher_t"], 0, -6, 6, n["w_deck0"], n["w_dart"] - 15.0)
    pu = pu - cyl_u(7.0, -20, 5, 0, n["w_dart"])
    # hollow: a slot through y (vertical as printed) leaves a 3 mm front plate (dart tail, 45 N) and a 2.5 mm rear
    # plate (yoke cross bar), tied by the top, the lower block and a tube round the tether hole
    pu = pu - (B(-5.5, -3.0, -12, 12, n["w_dart"] - 12.0, n["w_dart"] + 18.0) - cyl_u(9.5, -9, 1, 0, n["w_dart"]))
    s.append(pu.translate((n["pusher_t"], 0, 0)))
    # arm stubs (lower half of a half-lap joint with the bolted SPRING ARMS, see carriage_arm_solid) and webs
    for sg in (1, -1):
        s.append(B(0, n["pusher_t"], *sorted((sg * 12.0, sg * Y_BODY)), n["arm_w0"], ARM_LAP_W))
        yc, yd = sorted((sg * 12.0, sg * (n["y_lip0"] - 0.5)))
        s.append(B(0, n["pusher_t"], yc, yd, n["w_deck0"], ARM_LAP_W))                 # web deck -> stub (inside the lips)
    c = prim.union(s)
    c = c ^ B(-100, 400, -Y_BODY, Y_BODY, -100, 300)        # body inside the shoe faces: prints on its side, flat
    c = c + MZ.muzzle_carriage_add(P, n)                    # lip roller fork and retrieval funnel (launch_muzzle.py)
    for sg in (1, -1):                                       # keyway slots for the yoke key plates
        c = c - B(KEY_U[0], KEY_U[1], *sorted((sg * 12.9, sg * 17.6)), 61.5, ARM_LAP_W + 1.0)
    for sg in (1, -1):                                       # arm bolt pilots (M3 self tap, from above)
        c = c - cyl_w(1.3, n["arm_w0"] - 6.0, ARM_LAP_W + 0.5, ARM_BOLT_U, sg * ARM_BOLT_Y)
    return c


def carriage_yoke_solid(P):
    """Bolted YOKE (one part, carriage local frame) across the carriage rear: both spring arms (they hit the stop units
    and carry the spring pegs at y +-38 and the D5 provision pegs at +-56), a cross bar behind the pusher (bears on the
    body's rear faces: spring pull), front keys in front of the body webs (stop impact), tongues over the body stubs
    (1 x M3 per side down into the webs). Each arm's stop moment balances across the cross bar.
    Left: SW_SEATED block (pushrod bore, plunger stem bore). Right: rigid SW_HOME foot.
    Both feet run from local u -30 to -0.5 so the rail lever screw stays covered over the stop overshoot."""
    n = L(P)
    up = n["u_peg_c"]
    st = seat_geom(P)
    a0, a1 = n["arm_w0"], n["arm_w1"]
    s = []
    for sg in (1, -1):
        # Arm: a planar Warren truss (windows through w, vertical as printed) between a 3 mm front chord (stop pad
        # face, u 5..8) and a 2.5 mm rear chord (u -8..-5.5), solid root posts, a boss round the spring peg that also
        # backs the pad (y 33..41). Root check (y 24, through the windows): stop 148 N at the pad centre y 37, lever
        # 13 mm -> 1924 Nmm. Chords 3 x 9.8 at u 6.5 and 2.5 x 9.8 at u -6.75 (diagonals ignored): centroid u 0.5,
        # I = 29.4 * 6.0^2 + 24.5 * 7.25^2 + 9.8 * (27 + 15.6) / 12 = 2380 mm4, c = 8.5 -> Z = 280 mm3 -> 6.9 MPa
        # (cross bar side, y 21.5: 148 * 15.5 / 280 = 8.2 MPa) << 30 MPa, stress along y in the layers (prints w up).
        # Pad face chord between truss nodes (span about 12, backed by the peg boss over y 32.5..43.5): 148 * 12 / 8
        # / (9.8 * 3^2 / 6) = 15 MPa worst case.
        arm = B(0, n["pusher_t"], *sorted((sg * Y_BODY, sg * (Y_SPRING2 + 5))), a0, a1)
        arm = arm + B(-6.0, n["pusher_t"], *sorted((sg * 20.0, sg * (Y_SPRING2 + 5))), a1 - 0.01, 83.0)   # taller
        arm = arm + B(-8.0, 0.01, *sorted((sg * Y_BODY, sg * 50.0)), a0, a1)                       # stiffener
        arm = arm + B(-8.0, -5.9, *sorted((sg * Y_BODY, sg * 50.0)), a1 - 0.01, 83.0)              # rear chord top
        win = _truss_windows(*sorted((sg * Y_BODY, sg * 50.0)), -5.5, 5.0, a0 - 1.0, 84.0, 2.5, plane="yu")
        win = win - cyl_w(5.5, a0 - 2.0, 85.0, up, sg * n["y_spring"])
        s.append(arm - win)
        s.append(B(0, n["pusher_t"], *sorted((sg * 13.0, sg * Y_BODY)), ARM_LAP_W, a1))           # tongue
        s.append(B(KEY_U[0] + 0.1, KEY_U[1] - 0.1, *sorted((sg * 13.1, sg * 17.4)), 62.0, ARM_LAP_W + 0.01))  # key
        for yy in (n["y_spring"], Y_SPRING2):
            s.append(cyl_w(3.0, n["w_spring"] - 9, a0 + 1, up, sg * yy))
            s.append(cyl_w(4.5, n["w_spring"] - 11, n["w_spring"] - 9, up, sg * yy))            # keeper flange
    s.append(B(-8.0, 0.0, -Y_BODY, Y_BODY, a0, a1))                                            # cross bar
    s.append(B(-14.0, 0.0, 11.0, 21.5, a0, st["w_push"] + 4.0))                                # SW_SEATED block
    s.append(B(0, n["pusher_t"], 11.2, 20.0, a1 - 0.01, st["w_push"] + 4.0))                   # pushrod lug
    s.append(B(-32.0, -6.0, -28.0, -Y_BODY, a0, ARM_LAP_W))                                    # right foot carrier
    y = prim.union(s)
    y = y - cyl_u(1.85, -15.0, n["pusher_t"] + 1.0, st["y_push"], st["w_push"])               # rod bore
    y = y - cyl_u(2.7, -15.0, 0.0, st["y_push"], st["w_push"])                                # collar counterbore
    y = y - cyl_w(2.25, a0 - 2.0, st["w_push"] + 1.5, st["u_plunger_local"], STEM_Y)          # stem bore
    y = y - cyl_w(1.25, a0 - 2.0, st["w_push"] + 10.0, st["u_plunger_local"], PIN_Y)          # pin bore (through)
    for sg in (1, -1):                       # spring pegs hollow (d3 blind from the flange end): lighter, and an optional
        for yy in (n["y_spring"], Y_SPRING2):    # d3 steel pin pressed in carries the hook load across the layers
            y = y - cyl_w(1.5, n["w_spring"] - 12.0, a0 + 4.0, up, sg * yy)
    y = y - B(FOOT_U0 - 0.5, FOOT_U1 + 1.1, 12.5, 28.5, a0 - 2.0, a0 + 1.4)                  # leaf travel recess
    for sg in (1, -1):
        y = y - cyl_w(1.7, ARM_LAP_W - 1.0, a1 + 1.0, ARM_BOLT_U, sg * ARM_BOLT_Y)
        y = y - cyl_w(2.9, a1 - 2.5, a1 + 12.0, ARM_BOLT_U, sg * ARM_BOLT_Y)                  # head counterbore
    return y + home_foot_solid(P)


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
    from manifold3d import Manifold
    pts = []
    for da, hz in ((-(BUMP_DEG + BUMP_RAMP_DEG), 0.01), (-BUMP_DEG, -BUMP_H), (BUMP_DEG, -BUMP_H),
                   (BUMP_DEG + BUMP_RAMP_DEG, 0.01)):
        for rr in (BUMP_R - 3.0, BUMP_R + 3.0):
            a = math.radians(a0 + da)
            pts.append((rr * math.cos(a), rr * math.sin(a), hz))
            pts.append((rr * math.cos(a), rr * math.sin(a), 0.01))
    g = g + Manifold.hull_points(pts)                                     # cam with 22 deg ramps both ways
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
    # spacer collars to both cheek faces (0.2 clear): the drive gear cannot walk off the sprag rib (review 2)
    ym = 0.5 * (n["y_full"][0] + n["y_full"][1])
    za, zb = (n["y_chA"][1] + 0.2) - ym, (n["y_chB"][0] - 0.2) - ym
    g = g + prim.tube(16.0, 13.4, -(sp["w"] + 2.0) / 2 - za + 0.01, z=za)
    g = g + prim.tube(sp["od"] / 2 + 0.1, 13.4, 0.8, z=-(sp["w"] + 2.0) / 2)      # lip: ties the collar on, seats the sprag
    return g


def drive_washer_solid(P):
    """Printed washer on the sleeve between the drive gear hub and the left cheek (axial location)."""
    n = L(P)
    sp = dict(od=47.0, w=14.0)
    sp.update(getattr(P, "SPRAG", None) or {})
    ym = 0.5 * (n["y_full"][0] + n["y_full"][1])
    y0 = ym + (sp["w"] + 2.0) / 2 + 0.1
    return prim.tube(16.0, 13.4, (n["y_chB"][0] - 0.2) - y0, z=y0)        # bore passes the sun rib (r 12.8)


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
    pin = pin - prim.tube(6.0, 4.0, 1.6, z=y1 + 0.4)
    return pin - prim.box(4.5, 20.0, -20.0, 20.0, -100.0, 100.0)       # 0.5 flat: prints lying on its side


def gear_washer_solid(P):
    """Printed spacer washer on the big gear pin between the gear hub (full-layer face) and the left cheek."""
    n = L(P)
    y0 = n["hub_y"][1] + 0.1
    return prim.tube(9.0, 5.2, (n["y_chB"][0] - 0.1) - y0, z=y0)


RIB_W, RIB_R = 6.0, 12.8      # integral key rib on the sleeve (D13): 6 wide, top at r 12.8 (sprag keyway depth 2.8)
SPIGOT_BORE = 12.35           # cad-drum d12 spigot runs in this bore the whole sleeve length


def sleeve_solid(P):
    """Launch sleeve along +z from z = 0 (flange face towards the motor, world y_flange) to the outboard end.
    d20 journals, integral key rib (no keyway cut: D13) for sprag #1 and the splitter sun, flange for the drum's
    LAUNCH dog ring (D37: 3 lugs 6 x 3 on the flange face into the ring slots, 3 x M3 at r 28.5 into the tapped ring)."""
    n = L(P)
    z_end = n["Y0"] + 53.0 - n["y_flange"]
    s = prim.cyl(10.0, z_end) + prim.cyl(32.0, 8.0)
    zg = n["Y0"] + 0.5 * (n["y_full"][0] + n["y_full"][1]) - n["y_flange"]
    z_sun1 = n["Y0"] + 42.0 - n["y_flange"]
    # two rib lengths: under sprag #1 (between the rail cheeks) and under the sun; never inside a cheek bore
    s = s + prim.box(9.0, RIB_R, -RIB_W / 2, RIB_W / 2, zg - 7.5, zg + 7.5)
    s = s + prim.box(9.0, RIB_R, -RIB_W / 2, RIB_W / 2, z_sun1 - 12.0, z_sun1)     # full sun face (y 170..182)
    s = s - prim.cyl(SPIGOT_BORE / 2, z_end + 2, z=-1)
    # D37: 3 lugs 6.0 wide x 3.0 tall on the flange face (r 15..21, at 60/180/300 deg) into cad-drum's ring slots,
    # 3 x M3 clearance holes at r 28.5 (0/120/240 deg), screws from this side (counterbored heads) into the tapped ring
    for k in range(3):
        a = math.radians(120 * k)
        x, y = 28.5 * math.cos(a), 28.5 * math.sin(a)
        s = s - prim.cyl(1.7, 10, x=x, y=y, z=-1)
        s = s - prim.cyl(3.0, 3.0, x=x, y=y, z=8.0 - 3.0 + 0.01)
        lug = prim.box(15.0, 21.0, -3.0, 3.0, -3.0, 0.01)
        s = s + prim.place(lug, tf.Rz(60 + 120 * k))
    return s


def spring_segment_solid(P, length):
    n = L(P)
    ro = n["spring_od"] / 2
    return prim.cylX(ro, 0, length) - prim.cylX(ro - n["spring_wire"], -1, length + 1)


HOOK_L = 12.0     # pin centre to the first coil


def hook_solid(P, sign=1):
    """Spring hook loop round a d8 pin (ring about w) with a neck to the coils (towards +u for sign 1)."""
    n = L(P)
    r = n["spring_wire"]
    ring = prim.tube(ANCHOR_D / 2 + 2 * r, ANCHOR_D / 2 + 0.05, r * 2, z=-r)
    neck = prim.box(ANCHOR_D / 2, HOOK_L + 0.01, -r, r, -r, r)
    neck = neck + prim.cylX(n["spring_od"] / 2, HOOK_L - 2.0, HOOK_L) - prim.cylX(n["spring_od"] / 2 - r, HOOK_L - 3, HOOK_L + 1)
    h = ring + neck
    return h if sign > 0 else prim.place(h, tf.Rz(180))


ANCHOR_D = 8.0     # spring anchor pins: 8 mm steel rod (Harry's stock), vertical, double shear (post + strap)
ANCHOR_F = 8.5     # fallback hole set F rearward of the main holes (the stop frame front ends at u 248.15: 8.5 max)


def anchor_pin_solid(P):
    """8 mm steel rod (vertical) through post, hook and strap (double shear), flush with the strap top + 1.5."""
    n = L(P)
    h = n["w_spring"] + 12.5 + 1.5 - (n["w_floor0"] - 1.0)
    return prim.cyl(ANCHOR_D / 2, h)


N_SEG = 5
Y_BODY = 21.5         # carriage body half width (shoe faces): body prints on its side on them
ARM_LAP_W = 76.0      # half-lap joint plane between the body stub and the bolted spring arm
ARM_BOLT_U, ARM_BOLT_Y = 6.0, 15.0
KEY_U = (0.8, 3.2)            # yoke key plates in slots of the body webs (bear both ways in u)
FOOT_U0, FOOT_U1 = -30.0, -0.5   # SW feet span (carriage local): covers the lever screw over the stop overshoot
SPLICE_Y = 25.5      # top splice strap centre line (on the wall top / lip)
Y_SPRING2 = 56.0      # second spring pair provision (D5): pegs, anchor holes and clear space at y +-56, w 60
LUG_BOLTS_U = (107.0, 123.0)   # launch_aim lug_solid bolt positions (w 41)
RACK_BACK = 3.0       # rack body above the tooth roots (carriage mass: stress check in carriage_solid)
DECK_T = 4.0          # deck rib tops above the tooth roots (= launch_muzzle.RACK_BACK_MZ: the fork and funnel sit on them)
RING_U = (14.0, 78.0, 146.0)  # cradle rings (carriage local u): rear, middle, front; the V strips run between them
RING_T, RIB_T = 4.0, 3.0      # cradle ring width and ring-to-shoe rib thickness along u (carriage mass)
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


def _uw_extrude(pts, y0, y1):
    """Prism from a polygon in the (u, w) plane, spanning y0..y1 (carriage local or rail frame)."""
    cs = prim.poly_cs(pts)
    sol = prim.extrude(cs, (y1 - y0), z=0.0)                    # x = u, y = w, z
    return prim.place(sol, tf.mul(tf.T(0, y1, 0), tf.Rx(90)))   # z -> -y: spans y1 .. y0


def _foot(P, y0, y1):
    """A switch foot from FOOT_U0 to FOOT_U1 with 27 deg ramps at both ends (the screw tip meets a slope)."""
    st = seat_geom(P)
    wb = st["w_tip"]
    f = B(FOOT_U0, FOOT_U1, y0, y1, wb, wb + 1.6)
    f = f - _uw_extrude([(FOOT_U1 + 0.01, wb - 0.01), (FOOT_U1 + 0.01, wb + 1.3), (FOOT_U1 - 3.0, wb - 0.01)], y0 - 0.1, y1 + 0.1)
    f = f - _uw_extrude([(FOOT_U0 - 0.01, wb - 0.01), (FOOT_U0 + 3.0, wb - 0.01), (FOOT_U0 - 0.01, wb + 1.3)], y0 - 0.1, y1 + 0.1)
    return f


def home_foot_solid(P):
    """Rigid SW_HOME foot under the right spring arm (part of the yoke)."""
    return _foot(P, -28.0, -Y_BODY)


def _wedge_half(P, upper=True):
    """Half space above (upper) or below the 45 deg wedge plane w = w0 - (u - u_p) (carriage local)."""
    st = seat_geom(P)
    up, w0 = st["u_plunger_local"], st["w_push"] - 0.8
    if upper:
        pts = [(up - 40.0, w0 + 40.0), (up + 40.0, w0 - 40.0), (up + 40.0, w0 + 60.0), (up - 40.0, w0 + 60.0)]
    else:
        pts = [(up - 40.0, w0 + 40.0), (up - 40.0, w0 - 60.0), (up + 40.0, w0 - 60.0), (up + 40.0, w0 - 40.0)]
    return _uw_extrude(pts, -100.0, 100.0)


STEM_Y, PIN_Y = 17.0, 18.6     # plunger stem (r 2) sits 2 mm beside the pushrod so a thin pin (r 1) passes it


def seat_plunger_solid(P):
    """SW_SEATED plunger (PPA-CF, carriage local, pressed): foot pad under the left arm, stem (r 2) whose 45 deg chamfer
    lies under the pushrod's wedge end, and a thin pin (r 1) beside the pushrod up through the yoke block to a head that
    rests on the block top when the plunger is free (so it cannot drop). Pad, stem and pin print as one part: push the
    pin up through the yoke block from below, then press the printed head cap onto the pin top."""
    n = L(P)
    st = seat_geom(P)
    up = st["u_plunger_local"]
    pad = _foot(P, 13.0, 28.0)
    stem = cyl_w(2.0, st["w_tip"] + 1.59, st["w_push"] + 1.0, up, STEM_Y) ^ _wedge_half(P, upper=False)
    pin = cyl_w(1.0, st["w_tip"] + 1.59, SEAT_HEAD_W, up, PIN_Y)
    return pad + stem + pin


SEAT_HEAD_W = 91.2       # pin top (pressed); the head cap sits on it, 1.2 above the block top (w 90) when pressed


def seat_clip_solid(P):
    """Head cap pressed onto the plunger pin top: rests on the yoke block when the plunger is free."""
    st = seat_geom(P)
    c = cyl_w(2.8, SEAT_HEAD_W - 1.2, SEAT_HEAD_W + 1.2, st["u_plunger_local"], PIN_Y)
    return c - cyl_w(1.0, SEAT_HEAD_W - 1.3, SEAT_HEAD_W, st["u_plunger_local"], PIN_Y)


def seat_pushrod_solid(P):
    """Pushrod d3.2, inserted from the rear of the yoke block BEFORE the plunger: a collar (r 2.4) stops it at the lug
    step (forward limit, 1.2 mm of free travel), its rear end is the 45 deg wedge over the plunger stem's chamfer (the
    stem stops it going back). Pressed position: front flush with the pusher face. Top flat for printing."""
    n = L(P)
    st = seat_geom(P)
    up = st["u_plunger_local"]
    r = cyl_u(1.6, up - 3.0, n["pusher_t"], st["y_push"], st["w_push"])
    r = r + cyl_u(2.4, SEAT_COLLAR_U[0], SEAT_COLLAR_U[1], st["y_push"], st["w_push"])
    r = r ^ _wedge_half(P, upper=True)
    return r - B(-100, 100, -100, 100, st["w_push"] + 1.2, 200)


SEAT_COLLAR_U = (-3.0, -1.2)     # pushrod collar (pressed); the lug step is at u 0


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
    yk = carriage_yoke_solid(P)
    out.append(Part("launch_carriage_yoke", yk, "PLA", joint="launch_carriage", place=tf.T(n["u_c_home"], 0, 0),
                    print_tf=best_bed(yk), color="#c86f00",
                    notes="Bolted yoke: both spring arms (hit the stop, carry the spring pegs), cross bar, keys, "
                          "SW_SEATED block, rigid SW_HOME foot"))
    bg = big_gear_solid(P)
    out.append(Part("launch_big_gear", bg, "PPA-CF", joint="launch_big_gear",
                    place=tf.mul(tf.T(0, n["y_sec"][0], 0), tf.Rx(-90)), print_tf=bed(bg, tf.Rx(180)), color="#3d7cc9",
                    notes=f"Big gear m2.5: sector layer {n['n_present']} of 40 teeth (rack, v10 release teeth) + full 40T "
                          f"layer (drive), SW_GAP cam bump, on the rail pin"))
    gw = gear_washer_solid(P)
    out.append(Part("launch_gear_washer", gw, "PLA", joint="launch_rail", place=tf.mul(tf.T(n["cd"], 0, 0), tf.Rx(-90)),
                    print_tf=bed(gw), color="#5a5f65", notes="Spacer washer: big gear hub to the left cheek"))
    gp = gear_pin_solid(P)
    out.append(Part("launch_gear_pin", gp, "PPA-CF", joint="launch_rail", place=tf.mul(tf.T(n["cd"], 0, 0), tf.Rx(-90)),
                    print_tf=best_bed(gp), color="#3a3f45", notes="Snap pin d10 for the big gear, C clip; prints lying on its flat"))
    out.append(Part("launch_gear_clip", clip_solid(P), "PLA", joint="launch_rail",
                    place=tf.mul(tf.T(n["cd"], 0, 0), tf.Rx(-90)), print_tf=bed(clip_solid(P)), color="#5a5f65",
                    notes="C clip on the big gear pin groove"))
    ym = 0.5 * (n["y_full"][0] + n["y_full"][1])
    dg = drive_gear_solid(P)
    out.append(Part("launch_drive_gear", dg, "PPA-CF", joint="launch_drive", place=tf.mul(tf.T(0, ym, 0), tf.Rx(-90)),
                    print_tf=bed(dg), color="#2f6db5", notes="Drive gear m2.5 26T, sprag #1 inside on an integral key rib"))
    dw = drive_washer_solid(P)
    out.append(Part("launch_drive_washer", dw, "PLA", joint="launch_drive", place=tf.Rx(-90), print_tf=bed(dw),
                    color="#5a5f65", notes="Spacer washer: drive gear hub to the left cheek"))
    out.append(Part("launch_sprag", sprag_solid(P), "bought", joint="launch_drive", place=tf.mul(tf.T(0, ym, 0), tf.Rx(-90)),
                    color="#777c84", bought_ref="CSK20PP 20x47x14 sprag #1", notes="Sprag #1: forward wiper cocks"))
    sl = sleeve_solid(P)
    out.append(Part("launch_sleeve", sl, "PPA-CF", joint="launch_sleeve",
                    place=tf.mul(tf.T(0, n["y_flange"] - n["Y0"], 0), tf.Rx(-90)), print_tf=bed(sl, tf.Rx(180)), color="#3a3f45",
                    notes="Launch sleeve d20: dog ring flange, integral key ribs (sprag #1, splitter sun), journals"))
    # springs: hooks + coil segments that spread as the carriage moves
    seg_len = (n["L_home"] - 2 * HOOK_L) / N_SEG
    spref = (getattr(P, "SPRING", None) or {}).get("name", "extension spring") + " extension spring"
    for sg, side in ((1, "l"), (-1, "r")):
        y = sg * n["y_spring"]
        ap = anchor_pin_solid(P)
        out.append(Part(f"launch_anchor_pin_{side}", ap, "steel", joint="launch_rail",
                        place=tf.T(n["u_anchor"], y, n["w_floor0"] - 1.0), color="#3a3f45",
                        bought_ref="8 mm steel rod, cut to length (see note), ends deburred",
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
    for sg, side in ((1, "l"), (-1, "r")):
        sp = splice_solid(P, sg)
        out.append(Part(f"launch_rail_splice_{side}", sp, "PLA", joint="launch_rail", place=tf.I4, print_tf=bed(sp),
                        color="#8796a5", notes="Top splice strap over the front rail joint (4 x M3 x 10 self tap)"))
    out += ST.stop_parts(P, n)
    out += MZ.muzzle_parts(P, n, bed) + GD.guard_parts(P, n)
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
                    print_tf=best_bed(pl), color="#3a3f45",
                    notes="SW_SEATED plunger: foot pad + stem with a 45 deg chamfer, clip groove on top"))
    pr = seat_pushrod_solid(P)
    out.append(Part("launch_seat_pushrod", pr, "PPA-CF", joint="launch_carriage", place=tf.T(n["u_c_home"], 0, 0),
                    print_tf=best_bed(pr), color="#3a3f45", notes="SW_SEATED pushrod d3.2 with the stem slot"))
    cl = seat_clip_solid(P)
    out.append(Part("launch_seat_clip", cl, "PLA", joint="launch_carriage", place=tf.T(n["u_c_home"], 0, 0),
                    print_tf=best_bed(cl), color="#5a5f65", notes="Head cap on the plunger pin (rests on the yoke block)"))
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
        Joint("launch_carriage", "launch_rail", "slide", (0, 0, 0), (-1, 0, 0), limits=(-ST.TRAVEL - 0.5, n["stroke"] + 0.5)),
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
    if sleeve_cmd is None and S.get("wiper_deg") is not None and not any(
            S.get(k) is not None for k in ("aim_deg", "aim_psi_from_low", "aim_split", "cock", "phase_aim")):
        sleeve_cmd = S["wiper_deg"]      # cad-drum's states (wiper only): in LAUNCH its dog ring turns with the wiper
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
        held = (_f(S, "cock", 0.0) > 0.0 or (bool(_f(S, "fired", 0.0)) and S.get("phase") == "fire"))
        if held and S.get("aim_deg") is not None and hasattr(AIM, "psi_of_pitch"):
            # cocking / the shot: the self-locking worm holds the crank where the aim left it (review 3)
            pp = AIM.psi_of_pitch(P, float(np.clip(_f(S, "aim_deg", pmin), pmin, pmax)))
            if pp is not None:
                d = float(pp[1])
                wb = d * K.wiper_per_crank(P)
        def s_eng(dd):
            return AIM.kin_aim(P, {"aim_psi_from_low": dd % 360.0}, float(K.pitch_of_psi(P, (t["psi_low"] - dd) % 360.0)))["launch_sleeve"]
        K0 = s_eng(0.0)
        if not held:
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
            off = (s_eng - float(sleeve_cmd) + 6.0) % 12.0 - 6.0
            if not cocking and abs(off) > 0.5:
                # sleeve off the 12 deg sun phase (forward of LOW): the splitter arm swings out, the crank holds
                out.update(AIM.kin_aim(P, dict(S_aim, aim_split="out", wiper_deg=float(sleeve_cmd)), pitch))
            # the sleeve follows sleeve_deg exactly: cad-drum's LAUNCH dog ring is bolted to it (D37 lugs)
            out["launch_sleeve"] = float(sleeve_cmd)
        elif cocking:
            S_out = dict(S_aim, aim_split="out", wiper_deg=s_eng + drive_rel)
            out.update(AIM.kin_aim(P, S_out, pitch))
            out["launch_sleeve"] = s_eng + drive_rel
        elif fired:
            out["launch_sleeve"] = s_eng + 12.0 * round(drive_rel / 12.0)      # keeps the sun phased with the idler
        if sleeve_cmd is None and not cocking and S.get("wiper_deg") is not None:
            # timeline states with wiper_deg but no sleeve_deg (load): cad-drum's dog ring sits at wiper_deg, and it is
            # bolted to the sleeve (D37), so the sleeve does too; off the sun phase the splitter arm is out
            wv = float(S["wiper_deg"])
            if abs((out["launch_sleeve"] - wv + 6.0) % 12.0 - 6.0) > 0.5:
                out.update(AIM.kin_aim(P, dict(S_aim, aim_split="out", wiper_deg=wv), pitch))
            out["launch_sleeve"] = wv
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
    if S.get("phase") == "to_launch" or rt >= 1.0:
        # home on the sled: to_launch winds dart_flight and dart_return back to 0 together (a reset, not a flight)
        return 0.0, 0.0, 0.0
    lf = dict(range_m=4.5, apex_m=1.0, t_flight_s=1.0)
    lf.update((getattr(P, "LAUNCH", None) or {}).get("flight", {}) or {})
    R, H = lf["range_m"] * 1000.0, lf["apex_m"] * 1000.0
    if rt > 0.0:
        # reeled home tail first: from the landing point up to the rail axis line well ahead of the muzzle, then
        # straight along the axis onto the sled (the line runs over the lip roller and through the funnel)
        u_in = 320.0
        if rt < 0.7:
            s = rt / 0.7
            X0, Z0 = R, -165.0
            c0, s0 = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
            X1, Z1 = u_in * c0, u_in * s0                    # the point u_in ahead on the rail axis (world offsets)
            X, Z, ang = X0 + (X1 - X0) * s, Z0 + (Z1 - Z0) * s, 0.0
        else:
            return u_in * (1.0 - (rt - 0.7) / 0.3), 0.0, 0.0
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
        Allow("launch_spring_*_hook_c", "launch_carriage_yoke", "hook loop round the arm peg"),
        Allow("launch_stop_spring_*", "launch_carriage_yoke", "arm on the stop pad (contact)"),
        Allow("launch_spring_*", "launch_spring_*", "segmented spring model telescopes when the carriage overshoots"),
        Allow("launch_seat_clip", "launch_seat_plunger", "head cap pressed on the pin"),
        Allow("launch_sw_home_screw", "launch_sw_home", "screw threaded into the lever"),
        Allow("launch_sw_seated_screw", "launch_sw_seated", "screw threaded into the lever"),
    ] + LN.allowed_line() + ST.stop_allowed() + MZ.muzzle_allowed() + GD.guard_allowed() + _aim("allowed_aim", P)


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
    S += ST.stop_audit_states(P)
    for st in _aim("audit_states_aim", P):
        if st.get("aim_split") == "out" and st.get("wiper_deg") is not None and st.get("sleeve_deg") is None:
            st = dict(st, sleeve_deg=st["wiper_deg"])        # cad-drum's dog ring follows sleeve_deg (D37 lugs)
        S.append(st)
    return S


def checks(P, asm):
    """Carriage stop overshoot (the compressed flexure is not modelled as a moving part): at overshoot d the arms may
    touch the stop's rear bar only (contact), never the swept stack / frame / ledges (keepout), and the hard stop only
    at full travel."""
    n = L(P)
    fails = []
    arms = [prim.place(carriage_yoke_solid(P), tf.T(n["u_c_home"], 0, 0))]
    body = prim.place(carriage_solid(P), tf.T(n["u_c_home"], 0, 0))
    for d in np.linspace(0.0, ST.TRAVEL, 7):
        mov = [a.translate((d, 0, 0)) for a in arms] + [body.translate((d, 0, 0))]
        for name, sol, role in ST.stop_overshoot_solids(P, n, d):
            v = sum((m ^ sol).volume() for m in mov)
            if role == "keepout" and v > 1e-3:
                fails.append(f"stop overshoot {d:.1f} mm: carriage enters {name} ({v:.2f} mm3)")
            if role == "contact" and v > 1.0:
                fails.append(f"stop overshoot {d:.1f} mm: carriage overlaps the stop bar {name} ({v:.2f} mm3)")
            if role == "hardstop" and d < ST.TRAVEL - 1e-6 and v > 1e-3:
                fails.append(f"stop overshoot {d:.1f} mm: hard stop touched early ({v:.2f} mm3)")
    return fails


def grounded(P):
    """The aim housing stands on the deck; everything else must connect to it (or to other subsystems' grounded
    parts) through contacts: rail cheeks on the sleeve, sleeve in the housing bushing, conrod to the crank."""
    return ["launch_aim_housing"]


def GD_PART_NAMES(P):
    return [p.name for p in GD.guard_parts(P, L(P))]


def MZ_PART_NAMES(P):
    return [p.name for p in MZ.muzzle_parts(P, L(P), bed)]


def steps(P):
    """Assembly steps for the launcher and aim train (plain language, hardware per step)."""
    n = L(P)
    sw = [f"launch_spring_{sd}_{i}" for sd in "lr" for i in range(N_SEG)]
    out = [
        dict(title="Launcher 1: print the parts",
             text="All PLA unless named. PPA-CF: big gear, drive gear, launch sleeve, big gear pin, SW_SEATED plunger and "
                  "pushrod, and the aim gears (sun, idler, output, bevel, worm, crank). Use each part's print orientation "
                  "from the print sheet: the carriage body lies on its side across the bed diagonal; the stop springs "
                  "print upside down at 100 percent infill (all perimeters); rail pieces lie on their largest flat face. "
                  "0.2 mm layers, 4 walls on every loaded part. Hardware: M3 screws (x6, x10, x12, x16, x20) and nuts, "
                  "2 x 8 mm steel rod pins (cut to the length on the print sheet) for the spring anchors, 3 x 693ZZ for the tether "
                  "pulleys, sprag #1 (CSK20PP), the two springs.",
             parts=[]),
        dict(title="Launcher 2: join the rail",
             text="Lay the rear, mid and front rail pieces in a line on a flat table. Push each joint together and fit "
                  "2 x M3 x 20 with nuts through each flange pair (8 in total). Tighten evenly; file any step inside the "
                  "track where the pieces meet so the carriage shoes slide over it.",
             parts=["launch_rail_rear", "launch_rail_mid", "launch_rail_front"]),
        dict(title="Launcher 2b: splice straps and spring guards",
             text="Screw a splice strap on each wall top across the front joint (4 x M3 x 10 self tap each). Screw the "
                  "spring guard plates to the rail walls and anchor posts (M3 self tap into the pilots, see the print "
                  "sheet), three per side: rear, middle (under the stop units), front. Fit them before the springs.",
             parts=["launch_rail_splice_l", "launch_rail_splice_r"] + [p for p in GD_PART_NAMES(P)]),
        dict(title="Launcher 3: big gear",
             text="Hold the big gear between the two cheeks of the mid piece, the toothed sector towards the drum side "
                  "(right) and the full ring of teeth towards the left, with the spacer washer between the gear and the "
                  "left cheek. Push the big gear pin through cheek, gear, washer, cheek from the right and snap the C clip "
                  "into its groove. The gear must spin freely with almost no end play.",
             parts=["launch_big_gear", "launch_gear_pin", "launch_gear_clip", "launch_gear_washer"]),
        dict(title="Launcher 4: drive gear, sprag #1 and the launch sleeve",
             text="First bolt cad-drum's LAUNCH dog ring to the sleeve flange: lugs in the ring slots, 3 x M3 x 10 from the "
                  "flange side into the tapped ring (the heads are out of reach once the sleeve is in the cheeks). "
                  "Slide sprag #1 into the drive gear so its outer keyway takes the rib inside the gear. Check the "
                  "direction: holding the gear, the inner race must turn freely only the BACKWARD wiper way. Hold the drive "
                  "gear between the cheeks (collar side right, then the spacer washer on its left), meshing the big gear's "
                  "full ring of teeth. Turn the cheek bores so their slots "
                  "point down, then push the launch sleeve in from the motor side, outboard end first, ribs lined up with "
                  "the slots and the sprag keyway, until the long (15 mm) rib sits inside the sprag.",
             parts=["launch_drive_gear", "launch_sprag", "launch_sleeve", "launch_drive_washer"]),
        dict(title="Launcher 5: carriage",
             text="Turn the big gear so its toothless gap faces the rack. Slide the carriage BODY into the track from the "
                  "FRONT end (the tether fork blocks the rear), rack teeth down, until its rear face reaches the stop "
                  "position. Yoke: push the SW_SEATED pushrod into the block bore from the rear, collar first, then push "
                  "the plunger's pin up through the block from below (pad under the left arm) and press the head cap on "
                  "the pin top. Lower the yoke onto the carriage rear so its key plates drop into the slots in the body "
                  "webs, and fit 1 x M3 x 16 per side through the yoke into the webs.",
             parts=["launch_carriage", "launch_carriage_yoke", "launch_seat_plunger", "launch_seat_pushrod",
                    "launch_seat_clip"]),
        dict(title="Launcher 5b: lip roller",
             text="Press a 693ZZ into the lip roller and fit it in the fork at the carriage front with the M3 x 20 and a "
                  "nyloc nut through the side counterbores; it must spin freely. The tether runs over it in tow and "
                  "retrieval, and the dart's tail slides in over it through the funnel onto the cradle.",
             parts=MZ_PART_NAMES(P)),
        dict(title="Launcher 6: carriage stops",
             text="For each side: put the stop spring on its ledges over the main spring position, rear bar towards the "
                  "carriage arm. Fit the stop frame over it: 2 x M3 x 12 through the rear foot into the rail wall and "
                  "2 x M3 x 16 through the front foot into the rail front plate. Then pull the spring onto the frame's "
                  "front wall with the 2 x M3 x 20 draw screws, a turn each in turn, until it sits on the wall (4.4 mm of "
                  "preload). The rear bar must rest on both ledges.",
             parts=["launch_stop_spring_l", "launch_stop_spring_r", "launch_stop_frame_l", "launch_stop_frame_r"]),
        dict(title="Launcher 7: contacts",
             text="SW_HOME (right lip) and SW_SEATED (left lip): drop an M3 nut into the slot in the rail wall with a "
                  "stripped wire end under it. Screw an M3 x 6 up through the lever's leaf, head down, then screw the "
                  "lever root to the lip with one M3 x 10 (self tap). With the carriage on its stop, the right foot presses "
                  "SW_HOME closed; set the screw so the head just touches the nut, then back it off a quarter turn and "
                  "check it closes with the carriage home and opens 10 mm back. SW_SEATED closes only with a dart on the "
                  "sled (no springs yet: hold the carriage on its stop by hand to check). SW_GAP, SW_RAIL_LOW and the "
                  "AS5600 carrier are fitted in the Sensors steps (cad-sensors).",
             parts=["launch_sw_home", "launch_sw_home_screw", "launch_sw_home_nut", "launch_sw_seated",
                    "launch_sw_seated_screw", "launch_sw_seated_nut"]),
        dict(title="Launcher 8: time the big gear (fingers clear)",
             text="No springs fitted yet. Hold the carriage on its stop by hand and turn the drive gear FORWARD until the "
                  "first tooth after the toothless gap just touches the rearmost rack tooth: this is HOME and SW_GAP "
                  "closes. Turning on pulls the carriage back smoothly; at full cock the cut last tooth lets go (push "
                  "the carriage home by hand). Hold the drive gear from the side and "
                  "never put a hand in front of the carriage.",
             parts=[]),
        dict(title="Launcher 9: tether pulleys",
             text="Press a 693ZZ into each printed pulley. Fit the top and bottom pulleys in the fork behind the rail "
                  "(M3 x 16 axle, nut) and the pivot pulley on the right cheek lug under the axis (M3 x 16 from inside). "
                  "Thread the line from the reel side under the pivot pulley, back under the rail to the bottom pulley, up "
                  "over the top pulley and forward through the pusher hole to the dart's tail cap. Never fire without the "
                  "tether.",
             parts=["launch_pulley_top", "launch_pulley_top_brg", "launch_pulley_top_axle", "launch_pulley_bot",
                    "launch_pulley_bot_brg", "launch_pulley_bot_axle", "launch_pulley_piv", "launch_pulley_piv_brg",
                    "launch_pulley_piv_axle", "launch_line_b", "launch_line_c", "launch_line_p"]
                   + [f"launch_line_a{k}" for k in range(LN.N_A)]),
        dict(title="Aim 1: aim housing, worm, wheel, crank",
             text="Build the aim housing on the bench (it goes onto the base later, after the rail). Mount the AS5600 "
                  "board on the housing's outer bracket, chip facing inwards. Build SW_RAIL_LOW like SW_GAP and screw it "
                  "to the housing where the crank cam presses it. Fit the two 608 bearings into the crank (worm wheel) hub and slide it "
                  "onto the M8 bolt through the housing outer wall; fit the nut. Lay the worm shaft with the small bevel in "
                  "its saddles and screw the cap on. Fit the countershaft with the output gear and the big bevel.",
             parts=["launch_aim_housing", "launch_aim_cap", "launch_aim_worm", "launch_aim_crank", "launch_aim_bolt",
                    "launch_aim_brg_1", "launch_aim_brg_2", "launch_aim_output", "launch_aim_bevel30"]),
        dict(title="Aim 2: splitter",
             text="Slide the sun onto the sleeve's outer rib. Push the printed drag pad into the swing arm hub pocket with the "
                  "shim behind its root block (a thicker shim gives more drag), slide the arm on next to the sun, fit the "
                  "idler on the arm pin and snap the printed C clip into the pin groove. Turn the sleeve backward by hand: the arm "
                  "swings the idler into the output gear and drives it; forward, the arm swings out onto its stop and the "
                  "output gear stays still.",
             parts=["launch_aim_sun", "launch_aim_arm", "launch_aim_idler", "launch_aim_drag_pad", "launch_aim_drag_shim",
                    "launch_aim_idler_clip"]),
        dict(title="Aim 3: lug, conrod, rail onto the base",
             text="Bolt the lug to the left rail wall (2 x M3 self tap). With the sun and swing arm already on the sleeve, "
                  "slide the rail assembly's sleeve along the main axis onto the drum's d12 spigot from the outboard side "
                  "(the LAUNCH dog ring is already bolted to its flange, Launcher 4) until the ring's dogs sit by the drum "
                  "collar. Then slide the aim housing on from the outboard side so the sleeve end enters its bushing, and "
                  "screw the housing to the deck (4 x M3). Join the conrod to the crank pin and the lug pin and snap a printed C clip into each pin groove. Turn the wiper "
                  "backward slowly: the rail rises to 47 deg, then lowers to 6 deg.",
             parts=["launch_aim_lug", "launch_aim_conrod", "launch_aim_crankpin_clip", "launch_aim_lugpin_clip"]),
        dict(title="Launcher 10: wiring (battery unplugged)",
             text="Battery unplugged. Run the SW_HOME, SW_SEATED and SW_GAP wires along the rail to the pivot and leave a "
                  "150 mm loop at the main axis before they go to the frame. No wire goes onto the carriage. Each contact: "
                  "one wire to its ESP32 pin (SW_HOME 21, SW_GAP 39, SW_RAIL_LOW 40, SW_SEATED 41, as params.SWITCHES), "
                  "the other to GND.",
             parts=[]),
        dict(title="Test stage 5 only: springs (NOT during assembly)",
             text="The springs go on at test stage 5, after the big gear timing, the contacts and the aim train have been "
                  "checked without them. Spring guards on first (never cock with a guard off) and never fire without the "
                  "tether attached (D11). Hook the back end of each spring over the inner peg of the yoke arm. Put the "
                  "front hook between post and strap, lined up with the main hole, and push the 8 mm steel rod pin down "
                  "through strap, hook and post. Hole set F, 8.5 mm rearward, is the fallback if the hang test shows the "
                  "spring needs less stretch. The outer holes are spare. The springs are slightly stretched and hold the "
                  "carriage against the stop. A drop of glue on the pin top stops it working up.",
             parts=["launch_anchor_pin_l", "launch_anchor_pin_r", "launch_spring_l_hook_a", "launch_spring_r_hook_a",
                    "launch_spring_l_hook_c", "launch_spring_r_hook_c"] + sw, stage=999),
    ]
    # hard order (cad-infra plan check): the rail bench sub-assembly keeps the authored order, springs last
    for i, st in enumerate(out):
        st.setdefault("stage", 10 + i)
    return out


def _rail_to_world(P, pitch):
    n = L(P)
    return tf.mul(tf.T(0.0, n["Y0"], n["axis_z"]), tf.R((0, -1, 0), pitch))


def _tangent_angle(c, r, q, side):
    """Angle (deg, in the (u, w) plane) of the tangent point on circle (c, r) for a line to point q; side +1 / -1."""
    dx, dz = q[0] - c[0], q[1] - c[1]
    d = math.hypot(dx, dz)
    base = math.degrees(math.atan2(dz, dx))
    return base + side * math.degrees(math.acos(min(1.0, r / d)))


def line_entry(P, S=None):
    """World point where the tether arrives at the rail: the BOTTOM of the pivot pulley (cad-reel's feed comes in
    from the front at about the same height and the line wraps under the pulley and round its rear). The pulley is
    44 below the main axis and turns with the rail, so the feed length changes by about 25 mm over the aim range
    (slack on the way down is taken up by the take-up during cocking)."""
    S = S or {}
    n = L(P)
    pitch = kin(P, S)["launch_rail"]
    pv = LN.geom(n)["piv"]
    p = (pv[0], pv[1], pv[2] - LN.R_PATH)
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
    bt, tp = g["bot"], g["top"]
    q = (bt[0], bt[2] - LN.R_PATH)                               # bottom pulley's lowest path point
    a_t = _tangent_angle((pv[0], pv[2]), LN.R_PATH, q, 1.0) % 360.0     # leaves at the bottom-rear towards it
    arc = []
    for a in np.linspace(270.0, min(a_t, 270.0), 5):                  # feed arrives at the bottom, wraps rearwards
        t = math.radians(a)
        arc.append((pv[0] + LN.R_PATH * math.cos(t), pv[1], pv[2] + LN.R_PATH * math.sin(t)))
    bot = [(bt[0] + LN.R_PATH * math.cos(math.radians(a)), 0.0, bt[2] + LN.R_PATH * math.sin(math.radians(a)))
           for a in np.linspace(270.0, 180.0, 5)]
    top = [(tp[0] + LN.R_PATH * math.cos(math.radians(a)), 0.0, tp[2] + LN.R_PATH * math.sin(math.radians(a)))
           for a in np.linspace(180.0, 90.0, 5)]
    pusher = (n["u_c_home"] - travel, 0.0, g["w_line"])
    tail_seat = np.array((n["u_tail_home"] - travel, 0.0, g["w_line"]))
    du, dw = v.get("launch_dart", 0.0), v.get("launch_dart_w", 0.0)
    tail = tail_seat + np.array((du, 0.0, dw))
    rail_pts = np.array(arc + bot + top + [pusher])
    spans = [(tf.apply(W, rail_pts), {"launch_pulley_piv", "launch_pulley_bot", "launch_pulley_top", "launch_rail_rear"}),
             (tf.apply(W, np.array([pusher, tail])), {"launch_carriage", "launch_dart", "launch_line_p"})]
    return spans


import launch_stop as ST                                        # noqa: E402  (imports launch: keep at the end)
import launch_muzzle as MZ                                      # noqa: E402
import launch_guard as GD                                       # noqa: E402
