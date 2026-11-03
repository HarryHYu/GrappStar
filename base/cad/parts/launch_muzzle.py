"""Lip roller and retrieval funnel at the carriage front (BASE_SPEC 8), worker cad-launch.

Carriage LOCAL frame throughout (rail frame minus u_c_home in u): u from the carriage rear face (muzzle +), y left,
w up. The carriage moves -u by the cock travel (0..241) and +u by up to 19 into the stop.

LIP ROLLER: the tether runs from the pusher hole (u 0..8, y 0, w 84.6) forward to the dart. In tow and retrieval it
bends down over the carriage front, so a printed pulley (launch_line pulley, 693ZZ, M3 axle along y) sits in a fork
on the carriage front: centre (U_R, 0, W_R), top W_R + 7 = 62.3 (below the dart belly 63.6 as the dart flies off),
bottom 48.3 (nothing of the carriage below the rack tips 47.6). Light fork (carriage mass): two 2.5 mm cheek plates
(|y| 3..5.5, top w 61.5) from the u 250 deck rib past the front rib to the roller hub, a floor between them behind
the roller, lightening windows. At full cock the hub roots pass over the big gear (tip circle r 52.5 about rail
(82.6, 0)) about 2 mm clear.

RETRIEVAL FUNNEL (light): the dart is reeled home tail first. Two 2 mm guide rails on a cone concentric with the dart
axis (w 85), 20..48 deg either side of the bottom (the tail cap rides in their V like on the cradle strips), open from
the cradle bore r 21.4 at u 148 to r 27 at the mouth ring (u 190..196, full cone section, floor 58), which stands on
the u 190 deck rib through a bulkhead; posts onto the u 160 rib. Cut at w 80, |y| <= 21.5, clear of the rail lips.
A tail cap (d 42) arriving low or off centre meets the rails and is steered up onto the cradle centreline. Outside
r 21.4 about w 85 it never touches the dart in flight (hull r 21 about w 84.6). Added volume about 4 cm3.

Integration (launch.py):
    import launch_muzzle as MZ
    carriage_solid():  c = c + MZ.muzzle_carriage_add(P, n)    # before the |y| <= Y_BODY clip is fine too
    parts():           out += MZ.muzzle_parts(P, n)
    allowed():         ... + MZ.muzzle_allowed()
    line_path():       the pusher -> dart span may pass over MZ.roller_top(n) (carriage local, + u_c_home - travel)
"""
from __future__ import annotations

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from cad.lib import Part, Allow, prim, tf   # noqa: E402
import launch_line as LN                     # noqa: E402

# roller (carriage local)
U_R = 281.0                 # roller centre u (forward enough that the hub clears the big gear at full cock)
W_R = 55.3                  # roller centre w: top 62.3, bottom 48.3
Y_CHEEK0 = 3.0              # cheek inner faces |y| (pulley 5 wide: 0.5 side clearance)
Y_CHEEK1 = 5.5              # cheek plate outer faces (2.5 thick; axle head / nut seat)
R_HUB = 5.0                 # cheek hub radius about the axle (inside the groove bottom r 6: the line never touches)
AXLE_L = 16.0               # M3 x 16 SHCS + nyloc nut: head at |y| 5.5, nut 5.5..9.5, 1 mm through
WASHER = (2.3, 1.65, 2.05, 2.95)   # thrust washer hub r, bore r, |y| from..to (the hub bears on the bearing inner
WASHER_FLANGE = (3.1, 0.5)         # race); flange r / thickness on the cheek side (flat print face), inside the
                                   # roller bore r 3.95; roller + washers 5.9 wide in the 6.0 slot
U_FORK0 = 250.0             # fork root (deck rib at u 250)
W_FORK1 = 61.5              # cheek top (dart belly 63.6 in flight)
R_PUL_CLEAR = 8.5           # floor ends this far behind the roller centre (pulley r 7)
RACK_BACK_MZ = 4.0          # = launch.RACK_BACK: deck ribs top at w_rroot + 4
U_FRONT = U_R + R_HUB       # carriage front (286): the body prints across the bed diagonal (L + W <= 362)

# funnel
U_F0, U_F1 = 148.0, 196.0
U_MOUTH = 190.0             # mouth ring over the u 190 deck rib
R_F0, R_F1 = 21.4, 27.0
F_WALL = 2.0                # like the cradle pipe
W_F_TOP = 80.0


def _n(P, n):
    if n is None:
        import launch as LA
        n = LA.L(P)
    return n


def B(u0, u1, y0, y1, w0, w1):
    return prim.box(u0, u1, y0, y1, w0, w1)


def Bs(u0, u1, y0, y1, w0, w1):
    return B(u0, u1, y0, y1, w0, w1) + B(u0, u1, -y1, -y0, w0, w1)


def _cone_u(r0, r1, u0, u1, w):
    """Cone along +u from radius r0 at u0 to r1 at u1, axis at (y 0, w)."""
    return prim.place(prim.cyl(r0, u1 - u0, r2=r1, seg=96), tf.mul(tf.T(u0, 0.0, w), tf.Ry(90)))


def _lip_keepout(n):
    """Rail lips seen from the carriage (both sides, whole length): |y| > 17.5 between the shoe top and lip top + 0.4."""
    return Bs(-100, 400, n["y_lip0"] - 0.5, 40.0, n["shoe_w1"], n["lip_w1"] + 0.4)


def _wedge_u(a0, a1, u0, u1, w):
    """Sector about the axis (y 0, w) between angles a0..a1 from straight down (+ towards +y), along u0..u1."""
    pts = [(0.0, 0.0)] + [(60.0 * math.sin(math.radians(a)), -60.0 * math.cos(math.radians(a)))
                          for a in [a0 + (a1 - a0) * i / 8 for i in range(9)]]
    cs = prim.poly_cs(pts)                                           # (x = y, y = w - w_axis)
    M = [[0.0, 0.0, 1.0, u0], [1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, w], [0.0, 0.0, 0.0, 1.0]]
    import numpy as np
    return prim.place(prim.extrude(cs, u1 - u0), np.array(M))       # (x, y, z) -> (u = z, y = x, w = y)


def funnel_solid(P=None, n=None):
    """Light lead-in: two guide rails on the cone surface 20..48 deg either side of the bottom (the tail cap rides in
    their V, as on the cradle strips) from the cradle front ring to a mouth ring (u 190..196, full cone section)
    over the u 190 deck rib; small posts onto the u 160 rib."""
    n = _n(P, n)
    wd = n["w_dart"]
    k = (R_F1 - R_F0) / (U_F1 - U_F0)
    inner = _cone_u(R_F0 - k * 1.0, R_F1 + k * 1.0, U_F0 - 1.0, U_F1 + 1.0, wd)
    outer = _cone_u(R_F0 + F_WALL, R_F1 + F_WALL, U_F0, U_F1, wd)
    env = B(U_F0, U_F1, -n["y_shoe1"], n["y_shoe1"], n["w_deck0"], W_F_TOP) - _lip_keepout(n)
    shell = (outer - inner) ^ env
    keep = _wedge_u(20.0, 48.0, U_F0 - 1, U_F1 + 1, wd) + _wedge_u(-48.0, -20.0, U_F0 - 1, U_F1 + 1, wd) \
        + B(U_MOUTH, U_F1 + 1.0, -50, 50, 0, 200)
    s = shell ^ keep
    rib_top = n["w_rroot"] + RACK_BACK_MZ
    yo = n["y_lip0"] - 0.5
    # mouth bulkhead under the ring onto the rib (u 190..193), open in the middle for the line and the rack spine
    s = s + (B(U_MOUTH, U_MOUTH + 3.0, -yo, yo, rib_top - 0.5, wd - 15.0) - inner - B(0, 400, -5.0, 5.0, 0, 300))
    for sg in (1, -1):                                               # posts onto the u 160 rib under each rail
        y0, y1 = sorted((sg * 7.0, sg * 13.0))
        s = s + (B(160.0, 163.0, y0, y1, rib_top - 0.5, wd - 12.0) - inner)
    return s


def fork_solid(P=None, n=None):
    """Thin cheek plates (|y| 3..Y_CHEEK1) from the u 250 deck rib to the roller hub, a floor tying them behind the
    roller. No printed bosses (the roller must drop into the slot): two loose printed thrust washers on the
    axle take the bearing inner race."""
    n = _n(P, n)
    rib_top = n["w_rroot"] + RACK_BACK_MZ
    s = []
    s.append(Bs(U_FORK0, U_R, Y_CHEEK0, Y_CHEEK1, n["w_deck0"], W_FORK1))                # cheek plates
    s.append(prim.cylY(R_HUB, Y_CHEEK0, Y_CHEEK1, x=U_R, z=W_R, seg=48)
             + prim.cylY(R_HUB, -Y_CHEEK1, -Y_CHEEK0, x=U_R, z=W_R, seg=48))             # hubs (rounded front)
    s.append(Bs(U_R - 6.0, U_R, Y_CHEEK0, Y_CHEEK1, W_R - R_HUB, n["w_deck0"] + 0.01))  # hub roots under the deck
    s.append(B(n["c_len"] - 0.5, U_R - R_PUL_CLEAR, -Y_CHEEK1, Y_CHEEK1, n["w_deck0"], rib_top - 1.5))   # floor
    f = prim.union(s)
    # lightening windows through the cheek plates between the ribs
    f = f - Bs(U_FORK0 + 4.0, n["c_len"] - 4.0, Y_CHEEK0 - 1.0, Y_CHEEK1 + 1.0, rib_top + 1.0, W_FORK1 - 1.5)
    f = f - Bs(n["c_len"] + 1.5, U_R - 9.0, Y_CHEEK0 - 1.0, Y_CHEEK1 + 1.0, rib_top, W_FORK1 - 1.5)
    f = f - prim.cylY(1.7, -20.0, 20.0, x=U_R, z=W_R)
    return f


def muzzle_carriage_add(P=None, n=None):
    """Solid (carriage local) to UNION into carriage_solid: roller fork + shoe extension + retrieval funnel. The axle
    clearance hole is already cut (nothing else of the carriage is there)."""
    n = _n(P, n)
    s = fork_solid(P, n) + funnel_solid(P, n)
    return s ^ B(-100, 400, -n["y_shoe1"], n["y_shoe1"], n["w_rtip"], 300)


def roller_top(n=None):
    """Carriage-local point where the line leaves the roller groove going rearward (top of the line path)."""
    return (U_R, 0.0, W_R + LN.R_PATH)


def muzzle_parts(P=None, n=None, bed=None):
    """Roller, bearing, axle. joint launch_carriage, place = T(u_c_home) @ local pose."""
    n = _n(P, n)
    if bed is None:
        import launch as LA
        bed = LA.bed
    H = tf.T(n["u_c_home"], 0.0, 0.0)
    Mr = tf.mul(H, tf.T(U_R, 0.0, W_R), tf.Rx(-90))            # part +z -> +y
    ps = LN.pulley_solid()
    out = [Part("launch_muzzle_roller", ps, "PLA", joint="launch_carriage", place=Mr, print_tf=bed(ps),
                color="#e0c060", notes="Lip roller at the carriage front: the tether bends down over it in tow and "
                                       "retrieval (groove r 6 for the 1 mm line), 693ZZ inside"),
           Part("launch_muzzle_roller_brg", LN.bearing_solid(), "bought", joint="launch_carriage", place=Mr,
                color="#777c84", bought_ref="693ZZ 3x8x4", notes="Lip roller bearing")]
    Ma = tf.mul(H, tf.T(U_R, -Y_CHEEK1, W_R), tf.Rx(-90))      # head on the right cheek outer face, shank along +y
    out.append(Part("launch_muzzle_roller_axle", LN.axle_solid(AXLE_L), "bought", joint="launch_carriage", place=Ma,
                    color="#444444", bought_ref="M3x16 SHCS; M3 nyloc nut", notes="Lip roller axle through both cheeks"))
    ro, ri, y0, y1 = WASHER
    rf, tfl = WASHER_FLANGE
    ws = prim.tube(ro, ri, y1 - y0) + prim.tube(rf, ri, tfl, z=(y1 - y0) - tfl)   # z 0 = inner race side
    for sg, side in ((1, "l"), (-1, "r")):
        Mw = tf.mul(H, tf.T(U_R, sg * y0, W_R), tf.Rx(-90 if sg > 0 else 90))  # z along +-y from the race face
        out.append(Part(f"launch_muzzle_washer_{side}", ws, "PLA", joint="launch_carriage", place=Mw,
                        print_tf=bed(ws, tf.Rx(180)), color="#e0c060",
                        notes="Lip roller thrust washer (0.9 thick, flange on the cheek side) between the bearing inner race and the cheek: "
                              "put both in the roller bore, then drop roller and washers into the fork slot"))
    return out


def muzzle_allowed():
    return [Allow("launch_muzzle_roller_brg", "launch_muzzle_roller", "693ZZ pressed in the roller"),
            Allow("launch_muzzle_roller_axle", "launch_muzzle_roller_brg", "M3 axle through the bearing bore"),
            Allow("launch_muzzle_roller_axle", "launch_muzzle_washer_*", "M3 axle through the washer"),
            Allow("launch_line_*", "launch_muzzle_roller", "line in the groove")]


def muzzle_numbers(n=None):
    gear_cd, gear_tip = 82.6, 52.5
    return dict(roller_centre_local=(U_R, 0.0, W_R), roller_top_w=W_R + LN.R_PUL, roller_bottom_w=W_R - LN.R_PUL,
                fork_top_w=W_FORK1, carriage_front_u=U_FRONT, funnel=(U_F0, U_F1, R_F0, R_F1),
                gear_tip=(gear_cd, gear_tip))
