"""Lip roller and retrieval funnel at the carriage front (BASE_SPEC 8), worker cad-launch.

Carriage LOCAL frame throughout (rail frame minus u_c_home in u): u from the carriage rear face (muzzle +), y left,
w up. The carriage moves -u by the cock travel (0..241) and +u by up to 19 into the stop.

LIP ROLLER: the tether runs from the pusher hole (u 0..8, y 0, w 84.6) forward to the dart. In tow and retrieval it
bends down over the carriage front, so a printed pulley (launch_line pulley, 693ZZ, M3 axle along y) sits in a fork
on the carriage front: centre (U_R, 0, W_R), top W_R + 7 = 62.3 (below the dart belly 63.6 as the dart flies off),
bottom 48.3 (nothing of the carriage below the rack tips 47.6). The fork prongs (|y| 3..17.5, w 53.7..62.5) run back
over the deck to the rack end, the shoes run on to the fork. At full cock the fork passes over the big gear (tip
circle r 52.5 about rail (82.6, 0)): the parts below the deck sit far enough forward to clear it.

RETRIEVAL FUNNEL: the dart is reeled home tail first; a cone concentric with the dart axis (w 85) opens from the
cradle bore r 21.4 at u 148 to r 27 at u 200 (floor 63.6 -> 58, just above the rack spine 57.7), cut at w 80 and to
the carriage body |y| <= 21.5, clear of the rail lips (|y| > 17.5 at w 59.7..64.5). It catches a tail cap (d 42)
arriving low or off centre and steers it up and onto the cradle centreline. Being outside r 21.4 about w 85 it never
touches the dart in flight (hull r 21 about w 84.6).

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
Y_CHEEK1 = 8.5              # cheek outer faces at the hub (axle head / nut seat)
R_HUB = 5.0                 # cheek hub radius about the axle (inside the groove bottom r 6: the line never touches)
AXLE_L = 20.0               # M3 x 20 SHCS + nyloc nut
U_FORK0 = 238.0             # fork root (rack end)
W_FORK1 = 62.5              # fork top (dart belly 63.6 in flight)
U_FRONT = U_R + R_HUB       # carriage front (286): the body prints across the bed diagonal (L + W <= 362)

# funnel
U_F0, U_F1 = 148.0, 200.0
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


def funnel_solid(P=None, n=None):
    n = _n(P, n)
    wd = n["w_dart"]
    k = (R_F1 - R_F0) / (U_F1 - U_F0)
    inner = _cone_u(R_F0 - k * 1.0, R_F1 + k * 1.0, U_F0 - 1.0, U_F1 + 1.0, wd)
    outer = _cone_u(R_F0 + F_WALL, R_F1 + F_WALL, U_F0, U_F1, wd)
    env = B(U_F0, U_F1, -n["y_shoe1"], n["y_shoe1"], n["w_deck0"], W_F_TOP) - _lip_keepout(n)
    shell = (outer - inner) ^ env
    # keel onto the rack spine, webs on the deck cross ribs (u 160, 190) up to the cone
    keel = B(U_F0 + 10.0, U_F1, -1.5, 1.5, n["w_rroot"] + 3.5, wd - R_F0) - inner
    webs = prim.union([B(uc, uc + 3.0, -(n["y_lip0"] - 0.5), n["y_lip0"] - 0.5, n["w_rroot"] + 3.5, wd - 12.0)
                       for uc in (160.0, 190.0)]) - inner - _lip_keepout(n)
    return shell + keel + webs


def fork_solid(P=None, n=None):
    n = _n(P, n)
    yo = n["y_lip0"] - 0.5                                       # 17.5: inside the lips
    s = []
    s.append(Bs(U_FORK0, U_R, Y_CHEEK0, yo, n["w_deck0"], W_FORK1))                      # prongs
    s.append(Bs(n["c_len"] - 1.0, U_R, yo - 0.01, n["y_shoe1"], n["shoe_w0"], n["shoe_w1"]))   # shoes run on
    s.append(prim.cylY(R_HUB, Y_CHEEK0, Y_CHEEK1, x=U_R, z=W_R, seg=48)
             + prim.cylY(R_HUB, -Y_CHEEK1, -Y_CHEEK0, x=U_R, z=W_R, seg=48))             # hubs (rounded front)
    s.append(Bs(U_R - 6.0, U_R, Y_CHEEK0, Y_CHEEK1, W_R - R_HUB, n["w_deck0"] + 0.01))  # hub roots under the deck
    # bosses that clamp the bearing inner race (693ZZ 4 wide), clear of the pulley bore (r 3.95)
    s.append(prim.cylY(2.9, 2.05, Y_CHEEK0 + 0.01, x=U_R, z=W_R) + prim.cylY(2.9, -Y_CHEEK0 - 0.01, -2.05, x=U_R, z=W_R))
    f = prim.union(s)
    # lightening pockets in the prongs (open at the top), axle clearance hole
    f = f - Bs(U_FORK0 + 4.0, U_R - 9.0, 9.5, 16.0, n["w_deck0"] + 1.8, W_FORK1 + 1.0)
    f = f - prim.cylY(1.7, -20.0, 20.0, x=U_R, z=W_R)
    # counterbores for the screw head (right) and the nyloc nut (left), open through the shoes: fitted from the side
    f = f - prim.cylY(3.6, Y_CHEEK1, 30.0, x=U_R, z=W_R) - prim.cylY(3.6, -30.0, -Y_CHEEK1, x=U_R, z=W_R)
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
                    color="#444444", bought_ref="M3x20 SHCS; M3 nyloc nut", notes="Lip roller axle through both cheeks"))
    return out


def muzzle_allowed():
    return [Allow("launch_muzzle_roller_brg", "launch_muzzle_roller", "693ZZ pressed in the roller"),
            Allow("launch_muzzle_roller_axle", "launch_muzzle_roller_brg", "M3 axle through the bearing bore"),
            Allow("launch_line_*", "launch_muzzle_roller", "line in the groove")]


def muzzle_numbers(n=None):
    gear_cd, gear_tip = 82.6, 52.5
    return dict(roller_centre_local=(U_R, 0.0, W_R), roller_top_w=W_R + LN.R_PUL, roller_bottom_w=W_R - LN.R_PUL,
                fork_top_w=W_FORK1, carriage_front_u=U_FRONT, funnel=(U_F0, U_F1, R_F0, R_F1),
                gear_tip=(gear_cd, gear_tip))
