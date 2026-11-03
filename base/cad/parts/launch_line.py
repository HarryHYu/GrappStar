"""Tether path on the rail (DECISIONS D14), worker cad-launch.

The tether leaves the dart tail through the pusher hole on the rail axis, runs straight back above everything to a
TOP pulley on a fork behind the rail, down the back to a BOTTOM pulley, then forward and sideways under the rail ties to
a PIVOT pulley just under the main axis on the left (inboard) cheek. Length from the pivot pulley to the frame barely
changes with pitch (the pulley is 24 mm from the axis). cad-reel takes the line from the pivot pulley tangent point.
All pulleys: printed PLA, 693ZZ bearing (3 x 8 x 4, Harry has them), M3 screw axle with nut.

Rail frame (u, y, w) as in launch.py.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from cad.lib import Part, Joint, Allow, prim, tf   # noqa: E402

R_PUL = 7.0          # pulley outer radius
R_GROOVE = 6.0       # groove bottom
LINE_D = 1.0
R_PATH = R_GROOVE + LINE_D / 2
PUL_W = 5.0
N_A = 9              # telescoping segments of the pusher -> top pulley run


def geom(n):
    """Pulley centres and line points from the launch.L(P) dict."""
    w_line = n["w_dart"] - 0.4                       # tail hole centre height (dart rests 0.4 low on the cradle)
    u_t = n["u_track0"] - 22.6                        # top / bottom pulley u (behind the cocked SW_SEATED body)
    top = (u_t, 0.0, w_line - R_PATH)
    bot = (u_t, 0.0, 30.0)
    piv = (0.0, -21.0, -53.5)            # the reel feed (about w -60 ahead of it) wraps under it, round the rear
    return dict(w_line=w_line, top=top, bot=bot, piv=piv,
                p_top_front=(u_t, 0.0, w_line), p_top_rear=(u_t - R_PATH, 0.0, top[2]),
                p_bot_rear=(u_t - R_PATH, 0.0, bot[2]), p_bot_low=(u_t, 0.0, bot[2] - R_PATH),
                p_piv_top=(piv[0], piv[1], piv[2] + R_PATH), p_piv_tan=_piv_tangent(piv, bot))


def _piv_tangent(piv, bot):
    """Point where the line leaves the pivot pulley (bottom-rear) towards the bottom pulley's lowest path point."""
    qx, qz = bot[0], bot[2] - R_PATH
    dx, dz = qx - piv[0], qz - piv[2]
    d = math.hypot(dx, dz)
    a = math.atan2(dz, dx) + math.acos(min(1.0, R_PATH / d))
    return (piv[0] + R_PATH * math.cos(a), piv[1], piv[2] + R_PATH * math.sin(a))


def rail_add(n):
    """Solids to union into the rail (fork behind the rear end, lug under the pivot on the left cheek)."""
    g = geom(n)
    u_t = g["top"][0]
    u0 = n["u_track0"]
    s = []
    # cross plate behind the rear tie, then the fork plates
    s.append(prim.box(u_t + R_PUL + 1.5, u0 + 0.01, -n["y_wall1"], n["y_wall1"], n["w_floor0"] - 6.0, n["w_floor0"] + 4.0))
    for sg in (1, -1):
        y0, y1 = (PUL_W / 2 + 1.0, PUL_W / 2 + 5.0) if sg > 0 else (-PUL_W / 2 - 5.0, -PUL_W / 2 - 1.0)
        s.append(prim.box(u_t - 10.0, u0 - 1.0, y0, y1, g["bot"][2] - 10.0, g["top"][2] + 9.0))
    # pivot pulley lug on the left cheek (y -17 .. -9), down to below the pulley centre
    pv = g["piv"]
    s.append(prim.box(-9.0, 9.0, -17.0, -9.0, pv[2] - 8.0, 0.0))
    return prim.union(s)


def rail_cut(n):
    g = geom(n)
    c = []
    for p in (g["top"], g["bot"]):
        c.append(prim.cylY(1.7, -PUL_W / 2 - 6.0, PUL_W / 2 + 6.0, x=p[0], z=p[2]))
    pv = g["piv"]
    c.append(prim.cylY(1.7, -30, 0, x=pv[0], z=pv[2]))
    return prim.union(c)


def pulley_solid():
    p = prim.cyl(R_PUL, PUL_W, z=-PUL_W / 2)
    groove = prim.revolve_pts([(R_GROOVE, -0.7), (R_PUL + 1, -1.8), (R_PUL + 1, 1.8), (R_GROOVE, 0.7)])
    p = p - groove
    return p - prim.cyl(4.0 - 0.05, PUL_W + 2, z=-PUL_W / 2 - 1)


def bearing_solid():
    return prim.tube(4.0, 1.5, 4.0, z=-2.0)


def axle_solid(length):
    """M3 screw along z from z = 0 (under the head) to z = length, head below z = 0."""
    return prim.cyl(1.5, length) + prim.cyl(2.75, 3.0, z=-3.0)


def seg(p0, p1, r=LINE_D / 2):
    """Line segment between two points (rail frame) as a thin cylinder."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
        # noqa
    L = float(np.linalg.norm(d))
    c = prim.cyl(r, L, seg=12)
    z = np.array([0.0, 0.0, 1.0])
    v = d / L
    ax = np.cross(z, v)
    s = np.linalg.norm(ax)
    if s < 1e-9:
        M = tf.I4 if v[2] > 0 else tf.Rx(180)
    else:
        M = tf.R(ax / s, math.degrees(math.atan2(s, float(z @ v))))
    return prim.place(c, tf.mul(tf.T(*p0), M))


def parts_line(n, bed):
    g = geom(n)
    out = []
    Ry = tf.Rx(-90)                                  # part +z -> rail +y
    for nm, p, yc in (("top", g["top"], 0.0), ("bot", g["bot"], 0.0), ("piv", g["piv"], g["piv"][1])):
        Mp = tf.mul(tf.T(p[0], yc, p[2]), Ry)
        ps = pulley_solid()
        out.append(Part(f"launch_pulley_{nm}", ps, "PLA", joint=f"launch_pulley_{nm}", place=Ry, print_tf=bed(ps),
                        color="#e0c060", notes=f"Tether pulley ({nm}), groove r {R_GROOVE:g}, 693ZZ inside"))
        out.append(Part(f"launch_pulley_{nm}_brg", bearing_solid(), "bought", joint=f"launch_pulley_{nm}", place=Ry,
                        color="#777c84", bought_ref="693ZZ 3x8x4", notes="Pulley bearing"))
        if nm == "piv":
            ax = tf.mul(tf.T(p[0], -9.0, p[2]), tf.Rx(90))         # head on the cheek inner face, along -y
            out.append(Part("launch_pulley_piv_axle", axle_solid(17.0), "bought", joint="launch_rail", place=ax,
                            color="#444444", bought_ref="M3x16 SHCS + nut", notes="Pivot pulley axle"))
        else:
            ax = tf.mul(tf.T(p[0], -PUL_W / 2 - 5.0, p[2]), tf.Rx(-90))
            out.append(Part(f"launch_pulley_{nm}_axle", axle_solid(2 * (PUL_W / 2 + 5.0)), "bought", joint="launch_rail",
                            place=ax, color="#444444", bought_ref="M3x16 SHCS + nut", notes="Pulley axle"))
    # line: pusher -> top pulley, telescoped: N_A segments of the cocked length, spread evenly between the pulley
    # tangent and the pusher (segment k slides back by k/(N_A-1) of the carriage travel), so none passes either end
    L_home = n["u_c_home"] - g["p_top_front"][0]
    ls = L_home - n["stroke"] - 0.05
    for k in range(N_A):
        a0 = g["p_top_front"][0] + k * (L_home - ls) / (N_A - 1)
        out.append(Part(f"launch_line_a{k}", seg((a0, 0.0, g["w_line"]), (a0 + ls, 0.0, g["w_line"])), "bought",
                        joint=f"launch_line_a{k}", color="#f0f0f0",
                        bought_ref="Dyneema braid 1 mm (LINE)", notes="Tether, pusher to top pulley (telescoped in the model)"))
    out.append(Part("launch_line_p", seg((n["u_c_home"] - 0.01, 0.0, g["w_line"]), (n["u_tail_home"], 0.0, g["w_line"])),
                    "bought", joint="launch_carriage", color="#f0f0f0", notes="Tether through the pusher hole to the tail"))
    out.append(Part("launch_line_b", seg(g["p_top_rear"], g["p_bot_rear"]), "bought", joint="launch_rail", color="#f0f0f0",
                    notes="Tether down the back of the fork"))
    out.append(Part("launch_line_c", seg(g["p_bot_low"], g["p_piv_tan"]), "bought", joint="launch_rail", color="#f0f0f0",
                    notes="Tether under the rail to the pivot pulley (wraps under it to cad-reel's feed)"))
    return out


def joints_line(n):
    g = geom(n)
    out = []
    for nm, p, yc in (("top", g["top"], 0.0), ("bot", g["bot"], 0.0), ("piv", g["piv"], g["piv"][1])):
        out.append(Joint(f"launch_pulley_{nm}", "launch_rail", "rev", (p[0], yc, p[2]), (0, 1, 0)))
    for k in range(N_A):
        out.append(Joint(f"launch_line_a{k}", "launch_rail", "slide", (0, 0, 0), (-1, 0, 0),
                         gear=("launch_carriage", k / (N_A - 1), 0.0)))
    return out


def allowed_line():
    return [Allow("launch_line_a*", "launch_line_a*", "telescoped model of one straight line"),
            Allow("launch_pulley_*_brg", "launch_pulley_*", "693ZZ pressed in the pulley"),
            Allow("launch_pulley_*_axle", "launch_pulley_*_brg", "M3 axle through the bearing bore"),
            Allow("launch_line_*", "launch_pulley_*", "line in the groove"),
            Allow("launch_line_p", "launch_carriage", "line through the pusher hole"),
            Allow("launch_line_p", "launch_dart", "line enters the tail cap hole"),
            Allow("launch_line_a*", "launch_line_p", "same line")]
