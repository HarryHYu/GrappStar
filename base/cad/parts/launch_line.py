"""Tether on the rail (DECISIONS D49), worker cad-launch.

D49 rail reel: the spool, the capstan and the cup ride the rail behind its rear end. The tether runs in one straight
line on the dart axis (w 84.6) from cad-reel's cup eyelet (front mouth at u -183) forward through the pusher hole to
the dart's tail cap. The rear tether pulleys, their fork and the pivot pulley of D14/D34 are gone; the rail rear end
carries the flange the reel head bolts to (u -181.2, 4 M3 heat-set inserts in the wall ends). The lip roller on the
carriage (launch_muzzle.py) still uses the pulley helpers below.

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
N_A = 21             # telescoping segments of the eyelet -> pusher run (each <= 12.6, overlapping at HOME)


U_EYE = -183.0       # cad-reel cup eyelet front mouth (params RAIL_REEL line eyelet_u)
U_FLANGE = -181.2    # rear face of the rail flange the reel head bolts to
FLANGE_W = (24.0, 58.0)                        # below the cocked yoke arms (w 60 and up)
FLANGE_INSERTS = [(25.0, 36.0), (25.0, 52.0), (-25.0, 36.0), (-25.0, 52.0)]   # (y, w) rail frame, in the wall ends


def geom(n):
    """Line height and the eyelet point from the launch.L(P) dict."""
    w_line = n["w_dart"] - 0.4                       # tail hole centre height (dart rests 0.4 low on the cradle)
    return dict(w_line=w_line, eye=(U_EYE, 0.0, w_line))


def rail_add(n):
    """Rail rear flange (u -181.2 .. rail rear end): the reel head front plate bolts to it."""
    u0 = n["u_track0"]
    return prim.box(U_FLANGE, u0 + 0.5, -n["y_wall1"], n["y_wall1"], FLANGE_W[0], FLANGE_W[1])


def rail_cut(n):
    """M3 heat-set insert holes in the wall ends behind the flange face (d 4.0, 6 deep)."""
    return prim.union([prim.cylX(2.0, U_FLANGE - 1.0, U_FLANGE + 6.0, y=y, z=w) for y, w in FLANGE_INSERTS])


def pulley_solid():
    p = prim.cyl(R_PUL, PUL_W, z=-PUL_W / 2)
    groove = prim.revolve_pts([(R_GROOVE, -0.7), (R_PUL + 1, -1.8), (R_PUL + 1, 1.8), (R_GROOVE, 0.7)])
    p = p - groove
    p = p - prim.cyl(4.0 - 0.05, PUL_W + 2, z=-PUL_W / 2 - 1)
    # hub faces recessed 0.6 inside r 4.6 so bosses and the screw head bear on the bearing only (review 3)
    return p - prim.tube(4.6, 3.0, 0.6, z=PUL_W / 2 - 0.6) - prim.tube(4.6, 3.0, 0.6, z=-PUL_W / 2)


def bearing_solid():
    return prim.tube(4.0, 1.5, 4.0, z=-2.0)


def washer_solid():
    """Printed top-hat thrust washer for the fork pulleys: hub r 2.4 x 0.5 on the 693ZZ inner race, flange r 4.4 x 1.0
    on the fork plate (local z from 0 at the race to 1.5 at the plate)."""
    return prim.cyl(2.4, 0.51) + prim.cyl(4.4, 1.0, z=0.5) - prim.cyl(1.65, 3.0, z=-1.0)


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
    # line: eyelet -> pusher, telescoped: N_A segments of the cocked length, spread evenly between the eyelet and the
    # pusher (segment k slides back by k/(N_A-1) of the carriage travel), so none passes either end
    L_home = n["u_c_home"] - g["eye"][0]
    ls = L_home - n["stroke"] - 0.05
    for k in range(N_A):
        a0 = g["eye"][0] + k * (L_home - ls) / (N_A - 1)
        out.append(Part(f"launch_line_a{k}", seg((a0, 0.0, g["w_line"]), (a0 + ls, 0.0, g["w_line"])), "bought",
                        joint=f"launch_line_a{k}", color="#f0f0f0",
                        bought_ref="Dyneema braid 1 mm (LINE)", notes="Tether, cup eyelet to the pusher (telescoped in the model)"))
    out.append(Part("launch_line_p", seg((n["u_c_home"] - 0.01, 0.0, g["w_line"]), (n["u_tail_home"], 0.0, g["w_line"])),
                    "bought", joint="launch_carriage", color="#f0f0f0", notes="Tether through the pusher hole to the tail"))
    return out


def joints_line(n):
    out = []
    for k in range(N_A):
        out.append(Joint(f"launch_line_a{k}", "launch_rail", "slide", (0, 0, 0), (-1, 0, 0),
                         gear=("launch_carriage", k / (N_A - 1), 0.0)))
    return out


def allowed_line():
    return [Allow("launch_line_a*", "launch_line_a*", "telescoped model of one straight line"),
            Allow("launch_line_p", "launch_carriage", "line through the pusher hole"),
            Allow("launch_line_p", "launch_dart", "line enters the tail cap hole"),
            Allow("launch_line_a*", "launch_line_p", "same line")]
