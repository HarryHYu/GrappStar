"""Printed contact switch (no microswitch): a flex leaf carries an M3 screw whose head touches a captured M3 nut.

Local frame (all in mm):
- base plate z 0..BASE_T on the mounting surface, x from x0 to x1, y -W/2..W/2, two M3 holes (self tap into the
  mount: 2.6 pilot in the mount, 3.4 clear in the base);
- leaf: thin plate x 0..LEAF_T, rising from the base to z_top; the M3 screw is threaded through it along x at z_s;
- head on the +x side, nut captured in a post at +x; the actuator pushes the screw TIP (on the -x side) towards +x;
- modelled PRESSED (head face on the nut face). Free, the leaf springs back TRAVEL towards -x.
Wiring: one wire under the nut (nut trap has a slot for a stripped wire end, clamped by the nut), one wire on the
screw (ring terminal or wrap under a second nut on the screw tip side is not needed: wrap the wire round the leaf screw
before tightening). ESP32 pin with internal pull-up, firmware debounce.
"""
from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from cad.lib import prim  # noqa: E402

BASE_T = 4.0
LEAF_T = 1.2
W = 10.0
HEAD_D, HEAD_H = 5.5, 3.0
NUT_AF, NUT_T = 5.5, 2.4
TRAVEL = 1.2


def info(screw_len=6.0):
    return dict(tip_x=LEAF_T - screw_len - 0.0, head_x0=LEAF_T, head_x1=LEAF_T + HEAD_H, nut_x0=LEAF_T + HEAD_H)


def body(z_s=9.0, screw_len=6.0, x0=-3.0, holes=True, width=None, top=5.0):
    """Base + leaf + nut post (one printed PLA part). Returns the solid."""
    z_top = z_s + top
    Wd = width or W
    xn = LEAF_T + HEAD_H                     # nut face
    x1 = xn + NUT_T + 6.0 + 14.0             # post + room for two mounting holes
    base = prim.box(x0, x1, -Wd / 2, Wd / 2, 0, BASE_T)
    leaf = prim.box(0, LEAF_T, -Wd / 2 + 1, Wd / 2 - 1, BASE_T - 0.01, z_top)
    post = prim.box(xn, xn + NUT_T + 6.0, -Wd / 2, Wd / 2, BASE_T - 0.01, z_top)
    s = base + leaf + post
    # screw tap hole in the leaf, clearance in the post behind the nut, nut trap open at the top (drop the nut in)
    s = s - prim.cylX(1.3, -1, LEAF_T + 1, y=0, z=z_s)
    s = s - prim.cylX(1.8, xn - 0.5, xn + NUT_T + 7.0, y=0, z=z_s)
    hexr = NUT_AF / 2 / 0.8660 + 0.15
    nut_cut = prim.place(prim.cyl(hexr, NUT_T + 0.3, seg=6), _rot_y90())
    nut_cut = nut_cut.translate((xn, 0, z_s))
    s = s - nut_cut - prim.box(xn, xn + NUT_T + 0.3, -NUT_AF / 2 - 0.15, NUT_AF / 2 + 0.15, z_s, z_top + 1)
    # wire slot beside the nut
    s = s - prim.box(xn - 0.01, xn + NUT_T + 0.3, -1.0, 1.0, z_s - hexr - 2.0, z_s)
    if holes:
        for hx in mount_holes(x0, z_s):
            s = s - prim.cyl(1.7, BASE_T + 2, x=hx, z=-1)
    return s


def _rot_y90():
    import numpy as np
    from cad.lib import tf
    return tf.Ry(90)


def screw(screw_len=6.0, z_s=9.0):
    """M3 SHCS, head on +x, pressed position."""
    h = prim.cylX(HEAD_D / 2, LEAF_T, LEAF_T + HEAD_H, y=0, z=z_s)
    sh = prim.cylX(1.5, LEAF_T - screw_len, LEAF_T + 0.01, y=0, z=z_s)
    return h + sh


def nut(z_s=9.0):
    xn = LEAF_T + HEAD_H
    hexr = NUT_AF / 2 / 0.8660
    n = prim.place(prim.cyl(hexr, NUT_T, seg=6), _rot_y90()).translate((xn, 0, z_s))
    return n - prim.cylX(1.5, xn - 1, xn + NUT_T + 1, y=0, z=z_s)


def mount_holes(x0=-3.0, z_s=9.0):
    """x positions (local) of the base mounting holes."""
    xn = LEAF_T + HEAD_H
    x1 = xn + NUT_T + 6.0 + 14.0
    return [x1 - 3.5, x1 - 10.0]
