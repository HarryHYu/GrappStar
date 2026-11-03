"""Printed spring guards (DECISIONS D33), worker cad-launch. Rail frame (u, y, w), joint launch_rail.

Each side: a 2 mm PLA side plate at |y| 66..68 (outside the main spring at |y| 38 and the D5 provision spring at
|y| 56, d 16: outer face 64), in three printable segments:
  a (rear)  u -177.5 .. 72     w 37 .. 83   (behind the stop units; the carriage arms sweep inside, |y| <= 61)
  b (mid)   u 72.2 .. 239.6     w 37 .. 83 to u 78, then w 37 .. 69 (the stop spring covers the springs from above
                               there, w >= 69.5; the stop frame front legs at u 240.2..248.2 close the side)
  c (front) u 248.7 .. 271.5   w 37 .. 83   (hooks on the anchor pins)
Inward top lips (|y| 62.5..66) stiffen the plates. Brackets under the springs (w < 48, clear of the arm peg keepers
at w 49) bolt to the rail wall outer faces (|y| 28) with M3 x 12 self tap (pilots d2.6 at w 42, along y), a foot on
the front plate top (w 40, M3 x 10 down, pilot d2.6) and two bosses onto the anchor post outer faces (|y| 64, M3 x 12
along y). Hex key access holes d6.4 in the plates in front of the wall bolts. Fit the guards before the springs.
Bracket positions avoid the aim lug (+y, u 100..130), the rail split flanges (u -30, 140), the stop frame rear feet
(u 81.6..93.6) and, on the right, the sleeve flange / dog ring zone near the main axis.

Integration (launch.py):
    import launch_guard as GD
    rail_solid():  solid = solid - GD.guard_rail_cut(n)        # with the other pilots
    parts():       out += GD.guard_parts(P, n)
    allowed():     ... + GD.guard_allowed()   (empty: all face contacts)
"""
from __future__ import annotations

import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from cad.lib import Part, prim, tf   # noqa: E402

Y_IN, Y_OUT = 66.0, 68.0            # plate faces |y|
W_BOT = 37.0                        # plate bottom
W_TOP = 83.0                        # plate top outside the stop zone (arms reach 83 at |y| <= 61)
W_TOP_STOP = 69.0                   # plate top under the stop spring (stack bottom 69.5)
LIP_Y = 62.5                        # inward top lip inner edge (arms |y| <= 61)
LIP_T = 2.0
U_A = (-177.5, 72.0)
U_B = (72.2, 239.6)
U_B_TALL = 78.0                     # stop spring pad starts at u 78.6
U_C = (248.7, 271.5)
W_BOLT = 42.0                       # wall bolt height
Y_WALL = 28.0                       # rail wall outer face
# wall brackets (u centre) per segment and side (+1 left / -1 right)
WALL_BRACKETS = {
    "a": {1: (-160.0, -105.0, -60.0, 50.0), -1: (-160.0, -105.0, -60.0, 50.0)},
    "b": {1: (76.0, 165.0), -1: (76.0, 115.0, 165.0)},
}
TAB_HALF = 5.0                      # bracket half length along u
FOOT_U, FOOT_Y = 215.0, 47.0        # seg b foot on the rail front plate (w 40): M3 down
POST_BOLTS_U = (258.0, 269.0)       # seg c bolts into the anchor post outer face (|y| 64) at w 45
W_POST_BOLT = 45.0


def _bx(u0, u1, ya, yb, w0, w1, sg):
    y0, y1 = sorted((sg * ya, sg * yb))
    return prim.box(u0, u1, y0, y1, w0, w1)


def _cyl_y(r, ya, yb, u, w, sg):
    y0, y1 = sorted((sg * ya, sg * yb))
    return prim.cylY(r, y0, y1, x=u, z=w)


def _bracket(uc, sg):
    """Wall bracket: tab on the wall face (|y| 28..32, w 36.5..47.5), 45 deg gussets onto a web (w 40..44) out to the
    plate. Profile in (|y|, w), extruded along u."""
    pts = [(Y_WALL, 36.5), (Y_WALL + 4.0, 36.5), (Y_WALL + 7.5, 40.0), (Y_IN + 0.01, 40.0), (Y_IN + 0.01, 44.0),
           (Y_WALL + 7.5, 44.0), (Y_WALL + 4.0, 47.5), (Y_WALL, 47.5)]
    cs = prim.poly_cs([(sg * y, w) for (y, w) in pts])     # profile x = signed y, profile y = w
    s = prim.extrude(cs, 2 * TAB_HALF)                     # z = 0 .. 2 * TAB_HALF
    M = np.array([[0.0, 0.0, 1.0, uc - TAB_HALF], [1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    s = prim.place(s, M)                                   # (x, y, z) -> (u = z, y = x, w = y)
    return s


def segment_solid(seg, sg):
    if seg == "a":
        u0, u1 = U_A
        s = [_bx(u0, u1, Y_IN, Y_OUT, W_BOT, W_TOP, sg), _bx(u0, u1, LIP_Y, Y_IN + 0.01, W_TOP - LIP_T, W_TOP, sg)]
    elif seg == "b":
        u0, u1 = U_B
        s = [_bx(u0, u1, Y_IN, Y_OUT, W_BOT, W_TOP_STOP, sg), _bx(u0, U_B_TALL, Y_IN, Y_OUT, W_BOT, W_TOP, sg),
             _bx(u0, U_B_TALL, LIP_Y, Y_IN + 0.01, W_TOP - LIP_T, W_TOP, sg),
             _bx(U_B_TALL - 0.01, u1, LIP_Y, Y_IN + 0.01, W_TOP_STOP - LIP_T, W_TOP_STOP, sg),
             _bx(FOOT_U - 6.0, FOOT_U + 6.0, FOOT_Y - 4.0, Y_IN + 0.01, 40.0, 43.5, sg)]          # foot on the plate
    else:
        u0, u1 = U_C
        s = [_bx(u0, u1, Y_IN, Y_OUT, W_BOT, W_TOP, sg), _bx(u0, u1, LIP_Y, Y_IN + 0.01, W_TOP - LIP_T, W_TOP, sg)]
        for uu in POST_BOLTS_U:
            s.append(_cyl_y(4.0, 64.0, Y_IN + 0.01, uu, W_POST_BOLT, sg))                          # bosses
    for uc in WALL_BRACKETS.get(seg, {}).get(sg, ()):
        s.append(_bracket(uc, sg))
    g = prim.union(s)
    for uc in WALL_BRACKETS.get(seg, {}).get(sg, ()):
        g = g - _cyl_y(1.7, Y_WALL - 1.0, Y_WALL + 5.0, uc, W_BOLT, sg)                        # M3 clearance
        g = g - _cyl_y(3.2, Y_IN - 1.0, Y_OUT + 1.0, uc, W_BOLT, sg)                           # hex key access
    if seg == "b":
        g = g - prim.cyl(1.7, 6.0, x=FOOT_U, y=sg * FOOT_Y, z=39.0)
    if seg == "c":
        for uu in POST_BOLTS_U:
            g = g - _cyl_y(1.7, 63.0, Y_OUT + 1.0, uu, W_POST_BOLT, sg)
    return g


def guard_rail_cut(n=None):
    """Cut from rail_solid: M3 self tap pilots (d2.6) for the guard brackets. Wall pilots along y from the outer face
    (|y| 28) 10 deep into wall and shelf; foot pilots down into the front plate; post pilots into the anchor posts
    (stopping short of the D5 anchor hole at |y| 56)."""
    c = []
    for seg, d in WALL_BRACKETS.items():
        for sg, us in d.items():
            for uc in us:
                c.append(_cyl_y(1.3, 18.0, Y_WALL + 0.5, uc, W_BOLT, sg))
    for sg in (1, -1):
        c.append(prim.cyl(1.3, 9.5, x=FOOT_U, y=sg * FOOT_Y, z=31.0))
        for uu in POST_BOLTS_U:
            c.append(_cyl_y(1.3, 60.0, 64.5, uu, W_POST_BOLT, sg))
    return prim.union(c)


def pilot_positions():
    """(u, y, w, axis) of every rail pilot (for the NOTES / assembly text)."""
    out = []
    for seg, d in WALL_BRACKETS.items():
        for sg, us in d.items():
            out += [(uc, sg * Y_WALL, W_BOLT, "y") for uc in us]
    for sg in (1, -1):
        out.append((FOOT_U, sg * FOOT_Y, 40.0, "w"))
        out += [(uu, sg * 64.0, W_POST_BOLT, "y") for uu in POST_BOLTS_U]
    return out


def guard_parts(P=None, n=None, bed=None):
    if bed is None:
        import launch as LA
        bed = LA.bed
    out = []
    names = dict(a="rear", b="mid", c="front")
    for sg, side in ((1, "l"), (-1, "r")):
        for seg in ("a", "b", "c"):
            s = segment_solid(seg, sg)
            R = tf.Rx(-90) if sg > 0 else tf.Rx(90)          # outer face (|y| 68) down on the bed
            nb = len(WALL_BRACKETS.get(seg, {}).get(sg, ()))
            fix = {"a": f"{nb} brackets bolted to the rail wall (M3x12 self tap)",
                   "b": f"{nb} wall brackets (M3x12 self tap) and a foot on the front plate (M3x10 down)",
                   "c": "2 bosses on the anchor post (M3x12 self tap)"}[seg]
            out.append(Part(f"launch_guard_{side}_{seg}", s, "PLA", joint="launch_rail", place=tf.I4,
                            print_tf=bed(s, R), color="#b8c4cc",
                            bought_ref=f"{nb + (1 if seg == 'b' else 0) + (2 if seg == 'c' else 0)} x M3x12 self tap",
                            notes=f"Spring guard {'left' if sg > 0 else 'right'} {names[seg]} segment (D33): 2 mm side "
                                  f"plate outside both spring pairs, {fix}"))
    return out


def guard_allowed():
    return []
