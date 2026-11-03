"""Printed spring guards (DECISIONS D33), worker cad-launch. Rail frame (u, y, w), joint launch_rail.

Each side: a 2 mm PLA side plate at |y| 66..68 (outside the main spring at |y| 38 and the D5 provision spring at
|y| 56, d 16: outer face 64), in four printable segments:
  a  (rear)  u -180.9 .. 69.1   w 37 .. 85.5, rear END WALL (u -180.9..-178.9, |y| 28.5..66: behind the yoke at full
                                cock, clear of the tether fork |y| <= 7.5) and a TOP COVER (|y| 42..66, w 83.5..85.5:
                                above the yoke arms, top w 83, and outside the folded dart, |y| <= 40.8)
  b1 (mid)   u 69.3 .. 81.0     w 37 .. 85.5 with the top cover up to u 73.4 (the stop frame's outboard ledge / bar
                                close the side above w 69.5 from u 73.6), then w 37 .. 69
  b2 (mid)   u 94.3 .. 239.6    w 37 .. 69 (the stop spring covers the springs from above, w >= 69.5; the stop frame
                                rear leg closes the side at u 81.6..93.6 and the front legs at u 240.2..248.2)
  c  (front) u 248.7 .. 271.5   w 31 .. 85.5 with the top cover, and a low strip back to u 240.3 (w 31..39.5, under
                                the stop frame front foot) carrying the two bolts
What stays open: above w 83 the band |y| 28..42 over segments a and c (the folded dart's flank features sweep it,
|y| <= 40.8, on every shot) and the slot between the cover and the stop units (u 73.4..78.6) where the yoke arms
pass; the open band is 14 mm wide next to the rail wall (a d16 spring cannot pass it beside the rail, but next to the
carriage body, |y| 21.5, the open width is 20.5 mm).
Inward lips (|y| 62.5..66, w 67..69) on b1 / b2. Brackets under the springs (w <= 47.5, clear of the arm peg keepers
at w 49): a block from the rail wall outer face (|y| 28) to the plate (w 36.5..47.5) with a 4 mm tab and a d6 bore
on the bolt axis (w 42) out to a d6.4 hole in the plate, so the M3 x 12 head and the hex key pass from outside; pilots d2.6
along y, 10.5 deep. Segment b2: a foot on the front plate top (w 40, M3 x 10 down).
Segment c: two bosses (u 243.5 and 249, w 35) on the front plate's outer edge face (|y| 64, w 30..40), M3 x 12, pilots
9.5 deep, clear of the spring anchor holes (u 255 and 263.5).
Assembly: after the stop frames and the springs, every segment goes in by a straight move inward along y (b1 and b2
pass under the stop frame's outboard bar, w 69.5).

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
W_TOP = 85.5                        # plate top where the top cover is
W_TOP_STOP = 69.0                   # plate top under the stop spring (stack bottom 69.5)
COVER_Y = 42.0                      # top cover inner edge (folded dart |y| <= 40.8)
COVER_W = (83.5, 85.5)              # top cover (yoke arms top w 83)
LIP_Y = 62.5                        # inward lip inner edge (arms |y| <= 61)
LIP_T = 2.0
U_A = (-180.9, 69.1)
REAR_WALL_T = 2.0                   # end wall at the rear of segment a (yoke back face reaches u -178.8 at the limit)
U_B1 = (69.3, 81.0)
U_B1_TALL = 73.4                    # stop frame outboard ledge / bar start at u 73.6
U_B2 = (94.3, 239.6)                # behind the stop frame rear leg (u 81.6..93.6) and its bolt key paths
U_C = (248.7, 271.5)
U_C_STRIP = 240.3                   # low strip under the stop frame front foot (w >= 40)
W_C_BOT = 31.0
W_BOLT = 42.0                       # wall bolt height
Y_WALL = 28.0                       # rail wall outer face
# wall brackets: u centre or (u centre, half length), per segment and side (+1 left / -1 right)
WALL_BRACKETS = {
    "a": {1: (-160.0, -105.0, -60.0, 50.0), -1: (-160.0, -105.0, -60.0, 50.0)},
    "b1": {1: ((76.8, 4.2),), -1: ((76.8, 4.2),)},
    "b2": {1: (165.0,), -1: (115.0, 165.0)},
}
TAB_HALF = 5.0                      # bracket half length along u
BRACKET_W = (36.5, 47.5)            # bracket block height (peg keepers sweep at w >= 49)
FOOT_U, FOOT_Y = 215.0, 47.0        # seg b2 foot on the rail front plate (w 40): M3 x 10 down
EDGE_BOLTS_U = (243.5, 249.0)       # seg c bolts into the front plate outer edge face (|y| 64) at w 35
W_EDGE_BOLT = 35.0
Y_EDGE = 64.0
SCREWS = dict(wall="M3x12 self tap", foot="M3x10 self tap", edge="M3x12 self tap")


def _bx(u0, u1, ya, yb, w0, w1, sg):
    y0, y1 = sorted((sg * ya, sg * yb))
    return prim.box(u0, u1, y0, y1, w0, w1)


def _cyl_y(r, ya, yb, u, w, sg):
    y0, y1 = sorted((sg * ya, sg * yb))
    return prim.cylY(r, y0, y1, x=u, z=w)


def _uh(b):
    """Bracket entry: u centre or (u centre, half length)."""
    return (float(b[0]), float(b[1])) if isinstance(b, tuple) else (float(b), TAB_HALF)


def _bracket(b, sg):
    """Wall bracket: a block from the rail wall face (|y| 28) to the plate (w 36.5..47.5) with a d6 bore on the bolt
    axis from the 4 mm tab (|y| 28..32) out through the plate, so the M3 head and the hex key pass from outside. A bore
    (not a slot between two flanges) keeps the ceiling over the head seat small when printed plate down."""
    uc, hl = _uh(b)
    return _bx(uc - hl, uc + hl, Y_WALL, Y_IN + 0.01, BRACKET_W[0], BRACKET_W[1], sg)


def _plate(u0, u1, w0, w1, sg):
    return _bx(u0, u1, Y_IN, Y_OUT, w0, w1, sg)


def _cover(u0, u1, sg):
    return _bx(u0, u1, COVER_Y, Y_IN + 0.01, COVER_W[0], COVER_W[1], sg)


def _lip(u0, u1, sg):
    return _bx(u0, u1, LIP_Y, Y_IN + 0.01, W_TOP_STOP - LIP_T, W_TOP_STOP, sg)


def segment_solid(seg, sg):
    if seg == "a":
        u0, u1 = U_A
        s = [_plate(u0, u1, W_BOT, W_TOP, sg), _cover(u0, u1, sg),
             _bx(u0, u0 + REAR_WALL_T, Y_WALL + 0.5, Y_IN + 0.01, W_BOT, W_TOP, sg)]           # rear end wall
    elif seg == "b1":
        u0, u1 = U_B1
        s = [_plate(u0, u1, W_BOT, W_TOP_STOP, sg), _plate(u0, U_B1_TALL, W_BOT, W_TOP, sg),
             _cover(u0, U_B1_TALL, sg), _lip(U_B1_TALL - 0.01, u1, sg)]
    elif seg == "b2":
        u0, u1 = U_B2
        s = [_plate(u0, u1, W_BOT, W_TOP_STOP, sg), _lip(u0, u1, sg),
             _bx(FOOT_U - 6.0, FOOT_U + 6.0, FOOT_Y - 4.0, Y_IN + 0.01, 40.0, 43.5, sg)]       # foot on the plate
    else:
        u0, u1 = U_C
        s = [_plate(u0, u1, W_C_BOT, W_TOP, sg), _cover(u0, u1, sg),
             _plate(U_C_STRIP, u0 + 0.01, W_C_BOT, 39.5, sg)]
        for uu in EDGE_BOLTS_U:
            s.append(_cyl_y(4.0, Y_EDGE, Y_IN + 0.01, uu, W_EDGE_BOLT, sg))                      # bosses
    for b in WALL_BRACKETS.get(seg, {}).get(sg, ()):
        s.append(_bracket(b, sg))
    g = prim.union(s)
    for b in WALL_BRACKETS.get(seg, {}).get(sg, ()):
        uc = _uh(b)[0]
        g = g - _cyl_y(1.7, Y_WALL - 1.0, Y_WALL + 5.0, uc, W_BOLT, sg)                       # M3 clearance
        g = g - _cyl_y(2.95, Y_WALL + 4.0, Y_OUT + 1.0, uc, W_BOLT, sg)                       # head (d5.5) bore d5.9
        g = g - prim.place(prim.cyl(2.95, 3.0, r2=3.2), tf.mul(tf.T(uc, sg * (Y_IN - 0.5), W_BOLT), tf.Rx(-90 if sg > 0 else 90)))  # d6.4 at the plate face
    if seg == "b2":
        g = g - prim.cyl(1.7, 6.0, x=FOOT_U, y=sg * FOOT_Y, z=39.0)
    if seg == "c":
        for uu in EDGE_BOLTS_U:
            g = g - _cyl_y(1.7, Y_EDGE - 1.0, Y_OUT + 1.0, uu, W_EDGE_BOLT, sg)
    return g


def guard_rail_cut(n=None):
    """Cut from rail_solid: M3 self tap pilots (d2.6) for the guard brackets. Wall pilots along y from the outer face
    (|y| 28) 10.5 deep into wall and shelf; foot pilots down into the front plate; edge pilots 9.5 deep along y into
    the front plate's outer edge (|y| 64 .. 54.5), clear of the anchor holes."""
    c = []
    for seg, d in WALL_BRACKETS.items():
        for sg, us in d.items():
            for b in us:
                c.append(_cyl_y(1.3, 18.0, Y_WALL + 0.5, _uh(b)[0], W_BOLT, sg))
    for sg in (1, -1):
        c.append(prim.cyl(1.3, 9.5, x=FOOT_U, y=sg * FOOT_Y, z=31.0))
        for uu in EDGE_BOLTS_U:
            c.append(_cyl_y(1.3, Y_EDGE - 9.5, Y_EDGE + 0.5, uu, W_EDGE_BOLT, sg))
    return prim.union(c)


def pilot_positions():
    """(u, y, w, axis) of every rail pilot mouth (for the NOTES / assembly text)."""
    out = []
    for seg, d in WALL_BRACKETS.items():
        for sg, us in d.items():
            out += [(_uh(b)[0], sg * Y_WALL, W_BOLT, "y") for b in us]
    for sg in (1, -1):
        out.append((FOOT_U, sg * FOOT_Y, 40.0, "w"))
        out += [(uu, sg * Y_EDGE, W_EDGE_BOLT, "y") for uu in EDGE_BOLTS_U]
    return out


SEGMENTS = ("a", "b1", "b2", "c")


def guard_parts(P=None, n=None, bed=None):
    if bed is None:
        import launch as LA
        bed = LA.bed
    out = []
    names = dict(a="rear (end wall, top cover)", b1="mid front of the stop frame", b2="mid under the stop spring",
                 c="front (top cover)")
    for sg, side in ((1, "l"), (-1, "r")):
        for seg in SEGMENTS:
            s = segment_solid(seg, sg)
            R = tf.Rx(-90) if sg > 0 else tf.Rx(90)          # outer face (|y| 68) down on the bed
            nb = len(WALL_BRACKETS.get(seg, {}).get(sg, ()))
            refs, fix = [], []
            if nb:
                refs.append(f"{nb} x {SCREWS['wall']}")
                fix.append(f"{nb} wall bracket{'s' if nb > 1 else ''} ({SCREWS['wall']} into the rail wall)")
            if seg == "b2":
                refs.append(f"1 x {SCREWS['foot']}")
                fix.append(f"a foot on the front plate ({SCREWS['foot']} down)")
            if seg == "c":
                refs.append(f"2 x {SCREWS['edge']}")
                fix.append(f"2 bosses on the front plate edge ({SCREWS['edge']} along y)")
            out.append(Part(f"launch_guard_{side}_{seg}", s, "PLA", joint="launch_rail", place=tf.I4,
                            print_tf=bed(s, R), color="#b8c4cc", bought_ref="; ".join(refs),
                            notes=f"Spring guard {'left' if sg > 0 else 'right'} {names[seg]} (D33): 2 mm side plate "
                                  f"outside both spring pairs, " + " and ".join(fix) + "; fit by moving it inward"))
    return out


def guard_allowed():
    return []
