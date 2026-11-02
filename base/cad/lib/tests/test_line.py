"""Tests for cad.lib.line: eye throat 4.4 and faces 6.4, bosses, fairlead, line tube."""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

import math

import manifold3d as m3
import numpy as np
import pytest

from cad.lib import line as LN, prim


def ok(s):
    assert s.status() == m3.Error.NoError
    assert s.volume() > 0
    assert prim.shells(s) == 1


def hole_d(host, z):
    """Diameter of the hole in a host slice at z (max vertex radius of the inner ring)."""
    polys = host.slice(z).to_polygons()
    best = None
    for ring in polys:
        r = np.hypot(ring[:, 0], ring[:, 1])
        if r.max() < 5:                       # the hole ring, not the outer square
            best = 2 * r.max()
    return best


@pytest.mark.parametrize("t", [3.0, 6.0, 10.0])
def test_eye_throat_and_faces(t):
    c = LN.eye_cutter(t)
    ok(c)
    host = prim.cbox(14, 14, t) - c
    ok(host)
    assert abs(hole_d(host, t / 2) - 4.4) < 0.01
    assert abs(hole_d(host, 1e-4) - 6.4) < 0.01
    assert abs(hole_d(host, t - 1e-4) - 6.4) < 0.01
    i = LN.eye_info(t)
    assert abs(i["half_angle_deg"] - 45) < 1e-9 and abs(i["throat_len"] - (t - 2)) < 1e-9


def test_eye_thin_host():
    host = prim.cbox(14, 14, 1.5) - LN.eye_cutter(1.5)
    ok(host)
    assert abs(hole_d(host, 0.75) - 4.4) < 0.02


def test_eye_boss_and_fairlead():
    ok(LN.eye_boss(6))
    ok(LN.eye_boss(6, flange=1.5))
    f = LN.fairlead()
    ok(f)
    assert abs(hole_d(f, 3.0) - 4.4) < 0.01
    ok(prim.cbox(14, 14, 4) - LN.fairlead_cutter(4))


def test_line_tube():
    pts = [(0, 0, 0), (10, 0, 5), (20, 10, 5), (20, 10, 5), (30, 10, 0)]
    t = LN.line_tube(pts)
    ok(t)
    L = LN.line_length(pts)
    assert abs(t.volume() - math.pi * 0.25 * L) < 0.15 * math.pi * 0.25 * L
    hits = LN.line_hits(pts, {"in_way": prim.cbox(4, 4, 4, x=10, z=3), "clear": prim.cbox(4, 4, 4, x=0, y=20)})
    assert hits["in_way"] > 0 and hits["clear"] == 0
    # line through an eye on its axis clears the eye walls
    host = prim.cbox(14, 14, 6) - LN.eye_cutter(6)
    assert LN.line_hits([(0, 0, -5), (0, 0, 11)], [host])[0] == 0
