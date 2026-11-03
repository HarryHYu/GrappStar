import math
import os
import sys

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

import numpy as np
import manifold3d as m3
import pytest
from shapely.geometry import Point

from cad.lib import fits, gears, prim

FACE = 3.0


def ok(s, shells=1):
    assert s.status() == m3.Error.NoError
    assert len(s.decompose()) == shells
    assert s.volume() > 0


def pitch_arcs(poly, r):
    circ = Point(0, 0).buffer(r, 8192).exterior
    return [g.length for g in poly.intersection(circ).geoms]


@pytest.mark.parametrize("m,z,x", [(1.5, 12, 0.0), (2, 20, 0.0), (1, 17, 0.3), (1, 8, 0.0), (1, 8, 0.5),
                                   (1, 51, -0.3), (1, 120, 0.0)])
def test_profile_radii_thickness(m, z, x):
    bl = fits.GEAR_BACKLASH
    poly = gears.spur_profile(m, z, shift=x, backlash=bl)
    inf = gears.spur_info(m, z, x)
    c = np.asarray(poly.exterior.coords)
    r = np.hypot(c[:, 0], c[:, 1])
    assert abs(r.max() - inf["r_tip"]) < 0.05
    assert abs(r.min() - inf["r_root"]) < 0.05
    s_nom = math.pi * m / 2 + 2 * x * m * math.tan(math.radians(20)) - bl / 2
    arcs = pitch_arcs(poly, inf["r_pitch"])
    assert len(arcs) >= z
    assert abs(float(np.median(arcs)) - s_nom) < 0.03
    # every tooth (drop the arc split by the circle start point)
    full = [a for a in arcs if a > 0.6 * s_nom]
    assert all(abs(a - s_nom) < 0.03 for a in full)


def test_info_math():
    inf = gears.spur_info(2, 20)
    assert inf["r_pitch"] == 20 and abs(inf["r_tip"] - 22) < 1e-9 and abs(inf["r_root"] - 17.5) < 1e-9
    assert abs(inf["z_min_no_undercut"] - 17.1) < 0.1
    assert gears.spur_info(1, 12)["undercut"] and not gears.spur_info(1, 12, shift=0.4)["undercut"]
    assert abs(gears.center_distance(2, 20, 20) - (40 + fits.GEAR_CENTER_EXTRA)) < 1e-9
    aw = gears.working_pressure_angle(17, 51, 0.4, 0.1)
    assert 21.5 < aw < 22.5
    assert gears.center_distance(1, 17, 51, 0.4, 0.1, extra=0) > 34.4
    eps = gears.contact_ratio(2, 20, 20)
    assert 1.5 < eps < 1.65
    s = gears.lewis_stress(1.5, 20, 6, 0.5)
    assert abs(s - 500 / 15 / (6 * 1.5 * 0.322)) < 1e-6
    assert abs(gears.lewis_max_torque(1.5, 20, 6, s) - 0.5) < 1e-9
    assert gears.ALLOW_PLA < gears.ALLOW_PPA_CF


def _mesh_check(g1, g2, place2, rot1, rot2, steps, r1, gap_arc):
    """g2 placed by place2(angle2); g1 rotates about the origin. Zero overlap over the steps, overlap when g1 is
    turned on by the one-sided backlash plus a margin."""
    for i in range(steps):
        a1 = rot1 * i / steps
        a2 = rot2 * i / steps
        A = g1.rotate((0, 0, a1))
        B = place2(a2)
        assert (A ^ B).volume() < 1e-6, f"overlap at step {i}"
        dA = math.degrees((gap_arc + 0.05) / r1)
        hit = max((g1.rotate((0, 0, a1 + s * dA)) ^ B).volume() for s in (1, -1))
        assert hit > 1e-4, f"no contact within backlash at step {i}"


@pytest.mark.parametrize("m,z1,z2,x1,x2", [(1.5, 12, 40, 0.0, 0.0), (2, 20, 20, 0.0, 0.0), (1, 17, 51, 0.4, 0.1)])
def test_pair_mesh(m, z1, z2, x1, x2):
    g1 = gears.spur_gear(m, z1, FACE, shift=x1)
    g2 = gears.spur_gear(m, z2, FACE, shift=x2)
    ok(g1); ok(g2)
    a = gears.center_distance(m, z1, z2, x1, x2)
    aw = math.radians(gears.working_pressure_angle(z1, z2, x1, x2))
    jt = fits.GEAR_BACKLASH + 2 * fits.GEAR_CENTER_EXTRA * math.tan(aw)
    # gear 2: a tooth space faces gear 1's tooth 0 on +X
    ph2 = 180 - 180.0 / z2
    r1w = a * z1 / (z1 + z2)
    _mesh_check(g1, g2, lambda a2: g2.rotate((0, 0, ph2 + a2)).translate((a, 0, 0)),
                360.0 / z1, -360.0 / z2, 8, r1w, jt / 2)


def test_rack_mesh():
    m, z = 1.5, 14
    rk = gears.rack(m, 10, FACE, 4.0)
    ok(rk)
    g = gears.spur_gear(m, z, FACE)
    yc = -gears.rack_center_distance(m, z)
    p = math.pi * m
    steps = 8
    r = m * z / 2
    jt = fits.GEAR_BACKLASH + 2 * fits.GEAR_CENTER_EXTRA * math.tan(math.radians(20))
    for i in range(steps):
        ph = 360.0 / z * i / steps
        G = g.rotate((0, 0, 90 + ph)).translate((5 * p, yc, 0))
        R = rk.translate((-r * math.radians(ph), 0, 0))
        assert (G ^ R).volume() < 1e-6
        dA = (jt / 2 + 0.05) / r
        hit = max((g.rotate((0, 0, 90 + ph + s * math.degrees(dA))).translate((5 * p, yc, 0)) ^ R).volume()
                  for s in (1, -1))
        assert hit > 1e-4


def test_internal_mesh():
    m, zr, zp = 1.5, 40, 16
    ring = gears.internal_gear(m, zr, FACE, rim_od=m * zr + 16)
    ok(ring)
    inf = gears.internal_info(m, zr)
    pin = gears.spur_gear(m, zp, FACE)
    a = gears.internal_center_distance(m, zr, zp)
    jt = fits.GEAR_BACKLASH + 2 * fits.GEAR_CENTER_EXTRA * math.tan(math.radians(20))
    steps = 8
    rp = m * zp / 2
    for i in range(steps):
        ap = 360.0 / zp * i / steps
        ar = ap * zp / zr
        P = pin.rotate((0, 0, ap)).translate((a, 0, 0))
        R = ring.rotate((0, 0, ar))
        assert (P ^ R).volume() < 1e-6
        dA = math.degrees((jt / 2 + 0.05) / rp)
        hit = max((pin.rotate((0, 0, ap + s * dA)).translate((a, 0, 0)) ^ R).volume() for s in (1, -1))
        assert hit > 1e-4
    assert inf["r_tip"] < m * zr / 2


def test_options_valid():
    g = gears.spur_gear(1.5, 20, 6, bore=8, bore_kind="key", hub=(14, 5))
    ok(g)
    assert abs(prim.bbox(g)[1][2] - 11) < 1e-6
    for kind in ("round", "d", "hex", "spline"):
        ok(gears.spur_gear(1, 30, 4, bore=6, bore_kind=kind))
    part = gears.spur_gear(1.5, 24, 5, present=[0, 1, 2, 3], bore=5)
    ok(part)
    assert part.volume() < gears.spur_gear(1.5, 24, 5, bore=5).volume()
    small = gears.spur_gear(1, 8, 4)
    ok(small)
    big = gears.spur_gear(1, 120, 4, bore=10)
    ok(big)
    inf = gears.spur_info(1, 120)
    lo, hi = prim.bbox(big)
    assert abs(hi[0] - inf["r_tip"]) < 0.05


def test_din_key_table():
    assert gears._din_key(20) == (6, 2.8)
    assert gears._din_key(12) == (4, 1.8)
    assert gears._din_key(12.1) == (5, 2.3)
    assert gears._din_key(22) == (6, 2.8)
    assert gears._din_key(8) == (2, 1.0)
    assert gears._din_key(25) == (8, 3.3)


def test_key_bore_matches_keyed_shaft():
    from cad.lib import shafts
    L = 10.0
    sh = shafts.keyed_shaft(L, d=20)
    g = gears.spur_gear(2, 30, L, bore=20 + fits.SHAFT_IN_BORE, bore_kind="key")
    assert (sh ^ g).volume() < 1e-6
    assert (sh.rotate((0, 0, 3)) ^ g).volume() > 1e-3
    # keyway equals shafts.keyed_bore: 6.0 wide, floor at r 12.8
    from shapely.geometry import Polygon, box
    cs = gears.bore_cs(20 + fits.SHAFT_IN_BORE, "key")
    assert abs(cs.bounds()[2] - 12.8) < 1e-3
    slot = box(10.6, -5, 14, 5).intersection(Polygon(cs.to_polygons()[0]))
    assert abs((slot.bounds[3] - slot.bounds[1]) - 6.0) < 1e-3
