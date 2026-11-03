"""pytest for launch_gears.py (run: kedge/.venv/bin/python -m pytest base/cad/parts/test_launch_gears.py -s)."""
from __future__ import annotations

import math
import os
import sys

import pytest
from manifold3d import CrossSection, Error, Manifold
from shapely.geometry import Polygon

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import launch_gears as G  # noqa: E402

M = 2.5
Z1, Z2 = 26, 40


def check_solid(s: Manifold, genus: int):
    assert s.status() == Error.NoError
    assert s.volume() > 0
    assert len(s.decompose()) == 1
    assert s.genus() == genus


# ------------------------------------------------------------------------------------------- validity
@pytest.mark.parametrize("z", [10, 12, 17, 26, 40, 41, 42, 60, 80])
def test_profile_simple_polygon(z):
    p = G.involute_profile_2d(M, z)
    poly = Polygon(p)
    assert poly.is_valid and poly.exterior.is_simple
    assert G.root_r(M, z) - 1e-6 < max(math.hypot(*q) for q in p) <= G.tip_r(M, z) + 1e-6
    assert min(math.hypot(*q) for q in p) >= G.root_r(M, z) - 0.01


def test_spur_solids():
    check_solid(G.spur(M, Z1, 12), 0)
    check_solid(G.spur(M, Z1, 12, bore=8), 1)
    check_solid(G.spur(M, Z2, 12, bore=10, key=(3, 1.5), hub=(30, 6)), 1)
    check_solid(G.spur(M, 10, 8, bore=5), 1)


def test_rack_solid():
    check_solid(G.rack(M, 20, 12), 0)


def test_sector_solid():
    check_solid(G.sector_gear(M, Z2, 12, 12, bore=10, key=(3, 1.5)), 1)
    check_solid(G.sector_gear(M, Z2, 12, 12, bore=10, last_relief=False), 1)


def test_worm_solids():
    check_solid(G.worm(2.0, 24.0, 40.0, bore=6), 1)
    check_solid(G.worm(2.0, 24.0, 30.0, starts=2), 0)
    check_solid(G.worm_wheel(2.0, 20, 12.0, 24.0, bore=6), 1)


def test_bevel_solids():
    g1, g2, info = G.bevel_pair(2.0, 15, 30, 8.0, bore1=6, bore2=8)
    check_solid(g1, 1)
    check_solid(g2, 1)
    assert info["pitch_cone_deg"][0] + info["pitch_cone_deg"][1] == pytest.approx(90.0)
    assert info["cone_distance"] == pytest.approx(math.hypot(15, 30))


def test_helpers():
    assert G.pitch_r(M, Z2) == 50.0
    assert G.tip_r(M, Z2) == 52.5
    assert G.root_r(M, Z2) == pytest.approx(46.875)
    assert G.center_distance(M, Z1, Z2) == pytest.approx(82.6)


# ------------------------------------------------------------------------------------------- meshing 2D
def test_spur_pair_no_overlap():
    a, b = G.spur_2d(M, Z1), G.spur_2d(M, Z2)
    c = G.center_distance(M, Z1, Z2)
    ph = G.mesh_phase(Z2, 180.0)
    worst = 0.0
    for i in range(60):
        t = i * (2 * 360.0 / Z1) / 60          # two pinion tooth pitches
        worst = max(worst, G.gears_overlap_2d(a, (0, 0, t), b, (c, 0, ph - t * Z1 / Z2)))
    print(f"\nspur z{Z1}/z{Z2}: max overlap {worst:.4f} mm2")
    assert worst == 0.0
    # control: wrong phase must collide
    assert G.gears_overlap_2d(a, (0, 0, 0), b, (c, 0, ph + 180.0 / Z2)) > 1.0


def test_rack_spur_no_overlap():
    g, r = G.spur_2d(M, Z2), G.rack_2d(M, 24)
    R = G.pitch_r(M, Z2)
    p = math.pi * M
    worst = 0.0
    for i in range(60):
        th = i * 0.5                              # gear CCW, rack moves -x
        x = -R * math.radians(th) - 12 * p
        worst = max(worst, G.gears_overlap_2d(g, (0, -R, G.mesh_phase(Z2, 90.0) + th), r, (x, 0, 0)))
    print(f"\nrack vs z{Z2}: max overlap {worst:.4f} mm2")
    assert worst == 0.0
    assert G.gears_overlap_2d(g, (0, -R, 90.0), r, (-12 * p, 0, 0)) > 1.0


def _components_outside(cs: CrossSection, radius: float) -> int:
    return sum(1 for c in (cs - CrossSection.circle(radius, 512)).decompose() if c.area() > 1e-6)


@pytest.mark.parametrize("n", [5, 12, 30])
def test_sector_tooth_count(n):
    r, ra, rf = G.pitch_r(M, Z2), G.tip_r(M, Z2), G.root_r(M, Z2)
    s = G.sector_gear_2d(M, Z2, n, first_tooth_angle=17.0)
    assert len(s.decompose()) == 1
    assert _components_outside(s, rf + 0.3 * M) == n          # all teeth incl. the pitch-cut one
    assert _components_outside(s, r + 0.3 * M) == n - 1       # last tooth is cut to the pitch circle
    assert _components_outside(s, ra + 0.05 * M) == 1         # only the tall tooth beats the tip circle
    s2 = G.sector_gear_2d(M, Z2, n, last_relief=False)
    assert _components_outside(s2, r + 0.3 * M) == n
    assert _components_outside(s2, ra + 0.05 * M) == 0


def test_sector_tall_tip_width():
    s = G.sector_gear_2d(M, Z2, 12, tall_factor=1.6)
    rmax = s.bounds()
    t = G._Tooth(M, Z2)
    r_tall = t.r_for_width(0.25 * M, t.r + 1.6 * M)
    assert 2 * r_tall * math.sin(t.theta(r_tall)) >= 0.25 * M - 1e-6
    assert max(abs(v) for v in rmax) <= r_tall + 1e-3


def test_sector_vs_rack_cocking_order():
    """Sector turns CW; teeth must enter mesh 0, 1, 2 ... against a rack (deeper root for the tall tooth)."""
    n, a0 = 12, 120.0
    s = G.sector_gear_2d(M, Z2, n, first_tooth_angle=a0)
    p = math.pi * M
    rk = G.rack_2d(M, n + 12, dedendum=1.45)
    R = G.pitch_r(M, Z2)
    x0 = p / 2 - R * math.radians(a0 - 90.0) - (n + 6) * p
    worst = 0.0
    steps = 150
    span = (a0 - 90.0) + (n + 2) * 360.0 / Z2
    for i in range(steps + 1):
        th = span * i / steps
        worst = max(worst, G.gears_overlap_2d(s, (0, -R, -th), rk, (x0 + R * math.radians(th), 0, 0)))
    print(f"\nsector vs rack: max overlap {worst:.4f} mm2")
    assert worst == 0.0


# ------------------------------------------------------------------------------------------- worm 3D
def test_worm_wheel_mesh():
    m, pd, zw, face, length = 2.0, 24.0, 20, 12.0, 40.0
    w = G.worm(m, pd, length)
    ww = G.worm_wheel(m, zw, face, pd)
    a = G.worm_center_distance(m, zw, pd)
    lead = math.pi * m
    # worm along X at (y=a, z=face/2); its -Y side (own angle -90) faces the wheel. Thread centre at
    # x=0 needs local height length/2 at alpha=-90: phi0 = -90 - 360*(length/2)/lead.
    phi0 = -90.0 - 360.0 * (length / 2) / lead
    worst = 0.0
    for i in range(24):
        phi = i * 15.0
        wp = w.translate((0, 0, -length / 2)).rotate((0, 0, phi0 + phi)).rotate((0, 90, 0)) \
              .translate((0, a, face / 2))
        wh = ww.rotate((0, 0, G.mesh_phase(zw, 90.0) + phi / zw))
        worst = max(worst, G.overlap_volume(wp, wh))
    print(f"\nworm/wheel: max overlap {worst:.4f} mm3")
    assert worst < 5.0


# ------------------------------------------------------------------------------------------- bevel 3D
@pytest.mark.parametrize("z1,z2", [(15, 30), (20, 20)])
def test_bevel_mesh(z1, z2):
    g1, g2, info = G.bevel_pair(2.0, z1, z2, 8.0)
    worst = 0.0
    for i in range(30):
        a1 = i * (2 * 360.0 / z1) / 30
        worst = max(worst, G.overlap_volume(g1.rotate((0, 0, a1)),
                                            G.bevel_place_second(g2, info, -a1 * z1 / z2)))
    print(f"\nbevel z{z1}/z{z2}: max overlap {worst:.4f} mm3")
    assert worst < 1.0
    # control: unphased collides
    assert G.overlap_volume(g1, G.bevel_place_second(g2, info, 0.0, phase=False)) > 1.0
