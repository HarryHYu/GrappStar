"""Tests for drum_gears: geometry sanity and exact manifold mesh checks.

Run: cd base && ../kedge/.venv/bin/python -m pytest cad/parts/test_drum_gears.py -q
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drum_gears import (  # noqa: E402
    center_distance,
    gear,
    gear_info,
    inv,
    involute_profile,
    mesh_phase,
)

WIDTH = 4.0
TOL = 1e-3  # mm3, "no intersection"
MAX_TRI = 20000


def _overlap(g1, g2, a, t1, t2, phase1=0.0):
    """Exact intersection volume of gear 1 (origin, rotated t1) and gear 2
    (center at distance a in direction phase1, rotated t2)."""
    p = math.radians(phase1)
    g2p = g2.rotate((0, 0, t2)).translate((a * math.cos(p), a * math.sin(p), 0))
    return (g1.rotate((0, 0, t1)) ^ g2p).volume()


def _sweep(g1, g2, z1, z2, a, start, span, n, phase1=0.0):
    worst = 0.0
    for i in range(n):
        t1 = start + span * i / n
        t2 = mesh_phase(z1, z2, t1, phase1)
        worst = max(worst, _overlap(g1, g2, a, t1, t2, phase1))
    return worst


@pytest.fixture(scope="module")
def pair_a():
    g1 = gear(10, 1.0, WIDTH, x=0.3)
    g2 = gear(90, 1.0, WIDTH, x=-0.3)
    return g1, g2, center_distance(10, 90, 1.0, 0.3, -0.3)


@pytest.fixture(scope="module")
def pair_c():
    seg = gear(64, 1.25, WIDTH, keep=[(100, 325)])
    pin = gear(10, 1.25, WIDTH, x=0.4)
    return seg, pin, center_distance(64, 10, 1.25, 0.0, 0.4)


@pytest.fixture(scope="module")
def pair_d():
    g = gear(48, 1.5, WIDTH)
    return g, g, center_distance(48, 48, 1.5)


# ---------------------------------------------------------------- basics


def test_inv_and_center_distance():
    assert center_distance(10, 90, 1.0) == pytest.approx(50.0)
    # x1 + x2 = 0 gives the standard distance
    assert center_distance(10, 90, 1.0, 0.3, -0.3) == pytest.approx(50.0)
    # positive shift sum: a_w > pa and a grows by a bit less than m (x1+x2)
    a = center_distance(64, 10, 1.25, 0.0, 0.4)
    assert 46.25 < a < 46.25 + 1.25 * 0.4
    # consistency of the working pressure angle formula
    a0 = 1.25 * 74 / 2
    a_w = math.acos(a0 * math.cos(math.radians(20)) / a)
    lhs = inv(a_w)
    rhs = 2 * math.tan(math.radians(20)) * 0.4 / 74 + inv(math.radians(20))
    assert lhs == pytest.approx(rhs, rel=1e-9)


def test_gear_info():
    gi = gear_info(20, 2.0, x=0.5)
    assert gi["r_p"] == pytest.approx(20.0)
    assert gi["r_b"] == pytest.approx(20.0 * math.cos(math.radians(20)))
    assert gi["r_a"] == pytest.approx(2.0 * (10 + 1 + 0.5))
    assert gi["r_f"] == pytest.approx(2.0 * (10 - 1.25 + 0.5))
    assert gi["pitch"] == pytest.approx(math.pi * 2.0)


def test_profile_ccw_and_tooth0_on_x():
    pts = involute_profile(12, 2.0)
    area2 = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1)
                in zip(pts, pts[1:] + pts[:1]))
    assert area2 > 0  # CCW
    # the farthest points (tips) of tooth 0 are symmetric about +X
    gi = gear_info(12, 2.0)
    tips = [p for p in pts if math.hypot(*p) > gi["r_a"] - 1e-6
            and abs(math.atan2(p[1], p[0])) < math.pi / 12]
    assert tips
    ys = sorted(p[1] for p in tips)
    assert ys[0] == pytest.approx(-ys[-1], abs=1e-9)


def test_backlash_thins_teeth():
    # area drops by about z * backlash/2 * tooth height-ish; just monotonic
    from shapely.geometry import Polygon
    a0 = Polygon(involute_profile(20, 1.0, backlash=0.0)).area
    a1 = Polygon(involute_profile(20, 1.0, backlash=0.3)).area
    assert a1 < a0
    assert Polygon(involute_profile(10, 1.0, x=0.3)).is_valid


# ----------------------------------------------- (e) manifold validity


@pytest.mark.parametrize("kw", [
    dict(z=10, m=1.0, x=0.3),
    dict(z=90, m=1.0, x=-0.3),
    dict(z=64, m=1.25, keep=[(100, 325)]),
    dict(z=10, m=1.25, x=0.4),
    dict(z=48, m=1.5),
    dict(z=48, m=1.5, bore=8.0),
])
def test_valid_single_shell_and_volume(kw):
    g = gear(width=WIDTH, **kw)
    assert g.status().name == "NoError"
    assert len(g.decompose()) == 1
    assert 0 < g.num_tri() <= MAX_TRI
    gi = gear_info(kw["z"], kw["m"], x=kw.get("x", 0.0))
    hole = math.pi * (kw.get("bore") or 0.0) ** 2 / 4 * WIDTH
    v_root = math.pi * gi["r_f"] ** 2 * WIDTH - hole
    v_tip = math.pi * gi["r_a"] ** 2 * WIDTH - hole
    assert v_root < g.volume() < v_tip
    lo, hi = g.bounding_box()[2], g.bounding_box()[5]
    assert lo == pytest.approx(0.0) and hi == pytest.approx(WIDTH)


def test_segment_gear_removes_teeth():
    full = gear(64, 1.25, WIDTH)
    seg = gear(64, 1.25, WIDTH, keep=[(100, 325)])
    blank = gear(64, 1.25, WIDTH, keep=[])  # root polygon only
    assert len(blank.decompose()) == 1
    n_kept = sum(1 for k in range(64) if 100 <= k * 360 / 64 <= 325)
    v_tooth = (full.volume() - blank.volume()) / 64
    v_seg_teeth = seg.volume() - blank.volume()
    assert v_seg_teeth == pytest.approx(n_kept * v_tooth, rel=1e-3)


# ------------------------------------------------- mesh phase / overlaps


def test_mesh_phase_gap_faces_gear1():
    # theta1 = 0: a gear 2 gap center must point at 180 deg
    for z2 in (9, 10, 48, 90):
        t2 = mesh_phase(10, z2, 0.0)
        rel = (180.0 - t2) % (360.0 / z2)
        assert rel == pytest.approx(180.0 / z2)


def test_a_no_intersection(pair_a):
    g1, g2, a = pair_a
    # 60 samples over 1.5 pinion pitches
    assert _sweep(g1, g2, 10, 90, a + 0.1, 0.0, 54.0, 60) < TOL


def test_a_no_intersection_fine_nominal_distance(pair_a):
    # 1 deg pinion steps over a full pinion pitch, at the exact center distance
    g1, g2, a = pair_a
    assert _sweep(g1, g2, 10, 90, a, 0.0, 36.0, 36) < TOL


def test_a_off_axis_phase1(pair_a):
    g1, g2, a = pair_a
    assert _sweep(g1, g2, 10, 90, a + 0.1, 17.0, 36.0, 24, phase1=37.0) < TOL


def test_zero_backlash_exact_distance_one_degree(pair_a):
    # even with zero backlash the chordal (inscribed) involute keeps the
    # overlap below tolerance at the exact center distance
    g1 = gear(10, 1.0, WIDTH, x=0.3, backlash=0.0)
    g2 = gear(90, 1.0, WIDTH, x=-0.3, backlash=0.0)
    assert _sweep(g1, g2, 10, 90, pair_a[2], 0.0, 36.0, 36) < TOL


def test_b_too_close_intersects(pair_a):
    g1, g2, a = pair_a
    worst = _sweep(g1, g2, 10, 90, a - 0.6, 0.0, 36.0, 12)
    assert worst > 0.1


def test_wrong_phase_intersects(pair_a):
    # negative control: gear 2 rotated by half a pitch from mesh_phase clashes
    g1, g2, a = pair_a
    t2 = mesh_phase(10, 90, 0.0) + 180.0 / 90
    assert _overlap(g1, g2, a + 0.1, 0.0, t2) > 0.1


def test_c_segment_gear(pair_c):
    seg, pin, a = pair_c
    # segment gear is gear 1; its teeth face the pinion (+X) for
    # theta1 in [35, 260]. Sweep that whole toothed range.
    assert _sweep(seg, pin, 64, 10, a + 0.1, 30.0, 240.0, 60) < TOL


def test_c_segment_gear_toothless_any_pinion_angle(pair_c):
    seg, pin, a = pair_c
    # toothless sector facing the pinion: pinion may sit at any angle
    for t1 in (-20.0, 0.0, 15.0, 300.0):
        for t2 in (0.0, 7.0, 18.0, 29.0):
            assert _overlap(seg, pin, a + 0.1, t1, t2) < TOL


def test_d_48_48(pair_d):
    g1, g2, a = pair_d
    assert _sweep(g1, g2, 48, 48, a + 0.1, 0.0, 1.5 * 7.5, 60) < TOL
    assert _sweep(g1, g2, 48, 48, a - 0.6, 0.0, 7.5, 8) > 0.1
