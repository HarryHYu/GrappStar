"""Tests for cad.lib.motors: N20 models fit their mounts, wiper plate / coupler / envelope fit."""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

import manifold3d as m3
import pytest

from cad.lib import motors as M, prim, tf


def ok(s, one=True):
    assert s.status() == m3.Error.NoError
    assert s.volume() > 0
    if one:
        assert prim.shells(s) == 1


def V(a, b):
    return (a ^ b).volume()


def test_n20_worm_dims():
    w = M.n20_worm()
    ok(w)
    (x0, y0, z0), (x1, y1, z1) = prim.bbox(w)
    assert abs((z1 - z0) - M.N20W_SHAFT_TIPS) < 1e-6
    assert abs((x1 - x0) - (M.N20W_LEN + M.N20_TERM_L)) < 1e-6
    assert abs((y1 - y0) - 12.0) < 0.05


def test_n20_worm_mount():
    w = M.n20_worm()
    m = M.n20_worm_mount()
    for k in ("cutter", "cradle", "clamp"):
        ok(m[k])
    assert V(w, m["cradle"]) < 1e-6
    assert V(w, m["clamp"]) < 1e-6
    assert V(m["cradle"], m["clamp"]) < 1e-6
    assert abs(V(w, m["cutter"]) - w.volume()) < 1e-3      # model inside the cutter


def test_n20_straight_dims():
    s = M.n20_straight()
    ok(s)
    (_, y0, z0), (_, y1, z1) = prim.bbox(s)
    assert abs(z1 - M.N20S_SHAFT_L) < 1e-6
    assert abs((y1 - y0) - 10.0) < 1e-6


def test_n20_mount():
    s = M.n20_straight()
    n = M.n20_mount()
    ok(n["bracket"])
    ok(n["cutter"])
    assert V(s, n["bracket"]) < 1e-6
    assert abs(V(s, n["cutter"]) - s.volume()) < 1e-3
    ok(M.n20_mount(cradle=False)["bracket"])


@pytest.mark.parametrize("kw", [{}, {"slot": 6}, {"slot_deg": 20}, {"slot": 4, "slot_deg": 10},
                                {"holes": [(30, 0, 6), (-30, 0, 6)], "boss_d": 22}])
def test_wiper_plate(kw):
    p = M.wiper_plate(**kw)
    ok(p)
    assert V(p, M.wiper_motor_envelope()) < 1e-6


def test_wiper_plate_slot_removes_more():
    assert M.wiper_plate(slot=6).volume() < M.wiper_plate().volume()


@pytest.mark.parametrize("kw", [{}, {"out": "d"}, {"length": 60}])
def test_wiper_coupler_fits_shaft(kw):
    c = M.wiper_coupler(**kw)
    ok(c)
    e = M.wiper_motor_envelope()
    z = M.wiper_coupler_z()
    assert z > 6.0                                     # clear of the default 6 mm plate
    assert V(c.translate((0, 0, z)), e) < 1e-6
    # the flats drive it: turning the shaft 10 deg in the bore interferes
    assert V(prim.place(c, tf.T(0, 0, z) @ tf.Rz(10)), e) > 0


def test_wiper_coupler_variants():
    ok(M.wiper_coupler(thread=None))
    ok(M.wiper_coupler(taper_deg=10, flat=None))
    i = M.wiper_coupler_info()
    assert i["z_out"] > i["z_chamber"] > i["z_web"]


def test_wiper_envelope():
    e = M.wiper_motor_envelope()
    ok(e)
    e2 = M.wiper_motor_envelope({"case_d": 80, "pcd": 50})
    ok(e2)
    assert e2.volume() < e.volume()
