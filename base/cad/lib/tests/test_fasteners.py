"""Tests for cad.lib.fasteners: valid cutters and models, models fit in their cutters."""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

import manifold3d as m3
import pytest

from cad.lib import fasteners as F, prim, fits

CUTTERS = [
    lambda: F.m3_clear(10), lambda: F.m3_clear(10, horizontal=True, teardrop=True), lambda: F.m3_close(5),
    lambda: F.m3_tap(8), lambda: F.m3_tap(8, horizontal=True), lambda: F.m3_counterbore(12),
    lambda: F.m3_counterbore(12, horizontal=True, teardrop=True), lambda: F.m3_countersunk(8),
    lambda: F.m3_insert(), lambda: F.m3_insert(screw_below=5), lambda: F.m3_nut_trap(),
    lambda: F.m3_nut_trap(nut_z=3, slot=10), lambda: F.m3_nut_trap(nut_z=3, slot=10, horizontal=True),
    lambda: F.m3_nyloc_trap(slot=5), lambda: F.m8_clear(10), lambda: F.m8_clear(10, horizontal=True, teardrop=True),
    lambda: F.m8_nut_trap(nut_z=4, slot=12),
]
MODELS = [lambda: F.m3_shcs(10), lambda: F.m3_nut(), lambda: F.m3_nut(nyloc=True), lambda: F.m3_insert_model(),
          lambda: F.m8_bolt(30), lambda: F.m8_nut()]


@pytest.mark.parametrize("fn", CUTTERS + MODELS)
def test_valid(fn):
    s = fn()
    assert s.status() == m3.Error.NoError
    assert s.volume() > 0
    assert prim.shells(s) == 1


def host(cut):
    return prim.cbox(30, 30, 20, z=-20) - cut


def test_horizontal_extra():
    a = prim.bbox(F.m3_clear(5))[1][0]
    b = prim.bbox(F.m3_clear(5, horizontal=True))[1][0]
    assert abs((b - a) * 2 - fits.HOLE_HORIZ_EXTRA) < 0.01
    t = F.m3_clear(5, horizontal=True, teardrop=True)
    assert prim.bbox(t)[1][1] > (fits.M3_CLEAR_D + fits.HOLE_HORIZ_EXTRA) / 2 * 1.35


def test_fits():
    assert (host(F.m3_clear(20)) ^ F.m3_shcs(16)).volume() < 1e-6
    assert (host(F.m3_close(20)) ^ F.m3_shcs(16)).volume() < 1e-6
    assert (host(F.m3_counterbore(16)) ^ F.m3_shcs(12).translate((0, 0, -3.2))).volume() < 1e-6
    assert (host(F.m3_insert()) ^ F.m3_insert_model()).volume() < 1e-6
    assert (host(F.m8_clear(20)) ^ F.m8_bolt(16)).volume() < 1e-6
    assert (host(F.m3_tap(20)) ^ F.m3_shcs(10)).volume() > 1.0      # self tapping bites


def test_nut_traps():
    for nz, slot in [(0.0, 0.0), (3.0, 10.0)]:
        h = host(F.m3_nut_trap(nut_z=nz, slot=slot, bolt_len=20))
        assert (h ^ F.m3_nut().translate((0, 0, -nz - 0.15))).volume() < 1e-6
        assert (h ^ F.m3_shcs(16)).volume() < 1e-6
    h = host(F.m3_nut_trap(nut_z=3, slot=10, bolt_len=20))
    assert (h ^ F.m3_nut().translate((6, 0, -3.15))).volume() < 1e-6   # slides in along the slot
    h = host(F.m3_nyloc_trap(nut_z=2, bolt_len=20))
    assert (h ^ F.m3_nut(nyloc=True).translate((0, 0, -2.15))).volume() < 1e-6
    h = host(F.m8_nut_trap(nut_z=4, bolt_len=20))
    assert (h ^ F.m8_nut().translate((0, 0, -4.15))).volume() < 1e-6
    assert (h ^ F.m8_bolt(16)).volume() < 1e-6
    # the nut cannot turn in its trap
    from cad.lib import tf
    h = host(F.m3_nut_trap(bolt_len=20))
    assert (h ^ prim.place(F.m3_nut().translate((0, 0, -0.15)), tf.Rz(15))).volume() > 0.1
