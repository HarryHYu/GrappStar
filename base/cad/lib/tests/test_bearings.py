"""Tests for cad.lib.bearings: valid solids, crush-rib interference, free inner race, CSK20PP key engagement."""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

import manifold3d as m3
import pytest

from cad.lib import bearings as B, prim, tf


def ok(s, one_shell=True):
    assert s.status() == m3.Error.NoError
    assert s.volume() > 0
    if one_shell:
        assert prim.shells(s) == 1


@pytest.mark.parametrize("fn", ["seat_608", "bearing_608", "boss_608", "bearing_608_inner_race", "seat_csk20",
                                "bearing_csk20", "boss_csk20", "seat_693", "seat_f693", "bearing_693",
                                "bearing_f693", "boss_693"])
def test_valid(fn):
    ok(getattr(B, fn)())


@pytest.mark.parametrize("bottom", ["lip", "through", "blind"])
def test_608_bottoms_valid(bottom):
    ok(B.seat_608(bottom=bottom))


def test_seat_f693_no_recess_valid():
    ok(B.seat_f693(flange_recess=False))


def _host(seat, size, h):
    return prim.cbox(size, size, h, z=-h) - seat


def test_608_crush_ribs():
    host = _host(B.seat_608(), 40, 15)
    v = (host ^ B.bearing_608()).volume()
    # 8 ribs x about 0.5 mm3 each
    assert 1.0 < v < 10.0, v
    assert (host ^ B.bearing_608_inner_race()).volume() < 1e-9
    # bearing without ribs never touches the bore wall: shrink to the rib crests and check the gap
    assert (host ^ prim.cyl(B.B608_OD / 2 - B.BRG_RIB_INTERF - 0.02, 7, z=-7)).volume() < 1e-9


@pytest.mark.parametrize("bottom", ["through", "blind"])
def test_608_inner_race_free_other_bottoms(bottom):
    host = _host(B.seat_608(bottom=bottom), 40, 15)
    assert (host ^ B.bearing_608_inner_race()).volume() < 1e-9
    assert (host ^ B.bearing_608()).volume() > 1.0


def test_csk20_key_engages():
    host = _host(B.seat_csk20(), 70, 20)
    brg = B.bearing_csk20()
    near_key = prim.box(18, 30, -6, 6, -21, 2)
    assert (host ^ brg ^ near_key).volume() < 1e-6          # key sits in the keyway with clearance
    assert (host ^ prim.place(brg, tf.Rz(2)) ^ near_key).volume() > 1.0
    assert (host ^ prim.place(brg, tf.Rz(-2)) ^ near_key).volume() > 1.0
    v = (host ^ brg).volume()
    assert 1.0 < v < 25.0, v                                 # crush ribs only
    assert (host ^ B.bearing_csk20_inner_race()).volume() < 1e-9


def test_693_seats():
    for seat, model in [(B.seat_693(), B.bearing_693()), (B.seat_f693(), B.bearing_f693()),
                        (B.seat_f693(flange_recess=False), B.bearing_f693().translate((0, 0, 0.6)))]:
        host = _host(seat, 20, 8)
        v = (host ^ model).volume()
        assert 0.2 < v < 3.0, v
