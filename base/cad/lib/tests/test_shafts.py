"""Tests for cad.lib.shafts: valid solids, keyed and D fits, torque helper."""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

import manifold3d as m3
import pytest

from cad.lib import shafts as S, bearings as B, prim, tf


def ok(s):
    assert s.status() == m3.Error.NoError
    assert s.volume() > 0
    assert prim.shells(s) == 1


@pytest.mark.parametrize("s", [
    lambda: S.keyed_shaft(40),
    lambda: S.keyed_shaft(40, end_lo=(8, 9), end_hi=[(12, 3), (8, 9)]),
    lambda: S.keyed_bore(15),
    lambda: S.d_shaft(20, 5, 0.5), lambda: S.d_bore(10, 5, 0.5),
    lambda: S.journal_608(), lambda: S.journal(12, 10),
    lambda: S.hex_shaft(20, 8), lambda: S.hex_bore(10, 8),
])
def test_valid(s):
    ok(s())


def test_key_dims():
    lo, hi = prim.bbox(S.keyed_shaft(30))
    assert abs(hi[0] - (10 + S.KEY20_H)) < 1e-6
    assert abs(hi[1] - 10) < 0.01


def test_keyed_fit():
    host = prim.cbox(50, 50, 10, z=-10) - S.keyed_bore(10)
    sh = S.keyed_shaft(30).translate((0, 0, -25))
    assert (host ^ sh).volume() < 1e-9
    assert (host ^ prim.place(sh, tf.Rz(3))).volume() > 1.0
    assert (host ^ prim.place(sh, tf.Rz(-3))).volume() > 1.0


def test_keyed_shaft_in_csk20_inner():
    sh = S.keyed_shaft(30).translate((0, 0, -22))
    brg = B.bearing_csk20()
    assert (brg ^ sh).volume() < 0.5      # 20 on 20 nominal: polygon facets only, key free in the keyway
    assert (brg ^ prim.place(sh, tf.Rz(3))).volume() > 2.0


def test_journal_in_608():
    sh = S.keyed_shaft(30, end_lo=(8, 9)).translate((0, 0, 0.5))
    assert (B.bearing_608() ^ sh).volume() < 0.5   # 8 on 8 nominal, coincident surfaces only


def test_d_and_hex_fit():
    for bore, sh in [(S.d_bore(10, 5, 0.5), S.d_shaft(20, 5, 0.5)), (S.hex_bore(10, 8), S.hex_shaft(20, 8))]:
        host = prim.cbox(20, 20, 10, z=-10) - bore
        sh = sh.translate((0, 0, -15))
        assert (host ^ sh).volume() < 1e-9
        assert (host ^ prim.place(sh, tf.Rz(10))).volume() > 0.05


def test_torque():
    assert 12.0 < S.shaft_torque_capacity(20) < 13.0
    assert S.shaft_torque_capacity(20, d_in=8) < S.shaft_torque_capacity(20)
    assert 3 < S.key_torque_capacity(20, 14) < 10
