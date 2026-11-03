"""Tests for cad.lib.clips: valid solids, snap pin in hole (barbs only), clip seated in the groove."""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

import manifold3d as m3
import pytest

from cad.lib import clips as C, prim


def ok(s):
    assert s.status() == m3.Error.NoError
    assert s.volume() > 0
    assert prim.shells(s) == 1


def V(a, b):
    return (a ^ b).volume()


@pytest.mark.parametrize("d,L", [(3, 4), (4, 6), (6, 10), (12, 20)])
def test_snap_pin_in_hole(d, L):
    pin = C.snap_pin(d, L)
    ok(pin)
    info = C.snap_pin_info(d, L)
    assert info["closes"]
    host = prim.cbox(4 * d, 4 * d, L) - C.pin_hole_cutter(d, L)
    ok(host)
    assert V(pin, host) < 1e-6                      # seated: barbs past the far face, shank clear
    # pushed in until the barbs sit inside the hole (unflexed): overlap only in the barb band
    shift = -(info["barb_z1"] - L) + 0.2
    p2 = pin.translate((0, 0, shift))
    ov = p2 ^ host
    assert ov.volume() > 0
    (_, _, z0), (_, _, z1) = prim.bbox(ov)
    assert z0 >= info["barb_z0"] + shift - 1e-6 and z1 <= info["barb_z1"] + shift + 1e-6


@pytest.mark.parametrize("d", [4, 6, 8, 12])
@pytest.mark.parametrize("L", [5, 8, 10, 20])
def test_snap_pin_strain_defaults(d, L):
    i = C.snap_pin_info(d, L)
    assert i["strain_ok"], (d, L, i["strain"])
    assert i["strain"] <= C.PLA_STRAIN_OK
    assert i["barb"] > 0.15 and i["closes"]


@pytest.mark.parametrize("d,grip", [(4, 6), (6, 8), (12, 20)])
def test_clip_in_groove(d, grip):
    pin = C.clip_pin(d, grip)
    clip = C.c_clip(d)
    ok(pin)
    ok(clip)
    z = C.clip_pin_clip_z(d, grip)
    seated = clip.translate((0, 0, z))
    assert V(seated, pin) < 1e-6                    # sits in the groove, no interference
    g = C.groove_dims(d)
    i = C.clip_info(d)
    assert i["id"] < d - C.fits.HOLE_SLIDE          # cannot slide over the shank
    assert i["opening"] < g["core_d"]               # snaps over the core
    # captured axially: moving it past either groove wall hits the pin
    assert V(clip.translate((0, 0, z - 0.3)), pin) > 0
    assert V(clip.translate((0, 0, z + (g["width"] - i["t"]) + 0.3)), pin) > 0
    host = prim.cbox(3 * d, 3 * d, grip) - C.pin_hole_cutter(d, grip)
    assert V(pin, host) < 1e-6 and V(seated, host) < 1e-6


def test_v10_clip_matches_measured():
    i = C.clip_info(12)
    assert abs(i["od"] - 16.2) < 0.05 and abs(i["id"] - 10.35) < 0.01
    assert abs(i["t"] - 1.8) < 0.01 and abs(i["opening"] - 8.57) < 0.02
    assert abs(C.c_clip(12).volume() - 170.0) < 3.0     # v10 12_clip.stl is 170.0 mm3
    assert i["strain_ok"]


def test_groove_cutter():
    shaft = prim.cyl(3, 20)
    g = C.groove_dims(6)
    s = shaft - C.clip_groove_cutter(6, z=10)
    ok(s)
    assert abs((shaft.volume() - s.volume()) -
               3.14159 * (9 - (g["core_d"] / 2) ** 2) * g["width"]) < 0.5
