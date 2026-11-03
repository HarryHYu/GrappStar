import os
import sys

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

import math

import manifold3d as m3
import numpy as np

from cad.lib import cams

WP = [(0, 6), (70, 6), (130, 14), (200, 14), (260, 22), (320, 22)]
R, LEN, PIN, CLR, DEPTH = 20.0, 30.0, 6.0, 0.4, 3.0


def test_track():
    f = cams.track_fn(WP)
    assert f(0) == 6 and f(100) == 10 and f(165) == 14 and f(320) == 22
    assert abs(f(380) - f(20)) < 1e-12
    assert abs(cams.track_z(WP, 290) - 22) < 1e-12
    z = f(np.linspace(0, 360, 721))
    assert np.all(np.isfinite(z)) and z.min() >= 6 - 1e-9 and z.max() <= 22 + 1e-9
    o = cams.track_fn([(0, 0), (90, 10)], closed=False)
    assert o(-10) == 0 and o(200) == 10


def test_info():
    inf = cams.barrel_info(R, WP, DEPTH)
    rp = R - DEPTH / 2
    # cycloidal peak slope = 2 * dz / dtheta for the steepest move (return: 16 mm over 40 deg)
    expect = math.degrees(math.atan(2 * 16 / (rp * math.radians(40))))
    assert abs(inf["max_pressure_angle"] - expect) < 0.5
    assert inf["z_min"] == 6 and inf["z_max"] == 22


def test_drum_and_pin():
    drum = cams.barrel_cam(R, LEN, WP, PIN + CLR, DEPTH, bore=8, n_samples=240)
    assert drum.status() == m3.Error.NoError
    assert len(drum.decompose()) == 1
    full = math.pi * (R * R - 16) * LEN
    assert 0.6 * full < drum.volume() < full
    f = cams.track_fn(WP)
    for a in range(0, 360, 11):
        pin = cams.follower_pin(R, DEPTH, PIN, a, f(a))
        assert (pin ^ drum).volume() < 1e-6, a
    # a pin off the track hits the wall
    assert (cams.follower_pin(R, DEPTH, PIN, 35, f(35) + 0.6) ^ drum).volume() > 1e-3


def test_open_track():
    tr = cams.track_fn([(0, 5), (60, 5), (150, 15)], closed=False)
    d = cams.barrel_cam(R, LEN, tr, PIN + CLR, DEPTH, n_samples=120)
    assert d.status() == m3.Error.NoError and len(d.decompose()) == 1
