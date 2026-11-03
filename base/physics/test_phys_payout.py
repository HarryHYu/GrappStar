import math

import pytest

from common import Params
from payout import Reel, line_radius, reel_props, spin_up_in_stroke


def test_line_radius_volume():
    p = Params()
    rc = p.reel["core_d"] / 2
    assert line_radius(p, 0.0) == pytest.approx(rc)
    r = line_radius(p, 20.0)
    vol = math.pi * (r ** 2 - rc ** 2) * p.reel["width"]
    assert vol == pytest.approx(20.0 * p.reel["line_pitch"] ** 2, rel=1e-9)


def test_capacity_and_inertia_sane():
    p = Params()
    rp = reel_props(p)
    assert rp["capacity_m"] > 20.0
    assert rp["flange_margin_mm"] > 2.0
    assert 1e-6 < rp["inertia_empty"] < 5e-5
    assert rp["inertia_full"] > rp["inertia_empty"]


def test_brake_tension_rises_as_reel_empties():
    p = Params()
    reel = Reel(p, 0.75)
    t_full = reel.brake_torque / reel.radius()
    reel.paid = p.reel["line_stored"]
    t_empty = reel.brake_torque / reel.radius()
    assert t_full == pytest.approx(0.75, rel=1e-9)
    assert t_empty > t_full


def test_reel_spins_down_on_brake():
    p = Params()
    reel = Reel(p, 0.75)
    reel.omega = 300.0
    for _ in range(100000):
        reel.step(0.0, 1e-4)
        if reel.omega == 0:
            break
    assert reel.omega == 0.0


def test_spin_up_load_small():
    p = Params()
    s = spin_up_in_stroke(p, 8.0, 0.25, 0.75)
    assert s["t_line_in_stroke_n"] < 8.0
