import math

import pytest

from common import Params
from flight import simulate


def test_vacuum_matches_analytic():
    p = Params()
    p.launch["muzzle_height"] = 0.0
    r = simulate(p, 10.0, 45.0, tether=False, cda=0.0, muzzle_xz=(0.0, 0.0), dt=1e-4)
    assert r["range_m"] == pytest.approx(100 / 9.81, rel=0.005)
    assert r["apex_m"] == pytest.approx(25 / 9.81, rel=0.005)
    assert r["t_flight_s"] == pytest.approx(2 * 10 * math.sin(math.radians(45)) / 9.81, rel=0.005)


def test_drag_and_tether_cut_range():
    p = Params()
    a = simulate(p, 9.0, 35, tether=False, cda=0.0)["range_m"]
    b = simulate(p, 9.0, 35, tether=False)["range_m"]
    c = simulate(p, 9.0, 35, brake_n=0.5)["range_m"]
    d = simulate(p, 9.0, 35, brake_n=1.0)["range_m"]
    assert a > b > c > d


def test_tether_close_to_point_mass_with_constant_pull():
    """The bead chain should land near a point mass pulled by brake x eye gain (line weight and drag make it shorter)."""
    p = Params()
    v, th = 9.0, 35.0
    T = 0.75 * p.flight["eye_gain"]
    x, z = 0.10, p.launch["muzzle_height"]
    vx, vz = v * math.cos(math.radians(th)), v * math.sin(math.radians(th))
    dt = 1e-4
    while z > 0:
        s = math.hypot(vx, vz)
        F = 0.5 * 1.2 * p.dart.cda * s * s + T
        vx -= F * vx / s / p.dart.mass * dt
        vz -= (9.81 + F * vz / s / p.dart.mass) * dt
        x += vx * dt
        z += vz * dt
    pm = x - 0.10
    chain = simulate(p, v, th, brake_n=0.75)["range_m"]
    assert 0.85 * pm < chain < 1.02 * pm


def test_discretisation_converged():
    p = Params()
    a = simulate(p, 9.0, 35, seg=0.4, dt=1e-4)["range_m"]
    b = simulate(p, 9.0, 35, seg=0.8, dt=3e-4)["range_m"]
    assert a == pytest.approx(b, rel=0.01)


def test_reel_model_runs_and_reports():
    p = Params()
    r = simulate(p, 8.5, 35, brake_n=0.75, reel_model=True)
    assert r["landed"]
    assert r["reel_rpm_max"] < p.bearing.n_max_rpm_2rs
    assert 0 <= r["overrun_after_landing_m"] < 2.0
