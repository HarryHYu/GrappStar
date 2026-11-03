import numpy as np
import pytest

from attitude import mass_props, simulate


def test_mass_props_match_kedge():
    # kedge/out30/flight_sim.json inputs (15 g ballast): 0.279 kg, CG x 79.1 mm, Iyy 3.398e-4
    m, cgx, iyy = mass_props(15.0)
    assert m == pytest.approx(0.279, abs=0.001)
    assert cgx * 1000 == pytest.approx(79.1, abs=0.1)
    assert iyy == pytest.approx(3.398e-4, rel=0.01)


@pytest.mark.parametrize("kick", [0.0, 10.0])
def test_ballast_15g_no_tether_18ms_stable(kick):
    # kedge setup: 45 deg from z = 0, CG ground rule; kedge reports no tumble, max |alpha| after 0.6 s 18.1 deg
    r = simulate(18.0, 45.0, 0.0, kick, 0.0, 15.0, z0=0.0, ground_ends=False)
    assert not r["tumbled"][0]
    late = r["trace_alpha"][r["trace_t"] > 0.6, 0]
    assert np.nanmax(np.abs(late)) < 25.0
    if kick == 10.0:
        assert r["impact"]["x"][0] == pytest.approx(29.4, abs=0.5)


def test_no_ballast_no_tether_18ms_tumbles():
    # kedge "no tether" case: tumbles, range 27.7 m
    r = simulate(18.0, 45.0, 0.0, 10.0, 0.0, 0.0, z0=0.0, ground_ends=False)
    assert r["tumbled"][0]
    assert r["impact"]["x"][0] == pytest.approx(27.7, abs=0.5)


def test_free_body_keeps_pitch_rate():
    # no aero, no tether: torque free, theta-dot constant and theta linear in time
    w0 = 3.0
    r = simulate(7.0, 36.0, 0.0, 0.0, w0, 15.0, aero=False, ground_ends=False, z0=0.35)
    assert r["w_end"][0] == pytest.approx(w0, abs=1e-12)
    t_end = r["impact"]["t"][0]
    assert r["impact"]["theta"][0] == pytest.approx(36.0 + np.degrees(w0 * t_end), abs=1e-6)


def test_tether_brings_nose_first_at_base_speed():
    # base station shot: 7 m/s at 36 deg, 0.5 N brake, nose-up kick and rate disturbance
    r = simulate(7.0, 36.0, 0.5, 10.0, 5.0, 15.0)
    assert not r["tumbled"][0]
    assert r["nose_first"][0]
