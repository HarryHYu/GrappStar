import math

import pytest

from common import ExtSpring, Params
from launch import launch


def bare():
    p = Params()
    p.launch.update(stroke=0.25, mu_rail=0.0, pitch_deg=0.0, n_pulley_turns=0)
    p.flight["brake_n"] = 0.0
    return p


def test_spring_geometry_matches_catalogue():
    s = ExtSpring()
    assert 95 < s.n_body < 101
    assert s.rate_formula == pytest.approx(0.125, rel=0.06)   # catalogue estimate
    assert 0.045 < s.mass < 0.065


def test_spring_limits_hand_calc():
    s = ExtSpring(f_init=7.0)
    lim = s.limits()
    assert lim["sut_mpa"] == pytest.approx(2211 / 1.4 ** 0.145, rel=1e-6)
    # body: Kb 8 F D / (pi d^3); C = 10.43, Kb = 1.129
    C = 14.6 / 1.4
    kb = (4 * C + 2) / (4 * C - 3)
    tau1 = kb * 8 * 14.6 / (math.pi * 1.4 ** 3)          # MPa per N
    assert lim["f_body_n"] == pytest.approx(0.45 * lim["sut_mpa"] / tau1 / 1.2, rel=1e-6)
    assert lim["governed_by"] in ("hook torsion", "hook bending")
    assert 250 < lim["ext_safe_mm"] < 320


def test_energy_conservation_no_losses():
    p = bare()
    p.launch["carriage_mass"] = 0.0
    p.launch["stop_pad_travel"] = 0.004
    p.spring.wire.rho = 1.0                               # (almost) massless spring
    r = launch(p, 0.0, model="lumped", line_load=(0.0, 0.0))
    assert r["ke_dart_j"] == pytest.approx(r["energy_stored_j"], rel=0.01)


def test_lumped_and_chain_agree():
    p = Params()
    a = launch(p, 35, model="lumped", stroke=0.24)
    b = launch(p, 35, model="chain", stroke=0.24)
    assert a["v_muzzle"] == pytest.approx(b["v_muzzle"], rel=0.03)


def test_pulley_ratio_keeps_energy_and_halves_force():
    p = Params()
    a = launch(p, 35, stroke=0.20, pulley_ratio=1.0, model="lumped")
    b = launch(p, 35, stroke=0.40, pulley_ratio=2.0, model="lumped")
    assert a["energy_stored_j"] == pytest.approx(b["energy_stored_j"], rel=1e-9)
    assert b["f_carriage_full_n"] == pytest.approx(a["f_carriage_full_n"] / 2, rel=1e-9)


def test_more_pitch_less_speed():
    p = Params()
    assert launch(p, 10, stroke=0.24)["v_muzzle"] > launch(p, 60, stroke=0.24)["v_muzzle"]


def test_flags_over_safe_extension():
    p = Params()
    assert launch(p, 35, stroke=0.40)["over_safe"]
    assert not launch(p, 35, stroke=0.20)["over_safe"]
