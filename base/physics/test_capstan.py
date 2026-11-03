import math

import pytest

from common import Params
import capstan
import payout


def test_ratio_hand():
    assert capstan.ratio(0.15, 5) == pytest.approx(111.3, abs=0.1)


def test_wraps_needed():
    w = capstan.wraps_needed(50, 2, 0.15)
    assert w["raw"] == pytest.approx(math.log(25) / (2 * math.pi * 0.15))   # 3.42
    assert w["design"] == 6


def test_hold_and_takeup():
    assert capstan.max_hold(5, 2.0, 0.15) == pytest.approx(222.6, abs=0.2)
    assert capstan.takeup_needed(50, 5, 0.15) == pytest.approx(50 / 111.3, abs=0.01)
    assert capstan.slips(250, 5, 2.0, 0.15)
    assert not capstan.slips(200, 5, 2.0, 0.15)


def test_stall_current_with_sag():
    p = Params()
    t, i = capstan.stall_torque_sag(p, 12.6)
    # 12.6 / (0.667 + 0.076) = 16.96 A
    assert i == pytest.approx(16.96, abs=0.05)
    assert t == pytest.approx(15 / 16.5 * (i - 1.5), rel=1e-6)


def test_takeup_constant_torque():
    p = Params()
    rc = p.reel["core_d"] / 2
    assert capstan.takeup_tension(p, 0.0) == pytest.approx(2.0)
    r20 = payout.line_radius(p, 20.0)
    assert capstan.takeup_tension(p, 20.0) == pytest.approx(2.0 * rc / r20)
    tu = capstan.takeup_range(p)
    assert tu["t_hold_min"] < tu["t_hold_max"] == pytest.approx(2.0)
    assert tu["overspeed_min"] == pytest.approx(1.2)
    assert tu["overspeed_max"] == pytest.approx(1.2 * r20 / rc)


def test_tow_roundtrip_and_numbers():
    p = Params()
    th = capstan.takeup_tension(p, 20.0)
    t = capstan.wiper_torque_for(50, p)          # full reel
    # r (F - Th(20 m) + 1.2 x 2 N) / (ratio eta)
    assert t == pytest.approx(0.02 * (50 - th + 2.4) / 0.85)
    assert capstan.line_from_wiper(t, p, line_left=20.0) == pytest.approx(50.0)
    pt = capstan.tow_point(50, p, 11.1)
    assert 0.05 < pt["v_line"] < 0.1
    assert pt["current"] == pytest.approx(1.5 + t / (15 / 16.5))
    assert pt["t_drum"] == pytest.approx(t * 0.85)


def test_lame_hoop():
    p = Params()
    t = capstan.wiper_torque_for(50, p)
    ps = capstan.part_stresses(50, t, p, 5, 40, 2.0)
    pe = 50 / (0.02 * 1.05e-3)
    assert ps["hoop_mpa"] == pytest.approx(2 * pe * 0.02 ** 2 / (0.02 ** 2 - 0.01 ** 2) / 1e6)   # 6.35 MPa


def test_fw_trip():
    p = Params()
    ft = capstan.fw_trip(p)
    assert ft["hard_torque"] == pytest.approx(15 / 16.5 * 3.5)    # 3.18 N m
    assert ft["hard_gear_pass"] and ft["hard_gear_sf"] == pytest.approx(1.21, abs=0.01)
    assert ft["blank_ms"] == 50 and ft["hard_ms"] == 30 and ft["tow_trip_avg_ms"] == 200


def test_run_flags():
    r = capstan.run(Params())
    assert r["stall"]["full"]["f_line"] > 112
    assert any("dart holding" in w for w in r["warnings"])
    assert r["current_limit"]["current"] == pytest.approx(2.80, abs=0.02)
    assert r["max_hold_n"]["mu_low_worst"] < r["max_hold_n"]["mu_low_best"]
