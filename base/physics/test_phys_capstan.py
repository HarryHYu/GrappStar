import math

import pytest

from common import Params
import capstan
import printed_gears as gears
import payout
from common import PPA_CF


def _no_limiter():
    p = Params()
    p.wiper.limiter_nm = None
    return p


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
    p = _no_limiter()                            # raw motor stall physics
    t, i = capstan.stall_torque_sag(p, 12.6)
    # 12.6 / (0.667 + 0.076) = 16.96 A
    assert i == pytest.approx(16.96, abs=0.05)
    assert t == pytest.approx(15 / 16.5 * (i - 1.5), rel=1e-6)
    assert capstan.stall_label(p) == "motor stall (no limiter)"


def test_stall_at_limiter():
    p = Params()
    lim = p.wiper.limiter_nm
    assert lim is not None
    t, i = capstan.stall_torque_sag(p, 12.6)
    assert t == pytest.approx(lim)
    assert i == pytest.approx(p.wiper.i_free + lim / p.wiper.kt)       # 7.0 A at 5 N m
    t_m, i_m = capstan.motor_stall_sag(p, 12.6)
    assert i_m == pytest.approx(16.96, abs=0.05) and t_m > lim
    assert capstan.stall_label(p) == f"at limiter ({lim:g} N m)"


def test_takeup_constant_torque():
    p = Params()
    rc = p.reel["core_d"] / 2
    tk = p.capstan["takeup_n"]
    r20 = payout.line_radius(p, 20.0)
    assert capstan.takeup_tension(p, 20.0) == pytest.approx(tk)          # set with the reel full
    assert capstan.takeup_tension(p, 0.0) == pytest.approx(tk * r20 / rc)
    tu = capstan.takeup_range(p)
    assert tu["t_hold_min"] == pytest.approx(tk) and tu["t_hold_max"] > tk
    k, rcap = p.capstan["ratio"], p.capstan["d"] / 2
    assert tu["overspeed_min"] == pytest.approx(capstan.TAKEUP_OVERSPEED * k * rc / rcap)   # reel vs capstan line speed
    assert tu["overspeed_max"] == pytest.approx(capstan.TAKEUP_OVERSPEED * k * r20 / rcap)


def test_tow_roundtrip_and_numbers():
    p = Params()
    c = p.capstan
    r, k, eta, tk = c["d"] / 2, c["ratio"], c["eta"], c["takeup_n"]
    th = capstan.takeup_tension(p, 20.0)
    t = capstan.wiper_torque_for(50, p)          # full reel
    # r (F - Th(20 m) + overspeed x takeup_n) / (ratio eta)
    assert t == pytest.approx(r * (50 - th) / (k * eta) + capstan.clutch_torque(p) * capstan.TAKEUP_OVERSPEED / capstan.ETA_BUS)
    assert capstan.line_from_wiper(t, p, line_left=20.0) == pytest.approx(50.0)
    pt = capstan.tow_point(50, p, 11.1)
    # drum turns 1/ratio per wiper turn: v_line = wiper speed / ratio x r
    assert pt["v_line"] == pytest.approx(pt["rpm_wiper"] * 2 * math.pi / 60 / k * r)
    assert 0.05 < pt["v_line"] < 0.2
    assert pt["current"] == pytest.approx(p.wiper.i_free + t / p.wiper.kt)
    assert pt["t_drum"] == pytest.approx(capstan.drum_shaft_torque(t, p))
    assert t < 0.8 * p.wiper.limiter_nm          # tow does not slip the limiter


def test_lame_hoop():
    p = Params()
    t = capstan.wiper_torque_for(50, p)
    ps = capstan.part_stresses(50, t, p, 5, 40, 2.0)
    pe = 50 / (0.02 * 1.05e-3)
    assert ps["hoop_mpa"] == pytest.approx(2 * pe * 0.02 ** 2 / (0.02 ** 2 - 0.01 ** 2) / 1e6)   # 6.35 MPa


def test_fw_trip():
    p = Params()
    ft = capstan.fw_trip(p)
    assert ft["hard_torque"] == pytest.approx(15 / 16.5 * (capstan.TRIP_REEL_A - 1.5))    # REEL hard trip torque
    g = capstan.CAP_GEAR
    m = gears.mesh(ft["hard_torque"], p.capstan["ratio"], g["module"], g["z_driver"], g["face"], PPA_CF, 0.0)
    assert ft["hard_gear_pass"] and ft["hard_gear_sf"] == pytest.approx(m["sf"])
    # the capstan gear also survives the limiter torque
    assert gears.mesh(p.wiper.limiter_nm, p.capstan["ratio"], g["module"], g["z_driver"], g["face"], PPA_CF)["pass"]
    assert ft["blank_ms"] == 50 and ft["hard_ms"] == 30 and ft["tow_trip_avg_ms"] == 200


def test_run_flags():
    p = Params()
    r = capstan.run(p)
    lim = p.wiper.limiter_nm
    assert r["stall"]["full"]["t_wiper"] == pytest.approx(lim)
    assert r["stall"]["full"]["f_line"] == pytest.approx(capstan.line_from_wiper(lim, p))
    assert r["stall"]["full"]["f_line"] > 45 and r["limiter"]["f_line_empty_reel"] < p.line.breaking
    assert any("at limiter" in w and "dart holding" in w for w in r["warnings"])
    assert r["motor_stall"]["full"]["f_line"] > r["stall"]["full"]["f_line"]
    assert any("motor stall (no limiter)" in x for x in r["info"])
    assert not r["limiter"]["slips_at_tow"]
    t50 = capstan.wiper_torque_for(50, p, line_left=0.0)
    assert r["current_limit"]["current"] == pytest.approx(p.wiper.i_free + t50 / p.wiper.kt)
    assert r["max_hold_n"]["mu_low_worst"] < r["max_hold_n"]["mu_low_best"]


def test_run_no_limiter():
    p = _no_limiter()
    r = capstan.run(p)
    assert r["limiter"] is None and r["info"] == []
    assert r["stall"]["full"]["current"] == pytest.approx(16.96, abs=0.05)
    assert any("motor stall (no limiter)" in w and "dart holding" in w for w in r["warnings"])
