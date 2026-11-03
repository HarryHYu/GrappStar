import math

import pytest

from common import Params
import capstan
import drivetrain


def test_wiper_free_and_stall():
    p = Params()
    op = p.wiper.operate(0.0, p.battery, 12.0 + 1.5 * p.battery.r_source)
    assert op["rpm"] == pytest.approx(45.0, abs=0.01)    # free speed at 12 V terminal
    op = p.wiper.operate(20.0, p.battery, 12.6)
    assert op["stalled"] and op["current"] == pytest.approx(16.96, abs=0.05)
    assert op["current"] < p.battery.fuse_a


def test_screw_self_locking():
    eta, mu_e, tl = drivetrain.screw_eff(0.010, 0.004, 0.25, 15.0)
    assert 0.3 < eta < 0.4
    assert tl < mu_e


def test_cock_hand():
    p = Params()
    c = drivetrain.cock(p)
    L = p.launch
    e = L["preload_ext"] + L["stroke"] / L["pulley_ratio"]
    f_full = 2 * (p.spring.f_init + p.spring.k * e)
    assert c["f_spring_full"] == pytest.approx(f_full)
    fr = 0.30 * (L["carriage_mass"] + p.dart.mass) * 9.81 * math.cos(math.radians(p.flight["pitch_min"]))
    assert c["f_peak"] == pytest.approx(f_full + fr, abs=0.01)
    assert c["t_peak"] == pytest.approx(c["f_peak"] * 0.050 / (40 / 26 * 0.9))


def test_run():
    p = Params()
    p.capstan["path_gain"] = 1.15          # pulleys at the exit eye and ring
    r = drivetrain.run(p)
    f = r["functions"]
    t_m, _ = capstan.motor_stall_sag(p, p.battery.v_nom)
    for k in ("aim", "cock", "reel", "tow"):
        assert f[k]["frac_stall"] == pytest.approx(f[k]["t_peak"] / t_m)      # raw motor stall
        assert f[k]["frac_stall"] < drivetrain.STALL_FRAC_MAX
        assert f[k]["frac_limiter"] == pytest.approx(f[k]["t_peak"] / p.wiper.limiter_nm)
        assert f[k]["frac_limiter"] < drivetrain.LIMITER_FRAC_MAX
        assert f[k]["time_s"] > 0 and f[k]["energy_wh"] > 0
    assert not r["electrical"]["fuse_blows_on_stall"]
    assert r["electrical"]["stall_full"]["torque"] == pytest.approx(p.wiper.limiter_nm)
    assert f["tow"]["t_peak"] == pytest.approx(capstan.wiper_torque_for(50, p))
    # tow shaft at stall matches capstan.py
    cap = capstan.run(p)
    assert f["tow"]["shaft_torque_at_stall"] == pytest.approx(cap["parts_at_stall"]["t_drum"])
    for k in f:
        assert f[k]["gear_sf_trip"] > 1.0
    for k in f:
        # "(at stall)" checks run at the limiter torque
        assert f[k]["shaft_torque_at_stall"] < f[k]["shaft_torque_at_motor_stall"]
        over = f[k]["shaft_torque_at_stall"] > p.shaft.t_allow
        assert over == any("(at limiter (5 N m)) keyed shaft" in x for x in f[k]["flags"])


def test_run_no_limiter():
    p = Params()
    p.wiper.limiter_nm = None
    f = drivetrain.run(p)["functions"]
    for k in f:
        assert f[k]["frac_limiter"] is None and f[k]["info"] == []
        assert f[k]["shaft_torque_at_stall"] == pytest.approx(f[k]["shaft_torque_at_motor_stall"])
        over = f[k]["shaft_torque_at_stall"] > p.shaft.t_allow
        assert over == any("(motor stall (no limiter)) keyed shaft" in x for x in f[k]["flags"])
