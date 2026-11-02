import pytest

from common import Params
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
    # 2 x (7.5 + 125 x 0.31) = 92.5 N at full draw
    assert c["f_spring_full"] == pytest.approx(92.5)
    fr = 0.30 * (0.060 + 0.292) * 9.81 * 0.9659
    assert c["f_peak"] == pytest.approx(92.5 + fr, abs=0.01)
    assert c["t_peak"] == pytest.approx(c["f_peak"] * 0.015 / 0.9)


def test_run():
    r = drivetrain.run(Params())
    f = r["functions"]
    for k in ("aim", "cock", "reel", "tow"):
        assert f[k]["frac_stall"] < drivetrain.STALL_FRAC_MAX
        assert f[k]["time_s"] > 0 and f[k]["energy_wh"] > 0
    assert not r["electrical"]["fuse_blows_on_stall"]
    assert f["tow"]["t_peak"] == pytest.approx(0.02 * 50.4 / 0.85)
