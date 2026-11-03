import math

import pytest

from common import Params
import capstan


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


def test_tow_roundtrip_and_numbers():
    p = Params()
    t = capstan.wiper_torque_for(50, p)
    # r (F - Th + 1.2 Th) / (ratio eta) = 0.02 x 50.4 / 0.85
    assert t == pytest.approx(0.02 * 50.4 / 0.85)
    assert capstan.line_from_wiper(t, p) == pytest.approx(50.0)
    pt = capstan.tow_point(50, p, 11.1)
    assert 0.05 < pt["v_line"] < 0.1
    assert pt["current"] == pytest.approx(1.5 + t / (15 / 16.5))


def test_run_flags():
    r = capstan.run(Params())
    assert r["stall"]["full"]["f_line"] > 112
    assert any("dart holding" in w for w in r["warnings"])
    assert r["current_limit"]["current"] == pytest.approx(2.80, abs=0.02)
