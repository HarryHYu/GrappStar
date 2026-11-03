"""Tests on the REAL spec (base/spec/params.py through spec_bridge), not Params() defaults. They catch mapping
errors (units, ratio directions) and a stale committed ballistics table."""
import json
import math
import os

import pytest

import capstan
from common import load_params
from flight import simulate
from launcher import launch

HERE = os.path.dirname(os.path.abspath(__file__))
SPEC = os.path.join(HERE, "..", "spec", "params.py")
BALL = os.path.join(HERE, "..", "out", "ballistics.json")
META = os.path.join(HERE, "..", "out", "ballistics_meta.json")

pytestmark = pytest.mark.skipif(not os.path.exists(SPEC), reason="no base/spec/params.py")


@pytest.fixture(scope="module")
def p():
    return load_params()


def _spec():
    import spec_bridge
    return spec_bridge.load_spec_module()


def test_spec_loaded(p):
    assert "params.py" in p.source


def test_capstan_ratio_direction(p):
    s = _spec()
    # spec: capstan turns per main shaft turn; physics: wiper turns per capstan turn
    assert p.capstan["ratio"] == pytest.approx(1.0 / s.REEL["capstan_ratio"])
    # free line speed: 45 rpm x 2.27 x pi x 0.04 = 0.21 m/s (spec REEL.line_speed_mps.free)
    v_free = p.wiper.n0_rpm / 60 / p.capstan["ratio"] * math.pi * p.capstan["d"]
    assert v_free == pytest.approx(s.REEL["line_speed_mps"]["free"], rel=0.05)


def test_tow_50n_absolute(p):
    t = capstan.wiper_torque_for(50.0, p)
    # 50 N on a 20 mm drum through 0.44 / 0.85 is 2.6 N m, plus the FIRM take-up drag through the bus
    assert 2.3 < t < 3.8
    pt = capstan.tow_point(50.0, p, p.battery.v_nom)
    assert 0.10 < pt["v_line"] < 0.20
    assert 3.5 < pt["current"] < 6.0


def test_spring_geometry_from_spec(p):
    s = _spec()
    L = p.launch
    assert L["preload_ext"] * 1000 == pytest.approx(s.LAUNCH["spring_preload"])
    assert L["stroke"] * 1000 == pytest.approx(s.LAUNCH["stroke"])
    full = (L["preload_ext"] + L["stroke"]) * 1000
    assert full == pytest.approx(s.SPRING["max_ext"], abs=0.5)
    assert p.spring.wire_mm == pytest.approx(s.SPRING["wire"])


def test_reel_geometry_from_spec(p):
    st = _spec().REEL["storage"]
    assert p.reel["core_d"] * 1000 == pytest.approx(st["core_d"])
    assert p.reel["flange_d"] * 1000 == pytest.approx(st["flange_d"])
    assert p.reel["width"] * 1000 == pytest.approx(st["width"])


def test_headline_launch_band(p):
    r = launch(p, 36.0)
    assert not r["over_safe"]
    assert 6.5 < r["v_muzzle"] < 7.8


def test_committed_ballistics_not_stale(p):
    meta = json.load(open(META))
    assert meta["params_source"] == p.source, "ballistics.json was built from other params: run python ballistics.py"
    doc = json.load(open(BALL))
    row = min(doc["table"], key=lambda r: abs(r["pitch_deg"] - 36.0))
    v = launch(p, row["pitch_deg"])["v_muzzle"] * meta.get("v_scale", 1.0)
    assert row["v_muzzle"] == pytest.approx(v, rel=0.005)
    rng = simulate(p, v, row["pitch_deg"], brake_n=doc["drag_n"][1], reel_model=True)["range_m"]
    assert row["range_m"][1] == pytest.approx(rng, rel=0.01)
