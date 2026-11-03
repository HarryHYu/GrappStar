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
    assert p.capstan["ratio"] == pytest.approx(1.0 / s.REEL["capstan_ratio"]), "capstan ratio direction flipped in spec_bridge"
    # free line speed from the loaded WIPER.n0_rpm and REEL capstan: must match the spec's quoted REEL.line_speed_mps.free
    assert p.wiper.n0_rpm == pytest.approx(s.WIPER["n0_rpm"]), "WIPER.n0_rpm not mapped by spec_bridge"
    v_free = p.wiper.n0_rpm / 60 / p.capstan["ratio"] * math.pi * p.capstan["d"]
    quoted = s.REEL["line_speed_mps"]["free"]
    assert v_free == pytest.approx(quoted, rel=0.05), f"update REEL.line_speed_mps.free in the spec to {v_free:.2f}"


def test_tow_50n_absolute(p):
    """Tow at 50 N (and at the spec tow limit) with the loaded WIPER and REEL: lossless torque as a floor, the wiper's
    stall and the tow trip as ceilings, and the spec's quoted line speed."""
    s = _spec()
    r_cap = p.capstan["d"] / 2
    for f_tow in sorted({50.0, float(p.capstan["tow_limit_n"])}):
        t = capstan.wiper_torque_for(f_tow, p)
        t_ideal = f_tow * r_cap / p.capstan["ratio"]               # no path gain, no friction, no take-up
        assert t_ideal < t < 2.5 * t_ideal, f"tow torque {t:.2f} N m at {f_tow:g} N outside {t_ideal:.2f} to {2.5 * t_ideal:.2f}: check REEL mapping"
        lim = capstan.limiter_nm(p) or p.wiper.t_stall
        assert t < min(p.wiper.t_stall, lim), f"wiper too weak to tow {f_tow:g} N ({t:.2f} N m needed): see FINDINGS"
        pt = capstan.tow_point(f_tow, p, p.battery.v_nom)
        assert not pt["stalled"] and pt["v_line"] > 0, f"wiper too weak to tow {f_tow:g} N: see FINDINGS"
        trip = capstan.trip_a(p, "tow")
        assert pt["current"] < trip, (f"tow at {f_tow:g} N draws {pt['current']:.1f} A, over the {trip:g} A trip: "
                                      "wiper too weak (see FINDINGS) or raise REEL.current_trips_a.tow")
    pt = capstan.tow_point(50.0, p, p.battery.v_nom)
    quoted = (s.REEL.get("line_speed_mps") or {}).get("at_50n")
    if quoted is not None:
        assert pt["v_line"] == pytest.approx(quoted, rel=0.10), f"update REEL.line_speed_mps.at_50n in the spec to {pt['v_line']:.2f}"


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
    from common import params_hash
    stale = "run ./run_all.sh (regenerates the table, SUMMARY and the PDF)"
    assert meta.get("params_hash") == params_hash(p), f"ballistics.json was built from other physics inputs: {stale}"
    v_scale = meta.get("v_scale", 1.0)               # the table must use the spec LAUNCH v_scale (calib.py range)
    assert v_scale == pytest.approx(p.launch.get("v_scale", 1.0)), f"table v_scale {v_scale} != spec LAUNCH v_scale: {stale}"
    doc = json.load(open(BALL))
    row = min(doc["table"], key=lambda r: abs(r["pitch_deg"] - 36.0))
    from launcher import effective_muzzle
    v = effective_muzzle(p, row["pitch_deg"])["v"] * v_scale
    assert row["v_muzzle"] == pytest.approx(v, rel=0.005), stale
    rng = simulate(p, v, row["pitch_deg"], brake_n=doc["drag_n"][1], reel_model=True)["range_m"]
    assert row["range_m"][1] == pytest.approx(rng, rel=0.01), stale
    from ballistics import validate
    assert validate(doc) == []
    for r_ in (doc["table"][0], doc["table"][-1]):                # the ends too, all three drags
        v_ = effective_muzzle(p, r_["pitch_deg"])["v"] * v_scale
        for j, b in enumerate(doc["drag_n"]):
            got = simulate(p, v_, r_["pitch_deg"], brake_n=b, reel_model=True)["range_m"]
            assert r_["range_m"][j] == pytest.approx(got, rel=0.01), stale


def test_spec_launch_headline_matches_model(p):
    """BASE_SPEC section 11 / LAUNCH.flight quotes the range for the sim: keep it within 10 % of the model."""
    fl = (_spec().LAUNCH or {}).get("flight") or {}
    if fl.get("range_m") is None:
        pytest.skip("no LAUNCH.flight.range_m in the spec")
    from launcher import effective_muzzle
    th = fl.get("pitch_deg", 36.0)
    v = effective_muzzle(p, th)["v"]
    rng = simulate(p, v, th, brake_n=p.flight["brake_n"], reel_model=True)["range_m"]
    assert fl["range_m"] == pytest.approx(rng, rel=0.10), f"spec says {fl['range_m']} m, model {rng:.2f} m at {th} deg"


def test_wrap_time_from_spec(p):
    from retrieve import wrap_loads
    s = _spec()
    n = s.DRUM.get("n20") or {}
    if not n.get("drum_deg_per_s"):
        pytest.skip("spec has no DRUM.n20.drum_deg_per_s")
    w = wrap_loads(p)
    sector = w["params"]["drum_sector_deg"]
    if s.DRUM.get("cup_turns_per_drum_deg"):
        assert sector == pytest.approx(s.DRUM["wrap_turns"] / s.DRUM["cup_turns_per_drum_deg"])
    else:
        assert sector == pytest.approx(240.0)                     # WRAP window 62..302 deg
    assert w["wrap_time_s"] == pytest.approx(sector / n["drum_deg_per_s"])
    assert w["params"]["n20_ratio"] == pytest.approx(n["ratio"])
