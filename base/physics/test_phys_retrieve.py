import math

from common import Params
from retrieve import APPROACH, approach, snatch, takeup_speed, wrap_loads


def test_snatch_slower_than_taut():
    s = snatch(Params())
    assert s["v_after_snatch"] < s["v_taut"] < s["v_no_line_in_stroke"]
    assert s["line_spike_n_upper"] > 50


def test_wrap_cup_turns_and_monotonic():
    w = wrap_loads(Params())
    assert math.isclose(w["cup_turns_in_sector"], 7.0, rel_tol=1e-9)
    t = [r["t_n20_nm"] for r in w["rows"]]
    assert t[0] < t[1] < t[2]          # less line on the reel -> smaller radius -> more tension


def test_approach_need_hand():
    p = Params()
    a = approach(p)
    r = a["rows"][0]
    need = p.dart.mass * 9.81 * (math.sin(math.radians(5)) + 0.3 * math.cos(math.radians(5))) + 1.0
    assert math.isclose(r["need_n"], need, rel_tol=1e-9)


def test_takeup_speed_uses_eye_gain():
    p = Params()
    t = takeup_speed(p)
    from retrieve import seating_gain
    g = seating_gain(p)
    assert t["eye_gain"] == g
    for r in t["rows"]:
        assert math.isclose(r["drag_at_reel_n"], r["drag_n"] * g, rel_tol=1e-12)
        assert r["slips"] == (r["drag_n"] * g >= r["slip_tension_n"])


def test_seating_gain_cases():
    from retrieve import seating_gain
    p = Params()
    p.capstan["path_gain"] = 1.5
    assert seating_gain(p) == 1.75
    p.capstan["path_gain"] = 1.30                    # plain cup ring, pulleys at the guide and exit eye
    assert 1.15 < seating_gain(p) < 1.2
    p.capstan["path_gain"] = 1.15
    assert seating_gain(p) < 1.1
