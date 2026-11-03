import math

from common import Params
from retrieve import approach, snatch, wrap_loads


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
