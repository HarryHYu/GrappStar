import math

from common import Params
from recoil import ROVER, shot_loads, tip_dynamic


def test_tip_dynamic_energy():
    r = dict(ROVER)
    t = tip_dynamic(r, 0.0, 0.15)
    assert not t["tips"] and t["lift_deg"] == 0.0
    big = tip_dynamic(r, 10.0, 0.15)
    assert big["tips"]
    assert big["tip_energy_j"] > 0


def test_recoil_force_is_full_draw_force():
    p = Params()
    s = shot_loads(p, 36.0)
    assert abs(s["f_recoil_peak_n"] - 2 * p.spring.force(p.launch["preload_ext"] + p.launch["stroke"])) < 1e-6
    assert s["f_recoil_x_n"] > 0 and s["f_recoil_down_n"] > 0
    # lower pitch -> more horizontal -> more backward tipping moment
    assert shot_loads(p, 6.0)["moment_back_tip_nm"] > s["moment_back_tip_nm"]
