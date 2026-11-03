"""Physics sanity tests for reel_calc (storage reel, capstan, clutch, gear train, cup).

Run: cd base && ../kedge/.venv/bin/python -m pytest -q cad/parts/test_reel_calc.py
"""
import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from cad.parts import reel_calc as rc  # noqa: E402


@pytest.fixture
def d():
    return rc.get()


# ------------------------------------------------------------------------------------------------ capstan
def test_capstan_ratio_mu_zero_is_one():
    for wraps in (0, 1, 8, 20):
        assert rc.capstan_ratio(0.0, wraps) == pytest.approx(1.0)
        assert rc.capstan_ratio(0.0, wraps, extra_deg=90.0) == pytest.approx(1.0)


def test_capstan_ratio_zero_wrap_is_one():
    assert rc.capstan_ratio(0.2, 0) == pytest.approx(1.0)


def test_capstan_ratio_monotonic_in_wraps_and_mu():
    r = [rc.capstan_ratio(0.15, n) for n in range(0, 12)]
    assert np.all(np.diff(r) > 0)
    m = [rc.capstan_ratio(mu, 8) for mu in np.linspace(0.0, 0.3, 10)]
    assert np.all(np.diff(m) > 0)


def test_capstan_ratio_matches_euler_eytelwein():
    mu, n, extra = 0.12, 3, 45.0
    theta = 2 * math.pi * n + math.pi / 4
    assert rc.capstan_ratio(mu, n, extra) == pytest.approx(math.exp(mu * theta))
    # multiplicative in wrap angle: e^(mu(a+b)) = e^(mu a) e^(mu b)
    assert rc.capstan_ratio(mu, 5) == pytest.approx(rc.capstan_ratio(mu, 2) * rc.capstan_ratio(mu, 3))
    # 360 extra degrees is one more wrap
    assert rc.capstan_ratio(mu, 2, 360.0) == pytest.approx(rc.capstan_ratio(mu, 3))


@pytest.mark.parametrize("t_load,t_tail,mu,margin", [
    (100, 2.6, 0.10, 2.0), (100, 0.5, 0.15, 2.0), (300, 3.0, 0.10, 1.5), (50, 1.0, 0.25, 3.0), (667, 2.6, 0.12, 2.0),
])
def test_wraps_needed_achieves_margin_and_one_less_does_not(t_load, t_tail, mu, margin):
    n = rc.wraps_needed(t_load, t_tail, mu, margin)
    assert isinstance(n, int) and n >= 1
    assert t_tail * rc.capstan_ratio(mu, n) >= margin * t_load * (1 - 1e-12)
    if n > 1:
        assert t_tail * rc.capstan_ratio(mu, n - 1) < margin * t_load


def test_wraps_needed_trivial_load_gives_one():
    assert rc.wraps_needed(1.0, 10.0, 0.1) == 1


def test_design_wraps_hold_100n_with_reel_tail(d):
    cl = rc.clutch(d)
    tail = cl["reel"]["line_n_full"]
    assert tail * rc.capstan_ratio(d["mu_min"], d["wraps"]) > 2 * d["tow_limit_n"]
    assert rc.wraps_needed(d["tow_limit_n"], tail, d["mu_min"]) <= d["wraps"]


def test_wrap_line_mm(d):
    assert rc.line_r_cap(d) == pytest.approx(d["cap_r"] + d["line_d"] / 2)
    assert rc.wrap_line_mm(d, 1) == pytest.approx(2 * math.pi * rc.line_r_cap(d))
    assert rc.wrap_line_mm(d) == pytest.approx(d["wraps"] * rc.wrap_line_mm(d, 1))


# ------------------------------------------------------------------------------------------------ storage reel
def test_pack_r_empty_is_core(d):
    assert rc.pack_r(d, 0.0) == pytest.approx(d["core_r"])


def test_full_pack_below_flange(d):
    r = rc.pack_r(d, d["line_len_m"])
    assert d["core_r"] < r < d["flange_r"]


def test_pack_r_monotonic_and_volume_conserving(d):
    s = np.linspace(0, d["line_len_m"], 21)
    r = np.array([rc.pack_r(d, x) for x in s])
    assert np.all(np.diff(r) > 0)
    # annulus volume equals line volume / fill
    for x, rr in zip(s, r):
        vol_pack = math.pi * (rr ** 2 - d["core_r"] ** 2) * d["width"]
        assert vol_pack == pytest.approx(x * 1000 * d["line_d"] ** 2 / d["pack_fill"], rel=1e-9, abs=1e-9)


def test_reel_inertia_increases_with_stored_line(d):
    s = np.linspace(0, d["line_len_m"], 11)
    I = np.array([rc.reel_inertia(d, x) for x in s])
    assert np.all(np.diff(I) > 0)
    assert rc.reel_inertia(d) == pytest.approx(rc.reel_inertia(d, d["line_len_m"]))
    # empty reel is just structure + races
    assert rc.reel_inertia(d, 0.0) > 9.0e-6
    assert rc.reel_inertia(d, 0.0, structure_kgm2=0.0) == pytest.approx(rc.reel_inertia(d, 0.0) - 9.0e-6)


def test_spinup_rpm_matches_v_over_r(d):
    s = rc.spinup(d)
    R = rc.pack_r(d, d["line_len_m"]) / 1000
    assert s["omega"] == pytest.approx(d["v_launch"] / R)
    assert s["rpm"] == pytest.approx(d["v_launch"] / R * 60 / (2 * math.pi))
    assert s["accel"] == pytest.approx(d["v_launch"] ** 2 / (2 * d["stroke_m"]))
    I = rc.reel_inertia(d)
    assert s["energy_j"] == pytest.approx(0.5 * I * s["omega"] ** 2)
    assert s["pull_n"] > d["brake_launch_n"]
    # energy check: work by the inertial pull over the stroke equals spin energy
    assert (s["pull_n"] - d["brake_launch_n"]) * d["stroke_m"] == pytest.approx(s["energy_j"], rel=1e-9)


def test_overrun_slack_shrinks_with_brake(d):
    brakes = [0.25, 0.5, 1.0, 2.0, 6.0]
    slack = [rc.overrun(d, b, 8.0)["slack_m"] for b in brakes]
    assert np.all(np.diff(slack) < 0)
    # inverse proportional to brake
    assert rc.overrun(d, 1.0, 8.0)["slack_m"] == pytest.approx(2 * rc.overrun(d, 2.0, 8.0)["slack_m"])
    # energy balance: brake work equals spin energy
    o = rc.overrun(d, 1.0, 8.0)
    stored = d["line_len_m"] - 8.0
    I = rc.reel_inertia(d, stored)
    R = rc.pack_r(d, stored) / 1000
    assert 1.0 * o["slack_m"] == pytest.approx(0.5 * I * (8.0 / R) ** 2)


def test_overrun_grows_with_speed(d):
    assert rc.overrun(d, 1.0, 8.0)["slack_m"] > rc.overrun(d, 1.0, 4.0)["slack_m"]


def test_min_brake_no_overrun_below_launch_brake(d):
    b = rc.min_brake_no_overrun(d)
    assert 0 < b < d["brake_launch_n"]


# ------------------------------------------------------------------------------------------------ slip clutch
def test_clutch_preloads_ordered(d):
    cl = rc.clutch(d)
    assert 0 < cl["launch"]["preload_n"] < cl["mid"]["preload_n"] < cl["reel"]["preload_n"]
    assert cl["launch"]["torque_nm"] < cl["mid"]["torque_nm"] < cl["reel"]["torque_nm"]


def test_clutch_push_equals_preload_times_lever(d):
    cl = rc.clutch(d)
    lever = d["rocker_l"] / (d["rocker_l"] + d["rocker_a"])
    assert cl["rocker_lever"] == pytest.approx(lever)
    assert 0 < lever < 1
    for k in ("launch", "mid", "reel"):
        assert cl[k]["push_n"] == pytest.approx(cl[k]["preload_n"] * lever)
        assert cl[k]["push_n"] < cl[k]["preload_n"]


def test_clutch_torque_and_line_tension_consistent(d):
    cl = rc.clutch(d)
    r_eff = 0.5 * (d["face_r_in"] + d["face_r_out"]) / 1000
    assert cl["r_mid_mm"] == pytest.approx(rc.pack_r(d, d["line_len_m"]))   # D24: targets with the reel FULL
    for k in ("launch", "mid", "reel"):
        c = cl[k]
        assert c["torque_nm"] == pytest.approx(d["mu_face"] * c["preload_n"] * r_eff)
        assert c["line_n_empty"] > c["line_n_full"]
        # D24: the brake target is met with the reel full; the core sees more
        assert c["line_n_full"] == pytest.approx(d[f"brake_{k}_n"])
    assert cl["spring_rate_n_mm"] > 0
    assert cl["precompress_mm"] > 0
    assert cl["launch"]["preload_n"] + cl["spring_rate_n_mm"] * d["brake_stroke"] == pytest.approx(
        cl["reel"]["preload_n"])


def test_rocker_stress_and_stiffness(d):
    rk = rc.rocker(d)
    assert 0 < rk["sigma_max"] < 30.0
    assert rk["t"] > 0
    assert rk["EI"] == pytest.approx(rk["E"] * rk["b"] * rk["t"] ** 3 / 12)
    # the beam actually produces the preload change over the stroke
    cl = rc.clutch(d)
    L, a = d["rocker_l"], d["rocker_a"]
    dP = (cl["reel"]["preload_n"] - cl["launch"]["preload_n"]) * L / (L + a)
    delta = dP * a ** 2 * (L + a) / (3 * rk["EI"])
    assert delta == pytest.approx(d["brake_stroke"])
    assert rk["precompress_input"] > 0


# ------------------------------------------------------------------------------------------------ gear train
def test_train_takeup_margin(d):
    tr = rc.train(d)
    assert tr["takeup_margin"] > 1.2
    assert tr["cap_per_main"] == pytest.approx(d["z_main"] / d["z_cap"])
    assert tr["reel_per_cap"] == pytest.approx((d["z_bus"] / d["z_reel"]) / (d["z_main"] / d["z_cap"]))
    assert tr["reel_per_wiper"] == pytest.approx(2.5)


def test_train_stall_tension_below_break(d):
    tr = rc.train(d)
    assert d["tow_limit_n"] < tr["line_n_at_stall"] < d["line_break_n"]


def test_train_center_distances(d):
    tr = rc.train(d)
    assert tr["c_main_cap"] == pytest.approx(0.5 * d["m_cap"] * (d["z_main"] + d["z_cap"]))
    assert tr["c_take_idler"] == pytest.approx(0.5 * d["m_take"] * (d["z_take"] + d["z_idler"]))
    assert tr["c_idler_reel"] == pytest.approx(0.5 * d["m_take"] * (d["z_idler"] + d["z_reel"]))


def test_train_wraps_from_drum_whole(d):
    w = rc.train(d)["wraps_from_drum"]
    assert w == pytest.approx(round(w))
    assert round(w) == d["wraps"]


def test_train_at_tension_physics(d):
    tr = rc.train(d)
    f = tr["at_fn"]
    Ts = sorted(tr["at"])
    rpm = [tr["at"][T]["rpm_main"] for T in Ts]
    amps = [tr["at"][T]["amps"] for T in Ts]
    assert np.all(np.diff(rpm) < 0)
    assert np.all(np.diff(amps) > 0)
    stall = f(tr["line_n_at_stall"])
    assert stall["rpm_main"] == pytest.approx(0.0, abs=1e-9)
    assert stall["amps"] == pytest.approx(d["wiper_i_stall"])
    assert stall["tau_main"] == pytest.approx(d["wiper_stall_nm"])
    zero = f(0.0)
    assert zero["rpm_main"] == pytest.approx(d["wiper_n0_rpm"])
    assert zero["amps"] == pytest.approx(d["wiper_i_free"])
    # line speed = capstan omega * line radius
    a = tr["at"][50.0]
    assert a["v_m_s"] == pytest.approx(a["rpm_cap"] * 2 * math.pi / 60 * rc.line_r_cap(d) / 1000)
    # beyond stall the motor does not run backwards in this model
    assert f(2 * tr["line_n_at_stall"])["rpm_main"] == 0.0


def test_amps_tension_inverse(d):
    for T in np.linspace(0, 300, 13):
        assert rc.tension_from_amps(d, rc.amps_for_tension(d, T)) == pytest.approx(T, abs=1e-9)
    for A in np.linspace(d["wiper_i_free"], d["wiper_i_stall"], 9):
        assert rc.amps_for_tension(d, rc.tension_from_amps(d, A)) == pytest.approx(A)
    assert rc.amps_for_tension(d, 50.0) == pytest.approx(rc.train(d)["at"][50.0]["amps"])
    assert rc.tension_from_amps(d, d["wiper_i_stall"]) == pytest.approx(rc.train(d)["line_n_at_stall"])
    # below free current the estimate clamps to zero, never negative
    assert rc.tension_from_amps(d, 0.0) == 0.0


# ------------------------------------------------------------------------------------------------ cup and unwrap
def test_unwrap_schedule_totals(d):
    steps = rc.unwrap_schedule(d)
    assert sum(s[0] for s in steps) == pytest.approx(d["wraps"])
    up = d["z_main"] / d["z_cap"]
    r = rc.line_r_cap(d)
    for cup, cap_deg, main_deg in steps:
        slack = cup * 2 * math.pi * r
        feed = math.radians(cap_deg) * r
        assert 0 < feed < slack
        assert main_deg == pytest.approx(cap_deg / up)


@pytest.mark.parametrize("step", [0.25, 0.5, 1.0, 2.0])
def test_unwrap_schedule_dividing_steps(d, step):
    steps = rc.unwrap_schedule(d, step_turns=step, feed=0.9)
    assert sum(s[0] for s in steps) == pytest.approx(d["wraps"])


def test_cup_torque_magnitude(d):
    for mu in (0.1, 0.2, 0.3):
        assert abs(rc.cup_torque(d, 100.0, mu)) < 1.0
    assert rc.cup_torque(d, 0.0) == pytest.approx(0.0)
    # linear in tension
    assert rc.cup_torque(d, 100.0) == pytest.approx(2 * rc.cup_torque(d, 50.0))
    # more groove friction leaves less tension on the out side, so holding torque grows in magnitude
    assert abs(rc.cup_torque(d, 100.0, 0.3)) > abs(rc.cup_torque(d, 100.0, 0.1))


# ------------------------------------------------------------------------------------------------ params overlay
def test_get_defaults_without_p():
    d = rc.get()
    assert d == rc.DEFAULTS
    assert d is not rc.DEFAULTS
    d["wraps"] = 99
    assert rc.DEFAULTS["wraps"] == 7


def test_get_overlays_fake_p():
    P = SimpleNamespace(
        REEL=dict(wraps=10, core_r=18.0, flange_r=None, not_a_key=1),
        LINE=dict(d=1.2, length_m=25.0, break_n=900.0),
        WIPER=dict(n0_rpm=50.0, stall_nm=20.0, stall_a=22.0, free_a=2.0),
    )
    d = rc.get(P)
    assert d["wraps"] == 10
    assert d["core_r"] == 18.0
    assert d["flange_r"] == rc.DEFAULTS["flange_r"]   # None does not override
    assert "not_a_key" not in d
    assert (d["line_d"], d["line_len_m"], d["line_break_n"]) == (1.2, 25.0, 900.0)
    assert (d["wiper_n0_rpm"], d["wiper_stall_nm"], d["wiper_i_stall"], d["wiper_i_free"]) == (50.0, 20.0, 22.0, 2.0)
    assert rc.DEFAULTS["wraps"] == 7


def test_get_partial_line_and_ignores_non_dicts():
    P = SimpleNamespace(REEL="nope", LINE=dict(d=0.8), WIPER=None)
    d = rc.get(P)
    assert d["line_d"] == 0.8
    assert d["line_len_m"] == rc.DEFAULTS["line_len_m"]
    assert d["wraps"] == rc.DEFAULTS["wraps"]
    assert rc.get(object()) == rc.DEFAULTS


def test_summary_runs():
    rows = rc.summary()
    assert len(rows) > 20
    assert all(len(r) == 3 for r in rows)


def test_unwrap_schedule_partial_last_step():
    d = rc.get()
    for st in (0.3, 0.7, 3.0):
        steps = rc.unwrap_schedule(d, st)
        assert abs(sum(s[0] for s in steps) - d["wraps"]) < 1e-9
        assert all(0 < s[0] <= st + 1e-12 for s in steps)
