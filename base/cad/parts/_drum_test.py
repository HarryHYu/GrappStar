"""PC tests for the drum subsystem. Run from base/:  python -m pytest cad/parts/_drum_test.py -q"""
import math

import pytest

from cad.parts import _drum_kin as K

C = K.DEFAULTS


def test_travel_under_one_turn():
    assert C["stop_hi"] - C["stop_lo"] < 360.0


def test_codes_gray_and_valid():
    prev = None
    th = C["stop_lo"]
    seq = []
    while th <= C["stop_hi"]:
        code = K.code(th)
        assert code != 0, f"invalid code at {th}"
        if prev is not None and code != prev:
            assert bin(code ^ prev).count("1") == 1, f"non Gray step at {th}"
            seq.append(code)
        prev = code
        th += 0.1
    # each sector appears once (codes unique along the travel)
    assert len(seq) == len(set(seq))


def test_named_positions():
    assert K.sector(C["launch"]) == "LAUNCH"
    assert K.sector(C["neutral"]) == "NEUTRAL"
    assert K.sector(C["wrapped"]) == "WRAPPED"
    assert K.sector(C["reel"]) == "REEL"
    assert K.mode(C["launch"]) == "LAUNCH"
    assert K.mode(C["neutral"]) == "NEUTRAL"
    assert K.mode(C["wrapped"]) == "NEUTRAL"
    assert K.mode(C["reel"]) == "REEL"


def test_code_inside_groove_dwells():
    """Whenever the code names a dwell the collar is flat in it (no partial engagement)."""
    th = C["stop_lo"]
    while th <= C["stop_hi"]:
        s = K.sector(th)
        x = K.collar_x(th)
        if s == "LAUNCH":
            assert x == -C["stroke"]
        if s in ("NEUTRAL", "WRAPPING", "WRAPPED"):
            assert x == 0.0
        if s == "REEL":
            assert x == C["stroke"]
        th += 0.1


def test_wraps_only_in_neutral_and_in_order():
    """D16: the cup turns only while the collar is in NEUTRAL; REEL only with all wraps on; LAUNCH with none."""
    th = C["stop_lo"]
    full = K.wraps_total()
    while th <= C["stop_hi"]:
        w = K.wraps(th)
        x = K.collar_x(th)
        if 1e-9 < w < full - 1e-9:
            assert x == 0.0, f"cup turning with the collar off NEUTRAL at {th}"
        if x > 0:
            assert w == pytest.approx(full)
        if x < 0:
            assert w == 0.0
        th += 0.25


def test_wraps_integer_and_lock_phase():
    assert K.wraps_total() == pytest.approx(7.0)
    assert abs(K.wrap_turns_pinion() - round(K.wrap_turns_pinion())) < 1e-9
    assert K.wraps(C["wrapped"]) == pytest.approx(K.wraps_total())
    assert K.wraps(C["neutral"]) == 0.0


def test_brake_levels():
    assert K.brake_frac(C["launch"]) == 0.0
    assert K.brake_frac(C["neutral"]) == pytest.approx(1.0)
    assert K.brake_frac(180.0) == 0.0
    assert K.brake_frac(C["wrapped"]) == pytest.approx(1.0)
    assert K.brake_frac(C["reel"]) == pytest.approx(1.0)


def test_code_lift_matches_code():
    th = C["stop_lo"]
    while th <= C["stop_hi"]:
        for b in range(3):
            if K.code_lift(th, b) not in (0.5,):
                assert (K.code_lift(th, b) > 0.5) == bool(K.code(th) >> b & 1), (th, b)
        th += 0.1


def test_stops_clear_of_ramps():
    """At both hard stops every band is fully up or fully down (no reading on a ramp)."""
    for th in (C["stop_lo"], C["stop_hi"]):
        for b in range(3):
            assert K.code_lift(th, b) in (0.0, 1.0), (th, b, K.code_lift(th, b))


def test_stop_plan_extra_inside_dwell():
    plans = K.stop_plan()
    assert len(plans) == 6
    for p in plans:
        tgt = C[p["target"].lower()]
        assert p["edge"] + (-p["extra_deg"] if p["dir"] < 0 else p["extra_deg"]) == pytest.approx(tgt)
        # 30 percent rate error must still land with the same code as the target
        for err in (0.7, 1.3):
            th = p["edge"] + p["dir"] * p["extra_deg"] * err
            assert K.sector(th) == K.sector(tgt), (p, err)


def test_torque_budget():
    L = K.loads()
    assert L["worst"] <= L["cap_stall"] * 0.4, L


def test_timing_positive():
    t = K.timing()
    assert 2.0 < t["launch_to_neutral"] < 10.0
    assert t["neutral_to_wrapped"] < 45.0


def test_stall_estimate_reasonable():
    assert 0.1 < K.stall_estimate(9.0) < 0.5
