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
    assert K.sector(C["reel"]) == "REEL"
    assert K.sector(C["wrapped"]) == "WRAPPED"
    assert K.mode(C["launch"]) == "LAUNCH"
    assert K.mode(C["neutral"]) == "NEUTRAL"
    assert K.mode(C["reel"]) == "REEL"
    assert K.mode(C["wrapped"]) == "WRAPPED"


def test_code_inside_groove_dwells():
    """Whenever the code says LAUNCH / NEUTRAL / REEL the collar is in a flat dwell (no partial engagement)."""
    th = C["stop_lo"]
    while th <= C["stop_hi"]:
        s = K.sector(th)
        x = K.collar_x(th)
        if s == "LAUNCH":
            assert x == -C["stroke"]
        if s == "NEUTRAL":
            assert x == 0.0
        if s in ("REEL", "WRAPPING", "WRAPPED"):
            assert x == C["stroke"]
        th += 0.1


def test_no_wraps_outside_reel():
    th = C["stop_lo"]
    while th <= C["stop_hi"]:
        if K.wraps(th) > 0:
            assert K.collar_x(th) == C["stroke"], "wrap arm moved while the collar is not in REEL"
            assert K.sector(th) in ("WRAPPING", "WRAPPED")
        th += 0.25


def test_wraps_integer_and_lock_phase():
    assert abs(K.wraps_total() - round(K.wraps_total())) < 1e-9
    assert abs(K.wrap_turns_pinion() - round(K.wrap_turns_pinion())) < 1e-9
    assert K.wraps(C["wrapped"]) == pytest.approx(K.wraps_total())
    assert K.wraps(C["reel"]) == 0.0


def test_brake_levels():
    assert K.brake_frac(C["launch"]) == 0.0
    assert K.brake_frac(C["neutral"]) == 0.0
    assert K.brake_frac(C["wrapped"]) == pytest.approx(1.0)
    assert 0.0 < K.brake_frac(150.0) <= 0.5 + 1e-9
    assert K.brake_frac(150.0) == pytest.approx(C["brake_mid"])


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
    assert t["wrap"] < 40.0


def test_stall_estimate_reasonable():
    assert 0.1 < K.stall_estimate(9.0) < 0.5
