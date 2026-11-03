"""Tests for vision.flight (phases, IMU fusion, attitude, horizon, trajectory, landing frames)."""
import json
import math

import numpy as np
import pytest

from grappstar.sim.flight_video import render_flight, render_view
from grappstar.vision.flight import (FlightResult, analyze_flight, detect_horizon, imu_events, landing_frames,
                                     quick_quality)
from grappstar.video.frame import Frame

SMALL = (360, 240)
PHASE_TOL = 0.15     # s, launch and impact
RANGE_TOL = 0.30     # fraction of the true range

_cache = {}


def sim(seed, dropouts=True):
    key = (seed, dropouts)
    if key not in _cache:
        _cache[key] = render_flight(seed=seed, size=SMALL, dropouts=dropouts)
    return _cache[key]


@pytest.mark.parametrize("seed,dropouts,use_log", [
    (0, True, False), (0, True, True), (1, False, False), (1, False, True),
    (4, True, False), (4, True, True), (7, True, False), (9, True, True),
])
def test_phases(seed, dropouts, use_log):
    frames, truth = sim(seed, dropouts)
    r = analyze_flight(frames, truth["flightlog"] if use_log else None)
    assert r.source == ("video+imu" if use_log else "video")
    assert abs(r.t_launch - truth["t_launch"]) < PHASE_TOL
    assert abs(r.t_impact - truth["t_impact"]) < PHASE_TOL
    assert r.t_landed is not None and truth["t_impact"] <= r.t_landed + 0.05 < truth["t_landed"] + 0.5
    names = [p[0] for p in r.phases]
    assert names == ["ON_RAIL", "LAUNCH", "FLIGHT", "IMPACT", "LANDED"]
    for (_, a0, a1), (_, b0, _) in zip(r.phases[:-1], r.phases[1:]):
        assert a1 == pytest.approx(b0) and a1 >= a0
    assert r.phase_at(frames[-1].t) == "LANDED"
    assert r.phase_at(truth["t_launch"] - 0.3) == "ON_RAIL"
    assert r.phase_at(0.5 * (truth["t_launch"] + truth["t_impact"])) == "FLIGHT"
    # trajectory and range
    assert r.trajectory and r.range_m is not None
    assert abs(r.range_m - truth["range_m"]) < RANGE_TOL * truth["range_m"]
    assert r.trajectory[-1][3] == pytest.approx(0.0, abs=1e-6)
    json.dumps(r.to_dict())
    if use_log:
        assert abs(r.clock_offset_s - truth["clock_offset_s"]) < 0.05
        assert abs(r.launch_pitch_deg - truth["launch_pitch_deg"]) < 1.0
        assert abs(r.v0 - truth["v0"]) < 0.5
        assert abs(r.range_m - truth["range_m"]) < 0.1 * truth["range_m"]


def test_quality_flags_dropouts():
    frames, truth = sim(0)
    bad = truth["bad_idx"]
    q = {f.idx: quick_quality(f.img) for f in frames}
    assert all(q[i] < 0.4 for i in bad)
    false_bad = [i for i in q if q[i] < 0.4 and i not in bad]
    assert len(false_bad) <= 2


def _truth_rpy(truth, t):
    pt = np.array([p[0] for p in truth["poses"].values()])
    pr = np.array([p[4:7] for p in truth["poses"].values()])
    return [np.interp(t, pt, pr[:, k]) for k in range(3)]


def test_attitude_from_quaternions_explicit_offset():
    frames, truth = sim(1, False)
    r = analyze_flight(frames, truth["flightlog"], clock_offset_s=truth["clock_offset_s"])
    assert r.clock_offset_s == truth["clock_offset_s"]
    n = 0
    for t, roll, pitch, yaw in r.attitude:
        if not (frames[0].t <= t <= frames[-1].t):
            continue
        tr, tp, ty = _truth_rpy(truth, t)
        # truth poses are sampled at frame rate; allow the interpolation error during the impact shake
        tol = 4.0 if truth["t_impact"] - 0.02 < t < truth["t_landed"] else 1.0
        assert abs(pitch - tp) < tol
        assert abs((roll - tr + 180) % 360 - 180) < tol
        assert abs((yaw - ty + 180) % 360 - 180) < tol
        n += 1
    assert n > 100


def test_attitude_estimated_offset_static_parts():
    frames, truth = sim(4)
    r = analyze_flight(frames, truth["flightlog"])
    for t, roll, pitch, yaw in r.attitude:
        static = (frames[0].t < t < truth["t_launch"] - 0.1) or (truth["t_landed"] + 0.1 < t < frames[-1].t)
        if static:
            tr, tp, _ = _truth_rpy(truth, t)
            assert abs(pitch - tp) < 1.0 and abs(roll - tr) < 1.0


def test_attitude_from_horizon_without_imu():
    frames, truth = sim(0)
    r = analyze_flight(frames)
    assert len(r.attitude) > 20
    errs = []
    for t, roll, pitch, yaw in r.attitude:
        assert yaw is None
        tr, tp, _ = _truth_rpy(truth, t)
        errs.append(max(abs(roll - tr), abs(pitch - tp)))
    assert np.median(errs) < 2.0


@pytest.mark.parametrize("roll,pitch", [(20.0, -10.0), (-35.0, 5.0), (8.0, 20.0)])
def test_horizon_on_tilted_frame(roll, pitch):
    img = render_view((2.0, 0.0, 1.5, roll, pitch, 3.0), size=(720, 480), noise=0.02)
    hz = detect_horizon(img)
    assert hz["angle_deg"] == pytest.approx(roll, abs=3.0)
    assert hz["pitch_deg"] == pytest.approx(pitch, abs=3.0)
    assert 0.05 < hz["sky_ratio"] < 0.95


def test_horizon_none_when_all_ground_or_sky():
    down = render_view((2.0, 0.0, 1.5, 0.0, -80.0, 0.0), size=SMALL)
    up = render_view((2.0, 0.0, 1.5, 0.0, 80.0, 0.0), size=SMALL)
    h1, h2 = detect_horizon(down), detect_horizon(up)
    assert h1["angle_deg"] is None and h1["sky_ratio"] < 0.02
    assert h2["angle_deg"] is None and h2["sky_ratio"] > 0.98


def test_landing_frames():
    frames, truth = sim(0)
    r = analyze_flight(frames)
    lf = landing_frames(frames, r, n=10)
    assert len(lf) == 10
    assert all(f.t > truth["t_impact"] + 0.1 for f in lf)
    assert not set(f.idx for f in lf) & set(truth["bad_idx"])
    # without a result: last 2 s
    lf2 = landing_frames(frames, None, n=5)
    assert len(lf2) == 5 and all(f.t > frames[-1].t - 2.1 for f in lf2)


def test_imu_events_only():
    frames, truth = sim(9)
    ev = imu_events(truth["flightlog"])
    off = truth["clock_offset_s"]
    assert abs(ev["launch"] + off - truth["t_launch"]) < 0.03
    assert abs(ev["impact"] + off - truth["t_impact"]) < 0.03
    assert abs(ev["landed"] + off - truth["t_landed"]) < 0.15


def test_empty_and_bad_inputs():
    r = analyze_flight([])
    assert isinstance(r, FlightResult) and r.phases == [] and r.t_launch is None
    json.dumps(r.to_dict())
    blue = np.zeros((240, 360, 3), np.uint8)
    blue[:] = (200, 30, 20)
    bad = [Frame(img=blue.copy(), t=100 + i / 30, idx=i) for i in range(60)]
    r = analyze_flight(bad)
    assert r.good == [] and r.t_launch is None and r.notes
    r = analyze_flight(bad, flightlog={"hz": 50, "samples": []})
    assert r.t_launch is None
    flagged = [Frame(img=np.full((240, 360, 3), 120, np.uint8), t=i / 30, idx=i, bad=True) for i in range(5)]
    assert analyze_flight(flagged).good == []
    assert landing_frames(bad, r, n=5) == []
    assert landing_frames([], None) == []
    # flightlog alone (video with no launch): no crash, no alignment
    frames, truth = sim(0)
    r = analyze_flight(frames[:20], truth["flightlog"])
    assert r.t_launch is None or r.source == "video+imu"
