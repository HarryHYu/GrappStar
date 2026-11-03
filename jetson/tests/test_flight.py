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


# ---------------------------------------------------------------- real launcher speeds (review R1-C #6)
# The real launcher (base/out/ballistics.json, physics model) gives about 7.4 m/s and 6..47 deg, flights of
# 0.4 to 1.1 s. Video-only range must stay within 15 percent across 6 to 8 m/s. The sim's world uses its own
# drag model, so the C3 prior here is a table made from the sim physics at the REAL table's muzzle speed: the
# "launcher" the video sees and the table agree on the model, and the true speed is off the table by up to 20 %.

import os
import grappstar.vision.flight as FL
from grappstar.ballistics import REPO_TABLE, Ballistics

REAL_TOL = 0.15


def _real_v_muzzle():
    if not os.path.exists(REPO_TABLE):
        pytest.skip("no base/out/ballistics.json")
    with open(REPO_TABLE) as f:
        d = json.load(f)
    return float(np.median([r["v_muzzle"] for r in d["table"]])), float(d["pitch_min"]), float(d["pitch_max"])


def _sim_table(v, p_lo=6, p_hi=47):
    from grappstar.sim.flight_video import STROKE_S, RAIL_H
    rows = []
    for p in range(int(p_lo), int(p_hi) + 1):
        sl = v * STROKE_S / 2
        ts, xs, zs = FL.ballistic(v, p, RAIL_H + sl * math.sin(math.radians(p)), dt=0.001)
        r = float(xs[-1] + sl * math.cos(math.radians(p)))
        rows.append({"pitch_deg": float(p), "v_muzzle": v, "range_m": [r] * 3, "t_flight_s": [float(ts[-1])] * 3,
                     "apex_m": [float(zs.max())] * 3})
    return Ballistics({"version": 1, "pitch_min": p_lo, "pitch_max": p_hi, "drag_n": [0.5, 0.75, 1.0],
                       "table": rows}, source="sim_table")


_real_cache = {}


def real_sim(seed, pitch, v):
    key = (seed, pitch, v)
    if key not in _real_cache:
        _real_cache[key] = render_flight(seed=seed, size=SMALL, pitch_deg=pitch, v0=v)
    return _real_cache[key]


@pytest.mark.parametrize("pitch,speed,seed", [
    (8, "lo", 0), (8, "nom", 1), (8, "hi", 2),
    (20, "lo", 3), (20, "nom", 4), (20, "hi", 5),
    (40, "lo", 6), (40, "nom", 7), (40, "hi", 8),
])
def test_video_only_range_at_real_speed_with_aim(pitch, speed, seed):
    v_nom, _, _ = _real_v_muzzle()
    v = {"lo": 6.0, "nom": v_nom, "hi": 8.0}[speed]
    frames, truth = real_sim(seed, pitch, v)
    r = analyze_flight(frames, None, ballistics=_sim_table(v_nom), aim_pitch_deg=pitch)
    assert r.source == "video"
    err = (r.range_m - truth["range_m"]) / truth["range_m"]
    assert abs(err) < REAL_TOL, (r.range_m, truth["range_m"], r.v0, r.notes)
    assert r.launch_pitch_deg == pytest.approx(pitch)
    assert any("fused" in n for n in r.notes)
    assert 0.6 * v_nom - 1e-6 <= r.v0 <= 1.4 * v_nom + 1e-6          # searched only in the band around C3


@pytest.mark.parametrize("pitch,v,seed", [(8, 6.0, 11), (20, 8.0, 12), (40, 7.0, 13)])
def test_video_only_range_at_real_speed_without_aim(pitch, v, seed):
    """No commanded pitch: the pitch comes from the horizon on the rail or the flight time."""
    v_nom, _, _ = _real_v_muzzle()
    frames, truth = real_sim(seed, pitch, v)
    r = analyze_flight(frames, None, ballistics=_sim_table(v_nom))
    err = (r.range_m - truth["range_m"]) / truth["range_m"]
    assert abs(err) < REAL_TOL, (r.range_m, truth["range_m"], r.v0, r.launch_pitch_deg, r.notes)


@pytest.mark.parametrize("pitch,v,seed", [(8, 6.0, 0), (40, 8.0, 7)])
def test_imu_range_at_real_speed(pitch, v, seed):
    v_nom, _, _ = _real_v_muzzle()
    frames, truth = real_sim(seed, pitch, v)
    r = analyze_flight(frames, truth["flightlog"], ballistics=_sim_table(v_nom))
    assert r.source == "video+imu"
    assert abs(r.range_m - truth["range_m"]) < 0.10 * truth["range_m"]
    assert abs(r.v0 - v) < 0.5


def test_old_estimator_failure_case_is_fixed():
    """The review's case: pitch 10, 7.7 m/s, true range 3.8 m; the old video-only estimate was 4.7 to 6.8 m."""
    v_nom, _, _ = _real_v_muzzle()
    for seed in (0, 1):
        frames, truth = real_sim(seed, 10, 7.7)
        r = analyze_flight(frames, None, ballistics=_sim_table(v_nom), aim_pitch_deg=10)
        assert abs(r.range_m - truth["range_m"]) < REAL_TOL * truth["range_m"], (seed, r.range_m, truth["range_m"])


def test_range_calibrated_to_table():
    """The range follows the C3 table at the table speed (the simple drag model only interpolates around it)."""
    v_nom, _, _ = _real_v_muzzle()
    tab = _sim_table(v_nom)
    rows = [dict(r, range_m=[x * 0.8 for x in r["range_m"]]) for r in tab.data["table"]]
    short = Ballistics(dict(tab.data, table=rows))
    frames, truth = real_sim(4, 20, v_nom)
    a = analyze_flight(frames, None, ballistics=tab, aim_pitch_deg=20)
    b = analyze_flight(frames, None, ballistics=short, aim_pitch_deg=20)
    assert a.range_scale == pytest.approx(1.0, abs=0.01)
    assert b.range_scale == pytest.approx(0.8, abs=0.01)
    assert b.range_m == pytest.approx(0.8 * a.range_m, rel=0.03)
    assert b.trajectory[-1][1] == pytest.approx(b.range_m)


def test_fuse_speed_recovers_model_speed():
    from grappstar.sim.flight_video import RAIL_H
    for v_true, p in ((6.0, 10.0), (7.4, 25.0), (8.0, 42.0)):
        ts, xs, zs, gam = FL._gamma_track(v_true, p, RAIL_H)
        track = [(float(t), float(g)) for t, g in zip(ts[5::12], gam[5::12]) if t < ts[-1] - 0.05]
        v, d = FL.fuse_speed(p, RAIL_H, T_obs=float(ts[-1]) + 0.03, track=track, v_prior=7.4, launch_free=True)
        assert v == pytest.approx(v_true, abs=0.4), (v_true, p, v, d)     # the 7.4 prior pulls a little
        # time only (no track, no prior) at a steep pitch, where the flight time depends on the speed: a typical
        # 30 ms late impact is absorbed by the timing window (at 10 deg the 0.5 m launch height dominates the flight
        # time, 23 ms per m/s, so time alone says little there: that was the old estimator's failure)
        if p > 30:
            v2, _ = FL.fuse_speed(p, RAIL_H, T_obs=float(ts[-1]) + 0.03)
            assert v2 == pytest.approx(v_true, abs=0.3)
    assert FL.fuse_speed(20.0, RAIL_H) == (None, {"T_obs": None, "T_min": None, "track_n": 0, "v_prior": None,
                                                  "v_min": None})


def test_run_post_flight_passes_aim_and_table(tmp_path):
    from grappstar import analysis
    v_nom, _, _ = _real_v_muzzle()
    frames, truth = real_sim(4, 20, v_nom)
    res = analysis.run_post_flight(frames, None, None, do_mosaic=False, aim_pitch_deg=20,
                                   ballistics=_sim_table(v_nom))
    fl = res["flight"]
    assert fl["launch_pitch_deg"] == pytest.approx(20.0)
    assert abs(fl["range_m"] - truth["range_m"]) < REAL_TOL * truth["range_m"]
    # auto: the repo table (real launcher) is loaded and used as the prior
    res = analysis.run_post_flight(frames, None, None, do_mosaic=False, aim_pitch_deg=20)
    assert any("table" in n for n in res["flight"]["notes"])


def test_fuse_speed_matches_analyze_flight_model():
    """The speed model starts the free flight where the dart leaves the rail, like tf() and the range model:
    flight times from that model give back the true speed (IMU-timed, the saturated-push path)."""
    from grappstar.sim.flight_video import RAIL_H, STROKE_S
    for v_true, p in ((7.0, 10.0), (7.0, 20.0), (8.0, 40.0)):
        sl = v_true * STROKE_S / 2.0
        T = float(FL.ballistic(v_true, p, RAIL_H + sl * math.sin(math.radians(p)))[0][-1])
        v, _ = FL.fuse_speed(p, RAIL_H, T_obs=T, T_sigma=0.02)
        assert v == pytest.approx(v_true, abs=0.08), (v_true, p, v)


def test_video_speed_stays_in_table_band_and_default_from_table():
    """A flight time that would need a 2 or 30 m/s launcher cannot give one when there is a C3 table."""
    v_nom, _, _ = _real_v_muzzle()
    tab = _sim_table(v_nom)
    from grappstar.sim.flight_video import RAIL_H
    for T in (0.05, 3.0):
        v, _ = FL.fuse_speed(20.0, RAIL_H, T_obs=T, v_prior=v_nom, lo=0.6 * v_nom, hi=1.4 * v_nom)
        assert 0.6 * v_nom - 1e-6 <= v <= 1.4 * v_nom + 1e-6
    assert FL.DEFAULT_V0 < 8.0
    frames, truth = real_sim(4, 20, v_nom)
    r = analyze_flight(frames, None, ballistics=tab, aim_pitch_deg=20)
    assert 0.6 * v_nom <= r.v0 <= 1.4 * v_nom
