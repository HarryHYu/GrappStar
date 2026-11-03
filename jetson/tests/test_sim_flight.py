"""Tests for sim.flight_video (synthetic dart flight video + C2 flightlog)."""
import math
import time

import numpy as np
import pytest

from grappstar.sim.flight_video import (image_to_world, quat_from_rot, render_flight, render_view,
                                         rot_from_rpy, world_to_map)
from grappstar.vision.flight import quat_to_rpy

G = 9.81
SMALL = (360, 240)


@pytest.fixture(scope="module")
def flight():
    return render_flight(seed=3, size=SMALL)


def test_frames_and_truth_keys(flight):
    frames, truth = flight
    assert len(frames) > 100
    for k in ("t_launch", "t_impact", "t_landed", "landing_class", "landing_xy_world", "flightlog", "world_map",
              "landing_px_map", "poses", "bad_idx", "clock_offset_s", "camera"):
        assert k in truth
    assert frames[0].img.shape == (240, 360, 3) and frames[0].img.dtype == np.uint8
    ts = [f.t for f in frames]
    assert np.all(np.diff(ts) > 0)
    assert frames[0].t < truth["t_launch"] < truth["t_impact"] < truth["t_landed"] < frames[-1].t
    assert 0.8 < truth["t_launch"] - frames[0].t < 1.4            # about 1 s on the rail
    assert 0.9 < truth["t_impact"] - truth["t_launch"] < 1.8      # 1 to 1.5 s flight
    assert frames[-1].t - truth["t_landed"] >= 2.0                # still for 2 to 3 s
    assert 5.0 < truth["range_m"] < 12.0
    assert all(f.quality is None and not f.bad for f in frames)   # raw capture by default
    assert len(truth["bad_idx"]) >= 4
    col, row = truth["landing_px_map"]
    h, w = truth["world_map"].shape[:2]
    assert 0 <= col < w and 0 <= row < h
    assert world_to_map(truth, *truth["landing_xy_world"]) == pytest.approx((col, row))


def test_deterministic():
    a, ta = render_flight(seed=5, size=(180, 120))
    b, tb = render_flight(seed=5, size=(180, 120))
    assert len(a) == len(b)
    assert all(np.array_equal(x.img, y.img) for x, y in zip(a, b))
    assert ta["flightlog"] == tb["flightlog"]
    c, tc = render_flight(seed=6, size=(180, 120))
    assert tc["t_impact"] - c[0].t != pytest.approx(ta["t_impact"] - a[0].t)


def test_no_dropouts_and_mark_bad():
    frames, truth = render_flight(seed=2, size=(180, 120), dropouts=False)
    assert truth["bad_idx"] == {}
    frames, truth = render_flight(seed=2, size=(180, 120), mark_bad=True)
    assert {f.idx for f in frames if f.bad} == set(truth["bad_idx"])


def test_flightlog_profile(flight):
    frames, truth = flight
    fl = truth["flightlog"]
    assert fl["hz"] == 50
    s = np.array(fl["samples"], float)
    assert s.shape[1] == 8
    assert np.allclose(np.diff(s[:, 0]), 20)
    off = truth["clock_offset_s"]
    t = s[:, 0] / 1000.0 + off
    a = np.linalg.norm(s[:, 5:8], axis=1)
    rail = t < truth["t_launch"] - 0.05
    assert rail.sum() > 50 and np.allclose(a[rail], G, atol=0.3)
    assert np.allclose(s[rail, 7], G, atol=3.0)                   # z up when the dart is level-ish
    # launch spike over 4 g for at least 20 ms (2 or more consecutive samples)
    hi = np.nonzero(a > 4 * G)[0]
    first = hi[0]
    assert a[first + 1] > 4 * G
    assert abs(t[first] - truth["t_launch"]) < 0.03
    fl_mask = (t > truth["t_stroke_end"] + 0.03) & (t < truth["t_impact"] - 0.03)
    assert fl_mask.sum() > 30 and a[fl_mask].max() < 0.4 * G      # near free fall, drag only
    imp = (t >= truth["t_impact"]) & (t < truth["t_impact"] + 0.06)
    assert a[imp].max() > 6 * G
    late = t > truth["t_landed"] + 0.1
    assert late.sum() > 50 and np.allclose(a[late], G, atol=0.3)
    assert abs(fl["t_launch_ms"] / 1000.0 + off - truth["t_launch"]) < 0.05
    assert abs(fl["t_land_ms"] / 1000.0 + off - truth["t_impact"]) < 0.01


def test_quaternions_match_poses(flight):
    frames, truth = flight
    s = np.array(truth["flightlog"]["samples"], float)
    t = s[:, 0] / 1000.0 + truth["clock_offset_s"]
    pt = np.array([p[0] for p in truth["poses"].values()])
    pr = np.array([p[4:7] for p in truth["poses"].values()])
    for k in range(0, len(s), 7):
        if not (pt[0] <= t[k] <= pt[-1]):
            continue
        j = int(np.argmin(np.abs(pt - t[k])))
        if abs(pt[j] - t[k]) > 0.002:
            continue
        roll, pitch, yaw = quat_to_rpy(*s[k, 1:5])
        assert pitch == pytest.approx(pr[j, 1], abs=0.5)
        assert ((roll - pr[j, 0] + 180) % 360 - 180) == pytest.approx(0, abs=0.5)


def test_rotation_round_trip():
    for rpy in [(0, 0, 0), (10, 35, -5), (-30, -60, 170), (80, 20, 45)]:
        q = quat_from_rot(rot_from_rpy(*rpy))
        got = quat_to_rpy(*q)
        assert got == pytest.approx(rpy, abs=1e-6)
    # body x (nose) points up at positive pitch
    assert (rot_from_rpy(0, 30, 0) @ [1, 0, 0])[2] == pytest.approx(math.sin(math.radians(30)))


def test_image_to_world_hits_landing(flight):
    frames, truth = flight
    cam = truth["camera"]
    # the resting camera looks at its landing point
    idx = frames[-1].idx
    xy = image_to_world(truth, idx, cam["cx"], cam["cy"])
    assert xy is not None
    assert math.hypot(xy[0] - truth["landing_xy_world"][0], xy[1] - truth["landing_xy_world"][1]) < 0.05
    # rail view centre is sky
    assert image_to_world(truth, frames[0].idx, cam["cx"], cam["cy"]) is None


def test_render_view_horizon_and_speed():
    img = render_view((0, 0, 1.5, 0, 0, 0), size=SMALL)
    top, bottom = img[:40].mean(axis=(0, 1)), img[-40:].mean(axis=(0, 1))
    assert top[0] > top[2] + 20          # blue sky on top
    assert bottom.mean() < top.mean()    # darker ground below
    t0 = time.process_time()
    render_flight(seed=1, size=(720, 480))
    assert time.process_time() - t0 < 6.0   # about 2 s CPU on a desktop; generous for CI
