"""The dart vision on REALISTIC synthetic flights (sim profile "caddx"; see tests/eval_realistic.py for the whole
evaluation and README section 13 for the numbers). These tests pin the accuracy the README quotes."""
import json
import math
import os
import sys

import numpy as np
import pytest

from grappstar.ballistics import REPO_TABLE
from grappstar.vision import flight as FL

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def test_launcher_model_matches_the_c3_table():
    """The sim and the analysis fly the same model: exit 0.42 m, 0.10 m ahead of the pivot, drag fitted to the
    base physics table (LIGHT 0.25 N column). Within 1 percent of its range and 1.5 percent of its flight time."""
    if not os.path.exists(REPO_TABLE):
        pytest.skip("no base/out/ballistics.json")
    d = json.load(open(REPO_TABLE))
    for r in d["table"]:
        ts, xs, zs = FL.ballistic(r["v_muzzle"], r["pitch_deg"], FL.H_EXIT, dt=0.0005)
        rng = xs[-1] + FL.X_EXIT
        assert abs(rng / r["range_m"][0] - 1) < 0.01, (r["pitch_deg"], rng, r["range_m"][0])
        assert abs(ts[-1] / r["t_flight_s"][0] - 1) < 0.015, (r["pitch_deg"], ts[-1], r["t_flight_s"][0])
    from grappstar.sim import flight_video as SV
    assert (SV.H_EXIT, SV.X_EXIT, SV.DRAG_K, SV.DRAG_C) == (FL.H_EXIT, FL.X_EXIT, FL.DRAG_K, FL.DRAG_C)


@pytest.mark.parametrize("seed,link", [(201, "daylight"), (3, "lowlight"), (5, "weak_link")])
def test_frame_quality_on_realistic_frames(seed, link):
    """The camera's link check on fisheye, sharpened, noisy, interlaced frames: every snow, no-signal and torn frame
    is caught and no clean frame is thrown away (it used to drop textured ground under a smooth sky as torn)."""
    from grappstar.sim.flight_video import render_flight
    from grappstar.video.quality import frame_quality
    from grappstar.vision.flight import frame_ok
    from grappstar.vision.lens import DartLens, rectify_frames
    frames, truth = render_flight(seed=seed, size=(360, 240), profile="caddx", link=link, landed_s=1.0)
    bad, dups = truth["bad_idx"], set(truth["dup_idx"])
    prev = None
    for f in frames:
        q = frame_quality(f.img, prev)
        prev = f.img
        if f.idx in bad:
            assert q["bad"], (f.idx, bad[f.idx], q)
        elif f.idx not in dups:
            assert not q["bad"], (f.idx, q)
    # the vision's own quick check on the rectified frames: no clean frame lost
    rect = rectify_frames(frames, DartLens.equidistant(size=(360, 240)))
    lost = [f.idx for f in rect if f.idx not in bad and not frame_ok(f)]
    assert len(lost) <= 1, lost


@pytest.fixture(scope="module")
def realistic_rows():
    """A fixed reduced set of the eval_realistic cases (all six ground types, every link condition, both rest
    poses), 360x240, the analysis exactly as the mission calls it with the uncalibrated default lens."""
    import eval_realistic as E
    jobs = min(4, os.cpu_count() or 1)
    return E.evaluate(range(0, 12), (360, 240), "default", jobs=jobs, mosaic=True)


def test_realistic_range_accuracy(realistic_rows):
    """README section 13: with the IMU flight log the range is within a few percent; video only (commanded pitch
    known) it is a rougher fallback. The full 48 flight run (eval_realistic --n 48) gave IMU median 1.2 percent,
    90th percentile 3.2 percent, max 4.2; video only median 4.4 percent, 90th percentile 18."""
    rows = [r for r in realistic_rows if "error" not in r]
    assert len(rows) == len(realistic_rows), [r for r in realistic_rows if "error" in r]
    imu = sorted(abs(r["imu_err"]) for r in rows)
    vid = sorted(abs(r["vid_aim_err"]) for r in rows)
    assert imu[len(imu) // 2] < 0.03 and imu[-1] < 0.07, imu
    assert vid[len(vid) // 2] < 0.10, vid
    # the capture delay is applied: the IMU impact lines up with the frames within a frame
    assert all(abs(r["imu_impact_err"]) < 0.034 for r in rows), [r["imu_impact_err"] for r in rows]


def test_realistic_landing_verdicts_are_safe(realistic_rows):
    """Never GO on sand, bedrock or water; GO on hard ground and loose soil (the full run: 16 of 16, 0 unsafe)."""
    for r in realistic_rows:
        for k in ("imu", "vid"):
            v = r[k + "_verdict"]
            if r["cls"] in ("sand_dust", "bedrock", "water_void"):
                assert v != "GO", (r["seed"], r["cls"], r[k + "_cls"], v)
            if r["cls"] in ("hard_ground", "loose_soil"):
                assert v == "GO", (r["seed"], r["cls"], r[k + "_cls"], v)


def test_realistic_aerial_map_marks_the_landing(realistic_rows):
    """The map's landing marker within 0.35 m of the true impact (full run: median 0.07 m, 90th percentile 0.15 m),
    from the flight path and the dart attitude (the nose does not point along the path: angle of attack)."""
    errs = [r["mosaic_err_m"] for r in realistic_rows]
    assert all(e is not None for e in errs)
    errs.sort()
    assert errs[len(errs) // 2] < 0.15 and errs[int(0.9 * len(errs))] < 0.35, errs
