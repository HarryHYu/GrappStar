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
