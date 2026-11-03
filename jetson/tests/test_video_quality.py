"""video.quality on synthetic analog-video frames."""
import time

import numpy as np
import pytest

from grappstar.video import synth
from grappstar.video.quality import frame_quality, BAD_BELOW

SEEDS = range(4)


@pytest.mark.parametrize("seed", SEEDS)
def test_clean_frames_are_good(seed):
    q = frame_quality(synth.textured(seed=seed))
    assert q["score"] > 0.8 and not q["bad"]
    assert q["snow"] < 0.2 and q["dropout"] == 0 and q["sync_loss"] < 0.2


@pytest.mark.parametrize("seed", SEEDS)
def test_snow_is_bad(seed):
    q = frame_quality(synth.add_snow(synth.textured(seed=seed), 0.5, seed))
    assert q["bad"] and q["snow"] > 0.8


def test_light_snow_degrades_but_passes():
    q = frame_quality(synth.add_snow(synth.textured(seed=0), 0.15))
    assert 0.4 <= q["score"] < 0.95


@pytest.mark.parametrize("img", [synth.blue_screen(), synth.black_screen(), synth.blue_screen(576, 720)])
def test_dropout_screens_are_bad(img):
    q = frame_quality(img)
    assert q["bad"] and q["dropout"] > 0.9 and q["score"] == 0


@pytest.mark.parametrize("seed", SEEDS)
def test_torn_rows_are_bad(seed):
    q = frame_quality(synth.torn(synth.textured(seed=seed), bands=4, seed=seed))
    assert q["bad"], q
    assert q["sync_loss"] > 0.6


def test_vertical_roll_is_bad():
    q = frame_quality(synth.rolled(synth.textured(seed=5)))
    assert q["bad"] and q["roll_bar"]


def test_freeze_detected_only_for_identical_frames():
    a = synth.textured(seed=1)
    assert frame_quality(a, a.copy())["freeze"] == 1.0
    assert frame_quality(a, a.copy())["bad"]
    b = synth.add_snow(a, 0.02)            # tiny noise = a live picture
    q = frame_quality(b, a)
    assert q["freeze"] == 0.0 and not q["bad"]
    assert frame_quality(a, synth.textured(240, 320))["freeze"] == 0.0   # size mismatch


def test_score_range_and_keys():
    q = frame_quality(synth.textured())
    for k in ("score", "snow", "dropout", "sync_loss", "freeze"):
        assert 0.0 <= q[k] <= 1.0
    assert BAD_BELOW == 0.4


def test_grey_input_supported():
    q = frame_quality(synth.textured()[:, :, 1])
    assert q["score"] > 0.8


def test_fast_enough():
    img = synth.textured(480, 720)
    prev = img.copy()
    frame_quality(img, prev)
    n = 200
    t0 = time.perf_counter()
    for _ in range(n):
        frame_quality(img, prev)
    ms = (time.perf_counter() - t0) / n * 1000
    assert ms < 3.0, "frame_quality took %.2f ms" % ms
