"""Tests for vision.mosaic (aerial map from the flight frames, landing marker)."""
import json
import math

import cv2
import numpy as np
import pytest

from grappstar.sim.flight_video import image_to_world, render_flight
from grappstar.vision.flight import analyze_flight
from grappstar.vision.mosaic import MosaicResult, build_mosaic
from grappstar.video.frame import Frame

SMALL = (360, 240)
LANDING_TOL_M = 0.5   # landing marker vs true landing point, on the ground (sim truth); typically under 0.1 m


def landing_error_m(m: MosaicResult, truth: dict) -> float:
    """Map pixel -> reference frame pixel -> ground (sim truth camera) -> distance to the true landing point."""
    p = np.linalg.solve(m.ref_to_canvas, [m.landing_xy[0], m.landing_xy[1], 1.0])
    xy = image_to_world(truth, m.ref_idx, p[0] / p[2], p[1] / p[2])
    assert xy is not None
    lx, ly = truth["landing_xy_world"]
    return math.hypot(xy[0] - lx, xy[1] - ly)


@pytest.mark.parametrize("seed,use_log,dropouts", [(0, True, True), (1, False, True), (2, True, False),
                                                   (5, False, False)])
def test_mosaic_landing(seed, use_log, dropouts, tmp_path):
    frames, truth = render_flight(seed=seed, size=SMALL, dropouts=dropouts)
    r = analyze_flight(frames, truth["flightlog"] if use_log else None)
    m = build_mosaic(frames, r.t_impact, flight=r)
    assert len(m.used) >= 5, m.notes
    assert not set(m.used) & set(truth["bad_idx"])
    assert all(truth["t_launch"] < frames[i].t < truth["t_impact"] for i in m.used)
    assert m.image is not None and m.image.ndim == 3 and max(m.image.shape[:2]) <= 2000
    assert m.landing_xy is not None
    h, w = m.image.shape[:2]
    assert 0 <= m.landing_xy[0] < w and 0 <= m.landing_xy[1] < h
    err = landing_error_m(m, truth)
    assert err < LANDING_TOL_M
    # the marker is drawn in red
    x, y = int(m.landing_xy[0]), int(m.landing_xy[1])
    r_px = max(10, int(0.015 * max(h, w)))
    ring = m.image[max(0, y - r_px - 3):y + r_px + 4, max(0, x - r_px - 3):x + r_px + 4]
    assert np.any((ring[..., 2] > 200) & (ring[..., 1] < 60))
    json.dumps(m.to_dict())
    out = tmp_path / ("mosaic_%d.png" % seed)
    cv2.imwrite(str(out), m.image)
    assert out.stat().st_size > 1000


def test_mosaic_without_landing_time():
    frames, truth = render_flight(seed=3, size=SMALL)
    m = build_mosaic(frames)                      # runs analyze_flight itself
    assert len(m.used) >= 5
    assert landing_error_m(m, truth) < LANDING_TOL_M


def test_mosaic_full_size_frames():
    frames, truth = render_flight(seed=8, size=(720, 480))
    r = analyze_flight(frames, truth["flightlog"])
    m = build_mosaic(frames, r.t_impact, flight=r)
    assert len(m.used) >= 5
    assert landing_error_m(m, truth) < LANDING_TOL_M


def test_mosaic_degenerate_inputs():
    m = build_mosaic([])
    assert isinstance(m, MosaicResult) and m.used == [] and m.notes and m.image is not None
    json.dumps(m.to_dict())
    black = [Frame(img=np.zeros((240, 360, 3), np.uint8), t=i / 30.0, idx=i) for i in range(40)]
    m = build_mosaic(black, landing_t=1.0)
    assert m.used == [] and m.landing_xy is None and m.image is not None
    flagged = [Frame(img=np.full((240, 360, 3), 128, np.uint8), t=i / 30.0, idx=i, bad=True) for i in range(10)]
    m = build_mosaic(flagged)
    assert m.used == [] and m.notes
    frames, truth = render_flight(seed=0, size=SMALL)
    few = [f for f in frames if truth["t_impact"] - 0.2 < f.t < truth["t_impact"]][:2]
    m = build_mosaic(few, landing_t=truth["t_impact"])
    assert m.image is not None and len(m.used) <= 2
    json.dumps(m.to_dict())
