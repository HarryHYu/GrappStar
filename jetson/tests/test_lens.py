"""Dart camera lens model (vision/lens.py) and its checkerboard calibration (calibrate dart)."""
import math

import numpy as np
import pytest

from grappstar.vision.lens import CANON_HFOV_DEG, DEFAULT_HFOV_DEG, DartLens, rectify_frames


def test_equidistant_fov_and_roundtrip():
    lens = DartLens.equidistant(140.0, (720, 480), "4:3")
    assert lens.hfov_deg() == pytest.approx(140.0, abs=0.3)
    # 4:3 picture squeezed into 720x480: pixels 0.89 as wide as tall, so fy = 0.889 fx
    assert lens.K[1, 1] / lens.K[0, 0] == pytest.approx(480 / 720 * 4 / 3)
    uv = np.array([[10.0, 10.0], [360.0, 240.0], [700.0, 400.0], [100.0, 470.0]])
    back = lens.project(lens.unproject(uv))
    assert np.allclose(back, uv, atol=1e-6)
    # with distortion terms too
    lens2 = DartLens(lens.K, [0.03, -0.01, 0.002, 0.0], (720, 480))
    assert np.allclose(lens2.project(lens2.unproject(uv)), uv, atol=1e-5)
    assert lens.scaled((360, 240)).hfov_deg() == pytest.approx(lens.hfov_deg(), abs=0.3)


def test_default_is_the_caddx_guess_and_settings(tmp_path):
    d = DartLens.from_settings()
    assert d.model == "fisheye" and d.hfov_deg() == pytest.approx(DEFAULT_HFOV_DEG, abs=0.3)
    assert "not measured" in d.source
    p = DartLens.from_settings(hfov_deg=150).hfov_deg()
    assert p == pytest.approx(150, abs=0.3)
    assert DartLens.from_settings(lens="pinhole", hfov_deg=100).model == "pinhole"
    f = tmp_path / "dartcam.json"
    DartLens(d.K, [0.01, 0, 0, 0], d.size).save(str(f))
    got = DartLens.from_settings(str(f))
    assert got.source.startswith("calibration") and got.D[0] == pytest.approx(0.01)
    # a missing file falls back to the settings guess, never crashes
    assert DartLens.from_settings(str(tmp_path / "missing.json"), hfov_deg=130).hfov_deg() == pytest.approx(130, abs=0.3)


def test_canonical_pinhole_is_a_no_op():
    img = np.random.default_rng(0).integers(0, 255, (480, 720, 3), dtype=np.uint8)
    lens = DartLens.pinhole(CANON_HFOV_DEG, (720, 480))
    assert lens.rectify(img) is img


def _rail_horizon_errors(lens_for_analysis, seed=3, true_hfov=135.0):
    from grappstar.sim.flight_video import render_flight
    from grappstar.vision.flight import detect_horizon
    frames, truth = render_flight(seed=seed, size=(360, 240), profile="caddx", lens_hfov_deg=true_hfov,
                                  pitch_deg=18.0, landed_s=0.3, dropouts=False, link="daylight")
    if lens_for_analysis is not None:
        frames = rectify_frames(frames, lens_for_analysis)
    errs = []
    for f in frames:
        pose = truth["poses"][f.idx]
        if pose[0] < truth["t_launch"] - 0.05:
            h = detect_horizon(f.img, 100.0, "pinhole")
            if h["pitch_deg"] is not None:
                errs.append(h["pitch_deg"] - pose[5])
    return errs


def test_rectified_fisheye_gives_the_right_horizon_pitch():
    """The canonical view makes the fisheye look like the pinhole camera the vision code was tuned on."""
    good = _rail_horizon_errors(DartLens.equidistant(135.0, (360, 240)))
    assert len(good) >= 10 and abs(float(np.median(good))) < 1.0, good
    # a lens model 10 deg off still reads the horizon within 2.5 deg near the image centre at 18 deg pitch ...
    off = _rail_horizon_errors(DartLens.equidistant(145.0, (360, 240)))
    assert abs(float(np.median(off))) < 2.5
    # ... but analysing the raw fisheye picture as if it were the 100 deg pinhole is badly wrong
    raw = _rail_horizon_errors(None)
    assert not raw or abs(float(np.median(raw))) > 3.0


def test_calibrate_dart_recovers_the_fisheye():
    from grappstar.sim.checkerboard import random_views
    from grappstar.vision.calibrate import calibrate_dart
    true = DartLens(DartLens.equidistant(146.0, (720, 480)).K, [0.02, -0.01, 0.0, 0.0], (720, 480))
    views = random_views(true, 24, np.random.default_rng(1))
    lens, info = calibrate_dart(views)
    assert info["n_used"] >= 12 and info["rms"] < 0.6
    assert lens.hfov_deg() == pytest.approx(true.hfov_deg(), abs=1.5)
    assert lens.vfov_deg() == pytest.approx(true.vfov_deg(), abs=1.5)
    # pixels map to the same rays (within a tenth of a degree) over the picture
    uv = np.array([[60.0, 60.0], [360.0, 240.0], [660.0, 420.0], [200.0, 400.0]])
    a, b = true.unproject(uv), lens.unproject(uv)
    ang = np.degrees(np.arccos(np.clip((a * b).sum(1), -1, 1)))
    assert ang.max() < 0.5, ang


def test_calibrate_dart_refuses_too_few_views():
    from grappstar.sim.checkerboard import random_views
    from grappstar.vision.calibrate import calibrate_dart
    true = DartLens.equidistant(140.0, (720, 480))
    views = random_views(true, 4, np.random.default_rng(2))
    with pytest.raises(ValueError, match="need 8 or more"):
        calibrate_dart(views)


def test_calibrate_dart_cli(tmp_path):
    import cv2
    from grappstar.sim.checkerboard import random_views
    from grappstar.vision import calibrate
    true = DartLens.equidistant(138.0, (720, 480))
    paths = []
    for i, v in enumerate(random_views(true, 14, np.random.default_rng(3))):
        p = tmp_path / f"b{i:02d}.png"
        cv2.imwrite(str(p), v)
        paths.append(str(p))
    out = tmp_path / "dartcam.json"
    assert calibrate.main(["dart", "--images"] + paths + ["--out", str(out)]) == 0
    got = DartLens.load(str(out))
    assert got.hfov_deg() == pytest.approx(138.0, abs=2.0)
