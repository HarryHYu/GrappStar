"""Review round 2: tests that demonstrate real bugs (expected to FAIL until fixed)."""
import copy
import os
import time

import numpy as np
import pytest

from grappstar.video.frame import Frame
from grappstar.vision.flight import analyze_flight, imu_events


@pytest.fixture(scope="module")
def sim():
    from grappstar.sim.flight_video import render_flight
    return render_flight(seed=3, size=(360, 240))


def _saturate(fl, sat=78.0):
    fl = copy.deepcopy(fl)
    for s in fl["samples"]:
        for k in (5, 6, 7):
            s[k] = max(-sat, min(sat, s[k]))
    return fl


def test_imu_v0_with_bno085_saturation(sim):
    """API.md: the accelerometer saturates near 78 m/s^2, so the launch push reads flat at 78. The integrated v0
    (about 4.4 m/s instead of 9.1) passes the 3..30 gate and is used as truth: the range comes out about 3x short."""
    frames, truth = sim
    fl = _saturate(truth["flightlog"])
    r = analyze_flight(frames, fl, clock_offset_s=truth["clock_offset_s"])
    assert r.v0 == pytest.approx(truth["v0"], rel=0.2)
    assert r.range_m == pytest.approx(truth["range_m"], rel=0.2)


def test_single_glitch_sample_before_launch(sim):
    """One >4 g sample (latch click, knock while loading) inside the 2 s pre-launch window becomes the launch,
    and the real launch push (>6 g, >0.3 s later) becomes the impact. The firmware needs 20 ms over 4 g and the
    flightlog's own t_launch_ms is ignored."""
    frames, truth = sim
    fl = copy.deepcopy(truth["flightlog"])
    off = truth["clock_offset_s"]
    tl_ms = (truth["t_launch"] - off) * 1000.0
    k = min(range(len(fl["samples"])), key=lambda i: abs(fl["samples"][i][0] - (tl_ms - 800)))
    fl["samples"][k][7] = 50.0
    r = analyze_flight(frames, fl, clock_offset_s=off)
    assert abs(r.t_launch - truth["t_launch"]) < 0.05
    assert abs(r.t_impact - truth["t_impact"]) < 0.1


def test_fallback_launch_time_is_start_of_window():
    """API.md: 'Launch time = start of those 20 ms'. imu_events subtracts another 20 ms when only t_launch_ms is
    available (it matches sim.flight_video, which stamps the END of the window, not the firmware)."""
    ev = imu_events({"t_launch_ms": 3000, "t_land_ms": 4010, "samples": []})
    assert ev["launch"] == pytest.approx(3.000, abs=1e-6)


def test_mock_and_sim_agree_on_t_launch_ms():
    """comms.mock_dart.synth_flightlog puts t_launch_ms at the spike start; sim.flight_video at start + 20 ms."""
    from grappstar.comms.mock_dart import synth_flightlog
    fl = synth_flightlog(t_launch_ms=2000)
    first_hi = next(s[0] for s in fl["samples"] if np.linalg.norm(s[5:8]) > 4 * 9.81)
    assert fl["t_launch_ms"] == first_hi            # mock: start of the push
    from grappstar.sim.flight_video import render_flight
    _, truth = render_flight(seed=5, size=(160, 120), dropouts=False, landed_s=0.5)
    sim_ms = truth["flightlog"]["t_launch_ms"]
    true_ms = (truth["t_launch"] - truth["clock_offset_s"]) * 1000.0
    assert abs(sim_ms - true_ms) < 5                 # fails: sim is 20 ms late


@pytest.mark.parametrize("shape", [(481, 721, 4), (480, 720, 4), (480, 720)])
def test_analyze_flight_never_raises_on_non_bgr(shape):
    """Docstring: 'Never raises on bad input'. A 4-channel or grey frame raises (reshape(-1, 3), cv2.split, cvtColor)."""
    rng = np.random.default_rng(0)
    fr = [Frame(img=rng.integers(0, 255, shape, dtype=np.uint8), t=i / 30.0, idx=i, quality=0.9) for i in range(10)]
    analyze_flight(fr)


def test_camera_max_fps_equal_to_source_rate_drops_frames():
    """app.py sets max_fps=rover_fps (15) for a 15 fps source. With a few ms of jitter, every interval shorter than
    1/max_fps is dropped, so the stream runs at about 9 fps."""
    from grappstar.video.camera import Camera

    class Src:
        name = "jitter"

        def __init__(self, fps):
            self.fps, self.rng, self.next = fps, np.random.default_rng(0), time.time()

        def read(self):
            self.next += 1.0 / self.fps
            d = self.next + self.rng.uniform(-0.003, 0.003) - time.time()
            if d > 0:
                time.sleep(d)
            return Frame(img=np.zeros((24, 32, 3), np.uint8), t=time.time(), idx=0)

        def close(self):
            pass

    cam = Camera("r", lambda: Src(15), quality=False, max_fps=15)
    time.sleep(2.0)
    n = cam.status()["frames"]
    cam.stop()
    assert n / 2.0 > 13.0


def test_filesource_corrupt_jpeg_ends_playback(tmp_path):
    """One unreadable JPEG in a fallback folder makes read() return None forever (end of file) at that index."""
    from grappstar.video.record import Recorder
    from grappstar.video.replay import FileSource
    p = str(tmp_path / "f.avi")
    rec = Recorder(p, fps=30, force_jpeg_folder=True)
    for i in range(30):
        rec.write(Frame(img=np.full((24, 32, 3), i, np.uint8), t=100.0 + i / 30.0, idx=i))
    rec.close()
    with open(os.path.join(rec.path, "000010.jpg"), "wb") as fh:
        fh.write(b"not a jpeg")
    frames = FileSource(rec.path).read_all()
    assert len(frames) >= 29


def test_video_only_no_inflight_roll_passed_as_ground_tilt(sim, monkeypatch):
    """Without a flightlog the last attitude entry is a flight frame (horizon gone after landing). analysis.py must
    then pass no resting attitude to terrain rather than the in-flight roll (fixed in analysis.py)."""
    from grappstar import analysis
    from grappstar.vision import terrain
    frames, truth = sim
    got = {}
    orig = terrain.analyze_landing
    def spy(fr, model=None, attitude=None, **kw):
        got["attitude"] = attitude
        return orig(fr, model=model, attitude=attitude, **kw)
    monkeypatch.setattr(terrain, "analyze_landing", spy)
    analysis.run_post_flight(frames, None, None, do_mosaic=False)
    att = got.get("attitude")
    assert att is None or att[0] >= truth["t_landed"]


# ------------------------------------------------------------------ fixes (added with the round 2 fixes)

def test_sim_flightlog_is_saturated_and_imu_v0_is_lower_bound(sim):
    _, truth = sim
    acc = np.array([s[5:8] for s in truth["flightlog"]["samples"]])
    assert np.abs(acc).max() <= 78.0 + 1e-6
    ev = imu_events(truth["flightlog"])
    assert ev["saturated"] and ev["v0"] is None and ev["v0_min"] is not None and ev["v0_min"] < truth["v0"]


def test_glitch_without_t_launch_ms_needs_20_ms(sim):
    frames, truth = sim
    fl = copy.deepcopy(truth["flightlog"])
    fl.pop("t_launch_ms")
    off = truth["clock_offset_s"]
    tl_ms = (truth["t_launch"] - off) * 1000.0
    k = min(range(len(fl["samples"])), key=lambda i: abs(fl["samples"][i][0] - (tl_ms - 800)))
    fl["samples"][k][7] = 50.0
    ev = imu_events(fl)
    assert abs(ev["launch"] + off - truth["t_launch"]) < 0.03
    assert abs(ev["impact"] + off - truth["t_impact"]) < 0.03


def test_impact_is_last_impact_before_stillness():
    from grappstar.comms.mock_dart import synth_flightlog
    fl = synth_flightlog(t_launch_ms=2000)
    # a bounce: a second impact 200 ms after the first one, then still
    t_land = fl["t_land_ms"]
    for s in fl["samples"]:
        if t_land + 200 <= s[0] < t_land + 240:
            s[5:8] = [-78.0, 5.0, 20.0]
    ev = imu_events(fl)
    assert ev["launch"] == pytest.approx(2.0, abs=1e-6)
    assert abs(ev["impact"] - (t_land + 200) / 1000.0) < 0.025


@pytest.mark.parametrize("shape", [(120, 160), (120, 160, 1), (120, 160, 4)])
def test_non_bgr_entry_points(shape):
    from grappstar.video.quality import frame_quality
    from grappstar.vision.flight import FlightResult, landing_frames
    from grappstar.vision.mosaic import build_mosaic
    from grappstar.vision.target import detect_orange
    img = np.random.default_rng(1).integers(0, 255, shape, dtype=np.uint8)
    frame_quality(img, img)
    detect_orange(img)
    fr = [Frame(img=img.copy(), t=i / 30.0, idx=i, quality=0.9) for i in range(5)]
    landing_frames(fr, FlightResult(t_landed=0.0))
    assert "error" not in " ".join(build_mosaic(fr, flight=FlightResult(t_launch=0.0, t_impact=0.2)).notes)


def test_build_mosaic_uses_given_flight(monkeypatch, sim):
    import grappstar.vision.mosaic as mosaic
    frames, truth = sim
    fres = analyze_flight(frames)
    monkeypatch.setattr(mosaic, "analyze_flight", lambda *a, **k: pytest.fail("second analyze_flight pass"))
    mosaic.build_mosaic(frames, flight=fres, max_frames=8)


def test_filesource_short_sidecar_continues_and_trailing_slash(tmp_path):
    from grappstar.video.record import Recorder, sidecar_path
    from grappstar.video.replay import FileSource
    rec = Recorder(str(tmp_path / "f.avi"), fps=10, force_jpeg_folder=True)
    for i in range(10):
        rec.write(Frame(img=np.full((24, 32, 3), 10 * i, np.uint8), t=500.0 + i * 0.1, idx=i))
    rec.close()
    side = sidecar_path(rec.path)
    lines = open(side).read().splitlines()[:6]
    open(side, "w").write("\n".join(lines) + "\n")
    ts = [f.t for f in FileSource(rec.path + "/", fps=10).read_all()]
    assert ts == pytest.approx([500.0 + i * 0.1 for i in range(10)])


def test_camera_ring_capped_by_bytes():
    from grappstar.video.camera import Camera
    from grappstar.video.capture import ListSource
    img = np.zeros((480, 720, 3), np.uint8)          # about 1 MB
    fr = [Frame(img=img, t=i / 100.0, idx=i) for i in range(50)]
    cam = Camera("b", lambda: ListSource(fr, loop=True), quality=False, buffer_max_mb=5, start=False)
    for f in fr:
        cam._handle(f)
    assert 1 <= cam.status()["buffer_frames"] <= 5
