"""video.camera.Camera with in-memory sources."""
import time

import numpy as np
import pytest

from grappstar.video import synth
from grappstar.video.camera import Camera
from grappstar.video.capture import ListSource
from grappstar.video.frame import Frame
from grappstar.video.replay import FileSource


def live_frames(n=30, seed=0):
    """Distinct clean frames (small noise each, like a live analog picture)."""
    base = synth.textured(240, 320, seed=seed)
    return [synth.add_snow(base, 0.02, seed=i) for i in range(n)]


def wait_for(cond, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_camera_with_list_source():
    imgs = live_frames(20)
    cam = Camera("sim", lambda: ListSource(imgs, realtime=True, loop=True, restamp=True, fps=50),
                 backoff_min=0.01)
    try:
        assert cam.wait_frame(3.0) is not None
        assert wait_for(lambda: cam.status()["frames"] >= 15)
        f = cam.latest()
        assert f.quality is not None and f.quality > 0.8 and not f.bad
        st = cam.status()
        assert st["connected"] and st["resolution"] == [320, 240]
        assert 30 < st["fps"] < 70
        assert st["bad_frac"] == 0.0
        jpg = cam.jpeg()
        assert jpg[:2] == b"\xff\xd8"
        assert cam.jpeg() is jpg            # cached for the same frame
        assert len(cam.buffer(0.1)) <= len(cam.buffer()) and cam.buffer()
    finally:
        cam.stop()
    assert not cam.status()["connected"]


def test_camera_reconnects_after_factory_failures():
    calls = {"n": 0}
    imgs = live_frames(5)

    def factory():
        calls["n"] += 1
        if calls["n"] <= 3:
            raise RuntimeError("camera unplugged")
        return ListSource(imgs, loop=True, realtime=True, fps=100, restamp=True)

    cam = Camera("dart", factory, backoff_min=0.01, backoff_max=0.05)
    try:
        assert cam.wait_frame(3.0) is not None
        st = cam.status()
        assert calls["n"] == 4
        assert st["errors"] >= 3 and "unplugged" in st["last_error"]
        assert st["connected"]
    finally:
        cam.stop()


def test_camera_reconnects_when_source_returns_none():
    opened = {"n": 0}

    class Flaky:
        name = "flaky"

        def __init__(self):
            opened["n"] += 1
            self.left = 3 if opened["n"] == 1 else 10 ** 9

        def read(self):
            if self.left <= 0:
                return None            # signal lost
            self.left -= 1
            time.sleep(0.005)
            return Frame(img=synth.add_snow(synth.textured(120, 160), 0.02, seed=self.left),
                         t=time.time(), idx=0)

        def close(self):
            pass

    cam = Camera("x", Flaky, backoff_min=0.01, none_limit=5)
    try:
        assert wait_for(lambda: opened["n"] >= 2 and cam.status()["frames"] > 5)
        assert cam.status()["reconnects"] >= 1
    finally:
        cam.stop()


def test_factory_returning_none_and_read_raising():
    state = {"n": 0}

    class Boom:
        name = "boom"

        def read(self):
            raise IOError("usb error")

        def close(self):
            pass

    def factory():
        state["n"] += 1
        return None if state["n"] == 1 else Boom()

    cam = Camera("b", factory, backoff_min=0.01, backoff_max=0.02)
    try:
        assert wait_for(lambda: cam.status()["errors"] >= 4)
        assert cam.latest() is None and not cam.status()["connected"]
        assert cam.jpeg() is None and cam.buffer() == []
    finally:
        cam.stop()


def test_bad_frames_flagged_and_counted():
    good = live_frames(10)
    bad = [synth.blue_screen(240, 320)] * 10
    cam = Camera("q", lambda: ListSource(good + bad, realtime=True, fps=100, restamp=True),
                 backoff_min=5.0)
    try:
        assert wait_for(lambda: cam.status()["frames"] >= 20)
        frames = cam.buffer()
        assert [f.bad for f in frames[:10]] == [False] * 10
        assert all(f.bad for f in frames[10:20])
        assert 0.3 < cam.status()["bad_frac"] < 0.7
        assert cam.status()["quality_detail"]["dropout"] == 1.0
    finally:
        cam.stop()


def test_quality_off_and_deinterlace():
    imgs = live_frames(3)
    cam = Camera("d", lambda: ListSource(imgs, realtime=True, fps=100), quality=False,
                 deinterlace=True, backoff_min=5.0)
    try:
        f = cam.wait_frame(2.0)
        assert f.quality is None and not f.bad
        assert cam.status()["deinterlace"] == "linear"
    finally:
        cam.stop()


def test_buffer_window_and_max_fps():
    imgs = live_frames(10)
    cam = Camera("w", lambda: ListSource(imgs, loop=True, restamp=True, realtime=True, fps=200),
                 buffer_s=0.3, max_fps=40, backoff_min=0.01)
    try:
        time.sleep(0.8)
        st = cam.status()
        assert st["fps"] <= 45
        buf = cam.buffer()
        assert buf and (buf[-1].t - buf[0].t) <= 0.35
    finally:
        cam.stop()


def test_recording_round_trip(tmp_path):
    imgs = live_frames(15)
    cam = Camera("r", lambda: ListSource(imgs, realtime=True, fps=60, restamp=True),
                 backoff_min=5.0, start=False)
    path = str(tmp_path / "cam.avi")
    cam.start_recording(path, fps=60)
    assert cam.status()["recording"] == path
    cam.start()
    assert wait_for(lambda: cam.status()["frames"] >= 15)
    out = cam.stop_recording()
    assert out == path and cam.status()["recording"] is None
    seen = cam.buffer()
    cam.stop()
    back = FileSource(out).read_all()
    assert len(back) == 15
    for a, b in zip(seen, back):
        assert abs(a.t - b.t) < 1e-3 and b.quality == pytest.approx(a.quality, abs=1e-3)


# ---------------------------------------------------------------- ring sizing and the flight hold (review R1-C #4)

DART_W, DART_H, DART_FPS = 720, 480, 30.0      # the real DIGITNOW capture (C4), BGR after the GStreamer sink


def _feed(cam, img, t0, seconds, start_idx=0):
    n = int(round(seconds * DART_FPS))
    for i in range(n):
        t = t0 + (start_idx + i) / DART_FPS
        cam._handle(Frame(img=img, t=t, idx=start_idx + i), now=t)
    return start_idx + n


def _dart_img():
    return synth.textured(DART_H, DART_W, seed=3)


def test_ring_keeps_buffer_s_at_the_real_capture_rate():
    """The default (automatic) RAM cap follows buffer_s: 15 s of 720x480 at 30 fps is about 470 MB, above the old
    fixed 400 MB that held only 13.5 s."""
    cam = Camera("dart", lambda: None, quality=False, buffer_s=15, start=False)
    t0 = 1_800_000_000.0
    _feed(cam, _dart_img(), t0, 25)
    st = cam.status()
    assert st["buffer_s"] >= 14.9, st
    assert 440 < st["buffer_mb"] < 520 and st["packed_frames"] == 0
    # fixed cap (old behaviour) still available: 400 MB is about 13.5 s
    cam2 = Camera("dart", lambda: None, quality=False, buffer_s=15, start=False, buffer_max_mb=400)
    _feed(cam2, _dart_img(), t0, 25)
    assert 13.0 < cam2.status()["buffer_s"] < 14.0


def test_hold_keeps_launch_frames_through_a_long_landing_wait():
    """Fire at t_fire, hold from t_fire - 2 s; the landing wait (10 s), settle (2 s) and an operator prompt with no
    timeout (30 s more) must not lose a single frame from 2 s before the launch."""
    cam = Camera("dart", lambda: None, quality=False, buffer_s=15, start=False)
    img = _dart_img()
    t0 = 1_800_000_000.0
    k = _feed(cam, img, t0, 20)                      # sitting on the rail
    t_fire = t0 + k / DART_FPS
    cam.hold(t_fire - 2.0)
    k = _feed(cam, img, t0, 2 + 10 + 30, start_idx=k)
    held = cam.held()
    assert held[0].t == pytest.approx(t_fire - 2.0, abs=1.0 / DART_FPS)
    assert len(held) >= int((2 + 2 + 10 + 30) * DART_FPS) - 1
    idx = [f.idx for f in held]
    assert idx == list(range(idx[0], idx[0] + len(idx)))        # no gaps
    st = cam.status()
    assert st["packed_frames"] > 0 and st["dropped_held"] == 0
    assert st["buffer_mb"] < 1200 * 1.05
    # packed frames come back as full images, close to the original
    f = held[0]
    assert f.img.shape == (DART_H, DART_W, 3)
    assert float(np.abs(f.img.astype(int) - img.astype(int)).mean()) < 4.0
    # the analysis reads the same frames through buffer()
    frames = [f for f in cam.buffer() if f.t >= t_fire - 2.0]
    assert len(frames) == len(held)
    # release: back to the normal time window on the next frames
    cam.release_hold()
    _feed(cam, img, t0, 1, start_idx=k)
    st = cam.status()
    assert st["buffer_s"] <= 15.0 + 1e-6 and st["hold"] is None and cam.held() == []


def test_hold_with_fixed_cap_packs_instead_of_dropping():
    cam = Camera("dart", lambda: None, quality=False, buffer_s=15, start=False, buffer_max_mb=100)
    img = _dart_img()
    t0 = 1_800_000_000.0
    k = _feed(cam, img, t0, 3)
    cam.hold(t0 + 1.0)
    _feed(cam, img, t0, 20, start_idx=k)
    held = cam.held()
    assert len(held) == int((3 - 1.0 + 20) * DART_FPS)
    assert cam.status()["buffer_mb"] <= 101


def test_forgotten_hold_expires():
    cam = Camera("dart", lambda: None, quality=False, buffer_s=2, start=False, hold_max_s=0.0)
    cam.hold(0.0)
    _feed(cam, synth.textured(48, 64, seed=1), time.time() + 1.0, 4)
    assert cam.status()["hold"] is None and cam.status()["buffer_s"] <= 2.0 + 1e-6
