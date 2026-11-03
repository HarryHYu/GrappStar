"""video.capture with a mocked cv2.VideoCapture (no camera on the test machine)."""
import os
import time

import cv2
import numpy as np
import pytest

from grappstar.video import capture
from grappstar.video.capture import (V4L2Source, GstSource, ListSource, resolve_device,
                                     fourcc_to_str, gst_pipeline_yuyv, gst_pipeline_mjpeg,
                                     has_gstreamer)
from grappstar.video.frame import Frame

NAMES = {cv2.CAP_PROP_FOURCC: "FOURCC", cv2.CAP_PROP_FRAME_WIDTH: "WIDTH",
         cv2.CAP_PROP_FRAME_HEIGHT: "HEIGHT", cv2.CAP_PROP_FPS: "FPS",
         cv2.CAP_PROP_BUFFERSIZE: "BUFFERSIZE"}


class FakeCapture:
    """Stands in for cv2.VideoCapture and records every call."""
    instances = []

    def __init__(self, target, backend=None, opened=True):
        self.target, self.backend = target, backend
        self.calls = []
        self.props = {}
        self.opened = opened
        self.reads = 0
        FakeCapture.instances.append(self)

    def isOpened(self):
        return self.opened

    def set(self, prop, value):
        self.calls.append((NAMES.get(prop, prop), value))
        self.props[prop] = value
        return True

    def get(self, prop):
        return float(self.props.get(prop, 0))

    def read(self):
        self.reads += 1
        if self.reads > 3:
            return False, None
        return True, np.full((480, 720, 3), self.reads, np.uint8)

    def release(self):
        self.opened = False


@pytest.fixture
def fake(monkeypatch):
    FakeCapture.instances = []
    monkeypatch.setattr(capture.cv2, "VideoCapture", FakeCapture)
    return FakeCapture


@pytest.mark.parametrize("cc", ["YUYV", "MJPG"])
def test_v4l2_sets_fourcc_before_size(fake, cc):
    src = V4L2Source(0, fourcc=cc, width=720, height=480, fps=30)
    cap = fake.instances[-1]
    assert cap.backend == cv2.CAP_V4L2
    order = [name for name, _ in cap.calls]
    assert order.index("FOURCC") < order.index("WIDTH") < order.index("HEIGHT") < order.index("FPS")
    assert order[0] == "FOURCC"
    assert cap.calls[0][1] == cv2.VideoWriter_fourcc(*cc)
    # info reports what the driver gives back
    assert src.info["fourcc"] == cc and src.info["width"] == 720 and src.info["height"] == 480


def test_v4l2_read_and_close(fake):
    src = V4L2Source("/dev/video0", "MJPEG")
    assert src.fourcc == "MJPG"
    frames = [src.read() for _ in range(4)]
    assert [f.idx for f in frames[:3]] == [0, 1, 2]
    assert frames[0].img.shape == (480, 720, 3)
    assert abs(frames[0].t - time.time()) < 5
    assert frames[3] is None
    src.close()
    assert src.read() is None


def test_v4l2_open_failure_raises(monkeypatch):
    monkeypatch.setattr(capture.cv2, "VideoCapture",
                        lambda *a: FakeCapture(*a, opened=False))
    with pytest.raises(RuntimeError):
        V4L2Source(5)


def test_resolve_device(tmp_path):
    assert resolve_device(2) == 2
    assert resolve_device("3") == 3
    assert resolve_device("/dev/video7") == 7
    link = tmp_path / "grappstar-dartcam"          # like a udev symlink
    os.symlink("/dev/video4", str(link))
    assert resolve_device(str(link)) == 4
    byid = tmp_path / "usb-MACROSILICON_USB_Video-video-index0"
    os.symlink("/dev/video2", str(byid))
    assert resolve_device(str(byid)) == 2


def test_fourcc_to_str():
    assert fourcc_to_str(cv2.VideoWriter_fourcc(*"YUYV")) == "YUYV"
    assert fourcc_to_str(0) == ""


def test_gst_pipelines():
    p = gst_pipeline_yuyv("/dev/video0", 720, 480, 30)
    assert p.startswith("v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=720,height=480,framerate=30/1")
    assert p.endswith("appsink drop=true max-buffers=1 sync=false")
    m = gst_pipeline_mjpeg("/dev/video1", 1920, 1080, 30, decoder="nvjpegdec")
    assert "image/jpeg,width=1920" in m and "nvjpegdec" in m and "format=BGR" in m
    assert "jpegdec" in gst_pipeline_mjpeg("/dev/video1")
    with pytest.raises(ValueError):
        gst_pipeline_mjpeg("/dev/video1", decoder="bogus")


def test_has_gstreamer(monkeypatch):
    monkeypatch.setattr(capture.cv2, "getBuildInformation",
                        lambda: "  Video I/O:\n    GStreamer:                   YES (1.20.3)\n")
    assert has_gstreamer()
    monkeypatch.setattr(capture.cv2, "getBuildInformation",
                        lambda: "  Video I/O:\n    GStreamer:                   NO\n")
    assert not has_gstreamer()


def test_gst_source_uses_gstreamer_backend(fake):
    src = GstSource("videotestsrc ! appsink", name="t")
    assert fake.instances[-1].backend == cv2.CAP_GSTREAMER
    assert src.read().idx == 0
    src.close()


def _frames(n, t0=1000.0, dt=0.02):
    return [Frame(img=np.full((4, 4, 3), i, np.uint8), t=t0 + i * dt, idx=i) for i in range(n)]


def test_list_source_basic():
    src = ListSource(_frames(3))
    out = [src.read() for _ in range(4)]
    assert [f.t for f in out[:3]] == [1000.0, 1000.02, 1000.04]
    assert out[3] is None
    out[0].bad = True
    assert src.frames[0].bad is False     # originals untouched


def test_list_source_loop_and_restamp():
    src = ListSource(_frames(2), loop=True, restamp=True)
    out = [src.read() for _ in range(5)]
    assert [f.idx for f in out] == [0, 1, 2, 3, 4]
    assert all(abs(f.t - time.time()) < 1 for f in out)


def test_list_source_realtime_pacing():
    src = ListSource(_frames(6, dt=0.02), realtime=True)
    t0 = time.time()
    while src.read() is not None:
        pass
    assert time.time() - t0 >= 0.09        # 5 gaps of 20 ms


def test_list_source_from_images():
    src = ListSource([np.zeros((4, 4), np.uint8)] * 2, fps=10)
    a, b = src.read(), src.read()
    assert a.img.shape == (4, 4, 3)
    assert abs((b.t - a.t) - 0.1) < 1e-6
