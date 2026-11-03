"""Recorder -> FileSource round trip (AVI and JPEG folder fallback)."""
import json
import os
import threading

import numpy as np
import pytest

from grappstar.video import synth
from grappstar.video.frame import Frame
from grappstar.video.record import Recorder, sidecar_path
from grappstar.video.replay import FileSource


def make_frames(n=20, t0=1793000000.123456):
    base = synth.textured(240, 320, seed=0)
    out = []
    for i in range(n):
        img = np.roll(base, i * 3, axis=1)
        # Irregular spacing, like real capture jitter.
        out.append(Frame(img=img, t=t0 + i * 0.0333 + (i % 3) * 0.0041, idx=100 + i,
                         quality=0.9 - i * 0.01, bad=(i == 5)))
    return out


@pytest.mark.parametrize("force_jpeg", [False, True])
def test_round_trip_preserves_timestamps(tmp_path, force_jpeg):
    frames = make_frames()
    path = str(tmp_path / "rec" / "flight.avi")
    rec = Recorder(path, fps=30, force_jpeg_folder=force_jpeg)
    for f in frames:
        rec.write(f)
    written = rec.close()
    assert written == rec.path
    assert rec.mode == ("jpeg" if force_jpeg else "video")
    assert os.path.exists(written) and os.path.exists(sidecar_path(written))
    lines = open(sidecar_path(written)).read().splitlines()
    assert len(lines) == len(frames)
    assert set(json.loads(lines[0])) == {"i", "t", "q", "bad"}

    src = FileSource(written)
    back = src.read_all()
    src.close()
    assert len(back) == len(frames)
    for a, b in zip(frames, back):
        assert abs(a.t - b.t) < 1e-3
        assert b.img.shape == a.img.shape
        assert abs(b.quality - a.quality) < 1e-3 and b.bad == a.bad
        # lossy JPEG, but the picture must be the same one
        assert np.abs(b.img.astype(int) - a.img.astype(int)).mean() < 12


def test_replay_without_sidecar_synthesises_time(tmp_path):
    path = str(tmp_path / "x.avi")
    rec = Recorder(path, fps=25)
    for f in make_frames(5):
        rec.write(f)
    rec.close()
    os.remove(sidecar_path(path))
    src = FileSource(path, t0=0)
    ts = [f.t for f in src.read_all()]
    assert ts == pytest.approx([0, 0.04, 0.08, 0.12, 0.16])
    src2 = FileSource(path)                 # default: the file mtime is the END of the recording
    assert abs(src2.read().t - (os.path.getmtime(path) - len(src2) / src2.fps)) < 1e-6


def test_recorder_resizes_and_is_thread_safe(tmp_path):
    path = str(tmp_path / "t.avi")
    rec = Recorder(path, fps=30)
    img_a = synth.textured(120, 160)
    img_b = synth.textured(240, 320)

    def writer(k):
        for i in range(10):
            rec.write(Frame(img=img_a if k == 0 else img_b, t=float(k * 100 + i), idx=i))
    th = [threading.Thread(target=writer, args=(k,)) for k in range(3)]
    for t in th:
        t.start()
    for t in th:
        t.join()
    rec.close()
    rec.write(Frame(img=img_a, t=0.0, idx=0))       # ignored after close
    back = FileSource(path).read_all()
    assert len(back) == 30
    assert {f.img.shape for f in back} == {img_a.shape if rec.size == (160, 120) else img_b.shape}
    assert [json.loads(l)["i"] for l in open(sidecar_path(path))] == list(range(30))


def test_fallback_when_videowriter_fails(tmp_path, monkeypatch):
    from grappstar.video import record

    class BrokenWriter:
        def __init__(self, *a):
            pass

        def isOpened(self):
            return False

        def release(self):
            pass
    monkeypatch.setattr(record.cv2, "VideoWriter", BrokenWriter)
    path = str(tmp_path / "f.avi")
    rec = Recorder(path)
    for f in make_frames(3):
        rec.write(f)
    assert rec.close() == path + "_jpg"
    assert len(os.listdir(path + "_jpg")) == 3


def test_replay_realtime(tmp_path):
    import time
    path = str(tmp_path / "rt.avi")
    rec = Recorder(path)
    for i, f in enumerate(make_frames(4)):
        f.t = 100.0 + i * 0.03
        rec.write(f)
    rec.close()
    src = FileSource(path, realtime=True)
    t0 = time.time()
    assert len(src.read_all()) == 4
    assert time.time() - t0 >= 0.08


def test_missing_file():
    with pytest.raises(FileNotFoundError):
        FileSource("/nonexistent/nothing.avi")
