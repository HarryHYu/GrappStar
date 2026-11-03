"""Whole-system simulator for the --sim operator page and the mission tests.

Joins a MockBase (C1), a MockDart (C2, HTTP on 127.0.0.1) and two synthetic cameras:
- dart camera: a pre-rendered synthetic flight (sim.flight_video). Before the shot it shows the rail view; when the
  base fires, it plays the flight in real time, then keeps showing the landed view.
- rover camera: a ground plane with an orange marker seen through the default rover CameraModel, so a click on the
  marker gives a real range and bearing.
When the base fires, the dart "launches", lands after the simulated flight time, the line pays out to the range, and
after landing the dart lies free (or anchored after /auto).
"""
from __future__ import annotations

import math
import threading
import time
from typing import Optional

import numpy as np

from ..comms.base import BaseClient, pipe_pair
from ..comms.mock_base import MockBase, World
from ..comms.mock_dart import MockDart
from ..video.frame import Frame


def default_rover_model(width: int = 1280, height: int = 720, height_m: float = 0.45, pitch_deg: float = 12.0):
    """Kiyo X default: about 82 deg diagonal FOV at 16:9, mounted 0.45 m up, 12 deg down (guesses until calibrated)."""
    from ..vision.target import CameraModel
    diag = math.hypot(width, height)
    f = (diag / 2.0) / math.tan(math.radians(82.0) / 2.0)
    return CameraModel(fx=f, fy=f, cx=width / 2.0 - 0.5, cy=height / 2.0 - 0.5, height_m=height_m, pitch_deg=pitch_deg,
                       calib_size=(width, height), uncalibrated=True, notes="default Kiyo X guess, not calibrated")


def render_rover_view(model, size=(640, 360), marker=(8.0, 6.0), seed: int = 0) -> np.ndarray:
    """Rover camera view of flat ground (textured) with an orange marker at (range_m, bearing_deg)."""
    import cv2
    w, h = size
    m = model.for_size(w, h)
    rng = np.random.default_rng(seed)
    res = 0.02                                  # ground texture: 2 cm per pixel, X -10..10 m, Y 0..25 m
    gw, gh = 1000, 1250
    base = rng.normal(0, 1, (gh // 25 + 2, gw // 25 + 2)).astype(np.float32)
    tex = cv2.resize(base, (gw, gh), interpolation=cv2.INTER_CUBIC)
    tex += 0.35 * rng.normal(0, 1, (gh, gw)).astype(np.float32)
    g = np.clip(110 + 22 * tex, 0, 255).astype(np.uint8)
    ground = cv2.merge([(g * 0.75).astype(np.uint8), (g * 0.9).astype(np.uint8), g])
    # orange marker (a 0.3 m disc) at the target
    r_m, b_deg = marker
    mx, my = r_m * math.sin(math.radians(b_deg)), r_m * math.cos(math.radians(b_deg))
    col, row = (mx + 10.0) / res, (25.0 - my) / res
    cv2.circle(ground, (int(col), int(row)), int(0.15 / res), (0, 110, 255), -1)

    def gpix(X, Y):
        return ((X + 10.0) / res, (25.0 - Y) / res)
    src, dst = [], []
    for X, Y in ((-3, 1.5), (3, 1.5), (-6, 20), (6, 20)):
        p = m.world_to_pixel(X, Y, 0.0)
        if p is None:
            continue
        src.append(gpix(X, Y))
        dst.append(p)
    H = cv2.getPerspectiveTransform(np.float32(src), np.float32(dst))
    img = cv2.warpPerspective(ground, H, (w, h), flags=cv2.INTER_LINEAR, borderValue=(0, 0, 0))
    hv = int(max(0, min(h, m.horizon_v())))
    sky = np.zeros((h, w, 3), np.uint8)
    sky[:] = (235, 200, 160)
    mask = np.zeros((h, w), bool)
    mask[:hv] = True
    img[mask] = sky[mask]
    return img


class _LoopSource:
    """Frame source that calls a function for each frame, paced at fps (never two identical frames)."""

    def __init__(self, fn, fps: float, name: str):
        self.fn, self.period, self.name = fn, 1.0 / fps, name
        self.idx = 0
        self.t_next = time.time()

    def read(self) -> Optional[Frame]:
        now = time.time()
        if now < self.t_next:
            time.sleep(self.t_next - now)
        self.t_next = max(self.t_next + self.period, time.time() - self.period)
        self.idx += 1
        img = self.fn()
        return Frame(img=img, t=time.time(), idx=self.idx)

    def close(self) -> None:
        pass


class SimWorld:
    def __init__(self, seed: int = 0, landing_class: str = "hard_ground", time_scale: float = 1.0,
                 dart_size=(480, 320), rover_size=(640, 360), target=(8.0, 6.0), fps: float = 25.0,
                 anchor_holds: bool = True):
        self.seed, self.time_scale, self.fps = seed, time_scale, fps
        self.anchor_holds = anchor_holds
        self.world = World()
        self.world.on_fire = self._on_fire
        a, b = pipe_pair()
        self.mock_base = MockBase(a, time_scale=time_scale, world=self.world).start()
        self.base_end = b
        self.mock_dart = MockDart(time_scale=time_scale).start()
        self.dart_host = self.mock_dart.host
        self.t_fire: Optional[float] = None
        self.target = target
        self.rover_model = default_rover_model()
        self._rover_img = render_rover_view(self.rover_model, rover_size, marker=target, seed=seed)
        self._rng = np.random.default_rng(seed)
        self.flight = None
        self.truth = None
        self._ready = threading.Event()
        self.landing_class = landing_class
        self.dart_size = dart_size
        threading.Thread(target=self._prerender, name="sim-render", daemon=True).start()
        self._stop = threading.Event()
        threading.Thread(target=self._sync, name="sim-sync", daemon=True).start()

    def _prerender(self) -> None:
        from .flight_video import render_flight
        frames, truth = render_flight(seed=self.seed, fps=self.fps, landing_class=self.landing_class,
                                      size=self.dart_size, dropouts=True, landed_s=3.0)
        self.flight, self.truth = frames, truth
        tl = truth["t_launch"]
        self._rail = [f for f in frames if f.t < tl - 0.05] or frames[:1]
        self._landed = [f for f in frames if f.t > truth["t_landed"]] or frames[-1:]
        self.flight_s = truth["t_impact"] - tl
        self.range_m = float(truth.get("range_m", 8.0))
        self.mock_dart.flightlog_provider = lambda: truth["flightlog"]
        self._ready.set()

    def wait_ready(self, timeout: float = 30.0) -> bool:
        return self._ready.wait(timeout)

    # ------------------------------------------------ world events
    def _on_fire(self) -> None:
        self.wait_ready(30)
        self.t_fire = time.time()
        self.mock_dart.launch()
        fs = self.flight_s / self.time_scale
        rng_m = self.range_m
        self.world.flight = lambda t: rng_m * min(1.0, t / max(self.flight_s, 0.1))
        threading.Timer(fs, self._on_land).start()

    def _on_land(self) -> None:
        self.mock_dart.land()
        self.world.dart = "free"

    def _sync(self) -> None:
        """Keep the base world in step with the dart (anchored, released)."""
        last = None
        while not self._stop.is_set():
            ph = self.mock_dart.phase
            if ph != last:
                if ph == "ANCHORED" and self.anchor_holds:
                    self.world.dart = "anchored"
                    self.world.slack_m = 0.3
                elif ph in ("RELEASING", "IDLE", "FAULT") and self.world.dart == "anchored":
                    self.world.dart = "free"
                last = ph
            self._stop.wait(0.05)

    def rover_advance(self, m: float) -> None:
        """The rover drove m metres toward the dart: slack appears in the line."""
        self.world.slack_m += m
        self.mock_base.tension = 1.0

    # ------------------------------------------------ cameras
    def _dart_frame(self) -> np.ndarray:
        if not self._ready.is_set():
            import cv2
            img = np.full((self.dart_size[1], self.dart_size[0], 3), 40, np.uint8)
            cv2.putText(img, "sim: rendering flight...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            return self._jitter(img)
        tl = self.truth["t_launch"]
        if self.t_fire is None:
            i = int(time.time() * self.fps) % len(self._rail)
            return self._jitter(self._rail[i].img)
        vt = tl + (time.time() - self.t_fire) * self.time_scale
        if vt >= self._landed[0].t:
            i = int(time.time() * self.fps) % len(self._landed)
            return self._jitter(self._landed[i].img)
        best = min(self.flight, key=lambda f: abs(f.t - vt))
        return best.img.copy()

    def _jitter(self, img: np.ndarray) -> np.ndarray:
        n = self._rng.integers(-2, 3, img.shape, dtype=np.int16)
        return np.clip(img.astype(np.int16) + n, 0, 255).astype(np.uint8)

    def dart_source(self):
        return _LoopSource(self._dart_frame, self.fps, "sim-dartcam")

    def rover_source(self):
        return _LoopSource(lambda: self._jitter(self._rover_img), 15.0, "sim-rovercam")

    def base_client(self) -> BaseClient:
        return BaseClient(self.base_end, reconnect_s=0.1)

    def close(self) -> None:
        self._stop.set()
        self.mock_base.stop()
        self.mock_dart.stop()
