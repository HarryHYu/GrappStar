"""What the analog dart video chain does to a picture: Caddx Ant camera, 5.8 GHz FM link, RC832 receiver, USB
capture dongle. Used by sim.flight_video (profile "caddx") so the vision code is tested on frames that look like
the real ones, not on clean renders.

Modelled, in the order the signal meets them:

1. Camera exposure (auto exposure, AE). The camera shortens the exposure in bright light and lengthens it up to one
   field (1/60 s) in low light, then raises the gain (AGC, up to about 8x). The AE follows a change of brightness
   with a time lag (about 0.25 s): when the dart turns from the sky to the ground in flight the picture is too
   dark or too bright for a few frames. A long exposure smears the picture while the dart moves (motion blur: the
   caller renders sub-frames across the exposure and averages them).
2. Sensor noise: shot noise (grows with the square root of the light) and read noise times the gain. In low light
   the camera also washes out the colours.
3. Camera processing: tone curve, edge sharpening (the white and dark halos analog FPV cameras show), lens
   vignetting (darker corners with the fisheye).
4. Composite video (NTSC): the brightness signal has about 4.2 MHz (a little horizontal softness), the colour only
   about 1 MHz and arrives a little late (colour smeared sideways and shifted by about 1.5 px).
5. The FM link: noise that grows when the signal is weak (the dart lying on the ground with its antenna in the
   grass: more noise after landing), "sparkles" (short white or black streaks), small sideways line jitter.
   Full dropouts, snow and torn frames are added by flight_video itself (it knows when they happen).
6. Interlacing: each frame is two fields 1/60 s apart, the even rows from the first, the odd rows from the second.
   When the dart moves the two disagree (comb edges).
7. The capture dongle: YUYV 4:2:2 (colour at half the horizontal resolution) or MJPG (JPEG blocks).

The numbers are typical for a 25 mW FPV link at a few metres and a 1200TVL class camera. They are not measured on
Harry's parts: record real clips and compare (README section "Re-learn on your own footage").
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Optional

import cv2
import numpy as np

FIELD_S = 1.0 / 59.94           # NTSC field period
BASE_EXPOSURE_S = 1.0 / 2000    # exposure that gives a mid-grey picture at gain 1 in full daylight (scene mean 0.18)


@dataclass
class LinkConfig:
    light: float = 1.0              # scene light: 1 full daylight, 0.3 overcast, 0.05 dusk / deep shade
    gain_max: float = 8.0           # AGC limit
    ae_tau_s: float = 0.25          # AE time lag
    shot_noise: float = 0.010       # at full scale, before gain
    read_noise: float = 0.004       # times the gain
    sharpen: float = 0.6            # unsharp mask amount (halos)
    vignette: float = 0.35          # brightness lost at the image corner (fisheye)
    luma_sigma: float = 0.6         # px, horizontal blur of the brightness signal
    chroma_sigma: float = 2.2       # px, horizontal blur of the colour signal
    chroma_shift: float = 1.5       # px, colour arrives late
    rf_noise: float = 0.012         # link noise (fraction of full scale) with a strong signal
    rf_noise_landed: float = 0.030  # with the dart on the ground (weak signal)
    sparkle_rate: float = 2e-5      # sparkles per pixel per field with a strong signal
    line_jitter: float = 0.35       # px, sideways jitter per line (sd)
    interlace: bool = True
    capture: str = "yuyv"           # "yuyv" or "mjpg"
    jpeg_quality: int = 75

    @classmethod
    def preset(cls, name: str) -> "LinkConfig":
        """Named conditions for tests: daylight, overcast, lowlight, weak_link, clean."""
        if name == "daylight":
            return cls()
        if name == "overcast":
            return cls(light=0.3)
        if name == "lowlight":
            return cls(light=0.05, rf_noise=0.016)
        if name == "weak_link":
            return cls(rf_noise=0.03, rf_noise_landed=0.06, sparkle_rate=2e-4, line_jitter=0.8)
        if name == "clean":
            return cls(shot_noise=0.0, read_noise=0.0, sharpen=0.0, vignette=0.0, luma_sigma=0.0, chroma_sigma=0.0,
                       chroma_shift=0.0, rf_noise=0.0, rf_noise_landed=0.0, sparkle_rate=0.0, line_jitter=0.0,
                       interlace=False, capture="none")
        raise ValueError(f"unknown link preset {name!r}")


_LUT_TO_LIN = ((np.arange(256) / 255.0) ** 2.2).astype(np.float32)


def to_linear(img_u8: np.ndarray) -> np.ndarray:
    return _LUT_TO_LIN[img_u8]


@dataclass
class AutoExposure:
    """AE state: the camera's amplification A = exposure gain x AGC gain, following the scene with a lag."""
    cfg: LinkConfig
    log_a: Optional[float] = None
    t: Optional[float] = None

    def update(self, t: float, scene_mean_lin: float) -> dict:
        """Advance to time t with this scene brightness (linear, before light). Returns exposure_s and gain."""
        need = 0.18 / max(1e-4, scene_mean_lin * self.cfg.light)
        target = math.log(need)
        if self.log_a is None:
            self.log_a = target
        else:
            dt = max(0.0, t - (self.t if self.t is not None else t))
            k = 1.0 - math.exp(-dt / max(1e-3, self.cfg.ae_tau_s))
            self.log_a += (target - self.log_a) * k
        self.t = t
        a = math.exp(self.log_a)
        a = min(a, (FIELD_S / BASE_EXPOSURE_S) * self.cfg.gain_max)      # cannot amplify beyond max exposure x gain
        exposure = min(FIELD_S, max(1e-4, a * BASE_EXPOSURE_S))
        gain = max(1.0, a * BASE_EXPOSURE_S / exposure)
        return {"amp": a, "exposure_s": exposure, "gain": gain}


def _vignette_map(h: int, w: int, strength: float) -> np.ndarray:
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    r = np.hypot((xx - (w - 1) / 2) / (w / 2), (yy - (h - 1) / 2) / (h / 2)) / math.sqrt(2)
    return (1.0 - strength * r ** 2).astype(np.float32)


def _randn(shape) -> np.ndarray:
    """Standard normal float32 noise (OpenCV's generator: about 10x faster than numpy's)."""
    a = np.empty(shape, np.float32)
    cv2.randn(a, 0.0, 1.0)
    return a


class AnalogChain:
    """Turns rendered scene fields (uint8 BGR, the scene at full daylight and gain 1) into captured frames."""

    def __init__(self, size, cfg: LinkConfig, rng: np.random.Generator):
        self.w, self.h = int(size[0]), int(size[1])
        self.cfg = cfg
        self.rng = rng
        cv2.setRNGSeed(int(rng.integers(0, 2 ** 31 - 1)))     # reproducible noise for a seed
        self._mapy = np.repeat(np.arange(self.h, dtype=np.float32)[:, None], self.w, axis=1)
        self._xs = np.arange(self.w, dtype=np.float32)[None, :]
        self.ae = AutoExposure(cfg)
        self.vig = _vignette_map(self.h, self.w, cfg.vignette) if cfg.vignette > 0 else None

    def exposure_for(self, t: float, scene_u8: np.ndarray) -> dict:
        """Update the AE with this field's scene and return {exposure_s, gain, amp}."""
        small = scene_u8[::8, ::8]
        m = float(to_linear(small).mean())
        return self.ae.update(t, m)

    def field(self, scene_lin: np.ndarray, ae: dict, landed: bool) -> np.ndarray:
        """One field: scene (linear float32 0..1, maybe exposure-averaged) -> camera output, float32 0..1 BGR."""
        c = self.cfg
        rng = self.rng
        x = scene_lin * (c.light * ae["amp"])
        if c.shot_noise > 0 or c.read_noise > 0:
            # shot noise is set by the light actually collected (before the AGC gain)
            collected = np.clip(x / ae["gain"], 0, 1)
            sd = np.sqrt(c.shot_noise ** 2 * collected) * ae["gain"]
            sd = np.sqrt(sd * sd + (c.read_noise * ae["gain"]) ** 2)
            n = _randn((self.h, self.w))                                    # mostly luma noise
            nc = cv2.resize(_randn((self.h // 2, self.w // 2, 3)), (self.w, self.h),
                            interpolation=cv2.INTER_LINEAR) * 0.5
            x = x + sd * (n[..., None] + nc)
        if self.vig is not None:
            x = x * self.vig[..., None]
        x = np.clip(x, 0, 1.2)
        y = np.power(np.clip(x, 0, None), 1 / 2.2)                           # camera tone curve
        if ae["gain"] > 2.0:                                                   # washed out colours at high gain
            s = 1.0 - 0.5 * min(1.0, (ae["gain"] - 2.0) / 6.0)
            g = y.mean(axis=2, keepdims=True)
            y = g + (y - g) * s
        if c.sharpen > 0:
            b = cv2.GaussianBlur(y, (0, 0), 1.2)
            y = y + c.sharpen * (y - b)
        return np.clip(y, 0, 1)

    def composite(self, y: np.ndarray, landed: bool) -> np.ndarray:
        """Composite encoding, FM link and receiver: float32 0..1 BGR in, float32 out."""
        c = self.cfg
        rng = self.rng
        ycc = cv2.cvtColor(y.astype(np.float32), cv2.COLOR_BGR2YCrCb)
        if c.luma_sigma > 0:
            ycc[..., 0] = cv2.GaussianBlur(ycc[..., 0], (0, 1), sigmaX=c.luma_sigma, sigmaY=0.01)
        if c.chroma_sigma > 0 or c.chroma_shift:
            for k in (1, 2):
                ch = ycc[..., k]
                if c.chroma_sigma > 0:
                    ch = cv2.GaussianBlur(ch, (0, 1), sigmaX=c.chroma_sigma, sigmaY=0.01)
                if c.chroma_shift:
                    M = np.float32([[1, 0, c.chroma_shift], [0, 1, 0]])
                    ch = cv2.warpAffine(ch, M, (self.w, self.h), borderMode=cv2.BORDER_REPLICATE)
                ycc[..., k] = ch
        rf = c.rf_noise_landed if landed else c.rf_noise
        if rf > 0:
            n = _randn((self.h, self.w))
            # FM noise is mostly in the brightness, a little horizontally correlated
            n = cv2.GaussianBlur(n, (0, 1), sigmaX=0.7, sigmaY=0.01) * 1.4
            ycc[..., 0] += rf * n
            ycc[..., 1:] += 0.5 * rf * _randn((self.h, self.w, 2))
        rate = c.sparkle_rate * (3.0 if landed else 1.0)
        if rate > 0:
            k = rng.poisson(rate * self.h * self.w)
            for _ in range(int(k)):
                r0, x0 = int(rng.integers(0, self.h)), int(rng.integers(0, self.w))
                ln = int(rng.integers(3, 14))
                ycc[r0, x0:x0 + ln, 0] = 1.0 if rng.random() < 0.6 else 0.0
        out = cv2.cvtColor(ycc, cv2.COLOR_YCrCb2BGR)
        if c.line_jitter > 0:
            sh = rng.normal(0, c.line_jitter, self.h).astype(np.float32)
            mapx = self._xs + sh[:, None]
            out = cv2.remap(out, mapx, self._mapy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        return out

    def weave(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Interlaced frame: even rows from field a (earlier), odd rows from field b (1/60 s later)."""
        if not self.cfg.interlace or b is None:
            return a
        out = a.copy()
        out[1::2] = b[1::2]
        return out

    def capture(self, y: np.ndarray) -> np.ndarray:
        """The capture dongle: float 0..1 BGR -> uint8 BGR as the Jetson sees it."""
        img = np.clip(y * 255.0 + 0.5, 0, 255).astype(np.uint8)
        c = self.cfg
        if c.capture == "yuyv":
            ycc = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
            ch = ycc[:, 0::2, 1:]
            ycc[:, :, 1:] = np.repeat(ch, 2, axis=1)[:, :self.w]
            img = cv2.cvtColor(ycc, cv2.COLOR_YCrCb2BGR)
        elif c.capture == "mjpg":
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(c.jpeg_quality)])
            if ok:
                img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return img
