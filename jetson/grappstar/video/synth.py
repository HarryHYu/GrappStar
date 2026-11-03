"""Synthetic frames that look like (bad) analog FPV video.

Used by the tests and handy for the simulator or for tuning video.quality
without a camera. Every function is deterministic for a given seed.
"""
import numpy as np
import cv2


def textured(h=480, w=720, seed=0):
    """A natural looking scene: sky gradient on top, multi-scale rocky texture below."""
    rng = np.random.default_rng(seed)
    img = np.zeros((h, w), np.float32)
    # Sum of blurred noise at several scales gives a rocky, natural texture.
    for scale, amp in ((64, 60.0), (16, 35.0), (4, 20.0)):
        small = rng.standard_normal((max(2, h // scale), max(2, w // scale))).astype(np.float32)
        img += cv2.resize(small, (w, h), interpolation=cv2.INTER_CUBIC) * amp
    img += 110.0
    horizon = int(h * (0.25 + 0.1 * rng.random()))
    sky = np.linspace(200, 160, horizon, dtype=np.float32)[:, None]
    img[:horizon] = sky
    grey = np.clip(img, 0, 255).astype(np.uint8)
    bgr = cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR).astype(np.int16)
    bgr[:horizon, :, 0] += 30           # bluish sky
    bgr[horizon:, :, 2] += 15           # warm ground
    # A few hard edged shapes (rocks / the rover) for good measure.
    for _ in range(4):
        cx, cy = int(rng.integers(0, w)), int(rng.integers(horizon, h))
        r = int(rng.integers(10, 50))
        cv2.circle(bgr, (cx, cy), r, (int(rng.integers(30, 220)),) * 3, -1)
    return np.clip(bgr, 0, 255).astype(np.uint8)


def add_snow(img, amount=0.5, seed=1):
    """Analog 'snow': gaussian noise plus random white/black speckles. amount 0..1."""
    rng = np.random.default_rng(seed)
    out = img.astype(np.float32)
    out += rng.normal(0, 60 * amount, img.shape[:2])[..., None]
    mask = rng.random(img.shape[:2]) < 0.25 * amount
    out[mask] = rng.choice([0.0, 255.0], size=int(mask.sum()))[:, None]
    return np.clip(out, 0, 255).astype(np.uint8)


def blue_screen(h=480, w=720):
    """What many capture dongles output when the signal is lost."""
    img = np.zeros((h, w, 3), np.uint8)
    img[:] = (255, 0, 0)
    return img


def black_screen(h=480, w=720, noise=1.0, seed=2):
    """Near black frame with a little sensor noise."""
    rng = np.random.default_rng(seed)
    n = rng.normal(8, noise, (h, w, 1))
    return np.clip(np.repeat(n, 3, axis=2), 0, 255).astype(np.uint8)


def torn(img, bands=4, max_shift=120, seed=3):
    """Sync loss: horizontal bands of rows shifted sideways by different amounts."""
    rng = np.random.default_rng(seed)
    out = img.copy()
    h = img.shape[0]
    edges = np.sort(rng.choice(np.arange(10, h - 10), size=bands * 2, replace=False))
    for a, b in zip(edges[0::2], edges[1::2]):
        shift = int(rng.integers(max_shift // 3, max_shift)) * int(rng.choice([-1, 1]))
        out[a:b] = np.roll(img[a:b], shift, axis=1)
    return out


def rolled(img, offset_frac=0.35, bar_frac=0.08):
    """Vertical roll: the picture slides down and the black blanking bar shows."""
    h = img.shape[0]
    out = np.roll(img, int(h * offset_frac), axis=0)
    a = int(h * offset_frac) - int(h * bar_frac)
    out[max(0, a):int(h * offset_frac)] = 0
    return out
