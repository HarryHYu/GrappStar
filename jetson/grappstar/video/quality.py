"""Per-frame quality score for analog 5.8 GHz video (RC832 receiver + cheap capture dongle).

frame_quality(img, prev=None) returns a dict:
    score      0..1, 1 = clean picture. bad = score < BAD_BELOW (0.4).
    snow       0..1, random speckle noise (weak signal)
    dropout    0..1, near uniform frame (blue or black "no signal" screen)
    sync_loss  0..1, torn rows (horizontal shifts in bands) or a vertical roll bar
    freeze     0 or 1, identical to the previous frame (dongle repeating a frame)
    plus raw measurements (noise_frac, std, tear_rows, roll_bar) for tuning.

Everything runs on a ~160 px wide subsampled grey copy so it stays well under 3 ms.
We subsample with plain slicing (nearest pixel) rather than averaging, because
averaging would smooth away exactly the speckle noise we want to measure.
"""
from typing import Optional

import cv2
import numpy as np

from .frame import ensure_bgr

BAD_BELOW = 0.4
TARGET_W = 160


def _small_grey(img: np.ndarray) -> np.ndarray:
    """Subsample to about TARGET_W px wide and convert to grey (int16 for safe maths)."""
    step = max(1, img.shape[1] // TARGET_W)
    sub = img[::step, ::step]
    if sub.ndim == 3:
        sub = cv2.cvtColor(np.ascontiguousarray(sub), cv2.COLOR_BGR2GRAY)
    return sub


def _ramp(x: float, lo: float, hi: float) -> float:
    """0 below lo, 1 above hi, linear in between."""
    if hi <= lo:
        return float(x >= hi)
    return float(min(1.0, max(0.0, (x - lo) / (hi - lo))))


def frame_quality(img: np.ndarray, prev: Optional[np.ndarray] = None) -> dict:
    img = ensure_bgr(img)
    if prev is not None:
        prev = ensure_bgr(prev)
    g = _small_grey(img)
    gi = g.astype(np.int16)

    # --- dropout: a "no signal" screen is nearly uniform (tiny variance).
    # Colour matters too: a pure blue screen has a uniform grey value as well.
    std = float(gi.std())
    dropout = 1.0 - _ramp(std, 4.0, 12.0)

    # --- snow: residual after a 3x3 median blur. Real texture survives a median
    # filter, isolated speckles do not. We count pixels whose residual is large.
    med = cv2.medianBlur(g, 3).astype(np.int16)
    resid = np.abs(gi - med)
    noise_frac = float((resid > 30).mean())
    snow = _ramp(noise_frac, 0.03, 0.15)

    # --- sync loss (tearing): compare each row with the next one.
    # pix = mean absolute pixel difference between neighbouring rows
    # mean = difference of the row averages
    # A band shifted sideways keeps its row average, so pix jumps while mean
    # barely moves. A real horizon changes both, so it is not counted.
    rows = gi.astype(np.float32)
    pix = np.abs(rows[1:] - rows[:-1]).mean(axis=1)
    mean_d = np.abs(rows[1:].mean(axis=1) - rows[:-1].mean(axis=1))
    tear = pix - mean_d
    base = float(np.median(tear)) + 1.0
    tear_rows = int((tear > 2.0 * base + 6.0).sum())
    tear_score = _ramp(tear_rows, 1.5, 4.5)

    # --- vertical roll: a band of black, flat rows (the blanking interval) inside an
    # otherwise normal picture.
    row_mean = rows.mean(axis=1)
    row_std = rows.std(axis=1)
    flat_dark = (row_mean < 20) & (row_std < 4)
    roll_frac = float(flat_dark.mean())
    roll_bar = 0.02 < roll_frac < 0.6 and dropout < 0.5
    sync_loss = max(tear_score, 1.0 if roll_bar else 0.0)

    # --- freeze: exactly the same picture as last time. Analog video always has some
    # noise, so a bit-identical frame means the dongle repeated an old one.
    freeze = 0.0
    if prev is not None and prev.shape == img.shape:
        gp = _small_grey(prev)
        if gp.shape == g.shape and not np.any(gp != g):
            freeze = 1.0

    score = (1.0 - snow) * (1.0 - dropout) * (1.0 - sync_loss)
    if freeze:
        score *= 0.35
    score = float(min(1.0, max(0.0, score)))
    return {
        "score": round(score, 3),
        "bad": score < BAD_BELOW,
        "snow": round(snow, 3),
        "dropout": round(dropout, 3),
        "sync_loss": round(sync_loss, 3),
        "freeze": freeze,
        "noise_frac": round(noise_frac, 4),
        "std": round(std, 2),
        "tear_rows": tear_rows,
        "roll_bar": bool(roll_bar),
    }


def is_bad(q: dict) -> bool:
    return q["score"] < BAD_BELOW
