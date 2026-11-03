"""Simple deinterlacing for NTSC/PAL frames from the analog capture dongle.

An interlaced frame holds two "fields" captured 1/60 s apart: the even rows
(0, 2, 4, ...) and the odd rows. When the dart moves, the two fields disagree
and you see comb shaped edges. All modes return an image of the same shape.

    none    unchanged
    bob     keep the even field and double every line (sharp in time, half resolution)
    blend   average each row with the next one (soft, no combing, slight ghosting)
    linear  keep the even field, rebuild odd rows as the average of the rows above and below
"""
import numpy as np

MODES = ("none", "bob", "blend", "linear")


def normalize_mode(mode) -> str:
    """Accept True/False/None as well as a mode name (INTERFACES uses deinterlace=False)."""
    if mode is None or mode is False:
        return "none"
    if mode is True:
        return "linear"
    mode = str(mode).lower()
    if mode not in MODES:
        raise ValueError("deinterlace mode must be one of %s, got %r" % (MODES, mode))
    return mode


def deinterlace(img: np.ndarray, mode="linear") -> np.ndarray:
    mode = normalize_mode(mode)
    h = img.shape[0]
    if mode == "none" or h < 2:
        return img
    if mode == "bob":
        # Repeat each even row twice, then trim to the original height (odd heights).
        return np.repeat(img[0::2], 2, axis=0)[:h]
    if mode == "blend":
        out = img.astype(np.uint16)
        out[:-1] = (out[:-1] + out[1:] + 1) // 2   # last row stays as it is
        return out.astype(np.uint8)
    # linear: odd row r becomes the mean of even rows r-1 and r+1.
    out = img.copy()
    above = img[0:h - 2:2].astype(np.uint16)       # rows 0, 2, ...
    below = img[2:h:2].astype(np.uint16)           # rows 2, 4, ...
    n = min(len(above), len(below))
    out[1:1 + 2 * n:2] = ((above[:n] + below[:n] + 1) // 2).astype(np.uint8)
    if h % 2 == 0:
        out[h - 1] = img[h - 2]                    # last odd row has no row below
    return out
