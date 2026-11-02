"""Procedural terrain textures, one generator per landing class.

Used by the tests, by the terrain classifier (its prototypes are learned from these
textures) and by the flight video simulator. The images are meant to look like what
the dart nose camera (analog Caddx Ant, 720x480 after capture) sees from a few
centimetres to a few tens of centimetres above the ground after landing:

- bedrock: grey or brown rock, large smooth plates, sharp dark cracks, little fine texture.
- hard_ground: compact earth, fine grain, small scattered pebbles, maybe hairline cracks.
- loose_soil: darker brown, clumpy, lots of small shadows (medium to high texture).
- sand_dust: pale, very fine grain, wind ripples, low contrast.
- vegetation: green, many thin high frequency blades over dark soil.
- water_void: dark water with smooth reflections and specular glints, or a black void.

All sizes are in pixels at the dart camera scale, so a texture rendered at 480x720 has
the same grain size as one rendered at 480x480 (the classifier relies on that).

Public API:
    texture(cls, h, w, seed, lighting=True, noise=0.0, analog=False) -> BGR uint8
    CLASSES: tuple of class names
    apply_lighting(img, rng), add_camera_noise(img, sigma, rng, analog=False)
"""
from __future__ import annotations

from typing import Callable, Dict

import cv2
import numpy as np

CLASSES = ("bedrock", "hard_ground", "loose_soil", "sand_dust", "vegetation", "water_void")


# ---------------------------------------------------------------- noise helpers

def _value_noise(h: int, w: int, scale: float, rng: np.random.Generator) -> np.ndarray:
    """Smooth random field in about -1..1 with features of roughly `scale` pixels."""
    scale = max(1.0, float(scale))
    gh = int(np.ceil(h / scale)) + 3
    gw = int(np.ceil(w / scale)) + 3
    grid = rng.random((gh, gw)).astype(np.float32) * 2.0 - 1.0
    if scale <= 1.0:
        return grid[:h, :w]
    big = cv2.resize(grid, (int(gw * scale), int(gh * scale)), interpolation=cv2.INTER_CUBIC)
    oy = int(rng.integers(0, max(1, big.shape[0] - h)))
    ox = int(rng.integers(0, max(1, big.shape[1] - w)))
    return big[oy:oy + h, ox:ox + w]


def _fbm(h: int, w: int, scale: float, rng: np.random.Generator, octaves: int = 4,
         gain: float = 0.5) -> np.ndarray:
    out = np.zeros((h, w), np.float32)
    amp, total = 1.0, 0.0
    s = scale
    for _ in range(octaves):
        out += amp * _value_noise(h, w, s, rng)
        total += amp
        amp *= gain
        s = max(1.0, s / 2.0)
    return out / total


def _colour_img(h: int, w: int, bgr) -> np.ndarray:
    img = np.empty((h, w, 3), np.float32)
    img[:] = np.asarray(bgr, np.float32)
    return img


def _jitter_colour(rng: np.random.Generator, bgr, amount: float = 12.0):
    return tuple(float(c + rng.normal(0, amount)) for c in bgr)


def _random_walk_crack(img: np.ndarray, rng: np.random.Generator, length: int,
                       darkness: float, width: int) -> None:
    h, w = img.shape[:2]
    x, y = float(rng.uniform(0, w)), float(rng.uniform(0, h))
    ang = float(rng.uniform(0, 2 * np.pi))
    pts = [(int(x), int(y))]
    step = 6.0
    for _ in range(max(2, int(length / step))):
        ang += float(rng.normal(0, 0.35))
        x += step * np.cos(ang)
        y += step * np.sin(ang)
        pts.append((int(x), int(y)))
    mask = np.zeros((h, w), np.uint8)
    cv2.polylines(mask, [np.array(pts, np.int32)], False, 255, width, cv2.LINE_AA)
    m = (mask.astype(np.float32) / 255.0)[..., None]
    img *= 1.0 - m * (1.0 - darkness)


# ---------------------------------------------------------------- class generators

def _bedrock(h: int, w: int, rng: np.random.Generator) -> np.ndarray:
    palette = [(118, 120, 124), (96, 108, 124), (84, 98, 118), (128, 128, 132), (100, 104, 110)]
    base = _jitter_colour(rng, palette[int(rng.integers(len(palette)))], 8)
    img = _colour_img(h, w, base)
    # Plates: Voronoi cells, each with its own brightness and a gentle tilt shading.
    n = int(rng.integers(5, 11))
    seeds = np.stack([rng.uniform(-0.1 * w, 1.1 * w, n), rng.uniform(-0.1 * h, 1.1 * h, n)], 1)
    ys, xs = np.mgrid[0:h, 0:w].astype(np.float32)
    d = (xs[None] - seeds[:, 0, None, None]) ** 2 + (ys[None] - seeds[:, 1, None, None]) ** 2
    lab = np.argmin(d, axis=0)
    offs = rng.normal(0, 11, n).astype(np.float32)
    gx = rng.normal(0, 0.02, n).astype(np.float32)
    gy = rng.normal(0, 0.02, n).astype(np.float32)
    shade = offs[lab] + gx[lab] * (xs - w / 2) + gy[lab] * (ys - h / 2)
    shade += 10.0 * _fbm(h, w, 140, rng, 3)
    shade += 2.5 * _value_noise(h, w, 2, rng)          # very little fine texture
    img += shade[..., None]
    # Mineral tint variation, low frequency.
    tint = 6.0 * _value_noise(h, w, 90, rng)
    img[..., 2] += tint
    img[..., 0] -= tint
    # Sharp cracks along some plate boundaries plus a few free cracks.
    edges = np.zeros((h, w), np.uint8)
    lab8 = lab.astype(np.int32)
    edges[:, 1:] |= (lab8[:, 1:] != lab8[:, :-1]).astype(np.uint8)
    edges[1:, :] |= (lab8[1:, :] != lab8[:-1, :]).astype(np.uint8)
    keep = rng.random(n) < 0.75
    edges &= keep[lab].astype(np.uint8)
    edges = cv2.dilate(edges * 255, np.ones((2, 2), np.uint8))
    wobble = cv2.blur((rng.random((h, w)) > 0.5).astype(np.float32), (5, 5))
    m = (edges.astype(np.float32) / 255.0) * (0.6 + 0.4 * wobble)
    img *= (1.0 - 0.7 * m)[..., None]
    for _ in range(int(rng.integers(1, 4))):
        _random_walk_crack(img, rng, int(rng.uniform(0.3, 0.9) * max(h, w)),
                           float(rng.uniform(0.25, 0.45)), int(rng.integers(1, 3)))
    img = cv2.GaussianBlur(img, (0, 0), 0.6)
    return img


def _hard_ground(h: int, w: int, rng: np.random.Generator) -> np.ndarray:
    palette = [(98, 128, 158), (88, 118, 150), (104, 126, 146), (80, 110, 140)]
    base = _jitter_colour(rng, palette[int(rng.integers(len(palette)))], 8)
    img = _colour_img(h, w, base)
    fine = 13.0 * _value_noise(h, w, 1.5, rng) + 8.0 * _value_noise(h, w, 5, rng)
    fine += 8.0 * _fbm(h, w, 70, rng, 3)
    img += fine[..., None]
    # Small pebbles with a highlight and a shadow.
    n = int(h * w / 1100 * rng.uniform(0.7, 1.3))
    for _ in range(n):
        x, y = int(rng.integers(0, w)), int(rng.integers(0, h))
        a, b = int(rng.integers(2, 6)), int(rng.integers(2, 5))
        ang = float(rng.uniform(0, 180))
        g = float(rng.uniform(-45, 35))
        col = tuple(float(c + g + rng.normal(0, 8)) for c in base)
        cv2.ellipse(img, (x + 1, y + 1), (a, b), ang, 0, 360, tuple(c * 0.55 for c in base), -1,
                    cv2.LINE_AA)
        cv2.ellipse(img, (x, y), (a, b), ang, 0, 360, col, -1, cv2.LINE_AA)
        cv2.circle(img, (x - a // 3, y - b // 3), max(1, a // 3),
                   tuple(min(255.0, c + 30) for c in col), -1, cv2.LINE_AA)
    if rng.random() < 0.5:
        for _ in range(int(rng.integers(1, 4))):
            _random_walk_crack(img, rng, int(0.3 * max(h, w)), 0.75, 1)
    return cv2.GaussianBlur(img, (0, 0), 0.5)


def _loose_soil(h: int, w: int, rng: np.random.Generator) -> np.ndarray:
    palette = [(42, 58, 82), (38, 52, 72), (48, 66, 90), (34, 46, 64)]
    base = _jitter_colour(rng, palette[int(rng.integers(len(palette)))], 6)
    img = _colour_img(h, w, base)
    img += (10.0 * _fbm(h, w, 40, rng, 3))[..., None]
    n = int(h * w / 90 * rng.uniform(0.8, 1.2))
    xs = rng.integers(0, w, n)
    ys = rng.integers(0, h, n)
    for i in range(n):
        x, y = int(xs[i]), int(ys[i])
        r = int(rng.integers(3, 12))
        rr = int(max(2, r * rng.uniform(0.6, 1.0)))
        ang = float(rng.uniform(0, 180))
        g = float(rng.normal(0, 16))
        col = tuple(float(c + g) for c in base)
        cv2.ellipse(img, (x + r // 3 + 1, y + r // 3 + 1), (r, rr), ang, 0, 360,
                    tuple(c * 0.4 for c in base), -1, cv2.LINE_AA)
        cv2.ellipse(img, (x, y), (r, rr), ang, 0, 360, col, -1, cv2.LINE_AA)
        cv2.ellipse(img, (x - r // 4, y - r // 4), (max(1, r // 2), max(1, rr // 2)), ang, 0, 360,
                    tuple(c + 12 for c in col), -1, cv2.LINE_AA)
    img += (9.0 * _value_noise(h, w, 1.5, rng))[..., None]
    return cv2.GaussianBlur(img, (0, 0), 0.6)


def _sand_dust(h: int, w: int, rng: np.random.Generator) -> np.ndarray:
    palette = [(150, 182, 208), (160, 186, 204), (168, 172, 178), (140, 170, 196), (175, 190, 205)]
    base = _jitter_colour(rng, palette[int(rng.integers(len(palette)))], 6)
    img = _colour_img(h, w, base)
    ys, xs = np.mgrid[0:h, 0:w].astype(np.float32)
    phi = float(rng.uniform(0, np.pi))
    lam = float(rng.uniform(18, 40))
    wob = 10.0 * _fbm(h, w, 80, rng, 2)
    s = (xs * np.cos(phi) + ys * np.sin(phi) + wob) / lam
    frac = s - np.floor(s)
    ripple = np.where(frac < 0.7, frac / 0.7, (1 - frac) / 0.3) * 2 - 1   # asymmetric ripple
    ripple = cv2.GaussianBlur(ripple.astype(np.float32), (0, 0), 1.5)
    amp = float(rng.uniform(4, 8)) if rng.random() < 0.8 else 1.0
    img += (amp * ripple + 4.0 * _value_noise(h, w, 1.0, rng)
            + 4.0 * _fbm(h, w, 100, rng, 2))[..., None]
    return cv2.GaussianBlur(img, (0, 0), 0.5)


def _vegetation(h: int, w: int, rng: np.random.Generator) -> np.ndarray:
    soil = _jitter_colour(rng, (30, 50, 52), 5)
    img = _colour_img(h, w, soil)
    img += (8.0 * _fbm(h, w, 30, rng, 3))[..., None]
    n = int(h * w / 30 * rng.uniform(0.8, 1.2))
    main_ang = float(rng.uniform(0, np.pi))
    spread = float(rng.uniform(0.4, 1.6))
    dry = float(rng.uniform(0.0, 0.25))
    for _ in range(n):
        x, y = float(rng.uniform(0, w)), float(rng.uniform(0, h))
        L = float(rng.uniform(8, 30))
        a = main_ang + float(rng.normal(0, spread))
        x2, y2 = x + L * np.cos(a), y + L * np.sin(a)
        if rng.random() < dry:
            col = (float(rng.uniform(40, 80)), float(rng.uniform(120, 170)), float(rng.uniform(130, 180)))
        else:
            gv = float(rng.uniform(70, 185))
            col = (float(rng.uniform(15, 55)), gv, float(rng.uniform(0.25, 0.65) * gv))
        cv2.line(img, (int(x), int(y)), (int(x2), int(y2)), col, int(rng.integers(1, 3)), cv2.LINE_AA)
    return cv2.GaussianBlur(img, (0, 0), 0.5)


def _water_void(h: int, w: int, rng: np.random.Generator) -> np.ndarray:
    ys, xs = np.mgrid[0:h, 0:w].astype(np.float32)
    if rng.random() < 0.5:
        # Black void: a hole, crevasse or deep shadow. Nearly no light comes back.
        lvl = float(rng.uniform(6, 22))
        img = _colour_img(h, w, (lvl, lvl, lvl * 1.05))
        img += (3.0 * _fbm(h, w, 120, rng, 2))[..., None]
        if rng.random() < 0.5:
            # A faint lit rim at one side.
            ang = float(rng.uniform(0, 2 * np.pi))
            t = (xs - w / 2) * np.cos(ang) + (ys - h / 2) * np.sin(ang)
            rim = np.clip((t - 0.42 * max(h, w)) / (0.1 * max(h, w)), 0, 1)
            img += (rim * 35.0)[..., None]
        return img
    # Water: dark, smooth, sky reflection gradient, gentle ripples, specular glints.
    base = _jitter_colour(rng, (62, 50, 36), 8)
    img = _colour_img(h, w, base)
    grad = (ys / h - 0.5) * float(rng.uniform(-30, 30))
    phi = float(rng.uniform(0, np.pi))
    rip = np.sin((xs * np.cos(phi) + ys * np.sin(phi)) / float(rng.uniform(10, 25))
                 + 3.0 * _fbm(h, w, 60, rng, 2))
    img += (grad + 6.0 * rip + 8.0 * _fbm(h, w, 150, rng, 2))[..., None]
    glint = np.clip((rip - 0.93) / 0.07, 0, 1) * (_value_noise(h, w, 40, rng) > 0.3)
    for _ in range(int(rng.integers(1, 4))):
        cx, cy = int(rng.integers(0, w)), int(rng.integers(0, h))
        r = int(rng.integers(6, 25))
        spot = np.zeros((h, w), np.float32)
        cv2.circle(spot, (cx, cy), r, 1.0, -1, cv2.LINE_AA)
        glint = np.maximum(glint, cv2.GaussianBlur(spot, (0, 0), r / 3.0))
    img = img * (1 - glint[..., None]) + 245.0 * glint[..., None]
    return img


_GENERATORS: Dict[str, Callable[[int, int, np.random.Generator], np.ndarray]] = {
    "bedrock": _bedrock,
    "hard_ground": _hard_ground,
    "loose_soil": _loose_soil,
    "sand_dust": _sand_dust,
    "vegetation": _vegetation,
    "water_void": _water_void,
}


# ---------------------------------------------------------------- lighting and camera

def apply_lighting(img: np.ndarray, rng: np.random.Generator, strength: float = 1.0) -> np.ndarray:
    """Multiplicative lighting: global gain, linear gradient, vignette, colour cast.

    `img` is float32 BGR 0..255. Returns float32.
    """
    h, w = img.shape[:2]
    ys, xs = np.mgrid[0:h, 0:w].astype(np.float32)
    gain = 1.0 + strength * float(rng.uniform(-0.25, 0.25))
    ang = float(rng.uniform(0, 2 * np.pi))
    t = ((xs - w / 2) * np.cos(ang) + (ys - h / 2) * np.sin(ang)) / max(h, w)
    grad = 1.0 + strength * float(rng.uniform(0, 0.3)) * t
    r2 = ((xs - w / 2) / (w / 2)) ** 2 + ((ys - h / 2) / (h / 2)) ** 2
    vig = 1.0 - strength * float(rng.uniform(0.0, 0.18)) * r2
    cast = 1.0 + strength * rng.normal(0, 0.025, 3).astype(np.float32)
    return img * (gain * grad * vig)[..., None] * cast[None, None, :]


def add_camera_noise(img: np.ndarray, sigma: float, rng: np.random.Generator,
                     analog: bool = False) -> np.ndarray:
    """Gaussian sensor noise (`sigma` in 0..1 of full scale), plus optional analog artefacts.

    analog=True adds chroma noise, per-line horizontal jitter and a little horizontal blur,
    like a composite video capture. `img` is float32 BGR 0..255; returns float32.
    """
    h, w = img.shape[:2]
    out = img.astype(np.float32)
    if analog:
        out = cv2.blur(out, (3, 1))
        shifts = np.round(rng.normal(0, 0.7, h)).astype(int)
        for y in np.nonzero(shifts)[0]:
            out[y] = np.roll(out[y], int(shifts[y]), axis=0)
        chroma = rng.normal(0, 255 * max(sigma, 0.01), (h // 2 + 1, w // 4 + 1, 2)).astype(np.float32)
        chroma = cv2.resize(chroma, (w, h), interpolation=cv2.INTER_LINEAR)
        out[..., 0] += chroma[..., 0]
        out[..., 2] += chroma[..., 1]
    if sigma > 0:
        out += rng.normal(0, 255.0 * sigma, out.shape).astype(np.float32)
    return out


def texture(cls: str, h: int, w: int, seed: int = 0, lighting: bool = True, noise: float = 0.0,
            analog: bool = False) -> np.ndarray:
    """Render one procedural texture of terrain class `cls` as a BGR uint8 image (h x w).

    lighting: apply random gain, gradient, vignette and colour cast (seeded).
    noise: Gaussian camera noise sigma as a fraction of full scale (0.02 to 0.06 is typical
    for the analog dart camera).
    analog: add composite video artefacts (line jitter, chroma noise, horizontal smear).
    """
    if cls not in _GENERATORS:
        raise ValueError("unknown terrain class %r, expected one of %s" % (cls, ", ".join(CLASSES)))
    rng = np.random.default_rng([int(seed) & 0xFFFFFFFF, CLASSES.index(cls)])
    img = _GENERATORS[cls](int(h), int(w), rng)
    if lighting:
        img = apply_lighting(img, rng)
    if noise > 0 or analog:
        img = add_camera_noise(img, float(noise), rng, analog=analog)
    return np.clip(img, 0, 255).astype(np.uint8)


if __name__ == "__main__":  # quick visual check: python -m grappstar.sim.terrain_textures out.png
    import sys
    tiles = [texture(c, 240, 320, seed=1, noise=0.03) for c in CLASSES]
    sheet = np.vstack([np.hstack(tiles[:3]), np.hstack(tiles[3:])])
    cv2.imwrite(sys.argv[1] if len(sys.argv) > 1 else "textures.png", sheet)
