"""Synthetic dart (KEDGE) flight video: a procedural ground map seen from the nose camera.

render_flight(seed=0, fps=30, landing_class="bedrock", size=(720, 480), dropouts=True, noise=0.02, ...)
    -> (frames: list[Frame], truth: dict)

What it simulates
-----------------
- A world ground map (default 30 m x 12 m at 2 cm/px) with textured regions: bedrock shelves with cracks,
  hard ground, loose soil, sand with ripples, grass and a pond. A blob of `landing_class` terrain is painted
  around the landing point, plus a 1.2 m high resolution (2 mm/px) detail patch so the close-up after landing
  shows the landing texture (from sim.terrain_textures when it is available, else a built-in generator).
- The rover rail at world (0, 0, h0) aiming downrange (+X) at 25 to 45 deg pitch, a spring stroke of 60 ms to
  about 10 m/s, a ballistic flight with simple drag (quadratic air drag plus a constant tether drag), a nose
  first impact with shake, then the dart resting nose down in the ground.
- A pinhole camera on the dart axis (HFOV `hfov_deg`, default 100 deg; the real Caddx Ant is about 150 deg
  diagonal with fisheye distortion, a moderate pinhole is used here for simplicity). The ground is rendered with
  one cv2.warpPerspective of the world map (plane Z = 0), the sky is a gradient above the horizon and far ground
  fades into haze.
- Analog link artefacts: Gaussian noise, softness, and (dropouts=True) snow frames, blue or black "no signal"
  frames and torn frames. Frames are returned as raw captures: quality=None and bad=False, unless
  mark_bad=True (then bad=True on the corrupted frames). truth["bad_idx"] lists them either way.
- A C2 flightlog (GET /flightlog) consistent with the video: 50 Hz, quaternion world from body, specific force
  (accelerometer reading, includes gravity) in the body frame in m/s^2, each axis clamped at +-78 m/s^2 like the
  BNO085 (the launch push saturates). t_launch_ms = start of the 20 ms over 4 g, t_land_ms = the last impact.

Frames and conventions
----------------------
World: X downrange (rover to target), Y to the left, Z up, origin on the ground under the rail end.
Body: x forward along the dart nose, y to the left, z up when the dart sits level.
Quaternion (qw, qx, qy, qz) rotates body vectors into the world frame (world from body).
Euler angles (deg): yaw about Z (positive = nose to the left), pitch = nose elevation (positive = nose up),
roll about the nose axis (positive = right side down). R = Rz(yaw) * Ry(-pitch) * Rx(roll).
Camera (OpenCV): x right, y down, z forward along body x.
Accelerometer: f = a - g_vec with g_vec = (0, 0, -9.81) in world, so a dart at rest reads +9.81 on body z.
Times: frame.t is wall time (time.time() style). Flightlog t_ms are dart millis; video_t = t_ms / 1000 +
truth["clock_offset_s"].

truth keys: t_launch, t_stroke_end, t_impact, t_landed, landing_class, landing_xy_world (m), landing_px_map
(col, row in world_map), range_m, launch_pitch_deg, launch_yaw_deg, v0, launch_xyz, flightlog, clock_offset_s,
world_map (BGR), map_res (m/px), map_origin (X at col 0, Y at row 0), camera (fx, fy, cx, cy, hfov_deg, w, h),
poses {frame idx: (t, x, y, z, roll, pitch, yaw)}, bad_idx {frame idx: kind}, phases [(name, t0, t1)].
Use image_to_world(truth, idx, u, v) / world_to_map(truth, x, y) to check vision results against the truth,
and render_view(pose, size) for a single clean view (tests).
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

try:
    from grappstar.video.frame import Frame
except Exception:  # pragma: no cover - the video package always ships frame.py
    from dataclasses import dataclass

    @dataclass
    class Frame:  # type: ignore[no-redef]
        img: np.ndarray
        t: float
        idx: int
        quality: Optional[float] = None
        bad: bool = False

G = 9.81
STROKE_S = 0.06          # spring stroke duration
RAIL_H = 0.5             # camera height at the start of the stroke (m)
DRAG_K = 0.01            # quadratic air drag, a = DRAG_K * v^2 (1/m)
DRAG_C = 1.0             # tether payout drag, constant deceleration (m/s^2)
IMPACT_PULSE_S = 0.05    # nose deceleration half sine
SETTLE_S = 0.25          # impact to still
ACC_SAT = 78.0           # m/s^2, accelerometer saturation per axis (firmware/dart/API.md)
WORLD_CLASSES = ("bedrock", "hard_ground", "loose_soil", "sand_dust", "vegetation", "water_void")
CLASS_ALIASES = {"sand": "sand_dust", "dust": "sand_dust", "grass": "vegetation", "water": "water_void",
                 "pond": "water_void", "soil": "loose_soil", "rock": "bedrock", "hard": "hard_ground"}


# ------------------------------------------------------------------ rotation helpers

def rot_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """World from body rotation from Euler angles in degrees (see module doc for the convention)."""
    r, p, y = math.radians(roll), math.radians(pitch), math.radians(yaw)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, -sp], [0, 1, 0], [sp, 0, cp]])  # Ry(-pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def quat_from_rot(R: np.ndarray) -> np.ndarray:
    """(qw, qx, qy, qz) from a rotation matrix, qw >= 0."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q, float)
    q /= np.linalg.norm(q)
    return -q if q[0] < 0 else q


# camera axes expressed in the body frame: cam x = right = -body y, cam y = down = -body z, cam z = body x
R_BODY_CAM = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def camera_K(size: Tuple[int, int], hfov_deg: float = 100.0) -> np.ndarray:
    w, h = size
    f = (w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return np.array([[f, 0, (w - 1) / 2.0], [0, f, (h - 1) / 2.0], [0, 0, 1.0]])


def _pose_R_cw(roll, pitch, yaw) -> np.ndarray:
    """Camera from world rotation."""
    return (rot_from_rpy(roll, pitch, yaw) @ R_BODY_CAM).T


def ground_homography(pose, K: np.ndarray, A: np.ndarray) -> np.ndarray:
    """Homography image <- plane pixels, where A maps plane pixels (col, row, 1) to world (X, Y, 1) on Z = 0.

    pose = (x, y, z, roll, pitch, yaw)."""
    x, y, z, roll, pitch, yaw = pose
    Rcw = _pose_R_cw(roll, pitch, yaw)
    t = -Rcw @ np.array([x, y, z], float)
    M = np.column_stack([Rcw[:, 0], Rcw[:, 1], t])
    return K @ M @ A


def map_affine(origin: Tuple[float, float], res: float) -> np.ndarray:
    """World (X, Y, 1) from map pixel (col, row, 1). origin = (X at col 0, Y at row 0); rows go to -Y."""
    return np.array([[res, 0, origin[0]], [0, -res, origin[1]], [0, 0, 1.0]])


def world_to_map(truth: dict, x: float, y: float) -> Tuple[float, float]:
    X0, Y0 = truth["map_origin"]
    r = truth["map_res"]
    return ((x - X0) / r, (Y0 - y) / r)


def image_to_world(truth: dict, idx: int, u: float, v: float) -> Optional[Tuple[float, float]]:
    """Ground point (X, Y) seen at full resolution pixel (u, v) of frame idx, or None (sky / unknown frame)."""
    pose = truth["poses"].get(idx)
    if pose is None:
        return None
    cam = truth["camera"]
    K = np.array([[cam["fx"], 0, cam["cx"]], [0, cam["fy"], cam["cy"]], [0, 0, 1.0]])
    H = ground_homography(pose[1:], K, np.eye(3))
    p = np.linalg.solve(H, np.array([u, v, 1.0]))
    if abs(p[2]) < 1e-12:
        return None
    # the ground point must be in front of the camera
    Rcw = _pose_R_cw(*pose[4:7])
    P = np.array([p[0] / p[2], p[1] / p[2], 0.0])
    if (Rcw @ (P - np.array(pose[1:4])))[2] <= 0:
        return None
    return (float(P[0]), float(P[1]))


# ------------------------------------------------------------------ textures

def _noise(h: int, w: int, scale: float, rng: np.random.Generator) -> np.ndarray:
    """Smooth noise about -1..1 with features of `scale` pixels."""
    gh, gw = max(2, int(h / scale) + 2), max(2, int(w / scale) + 2)
    g = rng.random((gh, gw), dtype=np.float32) * 2 - 1
    return cv2.resize(g, (w, h), interpolation=cv2.INTER_CUBIC)


def _fbm(h, w, scale, rng, octaves=4):
    out = np.zeros((h, w), np.float32)
    amp, tot, s = 1.0, 0.0, float(scale)
    for _ in range(octaves):
        out += amp * _noise(h, w, s, rng)
        tot += amp
        amp *= 0.55
        s = max(1.5, s / 2.0)
    return out / tot


def _class_texture(cls: str, h: int, w: int, rng: np.random.Generator, px_m: float) -> np.ndarray:
    """Built-in texture of one class, `px_m` metres per pixel. BGR float32 0..255."""
    k = 0.02 / px_m  # feature scale factor relative to the 2 cm/px world map
    n1 = _fbm(h, w, 40 * k, rng, 3)
    n2 = _fbm(h, w, 6 * k, rng, 3)
    img = np.empty((h, w, 3), np.float32)
    if cls == "bedrock":
        base = np.array([128, 132, 138], np.float32)
        img[:] = base
        img += (n1 * 30 + n2 * 14)[..., None]
        plates = (_noise(h, w, 60 * k, rng) > 0.25).astype(np.float32)
        img += (plates * 18)[..., None]
        layer = np.zeros((h, w), np.uint8)
        ncr = int(h * w / (900 * k * k)) + 3
        for _ in range(ncr):
            x, y = rng.uniform(0, w), rng.uniform(0, h)
            a = rng.uniform(0, 2 * math.pi)
            pts = []
            for _ in range(int(rng.integers(4, 12))):
                pts.append((int(x), int(y)))
                a += rng.normal(0, 0.5)
                st = rng.uniform(4, 12) * k
                x += math.cos(a) * st
                y += math.sin(a) * st
            cv2.polylines(layer, [np.array(pts, np.int32)], False, 255, max(1, int(round(1.2 * k ** 0.5))))
        img -= (layer.astype(np.float32) / 255.0 * 75)[..., None]
    elif cls == "hard_ground":
        img[:] = np.array([95, 120, 145], np.float32)
        img += (n1 * 18 + n2 * 16)[..., None]
        _pebbles(img, rng, int(h * w / (60 * k * k)), 1.5 * k, (60, 75, 90), (165, 180, 195))
    elif cls == "loose_soil":
        img[:] = np.array([55, 78, 105], np.float32)
        img += (n1 * 16 + n2 * 26)[..., None]
        _pebbles(img, rng, int(h * w / (25 * k * k)), 2.0 * k, (25, 35, 50), (110, 135, 160))
    elif cls == "sand_dust":
        img[:] = np.array([150, 190, 215], np.float32)
        yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
        warp = n1 * 40 * k
        rip = np.sin((xx * 0.8 + yy * 0.3 + warp) / (3.0 * k))
        img += (rip * 12 + n2 * 10 + n1 * 14)[..., None]
        _pebbles(img, rng, int(h * w / (400 * k * k)), 1.2 * k, (110, 140, 165), (190, 225, 240))
    elif cls == "vegetation":
        img[:] = np.array([45, 105, 70], np.float32)
        img += (n1 * 22)[..., None]
        layer = img
        nt = int(h * w / (14 * k * k))
        xs = rng.uniform(0, w, nt)
        ys = rng.uniform(0, h, nt)
        ang = rng.uniform(0, math.pi, nt)
        ln = rng.uniform(2, 6, nt) * k
        cols = rng.integers(0, 2, nt)
        pal = [(25, 70, 40), (80, 170, 120)]
        for i in range(nt):
            x2 = int(xs[i] + math.cos(ang[i]) * ln[i])
            y2 = int(ys[i] + math.sin(ang[i]) * ln[i])
            cv2.line(layer, (int(xs[i]), int(ys[i])), (x2, y2), pal[cols[i]], max(1, int(k ** 0.5)))
    else:  # water_void
        img[:] = np.array([70, 55, 30], np.float32)
        img += (n1 * 10)[..., None]
        glints = (_noise(h, w, 8 * k, rng) > 0.8).astype(np.float32)
        img += (glints * 70)[..., None]
    return img


def _pebbles(img, rng, n, r, dark, light):
    n = max(0, min(n, 60000))
    xs = rng.integers(0, img.shape[1], n)
    ys = rng.integers(0, img.shape[0], n)
    rs = np.maximum(1, (rng.random(n) * 2 * r).astype(int))
    which = rng.random(n) < 0.5
    for i in range(n):
        cv2.circle(img, (int(xs[i]), int(ys[i])), int(rs[i]), dark if which[i] else light, -1)


def _external_texture(cls: str, h: int, w: int, seed: int) -> Optional[np.ndarray]:
    """sim.terrain_textures.texture if that module exists and works, else None."""
    try:
        from grappstar.sim.terrain_textures import texture  # optional
        img = texture(cls, h, w, seed)
        if isinstance(img, np.ndarray) and img.shape == (h, w, 3):
            return img.astype(np.float32)
    except Exception:
        pass
    return None


def make_world(rng: np.random.Generator, size_m=(30.0, 12.0), res=0.02, origin=(-5.0, 6.0),
               landing_xy=None, landing_class="bedrock", landing_r=1.5):
    """World map (BGR uint8) and a class index map. Rows go to -Y (row 0 is Y = origin[1])."""
    w, h = int(round(size_m[0] / res)), int(round(size_m[1] / res))
    # class layout at low resolution from thresholded noise fields
    lh, lw = h // 10, w // 10
    fields = np.stack([_fbm(lh, lw, 30, rng, 3) + rng.uniform(-0.15, 0.15) for _ in WORLD_CLASSES])
    fields[WORLD_CLASSES.index("water_void")] -= 0.35   # ponds are rare
    fields[WORLD_CLASSES.index("bedrock")] += 0.05
    lab = np.argmax(fields, axis=0).astype(np.uint8)
    # hard ground around the rover
    cv2.circle(lab, (int((0 - origin[0]) / res / 10), int((origin[1] - 0) / res / 10)), int(1.5 / res / 10),
               WORLD_CLASSES.index("hard_ground"), -1)
    if landing_xy is not None:
        cx, cy = (landing_xy[0] - origin[0]) / res / 10, (origin[1] - landing_xy[1]) / res / 10
        pts = []
        for a in np.linspace(0, 2 * math.pi, 14, endpoint=False):
            rr = landing_r / res / 10 * rng.uniform(0.75, 1.25)
            pts.append((int(cx + rr * math.cos(a)), int(cy + rr * math.sin(a))))
        cv2.fillPoly(lab, [np.array(pts, np.int32)], WORLD_CLASSES.index(landing_class))
    lab_full = cv2.resize(lab, (w, h), interpolation=cv2.INTER_NEAREST)
    # soft blend weights: blurred one hot masks (they sum to 1), jittered per class and sharpened so region
    # borders are irregular but never leave a pixel without a class
    out = np.zeros((h, w, 3), np.float32)
    wsum = np.zeros((h, w), np.float32)
    for ci, cls in enumerate(WORLD_CLASSES):
        m = (lab == ci).astype(np.float32)
        if not m.any():
            continue
        m = cv2.resize(cv2.GaussianBlur(m, (7, 7), 1.5), (w, h), interpolation=cv2.INTER_LINEAR)
        ys, xs = np.where(m > 0.01)
        if len(ys) == 0:
            continue
        y0, y1, x0, x1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
        mm = m[y0:y1, x0:x1] * (1.0 + 0.6 * _noise(y1 - y0, x1 - x0, 20, rng))
        mm = np.maximum(mm, 0) ** 6 + 1e-6
        tex = _class_texture(cls, y1 - y0, x1 - x0, rng, res)
        out[y0:y1, x0:x1] += tex * mm[..., None]
        wsum[y0:y1, x0:x1] += mm
    out /= np.maximum(wsum, 1e-12)[..., None]
    # global lighting variation (cloud shadows)
    out *= (1.0 + 0.12 * _noise(h, w, 300, rng))[..., None]
    return np.clip(out, 0, 255).astype(np.uint8), lab_full


def make_detail_patch(world: np.ndarray, center_px, res_world: float, cls: str, seed: int,
                      rng: np.random.Generator, size_m=1.2, res=0.002):
    """High resolution patch (BGRA, alpha feathered) around the landing point.

    Low frequencies come from the world map (so the patch matches it seen from afar), high frequencies from a
    class texture (sim.terrain_textures if available)."""
    n = int(round(size_m / res))
    half_w = size_m / 2 / res_world
    cx, cy = center_px
    src = np.float32([[cx - half_w, cy - half_w], [cx + half_w, cy - half_w], [cx + half_w, cy + half_w]])
    dst = np.float32([[0, 0], [n, 0], [n, n]])
    up = cv2.warpAffine(world, cv2.getAffineTransform(src, dst), (n, n), flags=cv2.INTER_CUBIC,
                        borderMode=cv2.BORDER_REFLECT).astype(np.float32)
    tex = _external_texture(cls, n, n, seed)
    if tex is None:
        tex = _class_texture(cls, n, n, rng, res)
    # keep the texture's own detail (what the terrain classifier expects at close range) and take only the very
    # low frequencies (colour, big shadows) from the world map
    sigma = 4.0 * res_world / res
    patch = tex - cv2.GaussianBlur(tex, (0, 0), sigma) + cv2.GaussianBlur(up, (0, 0), sigma)
    patch = 0.7 * patch + 0.3 * (tex + (up.mean((0, 1)) - tex.mean((0, 1))) * 0.5)
    patch = np.clip(patch, 0, 255).astype(np.uint8)
    yy, xx = np.mgrid[0:n, 0:n].astype(np.float32)
    d = np.minimum(np.minimum(xx, n - 1 - xx), np.minimum(yy, n - 1 - yy)) / (0.12 * n)
    alpha = (np.clip(d, 0, 1) * 255).astype(np.uint8)
    return np.dstack([patch, alpha]), n


# ------------------------------------------------------------------ trajectory

def _simulate(rng, pitch, yaw, v0, roll0, t_launch, dt=0.001):
    """Kinematics at 1 kHz from the stroke start to impact. Returns dict of arrays."""
    u = np.array([math.cos(math.radians(pitch)) * math.cos(math.radians(yaw)),
                  math.cos(math.radians(pitch)) * math.sin(math.radians(yaw)),
                  math.sin(math.radians(pitch))])
    a_s = v0 / STROKE_S
    p = np.array([0.0, 0.0, RAIL_H])
    v = np.zeros(3)
    ts, ps, vs, fs = [], [], [], []
    t = 0.0
    while True:
        if t < STROKE_S:
            a = a_s * u
            f = a + np.array([0, 0, G])           # rail carries gravity during the stroke
        else:
            sp = np.linalg.norm(v)
            drag = -(DRAG_K * sp * sp + DRAG_C) * v / max(sp, 1e-6)
            a = np.array([0, 0, -G]) + drag
            f = drag
        ts.append(t)
        ps.append(p.copy())
        vs.append(v.copy())
        fs.append(f)
        pn = p + v * dt + 0.5 * a * dt * dt
        v = v + a * dt
        if t > STROKE_S and pn[2] <= 0:
            # interpolate to the ground crossing
            frac = p[2] / max(p[2] - pn[2], 1e-9)
            t_imp = t + frac * dt
            p_imp = p + (pn - p) * frac
            break
        p = pn
        t += dt
        if t > 5:
            raise RuntimeError("trajectory did not land")
    return {"t": np.array(ts) + t_launch, "p": np.array(ps), "v": np.array(vs), "f": np.array(fs),
            "t_impact": t_imp + t_launch, "p_impact": p_imp, "v_impact": v.copy(), "u": u}


class _Timeline:
    """Pose (x, y, z, roll, pitch, yaw) and specific force (world) at any wall time."""

    def __init__(self, rng, pitch, yaw, v0, roll0, t_launch):
        self.pitch0, self.yaw0, self.roll0 = pitch, yaw, roll0
        self.t_launch = t_launch
        self.sim = _simulate(rng, pitch, yaw, v0, roll0, t_launch)
        self.t_stroke_end = t_launch + STROKE_S
        self.t_impact = self.sim["t_impact"]
        self.t_landed = self.t_impact + SETTLE_S
        self.wob_a = rng.uniform(3.0, 6.0)
        self.wob_f = rng.uniform(2.5, 4.0)
        self.wob_ph = rng.uniform(0, 2 * math.pi)
        self.wob_tau = rng.uniform(0.35, 0.6)
        self.roll_rate = rng.uniform(-35, 35)
        # final resting pose: nose in the ground, camera looking at the landing patch
        self.pitch_f = rng.uniform(-66, -54)
        self.h_f = rng.uniform(0.12, 0.17)
        imp = self._flight_pose(self.t_impact)
        self.roll_f = imp[3] + rng.uniform(-15, 15)
        self.yaw_f = imp[5] + rng.uniform(-8, 8)
        self.p_land = self.sim["p_impact"]
        back = self.h_f / math.tan(math.radians(-self.pitch_f))
        self.pos_f = np.array([self.p_land[0] - back * math.cos(math.radians(self.yaw_f)),
                               self.p_land[1] - back * math.sin(math.radians(self.yaw_f)), self.h_f])
        self.shake_dir = rng.normal(0, 1, 3)
        self.shake_dir /= np.linalg.norm(self.shake_dir)

    def _wobble(self, t):
        s = t - self.t_stroke_end
        if s <= 0:
            return 0.0, 0.0
        e = self.wob_a * math.exp(-s / self.wob_tau)
        w = 2 * math.pi * self.wob_f * s + self.wob_ph
        ramp = min(1.0, s / 0.03)
        return e * math.sin(w) * ramp, 0.7 * e * math.cos(w) * ramp

    def _flight_pose(self, t):
        sim = self.sim
        i = int(np.clip(np.searchsorted(sim["t"], t) - 1, 0, len(sim["t"]) - 1))
        frac = 0.0 if i + 1 >= len(sim["t"]) else (t - sim["t"][i]) / (sim["t"][i + 1] - sim["t"][i])
        frac = float(np.clip(frac, 0.0, 1.0)) if i + 1 < len(sim["t"]) else 0.0
        if i + 1 < len(sim["t"]):
            p = sim["p"][i] * (1 - frac) + sim["p"][i + 1] * frac
            v = sim["v"][i] * (1 - frac) + sim["v"][i + 1] * frac
        else:
            dtx = t - sim["t"][i]
            p = sim["p"][i] + sim["v"][i] * dtx
            v = sim["v"][i]
        if t < self.t_stroke_end:
            pitch, yaw = self.pitch0, self.yaw0
        else:
            pitch = math.degrees(math.atan2(v[2], math.hypot(v[0], v[1])))
            yaw = math.degrees(math.atan2(v[1], v[0]))
        dp, dy = self._wobble(t)
        roll = self.roll0 + self.roll_rate * max(0.0, t - self.t_stroke_end)
        return (p[0], p[1], max(p[2], 0.02), roll, pitch + dp, yaw + dy)

    def pose(self, t):
        if t <= self.t_launch:
            return (0.0, 0.0, RAIL_H, self.roll0, self.pitch0, self.yaw0)
        if t < self.t_impact:
            return self._flight_pose(t)
        imp = self._flight_pose(self.t_impact)
        s = t - self.t_impact
        k = min(1.0, s / 0.1)
        k = k * k * (3 - 2 * k)
        pos = np.array(imp[:3]) * (1 - k) + self.pos_f * k
        att = np.array(imp[3:]) * (1 - k) + np.array([self.roll_f, self.pitch_f, self.yaw_f]) * k
        env = math.exp(-s / 0.05) if s < SETTLE_S else 0.0
        sh = env * math.sin(2 * math.pi * 12 * s)
        att = att + np.array([6.0, 5.0, 4.0]) * sh * self.shake_dir
        pos = pos + 0.01 * sh * self.shake_dir
        pos[2] = max(pos[2], 0.03)
        return (pos[0], pos[1], pos[2], att[0], att[1], att[2])

    def specific_force(self, t, R):
        """Accelerometer reading in the world frame (body = R.T @ this)."""
        rest = np.array([0.0, 0.0, G])
        if t < self.t_launch:
            return rest
        if t < self.t_impact:
            sim = self.sim
            i = int(np.clip(np.searchsorted(sim["t"], t) - 1, 0, len(sim["t"]) - 1))
            return sim["f"][i]
        s = t - self.t_impact
        f = rest.copy()
        vimp = self.sim["v_impact"]
        if s < IMPACT_PULSE_S:
            mag = math.pi / (2 * IMPACT_PULSE_S) * math.sin(math.pi * s / IMPACT_PULSE_S)
            f = f - vimp * mag            # deceleration opposite to the impact velocity
        if s < SETTLE_S:
            f = f + 30.0 * math.exp(-s / 0.06) * math.sin(2 * math.pi * 17 * s) * (R @ self.shake_dir)
        return f


# ------------------------------------------------------------------ renderer

class _Renderer:
    def __init__(self, world, A_world, detail, A_detail, size, hfov_deg, rng):
        self.world, self.A_world = world, A_world
        self.detail, self.A_detail = detail, A_detail
        self.w, self.h = size
        self.K = camera_K(size, hfov_deg)
        Ki = np.linalg.inv(self.K)
        uu, vv = np.meshgrid(np.arange(self.w, dtype=np.float32), np.arange(self.h, dtype=np.float32))
        self.dx = (Ki[0, 0] * uu + Ki[0, 2]).astype(np.float32)
        self.dy = (Ki[1, 1] * vv + Ki[1, 2]).astype(np.float32)
        self.inv_n = (1.0 / np.sqrt(self.dx ** 2 + self.dy ** 2 + 1.0)).astype(np.float32)
        # sky colour LUT over sin(elevation) 0..1
        e = np.linspace(0, 1, 256)[:, None]
        hor = np.array([235, 222, 205], float) + rng.uniform(-10, 10, 3)
        zen = np.array([200, 135, 70], float) + rng.uniform(-15, 15, 3)
        self.sky_lut = np.clip(hor + (zen - hor) * np.sqrt(e), 0, 255).astype(np.uint8)
        self.haze_d = 25.0

    def render(self, pose):
        x, y, z, roll, pitch, yaw = pose
        Rwc = (_pose_R_cw(roll, pitch, yaw)).T
        rz = Rwc[2]  # world z component of camera rays
        dz = (rz[0] * self.dx + rz[1] * self.dy + rz[2]) * self.inv_n   # sin(elevation)
        sky = self.sky_lut[np.clip(dz * 255, 0, 255).astype(np.uint8)]
        ground_w = np.clip(-dz * 400 + 0.5, 0, 1).astype(np.float32)  # about one pixel soft edge
        if ground_w.max() <= 0:
            return sky
        H = ground_homography(pose, self.K, self.A_world)
        ground = cv2.warpPerspective(self.world, H, (self.w, self.h), flags=cv2.INTER_LINEAR,
                                     borderMode=cv2.BORDER_REFLECT)
        if self.detail is not None and z < 4.0:
            # every ground pixel maps to its true plane point, so corners behind the camera do not matter
            # (wrong values only land on sky pixels, which are masked below)
            dn = self.detail.shape[0]
            cxy = self.A_detail @ np.array([dn / 2.0, dn / 2.0, 1.0])
            if math.hypot(cxy[0] - x, cxy[1] - y) < 8.0:
                Hd = ground_homography(pose, self.K, self.A_detail)
                d = cv2.warpPerspective(self.detail, Hd, (self.w, self.h), flags=cv2.INTER_LINEAR,
                                        borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0, 0))
                a = d[..., 3].astype(np.float32) / 255.0
                if a.max() > 0:
                    ground = cv2.blendLinear(d[..., :3], ground, a, 1.0 - a)
        # haze with distance: dist = z / sin(depression)
        dist = z / np.maximum(-dz, 1e-3)
        haze = np.minimum(0.55, 1.0 - np.exp(-dist / self.haze_d)).astype(np.float32)
        wg = ground_w * (1.0 - haze)
        return cv2.blendLinear(ground, sky, wg, 1.0 - wg)


# ------------------------------------------------------------------ link artefacts

def _snow(img, rng, strength):
    n = rng.normal(0, 90 * strength, img.shape[:2]).astype(np.float32)
    out = img.astype(np.float32) * (1 - 0.6 * strength) + 128 * 0.6 * strength + n[..., None]
    streak = rng.random(img.shape[:2]) < 0.02 * strength
    out[streak] = 255
    return np.clip(out, 0, 255).astype(np.uint8)


def _dropout(img, rng):
    out = np.zeros_like(img)
    if rng.random() < 0.6:
        out[:] = (200, 30, 20)   # receiver blue screen (BGR)
    else:
        out[:] = (8, 8, 8)
    return out


def _tear(img, rng):
    out = img.copy()
    h, w = img.shape[:2]
    r0 = int(rng.integers(h // 5, h * 3 // 4))
    nb = int(rng.integers(2, 4))
    edges = sorted(set([r0] + [int(rng.integers(r0, h)) for _ in range(nb)] + [h]))
    for a, b in zip(edges[:-1], edges[1:]):
        sh = int(rng.integers(w // 12, w // 4)) * (1 if rng.random() < 0.5 else -1)
        out[a:b] = np.roll(img[a:b], sh, axis=1)
        if sh > 0:
            out[a:b, :sh] = 10
        else:
            out[a:b, sh:] = 10
    return out


# ------------------------------------------------------------------ main entry

def _normalize_class(cls: str) -> str:
    c = CLASS_ALIASES.get(cls, cls)
    if c not in WORLD_CLASSES:
        raise ValueError("unknown landing class %r" % cls)
    return c


def render_flight(seed: int = 0, fps: float = 30, landing_class: str = "bedrock", size=(720, 480),
                  dropouts: bool = True, noise: float = 0.02, hfov_deg: float = 100.0,
                  pitch_deg: Optional[float] = None, v0: Optional[float] = None, rail_s: Optional[float] = None,
                  landed_s: float = 2.5, mark_bad: bool = False, buried: bool = True, t0: Optional[float] = None,
                  map_res: float = 0.02):
    """Render a full synthetic dart flight. See the module docstring for conventions and truth keys."""
    rng = np.random.default_rng([int(seed) & 0xFFFFFFFF, 7071])
    landing_class = _normalize_class(landing_class)
    w, h = int(size[0]), int(size[1])
    t0 = float(1_790_000_000.0 + 1000.0 * (int(seed) % 100000)) if t0 is None else float(t0)
    pitch = float(rng.uniform(25, 45)) if pitch_deg is None else float(pitch_deg)
    yaw = float(rng.uniform(-6, 6))
    speed = float(rng.uniform(9.0, 11.0)) if v0 is None else float(v0)
    roll0 = float(rng.uniform(-4, 4))
    rail = float(rng.uniform(0.9, 1.2)) if rail_s is None else float(rail_s)
    t_launch = t0 + rail + rng.uniform(0, 1.0 / fps)
    tl = _Timeline(rng, pitch, yaw, speed, roll0, t_launch)
    land_xy = (float(tl.p_land[0]), float(tl.p_land[1]))

    origin = (-5.0, 6.0)
    world, _lab = make_world(rng, res=map_res, origin=origin, landing_xy=land_xy, landing_class=landing_class)
    A_world = map_affine(origin, map_res)
    land_px = ((land_xy[0] - origin[0]) / map_res, (origin[1] - land_xy[1]) / map_res)
    detail, dn = make_detail_patch(world, land_px, map_res, landing_class, int(seed), rng)
    det_res = 1.2 / dn
    A_detail = map_affine((land_xy[0] - 0.6, land_xy[1] + 0.6), det_res)
    rend = _Renderer(world, A_world, detail, A_detail, (w, h), hfov_deg, rng)

    t_end = tl.t_landed + landed_s
    n_frames = int(math.floor((t_end - t0) * fps)) + 1
    times = t0 + np.arange(n_frames) / fps

    # corrupted frames
    bad: Dict[int, str] = {}
    if dropouts:
        def pick(k, lo, hi):
            lo_i = max(0, int((lo - t0) * fps))
            hi_i = min(n_frames - 1, int((hi - t0) * fps))
            if hi_i <= lo_i:
                return []
            return list(rng.choice(np.arange(lo_i, hi_i + 1), size=min(k, hi_i - lo_i + 1), replace=False))
        for i in pick(2, t0, t_launch - 0.1):
            bad[int(i)] = "snow"
        for i in pick(2, tl.t_stroke_end + 0.1, tl.t_impact - 0.1):
            bad[int(i)] = ["blue_or_black", "torn"][int(rng.integers(0, 2))]
        for i in pick(1, tl.t_stroke_end + 0.1, tl.t_impact - 0.1):
            bad[int(i)] = "snow"
        k_imp = int(math.ceil((tl.t_impact - t0) * fps))   # link fades as the dart reaches the ground
        for i in range(k_imp, min(n_frames, k_imp + int(rng.integers(1, 3)))):
            bad[i] = "snow"
        for i in pick(3, tl.t_landed + 0.2, t_end):
            bad[int(i)] = ["blue_or_black", "torn", "snow"][int(rng.integers(0, 3))]

    soil = None
    if buried:  # a bit of dirt over the bottom edge once the nose is in the ground
        soil = np.zeros((h, w), np.float32)
        edge = h - (0.07 * h + 0.03 * h * _noise(1, w, w / 8, rng)[0])
        rows = np.arange(h, dtype=np.float32)[:, None]
        soil = np.clip((rows - edge[None, :]) / 3.0, 0, 1).astype(np.float32)

    noise_bank = [rng.normal(0, 255 * noise, (h, w, 3)).astype(np.int16) for _ in range(6)] if noise > 0 else None
    frames: List[Frame] = []
    poses: Dict[int, tuple] = {}
    cache: Dict[str, np.ndarray] = {}
    for i, t in enumerate(times):
        t = float(t)
        pose = tl.pose(t)
        poses[i] = (t,) + tuple(float(v) for v in pose)
        if t <= t_launch:
            key = "rail"
        elif t >= tl.t_landed:
            key = "landed"
        else:
            key = None
        if key is not None and key in cache:
            img = cache[key]
        else:
            img = rend.render(pose)
            if t_launch < t < tl.t_launch + 0.2:   # launch motion blur along the image vertical
                kb = max(3, int(h * 0.02) | 1)
                img = cv2.blur(img, (1, kb))
            img = cv2.GaussianBlur(img, (3, 3), 0.8)
            if soil is not None and t >= tl.t_impact + 0.05:
                dirt = np.full_like(img, (30, 45, 60))
                img = cv2.blendLinear(dirt, img, soil, 1.0 - soil)
            if key is not None:
                cache[key] = img
        if noise_bank is not None:
            nb = noise_bank[int(rng.integers(0, len(noise_bank)))]
            sy, sx = int(rng.integers(0, h)), int(rng.integers(0, w))
            nb = np.roll(nb, (sy, sx), axis=(0, 1))
            img = np.clip(img.astype(np.int16) + nb, 0, 255).astype(np.uint8)
        else:
            img = img.copy()
        kind = bad.get(i)
        if kind == "snow":
            img = _snow(img, rng, rng.uniform(0.7, 1.0))
        elif kind == "blue_or_black":
            img = _dropout(img, rng)
        elif kind == "torn":
            img = _tear(img, rng)
        frames.append(Frame(img=img, t=t, idx=i, quality=None, bad=bool(mark_bad and kind is not None)))

    flightlog, offset = _make_flightlog(tl, rng, t0, t_end)
    truth = {
        "t_launch": t_launch, "t_stroke_end": tl.t_stroke_end, "t_impact": tl.t_impact, "t_landed": tl.t_landed,
        "landing_class": landing_class, "landing_xy_world": land_xy, "landing_px_map": land_px,
        "range_m": float(math.hypot(land_xy[0], land_xy[1])), "launch_pitch_deg": pitch, "launch_yaw_deg": yaw,
        "v0": speed, "launch_xyz": (0.0, 0.0, RAIL_H), "flightlog": flightlog, "clock_offset_s": offset,
        "world_map": world, "map_res": map_res, "map_origin": origin,
        "camera": {"fx": float(rend.K[0, 0]), "fy": float(rend.K[1, 1]), "cx": float(rend.K[0, 2]),
                   "cy": float(rend.K[1, 2]), "hfov_deg": hfov_deg, "w": w, "h": h},
        "poses": poses, "bad_idx": bad,
        "phases": [("ON_RAIL", t0, t_launch), ("LAUNCH", t_launch, tl.t_stroke_end),
                   ("FLIGHT", tl.t_stroke_end, tl.t_impact), ("IMPACT", tl.t_impact, tl.t_landed),
                   ("LANDED", tl.t_landed, t_end)],
        "fps": fps,
    }
    return frames, truth


def _make_flightlog(tl: _Timeline, rng, t0: float, t_end: float):
    """C2 flightlog: 2 s before launch to 3 s after landing, 50 Hz, dart millis."""
    t_boot = t0 - float(rng.uniform(20.0, 300.0))
    offset = t_boot  # video_t = t_ms / 1000 + offset
    phase_ms = int(rng.integers(0, 20))
    start_ms = int((tl.t_launch - 2.0 - t_boot) * 1000) // 20 * 20 + phase_ms
    end_ms = int((tl.t_landed + 3.0 - t_boot) * 1000)
    samples = []
    q_prev = None
    for t_ms in range(start_ms, end_ms + 1, 20):
        t = t_ms / 1000.0 + t_boot
        x, y, z, roll, pitch, yaw = tl.pose(t)
        R = rot_from_rpy(roll, pitch, yaw)
        q = quat_from_rot(R)
        if q_prev is not None and float(np.dot(q, q_prev)) < 0:
            q = -q
        q_prev = q
        f_b = R.T @ tl.specific_force(t, R) + rng.normal(0, 0.05, 3)
        f_b = np.clip(f_b, -ACC_SAT, ACC_SAT)       # BNO085 accelerometer saturates near 78 m/s^2 per axis
        samples.append([int(t_ms)] + [round(float(v), 5) for v in q] + [round(float(v), 3) for v in f_b])
    # firmware (API.md): launch time = the START of the 20 ms over 4 g; landing time = the last impact (the dart
    # hits the ground once here, so the last impact is the impact)
    t_launch_ms = int(round((tl.t_launch - t_boot) * 1000))
    t_land_ms = int(round((tl.t_impact - t_boot) * 1000))
    return {"hz": 50, "t_launch_ms": t_launch_ms, "t_land_ms": t_land_ms, "samples": samples}, offset


def render_view(pose, size=(720, 480), seed: int = 0, hfov_deg: float = 100.0, noise: float = 0.0) -> np.ndarray:
    """One clean camera view of a seeded world from pose = (x, y, z, roll, pitch, yaw) (m, deg). For tests."""
    rng = np.random.default_rng([int(seed) & 0xFFFFFFFF, 7072])
    origin, res = (-5.0, 6.0), 0.02
    world, _ = make_world(rng, res=res, origin=origin)
    rend = _Renderer(world, map_affine(origin, res), None, None, (int(size[0]), int(size[1])), hfov_deg, rng)
    img = cv2.GaussianBlur(rend.render(tuple(float(v) for v in pose)), (3, 3), 0.8)
    if noise > 0:
        img = np.clip(img.astype(np.float32) + rng.normal(0, 255 * noise, img.shape), 0, 255).astype(np.uint8)
    return img
