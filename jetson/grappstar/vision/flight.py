"""Dart flight analysis from the nose camera video, fused with the dart IMU flightlog when there is one.

analyze_flight(frames, flightlog=None, clock_offset_s=None, ...) -> FlightResult
landing_frames(frames, result, n=15) -> list[Frame]

Phases: ON_RAIL, LAUNCH, FLIGHT, IMPACT, LANDED, each as (name, t0, t1) in video (wall) time.

How it works
------------
1. Frame quality. Frames with bad=True or quality < 0.4 are skipped. Frames with quality=None get a quick own
   check (uniform blue or black "no signal" frame, snow speckle, torn rows), so raw captures work too.
2. Video motion energy. Each good frame is shrunk to 160 px wide grey and blurred; the energy of a pair of
   consecutive good frames is the mean absolute difference divided by the frame gap. The static floor (rail and
   landed parts) is a low percentile; "moving" is energy above a threshold derived from it. Launch = first
   sustained motion, landed = after the last motion, impact = the end of the large flight motion (the last
   pair above half the peak energy of the final second; after it only a fast decaying shake follows).
3. IMU (C2 flightlog: t_ms dart millis, quaternion world from body, specific force in body frame m/s^2).
   Launch = the run over 4 g that matches the flightlog's t_launch_ms (else the first run of 20 ms), launch time
   = the start of the window (firmware convention). Impact = the last run over 3.5 g once the push has ended (under
   2.5 g) and 150 ms after launch (the one followed by stillness), landed = first time after impact the reading
   stays at its final resting vector (1 g) for 0.3 s. A saturated push (about 75 m/s^2 or more on an axis) gives
   only a lower bound on v0, which is then solved from the flight time.
   Clock alignment: video_t = t_ms / 1000 + offset. The offset is either given (clock_offset_s) or estimated by matching the IMU launch spike to the
   video launch onset (and the IMU impact to the video impact when both exist; the median is used). With a
   flightlog, phase times come from the IMU (precise) shifted by the offset.
4. Attitude: from quaternions (roll, pitch, yaw deg, convention below) when there is a flightlog, else roll and
   pitch from the horizon (yaw unknown, None).
5. Horizon per frame: sky = bright, blue (or very bright) and smooth pixels; largest sky region; the boundary
   between sky and ground is fitted with a robust line. angle_deg is the horizon rotation (counter clockwise
   positive, which equals the camera roll for a pinhole camera); pitch = atan(d / f), d = signed distance of the
   image centre from the horizon (positive when the centre is in the sky). sky_ratio = sky fraction.
6. Trajectory: launch pitch (IMU rail attitude, else horizon on the rail, else solved from the flight time),
   muzzle speed (IMU launch impulse, else solved from pitch and flight time, else 10 m/s), integrated with a
   ballistic model with simple drag. x downrange along the launch heading, y left, z up, origin on the ground
   under the camera at the start of the spring stroke (camera height h0 = 0.5 m there), so range_m is the
   horizontal distance from the dart's rest position on the rail to the landing point.

Conventions (shared with sim.flight_video): body x forward along the dart nose, y left, z up when the dart sits
level. Quaternion = world from body. World: X downrange, Y left, Z up. yaw about Z (positive = nose left),
pitch = nose elevation (positive up), roll about the nose (positive = right side down);
R = Rz(yaw) Ry(-pitch) Rx(roll).

Camera model: pinhole with horizontal FOV hfov_deg (default 100 deg, the simulator's value). For the real
Caddx Ant (about 150 deg diagonal, fisheye) pass its effective FOV; horizon angles stay valid, pitch from the
horizon is approximate.
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from ..video.frame import Frame, ensure_bgr

G = 9.81
BAD_BELOW = 0.4
SMALL_W = 160
DEFAULT_V0 = 7.0         # m/s, fallback only (no C3 table): physics v1.2 muzzle speed is about 7.0 to 7.4
DEFAULT_PITCH = 35.0
RAIL_H = 0.5
STROKE_S = 0.06
DRAG_K = 0.01
DRAG_C = 1.0
IMPACT_G = 3.5           # g: firmware landing impact threshold (was 6 g)
SAT_NEAR = 75.0          # m/s^2: the BNO085 accelerometer saturates near 78 m/s^2 (firmware/dart/API.md)
PHASES = ("ON_RAIL", "LAUNCH", "FLIGHT", "IMPACT", "LANDED")


# ------------------------------------------------------------------ result

def _clean(v):
    if isinstance(v, float):
        return None if not math.isfinite(v) else round(v, 4)
    if isinstance(v, (np.floating,)):
        return _clean(float(v))
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    if isinstance(v, dict):
        return {str(k): _clean(x) for k, x in v.items()}
    return v


@dataclass
class FlightResult:
    phases: List[Tuple[str, float, float]] = field(default_factory=list)
    t_launch: Optional[float] = None
    t_impact: Optional[float] = None
    t_landed: Optional[float] = None
    attitude: List[Tuple[float, Optional[float], Optional[float], Optional[float]]] = field(default_factory=list)
    horizon: List[Tuple[float, Optional[float], Optional[float]]] = field(default_factory=list)
    trajectory: List[Tuple[float, float, float, float]] = field(default_factory=list)
    range_m: Optional[float] = None
    launch_pitch_deg: Optional[float] = None
    launch_yaw_deg: Optional[float] = None
    v0: Optional[float] = None
    flight_time_s: Optional[float] = None
    clock_offset_s: Optional[float] = None
    source: str = "none"                  # "video", "imu", "video+imu", "none"
    motion: List[Tuple[float, float]] = field(default_factory=list)   # (t, energy) per good frame pair
    good: List[int] = field(default_factory=list)                     # idx of frames judged usable
    notes: List[str] = field(default_factory=list)
    video_times: Dict[str, Optional[float]] = field(default_factory=dict)
    imu_times: Dict[str, Optional[float]] = field(default_factory=dict)
    range_scale: float = 1.0              # model-to-C3-table range calibration applied (1 = none)
    launch_fit_t: Optional[float] = None  # launch time the speed fit found (video launch seen late); phases keep t_launch

    def phase_at(self, t: float) -> Optional[str]:
        for name, t0, t1 in self.phases:
            if t0 <= t < t1:
                return name
        if self.phases and t >= self.phases[-1][2]:
            return self.phases[-1][0]
        return None

    def to_dict(self) -> dict:
        return _clean({
            "phases": [list(p) for p in self.phases],
            "t_launch": self.t_launch, "t_impact": self.t_impact, "t_landed": self.t_landed,
            "attitude": [list(a) for a in self.attitude],
            "horizon": [list(h) for h in self.horizon],
            "trajectory": [list(p) for p in self.trajectory],
            "range_m": self.range_m, "launch_pitch_deg": self.launch_pitch_deg,
            "launch_yaw_deg": self.launch_yaw_deg, "v0": self.v0, "flight_time_s": self.flight_time_s,
            "clock_offset_s": self.clock_offset_s, "source": self.source,
            "n_good": len(self.good), "notes": list(self.notes),
            "video_times": dict(self.video_times), "imu_times": dict(self.imu_times),
            "range_scale": self.range_scale, "launch_fit_t": self.launch_fit_t,
        })


# ------------------------------------------------------------------ attitude maths

def quat_to_rpy(qw: float, qx: float, qy: float, qz: float) -> Tuple[float, float, float]:
    """(roll, pitch, yaw) in degrees from a world-from-body quaternion (see module doc)."""
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1.0
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    r20 = 2 * (qx * qz - qw * qy)
    r21 = 2 * (qy * qz + qw * qx)
    r22 = 1 - 2 * (qx * qx + qy * qy)
    r10 = 2 * (qx * qy + qw * qz)
    r00 = 1 - 2 * (qy * qy + qz * qz)
    # R = Rz(yaw) Ry(-pitch) Rx(roll): R20 = sin(pitch), R21 = cos p sin r, R22 = cos p cos r
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, r20))))
    roll = math.degrees(math.atan2(r21, r22))
    yaw = math.degrees(math.atan2(r10, r00))
    return roll, pitch, yaw


def quat_rotate_to_world(q, v):
    """Rotate body vector v into the world frame with quaternion q = (qw, qx, qy, qz)."""
    qw, qx, qy, qz = q
    R = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)]])
    return R @ np.asarray(v, float)


# ------------------------------------------------------------------ frame quality

def _small(img: np.ndarray, width: int = SMALL_W) -> np.ndarray:
    h, w = img.shape[:2]
    if w <= width:
        return img
    return cv2.resize(img, (width, max(1, int(round(h * width / w)))), interpolation=cv2.INTER_AREA)


def quick_quality(img: np.ndarray) -> float:
    """0..1 quality of a raw analog frame (1 = clean). Cheap own check used when frame.quality is None.

    Catches the receiver's blue / black screen (uniform frame), snow (speckle that a 3x3 median removes) and
    torn frames (bands of rows shifted sideways)."""
    if img is None or img.ndim != 3 or img.size == 0:
        return 0.0
    step = max(1, img.shape[1] // SMALL_W)
    sub = np.ascontiguousarray(img[::step, ::step])
    g = cv2.cvtColor(sub, cv2.COLOR_BGR2GRAY)
    gi = g.astype(np.int16)
    std = float(gi.std())
    mean_bgr = sub.reshape(-1, 3).mean(0)
    chan_std = sub.reshape(-1, 3).std(0).max()
    score = 1.0
    if chan_std < 7.0:                         # uniform: no signal screen (a real sky still has a gradient)
        score = 0.0
    elif float(mean_bgr.max()) < 18:           # black
        score = 0.0
    med = cv2.medianBlur(g, 3).astype(np.int16)
    noise_frac = float((np.abs(gi - med) > 30).mean())
    if noise_frac > 0.08:
        score *= max(0.0, 1.0 - (noise_frac - 0.08) / 0.07)
    rows = gi.astype(np.float32)
    pix = np.abs(rows[1:] - rows[:-1]).mean(axis=1)
    mean_d = np.abs(rows[1:].mean(axis=1) - rows[:-1].mean(axis=1))
    tear = pix - mean_d
    base = float(np.median(tear)) + 1.0
    tear_rows = int((tear > 2.5 * base + 8.0).sum())
    if tear_rows >= 2:
        score *= 0.2
    # flat black bands on the side of a torn band
    side = np.minimum(sub[:, :3].reshape(sub.shape[0], -1).max(1), sub[:, -3:].reshape(sub.shape[0], -1).max(1))
    if std > 8 and float((side < 16).mean()) > 0.08:
        score *= 0.3
    return float(max(0.0, min(1.0, score)))


def frame_ok(frame) -> bool:
    if frame is None or getattr(frame, "img", None) is None:
        return False
    if getattr(frame, "bad", False):
        return False
    q = getattr(frame, "quality", None)
    if q is None:
        q = quick_quality(frame.img)
    return q >= BAD_BELOW


# ------------------------------------------------------------------ horizon

def sky_mask(img: np.ndarray) -> np.ndarray:
    """uint8 mask (255 = sky) of a (small) BGR image: bright, blue or very bright, smooth pixels.

    Only the largest sky region touching the image border is kept."""
    b, g, r = [c.astype(np.int16) for c in cv2.split(img)]
    v = np.maximum(np.maximum(b, g), r)
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    tex = cv2.blur(np.abs(cv2.Laplacian(cv2.GaussianBlur(grey, (3, 3), 0), cv2.CV_16S)).astype(np.float32),
                   (5, 5))
    m = (tex < 9.0) & (((v > 140) & (b >= r + 12)) | (v > 215))
    m = m.astype(np.uint8) * 255
    k = np.ones((3, 3), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=4)
    if n <= 1:
        return np.zeros_like(m)
    h, w = m.shape
    best, best_area = 0, 0
    for i in range(1, n):
        x, y, ww, hh, area = stats[i]
        touches = x == 0 or y == 0 or x + ww == w or y + hh == h
        if touches and area > best_area and area > 0.01 * h * w:
            best, best_area = i, area
    if best == 0:
        return np.zeros_like(m)
    out = (lab == best).astype(np.uint8) * 255
    # fill holes (clouds, noise) inside the sky
    inv = cv2.bitwise_not(out)
    n2, lab2, stats2, _ = cv2.connectedComponentsWithStats(inv, connectivity=4)
    for i in range(1, n2):
        x, y, ww, hh, area = stats2[i]
        if not (x == 0 or y == 0 or x + ww == w or y + hh == h) and area < 0.05 * h * w:
            out[lab2 == i] = 255
    return out


def detect_horizon(img: np.ndarray, hfov_deg: float = 100.0) -> dict:
    """Horizon of one BGR frame: dict(angle_deg, pitch_deg, sky_ratio, line=(x0, y0, dx, dy) in full res px).

    angle_deg / pitch_deg are None when no horizon line is visible (all sky, all ground or a messy boundary)."""
    H0, W0 = img.shape[:2]
    small = _small(img)
    s = W0 / small.shape[1]
    m = sky_mask(small)
    h, w = m.shape
    sky_ratio = float((m > 0).mean())
    out = {"angle_deg": None, "pitch_deg": None, "sky_ratio": sky_ratio, "line": None}
    if sky_ratio < 0.01 or sky_ratio > 0.98:
        return out
    edge = cv2.morphologyEx(m, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))
    edge[:2, :] = 0
    edge[-2:, :] = 0
    edge[:, :2] = 0
    edge[:, -2:] = 0
    ys, xs = np.nonzero(edge)
    if len(xs) < 12:
        return out
    pts = np.column_stack([xs, ys]).astype(np.float32)
    dx, dy, x0, y0 = [float(v) for v in cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()]
    resid = np.abs((pts[:, 0] - x0) * dy - (pts[:, 1] - y0) * dx)
    if float((resid < 3.0).mean()) < 0.6:
        return out
    # orient the line so the sky is on its left (y up display coordinates)
    sy, sx = np.nonzero(m)
    cxs, cys = float(sx.mean()), float(sy.mean())
    # left normal of (dx, -dy) in y-up coords is (dy, dx) -> in image coords (dy, -dx)
    if (cxs - x0) * dy + (cys - y0) * (-dx) < 0:
        dx, dy = -dx, -dy
    angle = math.degrees(math.atan2(-dy, dx))
    # signed distance of the image centre from the line, positive on the sky side
    cxi, cyi = (w - 1) / 2.0, (h - 1) / 2.0
    d = ((cxi - x0) * dy + (cyi - y0) * (-dx)) * s
    f = (W0 / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    out.update(angle_deg=angle, pitch_deg=math.degrees(math.atan2(d, f)),
               line=(x0 * s, y0 * s, dx, dy))
    return out


# ------------------------------------------------------------------ ballistics

def ballistic(v0: float, pitch_deg: float, h0: float = RAIL_H, dt: float = 0.005,
              drag_k: float = DRAG_K, drag_c: float = DRAG_C):
    """Flight from the rail end with simple drag until z = 0. Returns (t, x, z) arrays."""
    p = math.radians(pitch_deg)
    vx, vz = v0 * math.cos(p), v0 * math.sin(p)
    x, z, t = 0.0, h0, 0.0
    ts, xs, zs = [0.0], [x], [z]
    while t < 10.0:
        sp = math.hypot(vx, vz) or 1e-6
        dec = drag_k * sp * sp + drag_c
        ax, az = -dec * vx / sp, -G - dec * vz / sp
        xn, zn = x + vx * dt + 0.5 * ax * dt * dt, z + vz * dt + 0.5 * az * dt * dt
        vx, vz = vx + ax * dt, vz + az * dt
        if zn <= 0 and t > 0:
            frac = z / max(z - zn, 1e-9)
            ts.append(t + frac * dt)
            xs.append(x + (xn - x) * frac)
            zs.append(0.0)
            break
        x, z, t = xn, zn, t + dt
        ts.append(t)
        xs.append(x)
        zs.append(z)
    return np.array(ts), np.array(xs), np.array(zs)


def _bisect(fn, lo, hi, target, n=40):
    flo = fn(lo) - target
    fhi = fn(hi) - target
    if flo * fhi > 0:
        return None
    for _ in range(n):
        mid = 0.5 * (lo + hi)
        fm = fn(mid) - target
        if fm * flo <= 0:
            hi = mid
        else:
            lo, flo = mid, fm
    return 0.5 * (lo + hi)


# ------------------------------------------------------------------ speed fusion (video-only and saturated IMU)

# Video impact timing error model, from the flight simulator at the real launcher speeds (6..8 m/s, 6..47 deg):
# the motion-energy impact is late by 0 to about 0.11 s (the crash and settle motion follows the impact), rarely
# early by up to 0.03 s. Flight time residual r = T_model - T_video is therefore expected in [-0.11, +0.03].
VID_T_LATE = 0.11
VID_T_EARLY = 0.03
VID_IMPACT_LATE_TYP = 0.035   # typical lateness of the video impact (for the clock offset without a video launch)
TRACK_SIGMA_DEG = 3.0    # horizon pitch noise during flight (fin wobble 3 to 6 deg decaying, fisheye)
TRACK_CORR = 2.0         # consecutive horizon samples are correlated (wobble): count them as 1 in 2
PRIOR_SIGMA = 0.20       # table muzzle speed prior: 20 percent (spring, clutch and model spread, ASSUMPTION)


def _gamma_track(v0: float, pitch: float, h0: float):
    """(t, x, z, gamma_deg) of the model flight from the rail end (t = 0 at the end of the stroke).

    h0 is the camera height at the stroke START; the free flight starts where the dart leaves the rail, half a
    stroke further along the rail (the same start as analyze_flight's tf() and the range model)."""
    ts, xs, zs = ballistic(v0, pitch, h0 + v0 * STROKE_S / 2.0 * math.sin(math.radians(pitch)), dt=0.005)
    if len(ts) < 3:
        return ts, xs, zs, np.full(len(ts), pitch)
    gam = np.degrees(np.arctan2(np.gradient(zs, ts), np.gradient(xs, ts)))
    return ts, xs, zs, gam


def fuse_speed(pitch: float, h0: float, T_obs: Optional[float] = None, T_sigma: Optional[float] = None,
               T_min: Optional[float] = None, track: Sequence[Tuple[float, float]] = (),
               v_prior: Optional[float] = None, prior_sigma: Optional[float] = None, v_min: Optional[float] = None,
               lo: float = 3.0, hi: float = 16.0, launch_free: bool = False,
               launch_late_max: float = 0.10, launch_late_sigma: float = 0.06) -> Tuple[Optional[float], dict]:
    """Muzzle speed that best explains every cue, by a search over v0. Returns (v0 or None, details).

    Cues (each optional), all times measured from the end of the stroke (where the model flight starts):
    - T_obs: observed flight time. T_sigma None = video timing (asymmetric window above); a number = symmetric
      Gaussian (IMU times).
    - T_min: lower bound on the flight time (last frame that still showed the sky in flight).
    - track: [(t, horizon pitch deg)] during flight; the dart flies nose first, so its pitch follows the flight
      path angle gamma(t) of the model (plus wobble). The slope of tan(gamma) gives the horizontal speed.
    - v_prior: table muzzle speed (C3 v_muzzle) with prior_sigma relative spread.
    - v_min: lower bound (a saturated IMU push).
    launch_free: the launch time itself is uncertain (video: the first motion can be seen late when the camera
    looks at plain sky). A launch time shift d (true launch = observed - d, in [-0.04, launch_late_max] s) is
    searched too; it moves every time cue. details["launch_shift_s"] gives it."""
    cues = {"T_obs": T_obs, "T_min": T_min, "track_n": len(track), "v_prior": v_prior, "v_min": v_min}
    if T_obs is None and not track and v_prior is None:
        return None, cues
    prior_sigma = PRIOR_SIGMA if prior_sigma is None else prior_sigma
    tr = np.array(track, float) if len(track) else np.zeros((0, 2))
    shifts = np.arange(-0.04, launch_late_max + 1e-4, 0.01) if launch_free else np.array([0.0])

    def cost(v):
        ts, xs, zs, gam = _gamma_track(v, pitch, h0)
        T = float(ts[-1])
        best = (float("inf"), 0.0)
        for d in shifts:
            c = 0.0
            if launch_free:     # the video launch is more often late (sky view) than early
                c += (d / launch_late_sigma) ** 2 if d >= 0 else (d / 0.02) ** 2
            if v_prior is not None:
                c += ((v - v_prior) / (prior_sigma * v_prior)) ** 2
            if T_obs is not None:
                r = T - (T_obs + d)
                if T_sigma is not None:
                    c += (r / T_sigma) ** 2
                elif r < -VID_T_LATE:
                    c += ((r + VID_T_LATE) / 0.025) ** 2
                elif r > VID_T_EARLY:
                    c += ((r - VID_T_EARLY) / 0.015) ** 2
                else:
                    c += 0.5 * ((r + 0.03) / 0.07) ** 2      # mild pull to the typical 30 ms lateness
            if T_min is not None and T < T_min + d:
                c += ((T_min + d - T) / 0.015) ** 2
            if v_min is not None and v < v_min:
                c += ((v_min - v) / (0.03 * v_min)) ** 2
            if len(tr):
                tt = tr[:, 0] + d
                ok = (tt < T) & (tt > 0)
                if ok.any():
                    g = np.interp(tt[ok], ts, gam)
                    c += float(np.sum(((tr[ok, 1] - g) / TRACK_SIGMA_DEG) ** 2)) / TRACK_CORR
                c += float((tt >= T).sum()) * 4.0 / TRACK_CORR      # sky seen after the model already landed
            if c < best[0]:
                best = (c, float(d))
        return best

    grid = np.arange(lo, hi + 1e-9, 0.25)
    cs = [cost(float(v))[0] for v in grid]
    k = int(np.argmin(cs))
    fine = np.arange(max(lo, grid[k] - 0.3), min(hi, grid[k] + 0.3) + 1e-9, 0.02)
    cf = [cost(float(v)) for v in fine]
    j = int(np.argmin([c[0] for c in cf]))
    cues["cost"], cues["launch_shift_s"] = cf[j]
    return float(fine[j]), cues


# ------------------------------------------------------------------ IMU

def _parse_flightlog(fl) -> Optional[np.ndarray]:
    """C2 samples as an (n, 8) float array sorted by time. Rows that are short, non-numeric or non-finite are
    dropped, and duplicate timestamps keep their first row. None when fewer than 5 usable rows remain."""
    if not fl or not isinstance(fl, dict):
        return None
    s = fl.get("samples") or []
    if not isinstance(s, (list, tuple)):
        return None
    rows = []
    for r in s:
        if not isinstance(r, (list, tuple)) or len(r) < 8:
            continue
        try:
            v = [float(x) for x in r[:8]]
        except (TypeError, ValueError):
            continue
        if all(math.isfinite(x) for x in v):
            rows.append(v)
    if len(rows) < 5:
        return None
    a = np.array(rows, float)
    a = a[np.argsort(a[:, 0], kind="stable")]
    keep = np.concatenate([[True], np.diff(a[:, 0]) > 0])
    a = a[keep]
    return a if len(a) >= 5 else None


def _fl_num(fl, key) -> Optional[float]:
    """A finite number from the flightlog dict, else None (strings, null, NaN are ignored)."""
    v = fl.get(key) if isinstance(fl, dict) else None
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v)):
        return None
    return float(v)


def _runs(mask: np.ndarray, lo: int = 0) -> List[Tuple[int, int]]:
    """(first, last) index pairs of consecutive True runs in mask, starting at index lo."""
    out, i, n = [], max(0, lo), len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            out.append((i, j))
            i = j + 1
        else:
            i += 1
    return out


def imu_events(fl: dict) -> dict:
    """IMU phase times in dart seconds: launch, launch_end, impact, landed; plus rail attitude and v0.

    Follows the firmware detector (firmware/dart/API.md):
    - launch: the run of samples over 4 g closest to the flightlog's own t_launch_ms (within 60 ms) when it has
      one, else the first run of at least 20 ms (2 consecutive samples at 50 Hz), so a single glitch sample (latch
      click, knock while loading) is ignored. Launch time = the START of the window.
    - impact: searched only after the push has ended (|a| under 2.5 g) and at least 150 ms after launch; an impact
      is a run of samples over 3.5 g (firmware: the BNO085 clips at 8 g and low-pass filters, so a short impact
      can read under 6 g), and the landing impact is the LAST one (the one followed by stillness).
    - v0: integrated push. The accelerometer saturates near 78 m/s^2; when the push touches about 75 m/s^2 on any
      axis the integral is only a lower bound (v0_min, saturated=True) and v0 stays None so analyze_flight solves
      it from the flight time and the rail pitch."""
    a = _parse_flightlog(fl)
    out = {"launch": None, "launch_end": None, "impact": None, "landed": None, "v0": None, "v0_min": None,
           "saturated": False, "rail_rpy": None}
    fl_launch = _fl_num(fl, "t_launch_ms")
    fl_land = _fl_num(fl, "t_land_ms")
    if a is None:
        if fl_launch is not None:
            out["launch"] = fl_launch / 1000.0
        if fl_land is not None:
            out["impact"] = fl_land / 1000.0
        return out
    t = a[:, 0] / 1000.0
    acc = np.linalg.norm(a[:, 5:8], axis=1)
    dts = np.diff(t)
    dts = dts[dts > 0]
    dt = float(np.median(dts)) if len(dts) else 0.02
    dt = max(dt, 1e-3)
    runs4 = _runs(acc > 4 * G)
    run = None
    from_fl = False
    if fl_launch is not None and runs4:
        tl = fl_launch / 1000.0
        # distance from t_launch_ms to the run (0 when it lies inside the run's window)
        def dist(r):
            t0 = t[r[0]] - dt
            t1 = t[r[1]]
            return 0.0 if t0 <= tl <= t1 else min(abs(tl - t0), abs(tl - t1))
        best = min(runs4, key=dist)
        if dist(best) <= 0.06:
            run, from_fl = best, True
    no_push = False
    if run is None and fl_launch is not None:
        # the firmware says when the launch was, but no push lies there (the log starts after it, or dropped
        # samples): trust t_launch_ms, measure no rail attitude or v0, and look for the impact after it. Falling
        # back to "the first 4 g run" here would take the impact for the launch.
        no_push = True
    elif run is None:
        need = max(2, int(round(0.02 / dt)) + 1)       # samples spanning 20 ms (2 at 50 Hz)
        # a launch has the rail before it: at least 0.3 s of log (the firmware keeps 2 s)
        long_runs = [r for r in runs4 if r[1] - r[0] + 1 >= need and t[r[0]] - t[0] >= 0.3]
        if long_runs:
            run = long_runs[0]
    if run is None and not no_push:
        if fl_land is not None:
            out["impact"] = fl_land / 1000.0
        return out
    if no_push:
        out["launch"] = fl_launch / 1000.0
        out["no_push"] = True
        k = int(np.searchsorted(t, out["launch"]))
        i0, i_end = k, max(k - 1, 0)
        if fl_land is not None:
            out["impact"] = fl_land / 1000.0
    else:
        i0, i_end = run
    if no_push:
        pass
    elif from_fl:
        out["launch"] = fl_launch / 1000.0
    else:
        # the push started between the previous sample and this one
        out["launch"] = float(t[i0] - 0.5 * (t[i0] - t[i0 - 1])) if i0 > 0 else float(t[i0])
    t_launch = out["launch"]
    j = i_end
    while not no_push and j + 1 < len(acc) and acc[j + 1] > 2 * G:
        j += 1
    if not no_push:
        out["launch_end"] = float(t[j] + 0.5 * (t[min(j + 1, len(t) - 1)] - t[j]))
    # muzzle speed: integrate the axial specific force minus the gravity component along the rail
    if i0 > 0 and not no_push:
        q = a[i0 - 1, 1:5]
        _, pitch, _ = quat_to_rpy(*q)
        push = a[i0:j + 1, 5:8]
        # each sample stands for the time to the next one (irregular or dropped samples), at most 3 periods
        tt = t[i0:min(j + 2, len(t))]
        steps = np.diff(tt) if len(tt) > len(push) - 1 else np.array([])
        steps = np.concatenate([steps, np.full(max(0, len(push) - len(steps)), dt)])[:len(push)]
        steps = np.clip(steps, 0.0, 3.0 * dt)
        v0 = float(np.sum((push[:, 0] - G * math.sin(math.radians(pitch))) * steps))
        if push.size and float(np.abs(push).max()) >= SAT_NEAR:
            out["saturated"] = True
            if v0 > 0:
                out["v0_min"] = v0
        elif 3.0 < v0 < 30.0:
            out["v0"] = v0
        out["rail_rpy"] = quat_to_rpy(*q)
    # impact search: after the push has ended (under 2.5 g) and at least 150 ms after launch
    k0 = i0 if no_push else i_end + 1
    while k0 < len(acc) and acc[k0] >= 2.5 * G:
        k0 += 1
    while k0 < len(acc) and t[k0] < t_launch + 0.15:
        k0 += 1
    runs_imp = _runs(acc > IMPACT_G * G, k0)
    if not runs_imp:
        return out
    # the landing impact is the last one: the first impact followed by 1 s of stillness (0.85..1.15 g) before the
    # next impact; the log may end sooner, then 0.3 s still up to the end counts. No stillness: the last impact.
    band = (acc > 0.85 * G) & (acc < 1.15 * G)
    n_fw = max(3, int(round(1.0 / dt)))
    n_min = max(3, int(round(0.3 / dt)))
    chosen = runs_imp[-1]
    for ri, (s0, s1) in enumerate(runs_imp):
        last = ri + 1 == len(runs_imp)
        nxt = len(acc) if last else runs_imp[ri + 1][0]
        quiet = [r for r in _runs(band[:nxt], s1 + 1)
                 if r[1] - r[0] + 1 >= n_fw or (last and r[1] == len(acc) - 1 and r[1] - r[0] + 1 >= n_min)]
        if quiet:
            chosen = (s0, s1)
            break
    k, k_end = chosen
    out["impact"] = float(t[k] - 0.5 * (t[k] - t[k - 1])) if k > 0 else float(t[k])
    # still: the accelerometer vector stays at its final resting value (within 0.6 m/s^2) and |a| within
    # 0.05 g of 1 g, for 0.3 s (the vector test also catches sideways shake that hardly changes |a|)
    rest = np.median(a[-10:, 5:8], axis=0)
    still = (np.linalg.norm(a[:, 5:8] - rest, axis=1) < 0.6) & (np.abs(acc - G) < 0.05 * G)
    n_need = max(3, int(round(0.3 / dt)))
    for m in range(k_end + 1, len(acc) - n_need + 1):
        if still[m:m + n_need].all():
            out["landed"] = float(t[m])
            break
    return out


# ------------------------------------------------------------------ video phases

def _video_phases(times: np.ndarray, energy: np.ndarray, fps: float) -> dict:
    """Video-only launch / impact / landed times from motion energy of good frame pairs.

    times[i] is the time of the later frame of pair i, energy[i] the per-frame motion energy.
    - floor: 20th percentile of the energy (the static rail and landed parts dominate any flight video);
      threshold = floor + max(0.1, 6 * MAD of the quiet pairs) so even a mostly sky view on the rail shows the
      launch.
    - launch: first pair above threshold that is part of a sustained run (2 of 3), placed between the two frames.
    - landed: the later frame of the last moving pair.
    - impact: the nose hitting the ground ends the large flight motion abruptly (only a decaying shake follows),
      so impact = the last pair still above half of the peak energy of the final second before landed."""
    out = {"launch": None, "impact": None, "landed": None, "threshold": None, "floor": None}
    if len(energy) < 6:
        return out
    floor = float(np.percentile(energy, 20))
    quiet = energy[energy <= 2.0 * floor + 1e-6]
    mad = float(np.median(np.abs(quiet - np.median(quiet)))) if len(quiet) else 0.0
    thr = floor + max(0.1, 6.0 * mad, 0.15 * floor)
    out["threshold"], out["floor"] = thr, floor
    moving = energy > thr
    sm = moving.copy()
    for i in range(len(moving)):
        lo, hi = max(0, i - 1), min(len(moving), i + 2)
        sm[i] = moving[i] and moving[lo:hi].sum() >= 2
    idx = np.nonzero(sm)[0]
    if len(idx) == 0:
        return out
    i0 = int(idx[0])
    prev_t = times[i0 - 1] if i0 > 0 else times[i0] - 1.0 / fps
    out["launch"] = float(0.5 * (prev_t + times[i0]) - 0.25 / fps)
    i1 = int(idx[-1])
    out["landed"] = float(times[i1])
    win = np.nonzero((times >= times[i1] - 1.0) & (times <= times[i1]) & (np.arange(len(times)) >= i0))[0]
    if len(win):
        peak = float(energy[win].max())
        hi_idx = [k for k in win if energy[k] >= 0.5 * peak]
        k = int(hi_idx[-1])
        nxt = times[k + 1] if k + 1 < len(times) else times[k] + 1.0 / fps
        out["impact"] = float(times[k] + 0.25 * (nxt - times[k]))
    return out


# ------------------------------------------------------------------ main

def _bgr_frame(f):
    """The frame with a HxWx3 uint8 image (grey, HxWx1 and BGRA are converted), None when unusable."""
    img = f.img
    try:
        if not isinstance(img, np.ndarray) or img.size == 0 or img.ndim not in (2, 3):
            return None
        if img.dtype != np.uint8:
            img = np.clip(img, 0, 255).astype(np.uint8)
        b = ensure_bgr(img)
        if b.ndim != 3 or b.shape[2] != 3:
            return None
    except Exception:
        return None
    if b is f.img:
        return f
    try:
        return dataclasses.replace(f, img=b)
    except Exception:
        return Frame(img=b, t=f.t, idx=getattr(f, "idx", 0), quality=getattr(f, "quality", None),
                     bad=getattr(f, "bad", False))


def analyze_flight(frames: Sequence, flightlog: Optional[dict] = None, clock_offset_s: Optional[float] = None,
                   hfov_deg: float = 100.0, v0_default: Optional[float] = None, h0: float = RAIL_H,
                   ballistics=None, drag_n: float = 0.5, aim_pitch_deg: Optional[float] = None) -> FlightResult:
    """Analyse a dart flight video (list of Frame) with an optional C2 flightlog. Never raises on bad input.

    ballistics: a grappstar.ballistics.Ballistics (C3 table) used as the prior for the muzzle speed and to
    calibrate the range (the table is the launcher model; the simple drag model here only interpolates around it).
    drag_n: the clutch drag column (LIGHT 0.5 N at launch). aim_pitch_deg: the commanded rail pitch (a strong
    prior for the video-only path)."""
    res = FlightResult()
    frames = [_bgr_frame(f) for f in (frames or []) if f is not None and getattr(f, "img", None) is not None]
    frames = [f for f in frames if f is not None]
    frames.sort(key=lambda f: f.t)
    fps = 30.0
    if len(frames) >= 2:
        dts = np.diff([f.t for f in frames])
        dts = dts[dts > 0]
        if len(dts):
            fps = 1.0 / float(np.median(dts))

    # --- per frame quality, horizon, motion energy
    prev_small, prev_i = None, None
    m_t, m_e = [], []
    hor_by_idx = {}
    for i, f in enumerate(frames):
        ok = frame_ok(f)
        if not ok:
            res.horizon.append((f.t, None, None))
            continue
        res.good.append(f.idx)
        hz = detect_horizon(f.img, hfov_deg)
        hor_by_idx[i] = hz
        res.horizon.append((f.t, hz["angle_deg"], hz["sky_ratio"]))
        g = cv2.cvtColor(_small(f.img), cv2.COLOR_BGR2GRAY)
        g = cv2.GaussianBlur(g, (5, 5), 0).astype(np.int16)
        if prev_small is not None and prev_small.shape == g.shape:
            gap = max(1.0, (f.t - frames[prev_i].t) * fps)
            m_t.append(f.t)
            m_e.append(float(np.abs(g - prev_small).mean()) / gap)
        prev_small, prev_i = g, i
    res.motion = list(zip(m_t, m_e))
    if not frames:
        res.notes.append("no frames")
    elif len(res.good) < 3:
        res.notes.append("too few good frames (%d of %d)" % (len(res.good), len(frames)))
    vid = _video_phases(np.array(m_t), np.array(m_e), fps) if m_e else \
        {"launch": None, "impact": None, "landed": None}
    res.video_times = {k: vid.get(k) for k in ("launch", "impact", "landed")}

    # --- IMU
    try:
        imu = imu_events(flightlog) if flightlog else None
        samples = _parse_flightlog(flightlog) if flightlog else None
    except Exception as e:          # a garbage flightlog never costs the video result
        imu, samples = None, None
        res.notes.append("flightlog unusable (%s): video only" % type(e).__name__)
    if imu is not None and imu.get("no_push"):
        res.notes.append("flightlog has no push at t_launch_ms (log starts late?): no IMU rail attitude or speed")
    offset = clock_offset_s
    if imu is not None:
        res.imu_times = {k: imu.get(k) for k in ("launch", "launch_end", "impact", "landed")}
        if offset is None:
            # the video launch is sharp (within about 0.04 s); the video impact runs late by 0 to 0.11 s (the crash
            # and settle motion), so it is used only without a video launch, corrected by its typical lateness
            if imu["launch"] is not None and vid.get("launch") is not None:
                offset = float(vid["launch"] - imu["launch"])
                res.notes.append("clock offset from the video/IMU launch match")
            elif imu["impact"] is not None and vid.get("impact") is not None:
                offset = float(vid["impact"] - VID_IMPACT_LATE_TYP - imu["impact"])
                res.notes.append("clock offset from the video/IMU impact match (no video launch)")
            else:
                res.notes.append("cannot align flightlog to video (no launch seen in video)")
        res.clock_offset_s = offset

    use_imu = imu is not None and offset is not None and imu.get("launch") is not None
    t_first = frames[0].t if frames else None
    t_last = frames[-1].t if frames else None
    if use_imu:
        res.source = "video+imu" if vid.get("launch") is not None else "imu"
        sh = lambda v: None if v is None else v + offset  # noqa: E731
        t_launch, t_launch_end = sh(imu["launch"]), sh(imu.get("launch_end"))
        t_impact, t_landed = sh(imu.get("impact")), sh(imu.get("landed"))
        if t_impact is None:
            t_impact = vid.get("impact")
        if t_landed is None:
            t_landed = vid.get("landed")
        if imu.get("v0"):
            res.v0 = imu["v0"]
        elif imu.get("saturated"):
            res.notes.append("IMU push saturated: v0 from the flight time")
        if imu.get("rail_rpy"):
            res.launch_pitch_deg = imu["rail_rpy"][1]
            res.launch_yaw_deg = imu["rail_rpy"][2]
    else:
        res.source = "video" if vid.get("launch") is not None else "none"
        t_launch, t_impact, t_landed = vid.get("launch"), vid.get("impact"), vid.get("landed")
        t_launch_end = None if t_launch is None else t_launch + STROKE_S
    if t_launch_end is None and t_launch is not None:
        t_launch_end = t_launch + STROKE_S
    if t_landed is not None and t_impact is not None and t_landed < t_impact:
        t_landed = t_impact + 0.2
    res.t_launch, res.t_impact, res.t_landed = t_launch, t_impact, t_landed

    # --- phases
    if t_first is not None:
        t_end = max(t_last, t_landed or t_last)
        bounds = [("ON_RAIL", t_first), ("LAUNCH", t_launch), ("FLIGHT", t_launch_end), ("IMPACT", t_impact),
                  ("LANDED", t_landed)]
        known = [(n, t) for n, t in bounds if t is not None]
        for k, (n, t0) in enumerate(known):
            t1 = known[k + 1][1] if k + 1 < len(known) else t_end
            if t1 > t0 or k + 1 == len(known):
                res.phases.append((n, float(t0), float(max(t0, t1))))
        if t_launch is None:
            res.notes.append("no launch found")

    # --- attitude
    if samples is not None and offset is not None:
        for r in samples:
            roll, pitch, yaw = quat_to_rpy(*r[1:5])
            res.attitude.append((float(r[0] / 1000.0 + offset), roll, pitch, yaw))
    else:
        for i, hz in sorted(hor_by_idx.items()):
            if hz["angle_deg"] is not None:
                res.attitude.append((frames[i].t, hz["angle_deg"], hz["pitch_deg"], None))

    # --- launch pitch from the rail horizon when there is no IMU
    if res.launch_pitch_deg is None and t_launch is not None:
        pts = [hz["pitch_deg"] for i, hz in hor_by_idx.items()
               if frames[i].t < t_launch - 0.05 and hz["pitch_deg"] is not None]
        if len(pts) >= 3:
            res.launch_pitch_deg = float(np.median(pts))
            res.notes.append("launch pitch from the horizon on the rail")

    # --- trajectory
    if t_launch is not None and t_impact is not None and t_impact > t_launch + 0.2:
        T = t_impact - t_launch - STROKE_S
        res.flight_time_s = float(t_impact - t_launch)
        pitch, v0 = res.launch_pitch_deg, res.v0
        t_start = t_launch + STROKE_S
        if aim_pitch_deg is not None and math.isfinite(float(aim_pitch_deg)) and not (use_imu and imu.get("rail_rpy")):
            pitch = float(aim_pitch_deg)
            res.notes.append("launch pitch from the commanded rail pitch %.1f deg" % pitch)
        bal = _table(ballistics)
        if v0_default is None:      # the fallback speed comes from the C3 table when there is one
            try:
                v0_default = float(np.median([r["v_muzzle"] for r in bal.rows if r.get("v_muzzle")])) if bal else None
            except Exception:
                v0_default = None
            v0_default = v0_default or DEFAULT_V0

        def v_tab(p):
            if bal is None or p is None or not (bal.pitch_min - 3 <= p <= bal.pitch_max + 3):
                return None
            v = bal.at_pitch(p, drag_n).get("v_muzzle")
            return float(v) if v and v > 0 else None

        def tf(v, p):
            sl_ = v * STROKE_S / 2.0
            return float(ballistic(v, p, h0 + sl_ * math.sin(math.radians(p)))[0][-1])
        video_T = not use_imu
        # horizon track in flight and the last frame that still saw the sky (a lower bound on the flight time)
        track, t_sky = [], None
        t_end_track = t_impact if not video_T else (vid.get("impact") or t_impact)
        for i, hz in hor_by_idx.items():
            ft = frames[i].t
            if t_start + 0.03 < ft < t_end_track:
                if hz["sky_ratio"] is not None and hz["sky_ratio"] >= 0.01:
                    t_sky = ft if t_sky is None else max(t_sky, ft)
                if hz["pitch_deg"] is not None:
                    track.append((ft - t_start, float(hz["pitch_deg"])))
        v0_min = imu.get("v0_min") if imu else None
        # a rail view that is all sky hides the launch motion (no texture): the video launch can be late by more
        rail_sky = [hz["sky_ratio"] for i, hz in hor_by_idx.items()
                    if frames[i].t < t_launch - 0.05 and hz["sky_ratio"] is not None]
        sky_rail = len(rail_sky) >= 3 and float(np.median(rail_sky)) >= 0.97

        def fuse(p):
            vp_ = v_tab(p)
            return fuse_speed(p, h0, T_obs=T, T_sigma=None if video_T else 0.02,
                              T_min=None if t_sky is None else t_sky - t_start, track=track,
                              v_prior=vp_, v_min=v0_min, launch_free=video_T,
                              launch_late_max=0.18 if sky_rail else 0.10, launch_late_sigma=0.10 if sky_rail else 0.06,
                              # with a table the speed is searched only inside a band around it (a noisy
                              # flight time can never give a 2 or 30 m/s launcher)
                              lo=0.6 * vp_ if vp_ else 3.0, hi=1.4 * vp_ if vp_ else 16.0)
        if pitch is None and v0 is None and len(track) >= 4:
            # no pitch from the IMU, the rail horizon or the command: fit pitch and speed together to the horizon
            # track (its start says the launch angle, its slope the speed) and the flight time
            lo_p, hi_p = (bal.pitch_min, bal.pitch_max) if bal is not None else (4.0, 60.0)
            # a rail view that is all sky means the camera looks above the horizon by more than half the vertical
            # field of view: the launch pitch is at least that
            if sky_rail:
                hh, ww = frames[0].img.shape[:2]
                half_v = math.degrees(math.atan(math.tan(math.radians(hfov_deg) / 2.0) * hh / float(ww)))
                if half_v - 1.0 < hi_p:
                    lo_p = max(lo_p, half_v - 1.0)
                    res.notes.append("rail view all sky: launch pitch at least %.0f deg" % (half_v - 1.0))
            coarse = [(fuse(float(p))[1].get("cost", float("inf")), float(p))
                      for p in np.arange(lo_p, hi_p + 1e-9, 2.0)]
            p0 = min(coarse)[1]
            fine = [(fuse(float(p))[1].get("cost", float("inf")), float(p))
                    for p in np.arange(max(lo_p, p0 - 1.5), min(hi_p, p0 + 1.5) + 1e-9, 0.5)]
            pitch = min(fine)[1]
            res.notes.append("pitch %.1f deg fitted to the horizon track in flight" % pitch)
        if pitch is None:
            # no pitch and no usable track: solve it from the flight time at the table speed (or the default
            # speed), allowing for the typical 30 ms late video impact
            T_fit = T - (0.03 if video_T else 0.0)
            pitch = _bisect(lambda p: tf(v_tab(p) or v0 or v0_default, p), 2.0, 80.0, T_fit)
            if pitch is None:
                pitch = DEFAULT_PITCH
                res.notes.append("pitch default %.0f deg" % pitch)
            else:
                res.notes.append("pitch solved from flight time with v0 %.1f m/s" % (v_tab(pitch) or v0 or v0_default))
        if v0 is None:
            vp = v_tab(pitch)
            v_f, cues = fuse(pitch)
            if v_f is not None:
                v0 = v_f
                used = [n for n, ok in (("flight time", True), ("sky track", bool(track)),
                                        ("table prior", vp is not None), ("IMU lower bound", bool(v0_min))) if ok]
                res.notes.append("speed %.2f m/s fused from %s" % (v0, ", ".join(used)))
                d = float(cues.get("launch_shift_s") or 0.0)
                if video_T and abs(d) >= 0.005:
                    t_start -= d          # the trajectory starts at the launch the fit found
                    res.launch_fit_t = float(t_launch - d)
                    res.notes.append("video launch looks %d ms %s" % (round(abs(d) * 1000), "late" if d > 0 else "early"))
            elif v0_min:
                v0 = v0_min
                res.notes.append("speed from the saturated IMU push (lower bound) %.1f m/s" % v0)
            else:
                v0 = v0_default
                res.notes.append("speed default %.1f m/s" % v0)
        res.launch_pitch_deg, res.v0 = float(pitch), float(v0)
        # the camera starts at h0 at the stroke start and leaves the rail after about v0 * STROKE_S / 2
        def model_range(v):
            sl_ = v * STROKE_S / 2.0
            xx = ballistic(v, pitch, h0 + sl_ * math.sin(math.radians(pitch)))[1]
            return float(xx[-1] + sl_ * math.cos(math.radians(pitch)))
        sl = v0 * STROKE_S / 2.0
        x_off = sl * math.cos(math.radians(pitch))
        ts, xs, zs = ballistic(v0, pitch, h0 + sl * math.sin(math.radians(pitch)))
        xs = xs + x_off
        # calibrate the simple model to the C3 table at this pitch (the table is the launcher model)
        scale = 1.0
        vp = v_tab(pitch)
        if vp is not None and bal.pitch_min - 1e-6 <= pitch <= bal.pitch_max + 1e-6:
            r_tab = float(bal.at_pitch(pitch, drag_n)["range_m"])
            r_mod = model_range(vp)
            if r_mod > 0.1:
                scale = float(min(1.4, max(0.6, r_tab / r_mod)))
                if abs(scale - 1.0) > 0.005:
                    res.notes.append("range scaled x%.3f to the C3 table at %.1f deg" % (scale, pitch))
        xs = xs * scale
        res.range_scale = scale
        tt = np.arange(0.0, ts[-1], 1.0 / max(fps, 1.0))
        tt = np.append(tt, ts[-1])
        xi, zi = np.interp(tt, ts, xs), np.interp(tt, ts, zs)
        res.trajectory = [(float(t_start + a), float(b), 0.0, float(c)) for a, b, c in zip(tt, xi, zi)]
        res.range_m = float(xs[-1])
    elif t_launch is not None:
        res.notes.append("no impact found, no trajectory")
    return res


def _table(ballistics):
    """A Ballistics object, or None. Accepts an object with at_pitch(), a C3 dict or a path."""
    if ballistics is None:
        return None
    if hasattr(ballistics, "at_pitch"):
        return ballistics
    try:
        from ..ballistics import Ballistics
        if isinstance(ballistics, dict):
            return Ballistics(ballistics)
        if isinstance(ballistics, str):
            return Ballistics.load(ballistics)
    except Exception:
        return None
    return None


def sharpness(img: np.ndarray) -> float:
    g = cv2.cvtColor(_small(img, 320), cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(g, cv2.CV_32F).var())


def landing_frames(frames: Sequence, result: Optional[FlightResult], n: int = 15) -> list:
    """Up to n good frames after landing, sharpest first. Without a landed time, the last 2 s of video."""
    frames = [_bgr_frame(f) for f in (frames or []) if f is not None and getattr(f, "img", None) is not None]
    frames = [f for f in frames if f is not None]
    if not frames or n <= 0:
        return []
    t_land = None
    if result is not None:
        t_land = result.t_landed if result.t_landed is not None else result.t_impact
    if t_land is None:
        t_land = max(f.t for f in frames) - 2.0
    cand = [f for f in frames if f.t >= t_land + 0.05 and frame_ok(f)]
    cand.sort(key=lambda f: sharpness(f.img), reverse=True)
    return cand[:n]
