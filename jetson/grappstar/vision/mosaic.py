"""Aerial map (mosaic) of the ground seen by the dart nose camera during the flight, with the landing point.

build_mosaic(frames, landing_t=None, flight=None, ...) -> MosaicResult

This is Harry's "aerial terrain mapping" idea: while the dart arcs down, the nose camera sweeps the ground ahead of
the landing point; stitching those frames gives the operator a map of bedrock shelves vs dust traps around the
anchor.

Pipeline
--------
1. Frame selection: good frames (video quality) between the start of the flight and landing_t (the impact time;
   when None it comes from `flight` or from vision.flight.analyze_flight run on the frames). Only frames that
   mostly see ground (sky ratio below 0.5) are used, at most `max_frames` (evenly thinned, the last one kept).
   Frames are downscaled to `work_w` px wide (480 by default) for speed on a Jetson.
2. Ground mask per frame: not sky (vision.flight.sky_mask) and not the hazy band just below the horizon, so
   features are only taken on the ground plane.
3. ORB features, BFMatcher Hamming knn (k=2) with Lowe ratio 0.75, RANSAC homography (4 px). A link is accepted
   only with at least `min_inliers` inliers, inlier ratio 0.3, a moderate scale change (det of the affine part
   0.2 to 5), little shear (singular value ratio below 4) and little perspective; consecutive frames that fail are
   bridged (the next frame is matched to the same tail) up to 3 misses. Frames form runs; the longest run (latest
   on ties) is used and chained to its last frame (the reference, closest to the landing point). The chain is
   capped at `max_chain` links and every frame's footprint must stay within a bounded region around the
   reference (accumulated drift bound).
4. Warp into a canvas (reference coordinates with a margin, scaled so the longest side is <= max_canvas px), only
   the part of each frame whose homography is well conditioned (clipped where the projective depth gets small),
   blended by feathered weights (more weight to pixels seen at higher resolution).
5. Landing point: the sharpest landed frames (after landing) are matched to the reference and their image centre
   projected (the nose camera looks at the spot it hit); otherwise the image centre of the reference frame (the
   last good flight frame: the dart flies nose first, so its axis points at the impact point), extrapolated along
   the chain when the reference is well before impact. A red marker and a legend are drawn.

Never raises on bad input: with too few usable frames it returns a placeholder image and a note.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from grappstar.vision.flight import FlightResult, analyze_flight, detect_horizon, frame_ok, sharpness, sky_mask

RATIO = 0.75
RANSAC_PX = 4.0


@dataclass
class MosaicResult:
    image: np.ndarray
    used: List[int] = field(default_factory=list)          # frame idx stitched into the map
    rejected: List[int] = field(default_factory=list)      # frame idx considered but not used
    landing_xy: Optional[Tuple[float, float]] = None       # landing pixel in the map image
    ref_idx: Optional[int] = None                          # frame idx of the reference frame
    ref_to_canvas: Optional[np.ndarray] = None             # 3x3: full resolution reference px -> map px
    landing_source: Optional[str] = None                   # "landed_frame" | "ref_centre" | "extrapolated"
    reasons: Dict[int, str] = field(default_factory=dict)  # why frames were rejected
    notes: List[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "used": [int(i) for i in self.used],
            "rejected": [int(i) for i in self.rejected],
            "n_used": len(self.used),
            "landing_xy": None if self.landing_xy is None else [round(float(v), 1) for v in self.landing_xy],
            "ref_idx": None if self.ref_idx is None else int(self.ref_idx),
            "landing_source": self.landing_source,
            "size": None if self.image is None else [int(self.image.shape[1]), int(self.image.shape[0])],
            "reasons": {str(k): v for k, v in self.reasons.items()},
            "notes": list(self.notes),
        }


# ------------------------------------------------------------------ helpers

def _placeholder(text_lines: Sequence[str], size=(480, 320)) -> np.ndarray:
    img = np.full((size[1], size[0], 3), 40, np.uint8)
    for k, line in enumerate(text_lines):
        cv2.putText(img, line, (12, 40 + 26 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1, cv2.LINE_AA)
    return img


def _ground_mask(small: np.ndarray) -> np.ndarray:
    """255 where the pixel is ground and not near the horizon (same size as `small`)."""
    h, w = small.shape[:2]
    tiny_w = 160
    tiny = cv2.resize(small, (tiny_w, max(1, int(round(h * tiny_w / w)))), interpolation=cv2.INTER_AREA)
    sky = sky_mask(tiny)
    if sky.any():
        k = max(3, int(0.08 * tiny.shape[0]) | 1)
        sky = cv2.dilate(sky, np.ones((k, k), np.uint8))   # drop the far, hazy band under the horizon
    g = cv2.resize(cv2.bitwise_not(sky), (w, h), interpolation=cv2.INTER_NEAREST)
    g = cv2.erode(g, np.ones((5, 5), np.uint8))
    return g


class _Item:
    __slots__ = ("frame", "small", "gray", "mask", "kp", "des", "scale")

    def __init__(self, frame, work_w, orb):
        img = frame.img
        self.frame = frame
        self.scale = min(1.0, work_w / float(img.shape[1]))
        if self.scale < 1.0:
            img = cv2.resize(img, (int(round(img.shape[1] * self.scale)), int(round(img.shape[0] * self.scale))),
                             interpolation=cv2.INTER_AREA)
        self.small = img
        self.gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.mask = _ground_mask(img)
        self.kp, self.des = orb.detectAndCompute(self.gray, self.mask)


def _match(a: _Item, b: _Item, bf, min_inliers: int, loose: bool = False):
    """Homography mapping a's pixels into b's, or (None, reason)."""
    if a.des is None or b.des is None or len(a.kp) < 8 or len(b.kp) < 8:
        return None, "few features"
    try:
        knn = bf.knnMatch(a.des, b.des, k=2)
    except cv2.error:
        return None, "match error"
    good = [m[0] for m in knn if len(m) == 2 and m[0].distance < RATIO * m[1].distance]
    if len(good) < min_inliers:
        return None, "few matches (%d)" % len(good)
    src = np.float32([a.kp[m.queryIdx].pt for m in good])
    dst = np.float32([b.kp[m.trainIdx].pt for m in good])
    H, inl = cv2.findHomography(src, dst, cv2.RANSAC, RANSAC_PX, maxIters=2000, confidence=0.995)
    if H is None or inl is None:
        return None, "no homography"
    n_in = int(inl.sum())
    if n_in < min_inliers or n_in < 0.3 * len(good):
        return None, "few inliers (%d/%d)" % (n_in, len(good))
    if abs(H[2, 2]) < 1e-9:
        return None, "degenerate"
    H = H / H[2, 2]
    A = H[:2, :2]
    det = float(np.linalg.det(A))
    sv = np.linalg.svd(A, compute_uv=False)
    persp = float(np.hypot(H[2, 0], H[2, 1])) * a.gray.shape[1]
    lo, hi = (0.02, 50.0) if loose else (0.2, 5.0)
    if not (lo < det < hi):
        return None, "scale change %.2f" % det
    if sv[1] <= 1e-9 or sv[0] / sv[1] > (8.0 if loose else 4.0):
        return None, "shear"
    if persp > (3.0 if loose else 1.2):
        return None, "perspective %.2f" % persp
    h, w = a.gray.shape
    c = np.array([[0, 0, 1], [w, 0, 1], [w, h, 1], [0, h, 1]], float).T
    pc = H @ c
    if not loose and np.any(pc[2] <= 0):
        return None, "corner behind"
    return H, n_in


def _clip_poly(poly: np.ndarray, a: float, b: float, c: float) -> np.ndarray:
    """Sutherland-Hodgman clip of polygon (N, 2) to the half plane a*x + b*y + c >= 0."""
    out = []
    n = len(poly)
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        fp, fq = a * p[0] + b * p[1] + c, a * q[0] + b * q[1] + c
        if fp >= 0:
            out.append(p)
        if (fp >= 0) != (fq >= 0):
            t = fp / (fp - fq)
            out.append(p + t * (q - p))
    return np.array(out, float).reshape(-1, 2)


def _project(H, pts):
    p = np.column_stack([pts, np.ones(len(pts))]) @ H.T
    return p[:, :2] / p[:, 2:3]


def _valid_poly(H: np.ndarray, w: int, h: int, w_min_rel: float = 0.3):
    """Part of a w x h frame that H maps well: projective depth at least w_min_rel of the largest depth over the
    frame (cuts the band towards the horizon where the map would be stretched to infinity)."""
    pts = np.array([[0, 0, 1], [w, 0, 1], [w, h, 1], [0, h, 1], [w / 2.0, h / 2.0, 1]], float)
    ws = pts @ H[2]
    wmax = float(ws.max())
    if wmax <= 0:
        return None
    rect = np.array([[0, 0], [w, 0], [w, h], [0, h]], float)
    poly = _clip_poly(rect, H[2, 0], H[2, 1], H[2, 2] - w_min_rel * wmax)
    if len(poly) < 3:
        return None
    return poly


def _rot_from_rpy(roll: float, pitch: float, yaw: float = 0.0) -> np.ndarray:
    """World from body, R = Rz(yaw) Ry(-pitch) Rx(roll) (same convention as vision.flight)."""
    r, p, y = math.radians(roll), math.radians(pitch), math.radians(yaw)
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    ry = np.array([[math.cos(p), 0, -math.sin(p)], [0, 1, 0], [math.sin(p), 0, math.cos(p)]])
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    return rz @ ry @ rx


_R_BODY_CAM = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
# virtual top-down camera: looks down, image up = forward (+X), image right = -Y
_R_WORLD_NADIR = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def nadir_homography(roll: float, pitch: float, w: int, h: int, hfov_deg: float) -> np.ndarray:
    """Rotation-only homography from a camera at (roll, pitch) to a virtual camera looking straight down."""
    f = (w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    K = np.array([[f, 0, (w - 1) / 2.0], [0, f, (h - 1) / 2.0], [0, 0, 1.0]])
    R_wc = _rot_from_rpy(roll, pitch) @ _R_BODY_CAM
    R_vc = _R_WORLD_NADIR.T @ R_wc
    return K @ R_vc @ np.linalg.inv(K)


def _attitude_for(frame, flight: Optional[FlightResult], hfov_deg: float):
    """(roll, pitch) of a frame from the flight result (IMU or horizon) or from its own horizon, or None."""
    if flight is not None and flight.attitude:
        ts = np.array([a[0] for a in flight.attitude])
        k = int(np.argmin(np.abs(ts - frame.t)))
        a = flight.attitude[k]
        if abs(ts[k] - frame.t) < 0.06 and a[1] is not None and a[2] is not None:
            return float(a[1]), float(a[2])
    hz = detect_horizon(frame.img, hfov_deg)
    if hz["angle_deg"] is not None and hz["pitch_deg"] is not None:
        return float(hz["angle_deg"]), float(hz["pitch_deg"])
    return None


# ------------------------------------------------------------------ main

def build_mosaic(frames: Sequence, landing_t: Optional[float] = None, flight: Optional[FlightResult] = None,
                 max_frames: int = 40, work_w: int = 480, max_canvas: int = 2000, min_inliers: int = 15,
                 max_chain: int = 40, n_features: int = 1200, rectify: bool = True,
                 hfov_deg: float = 100.0) -> MosaicResult:
    """Stitch the flight frames into an aerial map and mark the landing point. Never raises on bad input."""
    try:
        return _build(frames, landing_t, flight, max_frames, work_w, max_canvas, min_inliers, max_chain,
                      n_features, rectify, hfov_deg)
    except Exception as e:  # keep the mission going: a failed map must not stop anything
        return MosaicResult(image=_placeholder(["mosaic failed", str(e)[:60]]), notes=["error: %s" % e])


def _build(frames, landing_t, flight, max_frames, work_w, max_canvas, min_inliers, max_chain, n_features,
           rectify, hfov_deg):
    frames = sorted([f for f in (frames or []) if f is not None and getattr(f, "img", None) is not None
                     and f.img.ndim == 3 and f.img.size > 0], key=lambda f: f.t)
    res = MosaicResult(image=None)
    if not frames:
        res.image = _placeholder(["no frames"])
        res.notes.append("no frames")
        return res
    fps = 30.0
    if len(frames) > 2:
        d = np.diff([f.t for f in frames])
        d = d[d > 0]
        if len(d):
            fps = 1.0 / float(np.median(d))

    # --- flight window
    t_a, t_b, t_landed = None, landing_t, None
    if flight is None and landing_t is None:
        flight = analyze_flight(frames)
    if flight is not None:
        t_a = flight.t_launch
        if t_b is None:
            t_b = flight.t_impact
        t_landed = flight.t_landed
    if t_b is None:
        res.notes.append("landing time unknown: using all good ground frames")
        t_b = frames[-1].t + 1.0
    if t_a is None:
        t_a = t_b - 2.5
    if t_landed is None:
        t_landed = t_b + 0.4

    orb = cv2.ORB_create(nfeatures=n_features, scaleFactor=1.2, nlevels=8, fastThreshold=10)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    cand = [f for f in frames if t_a + 0.05 < f.t < t_b - 0.4 / fps]
    rejected: Dict[int, str] = {}
    good = []
    for f in cand:
        if not frame_ok(f):
            rejected[f.idx] = "bad frame"
            continue
        good.append(f)
    items: List[_Item] = []
    for f in good:
        it = _Item(f, work_w, orb)
        ground = float((it.mask > 0).mean())
        if ground < 0.4:
            rejected[f.idx] = "mostly sky"
            continue
        items.append(it)
    if len(items) > max_frames:
        keep = set(np.round(np.linspace(0, len(items) - 1, max_frames)).astype(int).tolist())
        for k, it in enumerate(items):
            if k not in keep:
                rejected[it.frame.idx] = "thinned"
        items = [it for k, it in enumerate(items) if k in keep]
    if len(items) < 2:
        res.rejected = sorted(rejected)
        res.reasons = rejected
        res.notes.append("too few ground frames during the flight (%d)" % len(items))
        if items:
            return _single(res, items[0], frames, t_landed, work_w)
        res.image = _placeholder(["no aerial map:", "too few ground frames (%d)" % len(items)])
        return res

    # --- runs of linked frames (H maps the earlier frame into the later chain tail)
    runs: List[List[Tuple[_Item, Optional[np.ndarray]]]] = []
    cur: List[Tuple[_Item, Optional[np.ndarray]]] = [(items[0], None)]
    misses = 0
    pending: List[_Item] = []
    for it in items[1:]:
        H, info = _match(cur[-1][0], it, bf, min_inliers)
        if H is not None:
            for p in pending:
                rejected[p.frame.idx] = "no link"
            pending = []
            cur.append((it, H))
            misses = 0
        else:
            misses += 1
            pending.append(it)
            rejected.setdefault(it.frame.idx, str(info))
            if misses > 3:
                runs.append(cur)
                # restart from the oldest unlinked frame
                cur = [(pending[0], None)]
                rejected.pop(pending[0].frame.idx, None)
                rest = pending[1:]
                pending = []
                misses = 0
                for r in rest:
                    H2, info2 = _match(cur[-1][0], r, bf, min_inliers)
                    if H2 is not None:
                        cur.append((r, H2))
                        rejected.pop(r.frame.idx, None)
                    else:
                        rejected[r.frame.idx] = str(info2)
    runs.append(cur)
    best = max(range(len(runs)), key=lambda k: (len(runs[k]), k))
    run = runs[best]
    for k, r in enumerate(runs):
        if k != best:
            for it, _ in r:
                rejected[it.frame.idx] = "other run"
    if len(run) > max_chain + 1:
        for it, _ in run[:len(run) - max_chain - 1]:
            rejected[it.frame.idx] = "chain cap"
        run = run[len(run) - max_chain - 1:]
        run[0] = (run[0][0], None)

    ref = run[-1][0]
    rh, rw = ref.gray.shape
    # chain every frame of the run to the reference
    to_ref: List[Tuple[_Item, np.ndarray]] = [(ref, np.eye(3))]
    acc = np.eye(3)
    for k in range(len(run) - 1, 0, -1):
        acc = acc @ run[k][1]
        acc = acc / acc[2, 2]
        to_ref.append((run[k - 1][0], acc.copy()))
    to_ref.reverse()

    # --- map frame: top-down view through the attitude of the latest chain frame whose attitude is known
    #     (IMU, or the horizon), else the reference perspective
    B = np.eye(3)
    anchor = None
    if rectify:
        for it, H in reversed(to_ref):
            att = _attitude_for(it.frame, flight, hfov_deg)
            if att is not None and att[1] < -8.0:
                h, w = it.gray.shape
                B = nadir_homography(att[0], att[1], w, h, hfov_deg) @ np.linalg.inv(H)
                wc = float(B[2] @ np.array([rw / 2.0, rh / 2.0, 1.0]))   # keep w > 0 at the reference centre
                if abs(wc) > 1e-12:
                    B = B / wc
                anchor = it
                res.notes.append("top-down view from frame %d attitude (roll %.0f, pitch %.0f deg)"
                                 % (it.frame.idx, att[0], att[1]))
                break
        if anchor is None:
            res.notes.append("no attitude for the chain: map shown in the reference camera perspective")
    polys = []
    for it, H in to_ref:
        h, w = it.gray.shape
        Hm = B @ H
        wc = float(Hm[2] @ np.array([w / 2.0, h / 2.0, 1.0]))
        if wc < 0:
            Hm = -Hm
        poly = _valid_poly(Hm, w, h)
        if poly is None:
            rejected[it.frame.idx] = "behind reference"
            continue
        pp = _project(Hm, poly)
        if not np.all(np.isfinite(pp)):
            rejected[it.frame.idx] = "unstable"
            continue
        polys.append((it, Hm, poly, pp))
    if not polys:
        res.image = _placeholder(["no aerial map:", "no stable frames"])
        res.notes.append("no stable frames")
        return res
    # bounded region (drift / stretch bound): 3 footprints of the reference frame around it on every side
    refp = [p for p in polys if p[0] is ref]
    rp = refp[0][3] if refp else polys[-1][3]
    rx0, ry0 = rp.min(0)
    rx1, ry1 = rp.max(0)
    span = max(rx1 - rx0, ry1 - ry0)
    lim = np.array([rx0 - 3 * span, ry0 - 3 * span, rx1 + 3 * span, ry1 + 3 * span])
    kept = []
    for it, Hm, poly, pp in polys:
        area = abs(cv2.contourArea(pp.astype(np.float32)))
        if area > 60 * span * span:
            rejected[it.frame.idx] = "drift (footprint too large)"
            continue
        kept.append((it, Hm, poly, pp))
    polys = kept
    if not polys:
        res.image = _placeholder(["no aerial map:", "no stable frames"])
        return res
    allp = np.vstack([p[3] for p in polys])
    x0, y0 = np.maximum(allp.min(0), lim[:2])
    x1, y1 = np.minimum(allp.max(0), lim[2:])
    x0, y0 = min(x0, rx0), min(y0, ry0)
    x1, y1 = max(x1, rx1), max(y1, ry1)
    s = min(1.0, max_canvas / max(x1 - x0, y1 - y0))
    T = np.array([[s, 0, -x0 * s], [0, s, -y0 * s], [0, 0, 1.0]])
    cw = int(min(max_canvas, math.ceil((x1 - x0) * s) + 1))
    ch = int(min(max_canvas, math.ceil((y1 - y0) * s) + 1))

    acc_img = np.zeros((ch, cw, 3), np.float32)
    acc_w = np.zeros((ch, cw), np.float32)
    n_items = len(polys)
    for k, (it, H, poly, _pp) in enumerate(polys):
        h, w = it.gray.shape
        Hc = T @ H
        cp = _project(Hc, poly)
        bx0 = int(max(0, math.floor(cp[:, 0].min())))
        by0 = int(max(0, math.floor(cp[:, 1].min())))
        bx1 = int(min(cw, math.ceil(cp[:, 0].max()) + 1))
        by1 = int(min(ch, math.ceil(cp[:, 1].max()) + 1))
        if bx1 - bx0 < 2 or by1 - by0 < 2:
            rejected[it.frame.idx] = "outside map"
            continue
        Tr = np.array([[1, 0, -bx0], [0, 1, -by0], [0, 0, 1.0]])
        Hr = Tr @ Hc
        size = (bx1 - bx0, by1 - by0)
        # weight: ground mask feathered towards its edges, more weight for later (closer, sharper) frames
        dist = cv2.distanceTransform((it.mask > 0).astype(np.uint8), cv2.DIST_L2, 3)
        feather = np.clip(dist / 25.0, 0, 1).astype(np.float32)
        feather *= (0.5 + 0.5 * (k + 1) / n_items)
        wimg = cv2.warpPerspective(feather, Hr, size, flags=cv2.INTER_LINEAR, borderValue=0)
        valid = np.zeros((size[1], size[0]), np.uint8)
        cv2.fillPoly(valid, [np.round(cp - [bx0, by0]).astype(np.int32)], 1)
        wimg *= valid
        if wimg.max() <= 0:
            rejected[it.frame.idx] = "no ground in map"
            continue
        warped = cv2.warpPerspective(it.small, Hr, size, flags=cv2.INTER_LINEAR, borderValue=(0, 0, 0))
        acc_img[by0:by1, bx0:bx1] += warped.astype(np.float32) * wimg[..., None]
        acc_w[by0:by1, bx0:bx1] += wimg
        res.used.append(it.frame.idx)
        rejected.pop(it.frame.idx, None)
    covered = acc_w > 1e-4
    out = np.full((ch, cw, 3), 30, np.uint8)
    out[covered] = np.clip(acc_img[covered] / acc_w[covered][:, None], 0, 255).astype(np.uint8)

    res.ref_idx = ref.frame.idx
    res.ref_to_canvas = T @ B @ np.diag([ref.scale, ref.scale, 1.0])

    # --- landing point
    land = None
    lf = [f for f in frames if f.t >= t_landed + 0.1 and frame_ok(f)]
    if lf:
        lf = sorted(lf, key=lambda f: sharpness(f.img), reverse=True)[:3]
        for f in lf:
            li = _Item(f, work_w, orb)
            li.mask = np.full_like(li.gray, 255)
            li.mask[int(0.85 * li.mask.shape[0]):] = 0           # dirt over the bottom edge
            li.kp, li.des = orb.detectAndCompute(li.gray, li.mask)
            H, info = _match(li, ref, bf, max(12, min_inliers), loose=True)
            if H is not None:
                h, w = li.gray.shape
                p = _project(H, np.array([[w / 2.0, h / 2.0]]))[0]
                if -0.5 * rw < p[0] < 1.5 * rw and -0.5 * rh < p[1] < 1.5 * rh:
                    land = p
                    res.landing_source = "landed_frame"
                    break
    if land is None:
        land = np.array([rw / 2.0, rh / 2.0])
        res.landing_source = "ref_centre"
        gap = t_b - ref.frame.t
        if gap > 0.12 and len(to_ref) >= 2:
            prev, Hp = to_ref[-2]
            ph, pw = prev.gray.shape
            pc = _project(Hp, np.array([[pw / 2.0, ph / 2.0]]))[0]
            dt = ref.frame.t - prev.frame.t
            if dt > 0:
                land = land + 0.5 * (land - pc) / dt * gap
                res.landing_source = "extrapolated"
                res.notes.append("reference frame %.2f s before impact, landing point extrapolated" % gap)
    lp = _project(T @ B, np.array([land]))[0]
    # crop the canvas to the covered area (and the landing marker)
    ys, xs = np.nonzero(covered)
    if len(xs):
        pad = 30
        cx0 = int(max(0, min(xs.min(), lp[0] - 60) - pad))
        cy0 = int(max(0, min(ys.min(), lp[1] - 60) - pad))
        cx1 = int(min(cw, max(xs.max(), lp[0] + 60) + pad + 1))
        cy1 = int(min(ch, max(ys.max(), lp[1] + 60) + pad + 1))
        if cx1 - cx0 > 50 and cy1 - cy0 > 50:
            out = np.ascontiguousarray(out[cy0:cy1, cx0:cx1])
            Tc = np.array([[1, 0, -cx0], [0, 1, -cy0], [0, 0, 1.0]])
            T = Tc @ T
            res.ref_to_canvas = T @ B @ np.diag([ref.scale, ref.scale, 1.0])
            lp = lp - [cx0, cy0]
    res.landing_xy = (float(lp[0]), float(lp[1]))
    _draw_marker(out, lp, len(res.used))
    res.image = out
    res.rejected = sorted(i for i in rejected if i not in res.used)
    res.reasons = {i: rejected[i] for i in res.rejected}
    if len(res.used) < 3:
        res.notes.append("only %d frames stitched" % len(res.used))
    return res


def _single(res: MosaicResult, it: _Item, frames, t_landed, work_w) -> MosaicResult:
    """One usable ground frame: show it as the map with its centre as the landing estimate."""
    img = it.small.copy()
    h, w = img.shape[:2]
    p = np.array([w / 2.0, h / 2.0])
    res.used = [it.frame.idx]
    res.ref_idx = it.frame.idx
    res.ref_to_canvas = np.diag([it.scale, it.scale, 1.0])
    res.landing_xy = (float(p[0]), float(p[1]))
    res.landing_source = "ref_centre"
    _draw_marker(img, p, 1)
    res.image = img
    return res


def _draw_marker(img: np.ndarray, p, n_used: int) -> None:
    h, w = img.shape[:2]
    x, y = int(round(p[0])), int(round(p[1]))
    r = max(10, int(0.015 * max(h, w)))
    for col, th in (((255, 255, 255), 5), ((0, 0, 255), 2)):
        cv2.circle(img, (x, y), r, col, th, cv2.LINE_AA)
        cv2.line(img, (x - 2 * r, y), (x - r // 2, y), col, th, cv2.LINE_AA)
        cv2.line(img, (x + r // 2, y), (x + 2 * r, y), col, th, cv2.LINE_AA)
        cv2.line(img, (x, y - 2 * r), (x, y - r // 2), col, th, cv2.LINE_AA)
        cv2.line(img, (x, y + r // 2), (x, y + 2 * r), col, th, cv2.LINE_AA)
    fs = max(0.45, min(1.0, w / 1400.0))
    lines = ["GrappStar aerial map (dart nose camera)", "%d frames stitched" % n_used, "red target = landing point"]
    lh = int(26 * fs / 0.6)
    bw = int(380 * fs / 0.6)
    bh = lh * len(lines) + 10
    y0 = h - bh - 8
    overlay = img.copy()
    cv2.rectangle(overlay, (8, y0), (8 + bw, y0 + bh), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, img, 0.4, 0, dst=img)
    for k, line in enumerate(lines):
        col = (80, 80, 255) if k == 2 else (240, 240, 240)
        cv2.putText(img, line, (16, y0 + lh * (k + 1) - 4), cv2.FONT_HERSHEY_SIMPLEX, fs, col, 1, cv2.LINE_AA)
