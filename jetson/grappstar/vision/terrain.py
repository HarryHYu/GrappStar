"""Landing zone verdict from the dart nose camera.

After landing, the dart nose camera (analog Caddx Ant, 720x480 after capture, noisy) looks
at the patch it landed on from very close. The question (Harry: "tell solid bedrock shelves
from bottomless dust traps") is whether the dart can anchor there: a kick leg plants a foot
auger, then two drill augers (16 to 17 mm diameter, 70 mm long) screw into the ground and
the rover winches itself towards the dart.

    report = analyze_landing(frames, model=None)
    report.cls, report.probs, report.slope_deg, report.score, report.verdict, report.reasons
    report.image  (960x540 BGR composite), report.to_dict()

Pipeline
--------
0. Dark landing check (on all frames, before any quality filtering): if most frames are almost
   black the verdict is NO_GO at once (see "Dark landings" below).
1. Frame selection: drop frames marked bad or with quality below `min_quality`. Frames without a
   quality value get the cheap link check of vision.flight (blue or black screen, snow, torn),
   unless that would drop all of them. Then keep the `n_best` sharpest (Laplacian variance of
   the central region).
2. Central ROI: the central square (60 percent of the short side) is resized to 192x192. The
   edges are skipped because the dart body, legs, buried nose dirt and lens vignetting live
   there. The fixed ROI size also normalises the capture resolution (720x480 and 480x320 give the
   same angular scale).
3. Scale tolerant classical features (numpy and OpenCV only, see FEATURE_NAMES and
   extract_features): brightness, Lab and HSV colour moments, green, dark and specular
   fractions, chroma variation; contrast relative to the mean brightness overall and per octave
   (difference of Gaussians at sigma 1 to 16) with the band centroid and peak; edge density,
   uniform LBP histogram, gradient orientation entropy and structure tensor coherence pooled over
   a three level pyramid; lag-1 autocorrelation at two levels; a multi scale crack measure.
   Ratios instead of absolute values wherever possible, so exposure, gain and the unknown
   distance to the ground matter less.
4. Classifier: z-scored features projected by a regularised linear discriminant (LDA), then up
   to two diagonal Gaussian prototypes per class in that space; probabilities are a softmax
   over negative squared distances. Learned from procedural textures seen at many scales, blurs,
   exposures and noise levels plus the landing frames of simulated flights (seeds 10 and up),
   stored in _PROTOTYPES_JSON, regenerated with `python -m grappstar.vision.terrain --learn
   --write`. Three hand rules sit on top: lots of green with edges pushes towards vegetation, a
   mostly dark patch and bright glints on a dark smooth patch push towards water_void.
5. "unknown" when frames are missing or poor, the image looks like pure noise or is blank, the
   feature vector is far from every class (per class diagonal Gaussians in z space, threshold
   learned from the training data), or the top probability is low.
6. Optional learned model (TerrainModel) fused with the classical probabilities by a
   weighted average.
7. Anchor score 0..100, verdict GO (>= 60), UNSURE (40..59), NO_GO (< 40), with reasons.

Anchor scoring rationale (base score per class, before slope and confidence)
---------------------------------------------------------------------------
The anchor is a set of small screw augers, so what matters is whether the threads can
cut in AND hold when the rover pulls:

- hard_ground 90: compact earth. The augers screw in and the threads bear on firm soil:
  the best anchor.
- loose_soil 65: augers go in easily but pull-out resistance is lower. Usable (just GO),
  winch gently.
- vegetation 55: roots add holding but the cover can hide rock, water or loose ground,
  so on its own it is UNSURE.
- sand_dust 25: low shear strength, threads pull out, and the dart may sink. This is the
  "bottomless dust trap" in Harry's description.
- bedrock 15: a solid shelf is great for a drilled expansion bolt but 16 to 17 mm by
  70 mm screw augers cannot penetrate it. Visible cracks raise it to 30 (an auger might
  wedge in a crack) but it stays NO_GO and the reasons say so.
- water_void 0: nothing to hold. Always NO_GO.
- unknown 45: forces UNSURE so the operator decides.

The score is the probability weighted mean of the base scores, so an uncertain mix pulls
towards the middle (for example 50/50 hard ground and sand gives 57, UNSURE). Slope
subtracts points above 10 degrees. Low image quality or low confidence shrinks the score
towards 50 (UNSURE) and very poor quality (below 0.5) clamps the verdict to UNSURE.

Dark landings
-------------
An almost black landing picture (central region median grey below 30 of 255 in at least 60
percent of the landing frames, whatever their quality flags say) is a hole, crevasse, deep
shadow, dark water at low light, or the nose buried in soft ground. video.quality marks such
near uniform frames as dropouts, so simply skipping them would end in "no usable frames" and
UNSURE, hiding the most dangerous case. Decision: it is checked first and always gives NO_GO.
With some texture or sensor noise in the dark frames it is reported as water_void (score 5);
a perfectly flat black screen (grey std below 0.6, the receiver's "no signal" screen looks like
that) is reported as unknown but still NO_GO (score 30), with reasons that say the link may be
down and the operator must look before winching.

Slope
-----
If the dart attitude is known (imu_roll_deg, imu_pitch_deg: tilt of the dart body from its
upright nose-down resting pose) the tilt is used as the slope estimate. This is a proxy:
the dart can lean in soft soil, and a lean is also bad for the augers. Without IMU data a
rough texture gradient cue is used: on a tilted plane the far side shows finer texture.
That estimate is cheap and approximate (several degrees of noise, fooled by uneven
texture), so it is only used for a small penalty and the reasons say "rough".
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np

CLASSES = ("bedrock", "hard_ground", "loose_soil", "sand_dust", "vegetation", "water_void")
ALL_CLASSES = CLASSES + ("unknown",)

BASE_SCORE = {
    "hard_ground": 90.0,
    "loose_soil": 65.0,
    "vegetation": 55.0,
    "sand_dust": 25.0,
    "bedrock": 15.0,
    "water_void": 0.0,
    "unknown": 45.0,
}
BEDROCK_CRACK_BONUS = 15.0

GO_MIN = 60
UNSURE_MIN = 40

ROI_FRAC = 0.6
ROI_SIZE = 192

FEATURE_NAMES = (
    "v_mean", "lab_a", "lab_b", "sat_mean", "hue_cos", "hue_sin",
    "green_frac", "dark_frac", "spec_frac", "chroma_std",
    "log_cv", "band0", "band1", "band2", "band3", "band4", "band_centroid", "band_peak",
    "edge_max", "edge_mean", "lbp_entropy", "lbp_flat", "lbp_nonuni",
    "orient_entropy", "coherence", "autocorr0", "autocorr2", "crack_frac",
)
FEATURE_VERSION = 6

CLASS_COLOURS = {  # BGR, for bars in the report image
    "bedrock": (140, 140, 150), "hard_ground": (80, 130, 190), "loose_soil": (50, 80, 120),
    "sand_dust": (150, 200, 230), "vegetation": (60, 170, 60), "water_void": (150, 80, 30),
    "unknown": (120, 120, 120),
}
VERDICT_COLOURS = {"GO": (60, 180, 60), "UNSURE": (0, 170, 230), "NO_GO": (40, 40, 220)}


# ============================================================ models (pluggable)

class TerrainModel:
    """Interface for a learned terrain classifier: predict(img) -> {class: prob}.

    `img` is a BGR uint8 image (the central landing ROI is passed). Class names outside
    CLASSES are ignored by the fusion step.
    """
    name = "model"

    def predict(self, img: np.ndarray) -> Dict[str, float]:  # pragma: no cover - interface
        raise NotImplementedError


class StubModel(TerrainModel):
    """Returns fixed probabilities (uniform by default). For tests and dry runs."""
    name = "stub"

    def __init__(self, probs: Optional[Dict[str, float]] = None):
        self.probs = dict(probs) if probs else {c: 1.0 / len(CLASSES) for c in CLASSES}

    def predict(self, img: np.ndarray) -> Dict[str, float]:
        return dict(self.probs)


def _softmax(x: np.ndarray) -> np.ndarray:
    x = np.asarray(x, np.float64)
    e = np.exp(x - x.max())
    return e / e.sum()


class OnnxTerrainModel(TerrainModel):
    """Image classification ONNX model run through onnxruntime (imported lazily).

    Expected model input: one image, RGB, NCHW float32, values 0..1, shape
    (1, 3, input_size, input_size). With imagenet_norm=True the input is further
    normalised with ImageNet mean (0.485, 0.456, 0.406) and std (0.229, 0.224, 0.225)
    (typical for torchvision exports). YOLOv8n-cls (Ultralytics) exports expect the plain
    0..1 RGB input (imagenet_norm=False, the default) and output (1, N) probabilities that
    already sum to 1; other models may output logits. Both are handled: if the output is
    not a probability vector, a softmax is applied.

    class_names: list of N names in output order. If omitted, the Ultralytics "names"
    metadata entry is used when present, else CLASSES.

    Jetson / TensorRT (optional): either convert once with
        /usr/src/tensorrt/bin/trtexec --onnx=terrain.onnx --saveEngine=terrain.engine --fp16
    to check the model builds and time it, or let onnxruntime-gpu build and cache the
    engine itself: the default providers list is TensorrtExecutionProvider,
    CUDAExecutionProvider, CPUExecutionProvider (filtered by what is installed), and
    provider_options can enable trt_engine_cache_enable / trt_engine_cache_path so the
    engine is built only on the first run.
    """
    name = "onnx"

    def __init__(self, path: str, class_names: Optional[Sequence[str]] = None,
                 input_size: int = 224, imagenet_norm: bool = False,
                 providers: Optional[Sequence[str]] = None, center_crop: bool = True,
                 provider_options: Optional[list] = None):
        import onnxruntime as ort  # lazy: optional dependency
        self.path = path
        self.input_size = int(input_size)
        self.imagenet_norm = bool(imagenet_norm)
        self.center_crop = bool(center_crop)
        want = list(providers) if providers else [
            "TensorrtExecutionProvider", "CUDAExecutionProvider", "CPUExecutionProvider"]
        try:
            avail = set(ort.get_available_providers())
            use = [p for p in want if p in avail] or ["CPUExecutionProvider"]
        except Exception:
            use = want
        kwargs = {"providers": use}
        if provider_options is not None:
            kwargs["provider_options"] = provider_options
        self.session = ort.InferenceSession(path, **kwargs)
        self.input_name = self.session.get_inputs()[0].name
        names = list(class_names) if class_names else self._names_from_metadata()
        self.class_names = names or list(CLASSES)

    def _names_from_metadata(self) -> Optional[List[str]]:
        try:
            import ast
            meta = self.session.get_modelmeta().custom_metadata_map
            raw = meta.get("names")
            if not raw:
                return None
            d = ast.literal_eval(raw)
            if isinstance(d, dict):
                return [str(d[k]) for k in sorted(d)]
            return [str(x) for x in d]
        except Exception:
            return None

    def preprocess(self, img: np.ndarray) -> np.ndarray:
        if self.center_crop:
            h, w = img.shape[:2]
            s = min(h, w)
            img = img[(h - s) // 2:(h - s) // 2 + s, (w - s) // 2:(w - s) // 2 + s]
        x = cv2.resize(img, (self.input_size, self.input_size), interpolation=cv2.INTER_AREA)
        x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        if self.imagenet_norm:
            x = (x - np.array([0.485, 0.456, 0.406], np.float32)) / np.array(
                [0.229, 0.224, 0.225], np.float32)
        return np.ascontiguousarray(x.transpose(2, 0, 1)[None])

    def predict(self, img: np.ndarray) -> Dict[str, float]:
        out = self.session.run(None, {self.input_name: self.preprocess(img)})[0]
        v = np.asarray(out, np.float64).reshape(-1)
        if not (v.min() >= -1e-6 and v.max() <= 1 + 1e-6 and abs(v.sum() - 1.0) < 1e-3):
            v = _softmax(v)
        return {str(self.class_names[i]): float(v[i]) for i in range(min(len(v), len(self.class_names)))}


# ============================================================ features

def central_roi(img: np.ndarray, frac: float = ROI_FRAC, size: int = ROI_SIZE) -> np.ndarray:
    """Central square crop (frac of the short side), resized to size x size.

    Resizing the crop to a fixed pixel size normalises the angular scale: the same camera captured
    at 720x480 or at 480x320 gives the same ROI scale (only the detail differs).
    """
    h, w = img.shape[:2]
    s = max(8, int(min(h, w) * frac))
    y0, x0 = (h - s) // 2, (w - s) // 2
    crop = img[y0:y0 + s, x0:x0 + s]
    interp = cv2.INTER_AREA if s > size else cv2.INTER_LINEAR
    return cv2.resize(crop, (size, size), interpolation=interp)


def _lbp_hist(g: np.ndarray, tol: float = 3.0) -> np.ndarray:
    """Uniform LBP (8 neighbours, radius 1) histogram with 10 bins (0..8 ones, 9 = non-uniform)."""
    c = g[1:-1, 1:-1]
    offs = [(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)]
    bits = []
    for dy, dx in offs:
        n = g[1 + dy:g.shape[0] - 1 + dy, 1 + dx:g.shape[1] - 1 + dx]
        bits.append((n >= c + tol).astype(np.int8))
    bits = np.stack(bits, 0)
    ones = bits.sum(0)
    trans = np.abs(bits - np.roll(bits, 1, axis=0)).sum(0)
    code = np.where(trans <= 2, ones, 9)
    hist = np.bincount(code.ravel(), minlength=10).astype(np.float64)
    return hist / hist.sum()


def _entropy(p: np.ndarray) -> float:
    p = p[p > 0]
    return float(-(p * np.log2(p)).sum())


def _autocorr1(g: np.ndarray) -> float:
    g = g.astype(np.float64)
    g = g - g.mean()
    var = (g * g).mean()
    if var < 1e-6:
        return 1.0
    ch = (g[:, 1:] * g[:, :-1]).mean() / var
    cv = (g[1:, :] * g[:-1, :]).mean() / var
    return float(max(ch, cv))


def _half(g: np.ndarray) -> np.ndarray:
    return cv2.resize(g, (g.shape[1] // 2, g.shape[0] // 2), interpolation=cv2.INTER_AREA)


def _level_texture(x: np.ndarray) -> Dict[str, float]:
    """Contrast normalised texture measures of one pyramid level (float32 grey)."""
    xb = cv2.GaussianBlur(x, (0, 0), 0.8)
    sd = max(4.0, float(xb.std()))            # floor: do not blow up the noise of a flat patch
    xu = np.clip((xb - float(xb.mean())) / sd * 40.0 + 128.0, 0, 255).astype(np.uint8)
    out = {"edge": float((cv2.Canny(xu, 60, 140) > 0).mean())}
    hist = _lbp_hist(xb, 0.15 * sd + 1.0)
    out["lbp_entropy"] = _entropy(hist) / math.log2(10)
    out["lbp_flat"] = float(hist[0] + hist[8])
    out["lbp_nonuni"] = float(hist[9])
    gx = cv2.Sobel(xb, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(xb, cv2.CV_32F, 0, 1, ksize=3)
    mag = np.sqrt(gx * gx + gy * gy)
    ang = np.arctan2(gy, gx) % np.pi
    oh, _ = np.histogram(ang, bins=18, range=(0, np.pi), weights=mag)
    out["orient"] = _entropy(oh / oh.sum()) / math.log2(18) if oh.sum() > 0 else 1.0
    jxx = cv2.GaussianBlur(gx * gx, (0, 0), 3.0)
    jyy = cv2.GaussianBlur(gy * gy, (0, 0), 3.0)
    jxy = cv2.GaussianBlur(gx * gy, (0, 0), 3.0)
    coh = np.sqrt((jxx - jyy) ** 2 + 4.0 * jxy ** 2) / (jxx + jyy + 16.0)
    out["coherence"] = float(coh.mean())
    out["autocorr"] = _autocorr1(x)
    return out


def extract_features(roi: np.ndarray) -> Dict[str, float]:
    """Scale tolerant texture and colour features of a BGR ROI (ideally ROI_SIZE square).

    Colour features do not depend on scale. Texture is measured as contrast relative to the mean
    brightness (so exposure and gain cancel), per octave (band0..band4, difference of Gaussians at
    sigma 1, 2, 4, 8, 16) plus scale free summaries (band centroid and peak), and edge, LBP,
    orientation and coherence measures pooled over a three level pyramid (max, mean or min over the
    levels), so a texture seen two to four times larger still gives similar values.
    """
    grey_u8 = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    g = grey_u8.astype(np.float32)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    H = hsv[..., 0].astype(np.float32)
    S = hsv[..., 1].astype(np.float32)
    V = hsv[..., 2].astype(np.float32)
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB).astype(np.float32)

    f: Dict[str, float] = {}
    f["v_mean"] = float(V.mean() / 255.0)
    f["lab_a"] = float((lab[..., 1].mean() - 128.0) / 128.0)
    f["lab_b"] = float((lab[..., 2].mean() - 128.0) / 128.0)
    f["sat_mean"] = float(S.mean() / 255.0)
    hr = H * (2 * np.pi / 180.0)
    wsum = max(1e-6, float(S.sum()))
    f["hue_cos"] = float((np.cos(hr) * S).sum() / wsum)
    f["hue_sin"] = float((np.sin(hr) * S).sum() / wsum)
    f["green_frac"] = float(((H >= 35) & (H <= 85) & (S > 50) & (V > 35)).mean())
    f["dark_frac"] = float((V < 45).mean())
    f["spec_frac"] = float(((V > 220) & (S < 50)).mean())
    f["chroma_std"] = float(math.log10(1.0 + float(lab[..., 1].std()) + float(lab[..., 2].std())))

    den = float(g.mean()) + 12.0
    f["log_cv"] = float(math.log10(1e-3 + float(g.std()) / den))
    blurs = [g] + [cv2.GaussianBlur(g, (0, 0), s) for s in (1.0, 2.0, 4.0, 8.0, 16.0)]
    e = [float((blurs[i] - blurs[i + 1]).std()) / den for i in range(5)]
    le = [math.log10(1e-4 + x) for x in e]
    for i, x in enumerate(le):
        f["band%d" % i] = x
    p = np.array(e) / (sum(e) + 1e-9)
    f["band_centroid"] = float((p * np.arange(5)).sum())
    f["band_peak"] = float(max(le))

    levels = [g, _half(g)]
    levels.append(_half(levels[1]))
    lt = [_level_texture(x) for x in levels]
    f["edge_max"] = max(t["edge"] for t in lt)
    f["edge_mean"] = float(np.mean([t["edge"] for t in lt]))
    f["lbp_entropy"] = float(np.mean([t["lbp_entropy"] for t in lt]))
    f["lbp_flat"] = float(np.mean([t["lbp_flat"] for t in lt]))
    f["lbp_nonuni"] = float(np.mean([t["lbp_nonuni"] for t in lt]))
    f["orient_entropy"] = float(min(t["orient"] for t in lt))
    f["coherence"] = float(max(t["coherence"] for t in lt))
    f["autocorr0"] = lt[0]["autocorr"]
    f["autocorr2"] = lt[2]["autocorr"]
    f["crack_frac"] = float(math.sqrt(crack_fraction(roi, grey=grey_u8, half=levels[1])))
    return f


def _vec(f: Dict[str, float]) -> np.ndarray:
    return np.array([f[k] for k in FEATURE_NAMES], np.float64)


def _crack_level(g: np.ndarray, k: int) -> float:
    bh = cv2.morphologyEx(g, cv2.MORPH_BLACKHAT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    thr = max(12.0, 3.0 * float(np.median(bh)) + 6.0)
    m = (bh > thr).astype(np.uint8)
    n, _lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    min_len = 0.15 * max(g.shape)
    total = 0
    for i in range(1, n):
        w, h, a = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT], stats[i, cv2.CC_STAT_AREA]
        if max(w, h) >= min_len and a < 0.25 * w * h + 1:
            total += a
    return float(total) / m.size


def crack_fraction(roi: np.ndarray, grey: Optional[np.ndarray] = None, half: Optional[np.ndarray] = None) -> float:
    """Fraction of ROI pixels on long thin dark lines (black-hat filter, elongated blobs).

    Measured at full and half ROI resolution (the larger of the two), with the minimum line length
    relative to the ROI, so a crack seen closer up (wider, blurrier) is still found.
    """
    g = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if grey is None else grey
    h2 = _half(g.astype(np.float32)) if half is None else half
    return max(_crack_level(cv2.GaussianBlur(g, (3, 3), 0), 9),
               _crack_level(np.clip(h2, 0, 255).astype(np.uint8), 7))


# ============================================================ prototypes
#
# Classifier: the features are z-scored (mu, sd), projected by a regularised linear discriminant
# (LDA, "lda_w", 5 axes, scaled so every class has unit spread), and compared with one Gaussian
# prototype per class in that space. LDA keeps the feature combinations that separate the classes
# across all training conditions (scale, blur, exposure, noise) and drops the ones that mostly
# vary with them, which is what makes the classifier tolerant. Per class diagonal Gaussians in the
# full z space ("classes") are kept for the out of distribution check.
#
# Training data (learn_prototypes, `python -m grappstar.vision.terrain --learn [--write]`):
# - procedural textures (sim.terrain_textures.texture_view) at zoom 1 to 4, blur, exposure (gain,
#   gamma), Gaussian and analog noise, at 720x480 and 480x320; a quarter at the native scale;
# - landing frames of full simulated flights (sim.flight_video.render_flight, seeds 10 and up, both
#   frame sizes, dropouts on), picked by vision.flight.landing_frames exactly as the mission does.
# Seeds 0..5 are never used for learning: tests/eval_terrain_flight.py tests on them.

_PROTOTYPES_JSON = r"""
{"version": 6, "features": ["v_mean", "lab_a", "lab_b", "sat_mean", "hue_cos", "hue_sin", "green_frac",
"dark_frac", "spec_frac", "chroma_std", "log_cv", "band0", "band1", "band2", "band3", "band4",
"band_centroid", "band_peak", "edge_max", "edge_mean", "lbp_entropy", "lbp_flat", "lbp_nonuni",
"orient_entropy", "coherence", "autocorr0", "autocorr2", "crack_frac"], "mu": [0.47209, -0.01197, 0.0976,
0.39209, 0.38381, 0.3849, 0.153, 0.07451, 0.0167, 0.95362, -0.98529, -1.57072, -1.8002, -1.72605, -1.71713,
-1.75957, 1.81495, -1.42616, 0.2592, 0.19269, 0.51137, 0.54576, 0.03651, 0.92654, 0.45774, 0.79488, 0.7672,
0.04812], "sd": [0.20969, 0.09093, 0.10694, 0.1824, 0.61419, 0.43608, 0.31664, 0.21966, 0.1085, 0.28992,
0.41164, 0.37828, 0.4436, 0.52693, 0.56657, 0.54856, 0.59642, 0.38858, 0.12614, 0.11569, 0.27136, 0.29033,
0.02895, 0.10992, 0.18106, 0.23158, 0.17124, 0.08482], "lda_w": [[-0.56158, 0.35141, 0.95144, -0.64636,
0.20678], [-0.91265, 0.8481, -0.42556, -0.24222, 0.07217], [0.71961, 0.33308, 0.1698, -0.35286, -0.0279],
[0.58685, -0.41101, -0.39422, -0.96849, -0.76306], [0.31165, 1.07346, 0.30631, 1.08819, -0.78214], [-0.021,
0.78379, 0.06926, 0.34192, -0.40221], [2.50552, -0.86349, 1.70819, 0.57926, 0.239], [-0.22908, -0.53523,
0.11798, -0.40574, -0.21287], [0.08955, 0.02263, -0.0347, -0.16555, -0.31667], [0.43131, -0.35857, 0.49717,
0.30799, 0.27389], [-0.10542, -0.46826, -0.35212, -0.32622, -0.44117], [-0.40394, 0.25281, -0.21604, 0.30459,
0.05286], [0.21659, -0.32278, 0.09787, 0.08473, -0.0362], [0.16634, 0.1121, 0.01647, 0.44959, 0.60401],
[0.01853, 0.21732, -0.16104, 0.43798, 0.46312], [0.11554, 0.26489, -0.27562, 0.00503, -0.18192], [0.28825,
0.04176, -0.46319, -0.2433, 0.16054], [-0.24558, -0.30761, -0.19527, 0.04395, -0.77132], [0.18271, 0.10834,
0.16178, 0.13069, -1.14645], [0.01269, 0.36738, 0.23889, -1.01301, 1.42758], [0.60825, 0.86171, -0.98009,
-0.35426, 1.00607], [-1.25794, -0.52753, 1.37122, 0.29861, 0.70895], [-0.33861, 0.1107, 0.57265, 0.53199,
-0.46351], [0.2376, 0.30054, -0.32665, 0.08224, 0.06558], [-0.5803, -0.07496, 1.1182, -0.04432, -1.02703],
[0.50091, 0.0658, -0.19715, 0.10577, 0.0217], [-0.09826, 0.2257, 0.07919, 0.10769, -0.40391], [-0.16687,
-0.07452, 0.00508, 0.85696, 0.12321]], "lda_classes": {"bedrock": [{"mean": [-4.61099, -0.32091, 1.71733,
3.49727, 0.68941], "std": [1.35642, 1.27218, 1.1277, 1.47206, 1.21738]}], "hard_ground": [{"mean": [0.60204,
4.56843, -2.02721, -1.41764, 1.62533], "std": [0.89176, 0.65875, 0.83562, 0.70194, 0.62479]}], "loose_soil":
[{"mean": [0.73304, 0.59123, -4.9536, -0.14222, -2.35192], "std": [0.9577, 1.13733, 0.63741, 0.74958,
0.91406]}, {"mean": [0.80953, 2.872, -4.10505, 1.30443, -1.31098], "std": [0.68298, 0.54021, 0.64085, 0.79496,
0.62791]}], "sand_dust": [{"mean": [-4.08413, 1.76676, 3.62321, -0.82362, 0.1214], "std": [1.02948, 1.01105,
0.85539, 0.73977, 0.97092]}, {"mean": [-4.28203, 2.86653, 5.15379, -2.26692, -1.58234], "std": [0.74476,
0.90659, 0.71604, 0.80557, 0.77136]}], "vegetation": [{"mean": [10.7475, -2.31573, 0.69721, 0.18639, -0.5649],
"std": [0.94647, 0.5821, 0.80891, 0.71434, 0.75836]}, {"mean": [12.379, -2.20813, 2.2937, 0.17841, 0.19466],
"std": [0.82765, 0.5, 0.55873, 0.636, 0.57985]}], "water_void": [{"mean": [-5.07774, -7.12744, -1.22062,
-1.69743, 0.09959], "std": [0.61537, 0.68683, 0.85677, 0.68418, 1.3288]}, {"mean": [-3.74293, -6.21157,
-3.02374, -1.11588, 0.30813], "std": [0.6955, 1.1757, 0.87781, 1.08676, 1.05196]}]}, "classes": {"bedrock":
[{"mean": [0.09732, 0.34433, -0.60173, -1.28734, 0.32135, -0.32176, -0.40918, -0.32283, -0.14971, -0.32596,
-0.35495, -0.29284, -0.32101, -0.24097, -0.24892, -0.26815, -0.08172, -0.43769, -0.94171, -0.97287, -0.82352,
0.95141, -0.4656, -0.56549, 0.10665, 0.08425, 0.49318, 1.33462], "std": [0.48801, 0.4022, 0.41059, 0.57998,
0.66509, 0.81106, 0.28498, 0.25, 0.25, 0.74744, 0.65189, 0.70327, 0.60558, 0.64546, 0.69012, 0.72716, 0.79914,
0.57842, 0.77002, 0.47892, 0.50392, 0.37854, 0.55405, 1.15572, 0.99407, 0.78465, 0.43994, 1.27606]}],
"hard_ground": [{"mean": [0.48012, 0.62425, 0.41422, -0.20731, 0.72237, 0.27408, -0.43277, -0.33922, -0.1515,
-0.34526, -0.49805, -0.28877, -0.19759, -0.14262, -0.18426, -0.32245, -0.07695, -0.50794, 0.68136, 1.0178,
0.81224, -0.72566, 0.54455, 0.59914, -0.68475, 0.01417, -0.5248, -0.55053], "std": [0.46807, 0.44811, 0.37092,
0.34148, 0.25, 0.33478, 0.25, 0.25, 0.25, 0.72839, 0.25, 0.64712, 0.28774, 0.25, 0.25, 0.27647, 0.58873,
0.4425, 0.25, 0.28411, 0.25, 0.25, 0.56788, 0.25, 0.26918, 0.65932, 0.66474, 0.25]}], "loose_soil": [{"mean":
[-0.75383, 0.69639, 0.02324, 0.49245, 0.74042, 0.05144, -0.44, 0.38518, -0.15395, 0.03077, 0.93617, 0.55463,
0.81937, 0.93613, 0.99765, 1.01014, 0.71731, 0.85038, 0.39394, 0.27665, 0.76518, -0.77793, 0.47251, 0.53599,
0.30478, 0.55824, -0.05253, 0.47651], "std": [0.43726, 0.30121, 0.3255, 0.44669, 0.25, 0.4246, 0.25, 0.9767,
0.25, 0.57087, 0.25, 0.77412, 0.48812, 0.27648, 0.25, 0.25, 0.57744, 0.32055, 0.41934, 0.58107, 0.29817,
0.37927, 0.97036, 0.25, 0.424, 0.30249, 0.91981, 1.015]}], "sand_dust": [{"mean": [1.54496, 0.39304, 0.35503,
-0.79726, 0.59887, 0.37474, -0.45971, -0.33922, 0.64337, -0.56936, -1.41456, -1.06383, -1.4732, -1.49378,
-1.35938, -1.25583, -0.64085, -1.30348, 0.19847, -0.00349, -0.57747, 0.56991, -0.57013, -1.0272, 0.53596,
-0.60302, 0.62473, -0.56733], "std": [0.62624, 0.36221, 0.48761, 0.4221, 0.33016, 0.46782, 0.25, 0.25,
2.31479, 0.83229, 0.38965, 0.87167, 0.46562, 0.35808, 0.44079, 0.4869, 0.86525, 0.6746, 0.7882, 0.78965,
0.65968, 0.59097, 0.70144, 1.24732, 1.36329, 1.004, 0.64135, 0.25]}], "vegetation": [{"mean": [-0.27186,
-2.01904, 1.38142, 1.25909, -0.75232, 1.2505, 2.19917, -0.19816, -0.15395, 1.58589, 0.8431, 0.69549, 1.08425,
1.02722, 0.91147, 0.79008, 0.35549, 0.78829, 0.7746, 0.86913, 1.06755, -1.19609, 1.01303, 0.50909, 0.27061,
0.4893, -0.63643, -0.15647], "std": [0.38707, 0.37355, 0.37544, 0.35462, 0.25, 0.25, 0.25, 0.33899, 0.25,
0.25, 0.25, 0.53403, 0.27217, 0.25, 0.25, 0.25, 0.51038, 0.28465, 0.25, 0.34265, 0.25, 0.25, 0.7656, 0.25,
0.38058, 0.3345, 0.7816, 0.58476]}], "water_void": [{"mean": [-1.09672, -0.03897, -1.57218, 0.54037, -1.63069,
-1.629, -0.45752, 0.81424, -0.03426, -0.37608, 0.4883, 0.39533, 0.08818, -0.08599, -0.11655, 0.04622,
-0.27328, 0.61045, -1.10665, -1.18722, -1.24398, 1.17835, -0.99436, -0.05154, -0.53324, -0.54294, 0.09586,
-0.5368], "std": [0.58111, 0.25, 0.4109, 0.76673, 0.78831, 0.53651, 0.25, 1.93475, 0.36246, 0.79493, 1.04098,
1.09267, 0.92422, 1.04105, 1.20015, 1.28177, 1.59164, 0.98231, 1.09343, 0.62248, 0.46329, 0.34553, 0.52363,
0.89769, 1.24025, 1.58791, 1.47322, 0.25]}]}, "n_samples": 1080, "ood_d2": 9.0}
"""
_PROTOTYPES: Optional[dict] = json.loads(_PROTOTYPES_JSON)
_LEARNED: Optional[dict] = None

LEARN_TEXTURE_SEEDS = tuple(range(9000, 9100))
LEARN_FLIGHT_SEEDS = tuple(range(10, 30))
LEARN_SIZES = ((720, 480), (480, 320))
LDA_REG = 0.1


def _texture_params(cls: str, seed: int) -> dict:
    """Random but reproducible viewing conditions for one training texture."""
    rng = np.random.default_rng([int(seed), CLASSES.index(cls), 4242])
    native = seed % 4 == 0
    w, h = LEARN_SIZES[int(rng.integers(len(LEARN_SIZES)))] if not native else (720, 480)
    return {
        "h": h, "w": w,
        "zoom": 1.0 if native else float(rng.choice([1.0, 1.5, 2.0, 2.5, 3.0, 4.0])),
        "blur": 0.0 if native else float(rng.choice([0.0, 0.6, 1.0, 1.5, 2.0])),
        "gain": 1.0 if native else float(rng.uniform(0.7, 1.35)),
        "gamma": 1.0 if native else float(rng.uniform(0.75, 1.3)),
        "noise": float(rng.choice([0.0, 0.02, 0.03, 0.05])),
        "analog": bool(rng.random() < 0.35),
    }


def _texture_sample(args) -> tuple:
    from grappstar.sim.terrain_textures import texture_view
    cls, seed = args
    p = _texture_params(cls, seed)
    img = texture_view(cls, p.pop("h"), p.pop("w"), seed=int(seed), **p)
    return cls, [_vec(extract_features(central_roi(img)))]


def _flight_sample(args) -> tuple:
    """Feature vectors of the frames analyze_landing would use in one simulated flight."""
    from grappstar.sim.flight_video import render_flight
    from grappstar.vision import flight
    cls, seed, size, n = args
    frames, _truth = render_flight(seed=int(seed), landing_class=cls, size=tuple(size), dropouts=True)
    res = flight.analyze_flight(frames, None)
    land = flight.landing_frames(frames, res, n=15)
    land.sort(key=lambda fr: _sharpness(fr.img), reverse=True)
    return cls, [_vec(extract_features(central_roi(fr.img))) for fr in land[:n]]


def learn_prototypes(texture_seeds: Sequence[int] = LEARN_TEXTURE_SEEDS,
                     flight_seeds: Sequence[int] = LEARN_FLIGHT_SEEDS,
                     sizes: Sequence[tuple] = LEARN_SIZES, frames_per_flight: int = 2,
                     jobs: int = 1, verbose: bool = False) -> dict:
    """Learn the classifier constants (see the comment above _PROTOTYPES_JSON). Deterministic."""
    tasks_t = [(c, int(s)) for c in CLASSES for s in texture_seeds]
    tasks_f = [(c, int(s), tuple(sz), int(frames_per_flight)) for c in CLASSES for s in flight_seeds
               for sz in sizes]
    if jobs > 1:
        import multiprocessing as mp
        with mp.get_context("fork").Pool(jobs) as pool:
            parts = pool.map(_texture_sample, tasks_t) + pool.map(_flight_sample, tasks_f)
    else:
        parts = [_texture_sample(t) for t in tasks_t] + [_flight_sample(t) for t in tasks_f]
    rows: Dict[str, List[np.ndarray]] = {c: [] for c in CLASSES}
    for c, vs in parts:
        rows[c].extend(vs)
    if verbose:
        print("samples per class:", {c: len(v) for c, v in rows.items()})
    X = np.concatenate([np.stack(rows[c]) for c in CLASSES])
    y = np.concatenate([np.full(len(rows[c]), i) for i, c in enumerate(CLASSES)])
    mu = X.mean(0)
    sd = X.std(0) + 1e-9
    Z = (X - mu) / sd
    W = _lda(Z, y, LDA_REG)
    P = Z @ W
    protos, lprotos = {}, {}
    for k, c in enumerate(CLASSES):
        zk = Z[y == k]
        protos[c] = [{"mean": zk.mean(0).tolist(), "std": np.maximum(zk.std(0), 0.25).tolist()}]
        # up to two prototypes per class in LDA space (rippled vs flat sand, cracked vs smooth rock, ...)
        lprotos[c] = [{"mean": m.tolist(), "std": np.maximum(s_, 0.5).tolist()}
                      for m, s_ in _two_means(P[y == k])]
    out = {"version": FEATURE_VERSION, "features": list(FEATURE_NAMES), "mu": mu.tolist(), "sd": sd.tolist(),
           "lda_w": W.tolist(), "lda_classes": lprotos, "classes": protos, "n_samples": int(len(X))}
    # out of distribution threshold from the training data itself
    dz = np.array([_ood_d2(z, out) for z in Z])
    out["ood_d2"] = float(max(9.0, 2.0 * np.quantile(dz, 0.995)))
    return _rounded(out)


def _two_means(z: np.ndarray, iters: int = 10) -> List[tuple]:
    """Split one class into up to two clusters, returned as [(mean, std), ...].

    Deterministic 2-means; both clusters are kept only if each holds at least 20 percent of the
    samples, otherwise the class gets a single prototype.
    """
    one = [(z.mean(0), z.std(0))]
    if len(z) < 10:
        return one
    a = z[np.argmax(((z - z.mean(0)) ** 2).sum(1))]
    b = z[np.argmax(((z - a) ** 2).sum(1))]
    cents = np.stack([a, b])
    lab = np.zeros(len(z), int)
    for _ in range(iters):
        lab = np.argmin(((z[:, None, :] - cents[None]) ** 2).sum(2), 1)
        if lab.min() == lab.max():
            return one
        cents = np.stack([z[lab == k].mean(0) for k in range(2)])
    if np.bincount(lab, minlength=2).min() < 0.2 * len(z):
        return one
    return [(z[lab == k].mean(0), z[lab == k].std(0)) for k in range(2)]


def _lda(Z: np.ndarray, y: np.ndarray, reg: float) -> np.ndarray:
    """Regularised LDA projection (n_features x n_classes-1), unit within class spread per axis."""
    K = int(y.max()) + 1
    m = Z.mean(0)
    Sw = np.zeros((Z.shape[1], Z.shape[1]))
    Sb = np.zeros_like(Sw)
    for k in range(K):
        zk = Z[y == k]
        mk = zk.mean(0)
        Sw += (zk - mk).T @ (zk - mk) / len(Z)
        Sb += np.outer(mk - m, mk - m) / K
    Sw = (1 - reg) * Sw + reg * np.eye(len(Sw)) * np.trace(Sw) / len(Sw)
    ev, evec = np.linalg.eig(np.linalg.solve(Sw, Sb))
    order = np.argsort(-ev.real)[:K - 1]
    W = evec[:, order].real
    W = W / np.linalg.norm(W, axis=0)
    P = Z @ W
    spread = np.concatenate([P[y == k] - P[y == k].mean(0) for k in range(K)]).std(0)
    W = W / spread
    # fixed sign convention so --learn output is reproducible across LAPACK builds
    W = W * np.where(W[np.abs(W).argmax(0), np.arange(W.shape[1])] < 0, -1.0, 1.0)
    return W


def _rounded(x, nd: int = 5):
    if isinstance(x, dict):
        return {k: _rounded(v, nd) for k, v in x.items()}
    if isinstance(x, list):
        return [_rounded(v, nd) for v in x]
    if isinstance(x, float):
        return round(x, nd)
    return x


def _ood_d2(z: np.ndarray, p: dict) -> float:
    best = float("inf")
    for c in CLASSES:
        for pr in p["classes"][c]:
            d = float((((z - np.asarray(pr["mean"])) / np.asarray(pr["std"])) ** 2).mean())
            best = min(best, d)
    return best


def _prototypes() -> dict:
    global _LEARNED
    if _LEARNED is None:
        p = _PROTOTYPES
        if not p or p.get("version") != FEATURE_VERSION or p.get("features") != list(FEATURE_NAMES):
            p = learn_prototypes(flight_seeds=())   # fast fallback (textures only, a few seconds)
        means, stds, lmeans, lstds, lowner = [], [], [], [], []
        for i, c in enumerate(CLASSES):
            for pr in p["classes"][c]:
                means.append(pr["mean"])
                stds.append(pr["std"])
            for pr in p["lda_classes"][c]:
                lmeans.append(pr["mean"])
                lstds.append(pr["std"])
                lowner.append(i)
        _LEARNED = {"mu": np.array(p["mu"]), "sd": np.array(p["sd"]), "mean": np.array(means),
                    "std": np.array(stds), "W": np.array(p["lda_w"]), "lmean": np.array(lmeans),
                    "lstd": np.array(lstds), "lowner": np.array(lowner),
                    "ood_d2": float(p.get("ood_d2", OOD_DIST))}
    return _LEARNED


TEMPERATURE = 1.5       # softmax temperature on the squared LDA distance (higher = softer)
OOD_DIST = 9.0          # default mean squared z distance to the nearest class for "unknown"
MIN_CONFIDENCE = 0.45   # top probability below this = unknown


def classify_features(f: Dict[str, float]) -> Dict[str, object]:
    """Prototype classifier. Returns probs (over CLASSES), OOD distance and rule notes."""
    P = _prototypes()
    z = (_vec(f) - P["mu"]) / P["sd"]
    q = z @ P["W"]
    dp = (((q[None, :] - P["lmean"]) / P["lstd"]) ** 2).sum(1)
    d2 = np.array([dp[P["lowner"] == i].min() for i in range(len(CLASSES))])
    probs = _softmax(-0.5 * d2 / TEMPERATURE)
    pr = {c: float(probs[i]) for i, c in enumerate(CLASSES)}
    dz = (((z[None, :] - P["mean"]) / P["std"]) ** 2).mean(1)
    notes: List[str] = []
    # Hand rules (transparent overrides on top of the prototypes).
    if f["green_frac"] > 0.25 and f["edge_max"] > 0.04:
        target = min(0.9, 0.4 + f["green_frac"])
        if pr["vegetation"] < target:
            pr = _push(pr, "vegetation", target)
            notes.append("%d percent of the patch is green (plants)." % round(100 * f["green_frac"]))
    if f["dark_frac"] > 0.7:
        if pr["water_void"] < 0.85:
            pr = _push(pr, "water_void", 0.85)
        notes.append("The patch is very dark: water, a hole or deep shadow.")
    elif f["spec_frac"] > 0.01 and f["v_mean"] < 0.4 and f["log_cv"] < -0.6:
        if pr["water_void"] < 0.7:
            pr = _push(pr, "water_void", 0.7)
        notes.append("Bright glints on a dark smooth surface look like water.")
    return {"probs": pr, "min_d2": float(dz.min()), "ood_d2": P["ood_d2"], "notes": notes}


def _push(pr: Dict[str, float], cls: str, target: float) -> Dict[str, float]:
    rest = sum(v for k, v in pr.items() if k != cls)
    out = {k: (v / rest * (1 - target) if rest > 0 else (1 - target) / (len(pr) - 1))
           for k, v in pr.items() if k != cls}
    out[cls] = target
    return out


# ============================================================ slope

def slope_from_imu(roll_deg: float, pitch_deg: float) -> float:
    """Tilt of the dart body from upright (deg) from roll and pitch."""
    c = math.cos(math.radians(roll_deg)) * math.cos(math.radians(pitch_deg))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def slope_from_attitude(att) -> Optional[float]:
    """Dart tilt from upright (deg) from a flight attitude sample.

    Accepts (t, roll, pitch, yaw), (roll, pitch) or a dict with roll/pitch. If |pitch| is
    above 45 degrees the pitch is taken as measured from the horizontal (nose down = -90),
    so the tilt is 90 - |pitch| (roll about a vertical body axis does not tilt it).
    Otherwise roll and pitch are taken as already relative to the upright landed pose.
    """
    try:
        if isinstance(att, dict):
            roll, pitch = float(att["roll"]), float(att["pitch"])
        elif len(att) >= 4:
            roll, pitch = float(att[1]), float(att[2])
        else:
            roll, pitch = float(att[0]), float(att[1])
    except (TypeError, ValueError, KeyError, IndexError):
        return None
    if not (math.isfinite(roll) and math.isfinite(pitch)):
        return None
    if abs(pitch) > 45.0:
        return max(0.0, 90.0 - abs(pitch))
    return slope_from_imu(roll, pitch)


def slope_from_texture(roi: np.ndarray, half_angle_deg: float = 20.0) -> Optional[float]:
    """Very rough slope from the texture gradient (finer texture = farther away).

    Measures a texture frequency proxy (mean |Laplacian| / mean |gradient|) in the four
    half images. The ratio between opposite halves approximates their depth ratio, which
    for a plane tilted by a gives tan(a) = (r - 1) / ((r + 1) tan(b)), b = angle from the
    optical axis to the centre of a half. Returns None when the texture is too weak.
    Approximate: expect several degrees of noise and bias from uneven texture.
    """
    g = cv2.GaussianBlur(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY).astype(np.float32), (0, 0), 1.0)
    lap = np.abs(cv2.Laplacian(g, cv2.CV_32F))
    gm = np.abs(cv2.Sobel(g, cv2.CV_32F, 1, 0)) + np.abs(cv2.Sobel(g, cv2.CV_32F, 0, 1))
    if float(gm.mean()) < 4.0:
        return None
    h, w = g.shape

    def freq(sl):
        return float(lap[sl].mean()) / max(1e-6, float(gm[sl].mean()))

    top, bot = freq(np.s_[:h // 2, :]), freq(np.s_[h // 2:, :])
    lef, rig = freq(np.s_[:, :w // 2]), freq(np.s_[:, w // 2:])
    tb = math.tan(math.radians(half_angle_deg))
    rv = max(top, bot) / max(1e-6, min(top, bot))
    rh = max(lef, rig) / max(1e-6, min(lef, rig))
    tv = (rv - 1) / ((rv + 1) * tb)
    th = (rh - 1) / ((rh + 1) * tb)
    return float(min(45.0, math.degrees(math.atan(math.hypot(tv, th)))))


# ============================================================ report

@dataclass
class TerrainReport:
    cls: str
    probs: Dict[str, float]
    slope_deg: Optional[float]
    score: int
    verdict: str
    reasons: List[str]
    image: Optional[np.ndarray] = None
    confidence: float = 0.0
    slope_source: str = "none"
    quality: Optional[float] = None
    used_frames: List[int] = field(default_factory=list)
    classical_probs: Dict[str, float] = field(default_factory=dict)
    model_probs: Optional[Dict[str, float]] = None
    features: Dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> dict:
        def r(x, n=4):
            return None if x is None else round(float(x), n)
        return {
            "cls": self.cls,
            "probs": {k: r(v) for k, v in self.probs.items()},
            "slope_deg": r(self.slope_deg, 1),
            "slope_source": self.slope_source,
            "score": int(self.score),
            "verdict": self.verdict,
            "reasons": [str(s) for s in self.reasons],
            "confidence": r(self.confidence, 3),
            "quality": r(self.quality, 3),
            "used_frames": [int(i) for i in self.used_frames],
            "classical_probs": {k: r(v) for k, v in self.classical_probs.items()},
            "model_probs": None if self.model_probs is None else {k: r(v) for k, v in self.model_probs.items()},
            "features": {k: r(v) for k, v in self.features.items()},
            "image_shape": None if self.image is None else list(self.image.shape),
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict())


def verdict_for(score: float) -> str:
    if score >= GO_MIN:
        return "GO"
    if score >= UNSURE_MIN:
        return "UNSURE"
    return "NO_GO"


def _as_items(frames) -> List[tuple]:
    """Normalise input to a list of (img, idx, quality, bad)."""
    if isinstance(frames, np.ndarray):
        frames = [frames]
    from ..video.frame import ensure_bgr
    items = []
    for i, fr in enumerate(frames or []):
        img = fr if isinstance(fr, np.ndarray) else getattr(fr, "img", None)
        if not isinstance(img, np.ndarray) or img.size == 0 or img.ndim not in (2, 3):
            continue
        try:
            img = ensure_bgr(img)          # grey, HxWx1 and BGRA inputs become BGR
        except Exception:
            continue
        if isinstance(fr, np.ndarray):
            items.append((img, i, None, False))
        else:
            items.append((img, int(getattr(fr, "idx", i)), getattr(fr, "quality", None),
                          bool(getattr(fr, "bad", False))))
    return items


def _raw_autocorr(img: np.ndarray) -> float:
    """Lag-1 autocorrelation of the full resolution central crop (no resampling).

    Natural textures are strongly correlated between neighbouring pixels; pure snow is not.
    """
    h, w = img.shape[:2]
    s = max(8, int(min(h, w) * ROI_FRAC))
    crop = img[(h - s) // 2:(h - s) // 2 + s, (w - s) // 2:(w - s) // 2 + s]
    g = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY) if crop.ndim == 3 else crop
    return _autocorr1(g)


def _sharpness(img: np.ndarray) -> float:
    roi = central_roi(img, size=160)
    g = cv2.GaussianBlur(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), (3, 3), 0)
    return float(cv2.Laplacian(g, cv2.CV_32F).var())


DARK_MEDIAN = 30       # grey level (0..255) of the central region below which a frame counts as black
DARK_P90 = 60
DARK_SHARE = 0.6       # share of the landing frames that must be black for the dark landing verdict
UNIFORM_STD = 0.6      # grey std below which a black frame is a flat screen (no sensor noise at all)


def _dark_stats(img: np.ndarray) -> tuple:
    """(median grey, 90th percentile grey, grey std, mean B - mean R) of the central region."""
    h, w = img.shape[:2]
    s = max(8, int(min(h, w) * ROI_FRAC))
    crop = img[(h - s) // 2:(h - s) // 2 + s:2, (w - s) // 2:(w - s) // 2 + s:2]
    if crop.ndim == 2:
        g, blue = crop, 0.0
    else:
        g = cv2.cvtColor(np.ascontiguousarray(crop), cv2.COLOR_BGR2GRAY)
        blue = float(crop[..., 0].mean()) - float(crop[..., 2].mean())
    p50, p90 = np.percentile(g, (50, 90))
    return float(p50), float(p90), float(g.std()), blue


def _dark_landing(items: List[tuple]) -> Optional[dict]:
    """Detect a black landing picture (most frames black), whatever the frame quality flags say.

    A black void (hole, crevasse, deep shadow, dark water at low light) or a camera buried in the
    ground looks like a near uniform black frame, which video.quality marks as a dropout. Dropping
    those frames would hide the most dangerous case, so it is checked first, on all frames.
    """
    if not items:
        return None
    stats = [_dark_stats(it[0]) for it in items]
    dark = [st for st in stats if st[0] < DARK_MEDIAN and st[1] < DARK_P90 and st[3] < 40.0]
    if len(dark) < max(1, math.ceil(DARK_SHARE * len(items))):
        return None
    return {"n_dark": len(dark), "n": len(items),
            "uniform": float(np.median([st[2] for st in dark])) < UNIFORM_STD,
            "level": float(np.median([st[0] for st in dark]))}


def _dark_report(dark: dict, items: List[tuple]) -> "TerrainReport":
    """NO_GO report for a black landing picture (see the module docstring, Dark landings)."""
    n_dark, n = dark["n_dark"], dark["n"]
    used = [it[1] for it in items]
    if dark["uniform"]:
        probs = {c: 0.0 for c in CLASSES} | {"water_void": 0.4, "unknown": 0.6}
        reasons = ["%d of %d landing frames are a flat black screen (no texture, no sensor noise)." % (n_dark, n),
                   "Either the video link is down or the camera faces a black void or is buried.",
                   "Ground cannot be confirmed: treat as NO_GO until the dart camera shows ground.",
                   "Check the dart camera picture yourself before winching."]
        return TerrainReport("unknown", probs, None, 30, "NO_GO", reasons, None, 0.0, "none", None, used,
                             {c: 0.0 for c in CLASSES}, None, {"dark_level": dark["level"]})
    probs = {c: 0.0 for c in CLASSES} | {"water_void": 0.8, "unknown": 0.2}
    reasons = ["%d of %d landing frames are almost black (grey level about %d of 255)." % (n_dark, n,
                                                                                            round(dark["level"])),
               "Looks like a void: a hole, crevasse, deep shadow or dark water, or the nose is buried.",
               "Nothing visible to anchor in: do not rely on this anchor, retrieve and relaunch.",
               "If the camera may be covered, check the dart camera picture yourself."]
    return TerrainReport("water_void", probs, None, 5, "NO_GO", reasons, None, 0.6, "none", None, used,
                         {c: (1.0 if c == "water_void" else 0.0) for c in CLASSES}, None,
                         {"dark_level": dark["level"]})


def _drop_link_artefacts(good: List[tuple], min_quality: float) -> List[tuple]:
    """Drop frames without a quality value that look like link artefacts (blue or black screen,
    snow, torn), using the same cheap check as vision.flight.landing_frames. If every frame looks
    bad the list is returned unchanged and the noise and blank checks decide later."""
    if all(it[2] is not None for it in good):
        return good
    try:
        from grappstar.vision.flight import quick_quality
    except Exception:  # pragma: no cover - flight ships with the package
        return good
    keep = [it for it in good if it[2] is not None or quick_quality(it[0]) >= min_quality]
    return keep or good


def analyze_landing(frames, model: Optional[TerrainModel] = None, model_weight: float = 0.5,
                    imu_roll_deg: Optional[float] = None, imu_pitch_deg: Optional[float] = None,
                    n_best: int = 3, min_quality: float = 0.4, make_image: bool = True,
                    attitude=None) -> TerrainReport:
    """Classify the landing patch and score it as an anchor site.

    frames: list of video.frame.Frame objects or plain BGR images (or one image).
    model: optional TerrainModel; its probabilities are fused with the classical ones as
    model_weight * model + (1 - model_weight) * classical.
    imu_roll_deg / imu_pitch_deg: dart tilt from upright, if known (better slope cue).
    attitude: alternative to the above, a vision.flight attitude sample (t, roll, pitch, yaw)
    or (roll, pitch) in degrees, see slope_from_attitude().
    """
    if attitude is not None and imu_roll_deg is None:
        tilt = slope_from_attitude(attitude)
        if tilt is not None:
            imu_roll_deg, imu_pitch_deg = tilt, 0.0
    items = [it for it in _as_items(frames) if it[0] is not None and it[0].size > 0]
    reasons: List[str] = []
    dark = _dark_landing(items)
    if dark is not None:
        rep = _dark_report(dark, items)
        if make_image:
            rep.image = render_report(rep, items[0][0] if items else None)
        return rep
    good = [it for it in items if not it[3] and (it[2] is None or it[2] >= min_quality)]
    good = _drop_link_artefacts(good, min_quality)
    if not good:
        reasons.append("No usable landing frames (all missing, bad or too noisy).")
        reasons.append("Look at the dart camera yourself before winching.")
        rep = TerrainReport("unknown", {c: 0.0 for c in CLASSES} | {"unknown": 1.0}, None,
                            int(BASE_SCORE["unknown"]), "UNSURE", reasons, None, 0.0,
                            used_frames=[])
        if make_image:
            rep.image = render_report(rep, items[0][0] if items else None)
        return rep

    good.sort(key=lambda it: _sharpness(it[0]), reverse=True)
    sel = good[:max(1, n_best)]
    rois = [central_roi(it[0]) for it in sel]
    qs = [it[2] for it in sel if it[2] is not None]
    quality = float(np.mean(qs)) if qs else None

    feats = [extract_features(r) for r in rois]
    results = [classify_features(f) for f in feats]
    cprobs = {c: float(np.mean([r["probs"][c] for r in results])) for c in CLASSES}
    min_d2 = float(np.median([r["min_d2"] for r in results]))
    ood_d2 = float(results[0]["ood_d2"])
    fmean = {k: float(np.mean([f[k] for f in feats])) for k in FEATURE_NAMES}
    notes: List[str] = []
    for r in results:
        for n in r["notes"]:
            if n not in notes:
                notes.append(n)

    mprobs = None
    probs = dict(cprobs)
    if model is not None:
        try:
            outs = [model.predict(r) for r in rois]
            mprobs = {c: float(np.mean([o.get(c, 0.0) for o in outs])) for c in CLASSES}
            tot = sum(mprobs.values())
            if tot > 0:
                mprobs = {c: v / tot for c, v in mprobs.items()}
                w = float(min(1.0, max(0.0, model_weight)))
                probs = {c: w * mprobs[c] + (1 - w) * cprobs[c] for c in CLASSES}
            else:
                mprobs = None
                reasons.append("The learned model gave no usable output; classical result only.")
        except Exception as e:  # never let a model failure kill the verdict
            mprobs = None
            reasons.append("The learned model failed (%s); classical result only." % type(e).__name__)

    top = max(probs, key=probs.get)
    conf = probs[top]
    raw_ac = float(np.median([_raw_autocorr(it[0]) for it in sel]))
    grey_std = float(np.mean([cv2.cvtColor(r, cv2.COLOR_BGR2GRAY).std() for r in rois])) / 255.0
    noise_like = raw_ac < 0.15 and grey_std > 0.05
    ood = min_d2 > ood_d2 and (mprobs is None or max(mprobs.values()) < 0.6)
    blank = grey_std < 0.006 and fmean["dark_frac"] < 0.7
    if blank and top == "water_void" and conf >= 0.6 and not ood:
        # featureless but coloured like water and close to the water prototypes: still water is the
        # one natural surface without texture, and water_void is the safe (NO_GO) reading anyway
        blank = False
        notes.append("Smooth and featureless with the colour of water.")
    q_factor = 1.0 if quality is None else float(min(1.0, max(0.0, (quality - 0.3) / 0.4)))

    cls = top
    if noise_like:
        cls = "unknown"
        reasons.append("The picture looks like pure noise (no structure between neighbouring pixels).")
    elif blank:
        cls = "unknown"
        reasons.append("The picture is blank (no texture at all): lens covered, dropout or out of focus.")
    elif ood:
        cls = "unknown"
        reasons.append("The patch does not look like any known terrain type.")
    elif conf < MIN_CONFIDENCE:
        cls = "unknown"
        reasons.append("Terrain type unclear: best guess %s at %d percent." % (_nice(top), round(100 * conf)))

    out_probs = dict(probs)
    if cls == "unknown":
        out_probs = {c: v * 0.5 for c, v in probs.items()}
        out_probs["unknown"] = 0.5
    else:
        out_probs["unknown"] = 0.0
    confidence = (conf if cls != "unknown" else 0.0) * (q_factor if quality is not None else 1.0)

    # ---- score
    roi0 = rois[0]
    cracks = crack_fraction(roi0)
    base = dict(BASE_SCORE)
    has_cracks = cracks > 0.004
    if has_cracks:
        base["bedrock"] = BASE_SCORE["bedrock"] + BEDROCK_CRACK_BONUS
    if cls == "unknown":
        score = BASE_SCORE["unknown"]
    else:
        score = sum(probs[c] * base[c] for c in CLASSES)
        reasons.extend(_class_reasons(cls, conf, has_cracks))
        second = sorted(probs, key=probs.get, reverse=True)[1]
        if probs[second] > 0.2:
            reasons.append("Could also be %s (%d percent)." % (_nice(second), round(100 * probs[second])))
    reasons.extend(n for n in notes if n not in reasons)
    if model is not None and mprobs is not None:
        mt = max(mprobs, key=mprobs.get)
        reasons.append("Learned model says %s (%d percent), weight %d percent."
                       % (_nice(mt), round(100 * mprobs[mt]), round(100 * model_weight)))

    # ---- slope
    slope, slope_src = None, "none"
    if imu_roll_deg is not None and imu_pitch_deg is not None:
        slope, slope_src = slope_from_imu(imu_roll_deg, imu_pitch_deg), "imu"
    else:
        sl = [s for s in (slope_from_texture(r) for r in rois) if s is not None]
        if sl:
            slope, slope_src = float(np.median(sl)), "texture"
    if slope is not None and cls != "unknown":
        if slope_src == "imu":
            pen = 0.0 if slope <= 10 else (slope - 10) * 1.0 if slope <= 25 else 15 + (slope - 25) * 2.0
            if pen > 0:
                reasons.append("Dart is tilted %d degrees: slope or a lean, augers may not seat straight."
                               % round(slope))
        else:
            pen = 0.0 if slope <= 25 else min(10.0, (slope - 25) * 0.5)
            if pen > 0:
                reasons.append("Ground may slope about %d degrees (rough estimate from texture)." % round(slope))
        score -= pen

    # ---- quality and confidence
    if quality is not None and quality < 0.6:
        reasons.append("Picture quality is poor (%d percent): lower confidence." % round(100 * quality))
    shrink = q_factor * (min(1.0, conf / 0.7) if cls != "unknown" else 1.0)
    score = 50.0 + (score - 50.0) * shrink
    if quality is not None and quality < 0.5:
        score = min(max(score, UNSURE_MIN), GO_MIN - 1)
    score = int(round(min(100.0, max(0.0, score))))
    verdict = verdict_for(score)
    if cls == "unknown":
        reasons.append("Check the dart camera picture yourself before winching.")
    elif verdict == "GO":
        reasons.append("Anchor site looks good: drive the augers, then winch gently.")
    elif verdict == "UNSURE":
        reasons.append("Not sure: look at the picture and test the anchor with a gentle pull.")
    else:
        reasons.append("Do not rely on this anchor: retrieve and relaunch.")

    rep = TerrainReport(cls, out_probs, slope, score, verdict, reasons, None, confidence, slope_src,
                        quality, [it[1] for it in sel], cprobs, mprobs, fmean)
    if make_image:
        rep.image = render_report(rep, sel[0][0])
    return rep


_NICE = {"sand_dust": "sand or dust", "water_void": "water or void"}


def _nice(c: str) -> str:
    return _NICE.get(c, c.replace("_", " "))


def _class_reasons(cls: str, conf: float, cracks: bool) -> List[str]:
    p = round(100 * conf)
    if cls == "hard_ground":
        return ["Looks like firm compact earth (%d percent): the best ground for the augers." % p]
    if cls == "loose_soil":
        return ["Looks like loose soil (%d percent): augers go in easily but hold less." % p]
    if cls == "vegetation":
        return ["Looks like vegetation (%d percent): roots help, but plants may hide rock or water." % p]
    if cls == "sand_dust":
        return ["Looks like sand or dust (%d percent): augers will likely pull out (dust trap)." % p]
    if cls == "bedrock":
        out = ["Looks like bedrock (%d percent): the 16 to 17 mm augers cannot screw into solid rock." % p]
        if cracks:
            out.append("Cracks are visible: an auger might wedge in one, but do not count on it.")
        else:
            out.append("No cracks seen for the augers to bite into.")
        return out
    if cls == "water_void":
        return ["Looks like water or a void (%d percent): nothing to anchor in." % p]
    return []


def _wrap(text: str, width: int) -> List[str]:
    words, lines, cur = text.split(), [], ""
    for wd in words:
        if len(cur) + len(wd) + 1 > width and cur:
            lines.append(cur)
            cur = wd
        else:
            cur = (cur + " " + wd).strip()
    if cur:
        lines.append(cur)
    return lines


def render_report(rep: TerrainReport, img: Optional[np.ndarray], size=(960, 540)) -> np.ndarray:
    """960x540 composite: landing frame (with ROI box), verdict, score, class bars, reasons."""
    W, H = size
    out = np.full((H, W, 3), 32, np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    # Left: landing frame fitted in 560x420.
    lw, lh = 560, 420
    if img is not None and img.size > 0:
        h, w = img.shape[:2]
        s = min(lw / w, lh / h)
        nw, nh = max(1, int(w * s)), max(1, int(h * s))
        small = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
        x0, y0 = 10 + (lw - nw) // 2, 10 + (lh - nh) // 2
        out[y0:y0 + nh, x0:x0 + nw] = small
        side = int(min(nw, nh) * ROI_FRAC)
        cx, cy = x0 + nw // 2, y0 + nh // 2
        cv2.rectangle(out, (cx - side // 2, cy - side // 2), (cx + side // 2, cy + side // 2),
                      (255, 255, 255), 1)
    else:
        cv2.putText(out, "no usable frame", (180, 220), font, 0.8, (200, 200, 200), 2, cv2.LINE_AA)
    # Verdict banner.
    col = VERDICT_COLOURS.get(rep.verdict, (128, 128, 128))
    cv2.rectangle(out, (580, 10), (W - 10, 90), col, -1)
    cv2.putText(out, rep.verdict.replace("_", " "), (595, 68), font, 1.6, (255, 255, 255), 3, cv2.LINE_AA)
    cv2.putText(out, "%d/100" % rep.score, (800, 68), font, 1.1, (255, 255, 255), 2, cv2.LINE_AA)
    info = "class: %s  conf %d%%" % (_nice(rep.cls), round(100 * rep.confidence))
    cv2.putText(out, info, (585, 115), font, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
    sl = "slope: n/a" if rep.slope_deg is None else "slope: %.0f deg (%s)" % (rep.slope_deg, rep.slope_source)
    q = "" if rep.quality is None else "  quality %d%%" % round(100 * rep.quality)
    cv2.putText(out, sl + q, (585, 137), font, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    # Class bars.
    y = 155
    for c in ALL_CLASSES:
        p = float(rep.probs.get(c, 0.0))
        cv2.putText(out, _nice(c), (585, y + 13), font, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.rectangle(out, (700, y + 2), (900, y + 16), (70, 70, 70), -1)
        cv2.rectangle(out, (700, y + 2), (700 + int(200 * p), y + 16), CLASS_COLOURS[c], -1)
        cv2.putText(out, "%d" % round(100 * p), (908, y + 14), font, 0.42, (220, 220, 220), 1, cv2.LINE_AA)
        y += 22
    # Reasons along the bottom and the lower right.
    lines: List[str] = []
    for r in rep.reasons:
        wrapped = _wrap(r, 88)
        lines.append("- " + wrapped[0])
        lines.extend("  " + x for x in wrapped[1:])
    y = 448
    for ln in lines[:5]:
        cv2.putText(out, ln, (12, y), font, 0.45, (235, 235, 235), 1, cv2.LINE_AA)
        y += 19
    y = 330
    for ln in lines[5:]:
        for sub in _wrap(ln, 44):
            if y > 425:
                break
            cv2.putText(out, sub, (585, y), font, 0.42, (210, 210, 210), 1, cv2.LINE_AA)
            y += 17
    return out


def _format_prototypes(p: dict, width: int = 110) -> str:
    """JSON of the learned constants wrapped to lines of about `width` characters."""
    text = json.dumps(p, separators=(", ", ": "))
    lines, cur = [], ""
    for tok in text.split(" "):
        if cur and len(cur) + 1 + len(tok) > width:
            lines.append(cur)
            cur = tok
        else:
            cur = (cur + " " + tok) if cur else tok
    lines.append(cur)
    return "\n".join(lines)


def write_prototypes(p: dict, path: Optional[str] = None) -> str:
    """Replace the _PROTOTYPES_JSON block of this source file with `p`. Returns the path."""
    import os
    path = path or os.path.abspath(__file__)
    with open(path) as fh:
        src = fh.read()
    head = '_PROTOTYPES_JSON = r"""\n'
    a = src.index(head) + len(head)
    b = src.index('"""', a)
    src = src[:a] + _format_prototypes(p) + "\n" + src[b:]
    with open(path, "w") as fh:
        fh.write(src)
    return path


if __name__ == "__main__":
    import argparse
    import sys
    ap = argparse.ArgumentParser(description="Landing zone verdict from dart camera images")
    ap.add_argument("images", nargs="*", help="landing images (BGR files)")
    ap.add_argument("--out", default="landing.png", help="report image path")
    ap.add_argument("--learn", action="store_true",
                    help="learn the classifier constants (textures plus simulated flights, a few minutes)")
    ap.add_argument("--write", action="store_true", help="with --learn: store them in this source file")
    ap.add_argument("--jobs", type=int, default=4, help="with --learn: worker processes")
    ap.add_argument("--textures-only", action="store_true", help="with --learn: no simulated flights")
    a = ap.parse_args()
    if a.learn:
        protos = learn_prototypes(flight_seeds=() if a.textures_only else LEARN_FLIGHT_SEEDS,
                                  jobs=a.jobs, verbose=True)
        if a.write:
            print("written to", write_prototypes(protos))
        else:
            print(json.dumps(protos))
        sys.exit(0)
    imgs = [cv2.imread(p) for p in a.images]
    rep = analyze_landing([i for i in imgs if i is not None])
    if rep.image is not None:
        cv2.imwrite(a.out, rep.image)
    print(json.dumps(rep.to_dict(), indent=1))
