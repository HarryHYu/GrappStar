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
1. Frame selection: drop frames marked bad or with quality below `min_quality`, then keep
   the `n_best` sharpest (Laplacian variance of the central region).
2. Central ROI: the central square (60 percent of the short side) is resized to 256x256.
   The edges are skipped because the dart body, legs and lens vignetting live there.
3. Classical features (numpy and OpenCV only, see FEATURE_NAMES): brightness, contrast,
   Laplacian variance, Canny edge density, uniform local binary pattern histogram, a small
   GLCM-like contrast at two distances, gradient orientation entropy, Lab and HSV colour
   moments, green fraction, dark and specular fractions, fine/coarse texture energy and
   lag-1 autocorrelation.
4. Classifier: per-class diagonal Gaussian prototypes (a nearest centroid classifier with
   per-feature spread) in z-scored feature space. Prototypes are learned from the
   procedural textures in grappstar.sim.terrain_textures (stored in _PROTOTYPES below,
   regenerated with `python -m grappstar.vision.terrain --learn`). Some classes get two
   prototypes (water vs void). Probabilities are a softmax over negative squared
   distances. Two hand rules sit on top: lots of green with edges pushes towards
   vegetation, a very dark and smooth patch pushes towards water_void.
5. "unknown" when frames are missing or poor, the image looks like pure noise or is
   blank, the feature vector is far from every prototype, or the top probability is low.
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
ROI_SIZE = 256

FEATURE_NAMES = (
    "v_mean", "grey_std", "log_lap", "edge_density",
    "lbp_entropy", "lbp_flat", "lbp_nonuniform",
    "glcm_c1", "glcm_c8", "orient_entropy",
    "lab_a", "lab_b", "sat_mean", "sat_std", "hue_cos", "hue_sin",
    "green_frac", "dark_frac", "spec_frac",
    "log_fine", "log_mid", "log_coarse", "autocorr1", "crack_frac",
)
FEATURE_VERSION = 5

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
    """Central square crop (frac of the short side), resized to size x size."""
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


def extract_features(roi: np.ndarray) -> Dict[str, float]:
    """Classical texture and colour features of a BGR ROI (ideally ROI_SIZE square)."""
    grey_u8 = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    g = grey_u8.astype(np.float32)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    H = hsv[..., 0].astype(np.float32)
    S = hsv[..., 1].astype(np.float32)
    V = hsv[..., 2].astype(np.float32)
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB).astype(np.float32)

    f: Dict[str, float] = {}
    f["v_mean"] = float(V.mean() / 255.0)
    f["grey_std"] = float(g.std() / 255.0)
    gb = cv2.GaussianBlur(g, (0, 0), 1.0)
    lap = cv2.Laplacian(gb, cv2.CV_32F, ksize=3)
    f["log_lap"] = float(math.log10(float(lap.var()) + 1.0))
    edges = cv2.Canny(cv2.GaussianBlur(grey_u8, (5, 5), 1.2), 40, 110)
    f["edge_density"] = float((edges > 0).mean())

    hist = _lbp_hist(gb)
    f["lbp_entropy"] = _entropy(hist) / math.log2(10)
    f["lbp_flat"] = float(hist[0] + hist[8])
    f["lbp_nonuniform"] = float(hist[9])

    q = np.floor(gb / 16.0)
    sd = max(1e-3, float(q.std()))
    f["glcm_c1"] = float(np.abs(q[:, 1:] - q[:, :-1]).mean() + np.abs(q[1:] - q[:-1]).mean()) / (2 * sd)
    f["glcm_c8"] = float(np.abs(q[:, 8:] - q[:, :-8]).mean() + np.abs(q[8:] - q[:-8]).mean()) / (2 * sd)

    gx = cv2.Sobel(gb, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gb, cv2.CV_32F, 0, 1, ksize=3)
    mag = np.sqrt(gx * gx + gy * gy)
    ang = (np.arctan2(gy, gx) % np.pi)
    oh, _ = np.histogram(ang, bins=18, range=(0, np.pi), weights=mag)
    f["orient_entropy"] = _entropy(oh / max(1e-9, oh.sum())) / math.log2(18) if oh.sum() > 0 else 1.0

    f["lab_a"] = float((lab[..., 1].mean() - 128.0) / 128.0)
    f["lab_b"] = float((lab[..., 2].mean() - 128.0) / 128.0)
    f["sat_mean"] = float(S.mean() / 255.0)
    f["sat_std"] = float(S.std() / 255.0)
    hr = H * (2 * np.pi / 180.0)
    wsum = max(1e-6, float(S.sum()))
    f["hue_cos"] = float((np.cos(hr) * S).sum() / wsum)
    f["hue_sin"] = float((np.sin(hr) * S).sum() / wsum)
    f["green_frac"] = float(((H >= 35) & (H <= 85) & (S > 60) & (V > 40)).mean())
    f["dark_frac"] = float((V < 45).mean())
    f["spec_frac"] = float(((V > 220) & (S < 50)).mean())

    b2 = cv2.GaussianBlur(g, (0, 0), 2.0)
    b8 = cv2.GaussianBlur(g, (0, 0), 8.0)
    f["log_fine"] = float(math.log10(float((g - b2).std()) + 1.0))
    f["log_mid"] = float(math.log10(float((b2 - b8).std()) + 1.0))
    f["log_coarse"] = float(math.log10(float(b8.std()) + 1.0))
    f["autocorr1"] = _autocorr1(g)
    f["crack_frac"] = float(math.sqrt(crack_fraction(roi)))
    return f


def _vec(f: Dict[str, float]) -> np.ndarray:
    return np.array([f[k] for k in FEATURE_NAMES], np.float64)


# ============================================================ prototypes

# Learned from grappstar.sim.terrain_textures by `python -m grappstar.vision.terrain --learn`
# (z-score parameters mu/sd plus one or two diagonal Gaussian prototypes per class).
# None means "learn on first use" (about 10 s). Regenerate after changing the features.
_PROTOTYPES: Optional[dict] = json.loads("""
{"version": 5, "features": ["v_mean", "grey_std", "log_lap", "edge_density", "lbp_entropy",
"lbp_flat", "lbp_nonuniform", "glcm_c1", "glcm_c8", "orient_entropy", "lab_a", "lab_b",
"sat_mean", "sat_std", "hue_cos", "hue_sin", "green_frac", "dark_frac", "spec_frac", "log_fine",
"log_mid", "log_coarse", "autocorr1", "crack_frac"], "mu": [0.44549, 0.05785, 2.13559, 0.06926,
0.46251, 0.57783, 0.02369, 0.16595, 0.62265, 0.97505, -0.00714, 0.10461, 0.41014, 0.09538,
0.4451, 0.37975, 0.15078, 0.13389, 0.00981, 0.85953, 0.76195, 0.81899, 0.81055, 0.07285], "sd":
[0.21462, 0.03242, 0.72249, 0.09198, 0.2887, 0.32414, 0.02064, 0.09095, 0.33232, 0.04743,
0.09922, 0.10791, 0.18829, 0.0586, 0.54843, 0.42643, 0.31054, 0.28691, 0.06116, 0.27315, 0.3029,
0.24152, 0.17198, 0.0879], "classes": {"bedrock": [{"mean": [0.12409, -0.2945, -0.28516,
-0.53985, -1.05157, 1.07333, -0.72689, -1.04851, -1.04386, -0.52018, 0.17016, -0.36998,
-1.07079, -0.75738, 0.17322, 0.16828, -0.43605, -0.4647, -0.16035, -0.4842, -0.34974, 0.78791,
0.7334, 0.82643], "std": [0.29531, 0.30189, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.71351,
0.51778, 0.46936, 0.51319, 0.28343, 0.76653, 0.90582, 0.25, 0.25, 0.25, 0.25, 0.25, 0.49021,
0.25, 0.4839]}, {"mean": [0.21573, -0.14602, 0.14301, -0.50117, -0.09543, 0.36151, -0.07035,
-0.31449, -0.58558, 0.20528, 0.50786, -0.61669, -0.95593, -0.10822, 0.20545, -0.65495, -0.3841,
-0.46138, -0.16035, 0.18971, -0.193, 0.7269, -0.0051, 1.08575], "std": [0.49335, 0.25, 0.25,
0.25, 0.44701, 0.36098, 0.26881, 0.28772, 0.25, 0.29855, 0.40266, 0.42499, 0.35277, 0.45235,
0.77723, 0.68393, 0.25, 0.25, 0.25, 0.25, 0.25, 0.31444, 0.28992, 0.42088]}], "hard_ground":
[{"mean": [0.45742, -0.72919, -0.16047, -0.62349, 0.1979, 0.05693, -0.42908, 0.25168, 0.1295,
0.49207, 0.70429, 0.34453, -0.12286, -0.97338, 0.74334, 0.13132, -0.459, -0.46667, -0.16035,
-0.29325, -0.38543, -0.35311, 0.05945, -0.82876], "std": [0.32322, 0.25, 0.25, 0.25, 0.25, 0.25,
0.25, 0.25, 0.25, 0.25, 0.49767, 0.46918, 0.39556, 0.29566, 0.25, 0.53802, 0.25, 0.25, 0.25,
0.25, 0.25, 0.36539, 0.36406, 0.25]}, {"mean": [0.74642, -0.43529, 0.2058, -0.56, 0.70302,
-0.4948, 0.50966, 0.68337, 0.33102, 0.50055, 0.8792, 0.70554, 0.17113, -0.25839, 0.72214,
0.19526, -0.45163, -0.46667, -0.15796, 0.29733, -0.22323, -0.36863, -0.90495, -0.82876], "std":
[0.35569, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25476, 0.25773, 0.26666, 0.25, 0.5315, 0.49521,
0.57645, 0.41338, 0.25, 0.27989, 0.25, 0.25, 0.25, 0.25, 0.25, 0.54171, 0.38475, 0.25]}],
"loose_soil": [{"mean": [-1.12996, 0.11893, 0.37077, 0.21821, 0.89144, -0.91385, 0.36356,
0.45742, 1.02923, 0.40963, 0.54221, -0.43597, 0.5164, 1.07556, 0.67511, -0.21853, -0.4242,
0.82659, -0.16035, 0.25768, 0.73007, -0.26013, 0.4639, -0.29436], "std": [0.25, 0.25, 0.25,
0.3386, 0.25, 0.25, 0.29811, 0.25, 0.25, 0.25, 0.28501, 0.28704, 0.3944, 0.68185, 0.25, 0.63217,
0.25, 0.31276, 0.25, 0.25, 0.25, 0.25, 0.25, 0.42545]}, {"mean": [-0.7623, 0.68605, 0.73465,
0.98114, 1.04066, -1.10807, 0.79778, 0.60858, 1.14826, 0.37375, 0.50899, -0.00747, 0.51842,
0.6194, 0.66254, 0.15785, -0.4109, 0.13778, -0.16035, 0.66224, 1.07113, 0.0661, 0.42942,
1.34638], "std": [0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.32952, 0.29525,
0.38234, 0.65529, 0.25, 0.43779, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.65425]}],
"sand_dust": [{"mean": [1.53101, -0.85935, -1.09452, -0.75298, -0.95254, 0.88134, -0.84799,
-0.56558, -0.54604, -0.93593, 0.34488, 0.10869, -0.9886, -1.03593, 0.53111, 0.19415, -0.47938,
-0.46667, 0.67681, -0.94639, -0.93935, -0.18928, -0.24637, -0.82876], "std": [0.44192, 0.25,
0.73014, 0.25, 0.66946, 0.4726, 0.40761, 0.63877, 0.53208, 1.84485, 0.3268, 0.62342, 0.51209,
0.40358, 0.53828, 0.64218, 0.25, 0.25, 2.32547, 0.82286, 0.36709, 0.69611, 1.12747, 0.25]}],
"vegetation": [{"mean": [0.05578, 1.4427, 1.40257, 2.08031, 1.21382, -1.44657, 1.85188, 1.46849,
1.27604, 0.32304, -2.13434, 1.66294, 1.33762, 0.43448, -0.90952, 1.29645, 2.19655, -0.45031,
-0.16035, 1.48661, 1.14898, 0.2307, -0.01493, 1.61934], "std": [0.25, 0.26577, 0.25, 0.25, 0.25,
0.25, 0.25, 0.25, 0.25, 0.33577, 0.34002, 0.25, 0.25, 0.38176, 0.25, 0.25, 0.25, 0.25, 0.25,
0.25, 0.25, 0.25287, 0.26168, 0.37147]}, {"mean": [-0.23336, 0.86577, 1.13551, 1.75575, 1.17558,
-1.37566, 1.59887, 1.36745, 1.19744, 0.43379, -1.90409, 1.32785, 1.30353, 0.69495, -0.94703,
1.27791, 2.23124, -0.39506, -0.16035, 1.1674, 0.89064, 0.04336, 0.02496, 0.30483], "std": [0.25,
0.25, 0.25, 0.25, 0.25, 0.25, 0.26589, 0.25, 0.25, 0.25, 0.34332, 0.25, 0.25, 0.39709, 0.25,
0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.27829, 0.31804, 0.50637]}], "water_void": [{"mean":
[-1.74996, -1.39334, -1.62863, -0.75298, -1.26435, 1.09616, -1.04003, -1.01452, -1.34098,
0.2581, 0.11893, -0.97407, -0.07896, 0.9025, -0.17612, -0.99195, -0.47436, 2.98847, -0.16035,
-1.50349, -1.88747, -2.15755, -1.38575, -0.82876], "std": [0.25, 0.25, 1.04252, 0.25, 0.47239,
0.30527, 0.25, 1.19537, 0.85007, 0.37792, 0.25, 0.25, 1.32926, 1.39091, 0.72215, 0.31434, 0.25,
0.25, 0.25, 1.15899, 0.36791, 0.25559, 2.01048, 0.25]}, {"mean": [-0.67585, 1.84304, 0.51168,
-0.61482, -0.61916, 0.7525, -0.87656, -1.13047, -0.91528, -0.53158, -0.01534, -1.90061, 0.47748,
0.74793, -2.32375, -1.98897, -0.42164, -0.39341, 0.10229, 0.43186, 1.28714, 1.83781, 0.88229,
-0.82876], "std": [0.25, 0.71994, 0.28417, 0.25, 0.53025, 0.41176, 0.34346, 0.25, 0.25, 0.86243,
0.25, 0.25, 0.309, 0.64724, 0.25, 0.28908, 0.25, 0.25, 0.25, 0.3272, 0.25, 0.53939, 0.25,
0.25]}]}}
""")
_LEARNED: Optional[dict] = None


def learn_prototypes(seeds: Sequence[int] = tuple(range(9000, 9012)),
                     noises: Sequence[float] = (0.0, 0.03, 0.06), size: int = 480) -> dict:
    """Learn per-class feature means and spreads from the procedural textures."""
    from grappstar.sim.terrain_textures import texture
    rows: Dict[str, List[np.ndarray]] = {c: [] for c in CLASSES}
    for c in CLASSES:
        for s in seeds:
            for i, nz in enumerate(noises):
                img = texture(c, size, size, seed=int(s) * 7 + i, noise=float(nz),
                              analog=bool((s + i) % 3 == 0))
                rows[c].append(_vec(extract_features(central_roi(img))))
    allx = np.concatenate([np.stack(v) for v in rows.values()])
    mu = allx.mean(0)
    sd = allx.std(0) + 1e-9
    protos = {}
    for c in CLASSES:
        z = (np.stack(rows[c]) - mu) / sd
        protos[c] = [{"mean": m.tolist(), "std": np.maximum(s, 0.25).tolist()}
                     for m, s in _two_means(z)]
    return {"version": FEATURE_VERSION, "features": list(FEATURE_NAMES), "mu": mu.tolist(),
            "sd": sd.tolist(), "classes": protos}


def _two_means(z: np.ndarray, iters: int = 10) -> List[tuple]:
    """Split one class into up to two clusters (some classes are bimodal, e.g. water vs void).

    Deterministic 2-means; a cluster is kept only if it holds at least 25 percent of the
    samples, otherwise the class gets a single prototype.
    """
    one = [(z.mean(0), z.std(0))]
    if len(z) < 8:
        return one
    a = z[np.argmax(((z - z.mean(0)) ** 2).sum(1))]
    b = z[np.argmax(((z - a) ** 2).sum(1))]
    cents = np.stack([a, b])
    for _ in range(iters):
        lab = np.argmin(((z[:, None, :] - cents[None]) ** 2).sum(2), 1)
        if lab.min() == lab.max():
            return one
        cents = np.stack([z[lab == k].mean(0) for k in range(2)])
    counts = np.bincount(lab, minlength=2)
    if counts.min() < 0.25 * len(z):
        return one
    return [(z[lab == k].mean(0), z[lab == k].std(0)) for k in range(2)]


def _prototypes() -> dict:
    global _LEARNED
    if _LEARNED is None:
        p = _PROTOTYPES
        if not p or p.get("version") != FEATURE_VERSION or p.get("features") != list(FEATURE_NAMES):
            p = learn_prototypes()
        means, stds, owner = [], [], []
        for i, c in enumerate(CLASSES):
            for pr in p["classes"][c]:
                means.append(pr["mean"])
                stds.append(pr["std"])
                owner.append(i)
        _LEARNED = {"mu": np.array(p["mu"]), "sd": np.array(p["sd"]), "mean": np.array(means),
                    "std": np.array(stds), "owner": np.array(owner)}
    return _LEARNED


TEMPERATURE = 0.3       # softmax temperature on mean squared z distance
OOD_DIST = 9.0          # mean squared z distance to the nearest prototype above this = unknown
MIN_CONFIDENCE = 0.45   # top probability below this = unknown


def classify_features(f: Dict[str, float]) -> Dict[str, object]:
    """Prototype classifier. Returns probs (over CLASSES), min distance and rule notes."""
    P = _prototypes()
    z = (_vec(f) - P["mu"]) / P["sd"]
    dp = (((z[None, :] - P["mean"]) / P["std"]) ** 2).mean(1)
    d2 = np.array([dp[P["owner"] == i].min() for i in range(len(CLASSES))])
    probs = _softmax(-0.5 * d2 / TEMPERATURE)
    pr = {c: float(probs[i]) for i, c in enumerate(CLASSES)}
    notes: List[str] = []
    # Hand rules (transparent overrides on top of the prototypes).
    if f["green_frac"] > 0.25 and f["edge_density"] > 0.04:
        target = min(0.9, 0.4 + f["green_frac"])
        if pr["vegetation"] < target:
            pr = _push(pr, "vegetation", target)
            notes.append("%d percent of the patch is green (plants)." % round(100 * f["green_frac"]))
    if f["dark_frac"] > 0.7 and f["log_fine"] < 1.4:
        if pr["water_void"] < 0.85:
            pr = _push(pr, "water_void", 0.85)
        notes.append("The patch is very dark and smooth: water, a hole or deep shadow.")
    elif f["spec_frac"] > 0.01 and f["v_mean"] < 0.45 and f["log_fine"] < 1.2:
        if pr["water_void"] < 0.7:
            pr = _push(pr, "water_void", 0.7)
        notes.append("Bright glints on a dark smooth surface look like water.")
    return {"probs": pr, "min_d2": float(d2.min()), "notes": notes}


def _push(pr: Dict[str, float], cls: str, target: float) -> Dict[str, float]:
    rest = sum(v for k, v in pr.items() if k != cls)
    out = {k: (v / rest * (1 - target) if rest > 0 else (1 - target) / (len(pr) - 1))
           for k, v in pr.items() if k != cls}
    out[cls] = target
    return out


def crack_fraction(roi: np.ndarray) -> float:
    """Fraction of ROI pixels on long thin dark lines (black-hat filter, elongated blobs)."""
    g = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    g = cv2.GaussianBlur(g, (3, 3), 0)
    bh = cv2.morphologyEx(g, cv2.MORPH_BLACKHAT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))
    m = (bh > 18).astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    total = 0
    for i in range(1, n):
        w, h, a = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT], stats[i, cv2.CC_STAT_AREA]
        if max(w, h) >= 30 and a < 0.25 * w * h + 1:
            total += a
    return float(total) / m.size


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
    items = []
    for i, fr in enumerate(frames or []):
        if isinstance(fr, np.ndarray):
            items.append((fr, i, None, False))
        elif hasattr(fr, "img"):
            items.append((fr.img, int(getattr(fr, "idx", i)), getattr(fr, "quality", None),
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
    items = _as_items(frames)
    good = [it for it in items if it[0] is not None and it[0].size > 0 and not it[3]
            and (it[2] is None or it[2] >= min_quality)]
    reasons: List[str] = []
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
    noise_like = raw_ac < 0.15 and fmean["grey_std"] > 0.05
    blank = fmean["grey_std"] < 0.006 and fmean["dark_frac"] < 0.7
    ood = min_d2 > OOD_DIST and (mprobs is None or max(mprobs.values()) < 0.6)
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


if __name__ == "__main__":
    import argparse
    import sys
    ap = argparse.ArgumentParser(description="Landing zone verdict from dart camera images")
    ap.add_argument("images", nargs="*", help="landing images (BGR files)")
    ap.add_argument("--out", default="landing.png", help="report image path")
    ap.add_argument("--learn", action="store_true", help="print learned prototypes as JSON")
    a = ap.parse_args()
    if a.learn:
        print(json.dumps(learn_prototypes()))
        sys.exit(0)
    imgs = [cv2.imread(p) for p in a.images]
    rep = analyze_landing([i for i in imgs if i is not None])
    if rep.image is not None:
        cv2.imwrite(a.out, rep.image)
    print(json.dumps(rep.to_dict(), indent=1))
