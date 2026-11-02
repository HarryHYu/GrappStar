import json
import sys
import types

import numpy as np
import pytest

from grappstar.sim.terrain_textures import CLASSES, texture
from grappstar.video.frame import Frame
from grappstar.vision import terrain as T
from grappstar.vision.terrain import (OnnxTerrainModel, StubModel, TerrainModel, analyze_landing,
                                      verdict_for)

SEEDS = range(6)
NOISES = (0.0, 0.03, 0.06)


def _frames(cls, seed=0, n=3, noise=0.03, quality=0.9):
    return [Frame(texture(cls, 480, 720, seed=seed * 10 + i, noise=noise), t=float(i), idx=i,
                  quality=quality) for i in range(n)]


def test_confusion_matrix():
    names = list(CLASSES) + ["unknown"]
    cm = np.zeros((len(CLASSES), len(names)), int)
    for i, c in enumerate(CLASSES):
        for s in SEEDS:
            for j, nz in enumerate(NOISES):
                img = texture(c, 480, 720, seed=s, noise=nz, analog=bool((s + j) % 2))
                rep = analyze_landing([img], make_image=False)
                cm[i, names.index(rep.cls)] += 1
    print("\nconfusion matrix (rows = truth, cols = predicted)")
    print("            " + " ".join(n[:7].rjust(7) for n in names))
    for i, c in enumerate(CLASSES):
        print(c[:11].ljust(11), " ".join(str(x).rjust(7) for x in cm[i]))
    per_class = cm.diagonal() / cm.sum(1)
    print("per class accuracy", dict(zip(CLASSES, np.round(per_class, 2))))
    assert per_class.min() >= 0.75
    assert cm.diagonal().sum() / cm.sum() >= 0.85


def test_unknown_for_pure_noise():
    rng = np.random.default_rng(0)
    imgs = [rng.integers(0, 256, (480, 720, 3), dtype=np.uint8) for _ in range(3)]
    rep = analyze_landing(imgs)
    assert rep.cls == "unknown"
    assert rep.verdict == "UNSURE"


def test_unknown_for_blank_frame():
    rep = analyze_landing([np.full((480, 720, 3), 120, np.uint8)])
    assert rep.cls == "unknown"


def test_all_bad_frames():
    frames = _frames("hard_ground")
    for f in frames:
        f.bad = True
    rep = analyze_landing(frames)
    assert rep.cls == "unknown" and rep.verdict == "UNSURE"
    assert rep.image.shape == (540, 960, 3)
    low_q = _frames("hard_ground", quality=0.1)
    assert analyze_landing(low_q).cls == "unknown"
    assert analyze_landing([]).cls == "unknown"


def test_only_good_frames_used():
    frames = _frames("loose_soil", n=4)
    frames[0].bad = True
    frames[1].quality = 0.2
    rep = analyze_landing(frames)
    assert set(rep.used_frames) <= {2, 3}
    assert rep.cls == "loose_soil"


def test_verdicts_per_class():
    expect = {"hard_ground": "GO", "sand_dust": "NO_GO", "water_void": "NO_GO", "bedrock": "NO_GO"}
    for c, v in expect.items():
        rep = analyze_landing(_frames(c, seed=1))
        assert rep.cls == c, (c, rep.probs)
        assert rep.verdict == v, (c, rep.score, rep.reasons)
    rep = analyze_landing(_frames("bedrock", seed=1))
    assert any("augers cannot" in r for r in rep.reasons)
    assert any("rack" in r for r in rep.reasons)          # cracks mentioned either way
    assert verdict_for(60) == "GO" and verdict_for(59) == "UNSURE" and verdict_for(39) == "NO_GO"


def test_low_quality_pushes_to_unsure():
    good = analyze_landing(_frames("hard_ground", seed=2, quality=0.95))
    poor = analyze_landing(_frames("hard_ground", seed=2, quality=0.45))
    assert good.verdict == "GO"
    assert poor.verdict == "UNSURE"
    assert poor.confidence < good.confidence
    assert any("quality" in r for r in poor.reasons)


def test_imu_slope_penalty():
    flat = analyze_landing(_frames("hard_ground", seed=3), imu_roll_deg=2, imu_pitch_deg=1)
    steep = analyze_landing(_frames("hard_ground", seed=3), imu_roll_deg=30, imu_pitch_deg=20)
    assert flat.slope_source == "imu" and steep.slope_deg > 30
    assert steep.score < flat.score - 15
    assert any("tilted" in r for r in steep.reasons)


def test_report_image_and_json():
    rep = analyze_landing(_frames("vegetation", seed=4))
    assert rep.image.shape == (540, 960, 3) and rep.image.dtype == np.uint8
    d = rep.to_dict()
    s = json.dumps(d)
    assert "image" not in d and json.loads(s)["verdict"] == rep.verdict
    assert set(d["probs"]) == set(CLASSES) | {"unknown"}
    assert abs(sum(d["probs"].values()) - 1.0) < 1e-2
    assert all(chr(0x2014) not in r for r in rep.reasons)


def test_plain_image_input():
    rep = analyze_landing(texture("sand_dust", 480, 720, seed=9, noise=0.03))
    assert rep.cls == "sand_dust"


def test_stub_model_fusion():
    frames = _frames("hard_ground", seed=5)
    assert isinstance(StubModel(), TerrainModel)
    uni = analyze_landing(frames, model=StubModel())
    assert uni.cls == "hard_ground" and uni.model_probs is not None
    forced = analyze_landing(frames, model=StubModel({"water_void": 1.0}), model_weight=1.0)
    assert forced.cls == "water_void" and forced.verdict == "NO_GO"
    half = analyze_landing(frames, model=StubModel({"water_void": 1.0}), model_weight=0.5)
    assert abs(half.probs["water_void"] - (0.5 + 0.5 * half.classical_probs["water_void"])) < 1e-6


def test_failing_model_falls_back():
    class Broken(TerrainModel):
        def predict(self, img):
            raise RuntimeError("boom")
    rep = analyze_landing(_frames("loose_soil", seed=1), model=Broken())
    assert rep.cls == "loose_soil"
    assert any("model failed" in r for r in rep.reasons)


class _FakeSession:
    last_input = None

    def __init__(self, path, providers=None, **kw):
        self.path = path
        self.providers = providers

    def get_inputs(self):
        return [types.SimpleNamespace(name="images", shape=[1, 3, 224, 224])]

    def get_modelmeta(self):
        names = {0: "bedrock", 1: "hard_ground", 2: "loose_soil", 3: "sand_dust",
                 4: "vegetation", 5: "water_void"}
        return types.SimpleNamespace(custom_metadata_map={"names": repr(names)})

    def run(self, outputs, feeds):
        _FakeSession.last_input = feeds["images"]
        return [np.array([[0.1, 4.0, 0.2, 0.3, -1.0, 0.0]], np.float32)]   # logits


def test_fake_onnx_model(monkeypatch):
    fake = types.ModuleType("onnxruntime")
    fake.InferenceSession = _FakeSession
    fake.get_available_providers = lambda: ["CPUExecutionProvider"]
    monkeypatch.setitem(sys.modules, "onnxruntime", fake)
    m = OnnxTerrainModel("terrain.onnx")
    assert m.class_names[1] == "hard_ground"
    p = m.predict(texture("sand_dust", 480, 720, seed=1))
    x = _FakeSession.last_input
    assert x.shape == (1, 3, 224, 224) and x.dtype == np.float32
    assert 0.0 <= x.min() and x.max() <= 1.0
    assert max(p, key=p.get) == "hard_ground" and abs(sum(p.values()) - 1) < 1e-6
    # YOLOv8-cls style output (already probabilities) passes through unchanged.
    monkeypatch.setattr(_FakeSession, "run",
                        lambda self, o, f: [np.array([[0.05, 0.05, 0.1, 0.7, 0.05, 0.05]], np.float32)])
    p2 = m.predict(texture("sand_dust", 480, 720, seed=1))
    assert abs(p2["sand_dust"] - 0.7) < 1e-6
    rep = analyze_landing(_frames("sand_dust", seed=1), model=m)
    assert rep.cls == "sand_dust" and rep.model_probs["sand_dust"] > 0.69
    # ImageNet normalisation option changes the input range.
    m2 = OnnxTerrainModel("terrain.onnx", class_names=list(CLASSES), imagenet_norm=True)
    x2 = m2.preprocess(texture("bedrock", 100, 100, seed=0))
    assert x2.min() < 0


def test_onnx_missing_runtime(monkeypatch):
    monkeypatch.setitem(sys.modules, "onnxruntime", None)
    with pytest.raises(ImportError):
        OnnxTerrainModel("x.onnx")


def test_attitude_tuple_slope():
    rep = analyze_landing(_frames("hard_ground", seed=3), attitude=(1.0, 5.0, -60.0, 10.0))
    assert rep.slope_source == "imu" and abs(rep.slope_deg - 30.0) < 1e-6
    assert T.slope_from_attitude((2.0, 3.0)) > 3.0
    assert T.slope_from_attitude("bad") is None
