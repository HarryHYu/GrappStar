"""Landing terrain verdict on full simulated flights (reduced version of tests/eval_terrain_flight.py).

2 held-out seeds x 6 classes at 480x320, through the same steps as analysis.run_post_flight. The classifier
constants are learned on flight seeds 10 and up, so seeds 0 and 1 are unseen.
"""
import time

import numpy as np

import eval_terrain_flight as E
from grappstar.video.frame import Frame
from grappstar.vision import terrain as T


def test_flight_landing_frames_480x320():
    t0 = time.time()
    out = E.evaluate(seeds=(0, 1), sizes=((480, 320),), flightlog_modes=(False,), jobs=1, verbose=True)
    elapsed = time.time() - t0
    d = out["by_size"]["480x320"]
    assert d["acc"] >= 0.70, E.format_confusion(d["cm"])
    assert not d["unsafe_go"], d["unsafe_go"]
    assert elapsed < 90, elapsed


def test_dark_landing_is_no_go():
    rng = np.random.default_rng(1)
    # a dark void with sensor noise, marked bad by the camera's quality check: still NO_GO, not "no frames"
    frames = [Frame(np.clip(12 + rng.normal(0, 4, (480, 720, 3)), 0, 255).astype(np.uint8), t=float(i), idx=i,
                    quality=0.1, bad=True) for i in range(6)]
    rep = T.analyze_landing(frames)
    assert rep.cls == "water_void" and rep.verdict == "NO_GO"
    assert any("void" in r for r in rep.reasons)
    # the receiver's flat black screen: cannot tell, but never GO
    black = [np.full((480, 720, 3), 8, np.uint8) for _ in range(5)]
    rep = T.analyze_landing(black)
    assert rep.cls == "unknown" and rep.verdict == "NO_GO"
    assert any("link" in r for r in rep.reasons)
    assert abs(sum(rep.to_dict()["probs"].values()) - 1.0) < 1e-6


def test_link_artefacts_skipped_without_quality():
    from grappstar.sim.terrain_textures import texture
    good = [texture("hard_ground", 480, 720, seed=3, noise=0.02) for _ in range(2)]
    blue = np.zeros((480, 720, 3), np.uint8)
    blue[:] = (200, 30, 20)
    snow = np.random.default_rng(0).integers(0, 256, (480, 720, 3), dtype=np.uint8)
    rep = T.analyze_landing([snow, blue] + good, make_image=False)
    assert set(rep.used_frames) <= {2, 3}
    assert rep.cls == "hard_ground"


def test_runtime_15_frames_720x480():
    from grappstar.sim.terrain_textures import texture_view
    frames = [Frame(texture_view("loose_soil", 480, 720, seed=i, zoom=2.5, blur=1.0, noise=0.02), t=float(i),
                    idx=i) for i in range(15)]
    T.analyze_landing(frames[:3])      # warm up (prototype tables)
    t0 = time.perf_counter()
    rep = T.analyze_landing(frames)
    dt = time.perf_counter() - t0
    assert rep.cls == "loose_soil"
    assert dt < 1.0, dt                # target about 0.3 s on the dev PC, generous for slow CI
