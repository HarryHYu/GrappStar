"""Landing terrain verdict on full simulated flights (reduced version of tests/eval_terrain_flight.py).

2 held-out seeds x 6 classes at 480x320, through the same steps as analysis.run_post_flight. The classifier
constants are learned on flight seeds 10 and up, so seeds 0 and 1 are unseen.
"""
import time

import pytest

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


# ---------------------------------------------------------------- roll-invariant landing tilt (review R1-C #1)

def _spin_log(fl, roll_deg):
    """The same flight, with the dart spun by roll_deg about its own nose for the whole log."""
    import math
    c, s = math.cos(math.radians(roll_deg) / 2), math.sin(math.radians(roll_deg) / 2)
    cr, sr = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
    out = dict(fl)
    rows = []
    for r in fl["samples"]:
        w, x, y, z = r[1:5]
        # q' = q * qx(roll): w' = w c - x s, x' = x c + w s, y' = y c + z s, z' = z c - y s
        q = [w * c - x * s, x * c + w * s, y * c + z * s, z * c - y * s]
        ax, ay, az = r[5:8]
        # body vector in the spun body frame: Rx(roll)^T a
        a = [ax, cr * ay + sr * az, -sr * ay + cr * az]
        rows.append([r[0]] + q + a)
    out["samples"] = rows
    return out


def test_rest_tilt_is_roll_invariant_and_verdict_does_not_change():
    from grappstar import analysis
    from grappstar.sim.flight_video import render_flight
    frames, truth = render_flight(seed=3, size=(360, 240), landing_class="hard_ground", pitch_deg=30, v0=7.4)
    base = truth["flightlog"]
    reps = []
    for roll in (0, 45, 90, 180):
        fl = _spin_log(base, roll)
        t = analysis.rest_tilt(fl)
        assert t is not None and t["source"] == "accelerometer"
        res = analysis.run_post_flight(frames, fl, None, do_mosaic=False)
        reps.append((round(t["nose_elev_deg"], 1), res["terrain"]["verdict"], res["terrain"]["score"],
                     res["terrain"].get("slope_deg"), res["rest_tilt"]["tilt_deg"]))
    assert all(r[:4] == reps[0][:4] for r in reps[1:]), reps
    assert all(abs(r[4] - reps[0][4]) < 0.1 for r in reps), reps
    # the sim dart rests nose down at -54..-66 deg on flat ground: small tilt, and no tilt penalty reason
    pitch_f = [p for p in truth["poses"].values()][-1][5]
    assert reps[0][0] == pytest.approx(pitch_f, abs=2.0)
    assert reps[0][4] < 8.0
    assert reps[0][1] != "NO_GO"


def test_rest_tilt_shapes():
    import math
    from grappstar import analysis

    def log(elev_deg, roll_deg=0.0, n=60, g=9.81):
        # dart at nose elevation elev, spun by roll: specific force in body = R^T (0, 0, g)
        e, r = math.radians(elev_deg), math.radians(roll_deg)
        fx = g * math.sin(e)
        fy = g * math.cos(e) * math.sin(r)
        fz = g * math.cos(e) * math.cos(r)
        return {"hz": 50, "samples": [[i * 20, 1, 0, 0, 0, fx, fy, fz] for i in range(n)]}

    for roll in (0, 45, 90, 180, -120):
        t = analysis.rest_tilt(log(-60, roll))
        assert t["nose_elev_deg"] == pytest.approx(-60, abs=0.01) and t["tilt_deg"] == pytest.approx(0, abs=0.01)
    t = analysis.rest_tilt(log(-2.0, 90))
    assert t["expected_deg"] == 0.0 and t["tilt_deg"] == pytest.approx(2.0, abs=0.01)    # lying flat
    t = analysis.rest_tilt(log(-30.0, 33))
    assert t["tilt_deg"] == pytest.approx(30.0, abs=0.01)                               # neither pose: tilted
    assert analysis.rest_tilt(None) is None and analysis.rest_tilt({"samples": []}) is None
