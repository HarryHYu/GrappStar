"""Re-learning on recorded footage (grappstar/learn.py): run folders made from realistic synthetic shots, exactly as a
mission writes them (dart_shotN.avi + time stamps, flightlog_shotN.json with clock_offset_s, mission.jsonl)."""
import json
import os

import cv2
import numpy as np
import pytest

from grappstar.sim.flight_video import render_flight
from grappstar.video.record import Recorder

SIZE = (360, 240)


def make_run(folder, seed, pitch, cls="hard_ground", latency=0.06):
    os.makedirs(folder, exist_ok=True)
    frames, truth = render_flight(seed=seed, size=SIZE, profile="caddx", pitch_deg=pitch, landing_class=cls,
                                  latency_s=latency, link="daylight", landed_s=1.5)
    with Recorder(os.path.join(folder, "dart_shot0.avi"), fps=30) as rec:
        for f in frames:
            rec.write(f)
    fl = dict(truth["flightlog"], clock_offset_s=truth["clock_offset_s"])
    with open(os.path.join(folder, "flightlog_shot0.json"), "w") as f:
        json.dump(fl, f)
    with open(os.path.join(folder, "mission.jsonl"), "w") as f:
        f.write(json.dumps({"t": truth["t_launch"] - 20, "kind": "info", "msg": f"aimed at {pitch} deg (target {pitch})"})
                + "\n")
    return truth


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    base = tmp_path_factory.mktemp("runs")
    out = []
    for i, (seed, pitch) in enumerate(((201, 15.0), (202, 28.0), (203, 40.0))):
        d = str(base / f"run{i}")
        out.append((d, make_run(d, seed, pitch)))
    return out


def test_learn_latency_finds_the_capture_delay(runs, monkeypatch, tmp_path):
    from grappstar import learn
    monkeypatch.setattr(learn, "SETTINGS", str(tmp_path / "none.json"))
    med = learn.latency([d for d, _ in runs])
    assert med is not None and abs(med - 0.06) < 0.035, med


def test_learn_flight_writes_table_scale_and_video_timing(runs, monkeypatch, tmp_path):
    from grappstar import learn
    monkeypatch.setattr(learn, "SETTINGS", str(tmp_path / "none.json"))
    csv = tmp_path / "shots.csv"
    # the tape says every shot went 10 percent further than the simulator's truth (a launcher stronger than the
    # table): table_scale must come out near 1.1 x (truth / table)
    lines = ["run_folder,shot,range_m"]
    for d, tr in runs:
        lines.append(f"{d},0,{tr['range_pivot_m'] * 1.10:.3f}")
    csv.write_text("\n".join(lines) + "\n")
    out = tmp_path / "flight_tuning.json"
    r = learn.flight_tune(str(csv), write=True, out=str(out))
    got = json.loads(out.read_text())
    assert 1.05 < got["table_scale"] < 1.16, r
    assert 0.05 <= got["VID_T_LATE"] < 0.4 and 0.0 <= got["VID_IMPACT_LATE_TYP"] < 0.3
    for row in r["rows"]:
        assert row["imu_m"] is not None and abs(row["imu_m"] / (row["tape_m"] / 1.10) - 1) < 0.04, row


def test_table_scale_reaches_the_solver_and_the_tuning_reaches_flight(tmp_path, monkeypatch):
    from grappstar import ballistics
    from grappstar.vision import flight
    p = tmp_path / "flight_tuning.json"
    p.write_text(json.dumps({"table_scale": 1.1, "VID_T_LATE": 0.2, "bogus": 5}))
    monkeypatch.setenv("GRAPPSTAR_FLIGHT_TUNING", str(p))
    plain = ballistics.Ballistics.load(ballistics.REPO_TABLE)
    assert plain.range_scale == pytest.approx(1.1)
    raw = ballistics.Ballistics(json.load(open(ballistics.REPO_TABLE)))
    assert plain.at_pitch(30)["range_m"] == pytest.approx(1.1 * raw.at_pitch(30)["range_m"])
    s = plain.solve(4.0)
    assert "measured shots" in s.note and s.pitch_deg < raw.solve(4.0).pitch_deg
    old = flight.VID_T_LATE
    try:
        assert flight.load_tuning(str(p)) == {"VID_T_LATE": 0.2}
        assert flight.VID_T_LATE == 0.2
    finally:
        flight.VID_T_LATE = old


def test_learn_extract_and_terrain_from_labelled_pictures(runs, monkeypatch, tmp_path):
    from grappstar import learn
    from grappstar.vision import terrain
    monkeypatch.setattr(learn, "SETTINGS", str(tmp_path / "none.json"))
    lab = tmp_path / "labelled"
    n = learn.extract([d for d, _ in runs], str(lab), n=3)
    assert n >= 6 and len(os.listdir(lab / "_unsorted")) == n
    # Harry sorts them: all three runs landed on hard ground
    for f in os.listdir(lab / "_unsorted"):
        os.replace(lab / "_unsorted" / f, lab / "hard_ground" / f)
    # and adds a few sand pictures from elsewhere
    from grappstar.sim.terrain_textures import texture_view
    for i in range(4):
        cv2.imwrite(str(lab / "sand_dust" / f"s{i}.png"), texture_view("sand_dust", 480, 720, seed=500 + i))
    from grappstar.vision.lens import DartLens
    real = terrain.real_samples(str(lab), lens=DartLens.from_settings())
    # a dart leaning on a flank on hard ground (dart-physics) can show too little ground in some pictures: skipped
    assert len(real["hard_ground"]) >= 3 and len(real["sand_dust"]) == 4
    monkeypatch.setattr(terrain, "site_prototypes_path", lambda: str(tmp_path / "calib" / "terrain_prototypes.json"))
    r = learn.terrain_learn(str(lab), write=True, jobs=1, quick=True)
    assert r["after"] is not None and r["after"] >= 0.8
    assert os.path.exists(tmp_path / "calib" / "terrain_prototypes.json")
