"""Integration: the App in simulator mode, driven through its action API like the web page does, then replayed."""
import json
import os
import time

import pytest

from grappstar.app import App, AppConfig
from grappstar import cli


@pytest.fixture(scope="module")
def app(tmp_path_factory):
    d = tmp_path_factory.mktemp("runs")
    cfg = AppConfig(sim=True, runs_dir=str(d), sim_landing="hard_ground", sim_seed=2)
    cfg.mission.settle_s = 0.5
    a = App(cfg).start()
    assert a.sim.wait_ready(60)
    t0 = time.time()
    while not all(c.status().get("connected") for c in a.cams.values()) and time.time() - t0 < 10:
        time.sleep(0.1)
    yield a
    a.stop()


def test_status_shape(app):
    s = app.status()
    for k in ("base", "dart", "mission", "cams", "results", "target", "config", "log"):
        assert k in s
    assert s["base"]["link"]["connected"]
    assert s["dart"]["link"]["connected"] or app.dart.status()
    json.dumps(s)


def test_targeting_actions(app):
    r = app.action("target_auto", {})
    assert r["ok"], r
    t = r["target"]
    assert abs(t["range_m"] - 3.6) < 0.4 and abs(t["bearing_deg"] - 5.0) < 1.5
    assert t["solution"]["reachable"]
    r = app.action("target_click", {"u": 0.5, "v": 0.05})
    assert not r["ok"]                     # sky: above the horizon
    r = app.action("target_range", {"range_m": 3.8, "bearing_deg": 0})
    assert r["ok"] and app.target["pitch_deg"] is not None
    assert not app.action("set", {"key": "tow_limit_n", "val": 500})["ok"]
    assert app.action("set", {"key": "tow_limit_n", "val": 35})["ok"]
    assert app.action("nonsense", {})["ok"] is False
    assert app.action("aim", {"bogus": 1})["ok"] is False


def test_full_mission_via_actions_then_replay(app):
    assert app.action("target_auto", {})["ok"]
    r = app.action("start", {})
    assert r["ok"], r
    assert not app.action("start", {})["ok"]          # already running
    seen, t0 = set(), time.time()
    while time.time() - t0 < 420:
        m = app.status()["mission"]
        p = m["prompt"]
        if p and p["id"] not in seen:
            seen.add(p["id"])
            ch = p["choices"][0]
            if "Landing zone" in p["text"]:
                ch = "Anchor" if "Anchor" in p["choices"] else "Anchor anyway"
            if "Line is holding" in p["text"]:
                ch = "Rover free"
            assert app.action("answer", {"prompt_id": p["id"], "choice": ch})["ok"]
        if m["state"] in ("DONE", "ABORTED", "FAILED"):
            break
        time.sleep(0.1)
    assert m["state"] == "DONE", (m["error"], m["step"])
    assert [h["state"] for h in m["history"]][-1] == "DONE"
    s = app.status()
    assert s["results"]["terrain"]["verdict"] in ("GO", "UNSURE", "NO_GO")
    for name in ("landing.png", "mosaic.png", "flight.json", "mission.jsonl"):
        assert app.artifact(name) and os.path.exists(app.artifact(name)), name
    fl = s["results"]["flight"]
    assert fl and fl.get("t_launch") and fl.get("t_impact")
    # replay the recorded run on the PC
    rc = cli.main(["replay", app.run_dir])
    assert rc == 0
    assert os.path.exists(os.path.join(app.run_dir, "replay_shot0", "flight.json"))


def test_stop_action_aborts(app):
    assert app.action("target_range", {"range_m": 3.8})["ok"]
    assert app.action("start", {})["ok"]
    time.sleep(1.0)
    r = app.action("stop", {})
    assert r["ok"]
    app.mission.thread.join(5)
    assert app.mission.state == "ABORTED"
    assert app.action("clear", {})["ok"]
