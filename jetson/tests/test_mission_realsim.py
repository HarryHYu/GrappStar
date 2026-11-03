"""The whole mission against the REAL base firmware logic (jetson/tools/basesim: base_logic.h + sim.h) and the mock
dart (API v4). Skipped when basesim cannot be built. Speed 8x real time; about a minute per run.
"""
import os
import threading
import time

import pytest

from basesim_transport import find_basesim, start_basesim
from grappstar.ballistics import Ballistics
from grappstar.comms.dart import Dart
from grappstar.comms.mock_dart import MockDart
from grappstar.jlog import JLog
from grappstar.mission import Mission, MissionConfig
from grappstar.rover import RoverDrive

SPEED = float(os.environ.get("GRAPPSTAR_BASESIM_SPEED", "8"))
pytestmark = pytest.mark.skipif(find_basesim() is None, reason="basesim not built")


class Rover(RoverDrive):
    def __init__(self):
        self.calls = 0

    def tow_assist(self, mission, tension_n):
        self.calls += 1
        return "done"


class RealRig:
    def __init__(self, tmp_path, verdict="GO", line_out=3.8, slack=0.3):
        self.proc, self.base = start_basesim(speed=SPEED)
        if (self.base.tel or {}).get("fault"):
            self.base.cmd("clear", check=False)
        # WIP firmware trips during the draw (reported to fw-base); a bench-sane trip keeps the test about the Jetson
        try:
            r = self.base.cmd("get", key="trip_a")
            if r.get("val", 99) < 7:
                self.base.cmd("set", key="trip_a", val=7)
        except Exception:
            pass
        self.md = MockDart(time_scale=SPEED).start()
        self.dart = Dart(self.md.host, timeout=0.5)
        self.line_out, self.slack, self.verdict = line_out, slack, verdict
        self.base.add_evt_callback(self._evt)
        self._stop = threading.Event()
        threading.Thread(target=self._sync, daemon=True).start()
        cfg = MissionConfig(settle_s=0.1, still_s=0.3, approach_m=1.0)
        self.log = JLog(str(tmp_path / "mission.jsonl"))
        bal = Ballistics.load()
        probe = bal.solve(1.0)
        rng = round((probe.range_min_m + probe.range_max_m) / 2, 2)
        self.m = Mission(self.base, self.dart, {}, bal, self.log, cfg=cfg, rover=Rover(),
                         get_target=lambda: {"range_m": rng, "bearing_deg": 0.0}, run_dir=str(tmp_path),
                         analyse=self._analyse)
        self.answers = []
        threading.Thread(target=self._operator, daemon=True).start()

    def _evt(self, e):
        if e.get("evt") == "FIRED":
            self.md.launch()
            self.base.transport.sim(f"set lineOut {self.line_out}")
            threading.Timer(1.0 / SPEED, self.md.land).start()

    def _sync(self):
        last = None
        while not self._stop.is_set():
            ph = self.md.phase
            if ph != last:
                if ph == "ANCHORED":
                    self.base.transport.sim(f"anchor {self.line_out - self.slack}")
                elif ph == "IDLE" and last == "RELEASING":
                    self.base.transport.sim("free")
                last = ph
            time.sleep(0.02)

    def _analyse(self, frames, fl, outdir, **kw):
        v = self.verdict
        return {"terrain": {"cls": "hard_ground", "score": 80, "verdict": v, "reasons": ["test"]},
                "flight": {"range_m": self.line_out - 0.3}, "mosaic": None, "files": {}, "errors": []}

    def _operator(self):
        while not self._stop.is_set():
            p = self.m.prompt
            if p is not None and p.answer is None:
                ch = p.choices[0]
                self.answers.append((self.m.step, p.text, ch))
                self.m.answer(p.id, ch)
            time.sleep(0.01)

    def close(self):
        self._stop.set()
        self.base.close()
        self.md.stop()
        self.log.close()


def test_full_mission_against_real_base_firmware(tmp_path):
    r = RealRig(tmp_path)
    try:
        r.m.start()
        r.m.thread.join(240)
        steps = [h["state"] for h in r.m.history]
        assert not r.m.thread.is_alive(), f"hung at {r.m.step}: {r.answers[-3:]}"
        assert r.m.state == "DONE", (r.m.error, steps, r.answers[-4:])
        t = r.base.tel
        assert t["mode"] == "LAUNCH" and (t.get("sw") or {}).get("S") and not t.get("fault")
        for s in ("COCK", "FIRE", "BASE_NEUTRAL", "WRAP", "CHECK", "TOW", "RELEASE", "RETRIEVE", "UNWRAP"):
            assert s in steps, s
        failed = [a for a in r.answers if "failed" in a[1]]
        assert not failed, failed
    finally:
        r.close()


# ------------------------------------------------------------------ both real firmwares
from test_dart_realsim import SIM as DART_SIM, _free_port  # noqa: E402


@pytest.mark.skipif(DART_SIM is None, reason="dart firmware simulator not built")
def test_full_mission_both_real_firmwares(tmp_path):
    import subprocess
    import urllib.request
    port = _free_port()
    dp = subprocess.Popen([DART_SIM, "--port", str(port)], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL)
    r = RealRig(tmp_path)
    r.md.stop()                                    # replace the mock dart by the real firmware simulator
    dart = Dart(f"127.0.0.1:{port}", timeout=1.0)
    t0 = time.time()
    while True:
        try:
            dart.status()
            break
        except Exception:
            assert time.time() - t0 < 10, "dart simulator did not start"
            time.sleep(0.2)
    r.m.dart = dart
    r.dart = dart

    def evt(e):
        if e.get("evt") == "FIRED":
            urllib.request.urlopen(f"http://127.0.0.1:{port}/sim/launch?push_ms=40&flight_ms=900&impact=1", timeout=2)
            r.base.transport.sim(f"set lineOut {r.line_out}")
    r.base._evt_cbs = [evt]
    r._stop.set()                                  # stop the mock-dart sync; run our own on the real dart

    stop = threading.Event()

    def sync():
        last = None
        while not stop.is_set():
            try:
                ph = dart.status()["phase"]
            except Exception:
                time.sleep(0.2)
                continue
            if ph != last:
                if ph == "ANCHORED":
                    r.base.transport.sim(f"anchor {r.line_out - r.slack}")
                elif ph == "IDLE" and last == "RELEASING":
                    r.base.transport.sim("free")
                last = ph
            time.sleep(0.2)
    threading.Thread(target=sync, daemon=True).start()
    answers = []

    def operator():
        while not stop.is_set():
            p = r.m.prompt
            if p is not None and p.answer is None:
                answers.append((r.m.step, p.text, p.choices[0]))
                r.m.answer(p.id, p.choices[0])
            time.sleep(0.01)
    threading.Thread(target=operator, daemon=True).start()
    try:
        r.m.start()
        r.m.thread.join(300)
        steps = [h["state"] for h in r.m.history]
        assert not r.m.thread.is_alive(), f"hung at {r.m.step}: {answers[-3:]}"
        assert r.m.state == "DONE", (r.m.error, steps, answers[-4:])
        assert not [a for a in answers if "failed" in a[1]], answers
        assert r.m.shot.get("landing_sources") and "dart" in r.m.shot["landing_sources"]
    finally:
        stop.set()
        r.close()
        dp.terminate()
        dp.wait(5)
