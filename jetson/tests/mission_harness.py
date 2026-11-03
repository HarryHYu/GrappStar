"""Test harness: mock base + mock dart linked like the real system, and a scripted operator."""
import threading
import time

from grappstar.ballistics import Ballistics, make_placeholder
from grappstar.comms.base import BaseClient, pipe_pair
from grappstar.comms.dart import Dart
from grappstar.comms.mock_base import MockBase, World
from grappstar.comms.mock_dart import MockDart
from grappstar.jlog import JLog
from grappstar.mission import Mission, MissionConfig
from grappstar.rover import RoverDrive

SCALE = 20.0


class FakeCam:
    def __init__(self, connected=True):
        self.connected = connected
        self.rec = None

    def status(self):
        return {"connected": self.connected, "bad_frac": 0.0, "recording": self.rec}

    def buffer(self, seconds=None):
        return []

    def start_recording(self, p):
        self.rec = p

    def stop_recording(self):
        p, self.rec = self.rec, None
        return p


class Rig:
    def __init__(self, tmp_path, verdict="GO", flight_s=1.1, cfg=None, rover=None, anchor_holds=True,
                 cams=None, target=(3.6, 3.0)):
        self.world = World()
        a, b = pipe_pair()
        self.mb = MockBase(a, time_scale=SCALE, world=self.world).start()
        self.base = BaseClient(b, reconnect_s=0.05).start()
        self.base.wait_connected(2)
        self.base.wait_tel(lambda t: True, 2)
        self.md = MockDart(time_scale=SCALE).start()
        self.dart = Dart(self.md.host, timeout=0.5, retries=1)
        self.flight_s = flight_s
        self.anchor_holds = anchor_holds
        self.world.on_fire = self._fire
        self._stop = threading.Event()
        threading.Thread(target=self._sync, daemon=True).start()
        self.verdict = verdict
        self.analyse_calls = 0
        c = cfg or MissionConfig()
        c.settle_s = 0.1
        c.host_timeout_s = 0          # the mock runs 20x fast: a 1.5 s watchdog would be 75 ms of wall time
        c.still_s = 0.3
        c.mode_timeout_s = 5
        c.anchor_timeout_s = 10
        c.release_timeout_s = 10
        c.retrieve_speed_mps = 0.5
        self.log = JLog(str(tmp_path / "mission.jsonl"))
        self.target = {"range_m": target[0], "bearing_deg": target[1]} if target else None
        self.m = Mission(self.base, self.dart, cams if cams is not None else {"dart": FakeCam(), "rover": FakeCam()},
                         Ballistics.load(), self.log, cfg=c,
                         rover=rover or ScriptRover(self), get_target=lambda: self.target, run_dir=str(tmp_path),
                         analyse=self._analyse)
        self.answers = []
        self.policy = {}
        self._op = threading.Thread(target=self._operator, daemon=True)
        self._op.start()

    def _fire(self):
        self.md.launch()
        self.world.flight = lambda t: 3.8 * min(1.0, t / self.flight_s)
        threading.Timer(self.flight_s / SCALE, self._land).start()

    def _land(self):
        self.md.land()
        self.world.dart = "free"

    def _sync(self):
        last = None
        while not self._stop.is_set():
            ph = self.md.phase
            if ph != last:
                if ph == "ANCHORED" and self.anchor_holds:
                    self.world.dart = "anchored"; self.world.slack_m = 0.2
                elif ph in ("RELEASING", "IDLE", "FAULT") and self.world.dart == "anchored":
                    self.world.dart = "free"
                last = ph
            time.sleep(0.01)

    def _analyse(self, frames, fl, outdir, **kw):
        self.analyse_calls += 1
        v = self.verdict if isinstance(self.verdict, str) else self.verdict[min(self.analyse_calls, len(self.verdict)) - 1]
        return {"terrain": {"cls": "hard_ground" if v == "GO" else "sand_dust", "score": 80 if v == "GO" else 20,
                            "verdict": v, "reasons": ["test"]}, "flight": None, "mosaic": None, "files": {},
                "errors": [], "flightlog_ok": fl is not None}

    def _operator(self):
        while not self._stop.is_set():
            p = self.m.prompt
            if p is not None and p.answer is None:
                choice = None
                for key, ch in self.policy.items():
                    if key in p.text:
                        choice = ch(p) if callable(ch) else ch
                        break
                if choice is None:
                    choice = p.choices[0]
                self.answers.append((p.text, choice))
                self.m.answer(p.id, choice)
            time.sleep(0.01)

    def run(self, timeout=60):
        self.m.start()
        self.m.thread.join(timeout)
        assert not self.m.thread.is_alive(), f"mission hung at {self.m.step}: {self.answers[-3:]}"
        return self.m

    def steps(self):
        return [h["state"] for h in self.m.history]

    def close(self):
        self._stop.set()
        self.base.close(); self.mb.stop(); self.md.stop(); self.log.close()


class ScriptRover(RoverDrive):
    name = "script"

    def __init__(self, rig, rounds=2):
        self.rig, self.rounds, self.calls = rig, rounds, 0

    def tow_assist(self, mission, tension_n):
        self.calls += 1
        if self.calls >= self.rounds:
            return "done"
        with self.rig.mb.lock:
            self.rig.world.slack_m += 0.3   # rover drove 0.3 m toward the dart: the held line goes slack
            self.rig.mb.tension = 1.0       # (with a load cell the base winds again by itself, like the firmware)
        return "slack"
