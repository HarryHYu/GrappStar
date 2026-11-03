#!/usr/bin/env python3
"""GrappStar end-to-end system test: the REAL code of all three parts talking to each other, no hardware.

  base    systest/build/base_shim: firmware/base/grappstar_base/base_logic.h + firmware/base/test/sim.h plant, C1 over a pty
  dart    firmware/dart/sim/kedge_sim: the real kedge_dart.ino on stub Arduino headers, C2 over HTTP on 127.0.0.1
  jetson  jetson/grappstar App (comms/base.py over the pty, comms/dart.py over HTTP, mission.py, vision, analysis) with
          synthetic camera sources (jetson/grappstar/sim: rendered dart flight, rover view with an orange cone)

The harness plays the outside world and the operator:
  - world coupler: when the base plant releases the carriage, the dart sim plays a launch + landing accelerometer trace
    and the dart video plays the rendered flight; when the dart says ANCHORED the line on the base plant goes taut;
    when it releases the line goes free; the rover hook drives toward the dart (slack) during the tow
  - operator: answers every mission prompt by a per-scenario policy (all prompts and answers are logged)
  - faults: per-scenario triggers inject faults through the shim control socket, the dart /sim/set knobs, or by
    killing and restarting the base process

  python3 systest/harness.py [--scenario NAME|all] [--speed X] [--list]
Results: systest/runs/<scenario>/{timeline.jsonl, summary.json, base.log, dart.log, jetson/...}; exit code 0 = all pass.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
import urllib.request
from typing import Any, Callable, Dict, List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "jetson"))

SHIM = os.path.join(HERE, "build", "base_shim")
DART_SIM = os.path.join(ROOT, "firmware", "dart", "sim", "kedge_sim")
RUNS = os.path.join(HERE, "runs")


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# ====================================================================================================== base process
class BaseProc:
    def __init__(self, rundir: str, speed: float):
        self.rundir, self.speed = rundir, speed
        self.link = os.path.join(rundir, "base_tty")
        self.state = os.path.join(rundir, "base_state")
        self.port = free_port()
        self.proc: Optional[subprocess.Popen] = None
        self.lock = threading.Lock()
        self.sock = None
        self.f = None
        self.log = open(os.path.join(rundir, "base.log"), "a")

    def start(self, fresh: bool) -> None:
        args = [SHIM, "--link", self.link, "--ctl", str(self.port), "--state", self.state, "--speed", str(self.speed)]
        if fresh:
            args.append("--fresh")
        self.proc = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        t0 = time.time()
        while True:
            try:
                self.sock = socket.create_connection(("127.0.0.1", self.port), timeout=2)
                self.f = self.sock.makefile("rw")
                break
            except OSError:
                if time.time() - t0 > 5:
                    raise RuntimeError("base shim did not start")
                time.sleep(0.05)

    def ctl(self, line: str) -> dict:
        with self.lock:
            try:
                self.f.write(line + "\n")
                self.f.flush()
                r = self.f.readline()
                return json.loads(r) if r else {}
            except (OSError, ValueError, AttributeError):
                return {}

    def get(self) -> dict:
        return self.ctl("get")

    def set(self, **kv) -> dict:
        return self.ctl("set " + " ".join(f"{k} {v}" for k, v in kv.items()))

    def cmd(self, obj: dict) -> dict:
        return self.ctl("cmd " + json.dumps(obj))

    def kill(self) -> None:
        """Power loss: SIGKILL, nothing is flushed (plant.bin is at most 100 ms old, nvs.bin as written)."""
        with self.lock:
            if self.proc:
                self.proc.send_signal(signal.SIGKILL)
                self.proc.wait()
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = self.f = None
            try:
                os.unlink(self.link)          # the port disappears with the board
            except OSError:
                pass

    def stop(self) -> None:
        try:
            self.ctl("quit")
            self.proc.wait(3)
        except Exception:
            if self.proc:
                self.proc.kill()


# ====================================================================================================== dart process
class DartProc:
    def __init__(self, rundir: str):
        self.port = free_port()
        self.host = f"127.0.0.1:{self.port}"
        self.log = open(os.path.join(rundir, "dart.log"), "a")
        self.proc = subprocess.Popen([DART_SIM, "--port", str(self.port)], stdout=self.log, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL)
        t0 = time.time()
        while True:
            try:
                self.get("/status")
                break
            except Exception:
                if time.time() - t0 > 10:
                    raise RuntimeError("dart sim did not start")
                time.sleep(0.1)

    def get(self, path: str, timeout: float = 2.0) -> str:
        with urllib.request.urlopen(f"http://{self.host}{path}", timeout=timeout) as r:
            return r.read().decode()

    def status(self) -> dict:
        return json.loads(self.get("/status"))

    def stop(self) -> None:
        self.proc.terminate()
        try:
            self.proc.wait(3)
        except Exception:
            self.proc.kill()


# ====================================================================================================== synthetic video
class DartVideo:
    """Dart camera: rail view before the shot, the rendered flight from the moment the base releases, then the
    landed view. Rendered for the pitch the base aimed at."""

    def __init__(self, fps: float = 15.0, size=(480, 320), landing_class: str = "hard_ground", seed: int = 3):
        self.fps, self.size, self.landing_class, self.seed = fps, size, landing_class, seed
        self.t_fire: Optional[float] = None
        self.frames = None
        self.truth = None
        self.pitch = None
        self.lock = threading.Lock()
        import numpy as np
        self._rng = np.random.default_rng(seed)

    def render(self, pitch: float, v0: float) -> None:
        from grappstar.sim.flight_video import render_flight
        frames, truth = render_flight(seed=self.seed, fps=self.fps, landing_class=self.landing_class, size=self.size,
                                      dropouts=True, landed_s=3.0, pitch_deg=pitch, v0=v0)
        truth.pop("world_map", None)
        with self.lock:
            self.frames, self.truth, self.pitch = frames, truth, pitch
            tl = truth["t_launch"]
            self.rail = [f for f in frames if f.t < tl - 0.05] or frames[:1]
            self.landed = [f for f in frames if f.t > truth["t_landed"]] or frames[-1:]

    def flight_s(self) -> float:
        return float(self.truth["t_impact"] - self.truth["t_launch"]) if self.truth else 1.0

    def frame(self):
        import numpy as np
        with self.lock:
            if self.frames is None:
                return np.full((self.size[1], self.size[0], 3), 40, np.uint8)
            if self.t_fire is None:
                img = self.rail[int(time.time() * self.fps) % len(self.rail)].img
            else:
                vt = self.truth["t_launch"] + (time.time() - self.t_fire)
                if vt >= self.landed[0].t:
                    img = self.landed[int(time.time() * self.fps) % len(self.landed)].img
                else:
                    img = min(self.frames, key=lambda f: abs(f.t - vt)).img
        n = self._rng.integers(-2, 3, img.shape, dtype=np.int16)
        return np.clip(img.astype(np.int16) + n, 0, 255).astype(np.uint8)


class LoopSource:
    def __init__(self, fn, fps: float):
        self.fn, self.period, self.idx, self.t_next = fn, 1.0 / fps, 0, time.time()

    def read(self):
        from grappstar.video.frame import Frame
        now = time.time()
        if now < self.t_next:
            time.sleep(self.t_next - now)
        self.t_next = max(self.t_next + self.period, time.time() - self.period)
        self.idx += 1
        return Frame(img=self.fn(), t=time.time(), idx=self.idx)

    def close(self):
        pass


# ====================================================================================================== the test rig
class Rig:
    def __init__(self, sc: "Scenario", speed: float):
        self.sc = sc
        self.speed = speed
        self.dir = os.path.join(RUNS, sc.name)
        shutil.rmtree(self.dir, ignore_errors=True)
        os.makedirs(self.dir)
        self.tl = open(os.path.join(self.dir, "timeline.jsonl"), "w")
        self.tl_lock = threading.Lock()
        self.t0 = time.time()
        self.stop_ev = threading.Event()
        self.events: List[dict] = []
        self.prompts: List[dict] = []
        self.dart_phases: List[str] = []
        self.plant_max: Dict[str, float] = {}
        self.plant_last: dict = {}
        self.step_plant: Dict[str, dict] = {}
        self.checks: List[dict] = []
        self.fired_seen = 0
        self.tow_rounds = 0
        self.fault_injected: List[str] = []
        self.done_triggers = set()
        self.base_seen_off = False

    # -------------------------------------------------------------- logging
    def note(self, kind: str, **kw) -> None:
        rec = dict(t=round(time.time() - self.t0, 2), kind=kind, **kw)
        with self.tl_lock:
            self.tl.write(json.dumps(rec, default=str) + "\n")
            self.tl.flush()
        if kind in ("step", "prompt", "inject", "evt", "dart", "check", "info"):
            s = " ".join(f"{k}={v}" for k, v in kw.items())
            print(f"  [{rec['t']:7.1f}] {kind:6s} {s[:200]}", flush=True)

    # -------------------------------------------------------------- start
    def start(self) -> None:
        sc = self.sc
        self.base = BaseProc(self.dir, self.speed)
        self.base.start(fresh=True)
        t0 = time.time()                       # wait for BOOT (60 ms): earlier cal/set lines are lost (FINDINGS F1)
        while self.base.get().get("t", 0) < 200 and time.time() - t0 < 3:
            time.sleep(0.02)
        # a new build on the bench (BENCH_TEST / CALIBRATION): carriage declared at rest, motor constants as calibrated
        self.base.cmd({"id": 0, "cmd": "cal", "what": "cock_rest"})
        if sc.calibrated:   # CALIBRATION.md done on the bench: k_n_per_a and free_a match the motor (here: the plant model)
            g = self.base.get()
            self.base.cmd({"id": 0, "cmd": "set", "key": "k_n_per_a", "val": round(g["kNperA_cal"], 3)})
            self.base.cmd({"id": 0, "cmd": "set", "key": "free_a", "val": round(g["freeA_cal"], 3)})
        for k, v in sc.base_setup.items():
            self.base.cmd({"id": 0, "cmd": "set", "key": k, "val": v})
        if sc.plant_setup:
            self.base.set(**sc.plant_setup)
        self.dart = DartProc(self.dir)
        for q in sc.dart_setup:
            self.dart.get("/sim/set?" + q)
        # Jetson
        from grappstar.app import App, AppConfig
        from grappstar.sim.world import default_rover_model, render_rover_view
        self.video = DartVideo(landing_class=sc.landing_class)
        self.video.render(30.0, 7.5)           # provisional rail view (re-rendered for the solution before the start)
        cfg = AppConfig(sim=False, base_port=self.base.link, dart_host=self.dart.host,
                        runs_dir=os.path.join(self.dir, "jetson"), calib_path=os.path.join(self.dir, "nocalib.json"))
        for k, v in sc.mission_cfg.items():
            setattr(cfg.mission, k, v)
        rig = self
        model = default_rover_model(1280, 720)
        rover_img = render_rover_view(model, (640, 360), marker=sc.target, seed=1)

        class TestApp(App):
            def _start_cameras(self):
                from grappstar.video.camera import Camera
                self.cams["dart"] = Camera("dart", lambda: LoopSource(rig.video.frame, rig.video.fps), buffer_s=15)
                self.cams["rover"] = Camera("rover", lambda: LoopSource(lambda: rover_img.copy(), 10.0), buffer_s=2,
                                            quality=False)

            def _load_cam_model(self):
                self.cam_model = model

        self.app = TestApp(cfg, rover=ScriptRover(self)).start()
        self.app.base.add_evt_callback(self._on_evt)
        threading.Thread(target=self._world, name="world", daemon=True).start()
        threading.Thread(target=self._operator, name="operator", daemon=True).start()

    def _on_evt(self, e: dict) -> None:
        e2 = {k: v for k, v in e.items() if not k.startswith("_")}
        self.events.append(dict(e2, _wall=time.time()))
        self.note("evt", e=json.dumps(e2))

    # -------------------------------------------------------------- world coupler
    def _world(self) -> None:
        last_shots, last_phase, last_step = 0, None, None
        anchored = False
        while not self.stop_ev.is_set():
            p = self.base.get()
            if p:
                self.plant_last = p
                step = self.app.mission.step if self.app.mission else ""
                for k in ("tension", "amps"):
                    key = f"{step}:{k}"
                    self.plant_max[key] = max(self.plant_max.get(key, 0.0), float(p.get(k) or 0))
                    self.plant_max[k] = max(self.plant_max.get(k, 0.0), float(p.get(k) or 0))
                self.plant_max["dryFires"] = max(self.plant_max.get("dryFires", 0), p.get("dryFires", 0))
                if step != last_step:
                    self.step_plant[step] = p
                    self.note("plant", step=step, **{k: p.get(k) for k in ("state", "drumDeg", "lineOut", "wraps",
                                                                           "tension", "seated", "inLaunch", "inReel")})
                    last_step = step
                if p.get("shots", 0) > last_shots:
                    last_shots = p["shots"]
                    self.fired_seen = last_shots
                    self.video.t_fire = time.time()
                    fms = int(self.video.flight_s() * 1000)
                    try:
                        self.dart.get(f"/sim/launch?push_ms=40&flight_ms={fms}&impact=1")
                    except Exception as e:
                        self.note("info", msg=f"dart launch trace failed: {e}")
                    self.note("info", msg=f"plant released the carriage (shot {last_shots}), line out {p.get('lineOut')}")
            try:
                ds = self.dart.status()
                ph = ds.get("phase")
                if ph != last_phase:
                    self.dart_phases.append(ph)
                    self.note("dart", phase=ph, seq=ds.get("seq"), ev=ds.get("event"))
                    if ph == "ANCHORED" and self.sc.anchor_holds:
                        lo = float(self.base.get().get("lineOut") or 0)
                        self.base.set(anchorTautAt=round(max(0.0, lo - 0.3), 3))
                        anchored = True
                    elif anchored and ph in ("RELEASING", "IDLE", "FAULT"):
                        self.base.set(anchorTautAt=-1)
                        anchored = False
                    last_phase = ph
            except Exception:
                pass
            for trig in self.sc.triggers:
                tid = trig[0]
                if tid in self.done_triggers:
                    continue
                try:
                    if trig[1](self):
                        self.done_triggers.add(tid)
                        self.note("inject", what=tid)
                        self.fault_injected.append(tid)
                        trig[2](self)
                except Exception as e:
                    self.note("info", msg=f"trigger {tid} error {e}", tb=traceback.format_exc(limit=3))
            if self.video.t_fire and self.app.mission and self.app.mission.step in ("LAUNCH_MODE", "AIM") \
                    and time.time() - self.video.t_fire > 10:
                self.video.t_fire = None           # dart home: rail view again for a reshoot
            self.stop_ev.wait(0.05)

    # -------------------------------------------------------------- operator
    def _operator(self) -> None:
        while not self.stop_ev.is_set():
            m = self.app.mission
            if m is not None:
                p = m.prompt
                if p is not None and p.answer is None:
                    ch = self.sc.answer(self, m.step, p.text, p.choices)
                    self.prompts.append({"step": m.step, "text": p.text, "choices": p.choices, "answer": ch})
                    self.note("prompt", step=m.step, text=p.text[:150], choices=p.choices, answer=ch)
                    time.sleep(0.05)
                    m.answer(p.id, ch)
            time.sleep(0.05)

    # -------------------------------------------------------------- run
    def run(self) -> dict:
        self.start()
        app = self.app
        # wait for the base link, the first telemetry and the dart video render
        t0 = time.time()
        while not (app.base.link().get("connected") and app.base.tel):
            if time.time() - t0 > 10:
                raise RuntimeError("base link never came up")
            time.sleep(0.1)
        app.base.set("line_m", 0)   # the operator tells a new build that the line is fully in (see FINDINGS F2)
        # target: the orange cone on the rover view (vision), else typed range
        r = None
        for _ in range(30):
            r = app.action("target_auto")
            if r.get("ok"):
                break
            time.sleep(0.2)
        self.note("info", msg=f"target: {r}")
        if not (r or {}).get("ok"):
            app.action("target_range", {"range_m": self.sc.target[0], "bearing_deg": 0})
        tgt = app.target or {}
        sol = app.bal.solve(float(tgt.get("range_m") or self.sc.target[0]), app.cfg.mission.drag_n)
        self.solution = sol.to_dict()
        rng_true = self.sc.target[0]
        self.base.set(lineOutAfterShot=round(rng_true * 1.05 + 0.3, 2))
        self.video.render(sol.pitch_deg if sol.reachable else 35.0, float(sol.to_dict().get("v_muzzle") or 7.1))
        self.note("info", msg=f"solution {self.solution}")
        time.sleep(4.0)                        # let the camera quality window fill with real frames
        r = app.action("start")
        self.note("info", msg=f"start: {r}")
        deadline = time.time() + self.sc.timeout_s
        while app.mission and app.mission.running() and time.time() < deadline:
            time.sleep(0.2)
        hung = bool(app.mission and app.mission.running())
        if hung:
            self.note("info", msg="mission still running at the scenario timeout: STOP")
            app.action("stop")
        time.sleep(1.0)
        res = self.collect(hung)
        return res

    def collect(self, hung: bool) -> dict:
        app = self.app
        m = app.mission
        steps = [h["state"] for h in (m.history if m else [])]
        tel = dict(app.base.tel or {})
        plant = self.base.get()
        res = {"scenario": self.sc.name, "mission_state": m.state if m else None, "error": m.error if m else None,
               "hung": hung, "steps": steps, "final_tel": tel, "plant": plant, "plant_max": self.plant_max,
               "events": [e.get("evt") for e in self.events], "dart_phases": self.dart_phases,
               "prompts": self.prompts, "faults_injected": self.fault_injected, "shot": m.shot if m else None,
               "solution": getattr(self, "solution", None), "tow_rounds": self.tow_rounds,
               "duration_s": round(time.time() - self.t0, 1)}
        self.res = res
        checks = []

        def chk(name, ok, detail=""):
            checks.append({"check": name, "ok": bool(ok), "detail": str(detail)[:300]})
            self.note("check", name=name, ok=bool(ok), detail=str(detail)[:200])
        for name, fn in self.sc.checks:
            try:
                ok, det = fn(self, res)
            except Exception as e:
                ok, det = False, f"check raised {type(e).__name__}: {e}"
            chk(name, ok, det)
        res["checks"] = checks
        res["pass"] = all(c["ok"] for c in checks)
        with open(os.path.join(self.dir, "summary.json"), "w") as f:
            json.dump(res, f, indent=1, default=str)
        return res

    def close(self) -> None:
        self.stop_ev.set()
        try:
            self.app.stop()
        except Exception:
            pass
        try:
            self.base.stop()
        except Exception:
            pass
        try:
            self.dart.stop()
        except Exception:
            pass

    # -------------------------------------------------------------- helpers for scenarios
    def step(self) -> str:
        return self.app.mission.step if self.app.mission else ""

    def evt_names(self) -> List[str]:
        return [e.get("evt") for e in self.events]

    def restart_base(self, off_s: float = 2.0) -> None:
        self.base.kill()
        self.note("info", msg=f"base process killed (power loss), off {off_s} s")
        time.sleep(off_s)
        self.base.start(fresh=False)
        self.note("info", msg="base process restarted (plant + NVS kept)")


class ScriptRover:
    """Rover hook during the tow: the rover drives toward the dart (slack appears on the anchored line) for a few
    rounds, then reports free."""
    can_drive = True
    name = "systest"

    def __init__(self, rig: Rig):
        self.rig = rig

    def drive(self, speed, seconds):
        pass

    def stop(self):
        pass

    def tow_assist(self, mission, tension_n):
        r = self.rig
        r.tow_rounds += 1
        r.note("info", msg=f"tow round {r.tow_rounds}: base holds at {tension_n} N (plant {r.base.get().get('tension')} N)")
        if r.tow_rounds >= r.sc.tow_rounds:
            return "done"
        p = r.base.get()
        if float(p.get("anchorTautAt", -1)) >= 0:
            r.base.set(anchorTautAt=round(float(p["anchorTautAt"]) - 0.6, 3))   # rover 0.6 m closer: slack
        return "slack"


# ====================================================================================================== scenarios
class Scenario:
    def __init__(self, name: str, desc: str, **kw):
        self.name, self.desc = name, desc
        self.target = kw.get("target", (4.0, 0.0))
        self.landing_class = kw.get("landing_class", "hard_ground")
        self.base_setup = kw.get("base_setup", {})
        self.plant_setup = kw.get("plant_setup", {})
        self.dart_setup = kw.get("dart_setup", [])
        self.mission_cfg = dict(dict(settle_s=1.0), **kw.get("mission_cfg", {}))
        self.triggers = kw.get("triggers", [])
        self.anchor_holds = kw.get("anchor_holds", True)
        self.calibrated = kw.get("calibrated", True)
        self.tow_rounds = kw.get("tow_rounds", 3)
        self.timeout_s = kw.get("timeout_s", 420)
        self.policy = kw.get("policy", {})        # substring of prompt text -> answer (first match wins)
        self.retry_max = kw.get("retry_max", 3)
        self.checks = kw.get("checks", []) or nominal_checks()
        self.extra_checks = kw.get("extra_checks", [])
        self.checks = self.checks + self.extra_checks
        self._retries = 0

    def answer(self, rig: Rig, step: str, text: str, choices: List[str]) -> str:
        for sub, ans in self.policy.items():
            if sub in text:
                if callable(ans):
                    a = ans(rig, text, choices)
                    if a in choices:
                        return a
                elif ans in choices:
                    return ans
        prefer = ["Continue", "Aim", "Fire", "FIRE NOW", "Anchor", "Anchor anyway", "Shoot again", "Dart is down",
                  "Release the dart"]
        for p in prefer:
            if p in choices:
                return p
        if "Retry" in choices and self._retries < self.retry_max:
            self._retries += 1
            time.sleep(1.0)
            return "Retry"
        return "Abort" if "Abort" in choices else choices[0]


# -------------------------------------------------------------------------------- checks (rig, res) -> (ok, detail)
def c_done(rig, res):
    return res["mission_state"] == "DONE" and not res["hung"], f"{res['mission_state']} {res['error']}"


def c_evt(name, n=1):
    def f(rig, res):
        k = res["events"].count(name)
        return k >= n, f"{name} x{k}"
    return f


def c_steps(*names):
    def f(rig, res):
        miss = [s for s in names if s not in res["steps"]]
        return not miss, f"missing {miss}" if miss else "all present"
    return f


def c_aim(rig, res):
    ev = [e for e in rig.events if e.get("evt") == "AIMED"]
    if not ev:
        return False, "no AIMED"
    e = ev[-1]
    sol = (res.get("solution") or {}).get("pitch_deg")
    ok = e.get("ok") and sol is not None and abs(e.get("pitch_deg", 0) - sol) <= 1.0
    return ok, f"AIMED {e.get('pitch_deg')} target {e.get('target_deg')} solution {sol}"


def c_fired(rig, res):
    return res["plant"].get("shots", 0) >= 1 and "FIRED" in res["events"], f"plant shots {res['plant'].get('shots')}"


def c_landed(rig, res):
    return "LANDED" in res["dart_phases"], f"dart phases {res['dart_phases']}"


def c_anchored(rig, res):
    return "ANCHORED" in res["dart_phases"], f"dart phases {res['dart_phases']}"


def c_tidy(rig, res):
    """Spec D16/D22: TIDY after anchoring (take-up winds the slack). Pass when the TIDY step ran and the base did it."""
    msgs = [l for l in _mission_log(rig) if "tidy" in l.lower()]
    ran = "TIDY" in res["steps"]
    skipped = any("no tidy" in m for m in msgs)
    return ran and not skipped, ("skipped: " + msgs[0]) if skipped else f"TIDY step {'ran' if ran else 'missing'}"


def c_wrapped(rig, res):
    p = rig.step_plant.get("CHECK") or rig.step_plant.get("TOW") or {}
    return "WRAPPED" in res["events"] and float(p.get("wraps") or 0) >= 3.5, f"plant wraps at CHECK/TOW {p.get('wraps')}"


def c_reel_engaged(rig, res):
    p = rig.step_plant.get("TOW") or {}
    # the TOW snapshot is taken at the step change; the dog must be in REEL whenever the capstan pulls
    return bool(rig.plant_max.get("TOW:tension", 0) > 5) and p.get("drumDeg") is not None, \
        f"max tension in TOW {rig.plant_max.get('TOW:tension')} N"


def c_tow_limit(rig, res):
    lim = rig.app.cfg.mission.tow_limit_n
    mx = max(rig.plant_max.get("TOW:tension", 0), rig.plant_max.get("CHECK:tension", 0))
    return mx <= lim * 1.15, f"max plant tension {mx:.1f} N, limit {lim} N (+15 %)"


def c_released(rig, res):
    return "RELEASING" in res["dart_phases"], f"dart phases {res['dart_phases']}"


def c_reeled(rig, res):
    """Reel in on the capstan (wraps on) to the approach distance or all the way to seated."""
    p = rig.step_plant.get("UNWRAP") or {}
    r = rig.step_plant.get("RETRIEVE") or {}
    return p.get("lineOut") is not None and float(p["lineOut"]) <= rig.app.cfg.mission.approach_m + 0.3, \
        f"line out {r.get('lineOut')} m at RETRIEVE start, {p.get('lineOut')} m at UNWRAP"


def c_unwrapped(rig, res):
    return float(res["plant"].get("wraps", 9)) < 0.05 and "UNWRAPPED" in res["events"], f"plant wraps {res['plant'].get('wraps')}"


def c_seated(rig, res):
    return bool(res["plant"].get("seated")) and (res["final_tel"].get("sw") or {}).get("S") == 1, \
        f"plant seated {res['plant'].get('seated')}, tel sw {res['final_tel'].get('sw')}"


def c_launch(rig, res):
    t = res["final_tel"]
    return t.get("drum_pos") == "LAUNCH" and res["plant"].get("inLaunch") and not t.get("fault"), \
        f"tel {t.get('drum_pos')} {t.get('mode')} fault {t.get('fault')}, plant inLaunch {res['plant'].get('inLaunch')}"


def c_no_dry(rig, res):
    return rig.plant_max.get("dryFires", 0) == 0 and res["plant"].get("dryFires", 0) == 0, "dry fires (collar left LAUNCH with the spring drawn)"


def c_no_fault_left(rig, res):
    return not res["final_tel"].get("fault"), f"fault {res['final_tel'].get('fault')}"


def _mission_log(rig) -> List[str]:
    out = []
    d = rig.app.run_dir
    if d and os.path.exists(os.path.join(d, "mission.jsonl")):
        for l in open(os.path.join(d, "mission.jsonl")):
            try:
                out.append(json.loads(l).get("msg") or "")
            except ValueError:
                pass
    return out


def nominal_checks():
    return [("mission DONE", c_done), ("aim reached", c_aim), ("fired", c_fired), ("landed (dart)", c_landed),
            ("anchored (dart)", c_anchored), ("tidy ran", c_tidy), ("wrapped (4+ wraps on the capstan)", c_wrapped),
            ("REEL engaged, tow pulled", c_reel_engaged), ("tow limit respected", c_tow_limit),
            ("dart released", c_released), ("reeled in to the approach", c_reeled), ("unwrapped", c_unwrapped),
            ("seated", c_seated), ("back to LAUNCH", c_launch), ("no dry fire", c_no_dry)]


# triggers: (id, condition(rig), action(rig))
def at_step(name, delay=0.0):
    st = {}

    def cond(rig):
        if rig.step() == name:
            st.setdefault("t", time.time())
            return time.time() - st["t"] >= delay
        return False
    return cond


def plant_state(state, step=None, delay=0.0):
    st = {}

    def cond(rig):
        if rig.plant_last.get("state") == state and (step is None or rig.step() == step):
            st.setdefault("t", time.time())
            return time.time() - st["t"] >= delay
        return False
    return cond


def on_evt(name, after=0.0):
    st = {}

    def cond(rig):
        if name in rig.evt_names():
            st.setdefault("t", time.time())
            return time.time() - st["t"] >= after
        return False
    return cond


def later(dt, then):
    def act(rig):
        threading.Timer(dt, then, args=(rig,)).start()
    return act


SCENARIOS: Dict[str, Scenario] = {}


def sc(s: Scenario):
    SCENARIOS[s.name] = s
    return s


sc(Scenario("nominal", "whole mission, aim to seated, no faults"))

sc(Scenario("nominal_uncal", "whole mission with the factory k_n_per_a / free_a (CALIBRATION.md not done)",
            calibrated=False))

sc(Scenario("drum_timeout", "drum N20 jams on the way to NEUTRAL after the shot: DRUM_TIMEOUT, operator frees it, Retry",
            triggers=[("jam drum", plant_state("MODE", step="BASE_NEUTRAL", delay=0.3),
                       lambda r: r.base.set(drumJam=1)),
                      ("operator frees the drum", on_evt("FAULT", after=1.0), lambda r: r.base.set(drumJam=0))],
            extra_checks=[("DRUM_TIMEOUT raised", lambda r, res: (any(e.get("code") == "DRUM_TIMEOUT" for e in r.events),
                                                                  [e for e in r.events if e.get("evt") == "FAULT"]))]))

sc(Scenario("dog_tip", "dog teeth land tip to tip on every engagement: the firmware must wiggle them in",
            plant_setup={"toothOnTooth": 1, "toothNeedWiper": 12}))

sc(Scenario("stall_cock", "the wiper stalls while cocking (STALL), load removed, Retry: no dry fire, mission completes",
            triggers=[("stall load", plant_state("COCK", delay=0.6),
                       lambda r: r.base.set(extraLoadNm=20)),
                      ("load removed", on_evt("FAULT", after=1.0), lambda r: r.base.set(extraLoadNm=0))],
            extra_checks=[("STALL or OVERCURRENT raised", lambda r, res: (any(e.get("code") in ("STALL", "OVERCURRENT") for e in r.events),
                                                                          [e for e in r.events if e.get("evt") == "FAULT"]))]))

sc(Scenario("power_unwrap", "power loss mid-unwrap: kill the base process, restart it, the mission recovers",
            triggers=[("kill base mid-unwrap", plant_state("UNWRAP", delay=3.0), lambda r: r.restart_base(2.0))],
            extra_checks=[("BOOT after restart", lambda r, res: ("BOOT" in res["events"] or r.base.get().get("boots", 0) >= 1,
                                                                 f"boots {r.base.get().get('boots')}"))]))

sc(Scenario("dart_never_anchors", "the dart's drill motor is jammed: anchor fails, operator releases and reshoots",
            dart_setup=["drill_jam=1"],
            policy={"did not anchor": "Release and reshoot", "refused auto": "Release and reshoot",
                    "release did not finish": "Abort"},
            checks=[("mission ends (DONE or ABORTED, not hung)", lambda r, res: (not res["hung"] and res["mission_state"] in ("DONE", "ABORTED"),
                                                                              f"{res['mission_state']} {res['error']}")),
                    ("anchor failure reported", lambda r, res: (any("anchor" in p["text"].lower() for p in res["prompts"]),
                                                                [p["text"][:80] for p in res["prompts"]][-4:])),
                    ("no tow on an unanchored dart", lambda r, res: ("TOW" not in res["steps"], res["steps"])),
                    ("no dry fire", c_no_dry)]))

sc(Scenario("snag_retrieve", "line snags while reeling in: LIMIT, operator frees it, Retry",
            triggers=[("snag", lambda r: r.step() == "RETRIEVE" and float(r.plant_last.get("lineOut") or 0) < 3.0
                       and r.plant_last.get("state") == "REEL",
                       lambda r: (r.base.set(anchorTautAt=round(float(r.base.get()["lineOut"]) - 0.15, 3)),
                                  later(8, lambda r2: r2.base.set(anchorTautAt=-1))(r)))],
            extra_checks=[("snag held under the retrieve limit", lambda r, res: (r.plant_max.get("RETRIEVE:tension", 0) <= r.app.cfg.mission.retrieve_limit_n * 1.2,
                                                                                f"max {r.plant_max.get('RETRIEVE:tension')} N")),
                          ("LIMIT seen in RETRIEVE", c_evt("LIMIT"))]))

sc(Scenario("no_angle_sensor", "AS5600 magnet missing from boot (D12: SW_INDEX fallback): the mission must run or refuse clearly",
            plant_setup={"encMissing": 1}, timeout_s=200,
            checks=[("mission DONE in SW_INDEX mode", c_done),
                    ("refusal is explicit (ANGLE_SENSOR)", lambda r, res: (any("ANGLE_SENSOR" in (p["text"]) for p in res["prompts"]) or "ANGLE_SENSOR" in str(res["error"]),
                                                                            res["error"])),
                    ("no dry fire", c_no_dry)]))

sc(Scenario("usb_unplug_tow", "USB unplugged for 3 s while the base holds the tow (board loses power), replugged",
            triggers=[("unplug", lambda r: r.step() == "TOW" and r.plant_last.get("state") in ("HOLD", "REEL"),
                       lambda r: (r.base.ctl("unplug"), later(1.0, lambda r2: setattr(r2, "unplugged_plant", r2.base.get()))(r),
                                  later(3.0, lambda r2: r2.base.ctl("replug"))(r),
                                  later(4.5, lambda r2: setattr(r2, "replug_plant", r2.base.get()))(r)))],
            extra_checks=[("motors off while unplugged, no motion by itself after replug",
                           lambda r, res: (getattr(r, "unplugged_plant", {}).get("wDir") == 0 and getattr(r, "replug_plant", {}).get("wDir") == 0
                                           and getattr(r, "replug_plant", {}).get("dDir") == 0,
                                           f"unplugged {getattr(r, 'unplugged_plant', {}).get('wDir')}, after replug state {getattr(r, 'replug_plant', {}).get('state')} wDir {getattr(r, 'replug_plant', {}).get('wDir')}")),
                          ("tow limit respected", c_tow_limit)]))

sc(Scenario("usb_glitch_reel", "USB re-enumerates (no reset) while reeling in: the link recovers",
            triggers=[("glitch", lambda r: r.step() == "RETRIEVE" and r.plant_last.get("state") == "REEL",
                       lambda r: r.base.ctl("glitch"))]))


# ---------------------------------------------------------------- more scenarios
def decock_then_retry(rig, text, choices):
    """Operator presses Decock on the page (carriage drawn with the angle unknown), waits for RESET, then Retry."""
    r = rig.app.action("decock")
    rig.note("info", msg=f"operator: decock -> {r}")
    t0 = time.time()
    while time.time() - t0 < 60 and "RESET" not in [e.get("evt") for e in rig.events[-5:]]:
        time.sleep(0.2)
    return "Retry"


def c_recovered(rig, res):
    return c_done(rig, res)


def c_d22_order(rig, res):
    """D22: WRAP before the dart anchors."""
    st = res["steps"]
    if "WRAP" not in st or "ANCHOR" not in st:
        return False, f"steps {st}"
    return st.index("WRAP") < st.index("ANCHOR"), f"WRAP at {st.index('WRAP')}, ANCHOR at {st.index('ANCHOR')}"


SCENARIOS["nominal"].checks.append(("D22 order: WRAP before ANCHOR", c_d22_order))

POWER_STATES = [("LAUNCH_MODE", "MODE"), ("AIM", "AIM"), ("COCK", "COCK"), ("BASE_NEUTRAL", "MODE"), ("WRAP", "WRAP"),
                ("CHECK", "REEL"), ("TOW", "HOLD"), ("RETRIEVE", "REEL"), ("LAUNCH_READY", "MODE")]
for _step, _pst in POWER_STATES:
    sc(Scenario(f"power_{_step.lower()}", f"power loss in {_step} (base state {_pst}): kill, restart, mission recovers",
                triggers=[(f"kill base in {_step}", plant_state(_pst, step=_step, delay=1.0), lambda r: r.restart_base(2.0))],
                policy={"LOADED_UNKNOWN": decock_then_retry, "phase UNKNOWN": decock_then_retry, "Decock": decock_then_retry},
                retry_max=4,
                checks=[("mission DONE after the power loss", c_done), ("no dry fire", c_no_dry),
                        ("seated", c_seated), ("back to LAUNCH", c_launch), ("tow limit respected", c_tow_limit)]))


def inject_garbage(rig):
    # the shim's inject command understands \n and \xHH escapes; quotes are plain
    lines = ['ESP-ROM:esp32s3-20210327\nrst:0x1 (POWERON),boot:0x8 (SPI_FAST_FLASH_BOOT)\n',
             '{not json at all\n',
             '{"id":1,"ok":true}\n',                      # a stale duplicate reply to an old id
             '{"tel":{"state":"IDL', 'E"}}\n',            # one line split in two writes
             '\xff\xfe\x00garbage\n',                     # bad UTF-8 and a NUL
             '{"evt":"NEWTHING","t":1}\n',                # an event the Jetson does not know
             '{"tel":"x' + ' ' * 300 + '"}\n']            # over 256 bytes
    for raw in lines:
        rig.base.ctl("inject " + raw)
        time.sleep(0.05)


sc(Scenario("serial_garbage", "malformed and wrong lines on the serial link (boot text, bad JSON, a stale reply, split, bad UTF-8, unknown event, 300 bytes) during AIM and TOW",
            triggers=[("garbage in AIM", plant_state("AIM", delay=1.0), inject_garbage),
                      ("garbage in TOW", plant_state("HOLD", step="TOW", delay=0.5), inject_garbage)]))


def nogo_policy(rig, text, choices):
    if "Retrieve and reshoot" in choices and rig.video.landing_class != "hard_ground":
        rig.video.landing_class = "hard_ground"     # the operator picks a better target for the second shot
        sol = rig.solution
        threading.Thread(target=rig.video.render, args=(sol["pitch_deg"], sol.get("v_muzzle") or 7.5), daemon=True).start()
        return "Retrieve and reshoot"
    return "Anchor" if "Anchor" in choices else "Anchor anyway"


sc(Scenario("nogo_reshoot", "first landing on water (NO GO): retrieve the free dart and reshoot, second lands on hard ground",
            landing_class="water_void", policy={"Landing zone": nogo_policy}, timeout_s=600,
            extra_checks=[("two shots", lambda r, res: (res["plant"].get("shots", 0) == 2, f"shots {res['plant'].get('shots')}"))]))

sc(Scenario("dart_wifi_drop", "the dart's WiFi station drops right after landing",
            triggers=[("wifi drop", lambda r: "LANDED" in r.dart_phases, lambda r: r.dart.get("/sim/disconnect"))]))

sc(Scenario("brownout_tow", "battery sags under 9.9 V while towing: BROWNOUT, the drum holds (wraps on), battery back, Retry",
            triggers=[("sag", plant_state("REEL", step="TOW", delay=0.5),
                       lambda r: (r.base.set(vbatOverride=9.0), later(3.0, lambda r2: r2.base.set(vbatOverride=-1))(r)))],
            extra_checks=[("BROWNOUT raised", lambda r, res: (any(e.get("code") == "BROWNOUT" for e in r.events),
                                                              [e.get("code") for e in r.events if e.get("evt") == "FAULT"]))]))

sc(Scenario("encoder_glitch", "AS5600 returns a garbage angle every 50 reads for the whole mission",
            plant_setup={"encGlitchEvery": 50}))


# ====================================================================================================== main
def build() -> None:
    src = os.path.join(HERE, "base_shim.cpp")
    hdrs = [os.path.join(ROOT, "firmware", "base", "grappstar_base", f) for f in os.listdir(os.path.join(ROOT, "firmware", "base", "grappstar_base"))]
    hdrs.append(os.path.join(ROOT, "firmware", "base", "test", "sim.h"))
    if not os.path.exists(SHIM) or any(os.path.getmtime(h) > os.path.getmtime(SHIM) for h in hdrs + [src]):
        os.makedirs(os.path.dirname(SHIM), exist_ok=True)
        subprocess.check_call(["g++", "-std=c++17", "-O2", "-Wno-unused-parameter", "-Wno-misleading-indentation",
                               "-o", SHIM, src])
    subprocess.check_call(["make", "-s", "-C", os.path.dirname(DART_SIM)], stdout=subprocess.DEVNULL)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="nominal")
    ap.add_argument("--speed", type=float, default=float(os.environ.get("SYSTEST_SPEED", "2")))
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()
    if a.list:
        for s in SCENARIOS.values():
            print(f"{s.name:20s} {s.desc}")
        return 0
    build()
    names = list(SCENARIOS) if a.scenario == "all" else a.scenario.split(",")
    results = []
    for n in names:
        s = SCENARIOS[n]
        print(f"\n=== {n}: {s.desc}", flush=True)
        rig = Rig(s, a.speed)
        try:
            res = rig.run()
        except Exception as e:
            traceback.print_exc()
            res = {"scenario": n, "pass": False, "error": f"harness: {type(e).__name__}: {e}", "checks": []}
        finally:
            rig.close()
        results.append(res)
        print(f"=== {n}: {'PASS' if res.get('pass') else 'FAIL'} ({res.get('mission_state')}, {res.get('duration_s')} s)")
        for c in res.get("checks", []):
            print(f"    {'ok  ' if c['ok'] else 'FAIL'} {c['check']}: {c['detail']}")
    os.makedirs(RUNS, exist_ok=True)
    with open(os.path.join(RUNS, "results.json"), "w") as f:
        json.dump([{k: r.get(k) for k in ("scenario", "pass", "mission_state", "error", "duration_s", "checks", "steps")}
                   for r in results], f, indent=1, default=str)
    print("\nSUMMARY")
    for r in results:
        bad = [c["check"] for c in r.get("checks", []) if not c["ok"]]
        print(f"  {'PASS' if r.get('pass') else 'FAIL'}  {r['scenario']:20s} {r.get('mission_state')}  {'failed: ' + ', '.join(bad) if bad else ''}")
    return 0 if all(r.get("pass") for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
