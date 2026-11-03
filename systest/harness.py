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
ROOT = os.environ.get("SYSTEST_ROOT") or os.path.dirname(HERE)   # SYSTEST_ROOT: test another checkout (preview of branches)
sys.path.insert(0, os.path.join(ROOT, "jetson"))

SHIM = os.path.join(ROOT if os.environ.get("SYSTEST_ROOT") else HERE, "systest" if os.environ.get("SYSTEST_ROOT") else "", "build", "base_shim")
DART_SIM = os.path.join(ROOT, "firmware", "dart", "sim", "kedge_sim")
RUNS = os.environ.get("SYSTEST_RUNS") or os.path.join(HERE, "runs")


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# ====================================================================================================== base process
class BaseProc:
    def __init__(self, rundir: str, speed: float, hello_delay: int = 2100):
        self.rundir, self.speed, self.hello_delay = rundir, speed, hello_delay
        self.link = os.path.join(rundir, "base_tty")
        self.state = os.path.join(rundir, "base_state")
        self.port = free_port()
        self.proc: Optional[subprocess.Popen] = None
        self.lock = threading.Lock()
        self.sock = None
        self.f = None
        self.log = open(os.path.join(rundir, "base.log"), "a")

    def start(self, fresh: bool) -> None:
        with self.lock:
            self._start(fresh)

    def _start(self, fresh: bool) -> None:
        args = [SHIM, "--link", self.link, "--ctl", str(self.port), "--state", self.state, "--speed", str(self.speed), "--hello-delay", str(self.hello_delay)]
        if fresh:
            args.append("--fresh")
        self.proc = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        t0 = time.time()
        while True:
            try:
                self.sock = socket.create_connection(("127.0.0.1", self.port), timeout=2)
                self.sock.settimeout(None)             # a SocketIO that once timed out refuses every later read
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
            self.kills = getattr(self, "kills", 0) + 1
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
        self.probe_results: List[dict] = []
        self.dart_d: Optional[float] = None       # dart distance along the line from the base (m)

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
        self.base = BaseProc(self.dir, self.speed, sc.hello_delay)
        self.base.start(fresh=True)
        t0 = time.time()                       # wait for BOOT (60 ms): earlier cal/set lines are lost (FINDINGS F1)
        while time.time() - t0 < 8:          # setup() about 2.1 s, then BOOT 60 ms after the first tick
            g = self.base.get()
            if g.get("state") not in (None, "OFF") and g.get("t", 0) >= self.base.hello_delay + 150:
                break
            time.sleep(0.02)
        # a new build on the bench (BENCH_TEST / CALIBRATION): carriage declared at rest, motor constants as calibrated
        self.base.cmd({"id": 0, "cmd": "cal", "what": "cock_rest"})
        if sc.calibrated:   # CALIBRATION.md done on the bench: k_n_per_a and free_a match the motor (here: the plant model)
            g = self.base.get()
            # the plant tension is capstan-side; the firmware's estimate is DART-side (D43): k / path gain
            self.base.cmd({"id": 0, "cmd": "set", "key": "k_n_per_a", "val": round(g["kNperA_cal"] / sc.path_gain, 3)})
            self.base.cmd({"id": 0, "cmd": "set", "key": "free_a", "val": round(g["freeA_cal"], 3)})
            if not sc.doc_cal:      # a two-point calibration also zeroes the offset (FINDINGS F14)
                self.base.cmd({"id": 0, "cmd": "set", "key": "tension_offset_n", "val": 0})
        for k, v in sc.base_setup.items():
            self.base.cmd({"id": 0, "cmd": "set", "key": k, "val": v})
        d22 = sc.d22 or os.environ.get("SYSTEST_D22_ALL") == "1" or os.environ.get("SYSTEST_STRICT_ALL") == "1"
        self.base.set(wrapDraw=1 if d22 else 0)
        if not d22:     # until F4 (D22 order) is fixed, fault scenarios use the lenient plant so they test their own fault
            self.note("info", msg="lenient wrap plant (wrap does not draw dart-side line); SYSTEST_D22_ALL=1 for the strict one")
            sc.checks = [c for c in sc.checks if c[1] not in (c_wrap_free, c_tug)]
        if sc.slip_n:
            self.base.set(slipN=sc.slip_n)
        ps = dict(sc.plant_setup)
        if "drumDeg" in ps:          # the drum is turned by hand with the board off, then it boots there
            self.base.ctl("power off")
            self.base.boot_t = self.base.get().get("t", 0)
            self.base.set(drumDeg=ps.pop("drumDeg"))
            self.base.ctl("power on")
            t0 = time.time()
            while time.time() - t0 < 8:
                g = self.base.get()
                if g.get("state") not in (None, "OFF") and g.get("t", 0) >= self.base.boot_t + self.base.hello_delay + 150:
                    break
                time.sleep(0.02)
        if ps:
            self.base.set(**ps)
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
        if e2.get("evt") == "AIMED":
            self.aimed_plant = self.base.get()       # the rail's real pitch when the firmware says it is aimed
        e2["_step"] = self.step()
        self.events.append(dict(e2, _wall=time.time()))
        self.note("evt", e=json.dumps({k: v for k, v in e2.items() if k != "_step"}))

    # -------------------------------------------------------------- world coupler
    def _world(self) -> None:
        last_shots, last_phase, last_step = 0, None, None
        anchored = False
        while not self.stop_ev.is_set():
            step = self.step()
            p = self.base.ctl("peakreset")            # plant truth; peaks are tracked every 5 ms tick inside the shim
            if p and "t" in p:
                self.plant_last = p
                self.ctl_ok = time.time()
                for k, pk in (("tension", "peakTension"), ("amps", "peakAmps")):
                    v = float(p.get(pk) or 0)
                    key = f"{step}:{k}"
                    self.plant_max[key] = max(self.plant_max.get(key, 0.0), v)
                    self.plant_max[k] = max(self.plant_max.get(k, 0.0), v)
                self.plant_max["dryFires"] = max(self.plant_max.get("dryFires", 0), p.get("dryFires", 0))
                if step != last_step:
                    self.step_plant[step] = p
                    self.note("plant", step=step, **{k: p.get(k) for k in ("state", "drumDeg", "lineOut", "wraps",
                                                                           "tension", "seated", "inLaunch", "inReel")})
                    last_step = step
                if p.get("seated") and not getattr(self, "_was_seated", True) and self.fired_seen:
                    self.seat_rec = {k: p.get(k) for k in ("inReel", "inLaunch", "wraps", "drumDeg", "state")}
                    self.seat_rec["step"] = step
                    self.note("info", msg=f"dart seated: {self.seat_rec}")
                self._was_seated = bool(p.get("seated"))
                if not anchored and self.dart_d is not None and p.get("lineOut") is not None:
                    self.dart_d = min(self.dart_d, float(p["lineOut"]))   # a free dart is tugged along by a short line
                if p.get("shots", 0) > last_shots:
                    last_shots = p["shots"]
                    pt = p.get("pitchTrue")
                    if isinstance(pt, (int, float)) and pt == pt:   # the dart flies from the rail's REAL pitch
                        rr = self.app.bal.at_pitch(float(pt), self.app.cfg.mission.drag_n).get("range_m")
                        if rr:
                            self.base.set(lineOut=round(rr * 1.05 + 0.3, 3))
                            p["lineOut"] = round(rr * 1.05 + 0.3, 3)
                            self.note("info", msg=f"real pitch {pt:.2f} deg: range {rr:.2f} m, line out {p['lineOut']} m")
                    self.dart_d = float(p.get("lineOut") or 0) - self.sc.landing_slack   # landing slack on the dart side
                    self.dart_d0 = self.dart_d
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
                    if ph == "ANCHORED" and (self.sc.anchor_holds or getattr(self, "anchor_ok", False)):
                        lo = float(self.base.get().get("lineOut") or 0)
                        d = min(self.dart_d if self.dart_d is not None else lo, lo)   # the dart lies at d: taut at d
                        self.anchor_at(d)
                        self.dart_d_anchor = d
                        self.note("info", msg=f"dart anchored at {d:.2f} m, line out {lo:.2f} m (dart-side slack {lo - d:.2f} m)")
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
        if self.sc.driver == "ref":
            self.ref = RefMission(self)
            self.ref.start()
            self.note("info", msg="reference driver (BASE_SPEC 10.1) started")
            deadline = time.time() + self.sc.timeout_s
            while self.ref.thread.is_alive() and time.time() < deadline:
                time.sleep(0.2)
            time.sleep(1.0)
            return self.collect(self.ref.thread.is_alive())
        r = app.action("start")
        self.note("info", msg=f"start: {r}")
        deadline = time.time() + self.sc.timeout_s
        while app.mission and app.mission.running() and time.time() < deadline:
            time.sleep(0.2)
        hung = bool(app.mission and app.mission.running())
        after = self.sc.after or (decock_and_restart if getattr(self, "wants_decock_restart", False) else None)
        if not hung and after:
            self.first_mission = {"state": app.mission.state, "error": app.mission.error,
                                  "steps": [h["state"] for h in app.mission.history]}
            self.note("info", msg=f"first mission ended {self.first_mission['state']}: {self.first_mission['error']}")
            after(self)
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
        m = getattr(self, "ref", None) or app.mission
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
        for tid, _c, _a in self.sc.triggers:
            chk(f"fault injected: {tid}", tid in self.done_triggers, "trigger fired" if tid in self.done_triggers else "NEVER FIRED (scenario did not test what it says)")
        chk("harness: plant link alive to the end", bool(plant) and "t" in plant and time.time() - getattr(self, "ctl_ok", 0) < 5, "ctl")
        extra = []
        if not self.sc.triggers and not self.sc.expect_faults:
            extra = [("no FAULT events (no fault scenario)", c_no_fault), ("no failure prompts or retries", c_no_retry)]
        for name, fn in list(self.sc.checks) + extra:
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
        m = getattr(self, "ref", None) or self.app.mission
        return m.step if m else ""

    def evt_names(self) -> List[str]:
        return [e.get("evt") for e in self.events]

    def anchor_at(self, d: float) -> None:
        """The line goes taut at d m out, with the stiffness of d m of 1 mm Dyneema (R2-L 1: EA about 20 kN)."""
        d = max(0.0, float(d))
        stiff = self.sc.stiff or os.environ.get("SYSTEST_STRICT_ALL") == "1"
        k = self.sc.line_ea_n / max(d, 0.5) if stiff else 40.0     # 40 N/m = sim.h default (lenient, until F20)
        self.base.set(anchorTautAt=round(d, 3), anchorK=round(k, 1))

    def restart_base(self, off_s: float = 2.0) -> None:
        threading.Thread(target=self._restart_base, args=(off_s,), daemon=True).start()

    def _restart_base(self, off_s: float) -> None:
        self.base.kill()
        self.note("info", msg=f"base process killed (power loss), off {off_s} s")
        time.sleep(off_s)
        self.base.start(fresh=False)
        self.note("info", msg="base process restarted (plant + NVS kept)")


from grappstar.comms.base import BaseTimeout as BaseTimeoutErr  # noqa: E402


class RefMission:
    """BASE_SPEC 10.1 (D35) driven step by step with the Jetson's own clients (comms/base.py BaseClient, comms/dart.py
    Dart), no mission.py. It checks the firmware, the dart and the plant against the binding sequence, independent of
    the mission's own order. Step names match mission.py so the same checks apply."""

    def __init__(self, rig: Rig):
        self.rig, self.b, self.d = rig, rig.app.base, rig.app.dart
        self.history: List[dict] = []
        self.step = ""
        self.state, self.error, self.shot = "RUNNING", "", {}
        self.thread = threading.Thread(target=self._run, name="ref", daemon=True)
        self.tokens: List[str] = []
        self.probed = set()

    def start(self):
        self.thread.start()

    def _set(self, name):
        self.step = name
        self.history.append({"state": name, "t": round(time.time(), 2)})
        self.rig.note("step", name=name)

    def tx(self, name, **args):
        self.tokens.append(name + (f" {args.get('to') or args.get('until') or ''}".rstrip() if args else ""))
        return self.b.cmd(name, timeout=2.0, **args)

    def wait(self, names, timeout, m, pred=None):
        return self.b.wait_event(names, timeout, since=m, pred=pred)

    def act(self, name, timeout, wait_for, pred=None, **args):
        m = self.b.event_mark()
        self.tx(name, **args)
        return self.wait(wait_for, timeout, m, pred=pred)

    # ---------------------------------------------------------------- steps (BASE_SPEC 10.1)
    def s_launch(self):
        if (self.b.tel or {}).get("drum_pos") != "LAUNCH":
            self.act("mode", 60, "MODE", to="LAUNCH")

    def s_aim(self):
        self.act("aim", 90, "AIMED", deg=self.rig.solution["pitch_deg"])

    def s_cock(self):
        if ((self.b.tel or {}).get("cock") or 0) < 0.999:
            self.act("cock", 40, "COCKED")

    def s_fire(self):
        b, d = self.b, self.d
        if self.rig.base.get().get("shots", 0) >= 1:
            return                              # already out (the FIRED event may have been lost)
        try:
            d.arm(True, auto=False)
        except Exception as e:
            self.rig.note("info", msg=f"dart arm failed: {e}")
        tok = self.tx("arm").get("token")
        m = b.event_mark()
        self.tx("fire", token=tok)
        self.wait("FIRED", 30, m)
        self.fire_mark = m

    def s_flight(self):
        self.d.wait_phase(("LANDED", "ANCHORING", "ANCHORED"), 20)
        try:
            self.wait("RESET", 30, getattr(self, "fire_mark", 0))
        except BaseTimeoutErr:
            pass

    def s_line(self):
        self.tx("set", key="line_m", val=round(self.rig.sc.target[0] * 1.05 + 0.3, 2))

    def s_wrap(self):
        if (self.b.tel or {}).get("drum_pos") != "WRAPPED":
            self.act("wrap", 60, "WRAPPED")

    def s_anchor(self):
        st = self.d.status()
        if st.get("phase") != "ANCHORED":
            self.tokens.append("dart auto")
            if st.get("phase") != "ANCHORING":
                self.d.auto()
            self.d.wait_phase(("ANCHORED",), 60)

    def s_tidy(self):
        self.act("tidy", 20, "TIDIED", ms=3000, duty=0.3)

    def s_flyin(self):
        if (self.b.tel or {}).get("drum_pos") != "REEL":
            self.act("mode", 60, "MODE", pred=lambda e: e.get("mode") == "REEL", to="REEL")

    def s_tow(self):
        rig = self.rig
        lim = rig.app.cfg.mission.tow_limit_n
        while rig.tow_rounds < rig.sc.tow_rounds:
            if (self.b.tel or {}).get("drum_pos") != "REEL":
                self.s_flyin()                  # a fault in REEL parks the drum at WRAPPED (D35 recovery)
            e = self.act("reel", 60, ("LIMIT", "SEATED"), duty=rig.sc.tow_duty, max_n=lim, until="tension")
            if e.get("evt") != "LIMIT":
                raise RuntimeError(f"tow: {e}")
            rig.tow_rounds += 1
            p = rig.base.get()
            rig.note("info", msg=f"ref tow round {rig.tow_rounds}: LIMIT {e.get('tension_n')} N, plant {p.get('tension')} N")
            if rig.tow_rounds < rig.sc.tow_rounds and float(p.get("anchorTautAt", -1)) >= 0:
                rig.anchor_at(float(p["anchorTautAt"]) - 0.6)

    def s_release(self):
        st = self.d.status()
        if st.get("phase") == "IDLE" and st.get("seq") == "released":
            return
        self.tokens.append("dart release")
        if st.get("phase") != "RELEASING":
            self.d.release()
        self.b.cmd("stop", timeout=2.0)
        self.d.wait_phase(("IDLE",), 40, want_seq="released")

    def s_retrieve(self):
        b = self.b
        line = (b.tel or {}).get("line_m")
        if line is not None and line > 1.8:
            if (b.tel or {}).get("drum_pos") != "REEL":
                self.s_flyin()
            m = b.event_mark()
            self.tx("reel", duty=0.6, max_n=30, until="stop")
            t_end = time.time() + 120
            while ((b.tel or {}).get("line_m") or 0) > 1.5 and time.time() < t_end:
                ev = [e for e in b.events(m) if e.get("evt") in ("LIMIT", "FAULT", "ABORTED", "HELLO")]
                if ev:
                    raise RuntimeError(f"reel-in ended: {ev[0]}")
                time.sleep(0.05)
            b.cmd("stop", timeout=2.0)

    def s_unwrap(self):
        t = self.b.tel or {}
        if t.get("drum_pos") in ("WRAPPED", "REEL", "BETWEEN") or (t.get("wraps") or 0) > 0:
            m = self.b.event_mark()
            self.tx("unwrap")
            self.wait("UNWRAPPED", 60, m)

    def s_final(self):
        if not (self.b.tel or {}).get("sw", {}).get("S"):
            self.act("retrieve", 90, "SEATED", duty=0.3)

    def s_end(self):
        self.act("mode", 60, "MODE", pred=lambda e: e.get("mode") == "LAUNCH", to="LAUNCH")

    STEPS = [("LAUNCH_MODE", "s_launch"), ("AIM", "s_aim"), ("COCK", "s_cock"), ("FIRE", "s_fire"), ("FLIGHT", "s_flight"),
             ("LANDING_ZONE", "s_line"), ("WRAP", "s_wrap"), ("ANCHOR", "s_anchor"), ("TIDY", "s_tidy"),
             ("REEL_FLYIN", "s_flyin"), ("TOW", "s_tow"), ("RELEASE", "s_release"), ("RETRIEVE", "s_retrieve"),
             ("UNWRAP", "s_unwrap"), ("FINAL_APPROACH", "s_final"), ("LAUNCH_READY", "s_end")]

    def recover(self, step, err):
        """The operator: wait for the base, clear the fault, decock a carriage left drawn (then re-aim), retry."""
        rig, b = self.rig, self.b
        rig.note("info", msg=f"ref: {step} failed ({err}); recovering")
        t0 = time.time()
        while not b.link().get("connected") and time.time() - t0 < 15:
            time.sleep(0.2)
        time.sleep(0.5)
        for _ in range(3):
            try:
                b.cmd("clear", timeout=2.0)
                break
            except Exception as e:
                rig.note("info", msg=f"ref: clear: {e}")
                time.sleep(1.0)
        time.sleep(0.5)
        try:
            st = b.cmd("status", timeout=2.0)
        except Exception:
            st = {}
        tel = b.tel or {}
        if "LAUNCH_BUSY" in err and st.get("phase") == "UNKNOWN" and (tel.get("sw") or {}).get("H"):
            # carriage home but the phase unknown, drum out of LAUNCH: the only firmware path back to REST is
            # mode LAUNCH, cock (creeps to SW_HOME, re-zeroes, draws the spring), decock (FINDINGS F15)
            rig.note("info", msg="ref: phase UNKNOWN with the carriage home outside LAUNCH: mode LAUNCH, cock, decock")
            self.act("mode", 60, "MODE", pred=lambda e: e.get("mode") == "LAUNCH", to="LAUNCH")
            self.act("cock", 60, "COCKED")
            m = b.event_mark()
            self.tx("decock")
            self.wait("RESET", 90, m)
            self.redo_aim = True
            return
        if st.get("phase") in ("UNKNOWN", "COCKING") and not (tel.get("sw") or {}).get("H"):
            rig.note("info", msg="ref: carriage drawn with the phase unknown: decock")
            m = b.event_mark()
            self.tx("decock")
            self.wait("RESET", 90, m)
            self.redo_aim = True

    def _run(self):
        rig = self.rig
        i = 0
        tries = 0
        try:
            while i < len(self.STEPS):
                name, fn = self.STEPS[i]
                if getattr(self, "redo_aim", False) and name in ("COCK", "FIRE"):
                    self.redo_aim = False
                    self._set("AIM")
                    self.s_aim()
                self._set(name)
                for pst, pcmd, pargs in rig.sc.probes:
                    if pst == name and (pst, pcmd) not in self.probed:
                        self.probed.add((pst, pcmd))
                        try:
                            r = self.b.cmd(pcmd, timeout=2.0, check=False, **pargs)
                        except Exception as e:
                            r = {"ok": None, "err": f"{type(e).__name__}: {e}"}
                        rig.probe_results.append({"step": pst, "cmd": pcmd, "args": pargs, "reply": r})
                        rig.note("info", msg=f"probe at {pst}: {pcmd} {pargs} -> {r}")
                        if r.get("ok"):
                            self.b.cmd("stop", timeout=2.0)      # it started: stop it again
                            time.sleep(1.0)
                try:
                    getattr(self, fn)()
                    i += 1
                    tries = 0
                except Exception as e:
                    tries += 1
                    rig.prompts.append({"step": name, "text": f"{name} failed: {type(e).__name__}: {e}", "choices": ["Retry"], "answer": "Retry"})
                    if tries > rig.sc.retry_max:
                        raise
                    self.recover(name, f"{type(e).__name__}: {e}")
            self.state = "DONE"
            self._set("DONE")
        except Exception as e:
            self.state, self.error = "FAILED", f"{type(e).__name__}: {e}"
            rig.note("info", msg=f"ref driver failed at {self.step}: {self.error}")
            try:
                self.b.safe_stop()
            except Exception:
                pass


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
            r.anchor_at(float(p["anchorTautAt"]) - 0.6)   # rover 0.6 m closer: slack
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
        self.hello_delay = kw.get("hello_delay", 2100)
        self.landing_slack = kw.get("landing_slack", 0.9)
        self.line_ea_n = kw.get("line_ea_n", 20000.0)   # 1 mm Dyneema: about 3.5 % at 667 N
        self.slip_n = kw.get("slip_n", 0.0)
        self.stiff = kw.get("stiff", False)
        self.path_gain = kw.get("path_gain", 1.15)     # D43: capstan-side tension = 1.15 x dart-side (pulleys)          # strict line: 1 mm Dyneema stiffness (else the sim.h 40 N/m spring)             # the anchor (or a dragging dart) slides above this tension
        self.c1_from_last_aim = kw.get("c1_from_last_aim", False)
        self.expect_faults = kw.get("expect_faults", False)
        self.d22 = kw.get("d22", False)
        self.driver = kw.get("driver", "mission")
        self.tow_duty = kw.get("tow_duty", 1.0)
        self.doc_cal = kw.get("doc_cal", False)
        self.probes = kw.get("probes", [])         # ref driver: (step, cmd, args) sent at the start of step; must be refused     # calibrate literally as CALIBRATION.md says (offset left at default)     # BASE_SPEC 10.1 step 13: duty 1  # "mission" = jetson mission.py; "ref" = RefMission (BASE_SPEC 10.1)            # strict D22 plant: laying turns draws line from the dart side   # D22: about 0.9 m of line lies slack after the landing   # ms from power on to HELLO (the sketch's setup())
        self.after = kw.get("after")              # run after the first mission ends (operator recovery + a new mission)
        self.tow_rounds = kw.get("tow_rounds", 3)
        self.timeout_s = kw.get("timeout_s", 420)
        self.policy = kw.get("policy", {})        # substring of prompt text -> answer (first match wins)
        self.retry_max = kw.get("retry_max", 3)
        self.checks = kw.get("checks", []) or nominal_checks()
        self.extra_checks = kw.get("extra_checks", [])
        self.checks = self.checks + self.extra_checks
        self._retries = 0

    def answer(self, rig: Rig, step: str, text: str, choices: List[str]) -> str:
        if "failed" in text.lower() or "FAULT" in text:
            time.sleep(2.0)                   # the operator reads the fault and looks at the base first
        for sub, ans in self.policy.items():
            if sub in text:
                if callable(ans):
                    a = ans(rig, text, choices)
                    if a in choices:
                        return a
                elif ans in choices:
                    return ans
        prefer = ["Continue", "Base may move", "Aim", "Cock", "Decock and re-aim", "Fire", "FIRE NOW", "Anchor", "Anchor anyway", "Shoot again", "Dart is down",
                  "Release the dart"]
        for p in prefer:
            if p in choices:
                return p
        if step != getattr(self, "_retry_step", None):   # the retry budget is per step
            self._retry_step, self._retries = step, 0
        if "Retry" in choices and self._retries < self.retry_max:
            self._retries += 1
            time.sleep(1.0)
            return "Retry"
        if "Retry" not in choices:          # a prompt the script does not know: go on with the first choice that is not Abort
            for c in choices:
                if c not in ("Abort", "Skip"):
                    rig.note("info", msg=f"operator: unknown prompt, chose {c!r}")
                    return c
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
    """The rail's REAL pitch (plant aim phase through the factory table) when AIMED came, against the solution."""
    ev = [e for e in rig.events if e.get("evt") == "AIMED"]
    if not ev:
        return False, "no AIMED"
    e = ev[-1]
    sol = (res.get("solution") or {}).get("pitch_deg")
    pt = (getattr(rig, "aimed_plant", None) or {}).get("pitchTrue")
    ok = bool(e.get("ok")) and sol is not None and isinstance(pt, (int, float)) and abs(pt - sol) <= 1.0
    return ok, f"plant pitch {pt} at AIMED (firmware says {e.get('pitch_deg')}), solution {sol}"


def c_fired(rig, res):
    return res["plant"].get("shots", 0) >= 1 and "FIRED" in res["events"], f"plant shots {res['plant'].get('shots')}"


def c_landed(rig, res):
    return "LANDED" in res["dart_phases"], f"dart phases {res['dart_phases']}"


def c_anchored(rig, res):
    return "ANCHORED" in res["dart_phases"], f"dart phases {res['dart_phases']}"


def c_tidy(rig, res):
    """TIDY ran and the take-up left the dart side taut (plant line out minus the taut length under 0.1 m)."""
    st = res["steps"]
    if "TIDY" not in st:
        return False, "TIDY step missing"
    if not (rig.sc.d22 or os.environ.get("SYSTEST_D22_ALL") == "1" or os.environ.get("SYSTEST_STRICT_ALL") == "1"):
        return True, "TIDY ran (slack not judged: lenient wrap plant leaves the landing slack the wrap would take)"
    nxt = st[st.index("TIDY") + 1] if st.index("TIDY") + 1 < len(st) else None
    p = rig.step_plant.get(nxt) or {}
    if p.get("anchorTautAt") is None or float(p["anchorTautAt"]) < 0:
        return False, f"no anchored snapshot after TIDY ({nxt})"
    slack = float(p["lineOut"]) - float(p["anchorTautAt"])
    return slack <= 0.1, f"dart-side slack after TIDY {slack:.2f} m (at {nxt})"


def c_wrapped(rig, res):
    p = rig.step_plant.get("CHECK") or rig.step_plant.get("TOW") or {}
    return "WRAPPED" in res["events"] and float(p.get("wraps") or 0) >= 3.5, f"plant wraps at CHECK/TOW {p.get('wraps')}"


def c_reel_engaged(rig, res):
    p = rig.step_plant.get("TOW") or {}
    # the TOW snapshot is taken at the step change; the dog must be in REEL whenever the capstan pulls
    return bool(rig.plant_max.get("TOW:tension", 0) > 5) and p.get("drumDeg") is not None, \
        f"max tension in TOW {rig.plant_max.get('TOW:tension')} N"


def c_tow_limit(rig, res):
    """Peak plant line tension over the whole pull (check, fly-in, tow, release: the 50 ms poll can attribute a peak
    to the next step), against the tow limit + 15 %."""
    lim = rig.app.cfg.mission.tow_limit_n
    keys = [k for k in rig.plant_max if k.endswith(":tension") and k.split(":")[0] in
            ("CHECK", "TOW", "FLY_IN", "REEL_FLYIN", "RELEASE", "TIDY", "ANCHOR")]
    mx = max([rig.plant_max[k] for k in keys] or [0.0]) / rig.sc.path_gain       # dart-side (D43)
    worst = max(keys, key=lambda k: rig.plant_max[k]) if keys else "-"
    return mx <= lim * 1.15, f"max dart-side tension {mx:.1f} N (capstan / {rig.sc.path_gain}, at {worst}), limit {lim} N (+15 %)"


def c_released(rig, res):
    return "RELEASING" in res["dart_phases"], f"dart phases {res['dart_phases']}"


def c_reeled(rig, res):
    """10.1 step 15: the capstan reel-in stops with about approach_m left (never seats on the capstan)."""
    p = rig.step_plant.get("UNWRAP") or {}
    r = rig.step_plant.get("RETRIEVE") or {}
    a = rig.app.cfg.mission.approach_m
    lo = p.get("lineOut")
    if lo is None:
        return False, "no UNWRAP snapshot"
    started_short = r.get("lineOut") is not None and float(r["lineOut"]) <= a + 0.3   # nothing to reel on the capstan
    ok = (started_short or a - 0.4 <= float(lo) <= a + 0.3) and not p.get("seated")
    return ok, f"line out {r.get('lineOut')} m at RETRIEVE start, {lo} m at UNWRAP (want {a - 0.4:.1f}..{a + 0.3:.1f}), seated {p.get('seated')}"


def c_seat_neutral(rig, res):
    """10.1 step 17 / D39: the dart seats on the take-up in NEUTRAL with the wraps off, never on the capstan."""
    sr = getattr(rig, "seat_rec", None)
    if not sr:
        return False, "never seated after the shot"
    ok = (not sr.get("inReel")) and (float(sr.get("wraps") or 0) < 0.05 or sr.get("step") == "UNWRAP")
    return ok, f"seated with {sr} (seating by the take-up during the unwrap, collar NEUTRAL, counts as take-up)"


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
            ("seated", c_seated), ("back to LAUNCH", c_launch), ("no dry fire", c_no_dry),
            ("wraps laid without jamming (D22 slack)", c_wrap_free), ("free dart tugged under 1 m by the wrap", c_tug),
            ("C1 command order = BASE_SPEC 10.1 (D35)", c_c1_sequence),
            ("seated on the take-up in NEUTRAL, wraps off (10.1 step 17)", c_seat_neutral)]


def c_no_fault(rig, res):
    f = [e for e in rig.events if e.get("evt") == "FAULT"]
    return not f, f"FAULT events: {[(e.get('code'), e.get('_step')) for e in f]}"


def c_no_retry(rig, res):
    bad = [p for p in res["prompts"] if "failed" in p["text"].lower() or p["answer"] == "Retry"]
    return not bad, f"failure prompts / retries: {[p['text'][:60] for p in bad]}"


import re as _re

C1_SEQ = (r"^(mode LAUNCH > )?aim > (cock > )?arm > fire > set line_m > AIM6(mode NEUTRAL > )?wrap > dart auto > tidy > "
          r"mode REEL > (reel tension > )+dart release > stop > (reel stop > )?unwrap > RETRIEVE(retrieve > )mode LAUNCH$")


def c1_tokens(rig) -> List[str]:
    """The commands the Jetson sent, in order, from its own mission log (base_tx and dart_tx lines)."""
    d = rig.app.run_dir
    out = []
    if not d or not os.path.exists(os.path.join(d, "mission.jsonl")):
        return out
    for l in open(os.path.join(d, "mission.jsonl")):
        try:
            r = json.loads(l)
        except ValueError:
            continue
        o = r.get("obj") or {}
        if r.get("kind") == "base_tx":
            c = o.get("cmd")
            if c in ("ping", "status", "get", "clear", "estop", None):
                continue
            if c == "mode":
                c = f"mode {o.get('to')}"
            elif c == "reel":
                c = f"reel {o.get('until', 'stop')}"
            elif c == "set":
                if o.get("key") != "line_m":
                    continue                     # bench settings (host_timeout_s ...) are not part of 10.1
                c = "set line_m"
            elif c == "cal":
                c = "decock" if o.get("what") == "decock" else f"cal {o.get('what')}"
            out.append(c)
        elif r.get("kind") == "dart_tx":
            if o.get("auto") is True and "arm" not in o:
                out.append("dart auto")
            elif o.get("release"):
                out.append("dart release")
    coll = []
    for t in out:                       # a retry repeats the same command: collapse runs
        if not coll or coll[-1] != t:
            coll.append(t)
    return coll


def c_c1_sequence(rig, res):
    """BASE_SPEC 10.1 (D35): the one C1 command sequence. `stop` counts only right after `dart release` (step 14:
    release, then stop); `retrieve` may only be missing when the unwrap take-up already seated the dart in NEUTRAL."""
    ref = getattr(rig, "ref", None)
    toks = c1_tokens(rig) if ref is None else ["set line_m" if t.startswith("set") else t for t in ref.tokens
                                               if not t.startswith("set") or "line" in t or t == "set"]
    keep = []
    for i, t in enumerate(toks):
        if t == "stop" and not (keep and keep[-1] == "dart release"):
            continue
        if not keep or keep[-1] != t:
            keep.append(t)
    if rig.sc.c1_from_last_aim and "aim" in keep:
        keep = keep[len(keep) - 1 - keep[::-1].index("aim"):]
    seq = " > ".join(keep)
    sr = getattr(rig, "seat_rec", None) or {}
    seated_neutral = sr and not sr.get("inReel") and float(sr.get("wraps") or 0) < 0.05 and sr.get("step") in ("UNWRAP",)
    pat = C1_SEQ.replace("RETRIEVE(retrieve > )", "(retrieve > )?" if seated_neutral else "retrieve > ")
    shot_pitch = (rig.solution or {}).get("pitch_deg") if hasattr(rig, "solution") else None
    pat = pat.replace("AIM6", "aim > " if (shot_pitch or 0) > 15 else "")      # D45: aim 6 before the wrap above 15 deg
    return bool(_re.match(pat, seq)), seq


def c_wrap_free(rig, res):
    n = res["plant"].get("wrapBlocks", -1)
    return n == 0, f"wrap cup blocked for {n} ticks (taut dart side while laying turns)"


def c_tug(rig, res):
    d0, d = getattr(rig, "dart_d0", None), getattr(rig, "dart_d_anchor", None)
    if d0 is None or d is None:
        return False, "no shot"
    return d0 - d < 1.0, f"free dart dragged {d0 - d:.2f} m toward the base before it anchored (line taken by the wrap {res['plant'].get('dartTug', 0):.2f} m)"


# triggers: (id, condition(rig), action(rig))
def at_step(name, delay=0.0):
    st = {}

    def cond(rig):
        if rig.step() == name:
            st.setdefault("t", time.time())
            return time.time() - st["t"] >= delay
        st.pop("t", None)
        return False
    return cond


def plant_state(state, step=None, delay=0.0):
    st = {}

    def cond(rig):
        if rig.plant_last.get("state") == state and (step is None or rig.step() == step):
            st.setdefault("t", time.time())
            return time.time() - st["t"] >= delay
        st.pop("t", None)
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


def decock_and_restart(rig):
    """What the operator does after an aborted mission with the launcher in an unknown state: Clear, Decock (wait
    RESET), then Start again."""
    app = rig.app
    rig.note("info", msg=f"operator: clear -> {app.action('clear')}")
    time.sleep(0.5)
    mark = len(rig.events)
    r = app.action("decock")
    rig.note("info", msg=f"operator: decock -> {r}")
    t0 = time.time()
    while time.time() - t0 < 30 and not any(e.get("evt") in ("RESET", "FAULT", "ABORTED") for e in rig.events[mark:]):
        time.sleep(0.2)
    rig.note("info", msg=f"after decock: {[e.get('evt') for e in rig.events[mark:]]}, tel {app.base.tel}")
    rig.sc._retries = 0
    rig.note("info", msg=f"operator: start again -> {app.action('start')}")


def c_first_aborted_cleanly(rig, res):
    fm = getattr(rig, "first_mission", None)
    return fm is not None, f"first mission: {fm}"



SCENARIOS: Dict[str, Scenario] = {}


def sc(s: Scenario):
    SCENARIOS[s.name] = s
    return s


sc(Scenario("nominal", "whole mission, aim to seated, no faults (strict D22 plant)", d22=True))

sc(Scenario("ref_nominal", "BASE_SPEC 10.1 sequence driven directly (no mission.py): firmware + dart + D22 plant",
            driver="ref", d22=True,
            checks=[c for c in nominal_checks() if c[0] not in ("tidy ran",)] + [("D22 order: WRAP step before ANCHOR step", lambda r, res: c_d22_order(r, res))]))

def c_probes_refused(rig, res):
    pr = rig.probe_results
    bad = [p for p in pr if p["reply"].get("ok") is not False]
    return bool(pr) and not bad and len(pr) == len(rig.sc.probes), [(p["step"], p["cmd"], p["reply"].get("err")) for p in pr]


sc(Scenario("ref_refusals", "BASE_SPEC 10.1 with wrong commands thrown in: retrieve at the WRAPPED park (D39: NEUTRAL with the "
            "wraps off only), tow max_n 80 (D39 cap 70), reel seated on the capstan, wrap while cocked, fire without arm, aim out of range",
            driver="ref", d22=True,
            probes=[("FIRE", "fire", {"token": 123456}), ("COCK", "aim", {"deg": 85}),
                    ("FIRE", "wrap", {}),
                    ("TIDY", "retrieve", {"duty": 0.3}), ("TOW", "reel", {"duty": 1.0, "max_n": 80, "until": "tension"}),
                    ("RETRIEVE", "reel", {"duty": 0.6, "max_n": 30, "until": "seated"})],
            checks=[("every wrong command refused", c_probes_refused), ("mission DONE", c_done),
                    ("tow limit respected", c_tow_limit), ("no dry fire", c_no_dry)]))

sc(Scenario("ref_tow_duty08", "as ref_nominal with the tow at duty 0.8 (the mission's reel_duty): is the tow limit duty dependent?",
            driver="ref", d22=True, tow_duty=0.8,
            checks=[("mission DONE", c_done), ("tow limit respected", c_tow_limit)]))

sc(Scenario("nominal_uncal", "whole mission with the factory k_n_per_a / free_a (CALIBRATION.md not done): the Jetson must "
            "refuse the tow (F5) and bring the dart home", calibrated=False, expect_faults=True,
            policy={"TOW refused": "Release the dart"},
            checks=[("TOW refused without calibration", lambda r, res: (any("refused" in p["text"] and "TOW" in p["text"] for p in res["prompts"]),
                                                                        [p["text"][:70] for p in res["prompts"] if "TOW" in p["text"]])),
                    ("never pulled over the limit", c_tow_limit), ("mission DONE", c_done), ("dart released", c_released),
                    ("seated", c_seated), ("back to LAUNCH", c_launch), ("no dry fire", c_no_dry)]))

sc(Scenario("tow_stiff", "short stiff tow: 3 m of 1 mm Dyneema (EA 20 kN, about 7 kN/m): does the tow stop within the limit?",
            target=(3.0, 0.0), d22=True))

sc(Scenario("ref_tow_stiff", "tow_stiff with the BASE_SPEC 10.1 reference driver, tow at duty 1", target=(3.0, 0.0), d22=True, driver="ref",
            checks=[("mission DONE", c_done), ("tow limit respected", c_tow_limit), ("no dry fire", c_no_dry)]))

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
                      ("load removed", on_evt("FAULT", after=0.3), lambda r: r.base.set(extraLoadNm=0))],
            after=decock_and_restart, timeout_s=400,
            extra_checks=[("first mission ended, operator recovered with Decock", c_first_aborted_cleanly),
                          ("STALL or OVERCURRENT raised", lambda r, res: (any(e.get("code") in ("STALL", "OVERCURRENT") for e in r.events),
                                                                          [e for e in r.events if e.get("evt") == "FAULT"]))]))

sc(Scenario("power_unwrap", "power loss mid-unwrap: kill the base process, restart it, the mission recovers",
            triggers=[("kill base mid-unwrap", plant_state("UNWRAP", delay=3.0), lambda r: r.restart_base(2.0))],
            extra_checks=[("Jetson notices the reset within 10 s (no full 60 s wait)",
                           lambda r, res: (not any("in 60 s" in p["text"] for p in res["prompts"]),
                                           [p["text"][:70] for p in res["prompts"] if "failed" in p["text"]])),
                          ("base restarted and the Jetson saw it", lambda r, res: (getattr(r.base, "kills", 0) >= 1 and open(os.path.join(r.dir, "base.log")).read().count("BOOT #1") >= 2,
                                                                                   f"kills {getattr(r.base, 'kills', 0)}"))]))

sc(Scenario("power_unwrap_fasthello", "power loss mid-unwrap with HELLO sent as soon as the port exists (lost in the pyserial open flush)",
            hello_delay=0,
            triggers=[("kill base mid-unwrap", plant_state("UNWRAP", delay=3.0), lambda r: r.restart_base(2.0))],
            extra_checks=[("Jetson notices the reset within 10 s (no full 60 s wait)",
                           lambda r, res: (not any("in 60 s" in p["text"] for p in res["prompts"]),
                                           [p["text"][:70] for p in res["prompts"] if "failed" in p["text"]]))]))

def unjam_dart_retry(rig, text, choices):
    """Operator frees the dart drills by hand (manual keys / clears the stone), then Retry."""
    rig.dart.get("/sim/set?drill_jam=0")
    rig.anchor_ok = True
    rig.note("info", msg="operator: dart drills freed")
    return "Retry"


sc(Scenario("dart_never_anchors", "the dart's drills are jammed: /auto still says ANCHORED (stall = stone), the dart does not hold; "
            "the check pull must catch it, release (jammed: operator frees it) and reshoot",
            dart_setup=["drill_jam=1"], slip_n=6.0, timeout_s=700, expect_faults=True,
            policy={"did not anchor": "Release and reshoot", "refused auto": "Release and reshoot",
                    "dragging": "Release and reshoot", "did not hold": "Release and reshoot", "slipping": "Release and reshoot",
                    "release did not finish": unjam_dart_retry},
            checks=[("mission DONE (dart retrieved after the failed check)", c_done),
                    ("check pull caught the non-holding anchor", lambda r, res: (any(("dragging" in p["text"] or "did not hold" in p["text"] or "home during the check" in p["text"])
                                                                                 for p in res["prompts"]), [p["text"][:80] for p in res["prompts"] if "failed" in p["text"]])),
                    ("no TOW before the failed check is resolved", lambda r, res: ("TOW" not in res["steps"] or ("RELEASE" in res["steps"] and res["steps"].index("RELEASE") < res["steps"].index("TOW")), res["steps"])),
                    ("seated", c_seated), ("back to LAUNCH", c_launch), ("no dry fire", c_no_dry)]))

sc(Scenario("dart_drags20", "the dart drags at 20 N (above the 15 N check pull): the check passes, the tow must find the dart "
            "coming in at 20 N and never claim the rover is held", dart_setup=[], slip_n=20.0, timeout_s=700, expect_faults=True,
            policy={"dragging": "Release and reshoot", "did not hold": "Release and reshoot", "slipping": "Release and reshoot",
                    "came home": "Release and bring home", "pulled out": "Release and bring home"},
            checks=[("mission ends (not hung)", lambda r, res: (not res["hung"], res["mission_state"])),
                    ("dragging reported to the operator", lambda r, res: (any(k in p["text"].lower() for p in res["prompts"] for k in ("drag", "slip", "pulled out", "came home", "not hold", "limit")),
                                                                          [p["text"][:80] for p in res["prompts"] if "failed" in p["text"].lower()])),
                    ("tow limit respected", c_tow_limit), ("no dry fire", c_no_dry)]))

sc(Scenario("snag_retrieve", "line snags while reeling in: LIMIT, operator frees it, Retry",
            triggers=[("snag", lambda r: r.step() == "RETRIEVE" and float(r.plant_last.get("lineOut") or 0) < 3.0
                       and r.plant_last.get("state") == "REEL",
                       lambda r: (r.anchor_at(float(r.base.get()["lineOut"]) - 0.15),
                                  later(8, lambda r2: r2.base.set(anchorTautAt=-1))(r)))],
            extra_checks=[("snag held under the retrieve limit", lambda r, res: (r.plant_max.get("RETRIEVE:tension", 0) <= r.app.cfg.mission.retrieve_limit_n * 1.2,
                                                                                f"max {r.plant_max.get('RETRIEVE:tension')} N")),
                          ("LIMIT seen in RETRIEVE", lambda r, res: (any(e.get("evt") == "LIMIT" and e.get("_step") == "RETRIEVE" for e in r.events),
                                                                      [e.get("_step") for e in r.events if e.get("evt") == "LIMIT"]))]))

sc(Scenario("no_angle_sensor", "AS5600 magnet missing from boot (D12: SW_INDEX fallback): the mission must run or refuse clearly",
            plant_setup={"encMissing": 1}, timeout_s=200,
            expect_faults=True,
            checks=[("DONE in SW_INDEX mode (D12), or refused at PREFLIGHT before any motion (F9 / F10)",
                     lambda r, res: (c_done(r, res)[0] or (res["steps"][-1:] == ["PREFLIGHT"] and any(k in str(res["error"]).lower() + " ".join(p["text"].lower() for p in res["prompts"]) for k in ("angle sensor", "magnet"))),
                                     f"{res['mission_state']} at {res['steps'][-1:]}: {res['error']}")),
                    ("refusal is explicit (angle sensor / magnet named)", lambda r, res: (any(k in p["text"] for p in res["prompts"] for k in ("ANGLE_SENSOR", "magnet", "AS5600")) or "ANGLE_SENSOR" in str(res["error"]),
                                                                                         [p["text"][:70] for p in res["prompts"]][:1])),
                    ("no dry fire", c_no_dry)]))

sc(Scenario("usb_unplug_tow", "USB unplugged for 3 s while the base holds the tow (board loses power), replugged",
            triggers=[("unplug", lambda r: r.step() == "TOW" and r.plant_last.get("state") in ("HOLD", "REEL"),
                       lambda r: (r.base.ctl("unplug"), later(1.0, lambda r2: setattr(r2, "unplugged_plant", r2.base.get()))(r),
                                  later(3.0, lambda r2: r2.base.ctl("replug"))(r),
                                  later(4.5, lambda r2: setattr(r2, "replug_plant", r2.base.get()))(r)))],
            extra_checks=[("no motion by itself after replug",
                           lambda r, res: (getattr(r, "replug_plant", {}).get("wDir") == 0
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
    if not r.get("ok"):          # the page refuses Decock while the mission runs (F7): Abort, then Decock, then Start
        rig.wants_decock_restart = True
        return "Abort"
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


SCENARIOS["nominal"].checks.append(("D22 order: WRAP step before ANCHOR step", c_d22_order))

POWER_STATES = [("LAUNCH_MODE", "MODE"), ("AIM", "AIM"), ("COCK", "COCK"), ("BASE_NEUTRAL", "MODE"), ("WRAP", "WRAP"),
                ("CHECK", "REEL"), ("TOW", "HOLD"), ("RETRIEVE", "REEL"), ("LAUNCH_READY", "MODE")]
for _step, _pst in POWER_STATES:
    sc(Scenario(f"power_{_step.lower()}", f"power loss in {_step} (base state {_pst}): kill, restart, mission recovers",
                triggers=[(f"kill base in {_step}", plant_state(_pst, step=_step, delay=1.0), lambda r: r.restart_base(2.0))],
                policy={"LOADED_UNKNOWN": decock_then_retry, "PHASE_UNKNOWN": decock_then_retry, "phase UNKNOWN": decock_then_retry, "use Decock": decock_then_retry},
                retry_max=4, plant_setup={"drumDeg": 41} if _step == "LAUNCH_MODE" else {},
                checks=[("mission DONE after the power loss", c_done), ("no dry fire", c_no_dry),
                        ("seated", c_seated), ("back to LAUNCH", c_launch), ("tow limit respected", c_tow_limit)]))


def inject_garbage(rig):
    # the shim's inject command understands \n and \xHH escapes; quotes are plain
    lines = [r'ESP-ROM:esp32s3-20210327\nrst:0x1 (POWERON),boot:0x8 (SPI_FAST_FLASH_BOOT)\n',
             r'{not json at all\n',
             r'{"id":1,"ok":true}\n',                     # a stale duplicate reply to an old id
             r'{"tel":{"state":"IDL', r'E"}}\n',          # one line split in two writes
             r'\xff\xfe\x00garbage\n',                    # bad UTF-8 and a NUL
             r'{"evt":"NEWTHING","t":1}\n',               # an event the Jetson does not know
             r'{"tel":"x' + ' ' * 300 + r'"}\n']          # over 256 bytes
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
            landing_class="water_void", policy={"Landing zone": nogo_policy}, timeout_s=600, c1_from_last_aim=True,
            extra_checks=[("two shots", lambda r, res: (res["plant"].get("shots", 0) == 2, f"shots {res['plant'].get('shots')}"))]))

sc(Scenario("land_short", "the dart lands with only 0.3 m of slack on its side (short of the turn length): the wrap must "
            "tug the free dart, never jam against an anchored one", landing_slack=0.3, d22=True))

sc(Scenario("dart_wifi_drop", "the dart's WiFi station drops right after landing",
            triggers=[("wifi drop", lambda r: "LANDED" in r.dart_phases, lambda r: r.dart.get("/sim/disconnect"))]))

sc(Scenario("brownout_tow", "battery sags under 9.9 V while towing: BROWNOUT, the drum holds (wraps on), battery back, Retry",
            triggers=[("sag", plant_state("REEL", step="TOW", delay=0.5),
                       lambda r: (r.base.set(vbatOverride=9.0), later(3.0, lambda r2: r2.base.set(vbatOverride=-1))(r)))],
            extra_checks=[("BROWNOUT raised", lambda r, res: (any(e.get("code") == "BROWNOUT" for e in r.events),
                                                              [e.get("code") for e in r.events if e.get("evt") == "FAULT"]))]))

sc(Scenario("encoder_glitch", "AS5600 returns a garbage angle every 50 reads for the whole mission",
            plant_setup={"encGlitchEvery": 50}))


# ---------------------------------------------------------------- the same faults along BASE_SPEC 10.1 (reference driver)
import copy as _copy

_REF_STEP = {"BASE_NEUTRAL": "WRAP", "CHECK": "TOW"}     # mission step names that the reference driver does not have
for _n in [n for n in list(SCENARIOS) if n.startswith("power_") or n in (
        "drum_timeout", "dog_tip", "stall_cock", "snag_retrieve", "usb_unplug_tow", "usb_glitch_reel", "serial_garbage",
        "brownout_tow", "encoder_glitch", "land_short", "dart_wifi_drop")]:
    _s = SCENARIOS[_n]
    if _n in ("power_base_neutral", "power_check"):
        continue
    _r = _copy.copy(_s)
    _r.name, _r.desc, _r.driver, _r.d22 = "ref_" + _n, _s.desc + " [BASE_SPEC 10.1 reference driver]", "ref", True
    _r.after = None
    _r.checks = [c for c in _s.checks if c[1] not in (c_tidy,)]
    if not any(c[1] is c_done for c in _r.checks):
        _r.checks = [("mission DONE", c_done)] + _r.checks
    _r.checks = [c for c in _r.checks if c[0] not in ("first mission ended, operator recovered with Decock",
                                                       "Jetson notices the reset within 10 s (no full 60 s wait)")]
    if _n == "drum_timeout":     # 10.1 has no mode NEUTRAL: jam the drum on its way to the WRAPPED park instead
        _r.triggers = [("jam drum", plant_state("WRAP", step="WRAP", delay=2.0), lambda r: r.base.set(drumJam=1)),
                       ("operator frees the drum", on_evt("FAULT", after=1.0), lambda r: r.base.set(drumJam=0))]
    if _n in ("snag_retrieve", "usb_glitch_reel", "power_retrieve"):
        _r.tow_rounds = 1        # leave enough line out for a capstan reel-in
    if _n == "snag_retrieve":
        _r.triggers = [("snag", plant_state("REEL", step="RETRIEVE", delay=0.5),
                        lambda r: (r.anchor_at(float(r.base.get()["lineOut"]) - 0.15),
                                   later(8, lambda r2: r2.base.set(anchorTautAt=-1))(r)))]
    SCENARIOS[_r.name] = _r


for _n in ("nominal", "ref_nominal", "tow_stiff", "ref_tow_stiff", "ref_refusals"):
    SCENARIOS[_n].stiff = True      # strict plant: real Dyneema stiffness (F20 shows up here first)


# ====================================================================================================== main
def build() -> None:
    src = os.path.join(HERE, "base_shim.cpp")
    if os.environ.get("SYSTEST_ROOT"):          # the shim includes ../firmware/base relative to itself: copy it into the tree
        os.makedirs(os.path.join(ROOT, "systest"), exist_ok=True)
        shutil.copy(src, os.path.join(ROOT, "systest", "base_shim.cpp"))
        src = os.path.join(ROOT, "systest", "base_shim.cpp")
    hdrs = [os.path.join(ROOT, "firmware", "base", "grappstar_base", f) for f in os.listdir(os.path.join(ROOT, "firmware", "base", "grappstar_base"))]
    hdrs.append(os.path.join(ROOT, "firmware", "base", "test", "sim.h"))
    if not os.path.exists(SHIM) or any(os.path.getmtime(h) > os.path.getmtime(SHIM) for h in hdrs + [src]):
        os.makedirs(os.path.dirname(SHIM), exist_ok=True)
        tmp = SHIM + f".tmp{os.getpid()}"          # build aside, then rename: a running shim keeps its old binary
        subprocess.check_call(["g++", "-std=c++17", "-O2", "-Wno-unused-parameter", "-Wno-misleading-indentation",
                               "-o", tmp, src])
        os.replace(tmp, SHIM)
    subprocess.check_call(["make", "-s", "-C", os.path.dirname(DART_SIM)], stdout=subprocess.DEVNULL)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="nominal")
    ap.add_argument("--speed", type=float, default=float(os.environ.get("SYSTEST_SPEED", "2")))
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--shard", default="", help="i/n: run every n-th scenario starting at i (parallel batches)")
    a = ap.parse_args()
    if a.list:
        for s in SCENARIOS.values():
            print(f"{s.name:20s} {s.desc}")
        return 0
    build()
    names = list(SCENARIOS) if a.scenario == "all" else a.scenario.split(",")
    tag = ""
    if a.shard:
        i, n = (int(x) for x in a.shard.split("/"))
        names, tag = names[i::n], f"_shard{i}"
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
    with open(os.path.join(RUNS, f"results{tag}.json"), "w") as f:
        json.dump([{k: r.get(k) for k in ("scenario", "pass", "mission_state", "error", "duration_s", "checks", "steps")}
                   for r in results], f, indent=1, default=str)
    print("\nSUMMARY")
    for r in results:
        bad = [c["check"] for c in r.get("checks", []) if not c["ok"]]
        print(f"  {'PASS' if r.get('pass') else 'FAIL'}  {r['scenario']:20s} {r.get('mission_state')}  {'failed: ' + ', '.join(bad) if bad else ''}")
    return 0 if all(r.get("pass") for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
