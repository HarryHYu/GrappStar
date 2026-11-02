"""GrappStar mission: the whole shot from preflight to NEUTRAL, with operator confirmations and fault handling.

Steps (normal path):
  PREFLIGHT     base link, no fault, dart seated, battery, dart reachable, both videos
  LAUNCH_MODE   base drum to LAUNCH
  AIM           ballistics pitch for the target range; operator turns the rover (the base has no yaw); base aims
  ARM           arm dart landing detection, start recording
  FIRE          double confirmation, then base arm (token) and fire
  FLIGHT        wait for the landing: dart phase, line out stopped, dart video still (any two, or one plus time)
  LANDING_ZONE  flight analysis, terrain verdict, aerial map; operator chooses: anchor, or retrieve and reshoot
  ANCHOR        dart /auto, wait ANCHORED
  REEL_MODE     base drum to REEL
  WRAP          wrap the capstan
  CHECK         test pull at a low tension: the line must stop coming in (the dart holds)
  TOW           reel to the tension limit, hold, rover drives (hook or operator), repeat until the rover is free
  RELEASE       dart /release, wait for IDLE
  RETRIEVE      reel until the dart is seated
  UNWRAP        unwrap the capstan
  NEUTRAL       base drum to NEUTRAL
  DONE

Reshoot path (NO GO or a failed anchor): REEL_MODE, WRAP, RETRIEVE, UNWRAP, then back to LAUNCH_MODE.
Any error: the base is stopped (the firmware itself goes NEUTRAL on a fault) and the operator chooses Retry, Skip (only
where safe) or Abort. STOP on the page aborts at once.
"""
from __future__ import annotations

import os
import threading
import time
import traceback
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

from .comms.base import BaseDisconnected, BaseError, BaseTimeout
from .comms.dart import DartError
from .rover import OperatorRover, RoverDrive

STEPS = ["PREFLIGHT", "LAUNCH_MODE", "AIM", "ARM", "FIRE", "FLIGHT", "LANDING_ZONE", "ANCHOR", "REEL_MODE", "WRAP",
         "CHECK", "TOW", "RELEASE", "RETRIEVE", "UNWRAP", "NEUTRAL", "DONE"]
RESHOOT_STEPS = ["REEL_MODE", "WRAP", "RETRIEVE", "UNWRAP"]
SKIPPABLE = {"CHECK", "LANDING_ZONE"}


class Aborted(Exception):
    """The operator pressed STOP or chose Abort."""


@dataclass
class MissionConfig:
    drag_n: float = 0.75            # payout brake setting (C3 column)
    tow_limit_n: float = 40.0       # line tension limit while towing (base firmware caps it too)
    check_n: float = 15.0           # anchored check pull
    check_max_line_m: float = 0.15  # line that may come in during the check once taut
    retrieve_limit_n: float = 30.0  # snag limit while retrieving
    reel_duty: float = 0.8
    retrieve_duty: float = 0.6
    vbat_min: float = 10.8          # 3S: 3.6 V per cell under light load
    mode_timeout_s: float = 15.0
    aim_timeout_s: float = 20.0
    fire_timeout_s: float = 12.0    # cocking plus release
    land_timeout_s: float = 10.0
    still_s: float = 1.0            # line out unchanged this long = landed
    settle_s: float = 2.0           # wait after landing before analysing (dart video settles)
    anchor_timeout_s: float = 60.0
    release_timeout_s: float = 40.0
    retrieve_speed_mps: float = 0.2  # slowest expected line speed, sets the retrieve timeout
    tow_max_rounds: int = 30
    auto_anchor_on_landing: bool = False   # True: press A so the dart anchors by itself (skips the verdict)
    require_dart_video: bool = False       # True: preflight refuses without a dart video signal
    max_reshoots: int = 3
    pre_s: float = 2.0              # dart video kept from before the shot


@dataclass
class Prompt:
    id: str
    text: str
    choices: List[str]
    danger: bool = False
    step: str = ""
    t: float = field(default_factory=time.time)
    answer: Optional[str] = None

    def to_dict(self) -> dict:
        return {"id": self.id, "text": self.text, "choices": self.choices, "danger": self.danger, "step": self.step}


class Mission:
    """Runs in its own thread. The App feeds it operator answers (answer()) and STOP (abort())."""

    def __init__(self, base, dart, cams: Dict[str, Any], ballistics, log, cfg: Optional[MissionConfig] = None,
                 rover: Optional[RoverDrive] = None, get_target: Optional[Callable[[], Optional[dict]]] = None,
                 run_dir: Optional[str] = None, analyse: Optional[Callable[..., dict]] = None,
                 on_results: Optional[Callable[[dict], None]] = None):
        self.base, self.dart, self.cams, self.bal, self.log = base, dart, cams, ballistics, log
        self.cfg = cfg or MissionConfig()
        self.rover = rover or OperatorRover()
        self.get_target = get_target or (lambda: None)
        self.run_dir = run_dir
        if analyse is None:
            from .analysis import run_post_flight
            analyse = run_post_flight
        self.analyse = analyse
        self.on_results = on_results or (lambda r: None)
        self.state = "IDLE"            # IDLE, RUNNING, DONE, ABORTED, FAILED
        self.step = ""
        self.history: List[dict] = []
        self.error = ""
        self.prompt: Optional[Prompt] = None
        self._pcv = threading.Condition()
        self._pid = 0
        self.cancel = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.results: Dict[str, Any] = {}
        self.shot: Dict[str, Any] = {}
        self.reshoots = 0
        self.dart_anchored = False

    # ------------------------------------------------ public
    def start(self) -> None:
        if self.thread and self.thread.is_alive():
            raise RuntimeError("mission already running")
        self.cancel.clear()
        self.state = "RUNNING"
        self.thread = threading.Thread(target=self._run, name="mission", daemon=True)
        self.thread.start()

    def running(self) -> bool:
        return bool(self.thread and self.thread.is_alive())

    def abort(self, why: str = "operator STOP") -> None:
        if not self.running():
            return
        self.log.warn(f"abort requested: {why}")
        self.cancel.set()
        with self._pcv:
            self._pcv.notify_all()

    def answer(self, prompt_id: str, choice: str) -> bool:
        with self._pcv:
            p = self.prompt
            if p is None or p.id != prompt_id or choice not in p.choices:
                return False
            p.answer = choice
            self._pcv.notify_all()
        return True

    def status(self) -> dict:
        p = self.prompt
        return {"state": self.state, "step": self.step, "steps": STEPS, "history": self.history[-40:],
                "prompt": p.to_dict() if p and p.answer is None else None, "error": self.error,
                "shot": self.shot, "reshoots": self.reshoots, "run_dir": self.run_dir, "rover": self.rover.name}

    # ------------------------------------------------ helpers used by steps and the rover hook
    def ask(self, text: str, choices: List[str], danger: bool = False, timeout: Optional[float] = None) -> str:
        """Show a prompt and block until the operator answers. STOP raises Aborted."""
        with self._pcv:
            self._pid += 1
            p = Prompt(f"p{self._pid}", text, list(choices), danger, self.step)
            self.prompt = p
        self.log.write("prompt", id=p.id, text=text, choices=choices, danger=danger, step=self.step)
        t_end = None if timeout is None else time.time() + timeout
        with self._pcv:
            while p.answer is None:
                if self.cancel.is_set():
                    self.prompt = None
                    raise Aborted("operator STOP")
                if t_end is not None and time.time() > t_end:
                    self.prompt = None
                    raise TimeoutError("no answer")
                self._pcv.wait(0.2)
            self.prompt = None
        self.log.write("answer", id=p.id, choice=p.answer, msg=f"{p.answer} ({text[:60]})")
        if p.answer == "Abort":
            raise Aborted("operator chose Abort")
        return p.answer

    def check_cancel(self) -> None:
        if self.cancel.is_set():
            raise Aborted("operator STOP")

    def sleep(self, s: float) -> None:
        if self.cancel.wait(s):
            raise Aborted("operator STOP")

    def tel(self) -> dict:
        return self.base.tel or {}

    def wait_evt(self, names, timeout: float, since: int, pred=None) -> dict:
        """Base event wait that STOP can interrupt."""
        t_end = time.monotonic() + timeout
        while True:
            self.check_cancel()
            left = t_end - time.monotonic()
            if left <= 0:
                raise BaseTimeout(f"no {names} event from the base in {timeout:.0f} s")
            try:
                return self.base.wait_event(names, min(0.25, left), since=since, pred=pred)
            except BaseTimeout:
                continue

    def wait_tel(self, pred, timeout: float, what: str) -> dict:
        t_end = time.monotonic() + timeout
        while True:
            self.check_cancel()
            left = t_end - time.monotonic()
            if left <= 0:
                raise BaseTimeout(f"base: timed out waiting for {what}")
            try:
                return self.base.wait_tel(pred, min(0.25, left), what)
            except BaseTimeout:
                continue

    def base_mode(self, to: str) -> None:
        t = self.tel()
        if t.get("mode") == to and t.get("drum_pos") == to and t.get("state") in ("IDLE", "HOLD"):
            return
        m = self.base.event_mark()
        self.base.mode(to)
        self.wait_evt("MODE", self.cfg.mode_timeout_s, m, pred=lambda e: e.get("mode", to) == to)
        self.wait_tel(lambda t: t.get("drum_pos") == to, 3.0, f"drum at {to}")

    # ------------------------------------------------ run
    def _set_step(self, name: str) -> None:
        self.step = name
        self.history.append({"state": name, "t": round(time.time(), 2)})
        self.log.write("step", name=name, msg=name)

    def _run(self) -> None:
        try:
            self._sequence()
            self.state = "DONE"
            self._set_step("DONE")
        except Aborted as e:
            self.state = "ABORTED"
            self.error = str(e)
            self.log.warn(f"mission aborted at {self.step}: {e}")
            self._safe_stop()
        except Exception as e:  # a bug: never leave motors running
            self.state = "FAILED"
            self.error = f"{type(e).__name__}: {e}"
            self.log.error(f"mission failed at {self.step}: {self.error}", trace=traceback.format_exc(limit=5))
            self._safe_stop()
        finally:
            self.prompt = None
            for c in self.cams.values():
                try:
                    if c is not None and c.status().get("recording"):
                        c.stop_recording()
                except Exception:
                    pass

    def _safe_stop(self) -> None:
        ok_b = self.base.safe_stop()
        ok_d = True if not self.dart_motion_possible() else self.dart.safe_stop()
        self.log.write("info", msg=f"safe stop: base {'ok' if ok_b else 'NO REPLY'}, dart {'ok' if ok_d else 'NO REPLY'}")

    def dart_motion_possible(self) -> bool:
        s = self.dart.last or {}
        return s.get("phase") in ("ANCHORING", "RELEASING") or bool(s.get("current_mA", 0) > 200)

    def _do(self, name: str, fn: Callable[[], Any]) -> Any:
        """Run one step with fault handling: Retry, Skip (where safe) or Abort."""
        while True:
            self._set_step(name)
            try:
                return fn()
            except Aborted:
                raise
            except (BaseError, BaseTimeout, BaseDisconnected, DartError, TimeoutError, StepFailed) as e:
                msg = f"{name} failed: {e}"
                self.error = msg
                self.log.error(msg)
                self.base.safe_stop() if not isinstance(e, BaseDisconnected) else None
                if isinstance(e, DartError) and self.dart_motion_possible():
                    self.dart.safe_stop()
                choices = ["Retry", "Abort"]
                if name in SKIPPABLE:
                    choices.insert(1, "Skip")
                extra = getattr(e, "choices", None)
                if extra:
                    choices = list(extra) + ["Abort"]
                if isinstance(e, BaseError) and e.err in ("STALL", "OVERCURRENT", "DRUM_TIMEOUT", "DRUM_SENSOR",
                                                           "ANGLE_SENSOR", "SWITCH_DISAGREE", "BROWNOUT", "BAD_STATE",
                                                           "FAULT"):
                    msg += ". The base is in FAULT and went to NEUTRAL. Check it, then Retry clears the fault."
                c = self.ask(msg, choices, danger=True)
                if c == "Skip":
                    self.log.warn(f"{name} skipped by the operator")
                    return None
                if c == "Retry":
                    self._clear_fault()
                    continue
                return c   # a custom choice from StepFailed.choices

    def _clear_fault(self) -> None:
        try:
            t = self.tel()
            if t.get("fault") or t.get("state") == "FAULT":
                self.base.clear()
                self.wait_tel(lambda t: not t.get("fault"), 3.0, "fault cleared")
        except (BaseError, BaseTimeout, BaseDisconnected) as e:
            self.log.warn(f"clear failed: {e}")

    def _sequence(self) -> None:
        self._do("PREFLIGHT", self.preflight)
        while True:
            self._do("LAUNCH_MODE", lambda: self.base_mode("LAUNCH"))
            self._do("AIM", self.aim)
            self._do("ARM", self.arm)
            self._do("FIRE", self.fire)
            self._do("FLIGHT", self.wait_landing)
            choice = self._do("LANDING_ZONE", self.landing_zone)
            if choice == "reshoot":
                self.reshoot()
                continue
            r = self._do("ANCHOR", self.anchor)
            if r == "Release and reshoot":
                self.release_dart()
                self.reshoot()
                continue
            break
        self._do("REEL_MODE", lambda: self.base_mode("REEL"))
        self._do("WRAP", self.wrap)
        r = self._do("CHECK", self.anchored_check)
        if r == "Release and reshoot":
            self._do("RELEASE", self.release_dart)
            self._do("RETRIEVE", self.retrieve)
            self._do("UNWRAP", self.unwrap)
            self.reshoots += 1
            self._do("NEUTRAL", lambda: self.base_mode("NEUTRAL"))
            self.log.info("dart retrieved after a failed anchor check; start a new mission to shoot again")
            return
        self._do("TOW", self.tow)
        self._do("RELEASE", self.release_dart)
        self._do("RETRIEVE", self.retrieve)
        self._do("UNWRAP", self.unwrap)
        self._do("NEUTRAL", lambda: self.base_mode("NEUTRAL"))

    # ------------------------------------------------ steps
    def preflight(self) -> None:
        problems, warnings = [], []
        try:
            r = self.base.ping(timeout=1.5)
            self.log.info(f"base {r.get('fw', 'firmware ?')}")
            t = self.base.status().get("tel") or self.tel()
        except (BaseError, BaseTimeout, BaseDisconnected) as e:
            raise StepFailed(f"base not answering: {e}")
        if t.get("fault"):
            raise StepFailed(f"base in FAULT {t['fault']}: clear it first")
        if t.get("drum_pos") in ("UNKNOWN", "BETWEEN"):
            problems.append(f"drum position {t.get('drum_pos')} (run cal or jog the drum)")
        if (t.get("wraps") or 0) > 0.05:
            problems.append("capstan still wrapped (unwrap first)")
        sw = t.get("sw") or {}
        if sw.get("seated") is False:
            problems.append("dart not seated on the launcher")
        vb = t.get("vbat")
        if vb is not None and vb < self.cfg.vbat_min:
            problems.append(f"battery low {vb:.1f} V")
        try:
            ds = self.dart.status()
            if ds["phase"] not in ("IDLE", "ARMED"):
                problems.append(f"dart busy ({ds['phase']}: {ds.get('event')})")
            rv = ds.get("rail_V")
            if isinstance(rv, (int, float)) and rv < 6.4:
                warnings.append(f"dart battery low ({rv:.2f} V)")
        except DartError as e:
            problems.append(f"dart unreachable: {e}")
        for name in ("dart", "rover"):
            c = self.cams.get(name)
            st = c.status() if c is not None else {"connected": False}
            if not st.get("connected"):
                (problems if (name == "dart" and self.cfg.require_dart_video) else warnings).append(
                    f"{name} video not connected")
            elif name == "dart" and (st.get("bad_frac") or 0) > 0.5:
                warnings.append("dart video mostly bad (check RC832 channel and antennas)")
        tgt = self.get_target()
        if not tgt or not tgt.get("range_m"):
            problems.append("no target: tap the target on the rover view or enter a range")
        if problems:
            raise StepFailed("; ".join(problems))
        if warnings:
            self.ask("Preflight warnings: " + "; ".join(warnings) + ". Continue?", ["Continue", "Abort"])
        self.log.info("preflight ok")

    def aim(self) -> None:
        tgt = self.get_target() or {}
        rng = float(tgt.get("range_m") or 0)
        sol = self.bal.solve(rng, self.cfg.drag_n)
        self.shot = {"range_m": rng, "bearing_deg": tgt.get("bearing_deg"), "solution": sol.to_dict()}
        if not sol.reachable:
            raise StepFailed(f"target {rng:.1f} m not reachable ({sol.note}). Move the rover or pick another target.")
        b = tgt.get("bearing_deg")
        turn = ""
        if b is not None and abs(b) >= 1.0:
            turn = f"Turn the rover {abs(b):.0f} degrees {'right' if b > 0 else 'left'} first. "
        self.ask(f"{turn}Target {rng:.1f} m: pitch {sol.pitch_deg:.1f} deg, flight {sol.t_flight_s:.2f} s, "
                 f"line out about {sol.line_out_m:.1f} m. Aim now?", ["Aim", "Abort"])
        m = self.base.event_mark()
        self.base.aim(sol.pitch_deg)
        e = self.wait_evt("AIMED", self.cfg.aim_timeout_s, m)
        self.shot["pitch_deg"] = e.get("deg", sol.pitch_deg)
        self.log.info(f"aimed at {self.shot['pitch_deg']} deg")

    def arm(self) -> None:
        if self.cfg.auto_anchor_on_landing:
            if not self.dart.arm_landing():
                raise StepFailed("dart did not arm")
        try:
            self.shot["dart_phase_at_arm"] = self.dart.status()["phase"]
        except DartError as e:
            self.log.warn(f"dart not answering at arm: {e} (landing will be detected from the line and video)")
        if self.run_dir:
            for name, c in self.cams.items():
                if c is None:
                    continue
                try:
                    c.start_recording(os.path.join(self.run_dir, f"{name}_shot{self.reshoots}.avi"))
                except Exception as e:
                    self.log.warn(f"{name} recording failed: {e}")

    def fire(self) -> None:
        p = self.shot.get("pitch_deg")
        self.ask(f"FIRE at pitch {p} deg toward {self.shot.get('range_m', 0):.1f} m? Area clear, line free, "
                 "everyone behind the launcher?", ["Fire", "Abort"], danger=True)
        self.ask("Second confirmation: FIRE NOW?", ["FIRE NOW", "Abort"], danger=True)
        self.check_cancel()
        token = self.base.arm()
        m = self.base.event_mark()
        self.base.fire(token)
        e = self.wait_evt("FIRED", self.cfg.fire_timeout_s, m)
        self.shot["t_fire"] = e.get("_t", time.time())
        self.log.info("FIRED")

    def wait_landing(self) -> None:
        """Landed when the dart says so, or the line has stopped paying out (and the dart video is still)."""
        t_fire = self.shot.get("t_fire", time.time())
        t_end = time.time() + self.cfg.land_timeout_s
        last_line, line_t = None, time.time()
        sources = set()
        # a dart still showing LANDED from an earlier shot only counts after it has been seen in FLIGHT again
        stale = self.shot.get("dart_phase_at_arm") in ("LANDED",)
        seen_flight = False
        while time.time() < t_end:
            self.check_cancel()
            t = self.tel()
            line = t.get("line_m")
            if line is not None:
                if last_line is None or abs(line - last_line) > 0.02:
                    last_line, line_t = line, time.time()
                elif line > 1.0 and time.time() - line_t >= self.cfg.still_s:
                    sources.add("line")
            try:
                ds = self.dart.status()
                if ds["phase"] == "FLIGHT":
                    seen_flight = True
                if ds["phase"] in ("LANDED", "ANCHORING", "ANCHORED") and (seen_flight or not stale):
                    sources.add("dart")
            except DartError:
                pass
            if self._video_still(t_fire):
                sources.add("video")
            if "dart" in sources or len(sources) >= 2 or ("line" in sources and time.time() - t_fire > 3.0):
                break
            self.sleep(0.1)
        self.shot["landing_sources"] = sorted(sources)
        self.shot["line_out_m"] = self.tel().get("line_m")
        if not sources:
            c = self.ask("Landing not detected (no dart signal, line still moving or no data). Is the dart down?",
                         ["Dart is down", "Abort"], danger=True)
            self.shot["landing_sources"] = ["operator"]
        self.log.info(f"landed ({', '.join(self.shot['landing_sources'])}), line out {self.shot['line_out_m']} m")
        self.sleep(self.cfg.settle_s)

    def _video_still(self, t_fire: float) -> bool:
        c = self.cams.get("dart")
        if c is None:
            return False
        try:
            fr = [f for f in c.buffer(1.0) if f.t > t_fire + 0.8 and not f.bad]
        except Exception:
            return False
        if len(fr) < 8:
            return False
        import numpy as np
        a = [np.asarray(f.img[::8, ::8], dtype=np.float32).mean(axis=2) for f in fr[-8:]]
        diffs = [float(np.abs(a[i + 1] - a[i]).mean()) for i in range(len(a) - 1)]
        return max(diffs) < 4.0

    def landing_zone(self) -> str:
        t_fire = self.shot.get("t_fire", time.time())
        frames = []
        c = self.cams.get("dart")
        if c is not None:
            try:
                frames = [f for f in c.buffer() if f.t >= t_fire - self.cfg.pre_s]
            except Exception as e:
                self.log.warn(f"no dart frames: {e}")
        fl = None
        try:
            fl = self.dart.flightlog()
            if self.run_dir:
                import json
                with open(os.path.join(self.run_dir, f"flightlog_shot{self.reshoots}.json"), "w") as f:
                    json.dump(fl, f)
        except DartError as e:
            self.log.warn(f"no flightlog: {e}")
        for name, cam in self.cams.items():
            if cam is not None and name == "dart":
                try:
                    p = cam.stop_recording()
                    if p:
                        self.log.write("file", name=f"{name} video", path=p)
                except Exception:
                    pass
        self.log.info(f"analysing {len(frames)} dart frames")
        res = self.analyse(frames, fl, self.run_dir, t_fire=t_fire, log=self.log.warn)
        self.results = res
        self.on_results(res)
        self.log.write("result", msg="landing zone", terrain=res.get("terrain"), flight=res.get("flight"),
                       mosaic=res.get("mosaic"), errors=res.get("errors"))
        tr = res.get("terrain") or {}
        verdict = tr.get("verdict", "UNSURE")
        cls = tr.get("cls", "unknown")
        score = tr.get("score", 0)
        reasons = " ".join(tr.get("reasons", [])[:4])
        head = f"Landing zone: {cls.replace('_', ' ')}, score {score}/100, verdict {verdict.replace('_', ' ')}. {reasons}"
        if verdict == "GO":
            choices = ["Anchor", "Retrieve and reshoot", "Abort"]
        else:
            choices = ["Retrieve and reshoot", "Anchor anyway", "Abort"]
        c = self.ask(head, choices, danger=(verdict != "GO"))
        if c == "Retrieve and reshoot":
            return "reshoot"
        return "anchor"

    def anchor(self) -> Optional[str]:
        s = self.dart.status()
        if s["phase"] == "ANCHORED":
            self.dart_anchored = True
            return None
        if s["phase"] not in ("ANCHORING",):
            self.dart.auto()
        try:
            s = self.dart.wait_phase("ANCHORED", self.cfg.anchor_timeout_s, cancel=self.cancel,
                                     log=self.log.info)
        except DartError as e:
            if "cancelled" in str(e):
                raise Aborted("operator STOP")
            raise StepFailed(f"dart did not anchor: {e}", choices=["Retry", "Release and reshoot"])
        self.dart_anchored = True
        self.log.info("dart anchored")
        return None

    def wrap(self) -> None:
        t = self.tel()
        if t.get("drum_pos") == "WRAPPED":
            return
        m = self.base.event_mark()
        self.base.wrap()
        self.wait_evt("WRAPPED", self.cfg.mode_timeout_s, m)

    def unwrap(self) -> None:
        t = self.tel()
        if (t.get("wraps") or 0) < 0.05 and t.get("drum_pos") == "REEL":
            return
        m = self.base.event_mark()
        self.base.unwrap()
        self.wait_evt("UNWRAPPED", self.cfg.mode_timeout_s, m)

    def anchored_check(self) -> Optional[str]:
        """Pull to check_n. If the line stops coming in, the dart holds. If it keeps coming in at the check tension,
        the dart is dragging."""
        line0 = self.tel().get("line_m") or 0.0
        m = self.base.event_mark()
        self.base.reel(self.cfg.reel_duty * 0.6, max_n=self.cfg.check_n, until="tension")
        try:
            e = self.wait_evt(("LIMIT", "SEATED"), 30.0, m)
        except BaseTimeout:
            self.base.safe_stop()
            raise StepFailed("check pull never reached the test tension: the dart may be dragging",
                             choices=["Retry", "Release and reshoot"])
        if e.get("evt") == "SEATED":
            raise StepFailed("the dart came all the way home during the check (it did not hold)",
                             choices=["Release and reshoot"])
        line1 = self.tel().get("line_m") or 0.0
        self.sleep(1.0)
        line2 = self.tel().get("line_m") or 0.0
        t = self.tel()
        held = (line1 - line2) < self.cfg.check_max_line_m
        self.shot["check"] = {"line_in_m": round(line0 - line2, 3), "tension_n": t.get("tension_n"), "held": held}
        if not held:
            raise StepFailed("line still creeping in at the check tension: the anchor is slipping",
                             choices=["Retry", "Release and reshoot"])
        self.log.info(f"anchor holds at {t.get('tension_n')} N")
        return None

    def tow(self) -> None:
        rounds = 0
        while rounds < self.cfg.tow_max_rounds:
            rounds += 1
            self.check_cancel()
            m = self.base.event_mark()
            self.base.reel(self.cfg.reel_duty, max_n=self.cfg.tow_limit_n, until="tension")
            e = self.wait_evt(("LIMIT", "SEATED"), 60.0, m)
            if e.get("evt") == "SEATED":
                raise StepFailed("the dart came home during the tow: the anchor pulled out",
                                 choices=["Retry"])
            tn = e.get("tension_n", self.tel().get("tension_n"))
            self.log.info(f"tow round {rounds}: holding at {tn} N, line {self.tel().get('line_m')} m")
            r = self.rover.tow_assist(self, tn)
            if r == "done":
                self.log.info("rover free")
                return
            if r == "abort":
                raise Aborted("operator chose Abort")
        self.log.warn("tow round limit reached")

    def release_dart(self) -> None:
        try:
            s = self.dart.status()
        except DartError as e:
            raise StepFailed(f"dart unreachable for release: {e}")
        if s["phase"] in ("IDLE", "LANDED", "ARMED") and not self.dart_anchored:
            return
        self.base.safe_stop()   # hold the line, never reel against a releasing dart
        self.dart.release()
        try:
            self.dart.wait_phase(("IDLE",), self.cfg.release_timeout_s, cancel=self.cancel, log=self.log.info)
        except DartError as e:
            if "cancelled" in str(e):
                raise Aborted("operator STOP")
            raise StepFailed(f"dart release did not finish: {e}. Use the manual dart keys, then Retry.")
        self.dart_anchored = False
        self.log.info("dart released")

    def retrieve(self) -> None:
        t = self.tel()
        if (t.get("sw") or {}).get("seated"):
            return
        line = t.get("line_m") or 10.0
        timeout = 20.0 + line / self.cfg.retrieve_speed_mps
        m = self.base.event_mark()
        self.base.reel(self.cfg.retrieve_duty, max_n=self.cfg.retrieve_limit_n, until="seated")
        e = self.wait_evt(("SEATED", "LIMIT"), timeout, m)
        if e.get("evt") == "LIMIT":
            raise StepFailed(f"snag while retrieving ({e.get('tension_n')} N at {e.get('line_m')} m). "
                             "Free the line or jog, then Retry.")
        self.log.info("dart seated")

    def reshoot(self) -> None:
        self.reshoots += 1
        if self.reshoots > self.cfg.max_reshoots:
            raise Aborted("too many reshoots")
        self.log.info(f"retrieving for reshoot {self.reshoots}")
        self._do("REEL_MODE", lambda: self.base_mode("REEL"))
        self._do("WRAP", self.wrap)
        self._do("RETRIEVE", self.retrieve)
        self._do("UNWRAP", self.unwrap)
        tgt = self.get_target()
        self.ask("Dart is home. Check the dart and the line, set the new target if needed, then shoot again.",
                 ["Shoot again", "Abort"])


class StepFailed(Exception):
    """A step could not complete; choices (optional) replace Retry in the operator prompt."""

    def __init__(self, msg: str, choices: Optional[List[str]] = None):
        super().__init__(msg)
        self.choices = choices
