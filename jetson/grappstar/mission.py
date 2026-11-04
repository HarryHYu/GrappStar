"""GrappStar mission: the whole shot from preflight back to LAUNCH, with operator confirmations and fault handling.

Follows firmware/base/PROTOCOL.md (C1 final) and the coordinator's binding sequence (3 Nov):
  PREFLIGHT       base link, no fault, dart seated, battery, dart reachable, both videos, a target
  LAUNCH_MODE     base drum to LAUNCH
  AIM             ballistics pitch for the target range; operator turns the rover (no yaw axis); base aims
                  (aim only with the carriage home, so always before cocking)
  COCK            base cocks the spring (COCKED); optional, fire from rest also works
  ARM             arm the dart landing detector (log only: the mission anchors after the verdict), start recording
  FIRE            double confirmation, then base arm (token) and fire (FIRED)
  FLIGHT          wait for the landing: dart phase LANDED, or the dart video goes still
  LANDING_ZONE    flight analysis, terrain verdict, aerial map; tell the base the line out (set line_m);
                  operator chooses: anchor, or retrieve and reshoot
  BASE_NEUTRAL    wait for the launcher RESET, then drum to NEUTRAL
  WRAP            capstan wraps on at once, while the line is still slack (D22; on firmware that needs it,
                  mode REEL first)
  ANCHOR          dart /auto, wait ANCHORED
  TIDY            base winds the slack onto the storage reel (only if the firmware has "tidy")
  CHECK           test pull at a low tension: it must reach the tension and hold it
  TOW             reel to the tension limit, hold, rover drives (hook or operator), repeat until free
  RELEASE         base stop (hold), dart /release, wait for seq "released"
  RETRIEVE        reel in on the capstan until about approach_m of line is left (or until seated on firmware
                  without "retrieve")
  UNWRAP          capstan wraps off
  FINAL_APPROACH  take-up until the dart is seated ("retrieve"; skipped when RETRIEVE already seated it)
  LAUNCH_READY    drum back to LAUNCH for the next shot
  DONE

Reshoot path (NO GO or a failed anchor): RELEASE if needed, BASE_NEUTRAL, WRAP, RETRIEVE, UNWRAP, FINAL_APPROACH,
then back to LAUNCH_MODE. Any error: the base is stopped (the firmware itself goes NEUTRAL on a fault when safe) and the
operator chooses Retry, Skip (only where safe) or Abort. STOP on the page aborts at once.
"""
from __future__ import annotations

import os
import re
import threading
import time
import traceback
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

from .comms.base import BaseDisconnected, BaseError, BaseTimeout
from .comms.dart import DartError, DartRefused, remap_flightlog
from .rover import OperatorRover, RoverDrive

STEPS = ["PREFLIGHT", "LAUNCH_MODE", "AIM", "COCK", "ARM", "FIRE", "FLIGHT", "LANDING_ZONE", "RAIL_LOW", "BASE_NEUTRAL",
         "WRAP",
         "ANCHOR", "TIDY", "FLY_IN", "CHECK", "TOW", "RELEASE", "RETRIEVE", "UNWRAP", "FINAL_APPROACH", "LAUNCH_READY", "DONE"]
RESHOOT_STEPS = ["BASE_NEUTRAL", "WRAP", "RETRIEVE", "UNWRAP", "FINAL_APPROACH"]
SKIPPABLE = {"CHECK", "LANDING_ZONE", "TIDY", "COCK"}
DECOCK_STEPS = ("COCK", "FIRE", "AIM", "LAUNCH_MODE", "RAIL_LOW", "BASE_NEUTRAL")
DEFAULT_DRAG_N = 0.25            # the LIGHT clutch: the one default for the CLI, settings and the App
def approach_max_s(line_m: Optional[float], duty: float) -> float:
    """Final approach time limit, BASE_SPEC 10.1 step 17 and the firmware's approachLimit() (base-1.3.1): the longer of
    20 s and 10 s + 1.5 x line_m / v, at most 600 s, where v is the slowest take-up speed at the duty from the physics
    (config.h takeupMpsPerDuty 0.29 x duty - takeupMpsOff 0.03, duty clamped to reelMinDuty 70/255 .. takeupMaxDuty
    102/255, v at least 0.02 m/s; about 0.057 m/s at duty 0.3); 30 s with line_m unknown (None)."""
    if line_m is None:
        return 30.0
    d = min(max(float(duty), 70 / 255.0), 102 / 255.0)
    v = max(0.02, 0.29 * d - 0.03)
    return round(min(600.0, max(20.0, 10.0 + 1.5 * max(0.0, float(line_m)) / v)), 1)


# Earlier firmware defaults of k_n_per_a (base-1.0 8, physics v1.2 capstan law 17, D43 dart-side law 14.8): a unit
# flashed with an older build keeps them. The current default is read from base/spec/params.py (REEL
# tow_tension_model n_per_a), never written here (systest F28: the literal match silently stopped warning).
LEGACY_K_N_PER_A = (8.0, 17.0, 14.8, 13.1)   # 13.1: params 1.3.2, also when params.py is not on the Jetson
_PARAMS_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "base", "spec", "params.py")


def params_default_k(path: str = _PARAMS_PY) -> Optional[float]:
    """REEL tow_tension_model n_per_a from base/spec/params.py (the repo copy on the Jetson), None if not found."""
    try:
        with open(path, encoding="utf-8") as f:
            m = re.search(r"tow_tension_model\s*=\s*dict\(\s*n_per_a\s*=\s*([0-9.]+)", f.read())
        return float(m.group(1)) if m else None
    except (OSError, ValueError):
        return None


def k_is_factory(k, tension_cal: Optional[bool] = None, params_k: Optional[float] = None) -> bool:
    """True when the tow tension looks uncalibrated. The base's own tension_cal flag (base-1.3.0, get / status) wins
    when present; without it, k_n_per_a equal (to 0.05 N/A) to the params.py default or an earlier firmware
    default counts as uncalibrated."""
    if isinstance(tension_cal, bool):
        return not tension_cal
    defaults = LEGACY_K_N_PER_A + ((params_k,) if params_k else ())
    try:
        return k is not None and any(abs(float(k) - f) < 0.05 for f in defaults)
    except (TypeError, ValueError):
        return False


BASE_FAULTS = ("STALL", "OVERCURRENT", "DRUM_TIMEOUT", "DRUM_SENSOR", "ANGLE_SENSOR", "SWITCH_DISAGREE", "BROWNOUT",
               "BAD_STATE", "FAULT", "FAULTED", "DART_STUCK")


def sw_closed(tel: dict, key: str) -> Optional[bool]:
    """Contact state from telemetry sw. PROTOCOL.md: closed contacts only, one-letter keys (S seated, H carriage home,
    L rail low, W wrap arm home, D dogs tip to tip, a b c drum code). Older long names are accepted too."""
    sw = tel.get("sw")
    if not isinstance(sw, dict):
        return None
    # base-1.2 (D31): l, r = SW_DOG_L, SW_DOG_R; I = SW_INDEX; S seated; H carriage home; L rail low; W wrap cup away
    long = {"S": "seated", "H": "home", "L": "rail_low", "W": "cup_away", "l": "dog_l", "r": "dog_r", "I": "index"}
    if key in sw:
        return bool(sw[key])
    if long.get(key) in sw:
        return bool(sw[long[key]])
    return False if sw is not None else None


def dart_seated(tel: dict) -> Optional[bool]:
    """The dart in the cradle, from SW_SEATED (S). As built (cad-launch) the carriage plunger presses that lever on the
    rail lip only with the carriage on its front stop, so S reads the dart only while SW_HOME (H) is closed: None while
    the carriage is drawn (cocked), or without telemetry. The base latches the reading taken when the cock started."""
    if sw_closed(tel, "H") is False:
        return None
    return sw_closed(tel, "S")


class Aborted(Exception):
    """The operator pressed STOP or chose Abort."""


@dataclass
class MissionConfig:
    drag_n: float = DEFAULT_DRAG_N  # payout brake setting (C3 column): LIGHT 0.25 N (D49)
    tow_limit_n: float = 40.0       # tension at the dart, at most 55 N (D46, spec 1.3.2)       # line tension limit while towing (base firmware caps it too)
    check_n: float = 15.0           # anchored check pull
    check_hold_s: float = 2.0       # hold at the check tension this long
    check_keep_frac: float = 0.7    # tension must stay above this fraction of the limit (load cell only)
    retrieve_limit_n: float = 30.0  # snag limit while retrieving
    reel_duty: float = 0.8
    retrieve_duty: float = 0.6
    vbat_min: float = 10.8          # 3S: 3.6 V per cell under light load
    mode_timeout_s: float = 60.0    # drum about 8.1 deg/s: LAUNCH to NEUTRAL about 5.5 s, WRAPPED to REEL about 5 s
    wrap_timeout_s: float = 90.0    # WRAPPED / UNWRAPPED: about 37 s from NEUTRAL, 42 s from LAUNCH, 41 s from REEL at
                                    # 8.1 deg/s; BASE_SPEC 10.1 step 9 and PROTOCOL: allow 90 s
    aim_timeout_s: float = 60.0     # one-way aim through a crank: a full aim can take about 17 s
    cock_timeout_s: float = 40.0    # COCKED in up to about 20 s
    fire_timeout_s: float = 30.0    # from rest the fire runs the whole draw first
    reset_timeout_s: float = 30.0   # RESET after FIRED
    cock_first: bool = True         # cock before the confirmations (fire is then instant); False = fire from rest
    approach_m: float = 1.5         # RETRIEVE on the capstan stops with this much line left, FINAL_APPROACH does the rest
    final_timeout_s: float = 90.0
    takeup_duty: float = 0.3        # 10.1 steps 11 and 17: take-up duty
    tidy_ms: int = 3000
    rail_low_above_deg: float = 15.0   # D45
    end_mode: str = "LAUNCH"        # drum position at the end (coordinator: LAUNCH, ready for the next shot)
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
    host_timeout_s: float = 2.5     # base host watchdog during the mission (keep-alive ping every 0.4 s; F24)
    allow_uncalibrated_tow: bool = False   # True only for bench tests with no load on the line
    imu_axes: str = "x,y,z"         # BNO085 chip axes to dart body axes (BENCH_TEST.md step 7 finds this)


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


_PROMPT_SEQ = [0]                # prompt ids are unique across missions (a stale card can never match)
_PROMPT_LOCK = threading.Lock()


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
        self.fire_lock = threading.RLock()
        self.go = _Motion(self)          # every motion command goes through self.go
        self.thread: Optional[threading.Thread] = None
        self.results: Dict[str, Any] = {}
        self.shot: Dict[str, Any] = {}
        self.reshoots = 0
        self.dart_anchored = False

    # ------------------------------------------------ public
    def start(self, recover: bool = False) -> None:
        """recover=True: bring the dart home after an aborted mission instead of shooting."""
        if self.thread and self.thread.is_alive():
            raise RuntimeError("mission already running")
        self.cancel.clear()
        self.state = "RUNNING"
        self.recover_only = recover
        self.thread = threading.Thread(target=self._run, name="mission", daemon=True)
        self.thread.start()

    def running(self) -> bool:
        return bool(self.thread and self.thread.is_alive())

    def abort(self, why: str = "operator STOP", base_stop: bool = True) -> None:
        """base_stop=False (ESTOP): the mission must not send its own stop afterwards, because a stop would cancel
        the firmware's estop drum-to-NEUTRAL safing (review r3_M #2)."""
        if not base_stop:
            self._skip_base_stop = True
        if not self.running():
            return
        self.log.warn(f"abort requested: {why}")
        self.cancel.set()
        with self.fire_lock:      # wait out an arm+fire already in progress (a few ms), so STOP comes after it
            pass
        with self._pcv:
            self._pcv.notify_all()

    def answer(self, prompt_id: str, choice: str, text: Optional[str] = None) -> bool:
        """text (optional, sent by the page): the prompt text the operator saw; it must match (review r2_H #9)."""
        with self._pcv:
            p = self.prompt
            if p is None or p.id != prompt_id or choice not in p.choices:
                return False
            if text is not None and text != p.text:
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
        with _PROMPT_LOCK:
            _PROMPT_SEQ[0] += 1
            pid = f"p{_PROMPT_SEQ[0]}-{int(time.time()) % 100000}"
        with self._pcv:
            p = Prompt(pid, text, list(choices), danger, self.step)
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

    def stop_settle(self) -> None:
        """stop, then wait for the ABORTED event it causes (events follow the reply), so a later wait never sees it."""
        m = self.base.event_mark()
        self.base.stop()
        try:
            self.base.wait_event("ABORTED", 1.0, since=m, fail_on_fault=False)
        except (BaseTimeout, BaseError):
            pass

    def wait_evt(self, names, timeout: float, since: int, pred=None) -> dict:
        """Base event wait that STOP can interrupt. A link drop at any moment raises BASE_RESET (session break)."""
        sess = self.base.session if hasattr(self.base, "session") else None
        t_end = time.monotonic() + timeout
        while True:
            self.check_cancel()
            left = t_end - time.monotonic()
            if left <= 0:
                raise BaseTimeout(f"no {names} event from the base in {timeout:.0f} s")
            try:
                if sess is None:
                    e = self.base.wait_event(names, min(0.25, left), since=since, pred=pred)
                else:
                    e = self.base.wait_event(names, min(0.25, left), since=since, pred=pred, session=sess)
                self.check_cancel()
                return e
            except BaseTimeout:
                continue

    def wait_tel(self, pred, timeout: float, what: str, fail_on_fault: bool = True) -> dict:
        t_end = time.monotonic() + timeout
        while True:
            self.check_cancel()
            left = t_end - time.monotonic()
            if left <= 0:
                raise BaseTimeout(f"base: timed out waiting for {what}")
            try:
                t = self.base.wait_tel(pred, min(0.25, left), what, fail_on_fault=fail_on_fault)
                self.check_cancel()
                return t
            except BaseTimeout:
                continue

    def base_mode(self, to: str) -> None:
        t = self.tel()
        if t.get("mode") == to and t.get("drum_pos") == to and t.get("state") in ("IDLE", "HOLD"):
            return
        if t.get("state") == "HOLD":
            self.stop_settle()            # HOLD refuses everything but reel: stop first
        # after a fault the drum may still be travelling to NEUTRAL: wait for it to settle first
        self.wait_tel(lambda t: t.get("state") in ("IDLE", "HOLD", "FAULT"), self.cfg.mode_timeout_s, "base idle",
                      fail_on_fault=False)
        m = self.base.event_mark()
        self.go.mode(to)
        self.wait_evt("MODE", self.cfg.mode_timeout_s, m, pred=lambda e: e.get("mode", to) == to)
        self.wait_tel(lambda t: t.get("drum_pos") == to, 3.0, f"drum at {to}")

    # ------------------------------------------------ run
    def _set_step(self, name: str) -> None:
        self.step = name
        self.history.append({"state": name, "t": round(time.time(), 2)})
        self.log.write("step", name=name, msg=name)

    def _run(self) -> None:
        self._keepalive_start()
        try:
            if getattr(self, "recover_only", False):
                self._recover_sequence()
            else:
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
            self._keepalive_stop()
            self.prompt = None
            phase = None
            if self.state in ("ABORTED", "FAILED"):
                try:
                    phase = self.base.status(timeout=1.0).get("phase")
                except Exception:
                    pass
            drawn = phase in ("COCKING", "COCKED", "FIRING", "UNKNOWN") or (
                phase is None and self.shot.get("cocked") and not self.shot.get("fire_sent"))
            if self.state in ("ABORTED", "FAILED") and drawn:
                msg = (f"Launcher spring may be DRAWN (phase {phase or 'cocked'}): press Decock on the page before "
                       "touching the launcher.")
                self.error = (self.error + ". " if self.error else "") + msg
                self.log.warn(msg)
            dc = self.cams.get("dart")
            if dc is not None and hasattr(dc, "release_hold"):
                try:
                    dc.release_hold()
                except Exception:
                    pass
            for c in self.cams.values():
                try:
                    if c is not None and c.status().get("recording"):
                        c.stop_recording()
                except Exception:
                    pass

    def _safe_stop(self) -> None:
        if getattr(self, "_skip_base_stop", False):
            ok_d = True if not self.dart_motion_possible() else self.dart.safe_stop()
            self.log.write("info", msg="ESTOP owns the base (no mission stop sent)")
            return
        ok_b = self.base.safe_stop()
        ok_d = True if not self.dart_motion_possible() else self.dart.safe_stop()
        self.log.write("info", msg=f"safe stop: base {'ok' if ok_b else 'NO REPLY'}, dart {'ok' if ok_d else 'NO REPLY'}")

    def dart_motion_possible(self) -> bool:
        s = self.dart.last or {}
        return s.get("phase") in ("ANCHORING", "RELEASING") or bool((s.get("current_mA") or 0) > 200)

    def _do(self, name: str, fn: Callable[[], Any]) -> Any:
        """Run one step with fault handling: Retry, Skip (where safe) or Abort."""
        while True:
            self._set_step(name)
            try:
                return fn()
            except Aborted:
                raise
            except (BaseError, BaseTimeout, BaseDisconnected, DartError, TimeoutError, StepFailed) as e:
                if self.cancel.is_set():
                    raise Aborted("operator STOP")      # STOP/ESTOP caused this error: never answer it with a stop
                msg = f"{name} failed: {e}"
                self.error = msg
                self.log.error(msg)
                is_fault = isinstance(e, BaseError) and e.err in BASE_FAULTS
                st = self.tel().get("state")
                if not isinstance(e, BaseDisconnected) and not is_fault and st not in ("SAFING", "FAULT"):
                    self.base.safe_stop()      # a fault: the firmware is already safing; a stop would halt its drum
                if is_fault or st in ("SAFING", "FAULT"):
                    try:                       # let the safing move finish so the message is true
                        self.wait_tel(lambda t: t.get("state") != "SAFING", 15.0, "safing done", fail_on_fault=False)
                    except (BaseTimeout, Aborted):
                        pass
                if self.dart_motion_possible():      # whatever failed, never leave dart motors running
                    self.dart.safe_stop()
                choices = ["Retry", "Abort"]
                if name in SKIPPABLE:
                    choices.insert(1, "Skip")
                extra = getattr(e, "choices", None)
                if extra:
                    choices = list(extra) + ["Abort"]
                if is_fault and name in ("TOW", "CHECK"):
                    choices = ["Release the dart", "Retry", "Abort"]
                if name in DECOCK_STEPS:
                    phase = None
                    try:
                        phase = self.base.status(timeout=1.0).get("phase")
                    except Exception:
                        pass
                    if phase == "UNKNOWN" or (isinstance(e, BaseError) and e.err in ("LOADED_UNKNOWN",
                                                                                  "PHASE_UNKNOWN", "NOT_AT_REST",
                                                                                  "LAUNCH_BUSY")):
                        # 10.1 recovery: a fault or stop while cocking leaves the phase UNKNOWN (systest F7: no Skip)
                        choices = ["Decock and re-aim", "Abort"]
                if is_fault:
                    held = [ev for ev in self.base.events(max(0, self.base.event_mark() - 10))
                            if ev.get("evt") == "MODE" and ev.get("err") == "DRUM_HELD"]
                    pos = self.tel().get("drum_pos")
                    msg += (". The base is in FAULT. " + (f"The drum HELD where it is ({held[-1].get('why')}). "
                            if held else f"Drum now at {pos}. ") + "Check it, then Retry clears the fault.")
                if isinstance(e, BaseError) and e.err == "OVERCURRENT" and name == "AIM":
                    msg += ". Aim jammed: check the rail and the aim train before Retry (D40: 3 A trip backward)."
                if isinstance(e, BaseError) and e.err in ("LOADED_UNKNOWN", "PHASE_UNKNOWN", "NOT_AT_REST", "LAUNCH_BUSY"):
                    msg += (". The launcher spring may be drawn: choose Decock and re-aim (or Abort, then Decock on "
                            "the page).")
                if isinstance(e, BaseError) and e.err == "BASE_RESET":
                    msg += ". The base restarted (power or USB): check it before Retry."
                c = self.ask(msg, choices, danger=True)
                if c == "Skip":
                    self.log.warn(f"{name} skipped by the operator")
                    return None
                if c == "Retry":
                    self._clear_fault()
                    continue
                if c == "Decock and re-aim":
                    self._clear_fault()
                    try:
                        self.decock_and_reaim(low=name in ("RAIL_LOW", "BASE_NEUTRAL"))
                    except (BaseError, BaseTimeout, BaseDisconnected, StepFailed, DartError) as de:
                        self.error = f"decock and re-aim failed: {de}"
                        self.log.error(self.error)
                    continue
                return c   # a custom choice from StepFailed.choices

    def decock_and_reaim(self, low: bool = False) -> None:
        """decock (wait RESET; ok false = nothing to undo), then aim again (the decock turns the aim train). low: after
        the shot (RAIL_LOW, BASE_NEUTRAL) the rail goes to the D45 low pitch, never back up to the shot pitch."""
        m = self.base.event_mark()
        try:
            self.go.cmd("decock")
        except BaseError as e:
            if e.err != "UNKNOWN_CMD":
                raise
            self.go.cmd("cal", what="decock")
        try:
            self.wait_evt("RESET", 90.0, m)
        except BaseError as e:
            if not (isinstance(e.reply, dict) and e.reply.get("evt") == "RESET"):
                raise
            if str(e.reply.get("why", "")).startswith("SW_GAP open"):
                # PROTOCOL item 66: the carriage reads home but the big gear is out of its gap (a stopped reset or
                # gap run). Only the operator's eyes can tell a lying SW_HOME from a slack spring.
                self.ask("The launcher reads home but the big gear is out of its gap (SW_GAP open after a stopped "
                         "reset). Look at the launcher: is the carriage fully forward on its stop and are the springs "
                         "slack? Only then declare it at rest (the next aim closes the gap with a slow forward run "
                         "that stops if the carriage moves).", ["Carriage at rest", "Abort"], danger=True)
                self.go.cmd("cal", what="cock_rest", force=True)
            else:
                self.log.info(f"decock: nothing to undo ({e.err})")
        self.shot.pop("cocked", None)
        p = self.low_pitch() if low else self.shot.get("pitch_deg")
        if p is not None:
            if self.tel().get("mode") != "LAUNCH":
                self.base_mode("LAUNCH")
            m = self.base.event_mark()
            self.go.aim(p)
            self.wait_evt("AIMED", self.cfg.aim_timeout_s, m)
        self.log.info("decocked and re-aimed")

    def _clear_fault(self) -> None:
        try:
            # always send clear (harmless without a fault; telemetry may not show a fresh fault yet)
            t0 = time.time()
            self.base.clear()
            # wait for telemetry sent after the clear (older lines may predate the fault)
            self.wait_tel(lambda t: self.base.tel_t > t0 + 0.02 and not t.get("fault"), 3.0, "fault cleared",
                          fail_on_fault=False)
        except (BaseError, BaseTimeout, BaseDisconnected) as e:
            self.log.warn(f"clear failed: {e}")

    def _sequence(self) -> None:
        """BASE_SPEC 10.1 (D35), the one C1 sequence."""
        self._do("PREFLIGHT", self.preflight)
        while True:
            self._do("LAUNCH_MODE", lambda: self.base_mode("LAUNCH"))
            self._do("AIM", self.aim)
            if self.cfg.cock_first:
                self._do("COCK", self.cock)
            self._do("ARM", self.arm)
            self._do("FIRE", self.fire)
            self._do("FLIGHT", self.wait_landing)
            choice = self._do("LANDING_ZONE", self.landing_zone)
            if choice != "reshoot":
                self._do("RAIL_LOW", self.rail_low)   # D45: tow with the rail at its minimum
            self._do("BASE_NEUTRAL", self.base_neutral)
            if choice == "reshoot":
                self.reshoot()                  # NO_GO before the wrap: retrieve on the take-up
                continue
            self._do("WRAP", self.wrap)          # step 9: park at WRAPPED while the line is still slack
            r = self._do("ANCHOR", self.anchor)  # step 10
            if r == "Release and reshoot":
                self.dart_anchored = True       # it may be partly dug in: always run the release
                self._do("RELEASE", self.release_dart)
                self.reshoot()
                continue
            break
        self._do("TIDY", self.tidy)              # step 11
        self._do("FLY_IN", self.fly_in)          # step 12: mode REEL from the WRAPPED park
        r = self._do("CHECK", self.anchored_check)
        if r in ("Release and reshoot", "Release the dart"):
            self._do("RELEASE", self.release_dart)
            self.bring_home()
            self.reshoots += 1
            self._do("LAUNCH_READY", lambda: self.base_mode(self.cfg.end_mode))
            self.log.info("dart retrieved after a failed anchor check; start a new mission to shoot again")
            return
        self._do("TOW", self.tow)                # step 13
        self._do("RELEASE", self.release_dart)   # step 14
        self.bring_home()                        # steps 15 to 17
        self._do("LAUNCH_READY", lambda: self.base_mode(self.cfg.end_mode))   # step 18

    def _recover_sequence(self) -> None:
        """After an abort with the dart out or anchored (review r2_H #4)."""
        self._set_step("PREFLIGHT")
        self.ask("Recover the dart: the base will release, reel in and retrieve it. Area clear and hands off the base?",
                 ["Base may move", "Abort"], danger=True)
        self._clear_fault()
        try:
            ph = self.dart.status()["phase"]
        except DartError:
            ph = None
        if ph in ("ANCHORING", "ANCHORED", "FAULT", "RELEASING"):
            self.dart_anchored = True
            self._do("RELEASE", self.release_dart)
        t = self.tel()
        if dart_seated(t) is None:
            self._do("PREFLIGHT", self._need_rest)   # carriage drawn: SW_SEATED cannot read the dart
            t = self.tel()
        if dart_seated(t) and self.wraps_maybe_on(t):
            self._do("UNWRAP", self.unwrap)          # seated with the wraps on or not known off: unwrap first
        if not dart_seated(t):
            if t.get("mode") == "LAUNCH":
                self._do("BASE_NEUTRAL", self.base_neutral)
            if (t.get("line_m") or self.shot.get("line_est_m") or 0) > self.cfg.approach_m + 0.5 \
                    and not self.is_wrapped(t):
                self._do("WRAP", self.wrap)          # (also from inside the wrap window: on to the WRAPPED park)
            self.bring_home()                        # unwraps a drum stopped inside the window (wraps_maybe_on)
        self._do("LAUNCH_READY", lambda: self.base_mode(self.cfg.end_mode))

    # ---- host watchdog (review r2_H #7, coordinator 1): if the Jetson stalls, the base stops by itself
    def _keepalive_start(self) -> None:
        self._ka_stop = threading.Event()
        self._ka_prev = None
        if self.cfg.host_timeout_s <= 0:
            return
        try:
            self._ka_prev = self.base.get("host_timeout_s").get("val")
            self.base.set("host_timeout_s", self.cfg.host_timeout_s)
        except Exception:
            self._ka_prev = None
            return

        def run():
            # F24: a fixed 0.4 s schedule (a slow reply must not stretch the gap between pings past the watchdog)
            nxt = time.monotonic()
            while True:
                nxt += 0.4
                if self._ka_stop.wait(max(0.0, nxt - time.monotonic())):
                    break
                try:
                    self.base.ping(timeout=0.35)
                except Exception:
                    pass
                if time.monotonic() > nxt + 0.4:
                    nxt = time.monotonic()      # fell behind (link stall): restart the schedule, no burst
        self._ka_thread = threading.Thread(target=run, name="base-keepalive", daemon=True)
        self._ka_thread.start()

    def _keepalive_stop(self) -> None:
        ev = getattr(self, "_ka_stop", None)
        if ev is not None:
            ev.set()
        if getattr(self, "_ka_prev", None) is not None:
            try:
                self.base.set("host_timeout_s", 0)   # r3_M #4: off outside a mission, on every exit path
            except Exception:
                pass

    def tel_fresh(self, max_age: float = 0.5) -> dict:
        """Telemetry no older than max_age, else StepFailed (stop first): never act on a stalled link."""
        age = time.time() - getattr(self.base, "tel_t", time.time())
        if age > max_age:
            self.stop_settle()
            raise StepFailed(f"base telemetry stalled ({age:.1f} s old): stopped. Check the USB link, then Retry.")
        return self.tel()

    def fresh_after(self, t0: float, timeout: float = 1.0) -> dict:
        """Wait for a telemetry line sent after t0 (state right after an event is often stale)."""
        try:
            return self.wait_tel(lambda t: self.base.tel_t > t0, timeout, "fresh telemetry", fail_on_fault=False)
        except BaseTimeout:
            return self.tel()

    def bring_home(self) -> None:
        """Steps 15 to 17: capstan reel-in to about approach_m (only when wrapped), UNWRAP, then the take-up final
        approach (retrieve) in NEUTRAL. Never seat on the capstan."""
        t = self.tel()
        if self.is_wrapped(t):
            self._do("RETRIEVE", self.retrieve)
            self._do("UNWRAP", self.unwrap)
        elif self.wraps_maybe_on(t):
            self._do("UNWRAP", self.unwrap)   # stopped inside the wrap window: only unwrap reaches NEUTRAL
        self._do("FINAL_APPROACH", self.final_approach)

    def _need_rest(self) -> None:
        """Recover with the carriage drawn (cocked or unknown): SW_SEATED cannot read the dart and the drum cannot
        leave LAUNCH until a decock."""
        try:
            phase = self.base.status(timeout=1.0).get("phase")
        except (BaseError, BaseTimeout, BaseDisconnected):
            phase = None
        if phase in ("COCKING", "COCKED", "FIRING", "RESETTING", "UNKNOWN"):   # (no answer: the drum moves tell)
            raise StepFailed(f"launcher not at rest (phase {phase}): the seated contact reads the dart only with the "
                             "carriage home, and the drum cannot leave LAUNCH. Abort, press Decock on the page, then "
                             "Recover dart again.")

    def fly_in(self) -> None:
        """mode REEL from the WRAPPED park (the collar engages the capstan)."""
        t = self.tel()
        if t.get("mode") == "REEL" and t.get("state") in ("IDLE", "HOLD"):
            return
        self.base_mode("REEL")

    # ------------------------------------------------ steps
    def preflight(self) -> None:
        problems, warnings = [], []
        self.base_live_confirmed = False
        try:
            r = self.base.ping(timeout=1.5)
            self.log.info(f"base {r.get('fw', 'firmware ?')}")
            st = self.base.status()
            t = dict(self.tel())
            t.update({k: v for k, v in st.items() if k not in ("id", "ok")})   # PROTOCOL: flat status fields
        except (BaseError, BaseTimeout, BaseDisconnected) as e:
            raise StepFailed(f"base not answering: {e}")
        if t.get("fault"):
            raise StepFailed(f"base in FAULT {t['fault']}: clear it first")
        if t.get("magnet") == "NONE":
            problems.append("AS5600 sees no magnet (status magnet NONE): wiper actions will fail. Check the magnet "
                            "and the sensor distance")
        elif t.get("magnet") in ("WEAK", "STRONG"):
            warnings.append(f"AS5600 magnet {t.get('magnet')}: angle readings may glitch")
        if t.get("phase") == "UNKNOWN":
            problems.append("launcher shot phase UNKNOWN (power lost mid-cycle): Abort, press Decock on the page, "
                            "then start again")
        elif t.get("phase") in ("COCKING", "COCKED"):
            problems.append(f"launcher {t.get('phase')} (the seated contact reads the dart only with the carriage "
                            "home): Abort, press Decock on the page, then start again")
        if t.get("drum_pos") in ("UNKNOWN", "BETWEEN"):
            problems.append(f"drum position {t.get('drum_pos')} (inside the wrap window: press Unwrap or Recover dart; "
                            "between LAUNCH and NEUTRAL: Neutral or jog the drum)")
        if t.get("wraps") is None and "wraps" in t:
            problems.append("capstan wraps unknown (after a restart): press Unwrap (Manual section) or Recover dart, "
                            "then Retry")
        elif (t.get("wraps") or 0) > 0.05:
            problems.append("capstan still wrapped (unwrap first)")
        if dart_seated(t) is False:
            problems.append("dart not seated on the launcher (seated contact open)")
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
        if t.get("tension_n") is None:
            try:
                if self.tension_uncalibrated():
                    k = self.base.get("k_n_per_a").get("val")
                    warnings.append(f"no load cell and the tension constants are not calibrated (k_n_per_a {k}): "
                                    "the tow limit is a rough current estimate (calibrate, see firmware/base/"
                                    "CALIBRATION.md)")
            except (BaseError, BaseTimeout, BaseDisconnected, ValueError, TypeError):
                pass
        tgt = self.get_target()
        if not tgt or not tgt.get("range_m"):
            problems.append("no target: tap the target on the rover view or enter a range")
        if problems:
            raise StepFailed("; ".join(problems))
        if warnings:
            self.ask("Preflight warnings: " + "; ".join(warnings) + ". Continue?", ["Continue", "Abort"])
        self.ask("Preflight OK. From now on the base moves by itself between prompts (drum shifts, the wiper turns a "
                 "few degrees during a mode change, aiming). Area clear and hands off the base?",
                 ["Base may move", "Abort"], danger=True)
        self.log.info("preflight ok")

    def aim(self) -> None:
        t = self.tel()
        if t.get("drum_pos") != "LAUNCH":
            self.base_mode("LAUNCH")
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
        self.go.aim(sol.pitch_deg)
        e = self.wait_evt("AIMED", self.cfg.aim_timeout_s, m)
        self.shot["pitch_deg"] = sol.pitch_deg
        self.shot["pitch_actual"] = e.get("pitch_deg", e.get("deg", sol.pitch_deg))
        self.log.info(f"aimed at {self.shot['pitch_actual']} deg (target {sol.pitch_deg})")

    def cock(self) -> None:
        """Cock the spring now, so the fire after the confirmations is instant. Aim must already be done (PROTOCOL:
        aim needs the carriage home)."""
        self.ask("Cock the launcher now? The spring is drawn and stays drawn until the shot or a Decock. "
                 "Keep hands and the line clear of the rail.", ["Cock", "Abort"], danger=True)
        m = self.base.event_mark()
        self.go.cmd("cock")                 # already cocked: ok and an immediate COCKED event
        self.wait_evt("COCKED", self.cfg.cock_timeout_s, m)
        self.shot["cocked"] = True
        self.log.info("cocked")

    def wait_launcher_rest(self) -> None:
        """After the shot: wait for the launcher RESET (mode and aim refuse until it is at rest)."""
        m0 = self.shot.pop("fire_mark", None)      # only the first attempt may look back to the shot
        t = self.tel()
        if t.get("mode") == "LAUNCH" and not (t.get("cock") == 0 and t.get("state") == "IDLE"):
            if m0 is not None:
                try:
                    self.wait_evt("RESET", self.cfg.reset_timeout_s, m0)
                    return
                except BaseTimeout:
                    pass
            # no shot mark (a later step, or the RESET missed): the telemetry says when the launcher is at rest.
            # Waiting for a new RESET here would wait out the whole timeout for an event that already came.
            self.wait_tel(lambda t: t.get("state") == "IDLE" and t.get("cock") in (0, 0.0, None),
                          self.cfg.reset_timeout_s if m0 is None else 5.0, "launcher at rest")

    def rail_low(self) -> None:
        """BASE_SPEC 10.1 step 7b (D45): before the wrap, bring the rail to its minimum pitch when it is above
        15 deg, so the tow over the lip roller does not push the carriage back against the spring preload."""
        self.wait_launcher_rest()
        t = self.tel()
        p = t.get("pitch_deg")
        if t.get("mode") != "LAUNCH" or p is None or p <= self.cfg.rail_low_above_deg:
            return
        lo = self.low_pitch()
        m = self.base.event_mark()
        self.go.aim(lo)
        self.wait_evt("AIMED", self.cfg.aim_timeout_s, m)
        self.log.info(f"rail low for the tow ({lo:.1f} deg)")

    def low_pitch(self) -> float:
        """D45: the minimum pitch the base accepts (pitch_range min, at least 6.0), plus 0.05."""
        lo = 6.0
        try:
            r = self.base.get("pitch_range")
            lo = max(lo, float(r.get("min", lo)))
        except (BaseError, BaseTimeout, BaseDisconnected, TypeError, ValueError):
            pass
        return round(lo + 0.05, 2)

    def base_neutral(self) -> None:
        """Launcher at rest, then the drum to NEUTRAL."""
        self.wait_launcher_rest()
        self.base_mode("NEUTRAL")

    def tidy(self) -> None:
        """10.1 step 11: tidy {ms, duty} -> TIDIED (take-up only, by time). reel until "slip" is the alternative on
        firmware without tidy."""
        m = self.base.event_mark()
        try:
            self.go.cmd("tidy", ms=int(self.cfg.tidy_ms), duty=self.cfg.takeup_duty)
            self.wait_evt(("TIDIED",), self.cfg.wrap_timeout_s, m)
            self.log.info("line tidied")
            return
        except BaseError as e:
            if e.err != "UNKNOWN_CMD":
                raise
        m = self.base.event_mark()
        self.go.reel(self.cfg.takeup_duty, max_n=self.cfg.check_n, until="slip")
        e = self.wait_evt(("LIMIT",), self.cfg.wrap_timeout_s, m)
        if self.tel().get("state") == "HOLD":
            self.stop_settle()
        self.log.info(f"line tidied ({e.get('why')})")

    def arm(self) -> None:
        """Arm the dart landing detector right before the shot (/stop disarms it). By default auto=0: the dart only
        records the flight and the mission anchors after the landing zone verdict."""
        try:
            st = self.dart.status()
            if st.get("imu_ok") is False:
                self.ask("The dart IMU is not reporting (imu_ok false): the launch and landing cannot be seen and there "
                         "will be no flight log. Fire anyway (landing from the video and your call)?",
                         ["Fire without IMU", "Abort"], danger=True)
        except DartError:
            pass
        try:
            try:
                s = self.dart.arm(True, auto=self.cfg.auto_anchor_on_landing)
            except DartRefused as e:
                if not (e.reason.startswith("anchored") or e.reason.startswith("legs out")):
                    raise
                # API v4: /arm is refused while ANCHORED (legs out). Never fire a dart with its legs out:
                # release it (legs and kick stowed), then arm again.
                self.ask(f"The dart is ANCHORED ({e.reason}): its legs are out and it must not be fired like that. "
                         "Release it (stows the legs), then arm?", ["Release the dart", "Abort"], danger=True)
                self.dart_anchored = True
                try:
                    self.release_dart()
                    s = self.dart.arm(True, auto=self.cfg.auto_anchor_on_landing)
                except DartRefused as e2:
                    # never reach the generic "Continue without" below with the legs out (review 6 #4)
                    raise StepFailed(f"dart still anchored or refused after the release ({e2.reason}): use the "
                                     "manual dart keys, then Retry")
            self.shot["dart_phase_at_arm"] = s["phase"]
            self.shot["dart_t_ms_at_arm"] = s.get("t_ms")
            if not s.get("armed"):
                raise StepFailed("dart did not arm")
        except DartRefused as e:
            if e.reason.startswith("anchored") or e.reason.startswith("legs out"):
                raise StepFailed(f"dart still anchored after the release ({e.reason}): use the manual dart keys")
            c = self.ask(f"Dart refused to arm ({e.reason}). Without it there is no flight log and landing comes "
                         "from the line and video only. Continue?", ["Continue without", "Retry", "Abort"])
            if c == "Retry":
                raise StepFailed(f"dart refused arm: {e.reason}")
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

    def ensure_launch(self) -> None:
        """After a fault the base is in NEUTRAL: put it back in LAUNCH and re-aim before firing again."""
        t = self.tel()
        if t.get("drum_pos") != "LAUNCH" or t.get("mode") != "LAUNCH":
            self.log.info("base not in LAUNCH: switching back")
            self.base_mode("LAUNCH")
            t = self.tel()
        p = self.shot.get("pitch_deg")
        cur = t.get("pitch_deg")
        aimed = None
        try:
            aimed = self.base.status().get("aimed")
        except (BaseError, BaseTimeout):
            pass
        if p is not None and (aimed is False or cur is None or abs(cur - p) > 1.0):
            self.log.info(f"re-aiming to {p} deg")
            m = self.base.event_mark()
            self.go.aim(p)
            self.wait_evt("AIMED", self.cfg.aim_timeout_s, m)

    @staticmethod
    def wraps_maybe_on(t: dict) -> bool:
        """Wraps on or not known off: outside the LAUNCH and NEUTRAL parks with drum code bit c closed (WRAP_PREP,
        WRAP, N_TO_R, REEL), or wraps null or above 0. Only unwrap brings such a drum back to NEUTRAL (mode, a drum jog
        and retrieve are refused WRAPS_ON inside the window)."""
        if t.get("drum_pos") in ("LAUNCH", "NEUTRAL"):
            return False
        w = t.get("wraps")
        return bool(sw_closed(t, "c")) or w is None or (isinstance(w, (int, float)) and w > 0)

    @staticmethod
    def is_wrapped(t: dict) -> bool:
        """Wraps on: parked at WRAPPED (314) or flown in to REEL. Not while the drum is still laying them (WRAP)."""
        if t.get("state") in ("WRAP", "UNWRAP"):
            return False
        w = t.get("wraps")
        # A2: all 7 wraps and the drum at the park or in REEL; a hold partway through the wrap is NOT wrapped
        return t.get("drum_pos") in ("WRAPPED", "REEL") and (w is None or (isinstance(w, (int, float)) and w >= 7))

    def ensure_reel(self, wrapped: bool = True) -> None:
        """Before a reel on the capstan: wraps on and the collar in REEL (fly-in). A fault in REEL holds the drum
        (base-1.2.1, wraps on); if it is at the WRAPPED park after a clear, this flies in again."""
        if wrapped and not self.is_wrapped(self.tel()):
            self.log.info("capstan not wrapped: wrapping")
            self.wrap()
        if wrapped and self.tel().get("mode") != "REEL":
            self.fly_in()

    def dart_is_out(self) -> bool:
        """True when telemetry says the dart has left the launcher (line out, or not seated with the carriage home:
        SW_SEATED is open whenever the carriage is drawn, seated or not)."""
        t = self.tel()
        return (t.get("line_m") or 0) > 0.3 or dart_seated(t) is False

    def fire(self) -> None:
        if self.shot.get("fire_sent") and self.dart_is_out():
            # the shot happened but its FIRED event was lost (USB glitch, reset): carry on with the flight
            self.log.warn("dart already out after the fire command: continuing to FLIGHT")
            self.shot.setdefault("t_fire", self.shot["fire_sent"])
            return
        self.ensure_launch()
        if self.cfg.cock_first:
            try:
                phase = self.base.status().get("phase")      # fresh, unlike telemetry right after COCKED
            except (BaseError, BaseTimeout):
                phase = None
            if phase is not None and phase != "COCKED":
                self.log.info(f"launcher phase {phase}, not COCKED: cocking again")
                self.cock()
        p = self.shot.get("pitch_deg")
        self.ask(f"FIRE at pitch {p} deg toward {self.shot.get('range_m', 0):.1f} m? Area clear, line free, "
                 "everyone behind the launcher?", ["Fire", "Abort"], danger=True)
        self.ask("Second confirmation: FIRE NOW?", ["FIRE NOW", "Abort"], danger=True)
        with self.fire_lock:               # STOP takes this lock too, so it can never slip between arm and fire
            self.check_cancel()
            token = self.base.arm()
            self.check_cancel()
            m = self.base.event_mark()
            self.shot["fire_mark"] = m
            c = self.cams.get("dart")
            if c is not None and hasattr(c, "hold"):
                c.hold(time.time() - self.cfg.pre_s)     # keep the launch frames until the analysis has them
            self.base.fire(token)
            self.shot["fire_sent"] = time.time()
        try:
            e = self.wait_evt("FIRED", self.cfg.fire_timeout_s, m)
        except (BaseTimeout, BaseDisconnected):
            if self.dart_is_out():
                self.log.warn("no FIRED event but the dart is out: continuing to FLIGHT")
                self.shot["t_fire"] = self.shot["fire_sent"]
                return
            raise
        self.shot["t_fire"] = e.get("_t", time.time())
        self.log.info("FIRED")

    def wait_landing(self) -> None:
        """Landed when the dart says so, or the line has stopped paying out (and the dart video is still)."""
        t_fire = self.shot.get("t_fire", time.time())
        t_end = time.time() + self.cfg.land_timeout_s
        last_line, line_t = None, time.time()
        sources = set()
        # a dart still showing LANDED from an earlier shot only counts after it has been seen in FLIGHT again
        stale = self.shot.get("dart_phase_at_arm") != "ARMED"     # any arm not confirmed needs a seen FLIGHT
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
                if time.time() - getattr(self.dart, "last_t", 0) >= 0.25:   # C2: never poll faster than 5 Hz overall
                    ds = self.dart.status()
                else:
                    ds = self.dart.last or self.dart.status()
                if ds["phase"] == "FLIGHT":
                    seen_flight = True
                t_arm = self.shot.get("dart_t_ms_at_arm")
                if isinstance(t_arm, (int, float)) and isinstance(ds.get("t_ms"), (int, float)) and ds["t_ms"] < t_arm:
                    # the dart rebooted (power plug dropout at impact): armed state and flight log are lost.
                    # fw-dart: treat it as landed, then /stop and /auto.
                    self.shot["dart_rebooted"] = True
                    sources.add("dart")
                    self.log.warn(f"dart rebooted during the flight (reset {ds.get('reset')}): treating as landed")
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
            self.ask("Landing not detected (no dart signal, line still moving or no data). Is the dart down?",
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
                try:
                    frames = c.buffer(since=t_fire - self.cfg.pre_s)   # filters before decoding held frames
                except TypeError:          # cameras without since=
                    frames = [f for f in c.buffer() if f.t >= t_fire - self.cfg.pre_s]
            except Exception as e:
                self.log.warn(f"no dart frames: {e}")
        fl = None
        try:
            # /flightlog is refused (409) while armed, in FLIGHT or with motors running: let the dart decide first
            ph = None
            done = ("LANDED", "ANCHORED", "IDLE", "FAULT", "ARMED", "ANCHORING", "RELEASING")
            still = None
            try:
                ph = self.dart.wait_phase(done, 3.0, poll=0.3, fail=(), cancel=self.cancel)["phase"]
            except DartError as e:
                if "cancelled" in str(e):
                    raise Aborted("operator STOP")
                try:
                    still = self.dart.status().get("phase")
                except DartError:
                    still = None                       # link gap: never stop blind, wait for the dart instead
                if still != "FLIGHT":               # a link gap or a running sequence: wait as before, never stop
                    try:
                        ph = self.dart.wait_phase(done, 37.0, poll=0.3, fail=(), cancel=self.cancel)["phase"]
                    except DartError as e3:
                        if "cancelled" in str(e3):
                            raise Aborted("operator STOP")
            # the mission already decided the dart is down (line, video or the operator) and the dart CONFIRMS it
            # is still in FLIGHT: a full /stop ends flight tracking (landing "stop", the log freezes 3 s later;
            # API v4; /stop?disarm=0 would keep tracking) instead of waiting up to 40 s for the dart's own timeout.
            # Never while ANCHORING or RELEASING (review 6 #1: that would abort the sequence half way).
            if ph is None and still == "FLIGHT":
                self.log.warn("dart still in FLIGHT after the landing decision: ending its flight tracking")
                try:
                    self.dart.stop()
                    ph = self.dart.wait_phase(done, 6.0, poll=0.3, fail=(), cancel=self.cancel)["phase"]
                except DartError as e2:
                    if "cancelled" in str(e2):
                        raise Aborted("operator STOP")
            if ph == "ARMED":
                self.log.warn("dart still ARMED (launch not seen): disarming to read the live log")
                self.dart.arm(False)
            fl = self.dart.flightlog()
            t_arm = self.shot.get("dart_t_ms_at_arm")
            if not fl.get("samples"):
                self.log.warn("flight log is empty (dart was not armed or saw no launch)")
                fl = None
            elif self.shot.get("dart_phase_at_arm") != "ARMED" or fl.get("t_launch_ms") is None or (
                    isinstance(t_arm, (int, float)) and fl["t_launch_ms"] < t_arm):
                self.log.warn("flight log is not from this shot (arm not confirmed or launch before the arm): ignored")
                fl = None
            else:
                fl = remap_flightlog(fl, self.cfg.imu_axes)
            if self.run_dir:
                import json
                if fl is not None:
                    with open(os.path.join(self.run_dir, f"flightlog_shot{self.reshoots}.json"), "w") as f:
                        json.dump(dict(fl, clock_offset_s=self.dart.clock_offset()), f)
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
        off = self.dart.clock_offset() if hasattr(self.dart, "clock_offset") else None
        kw = dict(t_fire=t_fire, log=self.log.warn, clock_offset_s=off)
        try:      # jetson-vision: the commanded pitch and the clutch drag are strong priors for the range
            res = self.analyse(frames, fl, self.run_dir, aim_pitch_deg=self.shot.get("pitch_actual") or
                               self.shot.get("pitch_deg"), drag_n=self.cfg.drag_n, **kw)
        except TypeError as e:
            if "aim_pitch_deg" not in str(e) and "drag_n" not in str(e):
                raise
            res = self.analyse(frames, fl, self.run_dir, **kw)
        if c is not None and hasattr(c, "release_hold"):
            c.release_hold()
        self.results = res
        self.on_results(res)
        self.tell_line_out(res)
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

    def tell_line_out(self, res: dict) -> None:
        """The base has no payout sensor (line_m is null after a shot): give it our estimate (PROTOCOL set line_m).
        R2-C2: a video-only range can be far too long (9 m against 5 m), and a long estimate would let the capstan
        reel-in seat the dart on the capstan. So the flight range is used only when the IMU flight log backs it, and
        never beyond what the C3 table allows; otherwise the ballistic solution's line out."""
        fl = res.get("flight") or {}
        sol = (self.shot.get("solution") or {})
        est, src = None, ""
        rng = fl.get("range_m")
        imu = "imu" in str(fl.get("source") or "").lower()
        if imu and isinstance(rng, (int, float)) and 0.5 < rng < 30:
            est, src = rng * 1.05 + 0.3, "IMU flight log"     # straight distance plus a little sag and the rail
        elif sol.get("line_out_m"):
            est, src = sol["line_out_m"], "ballistics"
        if est is None:
            return
        rmax = sol.get("range_max_m")
        if isinstance(rmax, (int, float)) and rmax > 0:
            est = min(est, rmax * 1.1 + 0.5)
        est = round(min(est, 100.0), 2)
        self.shot["line_est_m"] = est
        try:
            self.base.set("line_m", est)
            self.log.info(f"told the base line_m = {est} m ({src})")
        except (BaseError, BaseTimeout, BaseDisconnected) as e:
            self.log.warn(f"could not set line_m on the base: {e}")

    def anchor(self) -> Optional[str]:
        if self.shot.pop("dart_rebooted", False):
            self.dart.safe_stop()              # clears a reboot FAULT before /auto
        s = self.dart.status()
        if s["phase"] == "FLIGHT":
            # /auto is refused in FLIGHT: the dart's own landing decision ends it (15 s timeout at worst)
            self.log.info("dart still in FLIGHT: waiting for its landing decision")
            s = self.dart.wait_phase(("LANDED", "ANCHORING", "ANCHORED", "IDLE", "ARMED"), 17.0, cancel=self.cancel)
        if s["phase"] == "ANCHORED":
            self.dart_anchored = True
            return None
        if s["phase"] == "ARMED":
            # the dart never saw the launch: /auto is refused while armed ("disarm first")
            self.log.warn("dart still ARMED (launch not seen): disarming before /auto")
            self.dart.arm(False)
        if s["phase"] not in ("ANCHORING",):
            try:
                self.dart.auto()
            except DartRefused as e:
                if "already anchored" not in e.reason:
                    raise StepFailed(f"dart refused auto: {e.reason}", choices=["Retry", "Release and reshoot"])
        try:
            s = self.dart.wait_phase("ANCHORED", self.cfg.anchor_timeout_s, cancel=self.cancel,
                                     log=self.log.info)
        except DartError as e:
            if "cancelled" in str(e):
                raise Aborted("operator STOP")
            if "NO_GROUND" in str(e):
                # the kick swung through air: the dart lies on its back or side. A new /auto faults again; /release
                # stows the kick, then the dart comes home for a new shot.
                raise StepFailed(f"dart on its back or side (NO_GROUND: {e}). Retrying the anchor faults again: "
                                 "release it and reshoot", choices=["Release and reshoot"])
            raise StepFailed(f"dart did not anchor: {e}", choices=["Retry", "Release and reshoot"])
        self.dart_anchored = True
        warn = [w for w in (s.get("warnings") or []) if w]
        self.shot["anchor_warnings"] = warn
        if warn:
            c = self.ask(f"Dart anchored WITH WARNINGS ({', '.join(warn)}): the anchor may be weak. "
                         "Tow anyway (the check pull comes first), or release and reshoot?",
                         ["Tow anyway", "Release and reshoot", "Abort"], danger=True)
            if c == "Release and reshoot":
                return c
        self.log.info("dart anchored")
        return None

    def wrap(self) -> None:
        if self.is_wrapped(self.tel()):
            return
        m = self.base.event_mark()
        try:
            self.go.wrap()
        except BaseError as e:
            if e.err != "NOT_IN_REEL":
                raise
            self.base_mode("REEL")           # firmware with REEL before the wrap sectors
            m = self.base.event_mark()
            self.go.wrap()
        self.wait_evt("WRAPPED", self.cfg.wrap_timeout_s, m)
        self.wait_tel(self.is_wrapped, 5.0, "wraps on")

    def unwrap(self) -> None:
        if self.tel().get("state") == "HOLD":
            self.stop_settle()
        t = self.tel()
        if not self.is_wrapped(t) and not self.wraps_maybe_on(t):
            return                          # wraps None (unknown) still unwraps: the firmware knows the sector
        m = self.base.event_mark()
        self.go.unwrap()
        self.wait_evt("UNWRAPPED", self.cfg.wrap_timeout_s, m)
        try:                                   # 10.1 step 16: UNWRAPPED, then MODE NEUTRAL
            self.wait_evt("MODE", 15.0, m, pred=lambda e: e.get("mode") == "NEUTRAL")
        except BaseTimeout:
            pass
        self.wait_tel(lambda t: (t.get("wraps") or 0) == 0, 5.0, "wraps off")

    def anchored_check(self) -> Optional[str]:
        """Pull to check_n. If the line stops coming in, the dart holds. If it keeps coming in at the check tension,
        the dart is dragging."""
        self.ensure_reel()
        line0 = self.tel().get("line_m") or 0.0
        m = self.base.event_mark()
        self.go.reel(self.cfg.reel_duty * 0.6, max_n=self.cfg.check_n, until="tension")
        try:
            e = self.wait_evt(("LIMIT", "SEATED"), 30.0, m)
        except BaseTimeout:
            self.base.safe_stop()
            raise StepFailed("check pull never reached the test tension: the dart may be dragging",
                             choices=["Retry", "Release and reshoot"])
        if e.get("evt") == "SEATED" or sw_closed(self.tel(), "S"):
            self.stop_settle()
            raise StepFailed("the dart came all the way home during the check (it did not hold)",
                             choices=["Release and reshoot"])
        # Reaching the check tension is itself the main evidence: a dart dragging over the ground pulls only a few N.
        # The worm self-locks after LIMIT, so line cannot come in during the hold; instead watch the tension
        # (load cell) for a decay that means the anchor is creeping out.
        t1 = self.tel()
        line1 = t1.get("line_m") or 0.0
        n1 = e.get("tension_n", t1.get("tension_n"))
        self.sleep(self.cfg.check_hold_s)
        t = self.tel()
        n2 = t.get("tension_n")
        decayed = (n1 is not None and n2 is not None and n2 < self.cfg.check_keep_frac * float(n1))
        held = not decayed
        self.shot["check"] = {"line_in_m": round(line0 - line1, 3), "tension_n": n2, "tension_at_limit_n": n1,
                              "held": held, "load_cell": n2 is not None}
        if not held:
            raise StepFailed(f"tension fell from {n1} to {n2} N during the hold: the anchor is slipping",
                             choices=["Retry", "Release and reshoot"])
        self.log.info(f"anchor holds at {t.get('tension_n')} N")
        return None

    def tension_uncalibrated(self, k=None) -> bool:
        """F28: the base's tension_cal flag (get tension_cal) when the firmware has it, else k_n_per_a against the
        params.py default and the earlier firmware defaults."""
        try:
            flag = self.base.get("tension_cal").get("val")
            if isinstance(flag, bool):
                return not flag
        except (BaseError, BaseTimeout, BaseDisconnected):
            pass                                   # older firmware: BAD_KEY
        if k is None:
            try:
                k = self.base.get("k_n_per_a").get("val")
            except (BaseError, BaseTimeout, BaseDisconnected):
                return True                        # cannot tell: fail closed (review 6 #3)
        return k_is_factory(k, params_k=params_default_k())

    def tow(self) -> None:
        if self.tel().get("tension_n") is None:
            try:
                k = self.base.get("k_n_per_a").get("val")
            except (BaseError, BaseTimeout):
                k = None
            if self.tension_uncalibrated(k) and not self.cfg.allow_uncalibrated_tow:
                raise StepFailed(f"TOW refused: no load cell and the tension constants are not calibrated (k_n_per_a {k}), so the tension limit "
                                 "cannot be trusted (a test reached 61 N against 40 N). Calibrate free_a and k_n_per_a "
                                 "(firmware/base/CALIBRATION.md (d) and (f)), then Retry; or release the dart.",
                                 choices=["Retry", "Release the dart"])
        try:
            lc = bool(self.base.status().get("hx"))      # a load cell fitted: its tension_n is a real reading
        except (BaseError, BaseTimeout, BaseDisconnected):
            lc = False                                    # unknown: treat tension_n as the held current estimate
        rounds = 0
        while rounds < self.cfg.tow_max_rounds + getattr(self, "_extra_tow_rounds", 0):
            rounds += 1
            self.check_cancel()
            self.ensure_reel()
            m = self.base.event_mark()
            fresh = self.fresh_after(getattr(self, "_last_limit_t", 0.0))   # F18: the state right after LIMIT is stale
            tn_now = fresh.get("tension_n")
            if lc and tn_now is not None and tn_now >= 0.95 * self.cfg.tow_limit_n:
                # a load cell reading is real: the line is still at the limit, do not pull again
                self.log.info(f"line already at {tn_now} N: not pulling again")
                r = self.rover.tow_assist(self, tn_now)
                if r == "done":
                    return
                if r == "abort":
                    raise Aborted("operator chose Abort")
                continue
            if lc and fresh.get("state") == "REEL":
                # load cell: the firmware already winds again by itself when the held line slackens; just wait
                self.log.info("base is already winding (slack taken up by itself)")
            else:
                if fresh.get("state") in ("HOLD", "REEL"):
                    # F22 (by design): without a load cell the HOLD keeps its held estimate at the limit, so a tow
                    # re-sent from HOLD answers LIMIT "already at the limit" and never takes up the rover's slack.
                    # stop first (wait for ABORTED), then a new reel: the firmware restarts at minimum duty and
                    # cuts in about 110 ms if the line really is still at the limit.
                    self.stop_settle()
                    m = self.base.event_mark()
                try:
                    self.shot["towed"] = True        # the tow shortens the line by an amount the landing estimate does not know
                    self.go.reel(self.cfg.reel_duty, max_n=self.cfg.tow_limit_n, until="tension")
                except BaseError as be:
                    if be.err != "BUSY":
                        raise
                    self.log.info("base busy winding by itself: waiting for its LIMIT")
            e = self.wait_evt(("LIMIT", "SEATED"), 60.0, m)
            if e.get("evt") == "SEATED" or sw_closed(self.tel(), "S"):
                self.stop_settle()
                raise StepFailed("the dart came home during the tow: the anchor pulled out",
                                 choices=["Release and bring home"])
            self._last_limit_t = time.time()
            tn = e.get("tension_n", self.tel().get("tension_n"))
            self.log.info(f"tow round {rounds}: holding at {tn} N, line {self.tel().get('line_m')} m")
            r = self.rover.tow_assist(self, tn)
            if r == "done":
                self.log.info("rover free")
                return
            if r == "abort":
                raise Aborted("operator chose Abort")
        c = self.ask(f"Tow round limit ({self.cfg.tow_max_rounds + getattr(self, '_extra_tow_rounds', 0)}) reached. Release the dart now?",
                     ["Release the dart", "Keep towing", "Abort"])
        if c == "Keep towing":
            self._extra_tow_rounds = getattr(self, "_extra_tow_rounds", 0) + 10   # this mission only
            return self.tow()

    def release_dart(self) -> None:
        try:
            s = self.dart.status()
        except DartError as e:
            raise StepFailed(f"dart unreachable for release: {e}")
        if s["phase"] in ("IDLE", "LANDED", "ARMED") and not self.dart_anchored:
            return
        if self.tel().get("state") in ("HOLD", "REEL", "JOG"):
            self.stop_settle()   # 10.1 step 14: stop FIRST (ABORTED); the worm holds the line while the dart lets go
        try:
            self.dart.release()
        except DartRefused as e:
            if e.reason.startswith("busy") or e.reason.startswith("in flight") or e.reason.startswith("fault latched"):
                # an anchor sequence still running, the dart still in FLIGHT or a latched fault: /stop ends or
                # clears them (the log is kept)
                self.dart.stop()
                self.dart.release()
            else:
                raise StepFailed(f"dart refused release: {e.reason}")
        try:
            self.dart.wait_phase(("IDLE",), self.cfg.release_timeout_s, cancel=self.cancel, log=self.log.info,
                                 want_seq="released")
        except DartError as e:
            if "cancelled" in str(e):
                raise Aborted("operator STOP")
            raise StepFailed(f"dart release did not finish: {e}. Use the manual dart keys, then Retry.")
        self.dart_anchored = False
        self.log.info("dart released")

    def retrieve(self) -> None:
        """10.1 step 15: reel in on the capstan and stop with about approach_m of line left. The base needs line_m
        (set after the landing); without any estimate this step is skipped and the take-up does all of it (slow)."""
        t = self.tel()
        if sw_closed(t, "S"):
            return
        line = t.get("line_m")
        if line is None and not self.shot.get("towed"):
            line = self.shot.get("line_est_m")   # no tow yet: the landing estimate still holds
        if line is None or line <= self.cfg.approach_m + 0.3:
            # after a tow with line_m lost (base reset), the landing estimate is too long: reeling on the capstan
            # by it would pull the dart into the cradle on the capstan (D16), so the take-up does all of it
            self.log.info("capstan reel-in skipped (line out unknown, or already short)")
            return
        self.ensure_reel()
        timeout = 30.0 + line / self.cfg.retrieve_speed_mps
        m = self.base.event_mark()
        self.go.reel(self.cfg.retrieve_duty, max_n=self.cfg.retrieve_limit_n, until="stop")
        t_end = time.monotonic() + timeout
        while True:
            self.check_cancel()
            tt = self.tel_fresh(0.5)
            cur = tt.get("line_m")
            if sw_closed(tt, "S"):
                self.stop_settle()
                self.log.warn("dart reached the cradle on the capstan (line_m estimate too long)")
                # D16 hazard: unwrapping with the dart pulled into the cradle can snatch it; let the operator slacken
                self.ask("The dart reached the cradle on the CAPSTAN (the line estimate was too long). Pay out a "
                         "little line by hand so it is slack at the cradle, then press Unwrap.",
                         ["Line slack: unwrap", "Abort"], danger=True)
                self.shot["seated_in_retrieve"] = True
                return
            if cur is not None and cur <= self.cfg.approach_m:
                self.stop_settle()
                self.log.info(f"capstan reel-in done: {cur:.2f} m left")
                return
            ev = [e for e in self.base.events(m) if e.get("evt") in ("LIMIT", "FAULT", "ABORTED")]
            if ev:
                e = ev[0]
                if e["evt"] == "LIMIT":
                    self.stop_settle()
                    raise StepFailed(f"snag while reeling in ({e.get('tension_n')} N). Free the line or jog, "
                                     "then Retry.")
                raise BaseError("reel", str(e.get("code") or e.get("why") or e["evt"]), e)
            if time.monotonic() > t_end:
                self.stop_settle()
                raise StepFailed("reel-in took too long (line_m not falling?)")
            self.sleep(0.1)

    def final_approach(self) -> None:
        """10.1 step 17: retrieve {duty 0.3, max_s} on the take-up in NEUTRAL until SEATED (FAULT DART_STUCK after
        max_s). The take-up is slow (about 0.057 m/s at duty 0.3): max_s from approach_max_s(), the firmware's own
        default formula. Never seat on the capstan."""
        t = self.tel()
        if sw_closed(t, "S"):
            return
        if self.is_wrapped(t) or self.wraps_maybe_on(t):
            self.unwrap()                       # the take-up only works with the wraps off (and only in NEUTRAL)
        line = self.tel().get("line_m")
        if line is None:
            line = self.shot.get("line_est_m")
        max_s = approach_max_s(line, self.cfg.takeup_duty)   # the firmware default (30 s unknown), sent explicitly
        m = self.base.event_mark()
        self.go.cmd("retrieve", duty=self.cfg.takeup_duty, max_n=self.cfg.retrieve_limit_n, max_s=max_s)
        self._wait_seated(m, max_s + 10.0)

    def _wait_seated(self, m: int, timeout: Optional[float] = None) -> None:
        try:
            e = self.wait_evt(("SEATED", "LIMIT"), timeout or self.cfg.final_timeout_s, m)
        except BaseError as be:
            if be.err == "DART_STUCK":
                raise StepFailed("the dart did not seat (DART_STUCK): check the cradle and the line, then Retry")
            raise
        if e.get("evt") == "LIMIT":
            self.stop_settle()
            raise StepFailed(f"snag on the final approach ({e.get('tension_n')} N): check the cradle")
        self.log.info("dart seated")

    def reshoot(self) -> None:
        self.reshoots += 1
        self.log.info(f"retrieving for reshoot {self.reshoots}")
        try:
            ph = self.dart.status()["phase"]
        except DartError:
            ph = None
        if ph in ("ANCHORING", "ANCHORED") or self.dart_anchored:
            self.dart_anchored = True
            self._do("RELEASE", self.release_dart)
        self.bring_home()                     # wrapped: reel in, unwrap, take-up; not wrapped: take-up only
        if self.reshoots > self.cfg.max_reshoots:
            self._do("LAUNCH_READY", lambda: self.base_mode(self.cfg.end_mode))
            raise Aborted(f"dart retrieved; reshoot limit ({self.cfg.max_reshoots}) reached")
        self._set_step("AIM")
        self.ask("Dart is home. Check the dart and the line, set the new target if needed, then shoot again.",
                 ["Shoot again", "Abort"])


class _Motion:
    """base.<motion>(...) through the mission's lock: STOP (abort) takes the same lock, so no motion command can be
    sent after STOP was pressed (review round 4 #2)."""

    def __init__(self, mission):
        self.m = mission

    def __getattr__(self, name):
        fn = getattr(self.m.base, name)

        def call(*a, **k):
            with self.m.fire_lock:
                self.m.check_cancel()
                return fn(*a, **k)
        return call


class StepFailed(Exception):
    """A step could not complete; choices (optional) replace Retry in the operator prompt."""

    def __init__(self, msg: str, choices: Optional[List[str]] = None):
        super().__init__(msg)
        self.choices = choices
