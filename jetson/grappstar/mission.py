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
import threading
import time
import traceback
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

from .comms.base import BaseDisconnected, BaseError, BaseTimeout
from .comms.dart import DartError, DartRefused, remap_flightlog
from .rover import OperatorRover, RoverDrive

STEPS = ["PREFLIGHT", "LAUNCH_MODE", "AIM", "COCK", "ARM", "FIRE", "FLIGHT", "LANDING_ZONE", "BASE_NEUTRAL", "WRAP",
         "ANCHOR", "TIDY", "FLY_IN", "CHECK", "TOW", "RELEASE", "RETRIEVE", "UNWRAP", "FINAL_APPROACH", "LAUNCH_READY", "DONE"]
RESHOOT_STEPS = ["BASE_NEUTRAL", "WRAP", "RETRIEVE", "UNWRAP", "FINAL_APPROACH"]
SKIPPABLE = {"CHECK", "LANDING_ZONE", "TIDY", "COCK"}
DEFAULT_DRAG_N = 0.5             # the LIGHT clutch: the one default for the CLI, settings and the App
FACTORY_K_N_PER_A = (8.0, 17.0)   # firmware defaults (base-1.0, base-1.2 physics model): not bench calibrated
BASE_FAULTS = ("STALL", "OVERCURRENT", "DRUM_TIMEOUT", "DRUM_SENSOR", "ANGLE_SENSOR", "SWITCH_DISAGREE", "BROWNOUT",
               "BAD_STATE", "FAULT", "FAULTED", "DART_STUCK")


def sw_closed(tel: dict, key: str) -> Optional[bool]:
    """Contact state from telemetry sw. PROTOCOL.md: closed contacts only, one-letter keys (S seated, H carriage home,
    L rail low, W wrap arm home, D dogs tip to tip, a b c drum code). Older long names are accepted too."""
    sw = tel.get("sw")
    if not isinstance(sw, dict):
        return None
    # base-1.2 (D31): l, r = SW_DOG_L, SW_DOG_R; I = SW_INDEX; S seated; H carriage home; L rail low; W wrap home
    long = {"S": "seated", "H": "home", "L": "rail_low", "W": "wrap_home", "l": "dog_l", "r": "dog_r", "I": "index"}
    if key in sw:
        return bool(sw[key])
    if long.get(key) in sw:
        return bool(sw[long[key]])
    return False if sw is not None else None


class Aborted(Exception):
    """The operator pressed STOP or chose Abort."""


@dataclass
class MissionConfig:
    drag_n: float = DEFAULT_DRAG_N  # payout brake setting (C3 column): LIGHT 0.5 N (coordinator)
    tow_limit_n: float = 40.0       # at most 70 N (D39)       # line tension limit while towing (base firmware caps it too)
    check_n: float = 15.0           # anchored check pull
    check_hold_s: float = 2.0       # hold at the check tension this long
    check_keep_frac: float = 0.7    # tension must stay above this fraction of the limit (load cell only)
    retrieve_limit_n: float = 30.0  # snag limit while retrieving
    reel_duty: float = 0.8
    retrieve_duty: float = 0.6
    vbat_min: float = 10.8          # 3S: 3.6 V per cell under light load
    mode_timeout_s: float = 60.0    # drum about 10.8 deg/s: 4 s per detent step, more after a wiggle
    wrap_timeout_s: float = 60.0    # about 24 s for the 4 wraps
    aim_timeout_s: float = 60.0     # one-way aim through a crank: a full aim can take about 17 s
    cock_timeout_s: float = 40.0    # COCKED in up to about 20 s
    fire_timeout_s: float = 30.0    # from rest the fire runs the whole draw first
    reset_timeout_s: float = 30.0   # RESET after FIRED
    cock_first: bool = True         # cock before the confirmations (fire is then instant); False = fire from rest
    approach_m: float = 1.5         # RETRIEVE on the capstan stops with this much line left, FINAL_APPROACH does the rest
    final_timeout_s: float = 90.0
    takeup_duty: float = 0.3        # 10.1 steps 11 and 17: take-up duty
    tidy_ms: int = 3000
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
    host_timeout_s: float = 1.5     # base host watchdog during the mission (keep-alive ping every 0.4 s)
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
                if name in ("COCK", "FIRE", "AIM"):
                    phase = None
                    try:
                        phase = self.base.status(timeout=1.0).get("phase")
                    except Exception:
                        pass
                    if phase == "UNKNOWN" or (isinstance(e, BaseError) and e.err in ("LOADED_UNKNOWN",
                                                                                  "PHASE_UNKNOWN")):
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
                    msg += ". The launcher spring may be drawn: use Decock (Manual section), then Retry."
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
                        self.decock_and_reaim()
                    except (BaseError, BaseTimeout, BaseDisconnected, StepFailed, DartError) as de:
                        self.error = f"decock and re-aim failed: {de}"
                        self.log.error(self.error)
                    continue
                return c   # a custom choice from StepFailed.choices

    def decock_and_reaim(self) -> None:
        """decock (wait RESET; ok false = nothing to undo), then aim again (the decock turns the aim train)."""
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
            self.log.info(f"decock: nothing to undo ({e.err})")
        self.shot.pop("cocked", None)
        p = self.shot.get("pitch_deg")
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
        if sw_closed(t, "S") and (self.is_wrapped(t) or (t.get("wraps") or 0) > 0):
            self._do("UNWRAP", self.unwrap)          # seated on the capstan with the wraps on: unwrap first
        if not sw_closed(t, "S"):
            if t.get("mode") == "LAUNCH":
                self._do("BASE_NEUTRAL", self.base_neutral)
            if (t.get("line_m") or self.shot.get("line_est_m") or 0) > self.cfg.approach_m + 0.5 \
                    and not self.is_wrapped(t):
                self._do("WRAP", self.wrap)
            self.bring_home()
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
            while not self._ka_stop.wait(0.4):
                try:
                    self.base.ping(timeout=0.5)
                except Exception:
                    pass
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
        if self.is_wrapped(self.tel()):
            self._do("RETRIEVE", self.retrieve)
            self._do("UNWRAP", self.unwrap)
        self._do("FINAL_APPROACH", self.final_approach)

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
            problems.append("launcher shot phase UNKNOWN (power lost mid-cycle): use Decock on the page")
        if t.get("drum_pos") in ("UNKNOWN", "BETWEEN"):
            problems.append(f"drum position {t.get('drum_pos')} (run cal or jog the drum)")
        if t.get("wraps") is None and "wraps" in t:
            problems.append("capstan wraps unknown (after a restart): run unwrap from the page, then Retry")
        elif (t.get("wraps") or 0) > 0.05:
            problems.append("capstan still wrapped (unwrap first)")
        if sw_closed(t, "S") is False:
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
                k = self.base.get("k_n_per_a").get("val")
                if k is not None and any(abs(float(k) - f) < 1e-4 for f in FACTORY_K_N_PER_A):
                    warnings.append(f"no load cell and the tension constant k_n_per_a is still the factory {k}: "
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

    def base_neutral(self) -> None:
        """After the shot: wait for the launcher RESET (mode refuses LAUNCH_BUSY until it is at rest), then NEUTRAL."""
        m0 = self.shot.pop("fire_mark", None)      # only the first attempt may look back to the shot
        t = self.tel()
        if t.get("mode") == "LAUNCH" and not (t.get("cock") == 0 and t.get("state") == "IDLE"):
            try:
                self.wait_evt("RESET", self.cfg.reset_timeout_s, m0 if m0 is not None else self.base.event_mark())
            except BaseTimeout:
                self.wait_tel(lambda t: t.get("state") == "IDLE" and t.get("cock") in (0, 0.0, None), 5.0,
                              "launcher at rest")
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
            s = self.dart.arm(True, auto=self.cfg.auto_anchor_on_landing)
            self.shot["dart_phase_at_arm"] = s["phase"]
            self.shot["dart_t_ms_at_arm"] = s.get("t_ms")
            if not s.get("armed"):
                raise StepFailed("dart did not arm")
        except DartRefused as e:
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
    def is_wrapped(t: dict) -> bool:
        """Wraps on: parked at WRAPPED (314) or flown in to REEL. Not while the drum is still laying them (WRAP)."""
        if t.get("state") in ("WRAP", "UNWRAP"):
            return False
        w = t.get("wraps")
        return t.get("drum_pos") in ("WRAPPED", "REEL") or (isinstance(w, (int, float)) and w >= 3.5)

    def ensure_reel(self, wrapped: bool = True) -> None:
        """Before a reel on the capstan: wraps on and the collar in REEL (fly-in). A fault in REEL parks the drum at
        WRAPPED, so after a clear this flies in again."""
        if wrapped and not self.is_wrapped(self.tel()):
            self.log.info("capstan not wrapped: wrapping")
            self.wrap()
        if wrapped and self.tel().get("mode") != "REEL":
            self.fly_in()

    def dart_is_out(self) -> bool:
        """True when telemetry says the dart has left the launcher (line out, not seated)."""
        t = self.tel()
        return (t.get("line_m") or 0) > 0.3 or sw_closed(t, "S") is False

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
            try:
                ph = self.dart.wait_phase(("LANDED", "ANCHORED", "IDLE", "FAULT", "ARMED"), 40.0, poll=0.3, fail=(),
                                          cancel=self.cancel)["phase"]
            except DartError as e:
                if "cancelled" in str(e):
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
        if not self.is_wrapped(t) and t.get("wraps") == 0:
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

    def tow(self) -> None:
        if self.tel().get("tension_n") is None:
            try:
                k = self.base.get("k_n_per_a").get("val")
            except (BaseError, BaseTimeout):
                k = None
            if k is not None and any(abs(float(k) - f) < 1e-4 for f in FACTORY_K_N_PER_A) \
                    and not self.cfg.allow_uncalibrated_tow:
                raise StepFailed(f"TOW refused: no load cell and k_n_per_a is still the factory {k}, so the tension limit "
                                 "cannot be trusted (a test reached 61 N against 40 N). Calibrate free_a and k_n_per_a "
                                 "(firmware/base/CALIBRATION.md b and e), then Retry; or release the dart.",
                                 choices=["Retry", "Release the dart"])
        rounds = 0
        while rounds < self.cfg.tow_max_rounds + getattr(self, "_extra_tow_rounds", 0):
            rounds += 1
            self.check_cancel()
            self.ensure_reel()
            m = self.base.event_mark()
            fresh = self.fresh_after(getattr(self, "_last_limit_t", 0.0))   # F18: the state right after LIMIT is stale
            tn_now = fresh.get("tension_n")
            if tn_now is not None and tn_now >= 0.95 * self.cfg.tow_limit_n:
                self.log.info(f"line already at {tn_now} N: not pulling again")
                r = self.rover.tow_assist(self, tn_now)
                if r == "done":
                    return
                if r == "abort":
                    raise Aborted("operator chose Abort")
                continue
            if fresh.get("state") == "REEL" and fresh.get("tension_n") is not None:
                # load cell: the firmware already winds again by itself when the held line slackens; just wait
                self.log.info("base is already winding (slack taken up by itself)")
            else:
                try:
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
            if e.reason.startswith("busy") or e.reason.startswith("in flight"):
                # an anchor sequence still running, or the dart still in FLIGHT: /stop ends both (the log is kept)
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
        if line is None:
            line = self.shot.get("line_est_m")
        if line is None or line <= self.cfg.approach_m + 0.3:
            self.log.info("capstan reel-in skipped (line out unknown or already short)")
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
        max_s). The take-up is slow (about 0.08 m/s): max_s = 20 + line_m / 0.08. Never seat on the capstan."""
        t = self.tel()
        if sw_closed(t, "S"):
            return
        if self.is_wrapped(t):
            self.unwrap()                       # the take-up only works with the wraps off
        line = self.tel().get("line_m")
        if line is None:
            line = self.shot.get("line_est_m") or self.cfg.approach_m
        max_s = min(600.0, max(5.0, round(20.0 + float(line) / 0.08, 1)))   # firmware range 5..600
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
