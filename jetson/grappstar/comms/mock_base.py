"""Simulated base station that speaks contract C1 exactly as firmware/base/PROTOCOL.md (fw "base-1.0") states it.

Run it in-process (connected to BaseClient through a pipe) or on a pseudo terminal so the real client (or a terminal
program) can open it like the USB port:

  python3 -m grappstar mockbase --pty        # prints /dev/pts/N; then: python3 -m grappstar base --port /dev/pts/N status

Commands, refusals (codes and check order), events, telemetry keys, states and phases follow PROTOCOL.md and
base_logic.h. Jetson-side notes: jetson/docs/C1_NOTES.md. The mechanics are simplified:

- Selector drum (N20 worm, absolute drum code): stop points LAUNCH 0, NEUTRAL 40, REEL 80, WRAPPED 338 drum deg at
  10.8 deg/s (about 4 s per detent step, 24 s to lay the 4 wraps). Between REEL and WRAPPED are the wrap sectors
  (drum_pos BETWEEN, mode REEL, wraps estimated). Mode changes are refused in the wrap sectors (WRAPS_ON) and when
  leaving LAUNCH with the shot phase not REST (LAUNCH_BUSY). The stop points and their order live in DRUM_DEG /
  DRUM_ORDER so a new drum layout (or new drum commands) is a table change plus a handler in _commands().
- Wiper in LAUNCH: BACKWARD aims (crank-rocker aim train, pitch 15..60 deg over a 360 deg aim cycle, one way only),
  FORWARD cocks (rest -> 560 deg), fires (release at 580 deg) and resets (720 deg = one big gear turn). Aim needs the
  train at rest, so the order is mode LAUNCH, aim, (cock, optional), arm, fire. Fire from REST runs the whole cycle.
- Wiper in REEL with the wraps on: winds the capstan. The line tension comes from the attached World (dart free,
  anchored, seated). Telemetry tension_n is the load cell (null when load_cell is False).
- Faults: inject_fault(code) or arm one with fail_next[cmd] = code (raised part way through that command's motion).
  A fault stops the wiper and sends the drum to NEUTRAL only when that is safe; otherwise the drum holds and a MODE
  event with ok false and err DRUM_HELD says why.

One deliberate difference, switchable: seat_ends_any_reel (default True) makes the seat contact end every reel with a
SEATED event. PROTOCOL.md only stops at SW_SEATED for until "seated" (with "tension" or "stop" it winds on until
max_n). Set it to False for the exact firmware behaviour.

Times run on a scaled clock (time_scale) so tests can go faster than real life.
"""
from __future__ import annotations

import json
import math
import os
import random
import threading
import time
from typing import Any, Callable, Dict, Optional

from .base import MAX_LINE, PipeEnd, pipe_pair

FW = "base-1.0"
PARAMS_VERSION = "0.1-stub"

# ---- drum (PROTOCOL.md section 3: stop points, deg from LAUNCH; the drum code is absolute)
DRUM_DEG = {"LAUNCH": 0.0, "NEUTRAL": 40.0, "REEL": 80.0, "WRAPPED": 338.0}
DRUM_ORDER = ("LAUNCH", "NEUTRAL", "REEL", "WRAPPED")   # physical order along the drum travel
DETENTS = ("LAUNCH", "NEUTRAL", "REEL")                 # the stop points "mode" goes to
N_WRAPS = 4               # wraps laid between REEL and WRAPPED
WRAP_DEG = (DRUM_DEG["WRAPPED"] - DRUM_DEG["REEL"]) / N_WRAPS   # drum travel per capstan wrap
WRAP_START_DEG = DRUM_DEG["REEL"]
MAX_WRAPS = N_WRAPS
DRUM_DPS = 10.8           # drum speed, deg/s
DRUM_WIN_DEG = 5.0        # half width of a stop point's code sector
POS_TOL = DRUM_WIN_DEG    # kept for older imports
# drum code (a = b0, b = b1, c = b2) per sector, in drum order: LAUNCH, L-N, NEUTRAL, N-R, REEL, wrap sectors, WRAPPED
SECTOR_CODE = {"LAUNCH": "a", "L-N": "ab", "NEUTRAL": "b", "N-R": "bc", "REEL": "c", "WRAP": "ac", "WRAPPED": "abc"}
# ---- aim train
PITCH_MIN, PITCH_MAX = 15.0, 60.0
AIM_CYCLE_DEG = 360.0
AIM_TOL_DEG = 0.75
AIM_DPS = 200.0           # wiper deg/s while aiming
AIM_SLOW_DPS = 110.0      # last AIM_SLOW_DEG
AIM_SLOW_DEG = 25.0
AIM_DIR, COCK_DIR, REEL_DIR = -1, +1, +1
# ---- shot cycle (wiper deg forward from rest, one big gear turn per shot)
CYCLE_DEG = 720.0
HOME_OPEN_DEG = 40.0      # SW_HOME opens (first tooth takes the rack)
COCK_END_DEG = 560.0
COCK_SLOW_FROM = 520.0
RELEASE_DEG = 580.0
RELEASE_WIN_DEG = 20.0
DECOCK_EXTRA_DEG = 15.0
COCK_DPS, COCK_SLOW_DPS = 170.0, 95.0
FIRE_DPS, RESET_DPS = 240.0, 200.0
DECOCK_DPS, JOG_DPS = 110.0, 150.0
TOKEN_S = 3.0             # arm token lifetime (armMs)
JOG_MAX_MS = 1500
# ---- reel
REEL_MPS = 0.35           # line speed at full duty with a light load (placeholder capstan gearing)
LINE_K = 400.0            # N per metre once the line is taut (stretch plus rover compliance)
LIMIT_N, LIMIT_MAX_N = 20.0, 150.0
RESUME_FRAC = 0.6         # holding winds again below this fraction of the limit (load cell only)
TENSION_IGNORE_S = 0.35
TAPER_FRAC = 0.30
REEL_DUTY = 0.86
REEL_MIN_DUTY = 0.27
SEAT_SLOW_M, SEAT_SLOW_DUTY = 1.0, 110.0 / 255.0
REEL_MAX_S = 240.0
TRIP_A, COCK_TRIP_A = 17.0, 15.0
# ---- power, protocol
VBAT_LOW, VBAT_PRESENT = 9.9, 6.0
TEL_SLOW_S, TEL_FAST_S = 0.10, 0.05

STATES = ("IDLE", "MODE", "AIM", "COCK", "FIRE", "WRAP", "UNWRAP", "REEL", "HOLD", "JOG", "DECOCK", "SAFING",
          "FAULT")
PHASES = ("REST", "COCKING", "COCKED", "FIRING", "RESETTING", "UNKNOWN")
FAULTS = ("STALL", "OVERCURRENT", "DRUM_TIMEOUT", "DRUM_SENSOR", "ANGLE_SENSOR", "SWITCH_DISAGREE", "BROWNOUT",
          "BAD_STATE")
# telemetry "sw" keys (PROTOCOL.md section 3): closed contacts only, value 1, absent = open
SW_KEYS = {"a": "SW_DRUM_A", "b": "SW_DRUM_B", "c": "SW_DRUM_C", "D": "SW_DOG", "S": "SW_SEATED", "H": "SW_HOME",
           "L": "SW_RAIL_LOW", "W": "SW_WRAP_HOME"}

# settable keys: name -> (lo, hi)
SET_KEYS = {"limit_n": (1, LIMIT_MAX_N), "k_n_per_a": (0.1, 200), "free_a": (0, 10), "amps_per_volt": (1, 200),
            "offset_a": (0, 5), "hx_n_per_count": (1e-7, 1), "drum_dps": (5, 720), "aim_tol": (0.1, 5),
            "aim_offset": (-20, 20), "aim_home_deg": (-7200, 7200), "line_m": (0, 100), "trip_a": (3, 19),
            "cock_trip_a": (3, 19), "vbat_low": (6, 12), "host_timeout_s": (0, 600)}
BOOL_KEYS = ("require_seated", "vbat_wired")


def pitch_at(phase: float, offset: float = 0.0) -> float:
    """Placeholder aim table (base_logic.h loadAimTable): pitch min at phase 0, max at half a cycle."""
    x = phase % AIM_CYCLE_DEG
    return PITCH_MIN + (PITCH_MAX - PITCH_MIN) * 0.5 * (1 - math.cos(x * 2 * math.pi / AIM_CYCLE_DEG)) + offset


def _n(v: Optional[float], dec: int):
    """A number the way JOut.num prints it: dec decimals, trailing zeros trimmed (3.00 -> 3), NaN -> null."""
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return None
    r = round(float(v), dec)
    if r == 0:
        return 0
    return int(r) if r == int(r) else r


def _dump(obj: dict) -> str:
    return json.dumps(obj, separators=(",", ":"))


class World:
    """What the line is attached to. The default is a dart sitting in the cradle.

    dart: "seated" (in the cradle), "flying", "free" (on the ground, not anchored), "anchored".
    slack_m: line that must come in before the line goes taut (anchored dart).
    """

    def __init__(self):
        self.dart = "seated"
        self.slack_m = 0.3
        self.flight: Optional[Callable[[float], float]] = None   # t since fire -> line out (m); None = default
        self.on_fire: Optional[Callable[[], None]] = None
        self.drag_free_n = 2.0      # pull needed to drag a free dart home

    def default_flight(self, t: float) -> float:
        """Line out after fire: about 10 m/s then the dart lands at about 8 m after 1 s."""
        return min(8.0, 10.0 * t * (1.0 - 0.1 * t)) if t < 1.2 else 8.0


class MockBase:
    def __init__(self, transport: Any = None, time_scale: float = 1.0, load_cell: bool = True,
                 world: Optional[World] = None, seed: int = 0, tel_hz: float = 10.0, fresh: bool = False,
                 seat_ends_any_reel: bool = True):
        """fresh=True: a new build with no NVS record (phase UNKNOWN, aim unknown: run cal cock_rest first).
        The default is a calibrated base that booted at rest, aim known, drum at NEUTRAL."""
        if transport is None:
            transport, self.client_end = pipe_pair()
        else:
            self.client_end = None
        self.tr = transport
        self.scale = float(time_scale)
        self.load_cell = load_cell
        self.world = world or World()
        self.rng = random.Random(seed)
        self.tel_hz = tel_hz
        self.seat_ends_any_reel = seat_ends_any_reel
        self.lock = threading.RLock()
        self._txlock = threading.Lock()
        # ---- live state (firmware names in comments)
        self.state = "IDLE"
        self.fault: Optional[str] = None
        self.fault_why = ""
        self.fault_present = False          # test hook: clear answers STILL_PRESENT while True
        self.booted = False
        self.drum_deg = DRUM_DEG["NEUTRAL"]
        self.drum_known = True
        self.wraps_known = True
        self.aim_phase = 0.0                # truth: aim train phase (deg of wiper travel in the aim direction)
        self.aim_known = not fresh
        self.aim_offset = 0.0
        self.aimed = False
        self.aim_target: Optional[float] = None
        self.cock_deg = 0.0                 # truth: launch train angle inside the shot cycle
        self.phase = "UNKNOWN" if fresh else "REST"
        self.token: Optional[int] = None
        self.token_t = 0.0
        self.t_fire: Optional[float] = None
        self.line_m = 0.0                   # truth: line out (m)
        self.line_est = 0.0                 # firmware estimate (lineM)
        self.line_known = True
        self.tension = 0.0                  # truth: line tension (N)
        self.wiper_deg = 0.0
        self.rpm = 0.0
        self.amps = 0.0
        self.peak_a = 0.0
        self.vbat0 = 12.4
        self.vbat = 12.4
        self.require_seated = True
        self.vbat_wired = True
        self.enc_sign = 1
        self.settings: Dict[str, float] = {"limit_n": LIMIT_N, "k_n_per_a": 8.0, "free_a": 1.5,
                                           "amps_per_volt": 12.5, "offset_a": 0.0, "hx_n_per_count": 1 / 21000.0,
                                           "drum_dps": DRUM_DPS, "aim_tol": AIM_TOL_DEG, "aim_offset": 0.0,
                                           "aim_home_deg": 3365.0, "trip_a": TRIP_A, "cock_trip_a": COCK_TRIP_A,
                                           "vbat_low": VBAT_LOW, "host_timeout_s": 0.0}
        self.code_bad = False               # test hook: drum code invalid (drum_pos UNKNOWN, POSITION_UNKNOWN)
        self.dog_tip = False                # test hook: SW_DOG closed (dogs tip to tip)
        self.last_rx_t = 0.0                # sim time of the last line from the host (host watchdog)
        self.commands = self._commands()
        self.op: Dict[str, Any] = {}
        self._seat_prev = self.seated()
        # ---- test hooks
        self.fail_next: Dict[str, str] = {}   # command name -> fault code raised part way through its motion
        self.ignore: set = set()              # command names that get no reply (timeout tests)
        self.silent = False                   # stop sending telemetry
        self.received: list = []              # every command object received
        self.tx_max = 0                       # longest line sent (bytes, without the newline)
        self.sim_t = 0.0
        self._stop = threading.Event()
        self._threads: list = []

    # ------------------------------------------------ compatibility views
    @property
    def mode(self) -> str:
        """modeName(): derived from the drum position, NONE between detents."""
        if self.code_bad:
            return "NONE"
        k = self.detent()
        if k in ("LAUNCH", "NEUTRAL"):
            return k
        if self.drum_deg >= DRUM_DEG["REEL"] - DRUM_WIN_DEG:
            return "REEL"                    # REEL, the wrap sectors and WRAPPED: the collar stays in REEL
        return "NONE"

    @mode.setter
    def mode(self, to: str) -> None:       # tests that place the drum by hand
        if to in DRUM_DEG:
            self.drum_deg = DRUM_DEG[to]
            self.drum_known = True

    @property
    def pitch(self) -> Optional[float]:
        return pitch_at(self.aim_phase, self.aim_offset) if self.aim_known else None

    @property
    def cock(self) -> Optional[float]:
        p = self.phase
        if p == "REST":
            return 0.0
        if p in ("COCKED", "FIRING"):
            return 1.0
        if p == "COCKING":
            return min(1.0, max(0.0, self.cock_deg / COCK_END_DEG))
        if p == "RESETTING":
            return max(0.0, 1 - (self.cock_deg - RELEASE_DEG) / (CYCLE_DEG - RELEASE_DEG))
        return None

    # ------------------------------------------------ run
    def start(self) -> "MockBase":
        self._send({"evt": "HELLO", "fw": FW, "reset": "POWERON", "hx711": bool(self.load_cell), "as5600": True,
                    "pwm": True})
        for fn, name in ((self._rx_loop, "mock-base-rx"), (self._sim_loop, "mock-base-sim")):
            th = threading.Thread(target=fn, name=name, daemon=True)
            th.start()
            self._threads.append(th)
        return self

    def stop(self) -> None:
        self._stop.set()
        for th in self._threads:
            th.join(timeout=1)
        try:
            self.tr.close()
        except Exception:
            pass

    def _send_line(self, line: str) -> None:
        self.tx_max = max(self.tx_max, len(line))
        try:
            with self._txlock:
                self.tr.write(line.encode() + b"\n")
        except Exception:
            pass

    def _send(self, obj: dict) -> None:
        self._send_line(_dump(obj))

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                line = self.tr.readline(0.1)
            except Exception:
                time.sleep(0.05)
                continue
            if not line:
                continue
            with self.lock:
                try:
                    reply = self.handle_line(line)
                except Exception as e:  # never crash the mock
                    reply = _dump({"id": None, "ok": False, "err": "BAD_ARG", "msg": str(e)[:60]})
            if reply is not None:
                self._send_line(reply)

    def _sim_loop(self) -> None:
        dt_real = 0.02
        last_tel = 0.0
        while not self._stop.is_set():
            time.sleep(dt_real)
            with self.lock:
                self.step(dt_real * self.scale)
                period = (TEL_FAST_S if self.moving() else TEL_SLOW_S) * 10.0 / self.tel_hz
                now = time.monotonic()
                line = None
                if not self.silent and now - last_tel >= period * 0.999:
                    last_tel = now
                    line = self.telemetry_line()
            if line:
                self._send_line(line)

    # ------------------------------------------------ model: positions and contacts
    def now_ms(self) -> int:
        return int(self.sim_t * 1000)

    def detent(self) -> Optional[str]:
        """The stop point whose code sector the drum is in (LAUNCH, NEUTRAL, REEL, WRAPPED), or None."""
        if self.code_bad:
            return None
        for name in DRUM_ORDER:
            if abs(self.drum_deg - DRUM_DEG[name]) <= DRUM_WIN_DEG:
                return name
        return None

    def in_wrap_sectors(self) -> bool:
        return self.drum_deg > DRUM_DEG["REEL"] + DRUM_WIN_DEG

    def sector(self) -> str:
        k = self.detent()
        if k:
            return k
        d = self.drum_deg
        if d < DRUM_DEG["NEUTRAL"]:
            return "L-N"
        if d < DRUM_DEG["REEL"]:
            return "N-R"
        return "WRAP"

    def wraps(self) -> int:
        if self.drum_deg <= DRUM_DEG["REEL"] + DRUM_WIN_DEG:
            return 0
        if self.drum_deg >= DRUM_DEG["WRAPPED"] - DRUM_WIN_DEG:
            return N_WRAPS
        return max(0, min(N_WRAPS - 1, int((self.drum_deg - DRUM_DEG["REEL"]) // WRAP_DEG)))

    def drum_pos(self) -> str:
        if self.code_bad:
            return "UNKNOWN"
        return self.detent() or "BETWEEN"

    def seated(self) -> bool:
        return self.world.dart == "seated" and self.line_m <= 0.01

    def home_closed(self) -> bool:
        """SW_HOME: carriage fully forward (before the first tooth, or after the release until the cycle ends)."""
        return self.cock_deg < HOME_OPEN_DEG or self.cock_deg >= RELEASE_DEG

    def sw(self) -> dict:
        out = {}
        if not self.code_bad:
            for key in SECTOR_CODE[self.sector()]:
                out[key] = 1
        if self.dog_tip:
            out["D"] = 1
        if self.seated():
            out["S"] = 1
        if self.home_closed():
            out["H"] = 1
        if self.aim_phase % AIM_CYCLE_DEG < 8:
            out["L"] = 1                     # rail at its low dead point
        if not self.in_wrap_sectors():
            out["W"] = 1                     # wrap arm at home
        return {k2: out[k2] for k2 in SW_KEYS if k2 in out}

    def moving(self) -> bool:
        return self.state not in ("IDLE", "HOLD", "FAULT")

    # ------------------------------------------------ telemetry and events
    def telemetry(self, lean: bool = False) -> dict:
        d1, d2 = (0, 1) if lean else (1, 2)
        return {"state": self.state, "mode": self.mode,
                "drum_deg": _n(self.drum_deg, d1) if self.drum_known else None,
                "drum_pos": self.drum_pos(), "wiper_deg": _n(self.wiper_deg, d1), "rpm": _n(self.rpm, d1),
                "amps": _n(self.amps, d1), "pitch_deg": _n(self.pitch, d2), "cock": _n(self.cock, 2),
                "wraps": self.wraps() if self.wraps_known else None,
                "line_m": _n(self.line_est, d2) if self.line_known else None,
                "tension_n": _n(self.tension, d1) if self.load_cell else None,
                "sw": self.sw(), "fault": self.fault, "vbat": _n(self.vbat, 1)}

    def telemetry_line(self) -> str:
        for lean in (False, True):
            s = _dump({"tel": self.telemetry(lean)})
            if len(s) <= MAX_LINE:
                return s
        return _dump({"tel": {"state": self.state, "mode": self.mode, "drum_pos": self.drum_pos(),
                              "fault": self.fault, "trunc": True}})

    def evt(self, name: str, **kw) -> None:
        d = {"evt": name, "t": self.now_ms()}
        d.update(kw)
        s = _dump(d)
        self._send_line(s if len(s) <= MAX_LINE else '{"evt":"OVERFLOW"}')

    def _ev_mode(self, ok: bool, err: Optional[str] = None, why: Optional[str] = None) -> None:
        kw: Dict[str, Any] = {"ok": ok, "mode": self.mode, "drum_pos": self.drum_pos()}
        if err:
            kw["err"] = err
        if why:
            kw["why"] = why
        self.evt("MODE", **kw)

    def _ev_aimed(self, ok: bool, err: Optional[str] = None) -> None:
        kw: Dict[str, Any] = {"ok": ok, "pitch_deg": _n(self.pitch, 2), "target_deg": _n(self.aim_target, 2)}
        if err:
            kw["err"] = err
        self.evt("AIMED", **kw)

    # ------------------------------------------------ faults
    def _hold_reason(self, code: Optional[str]) -> Optional[str]:
        """safeToNeutral(): None if the drum may go to NEUTRAL now, else why it must hold."""
        if code in ("DRUM_TIMEOUT", "DRUM_SENSOR"):
            return "drum fault"
        if self.code_bad:
            return "drum code invalid"
        if self.in_wrap_sectors():
            return "wraps on: the capstan holds the line"
        if self.drum_pos() == "LAUNCH" and self.phase != "REST":
            return "launch train not at rest: the wiper worm holds the spring (a shift under load could dry fire)"
        return None

    def raise_fault(self, code: str, drum_to_neutral: bool = True, why: Optional[str] = None) -> None:
        self.rpm = 0.0
        self.amps = 0.0
        if self.state in ("FAULT", "SAFING"):
            return                           # the first fault wins
        why = why or {"STALL": "wiper stalled", "OVERCURRENT": "wiper over 17 A", "DRUM_TIMEOUT":
                      "drum did not reach the detent in time", "DRUM_SENSOR": "drum code seen on the way",
                      "ANGLE_SENSOR": "AS5600 lost while the wiper runs", "SWITCH_DISAGREE":
                      "drum code bumps give an impossible code", "BROWNOUT": "battery sagged under load",
                      "BAD_STATE": "bad state"}.get(code, code)
        self.fault, self.fault_why = code, why[:47]
        self.token = None
        self.evt("FAULT", code=code, why=self.fault_why)
        if code == "STALL":
            self.evt("STALL")
        hold = self._hold_reason(code) if drum_to_neutral else "drum fault"
        if not hold and self.detent() != "NEUTRAL":
            self._start_drum(DRUM_DEG["NEUTRAL"], "safing", self.fail_next.pop("safing", None))
            self.state = "SAFING"
            return
        self.op = {}
        self.state = "FAULT"
        if hold and self.detent() != "NEUTRAL":
            self._ev_mode(False, "DRUM_HELD", hold)

    def inject_fault(self, code: str, present: bool = False, why: Optional[str] = None) -> None:
        with self.lock:
            self.fault_present = present
            self.raise_fault(code, why=why)

    # ------------------------------------------------ simulation
    def _start_drum(self, target: float, kind: str, fail: Optional[str] = None) -> None:
        self.op = {"kind": kind, "target": target, "from": self.drum_deg, "fail": fail}

    def step(self, dt: float) -> None:
        while dt > 1e-9:
            h = min(dt, 0.01)
            self._substep(h)
            dt -= h

    def _substep(self, dt: float) -> None:
        self.sim_t += dt
        if not self.booted and self.sim_t >= 0.06:
            self.booted = True
            self.evt("BOOT", fw=FW, record=True, was_moving=False, drum_known=self.drum_known,
                     wraps_known=self.wraps_known, aim_known=self.aim_known, phase=self.phase)
        self._world_step(dt)
        if self.token is not None and self.sim_t - self.token_t > TOKEN_S:
            self.token = None
        self.rpm = 0.0
        self.amps = 0.0
        st = self.state
        ht = self.settings["host_timeout_s"]
        if ht > 0 and st not in ("IDLE", "HOLD", "FAULT", "SAFING") and self.sim_t - self.last_rx_t > ht:
            self._halt()                     # host watchdog
            self.evt("ABORTED", what="HOST", why="host silent")
            return
        if st in ("MODE", "SAFING", "WRAP", "UNWRAP"):
            self._drum_step(dt)
        elif st == "AIM":
            self._aim_step(dt)
        elif st == "COCK":
            self._cock_step(dt)
        elif st == "FIRE":
            self._fire_step(dt)
        elif st == "DECOCK":
            self._decock_step(dt)
        elif st == "REEL":
            self._reel_step(dt)
        elif st == "HOLD":
            self._hold_step(dt)
        elif st == "JOG":
            self._jog_step(dt)
        elif st == "IDLE":
            if self.world.dart != "anchored" and self.world.dart != "flying":
                self.tension = 0.0 if self.line_m <= 0 or self.world.dart == "seated" else min(self.tension, 0.5)
        self.peak_a = max(self.peak_a, self.amps)
        self.vbat = self.vbat0 - 0.05 * self.amps
        if self.state == "REEL" and self.amps > self.settings["trip_a"]:
            if self.op.get("until") == "tension":
                self._enter_limit("over current at the limit")
            else:
                self.raise_fault("OVERCURRENT")
        self._seat_prev = self.seated()

    def _world_step(self, dt: float) -> None:
        w = self.world
        if self.t_fire is not None and w.dart == "flying":
            t = self.sim_t - self.t_fire
            f = w.flight or w.default_flight
            self.line_m = max(self.line_m, f(t))
            self.tension = 0.8
            if t > 1.5 and w.flight is None:
                w.dart = "free"

    def _wiper(self, direction: int, dps: float, amps: float) -> float:
        """Turn the wiper for one step; returns the travel (deg)."""
        self.rpm = direction * dps / 6.0
        self.amps = amps
        return dps

    def _drum_step(self, dt: float) -> None:
        op = self.op
        if "target" not in op:
            self.state = "IDLE"
            return
        target = op["target"]
        d = target - self.drum_deg
        stepd = self.settings["drum_dps"] * dt
        fail = op.get("fail")
        if fail:
            span = abs(target - op["from"]) or 1.0
            if abs(self.drum_deg - op["from"]) >= 0.4 * span:
                op["fail"] = None
                if op["kind"] == "safing":
                    self.state = "FAULT"
                    self.op = {}
                    self._ev_mode(False, fail, "safing move failed: drum did not reach NEUTRAL")
                else:                        # mode, wrap, unwrap, a failed estop move: a drum fault
                    self.raise_fault(fail)
                return
        if abs(d) > stepd:
            self.drum_deg += stepd if d > 0 else -stepd
            return
        self.drum_deg = target
        kind = op["kind"]
        self.op = {}
        if kind == "safing":
            self.state = "FAULT"
            self._ev_mode(True)
        elif kind == "wrap":
            self.state = "IDLE"
            self.evt("WRAPPED", ok=True, wraps=self.wraps())
        elif kind == "unwrap":
            self.state = "IDLE"
            self.evt("UNWRAPPED", ok=True)
        else:                                # mode, estop
            self.state = "IDLE"
            self._ev_mode(True)

    def aim_travel_to(self, target: float) -> float:
        """Smallest forward aim travel that brings the pitch to target (-1 if never), exact to 1e-6 deg."""
        f = lambda t: pitch_at(self.aim_phase + t, self.aim_offset) - target
        prev = f(0.0)
        if abs(prev) <= self.settings["aim_tol"] * 0.25:
            return 0.0
        t, step = 0.0, 0.25
        while t <= AIM_CYCLE_DEG + step:
            v = f(t + step)
            if v == 0 or (v > 0) != (prev > 0):
                lo, hi = t, t + step
                for _ in range(40):
                    mid = 0.5 * (lo + hi)
                    if (f(mid) > 0) == (prev > 0):
                        lo = mid
                    else:
                        hi = mid
                return 0.5 * (lo + hi)
            prev, t = v, t + step
        return -1.0

    def _aim_step(self, dt: float) -> None:
        op = self.op
        if op.get("settle") is not None:
            if self.sim_t - op["settle"] >= 0.15:
                self.aimed = True
                self.state = "IDLE"
                self.op = {}
                self._ev_aimed(True)
            return
        if op.get("homing"):
            trav = self._wiper(AIM_DIR, AIM_SLOW_DPS, 2.0) * dt
            before = (self.aim_phase - self.settings["aim_home_deg"]) % AIM_CYCLE_DEG
            self.aim_phase = (self.aim_phase + trav) % AIM_CYCLE_DEG
            self.wiper_deg += AIM_DIR * trav
            after = (self.aim_phase - self.settings["aim_home_deg"]) % AIM_CYCLE_DEG
            if after < before:               # SW_RAIL_LOW rose
                self.aim_known = True
                op["homing"] = False
                op["left"] = self.aim_travel_to(self.aim_target)
                if op["left"] < 0:
                    self.state = "IDLE"
                    self.op = {}
                    self._ev_aimed(False, "OUT_OF_RANGE")
            return
        left = op["left"]
        dps = AIM_SLOW_DPS if left < AIM_SLOW_DEG else AIM_DPS
        trav = min(left, self._wiper(AIM_DIR, dps, 2.5) * dt)
        self.aim_phase = (self.aim_phase + trav) % AIM_CYCLE_DEG
        self.wiper_deg += AIM_DIR * trav
        op["left"] = left - trav
        f = self.fail_next.pop("_aim_now", None) if op["left"] < op["total"] * 0.5 else None
        if f:
            self.raise_fault(f)
            return
        if op["left"] <= 1e-9:
            op["settle"] = self.sim_t

    def _cock_step(self, dt: float) -> None:
        if self.phase == "UNKNOWN":          # recovery creep until SW_HOME opens
            trav = self._wiper(COCK_DIR, COCK_SLOW_DPS, 2.0) * dt
            self.cock_deg += trav
            self.wiper_deg += trav
            if not self.home_closed():
                self.phase = "COCKING"
            return
        dps = COCK_SLOW_DPS if self.cock_deg >= COCK_SLOW_FROM else COCK_DPS
        load = 0.5 if self.cock_deg < HOME_OPEN_DEG else 2.0 + 6.0 * (self.cock_deg - HOME_OPEN_DEG) / (
            RELEASE_DEG - HOME_OPEN_DEG)
        trav = min(COCK_END_DEG - self.cock_deg, self._wiper(COCK_DIR, dps, load) * dt)
        self.cock_deg += trav
        self.wiper_deg += trav
        if self.cock_deg >= 300.0:
            f = self.fail_next.pop("_cock_now", None)
            if f:
                self.raise_fault(f)
                return
        if self.cock_deg >= COCK_END_DEG - 1e-6:
            self.cock_deg = COCK_END_DEG
            self.phase = "COCKED"
            self.state = "IDLE"
            self.evt("COCKED", ok=True, cock_deg=_n(self.cock_deg, 1))

    def _fire_step(self, dt: float) -> None:
        if self.phase == "FIRING":
            f = self.fail_next.pop("_fire_now", None)
            if f:
                self.raise_fault(f)
                return
            trav = self._wiper(COCK_DIR, FIRE_DPS, 8.0) * dt   # from COCKED, or the whole cycle from REST
            before = self.cock_deg
            self.cock_deg += trav
            self.wiper_deg += trav
            if before < RELEASE_DEG <= self.cock_deg:
                self.phase = "RESETTING"
                self.aimed = False
                self.line_known = False
                self.t_fire = self.sim_t
                self.world.dart = "flying"
                self.evt("FIRED", pitch_deg=_n(self.pitch, 2), cock_deg=_n(RELEASE_DEG, 1))
                if self.world.on_fire:
                    try:
                        self.world.on_fire()
                    except Exception:
                        pass
            return
        trav = self._wiper(COCK_DIR, RESET_DPS, 1.5) * dt
        self.cock_deg += trav
        self.wiper_deg += trav
        if self.cock_deg >= CYCLE_DEG - 1.0:
            self.cock_deg = 0.0
            self.phase = "REST"
            self.state = "IDLE"
            self.evt("RESET", ok=True)

    def _decock_step(self, dt: float) -> None:
        op = self.op
        trav = self._wiper(AIM_DIR, DECOCK_DPS, 2.0) * dt
        self.wiper_deg += AIM_DIR * trav
        self.aim_phase = (self.aim_phase + trav) % AIM_CYCLE_DEG
        if op.get("home_at") is None:
            self.cock_deg = max(HOME_OPEN_DEG - 1.0, self.cock_deg - trav)
            if self.cock_deg <= HOME_OPEN_DEG - 1.0:
                op["home_at"] = self.wiper_deg
            return
        if abs(self.wiper_deg - op["home_at"]) >= DECOCK_EXTRA_DEG:
            self.phase = "REST"
            self.cock_deg = HOME_OPEN_DEG - 2.0
            self.state = "IDLE"
            self.op = {}
            self.evt("RESET", ok=True, how="decock")

    def _reel_tension(self, speed: float, dt: float) -> float:
        """Advance the line for one step of winding at speed (m/s at no load). Returns the capstan line travel."""
        w = self.world
        if w.dart == "anchored" or (w.dart == "seated" and self.line_m <= 0.01):
            if w.dart == "anchored" and w.slack_m > 0:
                take = min(w.slack_m, speed * dt)
                w.slack_m -= take
                self.line_m = max(0.0, self.line_m - take)
                self.tension = 1.0
                return take
            # taut: tension rises with the wiper torque, speed falls (worm drive, stalls at the limit)
            self.tension = min(self.tension + LINE_K * speed * dt * 0.3, 400.0)
            take = 0.002 * dt
            self.line_m = max(0.0, self.line_m - take)
            return take
        take = min(self.line_m, speed * dt)
        self.line_m = max(0.0, self.line_m - take)
        if self.line_m > 0:
            self.tension = w.drag_free_n + self.rng.uniform(-0.3, 0.3)
        else:
            self.tension = 0.0
            if w.dart != "seated":
                w.dart = "seated"
        return take

    def _enter_limit(self, why: str) -> None:
        op = self.op
        holding = op.get("until") == "tension"
        self.state = "HOLD" if holding else "IDLE"
        self.rpm = 0.0
        self.amps = 0.0
        self.evt("LIMIT", tension_n=_n(self.tension, 1), max_n=_n(op.get("max_n", LIMIT_N), 1), why=why,
                 holding=holding)
        if not holding:
            self.op = {}

    def _reel_step(self, dt: float) -> None:
        op = self.op
        if "until" not in op:
            self.state = "IDLE"
            return
        seat_rose = self.seated() and not self._seat_prev
        if self.seated() and (op["until"] == "seated" and (seat_rose or op.get("seat_watch"))
                              or self.seat_ends_any_reel):
            self._seated_now()
            return
        max_n = op["max_n"]
        frac = self.tension / max_n if max_n > 0 else 0.0
        if frac >= 1.0 and self.sim_t - op["t0"] > TENSION_IGNORE_S:
            self._enter_limit("tension limit")
            return
        duty = op["duty"]
        if frac > 1 - TAPER_FRAC:
            duty = max(REEL_MIN_DUTY, duty * (1 - frac) / TAPER_FRAC)
        if op["until"] == "seated" and self.line_known and self.line_est < SEAT_SLOW_M:
            duty = min(duty, SEAT_SLOW_DUTY)
        speed = REEL_MPS * duty * max(0.1, 1.0 - self.tension / 300.0)
        take = self._reel_tension(speed, dt)
        self.line_est = max(0.0, self.line_est - take)
        dps = speed / (math.pi * 0.03) * 360.0
        self._wiper(REEL_DIR, dps, 1.5 + self.tension * 0.12)
        self.wiper_deg += dps * dt
        op["seat_watch"] = True
        if self.sim_t - op["t0"] > 0.3:
            f = self.fail_next.pop("_reel_now", None)
            if f:
                self.raise_fault(f)
                return
        if self.sim_t - op["t_start"] > REEL_MAX_S:
            self.state = "IDLE"
            self.op = {}
            self.evt("ABORTED", what="REEL", why="timeout")

    def _seated_now(self) -> None:
        self.state = "IDLE"
        self.op = {}
        self.tension = 0.0
        self.line_est, self.line_known = 0.0, True
        self.evt("SEATED")

    def _hold_step(self, dt: float) -> None:
        w = self.world
        if w.dart == "anchored":
            if w.slack_m > 0:
                self.tension = 1.0
        else:
            self.tension = w.drag_free_n if self.line_m > 0 else 0.0
        op = self.op
        if (self.load_cell and op.get("until") == "tension" and "max_n" in op
                and self.tension < RESUME_FRAC * op["max_n"]):
            op["t0"] = self.sim_t                # the load cell shows the line slackened: wind again
            self.state = "REEL"

    def _jog_step(self, dt: float) -> None:
        op = self.op
        if "motor" not in op:
            self.state = "IDLE"
            return
        if op["motor"] == "wiper":
            p = self.drum_pos()
            trav = self._wiper(op["dir"], JOG_DPS, 1.5) * dt
            self.wiper_deg += op["dir"] * trav
            if p == "LAUNCH" and op["dir"] == AIM_DIR:
                self.aim_phase = (self.aim_phase + trav) % AIM_CYCLE_DEG
            elif p == "LAUNCH" and op["dir"] == COCK_DIR:
                was_home = self.home_closed()
                self.cock_deg += trav
                if was_home and not self.home_closed() and self.phase == "REST":
                    self.phase = "COCKING"
                if self.cock_deg >= COCK_END_DEG - RELEASE_WIN_DEG:
                    self.state = "IDLE"
                    self.op = {}
                    self.evt("ABORTED", what="JOG", why="would pass the release tooth")
                    return
            elif self.mode == "REEL" and self.wraps() > 0:
                take = self._reel_tension(trav / 360.0 * math.pi * 0.03, 1.0)
                self.line_est = max(0.0, self.line_est - take)
        else:
            self.drum_deg += op["dir"] * self.settings["drum_dps"] * dt
        if self.sim_t >= op["t_end"]:
            self.state = "IDLE"
            self.op = {}

    # ------------------------------------------------ protocol
    def handle_line(self, raw: bytes) -> Optional[str]:
        """One line in, exactly one reply line out (None only for a command in self.ignore)."""
        if len(raw) > MAX_LINE:
            return '{"id":null,"ok":false,"err":"TOO_LONG"}'
        try:
            m = json.loads(raw.decode("utf-8"))
            if not isinstance(m, dict):
                raise ValueError
        except Exception:
            return '{"id":null,"ok":false,"err":"BAD_JSON"}'
        self.last_rx_t = self.sim_t
        self.received.append(m)
        i = m.get("id")
        if isinstance(i, bool) or not isinstance(i, (int, float, str)):
            i = None
        elif isinstance(i, float) and i == int(i):
            i = int(i)
        elif isinstance(i, str):
            i = i[:63]
        c = m.get("cmd")
        if not isinstance(c, str):
            return _dump({"id": i, "ok": False, "err": "NO_CMD"})
        if c in self.ignore:
            return None
        extra: Dict[str, Any] = {}
        err = self.handle(m, extra)
        r: Dict[str, Any] = {"id": i, "ok": err is None}
        if err:
            r["err"] = err
        r.update(extra)
        s = _dump(r)
        if len(s) > MAX_LINE:
            short: Dict[str, Any] = {"id": i if not isinstance(i, str) else i[:23], "ok": err is None}
            if err:
                short["err"] = err
            elif "token" in extra:
                short["token"] = extra["token"]
            s = _dump(short)
        return s

    def _idle_check(self, need_enc: bool = False) -> Optional[str]:
        if self.state in ("FAULT", "SAFING"):
            return "FAULTED"
        if self.state not in ("IDLE", "HOLD"):
            return "BUSY"
        if not self.booted:
            return "BOOTING"
        if self.vbat_wired and self.vbat < VBAT_PRESENT:
            return "NO_POWER"
        return None

    @staticmethod
    def _num(m: dict, k: str) -> Optional[float]:
        v = m.get(k)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            return None
        return float(v)

    def _commands(self) -> Dict[str, Callable[[dict, dict], Optional[str]]]:
        """Command name -> handler(msg, reply_extra) returning an error code or None. Add new commands here."""
        return {"ping": self._c_ping, "status": self._c_status,
                "stop": lambda m, x: self._cmd_stop(False), "estop": lambda m, x: self._cmd_stop(True),
                "clear": lambda m, x: self._cmd_clear(),
                "mode": lambda m, x: self._cmd_mode(m["to"]) if isinstance(m.get("to"), str) else "BAD_ARG",
                "aim": self._c_aim, "cock": lambda m, x: self._cmd_cock(), "arm": self._c_arm, "fire": self._c_fire,
                "wrap": lambda m, x: self._cmd_wrap(), "unwrap": lambda m, x: self._cmd_unwrap(),
                "reel": self._c_reel, "jog": self._c_jog,
                "cal": lambda m, x: self._cmd_cal(m["what"], m) if isinstance(m.get("what"), str) else "BAD_ARG",
                "set": self._c_set, "get": self._c_get}

    def handle(self, m: dict, extra: Optional[dict] = None) -> Optional[str]:
        """Run one command. Returns an error code or None; reply fields go into extra."""
        extra = {} if extra is None else extra
        fn = self.commands.get(m.get("cmd"))
        return fn(m, extra) if fn else "UNKNOWN_CMD"

    def _c_ping(self, m: dict, extra: dict) -> Optional[str]:
        extra.update(fw=FW, t=self.now_ms())
        return None

    def _c_status(self, m: dict, extra: dict) -> Optional[str]:
        extra.update(phase=self.phase, aimed=self.aimed, aim_known=self.aim_known, drum_known=not self.code_bad,
                     wraps_known=not self.code_bad, magnet="OK", glitches=0, hx=bool(self.load_cell),
                     peak_a=_n(self.peak_a, 1), fw=FW)
        if self.fault:
            extra["why"] = self.fault_why[:32]
        return None

    def _c_aim(self, m: dict, extra: dict) -> Optional[str]:
        deg = self._num(m, "deg")
        return "BAD_ARG" if deg is None else self._cmd_aim(deg)

    def _c_arm(self, m: dict, extra: dict) -> Optional[str]:
        e = self._fire_check()
        if e:
            return e
        self.token = self.rng.randrange(100000, 1000000)     # a new arm replaces the old token
        self.token_t = self.sim_t
        extra.update(token=self.token, valid_ms=int(TOKEN_S * 1000))
        return None

    def _c_fire(self, m: dict, extra: dict) -> Optional[str]:
        tok = self._num(m, "token")
        if tok is None and isinstance(m.get("token"), str):
            try:
                tok = float(m["token"])
            except ValueError:
                tok = 0.0
        return "BAD_ARG" if tok is None else self._cmd_fire(tok)

    def _c_reel(self, m: dict, extra: dict) -> Optional[str]:
        duty = self._num(m, "duty")
        max_n = self._num(m, "max_n")
        until = m.get("until") if isinstance(m.get("until"), str) else None
        return self._cmd_reel(REEL_DUTY if duty is None else duty,
                              self.settings["limit_n"] if max_n is None else max_n, until)

    def _c_jog(self, m: dict, extra: dict) -> Optional[str]:
        ms = self._num(m, "ms")
        d = self._num(m, "dir")
        motor = m.get("motor")
        if not isinstance(motor, str) or d is None:
            return "BAD_ARG"
        return self._cmd_jog(motor, d, 300.0 if ms is None else ms)

    def _c_set(self, m: dict, extra: dict) -> Optional[str]:
        k = m.get("key")
        v = m.get("val")
        if isinstance(v, bool):
            v = 1.0 if v else 0.0
        if not isinstance(k, str) or not isinstance(v, (int, float)):
            return "BAD_ARG"
        return self._cmd_set(k, float(v))

    def _c_get(self, m: dict, extra: dict) -> Optional[str]:
        k = m.get("key")
        if not isinstance(k, str):
            return "BAD_ARG"
        return None if self._get_key(k, extra) else "BAD_KEY"

    # ---- commands (base_logic.h cmdXxx, same checks in the same order)
    def _halt(self) -> None:
        """Wiper and drum off now; a stop during cock or fire makes the phase UNKNOWN."""
        self.token = None
        if self.state in ("COCK", "FIRE"):
            self.phase = "UNKNOWN"
        self.state = "IDLE"
        self.op = {}
        self.rpm = self.amps = 0.0

    def _cmd_stop(self, estop: bool) -> None:
        self.token = None
        was = self.state
        if was in ("FAULT", "SAFING"):
            self.state = "FAULT"
            self.op = {}
        else:
            if was != "IDLE":
                self.evt("ABORTED", what=was, why="estop" if estop else "stop")
            if was in ("COCK", "FIRE"):
                self.phase = "UNKNOWN"
            self.state = "IDLE"
            self.op = {}
        if estop and self.state != "FAULT":
            hold = self._hold_reason(None)
            if not hold and self.detent() != "NEUTRAL":
                self._start_drum(DRUM_DEG["NEUTRAL"], "estop")
                self.state = "MODE"
            elif hold and self.detent() != "NEUTRAL":
                self._ev_mode(False, "DRUM_HELD", hold)

    def _cmd_clear(self) -> Optional[str]:
        if self.state != "FAULT":
            return "BUSY" if self.state == "SAFING" else None
        if self.fault_present:
            return "STILL_PRESENT"
        self.fault, self.fault_why = None, ""
        self.state = "IDLE"
        return None

    def _cmd_mode(self, to: str) -> Optional[str]:
        if to not in DETENTS:
            return "BAD_ARG"
        e = self._idle_check()
        if e:
            return e
        if self.state == "HOLD":
            return "HOLDING"
        if self.in_wrap_sectors():
            return "WRAPS_ON"
        if self.code_bad:
            return "POSITION_UNKNOWN"
        k = self.detent()
        if k == to:
            self._ev_mode(True)
            return None
        if k == "LAUNCH" and self.phase != "REST":
            return "LAUNCH_BUSY"
        if k == "LAUNCH":
            self.wiper_deg -= 3.0            # unload the dogs: 3 deg backward before leaving LAUNCH
        self.aimed = False
        self.token = None
        self._start_drum(DRUM_DEG[to], "mode", self.fail_next.pop("mode", None))
        self.state = "MODE"
        return None

    def _cmd_aim(self, deg: float) -> Optional[str]:
        e = self._idle_check(True)
        if e:
            return e
        if self.state == "HOLD":
            return "HOLDING"
        if self.detent() != "LAUNCH":
            return "NOT_IN_LAUNCH"
        if self.phase == "UNKNOWN":
            return "PHASE_UNKNOWN"
        if self.phase != "REST":
            return "NOT_AT_REST"
        lo = max(PITCH_MIN, PITCH_MIN + self.aim_offset)
        hi = min(PITCH_MAX, PITCH_MAX + self.aim_offset)
        if not (lo - 1e-3 <= deg <= hi + 1e-3):
            return "OUT_OF_RANGE"
        self.aim_target = deg
        self.aimed = False
        self.token = None
        f = self.fail_next.pop("aim", None)
        if f:
            self.fail_next["_aim_now"] = f
        if not self.aim_known:
            self.op = {"homing": True, "left": 0.0, "total": 1.0}
        else:
            t = self.aim_travel_to(deg)
            if t < 0:
                return "OUT_OF_RANGE"
            self.op = {"homing": False, "left": t, "total": max(t, 1e-6)}
        self.state = "AIM"
        return None

    def _cmd_cock(self) -> Optional[str]:
        e = self._idle_check(True)
        if e:
            return e
        if self.state == "HOLD":
            return "HOLDING"
        if self.detent() != "LAUNCH":
            return "NOT_IN_LAUNCH"
        if self.phase == "COCKED":
            if not self.home_closed():
                self.evt("COCKED", ok=True, cock_deg=_n(self.cock_deg, 1))
                return None
            return "SWITCH_DISAGREE"
        if self.phase == "REST" and not self.home_closed():
            return "SWITCH_DISAGREE"
        if self.phase == "UNKNOWN" and not self.home_closed():
            return "LOADED_UNKNOWN"
        self.token = None
        if self.phase != "UNKNOWN":
            self.phase = "COCKING"
        f = self.fail_next.pop("cock", None)
        if f:
            self.fail_next["_cock_now"] = f
        self.op = {}
        self.state = "COCK"
        return None

    def _fire_check(self) -> Optional[str]:
        e = self._idle_check(True)
        if e:
            return e
        if self.state == "HOLD":
            return "HOLDING"
        if self.detent() != "LAUNCH":
            return "NOT_IN_LAUNCH"
        if not self.wraps_known or self.wraps() > 0:
            return "WRAPS_ON"
        cocked = self.phase == "COCKED" and not self.home_closed()
        at_rest = self.phase == "REST" and self.home_closed()     # fire from rest runs the whole cycle
        if not (cocked or at_rest):
            return "NOT_COCKED"
        p = self.pitch
        if (not self.aim_known or not self.aimed or self.aim_target is None or p is None
                or abs(p - self.aim_target) > self.settings["aim_tol"]):
            return "NOT_AIMED"
        if self.require_seated and not self.seated():
            return "NOT_SEATED"
        return None

    def _cmd_fire(self, tok: float) -> Optional[str]:
        if self.token is None:
            return "NOT_ARMED"
        if self.sim_t - self.token_t > TOKEN_S:
            self.token = None
            return "TOKEN_EXPIRED"
        if tok != float(self.token):
            self.token = None                # a wrong token burns the arm
            return "BAD_TOKEN"
        self.token = None
        e = self._fire_check()
        if e:
            return e
        self.phase = "FIRING"
        f = self.fail_next.pop("fire", None)
        if f:
            self.fail_next["_fire_now"] = f
        self.op = {}
        self.state = "FIRE"
        return None

    def _cmd_wrap(self) -> Optional[str]:
        e = self._idle_check()
        if e:
            return e
        if self.state == "HOLD":
            return "HOLDING"
        if self.drum_pos() == "WRAPPED":
            self.evt("WRAPPED", ok=True, wraps=self.wraps())
            return None
        if self.code_bad or self.drum_deg < DRUM_DEG["REEL"] - DRUM_WIN_DEG:
            return "NOT_IN_REEL"             # from REEL, or the wrap sectors (an interrupted wrap is repeated)
        self._start_drum(DRUM_DEG["WRAPPED"], "wrap", self.fail_next.pop("wrap", None))
        self.state = "WRAP"
        return None

    def _cmd_unwrap(self) -> Optional[str]:
        e = self._idle_check()
        if e:
            return e
        if self.state == "HOLD":
            return "HOLDING"
        if self.code_bad:
            return "POSITION_UNKNOWN"
        if self.detent() == "REEL":
            self.evt("UNWRAPPED", ok=True)
            return None
        if not self.in_wrap_sectors():
            return "NOT_WRAPPED"
        self._start_drum(DRUM_DEG["REEL"], "unwrap", self.fail_next.pop("unwrap", None))
        self.state = "UNWRAP"
        return None

    def _cmd_reel(self, duty: float, max_n: float, until: Optional[str]) -> Optional[str]:
        e = self._idle_check(True)
        if e:
            return e
        if self.drum_pos() != "WRAPPED":
            return "NOT_WRAPPED"
        u = "stop" if until is None else until
        if u not in ("stop", "seated", "tension"):
            return "BAD_ARG"
        if not (0 < duty <= 1):
            return "BAD_ARG"
        if not (max_n > 0):
            return "BAD_ARG"
        max_n = min(max_n, LIMIT_MAX_N)
        duty = max(duty, REEL_MIN_DUTY)
        f = self.fail_next.pop("reel", None)
        if f:
            self.fail_next["_reel_now"] = f
        self.op = {"until": u, "max_n": max_n, "duty": duty, "t0": self.sim_t, "t_start": self.sim_t}
        if u == "seated" and self.seated():
            self._seated_now()
            return None
        self.state = "REEL"
        return None

    def _cmd_jog(self, motor: str, d: float, ms: float) -> Optional[str]:
        if motor not in ("wiper", "drum"):
            return "BAD_ARG"
        if d not in (1, -1):
            return "BAD_ARG"
        if not ms > 0:
            return "BAD_ARG"
        ms = min(ms, JOG_MAX_MS)
        dd = int(d)
        op = self.op
        if self.state == "JOG" and op.get("motor") == motor and op.get("dir") == dd:
            op["t_end"] = self.sim_t + ms / 1000.0       # deadman refresh
            return None
        e = self._idle_check(motor == "wiper")
        if e:
            return e
        if self.state == "HOLD":
            return "HOLDING"
        if motor == "wiper":
            p = self.drum_pos()
            if p == "LAUNCH" and dd == COCK_DIR and self.phase in ("COCKED", "UNKNOWN", "FIRING"):
                return "WOULD_FIRE"
            if p == "LAUNCH" and dd == AIM_DIR and self.phase != "REST":
                return "NOT_AT_REST"
            if self.mode == "REEL" and dd != REEL_DIR:
                return "BACKSTOP"
            if p == "LAUNCH" and dd == AIM_DIR:
                self.aimed = False
        else:
            self.aimed = False
        self.token = None
        self.op = {"motor": motor, "dir": dd, "t_end": self.sim_t + ms / 1000.0}
        self.state = "JOG"
        return None

    def _cmd_cal(self, what: str, m: dict) -> Optional[str]:
        if what == "current":
            return "BUSY" if self.moving() else None
        if what == "tare":
            if not self.load_cell:
                return "NO_SENSOR"
            return None
        if what == "free":
            return None if self.state in ("REEL", "JOG") else "NOT_RUNNING"
        if what == "aim":
            deg = self._num(m, "deg")
            if deg is None:
                return "BAD_ARG"
            if not self.aim_known:
                return "AIM_UNKNOWN"
            self.aim_offset = deg - pitch_at(self.aim_phase)
            self.settings["aim_offset"] = self.aim_offset
            self.aimed = False
            return None
        if what == "aim_home":
            if self.moving():
                return "BUSY"
            self.aim_phase = 0.0             # (the mock's aim cycle has its home at phase 0)
            self.aim_known = True
            self.aimed = False
            return None
        if what == "aim_forget":
            if self.moving():
                return "BUSY"
            self.aim_known = False
            self.aimed = False
            return None
        if what == "cock_rest":
            if self.moving():
                return "BUSY"
            if not self.home_closed():
                return "SWITCH_DISAGREE"
            self.phase = "REST"
            self.cock_deg = 0.0
            return None
        if what == "decock":
            e = self._idle_check(True)
            if e:
                return e
            if self.detent() != "LAUNCH":
                return "NOT_IN_LAUNCH"
            if self.home_closed():
                return None
            self.aim_known = False
            self.aimed = False
            self.token = None
            self.op = {"home_at": None}
            self.state = "DECOCK"
            return None
        if what == "drum":                   # the drum code is absolute: this only reports
            if self.code_bad:
                return "POSITION_UNKNOWN"
            self._ev_mode(True)
            return None
        return "BAD_ARG"

    def _cmd_set(self, k: str, v: float) -> Optional[str]:
        if k in BOOL_KEYS:
            setattr(self, k, v != 0)
            return None
        if k == "enc_sign":
            if v not in (1, -1):
                return "BAD_ARG"
            if self.moving():
                return "BUSY"
            self.enc_sign = int(v)
            return None
        if k not in SET_KEYS:
            return "BAD_KEY"
        lo, hi = SET_KEYS[k]
        if not (lo <= v <= hi):
            return "OUT_OF_RANGE"
        if k == "line_m":
            self.line_est, self.line_known = v, True
        elif k == "aim_offset":
            self.aim_offset = v
            self.settings[k] = v
        else:
            self.settings[k] = v
        return None

    def _get_key(self, k: str, extra: dict) -> bool:
        if k == "fw":
            extra["val"] = FW
        elif k == "params":
            extra["val"] = PARAMS_VERSION
        elif k in BOOL_KEYS:
            extra["val"] = bool(getattr(self, k))
        elif k == "enc_sign":
            extra["val"] = self.enc_sign
        elif k == "pitch_range":
            extra.update(min=_n(PITCH_MIN, 2), max=_n(PITCH_MAX, 2))
        elif k == "line_m":
            extra["val"] = _n(self.line_est, 6)
        elif k in SET_KEYS:
            extra["val"] = _n(self.settings.get(k, 0.0), 6)
        else:
            return False
        return True


# ---------------------------------------------------------------- pty
class PtyEnd:
    """The master side of a pseudo terminal. The slave path (self.path) behaves like a serial port."""

    def __init__(self):
        import tty
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self._buf = b""

    def readline(self, timeout: float = 0.1) -> Optional[bytes]:
        import select
        t_end = time.monotonic() + timeout
        while True:
            i = self._buf.find(b"\n")
            if i >= 0:
                line, self._buf = self._buf[:i], self._buf[i + 1:]
                return line.rstrip(b"\r")
            left = t_end - time.monotonic()
            if left <= 0:
                return None
            r, _, _ = select.select([self.master], [], [], left)
            if r:
                try:
                    self._buf += os.read(self.master, 4096)
                except OSError:
                    return None

    def write(self, data: bytes) -> None:
        os.write(self.master, data)

    def close(self) -> None:
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass


def connected_pair(time_scale: float = 1.0, **kw):
    """(MockBase started, BaseClient started) joined by an in-process pipe."""
    from .base import BaseClient
    a, b = pipe_pair()
    mb = MockBase(a, time_scale=time_scale, **kw).start()
    c = BaseClient(b, reconnect_s=0.1).start()
    c.wait_connected(2.0)
    return mb, c


def serve_pty(time_scale: float = 1.0, **kw) -> MockBase:
    end = PtyEnd()
    mb = MockBase(end, time_scale=time_scale, **kw).start()
    mb.pty_path = end.path
    return mb


__all__ = ["MockBase", "World", "PtyEnd", "PipeEnd", "connected_pair", "serve_pty", "DRUM_DEG", "WRAP_DEG",
           "N_WRAPS", "STATES", "PHASES", "FAULTS", "SW_KEYS"]
