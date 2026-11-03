"""Simulated base station that speaks contract C1 like the real firmware (firmware/base/grappstar_base/base_logic.h,
fw "base-1.2") and follows the one C1 sequence of base/spec/BASE_SPEC.md section 10.1 (D35).

Run it in-process (connected to BaseClient through a pipe) or on a pseudo terminal so the real client (or a terminal
program) can open it like the USB port:

  python3 -m grappstar mockbase --pty        # prints /dev/pts/N; then: python3 -m grappstar base --port /dev/pts/N status

Commands, refusals (codes and check order), events, telemetry keys, states and phases follow base_logic.h (PROTOCOL.md
may lag it). jetson/tools/basesim runs the real firmware logic for comparison (tests/test_comms_base.py does). The
mechanics are simplified:

- Selector drum (N20 worm, absolute 3-bit drum code, params.DRUM): zones LAUNCH -4..8, L_TO_N 8..32, NEUTRAL 32..50,
  WRAP_PREP 50..62, WRAP 62..302 (the cup lays 7 wraps, collar NEUTRAL), N_TO_R 302..340, REEL 340..352; stop points
  LAUNCH 0, NEUTRAL 41, WRAPPED park 314 (in N_TO_R: wraps on, collar still NEUTRAL below 318), REEL 347; 8.1 deg/s.
  "wrap" (from LAUNCH or NEUTRAL) parks at WRAPPED and ends with WRAPPED only; "mode REEL" flies the collar in from
  the park; "unwrap" goes back to NEUTRAL (take-up winding meanwhile): UNWRAPPED then MODE. A fault or estop in REEL
  or N_TO_R parks the drum at WRAPPED, otherwise at NEUTRAL, unless the drum must hold (MODE ok false DRUM_HELD).
- Wiper in LAUNCH: BACKWARD aims (the firmware's default aim table, pitch 6..47 deg over a 3600 deg aim cycle, homed on
  SW_RAIL_LOW at 3365; a shift into LAUNCH forgets the aim), FORWARD cocks (415 deg), fires (release 432.7) and resets
  (553.8 = one big gear turn). Fire from REST runs the whole cycle.
- Wiper in REEL (collar engaged, wraps on): winds the capstan (about 0.22 m/s at duty 1). In NEUTRAL or at the WRAPPED
  park only the take-up turns (forward, duty 0.27..0.4, about 0.3 m/s per unit duty, slips at about 5 N): "tidy" (by
  time, TIDIED), "retrieve" (until SW_SEATED, SEATED; FAULT DART_STUCK after max_s 5..600, default as approachLimit()), "reel"
  until stop / slip / seated. The line tension comes from the attached World (dart seated, flying, free, anchored).
  Telemetry tension_n is the load cell (null when load_cell is False).
- Faults: inject_fault(code) or arm one with fail_next[cmd] = code (raised part way through that command's motion).

Where BASE_SPEC 10.1 is stricter than base-1.2 the mock follows 10.1 (as the newer firmware base-1.2.1 does):
"retrieve" and "reel until seated" are refused in REEL (WRAPS_ON, never seat on the capstan) and "retrieve" takes
max_s; "cock" needs SW_SEATED (NOT_SEATED); "decock" with the carriage home and the phase UNKNOWN answers RESET ok
false; status carries ev_drops; set accepts tow_trip_a. seat_ends_any_reel (default False = firmware) makes the seat
contact end every reel with SEATED.

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

FW = "base-1.2"
PARAMS_VERSION = "1.2.1"

# ---- drum (params.DRUM, base_logic.h Cfg): zones in travel order (name, lo, hi, closed code contacts)
ZONES = (("LAUNCH", -4.0, 8.0, "a"), ("L_TO_N", 8.0, 32.0, "ab"), ("NEUTRAL", 32.0, 50.0, "b"),
         ("WRAP_PREP", 50.0, 62.0, "bc"), ("WRAP", 62.0, 302.0, "c"), ("N_TO_R", 302.0, 340.0, "ac"),
         ("REEL", 340.0, 352.0, "abc"))
ZONE_NAMES = tuple(z[0] for z in ZONES)
ZI = {n: i for i, n in enumerate(ZONE_NAMES)}
DRUM_DEG = {"LAUNCH": 0.0, "NEUTRAL": 41.0, "REEL": 347.0, "WRAPPED": 314.0}
TARGET_ZONE = {"LAUNCH": "LAUNCH", "NEUTRAL": "NEUTRAL", "REEL": "REEL", "WRAPPED": "N_TO_R"}
DRUM_ORDER = ("LAUNCH", "NEUTRAL", "WRAPPED", "REEL")   # physical order of the stop points along the drum travel
DETENTS = ("LAUNCH", "NEUTRAL", "REEL")                 # the stop points "mode" goes to
DRUM_POS_OF = {"LAUNCH": "LAUNCH", "NEUTRAL": "NEUTRAL", "REEL": "REEL", "N_TO_R": "WRAPPED"}
N_WRAPS = MAX_WRAPS = 7
WRAP_FROM, WRAP_TO = 62.0, 302.0
WRAP_DEG = (WRAP_TO - WRAP_FROM) / N_WRAPS             # drum travel per capstan wrap
COLLAR_N_DEG = 318.0      # in N_TO_R the collar stays NEUTRAL below this
LN_ENGAGE, NR_ENGAGE = 12.0, 336.0                     # SW_DOG_L closed below, SW_DOG_R closed above
DRUM_DPS = 8.1
PARK_TOL = 4.0            # atPark()
# ---- wiper (45 rpm free = 270 deg/s at full duty)
N0_DPS = 270.0
AIM_DIR, COCK_DIR, REEL_DIR = -1, +1, +1
# ---- aim train (base_logic.h loadAimTable default: pitch every 100 aim deg over one 3600 deg cycle)
AIM_TABLE = (6.01, 6.24, 6.93, 8.12, 9.83, 12.10, 14.93, 18.32, 22.22, 26.50, 30.98, 35.39, 39.42, 42.76, 45.19,
             46.62, 47.06, 46.59, 45.37, 43.52, 41.20, 38.54, 35.64, 32.59, 29.49, 26.39, 23.36, 20.46, 17.72, 15.19,
             12.91, 10.91, 9.21, 7.85, 6.85, 6.23, 6.01)
AIM_CYCLE_DEG = 3600.0
PITCH_MIN, PITCH_MAX = 6.0, 47.0
AIM_TOL_DEG = 0.75
AIM_HOME_DEG = 3365.0
AIM_DPS, AIM_SLOW_DPS, AIM_SLOW_DEG = N0_DPS * 200 / 255, N0_DPS * 110 / 255, 60.0
# ---- shot cycle (wiper deg forward from rest, one big gear turn per shot)
CYCLE_DEG = 553.8
HOME_OPEN_DEG, HOME_WIN_DEG = 4.0, 12.0
COCK_END_DEG, COCK_SLOW_FROM = 415.0, 380.0
RELEASE_DEG, RELEASE_WIN_DEG = 432.7, 15.0
GAP_CLOSE_DEG = 520.0
DECOCK_EXTRA_DEG = 15.0
COCK_DPS, COCK_SLOW_DPS = N0_DPS, N0_DPS * 140 / 255
FIRE_DPS, RESET_DPS = N0_DPS, N0_DPS * 200 / 255
DECOCK_DPS, JOG_DPS = N0_DPS * 120 / 255, N0_DPS * 150 / 255
TOKEN_S = 3.0
JOG_MAX_MS = 1500
# ---- reel (capstan 41 mm line centre x 50/22) and take-up
LINE_PER_WIPER_DEG = math.pi * 0.041 * (50.0 / 22.0) / 360.0
REEL_MPS = N0_DPS * LINE_PER_WIPER_DEG                 # line speed at duty 1, light load (about 0.22 m/s)
TAKEUP_MPS = 0.3          # take-up line speed per unit duty (0.09 m/s at duty 0.3)
TAKEUP_SLIP_N = 5.0       # FIRM take-up clutch
LINE_K = 400.0            # N per metre once the line is taut
LIMIT_N, LIMIT_MAX_N = 50.0, 110.0
RESUME_FRAC = 0.6
TENSION_IGNORE_S = 0.35
TAPER_FRAC = 0.30
REEL_DUTY = 220.0 / 255.0
REEL_MIN_DUTY = 70.0 / 255.0
TAKEUP_MAX_DUTY = 102.0 / 255.0
TIDY_MS = 3000
TIDY_STEADY_S, TIDY_MAX_S = 2.0, 15.0
APPROACH_S = 30.0         # retrieve with line_m unknown: DART_STUCK after this (config.h approachMaxMs)
TAKEUP_MPS_PER_DUTY, TAKEUP_MPS_OFF = 0.29, 0.03   # config.h: slowest take-up speed = 0.29 x duty - 0.03 m/s


def approach_s(line_m: Optional[float], duty: float) -> float:
    """base-1.2.1 approachLimit(): 10 + 1.5 x line_m / v, at least 20 s, at most 600 s; 30 s with line_m unknown."""
    if line_m is None:
        return APPROACH_S
    d = min(max(duty, REEL_MIN_DUTY), TAKEUP_MAX_DUTY)
    v = max(0.02, TAKEUP_MPS_PER_DUTY * d - TAKEUP_MPS_OFF)
    return min(600.0, max(20.0, 10.0 + 1.5 * line_m / v))
REEL_MAX_S = 240.0
TRIP_A, COCK_TRIP_A, TOW_TRIP_A, TAKEUP_TRIP_A, HARD_TRIP_A = 5.0, 6.5, 6.5, 2.5, 17.0
# ---- power, protocol
VBAT_LOW, VBAT_PRESENT = 9.9, 6.0
TEL_SLOW_S, TEL_FAST_S = 0.10, 0.05

STATES = ("IDLE", "MODE", "AIM", "COCK", "FIRE", "WRAP", "UNWRAP", "REEL", "HOLD", "JOG", "DECOCK", "SAFING",
          "FAULT")
PHASES = ("REST", "COCKING", "COCKED", "FIRING", "RESETTING", "UNKNOWN")
FAULTS = ("STALL", "OVERCURRENT", "DRUM_TIMEOUT", "DRUM_SENSOR", "ANGLE_SENSOR", "SWITCH_DISAGREE", "BROWNOUT",
          "BAD_STATE", "DART_STUCK")
# telemetry "sw" keys in the firmware's order (config.h SW_TEL): closed contacts only, value 1, absent = open
SW_KEYS = {"a": "SW_DRUM_A", "b": "SW_DRUM_B", "c": "SW_DRUM_C", "l": "SW_DOG_L", "r": "SW_DOG_R", "n": "SW_N20",
           "H": "SW_HOME", "G": "SW_GAP", "L": "SW_RAIL_LOW", "S": "SW_SEATED", "W": "SW_ARM", "I": "SW_INDEX"}

# settable keys (base_logic.h keys(), plus tow_trip_a of base-1.2.1): name -> (lo, hi)
SET_KEYS = {"limit_n": (1, LIMIT_MAX_N), "k_n_per_a": (0.1, 200), "tension_offset_n": (-50, 50), "free_a": (0, 10),
            "amps_per_volt": (1, 200), "offset_a": (0, 5), "hx_n_per_count": (1e-7, 1), "drum_dps": (5, 720),
            "aim_tol": (0.1, 5), "aim_offset": (-20, 20), "line_m": (0, 100), "trip_a": (3, 19),
            "cock_trip_a": (3, 19), "tow_trip_a": (3, 19), "vbat_low": (6, 12), "aim_home_deg": (-7200, 7200),
            "host_timeout_s": (0, 600)}
BOOL_KEYS = ("require_seated", "vbat_wired")
DRUM_FAULTS = ("DRUM_TIMEOUT", "DRUM_SENSOR")
HOLD_LAUNCH = "launch train not at rest: the wiper worm holds the spring (a shift under load could dry fire)"


def pitch_at(phase: float, offset: float = 0.0) -> float:
    """The firmware's default aim table (periodic, linear interpolation) at an aim phase."""
    x = phase % AIM_CYCLE_DEG
    step = AIM_CYCLE_DEG / (len(AIM_TABLE) - 1)
    i = min(int(x // step), len(AIM_TABLE) - 2)
    f = (x - i * step) / step
    return AIM_TABLE[i] + (AIM_TABLE[i + 1] - AIM_TABLE[i]) * f + offset


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
                 seat_ends_any_reel: bool = False, drum_at: str = "NEUTRAL"):
        """fresh=True: a new build with no NVS record (phase UNKNOWN, aim unknown: run cal cock_rest first).
        The default is a calibrated base that booted at rest, aim known, drum at drum_at (a stop point name)."""
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
        self.drum_deg = DRUM_DEG.get(drum_at, DRUM_DEG["NEUTRAL"])
        self.drum_running = False
        self.aim_phase = 1100.0             # truth: aim train phase (deg of wiper travel in the aim direction)
        self.aim_known = not fresh
        self.aim_offset = 0.0
        self.aimed = False
        self.aim_target: Optional[float] = None
        self.cock_deg = 0.0                 # truth: launch train angle inside the shot cycle
        self.phase = "UNKNOWN" if fresh else "REST"
        self.phase_hint = -1.0              # phaseHint: last angle when the phase became UNKNOWN
        self.token: Optional[int] = None
        self.token_t = 0.0
        self.token_expired = False
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
        self.settings: Dict[str, float] = {"limit_n": LIMIT_N, "k_n_per_a": 17.0, "tension_offset_n": -10.4,
                                           "free_a": 1.5, "amps_per_volt": 12.5, "offset_a": 0.0,
                                           "hx_n_per_count": 1 / 21000.0, "drum_dps": DRUM_DPS,
                                           "aim_tol": AIM_TOL_DEG, "aim_offset": 0.0, "aim_home_deg": AIM_HOME_DEG,
                                           "trip_a": TRIP_A, "cock_trip_a": COCK_TRIP_A, "tow_trip_a": TOW_TRIP_A,
                                           "vbat_low": VBAT_LOW, "host_timeout_s": 0.0}
        self.code_bad = False               # test hook: drum code invalid (drum_pos UNKNOWN, POSITION_UNKNOWN)
        self.ev_drops = 0
        self.last_rx_t = 0.0                # sim time of the last line from the host (host watchdog)
        self.commands = self._commands()
        self._defer: Optional[list] = None  # events raised while a command is handled, sent after its reply
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

    # ------------------------------------------------ views (modeName, drumPos, wrapsAt, ...)
    def zone(self) -> Optional[str]:
        """The drum code sector, None when the code is invalid."""
        if self.code_bad:
            return None
        d = self.drum_deg
        for name, lo, hi, _ in ZONES:
            if d < hi:
                return name
        return "REEL"

    def zi(self) -> int:
        z = self.zone()
        return -1 if z is None else ZI[z]

    def dog_l(self) -> bool:
        return not self.code_bad and self.drum_deg < LN_ENGAGE

    def dog_r(self) -> bool:
        return not self.code_bad and self.drum_deg > NR_ENGAGE

    @property
    def mode(self) -> str:
        """modeName(): what the collar really does."""
        z = self.zone()
        if z == "LAUNCH" and not self.dog_r():
            return "LAUNCH"
        if z == "REEL" and not self.dog_l():
            return "REEL"
        if (z in ("NEUTRAL", "WRAP_PREP", "WRAP") or (z == "N_TO_R" and self.drum_deg < COLLAR_N_DEG)) \
                and not self.dog_l() and not self.dog_r():
            return "NEUTRAL"
        return "NONE"

    @mode.setter
    def mode(self, to: str) -> None:       # tests that place the drum by hand
        if to in DRUM_DEG:
            self.drum_deg = DRUM_DEG[to]

    @property
    def drum_known(self) -> bool:
        return not self.code_bad             # the drum code is absolute

    @property
    def wraps_known(self) -> bool:
        return not self.code_bad

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

    def detent(self) -> Optional[str]:
        """detentClosed(): LAUNCH / NEUTRAL / REEL when the drum is in that sector, else None."""
        z = self.zone()
        return z if z in DETENTS else None

    def wraps(self) -> int:
        z = self.zone()
        if z in ("N_TO_R", "REEL"):
            return N_WRAPS
        if z != "WRAP":
            return 0
        f = (self.drum_deg - WRAP_FROM) / (WRAP_TO - WRAP_FROM) * N_WRAPS
        return max(0, min(N_WRAPS, int(math.floor(f + 0.15))))

    def in_wrap_sectors(self) -> bool:
        return self.zi() >= ZI["WRAP"]

    def drum_pos(self) -> str:
        z = self.zone()
        if z is None:
            return "UNKNOWN"
        return DRUM_POS_OF.get(z, "BETWEEN")

    def seated(self) -> bool:
        return self.world.dart == "seated" and self.line_m <= 0.01

    def home_closed(self) -> bool:
        """SW_HOME: carriage fully forward (before the first tooth, or after the release until the cycle ends)."""
        return self.cock_deg < HOME_OPEN_DEG or self.cock_deg >= RELEASE_DEG

    def gap_closed(self) -> bool:
        """SW_GAP: big gear in its toothless gap (rest, or the end of the reset)."""
        return self.cock_deg < HOME_OPEN_DEG or self.cock_deg >= GAP_CLOSE_DEG

    def takeup_zone(self) -> bool:
        """cmdReel 'neutral': collar NEUTRAL in the NEUTRAL sector or at the WRAPPED park (below 318)."""
        z = self.zone()
        return (z == "NEUTRAL" or (z == "N_TO_R" and self.drum_deg < COLLAR_N_DEG)) and not self.dog_l() \
            and not self.dog_r()

    def launch_centred(self) -> bool:
        return self.mode == "LAUNCH" and not self.drum_running and abs(self.drum_deg - DRUM_DEG["LAUNCH"]) <= 6

    def loaded_known(self) -> bool:
        return self.phase in ("COCKING", "COCKED", "FIRING")

    def maybe_loaded(self) -> bool:
        return self.phase != "REST" or self.cock_deg > HOME_OPEN_DEG + 0.5 or not self.home_closed()

    def launch_exit_check(self) -> Optional[str]:
        if self.maybe_loaded() or not self.gap_closed():
            return "LAUNCH_BUSY"
        return None

    def sw(self) -> dict:
        out = {}
        z = self.zone()
        if z is not None:
            for key in ZONES[ZI[z]][3]:
                out[key] = 1
        if self.dog_l():
            out["l"] = 1
        if self.dog_r():
            out["r"] = 1
        out["n"] = 1                          # N20 in mesh (no hand mode in the mock)
        if self.home_closed():
            out["H"] = 1
        if self.gap_closed():
            out["G"] = 1
        if 0 <= (self.aim_phase - self.settings["aim_home_deg"]) % AIM_CYCLE_DEG < 8:
            out["L"] = 1                     # rail at its low dead point (aim home)
        if self.seated():
            out["S"] = 1
        wf = ((self.drum_deg - WRAP_FROM) / WRAP_DEG) % 1.0
        if z == "WRAP" and 0.014 <= wf <= 0.986:
            out["W"] = 1                     # cad-reel: SW_ARM closed while the cup is away, opens about +-5 deg at home
        return {k2: out[k2] for k2 in SW_KEYS if k2 in out}

    def moving(self) -> bool:
        return self.drum_running or self.state not in ("IDLE", "HOLD", "FAULT")

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
                # the reply to a command is written before any event that command caused
                self._defer = []
                try:
                    reply = self.handle_line(line)
                except Exception as e:  # never crash the mock
                    reply = _dump({"id": None, "ok": False, "err": "BAD_ARG", "msg": str(e)[:60]})
                deferred, self._defer = self._defer, None
                if reply is not None:
                    self._send_line(reply)
                for ev in deferred:
                    self._send_line(ev)

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

    def now_ms(self) -> int:
        return int(self.sim_t * 1000)

    # ------------------------------------------------ telemetry and events
    def telemetry(self, lean: bool = False) -> dict:
        """telBuild(): the full form, or the lean form (fewer decimals, null fields left out, lean true)."""
        d1, d2 = (0, 1) if lean else (1, 2)
        tel: Dict[str, Any] = {
            "state": self.state, "mode": self.mode,
            "drum_deg": _n(self.drum_deg, d1) if self.drum_known else None, "drum_pos": self.drum_pos(),
            "wiper_deg": _n(self.wiper_deg, d1), "rpm": _n(self.rpm, d1), "amps": _n(self.amps, d1),
            "pitch_deg": _n(self.pitch, d2), "cock": _n(self.cock, 2),
            "wraps": self.wraps() if self.wraps_known else None,
            "line_m": _n(self.line_est, d2) if self.line_known else None,
            "tension_n": _n(self.tension, d1) if self.load_cell else None,
            "sw": self.sw(), "fault": self.fault, "vbat": _n(self.vbat, 1) if self.vbat_wired else None}
        if lean:
            tel = {k: v for k, v in tel.items() if v is not None}
            tel["lean"] = True
        return tel

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
        s = s if len(s) <= MAX_LINE else '{"evt":"OVERFLOW"}'
        if self._defer is not None:
            self._defer.append(s)
        else:
            self._send_line(s)

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

    # ------------------------------------------------ faults (raise, safeToNeutral, safeTarget, atPark)
    def _hold_reason(self, code: Optional[str]) -> Optional[str]:
        """safeToNeutral(): None if the drum may go to its safe park now, else why it must hold."""
        if code in DRUM_FAULTS:
            return "drum fault"
        if self.code_bad:
            return "drum code invalid"
        if self.zi() >= ZI["WRAP"]:          # base-1.2.1 safeToNeutral: wraps on -> the drum holds (D27)
            return "wraps on: the drum holds, the operator unwraps when safe"
        if self.loaded_known() or (self.zi() <= ZI["L_TO_N"] and self.maybe_loaded()):
            return HOLD_LAUNCH
        return None

    def _safe_target(self) -> str:
        """REEL or N_TO_R: the WRAPPED park (never back through the wrap window), otherwise NEUTRAL."""
        return "WRAPPED" if self.zone() in ("REEL", "N_TO_R") else "NEUTRAL"

    def _at_park(self, t: str) -> bool:
        return self.zone() == TARGET_ZONE[t] and abs(self.drum_deg - DRUM_DEG[t]) < PARK_TOL and not self.drum_running

    def _phase_lost(self) -> None:
        if self.phase != "UNKNOWN" or self.cock_deg > self.phase_hint:
            self.phase_hint = self.cock_deg
        self.phase = "UNKNOWN"

    def raise_fault(self, code: str, drum_to_neutral: bool = True, why: Optional[str] = None) -> None:
        self.rpm = 0.0
        self.amps = 0.0
        why = why or {"STALL": "wiper stalled", "OVERCURRENT": "wiper over %.1f A" % self.settings["trip_a"],
                      "DRUM_TIMEOUT": "drum did not reach the target in time", "DRUM_SENSOR":
                      "drum code jumped over a sector", "ANGLE_SENSOR": "AS5600 lost while the wiper runs",
                      "SWITCH_DISAGREE": "SW_DOG_L and SW_DOG_R both closed", "BROWNOUT":
                      "battery sagged under load", "BAD_STATE": "bad state", "DART_STUCK":
                      "final approach: no SW_SEATED in time"}.get(code, code)
        if self.state == "SAFING":           # a second fault stops the safing move
            self.op = {}
            self.drum_running = False
            self.state = "FAULT"
            self._ev_mode(False, "DRUM_HELD", why[:47])
            return
        if self.state == "FAULT":
            return                           # the first fault wins
        self.fault, self.fault_why = code, why[:47]
        if self.phase in ("COCKING", "FIRING", "RESETTING") or self.state in ("COCK", "FIRE", "DECOCK"):
            self._phase_lost()               # a fault during launch-train motion: the phase is no longer trusted
        self.token = None
        self.evt("FAULT", code=code, why=self.fault_why)
        if code == "STALL":
            self.evt("STALL")
        hold = self._hold_reason(code) if drum_to_neutral else "drum fault"
        st = self._safe_target()
        if not hold and not self._at_park(st):
            self._start_drum(st, "safing", self.fail_next.pop("safing", None))
            self.state = "SAFING"
            return
        self.op = {}
        self.drum_running = False
        self.state = "FAULT"
        if hold and not self._at_park(st):
            self._ev_mode(False, "DRUM_HELD", hold)
        elif self._at_park(st):
            self._ev_mode(True)

    def inject_fault(self, code: str, present: bool = False, why: Optional[str] = None) -> None:
        with self.lock:
            self.fault_present = present
            self.raise_fault(code, why=why)

    # ------------------------------------------------ simulation
    def _start_drum(self, target: str, kind: str, fail: Optional[str] = None) -> None:
        """startDrumMove(): centering jog when leaving an engaged collar (not when safing), then the run."""
        z = self.zone()
        jog = z in ("LAUNCH", "REEL") and z != TARGET_ZONE[target] and kind != "safing"
        self.op = {"kind": kind, "target": target, "to": DRUM_DEG[target], "from": self.drum_deg, "fail": fail,
                   "jog_end": self.sim_t + (0.3 if jog else 0.0), "jog": z if jog else None,
                   "takeup": DRUM_DEG[target] < self.drum_deg and self.zi() >= ZI["WRAP_PREP"]}

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
                     wraps_known=self.wraps_known, aim_known=self.aim_known, phase=self.phase, angle="AS5600")
        self._world_step(dt)
        if self.token is not None and self.sim_t - self.token_t > TOKEN_S:
            self.token, self.token_expired = None, True
        self.rpm = 0.0
        self.amps = 0.0
        st = self.state
        ht = self.settings["host_timeout_s"]
        if ht > 0 and self.moving() and st not in ("FAULT", "SAFING") and self.sim_t - self.last_rx_t > ht:
            self._cmd_stop(False)            # host watchdog: the stopped state with why stop, then HOST
            self.evt("ABORTED", what="HOST", why="host silent")
            self.last_rx_t = self.sim_t
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
        if self.state in ("IDLE", "FAULT") and self.world.dart not in ("anchored", "flying"):
            self.tension = 0.0 if self.line_m <= 0 or self.world.dart == "seated" else min(self.tension, 0.5)
        self.peak_a = max(self.peak_a, self.amps)
        self.vbat = self.vbat0 - 0.05 * self.amps
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
        """Turn the wiper for one step; returns its speed (deg/s)."""
        self.rpm = direction * dps / 6.0
        self.amps = amps
        return dps

    def _drum_step(self, dt: float) -> None:
        op = self.op
        if "target" not in op:
            self.state = "FAULT" if self.state == "SAFING" else "IDLE"
            return
        if self.sim_t < op["jog_end"]:       # centering jog (LAUNCH: backward; REEL: forward, then back)
            d = AIM_DIR if op["jog"] == "LAUNCH" else (REEL_DIR if self.sim_t < op["jog_end"] - 0.15 else -REEL_DIR)
            self.wiper_deg += d * self._wiper(d, JOG_DPS, 1.5) * dt
            return
        target = op["to"]
        d = target - self.drum_deg
        stepd = self.settings["drum_dps"] * dt
        fail = op.get("fail")
        if fail:
            span = abs(target - op["from"]) or 1.0
            if abs(self.drum_deg - op["from"]) >= 0.4 * span:
                op["fail"] = None
                self.drum_running = False
                if op["kind"] == "safing":   # the fault is already latched: report the failed move
                    self.state = "FAULT"
                    self.op = {}
                    self._ev_mode(False, fail, "drum did not reach the target in time")
                else:                        # mode, wrap, unwrap, a failed estop move: a drum fault
                    self.raise_fault(fail)
                return
        if op["takeup"] and self.zone() in ("WRAP", "WRAP_PREP"):
            self._takeup(TAKEUP_MAX_DUTY, dt)   # unwrapping: the take-up winds the freed line
        elif op["target"] == "REEL" and self.drum_deg >= NR_ENGAGE - 18:
            self.wiper_deg += self._wiper(REEL_DIR, N0_DPS * 51 / 255, 1.8) * dt   # fly-in
        if abs(d) > stepd:
            self.drum_running = True
            self.drum_deg += stepd if d > 0 else -stepd
            return
        self.drum_deg = target
        self.drum_running = False
        kind = op["kind"]
        self.op = {}
        if op["target"] == "LAUNCH":
            self.aim_known = self.aimed = False   # the dog window is unknown: the next aim re-homes
        if kind == "safing":
            self.state = "FAULT"
            self._ev_mode(True)
        elif kind == "wrap":
            self.state = "IDLE"
            self.evt("WRAPPED", ok=True, wraps=self.wraps())
        elif kind == "unwrap":
            self.state = "IDLE"
            self.evt("UNWRAPPED", ok=True)
            self._ev_mode(True)
        else:                                # mode, estop
            self.state = "IDLE"
            self._ev_mode(True)

    def aim_travel_to(self, target: float) -> float:
        """Smallest forward aim travel that brings the pitch to target (-1 if never), exact to 1e-6 deg."""
        f = lambda t: pitch_at(self.aim_phase + t, self.aim_offset) - target
        prev = f(0.0)
        if abs(prev) <= self.settings["aim_tol"] * 0.25:
            return 0.0
        t, step = 0.0, 1.0
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
            trav = self._wiper(AIM_DIR, AIM_DPS, 2.0) * dt
            home = self.settings["aim_home_deg"]
            before = (self.aim_phase - home) % AIM_CYCLE_DEG
            self.aim_phase = (self.aim_phase + trav) % AIM_CYCLE_DEG
            self.wiper_deg += AIM_DIR * trav
            after = (self.aim_phase - home) % AIM_CYCLE_DEG
            if after < before:               # SW_RAIL_LOW rose
                self.aim_known = True
                op["homing"] = False
                op["left"] = op["total"] = self.aim_travel_to(self.aim_target)
                if op["left"] < 0:
                    self.state = "IDLE"
                    self.op = {}
                    self._ev_aimed(False, "OUT_OF_RANGE")
                elif op["left"] <= 1e-9:
                    op["settle"] = self.sim_t
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
        if self.phase == "UNKNOWN":          # recovery creep until SW_HOME opens (the first tooth = reference)
            trav = self._wiper(COCK_DIR, COCK_SLOW_DPS, 2.0) * dt
            self.cock_deg += trav
            if self.cock_deg >= CYCLE_DEG:
                self.cock_deg -= CYCLE_DEG
            self.wiper_deg += trav
            if not self.home_closed():
                self.phase = "COCKING"
                self.phase_hint = -1.0
            return
        dps = COCK_SLOW_DPS if self.cock_deg >= COCK_SLOW_FROM else COCK_DPS
        load = 0.5 if self.cock_deg < HOME_OPEN_DEG else 2.0 + 3.0 * (self.cock_deg - HOME_OPEN_DEG) / (
            RELEASE_DEG - HOME_OPEN_DEG)
        trav = min(COCK_END_DEG - self.cock_deg, self._wiper(COCK_DIR, dps, load) * dt)
        self.cock_deg += trav
        self.wiper_deg += trav
        if self.cock_deg >= 250.0:
            f = self.fail_next.pop("_cock_now", None)
            if f:
                self.raise_fault(f)
                return
        if self.cock_deg >= COCK_END_DEG - 1e-6:
            self.cock_deg = COCK_END_DEG
            self.phase = "COCKED"
            self.phase_hint = -1.0
            self.state = "IDLE"
            self.evt("COCKED", ok=True, cock_deg=_n(self.cock_deg, 1))

    def _fire_step(self, dt: float) -> None:
        if self.phase == "FIRING":
            f = self.fail_next.pop("_fire_now", None)
            if f:
                self.raise_fault(f)
                return
            trav = self._wiper(COCK_DIR, FIRE_DPS, 5.0) * dt   # from COCKED, or the whole cycle from REST
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
        f = self.fail_next.pop("_decock_now", None)
        if f:
            self.raise_fault(f)
            return
        if op.get("home_at") is None:
            self.cock_deg = max(HOME_OPEN_DEG - 1.0, self.cock_deg - trav)
            if self.home_closed():
                op["home_at"] = self.wiper_deg
            return
        if abs(self.wiper_deg - op["home_at"]) >= DECOCK_EXTRA_DEG:
            self.phase = "REST"
            self.cock_deg = HOME_OPEN_DEG - 2.0
            self.phase_hint = -1.0
            self.state = "IDLE"
            self.op = {}
            self.evt("RESET", ok=True, how="decock")

    def _reel_tension(self, speed: float, dt: float) -> float:
        """The capstan winds at speed (m/s at no load) for one step. Returns the line travel."""
        w = self.world
        if w.dart == "anchored" or (w.dart == "seated" and self.line_m <= 0.01):
            if w.dart == "anchored" and w.slack_m > 0:
                take = min(w.slack_m, speed * dt)
                w.slack_m -= take
                self.line_m = max(0.0, self.line_m - take)
                self.tension = 1.0
                return take
            # taut: tension rises with the wiper torque, speed falls (worm drive)
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

    def _takeup(self, duty: float, dt: float) -> None:
        """The take-up alone (capstan free) winds for one step: pulls slack or a free dart, slips at about 5 N."""
        w = self.world
        speed = TAKEUP_MPS * duty
        self.wiper_deg += self._wiper(REEL_DIR, N0_DPS * duty, self.settings["free_a"] * 0.8) * dt
        if w.dart == "anchored":
            take = min(w.slack_m, speed * dt)
            w.slack_m -= take
            self.line_m = max(0.0, self.line_m - take)
            self.tension = 1.0 if w.slack_m > 0 else TAKEUP_SLIP_N
        elif self.seated():
            self.tension = 0.0
        elif w.drag_free_n >= TAKEUP_SLIP_N:
            self.tension = TAKEUP_SLIP_N     # too heavy for the take-up: it slips
        else:
            take = min(self.line_m, speed * dt)
            self.line_m = max(0.0, self.line_m - take)
            self.tension = w.drag_free_n if self.line_m > 0 else 0.0
            if self.line_m <= 0 and w.dart in ("free", "flying"):
                w.dart = "seated"
        self.amps += 0.04 * self.tension

    def _enter_limit(self, why: str) -> None:
        op = self.op
        holding = op.get("until") == "tension"
        self.state = "HOLD" if holding else "IDLE"
        self.rpm = 0.0
        self.amps = 0.0
        op["hold_t"] = self.sim_t
        self.evt("LIMIT", tension_n=_n(self.tension, 1), max_n=_n(op.get("max_n", LIMIT_N), 1), why=why,
                 holding=holding)
        if not holding:
            self.op = {}

    def _end_reel(self, name: str, **kw) -> None:
        self.state = "IDLE"
        self.op = {}
        self.evt(name, **kw)

    def _reel_step(self, dt: float) -> None:
        op = self.op
        if "until" not in op:
            self.state = "IDLE"
            return
        u = op["until"]
        seat_rose = self.seated() and not self._seat_prev
        if seat_rose and (u == "seated" or self.seat_ends_any_reel):
            self._seated_now()
            return
        run_s = self.sim_t - op["t0"]
        seat0 = self.seated()
        if run_s > 0.3:
            f = self.fail_next.pop("_reel_now", None)
            if f:
                self.raise_fault(f)
                return
        if op["takeup"]:                     # NEUTRAL or the WRAPPED park: take-up only
            if u == "seated" and run_s > op["max_s"]:
                self.raise_fault("DART_STUCK", why="final approach: no SW_SEATED in %g s" % round(op["max_s"]))
                return
            if u == "time" and self.sim_t >= op["t_end"]:
                self._end_reel("TIDIED", ok=True)
                return
            self._takeup(op["duty"], dt)
            if u == "slip":                  # the take-up current is steady once the line is straight
                if self.world.dart == "anchored" and self.world.slack_m > 0 or run_s < 0.5:
                    op["slip_t"] = self.sim_t
                if self.sim_t - op.get("slip_t", op["t0"]) >= TIDY_STEADY_S:
                    self._end_reel("LIMIT", tension_n=None, max_n=_n(op["max_n"], 1),
                                   why="tidy done (take-up slipping)", holding=False)
                    return
                if run_s > TIDY_MAX_S:
                    self._end_reel("ABORTED", what="REEL", why="timeout")
                    return
        else:                                # REEL: the capstan winds
            max_n = op["max_n"]
            frac = self.tension / max_n if max_n > 0 else 0.0
            if frac >= 1.0 and run_s > TENSION_IGNORE_S:
                self._enter_limit("tension limit")
                return
            duty = op["duty"]
            if frac > 1 - TAPER_FRAC:
                duty = max(REEL_MIN_DUTY, duty * (1 - frac) / TAPER_FRAC)
            speed = REEL_MPS * duty * max(0.1, 1.0 - self.tension / 300.0)
            take = self._reel_tension(speed, dt)
            if self.line_known:
                self.line_est = max(0.0, self.line_est - take)
            dps = speed / LINE_PER_WIPER_DEG
            s = self.settings
            self._wiper(REEL_DIR, dps, s["free_a"] + max(0.0, self.tension - s["tension_offset_n"]) / s["k_n_per_a"])
            self.wiper_deg += dps * dt
            if self.amps > op["trip"]:
                self.raise_fault("OVERCURRENT", why="wiper over %.1f A" % op["trip"])
                return
        if self.seated() and not seat0 and (u == "seated" or self.seat_ends_any_reel):
            self._seated_now()               # SW_SEATED rose during this step
            return
        if run_s > REEL_MAX_S:
            self._end_reel("ABORTED", what="REEL", why="timeout")

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
                and self.tension < RESUME_FRAC * op["max_n"] and self.sim_t - op.get("hold_t", 0.0) > 1.0):
            op["t0"] = self.sim_t                # the load cell shows the line slackened: wind again
            self.state = "REEL"

    def _jog_step(self, dt: float) -> None:
        op = self.op
        if "motor" not in op:
            self.state = "IDLE"
            return
        if op["motor"] == "wiper":
            p = self.drum_pos()
            if op.get("takeup"):
                self._takeup(TAKEUP_MAX_DUTY, dt)
            else:
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
                        self._end_reel("ABORTED", what="JOG", why="would pass the release tooth")
                        return
                elif self.mode == "REEL":
                    take = self._reel_tension(trav * LINE_PER_WIPER_DEG / dt if dt > 0 else 0.0, dt)
                    if self.line_known:
                        self.line_est = max(0.0, self.line_est - take)
        else:
            self.drum_running = True
            self.drum_deg += op["dir"] * self.settings["drum_dps"] * dt
            if ((op["dir"] < 0 and self.zone() == "LAUNCH" and self.drum_deg <= DRUM_DEG["LAUNCH"] - 4)
                    or (op["dir"] > 0 and self.zone() == "REEL" and self.drum_deg >= DRUM_DEG["REEL"] + 4)):
                self.drum_running = False
                self._end_reel("ABORTED", what="JOG", why="drum end stop")
                return
        if self.sim_t >= op["t_end"]:
            self.drum_running = False
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
        elif isinstance(i, float):
            if not (math.isfinite(i) and i == int(i) and abs(i) < 2 ** 53):
                if m.get("cmd") in ("stop", "estop"):
                    self._cmd_stop(m["cmd"] == "estop")    # a stop is always obeyed, whatever the id
                return '{"id":null,"ok":false,"err":"BAD_ID"}'
            i = int(i)
        c = m.get("cmd")
        if not isinstance(c, str):
            return _dump({"id": i, "ok": False, "err": "NO_CMD"})
        if c in self.ignore:
            return None
        extra: Dict[str, Any] = {}
        err = self.handle(m, extra)
        r: Dict[str, Any] = {"id": i}
        if c == "get" and not err:
            r.update(extra)                  # get: val (or min, max) before ok
            extra = {}
        r["ok"] = err is None
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
            if len(s) > MAX_LINE:
                s = '{"id":null,"ok":false,"err":"REPLY_OVERFLOW"}'
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
                "reel": self._c_reel, "tidy": self._c_tidy, "retrieve": self._c_retrieve,
                "decock": lambda m, x: self._cmd_cal("decock", m), "jog": self._c_jog,
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
        extra.update(phase=self.phase, aimed=self.aimed, aim_known=self.aim_known, drum_known=self.drum_known,
                     wraps_known=self.wraps_known, magnet="OK", glitches=0, hx=bool(self.load_cell),
                     peak_a=_n(self.peak_a, 1), fw=FW, ev_drops=self.ev_drops)
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
        self.token_expired = False
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

    def _c_tidy(self, m: dict, extra: dict) -> Optional[str]:
        """D22 / 10.1 step 11: take-up only, forward by time (collar NEUTRAL). Ends with TIDIED."""
        ms = self._num(m, "ms")
        duty = self._num(m, "duty")
        ms = float(TIDY_MS) if ms is None else ms
        if not (0 < ms <= 60000):
            return "BAD_ARG"
        if not self.takeup_zone():
            return "POSITION_UNKNOWN" if self.code_bad else "NOT_IN_NEUTRAL"
        e = self._cmd_reel(0.3 if duty is None else duty, self.settings["limit_n"], "stop", "tidy")
        if not e:
            self.op.update(until="time", t_end=self.sim_t + ms / 1000.0)
        return e

    def _c_retrieve(self, m: dict, extra: dict) -> Optional[str]:
        """10.1 step 17: take-up until SW_SEATED in NEUTRAL only; DART_STUCK after max_s (approach_s())."""
        duty = self._num(m, "duty")
        max_n = self._num(m, "max_n")
        max_s = self._num(m, "max_s")
        if max_s is not None and not 5 <= max_s <= 600:
            return "BAD_ARG"
        if max_s is None:
            max_s = approach_s(self.line_est if self.line_known else None, 0.3 if duty is None else duty)
        return self._cmd_reel(0.3 if duty is None else duty, self.settings["limit_n"] if max_n is None else max_n,
                              "seated", "retrieve", max_s)

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
        """Wiper and drum off now (no event)."""
        self.token = None
        if self.state in ("COCK", "FIRE", "DECOCK"):
            self._phase_lost()
        self.state = "IDLE"
        self.op = {}
        self.drum_running = False
        self.rpm = self.amps = 0.0

    def _cmd_stop(self, estop: bool) -> None:
        self.token = None
        was = self.state
        self.drum_running = False
        if was in ("FAULT", "SAFING"):
            self.state = "FAULT"
            self.op = {}
        else:
            if was != "IDLE":
                self.evt("ABORTED", what=was, why="estop" if estop else "stop")
            if was in ("COCK", "FIRE", "DECOCK"):
                self._phase_lost()           # stopped mid-cycle: re-establish (cock or decock)
            self.state = "IDLE"
            self.op = {}
        if estop and self.state != "FAULT":
            hold = self._hold_reason(None)
            st = self._safe_target()
            if not hold and not self._at_park(st):
                self._start_drum(st, "estop")
                self.op["jog_end"] = self.sim_t   # the estop move never turns the wiper
                self.state = "MODE"
            elif hold and not self._at_park(st):
                self._ev_mode(False, "DRUM_HELD", hold)
            elif self._at_park(st):
                self._ev_mode(True)

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
        if self.code_bad:
            return "POSITION_UNKNOWN"
        zi = self.zi()
        if to == "REEL":                     # REEL engages only with the wraps on: the fly-in from the park
            if self.zone() == "REEL" and self.mode == "REEL":
                self._ev_mode(True)
                return None
            if zi < ZI["N_TO_R"]:
                return "NOT_WRAPPED"
        elif zi >= ZI["WRAP"]:
            return "WRAPS_ON"                # unwrap first
        if self.detent() == to and abs(self.drum_deg - DRUM_DEG[to]) < 3 and (to != "LAUNCH" or self.mode == "LAUNCH"):
            self._ev_mode(True)
            return None
        if (zi <= ZI["L_TO_N"] or self.dog_l() or self.loaded_known()) and to != "LAUNCH":
            e = self.launch_exit_check()
            if e:
                return e
        self._start_drum(to, "mode", self.fail_next.pop("mode", None))
        self.aimed = False
        self.token = None
        self.state = "MODE"
        return None

    def _cmd_aim(self, deg: float) -> Optional[str]:
        e = self._idle_check(True)
        if e:
            return e
        if self.state == "HOLD":
            return "HOLDING"
        if not self.launch_centred():
            return "NOT_IN_LAUNCH"
        if self.phase == "UNKNOWN":
            return "PHASE_UNKNOWN"
        if self.phase != "REST":
            return "NOT_AT_REST"
        lo = max(PITCH_MIN, min(AIM_TABLE) + self.aim_offset)
        hi = min(PITCH_MAX, max(AIM_TABLE) + self.aim_offset)
        if not (lo - 1e-3 <= deg <= hi + 1e-3):
            return "OUT_OF_RANGE"
        self.aim_target = deg
        self.aimed = False
        self.token = None
        if self.aim_known:
            t = self.aim_travel_to(deg)
            if t < 0:
                return "OUT_OF_RANGE"
            self.op = {"homing": False, "left": t, "total": max(t, 1e-6)}
            if t <= 1e-9:
                self.op["settle"] = self.sim_t
        else:
            self.op = {"homing": True, "left": 0.0, "total": 1.0}
        f = self.fail_next.pop("aim", None)
        if f:
            self.fail_next["_aim_now"] = f
        self.state = "AIM"
        return None

    def _cmd_cock(self) -> Optional[str]:
        e = self._idle_check(True)
        if e:
            return e
        if self.state == "HOLD":
            return "HOLDING"
        if not self.launch_centred():
            return "NOT_IN_LAUNCH"
        if self.phase == "COCKED":
            if not self.home_closed():
                self.evt("COCKED", ok=True, cock_deg=_n(self.cock_deg, 1))
                return None
            return "SWITCH_DISAGREE"
        if self.phase == "REST" and not self.home_closed():
            return "SWITCH_DISAGREE"
        if self.phase == "UNKNOWN" and not self.home_closed():
            return "LOADED_UNKNOWN"          # carriage drawn, angle unknown: decock first
        if self.require_seated and not self.seated():
            return "NOT_SEATED"              # lens B I2 (base-1.2.1)
        if self.phase == "UNKNOWN" and HOME_OPEN_DEG + HOME_WIN_DEG < self.phase_hint < RELEASE_DEG + RELEASE_WIN_DEG:
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
        if not self.launch_centred():
            return "NOT_IN_LAUNCH"
        if not self.wraps_known or self.wraps() > 0:
            return "WRAPS_ON"
        gap_ok = self.gap_closed() if self.phase == "REST" else not self.gap_closed()
        cocked = self.phase == "COCKED" and not self.home_closed()
        at_rest = self.phase == "REST" and self.home_closed()     # fire from rest runs the whole cycle
        if not (cocked or at_rest) or not gap_ok:
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
            return "TOKEN_EXPIRED" if self.token_expired else "NOT_ARMED"
        if self.sim_t - self.token_t > TOKEN_S:
            self.token = None
            self.token_expired = False
            return "TOKEN_EXPIRED"
        self.token_expired = False
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
        """D22: from LAUNCH or NEUTRAL through the WRAP sector to the WRAPPED park (314, collar NEUTRAL)."""
        e = self._idle_check()
        if e:
            return e
        if self.state == "HOLD":
            return "HOLDING"
        if self.zone() in ("N_TO_R", "REEL"):
            self.evt("WRAPPED", ok=True, wraps=self.wraps())
            return None
        if self.code_bad:
            return "POSITION_UNKNOWN"
        if self.zi() <= ZI["L_TO_N"] or self.dog_l() or self.loaded_known():
            e = self.launch_exit_check()
            if e:
                return e
        if self.loaded_known():
            return "LAUNCH_BUSY"
        self._start_drum("WRAPPED", "wrap", self.fail_next.pop("wrap", None))
        self.aimed = False
        self.token = None
        self.state = "WRAP"
        return None

    def _cmd_unwrap(self) -> Optional[str]:
        """Back to NEUTRAL (the take-up winds the freed line). UNWRAPPED, then MODE NEUTRAL."""
        e = self._idle_check()
        if e:
            return e
        if self.state == "HOLD":
            return "HOLDING"
        z = self.zone()
        if z == "NEUTRAL":
            self.evt("UNWRAPPED", ok=True)
            return None
        if z is None:
            return "POSITION_UNKNOWN"
        if self.zi() < ZI["WRAP_PREP"]:
            return "NOT_WRAPPED"
        if self.loaded_known():
            return "LAUNCH_BUSY"
        self._start_drum("NEUTRAL", "unwrap", self.fail_next.pop("unwrap", None))
        self.state = "UNWRAP"
        return None

    def _cmd_reel(self, duty: float, max_n: float, until: Optional[str], kind: str = "reel",
                  max_s: Optional[float] = None) -> Optional[str]:
        """REEL: the capstan winds. NEUTRAL / the WRAPPED park: the take-up only (at most 40 percent duty)."""
        e = self._idle_check(True)
        if e:
            return e
        u = "stop" if until is None else until
        if u not in ("stop", "seated", "tension", "slip"):
            return "BAD_ARG"
        if not (0 < duty <= 1):
            return "BAD_ARG"
        if not (max_n > 0):
            return "BAD_ARG"
        neutral = self.takeup_zone()
        if self.mode != "REEL" and not neutral:
            return "NOT_WRAPPED"
        if neutral and u == "tension":
            return "NOT_WRAPPED"             # no tension without the capstan
        if not neutral and u == "slip":
            return "BAD_ARG"
        if not neutral and u == "seated":
            return "WRAPS_ON"                # D35: never seat the dart on the capstan: unwrap first
        max_n = min(max_n, LIMIT_MAX_N)
        duty = max(duty, REEL_MIN_DUTY)
        if neutral:
            duty = min(duty, TAKEUP_MAX_DUTY)
        f = self.fail_next.pop(kind, None)
        if f:
            self.fail_next["_reel_now"] = f
        s = self.settings
        if neutral:
            trip = TAKEUP_TRIP_A
        elif u == "tension":                 # D34 tow trip: above what the limit needs, never below tow_trip_a
            need = (max_n - s["tension_offset_n"]) / s["k_n_per_a"] + s["free_a"] + 1.0
            trip = min(HARD_TRIP_A, max(s["tow_trip_a"], need))
        else:
            trip = s["trip_a"]
        self.op = {"until": u, "max_n": max_n, "duty": duty, "t0": self.sim_t, "takeup": neutral, "trip": trip,
                   "max_s": APPROACH_S if max_s is None else max_s}
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
        z, p = self.zone(), self.drum_pos()
        takeup = False
        if motor == "wiper":
            if z in ("L_TO_N", "N_TO_R", None):
                return "SHIFTING"            # dogs half engaged or position unknown
            if (p == "LAUNCH" or self.dog_l()) and not self.launch_centred():
                return "NOT_IN_LAUNCH"
            if self.loaded_known() and p != "LAUNCH":
                return "LAUNCH_BUSY"
            if p == "LAUNCH" and dd == COCK_DIR and self.phase in ("COCKED", "UNKNOWN", "FIRING", "RESETTING"):
                return "WOULD_FIRE"
            if p == "LAUNCH" and dd == AIM_DIR and self.phase != "REST":
                return "NOT_AT_REST"
            if (z == "REEL" or self.dog_r()) and dd != REEL_DIR:
                return "BACKSTOP"
            takeup = ZI["NEUTRAL"] <= ZI[z] <= ZI["WRAP"] and dd == REEL_DIR
            if p == "LAUNCH" and dd == AIM_DIR:
                self.aimed = False
        else:
            if self.code_bad:
                return "POSITION_UNKNOWN"
            if (ZI[z] <= ZI["L_TO_N"] or self.loaded_known()) and not (z == "LAUNCH" and dd < 0):
                e = self.launch_exit_check()
                if e:
                    return e
            if z == "REEL" and dd < 0:
                return "WRAPS_ON"            # leaving REEL needs the centering jog: use unwrap
            self.aimed = False
        self.token = None
        self.op = {"motor": motor, "dir": dd, "t_end": self.sim_t + ms / 1000.0, "takeup": takeup}
        self.state = "JOG"
        return None

    def _decock(self, m: dict) -> Optional[str]:
        """decock / cal decock: run BACKWARD until SW_HOME closes (springs relaxed), RESET how decock."""
        e = self._idle_check(True)
        if e:
            return e
        if self.detent() != "LAUNCH":
            return "NOT_IN_LAUNCH"
        if self.home_closed():
            if self.phase == "REST":
                self.evt("RESET", ok=True, how="decock")
                return None
            drawn = HOME_OPEN_DEG + HOME_WIN_DEG < self.phase_hint < RELEASE_DEG + RELEASE_WIN_DEG
            if self.phase == "UNKNOWN" and not drawn:
                self.evt("RESET", ok=False, how="decock", why="carriage home: nothing to decock, cock finds the angle")
                return None
            if m.get("force") is not True:
                return "SWITCH_DISAGREE"     # the spring should be drawn but SW_HOME says home: lying switch
            self.phase, self.cock_deg, self.phase_hint = "REST", 0.0, -1.0   # forced: back by the travel
            self.aim_known = self.aimed = False
            self.token = None
            self.evt("RESET", ok=True, how="decock by travel")
            return None
        self.aim_known = self.aimed = False  # backward also turns the aim train: aim again
        self.token = None
        f = self.fail_next.pop("decock", None)
        if f:
            self.fail_next["_decock_now"] = f
        self.op = {"home_at": None}
        self.state = "DECOCK"
        return None

    def _cmd_cal(self, what: str, m: dict) -> Optional[str]:
        if what == "current":
            return "BUSY" if self.moving() else None
        if what == "tare":
            return None if self.load_cell else "NO_SENSOR"
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
            self.aim_phase = self.settings["aim_home_deg"] % AIM_CYCLE_DEG
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
            if not self.home_closed() and m.get("force") is not True:
                return "SWITCH_DISAGREE"
            self.phase, self.cock_deg, self.phase_hint = "REST", 0.0, -1.0
            return None
        if what == "decock":
            return self._decock(m)
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
            extra.update(min=_n(max(PITCH_MIN, min(AIM_TABLE) + self.aim_offset), 2),
                         max=_n(min(PITCH_MAX, max(AIM_TABLE) + self.aim_offset), 2))
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
    t_end = time.monotonic() + 2.0
    while not mb.booted and time.monotonic() < t_end:   # BOOT comes 60 ms (simulated) after HELLO
        time.sleep(0.005)
    return mb, c


def serve_pty(time_scale: float = 1.0, **kw) -> MockBase:
    end = PtyEnd()
    mb = MockBase(end, time_scale=time_scale, **kw).start()
    mb.pty_path = end.path
    return mb


__all__ = ["MockBase", "World", "PtyEnd", "PipeEnd", "connected_pair", "serve_pty", "DRUM_DEG", "WRAP_DEG",
           "N_WRAPS", "STATES", "PHASES", "FAULTS", "SW_KEYS", "ZONES", "FW"]
