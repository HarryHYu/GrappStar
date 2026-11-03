"""Simulated base station that speaks contract C1, for tests and the --sim operator page.

Run it in-process (connected to BaseClient through a pipe) or on a pseudo terminal so the real client (or a terminal
program) can open it like the USB port:

  python3 -m grappstar mockbase --pty        # prints /dev/pts/N; then: python3 -m grappstar base --port /dev/pts/N status

Model (simplified but faithful to C1):
- Selector drum (N20): positions on one line, LAUNCH 0 deg, NEUTRAL 90, REEL 180, then the wrap arm: each wrap is
  another WRAP_DEG past REEL; WRAPPED when all wraps are on. The drum moves at DRUM_DPS. The mode cannot change while
  wraps are on, and wrap only works in REEL (the physical order on the drum).
- Wiper in LAUNCH: one direction aims (pitch moves at AIM_DPS), the other cocks then fires (cock 0 to 1, COCKED, FIRED).
- Wiper in REEL: reels in. Line tension comes from the attached world (dart free, anchored, seated).
- Faults: inject with inject_fault(code) or arm one with fail_next[cmd] = code. Any fault stops the wiper and sends
  the drum to NEUTRAL (unless it is a drum fault, then the drum stops where it is).

Times run on a scaled clock (time_scale) so tests can go faster than real life.
"""
from __future__ import annotations

import json
import os
import random
import threading
import time
from typing import Any, Callable, Dict, Optional

from .base import PipeEnd, pipe_pair

DRUM_DEG = {"LAUNCH": 0.0, "NEUTRAL": 90.0, "REEL": 180.0}
WRAP_DEG = 60.0
N_WRAPS = 3
DRUM_DPS = 120.0          # drum speed, deg/s
AIM_DPS = 8.0             # rail pitch speed, deg/s
PITCH_MIN, PITCH_MAX = 15.0, 60.0
COCK_S = 2.0              # time to cock and release
TOKEN_S = 3.0             # arm token lifetime
REEL_MPS = 0.35           # line speed at full duty with light load
LINE_K = 400.0            # N per metre once the line is taut (stretch plus rover compliance)
POS_TOL = 3.0             # drum position tolerance, deg


class World:
    """What the line is attached to. The default is a dart sitting free on the ground at line_m.

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
                 world: Optional[World] = None, seed: int = 0, tel_hz: float = 10.0):
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
        self.lock = threading.RLock()
        # state
        self.state = "IDLE"
        self.mode = "NEUTRAL"
        self.drum_deg = DRUM_DEG["NEUTRAL"]
        self.drum_target: Optional[float] = None
        self.drum_next_evt: Optional[tuple] = None
        self.pitch = 20.0
        self.pitch_target: Optional[float] = None
        self.aimed = False
        self.cock = 0.0
        self.token: Optional[str] = None
        self.token_t = 0.0
        self.t_fire: Optional[float] = None
        self.line_m = 0.0
        self.tension = 0.0
        self.reel_cfg: Optional[dict] = None
        self.jog_cfg: Optional[dict] = None
        self.wiper_deg = 0.0
        self.rpm = 0.0
        self.amps = 0.1
        self.vbat = 12.4
        self.fault: Optional[str] = None
        self.settings: Dict[str, Any] = {"max_n": 50, "reel_duty": 0.8}
        # test hooks
        self.fail_next: Dict[str, str] = {}   # command name -> fault code raised while executing it
        self.ignore: set = set()              # command names that get no reply (timeout tests)
        self.silent = False                   # stop sending telemetry
        self.received: list = []              # every command object received
        self.sim_t = 0.0
        self._stop = threading.Event()
        self._threads: list = []

    # ------------------------------------------------ run
    def start(self) -> "MockBase":
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

    def _send(self, obj: dict) -> None:
        try:
            self.tr.write(json.dumps(obj, separators=(",", ":")).encode() + b"\n")
        except Exception:
            pass

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                line = self.tr.readline(0.1)
            except Exception:
                time.sleep(0.05)
                continue
            if not line:
                continue
            try:
                msg = json.loads(line.decode())
            except Exception:
                continue
            if not isinstance(msg, dict) or "cmd" not in msg:
                continue
            self.received.append(msg)
            if msg["cmd"] in self.ignore:
                continue
            with self.lock:
                try:
                    reply = self.handle(msg)
                except Exception as e:  # never crash the mock
                    reply = {"ok": False, "err": "BAD_ARGS", "msg": str(e)}
            reply["id"] = msg.get("id")
            self._send(reply)

    def _sim_loop(self) -> None:
        dt_real = 0.02
        last_tel = 0.0
        while not self._stop.is_set():
            time.sleep(dt_real)
            with self.lock:
                self.step(dt_real * self.scale)
                moving = self.state not in ("IDLE", "FAULT", "HOLD")
                period = 1.0 / (self.tel_hz * (2 if moving else 1))   # C1: 10 Hz, 20 Hz while moving
                now = time.monotonic()
                if not self.silent and now - last_tel >= period:
                    last_tel = now
                    self._send({"tel": self.telemetry()})

    # ------------------------------------------------ model
    def drum_pos(self) -> str:
        d = self.drum_deg
        for name, deg in DRUM_DEG.items():
            if abs(d - deg) <= POS_TOL:
                return name
        if abs(d - (DRUM_DEG["REEL"] + WRAP_DEG * N_WRAPS)) <= POS_TOL:
            return "WRAPPED"
        return "BETWEEN"

    def wraps(self) -> float:
        return max(0.0, (self.drum_deg - DRUM_DEG["REEL"]) / WRAP_DEG)

    def sw(self) -> dict:
        p = self.drum_pos()
        return {"drum_launch": p == "LAUNCH", "drum_neutral": p == "NEUTRAL", "drum_reel": p == "REEL",
                "drum_wrapped": p == "WRAPPED", "seated": self.world.dart == "seated" and self.line_m <= 0.01,
                "cocked": self.cock >= 0.999}

    def telemetry(self) -> dict:
        return {"state": self.state, "mode": self.mode, "drum_deg": round(self.drum_deg, 1), "drum_pos": self.drum_pos(),
                "wiper_deg": round(self.wiper_deg, 1), "rpm": round(self.rpm, 1), "amps": round(self.amps, 2),
                "pitch_deg": round(self.pitch, 2), "cock": round(self.cock, 3), "wraps": round(self.wraps(), 2),
                "line_m": round(self.line_m, 3),
                "tension_n": round(self.tension, 1) if self.load_cell else None,
                "sw": self.sw(), "fault": self.fault, "vbat": round(self.vbat, 2)}

    def evt(self, name: str, **kw) -> None:
        d = {"evt": name}
        d.update(kw)
        self._send(d)

    def raise_fault(self, code: str, drum_to_neutral: bool = True) -> None:
        self.fault = code
        self.state = "FAULT"
        self.reel_cfg = self.jog_cfg = None
        self.pitch_target = None
        self.rpm = 0.0
        self.amps = 0.05
        if self.cock > 0 and self.t_fire is None:
            pass   # a cocked spring stays cocked (the sear holds it); the firmware must not fire on a fault
        if drum_to_neutral and code not in ("DRUM_TIMEOUT", "DRUM_SENSOR"):
            self._move_drum(DRUM_DEG["NEUTRAL"], ("MODE", {"mode": "NEUTRAL"}))
            self.mode = "NEUTRAL"
        else:
            self.drum_target = None
        self.evt("FAULT", code=code)

    def inject_fault(self, code: str) -> None:
        with self.lock:
            self.raise_fault(code)

    def _move_drum(self, target: float, evt: Optional[tuple]) -> None:
        self.drum_target = target
        self.drum_next_evt = evt

    def step(self, dt: float) -> None:
        self.sim_t += dt
        self.vbat = max(10.5, self.vbat - dt * 0.00002) if self.rpm == 0 else self.vbat
        busy_wiper = False
        # drum (keeps moving to NEUTRAL even in FAULT)
        if self.drum_target is not None:
            d = self.drum_target - self.drum_deg
            stepd = DRUM_DPS * dt
            if abs(d) <= stepd:
                self.drum_deg = self.drum_target
                self.drum_target = None
                if self.state == "DRUM":
                    self.state = "IDLE"
                if self.drum_next_evt:
                    name, kw = self.drum_next_evt
                    self.drum_next_evt = None
                    self.evt(name, **kw)
            else:
                self.drum_deg += stepd if d > 0 else -stepd
                f = self.fail_next.pop("_drum_now", None)
                if f:
                    self.raise_fault(f, drum_to_neutral=False)
        if self.state == "FAULT":
            self.rpm = 0.0
            self.amps = 0.05
            return
        # aim
        if self.state == "AIM" and self.pitch_target is not None:
            d = self.pitch_target - self.pitch
            s = AIM_DPS * dt
            busy_wiper = True
            if abs(d) <= s:
                self.pitch = self.pitch_target
                self.pitch_target = None
                self.state = "IDLE"
                self.aimed = True
                self.evt("AIMED", deg=round(self.pitch, 2))
            else:
                self.pitch += s if d > 0 else -s
                self.wiper_deg += (1 if d > 0 else -1) * s * 40
        # cock and fire
        if self.state == "COCK":
            busy_wiper = True
            self.cock = min(1.0, self.cock + dt / COCK_S)
            self.wiper_deg -= dt * 360
            f = self.fail_next.pop("_cock_now", None)
            if f:
                self.raise_fault(f)
                return
            if self.cock >= 1.0:
                self.evt("COCKED")
                self.cock = 0.0
                self.t_fire = self.sim_t
                self.state = "PAYOUT"
                self.world.dart = "flying"
                self.evt("FIRED", pitch_deg=round(self.pitch, 2))
                if self.world.on_fire:
                    try:
                        self.world.on_fire()
                    except Exception:
                        pass
        if self.state == "PAYOUT":
            t = self.sim_t - (self.t_fire or self.sim_t)
            f = self.world.flight or self.world.default_flight
            self.line_m = max(self.line_m, f(t))
            self.tension = 0.8 if self.world.dart == "flying" else 0.0
            if t > 1.5 and self.world.dart == "flying" and self.world.flight is None:
                self.world.dart = "free"
            if self.world.dart != "flying":
                self.state = "IDLE"
                self.aimed = False
        # reel
        if self.state == "REEL" and self.reel_cfg is not None:
            busy_wiper = True
            cfg = self.reel_cfg
            duty = cfg["duty"]
            max_n = cfg.get("max_n") or self.settings["max_n"]
            dart = self.world.dart
            if dart == "anchored":
                if self.world.slack_m > 0:
                    take = min(self.world.slack_m, REEL_MPS * duty * dt)
                    self.world.slack_m -= take
                    self.line_m = max(0.0, self.line_m - take)
                    self.tension = 1.0
                else:
                    # taut: tension rises with the wiper torque, speed falls (worm drive, stalls at the limit)
                    self.tension = min(self.tension + LINE_K * REEL_MPS * duty * dt * 0.3, 400.0)
                    self.line_m = max(0.0, self.line_m - 0.002 * dt)
            elif dart in ("free", "seated", "flying"):
                take = REEL_MPS * duty * dt
                self.line_m = max(0.0, self.line_m - take)
                self.tension = self.world.drag_free_n + self.rng.uniform(-0.3, 0.3) if self.line_m > 0 else 0.0
                if self.line_m <= 0.0 and dart != "seated":
                    self.world.dart = "seated"
            self.amps = 1.5 + self.tension * 0.12
            self.rpm = 60.0 * duty * max(0.1, 1.0 - self.tension / 300.0)
            self.wiper_deg += self.rpm * 6.0 * dt
            if self.amps > 25.0:
                self.raise_fault("OVERCURRENT")
                return
            until = cfg.get("until", "stop")
            if self.tension >= max_n:
                self.reel_cfg = None
                self.state = "HOLD"
                self.rpm = 0.0
                self.evt("LIMIT", tension_n=round(self.tension, 1), line_m=round(self.line_m, 3))
            elif self.sw()["seated"]:      # dart home: always stop, whatever "until" says
                self.reel_cfg = None
                self.state = "IDLE"
                self.tension = 0.0
                self.evt("SEATED", line_m=0.0)
            elif cfg.get("t_end") is not None and self.sim_t >= cfg["t_end"]:
                self.reel_cfg = None
                self.state = "IDLE"
            f = self.fail_next.pop("_reel_now", None)
            if f:
                self.raise_fault(f)
                return
        if self.state == "HOLD":
            # worm self-locks: tension stays; relaxes if the dart is released or the rover moves toward it
            if self.world.dart != "anchored":
                self.tension = self.world.drag_free_n if self.line_m > 0 else 0.0
        if self.state == "JOG" and self.jog_cfg is not None:
            busy_wiper = self.jog_cfg["motor"] == "wiper"
            if self.sim_t >= self.jog_cfg["t_end"]:
                self.jog_cfg = None
                self.state = "IDLE"
        if not busy_wiper and self.state != "REEL":
            self.rpm = 0.0
            self.amps = 0.1 if self.drum_target is None else 0.4

    # ------------------------------------------------ commands
    def handle(self, m: dict) -> dict:
        c = m.get("cmd")
        ok = {"ok": True}
        if c == "ping":
            return {"ok": True, "fw": "mock-base 1.0", "t_ms": int(self.sim_t * 1000)}
        if c == "status":
            return {"ok": True, "tel": self.telemetry()}
        if c == "get":
            k = m.get("key")
            if k not in self.settings:
                return {"ok": False, "err": "BAD_KEY"}
            return {"ok": True, "key": k, "val": self.settings[k]}
        if c == "set":
            k = m.get("key")
            if k not in self.settings:
                return {"ok": False, "err": "BAD_KEY"}
            self.settings[k] = m.get("val")
            return ok
        if c == "stop":
            self.reel_cfg = self.jog_cfg = None
            self.pitch_target = None
            if self.state not in ("FAULT",):
                self.state = "IDLE" if self.drum_target is None else "DRUM"
            if self.state == "DRUM":
                self.drum_target = None    # drum holds (worm) where it is
                self.state = "IDLE"
            self.rpm = 0.0
            return ok
        if c == "estop":
            self.reel_cfg = self.jog_cfg = None
            self.pitch_target = None
            self.rpm = 0.0
            if self.state != "FAULT":
                self.state = "DRUM"
            self.mode = "NEUTRAL"
            self._move_drum(DRUM_DEG["NEUTRAL"], ("MODE", {"mode": "NEUTRAL"}))
            return ok
        if c == "clear":
            if self.state != "FAULT":
                return ok
            self.fault = None
            self.state = "DRUM" if self.drum_target is not None else "IDLE"
            return ok
        if self.state == "FAULT":
            return {"ok": False, "err": "FAULT", "fault": self.fault}
        if c == "jog":
            if self.state not in ("IDLE", "HOLD"):
                return {"ok": False, "err": "BUSY"}
            ms = int(m.get("ms", 0))
            if not (0 < ms <= 2000) or m.get("motor") not in ("wiper", "drum") or m.get("dir") not in (1, -1):
                return {"ok": False, "err": "BAD_ARGS"}
            if m["motor"] == "drum":
                self.drum_deg += m["dir"] * DRUM_DPS * ms / 1000.0
            self.jog_cfg = {"motor": m["motor"], "t_end": self.sim_t + ms / 1000.0}
            self.state = "JOG"
            return ok
        if c == "cal":
            return {"ok": True, "what": m.get("what")}
        busy = self.state not in ("IDLE", "HOLD")
        if c == "mode":
            to = m.get("to")
            if to not in DRUM_DEG:
                return {"ok": False, "err": "BAD_ARGS"}
            if busy:
                return {"ok": False, "err": "BUSY"}
            if self.wraps() > 0.05:
                return {"ok": False, "err": "WRAPPED"}
            if self.state == "HOLD" and self.tension > 5:
                return {"ok": False, "err": "UNDER_LOAD"}
            self.mode = to
            self.aimed = False
            self.state = "DRUM"
            f = self.fail_next.pop("mode", None)
            if f:
                self.fail_next["_drum_now"] = f
            self._move_drum(DRUM_DEG[to], ("MODE", {"mode": to}))
            return ok
        if c == "aim":
            if self.mode != "LAUNCH" or self.drum_pos() != "LAUNCH":
                return {"ok": False, "err": "NOT_IN_LAUNCH"}
            if busy:
                return {"ok": False, "err": "BUSY"}
            try:
                deg = float(m.get("deg"))
            except (TypeError, ValueError):
                return {"ok": False, "err": "BAD_ARGS"}
            if not (PITCH_MIN <= deg <= PITCH_MAX):
                return {"ok": False, "err": "RANGE", "min": PITCH_MIN, "max": PITCH_MAX}
            self.pitch_target = deg
            self.state = "AIM"
            self.aimed = False
            return ok
        if c == "arm":
            if self.mode != "LAUNCH" or self.drum_pos() != "LAUNCH":
                return {"ok": False, "err": "NOT_IN_LAUNCH"}
            if busy:
                return {"ok": False, "err": "BUSY"}
            if not self.sw()["seated"]:
                return {"ok": False, "err": "NOT_SEATED"}
            self.token = "%06d" % self.rng.randrange(1_000_000)
            self.token_t = self.sim_t
            return {"ok": True, "token": self.token, "valid_ms": int(TOKEN_S * 1000)}
        if c == "fire":
            if self.mode != "LAUNCH" or self.drum_pos() != "LAUNCH":
                return {"ok": False, "err": "NOT_IN_LAUNCH"}
            if busy:
                return {"ok": False, "err": "BUSY"}
            if self.token is None:
                return {"ok": False, "err": "NOT_ARMED"}
            if str(m.get("token")) != self.token:
                self.token = None
                return {"ok": False, "err": "BAD_TOKEN"}
            if self.sim_t - self.token_t > TOKEN_S:
                self.token = None
                return {"ok": False, "err": "TOKEN_EXPIRED"}
            self.token = None
            self.state = "COCK"
            f = self.fail_next.pop("fire", None)
            if f:
                self.fail_next["_cock_now"] = f
            return ok
        if c in ("wrap", "unwrap"):
            if self.mode != "REEL":
                return {"ok": False, "err": "NOT_IN_REEL"}
            if busy:
                return {"ok": False, "err": "BUSY"}
            if c == "wrap":
                target = DRUM_DEG["REEL"] + WRAP_DEG * N_WRAPS
                ev = ("WRAPPED", {"wraps": N_WRAPS})
            else:
                if self.state == "HOLD" and self.tension > 5:
                    return {"ok": False, "err": "UNDER_LOAD"}
                target = DRUM_DEG["REEL"]
                ev = ("UNWRAPPED", {})
            self.state = "DRUM"
            f = self.fail_next.pop(c, None)
            if f:
                self.fail_next["_drum_now"] = f
            self._move_drum(target, ev)
            return ok
        if c == "reel":
            if self.mode != "REEL" or self.drum_pos() not in ("REEL", "WRAPPED"):
                return {"ok": False, "err": "NOT_IN_REEL"}
            if self.drum_pos() != "WRAPPED":
                return {"ok": False, "err": "NOT_WRAPPED"}
            if busy:
                return {"ok": False, "err": "BUSY"}
            try:
                duty = float(m.get("duty", self.settings["reel_duty"]))
            except (TypeError, ValueError):
                return {"ok": False, "err": "BAD_ARGS"}
            until = m.get("until", "stop")
            if not (0.0 <= duty <= 1.0) or until not in ("seated", "tension", "stop"):
                return {"ok": False, "err": "BAD_ARGS"}
            max_n = m.get("max_n")
            if max_n is not None and float(max_n) > 100:
                return {"ok": False, "err": "RANGE"}
            self.reel_cfg = {"duty": duty, "max_n": max_n, "until": until,
                             "t_end": (self.sim_t + float(m["ms"]) / 1000.0) if m.get("ms") else None}
            self.state = "REEL"
            f = self.fail_next.pop("reel", None)
            if f:
                self.fail_next["_reel_now"] = f
            return ok
        return {"ok": False, "err": "UNKNOWN_CMD"}


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


__all__ = ["MockBase", "World", "PtyEnd", "PipeEnd", "connected_pair", "serve_pty"]
