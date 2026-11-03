"""Base station client: USB serial, JSON lines (contract C1).

The base (ESP32-S3 DevKitC) sends one JSON object per line:
  replies     {"id": 17, "ok": true, ...} or {"id": 17, "ok": false, "err": "NOT_IN_LAUNCH"}
  telemetry   {"tel": {...}}             10 Hz, 20 Hz while anything moves
  events      {"evt": "FIRED", ...}
The Jetson sends {"id": 17, "cmd": "aim", "deg": 32.5}. Every command gets exactly one reply.

BaseClient runs one reader thread. It reconnects by itself when the USB cable is pulled or the board resets,
matches replies to commands by id, keeps the latest telemetry, and calls your callbacks for telemetry and events.
All public methods are thread safe.

  from grappstar.comms.base import BaseClient
  b = BaseClient("/dev/grappstar-base"); b.start()
  b.mode("LAUNCH"); b.wait_event("MODE", timeout=10); b.aim(32.5)

The contract is firmware/base/PROTOCOL.md (fw "base-1.0"); Jetson-side notes in jetson/docs/C1_NOTES.md. A shot is:
mode LAUNCH (MODE), aim (AIMED), cock (COCKED, optional), arm (token), fire (FIRED, then RESET). In wait_event a
FAULT, an ABORTED, an awaited event with "ok": false, and a HELLO (the base restarted) raise BaseError. A HELLO also
fails every command still waiting for its reply.
"""
from __future__ import annotations

import collections
import json
import logging
import os
import threading
import time
from typing import Any, Callable, Deque, Dict, List, Optional

log = logging.getLogger("grappstar.base")

MAX_LINE = 256          # C1: at most 256 bytes per line
DEFAULT_TIMEOUT = 1.0   # seconds to wait for a reply

MODES = ("LAUNCH", "NEUTRAL", "REEL")
TEL_MODES = MODES + ("NONE",)              # telemetry "mode" is derived from the drum: NONE between detents
DRUM_POS = ("LAUNCH", "NEUTRAL", "REEL", "WRAPPED", "BETWEEN", "UNKNOWN")
STATES = ("IDLE", "MODE", "AIM", "COCK", "FIRE", "WRAP", "UNWRAP", "REEL", "HOLD", "JOG", "DECOCK", "SAFING",
          "FAULT")
PHASES = ("REST", "COCKING", "COCKED", "FIRING", "RESETTING", "UNKNOWN")
EVENTS = ("HELLO", "BOOT", "MODE", "AIMED", "COCKED", "FIRED", "RESET", "WRAPPED", "UNWRAPPED", "SEATED", "STALL",
          "LIMIT", "ABORTED", "FAULT", "OVERFLOW")
FAULTS = ("STALL", "OVERCURRENT", "DRUM_TIMEOUT", "DRUM_SENSOR", "ANGLE_SENSOR", "SWITCH_DISAGREE", "BROWNOUT",
          "BAD_STATE")
# reply error codes (base_logic.h); events can also carry err (AIMED: OUT_OF_RANGE, AIM_MISS; MODE: DRUM_HELD or a why)
ERRORS = ("TOO_LONG", "BAD_JSON", "NO_CMD", "UNKNOWN_CMD", "BAD_ARG", "BAD_KEY", "REPLY_OVERFLOW", "FAULTED", "BUSY",
          "BOOTING", "NO_POWER", "ANGLE_SENSOR", "HOLDING", "POSITION_UNKNOWN", "WRAPS_ON", "SWITCH_DISAGREE",
          "LAUNCH_BUSY", "NOT_IN_LAUNCH", "PHASE_UNKNOWN", "NOT_AT_REST", "OUT_OF_RANGE", "AIM_UNKNOWN", "NO_SENSOR",
          "LOADED_UNKNOWN", "NOT_COCKED", "NOT_AIMED", "NOT_SEATED", "NOT_ARMED", "TOKEN_EXPIRED", "BAD_TOKEN",
          "NOT_IN_REEL", "NOT_WRAPPED", "WOULD_FIRE", "BACKSTOP", "NOT_RUNNING", "STILL_PRESENT")
# telemetry "sw": closed contacts only (absent = open), one-letter keys (PROTOCOL.md section 3)
SW_KEYS = {"a": "SW_DRUM_A", "b": "SW_DRUM_B", "c": "SW_DRUM_C", "D": "SW_DOG", "S": "SW_SEATED", "H": "SW_HOME",
           "L": "SW_RAIL_LOW", "W": "SW_WRAP_HOME"}
SW_BY_NAME = {v: k for k, v in SW_KEYS.items()}


def tel_contact(tel: Optional[dict], name: str) -> bool:
    """True if the contact (SW_SEATED, SW_HOME, ... or its letter) is closed in this telemetry."""
    sw = (tel or {}).get("sw") or {}
    return bool(sw.get(SW_BY_NAME.get(name, name)))


def tel_seated(tel: Optional[dict]) -> bool:
    """Dart seated in the cradle (SW_SEATED closed)."""
    return tel_contact(tel, "SW_SEATED")


def tel_carriage_home(tel: Optional[dict]) -> bool:
    """Launch carriage fully forward (SW_HOME closed): at rest, or released."""
    return tel_contact(tel, "SW_HOME")


class BaseError(Exception):
    """The base answered ok: false. .err holds the code (for example NOT_IN_LAUNCH)."""

    def __init__(self, cmd: str, err: str, reply: Optional[dict] = None):
        super().__init__(f"base refused {cmd}: {err}")
        self.cmd, self.err, self.reply = cmd, err, reply or {}


class BaseTimeout(Exception):
    """No reply (or no awaited event) in time."""


class BaseDisconnected(Exception):
    """The serial link is down."""


# ---------------------------------------------------------------- transports
class SerialTransport:
    """pyserial on a device path (/dev/grappstar-base, /dev/ttyACM0, or a pty from the mock base)."""

    def __init__(self, port: str, baud: int = 115200):
        self.port, self.baud = port, baud
        self.ser = None
        self._buf = b""

    def open(self) -> None:
        """Open without resetting the board (review r2_J #11, round 4 #6). Linux raises DTR and RTS when the port
        opens, and pyserial then applies DTR before RTS. The ESP32-S3 USB-Serial-JTAG resets on DTR low with RTS
        high, so drop only RTS and keep DTR high: the lines go (1,1) -> (1,0), never (0,1).
        BENCH CHECK: no HELLO/BOOT event may appear when the Jetson connects (python3 -m grappstar base monitor)."""
        import serial  # pyserial, imported here so the rest works without it
        ser = serial.Serial()
        ser.port = self.port
        ser.baudrate = self.baud
        ser.timeout = 0.05
        ser.write_timeout = 1.0
        try:
            ser.dtr = True
            ser.rts = False
        except Exception:
            pass
        ser.open()
        self.ser = ser
        self._buf = b""

    def readline(self, timeout: float = 0.2) -> Optional[bytes]:
        """One line without the newline, or None when nothing complete arrived in time."""
        t_end = time.monotonic() + timeout
        while True:
            i = self._buf.find(b"\n")
            if i >= 0:
                line, self._buf = self._buf[:i], self._buf[i + 1:]
                return line.rstrip(b"\r")
            if self.ser is None:
                raise BaseDisconnected("port closed")
            chunk = self.ser.read(self.ser.in_waiting or 1)   # raises SerialException when unplugged
            if chunk:
                self._buf += chunk
                if len(self._buf) > 64 * 1024:   # garbage with no newline: drop it
                    self._buf = b""
            elif time.monotonic() >= t_end:
                return None

    def write(self, data: bytes) -> None:
        if self.ser is None:
            raise BaseDisconnected("port closed")
        self.ser.write(data)
        self.ser.flush()

    def close(self) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None


class PipeEnd:
    """One end of an in-process line pipe (used to connect BaseClient to MockBase without a pty)."""

    def __init__(self):
        self.rx: Deque[bytes] = collections.deque()
        self.cv = threading.Condition()
        self.peer: Optional["PipeEnd"] = None
        self.closed = False
        self.broken = False      # tests set this to simulate a pulled cable

    def open(self) -> None:
        if self.broken:
            raise BaseDisconnected("pipe broken")
        self.closed = False

    def readline(self, timeout: float = 0.2) -> Optional[bytes]:
        with self.cv:
            if self.broken:
                raise BaseDisconnected("pipe broken")
            if not self.rx:
                self.cv.wait(timeout)
            if self.broken:
                raise BaseDisconnected("pipe broken")
            return self.rx.popleft() if self.rx else None

    def write(self, data: bytes) -> None:
        if self.broken or self.peer is None:
            raise BaseDisconnected("pipe broken")
        p = self.peer
        with p.cv:
            for line in data.split(b"\n"):
                if line:
                    p.rx.append(line)
            p.cv.notify_all()

    def close(self) -> None:
        self.closed = True


def pipe_pair():
    a, b = PipeEnd(), PipeEnd()
    a.peer, b.peer = b, a
    return a, b


# ---------------------------------------------------------------- client
class BaseClient:
    """Thread-safe C1 client with reconnect.

    port: a device path (str) or a transport object with open/readline/write/close.
    on_tel(tel_dict) and on_evt(evt_dict) are called from the reader thread: keep them quick.
    """

    def __init__(self, port: Any = "/dev/grappstar-base", baud: int = 115200, timeout: float = DEFAULT_TIMEOUT,
                 on_tel: Optional[Callable[[dict], None]] = None, on_evt: Optional[Callable[[dict], None]] = None,
                 on_line: Optional[Callable[[str, dict], None]] = None, reconnect_s: float = 1.0):
        self.transport = SerialTransport(port, baud) if isinstance(port, str) else port
        self.port_name = port if isinstance(port, str) else type(port).__name__
        self.timeout = timeout
        self.reconnect_s = reconnect_s
        self._tel_cbs: List[Callable[[dict], None]] = [on_tel] if on_tel else []
        self._evt_cbs: List[Callable[[dict], None]] = [on_evt] if on_evt else []
        self.on_line = on_line          # on_line(direction "rx"|"tx", obj) for the JSONL mission log
        self._lock = threading.RLock()
        self._wlock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._next_id = 1
        self._pending: Dict[int, Optional[dict]] = {}
        self._events: Deque[dict] = collections.deque(maxlen=200)
        self._evt_seq = 0
        self.tel: Optional[dict] = None
        self.tel_t = 0.0                # time.time() of the latest telemetry
        self.rx_t = 0.0                 # time.time() of the latest line of any kind
        self.connected = False
        self.port_open = False
        self.session = 0                # increments at every link drop (and HELLO)
        self.errors = 0                 # bad lines seen
        self.reconnects = 0
        self.last_error = ""
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------ lifecycle
    def start(self) -> "BaseClient":
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="base-reader", daemon=True)
            self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self.transport.close()
        self._set_connected(False)

    def add_tel_callback(self, cb: Callable[[dict], None]) -> None:
        self._tel_cbs.append(cb)

    def add_evt_callback(self, cb: Callable[[dict], None]) -> None:
        self._evt_cbs.append(cb)

    def wait_connected(self, timeout: float = 5.0) -> bool:
        with self._cv:
            return self._cv.wait_for(lambda: self.connected, timeout)

    def _set_connected(self, up: bool) -> None:
        with self._cv:
            if self.connected and not up:
                self.session += 1          # systest F8: a link drop is a session break for every pending wait
                # fail every waiting command now instead of letting each one time out
                for k in list(self._pending):
                    if self._pending[k] is None:
                        self._pending[k] = {"id": k, "ok": False, "err": "DISCONNECTED", "_disconnected": True}
            self.connected = up
            self._cv.notify_all()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self.transport.open()
                self.port_open = True
                # the link counts as up only after a valid C1 JSON line arrives (r3_N #7): a wrong board on the port
                # (the dart's XIAO has the same USB id) never gets a command
                log.info("port %s open, waiting for the base to speak C1", self.port_name)
                while not self._stop.is_set():
                    line = self.transport.readline(0.2)
                    if line is not None:
                        self._handle_line(line)
            except Exception as e:  # unplugged, reset, permission, missing device
                self.last_error = f"{type(e).__name__}: {e}"
                if self.connected:
                    log.warning("base link down: %s", self.last_error)
            finally:
                was = self.connected
                self.port_open = False
                self._set_connected(False)
                try:
                    self.transport.close()
                except Exception:
                    pass
            if self._stop.is_set():
                break
            if was:
                self.reconnects += 1
            self._stop.wait(self.reconnect_s)

    def _handle_line(self, raw: bytes) -> None:
        now = time.time()
        self.rx_t = now
        try:
            text = raw.decode("utf-8", errors="replace").strip()
            if not text:
                return
            if not text.startswith("{"):   # boot messages from the ESP32 ROM or debug prints: ignore
                return
            obj = json.loads(text)
            if not isinstance(obj, dict):
                raise ValueError("not an object")
            if not ("tel" in obj or "evt" in obj or "id" in obj):
                raise ValueError("not a C1 line")
        except Exception:
            self.errors += 1
            return
        if not self.connected:
            self._set_connected(True)
            log.info("base link up on %s", self.port_name)
        if len(raw) > MAX_LINE:
            log.debug("base line over %d bytes (%d)", MAX_LINE, len(raw))
        if self.on_line:
            try:
                self.on_line("rx", obj)
            except Exception:
                pass
        if obj.get("evt") == "HELLO":
            # the base restarted: every command still waiting for a reply failed (PROTOCOL.md section 4)
            with self._cv:
                for k in list(self._pending):
                    if self._pending[k] is None:
                        self._pending[k] = {"id": k, "ok": False, "err": "BASE_RESET", "_reset": True}
                self._cv.notify_all()
        if "id" in obj and ("ok" in obj or "err" in obj):
            # a reply (a status reply may also carry a tel object: it is a reply, not the telemetry stream)
            with self._cv:
                i = obj.get("id")
                if i is None:
                    # TOO_LONG / BAD_JSON / REPLY_OVERFLOW come back with id null: the firmware answers in order,
                    # so it belongs to the oldest command still waiting
                    waiting = [k for k, v in self._pending.items() if v is None]
                    if waiting:
                        i = min(waiting)
                if i in self._pending and self._pending[i] is None:
                    self._pending[i] = obj
                    self._cv.notify_all()
            return
        if "tel" in obj and isinstance(obj["tel"], dict):
            with self._cv:
                self.tel = obj["tel"]
                self.tel_t = now
                self._cv.notify_all()
            for cb in list(self._tel_cbs):
                try:
                    cb(obj["tel"])
                except Exception:
                    log.exception("telemetry callback failed")
        elif "evt" in obj:
            obj.setdefault("_t", now)
            with self._cv:
                self._evt_seq += 1
                obj["_seq"] = self._evt_seq
                self._events.append(obj)
                self._cv.notify_all()
            for cb in list(self._evt_cbs):
                try:
                    cb(obj)
                except Exception:
                    log.exception("event callback failed")
        elif "id" in obj:
            with self._cv:
                i = obj.get("id")
                if i in self._pending and self._pending[i] is None:
                    self._pending[i] = obj
                    self._cv.notify_all()

    # ------------------------------------------------ commands
    def cmd(self, name: str, timeout: Optional[float] = None, check: bool = True, **args: Any) -> dict:
        """Send one command and wait for its reply.

        Raises BaseDisconnected (link down), BaseTimeout (no reply), BaseError (ok false, when check is True).
        """
        timeout = self.timeout if timeout is None else timeout
        with self._cv:
            if not self.connected:
                raise BaseDisconnected(f"base not connected ({self.last_error or 'no link yet'})")
            i = self._next_id
            self._next_id = 1 if self._next_id >= 1_000_000 else self._next_id + 1
            self._pending[i] = None
        msg = {"id": i, "cmd": name}
        msg.update({k: v for k, v in args.items() if v is not None})
        data = json.dumps(msg, separators=(",", ":")).encode()
        if len(data) > MAX_LINE:              # 256 bytes is legal (newline not counted)
            with self._cv:
                self._pending.pop(i, None)
            raise ValueError(f"command too long for C1 ({len(data)} bytes)")
        try:
            with self._wlock:
                self.transport.write(data + b"\n")
        except Exception as e:
            with self._cv:
                self._pending.pop(i, None)
            raise BaseDisconnected(f"write failed: {e}") from e
        if self.on_line:
            try:
                self.on_line("tx", msg)
            except Exception:
                pass
        with self._cv:
            got = self._cv.wait_for(lambda: self._pending.get(i) is not None, timeout)
            reply = self._pending.pop(i, None)
        if not got or reply is None:
            raise BaseTimeout(f"no reply to {name} in {timeout:.1f} s")
        if reply.get("_disconnected"):
            raise BaseDisconnected(f"link lost while waiting for {name}")
        if reply.get("_reset"):
            raise BaseError(name, "BASE_RESET", reply)
        if check and not reply.get("ok", False):
            raise BaseError(name, str(reply.get("err", "UNKNOWN")), reply)
        return reply

    # thin helpers, one per C1 command
    def ping(self, **kw) -> dict: return self.cmd("ping", **kw)
    def status(self, **kw) -> dict: return self.cmd("status", **kw)
    def stop(self, **kw) -> dict: return self.cmd("stop", **kw)
    def estop(self, **kw) -> dict: return self.cmd("estop", **kw)
    def clear(self, **kw) -> dict: return self.cmd("clear", **kw)
    def wrap(self, **kw) -> dict: return self.cmd("wrap", **kw)
    def unwrap(self, **kw) -> dict: return self.cmd("unwrap", **kw)

    def mode(self, to: str, **kw) -> dict:
        if to not in MODES:
            raise ValueError(f"mode must be one of {MODES}")
        return self.cmd("mode", to=to, **kw)

    def aim(self, deg: float, **kw) -> dict:
        return self.cmd("aim", deg=round(float(deg), 2), **kw)

    def cock(self, **kw) -> dict:
        """Optional: draw the launch spring (LAUNCH, after aim). COCKED event; already cocked: COCKED at once.
        Without it, fire from REST runs the whole cycle (about 2 s slower)."""
        return self.cmd("cock", **kw)

    def arm(self, **kw) -> Any:
        """Returns the fire token (a six digit int, valid_ms 3000). Needs LAUNCH, phase REST (carriage home) or
        COCKED, aimed, the dart seated, no wraps."""
        r = self.cmd("arm", **kw)
        if "token" not in r:
            raise BaseError("arm", "NO_TOKEN", r)
        return r["token"]

    def fire(self, token: Any, **kw) -> dict:
        """Fire with the token from arm (single use; a wrong token burns it). FIRED at the release, RESET at rest."""
        return self.cmd("fire", token=token, **kw)

    def decock(self, **kw) -> dict:
        """cal decock: unwind a drawn spring without a shot (aim must be redone). RESET {how: decock} at rest;
        the reply alone (no event) when the carriage is already home."""
        return self.cmd("cal", what="decock", **kw)

    def cal_drum(self, **kw) -> dict:
        """cal drum: the drum code is absolute, so this only reports a MODE event (POSITION_UNKNOWN if invalid)."""
        return self.cmd("cal", what="drum", **kw)

    def cal_cock_rest(self, **kw) -> dict:
        """cal cock_rest: declare the launch train at rest (operator checked; SW_HOME must be closed)."""
        return self.cmd("cal", what="cock_rest", **kw)

    def cal_aim_home(self, **kw) -> dict:
        """cal aim_home: declare the aim train at its home (pitch minimum)."""
        return self.cmd("cal", what="aim_home", **kw)

    def cal_aim_forget(self, **kw) -> dict:
        """cal aim_forget: the next aim re-homes on SW_RAIL_LOW."""
        return self.cmd("cal", what="aim_forget", **kw)

    def cal_aim(self, deg: float, **kw) -> dict:
        """cal aim: the rail really is at deg now (sets aim_offset)."""
        return self.cmd("cal", what="aim", deg=round(float(deg), 2), **kw)

    def tare(self, **kw) -> dict:
        return self.cmd("cal", what="tare", **kw)

    def reel(self, duty: float, max_n: Optional[float] = None, until: str = "stop", **kw) -> dict:
        """Wind the capstan (needs drum_pos WRAPPED; also accepted in HOLD). duty 0 < d <= 1 (firmware default 0.86,
        floor 0.27), max_n > 0 (default limit_n 20, capped at 150). Ends: SEATED (until seated), LIMIT (holding true
        and state HOLD for until tension, else IDLE), ABORTED (stop, timeout, host watchdog)."""
        if until not in ("seated", "tension", "stop", "slip"):
            raise ValueError("until must be seated, tension, stop or slip")
        duty = round(max(0.0, min(1.0, float(duty))), 3)
        if duty <= 0:
            raise ValueError("reel duty must be above 0")
        return self.cmd("reel", duty=duty, max_n=max_n, until=until, **kw)

    def jog(self, motor: str, direction: int, ms: int = 300, **kw) -> dict:
        if motor not in ("wiper", "drum") or direction not in (1, -1):
            raise ValueError("jog: motor wiper|drum, dir 1|-1")
        return self.cmd("jog", motor=motor, dir=direction, ms=int(ms), **kw)

    def cal(self, what: str, **kw) -> dict: return self.cmd("cal", what=what, **kw)
    def set(self, key: str, val: Any, **kw) -> dict: return self.cmd("set", key=key, val=val, **kw)
    def get(self, key: str, **kw) -> dict: return self.cmd("get", key=key, **kw)

    def safe_stop(self) -> bool:
        """Best-effort stop that never raises (for abort paths). Returns True if the base acknowledged."""
        for c in ("stop", "estop"):
            try:
                self.cmd(c, timeout=0.5, check=False)
                return True
            except Exception:
                continue
        return False

    # ------------------------------------------------ waiting
    def event_mark(self) -> int:
        """Sequence number of the latest event; pass it to wait_event(since=...) to ignore older events."""
        with self._cv:
            return self._evt_seq

    def events(self, since: int = 0) -> List[dict]:
        with self._cv:
            return [e for e in self._events if e["_seq"] > since]

    def wait_event(self, names, timeout: float, since: Optional[int] = None,
                   fail_on_fault: bool = True, pred: Optional[Callable[[dict], bool]] = None,
                   fail_on_not_ok: bool = True, fail_on_abort: bool = True,
                   session: Optional[int] = None) -> dict:
        """Wait for an event named in names (str or tuple) newer than since.

        Raises BaseError for: a FAULT event (err = fault code), an awaited event with "ok": false (err = its err, for
        example MODE DRUM_HELD or AIMED AIM_MISS; the event is in .reply), an ABORTED event (err "ABORTED", the
        motion was stopped, timed out or the host watchdog fired) and a HELLO (err "BASE_RESET": the base
        restarted). FAULT, ok false and ABORTED can be switched off; an event named in names is returned.
        """
        if isinstance(names, str):
            names = (names,)
        since = self.event_mark() if since is None else since
        session = self.session if session is None else session
        t_end = time.monotonic() + timeout
        with self._cv:
            while True:
                if not self.connected or self.session != session:
                    raise BaseError("wait", "BASE_RESET", {"why": "link dropped while waiting"})
                for e in self._events:
                    if e["_seq"] <= since:
                        continue
                    name = e.get("evt")
                    if name in names and (pred is None or pred(e)):
                        if fail_on_not_ok and e.get("ok") is False:
                            raise BaseError(str(name), str(e.get("err") or "NOT_OK"), e)
                        return e
                    if fail_on_fault and name == "FAULT" and "FAULT" not in names:
                        raise BaseError("wait", str(e.get("code") or e.get("fault") or "FAULT"), e)
                    if fail_on_abort and name == "ABORTED" and "ABORTED" not in names:
                        raise BaseError("wait", "ABORTED", e)
                    if name == "HELLO" and "HELLO" not in names:
                        raise BaseError("wait", "BASE_RESET", e)
                left = t_end - time.monotonic()
                if left <= 0:
                    raise BaseTimeout(f"no {'/'.join(names)} event in {timeout:.1f} s")
                self._cv.wait(min(left, 0.5))

    def wait_tel(self, pred: Callable[[dict], bool], timeout: float, what: str = "condition",
                 fail_on_fault: bool = True, session: Optional[int] = None) -> dict:
        """Wait until pred(telemetry) is true. A telemetry fault raises BaseError; a link drop raises BASE_RESET."""
        session = self.session if session is None else session
        t_end = time.monotonic() + timeout
        with self._cv:
            while True:
                if not self.connected or self.session != session:
                    raise BaseError("wait", "BASE_RESET", {"why": "link dropped while waiting"})
                t = self.tel
                if t is not None:
                    if pred(t):
                        return t
                    if fail_on_fault and t.get("fault"):
                        raise BaseError("wait", str(t["fault"]), {"tel": t})
                left = t_end - time.monotonic()
                if left <= 0:
                    raise BaseTimeout(f"base: timed out waiting for {what}")
                self._cv.wait(min(left, 0.5))

    def link(self) -> dict:
        """Link summary for the operator page."""
        now = time.time()
        return {"connected": self.connected, "port": self.port_name,
                "tel_age_s": round(now - self.tel_t, 2) if self.tel_t else None,
                "errors": self.errors, "reconnects": self.reconnects, "last_error": self.last_error}


def open_base(port: str, **kw) -> BaseClient:
    """Convenience: BaseClient(port).start() and wait briefly for the link."""
    c = BaseClient(port, **kw).start()
    c.wait_connected(3.0)
    return c


if os.environ.get("GRAPPSTAR_DEBUG"):
    logging.basicConfig(level=logging.DEBUG)
