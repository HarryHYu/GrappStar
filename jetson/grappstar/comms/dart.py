"""KEDGE dart client: HTTP on the dart access point (contract C2). Standard library only.

The dart is a WiFi access point "KEDGE" at 192.168.4.1. Endpoints:
  GET /status                 JSON: rail_V, current_mA, roll, pitch, seq, armed, landed, event, phase (C2 adds phase)
  GET /key?c=<key>&ms=<run>   run a motor key for ms (hold-to-run: repeat every 150 ms, each repeat 350 ms)
  GET /stop                   all motors off
  GET /auto                   auto-anchor sequence
  GET /release                release sequence (drills out, foot out, kick back, deploy up)   (C2, new)
  GET /flightlog              {"hz", "t_launch_ms", "t_land_ms", "samples": [[t_ms, qw, qx, qy, qz, ax, ay, az], ...]}

Keys: 1/! deploy down/up, 2/@ kick out/back, 3/# drills in/out, 4/$ foot in/out, A arm auto-anchor on landing.

Older firmware (no phase field) still works: phase is derived from seq, armed and landed.
"""
from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Callable, Dict, List, Optional

KEYS = {"1": "deploy down", "!": "deploy up", "2": "kick out", "@": "kick back", "3": "drills in",
        "#": "drills out", "4": "foot in", "$": "foot out"}
PHASES = ("IDLE", "ARMED", "FLIGHT", "LANDED", "ANCHORING", "ANCHORED", "RELEASING", "FAULT")


class DartError(Exception):
    """The dart could not be reached or answered something unusable."""


def derive_phase(s: dict) -> str:
    """phase for firmware that does not send it yet (kedge_dart v3: seq, armed, landed)."""
    if s.get("phase") in PHASES:
        return s["phase"]
    seq = str(s.get("seq", "idle"))
    if seq == "anchored":
        return "ANCHORED"
    if seq == "aborted":
        return "FAULT"
    if seq.startswith("release"):
        return "RELEASING"
    if seq != "idle":
        return "ANCHORING"
    if s.get("landed"):
        return "LANDED"
    if s.get("armed"):
        return "ARMED"
    return "IDLE"


class Dart:
    def __init__(self, host: str = "192.168.4.1", timeout: float = 1.0, retries: int = 1,
                 on_line: Optional[Callable[[str, dict], None]] = None):
        self.host = host
        self.base = host if host.startswith("http") else f"http://{host}"
        self.timeout = timeout
        self.retries = retries
        self.on_line = on_line
        self.last: Optional[dict] = None     # latest status
        self.last_t = 0.0
        self.ok_t = 0.0
        self.fails = 0
        self.last_error = ""
        self._lock = threading.Lock()

    # ------------------------------------------------ http
    def _get(self, path: str, timeout: Optional[float] = None, retries: Optional[int] = None) -> str:
        tries = 1 + (self.retries if retries is None else retries)
        err: Optional[Exception] = None
        for i in range(tries):
            try:
                with urllib.request.urlopen(self.base + path, timeout=timeout or self.timeout) as r:
                    body = r.read().decode("utf-8", errors="replace")
                self.ok_t = time.time()
                self.fails = 0
                return body
            except urllib.error.HTTPError as e:
                err = e
                if e.code == 404:
                    raise DartError(f"dart does not know {path.split('?')[0]} (old firmware?)") from e
            except Exception as e:  # timeout, refused, no route (not on the KEDGE WiFi)
                err = e
            if i + 1 < tries:
                time.sleep(0.05)
        self.fails += 1
        self.last_error = f"{type(err).__name__}: {err}"
        raise DartError(f"dart unreachable at {self.base}{path.split('?')[0]}: {self.last_error}")

    def _log(self, kind: str, obj: dict) -> None:
        if self.on_line:
            try:
                self.on_line(kind, obj)
            except Exception:
                pass

    # ------------------------------------------------ API
    def status(self) -> dict:
        s = json.loads(self._get("/status"))
        if not isinstance(s, dict):
            raise DartError("status is not a JSON object")
        s["phase"] = derive_phase(s)
        with self._lock:
            self.last, self.last_t = s, time.time()
        return s

    def key(self, c: str, ms: int = 350) -> str:
        if c not in KEYS and c not in ("A", "a"):
            raise ValueError(f"unknown dart key {c!r}")
        self._log("dart_tx", {"key": c, "ms": int(ms)})
        return self._get("/key?" + urllib.parse.urlencode({"c": c, "ms": int(ms)}), retries=0)

    def stop(self) -> str:
        self._log("dart_tx", {"stop": True})
        return self._get("/stop", retries=2)

    def auto(self) -> str:
        self._log("dart_tx", {"auto": True})
        return self._get("/auto")

    def release(self) -> str:
        self._log("dart_tx", {"release": True})
        return self._get("/release")

    def arm_landing(self) -> bool:
        """Arm auto-anchor on landing (key A toggles in kedge_dart v3, so only press it when not armed)."""
        s = self.status()
        if not s.get("armed"):
            self.key("A", 0)
            s = self.status()
        return bool(s.get("armed"))

    def flightlog(self, timeout: float = 3.0) -> dict:
        d = json.loads(self._get("/flightlog", timeout=timeout))
        if not isinstance(d, dict) or "samples" not in d:
            raise DartError("flightlog missing samples")
        return d

    def safe_stop(self) -> bool:
        try:
            self.stop()
            return True
        except Exception:
            return False

    def hold(self, keys, seconds: float, period: float = 0.15, ms: int = 350,
             cancel: Optional[threading.Event] = None) -> None:
        """Hold one or more keys like the web page (deadman): repeat every 150 ms, each repeat runs 350 ms.
        Always ends with /stop. cancel (threading.Event) ends early."""
        if isinstance(keys, str):
            keys = [keys]
        t_end = time.time() + seconds
        try:
            while time.time() < t_end and not (cancel and cancel.is_set()):
                for k in keys:
                    self.key(k, ms)
                time.sleep(period)
        finally:
            self.safe_stop()

    def wait_phase(self, phases, timeout: float, poll: float = 0.1, fail: tuple = ("FAULT",),
                   cancel: Optional[threading.Event] = None, log: Optional[Callable[[str], None]] = None) -> dict:
        """Poll /status until phase is in phases. Raises DartError on a fail phase or timeout.
        A few missed polls are tolerated (WiFi drops); a long outage raises."""
        if isinstance(phases, str):
            phases = (phases,)
        t_end = time.time() + timeout
        last_ev = None
        misses = 0
        while time.time() < t_end:
            if cancel and cancel.is_set():
                raise DartError("cancelled")
            try:
                s = self.status()
                misses = 0
            except DartError:
                misses += 1
                if misses > max(10, int(3.0 / poll)):
                    raise
                time.sleep(poll)
                continue
            if log and s.get("event") != last_ev:
                last_ev = s.get("event")
                log(f"dart {s['phase']}: {last_ev}")
            if s["phase"] in phases:
                return s
            if s["phase"] in fail and s["phase"] not in phases:
                raise DartError(f"dart {s['phase']}: {s.get('event')}")
            time.sleep(poll)
        raise DartError(f"dart did not reach {'/'.join(phases)} in {timeout:.0f} s")

    def link(self) -> dict:
        now = time.time()
        return {"connected": self.ok_t > 0 and now - self.ok_t < 3.0 and self.fails < 3, "host": self.host,
                "age_s": round(now - self.ok_t, 2) if self.ok_t else None, "fails": self.fails,
                "last_error": self.last_error}


class DartPoller:
    """Polls /status in the background (5 Hz) so the UI always has a recent value."""

    def __init__(self, dart: Dart, hz: float = 5.0):
        self.dart, self.period = dart, 1.0 / hz
        self._stop = threading.Event()
        self.th = threading.Thread(target=self._run, name="dart-poll", daemon=True)

    def start(self) -> "DartPoller":
        self.th.start()
        return self

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self.dart.status()
            except Exception:
                pass
            self._stop.wait(self.period)


def flightlog_arrays(fl: dict) -> Dict[str, List[float]]:
    """Columns of a flightlog: t (s, dart clock), q (list of 4-tuples), a (list of 3-tuples), amag (m/s^2)."""
    t, q, a, amag = [], [], [], []
    for row in fl.get("samples", []):
        if len(row) < 8:
            continue
        t.append(row[0] / 1000.0)
        q.append(tuple(row[1:5]))
        a.append(tuple(row[5:8]))
        amag.append((row[5] ** 2 + row[6] ** 2 + row[7] ** 2) ** 0.5)
    return {"t": t, "q": q, "a": a, "amag": amag}
