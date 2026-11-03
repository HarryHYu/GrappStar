"""KEDGE dart client: HTTP on the dart access point (contract C2). Standard library only.

The dart is a WiFi access point "KEDGE" at 192.168.4.1. Endpoints:
  GET /status                 JSON: rail_V, current_mA, roll, pitch, seq, armed, landed, event, phase (C2 adds phase)
  GET /key?c=<key>&ms=<run>   run a motor key for ms (hold-to-run: repeat every 150 ms, each repeat 350 ms)
  GET /stop                   all motors off
  GET /auto                   auto-anchor sequence
  GET /release                release sequence: drills out, pods up, foot out, kick back (reverse of /auto)
  GET /arm?on=1&auto=0        arm the landing detector (auto=0: log only)
  GET /flightlog              {"hz", "t_launch_ms", "t_land_ms", "samples": [[t_ms, qw, qx, qy, qz, ax, ay, az], ...]}

Keys: 1/! deploy down/up, 2/@ kick out/back, 3/# drills in/out, 4/$ foot in/out, A arm auto-anchor on landing.

Older firmware (no phase field) still works: phase is derived from seq, armed and landed.
"""
from __future__ import annotations

import json
import math
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


class DartRefused(DartError):
    """HTTP 409: the dart answered and refused (nothing changed). .reason is the text after 'refused: '.
    This is an answer, not a broken link: never retry it blindly."""

    def __init__(self, path: str, reason: str):
        super().__init__(f"dart refused {path}: {reason}")
        self.path, self.reason = path, reason


def derive_phase(s: dict) -> str:
    """phase for firmware that does not send it (kedge_dart v3: seq, armed, landed). v4 sends phase."""
    if s.get("phase") in PHASES:
        return s["phase"]
    seq = str(s.get("seq", "idle"))
    if seq == "anchored":
        return "ANCHORED"
    if seq == "aborted":
        return "FAULT"
    if seq == "released":
        return "IDLE"
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
        self._clock: list = []
        self._last_tms = None

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
                self.ok_t = time.time()          # an HTTP error is still an answer: the link is up
                p = path.split('?')[0]
                if e.code == 409:
                    try:
                        body = e.read().decode("utf-8", errors="replace").strip().splitlines()[0]
                    except Exception:
                        body = "refused"
                    raise DartRefused(p, body.replace("refused:", "").strip() or "refused") from None
                if e.code == 404:
                    raise DartError(f"dart does not know {p} (old firmware?)") from e
                if e.code == 400:
                    raise DartError(f"dart says bad request for {p}") from e
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
        t0 = time.time()
        body = self._get("/status")
        t1 = time.time()
        try:
            s = json.loads(body)
        except ValueError as e:
            self.fails += 1
            raise DartError(f"dart status is not valid JSON ({len(body)} bytes)") from e
        if not isinstance(s, dict):
            raise DartError("status is not a JSON object")
        s["phase"] = derive_phase(s)
        with self._lock:
            self.last, self.last_t = s, t1
            self._clock_sample(s.get("t_ms"), t0, t1)
        return s

    def _clock_sample(self, t_ms, t0: float, t1: float) -> None:
        """Dart millis to Jetson time: offset = jetson_time - t_ms / 1000, taken from the fastest recent replies."""
        if not isinstance(t_ms, (int, float)):
            return
        rtt = t1 - t0
        off = (t0 + t1) / 2.0 - t_ms / 1000.0
        if self._last_tms is not None and t_ms < self._last_tms:   # the dart rebooted: start again
            self._clock = []
        self._last_tms = t_ms
        self._clock.append((rtt, off, t1))
        self._clock = [c for c in self._clock if t1 - c[2] < 60.0][-50:]

    def clock_offset(self) -> Optional[float]:
        """Seconds to add to a dart t_ms / 1000 to get Jetson time.time(), or None before any /status."""
        with self._lock:
            if not self._clock:
                return None
            best = sorted(self._clock)[:max(1, len(self._clock) // 4)]
            offs = sorted(c[1] for c in best)
            return offs[len(offs) // 2]

    def key(self, c: str, ms: int = 350) -> str:
        if c not in KEYS and c not in ("A", "a", "r", "s", " "):
            raise ValueError(f"unknown dart key {c!r}")
        self._log("dart_tx", {"key": c, "ms": int(ms)})
        return self._get("/key?" + urllib.parse.urlencode({"c": c, "ms": int(ms)}), retries=0)

    def stop(self, disarm: bool = True) -> str:
        """All motors off, abort any sequence. v4: also disarms unless disarm=False."""
        self._log("dart_tx", {"stop": True, "disarm": disarm})
        return self._get("/stop" if disarm else "/stop?disarm=0", retries=2)

    def arm(self, on: bool = True, auto: bool = False) -> dict:
        """Arm the landing detector (v4 /arm). auto=False only records the flight (the mission anchors after the
        landing zone verdict). Falls back to the v3 A toggle on old firmware. Returns the new status."""
        self._log("dart_tx", {"arm": on, "auto": auto})
        try:
            self._get("/arm?" + urllib.parse.urlencode({"on": int(on), "auto": int(auto)}))
        except DartRefused:
            raise
        except DartError as e:
            if "does not know" not in str(e):
                raise
            s = self.status()                     # v3: A toggles arm with auto on landing
            if bool(s.get("armed")) != on:
                self.key("A", 0)
        return self.status()

    def auto(self) -> str:
        self._log("dart_tx", {"auto": True})
        return self._get("/auto", retries=0)      # never send twice blindly; v4 ignores a repeat anyway

    def release(self) -> str:
        self._log("dart_tx", {"release": True})
        return self._get("/release", retries=0)

    def arm_landing(self, auto: bool = True) -> bool:
        """Arm the landing detector (auto=True: anchor by itself on landing)."""
        return bool(self.arm(True, auto).get("armed"))

    def flightlog(self, timeout: float = 4.0, tries: int = 3, busy_wait_s: float = 45.0) -> dict:
        """GET /flightlog. 503 (no memory on the dart) is retried. 409 "busy" (motors running: auto-anchor right
        after landing) is retried with backoff for up to busy_wait_s; other refusals (armed, in flight) raise."""
        last: Optional[Exception] = None
        t_end = time.time() + busy_wait_s
        delay = 0.5
        while True:
            try:
                return self._flightlog_once(timeout, tries)
            except DartRefused as e:
                if not e.reason.startswith("busy") or time.time() + delay > t_end:
                    raise
                time.sleep(delay)
                delay = min(delay * 1.5, 3.0)

    def _flightlog_once(self, timeout: float, tries: int) -> dict:
        last: Optional[Exception] = None
        for _ in range(tries):
            try:
                body = self._get("/flightlog", timeout=timeout, retries=0)
                try:
                    d = json.loads(body)
                except ValueError as e:
                    raise DartError(f"flightlog is not valid JSON ({len(body)} bytes)") from e
                if not isinstance(d, dict) or "samples" not in d:
                    raise DartError("flightlog missing samples")
                return d
            except DartRefused:
                raise
            except (DartError, ValueError) as e:
                last = e
                time.sleep(0.3)
        raise DartError(f"flightlog failed: {last}")

    def safe_stop(self, fast: bool = False) -> bool:
        """Best-effort stop that never raises. fast=True: one short try (for ESTOP paths)."""
        try:
            if fast:
                self._get("/stop", timeout=0.4, retries=0)
            else:
                self.stop()
            return True
        except Exception:
            return False

    def hold(self, keys, seconds: float, period: float = 0.15, ms: int = 350,
             cancel: Optional[threading.Event] = None) -> None:
        """Hold one or more keys like the web page (deadman): repeat every 150 ms, each repeat runs 350 ms.
        Ends with /stop?disarm=0 unless the dart refused the very first key: a refused hold (an /auto or /release
        running, a latched fault) moved nothing, and a /stop would abort that sequence or clear the fault.
        A lost reply counts as moved (stop to be safe). cancel (threading.Event) ends early."""
        if isinstance(keys, str):
            keys = [keys]
        t_end = time.time() + seconds
        moved = False
        try:
            while time.time() < t_end and not (cancel and cancel.is_set()):
                try:
                    for k in keys:
                        try:
                            self.key(k, ms)
                        except DartRefused:
                            raise
                        except DartError:
                            moved = True        # the request may have reached the dart
                            raise
                        moved = True
                except DartRefused as e:
                    if e.reason.startswith("stalled"):
                        moved = True
                        break           # the motor reached its end stop: the normal end of a hold
                    if e.reason.startswith("hold limit"):
                        moved = True    # the 8 s manual cap ended the move UNFINISHED (fw-dart f5487cd): raise
                    raise
                time.sleep(period)
        finally:
            if moved:
                # stop the motors but keep the landing detector armed (a full /stop disarms, v4)
                try:
                    self.stop(disarm=False)
                except Exception:
                    self.safe_stop()

    def wait_phase(self, phases, timeout: float, poll: float = 0.2, fail: tuple = ("FAULT",),
                   cancel: Optional[threading.Event] = None, log: Optional[Callable[[str], None]] = None,
                   want_seq: Optional[str] = None) -> dict:
        """Poll /status until phase is in phases (and seq == want_seq if given). Raises DartError on a fail phase,
        on seq "aborted" (a /stop ended the sequence), or on timeout. A few missed polls are tolerated (WiFi drops);
        a long outage raises. v4 asks for at most 5 Hz polling."""
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
            if s["phase"] in phases and (want_seq is None or s.get("seq") == want_seq):
                return s
            if s["phase"] in fail and s["phase"] not in phases:
                raise DartError(f"dart {s['phase']} {s.get('fault') or ''}: {s.get('event')}")
            if s.get("seq") == "aborted" and s["phase"] not in phases:
                raise DartError(f"dart sequence aborted: {s.get('event')}")
            if want_seq is not None and s.get("seq") == "aborted":
                raise DartError(f"dart sequence aborted: {s.get('event')}")
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


# ------------------------------------------------------------------ IMU axes
def _axis_matrix(axes: str):
    """'x,y,z' style mapping: body x, y, z = these chip axes (with optional '-'). Returns a 3x3 list M with
    v_body = M v_chip."""
    parts = [p.strip().lower() for p in axes.split(",")]
    if len(parts) != 3:
        raise ValueError("imu axes need three entries like 'x,y,z' or '-y,x,z'")
    M = [[0.0] * 3 for _ in range(3)]
    for i, p in enumerate(parts):
        sign = -1.0 if p.startswith("-") else 1.0
        k = "xyz".index(p.lstrip("+-"))
        M[i][k] = sign
    import numpy as np
    m = np.array(M)
    if abs(np.linalg.det(m) - 1.0) > 1e-6:
        raise ValueError(f"imu axes {axes!r} must be a rotation (a permutation with determinant +1); "
                         "a mirrored mapping means one sign is wrong")
    return m


def remap_flightlog(fl: dict, axes: str = "x,y,z") -> dict:
    """Return a copy of a C2 flightlog with vectors and quaternions in the dart BODY frame (x along the nose,
    y left, z up when level). axes = how the BNO085 chip axes map to the body (found on the bench, BENCH_TEST.md
    step 7): "x,y,z" means the chip already matches the body. The firmware does not fix this mapping."""
    if not fl or axes.replace(" ", "") in ("x,y,z", "+x,+y,+z"):
        return fl
    import numpy as np
    M = _axis_matrix(axes)
    out = dict(fl)
    rows = []
    for r in fl.get("samples", []):
        if len(r) < 8:
            continue
        qw, qx, qy, qz = r[1:5]
        R = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                      [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                      [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
        Rb = R @ M.T                      # world from body = world from chip * chip from body
        w = math.sqrt(max(0.0, 1 + Rb[0, 0] + Rb[1, 1] + Rb[2, 2])) / 2
        if w > 1e-6:
            x = (Rb[2, 1] - Rb[1, 2]) / (4 * w); y = (Rb[0, 2] - Rb[2, 0]) / (4 * w); z = (Rb[1, 0] - Rb[0, 1]) / (4 * w)
        else:
            x = math.sqrt(max(0.0, 1 + Rb[0, 0] - Rb[1, 1] - Rb[2, 2])) / 2
            y = math.copysign(math.sqrt(max(0.0, 1 - Rb[0, 0] + Rb[1, 1] - Rb[2, 2])) / 2, Rb[0, 1] + Rb[1, 0])
            z = math.copysign(math.sqrt(max(0.0, 1 - Rb[0, 0] - Rb[1, 1] + Rb[2, 2])) / 2, Rb[0, 2] + Rb[2, 0])
        a = M @ np.array(r[5:8], dtype=float)
        rows.append([r[0], round(w, 5), round(x, 5), round(y, 5), round(z, 5)] + [round(float(v), 3) for v in a])
    out["samples"] = rows
    out["axes"] = axes
    return out
