"""Simulated KEDGE dart HTTP API, faithful to firmware kedge-dart-v4 (firmware/dart/API.md, contract C2 final).

  python3 -m grappstar mockdart --port 8780      # then: python3 -m grappstar dart --host 127.0.0.1:8780 status

Phases (priority as in API.md): FAULT > RELEASING > ANCHORING > FLIGHT > ANCHORED > LANDED > ARMED > IDLE.
World events from the simulator or tests: launch() (only counted while armed, like the real detector), land().
Refusals are HTTP 409 with "refused: <reason>". Test hooks: fail_anchor, no_ground (/auto faults NO_GROUND: dart on its back or side, every retry too),
fail_release, no_landing, no_imu, down
(answer 503 to everything), slow_s (delay every reply), flightlog_provider.
"""
from __future__ import annotations

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Optional
from urllib.parse import parse_qs, urlparse

ANCHOR_SEQ = [(0.0, "kick swing"), (1.0, "kick + foot planting"), (1.6, "foot finishing"), (2.0, "deploy + drills"),
              (2.8, "drills finishing"), (3.4, "anchored")]
RELEASE_SEQ = [(0.0, "release: drills out"), (1.5, "release: drills out + pods up"), (2.5, "release: foot out"),
               (3.0, "release: kick back + foot out"), (4.5, "released")]
RUN_KEYS = set("1!2@3#4$")
G = 9.81
SAT = 78.0          # BNO085 saturates near 8 g


def synth_flightlog(t_launch_ms: int = 2000, flight_s: float = 1.1, pitch_deg: float = 35.0, hz: int = 50,
                    land_still_s: float = 3.0) -> dict:
    """A plausible v4 flightlog: 2 s on the rail, launch spike, ballistic flight, impact, still on the ground.
    Body frame: x forward (nose), y left, z up when level. Quaternion = world from body (w, x, y, z)."""
    samples = []
    dt = 1000.0 / hz
    t_land = t_launch_ms + int(flight_s * 1000)
    t_end = t_land + int(land_still_s * 1000)
    t = 0.0
    while t <= t_end:
        if t < t_launch_ms:
            pitch = pitch_deg
            a_body = (G * math.sin(math.radians(pitch_deg)), 0.0, G * math.cos(math.radians(pitch_deg)))
        elif t < t_launch_ms + 60:
            pitch, a_body = pitch_deg, (SAT, 0.0, G * math.cos(math.radians(pitch_deg)))
        elif t < t_land:
            f = (t - t_launch_ms) / (t_land - t_launch_ms)
            pitch = pitch_deg - f * (pitch_deg + 30.0)
            a_body = (-0.4, 0.0, 0.2)          # near free fall: drag only
        elif t < t_land + 40:
            pitch, a_body = -40.0, (-SAT, 0.5 * G, 2.0 * G)
        else:
            pitch = -40.0
            a_body = (G * math.sin(math.radians(pitch)), 0.0, G * math.cos(math.radians(pitch)))
        h = math.radians(pitch) / 2.0          # rotation about body y (left); nose up = negative about +y
        qw, qy = math.cos(h), -math.sin(h)
        samples.append([int(t), round(qw, 4), 0.0, round(qy, 4), 0.0,
                        round(a_body[0], 2), round(a_body[1], 2), round(a_body[2], 2)])
        t += dt
    return {"hz": hz, "frozen": True, "phase": "LANDED", "t_launch_ms": t_launch_ms, "t_land_ms": t_land,
            "landing": "impact", "n": len(samples), "fields": ["t_ms", "qw", "qx", "qy", "qz", "ax", "ay", "az"],
            "samples": samples}


class MockDart:
    def __init__(self, port: int = 0, host: str = "127.0.0.1", time_scale: float = 1.0):
        self.scale = time_scale
        self.lock = threading.RLock()
        self.t_boot = time.time()
        self.seq = "idle"
        self.event = "boot"
        self.fault: Optional[str] = None
        self.armed = False
        self.auto_on_landing = True
        self.landed = False
        self.flying = False
        self.seq_t0: Optional[float] = None
        self.seq_kind: Optional[str] = None
        self.t_launch_ms: Optional[int] = None
        self.t_land_ms: Optional[int] = None
        self.keys: list = []
        self.requests: list = []
        self.roll, self.pitch = 1.0, 35.0
        self.current_mA = 0
        self.peak_mA = 0
        # hooks
        self.fail_anchor = False
        self.no_ground = False
        self.fail_release = False
        self.no_landing = False
        self.no_imu = False
        self.down = False
        self.slow_s = 0.0
        self.flightlog_provider: Optional[Callable[[], dict]] = None
        self._flightlog: Optional[dict] = None
        mock = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                mock._handle(self)

        self.srv = ThreadingHTTPServer((host, port), H)
        self.srv.daemon_threads = True
        self.port = self.srv.server_address[1]
        self.host = f"{host}:{self.port}"
        self.th = threading.Thread(target=self.srv.serve_forever, name="mock-dart", daemon=True)

    def start(self) -> "MockDart":
        self.th.start()
        return self

    def stop(self) -> None:
        self.srv.shutdown()
        self.srv.server_close()

    def millis(self) -> int:
        return int((time.time() - self.t_boot) * 1000 * self.scale)

    # ------------------------------------------------ phase (API.md priority order)
    @property
    def phase(self) -> str:
        with self.lock:
            self._update()
            if self.fault:
                return "FAULT"
            if self.seq_kind == "release" and self.seq_t0 is not None:
                return "RELEASING"
            if self.seq_kind == "anchor" and self.seq_t0 is not None:
                return "ANCHORING"
            if self.flying:
                return "FLIGHT"
            if self.seq == "anchored":
                return "ANCHORED"
            if self.landed:
                return "LANDED"
            if self.armed:
                return "ARMED"
            return "IDLE"

    # ------------------------------------------------ world events
    def reboot(self, reason: str = "brownout") -> None:
        """Power dropout (the JST plug at impact): armed state and flight log are lost, millis restart."""
        with self.lock:
            self.t_boot = time.time()
            self.armed = self.flying = self.landed = False
            self._flightlog = None
            self.t_launch_ms = self.t_land_ms = None
            self.seq_t0 = self.seq_kind = None
            self.seq = "idle"
            self.fault = "BROWNOUT" if reason == "brownout" else None
            self.event = f"boot ({reason})"

    def launch(self) -> None:
        with self.lock:
            if not self.armed or self.no_imu:
                return                       # the detector only runs while armed
            self.flying = True
            self.landed = False
            self.t_launch_ms = self.millis()
            self.t_land_ms = None
            self.event = "launch detected"

    def land(self) -> None:
        with self.lock:
            if self.no_landing or not self.flying:
                return
            self.flying = False
            self.landed = True
            self.armed = False
            self.t_land_ms = self.millis()
            self.pitch = -40.0
            self.event = "landed (impact)"
            t_l = self.t_launch_ms or max(0, self.t_land_ms - 1100)
            self._flightlog = (self.flightlog_provider() if self.flightlog_provider else
                               synth_flightlog(t_launch_ms=t_l, flight_s=max(0.3, (self.t_land_ms - t_l) / 1000.0)))
            if self.auto_on_landing:
                self._start_seq("anchor")

    def _start_seq(self, kind: str) -> None:
        self.seq_kind = kind
        self.seq_t0 = time.time()
        self.fault = None
        self.peak_mA = 0

    def _update(self) -> None:
        if self.seq_t0 is None:
            return
        el = (time.time() - self.seq_t0) * self.scale
        if self.seq_kind == "anchor":
            if self.no_ground and el >= 1.0:
                self.seq, self.event, self.fault = "aborted", "auto: kick hit its end stop in air", "NO_GROUND"
                self.seq_t0, self.seq_kind, self.current_mA = None, None, 0
                return
            if self.fail_anchor and el >= 1.6:
                self.seq, self.event, self.fault = "aborted", "auto: kick found no ground", "KICK_TIMEOUT"
                self.seq_t0, self.seq_kind, self.current_mA = None, None, 0
                return
            for t, n in ANCHOR_SEQ:
                if el >= t:
                    self.seq, self.event = n, "auto: " + n
            self.current_mA = 420 if self.seq != "anchored" else 0
            self.peak_mA = max(self.peak_mA, self.current_mA)
            if self.seq == "anchored":
                self.seq_t0, self.seq_kind = None, None
                self.landed = False
        else:
            if self.fail_release and el >= 2.0:
                self.seq, self.event, self.fault = "aborted", "release: drills jammed", "DRILL_JAM"
                self.seq_t0, self.seq_kind, self.current_mA = None, None, 0
                return
            for t, n in RELEASE_SEQ:
                if el >= t:
                    self.seq, self.event = n, n
            self.current_mA = 380 if self.seq != "released" else 0
            if self.seq == "released":
                self.seq_t0, self.seq_kind = None, None
                self.landed = False

    def status_dict(self) -> dict:
        with self.lock:
            ph = self.phase
            return {"fw": "kedge-dart-v4-mock", "t_ms": self.millis(), "phase": ph, "fault": self.fault,
                    "rail_V": 7.62, "current_mA": self.current_mA, "peak_mA": self.peak_mA, "speed": 255,
                    "motors": {"deploy": 0, "kick": 0, "drills": 0, "foot": 0}, "seq": self.seq,
                    "armed": self.armed, "auto_on_landing": self.auto_on_landing, "landed": self.landed,
                    "roll": self.roll, "pitch": self.pitch, "accel_g": 1.0, "peak_g": 8.0 if self.t_land_ms else 1.0, "ina_ok": True,
                    "imu_ok": not self.no_imu, "t_launch_ms": self.t_launch_ms, "t_land_ms": self.t_land_ms,
                    "log_frozen": self._flightlog is not None, "reset": "poweron", "event": self.event}

    # ------------------------------------------------ http
    def _handle(self, h: BaseHTTPRequestHandler) -> None:
        u = urlparse(h.path)
        q = parse_qs(u.query)
        self.requests.append((time.time(), u.path, u.query))
        if self.slow_s:
            time.sleep(self.slow_s)
        if self.down:
            h.send_response(503)
            h.end_headers()
            return
        code, body, ctype = 200, "", "text/plain"

        def refuse(why):
            return 409, "refused: " + why

        with self.lock:
            ph = self.phase
            busy = self.seq_t0 is not None
            if u.path == "/status":
                body, ctype = json.dumps(self.status_dict()), "application/json"
            elif u.path == "/key":
                c = q.get("c", [""])[0]
                ms = int(q.get("ms", ["350"])[0] or 0)
                if c in RUN_KEYS:
                    if ph == "FLIGHT":
                        code, body = refuse("in flight")
                    elif busy:
                        code, body = refuse("busy")
                    else:
                        self.keys.append((time.time(), c, max(50, min(500, ms))))
                        if self.seq in ("anchored", "released", "aborted"):
                            self.seq = "idle"
                elif c == "A":
                    self.keys.append((time.time(), c, ms))
                    self.armed = not self.armed
                    self.landed = False
                elif c in (" ", "s"):
                    self.keys.append((time.time(), " ", 0))
                    self._stop(True)
                else:
                    code, body = refuse("unknown key")
            elif u.path == "/stop":
                disarm = q.get("disarm", ["1"])[0] != "0"
                self.keys.append((time.time(), " ", 0))
                self._stop(disarm)
                body = "stopped"
            elif u.path == "/arm":
                on = q.get("on", ["1"])[0] != "0"
                auto = q.get("auto", ["1"])[0] != "0"
                if ph == "FLIGHT":
                    code, body = refuse("in flight")
                elif busy:
                    code, body = refuse("busy")
                elif on and ph == "ANCHORED":
                    code, body = refuse("anchored: release first")
                elif on and self.no_imu:
                    code, body = refuse("no IMU")
                else:
                    self.armed, self.auto_on_landing = on, auto
                    if on:
                        self.landed = False
                        self._flightlog = None
                        self.t_launch_ms = self.t_land_ms = None
                        self.fault = None
                    self.event = f"armed (auto {'on' if auto else 'off'})" if on else "disarmed"
                    body = self.event
            elif u.path == "/auto":
                if ph == "FLIGHT":
                    code, body = refuse("in flight")
                elif self.seq_kind == "release" and busy:
                    code, body = refuse("busy")
                elif ph == "ANCHORED":
                    code, body = refuse("already anchored, release first")
                elif self.seq_kind == "anchor" and busy:
                    body = "anchoring"          # repeat is accepted and does not restart
                else:
                    self.armed = False
                    self._start_seq("anchor")
                    body = "auto"
            elif u.path == "/release":
                if ph == "FLIGHT":
                    code, body = refuse("in flight")
                elif self.seq_kind == "anchor" and busy:
                    code, body = refuse("busy")
                elif self.seq_kind == "release" and busy:
                    body = "releasing"
                else:
                    self.armed = False
                    self._start_seq("release")
                    body = "release"
            elif u.path == "/flightlog" and (self.armed or ph == "FLIGHT"):
                code, body = refuse("armed" if self.armed and ph != "FLIGHT" else "in flight")
            elif u.path == "/flightlog" and busy:
                code, body = refuse("busy")
            elif u.path == "/flightlog":
                fl = self._flightlog or (self.flightlog_provider() if self.flightlog_provider else None)
                if fl is None:
                    fl = {"hz": 50, "frozen": False, "phase": ph, "t_launch_ms": None, "t_land_ms": None,
                          "landing": None, "n": 0, "fields": ["t_ms", "qw", "qx", "qy", "qz", "ax", "ay", "az"],
                          "samples": []}
                body, ctype = json.dumps(fl), "application/json"
            else:
                code, body = 404, "not found"
        data = body.encode()
        h.send_response(code)
        h.send_header("Content-Type", ctype)
        h.send_header("Content-Length", str(len(data)))
        h.end_headers()
        h.wfile.write(data)

    def _stop(self, disarm: bool) -> None:
        if self.seq_t0 is not None:
            self.seq = "aborted"
        self.seq_t0, self.seq_kind = None, None
        self.fault = None
        self.current_mA = 0
        if self.flying:
            self.flying = False
            self.landed = True
            self.t_land_ms = self.millis()
        if disarm:
            self.armed = False
        self.event = "stop"


def serve(port: int = 8780, **kw) -> MockDart:
    return MockDart(port=port, **kw).start()
