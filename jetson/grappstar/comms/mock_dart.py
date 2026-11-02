"""Simulated KEDGE dart HTTP API (contract C2) for tests and the --sim operator page.

  python3 -m grappstar mockdart --port 8780      # then: python3 -m grappstar dart --host 127.0.0.1:8780 status

Phases: IDLE -> ARMED (key A) -> FLIGHT (launch()) -> LANDED (land()) -> ANCHORING (auto, or by itself when armed)
-> ANCHORED -> RELEASING (/release) -> IDLE. FAULT when a sequence aborts.
Test hooks on the MockDart object: fail_anchor, fail_release, no_landing, down (refuse every request with 503),
slow_s (delay every reply), flightlog_provider.
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
RELEASE_SEQ = [(0.0, "release: drills out"), (1.5, "release: foot out"), (2.5, "release: kick back"),
               (3.5, "release: deploy up"), (4.5, "idle")]
G = 9.81


def synth_flightlog(t_launch_ms: int = 2000, flight_s: float = 1.1, pitch_deg: float = 35.0, hz: int = 50,
                    land_still_s: float = 3.0) -> dict:
    """A plausible C2 flightlog: 2 s on the rail, launch spike, ballistic flight, impact, still on the ground.
    Body frame: x forward (nose), y left, z up when level. Quaternion = world from body (w, x, y, z)."""
    samples = []
    dt = 1000.0 / hz
    t_land = t_launch_ms + int(flight_s * 1000)
    t_end = t_land + int(land_still_s * 1000)
    t = 0.0
    while t <= t_end:
        if t < t_launch_ms:
            pitch, a_body = pitch_deg, (G * math.sin(math.radians(pitch_deg)), 0.0, G * math.cos(math.radians(pitch_deg)))
        elif t < t_launch_ms + 60:
            pitch, a_body = pitch_deg, (6.0 * G, 0.0, G * math.cos(math.radians(pitch_deg)))
        elif t < t_land:
            f = (t - t_launch_ms) / (t_land - t_launch_ms)
            pitch = pitch_deg - f * (pitch_deg + 30.0)
            a_body = (-0.4, 0.0, 0.2)          # near free fall: drag only
        elif t < t_land + 40:
            pitch, a_body = -40.0, (-9.0 * G, 0.5 * G, 2.0 * G)
        else:
            pitch = -40.0
            a_body = (G * math.sin(math.radians(pitch)), 0.0, G * math.cos(math.radians(pitch)))
        h = math.radians(pitch) / 2.0          # rotation about body y; nose up = positive pitch
        qw, qy = math.cos(h), -math.sin(h)     # right-handed y axis points left, so nose up is negative about y
        samples.append([int(t), round(qw, 4), 0.0, round(qy, 4), 0.0,
                        round(a_body[0], 3), round(a_body[1], 3), round(a_body[2], 3)])
        t += dt
    return {"hz": hz, "t_launch_ms": t_launch_ms, "t_land_ms": t_land, "samples": samples}


class MockDart:
    def __init__(self, port: int = 0, host: str = "127.0.0.1", time_scale: float = 1.0):
        self.scale = time_scale
        self.lock = threading.RLock()
        self.t_boot = time.time()
        self.phase = "IDLE"
        self.seq = "idle"
        self.event = "boot"
        self.armed = False
        self.landed = False
        self.seq_t0: Optional[float] = None
        self.seq_kind: Optional[str] = None
        self.keys: list = []
        self.requests: list = []
        self.roll, self.pitch = 1.0, 35.0
        self.current_mA = 60
        # hooks
        self.fail_anchor = False
        self.fail_release = False
        self.no_landing = False
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

    # ------------------------------------------------ world events
    def launch(self) -> None:
        with self.lock:
            self.phase = "FLIGHT"
            self.event = "launch detected"
            self.landed = False
            self._t_launch_ms = self.millis()

    def land(self) -> None:
        with self.lock:
            if self.no_landing:
                return
            self.landed = True
            self.phase = "LANDED"
            self.event = "landed"
            self.pitch = -40.0
            t_l = getattr(self, "_t_launch_ms", max(0, self.millis() - 1100))
            self._flightlog = (self.flightlog_provider() if self.flightlog_provider else
                               synth_flightlog(t_launch_ms=t_l, flight_s=max(0.3, (self.millis() - t_l) / 1000.0)))
            if self.armed and self.seq_t0 is None:
                self._start_seq("anchor")

    def _start_seq(self, kind: str) -> None:
        self.seq_kind = kind
        self.seq_t0 = time.time()
        self.phase = "ANCHORING" if kind == "anchor" else "RELEASING"

    def _update(self) -> None:
        if self.seq_t0 is None:
            return
        el = (time.time() - self.seq_t0) * self.scale
        if self.seq_kind == "anchor":
            if self.fail_anchor and el >= 1.6:
                self.seq, self.event, self.phase = "aborted", "auto: foot stalled, no contact", "FAULT"
                self.seq_t0 = None
                self.current_mA = 0
                return
            for t, n in ANCHOR_SEQ:
                if el >= t:
                    self.seq, self.event = n, "auto: " + n
            self.current_mA = 420 if self.seq != "anchored" else 0
            if self.seq == "anchored":
                self.phase = "ANCHORED"
                self.armed = False
                self.seq_t0 = None
        else:
            if self.fail_release and el >= 2.0:
                self.seq, self.event, self.phase = "aborted", "release: kick stalled", "FAULT"
                self.seq_t0 = None
                return
            for t, n in RELEASE_SEQ:
                if el >= t:
                    self.seq, self.event = n, n
            if self.seq == "idle":
                self.phase = "IDLE"
                self.landed = False
                self.event = "released"
                self.seq_t0 = None

    def status_dict(self) -> dict:
        with self.lock:
            self._update()
            return dict(t_ms=self.millis(), rail_V=7.62, current_mA=self.current_mA, peak_mA=480, speed=255,
                        motors={}, seq=self.seq, armed=self.armed, landed=self.landed, roll=self.roll,
                        pitch=self.pitch, event=self.event, phase=self.phase)

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
        with self.lock:
            if u.path == "/status":
                body, ctype = json.dumps(self.status_dict()), "application/json"
            elif u.path == "/key":
                c = q.get("c", [""])[0]
                ms = int(q.get("ms", ["350"])[0] or 0)
                self.keys.append((time.time(), c, ms))
                if c == "A":
                    self.armed = not self.armed
                    self.landed = False
                    if self.phase in ("IDLE", "ARMED", "LANDED"):
                        self.phase = "ARMED" if self.armed else "IDLE"
                    self.event = "auto armed for landing" if self.armed else "auto disarmed"
                body = "ok"
            elif u.path == "/stop":
                self.keys.append((time.time(), " ", 0))
                self.event = "stop"
                if self.seq_t0 is not None:
                    self.seq, self.phase, self.seq_t0 = "aborted", "FAULT", None
                body = "stopped"
            elif u.path == "/auto":
                self._start_seq("anchor")
                body = "auto"
            elif u.path == "/release":
                self._start_seq("release")
                body = "release"
            elif u.path == "/flightlog":
                fl = self._flightlog or (self.flightlog_provider() if self.flightlog_provider else None)
                if fl is None:
                    code, body = 409, "no flight recorded"
                else:
                    body, ctype = json.dumps(fl), "application/json"
            else:
                code, body = 404, "not found"
        data = body.encode()
        h.send_response(code)
        h.send_header("Content-Type", ctype)
        h.send_header("Content-Length", str(len(data)))
        h.end_headers()
        h.wfile.write(data)


def serve(port: int = 8780, **kw) -> MockDart:
    return MockDart(port=port, **kw).start()
