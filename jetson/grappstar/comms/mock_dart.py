"""Simulated KEDGE dart HTTP API, faithful to firmware kedge-dart-v4.3 for the v34 dart (firmware/dart/API.md, contract C2).

  python3 -m grappstar mockdart --port 8780      # then: python3 -m grappstar dart --host 127.0.0.1:8780 status

Phases (priority as in API.md): FAULT > RELEASING > ANCHORING > FLIGHT > ANCHORED > LANDED > ARMED > IDLE.
World events from the simulator or tests: launch() (only counted while armed, like the real detector), land().
v4.3 (v34 dart): no foot, no kick. The kick channel drives the righting leg (motors.kick), motors.foot is always 0 and keys
4 / $ are refused "no foot on v34". /auto rights the dart first when orient is not upright (belly up: leg flip; on a flank,
leaning or nose steep: the pods roll it, drills off; seq "righting: ..."), then pods down with the drills, drills finishing,
anchored. /auto?force=1 skips the righting. No "not upright" refusals.
Refusals are HTTP 409 with "refused: <reason>". Test hooks: fail_anchor (NO_CURRENT in the pods step), cannot_right (the
righting gives up: fault NOT_UPRIGHT after its 6 moves, every retry too unless forced), warn_pods_limit (anchored with warning
pods_time_limit), fail_release, no_landing, no_imu, down (answer 503 to everything), slow_s (delay every reply),
flightlog_provider.
"""
from __future__ import annotations

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Optional
from urllib.parse import parse_qs, urlparse

# firmware v4.3 seq names and motors per step (deploy, kick = the righting leg, drills; foot is always 0)
ANCHOR_SEQ = [(0.0, "deploy + drills"), (2.0, "drills finishing"), (3.4, "anchored")]          # belly down: no righting
RIGHT_LEG = [(0.0, "righting: leg out"), (1.0, "righting: leg folding"), (1.8, "righting: settling"),   # belly up: leg flip,
             (2.0, "righting: pods out, drills off"), (2.6, "righting: settling")]                       # lands on a flank, pods roll
RIGHT_PODS = [(0.0, "righting: pods out, drills off"), (0.6, "righting: settling")]           # on a flank / leaning: pods roll
RELEASE_SEQ = [(0.0, "release: drills out"), (2.5, "release: drills out + pods up"), (4.5, "released")]
SEQ_MOTORS = {"righting: leg out": (0, 1, 0), "righting: leg folding": (0, -1, 0), "righting: pods out, drills off": (1, 0, 0),
              "deploy + drills": (1, 0, 1), "drills finishing": (0, 0, 1), "release: drills out": (0, 0, -1),
              "release: drills out + pods up": (-1, 0, -1)}
RUN_KEYS = set("1!2@3#")
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
        self.cannot_right = False
        self.warn_pods_limit = False
        self.warnings: list = []
        self.anchor_plan: list = ANCHOR_SEQ
        self.righting = False
        self.seq_log: list = []      # every seq the mock went through (tests: the client polls too slowly to see them all)
        self.fail_release = False
        self.no_landing = False
        self.no_imu = False
        self.down = False
        self.slow_s = 0.0
        self.gap_until = 0.0         # WiFi gap: until this time.time() requests get no answer at all
        self.freeze_s = 3.0          # API v4: the flight log freezes 3 s (dart time) after the landing
        self.t_land_wall: Optional[float] = None
        self.rail_V = 7.62
        self.ina_ok = True
        self.orient = "upright"      # API v4.1: upright, leaning, side, inverted, nose_down, nose_up, unknown (v4.3: righted)
        self.batt = "ok"             # API v4.1: ok, low, critical, none, unknown
        self.moving = False          # API v4.1: the anchor moving under the tow (only while ANCHORED)
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
            self.t_land_wall = time.time()
            self.pitch = -40.0
            self.event = "landed (impact)"
            t_l = self.t_launch_ms or max(0, self.t_land_ms - 1100)
            self._flightlog = (self.flightlog_provider() if self.flightlog_provider else
                               synth_flightlog(t_launch_ms=t_l, flight_s=max(0.3, (self.t_land_ms - t_l) / 1000.0)))
            if self.auto_on_landing:
                self._start_seq("anchor")         # v4.3: on its side or back too (the auto rights it first)

    def _start_seq(self, kind: str, force: bool = False) -> None:
        self.seq_kind = kind
        self.seq_t0 = time.time()
        self.fault = None
        self.peak_mA = 0
        if kind == "anchor":
            self.warnings = []
            # v4.3: tilt 15 deg or more is righted first (belly up: leg flip then pods; a flank: pods); force skips it
            pre = [] if force or self.orient in ("upright", "unknown") else (RIGHT_LEG if self.orient == "inverted" else RIGHT_PODS)
            off = (pre[-1][0] + 0.4) if pre else 0.0
            self.anchor_plan = pre + [(t + off, n) for t, n in ANCHOR_SEQ]
            self.righting = bool(pre)
            if pre and self.cannot_right:
                self.anchor_plan = pre                 # it never gets upright: NOT_UPRIGHT after the moves

    def _update(self) -> None:
        if self.seq_t0 is None:
            return
        el = (time.time() - self.seq_t0) * self.scale
        if self.seq_kind == "anchor":
            plan = self.anchor_plan
            if self.cannot_right and self.righting and el >= plan[-1][0] + 0.6:
                self.seq, self.fault = "aborted", "NOT_UPRIGHT"
                self.event = "auto: not upright after 6 righting moves (tilt %d deg)" % (175 if self.orient == "inverted" else 40)
                self.seq_t0, self.seq_kind, self.current_mA = None, None, 0
                return
            deploy_t = next((t for t, n in plan if n == "deploy + drills"), None)
            if self.fail_anchor and deploy_t is not None and el >= deploy_t + 0.4:
                self.seq, self.event, self.fault = "aborted", "deploy draws no current: wire open or shorted", "NO_CURRENT"
                self.seq_t0, self.seq_kind, self.current_mA = None, None, 0
                return
            for t, n in plan:
                if el >= t:
                    if n != self.seq:
                        self.event = ("auto: " + n) if not n.startswith("righting") else n
                        self.seq_log.append(n)
                    self.seq = n
            if deploy_t is not None and el >= deploy_t and self.orient != "unknown":
                self.orient = "upright"                # righted (or it was already)
            self.current_mA = (420 if self.seq in ("deploy + drills", "drills finishing") else 60) if self.seq != "anchored" else 0
            self.peak_mA = max(self.peak_mA, self.current_mA)
            if self.seq == "anchored":
                self.seq_t0, self.seq_kind = None, None
                self.landed = False
                if self.warn_pods_limit:
                    self.warnings = ["pods_time_limit"]
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
            mot = SEQ_MOTORS.get(self.seq, (0, 0, 0)) if self.seq_t0 is not None else (0, 0, 0)
            tilt = {"upright": 0, "leaning": 38, "side": 90, "inverted": 175, "nose_down": 30, "nose_up": 20}.get(self.orient, 0)
            return {"fw": "kedge-dart-v4.3-mock", "t_ms": self.millis(), "phase": ph, "fault": self.fault,
                    "rail_V": self.rail_V, "current_mA": self.current_mA, "peak_mA": self.peak_mA, "speed": 255,
                    "motors": {"deploy": mot[0], "kick": mot[1], "drills": mot[2], "foot": 0}, "seq": self.seq,
                    "armed": self.armed, "auto_on_landing": self.auto_on_landing, "landed": self.landed,
                    "roll": self.roll, "pitch": self.pitch, "accel_g": 1.0, "peak_g": 8.0 if self.t_land_ms else 1.0, "ina_ok": self.ina_ok,
                    "imu_ok": not self.no_imu, "t_launch_ms": self.t_launch_ms, "t_land_ms": self.t_land_ms,
                    "log_frozen": self.log_frozen(), "reset": "poweron", "event": self.event,
                    "batt": self.batt, "batt_V": self.rail_V, "orient": self.orient, "tilt_deg": tilt,
                    "warnings": list(self.warnings), "moving": bool(self.moving and ph == "ANCHORED"), "video": True}

    def log_frozen(self) -> bool:
        if self._flightlog is None:
            return False
        if self.t_land_wall is None:
            return True
        return time.time() - self.t_land_wall >= self.freeze_s / self.scale

    # ------------------------------------------------ http
    def _handle(self, h: BaseHTTPRequestHandler) -> None:
        u = urlparse(h.path)
        q = parse_qs(u.query)
        self.requests.append((time.time(), u.path, u.query))
        if time.time() < self.gap_until:
            # WiFi gap: the request never gets an answer (the client times out), like a dropped link
            time.sleep(min(1.5, max(0.0, self.gap_until - time.time())))
            h.close_connection = True
            return
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
                if c in ("4", "$"):
                    code, body = refuse("no foot on v34")
                elif c in RUN_KEYS:
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
                force = q.get("force", ["0"])[0] == "1"
                if ph == "FLIGHT":
                    code, body = refuse("in flight")
                elif self.batt == "critical":
                    code, body = refuse("battery low: charge it (release still works)")
                elif self.seq_kind == "release" and busy:
                    code, body = refuse("busy")
                elif ph == "ANCHORED":
                    code, body = refuse("already anchored, release first")
                elif self.seq_kind == "anchor" and busy:
                    body = "anchoring"          # repeat is accepted and does not restart
                else:
                    self.armed = False
                    self._start_seq("anchor", force=force)
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
                if fl is not None and self._flightlog is not None:
                    fl = dict(fl, frozen=self.log_frozen())
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
