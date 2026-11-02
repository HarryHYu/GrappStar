"""Tests for the operator web server (grappstar.ui.server) against a FakeApp (no real App needed)."""

import http.client
import json
import os
import re
import socket
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from grappstar.ui import server as ui_server

UI_DIR = os.path.dirname(ui_server.__file__)


class FakeCamera:
    """Produces a new JPEG frame every 1/30 s; latest() gives the frame identity like video.camera.Camera."""

    def __init__(self, empty=False):
        self.empty = empty
        self.t0 = time.time()

    def _idx(self):
        return int((time.time() - self.t0) * 30)

    def latest(self):
        if self.empty:
            return None
        i = self._idx()
        return SimpleNamespace(idx=i, t=self.t0 + i / 30.0)

    def jpeg(self):
        if self.empty:
            return None
        img = np.zeros((120, 160, 3), np.uint8)
        cv2.putText(img, str(self._idx()), (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)
        ok, buf = cv2.imencode(".jpg", img)
        assert ok
        return buf.tobytes()


class FakeApp:
    def __init__(self, tmpdir, token=""):
        self.calls = []
        self.ui_token = token
        self.cams = {"rover": FakeCamera(), "dart": FakeCamera(empty=True)}
        self.files = {}
        png = os.path.join(tmpdir, "landing.png")
        ok, buf = cv2.imencode(".png", np.zeros((10, 10, 3), np.uint8))
        with open(png, "wb") as f:
            f.write(buf.tobytes())
        self.files["landing.png"] = png
        log = os.path.join(tmpdir, "mission.jsonl")
        with open(log, "w") as f:
            f.write('{"t": 1.0, "ev": "start"}\n')
        self.files["mission.jsonl"] = log

    def status(self):
        return {"base": {"link": True, "state": "IDLE", "tension_n": float("nan")},
                "dart": {"link": False}, "mission": {"state": "IDLE", "step": "", "history": [], "prompt": None},
                "cams": {"rover": {"connected": True}}, "results": {"terrain": None, "mosaic": None},
                "target": None, "t": time.time(), "arr": np.arange(3)}

    def action(self, name, args):
        self.calls.append((name, args))
        if name == "boom":
            raise RuntimeError("kaboom")
        return {"ok": True, "msg": f"did {name}", "echo": args}

    def camera(self, name):
        return self.cams.get(name)

    def artifact(self, name):
        return self.files.get(name)


@pytest.fixture
def srv(tmp_path):
    servers = []

    def start(token=""):
        app = FakeApp(str(tmp_path), token)
        s = ui_server.make_server(app, host="127.0.0.1", port=0, max_fps=20)
        th = threading.Thread(target=s.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        th.start()
        servers.append(s)
        return s, app, s.server_address[1]

    yield start
    for s in servers:
        s.shutdown()
        s.server_close()


def req(port, method, path, body=None, headers=None):
    c = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    c.request(method, path, body=body, headers=headers or {})
    r = c.getresponse()
    data = r.read()
    c.close()
    return r, data


def test_index_and_static(srv):
    _, _, port = srv()
    r, data = req(port, "GET", "/")
    assert r.status == 200 and b"<html" in data and r.getheader("Content-Type").startswith("text/html")
    for name, ctype in (("app.js", "text/javascript"), ("style.css", "text/css")):
        r, data = req(port, "GET", "/static/" + name)
        assert r.status == 200 and r.getheader("Content-Type").startswith(ctype) and len(data) > 100


@pytest.mark.parametrize("path", ["/static/../server.py", "/static/..%2fserver.py", "/static/%2e%2e/server.py",
                                  "/static/../../__init__.py", "/static/..\\server.py", "/static/",
                                  "/static//etc/passwd", "/static/nope.js"])
def test_static_traversal_blocked(srv, path):
    _, _, port = srv()
    r, data = req(port, "GET", path)
    assert r.status == 404
    assert b"import" not in data


def test_status_json(srv):
    _, _, port = srv()
    r, data = req(port, "GET", "/api/status")
    assert r.status == 200 and r.getheader("Content-Type") == "application/json"
    st = json.loads(data)
    assert st["base"]["state"] == "IDLE"
    assert st["base"]["tension_n"] is None      # NaN becomes null (strict JSON)
    assert st["arr"] == [0, 1, 2]


def test_action_routes_args(srv):
    _, app, port = srv()
    r, data = req(port, "POST", "/api/action/target_click", json.dumps({"cam": "rover", "u": 0.5, "v": 0.25}),
                  {"Content-Type": "application/json"})
    assert r.status == 200
    j = json.loads(data)
    assert j["ok"] is True and j["echo"] == {"cam": "rover", "u": 0.5, "v": 0.25}
    assert app.calls[-1] == ("target_click", {"cam": "rover", "u": 0.5, "v": 0.25})
    r, data = req(port, "POST", "/api/action/stop", b"")       # empty body = no args
    assert r.status == 200 and app.calls[-1] == ("stop", {})


def test_action_bad_json(srv):
    _, app, port = srv()
    r, data = req(port, "POST", "/api/action/aim", b"{not json")
    assert r.status == 400 and json.loads(data)["ok"] is False
    r, data = req(port, "POST", "/api/action/aim", b"[1, 2]")
    assert r.status == 400
    assert app.calls == []


def test_action_exception(srv):
    _, _, port = srv()
    r, data = req(port, "POST", "/api/action/boom", b"{}")
    j = json.loads(data)
    assert j["ok"] is False and "kaboom" in j["msg"]


def _read_mjpeg_parts(port, path, n, timeout=5.0):
    s = socket.create_connection(("127.0.0.1", port), timeout=timeout)
    s.sendall(f"GET {path} HTTP/1.1\r\nHost: x\r\n\r\n".encode())
    buf = b""
    t_end = time.time() + timeout
    while time.time() < t_end:
        chunk = s.recv(65536)
        if not chunk:
            break
        buf += chunk
        if len(re.findall(rb"\xff\xd8.*?\xff\xd9\r\n--", buf, re.S)) >= n:
            break
    s.close()          # abrupt client disconnect: the server must cope quietly
    head, _, body = buf.partition(b"\r\n\r\n")
    return head, body


def test_mjpeg_stream(srv):
    _, _, port = srv()
    head, body = _read_mjpeg_parts(port, "/stream/rover.mjpg", 2)
    assert b"200" in head.split(b"\r\n")[0]
    assert b"multipart/x-mixed-replace" in head
    boundary = re.search(rb"boundary=(\S+)", head).group(1)
    parts = [p for p in body.split(b"--" + boundary) if b"image/jpeg" in p]
    jpegs = [p.split(b"\r\n\r\n", 1)[1] for p in parts if b"\xff\xd9" in p]
    assert len(jpegs) >= 2
    img = cv2.imdecode(np.frombuffer(jpegs[0].rstrip(b"\r\n"), np.uint8), cv2.IMREAD_COLOR)
    assert img is not None and img.shape[:2] == (120, 160)


def test_mjpeg_placeholder_and_unknown_camera(srv):
    _, _, port = srv()
    head, body = _read_mjpeg_parts(port, "/stream/dart.mjpg", 1)
    jpg = body.split(b"\r\n\r\n", 1)[1]
    img = cv2.imdecode(np.frombuffer(jpg[: jpg.index(b"\xff\xd9") + 2], np.uint8), cv2.IMREAD_COLOR)
    assert img is not None and img.shape[:2] == (360, 640)      # NO VIDEO placeholder
    r, _ = req(port, "GET", "/stream/nope.mjpg")
    assert r.status == 404


def test_snapshot(srv):
    _, _, port = srv()
    r, data = req(port, "GET", "/snapshot/rover.jpg")
    assert r.status == 200 and r.getheader("Content-Type") == "image/jpeg" and data[:2] == b"\xff\xd8"
    r, data = req(port, "GET", "/snapshot/dart.jpg")          # no frame: placeholder
    assert r.status == 200 and data[:2] == b"\xff\xd8"
    r, _ = req(port, "GET", "/snapshot/nope.jpg")
    assert r.status == 404


def test_artifacts_and_log(srv):
    _, _, port = srv()
    r, data = req(port, "GET", "/artifact/landing.png")
    assert r.status == 200 and r.getheader("Content-Type") == "image/png" and data[:4] == b"\x89PNG"
    for bad in ("/artifact/mosaic.png", "/artifact/../server.py", "/artifact/..%2fserver.py", "/artifact/"):
        r, _ = req(port, "GET", bad)
        assert r.status == 404, bad
    r, data = req(port, "GET", "/api/log")
    assert r.status == 200 and b"start" in data
    assert "attachment" in r.getheader("Content-Disposition")
    assert "mission.jsonl" in r.getheader("Content-Disposition")


def test_token_protection(srv):
    _, app, port = srv(token="s3cret")
    r, _ = req(port, "GET", "/api/status")
    assert r.status == 401
    r, data = req(port, "GET", "/")
    assert r.status == 401 and b"token" in data
    r, _ = req(port, "POST", "/api/action/stop", b"{}")
    assert r.status == 401 and app.calls == []
    r, _ = req(port, "GET", "/api/status?token=wrong")
    assert r.status == 401
    r, _ = req(port, "GET", "/api/status", headers={"Cookie": "grappstar_token=wrong"})
    assert r.status == 401
    r, _ = req(port, "GET", "/?token=s3cret")
    assert r.status == 200
    cookie = r.getheader("Set-Cookie")
    assert cookie and "grappstar_token=s3cret" in cookie
    r, _ = req(port, "GET", "/api/status", headers={"Cookie": "other=1; grappstar_token=s3cret"})
    assert r.status == 200
    r, _ = req(port, "POST", "/api/action/stop?token=s3cret", b"{}")
    assert r.status == 200 and app.calls[-1][0] == "stop"


def test_no_token_when_empty(srv):
    _, _, port = srv(token="")
    r, _ = req(port, "GET", "/api/status")
    assert r.status == 200


def test_no_em_or_en_dashes_in_ui():
    files = []
    for root, _, names in os.walk(UI_DIR):
        files += [os.path.join(root, n) for n in names if n.endswith((".py", ".html", ".js", ".css"))]
    files.append(os.path.abspath(__file__))
    assert any(f.endswith("app.js") for f in files)
    for path in files:
        with open(path, encoding="utf-8") as f:
            text = f.read()
        assert chr(0x2014) not in text, path
        assert chr(0x2013) not in text, path
