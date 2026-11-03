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


def post(s, port, path, body=b"{}", headers=None):
    """POST like the page does: with the session token header (and a same-origin Origin)."""
    h = {"X-GrappStar-Token": s.session_token, "Origin": f"http://127.0.0.1:{port}"}
    h.update(headers or {})
    c = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    c.request("POST", path, body=body, headers=h)
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
    s, app, port = srv()
    r, data = post(s, port, "/api/action/target_click", json.dumps({"cam": "rover", "u": 0.5, "v": 0.25}),
                   {"Content-Type": "application/json"})
    assert r.status == 200
    j = json.loads(data)
    assert j["ok"] is True and j["echo"] == {"cam": "rover", "u": 0.5, "v": 0.25}
    assert app.calls[-1] == ("target_click", {"cam": "rover", "u": 0.5, "v": 0.25})
    r, data = post(s, port, "/api/action/stop", b"")       # empty body = no args
    assert r.status == 200 and app.calls[-1] == ("stop", {})


def test_action_bad_json(srv):
    s, app, port = srv()
    r, data = post(s, port, "/api/action/aim", b"{not json")
    assert r.status == 400 and json.loads(data)["ok"] is False
    r, data = post(s, port, "/api/action/aim", b"[1, 2]")
    assert r.status == 400
    assert app.calls == []


def test_action_exception(srv):
    s, _, port = srv()
    r, data = post(s, port, "/api/action/boom", b"{}")
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
    s, app, port = srv(token="s3cret")
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
    r, _ = post(s, port, "/api/action/stop?token=s3cret", b"{}")
    assert r.status == 200 and app.calls[-1][0] == "stop"
    # the shared password alone is not enough: the session token is still required on POST
    n = len(app.calls)
    r, _ = req(port, "POST", "/api/action/stop?token=s3cret", b"{}")
    assert r.status == 403 and len(app.calls) == n
    # and the session token alone does not get past the shared password
    r, _ = post(s, port, "/api/action/stop", b"{}")
    assert r.status == 401 and len(app.calls) == n


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


# ---- action API protection (review r2_H #8) ------------------------------------------------------------------
def test_session_token_injected_into_page(srv):
    s, _, port = srv()
    tok = s.session_token
    assert len(tok) >= 20
    r, data = req(port, "GET", "/")
    page = data.decode()
    assert r.status == 200
    assert f'<meta name="grappstar-token" content="{tok}">' in page
    assert page.count("grappstar-token") == 1
    # a second server (another process start) gets another token
    s2, _, port2 = srv()
    assert s2.session_token != tok
    assert s2.session_token in req(port2, "GET", "/index.html")[1].decode()


def test_post_requires_session_token(srv):
    s, app, port = srv()
    for headers in ({}, {"X-GrappStar-Token": ""}, {"X-GrappStar-Token": "wrong"},
                    {"X-GrappStar-Token": s.session_token + "x"}):
        r, data = req(port, "POST", "/api/action/answer",
                      json.dumps({"prompt_id": "p1", "choice": "FIRE NOW", "text": "x"}), headers)
        j = json.loads(data)
        assert r.status == 403 and j["ok"] is False and j["reload"] is True, headers
        assert r.getheader("Content-Type") == "application/json"
    assert app.calls == []
    r, data = req(port, "POST", "/api/action/stop", b"{}", {"X-GrappStar-Token": s.session_token})
    assert r.status == 200 and app.calls == [("stop", {})]      # no Origin/Referer (curl): token is enough


@pytest.mark.parametrize("hdr", [
    {"Origin": "http://evil.example"},
    {"Origin": "http://evil.example:{port}"},
    {"Origin": "http://127.0.0.1:1"},
    {"Origin": "null"},
    {"Origin": "file://"},
    {"Referer": "http://evil.example/page.html"},
    {"Referer": "http://127.0.0.1:1/"},
])
def test_post_wrong_origin_rejected(srv, hdr):
    s, app, port = srv()
    hdr = {k: v.format(port=port) for k, v in hdr.items()}
    r, data = req(port, "POST", "/api/action/stop", b"{}", dict(hdr, **{"X-GrappStar-Token": s.session_token}))
    assert r.status == 403 and json.loads(data)["ok"] is False
    assert "origin" in json.loads(data)["msg"]
    assert app.calls == []


def test_post_right_origin_and_token_accepted(srv):
    s, app, port = srv()
    tok = {"X-GrappStar-Token": s.session_token}
    for hdr in ({"Origin": f"http://127.0.0.1:{port}"}, {"Origin": f"http://LOCALHOST:{port}", "Host": f"localhost:{port}"},
                {"Referer": f"http://127.0.0.1:{port}/"},
                # a phone on another address: Host is whatever it typed, and Origin matches it
                {"Host": f"192.168.4.1:{port}", "Origin": f"http://192.168.4.1:{port}"},
                {"Host": "rover.local", "Origin": "http://rover.local"},
                {"Host": "rover.local:80", "Origin": "http://rover.local"},
                {"Host": f"[fe80::1]:{port}", "Origin": f"http://[fe80::1]:{port}"}):
        r, data = req(port, "POST", "/api/action/answer",
                      json.dumps({"prompt_id": "p7", "choice": "Fire", "text": "FIRE at 30 deg?"}), dict(hdr, **tok))
        assert r.status == 200, hdr
        assert json.loads(data)["echo"] == {"prompt_id": "p7", "choice": "Fire", "text": "FIRE at 30 deg?"}
    assert len(app.calls) == 7
    r, _ = req(port, "POST", "/api/action/stop", b"{}",
               dict(tok, Host=f"192.168.4.1:{port}", Origin=f"http://127.0.0.1:{port}"))
    assert r.status == 403 and len(app.calls) == 7         # Origin must match the Host the browser used


def test_get_stays_open_without_session_token(srv):
    _, app, port = srv()
    for path in ("/api/status", "/snapshot/rover.jpg", "/artifact/landing.png", "/api/log"):
        r, _ = req(port, "GET", path, headers={"Origin": "http://evil.example"})
        assert r.status == 200, path
    r, _ = req(port, "GET", "/api/action/stop")
    assert r.status == 404 and app.calls == []                # GET never runs an action


def test_post_refused_for_foreign_host_name(tmp_path):
    """DNS rebinding: a POST whose Host is some other DNS name is refused even with the token and a matching Origin."""
    import http.client
    app = FakeApp(tmp_path)
    srv = ui_server.make_server(app, host="127.0.0.1", port=0)
    import threading
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        port = srv.server_address[1]
        c = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
        c.request("POST", "/api/action/stop", body="{}", headers={
            "Host": f"evil.example:{port}", "Origin": f"http://evil.example:{port}",
            "X-GrappStar-Token": srv.session_token, "Content-Type": "application/json"})
        r = c.getresponse()
        assert r.status == 403
        c.close()
        c = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
        c.request("POST", "/api/action/stop", body="{}", headers={
            "Host": f"127.0.0.1:{port}", "Origin": f"http://127.0.0.1:{port}",
            "X-GrappStar-Token": srv.session_token, "Content-Type": "application/json"})
        assert c.getresponse().status == 200
    finally:
        srv.shutdown()
