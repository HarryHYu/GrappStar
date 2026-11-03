"""Operator web page server for the GrappStar rover.

Why the standard library (http.server.ThreadingHTTPServer) and not Flask, aiohttp or similar:
the rover runs on a Jetson Orin Nano with JetPack, where every extra pip package is one more thing to
install, pin and keep working offline in the field. The page is used by one or two operators over the
rover WiFi, so a thread per connection is plenty (an MJPEG stream holds one thread per viewer). Nothing
here needs more than the stdlib; OpenCV (already required by the package) is imported lazily only to
draw the "NO VIDEO" placeholder.

The UI talks ONLY to an App object (see INTERFACES.md, section "app"):
    app.status() -> dict, app.action(name, args) -> dict, app.camera(name) -> Camera or None,
    app.artifact(name) -> path or None.

Routes:
    GET  /                      index.html
    GET  /static/<file>         files from ui/static (flat names only, no path traversal)
    GET  /api/status            JSON of app.status()
    POST /api/action/<name>     JSON object body = args; reply JSON from app.action (400 on bad JSON,
                                500 with {"ok": false, "msg": ...} when the App raises; 403 without the
                                session token header or from another origin, see below)
    GET  /stream/<cam>.mjpg     multipart/x-mixed-replace MJPEG of app.camera(cam).jpeg()
    GET  /snapshot/<cam>.jpg    one JPEG (placeholder "NO VIDEO" image when the camera has no frame)
    GET  /artifact/<name>       file at app.artifact(name), with its content type (404 when None)
    GET  /api/log               mission.jsonl as a download (Content-Disposition: attachment)

Optional shared token: if the App has a non-empty attribute `ui_token`, every request must carry it,
either as the cookie `grappstar_token` or as the query parameter `?token=...`. Opening
http://<rover>:8080/?token=SECRET once sets the cookie, so the page, its API calls and the video
streams then work without the parameter. Without the token the server answers 401 (a small form to
enter it for "/"). The token only keeps casual WiFi users out; it is not encryption (plain HTTP).

Action API protection (review r2_H #8), always on, on top of the optional ui_token:
  * Session token: the server makes a random token at start (secrets.token_urlsafe) and writes it into
    index.html when serving it (<meta name="grappstar-token" content="...">). Every POST must carry it in the
    header X-GrappStar-Token, else 403. Another web site cannot read the page, so it cannot learn the token,
    and a custom header cannot be sent cross site without a CORS preflight, which this server never allows.
    After a server restart an old page holds a stale token: the 403 says "reload": true and the page reloads.
  * Same origin: a POST whose Origin (or Referer when there is no Origin) host:port differs from the Host
    header is refused with 403. A phone on another IP works: its Host header is whatever address it used.
GET requests never change state and stay open (status, video, snapshots, artifacts).
"""

from __future__ import annotations

import hmac
import json
import logging
import math
import mimetypes
import os
import re
import secrets
import threading
import time
from http import HTTPStatus
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional
from urllib.parse import parse_qs, quote, unquote, urlsplit

log = logging.getLogger("grappstar.ui")

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
COOKIE_NAME = "grappstar_token"
BOUNDARY = "grappstarframe"
MAX_BODY = 64 * 1024
DEFAULT_MAX_FPS = 12.0
KEEPALIVE_S = 2.0           # resend the last frame this often when nothing new arrives (detects dead clients)
TOKEN_HEADER = "X-GrappStar-Token"
TOKEN_META_RE = re.compile(rb'<meta name="grappstar-token" content="[^"]*">')
NAME_RE = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_.\-]{0,63}$")

CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json",
    ".jsonl": "application/x-ndjson",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
    ".txt": "text/plain; charset=utf-8",
    ".csv": "text/csv; charset=utf-8",
    ".mp4": "video/mp4",
    ".avi": "video/x-msvideo",
    ".mkv": "video/x-matroska",
}


def content_type(path: str) -> str:
    ext = os.path.splitext(path)[1].lower()
    if ext in CONTENT_TYPES:
        return CONTENT_TYPES[ext]
    guess, _ = mimetypes.guess_type(path)
    return guess or "application/octet-stream"


def _clean(obj: Any) -> Any:
    """Make a status/action reply strict-JSON safe (NaN/inf -> null, numpy scalars and arrays -> Python)."""
    if obj is None or isinstance(obj, (bool, int, str)):
        return obj
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {str(k): _clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple, set)):
        return [_clean(v) for v in obj]
    if hasattr(obj, "tolist"):          # numpy array or scalar
        return _clean(obj.tolist())
    if hasattr(obj, "to_dict"):
        try:
            return _clean(obj.to_dict())
        except Exception:
            pass
    if isinstance(obj, bytes):
        return obj.decode("utf-8", "replace")
    return str(obj)


def to_json(obj: Any) -> bytes:
    return json.dumps(_clean(obj), allow_nan=False, separators=(",", ":")).encode("utf-8")


_placeholder_lock = threading.Lock()
_placeholder: Optional[bytes] = None


def placeholder_jpeg() -> bytes:
    """A 640x360 dark gray JPEG reading NO VIDEO (cached)."""
    global _placeholder
    with _placeholder_lock:
        if _placeholder is None:
            try:
                import cv2
                import numpy as np
                img = np.full((360, 640, 3), 40, np.uint8)
                font = cv2.FONT_HERSHEY_SIMPLEX
                (tw, th), _ = cv2.getTextSize("NO VIDEO", font, 2.0, 4)
                cv2.putText(img, "NO VIDEO", ((640 - tw) // 2, (360 + th) // 2), font, 2.0, (255, 255, 255), 4,
                            cv2.LINE_AA)
                ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                _placeholder = buf.tobytes() if ok else b""
            except Exception:       # no OpenCV: the browser just shows a broken image
                log.warning("cannot draw the NO VIDEO placeholder", exc_info=True)
                _placeholder = b""
        return _placeholder


class UIServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr, app, max_fps: float = DEFAULT_MAX_FPS):
        self.app = app
        self.max_fps = max(0.5, float(max_fps))
        self.stopping = threading.Event()
        self.session_token = secrets.token_urlsafe(24)     # per process: the page must echo it on every POST
        super().__init__(addr, Handler)

    def handle_error(self, request, client_address):
        # Client went away (phone locked, tab closed): not worth a stack trace.
        import sys
        exc = sys.exc_info()[1]
        if isinstance(exc, (ConnectionError, TimeoutError)):
            return
        log.exception("UI request from %s failed", client_address)

    def shutdown(self):
        self.stopping.set()
        super().shutdown()

    def server_close(self):
        self.stopping.set()
        super().server_close()

    @property
    def token(self) -> str:
        tok = getattr(self.app, "ui_token", None)
        return str(tok) if tok else ""


class Handler(BaseHTTPRequestHandler):
    server: UIServer
    protocol_version = "HTTP/1.1"
    server_version = "GrappStarUI/1.0"

    # ---- plumbing -------------------------------------------------------------------------------
    def log_message(self, fmt, *args):     # quiet: no per-request lines
        pass

    def log_error(self, fmt, *args):
        log.debug("UI: " + fmt, *args)

    @property
    def app(self):
        return self.server.app

    def _send(self, code: int, body: bytes, ctype: str, headers: Optional[dict] = None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (headers or {}).items():
            self.send_header(k, v)
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _json(self, code: int, obj: Any, headers: Optional[dict] = None):
        self._send(code, to_json(obj), "application/json", headers)

    def _text(self, code: int, text: str):
        self._send(code, text.encode("utf-8"), "text/plain; charset=utf-8")

    def _file(self, path: str, ctype: Optional[str] = None, headers: Optional[dict] = None):
        try:
            with open(path, "rb") as f:
                body = f.read()
        except OSError:
            return self._text(404, "not found")
        self._send(200, body, ctype or content_type(path), headers)

    def _split(self):
        parts = urlsplit(self.path)
        return unquote(parts.path), parse_qs(parts.query)

    # ---- token ----------------------------------------------------------------------------------
    def _check_token(self, query) -> Optional[dict]:
        """None = refused (reply already sent); else extra headers to send (cookie when given by query)."""
        token = self.server.token
        if not token:
            return {}
        given = (query.get("token") or [""])[0]
        if given and hmac.compare_digest(given.encode(), token.encode()):
            return {"Set-Cookie": f"{COOKIE_NAME}={quote(token)}; Path=/; SameSite=Strict; HttpOnly"}
        cookie = SimpleCookie()
        try:
            cookie.load(self.headers.get("Cookie", ""))
        except Exception:
            pass
        if COOKIE_NAME in cookie:
            if hmac.compare_digest(unquote(cookie[COOKIE_NAME].value).encode(), token.encode()):
                return {}
        path = urlsplit(self.path).path
        if path in ("/", "/index.html"):
            page = ("<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
                    "<title>GrappStar</title><body style='font:20px sans-serif;margin:24px'>"
                    "<h1>GrappStar</h1><form method=get><label>Token <input name=token autofocus "
                    "style='font-size:20px;padding:8px'></label> <button style='font-size:20px;padding:8px 16px'>"
                    "Open</button></form>")
            self._send(401, page.encode(), "text/html; charset=utf-8")
        else:
            self._json(401, {"ok": False, "msg": "token required"})
        return None

    # ---- GET ------------------------------------------------------------------------------------
    def do_HEAD(self):
        self.do_GET()

    def do_GET(self):
        path, query = self._split()
        extra = self._check_token(query)
        if extra is None:
            return
        try:
            if path in ("/", "/index.html"):
                return self._index(extra)
            if path.startswith("/static/"):
                return self._static(path[len("/static/"):])
            if path == "/api/status":
                return self._status()
            if path == "/api/log":
                return self._artifact("mission.jsonl", attachment=True)
            if path.startswith("/artifact/"):
                return self._artifact(path[len("/artifact/"):])
            m = re.fullmatch(r"/stream/([A-Za-z0-9_\-]+)\.mjpg", path)
            if m:
                return self._stream(m.group(1))
            m = re.fullmatch(r"/snapshot/([A-Za-z0-9_\-]+)\.jpg", path)
            if m:
                return self._snapshot(m.group(1))
            if path == "/favicon.ico":
                return self._send(204, b"", "image/x-icon")
            return self._text(404, "not found")
        except (ConnectionError, TimeoutError):
            self.close_connection = True

    def _index(self, headers: dict):
        """index.html with this process's session token written into its grappstar-token meta tag."""
        try:
            with open(os.path.join(STATIC_DIR, "index.html"), "rb") as f:
                body = f.read()
        except OSError:
            return self._text(404, "not found")
        meta = f'<meta name="grappstar-token" content="{self.server.session_token}">'.encode("ascii")
        body, n = TOKEN_META_RE.subn(lambda _m: meta, body, count=1)
        if not n:
            body = body.replace(b"</head>", meta + b"\n</head>", 1)
        self._send(200, body, "text/html; charset=utf-8", headers)

    def _static(self, name: str):
        # Flat names only: rejects "..", "/", "\\" and anything else odd, then double-checks the real path.
        if not NAME_RE.match(name) or ".." in name:
            return self._text(404, "not found")
        full = os.path.realpath(os.path.join(STATIC_DIR, name))
        if os.path.dirname(full) != os.path.realpath(STATIC_DIR) or not os.path.isfile(full):
            return self._text(404, "not found")
        return self._file(full)

    def _status(self):
        try:
            st = self.app.status()
        except Exception as e:
            log.exception("app.status() failed")
            return self._json(500, {"ok": False, "msg": f"status failed: {e}"})
        return self._json(200, st)

    def _artifact(self, name: str, attachment: bool = False):
        if not NAME_RE.match(name) or ".." in name:
            return self._text(404, "not found")
        try:
            p = self.app.artifact(name)
        except Exception:
            log.exception("app.artifact(%r) failed", name)
            p = None
        if not p or not os.path.isfile(str(p)):
            return self._text(404, "not found")
        headers = {}
        if attachment:
            headers["Content-Disposition"] = f'attachment; filename="{os.path.basename(str(p))}"'
        return self._file(str(p), content_type(name), headers)

    def _camera(self, name: str):
        try:
            return self.app.camera(name)
        except Exception:
            log.exception("app.camera(%r) failed", name)
            return None

    @staticmethod
    def _jpeg(cam) -> Optional[bytes]:
        try:
            data = cam.jpeg()
        except Exception:
            log.debug("camera jpeg() failed", exc_info=True)
            return None
        return bytes(data) if data else None

    @staticmethod
    def _frame_key(cam):
        """Identity of the newest frame, or None when the camera cannot tell (then bytes are compared)."""
        latest = getattr(cam, "latest", None)
        if latest is None:
            return None
        try:
            f = latest()
        except Exception:
            return None
        if f is None:
            return ("none",)
        return (getattr(f, "idx", None), getattr(f, "t", None), id(f))

    def _snapshot(self, name: str):
        cam = self._camera(name)
        if cam is None:
            return self._text(404, "no such camera")
        data = self._jpeg(cam) or placeholder_jpeg()
        return self._send(200, data, "image/jpeg")

    def _stream(self, name: str):
        cam = self._camera(name)
        if cam is None:
            return self._text(404, "no such camera")
        self.close_connection = True
        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()
        if self.command == "HEAD":
            return
        period = 1.0 / self.server.max_fps
        last_key: Any = object()
        last_frame: Optional[bytes] = None    # last real camera JPEG sent
        have_frame = False                    # did the camera have a frame at the last check
        last_sent = None
        try:
            while not self.server.stopping.is_set():
                t0 = time.monotonic()
                key = self._frame_key(cam)
                data = None
                if key == ("none",):
                    have_frame = False
                elif key is None or key != last_key:
                    data = self._jpeg(cam)
                    have_frame = data is not None
                    if data is not None and key is None and data == last_frame:
                        data = None             # camera cannot tell frames apart: same bytes, same frame
                    elif data is not None:
                        last_key = key
                if data is None and (last_sent is None or t0 - last_sent >= KEEPALIVE_S):
                    # Nothing new: placeholder when the camera has no frame, else repeat the last one so a
                    # dead client is noticed and a fresh viewer gets a picture.
                    data = last_frame if (have_frame and last_frame) else placeholder_jpeg()
                elif data is not None:
                    last_frame = data
                if data:
                    self.wfile.write(
                        (f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(data)}\r\n\r\n")
                        .encode("ascii") + data + b"\r\n")
                    self.wfile.flush()
                last_sent = t0 if data or last_sent is None else last_sent
                time.sleep(max(0.005, period - (time.monotonic() - t0)))
        except (ConnectionError, TimeoutError, OSError):
            pass        # client disconnected

    # ---- POST -----------------------------------------------------------------------------------
    @staticmethod
    def _hostport(netloc: str, scheme: str = "http") -> str:
        """Lower-case host:port with the default port filled in ("rover" -> "rover:80")."""
        netloc = (netloc or "").strip().lower().rsplit("@", 1)[-1]
        if not netloc:
            return ""
        if netloc.startswith("["):              # IPv6 literal: [::1]:8080
            host, _, rest = netloc.partition("]")
            host += "]"
            port = rest[1:] if rest.startswith(":") else ""
        elif ":" in netloc:
            host, _, port = netloc.rpartition(":")
        else:
            host, port = netloc, ""
        if not port:
            port = "443" if scheme == "https" else "80"
        return f"{host}:{port}"

    def _same_origin(self) -> bool:
        """False when Origin (or Referer if no Origin) names another host:port than the Host header.
        Requests with neither header (curl, scripts) pass here; they still need the session token."""
        src = self.headers.get("Origin")
        if src is None:
            src = self.headers.get("Referer")
            if not src:
                return True
        src = src.strip()
        if not src or src == "null":
            return False
        parts = urlsplit(src)
        if parts.scheme not in ("http", "https") or not parts.netloc:
            return False
        host = self.headers.get("Host") or ""
        return bool(host) and self._hostport(parts.netloc, parts.scheme) == self._hostport(host)

    def _host_allowed(self) -> bool:
        """DNS rebinding guard (review round 4 #7): the Host must be an IP literal, localhost or this machine's own
        name, never an arbitrary DNS name that could be rebound to the rover."""
        import ipaddress
        import socket
        host = (self.headers.get("Host") or "").strip()
        if not host:
            return True                      # HTTP/1.0 clients without Host: still need the token
        name = host.rsplit(":", 1)[0] if not host.startswith("[") else host[1:host.find("]")]
        try:
            ipaddress.ip_address(name)
            return True
        except ValueError:
            pass
        n = name.lower().rstrip(".")
        me = socket.gethostname().lower()
        # IP literals, localhost, this machine, mDNS names (*.local, only resolvable on the local link) and bare
        # single-label names (the rover's own hostname, for example GrappStar); public DNS names are refused
        return n in ("localhost", me) or n.endswith(".local") or "." not in n

    def _check_post(self) -> bool:
        """Same origin plus the per-process session token. False = refused (403 already sent)."""
        if not self._host_allowed():
            self._json(403, {"ok": False, "msg": "unknown host name: open the page by its IP address"})
            return False
        if not self._same_origin():
            self._json(403, {"ok": False, "msg": "cross-origin request refused"})
            return False
        given = self.headers.get(TOKEN_HEADER) or ""
        if not given or not hmac.compare_digest(given.encode("utf-8", "replace"),
                                                self.server.session_token.encode()):
            self._json(403, {"ok": False, "reload": True,
                             "msg": "page session expired (rover restarted?): reload the page"})
            return False
        return True

    def do_POST(self):
        path, query = self._split()
        extra = self._check_token(query)
        if extra is None:
            return
        # Refused: the body is not read, so close the connection rather than keep it alive out of sync.
        if not self._check_post():
            self.close_connection = True
            return
        m = re.fullmatch(r"/api/action/([A-Za-z0-9_\-]{1,64})", path)
        if not m:
            return self._json(404, {"ok": False, "msg": "not found"})
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            return self._json(400, {"ok": False, "msg": "bad Content-Length"})
        if length < 0 or length > MAX_BODY:
            return self._json(400, {"ok": False, "msg": "body too large"})
        raw = self.rfile.read(length) if length else b""
        if raw.strip():
            try:
                args = json.loads(raw.decode("utf-8"))
            except (ValueError, UnicodeDecodeError) as e:
                return self._json(400, {"ok": False, "msg": f"bad JSON: {e}"})
            if not isinstance(args, dict):
                return self._json(400, {"ok": False, "msg": "body must be a JSON object"})
        else:
            args = {}
        name = m.group(1)
        try:
            res = self.app.action(name, args)
        except Exception as e:
            log.exception("action %s failed", name)
            return self._json(500, {"ok": False, "msg": f"{name} failed: {type(e).__name__}: {e}"})
        if not isinstance(res, dict):
            res = {"ok": bool(res) if res is not None else True, "msg": "" if res is None else str(res)}
        res.setdefault("ok", True)
        res.setdefault("msg", "")
        return self._json(200, res)


def make_server(app, host: str = "0.0.0.0", port: int = 8080, max_fps: float = DEFAULT_MAX_FPS) -> UIServer:
    """Bind the UI server (port 0 picks a free port: see server.server_address). Call serve_forever()."""
    return UIServer((host, port), app, max_fps=max_fps)


def serve(app, host: str = "0.0.0.0", port: int = 8080, max_fps: float = DEFAULT_MAX_FPS) -> None:
    """Run the UI server until KeyboardInterrupt (blocking)."""
    srv = make_server(app, host, port, max_fps)
    log.info("operator page on http://%s:%d/", host, srv.server_address[1])
    try:
        srv.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        srv.stopping.set()
        srv.server_close()
