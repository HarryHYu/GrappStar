"""Mock KEDGE dart HTTP API for testing kedge_client.py off the hardware."""
import json, time, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
S = dict(seq="idle", event="boot", keys=[], t0=None)
SEQ = [(0, "kick swing"), (1.0, "kick + foot planting"), (1.6, "foot finishing"), (2.0, "deploy + drills"), (2.8, "drills finishing"), (3.4, "anchored")]
class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        u = urlparse(self.path); q = parse_qs(u.query); body = ""
        if u.path == "/key": S["keys"].append((time.time(), q["c"][0], int(q.get("ms", ["350"])[0]))); body = ""
        elif u.path == "/stop": S["event"] = "stop"; S["keys"].append((time.time(), " ", 0))
        elif u.path == "/auto": S["t0"] = time.time()
        elif u.path == "/status":
            if S["t0"]:
                el = time.time() - S["t0"]
                for t, n in SEQ:
                    if el >= t: S["seq"] = n; S["event"] = "auto: " + n
            body = json.dumps(dict(t_ms=0, rail_V=7.6, current_mA=120, peak_mA=400, speed=255, motors={}, seq=S["seq"], armed=False, landed=False, roll=1.0, pitch=-2.0, event=S["event"]))
        self.send_response(200); self.end_headers(); self.wfile.write(body.encode())
def serve(port=8780):
    srv = ThreadingHTTPServer(("127.0.0.1", port), H); threading.Thread(target=srv.serve_forever, daemon=True).start(); return srv
