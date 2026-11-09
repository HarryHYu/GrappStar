"""Mock KEDGE dart HTTP API for testing kedge_client.py (and mission.py) off the hardware.
land() simulates the BNO085 landing detector; if armed ('A' key) the auto-anchor starts on landing like the firmware."""
import json, time, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
S = dict(seq="idle", event="boot", keys=[], t0=None, armed=False, landed=False)
# firmware v4.3 (dart v34), dart landed belly down: no righting, pods down with the drills, then anchored (a dart on its
# back or side first shows "righting: ..." steps)
SEQ = [(0, "deploy + drills"), (1.4, "drills finishing"), (3.4, "anchored")]
def land():
    S["landed"] = True; S["event"] = "landed"
    if S["armed"] and S["t0"] is None: S["t0"] = time.time()
class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        u = urlparse(self.path); q = parse_qs(u.query); body = ""
        if u.path == "/key":
            c = q["c"][0]; S["keys"].append((time.time(), c, int(q.get("ms", ["350"])[0]))); body = ""
            if c == "A": S["armed"] = not S["armed"]; S["landed"] = False; S["event"] = "auto armed for landing" if S["armed"] else "auto disarmed"
        elif u.path == "/stop": S["event"] = "stop"; S["keys"].append((time.time(), " ", 0))
        elif u.path == "/auto": S["t0"] = time.time()
        elif u.path == "/status":
            if S["t0"]:
                el = time.time() - S["t0"]
                for t, n in SEQ:
                    if el >= t: S["seq"] = n; S["event"] = "auto: " + n
            body = json.dumps(dict(t_ms=0, rail_V=7.6, current_mA=120, peak_mA=400, speed=255, motors={}, seq=S["seq"], armed=S["armed"], landed=S["landed"], roll=1.0, pitch=-2.0, event=S["event"]))
        self.send_response(200); self.end_headers(); self.wfile.write(body.encode())
def serve(port=8780):
    srv = ThreadingHTTPServer(("127.0.0.1", port), H); threading.Thread(target=srv.serve_forever, daemon=True).start(); return srv
