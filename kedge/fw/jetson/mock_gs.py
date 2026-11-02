"""Mock GrappStar ground station HTTP API: same states, refusals and JSON as gs_logic.h, driven by time instead of a motor.
"cocked" here = carriage held behind the sear hook. The reported switches.cocked is the KW11 (COM + NO, pressed by the sear arm): hook up = true.

  import mock_gs; srv = mock_gs.serve(port=8781)        # then GroundStation("127.0.0.1:8781")
  mock_gs.on_shot = lambda: ...                          # called when the shot goes (hook a mock dart landing to it)
"""
import json, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

# seconds for each motion (plant model: cock ~2.9 s from 338, fire ~0.9 s, retrieve ~100 s for 12.4 m)
T = dict(cock=1.0, fire=0.6, flight=1.2, tow=1.5, retrieve=2.0, lower=0.6)
PAYOUT_MPS = 10.8; PAYOUT_MAX = 12.0   # CAD flight: 10.8 m/s, 11.5 m of line out, range 10.0 m
PHI_COCKED, PHI_TRIP, PHI_END, PHI_ZONE = 249.5, 294.5, 333.0, 241.0   # CAD angles after review round 9 (fire ends as the rail lands, phi re-zeroed to 333) (gs_logic.h Tuning)
on_shot = None
lock = threading.RLock()


def fresh():
    return dict(state="ready", cycle="off", phi=PHI_COCKED, phi_known=True, line_m=0.0, tension=0.0, limit=20.0, holding=False,
                low=True, cocked=True, hook=True, force=0.0, finishing=False, seated=True, anchored=False, released=False, shot=False, fault="", event="boot",
                t0=0.0, jog_until=0.0, jog_dir=0, tow_dist=0.0, tow_line0=0.0, line0=0.0, keys=[], cmds=[], inject_fault=None)


S = fresh()


def ev(s): S["event"] = s


def theta(phi):
    """coarse copy of the CAD cam (round 6): lift 252.5 -> 302.5 to 35 deg, top 302.5 -> 304.5, drop 304.5 -> 332.5"""
    if phi < 252.5 or phi >= 332.5: return 4.0
    if phi < 302.5: return 4 + 31 * (phi - 252.5) / 50.0
    if phi < 304.5: return 35.0
    return 35 - 31 * (phi - 304.5) / 28.0


def settle():
    S["holding"] = False
    S["state"] = "ready" if S["cocked"] and S["low"] and PHI_ZONE <= S["phi"] <= 252.5 else "idle"
    if S["state"] == "ready": S["phi"] = PHI_COCKED


def can_start(): return S["state"] in ("idle", "ready", "payout") or (S["state"] == "tow" and S["holding"])


def f_ok():
    """F refused with the carriage partly drawn (phi 340..241, held only by backstop C) unless force was sent"""
    p = S["phi"]
    if not (p > 340 or p < PHI_ZONE): return True
    if time.time() - S["force"] < 10: S["force"] = 0.0; return True
    ev("F refused: carriage partly drawn (phi 340..241). cock first, or force"); return False


def start(state):
    S["state"] = state; S["t0"] = time.time()


def do_cmd(c, v):
    """returns (ok, text) with the same rules as GS::cmd"""
    S["cmds"].append((time.time(), c, v))
    if c == "stop":
        was = S["cycle"] != "off"; S["cycle"] = "off"
        if S["state"] == "payout": S["state"] = "idle"; ev("stop")
        elif S["state"] != "fault": settle(); ev("cycle aborted, stopped" if was else "stop")
        return True, S["event"]
    if c == "clear":
        if S["state"] == "fault": S["fault"] = ""; settle(); ev("fault cleared")
        return True, S["event"]
    if c == "limit":
        if 2 <= v <= 50: S["limit"] = v; ev(f"limit {v:.1f} N"); return True, S["event"]
        ev("limit must be 2..50 N (reel gears and grubs)"); return False, S["event"]
    if c in ("anchored", "released"): S[c] = True; ev(f"{c} flag set"); return True, S["event"]
    if c == "line": S["line_m"] = v; ev("line estimate set"); return True, S["event"]
    if c == "zero":
        if S["state"] == "tow" or S["cycle"] != "off": ev("z refused while towing or in a cycle: the line may be loaded (HX711 tare)"); return False, S["event"]
        ev("zeroed"); return True, S["event"]
    if c == "force": S["force"] = time.time(); ev("force: next F allowed with the carriage partly drawn (10 s)"); return True, S["event"]
    if S["state"] == "fault": ev("in FAULT: clear first"); return False, S["event"]
    if S["cycle"] != "off": ev("cycle running: stop first"); return False, S["event"]
    if c == "fire" or c == "cycle":
        if S["state"] != "ready": ev("refused: fire only from READY (cock first)"); return False, S["event"]
        if not S["seated"]: ev("refused: dart not seated"); return False, S["event"]
        if c == "cycle": S["cycle"] = "fire"; S["tow_dist"] = v; S["anchored"] = S["released"] = False
        S["shot"] = False; S["finishing"] = False; S["anchored"] = False; start("firing"); ev("firing"); return True, S["event"]
    if not can_start(): ev("busy: stop first"); return False, S["event"]
    if c == "cock":
        if S["cocked"] and S["low"] and PHI_ZONE <= S["phi"] <= 252.5: settle(); ev("already cocked"); return True, S["event"]
        if not S["low"]: ev("cock refused: rail is up (SW_LOW open). Use finish" if S["phi_known"] else "cock refused: rail is up, phi unknown. Use lower"); return False, S["event"]
        if 252.5 < S["phi"] < 331: ev("refused: rail is in the lift/drop zone. Use finish"); return False, S["event"]
        start("cocking"); S["holding"] = False; ev("cocking"); return True, S["event"]
    if c == "finish":
        if not S["phi_known"]: ev("finish refused: phi unknown with the rail up. Use lower" if not S["low"] else "finish refused: phi unknown"); return False, S["event"]
        if not PHI_ZONE <= S["phi"] < 338: ev("finish refused: only from phi 241..338"); return False, S["event"]
        S["shot"] = False; S["finishing"] = True; start("firing"); ev("finish: running B to 338"); return True, S["event"]
    if c == "lower":
        if S["low"]: ev("lower refused: rail is already down"); return False, S["event"]
        if S["phi_known"]: ev("lower refused: phi is known. Use finish"); return False, S["event"]
        start("lower"); ev("lower: B slowly until the rail is down (fires if cocked)"); return True, S["event"]
    if c in ("tow", "retrieve") and not S["seated"] and not f_ok(): return False, S["event"]
    if c == "tow":
        S["tow_dist"] = v; S["tow_line0"] = S["line_m"]; S["holding"] = False; start("tow"); ev(f"tow, limit {S['limit']:.1f} N"); return True, S["event"]
    if c == "retrieve":
        if S["seated"]: S["line_m"] = 0; settle(); ev("dart already seated"); return True, S["event"]
        S["line0"] = S["line_m"]; S["holding"] = False; start("retrieve"); ev("retrieve"); return True, S["event"]
    ev("unknown command"); return False, S["event"]


def fault(why):
    S["state"] = "fault"; S["cycle"] = "off"; S["holding"] = False; S["fault"] = why; ev("FAULT: " + why)


def tick():
    now = time.time(); el = now - S["t0"]; st = S["state"]
    if S["inject_fault"] and st in ("cocking", "firing", "tow", "retrieve", "jog", "lower"): fault(S["inject_fault"]); S["inject_fault"] = None; return
    if st == "jog" and now >= S["jog_until"]: settle()
    elif st == "cocking":
        S["phi"] = (PHI_END + (360 - PHI_END + PHI_COCKED) * min(1, el / T["cock"])) % 360
        if el >= T["cock"]: S["cocked"] = True; settle(); ev("cocked: READY")
    elif st == "firing":
        S["phi"] = PHI_COCKED + (PHI_END - PHI_COCKED) * min(1, el / T["fire"]); S["low"] = theta(S["phi"]) < 4.5
        if not S["shot"] and S["phi"] >= PHI_TRIP:
            S["shot"] = True; S["cocked"] = False; S["seated"] = False; S["shot_t"] = now; ev("shot: sear tripped (SW_COCKED open)")
            if on_shot: threading.Thread(target=on_shot, daemon=True).start()
        if el >= T["fire"] and S["finishing"]: S["low"] = True; S["phi"] = PHI_END; S["finishing"] = False; settle(); ev("finished: rail down")
        elif el >= T["fire"]: S["low"] = True; S["phi"] = PHI_END; start("payout"); S["shot_t"] = S.get("shot_t", now); ev("fired: PAYOUT")
    elif st == "lower" and el >= T["lower"]:
        if S["cocked"]:
            S["cocked"] = False; S["seated"] = False; S["shot"] = True; S["shot_t"] = now; ev("lower: sear cord pulled (shot, if it was cocked)")
            if on_shot: threading.Thread(target=on_shot, daemon=True).start()
        S["low"] = True; S["phi"] = PHI_END; S["phi_known"] = True; settle(); ev("lowered")
    elif st == "payout":
        S["line_m"] = max(S["line_m"], min(PAYOUT_MAX, PAYOUT_MPS * (now - S["shot_t"])))
    elif st == "tow" and not S["holding"]:
        f = min(1, el / T["tow"]); S["tension"] = S["limit"] * f; S["line_m"] = max(0, S["tow_line0"] - 1.5 * f)
        if S["tow_dist"] and S["tow_line0"] - S["line_m"] >= S["tow_dist"]: S["holding"] = True; ev("tow distance reached: holding")
        elif f >= 1: S["holding"] = True; ev(f"holding at limit {S['tension']:.1f} N")
    elif st == "retrieve":
        S["tension"] = 3.0; S["line_m"] = max(0, S["line0"] * (1 - el / T["retrieve"]))
        if el >= T["retrieve"]: S["seated"] = True; S["line_m"] = 0; S["tension"] = 0; settle(); ev("dart seated: READY" if S["state"] == "ready" else "dart seated (not cocked)")
    if st != "tow" and S["state"] != "tow": S["tension"] = 0.0 if S["state"] != "retrieve" else S["tension"]
    # cycle helper
    cy = S["cycle"]
    if cy == "fire" and S["state"] == "payout": S["cycle"] = "wait anchored"; ev("cycle: waiting for anchored")
    elif cy == "wait anchored" and S["anchored"]: do_cmd_cycle("tow", S["tow_dist"]); S["cycle"] = "tow"
    elif cy == "tow" and S["state"] == "tow" and S["holding"]: S["cycle"] = "wait released"; S["released"] = False; ev("cycle: waiting for released")
    elif cy == "wait released" and S["released"]: do_cmd_cycle("cock", 0); S["cycle"] = "cock"
    elif cy == "cock" and S["state"] == "ready": do_cmd_cycle("retrieve", 0); S["cycle"] = "retrieve"
    elif cy == "retrieve" and S["state"] != "retrieve": ev("cycle complete: READY" if S["state"] == "ready" else "cycle ended, not READY"); S["cycle"] = "off"


def do_cmd_cycle(c, v):
    saved = S["cycle"]; S["cycle"] = "off"
    try: do_cmd(c, v)
    finally: S["cycle"] = saved


def status():
    return dict(t_ms=int(time.time() * 1000) % 100000000, state=S["state"], cycle=S["cycle"], phi=round(S["phi"], 1), phi_known=S["phi_known"],
                theta_est=round(theta(S["phi"]), 1), turns=0.0, turns_f=0.0, turns_b=0.0, line_m=round(S["line_m"], 2), current_A=1.6 if S["state"] in ("cocking", "firing", "tow", "retrieve", "jog") else 0.0,
                peak_A=3.2, tension_N=round(S["tension"], 1), tension_src="current", limit_N=S["limit"], holding=S["holding"], dir=0, duty=0,
                switches=dict(low=S["low"], cocked=not (S["state"] == "firing" and theta(S["phi"]) >= 25), seated=S["seated"]), enc_ok=True, anchored=S["anchored"], released=S["released"],
                shot=S["shot"], fault=S["fault"], event=S["event"])


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def do_GET(self):
        u = urlparse(self.path); q = parse_qs(u.query); code, body, ctype = 200, "", "text/plain"
        with lock:
            tick()
            if u.path == "/status": body = json.dumps(status()); ctype = "application/json"
            elif u.path == "/stop": do_cmd("stop", 0); body = S["event"] + "\n"
            elif u.path == "/cmd":
                ok, txt = do_cmd(q.get("c", [""])[0], float(q.get("v", ["0"])[0])); code = 200 if ok else 409; body = txt + "\n"
            elif u.path == "/key":
                c = q.get("c", [""])[0]; ms = min(max(int(q.get("ms", ["350"])[0]), 50), 1500); S["keys"].append((time.time(), c, ms))
                if c in (" ", "s"): do_cmd("stop", 0)
                elif c in ("f", "b"):
                    busy = S["cycle"] != "off" or not (S["state"] in ("idle", "ready", "payout", "jog") or (S["state"] == "tow" and S["holding"]))
                    if busy: ev("jog refused: busy or FAULT")
                    elif c == "b" and ((S["phi_known"] and PHI_ZONE <= S["phi"] <= 338) or not S["low"]): ev("jog b refused: cocked, lifting or rail up (phi 241..338). Use fire or finish")
                    elif c == "f" and S["state"] != "jog" and not f_ok(): pass
                    else: S["state"] = "jog"; S["holding"] = False; S["jog_until"] = time.time() + ms / 1000
        self.send_response(code); self.send_header("Content-Type", ctype); self.end_headers(); self.wfile.write(body.encode())


def _ticker(stop):
    while not stop.is_set():
        with lock: tick()
        time.sleep(0.02)


def serve(port=8781):
    global S
    S.clear(); S.update(fresh())
    srv = ThreadingHTTPServer(("127.0.0.1", port), H); srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    stop = threading.Event(); threading.Thread(target=_ticker, args=(stop,), daemon=True).start()
    orig = srv.shutdown
    def shutdown(): stop.set(); orig()
    srv.shutdown = shutdown
    return srv
