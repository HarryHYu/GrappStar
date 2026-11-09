#!/usr/bin/env python3
"""KEDGE dart client for the rover (Jetson Orin Nano) or any laptop on the KEDGE WiFi.

Uses only the Python standard library for control (urllib); OpenCV is optional, for the FPV overlay.

  python3 kedge_client.py status                 # one status line
  python3 kedge_client.py watch                  # status 5x per second, logs to kedge_log.csv
  python3 kedge_client.py key 2 --hold 1.5       # hold "leg out" for 1.5 s (deadman repeats like the web page)
  python3 kedge_client.py auto                   # run the auto-anchor sequence (righting first if needed) and follow it
  python3 kedge_client.py drive                  # keyboard driving in a terminal (hold keys; space = stop; q = quit)
  python3 kedge_client.py video --dev 0          # FPV from the RC832 + capture card with telemetry overlay (needs opencv)

In your own code:
  from kedge_client import Dart
  d = Dart(); print(d.status()); d.hold('2', 1.0); d.stop()
"""
import argparse, csv, json, sys, time, threading, urllib.request, urllib.parse

# firmware v4.3 (dart v34): 2/@ drive the righting leg (the old kick channel); there is no foot (4/$ are refused)
KEYS = {"1": "pods down", "!": "pods up", "2": "leg out", "@": "leg back", "3": "drills in", "#": "drills out"}


class Dart:
    def __init__(self, host="192.168.4.1", timeout=1.0):
        self.base = f"http://{host}"; self.timeout = timeout

    def _get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=self.timeout) as r:
            return r.read().decode()

    def status(self):
        return json.loads(self._get("/status"))

    def key(self, c, ms=350):
        return self._get("/key?" + urllib.parse.urlencode({"c": c, "ms": int(ms)}))

    def stop(self):
        return self._get("/stop")

    def auto(self):
        return self._get("/auto")

    def hold(self, c, seconds, period=0.15):
        """run a motor key for `seconds` the same way the web page does: repeat every 150 ms, each repeat runs 350 ms."""
        t_end = time.time() + seconds
        try:
            while time.time() < t_end:
                self.key(c, 350); time.sleep(period)
        finally:
            self.stop()

    def wait_auto(self, poll=0.2, limit=60, log=print):
        t0 = time.time(); last = None
        while time.time() - t0 < limit:
            s = self.status()
            if s["event"] != last: log(f"{time.time() - t0:6.1f}s  {s['seq']:<22} {s['current_mA']:>6} mA  {s['event']}"); last = s["event"]
            if s["seq"] in ("anchored", "aborted"): return s
            time.sleep(poll)
        self.stop(); raise TimeoutError("auto sequence did not finish")


def cmd_watch(d, args):
    with open(args.log, "a", newline="") as f:
        w = csv.writer(f)
        while True:
            try:
                s = d.status(); row = [time.strftime("%H:%M:%S"), s["rail_V"], s["current_mA"], s["roll"], s["pitch"], s["seq"], s["event"]]
                w.writerow(row); f.flush(); print("  ".join(map(str, row)))
            except Exception as e:
                print("no link:", e)
            time.sleep(0.2)


def cmd_drive(d, args):
    import curses
    def ui(scr):
        curses.cbreak(); scr.nodelay(True); scr.keypad(True)
        scr.addstr(0, 0, "KEDGE drive: hold 1/! 2/@ 3/#   space stop   a auto   q quit")
        last_key, last_t = None, 0
        while True:
            ch = scr.getch(); now = time.time()
            if ch != -1:
                c = chr(ch) if ch < 256 else ""
                if c == "q": d.stop(); return
                if c in KEYS: d.key(c, 350); last_key, last_t = c, now
                elif c in (" ", "s"): d.stop(); last_key = None
                elif c == "a": d.auto()
            elif last_key and now - last_t > 0.25:    # terminal key repeat stopped = key released
                last_key = None
            try:
                s = d.status(); scr.addstr(2, 0, f"rail {s['rail_V']:5.2f} V  {s['current_mA']:6.0f} mA   roll {s['roll']:6.1f}  pitch {s['pitch']:6.1f}   {s['seq']:<22}".ljust(90))
                scr.addstr(3, 0, f"last: {s['event']}".ljust(90))
            except Exception as e:
                scr.addstr(2, 0, f"no link: {e}".ljust(90))
            scr.refresh(); time.sleep(0.05)
    curses.wrapper(ui)


def cmd_video(d, args):
    import cv2
    cap = cv2.VideoCapture(args.dev)
    st = {"s": None}
    def poll():
        while True:
            try: st["s"] = d.status()
            except Exception: st["s"] = None
            time.sleep(0.2)
    threading.Thread(target=poll, daemon=True).start()
    while True:
        ok, fr = cap.read()
        if not ok: print("no video frame (check the capture card is /dev/video%d)" % args.dev); time.sleep(0.5); continue
        s = st["s"]
        txt = "no link" if s is None else f"{s['rail_V']:.2f} V  {s['current_mA']:.0f} mA  roll {s['roll']:.0f}  pitch {s['pitch']:.0f}  {s['seq']}"
        cv2.putText(fr, txt, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("KEDGE FPV", fr)
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"): break
        if k == ord(" "): d.stop()
        if chr(k) in KEYS: d.key(chr(k), 350)


def main():
    ap = argparse.ArgumentParser(description="KEDGE dart client")
    ap.add_argument("cmd", choices=["status", "watch", "key", "stop", "auto", "drive", "video"])
    ap.add_argument("c", nargs="?"); ap.add_argument("--hold", type=float, default=1.0)
    ap.add_argument("--host", default="192.168.4.1"); ap.add_argument("--log", default="kedge_log.csv"); ap.add_argument("--dev", type=int, default=0)
    a = ap.parse_args(); d = Dart(a.host)
    if a.cmd == "status": print(json.dumps(d.status(), indent=1))
    elif a.cmd == "watch": cmd_watch(d, a)
    elif a.cmd == "key": d.hold(a.c, a.hold)
    elif a.cmd == "stop": print(d.stop())
    elif a.cmd == "auto": d.auto(); d.wait_auto()
    elif a.cmd == "drive": cmd_drive(d, a)
    elif a.cmd == "video": cmd_video(d, a)


if __name__ == "__main__":
    main()
