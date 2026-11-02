#!/usr/bin/env python3
"""GrappStar ground station client (launcher + reel) for the rover Jetson or any laptop on the GRAPPSTAR-GS WiFi (192.168.5.1).

Standard library only.

  python3 gs_client.py status                    # one status JSON
  python3 gs_client.py watch                     # status 5x per second, logs to gs_log.csv
  python3 gs_client.py cmd cock                  # any /cmd: cock fire tow retrieve stop cycle limit anchored released clear line zero finish lower force
  python3 gs_client.py cmd limit 15              # with a value
  python3 gs_client.py jog f --hold 1.5          # hold reel wind for 1.5 s (deadman repeats like the web page)

In your own code:
  from gs_client import GroundStation
  g = GroundStation(); g.cock(); g.wait_state("ready"); g.fire()
"""
import argparse, csv, json, time, urllib.request, urllib.parse, urllib.error


class Refused(Exception):
    """the ground station answered 409: command not allowed in this state (text says why)"""


class GroundStation:
    def __init__(self, host="192.168.5.1", timeout=1.0):
        self.base = f"http://{host}"; self.timeout = timeout

    def _get(self, path):
        try:
            with urllib.request.urlopen(self.base + path, timeout=self.timeout) as r:
                return r.read().decode()
        except urllib.error.HTTPError as e:
            if e.code == 409: raise Refused(e.read().decode().strip()) from None
            raise

    def status(self):
        return json.loads(self._get("/status"))

    def cmd(self, c, v=None):
        q = {"c": c}
        if v is not None: q["v"] = v
        return self._get("/cmd?" + urllib.parse.urlencode(q))

    def key(self, c, ms=350):
        return self._get("/key?" + urllib.parse.urlencode({"c": c, "ms": int(ms)}))

    def stop(self): return self._get("/stop")
    def cock(self): return self.cmd("cock")
    def fire(self): return self.cmd("fire")
    def tow(self, dist_m=0): return self.cmd("tow", dist_m)
    def retrieve(self): return self.cmd("retrieve")
    def cycle(self, tow_m=0): return self.cmd("cycle", tow_m)
    def limit(self, newtons): return self.cmd("limit", newtons)
    def anchored(self): return self.cmd("anchored")
    def released(self): return self.cmd("released")
    def set_line(self, m): return self.cmd("line", m)
    def zero(self): return self.cmd("zero")
    def clear(self): return self.cmd("clear")
    def finish(self): return self.cmd("finish")   # run B from phi 241..331 until the rail lands (phi 333): FIRES if cocked
    def lower(self): return self.cmd("lower")     # rail up and phi unknown only: B until the rail lands, phi re-zeroed to 333. FIRES if cocked
    def force(self): return self.cmd("force")     # allow the next F with the carriage partly drawn

    def hold(self, c, seconds, period=0.15):
        """jog f or b for `seconds` the way the web page does: repeat every 150 ms, each repeat runs 350 ms."""
        t_end = time.time() + seconds
        try:
            while time.time() < t_end:
                self.key(c, 350); time.sleep(period)
        finally:
            self.stop()

    def wait(self, pred, limit=60, poll=0.1, log=None, what="condition"):
        """poll status until pred(status) is true. Raises on FAULT or timeout (motor stopped first)."""
        t0 = time.time(); last = None
        while time.time() - t0 < limit:
            s = self.status()
            if log and s["event"] != last: log(f"gs {time.time() - t0:6.1f}s  {s['state']:<9} phi {s['phi']:6.1f}  line {s['line_m']:5.2f} m  {s['tension_N']:5.1f} N  {s['event']}"); last = s["event"]
            if pred(s): return s
            if s["state"] == "fault": raise RuntimeError(f"ground station FAULT: {s['fault']}")
            time.sleep(poll)
        self.stop(); raise TimeoutError(f"ground station: timed out waiting for {what}")

    def wait_state(self, *states, limit=60, log=None):
        return self.wait(lambda s: s["state"] in states, limit, log=log, what="/".join(states))

    def wait_holding(self, limit=180, log=None):
        return self.wait(lambda s: s["state"] == "tow" and s["holding"], limit, log=log, what="tow holding")


def main():
    ap = argparse.ArgumentParser(description="GrappStar ground station client")
    ap.add_argument("what", choices=["status", "watch", "cmd", "jog", "stop"])
    ap.add_argument("c", nargs="?"); ap.add_argument("v", nargs="?", type=float)
    ap.add_argument("--hold", type=float, default=1.0); ap.add_argument("--host", default="192.168.5.1"); ap.add_argument("--log", default="gs_log.csv")
    a = ap.parse_args(); g = GroundStation(a.host)
    if a.what == "status": print(json.dumps(g.status(), indent=1))
    elif a.what == "stop": print(g.stop())
    elif a.what == "cmd":
        try: print(g.cmd(a.c, a.v))
        except Refused as e: print("refused:", e)
    elif a.what == "jog": g.hold(a.c, a.hold)
    elif a.what == "watch":
        with open(a.log, "a", newline="") as f:
            w = csv.writer(f)
            while True:
                try:
                    s = g.status(); row = [time.strftime("%H:%M:%S"), s["state"], s["phi"], s["line_m"], s["current_A"], s["tension_N"], s["holding"], s["event"]]
                    w.writerow(row); f.flush(); print("  ".join(map(str, row)))
                except Exception as e:
                    print("no link:", e)
                time.sleep(0.2)


if __name__ == "__main__":
    main()
