#!/usr/bin/env python3
"""GrappStar mission: drives the KEDGE dart and the ground station together.

  fire -> dart lands (BNO085 landing flag) -> dart auto-anchors -> tow to the tension limit and hold -> [rover recovers]
  -> dart releases (drills out + pods up, then foot out + kick back) -> cock -> retrieve -> READY

  python3 mission.py --dry-run                   # against mock_dart + mock_gs on this machine
  python3 mission.py --dry-run --cycle           # same, but the ground station's own cycle helper sequences its side
  python3 mission.py --dart 192.168.4.1 --gs 192.168.5.1 --limit 20

Network: the dart (AP "KEDGE", 192.168.4.1) and the ground station (AP "GRAPPSTAR-GS", 192.168.5.1) are separate access
points on separate subnets. The Jetson joins each with its own WiFi interface (e.g. a USB WiFi stick for the second).
Any error stops both machines.
"""
import argparse, time
from kedge_client import Dart
from gs_client import GroundStation


def hold_keys(dart, keys, seconds, period=0.15):
    """hold several dart keys at once (deadman repeats, like the web page holding two buttons), then stop"""
    t_end = time.time() + seconds
    try:
        while time.time() < t_end:
            for k in keys: dart.key(k, 350)
            time.sleep(period)
    finally:
        dart.stop()


class Mission:
    def __init__(self, dart, gs, log=print, limit_n=20, tow_m=0, release_s=(4.0, 8.0), land_timeout=30, anchor_timeout=60, recover=None):
        self.d, self.g, self.log = dart, gs, log
        self.limit_n, self.tow_m, self.release_s = limit_n, tow_m, release_s
        self.land_timeout, self.anchor_timeout = land_timeout, anchor_timeout
        self.recover_fn = recover or (lambda m: None)   # the rover's self-recovery drive goes here, line held at the limit
        self.steps = []

    def step(self, name): self.steps.append(name); self.log(f"== {name}")

    def preflight(self):
        self.step("preflight")
        s = self.g.status()
        if s["state"] == "fault": raise RuntimeError(f"ground station in FAULT: {s['fault']}")
        if s["state"] != "ready": raise RuntimeError(f"ground station not READY ({s['state']}): cock first")
        if not s["switches"]["seated"]: raise RuntimeError("dart not seated")
        ds = self.d.status()
        if ds["seq"] not in ("idle", "aborted"): raise RuntimeError(f"dart busy ({ds['seq']})")
        self.g.limit(self.limit_n)
        if not ds["armed"]: self.d.key("A")            # arm auto-anchor on landing (also re-arms the landing detector)
        if not self.d.status()["armed"]: raise RuntimeError("dart did not arm")

    def wait_landing(self):
        self.step("wait landing")
        t0 = time.time()
        while time.time() - t0 < self.land_timeout:
            if self.d.status()["landed"]: self.log(f"dart landed after {time.time() - t0:.1f} s"); return
            time.sleep(0.1)
        raise TimeoutError("dart landing not detected")

    def anchor(self):
        self.step("anchor")
        t0 = time.time()
        while time.time() - t0 < 2.0 and self.d.status()["seq"] == "idle": time.sleep(0.1)   # armed: it starts by itself
        if self.d.status()["seq"] == "idle": self.log("auto did not start on landing: starting it"); self.d.auto()
        s = self.d.wait_auto(poll=0.1, limit=self.anchor_timeout, log=self.log)
        if s["seq"] != "anchored": raise RuntimeError(f"dart did not anchor ({s['seq']}: {s['event']})")

    def release(self):
        self.step("release dart")
        hold_keys(self.d, ["#", "!"], self.release_s[0])   # drills unscrew while the pods lift
        hold_keys(self.d, ["$", "@"], self.release_s[1])   # foot unscrews while the kick swings back

    def run(self):
        """ground station driven step by step from here"""
        try:
            self.preflight()
            self.step("fire"); self.g.fire(); self.g.wait_state("payout", limit=15, log=self.log)
            self.wait_landing(); self.anchor()
            self.step("tow"); self.g.tow(self.tow_m); s = self.g.wait_holding(log=self.log); self.log(f"holding at {s['tension_N']} N, line {s['line_m']} m")
            self.step("recover"); self.recover_fn(self)
            self.release()
            self.step("cock"); self.g.cock(); s = self.g.wait_state("ready", "idle", limit=20, log=self.log)
            if s["state"] != "ready": raise RuntimeError("cocked but not READY (rail-down switch?)")
            self.step("retrieve"); self.g.retrieve(); s = self.g.wait_state("ready", "idle", limit=240, log=self.log)
            if s["state"] != "ready" or not s["switches"]["seated"]: raise RuntimeError("retrieve ended without the dart seated")
            self.step("done"); return s
        except BaseException:
            self.abort(); raise

    def run_cycle(self):
        """the ground station's cycle helper sequences its side; this feeds it the anchored / released flags"""
        try:
            self.preflight()
            self.step("cycle: fire"); self.g.cycle(self.tow_m)
            self.wait_landing(); self.anchor()
            self.step("cycle: anchored -> tow"); self.g.anchored()
            s = self.g.wait(lambda s: s["cycle"] == "wait released", limit=180, log=self.log, what="tow holding")
            self.log(f"holding at {s['tension_N']} N, line {s['line_m']} m")
            self.step("recover"); self.recover_fn(self)
            self.release()
            self.step("cycle: released -> cock -> retrieve"); self.g.released()
            s = self.g.wait(lambda s: s["cycle"] == "off", limit=260, log=self.log, what="cycle end")
            if s["state"] != "ready" or not s["switches"]["seated"]: raise RuntimeError(f"cycle ended {s['state']}: {s['event']}")
            self.step("done"); return s
        except BaseException:
            self.abort(); raise

    def abort(self):
        self.log("mission aborted: stopping both")
        for f in (self.g.stop, self.d.stop):
            try: f()
            except Exception as e: self.log(f"stop failed: {e}")


def dry_run_setup(dart_port=8780, gs_port=8781, flight_s=1.2):
    """start both mocks and link them: the shot makes the mock dart land flight_s later"""
    import mock_dart, mock_gs
    sd = mock_dart.serve(dart_port); sg = mock_gs.serve(gs_port)
    mock_gs.on_shot = lambda: (time.sleep(flight_s), mock_dart.land())
    return sd, sg, Dart(f"127.0.0.1:{dart_port}"), GroundStation(f"127.0.0.1:{gs_port}")


def main():
    ap = argparse.ArgumentParser(description="GrappStar mission (dart + ground station)")
    ap.add_argument("--dart", default="192.168.4.1"); ap.add_argument("--gs", default="192.168.5.1")
    ap.add_argument("--limit", type=float, default=20); ap.add_argument("--tow-m", type=float, default=0)
    ap.add_argument("--cycle", action="store_true"); ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    if a.dry_run:
        sd, sg, d, g = dry_run_setup(); rel = (0.5, 0.5)
    else:
        d, g = Dart(a.dart), GroundStation(a.gs); rel = (4.0, 8.0)
    m = Mission(d, g, limit_n=a.limit, tow_m=a.tow_m, release_s=rel)
    s = m.run_cycle() if a.cycle else m.run()
    print("mission complete:", s["state"], "seated" if s["switches"]["seated"] else "")
    if a.dry_run: sd.shutdown(); sg.shutdown()


if __name__ == "__main__":
    main()
