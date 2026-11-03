"""Checks that need no hardware: python3 -m grappstar selftest. Prints PASS or FAIL per check."""
from __future__ import annotations

import sys
import tempfile
import time
import traceback


def _check(name, fn, results, verbose):
    t0 = time.time()
    try:
        msg = fn() or ""
        results.append((name, True))
        if verbose:
            print(f"PASS  {name}  {msg}  ({time.time() - t0:.1f} s)")
    except Exception as e:
        results.append((name, False))
        if verbose:
            print(f"FAIL  {name}: {type(e).__name__}: {e}")
            traceback.print_exc(limit=2)


def run(verbose: bool = True) -> int:
    results = []

    def versions():
        import cv2
        import numpy as np
        v = f"python {sys.version.split()[0]}, numpy {np.__version__}, opencv {cv2.__version__}"
        if sys.version_info < (3, 8):
            raise RuntimeError("Python too old: " + v)
        return v

    def base():
        from .comms.mock_base import connected_pair
        mb, c = connected_pair(time_scale=10)
        try:
            c.ping()
            m = c.event_mark()
            c.mode("LAUNCH")
            c.wait_event("MODE", 5, since=m)
            return "mock base: ping, mode LAUNCH ok"
        finally:
            c.close(); mb.stop()

    def dart():
        from .comms.dart import Dart
        from .comms.mock_dart import MockDart
        md = MockDart(time_scale=10).start()
        try:
            d = Dart(md.host)
            md.land(); d.auto()
            d.wait_phase("ANCHORED", 5, poll=0.05)
            return "mock dart: land, auto, anchored ok"
        finally:
            md.stop()

    def ballistics():
        from .ballistics import Ballistics
        b = Ballistics.load()
        s = b.solve(7.0)
        if not s.reachable:
            raise RuntimeError(s.note)
        return f"7 m -> pitch {s.pitch_deg} deg ({b.source})"

    def terrain():
        from .sim.terrain_textures import texture
        from .vision.terrain import analyze_landing
        r = analyze_landing([texture("hard_ground", 480, 720, seed=3)])
        return f"hard ground texture -> {r.cls} {r.score} {r.verdict}"

    def target():
        from .sim.world import default_rover_model, render_rover_view
        from .vision.target import detect_orange
        m = default_rover_model()
        img = render_rover_view(m, (1280, 720), marker=(6.0, 5.0))
        d = detect_orange(img)
        rb = m.pixel_to_ground(d[0], d[1], image_size=(1280, 720))
        if abs(rb[0] - 6.0) > 0.6 or abs(rb[1] - 5.0) > 1.5:
            raise RuntimeError(f"marker at 6 m 5 deg read as {rb}")
        return f"marker 6.0 m 5 deg -> {rb[0]:.2f} m {rb[1]:.1f} deg"

    def flight():
        from .sim.flight_video import render_flight
        from .vision.flight import analyze_flight
        frames, truth = render_flight(seed=1, size=(360, 240))
        r = analyze_flight(frames, truth["flightlog"])
        return f"launch err {abs(r.t_launch - truth['t_launch']):.2f} s, impact err {abs(r.t_impact - truth['t_impact']):.2f} s"

    def ui():
        import json
        import threading
        import urllib.request
        from .ui.server import make_server

        class A:
            ui_token = ""
            def status(self): return {"ok": 1}
            def action(self, n, a): return {"ok": True, "msg": n}
            def camera(self, n): return None
            def artifact(self, n): return None
        srv = make_server(A(), "127.0.0.1", 0)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        try:
            port = srv.server_address[1]
            s = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{port}/api/status", timeout=2).read())
            html = urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=2).read()
            if s.get("ok") != 1 or b"<html" not in html.lower():
                raise RuntimeError("bad page")
            return "page and API answer"
        finally:
            srv.shutdown()

    for name, fn in (("versions", versions), ("mock base C1", base), ("mock dart C2", dart),
                     ("ballistics C3", ballistics), ("terrain", terrain), ("targeting", target),
                     ("flight analysis", flight), ("web page", ui)):
        _check(name, fn, results, verbose)
    bad = [n for n, ok in results if not ok]
    print(("ALL PASS" if not bad else "FAILED: " + ", ".join(bad)))
    return 1 if bad else 0
