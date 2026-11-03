"""Command line: python3 -m grappstar <command> ...

  serve       operator web page (http://<jetson>:8080); --sim runs everything simulated
  cameras     list cameras and their formats
  snap        save frames from a camera (for calibration images)
  base        talk to the base: status | ping | monitor | cmd '<json>' | mode X | aim DEG | stop | estop | clear
  dart        talk to the dart: status | key C --hold S | stop | auto | release | flightlog
  mockbase    run the simulated base on a pseudo terminal (--pty) for testing the client
  mockdart    run the simulated dart HTTP API
  calibrate   rover camera: intrinsics | ground | mount | marker | default (writes calib/rovercam.json)
  replay      re-run the flight and landing analysis on a recorded run folder
  solve       ballistics: pitch for a range
  selftest    checks that need no hardware
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
JETSON_DIR = os.path.dirname(HERE)
DEFAULT_CALIB = os.path.join(JETSON_DIR, "calib", "rovercam.json")


def _size(s: str):
    w, h = s.lower().split("x")
    return (int(w), int(h))


def cmd_serve(a) -> int:
    from .app import App, AppConfig
    from .ui.server import serve
    cfg = AppConfig(sim=a.sim, base_port=a.base, dart_host=a.dart, rover_cam=a.rover_cam, dart_cam=a.dart_cam,
                    dart_fourcc=a.fourcc, rover_fourcc=a.rover_fourcc, rover_size=_size(a.rover_size),
                    dart_size=_size(a.dart_size), gstreamer=a.gst, ui_token=a.token or "",
                    sim_landing=a.sim_landing, sim_seed=a.sim_seed, calib_path=a.calib,
                    dart_deinterlace=a.deinterlace, terrain_model=a.model, ballistics_path=a.ballistics)
    if a.runs:
        cfg.runs_dir = a.runs
    cfg.mission.tow_limit_n = a.tow_limit
    cfg.mission.drag_n = a.drag
    app = App(cfg).start()
    print(f"GrappStar operator page: http://0.0.0.0:{a.port}/  ({'SIMULATOR' if a.sim else 'hardware'})")
    try:
        serve(app, host=a.host, port=a.port)
    except KeyboardInterrupt:
        pass
    finally:
        app.stop()
    return 0


def cmd_cameras(a) -> int:
    from .video.discover import find_cameras
    cams = find_cameras()
    if not cams:
        print("no cameras found (is the capture dongle / Kiyo plugged in? ls /dev/video*)")
        return 1
    for c in cams:
        print(f"{c.get('dev')}  role={c.get('role')}  name={c.get('name')}")
        if c.get("by_id"):
            print(f"    by-id: {c['by_id']}")
        for f in c.get("formats") or []:
            sizes = ", ".join(f"{s.get('width')}x{s.get('height')}@{'/'.join(str(x) for x in s.get('fps', []))}"
                              for s in (f.get("sizes") or [])[:6]) if isinstance(f, dict) else str(f)
            name = f.get("fourcc") if isinstance(f, dict) else ""
            print(f"    {name}: {sizes}")
    return 0


def cmd_snap(a) -> int:
    import cv2
    from .video.capture import V4L2Source
    src = V4L2Source(a.dev, fourcc=a.fourcc, width=_size(a.size)[0], height=_size(a.size)[1], fps=a.fps)
    os.makedirs(a.out, exist_ok=True)
    print(f"camera mode {getattr(src, 'info', '')}")
    n = 0
    try:
        while n < a.n:
            if a.interval <= 0:
                input(f"[{n + 1}/{a.n}] Enter to capture (move the board between shots) ")
            else:
                time.sleep(a.interval)
            f = None
            for _ in range(5):         # flush stale buffered frames
                f = src.read()
            if f is None:
                print("no frame")
                continue
            p = os.path.join(a.out, f"snap_{n:02d}.png")
            cv2.imwrite(p, f.img)
            print("saved", p)
            n += 1
    finally:
        src.close()
    return 0


def _base_client(a):
    from .comms.base import BaseClient
    c = BaseClient(a.port).start()
    if not c.wait_connected(3.0):
        print(f"cannot open {a.port}: {c.last_error or 'no link'}")
        print("check: ls -l /dev/grappstar-base /dev/ttyACM*; are you in the dialout group?")
        sys.exit(2)
    return c


def cmd_base(a) -> int:
    from .comms.base import BaseError, BaseTimeout
    c = _base_client(a)
    try:
        if a.what == "monitor":
            c.add_tel_callback(lambda t: print(json.dumps(t)))
            c.add_evt_callback(lambda e: print("EVENT", json.dumps({k: v for k, v in e.items() if k[0] != "_"})))
            while True:
                time.sleep(1)
        if a.what == "cmd":
            obj = json.loads(a.args[0])
            name = obj.pop("cmd")
            obj.pop("id", None)
            r = c.cmd(name, check=False, timeout=a.timeout, **obj)
        elif a.what == "mode":
            r = c.cmd("mode", to=a.args[0].upper(), check=False)
        elif a.what == "aim":
            r = c.cmd("aim", deg=float(a.args[0]), check=False)
        elif a.what == "reel":
            r = c.cmd("reel", duty=float(a.args[0]) if a.args else 0.5, until="stop", ms=1000, check=False)
        elif a.what == "status":
            r = c.cmd("status", check=False)
            if c.tel and "tel" not in r:
                r["tel"] = c.tel
        else:
            r = c.cmd(a.what, check=False, timeout=a.timeout)
        print(json.dumps(r, indent=1))
        return 0 if r.get("ok") else 1
    except BaseTimeout as e:
        print("timeout:", e)
        return 1
    except KeyboardInterrupt:
        return 0
    finally:
        c.close()


def cmd_dart(a) -> int:
    from .comms.dart import Dart, DartError
    d = Dart(a.host, timeout=a.timeout)
    try:
        if a.what == "status":
            print(json.dumps(d.status(), indent=1))
        elif a.what == "key":
            d.hold(a.c, a.hold)
            print("done (stopped)")
        elif a.what == "stop":
            print(d.stop())
        elif a.what == "auto":
            d.auto()
            print(d.wait_phase(("ANCHORED",), 90, log=print)["phase"])
        elif a.what == "release":
            d.release()
            print(d.wait_phase(("IDLE",), 90, log=print)["phase"])
        elif a.what == "flightlog":
            fl = d.flightlog()
            if a.out:
                with open(a.out, "w") as f:
                    json.dump(fl, f)
                print("saved", a.out, len(fl["samples"]), "samples")
            else:
                print(json.dumps({k: v for k, v in fl.items() if k != "samples"}), len(fl["samples"]), "samples")
        elif a.what == "watch":
            while True:
                s = d.status()
                print(f"{s['phase']:<10} {s.get('rail_V')} V {s.get('current_mA')} mA  {s.get('event')}")
                time.sleep(0.2)
        return 0
    except DartError as e:
        print(e)
        print("check: is this machine on the KEDGE WiFi? ping 192.168.4.1")
        return 2
    except KeyboardInterrupt:
        d.safe_stop()
        return 0


def cmd_mockbase(a) -> int:
    from .comms.mock_base import serve_pty
    mb = serve_pty(time_scale=a.scale)
    print(f"mock base on {mb.pty_path}")
    print(f"try: python3 -m grappstar base --port {mb.pty_path} status")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        mb.stop()
    return 0


def cmd_mockdart(a) -> int:
    from .comms.mock_dart import MockDart
    md = MockDart(port=a.port, host=a.bind).start()
    print(f"mock dart on http://{md.host}")
    print("type launch, land or quit (Enter after each)")
    try:
        while True:
            s = input("> ").strip()
            if s == "launch":
                md.launch()
            elif s == "land":
                md.land()
            elif s in ("quit", "q"):
                break
    except (KeyboardInterrupt, EOFError):
        pass
    md.stop()
    return 0


def cmd_calibrate(a, rest) -> int:
    from .vision import calibrate
    argv = list(rest)
    if argv and argv[0] == "mount":
        argv[0] = "ground"
    if argv and argv[0] in ("ground", "marker", "default") and "--out" not in argv:
        os.makedirs(os.path.dirname(DEFAULT_CALIB), exist_ok=True)
        argv += ["--out", DEFAULT_CALIB]
    if argv and argv[0] == "intrinsics" and "--out" not in argv:
        os.makedirs(os.path.dirname(DEFAULT_CALIB), exist_ok=True)
        argv += ["--out", os.path.join(os.path.dirname(DEFAULT_CALIB), "intrinsics.json")]
    return int(calibrate.main(argv) or 0)


def cmd_replay(a) -> int:
    from .replay import replay_run
    res = replay_run(a.folder, out=a.out, shot=a.shot)
    tr = res.get("terrain") or {}
    fl = res.get("flight") or {}
    print(json.dumps({"terrain": {k: tr.get(k) for k in ("cls", "score", "verdict", "reasons")},
                      "flight": {k: fl.get(k) for k in ("t_launch", "t_impact", "t_landed", "range_m")},
                      "files": res.get("files"), "errors": res.get("errors")}, indent=1))
    return 0


def cmd_solve(a) -> int:
    from .ballistics import Ballistics
    b = Ballistics.load(a.table)
    s = b.solve(a.range, a.drag, lob=a.lob)
    print(json.dumps(s.to_dict(), indent=1))
    return 0 if s.reachable else 1


def cmd_selftest(a) -> int:
    from .selftest import run
    return run(verbose=True)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python3 -m grappstar", description="GrappStar rover software")
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("serve", help="operator web page")
    s.add_argument("--sim", action="store_true", help="simulated base, dart and cameras")
    s.add_argument("--host", default="0.0.0.0")
    s.add_argument("--port", type=int, default=8080)
    s.add_argument("--base", default="/dev/grappstar-base")
    s.add_argument("--dart", default="192.168.4.1")
    s.add_argument("--rover-cam", default="/dev/grappstar-rovercam")
    s.add_argument("--dart-cam", default="/dev/grappstar-dartcam")
    s.add_argument("--fourcc", default="YUYV", choices=["YUYV", "MJPG"], help="dart capture format")
    s.add_argument("--rover-fourcc", default="MJPG", choices=["YUYV", "MJPG"])
    s.add_argument("--rover-size", default="1280x720")
    s.add_argument("--dart-size", default="720x480")
    s.add_argument("--deinterlace", default="none", choices=["none", "bob", "blend", "linear"])
    s.add_argument("--gst", action="store_true", help="capture through GStreamer (nvjpegdec)")
    s.add_argument("--token", help="require this token to open the page")
    s.add_argument("--calib", default=DEFAULT_CALIB)
    s.add_argument("--model", help="ONNX terrain classifier (optional)")
    s.add_argument("--ballistics", help="ballistics.json (default base/out/ballistics.json, else placeholder)")
    s.add_argument("--tow-limit", type=float, default=40.0, help="tow tension limit, N")
    s.add_argument("--drag", type=float, default=0.75, help="payout brake setting, N (C3 column)")
    s.add_argument("--runs", help="folder for run logs (default jetson/runs)")
    s.add_argument("--sim-landing", default="hard_ground")
    s.add_argument("--sim-seed", type=int, default=0)

    sub.add_parser("cameras", help="list cameras")

    s = sub.add_parser("snap", help="save frames from a camera")
    s.add_argument("--dev", default="/dev/grappstar-rovercam")
    s.add_argument("--fourcc", default="MJPG")
    s.add_argument("--size", default="1280x720")
    s.add_argument("--fps", type=int, default=15)
    s.add_argument("--n", type=int, default=15)
    s.add_argument("--interval", type=float, default=0.0, help="seconds between shots (0 = press Enter)")
    s.add_argument("--out", default=os.path.join(JETSON_DIR, "calib", "images"))

    s = sub.add_parser("base", help="talk to the base")
    s.add_argument("what", choices=["status", "ping", "monitor", "cmd", "mode", "aim", "stop", "estop", "clear",
                                    "wrap", "unwrap", "reel"])
    s.add_argument("args", nargs="*")
    s.add_argument("--port", "--base", dest="port", default="/dev/grappstar-base")
    s.add_argument("--timeout", type=float, default=2.0)

    s = sub.add_parser("dart", help="talk to the dart")
    s.add_argument("what", choices=["status", "key", "stop", "auto", "release", "flightlog", "watch"])
    s.add_argument("c", nargs="?", help="key: 1 ! 2 @ 3 # 4 $ (quote ! # $ in the shell)")
    s.add_argument("--hold", type=float, default=1.0)
    s.add_argument("--host", default="192.168.4.1")
    s.add_argument("--timeout", type=float, default=1.5)
    s.add_argument("--out")

    s = sub.add_parser("mockbase", help="simulated base on a pty")
    s.add_argument("--pty", action="store_true", default=True)
    s.add_argument("--scale", type=float, default=1.0)

    s = sub.add_parser("mockdart", help="simulated dart")
    s.add_argument("--port", type=int, default=8780)
    s.add_argument("--bind", default="127.0.0.1")

    sub.add_parser("calibrate", help="rover camera calibration (see calibrate --help)", add_help=False)

    s = sub.add_parser("replay", help="re-run analysis on a run folder")
    s.add_argument("folder")
    s.add_argument("--out")
    s.add_argument("--shot", type=int, default=None)

    s = sub.add_parser("solve", help="pitch for a range")
    s.add_argument("range", type=float)
    s.add_argument("--drag", type=float, default=0.75)
    s.add_argument("--lob", action="store_true")
    s.add_argument("--table")

    sub.add_parser("selftest", help="no-hardware checks")

    argv = sys.argv[1:] if argv is None else list(argv)
    if argv and argv[0] == "calibrate":
        return cmd_calibrate(None, argv[1:])
    a = ap.parse_args(argv)
    return {"serve": cmd_serve, "cameras": cmd_cameras, "snap": cmd_snap, "base": cmd_base, "dart": cmd_dart,
            "mockbase": cmd_mockbase, "mockdart": cmd_mockdart, "replay": cmd_replay, "solve": cmd_solve,
            "selftest": cmd_selftest}[a.cmd](a)
