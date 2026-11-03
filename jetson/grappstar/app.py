"""App: owns the base link, the dart link, both cameras, the mission and the targeting. The operator page (ui/server.py)
talks only to this object: status(), action(name, args), camera(name), artifact(name).

Real hardware:  App(AppConfig(base_port="/dev/grappstar-base", dart_host="192.168.4.1", ...)).start()
Simulator:      App(AppConfig(sim=True)).start()
Nothing here blocks or crashes when a device is missing: links retry in the background and show as down on the page.
"""
from __future__ import annotations

import json
import os
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, Optional

from .ballistics import Ballistics
from .comms.base import BaseClient, BaseDisconnected, BaseError, BaseTimeout
from .comms.dart import Dart, DartError, DartPoller
from .jlog import JLog
from .mission import STEPS, Mission, MissionConfig
from .rover import OperatorRover, RoverDrive

HERE = os.path.dirname(os.path.abspath(__file__))
JETSON_DIR = os.path.dirname(HERE)


@dataclass
class AppConfig:
    sim: bool = False
    base_port: str = "/dev/grappstar-base"
    dart_host: str = "192.168.4.1"
    rover_cam: str = "/dev/grappstar-rovercam"
    dart_cam: str = "/dev/grappstar-dartcam"
    rover_fourcc: str = "MJPG"          # Kiyo X: MJPG for 720p at 30 fps (YUYV is limited to low fps at 720p)
    dart_fourcc: str = "YUYV"           # capture dongle: YUYV avoids the JetPack JPEG library mismatch
    rover_size: tuple = (1280, 720)
    dart_size: tuple = (720, 480)
    rover_fps: int = 15
    dart_fps: int = 30
    dart_deinterlace: str = "none"
    gstreamer: bool = False
    runs_dir: str = os.path.join(JETSON_DIR, "runs")
    calib_path: str = os.path.join(JETSON_DIR, "calib", "rovercam.json")
    ballistics_path: Optional[str] = None
    ui_token: str = ""
    sim_seed: int = 0
    sim_landing: str = "hard_ground"
    sim_time_scale: float = 1.0
    terrain_model: Optional[str] = None   # path to an ONNX classifier (optional)
    mission: MissionConfig = field(default_factory=MissionConfig)


class App:
    def __init__(self, cfg: Optional[AppConfig] = None, rover: Optional[RoverDrive] = None):
        self.cfg = cfg or AppConfig()
        self.ui_token = self.cfg.ui_token
        self.rover = rover or OperatorRover()
        self.lock = threading.RLock()
        os.makedirs(self.cfg.runs_dir, exist_ok=True)
        self.syslog = JLog(os.path.join(self.cfg.runs_dir, "system.jsonl"), echo=print)
        self.mlog: Optional[JLog] = None
        self.run_dir: Optional[str] = None
        self.sim = None
        self.bal = Ballistics.load(self.cfg.ballistics_path)
        self.target: Optional[dict] = None
        self.results: Dict[str, Any] = {}
        self.files: Dict[str, str] = {}
        self.mission: Optional[Mission] = None
        self.cams: Dict[str, Any] = {}
        self.cam_model = None
        self.terrain_model = None

    # ------------------------------------------------ setup
    def start(self) -> "App":
        c = self.cfg
        if c.sim:
            from .sim.world import SimWorld
            self.sim = SimWorld(seed=c.sim_seed, landing_class=c.sim_landing, time_scale=c.sim_time_scale)
            self.base = self.sim.base_client()
            self.dart = Dart(self.sim.dart_host, timeout=1.0)
        else:
            self.base = BaseClient(c.base_port)
            self.dart = Dart(c.dart_host, timeout=1.0)
        self.base.on_line = self._base_line
        self.base.start()
        self.dart_poller = DartPoller(self.dart).start()
        self._load_cam_model()
        self._start_cameras()
        if c.terrain_model:
            try:
                from .vision.terrain import OnnxTerrainModel
                self.terrain_model = OnnxTerrainModel(c.terrain_model)
            except Exception as e:
                self.syslog.warn(f"terrain model not loaded: {e}")
        self.syslog.info(f"app started ({'SIMULATOR' if c.sim else 'hardware'}), ballistics {self.bal.source}")
        return self

    def _load_cam_model(self) -> None:
        from .sim.world import default_rover_model
        if self.sim:
            self.cam_model = self.sim.rover_model
            return
        try:
            from .vision.target import CameraModel
            self.cam_model = CameraModel.load(self.cfg.calib_path)
        except Exception:
            self.cam_model = default_rover_model(*self.cfg.rover_size)
            self.syslog.warn("rover camera not calibrated: using default guesses (python3 -m grappstar calibrate)")

    def _start_cameras(self) -> None:
        from .video.camera import Camera
        c = self.cfg
        if self.sim:
            self.cams["dart"] = Camera("dart", self.sim.dart_source, buffer_s=15)
            self.cams["rover"] = Camera("rover", self.sim.rover_source, buffer_s=2, quality=False)
            return
        from .video import capture

        def factory(dev, fourcc, size, fps):
            def make():
                if c.gstreamer and capture.has_gstreamer():
                    p = (capture.gst_pipeline_mjpeg(dev, size[0], size[1], fps) if fourcc == "MJPG"
                         else capture.gst_pipeline_yuyv(dev, size[0], size[1], fps))
                    return capture.GstSource(p)
                return capture.V4L2Source(dev, fourcc=fourcc, width=size[0], height=size[1], fps=fps)
            return make
        self.cams["dart"] = Camera("dart", factory(c.dart_cam, c.dart_fourcc, c.dart_size, c.dart_fps),
                                   deinterlace=c.dart_deinterlace, buffer_s=15)
        # rover camera: small buffer (1080p frames are big), quality check off (it is not an analog link)
        self.cams["rover"] = Camera("rover", factory(c.rover_cam, c.rover_fourcc, c.rover_size, c.rover_fps),
                                    buffer_s=2, quality=False, max_fps=c.rover_fps)

    def stop(self) -> None:
        if self.mission:
            self.mission.abort("app stop")
        for cam in self.cams.values():
            try:
                cam.stop()
            except Exception:
                pass
        try:
            self.dart_poller.stop()
            self.base.close()
        except Exception:
            pass
        if self.sim:
            self.sim.close()

    def _base_line(self, direction: str, obj: dict) -> None:
        lg = self.mlog
        if lg is not None:
            lg.base_line(direction, obj)

    # ------------------------------------------------ UI interface
    def status(self) -> dict:
        tel = self.base.tel or {}
        ds = self.dart.last or {}
        m = self.mission.status() if self.mission else {"state": "IDLE", "step": "", "steps": STEPS, "history": [],
                                                       "prompt": None, "error": "", "run_dir": None}
        log_tail = (self.mlog.tail if self.mlog else self.syslog.tail)[-12:]
        return {
            "t": time.time(),
            "sim": bool(self.sim),
            "base": dict(tel, link=self.base.link()),
            "dart": dict(ds, link=self.dart.link()),
            "mission": m,
            "cams": {k: v.status() for k, v in self.cams.items()},
            "results": {"flight": self.results.get("flight"), "terrain": self.results.get("terrain"),
                        "mosaic": self.results.get("mosaic"), "errors": self.results.get("errors"),
                        "files": sorted(self.files)},
            "target": self.target,
            "log": log_tail,
            "config": {"drag_n": self.cfg.mission.drag_n, "tow_limit_n": self.cfg.mission.tow_limit_n,
                       "ballistics": os.path.basename(self.bal.source), "placeholder": self.bal.placeholder,
                       "cam_uncalibrated": bool(getattr(self.cam_model, "uncalibrated", True))},
        }

    def camera(self, name: str):
        return self.cams.get(name)

    def artifact(self, name: str) -> Optional[str]:
        if name == "mission.jsonl":
            p = self.mlog.path if self.mlog else self.syslog.path
            return p if p and os.path.exists(p) else None
        p = self.files.get(name)
        return p if p and os.path.exists(p) else None

    def action(self, name: str, args: Optional[dict] = None) -> dict:
        args = args or {}
        fn = getattr(self, "act_" + name, None)
        if fn is None:
            return {"ok": False, "msg": f"unknown action {name}"}
        try:
            r = fn(**args)
            return r if isinstance(r, dict) else {"ok": True, "msg": str(r or "ok")}
        except TypeError as e:
            return {"ok": False, "msg": f"bad arguments for {name}: {e}"}
        except BaseError as e:
            return {"ok": False, "msg": f"base refused: {e.err}"}
        except (BaseTimeout, BaseDisconnected, DartError) as e:
            return {"ok": False, "msg": str(e)}

    # ------------------------------------------------ actions
    def _busy(self) -> bool:
        return bool(self.mission and self.mission.running())

    def _new_run(self) -> str:
        d = os.path.join(self.cfg.runs_dir, time.strftime("%Y%m%d-%H%M%S"))
        os.makedirs(d, exist_ok=True)
        self.run_dir = d
        self.mlog = JLog(os.path.join(d, "mission.jsonl"), echo=print)
        self.dart.on_line = self.mlog.dart_line
        self.mlog.info(f"run folder {d}")
        print(f"mission log folder: {d}")
        return d

    def _on_results(self, res: dict) -> None:
        self.results = res
        self.files.update(res.get("files", {}))

    def _make_mission(self) -> Mission:
        d = self._new_run()
        tm = self.terrain_model

        def analyse(frames, fl, outdir, **kw):
            from .analysis import run_post_flight
            return run_post_flight(frames, fl, outdir, terrain_model=tm, **kw)
        return Mission(self.base, self.dart, self.cams, self.bal, self.mlog, cfg=self.cfg.mission, rover=self.rover,
                       get_target=lambda: self.target, run_dir=d, analyse=analyse, on_results=self._on_results)

    def act_start(self) -> dict:
        if self._busy():
            return {"ok": False, "msg": "mission already running"}
        if self.sim and not self.sim.wait_ready(0):
            return {"ok": False, "msg": "simulator still rendering the flight, try again in a few seconds"}
        self.mission = self._make_mission()
        self.mission.start()
        return {"ok": True, "msg": "mission started", "run_dir": self.run_dir}

    def act_preflight(self) -> dict:
        """Run only the preflight checks (no motion)."""
        if self._busy():
            return {"ok": False, "msg": "mission running"}
        m = Mission(self.base, self.dart, self.cams, self.bal, self.mlog or self.syslog, cfg=self.cfg.mission,
                    get_target=lambda: self.target, analyse=lambda *a, **k: {})
        m.ask = lambda text, choices, **k: choices[0]   # warnings are reported, not asked
        try:
            m.preflight()
            return {"ok": True, "msg": "preflight ok"}
        except Exception as e:
            return {"ok": False, "msg": f"preflight: {e}"}

    def act_answer(self, prompt_id: str, choice: str) -> dict:
        if not self.mission:
            return {"ok": False, "msg": "no mission"}
        ok = self.mission.answer(prompt_id, choice)
        return {"ok": ok, "msg": "ok" if ok else "that prompt is no longer open"}

    def act_stop(self) -> dict:
        if self.mission:
            self.mission.abort("operator STOP")
        ok_b = self.base.safe_stop()
        ok_d = self.dart.safe_stop()
        log = self.mlog or self.syslog
        log.warn(f"STOP: base {'ok' if ok_b else 'NO REPLY'}, dart {'ok' if ok_d else 'NO REPLY'}")
        return {"ok": ok_b, "msg": f"STOP sent: base {'ok' if ok_b else 'NO REPLY'}, dart {'ok' if ok_d else 'NO REPLY'}"}

    def act_estop(self) -> dict:
        if self.mission:
            self.mission.abort("operator ESTOP")
        self.dart.safe_stop()
        try:
            self.base.estop(timeout=0.5)
            return {"ok": True, "msg": "ESTOP: wiper off, drum to NEUTRAL"}
        except Exception as e:
            return {"ok": False, "msg": f"ESTOP not acknowledged: {e}"}

    def act_abort(self) -> dict:
        return self.act_stop()

    def act_clear(self) -> dict:
        self.base.clear()
        return {"ok": True, "msg": "fault cleared"}

    def act_neutral(self) -> dict:
        if self._busy():
            return {"ok": False, "msg": "mission running"}
        self.base.mode("NEUTRAL")
        return {"ok": True, "msg": "drum to NEUTRAL"}

    def act_mode(self, to: str) -> dict:
        if self._busy():
            return {"ok": False, "msg": "mission running"}
        self.base.mode(to)
        return {"ok": True, "msg": f"drum to {to}"}

    def act_jog(self, motor: str, dir: int, ms: int = 300) -> dict:
        if self._busy():
            return {"ok": False, "msg": "mission running"}
        self.base.jog(motor, int(dir), min(int(ms), 500))
        return {"ok": True, "msg": "jog"}

    def act_dart_key(self, c: str, ms: int = 350) -> dict:
        if self._busy():
            return {"ok": False, "msg": "mission running"}
        self.dart.key(c, min(int(ms), 400))
        return {"ok": True, "msg": "key"}

    def act_dart_stop(self) -> dict:
        ok = self.dart.safe_stop()
        return {"ok": ok, "msg": "dart stop" if ok else "dart not answering"}

    def act_set(self, key: str, val: Any) -> dict:
        """Mission settings from the page: drag_n, tow_limit_n, check_n."""
        if key in ("drag_n", "tow_limit_n", "check_n", "retrieve_limit_n"):
            v = float(val)
            if key.endswith("_n") and key != "drag_n" and not (1 <= v <= 100):
                return {"ok": False, "msg": "limit must be 1 to 100 N"}
            setattr(self.cfg.mission, key, v)
            if self.target:
                self._update_solution()
            return {"ok": True, "msg": f"{key} = {v}"}
        self.base.set(key, val)
        return {"ok": True, "msg": f"base {key} = {val}"}

    def act_record(self, on: bool = True) -> dict:
        if on:
            d = self.run_dir or self._new_run()
            for n, cam in self.cams.items():
                cam.start_recording(os.path.join(d, f"{n}_manual_{time.strftime('%H%M%S')}.avi"))
            return {"ok": True, "msg": f"recording to {d}"}
        paths = [cam.stop_recording() for cam in self.cams.values()]
        return {"ok": True, "msg": "recording stopped", "files": [p for p in paths if p]}

    def _image_size(self, cam: str):
        c = self.cams.get(cam)
        f = c.latest() if c else None
        if f is None:
            return None
        h, w = f.img.shape[:2]
        return (w, h)

    def _set_target(self, u: float, v: float, size, source: str) -> dict:
        from .vision.target import target_info
        info = target_info(self.cam_model, u, v, image_size=size)
        info.update(u=u / size[0], v=v / size[1], cam="rover", source=source, t=time.time())
        self.target = info
        self._update_solution()
        return self.target

    def _update_solution(self) -> None:
        t = self.target
        if t and t.get("range_m"):
            s = self.bal.solve(t["range_m"], self.cfg.mission.drag_n)
            t["solution"] = s.to_dict()
            t["reachable"] = s.reachable
            t["pitch_deg"] = s.pitch_deg

    def act_target_click(self, u: float, v: float, cam: str = "rover") -> dict:
        if cam != "rover":
            return {"ok": False, "msg": "targets are picked on the rover view"}
        size = self._image_size("rover") or tuple(self.cfg.rover_size)
        t = self._set_target(float(u) * size[0], float(v) * size[1], size, "click")
        return {"ok": bool(t.get("ok")), "msg": t.get("msg", ""), "target": t}

    def act_target_auto(self) -> dict:
        from .vision.target import detect_orange
        c = self.cams.get("rover")
        f = c.latest() if c else None
        if f is None:
            return {"ok": False, "msg": "no rover video"}
        d = detect_orange(f.img)
        if d is None:
            return {"ok": False, "msg": "no orange marker found"}
        h, w = f.img.shape[:2]
        t = self._set_target(d[0], d[1], (w, h), "orange marker")
        return {"ok": bool(t.get("ok")), "msg": t.get("msg", ""), "target": t}

    def act_target_range(self, range_m: float, bearing_deg: float = 0.0) -> dict:
        """Type a range by hand (for example measured with a tape)."""
        self.target = {"ok": True, "range_m": float(range_m), "bearing_deg": float(bearing_deg), "source": "manual",
                       "msg": f"manual target {float(range_m):.1f} m", "t": time.time()}
        self._update_solution()
        return {"ok": True, "msg": self.target["msg"], "target": self.target}

    def act_aim(self, deg: Optional[float] = None, range_m: Optional[float] = None) -> dict:
        if self._busy():
            return {"ok": False, "msg": "mission running"}
        if deg is None and range_m is not None:
            s = self.bal.solve(float(range_m), self.cfg.mission.drag_n)
            if not s.reachable:
                return {"ok": False, "msg": s.note}
            deg = s.pitch_deg
        if deg is None:
            return {"ok": False, "msg": "give deg or range_m"}
        self.base.aim(float(deg))
        return {"ok": True, "msg": f"aiming to {float(deg):.1f} deg"}

    def act_analyse(self) -> dict:
        """Re-run the landing analysis on the dart video buffer (manual use)."""
        from .analysis import run_post_flight
        cam = self.cams.get("dart")
        frames = cam.buffer() if cam else []
        try:
            fl = self.dart.flightlog()
        except DartError:
            fl = None
        d = self.run_dir or self._new_run()
        res = run_post_flight(frames, fl, d, terrain_model=self.terrain_model)
        self._on_results(res)
        tr = res.get("terrain") or {}
        return {"ok": True, "msg": f"{tr.get('cls')} {tr.get('score')} {tr.get('verdict')}"}
