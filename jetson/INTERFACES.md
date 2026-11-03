# jetson/ internal interfaces (shared between modules)

Package `grappstar` (run as `python3 -m grappstar ...`). Python 3.10, numpy 1.26 (JetPack pins numpy<2), OpenCV 4.x,
pyserial. Nothing else is required; onnxruntime is optional (imported lazily). No em dashes anywhere.

Time base: every timestamp is `time.time()` wall seconds (float). Telemetry log lines, video frame sidecars and
flightlog alignment all use it, so they line up when replayed.

## Ownership (files)

| Path | Module |
|---|---|
| `grappstar/comms/base.py`, `mock_base.py`, `dart.py`, `mock_dart.py` | comms (contracts C1, C2) |
| `grappstar/video/` | capture, discovery, quality, recording, replay |
| `grappstar/sim/flight_video.py` | synthetic dart flight video (terrain map seen from a moving camera) |
| `grappstar/sim/terrain_textures.py` | procedural terrain textures per class |
| `grappstar/vision/flight.py`, `mosaic.py` | flight analysis, aerial map |
| `grappstar/vision/terrain.py`, `target.py` | landing zone verdict, rover targeting |
| `grappstar/ballistics.py`, `grappstar/data/ballistics_placeholder.json` | contract C3 |
| `grappstar/mission.py`, `app.py`, `rover.py`, `jlog.py`, `config.py`, `cli.py`, `__main__.py` | mission and glue |
| `grappstar/ui/` | operator web page |
| `setup/`, `README.md`, `requirements.txt` | install and bring-up |
| `tests/test_<module>.py` | one test file (or more) per module |

## video

```python
@dataclass
class Frame:
    img: np.ndarray          # BGR uint8, HxWx3
    t: float                 # time.time() at capture
    idx: int                 # frame counter from the source
    quality: Optional[float] = None   # 0..1 from video.quality (None = not evaluated)
    bad: bool = False        # True = snow / dropout / sync loss: skip for vision

class FrameSource:           # duck-typed
    name: str
    def read(self) -> Optional[Frame]: ...   # None = no frame (timeout or end of file)
    def close(self) -> None: ...
```

- `video.capture.V4L2Source(dev, fourcc="YUYV"|"MJPG", width, height, fps)`, `GstSource(pipeline)`,
  `video.replay.FileSource(path, realtime=False)` (reads `<path>` + sidecar `<path>.frames.jsonl`),
  `video.capture.ListSource(frames)` (in-memory, for tests and sim).
- `video.camera.Camera(name, source_factory, quality=True, deinterlace=False)`: background thread, reconnects,
  `latest() -> Optional[Frame]`, `jpeg() -> Optional[bytes]` (latest frame as JPEG for the MJPEG stream),
  `start_recording(path)`, `stop_recording() -> path`, `buffer(seconds) -> list[Frame]` (ring buffer of recent frames),
  `status() -> dict` (fps, connected, quality, recording).
- `video.quality.frame_quality(img) -> dict(score, snow, dropout, sync_loss, ...)`; score 0..1, `bad = score < 0.4`.
- `video.discover.find_cameras() -> list[dict(dev, by_id, name, formats)]`.

## sim

- `sim.terrain_textures.texture(cls, h, w, seed) -> BGR image` for each terrain class.
- `sim.flight_video.render_flight(seed=0, fps=30, landing_class="bedrock", size=(720,480), dropouts=True,
  noise=0.02, ...) -> (frames: list[Frame], truth: dict)`; truth has `t_launch, t_impact, t_landed, landing_class,
  landing_xy_world, flightlog` (a C2 flightlog dict consistent with the video), `world_map`.

## vision

- `vision.flight.analyze_flight(frames, flightlog=None) -> FlightResult` (`.phases` list of `(name, t0, t1)` with names
  ON_RAIL, LAUNCH, FLIGHT, IMPACT, LANDED; `.t_launch`, `.t_impact`, `.t_landed`, `.attitude` list of
  `(t, roll, pitch, yaw)` deg, `.horizon` list of per-frame `(t, angle_deg, sky_ratio)`, `.trajectory` list of
  `(t, x, y, z)` m, `.to_dict()`), `landing_frames(frames, result, n=15) -> list[Frame]`.
- `vision.terrain.analyze_landing(frames, model=None) -> TerrainReport` (`.cls`, `.probs`, `.slope_deg`, `.score`
  0..100, `.verdict` "GO"|"NO_GO"|"UNSURE", `.reasons` list[str], `.image` BGR report image, `.to_dict()`).
  Classes: `bedrock, hard_ground, loose_soil, sand_dust, vegetation, water_void, unknown`.
  `TerrainModel` interface: `predict(img) -> dict[cls, prob]`; `OnnxTerrainModel(path)`, `StubModel()`.
- `vision.mosaic.build_mosaic(frames, landing_t=None) -> MosaicResult` (`.image`, `.used`, `.rejected`,
  `.landing_xy` pixel in the map or None, `.to_dict()`).
- `vision.target`: `CameraModel(fx, fy, cx, cy, height_m, pitch_deg, roll_deg=0)` with `.pixel_to_ground(u, v) ->
  (range_m, bearing_deg)` (bearing positive = target is to the right), `.save(path)`, `CameraModel.load(path)`;
  `detect_orange(img) -> Optional[(u, v, area_px)]`; calibration helpers.

## ballistics (C3)

`Ballistics.load(path=None)` (default: `base/out/ballistics.json` if it exists, else the placeholder);
`.solve(range_m, drag_n=0.75, lob=None) -> Solution(reachable, pitch_deg, t_flight_s, line_out_m, apex_m, v_muzzle,
note)`. range_m is from the ground point under the rail pivot (D29); the App adds the camera's offset ahead of the
pivot (params.FRAMEWORK.camera pos x, 150 mm) to the camera range before solving (grappstar/campose.py). lob=None
prefers the higher pitch when two solutions exist (D29).

## app (what the UI talks to)

`app.App` owns base, dart, cameras ("rover", "dart"), mission, ballistics, target model.

- `app.status() -> dict`: JSON-serialisable snapshot: `base` (latest telemetry + link), `dart` (latest /status +
  link), `mission` (state, step, history, prompt or null, last error), `cams` (per camera status), `results`
  (flight, terrain, mosaic dicts or null), `target` (range_m, bearing_deg, solution or null), `t`.
- `app.action(name, args: dict) -> dict(ok: bool, msg: str, ...)`. Names: `preflight`, `start` (run the whole mission
  with operator confirmations), `answer` {prompt_id, choice}, `target_click` {cam, u, v} (u, v in 0..1 of the image),
  `target_auto`, `aim` {deg} or {range_m}, `stop`, `estop`, `clear`, `abort`, `neutral`, `jog` {motor, dir, ms},
  `dart_key` {c, ms}, `dart_stop`, `record` {on}.
- `app.camera(name) -> Camera or None` (for `/stream/<name>.mjpg` and `/snapshot/<name>.jpg`).
- `app.artifact(name) -> Optional[path]`: `landing.png`, `mosaic.png`, `flight.json`, `mission.jsonl`.

Mission prompts (operator confirmations) appear as `status()["mission"]["prompt"]` =
`{"id": "p7", "text": "...", "choices": ["Fire", "Cancel"], "danger": true}` and are answered with `answer`.
