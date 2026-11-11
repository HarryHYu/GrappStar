# GrappStar mission animation in Blender: shared spec (read first)

Goal (Harry): a physically driven, photoreal-looking animation of OUR prototype (no rover): the real D50 base station
(launcher + reel + selector drum + wiper motor + electronics bay) and the real KEDGE v34 dart, on Harry's equestrian
property: uneven grass paddock, wood features. Motion fidelity matters most; image quality "HD, looks real", not 8K.
Rendered on Harry's Mac (M3, 16 GB, Blender 5.3): EEVEE for the full animation, Cycles switch for hero stills.
Here: bpy 5.3.0 in `render/.venv` (`render/.venv/bin/python script.py`), CPU only, 4 cores, ~5 GB free disk: keep
caches small, previews small (960x540, low samples).

## Conventions
- Blender 5.3, metres, Z up, 30 fps. World origin = centre of the base station's plywood board top surface.
- The launcher fires toward +X. The dart lands about 4.7 m downrange (x about 4.5 to 5.0 m). Ground near the base and
  in the landing patch (x 3.5 to 6.0, |y| < 1.0) is gently uneven but within +-2 cm of z 0 (the physics sims use flat
  ground there); elsewhere the paddock undulates freely.
- Object names: `base:<asm part name>` (base/out/base/asm.json names), `dart:<kedge part name>` (kedge/out34/parts.json),
  `elec:<name>` (Jetson etc. on the board), `env:<name>`, `cam:<name>`, `line:tether`.
- Collections: BASE, DART, ELEC, ENV, CAMERAS, LINE.

## Modules (each importable, each with a `__main__` that renders a preview to render/out/preview_<module>.png)
- `render/env.py`      -> `build_env(scene)`: terrain, grass, dirt, wood features, sky/HDRI, sun, world settings.
- `render/models.py`   -> `build_models(scene)`: every base + dart + electronics object at its REST pose with materials.
                          Rest pose: base parts at their asm.json "place" transform (mm -> m) relative to the base frame,
                          then the base frame placed on the board (models.py exposes BASE_TO_WORLD 4x4); dart parts posed
                          folded in the dart frame (nose tip origin, +x to the tail, +z up), parented to an empty
                          `dart:root` so motion moves the whole dart through one transform plus the moving sub-parts.
- `render/motion/`     -> scripts that write `render/motion/motion.json` (see below).
- `render/build.py`    -> integration: env + models + motion keyframes + line + cameras + render settings -> .blend.

## motion.json
{ "fps": 30, "frames": N, "film_time_note": "...",
  "objects": { "<object name>": {"frames": [f0, f1, ...], "loc": [[x,y,z],...], "quat": [[w,x,y,z],...]} , ... },
  "joints":  { "<object name>": {"frames": [...], "axis": [x,y,z], "pivot": [x,y,z] (in parent frame), "deg": [...] } },
  "line":    { "frames": [...], "points": [ [[x,y,z], ... K points], ... ] },
  "events":  [ {"frame": f, "name": "fire" }, ... ] }
World-space metres for `objects` and `line`. Base moving parts may be given as world transforms per frame (preferred:
computed from the base's own kinematics), dart sub-parts (pods, leg, augers spin) as joints relative to `dart:root`.

## Sequence (film time, about 50 to 60 s; slow-mo is baked into film time)
1. Establishing wide shot, idle (4 s).  2. Aim: rail to 34 deg (aim crank-rocker), ~5 s.  3. Cock: wiper turns, big
gear draws the carriage back 241 mm, springs stretch, ~6 s.  4. Fire: real launch 59 ms shown at ~20x slow-mo, line
pays off the storage reel.  5. Flight 4.7 m (real ~1 s, partly slowed), line trailing.  6. Impact + bounce + rest in
the grass.  7. Righting with the leg if it lands off its belly (use a sim seed that needs righting), ~8 s.
8. Pods deploy with drills spinning in (dirt), ~10 s.  9. Reel winds, line goes taut, tension holds, ~6 s.  10. End wide.

## Sources of truth
- Base CAD + kinematics: base/ (tools/assembly.py world(state), posed(); tools/timeline.py; tools/sim.py; base/out/base
  = full D50 build incl. electronics bay; spec/BASE_SPEC.md). The dart in base files is replaced by kedge v34.
- Dart CAD: kedge/out34 (parts.json + .npz per part; moving roots carry joint/sgn; see kedge/launcher_fit.py _posed()).
- Physics: launcher (base/physics, kedge launcher numbers: 12.1 J, stroke 241 mm in 59 ms, 6.8 m/s, 34 deg), flight
  (kedge/flight34.py), landing/righting/anchoring (kedge/sim_righting: index.html simCore, v34_check/run34_shared.mjs,
  node + @dimforge/rapier3d-compat in the scratchpad dphys/node_modules).
- No em dashes in any text. Commit to dart-v1 with the session trailer; never commit caches or frames.

## v2 direction (Harry, 11 Nov): supersedes the sequence and look above where they conflict
- Dart becomes KEDGE v35 (drill tips rake BACK toward the line, tent-peg style; pods hinge near x 50 and fold
  backward along the flanks). CAD lands in kedge/out35 (same parts.json + npz layout; kedge/out35/READY appears when
  final). Until then keep using out34 as a stand-in; every module takes the dart dir from one constant/env var
  (KEDGE_OUT, default kedge/out35 if READY exists else kedge/out34).
- Film = locked-off tripod cameras at a distance, no close-ups, no camera moves, no zooms. Goal: footage you cannot
  tell is rendered. Real time 1:1 everywhere (no slow-mo). Two static cameras only:
  cam:A wide (sees aim, cock, fire, flight, landing, righting, pods/drills start), then ONE hard cut to
  cam:B (static, roughly side-on to the line, a few metres off it) for the reel winding and the line going taut and
  holding. Time between the two may be compressed at that cut only (hides the long drilling).
- Setting: rural AUSTRALIAN equestrian paddock in summer: mostly dry straw-coloured grass with some green at the base,
  tussocks, bare and hoof-trodden dirt patches, scattered gum leaves, bark strips, twigs, sticks, small rocks, the odd
  dried horse manure pile; gum trees (eucalyptus) and timber post + wire fencing behind; harsh clear sun, hazy
  distance. Sulphur-crested cockatoos in the background (some on the ground far off that take off at the fire event,
  maybe a pair flying across). Subtle wind sway in grass and leaves.
- Materials: printed parts follow the build: PLA = dark blue (default for every printed part), PPA-CF = matte black
  carbon-fibre nylon (parts marked PPA-CF: base/out/base/asm.json "material", dart couplers per kedge pack). Visible
  FDM layer lines (0.2 mm, in each part's print orientation where known) and slight print imperfections. Bought parts
  real (steel, PCBs, Jetson Orin Nano dev kit, wiper motor). Dyneema line yellow.
- Output: "super HD": final 3840x2160 EEVEE on Harry's Mac (preset uhd), 1920x1080 preset final for speed. Real
  camera look: AgX, 180 deg shutter motion blur, light film grain, mild lens distortion/vignetting, sensible DOF.
