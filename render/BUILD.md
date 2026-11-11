# GrappStar mission animation: the integrated scene (render/build.py)

`render/build.py` builds the whole mission in a fresh Blender session from the current `env.py`, `models.py` and
`motion/motion.json` and saves **`render/out/grappstar_mission.blend`** (textures packed, relative paths, zstd
compressed; opens in Blender 5.3 on the Mac). Nothing in the .blend needs Python: no drivers, no handlers, no
auto-run scripts. Everything that moves is keyframes, geometry nodes or constraints-free cameras.

## Rerun

    render/.venv/bin/python render/build.py                    # final quality, saves the .blend (about 1 min)
    render/.venv/bin/python render/build.py --quality preview  # lighter grass (draft | preview | final | hero)
    render/.venv/bin/python render/build.py --no-save          # build only (e.g. with --shots)
    render/.venv/bin/python render/build.py --shots all        # + Cycles CPU 640x360 still per shot -> out/shot_NN_name.png
                                                               #   and out/shots_sheet.png (--samples 48, --still 4:560)
    render/.venv/bin/python render/build.py --check-only --motion-check   # 320x180 EEVEE preview of every 2nd frame
                                                               #   of the saved .blend -> out/motion_check.mp4
    render/.venv/bin/python render/build.py --no-save --frames 540,560 [--cam "x,y,z:tx,ty,tz:lens"]
                                                               # debug stills -> out/frames/check_FFFF.png

A rerun always picks up the latest models.py and motion.json (frame count, events, objects, joints and line are
all read at run time; shot boundaries follow the motion.json events; joint objects resolve through models.py
DART_JOINTS / DART_JOINT_ALIASES; the augers, their pod hinges, the dart length and CG station are taken from
motion.json joints, the meshes and "dart"), so a revised dart (other pod hinges, other drill directions) drops in
with a rerun as long as models.py and motion.json use the same object names. `out/build_report.json` lists what was keyed,
the shot frame ranges and the .blend size. Motion-check frames are cached in `out/frames/mc/` (gitignored, resumable:
delete the folder after a motion change).

## What is in the scene

| Collection | Content |
|---|---|
| ENV | env.py paddock: terrain, GN grass (`env:grass`), fences, gate, logs, bale, trough, HDRI world, sun |
| BASE, DART, ELEC | models.py: D50 base on the plywood board, KEDGE v34 dart (`dart:root` + joint objects), Jetson kit |
| LINE | `line:tether` (1 mm Dyneema braid, models.mat_rope) and `line:wraps` (the capstan turns) |
| FX | soil crumbs and heaps (`fx:soil_crumbs`), grass press empties (`fx:press_*`) |
| CAMERAS | `cam:01_establishing` .. `cam:11_end_wide`, switched by timeline markers bound to them; `ev:*` markers mark the motion.json events |

**Motion.** Every object in motion.json "objects" is keyed from its world transform, converted to its parent's
frame (base parts sit under `base:root` = BASE_TO_WORLD), location + quaternion, LINEAR, with bulk keyframing
(`keyframe_points.add` + `foreach_set`); one-key objects are just posed. Dart joints (righting leg, both pods, both
augers) are axis-angle keys on the joint objects (angle only, so the 7 auger turns interpolate exactly). The two
launch springs are restretched per key with models.py `set_spring_pins()` (geometry-nodes coil length keyed).
The base's own tether stand-ins `base:launch_line_*` are hidden. Keys are dense where the motion is fast (the
59 ms shot is keyed per film frame at 20x slow motion), so motion blur samples sub-frames correctly.

**Tether.** motion.json "line" (48 points per key: 12 on the base path, 36 free) is resampled (the free part 3x,
through the same points), led over the muzzle lip roller where motion.json lets the slack line fall straight from
the rail exit through the rail (see open points), draped onto the real ground (terrain, board and deck tops found by ray casts, rope
centre 1 mm above) and stored, all keys, as vertices of the `line:tether` mesh with a per-vertex key-frame
attribute. Its geometry-nodes modifier picks the two keys around the current (sub)frame, interpolates, and builds
the curve (base path poly, free rope Catmull-Rom) with a 1 mm round profile and the rope material. `line:wraps`
does the same for "line_wraps" and appears when the cup has laid the turns. Pure geometry nodes: renders headless,
motion blur works, no handlers.

**Grass.** `env:grass` keeps its env.py scatter; build.py adds to its node tree: a "Density" input (0..1, random
thinning: the EEVEE speed knob), a "Mow" input (blade height factor in soft-edged grazed-short patches round the
landing spot and the base, default 0.42, 0.22 within 0.45 m of the rest spot, so the 16 cm dart reads in the
grass), and the dart pressing the grass: `fx:press_0..4` ride along the dart's body and flatten the clumps under
it into flat mats (pushed a little outward) whenever it is within a few cm of the ground; `fx:press_impact` and
`fx:press_lie` keep the grass partly flattened where it hit and lay.
A 5 cm disc under the rest position is grass-free (the pressed clumps around it hide the edge); the grass is
trampled short round both drill cuts.

**Soil.** `fx:soil_crumbs` is a point cloud whose geometry nodes compute each crumb's ballistic path from its
start time, velocity and gravity (in film frames, so the impact burst plays at the 6x slow motion) and instance
small crumb meshes (FX_SRC): a burst at the impact, crumbs thrown out of both drill holes while the augers screw
in, and soil heaped along each drill's cut (the auger crosses the surface along a short track while its pod swings
down) that grows in as the cut passes. No popping: every crumb scales in over a few frames.

## Cameras and shots (30 fps; frames from the current motion.json)

| Shot | Camera | Frames | What | Timing from motion.json events |
|---|---|---|---|---|
| 01 | cam:01_establishing | 0-119 | wide, slow push in over the paddock, base right, downrange left | start .. aim_start |
| 02 | cam:02_aim | 120-284 | 3/4 close-up from +y: crank-rocker lifts the rail 6 -> 34 deg | aim_start .. cock_start |
| 03 | cam:03_cock | 285-494 | gear close-up craning up over the rail: wiper turns, big gear draws the carriage back, springs stretch | cock_start .. fire - 18 |
| 04 | cam:04_fire | 495-599 | side view, pans with the dart off the rail at 20x slow motion, line paying out | fire - 18 .. muzzle_exit + 34 |
| 05 | cam:05_flight | 600-702 | drone-like follow behind and beside the dart, line trailing back to the base | .. impact - 12 |
| 06 | cam:06_impact | 703-863 | low in the grass at the landing patch: impact (6x slow motion), bounce, comes to rest inverted | .. legFlip - 8 |
| 07 | cam:07_righting | 864-1044 | medium shot from above: the leg flips the dart upright | .. max(righted + 6, pods deploy - 25) |
| 08 | cam:08_drilling | 1045-1239 | macro on the right pod: pod swings down, auger screws in, soil crumbs and heaps | .. drills start + 150 |
| 09 | cam:09_anchoring | 1240-1395 | 3/4 front view of the dart: both drills going in until anchored | .. anchored + 10 |
| 10 | cam:10_reel_taut | 1396-1642 | behind the base: capstan and spool, then pulls back to look down the line as it lifts taut and tows | .. tension_hold - 5 |
| 11 | cam:11_end_wide | 1643-1723 | wide crane up and back: base, taut line, dart in the paddock | .. end |

All action reads right to left (cameras on the +y side of the launch line). Camera moves are eased (smootherstep)
and keyed every frame; DOF focus distance is keyed on the action (dart CG, gears, drill entry); cuts only at the
markers. Motion blur on (shutter 0.5 frame).

## Render on the Mac

Copy `render/out/grappstar_mission.blend` and `render/mac/` (`render_mac.command`, `preset.py`) into
`~/Desktop/grappstar_render/` and double-click `render_mac.command` (details in render/mac/README_MAC.md): EEVEE
1920x1080 PNG frames, resumable, then an MP4. Quick test: `bash render_mac.command preview eevee 700 760`. Cycles
hero stills: `bash render_mac.command hero cycles 560 560`. Speed knobs in the .blend: `env:grass` > Modifiers >
scatter > Density (1.0 full, 0.5 half the clumps), Mow; render samples in preset.py.

## Open points

- motion.json's free line starts at the rail exit (pusher hole) and, once the dart is out, falls straight down
  through the rail and the base (up to 33 cm below the rail line; 2 cm during the tow). build.py leads it along the
  rail and over the lip roller instead (`reroute_lip`, blended by dip depth, no pops; rope length is not
  conserved). Better: render-motion adds the lip roller to the rope's base path.
- The dart's tumble in the sim sinks it up to 1.7 mm into the terrain (hidden by the grass); at rest 0.1 mm.
- Grass has no collision: pressed clumps under the dart are flattened, but a blade from an unpressed clump can
  still cross the dart's edge in a close-up.
- The launch springs are keyed and stretch, but the rail guards hide most of them from the shot 03 camera.
- The 1 mm line is sub-pixel in the wide shots (realistic); it reads in shots 04 to 10.
- EEVEE in software here takes about 14 s per 320x180 frame (and minutes for the first shader compile), so the
  motion check is Cycles 8 spp, grass Density 0.5, every 2nd frame. On the Mac the first EEVEE frame compiles
  shaders (a few minutes), then it is fast.
- The .blend is about 150 MB (packed textures 110 MB), so it is not committed: rebuild it with build.py.
