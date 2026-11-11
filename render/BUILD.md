# GrappStar mission animation: the integrated scene (render/build.py)

`render/build.py` builds the whole mission in a fresh Blender session from the current `env.py`, `models.py` and
`motion/motion.json` and saves **`render/out/grappstar_mission.blend`** (textures packed, relative paths, zstd
compressed; opens in Blender 5.3 on the Mac). Nothing in the .blend needs Python: no drivers, no handlers, no
auto-run scripts. Everything that moves is keyframes, geometry nodes or constraints-free cameras.

## Rerun

    render/.venv/bin/python render/build.py                    # final quality, saves the .blend (about 1 min)
    render/.venv/bin/python render/build.py --quality preview  # lighter grass (draft | preview | final | hero)
    render/.venv/bin/python render/build.py --no-save          # build only (e.g. with --shots)
    render/.venv/bin/python render/build.py --shots all        # + Cycles CPU 960x540 stills at each camera's key
                                                               #   moments -> out/shot_A1_aim.png .. shot_B4_end.png
                                                               #   and out/shots_sheet.png (--shots A,B / A3_launch)
    render/.venv/bin/python render/build.py --check-only --motion-check   # 480x270 Cycles preview of every frame of
                                                               #   the saved .blend -> out/motion_check.mp4
    render/.venv/bin/python render/build.py --no-save --frames 540,560 [--cam "x,y,z:tx,ty,tz:lens"]
                                                               # debug stills -> out/frames/check_FFFF.png

A rerun always picks up the latest models.py and motion.json (frame count, events, objects, joints and line are
all read at run time; the camera cut and the stills follow the motion.json events; joint objects resolve through models.py
DART_JOINTS / DART_JOINT_ALIASES; the augers, their pod hinges, the dart length and CG station are taken from
motion.json joints, the meshes and "dart"), so a revised dart (other pod hinges, other drill directions) drops in
with a rerun as long as models.py and motion.json use the same object names. `out/build_report.json` lists what was keyed,
the cameras (position, lens, frames, stills) and the .blend size. Motion-check frames are cached in `out/frames/mc/` (gitignored, resumable:
delete the folder after a motion change).

## What is in the scene

| Collection | Content |
|---|---|
| ENV | env.py paddock: terrain, GN grass (`env:grass`), fencing, trees, props, sky, sun, cockatoos (whatever env.py builds) |
| BASE, DART, ELEC | models.py: D50 base on the plywood board, KEDGE dart (`dart:root` + joint objects; v35 once models.py switches), Jetson kit |
| LINE | `line:tether` (1 mm Dyneema braid, models.mat_rope) and `line:wraps` (the capstan turns) |
| FX | soil crumbs and heaps (`fx:soil_crumbs`), grass press empties (`fx:press_*`) |
| CAMERAS | `cam:A` (wide) and `cam:B` (the line), switched by the timeline marker bound to each; `ev:*` markers mark the motion.json events |

**Motion.** Every object in motion.json "objects" is keyed from its world transform, converted to its parent's
frame (base parts sit under `base:root` = BASE_TO_WORLD), location + quaternion, LINEAR, with bulk keyframing
(`keyframe_points.add` + `foreach_set`); one-key objects are just posed. Dart joints (righting leg, both pods, both
augers) are axis-angle keys on the joint objects (angle only, so the 7 auger turns interpolate exactly). The two
launch springs are restretched per key with models.py `set_spring_pins()` (geometry-nodes coil length keyed).
The base's own tether stand-ins `base:launch_line_*` are hidden. Keys are linear and as dense as motion.json
gives them, so the 180 deg motion blur samples sub-frames correctly (in real time the 59 ms shot spans 2 frames).

**Tether.** motion.json "line" (48 points per key: 12 on the base path, 36 free) is resampled (the free part 3x,
through the same points), draped onto the real ground (terrain, board and deck tops found by ray casts, rope
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
trampled short round both drill cuts. If env.py's grass tree changes shape, build.py skips these extras with a
warning (build_report.json "grass_mods_error") instead of failing.

**Soil.** `fx:soil_crumbs` is a point cloud whose geometry nodes compute each crumb's ballistic path from its
start time, velocity and gravity (in film frames, so the impact burst plays at the 6x slow motion) and instance
small crumb meshes (FX_SRC): a burst at the impact, crumbs thrown out of both drill holes while the augers screw
in (until the cut), and soil heaped along each drill's cut (the auger crosses the surface along a short track while its pod swings
down) that grows in as the cut passes. No popping: every crumb scales in over a few frames.

## Cameras (SPEC v2: locked-off tripods, real time, one cut)

| Camera | Frames | Placement | Shows |
|---|---|---|---|
| cam:A | 0 .. cut - 1 | wide, a slight front 3/4 from the -y side, 9.5 m from the base-landing midpoint, 1.5 m high | aim, cock, fire, flight, landing, righting, pods and drills starting |
| cam:B | cut .. end | side-on to the line from the -y side, a little toward the base: abreast of 1.9 m down the line, 5.2 m off it, 1.7 m high (tall tripod: the line is seen against the ground just beyond it), lens fitted within 24-35 mm | reel winding, the line lifting taut and holding |

Both cameras sit on the -y side (CAM_SIDE): the env's trees, fence and cockatoos are behind the action and its sun
is to the cameras' right, so the base's faces toward the camera are sunlit; the launch reads left to right.

The cut is motion.json's `cut` (frame 713 now; fallback `tidy_start`). Nothing is interpolated across it: the last
key before the cut is CONSTANT on every F-curve (objects, joints, springs, presses), the line cache holds its
pre-cut key up to the cut, crumbs and soil heaps are settled and complete after it, and the shutter opens at the
frame (motion_blur_position START, 180 deg), so frame 712 never sees post-cut poses and 713 never sees pre-cut
ones. Quarter-frame keys from motion.json are keyed at their fractional frames. Neither camera moves or zooms: build.py places
each one by these few parameters (CAM_A, CAM_B at the top of the camera section) and FITS its aim and focal length
to the action from motion.json (base board, the dart's flight arc and rest spot for A; the base, the line at its
last key and the dart for B), so a new dart, landing spot or timing reframes itself on a rerun (with the current
data: A about 50 mm, B about 30 mm). DOF is real (f/5.6, focus on the action's middle: everything that matters is
sharp at these distances). TRIPOD_SHAKE adds a sub-pixel F-curve noise to the camera rotation (a tripod in a
breeze; set 0 to remove).

**Camera look.** AgX (env.set_render), 180 deg shutter motion blur (shutter 0.5 frame), and the compositor node
group `GS_camera_look`: mild barrel distortion with a touch of lateral chromatic aberration, a soft vignette and a
light per-frame film grain (LOOK at the top of the camera section). The .blend is set to 3840x2160 EEVEE.

**Key-moment stills** (`--shots`): cam:A aim, cock, launch, flight apex, impact, righting, drills; cam:B after the
cut, taut, tow, end; frames are stored on the cameras (custom property `stills`) and printed on the contact sheet.

**Cockatoos.** If env.py has `add_cockatoos(scene, fire_frame)`, build.py calls it with the motion.json fire frame
(318 now): the ground birds behind the action take off 0.2 to 0.8 s after it.

## Render on the Mac

Copy `render/out/grappstar_mission.blend` and `render/mac/` (`render_mac.command`, `preset.py`) into
`~/Desktop/grappstar_render/` and double-click `render_mac.command` (details in render/mac/README_MAC.md): EEVEE
3840x2160 PNG frames (preset uhd, the default), resumable, then an MP4. `bash render_mac.command final` renders
1920x1080. Quick test: `bash render_mac.command preview eevee 500 560`. Cycles hero stills:
`bash render_mac.command hero cycles 520 520`. Rough time per frame on an M3 16 GB: preview 3-6 s, final 8-15 s,
uhd 25-45 s, plus a few minutes of shader compiling on the first frame. Speed knobs in the .blend: `env:grass` > Modifiers >
scatter > Density (1.0 full, 0.5 half the clumps), Mow; render samples in preset.py.

## Open points

- The dart's tumble in the sim sinks it up to 1.7 mm into the terrain (hidden by the grass); at rest 0.1 mm.
- Grass has no collision: pressed clumps under the dart are flattened, but a blade from an unpressed clump can
  still cross the dart's edge in a close-up.
- The 1 mm line is sub-pixel in cam:A (realistic). In cam:B at 3840x2160 it reads as a 1-2 px yellow line all the
  way from the base to the dart (native crops `--crop-test B:taut` -> render/out/crop_B_taut_1..3.png, 5.4 to
  6.0 m away); on straw-coloured grass its contrast will be lower than on the current green test paddock.
- EEVEE in software here takes about 14 s per 320x180 frame (and minutes for the first shader compile), so the
  motion check is Cycles 6 spp, 480x270, grass Density 0.5, every frame (the real-time shot spans 2 frames).
  On the Mac the first EEVEE frame compiles shaders (a few minutes), then it is fast.
- The .blend is about 150 MB (packed textures 110 MB), so it is not committed: rebuild it with build.py.
