# render/motion: motion data for the GrappStar animation (v2: real time)

`motion.json` (about 5 MB, 1432 frames at 30 fps, 47.7 s) is written by

    cd render/motion
    ../../kedge/.venv/bin/python make_motion.py [--rerun-sim] --node-path <dir with @dimforge/rapier3d-compat 0.14.0>

about 3 min on 4 cores. It also writes `checks.json` (sanity checks) and `plots/*.png` (key curves, posed scene check).
The dart build comes from `dart_motion.OUT`: `$KEDGE_OUT`, else `kedge/out35` once `kedge/out35/READY` exists, else
`kedge/out34`. The sim trace `sim/sim_pick_<build>.json` is committed and rerun whenever the flight's end state
changes. `search_landing.py` searches the launch inputs for a build (results in `search/`), `PICKS` in make_motion.py
holds the pick per build.

## Film clock: real time 1:1, one cut

Frame f shows real time f / 30 s up to the `cut` event (frame 732, 24.4 s: cam:A, static wide), then real time
58.0 + (f - 732) / 30 s (cam:B, static, side-on to the line). The cut skips 33.6 s: the drilling to the anchors
(anchored at 30.1 s real) and most of the base's 32 s wrap. `cut`, `real_time_of_frame`, `events[].real_time` and
`events_hidden_by_cut` are in motion.json. Around the shot and the impact some keys sit on quarter frames (320.25 ...)
for motion blur; key them at those fractional frames. Keys are sparse: interpolate LINEAR, never across the cut.

| real s | what | source |
|---|---|---|
| 0-3 | idle | |
| 3.0-7.4 | aim: wiper 1067.5 deg backward, rail 6 -> 33.99 deg | wiper as a DC motor: 45 rpm no load, 15 N m stall, 1.2 N m aim train load, 60 ms spin-up, braked stop |
| 7.9-9.8 | cock: wiper +425 deg against the springs, carriage back 241 mm | same motor against the spring torque through the big gear |
| 10.6-10.7 | the shot: release 60 ms after the wiper restarts, 58.6 ms stroke, carriage 18.8 mm into the stop flexure, 9.2 mm rebound, settled 53 ms; springs follow | release table, base/physics launcher.py trace, carriage_stop_model |
| 10.7-11.6 | flight 0.84 s, impact nose X 4.71 | Flight3D: kedge/flight34.py strip aero in both planes, tail tie drag, cross-wind |
| 11.6- | bounce, inverted, leg flip (13.0), righted 18.6, pods deploy, drills from 20.4 | righting sim (kedge/sim_righting SimV34, v34 tail tie, loam) started FROM the flight's end state, no blend |
| 12.4-22.8 | base: rail low over the top of the aim cycle (34 -> 47 -> 6 deg, D45) | wiper backward at the aim load |
| 23.4- | unload jog, drum to NEUTRAL (5 s), then the wrap 41 -> 302 deg (32 s, cup 7 turns) | drum N20 8.1 deg/s |
| cut | | |
| 58.0-62.8 | last wrap turns, park at 314 | |
| 63.3 | tidy (take-up, wiper duty 0.6 against the FIRM clutch) until tight; line rises off the grass | the wrap took the slack, the line is already near tight at the LIGHT 0.25 N |
| 65.3 | REEL fly-in (drum 314 -> 347, 4.1 s, wiper creep) | |
| 69.8-76.3 | tow: capstan takes in 7 mm while the tension ramps to 20 N (sim tow) | line stretch EA 30 kN + the dart's give in the soil |
| 76.3-81.3 | tension holds (worm), end | |

Events (frame): aim 90, cock 237, fire 320, muzzle exit 323, impact 348, rail low 372, leg flip 389, righted 556,
drills 611, cut 732, wrap done 822, park 875, tidy 890, reel taut 892, REEL 949, tow 1086, tension hold 1281, end 1431.

## Tether

`rope.py`: 1 mm Dyneema rope (gravity, quadratic air drag, ground and base-deck contact, fast-projection
inextensibility) from the base exit to the tail knot, on the base path (`reel.line_spans` + `launch.line_path`).
After the launch the line runs along the cradle to the funnel mouth and wraps over the muzzle lip roller whenever it
leaves below the pusher ray to the roller top (the base's own rule). The total line out is conserved: it changes only
by the spool (pulled out against the LIGHT drag, coasting 0.6 m after impact) and the take-up; the capstan wraps draw
the slack from the dart side. Checked: no free rope point inside any base part, none below the ground.

## Launch inputs (search_landing.py)

Rail 33..35 deg (reachable pitches), muzzle 6.7..6.9 m/s, tip-off pitch rate -1..1 rad/s and offset -5..5 deg, yaw
rate -1..1 rad/s and offset -5..5 deg, cross-wind -1.5..1.5 m/s, flat loam; qualifying = inverted or on a flank after
its own tumble, the leg used, righted, both drills in, 55 N held, no reel tug. out34 pick (seed 8, run 28): rail 33.987
commanded (33.9863 reached), muzzle 6.783 m/s, tip-off -0.908 rad/s nose down and +0.16 deg, yaw rate +0.368 rad/s and
+3.18 deg, cross-wind 0.645 m/s toward +Y, slope 0.

## motion.json beyond SPEC

- `objects["base:<asm part>"]`: world loc + quat per key (= `base_to_world @ JW(joint) @ place`, mm -> m); frame 0 is
  the load state = models.py rest pose. `base:launch_line_*` are the base's tether stand-ins (hide them).
- `objects["dart:root"]`: the dart frame (nose tip origin, +x to the tail, +z up).
- `joints`: `dart:righting_leg`, `dart:drill_leg_L/R` (axis (0, sgn, 0) at their parts.json pos, deg = parts.json
  joint deg; the rest of each pod follows), `dart:drill_leg_L/R_auger` (parent the pod, axis = the auger's own axis
  toward its tip, deg already carries the drill-in sign).
- `line` (48 points: 12 base path + 36 free rope), `line_wraps` (capstan turns), `springs` (pin centres per key).
- `dart.launch_inputs`, `dart.handover` (0.0008 mm, 0.0001 deg), `dart.sim_job`, `sources`.

## Approximations

- Wiper motor numbers are params assumptions (WIPER measured: False); aim train load 1.2 N m assumed.
- Carriage trace uses the base physics default dart (279 g); the stop flexure restitution is assumed 0.25.
- Spool overrun after impact capped at 0.6 m (base physics 0.2 m at 0.75 N, scaled to the LIGHT 0.25 N).
- The dart is still (its sim state held) between the anchors and the tow; the tow shows the sim's tension ramp.
- Sim contact is soft with simple colliders: the real meshes sink up to 3.3 mm into the soil in the bounce, 0.2 mm at rest.
