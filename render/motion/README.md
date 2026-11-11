# render/motion: motion data for the GrappStar animation

`motion.json` (about 5 MB, 1724 frames at 30 fps, 57.4 s) is written by

    cd render/motion
    ../../kedge/.venv/bin/python make_motion.py [--rerun-sim] --node-path <dir with @dimforge/rapier3d-compat 0.14.0>

about 90 s on 4 cores. It also writes `checks.json` (sanity checks) and `plots/*.png` (key curves, a posed scene check).
The sim trace `sim/sim_pick.json` is committed; it is rerun (byte for byte) whenever the flight's end state changes.
`search_landing.py` is the search that picked the launch inputs (results in `search/`).

## Segments and where each comes from

| Segment | Source | Notes |
|---|---|---|
| Base station, every moving part | `base_motion.py`: the base's own joints, `kin()` and `line_path()` from `base/cad/parts` (drum, frame, launch, reel, sensors = the full D50 build), posed with `base/out/base/asm.json` (names, joint, place); joints checked equal to asm.json | Timeline in `tools.timeline` custom format with `TL._state_custom` (the base's eased state machine), numbers from `base/spec/params.py` TIMELINE, aim at the picked rail pitch |
| Aim, cock | same | crank-rocker lifts the rail 6 -> 33.99 deg (wiper 1067.5 deg backward); wiper +425 deg draws the carriage back 240.9 mm, springs to 430 mm pin distance |
| The shot | `BaseKin.fire_state`: release table (gear -278.2 -> -281.25 on the tooth plateau), carriage from the `base/physics` launcher.py chain-spring trace (58.6 ms), then `carriage_stop_model`: carriage + spring mass on the PLA stop flexure (15 kN/m: 18.8 mm, about 285 N for the 2.7 J), rebound 9.2 mm, a second small hit, settled 53 ms after HOME; springs and their pin end points follow; gear carried on through its gap to -360 | dart rides the carriage, then slides 204 mm on the cradle at the muzzle speed and leaves over the lip roller; the tip-off attitude offset builds up over that slide |
| Free flight | `dart_motion.Flight3D`: `kedge/flight34.py` strip aero of both planes on the air-relative velocity (pitch: top-view widths, yaw: side-view widths), axial drag, gravity, 0.25 N payout drag on the tail tie, steady cross-wind; no roll mechanism in the model, so no roll | 0.842 s, nose first (angle of attack at most 12.4 deg), 24 mm lateral drift, impact nose at X 4.71 |
| Landing, bounce, righting, anchoring, tow | `sim_dump.mjs`: `kedge/sim_righting/index.html` simCore + `v34_check/run34_shared.mjs` SimV34 cut at run time (unchanged), T_rigid_tie config (rigid axle, tail tie), loam, STARTED FROM THE FLIGHT'S END STATE (pose, velocity, angular velocity; every body moved rigidly, the reel at the base's lip roller) when the dart's lowest point is 20 mm above the ground | no blend: handover position 0.0008 mm, rotation 0.0001 deg, angular velocity equal. Its own bounce leaves it inverted (tilt 174 deg); the leg alone rights it (legFlip, righted 6.9 s after contact, no reel tug); both drills in (41 / 44 mm); held 55 N in the sim; the film shows the tow ramp to 15 N, hold, step to 20 N |
| Tether | `rope.py`: rope of 1 mm Dyneema (gravity, quadratic air drag, ground friction, fast-projection inextensibility) from the base exit to the tail knot, on the base path `reel.line_spans` + `launch.line_path` (spool, capstan wraps, eyelet, rail-rear pulley, rail, pusher hole, lip roller) | storage spool: pulled by the line when tight, coasting against the 0.25 N LIGHT drag when slack (it stays tight in flight; the line bows behind the sweep with air drag); overrun after impact capped at 0.6 m (base physics 0.2 m at 0.75 N); take-up pulls it in; tight at the sim tension in the tow; line enters and leaves at the base end |

## Launch inputs (search_landing.py)

Searched uniformly: rail 33..35 deg (snapped to pitches the aim train reaches), muzzle 6.7..6.9 m/s, tip-off pitch rate
-1..1 rad/s and pitch offset -5..5 deg, yaw rate -1..1 rad/s and yaw offset -5..5 deg (flight34's disturbance range),
cross-wind -1.5..1.5 m/s, flat loam. 240 runs; qualifying = inverted or on a flank after its own tumble, the leg used,
righted, both drills in, 55 N held, no reel tug. Picked (seed 8, run 28): rail 33.987 commanded (33.9863 reached),
muzzle 6.783 m/s, tip-off -0.908 rad/s (nose down), +0.16 deg, yaw rate +0.368 rad/s, yaw offset +3.18 deg, cross-wind
0.645 m/s toward +Y, slope 0.

## motion.json beyond SPEC

- `objects["base:<asm part>"]`: world 4x4 as loc + quat per key (= `base_to_world @ JW(joint) @ place`, mm -> m), for
  every part that moves or sits off its place pose. Frame 0 is the load state = models.py rest pose.
  `base:launch_line_*` are the base's own tether stand-ins (hide them; `line` is the tether).
- `objects["dart:root"]`: the dart frame (nose tip origin, +x to the tail, +z up).
- `joints`: `dart:righting_leg`, `dart:drill_leg_L/R` (axis (0,-1,0) at their parts.json pos, deg = parts.json joint
  deg; couplers, deploy shaft, caps and motors follow the deploy angle), `dart:drill_leg_L/R_auger` (parent the pod,
  axis (-1,0,0) local through the pivot; L +deg, R -deg, both drill in, 45 rpm while drilling, 7.3 turns).
- `line`: 48 points per key, 12 on the base path (spool -> exit) + 36 free rope (exit -> knot). While the cup has
  laid the capstan turns, the main line skips them with a chord inside the drum and `line_wraps` gives the turns (64 points).
- `springs["launch_spring_l/r"]`: anchor pin and carriage pin centres per key (coil ends 12 mm inside each pin), for
  models.py `set_spring_pins(pin_c=carriage_pin, pin_a=anchor_pin)`.
- `events`, `dart.launch_inputs`, `dart.handover`, `sources`, `film_time_note`. Keys are sparse: interpolate LINEAR.

## Film clock (baked in)

Real time to the shot; 20x for the release, stroke, carriage stop and slide; 3x in flight; 6x at the impact and bounce;
real time from 1.2 s after impact through the righting and the drilling. After landing the base's drum sequence is
compressed to fit inside the dart's righting and anchoring: rail low over the top of the aim cycle (D45) 1.35x, drum to
NEUTRAL 2.69x, the 7-turn wrap 4.5x, the park 2.97x. Reel segment: tidy 1.33x, REEL engage 2.6x, tow 2x, 2.5 s hold.
Events (frame): aim 120, cock 285, fire 513, carriage home 548, muzzle exit 566, impact 715, leg flip 872, righted
1039, drills 1094, anchored 1386, reel taut 1435, tow 1558, tension hold 1648, end 1723.

## Approximations

- Carriage trace uses the base physics default dart (279 g); the free dart leaves at the picked muzzle speed.
- The stop flexure's restitution is assumed 0.25 (printed PLA); stiffness from the base's 19 mm / 285 N.
- Spool overrun after impact capped at 0.6 m (the base physics 0.2 m at 0.75 N, scaled to the LIGHT 0.25 N).
- The base's take-in during the tow is line stretch (EA 30 kN) plus the dart's give in the soil from the sim (7 mm).
- The line rests on z = 2 mm: drape it on the real terrain if the paddock between the base and the landing patch is not flat.
- Sim contact is soft with simple colliders: the real meshes sink up to 3.1 mm into the soil in the bounce, 0.2 mm at rest.
