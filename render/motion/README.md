# render/motion: motion data for the GrappStar animation

`motion.json` (about 5 MB, 1731 frames at 30 fps, 57.7 s) is written by

    cd render/motion
    ../../kedge/.venv/bin/python make_motion.py [--rerun-sim --node-path <dir with @dimforge/rapier3d-compat 0.14.0>]

about 80 s on 4 cores. It also writes `checks.json` (sanity checks) and `plots/*.png` (key curves, a posed scene check).
The sim trace `sim/sim_1021.json` is committed; `--rerun-sim` regenerates it byte for byte with node.

## Segments and where each comes from

| Segment | Source | Notes |
|---|---|---|
| Base station, every moving part | `base_motion.py`: the base's own joints, `kin()` and `line_path()` from `base/cad/parts` (drum, frame, launch, reel, sensors = the full D50 build), posed with `base/out/base/asm.json` (names, joint, place); joints checked equal to asm.json | Timeline in `tools.timeline` custom format with `TL._state_custom` (the base's eased state machine), numbers from `base/spec/params.py` TIMELINE, aim 34 deg instead of 36 (wiper 1067.8 deg backward, pitch 33.997) |
| Aim, cock | same | crank-rocker lifts the rail 6 -> 34 deg; wiper +425 deg draws the carriage back 240.9 mm, springs to 430 mm pin distance |
| The shot | `BaseKin.fire_state`: release table (gear -278.2 -> -281.25 on the tooth plateau), carriage from the `base/physics` launcher.py chain-spring trace (59.0 ms, 6.87 m/s), PLA stop overshoot (-17 mm, 7 ms), gear carried on through its gap to -360 (wiper 553.8 fwd) | dart rides the carriage, then slides 204 mm on the cradle at 6.8 m/s and leaves over the lip roller |
| Free flight | `dart_motion.Flight`: `kedge/flight34.py` forces (strip aero, axial drag, 0.25 N payout drag on the tail tie), v34 mass props, tip-off 0.4 rad/s nose down | 0.833 s, nose first (AoA max 16.5 deg), impact nose at X 4.77 |
| Landing, righting, anchoring, tow | `sim_dump.mjs`: `kedge/sim_righting/index.html` simCore + `v34_check/run34_shared.mjs` SimV34 cut at run time (unchanged), T_rigid_tie config (rigid axle, tail tie), loam seed 1021 | lands inverted, legFlip then podRoll:R, righted 8.3 s after impact, both drills in (41 / 44 mm), held 55 N in the sim; film shows the tow ramp to 15 N, hold, step to 20 N |
| Tether | `rope.py`: rope of 1 mm Dyneema (gravity, air drag, ground friction, fast-projection inextensibility) from the base exit to the tail knot, on the base path `reel.line_spans` + `launch.line_path` (spool, capstan wraps, eyelet, rail-rear pulley, rail, pusher hole, lip roller) | payout follows the dart (sag at 0.4 N), spool overrun 0.6 m after impact (base physics: 0.2 m at 0.75 N, LIGHT 0.25 N), take-up pulls it in, tight at the sim tension during the tow; line enters and leaves at the base end |

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
- `events`, `dart.handover_mismatch`, `sources`, `film_time_note` (the slow motion map). Keys are sparse: interpolate LINEAR.

## Film clock (baked in)

Real time to the shot; 20x for the release, stroke and slide; 3x in flight; 6x at the impact and bounce; real time from
1.2 s after impact through righting (8.3 s) and drilling (10.3 s). After landing the base's drum sequence is compressed
to fit inside the dart's righting and anchoring: rail low over the top of the aim cycle (D45) 1.33x, drum to NEUTRAL
2.67x, the 7-turn wrap 4.46x, the park 2.94x. Reel segment: tidy 1.33x, REEL engage 2.6x, tow 2x, then 2.5 s hold.
Events (frame): aim 120, cock 285, fire 513, muzzle exit 566, impact 717, leg flip 866, righted 1082, drills 1102,
anchored 1393, reel taut 1458, tow 1565, tension hold 1655, end 1730.

## Approximations

- Handover flight -> sim (the sim starts with the nose 90 mm up): sim seed at gamma 44.0 deg vs flight path 42.3 deg,
  speed 6.80 vs 6.36 m/s (0.48 m/s step), body pitch -44.0 vs -58.8 deg (slerped over the last 0.1 s), roll 154 deg in
  the sim (needed to land inverted) vs 0 in the planar flight: the dart rolls slowly over the flight (smootherstep from
  0.1 s), sim yaw -0.9 deg rotated onto the flight heading.
- Carriage trace uses the base physics default dart (279 g); the free dart leaves at 6.8 m/s (flight34, sim).
- The base's take-in during the tow is line stretch (EA 30 kN) plus the dart's give in the soil from the sim (7 mm).
- The line rests on z = 2 mm: drape it on the real terrain if the paddock between the base and the landing patch is not flat.
- Sim contact is soft with simple colliders: the real meshes sink up to 2.9 mm into the soil during the flip, 0.8 mm at rest.
