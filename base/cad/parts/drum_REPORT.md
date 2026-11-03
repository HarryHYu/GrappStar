# Drum subsystem report (worker cad-drum), 3 Nov 2026

Status: draft at 12:40 Sydney; final at 18:30. Design notes: `drum_NOTES.md` (v5, params v1.2). Firmware table:
`drum_table.json`.

## Done

- `drum.py`: 33 printed and bought parts, joints, kin, allowed, clearances, audit_states (drum -4..352 at 1 deg,
  wiper sweeps in every park, SW_INDEX sweep), grounded, steps (10 ordered steps with hardware).
  - Main axis: coupler hub, outer coupler with the shear pin limiter (D8/D26), bus gear 50T (D25), SW_INDEX cam,
    hex shaft with a d12 spigot to y 198.5, REEL sleeve, REEL gear m2 50T (D20), dog collar, LAUNCH dog ring
    following launch_sleeve, magnet cap (face y 201.5).
  - Selector drum: drum A and drum B on a fixed axle; groove 1, clutch cam, segment gear with locking arc, Gray code
    bands.
  - Wrap idler (two journals) with a 42T output gear in the cup gear plane.
  - Compliant fork with a contact tab, dog contact post (SW_DOG_L/R), shift rail, brackets A (with the N20 ear) and
    B, two idler posts.
  - N20 with its pinion (D bore), cradle (pivot and swing screw, SW_N20) and end plate.
  - Code switch block with 3 levers; index block with its lever; hand lever.
- `_drum_kin.py`: angles, groove and clutch profiles, Gray code (A B C per params), stop plan, timing, torque budget
  with pressure-angle efficiency and fork racking, firmware table (`--json`).
- `_drum_test.py`: 14 PC tests. `_drum_views.py`: quick renders and sections.

## Decisions made without Harry (recorded in NOTES)

- Drum order and zones per BASE_SPEC D16 and params v1.2, with these forced deviations:
  - single-turn travel -4..352;
  - park angles 56 and 314, off the code edges;
  - 42T idler output for exactly 7 cup turns;
  - clutch ramps 30..40 and 302..312, to keep the 2x N20 margin at FIRM 5 N.
- N20 to drum 10:120 (m0.8): needed for 7 cup turns inside one drum turn with a 2x margin.
- Torque limiter as a printed shear pin: a printed friction limiter cannot hold its 500 N preload without creep.
- Drum split into two parts on a fixed axle, with 45 deg cones, so both print standing.
- Contacts: printed M3 screw head on M3 nut everywhere.
  - Code levers close at 80 percent of the bump, with a soft 1.6 mm beam so the overtravel force stays small.
  - SW_DOG_L/R close only within 0.5 mm of full engagement.
- Main axis angle: taken from cad-launch's launch_sleeve when that module is present, so there is one master angle on
  the axis.
- Helpers are named `_drum_*.py`, because tools.assembly imports every non-underscore module as a subsystem.

## Verified

(audit and test outputs pasted at the end of the day)

## Requests to other workers (also sent to the coordinator)

- cad-launch:
  - its parts sit 165 mm above the params v1.2 world frame (main axis at z 165 instead of 0);
  - launch_aim_housing needs a d12.5 bore for my spigot and must end before y 195 (my magnet cap);
  - AIM.kin_aim overrides launch_sleeve even when S gives wiper_deg;
  - sleeve flange lugs and screws per NOTES 2b.
- cad-reel:
  - reel_wall cuts through my main axis stack: bus gear tip r 39, REEL gear tip r 52, collar r 30, dog ring r 31;
  - reel_exit_post hits my coupler;
  - confirm the pushrod force at FIRM 5 N (I assume 15 N);
  - brake follower at drum L angle 180, cam base r 30, lift 4 mm.
- cad-frame: frame.py crashes on params v1.2 (`s_["origin"]` gets a float, frame.py:106).
- cad-infra: audit refine() explodes on fast gear-follower joints (N20 and idler spin thousands of degrees).
- coordinator: params DRUM targets WRAPPED 314 and the clutch ramps above.

## Harry must measure

- Wiper output shaft: diameter, flats or spline, length, thread (the coupler hub is parametric).
- Worm N20 output shaft diameter and flat (modelled d3 with a 2.5 flat).
- N20 stall torque at 9 V: 50 mm lever and a kitchen scale.
- Shear pin break torque: lever and scale on a spare pin.
- Confirm a spare worm N20 for the drum exists (D19).
- AS5600 magnet, if any (SW_INDEX is the fallback).
