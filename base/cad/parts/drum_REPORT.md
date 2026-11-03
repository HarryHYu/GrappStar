# Drum subsystem report (worker cad-drum), 3 Nov 2026

Status: draft v4 at 15:00 Sydney; final at 18:20. Design notes: `drum_NOTES.md` (v7, params v1.3.x). Firmware
table: `drum_table.json`.

## Done

- `drum.py`: about 34 printed and bought parts, joints, kin, allowed, clearances, audit_states (drum -4..352 at
  1 deg, wiper sweeps in every park, SW_INDEX sweep), grounded, steps (10 ordered steps with hardware).
  - Main axis:
    - coupler hub on the wiper shaft, outer coupler with the shear pin limiter (D8/D26), bus gear 50T (D25),
      SW_INDEX cam;
    - hex shaft with a d12 spigot to y 198.5;
    - REEL gear m2 50T (D20) on the coupler journal, retaining washer, dog collar;
    - LAUNCH dog ring bolted to the launch sleeve, magnet cap (face y 201.5).
  - Selector drum: drum A and drum B on a fixed axle; groove 1, clutch cam, segment gear with locking arc, Gray code
    bands.
  - Wrap idler (two journals) with a 42T output gear in the cup gear plane.
  - Compliant fork with a contact tab, dog contact post (SW_DOG_L/R), shift rail, brackets A (with the N20 ear) and
    B, two idler posts (post 2 also holds the index block).
  - N20 with its pinion (D bore), cradle (pivot and swing screw, SW_N20) and end plate.
  - Code switch block with 3 levers (drum back side); index block with its lever; hand lever.
- `_drum_kin.py`: angles, groove and clutch profiles, Gray code (A B C per params), stop plan, timing, torque budget
  with pressure-angle efficiency and fork racking, firmware table (`--json`).
- `_drum_test.py` (14 kinematics tests) and `_drum_geom_test.py` (30 geometry tests). `_drum_views.py`: quick
  renders and sections.

## Decisions made without Harry (recorded in NOTES)

- Drum order and zones per BASE_SPEC D16 and params v1.2.6, with these forced deviations:
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
- Whole-base fits (13:00 to 14:00):
  - the coupler stays under r 28.5 for y < 40 (cad-reel's wall), so the bus gear ring prints on tree support;
  - SW_INDEX moved to y 33.5..38.5 with its block on idler post 2 (the frame cheek is auto-carved round drum
    parts, so nothing of mine bolts to it);
  - code contacts on the drum back (cad-reel's pulley 2 post fills the front);
  - coupler hub follows cad-frame's wiper shaft model from the D48 boss face datum (params WIPER boss_face_y -10:
    output boss to y -6, round to y -4, flats to y 10, M8 thread to y 22): the hub reaches into the puck bore from
    y -5, D bore on 13.5 mm of flats, M8 nut at y 13..19.5 on the thread (final cross-check cad_power-1).
- Review rounds 5 and 6 (14:00 to 15:00):
  - no shaft retention screw (its hole was inside the frame cheek): the shaft is captured by the magnet cap and
    the launch sleeve;
  - fork sleeve 83 long, racking model with the far yoke pad (margin 2.34);
  - drum B keyed to drum A;
  - firmware rules tightened: unexpected code stops, short timeouts, boot cross-checks with the dog and arm
    switches (all 7 nonzero codes are valid zones, so a failed contact reads as a wrong zone);
  - unload jog 6 deg (the dog play is +-8);
  - brake track LIGHT across the whole LAUNCH zone (cliff in the unreachable gap);
  - N20 cradle: swing slot in the cradle, SW_N20 from the front;
  - assembly order fixed (drum A, idler, rail, coupler with the REEL gear, index block, fork, drum B, N20,
    contacts, shaft).
- Main axis angle: wiper_deg drives the shaft group; sleeve_deg the LAUNCH dog ring; reel_deg the REEL gear
  (params 1.2.3). launch_sleeve is only a fallback when S has no wiper_deg.
- Helpers are named `_drum_*.py`, because tools.assembly imports every non-underscore module as a subsystem.

## Verified

(test and audit outputs pasted at the final push)

## Requests to other workers (also sent to the coordinator)

- Coordinator (assembly stages): the drum idler group before cad-reel's bus gear, SW_ARM bracket and cup gear; the
  drum dog post before cad-reel's pulley 2.
- cad-reel: brake follower tip r <= 2 mm (the FIRM to LIGHT cliff is 3.5 deg from the -4 stop).
- cad-infra: meshcheck skip for meshes whose other part is not loaded (sent; my 3 cross meshes are on by default).

## Harry must measure

- Wiper output shaft: diameter, flats, length, thread (the coupler hub is parametric; the model follows cad-frame's
  guess).
- Worm N20 output shaft diameter and flat (modelled d3 with a 2.5 flat).
- N20 stall torque at 9 V: 50 mm lever and a kitchen scale. The 2x margin needs at least 0.171 N m.
- Shear pin break torque: lever and scale on a spare pin.
- Confirm a spare worm N20 for the drum exists (D19).
- AS5600 magnet, if any (SW_INDEX is the fallback).
