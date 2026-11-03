# Drum subsystem report (worker cad-drum), 3 Nov 2026

Status: DRAFT (updated through the day; final at 18:30 Sydney). Design notes: `drum_NOTES.md`.
Firmware table: `drum_table.json`.

## Done

- `drum.py`: 31 parts, joints and kin per the cad-infra API v1.
  - Main axis: coupler hub, outer coupler with the D8 shear pin limiter and the D15 bus gear, shaft (hex 22 AF + d12
    spigot), REEL sleeve and REEL gear m2 40T, dog collar, LAUNCH dog ring, magnet cap.
  - Selector drum: drum A and drum B on a fixed axle; groove 1, brake track, segment gear and locking arc, Gray code
    bands.
  - Wrap idler and its output gear, compliant fork with the SW_DOG contact and its stop, shift rail, brackets A, B
    and idler, switch block with 3 lever contacts, N20 with its cradle, strap and pinion, hand lever.
  - Optional hooks: allowed, clearances, audit_states (drum -2..351 at 1 deg steps, plus wiper sweeps in every
    position), grounded, steps (10 ordered steps, hardware per step).
- `_drum_kin.py`: drum angle table, groove and brake profiles, Gray code, stop plan, timing, torque budget, firmware
  table. No CAD imports.
- `_drum_test.py`: 13 PC tests.
- `_drum_views.py`: quick-look renders and sections.

## Decisions made without Harry (all recorded in NOTES)

- Drum order per coordinator D16 (LAUNCH / NEUTRAL / WRAP / REEL), single turn, travel -2..351, Gray code on 3
  printed contacts.
- N20 to drum 10:120 (m0.8). Needed for 7 cup turns within one drum turn at the estimated N20 torque.
- D8 limiter as a printed shear pin (a printed friction limiter cannot hold the 500 N preload without creep).
- Drum split into two parts on a fixed printed axle so both print without supports.
- SW_DOG contact on the fork made REQUIRED (review round 1: a collar held engaged under load otherwise reads NEUTRAL).
- Helper modules named `_drum_*.py`: tools.assembly imports every non-underscore module in `cad/parts` as a subsystem.

## Verified

(to be filled with audit, test and review outputs)

## Requests to other workers

- cad-launch:
  - launch sleeve flange per NOTES 3b (3 lugs, 3 x M3 with nut traps, d12.35 spigot bushing);
  - two-way axial location of the launch sleeve.
- cad-reel:
  - cup ring 24T m1.5 at 49.6 from the idler axis, in the plane s 54..62;
  - brake follower at drum angle 200, s 41..52;
  - confirm a cup torque of 0.06 N m or less at LIGHT;
  - bus gear 40T m1.5 at s 1..9.
- cad-frame:
  - deck fixings per NOTES 11;
  - outboard bearing for the launch sleeve with two-way thrust;
  - AS5600 bracket facing s 166.
- cad-infra: skip modules in `cad/parts` that have no `parts()` (or document the underscore rule).
- fw-base: `drum_table.json` (positions, codes, edges, stop plan, timing, rules).
- coordinator: params.DRUM = dict(angles=dict(LAUNCH=0, NEUTRAL=41, WRAPPED=316, REEL=348.5)) so tools.timeline
  drives the drum through my angles (the placeholder 60/120/300 puts the drum in the middle of the wrap window).
  Any other DRUM keys named in `_drum_kin.DEFAULTS` override mine.

## Harry must measure

- Wiper output shaft: diameter, flats or spline, length, thread (the coupler hub is parametric).
- Worm N20 output shaft diameter and flat; N20 stall torque at 9 V (50 mm lever, kitchen scale).
- Shear pin break torque (bench, with a spare pin).
- Confirm a third worm N20 exists (kedge/CLAUDE.md says one more is to be bought for the dart).
- AS5600 magnet size.
