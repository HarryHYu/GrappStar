# Worker `physics`: report (draft, updated through the day)

Branch `w/physics`. Directory `base/physics/` (code) and `base/out/physics/`, `base/out/ballistics.json` (outputs).
Run everything: `cd base/physics && ./run_all.sh` (pytest, every model, ballistics table, summary PDF).

## Done

| File | What |
|---|---|
| common.py | every assumption in one place: battery sag, wiper (linear, one block), A228 extension spring (rate, hook and body limits, mass, surge), line, dart, 608, sprag, shaft |
| spec_bridge.py | overlays base/spec/params.py onto the defaults (only file that knows spec names) |
| launcher.py | spring launcher: chain (surge) and lumped spring models, pulley ratio, friction, reel inertia, stop impact, coil clash |
| flight.py | dart point mass with drag + bead-chain tether (line weight, cross-flow and skin drag, ground contact), ideal brake or coupled reel |
| payout.py | storage reel geometry and inertia, slip clutch (constant torque), centrifugal shoes, spin-up, overrun, bird's nest risk |
| capstan.py | Euler-Eytelwein wraps, constant-torque take-up, tow force/speed/current, firmware trip, part stresses |
| drivetrain.py | wiper torque/current/time/energy per function (aim, cock, reel, tow) vs stall, BTS7960, fuse |
| printed_gears.py | Lewis (printed allowables), key (steel key in printed shaft), shaft torsion (Kt 1.6 / 2 / 3), 608, sprag |
| ballistics.py | base/out/ballistics.json exactly per contract C3, with validate() |
| sensitivity.py | range swing for each uncertain input |
| calib.py | hang test fit (rate, initial tension) and wiper block from bench numbers |
| summary.py | SUMMARY.md + base/out/physics/physics_summary.pdf (one page, Liberation Sans, black) |
| FINDINGS.md | problems for the spec (pushed early, kept current) |

## Numbers

(filled at 18:30 from the final run)

## Reviews

Round 1 (three fresh reviewers, each recomputed the numbers independently):
- launch/spring: all Shigley numbers and both launch models reproduced; fixed: stop load now includes the spring KE
  (1.2 kN on a 4 mm pad), chain end node mass, eye gain on the reel inertia, spring length is over the hooks.
- flight/ballistics: ranges within 0.3 % of an independent model; fixed: reel start speed (line draw rate, not dart
  speed: low-pitch snap of 4.5 N was an artefact), time step (tension flip), reel peak filtered, stricter C3 validate.
- payout/capstan/drivetrain: all key numbers reproduced; fixed: payout JSON crash, key failure mode (plastic behind the
  keyway at ~7 N m), Lame hoop stress, constant-torque take-up, firmware trip, wiper inertia.

## Open items / requests

- Coordinator: dart mass 0.279 (spec) vs 0.292 (brief). Spec LAUNCH/AIM/REEL values still None: defaults follow
  cad-launch NOTES (stroke 245, preload 5 mm, carriage 80 g, pitch 6 to 47).

## Harry must measure

See SUMMARY.md (spring hang test first: every range number scales with it).
