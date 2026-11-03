# Worker `physics`: report

Branch `w/physics`. Code in `base/physics/`, outputs in `base/out/physics/` and `base/out/ballistics.json` (+ meta).
Run everything: `cd base/physics && ./run_all.sh` (pytest, every model, ballistics table, summary PDF, plots; about 3 to 5 min).
Every number runs on `base/spec/params.py` through `spec_bridge.py` (the only file that knows spec names).

## Done

| File | What |
|---|---|
| common.py | every assumption in one place: battery sag, wiper (one block, linear, 5 N m limiter), A228 extension spring (rate, hook and body limits, mass, surge), line, dart, 608, sprag, shaft, launcher/reel/capstan defaults |
| spec_bridge.py | overlays base/spec/params.py (v1.2) onto the defaults, lists what it used in `p.spec_used` |
| launcher.py | spring launcher: chain (surge) and lumped spring models, pulley ratio, friction, reel inertia in the stroke, stop impact, coil clash |
| flight.py | dart point mass with drag + bead-chain tether (line weight, cross-flow and skin drag, ground contact), ideal brake or coupled reel; spec geometry (pivot pulley, release point), range from the pivot ground point |
| payout.py | storage reel geometry and inertia, slip clutch (constant torque, set as line tension with the reel full), centrifugal shoes, spin-up, overrun, bird's nest risk |
| capstan.py | Euler-Eytelwein wraps, constant-torque take-up with the 2.5 bus overdrive, tow force/speed/current, limiter, firmware trips, part stresses |
| drivetrain.py | wiper torque/current/time/energy per function (aim worm + crank-rocker, cock big gear + rack, reel, tow) vs stall, limiter, BTS7960, fuse |
| printed_gears.py | Lewis (printed allowables), steel key in a printed shaft, shaft torsion (Kt 1.6 / 2 / 3), 608, sprag |
| attitude.py | pitch-plane 3-DOF dart attitude at 6 to 8 m/s (stability with and without the tether) |
| recoil.py | recoil on the rover at the shot, rock and tip-over, 2 vs 4 springs |
| retrieve.py | taut vs slack line at the shot, wrap loads on the cup and drum N20, final approach onto the cradle |
| mission.py | time and battery budget of one mission |
| ballistics.py | base/out/ballistics.json exactly per contract C3, validate(), wind sensitivity in the meta |
| sensitivity.py | range swing for each uncertain input |
| calib.py | hang test fit (rate, initial tension) and wiper block from bench numbers |
| summary.py, plots.py | SUMMARY.md + base/out/physics/physics_summary.pdf (one page, Liberation Sans, black) + two plain plots |
| FINDINGS.md | problems and numbers for the spec (pushed from 11:15, kept current) |

## Numbers (spec v1.2)

See FINDINGS.md "Current numbers" and SUMMARY.md (final values pasted below at the 18:20 push).

## Tests

(pasted at the final push)

## Reviews (fresh subagents, each recomputed the numbers independently; every confirmed finding fixed)

1. Round 1, three reviewers.
   - launch/spring: Shigley numbers and both launch models reproduced. Fixed: the stop load now includes the spring KE,
     the chain end-node mass, the eye gain on the reel inertia, and the spring length is taken over the hooks.
   - flight/ballistics: within 0.3 % of an independent model. Fixed: the reel start speed (the 4.5 N snap at low pitch
     was an artefact), the time-step tension flip, the filtered reel peak, and a stricter C3 validate().
   - payout/capstan/drivetrain: numbers reproduced. Fixed: the payout JSON crash, the key failure mode (plastic behind
     the keyway, about 7 N m), Lame hoop stress, the constant-torque take-up, the firmware trips and the wiper inertia.
2. Round 2: the aim wiper torque was 4x too low, now fixed. Also fixed: sprag #1 torque taken on the wiper axis, full
   shot time, summary layout and font, sensitivity run on the chain model. attitude.py verdict robust to Cd and tether point.
3. Round 3: the recoil rock was overstated, because the stop pulse cancels most of the stroke pulse: 5 deg, not 17.
   retrieve and limiter numbers reproduced.
4. Round 4 (spec v1.0): mapping confirmed, Jetson solver loads the table. Fixed: the capstan gear (50 T), the overdrive
   now read from the spec, both clutch settings referenced to the full reel, and a complete spec_used.
5. Cross-worker: module names launch.py and gears.py clashed with cad-launch / cad lib in a combined pytest run. Renamed
   them to launcher.py and printed_gears.py, and the tests to test_phys_*.py. conftest.py pins this directory's modules
   and fails if a test writes this worker's outputs or kedge/out30.
6. Round 5 (robustness): fixed a 3.12-only f-string (blocked Python 3.10), an infinite loop on --step 0, calib
   tracebacks, --help missing on four CLIs, odd flags overwriting committed outputs, and stale docs. A test now
   guards 3.10 syntax.
7. Blind cross-check: a fresh agent with no access to the code got the same muzzle speed (7.77 vs 7.78 m/s). Its longer
   range came from a taut/slack chatter in its own script.
8. Catalogue check (web): the spring is Century C-243, 1.37 mm wire, 38.4 N / 302 mm. This led to D30 (265 mm full cock).
9. Adversarial critic: found the line slack at the shot with the D14 pivot guide (snatch, about -0.5 m), which led to
   D36 (rear guide). Also a heavy CAD carriage, line-path friction (eye gain now 1.35), carriage shoe couple friction
   (now modelled) and terrain height (impact angles in the meta).
10. Coordinator review F (round 1): mostly already fixed. Added tests on the REAL spec and a content-hash staleness
   test for ballistics.json.
11. Round 6: corrected the take-up drag (D25: clutch disk at 2.5 x wiper). The tow model offset is now -3.0 N, not
   -10.4. Also: flight eye at the D36 guide, taut detection from the peak slack, trips from the spec.
12. Coordinator review F2 (round 2): tow limit cap check (72 N available vs the 70 N D39 cap), validate() and end
   rows in the staleness test, and the spec headline kept within 10 % of the model by a test.

## Open items / requests

- Coordinator: clutch convention (LIGHT and FIRM are line tension with the reel full) into the spec text. The final
  approach is fine on FIRM 5 N (D24, margin 1.5 to 1.9). The rover mass, wheelbase and deck height
  are assumed (4 kg, 0.30 m, 0.15 m).
- Jetson: range_m is from the pivot ground point; the camera is 150 mm ahead of the pivot, so add the offset. Clamp
  to AIM.launch_pitch_range or ask me to narrow the table.
- fw-base: trips (50 ms blank; 30 ms hard trips at 5 A in aim, 6 A in REEL, 6.5 A cocking; tow trip at measured free current + 3.5 A for 50 N over 200 ms: line = 17 x (I - I_free) - 10.4 N at FIRM 5 N), and
  sample the BTS7960 IS pin in the PWM on-time.

## Harry must measure

1. Spring hang test (first: every range number scales with it): hang 1, 2, 3 kg from one spring, measure the length,
   then run `python calib.py spring 165.1 1:<mm> 2:<mm> 3:<mm>` and put the result in spec SPRING.
2. Wiper: free speed (turns in 30 s at 12.0 V), free current, stall torque (arm + luggage scale, 1 s only), stall
   current. `python calib.py wiper ...` gives the block.
3. Dart mass with ballast and tether knot (0.279 kg assumed).
4. Slip clutch: luggage scale on the line with the reel full: LIGHT 0.5 N, FIRM 5 N.
5. Rail friction (tilt test) and the drum N20 torque.
6. Rover: mass, wheelbase, deck height (recoil and launch height).
