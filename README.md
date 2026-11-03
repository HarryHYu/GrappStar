# GrappStar

A rover fires an anchor dart (KEDGE) on a Dyneema tether. The dart films its flight and landing from a nose camera,
screws itself into the ground, and the rover winches itself out against it. Then the dart lets go and is reeled home.
One 12 V wiper motor runs the whole base station (launch, aim and reel); a small N20 selector drum swaps which system
it drives, in a fixed physical order: LAUNCH, NEUTRAL, WRAP, REEL.

Online: base station site https://claude.ai/artifact/5DbSKfyVkJ5iwBb2X2DQ4j (mission sim, assembly steps, measure
list, datasheet). Dart viewer https://claude.ai/artifact/Sqt8tF67veTYfxQjUFTZ5f.

## Start here

0. **`docs/build/FIRST_DAY.md`**: the first session back, in order (read, measure, buy, print the test coupons).
1. **`docs/build/MEASURE_LIST.md`**: the few numbers nobody could know yet (the wiper motor, a hang test of both
   springs). Put them in `base/spec/params.py`, then rerun the base pipeline and the physics (the commands are at
   the end of the list): the motor adapter, coupler and spring anchors regenerate, and so does the ballistics table
   the Jetson aims with.
2. **`docs/build/GrappStar_Build_Guide.pdf`**: the one guide, in order: safety, parts, printing, base assembly, dart
   build, wiring, flashing, Jetson setup, calibration, bench tests, first mission, troubleshooting.
3. **`docs/build/TEST_PLAN.md`**: staged first tests, bench to field, with pass criteria. Never fire without the
   tether attached.
4. **`docs/build/SYSTEM_ELECTRICAL.md`**: the whole rover electrical system and the first power-on sequence.
5. **Try the whole mission on any PC first**: `cd jetson && python3 -m grappstar serve --sim`, then open
   http://localhost:8080: the operator page runs every step against the simulated base, dart and cameras.

## Status (3 Nov 2026, end of the build day)

| Area | State |
|---|---|
| Base CAD | 5 modules (frame, drum, launch, reel, sensors), about 230 parts, every number from `base/spec/params.py`. Exact-mesh collision audit, 235 parts over 44678 poses of the whole mission: nearly clean (2 small overlaps in passing poses, 24.5 and 55.6 mm3, and 1 bracket STL that is not watertight) |
| Print pack | `base/out/base/pack`: PLA 108 parts on 13 plates (about 3.8 kg, 103 h), PPA-CF 48 parts on 4 plates (about 1.4 kg, 67 h); test coupons first (`base/pack/coupons`, `base/pack/fit_coupon`) |
| Sim and steps | HD mission sim (19 phases, tether drawn) and the assembly steps page, on the site and in `base/out/base` |
| Physics | 12.1 J, 6.85 m/s, about 4.2 m at 34 deg with the 0.5 N brake; ballistics table `base/out/ballistics.json` for the Jetson; 83 tests |
| Base firmware (ESP32-S3) | 446 PC checks plus fuzzing, compiles for the DevKitC-1 (esp32 core 3.3.x); protocol, wiring, calibration, bench test |
| Dart firmware (XIAO ESP32S3) | host checks, fuzz, simulator, Jetson-client and web-page tests all pass |
| Jetson | mission, vision (landing zone, aerial map, targeting), operator page, setup; 397 tests; `serve --sim` runs a whole mission on any PC |
| System test | the real firmware and the Jetson app together: nominal mission, refusals and 9 power-loss phases pass (`systest/REPORT.md`) |
| Build guide | `docs/build/GrappStar_Build_Guide.pdf` (plain Arial), measure list, test plan, electrical system |

Open items (details in each REPORT.md):
- CAD: check these by hand at assembly. The drum coupler brushes the index lever during the fire phase (24.5 mm3); the aim sun gear brushes the aim idler in one sensor test pose (55.6 mm3); the SW_RAIL_LOW bracket STL is not watertight (let the slicer repair it). Details in `base/out/base/audit.txt`.
- Tow on a very stiff short line can overshoot a 40 N limit by up to about 10 percent (systest F21); calibrate on the bench first (CALIBRATION.md) and keep the first field tows at or under 40 N (D46: the cup ring is a plain eye in v1).
- Vision thresholds are tuned on synthetic frames; re-learn on real Caddx Ant footage (`jetson/REPORT.md`). Measure the Caddx Ant field of view (fisheye).
- Rover drive is not driven by the Jetson yet (unknown hardware): the tow asks the operator to drive.
- Assembly order: three dependency notes remain (bench sub-assemblies); follow the step order on the steps page.

## What to buy

8 to 10 bearings 608ZZ (search "608ZZ bearing 8x22x7"). Optional: a small neodymium disc magnet for the AS5600 angle
sensor (the Adafruit board ships without one; the base also runs without it on a printed index contact). Everything
else is in hand or printed (Bambu X2D, PLA; PPA-CF for loaded gears and keyed shafts). Range is about 4.2 m at 34 deg
with the two launch springs (physics model: 12.1 J, 6.85 m/s, 0.5 N payout brake; the first test shots calibrate it). The rail has holes for a
second pair of springs, but the drive train is not rated for them in v1 (decision D38).

## Layout

| Path | What |
|---|---|
| `docs/build/` | build guide (PDF and Markdown) and its generator, electrical system, measure list, test plan |
| `base/spec/` | base station spec (`BASE_SPEC.md`), every shared number (`params.py`), decisions log (`DECISIONS.md`), architecture panel (`panel/`, reference) |
| `base/cad/` | base CAD: shared library (`lib/`) and one module per subsystem (`parts/`: drum, launch, reel, frame) |
| `base/tools/` | assembly, kinematics, exact-mesh collision audit, renders, print pack, HD sim, assembly steps, guide |
| `base/physics/` | launch, flight, payout, capstan and motor models; the ballistics table for the Jetson |
| `base/out/`, `base/pack/`, `base/site/` | generated: model data, print pack, web site |
| `firmware/base/` | ESP32-S3 base firmware (Arduino), protocol, wiring, calibration, bench test, PC tests |
| `firmware/dart/` | KEDGE dart firmware (XIAO ESP32S3) with the new pin map, API, wiring, PC tests |
| `jetson/` | rover software: comms, video, vision, ballistics, mission, operator web page, setup, tests |
| `kedge/` | dart project and history; `kedge/dist_v30` is the dart v30 print and build pack |
| `knowledge/` | system plan, work plans, messages, screenshots |
| `originals/` | Harry's source files, untouched |
| `scripts/` | `check_all.sh` (every test suite), `make_site.py` (the site) |

## Rebuild

- Tests: `scripts/check_all.sh` (`--full` also runs the base CAD pipeline).
- Base CAD pipeline: `cd base && bash run_all.sh base` (assembly, audit, renders, sim, steps, pack; it stops on a NOT CLEAN audit unless run with `FORCE=1`). The audit lists the CAD open items above, so a rebuild needs `FORCE=1`; then check that `base/out/base/audit.txt` lists nothing new.
- Physics: `sh base/physics/run_all.sh` (every model, the ballistics table `base/out/ballistics.json` for the Jetson, SUMMARY, tests).
- Build guide: `kedge/.venv/bin/python docs/build/make_guide.py` (needs reportlab and pypdf: `kedge/requirements.txt`).
- Site: `python3 scripts/make_site.py`.

## Rules

- Battery unplugged before any wire moves.
- No em dashes in any deliverable. Plain black Arial PDFs, no name on documents.
- Every geometry change goes through the exact-mesh collision audit before it is shown.
- Only parts already in hand, plus 608 bearings.

History: the September README is `docs/README_sep2026.md`; the superseded overnight ground station (3 Nov) is in
`kedge/gs` (site https://claude.ai/artifact/Uof3qNiqzPXed57HCM79jK).
