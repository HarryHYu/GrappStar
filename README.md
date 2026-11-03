# GrappStar

A rover fires an anchor dart (KEDGE) on a Dyneema tether. The dart films its flight and landing from a nose camera,
screws itself into the ground, and the rover winches itself out against it. Then the dart lets go and is reeled home.
One 12 V wiper motor runs the whole base station (launch, aim and reel); a small N20 selector drum swaps which system
it drives, in a fixed physical order: LAUNCH, NEUTRAL, WRAP, REEL.

Online: base station site https://claude.ai/artifact/5DbSKfyVkJ5iwBb2X2DQ4j (mission sim, assembly steps, measure
list, datasheet). Dart viewer https://claude.ai/artifact/Sqt8tF67veTYfxQjUFTZ5f.

## Start here

0. **`docs/build/FIRST_DAY.md`**: the first session back, in order (read, measure, buy, print the test coupons).
1. **`docs/build/MEASURE_LIST.md`**: the few numbers nobody could know yet (the wiper motor, one spring hang test).
   Put them in `base/spec/params.py` and rerun the base pipeline: the motor adapter, coupler and spring anchors
   regenerate.
2. **`docs/build/GrappStar_Build_Guide.pdf`**: the one guide, in order: safety, parts, printing, base assembly, dart
   build, wiring, flashing, Jetson setup, calibration, bench tests, first mission, troubleshooting.
3. **`docs/build/TEST_PLAN.md`**: staged first tests, bench to field, with pass criteria. Never fire without the
   tether attached.
4. **`docs/build/SYSTEM_ELECTRICAL.md`**: the whole rover electrical system and the first power-on sequence.
5. **Try the whole mission on any PC first**: `cd jetson && python3 -m grappstar serve --sim`, then open
   http://localhost:8080: the operator page runs every step against the simulated base, dart and cameras.

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
- Base CAD pipeline: `cd base && bash run_all.sh base` (assembly, audit must say CLEAN, renders, sim, steps, pack).
- Build guide: `python3 docs/build/make_guide.py`.
- Site: `python3 scripts/make_site.py`.

## Rules

- Battery unplugged before any wire moves.
- No em dashes in any deliverable. Plain black Arial PDFs, no name on documents.
- Every geometry change goes through the exact-mesh collision audit before it is shown.
- Only parts already in hand, plus 608 bearings.

History: the September README is `docs/README_sep2026.md`; the superseded overnight ground station (3 Nov) is in
`kedge/gs` (site https://claude.ai/artifact/Uof3qNiqzPXed57HCM79jK).
