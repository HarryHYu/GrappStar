# GrappStar

A rover fires an anchor dart (KEDGE) on a Dyneema tether. The dart films its flight and landing from a nose camera,
screws itself into the ground, and the rover winches itself out against it. Then the dart lets go and is reeled
home. One 12 V wiper motor runs the whole base station (launch, aim and reel); a small N20 selector drum swaps which
system it drives.

## Start here

1. **`docs/build/GrappStar_Build_Guide.pdf`**: the one guide to build and run everything, in order: safety, parts,
   printing, base assembly, dart build, wiring, flashing, Jetson setup, calibration, bench tests, first mission,
   troubleshooting, and what to measure. Print it.
2. **`docs/build/MEASURE_LIST.md`**: do groups A (wiper motor) and B (springs) first. The base parts that touch the
   motor are generated from those numbers.
3. **`docs/build/TEST_PLAN.md`**: the staged first tests, bench to field, with pass criteria.
4. **`docs/build/SYSTEM_ELECTRICAL.md`**: the whole rover electrical system, wire gauges, first power-on sequence.

## The only purchase

608 bearings, 8 to 10 (search "608ZZ bearing 8x22x7"). Everything else is already in hand or printed (Bambu X2D,
PLA; PPA-CF for loaded gears and keyed shafts).

## Layout

| Path | What |
|---|---|
| `docs/build/` | build guide (PDF and Markdown), its generator `make_guide.py`, electrical system, measure list, test plan |
| `base/spec/` | base station spec (`BASE_SPEC.md`) and every shared number (`params.py`) |
| `base/cad/` | base CAD: shared library and one module per subsystem (drum, launch, reel, frame) |
| `base/tools/` | assembly, kinematics, collision audit, renders, print pack, HD sim, assembly steps |
| `base/physics/` | launch, flight, payout, capstan and motor models; the ballistics table |
| `base/out/`, `base/pack/`, `base/guide/`, `base/site/` | generated: model data, print pack, assembly steps, web site |
| `firmware/base/` | ESP32-S3 base firmware (Arduino), wiring, protocol, calibration, bench test, PC tests |
| `firmware/dart/` | KEDGE dart firmware (XIAO ESP32S3) with the new pin map, API, PC tests |
| `jetson/` | rover software: comms, video, vision, ballistics, mission, operator web page, tests, setup |
| `kedge/` | dart project and history; `kedge/dist_v30` and `kedge/pack_v30` are the dart v30 print and build pack |
| `knowledge/` | system plan, work plans, messages and decisions |
| `originals/` | the original source files, untouched |
| `scripts/check_all.sh` | runs every test suite |

## Rebuild the outputs

- Build guide: `python3 docs/build/make_guide.py` (needs reportlab and pypdf; graphviz for the diagram).
- Base pipeline: see `base/README.md` (assembly, audit must say CLEAN, physics, print pack, guide).
- Tests: `scripts/check_all.sh`.

## Rules

- Battery unplugged before any wire moves.
- No em dashes in any deliverable. Plain black Arial PDFs, no name on documents.
- Every geometry change goes through the exact-mesh collision audit before it is shown.
- Only parts already in hand, plus 608 bearings.
