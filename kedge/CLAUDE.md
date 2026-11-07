# KEDGE DART: CLAUDE CODE HANDOVER (state at 2 Nov 2026 23:30 Sydney, current design = v29)

Read this whole file first. It is everything known about the project from the cloud session (1-2 Nov 2026).
Older full handover (v26, bench history, v26 source) is in `HANDOVER_v26_original.md`. Original caliper PDF with
photos of every part is `measurements.pdf` (145 MB, look at it for SHAPES, not just numbers).

---------------------------------------------------------------------------------------------------

## 0. HOW HARRY WANTS YOU TO WORK

- Very brief replies. Short and direct. He pushes back if long.
- No em dashes in ANY deliverable (docs, comments shown to him, PDFs).
- Documents: PDF, plain black Arial (Liberation Sans is fine as the Arial metric twin), no decorative styling, no name on documents.
- Never advise him to stop iterating or freeze the design. Keep auditing every round.
- Don't ask questions when he says "make the call": decide and note the decision.
- When he says work until a time, WORK UNTIL THAT TIME. The cloud session failed this twice (stopped after an unrelated
  message overnight, and stopped at 4:05 pm when told 5 pm). He was rightly angry. Do not repeat it.
- Hands-on electronics: walk him step by step in plain language (he is learning; the first XIAO died from wiring with the battery live).
- Price is not the point. Minimise actuators (no springs, no extra motors; a second hinge was rejected).
- Before sending any CAD: run the full audit, look at the 24-angle sheet and section cuts yourself. Never send unchecked geometry.
- His standards: smooth surfaces, minimum mass, nothing floating, nothing inside something else, every part explained.
- He uses a Bambu X2D printer, PLA (PPA-CF only for reel gears). Mac. Arduino IDE.

---------------------------------------------------------------------------------------------------

## 1. WHAT THE PROJECT IS

GrappStar: a rover (Jetson) with a launcher fires a dart (KEDGE) on a tether. The dart lands, anchors itself in soil,
and the rover's reel then winches against it (grappling / self-recovery). Three subsystems: dart, launcher, reel.

The dart after landing:
1. Kick leg swings ~202° over the nose and plants a foot auger AHEAD of the nose (stops the nose lifting under tether pull).
   The foot N20 in the foot housing spins the foot auger while the kick drives.
2. Two drill pods swing down 42.25° on the flanks (one deploy N20 through a shared axle) while the drill motors spin
   the augers in (Ø16 x 70, left pod right-hand thread, right pod left-hand, counter-rotating so the dart doesn't twist).
3. Tether runs from the tail cap (two Ø4.2 holes) to the reel.
Camera (analog Caddx Ant) in the blunt nose, 5.8 GHz VTX to the rover. Control over WiFi from the XIAO ESP32S3.

---------------------------------------------------------------------------------------------------

## 2. ALL CALIPER MEASUREMENTS (mm), from Harry (very small chance of error; trust them)

Motors
- Kick/deploy/foot worm N20 (right-angle, self-locking, 12 V): body 12 x 10, worm box 10 x 12 (box 11.93 tall),
  33.04 long box+motor without shafts, 29.97 shaft tip to tip (biaxial output), 9.94 worm box width.
- Drill motor (straight gearmotor, threaded shaft, Bringsmart type, 100 rpm 12 V): gearbox 11.95 tall, 12 x 10 body,
  24.27 face to back, shaft 2.84 dia (M3 thread), 56.03 long past the face.

Boards (L x W x H)
- XIAO ESP32S3 (plain, no Sense): 21.45 x 17.74, 22.75 incl USB-C lip, 4.49 thick board+USB, 13.01 with header pins.
- DRV8833 carrier: 20.31 x 12.70, 7.19 with header body, 11.68 with pin tips. Modelled 3.5 thick with headers removed.
- INA219: 25.69 x 20.32, bare PCB 1.58, 4.70 with JST, 10.07 with screw terminal (terminal to be removed).
- BNO085: 25.72 x 22.77, 4.77 with JST.
- MP1584 buck: 22.26 x 16.96 x 3.98.
- 2S BMS old 10 A: 47.36 x 23.63 x 3.35 (too big, replaced).
- 2S BMS new OTOTEC 8 A stick (listing, NOT yet calipered): 40.8 x 8 x 2.1.
- HGLRC Zeus nano VTX: 42.25 x 19.91 x 6.01, 13.55 across the waist.

Camera
- Caddx Ant body: 13.99 wide x 14.59 tall, 18.34 lens front to connector back, board 2.69 thick. Lens Ø8, protrudes 2.7.
- Ant bracket 18.93 x 18.95 x 8.30 (not used). Ant OSD joystick board 28.15 x 22.15 x 9.83 (bench only).

Power
- 18350 cell (HIBATT IMR): Ø18.66 with wrap, 36.87 button to base, 36.15 body without button.

Hardware
- F693ZZ flanged bearing 3 x 8 x 4 (bag label). Flange modelled Ø9.5 x 0.6 (datasheet; caliper it).
- Datasheet values used (not measured): N20 mount holes and D-flat depth, M3 nyloc (5.5 AF, 4 thick), M3 nut (5.5 AF, 2.4), u.FL antenna.

Measured currents at 8 V (bench, INA219)
- N20 worm (kick/deploy): 20-27 mA free, ~180 mA stall.
- Drill pair: 60-70 mA free, ~700 mA stall (both).
- Foot/third motor: ~40 mA free, 500-600 mA stall.
- Everything together under 2 A worst case.

---------------------------------------------------------------------------------------------------

## 3. PARTS LIST (dart)

| Part | Qty | Status |
|---|---|---|
| Seeed XIAO ESP32S3 (plain) | 1 (+1 spare) | in hand (the first one, a Sense, died: wired with battery live) |
| DRV8833 dual H-bridge carrier | 3 | in hand |
| INA219 | 1 | in hand |
| BNO085 | 1 | in hand, blank Serial Monitor bug on new XIAO was unresolved (likely USB CDC On Boot; firmware now refuses to compile without it) |
| MP1584 buck | 2 | in hand, set 8.12 V (bench motor rail) and 5.04 V |
| OTOTEC 2S 8 A stick BMS 40.8x8x2.1 | 1 | ordered 24 Oct |
| HIBATT IMR 18350 | 2 | in hand (one wrap re-taped) |
| XTAR MC2 charger | 1 | in hand |
| N20 right-angle worm 12 V | 3 needed | 2 in hand, BUY ONE MORE (foot) |
| Straight gearmotor M3x56 threaded shaft 100 rpm | 2 | 3 in hand |
| F693ZZ | 5 | 2 per drill pod + 1 in the kick ear |
| M3 nyloc | 2 | auger tips; M3 nut x2 (pod inner bearing) |
| M3 countersunk screws | radial hull screws (see run sheet for positions/lengths) | |
| M3 screws + washers | kick hub screw, 2 coupler screws, ear root screw | |
| Battery contacts | springs (front disc), rigid plates (rear disc) | |
| JST-XH 3-pin charge socket + JST-XH 2-pin power loop socket with shorting plug (= ON switch) | 1 each | tail window |
| Schottky diode 1N5817 / SS14 | 1 | 5 V feed to XIAO, stripe toward XIAO |
| Electrolytic capacitor | 1 | across MP1584 output: long leg OUT+, striped leg OUT- |
| Caddx Ant nano camera | 1 | in hand |
| HGLRC Zeus nano VTX (25 mW) | 1 | in hand |
| BetaFPV u.FL 5.8 GHz antenna | 1 | in hand (antenna sits at z 12.8 in the tail) |
| 15 g nose ballast (lead/tungsten putty in the nose cavity, fills from tip to x≈13.7) | 1 | needed for stable flight |
| Rover side: Lockmaner RC832 receiver, DIGITNOW USB AV capture | 1 each | ordered |
| Bench: Zeee 3S LiPo, iMax B6 clone, XT60 lead #1 10 A fuse, XT60 lead #2 20 A fuse (reel) | | built |

---------------------------------------------------------------------------------------------------

## 4. ELECTRICAL (v29 final, supersedes the v26 handover pin map)

> SUPERSEDED for wiring by firmware/dart/WIRING.md (dart firmware v4, 3 Nov 2026: DRV8833 #3 is off the dart, no
> bridge across the DRV#2 channels). Do not wire from the pin map below.

Power tree (dart):
- 2 x 18350 series (2S, 6.0-8.4 V). Springs on the FRONT cell-bay disc, rigid plates on the REAR disc (launch pushes
  cells aft onto the plates so contact never breaks). Series link wire between the two front springs.
- OTOTEC BMS: B-, BM, B+ from cells; P+/P- out.
- JST-XH 3-pin charge socket in tail: pin1 B-, pin2 BM, pin3 B+.
- P+ -> JST-XH 2-pin power loop socket (shorting plug = ON switch) -> INA219 VIN+.
- INA219 VIN+ pad also feeds MP1584 IN+ (buck taps BEFORE the shunt, so INA219 reads motor current only).
- INA219 VIN- -> motor rail -> DRV#1 VM -> DRV#2 VM -> DRV#3 VM (2S direct, no buck for motors).
- MP1584 OUT 5.04 V -> Schottky -> XIAO 5V; also VTX 5 V and camera 5 V. Capacitor across MP1584 OUT.
- XIAO 3V3 -> DRV8833 SLP/nSLEEP/EEP pad on all three (NOT VCC/VIN: on these carriers "VIN"/"VCC" may be the motor
  supply. Meter check: if the pad beeps to VM it is the motor supply, do NOT put 3V3 on it), INA219 VCC, BNO085 VIN.
- RULE: power loop plug out whenever a wire moves or USB is plugged in.

XIAO pin map (firmware v3):
| Pin | To | Function |
|---|---|---|
| D0 / D1 | DRV#1 AIN1 / AIN2 | deploy N20 |
| D2 / D3 | DRV#1 BIN1 / BIN2 | kick N20 |
| D4 / D5 | INA219 + BNO085 SDA / SCL | I2C (INA219 0x40, BNO085 0x4A) |
| D8 / D9 | DRV#2 AIN1+BIN1 bridged / AIN2+BIN2 bridged | drill pair (right drill motor leads SWAPPED so they counter-rotate) |
| D6 / D7 | DRV#3 AIN1 / AIN2 | foot N20 (D6/D7 = GPIO43/44 UART0, twitch at reset: harmless on the slow worm) |
| 3V3 | DRV SLP x3, INA219 VCC, BNO085 VIN | logic |
| 5V | via Schottky from MP1584 | |

Full 53-wire cut list with colours, gauges and lengths: `pack_v29/wire_cut_list.csv` (copy in Appendix A below).
Wiring diagram: `pack_v29/wiring.png`.

Reel (separate ESP32-S3 DevKitC-1 + BTS7960): RPWM GPIO4, LPWM GPIO5, R_EN GPIO6, L_EN GPIO7, VCC 3V3, B+/B- via
XT60 lead #2 20 A fuse (10 A blew on inrush). Wiper motor rebuilt to 2 wires, park linkage removed. Drum and gears
to print in PPA-CF. Tension limiting sketch: `fw/reel/reel_tension_limit/` (BTS7960 IS current sense on GPIO1, or HX711
load cell; speed tapers over last 30%, stops at limit, `l20` sets 20 N). Needs calibration on the bench.

Camera chain: Caddx Ant -> Zeus VTX (5 V, common GND, u.FL antenna) -> 5.8 GHz -> RC832 on rover -> DIGITNOW capture ->
Jetson /dev/video0. Control: 2.4 GHz WiFi, XIAO softAP.

---------------------------------------------------------------------------------------------------

## 5. GEOMETRY v31 (generator `kedge_v27.py`, its defaults ARE v31; v29/v30 history at the end of this section)

Coordinates: x along hull, nose tip x=0, tail x≈152. y across (+y = LEFT = kick side), z up. Ground z = -21 lying flat.

Key parameters (all in `out31/parts.json` "P"; harness way points in P.WIRE_PTS):
R_OUT 21 (Ø42), R_BAY 19.5, WALL 1.5, NOSE_L 20 (frustum from Ø22 tip), NOSE_SPIG 7, TAIL_L 11, TAIL_SPIG 4.4,
L 151.97. N20_POCK 12.4 square (either motor orientation), TERM 1.6 terminal room behind every worm N20. SADDLE_X0 23.8.
Kick: hub Ø12 x 14 at (31.3, 14.9, 27.5), pole Ø8 x 72, KICK_DEG 201.75 (solved), KICK_REST 1.5° past it, foot auger
Ø13 x 40 (core 6) 0.3 over its housing, ear y 22.3-26.7 with F693ZZ seated 0.4 proud onto a printed inner-race boss on the hub.
Drills: axle x 56, pivot z 11 (was 10), pods y ±29 (was 31), DEPLOY_DEG 43.0 (solved), augers Ø13 x 72.8, core 6, pitch 11,
tip 12, flat core end on the outer bearing's inner race. Pod stack from the pivot: pocket 4.4 (0.78 wall to the coupler bore),
DTERM 1.2 terminal room (tabs bent up), motor face XS 29.87, BUSH_GAP 0.6 to the M3 nut, F693ZZ pair back to back, nose face
XA 40.47, bottom nose edge chamfered 1.35. Couplers Ø7 D in PPA-CF (printed on the flat), printed pod caps Ø10 x 1.5 in a
counterbore under the M3 x 8 (the pod floats 0.7, never clamped). DEPLOY_Y -2.92 (computed).
Hard stops: DEPLOY HORNS on hull_top in front of each pod head (HORN_GAP 0.15, about 1° past DEPLOY_DEG), FOLD LUGS under
the heads (about 2° past folded), KICK REST on the ear root. Tow turns the legs further OUT, so at full deploy the tow load
goes into the hull, not into the worms, their D stubs or the couplers. The audit checks every stop touches and never early.
Layout "cells_fwd" along x: nose 0-20 (camera; no ballast in v31) | sled + disc1 27.5-30.5 (deploy N20 cradle, MP1584 shelf,
BNO085 on rails under it, turned across the dart) | disc2 62.6-65.6 | cells 65.6-106.07 side by side y ±9.6 (VTX over, BMS
under in the keel channel) | disc3 106.07-108.07 | boards 108.07-134.57 (deck B: XIAO turned, USB-C to -y through a 12.5 x 7
hull window, DRV#1; deck C: DRV#2 + INA219; no deck A) | disc4 134.57-136.57 | tail spigot | tail cap (power-loop socket in a
recessed TOP seat = ON switch reachable on the launcher; charge socket in the bottom seat; tether holes chamfered 0.8).
Harness: KEEL channel 9.2 x 8.4 under the cells (discs 2-4, saddles, sled rear wall) + TOP over the VTX; drill leads out at
x 59.5 z 16.5 into the pod tops; worst fill 0.26 keel / 0.19 top (`wireroom.py`). DRV8833 #3 is off the dart (firmware v4).
Hull split horizontally at z=0, belly is a plain cylinder (launcher cradle); legs stow on top and flanks.

Printed parts (15 + fit coupon): hull_top, hull_bottom, nose, tail_cap, tray, kick_leg, foot_auger, drill_leg_L/R,
drill_leg_L/R_auger, coupler_L/R (PPA-CF), pod_cap_L/R. Bought-part envelopes and all hardware are modelled too (57 parts).

Results (out31, audit CLEAN at kick 1° steps and drill 1° steps, kick x drill cross sweep, ground, belly, hard stops,
watertight, one body per printed part; all STEP files reimport valid; launcher_fit FITS: zero overlap with every part of
base/cad/parts/launch.py over 1684 launcher and mission states, same as v30):
- Folded bbox x 0-169.9, y ±38.8, z -21 to 75.3. Barrel channel (1 mm clear) y ±38, z -22 to 76.3.
- Drills 45.7 mm deep (v30 44.8), foot 35.1. Pod ground clearance 0.52 (solver), foot housing 0.02.
- Printed 87.6 cm3, 108.6 g at 4 walls / 40%. Mass 260.5 g with catalogue bought masses, no ballast. CG x 80.9, z +6.8.
- Physics at the real 7 m/s launcher, holding, milling deploy, landing: kedge/DART_PHYSICS31.md (dart-physics).
- Print: plate A (hull, nose, tail, pod caps, PLA, tree supports inside the hull halves only), plate B (tray, legs, augers,
  PLA, no supports, brim 8 on the augers), plate C (couplers, PPA-CF). Run sheet and engineering report in pack_v31.

v29 (Ø16 x 70, pods 30.5, pivot 10) and v30 (Ø17 x 72, pods 31) are in out29 / out30 and dist_v29 / dist_v30. v30 defects
fixed in v31: the kick N20 could not go into the saddle (closed shaft holes both sides), the foot N20 likewise; the pods were not
held on their couplers (M3 washer Ø7 smaller than the Ø7.25 pod bore); the power-loop plug in the bottom window would stick out
of the belly onto the launcher's rear cradle ring; no room for the N20 end-cap tabs or the drill gearbox bushing; the auger base
boss and the kick washer touched the bearing shields / outer rings.

Rules learned (do not regress):
- A drill pointing down deployed points up folded; single hinges only. Legs can't stow inside a cylinder: belly stays plain, barrel cut around top/flanks.
- Exact-mesh collision sweep on saved vertices (npz), not STL round-trips. Check manifold status. Test every sub-part.
- Augers in build123d: OCC helix sweep booleans fail, build them in trimesh and write faceted STEP (`meshstep.py`, which now merges
  vertices at 1e-3 first: slivers made the v31 foot auger STEP invalid).
- Disconnected bodies: keep annulus joints / rings so every printed part is one body.
- Countersinks in the tray/tail broke STEP validity: use flat cylinder seat() pockets.
- Augers L and R MUST be opposite hands (v26 had both the same: one would unscrew).
- Every bought part must be insertable in the order of the run sheet: biaxial shafts need entry slots, washers must be bigger
  than the bore they retain, bearing faces may only touch the inner race of a turning part.
- Stops: legs that tow pushes outward get a hull stop at their deployed angle (horns, kick rest); `solve_deg.py` exits 3 and
  `run_all.sh` regenerates when the solved angles differ from the ones the stops were built at.
- prism_xz extrusion direction follows the polygon winding: place by bounding box (the first kick rest came out on the wrong side).

---------------------------------------------------------------------------------------------------

## 6. FIRMWARE

`fw/kedge_dart/kedge_dart.ino` + `kedge_logic.h` (v3). Board XIAO_ESP32S3, Tools > USB CDC On Boot: Enabled (sketch
#errors otherwise, because with it disabled Serial uses D6/D7). Libraries: Adafruit INA219, Adafruit BNO08x.
Compiles on ESP32 Arduino core 2.0.17 and 3.3.12.
- Keys (Serial 115200 "No line ending", or WiFi AP "KEDGE" / "grappstar" at http://192.168.4.1, hold-to-run):
  1/! deploy down/up, 2/@ kick out/back, 3/# drills in/out, 4/$ foot in/out, space/s STOP, +/- speed, i current,
  b attitude, z I2C scan, a auto-anchor, A arm auto on landing, h help.
- API: GET /status (JSON), /key?c=&ms=, /stop, /auto. OTA port "kedge".
- Safety + all I2C in a FreeRTOS task every 5 ms (core 1, mutex). Trip 2500 mA, deadman 350 ms on WiFi, 1.5 s per serial key.
- Auto sequence: kick swing -> kick + foot planting (stall on contact) -> foot finishing -> deploy + drills -> drills finishing -> anchored.
  Stall detection while two motors share the INA219: helper motor paused 150 ms every 400 ms so the main one is read alone.
- Tuning (kedge_logic.h struct Tuning, replace with bench values): kickContactMa 60, kickStallMa 150, deployStallMa 150, tripMa 2500.
- PC test: `cd fw/test && g++ -std=c++17 -O1 -I../kedge_dart -o test_logic test_logic.cpp && ./test_logic` -> 23 checks ALL PASS (plant model).
- Jetson: `fw/jetson/kedge_client.py` (Dart class: status, key, hold, stop, auto, wait_auto; CLI drive/watch/video),
  `mock_dart.py` (fake dart server), `test_client.py` ALL PASS.

---------------------------------------------------------------------------------------------------

## 7. FILES

Pipeline (run from this folder; `.venv/bin/python` has build123d, trimesh, manifold3d; node + playwright for the shots):
- `run_all.sh out31 v31 [stepK stepD]` : the whole pipeline (generate, solve and regenerate at the solved angles, audit,
  launcher fit, clearance, meshes, shots + sheet, pack, mass props, STEP check, physics, coupon, wires, wire room, wiring
  diagram, barrel, flight sim, assembly STEP, plate view, run sheet, engineering report), about 30 min. `make_dist.sh v31` : zips.
- `kedge_v27.py OUTDIR` : generator. `KOV='{"AUGER_D":17}'` env JSON overrides any P.
- `solve_deg.py OUT` : solves KICK_DEG / DEPLOY_DEG from ground clearance; exit 3 = regenerate (stops sit at the old angles).
- `audit27.py OUT stepK stepD` : collision sweep, cross sweep, ground, belly, hard stops, watertight, multi-body -> CLEAN / NOT CLEAN.
- `launcher_fit.py OUT [REF]` : the dart against the real launcher (base/cad/parts/launch.py, all base states), vs the v30 dart.
- `wireroom.py OUT PACK` : free section of the keel and top passages along x against the cut-list bundle.
- `massprops.py OUT` : out31/mass_props.json (volume, centroid, unit-density inertia per part, folded and deployed) for dart-physics.
- `stepcheck.py PACK` : reimports every STEP. `wires.py OUT PACK` : the firmware v4 50-wire list on the v31 routes (route column).
- `clearance.py`, `meshes.py`, `shots.js` + `sheet.py` (24-angle sheet + 12 sections), `pack.py` (print poses, supports, 3 plates),
  `coupon.py` (fit coupon, v31 row: JST seats, USB-C window, pod cap), `wiring_dot.py`, `barrel.py`, `assembly_step.py`,
  `plate_view.py`, `meshstep.py`, `runsheet.py`, `report.py` (PDFs, Arial on the Mac, Liberation Sans elsewhere).
- Physics files (dart-physics): `physics.py`, `flight_sim.py`, `dart_*.py`, `DART_PHYSICS31.md`. Shared notes: `DART_V31_NOTES.md`.
- `viewer.html` : three.js viewer (pose sliders, explode, section cut, audit panel, Play deployment animation).
Outputs:
- `out31/` v31 data (STL/STEP/NPZ per part, parts.json, audit.json, launcher_fit.json, wireroom.json, mass_props.json,
  physics.json, renders, audit_sheet.png, sections.png). `pack_v31/` print pack (STL, 3MF plates A/B/C, STEP, print_report.json,
  step_check.json, wire list, wiring.png, barrel files, run sheet and engineering report PDFs, assembly STEP). `dist_v31/` zips.
- `out29/ out30/ pack_v29/ pack_v30/ dist_v29/ dist_v30/` : the previous versions.
- Firmware lives in `../firmware/dart` (v4, WIRING.md, BENCH_TEST.md); `fw/` here is the old v3 copy.
- `pub/` files for the claude.ai viewer artifact (https://claude.ai/artifact/Sqt8tF67veTYfxQjUFTZ5f, Version 30).

Environment: build123d 0.13.0, cadquery-ocp, trimesh 5.2.0, manifold3d 3.5.4, numpy, scipy, shapely 2.2, reportlab, ezdxf,
lib3mf, matplotlib, playwright (+ chromium), node 22, graphviz. See `requirements.txt` and `setup_mac.sh`.
Arduino: arduino-cli or IDE, esp32 core by Espressif, board XIAO_ESP32S3.

---------------------------------------------------------------------------------------------------

## 8. REVIEWS ALREADY DONE (independent agents, all findings fixed)

Firmware x2 (3V3 to SLP not VCC; D6/D7 reset twitch; safety blocked by web server -> tick task; Shift release and touch
cancel left motors running; landing detect once per boot; CDC On Boot guard; OTA could restart motors; JS key repeat),
wiring, geometry/physics pipeline (pole section modulus 1000x off; clay Broms fixed-head -> free-head; axial/lateral
combination; embedded length; tip-over levers; shell mass; manifold status; finer sweep; cross sweep; belly sampling),
run sheet for a beginner x2 (order, capacitor polarity, motor direction test before routing, charge socket polarity,
heat on PLA, bench supply = MP1584 #2 at 8.0 V, directions defined).

---------------------------------------------------------------------------------------------------

## 9. OPEN ITEMS / NEXT WORK

Not done yet:
1. Soil pull-test procedure + logging script (promised, not written). Use INA219/HX711 or a luggage scale; log pull vs displacement; compare with the Broms numbers.
2. Package v30 (Ø17 x 72 augers, pods y 31) through make_dist if Harry wants +8% holding.
3. Launcher design is only a barrel silhouette + keep-out; launcher itself not designed here. Reel only has firmware.
4. Nothing has run on real hardware with v29 geometry yet. Print the fit coupon first.
Bench numbers needed from Harry (then re-run physics/firmware tuning):
- Weigh all bought parts (physics.py BOUGHT_G). Stall currents per motor at 2S -> kedge_logic.h Tuning.
- Launcher stroke (physics.py STROKE_M, assumed 0.2 m). Reel brake force and BTS7960 IS calibration.
- Soil pull test. Caliper the OTOTEC BMS stick and the F693ZZ flange. Which DRV8833 pad is SLP on his carriers.
- Kick N20 torque sanity (gravity 0.19 kg.cm, fine) and the BNO085 blank-serial bug (try with CDC On Boot enabled + `z` I2C scan).

---------------------------------------------------------------------------------------------------

## APPENDIX A: WIRE CUT LIST (v29, 53 wires)

```
#,from,to,colour,gauge,cut length mm,note
1,"Rear plate, cell 1 (-)",BMS B-,black,22 AWG,45,solder to the plate tab before it goes in its recess
2,"Front spring, cell 1 (+)","Front spring, cell 2 (-)",red,22 AWG,45,the series link: carries the full battery current; glue both springs in the front recess
3,Front spring pair (cell 1 + / cell 2 -),BMS BM,white,26 AWG,55,the two springs are joined by a short wire; this is the midpoint sense
4,"Rear plate, cell 2 (+)",BMS B+,red,22 AWG,45,
5,BMS B-,Charge socket pin 1 (tail window),black,26 AWG,80,"JST-XH 3-pin, balance charge"
6,BMS BM,Charge socket pin 2,white,26 AWG,125,
7,BMS B+,Charge socket pin 3,red,26 AWG,80,
8,BMS P+,Power loop socket pin 1 (tail window),red,22 AWG,80,JST-XH 2-pin; the shorting plug is the ON switch
9,Power loop socket pin 2,INA219 VIN+,red,22 AWG,95,main + after the loop
10,INA219 VIN+ (same pad),MP1584 IN+,red,22 AWG,140,"buck taps before the shunt, so INA219 reads motor current only"
11,INA219 VIN-,DRV#1 VM,red,22 AWG,50,motor rail 6.0-8.4 V
12,DRV#1 VM,DRV#2 VM,red,22 AWG,65,daisy chain
13,DRV#2 VM,DRV#3 VM,red,22 AWG,155,forward into the sled (keel)
14,BMS P-,MP1584 IN-,black,22 AWG,130,main ground
15,MP1584 IN- (same pad),DRV#1 GND,black,22 AWG,120,
16,DRV#1 GND,DRV#2 GND,black,22 AWG,65,
17,DRV#2 GND,DRV#3 GND,black,22 AWG,155,
18,MP1584 OUT+ (5.04 V),XIAO 5V via a Schottky diode (stripe toward the XIAO),orange,24 AWG,160,"set and check 5.04 V BEFORE connecting; the diode stops the buck back-feeding USB (1N5817 or SS14, in heat shrink)"
19,MP1584 OUT-,XIAO GND,black,24 AWG,160,
20,MP1584 OUT+,VTX 5V in,orange,26 AWG,95,
21,MP1584 OUT-,VTX GND,black,26 AWG,95,
22,VTX 5V in (same pad),Camera + (red),orange,28 AWG,105,runs forward along the top through the disc notches
23,VTX GND (same pad),Camera - (black),black,28 AWG,105,
24,Camera video (yellow),VTX video in,yellow,28 AWG,105,twist with the camera ground
25,XIAO 3V3,DRV#1 SLP / nSLEEP / EEP pad (NOT VCC or VIN),red/white,28 AWG,55,"wakes the driver. Check first: if the pad you picked beeps to VIN/VM on the meter, it is the motor supply: do NOT connect 3V3 to it"
26,DRV#1 SLP,DRV#2 SLP,red/white,28 AWG,65,
27,DRV#2 SLP,DRV#3 SLP,red/white,28 AWG,155,
28,XIAO 3V3,INA219 VCC,red/white,28 AWG,60,
29,INA219 VCC,BNO085 VIN,red/white,28 AWG,60,
30,XIAO GND,INA219 GND,black,28 AWG,60,
31,INA219 GND,BNO085 GND,black,28 AWG,60,
32,XIAO D4,INA219 SDA,blue,28 AWG,60,I2C SDA
33,INA219 SDA,BNO085 SDA,blue,28 AWG,60,
34,XIAO D5,INA219 SCL,green,28 AWG,60,I2C SCL
35,INA219 SCL,BNO085 SCL,green,28 AWG,60,
36,XIAO D0,DRV#1 AIN1,purple,28 AWG,55,deploy
37,XIAO D1,DRV#1 AIN2,grey,28 AWG,55,deploy
38,XIAO D2,DRV#1 BIN1,purple,28 AWG,55,kick
39,XIAO D3,DRV#1 BIN2,grey,28 AWG,55,kick
40,XIAO D8,DRV#2 AIN1 + BIN1 (bridge the two pads),purple,28 AWG,45,drills (moved off D6/D7: those twitch at reset)
41,XIAO D9,DRV#2 AIN2 + BIN2 (bridge the two pads),grey,28 AWG,45,drills
42,XIAO D6,DRV#3 AIN1,purple,28 AWG,145,"foot (slow worm, harmless if it twitches at reset)"
43,XIAO D7,DRV#3 AIN2,grey,28 AWG,145,foot
44,DRV#1 AOUT1,Deploy N20 + (red),red,26 AWG,165,motor sits in the sled cradle
45,DRV#1 AOUT2,Deploy N20 - (black),black,26 AWG,165,
46,DRV#1 BOUT1,Kick N20 + (red),red,26 AWG,150,out through the hole in the saddle floor
47,DRV#1 BOUT2,Kick N20 - (black),black,26 AWG,150,
48,DRV#2 AOUT1,Drill motor LEFT + (red),red,26 AWG,195,"out the left flank hole, 30 mm loop for the pod swing"
49,DRV#2 AOUT2,Drill motor LEFT - (black),black,26 AWG,195,
50,DRV#2 BOUT1,Drill motor RIGHT - (black)  SWAPPED,black,26 AWG,170,reversed on purpose: the pair counter-rotates
51,DRV#2 BOUT2,Drill motor RIGHT + (red)  SWAPPED,red,26 AWG,170,
52,DRV#3 AOUT1,Foot N20 + (red),red,26 AWG,255,"out the skin hole, 60 mm loop round the hub, through the pole channel"
53,DRV#3 AOUT2,Foot N20 - (black),black,26 AWG,255,"silicone wire, it flexes every deploy"
```

---------------------------------------------------------------------------------------------------

## 10. GROUND STATION (launcher + reel on ONE wiper motor), designed overnight 3 Nov 2026

Online build pack: https://claude.ai/artifact/Uof3qNiqzPXed57HCM79jK (HD mission sim, assembly steps, downloads).
Decisions Harry delegated ("make the call", all to scale, no more test builds): prototype scale sized to KEDGE v30;
dart v30 = Ø17 x 72 augers (packaged in dist_v30/); end-pull spincast reel replaces the v7 capstan + wrap cup; fixed 35 deg
fire (rail lifts 4 to 35 on a cam, a trip CORD from a grooved pin on the sear to a sprung eye bolt on a post trips it at
rail 33.0, phi 294.5); wiper dims unknown so a slotted motor plate + crank-arm coupler.
- Principle: motor FORWARD = reel (rotor geared 58:39 to the main shaft, pickup pin out); motor BACKWARD = launcher program
  gear through 2 x HF1216 (A); 2 x HF1216 backstop (C). One program-gear turn = cock 208 mm, sear catch, lift, shot, drop.
- Files: gs/gs_gen.py (all geometry, every number), gs/pose.py (kinematics), gs/audit_gs.py (exact-mesh sweep, must say CLEAN),
  gs/physics_gs.py (launch/flight/motor: 10.7 m/s, 8.5 m with 0.8 N end-pull peel), gs/timeline.py (mission), gs/pack_gs.py (print pack), gs/guide_gs.py
  (build guide PDF), sim_app.js / assembly_app.js (HD pages), gs/run_gs.sh (whole pipeline, ~1 min), gs/make_site.sh (site files).
- Outputs: gs/out/, pack_gs/ (print STLs, 3MF plates, STEP, renders, guide PDF, sim + assembly HTML), site/ (published files).
- Firmware: fw/gs/grappstar_gs/ (ESP32-S3 + BTS7960 + AS5600 + 3 switches), fw/test/test_gs.cpp (105 checks), fw/jetson/
  gs_client.py, mission.py (runs dart + ground station together). GS AP GRAPPSTAR-GS at 192.168.5.1.
- Review rounds (see below). Round 3 fixes (3 Nov ~3 am): catch run 1.4 mm + 18 mm keel (pad removed in round 4; the old 5 mm run would
  break the keel), cord outside the cheek in plane y -50 (generator checks the line against parts), sear down-stop -24.5,
  ISO 7379 5x25 shoulder bolt, 8-10 N sear spring on a screwed post, cheeks split (cheek_L/R/tie) to print flat, inserts in
  tiles for every frame foot, D flats only at shaft ends, PH_TRIP from the real cam (mkdata.py). Firmware edges: SW_COCKED
  opens 215.2 / closes 241.2 while cocking; cocked zone starts 241.
- Rounds 4-6 (3 Nov 3-5 am): sear spring slots, no keel pad (sled printed on its side), hook/keel chamfers, captive nut lug,
  M4 x 35 eye bolt, tiles to deck first, seated switch raised, O-ring squeeze, ceramic eye, rounded rotor edges, retrieve
  funnel + 115 deg horn, PLANET CARRIER: alpha = theta - phi*30/14, so the lift is now 252.5-302.5 blended constant speed to
  35 deg (amp <= 1.63), drop 304.5-332.5, cord taut 284.5, shot 294.5; steel flange hubs on PG and cam, 608 cam roller,
  18x24 machined clutch sleeves, firmware: planet phi tracking (SW_LOW open), dropShift 14.2 at the SW_LOW closing edge,
  IS current scaled by duty, tow limit <= 50 N. Retrieve onto the cradle is still the weakest link (hand-seat fallback).
- Round 7 docs consistency; round 8 launch: 40 mm 90A stop pads in wall-joined blocks, rail check strap (pin on cheek_R to a
  second sprung post at world (130, +50, 46), taut at 36 deg) takes the recoil kick, spring screw-in ends + foam dampers,
  flanged pulleys, 14 mm pusher hole (knot inside the tail cap). Round 9 (6 am): keel/stem slots +16 mm, strap at 37.5 deg with a 40 mm die
  spring, bumpers 16x15 (free-fall drop), firmware re-zeroes phi to 333 at the SW_LOW edge after the drop (roller clutch,
  no dead band) and stops there, markRunning() saves phi-unknown at each B start, fire eases off 285-302.5, standard 12 mm
  flange couplings (32x5, PCD 24). Site v17.
- Round 10: finish with the rail up sets liftSeen, SW_LOW sanity FAULTs (never opened / never closed by 330), drop edge only
  near the top, b jog marks phi unknown, bumper posts M6 inserts, fully threaded eye bolts. Site v18 (6:10 am).
- Round 11: dropMaxMs timer (300 ms, MEASURE the real fall) replaces the count check, dropDuty 120 after 302.5, re-zero at
  the debounced SW_LOW edge, rail-landed-while-stopped re-zero, finish after the trip window keeps tripSeen. Site v19.
- Rounds 12-13: guide safety block, glossary, missing steps; physics launches at the 33 deg trip angle (8.4 m); cable routes
  (never at the pivot, one 180 mm loop from the cheek tie to a base anchor at x 130, y 0), buck fed from the BTS7960 terminals,
  IS scale formula, ebox re-laid (BTS at +y with its own 14 AWG window, USB slot, sensor window), pull test capped at 50 N. Site v23.
- 7 am: site v27. Round 14 fixed page counts (47 parts, 8 plates). The source zip now includes meshstep.py, out30 data and
  vendored three.js: a clean extract rebuilds end to end (checked). The firmware zip compiles and tests clean from an extract.
  STEP: 45 of 47 valid; rotor and rail_beam_C faceted STEP reimport invalid (use the STL).
- Known: faceted STEP of rail_beam_C reimports invalid (use the STL). Retrieve onto the cradle and the wiper interface are
  the open risks; measure wiper torque/amp, spring rate, drop time first.
- Still assumed, measure and rerun: wiper torque/speed (physics_gs.py WIPER), spring rate, bench current constants.
