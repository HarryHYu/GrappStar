# KEDGE DART: CLAUDE CODE HANDOVER (state at 2 Nov 2026 23:30 Sydney; geometry sections 5 and 7 updated 11 Nov 2026, current design = v35)

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
| F693ZZ | 1 | in the kick ear (v32/v33 drills use none) |
| M3 nut | 2 | v33: one per drill auger, pressed in the hex pocket at the end of the hub bore (no nylocs any more); threadlocker on the drill shafts |
| M3 countersunk screws | radial hull screws (see run sheet for positions/lengths) | |
| M3 screws + washers | kick hub screw, 2 coupler screws, ear root screw | |
| Battery contacts | springs (front disc), rigid plates (rear disc) | |
| JST-XH 3-pin charge socket + JST-XH 2-pin power loop socket with shorting plug (= ON switch) | 1 each | tail window |
| Schottky diode 1N5817 / SS14 | 1 | 5 V feed to XIAO, stripe toward XIAO |
| Electrolytic capacitor | 1 | across MP1584 output: long leg OUT+, striped leg OUT- |
| Caddx Ant nano camera | 1 | in hand |
| HGLRC Zeus nano VTX (25 mW) | 1 | in hand |
| BetaFPV u.FL 5.8 GHz antenna | 1 | in hand (antenna sits at z 12.8 in the tail) |
| 14.6 g nose ballast (lead/tungsten putty, v34: in the nose round and under the camera tube up to the collar at x 13.2) | 1 | in hand (15 g); helps flight |
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

## 5. GEOMETRY v35 (generator `kedge_v35.py`, its defaults ARE v35; v34 = `kedge_v34.py` right after the v35 block; v33 = `kedge_v33.py` and v32c = `kedge_v32.py` after that; v31 = `kedge_v27.py`)

v35 (11 Nov 2026) = v34 with the drill pods turned round (coordinator: "the drill tips must rake BACK toward the tether, like a tent peg";
"the tow must close the pod onto that stop, never open it"). Running handover: kedge/V35_PROGRESS.md. Everything not listed here is v34.
PODS AT THE FRONT: pivot x 50 (P.DRILL_X), z 9.5 (kept: the launcher's stop springs pass the dart at z -1.1 for |y| > 25, so the hubs and
the arm-head lugs must stay above), drill axis |y| 31.15, folded BACKWARD along the flanks (auger tips x 151.3), parts.json "sgn" +1 for the
pods (the leg stays -1). Deploy solved 46.75 deg (same as v34): the tips go down and aft (x about 119, 43.5 deep), under the line. Pivot blocks:
the v33 method unmirrored (deploy stop now forward of and below the head, fold stop under it). The tow pushes each pod onto its deploy stop.
REUSED PARTS: the v34 arms, augers, couplers, pod caps (and motors, nuts) are the same STLs, turned round and swapped over: v35 drill_leg_L =
v34 drill_leg_R (checked: identical by rotation, so hands kept per part). So the LEFT pod now has the LEFT-hand auger and the swapped drill
pair (wires 44, 45). mirror_x() is a no-op in v35 (P.POD_DIR 1); pack.py prints arms and augers with RM(-pi/2, Y).
INTERIOR: leg cradle | disc 1 (x 39.4) | deploy cradle x 43.6..56.4 (worm box under the leg's bridge) with DRV#1 / DRV#2 standing beside it |
disc 1b x 56.6..58.6 (B- spring on its rear face, top window |y| 13.2 above z 7 for the board wires) | cells x 62..136 | rear disc 137.2 |
sockets, capacitor on the tail key. Deck 1 / deck 2 boards start behind disc 1b (XIAO x 59). The deploy worm box sits under the righting
leg's bridge: P.LEG_CH (a channel x 9..24.5 in the leg-frame, 2.4 under, ±7 wide, in the bridge underside) + a hump in the hull top. Drill
leads through the tight holes at x DRILL_X + 10.5 (60.5), z 16.5. USB-C: the right pod covers the window folded AND deployed (plug box
checked at 0 to 46.75 deg): flash over USB before the pods are fitted, OTA after (run sheet).
REPRINT vs v34: hull_top, hull_bottom, tray, righting_leg (bridge channel). Same: nose (0.03 mm mesh noise), arms, augers, couplers, pod caps,
leg couplers, skin plugs, fit coupon + pegs. FIRMWARE: pin map unchanged; the deploy turns the other way (swap the deploy N20 leads at DRV#1
AOUT1 / AOUT2, or PINS[DEPLOY].sign -1); deploy 46.75 deg and its stops as v34; DRILL_SIGN +1; the swapped drill pair is now the LEFT one.
Results (out35): audit CLEAN at 1 deg (leg 179 steps, pods 48 steps), hard stops (deploy +2.5 deg 5.24 mm3, fold -3 deg 5.57, leg rest +2 deg
58.54, leg fold -2 deg 429.87), one shell per printed part, insertion sweeps zero, screw bite 10.1; ALL STEP VALID; launcher_fit FITS over
1684 states (rope tie ok; only base's old line model meets the rope, as v34); wire room OK; folded bbox x 0-152, y ±40.5, z -21 to 21.3;
printed 81.22 cm3; mass 249.6 g, CG folded x 75.57, z +0.39 (v34 83.1, +0.44); anchored z -3.83; leg out x 73.64, z +0.29.
Righting / tow (sim_righting/v35_check: run35_shared.mjs = run34_shared.mjs with the job's layout, here "front" pivotX 50; hull 192.1 g CG x 74.9
z -2.55, rigid axle, line tied in the tail face; summarize35.py -> v35_righting_tow.json): righted loam 80/80, sand 20/20, the 5 named poses
30/30, fixed arrival paths 25/35/45/57 deg 40/40 (median 7 s, slowest 17 s). Tow: deploy worm 0.00 N m (v34 tail tie 0.94), deploy stops
4.24 N m for both pods (about 2.4 a pod at 55 N, about 280 N on each stop face 8.5 mm from the pivot). Holding in the sim: loam moved at 45 to
49.5 N (median 49.0; v34 79 of 80 held 55), sand 17.2 (v34 18.3): the anchors are near the tail, so the pull pitches the nose up about the tail
edge (12 deg at 45 N), weight comes off the hull and the drills reach the sim's 25 N isotropic soil cap; each drill is loaded about 95 %
sideways (only about 4 N outward along its axis), which that soil model does not credit: a pull test decides. Flight (flight35.py ->
phys35/flight35.json, flight34's model, tail tie): nose first at every rail 6 to 47 deg, tow arm 76.4 mm (v34 68.9), neutral speed 11.7 m/s,
tip-off AoA max 23 deg (v34 28), sideslip 4.
Review renders: out35/review/ (review34.js), out35/audit_sheet.png, out35/sections.png, out35/img_exploded.png + img_stage1..4. Viewer: pub35.

v34 (9 Nov 2026, dart-cad4) = the SELF-RIGHTING layout from the righting study (kedge/sim_righting, results.json, published artifact
TGTtc2um8jqGDkG5ic6hQA). Running handover: kedge/V34_PROGRESS.md. Harry's rules for v34: simple and working (NO latch: plain hard stops,
the self-locking worm may carry a little), own pivot pins in hull bushings where simple, sealed body, assembly that makes physical sense.
CONVENTIONS (new): moving roots in parts.json carry "sgn" (rotation sign about +y; -1 for both groups here) and "joint" ("leg" or
"deploy"); world = T(pos) * RotY(sgn * deg) * local. audit27, solve_deg, clearance, massprops, viewer.html read them. The leg part is
"righting_leg"; P.KICK_DEG is kept as an alias of P.LEG_DEG for old tools. v34 audit belly rule: below z -5 (the launcher cradle ring
tops) nothing between the hull and r 26.
PODS AT THE TAIL: pivot x 132, z 9.5 (study x 140, z 10: x 132 leaves room for the leg pad in front of the flat tail face the pusher
needs; z 9.5 keeps the CG near the axis), drill axis |y| 31.15, folded FORWARD along the flanks, deploy solved 46.75 deg
(the motor end inside the hub 0.5 above flat ground), drills 43.5 mm deep entering the soil about 106 from the nose tip. v33 arms and
tapered gimlets, built pointing +x and mirrored (hands kept: left pod right-hand, DRILL_SIGN +1). One deploy N20 on the axle at
(132, 0, 9.5), motor pointing DOWN in the tray's rear cradle (the worm box rests on the tube's front/back walls), PPA-CF couplers
running in the pivot-block bores, pod caps + M3 x 8. dart-cad5: every coupler is a PRESS FIT with CA, no play (stub socket Ø3.05 with the
flat 1.03 from the axis on the Ø3.0 N20 stub; arm-head D bore 7.05 on the Ø7 coupler; cheek D bore 6.05 on the Ø6 leg coupler; set with
the pods / leg on their fold stops and the worm driven to stall toward fold): the axle is stiff, so both pods load the one worm together
(review section 5: median 0.13 N m, under the 0.21 stall). Fits on the fit coupon v34 row + fit_coupon_pegs.stl (PPA-CF). Pivot blocks: v33 method mirrored (deploy stop
behind the head, fold stop under it), smaller circles, kept out of the bay. NO LATCH (Harry's update): instead the BRIDLE EYE is a lug on
each arm head's outboard 5 mm, 3.9 forward and 6.8 below the pivot folded (behind and below it deployed, about 23 above the soil), so
the tow pulls the pod onto its deploy stop (about half the study eye's moment: it cannot hang lower, the launcher's stop springs pass
0.6 mm under it at the shot). Drill leads through tight Ø3 holes at x 121.5, z 16.5 (silicone).
TETHER (9 Nov, Harry: "keep it simple"): the rover line goes in one of two Ø2.0 holes in the middle of the tail face (hull_bottom end wall, y 1.6 and 5.4, z -2.4, inside the launcher pusher's Ø14 hole), out the other, and is tied to itself (bowline + stopper) right behind the tail; it runs straight back. Thread it BEFORE the tray goes in. The bridle grooves are gone (P.TETHER="tail"; "bridle" rebuilds the grooved bridle). The arm-head eye lugs stay as spare tie points. Cost (sim_righting/v34_check/v34_tail_tie_worm.json): with the pull on the hull the deploy worm holds the pods against the soil moment, about 0.9 N m at 55 N in loam (0.36 in sand at the 18 N slide), against 0.13 with the eye bridle: start with the reel limit about 25 N and bench-check that the unpowered worm holds about 1.5 kg pulled back at a drill tip. No tail boom, no tail cap: the two hull halves have integral 2.5 tail end walls (flat pusher face, edge r 2.5).
RIGHTING LEG (no foot, no foot motor; the foot N20 is a spare): hinge x 33, z 14.5 on the centreline, swing 0 to 178 deg, bed 1.5 deg
further on. The leg N20 (old kick motor) sits on the tray, motor pointing down, worm box in the KNUCKLE under a 0.6 skin blister (box
top 20.5); both stubs trimmed to |y| 10.8. Fork root: a round cheek each side of the knuckle in a side pocket (|y| 7.6..11.4), keyed on a
Ø6 PPA-CF LEG COUPLER that runs in the knuckle-wall bore and in the blind bore of a flush SKIN PLUG (Ø7.5, pressed in a counterbore,
CA); press fits with CA on the stub and in the cheek (no grub screws); plug + worm box hold it axially. The stub slots under the knuckle (the hull
top is lowered over the tray-mounted motor) are closed by two TONGUES on the tray's leg cradle. Blade 8.5 wide, I-channel (1.3 skin, 1.5
walls), its top IS the hull crown, flush in the dorsal groove (floor 0.15 under it = the fold stop); bridge behind the knuckle (r >= 10.2,
swings over it). Pad = a 2 mm patch of the hull skin (+-62 deg, x 143.5..149.3) on the tail top, centre 113 from the hinge (study 128),
about 37 wide (study 56): limited by the launcher pusher (tail face) and the folded pods' hubs. Forward of the hinge the groove becomes
the bed the leg root lies in at the far rest (a swept saddle in the hull front and the nose top; camera lowered to z -2.4, 1.2 over it).
LOW CG: cells IN LINE on the belly (z -10: front spring B-, an AA-holder flat plate as the BM tap between them, rear plate B+), located
by four U saddles on the tray (open 19.2 at the bottom: they slide in, the belly carries them; no belly rails any more), deck 1 is the
tray's backbone right over them, deck 2 for the VTX and MP1584, DRV#1/#2 standing beside the deploy motor, both worm motors pointing down.
BALLAST (dart-cad5, from the v34 physics: righting works from any attitude, the putty helps flight): P.BALLAST_G 14.6 g of Harry's putty in
the NOSE, the lower part of the hollow round the camera tube from the front back to a 45 deg retaining collar at x 13.2 (P.BAL_XE), pressed
in from the back before the nose goes on; part "ballast" (mass_g).
SEALING / ASSEMBLY: deck 1's edges sit in a seam groove at the split (both halves clamp them, they back the seam); nose spigot + lock rib
(keys the front); a TAIL KEY: deck 1 continues as a tongue into slots in both end walls (keys the unfastened tail; a lip across the split
cannot print with both halves split face down); the JST sockets in flank seats behind the pivot blocks (left 2-pin ON loop, right 3-pin
charge B3B-XH-A: opening 10.2, pin pocket 7.0), clamped by the halves; the USB-C window open down to the split (a tray tab fills it under
the receptacle) so the hull top drops straight on; 8 radial M3 x 8 into solid 6 mm bosses on disc 1 (x 39.8) and the rear disc (bite
10.1 mm3 each of 11.9 for a full 7 mm: the countersunk head takes the rest); capacitor modelled Ø8 x 12, standing on the tail key;
tray prints on a flat underside (z -16). Assembly order and access: pack_v34/KEDGE_v34_run_sheet.pdf
(tray populated in the open; tray into the bottom; drill leads in; hull top lowered over the stubs; 8 radial screws; pods; leg + plugs).
Results (out34, dart-cad5 9 Nov): audit CLEAN at 1 deg (leg 179 steps to 178.0, pods 48 steps to 46.75), cross sweep, ground, cradle-zone belly rule, hard stops (deploy_at_angle 0.0, deploy_horn_+2.5deg 5.23, fold_at_0 0.0, fold_lug_-3deg 5.57, kick_rest_-0.5deg 0.0, kick_rest_+2deg 58.54, leg_fold_at_0 0.0, leg_fold_-2deg 449.61), watertight, one body AND one mesh shell per printed part (new check), insertion sweeps zero overlap (new: tray into the bottom, hull top onto the tray, 0.25 mm steps), screw bite 10.1 mm3 at each of the 8 screws (new table); ALL STEP VALID; launcher_fit FITS over 1684 states with the stowed rope modelled (the rope only meets base's old line model); rope checks: stowed legs in the grooves, 3.26 mm over the ON plug + 10 mm loop, towed legs 0.9 mm clear of the dart (Ø1.2 model line, unchanged geometry); wire room OK (corridor fills 0.27 / 0.10); folded bbox x 0-152, y +-40.5, z -21.0 to 21.3; printed 79.95 cm3 (99.2 g); mass 248.0 g incl. 14.6 g nose putty, CG folded x 83.14, z +0.44; anchored z -3.81; leg out x 81.14, z +0.34 (out34/mass_props.json totals). physics.py / flight_sim (dart-physics) still fail on out34 (kick_leg / foot hard-coded); dart-physics' own v34 runs: kedge/DART_PHYSICS34.md.
Review renders: out34/review/ (01-22 review34.js; c01-c24 = the dart-cad5 fixes, views_cad5.json; h1-h3, L1-L4, R1-R6, parts/ = views_misc.json), out34/img_exploded.png and img_stage1..4
(assembly stages), out34/audit_sheet.png, out34/sections.png. Viewer data: pub34/ (the viewer draws the stowed rope in the grooves when the pods are folded).


v33 (7 Nov 2026, dart-cad3) = v32c with Harry's TAPERED DRILLS and a gimlet foot auger. Everything not listed here is v32c (below).
DRILLS: each drill gearmotor still sits INSIDE its auger, but only in a short hub: Ø18.7 (bore 16.3 over the gearbox diagonal 15.62) from the
arm shoulder (8.7 from the pivot) to 1 mm past the gearbox face (36.5). Decision (hub vs a stationary housing with the auger from the motor face):
the 15.62 diagonal makes any wall round the motor about Ø18.7 whether it turns or not, so a stationary housing is no slimmer; in the hub the v32
bearing path stays (hub on the arm's 17.7 mm bushing journal, drilling push on the arm shoulder, the shaft only drives), the motor goes in from
the front with no cap, and there is one seam (at the head) instead of two. Past the hub: a 9 mm tangent S-curve neck (cosine, no step) to a
GIMLET: core Ø9.6 (holds nut A with 1.2 walls) tapering to Ø6.0 at 87.3, then a point over 14 to 101.3; anchor (buttress) flight OD 16, pitch 12,
full height from 45.5 to 79.3, crest tapering to the point over 22; load face toward the motor 45 deg (prints point up without supports), leading
face 30 deg, crest land 0.6; L/R opposite hands (left pod right-hand). Provisional: FLIGHT_D, D_PITCH, D_CORE0/1, D_TAPER, D_TAPER_F, D_LOAD_DEG,
D_LEAD_DEG, D_LAND are parameters for Harry's T12/T16/T20 bench results (KOV overrides). STEEL SPINE: the motor's M3 shaft (2.84 x 56) screws
through M3 nut A pressed in a hex pocket at the apex of the hub bore's 45 deg roof (x 40.5) and self-taps a Ø2.6 bore to x 92.0, 10.2 mm from the
point (0.8 PLA wall round its end); threadlocker on the shaft: nut + 49 mm of self-cut thread key the auger both ways. Drive is direct (the auger
turns with its motor, DRILL_SIGN +1). Assembly: nut in, threadlocker, screw the auger on until the hub base touches the shoulder, back off half a
turn. No tip cap, no nyloc any more. DEPLOY 50 deg (DEPLOY_TARGET 50, was 45): arm and motor 2.0 above the ground (pod head 7.9), hub rim about
1 mm in the soil; drills 44.8 mm deep; pivot-block stops carved at 50. Drill axis |y| 31.15 (R + hub/2 + 0.8; was 32.8). Spin torque estimate
(c D^2 L): about 0.35 of the v32 sleeve, release margin about 2.2 on the in-hand 100 rpm motor (v32 0.78): dart-physics to confirm.
FOOT: same housing, tilt, plant (KICK_DEG 200.75, vertical, 67.9 mm in) and drive; the auger is a gimlet: OD 13, pitch 11, Ø9 x 9 hub with the
D-bore and grub screw, S-curve neck to a core Ø7 -> Ø4, 12 mm point, crest taper 20, same buttress flight (no spine: the worm output is short).
KICK: Harry asked for the foot IN LINE with a straight kick leg (folds flat on the hull top, swings about 250 deg, plants 60 to 75 deg ahead of the
nose). NOT BUILT, because it cannot clear the body with the hinge on the hull top: any straight leg from the hinge (x 31.3, z 27.5) steeper than
about 24 deg runs through the nose, hull or camera (overlap 1300 mm3 at 45 deg, 707 mm3 at 70 deg; side view out33/review/27_kick_inline_check.png),
and at 24 deg an in-line foot needs a 190 mm leg to get 30 mm deep. With one y hinge, "folds flat" means "in line with the leg", so a steep foot
that folds flat needs the hinge outside the body's silhouette. Options for Harry: (a) keep v32c's vertical foot (folded tip z 101.9, fits the
launcher); (b) tilt the foot to plant 70 deg forward on the same leg (FOOT_TILT 40.75): tip only 13 mm lower (z 88.5) but at x 186, past the
launcher rail end at full cock, and the foot only 44.6 mm in (not worth it); (c) straight leg + in-line foot with the hinge on a tower at the nose
front (hinge about 15 mm above the nose skin at x 5 to 10, the kick N20 moved there, in the impact zone, in front of the camera at the plant);
(d) a second (passive) joint at the foot (single hinges were the rule). v33 keeps (a).
Results (out33): audit CLEAN at kick and drill 1 deg steps (202 / 51), cross sweep, ground, belly, hard stops (deploy +2.5 deg 1.69 mm3, fold -3 deg 3.34, kick rest +2 deg 90.56), watertight, one body per printed part; ALL STEP VALID; wire room OK;
launcher_fit FITS over 1684 states (only base's tether model overlaps, as v32). Folded bbox x 0-179.5, y +-40.5 (was 43.8), z -21 to 101.9. Printed 99.69 cm3 (v32c 112.07); mass 270.3 g, CG x 83.3, z +7.9 (v32c 286.8 g, x 87, z +8.2); per-part volume, centroid, inertia in out33/mass_props.json.
Review renders: out33/review/ (01-14 review.js, 15-26 review33.js close-ups, 27_kick_inline_check.png), out33/review/pub (viewer page),
out33/audit_sheet.png, out33/sections.png. Viewer data: pub33/.

v32c (generator `kedge_v32.py`, out32; the base of v33):

Coordinates: x along hull, nose tip x=0, tail x≈152. y across (+y = LEFT), z up. Ground z = -21 lying flat.
v32 = Harry's anchoring layout (7 Nov 2026): drill motors inside the augers, kick leg on the centreline, tether on a tail boom.

Key parameters (all in `out32/parts.json` "P"; harness way points in P.WIRE_PTS). Hull, nose, tray, boards as v31:
R_OUT 21, R_BAY 19.5, L 151.97, layout "cells_fwd" (sled + disc1 | cells | boards | disc4 | tail cap), keel wire channel,
XIAO USB-C window, BNO085 in the sled, power-loop socket on top of the tail cap, charge socket underneath.
DRILLS (one deploy worm N20 on the axle, as before): pivot x 56, z 12, sleeve axis |y| 32.8, DEPLOY_DEG 45 (DEPLOY_TARGET:
no ground limit any more, the arm clears by 8.9). Each drill gearmotor sits INSIDE a hollow sleeve auger: the arm (drill_leg_L/R:
round head on the Ø7 D coupler, neck, thrust shoulder, bushing Ø15.8) holds the motor by its can (flats key it, 1.2 mm tab room,
leads out of the neck top); the sleeve (drill_leg_L/R_auger) has bore 16.3 (gearbox diagonal 15.62), core 18.7, flights Ø22,
pitch 15 (dart-physics), 1.2 to 2.0 thick, 45 deg lower flank, L/R opposite hands, 100 long from 8.7 mm off the pivot; it runs on
the bushing and the shoulder (push goes into the arm) and the motor shaft drives its tip hub through a jammed pair (M3 nut in a
side slot + nyloc tightened through the tip bore): keyed in both directions. Drills 49.1 mm deep.
STOPS: one faired pivot block per flank (tangent-arc teardrop to the split plane) is the coupler boss and, carved by the swept arm
(+HORN_GAP 0.15), leaves the deploy stop in front of the head and the fold stop under it (both about 1 deg past the ends).
SKIN FEATURES (v32c): saddle, ear and pivot blocks are tangent-arc outlines (arc_prism: the convex hull of circles as true arcs joined
by outer tangents, no polygon facets), side edges r 1.2 (saddle outboard shoulder r 4.5), blended into the skin with an r 1.5 skirt
(skirt(): local skin patch + fillet). Arm heads: face edges r 1.2, stop corner r 0.5.
KICK on the centreline: kick N20 beside it at y -11.9 in a low saddle fairing (its unused -y stub cut to 1 mm past the worm box, so
no outboard slot; no strap: the square worm box keys it in the square pocket, a drop of CA stops rattle), hub on its +y stub (y -4..7.5),
pivot z 27.5 (KICK_RAISE 0), teardrop ear with the F693ZZ on the left, pole and foot housing at y 0, plant KICK_DEG 200.75.
At the plant the pole lies in a round-cornered FLUTE in the nose top (the pole's underside at the rest angle grown 0.25, corner r 3.8;
the pole's datum-side corners are 45 deg chamfers 3.5 to fit it; a solid strip under it keeps the nose closed, 1.6 over the camera tube);
the hull wall is slotted over the nose spigot there. The flute floor is the KICK REST (1.5 deg past the plant; audit checks it
against hull_top + nose). FOOT: the housing is tilted FOOT_TILT 20.75 deg (= KICK_DEG - 180) about the foot N20's output shaft and
raised FOOT_RISE 3.4 (folds clear of the hull top), joined to the pole by a web: the foot auger plants VERTICAL with the housing flat
on the ground, 67.9 mm in (was 59.3 at 24.5 deg); its tip meets the soil near 170 deg. Foot auger Ø13 x 68 (core 6); the worm
N20 cannot go inside a hollow foot auger (its output is at right angles to the motor). Folded foot tip at z 101.8, x about 164 (leans
aft over the tail): launcher room per 4 mm x band (probe of the real launcher states, |y| <= 8) leaves 8.7 mm at x 152-156, 13 to 18 elsewhere.
TETHER: PPA-CF tail boom on the axis, 28 mm behind the tail (the longest the launcher allows: rail rear and top pulley at full cock),
9 x 9 rounded, clevis eye with a steel M3 x 8 countersunk pin (flush); a 9 x 12 head sits in a PPA-CF anchor disc in the tail
spigot and the four tail M3 screws run hull -> tail cap spigot -> anchor disc: the tow goes into the hull. Square shank keyed in
the tail cap end wall; the pusher face is still the flat end ring. No fins (they would hit the pusher's Ø14 hole).

Printed parts (17 + fit coupon): hull_top, hull_bottom, nose, tail_cap, tray, kick_leg, foot_auger, drill_leg_L/R (arms),
drill_leg_L/R_auger (sleeves), coupler_L/R, pod_cap_L/R (PLA) + coupler_L/R, tail_boom, boom_anchor (PPA-CF plate C).

Results (out32, v32c): audit CLEAN at kick and drill 1 deg steps, cross sweep, ground, belly, hard stops, watertight, one body per
printed part; all STEP valid; wire room OK. Folded bbox x 0-179.5, y ±43.8, z -21 to 101.8. Printed 112.07 cm3 (138.9 g at 4 walls / 40%);
mass 286.8 g with catalogue bought masses, CG x 87, z +8.2 (out32/physics.json; per-part volume, centroid and inertia in out32/mass_props.json).
launcher_fit FITS against base/cad/parts/launch.py over 1684 states; the only overlaps are base's tether model, which still ends at
the old tail holes (base moves the line end to the boom eye: DART_V31_NOTES.md lists what base needs, incl. 22 mm for a 50 mm boom).
Review renders: out32/review/ (close-ups, viewer page shots), out32/audit_sheet.png, out32/sections.png. Viewer data: pub32/.

v29 (Ø16 x 70), v30 (Ø17 x 72), v31 (Ø13 x 72.8 radial augers on F693ZZ pairs, kick offset at y 14.9, tether holes in the tail
cap) are in out29..out31 / dist_v29..dist_v31.

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
- Stop faces are carved, not placed: subtract the arm's 2D profile swept over the swing (+HORN_GAP) from a faired block (pivot_blocks).
- Every rotating part checked against the launcher (launcher_fit.py) before packaging; probe the room first (boom length, foot height).
- Name clash bit v32 once (SH_X1 in the tray vs the arm): prefix new layout constants.
- OCC fillets: fillet a sub-solid before the boolean that makes awkward edges (arm head before the shoulder union); skirt features into
  the skin through a local patch (skirt()); one skirt per feature group (saddle + ear together failed with the blocks); fillet only the long
  lip edges of a groove; chamfer (not fillet) edges that sit on the print bed.

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
- `GEN=kedge_v33.py run_all.sh out33 v33 [stepK stepD]` (GEN default is kedge_v32.py = v32c; GEN=kedge_v27.py rebuilds v31) : the whole pipeline (generate, solve and regenerate at the solved angles, audit,
  launcher fit, clearance, meshes, shots + sheet, pack, mass props, STEP check, physics, coupon, wires, wire room, wiring
  diagram, barrel, flight sim, assembly STEP, plate view, run sheet, engineering report), about 30 min. `make_dist.sh v31` : zips.
- `kedge_v32.py OUTDIR` : v32 generator (`kedge_v27.py` = v31). `KOV='{"AUGER_D":17}'` env JSON overrides any P.
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
- `review.js OUT` + `montage.py` : close-up renders to OUT/review; `pubshots.js PUBDIR OUTDIR` : shots of the published viewer page.
- `viewer.html` : three.js viewer (pose sliders, explode, section cut, audit panel, Play deployment animation).
Outputs:
- `out35/ pack_v35/ dist_v35/ pub35/` : v35 (`GEN=kedge_v35.py ./run_all.sh out35 v35 1 1`, `GEN=kedge_v35.py ./make_dist.sh v35`; close-ups `node review34.js out35`; v35 tools: runsheet35.py, report35.py (runsheet.py / report.py dispatch on P.POD_DIR 1), wires34.py (v35 routes when P.POD_DIR 1), flight35.py (-> phys35/flight35.json), sim_righting/v35_check/ (run35_shared.mjs, summarize35.py, v35_righting_tow.json). out35/READY = the CAD is final (the Blender pipeline waits on it).
- `out34/ pack_v34/ dist_v34/ pub34/` : v34 (`GEN=kedge_v34.py ./run_all.sh out34 v34 1 1`, `GEN=kedge_v34.py ./make_dist.sh v34`; close-ups `node review34.js out34 [views.json]`; v34 tools: wires34.py, runsheet34.py, report34.py, ovl.py (overlap locator), qrender.py).
- `out33/ pack_v33/ dist_v33/ pub33/` : v33 (`GEN=kedge_v33.py ./make_dist.sh v33`; close-ups `node review33.js out33`, kick check `kick_inline_check.py`).
- `out32/ pack_v32/ dist_v32/ pub32/` : v32 (pub32 = index.html + parts/meshes/audit json for the viewer artifact; review
  renders in out32/review/).
- `out31/` v31 data (STL/STEP/NPZ per part, parts.json, audit.json, launcher_fit.json, wireroom.json, mass_props.json,
  physics.json, renders, audit_sheet.png, sections.png). `pack_v31/` print pack (STL, 3MF plates A/B/C, STEP, print_report.json,
  step_check.json, wire list, wiring.png, barrel files, run sheet and engineering report PDFs, assembly STEP). `dist_v31/` zips.
- `out29/ out30/ pack_v29/ pack_v30/ dist_v29/ dist_v30/` : the previous versions.
- Firmware lives in `../firmware/dart` (v4, WIRING.md, BENCH_TEST.md); `fw/` here is the old v3 copy.
- `pub/` files for the claude.ai viewer artifact (https://claude.ai/artifact/Sqt8tF67veTYfxQjUFTZ5f, now showing v35 from pub35).

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
