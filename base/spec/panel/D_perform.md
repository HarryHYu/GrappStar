# GrappStar base station: integrated architecture, PERFORMANCE lens (D_perform)

Planning document, 3 Nov 2026. Units mm, N, N m, s, deg. Every number below comes from the scripts in
`scratchpad/calc/` (launch.py, shot.py, motor.py, aim3.py, reel.py, cd.py, pts.py), run with the kedge venv.
Assumed values (wiper, spring hang test, Dyneema friction) are marked ASSUMED and listed in section 9.

---------------------------------------------------------------------------------------------------------------

## 1. Concept summary

1. One wiper drives a Ø20 keyed PPA-CF main shaft, coaxial with the wiper output (axis +Y). A drum-moved dog collar on the main shaft selects the LAUNCH stub (coaxial, +y) or the REEL gear (free on the main shaft, -y). NEUTRAL couples neither.
2. Launch: the stub carries sprag #1 inside a 32T drive gear on the rail pivot axis. This meshes a 48T big gear on the rail (40 teeth present, relieved last tooth), which drives the carriage rack. One forward stub turn and a half = cock 245 mm + fire + reset. The drive gear's reaction pushes the rail nose down, same as v10.
3. Springs: both Century springs direct 1:1 (no pulley; a pulley gains nothing, see 5.3). 5 mm preload, 250 mm at full cock, 11.2 J. Anchor holes for 275 and 300 mm are ready for after the hang test (12.8 J and 14.3 J).
4. Aim: backward stub turns go through a pawl-free swing-arm splitter, a 1:1 bevel and a self-locking single-start worm (lead 5.7 deg, 25:1) to a crank-rocker. Pitch runs -10.3 to +45.4 deg, minimum transmission angle 60 deg. The worm holds the pitch while cocking and through the shot kick.
5. Reel: a 48T REEL gear on the main shaft drives a 24T idler, then a 16T capstan gear (capstan turns 3.0x the main shaft). Sprag #2 is the backstop on the idler shaft, so the capstan only turns wind-in. The Ø40 capstan takes 6 wraps from Harry's v7 wrap cup.
6. Storage reel (Ø50 core, 22 wide, Ø66 flanges) spins on 2 x 608 on a fixed axle. A printed slip plate is the payout brake (0.5 N LAUNCH) and also the take-up drive.
7. CONCEPT CHANGE C1: the take-up is driven from the main shaft ahead of the dog, so it runs in every mode, through a printed face ratchet. This keeps the line taut while the dart rides back 245 mm during cocking. The wiper's self-locking worm is the brake reaction.
8. Selector drum: single-turn barrel, N20 worm through 10T:72T. Fixed order: LAUNCH 0 / NEUTRAL 40 / REEL 80 / WRAP sector 95 to 335 (6 wraps) / WRAPPED 335 to 350 / hard stop. Three tracks set the fork, the preload and the code bumps. A 240 deg segment gear with locking arcs drives the cup. Manual lever on the shaft end; the N20 swings out.
9. Line in LAUNCH: dart, pusher hole, rail, R2 swivel block (rail breech), E swivel block (frame), cup ring, guide eye, reel. Every bend over 10 deg runs on an F693ZZ roller.
10. Performance at the real springs (0.5 N brake, 37 deg): 7.2 m/s, 4.8 m range; 6.0 m with the 300 mm hole. Shot cycle 2.3 s. Capstan 0.29 m/s free, 0.25 m/s at 30 N, 145 N at 70 % stall, 207 N stall.

### Concept changes (flagged, with numbers)

**C1. Storage take-up from the main shaft (always geared, printed face ratchet), not from the capstan train.**
- The problem. The dart has to ride back with the carriage during cocking. If it does not, the carriage hits a stationary 279 g dart at 7.3 m/s, and momentum transfer gives the dart only (1+e)·45/(45+279) of that speed.
- With the baseline, the take-up only runs in REEL, so every cock leaves 245 mm of line slack between the pusher and the breech. That slack lands in the rack, big gear and spring zone.
- At fire, that slack also turns the reel spin-up into a snatch. Energy loss is the same 0.66 J, but the snatch puts a shock load on the tail cap bridle and line. Dyneema is stiff, so the peak would be tens of N.
- With C1, forward turns in LAUNCH wind the reel at 2.8x the rack speed through the slip clutch (0.5 N). The line stays taut, and the dart is held on the pusher by 0.5 N plus gravity (1.4 to 1.6 N at 35 to 40 deg).
- C1 also gives NEUTRAL + forward as a pure take-up mode. It recovers the post-landing overrun (0.9 m) and tensions the line before wrapping, with no extra part. The take-up gear just moves from the capstan train to the main shaft.
- The payout reaction is still positive in every mode: plate, ratchet (locked), main shaft, self-locking wiper.
- Sprag #2 stays on the capstan train as the backstop. The capstan and launch trains stay separate. Only the slip-limited take-up (at most 1.5 N at the line) is common to both.

No other concept change. Everything else below is detail inside the baseline: the -10 deg retrieve pitch, the swivel blocks, the drum track 2 profile, the NEUTRAL brake ramp and the 6 wraps.

---------------------------------------------------------------------------------------------------------------

## 2. Kinematic layout

### 2.1 Frames

- WORLD: origin on the wiper output shaft axis at the adapter face. +X forward (launch azimuth), +Y left, +Z up. The main shaft, launch stub and rail pivot all lie on the Y axis (x = 0, z = 0).
- RAIL frame: u forward along the rail, w up perpendicular to it, same y. Pitch φ is positive nose up. Conversion: x = u cos φ - w sin φ, z = u sin φ + w cos φ. A positive rotation about +Y moves the nose DOWN. Forward (cocking) = +ω about +Y, which is clockwise seen from the right side (x right, z up).
- Base plate top at z = -135. The pivot sits 135 mm above the plate; the rover deck is below it.

### 2.2 Main shaft and stub stack (axis Y at x = 0, z = 0)

| y range | part | note |
|---|---|---|
| -60..0 | wiper gearbox, ASSUMED box x -45..45, z -40..40 | motor can points +X at y -30 (x 45..155), clear of the rail (y >= 72) and of the reel module (x < -60) |
| 0..12 | coupling hub, PPA-CF | DD socket 10 x 8 for location; torque through the original steel crank arm clamped in a slot (2 pins at r 25 to 35); M8 nut. Interface parametric |
| 12..22 | bulkhead A | main shaft bushing (Ø25 section); same plate carries the idler and capstan bushings |
| 24..34 | TG1 60T m1.5, keyed | take-up drive (C1) |
| 38..53 | RG 48T m2.5 face 15, free on bushing | REEL output; 6 dogs on +y face, y 53..57 |
| 59..79 | dog collar (NEUTRAL), keyed slide | body 12 + 4 mm dogs each face; travel ±6 (LAUNCH y 65..85, REEL y 53..73); fork groove y 66..72 |
| 81..91 | stub dog flange | dogs y 81..85; main shaft ends y 80 with a Ø10 pilot into the flange |
| 93..103 | bulkhead B | stub bushing Ø20 |
| 105..113 | rail boss -y | rail rides on the stub (v10 style) |
| 114..130 | DG 32T m2 face 16, PPA-CF, on sprag #1 (y 115..129) | rail centre plane y = 122 |
| 131..139 | rail boss +y | |
| 141..151 | bulkhead C | stub bushing; pedestal straight down (keeps the rear-upper sector free for the line) |
| 153..163 | AG 40T m1.5, keyed | aim input; swing arm pivots on the stub y 163..167; stub ends y 168 |

### 2.3 Every shaft and body (world)

| item | position (x, y, z) | axis | note |
|---|---|---|---|
| main shaft / launch stub / rail pivot | (0, 0..168, 0) | Y | one line; stub separate shaft from y 81 |
| big gear BG 48T m2 (on rail) | rail (u 80, w 0), y 114..130 | Y | world (80, 0) at φ 0; (79, -14) at -10; (57, 57) at 45 |
| rack (on carriage) | pitch line w = 48, y 122 | along u | m2, 40 teeth + end teeth, 250 long |
| swing idler SI 12T m1.5 | on arm about main axis, engaged (6.9, 160, -38.4), free stop (-20, 160, -33) | Y | backward = CCW from right side swings to +x into mesh |
| aim input AI 12T m1.5 | (0, 153..186, -55) | Y | bevel 15T m1.5 at y 186 |
| worm shaft (worm m2, 1 start, Ø20) | z -55, y 186, x +10..-135; worm at x -110 | X | bevel apex (0, 186, -55); thrust shoulder both ways |
| worm wheel WW 25T m2 face 12 + crank | (-110, 180..205, -90) | Y | crank a = 28 at y 205 |
| conrod b = 132 | plane y 205 | | to rail lug |
| rail lug | rail (u 0, w -60), y 199..211 | Y | world (0, -60) at φ 0; (-10, -59) at -10; (42, -42) at 45 |
| idler IG 24T m2.5 + TT 60T m1.5 (free) + sprag #2 | (-90, 10..64, 0) | Y | Ø20 keyed shaft; sprag #2 y 22..36, outer race keyed in bulkhead A boss; must stay at y < 66 (rail sweep) |
| capstan sleeve (Ø24, bore 12.5) + CG 16T m2.5 | (-130, -45..53, 30) | Y | CG y 38..53; drum Ø40 y -32..8 (base flange y 8); cup y -39..-25; ring plane y -39.5 |
| cup shaft Ø12 inside capstan sleeve | (-130, -45..62, 30) | Y | cup gear 24T m1 at y 56..62 |
| storage reel axle (fixed, Ø8) | (-140.0, -48..34, -28.9) | Y | reel y -40..-14 on 2 x 608, plate y -13..-9, plate gear PG 17T m1.5 y 24..34 (sleeve, face ratchet between PG and sleeve) |
| AS5600 | on PG outer face centre (axle ends inside PG) | Y | PG turns 3.53x main shaft in every mode, before the ratchet |
| selector drum Ø56 x 70 | (-80, -50..20, -70) | Y | ring gear 72T m1 at y -50..-44; segment gear y 14..20 |
| N20 worm (biaxial) | pinion 10T m1 at CD 41 from drum, e.g. (-121, -47, -70) | Y | on swing plate with snap latch |
| fork rod | (-45, -30..70, -45) | Y | follower in drum groove 1; fork at y 69 onto collar groove (arm 40) |
| wrap transfer | pinion 12T m1 at CD 42 from drum, 36T on its shaft, idler 40T m1, cup gear 24T | Y | positions chosen by the reel worker on the drum-to-capstan line (112 mm, sum 42 + 38 + 32 = 112 OK) |
| R2 swivel block (rail) | rail (u -175, w 85), y 122 | swivel along u | world (-175, 85) at φ 0; (-158, 114) at -10; (-184, -64) at 45 |
| E swivel block (frame) | (-175, -67, 55), ASSUMED for the reel worker | swivel along inbound line from the ring | R2-E length 191 to 223 mm over the aim range |
| springs (2) | rail (w 73, y 122 ± 34), from carriage pegs to anchors at u 328 | along u | 16 OD, under the dart equator, clear of pods by 3.5 to 5.5 |
| dart (rest) | rail u 158..325, axis w 85, y 122 | along u | belly z_dart -21 sits at w 64 on the carriage cradle |
| dart (cocked) | rail u -87..80 | | pusher at -87 |

### 2.4 Rail geometry (rail frame)

- Length u -180 to +345 (525 mm), printed in 2 pieces (spliced at u +80 over the big gear boss with M3 + inserts).
- Channel inner ±43 at y 122, so the walls sit at y 74..170. The floor is at w 35..40 with a slot at u 55..105 for the big gear.
- V grooves on the inner walls at w 60.
- The rack sweeps u -165..+325 because it travels its own length. This is what sets the rail length (2 x stroke), as in v10.
- Pusher at rest u 158, cocked -87. Dart nose at rest u 325. Muzzle lip u 330..345 with a roll funnel (±45 flaring to ±70).
- Spring anchors (3 holes) at u 328 / 353 / 378 for preload 5 / 30 / 55 mm. The 30 and 55 mm holes sit on a removable nose block.
- Front stop: a hard stop at the carriage rest position, plus a fin-pinch friction brake over the last 15 mm (2 PPA-CF flexure shoes). The pinch absorbs about 0.85 J and the hard stop 0.35 J, so the carriage always ends on the hard stop (rack phase defined) with no rebound.
- Rail mass about 0.75 kg printed + 0.11 kg springs + 0.28 kg dart. CG near u +110, so the gravity moment is about 1.4 N m nose down, held by the worm.

At min and max pitch (world x, z), from pts.py:

| point (rail u, w) | φ -10 | φ 0 | φ 37 | φ 45 |
|---|---|---|---|---|
| muzzle dart axis (325, 85) | (335, 27) | (325, 85) | (208, 263) | (170, 290) |
| muzzle floor (340, 30) | (340, -29) | (340, 30) | (253, 229) | (219, 262) |
| breech floor (-175, 35) | (-166, 65) | (-175, 35) | (-161, -77) | (-148, -99) |
| R2 (-175, 85) | (-158, 114) | (-175, 85) | (-191, -37) | (-184, -64) |
| big gear centre (80, 0) | (79, -14) | (80, 0) | (64, 48) | (57, 57) |
| lug (0, -60) | (-10, -59) | (0, -60) | (36, -48) | (42, -42) |

Lowest moving parts:
- Breech floor at -99 at 45 deg, crank circle -118, big gear -64 at -10. All clear the base plate at -135.
- The muzzle at -10 is at z -29, so the retrieve funnel sits 106 above the plate and overhangs the front.
- Departure height about 0.56 m above ground at 37 deg (deck about 0.15 m). The sims use 0.5 m.

### 2.5 Line path (all lengths at φ 0)

LAUNCH (wraps off, cup at home):

```
dart tail cap (2 x Ø4.2 bridle) -> pusher Ø14 coned hole (no bend, line on the rail axis)
  -> along rail u (w 85, y 122), 333 mm at rest, 88 mm cocked (the take-up absorbs the 245 mm)
  -> R2 swivel block on F693ZZ, 90 to 112 deg bend
  -> 191..223 mm free span to E (length changes 32 mm over the aim range; the take-up recovers it while cocking)
  -> E swivel block on F693ZZ, under 30 deg
  -> cup ring (home) -> across the capstan top -> guide eye
  -> storage reel, tangent at r 25..29.9
```

- Guide eye, home ring and E must be collinear within 10 deg in LAUNCH.
- Note: Harry's v7 spacing (guide 21 behind the base flange, exit 28 past the ring) bends the line about 31 deg at the ring. That is 11 % extra drag through a 2.8 mm hole. The reel worker keeps v7's wrap rules and moves E outward until the bend is under 10 deg (3 % drag).
- REEL / wrapped: the same path. Six turns are laid on the drum between the cup rim and the base flange. The tail runs from the drum to the guide eye (behind the flange), so the turns walk to the base (v7 rule).

### 2.6 Side sketch (right side view, φ 37, launch ready, cocked)

```
                                   . dart nose (u 80)            z
                         spring  .'                              ^
                 anchors (218,256)                               |
                       .'  dart cocked u -87..80                 +--> x
            pusher  .'  springs stretched to 415 mm
   R2 (-191,-37) .'   rack over big gear (64,48)
        \      [rail]=====O====== pivot (0,0) = wiper axis = main shaft
         \           drive gear 32T on sprag #1
          E (-175,55 at y -67)       lug (36,-48)
   reel module           \           |
   capstan (-130,30)      crank (-110,-90) a 28, conrod 132
   reel (-140,-29)   idler (-90,0)
 =================== base plate z -135 ==========================
```

### 2.7 Centre distance check (m(z1+z2)/2 against placed positions, cd.py)

| pair | module | required | placed |
|---|---|---|---|
| DG 32 / BG 48 (rail) | 2 | 80.00 | 80.00 (rail u 80, w 0) |
| BG / rack | 2 | pitch line at 48 | w 48 |
| TG1 60 / TT 60 | 1.5 | 90.00 | 90.00 (0,0) to (-90,0) |
| RG 48 / IG 24 | 2.5 | 90.00 | 90.00 (same axes, coaxial with TG1/TT pair) |
| IG 24 / CG 16 | 2.5 | 50.00 | 50.00 (-90,0) to (-130,30) |
| TT 60 / PG 17 | 1.5 | 57.75 | 57.75 (-90,0) to (-140.0,-28.9) |
| AG 40 / SI 12 | 1.5 | 39.00 | 39.00 (arm length) |
| SI 12 / AI 12 | 1.5 | 18.00 | 18.00 (engaged (6.9,-38.4) to (0,-55)), angle at idler 147 deg |
| bevel 15 / 15 | 1.5 | apex on both axes | (0, 186, -55) |
| worm Ø20 / WW 25 | 2 | 35.00 | 35.00 (z -55 to -90) |
| N20 10 / drum 72 | 1 | 41.00 | 41.00 |
| segment (72 equiv, 48 teeth) / pinion 12 | 1 | 42.00 | 42.00 |
| pinion-shaft 36 / idler 40 / cup gear 24 | 1 | 38.00, 32.00 | chain 112 mm drum to capstan (with the 42) |

Clearance flags for the frame worker:
- Reel flange to capstan cup: 59.7 centre distance against 57 radii (2.7 mm), and they sit at different y.
- IG and sprag #2 must stay at y < 66, because the rail sweeps y 74..170 behind the pivot.
- The WW and worm must stay at y > 172.

---------------------------------------------------------------------------------------------------------------

## 3. Trains

### 3.1 Launch train (LAUNCH, forward)

- Wiper, coupling, main shaft, dog collar (LAUNCH), stub, sprag #1 (CSK20PP, inner keyed to the stub, outer keyed in the DG bore), DG, BG, rack.
- DG: 32T m2, face 16, PPA-CF, Ø64. The 47 bore leaves a 6 mm wall to the root (Ø59).
- BG: 48T m2, face 14, PPA-CF, Ø96. 40 teeth present. The last tooth is cut to the pitch line and the one before it is 30 % taller, so they let go together (v10 rule). 8-tooth gap.
- Rack: m2, face 10, PPA-CF bar 250 long, on the carriage underside.
- Main shaft turns per shot = 48/32 = 1.500. The cock pull is 245 mm = 1.22 turns; release, then 0.28 turns of gap to re-engage tooth 1 at the hard stop.
- Teeth: 80 N on a m2 face 10 PPA-CF tooth gives Lewis 10 MPa (rack Y 0.48), and about 11 MPa on the relieved tip. OK in PPA-CF (fatigue about 25 MPa). Sprag #1 at peak is 3.9 N m against 50 rated. The stub (Ø20 keyed PPA-CF, 12 N m rating) sees 3.9 N m in normal use.
- Rail reaction: during cocking the rail gets T_DG = 2.6 to 3.5 N m nose down plus gravity 1.4 N m. That load goes through the conrod to the worm (self-locked).
- Backward: sprag #1 freewheels for the hub against the DG. The hard stop holds the carriage forward, so the DG cannot follow backward. A cocked spring is never left in place: the firmware never reverses mid-cock (reversing would de-cock and change the aim together).

### 3.2 Aim train (LAUNCH, backward)

- Stub, AG 40T m1.5 (keyed, face 10), swing idler SI 12T on an arm pivoting on the stub (printed flexure drag washer gives the swing), AI 12T m1.5.
- Then a bevel pair 15T:15T m1.5 PPA-CF, the worm (m2, single start, Ø20, lead 6.28, lead angle 5.7 deg) and the worm wheel 25T m2 PPA-CF face 12, to the crank (a 28).
- Ratio main : crank = 25 / (40/12) = 7.5 : 1.
- Self-locking: the worm locks if mu > tan 5.7 deg = 0.10. Printed PPA-CF on PPA-CF greased is 0.15 to 0.25, so it locks. Efficiency driving is about 0.39.
- Pitch -10.3 to +45.4 deg; up and down strokes are 180 crank deg each.
- Full sweep (-10 to 45) = 3.75 main turns. Full crank turn = 7.5 main turns. At about 43 rpm (light load) the full sweep takes 5.2 s.
- Resolution: AS5600 gives 0.025 deg at the main shaft, so pitch resolution is about 0.01 deg. Real error is the swing engage lag (about 15 deg of main shaft, 0.3 to 0.6 deg pitch) plus backlash. The firmware re-zeroes at AIM_HOME each crank turn.
- Loads: worst crank torque 2.8 N m (conrod 100 N, WW tooth 112 N, Lewis 14 MPa). Maximum conrod force 108 N under the 5.6 N m shot kick. Driving torque at the stub is about 0.3 N m.
- Forward: the swing idler swings out to its stop, so the aim is untouched while cocking.

### 3.3 Reel train (REEL, forward only)

- Wiper, main shaft, dog collar (REEL), RG 48T m2.5 (Ø120, face 15, PPA-CF), IG 24T m2.5 (keyed on the Ø20 idler shaft, with sprag #2 as a backstop to the frame), CG 16T m2.5, capstan sleeve, Ø40 capstan.
- Ratio i = 48/16 = 3.0 (the idler does not change the ratio, it only sets the direction). Line per main turn is 386 mm.
- Sprag #2 torque at wiper stall: capstan 4.24 N m, idler 6.4 N m. That is under 12 N m (shaft) and 50 N m (sprag). CG tooth force at 207 N line is 212 N; Lewis 212/(15·2.5·0.29) = 19.5 MPa at stall, 13.5 MPa at 145 N.
- Backward in REEL is never commanded: it stalls against sprag #2, and the current trip catches it.

### 3.4 Take-up train (every mode, forward only; C1)

- TG1 60T m1.5 (main shaft), TT 60T m1.5 (free on the idler axis, 1:1), PG 17T m1.5 (reel axis), printed sawtooth face ratchet (24 teeth, PLA flexure washer preload), sleeve, slip plate, storage reel.
- Ratio plate : main = 3.53.
- Overdrive at the empty core: 1.46x the capstan line speed, 2.81x the cocking rack speed. Slip power about 0.1 W.
- The ratchet locks plate-backward relative to PG. So the payout friction torque goes PG, main shaft, self-locking wiper: positive brake reaction in any mode with the motor stopped or turning forward.
- Backward (aim): the ratchet clicks and the plate stays.

### 3.5 One-way elements and clutches (complete list)

| element | where | job |
|---|---|---|
| sprag #1 CSK20PP | stub / DG | cock forward, freewheel backward |
| sprag #2 CSK20PP | idler shaft / frame | capstan backstop: holds tow load in any drum state |
| swing-arm splitter | AG / SI / AI | aim only on backward turns, pawl-free |
| worm 1-start | aim | self-lock, holds pitch while cocking and through the kick |
| face ratchet (printed) | PG / plate sleeve | take-up one-way plus brake reaction (system plan allows printed ratchets for take-up) |
| dog collar 6 dogs, 3 deg back-taper | main shaft | mode select |
| slip clutch plate | storage reel face | payout brake and take-up limit, preload from drum track 2 |
| N20 worm (bought) | drum | self-locks the drum |

### 3.6 Torque limiting

- Firmware is primary. BTS7960 IS current trip: LAUNCH 8 A (6.0 N m, 1.5x the cock peak); REEL set by tow limit, default 45 N (3.9 A), maximum 145 N (11 A); aim 4 A. AS5600 stall trip: no motion for 150 ms while driven.
- Mechanical:
  - The wiper stall (15 N m) only reaches 207 N at the line in REEL, under the line break (667 N).
  - Every printed shaft sees at most 15 N m only if the firmware fails. At 15 N m a Ø20 keyed PPA-CF shaft is at 10 MPa against the 8 MPa design stress; its strength margin is about 3. Accepted.
  - The slip plate limits the take-up to 1.5 N. The N20 is limited by its own stall, with the DRV8833 current limit at 0.3 A.

### 3.7 Wiper turns per function

| function | main shaft turns | time (ASSUMED wiper) |
|---|---|---|
| full aim sweep -10 to 45 | 3.75 backward | 5.2 s |
| full crank cycle | 7.5 backward | 10.5 s |
| one shot (cock 245 + fire + reset) | 1.50 forward | 1.9 s + 0.4 s = 2.3 s |
| capstan line | 386 mm per turn forward | 0.29 m/s free |

---------------------------------------------------------------------------------------------------------------

## 4. Selector drum

- Barrel Ø56 x 70 PLA, on a Ø8 shaft in 2 x 608.
- Driven by the N20 worm (27 rpm at 12 V, about 22.5 rpm at 10 V from the MP1584) through 10T:72T m1 (7.2:1). That gives about 3.1 rpm (18.7 deg/s) at the drum.
- Single turn. A hard stop between 350 and -5, so WRAPPED can never step to LAUNCH.

### 4.1 Angle table

| drum deg | state | groove 1 (dog fork) | track 2 (brake/take-up at the line, full reel) | segment gear (cup) | code S1 S2 S3 |
|---|---|---|---|---|---|
| -5..10 | LAUNCH | +6 (LAUNCH) | 0.5 N (axial 2.8 N) | locked home | 0 0 1 |
| 10..30 | shift | ramps +6 to 0 | ramps 0.5 to 1.5 | locked | 0 1 1 |
| 30..50 | NEUTRAL (40) | 0 | 1.5 N (axial 8.3 N): overrun stop, slack pull | locked | 0 1 0 |
| 50..70 | shift | ramps 0 to -6 | ramps 1.5 to 0.8 | locked | 1 1 0 |
| 70..95 | REEL (80) | -6 (REEL) | 0.8 N (axial 4.4 N) | locked | 1 0 0 |
| 95..335 | WRAP sector | -6 dwell | 0.4 N (light wrap load), ramps at both ends over 10 deg | 48 teeth: 6 cup turns = 6 wraps (40 deg per wrap) | 1 0 1 |
| 335..350 | WRAPPED (tow) | -6 dwell | 0.8 N: 6 wraps x mu 0.15 = 286x gives 229 N hold, above the 207 N stall | locked at home angle | 1 1 1 |
| 000 | | | | | fault: wire off or contact broken |

- Gray coded: every step changes one bit.
- Groove helix angle 30 deg. The fork follower is a printed pin on the sliding rod. The fork is a PLA flexure preloaded 10 N, so the drum can finish its move while the dogs butt. The collar then snaps in when the wiper jogs.

### 4.2 Wrap drive

- Segment gear: 72T-equivalent m1, teeth only over 240 deg, convex locking arcs elsewhere.
- Pinion 12T m1 with a concave locking shoe (Geneva style). It makes exactly 4 turns, so it re-locks at the same angle at both ends.
- On the pinion shaft: 36T, then a 40T idler, then the cup gear 24T. Cup = 4 x 1.5 = 6 turns.
- The cup shaft (Ø12 PPA-CF) runs inside the capstan sleeve to the cup at the front. The front stays clear, because the dart line sweeps the cup face (v7 rule).

### 4.3 N20 torque check

N20 ASSUMED 0.21 N m stall at 10 V. The usable 40 % (0.085 N m) gives 0.55 N m at the drum (stall 1.36 N m).

| load | drum torque |
|---|---|
| fork 10 N at r 28, helix 30 deg, mu 0.2 | 0.24 N m |
| preload cam 8.3 N, 20 deg ramp | 0.14 N m |
| wrapping, peak | cup torque (0.4 N storage + 0.8 N dart side) x 1.22 groove x 24.5 mm = 0.036 N m, x 9 / 0.75 = 0.43 N m |
| worst case (no overlap: fork and track move only outside the wrap sector) | 0.43 N m |

The worst case is under 0.55 N m.

### 4.4 Times

| move | time |
|---|---|
| LAUNCH to NEUTRAL (40 deg) | 2.1 s |
| NEUTRAL to REEL | 2.1 s |
| REEL to WRAPPED (255 deg) | 13.6 s (2.1 s per wrap) |
| unwrap | the same in reverse |

### 4.5 Contacts, manual lever, mechanical interlocks

- Contacts: printed flexure levers, an M3 screw head touching an M3 nut, ESP32 internal pull-ups, 10 ms debounce. Code bumps sit on a ring at the drum's +y end, 3 tracks 4 mm wide.
- Manual lever: on the drum shaft +y end. First pull the N20 swing-plate latch: the N20 swings 8 mm out of mesh (contact MANUAL opens, firmware locks the wiper out). Then turn the drum by hand. The detents are the code bumps.
- Mechanical interlocks:
  - Wraps exist only with the dog at REEL (groove 1 dwells -6 from 70 to 350).
  - The cup is locked at home whenever the dog can be at LAUNCH.
  - The mode cannot change with wraps on, because the drum must travel back through the whole WRAP sector (unwrap) before groove 1 moves.

---------------------------------------------------------------------------------------------------------------

## 5. Launch physics (Century springs)

### 5.1 Springs

- 2 x 16 OD x 1.4 wire, mean D 14.6, index 10.4, Wahl 1.139. The formula gives 97.9 active coils, 4.57 m of wire and 55.2 g each. k 0.125 N/mm, F0 7 N, both ASSUMED until the hang test.
- Body stress is 15.4 MPa per N:

| cocked extension | force per spring | body stress |
|---|---|---|
| 250 mm | 38.2 N | 590 MPa |
| 275 mm | 41.4 N | 638 MPa |
| 300 mm | 44.5 N | 686 MPa |

- Limits: music wire (Sut about 2100 MPa) allows 945 MPa at 45 %, so 300 mm is fine on paper. Hard-drawn (1670 MPa) allows 750 MPa; the hook torsion limits it near 260 mm. Hence Harry's 250 mm until the hang test.
- Stored energy (two springs):

| hole | preload / cock | energy | cock force |
|---|---|---|---|
| A (default) | 5 / 250 mm | 11.24 J | 76.5 N |
| B | 30 / 275 mm | 12.77 J | 82.8 N |
| C | 55 / 300 mm | 14.30 J | 89.0 N |

  The stroke is 245 mm in every hole; only the anchor moves.

### 5.2 Launch model (shot.py)

- Masses: dart 279, carriage 45 (target, PPA-CF rack 15 + skeletal PLA 30), springs 2·55.2/3 = 36.8, reel m_eff 22, swivel rollers 3. Total 386 g.
- Losses modelled:
  - Runner friction mu 0.25 (PLA V groove), on gravity plus 8 % of spring force (pitching couple, spring line 6 mm below the CG).
  - Gravity along the rail.
  - Brake 0.5 N plus 3 % roller loss.
  - The dart slides off the stopped cradle over half its length.

Results at hole A, 0.5 N brake:

| pitch | v dart | stroke time | peak accel | efficiency |
|---|---|---|---|---|
| 30 | 7.21 m/s | 56 ms | 19.0 g | 0.645 |
| 35 | 7.18 | 56 | 19.0 | 0.640 |
| 40 | 7.16 | 56 | 18.9 | 0.636 |
| 45 | 7.14 | 56 | 18.9 | 0.633 |

Energy budget at 40 deg (11.24 J):

| item | energy | note |
|---|---|---|
| dart | 7.15 J | |
| carriage into the stop | 1.19 J | |
| spring surge | 0.97 J | |
| reel spin-up | 0.66 J | half comes back in flight as lower tension |
| gravity | 0.53 J | |
| runner friction | 0.37 J | |
| brake | 0.13 J | |

Levers (v dart at 40 deg, base 7.16 m/s):

| change | v dart |
|---|---|
| hole B / hole C | 7.68 / 8.17 m/s |
| carriage 35 g / 70 g | 7.26 / 6.91 |
| reel 15 g / 35 g | 7.23 / 7.04 |
| runner mu 0.15 | 7.22 |
| brake 0.3 / 1.0 N at launch | 7.19 / 7.09 |

What matters, in order: spring extension, brake level in flight, carriage mass, reel mass, friction.

### 5.3 Why no pulley

- With a 2:1 pulley (carriage moves twice the spring travel), the reflected spring mass drops from 36.8 to 9.2 g. But the sheave loss (2 x F693ZZ sheaves plus cord, about 5 %) cancels it: efficiency 0.736 against 0.729. The stroke doubles to 490 and the rail to about 750 mm.
- The other way, 1:2 for a short rail, gives 147 g of reflected spring mass and drops the efficiency to 0.53.
- Energy is capped by the extension limit (11.3 J at 250 mm) whatever the pulley. Direct 1:1 it is.

### 5.4 Range (flight sim: point mass, axial Cd 0.9 on 1.9x frontal area, tether = brake + reel inertia + line drag, reel overrun allowed)

Muzzle 0.5 m above ground. Range in m:

| hole / brake | 30 deg | 35 deg | 40 deg | 45 deg | best |
|---|---|---|---|---|---|
| A / 0.3 N | 4.92 | 5.10 | 5.15 | 5.07 | |
| A / 0.5 N | 4.63 | 4.75 | 4.74 | 4.62 | 37 deg 4.76 |
| A / 0.75 N | 4.28 | 4.33 | 4.27 | 4.10 | |
| A / 1.0 N | 3.95 | 3.95 | 3.84 | 3.63 | |
| B / 0.5 N | 5.23 | 5.38 | 5.39 | 5.26 | |
| C / 0.3 N | 6.17 | 6.43 | 6.52 | 6.44 | |
| C / 0.5 N | 5.82 | 6.00 | 6.02 | 5.89 | 38 deg 6.03 |
| C / 1.0 N | 5.00 | 5.03 | 4.91 | 4.67 | |

- No brake at all at 40 deg: 5.62 m (A).
- Time of flight is 0.85 to 1.25 s; landing speed 6.1 to 7.4 m/s.
- At these speeds the tether brake costs 8 % (0.5 N) to 19 % (1.0 N) of range, far more than at the 18 m/s the old sim assumed. So: LAUNCH brake 0.5 N, adjustable down to 0.3 N (M3 stop screw on the track-2 lever), never over 0.75 N.
- Firmware aims at 37 deg by default (38 with hole C).

### 5.5 Cocking at the wiper

Rack force = springs + runner friction + gravity component + 80 N pinch over the first 15 mm + 1 N take-up. Torque = F x 32 mm / 0.87 + 0.15.

| hole | peak torque | current | cock time | reset |
|---|---|---|---|---|
| A | 3.92 N m (26 % of 15 N m stall) | 5.8 A | 1.91 s | 0.39 s |
| C | 4.39 N m (29 %) | 6.3 A | 1.98 s | |

Electrical energy is about 90 J per shot. The battery (5.2 Ah 3S, 58 Wh) gives about 2,300 shots on paper.

### 5.6 Shot kick and stop

- The spring reaction is 76.5 N at w 73. That gives 5.6 N m nose up for 56 ms, through the worm (conrod 108 N).
- Carriage stop: 1.19 J. The fin pinch takes about 0.85 J, the hard stop about 0.35 J. Peak deceleration over 136 g, so the dart separates cleanly.
- Tip-off as the tail leaves the cradle: about 0.4 deg and 1.4 rad/s nose down. The 15 g ballast stabilises it.

---------------------------------------------------------------------------------------------------------------

## 6. Reel physics

### 6.1 Storage reel and payout

- Reel: core Ø50, inner width 22, flanges Ø66, walls 1.2 PLA, 19.3 g.
- 20 m of 1 mm Dyneema (13 g, packing 0.85) fills to r 29.9, leaving a 3.1 mm rim.
- Effective mass at the line, I / r squared:

| line radius | m_eff |
|---|---|
| r 29.9 (full) | 21.3 g |
| r 27.4 | 25.3 g |
| r 25.5 (core) | 29.3 g |

- Spin at 7.2 m/s: 2,300 rpm full, 2,750 rpm at the core (608 fine). Swivel rollers Ø16 run at 8,600 rpm (F693ZZ fine).
- Spin-up: no motor pre-spin. The line is taut at fire (C1), so the reel spins up during the stroke as part of the launched mass (0.66 J, included above). About half of that comes back in flight, because the reel's inertia lowers the line tension while the dart decelerates.
- Taut check in flight: the reel can brake itself at F/m_eff = 0.5/0.022 = 23 m/s squared. The dart decelerates along its path at about 8 m/s squared in the climb. So the line stays taut, and the dart actually feels only 0.32 N in the climb.

### 6.2 Brake (slip clutch)

- Clutch: mu 0.3 (PPA-CF plate on PLA face), friction radius 18 mm.

| line force (full reel) | torque | axial preload |
|---|---|---|
| 0.3 N | 9 N mm | 1.7 N |
| 0.5 N | 14.9 N mm | 2.8 N |
| 0.8 N | 23.9 N mm | 4.4 N |
| 1.5 N | 44.8 N mm | 8.3 N |

- Constant torque means the line force rises as the reel empties: 0.5 N full becomes 0.6 N at the core. The large core keeps it inside the 0.5 to 1 N window.
- Preload comes from a PLA leaf flexure pressed by the track-2 follower lever (lever 1:1). An M3 adjuster sets the LAUNCH level.

### 6.3 Overrun (the bird's nest risk)

At landing the line stops. The reel coasts against the brake: overrun = m_eff v squared / (2F).

| case | overrun | stop time |
|---|---|---|
| 0.5 N from 6.2 m/s | 0.93 m | 0.30 s |
| 0.3 N from 6.7 m/s | 1.82 m | 0.54 s |
| 1.5 N from 6.2 m/s | 0.31 m | 0.10 s |

Three measures, no extra parts:
1. Printed shroud over the reel: 300 deg, 0.5 mm from the flange rims, exit slot at the guide. Loose turns stay on the spool.
2. NEUTRAL brake ramp: the firmware starts LAUNCH to NEUTRAL at predicted time of flight minus 0.6 s, or at the dart IMU landing event if sooner. Track 2 is at 0.9 N at landing and 1.5 N 0.4 s later. Cost about 0.1 m of range; overrun about 0.35 m.
3. Post-landing take-up: NEUTRAL + forward winds the overrun back under 1.5 N (C1).

### 6.4 Wraps (Euler-Eytelwein, T_load / T_hold = e^(mu θ))

Multiplication factor per number of wraps:

| mu | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|
| 0.15 | 17 | 43 | 111 | 286 | 733 |
| 0.20 | 43 | 152 | 535 | 1,881 | 6,611 |
| 0.25 | 111 | 535 | 2,576 | 12,392 | 59,610 |

- Choice: 6 wraps. At the worst mu 0.15 and a 0.8 N hold: 229 N, above the 207 N wiper stall, so the line cannot slip on the capstan in any case. 5 wraps would need a 1.9 N hold (higher wrap load on the N20).
- Cup groove (58 deg) adds about 1.3x on the dart side; ignored here (conservative).
- Six turns of 1 mm on the 40 long drum take 6 to 7 mm.

### 6.5 Tow force and line speed (capstan i = 3, Ø40, line radius 20.5, gear efficiency 0.85)

| line tension | speed | current |
|---|---|---|
| 0 | 0.290 m/s | |
| 10 N | 0.276 m/s | 2.3 A |
| 30 N (dart dragged home) | 0.248 m/s | 3.9 A |
| 60 N | 0.206 m/s | 6.3 A |
| 100 N | 0.150 m/s | 9.5 A |
| 145 N | 70 % stall (13 A, short duty) | |
| 207 N | stall | |

Ratio trade (motor.py), free speed / force at 70 % stall:

| i | free speed | force at 70 % stall |
|---|---|---|
| 2.0 | 0.19 m/s | 217 N |
| 2.5 | 0.24 | 174 |
| 3.0 | 0.29 | 145 |
| 3.5 | 0.34 | 124 |
| 4.0 | 0.39 | 109 |

- Why i = 3: the dart holds 7 N (dry sand), 45 N (loam) and 112 N (firm clay). 145 N covers the best soil with margin, and the dart comes home at 0.25 m/s (7 m in 28 s).
- Higher ratios gain speed only at low load. Above about 60 N the wiper is power-limited (17.7 W maximum mechanical, ASSUMED), so every ratio gives about 0.15 m/s at 100 N.
- Printed limits are not the constraint: the capstan sees 4.2 N m at stall, the idler 6.4 N m, the sprag 50 N m rated.
- Firmware tow limit: default 45 N (loam), selectable 7 / 45 / 110 N from the soil guess.

### 6.6 Take-up tension and slip settings

| state | at the line | job |
|---|---|---|
| LAUNCH | 0.5 N | payout brake, holds the dart on the pusher while cocking |
| NEUTRAL | 1.5 N | overrun stop, slack pull after landing |
| REEL | 0.8 N | take-up behind the capstan, packing |
| WRAP sector | 0.4 N | light wrap load |
| WRAPPED | 0.8 N | tow hold, 229 N |

- Take-up overdrive 1.46x at the empty core, so the reel always keeps up with the capstan.
- Winding is soft (0.8 N): no layer digging, easy next payout.

---------------------------------------------------------------------------------------------------------------

## 7. Mission sequence (firmware state machine)

Inputs:
- AS5600 on PG (main shaft angle x 3.53, multi-turn counted).
- BTS7960 IS current.
- Drum code S1 S2 S3.
- AIM_HOME (crank at min-pitch dead centre).
- CARRIAGE_HOME (carriage on the hard stop).
- DART_SEATED (pusher flexure contact).
- MANUAL (N20 swung out).
- Optional REEL_TICK (bump on the reel flange, once per turn).
- Jetson USB serial (dart IMU events over WiFi, operator commands).

| # | state | action | exit event (sensor) |
|---|---|---|---|
| 0 | BOOT | read code; if 000 or MANUAL: FAULT. If wraps on (101/111): HOLD-TOW, else drum to NEUTRAL (010) | code 010 |
| 1 | READY-LAUNCH | drum to LAUNCH (001). If the dogs butt (N20 current over 0.25 A and code stuck at 011): wiper forward 5 deg at 20 % PWM, retry x3 | code 001 |
| 2 | AIM | wiper backward until pitch = target (37 deg default, from crank angle = count/7.5 + offset, re-zeroed on each AIM_HOME edge) | angle reached; trip at 4 A |
| 3 | COCK-FIRE | wiper forward. Take-up keeps the line taut. Expect DART_SEATED closed throughout and CARRIAGE_HOME opening at about 5 deg | release = current drop over 60 % within 20 ms plus speed jump, at the count for tooth 40 (1.22 turns); trip 8 A |
| 4 | RESET | continue forward to 1.50 turns at 40 % PWM, stop | CARRIAGE_HOME closed + count = 1.50; launch train unloaded |
| 5 | FLIGHT | timer t_pred (0.85 to 1.25 s from pitch and hole). At t_pred - 0.6 s, or the dart IMU impact, drum LAUNCH to NEUTRAL (brake ramp) | code 010 |
| 6 | SLACK-PULL | NEUTRAL + forward 30 % PWM: take-up winds overrun and slack at 1.5 N | REEL_TICK stops for 0.5 s (or Jetson says line taut, or 3 s timeout) |
| 7 | ANCHOR-WAIT | motor off, dart anchors (kick leg, drills) | dart "anchored" over WiFi |
| 8 | REEL | drum to REEL (100); then forward 1 turn: capstan unwrapped, take-up sets 0.8 N dart-side tension | code 100 |
| 9 | WRAP | drum to WRAPPED (111), wiper stopped | code 111 (then 101 to 111 edge) |
| 10 | TOW | forward; current limit = tow limit; stop when the Jetson says arrived, or force over limit for 2 s, or line-out count = 0 | event; on over-limit: stop, HOLD (sprag holds), ask Jetson |
| 11 | RELEASE | motor off (sprag holds), dart retracts | dart "released" |
| 12 | RETRIEVE | forward 0.25 m/s; slow to 20 % PWM for the last 1.0 m (line-out count); at the muzzle the dart climbs the funnel at rail -10 deg (set in AIM before TOW, or at step 8) | DART_SEATED closed + current rise; timeout to FAULT (hand-seat) |
| 13 | UNWRAP | drum WRAPPED to REEL (reverse through 101 to 100); take-up at 0.4 N takes the 6 turns back (0.77 m) | code 100 |
| 14 | to READY | drum to NEUTRAL then LAUNCH; go to 2 | |
| F | FAULT | motor off. If wraps off: drum to NEUTRAL. If wraps on: stay (sprag #2 + self-locking wiper hold the tow), alarm, wait for the operator. Never unwrap under load | operator |

Notes:
- The retrieve pitch (-10) is set with backward turns BEFORE the reel phase (step 8), because aiming needs LAUNCH. The rail stays at -10 through tow and retrieve, held by the worm.
- In step 2 the dart rides from -10 to 37 deg against the pusher. The line grows 12 mm (paid out against the brake). Cocking removes any slack.

---------------------------------------------------------------------------------------------------------------

## 8. Part list

### 8.1 Printed

Material: PPA-CF for loaded gears and keyed shafts, PLA otherwise.

| part | rough size | mat | subsystem |
|---|---|---|---|
| coupling hub (crank-arm clamp, DD socket) | Ø70 x 12 | PPA-CF | frame/launch |
| main shaft Ø20 keyed + Ø25 bearing section, pilot | Ø25 x 80 | PPA-CF | launch/reel |
| dog collar (6+6 dogs, fork groove) | Ø44 x 20 | PPA-CF | drum |
| launch stub Ø20 keyed + dog flange | Ø44 x 87 | PPA-CF | launch |
| drive gear DG 32T m2 (47 bore, keyway) | Ø68 x 16 | PPA-CF | launch |
| big gear BG 48T m2, 40 teeth, relieved last | Ø100 x 14 | PPA-CF | launch |
| big gear pin | Ø12 x 40 | PPA-CF | launch |
| rail front and rail rear (channel, V grooves, floor slot, bosses, R2 mount, lug bracket) | 2 x about 270 x 100 x 90 | PLA | launch |
| nose block (anchor holes B/C, roll funnel, front hard stop, pinch shoes) | 90 x 140 x 60 | PLA + PPA-CF shoes | launch |
| carriage (skeletal cradle, pusher with Ø14 coned hole, DART_SEATED flexure, spring pegs, runners, fin) | 250 x 80 x 45, target 30 g | PLA | launch |
| rack m2, 250 | 250 x 10 x 9 | PPA-CF | launch |
| aim gear AG 40T m1.5 | Ø63 x 10 | PPA-CF | launch |
| swing arm + idler SI 12T + drag washer | 50 x 15 x 12 | PPA-CF / PLA | launch |
| aim input shaft + AI 12T + bevel 15T | Ø25 x 40 | PPA-CF | launch |
| worm shaft (worm m2 Ø20 + bevel 15T) | Ø25 x 150 | PPA-CF | launch |
| worm wheel 25T m2 + crank (a 28) | Ø54 x 30 | PPA-CF | launch |
| conrod 132 | 150 x 18 x 8 | PPA-CF | launch |
| crank pin, lug pin, clips | Ø8 to Ø12 | PPA-CF / PLA | launch |
| REEL gear RG 48T m2.5 (dogs) | Ø125 x 19 | PPA-CF | reel |
| idler shaft Ø20 keyed | Ø20 x 55 | PPA-CF | reel |
| idler IG 24T m2.5 | Ø65 x 15 | PPA-CF | reel |
| capstan gear CG 16T m2.5 + capstan sleeve Ø24 (bore 12.5) | Ø45 x 100 | PPA-CF | reel |
| capstan drum Ø40 x 40 with base flange (v7) | Ø46 x 43 | PLA (PPA-CF for real tows) | reel |
| wrap cup (v7 ring, groove) + cup shaft Ø12 + cup gear 24T m1 | Ø48 x 14; Ø12 x 110 | PLA / PPA-CF | reel |
| TG1 60T m1.5 and TT 60T m1.5 | Ø93 x 10 each | PPA-CF | reel |
| storage reel (Ø50 core, Ø66 flanges, 608 seats) | Ø66 x 25 | PLA | reel |
| slip plate + sleeve + face ratchet + PG 17T m1.5 + magnet pocket | Ø50 x 45 | PPA-CF | reel |
| ratchet flexure washer, brake leaf flexure | Ø30; 60 x 10 x 1.5 | PLA | reel |
| reel shroud (300 deg) | Ø72 x 30 | PLA | reel |
| reel axle Ø8 (if no M8 bolt) | Ø8 x 85 | PPA-CF | reel |
| guide eye post, E swivel block (yoke + Ø16 roller), R2 swivel block | 30 x 30 x 40 | PLA / PPA-CF | reel / launch |
| selector drum (3 tracks, 72T ring gear, segment gear with locking arcs) | Ø76 x 70 | PLA (gears PPA-CF) | drum |
| drum shaft Ø8 + manual lever | Ø8 x 110; 80 lever | PPA-CF / PLA | drum |
| fork rod + fork (flexure) + guides | Ø8 x 100; 60 x 40 | PPA-CF / PLA | drum |
| preload follower lever | 90 x 12 x 6 | PLA | drum |
| wrap pinion 12T with locking shoe + 36T, transfer idler 40T | Ø40 | PPA-CF | drum |
| N20 swing plate + latch + pinion 10T | 50 x 30 x 15 | PLA / PPA-CF | drum |
| contacts (flex levers) x 7 | 25 x 8 x 3 | PLA | drum/launch |
| bulkheads A, B, C + reel module plates | up to 250 x 180 x 10 | PLA (bushing inserts PPA-CF) | frame |
| base plate (2 pieces) | 2 x 250 x 300 x 6 | PLA | frame |
| wiper adapter plate (slotted, parametric holes) | 140 x 120 x 10 | PPA-CF | frame |
| electronics bay (ESP32, BTS7960, DRV8833, MP1584, AS5600 bracket) | 150 x 80 x 40 | PLA | frame |
| bushings Ø20 x 6 (main, stub, idler, capstan), Ø12, Ø8 | | PPA-CF | frame |

### 8.2 Bought (all in hand except the 608s)

| item | qty | use |
|---|---|---|
| 12 V wiper motor, self-locking | 1 | everything |
| CSK20PP sprag | 2 | #1 cock and fire, #2 capstan backstop |
| 608ZZ | 6 of 8 to 10 | reel 2, drum 2, idler shaft end 1, spare 1 (more where printed bushings are marginal) |
| F693ZZ | 2 | R2 and E rollers |
| 693ZZ | 2 | swivel bearings of R2 and E |
| Century 16 x 165.1 x 1.4 extension spring | 2 | launch |
| 1 mm Dyneema 150 lb | 20 m | tether |
| N20 right-angle worm 27 rpm | 1 | drum |
| ESP32-S3 DevKitC-1-N8, BTS7960, AS5600, DRV8833 #3, MP1584 | 1 each | base electronics |
| M3 screws, nuts, inserts M3x4x5, shaft collars | | everywhere; contacts |
| optional 100 kg load cell + HX711 | | in the E post to read line tension (tow limit, landing) |
| M8 bolts if found | 2 to 3 | reel axle, drum shaft (else printed) |

---------------------------------------------------------------------------------------------------------------

## 9. Risks, open questions, measurements

### 9.1 Ranked risks

1. **Range is energy-limited to about 4.8 m (hole A)**, not the 18 m of the old 43 J assumption. The Century springs give 11.2 J at 250 mm. Mitigation: hole C (300 mm) gives 6.0 m if the hang test passes; keep the brake at 0.5 N or less; keep the carriage at 45 g or less. Nothing else moves range by more than 3 %.
2. **Wiper interface and real torque/speed unknown.** Everything is sized at 26 to 29 % of the ASSUMED stall, so a 10 N m wiper still cocks. Reel speeds scale with free rpm.
3. **Overrun at landing (0.9 m at 0.5 N)** gives bird's nest risk. Mitigation: shroud, NEUTRAL brake ramp (0.35 m), post-landing take-up (C1). Test with the dart thrown by hand off the reel.
4. **Retrieve onto the carriage** (roll orientation of a legged dart). Funnel at -10 deg plus DART_SEATED plus a hand-seat fallback. Still the weakest link (same as overnight).
5. **Printed shafts at stall** (15 N m against a 12 N m rating). Firmware trips at 8 A (6 N m) and on AS5600 stall.
6. **Carriage stop 1.2 J and spring surge 1.0 J per shot.** Pinch shoes wear; PLA hard-stop fatigue. Inspect every 50 shots; shoes are replaceable.
7. **Dog butting during a shift.** Flexure fork plus firmware jog. Back-tapered dogs against walkout under load.
8. **Swing arm** needs a steady drag to swing. The drag washer wears; check the engage lag in calibration.
9. **Face ratchet** clicks during aim and wears over thousands of aim turns. Tooth 1.5 mm tall, PPA-CF.
10. **R2-E span changes 32 mm over the aim range.** Slack until the next cock is harmless (C1), but the line must not leave the swivel rollers. Use deep V grooves with keepers.

### 9.2 Open questions (made the call, can be changed)

- Hole A (250 mm) is the default until the hang test.
- 37 deg is the default launch pitch.
- 6 wraps; i = 3.
- -10 deg retrieve pitch.
- LAUNCH brake 0.5 N.
- The fault state with wraps on is HOLD, not NEUTRAL (unwrapping under load would drop the rover).
- E position and the v7 ring geometry are left to the reel worker (bend at the ring under 10 deg in LAUNCH).

### 9.3 What Harry must measure (in this order)

1. **Spring hang test**, each spring:
   - Hang 1, 2, 3, 4 kg and record the extension: gives k and F0.
   - Hold 4.5 kg (300 mm) for 10 min and check the free length after. Permanent set under 1 mm means hole C is allowed.
2. **Wiper motor**:
   - Free rpm and current at 12 V.
   - Stall torque: arm + luggage scale or the load cell, short pulse.
   - Output shaft (diameter, flats, spline, thread length) and crank arm shape.
   - Mount holes and body envelope; which way the motor can points.
3. **Dyneema on printed PLA friction**: hang a weight over a printed Ø40 cylinder, 1 and 2 wraps, and find the pull to lift it. Gives mu (decides 5 or 6 wraps).
4. **N20 worm** stall torque at 10 V (string on a Ø20 pulley + scale).
5. **Dart**: mass with ballast; the bridle shape at the tail cap (does a knot pass a Ø14 hole?).
6. **Rover deck**: size and height (sets the base plate split and the departure height).
7. **Hardware box**: M8 bolts? How many F693ZZ / 693ZZ?
