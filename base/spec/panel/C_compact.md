# GrappStar base station: integrated architecture, lens C (compact and printable)

Planning only. Every number below comes from the scripts in the scratchpad (`calc/c1.py` to `c4.py`, run with the kedge venv) or from the repo files named. Units: mm, deg, N, N m, s. No part bought beyond the list in the brief plus 608s.

---------------------------------------------------------------------------------------------------

## 1. Concept summary (10 lines) and concept changes

1. One wiper motor turns ONE main shaft M (PPA-CF, coaxial with the wiper output, also the rail pivot). A dog collar on M's hex picks the LAUNCH output gear or the REEL output gear; the N20 worm drum moves it.
2. LAUNCH forward: drive gear z24 on M turns a pin gear z36 on the rail; sprag #1 inside the big gear z36 (on the same rail pin) turns it forward only; the big gear (v10 relieved last tooth) pulls the carriage rack 250 mm, releases, and re-engages: 1.5 wiper turns per shot.
3. LAUNCH backward: a swing-arm splitter gear on the same output drives a face gear + single-start worm (one vertical part) into a worm wheel with an eccentric crank (a 19 mm) and a 61 mm conrod to the rail: pitch oscillates 10 to 47 deg, about 9 wiper turns per sweep; the worm self-locks so the aim holds while cocking.
4. The carriage is a 255 mm SLED: pusher at the rear, rack underneath, the dart lies on it. It carries the dart back while cocking and pushes it out on release.
5. Springs: the two Century springs side by side outboard of the rail, 0 to 250 mm extension, 11.3 J, muzzle speed 7.0 m/s, range 3.3 to 4.3 m with 0.5 to 1 N tether drag (4.3 to 5.6 m if the hang test allows 300 mm).
6. REEL: the REEL output gear z48 on M meshes BOTH the capstan gear z20 (x2.4, capstan d40) and the storage-reel clutch gear z12 (x4). Sprag #2 is a capstan backstop to the frame. The reel spins on 2 x 608 and drags against its clutch gear (held still by the backstopped capstan through the shared mesh) during payout, and is overdriven 17 to 92 % through the same clutch during reel-in.
7. Harry's v7 wrap cup over the capstan, turned 4 times by the drum through a z50 segment, a z8/z54 compound idler and a z54 ring on the cup (x6.25), and locked at home by the segment's locking arc.
8. Selector drum (one printed barrel, worm N20 through x4): LAUNCH 0, NEUTRAL 40, REEL 80, wrap sector 100 to 345 (4 wraps), end stops at -8 and 348. Groove 1 moves the fork, track 2 sets the reel clutch (1.5 N preload LAUNCH and NEUTRAL, 15 N REEL), 3 printed contacts read a Gray code plus position ticks.
9. Line path is length-invariant over pitch: dart tail, pusher hole, breech pulley, under-pivot pulley T on the rail, frame pulley E, cup ring, capstan (0 wraps launch, 4 wraps reel), guide eye, reel. Reel-in pulls the dart tail-first into the sled until the SEATED contact closes.
10. Footprint about 500 x 225 mm, height 330 mm stowed with the dart, 4 frame parts, 4 rail parts, about 40 printed parts total.

### CONCEPT CHANGE 1: sprag #1 moves from the drive gear on M to the big gear on the rail pin

- Baseline: launch sleeve free on M carrying sprag #1 (20 bore, 6 mm key). A sleeve of OD 20 needs M at 12 mm or less inside it. A 6 mm keyway in a 20 OD sleeve is about 3.5 deep: the wall left under the key over a 12 mm bore is 10 - 3.5 - 6 = 0.5 mm. It cannot carry the 2.6 N m cocking torque, let alone a 15 N m stall.
- Change: the sprag inner race sits on a solid PPA-CF 20 mm keyed pin in the rail cheeks (the pin gear z36 is keyed to the same pin); the outer race is keyed into the big gear hub (big gear root d 83.75, wall around the 47 race 18 mm). The sprag torque is 3.74 N m (rated 50). M stays a solid 20 to 24 mm shaft its whole length (9.5 MPa at 15 N m stall). Same part count, same function (forward cocks, backward freewheels, the cocked load holds through the wiper worm), and the LAUNCH output becomes one plain PPA-CF gear part free on M.

### CONCEPT CHANGE 2: the carriage is a sled carrying the dart, rack under the dart

- In any layout the carriage carries a 250 mm rack, so it weighs about 75 g. In the v10 arrangement the rack trails behind the pusher and the dart sits in front, so the rail is rack + stroke + dart = 250 + 250 + 167 = 667 mm plus margins. With the rack running forward from the pusher under the dart, the rail is rack + stroke = 510 mm. That is 24 % shorter, needs no extra parts and adds no mass.
- Bonus: the dart rides on the sled while cocking. Without this, the dart must slide back by gravity (cradle friction 0.25 means it needs above about 15 deg and is unreliable), or the carriage takes a run-up and hits the dart.

### Detail improvements (not concept changes)

- Reel take-up taken straight off the REEL output gear, not off the capstan shaft: removes one gear and one shaft step. Payout reaction still goes through the backstopped capstan via the shared mesh.
- Aim right-angle stage: face gear z27 and worm on ONE vertical PPA-CF part, so the splitter output and the worm are the same part.
- Line transfer between frame and rail through three pulleys on 693ZZ (Harry has them). Pulleys replace the large sliding-eye wraps (about 1 % loss each against e^(mu theta) of 1.3 to 1.6 per eye).

---------------------------------------------------------------------------------------------------

## 2. Kinematic layout

### 2.1 World frame and conventions

- Origin: the wiper output shaft axis where it meets the adapter face (the outer face of frame wall F2). +X forward = launch azimuth, +Y left, +Z up. The wiper body is on the -Y side, its output shaft points +Y.
- Rail coordinates (u along the rail forward, w perpendicular up, y unchanged), origin on the pivot axis. Pitch theta is nose-up. World = (u cos theta - w sin theta, y, u sin theta + w cos theta).
- Dart frame (kedge CLAUDE.md section 5: nose tip x_d = 0, tail x_d = 166.7 folded, belly z_d = -21, +y_d left): on the sled at rest u = 241.7 - x_d, w = z_d + 78, y = 76 + y_d. At full cock subtract 250 from u.
- Deck plate (rover mount face) top at z = -130.

### 2.2 Every axis and body

| Item | Position (world unless rail coords given) | Axis | Notes |
|---|---|---|---|
| Wiper output / M | (0, y, 0), y 0 to 112 | +Y | M: hub y 0-14 (D bore on 10 mm flats, M8 nut pocket), round 24 journal y 16-28, hex 24 AF y 28-46, round 20 journal y 46-104, 8 mm stub y 104-112 in a 608, magnet pocket in the end face |
| Wiper body (assumed, parametric) | box x -40..150, y -85..-5, z -110..40 | | Bolted to F2 through the wiper's own holes (WIPER.mount_holes) |
| REEL output gear z48 m2.5 b12 | on M, y 16-28 | +Y | Free on the 24 journal |
| Dog collar | on M hex, y 30-44 (travel +-4) | +Y | LAUNCH = +4 (toward y 48), REEL = -4 |
| LAUNCH output part | on M 20 journal, y 46-104 | +Y | Dog face y 46-49, hub through right cheek boss y 50-54, drive gear z24 m2.5 y 56-68, hub through left cheek boss y 84-92, splitter input z36 m1.5 y 94-102 |
| Rail pivot | M axis | +Y | Cheek bosses ride on the LAUNCH output hub (right) and on it again (left), plain PLA on PPA-CF |
| Rail | u -180 to +330, centre plane y 76 | | Cheeks y 46-54 and 84-92; springs outboard y 36 and 116 |
| Rail pin P1 (pin gear + big gear) | rail (75, 0), y 56-82 | +Y | Pin gear z36 y 56-68 meshes drive gear; big gear z36 y 70-82 meshes rack |
| Rack pitch line / sled | w 45; sled body w 45-57; cradle (dart belly) w 57; dart axis w 78 | | Sled pusher at u 75 at rest, u -175 at full cock |
| Springs | y 36 and y 116, w 40; posts at u 240; pegs on pusher outriggers | along u | Free length 165.1 at rest (x0 = 0), 415 at full cock |
| Lug (aim) | rail (-12.5, -58.7), y 129 | +Y | b = 60 from the pivot, on an outrigger of the left cheek |
| Swing arm + idler z18 m1.5 | arm on M (y 94-102), idler at (40.5, y 94-102, 0) engaged | +Y | Arm radius 40.5 = 1.5 x (36 + 18) / 2 |
| Face gear z27 + worm m2 (one part) | vertical axis at (40.5, 118.25, z), face pitch plane z -13.5, worm z -25 to -50 | +Z | Face rim meets the idler at y 98 = 118.25 - 20.25 |
| Worm wheel z24 m2 = crank | O2 = (74.5, y 112-124, -37.5) | +Y | Worm to wheel centre 34 = (20 + 48) / 2; crank pin at a = 19.0 |
| Conrod | crank pin to lug, y 126-132 | | c = 60.8 |
| Capstan C | (42.5, y 8-100, -73.6) | +Y | 85 from M at -60 deg; gear z20 y 16-28, sprag #2 y 30-44 in bulkhead F3, drum d40 x 40 y 46-86, cup y 72-90 with ring z54 m1 y 86-90 |
| Storage reel S | (-31.7, y 8-66, -68.0) | +Y | 75 from M at -115 deg; clutch gear z12 y 16-28, friction face y 28-30, spool y 32-58 (d52 flanges, d28 barrel, 24 wide, 2 x 608 inside), pressure plate + wave spring y 58-64 |
| Drum D | (8, y 8-110, -98) | +Y | OD 36, necked to 24 at y 70-90 (passes the cup); drum gear z48 m1 y 8-12; code bumps y 14-24; groove 1 y 30-44; track 2 y 60-66; segment z50 m1 y 92-96 |
| Wrap idler I (z8 + z54 m1) | (-11.4, y 86-96, -76.4) | +Y | D-I 29 = (50 + 8) / 2; I-C 54 = (54 + 54) / 2 |
| Drum N20 (right-angle worm) | output axis at (8 - 30 cos 30, y 8-12, -98 - 30 sin 30) = (-18.0, y, -113.0) | +Y | Pinion z12 m1, centre 30 = (48 + 12) / 2; on a swing bracket with a snap pin (manual declutch) |
| Fork | rod (8 mm) at (-25, y 20-60, -45), fork tip in the dog collar groove | +Y | Follower pin in groove 1 at the drum top |
| Manual lever | drum left end (hex), outside F4 at y 112-120 | | |
| AS5600 | facing M's end magnet, y 114.5, on F4 | | |
| Breech pulley B (693ZZ) | rail (-182, 52), y 76 | +Y | Line turns 180 deg here |
| Transfer pulley T (693ZZ) | rail (-20, -38), y 76 | +Y | Radius 43 from the pivot, arc 26 mm over 10-47 deg |
| Frame pulley E (F693ZZ) | (-20, 76, -112) | +Y | Line length T to E: 71.4 / 72.1 / 79.3 at 10 / 28.5 / 47 deg (8 mm change) |
| Guide eye G1 | (5, 50, -45) | | Between capstan base flange and reel, v7 style 21 mm offset |
| Dart (at rest) | tail u 75, nose u 241.7, belly w 57, centre y 76 | along u | Envelope y 35.7 to 116.3, w 57 to 153.8 |

### 2.3 Gear centre distances (m (z1 + z2) / 2)

| Pair | m | z1 + z2 | Required | Layout | OK |
|---|---|---|---|---|---|
| Drive z24 (M) / pin gear z36 (rail P1) | 2.5 | 60 | 75.0 | P1 at rail (75, 0): 75.0 | yes |
| Big gear z36 / rack | 2.5 | rack | pitch r 45 | rack line w 45 = P1 w 0 + 45 | yes |
| REEL z48 / capstan z20 | 2.5 | 68 | 85.0 | sqrt(42.5^2 + 73.6^2) = 85.0 | yes |
| REEL z48 / reel clutch z12 (profile shift +0.3 on z12) | 2.5 | 60 | 75.0 | sqrt(31.7^2 + 68.0^2) = 75.0 | yes |
| Splitter input z36 / idler z18 | 1.5 | 54 | 40.5 | arm radius 40.5 | yes |
| Idler z18 / face gear z27 | 1.5 | face | face rim r 20.25 | y offset 118.25 - 98 = 20.25, axes intersect at z 0 | yes |
| Worm m2 d20 (1 start) / wheel z24 | 2 | | 34.0 | 74.5 - 40.5 = 34.0 | yes |
| Drum gear z48 / N20 pinion z12 | 1 | 60 | 30.0 | 30.0 | yes |
| Segment z50 / idler pinion z8 (shift +0.4) | 1 | 58 | 29.0 | sqrt(19.4^2 + 21.6^2) = 29.0 | yes |
| Idler z54 / cup ring z54 | 1 | 108 | 54.0 | sqrt(53.9^2 + 2.8^2) = 54.0 | yes |

### 2.4 Rail at min and max pitch (key points, world x, z)

| Point (rail u, w) | theta 10 | theta 47 |
|---|---|---|
| Rear bottom (-180, 20) | (-180.7, -11.6) | (-137.4, -118.0) |
| Pivot (0, 0) | (0, 0) | (0, 0) |
| Big gear centre P1 (75, 0), tip r 47.5 | (73.9, 13.0), lowest z -34.5 | (51.2, 54.9) |
| Lug (-12.5, -58.7) | (-2.1, -60.0) | (34.4, -49.1) |
| Muzzle top of cradle (330, 57) | (315.1, 113.4) | (183.3, 280.2) |
| Dart top at rest (241.7, 153.8) | (211.3, 193.4) | (52.3, 281.6) |
| Dart nose at rest (241.7, 78) | (224.5, 118.8) | (107.8, 230.0) |

Rail release point: the muzzle is 0.40 m above the ground with the rover deck at about 0.25 m (assumed; Harry to confirm the rover deck height).

### 2.5 Sweep checks done by hand (CAD audit must confirm)

- Rail rear bottom at 47 deg: z -118 vs deck -130: 12 mm.
- Cheeks behind the pivot must stay above w 20 for u < -20 (shallow tail), otherwise they hit S at 47 deg. With that: rail underside to the reel S flange (r 26) 17 mm at 47 deg.
- Big gear tip at 10 deg (z -34.5) to the capstan cup top (z -49.6): 15 mm.
- Capstan cup (r 24) to drum: 6.3 mm with the drum necked to r 12 at y 70-90 (UNNECKED it is 0.3 mm: keep the neck).
- Reel flange to drum: 5.8 mm. Idler I gear to cheek boss: 14.8 mm. Line T-E to drum barrel: 28 mm.
- Left spring line (w 40) to face gear: 27.8 mm; to worm wheel: more than 50 mm.
- TIGHT SPOT for CAD: the line from E (-20, -112) up to the cup ring passes within 1 mm of the drum barrel if the ring sits at the cup's lower rear. Put the ring at the cup's upper rear (the 135 deg position) or move E to x -28. Each of these is under 10 mm of change.

### 2.6 Line path

```
LAUNCH payout (dart flies along +u):
dart tail cap -> pusher hole d14 (straight) -> back along the cradle centre groove -> B breech pulley (180 deg)
  -> forward under the sled, between the cheeks, w 15 -> T pulley under the pivot (82 to 119 deg)
  -> E frame pulley (about 90 deg) -> cup ring (cup locked at home) -> over the capstan, 0 wraps
  -> G1 guide eye -> storage reel S (spins on 608s, clutch drags)
REEL-IN: same path, 4 wraps on the capstan laid by the cup; the dart is drawn tail-first into the muzzle funnel,
  along the cradle, until its tail cap closes the SEATED contact on the pusher.
```

- Sliding-eye wrap at payout: cup groove 58 + ring and E-exit about 40 + G1 about 40 = about 140 deg. Factor e^(mu theta) = 1.44 (mu 0.15) to 1.63 (mu 0.2). Pulleys: about 1.03 for all three.
- Length change over 10 to 47 deg: 8 mm (T on a 43 mm radius, E 72 mm away on the arc bisector). In LAUNCH the reel pays out freely when the line needs length. Lowering the rail leaves 8 mm of slack, which the reel-in take-up removes.
- During the shot the line slides through the in-line pusher hole and over B. The sled stops at the front, the dart separates and the line runs straight from the cradle groove.

ASCII side view (theta 10, y not shown):

```
              dart (u 75..242, w 57..154)
            ____________________
   pusher  |____________________|  springs (outboard) to posts u 240 ->   muzzle u 330
 B o====== sled + rack (w 45..57) ======================================|> funnel
   |        cheeks .  P1 big gear (75,0)                                 |
   +--line--> T o  M (pivot, 0,0)  o drive z24
                  \      lug o-----conrod-----o crank (worm wheel O2 74.5,-37.5)
            S reel o  D drum o      C capstan o (cup + ring)
   ------------------------------------------------------------ deck z -130
```

---------------------------------------------------------------------------------------------------

## 3. Every train

### 3.1 Wiper and torque limits

- Wiper (assumed): 45 rpm free, 15 N m stall, 18 A stall, 1.5 A free. I = 1.5 + 16.5 T / 15. rpm = 45 (1 - T / 15).
- Torque limit on every train = BTS7960 IS current trip per mode (sampled at 1 kHz, trip after 20 ms), plus the self-locking wiper worm. Mechanical last line: M, dog, REEL gear and LAUNCH output in PPA-CF sized for the full 15 N m stall (M 9.5 MPa; dog: 3 jaws, 3 mm engaged x 6 mm radial at r 16, 312 N per jaw at stall = 17 MPa bearing, PPA-CF fine).

| Mode / function | Normal peak at M | Trip | Trip current |
|---|---|---|---|
| LAUNCH cock (forward) | 2.62 N m (x0 0), 3.1 (x0 50) | 4.5 N m | 6.5 A |
| LAUNCH aim (backward) | 0.3 N m | 1.5 N m | 3.0 A |
| REEL wind-in | 0.6 (no load) to 5.6 N m (100 N tow) | user tow limit, default 100 N = 5.6 N m | 7.6 A |
| Any mode, wrong direction (REEL backward hits sprag #2) | stall | 1.5 N m in the first 0.3 s | 3.0 A |

### 3.2 LAUNCH train (forward = cock, fire, reset)

| Element | Teeth / size | Module, face | Material | Notes |
|---|---|---|---|---|
| Dog jaws on LAUNCH output | 3 jaws, 3 deep, 3 deg back-cut | | PPA-CF | Back-cut holds engagement under load |
| Drive gear (on M) | z24 | m2.5, b12 | PPA-CF (part of LAUNCH output) | 2.62 N m |
| Pin gear (rail P1) | z36 | m2.5, b12 | PLA | keyed to the pin, 87 N tooth force, 7 MPa |
| Rail pin P1 | d20 keyed 6, 48 long, head + clip groove | | PPA-CF | 3.74 N m, 2.4 MPa |
| Sprag #1 CSK20PP | 20 x 47 x 14 | | bought | inner on pin, outer in big gear |
| Big gear | z36, teeth 1 to 32 present, 33-36 removed (40 deg gap), tooth 32 cut to the pitch line, tooth 31 30 % taller (v10 rule) | m2.5, b12 | PLA (100 % infill) | 83 N rack force, 6.8 MPa |
| Rack on sled | 32 teeth, 251 mm | m2.5, b12 | PLA | |

- Ratio: big gear turns per M turn = 24 / 36. One shot = one big gear turn = 1.5 wiper turns: 32 teeth x 7.85 mm = 251 mm stroke, then the 40 deg toothless gap lets the sled fly and resets.
- Release: tooth 32 lets go at big gear angle 32 x 10 = 320 deg; the sled flies 250 mm in about 70 ms. The gear meanwhile turns about 6 deg (at 36 rpm M, 24 rpm big gear), well inside the 40 deg gap. Tooth 1 re-meshes the rack with the sled at its front stop.
- Backward: sprag #1 freewheels; the big gear stays (sled against its front stop).
- Hold when cocked: big gear -> sprag (locks backward) -> pin -> pin gear -> drive gear -> dog -> M -> wiper worm.

### 3.3 AIM train (backward only)

| Element | Teeth / size | Material | Notes |
|---|---|---|---|
| Splitter input (on LAUNCH output) | z36 m1.5 b8 | PPA-CF | |
| Swing arm | 40.5 radius, PLA, flexure drag finger on the input hub (0.02 N m) | PLA | stops: engaged stop at 0 deg, retract stop at -12 deg (idler moves 8 mm clear) |
| Idler | z18 m1.5 b8 on a printed 6 mm pin | PLA | |
| Face gear + worm (one part) | face z27 m1.5 (pitch r 20.25); worm m2, 1 start, d20, lead angle 5.71 deg | PPA-CF | 2 x 693ZZ (or printed bushings), thrust shoulder on the frame (107 N max) |
| Worm wheel + eccentric crank | z24 m2 b12, crank pin d8 at a = 19.0 | PPA-CF | carries the AIM_ZERO bump |
| Conrod | 60.8 c-c, 16 x 8 section | PLA | 60 N max, 0.5 MPa |
| Lug | rail (-12.5, -58.7), b = 60 | PLA | |

- Wiper turns per crank turn = 24 x 27 / 36 = 18. Pitch 10.0 to 46.9 deg per crank turn, so one full up sweep or down sweep takes about 9 wiper turns, 12 s at 45 rpm. Average 4.1 deg of pitch per wiper turn, about 7 deg at the steepest. With the AS5600 (0.09 deg) the pitch resolution is far below the gear backlash (about 0.5 deg at the rail).
- Transmission angle 90 +- 19 deg over the whole cycle (synthesis, `c2.py`).
- Self-locking: lead 5.71 deg against friction angles of 6.8 to 16.7 deg (mu 0.12 to 0.30), so it locks at any realistic mu. Forward efficiency 0.24 to 0.45.
- Holding during cocking: rail moment = drive reaction 2.5 + gravity 0.9 = 3.4 N m. Rod 60 N, crank 1.14 N m, wheel tooth 48 N (m2 b12 PPA-CF: 9 MPa).

### 3.4 REEL train

| Element | Teeth / size | Material | Notes |
|---|---|---|---|
| REEL output gear (free on M) | z48 m2.5 b12, dog jaws on its face | PPA-CF | up to 15 N m |
| Capstan gear | z20 m2.5 b12, keyed | PPA-CF | step-up 2.4 |
| Capstan shaft | d20 keyed 6, 100 long | PPA-CF | |
| Sprag #2 CSK20PP | inner keyed to shaft, outer keyed into bulkhead F3 | bought | backstop: wind-in free, wind-out locked |
| Capstan drum | d40 x 40, base flange d50 (v7) | PLA (PPA-CF if towing above 150 N) | |
| Wrap cup (v7 geometry) | over the drum, ring gear z54 m1 on the rim | PLA | locked at home by the drum's locking arc |
| Reel clutch gear | z12 m2.5 b12 (+0.3 shift), friction face r 8 to 16 | PPA-CF | step-up 4 |
| Storage reel | d52 flanges, d28 barrel, 24 wide, 2 x 608 inside | PLA | holds 20 m to d43.5 |
| Wave spring + pressure plate | printed PPA-CF wave washer, k 3.5 N/mm | PPA-CF | 1.5 N (LAUNCH) to 15 N (REEL) |

- Capstan line speed per wiper rpm: 2.4 x 0.040 x pi / 60 = 5.0 mm/s. See section 6 for the table.
- Take-up overdrive: reel surface 4 x r_reel against capstan surface 2.4 x 20 = 48 (mm per rad of M), so 56 (empty core r 14) to 92 (full r 23): 17 to 92 % faster. The clutch always slips during reel-in and the take-up tension = clutch torque / r_reel.
- Payout: the reel spins in the payout direction. The clutch drag turns its gear in the capstan's wind-out sense through the REEL output gear (free on M in LAUNCH). Sprag #2 forbids that, so the clutch slips and acts as the brake.

### 3.5 DRUM train

- Worm N20 (27 rpm at 12 V, about 20 rpm at 9 V from the MP1584) -> pinion z12 m1 -> drum gear z48 m1: x4, drum 5 rpm, 345 deg in 11.5 s.
- Wrap: segment z50 m1 (teeth over 100 to 345 deg) -> pinion z8 -> compound z54 -> cup ring z54: x6.25. 245 deg of drum = 4.25 cup turns. Wrap k is complete at 100 + 57.6 k deg (157.6, 215.2, 272.8, 330.4).

---------------------------------------------------------------------------------------------------

## 4. Selector drum

### 4.1 Angle table

| Drum deg | Name | Groove 1 (fork / dog) | Track 2 (reel clutch) | Segment (wrap cup) | Code A B C |
|---|---|---|---|---|---|
| -8 | end stop | LAUNCH | low | locked | 0 0 0 |
| 0 (+-6 dwell) | LAUNCH | +4 (LAUNCH output engaged) | low, 1.5 N preload (0.3 N at the line) | locked home | 0 0 1 |
| 6 to 34 | transit | ramp +4 -> 0 | low | locked | 0 0 0 then 0 1 0 at 20 |
| 40 (+-6) | NEUTRAL | 0 (both free) | low | locked | 0 1 1 |
| 46 to 74 | transit | ramp 0 -> -4 | ramp low -> high | locked | 0 1 0 then 1 1 0 at 60 |
| 80 (+-6) | REEL | -4 (REEL output engaged) | high, 15 N preload (3 N take-up) | locked home | 1 1 1 |
| 86 to 95 | transit | REEL | high | locking arc ends at 95 | 1 1 0 then 1 0 0 at 95 |
| 100 | WRAP home | REEL | high | teeth start | 1 0 1 |
| 157.6 / 215.2 / 272.8 / 330.4 | wrap 1 / 2 / 3 / 4 | REEL | high | cup turns 1 / 2 / 3 / 4 | 1 0 1 tick each |
| 348 | end stop | REEL | high | past wrap 4 | 1 0 0 |

- Gray zones: A B = 00 LAUNCH zone (-10 to 20), 01 NEUTRAL zone (20 to 60), 11 REEL zone (60 to 95), 10 WRAP zone (95 to 350). Each zone boundary flips one bit, so there are no glitch codes. C = "at a nominal stop" (+-3 deg). The firmware counts C ticks in the WRAP zone, so wraps = ticks.
- The order is physical: wraps exist only in the WRAP zone, where the fork dwells at REEL, and leaving the WRAP zone toward NEUTRAL turns the cup back to home first. So there are no wraps in LAUNCH, and no swap happens with wraps on.
- Groove 1: 4 mm of lift over 28 deg at r 18 (arc 8.8 mm), helix 24 deg. Groove 3.2 wide for a 3 mm follower pin.
- Track 2: radial cam 4.2 mm of lift; a bell-crank (pivot axis X, 1:1) pushes the reel pressure plate along -Y, compressing the PPA-CF wave spring 0.4 -> 4.4 mm.
- Locking arc: the segment's plain arc (r 25.5) runs in the concave flank cut into the z8 pinion hub, so the cup cannot turn outside the wrap sector. The cup's home (the v7 lock pin position) is held through the compound idler.

### 4.2 Torque check (drum N20)

| Load at the drum | Value |
|---|---|
| Fork: 15 N shift (jaw chamfers + friction), groove 24 deg + friction 12 deg, r 18 | 0.20 N m |
| Track 2: 15 N spring at 25 deg cam + friction, r 18 | 0.17 N m |
| Wrap: cup line pull 3 N at r 27, x6.25, eff 0.85 | 0.10 N m (if the line is slack: 0.03) |
| Worst overlap (46 to 74 deg, fork + track 2) | 0.37 N m |
| Available: N20 worm 0.2 N m (assumed, kedge rating 1 to 4 kg cm) x 4 x 0.85 | 0.68 N m, margin 1.8 |

The N20 worm self-locks, so the drum holds any position unpowered. N20 stall at the end stops: 0.18 A at 8 V, harmless (DRV8833 limit about 1.2 A).

### 4.3 Manual lever

The N20 sits on a swing bracket held by one snap pin. Pull the pin and the pinion swings out of mesh, so the lever on the drum's left end turns the drum by hand. The detents and the code still work, so the firmware sees the new position.

---------------------------------------------------------------------------------------------------

## 5. Launch physics (Century springs)

- Spring (by formula): wire 1.4, mean D 14.6, 98 active coils, wire 4.58 m, 55.4 g each, k 0.125 N/mm, F0 7 N. Body shear stress at 250 mm extension: 590 MPa (Wahl 1.14). Music wire allows about 800 MPa, so the hang test decides how far beyond 250 is safe (275 mm: 638 MPa; 300 mm: 686 MPa).
- Pulley ratio: none (1:1). Energy is set only by extension. A 2:1 lengthens the stroke at half force, and a 1:2 makes the spring's own moving mass count 4 times (costs 23 % of dart energy). Both are worse.
- Stroke 250 mm, x0 = 0 (springs just taut at rest). Posts have two extra holes at +25 and +50 for pre-extension after the hang test.

| Item | x0 = 0 (max ext 250) | x0 = 50 (max ext 300, after hang test) |
|---|---|---|
| Stored energy (2 springs) | 11.31 J | 14.44 J |
| Peak rack force | 76.5 N | 89.0 N |
| Losses: sled friction 0.15, gravity over 250 at 35 deg, line spin-up (reel eff. mass 18.5 g x 110 m/s^2 = 2.0 N at the reel, about 3.9 N with path and brake) | 1.61 J | 1.61 J |
| Moving mass: dart 279 + sled 75 + 1/3 of springs 37 | 0.391 kg | 0.391 kg |
| Muzzle speed | 7.05 m/s | 8.10 m/s |
| KE left in sled + springs, taken by the front stop | 2.8 J (about 230 N average over a 12 mm flexure stop) | 3.7 J (310 N) |

Range (point mass, Cd 0.9 on the d42 frontal area, tether drag along the line back to the muzzle, release 0.40 m above ground):

| Pitch | 0.5 N | 0.75 N | 1.0 N | (x0 50) 0.5 / 0.75 / 1.0 N |
|---|---|---|---|---|
| 30 | 4.23 m | 3.89 | 3.59 | 5.42 / 5.01 / 4.62 |
| 35 | 4.32 | 3.93 | 3.58 | 5.57 / 5.08 / 4.63 |
| 40 | 4.29 | 3.86 | 3.47 | 5.57 / 5.01 / 4.52 |
| 45 | 4.16 | 3.69 | 3.28 | 5.42 / 4.81 / 4.28 |

- Best pitch with drag: 33 to 37 deg. Flight time 0.8 to 1.0 s.
- THE RANGE LIMIT IS THE SPRINGS, not the motor. The flight sim's 18 m needs about 43 J, about 4 times what these two springs store. Cocking uses 19 % of wiper stall.
- Cocking torque: big gear 3.74 N m peak (rack 76.5 N x 1.08 + friction) x 24/36 / 0.95 = 2.62 N m at the wiper (17 % of stall), 36.5 rpm, 4.6 A. With x0 50: 3.1 N m, 5.1 A. One shot cycle about 2.5 s.
- Dry fire: 11.3 J into the stop (940 N over 12 mm) would break the sled nose. Firmware blocks the last 10 % of the stroke unless SEATED is closed. The dart rides on the sled, so a dart seated at the start stays seated.

---------------------------------------------------------------------------------------------------

## 6. Reel physics

### 6.1 Payout

- LAUNCH clutch setting: 0.30 N at the reel line radius, which is 5.5 N mm. The axial preload is 1.5 N at mu 0.3 and mean radius 12 mm.
- Drag the dart sees: 0.30 x (1.44 to 1.63 eyes) x 1.03 (pulleys) = 0.45 to 0.50 N, plus the line pulled through the air, plus about 0.1 N of reel bearing drag. That gives 0.55 to 0.75 N, inside the 0.5 to 1 N window. Adjust with the wave spring shim after a pull test with a luggage scale.
- Spin-up: reel I = 9.8e-6 kg m^2 (flanges 8.4 g, barrel 7 g, line 13 g, 608 races). The effective mass at the line is 18.5 g on a full reel (r 23) and 50 g near empty (r 14). During the 70 ms stroke that is 2.0 N full and 5.5 N near empty, included in section 5. Speed 3,070 rpm full and 5,050 rpm near empty: fine for 608s.
- Overrun at landing: the reel coasts at about 4 m/s and decelerates at 16 m/s^2 on 0.3 N, so it throws about 0.5 m of slack. That is acceptable: the flanges hold it, and the reel-in take-up removes it before the wraps. Optional printed centrifugal shoes (2 swing pads inside the flange) if bird-nesting shows up.
- No motor pre-spin.

### 6.2 Wraps (Euler-Eytelwein, T_load / T_tail = e^(2 pi mu N))

| mu | N 3 | N 4 | N 5 |
|---|---|---|---|
| 0.15 | 17 (tail 8.9 N for 150 N) | 43 (3.5 N) | 111 (1.4 N) |
| 0.20 | 43 (3.5 N) | 152 (1.0 N) | 535 |
| 0.25 | 111 (1.4 N) | 535 | 2,576 |

- Choice: 4 wraps, 3 N take-up. That holds 130 N at mu 0.15 and over 450 N at mu 0.2. The cup groove (58 deg) adds another 1.2 to 1.3 times.
- Each wrap uses 126 mm of line, so 4 wraps use 0.5 m of slack, which comes from the dart side (the line is slack after landing). The wrap pull limit is 3 N at the cup, so wrapping a taut line stalls the drum safely.

### 6.3 Tow force and line speed (capstan d40, x2.4, take-up drag included)

| Line tension | Wiper torque | Wiper rpm | Current | Line speed | 20 m in |
|---|---|---|---|---|---|
| 5 N (dragging the dart home) | 0.63 N m | 43.1 | 2.2 A | 0.22 m/s | 92 s |
| 50 N | 2.97 N m | 36.1 | 4.8 A | 0.18 m/s | 110 s |
| 100 N (default limit) | 5.58 N m | 28.2 | 7.6 A | 0.14 m/s | 141 s |
| 150 N (max setting) | 8.19 N m | 20.4 | 10.5 A | 0.10 m/s | 195 s |

- Tow limit: firmware current limit, default 100 N; set lower for weak soil (dart holding: dry sand 7 N, loam 45 N, clay 112 N per kedge CLAUDE.md section 5). The optional load cell + HX711 replaces the current estimate if fitted (under the E pulley bracket).
- Holding: sprag #2 holds any line load in any drum state up to the line break (667 N = 13.3 N m on the capstan shaft; PPA-CF d20 keyed shaft: about 10 MPa there).
- Take-up tension: clutch 55.5 N mm / r_reel = 2.4 N (full) to 4.0 N (empty). Preload 15 N.

---------------------------------------------------------------------------------------------------

## 7. Mission sequence (firmware) with swap triggers

Sensors: AS5600 on M (multi-turn angle, rpm), BTS7960 IS current, drum code A B C, AIM_ZERO (bump on the worm wheel at pitch 10), LAUNCH_HOME (contact closed by the big gear at tooth 1 engaged, sled at front), SEATED (flexure contact on the pusher face, closed by the dart's tail cap), battery voltage. Optional tension (HX711).

| Step | Drum | Wiper | Exit trigger (event) | Fault handling |
|---|---|---|---|---|
| 0 Boot | read code; if BETWEEN, drive to the nearest nominal stop toward NEUTRAL (never past WRAP home toward LAUNCH with C ticks unaccounted) | off | code = 011 (NEUTRAL) | DRUM_SENSOR if no valid code in 15 s |
| 1 Load check | NEUTRAL | off | SEATED closed and LAUNCH_HOME closed | if not seated: step 9 (reel in) or ask the operator |
| 2 To LAUNCH | 40 -> 0 | off | code 001 | DRUM_TIMEOUT 8 s -> back to NEUTRAL |
| 3 Aim | LAUNCH | backward, slowing below 3 deg from target | pitch (from AS5600 count since AIM_ZERO, through the linkage table) within 0.5 deg; AIM_ZERO re-zeroes every pass | aim current above 3 A -> stop, FAULT |
| 4 Cock | LAUNCH | forward to 90 % of stroke (1.35 turns from LAUNCH_HOME) | angle reached, then check SEATED | SEATED open: hold (do not fire), report; uncock by slow backward (aim moves too, re-aim after) |
| 5 Arm + fire | LAUNCH | forward the last 0.15 turn after a valid `fire` token | current drop at release (spring load vanishes) + angle at tooth 32 = FIRED; continue to LAUNCH_HOME closed (reset) | no current drop by 1.6 turns -> STALL fault, hold |
| 6 Flight | LAUNCH (reel brake 0.3 N) | off | Jetson LANDED (dart IMU) or 5 s timeout | |
| 7 To REEL | 0 -> 80 | off (must be: never swap while cocked; the cock+fire cycle is complete at LAUNCH_HOME) | code 111 | refuse if LAUNCH_HOME open |
| 8 Take up slack | REEL | wind-in, duty 40 % | current rises (line taut, about 1.5 A over free) | |
| 9 Wrap | 80 -> 330.4 (4 ticks) | off | 4 C ticks counted, code 101 at 330.4 | drum stall in the wrap zone = line too taut: unwind 1 tick, pay 0.3 m (wiper off, reel brake), retry once, else FAULT |
| 10 Anchor | REEL+wraps | off | Jetson: dart ANCHORED | |
| 11 Tow | REEL+wraps | wind-in, current limit = tow limit | rover reached target (Jetson) or limit held 3 s (STALL) | over-limit: wiper off; sprag #2 holds the line; report. NO unwrap under load |
| 12 Release dart | REEL+wraps | off, then slow wind-in | dart RELEASED (Jetson) | |
| 13 Reel home | REEL+wraps | wind-in 0.2 m/s, slow to 0.05 m/s for the last 1 m (line in from AS5600: 2.4 capstan turns per M turn = 0.84 mm of line per M degree) | SEATED closes (dart tail on the pusher) | current spike without SEATED = dart jammed at the muzzle: stop, operator |
| 14 Unwrap | 330.4 -> 100 | off; reel takes up through the clutch | code 101 at 100 (WRAP home), then 1 1 1 at 80 | |
| 15 To NEUTRAL / LAUNCH | 80 -> 40 (-> 0 for the next shot) | off | code 011 / 001 | |

- Fault -> NEUTRAL rule: on any fault the wiper is cut first. The drum then goes to NEUTRAL only when no wraps are on and the launcher is not part-cocked (LAUNCH_HOME closed). With wraps on, the drum holds and sprag #2 holds the line. Part-cocked, the drum holds and the wiper worm holds the springs. In both cases the firmware reports and waits. Both exceptions exist because a blind move to NEUTRAL would drop a tow load or fire the shot.
- Line out estimate: during payout the reel speed is not sensed. Line out = Jetson range estimate. During reel-in, line in = capstan turns x 125.7 mm (2.4 capstan turns per M turn).

---------------------------------------------------------------------------------------------------

## 8. Part list

All printed parts print flat or standing without supports on the 256 x 256 x 260 bed. Gears print flat on a face; shafts print lying on a 1 mm flat so torsion runs along the layers; parts longer than 250 mm are split with a keyed lap joint and 2 x M3. Snaps: pins with a head and a 6.5 x 3 groove and a printed C clip (v10 style), plus M3 heat-set inserts where something must come apart. Material PLA unless marked PPA-CF.

### 8.1 Printed

| # | Part | Size (mm) | Material | Subsystem | Print |
|---|---|---|---|---|---|
| 1 | Deck plate | 180 x 150 x 4 + ribs 12 | PLA | frame | flat |
| 2 | Right wall / wiper adapter F2 | 200 x 150 x 8 | PLA | frame | flat; wiper holes parametric |
| 3 | Bulkhead F3 (sprag #2 keyed seat, fork rod end) | 120 x 110 x 14 | PPA-CF | frame | flat |
| 4 | Left wall F4 (M 608, C and D left ends, aim gearbox seats, AS5600, lever window) | 200 x 150 x 10 | PLA | frame | flat |
| 5 | Main shaft M (hub, hex, journals, stub, magnet pocket) | d32 hub x 112 | PPA-CF | launch/reel | lying on a flat |
| 6 | Dog collar (hex bore, 3 jaws each side, fork groove) | d40 x 14 | PPA-CF | drum | flat |
| 7 | REEL output gear z48 m2.5 with jaws | d125 x 12 | PPA-CF | reel | flat |
| 8 | LAUNCH output (jaws, drive z24, splitter z36, hubs) | d65 x 58 | PPA-CF | launch | standing |
| 9 | Rail left cheek, rear half (ties standing, lug outrigger, T bracket, B pulley boss) | 250 x 95 x 60 | PLA | launch | flat on outer face |
| 10 | Rail left cheek, front half (spring post outrigger, muzzle funnel half) | 255 x 95 x 60 | PLA | launch | flat |
| 11 | Rail right cheek, rear half | 250 x 80 x 8 + outrigger | PLA | launch | flat |
| 12 | Rail right cheek, front half (post outrigger) | 255 x 80 x 8 | PLA | launch | flat |
| 13 | Sled (pusher, rack z32 m2.5, cradle rails, peg outriggers, line groove, SEATED contact pocket) | 255 x 50 x 30 (diagonal on the bed) | PLA, 100 % at rack and nose | launch | on its side |
| 14 | Big gear z36 (relieved tooth, 47 keyed pocket, LAUNCH_HOME cam) | d95 x 12 | PLA 100 % | launch | flat |
| 15 | Pin gear z36 keyed | d95 x 12 | PLA 100 % | launch | flat |
| 16 | Rail pin P1 d20 keyed | d26 head x 48 | PPA-CF | launch | lying on flat |
| 17 | Front stop flexure pads x2 | 30 x 20 x 14 | PPA-CF | launch | flat |
| 18 | Swing arm (with idler pin, drag finger) | 60 x 20 x 8 | PLA | launch (aim) | flat |
| 19 | Idler z18 m1.5 | d30 x 8 | PLA | aim | flat |
| 20 | Face gear z27 + worm m2 | d45 x 60 | PPA-CF | aim | standing, face gear down |
| 21 | Worm wheel z24 m2 + crank pin + AIM_ZERO bump | d52 x 12 | PPA-CF | aim | flat |
| 22 | Conrod | 77 x 16 x 8 | PLA | aim | flat |
| 23 | Capstan shaft d20 keyed | 100 | PPA-CF | reel | lying |
| 24 | Capstan gear z20 m2.5 | d55 x 12 | PPA-CF | reel | flat |
| 25 | Capstan drum d40 x 40 + flange (v7) | d50 x 42 | PLA | reel | flange down |
| 26 | Wrap cup (v7) + ring z54 m1 | d56 x 18 | PLA | reel | open end down |
| 27 | Storage reel (608 pockets) | d52 x 28 | PLA | reel | front flange down, 56 deg rear cone (v7) |
| 28 | Reel clutch gear z12 m2.5 + friction face | d40 x 16 | PPA-CF | reel | flat |
| 29 | Pressure plate + wave spring (2 parts) | d36 x 6 | PPA-CF | reel | flat |
| 30 | Reel axle d8 (or M8 bolt) | 70 | PPA-CF | reel | lying |
| 31 | Drum (tracks, groove, code bumps, drum gear z48, segment z50, lever hex) | d50 x 110 | PLA (PPA-CF if teeth wear) | drum | standing |
| 32 | Fork + fork rod | 80 x 50 x 8, rod d8 x 60 | PLA / PPA-CF | drum | flat / lying |
| 33 | Wrap compound idler z8 + z54 | d56 x 10 | PPA-CF | drum | flat |
| 34 | Track 2 bell-crank | 50 x 30 x 8 | PLA | drum | flat |
| 35 | N20 swing bracket + snap pin | 40 x 30 x 20 | PLA | drum | flat |
| 36 | Manual lever | 80 x 20 x 8 | PLA | drum | flat |
| 37 | Pulleys B, T, E (693ZZ / F693ZZ hubs, U groove 1.2) x3 | d16 x 6 | PLA | reel/launch | flat |
| 38 | Guide eye G1 + muzzle funnel insert | small | PPA-CF (line wear) | reel | flat |
| 39 | Printed contacts (flexure lever, M3 screw head on M3 nut) x6 | 25 x 8 x 6 | PLA | frame | flat |
| 40 | Pins and C clips (v10 style): lug pin, crank pin, idler pin, conrod pins, 12 clips | small | PPA-CF pins, PLA clips | all | standing / flat |

About 40 part types and 60 pieces. The four frame parts bolt together with M3 into heat-set inserts and locate on printed dowels.

### 8.2 Bought (all in hand except the 608s)

| Part | Qty | Use |
|---|---|---|
| 12 V wiper motor | 1 | M |
| CSK20PP sprag | 2 | #1 big gear on rail pin, #2 capstan backstop |
| 608ZZ 8x22x7 | 7 of 8 to 10 | M left stub 1, reel 2, capstan left end 1, drum ends 2, spare 1 |
| 693ZZ / F693ZZ 3x8x4 | 5 | pulleys B, T, E; face gear shaft 2 |
| Century springs 16 x 165.1 x 1.4 | 2 | launch |
| Dyneema 1 mm, 20 m | 1 | tether |
| ESP32-S3 DevKitC-1-N8, BTS7960, AS5600 (+ diametric magnet, check it is in the bag), DRV8833 #3, MP1584 | 1 each | electronics |
| Optional 100 kg load cell + HX711 | 1 | tension |
| M3 screws, nuts, M3x4x5 inserts, shaft collars | about 40 screws, 20 inserts | frame, contacts |
| M8 nut (comes with the wiper) | 1 | hub clamp |

---------------------------------------------------------------------------------------------------

## 9. Ranked risks, open questions, what Harry must measure

### 9.1 Risks (highest first)

1. Range is 3.3 to 4.3 m, not 18 m. The springs store 11.3 J against the 43 J the flight sim assumed. Mitigation: pre-extension holes for 300 mm (5 m) after the hang test. Beyond that only stronger springs help, which would be a purchase.
2. Retrieval onto the sled: the dart must climb the muzzle funnel tail-first from the ground at pitch 10 (muzzle about 0.4 m up). The kedge/gs overnight design flagged the same thing as its weakest link. Mitigation: the funnel lip roller (693ZZ, already counted as pulley 37 if moved), slow last metre, SEATED contact and stall detect. Test early with the desk rail.
3. Swing-arm splitter reliability (engages on backward, releases on forward, holds mesh depth under 60 N rod loads). Mitigation: positive mesh stop, self-engaging geometry (check the sign in CAD), 0.02 N m drag finger.
4. Printed wave spring creep sets the payout brake (1.5 N) and take-up (15 N). Mitigation: PPA-CF, unloaded except in REEL, adjustable shim; pull test after every 10 shots at first.
5. Line wear and heat at 7 m/s through the cup groove, ring and G1 (sliding). Mitigation: PPA-CF inserts, smooth radii of 2 mm or more, replaceable G1.
6. Sled front stop takes 2.8 J every shot, and 11.3 J on a dry fire. Mitigation: flexure pads, SEATED interlock before the last 10 % of the stroke.
7. Wiper shaft interface unknown (flats, taper, serration). Mitigation: parametric D-bore hub on M, M8 nut clamp.
8. N20 worm torque is assumed (0.2 N m). If it is weaker, the drum stalls on the fork and track 2 overlap. Mitigation: split the ramps so they do not overlap (move track 2 ramp to 74 to 90), x4 ratio already.
9. Tight clearances listed in 2.5 (drum neck, line E to ring). They only need CAD moves under 10 mm.

### 9.2 Open questions (made the call, flagged)

- Pitch range 10 to 47: chosen for the 12 mm rail-tail clearance at 47 with the deck at z -130. Best pitch with tether drag is 33 to 37, well inside.
- Capstan d40 x 40 (Harry's v7) kept. A d50 capstan would give +25 % line speed and needs a 2 mm larger cup, if wanted.
- 4 wraps (holds 130 N at mu 0.15) instead of 5, so the wrap train fits a single drum turn.
- Rover deck height assumed 0.25 m (release height 0.40 m). Range changes about 0.1 m per 0.1 m of height.

### 9.3 Harry must measure

| What | How | Feeds |
|---|---|---|
| Spring rate, F0, safe max extension | hang test: 1, 2, 3, 4 kg, record length; stop at 300 mm | x0 hole, range table |
| Wiper free rpm, stall torque, stall current, shaft (diameter, flats, taper, thread length), mount holes, body box | caliper; a lever + luggage scale at 100 mm for stall; bench supply amps | M hub, F2 holes, current trips |
| Drum N20 stall torque at 9 V | lever + scale | drum margin |
| Dyneema on PLA mu | 1, 2, 3 wraps on a printed d40 drum, hanging weight, pull scale | wrap count |
| Payout drag with the real reel | pull the line through the whole path with a scale at walking speed | wave spring shim (target 0.3 N at the reel, 0.5 to 0.75 N at the dart) |
| Rover deck height and space for a 500 x 225 footprint | tape measure | layout, range |
| Whether the AS5600 bag has its magnet; how many 693ZZ / F693ZZ remain after the dart | look | pulleys, angle sensing |
