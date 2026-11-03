# GrappStar base station: integrated architecture, lens B (reliability first)

Planning document, 3 Nov 2026. Nothing in the repo was changed. Every number below comes from the calculation script
`scratchpad/calc/b_calc.py` (spring, launch, flight, cocking, linkage, capstan, reel, drum, centre distances) or is
marked ASSUMED. Wiper data is the agreed assumption (45 rpm free, 15 N m stall, 18 A stall, 1.5 A free), not measured.

Numbers at a glance

| Item | Value |
|---|---|
| Launch energy (2 Century springs, 226 mm stroke, 0 / 10 / 20 mm preload hole) | 9.6 / 10.1 / 10.7 J |
| Muzzle speed (20 mm hole, 40 deg) | 7.1 m/s, 54 ms stroke, 66 % of spring energy reaches the dart |
| Range at 30 to 45 deg, tether drag 0.5 / 0.75 / 1.0 N | 4.6 / 4.2 / 3.9 m best (35 deg); 3.5 m worst (45 deg, 1 N) |
| Peak cocking torque at the wiper | 3.23 N m = 22 % of stall, 35 rpm, 5.1 A |
| Wiper turns: one shot / aim 5 to 50 deg / full aim cycle | 1.333 / 6 / 12 turns |
| Capstan line speed | 0.19 m/s light, 0.156 m/s at the 60 N tow limit (4.7 A) |
| Wraps | 4 (holds 43 x tail tension at mu 0.15: 105 N with a 2.4 N tail) |
| Payout brake at the dart | 0.6 to 0.8 N (clutch 0.012 N m, eye losses x 1.39) |
| Torque limiters | TL0 7 N m at the wiper (all trains), TL-A 0.6 N m (aim), slip clutch (storage), N20 stall (drum, wrap) |

---------------------------------------------------------------------------------------------------

## 1. Concept summary

1. One wiper on one 20 mm PPA-CF main shaft, coaxial with the wiper output and the rail pivot, through a 7 N m input torque limiter (TL0).
2. A 3-dog sliding collar on the main shaft couples either the LAUNCH sleeve or the REEL gear; NEUTRAL couples neither. Dogs have a 60 deg free window, so every swap starts with a 30 deg "centering jog" that leaves the collar touching nothing.
3. LAUNCH sleeve, forward (wiper CW seen from behind the motor): sprag #1 drives the 24T drive gear, which turns the 32T big gear on the rail; 25 teeth pull the sled rack 226 mm, the relieved last tooth fires, the 7-tooth gap resets. 1.333 wiper turns per shot.
4. LAUNCH sleeve, backward: a pawl-free swing-arm one-way (splitter A) turns a self-locking printed worm (1-start, lead 7.1 deg), the worm wheel turns the crank of a 4-bar that rocks the rail 5 to 50 deg. Forward turns leave the aim locked in the worm.
5. REEL gear (40T) steps the capstan up 2:1. Sprag #2 is the capstan backstop to the frame: the capstan can only wind in, so tow load is held in every drum state.
6. Storage reel spins free on 2 x 608 for launch against a printed face slip clutch whose other side is a take-up "bus" gear, geared 2.5:1 from the main shaft through a second pawl-free swing one-way (splitter B). Forward wiper turns in ANY mode take up line; backward turns leave the bus parked on a 0.04 N m drag brake.
7. Selector drum (single turn, d 60) on the worm N20 through 5:1: LAUNCH 0, NEUTRAL 40, WRAP sector 62 to 302 (4 cup turns through a mutilated 6:1 gear with locking arcs), REEL 340 to 355. Groove 1 moves the fork through a flexure (tip-to-tip safe), track 2 sets the clutch preload, 3 Gray-coded contacts read the zone.
8. Harry's v7 wrap cup and ring, driven through an 8 mm shaft inside the hollow capstan tube, so no gear ever sits in the dart line's sweep.
9. Line path is fixed to the rail from a fairlead 45 mm from the pivot to a breech sheave (693ZZ), so pitch changes move the line by at most 35 mm, and the dart pulls line off the reel from the first millimetre of the stroke (no snatch).
10. Sensors: AS5600 on the main shaft, 10 printed M3 contacts, BTS7960 current, optional load cell. Firmware moves the drum only when the collar is unloaded, the springs are relaxed and the cup is home.

### Concept changes (each quantified)

**CONCEPT CHANGE 1: storage take-up is driven from the main shaft (upstream of the dog), not from the capstan.**
Cocking pulls the sled and the seated dart back 226 mm. The dart's tether runs back out of the breech, so that line must
be wound in during cocking or it becomes 226 mm of slack. With slack, the reel (15 g effective mass at full radius) is
snatched from 0 to 7.1 m/s at the end of the stroke: an inelastic hit that keeps only 87 % of the dart's energy (about
15 % range) and gives a 100 to 200 N line spike on the knot, tail cap and reel. Spinning the reel up during the stroke
instead costs 1.8 N against a 75 N spring force. In LAUNCH the capstan never turns, so a capstan-fed take-up cannot
wind in during cocking. A second reason: unwrapping needs active take-up while the capstan stands still (change 2).
Cost: one printed swing one-way, one 0.04 N m drag brake, 4 small gears.

**CONCEPT CHANGE 2: the collar stays in NEUTRAL across the wrap sector; REEL engages only when fully wrapped.**
Order on the drum stays a fixed line, but it reads LAUNCH / NEUTRAL / WRAP / REEL instead of REEL then wraps. Reason:
unwrapping must retract freed line into the storage reel, which needs forward wiper turns. If the dog were in REEL the
capstan would turn too; with the dart seated (or snagged) and turns still on, the capstan pulls it with up to
T_tail x e^(mu theta) = 2.4 N x 535 (4 wraps, mu 0.25) = 1,280 N theoretical, 157 N at TL0. A 10 to 20 N threshold is
only 0.3 to 0.8 A over a 1.9 A running current (table 6.4): the BTS7960 sense cannot see it reliably. With NEUTRAL, the capstan cannot
move during wrap or unwrap at all. Harry's rules still hold physically: no wraps in LAUNCH (the wrap sector lies beyond
NEUTRAL), no LAUNCH/REEL swap with wraps on (leaving REEL toward LAUNCH drives the cup backward through the whole sector).

**CONCEPT CHANGE 3: the cup is driven through the capstan, not round it.** The capstan is a hollow 20 mm keyed tube
(it is also sprag #2's inner race); an 8 mm cup shaft runs inside it to the cup gear at the back. Harry's v7 rule: the
dart line sweeps across the front face of the cup and passes about 14 mm in front of the axis when the ring is at the
bottom; any gear or hub on the front, or a ring gear on the skirt where the line rides in the groove, would catch it.

Not a concept change but the headline number: with the two Century springs the base throws the 279 g dart **3.5 to
4.6 m**, not the 18 m of the dart flight sim (that needed about 43 J; the springs store 10.7 J at the 246 mm limit).

---------------------------------------------------------------------------------------------------

## 2. Kinematic layout

### 2.1 World frame and conventions

- Origin: wiper output shaft axis on the adapter face. +X forward (launch azimuth), +Y left, +Z up.
- The wiper body is on the -Y side of the adapter face (Y < 0). Every shaft below is parallel to Y unless stated.
- FORWARD wiper direction = right-hand rotation about +Y = clockwise seen from behind the motor (from -Y). Forward = cock,
  fire, reset (LAUNCH) / wind in (REEL) / take up (any mode). BACKWARD = aim (LAUNCH) only.
- Pitch theta: rail angle above +X, muzzle up positive. Muzzle up = counter-clockwise seen from -Y.
- Rail frame (u, w): u along the rail toward the muzzle, w square to it, origin on the pivot. World:
  X = u cos(theta) - w sin(theta), Z = u sin(theta) + w cos(theta), Y = 147 (rail and dart plane).
- ASSUMED until measured: rover deck at Z = -120 (pivot 120 above the deck), deck 150 above ground; wiper gearbox
  envelope 110 (X) x 90 (Z) x 55 (Y) round the output at Y -5 to -60, motor can d 65 x 120 along -X. All wiper
  interfaces are parameters (shaft d, flat, thread, hole pattern) in params.py.

### 2.2 Main shaft stack (axis X 0, Z 0), Y bands

```
 Y:  0    8 10   22 24 30      54   68 |  collar zone  | 98 104 114 116 128 138 140  154 156 166 178 186 188
     |platR| TL0  | B1 |       | G5 REEL |<-- -9 . +9 -->| LS | A1 | bulkC |chkR | G1+sprag1 |chkL |platL|mag|AS5600
                                  free   dog collar 3x30deg  dog  aim  (boss OD50 |  drive gear  | (boss)|608 |
                                         travel +-9 mm      flange sun  128-138)  |  Y 147 = rail plane
```

| Y (mm) | Part | Notes |
|---|---|---|
| 0 to 8 | Plate R (motor adapter) | wiper bolts to its -Y face, parametric holes; boss for the drum shaft and manual lever |
| 10 to 22 | TL0 torque limiter, OD 70 | outer ring keyed to the wiper coupler (double-D socket + M8 nut), inner flexure hub keyed to the main shaft |
| 24 to 30 | B1 bus sun z40 m1.5, keyed | swing arm B pivots on its hub, idler B2 z16 |
| 54 to 68 | G5 REEL gear z40 m2.5 b14, free | PPA-CF bushing; 3 dog slots in its +Y face |
| 68 to 98 | Dog collar, OD 56, 14 body + 6 mm dogs each face | NEUTRAL centre Y 83, travel +-9 mm (6 engage + 3 clear) |
| 98 to 104 | LAUNCH sleeve dog flange | sleeve: PPA-CF tube ID 20.5, OD 32, Y 98 to 154 |
| 104 to 114 | A1 aim sun z30 m1.5 on the sleeve | swing arm A, idler A2 z18 |
| 116 to 128 | Bulkhead C | carries the sleeve bushing; boss OD 50 at Y 128 to 138 |
| 128 to 138 | Rail cheek R on the bulkhead boss | aim lever: lug pin at R 75 reaching Y 146 |
| 140 to 154 | G1 drive gear z24 m3 b14 on sprag #1 (CSK20PP) | sprag inner keyed to the sleeve, outer keyed in G1 |
| 156 to 166 | Rail cheek L on the plate L boss | line fairlead hole at R 45 |
| 166 to 178 | Plate L | 608 on the shaft's 8 mm end journal |
| 180 to 188 | d 6 diametric magnet, AS5600 board at Y 188 | magnet: check the bag (open question) |

Main shaft: PPA-CF, 20 mm with a 6 mm key slot from Y 10 to 154, 8 mm journal Y 166 to 178, about 180 long.
Torsion at TL0 slip: tau = 16 T / (pi d^3) = 4.5 MPa (allowable about 8 MPa). At a 15 N m stall without TL0 it would be
9.5 MPa: this is why TL0 exists.

### 2.3 Every axis

| Axis / body | Position (X, Z), Y band | Axis | Notes |
|---|---|---|---|
| Main shaft = rail pivot | (0, 0), Y 0 to 186 | Y | wiper output coaxial |
| Wiper body (ASSUMED) | gearbox round (0, 0), Y -5 to -60; can along -X to X -175 | | parametric |
| B2 bus swing idler, engaged | (41.8, -3.9), Y 24 to 30 | Y | disengaged stop at +8 deg (about (41.6, 5.8)) |
| B3/B3' compound bus idler | free on the capstan tube (60, -45), B3 z44 Y 24 to 30, B3' z60 Y 31 to 37 | Y | |
| Storage reel + B4 bus gear | (121.5, -45), reel Y 39 to 59, B4 Y 31 to 37 | Y | fixed 8 mm axle, reel on 2 x 608 |
| Guide eye | (95, -19), Y 50 | | 21 mm behind the capstan base flange (v7 rule) |
| Capstan tube (+ sprag #2, G6, drum, cup) | (60, -45), Y 8 to 119 | Y | 75.0 from the main axis |
| Cup ring | r 24.5 on the cup face, Y 119.5 | | v7 geometry |
| Exit eye (PPA-CF, carries tow load) | (60, -21.5), Y 147.5 | | 28 mm axially past the ring (v7 rule) |
| A3 aim output + bevel shaft | (40.5, -29.3), Y 104 to 133 | Y | 50.0 from the main axis |
| A2 aim swing idler, engaged | (15.9, -32.3), Y 104 to 114 | Y | disengaged stop at -76 deg |
| Worm shaft (worm m2.5, pd 20) | Z -29.3, Y 133, X 40.5 to 125 | X | bevel at X 40.5, worm X 95 to 125 |
| Crank shaft + worm wheel z24 | (110, -69.3), wheel Y 127 to 139, crank arm Y 140 to 146 | Y | 40.0 below the worm axis |
| Crank pin circle | r 28.7 round (110, -69.3), Y 143 | | |
| Conrod | L 110, plane Y 143 | | |
| Aim lever lug on cheek R | R 75 at rail angle -117.5 deg: (-28.7, -69.3) at 5 deg, (28.7, -69.3) at 50 deg | | vertical at 27.5 deg |
| Selector drum | (0, -85), Y 36 to 108 | Y | d 60; manual lever outside plate R at Y -12 |
| Drum N20 (right-angle worm) | output at (-36, -85), Y 94 to 106, motor body along -X | Y | on a swing plate with a thumb latch |
| Wrap intermediate shaft W2/W3 | (60.0, -82.5), W2 Y 40 to 48, W3 Y 8 to 16 | Y | locking disc on W2 |
| Fork rod | (0, -52), Y 60 to 106 | Y | fork shoes at X +-32, Z 0 |
| Big gear G2 on the rail | rail (78.9, 28.7); world (76.1, 35.5) at 5 deg, (28.7, 78.9) at 50 deg | Y | 8 mm pin on 2 x 608 |
| Rack pitch line on the sled | w = 76.7 | | |
| Dart axis on the sled | w = 116, Y 147 | | belly at w 95, legs up to w 192, flanks Y 106 to 188 |
| Springs | Y 119 and 175 (Y 147 +- 28), w 95 | | below the drill pods (pods start at w 107) |

### 2.4 Rail, dart and springs at the pitch limits (world X, Z; Y 147)

| Point (rail u, w) | Pitch 5 deg | Pitch 50 deg |
|---|---|---|
| Muzzle, dart axis (330, 116) | (318.6, 144.3) | (123.3, 327.4) |
| Breech bottom (-172, 40) | (-174.8, 24.9) | (-141.2, -106.0): 14 mm above the deck |
| Breech sheave (-172, 106) | (-180.6, 90.6) | (-191.8, -63.6) |
| Dart tail / nose at rest (74 / 226, 116) | (63.6, 122.0) / (215.0, 135.3) | (-41.3, 131.3) / (56.4, 247.7) |
| Dart tail / nose cocked (-152 / 0, 116) | (-161.5, 102.3) / (-10.1, 115.6) | (-186.6, -41.9) / (-88.9, 74.6) |
| Spring post (239, 101) | (229.3, 121.4) | (76.3, 248.0) |
| Spring peg rest / cocked (74 / -152, 95) | (65, 107) / (-160, 87) | (-30, 122) / (-175, -52) |
| Line fairlead on cheek L (-7.8, -44.3) | (-3.9, -44.8) | (28.9, -34.5) |

Rail beam: u -172 to +330 (502 long), w 40 to 80, open-top trough above w 80 (dart and folded legs ride free above
it), printed in two halves joined at u = 80 with a dovetail, 4 x M3 into heat-set inserts. Sled: 240 long, rack along
its full underside, pusher at its rear end (u 74 at rest). Behind the pivot nothing but the rail moves (the crank-rocker
sits in front, so the breech end can swing down to Z -106 at 50 deg without meeting anything).

### 2.5 Side view (seen from -Y, X right, Z up, pitch 5 deg), not to scale

```
                         dart (tail on pusher) ======>                 muzzle
     breech sheave o=====[sled + rack]=====================================|
                  |  rail beam (w 40 to 80)  big gear G2 (76,36)          |
   line runs back |____________________(O)___________________________________
   under the rail      fairlead (-4,-45) ^  main shaft (0,0) = pivot, G1 here
                                         |
                  drum (0,-85)   A3/bevel (40,-29) ---- worm shaft (Z -29) ---- worm ... wheel (110,-69) + crank
                                        capstan (60,-45) [cup ring top Z -21.5]      conrod --> lug (-29..29, -69)
                                        guide eye (95,-19)   storage reel (121.5,-45)
  ------------------------------------------- deck Z -120 -------------------------------------------------
```

### 2.6 Line path

Storage reel (121.5, -45, Y 49) -> guide eye (95, -19, Y 50) -> over the capstan top (clears the drum by 3 mm at zero
wraps: v7 bypass) -> cup groove -> cup ring (Y 119.5) -> exit eye (60, -21.5, Y 147.5) -> rail fairlead on cheek L
(R 45 from the pivot, Y 161) -> under the rail to the breech sheave (printed d 20 on a 693ZZ) -> 180 deg up and
forward along the rail centre channel at w 116 -> through the sled and the pusher hole -> dart tail cap (two d 4.2
holes). Everything after the fairlead rides on the rail, so the path length changes only with the fairlead's swing:
45 mm x 0.785 rad = 35 mm over the full pitch range. Eye wrap budget: at most 2.0 rad of plain eye wrap in total
(guide 0.4, ring 0.6, exit eye 0.6, fairlead 0.4) plus the sheave.

### 2.7 Centre distance check, a = m (z1 + z2) / 2

| Mesh | m | z1 / z2 | a required | a in layout |
|---|---|---|---|---|
| G1 drive / G2 big gear | 3 | 24 / 32 | 84.0 | (78.9, 28.7) -> 84.0 |
| G5 REEL / G6 capstan | 2.5 | 40 / 20 | 75.0 | (60, -45) -> 75.0 |
| A1 / A2 (engaged) | 1.5 | 30 / 18 | 36.0 | (15.9, -32.3) -> 36.0 |
| A2 / A3 | 1.5 | 18 / 15 | 24.75 | (15.9, -32.3) to (40.5, -29.3) -> 24.8 |
| B1 / B2 (engaged) | 1.5 | 40 / 16 | 42.0 | (41.8, -3.9) -> 42.0 |
| B2 / B3 | 1.5 | 16 / 44 | 45.0 | to (60, -45) -> 45.0 |
| B3' / B4 | 1.5 | 60 / 22 | 61.5 | (60, -45) to (121.5, -45) -> 61.5 |
| Bevel pair A3 shaft / worm shaft | 1.5 | 16 / 16 | intersecting axes at (40.5, Y 133, -29.3) | ok |
| Worm (pd 20) / wheel z24 | 2.5 | 1-start / 24 | (20 + 60) / 2 = 40.0 | Z -29.3 to -69.3 -> 40.0 |
| D1 N20 pinion / D2 drum ring | 1.0 | 12 / 60 | 36.0 | (-36, -85) to (0, -85) -> 36.0 |
| W1 drum segment / W2 | 1.25 | 72 / 24 | 60.0 | (0, -85) to (60.0, -82.5) -> 60.0 |
| W3 / W4 cup gear | 1.25 | 40 / 20 | 37.5 | (60.0, -82.5) to (60, -45) -> 37.5 |
| Rack / G2 | 3 | rack / 32 | rack pitch line = G2 centre w + 48 | 28.7 + 48 = 76.7 |

Radial clearance checks done by Y band (the frame worker must still run the exact-mesh sweep): drum (r 30) to G5 tip
(r 51.25) 3.75 mm; B3' tip (r 46.5) shares no Y band with sprag #2's housing (Y 38 to 52); bevel (r 12) to the collar
fork (r 34) 4 mm; drive gear tip (r 39) to the rail beam bottom (w 40): cut a 6 mm relief in the beam over the pivot.

---------------------------------------------------------------------------------------------------

## 3. Trains

### 3.1 Gear and element list

| Id | Part | z / m / face | Material | On |
|---|---|---|---|---|
| G1 | drive gear, keyed 47 bore for sprag #1 outer | 24 / 3 / 14 | PPA-CF | LAUNCH sleeve via sprag #1 |
| G2 | big gear: 25 teeth present, last tooth cut to the pitch line, the one before 30 % taller (v10), 7 missing | 32 / 3 / 14 | PPA-CF | rail pin, 2 x 608 |
| R1 | rack on the sled, 25 teeth | - / 3 / 14 | PPA-CF | sled |
| A1 | aim sun | 30 / 1.5 / 10 | PPA-CF | LAUNCH sleeve |
| A2 | aim swing idler | 18 / 1.5 / 10 | PPA-CF | swing arm A |
| A3 + bevel | aim output, bevel 16T at the shaft end | 15 / 1.5 / 10; bevel 16 / 1.5 / 8 | PPA-CF | A3 shaft |
| - | bevel on the worm shaft with TL-A friction cone | 16 / 1.5 / 8 | PPA-CF | worm shaft |
| W | worm, 1-start, pd 20, lead 7.85 mm, lead angle 7.1 deg | m 2.5 | PPA-CF | worm shaft |
| WW | worm wheel | 24 / 2.5 / 12 | PPA-CF | crank shaft |
| G5 | REEL gear with dog slots | 40 / 2.5 / 14 | PPA-CF | main shaft, free |
| G6 | capstan gear (part of the capstan tube) | 20 / 2.5 / 14 | PPA-CF | capstan tube |
| B1 | bus sun | 40 / 1.5 / 6 | PPA-CF | main shaft, keyed |
| B2 | bus swing idler | 16 / 1.5 / 6 | PPA-CF | swing arm B |
| B3/B3' | compound bus idler | 44 + 60 / 1.5 / 6 | PPA-CF | free on the capstan tube |
| B4 | bus gear + clutch ring | 22 / 1.5 / 6 | PPA-CF | reel axle, free |
| D1/D2 | N20 pinion / drum ring | 12 / 60 / 1 / 6 | PPA-CF | N20 shaft / drum |
| W1 | drum segment: 48 teeth over 240 deg + 2 locking arcs | 72 eq / 1.25 / 8 | PPA-CF | drum |
| W2/W3 | intermediate pinion + locking disc / gear | 24 and 40 / 1.25 / 8 | PPA-CF | W shaft |
| W4 | cup gear | 20 / 1.25 / 8 | PPA-CF | cup shaft, back end |

Lewis check: G2 at the normal peak rack force 87 N: 5.7 MPa; at TL0 slip (179 N): 11.7 MPa (PPA-CF printed about 110
MPa XY). Key bearing stress in G1 at TL0: 7.1 MPa. Worm wheel at the shot reaction (108 N): 11 MPa.

### 3.2 Launch train (forward)

Wiper -> TL0 -> main shaft -> collar (LAUNCH) -> sleeve -> sprag #1 (drives forward) -> G1 24T -> G2 32T (on the rail)
-> rack. Ratio 1.333 wiper turns per big gear turn = per shot. G2 rotates 270 deg (24 pitches = 226.2 mm) while
engaged, lets go at about 278 deg, then the 79 deg gap carries it round to re-engage the rack's rear tooth with the
sled home. The sled flies in 54 ms; the gap takes 0.4 s, so the sled is always home before the first tooth returns.

- Cocking reaction on the rail = drive torque, muzzle DOWN (same sense as gravity, the v10 key fix). Held by the worm.
- Big gear drag brake 0.1 N m (PPA-CF leaf on the G2 hub): stops sprag #1 overrun drag (0.027 N m reflected) from
  creeping G2 out of its gap during aim.
- Backward turns with the springs loaded: sprag #1 locks under the spring torque, so the springs drive the sleeve
  backward and the self-locking wiper lowers them under control. This is the DE-COCK path (it also moves the aim a
  little: about 1 wiper turn = 30 crank deg).

### 3.3 Aim train (backward)

Sleeve -> A1 30T -> swing idler A2 18T (engages only on backward turns) -> A3 15T (2:1 up) -> bevel 1:1 with TL-A
slip cone 0.6 N m -> worm (1-start) -> wheel 24T (24:1) -> crank r 28.7 -> conrod 110 -> lever R 75 -> rail.

- 12 wiper turns per crank turn. 5 to 50 deg takes 6 turns (8.6 s); the crank-rocker has no quick return (rise 0.50
  of a crank turn), minimum transmission angle 63 deg. Mean 0.021 deg pitch per wiper degree; the AS5600 (0.088 deg)
  gives 0.002 deg pitch resolution.
- Worst crank torque 2.1 N m (rail at 5 deg relaxed 0.96 N m + cocking 3.23 N m through the lever, x1.3), worm input
  0.25 N m (efficiency 0.37 at mu 0.2), 0.5 N m at the sleeve. TL-A at 0.6 N m caps the wheel at 5 N m.
- Self-locking: lead angle 7.1 deg locks for mu > 0.125 (printed PPA-CF on PPA-CF is 0.2 to 0.3).
- Load reversal: before the shot the conrod is in tension (gravity + cocking, muzzle down up to 4.2 N m); during the
  54 ms stroke the dart and sled accelerating 116 mm above the pivot push the muzzle UP with 8.5 N m peak (conrod 113 N
  compression). Pin and worm backlash give about 0.4 deg of pitch kick: near 35 to 40 deg the range curve is flat, so
  under 1 %.
- The swing arm on forward turns swings out to its stop; the worm holds. Forward turns never move the aim.

### 3.4 Reel train (forward, REEL only)

Main shaft -> collar (REEL) -> G5 40T -> G6 20T: capstan at 2x wiper speed, 258 mm of line per wiper turn on d 41
(drum 40 + line). Sprag #2 (CSK20PP) inner keyed to the capstan tube, outer keyed into the capstan housing: the capstan
turns only wind-in. The CSK is also the capstan's front radial bearing; the rear is a PPA-CF bushing (PV 0.05 MPa m/s
at 157 N, 90 rpm).

### 3.5 Take-up bus (forward, every mode)

Main shaft -> B1 40T -> swing idler B2 16T (engages only on forward turns) -> B3 44T / B3' 60T compound (free on the
capstan tube) -> B4 22T -> face slip clutch -> storage reel. Ratio 2.5 (= 40/16 overall): 322 mm of take-up capacity per
wiper turn at the empty core, above the capstan's 258 (24 % overdrive) and above cocking (226 mm in 1.0 wiper turn).
Bus drag brake 0.04 N m on B3 (3x the light clutch) parks the bus on backward turns, so the reel cannot be pushed out.

Swing one-way rule (both splitters): the engaged idler position is the end of the "too close" arc that the arm reaches
by swinging in the driving direction; there the mesh forces push the idler deeper (checked at the final positions:
B2 49 F mm, A2 36 F mm into mesh, F = tooth force; reversed, they push out with 76 and 60 F mm) against a hard stop set
at the exact centre distance. A light friction washer (0.01 N m) between arm hub
and sun swings the idler in and out. Reverse rotation swings it 13 deg out to a soft stop, 6 mm tooth clearance.

### 3.6 Drum and wrap train (N20)

N20 worm (self-locking) -> D1 12T -> D2 60T (5:1) -> drum. Drum segment W1 (72T equivalent, 48 teeth over 240 deg) ->
W2 24T (3:1) -> shaft -> W3 40T -> W4 20T (2:1) -> 8 mm cup shaft -> cup. 6 cup turns per drum turn inside the sector:
240 deg of drum = 4 wraps. The locking disc on the W2 shaft rides the drum's locking arcs outside the sector, so the
cup is locked at home at both ends (4 full turns returns it to home). See section 4.

### 3.7 Wiper turns per function

| Function | Wiper turns | Time | Current |
|---|---|---|---|
| Centering jog before any swap | 30 deg back (inside the dog window) | 0.15 s | under 2 A |
| Dog fly-in | up to 120 deg forward at 20 % duty | under 2 s | under 3 A |
| Aim 5 to 50 deg / full up-down cycle | 6 / 12 backward | 8.6 / 17 s | about 2 A |
| One shot (cock, fire, reset) | 1.333 forward | 2.0 s | 5.1 A peak |
| De-cock from full | 1.0 backward | 1.5 s | under 2 A |
| Capstan reel-in / tow | 258 mm per turn | 0.19 m/s light, 0.156 m/s at 60 N | 1.9 / 4.7 A |
| Take-up only (NEUTRAL) | up to 322 mm per turn (clutch slips) | | under 2 A |

---------------------------------------------------------------------------------------------------

## 4. Selector drum

Single-turn barrel d 60 x 72 (PPA-CF) on an 8 mm shaft, hard stops at -5 and 355 deg. Y layout on the drum: W1 segment
and locking arcs Y 40 to 48, code tracks A B C Y 52 to 64, groove 1 Y 66 to 90, track 2 cam Y 90 to 98, D2 ring Y 100 to
106. Forward drum rotation = toward REEL = wrap.

### 4.1 Angle table

| Drum deg | Zone | Collar (groove 1) | Clutch (track 2) | Cup | Code A B C |
|---|---|---|---|---|---|
| -5 to 8 | LAUNCH | L, +9 mm | LIGHT 0.012 N m | locked home | 1 0 0 |
| 8 to 32 | ramp L to N | 5 to 35 deg ramp, 9 mm | light to FIRM (12 to 32) | locked | 1 1 0 |
| 32 to 50 | NEUTRAL | N | FIRM 0.065 N m | locked | 0 1 0 |
| 50 to 62 | wrap prep | N | firm to LIGHT (50 to 60) | lock releases at 60 | 0 1 1 |
| 62 to 302 | WRAP sector | N | LIGHT | 4 turns (1 per 60 deg) | 0 0 1 |
| 302 to 340 | lock, preload, ramp N to R | N to R (318 to 340) | light to FIRM (302 to 316) | locked home from 302 | 1 0 1 |
| 340 to 355 | REEL (wrapped) | R, -9 mm | FIRM | locked | 1 1 1 |

Code 0 0 0 never occurs: it means a broken contact common wire (fault DRUM_SENSOR). Adjacent zones differ in one bit
(Gray), so a single misread gives an adjacent zone, never a far one; an out-of-sequence code is a fault.

### 4.2 What each track does

- Groove 1: helical ramps 9 mm over 30 deg at r 30 (helix 29.8 deg) and dwells. Follower pin on the fork base; the fork
  has a PPA-CF flexure between follower and shoes, 1.33 N/mm, 8 N at the full 6 mm dog depth. If the dogs meet tip to
  tip the drum still completes its move and the flexure snaps the collar in at the first alignment.
- Track 2: radial cam, rise 7 mm over 30 deg (pressure angle 24 deg) -> follower lever on a rocker shaft -> push rod ->
  preload plate on the reel clutch -> PPA-CF flexure spring (about 1.6 N/mm) with an M3 screw to calibrate. Preload
  2.7 N (light 0.012 N m) to 14.4 N (firm 0.065 N m) at r 15, mu 0.3. Preload and fork ramps never overlap in angle.
- W1 segment + locking arcs: drives the cup only inside 62 to 302; locks it at home elsewhere.
- Code tracks: 3 bump tracks under 3 printed contacts (M3 head on M3 nut, flex lever).
- Manual lever on the drum shaft end outside plate R, with a printed detent disc (bumps at the 4 dwell centres).
  The self-locking N20 cannot be back-driven, so the N20 sits on a swing plate: flip the thumb latch, the pinion swings
  out of mesh, turn the lever. The latch has its own contact (SW_N20) so firmware knows the drum is in hand mode.

### 4.3 N20 drive ratio and torque check

N20 right-angle worm, 27 rpm at 12 V, run at 9 V from the MP1584 (DRV8833 VM max 10.8 V): about 20 rpm. ASSUMED torque
0.10 N m continuous, 0.25 N m stall (the dart notes give "rated 1 to 4 kg cm"). Through 5:1 at 0.9: 0.45 N m continuous,
1.12 N m stall at the drum.

| Load at the drum | Torque | Margin vs 0.45 continuous |
|---|---|---|
| Fork ramp at full flexure (8 N) | 0.23 N m | 2.0x |
| Clutch preload cam at 14.4 N | 0.34 N m | 1.3x (separate angles) |
| Wrap: cup 0.028 N m x 6 / 0.81 | 0.21 N m | 2.1x |
| Drum speed | 4 rpm: L to N 1.7 s, wrap sector 10 s, L to R 15 s | |

If the cup snags, the N20 stalls at 1.12 N m on the drum = 6.5 N at the ring: it cannot hurt the line or the cup.

### 4.4 Position sensing

| Sensor | What closes it | Used for |
|---|---|---|
| DRUM_A, DRUM_B, DRUM_C | drum code bumps | zone (table 4.1) |
| SW_DOG_L, SW_DOG_R | fork position after the flexure (collar at least 5 mm engaged) | real collar state; both closed = fault |
| SW_ARM | bump on W4, once per cup turn at home | wrap count, cup home check |
| SW_N20 | thumb latch open | hand mode, firmware will not drive the drum |
| SW_CARR | sled at its front stop (springs relaxed) | cocked or not |
| SW_GAP | bump on G2 within +-20 deg of its gap centre | big gear phase |
| SW_SEAT | sprung plunger in the pusher pushed by the tail cap meets a rail-fixed contact only at sled home | dart seated (no wires on the sled) |
| SW_PITCH | bump on the crank disc at the 5 deg dead centre | pitch zero each crank turn |
| AS5600 | main shaft angle, multi-turn counted in firmware | everything that turns |
| BTS7960 IS | wiper current | stall and tow estimate |
| HX711 + load cell (optional) | under the exit eye | true line tension |

Mode = drum code AND collar contacts must agree: LAUNCH needs 100 + DOG_L; NEUTRAL and WRAP need DOG_L and DOG_R
open; REEL needs 111 + DOG_R.

---------------------------------------------------------------------------------------------------

## 5. Launch physics (2 x Century 16 x 165.1 x 1.4)

- Spring by formula: 97.9 active coils, k 0.125 N/mm, F0 7 N each, 55.4 g each (4.58 m of wire). Pair: 0.25 N/mm, 14 N.
- Rest: sled at the front stop with the springs at zero extension (14 N of initial tension holds it). Spring anchor
  bar has holes at +0, +10, +20 mm preload. Stroke 226.2 mm (24 rack pitches), no pulley: a 2:1 pulley would put 4x
  the spring's effective mass on the sled (148 g instead of 37 g) and lose about 23 % of range.
- Stress at 246 mm (20 mm hole): body 583 MPa, hook bending 1,127 MPa. The hook is the limit: hang test before
  using the +20 hole, look for set after 20 cycles.

| Preload hole | Max extension | Full force | Energy |
|---|---|---|---|
| +0 | 226 mm | 70.5 N | 9.56 J |
| +10 | 236 mm | 73.0 N | 10.13 J |
| +20 | 246 mm | 75.5 N | 10.69 J |

Moving mass: dart 279 g, sled 60 g, springs 37 g effective, reel 16 g effective (spun up during the stroke, 1.8 N).
Losses: sled on PLA V rail mu 0.25, gravity along the rail, 0.75 N tether during the stroke. 66 % of the spring energy
reaches the dart (7.05 J of 10.69 J at 40 deg).

Flight: Cd 1.0 on 2,864 mm2 frontal area, constant tether drag plus line momentum flux (0.6 g/m x v^2, 0.03 N),
launch height 0.52 to 0.56 m (pivot 0.27 m above ground ASSUMED).

| Pitch | v muzzle (+0 / +20) | Range +20 hole, drag 0.5 / 0.75 / 1.0 N | Range +0 hole, 0.75 N | Flight time |
|---|---|---|---|---|
| 30 | 6.72 / 7.14 m/s | 4.47 / 4.17 / 3.89 m | 3.76 m | 0.83 s |
| 35 | 6.70 / 7.12 | 4.58 / 4.22 / 3.88 | 3.79 | 0.92 s |
| 40 | 6.69 / 7.11 | 4.56 / 4.14 / 3.76 | 3.72 | 1.00 s |
| 45 | 6.67 / 7.09 | 4.42 / 3.96 / 3.54 | 3.55 | 1.07 s |

- No tether at all: 5.5 m at 40 deg. Best pitch with drag is 33 to 37 deg.
- Landing speed 5.7 m/s (the dart's landing parts were checked at much higher loads).
- Cocking torque at the wiper: (75.5 N + rack-guide friction from the 20 deg gear separating force) x 48 mm / 1.333 /
  0.92 = 3.23 N m peak = 22 % of the 15 N m stall, 35 rpm, 5.1 A. Firmware launch current trip 7 A, TL0 7 N m.
- Option if Harry buys 2 more of the same spring (4 in parallel): 9.8 m/s, 7.4 m at 40 deg and 0.75 N; cocking 6.5 N m,
  which needs TL0 raised to 9 N m and a PPA-CF shaft check (9 N m = 5.7 MPa, fine).

---------------------------------------------------------------------------------------------------

## 6. Reel physics

### 6.1 Storage reel

- Core d 40, width 20, flanges d 62, 20 m of 1 mm Dyneema fills to d 53.6. Printed reel about 14 g; I = 1.07e-5 kg m2
  with line; 15 g effective at full radius. Shroud (line guard) 2 mm outside the flanges keeps loose turns on the reel.
- Payout: 2,530 rpm at full, 3,300 rpm near the core at 7.1 m/s: 608s are fine.

### 6.2 Payout drag budget (at the dart)

| Source | Value |
|---|---|
| Clutch LIGHT 0.012 N m at r 26.8 to 20.5 | 0.45 to 0.59 N at the reel |
| 608 drag (2 x about 0.0005 N m) | 0.04 N |
| Eyes, 2.0 rad total at mu 0.15 | x 1.35 |
| Breech sheave on 693ZZ | x 1.03 |
| Line momentum flux at 7 m/s | 0.03 N |
| Total at the dart | 0.66 to 0.86 N (target 0.5 to 1 N) |

Spin-up is done by the springs during the 54 ms stroke (1.8 N average extra), never by a snatch: the line already runs
from the reel to the dart's tail through the breech sheave when the sled starts forward.

Overrun at landing (line 4.5 m/s, light clutch): the reel stops in about 150 ms and throws about 2 turns (0.3 m) of loose
line, held on the reel by the shroud and wound back by the NEUTRAL take-up after landing. During flight the reel can
decelerate the line at about 25 m/s2, faster than the dart slows along the line (under 10 m/s2), so no overrun in flight.
Centrifugal shoes: not fitted (they add drag at the start of the flight, where the sim says drag costs most).

### 6.3 Capstan

Drum d 40 x 40 (v7), base flange d 56, cup over the front end (v7 cup: bore 41.2, OD 48, depth 14, ring at r 24.5,
U groove 1.2 x 1.0, 30 deg helix, 58 deg wrap). Euler-Eytelwein hold ratio e^(mu theta):

| mu | 2 wraps | 3 wraps | 4 wraps | 5 wraps | Wraps for 60 N on a 2.5 N tail |
|---|---|---|---|---|---|
| 0.15 | 7 | 17 | 43 | 111 | 3.4 |
| 0.20 | 12 | 43 | 152 | 535 | 2.5 |
| 0.25 | 23 | 111 | 535 | 2,576 | 2.0 |

Decision: 4 wraps. At the worst mu the line slips round the capstan above 105 N (2.4 N tail x 43): a natural limit
below TL0's 157 N. With one wrap lost (3), the capstan still holds 41 N at mu 0.15: degraded, not failed.

### 6.4 Tow, speed, limits

| Line tension | Wiper torque | rpm | Current | Line speed |
|---|---|---|---|---|
| 3 N (dragging the dart) | 0.33 N m | 44 | 1.9 A | 0.189 m/s |
| 10 N | 0.65 | 43 | 2.2 A | 0.185 m/s |
| 30 N | 1.54 | 40 | 3.2 A | 0.173 m/s |
| 60 N (firmware tow limit) | 2.87 | 36 | 4.7 A | 0.156 m/s |

Limits, from soft to hard: firmware tow limit 60 N (from the load cell if fitted, else current 4.7 A minus the running
baseline, calibrated on the bench with a spring scale) -> capstan slip about 105 N at mu 0.15 -> TL0 157 N line -> line
break 667 N. Anchor holding (dart notes): dry sand 7 N, moist loam 45 N, firm clay 112 N, so 60 N is the right default.

### 6.5 Slip clutch and take-up settings

| Setting | Clutch torque | Preload at r 15, mu 0.3 | Line tension at the reel (full / empty) | Used in |
|---|---|---|---|---|
| LIGHT | 0.012 N m | 2.7 N | 0.45 / 0.59 N | LAUNCH (payout brake, cocking take-up), WRAP sector |
| FIRM | 0.065 N m | 14.4 N | 2.4 / 3.2 N | NEUTRAL (final approach, tidy), REEL (tow tail) |
| Bus drag brake | 0.04 N m | | | parks the bus on backward turns |

Final approach on FIRM: pulling the dart up the rail at 5 deg needs 1.1 N (mu 0.3) plus about 0.8 N over the ground: 1.9 N
needed, 2.4 N available. If the ground is rough, the capstan does the work (4 wraps) and only the last 1.5 m is on the
clutch.

---------------------------------------------------------------------------------------------------

## 7. Mission sequence for the firmware

### 7.1 Interlocks (checked before every motion)

- I1 Drum moves only if: wiper stopped, centering jog done, SW_N20 closed, and (if leaving LAUNCH) SW_CARR and SW_GAP
  closed, and (if leaving the WRAP sector toward NEUTRAL) SW_ARM closed with the wrap count at 0.
- I2 Wiper forward in LAUNCH (cock and fire) only if: mode LAUNCH verified (100 + DOG_L), SW_SEAT closed, aim done,
  `fire` token valid.
- I3 Wiper backward only in LAUNCH (aim, de-cock) or as the 30 deg centering jog. Backward in REEL would fight sprag
  #2: firmware refuses it.
- I4 Wiper forward in NEUTRAL or the WRAP sector = take-up only (allowed), duty limited to 40 %.
- I5 Current trips per state: aim 3 A, cock 7 A, tow per the 60 N setting, take-up 2.5 A, jog and fly-in 3 A.
  STALL = AS5600 below 2 rpm for 200 ms with current above the state threshold.
- I6 Drum move timeout 1.5x nominal (L to N 2.6 s, sector 15 s): stop, fault DRUM_TIMEOUT.

### 7.2 Sequence

| # | Step | Mode / drum | Wiper | Ends on (sensor event) |
|---|---|---|---|---|
| 0 | Boot: read drum code, DOG_L/R, SW_CARR, SW_GAP, SW_ARM, SW_SEAT; rebuild state (7.4) | any | off | consistent state, or FAULT |
| 1 | Dart seated, line taut on LIGHT | LAUNCH 0 | | SW_SEAT, SW_CARR, SW_GAP closed |
| 2 | Aim: take up the dog window (known from the last motion) then backward to the pitch from the Jetson table | LAUNCH | back | AS5600 target; SW_PITCH re-zero each turn; dart IMU cross-check |
| 3 | Arm + fire: forward 1.333 turns. Take-up winds the 226 mm as the sled cocks; G2 releases at about 278 deg | LAUNCH | fwd | SW_CARR opens at cock start, current drop at release, SW_CARR closes (sled home), SW_GAP closes (stop). Event FIRED |
| 4 | Flight 1 s, payout on LIGHT | LAUNCH | off | dart reports LANDED |
| 5 | Centering jog 30 deg back, drum to NEUTRAL | 0 to 40 | jog | code 010, DOG_L open |
| 6 | Wait for the dart ANCHORED, then tidy: forward at 20 % until the take-up slips (line straight at 2.4 N) | NEUTRAL | fwd | 2 s of steady current or AS5600 turns count |
| 7 | WRAP: drum 40 to 302 (cup lays 4 turns, line drawn off the reel against LIGHT) | 62 to 302 | off | 4 SW_ARM pulses, SW_ARM closed at 302, code 101 |
| 8 | Engage REEL: drum 302 to 340 while the wiper turns forward at 20 % (fly-in) | 302 to 355 | fwd slow | DOG_R closes, code 111 |
| 9 | TOW: rover winches to the anchored dart | REEL | fwd | Jetson target, or tow limit (event STALL/LIMIT) |
| 10 | Dart releases (`/release`), reel the dart in on the capstan | REEL | fwd | line count says about 1.5 m left, or Jetson camera |
| 11 | Unload: wiper stop, jog 30 deg back (collar free in its window, sprag #2 holds the line) | REEL | jog | |
| 12 | Drum 355 to 302 (collar to N, clutch to LIGHT, cup unlocks at 302) | 302 | off | DOG_R open, code 101 |
| 13 | UNWRAP: drum 302 to 62 while the wiper turns forward at 30 % (take-up only, capstan still) | 302 to 62 | fwd | 4 SW_ARM pulses, code 011 |
| 14 | Drum to NEUTRAL (clutch FIRM), final approach on take-up | 40 | fwd 30 % | SW_SEAT closes (or clutch-slip timeout 20 s: FAULT DART_STUCK) |
| 15 | Drum to LAUNCH (clutch LIGHT) with fly-in forward at most 40 deg (G2 is in its gap, no cocking) | 0 | fwd slow | code 100 + DOG_L |
| 16 | Back to step 2 | | | |

### 7.3 Swap triggers (all are firmware events, nothing is thrown mechanically)

- LAUNCH -> NEUTRAL: FIRED + SW_CARR + SW_GAP closed + dart LANDED.
- NEUTRAL -> WRAP -> REEL: dart ANCHORED + tidy done + operator `wrap`.
- REEL -> unwrap: dart within about 1.5 m (capstan line count minus Jetson's landing distance) or Jetson camera.
- NEUTRAL -> LAUNCH: SW_SEAT closed + wraps 0 + SW_ARM closed.
- Any fault -> NEUTRAL, with two exceptions that the hardware enforces anyway: cocked (stay in LAUNCH and de-cock first),
  wrapped (stay in REEL; going to NEUTRAL means unwrapping, which needs the operator).

### 7.4 Rebuilding state after a reset or power loss

| Reads | Means | Recovery |
|---|---|---|
| 100, DOG_L, CARR closed, GAP closed | LAUNCH, safe | none |
| 100, DOG_L, CARR open | springs partly cocked | operator chooses: finish the shot (if seated and aimed) or de-cock (backward until CARR + GAP close, then re-aim) |
| 111, DOG_R | wrapped, line held by sprag #2 and the worm | resume tow or go to step 11 |
| 001 / 011 / 101 | in the wrap sector | count unknown: run the drum backward to 62 with take-up running (unwrap is safe at any count), then re-wrap if needed |
| 110 or 101 with a DOG open | stopped mid-ramp | step 7.5 "drum stuck" |
| 000 | sensor wiring | FAULT DRUM_SENSOR, hand mode |

Wiper multi-turn count is lost at a reset: G2 phase is recovered from SW_GAP and SW_CARR, crank phase from SW_PITCH (one
aim cycle at most, done only when the next aim is commanded).

### 7.5 Failure walk (lens B)

| Failure | What the design does |
|---|---|
| Drum stuck between positions | N20 stops at the timeout; collar contacts say where the collar really is. If tip-to-tip (DOG open, drum in a ramp): fly-in jog up to 120 deg forward at 20 % (G2 gap gives 40 deg of free travel in LAUNCH; in REEL the capstan only winds in), retry once, else back to the last good zone. Then FAULT and hand lever. The wiper never runs above 3 A while a DOG is half-engaged. |
| Dog teeth tip to tip | 33 % chance per engagement with 1.5 mm chamfers. Fork flexure stores 8 N, the drum completes its move, the fly-in rotation drops the dogs in within 120 deg. Never a jam because the N20 never pushes rigidly. |
| Dog under load | Prevented: every swap starts with a 30 deg jog inside the 60 deg dog window, so the collar touches no flank. If load is still present (cocked springs), the 5 deg back-cut plus friction need 0.34 x tangential force to pull out; at 10 % cock that is already about 15 N, more than the 8 N flexure. The drum moves, the collar stays, DOG_L stays closed with code 010: SWITCH_DISAGREE, drum returns. Worst case energy if the collar did slip out at very low cock: 0.4 J. |
| Line slack in cocking | Take-up winds 226 mm through LIGHT during the same forward turn (CONCEPT CHANGE 1). |
| Line slack in unwrap | Take-up runs forward through the whole unwrap, capstan stationary (CONCEPT CHANGE 2); dart-side tension sits near T_tail / e^(mu theta), so freed line is pulled through the loose turns. |
| Overrun at landing | 0.3 m, kept on the reel by the shroud, wound back in step 6. |
| Slack while aiming | Fairlead swing at most 35 mm: payout against LIGHT, or slack wound in at cocking. |
| Wrap arm snag | N20 stall caps the ring at 6.5 N; drum timeout; firmware backs the drum 30 deg and retries once; SW_ARM pulse count catches a skipped turn; line twist (1 per wrap) is undone on unwrap. |
| Spring cocked during a swap | Firmware never commands it (I1); hardware holds the collar in (back-cut); hazard bounded at 0.4 J. |
| Power loss, any phase | Wiper worm and N20 worm self-lock: nothing moves. Cocked springs stay held by sprag #1 + wiper. Tow load held by sprag #2 + wiper (the rover does not roll back). A shot in progress completes mechanically and payout is passive. Reset table 7.4. |
| Power loss mid-unwrap | Take-up stops, freed line can go slack on the dart side; on reboot the unwrap is resumed with take-up first (step 13 is restartable at any drum angle). |
| AS5600 failure | Detected by its magnet status bits or a count that disagrees with SW_GAP / SW_PITCH edges. Degraded mode: shots run switch to switch (SW_CARR open -> closed -> SW_GAP), aim uses the dart IMU pitch over WiFi with short backward pulses. |
| Contact failure | Gray code catches single drum bits; 000 catches a broken common; DOG contacts cross-check the drum; SW_SEAT cross-checked by the dart IMU (pitch equals the rail, still) and the Jetson camera; SW_CARR vs current profile. |
| Current sense failure | TL0 still limits torque; STALL still detected by AS5600 speed. |
| TL0 slips | Clicks every 60 deg, AS5600 sees no motion at high current: STALL, stop. TL0 re-seats itself. |
| Dart stuck on the ground | Capstan tow limit (STALL event), Jetson decides: pause, retry, or drive the rover. |
| Dart stuck in the muzzle / not seated | Final approach is on the clutch (2.4 N max: cannot damage it). No SW_SEAT = no cocking (I2). FAULT DART_STUCK after 20 s. |
| Misfire (relieved tooth does not let go) | Wiper stalls near 278 deg: STALL, de-cock backward. |
| Big gear creeps out of its gap | G2 drag brake 0.1 N m beats sprag drag 0.027 N m; SW_GAP checked before every forward turn. |
| N20 / DRV8833 dead | Flip the latch, turn the drum by hand; detent disc gives the four positions; SW_N20 tells firmware. |
| Fault -> NEUTRAL exceptions | Cocked: de-cock first. Wrapped: unwrap needs the operator. |

---------------------------------------------------------------------------------------------------

## 8. Part list

### 8.1 Printed (rough size in mm, material, subsystem)

| # | Part | Size | Mat | Sub |
|---|---|---|---|---|
| 1 | Deck plate, 2 pieces (dovetail + M3) | 2 x 160 x 200 x 6 | PLA | frame |
| 2 | Plate R, motor adapter (parametric pattern, drum shaft boss) | 150 x 170 x 8 | PPA-CF | frame |
| 3 | Bulkhead C with boss OD 50 | 80 x 150 x 12 | PPA-CF | frame |
| 4 | Plate L with boss and 608 seat | 120 x 150 x 12 | PLA | frame |
| 5 | Capstan housing (sprag #2 keyed seat, rear bushing) | 70 x 90 x 16 | PPA-CF | frame |
| 6 | Reel bracket + guide eye + shroud | 80 x 100 x 70 | PLA | frame |
| 7 | Aim brackets: A3 shaft, worm shaft, crank shaft bushings (3) | 50 x 70 x 10 each | PPA-CF | frame |
| 8 | Drum bracket + N20 swing plate + thumb latch | 80 x 70 x 10 | PLA | frame |
| 9 | Electronics bay + lid | 150 x 90 x 40 | PLA | frame |
| 10 | Contact holders (10) and cable clips | 15 x 20 x 10 each | PLA | frame |
| 11 | Main shaft 20 keyed, 8 mm end journal, magnet pocket | d 20 x 186 | PPA-CF | frame |
| 12 | Wiper coupler (double-D socket, M8 nut pocket) | d 40 x 25 | PPA-CF | frame |
| 13 | TL0 outer notch ring + inner flexure hub (6 arms) | d 70 x 12 | PPA-CF | frame |
| 14 | Rail beam front half (u 80 to 330, spring post bar, front bumper blocks) | 250 x 70 x 50 | PLA | launch |
| 15 | Rail beam rear half (u -172 to 80, breech sheave mount, pivot relief) | 252 x 70 x 50 | PLA | launch |
| 16 | Rail cheek R with aim lever (lug R 75) | 110 x 110 x 10 | PPA-CF | launch |
| 17 | Rail cheek L with fairlead | 90 x 90 x 10 | PPA-CF | launch |
| 18 | Sled: saddle, pusher with SW_SEAT plunger, 25-tooth rack | 240 x 40 x 60 | PPA-CF | launch |
| 19 | Big gear G2 + 8 mm pin | d 102 x 14 | PPA-CF | launch |
| 20 | Drive gear G1 (47 keyed bore) | d 78 x 16 | PPA-CF | launch |
| 21 | LAUNCH sleeve with dog flange and A1 | d 56 x 56 | PPA-CF | launch |
| 22 | G2 drag brake leaf + M3 adjuster | 40 x 10 x 3 | PPA-CF | launch |
| 23 | Breech sheave d 20 (693ZZ) + front bumper pads 40 x 16 | small | PLA | launch |
| 24 | 6 mm keys (x4), spring anchor hooks (x4) | small | PPA-CF | launch |
| 25 | Swing arm A + idler A2 | 60 x 25 x 12 | PPA-CF | launch (aim) |
| 26 | A3 gear + shaft + bevel | d 25 x 35 | PPA-CF | launch (aim) |
| 27 | Worm shaft with worm and bevel + TL-A cone | d 25 x 100 | PPA-CF | launch (aim) |
| 28 | Worm wheel + crank shaft + crank disc (SW_PITCH bump) | d 65 x 30 | PPA-CF | launch (aim) |
| 29 | Crank arm + pin, conrod L 110 | 130 x 20 x 8 | PPA-CF | launch (aim) |
| 30 | Capstan tube: 20 keyed, G6, base flange, drum d 40 x 40 | d 56 x 111 | PPA-CF | reel |
| 31 | G5 REEL gear with dog slots | d 105 x 16 | PPA-CF | reel |
| 32 | Wrap cup (v7 geometry) | d 48 x 14 | PPA-CF | reel |
| 33 | Cup shaft 8 mm with W4 (if no steel rod) | d 8 x 115 | PPA-CF | reel |
| 34 | Storage reel (608 seats, clutch face) | d 62 x 24 | PLA | reel |
| 35 | B4 + clutch ring, preload plate, flexure spring + M3 | d 40 x 12 | PPA-CF | reel |
| 36 | B3/B3' compound idler + bus drag brake leaf | d 93 x 14 | PPA-CF | reel |
| 37 | B1 bus sun + swing arm B + B2 | d 63 x 8 | PPA-CF | reel |
| 38 | Exit eye block (polished cone bore) | 30 x 30 x 20 | PPA-CF | reel |
| 39 | Selector drum (groove 1, track 2, code, stops, D2, W1) | d 92 x 72 | PPA-CF | drum |
| 40 | Drum shaft 8, manual lever, detent disc | 120 long | PPA-CF / PLA | drum |
| 41 | N20 pinion D1 | d 14 x 8 | PPA-CF | drum |
| 42 | W2/W3 intermediate shaft with locking disc | d 53 x 45 | PPA-CF | drum |
| 43 | Fork with flexure, 2 shoes, follower; fork rod | 90 x 80 x 10 | PPA-CF | drum |
| 44 | Dog collar (3 x 30 deg dogs each face, 6 deep, 5 deg back-cut) | d 56 x 26 | PPA-CF | drum |
| 45 | Track-2 follower lever, rocker shaft, push rod | 150 total | PPA-CF | drum |

### 8.2 Bought (all in hand except the 608s)

| Part | Qty | Use |
|---|---|---|
| 12 V wiper motor | 1 | everything |
| CSK20PP | 2 | #1 cock and fire (in G1), #2 capstan backstop |
| N20 right-angle worm 27 rpm biaxial | 1 | selector drum |
| Century 16 x 165.1 x 1.4 extension springs | 2 | launch |
| 1 mm Dyneema 150 lb | 20 m | tether (about 0.3 m spare for tests) |
| 608ZZ | 8 to 10 | storage reel 2, big gear 2, main shaft end 1, crank shaft 2, cup shaft rear 1, spare 2 |
| 693ZZ | 1 | breech sheave |
| ESP32-S3 DevKitC-1-N8, BTS7960, Adafruit AS5600, DRV8833 #3, MP1584 | 1 each | electronics |
| Diametric magnet for the AS5600 | 1 | CHECK |
| 100 kg load cell + HX711 | optional | tow tension |
| M3 screws, nuts (20 contacts + structure), M3x4x5 inserts (about 40), shaft collars | | frame |
| M8 bolts or 8 mm rod | if present | big gear pin, reel axle, cup shaft; else printed PPA-CF 8 mm |

---------------------------------------------------------------------------------------------------

## 9. Risks, open questions, what Harry must measure

### 9.1 Ranked risks

1. **Range 3.5 to 4.6 m** with the two springs. Physics, not design: 10.7 J at the 246 mm limit. Fix only with more
   spring (2 more of the same: 7.4 m) or a longer stroke after the hang test (300 mm would give about 15 J and a 570 mm
   rail).
2. **Wiper unknowns.** Stall torque, speed and shaft drive TL0, every current threshold and the adapter. A weak motor
   (say 8 N m stall) still cocks (3.23 N m) but at lower margin; a stronger one makes TL0 essential.
3. **Printed swing one-ways (A and B).** Pawl-free and self-energizing on paper; untested in print. Failure is benign
   (aim does not move, or take-up does not run: both detected), but test a coupon on day one.
4. **Payout brake consistency.** 0.012 N m is tiny; PLA face friction varies with humidity and wear. Calibrate with a
   spring scale on the line before every field day; the M3 adjuster is on the outside.
5. **Final approach and seating** (tail first into the muzzle at 2.4 N): kick foot and pods must not catch the muzzle
   lip. Wide funnel, and the camera watches it.
6. **Rail in two halves, 502 mm.** Rack and V grooves across the joint must line up within 0.1 mm or the sled skips.
   Joint at u 80 where the sled is long enough to bridge it.
7. **Mutilated wrap gear transitions.** Locking-arc to tooth hand-over can jam if the cup is turned by hand off-phase.
   SW_ARM checks home at both ends; the latch-out lets Harry re-phase it.
8. **Flexures creeping** (fork, TL0, clutch spring, brakes): PPA-CF creeps little; all have screw adjusters or wide
   margins.
9. **Aim load reversal** at the shot (4.2 N m down to 8.5 N m up): about 0.4 deg kick, under 1 % range.
10. **Spring hooks** at 1,127 MPa (246 mm): hang test.

### 9.2 Open questions

- Rover deck height, mounting holes and free area (layout assumes a 290 x 195 deck footprint, pivot 120 above it).
- Is there a diametric magnet with the AS5600, a spare MP1584 for the DRV8833, any M8 bolts or 8 mm rod?
- Does the dart's tail cap and its two d 4.2 holes take the 157 N TL0 line limit (dart team)?
- Is 60 N the tow limit Harry wants (anchor: 45 N moist loam)?
- Should the base also carry the dart's charging contact (not in this scope)?

### 9.3 What Harry must measure (in this order)

1. Wiper: shaft diameter, flat depth, thread, length; mounting hole pattern; body envelope; free rpm at 12 V;
   stall torque (lever on the shaft, kitchen or bathroom scale) and stall current; park wire.
2. Springs: hang test, extension at 2, 4, 6 kg (k and F0); stretch one to 250 mm 20 times and check it returns.
3. N20 right-angle worm: output shaft diameter and flat; stall torque at 9 V (string on a 10 mm drum and a scale).
4. Dyneema on printed PLA and PPA-CF: wrap the line 1 turn round a printed d 40 cylinder, hang 200 g, pull: mu from
   the ratio (needed for the wrap count and the eye budget).
5. Dart: actual mass with ballast and the tail-cap tether arrangement.
6. Free time to measure later: printed clutch torque vs preload (two settings), TL0 slip torque (lever and scale).
