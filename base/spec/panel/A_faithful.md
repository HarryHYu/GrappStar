# GrappStar base station: integrated architecture, lens A (Harry-faithful)

Status: planning only, 3 Nov 2026. Every number below is computed (scripts: scratchpad `calc.py`, `fsim.py` which runs
`kedge/flight_sim.py` on `out30` with 15 g ballast). Assumed wiper data (45 rpm, 15 N m, 18 A) and formula spring data
(0.125 N/mm, F0 7 N) are marked MEASURE. Units mm, N, N m, deg.

Rule used: keep Harry's v10 launcher (big gear on the rail, relieved last tooth, drive gear on the pivot axis, sprag forward
= cock/fire/reset, crank-rocker pitch 6 to 47 deg, cocking pulls the rail DOWN) and Harry's v7 reel (d40 capstan, wrap cup
with ring and helical groove, storage spool, guide eye and exit eye, 21 mm offset) as literally as the real parts allow.
Every deviation is listed in section 1.2 with its reason.

---------------------------------------------------------------------------------------------------------------------

## 1. Concept summary

### 1.1 Ten lines

1. One wiper motor drives one main axis M (world Y axis). M is also the rail pivot, exactly as v10 (motor axle = pivot).
2. M is two collinear PPA-CF shafts: S1 (wiper side, carries the REEL output gear and the take-up gear) and S2 (launch side,
   carries sprag #1 + the drive gear and the aim sun gear). One sliding dog collar on S1 couples S1 to S2 (LAUNCH) or to
   the REEL output gear (REEL); centre = NEUTRAL.
3. LAUNCH forward: v10 train scaled 0.55: drive gear 12T m5.5 on sprag #1, big gear 19T m5.5 (14 teeth, relieved last
   tooth) on the rail, rack on the carriage. 1.583 wiper turns per shot (Harry's 19:12), stroke 241.9.
4. LAUNCH backward: v10 crank-rocker scaled 0.55 (crank 27.5, rod 143, lug arm 105.6, ground 105.6), pitch 6.0 to 47.4,
   driven through a swing-arm splitter (pawl-free) and a printed single-start worm (self-locks, holds aim while cocking).
4b. No third sprag: sprag #1 cocks, the splitter replaces v10 bearing B, the worm replaces v10 bearing C.
5. REEL forward: capstan (Harry's v7 drum d40) at 2.0x wiper speed through one idler; sprag #2 is the capstan backstop.
6. Storage spool (v7 spool) on 2 x 608, with a printed face slip clutch: brake during payout, take-up during reel-in.
7. Wrap: v7 wrap cup gets a ring gear; the selector drum's segment gear turns it exactly 5.0 turns across the sector past
   REEL and locks it at home everywhere else (locking arc replaces v7's lock pin).
8. Selector drum: single-turn barrel on the worm N20 (6:1): 0 LAUNCH, 40 NEUTRAL, 80 REEL, 85 to 325 wrap, 330 WRAPPED.
9. Two printed 3-bit sensor sets: drum code (3 contacts) and dog-engaged contacts (2), plus carriage, reset, crank, dart,
   line-taut contacts; AS5600 on M.
10. Real range with the Century springs at 248 mm extension: muzzle 6.9 to 7.0 m/s, 3.3 to 4.0 m at 30 to 45 deg with a
    0.5 to 1 N line drag. This is the number Harry must see first (section 5).

### 1.2 Deviations from Harry's originals (each forced, with the reason)

| # | Original | Here | Why |
|---|---|---|---|
| D1 | v10 at full scale: big gear 19T m10, stroke 440 | 19T m5.5, 14 teeth present, stroke 14 x pi x 5.5 = 241.9 | Century springs must stay under 250 mm extension: 6 mm preload + 241.9 = 247.9 |
| D2 | v10 bearing A inside the 12T drive gear (HF1216) | CSK20PP (47 OD) in a cup hub beside the 12T teeth | 12T m5.5 root dia is 52.25: a 47 OD sprag with a 6 mm outer key would leave 2.6 mm of rim. Ratio 19:12 kept |
| D3 | v10 bearing B (one-way pinion) | swing-arm splitter (sun 24T, compound planet) | only 2 sprags exist; plan reserves them for cocking and capstan |
| D4 | v10 bearing C (crank backstop), "drop" past the top | printed single-start worm, 24T wheel = the crank | plan: aim must hold while cocking; a worm also turns the free drop into a controlled glide (Harry accepted the drop, now it is gone) |
| D5 | v10 4-bar orientation: crank centre 29.7 deg up-forward of the pivot, lug straight above the pivot | same 4-bar rotated -72.4 deg about the pivot: crank centre (77.6, -71.6), lug arm at +17.6 deg in rail coords | a worm into a Y-axis crank needs a right-angle stage; this rotation puts the worm shaft vertical beside the splitter and keeps every link length and the 6.0 to 47.4 range identical (checked numerically) |
| D6 | v10 barrel tubes | open cradle on the carriage under the d42 belly, pods clear | KEDGE v30 legs stow on top and flanks (pods reach down to belly level 0.5 mm above ground) |
| D7 | v7 drum on 8 mm D axle, hand crank | 20 mm keyed PPA-CF capstan shaft, sprag #2 backstop, drum bore 20 keyed | 8 mm printed D axle carries about 0.6 N m; the wiper puts up to 6.8 N m on the capstan |
| D8 | v7 spool keyed to its axle, crank | spool free on 2 x 608 + slip clutch | plan (payout inertia and overrun fix) |
| D9 | v7 lock pin and hand-turned cup | ring gear on the cup, drum segment gear with locking arc | plan (N20 wraps, physical order) |
| D10 | v7 layout spool behind, exit eye in front | mirrored: spool in front (+X), exit eye behind (-X), plus one idler in the capstan train | the tether leaves the dart's tail backward to a rail-rear pulley; the mirrored v7 keeps every v7 offset (21 mm, 28 mm, 2 mm) and the line clear of the gears |
| D11 | v7 exit eye a PLA hole | exit and rail-rear turns on printed sheaves with 693ZZ | launch path turns total about 255 deg; on plain eyes e^(0.2 x 4.45) = 2.4x the brake reaches the dart. Sheaves leave only guide eye + ring (80 deg, 1.32x) |

### 1.3 CONCEPT CHANGE (one, quantified)

**CC1: the storage take-up is driven from S1 (upstream of the dog) through a printed ratchet, not from the capstan train.**

- Problem in the baseline: the dart rides on the carriage (it must, or the carriage hits it at 7 m/s). Cocking moves the
  dart 241.9 mm back toward the reel. In LAUNCH mode the baseline take-up (geared to the capstan train) is not driven, so
  241.9 mm of slack forms. At launch the dart then runs the whole stroke on slack and snaps the line tight at the muzzle at
  7 m/s. Spool + line effective mass I/r^2 = 1.0e-5 / 0.0213^2 = 0.022 kg; Dyneema 1 mm braid stiffness about 50 kN per m
  of free line (1 m free): peak = m v sqrt(k/m) = 0.0204 x 7 x sqrt(50000/0.0204) = about 220 N for about 1 ms, an impulse
  of 0.14 N s at the tail. The flight sim says a steady 3 N kills the flight; a 220 N jerk at the tail cap (two d4.2 PLA
  holes) is worse, and it repeats every shot.
- Fix: take-up gear G_T1 48T on S1 (always turning with the wiper) drives the clutch plate at 4.0x wiper speed through
  one idler and a printed 3-pawl ratchet. Forward turns (cocking in LAUNCH, wind-in in REEL) wind the spool through the
  slip clutch; backward turns (aim) click the ratchet and leave the spool alone; with the wiper stopped the ratchet holds
  the plate against the self-locking wiper, so the clutch is the payout brake.
- Result: line taut at full cock (0.56 N), the spool spins up DURING the stroke at 4.3 N peak and 2.2 N mean instead of a
  220 N snap; the same 0.54 J of spool energy, no shock. In REEL mode the plate turns 2.0x the capstan (4.0 / 2.0), still
  overdriven at the empty-spool radius (needs 1.46x, margin 1.37).
- Cost: G_T1, one idler, the ratchet (3 printed flexure pawls, light duty, allowed by the plan). Launch and reel output
  trains stay separate; only this 0.03 N m clutch branch is always connected, and it cannot drive either train.
- Sprag #2 stays as the capstan backstop (holds tow load in any drum state and blocks any powered payout).

---------------------------------------------------------------------------------------------------------------------

## 2. Kinematic layout

### 2.1 Frames and conventions

- World: origin on the wiper output axis at the adapter face (the wiper's mounting face plate). +X forward (launch
  azimuth), +Y left, +Z up. Deck (base plate top) at z = -150.
- M = the wiper output axis = world Y axis. Wiper body on the -Y side (y < 0), envelope assumed 120 x 110 x 160
  (x -60..60, y -110..0, z -50..110) until measured. Adapter bolt pattern: parametric (params.WIPER.mount_holes).
- FORWARD = positive rotation about +Y (right-hand rule) = top of M moves toward +X. Seen from the wiper side (-Y) with
  the launch direction to the right, forward is clockwise. BACKWARD = the opposite.
- Rail coordinates (u, v, w): origin on M, u along the rail toward the muzzle, w perpendicular up, v = world Y.
  World = (u cos p - w sin p, v, u sin p + w cos p) at pitch p (nose up positive). Pitch DOWN = forward rotation sense,
  which is why cocking (forward drive torque) pulls the rail down onto the conrod, as in v10.

### 2.2 Axial (Y) map of the main axis

| y range | item | notes |
|---|---|---|
| -110..0 | wiper motor body | not measured, parametric |
| 0..12 | coupler hub CH, PPA-CF, OD 40 | 10 mm double-D socket (flats 8 apart, parametric), M8 nut pocket, 20 mm keyed socket for S1 |
| 12..22 | frame cheek FR (PLA, 10 thick) | PPA-CF bushing on the CH journal OD 30; also houses sprag #2 for the capstan |
| 8..70 | S1, PPA-CF 20 mm, 6 mm key | nose y 62..70 turned to an 8 mm pilot in a 608 inside S2's dog boss |
| 24..32 | G_T1 take-up gear 48T m1.5, keyed to S1 | CC1 |
| 36..50 | G_R1 REEL output gear 50T m2, free on S1 (PPA-CF bushing) | 4 mm dog pockets in its left face y 46..50 |
| 55..65 | dog collar DC (neutral), PPA-CF, OD 44, 3 dogs each face 4 tall, fork groove 6 wide | REEL: body y 50..60; LAUNCH: body y 60..70; fork travel +-5 |
| 70..80 | S2 dog boss OD 34 (dog pockets y 70..74, 608 pilot pocket inside) | |
| 70..262 | S2, PPA-CF 20 mm, 6 mm key | left journal 8 mm in a 608 in cheek FL |
| 81..97 | sprag #1 (CSK20PP, y 82..96) in the drive-gear cup hub OD 62 | inner key to S2, outer key to the cup |
| 98..114 | drive gear G_L1 12T m5.5, face 16, PPA-CF | same part as the cup |
| 116..126 | rail right pivot cheek, bushing on S2 | v10: rail pivots on the motor axle |
| 127, 197 | the two springs (centres, in rail coords w 123) | below the pods |
| 137..187 | rail beam (50 wide), rail centre y_r = 162 | dart axis at y 162 |
| 208..218 | rail left pivot cheek, bushing on S2 | carries the aim lever (lug arm) out to y 250 |
| 228..236 | aim sun gear 24T m2, keyed to S2 | splitter |
| 228..250 | swing arm on S2 + compound planet (spur 16T m2 half y 228..236, 45 deg helical half y 238..248) | friction washer to the sun |
| 252..262 | frame cheek FL | 608 for S2 journal; bracket for the worm shaft |

### 2.3 Every shaft and body (world coordinates)

| item | position (x, y, z) | axis | notes |
|---|---|---|---|
| wiper output / S1 / S2 / rail pivot | (0, y, 0) | Y | one line |
| big gear pin (on the rail) | rail (60.6, -, 60.6); y 98..114 | Y | world p=6: (53.9, 106, 66.6); p=47.4: (-3.6, 106, 85.6) |
| rack pitch line (on the carriage) | rail w = 112.9, y 98..114 | along u | |
| carriage stroke | rack rear end at u 60.6 (home) to u -181.3 (full cock) | along u | |
| dart axis | rail w = 168, y 162 | along u | belly w 147, pods down to w 147.5 at y 121.7..202.3 |
| dart nose / tail at full cock | u 61 / u -91 | | on the carriage |
| dart nose / tail at home (leaves here) | u 303 / u 151 | | |
| spring posts (muzzle) | rail u 215, y 127 and 197, w 123 | Y | posts d16 PPA-CF on a 48 x 30 bar (v10 style) |
| carriage spring pegs | rail u 44 (home), u -197.9 (full cock), y 127 and 197, w 123 | Y | pegs at the carriage rear |
| rail beam | rail u -205..251, w 105..135, y 137..187 | | 2 printed halves of 228, dovetail + 4 x M3 |
| rail-rear turn sheave (693ZZ) | rail (-210, 162, 150) | Y | p=6: (-224.5, 162, 127.2); p=47.4: (-252.6, 162, -53.0) |
| aim lug pin (on the rail lever) | rail polar (105.6 at +17.6 deg) = (100.7, 250, 31.9) | Y | p=6: (96.8, 250, 42.3); p=47.4: (44.6, 250, 95.7) |
| crank / worm wheel centre (frame) | (77.6, 243, -71.6) | Y | 24T m2.5 wheel, crank pin r 27.5 |
| worm shaft | (33.9, 243, z -95..-30) | Z | worm centred z -71.6; 45 deg helical gear at z -40 |
| swing planet (engaged) | (0, 236, -40) | Y | arm points straight down, swings +X into mesh |
| capstan shaft CS | (0, y 10..118, -95.2) | Y | sprag #2 at y 10..24 in FR |
| capstan train idler G_Ri 20T m2 | (-31.6, 43, -62.7) | Y | |
| capstan gear G_R2 25T m2 | (0, 43, -95.2) | Y | |
| v7 drum (capstan) | (0, y 56..99, -95.2) | Y | flange y 56..59, body d40 y 59..99 |
| v7 wrap cup, ring gear 56T m1 on its rim | (0, y 92..106, -95.2) | Y | ring (line hole) at r 24.5, y 111.6, top |
| exit sheave (693ZZ), v7 "exit eye" | (-70, 139.6, -71.7) | | 28 past the ring, mirrored v7 |
| guide eye (v7 coned d4.4) | (45, 57, -69.2) | X | 2 above the cup top, mirrored v7 |
| storage spool (v7 spool, d52 flanges, d28 barrel) | (85, y 36..78, -95.2) | Y | 2 x 608 on a fixed 8 mm axle |
| clutch plate + T3 12T m1.5 | (85, y 24..36, -95.2) | Y | ratchet inside, preload flexure |
| take-up idler T2 56T m1.5 | (59.4, 28, -50.8) | Y | |
| selector drum | (70, y 51..96, -10) | Y | drum gear 60T m1 y 51..55, segment gear y 56..62, groove 1 y 64..76, track 2 y 78..86, code disc y 88..92; shaft runs out through FR to the manual lever at (70, -15, -10) |
| N20 worm motor pinion 10T m1 | (70, 53, -45.1) | Y | motor hangs below on a swing bracket (manual release) |
| cup pinion shaft: 12T m1.25 + lock disc (y 56..62), 70T m1 (y 92..97) | (52.0, y 56..97, -59.4) | Y | printed 8 mm shaft in 2 bushings |
| fork rod | (35, y 40..90, 8) | Y | 8 mm PPA-CF rod; follower (693ZZ) in groove 1 at (45, 70, -10), yoke on the collar groove at y 60 (L-shaped fork, compliant arm) |

### 2.4 Centre-distance check (every mesh)

| mesh | m | z1 / z2 | m(z1+z2)/2 | built | check |
|---|---|---|---|---|---|
| drive / big gear | 5.5 | 12 / 19 | 85.25 | 85.75 = 60.6 x sqrt2 | +0.50 backlash (v10 used +1.55 at m10) |
| G_R1 / idler G_Ri | 2 | 50 / 20 | 70.00 | 70.2 = |(-31.6, -62.7)| | +0.2 |
| G_Ri / G_R2 | 2 | 20 / 25 | 45.00 | 45.3 = |(31.6, -32.5)| | +0.3 |
| G_T1 / T2 | 1.5 | 48 / 56 | 78.00 | 78.2 = |(59.4, -50.8)| | +0.2 |
| T2 / T3 | 1.5 | 56 / 12 | 51.00 | 51.25 = |(25.6, -44.4)| | +0.25 |
| sun / planet | 2 | 24 / 16 | 40.00 | 40.15 (arm) | +0.15 |
| planet helical / worm-shaft helical (crossed 90 deg, mn 1.5, 45 deg) | 1.5 n | 16 / 16 | 2 x 16.97 = 33.94 | 33.9 + 0.15 | x of worm shaft = 34.1 |
| worm (1 start, pitch dia 27.4) / wheel 24T | 2.5 | - / 24 | 13.7 + 30 = 43.7 | 77.6 - 33.9 = 43.7 | 0 + 0.15 in CAD |
| N20 pinion / drum gear | 1 | 10 / 60 | 35.00 | 35.1 | +0.1 |
| drum segment (72T equivalent) / cup pinion | 1.25 | 72 / 12 | 52.50 | 52.6 = |(18.0, -49.4)| | +0.1 |
| intermediate / cup ring | 1 | 70 / 56 | 63.00 | 63.1 = |(52.0, -35.8)| | +0.1 |
| rack / big gear | 5.5 | rack / 19 | pitch line w = 60.6 + 52.25 = 112.85 | 112.9 | +0.05 |

Senses: G_R1 -> G_Ri -> G_R2: capstan turns WITH the wiper (forward = wind-in, top of drum moves +X toward the guide
eye and spool). G_T1 -> T2 -> T3: plate turns with the wiper at 4.0x (forward = take-up, spool top moves +X away from the
guide eye). Drive gear forward -> big gear top moves -X -> rack and carriage move back = cocking. Swing arm: backward
drags the arm tip (pointing -Z) toward +X into the worm-shaft helical; forward swings it back onto a stop.

### 2.5 Launcher geometry in rail coordinates (side view, muzzle to the right)

```
 w
 ^                                    dart (home, leaves here)        muzzle posts u 215
 |                         tail u151 [=====================] nose u303     |
168  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - |- - dart axis
150  cradle on carriage ...........................................  carriage front u 308 (overhangs)
135  ==== rail beam u -205 .................................... u 251 ====
123  springs (y 127, 197): pegs at carriage rear  ~~~~~~~~~~~~~~~~~~~~~~ posts
113  rack (y 98..114) teeth down: home u 55..308, full cock u -187..66
 61                 ( big gear 19T m5.5 centre (60.6, 60.6), OD 115.5 )
  0  O pivot = wiper axis = drive gear 12T          aim lug (100.6, 31.9) on the y 250 lever
     |
 -> u
```

- Carriage: 268 long (u 40..308 at home, u -201.9..66.1 at full cock), PLA, target 70 g; carries the rack bar (y 98..114, m5.5, 15 teeth incl. the
  first re-engage tooth), the cradle (y 145..179, contacts the belly only within |y - 162| < 16, w 135..150), the pusher
  shoe on the lower half of the tail cap with a 6 mm line slot, and the spring pegs at u 44 (home) on y 127 and 197.
- Springs: pegs at u 44 (home) to posts at u 215 = 171 hook to hook (6 mm preload, 15.5 N total holds home); full cock
  pegs at u -197.9: 412.9 hook to hook = 247.9 extension. The posts sit below the dart path (w 123 vs belly w 147).
- Carriage support: on the beam V grooves over u 40..251 at home (211 engaged), fully on the beam at full cock.
- Dart leaves the cradle 5 mm before the carriage front; its CG passes the edge 10 ms before the tail: about 0.5 mm drop.

### 2.6 Rail at min and max pitch (world)

| point | rail (u, w) | p = 6.0 | p = 47.4 |
|---|---|---|---|
| muzzle (beam front, top) | (251, 135) | (235.5, 160.5) | (70.5, 276.1) |
| dart nose at home | (303, 168) | (283.8, 198.8) | (81.4, 336.8) |
| rail rear end (beam bottom) | (-205, 105) | (-214.9, 83.0) | (-216.0, -79.8) |
| carriage rear at full cock (bottom) | (-201.9, 105) | (-211.8, 83.3) | (-214.0, -77.5) |
| big gear centre / lowest world z | (60.6, 60.6) | (53.9, 66.6) / z 8.9 | (-3.6, 85.6) / z 27.9 |

Clearances checked at p = 47.4: rail rear (-216.0, -79.8) is behind every reel part (storage spool at x +59..+111,
capstan x -24..24, selector drum x 24..116 at y 51..96 vs rail band y 116..218: no y overlap). Selector segment gear (r 46 at (70, -10),
y 56..62) vs dog collar (r 22): 70.7 > 68; drum gear (r 31) vs collar: 70.7 > 53; drum barrel vs sprag cup (r 31,
y 81..97): 70.7 > 56; launch spool line at y 57 passes 15.7 from the segment gear rim. Deck z -150: rail rear stays 70 above it. Big gear never goes below z 9.2: clear of the
capstan (top z -71.2) and drum (top z +36 at x -70 is 70 mm from M; big gear radius 57.75 around (53.9, 66.6) at p 6
reaches x -3.9 at most: clear).

### 2.7 Aim 4-bar (frame side, in the plane y 243 to 250)

- Ground: M to crank centre (77.6, -71.6) = 105.6 at -42.7 deg. Crank 27.5. Rod 143 (PPA-CF, 12 mm pins as v10 x 0.55
  would give 6.6: use 8 mm printed pins, or M3 shoulder screws in F693ZZ). Lug arm 105.6 at +17.6 deg in rail coords.
- Grashof: s + l = 27.5 + 143 = 170.5 < p + q = 211.2: crank-rocker (same as v10).
- Computed: pitch 6.0 to 47.4 deg; rising run takes 159.5 deg of crank (v10 "working run 334 to 174" = 160 deg), falling
  run 200.5 deg. Max slope 0.46 deg of pitch per deg of crank. Toggle checks: rod length check at p 6 = 115.5 = L - r,
  at p 47.4 = 170.4 = L + r.
- Load: rail weight about 1.1 kg with dart, CG about 130 forward of M: 1.4 N m x cos p; cocking reaction up to 2.5 N m
  (drive-gear torque). Both pitch the rail DOWN: rod in compression, as v10.

### 2.8 Line path (launch and reel, same path, no wraps at launch)

```
 storage spool top (85, 57, -74.2)  ->  guide eye (45, 57, -69.2)  ->  cup ring, top of the cup (0, 111.6, -70.7)
   ->  exit sheave (-70, 139.6, -71.7)  ->  rail-rear sheave (p 6: (-224.5, 162, 127.2); p 47.4: (-252.6, 162, -53.0))
   ->  forward along the carriage centre channel (w 150)  ->  pusher slot  ->  tail cap (two d4.2 holes)  ->  dart
```

- Drum top is at z -75.2, the line passes the drum at z -69.2..-70.7: 4.5 to 6 mm above it (v7 values): bypass.
- Turn angles: guide eye 51, ring 29 (plain v7 eyes, mu 0.2: factor 1.32); exit sheave 54 (p 6) to 16 (p 47.4);
  rail-rear sheave 122 (p 6) to 126 (p 47.4) (both on 693ZZ, negligible).
- Path length exit sheave to rail-rear sheave: 252.9 at p 6, 184.9 at p 47.4: aiming up frees 68 mm (ratchet does not take
  up during aim), the next forward run (cocking) takes it up (CC1).

---------------------------------------------------------------------------------------------------------------------

## 3. Trains

Mesh efficiency 0.92 per printed spur mesh. Wiper model (MEASURE): n = 45 (1 - T/15) rpm, I = 1.5 + 16.5 T/15 A.

### 3.1 LAUNCH forward: cock, fire, reset (v10 train)

| element | spec | material |
|---|---|---|
| dog collar to S2 | 3 dogs, 4 deep, 20 mm keyed collar on S1 | PPA-CF |
| sprag #1 | CSK20PP, inner keyed to S2, outer keyed to the drive-gear cup; locks when S2 turns forward | bought |
| drive gear G_L1 | 12T m5.5, 20 deg, face 16, cup hub OD 62 x 16 holding the sprag | PPA-CF |
| big gear | 19T m5.5, face 16, 14 teeth present + 5-tooth gap (94.7 deg); tooth 14 cut to the pitch line tapering toward the rail, tooth 13 30 % taller (v10) | PPA-CF |
| big gear pin | 12 mm PPA-CF pin, cantilevered from the rail's right plate, 2 printed bushings | PPA-CF |
| rack | m5.5 on the carriage, y 98..114, 15 teeth | PLA (Lewis: 77 N / (16 x 5.5 x 0.48) = 1.8 MPa) |

- Ratio 19:12. One shot = one big-gear turn = 1.583 wiper turns: 14 teeth cock (1.167 turns), release at tooth 14,
  the gap passes (0.417 turns) and tooth 1 drops back into the rack at home = reset. Same as v10.
- Peak load at full cock: springs 2 x (7 + 0.125 x 247.9) = 76.0 N + 0.7 N rail friction = 76.7 N. Big gear torque
  76.7 x 0.05225 = 4.0 N m. Wiper torque 76.7 x 0.033 / 0.92^2 = 2.99 N m = 20 % of stall, 4.8 A, 36 rpm.
- Cycle: mean 39.6 rpm, 2.4 s per shot, mean 3.5 A. Hold point for aiming-then-firing on command: stop at 95 % stroke
  (sprag #1 locked by the spring torque, wiper self-locks), then FIRE = run forward to the reset contact.
- Lewis, drive gear 12T (Y 0.245): 76.7 / (16 x 5.5 x 0.245) = 3.6 MPa. Big gear 19T: 2.8 MPa. PPA-CF fine, also for the
  release shock.
- Planetary check: while the carriage is home and the rail pitches, the drive gear turns with the rail (the big gear is
  held by the rack and the 15.5 N preload); pitch up = drive gear backward slower than S2: sprag freewheels; pitch down =
  drive gear forward: sprag overruns. No fight with the aim train at any pitch.

### 3.2 LAUNCH backward: aim (v10 crank-rocker)

| element | spec | material |
|---|---|---|
| sun | 24T m2, face 8, keyed to S2 at y 228..236 | PPA-CF |
| swing arm | on S2, radius 40.15, printed wave washer gives 0.02 N m drag to the sun hub; stop pad for the forward swing | PLA |
| compound planet | 16T m2 spur (y 228..236) + 16T 45 deg helical mn 1.5 (y 238..248), on an F693ZZ pair, M3 axle | PPA-CF |
| worm-shaft helical | 16T 45 deg mn 1.5, crossed 90 deg, at (34.1, 243, -40) | PPA-CF |
| worm | 1 start, axial pitch 7.854 (m2.5), pitch dia 27.4, lead angle 5.2 deg, length 32, centred z -71.6 | PPA-CF |
| worm wheel = crank | 24T m2.5, face 12, at (77.6, 243, -71.6), crank pin r 27.5 printed on | PPA-CF |
| conrod | 143 centres, 8 mm pins | PPA-CF |
| lug | on a lever printed with the rail's left cheek, 105.6 from M at +17.6 deg | PLA |

- Engagement: backward drags the arm (pointing -Z) toward +X: the planet's helical half swings into the worm-shaft helical
  and the drive reaction keeps it there. Forward swings it back onto the stop. Pawl-free, no third sprag.
- Ratio: sun 24 / planet 16 = 1.5 (planet), helical 1:1, worm 1:24: crank turns 1/16 per wiper turn.
- Wiper turns: full rise 6.0 to 47.4 = 159.5 deg of crank = 7.1 turns (about 10 s); fall back = 200.5 deg = 8.9 turns
  (12.4 s); one full cycle 16 turns (22 s). Any pitch is reached by continuing backward (rising or falling run).
- Resolution: steepest 0.46 deg pitch per crank deg = 10.4 deg pitch per wiper turn; AS5600 (4096 counts) gives 0.0025
  deg of pitch per count. Crank home contact at min pitch re-zeros every cycle.
- Aim load: rail 1.4 N m: crank torque about 0.5 N m peak, worm-wheel tooth 17 N, worm input 0.07 N m, about 0.2 N m at
  the wiper (negligible, so the splitter's 0.02 N m drag decides the swing reliably).
- Hold while cocking: rail moment 1.4 + 2.5 = 3.9 N m, crank torque up to about 1.4 N m, wheel tooth 47 N (Lewis 4.6 MPa).
  Worm back-drive: lead 5.2 deg < friction angle 12 deg (mu 0.2) or 6.1 deg (mu 0.1): self-locks with margin. Driving
  efficiency tan 5.2 / tan 17.2 = 0.29 (fine at aim loads).

### 3.3 REEL forward: capstan (v7 drum)

| element | spec | material |
|---|---|---|
| dog collar to G_R1 | same collar, other face | PPA-CF |
| G_R1 | 50T m2, face 14, free on S1 on a PPA-CF bushing, dog pockets in its left face | PPA-CF |
| idler G_Ri | 20T m2, face 14, on an 8 mm axle in 2 x 608 at (-31.6, 43, -62.7) | PPA-CF |
| G_R2 | 25T m2, face 14, keyed to CS | PPA-CF |
| capstan shaft CS | 20 mm keyed PPA-CF, y 10..118 | PPA-CF |
| sprag #2 | CSK20PP backstop: inner keyed to CS, outer keyed into the FR housing; CS can only turn wind-in | bought |
| drum | Harry's v7 drum: root d40 x 40, flange cone d46, bore changed to 20 keyed | PPA-CF |

- Ratio: capstan 2.0 x wiper (50/25; idler for sense). Line radius 20.5.
- 1 wiper turn = 2 capstan turns = 257.6 mm of line. 20 m = 77.6 wiper turns.
- Line speed: slack (about 0.3 N m) 0.189 m/s; 50 N tow 0.162 m/s (wiper 2.42 N m, 4.2 A); 100 N 0.131 m/s (4.85 N m,
  6.8 A). 20 m in about 125 s.
- Worst case at wiper stall: 15 x 0.85 / 2 / 0.0205 = 311 N, under the 667 N line rating. Capstan shaft at 6.4 N m:
  16 T / (pi d^3) = 4.1 MPa, inside the 8 MPa printed-shaft budget.

### 3.4 Storage spool: take-up and brake (CC1)

| element | spec | material |
|---|---|---|
| G_T1 | 48T m1.5, face 8, keyed to S1 at y 24..32 | PLA |
| idler T2 | 56T m1.5, face 8, at (59.4, 28, -50.8) on an M8/8 mm axle with a printed bushing | PLA |
| T3 + ratchet | 12T m1.5 hub with an 18-tooth internal ratchet ring | PPA-CF |
| clutch plate | d50 disc with 3 printed flexure pawls, friction annulus r 14..24 (mean 19) against the spool's right flange | PLA |
| preload | PPA-CF leaf 70 x 12 x 2.5 (about 1 N/mm, MEASURE) behind the plate, M3 set screw for the base preload, pushed by the track 2 rod | PPA-CF |
| spool | Harry's v7 spool (d52 flanges, d28 barrel, 26 barrel length, about 25 m capacity), bore opened to 22 for 2 x 608 | PLA 2 walls 15 % |
| spool axle | fixed M8 bolt, or 8 mm PPA-CF axle | |
| shroud | printed cover round the spool, 1.5 clearance to the flange rims, line exit slot toward the guide eye | PLA |
| drag pad | PLA flexure on the plate rim, 0.004 N m | PLA |

- Plate speed 4.0 x wiper (48/12), same sense as the wiper: forward = take-up.
- LIGHT (LAUNCH, NEUTRAL): 0.012 N m (preload 2.5 N at mu 0.25, r 19): line 0.56 N at a full spool (r 21.3), 0.86 N near
  empty (r 14).
- FIRM (REEL and past): 0.032 N m (preload 6.7 N, track 2 adds 4.4 mm): line 1.5 N full, 2.3 N near empty.
- Ratchet: holds the brake reaction (0.032 N m max) against the self-locking wiper; clicks during aim.

### 3.5 Wrap train (selector drum to cup)

| element | spec | material |
|---|---|---|
| N20 pinion / drum gear | 10T m1 on the N20 worm output / 60T m1 on the drum: 6:1 | PPA-CF / PLA |
| segment gear | m1.25, 72T pitch circle (r 45), 48 teeth spanning drum 85..325 deg, convex locking arc elsewhere | PPA-CF |
| cup pinion + lock disc | 12T m1.25 + concave locking disc (Geneva-style lock), same shaft | PPA-CF |
| intermediate | 70T m1, same shaft | PLA |
| cup ring gear | 56T m1 printed on Harry's v7 wrap cup rim (cup unchanged otherwise: OD 48, ring 0.5 in front of the face, 1 mm outside the wall, U groove 30 deg helix 58 deg) | PLA (cup) |

- Drum to cup = (72/12) x (70/56) = 7.5. 240 deg of drum = 5.0 cup turns (ring back at the top) = 4.0 pinion turns
  (lock disc back in line). Both integers, so the cup is locked at the same home angle before and after wrapping.

### 3.6 Wiper turns per function (summary)

| function | wiper turns | time |
|---|---|---|
| full aim rise 6.0 to 47.4 | 7.1 backward | 10 s |
| full aim fall 47.4 to 6.0 | 8.9 backward | 12.4 s |
| one shot (cock + fire + reset) | 1.583 forward | 2.4 s |
| take up the 242 mm cocking slack (inside the cock) | 0.45 of the 1.17 cocking turns | free |
| capstan, 1 m of line | 3.9 forward | 24 s at 50 N |
| capstan, 20 m | 77.6 forward | 125 s |

---------------------------------------------------------------------------------------------------------------------

## 4. Selector drum

### 4.1 Angle table (drum deg; hard stops at -8 and 342)

| drum deg | state | groove 1 (fork) | track 2 (clutch) | segment gear / cup | code C1 C2 C3 |
|---|---|---|---|---|---|
| -5..5 | LAUNCH detent | +5 (collar on S2) | LIGHT | locked by the arc | 1 0 0 |
| 5..35 | shifting | +5 to 0 | LIGHT | locked | 0 0 0 |
| 35..45 | NEUTRAL detent | 0 (both dogs 1 mm clear) | LIGHT | locked | 0 1 0 |
| 45..75 | shifting | 0 to -5 | LIGHT, ramp to FIRM 55..75 | locked | 0 0 0 |
| 75..85 | REEL detent, zero wraps | -5 (collar on G_R1) | FIRM | locked | 0 0 1 |
| 85..325 | wrap sector | -5 dwell | FIRM | teeth engaged: 5.0 cup turns | 0 1 1 |
| 325..335 | WRAPPED detent (tow) | -5 dwell | FIRM | locked | 1 0 1 |
| 335..342 | overtravel to stop | -5 | FIRM | locked | 1 0 1 |

- Groove 1: barrel d50, 693ZZ roller follower in an 8.3 wide groove, 5 mm per shift over 30 deg: rise 13.1 mm of arc,
  pressure angle 20.9 deg. Fork arm compliant (printed flexure, about 10 N at 3 mm) so the drum can finish a shift while the
  dogs sit tooth on tooth; the firmware then jogs the wiper (section 7).
- Track 2: radial lobe, base r 22, FIRM r 26.4 (4.4 lift) over 55..75 deg, pressure angle 27.6 deg; 693ZZ roller on a
  printed lever and a 6 mm PPA-CF push rod (about 100 long, printed guides) to the clutch leaf at (85, 30, -95.2).
- Segment gear: teeth only over 85..325; locking arc over the rest; a relief cut at both ends of the arc lets the lock disc
  turn as the first and last teeth mesh.
- Order is physical: wraps exist only past 85 deg; LAUNCH (0) is reachable only by turning back through the whole wrap
  sector, which unwraps. Dogs: LAUNCH and REEL are 10 mm apart on one collar, so both can never engage.

### 4.2 N20 drive and torque check

- N20 right-angle worm 12 V 27 rpm (Harry's), fed 10.0 V from the MP1584 through DRV8833 #3: about 22 rpm. Torque not
  measured: design value 0.2 N m stall at 10 V (MEASURE; the dart notes give 1 to 4 kg cm).
- 6:1 to the drum, efficiency 0.9: 1.08 N m available at the drum; drum speed 3.7 rpm = 22 deg/s.
- Loads: fork shift 10 N at r 25, pressure 20.9 + friction 11.3 deg: 0.16 N m. Track 2 ramp 6.7 N at r 24, 27.6 + 11.3
  deg: 0.14 N m (overlaps the fork shift: 0.30 N m). Wrapping: line 1.6 N (FIRM at r 20.1 with 16 m still on the spool)
  x 24.5 mm x 1.5 (groove and ring) + 0.02 bushing = 0.079 N m at the cup, x 7.5 / 0.846 = 0.70 N m at the drum.
  Worst = 0.70 N m: margin 1.54. If the hang-test N20 is weaker, go 8:1 (80T drum gear, 16.5 deg/s).
- Times: LAUNCH to NEUTRAL 1.8 s, to REEL 1.8 s, wrap 10.9 s, LAUNCH to WRAPPED 15 s.
- Holding: N20 worm self-locks; the drum holds any angle unpowered.
- Manual: the drum shaft runs out through FR to a lever at (70, -15, -10). To hand-turn, loosen one M3 thumb screw and
  swing the N20 bracket 8 deg out of mesh (the worm cannot be back-driven), turn the lever, swing back.

### 4.3 Position sensing

- 3 printed contacts on the code disc (y 88..92): printed flex lever, M3 screw head touching an M3 nut, ESP32 pull-up.
- Codes: 100 LAUNCH, 010 NEUTRAL, 001 REEL, 011 wrapping, 101 WRAPPED, 000 moving between detents. 110 and 111 never
  occur: either one = contact fault -> FAULT.
- Dog contacts: DOG_L (collar fully on S2) and DOG_R (collar fully on G_R1), printed contacts pressed by the fork.
  Mode is valid only when the drum code and the dog contact agree.
- Angle between edges: N20 run time x 22 deg/s, re-synced at every code edge (edges every 30 to 240 deg).
