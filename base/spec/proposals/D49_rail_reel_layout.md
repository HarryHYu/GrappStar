# Rail reel layout study (RR1): storage spool, capstan and selector drum on the rail

Status: APPLIED in params 1.4.0 (D49); positions as built are in params RAIL_REEL. Original study status: layout numbers only. Base: branch base-v1, params 1.3.8, assembled meshes base/out/base (3 Nov 20:48).
Scripts and raw output (all in this folder): `dump_states.py` (posed world transforms, 9 pitches x 3 cock states,
`states.pkl`), `envelopes.py`, `profile.py` (rear profile of the carriage group at full cock), `layout_check.py`
(every new element as a primitive, checked against the real meshes, result `layout_check.json`), `mass_aim.py`
(pitching mass and aim torque), `phys_range.py` (base/physics models run in memory with overrides).

Frames: RAIL frame as cad-launch: origin on the main axis, u along the rail toward the muzzle, w normal (up at pitch 0),
y = world y (invariant under pitch; rail centre plane y 140). World: x = u cos p - w sin p, z = u sin p + w cos p.
Deck rule: nothing below z -160 (5 mm over the deck-top plane, extended behind the deck as the rover is unknown).

## 0. The layout in one paragraph

The line runs on the dart axis (y 140, w 84.6) from the top of the spool pack, through an AXIAL eyelet on the cup, to the
dart tail: one straight segment at every carriage position and pitch, because spool, eyelet and dart all ride the
rail. The capstan sits ABOVE the line (axis y 140, w 110.6, parallel to u, 26 mm from the line) and the spool BELOW it
(axis parallel to y at u -224, w 57.6). This is the only arrangement that fits: a capstan below the line hits the
carriage SW feet or the deck at 47 deg, a spool above the line collides with the capstan or ends behind the deck
limit (section 4). The capstan gears, the cup gear, the wrap idler and a rail drum with its axis parallel to u fill
the space behind the spool tangent (where the line no longer exists). Two drives cross the pivot on an inboard
"bridge": REEL gear to countershaft C (capstan) and bus gear to c1' / swing idler / countershaft T (take-up). Both
turn 90 deg into shafts along the rail (u-shafts) on an inboard spine at y 64 and y 76.5.

## 1. Element positions (rail frame, mm)

### 1.1 Line path (all pitches 6 to 47 identical in the rail frame)

| point | u | y | w | note |
|---|---|---|---|---|
| spool tangent (top of pack, r 27.0) | -224.0 | 129..149 (pack) | 84.6 | design pack r 27.0 (about 2 m out); full 20 m r 27.4 (+0.4 mm), 15 m left r 25.75 (-1.25 mm, 1.8 deg) |
| eyelet (axial ring, d 2.8 bore, coned 4.0 mouths) | -183 .. -187 | 140 | 84.6 | at cup home the ring is at the cup BOTTOM, 26.0 from the capstan axis |
| dart tail at HOME | 78.6 | 140 | 84.6 | line through the pusher hole (u 70.6) as today |
| dart tail at full cock | -162.3 | 140 | 84.6 | 20.7 mm of line between tail and eyelet |
| in flight | along +u from the pusher | 140 | 84.6 | leaves over the cradle; lip roller only when the dart is below the axis (tow, retrieve), as today |

Spool tangent to dart tail: 302.6 mm at HOME, 61.7 mm at full cock. The path only gets longer during the stroke (D36
satisfied with no guide). Pitch changes nothing (no pivot pulley needed, D14/D34 obsolete).
World positions (x, z): eyelet (-193, 65) at 6 deg, (-203, 16) at 20, (-199, -40) at 36, (-188, -78) at 47;
spool tangent (-232, 61) / (-239, 3) / (-231, -63) / (-215, -106).
Checked: the cup surface clears the line by 2.0 mm (1.5 mm gap, line r 0.5), the drum bottom by 6.0, the drum base
flange by 3.0; the line ends at u -224, ahead of everything at u < -226.

Not perfectly straight, and cannot be: (a) fleet angle. The pack is 20 wide and the tangent is 37 to 41 mm behind the
eyelet: the line swings up to 15 to 17 deg sideways at the eyelet mouth as it unwinds across the pack (path gain about
e^(0.15 x 0.28) = 1.04, used in section 6). (b) pack radius: 1.2 deg vertical over the launch range. No fixed corner.

### 1.2 Reel head (bolts to the rail rear end u -178.3; rail rear fork and the 3 tether pulleys are deleted)

| element | axis / centre (y, w) | u range | size | checked clearance (mm) |
|---|---|---|---|---|
| eyelet lug on the cup (axial ring) | (140, 84.6) | -183..-187 | ring r 3.4 | carriage group at full cock 4.9 |
| v7 wrap cup | capstan (140, 110.6) | face -187, rim -201 | r 24 | carriage 2.7 (target 3: move the whole head 0.5 mm back if CAD confirms) |
| v7 capstan drum (unchanged) | capstan | nose -194, flange back face -237 | r 20 / flange r 23 | deck 50 (z -110 at 47 deg) |
| head plate P1 (tube C bushing) | | -238..-244 | lower edge shaped round the spool | |
| tube C (PPA-CF 20 OD) | capstan | -237..-256 | | |
| G_c 23T m1.5 on C / G_a 26T on u-shaft C / G_b 32T idler | (140, 110.6) / (76.5, 99.0) / (104.8, 132.1) | -245..-255 | tips r 18.75 / 21 / 25.5 | spool top 3+, deck 26..46 |
| drum segment (39 of 60T m1.25) + W pinion 10T m1.25 | drum (52, 150) / W (82.8, 118.6) | -257..-264 | r 38 / 8 | (reordered after check, see 4.4) |
| G_k cup gear 28T m1.5 on K + W output 49T m1.5 | capstan / W | -266..-273 | tips 22.5 / 38.25 | deck: W bottom w 80 vs limit 58 |
| rear plate P2 (K, W, drum, u-shaft bushings) | | -275..-281 | | lowest corner z -151 at 47 deg (14 mm) |
| rail drum (axis parallel to u) | (52, 150) | -182..-264 | barrel r 30, drum gear 120T m0.8 r 48.8 at -182..-190 (front end) | carriage 27, dart and guards clear, rail 19 |
| N20 (right-angle worm) on the drum gear, cd 52 | (3, 132) inboard of the drum | -170..-200 | | was at (78, 105) in the run: 0.1 mm from u-shaft C, so it moves inboard |
| spool (core d40, flange d64, pack 20 wide) | axis parallel to y at (u -224, w 57.6); pack y 129..149, flanges 127..129 / 149..151 | -256..-192 | | carriage 2.1 (SW_SEATED plunger y 153); deck z -156.7 at 47 deg |
| spool shroud r 35.5, open over the front-top quadrant | same | -259.5..-188.5 | y 125.5..152.5 | deck z -160.2 at 47.06 deg (4.8 mm, at world x -194, 30 mm behind the deck edge) |
| spool shaft S (8 mm) with the clutch disk, 2 x 608 in the spool | (u -224, w 57.6) | y 70..153 | | |
| bevel B3 (u-shaft T to S, m1 20/20) | apex (u -224, y 64, w 57.6) | | r 11 | |

Head envelope: u -178..-281, y 3..165, w 22..199. Today the rail ends at u -210.9; the rail is now 70 mm longer at
the rear. At 6 deg the head reaches world x -296 (today -219, deck rear edge -164).

### 1.3 Bridge (rail-fixed, near the pivot, inboard of the rail) and spine

| element | axis (u, w) or (y, w) | y / u range | size | clearance |
|---|---|---|---|---|
| countershaft C (axis parallel to y) | (u 0, w 99) | y 33..72 | 20 at the sprag | |
| sprag #2 (CSK20PP, moved from the capstan tube) in a hub | same | y 34..48 | hub r 27 | frame guard hood 1.1 at 39.5 deg: hood must be cut back |
| C1 49T m2 face 12 (meshes the REEL gear, cd 99) | same | y 50.5..62.5 | tip r 51 | |
| B1 bevel 24T:12T m1.5 (2:1) | apex (u 0, y 76.5, w 99) | gear y 63.5..71.5 | r 19.5 | spring guard r_a 1.9: trim its inboard wall locally (u -20..+20) |
| u-shaft C (10 PPA-CF, hollow) | (y 76.5, w 99) | u -18..-256 | | yoke sweep 3, dart 14, guard 8.8, hood 18.7 |
| countershaft T (axis parallel to y) | (u -110, w 57.6) | y 36..60 | | |
| T1 20T m1.5 / B2 bevel m1 20/20 | same | y 40..48 / 51..58 | | |
| c1' 46T m1.5, free on a stub (meshes the bus, cd 72.1) | (u -72.1, w -2.5) | y 40..46 | tip r 36 | |
| swing idler 30T m1.5 on an arm round c1' hub (57 from c1', 37.5 from T1) | (u -123.3, w 22.5) | y 40..46 | 12 deg swing | 17 mm to everything else |
| u-shaft T (8 PPA-CF) | (y 64, w 57.6) | u -118..-216 | | guard r_a 4.0 |
| fork shift rail (axis parallel to y) | (u -40, w -52), r 65.6 | y 71..100 | 10 | REEL gear tip 8.6 (the run at r 62 gave 4.2) |
| fork sleeve + compliant yoke (today's fork) | on the shift rail | yoke in the collar groove y 82..88 | | groove is a full ring: works at any pitch |
| bell crank, pivot parallel to w, arms 46 + 46 (1:1) | pivot (u -75, y 71, w -52) | | | |
| link (ball ends) drum follower to bell crank | from (u -190, y 25, w 115) to (u -75, y 25, w -52) | | | inboard of all bridge gears (they are at y >= 33) |
| dog post with SW_DOG_L / SW_DOG_R | on the bridge beside the fork | y 69..112 | | |
| bridge: inboard plate y 28..33, mid plate y 72..78 (between C1/B1 and the guards), joined to rail cheek A (y 123..131) by two box arms; optional plain journal r 22 on the outer coupler at y 28..32 for mesh accuracy | | | | |

Swept-volume checks (frame, aim housing, sensors and the main-axis stack, every 2 deg from 5.5 to 47.5): only the
intended meshes touch (C1 with the REEL gear, c1' with the bus gear). Nearest non-mesh: sprag #2 hub to the frame guard
hood 1.1 mm, B1 3.5, countershaft C 5.3 (hood redesign, cad-frame), fork rail to the REEL gear (fixed by r 65.6).
Remaining rail parts: B1 1.9 and u-shaft T 4.0 from launch_guard_r_a (local trim). Deck at 47.06 deg: worst shroud z -160.2, spool -156.7, P2 -151.

## 2. Gear trains

### 2.1 Capstan (REEL mode)
REEL gear m2 50T (main axis, y 50.5..62.5, unchanged) -> C1 m2 49T (cd 99, 1.020) -> B1 bevel 24T:12T m1.5 (2.000)
-> u-shaft C -> G_a 26T m1.5 -> G_b 32T idler (cd 43.5) -> G_c 23T m1.5 on tube C (cd 41.25; 26/23 = 1.130).
Capstan = 2.307 x wiper (today 2.273). Line speed at 45 rpm on the 41 mm line circle: 0.223 m/s free (today 0.21),
about 0.15 m/s at 50 N. Efficiency estimate 0.84 (3 spur meshes, 1 bevel, bearings; today 0.85).
At the 5 N m shear pin: capstan 1.82 N m, 89 N at the capstan, 75 N at the dart with path gain 1.18 (the line now
reaches the ring straight; only the ring-to-groove bend (plain eye, about 60 deg) and the lip roller remain; today
1.30). At 0.8 x pin: 60 N at the dart, so the D46 cap of 55 N can stay. Wiper stall 15 N m: 266 N at the capstan (below
the 667 N break). Tension law (to recalibrate): capstan side about 16.6 N/A, dart side about 14.1 N/A.
u-shaft C torque at the pin 2.45 N m (B1 halves it): d10 PPA-CF 12.5 MPa; wind-up about 0.4 rad over 238 mm, elastic.
An 8 mm steel rod (D19) cuts the wind-up to 0.06 rad for about +75 g; decide at CAD.
Sprag #2 now on countershaft C (1.02 x wiper): holds about 4.6 N m at a 100 N line pull (rating 50 N m), reacting into
the bridge (rail).

### 2.2 Take-up (every forward wiper turn)
bus m1.5 50T (coupler, y 40..48, unchanged) -> c1' 46T (free, cd 72.1) -> swing idler 30T (cd 57) -> T1 20T (cd 37.5)
on countershaft T: 50/20 = 2.5 x wiper (as today) -> B2 m1 20/20 -> u-shaft T -> B3 m1 20/20 -> shaft S = clutch disk at
2.5 x wiper. Take-up line speed before the clutch slips: 0.318 m/s at pack r 27, 0.236 at the bare core r 20, i.e.
1.43 / 1.06 x the capstan line speed (the core is never reached in a tow: at least 15 m stay on the spool).
Cock: 425 forward wiper deg = 2.95 spool turns = 500 mm of take-up capacity for 241 mm of slack. Clutch: FIRM 5.0 N at
r 27 = 135 N mm; launch brake 0.1 N = 2.7 N mm (owner); 608 drag about 0.4 N mm (0.015 N).

### 2.3 Wrap (N20, all on the rail)
N20 10T m0.8 -> drum gear 120T (12:1, 8.1 deg/s, unchanged) -> segment 39 of 60T m1.25 over drum 62..302 -> W pinion
10T m1.25 (cd 44.0, 3.9 turns, about 4) -> W output 49T m1.5 -> G_k 28T m1.5 on K (cd 57.75, 49/28 = 1.75, same as 42/24
today) = 7 cup turns, ring home at both ends. The drum zone table, park angles and code edges do not change (params
DRUM unchanged); only the cup gear (24T to 28T) and W output (42T to 49T) change, both on the rail.
Cup laying torque, lock arc and N20 margin as today (ratio identical).

### 2.4 Main shaft: what moves axially
Nothing on the main shaft has to move for the trains: C1 meshes the REEL gear where it is (y 50.5..62.5) and c1' meshes
the bus gear where it is (y 40..48). The order bus / REEL / collar / LAUNCH ring / sleeve cannot change (the collar
must sit between REEL and LAUNCH, the bus upstream of the dog), so nothing could get closer to the rail anyway; the
bridge spans the 60 to 90 mm from rail cheek A instead. What moves OFF the main-axis zone: the frame-fixed drum rail and
dog post (SW_DOG_L/R go to the bridge), the drum, the wrap idler, the reel wall and capstan. SW_INDEX must stay
FRAME-fixed (re-mount on frame_cheek_r or a frame bracket): read from the rail it would add the pitch to the count. Optional:
a plain journal seat (r 22, y 28..32) on the outer coupler for the bridge's inboard plate.

## 3. Planetary effect of pitching (rail pitches only in LAUNCH: aim, decock, rail_low)

- Capstan train: with the REEL gear free on its journal (collar in LAUNCH), pitching by dp rolls C1 round it: the capstan
  would turn 2.307 dp relative to the rail (95 deg over the full 41 deg). In the payout sense sprag #2 locks and the
  free REEL gear is carried round instead; in the wind-in sense whichever has less drag turns. The line lies 6 mm under
  the drum unwrapped, so a capstan turn moves no line. Harmless. The coupler drag on the REEL journal during aim (wiper
  backward) pushes the capstan in its payout sense, sprag #2 holds, the journal slips: as today.
- Take-up train: the bus turns with the wiper; relative to the rail it turns (wiper - pitch). The aim table slope is at
  most 4.5 pitch deg per 100 wiper deg, so the relative sense never reverses: backward wiper runs keep the swing idler
  out, forward runs (cock, tidy, approach, reel) never pitch. Harmless.
- Fork: the yoke orbits in the full-ring collar groove; axial position unchanged. SW_DOG on the bridge moves with the fork.
- Line length spool-to-dart is pitch independent (all on the rail): the aim no longer pulls or feeds line.
- What holds what in tow (rail at 6 deg = crank LOW dead point, D45): line pull on the lip roller about 133 N mm per N of
  tension (7.3 N m at 55 N) plus the REEL-mesh reaction (+-main shaft torque) act on the rail; at the dead point the
  conrod is in line with the crank, so the worm sees almost none of it; conrod about 70 to 100 N. Never tow above the
  dead point (rule exists, D45).
- Cables: N20 (2), SW_DRUM A/B/C + common, SW_ARM, SW_DOG L/R, SW_N20 cross the pivot: about 10 conductors in a flex
  loop over 41 deg (SW_HOME, SW_SEATED already ride the rail).

## 4. Keep-outs and fit

### 4.1 Carriage stroke (241 mm) and carriage group at full cock (`profile.py`)
Rear profile at full cock: carriage body u -170.3; yoke arms -174..-178 (w 60..80, y 76..203); SW_HOME foot
-202.3 (y 112..118.5, w 68..76); SW_SEATED plunger -200.3 (y 153..168, w 71.5..76); left yoke block -184 (y 150..162,
w 76..92); dart legs -179 (w 88..98); dart above w 104 only -155. Consequences: anything BELOW the line inside y
119..161 must start behind u -203 (feet), which kills a capstan below the line (drum then ends at -261: z -164.6 at 47
deg). Above the line the cup can start at -187. The spool below the line fits between the two feet (y 127..151,
2.1 mm from the plunger; centring the spool at y 138.5 gives 3.5 mm).

### 4.2 Deck and rover (47.06 deg is the worst pitch for everything behind the pivot)
Below-line items limit: z = 0.7314 u + 0.682 w >= -160. Spool: u_s >= -227.8 for flange r 32; chosen -224 (shroud 4.8 mm,
spool 8.3 mm over the plane). The real deck tile (rear edge x -164) is not under any of it (worst point at x -194):
the margin really protects the rover body. ROVER CHECK NEEDED: keep the region behind x -164 below z -165 clear for
y 100..170 (the rail tip already overhangs there today).

### 4.3 Other keep-outs
Big gear sweep (u 30..135, w >= -52.5): nothing new ahead of u -18 except the bridge at u 0 above w 48 (C1): clear.
Aim crank-rocker and housing (y 174..233, frame): nothing new outboard of y 165. Springs and guards (y 72..112, 168..208,
u -181..69): u-shafts at y 64 / 76.5 pass inboard of / above them (3 to 9 mm); B1 needs a local notch in guard r_a.
Frame guard hood (y 0..60 over the coupler): must be cut back for the C1 / sprag hub orbit. Wiper body (y < 0): clear.
Shift rail clear of the REEL gear at r 65.6.

### 4.4 Internal (new versus new)
After the check I swapped the wrap planes: segment + W pinion now -257..-264, G_k + W output -266..-273 (the first run had W
output in the drum's plane, 0.4 mm). Adjacent gear planes keep 2 mm axial gaps. Spool flange to drum base flange cone
1.9 mm: use a d62 flange (2.9 mm, 4.0 mm freeboard on 20 m) or accept. The shroud covers the lower half only under the
capstan (it came within 0.9 mm of the drum flange where it rose above the pack).

### 4.5 X2D bed (256 x 256 x 260) per part
All fit: rail drum about 100 x 100 x 84 (split A/B as today); head frame P1/P2 plates about 150 x 175 x 6; head side/spine
frame about 105 x 175 x 140; bridge plates about 140 x 230 x 6 (lay flat diagonal if needed), bridge arms about 95 long;
u-shaft C 238 long (lying, fits 256 with 9 mm each end; print it horizontal or use an 8 mm rod); u-shaft T 98; spool
64 dia; gears up to 76 dia (W output). The rail rear piece shortens (fork deleted). Nothing exceeds 245 mm.

## 5. Mass and aim

| item | g | rail (u, w) |
|---|---|---|
| reel head (capstan drum, tube, cup, K, gears, W, rail drum, N20, contacts, spool + line + shroud, S, B3, frames) | 582 | about (-232, 98) |
| spine (u-shafts C and T, bearings) | 35 | (-140, 85) |
| bridge (plates, countershafts, C1, sprag #2 + hub, bevels, c1', swing idler, T1, fork, bell crank, link, dog post) | 260 | (-45, 40) |
| removed (3 tether pulleys, line runs, fork of the rail rear) | -22 | (-170, 40) |

Pitching mass 1.49 kg today (my sum from asm.json, CG u +96, w 58) becomes 2.33 kg with CG at u -1, w 68: the head
balances the launcher. Gravity moment about the pivot (+ = nose down): today +1.30 / +0.64 / +0.34 N m at 6 / 36 / 47
deg; new -0.20 / -0.94 / -1.16 N m. Aim wiper torque (drivetrain.aim method with the real moment): peak 0.18 to 0.13
N m, mean 0.066 to 0.055 N m: the aim gets easier; the worm still holds (self-locking both ways). The falling stroke
and rail_low now need drive instead of being gravity assisted (included in the numbers). The physics model's ASSUMED
0.9 kg at 0.15 m (0.33 N m peak) stays conservative. Wiper: cock and tow loads unchanged (same ratios); the rail
inertia rises about 2.3x, irrelevant at the aim speed. Weak point: the bridge is a 60 to 90 mm cantilever off cheek A
carrying the REEL mesh force (about 100 N at the pin): make it stiff or use the coupler journal.

## 6. Launch resistance and range (base/physics, in-memory overrides, `phys_range.py`)

Spool d40 / d64 x 20 (PLA) on 2 x 608: I full 19.4 kg mm2, r full 27.4 mm, equivalent line mass 26.8 g (today 45 g).
Spin-up pull in the stroke 2.9 N (today 5.1 N), path gain 1.04 (today 1.35), guide at the eyelet (-185, 84.6), path taut.

| pitch | today 0.5 N | rail reel 0.1 N | 0.25 N | 0.5 N | v muzzle today / new |
|---|---|---|---|---|---|
| 6 | 2.55 m | 2.78 | 2.74 | 2.67 | 6.97 / 7.17 m/s |
| 25 | 3.92 | 4.67 | 4.48 | 4.20 | 6.88 / 7.09 |
| 34 | 4.19 | 5.20 | 4.93 | 4.51 | 6.85 / 7.05 |
| 40 | 4.19 | 5.35 | 5.02 | 4.52 | 6.83 / 7.03 |
| 47 | 4.02 | 5.29 | 4.91 | 4.34 | 6.81 / 7.01 |

Expected range about 5.2 to 5.35 m at 34 to 40 deg with the 0.1 N brake (+1.0 to 1.2 m), spool peak 2,460 rpm (608 fine).
Today's spool (d50/80 x 22) on the rail: 5.11 m at 34 deg (heavier). RISK: the free spool overruns after landing 3.4 m
at 34 deg and 5.6 m at 6 deg with 0.1 N (0.25 N: 1.25 m; 0.5 N: 0.5 m; today 0.55 m). Least-bad fix, no new actuator:
raise the brake track inside the LAUNCH zone (LIGHT 0.1 N at drum 0, 0.5 N by drum +5, collar still in LAUNCH, code still
100) and step the drum to +5 right after FIRED (0.6 s at 8.1 deg/s, flight 0.9 to 1.1 s at useful pitches; at pitches under
15 deg start at 0.5 N). Needs a firmware exception (drum move while the reset run finishes; the fork groove is flat there).
Fallback: LIGHT 0.25 N everywhere (4.9 m, 1.25 m overrun).

## 7. Change list by owner

coordinator (params.py, BASE_SPEC.md, DECISIONS.md)
- New decision D49 (rail reel, this layout); supersede D14, D34, D36 (guide = eyelet), D20 capstan placement, D25 c1 on the
  capstan tube, D43/D46 pulleys (exit eye and storage guide gone; path gain 1.18), D7/D24 LIGHT 0.1 N.
- LAYOUT: delete axes.capstan, reel_frame_R, keep_out.line; add RAIL_REEL block (all section 1 numbers), axes in the rail
  frame, keep_out for the head sweep and rover. Delete LAUNCH line_guide_u/w (or set -185 / 84.6).
- REEL: capstan_ratio 2.307, capstan gear 23T m1.5, cup gear 28T m1.5, path_gain 1.18, tow_tension_model (recalibrate),
  storage core 40 / flange 64 / width 20, clutch LIGHT 0.1 N (or 0.25), pulleys text, line_speed, take_up chain.
- DRUM: wrap_train idler_out 49T, cup_gear 28T, centre 57.75, idler_axis now rail frame; n20/drum axis in the rail frame;
  bus "meshes c1' 46T at cd 72.1 on the rail bridge"; zones unchanged; optional LAUNCH brake step (section 6).
- SWITCHES owners: SW_ARM, SW_DRUM, SW_DOG, SW_N20 now on the rail (wiring), SW_INDEX frame-fixed.
- ASSEMBLY_STAGES: drum and reel stages move after "launcher" (bridge and head bolt to the rail); new stage for the bridge.
- TIMELINE: no phase change; "flight" text (0.1 N), aim no longer moves line; remove pulley wording.

cad-reel (reel.py, reel_calc.py, reel_v7.py, reel_NOTES/REPORT, test_reel*.py)
- Rebuild as the rail head: frame R becomes a rail-frame transform (capstan along u at (y 140, w 110.6)); delete reel wall,
  sprag hub on the capstan, bus gear c1 on the tube, pulleys 1 and 2, swivel, exit post, guide eye, shroud geometry.
- reel_v7.cup: ring turned AXIAL (hole parallel to the cup axis, centre r 26.0 instead of 24.5 tangential, lug in front of
  the face, groove start moved to the ring's rear mouth). Drum unchanged. This is the one change to Harry's v7 cup.
- New: spool d40/64 x 20, shaft S, B3, u-shaft T, countershaft T, c1', swing idler, T1, B2 (bridge), u-shaft C, G_a, G_b,
  G_c, C1, B1, sprag #2 hub on countershaft C, P1/P2, spool shroud, clutch rocker driven by the rail drum cam.
- line_path(P, S): spool tangent -> eyelet -> launch.line_path from the pusher; tether_check states unchanged.
- reel_calc: clutch torques at r 27, tow and stall numbers (section 2), cup torque with 28T.

cad-drum (drum.py, _drum_kin.py, drum_table.json, drum_NOTES/REPORT, _drum_test.py, _drum_geom_test.py, _drum_views.py)
- Drum axis parallel to u at rail (y 52, w 150), u -182..-264, drum gear at the front end; segment plane -257..-264.
- Wrap idler W at (82.8, 118.6): 10T m1.25 pinion + 49T m1.5 output (was 42T).
- Fork: shift rail on the bridge (u -40, w -52), fork sleeve + yoke as today, new bell crank and link from a follower
  slider on the drum barrel; dog post and SW_DOG on the bridge; brake track drives the spool rocker (cad-reel lever).
- Main axis stack unchanged; SW_INDEX block re-mounted frame-fixed; delete bracket A/B, idler posts, frame drum rail.
- drum_table.json: zones and edges unchanged; add the LAUNCH brake step if adopted; check the wrap direction sign (the cup
  now faces +u: positive drum may need to stay "wrap" by mirroring the groove, else flip in firmware).

cad-launch (launch.py, launch_line.py, launch_kin.py, launch_guard.py, launch_numbers.py, launch_NOTES/REPORT, tests)
- Delete the rear pulley fork, top/bottom/pivot pulleys and line parts b/c (launch_line.py keeps only the pusher run, now
  from the eyelet u -185); rail rear end gets the head bolt flange at u -178.3; rail cheek A gets the bridge mounting.
- Trim guard r_a inboard wall for B1 (u -20..+20) and u-shaft T; check SW_HOME foot / SW_SEATED plunger stay where they are.
- line_entry/line_path hand-over moves to the eyelet; test_launch line asserts.

cad-frame (frame.py, _frame_*.py, frame_NOTES/INTEGRATION)
- Remove reel wall, pulley posts and drum bracket inserts from deck_contacts; cut back frame_guard_hood for the C1 and
  sprag hub orbit (pitch 5.5..47.5); motor plate notches for the old capstan/line go; SW_INDEX mount; cable flex loop
  to the rail; footprint note (rail tip to x -296 at 6 deg).

physics (common.py defaults, spec_bridge.py, payout.py, launcher.py, flight.py, capstan.py, retrieve.py, drivetrain.py,
mission.py, ballistics.json, FINDINGS/SUMMARY, test_phys_*.py)
- eye_gain 1.04, line guide (-185, 84.6), spool d40/64 x 20, brake 0.1 N: re-run ballistics (contract C3 table), payout
  overrun and the LAUNCH brake step; capstan ratio 2.307, path gain 1.18, tow law; retrieve seating gain (straight path,
  about 1.05); drivetrain aim with the real rail moment (replace the 0.9 kg / 0.15 m assumption, sign change); recoil
  check of the tow moment at the dead point; tests that pin 1.30, 1.35, 13.1/-2.3, 0.5 N.

fw-base (firmware/base: pins.h, drum table, sequences, PROTOCOL.md, CALIBRATION.md, tests) and jetson / mock / systest
- Tow tension constants (n_per_a, offset) and the 55 N cap check; capstan line speed for retrieve timeouts.
- Drum: optional LAUNCH brake step after FIRED (new exception to "wiper stopped before drum moves"); wrap direction sign.
- Remove nothing from the mission order (aim, cock, fire, rail low, NEUTRAL, wrap, anchor, tidy, REEL fly-in, tow, release,
  reel-in, unwrap, approach on FIRM, LAUNCH): it stays valid. Tidy and unwrap now back-drive the longer capstan train in
  its free sense (more drag than today: verify the FIRM take-up still pulls through; FIRM 5 N vs estimated 0.5 to 1 N drag).

tools / site (base/tools)
- mechanisms.py: storage, capstan, wrap, selector and tether texts and part patterns (bridge, head, u-shafts, bell crank;
  no pulleys); timeline.py unchanged keys; audit/meshcheck mesh list (new meshes: REEL-C1, bus-c1', bevels B1/B2/B3, W,
  G_a/G_b/G_c, G_k); sim show groups; steps/guide assembly order; BOM.

## 8. Risks (ranked)

1. Landing overrun with the 0.1 N free spool (3.4 to 5.6 m): needs the LAUNCH brake step or 0.25 N (section 6).
2. Fleet angle 15 to 17 deg at the eyelet: the take-up piles line in the middle of a 20 mm pack (D44 mounding, flange
   freeboard 4.6 mm at d64): bench-wind 20 m through the eyelet before the first shot; fallback 15 m of line or a d68
   flange (then u_s -228 and the shroud reaches the deck plane: pitch max 46 deg).
3. Tight clearances by design: cup 2.7 and spool 2.1 mm to the cocked carriage group, shroud 4.8 mm to the deck plane at
   47 deg, B1 1.9 mm to guard r_a: all measured on today's meshes; any growth of the carriage or the dart legs breaks them.
4. Harry's v7 cup needs the axial ring (owner's straight-through requirement is impossible with v7's tangential ring);
   wrap laying with the axial ring and the 60 deg ring-to-groove bend in tow need a bench test (D44 test still applies).
5. Bridge stiffness and mesh accuracy for C1 and c1' (cantilever 60 to 90 mm from cheek A).
6. Three bevel pairs and two printed u-shafts (wind-up 0.4 rad on u-shaft C unless steel); more free-sense drag in tidy
   and unwrap.
7. Rear overhang: the rail tip now reaches world x -296 at 6 deg and z -151..-161 at 47 deg behind the deck: rover check.
8. 10-wire flex loop across the pivot; the N20 and drum now ride the rail (shock at every shot: secure the N20 strap).
