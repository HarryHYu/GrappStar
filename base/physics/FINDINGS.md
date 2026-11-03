# Physics findings for the spec (worker `physics`)

Status: updated through the day; the Current numbers block is the latest. Sections F1 onward are the history of
the findings (their numbers come from earlier parameter sets). Every number is reproducible with the CLIs.

## Current numbers (spec 1.2.7: D36 rear line guide, carriage 70 g; model with the review fixes)

- Launch: 2 x Century C-243, 23.8 mm preload, 265 mm full (40.1 N each), 12.1 J, carriage 70 g: 6.95 m/s at 35 deg,
  19 g, 56 % efficient. Line path taut (D36 guide at u -200, w 85: the path only lengthens), no snatch.
- Range from the rail pivot's ground point (pivot 0.315 m up: deck 0.15 ASSUMED + 0.165), LIGHT 0.5 / 0.75 / 1.0 N:
  4.29 / 3.86 / 3.48 m at 34 deg (best; the line now leaves from the D36 guide in flight too). Table 6 to 47 deg in base/out/ballistics.json (C3), content hash, impact
  angles and wind sensitivity in the meta. Test shots then `calib.py range` refine it.
- 4 springs: NOT SUPPORTED IN v1 (D38: cocking needs 5.8 N m, above the shear pin). For reference 7.6 / 6.8 / 6.1 m.
- Stop: 2.5 J (carriage 1.7 + springs 0.8): the 19 mm flexure peaks at about 266 N if linear (D26 300 N: OK).
- Storage reel 50 / 72 / 22: 46 g, 31.4 kg mm2 full; overrun after landing at LIGHT 0.5 N about 0.55 m.
- Capstan 50:22, 7 wraps: tow 50 N = 2.83 N m, 4.62 A, 0.15 m/s. Shear pin slipping: about 92 N, 7 A.
  TAKE-UP DRAG CORRECTED (review round 6): the bus turns the clutch disk at 2.5 x the WIPER (D25), so the FIRM drag is
  0.43 N m at the wiper, not 0.82. Tow sensing: line = 17.0 x (I - I_free) - 3.0 N (was -10.4); tow trip at free
  + 3.05 A. Tow hard trip 6.5 A (D33) = about 83 N. Highest tow limit before the trip / 0.8 x shear pin: 72 N (D39 cap
  70 N: OK).
- Cock 2.9 N m, 4.7 A, 2.7 s per shot. Aim 0.33 N m peak, 1.9 A, 6.6 s for 6 to 47 deg.
- AIM GEAR (review R2-K, cad-launch as built): splitter sun m1.5 z30 face 8. At the aim peak sf 5.9 (PPA-CF). At a
  5 A trip (3.2 N m) it fails Lewis (sf 0.62 PPA-CF, 0.30 PLA). Recommend the aim hard trip at 3 A (1.36 N m,
  sf 1.44 in PPA-CF; print the splitter and bevels in PPA-CF, PLA fails even at 3 A).
- Final approach on FIRM 5 N (reel full): 3.85 N at the dart vs 2.1 to 2.6 N needed: margin 1.5 to 1.9.
- Wrap on LIGHT (DRUM.n20 12:1, 8.1 deg/s): N20 46 to 56 N mm (23 to 28 % of 0.20 N m); about 30 s.
- Take-up in NEUTRAL: 0.06 / 0.12 / 0.26 to 0.33 m/s at duty 0.3 / 0.5 / 1.0 (retrieve.json "takeup").
- Recoil rock (ASSUMED 4 kg rover): about 5 deg at 6 deg pitch braked, 0.5 deg at 36. No tip.
- Mission: about 3.6 min, 0.75 Wh from the base pack (about 62 missions per pack).
- SPRING CATALOGUE (web check): Bunnings 0487961 is Century C-243, 5/8 in OD x 6-1/2 in x 0.054 in wire (1.37 mm,
  not 1.4). Century lists max safe load 8.64 lb (38.4 N) and max deflection 11.9 in (302 mm), rated for about 100,000
  cycles. The earlier 280 mm full cock was 42 N with the 0.125 N/mm / 7 N estimate: inside the deflection limit but 9 %
  over the load rating (spec v1.2 now uses 265 mm, 40 N) (our A228 static limit with SF 1.2 is 43.6 N at 1.4 mm wire, about 41 N at 1.37 mm). For a few
  hundred shots this is acceptable; the hang test decides: keep the force at full cock at or under about 40 N
  (if the spring measures 0.125 N/mm and 7 N: about 265 mm). Each 10 mm less extension costs about 0.15 m of range.
- Tow current sensing: 17 N of line per amp above free current; the offset is the FIRM take-up drag (10.4 N at
  FIRM 5 N, see Current numbers). A firmware constant of 8 N/A would let the line reach about twice the set limit.
- Blind cross-check (fresh agent, no access to this code): muzzle speed 7.77 m/s (ours 7.78 on the same inputs, spec v1.0). Its range was 12 %
  longer; traced to a taut/slack chatter in its reel logic that applied the brake only about half the time. With the
  brake applied continuously its approach matches ours within 1 %.
- Sections F1 to F11 below are the history (numbers from earlier parameter sets).

## F1. RANGE: the two Century springs cannot reach 15 to 20 m. Expect 4 to 5 m. (biggest issue)

| Case (pitch 35 deg, tether brake 0.5 / 0.75 / 1.0 N) | spring ext | stored | v muzzle | range m |
|---|---|---|---|---|
| cad-launch NOTES: 250 mm, 1:1, preload 5 mm, carriage 80 g (current baseline) | 250 mm | 11.2 J | 6.9 m/s | 4.0 / 3.6 / 3.2 |
| spec max_ext 250 mm, 1:1, carriage 60 g | 250 mm | 11.2 J | 7.1 m/s | 4.2 / 3.8 / 3.4 |
| ext 280 mm (still under the wire limit, SF 1.25) | 280 | 13.6 | 7.9 | 5.0 / 4.5 / 4.1 |
| 280 mm + carriage 30 g | 280 | 13.6 | 8.2 | 5.5 / 4.9 / 4.4 |
| 280 mm + 30 g + 2:1 pulley (540 mm stroke) | 280 | 13.6 | 8.0 | 5.2 / 4.6 / 4.2 |

- Same 8.6 m/s shot with no tether: 7.6 m; in vacuum 7.8 m. The tether (brake + line weight and drag) costs ~30 %.
- Muzzle speed needed (best pitch, 0.75 N brake): 10 m needs 12.0 m/s (20 J in the dart, ~32 J stored at 64 %),
  15 m needs 15.0 m/s (31 J, ~49 J stored), 20 m needs 17.6 m/s (43 J, ~67 J stored). Two of these springs store at
  most ~16 J at their wire limit.
- Ask: (a) set the mission range to 4 to 5 m with these springs, or (b) Harry finds more/stronger springs (energy scales
  with spring count: 4 of these springs at 280 mm store ~27 J, roughly 8 to 9 m). Spec should say which.
- Pulley ratio does NOT add energy (it trades force for stroke). 1:1 is best on a short rail; 2:1 only helps the stop
  and spring surge, and loses to the extra friction of the doubled stroke.

## F2. Spring limits (ASTM A228, Shigley 10-7, extra SF 1.2)

- Formula rate 0.131 N/mm (96 body coils, 165.1 mm taken over the hooks; the 0.125 catalogue estimate is used). Mass 54 g each.
- Safe force 43.6 N, governed by hook torsion (r2 = 2.5 d assumed), body torsion 51.6 N. Safe extension 289 mm with
  F0 7.5 N (293 mm with spec F0 7.0). Without the 1.2 factor: ~356 mm. Spec max_ext 250 is conservative; 280 is fine.
- Initial tension 7 N is at the top of the Shigley preferred band for this index (C 10.4): the hang test will likely
  find 5 to 7 N.
- Surge 25 Hz fixed-fixed (one-way wave 21 ms vs a 57 ms stroke): the lumped 1/3-mass model and the chain model agree
  within 1 %. But after the carriage hits the stop the coils keep moving and clash (chain model: 8 to 14 m/s coil
  closing speed). More preload (40 mm instead of 10) cuts clash to 7.7 m/s and keeps 96 % of the speed.

## F3. Carriage stop

- Carriage KE at the stop 1.0 J (30 g) to 1.9 J (60 g), plus ~1.2 J of spring KE arriving through the cord.
  With a 4 mm pad the peak is ~1.2 kN (carriage + spring KE; the cord alone sees ~310 N after a rigid stop). Make the stop pad 15 to 20 mm of foam/TPU (peak ~150 to 250 N), and the
  carriage as light as possible (every 10 g is ~0.15 m/s).

## F4. Pitch range (cad-launch crank-rocker 6 to 47 deg: good, keep it)

- Baseline: best 36 deg (3.6 m at 0.75 N); 6 deg gives 2.2 m, 47 deg 3.3 m. The table spans 2.2 to 3.6 m, so every
  target from 2.2 to 3.6 m is reachable on the rising branch. Older note below was for an 8.6 m/s shot.

- Best range at 35 deg (0.75 N). 30 to 40 deg are within 2 % of best. Below 20 deg range drops fast (the dart lands at
  4.6 m from 8.6 m/s). An aim range of 15 to 50 deg covers 75 % to 100 % of max range; above 45 deg buys nothing but
  apex (lob over obstacles: apex 1.4 m at 35, 2.1 m at 50).

## F5. Reel brake, payout, bird's nest

- Slip-clutch torque set for 0.75 N at the full reel (r 27.6 mm) gives 1.0 N at the core (r 20 mm): the brake gets
  37 % harder as line pays out. Fine for 5 m shots (little line leaves).
- Reel at ~3,000 rpm at 8.6 m/s: 608s are fine. Overrun after landing ~0.24 m (reel stops in ~0.1 s): bird's nest risk
  low, centrifugal shoes not needed at these speeds. Reel inertia (19 kg mm2 full, two thirds of it the 64 mm flanges) adds ~26 g of
  effective mass to the dart during the stroke (line pull ~4 N in the stroke). Spoked or thinner flanges help.
- Range vs brake at 36 deg (coupled reel): 0.2 N 4.7 m (overrun 1.5 m), 0.3 N 4.5 m (0.9 m), 0.4 N 4.3 m (0.6 m),
  0.5 N 4.1 m (0.4 m), 0.75 N 3.7 m (0.2 m). Recommend the payout clutch at 0.4 to 0.5 N: under 0.6 m of loose line
  at the reel after landing, the take-up clears it at the start of reel-in. Below 0.3 N the overrun grows fast.

## F6. Wiper stall is the dangerous case (drivetrain.py, capstan.py)

- Stall 14.1 N m / 17 A at 12.6 V (with sag). Nothing mechanical limits it:
  - 20 mm keyed PPA-CF shaft allowable 12 N m < stall (and only ~6 N m if the keyseat is a sharp end-milled slot,
    Kt ~3: round the keyway corners). With a steel 6 mm key 14 mm long, the plastic shearing behind the shaft keyway
    governs at about 7 N m. So the "12 N m keyed shaft" figure in the plan is optimistic: either a longer key engagement
    (30 mm gives ~15 N m) or keep every keyed joint under ~5 N m with the trip below. Every printed gear (as assumed: m 2, 20 T, 12 mm) fails Lewis
    at stall (SF 0.27). On the capstan, stall pulls ~600 N on the line (break 667 N; dart holds 7 to 112 N).
  - The 20 A fuse holds a 17 A stall indefinitely. Firmware current trip (request to fw-base and the spec):
    ignore current for 50 ms after a start (inrush is near stall); hard trip at 5 A for 30 ms for aim, reel, tow (about
    3.2 N m at the wiper) and 6.5 A while cocking (cock peak 4.6 A); tow trip at measured free current + 1.3 A (50 N),
    averaged over 200 ms. The BTS7960 IS pin senses the high side only: sample in the PWM on-time or scale by duty.
    Current is a coarse tension sensor (1.3 A over free): the load cell is worth fitting.
  - After a trip, sprag #2 (capstan backstop) and the self-locking worm keep the stall tension locked in. So add a
    printed slip clutch of about 3 N m on the REEL output gear as well (reviewer recommendation, I agree).
  - Or: a printed slip clutch on the wiper output set ~5 N m (protects every train mechanically).
- Cock through the cad-launch train (big gear m 2.5 40 T on the rack, drive 26 T): 2.8 N m peak at the wiper, 4.6 A,
  2.0 s; so the hard trip while cocking must be higher (6.5 A, 4.5 N m). At wiper stall the big gear sees 19.5 N m:
  do not put it on a keyed 20 mm printed shaft (it rides on its pin: fine). Other loads: tow 50 N = 1.2 N m, aim 0.2 N m. All under
  13 % of stall. The wiper is far stronger than needed: use the margin for speed.

- Aim through the cad-launch train (2:1 bevel, worm 4.8 deg lead, 20 T wheel = crank, rail ~1.26 kg pitching incl.
  dart, CG 150 mm assumed): wiper 0.08 N m peak (worm efficiency 0.23, self-locking), 6.5 s for 6 to 47 deg. But at
  the 5 A trip the splitter/bevel stage sees 3.2 N m: module 2 face 10 passes (sf ~1.3), module 1.5 does not. At wiper
  stall the crank sees ~31 N m (8 mm axle, conrod pins): only the trip protects it.

## F7. Capstan and tow speed

- Wraps for 50 N at mu 0.15 with the 3 N REEL take-up (constant clutch torque, lowest at the full reel): 3.3 raw, 5 design. Five wraps hold 223 N (mu 0.15) with 2 N take-up, so
  the capstan itself never limits force: the firmware must.
- Tow at 1:1 on a 40 mm capstan: 0.077 m/s; reeling 20 m takes ~4 min. A 3:1 step-up to the capstan gives 0.23 m/s and
  still only ~24 % of stall at 50 N. Recommend capstan speed ratio 2.5 to 3 (and the same for the take-up overdrive).

## F8. Smaller notes

- Cock force at 280 mm: 84 N on the carriage (rack/big gear tooth load); sprag #1 (on the wiper axis) sees ~3.1 N m of 50: huge margin.
- Dart landing: 6.2 m/s at ~54 deg down (5.4 J). The dart's own landing check (kedge physics) used 13.3 m/s (25 J,
  ~300 g in 3 cm of soil); at 6.2 m/s it is ~65 g: the dart parts have a large margin.
- DRV8833 for the drum N20 needs the buck (VM max 10.8 V), as the workplan says.

## F9. Dart attitude at 6 to 8 m/s (attitude.py, 3-DOF pitch plane, same physics as kedge/flight_sim.py)

- With the tether at 0.5 to 1 N: nose first in all 324 cases (pitch 10 to 47, +-10 deg and +-5 rad/s disturbances),
  with or without the 15 g ballast. Nose-first body angle at impact -17 to -80 deg at 36 deg pitch.
- Without the tether it tumbles or lands tail first: at this speed the tether, not the ballast, does the stabilising.
  So never test-fire without the line on (a dry shot can land tail first).

## F10. After coordinator decisions D5 to D10 (preload 35, stroke 245, 280 mm full, brake 0.5 N, 5 N m limiter)

- 2 springs: 13.1 J, 7.6 m/s at 36 deg, range 4.9 / 4.5 / 4.0 m at 0.5 / 0.75 / 1.0 N (coupled reel model).
- 4 springs (second pair): 26.2 J, 10.4 m/s, 43 g peak, carriage force 168 N, range 8.7 / 7.9 / 7.2 m.
  Stop KE 8.5 J (vs 3.3 J): the D6 stop must be sized for it if the second pair is fitted.
- D6 stop: carriage + spring surge KE at the stop is 3.3 J (2 springs). A linear pad of 15 mm peaks at ~430 N; for
  a 300 N peak a linear pad needs 22 mm, or 15 mm of a near-constant-force element (~220 N, e.g. a crush or friction
  stop, or a pre-loaded spring).
- Recoil at the shot: the frame feels 84 N peak backward along the rail at the spring line for ~55 ms (dart impulse
  2.1 N s). The carriage stop ~4 ms later hands the carriage + spring momentum back on the same line, so the net
  angular impulse is small. With an ASSUMED rover (4 kg, pivot 0.30 m above ground, wheels 0.15 m behind and ahead of
  the pivot, CG 0.12 m high): wheels braked, the rover rocks back ~5 deg at 6 deg pitch, 2.5 deg at 20, 0.5 deg at 36,
  0 at 47; wheels free to roll, under 1 deg (it rolls back a little instead). Nothing tips. Low shots are the least
  accurate; rover mass, wheelbase and CG still needed (recoil.py ROVER block). (Corrected in review round 3: the first
  estimate of 17 deg ignored the stop pulse.)
- With the 5 N m limiter the stall cases in drivetrain/capstan are evaluated at the limiter torque (motor current
  ~7 A while it slips, caught by the 5 A trip).

## F11. Take-up bus, wrap and final approach (coordinator changes, 3 Nov; retrieve.py)

- Taut line at the shot (take-up bus) vs 245 mm of slack: taut gives 7.61 m/s. With slack the dart leaves at 7.95 m/s
  but the snatch shares its momentum with the reel (26 g effective x eye gain): 7.19 m/s after it, 5.5 % slower than
  taut (0.85 J lost), plus a line spike up to ~240 N (undamped bound) yanking the dart tail. Taut is right.
- Wrap: 7 cup turns in 240 deg of drum. Line drawn off the reel against the LIGHT clutch (0.5 N at the full reel),
  through ~180 deg of guides (mu 0.25): ring pull 1.1 to 1.5 N, cup 33 to 45 N mm, drum 0.49 to 0.67 N m, drum N20
  61 to 83 N mm (30 to 41 % of an ASSUMED 0.2 N m usable N20 worm torque at 9 V: MEASURE the N20). About 18 s per wrap
  at 20 rpm. 0.9 m of line is drawn.
- Final approach onto the cradle on the FIRM clutch alone (3 N at the core = 2.2 N at the full reel, 1.7 N at the dart
  after the exit eye and guides): NOT enough. Need 2.1 to 2.6 N at 5 to 10 deg pitch (gravity + mu 0.3 to 0.4 on the
  cradle + ~1 N to close SW_SEATED). Fix: FIRM about 7 N at the core (1.5 margin), or seat on the capstan (REEL with
  wraps) and use the FIRM clutch only for tidy, or lower the seat contact force to ~0.3 N.

## F12. Adversarial review of the assumptions (ranked by effect on range; 3 Nov afternoon)

1. Carriage mass: cad-launch's carriage_solid is 138.7 cm3 (172 g solid PLA, 100 to 130 g printed) vs the 60 g
   target. At 100 g: about 0.4 m less range and a stop peak of about 350 N (over D26 300 N). Pocket it / slicer mass.
2. Line slack at the shot: from the pivot pulley (D14) to the dart tail the path shortens by about 98 mm during the
   stroke (183 -> 85 -> 116 mm), faster than the take-up bus can follow, so the line snatches after the muzzle:
   about 7 % slower, about 0.5 m less range, plus a tail jerk. Fix: route the line from a pulley behind the rail end
   on the dart axis so the path only lengthens. (Sent to the coordinator.)
3. Line path friction: now modelled as eye gain 1.35 (eyes ~2 rad at mu 0.15, pulleys, the 47 deg pusher-hole bend)
   instead of 1.15. Static vs kinetic clutch friction can move the brake about 30 %: calibrate with test shots.
4. Spring initial tension 5 N instead of 7 N: about 0.35 m less (hang test).
5. Carriage shoe friction from the pusher / spring-line couple: now modelled (about 1 J at full draw, upper bound).
6. Rover rock during the stroke (pitching ~1.8 rad/s at release with a braked 4 kg rover): -0.1 to -0.3 m at 36 deg.
   Measure with slow-motion video.
7. Tow sensing depends on the assumed wiper constants and capstan efficiency: calibrate line pull vs current with a
   luggage scale at three loads before trusting the 50 N trip.
8. Terrain: the dart descends at about 54 deg, so ground 0.2 m higher lands about 0.15 m short. ballistics_meta now
   lists the impact angle per pitch for the Jetson's height correction.
Small (checked, under 0.1 m): rack release friction, tether drag, 1.5x dart drag, 3 m/s wind, 1 deg aim error above
15 deg, battery sag while cocking, anchor creep.
Combined realistic case with items 1, 2, 3 and 5 unfixed: about 3.0 to 3.4 m. Items 3 and 5 are now in the model;
1 and 2 are design fixes; the test-shot calibration (calib.py range) absorbs the rest.
