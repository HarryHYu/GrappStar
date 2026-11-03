# Physics findings for the spec (worker `physics`)

Status: updated 3 Nov (after review round 1 and cad-launch NOTES), run on base/spec/params.py v0.1-stub (LAUNCH, AIM, REEL still None, so
launcher/reel geometry below is my placeholder: stroke along the rail, 60 g carriage, 1:1 springs to carriage, reel
core 40 / flange 64 / width 20 mm, capstan 40 mm). Dart mass 0.279 kg (spec DART.mass_kg; the brief said 0.292 kg:
coordinator please confirm which). Models: base/physics/*.py, every number reproducible with the CLIs.

## Current numbers (spec v1.0, latest run; details in SUMMARY.md, base/out/physics/*.json)

- Launch: 2 springs, 35 mm preload, 280 mm full, 13.1 J, carriage 60 g: 7.8 m/s, 22 g, 65 % efficient.
- Range from the rail pivot's ground point (pivot 0.315 m up: deck 0.15 ASSUMED + 0.165), LIGHT 0.5 / 0.75 / 1.0 N:
  5.2 / 4.8 / 4.3 m at 36 deg (best). Table 6 to 47 deg in base/out/ballistics.json (C3): 2.9 m at 6 deg.
  4 springs (provision): 10.6 m/s, 9.0 / 8.2 / 7.5 m; stop KE 7.8 J.
- Stop: 2.8 J (carriage 1.8 + springs 1.0): 15 mm linear pad 370 N; for 300 N use about 19 mm or a constant-force stop.
- Storage reel 50/66/22: 43 g, 27.6 kg mm2 full; the spin-up adds 5.0 N of line pull in the stroke (the bigger reel
  costs 0.36 m/s vs no reel inertia, about the same for a 40 mm core); overrun after landing 0.30 m at 0.5 N.
- Capstan 50:22 (2.27x faster), 7 wraps: tow 50 N = 3.0 N m, 4.8 A, 0.15 m/s; 20 m reel-in 107 s; limiter slipping
  pulls 88 N (current 7 A). 7 wraps hold far more than any limit: the trips protect the dart, not the capstan.
- Cock 3.1 N m, 4.9 A, 2.7 s per shot. Aim 0.32 N m, 6.6 s for 6 to 47 deg.
- Recoil rock (ASSUMED 4 kg rover): 5.8 deg at 6 deg pitch braked (1.2 rolling), 0.7 deg at 36. No tip.
- Clutch convention (both LIGHT and FIRM): line tension with the reel FULL; it rises as the reel empties
  (x1.23 at the core). Final approach on FIRM 3 N: 2.3 N at the dart vs 2.1 to 2.6 N needed (margin 0.9 to 1.1):
  marginal. FIRM 5 N gives a 1.5 margin, or seat on the capstan.
- Wrap: drum 0.49 to 0.60 N m, N20 61 to 74 N mm.
- Mission (mission.py): aim, cock, flight, anchor, wrap, tow 3 m at 30 N, release, unwrap, reel 5 m: about 3.2 min,
  0.66 Wh from the base pack (about 70 missions per 3S 5200 mAh, base side only). Drum N20 timings ASSUMED.
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
