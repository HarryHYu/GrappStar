# Physics findings for the spec (worker `physics`)

Status: first pass, 3 Nov ~11:15 Sydney, run on base/spec/params.py v0.1-stub (LAUNCH, AIM, REEL still None, so
launcher/reel geometry below is my placeholder: stroke along the rail, 60 g carriage, 1:1 springs to carriage, reel
core 40 / flange 64 / width 20 mm, capstan 40 mm). Dart mass 0.279 kg (spec DART.mass_kg; the brief said 0.292 kg:
coordinator please confirm which). Models: base/physics/*.py, every number reproducible with the CLIs.

## F1. RANGE: the two Century springs cannot reach 15 to 20 m. Expect 4 to 5 m. (biggest issue)

| Case (pitch 35 deg, tether brake 0.5 / 0.75 / 1.0 N) | spring ext | stored | v muzzle | range m |
|---|---|---|---|---|
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

- Formula rate 0.128 N/mm (98 body coils, consistent with the 0.125 catalogue estimate). Mass 55 g each.
- Safe force 43.6 N, governed by hook torsion (r2 = 2.5 d assumed), body torsion 51.6 N. Safe extension 289 mm with
  F0 7.5 N (293 mm with spec F0 7.0). Without the 1.2 factor: ~356 mm. Spec max_ext 250 is conservative; 280 is fine.
- Initial tension 7 N is at the top of the Shigley preferred band for this index (C 10.4): the hang test will likely
  find 5 to 7 N.
- Surge 24 Hz fixed-fixed (one-way wave 21 ms vs a 57 ms stroke): the lumped 1/3-mass model and the chain model agree
  within 1 %. But after the carriage hits the stop the coils keep moving and clash (chain model: 8 to 14 m/s coil
  closing speed). More preload (40 mm instead of 10) cuts clash to 7.7 m/s and keeps 96 % of the speed.

## F3. Carriage stop

- Carriage KE at the stop 1.0 J (30 g) to 1.9 J (60 g), plus ~1.2 J of spring KE arriving through the cord.
  With a 4 mm pad the peak is ~1.2 kN (carriage + spring KE; the cord alone sees ~310 N after a rigid stop). Make the stop pad 15 to 20 mm of foam/TPU (peak ~150 to 250 N), and the
  carriage as light as possible (every 10 g is ~0.15 m/s).

## F4. Pitch range

- Best range at 35 deg (0.75 N). 30 to 40 deg are within 2 % of best. Below 20 deg range drops fast (the dart lands at
  4.6 m from 8.6 m/s). An aim range of 15 to 50 deg covers 75 % to 100 % of max range; above 45 deg buys nothing but
  apex (lob over obstacles: apex 1.4 m at 35, 2.1 m at 50).

## F5. Reel brake, payout, bird's nest

- Slip-clutch torque set for 0.75 N at the full reel (r 27.6 mm) gives 1.0 N at the core (r 20 mm): the brake gets
  37 % harder as line pays out. Fine for 5 m shots (little line leaves).
- Reel at ~3,000 rpm at 8.6 m/s: 608s are fine. Overrun after landing ~0.24 m (reel stops in ~0.1 s): bird's nest risk
  low, centrifugal shoes not needed at these speeds. Reel inertia (19 kg mm2 full, two thirds of it the 64 mm flanges) adds ~26 g of
  effective mass to the dart during the stroke (line pull ~4 N in the stroke). Spoked or thinner flanges help.
- Range falls ~0.45 m per +0.25 N of brake. Set the light brake as low as reliably repeatable (0.5 N).

## F6. Wiper stall is the dangerous case (drivetrain.py, capstan.py)

- Stall 14.1 N m / 17 A at 12.6 V (with sag). Nothing mechanical limits it:
  - 20 mm keyed PPA-CF shaft allowable 12 N m < stall (and only ~6 N m if the keyseat is a sharp end-milled slot,
    Kt ~3: round the keyway corners). With a steel 6 mm key 14 mm long, the plastic shearing behind the shaft keyway
    governs at about 7 N m. So the "12 N m keyed shaft" figure in the plan is optimistic: either a longer key engagement
    (30 mm gives ~15 N m) or keep every keyed joint under ~5 N m with the trip below. Every printed gear (as assumed: m 2, 20 T, 12 mm) fails Lewis
    at stall (SF 0.27). On the capstan, stall pulls ~600 N on the line (break 667 N; dart holds 7 to 112 N).
  - The 20 A fuse holds a 17 A stall indefinitely. Firmware current trip (request to fw-base and the spec):
    ignore current for 50 ms after a start (inrush is near stall); hard trip at 5 A for 30 ms in every mode (about
    3.2 N m at the wiper; cock peak is 2.9 A so no false trips); tow trip at measured free current + 1.3 A (50 N),
    averaged over 200 ms. The BTS7960 IS pin senses the high side only: sample in the PWM on-time or scale by duty.
    Current is a coarse tension sensor (1.3 A over free): the load cell is worth fitting.
  - After a trip, sprag #2 (capstan backstop) and the self-locking worm keep the stall tension locked in. So add a
    printed slip clutch of about 3 N m on the REEL output gear as well (reviewer recommendation, I agree).
  - Or: a printed slip clutch on the wiper output set ~5 N m (protects every train mechanically).
- Normal loads are tiny: cock 1.5 N m peak (15 mm winch drum equivalent), tow 50 N = 1.2 N m, aim 0.2 N m. All under
  13 % of stall. The wiper is far stronger than needed: use the margin for speed.

## F7. Capstan and tow speed

- Wraps for 50 N at mu 0.15 with 2 N take-up: 3.4; design 5 to 6. Five wraps hold 223 N (mu 0.15) with 2 N take-up, so
  the capstan itself never limits force: the firmware must.
- Tow at 1:1 on a 40 mm capstan: 0.077 m/s; reeling 20 m takes ~4 min. A 3:1 step-up to the capstan gives 0.23 m/s and
  still only ~24 % of stall at 50 N. Recommend capstan speed ratio 2.5 to 3 (and the same for the take-up overdrive).

## F8. Smaller notes

- Cock force at 280 mm: 84 N on the carriage (rack/big gear tooth load); sprag #1 sees ~1.5 N m: huge margin.
- DRV8833 for the drum N20 needs the buck (VM max 10.8 V), as the workplan says.
