# GrappStar reel: context handover (2 Nov 2026)

This chat is the reel only. Dart, launcher and vision live in their own chats (see kedge-full-handover-CLAUDE.md for the dart).

## What the reel is

Line path: storage spool -> guide eye -> over the capstan drum -> groove on the wrap cup -> ring on the cup -> exit eye -> dart.

Phases:
1. Launch: cup pinned, spool free. The dart pulls line off the spool.
2. Retrieve: spool crank (later the fast motor) winds the slack.
3. Wrap: pin out, turn the cup by hand (later an N20). The ring carries the line round; each turn lays one turn on the capstan behind the cup. Dart side sweeps in front of the cup face, never wraps.
4. Tow: pin in, cup stands still, wiper turns the capstan inside it. Turns live between the cup rim and the base flange. Tail runs to the spool.
5. Unwrap: pin out, cup back, spool takes up.

Design rules found the hard way:
- The cup must go OVER the drum (skirt covers the gap). A flat disc beside the drum leaves a crack the line finds.
- The ring on the cup must sit 0.5 mm in front of the face plane (any lower and the dart line cuts through the face when the cup turns) and 1 mm outside the wall (so the line from the guide reaches the rear mouth without grazing the edge).
- Exit eye must be well past the ring (now 28 mm) so the dart line clears it while the cup turns. The arm between ring and exit eye must be zero length: a long lug just defeats moving the exit eye out.
- Guide eye and spool sit 21 mm behind the capstan base flange, so the tail leaves the drum at an angle and the turns walk toward the base.
- Every visible hole is plain round. D flats that take torque are on the axles and hidden: drum bore, rear half of the spool bore, inside the crank lip. Eyes are dia 4.4 holes coned out to 6.4 both faces. No rings with lips, no flares breaking out of posts.
- Groove on the cup: U section, 1.2 across the line, 1.0 deep, 30 deg helix, 58 deg of wrap, funnelled both ends, cut LAST in the CadQuery build (cutting it before the hole makes an invalid solid).

## Current state: full size v7 (printed set, 21 Sep geometry, delivered 1 Nov)

Zip: grappstar_reel_full_v7.zip in this chat's outputs (full_v7.py generator + README + 9 STLs). Nothing printed yet.

Sized for 20 m of 1 mm Dyneema (arrived). Dia 40 x 40 capstan (8 Nm stall = 400 N), spool dia 52 with dia 28 barrel (holds ~25 m), 8 mm D axles. Assembled 203 x 152 x 80 mm. Open frame plate (10 mm border, spars under the capstan and guide lines, 8 mm pads at every foot), 131 cm3.

Eleven pieces from 9 STLs, all pushes, no screws, no glue:
- frame x1 (~90 g), drum x1 (~35 g), spool x1 (~25 g), wrap_cup x1 (~7 g), crank x2 (~5 g), axle_capstan x1, axle_spool x1 (~4 g), cap x2 (C-clips), lock_pin x1. ~175 g total. Print axles and clips solid.

Snaps and fits (all verified on the meshes, zero intersections, every stack checked):
- Axles: 8.0 D rod (flat plane 2.7 from axis), 7.6 neck + chamfered 8.2 head at the crank end; 4.5 neck + 5.6 head at the clip end.
- Cranks: round bore, 7.9 lip carrying the D flat inside four slotted leaves 2.25 thick x 10.5 long, 0.15 interference, ~40 N to click on, the head shoulder stops them coming off.
- C-clips: dia 11 x 2, 4.6 bore, 3.8 opening, on the 4.5 necks. Cup and spool end play 0.3 + 0.3.
- Keyed 8.4 D bores (drum, rear half of spool); 8.5 round in frame, cranks, front of spool; 8.6 round in the cup. Cup over drum 0.6 radial.
- Lock pin 4.15 in 4.3 holes, snug, through the lock post into a boss on the cup.
- Cup sweep clears drum 0.6, lock post 2.0, clip 0.3. Dart line clears cup and frame at every cup angle.

Print: PLA, 0.2 mm, 4 walls, 30% infill, no supports, orientations in the README (frame flat, drum flange down, cup open end down, spool front flange down, cranks flat hub up, clips flat, pin head down, axles on their flat).

Version trail this session: mini v14 -> v18 (lug, rails, helical channel), then full v1 (scaled up, U groove, fillets) -> v2 (ring guides, rejected) -> v3 (plain eyes, softer snap leaves, C-clip, spool forward) -> v4 (lug replaced by pad, round holes, jagged mesh defects fixed) -> v5 (pad replaced by ring) -> v6 (guide + spool moved 21 mm behind the drum) -> v7 (plate windowed into a frame).

Live artifacts (same links, updated each round):
- reel_sim.html, 6 phases with the v7 meshes: https://claude.ai/artifact/6utwke6YQpZ87q736L18o4
- reel_full_v7_3d.html, exploded viewer: https://claude.ai/artifact/1RH9QYQ44qrLQCx2XifazA

## Open problem: payout at launch

Harry: "when the line flies off, there is way too much resistance on the reel for the line to come out slowly."

Physics (checked numerically): it is inertia and overrun, not eye friction. The keyed spool + axle + crank has to go 0 to ~4000 rpm in the 50 ms barrel time (~8 N on the line at 12 m/s, ~30 N at 25 m/s), then keeps spinning after the dart slows and throws 4 to 5 m of loose line round the reel (bird's nest). Bearings alone do not fix it: a free spool starts easier but overruns worse. At 25 m/s the dart reaches the end of 20 m before landing and hits ~600 N.

Options given, in order of payoff:
1. End pull (stationary pack): spool pivoted to point at the guide, line slips off over the front lip like a spinning reel or a line thrower. Nothing rotates, ~0.3 N, zero overrun. Needs the spool to turn 90 deg between launch and retrieve, or a second winding axis. Recommended.
2. Flake tray: first 5 m laid loose between spool and guide; cheap test with the current print.
3. If it must spin: 608 bearings (8 x 22 x 7, drop-in on the 8 mm axle), spool free on the axle at launch with a dog for winding, lighter wider spool, motor-fed payout off the AS5600, centrifugal overrun brake (helps the current spool, cuts range ~2 m).

Payout sim built, NOT yet published (Harry declined the publish, then asked for this handover): /mnt/user-data/outputs/payout_sim.html, generator /home/claude/build_payout.py. Three modes (current spool / free spool / end pull), overrun brake toggle, launch speed, spool weight, readouts for rpm, pull, slack thrown, side view plot, loose loops drawn round the spool when it overruns. Headless-tested, screenshots looked right.

## Still to do

- Decide the payout fix (end pull most likely), redesign the spool mount accordingly, then print v7 or v8.
- Measure wiper stall torque and shaft: drum gets a motor hub in place of the rear crank, frame grows a motor mount. Spool crank becomes the fast motor. N20 on the cup if hand wrapping proves the mechanism.
- Exit eye carries the full tow load over a PLA edge: fine for bench pulls, PPA-CF for real tows.
- AS5600 count pulley, BTS7960 wiring, tow force testing.

## Working with Harry on this

Terse. Fix the thing, deliver the zip + sim, one short paragraph. He checks the 3D sim closely and catches geometry errors from screenshots; audit every round (watertight, pairwise intersections, cup sweep, line path clearances at all cup angles) before delivering. Do not add "clever" geometry (long lugs, lipped rings, flared posts); plain round holes, short rings, fillets. No em dashes.
