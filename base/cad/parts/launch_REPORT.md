# cad-launch REPORT (launcher + aim train)

Worker cad-launch, branch `w/cad-launch`. Harry's v10 launcher, sized to KEDGE v30, on BASE_SPEC v1.2.x.
Files (all in `base/cad/parts/`): `launch.py` (module contract: parts, joints, kin, allowed, clearances,
audit_states, checks, grounded, steps, line_path, line_entry), `launch_kin.py` (sizing, crank-rocker, aim table),
`launch_gears.py` (involute spur, sector with v10 release teeth, rack, worm, wheel, bevel), `launch_aim.py` (aim train),
`launch_stop.py` (PLA flexure carriage stop), `launch_line.py` (tether pulleys on the rail), `launch_muzzle.py`
(lip roller, retrieval funnel), `launch_guard.py` (spring guards), `launch_switch.py` (printed contact),
`launch_numbers.py` (numbers table), `launch_view.py` (side views and sections), `launch_dartmesh.py` +
`launch_dart_v30*.npz` (the folded dart as a keep-out part), tests `test_launch*.py`, `launch_NOTES.md`.

## 1. Done

- Mechanism (v10): rail pivots on the main axis; launch sleeve (D13 integral ribs) coaxial on cad-drum's d12 spigot;
  sprag #1 inside the 26T drive gear; TWO-LAYER big gear on the rail pin (27-tooth sector with the v10 relieved last
  tooth and 30 percent taller tooth for the rack, plus a full 40T layer for the drive: one sector layer would leave
  the drive in the toothless gap); one big gear turn = cock, release, reset (553.8 wiper deg).
- Tooth timing from a real 2D contact sweep (`release_table`): tooth 0 sits 3 pitches before the mesh centre at rest
  (a turning gear cannot enter a stationary rack at the mesh centre); release at big gear -281.25 deg, 432.7 forward
  wiper deg from rest; carriage travel 240.9 mm.
- Springs 1:1, preload from params (23.8 at max_ext 265), D5 second-pair provision (pegs + anchor holes at y +-56),
  6 mm steel anchor pins in double shear (post + strap).
- Carriage: body prints on its side (shoe faces), light V cradle (5 rings + 2 strips), pusher with tether hole,
  lip roller and retrieval funnel at the front; one bolted YOKE across the rear carries both spring arms (keys in the
  body webs, cross bar: each arm's stop moment balances across the carriage).
- Carriage stop: printed PLA flexure units (D26, no TPU), preloaded; the loop stack goes solid at full travel (23.8 mm); overshoot checked by
  `launch.checks` against the moving yoke and body.
- Contacts (printed, M3 screw head on M3 nut): SW_HOME (right rail lip lever, rigid ramped foot under the yoke),
  SW_SEATED (left rail lip lever, captive plunger pushed down by a 45 deg wedge pushrod that the dart tail presses: no
  wires on the carriage), SW_GAP (cam on the big gear at rest), SW_RAIL_LOW (cam on the aim crank).
- Aim train: v10 crank-rocker scaled 0.6 (pitch 6.01..47.06 deg), swing-arm splitter (pawl-free) on backward turns,
  2:1 bevel, self-locking worm (lead 4.76 deg), worm wheel = crank, 10 wiper turns per cycle; aim housing on the deck
  is also the sleeve's outboard bushing with the thrust face (D23), AS5600 bracket at the magnet.
- Tether (D14/D34): pivot pulley 44 below the axis on the right cheek, bottom and top pulleys on a fork behind the
  rail, line along the axis to the pusher; `line_entry` / `line_path` hand over to cad-reel.
- Spring guards (D33), splice straps over the front rail joint, rail clear of the sleeve flange and dog ring.
- Kin for every timeline key (aim_deg, cock, fired, dart_flight, dart_return, sleeve_deg, phase "lower" runs the crank
  over the top and down the falling stroke), None-safe; dart flight arc from params.LAUNCH.flight; return along the
  line and the rail axis.
- Assembly steps (15, ordered, hardware per step), print orientation per part (`print_tf`, chosen for the largest
  flat face), notes for fw-base (aim table, switch windows).

## 2. Verified

(pasted at the end of the day, see section 6)

## 3. Decisions I made (Harry away)

1. Two-layer big gear (review 1 found the single sector gear could not be driven through its gap).
2. 27 teeth present, not 33: with the 3-pitch lead-in, 33 would stretch the springs far past max_ext.
3. Stops on the spring arms with PLA flexures; no switch in any load path.
4. SW_SEATED on the rail through a carriage plunger: a wire on a carriage that hits 16 g and stops hard would fail.
5. Carriage body + bolted yoke (the carriage could not print flat with the arms on).
6. Pivot pulley 44 below the axis (the flange r 32 and cad-drum's fork rail at r 56.6 bound it); the feed length
   changes by about 25 mm over the aim range (take-up handles it).
7. Worm in the aim train although v10's notes say "no worm gear": the system plan and coordinator asked for a printed
   self-locking stage; it is pawl-free.
8. Fallback spring hole set F 8.5 mm rearward, not the 15 mm params asked (256.5 mm at full cock instead of 250): the
   stop frame front wall ends at u 248.15 and the anchor post starts at 255.5, so 8.5 is the room there is (accepted).
9. Anchor pins are 8 mm steel rod (on Harry's list), not 6 mm dowels (not on it) or printed pins.
10. The launch sleeve prints outboard end down with supports under the flange: the D37 lugs stand on the flange face,
    which used to be the bed face.
11. In kin, the sleeve follows S["sleeve_deg"] exactly (or wiper_deg in drum-only and load states), because cad-drum's
    dog ring is bolted to it. Off the 12 deg sun tooth phase, the splitter arm is drawn swung out.
12. Sun key rib 12 mm long (the full sun face), not D8's 20: cheek B ends at y 168 and the housing bushing starts at 187.
    Bearing stress at the 3.2 N m trip is about 7 MPa.
13. Aim drag: printed PLA finger with a shim for preload (0.43 percent strain), not an M3 screw: the arm hub is 4 mm thick.

## 4. Open items and risks

- Big gear backlash on aiming (review 3): the sprag drag can carry the big gear up to 127 deg backward on aim turns
  (no damage, SW_GAP opens). Firmware rule sent to the coordinator: after any aim move, run forward until SW_GAP closes,
  then fire. A printed detent would remove the rule; not done.
- Aim bevel pair z15: SF 0.70 at 3.2 N m (5 A), about 1.65 at the D40 3 A trip for backward runs (1.36 N m). Worm
  wheel face 16, splitter face 12 (SF 1.57 to 1.83 even at 5 A). Bevels, sun, idler, worm in PPA-CF (D40).
- Fallback hole set F at 8.5 mm (256.5 at full cock): accepted by the coordinator (MEASURE_LIST B4).
- Spring guards (D33) are not a full enclosure. The side plates run |y| 66..68 over the whole stroke; segment a has a
  rear end wall; a top cover covers |y| 42..66 at w 83.5..85.5. Still open above the springs: the band |y| 28..42 over
  segments a and c, which the dart's folded flank features sweep on every shot. That band is 14 mm wide next to the
  rail wall (a d16 spring cannot pass) but 20.5 mm next to the carriage body. The arm slot at u 73.4..78.6 is also open.
  Treat the launcher as a keep-clear zone while cocked.
- Rail front piece prints with about 5000 mm2 of support under the side shelves (front plate underside at w 30
  floats 6 mm over the w 24 ties). Extending the shelves down would add about 40 cm3; supports chosen instead.
- Rail shoe shelf is notched to y 19 around the face 11 gears: the shoes bear on the outer 3 mm (y 19..22) between
  u about 30 and 135. The lips hold the carriage down there.
- PLA stop returns most of the energy: the carriage rebounds up to about 100 mm before friction and the springs settle
  it. The gear is in its gap then (audited); firmware should wait 0.5 s after the release before trusting SW_HOME.
- In tow, the line over the lip roller pushes the carriage back by T (1 - cos a): lower the rail to minimum before
  going to REEL (a about 9 deg: 4 N at 270 N) so the carriage stays on its stop.
- Moving mass about 122 g (target 60 in D6): the rack must be one stroke long; mass cuts and the stop retune are in
  progress (section 6).
- Sleeve wall over the spigot is 3.83 mm (D13 asks 4): bore 12.35 is cad-drum's bushing size on a d20 sprag bore.
  Fine for 2.7 N m (about 2 MPa); flagged to the coordinator.
- params LAUNCH still lists big_gear present 33 and drive face 12/16; the model has 27 + full layer, face 7.
- Lip roller fork cheeks are now 2.5 mm plates (funnel lightening): about 380 N in tow at a 270 N line. Pull-test a
  printed fork to 270 N line before trusting a full tow.
- Overhang warnings (supports needed): rail pieces, big gear sector side, worm, crank, aim housing. Listed on the print
  sheet by the pipeline.

## 5. What Harry must measure

1. Springs: hang test (rate and initial tension) and whether 165.1 mm is the length outside or inside the hooks. The
   model takes it as the pin centre distance at zero extension; if it is inside the hooks, the true extension at full
   cock is about 7 mm more (about 272 mm, 41 N per spring against the 38.4 N rating): then use hole set F. Set
   SPRING.k_n_per_mm, F0_n, free_len, max_ext. Preload follows automatically.
2. Stop flexure: after printing, a press test (force at 10 mm) to confirm the stiffness.
3. Fit coupon for the 0.2 mm running clearances (sleeve journals 20.4 in the cheeks, gear pin 10.35).
4. The sprag direction before assembly (inner race must freewheel the BACKWARD wiper way).
5. Lip roller fork: pull a printed fork to 270 N on the line before a full tow.
6. The aim table zero: run backward until SW_RAIL_LOW closes; record the AS5600 angle at the closing edge.

## 6. Verification output

(filled in below)
