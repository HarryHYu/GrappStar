# cad-infra REPORT (worker `cad-infra`, 3 Nov 2026)

Status: draft, updated through the day. Final at 18:30.

## Done

- API v1 (`base/cad/README.md`): Part, Joint, Allow, Clearance, subsystem module contract, master state S, world
  frame per BASE_SPEC section 2. Frozen; only additive changes since the first push.
- Core: `cad/lib/core.py`, `tf.py` (4x4 helpers), `prim.py` (primitives, extrude, revolve, mesh IO), `fits.py`
  (every allowance in one place).
- Library `cad/lib/`: gears (involute by rack-cutter simulation: true trochoid root, undercut, profile shift, internal
  gear, rack, Lewis), worm + hob-cut worm wheel with self-locking check, trapezoidal lead screw and nut, barrel cam
  track, bearings (608 / CSK20PP / 693 / F693 crush-rib seats and models), shafts (20 mm keyed PPA-CF, D, hex,
  journals), fasteners (M3 / M8 holes, inserts, traps, models), clutch (dog clutch, slip clutch with printed
  flexure and M3 adjuster), clips (v10-style clip pins and C clips, snap pins), printed contact switch, motors (both
  N20 types, parametric wiper plate and coupler), line (Dyneema eyes, fairlead, line tube), springs (Century
  anchor posts, spring model).
- Tools `base/tools/`: assembly (compose, kinematics, export), audit (static checks, exact pairwise intersection
  over a refined timeline, allowed contacts, clearances, floating and grounded, joint limits), timeline (placeholder
  mission or P.TIMELINE with targets), render (24-angle sheet, 12 section cuts, headless Chromium), sim (HD page,
  real motion, standalone and artifact body variants), steps (assembly page), pack (STL, 3MF plates, PRINT_SHEET),
  step_export, bom, guide (PDF), `base/run_all.sh`.
- Demo subsystem `cad/parts/demo.py` through the whole chain.

## Decisions (made without Harry)

- Package layout: run from `base/`, import `cad.lib`, subsystem modules `cad/parts/<name>.py` (no underscore);
  helpers `<name>_*.py` are skipped. Created empty `base/cad/__init__.py` and `base/cad/parts/__init__.py`.
- Audit refines the timeline so no joint moves more than 1 deg or 0.5 mm between audited states.
- Printed parts must be watertight after vertex merge (what a slicer sees), one shell, flat on the bed.
- Gear backlash 0.15 mm, centre distance +0.1 mm, 608 crush ribs 0.15 mm radial interference: tune with the coupon.

## Review rounds

- Round 1 core/audit (independent agent): 14 findings, all fixed with regression tests (716476e).
- Round 1 library geometry (independent agent): 9 confirmed findings plus suspicions, fixes in progress.

## Verified

(test and audit output pasted at the end of the day)

## Open items

## What Harry must measure

- Wiper motor: output shaft diameter, flats, thread, mounting hole pattern, boss (all in `motors.WIPER_DEFAULTS`).
- CSK20PP outer keyway depth (assumed 1.5) and race diameters.
- Worm N20 output shaft position in the box; straight N20 face holes (assumed M1.6 at 9 mm).
- Century spring wire diameter (assumed 2.0).
- M3 heat-set insert OD (hole 4.0 assumes a CNC Kitchen style M3x4x5 with about 4.6 knurl).
- Print the fit coupon first and update `fits.py`.
