# cad-infra REPORT (worker `cad-infra`, 3 Nov 2026)

Status: draft, updated through the day. Final at 18:20 with pasted test and audit output.

## Done

API and core
- API v1 (`base/cad/README.md`): Part, Joint, Allow, Clearance, Mesh, subsystem module contract (parts, joints,
  kin; optional allowed, clearances, meshes, audit_states, checks, steps, grounded, line_path), master state S,
  world frame per BASE_SPEC section 2. Frozen since 10:31; only additive changes.
- Core: `cad/lib/core.py`, `tf.py`, `prim.py` (primitives, extrude, revolve, mesh IO, clean), `fits.py` (every
  allowance in one place).

Library `cad/lib/`
- gears: involute by rack-cutter simulation (true trochoid root, undercut, profile shift), DIN 6885 keyed bores,
  internal gear (interference checked for every pair, shifted pairs), rack, centre distance, contact ratio
  (clamped), Lewis, pointed-tip warning.
- worm + hob-cut worm wheel (self-locking from the normal-plane friction angle), trapezoidal lead screw and nut,
  barrel cam track.
- bearings (608 / CSK20PP / 693 / F693 crush-rib seats and models), shafts (20 mm keyed PPA-CF, 0.1 undersize,
  D, hex, journals), fasteners (M3 / M8 holes, inserts, traps, models).
- clutch (dog clutch with ramp gap, slip clutch with a flat-printable flexure and M3 adjuster, 10 MPa sustained),
  clips (v10 clip pins and C clips, strain-limited snap pins), printed contact switch (0.8 % strain), motors (both
  N20 types, parametric wiper plate and coupler), line (Dyneema eyes, fairlead, line tube), springs.

Tools `base/tools/`
- assembly (compose, kinematics with gear links, tether `line()`, export), timeline (P.TIMELINE with targets),
  audit (static checks incl. watertight as STL, exact pairwise intersection with per-pair refinement, motion
  envelopes, chunked booleans, allowed contacts, clearances, floating and grounded, joint limits, gear mesh check),
  meshcheck, render (24-angle sheet, 12 section cuts), sim (HD page, joint-space tracks, tether, mission view,
  dart follow, body fragment + web/*.json for the artifact), steps (stage bands, dependency validator, never drops a
  step), pack (STL, 3MF plates, PRINT_SHEET), step_export, bom, guide (PDF, coupons first), coupon (fit coupon),
  risk_coupon (day-one risk coupons from the subsystems' own functions), `base/run_all.sh`.
- Demo subsystem `cad/parts/demo.py` and the starter `cad/parts/_template.py`.

Outputs pushed
- `base/out/base/` interim site outputs (sim_body.html + web/, assembly_body.html + web/, shots/).
- `base/pack/coupons/` (COUPON_TEST.md/.pdf, risk_plate_1_PLA, risk_plate_2_PPA-CF) and `base/pack/fit_coupon/`.

## Decisions (made without Harry)

- Run from `base/`, import `cad.lib`; subsystem modules are `cad/parts/<name>.py` without an underscore.
- Audit refinement per pair: sub-states only where a pair's relative pose moves a point more than 0.5 mm and a
  joint more than 1 deg; sub-states evaluate kin on the blended state, joints are interpolated directly only
  across a string / flag change.
- Printed parts must be watertight after vertex merge (what a slicer sees), one shell, flat on the bed
  (min(max(20, 5 % of footprint), 150) mm2 contact).
- STEP export skips meshes over 100k triangles (faceted STEP is slow; the STL is the print file).
- run_all runs the audit in parallel with the outputs that do not need it.

## Review rounds (independent agents, all confirmed findings fixed with tests)

- R1 core/audit: 14 findings. R1 library geometry: 9 findings + mesh watertightness.
- R2 output tools: 13 findings. R2 web pages: 7 findings (joint-space tracks, size limits, theme).
- Coordinator review R2-E (cross-owner mesh check), R2-I (7 findings), R2-J (steps), O #12 (ground).

## Verified

(test and audit output pasted at the end of the day)

## Open items

## What Harry must measure

- Wiper motor: output shaft diameter, flats, thread, mounting hole pattern, boss (`motors.WIPER_DEFAULTS`).
- CSK20PP outer keyway depth (assumed 1.5) and race diameters.
- Worm N20 output shaft position in the box; straight N20 face holes (assumed M1.6 at 9 mm).
- Century spring wire diameter (assumed 2.0).
- M3 heat-set insert OD (hole 4.0 assumes a CNC Kitchen style M3x4x5 with about 4.6 knurl).
- Print the fit coupon and the risk coupons first and update `fits.py` (COUPON_TEST.md says how).
