# Round 2 E: base CAD parts and assembly

Reviewed at base-v1 commit 0bfd59a (started 12:20; later base-v1 commits may already fix some items). Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 23. Refuted and dropped: 2.

| # | Sev | File:line | Owner | Finding |
|---|---|---|---|---|
| 1 | critical | `base/cad/parts/launch.py:310` | cad-launch | Big gear is a single-layer 27-tooth sector: the drive gear sits in the 117 deg toothless gap at rest, so cocking can never start (D23 two-layer gear missing) |
| 2 | critical | `base/cad/parts/reel.py:456` | cad-reel | Reel module ignores LAYOUT.reel_frame_R: capstan placed 126.5 mm from the main axis, gear planes at world y 189..215, so no REEL, bus or cup mesh exists |
| 3 | high | `base/cad/parts/drum.py:31` | cad-drum | Drum REEL output gear 40T and bus gear 40T (spec 50T/50T) on the wrong planes; wrap idler output gear in plane y 54..62 instead of 68.5..76.5 |
| 4 | high | `base/cad/parts/drum.py:469` | cad-drum | The N20 pinion bore is a round 3.1 mm hole with no D flat, so the pinion spins on the N20 D-shaft |
| 5 | high | `base/cad/parts/drum.py:815` | cad-drum | Selector drum, wrap idler and N20 placed at drum axis (0,-72) with idler at 180 deg and N20 below; LAYOUT says drum (60,-112), idler (16.24,-104.88), N20 (99.9,-78.5) |
| 6 | high | `base/cad/parts/frame.py:106` | cad-frame | frame.cfg iterates the whole LAYOUT dict as if it were the axes table: Assembly.load crashes, so no assembly, audit or mesh check runs |
| 7 | high | `base/cad/parts/launch.py:299` | cad-launch | Carriage stop: one 8 x 5.3 mm PLA tab takes the whole stop impact (SF 0.34 at the 300 N D6 peak) |
| 8 | high | `base/cad/parts/launch.py:330` | cad-launch | Sprag #1 has no key into the drive gear: the gear has a keyway but no key part exists, and the bore is a 47.1 clearance fit |
| 9 | high | `base/cad/parts/launch.py:364` | cad-launch | The launch sleeve flange does not match the dog ring: screw holes are 45 deg off and both parts have slots, so neither has lugs |
| 10 | high | `base/cad/parts/reel.py:60` | cad-reel | Reel gear train still on superseded D3: capstan 24T (spec 22T), take-up m2 40/24/24/24/16 (spec m1.5 50/46, idler 30, reel 20); capstan ratio 1.667 instead of 2.27 |
| 11 | medium | `base/cad/parts/drum.py:311` | cad-drum | The dog ring screw counterbore is tangent to the ring OD: zero wall, non-watertight STL, audit static FAIL |
| 12 | medium | `base/cad/parts/launch.py:291` | cad-launch | Carriage spring pegs are d6 PLA cantilevers with no hook locator: SF 1.38 at the modelled hook height, 0.52 once the hook drops onto the keeper flange |
| 13 | medium | `base/cad/parts/launch.py:318` | cad-launch | The big gear and the drive gear have symmetric hubs thicker than the face, so the PPA-CF tooth rims print 2 to 2.5 mm above the bed |
| 14 | medium | `base/cad/parts/launch.py:429` | cad-launch | The launch rail rear and mid pieces print standing on the 6 mm split-flange plates, with the whole side wall in the air |
| 15 | medium | `base/cad/parts/launch.py:433` | cad-launch | The launch carriage has no flat face to print on: in its print_tf it stands on the 8 x 13.5 end of one spring arm |
| 16 | medium | `base/cad/parts/launch_aim.py:46` | cad-launch | Aim splitter and bevel built at m1.5 face 8, which physics says fails at the 5 A trip (sun SF 0.76, idler 0.66, bevel z15 about 0.45; 0.42 at the 5 N m limiter) |
| 17 | medium | `base/cad/parts/launch_aim.py:366` | cad-launch | The aim lug prints standing on the end of its d8 pin (50 mm2), and the audit fails it |
| 18 | medium | `base/tools/audit.py:1` | cad-infra | Audit has no cross-owner gear-mesh check: gears 60+ mm apart or on different modules pass CLEAN |
| 19 | low | `base/cad/parts/drum.py:446` | cad-drum | Wrap pinion (m1.25 z10 x0.4) and N20 pinion (m0.8 z10 x0.3) have pointed teeth below the printable tip land |
| 20 | low | `base/cad/parts/drum.py:468` | cad-drum | The wrap pinion and the N20 pinion have pointed teeth (tip land 0.27 and 0.20 mm) that a 0.4 nozzle cannot print |
| 21 | low | `base/cad/parts/drum.py:743` | cad-drum | Drum gear 120T m0.8 face 8 printed in PLA (drum A): SF 0.77 at the N20 stall it is sized against |
| 22 | low | `base/cad/parts/reel.py:431` | cad-reel | The storage reel prints 'front face down' on its 0.3 mm raised clutch annulus, so the front flange and 608 seat floor hang in the air |
| 23 | low | `base/cad/parts/reel.py:509` | cad-reel | Reel kinematics never turn the capstan from the wiper: REEL/tow states show the drum REEL gear turning while the capstan gear stays at 0 |

## 1. Big gear is a single-layer 27-tooth sector: the drive gear sits in the 117 deg toothless gap at rest, so cocking can never start (D23 two-layer gear missing)

- File: `base/cad/parts/launch.py:310`
- Severity: critical (claimed critical); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: Rest state (cock 0, big gear 0 deg) after any reset. The wiper turns forward in LAUNCH, but the 26T drive gear faces the 13-tooth gap of the same sector layer, so the big gear never turns and the dart is never cocked. After each shot the reset drives the gear back to -360 = 0, which is back in the gap. I posed launch_big_gear in the assembly from 0 to -360 deg and intersected it with a r3.5 probe at the drive-gear pitch point (/tmp/claude-0/review-scratch-E/bg.py). From 0 to -110 deg only the 5.4 mm3 root sliver is present (no tooth). Teeth (151 to 254 mm3) appear only from -120 to -350, and the gap returns at -360. So the drive gear has no tooth to push for the first ~115 deg of every cock.

Evidence: launch.py:310-320 `def big_gear_solid(P): ... cs = G.sector_gear_2d(m, n["z_big"], n["n_present"], first_tooth_angle=n["fta"]); g = prim.extrude(cs, n["face"])` builds one layer only. launch_kin.py:31 `face=12.0`. Run output: `cd 82.6 rb 50.0 rd 32.5 face 12.0 npresent 27 nlead 3 fta -63.0`, probe `0 5.4 ... -110 5.4, -120 171.1 ... -350 159.9, -360 5.4`. params LAUNCH.big_gear: `layers=dict(sector=(134.0, 146.0), full=(146.0, 154.0)) ... a full 40T layer meshes the drive gear so every cock can start (D23)`. The drive gear is placed at y 134..146 (launch.py:72 `Y0=... 140.0`, launch.py:442 `tf.T(0, 0, -n["face"] / 2)`), i.e. in the sector plane.

Fix: In big_gear_solid, union the sector extrusion over world y 134..146 (rack plane) with a full 40T m2.5 spur layer (G.spur_2d(m, 40) extruded) over y 146..154. Move the drive gear so it overlaps only the full layer: place it from LAUNCH.drive_gear y and face (16), or at least keep it inside y 146..154 with no overlap of the sector layer. Read LAUNCH.big_gear.layers and LAUNCH.drive_gear.y instead of the hard-coded Y0 +- face/2. Add a test that, at every big-gear angle 0..-360, at least one full-layer tooth sits at the drive pitch point.

> Skeptic (confirmed): I could not refute this. The defect is real. big_gear_solid (base/cad/parts/launch.py:310-320) extrudes one layer only: G.sector_gear_2d(m, z_big=40, n_present=27, fta=-63) over face=12. No other code path adds a full 40T layer. A grep for "full", "layers" and big_gear across base/cad finds nothing that reads LAUNCH.big_gear.layers. The drive gear is placed around Y0=140 with face 12 (launch.py:442), which puts it in the same plane as the sector. It does not use LAUNCH.drive_gear y=(142.5,158.5), face 16. params.LAUNCH.big_gear and DECISIONS D23 both require a two-layer gear, a sector layer fo
>
> Skeptic (confirmed): I could not refute this. The defect is real and the claim is accurate.

1. base/cad/parts/launch.py:310-320, big_gear_solid builds exactly one layer: G.sector_gear_2d(m, z_big, n_present, fta) extruded by n["face"], plus a hub. No full 40T layer exists anywhere. Grepping launch*.py for "full" or "layer" finds no gear layer.

2. launch_kin.py DEFAULTS has n_teeth=27, z_big=40, face=12.0. So 13 tooth slots, 117 deg, are empty on the only layer.

3. The drive gear (launch.py:442) and the big gear are both placed with tf.Rx(-90)·T(0,0,-face/2). That puts them in the same 12 mm plane around Y0=140,
>

## 2. Reel module ignores LAYOUT.reel_frame_R: capstan placed 126.5 mm from the main axis, gear planes at world y 189..215, so no REEL, bus or cup mesh exists

- File: `base/cad/parts/reel.py:456`
- Severity: critical (claimed critical); 2 of 2 skeptics confirmed
- Owner: cad-reel

Scenario: Any assembly or build from this code. _placement looks for LAYOUT['reel'], which params does not have (the key is 'reel_frame_R'), so it falls back to (-40, 140, 45) in deck coordinates. Measured in the assembly: the capstan axis is at x -40, z 45 (deck), i.e. (-40, -120) from the main axis, distance 126.5 (spec 72.1 at 255 deg, x -18.66 z -69.64). reel_gc spans y 188..201, reel_bus 161..188 and reel_gk (cup gear) 206..215, while the drum's REEL gear is at y 40.5..59.5, its bus gear at y 1..9 and its idler output at y 54..62. No gear pair overlaps in y or sits at its centre distance: wind-in, take-up and wrap are all undriven. The R-frame planes are also wrong on their own: even placed at the LAYOUT origin y 33.5 they would be 32 mm off the spec planes.

Evidence: reel.py:456-461 `def _placement(P): lay = getattr(P, "LAYOUT", None); if isinstance(lay, dict) and isinstance(lay.get("reel"), dict): ... return (-40.0, 140.0, H_AXIS), 0.0`. params LAYOUT key: `reel_frame_R=dict(origin=(_CAP_X, 33.5, _CAP_Z), ... note="capstan gear plane R Z -29..-17 = world y 50.5..62.5")`. reel.py:57-59 `BUS_Z = (-47.5, -35.5)`, `GC_Z = (-61.0, -49.0)`, `CUPGEAR_Z = (-75.0, -67.0)`. The spec needs bus R Z -14.5..-6.5, Gc -29..-17 and cup -43..-35. Assembly dump: `reel_gc org [-40. 201. 45.] bbox y 188.1..201`, `drum_gear_reel bbox y 40.5..59.5`.

Fix: In _placement read LAYOUT['reel_frame_R'] (origin plus the x/y/z axes: base = rotation with R +X -> world +X, R +Y -> world +Z, R +Z -> world -Y). Convert the origin to the module's world frame consistently with the other modules (main axis at deck +165). Move the planes to the spec: BUS_Z = (-14.5, -6.5) (or the c1 face-6 band inside it), GC_Z = (-29.0, -17.0), CUPGEAR_Z = (-43.0, -35.0). Re-derive GEAR_Z, the wall and the boss so they stay clear.

> Skeptic (confirmed): I could not refute this claim. The defect is real.

What the code does:
- base/cad/parts/reel.py:456-461. `_placement` only looks up `LAYOUT['reel']`. base/spec/params.py has no such key; it defines `reel_frame_R` at line 114, with origin (_CAP_X, 33.5, _CAP_Z), which works out to (-18.661, 33.5, -69.643). So the function always falls back to `(-40.0, 140.0, H_AXIS=45)`.
- I ran it to confirm: `reel._placement(P)` returns `((-40.0, 140.0, 45.0), 0.0)`.
- Drum's `drum_base` joint sits at (0, 0, 165) in the same deck frame, which puts the main axis at deck z 165. The capstan therefore lands at (
>
> Skeptic (confirmed): The claim holds up. I checked the code and ran the reel and drum modules myself.

1. **The layout key is never read.** reel.py:456-461 `_placement` looks only for `LAYOUT["reel"]`. params.py has no such key: the reel frame is at `LAYOUT["reel_frame_R"]` (line 114) and the capstan is at `LAYOUT["axes"]["capstan"]`. So the code always falls back to `(-40.0, 140.0, H_AXIS=45)` with rot 0. No other code places the reel; grep finds no other reader of `reel_frame_R`.

2. **Measured positions match the claim.** I loaded the assembly with only reel and drum, because frame.py crashes in `cfg()` with a 
>

## 3. Drum REEL output gear 40T and bus gear 40T (spec 50T/50T) on the wrong planes; wrap idler output gear in plane y 54..62 instead of 68.5..76.5

- File: `base/cad/parts/drum.py:31`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-drum

Scenario: With the capstan where LAYOUT puts it (72.1 from the main axis), the m2 40T REEL gear (pitch r 40) and the 22T capstan gear (r 22) leave a 10 mm radial gap (62.1 vs 72.1): REEL mode never turns the capstan, so no tow and no reel-in. The m1.5 40T bus (r 30) against c1 46T (r 34.5) gives 64.6 vs 72.1: no take-up, so the cocking slack is snatched every shot (D15). The planes are also off: REEL gear face at s 40.5..54.5 (spec 50.5..62.5), bus at s 1..9 (spec 40..48), idler output at s 54..62 (spec 68.5..76.5, the cup plane). The idler output plane overlaps the spec REEL/capstan plane 50.5..62.5.

Evidence: drum.py:28 `bus_m=1.5, bus_z=40, bus_s=(1.0, 9.0),`. drum.py:31 `out_m=2.0, out_z=40, out_face=14.0, rgear_s=40.5,`. drum.py:54 `idl=(25.0, 70.0), idl_gear=(54.0, 62.0)`. params DRUM: `bus=dict(module=1.5, teeth=50, face=8.0, plane_y=(40.0, 48.0) ...)`, `reel_output=dict(module=2.0, teeth=50, face=12.0, plane_y=(50.5, 62.5))`, `wrap_train=dict(... plane_y=(68.5, 76.5) ...)`. Assembly bbox: `drum_gear_reel bbox [-42. 40.5 ...] [42. 59.5 ...]` (tip r 42 = 40T m2), `drum_idler_gear bbox y 54..62`. Nothing reads DRUM['bus'] or DRUM['reel_output']: K.cfg only takes keys already in _drum_kin.DEFAULTS, and G() reads only DRUM['geom'].

Fix: Set out_z=50, out_face=12, rgear_s=50.5 (dogs to 67.5), bus_z=50, bus_s=(40.0, 48.0) (on the outer coupler after the shear pin; extend cpl_s accordingly), idl_gear=(68.5, 76.5). Better, read them from P.DRUM['reel_output'], P.DRUM['bus'] and P.DRUM['wrap_train'].plane_y in G(). Re-check the collar, dog ring and drum-A clearances against the REEL gear tip r 52.

> Skeptic (confirmed): I checked the claim against the code and it holds. In base/cad/parts/drum.py, GEOM hard-codes bus_m=1.5, bus_z=40, bus_s=(1.0, 9.0) (line 28), out_m=2.0, out_z=40, out_face=14.0, rgear_s=40.5 (line 33; the claim cites line 31) and idl_gear=(54.0, 62.0) (line 54). G() only overrides keys from P.DRUM['geom'], and params.DRUM has no 'geom' key. K.cfg() only takes DRUM keys that are already in _drum_kin.DEFAULTS. So nothing in the drum code reads DRUM['bus'] (50T, y 40..48), DRUM['reel_output'] (50T m2, face 12, y 50.5..62.5) or DRUM['wrap_train'].plane_y (68.5..76.5). The gears are built from the
>
> Skeptic (confirmed): The claim holds. Only the line numbers are a little off: the cited lines are drum.py:28, :33 and :54, not :31. What the code sets in base/cad/parts/drum.py GEOM:
- bus_m=1.5, bus_z=40, bus_s=(1.0, 9.0)
- out_m=2.0, out_z=40, out_face=14.0, rgear_s=40.5
- idl_gear=(54.0, 62.0)
- the collar is at 62..88; the spec stack puts it at 72..98.

These values are used directly: reel_gear() builds spur_gear(out_m, out_z, ...) at rgear_s (line ~271), the bus is built from bus_m/bus_z/bus_s (line ~233), and the idler output from idl_gear (line ~457).

Nothing overrides them. G() only applies P.DRUM['geom']
>

## 4. The N20 pinion bore is a round 3.1 mm hole with no D flat, so the pinion spins on the N20 D-shaft

- File: `base/cad/parts/drum.py:469`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-drum

Scenario: The assembly step says to press the pinion onto the N20 output shaft 'flat to flat'. The printed drum_n20_pinion has a plain round 3.1 mm bore (0.1 mm diametral clearance on a 3 mm shaft), so it can neither be pressed on nor keyed. When the N20 drives the drum (up to about 0.2 N m stall, FIRM 5 N on the fork), the pinion slips on the shaft. The selector drum does not move, and the drum stays at its old position while the firmware times out on the code switches.

Evidence: drum.py:469 `return p.translate((0, 0, g["dgear"][0])) - cyl(1.5 + 0.05, g["dgear"][0] - 1, g["dgear"][1] + 1, seg=24)`; drum.py:935 step text `"Press the pinion onto the N20 output shaft (flat to flat), flush with the shaft end."`; drum_NOTES.md:234 'N20 output shaft diameter and flat (modelled d3, D flat)'.

Fix: Cut a D bore that matches the N20 flat, with a light press: `bore = cyl(1.5 - 0.05, ...) - prim.box(1.5 - 0.5 + 0.0, 5, -5, 5, z0, z1)` (a 3.0 mm D with a 2.5 mm across-flat, -0.1 for a press). That is, subtract `cyl(1.45, dg0-1, dg1+1) - box(1.0, 5, -5, 5, dg0-2, dg1+2)` in place of the round cylinder, with the flat depth taken from the measured N20 shaft.

> Skeptic (confirmed): I could not refute this claim. In base/cad/parts/drum.py:466-469, n20_pinion() subtracts a plain round cylinder, cyl(1.5 + 0.05, ...), with seg=24. That is a 3.1 mm round bore with no flat and no press interference. It is the only bore subtraction for this part, and Part "drum_n20_pinion" at line 757 uses the function's output directly. Nothing elsewhere adds a flat, a set-screw hole or a key.

The notes contradict the geometry. Line 759 says "pressed on the N20 D shaft", step text at line 935 says "flat to flat", and drum_NOTES.md:234 says the shaft is "modelled d3, D flat". The file also sho
>
> Skeptic (confirmed): I checked the code and the defect is real. In base/cad/parts/drum.py:466-469, n20_pinion cuts only one plain round bore, `cyl(1.5 + 0.05, ...)` (radius 1.55, so diameter 3.1, 24 segments). The pinion has no D-flat key, no press interference, no set screw and no glue step. The N20 model in base/cad/lib/motors.py:22-23 and 79-95 (n20_worm uses _dshaft) is a 3.0 mm shaft with a D flat, 2.5 mm across. So the part has 0.1 mm of diametral clearance and nothing that keys it. The rest of the build contradicts this bore:
- Assembly step drum.py:935 says to press the pinion on "flat to flat".
- The part
>

## 5. Selector drum, wrap idler and N20 placed at drum axis (0,-72) with idler at 180 deg and N20 below; LAYOUT says drum (60,-112), idler (16.24,-104.88), N20 (99.9,-78.5)

- File: `base/cad/parts/drum.py:815`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-drum

Scenario: The drum is placed directly under the main axis: the assembly shows drum_drum_a origin x 0, deck z 93, i.e. (0,-72) from the main axis, versus LAYOUT (60,-112). The LAYOUT capstan axis (-18.66,-69.64) is then only 18.8 mm from the drum axis, inside drum A's 120T gear (tip r 48.6, s 16..24) and segment gear (r 38.75). A capstan and cup installed per the layout collide with the drum. The wrap idler lands at (-44.33,-72), 25.8 mm from the LAYOUT capstan instead of the 49.6 needed for the 42T/24T mesh. The N20 sits 30 deg below forward (L y is down) instead of 40 deg above forward. cad-frame's deck holes taken from LAYOUT would not match the brackets.

Evidence: drum.py:23 `drum_off=72.0,          # drum axis below the main axis`. drum.py:80-81 `def drum_xy(g): return (0.0, g["drum_off"])`. drum.py:51 `phi_pin=-90.0, phi_idler=180.0, phi_brake=200.0, phi_sw=0.0, phi_n20=30.0,`. drum.py:815 `Joint("drum_drum", "drum_base", "rev", (dx, dy, 0), ...)`. params LAYOUT axes `drum=dict(owner="cad-drum", origin=(60.0, 0.0, -112.0) ...)`, DRUM.wrap_train `idler_axis_xz=(16.24, -104.88)`, `n20=dict(axis_xz=(99.9, -78.5) ...)`. Check: |drum(60,-112)-idler(16.24,-104.88)| = 44.33 = gears.center_distance(1.25,60,10,0,0.4), |drum-N20| = 52.1 = a_n20, |idler-capstan| = 49.6. The layout uses the drum's own centre distances; only the placement and angles are wrong.

Fix: Place the drum from LAYOUT.axes.drum.origin: drum_xy returns (60.0, 112.0) in L (x fwd, y down), or read it from P. Set phi_idler = atan2(-7.12, -43.76) = 189.24 deg (idler 7.12 above and 43.76 behind the drum axis) and phi_n20 = atan2(-33.5, 39.9) = -40.0 deg. Move the brackets, the switch block, the fork rail (rail_xy) and the fork reach to the new axis so the fork still reaches the collar groove.

> Skeptic (confirmed): I could not refute this. The defect is real.

What the code does:
- drum.py GEOM sets drum_off=72.0, and drum_xy(g) returns (0.0, g["drum_off"]).
- G(P) only overrides main_height and s0_world_y from LAYOUT.main, plus DRUM["geom"] keys. params.DRUM has no "geom" key and nothing reads LAYOUT.axes.drum, so nothing replaces the 72.
- I ran drum.joints(params) and got drum_drum (0, 72, 0), drum_idler (-44.33, 72, 0) and drum_n20 (45.12, 98.05, 0). These are in frame L (x forward, y down, origin on the main axis).
- In world X/Z that puts the drum axis at (0, -72), the idler directly behind it at (
>
> Skeptic (confirmed): I could not refute this. The claim is real.

I ran drum.joints(P) against base/spec/params.py. It returns drum_drum at L (0, 72, 0), drum_idler at (-44.33, 72, 0) and drum_n20 at (45.12, 98.05, 0), with drum_base at (0, 0, 165). L is x forward, y down. So relative to the main axis the drum is at world (x 0, z -72), the idler is at (-44.33, -72), and the N20 is 30 deg below forward.

The contract says otherwise. params LAYOUT.axes.drum.origin is (60, 0, -112), and BASE_SPEC.md line 97 says "Selector drum axis at (x 60, z -112)". DRUM.wrap_train.idler_axis_xz is (16.24, -104.88), and DRUM.n20.ax
>

## 6. frame.cfg iterates the whole LAYOUT dict as if it were the axes table: Assembly.load crashes, so no assembly, audit or mesh check runs

- File: `base/cad/parts/frame.py:106`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-frame

Scenario: Assembly.load(load_params()) with the default subsystems (drum, frame, launch, reel) raises TypeError at the first LAYOUT key, deck_top_z = -165.0 (a float), so tools.assembly, audit and steps cannot produce any output. None of the gear-mesh and placement mismatches above can be caught by the pipeline.

Evidence: frame.py:88 `lay_src = _get(P, "LAYOUT", None)`. frame.py:105-106 `for n, s_ in lay_src.items(): ow = tuple(map(float, s_["origin"]))`. Run: `File "base/cad/parts/frame.py", line 106, in cfg  ow = tuple(map(float, s_["origin"])) TypeError: 'float' object is not subscriptable`. params LAYOUT holds the axes under `axes=dict(main=..., drum=..., capstan=..., aim_crank=...)`.

Fix: Use lay_src = (_get(P, "LAYOUT", {}) or {}).get("axes") at frame.py:88, skipping entries without an 'origin'. Add a test that Assembly.load(load_params()) succeeds with all subsystems.

> Skeptic (confirmed): The claim holds, and I reproduced it. frame.cfg (base/cad/parts/frame.py:88) sets lay_src = getattr(P, 'LAYOUT'), which is the whole LAYOUT dict from base/spec/params.py:86. That dict's top-level keys are deck_top_z (-165.0), footprint, axes, stack and others. Lines 105-106 then loop over lay_src.items() and read s_['origin'] on each value. The first value is the float -165.0, so this raises TypeError. Nothing catches it. base/tools/params.py load_params returns spec.params, and the stub only fills in names that are missing, so P.LAYOUT is always the real nested dict. I ran frame.cfg(load_para
>
> Skeptic (confirmed): The claim holds; I reproduced it. In base/spec/params.py:86, LAYOUT is a dict whose top-level keys are deck_top_z (-165.0), footprint, axes, stack, rail_plane_y and so on. The per-axis entries are under LAYOUT["axes"]. base/cad/parts/frame.py:88 reads lay_src = _get(P, "LAYOUT", None). Lines 104-106 then run `for n, s_ in lay_src.items(): ow = tuple(map(float, s_["origin"]))`, so the first value s_ is the float -165.0. tools/params.load_params imports spec.params, and the stub only fills in names that are missing, so the real LAYOUT is the one used. I ran `cd base && python3 -c "from tools.ass
>

## 7. Carriage stop: one 8 x 5.3 mm PLA tab takes the whole stop impact (SF 0.34 at the 300 N D6 peak)

- File: `base/cad/parts/launch.py:299`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: Every shot ends with the carriage hitting the stop at HOME. The rail has a stop block (line 215) and a pad (line 467) on the -y side only. Running a 0.3 mm interference check with manifold confirms that only the -y tab touches the pad, over y -14.5..-9.5 and w 44..50; the +y tab touches nothing. That tab is a PLA cantilever hanging from the deck (root at w 53.7), with the contact centroid at w 47, so the lever is 6.7 mm. The section is 8 mm along the load (u) by 5.3 mm (y), so Z = 5.3*8^2/6 = 56.5 mm3. At the D6 design peak of 300 N, M = 2018 N mm and sigma = 35.7 MPa. Against the PLA allowable of 12 MPa (physics common.py) that is SF 0.34. Physics FINDINGS F3 gives 434 N for a 15 mm pad, and _frame_calc.py uses 762 N: at 434 N sigma = 51.6 MPa, which is above the in-plane ultimate strength of printed PLA. The tab cracks within a few shots. The carriage then overruns HOME with nothing stopping it, slides forward off the shoe track (the springs go slack past the preload), and runs the rack into the big gear and the front posts.

Evidence: launch.py:215 `s.append(B(ust, ust + 27.0, -n["y_shoe0"], -(n["stop_y0"] - 0.5), n["w_floor0"], n["tab_w0"] + 6))` (one block, -y only); launch.py:298-299 `# stop tab under the rear ...` `s.append(Bs(0, 8, n["stop_y0"], n["y_shoe0"] - 0.7, n["tab_w0"], n["w_deck0"] + 0.1))`; launch.py:467 `Part("launch_stop_pad_r", B(0, PAD_L, -5.0, 0, 0, 14), "TPU", ...)` (one pad). Measured: tab_w0 44, w_deck0 53.725, stop_y0 9.5, y_shoe0 15.5; interference with the pad only at ((78.6,-14.5,44),(78.9,-9.5,50)).

Fix: Mirror the stop block and the pad to +y so both tabs carry the stop (add `s.append(B(ust, ust + 27.0, n["stop_y0"] - 0.5, n["y_shoe0"], n["w_floor0"], n["tab_w0"] + 6))`, the matching pocket, and a `launch_stop_pad_l` part). Lengthen the tab along u from 8 to 16 mm: `Bs(0, 16, ...)`, with the stop blocks and ust moved 8 mm forward to match. Per tab at 434/2 N, Z = 5.3*16^2/6 = 226 mm3, M = 217*6.7 = 1454 N mm and sigma = 6.4 MPa, which gives SF 1.9.

> Skeptic (confirmed): I could not refute this. I checked it against the code at /home/user/GrappStar/base/cad/parts/launch.py and evaluated L(params).

Geometry, all confirmed:
- Bs() mirrors in y, so the carriage does have two stop tabs, at y +/-(9.5..14.8), w 44..53.725, u 0..8. The carriage is PLA (line 432).
- The rail stop block (line 215) and the pad pocket (line 240) exist only at -y. The only pad part is launch_stop_pad_r (line 467), at -y. So only the -y tab takes the stop.
- The tab is connected only to the deck underside at w_deck0 = 53.725. The deck spans |y| up to 17.5. The shoe starts at y 15.5 and th
>
> Skeptic (confirmed): I could not refute it. I checked each part of the claim against the code in base/cad/parts/launch.py.

1) Only one tab takes the load. `Bs` (line 151) builds a tab on both sides, so carriage_solid line 299 makes tabs at y +/-(9.5..14.8). The rail, though, gets only one stop block (line 215, y -15.5..-9.0) and one pad pocket (line 252), both on -y. The single TPU pad `launch_stop_pad_r` (line 467) sits at y -14.5..-9.5, w 36..50. Nothing on +y can meet the +y tab. So all of the stop impact goes into the -y tab, and the off-centre hit also twists the carriage about w.

2) The tab works as a cant
>

## 8. Sprag #1 has no key into the drive gear: the gear has a keyway but no key part exists, and the bore is a 47.1 clearance fit

- File: `base/cad/parts/launch.py:330`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: When cocking, the wiper turns forward, the sleeve rib locks the sprag inner race and the sprag outer race has to drive the 26T gear at 2.6 to 3.1 N m. drive_gear_solid gives a plain 47.1 bore (radial clearance 0.05 over the 47.0 sprag OD) plus an empty 6.2 x 4 slot. Nothing in parts() fills the slot: no key and no integral rib (sprag_solid is a plain tube). bearings.py gives the CSK20PP outer keyway as only 1.5 deep, and that keyway is empty too. The outer race spins in the gear bore, the gear never turns, the spring is never cocked, and no shot can be fired. Assembly step 6 tells Harry to put 'its outer key in the gear slot', but there is no key on the bought part or in the parts list.

Evidence: launch.py:329 `g = g - prim.cyl(sp["od"] / 2 + 0.05, 40, z=-20)`; launch.py:330 `g = g - prim.box(sp["od"] / 2 - 1.0, sp["od"] / 2 + 3.0, -sp["key"] / 2 - 0.1, sp["key"] / 2 + 0.1, -20, 20)`; launch.py:373 `return prim.tube(sp["od"] / 2, sp["bore"] / 2, sp["w"], z=-sp["w"] / 2)`; bearings.py:35 `CSK20_OKEY_W, CSK20_OKEY_D = 6.0, 1.5`; launch.py:678 'Press sprag #1 into the drive gear with its outer key in the gear slot'.

Fix: Replace the slot cut with an integral key rib, the way reel.py seats sprag #2. Cut the bore with `bearings.seat_csk20(key_deg=...)`, or add `g = g + prim.box(sp["od"]/2 - (bearings.CSK20_OKEY_D - bearings.CSK20_KEY_RCLEAR), sp["od"]/2 + 0.1, -2.8, 2.8, -(sp["w"]+2)/2 + n["face"]/2, ...)` (a 5.6 wide rib standing 1.3 into the bore over the 14 mm sprag width). Alternatively, add a 6 x 4.5 x 14 steel key part to parts() and the BOM. Rib bearing at the 5 N m limiter: 213 N / (1.3*14) = 11.7 MPa, which gives SF 2.1 on PPA-CF.

> Skeptic (confirmed): I could not refute this claim. Every piece of evidence it cites is in the code.

- **The gear has a slot but no key.** `drive_gear_solid` (base/cad/parts/launch.py:329-330) cuts a plain bore of r 23.55 (47.1 diameter, a 0.1 diametral clearance over the 47.0 OD). It then cuts a 6.2 wide slot from r 22.5 to r 26.5. Nothing is added back into the bore: no rib, no key.
- **The sprag has no key either.** `sprag_solid` (line 335) is a plain tube. The real CSK20PP has keyways, not keys, on both races. bearings.py:16-17 and :35 give the outer keyway as 6 x 1.5. physics/common.py:279 says "keyways insi
>
> Skeptic (confirmed): I checked the claim against the code and it holds. In base/cad/parts/launch.py, drive_gear_solid (lines 323-332) cuts a plain bore of radius sp["od"]/2 + 0.05 (47.1 dia., a clearance fit: no crush ribs, no interference). It then subtracts a box at r 22.5..26.5, 6.2 wide. That is a slot cut out of the gear wall, not a rib, so nothing reaches into the sprag's outer keyway. sprag_solid (line 373) is a plain tube. parts() (lines 422-460) has launch_drive_gear, launch_sprag (bought "CSK20PP 20x47x14") and launch_sleeve, and no key part. bom.py builds the BOM from parts(), so no key is bought either
>

## 9. The launch sleeve flange does not match the dog ring: screw holes are 45 deg off and both parts have slots, so neither has lugs

- File: `base/cad/parts/launch.py:364`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: At the step 'Main shaft: LAUNCH dog ring', the drum_dogring_launch is placed on the launch_sleeve flange. The ring's M3 holes are at 0/120/240 deg and its lug slots at 60/180/300 deg. The sleeve's holes are at 45/165/285 deg, and it also has SLOTS at 60/180/300 deg (cut with s - box), not lugs. When the slots line up, the screw holes are 45 deg apart. I measured this by posing both parts with tools.assembly at t = 0: ring holes at angles 0/120/240, sleeve holes at about 79.6/-40.4/199.6 around the axis (40.4 deg off with a 4.6 deg slot offset). The 3 x M3 x 12 cannot go in. Even if new holes were drilled, nothing carries the launch torque (about 50 N m sprag path) except 3 M3 screws in shear. On top of that, the hex nut traps (circumradius 3.35 at r 28 in an r 32 flange) leave 0.65 mm of wall at the flange rim, below the 1.2 mm minimum.

Evidence: launch.py:364 `a = math.radians(45 + 120 * k)` (screw holes); launch.py:367-368 `hexr = 5.8 / 2 / 0.866` / `s = s - prim.cyl(hexr, 3.0, x=x, y=y, ...)`; launch.py:369-371 `b = math.radians(60 + 120 * k)` `slot = prim.box(15.0, 21.5, -3.2, 3.2, -0.01, 3.2)` `s = s - prim.place(slot, ...)`. Contract drum_NOTES.md:64-66: 'Lugs: 3 lugs 6.0 wide x 3.0 tall at r 15..21, at 60, 180 and 300 deg ... They go into my slots' and 'Screws: 3 x M3 holes at r 28, 0 / 120 / 240 deg, with nut traps on the outboard side'. drum.py:304 `a = 60.0 + 120.0 * k` (ring slots), drum.py:308 `a = math.radians(120.0 * k)` (ring holes).

Fix: In sleeve_solid: use `a = math.radians(120 * k)` for the holes and nut traps. Replace the slot subtraction with lugs that are added and stand proud of the flange face toward the motor: `s = s + prim.place(prim.box(15.0, 21.0, -3.0, 3.0, -3.0, 0.01), tf.Rz(60 + 120 * k))`. Move the ring's mating face or the y_flange by 3 mm so the lugs sit in the ring's 3.2-deep slots. Either grow the flange to r 33, or move the bolt circle and nut traps to r 27 on both parts (drum ring_screw_r too), so the nut trap wall is at least 1.2 mm.

> Skeptic (confirmed): I couldn't refute this. The code does what the claim says.

1) The angles are measured in the same frame on both parts. drum_dogring_launch hangs off joint drum_out_launch, whose parent drum_base has base=tf.Rx(-90) (drum.py:807-810). launch_sleeve is placed with tf.T(...)·tf.Rx(-90) (launch.py:447) on a rev joint about world +y. Both map local z to world +y and local x/y to world x/-z the same way. In LAUNCH mode the two joints are posed at the same angle w = wiper_deg (drum.py:854, launch.py:586), so they turn together.

2) Ring (drum.py:295-312): lug slots at 60+120k cut into the back face 
>
> Skeptic (confirmed): The defect is real, but one part of the scenario is overstated.

What the code does:
- The sleeve (base/cad/parts/launch.py:363-371) puts its M3 holes and nut traps at 45/165/285 deg. It also cuts three 6.4 x 3.2 slots at 60/180/300 deg into the flange face (z 0..3.2). These are subtractions, not lugs.
- The ring (base/cad/parts/drum.py:301-311) also cuts slots at 60/180/300 deg, in its back face at s 101, and puts its holes at 0/120/240 deg.
- I searched base/cad for "lug". Neither part adds a lug and no separate key part exists, so a cavity faces a cavity.
- drum_NOTES.md 3b says the sleeve 
>

## 10. Reel gear train still on superseded D3: capstan 24T (spec 22T), take-up m2 40/24/24/24/16 (spec m1.5 50/46, idler 30, reel 20); capstan ratio 1.667 instead of 2.27

- File: `base/cad/parts/reel.py:60`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-reel

Scenario: REEL output 50T m2 (spec) with a 24T capstan gear needs a 74.1 centre, not 72.1, so the gears jam 2 mm deep. The take-up c1 is m2, but the drum's bus gear is m1.5 (drum.py:28), and gears of different modules cannot mesh at any centre. With the spec 50T m1.5 bus at 72.1, an m2 24T c1 does not fit either. reel_calc uses up = 40/24 = 1.667 instead of 50/22 = 2.27. Its tension_from_amps/amps_for_tension and line speed are therefore off by 2.27/1.667 = 1.36x, against D32's 17 N/A model through 2.27, and line_n_at_stall is overstated.

Evidence: reel.py:60 `M_CAP, Z_MAIN, Z_CAPG, X_CAPG = 2.0, 40, 24, 0.0        # REEL output 40T : capstan gear 24T (D3)`. reel.py:61 `M_TAKE, Z_BUS, Z_C1, Z_C2, Z_IDL, Z_REELG = 2.0, 40, 24, 24, 24, 16`. reel_calc.py:28 `m_cap=2.0, z_main=40, z_cap=24, ... m_take=2.0, z_bus=40, z_c1=24, ...` and reel_calc.py:173 `up = d["z_main"] / d["z_cap"]`. params REEL: `capstan=dict(... gear=dict(module=2.0, teeth=22 ...))`, `capstan_ratio=50.0 / 22.0`, `take_up=dict(c1=dict(module=1.5, teeth=46 ...), swing_idler=dict(module=1.5, teeth=30), reel_gear=dict(module=1.5, teeth=20), disk_ratio=2.5)`. D20 and D25. Check: m2*(50+22)/2+0.1 = 72.1, m1.5*(50+46)/2+0.1 = 72.1, while m2*(50+24)/2+0.1 = 74.1.

Fix: Set Z_MAIN=50 and Z_CAPG=22 (m2). Set M_TAKE=1.5, Z_BUS=50, Z_C1=46, Z_IDL=30, Z_REELG=20, and pick the c2 (c1-hub) tooth count so the disk ratio is 2.5: 50/46 * z_c2/20 = 2.5 gives z_c2 = 46. Recompute idler_xy() for the new centres. Apply the same numbers to reel_calc.DEFAULTS (z_main 50, z_cap 22, m_take 1.5, ...), or read them from P.REEL.capstan.gear and P.REEL.take_up, so the tension and speed tables use 2.27.

> Skeptic (confirmed): I could not refute this. The defect is real.

- **reel.py is still on D3.** Lines 60-61 set the capstan train to m2 40T driving 24T and the take-up train to m2 40/24/24/24/16. The comment on line 60 cites D3 by name.
- **D3 is superseded.** D20 replaces it with REEL output m2 50T driving a capstan gear m2 22T at a 72.1 centre. D25 sets the take-up bus to m1.5 50T meshing c1 m1.5 46T, with a 30T idler and a 20T reel gear.
- **params.py agrees with D20 and D25**: capstan gear module 2.0 with 22 teeth, `capstan_ratio=50/22`, and the `take_up` c1/idler/reel values match. Line 83 places the capstan
>
> Skeptic (confirmed): I checked the code and the claim holds.

1. **reel.py still uses the old D3 numbers.** Lines 60-61 hard-code `M_CAP, Z_MAIN, Z_CAPG = 2.0, 40, 24` and `M_TAKE, Z_BUS, Z_C1, Z_C2, Z_IDL, Z_REELG = 2.0, 40, 24, 24, 24, 16`. These constants are what actually builds the parts: the capstan gear at line 386, c1 at 285, the idler at 415 and the reel gear at 425, plus `idler_xy`.
2. **The spec has moved on.** DECISIONS D20 replaces D3's 40T/24T pair with a 50T/22T pair at a 72.1 centre. D25 changes the take-up to m1.5: bus 50T, c1 46T, idler 30T, reel gear 20T. params.py matches D20 and D25 (`REEL.cap
>

## 11. The dog ring screw counterbore is tangent to the ring OD: zero wall, non-watertight STL, audit static FAIL

- File: `base/cad/parts/drum.py:311`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-drum

Scenario: ring_screw_r 28 + counterbore r 3.0 = 31.0 = ring_od / 2. Each counterbore breaks through the outer cylinder with a wall of zero thickness. The exported mesh has self-touching edges: tools.audit reports 'drum_dogring_launch: not watertight as STL', so the audit is NOT CLEAN and run_all.sh stops. The printed ring has three open notches at the rim, right where the screw heads must bear. I checked it: with ring_screw_r = 27.5 the STL is watertight.

Evidence: drum.py:38 `ring_s=(95.5, 101.0), ring_od=62.0, ring_id=27.0, ring_lug=(6.0, 3.0, 15.0, 21.0), ring_screw_r=28.0,`; drum.py:311 `cuts.append(cyl(3.0, s0 - 1, s0 + 3.2, x, y, seg=32))`. Scratch run: r 28.0 -> is_watertight False, r 27.5 -> True.

Fix: Keep at least 1.2 mm of wall beyond the counterbore. Either set ring_od=64.0 (it matches the sleeve flange OD 64 in BASE_SPEC's interface table) so the wall is 31 + 1.0... better: set ring_screw_r=27.0 with counterbore r 3.05 and ring_od=64.0 (wall 1.95 mm). Change the sleeve's bolt circle to the same radius at the same time (see the launch sleeve finding).

> Skeptic (confirmed): The defect is real. In base/cad/parts/drum.py:38, GEOM sets ring_od=62.0 and ring_screw_r=28.0. spec/params.py does not override either value: G(P) returns 62.0 and 28.0. dog_ring (lines 296-312) cuts the 3.0 mm counterbore at r 28, so the counterbore's outer edge sits at 31.0 = ring_od/2.

The meshes also make the tangency exact. The outer cylinder uses segs(31) = 192 segments, which puts a vertex at angle 0. The counterbore uses seg=32, so it also has a vertex at x = 28 + 3 = 31 on that axis. The two meshes touch at one point and one edge.

I reproduced the audit's own check from tools/audit
>
> Skeptic (confirmed): I could not refute this claim. I checked it and it is real.

**Geometry (base/cad/parts/drum.py)**
- The default geometry is GEOM at line 38: ring_od=62.0 and ring_screw_r=28.0.
- spec/params.py does not override these. I built G(P) with the spec params and it returns ring_screw_r 28.0 and ring_od 62.0.
- In dog_ring() (lines 296-313), the counterbore is `cyl(3.0, s0-1, s0+3.2, x, y, seg=32)` at radius 28. Its outer reach is 28 + 3 = 31.0, which equals the ring's outer radius of 62/2.
- The outer tube has 192 segments (the segs() cap), so it has a vertex at 0, 120 and 240 deg. Each 32-segment 
>

## 12. Carriage spring pegs are d6 PLA cantilevers with no hook locator: SF 1.38 at the modelled hook height, 0.52 once the hook drops onto the keeper flange

- File: `base/cad/parts/launch.py:291`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: Each peg carries one spring: 40.1 N at full cock (2 x (7 + 0.125*265)), on every cock and every shot. The peg hangs from the arm underside (w 64.6) down to a keeper flange at w 51. The hook is modelled at w 60, which is a 4.6 mm lever: M = 184 N mm, Z(d6) = 21.2 mm3, sigma = 8.7 MPa, SF 1.38 against PLA 12 MPa (1.15 against _frame_calc's in-layer 10 MPa). Nothing holds the hook ring at w 60. With the rail pitched 6..47 deg, gravity (g cos p along -w) and the shock of every shot slide both hook rings down. The anchor-pin hook comes to rest on its post top, but the carriage hook rests on the keeper flange (ring centre at w 52.4): lever 12.2 mm, M = 489 N mm, sigma = 23 MPa, SF 0.52. The PLA peg fatigues or creeps at the arm root and snaps. The spring then whips free and the carriage is pulled by one spring only, which twists it and jams it in the rail.

Evidence: launch.py:291 `s.append(cyl_w(3.0, n["w_spring"] - 9, n["lip_w1"] + 1, up, sg * n["y_spring"]))`; launch.py:292 `s.append(cyl_w(4.5, n["w_spring"] - 11, n["w_spring"] - 9, up, sg * n["y_spring"]))   # keeper flange`; launch.py:289 arm from `n["lip_w1"] + 0.5` (64.6); launch.py:432 carriage material "PLA"; measured w_spring 60, lip_w1 64.125.

Fix: Make the peg d10 and put a hook-locating groove at w 60: a peg from w 57 to 65.1 with the keeper flange at w 55..57, so the hook cannot sit lower than w 58.4. Worst-case lever 6.2 mm: M = 249 N mm, Z(d10) = 98 mm3, sigma = 2.5 MPa, SF 4.7. The alternative is a steel M5 screw through the arm as the peg.

> Skeptic (confirmed): I checked the claim against the code and could not refute it. Numbers come from running L(params) in base/cad/parts/launch.py.

Geometry (cyl_w(r, w0, w1, u, y)):
- Peg (line 291): r = 3 (d6), runs from w 51 to 65.125, at u = 6, y = +-38.
- Keeper flange (line 292): r = 4.5, from w 49 to 51.
- Arm (line 290): w 64.625 to 78.1, u 0..8, y 12..43.
- The deck/web blocks only reach |y| < 17.5, so nothing supports the peg except the arm root. It is a free cantilever about 13.6 mm long.
- The peg spans u 3..9, but the arm only goes to u 8, so 1 mm of the root section hangs past the arm and the real r
>
> Skeptic (confirmed): I checked the code and the claim holds up. In L(P), r_big is 50, which gives w_rp 50.1, w_rroot 53.725, shelf_top 53.125, shoe_w1 59.725, lip_w0 60.125 and lip_w1 64.125. w_spring is 60. In carriage_solid (launch.py:289-292) the arm block starts at lip_w1+0.5 = 64.625. The peg is cyl_w(r=3.0) running from w 51 to 65.125, so it is d6. The keeper flange is r 4.5 from w 49 to 51. The only thing the peg is joined to is that arm: the deck is |y|<17.5 and the webs are |y|<17.5, while the peg sits at y=+-38. So the peg is a pure cantilever. Its surface between w 51 and 64.6 is plain, with no groove o
>

## 13. The big gear and the drive gear have symmetric hubs thicker than the face, so the PPA-CF tooth rims print 2 to 2.5 mm above the bed

- File: `base/cad/parts/launch.py:318`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: The big gear hub is 17 mm tall and centred on the 12 mm face. With print_tf T(0,0,2.5), only the r 10 hub annulus touches the bed (230 mm2), and the whole 100 mm sector web and teeth start 2.5 mm in the air: 7168 mm2 of flat overhang under the loaded gear. The drive gear is the same: hub 16 mm on a 12 mm face, teeth 2 mm up, 878 mm2 of overhang. The slicer has to put supports under the tooth flanks of the gears that carry the spring cocking load (about 762 N at the rack). The first layers of the teeth come out drooped or scarred, and support has to be dug out of the tooth gaps.

Evidence: launch.py:318 `hub = prim.cyl(10.0, 17.0, z=-(17.0 - n["face"]) / 2)`; launch.py:327 `hub = prim.cyl(sp["od"] / 2 + 4.0, sp["w"] + 2.0, z=-(sp["w"] + 2.0 - n["face"]) / 2)`; launch.py:436 `print_tf=tf.T(0, 0, (17.0 - n["face"]) / 2)`. print_check: launch_big_gear bed 230.0, overhang 7168.2; launch_drive_gear overhang 878.4.

Fix: Make each hub one-sided, flush with the gear face that goes on the bed: big gear `hub = prim.cyl(10.0, 17.0, z=0)` with print_tf=tf.I4 and place shifted by the same 2.5 mm (or keep the symmetric hub and print_tf the gear on a 2.5 mm sacrificial ring/disc of r 48 below the web). Drive gear: `hub = prim.cyl(sp["od"]/2 + 4.0, sp["w"] + 2.0, z=0)`, with the sprag still centred by moving the bore cut, and its place adjusted.

> Skeptic (confirmed): I checked the code and the claim holds. In base/cad/parts/launch.py:318 the big gear's hub is prim.cyl(10, 17, z=-(17-12)/2), so it runs from z -2.5 to 14.5, while the 12 mm sector-gear extrusion runs from 0 to 12. The print_tf at line 436 is T(0,0,2.5). That puts the hub at 0..17 on the bed and the web and teeth at 2.5..14.5, so only the r10 hub annulus touches the bed and the whole underside of the gear is a flat overhang 2.5 mm up.

The drive gear (line 327) has the same problem. Its hub is 16 mm tall (w+2), centred on the 12 mm face, and runs from -2 to 14. Its print_tf is bed(), which per
>
> Skeptic (confirmed): The claim checks out against the code. launch.py:318: the big gear hub is `prim.cyl(10.0, 17.0, z=-(17.0 - face)/2)` and the face is 12 mm (extruded 0..12). So the hub sticks out 2.5 mm on both sides. print_tf at line 436 is T(0,0,2.5), which puts the hub bottom at z=0 and the whole web and tooth rim at z=2.5. drive_gear_solid (line 327) has the same pattern: hub width sp.w+2 = 16 on a 12 mm face, centred, so it protrudes 2 mm on each side. bed() drops the minimum z to 0, which leaves the gear body 2 mm in the air. Because the hub sticks out on both sides, no flip of either part gets the face 
>

## 14. The launch rail rear and mid pieces print standing on the 6 mm split-flange plates, with the whole side wall in the air

- File: `base/cad/parts/launch.py:429`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: The rail is printed with Rx(90), so its outer wall face (y = -28) should lie on the bed. But the split flange plates stick out 6 mm beyond that face (y 28..34), so each piece rests only on the flange plates. Measured bed contact: rear 108.5 mm2 (one 7 x 15.5 plate), mid 217 mm2. The 150 mm wall, lips and track shelves hang 6 mm above the bed: overhang 10301 mm2 (rear) and 19962 mm2 (mid). The rear piece fails the audit ('no real flat face on the bed (108.5 mm2, need 150)'), and in practice the track surfaces print on supports. Printing Rx(180) (lip tops down) gives 2965 / 3400 mm2 of bed contact and 3314 / 5663 mm2 of overhang, with no fails.

Evidence: launch.py:219-221 `# split flanges: plates on the outer wall faces ...` `s.append(Bs(us - 7.0, us + 7.0, n["y_wall1"] - 0.01, n["y_wall1"] + 6.0, n["w_floor0"] + 2.0, n["w_floor0"] + 17.5))`; launch.py:429 `print_tf=bed(sol, tf.Rx(90)), color=cols[k],`. Audit print_check: rear Rx90 bed 108.5 overhang 10301.2 FAIL; rear Rx180 bed 2965.3 overhang 3314.3; mid Rx90 bed 217.0 overhang 19962.4; mid Rx180 bed 3400.0 overhang 5662.9.

Fix: In parts(): `print_tf=bed(sol, tf.Rx(180) if k in ("rear", "mid") else tf.Rx(90))` (lip tops on the bed). Or move the split flanges under the floor (w_floor0-6..w_floor0, spanning y +-28 like the cross ties) so the outer wall face is flat for Rx(90).

> Skeptic (confirmed): I could not refute this. The defect is real, and I reproduced the numbers.

How the geometry works out:
- Rx(90) maps (x, y, z) to (x, -z, y), so after rotation the new z is the old y.
- bed() in launch.py:140 translates the part so its lowest z sits at 0. That puts the part's most negative y face on the bed.
- In rail_solid, the walls end at |y| = y_wall1 = 28 (launch.py:98). The split flanges at lines 220-221 are Bs(us±7, y 27.99..34, w_floor0+2..w_floor0+17.5), mirrored to both sides.
- For the rear and mid pieces nothing else extends past |y| = 28. The anchor posts at |y| up to 46 are only
>
> Skeptic (confirmed): I could not refute this claim. I checked the code and measured the geometry myself.

The code says what the reviewer quotes:
- launch.py:189 makes the outer wall face at y = ±y_wall1, and launch.py:98 sets y_wall1 = 28.
- launch.py:221 adds the split-flange plates with Bs(us-7, us+7, y_wall1-0.01, y_wall1+6, w_floor0+2, w_floor0+17.5). They are mirrored in y, so they stick out 6 mm past the outer face on both sides (|y| = 28..34).
- launch.py:429 prints every rail piece with bed(sol, tf.Rx(90)). Rx(90) maps y to z. bed() at launch.py:140 lifts the part so its lowest z sits on the bed, and the 
>

## 15. The launch carriage has no flat face to print on: in its print_tf it stands on the 8 x 13.5 end of one spring arm

- File: `base/cad/parts/launch.py:433`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: With print_tf = Rx(90), rz 45, the lowest point is the y = -43 face of the spring arm block, an 8 x 13.5 = 108 mm2 patch. The 263 mm deck, rack, shoes and cradle hang about 20 mm above the bed (7833 mm2 of overhang). The audit fails it ('no real flat face on the bed (108.0 mm2, need 150)'). The rack teeth and shoe faces, which are the sliding and meshing surfaces, would print on supports. No other orientation fits: I = 84.8 mm2, Rx180 = 0 mm2, and Ry(+-90) is 293 mm tall (over the 260 mm bed).

Evidence: launch.py:288-290 `ya, yb = (12.0, n["y_spring"] + 5) if sg > 0 else (-n["y_spring"] - 5, -12.0)` `s.append(B(0, n["pusher_t"], ya, yb, n["lip_w1"] + 0.5, n["lip_w1"] + 14.0))`; launch.py:433 `print_tf=bed(cs, tf.Rx(90), rz=45.0),`. Scratch print_check: Rx90 bed 108.0 FAIL; I 84.8 FAIL; Rx180 0.0 FAIL; Ry90 bbox 293 tall FAIL.

Fix: Make the spring arms and pegs (launch.py:287-295) a separate bolt-on part (2 x M3 into the pusher). Trim the cradle to the shoe width (`cr = cr ^ B(-1, 151, -n["y_shoe1"], n["y_shoe1"], ...)`) so the shoe faces at y = -21.5 form a full-length flat face. Then print_tf = bed(cs, tf.Rx(90), rz=45) puts the 263 mm shoe face on the bed.

> Skeptic (confirmed): I could not refute this; it reproduces exactly. I ran audit.print_check (the real audit function, bed 256x256x260) on launch.carriage_solid(spec/params.py) using launch.bed with the same transforms. Script: /tmp/claude-0/review-scratch-E/a_carr.py.

- Shipped print_tf, bed(cs, Rx(90), rz=45): bed contact 108.0 mm2, need 150, so it FAILs 'no real flat face'. Overhang is 7833.5 mm2 (14.8% of the area), which also triggers the supports warning. Rx(90) maps local y to print z, so the lowest point is min y = -43. That is the spring-arm block B(0, pusher_t, -y_spring-5, -12, lip_w1+0.5, lip_w1+14) f
>
> Skeptic (confirmed): I reproduced the claim with the repo's own checker. The scratch script /tmp/claude-0/review-scratch-E/pc_carriage.py calls tools.audit.print_check on the launch_carriage Part from launch.parts(P), using its actual print_tf and the default bed of 256 x 256 x 260 (audit.py:388). Results:
- Shipped orientation, bed(cs, Rx(90), rz=45): bed_contact 108.0 mm2, so it FAILS 'no real flat face on the bed (108.0 mm2, need 150)'. It also has 7833.5 mm2 of overhang, bbox 212 x 224 x 86.
- Alternatives: I gives 84.8 mm2 (fail), Rx180 gives 0.0 (fail) and Rx-90 gives 108.0 (fail). Ry90 is 292.9 mm tall, ove
>

## 16. Aim splitter and bevel built at m1.5 face 8, which physics says fails at the 5 A trip (sun SF 0.76, idler 0.66, bevel z15 about 0.45; 0.42 at the 5 N m limiter)

- File: `base/cad/parts/launch_aim.py:46`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: In aim, sprag #1 frees, so all of the wiper's backward torque goes sleeve rib -> sun z30 -> idler z18 -> output z30 -> bevel 30:15 -> worm. If the rail or crank is obstructed while aiming (rail hits the frame or ground at low pitch, a stiff printed worm, a hand in the way), the wiper loads up to the 5 A hard trip (3.18 N m, drivetrain.json t_trip) before firmware cuts it, and up to the 5 N m limiter during the 30 ms window. printed_gears.lewis on PPA-CF (25 MPa) at 3.18 N m: sun m1.5 z30 b8 gives Wt 141 N, 32.8 MPa, SF 0.76; idler z18 gives 38.1 MPa, SF 0.66. At 5 N m: SF 0.48 and 0.42. Bevel z15 (virtual z 16.8, mean module 1.26) at 1.59 N m: about 56 MPa, SF about 0.45. The sun also sits on only 8 mm of the sleeve rib: key_torque_capacity(20, 8, 2.6, 15 MPa) = 3.5 N m, so SF 1.1 at the trip and 0.7 at the limiter, and D8 requires at least 20 mm of key engagement. Physics sized this stage explicitly: 'm 2 needed: m 1.5 fails at the 5 A trip' (m2 z30 b10 gives SF 1.69 at the trip). A stripped splitter loses aiming for the rest of the mission.

Evidence: launch_aim.py:46 `M_S = 1.5             # splitter module`; launch_aim.py:48 `FACE_S = 8.0`; launch_aim.py:49 `M_B, Z_B1, Z_B2, FACE_B = 1.5, 30, 15, 8.0`; launch.py:361 `s = s + prim.box(9.0, RIB_R, -RIB_W / 2, RIB_W / 2, z_sun1 - 8.0, z_sun1)` (8 mm rib under the sun); base/physics/drivetrain.py:59 `gear=dict(module=2.0, face=10.0, z_driver=30),   # splitter / bevel stage (m 2 needed: m 1.5 fails at the 5 A trip)`.

Fix: Set M_S = 2.0 and FACE_S = 10.0 (sun z30, idler z18, output z30; idler SF at the trip about 1.4), and M_B = 2.0, FACE_B = 10.0 for the bevel pair. Re-derive cd_si, cd_io and the countershaft position, which follow from A(). Lengthen the sun hub and its rib to 20 mm (z_sun1 - 20.0 .. z_sun1) per D8. That gives 8.7 N m of key capacity, SF 1.7 at the limiter.

> Skeptic (confirmed): I could not refute this claim. Every check I ran supports it.

1. The CAD values are as cited. In base/cad/parts/launch_aim.py, line 46 sets M_S = 1.5, line 48 sets FACE_S = 8.0, and line 49 sets M_B, Z_B1, Z_B2, FACE_B = 1.5, 30, 15, 8.0. The part notes also say m1.5.

2. The physics sizing disagrees with the CAD. base/physics/drivetrain.py:58 sizes the aim stage at module 2.0, face 10, z_driver 30, with the comment "m 2 needed: m 1.5 fails at the 5 A trip". When I ran drivetrain.run(), it gave aim t_trip = 3.18 N m and gear_sf_trip = 1.37, and that result only holds for the m2 b10 stage.

3.
>
> Skeptic (confirmed): I could not refute this claim. The defect is real.

1. The code says what the claim says. base/cad/parts/launch_aim.py lines 46-49 set M_S=1.5, FACE_S=8.0 and the bevel pair to (1.5, 30, 15, 8.0). The sun's rib in launch.py sleeve_solid is `box(..., z_sun1 - 8.0, z_sun1)`, which is 8 mm long.

2. The physics conflicts with the build. The aim stage in base/physics/drivetrain.py uses gear=dict(module=2.0, face=10.0, z_driver=30), commented "m 2 needed: m 1.5 fails at the 5 A trip". FINDINGS.md line 119 says the same: at the 5 A trip the splitter/bevel stage sees 3.2 N m, module 2 face 10 passes 
>

## 17. The aim lug prints standing on the end of its d8 pin (50 mm2), and the audit fails it

- File: `base/cad/parts/launch_aim.py:366`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: Rx(-90) puts the most +y point at z = 0, and that is the tip of the d8 lug pin (it sticks out from y 46.5 to 54). The bracket balances on a 50 mm2 pin end with 1393 mm2 of overhang, and the pin carries the conrod load with its layers stacked on a free-standing stub. The audit reports 'no real flat face on the bed (50.1 mm2, need 87)'. With Rx(90), the 30 x 16 bolting-plate face (y = 28) is on the bed: 462 mm2 of contact, 981 mm2 of overhang, no fail.

Evidence: launch_aim.py:230 `s = s + prim.cylY(PIN_R, yo - 0.01, Y_ROD[1] + 1.0 - ln["Y0"], x=u, z=0.0)`; launch_aim.py:366 `print_tf=bed(lug, tf.Rx(-90)), color="#d97a00",`. Scratch print_check: Rx-90 bed 50.1 FAIL; Rx90 bed 462.0, no fails.

Fix: `print_tf=bed(lug, tf.Rx(90))` (bolting plate face on the bed, pin pointing up).

> Skeptic (confirmed): I couldn't refute this one. It's real.

**What the code does**
- tf.Rx(-90) maps a part's y to -z. bed() then shifts the part so its lowest point sits at z = 0, so the part's most +y point lands on the bed.
- In lug_solid, the d8 pin is `cylY(PIN_R, yo-0.01, Y_ROD[1]+1-Y0)`. It sticks out past the outer plate face (yo = 46.5), so the pin tip is the most +y point.
- So the part rests on the end of the pin, a d8 circle of about 50 mm2.

**What the audit says**
I ran base/tools/audit.print_check on the actual parts_aim(P) lug, using the repo's print_tf and the default bed (256, 256, 260). Script:
>
> Skeptic (confirmed): I tried to refute this and couldn't. The defect is real.

1. Rotation direction. tf.R (base/cad/lib/tf.py) is a standard right-hand rotation, so Rx(-90) maps a point to z' = -y. The point with the largest y ends up lowest, so it lands on the bed.

2. Geometry. lug_solid (launch_aim.py:217-235) adds `prim.cylY(PIN_R, yo - 0.01, Y_ROD[1] + 1.0 - ln["Y0"], x=u, z=0.0)`. That is the d8 pin, sticking out beyond the outer plate face (yo = 46.5). With current params the lug's bounding box is y 28.02..54.0, so the pin tip at y = 54 is the most +y feature. Under Rx(-90) it is the only thing touching th
>

## 18. Audit has no cross-owner gear-mesh check: gears 60+ mm apart or on different modules pass CLEAN

- File: `base/tools/audit.py:1`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-infra

Scenario: BASE_SPEC section 4 says that for a mesh across two owners 'the audit checks the mesh at 1 degree steps'. audit.py checks only solid interference and declared Clearance pairs, and no module declares a cross-owner gear Clearance. A REEL gear and capstan gear 126 mm apart and 130 mm apart in y, or an m1.5 gear against an m2 gear (drum bus vs reel c1), therefore report no error, and the D3 vs D20/D25 mismatches above went unnoticed.

Evidence: `grep -n -i "mesh\|gear\|centre\|center" tools/audit.py` only hits line 1 (`Exact-mesh audit of the assembly`) and the trimesh calls at lines 74 and 99. Nothing compares axis distances, modules or y planes. BASE_SPEC.md:52-53 `A gear mesh across two owners: ... the centre distance comes from params.py LAYOUT; the audit checks the mesh at 1 degree steps.`

Fix: Add a mesh table to audit (from params: REEL 50/22 m2 at 72.1, bus 50/46 m1.5 at 72.1, idler 42/24 m1.5 at 49.6, drive/big 26/40 m2.5 at 82.6, aim worm/wheel at 32.1). For each pair, read the posed world axes of the two parts and fail when the axis distance differs from m(z1+z2)/2 + 0.1 by more than 0.05 or the face y-ranges do not overlap. Also add Clearance(part_a, part_b, 0.0) pairs swept at 1 deg steps.

> Skeptic (confirmed): I could not refute it. The audit really has no cross-owner mesh check.

1. base/tools/audit.py (394 lines) runs six checks: static part checks, joint limits, collisions (exact intersection volume, with Allow), declared Clearance pairs, floating parts, and each module's own checks(). None of them reads centre distances, modules, tooth counts or face y-planes. The refine() step does cap motion at 1 deg per state. But all it then checks is solid overlap, and two gears that are too far apart, or on different y-planes, never overlap. So they pass silently.

2. I searched every Clearance(...) in cad
>
> Skeptic (confirmed): I could not refute this. I read the code and the claim holds.

**The spec promises a check that does not exist.** BASE_SPEC.md:52-53 says that for a gear mesh across two owners "the audit checks the mesh at 1 degree steps". base/tools/audit.py:run() checks only these things:
- static per-part checks
- joint limits
- pairwise intersection volume against Allow entries
- the declared Clearance pairs
- floating parts
- each subsystem's own checks()

Nothing reads module, tooth count, axis distance or face y-planes for any pair of parts.

**No other check covers it.**
- **Clearance declarations:** 
>

## 19. Wrap pinion (m1.25 z10 x0.4) and N20 pinion (m0.8 z10 x0.3) have pointed teeth below the printable tip land

- File: `base/cad/parts/drum.py:446`
- Severity: low (claimed low); unverified (low)
- Owner: cad-drum

Scenario: Printing on the X2D with the 0.4 nozzle: the tip land is 0.27 mm (wrap pinion) and 0.20 mm (N20 pinion), so the slicer drops or blobs the tips. The effective addendum is lost, the contact ratio of the 10T/60T segment mesh and the 10T/120T N20 mesh drops, and both carry the wrap and shift loads. The library's own rule (>= 0.4 mm and >= 0.25 m) is violated, and it warns on every build.

Evidence: Build warnings: `drum.py:446: UserWarning: spur_gear m1.25 z10 shift 0.4: pointed teeth (tip land 0.27 mm, need >= 0.4 mm for the 0.4 nozzle and >= 0.25 m)` and `drum.py:468: ... spur_gear m0.8 z10 shift 0.3: pointed teeth (tip land 0.20 mm ...)`. _drum_kin.py DEFAULTS `pin_z=10, pin_x=0.4` and `n20_z=10, n20_x=0.3, drum_gear_z=120, drum_gear_m=0.8, drum_gear_x=-0.3`.

Fix: Pass tip_short to spur_gear for both pinions so the tip land is >= 0.4 mm, and recompute the centre distance and contact ratio (keep it >= 1.2). Alternatively lower the shift (pin_x 0.25, n20_x 0.15) with matching mate shifts so a_seg = 44.33 and a_n20 = 52.1 stay as the layout uses them.


## 20. The wrap pinion and the N20 pinion have pointed teeth (tip land 0.27 and 0.20 mm) that a 0.4 nozzle cannot print

- File: `base/cad/parts/drum.py:468`
- Severity: low (claimed low); unverified (low)
- Owner: cad-drum

Scenario: Building drum.py emits library warnings: 'spur_gear m1.25 z10 shift 0.4: pointed teeth (tip land 0.27 mm...)' (drum_idler pinion) and 'spur_gear m0.8 z10 shift 0.3: pointed teeth (tip land 0.20 mm, need >= 0.4 mm...)' (drum_n20_pinion). The slicer drops or rounds the tooth tips, which shortens the working depth and contact ratio of the 10T pinions. This is worst on the m0.8 N20 mesh, which carries the drum torque.

Evidence: drum.py:446 `pin = gears.spur_gear(c["seg_m"], c["pin_z"], face, shift=c["pin_x"])...`; drum.py:468 `p = gears.spur_gear(c["drum_gear_m"], c["n20_z"], face, shift=c["n20_x"])`; gears.py:351 `if inf["pointed"] or not land == land or land < MIN_TIP_LAND: warnings.warn(...)`.

Fix: Pass tip shortening so the land is at least 0.4 mm: `gears.spur_gear(c["drum_gear_m"], c["n20_z"], face, shift=c["n20_x"], tip_short=0.25)` and `gears.spur_gear(c["seg_m"], c["pin_z"], face, shift=c["pin_x"], tip_short=0.2)`. Check with spur_info that s_tip >= 0.4, and recheck the centre distances and the cradle cut radius (which uses r_tip).


## 21. Drum gear 120T m0.8 face 8 printed in PLA (drum A): SF 0.77 at the N20 stall it is sized against

- File: `base/cad/parts/drum.py:743`
- Severity: low (claimed medium); 1 of 2 skeptics confirmed
- Owner: cad-drum

Scenario: The N20 is a self-locking worm on a DRV8833 with no current sense (D9), so any jam holds full stall torque until a firmware timeout. Jams come from: the groove-end hard stops at -2 and 351 on a misread code or at boot, a wrap cup snagged on the line, or the fork blocked tip to tip. drum_NOTES gives the drum capacity at stall as 2.16 N m (0.20 N m x 12 x 0.9). Lewis at 2.16 N m on the 120T m0.8 face 8 drum gear (pitch r 48): Wt 45 N, sigma 15.5 MPa, SF 0.77 against PLA 12 MPa. The negative shift x = -0.3 thins the root further. On the same part, the PLA 60T m1.25 face 8 segment gear is at SF 0.88 when the wrap jams. The PPA-CF 10T m0.8 pinion is at about 0.94 (z12 lower bound). Stripping the drum gear leaves the drum, and therefore the collar and the wrap cup, stuck wherever it jammed, possibly with wraps on.

Evidence: drum.py:43 `dgear=(16.0, 24.0), seg=(25.0, 33.0)` (face 8); _drum_kin.py:53 `n20_z=10, n20_x=0.3, drum_gear_z=120, drum_gear_m=0.8, drum_gear_x=-0.3`; drum.py:406 `dg = gears.spur_gear(c["drum_gear_m"], c["drum_gear_z"], ...)` inside drum_a; drum.py:743 `out.append(Part("drum_drum_a", da, "PLA", ...`; drum_NOTES.md section 9 'Drum capacity at 12:1 with 0.9 mesh efficiency: stall 2.16 N m'.

Fix: Print drum_drum_a in PPA-CF (change "PLA" to "PPA-CF" at drum.py:743). That gives drum gear SF 1.61 and segment SF 1.8. Also widen dgear to (14.0, 24.0) (face 10) so the pinion reaches SF about 1.2 at stall, moving the N20 cradle and strap planes to match.

> Skeptic (refuted): The geometry in the claim checks out in substance, but some of the evidence is quoted wrong. drum.py:43 actually reads dgear=(8.0, 16.0), seg=(52.0, 60.0), not (16,24)/(25,33). The drum_a Part is created at drum.py:912 (add("drum_drum_a", drum_a(g,c), "PLA", ...)), not at :743. The gear is built at :485, not :406. Face 8, 120T m0.8 with x=-0.3 in PLA is correct (_drum_kin.py:58). Cap_stall = 0.20*12*0.9 = 2.16 N m is also correct (_drum_kin.py:319). The hard stops are real and so is the lack of a current trip on the N20: the D8 trips are 5 A, and the N20 stalls at about 0.18 A. So the arithmet
>
> Skeptic (confirmed): The core claim holds, but it is weaker than stated and some of its evidence is wrong.

What checks out:
- The drum gear is part of drum_a. drum.py:485 calls gears.spur_gear(drum_gear_m 0.8, drum_gear_z 120, face = dgear[1] - dgear[0] = 8, shift -0.3).
- drum_drum_a is printed in PLA (drum.py:912, add("drum_drum_a", ..., "PLA", ...)).
- The N20 pinion is PPA-CF (drum.py:922).
- drum_NOTES section 9 gives the drum stall capacity as 2.16 N m (_drum_kin.loads: cap_stall = 0.20 x 12 x 0.9).
- I ran the project's own Lewis checker (base/physics/printed_gears.lewis, PLA sigma_lewis = 12 MPa in common
>

## 22. The storage reel prints 'front face down' on its 0.3 mm raised clutch annulus, so the front flange and 608 seat floor hang in the air

- File: `base/cad/parts/reel.py:431`
- Severity: low (claimed low); unverified (low)
- Owner: cad-reel

Scenario: With Rx(180), the lowest face is the clutch friction annulus (r 13..19, raised 0.3 mm). Bed contact is 603 mm2 = pi(19^2 - 13^2). The r 19..32 front flange starts 0.3 mm above the bed, and the front 608 seat floor at depth 7 is an unsupported annulus. The flange's first layer is extruded onto air or squashed onto the bed. Either way the 0.3 mm relief that sets the clutch friction radius is lost or uneven, which changes the slip torque the take-up relies on.

Evidence: reel.py:41 `ANNULUS = (13.0, 19.0, 0.3)   # clutch friction annulus on the spool front face: r_in, r_out, raised`; reel.py:245 `(rf, L - 0.6), (rf - 0.6, L), (ANNULUS[1], L), (ANNULUS[1], L + ANNULUS[2]),`; reel.py:431 `add("reel_spool", sp, "PLA", tf.Rx(180), ...`. print_check: reel_spool bed 603.1, overhang 2972.2.

Fix: Turn the annulus into a recess so the flange face stays the bed face: in the profile use (rf - 0.6, L), (ANNULUS[1], L), (ANNULUS[1], L - 0.3)... i.e. relieve the r 4.6..13 area by 0.3 instead of raising r 13..19 (`(ANNULUS[0], L), (ANNULUS[0], L - 0.3), (4.6, L - 0.3)`), and move Z_D back by 0.3 so the clutch disk still meets the annulus at r 13..19.


## 23. Reel kinematics never turn the capstan from the wiper: REEL/tow states show the drum REEL gear turning while the capstan gear stays at 0

- File: `base/cad/parts/reel.py:509`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-reel

Scenario: TIMELINE 'tow' state: wiper_deg -106.2 -> 3493.8 with the drum in REEL. drum.kin sets drum_out_reel = w (turns with the wiper), but reel.kin sets reel_capstan = S.get('capstan_deg', 0.0) = 0 because no timeline state supplies capstan_deg. The meshing gears therefore move at ratio 0 instead of 2.27. Once the gears are placed in mesh, every tow and reel-in frame shows tooth overlap or a frozen capstan, and the sim/firmware views of line wound in are wrong.

Evidence: reel.py:509 `cap = float(S.get("capstan_deg", 0.0))`. drum.py:853 `"drum_out_reel": w if md in ("REEL", "WRAPPED") else float(S.get("reel_out_deg", w)),`. A grep shows capstan_deg is set only in reel.audit_states (reel.py:570-571), never in TIMELINE or the drum.

Fix: In reel.kin, when the mode is REEL/TOW (or drum_deg is in the REEL zone), derive cap = -(Z_MAIN / Z_CAPG) * wiper_deg + mesh_offset(alpha_main, Z_MAIN, Z_CAPG), using the sign convention in the reel.py:510 comment and the R-frame direction of the main axis. Otherwise hold it at its last value. Alternatively declare Joint('reel_capstan', ..., gear=('drum_out_reel', -50/22, offset)) so the assembly couples the two.

> Skeptic (confirmed): The defect is real. I ran the code to check it rather than relying on the cited lines.

- **What the code does:** reel.kin (base/cad/parts/reel.py:509) sets `reel_capstan` from `S.get('capstan_deg', 0.0)`. The only place `capstan_deg` is ever set is reel.audit_states (line 571). It is not in params.TIMELINE (base/spec/params.py:260-305) or in tools/timeline.py.
- **Nothing else fills it in:** tools/assembly.py:119-145 only falls back to `Joint.gear` when no kin() returns the joint. reel.kin always returns `reel_capstan`, and its Joint declaration (reel.py:470) has no `gear=` anyway.
- **The ru
>
> Skeptic (confirmed): The main defect is real, but the claimed consequences are overstated.

Confirmed in the code:
- reel.py:509 reads `cap = float(S.get("capstan_deg", 0.0))`, and reel_capstan has no `gear=` coupling (reel.py:470).
- The only place that sets capstan_deg is reel.audit_states (reel.py:571). params.TIMELINE (params.py:260-) and tools/timeline.py never set it.
- drum.kin (drum.py:853) turns drum_out_reel with the wiper in REEL/WRAPPED. In 'tow' the wiper goes from -106.2 to 3493.8, and in 'reel_in' it goes on to 4093.8.
- So across the whole winch phase the capstan group (reel_c, reel_gc and the v7 c
>

## Refuted (dropped)

- `base/cad/parts/drum.py:409` In drum A, the segment gear and the brake cam float on the r10 neck over 1 mm and 2 mm air gaps
- `base/cad/parts/drum.py:469` N20 pinion has a round 3.1 bore on a 3 mm D shaft, so it transmits no torque
