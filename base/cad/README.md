# base/cad: shared CAD library and pipeline (API v1)

Owner: worker `cad-infra` (`base/cad/lib/`, `base/tools/`, this file). Subsystem workers own
`base/cad/parts/<subsystem>.py` (launch, reel, drum, frame). API v1 is frozen: only additive changes from here on.

Units: mm, degrees, kg, N. World frame (binding, `base/spec/BASE_SPEC.md` section 2): right-handed, origin ON the
wiper output / main shaft axis (x = 0, z = 0 on the axis), y = 0 at the motor adapter plate face. +X forward (launch
azimuth), +Y left looking forward, +Z up. The main shaft axis is the Y axis. The deck top sits at
z = -(axis height) from `P.LAYOUT` (v1.0). Rail pitch is measured up from +X, drum 0 = LAUNCH, wiper + = FORWARD
(cock direction). All solids are `manifold3d.Manifold`.

## 1. Imports

Run everything from `base/` (the tools put `base/` on `sys.path`):

```python
from cad.lib import Part, Joint, Allow, Clearance          # core dataclasses
from cad.lib import tf                                    # 4x4 helpers: tf.T, tf.Rx, tf.Ry, tf.Rz, tf.R(axis, deg), tf.mul
from cad.lib import prim                                  # cyl, cylX, cylY, box, tube, union, extrude, revolve, chamfer helpers
from cad.lib import gears, worm, screw, clutch, bearings, shafts, fasteners, clips, switch, motors, line, springs, cams
```

`P` is the params module: `base/spec/params.py` once the coordinator ships it, else `base/tools/params_stub.py`.
Get it with `from tools.params import load_params; P = load_params()`. Always read numbers with defaults while the
spec is moving: `getattr(P, "DRUM_R", 30.0)`.

## 2. Core objects (`cad/lib/core.py`)

```python
@dataclass
class Joint:
    name: str                       # globally unique, prefix with your subsystem: "reel_spool"
    parent: str | None = None       # parent joint name (any subsystem) or None = world
    kind: str = "fixed"             # "rev" (value in deg about axis), "slide" (value in mm along axis), "fixed"
    origin: tuple = (0, 0, 0)       # joint origin in the parent joint frame (mm)
    axis: tuple = (0, 0, 1)         # unit axis in the frame after origin and base (= parent frame if no base)
    base: np.ndarray | None = None  # optional extra 4x4 applied after origin, before the motion (frame orientation)
    limits: tuple | None = None     # (lo, hi) of the value, checked by the audit
    gear: tuple | None = None       # optional (other_joint, ratio, offset): value = ratio * value(other) + offset
                                    # used when kin() does not give a value

@dataclass
class Part:
    name: str                       # globally unique snake_case, prefix with your subsystem: "reel_spool"
    solid: Manifold                 # in the part's local frame (mm)
    material: str = "PLA"           # "PLA", "PPA-CF", "TPU", "steel", "bought"
    qty: int = 1                    # identical copies printed/bought (the model places ONE; place copies as own Parts
                                    # with names name_2 ... and qty 1, or keep qty for spares only)
    joint: str | None = None        # joint the part rides on, None = fixed to world
    place: np.ndarray = I4          # 4x4: part local -> joint frame
    print_tf: np.ndarray | None = None  # 4x4: part local -> print bed, flat face down at z = 0. None for bought/steel.
                                    # tools.audit checks it: min z == 0, inside 256 x 256 x 260
    color: str = "#c8ccd2"
    subsystem: str = ""             # filled in by the assembly from the module name
    notes: str = ""                 # one line: what it is and why (shows in BOM, guide, sim tooltips)
    bought_ref: str = ""            # bought items, ";" separated, optional count "<n> x <item>" (multiplied by qty):
                                    # "608ZZ 8x22x7", "4 x M3x10 SHCS; 4 x M3 heat-set insert", "2xM3x8 BHCS".
                                    # A plain number after the x is a size ("8 x 120 mm rod" = 1 rod).
    infill: float = 0.4             # for the mass estimate
    step: int | None = None         # assembly step number (assembly page and guide), None = auto by subsystem
```

World pose of a part in state `S`: `M_world(joint) @ part.place`, where
`M_world(j) = M_world(parent) @ T(origin) @ base @ motion(value)`.

## 3. Subsystem module contract (`base/cad/parts/<name>.py`)

Only modules whose name has no underscore are subsystems (`launch.py`, `reel.py`, `drum.py`, `frame.py`, `demo.py`).
Put helpers in `<subsystem>_*.py` (for example `launch_kin.py`): the assembly never imports them directly. Other
files in `cad/parts/` (NOTES.md, json, npz) are ignored.

Required:

```python
def parts(P) -> list[Part]: ...
def joints(P) -> list[Joint]: ...
def kin(P, S: dict) -> dict[str, float]: ...   # joint name -> value for master state S; missing joints use
                                               # Joint.gear, else 0
```

Optional (picked up if present):

```python
def allowed(P) -> list[Allow]: ...            # contacts that are meant to be there (press fits, shaft in bore)
def clearances(P) -> list[Clearance]: ...     # named pairs that must keep a gap (gear tips, rotating vs fixed)
def audit_states(P) -> list[dict]: ...        # extra states S to audit (joint sweeps, fault states)
def checks(P, asm) -> list[str]: ...          # own extra checks, return failure strings (empty = pass)
def steps(P) -> list[dict]: ...               # assembly steps: {"title", "text", "parts": [names]}
def line_path(P, S) -> list[tuple]: ...       # tether polyline in WORLD mm for state S (exit eye, pulleys, dart
                                              # tail); the sim draws it; the reel module starts it, launch continues
def grounded(P) -> list[str]: ...             # parts (fnmatch patterns) that rest on the rover; every other part must
                                              # connect to one through contacts in the rest state
```

`Allow(a, b, why, max_vol=None)`: pair (part names, order free, `fnmatch` wildcards allowed, e.g.
`Allow("reel_shaft", "reel_brg_*", "608 press fit")`). `max_vol` (mm3) caps the overlap; None = any overlap is fine
but it is still listed in the report.
`Clearance(a, b, min_mm, why)`: the audit reports the minimum gap over all sampled states and fails below `min_mm`.

## 4. Master state S

A plain dict. Keys every subsystem may read:

| key | unit | meaning |
|---|---|---|
| `t` | s | mission time |
| `wiper_deg` | deg | wiper output shaft, multi-turn, + = motor forward |
| `drum_deg` | deg | selector drum angle |
| `line_out_m` | m | tether paid out |
| `mode` | str | "LAUNCH", "NEUTRAL", "REEL", "WRAPPED" (informative; drum_deg is the truth) |
| `aim_deg`, `cock`, `fired`, `wraps`, ... | | named free coordinates, defined in BASE_SPEC.md |

`tools/timeline.py` builds the mission as a list of states (`timeline(P, dt) -> list[dict]`) with phase labels
(`S["phase"]`). Your `kin()` must accept any S (use `S.get(key, default)`).

## 5. Pipeline (`base/tools/`)

| tool | what |
|---|---|
| `assembly.py` | `Assembly.load(P, names=None)` imports every `cad/parts/*.py`; `asm.world(S)` -> {part: 4x4}; `asm.posed(name, S)`; `asm.export(out)` writes `out/asm.json` and `out/meshes/<part>.npz` (local frame) |
| `audit.py OUT` | static checks per part (manifold, watertight, one shell, print fit and flat face, overhang, volume, mass), pairwise exact intersection over the timeline + audit states (AABB broad phase), allowed contacts, clearances, floating parts, joint limits. Writes `OUT/audit.json`, prints CLEAN or NOT CLEAN |
| `render.py OUT` | 24-angle sheet + section cuts (headless Chromium, three.js) -> `OUT/audit_sheet.png`, `OUT/sections.png` |
| `sim.py OUT` | HD sim page from the manifest + per-part keyframe tracks (real motion) -> `OUT/sim.html` |
| `steps.py OUT` | assembly steps page -> `OUT/assembly.html` |
| `pack.py OUT PACK` | STL per printed part in print orientation, 3MF plates for the 256 bed, PRINT_SHEET |
| `step_export.py OUT` | STEP per part (faceted) |
| `bom.py OUT` | BOM (csv + md) |
| `guide.py OUT` | build guide PDF (Liberation Sans, plain black) |
| `run_all.sh` (in `base/`) | the whole chain |

Command line: `python -m tools.<tool> OUT [--only demo]` from `base/`.

## 6. Rules the audit enforces (Harry's standards)

- Every part: valid manifold, one shell (printed parts), no part floating (touches another part or the world within
  0.2 mm in the rest state), nothing inside something else (only listed `Allow` pairs may overlap).
- Printed parts fit the Bambu X2D bed (256 x 256 x 260) in `print_tf`, sit flat at z = 0.
- No em dashes in any text.

## 7. Library summary (`cad/lib/`)

Each function returns a `Manifold` in a documented local frame (axis +Z unless stated), or an info dict. Conventions:
cutters (seats, holes, bores) have the mouth at z = 0 and cut into -Z (with 1 mm overshoot above); bought-part models
(bearings, nuts) sit with the top face at z = 0 so the identity places them in their seat; screws have the head
underside at z = 0. Fit allowances live in `cad/lib/fits.py` (one place to tune after Harry prints the fit coupon).
Placeholder (unmeasured) numbers carry an `_ASSUMED` suffix or sit in `motors.WIPER_DEFAULTS`.

| module | public functions |
|---|---|
| `gears` | `bore_cs`, `center_distance`, `contact_ratio`, `cut_bore`, `internal_center_distance`, `internal_gear`, `internal_info`, `inv`, `inv_inverse`, `lewis_max_torque`, `lewis_stress`, `lewis_y`, `rack`, `rack_center_distance`, `rack_profile`, `spur_gear`, `spur_info`, `spur_profile`, `tip_chamfer`, `working_pressure_angle` |
| `worm` | `worm`, `worm_center_distance`, `worm_info`, `worm_pitch_d`, `worm_pose`, `worm_wheel` |
| `screw` | `helix_extrude`, `iso_crest_clearance`, `nut`, `nut_hole`, `screw`, `screw_info`, `thread_cs` |
| `cams` | `barrel_cam`, `barrel_info`, `follower_pin`, `groove_cutter`, `track_fn`, `track_z` |
| `bearings` | `bearing_608`, `bearing_608_inner_race`, `bearing_693`, `bearing_csk20`, `bearing_csk20_inner_race`, `bearing_f693`, `boss_608`, `boss_693`, `boss_csk20`, `ribbed_seat`, `seat_608`, `seat_693`, `seat_boss`, `seat_csk20`, `seat_f693` |
| `shafts` | `d_bore`, `d_shaft`, `hex_bore`, `hex_shaft`, `journal`, `journal_608`, `key_torque_capacity`, `keyed_bore`, `keyed_shaft`, `shaft_torque_capacity`, `stepped_shaft` |
| `fasteners` | `m3_clear`, `m3_close`, `m3_counterbore`, `m3_countersunk`, `m3_insert`, `m3_insert_model`, `m3_nut`, `m3_nut_trap`, `m3_nyloc_trap`, `m3_shcs`, `m3_tap`, `m8_bolt`, `m8_clear`, `m8_nut`, `m8_nut_trap`, `nut_trap`, `round_hole` |
| `clutch` | `dog_clutch`, `dog_mate`, `flexure_force`, `flexure_ring`, `flexure_stress`, `slip_clutch`, `slip_torque` |
| `clips` | `c_clip`, `clip_groove_cutter`, `clip_info`, `clip_pin`, `clip_pin_clip_z`, `groove_dims`, `pin_hole_cutter`, `snap_pin`, `snap_pin_info` |
| `switch` | `contact_switch`, `lever_pose` |
| `motors` | `n20_mount`, `n20_straight`, `n20_worm`, `n20_worm_mount`, `wiper_coupler`, `wiper_coupler_info`, `wiper_coupler_z`, `wiper_motor_envelope`, `wiper_plate` |
| `line` | `eye_boss`, `eye_cutter`, `eye_info`, `fairlead`, `fairlead_cutter`, `line_hits`, `line_length`, `line_tube` |
| `springs` | `anchor_post`, `anchor_post_info`, `extension_spring_model`, `spring_force`, `spring_info` |

## 8. Demo

`cad/parts/demo.py`: two meshing gears on keyed shafts in 608 seats on a small plate. Not part of the base (the
assembly skips it unless `--only demo`). Run `bash run_all.sh demo` to see the whole chain.
