# Round 2 I: CAD library and tools pipeline (cad-infra)

Reviewed at base-v1 commit 0bfd59a. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 7. Refuted and dropped: 1.

Re-checked 14:10 Sydney against base-v1 863d58d, one checker subagent per finding: see the Now column.

| # | Sev | File:line | Owner | Finding | Now (863d58d) |
|---|---|---|---|---|---|
| 1 | high | `base/tools/steps.py:64` | cad-infra | plan_steps silently drops every explicit assembly step whose parts are all already used, removing the 'Set the buck BEFORE wiring the DRV8833', 'Launcher 8: time the big gear' and deck-mounting steps from guide.pdf and assembly.html | fixed |
| 2 | medium | `base/tools/audit.py:779` | cad-infra | refine() caps sub-steps at 720 without warning, so a joint can move far more than MAX_STEP_DEG between audited states (wrap phase: reel_cup moves 3.5 deg per step) | fixed |
| 3 | medium | `base/tools/pack.py:112` | cad-infra | pack/bom/guide with --only reuse whatever OUT/asm.json exists and silently produce an empty print pack and BOM with exit 0 | partly fixed |
| 4 | low | `base/cad/lib/gears.py:106` | cad-infra | Internal gear helper skips the interference check for tooth difference >= 10 and crashes on a positive pinion shift; it returns a centre distance where the pinion collides with the ring | fixed |
| 5 | low | `base/cad/lib/worm.py:80` | cad-infra | worm_info self-locking flag uses the axial friction angle atan(mu), not the normal-plane angle atan(mu / cos an), so it contradicts back_efficiency | fixed |
| 6 | low | `base/tools/audit.py:782` | cad-infra | refine() copies non-numeric S keys from the earlier state, so a joint driven by a string or flag key jumps in one unrefined step at the end of the interval | fixed |
| 7 | low | `base/tools/bom.py:93` | cad-infra | BOM and guide parts list use a different mass model than the print pack, under-stating PPA-CF filament by about 20 percent | fixed |

## 1. plan_steps silently drops every explicit assembly step whose parts are all already used, removing the 'Set the buck BEFORE wiring the DRV8833', 'Launcher 8: time the big gear' and deck-mounting steps from guide.pdf and assembly.html

- Now (863d58d): **fixed** at `base/tools/steps.py:98-101,266-277`. plan_steps no longer filters anything out. Every authored step is kept: a step with no parts becomes a text step, and parts already claimed by an earlier step move to a new "handle" list. A scratch run showed a step with no parts ('Set the buck') and a step whose only part was already used ('Dup', handle=['a']) both stay in the output, and guide_steps and build both use this function.

- File: `base/tools/steps.py:64`
- Severity: high (claimed critical); 2 of 2 skeptics confirmed
- Owner: cad-infra

Scenario: Full base export (python -m tools.assembly OUT; python -m tools.guide OUT) gives 45 module steps in asm.json, but guide.pdf has only 37. Ten explicit steps disappear because their parts list is empty or every part in it was already claimed by an earlier step. The dropped steps are: Drill the rover, Deck on the rover, Motor-side cheek, Motor onto the cheek, Shafts and trains, Set the buck BEFORE wiring the DRV8833, Wiring and lid, Launcher 1: print the parts, Launcher 8: time the big gear, and Launcher 9: wiring. pdftotext of the guide has no 'Set the buck', 'time the big gear' or 'Drill the rover'. A builder following the guide wires the MP1584 output to the DRV8833 VIN without first trimming it to 9.0 V (the step text says 'the DRV8833 dies above 10.8 V'), which is hardware damage. The builder also never times the big gear to the rack (launcher HOME) and never bolts the deck to the rover. The only trace is 'note:' lines on stdout, and the tools exit 0. steps.build (assembly.html) uses the same function, so the page loses the same steps.

Evidence: base/tools/steps.py:33-37
            elif n in seen:
                warn.append(f"step '{st.get('title', '')}': part {n} already in an earlier step")
            else:
                parts.append(n)
base/tools/steps.py:64
    return [s for s in steps if s["parts"]], warn
Run output: 'note: step 'Set the buck BEFORE wiring the DRV8833': part frame_pcb_mp1584 already in an earlier step' ... 'guide: .../guide.pdf (37 steps, ...)'. Dropped step text from asm.json: 'Only then wire buck output to DRV8833 VIN (the DRV8833 dies above 10.8 V).'

Fix: Only filter empty auto steps; always keep authored steps: `return [s for s in steps if s["parts"] or not s["auto"]], warn`. Also keep the duplicates visible instead of deleting them. For example, put them in a separate list (`st_out["refit"]=[n for n in st.get('parts',[]) if n in seen]`) that guide.py prints as 'Also handles: ...' and the viewer shows as already-fitted parts. Add a test that asserts len(guide_steps(man)[0]) >= len(man['steps']) and that every authored step title appears in the output.

> Skeptic (confirmed): The claim holds; I could not refute it. In base/tools/steps.py plan_steps, the loop at lines 30-37 removes any part already claimed by an earlier step. Line 64 (`return [s for s in steps if s["parts"]], warn`) then drops every step whose parts list ends up empty, including authored ones (auto=False). guide.guide_steps (tools/guide.py:75) and steps.build (line 72, data["steps"]=steps) both use this output. Neither the guide nor the page reads man['steps'] or asm.steps anywhere else, so the dropped text never shows up. I ran plan_steps on the full Assembly.load(load_params(), None) with a scratc
>
> Skeptic (confirmed): I couldn't refute this. The defect is real. base/tools/steps.py:33-37 leaves out of a step's `parts` any part that an earlier step already claimed. Line 64 then returns `[s for s in steps if s["parts"]]`, which throws away every step with an empty list, authored steps included (auto=False). Only auto steps should be filtered. tools/guide.py:75 (guide_steps) and steps.build (line 72, assembly.html) both call plan_steps, so the guide and the page lose the same steps.

I ran plan_steps on the full Assembly.load(load_params(), None). It returns 37 steps out of 45 authored ones. Exactly the 10 step
>

## 2. refine() caps sub-steps at 720 without warning, so a joint can move far more than MAX_STEP_DEG between audited states (wrap phase: reel_cup moves 3.5 deg per step)

- Now (863d58d): **fixed** at `base/tools/audit.py:1016`. refine() now defaults to cap=100000 and raises ValueError when an interval needs more than that, so it no longer clips silently. The 2520-step wraps 0->7 jump is now refined at 1 deg steps. _run also reports 'refinement capped' errors (audit.py:611). The wraps int target in params is unchanged, but it no longer gets past the audit.

- File: `base/tools/audit.py:779`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-infra

Scenario: Real timeline (P.TIMELINE) in the wrap phase: wraps has the int target 7 (params.py:286). timeline._state_custom steps int targets at mid-phase (timeline.py:171), so between t=29.9 s and t=30.0 s wraps goes 0 -> 7 and reel_cup (kin: -360*wraps) jumps -2520 deg in one sample (I measured this: drum 170.2->171.5, wraps 0->7). refine() works out n = ceil(2520/1.0) = 2520, then n = min(n, 720) clips it, giving 3.5 deg per step. The cup gear reel_gk, which meshes the drum wrap idler, and every other part on joint reel_cup are swept at 3.5 deg steps, and the audit still reports CLEAN for that span. That breaks the workplan rule that gear meshes are checked at 1 deg steps (the MAX_STEP_DEG docstring: 'no rev joint moves more than this between audited states'). Proof: audit.refine on one rev joint with states a=0 and a=2520 returns 721 states with a 3.5 deg step (scratch /tmp/claude-0/review-scratch-I/t3.py).

Evidence: audit.py:778-779
            n = max(n, int(math.ceil(d / lim)))
        n = min(n, cap)
audit.py:765 def refine(..., cap=720)
timeline.py:169-171
                cur[k] = a + (v - a) * e          # floats ease; ints and bools (flags, counts) step at mid-phase
            else:
                cur[k] = v if u >= 0.5 else a
params.py:286  dict(drum_deg=302.0, wraps=7, line_out_m=4.3, ...)

Fix: In refine(), when n > cap, do not clip silently. Either return the list of capped intervals and add each one to rep["limits"]/rep["errors"] in main() ("joint X moves D deg between states k-1,k; audited at D/cap deg steps"), so the verdict becomes NOT CLEAN, or remove the cap for rev/slide joints. In timeline._state_custom, ease numeric counts such as wraps: treat int targets of continuous keys as floats (isinstance(v, (int, float)) and not isinstance(v, bool)) and keep the mid-phase step only for bools and explicit flags. Or have params.TIMELINE write wraps=7.0.

> Skeptic (confirmed): I could not refute this. Every piece checks out:
(1) audit.py:765 has refine(..., cap=720) and line 779 does `n = min(n, cap)`. Nothing records or warns when the cap is hit. main() (audit.py:831-834) calls refine() by default and puts the result straight into run(). Nothing marks the coarser span, so the summary can still print CLEAN.
(2) timeline.py:168-171 eases a target only when it is a float. params.py:286 sets wraps=7, an int, so it falls to the else branch and steps 0 -> 7 at the middle of the 30 s wrap phase. timeline.py:104 confirms that P.TIMELINE goes through _state_custom.
(3) reel
>
> Skeptic (confirmed): I checked the claim against the code and it holds. audit.py:765-779: refine() computes n = ceil(d/lim) for each joint and then does n = min(n, cap) with cap=720. Nothing warns, nothing is recorded in rep, and nothing makes the result NOT CLEAN when the cap clips. The MAX_STEP_DEG docstring (audit.py:47, 'no rev joint moves more than this between audited states') and BASE_SPEC.md:53 ('the audit checks the mesh at 1 degree steps') are therefore not enforced. The reel_cup kinematics are -360*wraps (base/cad/parts/reel.py:664,670). That joint carries reel_k, reel_cup, reel_cup_clip, the cup gear a
>

## 3. pack/bom/guide with --only reuse whatever OUT/asm.json exists and silently produce an empty print pack and BOM with exit 0

- Now (863d58d): **partly fixed** at `base/tools/pack.py:116`. Fixed: load_out (pack.py:128-151) now re-exports when a requested --only subsystem is missing or the file is stale, and raises SystemExit when no printed parts are left, so the empty pack/BOM with exit 0 no longer happens. Still open: the reverse case. An --only export writes a subset asm.json to OUT (assembly.py:220, subsystems=list(self.modules)), and a later run without --only returns early at export_reason line 116-117 ("if not names: return None"). Nothing compares man['subsystems'] with the default set, so that subset is silently treated as the whole base.

- File: `base/tools/pack.py:112`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-infra

Scenario: An OUT that holds a base export (asm.json without the demo subsystem) is run with `python -m tools.pack OUT PACK --only demo`. Reproduced on a copy: prints 'pack: 0 printed parts, 0 plates' and exits 0. It writes an empty print_report.json and PRINT_SHEET, and bom/guide likewise produce empty parts lists. The reverse case also fails: if load_out has to export with --only, it writes a subset asm.json to OUT, and a later run without --only treats that subset as the whole base. Nothing checks that the manifest's subsystems match the request, so the pipeline reports success with missing outputs.

Evidence: base/tools/pack.py:103-113
    fn = os.path.join(out, "asm.json")
    if not os.path.exists(fn):
        ...
        Assembly.load(load_params(), only_list(only)).export(out)
    ...
    names = only_list(only)
    if names:
        man["parts"] = [p for p in man["parts"] if p["subsystem"] in names]
Run: `python -m tools.pack out_copy out_copy/pack_demo --only demo` -> 'pack: 0 printed parts, 0 plates' rc=0

Fix: In load_out, compare the request with man['subsystems']. If any requested name is missing, or no --only is given and man['subsystems'] differs from the default set (subsystem_names() minus SKIP_DEFAULT), re-export the assembly for the request, or raise SystemExit with a clear message. Also raise when the filtered manifest has no parts, so pack/bom/guide exit non-zero instead of writing empty outputs.

> Skeptic (confirmed): The claim holds up in the code. In base/tools/pack.py:100-115, load_out only exports when OUT/asm.json is missing. If the file exists, it loads it and filters parts and steps by only_list(only). It never compares the request with man['subsystems'], even though Assembly writes that field (tools/assembly.py:206). By default the export skips demo (SKIP_DEFAULT = {"demo"}, assembly.py:32 and :101), so a normal base export never contains demo parts. Running the documented usage `tools.pack OUT PACK --only demo` (pack.py docstring; bom.py and guide.py document the same flag) against such an OUT filt
>
> Skeptic (confirmed): The mechanism is real. In base/tools/pack.py:100-115, load_out only calls the assembly export when OUT/asm.json is missing. Otherwise it loads whatever manifest is already there and filters parts and steps by `--only`. It never compares the request with man['subsystems'], which assembly.py:206 does write. pack.run (line 577 onward), bom.py:156 and guide.py:154 all use load_out, and pack.run has no check for an empty part list. So `tools.pack OUT PACK --only demo` against an OUT exported for base writes an empty pack and returns 0 unless something else adds errors. The reverse case is also poss
>

## 4. Internal gear helper skips the interference check for tooth difference >= 10 and crashes on a positive pinion shift; it returns a centre distance where the pinion collides with the ring

- Now (863d58d): **fixed** at `base/cad/lib/gears.py:45 and base/cad/lib/gears.py:116-123`. inv_inverse now raises ValueError when v <= 0. check_internal_pair no longer returns early for diff >= 10: it always computes aw and requires aw >= pa + 2*short. Running it confirms the fix: internal_center_distance(1,30,20,x_pinion=0.15/0.2) raises ValueError (aw 13.0 and 5.8 deg), x_pinion=0.3 raises ValueError instead of ZeroDivisionError, and center_distance(1,10,10,-0.3,-0.3) also raises ValueError.

- File: `base/cad/lib/gears.py:106`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-infra

Scenario: A pinion shifted to avoid undercut, run inside a ring: internal_center_distance(1, 30, 20, x_pinion=0.15). check_internal_pair returns early because diff = 10 >= INTERNAL_MIN_DIFF, so the working pressure angle is never checked. That angle has dropped to 13.0 deg; with x_pinion=0.2 it is 5.8 deg. The helper returns a = 4.72 mm (x_p 0.15) or 4.62 mm (x_p 0.2), which includes the default extra = 0.1. I rebuilt the 2D ring the same way internal_gear does and rotated it through one tooth pitch against spur_profile(1, 20, shift=x_p) at that centre distance. The profiles overlap: 0.0005 mm2 at x_p 0.15 and 0.027 mm2 at x_p 0.2, with min clearance 0. That is tip interference, so printed gears would jam. Scratch scripts are in /tmp/claude-0/review-scratch-I/g3.py. With x_pinion >= 0.25 the call raises ZeroDivisionError instead of a ValueError, because inv(aw) <= 0. The same crash happens in center_distance(1, 10, 10, -0.3, -0.3).

Evidence: gears.py:106-107 `if diff >= INTERNAL_MIN_DIFF:\n        return` (the aw check at 108-114 only runs for diff < 10). gears.py:45-48 `a = (3 * v) ** (1 / 3) if v > 0 else 0.0` ... `a -= f / (math.tan(a) ** 2)` gives a divide by zero when v <= 0. Measured: internal_working_pressure_angle(30,20,0,0.15) = 13.01 deg; internal_center_distance(1,30,20,x_pinion=0.25) raises ZeroDivisionError.

Fix: In inv_inverse, add `if v <= 0: raise ValueError("no real working pressure angle (inv(aw) <= 0): profile shifts too negative for this pair")` before the Newton loop. In check_internal_pair, always compute aw and require `need = pa + 2.0 * max(0, INTERNAL_MIN_DIFF - diff)`: delete the early `return` at lines 106-107 and keep the `aw < need - 1e-9` raise. A pair with diff >= 10 but a negative x_ring - x_pinion, which lowers aw below pa, is then rejected and the caller is told to shift the ring by the same amount (x_ring >= x_pinion).

> Skeptic (confirmed): I could not refute it. The code says what the claim says. In base/cad/lib/gears.py, lines 106-107 return before any working-pressure-angle check whenever z_ring - z_pinion >= 10. So a pair with a negative x_ring - x_pinion is never checked, even though it lowers aw below pa.

I reproduced the numbers myself:
- internal_working_pressure_angle(30,20,0,0.15) = 13.01 deg.
- internal_center_distance(1,30,20,x_pinion=0.15) = 4.722.
- internal_center_distance(..., x_pinion=0.25) raises ZeroDivisionError. center_distance(1,10,10,-0.3,-0.3) does too, because inv_inverse starts at a=0 when v<=0 and then
>
> Skeptic (confirmed): The defect is real, but nothing in the tree triggers it today.

What the code does: in base/cad/lib/gears.py, check_internal_pair returns early at lines 106-107 whenever diff >= INTERNAL_MIN_DIFF (10). So the working-pressure-angle check at lines 108-114 never runs for those pairs, even when x_ring - x_pinion is negative and pulls aw well below 20 deg. internal_center_distance (line 122) depends on this check, and its docstring promises "Raises ValueError for pairs with tip interference". For these pairs it does not.

What I reproduced:
- internal_working_pressure_angle(30,20,0,0.15) = 13.01 d
>

## 5. worm_info self-locking flag uses the axial friction angle atan(mu), not the normal-plane angle atan(mu / cos an), so it contradicts back_efficiency

- Now (863d58d): **fixed** at `base/cad/lib/worm.py:47`. worm_info now sets self_locking = gam <= phin, where phin = atan(mu/cos pa). This is the same angle back_efficiency uses, so the flag is True exactly when back_efficiency <= 0. friction_angle now reports phin, and the axial angle has moved to a separate key, friction_angle_axial.

- File: `base/cad/lib/worm.py:80`
- Severity: low (claimed low); unverified (low)
- Owner: cad-infra

Scenario: worm_info(2, 15.5, starts=2) with mu = MU_PLA 0.25 returns lead_angle 14.47 deg, friction_angle 14.04 deg, self_locking False and back_efficiency -0.029 (cannot back drive). The two fields in the same dict disagree. The standard condition for a 20 deg thread is tan(gamma) < mu / cos(an), with an effective friction angle of 14.90 deg here. A designer reading self_locking would add a brake or a ratchet that the worm does not need, and the reverse can happen if a caller passes an mu where the flags fall the other way.

Evidence: worm.py:74 `phi = math.atan(mu)`; worm.py:75 `phin = math.atan(mu / math.cos(math.radians(pa)))`; worm.py:77 `back = math.tan(gam - phin) / math.tan(gam)`; worm.py:80 `"self_locking": bool(gam < phi)`

Fix: worm.py:80: `"self_locking": bool(gam < phin)`. To keep the reported angle consistent, also report `"friction_angle": math.degrees(phin)` at line 79, or add it as a separate key. The flag is then True exactly when back_efficiency <= 0.


## 6. refine() copies non-numeric S keys from the earlier state, so a joint driven by a string or flag key jumps in one unrefined step at the end of the interval

- Now (863d58d): **fixed** at `base/tools/audit.py:1021 (refine) and base/tools/audit.py:496 (pair-refine sub); base/tools/assembly.py:127`. The defect is fixed. refine() now sets S["_joints"] on every inserted state, interpolated linearly between the endpoint joint values, and the per-pair sub() at audit.py:496 does the same. Assembly.values() uses _joints when it is present instead of calling kin(), so a joint driven by a string or flag key such as 'mode' or 'phase' now moves evenly across the interval instead of jumping at the end.

- File: `base/tools/audit.py:782`
- Severity: low (claimed low); unverified (low)
- Owner: cad-infra

Scenario: A kin() that maps a non-numeric S key to a joint value, such as drum_theta() falling back to S['mode'] (drum.py:1020), reel brake_mm() using S['mode'] (reel.py:648), or launch kin testing S['phase'] (launch.py:587). Take states A (mode 'A', joint 0 mm) and B (mode 'B', joint 20 mm). refine() works out n = 40 from the 20 mm change and adds 39 states, but each one is dict(A) with only the numeric keys interpolated. All 39 keep mode 'A', so the joint stays at 0 and then moves the full 20 mm between the last inserted state and B. The sweep in between is never audited, although refine claims no slide joint moves more than 0.5 mm. Proof: values of the refined list end with [0.0, 0.0, 20.0] over 41 states (scratch /tmp/claude-0/review-scratch-I/t3.py, case 3).

Evidence: audit.py:782-787
            S = dict(A)
            for key, x in B.items():
                y = A.get(key)
                if isinstance(x, (int, float)) and not isinstance(x, bool) and isinstance(y, (int, float)) \
                        and not isinstance(y, bool):
                    S[key] = y + (x - y) * u

Fix: After building the refined list, check it: compute asm.values() for each pair of consecutive refined states. Wherever a rev joint still moves more than max_deg, or a slide joint more than max_mm, between neighbours, interpolate the joint values directly (insert states carrying the joint overrides) or report the jump in rep['limits'] so the audit is NOT CLEAN. Do not insert states that cannot move the joint.


## 7. BOM and guide parts list use a different mass model than the print pack, under-stating PPA-CF filament by about 20 percent

- Now (863d58d): **fixed** at `base/tools/bom.py:91-108`. bom.build now takes out and computes each printed part's mass with pack.print_mass_g (PROFILE walls and infill) on the mesh area. This is the same call pack.py:641 uses for the print sheet. bom.run and guide.py:155 both pass out, so the BOM, the guide's 'Printed mass' line and the print pack now use the same model. The manifest mass_g is used only when out is None, which happens in unit tests with no meshes.

- File: `base/tools/bom.py:93`
- Severity: low (claimed low); unverified (low)
- Owner: cad-infra

Scenario: Full base run: BOM.md and the guide's 'Printed mass' line say PPA-CF 791 g (263 m) and PLA 2803 g. The print sheet and the guide's print list (PROFILE: PPA-CF 6 walls and 60 % infill) say PPA-CF 982 g and PLA 3031 g for the same parts. The BOM uses Part.mass_g (a 1.6 mm shell and Part.infill, which defaults to 0.4 for every material), not pack.PROFILE. Anyone buying filament from the BOM is short about 190 g of PPA-CF, and one guide shows two contradicting totals.

Evidence: base/tools/bom.py:93  m = p.get("mass_g") or 0.0
base/cad/lib/core.py:73  infill: float = 0.4 ; core.py:95 def mass_g(self, shell_mm: float = 1.6)
base/tools/pack.py:56  "PPA-CF": dict(walls=6, top=6, bottom=6, infill=0.60)
Run: bom 'PPA-CF: 791 g, 263.0 m filament' vs pack 'PPA-CF: 40 objects on 3 plates, 982 g'

Fix: In bom.build, compute printed mass with pack.print_mass_g(p['volume_mm3'], part_mesh(out, p['name']).area, p['material']). Do this either by passing out into build, or by reading PACK/print_report.json masses when present. Then BOM, guide and print sheet share the PROFILE table.


## Refuted (dropped)

- `base/tools/pack.py:602` Non-watertight STLs (non-manifold edges) go into the print pack as a warning only and pack exits 0
