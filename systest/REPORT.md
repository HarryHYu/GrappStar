# systest: end-to-end system test (worker `systest`, 3 Nov 2026)

Status: FINAL, pushed 19:35 Sydney. The worker process was restarted by the platform at about 15:25 and came back
only at 19:21, so the planned 18:20 final push and the last full batch did not happen; everything below is from the
runs finished before 15:25 plus a short wrap-up at 19:21.

Final cross-check update (3 Nov, base-1.3.1): every scenario was run again on this tree (results under "Final
cross-check run" below): 47 of 55 pass, among them the nominal mission, the refusals and every power-loss phase that
the drivers reach. The 8 failures are F21 (stiff 3 m tow, unchanged), the new F32 (index-mode tow), and harness
scenarios that do not reach their own trigger or check (listed there).

## Summary (for Harry)

- A whole GrappStar mission runs end to end with the real base firmware logic, the real dart firmware and the real
  Jetson code talking over their real links (serial pty, HTTP), no hardware. On the owners' latest branches the
  nominal mission and the BASE_SPEC 10.1 sequence complete with every check on plant truth.
- 31 findings went to the owners during the day (F1 to F31). Most are fixed on the owners' branches and re-tested
  green here (status table at the top of FINDINGS.md). The ones that mattered most: the wrap jamming against an
  anchored line in the old order (F4), the REEL fly-in tripping on a taut real line (F20, fixed as D41), the tow
  overshooting the limit on a stiff line (F21), settings lost at a power cycle (F16), power cuts leaving the
  launcher in a state nothing could clear (F15, F17), a calibration procedure that towed 10 N over the limit (F14).
- Open now: F21 (the stiff 3 m tow overshoots the +15 % bound at duty 1), F32 (new: an index-mode build cannot tow)
  and low-severity items. F22, F24, F25, F28 and F31 were fixed after the final batch and are green in the final
  cross-check run.

## What it is

`systest/run_mission.sh [scenario|all|--list]` runs a whole GrappStar mission with the REAL code of all three parts
talking to each other over their real interfaces, no hardware:

| Part | What runs | Link |
|---|---|---|
| base | `systest/build/base_shim`: `firmware/base/grappstar_base/base_logic.h` (the header the sketch compiles) + the plant model `firmware/base/test/sim.h`, wired like `grappstar_base.ino` (5 ms tick, reply before events, 24-line event ring, telemetry dropped when the port is full, NVS writes 20 ms with saveAck) | C1 JSON lines over a pseudo-terminal (pyserial opens it like `/dev/ttyACM0`) |
| dart | `firmware/dart/sim/kedge_sim`: the unchanged `kedge_dart.ino` on stub Arduino headers, plant and IMU traces | C2 HTTP on 127.0.0.1 |
| Jetson | `jetson/grappstar` App: `comms/base.py`, `comms/dart.py`, `mission.py`, target vision on a synthetic rover view (orange cone), landing analysis on a synthetic dart flight video (`jetson/grappstar/sim/flight_video.py`) | |

The harness (`systest/harness.py`) is the outside world and the operator: when the base plant releases the carriage
the dart sim plays a launch and impact trace and the dart video plays the rendered flight; ANCHORED makes the line
taut on the base plant; the release frees it; the rover hook drives toward the dart during the tow; a scripted
operator answers every prompt. Faults go in through the shim control socket (drum jam, tooth on tooth, stall load,
AS5600 missing, brownout, snag, USB unplug / glitch, raw garbage on the serial line), the dart `/sim/set` knobs, or
by SIGKILL and restart of the base process (power loss: the plant and what reached NVS survive).

Every check uses plant truth (line tension, wraps, seated, collar engaged, dry fires) as well as events and telemetry.

Relation to `jetson/tools/basesim` + `jetson/tests/test_mission_realsim.py` (jetson worker): those test the mission
with the real base logic and mostly the mock dart, with fake cameras and a fake analysis, inside pytest. This harness is
standalone, runs the whole App (vision, analysis, recording) with the real dart firmware sim, and adds power loss
with NVS persistence, USB unplug / replug, raw serial injection and per-scenario fault triggers, which basesim does
not have. Nothing under jetson/ or firmware/ was changed.

## How to run

```
systest/run_mission.sh --list            all scenarios
systest/run_mission.sh                   nominal mission (about 2 min)
systest/run_mission.sh all               everything (about 1.5 h)
systest/run_mission.sh power_wrap,ref_power_wrap
systest/preview.sh [scenario|all]        same, on base-v1 + w/fw-base + w/fw-dart + w/jetson (before a merge)
python3 systest/c1_order.py              the C1 command order each recorded run used, against BASE_SPEC 10.1
```
Each run leaves `systest/runs/<scenario>/`: `timeline.jsonl` (steps, prompts and answers, events, dart phases, plant
snapshots, injections, checks), `summary.json`, `base.log` (every serial line both ways with sim time), `dart.log`,
`jetson/<run>/mission.jsonl` (the App's own log) and recorded videos.

## Scenarios

Two drivers: `mission` (the real `jetson/grappstar/mission.py` through the App, with a scripted operator) and `ref`
(BASE_SPEC 10.1 step by step with the Jetson's own `BaseClient` / `Dart` clients, plus operator-style recovery). The
`ref_*` scenarios test the firmware and dart against the binding sequence even while the mission lags behind it.

Plant: sim.h plus the D22 additions in the shim: every laid turn draws a capstan circumference from the dart side
(a free dart is tugged, an anchored one jams the cup), and tidy pulls an anchored line taut. Strict for every
scenario since the final cross-check (F4 and F12 fixed); `SYSTEST_LENIENT=1` switches the mission fault scenarios back
to the lenient plant.

Checks per scenario (plant truth where possible): mission DONE, aim reached, fired, landed, anchored, tidy ran,
wrapped (plant wraps), REEL engaged and pulling, tow limit respected (peak plant tension per 5 ms tick, limit +15 %),
dart released, reeled in, unwrapped, seated (plant and SW_SEATED), back to LAUNCH, no dry fire (the plant counts the
collar leaving LAUNCH with the spring drawn), wraps laid without jamming, free dart tugged under 1 m, C1 command order
= BASE_SPEC 10.1, every injected fault actually fired, plus scenario specific ones.

## What Harry must know or measure (from this test)

- The tension limit is only as good as the bench calibration (CALIBRATION.md two-point fit). Without a load cell,
  do it before the first tow; re-check after any change to the line path (the path gain, D46).
- The real line stiffness matters: all tow and fly-in numbers here assume 1 mm Dyneema (EA about 20 kN). If the
  braid is softer at first (bedding in), the overshoots are smaller; if a stiffer line is used, re-run
  `systest/run_mission.sh ref_tow_stiff` with `line_ea_n` raised.
- Not tested here (no model): the real motor and gear friction, the clutch slip torque, the drum detent feel, the
  camera optics. The plant is the firmware tests' sim.h plus the D22 line model added in the shim.

## Results

### Final cross-check run (3 Nov, base-1.3.1, this tree)

All 55 scenarios, `harness.py --scenario all --shard i/2` (the two halves of `systest/run_mission.sh all`, base at 2x),
on firmware base-1.3.1 (the CAD contact windows in sim.h: SW_GAP closed within 22.9 wiper deg of rest, SW_RAIL_LOW
within 97.6 aim deg of the low dead point, SW_SEATED only with the carriage home; `aim_home_deg` 3502, `gap_close_deg`
531, the seated latch), the Jetson fixes of the cross-check, and every scenario on the strict D22 plant. The 24 `ref_*`
scenarios and `nogo_reshoot` ran again after the harness fixes below (the mission scenarios did not change).

```
PASS nominal, nominal_uncal (F28: TOW refused without calibration), dog_tip, drum_timeout, stall_cock, hand_lever,
     dart_never_anchors, dart_drags20 (designed abort), no_angle_sensor (designed refusal), dart_wifi_drop,
     encoder_glitch, serial_garbage, land_short, brownout_tow, usb_unplug_tow, nogo_reshoot,
     power_launch_mode, power_aim, power_cock, power_base_neutral, power_wrap, power_check, power_tow, power_unwrap,
     power_unwrap_fasthello, power_launch_ready,
     ref_nominal, ref_refusals, ref_tow_duty08, ref_drum_timeout, ref_dog_tip, ref_stall_cock, ref_usb_glitch_reel,
     ref_serial_garbage, ref_land_short, ref_dart_wifi_drop, ref_brownout_tow, ref_encoder_glitch,
     ref_power_launch_mode, ref_power_aim, ref_power_cock, ref_power_wrap, ref_power_tow, ref_power_retrieve,
     ref_power_unwrap, ref_power_unwrap_fasthello, ref_power_launch_ready
FAIL tow_stiff, ref_tow_stiff ... tow limit: 46.2 / 50.3 N dart-side on the 3 m stiff line vs the 46.0 bound (F21,
                                   unchanged)
FAIL no_magnet_index ........... F32 (new): index mode now leaves LAUNCH (F31 fixed), then the CHECK pull faults
                                   ANGLE_SENSOR "SW_INDEX bump overdue"
FAIL power_retrieve, snag_retrieve, usb_glitch_reel ... their RETRIEVE trigger never fires: the mission's line estimate
                                   is short (F29) and the base's line_m is about 1.6 m after the first tow round, so
                                   the capstan reel-in is skipped (mission DONE; the retrieve phase is covered by
                                   ref_power_retrieve and ref_usb_glitch_reel, which pass)
FAIL ref_snag_retrieve ......... the snag on the soft 40 N/m line never reaches the 30 N reel-in limit in RETRIEVE
FAIL ref_usb_unplug_tow ........ "reeled in to the approach": after the power loss in the tow `line_m` is null (by
                                   design, PROTOCOL 9), so the reference driver skips the capstan reel-in and the
                                   take-up brings the dart in
```

Harness changes in this round: the C1 check requires step 14 as BASE_SPEC 10.1 has it (`stop > dart release`) and the
reference driver stops first (F25); the reference driver sends `stop` and waits for `ABORTED` before every tow round
after the first (it read the telemetry right after `LIMIT`, still REEL, and re-sent `reel` from HOLD: the F22 case);
`nogo_reshoot` judges the C1 order from the aim of the last shot (the D45 low aim after it is an `aim` too); the strict
D22 plant is the default; `c1_order.py` runs again (it passes the scenario and the run summary to the check).

### Last full batch (batch 5, real time, 3 parallel shards, 38 of 55 scenarios finished before the worker stopped)

Trees: base-v1 279d4a4 + w/fw-base ecc413f + w/fw-dart 3cf8bfb + w/jetson 086749a (`PREVIEW=1 systest/batch.sh`).
Every mission reached DONE except the designed aborts (`dart_drags20`, `no_angle_sensor`) and `no_magnet_index`
(F31). Per scenario (checks that failed):

```
PASS ref_nominal, ref_refusals, ref_tow_duty08, dart_never_anchors, dart_drags20, no_angle_sensor,
     power_launch_mode, power_aim, power_cock, power_base_neutral, power_wrap, power_check, power_tow, power_launch_ready
FAIL nominal ............... reeled in to the approach (*)
FAIL nominal_uncal ......... TOW refused without calibration (F28: the guard no longer fires)
FAIL no_magnet_index ....... ABORTED after the shot: mode NEUTRAL LAUNCH_BUSY in index mode (F31)
FAIL ref_tow_stiff ......... tow limit respected: 46.2 N dart-side vs 46.0 bound (F21 nearly fixed; fw-base 18ab2b2
                             re-test below)
FAIL power_retrieve, snag_retrieve, usb_glitch_reel ... the retrieve-phase trigger never fired (the capstan reel-in
                             was skipped because line_m was already short: harness trigger, see (*))
FAIL dog_tip, drum_timeout, hand_lever, land_short, dart_wifi_drop, serial_garbage, stall_cock, usb_unplug_tow,
     power_unwrap, power_unwrap_fasthello, tow_stiff, brownout_tow, nogo_reshoot, ref_dog_tip, ref_drum_timeout,
     ref_power_unwrap, ref_power_unwrap_fasthello
                         ... only "LIMIT reports the dart-side tension" (**) and/or "reeled in to the approach" (*)
```

(*) "reeled in to the approach" in that batch compared the plant line out with 1.5 m; it failed by the landing
estimate error (F29: Jetson range 11 to 27 % short, partly the sim renderer vs the C3 table). It was re-defined during
the batch (pushed): step 15 is now judged on the base's own `line_m` at the stop and no seating on the capstan, and the
estimate error is reported as information.
(**) the failing samples were the F22 case: the mission re-sends `reel` from HOLD after the rover made slack and
the base answers `LIMIT 40 "already at the limit"` on a 19 N line. Re-classified during the batch (pushed): the
accuracy check skips those replies and the new check "tow rounds take up the rover's slack" flags F22 instead.

### Last re-tests on fw-base 18ab2b2 (base-v1 0c72153, jetson 3e57364, path gain 1.30 from params_gen.h)

```
ref_nominal   PASS  (tow 38.6 N dart-side at limit 40; LIMIT vs plant 37.6/38.6, 27.3/31.1, 31.8/37.1)
tow_stiff     FAIL  tow limit: 46.2 N dart-side on the 3 m stiff line (bound 46.0)
ref_tow_stiff FAIL  tow limit: 50.4 N dart-side at duty 1 on the 3 m stiff line (bound 46.0)
```
The stiff short tow (F21) is improved but still over the +15 % bound at duty 1; fw-base asked for these numbers and
for a re-test of F30 (15 N check pull) on 18ab2b2, which did not happen before the worker stopped.

### Earlier milestones (evidence in the commit history of systest/FINDINGS.md)

- 12:20 first end-to-end mission DONE (base-v1 then): base + dart sim + Jetson App over pty and HTTP.
- 13:40 strict D22 plant: the pre-D22 mission order jams the wrap (F4); the 10.1 reference driver passes.
- 14:10 R2-L review fixes; the stiff Dyneema line reveals the fly-in trip (F20, fixed as D41) and the tow overshoot (F21).
- 14:40 preview of the owners' branches: all power-loss phases recover; F4 to F8, F10, F12 to F21 green or fixed.

## Findings

See `systest/FINDINGS.md` (F1 onward), routed by the coordinator.

## Review R2-L (review/r2_L_systest.md): what changed

| # | Fix |
|---|---|
| 1 | Anchored line = 1 mm Dyneema, `anchorK = EA / d` with EA 20 kN (`line_ea_n`), re-sent whenever the anchor point moves (rover, snag). Strict in `nominal`, `ref_nominal`, `ref_refusals`, `tow_stiff`, `ref_tow_stiff`; the fault scenarios keep sim.h's 40 N/m until F20 is fixed (`SYSTEST_STRICT_ALL=1` for all). It found F20 at once |
| 2 | `c_aim` uses the plant's real rail pitch (`pitchTrue`: the plant aim phase through the factory aim table, exported by the shim) at the AIMED event; the shot's line out is set from the range at the real pitch |
| 3 | `c_reeled` is two-sided (approach_m - 0.4 .. + 0.3 at UNWRAP, not seated), new `c_seat_neutral` (the plant records collar and wraps when SW_SEATED first closes: must be out of REEL, wraps 0); `retrieve` mandatory in the C1 check unless the unwrap take-up already seated the dart in NEUTRAL |
| 4 | Scenarios without injected faults also check: no FAULT event, no failure prompt, no Retry |
| 5 | Drag / creep anchor: `slipN` (the anchor point follows the line when the tension passes it). `dart_never_anchors` drags at 6 N, new `dart_drags20` (above the 15 N check pull) |
| 6, 7 | Shim TX buffer 2048 bytes like `Serial.setTxBufferSize(2048)` (FIONREAD on the pty), replies wait up to 1 s, events 20 ms, telemetry dropped (`telDrops` in `get`); event ring 23 |
| 8 | Accepted: in the D22 order `land_short` is a regression guard (tug metric now measures the dart's own movement, not the slack taken) |
| 9 | Reference driver only ignores a timeout on RESET, not a FAULT |
| 10 | C1 check requires `dart release > stop` (step 14); since the final cross-check `stop > dart release`, as BASE_SPEC 10.1 step 14 now says |
| 11 | `tidy ran` now needs the dart side within 0.1 m of taut after TIDY (plant) |
| 12 | `dart_never_anchors`: TOW allowed only after the failed check's RELEASE |
| 13 | `LIMIT seen in RETRIEVE` counts only LIMIT events raised during the RETRIEVE step |
| 14 | `no_angle_sensor`: DONE in SW_INDEX mode OR a refusal at PREFLIGHT before any motion |
| 15 | `nogo_reshoot` checks the C1 order of the last shot (`c1_from_last_aim`) |
