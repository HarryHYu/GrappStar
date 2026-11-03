# systest: end-to-end system test (worker `systest`, 3 Nov 2026)

Status: DRAFT (updated through the day; final at 18:20 Sydney).

## Summary (for Harry)

- A whole GrappStar mission runs end to end with the real base firmware logic, the real dart firmware and the real
  Jetson code talking over their real links (serial pty, HTTP), no hardware. On the owners' latest branches the
  nominal mission and the BASE_SPEC 10.1 sequence complete with every check on plant truth.
- 30 findings went to the owners during the day (F1 to F30). Most are fixed on the owners' branches and re-tested
  green here (status table at the top of FINDINGS.md). The ones that mattered most: the wrap jamming against an
  anchored line in the old order (F4), the REEL fly-in tripping on a taut real line (F20, fixed as D41), the tow
  overshooting the limit on a stiff line (F21), settings lost at a power cycle (F16), power cuts leaving the
  launcher in a state nothing could clear (F15, F17), a calibration procedure that towed 10 N over the limit (F14).
- Open at the time of writing: see the status table; the remaining ones are the Jetson's tow re-pull after rover
  slack (F22 host side), the uncalibrated-tow guard (F28), and low-severity items.

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
(a free dart is tugged, an anchored one jams the cup), and tidy pulls an anchored line taut. Strict for `nominal`,
`land_short` and all `ref_*`; lenient for the mission fault scenarios until F4/F12 land (`SYSTEST_D22_ALL=1` for strict).

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

(pasted from the last run below)

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
| 10 | C1 check requires `dart release > stop` (step 14) |
| 11 | `tidy ran` now needs the dart side within 0.1 m of taut after TIDY (plant) |
| 12 | `dart_never_anchors`: TOW allowed only after the failed check's RELEASE |
| 13 | `LIMIT seen in RETRIEVE` counts only LIMIT events raised during the RETRIEVE step |
| 14 | `no_angle_sensor`: DONE in SW_INDEX mode OR a refusal at PREFLIGHT before any motion |
| 15 | `nogo_reshoot` checks the C1 order of the last shot (`c1_from_last_aim`) |
