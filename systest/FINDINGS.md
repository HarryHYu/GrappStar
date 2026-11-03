# System test findings

Found by the end-to-end harness (`systest/run_mission.sh`: the real base logic over a pty, the real dart firmware
sim over HTTP, the real Jetson App). Each entry: the exact failing scenario, the file and line, the owner.
Line numbers are against `origin/base-v1` at the time of the entry. Status: OPEN until the owner says fixed and the
harness re-runs green.

## F1 (fw-base) `cal` (and `set`) during the first 60 ms are acknowledged ok, then silently undone by the boot rebuild

- Scenario: fresh base, the host sends `{"id":0,"cmd":"cal","what":"cock_rest"}` 10 ms after power-on.
  Reply `{"id":0,"ok":true}`. At 60 ms `BOOT` reports `"phase":"UNKNOWN"`, and `status` keeps saying `UNKNOWN`: the
  cock_rest was thrown away. The Jetson preflight then refuses ("launcher shot phase UNKNOWN").
  Log: `systest/runs/nominal/base.log` (first run, 12:15 Sydney).
- Where: `firmware/base/grappstar_base/base_logic.h:803-807` (`cal cock_rest` sets `phase = PH_REST` with no boot
  check) and `resolveBoot()` at `base_logic.h:268` which overwrites `phase` from the saved record on the first tick
  after 60 ms. `cal aim_home`, `cal aim`, `set line_m` have the same race.
- Real-world trigger: the Jetson opens the port as soon as the board enumerates; a script or the operator page
  sending setup lines right after a reset hits it. Also the ESP32 sends HELLO before BOOT, so "send after HELLO" is
  not enough.
- Fix suggestion: refuse `cal` (all but `drum`) and `set` with `BOOTING` while `bootPending`, like the actions
  (PROTOCOL.md section 2 lists `BOOTING` as a common refusal for actions only).
- Harness workaround: waits for `BOOT` before bench setup.

## F2 (fw-base + jetson) TIDY never runs: the Jetson sends `tidy`, the firmware only knows `reel until:"slip"`

- Scenario `nominal` (every run): mission step TIDY logs "base has no tidy command: skipped". The base replied
  `UNKNOWN_CMD` to `{"cmd":"tidy"}`. The firmware has a tidy, but as `reel` with `until:"slip"`
  (`firmware/base/grappstar_base/base_logic.h:718` parse, `:1179-1183` the slip detector, LIMIT event with
  `why:"tidy done (take-up slipping)"`), and `PROTOCOL.md` section 2 does not document `slip` at all.
  Jetson: `jetson/grappstar/mission.py:488-500` (`tidy()` sends `tidy`, waits for `TIDY`/`TIDIED`/`LIMIT`).
- Spec: BASE_SPEC section 10 and D22 make TIDY a required step (the FIRM take-up pulls the dart-side slack through the
  capstan before REEL fly-in). Skipping it leaves the landing slack on the dart side.
- Fix: one of the two owners adopts the other's form and PROTOCOL.md documents it (suggest: the Jetson sends
  `{"cmd":"reel","until":"slip"}` when `tidy` is UNKNOWN_CMD, and waits for LIMIT with `why` starting "tidy").

## F3 (fw-base) PROTOCOL.md no longer matches the firmware on base-v1

- `wrap`: PROTOCOL.md:54 says "drum on from REEL to the WRAPPED stop point", refusal `NOT_IN_REEL`; the firmware
  (`base_logic.h:687-697`) wraps from NEUTRAL or later and refuses `NOT_IN_NEUTRAL`; a wrap with the collar already
  in REEL answers WRAPPED at once (`:690`).
- `reel until`: firmware accepts `slip` (`base_logic.h:718`), PROTOCOL.md:56 lists only stop / seated / tension.
- Telemetry `sw` keys: firmware `config.h:17` sends `l` and `r` (SW_DOG_L / SW_DOG_R) and `I` (SW_INDEX); PROTOCOL.md
  section 3 lists `D` (SW_DOG). D27 says one SW_DOG. The Jetson (`mission.py` `sw_closed`) reads `D`.
- Seen in the harness: telemetry `{"sw":{"a":1,"l":1,"S":1,"H":1,"W":1}}` at LAUNCH.
- The Jetson is written against PROTOCOL.md, so every drift is a latent integration bug. Please re-sync PROTOCOL.md
  (or state the version the firmware implements) when the D22 / D27 rework lands.

## F4 (jetson) mission order is still pre-D22: ANCHOR before WRAP

- Scenario `nominal`: steps `... LANDING_ZONE, BASE_NEUTRAL, ANCHOR, TIDY, WRAP, CHECK, TOW ...`
  (`jetson/grappstar/mission.py:325-347` `_sequence`). D22 (spec v1.1): LANDED -> NEUTRAL -> WRAP -> WRAPPED ->
  anchor -> TIDY -> REEL fly-in -> TOW (laying turns against a taut dart side needs about 360 N at the ring).
- In the sim plant the wrap still works after anchoring because sim.h models the anchored line with 0.3 m of slack;
  the real capstan would jam or tug the anchor. The harness will flag the order once the plant models D22 (pending).

## F5 (jetson + docs) the tow limit is only as good as CALIBRATION.md, and a tow fault has no way out

- Scenario `nominal_uncal` (factory `k_n_per_a` 8, `free_a` from params, i.e. CALIBRATION.md skipped): during TOW the
  plant line tension reached 61 N with `tow_limit_n` 40 N, then the base tripped OVERCURRENT (trip_a 5 A) holding
  the line (correct: wraps on, drum HELD). The Jetson offered Retry / Abort only; each Retry cleared the fault and
  restarted `reel until tension` into the same 61 N line: OVERCURRENT again within 50 ms, three times, then Abort
  with the dart anchored and the line at 61 N (log `systest/runs/nominal_uncal/`).
  Files: `jetson/grappstar/mission.py:300-322` (`_do`: a BaseError in TOW gives Retry / Abort; no "Release the dart"
  choice), `mission.py:808-830` (`tow`).
- Asks: (jetson) in TOW / CHECK, a base fault should offer "Release the dart" (stop, dart /release, then retrieve), and
  Retry should not restart a pull that is already above the limit; (jetson) preflight could warn when `get k_n_per_a`
  is still the factory 8.0 and there is no load cell; (fw-base / docs) BENCH_TEST or the first-mission checklist
  should make CALIBRATION.md of `free_a` and `k_n_per_a` a hard prerequisite of the first tow.
- With the calibrated bench (scenario `nominal`) the tow holds at 40.0 N exactly (plant truth 40.03 N).

## F6 (jetson) fault prompt says "went to NEUTRAL" when the base said DRUM_HELD

- Scenario `drum_timeout` (drum N20 jammed during `mode NEUTRAL` after the shot): the base raised DRUM_TIMEOUT and
  sent `{"evt":"MODE","ok":false,"mode":"LAUNCH","drum_pos":"LAUNCH","err":"DRUM_HELD","why":"drum fault"}`
  (correct per PROTOCOL.md section 6 / D27: a drum fault holds the drum). The operator prompt read "BASE_NEUTRAL
  failed: base refused wait: DRUM_TIMEOUT. The base is in FAULT and went to NEUTRAL."
- Where: `jetson/grappstar/mission.py:327` appends the NEUTRAL sentence for every fault code.
- Why it matters: the operator is told the collar is out of LAUNCH when it is still in LAUNCH (and in the cocked or
  wrapped cases the drum is deliberately held). Suggest: read the last MODE event; with `err:"DRUM_HELD"` say "the
  drum is HELD where it is (<why>)", and only say NEUTRAL when telemetry shows `drum_pos` NEUTRAL.
- Also: Retry right after DRUM_TIMEOUT re-sends the same `mode` at once (3 retries in 9 s in the log); the prompt
  could say what to check (N20 wires, drum free to turn, SW_N20 lever) before Retry.
