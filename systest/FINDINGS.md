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
