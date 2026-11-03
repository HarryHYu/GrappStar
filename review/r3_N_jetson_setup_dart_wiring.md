# Round 3 N: Jetson setup, first run and dart wiring (learner)

Reviewed at base-v1 commit 863d58d. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 7. Refuted and dropped: 0.

| # | Sev | File:line | Owner | Finding |
|---|---|---|---|---|
| 1 | high | `jetson/grappstar/selftest.py:42` | jetson | `python3 -m grappstar selftest` always FAILs 'mock base C1' (BOOTING race), and `python3 -m pytest` fails test_selftest |
| 2 | high | `jetson/grappstar/sim/world.py:210` | jetson | Simulator sync thread crashes on every mission, so `serve --sim` can never anchor or reach TOW (README step 8 dry mission is broken) |
| 3 | medium | `jetson/grappstar/cli.py:247` | jetson | README step 7 calibration commands fail as written, and `calibrate mount` silently discards the intrinsics `calibrate intrinsics` just saved |
| 4 | medium | `jetson/setup/hotspot.sh:35` | jetson | hotspot.sh 'up' with no interface can pick the built-in WiFi (the KEDGE radio) and make it an AP at every boot |
| 5 | low | `firmware/dart/WIRING.md:44` | fw-dart | WIRING.md Step A2 tells the learner to check v29 wire 15 instead of 16 for the DRV#2 GND pad |
| 6 | low | `jetson/README.md:433` | jetson | README dry-mission step 1 'Press Preflight. Everything should go green' always fails in sim because no target is set yet |
| 7 | low | `jetson/setup/99-grappstar.rules:26` | jetson | udev /dev/grappstar-base matches any ESP32-S3 (303a:1001), including the dart's XIAO, and the dart treats C1 JSON bytes as manual motor keys |

## 1. `python3 -m grappstar selftest` always FAILs 'mock base C1' (BOOTING race), and `python3 -m pytest` fails test_selftest

- File: `jetson/grappstar/selftest.py:42`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: A learner on a fresh machine runs README step 3: `python3 -m grappstar selftest`. It failed in 5 out of 5 runs with 'FAIL  mock base C1: BaseError: base refused mode: BOOTING' and exit code 1. tests/test_cli.py::test_selftest fails as well, so the README's '`python3 -m pytest` ... ends with "N passed"' and the checklist item at line 453 can never be met. Cause: connected_pair() returns as soon as the pipe is connected. ping and mode('LAUNCH') complete in a few ms over the in-process pipe. The mock's sim loop only advances sim_t after its first 20 ms sleep, and the mock refuses commands until sim_t >= 0.06 sets booted. The learner concludes the base client is broken when only the test is racing.

Evidence: selftest.py:38-43:
    mb, c = connected_pair(time_scale=10)
    c.ping()
    m = c.event_mark()
    c.mode("LAUNCH")
mock_base.py:1093-1094 `if not self.booted: return "BOOTING"`; mock_base.py:604 `if not self.booted and self.sim_t >= 0.06:`; mock_base.py:450-453 sim loop sleeps 0.02 s before its first step; mock_base.py:1712-1714 connected_pair only waits for wait_connected.
README.md:130-131.

Fix: In mock_base.connected_pair, after c.wait_connected(2.0), wait for the boot: `t0=time.time()\nwhile not mb.booted and time.time()-t0 < 2: time.sleep(0.01)`. Or in selftest.base(), call `c.wait_event("BOOT", 2)` (or poll until mode stops returning BOOTING) before c.mode("LAUNCH").

> Skeptic (confirmed): I could not refute this. I reproduced it. I ran `python3 -m grappstar selftest` from jetson/ 6 times and it failed every time with 'FAIL  mock base C1: BaseError: base refused mode: BOOTING' and exit code 1. (In my first 5 runs the 'rc=0' came from the pipe into tail, not from the selftest.) `python3 -m pytest -q tests/test_cli.py -k selftest` fails too: 'FAILED tests/test_cli.py::test_selftest - assert 1 == 0'.

The code matches the claim:
- jetson/grappstar/comms/mock_base.py:1707-1715: connected_pair only calls c.wait_connected(2.0). In comms/base.py:253-255 that just waits for the pipe fla
>
> Skeptic (confirmed): The claim holds, and I reproduced it. In jetson/grappstar/selftest.py:36-43, base() calls connected_pair(time_scale=10), then ping and mode("LAUNCH") straight away. connected_pair (jetson/grappstar/comms/mock_base.py:1707-1714) only waits for c.wait_connected(2.0). _sim_loop (mock_base.py:450-455) sleeps 0.02 s before its first step(). booted is only set inside step() once sim_t >= 0.06 (line 604), and _idle_check returns "BOOTING" while booted is False (lines 1093-1094). Over the in-process pipe, ping and mode take far less than 20 ms, so mode("LAUNCH") arrives before the first sim step every
>

## 2. Simulator sync thread crashes on every mission, so `serve --sim` can never anchor or reach TOW (README step 8 dry mission is broken)

- File: `jetson/grappstar/sim/world.py:210`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Ran `python3 -m grappstar serve --sim`, then target_auto, then start, answering every prompt (Base may move, Aim, Cock, Fire, FIRE NOW, Anchor). Entering LAUNCH sets mock_base aim_known=False (mock_base.py:696). Before the first aim, aim_target is None, so `tgt` falls back to `mb.pitch`, which is also None. `abs(tgt - self.rendered_pitch)` then raises TypeError and the sim-sync thread dies (log: 'Exception in thread sim-sync ... world.py, line 210 ... TypeError: unsupported operand type(s) for -: NoneType and float'). That thread is the only code that sets world.dart='anchored' when the dart reports ANCHORED (world.py:197-199). With the thread dead, the mock base treats the dart as free. CHECK fails ('check pull never reached the test tension: the dart may be dragging', then on Retry 'the dart came all the way home during the check'), and the learner can only Release and reshoot. TOW is never reached on the GO hard_ground default. The thread is also what re-renders the flight video for a new aim pitch, so that stops too. I patched only this line (added `tgt is not None`) and reran the same mission: 'anchor holds at 15.1 N', then STEP TOW. The slow test test_app_sim misses this because it accepts DONE reached through the release path.

Evidence: world.py:205-210:
    tgt = getattr(mb, "aim_target", None)
    if tgt is None:
        tgt = getattr(mb, "pitch", 35.0)
        tgt = tgt() if callable(tgt) else tgt
    if (self._ready.is_set() and not self._rendering and self.t_fire is None and self.rendered_pitch is not None
            and abs(tgt - self.rendered_pitch) > 1.0):
mock_base.py:302 `return pitch_at(...) if self.aim_known else None`; mock_base.py:696 `self.aim_known = self.aimed = False` on reaching LAUNCH.
README.md:438 '6. Follow the prompts through anchoring, tensioning, release and reeling in.'

Fix: In world.py _sync, skip the re-render check while tgt is None: change line 209-210 to `... and self.rendered_pitch is not None and tgt is not None and abs(tgt - self.rendered_pitch) > 1.0):`. Also wrap the loop body in try/except Exception that logs the error and continues, so one bad sample cannot kill the thread for the rest of the session. Tighten test_app_sim so it asserts 'TOW' is in the mission history on hard_ground.

> Skeptic (confirmed): I reproduced the defect, so I can't refute it.

How it happens:
- MockBase starts with drum_at="NEUTRAL" and aim_target=None.
- The mission's LAUNCH_MODE step comes before AIM (STEPS order in mission.py). It drives the drum to LAUNCH. When the drum gets there, mock_base.py:696 sets aim_known=False, so the `pitch` property returns None.
- In world.py:204-210, `tgt` is aim_target (None), then falls back to mb.pitch (also None). At this point _ready is set, rendered_pitch is 35.0 from the initial prerender and t_fire is None. So `abs(tgt - self.rendered_pitch)` raises TypeError. Nothing in the lo
>
> Skeptic (confirmed): I could not refute this. I ran it and the crash happens.

How the crash happens:
- In jetson/grappstar/comms/mock_base.py, `pitch` (line 302) is a property that returns None whenever aim_known is False. getattr(mb, "pitch", 35.0) therefore returns None, not the 35.0 default.
- MockBase starts with the drum at NEUTRAL (the drum_at default, line 181). The mission's LAUNCH_MODE step runs before AIM (mission.py STEPS, lines 459-460), so the drum really does travel to LAUNCH.
- When that travel finishes, line 696 sets aim_known and aimed to False. aim_target is still None because it is only assigne
>

## 3. README step 7 calibration commands fail as written, and `calibrate mount` silently discards the intrinsics `calibrate intrinsics` just saved

- File: `jetson/grappstar/cli.py:247`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: (1) README step 7 says to run `python3 -m grappstar calibrate intrinsics`, then hold the board in front of the camera for 20 views. The command does not capture from a camera: `--images` is required, so it exits with an argparse error. The README never mentions `snap`, the command that captures the images. (2) cli.cmd_calibrate saves intrinsics to calib/intrinsics.json (line 250-252). For `mount`/`ground` it adds only `--out` and never passes `--intrinsics`, so calibrate.py falls back to kiyo_default_intrinsics (calibrate.py:309-313). A learner who follows the README order (intrinsics, then `calibrate mount --image x.png`) gets calib/rovercam.json built from the Kiyo estimate, and their checkerboard calibration is lost. The only sign is one WARNING line. (3) README line 355-356 says 'give the measured size' of a square, but `--square` is in metres. Typing `--square 25.3` makes the ground solution 1000x off.

Evidence: README.md:362 '2. Run `python3 -m grappstar calibrate intrinsics` (with `--help` options for the camera and square size).' README.md:372 'Run `python3 -m grappstar calibrate mount` and follow what it asks'.
calibrate.py:267 `a.add_argument("--images", nargs="+", required=True)`; calibrate.py:273 `g.add_argument("--image", required=True)`; calibrate.py:270 help="square size in metres".
cli.py:247-252: ground/marker/default get only `argv += ["--out", DEFAULT_CALIB]`; intrinsics gets `--out .../intrinsics.json`.

Fix: In cli.cmd_calibrate, for argv[0] in ("ground", "marker"), when "--intrinsics" is not in argv and calib/intrinsics.json exists, append `["--intrinsics", <calib>/intrinsics.json]`. Rewrite README step 7: `python3 -m grappstar snap --n 20` (saves calib/images/snap_NN.png), then `python3 -m grappstar calibrate intrinsics --images calib/images/*.png --square 0.025`. For mount: `python3 -m grappstar snap --n 1 --out calib/ground`, then `python3 -m grappstar calibrate mount --image calib/ground/snap_00.png --square 0.025`. State that square size is in metres.

> Skeptic (confirmed): The claim cites README.md, but the text is in jetson/README.md, lines 343-376. After correcting that path, all three parts hold up against the code.

(1) jetson/grappstar/vision/calibrate.py:267 has `--images nargs="+" required=True`, and ground has `--image required=True`. So step 7.2, "Run `python3 -m grappstar calibrate intrinsics`" and then hold the board for 20 views, fails with an argparse error. Nothing in that subcommand reads from a camera. Step 7 never mentions `snap` (cli.py:77, which saves snap_NN.png). The only "snap" lines in the README are the unrelated warnings about the Ubuntu
>
> Skeptic (confirmed): I checked the claim against the actual code and it holds. Two corrections first: the README is jetson/README.md (its text is copied into docs/build/GrappStar_Build_Guide.md around line 2371), and calibrate.py is at jetson/grappstar/vision/calibrate.py. Neither changes the substance.

(1) The intrinsics step cannot work as written. jetson/README.md:362-363 says to run `python3 -m grappstar calibrate intrinsics` and then hold the board in front of the camera. But calibrate.py:267 has `--images` with required=True, and the code only reads image files (`_read_images`). It never opens a camera, so 
>

## 4. hotspot.sh 'up' with no interface can pick the built-in WiFi (the KEDGE radio) and make it an AP at every boot

- File: `jetson/setup/hotspot.sh:35`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: In the field the dart is off and no home WiFi is in range, so the built-in wlP1p1s0 shows as 'disconnected' (the saved 'kedge' connection is inactive). The learner follows README line 314 and runs `bash setup/hotspot.sh up` with no name. pick_iface returns the first wifi device in state 'disconnected', which is wlP1p1s0 and not the USB adapter (it may not even be plugged in yet). The script creates grappstar-hotspot on the built-in radio and line 67 sets autoconnect yes. From then on, whenever the Jetson boots before the dart is powered, NetworkManager brings the hotspot up on wlP1p1s0, because an AP connection is always 'available' and KEDGE is not visible yet. NM does not drop an active connection when a higher-priority one appears, so the Jetson never joins KEDGE: no dart link (status, auto, release, flight log), and the mission is lost. README line 323 says 'Do not run the hotspot on the built-in WiFi', but the script's automatic choice does exactly that.

Evidence: hotspot.sh:33-36
  pick_iface() {
      # First WiFi interface that is not connected to anything (the spare adapter).
      nmcli -t -f DEVICE,TYPE,STATE device | awk -F: '$2 == "wifi" && $3 == "disconnected" { print $1; exit }'
  }
hotspot.sh:46  IFACE="${2:-$(pick_iface)}"
hotspot.sh:67  sudo nmcli connection modify "$CON" connection.autoconnect yes ipv4.never-default yes
README.md:314  bash setup/hotspot.sh up                 # picks the free WiFi interface

Fix: Only auto-pick USB adapters, and refuse the interface that KEDGE uses. In pick_iface, loop over the wifi devices and keep one only if `readlink -f /sys/class/net/$dev/device` contains '/usb' (the built-in wlP1p1s0 is PCIe). Before creating the hotspot, also refuse IFACE when it equals `nmcli -g connection.interface-name connection show kedge` (or when it is not a USB device), unless the user passes a --force flag. Print 'No USB WiFi adapter found' instead of falling back to the built-in radio.

> Skeptic (confirmed): I could not refute this claim. The code matches it exactly. pick_iface (jetson/setup/hotspot.sh:33-36) returns the first wifi device whose STATE is "disconnected". It does not check that the device is a USB adapter, and it does not exclude the interface the kedge connection uses. Line 46 uses that result whenever no interface name is given, and the README tells users to run exactly that (`bash setup/hotspot.sh up  # picks the free WiFi interface`). Lines 60 and 64 then bind grappstar-hotspot to that interface, and line 67 sets it to start at boot.

The scenario is concrete. Suppose the operato
>
> Skeptic (confirmed): The claim holds up against the code. hotspot.sh:33-36 picks the first wifi device whose state is 'disconnected'. It does not check whether the device is USB and does not exclude the interface the 'kedge' connection uses. If KEDGE is out of range (dart off), the built-in wlP1p1s0 shows 'disconnected', and nothing else in the repo guards against this (grep finds hotspot handling only in hotspot.sh and the README/build guide). It is also worse than the claim says, if I have nmcli's device sort order right (state first, then type, then name; I recalled this rather than checked it). With the USB ad
>

## 5. WIRING.md Step A2 tells the learner to check v29 wire 15 instead of 16 for the DRV#2 GND pad

- File: `firmware/dart/WIRING.md:44`
- Severity: low (claimed low); unverified (low)
- Owner: fw-dart

Scenario: Path A conversion, Step A2. Line 36 says numbers in square brackets are v29 numbers, and A2 uses them for [13], [17] and [27]. Line 44 then says wire 17 shares the DRV#2 GND pad with wire 15. In the v29 list, wire 15 is 'MP1584 IN- (same pad) -> DRV#1 GND', which is not on DRV#2. The wire that really shares the DRV#2 GND pad is v29 wire 16 ('DRV#1 GND -> DRV#2 GND'). It became v4 wire 15, so the text mixes the two numbering schemes. A learner who follows the v29 numbers tugs the DRV#1-side ground wire and never checks the DRV#2 GND joint they just reheated while desoldering wire 17. The A7 meter checks do not test DRV#2 GND continuity. If that joint was disturbed, DRV#2 has no ground, so the drills and the foot do not run at all. The firmware only reports this later as NO_CURRENT at foot finishing (FAILURE_MODES.md D5).

Evidence: firmware/dart/WIRING.md:36 "Wire numbers in brackets are the v29 numbers, then the new numbers."
firmware/dart/WIRING.md:42 "- [17] DRV#2 GND to DRV#3 GND (black, 22 AWG)"
firmware/dart/WIRING.md:44 "Wires 13 and 17 share the DRV#2 VM and GND pads with wires 12 and 15: leave 12 and 15 in place and tug them after."
kedge/pack_v29/wires.json (v29 list): [15, 'MP1584 IN- (same pad)', 'DRV#1 GND'], [16, 'DRV#1 GND', 'DRV#2 GND']
firmware/dart/wiring/wires.json: v4 wire 15 = ['DRV#1 GND', 'DRV#2 GND', ..., v29 id 16]

Fix: Change WIRING.md line 44 to: "Wires [13] and [17] share the DRV#2 VM and GND pads with wires [12] and [16] (new 12 and 15): leave those in place and tug them after." Also add this line to the Step A7 checks: "Meter on beep: DRV#2 GND to DRV#1 GND beeps, DRV#2 VM to DRV#1 VM beeps."


## 6. README dry-mission step 1 'Press Preflight. Everything should go green' always fails in sim because no target is set yet

- File: `jetson/README.md:433`
- Severity: low (claimed low); unverified (low)
- Owner: jetson

Scenario: Following step 8 in order on a fresh `serve --sim`: pressing Preflight first returns {'ok': False, 'msg': 'preflight: no target: tap the target on the rover view or enter a range'}. The learner is told everything should go green and instead sees a failure. A target is only set at step 2.

Evidence: README.md:433 '1. Press **Preflight**. Everything should go green (base link, dart link, cameras).' README.md:434 '2. Click a target on the rover view (or press auto target).' Observed: preflight {'ok': False, 'msg': 'preflight: no target: tap the target on the rover view or enter a range'}; README.md:405 lists 'a target set' as a PREFLIGHT check.

Fix: Swap steps 1 and 2 in README section 8: set the target first (click it or press Auto target), then press Preflight.


## 7. udev /dev/grappstar-base matches any ESP32-S3 (303a:1001), including the dart's XIAO, and the dart treats C1 JSON bytes as manual motor keys

- File: `jetson/setup/99-grappstar.rules:26`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: The rule matches on VID:PID alone, and the dart's XIAO ESP32S3 enumerates as the same 303a:1001. Two plausible ways in. (1) Base and dart are both plugged into the Jetson by USB: the symlink points at whichever enumerated last, so the service talks to the wrong board and the base link is lost. (2) Only the dart is plugged in, for example to flash or watch Serial from the Jetson while grappstar.service is running: BaseClient opens it as the base and reports connected (it never checks the board's identity). Every command line is then sent byte by byte to the dart's loop(), which feeds each byte to handleKey(). Pressing Clear on the page sends {"id":N,"cmd":"clear"}: the 'a' starts the auto-anchor sequence (kick swing, deploy, drills), and the 'r' that follows is refused as 'busy', so the sequence keeps running. 'aim' and 'cal' messages also contain an 'a' with no later 's'. Digits in the id run DEPLOY, KICK, DRILLS or FOOT for up to serialRunMs = 1500 ms. The motors move only if the power loop is also in (against the WIRING/BENCH rule), but the dart's state is changed in every case and the base is silently replaced.

Evidence: 99-grappstar.rules:26  SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", SYMLINK+="grappstar-base", ...
99-grappstar.rules:24-25  # Note: the dart's XIAO ESP32S3 has the same IDs. Do not plug the dart into the Jetson by USB while the base is plugged in, or the name can point at the wrong board.
comms/base.py:380  msg = {"id": i, "cmd": name}
kedge_dart.ino:378  if (Serial.available()) { char c = Serial.read(); if (c != '\n' && c != '\r') { reply = ""; handleKey(c, K.tu.serialRunMs); } }
kedge_logic.h:427-428  if (c == 'a') return startAuto(now);
    if (c == 'r') return startRelease(now);
kedge_logic.h:433  case '1': m = DEPLOY; d = +1; break; ...

Fix: Match the base by its unique USB serial number (the chip's MAC address), not by VID:PID alone: SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", ATTRS{serial}=="<base MAC from `udevadm info -a -n /dev/ttyACM0 | grep serial`>", SYMLINK+="grappstar-base", ... Have install.sh fill in the serial (or prompt for it), and tell the learner how to read it. Also make BaseClient check the board on connect: after opening, send a ping and treat the link as up only when a valid C1 JSON reply arrives.

> Skeptic (confirmed): I checked each step of the claim against the code, and the mechanism holds.
(a) jetson/setup/99-grappstar.rules:26 matches ttys on 303a:1001 only, with no serial or other identity check. Lines 24-25 admit that the dart's XIAO ESP32S3 shares these IDs.
(b) jetson/grappstar/comms/base.py `_run` calls `_set_connected(True)` as soon as `transport.open()` succeeds. Nothing checks which board answered.
(c) firmware/dart/kedge_dart/kedge_dart.ino:378 passes every serial byte except CR/LF to `handleKey`, with runMs = serialRunMs (1500).
(d) kedge_logic.h `key()` maps 'a' to `startAuto` and 'r' to `sta
>
> Skeptic (confirmed): I checked the code, and the mechanism is real. I'd rate it low, not medium.

What checks out:
- jetson/setup/99-grappstar.rules:26 matches the base on 303a:1001 alone. There is no ATTRS{serial} match.
- The dart firmware requires "USB CDC On Boot: Enabled" (kedge_dart.ino:15 and :28) on the XIAO_ESP32S3 board. That board's default USB mode is the hardware CDC, so it shows up with the same 303a:1001 IDs. The rules file says so itself at lines 24-25.
- BaseClient._run (jetson/grappstar/comms/base.py:268-273) calls _set_connected(True) as soon as transport.open() succeeds. It never checks which b
>
