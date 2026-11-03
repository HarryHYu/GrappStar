# GrappStar rover software (Jetson)

This folder is the software that runs on the rover's Jetson Orin Nano. It talks to the base station (USB serial),
the KEDGE dart (WiFi), and two cameras, and it serves the operator page in a web browser.

Run everything from this folder (`jetson/`) with `python3 -m grappstar <command>`.
Every command has `--help`, for example `python3 -m grappstar serve --help`. If this README and `--help`
disagree, `--help` is right (and please fix this README).

Marked **NOT VERIFIED** below: things nobody has checked on the real hardware yet.

## Contents

0. What you need
1. Get the code
2. Install
3. Test with no hardware
4. Plug in the base
5. Cameras
6. Dart WiFi (and how the operator connects)
7. Calibrate the rover camera
8. Operator page tour and a full dry mission in sim
9. First real mission checklist
10. Troubleshooting
11. File map

Command summary:

| Command | What it does |
|---|---|
| `python3 -m grappstar serve` | operator page at `http://<jetson>:8080`. Options: `--sim`, `--port 8080`, `--base /dev/grappstar-base`, `--dart 192.168.4.1`, `--rover-cam /dev/grappstar-rovercam`, `--dart-cam /dev/grappstar-dartcam`, `--fourcc YUYV` or `MJPG` |
| `python3 -m grappstar cameras` | list cameras and the formats they offer |
| `python3 -m grappstar base status` / `ping` / `monitor` | talk to the base. `monitor` prints telemetry live (Ctrl+C stops). Only one program can hold the base port at a time (serve, monitor, console, cmd): a second one says "port busy" |
| `python3 -m grappstar base cmd '<json>'` | send one raw command, e.g. `'{"cmd": "status"}'` |
| `python3 -m grappstar base console` | one connection that prints telemetry (twice a second) and every event, and sends what you type: a word (`stop`, `estop`, `status`), a word with `key=value` pairs (`reel duty=0.3 max_n=20 until=stop`) or a JSON line. `quit` ends. Use it for calibration (watch the tension, type `stop`) |
| `python3 -m grappstar dart status` | one dart status reading |
| `python3 -m grappstar dart key <c> --hold <s>` | hold a dart motor key for s seconds |
| `python3 -m grappstar dart auto` / `release` / `flightlog` | auto anchor, release sequence, download the flight log |
| `python3 -m grappstar mockbase --pty` | fake base for testing; prints a pty path to use as `--base` |
| `python3 -m grappstar calibrate intrinsics` / `mount` | rover camera calibration (step 7) |
| `python3 -m grappstar replay <logdir>` | re-run the vision on a recorded mission |
| `python3 -m grappstar selftest` | run checks that need no hardware |

---

## 0. What you need

- Jetson Orin Nano Super with JetPack 6.2 on its SD card (already done in August: hostname `GrappStar`).
- Internet for the Jetson during install (home WiFi or Ethernet).
- A way to type on it: monitor and keyboard, or SSH from the Mac: `ssh <your user>@GrappStar.local`
  (or the IP address, which was 192.168.0.249 on home WiFi in August).
- For later steps: the base station (ESP32-S3 DevKitC-1) and a USB cable, the Razer Kiyo X, the RC832 receiver and
  the DIGITNOW USB capture dongle, the KEDGE dart, a phone or laptop for the operator page.
- A printed checkerboard for step 7.

Two things learned in August that this kit works around:

- snap is broken on JetPack 6.2. Use `apt` for everything, never `snap`.
- numpy must stay below 2. pip once upgraded it to 2.x and broke system packages. The install keeps it at 1.x.

## 1. Get the code

On the Jetson:

```bash
cd ~
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/HarryHYu/GrappStar.git
cd GrappStar
git checkout main
cd jetson
```

- `GIT_LFS_SKIP_SMUDGE=1` skips the big caliper photo PDF (145 MB). The Jetson does not need it.
- If `main` has no `jetson/` folder yet (before the build day merge), use the integration branch instead:
  `git checkout base-v1`.
- To update later: `cd ~/GrappStar && git pull`.

## 2. Install

```bash
cd ~/GrappStar/jetson
bash setup/install.sh
```

It asks for your password (sudo). What it does, in order:

1. apt installs `python3-numpy python3-serial python3-pytest python3-pip v4l-utils network-manager`.
2. OpenCV: JetPack normally already has OpenCV for Python (NVIDIA's build, with GStreamer). If `import cv2` already
   works the script leaves it alone. Only if it is missing does it run `apt install python3-opencv`.
   Why not `pip install opencv-python`? The pip version has no GStreamer, carries its own JPEG and video libraries
   (August's JPEG mismatch was exactly this kind of clash), and can drag numpy up to 2.x.
3. `pip install --user -r requirements.txt` for anything apt did not provide. The file pins `numpy>=1.21,<2`.
4. Adds you to the groups `dialout` (serial ports) and `video` (cameras).
5. Installs `setup/99-grappstar.rules` so the devices get fixed names:
   `/dev/grappstar-base`, `/dev/grappstar-base-uart`, `/dev/grappstar-rovercam`, `/dev/grappstar-dartcam`.
   `/dev/grappstar-base` needs the base's USB serial number: step 4 sets it. Until then that name is missing.
6. Runs `setup/check_env.py` and prints a report.

Then **log out and back in** (or `sudo reboot`) so the new groups work.

Check any time with:

```bash
python3 setup/check_env.py
```

Look for: numpy 1.x, cv2 with `V4L2 YES` (and `GStreamer YES` on the Jetson), pyserial, groups dialout and video yes.

Optional: start the operator page automatically at boot (do this after step 3 works):

```bash
bash setup/install.sh --service
systemctl status grappstar          # should say "active (running)"
journalctl -u grappstar -f          # live log, Ctrl+C to leave
```

The service runs `python3 -m grappstar serve --port 8080` as your user from this folder, and restarts it if it
crashes. While the service runs, do not also start `serve` by hand (both want port 8080). Stop it with
`sudo systemctl stop grappstar`, remove it from boot with `sudo systemctl disable --now grappstar`.

Optional: onnxruntime-gpu for a GPU terrain model. Not needed to run anything. See the note at the bottom of
`requirements.txt` (Jetson AI Lab wheel, NOT VERIFIED).

Optional: maximum performance mode (already set in August): `sudo nvpmodel -q` shows the mode. `jtop` shows load.

## 3. Test with no hardware

Nothing needs to be plugged in for this step. It also works on a PC.

```bash
cd ~/GrappStar/jetson
python3 -m pytest                  # all tests; ends with "N passed"
python3 -m grappstar selftest      # quick checks, prints PASS or FAIL per check
python3 -m grappstar serve --sim   # operator page with a simulated base, dart and cameras
```

With `serve --sim` running, open the page:

- on the Jetson itself: `http://localhost:8080`
- from the Mac or phone on the same network: `http://GrappStar.local:8080` or `http://<jetson IP>:8080`
  (find the IP with `hostname -I`).

You should see both camera views moving (simulated) and base and dart status updating. Ctrl+C in the terminal
stops the server. Step 8 walks through a whole simulated mission.

Test the real base client against a fake base (two terminals):

```bash
python3 -m grappstar mockbase --pty           # terminal 1: prints a path like /dev/pts/5
python3 -m grappstar base ping --base /dev/pts/5    # terminal 2, using the path it printed
```


## 4. Plug in the base

**Battery unplugged.** For this step the base runs from USB only: the Jetson's USB powers the DevKitC. Do not connect
the 3S LiPo, the BTS7960 or the motors yet (if you are following the build guide, they are already wired: battery
unplugged, just check the USB port). Any time you change wiring, unplug the battery first.

Why this step matters: the base (ESP32-S3 DevKitC) and the dart (XIAO ESP32S3) show up on USB with the same ID,
303a:1001. To tell them apart, the Jetson remembers the base's own USB serial number (each board has a different
one, it looks like `F4:12:FA:12:34:56`). Then `/dev/grappstar-base` only ever points at the base.

1. Battery unplugged. Unplug the dart's USB cable from the Jetson if it is plugged in. Only the base may be plugged
   in by USB for this step.
2. Plug a USB cable from the Jetson into the DevKitC port marked **USB** (the ESP32-S3 native port), not the one
   marked UART or COM. Check it is seen:

   ```bash
   lsusb                              # expect a line with 303a:1001 (Espressif)
   ls /dev/ttyACM*                    # expect /dev/ttyACM0
   ```

3. Save the base's serial number:

   ```bash
   bash setup/install.sh --no-apt
   ```

   It prints `ESP32-S3 board on /dev/ttyACM0, serial ...` and asks "Is only the base plugged in?". Answer `y`.
   It writes the serial into the installed rule and prints `base serial ...`.
   If it says more than one board is plugged in, unplug the dart and run it again.
   By hand instead: `udevadm info -q property -n /dev/ttyACM0 | grep ID_SERIAL_SHORT`, then
   `bash setup/install.sh --no-apt --base-serial <that value>`. Running install.sh again later keeps the saved
   serial. New base board? Run it again with `--base-serial <new value>`.
4. Unplug the base's USB cable and plug it back in. Check the name appeared:

   ```bash
   ls -l /dev/grappstar-base
   ```

   `/dev/grappstar-base -> ttyACM0` means it worked. If you used the UART port instead you get
   `/dev/grappstar-base-uart`; move the cable to the USB port.
5. Talk to it:

   ```bash
   sudo systemctl stop grappstar       # only if you installed the service: it holds the port and the cameras
   python3 -m grappstar base ping      # expect a reply with "ok": true
   python3 -m grappstar base status
   python3 -m grappstar base monitor   # telemetry lines about 10 per second, Ctrl+C stops
   ```

6. Only after this works, and following the base build guide: battery unplugged, wire the power side, then
   connect the battery last.

Now the dart may be plugged into the Jetson by USB too: it never gets the base's name. As a second guard, the
software only treats a port as the base once a valid base reply (a C1 JSON line) arrives, so it never sends base
commands to the dart.

The base firmware must use the native USB CDC (Arduino IDE: "USB CDC On Boot: Enabled", "USB Mode: Hardware CDC and
JTAG"). The base talks JSON lines (contract C1, `firmware/base/PROTOCOL.md`).

## 5. Cameras

**Battery unplugged** before connecting the RC832 receiver's power. Always screw the antennas onto the RC832 and onto
the dart's Caddx Ant VTX before powering them: a VTX powered without an antenna can burn out.

1. Plug in the Kiyo X and the DIGITNOW capture dongle (USB). Connect the RC832 video out (yellow) to the dongle's
   video in. Battery unplugged while you wire the RC832 power; power it as its label says (NOT VERIFIED which
   supply on the rover).
2. If the boot service is installed, stop it first (it holds the cameras): `sudo systemctl stop grappstar`.
   List the cameras:

   ```bash
   python3 -m grappstar cameras
   ls -l /dev/grappstar-rovercam /dev/grappstar-dartcam
   ```

   If a name is missing, run `lsusb`, find the camera's `ID vvvv:pppp`, and fix the line in
   `setup/99-grappstar.rules` (the Kiyo X ID 1532:0e4f and the dongle ID 534d:2109 are NOT VERIFIED). Then rerun
   `bash setup/install.sh --no-apt` and replug.
3. See exactly which formats each camera offers:

   ```bash
   v4l2-ctl -d /dev/grappstar-rovercam --list-formats-ext
   v4l2-ctl -d /dev/grappstar-dartcam --list-formats-ext
   ```

   Look for `YUYV` and `MJPG` and the sizes listed under each.
4. Pick the format. The software always asks for a format explicitly, because on JetPack the default path through
   NVIDIA's JPEG library failed in August (Cheese crashed). Default is YUYV:

   ```bash
   python3 -m grappstar serve --fourcc YUYV     # safe, works at lower resolutions
   python3 -m grappstar serve --fourcc MJPG     # needed for 1920x1080 on the Kiyo X
   ```

   If the picture is black or you see JPEG errors with one, try the other.
5. Dart video: power the dart (its VTX transmits). On the RC832, press the button to step through channels until
   the picture is clean. The RC832 must be on the same band and channel as the Caddx Ant VTX (the VTX shows its
   channel with its LED or button; see its manual). Snow means wrong channel, no antenna, or too far.
6. Start `serve` and check both views on the page. The dart view is analog: some snow and dropouts are normal,
   and the software marks bad frames and skips them for vision.

## 6. Dart WiFi (and how the operator connects)

The dart is a WiFi access point: name **KEDGE**, password **grappstar**, address **192.168.4.1**. The Jetson must
join it to control the dart. At the same time the operator's phone or laptop must reach the Jetson's page.

### The plan

- The Jetson's built-in WiFi joins KEDGE. Always.
- The operator connects in one of three ways. **Start with A** (nothing extra to buy). Move to B or C for real
  missions if A is slow or drops.

| Option | Operator connects to | Page address | Good | Bad |
|---|---|---|---|---|
| A | KEDGE too | `http://<Jetson KEDGE IP>:8080` | nothing extra, simplest | all video goes through the dart's small ESP32 radio, which also carries the dart's own commands. NOT VERIFIED how well it copes |
| B | Jetson hotspot GRAPPSTAR-ROVER on a USB WiFi adapter | `http://10.42.0.1:8080` | separate radio for video | needs a USB WiFi adapter with Linux AP mode support |
| C | travel router plugged into the Jetson's Ethernet | Jetson's address on the router | most reliable | one more box to power |

The ESP32 access point allows only a few clients at once (4 by default), so with option A keep it to the Jetson and
one operator device.

### Join KEDGE from the Jetson

Do the install (step 2) first: once the built-in WiFi is on KEDGE, the Jetson has no internet through it.

Power the dart, then on the Jetson:

```bash
nmcli device                                  # find the WiFi interface, usually wlP1p1s0
nmcli device wifi list                        # KEDGE should be listed
sudo nmcli device wifi connect KEDGE password grappstar ifname wlP1p1s0 name kedge
# KEDGE has no internet: never use it as the default route
# keep retrying when the dart is off, and prefer it over home WiFi
sudo nmcli connection modify kedge ipv4.never-default yes ipv6.method ignore \
    connection.autoconnect yes connection.autoconnect-priority 10 connection.autoconnect-retries 0
ping -c 3 192.168.4.1
python3 -m grappstar dart status
```

Replace `wlP1p1s0` if `nmcli device` shows a different name. Back to home WiFi for updates:
`nmcli connection up <home wifi name>` (list names with `nmcli connection show`).
Note: with priority 10, the Jetson picks KEDGE whenever the dart is on.

Dart commands:

```bash
python3 -m grappstar dart status            # phase, battery, motors
python3 -m grappstar dart key 2 --hold 1.0  # hold "kick out" for 1 s
python3 -m grappstar dart auto              # auto anchor sequence
python3 -m grappstar dart release           # release sequence
python3 -m grappstar dart flightlog         # download the last flight log
```

Dart keys: `1` deploy down, `!` deploy up, `2` kick out, `@` kick back, `3` drills in, `#` drills out, `4` foot in,
`$` foot out. Quote the symbols in the shell, e.g. `dart key '!' --hold 1.0`.

### Tell the Jetson how the IMU sits in the dart

The dart's flight log is in the BNO085 chip's own axes. The bench test in `firmware/dart/BENCH_TEST.md` (step 7)
finds how the chip sits in the hull. Write it as three entries: which chip axis points along the nose (body x),
which points left (body y) and which points up when the dart lies level (body z), with a minus sign where the chip
axis points the other way. Examples: `x,y,z` (chip matches the dart), `y,-x,z` (chip turned 90 degrees). Put it in
`settings.json` (below) as `"imu_axes": "y,-x,z"`, or pass `--imu-axes y,-x,z` to `serve`. A mirrored entry (for
example `y,x,z`) is refused: one sign is wrong.

### Settings file

Copy `settings.example.json` to `settings.json` in this folder and edit it once. `serve` (and the boot service)
read it, so you do not have to type options. Options typed on the command line still win. Keys: `base`, `dart`,
`rover_cam`, `dart_cam`, `fourcc`, `rover_fourcc`, `rover_size`, `dart_size`, `tow_limit`, `drag`, `imu_axes`.

### Option A: the operator joins KEDGE too

1. On the Jetson, find its KEDGE address:

   ```bash
   ip -4 addr show wlP1p1s0         # look for "inet 192.168.4.x"
   ```

2. Phone or laptop: join WiFi KEDGE (password grappstar). It will say "no internet": choose to stay connected
   (Android asks; turn off "switch to mobile data" if it keeps leaving).
3. Open `http://192.168.4.x:8080` (the x from step 1). `http://GrappStar.local:8080` may also work (mDNS works on
   iPhone and Mac, not on every Android).

### Option B: hotspot on a USB WiFi adapter

1. Plug in the USB WiFi adapter. `bash setup/hotspot.sh list` shows it (a name like `wlx...`).
2. Start the hotspot (it also starts at every boot from now on):

   ```bash
   bash setup/hotspot.sh up                 # picks the USB WiFi adapter
   bash setup/hotspot.sh up wlx00c0ca123456 # or name it
   ```

   It only ever uses a USB adapter. It refuses the built-in WiFi and the interface the `kedge` connection uses,
   and says "No USB WiFi adapter found" if none is plugged in.

   This runs `nmcli device wifi hotspot ifname <adapter> con-name grappstar-hotspot ssid GRAPPSTAR-ROVER
   password grappstar`.
3. Operator: join **GRAPPSTAR-ROVER** (password grappstar), open `http://10.42.0.1:8080`.
4. Stop it: `bash setup/hotspot.sh down`. Status: `bash setup/hotspot.sh status`.

Do not run the hotspot on the built-in WiFi: that radio is on KEDGE.

### Option C: travel router on Ethernet

1. Set up the travel router as a normal WiFi router (its own WiFi name and password, DHCP on). Plug a cable from
   one of its LAN ports to the Jetson's Ethernet port.
2. On the Jetson:

   ```bash
   nmcli device                                    # find the Ethernet name, usually eth0 or enP8p1s0
   sudo nmcli connection add type ethernet ifname eth0 con-name rover-lan ipv4.method auto \
       connection.autoconnect yes
   ip -4 addr show eth0                            # the Jetson's address on the router
   ```

   Replace `eth0` with the name `nmcli device` showed.

3. Operator: join the router's WiFi and open `http://<that address>:8080`. In the router's admin page you can
   reserve that address so it never changes.

## 7. Calibrate the rover camera

The rover aims the base at things it sees, so it must know how its camera maps pixels to angles (intrinsics) and
how the camera sits on the rover (mount: height and tilt).

You need: the rover camera plugged in (`/dev/grappstar-rovercam`, step 5) and a printed checkerboard.
Run every command from this folder (`jetson/`). Files go into `calib/`. Every command has `--help`.

### Print the checkerboard

- **9 by 6 inner corners** (that is 10 by 7 squares), **25 mm squares**. A generator such as calib.io
  ("Checkerboard", 10 columns, 7 rows of squares, 25 mm) or OpenCV's `pattern.png` works.
- Print at 100 percent ("actual size", no "fit to page"). Measure a square with calipers.
- `--square` is in **metres**: 25 mm is `--square 0.025`. If you measured 25.3 mm, type `--square 0.0253`
  (not 25.3: that would be 1000 times too big).
- Tape it flat on something stiff (cardboard, a clipboard). A bent board gives bad results.

### Intrinsics

1. Stop the operator page if it is running (it holds the camera): `sudo systemctl stop grappstar`.
2. Take 20 pictures of the board:

   ```bash
   python3 -m grappstar snap --n 20
   ```

   Before each picture, hold the board in a new place: near and far, left, right, top, bottom corners, and
   tilted. Keep it still, avoid glare, then press Enter. The pictures are saved as `calib/images/snap_00.png`
   to `snap_19.png`.
3. Work out the intrinsics from them:

   ```bash
   python3 -m grappstar calibrate intrinsics --images calib/images/*.png --square 0.025
   ```

4. It prints the result and saves `calib/intrinsics.json`. Check `rms` (the error in pixels): under 0.5 is good,
   over 1 means take the pictures again. `n_used` is how many pictures it could use.

### Mount

1. Fix the camera in its final position on the rover. If it moves later, redo this part.
2. Rover on flat ground. Lay the board flat on the ground in front of it, all of it in view.
3. Take one picture, then work out the mount:

   ```bash
   python3 -m grappstar snap --n 1 --out calib/ground
   python3 -m grappstar calibrate mount --image calib/ground/snap_00.png --square 0.025
   ```

   It uses `calib/intrinsics.json` from the step above automatically (it prints "using intrinsics"). If it
   prints a WARNING about an uncalibrated camera instead, do the Intrinsics part first.
4. It saves camera height, pitch and roll to `calib/rovercam.json`. Check the height against a tape measure.
5. Check: put an orange object at a distance you measured with a tape (for example 5.0 m) from the ground point
   under the rail pivot (the launcher's main axis, D29), click it on the page, and compare the range the page shows
   (it adds the camera offset to the pivot by itself). Within about 5 percent is good.

## 8. Operator page tour and a full dry mission in sim

Start the simulator and open the page (step 3):

```bash
python3 -m grappstar serve --sim
```

What is on the page (layout may differ a little from this list):

- **Camera views**: rover camera and dart camera, live. Tap (or click) the rover view to mark a target, or press
  **Auto target**.
- **Base panel**: link, state, mode, drum position, aim angle, current, battery, fault.
- **Dart panel**: link, phase (IDLE, ARMED, FLIGHT, LANDED, ANCHORING, ANCHORED, RELEASING, FAULT), battery.
- **Mission panel**: current step, history, **Preflight**, **Start mission**, **Abort**, and the **prompt** box.
  The mission stops and asks you before each big move. The confirmations, in order: **Base may move**, **Aim**,
  **Cock**, **Fire** then **FIRE NOW** (two presses), and the landing verdict (**Anchor** or **Retrieve and
  reshoot**). After "Base may move" the base is LIVE: drum shifts, aiming and small wiper turns happen between
  prompts.
- **Recover dart**: if a mission is aborted while the dart is still out, the Mission panel shows "The dart is still
  out" with a **Recover dart** button. It releases if needed, wraps, reels the dart in, unwraps and goes back to
  launch. Keep clear of the line.
- **LAUNCHER LOADED** (red banner at the top): the spring is drawn. Hands off the rail. Press **Decock** in the
  banner to unwind it before you touch the launcher.
- **Target**: range and bearing of the clicked or auto found target, and the ballistic solution (pitch, flight time).
  A typed range is measured from the ground point under the rail pivot (D29), not from the camera or the rover nose;
  tapped targets get the camera offset added automatically.
- **Results**: after a flight, the landing zone verdict (GO, NO_GO or UNSURE with reasons), the landing image, the
  aerial map built from the dart video, and the flight analysis.
- **Manual**: jog the wiper and drum motors, dart keys, record on or off; **Unwrap** brings the drum back through the
  wrap window to NEUTRAL with the take-up running (the way out when the drum stopped inside the window: there `mode`
  and drum jogs are refused).
- **STOP** (wiper off now) and **ESTOP** (wiper off and drum to neutral). Always visible at the top. Keyboard:
  **Esc** is STOP from anywhere on the page (Space too, when you are not typing). Holding STOP for 1 s is ESTOP.
  Know where they are before you press anything else.

Security note: the page needs no setup. The server puts a fresh session token into the page each time it
starts, and every button press must carry it, so another web site cannot press buttons for you. After a server
restart the page reloads itself to get the new token. Anyone on the same WiFi can still open the page, so for a
password add `--token SOMEPASSWORD` to `serve` and open `http://<jetson>:8080/?token=SOMEPASSWORD` once on each
phone or laptop (it remembers it). This is plain HTTP, not encryption. STOP and ESTOP always work from the page,
with or without a token.

The mission steps, in order (the page highlights the current one):

| Step | What happens | You do |
|---|---|---|
| PREFLIGHT | links, faults, dart seated, battery, videos, a target set | fix what it lists, then Retry; then confirm "Base may move" |
| LAUNCH_MODE | drum to LAUNCH (about 4 s per drum step) | |
| AIM | pitch from the ballistics table; the page says how far to turn the rover | turn the rover, press Aim |
| COCK | the base draws the spring (up to about 20 s) | confirm Cock |
| ARM | the dart's landing detector is armed (it only records; you decide on anchoring) | |
| FIRE | two confirmations, then the shot (FIRED, then the launcher RESET) | Fire, then FIRE NOW |
| FLIGHT | waits for the dart to say LANDED (or the dart video to go still) | |
| LANDING_ZONE | landing report, map, verdict; the base is told how much line is out | Anchor, or Retrieve and reshoot |
| RAIL_LOW | if the rail is above 15 degrees, it is lowered to its minimum pitch before the tow (skipped on a reshoot) | |
| BASE_NEUTRAL | launcher at rest, drum to NEUTRAL | |
| WRAP | the capstan wraps go on while the line is still slack; the drum parks at WRAPPED (about 35 to 40 s, drum 8.1 deg/s) | |
| ANCHOR | the dart digs in (kick, foot, drills) | if it reports warnings, choose Tow anyway or Release and reshoot |
| TIDY | slack wound onto the storage reel for a few seconds | |
| FLY_IN | drum on to REEL: the capstan is now driven | |
| CHECK | a gentle test pull must reach the check tension and hold it | |
| TOW | reel to the tension limit and hold | drive the rover gently, then Take up slack or Rover free |
| RELEASE | base stops (the worm holds the line), then the dart unscrews and folds | |
| RETRIEVE | capstan reels in until about 1.5 m is left | |
| UNWRAP | capstan wraps come off, drum back to NEUTRAL | |
| FINAL_APPROACH | the storage take-up pulls the last metre until the dart is seated (slow, about 0.058 m/s at duty 0.3) | |
| LAUNCH_READY | drum back to LAUNCH for the next shot | Shoot again, or Abort to finish |
| DONE | mission over; the log folder is shown | |

A NO GO verdict comes before the wrap: the dart is not anchored, so it comes home on the take-up alone.

If you abort after COCK and before FIRE, the spring stays drawn and the red **LAUNCHER LOADED** banner shows:
press **Decock** (in the banner or the Manual section) before you touch the launcher.

Dry mission in sim, step by step:

1. Set the target first: tap it on the rover view, or press **Auto target**. Check the range and solution.
2. Press **Preflight**. Everything should go green (base link, dart link, cameras, target). If it lists a
   problem, fix it and press Preflight again.
3. Press **Start mission**. Answer each prompt: Base may move, Aim, Cock, Fire, then FIRE NOW.
4. Watch the dart view: the simulated flight, impact and landing.
5. Read the landing verdict and look at the map. Press Anchor (or Retrieve and reshoot).
6. Follow the prompts through anchoring, tensioning, release and reeling in.
7. Try **STOP** (or press Esc) and **ESTOP** in the middle of a second run and check the mission stops and says
   why. If the dart is still out, press **Recover dart**. If the banner says LAUNCHER LOADED, press **Decock**.
8. Replay the recorded mission's vision:

   ```bash
   python3 -m grappstar replay <logdir>
   ```

   where `<logdir>` is the mission log folder (its path is printed in the terminal and shown on the page when
   the mission ends).

## 9. First real mission checklist

Before the first real launch:

- [ ] Steps 3 to 8 all worked. `python3 -m pytest` passes on the Jetson.
- [ ] Clear area: nobody in front of the launcher or anywhere along the flight path or the line. Check behind the
      target too. Everyone knows the shout for stop.
- [ ] Line attached at both ends (dart and reel) and laid so it cannot snag on the rover, feet or wires.
- [ ] Tow limit set, and you know what it is. The rule: the dart's hand-pull holding force (TEST_PLAN stage 8 step 2)
      minus 10 N, and 40 N for the first field tows until the wrap cup ring has been checked for wear (D46);
      firmware cap 55 N at the dart. Set it in two places: the base
      (`python3 -m grappstar base cmd '{"cmd": "set", "key": "limit_n", "val": 40}'`) and the Jetson (`settings.json` key `tow_limit`,
      or `serve --tow-limit <N>`, or the page's "Tow limit N at the dart" field). The default 40 N is only a start.
- [ ] Know the stop rule (the same in every GrappStar document):
      1. ESTOP or the page's STOP button.
      2. Pull lead #2's XT60 (motors off, the Jetson stays up).
      3. The main battery.
      Cutting power FREEZES a drawn spring and a taut line (the worm drives hold them); it does not release them.
      Treat a cocked launcher and a loaded line as live until Decock and unwrap.
- [ ] The base is live from the "Base may move" confirmation until DONE.
- [ ] Battery charged (3S full is 12.6 V), 20 A fuse fitted. Battery unplugged while you check any wiring.
- [ ] Antennas on the VTX and RC832 before power. Dart view clean on the page.
- [ ] Dart status OK (`python3 -m grappstar dart status`), base ping OK, preflight all green.
- [ ] Recording on, so you can replay it.
- [ ] HARD PREREQUISITE before any tow: `free_a` and `k_n_per_a` calibrated (firmware/base/CALIBRATION.md steps (d)
      free running current and (f) tension) and `limit_n` set. Reason: without a load cell the tension limit is estimated from the wiper current;
      with factory constants a simulated tow reached 61 N against a 40 N limit. The mission refuses the TOW step
      while `k_n_per_a` is still a firmware default (8, 17, 14.8 or 13.1) and there is no load cell.
- [ ] Start with a short shot at low pitch.
- [ ] Know the launcher's reachable range: it comes from `base/out/ballistics.json` (copied in at build time;
      `python3 -m grappstar solve <range>` answers for one range). The page says "not reachable" for anything
      outside it: move the rover instead.

## 10. Troubleshooting

| Problem | Likely cause | Fix |
|---|---|---|
| No `/dev/grappstar-base` | base serial not saved yet, rules not installed, cable in the UART port, charge-only cable, or a different USB ID | `ls /dev/ttyACM* /dev/ttyUSB*`, `lsusb` (expect 303a:1001). Use the DevKitC port marked USB and a data cable. With only the base plugged in, rerun `bash setup/install.sh --no-apt`, answer `y`, and replug (step 4). If lsusb shows another ID, edit `setup/99-grappstar.rules` |
| Only `/dev/grappstar-base-uart` | cable in the UART/COM port | move it to the port marked USB |
| `/dev/ttyUSB0` appears then vanishes (CH340) | brltty grabs it | `bash setup/install.sh --remove-brltty` |
| `Permission denied` on the serial port or camera | not in groups dialout or video yet | `groups` should list both. If not, `bash setup/install.sh --no-apt`, then log out and in (or reboot) |
| Base ping times out | wrong firmware USB mode, or another program has the port | check the firmware is built with "USB CDC On Boot: Enabled". Stop the service (`sudo systemctl stop grappstar`) or other `serve` before using `base` commands |
| Camera black, or JPEG errors like "Corrupt JPEG data" | format mismatch with NVIDIA's JPEG library (seen in August) | set the format explicitly: `--fourcc YUYV`, or `--fourcc MJPG` if YUYV does not offer the size. Check sizes with `v4l2-ctl --list-formats-ext` |
| Camera "busy" or will not open | another program has it (Cheese, a second `serve`, the service, calibrate) | close it. `sudo fuser /dev/video*` shows who holds it |
| Dart view is snow | RC832 channel does not match the VTX, antenna missing, dart off or too far | step the RC832 channel, check both antennas, move closer |
| Dart view stuck or blue screen | RC832 has no signal, dongle not getting video | check the yellow video cable and RC832 power |
| Dart unreachable (`dart status` fails, no ping to 192.168.4.1) | Jetson not on KEDGE, dart off, or out of range | `nmcli device` (is the WiFi on `kedge`?), `nmcli connection up kedge`, power the dart, move closer |
| Page unreachable from the phone | phone on a different network, wrong address, or serve not running | phone must be on KEDGE (A), GRAPPSTAR-ROVER (B) or the router (C). Find the address with `hostname -I`. On the Jetson, `curl -I http://localhost:8080` checks serve is up. `systemctl status grappstar` if using the service |
| Phone keeps leaving KEDGE | it wants internet | tell it to stay connected; turn off mobile data switching; or use option B |
| `numpy.core.multiarray failed to import`, "compiled using NumPy 1.x cannot be run in NumPy 2" | numpy 2 got installed by pip | `python3 -m pip install --user 'numpy>=1.21,<2'`, then `python3 setup/check_env.py` |
| Two OpenCVs (`cv2` version or GStreamer NO in check_env on the Jetson) | `pip install opencv-python` was run | `python3 -m pip uninstall opencv-python opencv-python-headless`, keep the apt one |
| A program wants `snap install` | snap is broken on JetPack 6.2 | use apt (or the project's apt or pip instructions) instead |
| Port 8080 already in use | the service is running, or an old `serve` | `sudo systemctl stop grappstar`, or close the other terminal. Or use `--port 8081` |

## 11. File map

```
jetson/
  README.md              this guide
  INTERFACES.md          how the modules fit together (for programmers)
  requirements.txt       Python packages (numpy below 2, pyserial, pytest; OpenCV from apt)
  pytest.ini             test settings (tests/ folder)
  setup/
    install.sh           installs everything (step 2)
    check_env.py         prints what is installed and plugged in
    99-grappstar.rules   udev rules: /dev/grappstar-base, -base-uart, -rovercam, -dartcam
    grappstar.service    systemd unit to start the page at boot
    hotspot.sh           optional operator hotspot on a USB WiFi adapter (option B)
  grappstar/
    __main__.py, cli.py  the command line (python3 -m grappstar ...)
    app.py               owns the base, dart, cameras and mission; what the page talks to
    mission.py           the mission steps and operator prompts
    rover.py             rover side helpers
                         (defaults: mission.MissionConfig, app.AppConfig, cli.py)
    jlog.py              mission logs
    ballistics.py        aim solution from the ballistics table
    data/                placeholder ballistics table
    comms/               base.py (serial), dart.py (HTTP), mock_base.py, mock_dart.py
    video/               camera capture, discovery, frame quality, recording, replay
    vision/              flight.py, mosaic.py (aerial map), terrain.py (landing verdict), target.py (aiming)
    sim/                 simulated dart flight video and terrain textures
    ui/                  operator web page
  tests/                 one test file per module, run with python3 -m pytest
```
