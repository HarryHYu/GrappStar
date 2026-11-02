# KEDGE DART: FULL HANDOVER
GrappStar / KEDGE. Harry Yu, Sydney. Updated 1 Nov 2026. Current design = v26.

This file is the whole state of the dart sub-project: parts, measurements, every wire and pin, what was built and tested on the bench, what died and why, the CAD source, the checking scripts, the viewer, the open items and the build-day plan. Reel and launcher are separate (only what touches the dart is here).

---------------------------------------------------------------------------------------------------

## 1. WHAT THE DART IS

A rover-launched projectile anchor. It flies nose first, lands, then:
1. Kick leg swings 200° over the nose and plants a foot auger ahead of the dart (stops the nose lifting).
2. Two drill legs swing down 38° on the flanks and screw Ø14 augers 38 mm into the ground, raked tailward (toward the tether pull).
3. Tether to the rover's reel runs from the tail eye.

Hull: Ø42 x 167 long clamshell, split on the horizontal centreline, bottom half a plain cylinder for the launcher cradle. Camera on the centreline in the blunt nose. Everything prints in PLA on the X2D (PPA-CF only for reel gears).

---------------------------------------------------------------------------------------------------

## 2. PARTS LIST (dart only)

| Part | Qty | Status | Where |
|---|---|---|---|
| Seeed XIAO ESP32S3 (plain, no Sense) | 1 (+1 spare) | in hand | Amazon AU, 2 units |
| DRV8833 dual H-bridge carrier | 3 | in hand | |
| INA219 current/voltage sensor | 1 | in hand | |
| BNO085 IMU | 1 | in hand, UNTESTED on the new XIAO | |
| MP1584 buck converter | 2 | in hand, set to 8.12 V and 5.04 V | |
| 2S BMS, OTOTEC 8A stick 40.8x8x2.1 | 1 (6-pack) | ordered 24 Oct, Amazon AU | replaces the 47x24 10 A one |
| HIBATT IMR 18350 cells | 2 | in hand (one re-taped wrap) | |
| XTAR MC2 charger | 1 | in hand | charges the 18350s |
| N20 right-angle worm gearmotor 12 V (self-locking), 12x10x33 | 3 needed | 2 in hand, BUY ONE MORE (foot) | |
| Bringsmart-type straight gearmotor 12x10x24.3, M3x56 threaded shaft, 100 rpm 12 V | 2 needed | 3 in hand (1 spare) | drill pair |
| F693ZZ flanged bearing 3x8x4 | 4 | in hand | 2 per drill pod |
| M3 nyloc nuts | 2 | | auger tips |
| M3 grub screw | 1 | | foot auger hub |
| M3 screws ~8 mm | 8 | | clamshell radial screws |
| Zip ties / straps small | ~6 | | motors in pockets |
| Caddx Ant nano camera (analog) | 1 | in hand | Phaser FPV |
| HGLRC Zeus nano VTX (run at 25 mW) | 1 | in hand | MantisFPV |
| BetaFPV u.FL 5.8 GHz antenna | 1 (2-pack) | in hand | |
| Lockmaner RC832 5.8 GHz receiver | 1 | ordered, Amazon AU, due 26-29 Oct | rover side |
| DIGITNOW USB AV capture card | 1 | ordered, Amazon AU | rover side, Jetson sees a webcam |
| JST-XH 3-pin charge socket | 1 | | tail cap window |
| XT60 lead #1 with 10 A blade fuse holder | 1 | built | bench supply from the 3S LiPo |
| XT60 lead #2 with 20 A blade fuse | 1 | built | reel wiper motor |
| Zeee 3S LiPo (bench supply), iMax B6 clone charger | 1 | in hand | |

Bench kit: 3 breadboards, jumpers, solder, brass wool, tape, tweezers, multimeter (autoranging), USB-C data cable, Mac.

---------------------------------------------------------------------------------------------------

## 3. ALL CALIPER MEASUREMENTS (mm)

Motors
- Kick/deploy worm N20: body 12 x 10, worm box 10 x 12 (box 11.93 tall), 33.04 long box+motor without shafts, 29.97 shaft tip to tip, 9.94 worm box width.
- Drill motor (straight, threaded shaft): gearbox 11.95 tall, 24.27 face to back, shaft 2.84 dia (M3), 56.03 long past the face.

Boards (L x W x H)
- XIAO ESP32S3: 21.45 x 17.74, 22.75 incl. USB-C lip, 4.49 thick board+USB, 13.01 with header pins.
- DRV8833: 20.31 x 12.70, 7.19 with header body, 11.68 with pin tips. Modelled 3.5 with headers removed.
- INA219: 25.69 x 20.32, bare PCB 1.58, 4.70 with JST, 10.07 with screw terminal. Terminal to be removed.
- BNO085: 25.72 x 22.77, 4.77 with JST.
- MP1584: 22.26 x 16.96 x 3.98.
- 2S BMS old 10 A: 47.36 x 23.63 x 3.35 (too big, replaced).
- 2S BMS new OTOTEC stick (listing): 40.8 x 8 x 2.1.
- HGLRC Zeus nano VTX: 42.25 x 19.91 x 6.01, 13.55 across the waist.

Camera
- Caddx Ant body: 13.99 wide x 14.59 tall, 18.34 lens front to connector back, board 2.69 thick. Modelled 14 x 14.6, 15.6 deep + 2.7 lens.
- Ant bracket 18.93 x 18.95 x 8.30 (not used). Ant OSD joystick board 28.15 x 22.15 x 9.83 (bench only, not in the dart).

Power
- 18350 cell: dia 18.66 with wrap, 36.87 button to base, 36.15 body without button (same cell both ways).

Hardware
- F693ZZ: 3 x 8 x 4 (bag label).

Datasheet values used (not measured): N20 mount hole spacing and D-flat depth, nyloc, u.FL antenna.

---------------------------------------------------------------------------------------------------

## 4. ELECTRICAL: EVERY WIRE, EVERY PIN

### 4.1 Power tree (bench build as tested, and the dart build)

BENCH (what is wired now):
- Zeee 3S LiPo (~11.5-12.6 V) -> XT60 lead #1 -> 10 A blade fuse -> red/black to breadboard rails.
- MP1584 #1: IN+ = battery red, IN- = battery black. OUT set to 8.12 V = "8 V motor rail" (fake 2S). Trimmer turned with a small screwdriver while reading the multimeter on OUT.
- MP1584 #2: IN from battery. OUT set to 5.04 V = "5 V rail".
- INA219: VIN+ = MP1 OUT+, VIN- = the 8 V motor rail going to the DRV8833 VM pins (so it measures all motor current). VCC = XIAO 3V3, GND common, SDA = XIAO D4, SCL = XIAO D5.
- XIAO ESP32S3: 5V pin = MP2 OUT+ (5.04 V), GND = common ground. 3V3 pin supplies logic to DRV8833 VCC x3, INA219 VCC, BNO085 VIN.
- All grounds common on the blue rail.

DART (final):
- 2 x 18350 in series (2S, 6.0-8.4 V) -> OTOTEC 2S BMS (B+, B-, BM between cells, P+, P- out).
- P+ -> INA219 VIN+. INA219 VIN- -> motor rail (DRV8833 VM x3, direct, no buck needed: 2S IS the motor rail).
- P+ -> MP1584 #1 IN, OUT 5.04 V -> XIAO 5V, VTX 5 V in, camera 5 V in.
- MP1584 #2 is then spare in the dart (both are in the CAD rack in case the VTX wants its own rail; the Zeus nano takes 5 V so one is enough).
- JST-XH 3-pin charge socket in the tail: pin1 B-, pin2 BM, pin3 B+ (balance charge with the XTAR MC2 on the cells, or an external 2S charger).
- RULE: battery/BMS lead unplugged whenever a jumper moves or a sketch uploads.

### 4.2 XIAO ESP32S3 pin map (final, same as the bench)

| XIAO pin | Goes to | Function |
|---|---|---|
| D0 | DRV8833 #1 AIN1 | deploy N20 (biax worm, through-axle) |
| D1 | DRV8833 #1 AIN2 | deploy N20 |
| D2 | DRV8833 #1 BIN1 | kick N20 (self-locking worm, saddle) |
| D3 | DRV8833 #1 BIN2 | kick N20 |
| D4 | INA219 SDA, BNO085 SDA | I2C SDA |
| D5 | INA219 SCL, BNO085 SCL | I2C SCL |
| D6 | DRV8833 #2 AIN1 AND BIN1 (tied) | drill pair, both motors |
| D7 | DRV8833 #2 AIN2 AND BIN2 (tied) | drill pair, both motors. ONE drill motor is wired with its leads swapped at BOUT1/BOUT2 so the pair counter-rotates |
| D8 | DRV8833 #3 AIN1 | foot N20 (worm, in the kick foot housing) |
| D9 | DRV8833 #3 AIN2 | foot N20 |
| 3V3 | DRV x3 VCC, INA219 VCC, BNO085 VIN | logic 3.3 V |
| 5V | MP1584 OUT+ (5.04 V) | board power |
| GND | common | |

Motor outputs:
- DRV#1 AOUT1/AOUT2 -> deploy N20 red/black. BOUT1/BOUT2 -> kick N20 red/black.
- DRV#2 AOUT1/AOUT2 -> drill motor L. BOUT1/BOUT2 -> drill motor R with red/black SWAPPED.
- DRV#3 AOUT1/AOUT2 -> foot N20.
- Every DRV8833: VM = motor rail (8 V bench / 2S dart), GND = common, VCC = 3V3, nSLEEP tied to VCC (or left: the carrier pulls it up), nFAULT unused.
- DRV8833 carrier labels: the pins read AIN1 AIN2 BIN1 BIN2 on one side (printed "A1IN" style on some), OUT OUT OUT OUT on the other side in the order AOUT1 AOUT2 BOUT1 BOUT2. "VIN" on the carrier = VM. Harry's boards have 8 holes per side but the same 4 labels each (two rows, same nets).

I2C addresses: INA219 0x40 (A0/A1 open). BNO085 0x4A (0x4B if the ADR pad is bridged).

Camera chain (separate radio, no wires to the XIAO): Caddx Ant video out -> Zeus VTX video in, both on 5 V and common ground, u.FL antenna on the VTX, antenna in the tail cap slot. 5.8 GHz analog video to the RC832 on the rover -> DIGITNOW capture -> Jetson /dev/video0. Control is 2.4 GHz WiFi from the XIAO (ESP32S3 softAP or station to the rover).

### 4.3 Measured currents at 8 V (bench, INA219)
- N20 worm (kick/deploy): 20-27 mA free, ~180 mA stall.
- Drill pair: 60-70 mA free, ~700 mA stall (both).
- Foot/third drill motor: ~40 mA free, 500-600 mA stall.
Everything together under 2 A worst case, hence the 8 A stick BMS is fine.

### 4.4 Reel (DevKitC + BTS7960), for completeness
- ESP32-S3 DevKitC-1, board "ESP32S3 Dev Module", UART port (micro-USB, CP210x driver, DATA cable).
- BTS7960: RPWM = GPIO4, LPWM = GPIO5, R_EN = GPIO6, L_EN = GPIO7, VCC = 3V3 (one of the 3V3 pins), GND = one of the G pins. B+ B- = battery via XT60 lead #2 and a 20 A blade fuse (10 A blew on inrush). M+ M- = wiper motor, rebuilt to 2 wires (red + black). Gearbox opened, oscillating linkage removed, white worm wheel kept; drum and gears to be printed in PPA-CF.

---------------------------------------------------------------------------------------------------

## 5. WHAT HAPPENED ON THE BENCH (chronological, the important bits)

1. Built XT60 lead #1 with 10 A blade fuse. Battery measured 11.5 V.
2. Set MP1584 #1 to 8.12 V, MP1584 #2 to 5.04 V.
3. First XIAO (the Sense version with camera) wired to DRV8833 #1; deploy N20 ran from the keyboard sketch. Compile error "A1 conflicts" fixed by renaming pin constants (MA1 etc). Board must be selected as XIAO_ESP32S3 after every replug (IDE auto-detects "Ozobot DRVKit", wrong).
4. Second N20 on the same DRV: both run, take turns. Second DRV8833 with both drill motors on tied inputs, one reversed: they counter-rotate together.
5. INA219 in series on the 8 V rail; voltage stable, current rising per motor; readings above.
6. BNO085 wiring with the battery still plugged in during an upload: the first XIAO died (hot, no USB enumeration, red LED only). Replaced with 2 x plain XIAO ESP32S3 from Amazon. Camera moved to the analog Caddx Ant instead of the Sense camera.
7. New XIAO: BNO085 test sketch gives a BLANK Serial Monitor. Unresolved. Check: board XIAO_ESP32S3, correct port, 115200 baud, press reset, "USB CDC on boot" enabled; then run an I2C scanner before the BNO library.
8. iMax B6 clone: the round-plug-to-croc lead is the 12 V INPUT, not an output; it was shorted and smoked (my mistake). The banana-to-croc lead is the output. Settings: LiPo Balance, 2.5 A, auto 3S, with the balance plug in. Works.
9. 18350: one cell's wrap was peeled, re-taped. XTAR MC2 charges them.
10. Reel: wiper motor is a 2-speed with case/park wiring; runs on green + red; rebuilt to 2 wires; BTS7960 + DevKitC sketch greeting works; motor moved then stalled on the park linkage; gearbox opened and linkage removed.

---------------------------------------------------------------------------------------------------

## 6. FIRMWARE

### 6.1 Dart keyboard sketch (XIAO ESP32S3, Arduino IDE, board XIAO_ESP32S3, USB CDC on boot)
Libraries: Adafruit INA219, Adafruit BNO08x (both via Library Manager).
Keys in Serial Monitor (115200, "No line ending"): 
- `1`/`!` deploy N20 fwd/rev, `2`/`@` kick N20 fwd/rev, `3`/`#` drill pair fwd/rev, `4`/`$` foot N20 fwd/rev, `s` stop all, `i` INA219 readout, `b` BNO085 readout, `+`/`-` speed.

```cpp
// KEDGE dart bench/flight firmware v1: keyboard control, INA219 current, BNO085 attitude.
// Board: XIAO_ESP32S3. Serial 115200. Battery lead OUT while uploading.
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_BNO08x.h>

// DRV8833 inputs
const int DEP_A = D0, DEP_B = D1;     // deploy N20 (DRV#1 A)
const int KICK_A = D2, KICK_B = D3;   // kick N20   (DRV#1 B)
const int DRL_A = D6, DRL_B = D7;     // drill pair (DRV#2 A+B tied)
const int FOOT_A = D8, FOOT_B = D9;   // foot N20   (DRV#3 A)
const int SDA_PIN = D4, SCL_PIN = D5;

Adafruit_INA219 ina;
Adafruit_BNO08x bno(-1);
sh2_SensorValue_t ev;
int spd = 255;   // 0..255 PWM

struct M { int a, b; int ch; };
M mot[4] = {{DEP_A, DEP_B, 0}, {KICK_A, KICK_B, 2}, {DRL_A, DRL_B, 4}, {FOOT_A, FOOT_B, 6}};
const char* nm[4] = {"deploy", "kick", "drills", "foot"};

void drive(int i, int dir) {            // dir -1,0,+1 ; PWM on one input, other low
  M &m = mot[i];
  if (dir == 0) { ledcWrite(m.ch, 0); ledcWrite(m.ch + 1, 0); }
  else if (dir > 0) { ledcWrite(m.ch, spd); ledcWrite(m.ch + 1, 0); }
  else { ledcWrite(m.ch, 0); ledcWrite(m.ch + 1, spd); }
}
void stopAll() { for (int i = 0; i < 4; i++) drive(i, 0); }

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    ledcSetup(mot[i].ch, 20000, 8); ledcAttachPin(mot[i].a, mot[i].ch);
    ledcSetup(mot[i].ch + 1, 20000, 8); ledcAttachPin(mot[i].b, mot[i].ch + 1);
  }
  stopAll();
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!ina.begin()) Serial.println("INA219 not found");
  if (!bno.begin_I2C()) Serial.println("BNO085 not found");
  else bno.enableReport(SH2_ROTATION_VECTOR, 50000);
  Serial.println("KEDGE ready. 1/! deploy  2/@ kick  3/# drills  4/$ foot  s stop  i current  b imu  +/- speed");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1': drive(0, +1); break; case '!': drive(0, -1); break;
      case '2': drive(1, +1); break; case '@': drive(1, -1); break;
      case '3': drive(2, +1); break; case '#': drive(2, -1); break;
      case '4': drive(3, +1); break; case '$': drive(3, -1); break;
      case 's': stopAll(); Serial.println("stop"); break;
      case '+': spd = min(255, spd + 25); Serial.println(spd); break;
      case '-': spd = max(0, spd - 25); Serial.println(spd); break;
      case 'i': {
        float v = ina.getBusVoltage_V(), mA = ina.getCurrent_mA();
        Serial.printf("rail %.2f V  %.0f mA\n", v, mA); break; }
      case 'b': {
        if (bno.getSensorEvent(&ev) && ev.sensorId == SH2_ROTATION_VECTOR)
          Serial.printf("q = %.3f %.3f %.3f %.3f\n", ev.un.rotationVector.real, ev.un.rotationVector.i, ev.un.rotationVector.j, ev.un.rotationVector.k);
        else Serial.println("no IMU event"); break; }
    }
  }
}
```

Flight sequence (to add once bench is done): on command from the rover over WiFi: kick to 200° (run until INA219 current spike = foot auger loaded, then stop), foot auger on 10 s, deploy to 38° (time-based, ~2 s on the worm), drills on until current plateau, report IMU quaternion for the pitch/roll of the planted dart, hold.

### 6.2 Reel sketch (DevKitC-1 + BTS7960), keys f/r/s/1-9

```cpp
const int RPWM = 4, LPWM = 5, R_EN = 6, L_EN = 7;
int duty = 128;
void setup() {
  Serial.begin(115200);
  pinMode(R_EN, OUTPUT); pinMode(L_EN, OUTPUT); digitalWrite(R_EN, HIGH); digitalWrite(L_EN, HIGH);
  ledcSetup(0, 20000, 8); ledcAttachPin(RPWM, 0); ledcSetup(1, 20000, 8); ledcAttachPin(LPWM, 1);
  Serial.println("reel ready: f fwd, r rev, s stop, 1-9 speed");
}
void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c >= '1' && c <= '9') duty = (c - '0') * 28;
  if (c == 'f') { ledcWrite(0, duty); ledcWrite(1, 0); }
  if (c == 'r') { ledcWrite(0, 0); ledcWrite(1, duty); }
  if (c == 's') { ledcWrite(0, 0); ledcWrite(1, 0); }
}
```

---------------------------------------------------------------------------------------------------

## 7. CAD: v26 GEOMETRY SUMMARY

Coordinates: x along the hull, nose tip x=0, tail x=167. y across, z up. Ground z=-21 when the dart lies flat.

| Thing | Position / size |
|---|---|
| Hull | Ø42 outer, Ø39 bay, x 20-159, split z=0. 4 radial M3 at z ±3 into the tray end discs (x 64 and 114). |
| Nose | x 0-20, frustum Ø22 flat tip to Ø42, spigot to x 28, camera pocket 14.4 x 15 on axis x 2.5-29, Ø9.8 lens hole. |
| Tail cap | x 159-167, spigot x 155-159, tether eye at x 170, JST window under, u.FL slot on top. |
| Kick saddle | on the hull top x 24-61, 14.6 wide x 12.6 tall, sunk 1 mm; N20 pocket open at the top, strap groove x 38-41; shaft hole on +y at x=30, z=26.3. Ear fin y 16.4-20.4, x 22-38, up to z 32.3. |
| Kick leg | hub Ø12 x 8 at (30, 12, 26.3), D-bore; stub Ø3 into the ear; pole Ø8 tube from x 35 to 120 (85 long); foot housing 14.6 x 12.6 x 39.6 from x 116; foot N20 inside, output up; foot auger Ø14 x 40 at x 149. Swing 0-200°. |
| Drill axle | Ø3 D-shaft, 60 long, at x=56, z=5, through Ø10 bosses to y ±21.8. |
| Deploy N20 | worm box on the axle at y -15.9..-5.9, body forward x 29-50, z -1..11. |
| Rack | x 30-60, +y side: MP1584 on floor z -16.9 (y -9.5..7.5), MP1584 on shelf z -11.1 (y -2..15), INA219 on top z -5.4 (y -3.2..17.2). End walls 1.5. |
| Tray | discs x 62-66 and 112-116 (Ø38.6 x 4, wire windows, radial M3 tapping holes). Decks (tops) z -5.2: DRV#1 + XIAO side by side then DRV#2 + DRV#3; z 2.6: VTX + BMS stick; z 10.4: BNO085. Stack x 66-112. |
| Cells | 2 x 18350 side by side at y ±9.6, x 118-155. |
| Drill pods | clevis head Ø12.6 on the axle at y ±29, housing 14.6 x 12.6 x 28 (motor pocket open top, strap groove), tapered nose 9 long with two 693ZZ seats, auger Ø14 x 54 (Ø5.5 core, 1.4 flight, 11 pitch, 12 tip, nyloc pocket at 43.4). Folded along the flank x 50-146. Deploy 38°. |
| Printed mass | ~84 cm3 = ~104 g PLA solid. Electronics + cells ~200 g. |

Folded envelope: 167 x 76 x 94. Deployed: foot 90 ahead of the nose, 38 in; drills 38 in.

Rules learned (do not regress):
- A drill pointing down deployed points up folded; the only in-envelope fix is a second hinge, rejected (no springs/actuators). Fixed foot.
- Legs can't stow inside a cylinder on single hinges. Belly stays a plain cylinder; the barrel is cut around top and flanks. No sabot rings.
- Always: exact-mesh collision sweep (STL round trip breaks watertightness, use the saved vertices), every sub-part tested on its own, ground-plane check for every deployed part, look at a 24-angle screenshot sheet BEFORE sending anything.
- Harry's standards: smooth surfaces (96-192 facets), minimum mass, nothing floating, nothing inside something else, every part explained.

Open review items: kick N20 torque check (leg ~50 g at 100 mm = 0.5 kg cm, worm rated must be >1 kg cm); steel stub in the ear instead of printed; BNO085 blank-serial bug; third N20 to buy; stick BMS to caliper when it lands; deployed foot housing stands 60 mm ahead of the lens, 12-19 mm to one side (in the camera's picture, right edge).

Next step agreed: port to build123d with fillets and STEP export, same dimensions, minimise length and mass at 1.5 walls, print orientations without supports, full audit each iteration, then the build-day pack.

---------------------------------------------------------------------------------------------------

## 8. CAD SOURCE: kedge_v26.py (trimesh + manifold3d + shapely; `python3 kedge_v26.py out26`)

```python
