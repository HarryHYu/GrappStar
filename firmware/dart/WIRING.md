# KEDGE dart wiring v4 (DRV8833 #3 removed)

This replaces Appendix A of `kedge/CLAUDE.md` (the v29 list of 53 wires). The new list has 50 wires:
`wiring/wire_cut_list.csv`. Diagram: `wiring/wiring.svg` / `wiring/wiring.png`. Changes only: `wiring/changes_from_v29.csv`.
Regenerate everything with `python3 firmware/dart/tools/wires_v4.py` (lengths from the v30 model, same rule as before).

What changed:
- DRV8833 #3 leaves the dart and goes to the base station (selector drum). **The dart geometry does not change.** v30 is
  still the default build; the DRV#3 pocket on the keel in the sled simply stays empty.
- DRV#2 channel A now drives **both** drill motors (wired in parallel on AOUT1 / AOUT2). The pair stalls at about
  0.7 A, inside one channel's 1.2 A. The right drill's leads stay swapped so the augers counter-rotate.
- DRV#2 channel B now drives the **foot** motor (it was on DRV#3).
- The pin bridge on DRV#2 (AIN1 to BIN1, AIN2 to BIN2) is **gone**. Channels A and B now have their own inputs.

Pin map (firmware v4):

| XIAO pin | Goes to | Motor |
|---|---|---|
| D0 / D1 | DRV#1 AIN1 / AIN2 | deploy N20 |
| D2 / D3 | DRV#1 BIN1 / BIN2 | kick N20 |
| D8 / D9 | DRV#2 AIN1 / AIN2 | both drill motors |
| D6 / D7 | DRV#2 BIN1 / BIN2 | foot N20 (these two pins twitch at reset: harmless on the slow worm) |
| D4 / D5 | INA219 + BNO085 SDA / SCL | I2C |
| 3V3 | DRV#1 SLP, DRV#2 SLP, INA219 VCC, BNO085 VIN | logic power |

## Rules for every step

1. **Battery unplugged.** Pull the power loop plug (the ON switch in the tail) before you touch any wire, and leave it
   out until the step says so. The first XIAO died from wiring with the battery live.
2. USB unplugged while you solder. Never have USB and the power loop plug in at the same time unless a step says so.
3. Heat shrink on before you solder, not after. Keep the iron off the PLA.
4. After each solder joint, tug the wire gently. It must not move.

## Path A: you already wired the v29 harness (convert it)

Do these steps in order. Wire numbers in brackets are the v29 numbers, then the new numbers.

**Step A1. Battery unplugged.** Pull the power loop plug. Unplug USB. Take the cells out if the hull is open.

**Step A2. Battery unplugged. Remove DRV#3.** Unsolder and remove these three wires completely:
- [13] DRV#2 VM to DRV#3 VM (red, 22 AWG)
- [17] DRV#2 GND to DRV#3 GND (black, 22 AWG)
- [27] DRV#2 SLP to DRV#3 SLP (red/white, 28 AWG)
Wires 13 and 17 share the DRV#2 VM and GND pads with wires 12 and 15: leave 12 and 15 in place and tug them after.
Also unsolder the foot motor wires [52] [53] from DRV#3, and remove the XIAO D6 / D7 wires [42] [43] completely
(desolder them at the XIAO end too and throw them away). Lift DRV#3 out of the sled and put it in a labelled bag for
the base station. The pocket stays empty.

**Step A3. Battery unplugged. Cut the bridge on DRV#2.** In v29 the D8 wire went to AIN1 *and* BIN1 (bridged), and the
D9 wire to AIN2 *and* BIN2. Desolder the jumper wire or solder blob that joins AIN1 to BIN1 and AIN2 to BIN2
(desoldering braid). The 45 mm D8 and D9 wires stay. After this:
- D8 goes to **AIN1 only** (new wire 37)
- D9 goes to **AIN2 only** (new wire 38)
Check with the meter on beep: AIN1 to BIN1 must NOT beep. AIN2 to BIN2 must NOT beep.

**Step A4. Battery unplugged. Move the right drill to channel A.** Unsolder the right drill wires from BOUT1 / BOUT2 and
solder them to the channel A pads, keeping the swap:
- right drill **black** to **AOUT1** (the same pad as the left drill red) (new wire 47)
- right drill **red** to **AOUT2** (the same pad as the left drill black) (new wire 48)
Two wires in one pad hole, one pad at a time: desolder the left drill red from AOUT1 (braid or pump). Strip 3 mm of
it and of the right drill black, twist the two bare ends together, tin them, push them through the hole, solder. Trim
the ends flush on the far side. Check with a magnifier that no strand touches the next pad. Tug both. Then the same on
AOUT2 with the left drill black and the right drill red.

**Step A5. Battery unplugged. Foot inputs to DRV#2 B.** Cut two new 45 mm 28 AWG wires (purple and grey):
- XIAO D6 to DRV#2 **BIN1** (purple, new wire 39)
- XIAO D7 to DRV#2 **BIN2** (grey, new wire 40)

**Step A6. Battery unplugged. Foot motor to DRV#2 B.** The foot wires are now longer, because DRV#2 sits in the board
bay behind the cells, not in the sled. Cut two new 330 mm silicone 26 AWG wires (red and black). Run them forward
along the top of the hull through the disc notches (with the camera wires), out the skin hole, a 60 mm loop round the
kick hub, then down the pole channel:
- DRV#2 **BOUT1** to foot N20 **+ (red)** (new wire 49)
- DRV#2 **BOUT2** to foot N20 **- (black)** (new wire 50)
If the old 255 mm foot wires are already through the pole channel, you may instead solder an 80 mm extension at the
DRV#2 end: joint inside the hull, heat shrunk, never in the hub loop.

**Step A7. Battery unplugged. Meter checks** (meter on beep, nothing powered):
- DRV#2 AOUT1 beeps to the left drill red AND the right drill black.
- DRV#2 AOUT2 beeps to the left drill black AND the right drill red.
- DRV#2 BOUT1 beeps to the foot red, BOUT2 to the foot black.
- No beep between any AOUT pad and any BOUT pad.
- No beep between XIAO D6, D7, D8, D9 (each pair).
- Meter on ohms (not beep): AOUT1 to AOUT2 and BOUT1 to BOUT2 read a few ohms or more (that is the motor winding),
  never 0.0.
- VM to GND: a short chirp is a capacitor charging and is fine; a steady beep is a short: find it before power.
Then go to "Power-on check" below.

## Path B: fresh build (the whole harness)

Use `wiring/wire_cut_list.csv` for colours, gauges and cut lengths. Cut every wire 10 mm longer than listed if you are
unsure; trim at the end. Work in this order. Every step starts with the battery unplugged.

**Step B1. Battery unplugged. Cells and BMS** (wires 1 to 8). Solder the plate tabs before the plates go in their
recesses. The series link (wire 2) carries the full current: 22 AWG. Charge socket pins: 1 = B-, 2 = BM, 3 = B+.

**Step B2. Set the buck before it is wired in** (as in the run sheet step 6.2). The MP1584 on its own: 3S LiPo on
XT60 lead #1 to MP1584 IN+ / IN-, meter on DC volts across OUT+ and OUT-, turn the small brass screw until it reads
**5.04 V**. Disconnect the LiPo. Then fit the capacitor across OUT (long leg OUT+, striped leg OUT-; rated 10 V or more).

**Step B3. Battery unplugged. Motor rail** (wires 9 to 15). Power loop socket pin 2 to INA219 VIN+, the buck IN+ taps
the same VIN+ pad, INA219 VIN- to DRV#1 VM, then DRV#1 VM to DRV#2 VM. Grounds: BMS P- to buck IN-, buck IN- to DRV#1 GND,
DRV#1 GND to DRV#2 GND. **The chain ends at DRV#2.** There is no DRV#3.

**Step B4. Battery unplugged. 5 V** (wires 16 to 21). Click the u.FL antenna onto the VTX now: never power a VTX
without its antenna. Buck OUT+ to XIAO 5V **through the Schottky diode, stripe
towards the XIAO**. Buck OUT- to XIAO GND. Buck OUT to the VTX, VTX pads on to the camera.

**Step B5. Battery unplugged. Camera video** (wire 22): camera yellow to VTX video in, twisted with the camera ground.

**Step B6. Battery unplugged. Find the SLP pad** (before wires 23, 24). On each DRV8833 board find the pad named SLP,
nSLEEP or EEP. Meter on **ohms** (beep mode only catches a dead short): that pad to VM. **Under 50 ohms means it is
tied to the motor supply on this board: do not connect 3V3 to it.** (Pololu boards pull SLP up through a resistor of
tens of kilohms, which is fine; on a generic board with a solder bridge from SLP to VCC, open the bridge first.)
Then: XIAO 3V3 to DRV#1 SLP (wire 23), DRV#1 SLP to DRV#2 SLP (wire 24).

**Step B7. Battery unplugged. Sensors** (wires 25 to 32). 3V3 and GND from the XIAO to the INA219, then on to the
BNO085. SDA (blue): D4 to INA219 SDA to BNO085 SDA. SCL (green): D5 to INA219 SCL to BNO085 SCL.

**Step B8. Battery unplugged. Driver inputs** (wires 33 to 40): D0, D1 to DRV#1 AIN1, AIN2 (deploy). D2, D3 to DRV#1
BIN1, BIN2 (kick). D8, D9 to DRV#2 AIN1, AIN2 (drills). D6, D7 to DRV#2 BIN1, BIN2 (foot). **No bridges between pads.**

**Step B9. Battery unplugged. Motors, leave them hanging outside the hull for now** (wires 41 to 50):
- DRV#1 AOUT1 / AOUT2 to deploy red / black.
- DRV#1 BOUT1 / BOUT2 to kick red / black.
- DRV#2 AOUT1 to left drill red **and** right drill black; DRV#2 AOUT2 to left drill black **and** right drill red
  (two wires per pad, twisted and tinned together exactly as in step A4).
- DRV#2 BOUT1 / BOUT2 to foot red / black (330 mm silicone).

**Step B10. Battery unplugged. Meter checks:** the list in step A7, plus 5V to GND and 3V3 to GND: no steady beep.

**Optional, if you have them (recommended):**
- 470 to 1000 uF, 10 V or more, across the XIAO 5V and GND pins, on the XIAO side of the Schottky diode (long leg to 5V).
  It keeps the XIAO running through a short power break at impact (the JST plug is the only power path), so the
  flight state survives. Glue or tape the power loop plug so it cannot shake loose.
- 100 uF, 16 V, across DRV#1 VM and GND (long leg to VM): bulk capacitor for the motor rail (TI asks for 10 uF or more).

## Power-on check (both paths)

1. **Battery unplugged.** Motors still outside the hull, legs free to turn, nothing touching the augers.
2. USB only, power loop plug out: flash the firmware (README.md). Serial Monitor at 115200. Type `z`: expect `0x40` and `0x4a` (or `0x4b`).
3. Antenna on the VTX. Unplug USB. Insert the power loop plug. Nothing should move, nothing should get warm. Join WiFi `KEDGE`.
4. Open `http://192.168.4.1`. Status line must show `IDLE`, `INA219 ok`, `BNO085 ok`, rail about 7 to 8.4 V.
5. Go through BENCH_TEST.md: steps 0 to 6 (directions, free currents) on the loose motors now, the rest after
   assembly (the table at the top of BENCH_TEST.md says which).

**Drill direction check** (BENCH_TEST.md step 4): hold "drills in" (key `3`) and look at each auger from its motor
end, looking toward the tip. The left auger (right-hand thread) must turn **clockwise**, the right auger (left-hand thread)
**anticlockwise**: then both screw into the ground.
- Left auger wrong: battery unplugged, swap the two thin input wires at DRV#2 AIN1 and AIN2 (D8 and D9). This
  reverses both drills. Check again.
- Then if the right auger is wrong: battery unplugged, swap the right drill's own two wires on AOUT1 / AOUT2.
- Swapping a motor's two input wires at its driver is the easy way to reverse any motor.

## Totals (cut from the reel)

| Wire | mm |
|---|---|
| 22 AWG red | 520 |
| 22 AWG black | 360 |
| 24 AWG orange / black | 160 / 160 |
| 26 AWG silicone red / black (foot pair) | 330 / 330 |
| 26 AWG red / black (other motors) | 760 / 855 |
| 26 AWG white / orange | 180 / 95 |
| 28 AWG red/white, black, blue, green, purple, grey, yellow, orange | 240, 225, 120, 120, 200, 200, 105, 105 |
