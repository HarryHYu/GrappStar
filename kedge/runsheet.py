"""KEDGE build-day run sheet PDF (plain black, Arial-metric font).  python3 runsheet.py out27 pack_v27"""
import json, sys, csv
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, Image, KeepTogether
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
OUT, PK = sys.argv[1], sys.argv[2]
pdfmetrics.registerFont(TTFont("Arial", "/System/Library/Fonts/Supplemental/Arial.ttf"))
pdfmetrics.registerFont(TTFont("Arial-Bold", "/System/Library/Fonts/Supplemental/Arial Bold.ttf"))
J = json.load(open(OUT + "/parts.json")); P = J["P"]; A = json.load(open(OUT + "/audit.json"))
PR = json.load(open(PK + "/print_report.json")); WJ = json.load(open(PK + "/wires.json"))
B = ParagraphStyle("b", fontName="Arial", fontSize=9.5, leading=12.5, textColor=colors.black)
S = ParagraphStyle("s", parent=B, fontSize=8, leading=10)
H1 = ParagraphStyle("h1", parent=B, fontName="Arial-Bold", fontSize=15, leading=19, spaceAfter=4)
H2 = ParagraphStyle("h2", parent=B, fontName="Arial-Bold", fontSize=11.5, leading=15, spaceBefore=8, spaceAfter=3)
H3 = ParagraphStyle("h3", parent=B, fontName="Arial-Bold", fontSize=10, leading=13, spaceBefore=5, spaceAfter=1)
def p(t, st=B): return Paragraph(t, st)
def tbl(rows, widths, head=True, fs=8):
    st = ParagraphStyle("t", parent=B, fontSize=fs, leading=fs + 2)
    data = [[Paragraph(str(c), ParagraphStyle("th", parent=st, fontName="Arial-Bold") if (head and i == 0) else st) for c in r] for i, r in enumerate(rows)]
    t = Table(data, colWidths=[w * mm for w in widths], repeatRows=1 if head else 0)
    t.setStyle(TableStyle([("GRID", (0, 0), (-1, -1), 0.4, colors.black), ("VALIGN", (0, 0), (-1, -1), "TOP"),
                           ("LEFTPADDING", (0, 0), (-1, -1), 3), ("RIGHTPADDING", (0, 0), (-1, -1), 3), ("TOPPADDING", (0, 0), (-1, -1), 2), ("BOTTOMPADDING", (0, 0), (-1, -1), 2)]))
    return t
def img(path, w_mm):
    from PIL import Image as PI
    im = PI.open(path); r = im.size[1] / im.size[0]
    return Image(path, width=w_mm * mm, height=w_mm * r * mm)
E = []
V = J.get("version", "v27")
E += [p(f"KEDGE dart {V}: build day run sheet", H1),
      p(f"Dart: Ø42 x {P['L']:.0f} mm, 1.5 mm walls, PLA on the X2D. Kick leg 0 to {P['KICK_DEG']}°, drill pair 0 to {P['DEPLOY_DEG']}°. "
        f"Collision sweep clean (kick in {A["kick_steps"]} steps to {A["kick_range"][1]}°, drills in 1° steps to {A["deploy_range"][1]}°, every sub-part, exact meshes). Printed volume {A['printed_cm3']} cm³ "
        f"(about {A['printed_g_solid_PLA']} g if solid; less at 40% infill). About 12 hours wall clock with the two plates printing in the background."),
      p("Rules for the whole day: the power loop plug stays OUT while you solder, move a wire or plug in USB. Never let the two cells touch metal "
        "together. One step at a time; each step says what you should see before you move on.")]
E += [p("1. What is in the pack", H2), tbl([
    ["File", "What it is"],
    ["print/plate_A_hull.3mf", "Print plate A: hull top, hull bottom, nose, tail cap, both couplers. Already in print orientation."],
    ["print/plate_B_legs_tray.3mf", "Print plate B: tray, kick leg, both drill pods, three augers. Already in print orientation."],
    ["print/fit_coupon.stl", "20 minute test print: checks every fit before the long plates (section 4)."],
    ["print/*.stl, step/*.step", "Every printed part on its own, oriented for printing. STEP is the editable CAD (augers are faceted in STEP)."],
    ["docs/wire_cut_list.csv", "All 53 wires, pad to pad, colour, gauge and cut length (also section 9 of this sheet)."],
    ["firmware/kedge_dart/", "Firmware v3: kedge_dart.ino + kedge_logic.h. Keyboard over USB and WiFi, telemetry /status for the rover, current trip, auto-anchor (foot spins while the kick plants it, drills spin while the pods deploy), auto-on-landing. Compiled on ESP32 core 2.0.17 and 3.3.12."],
    ["firmware/test/", "PC simulation of the control logic (all tests pass)."],
    ["firmware/reel/reel_tension_limit", "Reel sketch for the DevKitC + BTS7960: reel-in speed tapers and stops at a set tether tension (motor current or the load cell)."],
    ["firmware/jetson/kedge_client.py", "Rover / laptop client: status, logging, hold-a-key, auto, terminal driving, FPV overlay."],
    ["docs/KEDGE_*_engineering_report.pdf", "Flight, launch and landing loads, holding force by soil, reel limit, launcher silhouette, design versions."],
    ["launcher/barrel_profile.dxf / barrel_keepout.stl", "What the launcher barrel has to clear, with 1 mm clearance."],
    ["source/", "The parametric generator and the whole check pipeline (run_all.sh). Every number lives in P at the top; KOV overrides make variants."],
], [48, 125])]
E += [p("2. Parts and tools to have on the bench", H2), tbl([
    ["Item", "Qty", "Note"],
    ["XIAO ESP32S3, DRV8833 x3, INA219, BNO085, MP1584 (the 5.04 V one)", "1 each", "Headers clipped off (flush cutters), INA219 screw terminal cut off. MP1584 #2 stays spare."],
    ["OTOTEC 2S BMS stick", "1", "Caliper it when it arrives: the pocket under the cells is 40.8 x 8 x 2.1 plus 0.5 each way."],
    ["18350 cells, charged", "2", "Charge both to 4.2 V in the XTAR first."],
    ["Worm N20 (kick, deploy) + third worm N20 (foot, buy)", "3", "Buy the foot one single-shaft if listed; otherwise cut its lower stub to 2 mm."],
    ["Drill motors (12 x 10 x 24.27, M3 x 56 shaft)", "2", ""],
    ["F693ZZ flanged bearings", "5", "2 per pod + 1 in the kick ear (bag has 10)."],
    ["M3 nuts / M3 nylocs", "2 / 2", "Nut against each pod's inner bearing, nyloc in each auger."],
    ["M3 x 8 screws", "7 + 2", "7 countersunk (flat head, DIN 7991) for the radial hull screws, they sit flush with the skin; 2 pan head for the pod retainers (with washers)."],
    ["M3 x 12 screw, M3 x 10 screw", "1 + 1", "x12 goes through the ear root into tray disc 1; x10 is the kick hub screw."],
    ["M3 washers", "3", "2 pod retainers + 1 between kick hub and ear bearing."],
    ["M3 grub screw", "1", "Foot auger on the foot N20 shaft flat."],
    ["2xAA battery holders (for their contacts)", "2", "You need 2 springs and 2 flat plates. Launch throws the cells backwards, so springs go at the FRONT of the cell bay and flat plates at the BACK."],
    ["470 to 1000 µF capacitor, 10 V or more", "1", "Across the MP1584 5 V output: rides through contact bounce on landing."],
    ["Schottky diode 1N5817 or SS14", "1", "In wire 18 (5 V to the XIAO): stops back-feeding USB."],
    ["Nose ballast 15 g", "", "Steel shot or 3 to 4 M6 nuts set in 5-minute epoxy in the nose cavity. Without it the dart tumbles unless the tether is taut (engineering report, section 3)."],
    ["JST-XH 3-pin pair, JST-XH 2-pin pair + a 2-pin shorting plug", "1 each", "Charge socket and power loop in the tail window."],
    ["Silicone wire 22, 24, 26, 28 AWG", "", "Totals per colour in section 9."],
    ["Zip ties 2.5 mm, medium threadlocker (or CA), double-sided foam tape, hot glue", "", ""],
    ["Already on your bench", "", "Caddx Ant camera, Zeus VTX + u.FL antenna, 3S LiPo + XT60 lead #1, XTAR MC2 (charges the 18350s out of the dart), breadboard and jumpers."],
    ["Tools", "", "Soldering iron, solder, flux, multimeter, wire strippers, flush cutters, small flat screwdriver (MP1584 trimmer), heat shrink + lighter or heat gun, JST-XH crimper or pre-crimped pigtails, 5-minute epoxy, hex keys, tweezers, luggage scale (pull test)."],
], [72, 14, 87])]
E += [PageBreak(), p("3. Timeline", H2), tbl([
    ["Clock", "Step", "Printer"],
    ["0:00", "4. Fit coupon", "coupon (20 min)"],
    ["0:30", "5. Start plate A, flash the firmware", "plate A (about 3 h)"],
    ["1:00", "6. Prepare boards, set both MP1584s, cut and label all wires", "plate A"],
    ["2:00", "7. Solder the bench harness, power-up test, motor directions", "plate A"],
    ["3:30", "Start plate B. Clean plate A parts, test-fit the hull, nose ballast (8a)", "plate B (about 2.5 h)"],
    ["4:00", "8b. Battery contacts and BMS wiring off the dart", "plate B"],
    ["6:00", "8c. Pods, couplers, augers (plate B done)", ""],
    ["7:00", "8d. Populate the tray, battery power-up check", ""],
    ["8:30", "8e. Close the hull, couplers, pods", ""],
    ["9:30", "8f. Kick leg and foot", ""],
    ["10:15", "10. Function test and calibration", ""],
    ["12:00", "Done. Charge the cells (section 10, Charging).", ""],
], [18, 115, 40])]
E += [p("Bambu Studio settings (both plates): PLA, 0.4 nozzle, 0.16 mm layers, 4 wall loops (the 1.5 mm walls come out solid), 40% gyroid infill, top/bottom 5 layers, "
        "supports OFF, brim ON (auto) for the augers and couplers, X-Y hole compensation from the coupon (section 4). Plate times above are estimates; Studio shows the real ones.")]
rows = [["Part", "Print pose", "Size on bed mm", "Volume cm³"]]
for n, r in PR["parts"].items():
    rows.append([n, r["orient"] or "", " x ".join(f"{v:.0f}" for v in r["print_bbox"]), r["volume_cm3"]])
E += [tbl(rows, [32, 88, 30, 20])]
E += [p("4. Fit coupon (0:00, 20 minutes)", H2),
      p("Print fit_coupon.stl. It has, left to right: the worm N20 pocket, the drill motor pocket, the F693ZZ seat with its flange recess, the hull boss bore, "
        "the N20 D-bore, an M3 pilot, an M3 clearance hole, the pod D-bore, the 3.8 shaft clearance, a coupler peg, and a hull ring with its spigot ring."),
      tbl([["Check", "Should feel", "If not"],
           ["Worm N20 into the square pocket", "Slides in by hand, no wobble", "Tight: hole compensation +0.05. Loose: -0.05."],
           ["Drill motor into its pocket", "Slides in by hand", "Same as above"],
           ["F693ZZ into the seat", "Firm thumb press, flange flush in the recess", "Too tight: +0.05. Falls out: a drop of CA on assembly day."],
           ["Coupler peg into the boss bore", "Turns freely, little play", "Tight: sand the peg lightly"],
           ["Coupler peg D-socket onto an N20 shaft", "Pushes on firmly, no rotation slop", "Loose: a drop of CA on assembly"],
           ["Coupler peg into the pod D-bore", "Slides in, no rotation slop", ""],
           ["M3 screw into the 2.5 pilot", "Cuts its own thread, holds", ""],
           ["Spigot ring into the hull ring", "Push fit, no rattle", "Loose: one layer of tape on the spigots"]], [52, 58, 63])]
E += [p("Directions used below: left = the kick-leg side, seen from the tail with the saddle on top (y plus in the CAD). The left pod is the brighter red one in the viewer.", S)]
E += [p("5. Start plate A and flash the firmware (0:30)", H2),
      p("1. Send plate_A_hull.3mf to the X2D."),
      p("2. In the Arduino IDE: board <b>XIAO_ESP32S3</b>, Tools > USB CDC On Boot: <b>Enabled</b> (the sketch will not compile otherwise). Library Manager: install <b>Adafruit INA219</b> and <b>Adafruit BNO08x</b>."),
      p("3. Nothing else connected to the XIAO. Plug it in by USB-C, open firmware/kedge_dart/kedge_dart.ino, Upload. Re-select the board if the IDE switches it to the wrong one."),
      p("4. Serial Monitor at 115200, 'No line ending'. You should see the KEDGE ready help line (INA219 and BNO085 'not found' is normal now). If the monitor stays blank: press the XIAO reset button once, check the port and USB CDC On Boot."),
      p("5. On your phone or laptop, join WiFi <b>KEDGE</b> (password <b>grappstar</b>) and open <b>192.168.4.1</b>: you get the control page. From now on you can also upload over WiFi: Arduino IDE > Port > the network port called kedge. Unplug USB.")]
E += [p("6. Prepare boards and wires (1:00)", H2),
      p("1. Clip every header pin flush on the XIAO and the three DRV8833 boards (flush cutters, close to the board). Cut the INA219 screw terminal off. Tin each pad you will use."),
      p("2. MP1584s, with the 3S LiPo on XT60 lead #1 and the multimeter on the output, turning the small brass screw with a small screwdriver: the dart one to <b>5.04 V</b>; the spare one to <b>8.0 V</b> (this is your bench motor supply in step 7). Disconnect."),
      p("3. Cut all wires to the lengths in section 9. Strip 3 mm each end, tin, and put a tape flag with the wire number on each. Always slide heat shrink onto a wire before you solder its second end."),
      p("4. <b>DRV8833 check before any 3V3 wire</b>: the driver chip has no logic supply pin. The pin called VIN (or VCC on some red boards) is the MOTOR supply. The 3V3 wires (25 to 27) go only to the SLP / nSLEEP / EEP pad, which wakes the driver. "
        "Meter on beep: touch the pad you plan to use and the VIN/VM pin. If it beeps, it is the motor supply: do not put 3V3 on it, or the battery voltage reaches the XIAO."),
      p("5. Diode: wire 18 has a Schottky diode (1N5817 or SS14) in it, stripe toward the XIAO, in heat shrink. It stops the 5 V buck back-feeding your Mac when USB is plugged in. Still never plug USB in while the dart is powered.")]
E += [PageBreak(), p("7. Solder the bench harness and test it (2:00)", H2),
      p("Lay the boards on the bench in the order they sit in the dart (picture in section 11): sled boards (MP1584, DRV#3), XIAO + DRV#1, DRV#2 + INA219, BNO085. Solder in this order and check after each group:"),
      p("1. <b>Power</b>: wires 10 to 13 and 15 to 24, and wire 14 at its MP1584 end only (its BMS end comes in 8b). Fit the capacitor across MP1584 OUT+ / OUT-: <b>long leg to OUT+, the striped leg to OUT-</b>. "
        "Snap the u.FL antenna onto the VTX now: never power a VTX without its antenna. Also solder wire 9 at its INA219 VIN+ end only (its socket end is crimped in 8b). Meter on beep: red rail to black rail must NOT beep."),
      p("2. <b>3V3 and ground to the logic</b>: wires 25 to 31."),
      p("3. <b>I2C</b>: wires 32 to 35. <b>Motor control</b>: wires 36 to 43. Bridge AIN1 to BIN1 and AIN2 to BIN2 on DRV#2 with a short wire offcut."),
      p("4. <b>Motor leads</b>: solder wires 44 to 53 to the DRV outputs only; leave the far ends free."),
      p("5. <b>Power-up test</b> with the bench supply (the 8.0 V MP1584 on the LiPo): its OUT+ to the INA219 VIN+ pad, OUT- to the MP1584 IN- pad (main ground). "
        "You should see about 4.7 V on the XIAO 5V pad (5.04 minus the diode), the VTX LED on, and on the WiFi page: <b>i</b> reads about 8 V; <b>z</b> finds 0x40 and 0x4a. Disconnect the bench supply."),
      p("6. <b>Motor directions</b>, bench supply on again. Twist each motor lead temporarily onto its wire (no solder yet: the kick, drill and foot leads are threaded through holes later). Hold each key half a second and check:"),
      tbl([["Key", "Motor", "Must turn so that", "If not"],
           ["1", "deploy N20", "the output shaft turns the way that swings the pods DOWN (mark the shaft with a pen and compare with the viewer)", "swap the pair on wire 44 / 45"],
           ["2", "kick N20", "the hub would swing the leg UP over the nose", "swap 46 / 47"],
           ["3", "drills", "each drill output shaft turns: left motor clockwise seen from behind the motor, right motor anticlockwise (the augers are mirror images, so both then screw in)", "one wrong: swap that motor's pair; both wrong: swap 40 / 41"],
           ["4", "foot N20", "the foot N20 output shaft turns clockwise, seen from the shaft end (that screws the right-hand foot auger in)", "swap 52 / 53"]], [12, 22, 92, 48], fs=7.5),
      p("Note the free current on <b>i</b> for each (October bench: worms 20 to 30 mA, drill pair 60 to 70 mA). Mark each motor's lead that goes to the lower wire number. Solder the deploy N20 now (it never goes through a hole). Disconnect the bench supply.")]
E += [p("8. Mechanical assembly", H2),
      p("8a. Nose ballast (3:30, plate A done)", H3),
      p("Stand the nose on its tip. Drop 15 g of steel shot or 3 to 4 M6 nuts into the cavity around the camera tube (not into the camera pocket) and cover them with 5-minute epoxy. Leave it to set while you carry on."),
      p("8b. Battery contacts and BMS, off the dart (4:00)", H3),
      p("1. Join the two springs with wire 2 (the series link, 22 AWG) and solder wire 3 (BMS BM) to that link. Solder wire 1 to one flat plate and wire 4 to the other."),
      p("2. On the bench, solder wires 1, 3 and 4 to the BMS B-, BM and B+ pads, wire 14 to P-, wire 8 to P+, and wires 5 to 7 to B-, BM, B+ for the charge socket. Crimp or solder the JST-XH sockets on wires 5 to 7 (3-pin) and 8 / 9 (2-pin power loop)."),
      p("8c. Drill pods, couplers and augers (6:00, plate B done)", H3),
      p("1. Press one F693ZZ into the inner seat from the motor pocket side, flange toward the motor. Press the other into the nose end from outside, flange on the nose face."),
      p("2. Thread an M3 nut onto the drill motor shaft, about 3 mm from the motor face. Feed the shaft through both bearings, drop the motor into its pocket. "
        "Threadlocker on the shaft, then hold the nut with tweezers and turn the motor until the nut seats against the inner bearing. Zip-tie the motor in its groove."),
      p("3. Push an M3 nyloc into the side slot of the auger (nylon ring toward the point). Augers are handed: the LEFT pod takes drill_leg_L_auger (right-hand flight), the RIGHT pod takes drill_leg_R_auger (left-hand flight). "
        "Threadlocker on the shaft end, screw the auger on until its base boss touches the outer bearing."),
      p("4. Check: each auger spins freely with the motor, no rub on the pod nose."),
      p("8d. Populate the tray (7:00)", H3),
      p("1. Glue the joined springs into the long recess on the FRONT cell-bay disc (nose side), one spring in front of each cell position. Glue the wire 1 plate into the BACK disc recess on the right (cell 1 side) and the wire 4 plate on the left. Lay the BMS in the channel under the cell bay."),
      p("2. Deploy N20 into the sled cradle (worm box toward the tail, output shaft across), zip tie through the groove under the cradle."),
      p("3. Stick the boards on with foam tape: DRV#3 on the keel and MP1584 on the shelf in the sled; BNO085 on deck A; XIAO (USB toward the nose) and DRV#1 on deck B; DRV#2 and INA219 on deck C; VTX on the deck over the cells. "
        "Route bundles through the notches in the discs. Antenna lead to the tail."),
      p("4. Push both sockets into the tail cap window, antenna through the top slot (wire 9 and wire 14 are already soldered from 7.1 and 8b). Tape over the charge socket now: its pins are live as soon as the cells go in."),
      p("5. Cells in: cell 1 (right) button toward the nose, cell 2 (left) button toward the tail. Each pushes in sideways, squeezing its spring, and sits hard against its back plate."),
      p("5a. Loop still out. Meter on the charge socket pins (lift the tape corner): pin 1 to 2 reads 3.6 to 4.2 V, pin 2 to 3 reads 3.6 to 4.2 V, pin 1 to 3 reads 7.2 to 8.4 V. Any negative reading: pull the cells out and check their direction."),
      p("6. <b>Battery power-up check</b>: check the u.FL antenna is still clicked onto the VTX, then plug the loop in. WiFi page: <b>i</b> reads 7.4 to 8.4 V; <b>z</b> finds 0x40 and 0x4a; the deploy N20 runs on <b>1</b>. Pull the loop plug."),
      p("8e. Close the hull (8:30)", H3),
      p("1. Kick N20 into the saddle on the hull top, worm box toward the nose, output shaft through both side holes. Zip tie: down one slot beside the saddle, under the skin, up the other, over the motor. Its leads go down through the hole in the saddle floor."),
      p("2. Camera into the nose pocket from the back (lens into the hole), cable out the back. Tray into the bottom half, and seat the nose spigot in the bottom half at the same time: the groove on the spigot sits over the half-rib inside the hull."),
      p("3. Thread the leads before the top half goes on: kick leads (wires 46, 47) up through the saddle floor hole; foot leads (52, 53) out through the skin hole beside the saddle; drill leads (48 to 51) out through the two flank holes. Lower the top half on."),
      p("4. Radial screws into disc 1: M3 x 8 at left z -3, right z +3 and right z -3, and the M3 x 12 through the ear root at left z +3; four M3 x 8 into the tail spigot (z ±3 both sides). The top half's rib closes the nose groove, so the nose is locked."),
      p("5. Solder the kick N20 and the two drill motors to their leads (as marked in 7.6), heat shrink (slide it on before soldering; iron on each joint 3 s at most; shrink with a heat gun on low, no lighter near the PLA)."),
      p("6. Couplers: slide coupler_L in through the left boss, coupler_R through the right, turning gently until each D-socket drops onto the deploy N20 stub. "
        "Pods onto the coupler ends (D-bore), then M3 x 8 + washer into each coupler end. Fold the pods along the flanks."),
      p("8f. Kick leg and foot (9:30)", H3),
      p("1. Press an F693ZZ into the ear from outside, flange in the recess."),
      p("2. Foot N20 into the foot housing (opening on the inboard side), output shaft up through the top hole, zip tie. Foot auger onto the shaft, grub screw onto the flat."),
      p("3. Feed the foot leads from the foot pocket through the pole channel and out the small hole near the hub. Solder them to wires 52 and 53 (as marked in 7.6): heat shrink on first, iron 3 s at most per joint, heat gun on low, no lighter near the PLA. Leave the loop round the hub."),
      p("4. Push the hub onto the kick N20 stub (D flat), put the washer between hub and ear, M3 x 10 through the ear bearing into the hub. Plug the loop in and send @ until the leg rests folded on the hull.")]
E += [p("10. Function test and calibration (10:15)", H2),
      p("Use a tray of damp soil or sand. Laptop on the KEDGE WiFi page; hold a key to run, let go to stop. Space bar stops everything."),
      tbl([["Key", "Should happen", "If it goes the wrong way"],
           ["First, in the air", "Hold each key for half a second: directions were fixed in 7.6", "If one is still wrong, swap it at the DRV end (open the hull)"],
           ["2 (hold)", "Kick leg swings up over the nose and down; the foot auger tip meets the soil near 180°", "Swap wires 46 and 47"],
           ["2 and 4 (hold both)", "Foot auger screws IN while the kick keeps pushing, until the foot housing sits on the soil", "Foot backs out: swap wires 52 and 53"],
           ["1 and 3 (hold both)", "Both pods swing down and back while BOTH augers spin and cut in; the dart does not twist", "Pods go up: swap 44 and 45. One auger backs out: swap that motor's pair (48/49 or 50/51). Both: swap 40 and 41 at the XIAO"],
           ["i during each", "Note the free current, the current when the foot touches, and the stall current", "Put them in kedge_logic.h (kickContactMa, kickStallMa, deployStallMa) and upload over WiFi"],
           ["b", "Roll and pitch of the planted dart", ""],
           ["a", "Full auto sequence, same order as above, by itself", "Space aborts"],
           ["A then throw", "Arms auto-anchor on landing (the dart anchors 1.5 s after it comes to rest)", "Press A again to disarm"]], [28, 80, 65]),
      p("Charging: either take the cells out and use the XTAR MC2, or plug a 2S balance charger (JST-XH 3-pin) into the charge socket in the tail. Loop plug OUT while charging."),
      p("Recovery: # (drills out), ! (pods up), $ (foot out), @ (kick back). The worms hold every position with the power off. "
        "Note: STOP (space, /stop, or a client hold ending) also aborts a running auto sequence; that is deliberate."),
      p("Pin change from the October bench: the drill pair is now on D8/D9 and the foot on D6/D7. D6/D7 are the XIAO's serial pins and flicker at every reset; the slow foot worm does not care, the drills would twitch.", S)]
E += [PageBreak(), p("9. Wire cut list (pad to pad)", H2)]
rows = [["#", "From", "To", "Colour", "Gauge", "Cut mm", "Note"]] + [[str(c) for c in r] for r in WJ["wires"]]
E += [tbl(rows, [7, 44, 44, 13, 13, 11, 45], fs=7)]
E += [Spacer(1, 4), p("Totals by gauge and colour (mm): " + ", ".join(f"{k} {v}" for k, v in WJ["totals"].items()), S)]
E += [PageBreak(), p("11. Pictures", H2), p("Wiring (numbers match the cut list):", S), img(PK + "/wiring.png", 180),
      p("Exploded view and populated tray:", S), img(OUT + "/img_exploded.png", 88), img(OUT + "/img_tray_populated.png", 88),
      PageBreak(), p("Audit sheet: 24 views of the exact print geometry (8 folded, 8 deployed, 6 mid-motion, top, under).", S), img(OUT + "/audit_sheet.png", 180),
      p("Section cuts:", S), img(OUT + "/sections.png", 180)]
E += [p("12. Design notes", H2)] + [p("• " + t, S) for t in [
    "Pod thrust: the M3 nut against the inner bearing takes the auger's pull into the ground; the auger base against the outer bearing takes push. The gearbox only carries torque.",
    "The drill pair counter-rotates so the dart does not spin; the two augers are mirror images so both screw in.",
    "The deploy N20's own shaft stubs drive the pods through two printed Ø7 D couplers that run in the hull bosses.",
    "Tether load path: tail end wall, tail cap, 4 radial M3 into the hull. Tie the line through the two end holes round the web.",
    f"Ground geometry: kick stops at {P['KICK_DEG']}° with the foot housing on the ground and the foot auger {A['auger_depth_mm']['foot_auger']} mm in; drills stop at {P['DEPLOY_DEG']}° with the pods {list(A['pod_clear_mm'].values())[0]} mm clear and the augers {A['auger_depth_mm']['drill_leg_L_auger']} mm in.",
    "Power: 2S straight to the DRV8833 motor rail through the INA219; one MP1584 at 5.04 V for the XIAO, VTX and camera.",
    "Below the centreline the hull is a plain Ø42 cylinder for the launcher cradle; the folded pods sit just above it."]]
doc = SimpleDocTemplate(PK + f"/KEDGE_{V}_run_sheet.pdf", pagesize=A4, leftMargin=16 * mm, rightMargin=16 * mm, topMargin=14 * mm, bottomMargin=14 * mm,
                        title=f"KEDGE {V} run sheet", author="")
doc.build(E)
print("pdf ok")
