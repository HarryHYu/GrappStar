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
import os
_F = [("/System/Library/Fonts/Supplemental/Arial.ttf", "/System/Library/Fonts/Supplemental/Arial Bold.ttf"),
      ("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf", "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf")]
_reg, _bold = next(f for f in _F if os.path.exists(f[0]))       # Arial on the Mac, Liberation Sans (Arial metrics) elsewhere
pdfmetrics.registerFont(TTFont("Arial", _reg)); pdfmetrics.registerFont(TTFont("Arial-Bold", _bold))
J = json.load(open(OUT + "/parts.json")); P = J["P"]; A = json.load(open(OUT + "/audit.json"))
PR = json.load(open(PK + "/print_report.json")); WJ = json.load(open(PK + "/wires.json"))
NW = len(WJ["wires"]); TOT = {}
for w_ in WJ["wires"]: TOT[f"{w_['g']} {w_['col']}"] = TOT.get(f"{w_['g']} {w_['col']}", 0) + w_["len"]
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
        f"(about {A['printed_g_solid_PLA']} g if solid; less at 40% infill). Hard stops checked: pods stop on the hull horns about 1° past {P['DEPLOY_DEG']}° and on the fold lugs, the kick on its rest {P.get('KICK_REST_DEG', 0)}° past the plant. "
        f"Launcher fit checked against the real launcher CAD. About 12 hours wall clock with the two plates printing in the background."),
      p("Rules for the whole day: the power loop plug stays OUT while you solder, move a wire or plug in USB. Never let the two cells touch metal "
        "together. One step at a time; each step says what you should see before you move on.")]
E += [p("1. What is in the pack", H2), tbl([
    ["File", "What it is"],
    ["print/plate_A_hull.3mf", "Print plate A: hull top, hull bottom, nose, tail cap, both pod caps (PLA). Plate C in PPA-CF: both couplers, the tail boom and its anchor disc. Already in print orientation."],
    ["print/plate_B_legs_tray.3mf", "Print plate B: tray, kick leg, both drill pods, three augers. Already in print orientation."],
    ["print/fit_coupon.stl", "20 minute test print: checks every fit before the long plates (section 4)."],
    ["print/*.stl, step/*.step", "Every printed part on its own, oriented for printing. STEP is the editable CAD (augers are faceted in STEP)."],
    ["docs/wire_cut_list.csv", f"All {NW} wires (firmware v4 harness, DRV8833 #3 off the dart), pad to pad, colour, gauge, cut length on the v31 routes (also section 9)."],
    ["firmware/dart/ (repository)", "Firmware v4 and its WIRING.md (the soldering order) and BENCH_TEST.md. Follow WIRING.md Path B for the harness; this sheet covers the printing and the mechanical build."],
    ["firmware/test/", "PC simulation of the control logic (all tests pass)."],
    ["docs/KEDGE_*_engineering_report.pdf", "Flight, launch and landing loads, holding force by soil, reel limit, launcher silhouette, design versions."],
    ["launcher/barrel_profile.dxf / barrel_keepout.stl", "What the launcher barrel has to clear, with 1 mm clearance."],
    ["source/", "The parametric generator and the whole check pipeline (run_all.sh). Every number lives in P at the top; KOV overrides make variants."],
], [48, 125])]
E += [p("2. Parts and tools to have on the bench", H2), tbl([
    ["Item", "Qty", "Note"],
    ["XIAO ESP32S3 (+ its FPC antenna), DRV8833 x2, INA219, BNO085, MP1584 (the 5.04 V one)", "1 each", "Headers clipped off (flush cutters), INA219 screw terminal cut off. DRV8833 #3 goes to the base station. MP1584 #2 stays spare."],
    ["OTOTEC 2S BMS stick", "1", "Caliper it when it arrives: the pocket under the cells is 40.8 x 8 x 2.1 plus 0.5 each way."],
    ["18350 cells, charged", "2", "Charge both to 4.2 V in the XTAR first."],
    ["Worm N20 (kick, deploy) + third worm N20 (foot, buy)", "3", "Buy the foot one single-shaft if listed; otherwise cut its lower stub to 2 mm (it would hit the hull top when folded)."],
    ["Drill motors (12 x 10 x 24.27, M3 x 56 shaft)", "2", "Each sits INSIDE its auger sleeve, held by its can in the arm's bushing. Bend both solder tabs back along the can before soldering: 1.2 mm behind the end cap."],
    ["F693ZZ flanged bearing", "1", "In the kick ear (the drills need none now)."],
    ["M3 nuts / M3 nylocs", "2 / 2", "One jammed pair per drill sleeve: nut in the hub's side slot, nyloc on it through the tip."],
    ["M3 x 8 countersunk screw (one more of the radial ones)", "1", "The tether pin in the tail boom eye (head flush, threads into the lower tine; threadlocker)."],
    ["M3 x 8 screws", "8 + 2", "8 countersunk (flat head, DIN 7991) radial screws flush with the skin: 4 into tray disc 1, 4 through the tail spigot into the boom anchor; 2 pan head through the printed pod caps into the couplers."],
    ["M3 x 8 screw (kick hub)", "1", "Through the ear bearing into the hub (its head clamps the bearing's inner race onto the hub boss: no washer)."],
    ["M3 grub screw", "1", "Foot auger on the foot N20 shaft flat."],
    ["2xAA battery holders (for their contacts)", "2", "You need 2 springs and 2 flat plates. Launch throws the cells backwards, so springs go at the FRONT of the cell bay and flat plates at the BACK."],
    ["470 to 1000 µF capacitor, 10 V or more", "1", "Across the MP1584 5 V output: rides through contact bounce on landing."],
    ["Schottky diode 1N5817 or SS14", "1", "In wire 18 (5 V to the XIAO): stops back-feeding USB."],
    ["JST-XH 3-pin pair, JST-XH 2-pin pair + a 2-pin shorting plug", "1 each", "Charge socket in the bottom seat of the tail cap, power loop socket in the TOP seat (the shorting plug sits recessed under the skin, a short wire loop on it is the pull tab)."],
    ["Silicone wire 22, 24, 26, 28 AWG", "", "Totals per colour in section 9."],
    ["Zip ties 2.5 mm (4: kick N20, deploy N20, two drill motors, plus the foot strap), medium threadlocker (or CA), double-sided foam tape, hot glue", "", "No nose ballast in v31: at the real 7 m/s launch it adds nothing (dart-physics)."],
    ["Already on your bench", "", "Caddx Ant camera, Zeus VTX + u.FL antenna, 3S LiPo + XT60 lead #1, XTAR MC2 (charges the 18350s out of the dart), breadboard and jumpers."],
    ["Tools", "", "Soldering iron, solder, flux, multimeter, wire strippers, flush cutters, small flat screwdriver (MP1584 trimmer), heat shrink + lighter or heat gun, JST-XH crimper or pre-crimped pigtails, 5-minute epoxy, hex keys, tweezers, luggage scale (pull test)."],
], [72, 14, 87])]
E += [PageBreak(), p("3. Timeline", H2), tbl([
    ["Clock", "Step", "Printer"],
    ["0:00", "4. Fit coupon", "coupon (20 min)"],
    ["0:30", "5. Start plate A, flash the firmware", "plate A (about 3 h)"],
    ["1:00", "6. Prepare boards, set both MP1584s, cut and label all wires", "plate A"],
    ["2:00", "7. Solder the bench harness, power-up test, motor directions", "plate A"],
    ["3:30", "Start plate B (and the two couplers in PPA-CF). Clean plate A parts, test-fit the hull", "plate B (about 2.5 h)"],
    ["4:00", "8b. Battery contacts and BMS wiring off the dart", "plate B"],
    ["6:00", "8c. Drill arms, motors, sleeves (plate B done)", ""],
    ["7:00", "8d. Populate the tray, battery power-up check", ""],
    ["8:30", "8e. Close the hull, couplers, pods", ""],
    ["9:30", "8f. Kick leg and foot", ""],
    ["10:15", "10. Function test and calibration", ""],
    ["12:00", "Done. Charge the cells (section 10, Charging).", ""],
], [18, 115, 40])]
E += [p("Bambu Studio settings (both plates): PLA, 0.4 nozzle, 0.16 mm layers, 4 wall loops (the 1.5 mm walls come out solid), 40% gyroid infill, top/bottom 5 layers, "
        "tree supports ONLY where the table says (inside the two hull halves), brim ON (auto) for the augers, X-Y hole compensation from the coupon (section 4). Couplers in PPA-CF (hardened nozzle, dried). Plate times above are estimates; Studio shows the real ones.")]
rows = [["Part", "Print pose", "Supports", "Size on bed mm", "cm³"]]
for n, r in PR["parts"].items():
    rows.append([n, r["orient"] or "", r.get("supports", ""), " x ".join(f"{v:.0f}" for v in r["print_bbox"]), r["volume_cm3"]])
E += [tbl(rows, [26, 50, 62, 22, 12], fs=7)]
E += [p("4. Fit coupon (0:00, 20 minutes)", H2),
      p("Print fit_coupon.stl. It has, left to right: the worm N20 pocket, the drill motor pocket, the F693ZZ seat with its flange recess, the hull boss bore, "
        "the N20 D-bore, an M3 pilot, an M3 clearance hole, the pod D-bore, the 3.8 shaft clearance, a coupler peg, and a hull ring with its spigot ring. "
        "Second row (v31): the JST-XH 2-pin and 3-pin header seats of the tail cap, the USB-C window of the hull (your cable must plug through it), the pod head D-bore with its cap counterbore, and a pod cap."),
      tbl([["Check", "Should feel", "If not"],
           ["Worm N20 into the square pocket", "Slides in by hand, no wobble", "Tight: hole compensation +0.05. Loose: -0.05."],
           ["Drill motor into its pocket", "Slides in by hand", "Same as above"],
           ["F693ZZ into the seat", "Firm thumb press, flange flush in the recess", "Too tight: +0.05. Falls out: a drop of CA on assembly day."],
           ["Coupler peg into the boss bore", "Turns freely, little play", "Tight: sand the peg lightly"],
           ["Coupler peg D-socket onto an N20 shaft", "Pushes on firmly, no rotation slop", "Loose: a drop of CA on assembly"],
           ["Coupler peg into the pod D-bore", "Slides in, no rotation slop", ""],
           ["M3 screw into the 2.5 pilot", "Cuts its own thread, holds", ""],
           ["Spigot ring into the hull ring", "Push fit, no rattle", "Loose: one layer of tape on the spigots"],
           ["JST headers into their seats", "Push in, sit on the floor lip, no wobble", "Loose: a drop of CA on assembly day"],
           ["USB-C cable through the window", "The overmold passes the window", "Too tight: file the window corners on the hull"],
           ["Pod cap in the counterbore", "Drops in, turns freely", ""]], [52, 58, 63])]
E += [p("Directions used below: left = the kick-leg side, seen from the tail with the saddle on top (y plus in the CAD). The left pod is the brighter red one in the viewer.", S)]
E += [p("5. Start plate A and flash the firmware (0:30)", H2),
      p("1. Send plate_A_hull.3mf to the X2D."),
      p("2. In the Arduino IDE: board <b>XIAO_ESP32S3</b>, Tools > USB CDC On Boot: <b>Enabled</b> (the sketch will not compile otherwise). Library Manager: install <b>Adafruit INA219</b> and <b>Adafruit BNO08x</b>."),
      p("3. Nothing else connected to the XIAO. Plug it in by USB-C, open firmware/kedge_dart/kedge_dart.ino, Upload. Re-select the board if the IDE switches it to the wrong one."),
      p("4. Serial Monitor at 115200, 'No line ending'. You should see the KEDGE ready help line (INA219 and BNO085 'not found' is normal now). If the monitor stays blank: press the XIAO reset button once, check the port and USB CDC On Boot."),
      p("5. On your phone or laptop, join WiFi <b>KEDGE</b> (password <b>grappstar</b>) and open <b>192.168.4.1</b>: you get the control page. From now on you can also upload over WiFi: Arduino IDE > Port > the network port called kedge. Unplug USB.")]
E += [p("6. Prepare boards and wires (1:00)", H2),
      p("1. Clip every header pin flush on the XIAO and both DRV8833 boards (flush cutters, close to the board). Cut the INA219 screw terminal off. Tin each pad you will use."),
      p("2. Set the dart MP1584 to <b>5.04 V</b> before it is wired in (WIRING.md step B2). Set the spare one to <b>8.0 V</b>: your bench motor supply."),
      p("3. Cut all wires to the lengths in section 9 (route column: KEEL = the channel under the cells, TOP = over the VTX). Strip 3 mm, tin, tape flag with the wire number. Heat shrink on before the second end is soldered.")]
E += [p("7. Solder the harness and test it (2:00)", H2),
      p("Follow <b>firmware/dart/WIRING.md, Path B</b>, steps B1 to B10, and its power-on check, battery unplugged at every step. Then BENCH_TEST.md steps 0 to 6 on the loose motors (directions and free currents). "
        "v31 positions: XIAO on deck B turned with its USB-C to the RIGHT flank (the hull window), DRV#1 beside it; DRV#2 and INA219 on deck C; BNO085 in the sled under the MP1584 shelf, turned across the dart; "
        "power loop socket in the TOP seat of the tail cap, charge socket in the bottom seat. Drill motors: bend both tabs up before soldering.")]
E += [p("8. Mechanical assembly", H2),

      p("8b. Battery contacts and BMS, off the dart (4:00)", H3),
      p("1. Join the two springs with wire 2 (the series link, 22 AWG) and solder wire 3 (BMS BM) to that link. Solder wire 1 to one flat plate and wire 4 to the other."),
      p("2. On the bench, solder wires 1, 3 and 4 to the BMS B-, BM and B+ pads, wire 13 to P-, wire 8 to P+, and wires 5 to 7 to B-, BM, B+ for the charge socket. Solder the JST-XH 3-pin header on wires 5 to 7 and the 2-pin header on wires 8 / 9."),
      p("8c. Drill arms, motors and sleeve augers (6:00, plate B done)", H3),
      p("1. Solder 200 mm of lead to each drill motor first (tabs bent back), and feed the leads through the bushing's back wall and out of the arm neck."),
      p("2. Push the motor can into the arm's bushing, shaft first out, until the end cap is 1.2 mm off the back wall: the can's flats key it. A drop of CA on one flat holds it."),
      p("3. Drop an M3 nut into the side slot near the tip of the sleeve. Sleeves are handed: the LEFT arm takes drill_leg_L_auger (right-hand flight), the RIGHT arm takes drill_leg_R_auger (left-hand flight)."),
      p("4. Slide the sleeve over the motor from the front and turn it: the shaft finds the nut and the sleeve screws on until its base is 0.2 mm off the arm's shoulder (it must still turn by hand). "
        "Threadlocker on the shaft end, then an M3 nyloc down the tip bore with a 5.5 nut driver, tight against the hub wall while you hold the sleeve: the jammed pair keys the sleeve to the shaft in both directions."),
      p("5. Check: the sleeve turns with the motor without rubbing the bushing or the motor (no tick per turn)."),
      p("8d. Populate the tray (7:00)", H3),
      p("1. Glue the joined springs into the long recess on the FRONT cell-bay disc (nose side), one spring in front of each cell position. Glue the wire 1 plate into the BACK disc recess on the right (cell 1 side) and the wire 4 plate on the left. Lay the BMS in the keel channel under the cell bay: the KEEL wires run beside it through the same channel."),
      p("2. Deploy N20 into the sled cradle (worm box toward the tail, output shaft across), zip tie through the groove under the cradle."),
      p("3. Stick the boards on with foam tape: BNO085 on its two rails in the sled (across the dart), MP1584 on the shelf above it; XIAO on deck B with its USB-C at the right-hand edge (it must line up with the hull window) and DRV#1 beside it; DRV#2 and INA219 on deck C; VTX on the deck over the cells; the XIAO FPC antenna on the inside of the hull top over the board bay. "
        "KEEL wires: down the slot behind the XIAO, along the keel channel under the cells, out behind the deploy motor. TOP wires: through the top notches over the VTX. VTX antenna along the top through the disc 4 notch."),
      p("4. Push the 2-pin header into the TOP seat of the tail cap from the inside (pins down through the floor opening) and the 3-pin header into the bottom seat; a drop of CA each. Tape over the charge socket now: its pins are live as soon as the cells go in."),
      p("5. Cells in: cell 1 (right) button toward the nose, cell 2 (left) button toward the tail. Each pushes in sideways, squeezing its spring, and sits hard against its back plate."),
      p("5a. Loop still out. Meter on the charge socket pins (lift the tape corner): pin 1 to 2 reads 3.6 to 4.2 V, pin 2 to 3 reads 3.6 to 4.2 V, pin 1 to 3 reads 7.2 to 8.4 V. Any negative reading: pull the cells out and check their direction."),
      p("6. <b>Battery power-up check</b>: check the u.FL antenna is still clicked onto the VTX, then plug the loop in. WiFi page: <b>i</b> reads 7.4 to 8.4 V; <b>z</b> finds 0x40 and 0x4a; the deploy N20 runs on <b>1</b>. Pull the loop plug."),
      p("8e. Close the hull (8:30)", H3),
      p("1. Kick leg first goes on the kick N20, off the dart: push the hub onto the LEFT stub (D flat); the motor sits beside the leg on the right. Lower motor and leg together: the motor into the saddle on the right of the centreline, shaft into its two vertical slots, worm box toward the nose; the leg lies on the centreline. Zip tie over the motor through the two slots. Leads down through the hole in the saddle floor right behind the end cap."),
      p("2. Camera into the nose pocket from the back (lens into the hole), cable out the back. Tray into the bottom half, and seat the nose spigot in the bottom half at the same time: the groove on the spigot sits over the half-rib inside the hull."),
      p("3. Thread the leads before the top half goes on: kick leads (wires 43, 44) up through the saddle floor hole; foot leads (49, 50) out through the skin hole under the folded pole; drill leads (45 to 48) out through the two holes just behind the pivot blocks. Lower the top half on."),
      p("4. Tail boom: push its square shank through the anchor disc (head into the recess) and through the tail cap's square hole from inside; fit the tail cap with the disc in its spigot. Radial screws: four M3 x 8 into disc 1 (z ±3 both sides) and four M3 x 8 through the tail spigot into the anchor disc: those four carry the tow. The top half's rib closes the nose groove, so the nose is locked. Tether: a loop round the M3 x 8 pin in the boom eye."),
      p("5. Solder the kick N20 and the two drill motors to their leads (as marked in 7.6), heat shrink (slide it on before soldering; iron on each joint 3 s at most; shrink with a heat gun on low, no lighter near the PLA)."),
      p("6. Couplers: slide coupler_L in through the left boss, coupler_R through the right, turning gently until each D-socket drops onto the deploy N20 stub. "
        "Drill arms onto the coupler ends (D-bore), printed pod cap into each head's outer counterbore, M3 x 8 through the cap into the coupler. The cap sits on the coupler end: the arm must still swing freely (0.7 mm float). Fold the sleeves along the flanks: the heads rest just clear of the fold stops on the pivot blocks; swung down they land on the deploy stops in front of the heads."),
      p("8f. Kick leg and foot (9:30)", H3),
      p("1. Press the F693ZZ into the ear (left of the hub) from outside, flange down in the counterbore: its inner face stands 0.4 proud on the hub side and touches the hub's small boss."),
      p("2. Foot N20 into the foot housing from the open inboard side, output shaft sliding along the slot into the top hole, strap. Foot auger onto the shaft (it sits 0.3 above the housing), grub screw onto the flat."),
      p("3. Feed the foot leads from the foot pocket through the pole channel and out the small hole near the hub. Solder them to wires 52 and 53 (as marked in 7.6): heat shrink on first, iron 3 s at most per joint, heat gun on low, no lighter near the PLA. Leave the loop round the hub."),
      p("4. M3 x 8 through the ear bearing into the hub (no washer: the head clamps the inner race onto the hub boss). Plug the loop in (top of the tail cap) and send @ until the leg rests folded on the hull.")]
E += [p("10. Function test and calibration (10:15)", H2),
      p("Use a tray of damp soil or sand. Laptop on the KEDGE WiFi page; hold a key to run, let go to stop. Space bar stops everything."),
      tbl([["Key", "Should happen", "If it goes the wrong way"],
           ["First, in the air", "Hold each key for half a second: directions were fixed in 7.6", "If one is still wrong, swap it at the DRV end (open the hull)"],
           ["2 (hold)", "Kick leg swings up over the nose and down; the foot auger tip meets the soil near 180°", "Swap wires 43 and 44"],
           ["2 and 4 (hold both)", "Foot auger screws IN while the kick keeps pushing, until the foot housing sits on the soil (on a hard floor the pole lands on its rest on the ear root)", "Foot backs out: swap wires 49 and 50"],
           ["1 and 3 (hold both)", "Both pods swing down and back while BOTH augers spin and mill in; the dart does not twist; in air the pod heads stop on the hull horns", "Pods go up: swap 41 and 42. One auger backs out: swap that motor's own pair on AOUT1/AOUT2. Both: swap 37 and 38 at the XIAO"],
           ["i during each", "Note the free current, the current when the foot touches, and the stall current", "Put them in kedge_logic.h (kickContactMa, kickStallMa, deployStallMa) and upload over WiFi"],
           ["b", "Roll and pitch of the planted dart", ""],
           ["a", "Full auto sequence, same order as above, by itself", "Space aborts"],
           ["A then throw", "Arms auto-anchor on landing (the dart anchors 1.5 s after it comes to rest)", "Press A again to disarm"]], [28, 80, 65]),
      p("Charging: either take the cells out and use the XTAR MC2, or plug a 2S balance charger (JST-XH 3-pin) into the charge socket under the tail. Loop plug (top of the tail) OUT while charging. Firmware updates: OTA over WiFi, or USB-C through the window in the right flank with the loop plug out."),
      p("Recovery: # (drills out), ! (pods up), $ (foot out), @ (kick back). The worms hold every position with the power off. "
        "Note: STOP (space, /stop, or a client hold ending) also aborts a running auto sequence; that is deliberate."),
      p("Pins (firmware v4): drills on DRV#2 A (D8/D9), foot on DRV#2 B (D6/D7). D6/D7 flicker at every reset; the slow foot worm does not care.", S)]
E += [PageBreak(), p("9. Wire cut list (pad to pad)", H2)]
rows = [["#", "From", "To", "Colour", "Gauge", "Cut mm", "Note"]] + [[str(w_[k]) for k in ("n", "fr", "to", "col", "g", "len", "note")] for w_ in WJ["wires"]]
E += [tbl(rows, [7, 44, 44, 13, 13, 11, 45], fs=7)]
E += [Spacer(1, 4), p("Totals by gauge and colour (mm): " + ", ".join(f"{k} {v}" for k, v in sorted(TOT.items())), S)]
E += [PageBreak(), p("11. Pictures", H2), p("Wiring (numbers match the cut list):", S), img(PK + "/wiring.png", 180),
      p("Exploded view and populated tray:", S), img(OUT + "/img_exploded.png", 88), img(OUT + "/img_tray_populated.png", 88),
      PageBreak(), p("Audit sheet: 24 views of the exact print geometry (8 folded, 8 deployed, 6 mid-motion, top, under).", S), img(OUT + "/audit_sheet.png", 180),
      p("Section cuts:", S), img(OUT + "/sections.png", 180)]
E += [p("12. Design notes", H2)] + [p("• " + t, S) for t in [
    "Drills: each gearmotor is fixed in its arm and sits inside its sleeve; the sleeve runs on the arm's bushing and shoulder (push goes into the arm), the motor shaft turns it through the jammed nut pair at the tip.",
    "The drill pair counter-rotates so the dart does not spin; the two augers are mirror images so both screw in.",
    "The deploy N20's own shaft stubs drive the pods through two Ø7 D couplers (PPA-CF) that run in the hull bosses. At full deploy the pod heads sit on the hull horns: tow pull turns the pods further down, so that load goes into the hull, not into the worm or the couplers.",
    "The kick rest on the ear root does the same for the kick leg: tow lifts the nose, the soil pushes the planted foot down, the pole lands on the rest.",
    "Tether load path: tail end wall, tail cap, 4 radial M3 into the hull. Tie the line through the two end holes round the web.",
    f"Ground geometry: kick stops at {P['KICK_DEG']}° with the foot housing on the ground and the foot auger {A['auger_depth_mm']['foot_auger']} mm in; drills stop at {P['DEPLOY_DEG']}° with the pods {list(A['pod_clear_mm'].values())[0]} mm clear and the augers {A['auger_depth_mm']['drill_leg_L_auger']} mm in.",
    "Power: 2S straight to the DRV8833 motor rail through the INA219; one MP1584 at 5.04 V for the XIAO, VTX and camera.",
    "Below the centreline the hull is a plain Ø42 cylinder for the launcher cradle; the folded pods sit just above it. Nothing is plugged into the belly: the ON plug is recessed in the top of the tail cap."]]
doc = SimpleDocTemplate(PK + f"/KEDGE_{V}_run_sheet.pdf", pagesize=A4, leftMargin=16 * mm, rightMargin=16 * mm, topMargin=14 * mm, bottomMargin=14 * mm,
                        title=f"KEDGE {V} run sheet", author="")
doc.build(E)
print("pdf ok")
