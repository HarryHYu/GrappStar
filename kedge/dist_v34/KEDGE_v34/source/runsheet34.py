"""KEDGE v34 build-day run sheet PDF (plain black, Arial-metric font).  Run by runsheet.py when the build has a righting leg.
python3 runsheet34.py out34 pack_v34"""
import json, sys, os, csv
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, Image
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
OUT, PK = sys.argv[1], sys.argv[2]
_F = [("/System/Library/Fonts/Supplemental/Arial.ttf", "/System/Library/Fonts/Supplemental/Arial Bold.ttf"),
      ("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf", "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf")]
_reg, _bold = next(f for f in _F if os.path.exists(f[0]))
pdfmetrics.registerFont(TTFont("Arial", _reg)); pdfmetrics.registerFont(TTFont("Arial-Bold", _bold))
J = json.load(open(OUT + "/parts.json")); P = J["P"]; A = json.load(open(OUT + "/audit.json"))
PR = json.load(open(PK + "/print_report.json")); WJ = json.load(open(PK + "/wires.json"))
MP = json.load(open(OUT + "/mass_props.json")) if os.path.exists(OUT + "/mass_props.json") else {}
LF = json.load(open(OUT + "/launcher_fit.json")) if os.path.exists(OUT + "/launcher_fit.json") else {}
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
    if not os.path.exists(path): return p("(missing picture: %s)" % os.path.basename(path), S)
    im = PI.open(path); r = im.size[1] / im.size[0]
    return Image(path, width=w_mm * mm, height=w_mm * r * mm)
def steps(items): return [p("%d. %s" % (i + 1, t)) for i, t in enumerate(items)]
V = J.get("version", "v34"); T = MP.get("totals", {}); fo = T.get("folded", {}); an = T.get("deployed", {})
fit = "FITS" if LF and not LF.get("fails") else ("see the launcher notes" if LF else "not run")
E = []
E += [p(f"KEDGE dart {V}: build day run sheet", H1),
      p(f"Dart: Ø42 x {P['L']:.0f} mm, PLA on the X2D (couplers PPA-CF). Righting leg 0 to {P['LEG_DEG']:.0f}°, drill pods at the tail 0 to {P['DEPLOY_DEG']}°. "
        f"Collision sweep clean (leg in {A['kick_steps']} steps, pods in {A['deploy_steps']} steps, every sub-part, exact meshes); hard stops checked "
        f"(pods about 1° past {P['DEPLOY_DEG']}° and past folded, the leg on its groove floor folded and in its bed at {P['LEG_DEG'] + P['LEG_REST_DEG']:.1f}°). "
        f"Mass about {fo.get('mass_g', '?')} g with the bought parts, centre of mass {fo.get('cg_mm', ['?', '?', '?'])[2]} mm from the hull axis (folded). Launcher fit: {fit}."),
      p("Rules for the whole day: the power loop plug stays OUT while you solder, move a wire or plug in USB. Never let the two cells touch metal "
        "together. One step at a time; each step says what you should see before you move on.")]
E += [p("1. What is new in v34 (read once)", H2)] + [p("• " + t, S) for t in [
    "The drill pods moved to the TAIL. Folded they point forward along the flanks; deployed they swing down to a printed hard stop, so the drills go into the soil near the tail, under the tether.",
    "The tether is a short BRIDLE: one leg to the eye lug on each pod arm, joined in a ring behind the tail. There is no tail boom and no tail cap: the two hull halves close the tail and their flat end is what the launcher pusher pushes.",
    "The kick leg and the foot auger are gone. A slim RIGHTING LEG lies flush in the groove along the top; its pad is the curved strip on the tail top. It swings over the nose to flip the dart upright. The foot N20 is a spare.",
    "The two cells lie in line on the belly and both worm motors point down, so the centre of mass sits on the hull axis and the dart rests belly down. "
    f"{P.get('BALLAST_BELLY_G', 0)} g of your ballast putty goes into two pockets in the belly rails to finish the balance.",
    "Both sockets are in the tail flanks under the pad: ON loop on the left, charge on the right. The USB-C window in the right flank is tight round the receptacle."]]
E += [p("2. Bought parts, in the order they go in", H2), tbl([
    ["Part", "Qty", "Where it goes (step)"],
    ["Seeed XIAO ESP32S3 (plain), flashed", "1", "deck 1, USB-C to the right flank window (5.4)"],
    ["DRV8833 carrier", "2", "standing beside the deploy motor, left = DRV#1, right = DRV#2 (5.3)"],
    ["INA219 (screw terminal removed)", "1", "deck 1 behind the XIAO (5.4)"],
    ["BNO085", "1", "deck 1 behind the INA219, silkscreen arrow to the nose (5.4)"],
    ["MP1584 set to 5.04 V + 470 to 1000 uF capacitor + Schottky diode", "1 each", "buck on deck 2 behind the VTX; capacitor on deck 1, left; diode in the 5 V lead to the XIAO (5.4)"],
    ["OTOTEC 2S BMS stick", "1", "deck 1, left of the XIAO and INA219 (5.4)"],
    ["HGLRC Zeus nano VTX + u.FL antenna", "1", "deck 2 (VTX), antenna along deck 1's left edge (5.4)"],
    ["Caddx Ant camera", "1", "nose, from the back (5.6)"],
    ["HIBATT 18350 cell", "2", "in line under deck 1: front spring, BM strip, rear plate (5.5)"],
    ["Battery spring, contact plate, 0.3 nickel strip with a tab", "1 each", "spring on disc 1, strip between the cells, plate on the rear disc (5.5)"],
    ["Worm N20 (right-angle, self-locking)", "2", "leg motor in the front cradle (5.1), deploy motor in the rear cradle (5.2); motors point down"],
    ["Drill gearmotor M3 x 56 threaded shaft", "2", "in the pod arms (6.1)"],
    ["M3 nut", "2", "pressed in the auger hub hex pockets (6.1)"],
    ["JST-XH 2-pin socket + shorting plug; JST-XH 3-pin socket", "1 each", "left and right flank seats, clamped by the two hull halves (6.3)"],
    ["Lead/tungsten putty", f"{P.get('BALLAST_BELLY_G', 0)} g", "pressed flush into the two belly-rail pockets in the hull bottom (6.2)"],
    ["M3 x 8 countersunk screw", "8", "radial hull screws into disc 1 and the rear disc (6.4)"],
    ["M3 x 8 screw", "2", "pod caps into the pod couplers (6.5)"],
    ["Threadlocker, a drop of CA, a dab of silicone or hot glue, double-sided tape", "", "drill shafts; skin plugs; drill-lead holes; DRV boards"]], [70, 14, 92])]
E += [PageBreak(), p("3. Print", H2),
      p("Bambu Studio: PLA, 0.4 nozzle, 0.16 mm layers, 4 wall loops (the 1.5 mm walls come out solid), 40% gyroid infill, top/bottom 5 layers. "
        "Plate C in PPA-CF (hardened nozzle, dried filament). Every STL is already in its print orientation.")]
rows = [["Part", "Print as", "Supports", "Box mm", "cm3"]]
for n, r in PR["parts"].items():
    rows.append([n, r.get("orient") or "", r.get("supports") or "", " x ".join(str(round(v)) for v in (r.get("print_bbox") or [])), r.get("volume_cm3")])
E += [tbl(rows, [30, 60, 58, 20, 10], fs=7)]
E += [p("Plates: " + "; ".join(f"{k}: {', '.join(v['parts'])}" for k, v in PR["plates"].items()), S)]
E += [p("4. Before the build", H2)] + steps([
    "Flash the XIAO over USB with the firmware (firmware/dart). In v34 the kick channel drives the righting leg and the foot channel is not used.",
    "Set MP1584 #2 to 5.04 V on the bench supply BEFORE it is connected to anything. Remove the INA219 screw terminal.",
    "Bench-run each motor for a second on 8 V: both worm N20 (leg and deploy), both drill motors. Note which way each turns.",
    "Trim BOTH output stubs of the leg N20 to 5.8 mm past the worm box faces (shaft tips %.1f mm from the box centre). File the cut ends smooth; keep the D flats." % P["LEG_STUB"],
    "Print the plates. Check the tray's tongues (two small blocks on top of the front cradle) and the hull top's two stub slots under the knuckle: the tongues must slide into the slots."])
E += [p("5. Populate the tray (tray out of the hull: everything is open from above and from the sides)", H2)]
E += [p("5.1 Leg motor", H3)] + steps([
    "Solder 26 AWG silicone leads (red +, black -) to the leg N20 end-cap tabs, bend the tabs flat.",
    "Drop the motor, can first, into the FRONT cradle; the worm box rests on the cradle's front and back walls, the output shaft across the dart. The leads leave through the slot under the cradle.",
    "Power it briefly so both D flats face UP (the couplers go on later with their flats up)."])
E += [p("5.2 Deploy motor", H3)] + steps([
    "Leads on as 5.1. Drop it can first into the REAR cradle, output shaft across, D flats up. Leads under the cradle."])
E += [p("5.3 Drivers", H3)] + steps([
    "Solder the DRV8833 wires on the bench (cut list numbers), then stand DRV#1 on the left side and DRV#2 on the right side of the rear cradle, chips facing out, double-sided tape. Run the wires along the low corridors beside the cells."])
E += [p("5.4 Boards on the decks", H3)] + steps([
    "Pre-wire each board on the bench, then slide it in from the side between deck 1 and deck 2 (there is no room for a soldering iron under deck 2).",
    "Deck 1: XIAO at the front right with its USB-C at the right edge (it must line up with the hull window); INA219 behind it; BMS stick on the left; capacitor on the left edge; BNO085 behind the INA219, arrow to the nose.",
    "Deck 2: VTX at the front, MP1584 behind it. Antenna along deck 1's left edge, XIAO FPC antenna flat on the right flank wall beside the BNO085."])
E += [p("5.5 Cells", H3)] + steps([
    "Glue the spring in disc 1's rear recess, its lead out through the hole in disc 1 (B-). Solder the BM wire to the nickel strip's tab; pass the tab up through the slot in deck 1 at the cell joint.",
    "Glue the contact plate in the rear disc's front recess, lead through the hole (B+).",
    "Slide cell 1 in from the side under deck 1, BASE forward against the spring; the strip against its button; then cell 2, base against the strip, button against the rear plate. Check 7.4 to 8.4 V between B- and B+."])
E += [p("5.6 Camera and harness", H3)] + steps([
    "Camera into the nose from the back (lens into the 1.2 lip). Its three wires go aft through the nose spigot, beside the leg cradle, through disc 1's left window to the VTX.",
    f"Solder the harness from the cut list (section 8), {NW} wires, along the routes named there. Test with the power loop plug: XIAO boots, video on the receiver."])
E += [PageBreak(), p("6. Close the hull, fit the pods and the leg", H2)]
E += [p("6.1 Drill pods (off the dart)", H3)] + steps([
    "Push each drill motor into its arm's bushing, flats keying it; leads out of the neck top. A drop of CA on the can.",
    "Press an M3 nut into each auger's hex pocket. Threadlocker on the shaft, screw the auger on until the hub touches the arm shoulder, back off half a turn. Left pod = right-hand flight (brighter red arm)."])
E += [p("6.2 Hull bottom", H3)] + steps([
    f"Press {P.get('BALLAST_BELLY_G', 0)} g of putty flush into the two pockets in the belly rails (half each side).",
    "Seat the two JST sockets in the lower halves of the flank seats (left = 2-pin ON loop, right = 3-pin charge), pins inward.",
    "Lower the populated tray into the hull bottom: deck 1's edges drop into the seam groove, the cells rest on the belly between the rails."])
E += [p("6.3 Drill leads", H3)] + steps([
    "Pass each pod's two leads from outside in through the tight hole in the hull top just in front of its pivot block (before soldering), leave a 25 mm loop outside for the swing, then solder them to DRV#2 (right drill leads swapped). A dab of silicone seals each hole."])
E += [p("6.4 Hull top", H3)] + steps([
    "Lower the hull top straight down: its two stub slots slide over the leg N20 stubs and onto the tray tongues, the nose spigot goes in front (the rib locks the nose), the sockets are clamped. Nothing is trapped: look round the seam.",
    "8 radial M3 x 8 countersunk screws, 4 into disc 1 and 4 into the rear disc. The heads sit flush."])
E += [p("6.5 Pods on the dart", H3)] + steps([
    "Hold each arm on its pivot block, folded. Push its PPA-CF coupler through the arm head and the block bore onto the deploy stub (flat up). Pod cap + M3 x 8 into the coupler end.",
    "Power the deploy motor slowly: both pods swing down together to the stop and back to folded. They must stop on the printed stops, not on the motor."])
E += [p("6.6 Righting leg", H3)] + steps([
    "Lay the leg folded: cheeks into the two side pockets, blade into the top groove, pad on the tail top.",
    "Push each Ø6 leg coupler (PPA-CF) in through the plug counterbore on the flank, through the cheek and the knuckle wall onto the leg N20 stub (flat up).",
    "Press the two skin plugs in flush (a drop of CA). Their blind bores hold the couplers and carry the leg.",
    "Power the leg motor slowly: the leg swings over the top to its bed in the nose (about 178°) and back flush into the groove."])
E += [p("7. Function test", H2)] + steps([
    "ON plug in: XIAO and video up, IMU reads level belly down (it should rest belly down on the bench now).",
    "Leg out to the nose and back; pods down to the stops with the drills spinning (left and right counter-rotate), then up.",
    "Service: pry the two skin plugs, pull the leg couplers, remove the 8 radial screws and lift the hull top. Everything is reachable."])
E += [PageBreak(), p("8. Wire cut list (pad to pad)", H2)]
rows = [["#", "From", "To", "Colour", "Gauge", "mm", "Note"]] + [[w["n"], w["fr"], w["to"], w["col"], w["g"], w["len"], w["note"]] for w in WJ["wires"]]
E += [tbl(rows, [7, 44, 44, 13, 13, 11, 45], fs=7)]
E += [Spacer(1, 4), p("Totals by gauge and colour (mm): " + ", ".join(f"{k} {v}" for k, v in sorted(TOT.items())), S)]
E += [PageBreak(), p("9. Pictures", H2), p("Exploded view, and the build stages: populated tray, tray in the hull bottom, hull closed, complete.", S),
      img(OUT + "/img_exploded.png", 180), img(OUT + "/img_stage1_tray.png", 88), img(OUT + "/img_stage2_in_bottom.png", 88),
      img(OUT + "/img_stage3_closed.png", 88), img(OUT + "/img_stage4_complete.png", 88),
      PageBreak(), p("Wiring (numbers match the cut list):", S), img(PK + "/wiring.png", 180),
      p("Audit sheet: 24 views of the exact print geometry.", S), img(OUT + "/audit_sheet.png", 180), p("Section cuts:", S), img(OUT + "/sections.png", 180)]
doc = SimpleDocTemplate(PK + f"/KEDGE_{V}_run_sheet.pdf", pagesize=A4, leftMargin=16 * mm, rightMargin=16 * mm, topMargin=14 * mm, bottomMargin=14 * mm,
                        title=f"KEDGE {V} run sheet", author="")
doc.build(E)
print("run sheet", PK + f"/KEDGE_{V}_run_sheet.pdf")
