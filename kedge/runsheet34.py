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
RP = P.get("ROPE", {}); BG = P.get("BALLAST_G", 0); LG = RP.get("knot_behind_tail_mm", 26)
E += [p("1. What is new in v34 (read once)", H2)] + [p("• " + t, S) for t in [
    "The drill pods moved to the TAIL. Folded they point forward along the flanks; deployed they swing down to a printed hard stop, so the drills go into the soil near the tail, under the tether.",
    "The tether ties straight to the TAIL: the rover line goes in one of the two small holes in the middle of the tail face, out the other, and is tied to itself. "
    "It runs straight back through the launcher pusher's middle hole. (The small eye lugs on the pod arms are spare; leave them empty.)",
    "The kick leg and the foot auger are gone. A slim RIGHTING LEG lies flush in the groove along the top; its pad is the curved strip on the tail top. It swings over the nose to flip the dart upright. The foot N20 is a spare.",
    f"{BG} g of your ballast putty goes into the NOSE, round and under the camera tube (it helps the dart fly straight).",
    "Every coupler is a PRESS FIT with a drop of CA: no play between a motor and its pods or its leg. Even half a degree of play lets one pod load the worm alone.",
    "Both sockets are in the tail flanks under the pad: ON loop on the left, charge on the right. The USB-C window in the right flank is open down to the split so the hull top drops straight on."]]
E += [p("2. Bought parts and consumables, in the order they go in", H2), tbl([
    ["Part", "Qty", "Where it goes (step)"],
    ["Seeed XIAO ESP32S3 (plain), flashed", "1", "deck 1, USB-C to the right flank window (5.4)"],
    ["DRV8833 carrier", "2", "standing beside the deploy motor, left = DRV#1, right = DRV#2 (5.3)"],
    ["INA219 (screw terminal removed)", "1", "deck 1 behind the XIAO (5.4)"],
    ["BNO085", "1", "deck 1 behind the INA219, silkscreen arrow to the nose (5.4)"],
    ["MP1584 set to 5.04 V + Schottky diode", "1 each", "buck on deck 2 behind the VTX; diode in the 5 V lead to the XIAO (5.4)"],
    ["Electrolytic capacitor 470 to 1000 uF (up to Ø8 x 12)", "1", "standing on the tray's tail key behind the deploy motor, a dab of epoxy; leads to the MP1584 output (5.4)"],
    ["OTOTEC 2S BMS stick", "1", "deck 1, left of the XIAO and INA219 (5.4)"],
    ["HGLRC Zeus nano VTX + u.FL antenna", "1", "deck 2 (VTX), antenna along deck 1's left edge (5.4)"],
    ["Caddx Ant camera", "1", "nose, from the back (5.6)"],
    ["Lead/tungsten putty", f"{BG} g", "pressed into the nose from the back, round and under the camera tube (5.6)"],
    ["HIBATT 18350 cell", "2", "in line in the U saddles under deck 1: front spring, BM plate, rear plate (5.5)"],
    ["Battery spring and the two flat contact plates from the AA holders", "1 + 2", "spring on disc 1; one plate (its solder tab up) between the cells as the BM tap; one plate on the rear disc (5.5)"],
    ["Worm N20 (right-angle, self-locking)", "2", "leg motor in the front cradle (5.1), deploy motor in the rear cradle (5.2); motors point down"],
    ["Drill gearmotor M3 x 56 threaded shaft", "2", "in the pod arms (6.1)"],
    ["M3 nut", "2", "pressed in the auger hub hex pockets (6.1)"],
    ["JST-XH 2-pin socket + shorting plug; JST-XH 3-pin socket", "1 each", "left and right flank seats, clamped by the two hull halves (6.2)"],
    ["M3 x 8 countersunk screw", "8", "radial hull screws into the bosses on disc 1 and the rear disc (6.4)"],
    ["M3 x 8 screw", "2", "pod caps into the pod couplers (6.5)"],
    ["1 mm Dyneema (from the 20 m reel)", "the rover line", "the tether (8)"],
    ["CA glue, 5-minute epoxy, threadlocker, double-sided tape", "", "every coupler fit and the skin plugs (CA); drill-lead holes and the capacitor (epoxy); drill shafts (threadlocker); DRV boards (tape)"]], [70, 14, 92])]
E += [p("Tools: soldering iron, flush cutters, small files, a hacksaw or a rotary tool with a cut-off disc for the leg N20 stubs (flush cutters will not cut the 3 mm steel shaft), "
        "a vice or clamp, a pencil (to press the putty), a luggage scale (tether test), a multimeter.", S)]
E += [PageBreak(), p("3. Print", H2),
      p("Bambu Studio: PLA, 0.4 nozzle, 0.16 mm layers, 4 wall loops (the 1.5 mm walls come out solid), 40% gyroid infill, top/bottom 5 layers. "
        "Plate C in PPA-CF (hardened nozzle, dried filament). Every STL is already in its print orientation. Print the fit coupon FIRST.")]
E += [p("Fit coupon: fit_coupon.stl in PLA, fit_coupon_pegs.stl (two short D pegs) in PPA-CF. On the v34 row (the last row): the Ø7 peg goes into the "
        "Ø7.05 D hole (pod coupler in the arm head), the Ø6 peg into the Ø6.05 D hole (leg coupler in the cheek), and each peg's small D socket goes "
        "onto a real N20 stub (Ø3.05, flat at 1.03). Every one must press on by hand with a firm push, must not turn at all, and must not fall out. "
        "Also try the JST headers in the two seats (3-pin 10.2 tall, 2-pin 7.9). If a fit is too tight to press or it turns, tell Claude which one before "
        "printing the parts: that bore is changed and the parts reprinted.", S)]
rows = [["Part", "Print as", "Supports", "Box mm", "cm3"]]
for n, r in PR["parts"].items():
    rows.append([n, r.get("orient") or "", r.get("supports") or "", " x ".join(str(round(v)) for v in (r.get("print_bbox") or [])), r.get("volume_cm3")])
E += [tbl(rows, [30, 60, 58, 20, 10], fs=7)]
E += [p("Plates: " + "; ".join(f"{k}: {', '.join(v['parts'])}" for k, v in PR["plates"].items()), S)]
E += [p("4. Before the build", H2)] + steps([
    "Flash the XIAO over USB with the firmware (firmware/dart). In v34 the kick channel drives the righting leg and the foot channel is not used.",
    "Set MP1584 #2 to 5.04 V on the bench supply BEFORE it is connected to anything. Remove the INA219 screw terminal.",
    "Bench-run each motor for a second on 8 V: both worm N20 (leg and deploy), both drill motors.",
    "Trim BOTH output stubs of the leg N20 to 5.8 mm past the worm box faces (shaft tips %.1f mm from the box centre) with the hacksaw or the cut-off disc: clamp the gearbox, cut slowly and let it cool. File the cut ends smooth; keep the D flats." % P["LEG_STUB"],
    "Print the plates. Check the tray's tongues (two small blocks on top of the front cradle) and the hull top's two stub slots under the knuckle: the tongues must slide into the slots."])
E += [p("5. Populate the tray (tray out of the hull: everything is open from above and from the sides)", H2)]
E += [p("5.1 Leg motor", H3)] + steps([
    "Solder 26 AWG silicone leads (red +, black -) to the leg N20 end-cap tabs, bend the tabs flat.",
    "Drop the motor, can first, into the FRONT cradle; the worm box rests on the cradle's front and back walls, the output shaft across the dart. The leads leave through the slot under the cradle."])
E += [p("5.2 Deploy motor", H3)] + steps([
    "Leads on as 5.1. Drop it can first into the REAR cradle, output shaft across. Leads under the cradle."])
E += [p("5.3 Drivers", H3)] + steps([
    "Solder the DRV8833 wires on the bench (cut list numbers), then stand DRV#1 on the left side and DRV#2 on the right side of the rear cradle, chips facing out, double-sided tape. Run the wires along the low corridors beside the cells."])
E += [p("5.4 Boards on the decks", H3)] + steps([
    "Pre-wire each board on the bench, then slide it in from the side between deck 1 and deck 2 (there is no room for a soldering iron under deck 2).",
    "Deck 1: XIAO at the front right with its USB-C at the right edge, sitting on the small tab at the deck edge (it must line up with the hull notch); INA219 behind it; BMS stick on the left; BNO085 behind the INA219, arrow to the nose.",
    "Deck 2: VTX at the front, MP1584 behind it. Antenna along deck 1's left edge, XIAO FPC antenna flat on the right flank wall beside the BNO085.",
    "Capacitor: stand it on the tray's tail key (the flat tongue behind the deploy motor), a dab of epoxy; long leg to MP1584 OUT+, striped leg to OUT-."])
E += [p("5.5 Cells", H3)] + steps([
    "Glue the spring in disc 1's rear recess, its lead out through the hole in disc 1 (B-). Solder the BM wire to the solder tab of one AA-holder plate; pass the tab up through the slot in deck 1 at the cell joint.",
    "Glue the other plate in the rear disc's front recess, lead through the hole (B+).",
    "Turn the tray upside down. The four U saddles are open: lay cell 1 in, BASE forward against the spring; the BM plate against its button; then cell 2, base against the plate, button against the rear plate. "
    "The spring holds them. Check 7.4 to 8.4 V between B- and B+."])
E += [p("5.6 Camera, nose putty and harness", H3)] + steps([
    "Camera into the nose from the back (lens into the 1.2 lip).",
    f"Nose putty: with the nose spigot up, press {BG} g of putty in from the back with a pencil end, FRONT FIRST, round and under the camera tube (never into the camera pocket), up to the ring step "
    f"about {P.get('BAL_XE', 13):.0f} mm from the tip. Press it firm: the step holds it at launch.",
    "The camera's three wires go aft through the nose spigot, beside the leg cradle, through disc 1's left window to the VTX.",
    f"Solder the harness from the cut list (section 9), {NW} wires, along the routes named there. Test with the power loop plug: XIAO boots, video on the receiver.",
    "Motor direction check (now, while the drivers are reachable): put a marker dot on each motor stub. Seen from the LEFT side of the dart (nose to your right), key 1 must turn the deploy stub CLOCKWISE "
    "(that swings the drills DOWN) and key 2 must turn the leg stub CLOCKWISE (that lifts the leg off the tail and over the nose). If one turns the wrong way, swap that motor's two leads at DRV#1."])
E += [PageBreak(), p("6. Close the hull, fit the pods and the leg", H2)]
E += [p("6.1 Drill pods (off the dart)", H3)] + steps([
    "Push each drill motor into its arm's bushing, flats keying it; leads out of the neck top. A drop of CA on the can.",
    "Press an M3 nut into each auger's hex pocket. Threadlocker on the shaft, screw the auger on until the hub touches the arm shoulder, back off half a turn. Left pod = right-hand flight (brighter red arm)."])
E += [p("6.2 Hull bottom", H3)] + steps([
    "Seat the two JST sockets in the lower halves of the flank seats (left = 2-pin ON loop, its seat sits a little low; right = 3-pin charge), pins inward.",
    "Thread the tether through the two tail holes now (section 8, first step): once the tray is in you cannot reach the inside of the tail.",
    "Lower the populated tray, with the nose on its wires, straight down into the hull bottom: deck 1's edges drop into the seam groove, the tail key into the slot in the end wall, the nose spigot onto the lock rib, the cells onto the belly."])
E += [p("6.3 Drill leads", H3)] + steps([
    "Pass each pod's two leads from outside in through the tight hole in the hull top just in front of its pivot block (before soldering), leave a 25 mm loop outside for the swing, then solder them to DRV#2 (right drill leads swapped). A dab of 5-minute epoxy seals each hole."])
E += [p("6.4 Hull top", H3)] + steps([
    "Lower the hull top STRAIGHT DOWN: its two stub slots slide over the leg N20 stubs and onto the tray tongues, the USB-C notch drops over the XIAO receptacle onto the tray tab, the tail key goes into its end-wall slot, "
    "the rib locks the nose, the sockets are clamped. Nothing is trapped: look round the seam.",
    "8 radial M3 x 8 countersunk screws into the bosses, 4 at disc 1 and 4 at the rear disc. The heads sit flush. Do not over-tighten (they cut their own thread)."])
E += [p("6.5 Pods on the dart (press fits, CA)", H3)] + steps([
    "Dry run first: each Ø7 PPA-CF coupler must push onto the deploy stub and into its arm head with a firm push and no play. If a fit is loose, use the fit coupon to choose a reprint.",
    "One side at a time: a drop of CA in the coupler's stub socket and a thin film on its D. Hold the arm FOLDED, ON ITS FOLD STOP (pod pointing forward along the flank, head down on the block). "
    "Push the coupler through the arm head and the block bore onto the stub (flat up) until it is home. Then the other side, the same way.",
    "At once, while the CA is still setting: press key ! (pods up) and hold it for 2 s so the worm drives to stall toward fold with both pods on their fold stops. Leave it 5 minutes.",
    "Pod cap + M3 x 8 into each coupler end.",
    "Power the deploy motor slowly: both pods swing down together to the stop and back to folded. They must stop on the printed stops, not on the motor. Wiggle each pod by hand: nothing may move."])
E += [p("6.6 Righting leg (press fits, CA)", H3)] + steps([
    "Lay the leg folded: cheeks into the two side pockets, blade flat on the groove floor (its fold stop), pad on the tail top.",
    "A drop of CA in each Ø6 leg coupler's stub socket and on its D. Push each coupler (PPA-CF) in through the plug counterbore on the flank, through the cheek and the knuckle wall onto the leg N20 stub (flat up).",
    "At once: press key @ (leg back) for 2 s so the worm drives to stall toward fold with the leg on its groove floor. Leave it 5 minutes.",
    "Press the two skin plugs in flush (a drop of CA). Their blind bores hold the couplers and carry the leg.",
    "Power the leg motor slowly: the leg swings over the top to its bed in the nose (about 178°) and back flush into the groove. Wiggle it: nothing may move."])
E += [p("7. Function test", H2)] + steps([
    "ON plug: make its shorting loop from a short wire so it folds flat down against the flank, and tape it there (a loop sticking out can snag and switch the dart off). Plug in: XIAO and video up, IMU reads level.",
    "Key 1: the drills swing DOWN, key 2: the leg lifts off the tail and swings over the nose (else swap that motor's leads at DRV#1, 5.6). Pods down to the stops with the drills spinning (left and right counter-rotate), then up.",
    "Service: pry the two skin plugs, pull the leg couplers (CA: they may need a twist), remove the 8 radial screws and lift the hull top. Everything is reachable."])
E += [p("8. Tether", H2)] + steps([
    "BEFORE the tray goes into the hull bottom (6.2): push the end of the rover line in through one of the two small holes in the middle of the tail face (from outside), "
    "then back out through the other hole, so the line makes a small loop round the plastic between the holes on the inside. Pull 20 cm through. "
    "(Stiffen the line end with a drop of CA and let it dry first: it threads much more easily.)",
    "Tie the end to the line itself right behind the tail with a BOWLINE, then a double overhand stopper on the tail end. Trim the tail to 5 mm.",
    "Test: hook a luggage scale into the line, hold the dart and pull 6 kg for a few seconds. Nothing may slip.",
    "Loading on the launcher: press key ! (pods up) for 1 s so both pods sit hard on their fold stops. Put the dart on the cradle; the knot goes into the pusher's middle hole and the line runs straight back."])
E += [PageBreak(), p("9. Wire cut list (pad to pad)", H2)]
rows = [["#", "From", "To", "Colour", "Gauge", "mm", "Note"]] + [[w["n"], w["fr"], w["to"], w["col"], w["g"], w["len"], w["note"]] for w in WJ["wires"]]
E += [tbl(rows, [7, 44, 44, 13, 13, 11, 45], fs=7)]
E += [Spacer(1, 4), p("Totals by gauge and colour (mm): " + ", ".join(f"{k} {v}" for k, v in sorted(TOT.items())), S)]
E += [PageBreak(), p("10. Pictures", H2), p("Exploded view, and the build stages: populated tray, tray in the hull bottom, hull closed, complete.", S),
      img(OUT + "/img_exploded.png", 180), img(OUT + "/img_stage1_tray.png", 88), img(OUT + "/img_stage2_in_bottom.png", 88),
      img(OUT + "/img_stage3_closed.png", 88), img(OUT + "/img_stage4_complete.png", 88),
      PageBreak(), p("Wiring (numbers match the cut list):", S), img(PK + "/wiring.png", 180),
      p("Audit sheet: 24 views of the exact print geometry.", S), img(OUT + "/audit_sheet.png", 180), p("Section cuts:", S), img(OUT + "/sections.png", 180)]
doc = SimpleDocTemplate(PK + f"/KEDGE_{V}_run_sheet.pdf", pagesize=A4, leftMargin=16 * mm, rightMargin=16 * mm, topMargin=14 * mm, bottomMargin=14 * mm,
                        title=f"KEDGE {V} run sheet", author="")
doc.build(E)
print("run sheet", PK + f"/KEDGE_{V}_run_sheet.pdf")
