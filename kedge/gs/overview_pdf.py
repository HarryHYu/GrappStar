"""Whole-system overview (dart + ground station) for the write-up. python overview_pdf.py out ../pack_gs"""
import json, sys
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, PageBreak
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors

OUT, PK = sys.argv[1], sys.argv[2]
pdfmetrics.registerFont(TTFont("Arial", "/System/Library/Fonts/Supplemental/Arial.ttf"))
pdfmetrics.registerFont(TTFont("Arial-Bold", "/System/Library/Fonts/Supplemental/Arial Bold.ttf"))
P = json.load(open(f"{OUT}/gs_parts.json"))["P"]; PH = json.load(open(f"{OUT}/physics_gs.json"))
AU = json.load(open(f"{OUT}/audit_gs.json")); PR = json.load(open(f"{PK}/print_report.json")); DP = json.load(open("../out30/physics.json")); DA = json.load(open("../out30/audit.json"))
B = ParagraphStyle("b", fontName="Arial", fontSize=10, leading=13.5)
H1 = ParagraphStyle("h1", fontName="Arial-Bold", fontSize=17, leading=21, spaceAfter=6)
H2 = ParagraphStyle("h2", fontName="Arial-Bold", fontSize=12, leading=15, spaceBefore=8, spaceAfter=4)
def p(t, s=B): return Paragraph(t, s)
def img(path, w=170):
    from PIL import Image as PI
    im = PI.open(path); return Image(path, width=w * mm, height=w * im.height / im.width * mm)
def tbl(rows, widths):
    t = Table([[Paragraph(str(c), ParagraphStyle("c", fontName="Arial-Bold" if i == 0 else "Arial", fontSize=9, leading=11)) for c in r] for i, r in enumerate(rows)], colWidths=[w * mm for w in widths])
    t.setStyle(TableStyle([("GRID", (0, 0), (-1, -1), 0.4, colors.black), ("VALIGN", (0, 0), (-1, -1), "TOP")])); return t

S = []
S.append(p("GrappStar: a rover that anchors itself and winches out", H1))
S.append(p("A rover stuck in sand or mud cannot push itself out. GrappStar fires a small anchor dart (KEDGE) on a tether, the dart screws itself into the ground, and a reel on the rover winches against it. Everything on the rover side runs on one motor: a 12 V car wiper motor."))
S.append(Spacer(1, 4)); S.append(img(f"{PK}/renders/01_overview.png", 160))
S.append(p("1. The idea", H2))
S.append(p("Other approaches send a second robot to pull the first one out. GrappStar keeps it to one rover and one throw. The dart does the anchoring with its own small motors; the rover side does three jobs (launch, reel in, tow) with a single motor, using the motor's direction and two sets of one-way clutches to pick the job. Fewer motors means fewer things to fail and less mass to carry."))
S.append(p("2. The dart (KEDGE v30)", H2))
S.append(p("A fully printed 42 mm dart, 168 mm long folded, about %.0f g including 15 g of nose ballast. After landing a kick leg swings %.0f deg over the nose and screws a foot auger in ahead of it, then two pods swing down %.1f deg while two counter-rotating augers (17 mm x 72 mm, opposite hands so the dart does not twist) drill %.0f mm deep. A camera in the nose sends live video to the rover; an ESP32-S3 runs the sequence and talks over WiFi." %
           (DP["mass_g"] + 15, 201.75, 42.25, DA["auger_depth_mm"]["drill_leg_L_auger"])))
rows = [["predicted holding (Broms short pile model)", "pull before it moves"]]
for k, v in DP["anchoring"]["by_soil"].items():
    rows.append([k, "%.0f N" % v["total_N"]])
S.append(tbl(rows, [100, 60]))
S.append(PageBreak())
S.append(p("3. One motor, three jobs", H2))
S.append(p("<b>Backward:</b> a one-way clutch locks and the program gear turns once per shot. Its 18 teeth pull the carriage back %.0f mm against two springs (%.0f to %.0f N), a sear catches it, then a cam beside the gear lifts the launch rail from 4 to 35 deg. Near the top a cord tied to the base goes taut, pulls the sear down, and the dart leaves at %.1f m/s. "
           "<b>Forward:</b> the launcher clutch freewheels and a backstop holds it; the reel rotor winds. The spool itself never turns: at launch the line peels off its end (like a spinning fishing reel), so there is nothing to spin up and no tangle. "
           "<b>Stopped:</b> the wiper's worm gear is self locking, so everything holds with the power off." % (P["STROKE"], PH["springs"]["F_rest_N"], PH["springs"]["F_full_N"], PH["launch"]["v_muzzle"])))
S.append(img(f"{PK}/renders/05_sear.png", 120))
rows = [["result (computed)", "value"],
        ["muzzle speed / range (fires at %.0f deg)" % PH["launch"]["angle_deg"], "%.1f m/s / %.1f m (drag and tether included)" % (PH["launch"]["v_muzzle"], PH["flight"]["range_m"])],
        ["peak launch acceleration", "%.0f g" % PH["launch"]["a_peak_g"]],
        ["peak motor load / one shot cycle", "%.1f N m / %.1f s" % (PH["motor"]["T_motor_max_Nm"], PH["motor"]["cycle_s"])],
        ["tow speed at 20 N", "%.2f m/s" % [r for r in PH["reel"]["rows"] if r["T_line_N"] == 20][0]["line_m_s"]],
        ["printed parts / plastic", "%d parts, about %.1f kg (gears, sear and carriage in carbon-filled nylon)" % (sum(r.get("qty",1) for r in PR), sum(r["mass_g"]*r.get("qty",1) for r in PR)/1000)]]
S.append(tbl(rows, [70, 100]))
S.append(p("4. How it was checked before printing", H2))
for t in ["Every part is generated from one parameter file, so a change ripples through gears, cam, sear, rail and reel together.",
          "Collision sweep on the exact meshes: the whole mechanism stepped through the program gear's full turn in 1 deg steps (360 positions), with the dart, sear, carriage, springs and reel moving: %s." % ("no collisions" if not AU["collisions"] else "collisions found"),
          "The lift cam is not drawn by hand: it is the envelope of the roller over the programmed rail angle, then checked by solving the real rail angle from the cam.",
          "The rack travel is found by simulating the gear teeth pushing the rack tooth by tooth, which showed the first tooth catches early and the last lets go late; the design was changed to match.",
          "Launch, flight and tow are computed with spring, friction, drag and tether forces; the HD simulation plays these numbers back on the real parts.",
          "Firmware is tested on a PC against a model of the machine (105 automatic checks) before it touches the motor.",
          "Six review rounds attacked the design; their findings (sear catch, trip cord, spring stress, clutches, the planet-carrier lift torque, reel and retrieve, fixings, print and assembly order) were fixed and re-checked."]:
    S.append(p("- " + t))
S.append(p("5. What is still to measure", H2))
S.append(p("The wiper motor's real speed and torque, the bought springs' rate, and the holding force in real soil (a pull-test script logs force against movement using the rover's own reel)."))
doc = SimpleDocTemplate(f"{PK}/GrappStar_system_overview.pdf", pagesize=A4, leftMargin=20 * mm, rightMargin=20 * mm, topMargin=16 * mm, bottomMargin=16 * mm)
doc.build(S); print("overview ok")
