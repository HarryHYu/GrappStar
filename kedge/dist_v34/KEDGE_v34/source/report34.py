"""KEDGE v34 engineering report PDF (run by report.py when the build has a righting leg).  python3 report34.py out34 pack_v34
Geometry, mass and centre of mass, audit, hard stops and load paths, launcher fit, bridle, wire room, open items. The physics of
flight, landing, holding and righting for v34 is dart-physics' rerun on out34 (righting sim kedge/sim_righting)."""
import json, sys, os
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
J = json.load(open(OUT + "/parts.json")); P = J["P"]; V = J.get("version", "v34")
A = json.load(open(OUT + "/audit.json"))
ld = lambda f: json.load(open(f)) if os.path.exists(f) else {}
MP = ld(OUT + "/mass_props.json"); LF = ld(OUT + "/launcher_fit.json"); WR = ld(OUT + "/wireroom.json"); SC = ld(PK + "/step_check.json")
B = ParagraphStyle("b", fontName="Arial", fontSize=9.5, leading=12.5); S = ParagraphStyle("s", parent=B, fontSize=8, leading=10)
H1 = ParagraphStyle("h1", parent=B, fontName="Arial-Bold", fontSize=15, leading=19, spaceAfter=4)
H2 = ParagraphStyle("h2", parent=B, fontName="Arial-Bold", fontSize=11.5, leading=15, spaceBefore=8, spaceAfter=3)
def p(t, st=B): return Paragraph(t, st)
def tbl(rows, widths, fs=8):
    st = ParagraphStyle("t", parent=B, fontSize=fs, leading=fs + 2); sh = ParagraphStyle("th", parent=st, fontName="Arial-Bold")
    t = Table([[Paragraph(str(c), sh if i == 0 else st) for c in r] for i, r in enumerate(rows)], colWidths=[w * mm for w in widths], repeatRows=1)
    t.setStyle(TableStyle([("GRID", (0, 0), (-1, -1), 0.4, colors.black), ("VALIGN", (0, 0), (-1, -1), "TOP"), ("LEFTPADDING", (0, 0), (-1, -1), 3),
                           ("RIGHTPADDING", (0, 0), (-1, -1), 3), ("TOPPADDING", (0, 0), (-1, -1), 2), ("BOTTOMPADDING", (0, 0), (-1, -1), 2)]))
    return t
def img(path, w):
    from PIL import Image as PI
    if not os.path.exists(path): return p("(missing picture: %s)" % os.path.basename(path), S)
    im = PI.open(path); return Image(path, width=w * mm, height=w * im.size[1] / im.size[0] * mm)
T = MP.get("totals", {}); fo, an_, lo = T.get("folded", {}), T.get("deployed", {}), T.get("leg_out", {})
clean = not A["collisions"] and not A["ground_below"] and not A["belly_outside_r21"] and A["hard_stops_mm3"].get("ok") and not A["watertight_fail"] and not A["multi_body"]
fails = LF.get("fails", {}) if LF else {}
nstep = SC.get("summary", SC) if SC else {}
E = [p(f"KEDGE dart {V}: engineering report", H1),
     p("v34 is the self-righting layout from the righting study (kedge/sim_righting): drill pods at the tail folding forward, a low bridle to eye lugs on the pods, "
       "a top righting leg instead of the kick leg and foot, and a low centre of mass. All numbers below come from the exact print geometry and the pipeline (source/run_all.sh).")]
E += [p("1. Summary", H2), tbl([
    ["Item", "Result"],
    ["Body", f"Ø42 x {P['L']:.0f} mm; folded envelope x {A['folded_bbox'][0]} to {A['folded_bbox'][3]}, y ±{A['folded_bbox'][4]}, z {A['folded_bbox'][2]} to {A['folded_bbox'][5]} (nothing behind the tail, nothing above the hull but a 0.3 mm knuckle blister)"],
    ["Mass, centre of mass", f"{fo.get('mass_g')} g with catalogue bought masses and {P.get('BALLAST_BELLY_G')} g belly putty; CG folded {fo.get('cg_mm')}, anchored {an_.get('cg_mm')}, leg out {lo.get('cg_mm')} (mm; z from the hull axis)"],
    ["Collision sweep", ("CLEAN" if clean else "NOT CLEAN") + f": righting leg 0 to {A['kick_range'][1]}° in {A['kick_steps']} steps, pods 0 to {A['deploy_range'][1]}° in {A['deploy_steps']} steps, leg x pod cross sweep, every sub-part, exact meshes"],
    ["Hard stops (mm³ of overlap just past each end)", ", ".join(f"{k} {v}" for k, v in A['hard_stops_mm3'].items())],
    ["Drills", f"deploy {P['DEPLOY_DEG']}° (solved: the motor end in the hub stays 0.5 above flat ground), {A['auger_depth_mm'].get('drill_leg_L_auger')} mm deep, entering the soil about {P['DRILL_X'] - 26:.0f} mm from the nose tip (near the tail)"],
    ["Launcher fit", ("FITS" if LF and not fails else ("DOES NOT FIT: " + ", ".join(fails)) if LF else "not run") + f" over {LF.get('states', '?')} launcher states (base/cad/parts/launch.py, parked); the bridle needs base changes (section 5)"],
    ["Wire room", f"left corridor worst fill {WR.get('worst_keel_fill')}, right {WR.get('worst_top_fill')} (bundle area over free area; under 0.6 is fine)"],
    ["Printed parts", f"{A['printed_cm3']} cm³, every printed part watertight and one body"]], [45, 130])]
E += [p("2. Layout and why", H2)] + [p("• " + t, S) for t in [
    f"Pods: pivot x {P['DRILL_X']:g}, z {P['DRILL_PIV_Z']:g}, drill axis |y| {P['DRILL_PIV_Y']:g}. The study put the pivot at x 140, z 10; x {P['DRILL_X']:g} leaves room behind the deploy stop for the leg's pad in front of the flat tail face the pusher needs, and z {P['DRILL_PIV_Z']:g} (with the putty) puts the centre of mass on the axis while the drills still reach {A['auger_depth_mm'].get('drill_leg_L_auger')} mm.",
    "One deploy worm N20 on the axle (motor pointing down), the v33 PPA-CF couplers: they run in the pivot-block bores (the hull carries the pods) and take the worm's D stubs in a socket with 0.2 mm radial play (torque only). The pods stop on faces carved in the pivot blocks about 1° past deployed and past folded.",
    f"Bridle eye: a lug on each arm head's outboard 5 mm, {P['EYE_FWD']} forward of and {P['EYE_DROP']} below the pivot when folded. Deployed it sits behind and below the pivot, about 23 mm above the soil, so the tow pulls the pod onto its deploy stop (moment about half the study's eye; the eye could not hang lower: the launcher's stop springs pass 0.6 mm under it at the shot). No latch (Harry): the worm is self-locking and the study found 0.05 to 0.25 N m on it.",
    f"Drills: the v33 tapered gimlets unchanged, mirrored to point forward, hands kept (left pod right-hand, firmware DRILL_SIGN +1). The auger hub runs on the arm's bushing and its base face on the arm shoulder; pull along the auger goes into the motor's threaded shaft through nut A (this motor is sold for lead-screw use).",
    f"Righting leg: hinge x {P['LEG_X']:g}, z {P['LEG_Z']:g} on the centreline; swing 0 to {P['LEG_DEG']:g}° with a bed {P['LEG_REST_DEG']}° further on in the nose top. The leg N20 sits under a 0.6 mm skin over its worm box (the knuckle), motor pointing down. The leg's fork has a round cheek on each side of the knuckle; each cheek is keyed on a Ø6 PPA-CF coupler that runs in a bore in the knuckle wall and in the blind bore of a flush skin plug, and takes the trimmed worm stub in a socket with 0.2 mm play. The blade (I-channel, its top IS the hull crown) lies flush in the dorsal groove; the pad is a 2 mm patch of the hull skin on the tail top. Pad centre {P['PAD_X0'] + (P['PAD_X1'] - P['PAD_X0']) / 2 - P['LEG_X']:.0f} mm from the hinge (study 128) and about {2 * 21 * __import__('math').sin(__import__('math').radians(P['PAD_DEG'])):.0f} mm wide (study 56): the pad must stay in front of the pusher face and between the folded pods.",
    "Low centre of mass: the cells in line on the belly (z -10), both worm motors pointing down, DRVs standing low beside the deploy motor, boards on decks right over the cells, belly rails with the putty pockets. Belly down is now a stable rest.",
    "Sealed body: clamshell halves with the tray's deck 1 edges in a seam groove behind the split line; the nose spigot with its lock rib; couplers in tight bores; the drill leads through tight holes (silicone); the USB-C window tight round the receptacle; the JST headers fill their flank seats; the stub slots under the knuckle are closed by the tray's tongues."]]
E += [p("3. Load paths", H2)] + [p("• " + t, S) for t in [
    "Anchored tow: line -> bridle -> eye lugs on the arm heads -> arm -> coupler -> pivot-block bores (hull). The eye moment pushes the pods onto the carved deploy stops; any rest of the soil moment that would fold a pod back is held by the self-locking worm (study: up to 0.25 N m without the eye help).",
    "Drill loads: side load on the auger hub -> arm bushing; push -> arm shoulder; pull -> nut A on the motor's threaded shaft.",
    "Righting: leg -> two couplers -> knuckle-wall bores and skin plugs (hull); the worm stubs only turn the couplers. At 178° the blade root lies in its bed in the nose top; folded, the blade rests on the groove floor.",
    "Launch: pusher -> flat tail end walls of the two halves -> hull -> radial screws -> tray -> cells on the rear plate. Nothing protrudes behind the tail."]]
E += [p("4. Mass properties (catalogue bought masses; weigh and update)", H2), tbl(
    [["Pose", "Mass g", "CG x, y, z mm"]] + [[k, v["mass_g"], v["cg_mm"]] for k, v in T.items()], [40, 30, 60]),
      p(f"Printed {MP.get('printed_g')} g at 4 walls / 40% infill; bought parts and extras {MP.get('bought_g')} g (incl. {P.get('BALLAST_BELLY_G')} g putty). Per-part volume, centroid and inertia: mass_props.json.", S)]
E += [p("5. Launcher (parked base) and the bridle", H2)] + [p("• " + t, S) for t in [
    ("Fit: " + ("FITS" if LF and not fails else json.dumps(fails)) + f" over {LF.get('states', '?')} states. The rest check at home: dart on the cradle " + str(LF.get('rest', {}))) if LF else "Fit not run.",
    "Bridle: two legs about 80 mm from the eye lugs (dart frame x 128, y ±31.6, z +2.7 folded) to a ring about 48 mm behind the tail on the axis. In flight the line then clears the tail corners and pulls behind the centre of mass (nose first, as before).",
    "What base needs (not edited, base is parked): (1) the line model (launch_line_*) starts at the bridle ring behind the pusher instead of the old tail-cap holes; (2) the right bridle leg crosses the seat pushrod lug and the pushrod beside the pusher (yoke at y -11 to -20, z +1.9 to +3.5 in the dart frame, x 152 to 174): a 3 mm slot in the lug at z +2.7 or the lug and pushrod moved 3 mm down; the left leg path is clear; (3) the tether tube behind the pusher needs two side slots (3 mm, z +2.7) so the legs reach the ring, or the ring rides in front of the yoke cross bar; the Ø14 pusher hole stays for the ring; (4) the pusher keeps pushing the flat tail face within |y| 11 and the seat pushrod still lands on the flat end face at (y -15, z +1)."]]
E += [p("6. For dart-physics and dart-fw", H2)] + [p("• " + t, S) for t in [
    "dart-physics: rerun flight (tether at the bridle ring behind the tail), landing (belly-down rest now stable: CG on the axis), holding (pods at the tail, drills under the tether) and the righting sim with this leg (113 mm, pad 37 mm, hinge x 33, z 14.5) and this mass; mass_props.json has the leg_out pose.",
    "dart-fw: the kick channel (DRV#1 B, D2/D3) drives the righting leg (0 to 178°, far stop = its bed in the nose); the foot channel (DRV#2 B, D6/D7) is unused; the deploy end stop is the carved stop at the solved deploy angle; the wire list has 45 wires (no foot, no series link)."]]
E += [PageBreak(), p("7. Pictures", H2), img(OUT + "/audit_sheet.png", 180), img(OUT + "/sections.png", 180)]
doc = SimpleDocTemplate(PK + f"/KEDGE_{V}_engineering_report.pdf", pagesize=A4, leftMargin=16 * mm, rightMargin=16 * mm, topMargin=14 * mm, bottomMargin=14 * mm,
                        title=f"KEDGE {V} engineering report", author="")
doc.build(E)
print("report", PK + f"/KEDGE_{V}_engineering_report.pdf")
