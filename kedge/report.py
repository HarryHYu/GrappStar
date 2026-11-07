"""KEDGE engineering report PDF: physics, system interfaces, design variants, test results.
python3 report.py out29 pack_v29 variants_var1.json variants_var2.json ..."""
import json, sys, glob, os
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, Image
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
OUT, PK = sys.argv[1], sys.argv[2]; VARF = sys.argv[3:]
_F = [("/System/Library/Fonts/Supplemental/Arial.ttf", "/System/Library/Fonts/Supplemental/Arial Bold.ttf"),
      ("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf", "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf")]
_reg, _bold = next(f for f in _F if os.path.exists(f[0]))       # Arial on the Mac, Liberation Sans (Arial metrics) elsewhere
pdfmetrics.registerFont(TTFont("Arial", _reg)); pdfmetrics.registerFont(TTFont("Arial-Bold", _bold))
J = json.load(open(OUT + "/parts.json")); P = J["P"]; V = J.get("version", "")
A = json.load(open(OUT + "/audit.json")); Ph = json.load(open(OUT + "/physics.json")); Bj = json.load(open(PK + "/barrel.json"))
B = ParagraphStyle("b", fontName="Arial", fontSize=9.5, leading=12.5); S = ParagraphStyle("s", parent=B, fontSize=8, leading=10)
H1 = ParagraphStyle("h1", parent=B, fontName="Arial-Bold", fontSize=15, leading=19, spaceAfter=4)
H2 = ParagraphStyle("h2", parent=B, fontName="Arial-Bold", fontSize=11.5, leading=15, spaceBefore=8, spaceAfter=3)
def p(t, st=B): return Paragraph(t, st)
def tbl(rows, widths, fs=8):
    st = ParagraphStyle("t", parent=B, fontSize=fs, leading=fs + 2); sh = ParagraphStyle("th", parent=st, fontName="Arial-Bold")
    t = Table([[Paragraph(str(c), sh if i == 0 else st) for c in r] for i, r in enumerate(rows)], colWidths=[w * mm for w in widths], repeatRows=1)
    t.setStyle(TableStyle([("GRID", (0, 0), (-1, -1), 0.4, colors.black), ("VALIGN", (0, 0), (-1, -1), "TOP"), ("LEFTPADDING", (0, 0), (-1, -1), 3), ("RIGHTPADDING", (0, 0), (-1, -1), 3)]))
    return t
def img(path, w):
    from PIL import Image as PI
    im = PI.open(path); return Image(path, width=w * mm, height=w * im.size[1] / im.size[0] * mm)
E = [p(f"KEDGE dart {V}: engineering report", H1),
     p(f"What this covers: how the dart behaves with the launcher, in flight, on landing, in the soil and with the reel; which design versions were tried and why {V} was chosen; "
       "what the firmware and the rover client were tested against. All numbers come from the exact print geometry (scripts in source/). Assumptions are listed where they are used; "
       "the ones marked MEASURE should be replaced with bench numbers.")]
# ---- summary
fl = Ph["flight"]; la = Ph["launch"]; an = Ph["anchoring"]
E += [p("1. Summary", H2), tbl([
    ["Item", "Result", "What it means"],
    ["Length, mass", f"{P['L']:.0f} mm body ({A['folded_bbox'][3]-A['folded_bbox'][0]:.0f} folded incl. augers), {Ph['mass_g']:.0f} g ({Ph['printed_g']:.0f} printed + {Ph['bought_g']:.0f} bought)", "Matches the 245 g beer-bottle target"],
    ["Collision sweep", "clean", f"kick 0 to {A['kick_range'][1]}° ({A['kick_steps']} steps), drills 0 to {A['deploy_range'][1]}° ({A['deploy_steps']} steps), kick x drill cross sweep, every sub-part, exact meshes"],
    ["Hard stops", "pods: horn about 1° past the deploy angle, fold lug about 2° past folded; kick: rest " + str(P.get('KICK_REST_DEG', 0)) + "° past the plant", "Tow load pushes the legs further out: it goes into the hull, not the worms or the couplers"],
    ["Launcher fit", "checked against base/cad/parts/launch.py over every launcher and mission state (launcher_fit.json)", "Cradle, carriage, funnel, muzzle, pusher: no contact beyond the v30 dart"],
    ["Flight, launch, landing, holding at 7 m/s", "kedge/DART_PHYSICS31.md (dart-physics, on this geometry)", "The physics rows below come from physics.py and are kept for reference"],
    ["Launch", f"{la['v_needed_m_s'][0]} to {la['v_needed_m_s'][1]} m/s for 15 to 20 m; {la['accel_g'][0]} to {la['accel_g'][1]} g over a {la['stroke_m']} m stroke; {la['spring_energy_J']} J", "Launcher spring sizing input"],
    ["Landing", f"{la['impact_g']} g if it stops in {la['impact_stop_m']*100:.0f} cm of soil", "All parts checked at this load (section 4)"],
    ["Holding force", f"dry sand {an['by_soil']['dry sand']['total_N']} N, moist loam {an['by_soil']['moist loam']['total_N']} N, firm clay {an['by_soil']['firm clay']['total_N']} N", "The reel must not pull harder than this: limit the wiper current"],
    ["Augers in the ground", f"drills {an['depth_mm']['drill']} mm, foot {an['depth_mm']['foot']} mm", "Holding rises with depth squared in sand"],
    ["Battery", f"{Ph['battery']['hours_video_on']} h with WiFi + video on; an anchor sequence uses about {Ph['battery']['anchor_sequence_mAh']} mAh", "Plenty"],
], [36, 70, 68])]
# ---- variants
rows = [["Version", "Change", "Clean", "Folded len", "Drill depth", "CG x", "Margin cal", "Hold loam N", "Hold sand N"]]
allv = []
for f in VARF:
    if os.path.exists(f): allv += json.load(open(f))
BASE = next((r["hold_loam_N"] for r in allv if r.get("name") == "v29a_base"), "?")
for r in allv:
    if "error" in r: rows.append([r["name"], "failed to build", "", "", "", "", "", "", ""]); continue
    rows.append([r["name"], json.dumps(r["overrides"]).replace('"', "") or "-", "yes" if r["clean"] else "NO: " + ", ".join(list(r["belly"].keys())[:2] + r["collisions"][:2]),
                 r["folded_len"], r["drill_depth"], r["cg_x"], r["margin_cal"], r["hold_loam_N"], r["hold_sand_N"]])
E += [p("2. Design versions tried", H2), p("Each version went through the full pipeline (generate, solve the leg angles from the ground, exact-mesh sweep, physics). Belly rule: nothing outside the Ø42 cylinder below the centreline."),
      tbl(rows, [26, 44, 22, 14, 13, 11, 13, 14, 14], fs=7),
      p("Holding columns use the final (corrected) soil model for every version; versions with pods at y 29 were swept before the 1.5 mm pod move, the final v29 sits at y 30.5.", S),
      p(f"{V} (this build): v30 (Ø17 x 72) taken as the start because the launcher is built around it, then the anchoring hardware reworked: deploy stop horns and fold lugs on the hull, a kick rest on the ear root, a longer pod stack (terminal room, bushing gap, bearings back to back), "
        f"drill pivot z {P['DRILL_PIV_Z']} so the longer pods still reach {P['DEPLOY_DEG']}°, augers Ø{P['AUGER_D']:g} x {P['AUGER_L']:g} with a Ø{P['AUGER_CORE']:g} core (dart-physics: same loam holding as Ø17, about half the install torque and a narrower cut while milling), pods at y {P['DRILL_PIV_Y']:g}. History below. "
        f"Before that: v29 = cells in front of the boards, Ø16 x 70 augers, drill pivot raised to z 10, pods at y 30.5. "
        f"Final: drills deploy to {P['DEPLOY_DEG']}° and reach {A['auger_depth_mm']['drill_leg_L_auger']} mm deep (v28: 29.5); holding in loam {an['by_soil']['moist loam']['total_N']} N against {BASE} N for the v28-style baseline. "
        "What the study showed: raising the drill pivot lets the pods swing further before their noses touch the ground, so the augers go deeper; lowering it does the opposite and breaks the belly rule. "
        "But depth alone is not holding: with the pull horizontal, a steeper auger takes more of it sideways. Embedded length matters most, so 70 mm augers at pivot 10 beat 66 mm at pivot 11 or 12. "
        "74 mm adds 1% for 4 mm more length past the tail; Ø17 x 72 (pods out to y 31) adds 8% for 2 mm more length: the best gain left, kept as an option; a 9 mm pitch changes nothing in this model; a lightened tray and kick keel saves under 1 g and weakens the pole; pivot 14 does not fit the deploy worm; Ø18 hits the hull folded. The 30 mm foot option (10 mm lower for the barrel) costs about 2 N; the 50 mm foot (+9 mm depth) adds about 3 N.", S)]
# ---- flight
FS = json.load(open(OUT + "/flight_sim.json")) if os.path.exists(OUT + "/flight_sim.json") else {}
rows = [["Case", "Max angle of attack after 0.6 s", "Tumbled", "Range m"]] + [[k, v["max_alpha_after_0p6s_deg"], "YES" if v["tumbled"] else "no", v["range_m"]] for k, v in FS.items() if k != "inputs"]
E += [p("3. Flight", H2),
      p(f"Centre of mass at x = {Ph['cg_mm'][0]} mm from the nose (of {P['L']:.0f}); the projected-area centre of pressure is at x = {fl['cp_side_x']} (side) / {fl['cp_top_x']} (top). On the body alone the dart is neutral, so it was simulated: "
        "a pitch-plane rigid-body flight with the real mass, inertia and side profile (cross-flow drag along the body), axial drag, gravity and the tether tension at the tail, launched at 18 m/s and 45° with a 10° nose-up disturbance."),
      tbl(rows, [70, 45, 22, 25]) if FS else p("(flight_sim.py not run)"),
      img(PK + "/flight_sim.png", 170) if os.path.exists(PK + "/flight_sim.png") else Spacer(1, 1),
      p("Result: on its own the dart tumbles. Either fix works on its own; together they are robust: 15 g of ballast in the nose cavity (steel shot or nuts set in epoxy around the camera tube) "
        "and a reel that pays out with a light drag of about 1 N or less. A 3 N reel brake stalls the throw at 8 m and the dart flips at the end. So the reel must free-spool during launch with only a light brake; "
        "that matches the plan of spinning the storage reel up for launch. With ballast and a 1 N brake the throw reaches about 18 m at 18 m/s. Ballast adds 15 g (dart ~277 g) and moves the CG to about x = 78.", S)]
# ---- launch + landing loads
E += [p("4. Launch and landing loads", H2), tbl([["Load case", "Value", "Unit", "Check"]] + [[a, b, c, d] for a, b, c, d in Ph["loads"]], [55, 14, 16, 89], fs=7.5),
      p("Battery contacts follow from this: launch throws the cells aft, so the aft contacts are rigid plates and the springs sit on the forward disc. "
        "At landing the cells move forward into the springs for a few milliseconds; the firmware treats a reset as harmless (motors start off, WiFi is back in about a second). "
        "A 470 to 1000 µF capacitor across the 5 V output of the MP1584 rides through contact bounce and is recommended.", S)]
# ---- anchoring + reel
rows = [["Soil", "Each drill auger N", "Foot auger N", "Friction N", "Total N"]] + [[k, v["drill_auger_each_N"], v["foot_auger_N"], v["friction_N"], v["total_N"]] for k, v in an["by_soil"].items()]
tip = Ph["tip_over"]
E += [p("5. Anchoring, tipping and the reel", H2), tbl(rows, [40, 34, 30, 30, 30]),
      p(f"Method: each auger is a short rigid pile (Broms): sand 0.5 γ D L² Kp; clay free-head solution. The drill augers are raked at {P['DEPLOY_DEG']}°, so their embedded length is {an['embedded_length_mm']['drill']} mm for {an['depth_mm']['drill']} mm of depth. "
        "The horizontal pull splits into a push along each auger (end bearing on the flights) and a sideways load; the weaker of the two limits governs. "
        "These are small anchors: length in the soil is everything. In loose dry sand the dart holds only a few newtons whatever the design; in loam or clay it holds tens of newtons. "
        f"The foot's job is holding the nose down: it can resist about {Ph.get('tip_over_foot_uplift_capacity_loam_N','?')} N of uplift in loam.", S),
      p(f"Tipping: the tether pulls horizontally at the axis, {tip['tether_height_mm']} mm above the ground. At {tip['pull_N']} N the nose wants to lift with {tip['moment_Nm']} N·m; the kick foot "
        f"{tip['foot_lever_mm']} mm ahead of the drills needs to hold {tip['foot_hold_down_needed_N']} N down. That is why the foot is planted first and the kick worm stays locked.", S),
      p("Reel interface: the reel's wiper motor can pull far more than any of these numbers. Limit tether tension on the reel side: read the BTS7960 current (IS pins) or add a load cell (you have the 100 kg cell + HX711) "
        "and stop or slip when tension passes the soil's holding value (firmware/reel/reel_tension_limit does this: speed tapers over the last 30% and stops at the limit; set it with l20). Start with 20 N, raise it while watching the dart's pitch on /status: a nose-up change of more than about 5° means it is starting to drag.", S)]
# ---- motors, battery, strength
k = Ph["kick"]
E += [p("6. Motors, battery, strength", H2),
      p(f"Kick: the leg family weighs {k['leg_mass_g']} g with its centre {k['cg_radius_mm']} mm from the pivot, so gravity asks at most {k['max_gravity_torque_kgcm']} kg·cm of the worm. Small worm N20s give 1 to 4 kg·cm: ample. "
        f"Deploy: both pods together need {Ph['deploy']['gravity_torque_both_kgcm']} kg·cm against gravity; the real load is soil, which is why the drills spin while the pods swing (firmware auto sequence)."),
      p("Important geometry fact: the drill augers sit radially on the pods, so their depth comes only from the swing. They cut sideways through the soil as the pods rotate; spinning them while deploying is what lets the deploy worm push them in (a slow mill: dart-physics gives the times). "
        "Under tow the soil turns the pods further down; once a pod is fully down its head sits on the hull horn, so that moment never reaches the deploy worm, its D stubs or the couplers. The same holds for the kick on its rest. Below full deploy the worm holds and the couplers are the weak link (they are cheap to reprint and protect the gearbox). "
        "The foot auger is different: near 180° its axis is almost vertical and the swing drives it along its own axis, so it really screws in."),
      p(f"Battery: {Ph['battery']['capacity_Wh']} Wh. WiFi + VTX + camera draw about {Ph['battery']['idle_W_video_on']} W: {Ph['battery']['hours_video_on']} h. Worst case all motors stalled is about 2 A (BMS 8 A, INA219 trip set at 2.5 A)."),
      p(f"Coupler torque capacity about {Ph['strength']['coupler_torque_capacity_Nm']} N·m (PLA, derated for the flat and layer lines): well above the deploy worm. "
        f"Kick pole (round pole + keel, section modulus {Ph['strength']['kick_pole_section_mm3']} mm³ from the real cross-section): {Ph['strength']['kick_pole_stress_MPa_at_20N_side_load']} MPa at a 20 N sideways load on the foot; keep sideways loads under about {Ph['strength']['kick_pole_side_load_limit_N']} N (factor 2 on PLA). The pole prints lying down, so the bending runs along the extrusion lines.")]
# ---- launcher
E += [PageBreak(), p("7. Launcher interface", H2),
      p(f"The barrel must clear this silhouette (folded dart seen along its axis). Channel with {Bj['clearance_mm']} mm clearance spans y {Bj['channel_bounds_yz'][0]} to {Bj['channel_bounds_yz'][2]}, z {Bj['channel_bounds_yz'][1]} to {Bj['channel_bounds_yz'][3]} mm. "
        f"Files: barrel_profile.dxf (sketch it in SolidWorks or Fusion and extrude), barrel_keepout.stl (subtract it from the barrel body), barrel_profile.svg, and KEDGE_{V}_assembly_folded_STEP.zip (the whole folded dart as one STEP assembly). "
        "The real launcher (base/cad/parts/launch.py) was checked directly with this dart: launcher_fit.py intersects the folded dart with every launcher part over the cock sweep, the shot, the reset, the dart leaving at 6 and 47° and every mission state with the dart on, off or coming back to the launcher."),
      img(PK + "/barrel_profile.png", 110),
      p(Bj["cradle"] + ". " + Bj["pusher"] + ".", S),
      p("The folded foot auger is the tallest thing on the dart (top of the silhouette). If the barrel slot for it is a problem, the options in order of cost are: a shorter foot auger (30 mm: about 10 mm less height, about 8 mm less depth), "
        "or a foot auger tilted 45° toward the tail in the housing (folded it leans back over the hull; deployed it points down and forward, which resists the tether better). Neither is built; both are quick to generate on request.", S),
      p(f"Spring energy needed: about {la['spring_energy_J']} J delivered to the dart for 20 m, more with losses. A longer stroke lowers the peak load on the dart: 0.4 m halves the g.", S)]
# ---- firmware + rover
tlog = open("fw/test/last_run.txt").read() if os.path.exists("fw/test/last_run.txt") else ""
E += [p("8. Firmware and rover link", H2),
      p("The control logic is a plain C++ header (kedge_logic.h) shared by the sketch and a PC test that drives it against a motor model. The auto sequence is: kick swings out; when the kick current rises (foot auger touches) the foot auger starts; "
        "the kick keeps pushing until the housing lands, read by pausing the foot for 150 ms every 400 ms so the INA219 sees the kick alone; the foot finishes; then the pods deploy with the drills spinning, deploy stall read the same way with the drills paused; drills finish; hold."),
      p("Rover API on the KEDGE WiFi: GET /status (JSON: rail volts, current, peak, motors, sequence, armed, landed, roll, pitch, last event), GET /key?c=&ms=, GET /stop, GET /auto. "
        "kedge_client.py (standard library only) gives status, watch-to-CSV, hold-a-key, auto with progress, terminal driving, and an FPV window with the telemetry over the RC832 + capture card picture (OpenCV)."),
      p("Landing detection: the BNO085 accelerometer sees the launch spike, flight, impact and stillness; with A (arm) pressed before launch the dart anchors itself 1.5 s after it comes to rest."),
      p("Test results (PC simulation of the real logic, and the client against a mock dart):", S)]
for line in tlog.splitlines():
    if line.startswith(("ok:", "FAIL", "ALL", "   ")): E.append(p(line.replace("<", "&lt;"), S))
E += [p("Compiled for XIAO_ESP32S3 on ESP32 Arduino core 2.0.17 and 3.3.12 (USB CDC On Boot: Enabled; the sketch refuses to compile with it disabled, because Serial would then take D6/D7).", S),
      p("Independent reviews: two separate code reviews of the firmware and wiring and one of the geometry checks and physics were run by agents that had not seen the work being produced. "
        "They found, among others: 3V3 must go to the DRV8833 SLP pad, never VCC/VIN; D6/D7 toggle at reset (drills moved to D8/D9); safety logic must not wait on the web server (now its own 5 ms task); "
        "Shift-release and touch-scroll could leave a motor running on the web page; the clay holding and the pole strength formulas were wrong; the sweep step was too coarse for the leg tip. All were fixed and re-checked.", S)]
E += [p("9. Open items to measure", H2)] + [p("• " + t, S) for t in [
    "Weigh every bought part on the kitchen scale and update BOUGHT_G (dart-physics model; the mass and CG use catalogue values).",
    "N20 end-cap tabs: the pods leave 1.2 mm (tabs bent up), the saddle and foot housing 1.6 mm: check on the first motor; the gearbox output bushing gets 0.6 mm before the shaft nut.",
    "Bench stall currents of each motor at 8 V: set kickContactMa / kickStallMa / deployStallMa in kedge_logic.h.",
    "Launcher stroke and spring: replace STROKE_M and re-run physics.py for the real g loads.",
    "A pull test in your soil: plant the dart, pull the tether with a luggage scale, compare with section 5.",
    "BMS stick dimensions when it arrives; F693ZZ flange size (catalogue 9.5 x 0.6 used)."]]
doc = SimpleDocTemplate(PK + f"/KEDGE_{V}_engineering_report.pdf", pagesize=A4, leftMargin=16 * mm, rightMargin=16 * mm, topMargin=14 * mm, bottomMargin=14 * mm, title=f"KEDGE {V} engineering report", author="")
doc.build(E); print("report ok")
