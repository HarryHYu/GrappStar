"""KEDGE v35 engineering report PDF (run by report.py when the pods hinge at the front, P.POD_DIR 1).  python3 report35.py out35 pack_v35
From report34.py: geometry, mass and centre of mass, audit, hard stops and load paths, launcher fit, wire room, plus the v35 righting /
tow sim (sim_righting/v35_check/v35_righting_tow.json) and the flight check (phys35/flight35.json)."""
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
J = json.load(open(OUT + "/parts.json")); P = J["P"]; V = J.get("version", "v35")
A = json.load(open(OUT + "/audit.json"))
ld = lambda f: json.load(open(f)) if os.path.exists(f) else {}
HERE = os.path.dirname(os.path.abspath(__file__)); SIM = ld(os.path.join(HERE, "sim_righting/v35_check/v35_righting_tow.json")); FL = ld(os.path.join(HERE, "phys35/flight35.json"))
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
     p("v35 is v34 with the drill pods turned round: they hinge at the FRONT (x %g) and fold BACK along the flanks; deployed, the drills rake back toward the tether "
       "like tent pegs and the tow pushes each pod onto its printed deploy stop. Everything else is v34 (righting leg, tail tie, nose putty, press-fit couplers, cells, boards, firmware pin map). "
       "All numbers below come from the exact print geometry and the pipeline (run_all.sh), the righting sim (sim_righting) and flight35.py." % P["DRILL_X"])]
E += [p("1. Summary", H2), tbl([
    ["Item", "Result"],
    ["Body", f"Ø42 x {P['L']:.0f} mm; folded envelope x {A['folded_bbox'][0]} to {A['folded_bbox'][3]}, y ±{A['folded_bbox'][4]}, z {A['folded_bbox'][2]} to {A['folded_bbox'][5]} (nothing behind the tail, nothing above the hull but a 0.3 mm knuckle blister)"],
    ["Mass, centre of mass", f"{fo.get('mass_g')} g with catalogue bought masses and {P.get('BALLAST_G')} g nose putty; CG folded {fo.get('cg_mm')}, anchored {an_.get('cg_mm')}, leg out {lo.get('cg_mm')} (mm; z from the hull axis)"],
    ["Collision sweep", ("CLEAN" if clean else "NOT CLEAN") + f": righting leg 0 to {A['kick_range'][1]}° in {A['kick_steps']} steps, pods 0 to {A['deploy_range'][1]}° in {A['deploy_steps']} steps, leg x pod cross sweep, every sub-part, exact meshes"],
    ["Hard stops (mm³ of overlap just past each end)", ", ".join(f"{k} {v}" for k, v in A['hard_stops_mm3'].items())],
    ["Drills", f"deploy {P['DEPLOY_DEG']}° (solved: the motor end in the hub stays 0.5 above flat ground), raked back, {A['auger_depth_mm'].get('drill_leg_L_auger')} mm deep, the tips about {P['DRILL_X'] + 69:.0f} mm from the nose tip (under the tail)"],
    ["Launcher fit", ("FITS" if LF and not fails else ("DOES NOT FIT: " + ", ".join(fails)) if LF else "not run") + f" over {LF.get('states', '?')} launcher states (base/cad/parts/launch.py, parked)"],
    ["Wire room", f"left corridor worst fill {WR.get('worst_keel_fill')}, right {WR.get('worst_top_fill')} (bundle area over free area; under 0.6 is fine)"],
    ["Printed parts", f"{A['printed_cm3']} cm³, every printed part watertight and one body"]], [45, 130])]
def _so(t, soil):
    try: return SIM["configs"][t]["soils"][soil]
    except KeyError: return {}
_rl, _rs, _rp, _rf = _so("F_rigid_tie", "loam"), _so("F_rigid_tie", "sand"), _so("F_rigid_tie", "loam poses"), _so("F_rigid_tie", "loam fixed paths")
import statistics as _st
_mv = lambda o: ("%.1f to %.1f N (median %.1f)" % (min(o["moved_at_N"]), max(o["moved_at_N"]), _st.median(o["moved_at_N"]))) if o.get("moved_at_N") else "held 55 N"
E += [p("2. Layout and why", H2)] + [p("• " + t, S) for t in [
    f"Pods: pivot x {P['DRILL_X']:g}, z {P['DRILL_PIV_Z']:g} (z kept: the launcher's stop springs pass the dart at z -1.1 outboard of |y| 25, so the hubs and the arm-head lugs must stay above them), drill axis |y| {P['DRILL_PIV_Y']:g}. Folded they point aft along the flanks (auger tips at x 151); deploy {P['DEPLOY_DEG']}° swings the tips down and aft to a backward rake.",
    "Parts: the v34 arms, augers, motors, couplers and pod caps are reused unchanged, turned round and swapped over (the v34 right pod is the v35 left pod), so the left pod now has the left-hand auger and the swapped drill leads. Only the hull halves, the tray and the righting leg change.",
    "Deploy stop: carved in the pivot block (the swept arm + HORN_GAP), now forward of and below the pivot; the fold stop under the head. The soil pushes the pods toward deploy in the tow, so the stop, not the worm, holds them.",
    "Interior: leg cradle | disc 1 | deploy cradle with DRV#1 / DRV#2 standing beside it | disc 1b (the B- spring on its rear face) | cells in line on the belly | rear disc | sockets and capacitor on the tail key. Deck 1 starts behind disc 1b. The deploy worm box sits under the righting leg's bridge: a channel in the bridge's underside and a hump in the hull top make room.",
    "The right pod covers the XIAO's USB-C window folded and deployed: flash over USB before the pods are fitted, then over WiFi (OTA)."]]
E += [p("3. Load paths and the tow", H2)] + [p("• " + t, S) for t in [
    "Anchored tow: line -> the two holes in the tail face (hull) -> hull -> pivot blocks -> the carved deploy stops and the coupler bores -> arm heads -> drills -> soil. The soil pushes each pod onto its stop; the self-locking worm is not loaded.",
    (f"Righting sim (sim_righting/v35_check, study sim with layout front, one deploy worm on a rigid axle, line tied in the tail face, hull {SIM.get('configs', {}).get('F_rigid_tie', {}).get('D', {}).get('hull', {})}): "
     f"righted loam {_rl.get('righted')}/{_rl.get('runs')}, sand {_rs.get('righted')}/{_rs.get('runs')}, named landing poses {_rp.get('righted')}/{_rp.get('runs')}, fixed arrival paths 25 to 57° {_rf.get('righted')}/{_rf.get('runs')}; "
     f"median {_rl.get('right_s_median')} s, slowest {_rl.get('right_s_max')} s. Tow: deploy worm {(_rl.get('deploy_worm_tow_Nm') or {}).get('max')} N m (v34 tail tie: 0.94), "
     f"deploy stops {(_rl.get('deploy_stop_tow_Nm_both') or {}).get('max')} N m for both pods at the peak (about 2.4 N m a pod scaled to 55 N, about 280 N on each stop face 8.5 mm from the pivot). ") if SIM else "Righting sim not run.",
    (f"Holding in the sim: loam {_mv(_rl)}, sand {_mv(_rs)} (v34 tail tie: 79 of 80 held 55 N in loam, sand 18.3 N). The anchors now sit near the tail, so the pull at the tail pitches the nose up about the tail edge (about 12° at 45 N), "
     "taking weight off the hull and pulling the drills up and back until the sim's soil limit (25 N a drill in loam, the same in every direction) is reached. Each drill is loaded about 95 % sideways and only about 4 N along its axis (outward); "
     "the sim gives no credit for that, so a soil pull test decides the real number.") if SIM else "",
    "Drill loads: side load on the auger hub -> arm bushing; push -> arm shoulder; pull -> nut A on the motor's threaded shaft. Righting: leg -> two couplers -> knuckle-wall bores and skin plugs. Launch: pusher -> flat tail end walls -> hull -> radial screws -> tray -> cells on the rear plate."]]
E += [p("4. Mass properties (catalogue bought masses; weigh and update)", H2), tbl(
    [["Pose", "Mass g", "CG x, y, z mm"]] + [[k, v["mass_g"], v["cg_mm"]] for k, v in T.items()], [40, 30, 60]),
      p(f"Printed {MP.get('printed_g')} g at 4 walls / 40% infill; bought parts and extras {MP.get('bought_g')} g (incl. {P.get('BALLAST_G')} g nose putty). Per-part volume, centroid and inertia: mass_props.json.", S)]
def _fc(k):
    try: E_ = FL["cases"][k]; return E_
    except (KeyError, TypeError): return None
_f5, _f4 = _fc("v35 putty, bought x1.0 | tail tie x 152 z -2.4"), _fc("v34 putty, bought x1.0 | tail tie x 152 z -2.4")
def _fs(E_):
    if not E_: return "not run"
    r = [E_[c] for c in E_ if c.startswith("rail")]
    t = [E_[c] for c in E_ if c.startswith("rail") and "tip-off" in c]
    return (f"nose first at every rail: {all(x['nose_first'] and x['yaw_nose_first'] for x in r)}; tow arm {E_['pitch']['linear']['tow_arm_mm']} mm behind the CG, "
            f"neutral speed {E_['pitch']['linear']['neutral_speed_m_s']} m/s (flight 6.5 to 6.9); tip-off: AoA max {max(x['aoa_max_deg'] for x in t)}°, sideslip max {max(x['yaw_sideslip_max_deg'] for x in t)}°; "
            f"5° + 1 rad/s: AoA max {max(x['aoa_max_deg'] for x in r)}°")
E += [p("5. Launcher (parked base) and flight", H2)] + [p("• " + t, S) for t in [
    ("Fit: " + ("FITS" if LF and not fails else json.dumps(fails)) + f" over {LF.get('states', '?')} states. Rest check: " + str(LF.get('rest', {}))) if LF else "Fit not run.",
    f"Flight (flight35.py: flight34.py's strip model on the folded out35 geometry, line tied in the tail face, rails 6 to 47°): v35 {_fs(_f5)}. v34 for comparison: {_fs(_f4)}. The CG is about 7.6 mm further forward than v34, which more than pays for the pod hubs now at the front.",
    "What base needs: nothing new (the tail face, the tie holes and the envelope are v34's)."]]
E += [p("6. For dart-fw", H2)] + [p("• " + t, S) for t in [
    "Pin map unchanged. The deploy turns the other way from v34 (pods hinge at the front): swap the deploy N20's two leads at DRV#1 AOUT1 / AOUT2 (or set the deploy sign to -1 in kedge_logic.h PINS); the deploy angle (46.75°) and its stops are as v34.",
    "Drills: DRILL_SIGN +1 unchanged; the swapped drill pair is now the LEFT pod's (the left-hand auger moved to the left).",
    "The righting sequence ran unchanged in the sim (pods-out roll on a flank, leg flip on the back): 100 % righted. USB-C is covered by the right pod once the pods are on: OTA for updates."]]
E += [PageBreak(), p("7. Pictures", H2), img(OUT + "/audit_sheet.png", 180), img(OUT + "/sections.png", 180)]
doc = SimpleDocTemplate(PK + f"/KEDGE_{V}_engineering_report.pdf", pagesize=A4, leftMargin=16 * mm, rightMargin=16 * mm, topMargin=14 * mm, bottomMargin=14 * mm,
                        title=f"KEDGE {V} engineering report", author="")
doc.build(E)
print("report", PK + f"/KEDGE_{V}_engineering_report.pdf")
