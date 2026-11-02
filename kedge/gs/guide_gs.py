"""Ground station build guide PDF (plain black Arial). python guide_gs.py out ../pack_gs"""
import json, sys, os
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, PageBreak, KeepTogether
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors

OUT, PK = sys.argv[1], sys.argv[2]
pdfmetrics.registerFont(TTFont("Arial", "/System/Library/Fonts/Supplemental/Arial.ttf"))
pdfmetrics.registerFont(TTFont("Arial-Bold", "/System/Library/Fonts/Supplemental/Arial Bold.ttf"))
J = json.load(open(f"{OUT}/gs_parts.json")); P = J["P"]
PH = json.load(open(f"{OUT}/physics_gs.json"))
AU = json.load(open(f"{OUT}/audit_gs.json"))
PR = json.load(open(f"{PK}/print_report.json"))
PL = json.load(open(f"{PK}/plates.json"))

B = ParagraphStyle("b", fontName="Arial", fontSize=9.5, leading=12.5)
H1 = ParagraphStyle("h1", fontName="Arial-Bold", fontSize=15, leading=19, spaceAfter=6)
H2 = ParagraphStyle("h2", fontName="Arial-Bold", fontSize=11.5, leading=15, spaceBefore=8, spaceAfter=4)
SM = ParagraphStyle("sm", fontName="Arial", fontSize=8, leading=10)


def p(t, s=B): return Paragraph(t, s)


def tbl(rows, widths, head=True, fs=8):
    t = Table([[Paragraph(str(c), ParagraphStyle("c", fontName="Arial-Bold" if (head and i == 0) else "Arial", fontSize=fs, leading=fs + 2)) for c in r] for i, r in enumerate(rows)],
              colWidths=[w * mm for w in widths], repeatRows=1 if head else 0)
    t.setStyle(TableStyle([("GRID", (0, 0), (-1, -1), 0.4, colors.black), ("VALIGN", (0, 0), (-1, -1), "TOP"),
                           ("LEFTPADDING", (0, 0), (-1, -1), 3), ("RIGHTPADDING", (0, 0), (-1, -1), 3)]))
    return t


def img(path, w=170):
    from PIL import Image as PI
    im = PI.open(path); r = im.height / im.width
    return Image(path, width=w * mm, height=w * r * mm)


S = []
S.append(p("GrappStar ground station: launcher and reel on one wiper motor", H1))
S.append(p("Build guide, prototype scale, sized to the KEDGE v30 dart and the parts in hand. Every number here comes out of the generator "
           "(gs/gs_gen.py), the collision audit (gs/audit_gs.py) and the physics model (gs/physics_gs.py)."))
S.append(Spacer(1, 4)); S.append(img(f"{PK}/renders/01_overview.png", 165))

S.append(p("Words used here: sear = the spring-loaded hook that holds the cocked carriage until the cord pulls it down. phi = the program gear angle in degrees. D flat = a flat ground along a shaft end so a part cannot spin on it. Spot flat = a small flat ground only where a grub screw presses. Drawn cup = a thin-shell one-way clutch that needs a rigid steel sleeve round it. h6 / N6 = machining fit grades: ask for them by name. Keel = the fin under the carriage that the sear hook catches. Double shear = the pivot screw is held on both sides."))
S.append(p("1. How it works", H2))
S.append(p("One 12 V car wiper motor turns one 12 mm main shaft. The direction picks the job. "
           "<b>Forward</b>: only the reel moves. The reel rotor is geared straight to the main shaft (58:39). The launcher drive pinion sits on one-way "
           "bearing A (HF1216) and freewheels; backstop C (HF1216 on the program gear stub) holds the launcher still. "
           "<b>Backward</b>: bearing A locks and the program gear (m3, 30 teeth, 18 present) turns. One turn of it is one complete launcher cycle: "
           "its teeth draw the carriage back 208 mm through a rack, the spring-loaded sear snaps up in front of the carriage keel, then the lift cam beside the gear "
           "raises the rail 4 to 35 deg on a frame roller (slowly: the rail carries the program gear round the pinion like a planet carrier, so a steep lift would multiply the motor torque). Near the top a cord from the base, slack until then, pulls the sear down and the shot goes. "
           "The rail then drops back onto rubber feet. In the same backward run the reel rotor turns backward, which pulls its pickup pin in, so the line is free. "
           "<b>Stopped</b>: the wiper's worm is self locking, everything holds."))
S.append(p("The reel is a spincast type: the spool never turns. At launch the line peels off the front of the still spool, over the still rotor and out "
           "of the exit eye (end pull, about 0.8 N over the rounded rotor edges and the ceramic eye, no overrun, no bird's nest). To wind, the rotor turns forward, its drag disc pushes the pickup pin out, "
           "the pin catches the line and lays one turn on the spool per rotor turn. The reel rides on the rail, on the dart axis, so the line always "
           "points along the rail."))
rows = [["phi (program gear deg)", "what happens", "rail"],
        ["340 (-20)", "first tooth meets the rack", "4"],
        ["340 to %.1f" % P["PH_REL"], "carriage drawn back %.1f mm, springs %.0f to %.0f N; keel rides over the sear hook" % (P["STROKE"], PH["springs"]["F_rest_N"], PH["springs"]["F_full_N"]), "4"],
        ["%.1f to %.1f" % (P["PH_REL"], P["PH_LIFT0"]), "last tooth lets go, the carriage runs %.1f mm onto the sear hook (kept short: a gentle catch). COCKED: the motor stops at %.1f" % (P["S_SETTLED"] - P["P_D"], (P["PH_REL"] + P["PH_LIFT0"]) / 2), "4"],
        ["%.1f to %.1f" % (P["PH_LIFT0"], P["PH_LIFT1"]), "lift cam raises the rail (blended constant speed)", "4 to 35"],
        ["%.1f to %.1f" % (PH["sear_trip"]["phi_taut"], P["PH_TRIP"]), "trip cord goes taut at rail %.1f deg and pulls the sear down: the shot at %.1f" % (next(t for t, d in P["SEAR_CORD_TAB"] if d < -0.1), P["PH_TRIP"]), "%.1f" % P["TH_RELEASE"]],
        ["%.1f to %.1f" % (P["PH_DROP0"], P["PH_DROP1"]), "rail free-falls onto the bumpers (the program gear overruns); the fire run stops as SW_LOW closes, phi re-zeroed to 333", "35 to 4"]]
S.append(tbl(rows, [32, 118, 20]))

S.append(p("2. Numbers", H2))
L = PH["launch"]; F = PH["flight"]; M = PH["motor"]
rows = [["item", "value"],
        ["springs (2, design point)", "%.2f N/mm total, %.0f N at rest, %.0f N at full draw, %.1f J stored" % (PH["springs"]["k_total_N_per_mm"], PH["springs"]["F_rest_N"], PH["springs"]["F_full_N"], PH["springs"]["energy_J"])],
        ["launch at %.0f deg (the trip angle)" % L["angle_deg"], "%.1f m/s, %.0f ms, peak %.0f g, %.0f %% of the spring energy into the dart" % (L["v_muzzle"], L["t_ms"], L["a_peak_g"], 100 * L["efficiency"])],
        ["flight (drag + end-pull tether)", "range %.1f m, %.2f s, %.1f m of line out (33 to 35 deg is the best band in the sweep)" % (F["range_m"], F["tof_s"], F["line_out_m"])],
        ["carriage end stop", "%.1f J into the urethane stops every shot" % L["carriage_impact_J"]],
        ["range by spring anchor hole", "; ".join("%s: %.1f m/s, %.1f m" % (h["hole"], h["v"], h["range_m"]) for h in PH["anchor_holes"])],
        ["motor, worst point of the cycle", "%.1f N m at the wiper shaft (cocking), %.0f rpm, %.1f A; one program turn %.1f s" % (M["T_motor_max_Nm"], M["rpm_at_peak"], M["I_at_peak_A"], M["cycle_s"])],
        ["tow", "%s" % "; ".join("%d N: %.2f m/s, %.1f A" % (r["T_line_N"], r["line_m_s"], r["I_A"]) for r in PH["reel"]["rows"][1:5])],
        ["line tension at motor stall", "%.0f N (firmware limits it; default 20 N)" % PH["reel"]["line_tension_at_stall_N"]],
        ["masses", "dart %.0f g, carriage + rack %.0f g, rail assembly %.0f g" % (PH["masses_g"]["dart"], PH["masses_g"]["sled_rack"], PH["masses_g"]["rail_assembly"])],
        ["wiper assumed", "45 rpm no load, 15 N m and 18 A at stall (measure yours: physics_gs.py WIPER)"],
        ["collision audit", "%s: every part, exact meshes, program gear 0 to 360 in 1 deg steps, with the dart, sear, carriage and reel kinematics" % ("CLEAN" if not AU["collisions"] else "NOT CLEAN")]]
S.append(tbl(rows, [48, 122]))

S.append(PageBreak())
S.append(p("3. Bought parts", H2))
rows = [["qty", "part", "where / note"],
        ["1", "12 V car wiper motor", "in hand. Its own crank arm bolts to the printed coupler through two slots; its mount bolts go in the 3 radial slots of the motor plate (bolt circle r 30 to 55)"],
        ["1", "BTS7960 43 A driver, ESP32-S3 DevKitC-1, AS5600 + 6 x 2.5 diametric magnet", "in hand. Add 10 k pull-downs from R_EN and L_EN to GND (no twitch at reset)"],
        ["1", "12 mm hardened linear rod (h6), 400 mm", "cut into the main shaft (240) and the program gear stub (154). D flats only at shaft ends, never under a clutch or bearing (step 1)"],
        ["1", "8 mm steel rod, 72 mm", "reel stub: D flats at both ends (rotor front, stub gear rear)"],
        ["4", "HF1216 one-way drawn cup (12 x 18 x 16)", "2 stacked as A in the drive pinion, 1 + 1 as backstop C in the two cheek bosses. All four go in the same way round: seen from the left (-y) side with the muzzle to your right, each locks when the shaft turns clockwise inside it. Cocking turns the program gear counter-clockwise (top teeth moving back). Check by hand before the Loctite sets"],
        ["3", "machined steel sleeve 18 ID (N6, for drawn cups) x 24 OD (one 32 long, two 16 long)", "press each HF1216 into a sleeve (drawn cups need a rigid housing), then bond the sleeve into the print with Loctite 648 (a thin tube cannot hold a drawn cup)"],
        ["6", "6001-2RS bearing (12 x 28 x 8)", "2 frame plates, 2 rail pivot, 2 program gear stub"],
        ["4", "F693ZZ (3 x 8 x 4 flanged)", "in hand: 2 in each pulley"],
        ["1 + 1 + 2", "608ZZ (8 x 22 x 7) + 8 x 22 steel pin + 2 M8 washers", "the 608 IS the cam roller (steel on the PPA-CF cam), in the right plate's post"],
        ["2", "12 mm rigid flange coupling, steel: flange 32 x 5, hub 22, 4 holes on a 24 circle, set screws (common CNC part). The cam one needs a hub of 11 mm or less: saw it from the free end only if its set screw centres are 8 mm or less from the flange, otherwise buy a short-hub coupling", "program gear and lift cam: the whole cam and trip torque goes through these, not through grub screws in plastic. Grind two flats at 90 deg under each set screw pair"],
        ["1 + 1", "shoulder screw 5 x 25, M4 x 8 thread, head 9 x 4 (ISO 7379 style, e.g. GANTER ISO 7379-5-25-M4) + M4 nut", "sear pivot, double shear: left cheek and the lug under the shelf; the nut is captive in the lug"],
        ["1 + 2", "extension spring OD 6, wire 0.7, 16 mm hook to hook, about 3 N initial tension, about 1 N/mm + 2 mm steel pins x 2", "lifts the sear hook onto its stop (tail down, hook up): about 8 N fitted, 65 N mm"],
        ["1 + 1", "3 x 20 grooved pin (or 3 mm rod, groove cut 2 mm from one end) + 3 mm e-clip", "trip cord pin, pressed into the boss on the sear outer plate; the e-clip keeps the cord on"],
        ["2", "main extension springs", "per spring: music wire 1.6 mm, OD 12.6 mm (mean 11), about 134 coils, free length about 238 mm, rate about 0.36 N/mm, initial tension about 15 N, must stretch to 470 mm. Body stress about 800 MPa at 96 N. ENDS: screw-in plug or swivel-insert ends, not hooks or plain loops (hook bending is near the wire limit). Push a closed-cell foam rod (about 9 mm) into each coil as a surge damper: the springs ring at about 35 Hz, close to the 33 ms stroke. Measure the rate before use"],
        ["0.4 m + 1 + 3 + 1", "2 mm Dyneema; M4 x 60 eye bolt, machine thread, fully threaded (50 mm or more of thread), 3 M4 nuts + washer; light die spring 5 x 10 x 40 (16 mm travel); 3 x 16 grooved pin + e-clip", "rail check strap: from the pin on the right cheek to the second sprung post, taut at rail 37.5 deg (set it with a printed angle gauge); the long spring absorbs about 1.5 J so the frame, not the trip cord, takes the recoil kick"],
        ["2", "3 mm steel pins 76 mm", "spring hook pins across the anchor bar: rear hole (design) and front hole (more preload)"],
        ["1 m", "2 mm Dyneema cord", "two spring cords, 300 mm each"],
        ["0.5 m", "1 mm Dyneema", "trip cord: dowel on the sear to the eye bolt, %.0f mm between centres" % P["CORD_L"]],
        ["1 + 3", "M4 x 35 eye bolt, machine thread, fully threaded (32 mm or more of thread), 3 M4 nuts + washer; compression spring 5 ID x 10 OD x 25 (light die spring, about 15 N/mm)", "trip post: the nut above the shelf sets the eye height (fires sooner or later), the spring under the shelf is preloaded 5 mm (75 N) so an overdriven cord gives instead of breaking"],
        ["20 m + 1", "1 mm Dyneema; small ball-bearing fishing swivel (about 20 kg)", "in hand: the tether. The swivel goes between the line and the dart bridle: the end-pull peel puts about 7 turns per metre into the line, the swivel lets it out instead of rolling the dart"],
        ["1", "ceramic rod guide insert (SiC or alumina), OD 6, ID about 3.8", "exit eye, glued into the cover: the line never runs on carbon-filled print"],
        ["1", "O-ring 18 x 2 nitrile", "drag disc friction (0.4 mm squeeze in the disc bore; check 0.15 to 0.3 N m by hand with a luggage scale on a 30 mm arm: 5 to 10 N)"],
        ["1 + 1 + 1", "3 x 10 smooth steel dowel, 2 x 8 steel dowel, 2 x 16 steel pin", "pickup slider: line post (3 mm, chamfered, never a threaded screw) and drive peg (2 mm); the 2 x 16 pin crosses the rotor hub and the 8 mm stub"],
        ["2 + 2", "bumper 16 dia x 15 (sorbothane or 60A rubber, M6 stud, into 2 M6 heat-set inserts 8 x 10 in the post tops); urethane stop 40 x 14 x 6.5 (90A) with a 4 mm hole along it for the spring cord", "rail rest feet (they take the free-fall drop); carriage end stops, rear 27 mm in their pockets (the carriage arrives with about %.1f J)" % PH["launch"]["carriage_impact_J"]],
        ["3", "KW11-3Z micro switch (2 lever, 1 roller lever)", "all COM + NO: SW_LOW (rail down), SW_COCKED outside the left cheek (pressed by the sear arm while the hook is up), SW_SEATED (roller)"],
        ["52", "M4 heat-set inserts (5.6 mm hole, 6 to 8 long)", "base tiles under every frame foot (32), rail wall strip (6), reel pad (3), program gear and cam (8), spares"],
        ["2", "M6 heat-set insert 8 x 10", "bumper post tops, for the bumper studs"],
        ["1", "steel jaw coupling, one hub 12 mm bore (Lovejoy L-050 style)", "only if the wiper output is a taper-spline hub: the other hub is bored or bolted to the wiper output"],
        ["1", "HX711 + 5 kg load cell (optional)", "best line tension reading for the tow limit and the pull test"],
        ["M3", "2 x 35 + nylocs (pulley axles), 4 x 12 countersunk (rail A and B onto the anchor post and the tie), 1 x 35 + 3 x 12 dowel (rack), 3 x 14 (spool, into the core), 2 x 10 (cover), 4 x 30 (AS5600 bracket), "
         "2 grub x 6 (rotor, stub gear), 2 grub x 8 + nuts (reel gear, at 90 deg on spot flats)", ""],
        ["M4", "8 x 12 (flange hubs into the program gear and cam inserts), 6 x 8 (cheek tabs into the wall strip inserts), 8 x 20 + nuts (tie and anchor bar), 3 x 12 (reel bracket), 7 x 16 + nuts (rail flange), 20 x 12 (plates, bumpers, trip post, strap post), 8 x 16 (motor plate, saddle), 4 x 10 (electronics box), 20 x 16 + nuts (tiles to the deck), 2 grub x 6 (coupler)", ""],
        ["M2", "6 x 10 self tapping", "the three switches"], ["M3 (sear)", "2 x 10", "sear spring post, from outside the left cheek"],
        ["power", "3S LiPo, XT60 lead #2 with the 20 A fuse (10 A blew on inrush), 14 AWG motor leads", "in hand"],
        ["1", "MP1584 buck set to 5.0 V (third one) or the rover's 5 V rail", "powers the ESP32 in the field"],
        ["7", "spacer sleeves (printed)", "2 pinion, 2 program gear, 3 main shaft: every gear is located end to end"]]
S.append(tbl(rows, [13, 62, 95]))

S.append(p("4. Print list", H2))
S.append(p("Bambu X2D, 0.2 mm layers, 4 walls. Orientation is already set in every STL (least overhang). PPA-CF for the gears, rack, cam, sear, carriage, coupler, cheeks, spring anchor, trip post and the "
           "reel cover (its eye carries the tow load). Everything else PLA. %d parts on %d plates (3MF files in plates/), about %.1f kg." % (len(PR), len(PL), sum(r["mass_g"] for r in PR) / 1000)))
rows = [["part", "mat", "infill", "size mm", "g", "supports", "note"]]
for r in PR:
    rows.append([r["name"], r["material"], r["infill"], " x ".join("%.0f" % d for d in r["dims"]), "%.0f" % r["mass_g"], "yes" if r["supports"].startswith("yes") else "no", r["note"]])
S.append(tbl(rows, [27, 13, 11, 24, 9, 12, 74], fs=7))

S.append(PageBreak())
S.append(p("5. Assembly, step by step", H2))
steps = [
    "Shafts: cut the 12 mm rod into 240 (main) and 154 (program gear stub). D flats, 1 mm deep, only from a shaft END so the D bores can slide on: main shaft right end 24 mm (coupler), reel stub front end 5 mm (rotor) and rear end 13 mm (stub gear). The reel main gear sits mid shaft: round bore, 2 x M3 grubs with nuts at 90 deg; assemble dry, mark them, grind 3 mm spot flats there. The program gear and cam flange hubs: grind two flats at 90 deg under each set screw pair. Nothing under a clutch or bearing. Dremel cut-off wheel, light passes, keep it cool.",
    "Heat-set inserts first, while every face is open: base tiles (one under each frame foot hole), rail wall strip (6, from outside), reel pad on the left cheek (3), program gear (4) and lift cam (4), and 2 x M6 inserts (8 x 10) in the bumper post tops.",
    "Frame: tiles onto the rover deck FIRST (4 x M4 cap screws each, heads sit flush in the counterbores). Put the 608ZZ with a washer each side into the post on the right plate and drive the 8 x 22 pin in from the inside (the outer end is blind). Press a 6001 into each plate. Screw the plates, bumper posts (rubber feet), motor plate, saddle, electronics box, trip post and strap post onto the tiles (M4 into the inserts). Trip post: eye bolt down through the shelf with the height nut and washer above, spring and nut below, preload the spring 5 mm. AS5600 bracket onto the left plate (4 x M3 x 30).",
    "Cheeks: press the 6001s (pivot, program gear) into both cheeks. Press each HF1216 into its steel sleeve, then bond one backstop sleeve into each cheek boss with Loctite 648 so the program gear stub turns only in the cocking direction; the long sleeve with two HF1216 goes in the drive pinion so it is driven when the main shaft turns backward. Before the Loctite sets (a few minutes), push the bare 154 mm stub through both cheek bosses and turn it: it must turn only counter-clockwise seen from the left with the muzzle to your right. Then pull the stub out again. Bolt the tie between the cheeks (2 x M4 x 20 per side, nuts in the open traps) and the spring anchor bar behind the pivot (same).",
    "Program gear stub, from the left: left backstop C, left bearing, left spacer, flange hub with the program gear bolted on (4 x M4 into its inserts; hub set screws on their flats), right spacer, right bearing, right backstop C, then the lift cam on the second flange hub at the right end.",
    "Sear first, before the rail: press the grooved pin into the boss on the sear outer plate, fit the sear spring post in the left cheek pocket (2 x M3 x 10 from outside the cheek), hang the sear spring on its 2 mm pins, and stand the sear in the pocket with the pin out through the arc slot. Then the rail: screw B onto the tie post and A onto the anchor bar post (2 x M3 countersunk each, through the cradle slot). Turn the program gear so its toothless gap faces the rack, fix the rack to the carriage first (3 x 12 dowel pressed in, then the M3 x 35 down from the top), then slide the carriage with its rack into B from the front end and let it rest forward. Only now bolt C to B at the flange (7 x M4 x 16) and fit the cheek tabs: 3 x M4 x 8 per side into the wall strip inserts. Once the rack meets a gear tooth the backstop lets the carriage move only rearward.",
    "Sear pivot: press the M4 nut into the trap on the +y face of the lug. Shoulder screw from outside through the cheek, both plates and into the nut: snug, the sear must swing free (threadlocker). The hook snaps up onto its stop on its own. SW_COCKED outside the cheek (2 x M2), lever on the arm: it clicks when you push the hook down and let go. Trip cord: clove hitch on the pin between the cheek and the e-clip, down to the eye bolt, %.0f mm between centres. Lift the rail by hand to %.1f deg (phone inclinometer): turn the height nut until the hook just clears the keel there." % (P["CORD_L"], P["TH_RELEASE"]),
    "Pulleys: 2 x F693ZZ in each flanged pulley, M3 x 35 from outside the side wall through the forks, nyloc inside. Each spring cord through its carriage shoe from the front, knot in the back pocket, over the pulley groove, to the spring (screw-in or swivel end, foam damper inside); the spring's other end left OFF the rear pin for now: hook it on only after the first power-up runs (last step) pass, with the XT60 pulled and the carriage resting forward on its stops. Press the 40 mm urethane stops into their pockets (a drop of glue). Check strap: press the grooved pin into the right cheek, tie 2 mm Dyneema from it to the strap post eye; with the rail lifted by hand to 37.5 deg, set the height nut so the strap just comes tight. A hair elastic keeps its slack off the post shelf.",
    "Reel: O-ring in the spool boss groove. Spool onto the bracket (3 x M3 x 14 into the core). Drag disc over the O-ring, against the boss shoulder: turn it by hand, it should take 0.15 to 0.3 N m. Pickup slider with its 3 mm steel post and 2 mm steel peg into the rotor hole, the peg in the disc slot. Stub through bracket and spool; rotor on the front (grub on the D flat, then drill 2 mm through hub and stub for the cross pin), stub gear on the back (grub). Glue the ceramic eye ring into the cover, cover on (2 x M3). Bolt the bracket to the left cheek pad now (3 x M4 x 12): the main gear covers these heads later.",
    "Main shaft, in from the right: right frame plate bearing, long right spacer, right pivot bearing, right pinion spacer, drive pinion (meshing the program gear), left pinion spacer, left pivot bearing, middle spacer, reel main gear (meshing the stub gear), its spacer, left frame plate bearing. Then the coupler on the D flat outside the right plate (2 x M4 grubs) and the main gear grub on its spot flat: the stack is now held end to end. AS5600 magnet in its 3 mm cap on the shaft end. Do NOT run wires at the pivot (the reel gear and the cam pinch there): see the cable routes in section 6.",
    "Wiper: you measured it before printing (section 8: torque per amp with a lever and a luggage scale at 3 currents, its shaft, crank arm and mount bolts; firmware trips set to about 9 N m at the shaft). If its holes did not land in the motor plate slots, that plate was reprinted before step 3. The printed coupler and the 3-slot motor plate are a starting point only: a car wiper crank is a taper-spline hub with a big nut. Best: a steel jaw coupling (12 mm hub on the shaft flat, the other hub bored or bolted to the wiper output), and print the motor plate to the measured holes. Then a hose clamp over the can in the saddle.",
    "Switches: SW_SEATED on the web under the cradle (2 x M2), roller up through the hole next to the pusher; SW_LOW on the right bumper post boss (2 x M2), lever on the rail wall bottom when the rail is down. AS5600 board in its bracket, 2 mm from the magnet. A hair elastic from the middle of the trip cord to the post top keeps the slack off the base when the rail is down.",
    "Line: thread the tether from the spool, over the rotor skirt, out of the ceramic eye, through the 14 mm pusher hole, through the swivel, to the dart bridle. Tie the bridle knot INSIDE the tail cap cavity: nothing may sit between the pusher and the tail face.",
    "Wiring: section 6. Then first power-up in fw/gs/README.md (springs unhooked for the first runs).",
]
S.append(tbl([["#", "step"]] + [[str(i + 1), t] for i, t in enumerate(steps)], [8, 162], fs=8.5))
S.append(Spacer(1, 4)); S.append(img(f"{PK}/renders/05_sear.png", 120))

S.append(PageBreak())
S.append(p("6. Wiring (ground station)", H2))
S.append(p("RULE: pull the XT60 every time a wire moves. Battery in last. Charge and store the LiPo in a LiPo bag, never unattended, never a puffed pack. With the springs hooked (up to 192 N), never put a hand or face in front of the muzzle or in the carriage path, and pull the XT60 before touching the rail. During fire and tow stand behind the ground station and to the side of the line, never in line with the tether. Safety glasses when cutting or grinding the steel rod."))
rows = [["from", "to", "note"],
        ["3S LiPo +", "20 A fuse, then BTS7960 B+", "XT60 lead #2"], ["3S LiPo -", "BTS7960 B-", ""],
        ["wiper wire 1 / 2", "BTS7960 M+ / M-", "if f turns the launcher, swap them"],
        ["BTS7960 RPWM / LPWM", "GPIO4 / GPIO5", "F reel / B launcher"], ["BTS7960 R_EN / L_EN", "GPIO6 / GPIO7", ""],
        ["BTS7960 VCC / GND", "ESP32 3V3 / GND", ""], ["BTS7960 R_IS + L_IS", "1 k to GPIO1, 680 R GPIO1 to the BTS7960 GND pin, 1 uF across the 680 R", "current sense: IS only reports while the high side conducts, the firmware scales it by the duty. Unpowered, tie R_IS and L_IS together and meter that point to GND: call it Rb (open = none). Rx = 1680 in parallel with Rb; isAmpsPerVolt = 8500 x 1680 / (Rx x 680) (12.5 with no onboard resistor, about 54.5 with two onboard 1 k). Then calibrate against a 10 A meter (README Tuning 1)"],
        ["AS5600 VCC GND SDA SCL DIR", "3V3, GND, GPIO8, GPIO9, GND", ""],
        ["SW_LOW / SW_COCKED / SW_SEATED", "COM to GND, NO to GPIO10 / 11 / 12", "all normally open"],
        ["BTS7960 R_EN, L_EN", "10 k to GND each", "keeps the motor off while the ESP32 resets"],
        ["HX711 (optional) DT / SCK", "GPIO13 / GPIO14", "load cell for calibration (luggage-scale style, between a fixed point and the line)"],
        ["grounds and supply", "star ground at the LiPo minus; 1000 uF 25 V across B+ / B- at the BTS7960", "the motor current must not flow through the ESP32 ground"],
        ["switches, I2C", "4.7 k pull-up on each switch input and 100 nF at the ESP32 end; each switch as a twisted pair (signal + its own GND) back to the ESP32 GND pin, never in the same tie as the wiper leads; 4.7 k on SDA/SCL, twisted pairs, 100 kHz", "AS5600 at 3.3 V: bridge its VDD5V to VDD3V3. Magnet in a 3 mm printed or brass cap on the shaft end, not glued on the steel"],
        ["ESP32 5 V", "Schottky between the MP1584 OUT+ and the ESP32 5V pin", "so the buck and USB never fight"],
        ["BTS7960 B+ / B- screw terminals (the same screws as the battery leads)", "MP1584 buck IN+ / IN-, OUT 5.0 V via the Schottky to the ESP32 5V pin + GND", "not from the LiPo directly: then the ESP32 ground reaches power ground only through the BTS7960 GND pin. Set and meter 5.0 V before connecting"],
        ["HX711 VCC / GND", "3V3 / GND", "optional"]]
S.append(tbl(rows, [55, 60, 55]))
S.append(p("Cable routes. Rail switches (SW_COCKED, SW_SEATED) ride on the rail: run them along the underside of the rail beam to the bottom of the cheek tie, then drop ONE loop of silicone wire, about 180 mm free length, to a cable-tie anchor on the base at x 130, y 0 (centre line, behind the trip and strap posts), then along the base to the box; tie the loop at both ends, it whips on every drop. SW_COCKED: lead it rearward along the outside of the left cheek, behind the sear pivot, at least 8 mm clear of the trip cord, under the cheek to the beam; never forward of the cord pin. AS5600: 4 wires twisted, about 320 mm, down the outside of the left frame plate, along the base at y -125 to the box's -x window. SW_LOW: down the right bumper post into the -x window. Wiper leads (14 AWG, crimped ferrules for the BTS7960 screw terminals): along the base at y +125 into the box's +y window, at least 30 mm from any switch cable, tied every 60 mm. The XT60 uses the +x window; the ESP32 USB-C faces the -y slot."))
S.append(Spacer(1, 4)); S.append(img(f"{PK}/wiring_gs.png", 172))
S.append(p("WiFi: GRAPPSTAR-GS / grappstar at 192.168.5.1 (the dart stays on KEDGE at 192.168.4.1). Keys, API and the tuning steps are in fw/gs/README.md."))

S.append(p("7. Mission cycle (no hand operation)", H2))
for t in ["READY: cocked, dart seated, rail down.", "FIRE: motor backward from %.1f: lift, shot at %.1f, drop, stop as the rail lands (phi re-zeroed to 333). Line pays out by end pull." % ((P["PH_REL"] + P["PH_LIFT0"]) / 2, P["PH_TRIP"]),
          "Dart lands and anchors itself (dart firmware).", "TOW: motor forward, pin out, tension held at the limit (20 N default).",
          "RELEASE: dart pulls its drills out and folds.", "COCK: motor backward 333 to %.1f." % ((P["PH_REL"] + P["PH_LIFT0"]) / 2),
          "RETRIEVE: motor forward, the dart is dragged back tail first onto the cradle until SW_SEATED. READY again."]:
    S.append(p("- " + t))

S.append(p("Soil pull test (how hard the anchored dart holds): fw/jetson/pulltest.py steps the tow limit up 5 N at a time, logs tension against line wound in, stops when the dart lets go and plots it against the Broms predictions (dry sand 7.7 N, moist loam 48.5 N, firm clay 124 N). Chock the rover first, stand to the side of the line. The procedure is in the script header."))

S.append(p("8. Before printing: buy and measure", H2))
for t in ["Buy: 4 x HF1216 + machined steel sleeves 18 x 24, 2 steel 12 mm flange hubs, 6 x 6001-2RS, 1 x 608ZZ, 2 x M4 eye bolts (35 and 60) + 2 die springs 10 x 25 and 10 x 40, 1 x 3 x 16 grooved pin + e-clip (strap), 1 x 3 x 20 grooved pin + e-clip (trip), 2 extension springs 1.6 x 12.6 x 238, 12 mm hardened rod 400 mm, 5 x 25 M4 shoulder screw, sear spring, 3 x 12 dowel, 8 mm rod, 3 x 76 pins (2), 2 mm pins, 3 x 10 and 2 x 8 dowels, 2 x 16 pin, 8 x 22 pin, 3 x KW11-3Z, 2 urethane bars 40 x 14 x 6.5 (90A) with a 4 mm hole along each, 2 bumpers 16 x 15 (sorbothane or 60A), M4 and M6 heat-set inserts, O-ring 18 x 2, ceramic rod guide ring OD 6, ball-bearing swivel, Loctite 648, 9 mm foam rod, 1000 uF 25 V cap, Schottky diode, 10 k and 4.7 k resistors, 100 nF caps, 1 uF cap, hose clamp, the screws in section 3, a third MP1584 (or use the rover 5 V). Section 3 is the full list.",
          "Measure the wiper motor: output shaft and crank arm (hole spacing), the 3 mount bolt positions, and its no-load rpm and stall current at 12 V. Put rpm and stall torque in gs/physics_gs.py WIPER and rerun gs/run_gs.sh; check its bolts land in the motor plate slots (radius 30 to 55).",
          "Measure the rail drop time on the bench (film it, or read the firmware log from SW_COCKED opening to SW_LOW closing) and set gs_logic.h dropMaxMs to that plus 60 to 80 ms (default 300 ms).",
          "Measure the springs you get: hang 1 kg and 5 kg, note the lengths, compute the rate. Put it in physics_gs.py SPR and rerun.",
          "Print the PPA-CF parts first (gears, rack, cam, sear, carriage, coupler, cheeks): they carry the loads."]:
    S.append(p("- " + t))

S.append(p("9. Not proven yet (say so if asked)", H2))
for t in ["Spring rate and the wiper's real torque and speed are catalogue assumptions: measure both, put them in physics_gs.py, rerun.",
          "Retrieve onto the cradle is the weakest link. The horn curls 80 deg, past vertical, but its lower end is still about 300 mm above the ground: the dart has to be lifted hanging by its tail. Past the carriage stops the rail opens up (no shoe lips, low cradle edges and walls) and a stepped funnel, shaped from the folded dart rolled +-47 deg at the muzzle narrowing to +-8 deg at the lips, turns a rolled dart upright as it is dragged in. That funnel is geometry, not a contact simulation. Fallback: if the motor stops on its current spike before SW_SEATED, seat the dart by hand.",
          "The first program gear tooth meets the standing rack tip first (modelled exactly by the tooth push simulation); load is lowest there (42 N), PPA-CF teeth.",
          "No level wind on the spool: turns pile near the front and slide back under tension. Wind slowly in the first tests and watch it.",
          "The set screws in the two steel flange couplings carry up to %.1f N m: check them after the first 20 shots." % PH["motor"]["T_pg_max_Nm"],
          "The catch: the last tooth lets the carriage go at once, it runs %.1f mm onto the hook (about 0.3 J). Kept short on purpose; keel 18 mm long with root fillets, carriage printed on its side so the keel bends along its layers, PPA-CF on PPA-CF. Look at the keel and hook after 50 shots." % (P["S_SETTLED"] - P["P_D"]),
          "The trip cord runs outside the cheek in one plane; its slack lies loose when the rail is down (use the elastic). If the eye nut is set too low the sear lands on its down-stop and the eye spring gives.",
          "Launch dynamics beyond the lumped model: the springs surge (35 Hz), the cords go slack for about 15 ms after the stop, and the release puts up to about 95 g and 260 N on the dart tail for an instant (the dart's cells still pass, with little margin). A 1.5 mm EVA ring on the pusher face would halve that spike; it moves the seat 1.5 mm, so it is not in the CAD.",
          "The drop is a free fall (about 0.9 J into the two 16 x 15 bumpers every shot): bearing A overruns, then relocks at once, so the firmware re-zeroes phi to about 333 at the SW_LOW edge and the next cock re-zeroes at the sear switch. Above about 42 rpm at the top of the lift the rail can leave the cam; the firmware eases the duty from phi 285 to 302.5."]:
    S.append(p("- " + t))

doc = SimpleDocTemplate(f"{PK}/GrappStar_ground_station_build_guide.pdf", pagesize=A4, leftMargin=18 * mm, rightMargin=18 * mm, topMargin=15 * mm, bottomMargin=15 * mm)
doc.build(S)
print("guide ok")
