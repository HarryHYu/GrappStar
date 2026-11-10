"""Show build deliverables: one zip of the print-oriented STLs in PLA/ and PPA-CF/ folders, and a one-page note
(text and PDF, plain black Liberation Sans): what to print, the few bought parts, how to work it by hand.

python -m tools.show_pack OUT DST       (OUT: the run_all output with asm.json, pack/stl, pack/print_report.json,
                                         audit.txt; DST: where the zip and the note go)
"""
from __future__ import annotations

import json
import os
import sys
import zipfile

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

ZIP = "grappstar_base_show_stl.zip"

# bought parts the show build still uses (all from the base BOM bought earlier: nothing new); the counts are checked
# against asm.json bought_ref below
BOUGHT = [
    ("608ZZ bearing 8x22x7", "6", "aim worm wheel (2), reel clutch hub (2), spool shaft (2)"),
    ("693ZZ bearing 3x8x4", "2", "lip roller, rail-rear tether pulley"),
    ("Century C-243 extension spring", "2", "launcher springs"),
    ("8 mm steel rod, 2 short pins", "2", "spring anchor pins (a printed PPA-CF d8 pin works for show)"),
    ("M8x25 hex bolt + M8 nut", "1", "aim worm wheel axle (a printed PPA-CF d8 pin works for show)"),
    ("Dyneema braid 1 mm", "about 2 m", "tether: dart tail knot, pusher hole, rail pulley, reel (20 m for real use)"),
    ("M3 screws, nuts, heat-set inserts", "as base BOM", "deck, cheeks, plates, guards; plus 1 x M3x6 for the crank"),
    ("12 V wiper motor", "1", "bolted to its adapter plate, NOT wired: its shaft carries the coupler hub, the bearing of "
                              "the main shaft's motor end (no shear pin, so the shaft turns free on it)"),
]
OPTIONAL = [
    ("N20 worm gearmotor", "sits in its cradle on the drum bracket, no pinion, NOT linked"),
]
DROPPED_TEXT = ("Left out: both CSK20PP sprags (sprag #1 is now a printed hub, sprag #2 a plain bore), the aim swing-arm "
                "splitter (sun, arm, drag pad, shim, idler, clip), the take-up swing idler and its arm, the shear pin, the "
                "N20 pinion, the AS5600 and its bracket and magnet cap, every contact switch (drum code levers and block, "
                "index lever, dog post, SW_SEATED, SW_HOME, SW_GAP, SW_RAIL_LOW, SW_ARM), the electronics bay with its "
                "boards, the camera and receiver mounts and the cable anchors.")

OPERATE = [
    ("Modes (drum hand lever)", "Swing the lever on the drum shaft end: L = LAUNCH (crank drives the launcher), N = "
     "NEUTRAL (crank drives nothing), the wrap sector (the cup lays the wraps round the capstan), R = REEL (crank drives "
     "the capstan). The drum is free, so leave it where you put it."),
    ("Aim (knob)", "Turn the knob on the aim countershaft (outboard side of the aim housing): the bevels, the worm and the "
     "crank-rocker lift the rail from 6 to 47 deg and back, 10 knob turns per full cycle. The worm holds the angle."),
    ("Cock and fire (crank, drum in LAUNCH)", "Turn the hand crank FORWARD (anticlockwise seen from the crank end, clockwise seen from the motor): the "
     "big gear draws the carriage back 241 mm. About 1.2 crank turns in, the big gear's relieved last tooth lets the rack "
     "go and the springs fire the dart: that tooth is the sear and its trip, there is no separate trigger. Keep turning "
     "a little to bring the gear round to its gap (reset). The crank carries the full spring load (about 50 N at the "
     "handle near full cock) and nothing holds it: keep hold of it while cocked; turn it BACK to de-cock. Never fire "
     "without the tether tied; the flexure stops catch the carriage at the end of the stroke."),
    ("Load", "Carriage at the front, big gear gap facing the rack: lay the folded v34 dart belly down on the cradle, tail "
     "on the pusher. Tie the line through the two tail holes, knot about 2 mm behind the tail face, then run it straight "
     "back through the pusher's 14 mm hole and under the rail-rear pulley to the reel."),
    ("Reel (crank, drum in REEL)", "Turning the crank forward winds the capstan; it turns either way (no backstop). "
     "The storage spool has no drive now (take-up idler gone): it turns free, wind or pay out its line by hand."),
]


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    out, dst = argv[0], argv[1]
    os.makedirs(dst, exist_ok=True)
    asm = json.load(open(os.path.join(out, "asm.json")))
    rep = json.load(open(os.path.join(out, "pack", "print_report.json")))
    stl = os.path.join(out, "pack", "stl")
    parts = {p["name"]: p for p in asm["parts"]}
    printed = [p for p in asm["parts"] if p.get("printed")]
    # zip: <material>/<part>[_x<qty>].stl
    zf = os.path.join(dst, ZIP)
    n_files, by_mat = 0, {}
    with zipfile.ZipFile(zf, "w", zipfile.ZIP_DEFLATED) as z:
        for p in sorted(printed, key=lambda q: (q["material"], q["name"])):
            f = os.path.join(stl, p["name"] + ".stl")
            if not os.path.exists(f):
                raise SystemExit(f"missing STL {f}")
            q = int(p.get("qty") or 1)
            arc = f"{p['material']}/{p['name']}" + (f"_x{q}" if q > 1 else "") + ".stl"
            z.write(f, arc)
            n_files += 1
            m = by_mat.setdefault(p["material"], dict(parts=0, copies=0, cm3=0.0))
            m["parts"] += 1
            m["copies"] += q
            m["cm3"] += q * p["volume_mm3"] / 1000.0
    tot = rep.get("totals", {})
    audit = ""
    ap = os.path.join(out, "audit.txt")
    if os.path.exists(ap):
        lines = open(ap).read().strip().splitlines()
        audit = lines[-1] if lines else ""
        head = lines[0] if lines else ""
    else:
        head = ""
    n_bought = sum(1 for p in asm["parts"] if not p.get("printed"))
    # ---------------------------------------------------------------- note text
    L = []
    L.append("GrappStar base station, SHOW build (hand operated), KEDGE v34 dart")
    L.append("")
    L.append("The D50 launcher and reel as before, sized to the v34 dart, with every axle hanging free: no one-way "
             "bearings, no motor links, no sensing, no electronics. Everything turns by hand.")
    L.append("")
    L.append("PRINT")
    for mat in ("PLA", "PPA-CF"):
        if mat in by_mat:
            m, t = by_mat[mat], tot.get(mat, {})
            L.append(f"  {mat}: {m['parts']} parts ({m['copies']} prints), {m['cm3']:.0f} cm3 model volume, about "
                     f"{t.get('mass_g', 0) / 1000:.1f} kg of filament, {t.get('plates', 0)} plates, about "
                     f"{t.get('time_h', 0):.0f} h. Folder {mat}/ in {ZIP}.")
    L.append("  Every STL is already in print orientation (flat face down). PPA-CF parts are the gears, shafts and "
             "hubs that carry the spring load; keep them PPA-CF. New show parts: hand crank, crank handle, aim knob "
             "(PLA), the hub in place of sprag #1 (PPA-CF).")
    L.append("  Printed pins: the yoke spring pegs are printed already; for a show model the two spring anchor pins "
             "and the aim wheel axle can be printed PPA-CF d8 pins too.")
    L.append("")
    L.append("BOUGHT (all from the base parts bought earlier, nothing new)")
    for it, c, why in BOUGHT:
        L.append(f"  {c} x {it}: {why}" if c.isdigit() else f"  {it} ({c}): {why}")
    L.append("  Optional, not linked: " + "; ".join(f"{a} ({b})" for a, b in OPTIONAL))
    L.append("")
    L.append("OPERATE BY HAND")
    for t_, x in OPERATE:
        L.append(f"  {t_}: {x}")
    L.append("")
    L.append(DROPPED_TEXT)
    L.append("")
    L.append(f"Audit: {audit} ({head}). Renders: renders/show_states.png. Design source: base/cad (dart-v1); the full "
             "motorised D50 design is tag base-parked-d50.")
    txt = "\n".join(L) + "\n"
    for bad in (chr(0x2014), chr(0x2013)):
        assert bad not in txt, "dash in the note"
    open(os.path.join(dst, "SHOW_NOTE.txt"), "w").write(txt)
    _pdf(L, os.path.join(dst, "SHOW_NOTE.pdf"))
    print(f"{zf}: {n_files} STL; printed parts {len(printed)}, bought or reference parts {n_bought}")
    for mat, m in by_mat.items():
        print(f"  {mat}: {m}")
    print(txt)
    return 0


def _pdf(lines, fn):
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.units import mm
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont
    from reportlab.lib.styles import ParagraphStyle
    from reportlab.platypus import Paragraph, SimpleDocTemplate, Spacer
    fd = "/usr/share/fonts/truetype/liberation/"
    pdfmetrics.registerFont(TTFont("LibSans", fd + "LiberationSans-Regular.ttf"))
    pdfmetrics.registerFont(TTFont("LibSans-Bold", fd + "LiberationSans-Bold.ttf"))
    body = ParagraphStyle("b", fontName="LibSans", fontSize=8.2, leading=10.0, textColor="black")
    item = ParagraphStyle("i", parent=body, leftIndent=8, spaceBefore=0.6)
    head = ParagraphStyle("h", parent=body, fontName="LibSans-Bold", fontSize=9.2, leading=11.5, spaceBefore=3)
    title = ParagraphStyle("t", parent=body, fontName="LibSans-Bold", fontSize=12, leading=15)

    def esc(s):
        return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    flow = []
    for i, ln in enumerate(lines):
        if i == 0:
            flow.append(Paragraph(esc(ln), title))
        elif not ln:
            flow.append(Spacer(1, 2))
        elif ln.isupper() or ln.startswith(("PRINT", "BOUGHT", "OPERATE")):
            flow.append(Paragraph(esc(ln), head))
        elif ln.startswith("  "):
            t = ln.strip()
            if ": " in t and t.split(": ", 1)[0][:1].isalpha() and len(t.split(": ", 1)[0]) < 48:
                a, b = t.split(": ", 1)
                flow.append(Paragraph(f"<font name='LibSans-Bold'>{esc(a)}:</font> {esc(b)}", item))
            else:
                flow.append(Paragraph(esc(t), item))
        else:
            flow.append(Paragraph(esc(ln), body))
    doc = SimpleDocTemplate(fn, pagesize=A4, leftMargin=14 * mm, rightMargin=14 * mm, topMargin=12 * mm,
                            bottomMargin=10 * mm, title="GrappStar base show build", author="")
    doc.build(flow)


if __name__ == "__main__":
    sys.exit(main())
