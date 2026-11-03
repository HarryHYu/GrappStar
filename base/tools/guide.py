"""Build guide PDF: OUT/guide.pdf (reportlab, plain black Liberation Sans).

python -m tools.guide OUT [--only demo] [--title "..."] [--pack PACK]

Sections: title page (title, date), safety, parts list (tools.bom), print list (PACK/print_report.json from
tools.pack, default PACK = OUT/pack, skipped when missing), assembly steps, audit summary (OUT/audit.json) and what
must be measured (P.MEASURE when the params define it).

The assembly steps are tools.steps.plan_steps on the exported manifest, the same list as the assembly page
(assembly.html): the same numbering, the parts of Part.step and the auto steps for parts in no step, and the step
image OUT/shots/assembly_step<nn>.png (or step_<n>.png, step_<nn>.png) of that number. A wiring step (wire, lead,
solder, connector, plug, terminal, battery, JST, crimp in the title or text) always starts with "Battery unplugged.";
it is prepended, with a note, when the step text does not already start with it.
"""
from __future__ import annotations

import argparse
import os
import re
import sys
import time
from types import SimpleNamespace

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

from tools import bom as bom_tool  # noqa: E402
from tools.pack import canvasmaker, clean, doc_template, load_json, load_out, only_list, pdf_image, pdf_kit  # noqa: E402
from tools.steps import plan_steps  # noqa: E402

BATTERY = "Battery unplugged."
WIRING_RE = re.compile(r"\b(wir(e|es|ed|ing)|leads?|solder\w*|connect(or|ors)|plug(s|ged|ging)?|terminals?|"
                       r"batter(y|ies)|jst|crimp\w*)\b", re.I)

SAFETY = [
    "Wiring always starts with the battery unplugged. Plug it in last, after a check of every connection.",
    "Keep hands, hair and loose cloth away from the gears, the reel and the launcher while the motor can run.",
    "Never stand in front of the launcher. Treat it as loaded whenever the spring is cocked.",
    "Release the spring before you work on the launcher.",
    "Wear eye protection when you cut, drill or press parts, and when you test a launch.",
    "Let hot printed parts and the nozzle cool before you touch them. Print PPA-CF with the enclosure closed.",
    "Charge LiPo batteries on a fireproof surface and never leave them charging alone.",
]


def step_image(out, n):
    sd = os.path.join(out, "shots")
    for nm in (f"step_{n}.png", f"step_{n:02d}.png", f"assembly_step{n:02d}.png"):
        fn = os.path.join(sd, nm)
        if os.path.exists(fn):
            return fn
    return None


def is_wiring(step: dict) -> bool:
    return bool(WIRING_RE.search(f"{step.get('title', '')} {step.get('text', '')}"))


def battery_rule(step: dict) -> tuple[str, bool]:
    """(step text, changed): a wiring step gets "Battery unplugged. " in front unless it already starts with it."""
    txt = (step.get("text") or "").strip()
    if not is_wiring(step) or txt.lower().startswith(BATTERY.lower()):
        return txt, False
    return (f"{BATTERY} {txt}" if txt else BATTERY), True


def guide_steps(man: dict, only=None) -> tuple[list[dict], list[str]]:
    """The assembly page's step list (tools.steps.plan_steps) built from the manifest, battery rule applied.
    Returns (steps, notes)."""
    names = only_list(only)
    parts = {p["name"]: SimpleNamespace(step=p.get("step"), subsystem=p.get("subsystem", ""), joint=p.get("joint"),
                                        printed=bool(p.get("printed"))) for p in man["parts"]}
    mods = [m for m in man.get("subsystems", []) if not names or m in names]
    steps, notes = plan_steps(SimpleNamespace(steps=man.get("steps", []), parts=parts, modules=mods))
    for n, st in enumerate(steps, 1):
        st["text"], changed = battery_rule(st)
        if changed:
            notes.append(f"step {n} '{st['title']}' is a wiring step: prepended '{BATTERY}'")
    return steps, notes


def measure_items(P):
    items = getattr(P, "MEASURE", None) or []
    out = []
    for it in items:
        if isinstance(it, dict):
            name = it.get("name") or it.get("key") or ""
            txt = it.get("what") or it.get("text") or it.get("how") or ""
            out.append((name, txt))
        elif isinstance(it, (list, tuple)):
            out.append((str(it[0]), " ".join(str(x) for x in it[1:])))
        else:
            out.append(("", str(it)))
    return out


def audit_lines(au):
    if au is None:
        return "NO AUDIT", ["No audit.json found. Run python -m tools.audit OUT first."]
    L = [f"{au.get('n_parts', '?')} parts checked over {au.get('n_states', '?')} states."]
    for e in au.get("errors", []):
        L.append(f"Error: {e}")
    for r in au.get("static_fails", []):
        L.append(f"Part {r['part']}: {'; '.join(r['fails'])}")
    for c in au.get("collisions", []):
        L.append(f"Collision {c['a']} with {c['b']}: {c['max_vol_mm3']} mm3 in {c['n_states']} states.")
    for c in au.get("clearances", []):
        if not c.get("ok"):
            L.append(f"Clearance {c['a']} to {c['b']}: gap {c.get('gap')} mm, needs {c['min_mm']} mm.")
    for f in au.get("floating", []):
        L.append(f"Floating part {f['part']}: {f['why']}.")
    for x in au.get("limits", [])[:10]:
        L.append(f"Joint {x['joint']} out of its limits ({x['value']:.2f}).")
    for c in au.get("checks", []):
        L.append(f"Check {c['subsystem']}: {c['fail']}")
    L.append(f"Allowed contacts: {len(au.get('allowed_contacts', []))}.")
    return ("CLEAN" if au.get("clean") else "NOT CLEAN"), L


COUPON_MD = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "pack", "coupons", "COUPON_TEST.md")


def coupon_story(Pp, only=None, md=COUPON_MD):
    """Flowables for the coupon section from COUPON_TEST.md (headings, bullets, paragraphs; tables become lines)."""
    import re
    if only == "demo" or not os.path.exists(md):
        return [Pp("Print the fit coupon first (python -m tools.coupon OUT) and tune cad/lib/fits.py from it.")]
    out = [Pp("Print these before any big part. Each coupon uses the same geometry and numbers as the real part. "
              "Files: base/pack/coupons (3MF plates, STL, COUPON_TEST.pdf). Also print the fit coupon "
              "(python -m tools.coupon OUT) to tune cad/lib/fits.py.")]
    for line in open(md, encoding="utf-8").read().splitlines():
        t = line.strip()
        if not t or t.startswith("# ") or set(t) <= set("|-: "):
            continue
        t = re.sub(r"`([^`]*)`", r"\1", t).replace("**", "")
        t = t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        if t.startswith("### "):
            out.append(Pp(clean(t[4:]), "H3"))
        elif t.startswith("## "):
            out.append(Pp(clean(t[3:]), "H3"))
        elif t.startswith("|"):
            out.append(Pp(clean(" ".join(c.strip() for c in t.strip("|").split("|"))), "SM"))
        else:
            out.append(Pp(clean(t)))
    return out


def run(out, only=None, title=None, pack_dir=None, log=print) -> str:
    from reportlab.lib.units import mm
    from reportlab.platypus import KeepTogether, PageBreak, Spacer
    from tools.params import load_params
    P = load_params()
    man = load_out(out, only)
    bom = bom_tool.build(man, out)
    pack_dir = pack_dir or os.path.join(out, "pack")
    pr = load_json(os.path.join(pack_dir, "print_report.json"))
    au = load_json(os.path.join(out, "audit.json"))
    title = clean(title or ("GrappStar demo build guide" if only == "demo" else "GrappStar base station build guide"))
    st, Pp, tbl, note = pdf_kit()
    S = []
    # title page
    S += [Spacer(1, 60 * mm), Pp(title, "H1"), Pp(time.strftime("%d %B %Y")), Spacer(1, 6 * mm),
          Pp(f"Subsystems: {', '.join(man.get('subsystems', []))}. Model exported {man.get('created', '')}."),
          Pp("All sizes in mm. Masses and print times are estimates.")]
    if note:
        S.append(Pp(note, "SM"))
    S.append(PageBreak())
    # safety
    S.append(Pp("1. Safety", "H2"))
    S += [Pp("- " + s) for s in SAFETY]
    # first print: the day-one risk coupons (base/pack/coupons/COUPON_TEST.md, written by tools.risk_coupon)
    S.append(Pp("2. First print: test coupons", "H2"))
    S += coupon_story(Pp, only)
    # parts list
    S.append(Pp("3. Parts list", "H2"))
    S.append(Pp("Printed parts", "H3"))
    rows = [["part", "subsystem", "material", "qty", "mass g", "notes"]]
    rows += [[r["name"], r["subsystem"], r["material"], r["qty"], f"{r['mass_g']:.1f}", r["notes"]] for r in bom["printed"]]
    S.append(tbl(rows, [38, 22, 18, 10, 15, 79]))
    if bom["totals"]:
        S.append(Spacer(1, 3))
        S.append(Pp("Printed mass: " + "; ".join(f"{m} {t['mass_g']:.0f} g ({t['filament_m']:.1f} m of filament)"
                                                 for m, t in bom["totals"].items()) + "."))
    if bom["bought"]:
        S.append(Pp("Bought parts", "H3"))
        rows = [["item", "type", "count", "used by"]]
        rows += [[b["ref"], b["category"], b["count"], ", ".join(b["used_by"])] for b in bom["bought"]]
        S.append(tbl(rows, [55, 22, 13, 92]))
    # print list
    S.append(Pp("4. Print list", "H2"))
    if pr:
        S.append(Pp(f"{len(pr['parts'])} printed parts on {len(pr['plates'])} plates. The 3MF plates and STL files are in the print pack."))
        rows = [["part", "material", "qty", "print size mm", "supports"]]
        rows += [[r["name"], r["material"], r["qty"], " x ".join(f"{d:.0f}" for d in r["dims"]), r["supports"]] for r in pr["parts"]]
        S.append(tbl(rows, [45, 20, 10, 32, 75]))
        S.append(Spacer(1, 3))
        for pl in pr["plates"]:
            S.append(Pp(f"{pl['file']}: {', '.join(pl['objects'])}."))
        for e in pr.get("errors", []):
            S.append(Pp(f"Pack error: {e}"))
        S.append(Pp("Print settings are in PRINT_SHEET.pdf. Use a hardened nozzle for PPA-CF and dry it first."))
    else:
        S.append(Pp("No print pack found. Run python -m tools.pack OUT PACK first."))
    # steps
    S.append(PageBreak())
    S.append(Pp("5. Assembly steps", "H2"))
    steps, notes = guide_steps(man, only)
    for x in notes:
        log(f"  note: {x}")
    if not steps:
        S.append(Pp("No assembly steps are defined yet."))
    for n, stp in enumerate(steps, 1):
        blk = [Pp(stp.get("heading") or f"Step {n}. {stp.get('title', '')}", "H3")]
        if stp.get("text"):
            blk.append(Pp(stp["text"]))
        if stp.get("parts"):
            blk.append(Pp("Parts: " + ", ".join(stp["parts"]) + ".", "SM"))
        if stp.get("handle"):
            blk.append(Pp("Also handle (fitted earlier): " + ", ".join(stp["handle"]) + ".", "SM"))
        img = step_image(out, n)
        if img:
            blk += [Spacer(1, 2), pdf_image(img, 150, 95)]
        S.append(KeepTogether(blk))
        S.append(Spacer(1, 4))
    # audit
    verdict, lines = audit_lines(au)
    S.append(Pp("6. Audit summary", "H2"))
    S.append(Pp(f"Result: {verdict}"))
    S += [Pp(x) for x in lines]
    # measure
    items = measure_items(P)
    if items:
        S.append(Pp("7. What must be measured", "H2"))
        S.append(Pp("Measure these before the final print and put the numbers in the params file."))
        for name, txt in items:
            S.append(Pp("- " + (f"{name}: {txt}" if name and txt else name or txt)))
    fn = os.path.join(out, "guide.pdf")
    doc = doc_template(fn, title, leftMargin=14 * mm, rightMargin=14 * mm, topMargin=15 * mm, bottomMargin=15 * mm)
    doc.build(S, onLaterPages=_page_no(st), canvasmaker=canvasmaker())
    log(f"guide: {fn} ({len(steps)} steps, audit {verdict}, print list {'yes' if pr else 'no'})")
    return fn


def _page_no(st):
    def draw(canvas, doc):
        canvas.saveState()
        canvas.setFont(st["SM"].fontName, 8)
        canvas.drawRightString(doc.pagesize[0] - doc.rightMargin, 8 * 2.835, str(doc.page))
        canvas.restoreState()
    return draw


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--only", default="")
    ap.add_argument("--title", default="")
    ap.add_argument("--pack", default="", help="print pack directory (default OUT/pack)")
    a = ap.parse_args(argv)
    run(a.out, a.only or None, a.title or None, a.pack or None)
    return 0


if __name__ == "__main__":
    sys.exit(main())
