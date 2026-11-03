"""Bill of materials: OUT/BOM.csv and OUT/BOM.md.

python -m tools.bom OUT [--only demo]

Printed parts: name, subsystem, material, qty, mass (each and total), notes. Bought items are grouped by their
reference (case-insensitive, runs of spaces count as one): every part's bought_ref counts (a bought or steel part is
the item itself), and any part, printed ones too, may declare extra hardware in bought_ref:
    bought_ref="608ZZ 8x22x7"                              one 608 per part copy
    bought_ref="4 x M3x10 SHCS ISO 4762; 4 x M3 heat-set insert"   on a printed part: its fasteners
    bought_ref="2xM3x8 BHCS"                               two M3x8 (no spaces is fine before an M size)
    bought_ref="8 x 120 mm ground steel rod"               ONE rod: a number before a size is not a count
    bought_ref="2 x 608ZZ 8x22x7"                          two 608: a part code starting with digits still counts

bought_ref count syntax (the README sentence): bought_ref is a list of items separated by ";", and each item may
start with a count written "<n> x <item>" (for example "4 x M3x10 SHCS"), or "<n>x<item>" with no spaces only when
the item starts with an M size (for example "2xM3x8 BHCS"); when the text after the "x" starts with a plain number
(as in "8 x 120 mm rod") it is read as a size, so the whole item is the name and the count is 1, while a part code
that starts with digits (as in "2 x 608ZZ") still counts; counts are multiplied by the part qty.

Categories: bearings (a bearing code such as 608ZZ, 608-2RS, 693ZZ, F693ZZ, MR105ZZ, CSK20PP, a 6000 to 6900
series number, LMxxUU, or the word bearing or bushing), fasteners, other. Totals: printed mass per material and the
filament length (1.75 mm).
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

from tools.pack import DENSITY, FIL_D, MAT_ORDER, clean, filament_m, load_out  # noqa: E402

# "<n> x <item>" with spaces; the item must not start with a number (a size: "8 x 120 mm rod", "10 x 3x8 dowel") unless
# the digits run straight into letters or a dash, as a part code does ("2 x 608ZZ", "2 x 6204-2RS"; but "120mm" is a
# length); or "<n>x<item>" without spaces when the item starts with an M size ("2xM3x8 BHCS")
MULT_RE = re.compile(r"^\s*(\d+)\s*[xX]\s+(?!\d+(?:[.,]\d+)?(?:[\sxX*]|mm|cm|m\b|$))(\S.*)$"
                     r"|^\s*(\d+)\s*[xX](M\d.*)$")
_NOT_SIZE = r"(?![\d.,])(?!\s*mm)"     # not part of a longer number or a length ("608 mm")
BEARING_RE = re.compile(
    r"\b(F|MF|MR|SMR)?\d{3,4}-?(ZZ|2RS|2Z|RS|Z|DDU|LLU)\b"         # 608ZZ, 608-2RS, 693ZZ, F693ZZ, 6204-2RS
    r"|\b6[0-9]{3}" + _NOT_SIZE +                                    # 6000 to 6999 series (6204, 6806)
    r"|\b(60[3-9]|62[3-9]|68[3-9]|69[0-9])" + _NOT_SIZE +            # bare miniature codes (608, 625, 693)
    r"|\bMR\d{2,4}\b|\bCSK\d+P{0,2}\b|\bLM\d+(UU|LUU)?\b|\bHF\d+\b"   # MR105, CSK20PP, LM8UU, HF0612
    r"|bearing|bushing", re.I)
FASTENER_RE = re.compile(r"^M\d+(\.\d+)?(\b|x)|\b(SHCS|BHCS|FHCS|screw|bolt|nut|washer|insert|grub|dowel|pin|circlip|e-clip|rivet)\b", re.I)


def parse_ref(ref: str):
    """'4 x M3x10 SHCS; 608ZZ; 2xM3x8 BHCS; 8 x 120 mm rod' ->
    [(4, 'M3x10 SHCS'), (1, '608ZZ'), (2, 'M3x8 BHCS'), (1, '8 x 120 mm rod')]."""
    out = []
    for item in (ref or "").split(";"):
        item = " ".join(item.split())
        if not item:
            continue
        m = MULT_RE.match(item)
        if m:
            n, rest = (m.group(1), m.group(2)) if m.group(1) else (m.group(3), m.group(4))
            out.append((int(n), rest.strip()))
        else:
            out.append((1, item))
    return out


def category(ref: str) -> str:
    if BEARING_RE.search(ref):
        return "bearings"
    if FASTENER_RE.search(ref):
        return "fasteners"
    return "other"


def group_key(ref: str) -> str:
    """Grouping key of a bought item: case-insensitive, whitespace normalised, dashes cleaned."""
    return " ".join(clean(ref).split()).casefold()


def md_cell(s) -> str:
    """Text for a markdown table cell: no dashes, "|" escaped, no line breaks."""
    return " ".join(clean(s).split()).replace("|", "\\|")


def build(man: dict) -> dict:
    printed, bought, unref = [], {}, []
    for p in man["parts"]:
        qty = int(p.get("qty", 1))
        if p.get("printed"):
            m = p.get("mass_g") or 0.0
            printed.append(dict(name=p["name"], subsystem=p.get("subsystem", ""), material=p["material"], qty=qty,
                                mass_g=round(m, 1), mass_total_g=round(m * qty, 1), notes=clean(p.get("notes", ""))))
        refs = parse_ref(p.get("bought_ref", ""))
        if not p.get("printed") and not refs:
            unref.append(p["name"])
            refs = [(1, f"{p['name']} ({p['material']}, no bought_ref)")]
        for n, ref in refs:
            ref = clean(ref)
            b = bought.setdefault(group_key(ref), dict(ref=ref, category=category(ref), count=0, used_by=[]))
            b["count"] += n * qty
            if p["name"] not in b["used_by"]:
                b["used_by"].append(p["name"])
    order = {m: i for i, m in enumerate(MAT_ORDER)}
    printed.sort(key=lambda r: (order.get(r["material"], 99), r["subsystem"], r["name"]))
    totals = {}
    for r in printed:
        t = totals.setdefault(r["material"], dict(count=0, mass_g=0.0))
        t["count"] += r["qty"]
        t["mass_g"] += r["mass_total_g"]
    for mat, t in totals.items():
        t["mass_g"] = round(t["mass_g"], 1)
        t["filament_m"] = round(filament_m(t["mass_g"], mat), 1)
    cats = ["bearings", "fasteners", "other"]
    bl = sorted(bought.values(), key=lambda b: (cats.index(b["category"]), b["ref"].casefold()))
    return dict(printed=printed, bought=bl, totals=totals, unreferenced=unref)


def write(out: str, bom: dict, title="Bill of materials"):
    with open(os.path.join(out, "BOM.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["section", "name_or_ref", "subsystem_or_category", "material", "qty", "mass_g_each", "mass_g_total", "notes_or_used_by"])
        for r in bom["printed"]:
            w.writerow([clean(x) for x in ["printed", r["name"], r["subsystem"], r["material"], r["qty"], r["mass_g"],
                                           r["mass_total_g"], r["notes"]]])
        for b in bom["bought"]:
            w.writerow([clean(x) for x in ["bought", b["ref"], b["category"], "", b["count"], "", "", " ".join(b["used_by"])]])
        for mat, t in bom["totals"].items():
            w.writerow([clean(x) for x in ["total", mat, "", mat, t["count"], "", t["mass_g"],
                                           f"{t['filament_m']} m of {FIL_D} mm filament"]])
    L = [f"# {title}", "", "## Printed parts", "", "| part | subsystem | material | qty | mass g each | mass g total | notes |",
         "|---|---|---|---|---|---|---|"]
    for r in bom["printed"]:
        L.append(f"| {md_cell(r['name'])} | {md_cell(r['subsystem'])} | {md_cell(r['material'])} | {r['qty']} | "
                 f"{r['mass_g']:.1f} | {r['mass_total_g']:.1f} | {md_cell(r['notes'])} |")
    L += ["", "## Printed totals", "", "| material | parts | mass g | filament m (1.75 mm) |", "|---|---|---|---|"]
    for mat, t in bom["totals"].items():
        L.append(f"| {mat} | {t['count']} | {t['mass_g']:.0f} | {t['filament_m']:.1f} |")
    L += ["", "## Bought parts", ""]
    for cat in ("bearings", "fasteners", "other"):
        rows = [b for b in bom["bought"] if b["category"] == cat]
        if not rows:
            continue
        L += [f"### {cat.capitalize()}", "", "| item | count | used by |", "|---|---|---|"]
        L += [f"| {md_cell(b['ref'])} | {b['count']} | {md_cell(', '.join(b['used_by']))} |" for b in rows]
        L.append("")
    if bom["unreferenced"]:
        L += ["Parts that are not printed and have no bought_ref: " + ", ".join(bom["unreferenced"]) + ".", ""]
    with open(os.path.join(out, "BOM.md"), "w") as fh:
        fh.write(clean("\n".join(L)) + "\n")


def run(out, only=None, log=print) -> dict:
    man = load_out(out, only)
    bom = build(man)
    write(out, bom)
    log(f"bom: {len(bom['printed'])} printed parts, {len(bom['bought'])} bought items -> {out}/BOM.csv, BOM.md")
    for b in bom["bought"]:
        log(f"  {b['count']:4d} x {b['ref']}")
    for mat, t in bom["totals"].items():
        log(f"  {mat}: {t['mass_g']:.0f} g, {t['filament_m']:.1f} m filament")
    return bom


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--only", default="")
    a = ap.parse_args(argv)
    run(a.out, a.only)
    return 0


if __name__ == "__main__":
    sys.exit(main())
