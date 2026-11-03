"""Bill of materials: OUT/BOM.csv and OUT/BOM.md.

python -m tools.bom OUT [--only demo]

Printed parts: name, subsystem, material, qty, mass (each and total), notes. Bought items are grouped by their
reference: every part's bought_ref counts (a bought or steel part is the item itself), and any part, printed ones
too, may declare extra hardware in bought_ref as "; "-separated entries with an optional "<n> x " prefix:
    bought_ref="608ZZ 8x22x7"                              one 608 per part copy
    bought_ref="4 x M3x10 SHCS ISO 4762; 4 x M3 heat-set insert"   on a printed part: its fasteners
Counts are multiplied by the part qty. Categories: bearings, fasteners, other. Totals: printed mass per material
and the filament length (1.75 mm).
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

MULT_RE = re.compile(r"^\s*(\d+)\s*[xX]\s+(.+)$")
BEARING_RE = re.compile(r"^(\d{3,5}(ZZ|2RS|-2RS|RS|Z)?|HF\d+|MR\d+|F\d{3,4}|LM\d+|KFL|UCP|KP)\b|bearing|bushing", re.I)
FASTENER_RE = re.compile(r"^M\d+(\.\d+)?(\b|x)|\b(SHCS|BHCS|FHCS|screw|bolt|nut|washer|insert|grub|dowel|pin|circlip|e-clip|rivet)\b", re.I)


def parse_ref(ref: str):
    """'4 x M3x10 SHCS; 608ZZ' -> [(4, 'M3x10 SHCS'), (1, '608ZZ')]."""
    out = []
    for item in (ref or "").split(";"):
        item = item.strip()
        if not item:
            continue
        m = MULT_RE.match(item)
        out.append((int(m.group(1)), m.group(2).strip()) if m else (1, item))
    return out


def category(ref: str) -> str:
    if BEARING_RE.search(ref):
        return "bearings"
    if FASTENER_RE.search(ref):
        return "fasteners"
    return "other"


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
            b = bought.setdefault(ref, dict(ref=ref, category=category(ref), count=0, used_by=[]))
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
    bl = sorted(bought.values(), key=lambda b: (cats.index(b["category"]), b["ref"]))
    return dict(printed=printed, bought=bl, totals=totals, unreferenced=unref)


def write(out: str, bom: dict, title="Bill of materials"):
    with open(os.path.join(out, "BOM.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["section", "name_or_ref", "subsystem_or_category", "material", "qty", "mass_g_each", "mass_g_total", "notes_or_used_by"])
        for r in bom["printed"]:
            w.writerow(["printed", r["name"], r["subsystem"], r["material"], r["qty"], r["mass_g"], r["mass_total_g"], r["notes"]])
        for b in bom["bought"]:
            w.writerow(["bought", b["ref"], b["category"], "", b["count"], "", "", " ".join(b["used_by"])])
        for mat, t in bom["totals"].items():
            w.writerow(["total", mat, "", mat, t["count"], "", t["mass_g"], f"{t['filament_m']} m of {FIL_D} mm filament"])
    L = [f"# {title}", "", "## Printed parts", "", "| part | subsystem | material | qty | mass g each | mass g total | notes |",
         "|---|---|---|---|---|---|---|"]
    for r in bom["printed"]:
        L.append(f"| {r['name']} | {r['subsystem']} | {r['material']} | {r['qty']} | {r['mass_g']:.1f} | {r['mass_total_g']:.1f} | {r['notes']} |")
    L += ["", "## Printed totals", "", "| material | parts | mass g | filament m (1.75 mm) |", "|---|---|---|---|"]
    for mat, t in bom["totals"].items():
        L.append(f"| {mat} | {t['count']} | {t['mass_g']:.0f} | {t['filament_m']:.1f} |")
    L += ["", "## Bought parts", ""]
    for cat in ("bearings", "fasteners", "other"):
        rows = [b for b in bom["bought"] if b["category"] == cat]
        if not rows:
            continue
        L += [f"### {cat.capitalize()}", "", "| item | count | used by |", "|---|---|---|"]
        L += [f"| {b['ref']} | {b['count']} | {', '.join(b['used_by'])} |" for b in rows]
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
