#!/usr/bin/env python3
"""Build the GrappStar build guide (Markdown + PDF) from the docs in the repo.

Run from anywhere:  python3 docs/build/make_guide.py
Needs: reportlab (PDF), pypdf (appendices, optional), graphviz `dot` (diagram, optional).
Outputs: docs/build/GrappStar_Build_Guide.md, docs/build/GrappStar_Build_Guide.pdf,
         docs/build/system_electrical.svg / .png.

The section texts live in docs/build/src/NN_*.md. Lines in them that start with '@' are directives:
  @include <repo path> [optional]   insert that file (headings pushed down under the current section)
  @image <repo path> <caption>      insert a picture (PNG or JPG)
  @includeglob <glob> [first]       insert every file (or the first) matching the glob
  @imageglob <glob> <caption>       insert every picture matching the glob
  @exists <repo path>               a 'present' note, or the placeholder
  @ballistics                       range numbers from base/out/ballistics.json
  @csv <repo path>                  insert a CSV file as a table
  @wiremap                          dart run sheet wire numbers -> new cut list numbers
  @printlist base | dart            print tables from base/pack and kedge/pack_v30
  @assumed                          numbers still assumed in base/spec/params.py
  @status                           which source files exist right now
  @date                             build date and git commit
A missing file becomes a short placeholder that says where the content comes from, so the guide always builds.
"""
import csv
import datetime
import glob
import json
import os
import re
import runpy
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SRC = os.path.join(HERE, "src")
OUT_MD = os.path.join(HERE, "GrappStar_Build_Guide.md")
OUT_PDF = os.path.join(HERE, "GrappStar_Build_Guide.pdf")
TITLE = "GrappStar build guide"

# Files the guide pulls in, with the worker that writes them (for the status table and placeholders).
SOURCES = [
    ("base/spec/BASE_SPEC.md", "coordinator"),
    ("base/pack", "cad-infra (print pack)"),
    ("base/out", "cad-infra (assembly pipeline)"),
    ("base/out/base/guide.pdf", "cad-infra (base assembly guide, final run about 18:00)"),
    ("base/out/base/pack/PRINT_SHEET.md", "cad-infra (base print sheet, final run about 18:00)"),
    ("base/out/base/BOM.md", "cad-infra (bill of materials)"),
    ("base/pack/fit_coupon/COUPON.md", "cad-infra (base fit coupon)"),
    ("base/pack/coupons/COUPON_TEST.md", "cad-infra (risk coupons)"),
    ("kedge/pack_v30/KEDGE_v30_run_sheet.pdf", "dart v30 pack (done)"),
    ("firmware/base/WIRING.md", "fw-base"),
    ("firmware/base/README.md", "fw-base"),
    ("firmware/base/CALIBRATION.md", "fw-base"),
    ("firmware/base/BENCH_TEST.md", "fw-base"),
    ("firmware/base/PROTOCOL.md", "fw-base"),
    ("firmware/dart/WIRING.md", "fw-dart"),
    ("firmware/dart/README.md", "fw-dart"),
    ("firmware/dart/API.md", "fw-dart"),
    ("firmware/dart/BENCH_TEST.md", "fw-dart"),
    ("jetson/README.md", "jetson"),
]


# ----------------------------------------------------------------------------------------------- text helpers
def clean(text):
    """House rules: no em dashes. En dashes become plain hyphens. Smart quotes become plain."""
    text = text.replace(" \u2014 ", ", ").replace("\u2014", ", ")
    text = text.replace("\u2013", "-").replace("\u2018", "'").replace("\u2019", "'")
    text = text.replace("\u201c", '"').replace("\u201d", '"')
    # no names on documents (the GitHub account in a URL is an address, not a name)
    text = text.replace("Harry must measure", "What to measure").replace("Harry must confirm", "To confirm")
    text = re.sub(r"\bHarry's\b", "your", text)
    text = re.sub(r"\bHarry\b(?!HYu)", "you", text)
    return text


def rel(path):
    return os.path.relpath(path, ROOT)


def exists(p):
    if any(ch in p for ch in "*?["):
        return bool(glob.glob(os.path.join(ROOT, p), recursive=True))
    return os.path.exists(os.path.join(ROOT, p))


def placeholder(path, who):
    return (f"\n> NOT IN THE REPO YET: `{path}` (written by {who}). Re-run `python3 docs/build/make_guide.py` "
            f"after it lands and this section fills in.\n")


def who_writes(path):
    for p, w in SOURCES:
        if path.startswith(p):
            return w
    return "another worker"


def demote(md, base_level, src_dir):
    """Push headings down so an included file's '#' sits at base_level; fix relative image paths."""
    out, in_code = [], False
    levels = [len(m.group(1)) for m in re.finditer(r"^(#+)\s", md, re.M)]
    top = min(levels) if levels else 1
    for line in md.splitlines():
        if line.strip().startswith("```"):
            in_code = not in_code
        m = None if in_code else re.match(r"^(#+)\s+(.*)$", line)
        if m:
            lvl = min(6, len(m.group(1)) - top + base_level)
            line = "#" * lvl + " " + m.group(2)
        if not in_code:
            line = re.sub(r"!\[([^\]]*)\]\((?!http)([^)]+)\)",
                          lambda mm: f"![{mm.group(1)}]({rel(os.path.join(src_dir, mm.group(2)))})", line)
        out.append(line)
    return "\n".join(out)


# ----------------------------------------------------------------------------------------------- directives
def d_include(args, level):
    path = args[0]
    optional = "optional" in args[1:]
    full = os.path.join(ROOT, path)
    if not os.path.exists(full):
        return "" if optional else placeholder(path, who_writes(path))
    with open(full, encoding="utf-8") as f:
        md = f.read()
    return f"\n*From `{path}`:*\n\n" + demote(md, level + 1, os.path.dirname(full)) + "\n"


def d_image(args, level):
    path, caption = args[0], " ".join(args[1:])
    if not exists(path):
        return placeholder(path, who_writes(path))
    return f"\n![{caption}]({path})\n"


def _table(rows, header):
    if not rows:
        return ""
    s = "| " + " | ".join(header) + " |\n|" + "---|" * len(header) + "\n"
    for r in rows:
        s += "| " + " | ".join(str(c).replace("|", "/") for c in r) + " |\n"
    return s


def printlist_dart():
    rep = os.path.join(ROOT, "kedge/pack_v30/print_report.json")
    if not os.path.exists(rep):
        return placeholder("kedge/pack_v30/print_report.json", "dart v30 pack")
    d = json.load(open(rep))
    rows = []
    for name, p in d.get("parts", {}).items():
        bb = " x ".join(f"{v:.0f}" for v in p.get("print_bbox", []))
        rows.append((name, p.get("orient", ""), bb, p.get("volume_cm3", "")))
    plates = sorted(glob.glob(os.path.join(ROOT, "kedge/pack_v30/*.3mf")))
    s = "Dart plates (in `kedge/pack_v30/`, also inside `kedge/dist_v30/KEDGE_v30_build_pack.zip` under `print/`):\n\n"
    s += "".join(f"- `{rel(p)}`\n" for p in plates)
    s += "- `kedge/pack_v30/fit_coupon.stl` (print this first, 20 minutes)\n\n"
    s += _table(rows, ["Dart part", "Print pose", "Size on bed mm", "Volume cm3"])
    s += ("\nSettings: PLA, 0.4 nozzle, 0.16 mm layers, 4 walls, 40% gyroid infill, 5 top and bottom layers, "
          "supports off, brim on for the augers and couplers. Hole compensation from the fit coupon (run sheet section 4).\n")
    return s


def printlist_base():
    pack = next((os.path.join(ROOT, c) for c in ("base/out/base/pack", "base/pack")
                 if os.path.isdir(os.path.join(ROOT, c)) and any(
                     f for f in os.listdir(os.path.join(ROOT, c)) if f not in ("coupons", "fit_coupon"))),
                os.path.join(ROOT, "base/pack"))
    if not os.path.isdir(pack):
        return placeholder("base/pack", "cad-infra (print pack)")
    s = ""
    # A print list table, whatever form the pack writes it in.
    for name in ("PRINT_SHEET.md", "print_list.md", "PRINT_LIST.md", "README.md"):
        p = os.path.join(pack, name)
        if os.path.exists(p):
            s += demote(open(p, encoding="utf-8").read(), 3, pack) + "\n"
            break
    else:
        for name in ("print_list.csv", "parts.csv"):
            p = os.path.join(pack, name)
            if os.path.exists(p):
                rows = list(csv.reader(open(p, encoding="utf-8")))
                s += _table(rows[1:], rows[0]) + "\n"
                break
        else:
            for name in ("print_list.json", "print_report.json", "pack.json"):
                p = os.path.join(pack, name)
                if os.path.exists(p):
                    d = json.load(open(p))
                    parts = d.get("parts", d)
                    rows = []
                    if isinstance(parts, dict):
                        for k, v in parts.items():
                            v = v if isinstance(v, dict) else {}
                            rows.append((k, v.get("material", ""), v.get("plate", ""), v.get("orient", "")))
                    elif isinstance(parts, list):
                        for v in parts:
                            rows.append((v.get("name", ""), v.get("material", ""), v.get("plate", ""),
                                         v.get("orient", "")))
                    s += _table(rows, ["Base part", "Material", "Plate", "Print pose"]) + "\n"
                    break
    plates = [p for p in sorted(glob.glob(os.path.join(pack, "**/*.3mf"), recursive=True))
              if os.sep + "coupons" + os.sep not in p]
    if plates:
        s += "\nBase plates:\n\n" + "".join(f"- `{rel(p)}`\n" for p in plates)
    stls = [p for p in sorted(glob.glob(os.path.join(pack, "**/*.stl"), recursive=True))
            if os.sep + "coupons" + os.sep not in p]
    if stls and not s.strip():
        s += "\nBase STL files:\n\n" + "".join(f"- `{rel(p)}`\n" for p in stls)
    return s or placeholder("base/pack (print list)", "cad-infra (print pack)")


def d_includeglob(args, level):
    """Include every file matching a glob (recursive); placeholder if none."""
    hits = sorted(glob.glob(os.path.join(ROOT, args[0]), recursive=True))
    if not hits:
        return placeholder(args[0], who_writes(args[0]))
    if "first" in args[1:]:
        hits = hits[:1]
    return "".join(d_include([rel(h)], level) for h in hits)


def d_imageglob(args, level):
    """Every picture matching a glob, with the caption given (the file name is added)."""
    hits = sorted(glob.glob(os.path.join(ROOT, args[0]), recursive=True))
    caption = " ".join(args[1:])
    return "".join(f"\n![{caption} ({os.path.basename(h)})]({rel(h)})\n" for h in hits)


def d_exists(args, level):
    """A note that a file is present (and where it is), or the placeholder when it is not."""
    path = args[0]
    if exists(path):
        return f"\nPresent: `{path}`" + (" (attached at the end of this guide)." if path.endswith(".pdf") else ".") + "\n"
    return placeholder(path, who_writes(path))


def d_ballistics(args, level):
    """One line of range numbers from base/out/ballistics.json (contract C3), so the guide never goes stale."""
    p = os.path.join(ROOT, "base/out/ballistics.json")
    if not os.path.exists(p):
        return placeholder("base/out/ballistics.json", "physics")
    d = json.load(open(p))
    rows = [(r["pitch_deg"], r["range_m"][0], r.get("v_muzzle")) for r in d.get("table", [])]
    if not rows:
        return "(ballistics table empty)"
    lo, hi = min(rows, key=lambda r: r[1]), max(rows, key=lambda r: r[1])
    brake = d.get("drag_n", [0.5])[0]
    v = f", leaving the rail at about {hi[2]:.1f} m/s" if hi[2] else ""
    return (f"Range today (from `base/out/ballistics.json`, {brake} N payout brake, measured from the ground point under "
            f"the rail pivot): best about {hi[1]:.1f} m at {hi[0]:g} degrees, shortest about {lo[1]:.1f} m at {lo[0]:g} "
            f"degrees{v}. These numbers change after the spring and wiper measurements; rebuild the guide.")


def d_csv(args, level):
    path = args[0]
    full = os.path.join(ROOT, path)
    if not os.path.exists(full):
        return placeholder(path, who_writes(path))
    rows = [r for r in csv.reader(open(full, encoding="utf-8")) if r]
    return f"\n*From `{path}`:*\n\n" + _table(rows[1:], rows[0])


def d_wiremap(args, level):
    """Run sheet (v29) wire number -> new dart wire number, from the new cut list's 'v29 #' column."""
    new_p = os.path.join(ROOT, "firmware/dart/wiring/wire_cut_list.csv")
    old_p = os.path.join(ROOT, "kedge/pack_v30/wire_cut_list.csv")
    if not (os.path.exists(new_p) and os.path.exists(old_p)):
        return placeholder("firmware/dart/wiring/wire_cut_list.csv", "fw-dart")
    new = list(csv.DictReader(open(new_p, encoding="utf-8")))
    old = list(csv.DictReader(open(old_p, encoding="utf-8")))
    by_old = {}
    for r in new:
        for o in re.findall(r"\d+", r.get("v29 #", "") or ""):
            by_old[o] = r["#"]
    rows = []
    for r in old:
        n = by_old.get(r["#"], "REMOVED (no DRV#3)")
        rows.append((r["#"], n, f"{r['from']} to {r['to']}"))
    return _table(rows, ["Run sheet wire", "New wire", "Run sheet says"])


def d_printlist(args, level):
    return printlist_base() if args[0] == "base" else printlist_dart()


def d_assumed(args, level):
    p = os.path.join(ROOT, "base/spec/params.py")
    if not os.path.exists(p):
        return placeholder("base/spec/params.py", "coordinator")
    try:
        g = runpy.run_path(p)
    except Exception as e:  # never let a bad params file stop the guide
        return f"\n(params.py did not load: {e})\n"
    rows = []
    for k, v in g.items():
        if k.isupper() and isinstance(v, dict) and v.get("measured") is False:
            rows.append((k, "assumed, not measured yet", "MEASURE_LIST"))
        if k.isupper() and v is None:
            rows.append((k, "not decided yet (None)", "coordinator"))
    s = f"\nparams.py version: `{g.get('VERSION', '?')}`.\n\n"
    return s + (_table(rows, ["params.py name", "State", "Who fixes it"]) if rows else "Nothing assumed.\n")


def d_status(args, level):
    rows = [(f"`{p}`", w, "yes" if exists(p) else "NOT YET") for p, w in SOURCES]
    return "\n" + _table(rows, ["Source", "Written by", "In the repo"])


def d_date(args, level):
    try:
        commit = subprocess.check_output(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"], text=True).strip()
    except Exception:
        commit = "unknown"
    from zoneinfo import ZoneInfo
    now = datetime.datetime.now(ZoneInfo("Australia/Sydney"))
    return f"\nBuilt {now.strftime('%Y-%m-%d %H:%M')} Sydney time from commit `{commit}`.\n"


DIRECTIVES = {"include": d_include, "ballistics": d_ballistics, "exists": d_exists, "includeglob": d_includeglob, "imageglob": d_imageglob, "csv": d_csv, "wiremap": d_wiremap, "image": d_image, "printlist": d_printlist, "assumed": d_assumed,
              "status": d_status, "date": d_date}


def expand(md):
    out, level, in_code = [], 1, False
    for line in md.splitlines():
        if line.strip().startswith("```"):
            in_code = not in_code
        m = re.match(r"^(#+)\s", line)
        if m and not in_code:
            level = len(m.group(1))
        if line.startswith("@") and not in_code:
            parts = line[1:].split()
            fn = DIRECTIVES.get(parts[0])
            if fn:
                out.append(fn(parts[1:], level))
                continue
        out.append(line)
    return "\n".join(out)


def build_markdown():
    files = sorted(glob.glob(os.path.join(SRC, "[0-9][0-9]_*.md")))
    parts = []
    for f in files:
        parts.append(expand(open(f, encoding="utf-8").read()))
    return clean("\n\n".join(parts)).rstrip() + "\n"


# ----------------------------------------------------------------------------------------------- diagram
PINS_H = os.path.join(ROOT, "firmware/base/grappstar_base/pins.h")


def read_pins():
    """{'PIN_RPWM': 4, ...} from fw-base's pins.h (the GPIO owner)."""
    pins = {}
    if os.path.exists(PINS_H):
        for m in re.finditer(r"^#define\s+(PIN_\w+)\s+(\d+)", open(PINS_H).read(), re.M):
            pins[m.group(1)] = int(m.group(2))
    return pins


def render_diagram():
    dot = shutil.which("dot")
    src = os.path.join(HERE, "system_electrical.dot")
    if not dot or not os.path.exists(src):
        print("note: graphviz 'dot' not found, keeping the existing diagram files")
        return
    pins = read_pins()
    text = open(src, encoding="utf-8").read()
    text = re.sub(r"\{(PIN_\w+)\}", lambda m: str(pins.get(m.group(1), "?")), text)
    filled = os.path.join(HERE, ".system_electrical.filled.dot")
    with open(filled, "w", encoding="utf-8") as f:
        f.write(text)
    subprocess.run([dot, "-Tsvg", filled, "-o", os.path.join(HERE, "system_electrical.svg")], check=True)
    subprocess.run([dot, "-Tpng", "-Gdpi=130", filled, "-o", os.path.join(HERE, "system_electrical.png")], check=True)


# ----------------------------------------------------------------------------------------------- PDF
def build_pdf(md, out=None, doc_title=TITLE):
    from reportlab.lib import colors
    from reportlab.lib.enums import TA_LEFT
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles import ParagraphStyle
    from reportlab.lib.units import mm
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont
    from reportlab.platypus import (Image, KeepTogether, PageBreak, Paragraph, Preformatted, Spacer, Table,
                                    TableStyle)
    from reportlab.platypus.doctemplate import BaseDocTemplate, Frame, PageTemplate
    from reportlab.platypus.tableofcontents import TableOfContents

    fdir = "/usr/share/fonts/truetype/liberation"
    for name, fn in (("Sans", "LiberationSans-Regular.ttf"), ("Sans-Bold", "LiberationSans-Bold.ttf"),
                     ("Sans-Italic", "LiberationSans-Italic.ttf"), ("Sans-BoldItalic", "LiberationSans-BoldItalic.ttf"),
                     ("Mono", "LiberationMono-Regular.ttf")):
        pdfmetrics.registerFont(TTFont(name, os.path.join(fdir, fn)))
    from reportlab.pdfbase.pdfmetrics import registerFontFamily
    registerFontFamily("Sans", normal="Sans", bold="Sans-Bold", italic="Sans-Italic", boldItalic="Sans-BoldItalic")
    from reportlab import rl_config
    rl_config.canvas_basefontname = "Sans"     # no non-embedded Helvetica anywhere in the PDFs

    k = colors.black
    body = ParagraphStyle("body", fontName="Sans", bulletFontName="Sans", fontSize=9.5, leading=12.5, textColor=k, spaceAfter=4,
                          alignment=TA_LEFT)
    small = ParagraphStyle("small", parent=body, fontSize=8, leading=10, spaceAfter=0)
    small_b = ParagraphStyle("small_b", parent=small, fontName="Sans-Bold")
    quote = ParagraphStyle("quote", parent=body, leftIndent=8 * mm, fontName="Sans-Italic")
    title = ParagraphStyle("title", parent=body, fontName="Sans-Bold", fontSize=18, leading=22, spaceAfter=8)
    hs = {1: ParagraphStyle("h1", parent=body, fontName="Sans-Bold", fontSize=15, leading=19, spaceBefore=4,
                            spaceAfter=6),
          2: ParagraphStyle("h2", parent=body, fontName="Sans-Bold", fontSize=12, leading=15, spaceBefore=8,
                            spaceAfter=4),
          3: ParagraphStyle("h3", parent=body, fontName="Sans-Bold", fontSize=10.5, leading=13, spaceBefore=6,
                            spaceAfter=3)}
    for i in (4, 5, 6):
        hs[i] = ParagraphStyle(f"h{i}", parent=body, fontName="Sans-Bold", fontSize=9.5, leading=12,
                               spaceBefore=4, spaceAfter=2)
    code = ParagraphStyle("code", fontName="Mono", fontSize=7.5, leading=9, textColor=k, leftIndent=4 * mm,
                          spaceAfter=5)
    toc_styles = [ParagraphStyle("toc1", parent=body, fontSize=10, leading=13),
                  ParagraphStyle("toc2", parent=body, fontSize=9, leading=11, leftIndent=6 * mm)]

    def inline(t):
        t = t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        t = re.sub(r"`([^`]+)`", r'<font name="Mono">\1</font>', t)
        t = re.sub(r"\*\*([^*]+)\*\*", r"<b>\1</b>", t)
        t = re.sub(r"(?<![\w*])\*([^*\s][^*]*?)\*(?![\w*])", r"<i>\1</i>", t)
        t = re.sub(r"\[([^\]]+)\]\((http[^)]+)\)", r"\1 (\2)", t)
        t = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", r"\1", t)
        return t

    W = A4[0] - 36 * mm

    class Doc(BaseDocTemplate):
        def __init__(self, fn):
            super().__init__(fn, pagesize=A4, leftMargin=18 * mm, rightMargin=18 * mm, topMargin=16 * mm,
                             bottomMargin=16 * mm, title=doc_title, author="", subject="", creator="")
            frame = Frame(self.leftMargin, self.bottomMargin, self.width, self.height, id="f")
            self.addPageTemplates([PageTemplate(id="p", frames=[frame], onPage=self.footer)])

        def footer(self, c, d):
            c.setFont("Sans", 8)
            c.drawRightString(A4[0] - 18 * mm, 9 * mm, str(d.page))
            c.drawString(18 * mm, 9 * mm, doc_title)

        def afterFlowable(self, f):
            if isinstance(f, Paragraph) and getattr(f, "_toc", 0):
                lvl = f._toc - 1
                key = f"h{id(f)}"
                self.canv.bookmarkPage(key)
                self.canv.addOutlineEntry(f.getPlainText(), key, level=lvl, closed=lvl > 0)
                if not getattr(f, "_outline_only", False):
                    self.notify("TOCEntry", (lvl, f.getPlainText(), self.page, key))

    def table_flow(rows):
        ncol = max(len(r) for r in rows)
        rows = [r + [""] * (ncol - len(r)) for r in rows]
        def plain(c):
            return re.sub(r"[`*]", "", c)
        lens = [max(4, min(60, max(len(plain(r[i])) for r in rows))) for i in range(ncol)]
        # never narrower than the longest word in the column (about 0.55 em per character at 8 pt)
        def wmin(c):
            words = plain(c).split() or [""]
            return max(len(w) for w in words) * (5.0 if "`" in c else 4.6) + 8
        mins = [max(wmin(r[i]) for r in rows) for i in range(ncol)]
        tot = sum(lens)
        widths = [W * l / tot for l in lens]
        for _ in range(5):
            short = [i for i in range(ncol) if widths[i] < mins[i]]
            if not short:
                break
            need = sum(mins[i] - widths[i] for i in short)
            free = [i for i in range(ncol) if i not in short]
            spare = sum(widths[i] - mins[i] for i in free if widths[i] > mins[i])
            if spare <= 0:
                break
            f = min(1.0, need / spare)
            for i in free:
                if widths[i] > mins[i]:
                    widths[i] -= (widths[i] - mins[i]) * f
            for i in short:
                widths[i] = mins[i]
        data = [[Paragraph(inline(c), small_b if ri == 0 else small) for c in r] for ri, r in enumerate(rows)]
        t = Table(data, colWidths=widths, repeatRows=1)
        t.setStyle(TableStyle([("FONTNAME", (0, 0), (-1, -1), "Sans"), ("GRID", (0, 0), (-1, -1), 0.4, k), ("VALIGN", (0, 0), (-1, -1), "TOP"),
                               ("LEFTPADDING", (0, 0), (-1, -1), 3), ("RIGHTPADDING", (0, 0), (-1, -1), 3),
                               ("TOPPADDING", (0, 0), (-1, -1), 2), ("BOTTOMPADDING", (0, 0), (-1, -1), 2)]))
        return t

    def image_flow(path, caption):
        full = os.path.join(ROOT, path)
        try:
            from reportlab.lib.utils import ImageReader
            iw, ih = ImageReader(full).getSize()
        except Exception:
            return [Paragraph(inline(f"(picture missing: {path})"), body)]
        maxh = 200 * mm
        if iw > 1.6 * ih and iw > 1500:
            # wide diagram: turn it a quarter turn and give it a page of its own
            from PIL import Image as PILImage
            rot = os.path.join(HERE, ".rot_" + os.path.basename(full))
            PILImage.open(full).rotate(90, expand=True).save(rot)
            iw, ih = ih, iw
            s = min(W / iw, 235 * mm / ih)
            return [PageBreak(), Image(rot, width=iw * s, height=ih * s), Paragraph(inline(caption), small),
                    PageBreak()]
        s = min(W / iw, maxh / ih)
        return [Image(full, width=iw * s, height=ih * s), Paragraph(inline(caption), small), Spacer(1, 4)]

    story = []
    toc = TableOfContents()
    toc.levelStyles = toc_styles
    toc.tableStyle = TableStyle([("FONTNAME", (0, 0), (-1, -1), "Sans"), ("VALIGN", (0, 0), (-1, -1), "TOP"),
                                 ("LEFTPADDING", (0, 0), (-1, -1), 0), ("RIGHTPADDING", (0, 0), (-1, -1), 0)])
    lines = md.splitlines()
    i, para, first_h1 = 0, [], True

    def flush():
        if para:
            story.append(Paragraph(inline(" ".join(para)), body))
            para.clear()

    while i < len(lines):
        line = lines[i]
        s = line.strip()
        if s.startswith("```"):
            flush()
            j = i + 1
            buf = []
            while j < len(lines) and not lines[j].strip().startswith("```"):
                buf.append(lines[j])
                j += 1
            story.append(Preformatted("\n".join(buf), code, maxLineLength=104, newLineChars="  "))
            i = j + 1
            continue
        m = re.match(r"^(#+)\s+(.*)$", line)
        if m:
            flush()
            lvl = len(m.group(1))
            txt = m.group(2).strip()
            if lvl == 1 and first_h1:
                tp = Paragraph(inline(txt), title)
                tp._toc, tp._outline_only = 1, True
                story.append(tp)
                first_h1 = False
            else:
                if lvl == 1:
                    story.append(PageBreak())
                p = Paragraph(inline(txt), hs[lvl])
                if lvl <= 2:
                    p._toc = lvl
                story.append(p)
            i += 1
            continue
        if s == "@@toc":
            flush()
            story.append(toc)
            i += 1
            continue
        if s == "@@pagebreak":
            flush()
            story.append(PageBreak())
            i += 1
            continue
        mi = re.match(r"^!\[([^\]]*)\]\(([^)]+)\)\s*$", s)
        if mi:
            flush()
            story.extend(image_flow(mi.group(2), mi.group(1)))
            i += 1
            continue
        if s.startswith("|"):
            flush()
            rows = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                r = lines[i].strip()
                if not re.match(r"^\|[\s:|-]+\|$", r):
                    rows.append([c.strip() for c in r.strip("|").split("|")])
                i += 1
            if rows:
                story.append(table_flow(rows))
                story.append(Spacer(1, 5))
            continue
        ml = re.match(r"^(\s*)([-*]|\d+\.)\s+(.*)$", line)
        if ml:
            flush()
            indent = len(ml.group(1)) // 2
            bullet = ml.group(2) if ml.group(2)[0].isdigit() else "-"
            txt = ml.group(3)
            j = i + 1
            while j < len(lines) and lines[j].startswith("  ") and lines[j].strip() and \
                    not re.match(r"^\s*([-*]|\d+\.)\s+", lines[j]):
                txt += " " + lines[j].strip()
                j += 1
            st = ParagraphStyle("li", parent=body, leftIndent=(6 + 5 * indent) * mm, bulletIndent=(1 + 5 * indent) * mm,
                                spaceAfter=2)
            story.append(Paragraph(inline(txt), st, bulletText=bullet))
            i = j
            continue
        if s.startswith(">"):
            flush()
            txt = s.lstrip("> ").strip()
            j = i + 1
            while j < len(lines) and lines[j].strip().startswith(">"):
                txt += " " + lines[j].strip().lstrip("> ").strip()
                j += 1
            story.append(Paragraph(inline(txt), quote))
            i = j
            continue
        if not s:
            flush()
            i += 1
            continue
        para.append(s)
        i += 1
    flush()

    tmp = out or (OUT_PDF + ".main.pdf")
    doc = Doc(tmp)
    from reportlab.pdfgen.canvas import Canvas

    def sans_canvas(*a, **k):
        k.setdefault("initialFontName", "Sans")   # reportlab starts each page in Helvetica otherwise (not embedded)
        return Canvas(*a, **k)
    doc.multiBuild(story, canvasmaker=sans_canvas)
    return tmp


APPENDICES = [
    # (repo path or glob, bookmark title). Missing files are skipped.
    ("base/out/base/guide.pdf", "Appendix: base assembly guide (base pipeline)"),
    ("base/guide/*.pdf", "Appendix: base assembly steps (base pipeline)"),
]


def append_pdfs(main):
    try:
        from pypdf import PdfReader, PdfWriter
    except ImportError:
        print("note: pypdf not installed, appendices not attached")
        shutil.move(main, OUT_PDF)
        return []
    w = PdfWriter()
    w.append(PdfReader(main))
    added = []
    for pattern, label in APPENDICES:
        for p in sorted(glob.glob(os.path.join(ROOT, pattern))):
            start = len(w.pages)
            w.append(PdfReader(p))
            w.add_outline_item(f"{label}: {os.path.basename(p)}", start)
            added.append((rel(p), start + 1))
    w.add_metadata({"/Title": TITLE, "/Author": "", "/Creator": "", "/Producer": ""})
    with open(OUT_PDF, "wb") as f:
        w.write(f)
    os.remove(main)
    return added


def first_table_after(path, heading_regex):
    """The first Markdown table after the first heading matching heading_regex (or after a line matching it)."""
    full = os.path.join(ROOT, path)
    if not os.path.exists(full):
        return None
    lines = open(full, encoding="utf-8").read().splitlines()
    start = next((i for i, l in enumerate(lines) if re.search(heading_regex, l)), None)
    if start is None:
        return None
    out = []
    for l in lines[start + 1:]:
        if l.strip().startswith("|"):
            out.append(l)
        elif out:
            break
    return "\n".join(out) if out else None


def build_cheatsheets():
    """One printable page per board: the pin table from the owner's WIRING.md plus the power and safety lines."""
    sheets = []
    base_pins = first_table_after("firmware/base/WIRING.md", r"^## 1\. Pin table")
    base_md = f"""# Base ESP32-S3 wiring cheat sheet

**Battery unplugged before any wire moves.** ESP32-S3 DevKitC-1-N8, powered only by USB (port marked USB) from the
Jetson or the Mac. Pins from `firmware/base/WIRING.md` (that file wins if this page is older).

{base_pins or placeholder("firmware/base/WIRING.md", "fw-base")}

Power: 3S LiPo, XT60 lead #2 (20 A fuse), BTS7960 B+ / B- (14 AWG). Wiper on M+ (red) / M- (black), 14 AWG.
MP1584 #2 from the B+ / B- screws through a 2 A inline fuse, set to 9.0 V BEFORE its output is connected, out to
DRV8833 VM / GND. DRV8833 #3: only AIN1, AIN2, GND go to the ESP32; SLP is not wired (decision D9).

Resistors: 10 k R_EN to GND, 10 k L_EN to GND, IS: R_IS + L_IS tied, 1 k to GPIO1, 680 R GPIO1 to GND. Battery:
100 k at the B+ screw to GPIO2, 22 k GPIO2 to GND. Common ground: ESP32 GND, BTS7960 GND, DRV8833 GND, MP1584 OUT-.

Never: the 3S on the DRV8833 VM (10.8 V max); 3V3 on a pad that beeps to VM; a plug pulled with the battery in.
"""
    dart_pins = first_table_after("firmware/dart/WIRING.md", r"^Pin map")
    dart_md = f"""# Dart XIAO ESP32S3 wiring cheat sheet

**Power loop plug out before any wire moves or USB goes in.** Pins from `firmware/dart/WIRING.md` (v4, DRV#3
removed; that file wins if this page is older). Full wire list: `firmware/dart/wiring/wire_cut_list.csv`.

{dart_pins or placeholder("firmware/dart/WIRING.md", "fw-dart")}

Power: 2 x 18350 in series (2S, 6.0 to 8.4 V), OTOTEC BMS, JST-XH loop plug = ON switch, INA219 VIN+ then the motor
rail to DRV#1 VM and DRV#2 VM (no buck for motors). MP1584 #1 at 5.04 V taps INA219 VIN+: OUT through a Schottky
diode (stripe toward the XIAO) to XIAO 5V, and to the VTX and camera. Capacitor across MP1584 OUT (long leg OUT+).

DRV8833: 3V3 only to the SLP / nSLEEP / EEP pad (check it does NOT beep to VM). DRV#2 channel A drives both drills
(right drill leads swapped); channel B drives the foot. No bridge between pads.

Reverse a motor: swap its two input wires at the driver (drills D8 / D9, foot D6 / D7, deploy D0 / D1, kick
D2 / D3). One drill wrong only: swap that drill's motor leads. VTX antenna on before any power.
"""
    cmd_md = """# Commands cheat sheet

On the Jetson, in `~/GrappStar/jetson`. One program on the base port at a time. Full lists: `firmware/base/PROTOCOL.md`, `firmware/dart/API.md`,
`jetson/README.md`. If a command here and those files disagree, those files win.

**Stop:** 1. `estop` (or STOP on the operator page). 2. Pull lead #2's XT60 (motors off). 3. Main battery.
Cutting power does NOT release a drawn spring or a taut line. **Before walking up:** `estop`, lead #2 out, `cock` 0
and `H` closed, line slack, dart IDLE or its loop plug out. Never fire without the tether. Stand beside, never in
line with, a towing line.

## Jetson

| Command | What it does |
|---|---|
| `python3 -m grappstar serve` | operator page at http://<jetson>:8080 (`--sim` for no hardware) |
| `python3 -m grappstar base ping` / `status` / `monitor` | talk to the base; `monitor` shows live telemetry (Ctrl+C stops) |
| `python3 -m grappstar base console` | live telemetry and events; type commands (JSON, `mode to=REEL`, `stop`); `quit` |
| `python3 -m grappstar base cmd '<json>'` | send one base command (below) |
| `python3 -m grappstar base mode LAUNCH` / `base aim 30` / `base fire` | shift, aim, fire (fire asks you to type FIRE) |
| `python3 -m grappstar dart status` | one dart status reading |
| `python3 -m grappstar dart key 2 --hold 1.0` | hold a dart key for 1 s (quote `'!'`, `'#'`, `'$'`) |
| `python3 -m grappstar dart auto` / `release` / `flightlog` | anchor, release, download the flight log |
| `python3 -m grappstar cameras` | list the cameras |
| `sudo shutdown now` | before unplugging the battery |

## Base commands (JSON, for `base cmd`)

| Command | What it does |
|---|---|
| `{"cmd":"estop"}` | wiper off, drum to NEUTRAL when safe. Always safe to send |
| `{"cmd":"stop"}` | wiper off, drum stays |
| `{"cmd":"clear"}` | clear a fault once its cause is gone |
| `{"cmd":"mode","to":"LAUNCH"}` | drum to LAUNCH, NEUTRAL or REEL |
| `{"cmd":"aim","deg":30}` | rail to 30 degrees (only with the carriage home) |
| `python3 -m grappstar base fire` (Jetson CLI) | arm and fire in one go after you type FIRE. Tether on, area clear |
| `{"cmd":"decock"}` | unload a cocked carriage slowly (`"force":true` if SW_HOME is broken) |
| `{"cmd":"wrap"}` | lay 7 wraps, right after landing (from LAUNCH or NEUTRAL; parks at WRAPPED) |
| `{"cmd":"tidy"}` | take up the slack after the dart anchors |
| `{"cmd":"unwrap"}` | remove the wraps (after reeling in to about 1.5 m) |
| `{"cmd":"retrieve","duty":0.3}` | final approach on the take-up until the dart seats |
| `{"cmd":"reel","duty":0.5,"until":"seated"}` | wind in until the dart seats (`until`: stop, seated, tension) |
| `{"cmd":"jog","motor":"wiper","dir":1,"ms":300}` | short move; `motor` wiper or drum, `dir` 1 or -1, up to 1500 ms |
| `{"cmd":"set","key":"limit_n","val":40}` | set a stored value (tow limit; 40 N or less for the first field tows) |
| `{"cmd":"get","key":"limit_n"}` | read a stored value |

## Dart keys (WiFi KEDGE, password grappstar, http://192.168.4.1, or Serial 115200)

| Key | Motor |
|---|---|
| 1 / ! | deploy pods down / up |
| 2 / @ | kick out / back |
| 3 / # | drills in / out |
| 4 / $ | foot in / out |
| space | STOP everything |
| a / A | auto anchor now / arm auto-anchor on landing |
| i / b / z | current / attitude / I2C scan |
"""
    for name, md, title in (("Wiring_Cheat_Sheet_Base.pdf", base_md, "Base ESP32-S3 wiring"),
                            ("Wiring_Cheat_Sheet_Dart.pdf", dart_md, "Dart XIAO wiring"),
                            ("Commands_Cheat_Sheet.pdf", cmd_md, "Commands")):
        out = os.path.join(HERE, name)
        build_pdf(clean(md), out=out, doc_title=title)
        sheets.append(rel(out))
    return sheets


def build_results_sheet():
    """MEASURE_RESULTS.md / .pdf: every MEASURE_LIST item with an empty Result column, to print and fill in."""
    src = os.path.join(HERE, "MEASURE_LIST.md")
    out_md = os.path.join(HERE, "MEASURE_RESULTS.md")
    lines = ["# Measure results (fill in)", "",
             "Write each number with its unit. Photo: the file name of the caliper photo in "
             "`originals/measure_2026-11/`. Items are explained in MEASURE_LIST.md.", ""]
    for l in open(src, encoding="utf-8").read().splitlines():
        if l.startswith("## ") and l[3:4] in "ABCDEFGH":
            lines += ["", l, "", "| Item | Result (with unit) .................... | Photo file or note ................ |", "|---|---|---|"]
        m = re.match(r"^\| ([A-H]\d+ [^|]*)\|", l)
        if m:
            item = m.group(1).strip()
            item = item if len(item) < 70 else item[:67].rsplit(" ", 1)[0] + " ..."
            lines.append(f"| {item} |  |  |")
    md = "\n".join(lines) + "\n"
    with open(out_md, "w", encoding="utf-8") as f:
        f.write(md)
    build_pdf(clean(md), out=os.path.join(HERE, "MEASURE_RESULTS.pdf"), doc_title="Measure results")
    return rel(out_md)


def lint(md):
    bad = [n for n, line in enumerate(md.splitlines(), 1) if "\u2014" in line]
    if bad:
        print(f"ERROR: em dash on guide lines {bad[:10]}")
        return False
    return True


def main():
    render_diagram()
    print(f"wrote {build_results_sheet()}")
    md = build_markdown()
    ok = lint(md)
    with open(OUT_MD, "w", encoding="utf-8") as f:
        # image paths in the Markdown file are relative to docs/build so they show on GitHub
        md_out = re.sub(r"!\[([^\]]*)\]\((?!http)([^)]+)\)",
                        lambda m: f"![{m.group(1)}]({os.path.relpath(os.path.join(ROOT, m.group(2)), HERE)})", md)
        f.write(md_out.replace("@@toc\n", "").replace("@@pagebreak\n", ""))
    main_pdf = build_pdf(md)
    added = append_pdfs(main_pdf)
    for sh in build_cheatsheets():
        print(f"wrote {sh}")
    for p, page in added:
        print(f"appended {p} at page {page}")
    missing = [p for p, _ in SOURCES if not exists(p)]
    print(f"wrote {rel(OUT_MD)} and {rel(OUT_PDF)}")
    if missing:
        print("placeholders for: " + ", ".join(missing))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
