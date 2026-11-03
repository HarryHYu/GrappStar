"""Print pack: STL per printed part in print orientation, 3MF plates for the Bambu X2D bed, print sheet.

python -m tools.pack OUT PACK [--only demo] [--spacing 5] [--margin 5]

Reads OUT/asm.json and OUT/meshes/<part>.npz (runs the assembly export first if they are missing) and
OUT/audit.json for the overhang numbers when present. Writes
  PACK/stl/<name>.stl                    one per printed part, print_tf applied, min corner at (0, 0, 0)
  PACK/plates/plate_<n>_<material>.3mf   rectpack on the footprints (5 mm spacing), one material per plate,
                                         qty copies of every part; object names are unique (<part>__c<copy> for
                                         qty > 1) and the build items are counted on reread. A part that fits the
                                         bed but not inside the margin gets a plate of its own, centred (warning)
  PACK/plates/plate_<n>_<material>.png   top-down footprint preview
  PACK/print_report.json                 the numbers below (the guide reads it)
  PACK/PRINT_SHEET.md, PACK/PRINT_SHEET.pdf
Masses come from the volume and surface with walls and infill per material from PROFILE (the same table as the
settings on the print sheet); times count the layer time once per plate. Totals on the plates and the parts that
could not be placed are reported separately. Exit code 2 when a part does not fit the bed (the pack is still
written for the rest).

Also the shared helpers of the output tools (load_out, part_mesh, clean, fonts).
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import sys
import time
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

BED = (256.0, 256.0, 260.0)
FIL_D = 1.75                      # filament diameter mm
SUPPORT_MM2 = 100.0               # overhang area above which supports are suggested
EM_DASH, EN_DASH, FIG_DASH, HBAR = chr(0x2014), chr(0x2013), chr(0x2012), chr(0x2015)
_DASH_COMMA_RE = re.compile(rf"\s*[{EM_DASH}{HBAR}]\s*|\s+[{EN_DASH}{FIG_DASH}]\s+")
MAT_ORDER = ["PLA", "PETG", "PPA-CF", "TPU"]
DENSITY = {"PLA": 1.24, "PPA-CF": 1.25, "TPU": 1.21, "PETG": 1.27}
# rough effective print rate (mm3 of plastic per second, travel included) and time per layer
RATE = {"PLA": 9.0, "PETG": 7.0, "PPA-CF": 5.0, "TPU": 2.5}
LAYER_S = 5.0
LINE_W = 0.42                     # extrusion width of a wall line (0.4 nozzle), for the shell of the mass estimate
# Slicer profile per material: the one table behind the PRINT_SHEET settings and the pack mass and time estimates.
PROFILE = {
    "PLA": dict(walls=4, top=5, bottom=5, infill=0.40),
    "PETG": dict(walls=4, top=5, bottom=5, infill=0.40),
    "PPA-CF": dict(walls=6, top=6, bottom=6, infill=0.60),
    "TPU": dict(walls=3, top=None, bottom=None, infill=0.25),
}


def _walls(mat):
    pr = PROFILE[mat]
    s = f"{pr['walls']} walls"
    return s + (f", {pr['top']} top and {pr['bottom']} bottom layers" if pr["top"] else "")


def _infill(mat):
    return f"{100 * PROFILE[mat]['infill']:.0f} % gyroid infill"


SETTINGS = {
    "PLA": ["0.4 mm nozzle, 0.2 mm layers", _walls("PLA"), _infill("PLA"),
            "Nozzle 220 C, bed 55 to 60 C, textured PEI plate", "Door open or top vented"],
    "PETG": ["0.4 mm nozzle, 0.2 mm layers", _walls("PETG"), _infill("PETG"),
             "Nozzle 245 C, bed 70 C, glue stick on smooth PEI"],
    "PPA-CF": ["Hardened steel nozzle (0.4 or 0.6 mm), never a brass nozzle", "0.2 mm layers",
               _walls("PPA-CF"), _infill("PPA-CF"),
               "Dry the filament first (80 C for 8 h or more) and print from a dry box",
               "Enclosure closed, chamber heated if the printer can",
               "Nozzle 290 to 320 C, bed 100 to 110 C (check the spool label)"],
    "TPU": ["TPU 95A: 0.4 mm nozzle, 0.2 mm layers", f"{_walls('TPU')}, {_infill('TPU')} (more for stiffer parts)",
            "Slow: max volumetric speed about 3.5 mm3/s", "Retraction short (0.8 mm) or off",
            "Nozzle 220 to 230 C, bed 35 to 45 C", "Feed from an external spool, not through the AMS",
            "Dry the filament (55 C for 6 h)"],
}


# ---------------------------------------------------------------------------------------------- shared helpers
def clean(s) -> str:
    """Text for the outputs: no em or en dashes. An em dash or horizontal bar (and a spaced en or figure dash) becomes
    ", " with the spaces around it dropped; an unspaced en or figure dash (a range like 10-20) becomes "-"."""
    s = _DASH_COMMA_RE.sub(", ", str(s if s is not None else ""))
    return s.replace(EN_DASH, "-").replace(FIG_DASH, "-")


def only_list(only: str | None):
    return [s for s in (only or "").split(",") if s] or None


def _source_mtime() -> float:
    """Newest modification time of the model sources (cad/parts and cad/lib, tests excluded)."""
    t = 0.0
    for sub in ("parts", "lib"):
        for f in glob.glob(os.path.join(BASE, "cad", sub, "**", "*.py"), recursive=True):
            if f"{os.sep}tests{os.sep}" not in f:
                t = max(t, os.path.getmtime(f))
    return t


def export_reason(fn: str, names) -> str | None:
    """Why OUT/asm.json must be (re)exported for the subsystems in names (None: it can be reused). With names it is
    re-exported when it is missing, lacks a requested subsystem, or is older than a cad/parts or cad/lib source.
    Without names only a missing file is exported (a full export is the caller's job, see run_all.sh)."""
    if not os.path.exists(fn):
        return "missing"
    if not names:
        return None
    with open(fn) as fh:
        have = set(json.load(fh).get("subsystems") or [])
    miss = [n for n in names if n not in have]
    if miss:
        return f"it lacks subsystem(s) {', '.join(miss)} (has {', '.join(sorted(have)) or 'none'})"
    if os.path.getmtime(fn) < _source_mtime():
        return "it is older than the cad/parts or cad/lib sources"
    return None


def load_out(out: str, only: str | None = None, log=None) -> dict:
    """The exported manifest (OUT/asm.json). With only, the parts and steps of the other subsystems are dropped, and
    the assembly is re-exported first when asm.json is missing, lacks a requested subsystem or is older than the model
    sources (export_reason). Raises SystemExit (non-zero, with the reason) when no printed part is left, so pack, bom
    and guide never write an empty pack or parts list with exit 0."""
    fn = os.path.join(out, "asm.json")
    names = only_list(only)
    why = export_reason(fn, names)
    if why:
        from tools.assembly import Assembly
        from tools.params import load_params
        (log or (lambda s: print(s, file=sys.stderr)))(f"load_out: exporting the assembly to {out} ({why})")
        Assembly.load(load_params(), names).export(out)
    with open(fn) as fh:
        man = json.load(fh)
    if names:
        man["parts"] = [p for p in man["parts"] if p["subsystem"] in names]
        man["steps"] = [s for s in man.get("steps", []) if s.get("subsystem") in names]
    if not any(p.get("printed") for p in man["parts"]):
        raise SystemExit(f"ERROR: no printed parts in {fn}"
                         + (f" for --only {','.join(names)}" if names else "")
                         + f" (subsystems in the manifest: {', '.join(man.get('subsystems') or []) or 'none'}); "
                           f"nothing to pack or list, re-run the assembly export for these subsystems")
    return man


def part_mesh(out: str, name: str):
    import trimesh
    z = np.load(os.path.join(out, "meshes", f"{name}.npz"))
    return trimesh.Trimesh(np.asarray(z["v"], np.float64), np.asarray(z["f"], np.int64), process=False)


def load_json(fn):
    if os.path.exists(fn):
        with open(fn) as fh:
            return json.load(fh)
    return None


_FONTS = None


def fonts():
    """Register Liberation Sans for reportlab. Returns (regular, bold, note); note says when it fell back."""
    global _FONTS
    if _FONTS:
        return _FONTS
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont
    cands = [("LiberationSans", "LiberationSans-Regular.ttf", "LiberationSans-Bold.ttf"),
             ("DejaVuSans", "DejaVuSans.ttf", "DejaVuSans-Bold.ttf")]
    for fam, reg, bold in cands:
        r = glob.glob(f"/usr/share/fonts/**/{reg}", recursive=True)
        b = glob.glob(f"/usr/share/fonts/**/{bold}", recursive=True)
        if r and b:
            pdfmetrics.registerFont(TTFont(fam, r[0]))
            pdfmetrics.registerFont(TTFont(fam + "-Bold", b[0]))
            note = "" if fam == "LiberationSans" else "Liberation Sans not found, using DejaVu Sans"
            _FONTS = (fam, fam + "-Bold", note)
            return _FONTS
    _FONTS = ("Helvetica", "Helvetica-Bold", "Liberation Sans and DejaVu Sans not found, using Helvetica")
    return _FONTS


def pdf_kit():
    """Plain black reportlab styles and a table helper shared by the PDFs."""
    from reportlab.lib import colors
    from reportlab.lib.styles import ParagraphStyle
    from reportlab.lib.units import mm
    from reportlab.platypus import Paragraph, Table, TableStyle
    reg, bold, note = fonts()
    st = dict(
        B=ParagraphStyle("b", fontName=reg, fontSize=10, leading=13.5, spaceAfter=3),
        H1=ParagraphStyle("h1", fontName=bold, fontSize=17, leading=22, spaceAfter=8),
        H2=ParagraphStyle("h2", fontName=bold, fontSize=13, leading=17, spaceBefore=10, spaceAfter=5),
        H3=ParagraphStyle("h3", fontName=bold, fontSize=11, leading=14, spaceBefore=6, spaceAfter=3),
        SM=ParagraphStyle("sm", fontName=reg, fontSize=8.5, leading=11),
    )

    def esc(t):
        return clean(t).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")

    def P(t, s="B"):
        return Paragraph(esc(t), st[s])

    def tbl(rows, widths, fs=8.5):
        cs = [ParagraphStyle("c", fontName=reg, fontSize=fs, leading=fs + 2.5),
              ParagraphStyle("ch", fontName=bold, fontSize=fs, leading=fs + 2.5)]
        t = Table([[Paragraph(esc(c), cs[1 if i == 0 else 0]) for c in r] for i, r in enumerate(rows)],
                  colWidths=[w * mm for w in widths], repeatRows=1)
        # FONTNAME too: the table sets each cell's style font even for Paragraph cells (default Helvetica)
        t.setStyle(TableStyle([("FONTNAME", (0, 0), (-1, -1), reg), ("GRID", (0, 0), (-1, -1), 0.4, colors.black),
                               ("VALIGN", (0, 0), (-1, -1), "TOP"),
                               ("LEFTPADDING", (0, 0), (-1, -1), 3), ("RIGHTPADDING", (0, 0), (-1, -1), 3)]))
        return t

    return st, P, tbl, note


def canvasmaker():
    """reportlab canvas class whose initial font is ours, so the page preamble does not pull in the unembedded
    Helvetica. (SimpleDocTemplate passes initialFontName=None to the canvas, so a plain partial is not enough.)"""
    from reportlab.pdfgen.canvas import Canvas
    name = fonts()[0]

    def make(*a, **kw):
        kw["initialFontName"] = kw.get("initialFontName") or name
        return Canvas(*a, **kw)
    return make


def doc_template(fn, title, **kw):
    """A4 SimpleDocTemplate with our initial font and empty author, creator, subject and producer."""
    from reportlab.lib.pagesizes import A4
    from reportlab.platypus import SimpleDocTemplate
    return SimpleDocTemplate(fn, pagesize=A4, title=title, author="", creator="", subject="", producer="",
                             initialFontName=fonts()[0], **kw)


def pdf_image(path, max_w_mm, max_h_mm=None):
    from PIL import Image as PI
    from reportlab.lib.units import mm
    from reportlab.platypus import Image
    with PI.open(path) as im:
        r = im.height / im.width
    w = max_w_mm
    if max_h_mm and w * r > max_h_mm:
        w = max_h_mm / r
    return Image(path, width=w * mm, height=w * r * mm)


def filament_m(mass_g: float, material: str) -> float:
    rho = DENSITY.get(material, 1.24)
    vol_mm3 = mass_g / rho * 1000.0
    return vol_mm3 / (math.pi * (FIL_D / 2) ** 2) / 1000.0


def print_time_h(mass_g: float, height_mm: float, material: str) -> float:
    """One copy printed alone: plastic volume at RATE plus LAYER_S per 0.2 mm layer."""
    vol = mass_g / DENSITY.get(material, 1.24) * 1000.0
    return (vol / RATE.get(material, 6.0) + height_mm / 0.2 * LAYER_S) / 3600.0


def plate_time_h(masses_g, height_mm: float, material: str) -> float:
    """A whole plate: the plastic of every copy, but the layer time once (all copies share the layers)."""
    vol = sum(masses_g) / DENSITY.get(material, 1.24) * 1000.0
    return (vol / RATE.get(material, 6.0) + height_mm / 0.2 * LAYER_S) / 3600.0


def print_mass_g(volume_mm3: float, area_mm2: float, material: str) -> float:
    """Printed mass from the PROFILE of the material (the PRINT_SHEET settings): a solid shell of walls x LINE_W over
    the surface, the infill fraction of the core."""
    pr = PROFILE.get(material, PROFILE["PLA"])
    shell = min(volume_mm3, area_mm2 * pr["walls"] * LINE_W)
    return (shell + (volume_mm3 - shell) * pr["infill"]) * DENSITY.get(material, 1.24) / 1000.0


# ---------------------------------------------------------------------------------------------- pack
def oriented(out, p):
    """Mesh of a printed part in its print pose with the bbox min corner at the origin. Returns (mesh, dz)."""
    m = part_mesh(out, p["name"])
    M = np.asarray(p["print_tf"], float) if p.get("print_tf") is not None else np.eye(4)
    m.apply_transform(M)
    lo = m.bounds[0]
    m.apply_translation(-lo)
    return m, float(lo[2])


def geometric_watertight(m) -> bool:
    """Watertight as a slicer sees it: coincident vertices merged (an STL has no other topology)."""
    import trimesh
    return bool(trimesh.Trimesh(m.vertices, m.faces, process=True).is_watertight)


def overhang_local(m):
    from tools.audit import OVERHANG_DEG
    n = m.face_normals
    zmin = m.triangles[:, :, 2].min(axis=1)
    lim = -math.cos(math.radians(OVERHANG_DEG))
    on_bed = (m.triangles[:, :, 2].max(axis=1) < 0.01) & (n[:, 2] < -0.999)
    return float(m.area_faces[(n[:, 2] < lim) & ~on_bed & (zmin > 0.3)].sum())


def _fits(w, h, W, H):
    return (w <= W and h <= H) or (h <= W and w <= H)


def pack_plates(items, bed, spacing, margin, warnings=None):
    """items: list of dicts with name, material, dims, qty. Returns (plates, errors). Plate items carry x, y (min corner
    on the bed) and rot (90 deg about z). A part that fits the bed but not inside the margin gets a plate of its own
    per copy, centred, with a note in warnings (a list, when given); only a part bigger than the bed is an error."""
    import rectpack
    usable = (bed[0] - 2 * margin + spacing, bed[1] - 2 * margin + spacing)
    plates, errors = [], []
    mats = [m for m in MAT_ORDER if any(i["material"] == m for i in items)]
    mats += sorted({i["material"] for i in items} - set(mats))
    for mat in mats:
        rects, solo = [], []
        for it in items:
            if it["material"] != mat:
                continue
            w, h, z = it["dims"]
            if z > bed[2] or not _fits(w, h, bed[0], bed[1]):
                errors.append(f"{it['name']}: print bbox {w:.1f} x {h:.1f} x {z:.1f} does not fit the "
                              f"{bed[0]:g} x {bed[1]:g} x {bed[2]:g} bed")
                continue
            if not _fits(w + spacing, h + spacing, usable[0], usable[1]):
                if warnings is not None:
                    warnings.append(f"{it['name']}: print bbox {w:.1f} x {h:.1f} fits the bed but not inside the "
                                    f"{margin:g} mm margin; printed alone, centred on its own plate")
                solo.append(it)
                continue
            for c in range(int(it["qty"])):
                rects.append((it, c + 1))
        placed = 0
        if rects:
            pk = rectpack.newPacker(rotation=True, pack_algo=rectpack.MaxRectsBssf, sort_algo=rectpack.SORT_AREA)
            S = 100.0                                    # rectpack works in integers: 0.01 mm units
            for k, (it, c) in enumerate(rects):
                pk.add_rect(int(math.ceil((it["dims"][0] + spacing) * S)), int(math.ceil((it["dims"][1] + spacing) * S)), rid=k)
            pk.add_bin(int(usable[0] * S), int(usable[1] * S), count=len(rects))
            pk.pack()
            for b in pk:
                if not len(b):
                    continue
                pl = dict(material=mat, items=[])
                for rc in b:
                    it, c = rects[rc.rid]
                    w0 = int(math.ceil((it["dims"][0] + spacing) * S))
                    rot = rc.width != w0
                    pl["items"].append(dict(name=it["name"], copy=c, x=margin + rc.x / S, y=margin + rc.y / S, rot=bool(rot)))
                    placed += 1
                ux = max(rc.x + rc.width for rc in b) / S - spacing          # centre the packed group on the bed
                uy = max(rc.y + rc.height for rc in b) / S - spacing
                dx, dy = (bed[0] - 2 * margin - ux) / 2, (bed[1] - 2 * margin - uy) / 2
                for d in pl["items"]:
                    d["x"] = round(d["x"] + dx, 3)
                    d["y"] = round(d["y"] + dy, 3)
                pl["items"].sort(key=lambda d: (d["name"], d["copy"]))
                plates.append(pl)
            if placed != len(rects):
                errors.append(f"{mat}: rectpack placed {placed} of {len(rects)} copies")
        for it in solo:
            w, h, _ = it["dims"]
            rot = not (w <= bed[0] and h <= bed[1])
            fw, fh = (h, w) if rot else (w, h)
            for c in range(int(it["qty"])):
                plates.append(dict(material=mat, solo=True, items=[dict(
                    name=it["name"], copy=c + 1, x=round((bed[0] - fw) / 2, 3), y=round((bed[1] - fh) / 2, 3), rot=rot)]))
    return plates, errors


def place_on_plate(m, it):
    import trimesh
    m = m.copy()
    if it["rot"]:
        m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 0, 1]))
        m.apply_translation(-m.bounds[0])
    m.apply_translation([it["x"], it["y"], 0.0])
    return m


def object_names(items, qty_of) -> list[str]:
    """Unique 3MF object names for plate items: the part name, or <name>__c<copy> when the part has qty > 1 ("__"
    cannot clash with another copy's name the way "_c1" can: part x qty 2 against a part named x_c1), with a
    numeric suffix if a name is still taken."""
    names, used = [], set()
    for it in items:
        nm = it["name"] if qty_of[it["name"]] == 1 else f"{it['name']}__c{it['copy']}"
        base, k = nm, 2
        while nm in used:
            nm = f"{base}__{k}"
            k += 1
        used.add(nm)
        names.append(nm)
    return names


def write_3mf(fn, meshes):
    """meshes: list of (object name, trimesh), names unique. One object per copy, build items at identity."""
    import trimesh
    if len({nm for nm, _ in meshes}) != len(meshes):
        raise ValueError("3MF object names must be unique (trimesh drops build items on a name clash)")
    sc = trimesh.Scene()
    for nm, m in meshes:
        sc.add_geometry(m, node_name=nm, geom_name=nm)
    data = trimesh.exchange.threemf.export_3MF(sc)
    with open(fn, "wb") as fh:
        fh.write(data)


def count_3mf_items(fn) -> int:
    """Independent reread of 3D/3dmodel.model: the build items whose object is a mesh with triangles (or a component
    object built from such meshes). This is what a slicer puts on the plate."""
    with zipfile.ZipFile(fn) as z:
        root = ET.fromstring(z.read("3D/3dmodel.model"))

    def local(t):
        return t.rsplit("}", 1)[-1]

    objs = {}
    for el in root.iter():
        if local(el.tag) == "object":
            tri = sum(1 for x in el.iter() if local(x.tag) == "triangle")
            comps = [c.get("objectid") for c in el.iter() if local(c.tag) == "component"]
            objs[el.get("id")] = (tri, comps)

    def solid(oid, depth=0):
        tri, comps = objs.get(oid, (0, []))
        return tri > 0 or (depth < 8 and any(solid(c, depth + 1) for c in comps))

    return sum(1 for el in root.iter() if local(el.tag) == "item" and solid(el.get("objectid")))


def count_3mf_objects(fn) -> int:
    """Independent reread with lib3mf when available (else trimesh): number of mesh objects."""
    try:
        import lib3mf
        w = lib3mf.get_wrapper()
        model = w.CreateModel()
        model.QueryReader("3mf").ReadFromFile(fn)
        it = model.GetMeshObjects()
        n = 0
        while it.MoveNext():
            if it.GetCurrentMeshObject().GetTriangleCount() > 0:
                n += 1
        return n
    except ImportError:
        import trimesh
        return len(trimesh.load(fn, file_type="3mf").geometry)


def plate_png(fn, title, placed, bed):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from matplotlib.patches import Rectangle
    fig, ax = plt.subplots(figsize=(6.2, 6.2))
    ax.add_patch(Rectangle((0, 0), bed[0], bed[1], fill=False, ec="k", lw=1.2))
    legend = []
    for k, (nm, m) in enumerate(placed, 1):
        tri = m.triangles[:, :, :2]
        up = m.face_normals[:, 2] > 1e-6                   # top-down view: the upward faces cover the footprint
        ax.add_collection(PolyCollection(tri[up] if up.any() else tri, facecolors="#bdbdbd", edgecolors="#bdbdbd",
                                         linewidths=0.3))
        lo, hi = m.bounds
        ax.add_patch(Rectangle(lo[:2], hi[0] - lo[0], hi[1] - lo[1], fill=False, ec="k", lw=0.5, ls="--"))
        ax.text((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, str(k), fontsize=8, ha="center", va="center",
                bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="k", lw=0.4))
        legend.append(f"{k} {nm} ({hi[0] - lo[0]:.0f} x {hi[1] - lo[1]:.0f} x {hi[2]:.0f})")
    ax.text(0, -0.09, "\n".join(legend), transform=ax.transAxes, fontsize=7.5, va="top", ha="left")
    ax.set_xlim(-4, bed[0] + 4); ax.set_ylim(-4, bed[1] + 4); ax.set_aspect("equal")
    ax.set_xlabel("x mm"); ax.set_ylabel("y mm"); ax.set_title(title, fontsize=10)
    fig.savefig(fn, dpi=110, bbox_inches="tight")
    plt.close(fig)


def materials_of(rep):
    """Materials in print order that have parts (placed or not)."""
    return [m for m in MAT_ORDER + sorted({r["material"] for r in rep["parts"]} - set(MAT_ORDER))
            if any(r["material"] == m for r in rep["parts"])]


ESTIMATE_NOTE = ("Masses and times are rough estimates: walls and infill per material from the settings below, "
                 "volume at a typical print rate plus {:g} s per 0.2 mm layer, the layer time counted once per plate.")


def sheet_md(rep):
    L = ["# Print sheet", "", f"Generated {rep['created']}. Bed {rep['bed'][0]:g} x {rep['bed'][1]:g} x {rep['bed'][2]:g} mm "
         f"(Bambu X2D). " + ESTIMATE_NOTE.format(LAYER_S), ""]
    if rep["errors"]:
        L += ["## Errors", ""] + [f"- {clean(e)}" for e in rep["errors"]] + [""]
    if rep["warnings"]:
        L += ["## Warnings", ""] + [f"- {clean(e)}" for e in rep["warnings"]] + [""]
    L += ["## Parts", "", "| part | material | qty | print bbox mm | volume cm3 | mass g (each) | time h (each, alone) | overhang mm2 | supports |",
          "|---|---|---|---|---|---|---|---|---|"]
    for r in rep["parts"]:
        L.append(f"| {r['name']} | {r['material']} | {r['qty']} | {' x '.join(f'{d:.1f}' for d in r['dims'])} | "
                 f"{r['volume_cm3']:.1f} | {r['mass_g']:.1f} | {r['time_h']:.2f} | {fmt_oh(r)} | {r['supports']} |")
    L += [""]
    for r in rep["parts"]:
        L.append(f"- **{r['name']}**: {clean(r['orientation'])}" + (f" {clean(r['notes'])}" if r["notes"] else ""))
    L += ["", "## Plates", "", "| plate | material | objects | mass g | time h |", "|---|---|---|---|---|"]
    for pl in rep["plates"]:
        L.append(f"| {pl['file']} | {pl['material']} | {len(pl['items'])}: {', '.join(pl['objects'])} | "
                 f"{pl['mass_g']:.0f} | {pl['time_h']:.1f} |")
    L += ["", "## Totals on the plates", ""]
    for mat, t in rep["totals"].items():
        L.append(f"- {mat}: {t['count']} objects on {t['plates']} plates, {t['mass_g']:.0f} g, about "
                 f"{t['filament_m']:.1f} m of {FIL_D} mm filament, {t['time_h']:.1f} h")
    if rep["unplaced_totals"]:
        L += ["", "## Not placed (see the errors; not in the totals above)", ""]
        for mat, t in rep["unplaced_totals"].items():
            L.append(f"- {mat}: {t['count']} copies ({', '.join(t['parts'])}), {t['mass_g']:.0f} g, about "
                     f"{t['filament_m']:.1f} m of filament, {t['time_h']:.1f} h printed alone")
    L += ["", "## Recommended settings", ""]
    for mat in materials_of(rep):
        L.append(f"### {mat}")
        L += [f"- {s}" for s in SETTINGS.get(mat, ["Use the filament maker's profile"])]
        L.append("")
    return "\n".join(L) + "\n"


def fmt_oh(r):
    return "n/a" if r["overhang_mm2"] is None else f"{r['overhang_mm2']:.0f}"


def sheet_pdf(fn, rep, pack_dir):
    from reportlab.lib.units import mm
    from reportlab.platypus import Spacer, PageBreak
    st, P, tbl, note = pdf_kit()
    S = [P("Print sheet", "H1"),
         P(f"Generated {rep['created']}. Bed {rep['bed'][0]:g} x {rep['bed'][1]:g} x {rep['bed'][2]:g} mm (Bambu X2D). "
           + ESTIMATE_NOTE.format(LAYER_S))]
    if rep["errors"]:
        S.append(P("Errors", "H2"))
        S += [P(e) for e in rep["errors"]]
    if rep["warnings"]:
        S.append(P("Warnings", "H2"))
        S += [P(e) for e in rep["warnings"]]
    S.append(P("Parts", "H2"))
    rows = [["part", "material", "qty", "print bbox mm", "vol cm3", "mass g", "time h", "overhang mm2", "supports"]]
    for r in rep["parts"]:
        rows.append([r["name"], r["material"], r["qty"], " x ".join(f"{d:.1f}" for d in r["dims"]), f"{r['volume_cm3']:.1f}",
                     f"{r['mass_g']:.1f}", f"{r['time_h']:.2f}", fmt_oh(r), r["supports"]])
    S.append(tbl(rows, [36, 16, 9, 27, 13, 13, 12, 17, 37]))
    S.append(Spacer(1, 4))
    for r in rep["parts"]:
        S.append(P(f"{r['name']}: {r['orientation']}" + (f" {r['notes']}" if r["notes"] else ""), "SM"))
    S.append(P("Plates", "H2"))
    rows = [["plate", "material", "objects", "mass g", "time h"]]
    for pl in rep["plates"]:
        rows.append([pl["file"], pl["material"], len(pl["items"]), f"{pl['mass_g']:.0f}", f"{pl['time_h']:.1f}"])
    S.append(tbl(rows, [60, 25, 20, 25, 25]))
    S.append(P("Totals on the plates", "H2"))
    rows = [["material", "objects", "plates", "mass g", "filament m", "time h"]]
    for mat, t in rep["totals"].items():
        rows.append([mat, t["count"], t["plates"], f"{t['mass_g']:.0f}", f"{t['filament_m']:.1f}", f"{t['time_h']:.1f}"])
    S.append(tbl(rows, [35, 22, 22, 25, 25, 25]))
    if rep["unplaced_totals"]:
        S.append(P("Not placed (see the errors; not in the totals above)", "H2"))
        rows = [["material", "copies", "parts", "mass g", "filament m", "time h alone"]]
        for mat, t in rep["unplaced_totals"].items():
            rows.append([mat, t["count"], ", ".join(t["parts"]), f"{t['mass_g']:.0f}", f"{t['filament_m']:.1f}", f"{t['time_h']:.1f}"])
        S.append(tbl(rows, [25, 15, 62, 20, 20, 20]))
    S.append(P("Recommended settings", "H2"))
    for mat in materials_of(rep):
        S.append(P(mat, "H3"))
        S += [P("- " + s) for s in SETTINGS.get(mat, ["Use the filament maker's profile"])]
    for pl in rep["plates"]:
        S.append(PageBreak())
        S.append(P(f"{pl['file']}", "H2"))
        S.append(P(f"{pl['material']}, {len(pl['items'])} objects, about {pl['mass_g']:.0f} g and {pl['time_h']:.1f} h."))
        png = os.path.join(pack_dir, "plates", pl["png"])
        if os.path.exists(png):
            S.append(pdf_image(png, 160, 200))
    if note:
        S.append(P(note, "SM"))
    doc = doc_template(fn, "Print sheet", leftMargin=14 * mm, rightMargin=14 * mm, topMargin=14 * mm, bottomMargin=14 * mm)
    doc.build(S, canvasmaker=canvasmaker())


def _add(tot, mat, mass, time_h, count, name=None):
    t = tot.setdefault(mat, dict(mass_g=0.0, filament_m=0.0, time_h=0.0, count=0, plates=0, parts=[]))
    t["mass_g"] += mass
    t["time_h"] += time_h
    t["count"] += count
    if name and name not in t["parts"]:
        t["parts"].append(name)
    return t


def _round_totals(tot, keep_parts):
    for mat, t in tot.items():
        t["filament_m"] = round(filament_m(t["mass_g"], mat), 2)
        t["mass_g"] = round(t["mass_g"], 1)
        t["time_h"] = round(t["time_h"], 2)
        if not keep_parts:
            t.pop("parts")


def run(out, pack_dir, only=None, spacing=5.0, margin=5.0, log=print) -> dict:
    from tools.params import load_params
    P = load_params()
    bed = tuple(getattr(P, "BED", BED))
    man = load_out(out, only)
    audit = load_json(os.path.join(out, "audit.json")) or {}
    a_static = {r["name"]: r for r in audit.get("static", [])}
    for sub in ("stl", "plates"):
        os.makedirs(os.path.join(pack_dir, sub), exist_ok=True)
        for f in glob.glob(os.path.join(pack_dir, sub, "*")):       # no stale files from earlier runs
            os.remove(f)
    printed = [p for p in man["parts"] if p.get("printed")]
    rep = dict(created=time.strftime("%Y-%m-%d %H:%M"), bed=list(bed), spacing=spacing, margin=margin, parts=[],
               plates=[], errors=[], warnings=[], totals={}, unplaced_totals={})
    meshes = {}
    for p in printed:
        name = p["name"]
        if p.get("print_tf") is None:
            rep["errors"].append(f"{name}: printed part without print_tf (STL left as modelled)")
        m, dz = oriented(out, p)
        if abs(dz) > 1e-3:
            rep["warnings"].append(f"{name}: print_tf min z was {dz:.3f}, shifted to 0")
        m.export(os.path.join(pack_dir, "stl", f"{name}.stl"))
        meshes[name] = m
        dims = [float(x) for x in m.extents]
        a = a_static.get(name)
        oh = a.get("overhang_mm2") if a and "overhang_mm2" in a else round(overhang_local(m), 1)
        wt = geometric_watertight(m)
        if not wt:
            rep["warnings"].append(f"{name}: STL is not watertight once coincident vertices are merged (the mesh "
                                   f"touches itself along an edge or point); slicers may flag it, fix the model")
        mass = print_mass_g(p["volume_mm3"], float(m.area), p["material"])
        sup = "no"
        if oh is not None and oh > SUPPORT_MM2:
            sup = f"yes, tree supports from the plate ({oh:.0f} mm2 over 45 deg)"
        orient = "Flat face down as exported (print_tf)."
        if p.get("print_tf") is not None and np.allclose(np.asarray(p["print_tf"])[:3, :3], np.eye(3)):
            orient = "As modelled, bottom face on the bed."
        elif p.get("print_tf") is not None:
            zl = np.linalg.inv(np.asarray(p["print_tf"])[:3, :3])[:, 2]
            ax = "xyz"[int(np.argmax(np.abs(zl)))]
            orient = f"Rotated: the part's {'+' if zl[np.argmax(np.abs(zl))] > 0 else '-'}{ax} axis points up."
        prof = PROFILE.get(p["material"], PROFILE["PLA"])
        rep["parts"].append(dict(name=name, subsystem=p.get("subsystem", ""), material=p["material"], qty=int(p.get("qty", 1)),
                                 dims=[round(d, 2) for d in dims], volume_cm3=round(p["volume_mm3"] / 1000.0, 2),
                                 mass_g=round(mass, 1), model_mass_g=p.get("mass_g"), walls=prof["walls"],
                                 infill=prof["infill"], time_h=round(print_time_h(mass, dims[2], p["material"]), 2),
                                 overhang_mm2=oh, supports=sup, orientation=orient, notes=clean(p.get("notes", "")),
                                 watertight=wt, stl=f"stl/{name}.stl"))
    rep["parts"].sort(key=lambda r: (MAT_ORDER.index(r["material"]) if r["material"] in MAT_ORDER else 99, -r["volume_cm3"]))
    plates, errs = pack_plates(rep["parts"], bed, spacing, margin, rep["warnings"])
    rep["errors"] += errs
    byname = {r["name"]: r for r in rep["parts"]}
    qty_of = {n: r["qty"] for n, r in byname.items()}
    placed = {n: 0 for n in byname}
    for i, pl in enumerate(plates, 1):
        stem = f"plate_{i}_{pl['material']}"
        names = object_names(pl["items"], qty_of)
        objs = [(on, place_on_plate(meshes[it["name"]], it)) for on, it in zip(names, pl["items"])]
        fn = os.path.join(pack_dir, "plates", stem + ".3mf")
        write_3mf(fn, objs)
        n = count_3mf_items(fn)
        if n != len(objs):
            rep["errors"].append(f"{stem}.3mf reopens with {n} build items, expected {len(objs)}")
        masses = [byname[it["name"]]["mass_g"] for it in pl["items"]]
        hmax = max(byname[it["name"]]["dims"][2] for it in pl["items"])
        th = plate_time_h(masses, hmax, pl["material"])
        for it in pl["items"]:
            placed[it["name"]] += 1
        t = _add(rep["totals"], pl["material"], sum(masses), th, len(objs))
        t["plates"] += 1
        plate_png(os.path.join(pack_dir, "plates", stem + ".png"), f"plate {i}  {pl['material']}  ({len(objs)} objects)", objs, bed)
        rep["plates"].append(dict(file=stem + ".3mf", png=stem + ".png", material=pl["material"], items=pl["items"],
                                  objects=names, build_items=n, solo=bool(pl.get("solo")), mass_g=round(sum(masses), 1),
                                  time_h=round(th, 2)))
    for r in rep["parts"]:
        miss = r["qty"] - placed[r["name"]]
        if miss > 0:
            _add(rep["unplaced_totals"], r["material"], r["mass_g"] * miss, r["time_h"] * miss, miss, r["name"])
    _round_totals(rep["totals"], False)
    _round_totals(rep["unplaced_totals"], True)
    with open(os.path.join(pack_dir, "print_report.json"), "w") as fh:
        json.dump(rep, fh, indent=1)
    with open(os.path.join(pack_dir, "PRINT_SHEET.md"), "w") as fh:
        fh.write(sheet_md(rep))
    sheet_pdf(os.path.join(pack_dir, "PRINT_SHEET.pdf"), rep, pack_dir)
    log(f"pack: {len(rep['parts'])} printed parts, {len(rep['plates'])} plates -> {pack_dir}")
    for pl in rep["plates"]:
        log(f"  {pl['file']}: {', '.join(pl['objects'])} (reopened: {pl['build_items']} build items, "
            f"{pl['mass_g']:.0f} g, {pl['time_h']:.1f} h)")
    for mat, t in rep["totals"].items():
        log(f"  {mat}: {t['count']} objects on {t['plates']} plates, {t['mass_g']:.0f} g, {t['time_h']:.1f} h")
    for mat, t in rep["unplaced_totals"].items():
        log(f"  NOT PLACED {mat}: {t['count']} copies ({', '.join(t['parts'])}), {t['mass_g']:.0f} g")
    for w in rep["warnings"]:
        log(f"  WARN {w}")
    for e in rep["errors"]:
        log(f"  ERROR {e}")
    return rep


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("pack")
    ap.add_argument("--only", default="")
    ap.add_argument("--spacing", type=float, default=5.0)
    ap.add_argument("--margin", type=float, default=5.0)
    a = ap.parse_args(argv)
    rep = run(a.out, a.pack, a.only, a.spacing, a.margin)
    return 2 if rep["errors"] else 0


if __name__ == "__main__":
    sys.exit(main())
