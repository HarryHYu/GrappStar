#!/usr/bin/env python3
"""KEDGE dart findings report for the v31 to v33 round, the prototype drill bits and the self-righting study.

    kedge/.venv/bin/python kedge/reports/make_findings_report.py [--refresh-mass]

Writes kedge/reports/KEDGE_dart_findings_report.pdf and its graphs kedge/reports/fig/fig_*.png. The renders and
screenshots it uses are copies in kedge/reports/fig/ (cad_*, sim_*, proto_bits.png).

Reads only (never edits): kedge/out31, out32, out33 (physics.json, flight_sim.json, audit.json, launcher_fit.json,
phys31/anchor.json), kedge/out32/phys31/v32_release.json and v32_checks.json, kedge/sim_righting/results.json,
kedge/reports/data/righting_batch_rerun.json (from righting_batch_rerun.mjs: the page's own physics core run headless
with the page's batch seeds, to get each run's righting time and the centre of mass cases) and
kedge/reports/data/mass_rows.json (dart_model.Dart(out).mp rows; --refresh-mass recomputes them from the meshes).

Style (Harry's rules): plain black Arial (Liberation Sans, its metric twin, off the Mac), no decorative styling, no name
on the document, no em dash characters anywhere (checked before the PDF is written).
"""
import io
import json
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, Polygon, Rectangle
from PIL import Image as PILImage

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_CENTER
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (BaseDocTemplate, PageTemplate, Frame, Paragraph, Spacer, Table, TableStyle, Image,
                                PageBreak, KeepTogether, CondPageBreak)
from reportlab.platypus.tableofcontents import TableOfContents

HERE = os.path.dirname(os.path.abspath(__file__))
KEDGE = os.path.dirname(HERE)
ROOT = os.path.dirname(KEDGE)
FIG = os.path.join(HERE, "fig")
DATA = os.path.join(HERE, "data")
OUT_PDF = os.path.join(HERE, "KEDGE_dart_findings_report.pdf")
DATE = "9 November 2026"

# ------------------------------------------------------------------------------------------------ fonts
_FONTS = [
    ("/System/Library/Fonts/Supplemental/Arial.ttf", "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
     "/System/Library/Fonts/Supplemental/Arial Italic.ttf"),
    ("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
     "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
     "/usr/share/fonts/truetype/liberation/LiberationSans-Italic.ttf"),
]
_REG, _BOLD, _ITAL = next(f for f in _FONTS if os.path.exists(f[0]))
pdfmetrics.registerFont(TTFont("Arial", _REG))
pdfmetrics.registerFont(TTFont("Arial-Bold", _BOLD))
pdfmetrics.registerFont(TTFont("Arial-Italic", _ITAL))
pdfmetrics.registerFontFamily("Arial", normal="Arial", bold="Arial-Bold", italic="Arial-Italic", boldItalic="Arial-Bold")
for f in (_REG, _BOLD, _ITAL):
    font_manager.fontManager.addfont(f)
_MPL_FAMILY = font_manager.FontProperties(fname=_REG).get_name()
plt.rcParams.update({
    "font.family": _MPL_FAMILY, "font.size": 8.5, "axes.titlesize": 9, "axes.labelsize": 8.5, "legend.fontsize": 7.5,
    "xtick.labelsize": 8, "ytick.labelsize": 8, "axes.edgecolor": "black", "axes.linewidth": 0.7,
    "text.color": "black", "axes.labelcolor": "black", "xtick.color": "black", "ytick.color": "black",
    "grid.color": "#c8c8c8", "grid.linewidth": 0.5, "legend.frameon": False, "savefig.dpi": 220,
    "axes.spines.top": False, "axes.spines.right": False,
})
K, G1, G2, G3 = "black", "#555555", "#8c8c8c", "#c4c4c4"

# ------------------------------------------------------------------------------------------------ data
def J(*p):
    with open(os.path.join(KEDGE, *p)) as f:
        return json.load(f)


V = ("31", "32", "33")
VN = {"31": "v31", "32": "v32c", "33": "v33"}
PH = {v: J(f"out{v}", "physics.json") for v in V}
AN = {v: J(f"out{v}", "phys31", "anchor.json") for v in V}
FS = {v: J(f"out{v}", "flight_sim.json") for v in V}
AU = {v: J(f"out{v}", "audit.json") for v in V}
LF = {v: J(f"out{v}", "launcher_fit.json") for v in V}
REL = J("out32", "phys31", "v32_release.json")
CHK = J("out32", "phys31", "v32_checks.json")
RS = J("sim_righting", "results.json")
with open(os.path.join(DATA, "righting_batch_rerun.json")) as f:
    RR = json.load(f)["runs"]


def mass_rows(refresh=False):
    p = os.path.join(DATA, "mass_rows.json")
    if refresh or not os.path.exists(p):
        sys.path.insert(0, KEDGE)
        import dart_model as dm
        out = {}
        for v in V:
            d = dm.Dart(os.path.join(KEDGE, f"out{v}"), ballast_g=0.0)
            out[f"out{v}"] = [(r[0], round(float(r[1]), 2), [round(float(x), 1) for x in r[2]],
                               bool(d.PP.get(r[0], {}).get("printable"))) for r in d.mp["rows"]]
        with open(p, "w") as f:
            json.dump(out, f, indent=0)
    with open(p) as f:
        return json.load(f)


MR = mass_rows("--refresh-mass" in sys.argv)

SOILS = ("dry sand", "moist loam", "firm clay")
STALL = AN["33"]["motors_7V4"]["drill"]["stall_Nm"]          # drill gearmotor stall at 7.4 V (0.182 N m)
WORM = AN["33"]["motors_7V4"]["worm"]["stall_Nm"]             # worm N20 stall at 7.4 V (0.207 N m)
SETUP = REL["assumptions"]["setup"][1]                        # breakaway after standing = 1.3 x spin torque
PLANET = REL["cases"][0]["drives"]["2.33:1 planetary 28/12/8"]["stall_Nm"]   # 0.361 N m at the sleeve
TAPER = 0.35            # CLAUDE.md section 5: embedded tapered flight about 0.35 of the v32 sleeve torque (hand estimate)


def hold(v, soil):
    return PH[v]["anchoring"]["by_soil"][soil]["total_N"]


def loam_row(v, rover=3.0):
    return next(r for r in AN[v]["holding_limit"]["moist loam"] if r["rover_m"] == rover)


def coh_T(v):
    return [(r["c_kPa"], r["drill_install_Nm"]) for r in AN[v]["loam_cohesion_sensitivity_rover3m"]]


def T_at(v, c):
    cs, ts = zip(*coh_T(v))
    return float(np.interp(c, cs, ts))


def c_release_limit(fn_T, stall):
    """Cohesion (kPa) where stall / (1.3 T) falls to 1, by bisection on the interpolated torque."""
    lo, hi = 2.0, 20.0
    if stall / (SETUP * fn_T(hi)) > 1:
        return None
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if stall / (SETUP * fn_T(mid)) > 1:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def spin(v, soil="moist loam"):
    return AN[v]["soils"][soil]["drill_each"]["install_torque_Nm"]


def unscrew(v, soil="moist loam"):
    return AN[v]["soils"][soil]["unscrew"]


MARGIN = {v: unscrew(v)["drill_margin"] for v in V}
MARGIN_PLANET = next(c for c in REL["cases"] if c["c_kPa"] == 5.0 and c["flight_od_mm"] == 22.0)["drives"]["2.33:1 planetary 28/12/8"]["breakaway_margin_after_standing"]
MARGIN_TAPER = STALL / (SETUP * TAPER * spin("32"))
C_LIM = {
    "31": c_release_limit(lambda c: T_at("31", c), STALL),
    "32": c_release_limit(lambda c: T_at("32", c), STALL),
    "32p": c_release_limit(lambda c: T_at("32", c), PLANET),
    "33": c_release_limit(lambda c: T_at("33", c), STALL),
    "33t": c_release_limit(lambda c: TAPER * T_at("32", c), STALL),
}


def batch_stats(key):
    r = RR[key]
    ok = [x for x in r if x["righted"]]
    t = sorted(x["rightT"] for x in ok)
    levels = [15, 20, 25, 30, 35, 40, 45, 50, 55]
    held = {L: sum(1 for x in r if x["anchored"] == 2 and x["held"] >= L) for L in levels}
    wl = [x["motorPeak"] for x in r if x["anchored"] == 2]
    return dict(n=len(r), righted=len(ok), median=t[len(t) // 2] if t else None, slowest=t[-1] if t else None,
                held=held, worm_runs=sum(1 for m in wl if m >= 0.01), worm_peak=max(wl) if wl else 0.0,
                stuck=sum(1 for x in r if x["stuck"]), aligned=sum(1 for x in r if x["aligned"]),
                times=t, first=sum(1 for x in ok if x["moves"] == 0))


BS = {k: batch_stats(k) for k in RR}
RB = RS["batch_50_random"]
PRE = RS["presets"]

# ------------------------------------------------------------------------------------------------ styles
B = ParagraphStyle("B", fontName="Arial", fontSize=9.2, leading=12.2, spaceAfter=4.5)
BS_ = ParagraphStyle("BS", parent=B, fontSize=8.8, leading=11.4, spaceAfter=3)
LI = ParagraphStyle("LI", parent=B, leftIndent=12, bulletIndent=2, spaceAfter=2.5)
LIS = ParagraphStyle("LIS", parent=BS_, leftIndent=12, bulletIndent=2, spaceAfter=2)
EQ = ParagraphStyle("EQ", parent=B, leftIndent=22, spaceBefore=1, spaceAfter=4)
CAP = ParagraphStyle("CAP", parent=B, fontSize=8.2, leading=10.4, spaceBefore=3, spaceAfter=9)
TCAP = ParagraphStyle("TCAP", parent=CAP, spaceBefore=6, spaceAfter=3)
H1 = ParagraphStyle("H1", parent=B, fontName="Arial-Bold", fontSize=14, leading=18, spaceBefore=4, spaceAfter=6)
H2 = ParagraphStyle("H2", parent=B, fontName="Arial-Bold", fontSize=10.8, leading=14, spaceBefore=8, spaceAfter=3)
H3 = ParagraphStyle("H3", parent=B, fontName="Arial-Bold", fontSize=9.4, leading=12.5, spaceBefore=5, spaceAfter=2)
TT = ParagraphStyle("TT", parent=B, fontSize=7.6, leading=9.3, spaceAfter=0)
TH = ParagraphStyle("TH", parent=TT, fontName="Arial-Bold")
TITLE = ParagraphStyle("TITLE", parent=B, fontName="Arial-Bold", fontSize=22, leading=27, spaceAfter=8)
SUB = ParagraphStyle("SUB", parent=B, fontSize=13, leading=17, spaceAfter=6)

story = []
_TEXT = []           # every string that goes into the PDF, for the em dash check
_fig_n = [0]
_tab_n = [0]
FIG_ORDER = ["dep1", "dep2", "audit31", "sheets", "kick", "close", "section", "mass", "holding", "spin", "release",
             "proto", "sim1", "sim2", "hist", "cg", "tension", "layout", "tow"]
TAB_ORDER = ["key", "hold", "margins", "deploy", "coh", "loadpath", "bits", "bittests", "simparams", "presets", "batch",
             "cg", "launch", "flight", "jetson", "measure", "appA1", "appA2", "appB1", "appB2", "appB3", "appB4", "files", "commits"]


def fno(label):
    return FIG_ORDER.index(label) + 1


def tno(label):
    return TAB_ORDER.index(label) + 1


def P(text, st=B):
    _TEXT.append(text)
    return Paragraph(text, st)


def bullets(items, st=LI, mark="\u2022"):
    out = []
    for t in items:
        _TEXT.append(t)
        out.append(Paragraph(t, st, bulletText=mark) if mark else Paragraph(t, st))
    return out


def h1(t):
    story.append(CondPageBreak(60 * mm))
    story.append(P(t, H1))


def h2(t):
    story.append(CondPageBreak(30 * mm))
    story.append(P(t, H2))


def h3(t):
    story.append(P(t, H3))


def eq(t):
    story.append(P(t, EQ))


def table(rows, widths, caption=None, head=1, fs=None, keep=True, label=None):
    st, sh = TT, TH
    if fs:
        st = ParagraphStyle("t%s" % fs, parent=TT, fontSize=fs, leading=fs + 1.8)
        sh = ParagraphStyle("th%s" % fs, parent=st, fontName="Arial-Bold")
    data = []
    for i, r in enumerate(rows):
        data.append([P(str(c), sh if i < head else st) for c in r])
    t = Table(data, colWidths=[w * mm for w in widths], repeatRows=head)
    t.setStyle(TableStyle([
        ("GRID", (0, 0), (-1, -1), 0.35, K),
        ("LINEBELOW", (0, head - 1), (-1, head - 1), 0.9, K),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 2.6), ("RIGHTPADDING", (0, 0), (-1, -1), 2.6),
        ("TOPPADDING", (0, 0), (-1, -1), 1.6), ("BOTTOMPADDING", (0, 0), (-1, -1), 2.0),
    ]))
    out = []
    if caption:
        _tab_n[0] += 1
        assert label is None or tno(label) == _tab_n[0], (label, _tab_n[0])
        out.append(P(f"Table {_tab_n[0]}. {caption}", TCAP))
    out.append(t)
    out.append(Spacer(1, 6))
    if keep:
        story.append(KeepTogether(out))
    else:
        story.extend(out)
    return _tab_n[0]


def _img_buf(path, crop=None, maxw=1500, jpeg=True):
    im = PILImage.open(path).convert("RGB")
    if crop:
        im = im.crop(crop)
    if im.width > maxw:
        im = im.resize((maxw, round(im.height * maxw / im.width)), PILImage.LANCZOS)
    buf = io.BytesIO()
    if jpeg:
        im.save(buf, "JPEG", quality=86)
    else:
        im.save(buf, "PNG")
    buf.seek(0)
    return buf, im.width, im.height


def figure(paths, caption, width_mm=170, crops=None, jpeg=True, maxw=1500, gap_mm=3, label=None):
    """One image, or several side by side in one row, with a numbered caption under them."""
    if isinstance(paths, str):
        paths = [paths]
    crops = crops or [None] * len(paths)
    n = len(paths)
    w_each = (width_mm - gap_mm * (n - 1)) / n
    cells = []
    for p, c in zip(paths, crops):
        buf, w, h = _img_buf(p, c, maxw, jpeg and not p.endswith("_graph.png"))
        cells.append(Image(buf, width=w_each * mm, height=w_each * mm * h / w))
    _fig_n[0] += 1
    assert label is None or fno(label) == _fig_n[0], (label, _fig_n[0])
    cap = P(f"Figure {_fig_n[0]}. {caption}", CAP)
    if n == 1:
        body = cells[0]
    else:
        body = Table([cells], colWidths=[w_each * mm + (gap_mm * mm if i < n - 1 else 0) for i in range(n)])
        body.setStyle(TableStyle([("LEFTPADDING", (0, 0), (-1, -1), 0), ("RIGHTPADDING", (0, 0), (-1, -1), 0),
                                  ("TOPPADDING", (0, 0), (-1, -1), 0), ("BOTTOMPADDING", (0, 0), (-1, -1), 0),
                                  ("VALIGN", (0, 0), (-1, -1), "TOP")]))
    story.append(KeepTogether([Spacer(1, 3), body, cap]))
    return _fig_n[0]


def graph(name, fig):
    p = os.path.join(FIG, f"fig_{name}.png")
    fig.savefig(p, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return p


def f1(x):
    return f"{x:.1f}"


def f2(x):
    return f"{x:.2f}"


# ------------------------------------------------------------------------------------------------ graphs
def g_holding():
    soils = ("dry sand", "moist loam")
    fig, ax = plt.subplots(figsize=(6.4, 2.9))
    w = 0.26
    fills = ("white", G2, K)
    for i, v in enumerate(V):
        vals = [hold(v, s) for s in soils]
        xs = [k + (i - 1) * w for k in range(len(soils))]
        ax.bar(xs, vals, w, color=fills[i], edgecolor=K, linewidth=0.8, label=VN[v], zorder=3)
        for x, y in zip(xs, vals):
            ax.text(x, y + 1.2, f1(y), ha="center", va="bottom", fontsize=7.5, zorder=5,
                    bbox=dict(facecolor="white", edgecolor="none", pad=0.6))
    ax.axhline(AN["33"]["assumptions"]["first_tow_N"], ls="--", color=K, lw=0.8, zorder=2)
    ax.text(0.5, AN["33"]["assumptions"]["first_tow_N"] + 1.2, "first tow 15 N", ha="center", fontsize=7.5)
    ax.axhline(AN["33"]["assumptions"]["tow_cap_N"], ls=":", color=K, lw=0.9, zorder=2)
    ax.text(0.5, AN["33"]["assumptions"]["tow_cap_N"] + 1.2, "tow cap 55 N", ha="center", fontsize=7.5)
    ax.set_xticks(range(len(soils)), ["Dry sand (c 0, phi 32 deg)", "Moist loam (c 5 kPa, phi 28 deg)"])
    ax.set_ylabel("Holding at the dart, rover 3 m (N)")
    ax.set_ylim(0, 85)
    ax.grid(axis="y", zorder=0)
    ax.legend(loc="upper left", ncol=3)
    return graph("holding_by_soil", fig)


def g_release():
    cs = np.linspace(2, 20, 300)
    fig, ax = plt.subplots(figsize=(6.4, 3.3))
    lines = [
        ("v31: 13 mm radial auger, 100 rpm motor", [STALL / (SETUP * T_at("31", c)) for c in cs], dict(color=G2, ls="-.", lw=1.1)),
        ("v32c: 22 mm sleeve, 100 rpm motor (as built)", [STALL / (SETUP * T_at("32", c)) for c in cs], dict(color=K, ls=":", lw=1.4)),
        ("v32c: 22 mm sleeve + 2.33:1 planetary (not built)", [PLANET / (SETUP * T_at("32", c)) for c in cs], dict(color=G2, ls="--", lw=1.1)),
        ("v33: 16 mm tapered, pipeline (uniform flight)", [STALL / (SETUP * T_at("33", c)) for c in cs], dict(color=K, ls="-", lw=1.6)),
        ("v33: taper estimate (0.35 of the v32c torque)", [STALL / (SETUP * TAPER * T_at("32", c)) for c in cs], dict(color=K, ls="--", lw=1.1)),
    ]
    for lab, y, kw in lines:
        ax.plot(cs, y, label=lab, **kw)
    ax.axhline(1.0, color=K, lw=0.8)
    ax.text(20.3, 0.98, "1.00: releases after\nstanding above this", ha="left", va="top", fontsize=7, clip_on=False)
    guard = 1 / (REL["assumptions"]["depth_limit_frac"] * SETUP)
    ax.axhline(guard, color=G3, lw=0.8, ls="-")
    ax.text(20.3, guard + 0.02, f"{guard:.2f}: kept by the\n67 % deploy guard", ha="left", va="bottom", fontsize=7, color=G1, clip_on=False)
    ax.axvline(5, color=G2, lw=0.7, ls=":")
    ax.text(5.15, 5.25, "design loam\nc 5 kPa", fontsize=7, va="top")
    ax.set_xlim(2, 20)
    ax.set_ylim(0, 5.5)
    ax.set_xlabel("Soil cohesion c (kPa)")
    ax.set_ylabel("Release margin after standing\n(stall torque / (1.3 x spin torque))")
    ax.grid(True)
    ax.legend(loc="upper right", fontsize=7)
    return graph("release_vs_cohesion", fig)


def g_spin():
    D = np.linspace(10, 24, 200)
    t32, t33 = spin("32"), spin("33")
    L32 = AN["32"]["geometry"]["drill"]["embedded_effective_mm"]
    L33 = AN["33"]["geometry"]["drill"]["embedded_effective_mm"]
    fig, ax = plt.subplots(figsize=(6.4, 3.1))
    ax.plot(D, t32 * (D / 22) ** 2, color=K, lw=1.2, ls=":", label=f"T = {t32:.3f} x (D / 22)^2   (embedded length {L32:.1f} mm, v32c)")
    ax.plot(D, t33 * (D / 16) ** 2, color=K, lw=1.4, label=f"T = {t33:.3f} x (D / 16)^2   (embedded length {L33:.1f} mm, v33)")
    sz = [(c["flight_od_mm"], c["spin_Nm"]) for c in REL["cases"] if c["c_kPa"] == 5.0]
    ax.plot([s[0] for s in sz], [s[1] for s in sz], "s", color=K, ms=4.5, label="v32 sleeve sizing runs (v32_release.json, c 5 kPa)")
    pts = [("v31 13 mm", AN["31"]["geometry"]["drill"]["D_mm"], spin("31")),
           ("v32c 22 mm", 22.0, t32), ("v33 16 mm", 16.0, t33)]
    offs = {"v31 13 mm": (-8, 7), "v32c 22 mm": (-30, -14), "v33 16 mm": (7, -9)}
    for lab, d, t in pts:
        ax.plot(d, t, "o", color=K, ms=5)
        ax.annotate(lab, (d, t), textcoords="offset points", xytext=offs[lab], fontsize=7)
    for nm, d in (("T12", 12), ("T16", 16), ("T20", 20)):
        t = t33 * (d / 16) ** 2
        ax.plot(d, t, "o", mfc="white", mec=K, ms=6)
        ax.annotate(nm, (d, t), textcoords="offset points", xytext=(-6, 7) if nm != "T20" else (6, -10), fontsize=7)
    ax.plot([], [], "o", mfc="white", mec=K, label="prototype bits at the v33 embedded length (predicted)")
    ax.axhline(STALL, color=G2, lw=0.9, ls="--")
    ax.text(10.2, STALL + 0.004, f"drill stall at 7.4 V: {STALL:.3f} N m", fontsize=7, color=G1)
    ax.axhline(STALL / SETUP, color=G2, lw=0.9, ls="-.")
    ax.text(10.2, STALL / SETUP + 0.004, f"release limit after standing: stall / 1.3 = {STALL / SETUP:.3f} N m", fontsize=7, color=G1)
    ax.set_xlim(10, 24)
    ax.set_ylim(0, 0.32)
    ax.set_xlabel("Flight outside diameter D (mm)")
    ax.set_ylabel("Spin torque in moist loam, c 5 kPa (N m)")
    ax.grid(True)
    ax.legend(loc="upper left", fontsize=6.8)
    return graph("spin_torque_vs_od", fig)


MASS_GROUPS = [
    ("Hull, nose, tail cap, tray, boom (printed)", lambda n: n in ("hull_top", "hull_bottom", "nose", "tail_cap", "tray", "tail_boom", "boom_anchor")),
    ("Legs: kick, foot, arms, augers, couplers (printed)", lambda n: n in ("kick_leg", "foot_auger", "coupler_L", "coupler_R", "pod_cap_L", "pod_cap_R")
     or (n.startswith("drill_leg_") and (n.endswith("_auger") or n.endswith("_tip") or n in ("drill_leg_L", "drill_leg_R")))),
    ("Cells, 2 x 18350", lambda n: n.startswith("cell_")),
    ("Motors: 3 worm N20, 2 drill gearmotors", lambda n: n.startswith("n20_") or n.endswith("_motor")),
    ("Boards, camera, VTX, antennas", lambda n: n in ("xiao", "drv8833_1", "drv8833_2", "drv8833_3", "ina219", "bno085", "mp1584", "vtx", "bms_2s", "camera", "capacitor", "antenna", "xiao_antenna")),
    ("Screws, nuts, bearings, contacts, sockets", lambda n: any(k in n for k in ("screw", "brg", "_nut", "nyloc", "contacts", "jst", "washer"))),
    ("Wiring, glue, tether inside", lambda n: n.startswith("wiring") or n.startswith("glue") or n.startswith("tether")),
]


def mass_groups(v):
    rows = MR[f"out{v}"]
    out = []
    used = set()
    for lab, fn in MASS_GROUPS:
        s = sum(r[1] for r in rows if fn(r[0]) and r[0] not in used)
        used |= {r[0] for r in rows if fn(r[0])}
        out.append((lab, s))
    rest = [r for r in rows if r[0] not in used]
    assert not rest, rest
    return out


def g_mass():
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    labs = [g[0] for g in MASS_GROUPS][::-1]
    h = 0.26
    fills = ("white", G2, K)
    for i, v in enumerate(V):
        vals = dict(mass_groups(v))
        ys = [k + (1 - i) * h for k in range(len(labs))]
        xs = [vals[l] for l in labs]
        ax.barh(ys, xs, h, color=fills[i], edgecolor=K, linewidth=0.7, label=f"{VN[v]} ({PH[v]['mass_g']} g)", zorder=3)
        for x, y in zip(xs, ys):
            ax.text(x + 0.6, y, f1(x), va="center", fontsize=7.4)
    ax.set_yticks(range(len(labs)), labs)
    ax.set_xlabel("Mass (g): printed at 4 walls and 40 % infill, bought parts at catalogue values")
    ax.set_xlim(0, 105)
    ax.grid(axis="x", zorder=0)
    ax.legend(loc="lower right")
    return graph("mass_breakdown", fig)


def g_hist():
    a = BS["tail/v33/loam/cg+8"]["times"]
    b = BS["front/v33/loam/cg+8"]["times"]
    bins = np.arange(0, 38, 2)
    ha, _ = np.histogram(a, bins)
    hb, _ = np.histogram(b, bins)
    fig, ax = plt.subplots(figsize=(6.4, 2.8))
    c = bins[:-1] + 1
    ax.bar(c - 0.42, ha, 0.8, color=K, edgecolor=K, label=f"tail layout: median {BS['tail/v33/loam/cg+8']['median']:.1f} s, slowest {BS['tail/v33/loam/cg+8']['slowest']:.1f} s", zorder=3)
    ax.bar(c + 0.42, hb, 0.8, color=G3, edgecolor=K, linewidth=0.6, label=f"front layout: median {BS['front/v33/loam/cg+8']['median']:.1f} s, slowest {BS['front/v33/loam/cg+8']['slowest']:.1f} s", zorder=3)
    ax.set_xticks(bins)
    ax.set_yticks(range(0, max(max(ha), max(hb)) + 3, 2))
    ax.set_xlabel("Time from landing to belly down (s), 2 s bins")
    ax.set_ylabel("Runs (of 50)")
    ax.grid(axis="y", zorder=0)
    ax.legend(loc="upper left")
    return graph("righting_time_hist", fig)


def g_tension():
    levels = [15, 20, 25, 30, 35, 40, 45, 50, 55]
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    tl = RB["tail/v33/loam"]["heldAtN"]
    fr = RB["front/v33/loam"]["heldAtN"]
    ts = RB["tail/v33/sand"]["heldAtN"]
    y = lambda d: [d.get(str(L), 0) for L in levels]
    ax.plot(levels, y(tl), "-o", color=K, ms=4, lw=1.4, label="tail layout, moist loam")
    ax.plot(levels, y(fr), "-s", color=G2, ms=4, lw=1.4, label="front layout, moist loam")
    fr0 = BS["front/v33/loam/cg+0"]["held"]
    ax.plot(levels, [fr0[L] for L in levels], "--s", color=G2, mfc="white", ms=4, lw=1.0, label="front layout, loam, centre of mass on the axis (rerun)")
    ax.plot(levels, y(ts), "-^", color=K, mfc="white", ms=4, lw=1.0, label="tail layout, dry sand (front layout the same)")
    ax.set_xticks(levels)
    ax.set_ylim(-2, 54)
    ax.set_xlabel("Tow tension step at the dart (N)")
    ax.set_ylabel("Runs that held the step (of 50)")
    ax.grid(True)
    ax.legend(loc="center", bbox_to_anchor=(0.52, 0.5), fontsize=7)
    return graph("holding_vs_tension", fig)


def g_layouts():
    fig, axs = plt.subplots(1, 2, figsize=(7.2, 3.3))
    a = math.radians(50)
    for ax, lay in zip(axs, ("tail", "front")):
        ax.set_aspect("equal")
        ax.axis("off")
        ax.add_patch(Rectangle((-15, -100), 250, 79, facecolor="#ececec", edgecolor="none", zorder=0))
        ax.plot([-15, 235], [-21, -21], color=K, lw=0.9)
        ax.text(-12, -97, "soil", fontsize=7.4, color=G1)
        ax.add_patch(Polygon([(0, -7), (0, 7), (20, 21), (152, 21), (152, -21), (20, -21)], closed=True, facecolor="white", edgecolor=K, lw=1.0, zorder=2))
        if lay == "tail":
            pv, f = (140.0, 10.0), -1
            title = "Tail layout: pivots near the tail, pods fold forward"
        else:
            pv, f = (52.0, 10.0), 1
            title = "Front layout (like v33): pivots forward, pods fold aft"
        ax.text(86 if lay == "front" else 60, -12, "hull", ha="center", fontsize=7.4, zorder=3)
        fold_tip = (pv[0] + f * 100, pv[1])
        tip = (pv[0] + f * 100 * math.cos(a), pv[1] - 100 * math.sin(a))
        ax.plot([pv[0], fold_tip[0]], [pv[1], fold_tip[1]], color=G3, lw=5, solid_capstyle="round", zorder=3)
        ax.plot([pv[0], tip[0]], [pv[1], tip[1]], color=K, lw=5, solid_capstyle="round", zorder=4)
        ax.plot(*pv, "o", mfc="white", mec=K, ms=5, zorder=5)
        ax.text(pv[0], 23, "pivot", fontsize=7.4, ha="center", va="bottom", zorder=6)
        ax.text(fold_tip[0] + (8 if lay == "tail" else -8), 23, "folded pod", fontsize=7.4, color=G1, ha="center", va="bottom")
        th = np.linspace(0, a, 30)
        arc = np.c_[pv[0] + f * 100 * np.cos(th), pv[1] - 100 * np.sin(th)]
        ax.plot(arc[:, 0], arc[:, 1], color=G1, lw=0.8, ls=":", zorder=3)
        ax.add_patch(FancyArrowPatch(tuple(arc[-6]), tuple(arc[-1]), arrowstyle="-|>", mutation_scale=8, color=G1, lw=0.8, zorder=3))
        eye = (152, -14) if lay == "front" else (pv[0] + f * 15 * math.cos(a), pv[1] - 15 * math.sin(a))
        ax.add_patch(FancyArrowPatch(eye, (230, 22), arrowstyle="-|>", mutation_scale=9, color=K, lw=1.0, zorder=6))
        ax.text(232, 27, "tow to the rover", fontsize=7.4, ha="right", va="bottom")
        ax.add_patch(FancyArrowPatch(tip, (tip[0] - 32, tip[1]), arrowstyle="-|>", mutation_scale=9, color=K, lw=1.4, zorder=6))
        ax.text(tip[0] - 16, tip[1] - 6, "soil on the tip", fontsize=7.4, ha="center", va="top")
        r0 = 62
        if lay == "tail":
            p1 = (pv[0] - r0 * math.cos(a), pv[1] - r0 * math.sin(a))
            p2 = (pv[0] - r0 * math.cos(math.radians(28)), pv[1] - r0 * math.sin(math.radians(28)))
            ax.add_patch(FancyArrowPatch(p1, p2, connectionstyle="arc3,rad=-0.25", arrowstyle="-|>", mutation_scale=10, color=K, lw=1.1, zorder=6))
            ax.text(-10, -45, "the pod turns back\ntoward folded: only\nthe worm holds it", fontsize=7.4, ha="left", va="center")
        else:
            p1 = (pv[0] + r0 * math.cos(a), pv[1] - r0 * math.sin(a))
            p2 = (pv[0] + r0 * math.cos(math.radians(70)), pv[1] - r0 * math.sin(math.radians(70)))
            ax.add_patch(FancyArrowPatch(p1, p2, connectionstyle="arc3,rad=-0.25", arrowstyle="-|>", mutation_scale=10, color=K, lw=1.1, zorder=6))
            ax.text(-10, -45, "the pod turns further\ndown, into the hard\nstop at 50 deg", fontsize=7.4, ha="left", va="center")
        ax.text(-15, 44, title, fontsize=7.5, fontweight="bold", va="bottom")
        ax.set_xlim(-15, 237)
        ax.set_ylim(-102, 56)
    fig.subplots_adjust(wspace=0.04, left=0.01, right=0.99, top=0.98, bottom=0.02)
    return graph("layout_worm_load", fig)


def g_cg_effect():
    keys = [("tail/v33/loam/cg+8", "270 g, CG 8 mm above"), ("tail/v33/loam/cg+0", "270 g, CG on the axis"), ("tail/v33/loam/cg-8", "270 g, CG 8 mm below"),
            ("tail/brief/loam/cg+8", "355 g, CG 8 mm above"), ("tail/brief/loam/cg+0", "355 g, CG on the axis")]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(6.6, 2.5))
    labs = [k[1] for k in keys]
    y = np.arange(len(keys))[::-1]
    a1.barh(y, [BS[k]["righted"] for k, _ in keys], 0.6, color=[K, K, K, G3, G3], edgecolor=K, zorder=3)
    for yy, (k, _) in zip(y, keys):
        a1.text(BS[k]["righted"] + 0.8, yy, str(BS[k]["righted"]), va="center", fontsize=7)
    a1.set_yticks(y, labs)
    a1.set_xlim(0, 58)
    a1.set_xlabel("Righted (of 50 random landings)")
    a1.grid(axis="x", zorder=0)
    med = [BS[k]["median"] or 0 for k, _ in keys]
    slow = [BS[k]["slowest"] or 0 for k, _ in keys]
    a2.barh(y + 0.17, med, 0.32, color=K, label="median", zorder=3)
    a2.barh(y - 0.17, slow, 0.32, color="white", edgecolor=K, linewidth=0.7, label="slowest", zorder=3)
    a2.set_yticks(y, [""] * len(y))
    a2.set_xlabel("Time to belly down, righted runs (s)")
    a2.grid(axis="x", zorder=0)
    a2.set_xlim(0, 50)
    a2.legend(loc="lower right", fontsize=7)
    fig.subplots_adjust(wspace=0.08)
    return graph("cg_height_effect", fig)


# ------------------------------------------------------------------------------------------------ document
class Doc(BaseDocTemplate):
    def __init__(self, fn):
        super().__init__(fn, pagesize=A4, leftMargin=20 * mm, rightMargin=20 * mm, topMargin=18 * mm, bottomMargin=18 * mm,
                         title="KEDGE dart findings, v31 to v33", author="", subject="KEDGE dart design findings", creator="make_findings_report.py")
        fr = Frame(self.leftMargin, self.bottomMargin, self.width, self.height, id="f")
        self.addPageTemplates([PageTemplate(id="p", frames=[fr], onPage=self._foot)])

    def _foot(self, c, doc):
        if doc.page > 1:
            c.setFont("Arial", 8)
            c.drawCentredString(A4[0] / 2, 10 * mm, str(doc.page))

    def afterFlowable(self, fl):
        if isinstance(fl, Paragraph) and fl.style.name == "H1":
            txt = fl.getPlainText()
            self.notify("TOCEntry", (0 if fl.style.name == "H1" else 1, txt, self.page))


def build():
    gp = dict(holding=g_holding(), release=g_release(), spin=g_spin(), mass=g_mass(), hist=g_hist(), tension=g_tension(),
              layout=g_layouts(), cg=g_cg_effect())
    F = lambda n: os.path.join(FIG, n)

    # numbers used in the text
    m = {v: PH[v]["mass_g"] for v in V}
    cg = {v: PH[v]["cg_mm"] for v in V}
    lr33 = loam_row("33")
    tl, fr, ts = RB["tail/v33/loam"], RB["front/v33/loam"], RB["tail/v33/sand"]
    tip = PH["33"]["tip_over"]
    g33 = AN["33"]["geometry"]
    s_lever = (g33["drill"]["entry_xz_mm"][0] - g33["foot"]["entry_xz_mm"][0]) / 1000
    d_res = 0.0215     # depth of the drills' sideways soil resultant below the surface (pile model x_res sin 50, out33 loam)
    h_t = tip["tether_height_mm"] / 1000
    P_ = tip["pull_N"]
    M0, M1 = P_ * h_t, P_ * (h_t + d_res)
    F0, F1 = M0 / s_lever, M1 / s_lever
    q_foot = PH["33"]["tip_over_foot_uplift_capacity_loam_N"]
    rest33 = PH["33"]["landing"]["rest_states"]
    lean_share = sum(r["share_pct"] for r in rest33 if abs(r["roll_deg"]) < 60)
    side_share = sum(r["share_pct"] for r in rest33 if 60 <= abs(r["roll_deg"]) <= 140)
    zc_kick = sum(r[1] * r[2][2] for r in MR["out33"] if r[0] in ("kick_leg", "n20_kick", "n20_foot", "foot_auger")) / m["33"]
    zc_pods = sum(r[1] * r[2][2] for r in MR["out33"] if r[0].startswith("drill_leg_") or r[0] in ("n20_deploy", "coupler_L", "coupler_R", "pod_cap_L", "pod_cap_R", "coupler_L_screw", "coupler_R_screw")) / m["33"]
    zc_top = sum(r[1] * r[2][2] for r in MR["out33"] if r[0] == "hull_top") / m["33"]
    brief_need = 0.23
    foot_g = sum(r[1] for r in MR["out33"] if r[0] in ("foot_auger", "n20_foot"))
    from collections import Counter
    mv_tail = Counter(x["moves"] for x in RR["tail/v33/loam/cg+8"])
    est270 = brief_need * 270 / 355

    # ============================================================== title page
    story.append(Spacer(1, 38 * mm))
    story.append(P("KEDGE dart", TITLE))
    story.append(P("Design findings for v31 to v33, the prototype drill bits and the self-righting simulation study", SUB))
    story.append(Spacer(1, 4 * mm))
    story.append(P(DATE, B))
    story.append(P("GrappStar repository, branch dart-v1. All numbers come from the files listed in Appendix C; where two "
                   "sources disagree the newer one is used and the older one is named.", B))
    story.append(Spacer(1, 12 * mm))
    story.append(P("Contents", H3))
    toc = TableOfContents()
    toc.levelStyles = [ParagraphStyle("toc0", fontName="Arial", fontSize=9.5, leading=13, leftIndent=0),
                       ParagraphStyle("toc1", fontName="Arial", fontSize=8.6, leading=11.2, leftIndent=12)]
    toc.dotsMinLevel = 0
    story.append(toc)
    story.append(PageBreak())

    # ============================================================== 1 summary
    story.append(P("1  Summary of findings and recommendations", H1))
    story.append(P(f"This round took the dart from v31 to v33, made four prototype drill bits for bench tests, and tested "
                   f"landing, self-righting, drilling in and the reel tow in a rigid-body simulation. Main findings:", B))
    story.extend(bullets([
        f"<b>1. Self-righting works at 270 g.</b> With a 40 g righting leg hinged on the top near the nose plus the two pods, "
        f"the simulated dart righted itself from every landing preset and from {tl['righted']} of 50 random landings "
        f"(median {tl['medianS']:.1f} s, slowest {tl['slowestS']:.1f} s, moist loam). At 355 g (255 g body plus 100 g of legs, "
        f"as in the brief) only {RB['tail/brief/loam']['righted']} of 50 righted: every upside-down rest fails, because lifting "
        f"the dart over its nose needs about 0.23 N m and the worm N20 stalls at 0.21 N m.",
        f"<b>2. The tail layout holds best but loads the worms.</b> In loam the tail layout (pivots near the tail, pods fold "
        f"forward, drills under the tether) held the 55 N cap in {tl['heldAtN']['55']} of 50 runs; the front layout held 45 N in "
        f"{fr['heldAtN']['45']} and 55 N in none. But in all 50 tail runs the tow reached a pod worm (peak {tl['wormPeakNm']:.2f} N m, "
        f"above its stall): a forward-folding pod's tip moves aft as it deploys, so the soil pushes it back toward folded and only "
        f"the worm resists. In the front layout the hard stops carry it (2.4 N m) and the worms carry nothing. Fix: a deploy latch.",
        f"<b>3. The centre of mass is too high.</b> v33 has it {cg['33'][2]:.1f} mm above the axis (v31 {cg['31'][2]:.1f}, v32c "
        f"{cg['32'][2]:.1f}). On hard ground the folded dart never rests belly down: it leans about 41 deg on a pod ({lean_share:.0f} % "
        f"of random tumbles). In the simulation, moving it to 8 mm below the axis cut the median righting time from "
        f"{BS['tail/v33/loam/cg+8']['median']:.1f} to {BS['tail/v33/loam/cg-8']['median']:.1f} s; at 355 g, on the axis instead of "
        f"8 mm above raised the righted runs from {BS['tail/brief/loam/cg+8']['righted']} to {BS['tail/brief/loam/cg+0']['righted']} of 50. "
        f"The kick leg, foot motor, foot auger and kick motor alone lift it by {zc_kick:.1f} mm.",
        f"<b>4. Tapered drills cut the torque.</b> Spin torque grows as c x D<super>2</super> x L. The v33 16 mm gimlet needs "
        f"{spin('33'):.3f} N m in loam against {spin('32'):.3f} N m for the v32c 22 mm sleeve, so the in-hand 100 rpm motor "
        f"releases with a margin of {MARGIN['33']:.2f} (pipeline) to about {MARGIN_TAPER:.1f} (hand estimate with the taper), "
        f"against {MARGIN['32']:.2f} for v32c. No 50 rpm motor or tip planetary is needed for loam up to about "
        f"{C_LIM['33']:.0f} kPa cohesion.",
        f"<b>5. The in-line foot does not fit.</b> A straight kick leg from the hull-top hinge steeper than about 24 deg runs "
        f"through the nose, hull or camera (1300 mm<super>3</super> overlap at 45 deg, 707 mm<super>3</super> at 70 deg); at 24 deg "
        f"an in-line foot would need a 190 mm leg to reach 30 mm deep. v33 keeps the v32c vertical foot (67.9 mm in the ground).",
        f"<b>6. The load-path principle.</b> Anchored, the dart must act as one rigid body: no tow load on any motor or motor "
        f"shaft. v33 meets it at the pods (the stops carry {lr33['horn_moment_Nm']:.2f} N m at {hold('33', 'moist loam')} N), "
        f"but not at the kick (in loam the kick worm reaches its assumed 0.6 N m holding limit: {lr33['F_design_N']} N against "
        f"{lr33['F_if_worms_never_slip_N']} N if it never slipped), at the drill augers or at the foot (side and pull-out loads "
        f"reach the motor shafts).",
    ], LI, mark=None))
    story.append(P("<b>Recommended v34</b> (to be designed and audited like every round):", B))
    story.extend(bullets([
        "Keep the v33 tapered drills on the in-hand 100 rpm motors; set the final diameter from the T12, T16 and T20 bench "
        "tests (Section 5); keep the 67 % deploy guard for firm soil.",
        "Either keep the front layout (stops carry the tow), or move to the tail layout for its holding only together with a "
        "deploy latch that holds the pods at 50 deg in both directions, worked by the last degrees of the same deploy worm "
        "(no spring, no extra motor).",
        "Add a catch at the kick plant that holds the leg both ways, so the foot's tow load goes into the nose, not the kick worm.",
        "Carry each drill auger on two journals of its arm, with an axial retainer, so the motor shaft only drives. Do the "
        "same for the foot auger in its housing.",
        "Lower the centre of mass to the axis or below: move the kick and foot motors (now at z 27.5 to 34.3 mm) down; belly "
        "ballast would cost about 140 g and is not a fix.",
        "Add self-righting: test whether the kick leg (41 g, hinge on the top at x 31.3, 200.75 deg swing) can do the "
        "simulated leg's flip, and replace the firmware's 'not upright' refusal with the simulated moves. Keep the mass near "
        "270 g: at 355 g the worm cannot lift the dart.",
        "Keep the first-tow rule: 15 N, then 5 N steps with a 3 s hold, stop on motion.",
    ], LI))
    story.append(PageBreak())

    # ============================================================== 2 design evolution
    h1("2  Design evolution, v31 to v33")
    story.append(P("Three geometry versions were generated, audited and packed this round, each after a round of physics "
                   "numbers and Harry's feedback. Every version passed the full collision audit at 1 deg steps and the "
                   f"launcher fit over the real launcher's 1684 states. Table {tno('key')} lists the key numbers."))
    h2("2.1  v31: anchoring fixes on the v30 body")
    story.append(P("v31 (generator kedge_v27.py, out31) started from v30, the dart the launcher is sized to. The first physics "
                   "run showed that under tow the pods turned back through the deploy worm, which capped the loam holding "
                   "near 23 N. Instead of the 3:1 gear stage dart-physics asked for (its 36-tooth gear would reach z 20.5 under "
                   "a bay roof at 19.5, and m0.5 teeth would see about 140 MPa), v31 got carved deploy stop horns and a kick rest, "
                   "so the tow moment goes horn to hull and never through a worm. Other changes: 13 mm radial augers (core 6, "
                   "pitch 11, 72.8 long on F693ZZ bearing pairs; install torque falls with diameter while the loam holding "
                   "stays the same), pods moved in to y 29, PPA-CF couplers printed flat, a keel wire channel, the XIAO turned "
                   "so its USB-C is reachable through a hull window, and no ballast (it did nothing for stability at 7 m/s)."))
    story.append(P("Harry's feedback on v31 set v32: put the drill motors inside the augers, put the kick leg on the "
                   "centreline, and take the tether from a tail boom. He asked for a 50 mm boom; the launcher allows 28 mm "
                   "(rail rear end and top pulley at full cock), so base needs 22 mm more room for a 50 mm boom."))
    h2("2.2  v32 and v32c: motors inside sleeve augers, centred kick, tail boom")
    story.append(P("In v32 (kedge_v32.py, out32) each drill gearmotor sits inside a hollow sleeve auger: flights 22 mm on an "
                   "18.7 mm core (the bore 16.3 clears the 15.62 mm gearbox diagonal), pitch 15, 100 mm long, deploying 45 deg "
                   "from pivots at x 56, z 12. The kick moved to the centreline with a 68 mm foot auger, and a 28 mm PPA-CF "
                   "boom with a flush steel M3 pin carries the tether into the hull through an anchor disc and the four tail "
                   "screws. Physics then found the weak point: the 22 mm sleeve needs "
                   f"{spin('32'):.3f} N m to spin in loam, so releasing after standing needs {unscrew('32')['drill_unscrew_torque_Nm']:.3f} N m "
                   f"against the drill's {STALL:.3f} N m stall: margin {MARGIN['32']:.2f}, releasing only up to about "
                   f"{C_LIM['32']:.0f} kPa cohesion. A 2.33:1 star planetary in the tip hub would give {MARGIN_PLANET:.2f}, but it "
                   "was not built (1.5 mm steel planet pins are not in Harry's parts, m0.5 teeth want a 0.2 mm nozzle, and the sun "
                   "would need a jam nut pair on the round M3 thread). As built, v32 drives the sleeve directly, and the firmware "
                   "guard at 67 % of the drill stall is the release safety."))
    story.append(P("v32c was a polish round to Harry's standing rules (smooth tangent surfaces, nothing floating, every part "
                   "explained): kick pivot 5 mm lower (z 27.5), plant at 200.75 deg; the foot housing tilted 20.75 deg on the "
                   "leg so the foot auger plants vertical, 67.9 mm deep (was 59.3 mm at 24.5 deg off vertical); the planted "
                   "pole lies in a round-cornered flute in the nose top whose floor is the kick rest; saddle, ear and pivot "
                   "blocks became tangent-arc outlines with r1.2 edges and r1.5 skirts; the kick N20's unused stub was cut "
                   "inside its pocket, so no slot and no zip tie. Mass 286.8 g, centre of mass x 87, z +8.2."))
    h2("2.3  v33: Harry's tapered drills")
    story.append(P("v33 (kedge_v33.py, out33) is v32c with Harry's tapered drills. Each motor still sits in its auger, but "
                   "only in a short hub (18.7 mm, from 8.7 to 36.5 mm off the pivot); a stationary housing would be no slimmer, "
                   "because the gearbox diagonal sets about 18.7 mm either way. Past the hub a 9 mm tangent S-curve neck runs "
                   "into a gimlet: core 9.6 mm tapering to 6.0 mm at 87.3 mm, then a 14 mm point to 101.3 mm. The anchor "
                   "(buttress) flight is 16 mm across, pitch 12, full height from 45.5 to 79.3 mm, with the face toward the motor "
                   "at 45 deg (prints point up with no supports), the leading face at 30 deg and a 0.6 mm crest land; left pod "
                   "right-hand, right pod left-hand. The motor's M3 shaft is a steel spine: it passes a nut pressed into the hub "
                   "roof at 40.5 mm and self-taps a 2.6 mm bore to 92.0 mm. Deploy went from 45 to 50 deg (arm 2.0 mm above "
                   "the ground, drills 44.8 mm deep). The foot became a gimlet too (13 mm, pitch 11, core 7 to 4 mm)."))
    story.append(P("Harry asked for the foot in line with a straight kick leg that folds flat on the hull top, swings about "
                   "250 deg and plants 60 to 75 deg ahead of the nose. It was not built: with one hinge on the hull top "
                   "(x 31.3, z 27.5), 'folds flat' means 'in line with the leg', and any straight leg steeper than about 24 deg "
                   f"runs through the nose, hull or camera (Figure {fno('kick')}). The options left for Harry are: (a) keep the vertical "
                   "foot (v33 does); (b) tilt the foot to plant 70 deg forward on the same leg (only 13 mm lower folded, past "
                   "the launcher rail end, 44.6 mm deep: not worth it); (c) a straight leg with the hinge on a tower at the nose "
                   "front (in the impact zone and in front of the camera); (d) a second, passive joint at the foot."))
    r = lambda v, s: f"{hold(v, s):.1f}"
    bb = {v: AU[v]["folded_bbox"] for v in V}
    table([
        ["", "v31", "v32c", "v33"],
        ["Generator, output", "kedge_v27.py, out31", "kedge_v32.py, out32", "kedge_v33.py, out33"],
        ["Mass (g): total, printed, bought", f"{m['31']}, {PH['31']['printed_g']}, {PH['31']['bought_g']}", f"{m['32']}, {PH['32']['printed_g']}, {PH['32']['bought_g']}", f"{m['33']}, {PH['33']['printed_g']}, {PH['33']['bought_g']}"],
        ["Centre of mass x, z (mm)", f"{cg['31'][0]}, +{cg['31'][2]}", f"{cg['32'][0]}, +{cg['32'][2]}", f"{cg['33'][0]}, +{cg['33'][2]}"],
        ["Drill auger", "13 mm radial, core 6, pitch 11, 72.8 long", "22 mm sleeve on an 18.7 core, pitch 15, 100 long", "16 mm tapered gimlet, pitch 12, core 9.6 to 6.0, point at 101.3"],
        ["Drill depth (mm): audit, physics", f"{AU['31']['auger_depth_mm']['drill_leg_L_auger']}, {AN['31']['geometry']['drill']['depth_mm']}", f"{AU['32']['auger_depth_mm']['drill_leg_L_auger']}, {AN['32']['geometry']['drill']['depth_mm']}", f"{AU['33']['auger_depth_mm']['drill_leg_L_auger']}, {AN['33']['geometry']['drill']['depth_mm']}"],
        ["Deploy angle (deg)", f"{AU['31']['deploy_range'][1]}", f"{AU['32']['deploy_range'][1]}", f"{AU['33']['deploy_range'][1]}"],
        ["Foot auger, depth (mm)", f"13 mm, {AU['31']['auger_depth_mm']['foot_auger']}", f"13 x 68 mm vertical, {AU['32']['auger_depth_mm']['foot_auger']}", f"13 mm gimlet vertical, {AU['33']['auger_depth_mm']['foot_auger']}"],
        ["Printed volume (cm<super>3</super>)", f"{AU['31']['printed_cm3']}", f"{AU['32']['printed_cm3']}", f"{AU['33']['printed_cm3']}"],
        ["Folded envelope x, y, z (mm)", f"0 to {bb['31'][3]}, +-{bb['31'][4]}, -21 to {bb['31'][5]}", f"0 to {bb['32'][3]}, +-{bb['32'][4]}, -21 to {bb['32'][5]}", f"0 to {bb['33'][3]}, +-{bb['33'][4]}, -21 to {bb['33'][5]}"],
        ["Holding at 3 m (N): loam, sand", f"{r('31', 'moist loam')}, {r('31', 'dry sand')}", f"{r('32', 'moist loam')}, {r('32', 'dry sand')}", f"{r('33', 'moist loam')}, {r('33', 'dry sand')}"],
        ["Loam holding without the foot (N)", f"{PH['31']['tip_over']['without_foot_N_loam_3m']}", f"{PH['32']['tip_over']['without_foot_N_loam_3m']}", f"{PH['33']['tip_over']['without_foot_N_loam_3m']}"],
        ["Release margin, loam c 5 kPa, 100 rpm", f"{MARGIN['31']:.2f}", f"{MARGIN['32']:.2f} ({MARGIN_PLANET:.2f} with the planetary, not built)", f"{MARGIN['33']:.2f} pipeline, about {MARGIN_TAPER:.1f} with the taper"],
        ["Milling deploy in loam (s)", f"{AN['31']['soils']['moist loam']['deploy']['time_to_deploy_s']}", f"{AN['32']['soils']['moist loam']['deploy']['time_to_deploy_s']}", f"{AN['33']['soils']['moist loam']['deploy']['time_to_deploy_s']}"],
        ["Collision audit (kick, deploy steps)", f"CLEAN ({AU['31']['kick_steps']}, {AU['31']['deploy_steps']})", f"CLEAN ({AU['32']['kick_steps']}, {AU['32']['deploy_steps']})", f"CLEAN at 1 deg ({AU['33']['kick_steps']}, {AU['33']['deploy_steps']})"],
        ["Launcher fit (1684 states)", "FITS, no overlaps", "FITS; only base's old tether line model overlaps", "FITS; same as v32c"],
    ], [43, 40, 44, 43], "Key numbers per version. Physics from out31..out33/physics.json and phys31/anchor.json; geometry from "
       "audit.json. v32c replaces v32b in out32: DART_PHYSICS32.md (288.6 g, loam 73.8 N) was written on v32b and is older than "
       "out32/physics.json (286.8 g, 73.2 N). The v31 note 'FINAL v31 run' (51.1 N) is older than out31/physics.json (45.5 N, "
       "after the near-surface cohesion change).", label="key")
    figure([F("cad_v31_deployed_side.png"), F("cad_v32b_deployed_side.png")],
           "Deployed side views. (a) v31: radial 13 mm augers, offset kick, tether holes in the tail cap. (b) v32b: drill motors "
           "inside 22 mm sleeves, centred kick with the foot 24.5 deg off vertical, tail boom.", crops=[(150, 150, 1300, 820), (0, 120, 1100, 700)])
    figure([F("cad_v32c_deployed_side.png"), F("cad_v33_deployed_side.png")],
           "Deployed side views. (a) v32c: lower kick pivot, foot vertical, 67.9 mm deep. (b) v33: tapered drills deployed 50 deg, "
           "gimlet foot.", crops=[(0, 120, 1100, 700), (0, 120, 1100, 700)])
    figure(F("cad_v31_audit_sheet.png"), "v31 audit sheet: 8 folded, 8 deployed and 6 mid-motion views, top and underside "
           "(out31/audit_sheet.png).", maxw=2200)
    figure([F("cad_v32c_final.png"), F("cad_v33_sheet.png")],
           "Review sheets. (a) v32c final: folded and deployed views and close-ups of the pod heads, pivot blocks and kick hinge "
           "(out32/review/v32c_final.png). "
           "(b) v33: side and three-quarter views, the folded drill, the in-line kick check and the viewer page "
           "(out33/review/v33_sheet.png).", maxw=1400)
    figure(F("cad_v33_27_kick_inline_check.png"), "Why the in-line foot does not fit (out33/review/27_kick_inline_check.png): "
           "straight legs from the hull-top hinge at x 31.3, z 27.5. At 22 deg the overlap is 20 mm<super>3</super>; at 45 deg "
           "1300 mm<super>3</super> and at 70 deg 707 mm<super>3</super> run through the nose, hull or camera.", width_mm=135, label="kick")
    figure([F("cad_v33_16_auger_hub_folded.png"), F("cad_v33_17_auger_right_low.png")],
           "v33 drill close-ups. (a) The short hub round the motor, the S-curve neck and the buttress flight, folded. (b) The "
           "right pod from low down: the gimlet point and the 45/30 deg flight faces.", maxw=1100)
    figure([F("cad_v33_25_pod_section.png"), F("cad_v33_24_foot_gimlet.png")],
           "(a) Section through a v33 pod: the drill gearmotor inside the hub and its M3 shaft running up the gimlet as a steel "
           "spine. (b) The v33 gimlet foot auger.", maxw=1100)
    h2("2.4  Mass and centre of mass")
    story.append(P(f"Figure {fno('mass')} breaks the mass down. v32c was the heaviest ({m['32']} g) because each 22 mm sleeve weighed "
                   f"14.1 g; the v33 tapered auger is 8.15 g, which brought the dart back to {m['33']} g and moved the centre of "
                   f"mass forward to x {cg['33'][0]}. Bought parts stay near 147 g, of which the two cells are 44 g and the five "
                   f"motors 53.5 g. The centre of mass height comes from what sits on top: the kick assembly (leg, kick motor, "
                   f"foot motor and foot auger) lifts it by {zc_kick:.1f} mm, the pods and deploy motor (pivots at z 12) by "
                   f"{zc_pods:.1f} mm and the hull top by {zc_top:.1f} mm, while the hull bottom and tray pull it down by about 0.9 mm. "
                   f"To bring it to the axis the kick and foot motors must come down; ballast at z -15 would need about "
                   f"{m['33'] * cg['33'][2] / 15:.0f} g."))
    figure(gp["mass"], "Mass breakdown per version, from the dart_model mass rows (data/mass_rows.json). Printed parts at 4 walls "
           "(1.7 mm) and 40 % infill in PLA, bought parts at catalogue values (weigh them on the day).", jpeg=False, maxw=2000, label="mass", width_mm=150)

    # ============================================================== 3 anchoring physics
    h1("3  Anchoring physics")
    story.append(P("The anchoring numbers come from dart_anchor.py on each version's geometry: the drills and the foot are short "
                   "rigid piles (Broms in sand, Matlock near-surface cohesion rising from 3 c D to 9 c D), the axial grip is "
                   "shear on the flight-tip cylinder, and a planar limit analysis finds the largest tether pull the hull, pods "
                   "and kick leg can take together with their ground contacts. The rover is 3 m away (line 7.3 deg). Soils: dry "
                   "sand (16 kN/m<super>3</super>, phi 32 deg, c 0), moist loam (18 kN/m<super>3</super>, phi 28 deg, c 5 kPa) "
                   "and firm clay (19 kN/m<super>3</super>, c 40 kPa). Motor torques come from the bench currents at 7.4 V: worm "
                   f"N20 {WORM:.3f} N m, drill gearmotor {STALL:.3f} N m."))
    h2("3.1  Holding by soil and version")
    rows = [["Soil", "v31", "v32c", "v33", "v33: each drill / foot / friction (N)"]]
    for s in SOILS:
        b = PH["33"]["anchoring"]["by_soil"][s]
        rows.append([s, f1(hold("31", s)), f1(hold("32", s)), f1(hold("33", s)), f"{b['drill_auger_each_N']} / {b['foot_auger_N']} / {b['friction_N']}"])
    rows.append(["moist loam, no foot", f1(PH["31"]["tip_over"]["without_foot_N_loam_3m"]), f1(PH["32"]["tip_over"]["without_foot_N_loam_3m"]), f1(PH["33"]["tip_over"]["without_foot_N_loam_3m"]), ""])
    rows.append(["moist loam, rover 1 m", f1(PH["31"]["tip_over"]["loam_rover_1m_N"]), f1(PH["32"]["tip_over"]["loam_rover_1m_N"]), f1(PH["33"]["tip_over"]["loam_rover_1m_N"]), "line at 21 deg"])
    table(rows, [38, 22, 22, 22, 66], "Holding at the dart (N), rover 3 m unless stated. Firm clay is what the anchor would hold "
          "if it could go in: it cannot (the v33 drills need 0.66 N m to spin in clay against 0.182 N m).")
    figure(gp["holding"], "Holding by soil and version (physics.json, by_soil total_N). Loam holds the 15 N first tow in every "
           "version and the 55 N cap in v32c and v33; dry sand holds neither, whatever the auger.", jpeg=False, maxw=2000, width_mm=150)
    story.append(P(f"v33 holds less than v32c in loam ({hold('33', 'moist loam')} against {hold('32', 'moist loam')} N) because the "
                   f"drills are thinner (16 against 22 mm; the sideways soil pressure scales with D) and their embedded length is "
                   f"shorter ({AN['33']['geometry']['drill']['embedded_effective_mm']} against "
                   f"{AN['32']['geometry']['drill']['embedded_effective_mm']} mm). It still clears the 55 N cap. Holding also "
                   f"depends on how far the pods got: at 20, 30, 40 and 50 deg the v33 loam holding is "
                   + ", ".join(f"{h:.1f}" for a_, h in AN["33"]["holding_vs_deploy_angle_rover3m"]["moist loam"] if a_ in (20, 30, 40, 50))
                   + " N."))
    h2("3.2  Nose lift and the foot couple")
    story.append(P("The tether pulls at the tail, about h above the soil; the drills resist sideways at a depth d below the "
                   "surface. The pull therefore makes a moment that lifts the nose, and the planted foot ahead of the nose has "
                   "to hold it down with a couple over the lever s from the drills to the foot:"))
    eq("M = P x (h + d)")
    eq("F = M / s")
    story.append(P(f"For v33 in moist loam at the holding limit, P = {P_} N, h = {tip['tether_height_mm']} mm (the line on the axis, "
                   f"21 mm above the soil), s = {s_lever * 1000:.1f} mm (drill entry at x {g33['drill']['entry_xz_mm'][0]} to foot entry "
                   f"at x {g33['foot']['entry_xz_mm'][0]}), and d = {d_res * 1000:.1f} mm (the depth of the drills' sideways soil "
                   f"resultant from the same pile model, {AN['33']['geometry']['drill']['embedded_effective_mm']} mm embedded at 50 deg). "
                   "physics.py takes moments about the drill entry at the surface (d = 0), which assumes the drills carry their own "
                   "moment in the soil; taking them about the resultant (no moment in the drills) is the upper bound:"))
    eq(f"d = 0:            M = {P_} N x {h_t:.3f} m = {M0:.2f} N m,      F = {M0:.2f} N m / {s_lever:.3f} m = {F0:.1f} N")
    eq(f"d = {d_res * 1000:.1f} mm:   M = {P_} N x ({h_t:.3f} + {d_res:.4f}) m = {M1:.2f} N m,      F = {M1:.2f} / {s_lever:.3f} = {F1:.1f} N")
    story.append(P(f"The foot's pull-out capacity in loam is {q_foot} N and the dart's weight ({tip['dart_weight_N']} N) acts almost "
                   f"over the drills, so it helps little. The real demand lies between the two numbers; the full limit analysis "
                   f"(which spreads the soil pressure along each drill) has the foot at its soil capacity as one of the governing "
                   f"limits. Without the foot the nose lifts and the loam holding falls from {hold('33', 'moist loam')} to "
                   f"{PH['33']['tip_over']['without_foot_N_loam_3m']} N, so the foot auger and its motor ({foot_g:.1f} g) earn their place."))
    h2("3.3  Drill torque scales as c x D<super>2</super> x L")
    story.append(P("A radial drill gets no depth from screwing (its axis passes through the pod pivot), so the motor only has to "
                   "spin it in place against shear on the cylinder through the flight tips:"))
    eq("T = (pi / 2) x D<super>2</super> x L x (alpha x c + K x gamma x z x tan phi)")
    story.append(P("with alpha 0.8, K 1.0 and z the mean depth. The cohesion term dominates in loam, so the torque goes as "
                   "c x D<super>2</super> x L; pitch and flight thickness do not change it. Releasing after the soil has set "
                   "needs 1.3 times the spin torque. Substituted for each version in moist loam (c 5 kPa, gamma 18 kN/m<super>3</super>, "
                   "phi 28 deg):"))
    for v, ang in (("31", 43), ("32", 45), ("33", 50)):
        g_ = AN[v]["geometry"]["drill"]
        D_, L_ = g_["D_mm"] / 1000, g_["embedded_effective_mm"] / 1000
        z_ = L_ / 2 * math.sin(math.radians(g_["angle_deg"]))
        tau = 0.8 * 5000 + 18000 * z_ * math.tan(math.radians(28))
        T_ = math.pi / 2 * D_ ** 2 * L_ * tau
        eq(f"{VN[v]}: T = 1.571 x {D_:.3f}<super>2</super> x {L_:.4f} x ({0.8 * 5000:.0f} + {tau - 4000:.0f}) Pa = {T_:.3f} N m;   "
           f"release 1.3 x {spin(v):.3f} = {unscrew(v)['drill_unscrew_torque_Nm']:.3f} N m;   margin {STALL:.3f} / "
           f"{unscrew(v)['drill_unscrew_torque_Nm']:.3f} = {MARGIN[v]:.2f}")
    eq(f"v32c with the 2.33:1 planetary: stall at the sleeve {STALL:.3f} x 2.33 x 0.85 = {PLANET:.3f} N m;   margin "
       f"{PLANET:.3f} / {unscrew('32')['drill_unscrew_torque_Nm']:.3f} = {MARGIN_PLANET:.2f}")
    eq(f"v33 with the taper (hand estimate, 0.35 of the v32c torque): T = 0.35 x {spin('32'):.3f} = {TAPER * spin('32'):.3f} N m;   "
       f"margin {STALL:.3f} / (1.3 x {TAPER * spin('32'):.3f}) = {MARGIN_TAPER:.2f}")
    story.append(P(f"The pipeline number for v33 ({MARGIN['33']:.2f}) treats the drill as a uniform 16 mm flight over its "
                   f"{AN['33']['geometry']['drill']['embedded_effective_mm']} mm embedded length on the mean core; the hand estimate "
                   f"({MARGIN_TAPER:.1f}, CLAUDE.md section 5) counts the taper. Both come from the same day (7 Nov); the pipeline ran "
                   "on the finished out33 and is the conservative one, the hand estimate still waits for a dart-physics check. The "
                   "bench tests in Section 5 settle it."))
    figure(gp["spin"], "Spin torque against flight diameter in moist loam: the D<super>2</super> scaling. Lines scale the "
           "pipeline torques of v32c and v33 at their embedded lengths; squares are the v32 sizing runs (OD 19, 20, 22 mm), "
           "circles the three versions, open circles the prototype bits predicted at the v33 embedded length. Above the "
           "dash-dot line a drill cannot release after standing on the 100 rpm motor.", jpeg=False, maxw=2000, width_mm=155)
    figure(gp["release"], "Release margin after standing against soil cohesion, for each drill option. Computed from the "
           "cohesion rows of anchor.json (spin torque at c 2, 5, 10 and 20 kPa, interpolated) as stall / (1.3 x spin "
           "torque); the planetary line uses the geared stall 0.361 N m and the taper line 0.35 x the v32c torque.",
           jpeg=False, maxw=2000, label="release", width_mm=155)
    story.append(P(f"The v33 pipeline line lies almost on the line of v32c with the 2.33:1 planetary ({MARGIN['33']:.2f} against "
                   f"{MARGIN_PLANET:.2f} at 5 kPa): the thinner tapered flight buys what the gear stage would have, with no gears, "
                   f"pins or extra parts."))
    rows = [["Drill option", "Margin at c 5 kPa", "Releases after standing up to c (kPa)"],
            ["v31 13 mm radial, 100 rpm", f"{MARGIN['31']:.2f}", f"{C_LIM['31']:.1f}"],
            ["v32c 22 mm sleeve, 100 rpm (as built)", f"{MARGIN['32']:.2f}", f"{C_LIM['32']:.1f}"],
            ["v32c 22 mm sleeve + 2.33:1 planetary (not built)", f"{MARGIN_PLANET:.2f}", f"{C_LIM['32p']:.1f}"],
            ["v33 16 mm tapered, pipeline", f"{MARGIN['33']:.2f}", f"{C_LIM['33']:.1f}"],
            ["v33 16 mm tapered, taper estimate", f"{MARGIN_TAPER:.2f}", f"{C_LIM['33t']:.1f}"]]
    table(rows, [80, 38, 52], "Release margins and the cohesion where each drill stops being able to back out after standing "
          f"(margin 1), from the curves in Figure {fno('release')}.", label="margins")
    h2("3.4  Milling deploy")
    d33 = AN["33"]["soils"]["moist loam"]["deploy"]
    story.append(P(f"The radial drills sweep sideways through the soil as the pods swing down. Sweeping both v33 drills without "
                   f"spinning needs {d33['passive_moment_both_pods_at_full_Nm']} N m in loam against the deploy worm's {WORM:.3f} N m, "
                   f"{d33['ratio_needed_over_available']} times too much, so they go in only as a mill: the drills spin and the worm "
                   f"pushes at stall, and the feed per drill turn settles where the worm torque balances the cut:"))
    eq(f"f / D = T<sub>worm</sub> / M<sub>passive</sub> = {WORM:.3f} / {d33['passive_moment_both_pods_at_full_Nm']} = {d33['milling_feed_over_D']:.4f};   "
       f"f = {d33['milling_feed_over_D']:.4f} x 16 mm = {d33['milling_feed_over_D'] * 16:.2f} mm per drill turn")
    story.append(P(f"The tip has {d33['tip_arc_in_soil_mm']} mm of arc to cut (the tip touches the ground at "
                   f"{g33['drill']['tip_touches_ground_at_deg']} deg and goes on to 50), about "
                   f"{d33['tip_arc_in_soil_mm'] / (d33['milling_feed_over_D'] * 16):.0f} drill turns; the model gives "
                   f"{d33['time_to_deploy_s']} s. While milling the drills work at {d33['drill_torque_while_milling_Nm']} N m "
                   f"(margin {d33['drill_margin']}), under the 67 % guard line of {0.67 * STALL:.3f} N m, so the guard only acts in "
                   f"firmer soil. Heating over a 50 s loam mill: drills +{AN['33']['motor_heating_loam']['cases']['100 rpm drills']['drill_rise_K']:.0f} K, "
                   f"deploy worm +{AN['33']['motor_heating_loam']['cases']['100 rpm drills']['worm_rise_K']:.0f} K (PLA softens near 60 C); "
                   "the firmware caps a mill at 150 s and then rests 60 s."))
    rows = [["Deploy time (s)", "v31", "v32c", "v33"]]
    for s in SOILS:
        rows.append([s] + [f"{AN[v]['soils'][s]['deploy']['time_to_deploy_s']}" + (" (never: drills stall)" if s == "firm clay" else "") for v in V])
    rows.append(["drill margin while milling loam"] + [f"{AN[v]['soils']['moist loam']['deploy']['drill_margin']}" for v in V])
    table(rows, [50, 40, 40, 40], "Milling deploy times. In firm clay the drills stall (margin 0.1 to 0.3), so the clay times are "
          "never reached.")
    h2("3.5  Cohesion sensitivity")
    rows = [["Loam cohesion c (kPa)", "Holding v31 (N)", "Holding v32c (N)", "Holding v33 (N)", "v33 drill spin torque (N m)", "v33 stop moment, both pods (N m)"]]
    c31, c32, c33 = (AN[v]["loam_cohesion_sensitivity_rover3m"] for v in V)
    for a_, b_, c_ in zip(c31, c32, c33):
        rows.append([f"{c_['c_kPa']:.0f}", f1(a_["F_N"]), f1(b_["F_N"]), f1(c_["F_N"]), f"{c_['drill_install_Nm']:.3f}", f"{c_['horn_Nm']:.2f}"])
    table(rows, [30, 26, 26, 26, 30, 32], "Loam holding at 3 m against cohesion. Holding grows almost in proportion to c, and so "
          "does the drill torque; the pull test (DART_PULLTEST.md) sets the real c. Until then use the 5 kPa row.", label="coh")
    h2("3.6  First tow")
    story.append(P(f"Tow rule for base and the Jetson: first tow at {AN['33']['assumptions']['first_tow_N']:.0f} N, then 5 N steps "
                   "with a 3 s hold; stop as soon as the dart reports motion (pitch change over 2 deg, or 0.15 g for 200 ms: "
                   "'moving' in /status); never go above the pull-test force at 5 mm of movement divided by 1.5, and never "
                   f"above the {AN['33']['assumptions']['tow_cap_N']:.0f} N cap. The old 40 N first tow would drag a loam anchor of the "
                   f"v31 kind and any sand anchor: v33 holds only {hold('33', 'dry sand')} N in dry sand, so in sand the first 15 N tow "
                   "already drags it. The simulation's tow uses the same steps (Section 6)."))

    # ============================================================== 4 load path
    h1("4  The load-path principle")
    story.append(P("Once anchored, the dart must act as one rigid body between the soil and the tether: every load from the tow "
                   "goes from the drills and the foot through printed parts, pins and hard stops into the hull and out through "
                   "the boom, and no load passes through any motor, gearbox or motor shaft. A worm gearbox's holding strength is "
                   "unknown (0.6 N m is assumed until bench test B2), a motor shaft is not a bearing, and a load that reaches a "
                   f"motor can back-drive, bend or pull it. Table {tno('loadpath')} checks each joint of v33 against the rule, from the physics "
                   "limit analysis and the CAD."))
    table([
        ["Joint", "Turning load (about the joint)", "Side and pull loads", "v33 today", "Fix"],
        ["Drill pods (one deploy worm, PPA-CF D couplers on its two shaft ends)",
         f"Tow turns the pods further down (they fold aft), into the carved pivot-block stops: {lr33['horn_moment_Nm']:.2f} N m for both "
         f"pods at {lr33['F_design_N']} N in loam. Checked: bearing 13 MPa, stop root 19 MPa, safety factor 2.4 (at 3.3 N m per pod).",
         "Pod bore on the coupler, coupler in the hull boss. Squeezed between stop and soil the coupler carries about 330 N near the "
         "boss (bending about 46 MPa, PPA-CF, factor about 2).",
         "Meets the rule at full deploy. Not when the 67 % guard stops the pods part way: then they are off the stops and the "
         "tow moment goes into the deploy worm.",
         "Keep the stops. For a depth-limited anchor, a latch or ratchet that holds at any angle; for the tail layout, a deploy "
         "latch (Section 6.6)."],
        ["Kick leg (hub on the kick N20's output stub, F693ZZ in the ear)",
         f"Sand: the foot is pushed past the plant into the flute rest ({next(r_ for r_ in AN['33']['holding_limit']['dry sand'] if r_['rover_m'] == 3.0)['kick_rest_moment_Nm']} N m). "
         f"Loam: the foot is pushed back toward folded, the rest carries 0 and the kick worm is at its assumed 0.6 N m limit "
         f"({lr33['F_design_N']} N against {lr33['F_if_worms_never_slip_N']} N if it never slipped).",
         "Leg loads go into the N20's 3 mm output shaft and the ear bearing (landing: 21 N on the shaft in soft soil).",
         "Fails in loam (worm loaded); the motor shaft is a pivot.",
         "A catch at the plant that holds the pole in the flute both ways; leg hub on a shaft carried by the ear and the saddle, "
         "the N20 driving it through a D only."],
        ["Drill augers (motor in the hub, M3 shaft as the spine)",
         "Spin and release torque: the motor's own job. The tow does not turn a radial auger.",
         "The hub runs on one 17.7 mm journal of the arm at its root. Side load on the gimlet (up to "
         f"{AN['33']['soils']['moist loam']['drill_each']['H_translation_N']} N per drill in loam) makes a moment one journal cannot "
         "take: the rest goes into the M3 shaft and the gearbox output bearing. Pull-out goes through the pressed nut and 49 mm of "
         "self-cut thread into the gearbox; push toward the motor lands on the arm shoulder after the half-turn gap closes.",
         "Fails: side and pull-out loads reach the motor shaft.",
         "Two journals on the arm (root and gearbox end) and an axial retainer, so the shaft only drives (key or hex). Today the "
         "hub bore clears the gearbox diagonal by 0.34 mm, so the outer journal needs a new arm shape."],
        ["Foot auger (worm output shaft, D bore and grub screw)",
         "Spun by the foot N20. The tow does not turn it.",
         f"Uplift from the nose lift ({F0:.1f} to {F1:.1f} N, Section 3.2) and side load ({AN['33']['soils']['moist loam']['foot']['H_translation_N']} N "
         "in loam) go through the grub screw and D bore into the worm output shaft. Push down runs 0.3 mm over the housing.",
         "Fails for uplift and side load.",
         "A boss on the housing round the auger hub, and a retainer, so only torque reaches the shaft."],
        ["Tail boom and tether",
         "None.",
         "Boom, anchor disc, four M3 screws, hull. At 3 x 55 N, 36 deg off the axis: 40 MPa at the root, factor 2.0 printed lying.",
         "Meets the rule.", "Print it lying (layers along the boom)."],
    ], [24, 38, 42, 30, 36], "Load path of each v33 joint. Numbers from out33/phys31/anchor.json (holding_limit, rover 3 m), "
       "out32/phys31/v32_checks.json (boom) and the dart-cad strength checks in DART_V31_NOTES.md.", fs=7.2, keep=False, label="loadpath")

    # ============================================================== 5 prototype bits
    h1("5  Prototype drill bits")
    story.append(P("Four test bits (kedge/proto/drill_proto.py, STLs in kedge/proto/out) let Harry choose the drill size on the "
                   "bench and in a pot of soil before it is fixed in the dart. Each bit screws straight onto the drill gearmotor's "
                   "M3 threaded shaft (2.84 x 56 mm past the face), which runs up its middle as a steel spine, the same way the "
                   "v33 augers are built."))
    h2("5.1  Geometry")
    story.extend(bullets([
        "Length 72 mm; the core and flight taper over the last 26 mm to a sharp point (tip radius 0.5 mm).",
        "Base hub 11 mm across and 4 mm tall, a 3 mm shoulder up to the core; core 6.8 to 7.5 mm.",
        "Buttress (anchor) flight: the face toward the motor takes the pull-out load and is 45 deg, so the bit prints tip up "
        "with no supports; the leading face toward the tip is 30 deg so it cuts in; 0.6 mm crest land; the flight depth fades "
        "in and out over a quarter turn.",
        "Spine: a 2.6 mm bore 54 mm deep, which the M3 shaft taps itself (the shaft is 56 mm long; the jam nut takes 2.4 mm).",
        "Lock in both directions: an M3 nut pressed into a 5.65 mm hex pocket (2.6 deep) in the base, then a second M3 nut on the "
        "shaft jammed up against the base, with a drop of threadlocker.",
    ]))
    rows = [["Bit", "Flight OD (mm)", "Pitch (mm)", "Core (mm)", "Volume (cm<super>3</super>), solid PLA (g)", "Spin torque in loam, scaled (N m)", "Release margin, 100 rpm", "Use"]]
    bits = [("T12_RH", 12, 9, 6.8, 2.93, "slim: least torque"), ("T16_RH", 16, 12, 7.0, 4.24, "main candidate"),
            ("T16_LH", 16, 12, 7.0, 4.24, "mirror for the other pod"), ("T20_RH", 20, 15, 7.5, 5.96, "wide: most holding")]
    for nm, od, pitch, core, vol, use in bits:
        t = spin("33") * (od / 16) ** 2
        rows.append([nm, od, pitch, core, f"{vol}, {vol * 1.24:.1f}", f"{t:.3f}", f"{STALL / (SETUP * t):.2f}", use])
    table(rows, [16, 16, 14, 14, 30, 28, 24, 28], "The four bits. Volumes from the STLs; torque scaled from the v33 pipeline value "
          f"{spin('33'):.3f} N m by (D / 16)<super>2</super> at the v33 embedded length, margin = {STALL:.3f} / (1.3 x T). The "
          "drills' share of the holding scales about with D (sideways soil pressure is proportional to c x D).")
    figure(F("proto_bits.png"), "The prototype bits T12, T16 (right- and left-hand) and T20, and the hand grip for the gearmotor "
           "(kedge/proto/out/proto_bits.png).", crops=[(60, 160, 1720, 560)], width_mm=160)
    h2("5.2  Printing and fitting")
    story.append(P("PLA, 0.16 mm layers, 100 % infill (or 6 walls), standing on the base with the tip up, no supports. The grip "
                   "prints standing on its T-bar, also without supports. Fit: press an M3 nut into the hex pocket; run a second M3 "
                   "nut down the motor shaft to the motor face; screw the bit on until it is fully home (the shaft cuts its own "
                   "thread); turn the shaft nut back up against the base and tighten it."))
    h2("5.3  Test procedure and what each test decides")
    table([
        ["Step", "What to do", "What it decides"],
        ["1  Direction", "Gearmotor in the grip, bench supply at 8 V through the INA219. A right-hand bit drills in turning "
         "clockwise seen from the motor; swap the leads if not.", "The wiring sense per pod (left pod right-hand, right pod left-hand)."],
        ["2  Drive in", "Drill each bit into a pot of damp garden soil, then dry sand. Note the time to full depth and the current, "
         "or whether it stalls.", "Install torque per size: T is about 0.6 x 1.04 N m/A x (I - 0.03 A) from the motor model. "
         "Compare with the table above; the largest bit that goes in without stalling stays in the running."],
        ["3  Pull out", "Hook a luggage scale on the grip and pull straight up slowly. Note the peak force.",
         f"Axial grip per drill (model for v33 in loam: {AN['33']['soils']['moist loam']['drill_each']['Q_pull_N']} N) and how much "
         "the bigger flight buys."],
        ["4  Back out", "Drive the motor in reverse. Note whether it backs out or stalls. Repeat after the bit has stood in the soil "
         "for a while (the 1.3 set-up factor is for a drill that has stood).",
         "The release margin directly: whether the T16 (or T20) needs no planetary and no 50 rpm motor."],
        ["5  Send the numbers", "Times, currents, peak pulls, back-out yes or no.",
         "FLIGHT_D, D_PITCH, D_CORE0/1 and the taper parameters for v34 (KOV overrides in kedge_v33.py)."],
    ], [22, 75, 73], "Bit tests (kedge/proto/out/README.md).")

    # ============================================================== 6 self-righting simulation
    h1("6  Self-righting simulation study")
    story.append(P("The study asked whether the dart can land in any attitude, right itself with only its own legs and the "
                   "worm N20s, drill in and then hold the reel's tow, and which joint carries the tow. It is a design study on "
                   "simplified geometry, not the v33 CAD: the page is kedge/sim_righting/index.html (published as a private "
                   "artifact), the results are in kedge/sim_righting/results.json."))
    h2("6.1  Method")
    story.append(P("Physics: Rapier 3D 0.14.0 rigid bodies (WebAssembly), gravity 9.81 m/s<super>2</super>, 1 ms steps, 6 solver "
                   "iterations, stiff soil contacts (240 Hz). The hull, the two pods and the righting leg are separate bodies "
                   "joined by Rapier hinge joints. The worm gears and the hard stops are explicit stiff torque springs (gear 25 "
                   "N m/rad with 3 deg backlash, stops 60 N m/rad), so the page can report exactly how much each one carries. "
                   "A worm is self-locking (a load never turns it backwards); its speed falls linearly with load to zero at the "
                   "0.21 N m stall, and the firmware sees a stall at 90 % of it through a 20 ms current filter."))
    table([
        ["Item", "Model value"],
        ["Hull", "Cylinder 42 x 152 mm with a 20 mm nose cone from a 14 mm tip; centre of mass 85 mm from the nose tip, 8 mm above the axis (0 and -8 also run)"],
        ["Mass", "v33: 270 g in total (hull 170 g + pods 2 x 30 g + leg 40 g). Brief: 255 g body + 100 g of legs = 355 g. Inertia of a solid cylinder, scaled"],
        ["Pods", "30 g, 100 mm long, 16 mm thick; pivots 10 mm above the axis, 30 mm out; hard stop at 50 deg; each on its own worm N20 (50 deg in 8 s, 4 deg/s while drilling)"],
        ["Tail layout", "Pivots 140 mm from the nose tip, pods fold forward, drills enter just in front of the pivots under the tether; bridle to an eye on each pod hub 15 mm from the pivot, shared by a free ring"],
        ["Front layout", "Pivots 52 mm from the nose tip, pods fold aft; one tether eye on the hull at 150 mm, 14 mm below the axis"],
        ["Righting leg", "40 g, 128 mm hinge to pad, hinge on the top centreline 36 mm from the nose tip and 16 mm above the axis, folded flush; rounded 56 mm cross-bar pad; 0 to 178 deg, 180 deg in 4 s; rests folded after anchoring"],
        ["Soil", "Friction 0.6, restitution 0.02, ploughing drag only during the impact. Each drill tip on a 3 kN/m spring (damping 1.5 N s/m) that slides past the soil limit (7 N per drill in dry sand, 25 N in moist loam) and pulls out after 30 mm"],
        ["Rope and reel", "Reel 3 m behind the dart, 0.25 m high; tension follows the set point with an 80 ms lag; light drag 0.25 N"],
        ["Arrival", "6.8 m/s nose first. Presets: 40 deg path, roll 0 (belly), -40 (leaning), -90 and +90 (sides), 180 (upside down). Random: roll -180 to 180, path 25 to 55 deg, yaw +-5 deg, spin up to 2 rad/s"],
        ["Tow", "0 to 15 N in 3 s, hold 1.5 s, then +5 N each 2 s up to 55 N; 'moved' = 20 mm at the tether eye, then the reel stops"],
    ], [28, 142], "Model parameters (constants D and LAYOUTS in index.html, design block in results.json).", fs=7.4)
    story.append(P("Controller (it sees only what the firmware can: the IMU up vector in the body, yaw from the gyro, motor "
                   "positions and stall): after landing it waits until the dart is still for 0.5 s, then picks a move. Under 15 deg "
                   "of tilt it is upright. End-on: leg rock, both pods, leg flip. Upside down (over 110 deg): leg flip, leg flip, "
                   "pods then flip, leg rock. On a side or leaning: the low-side pod rolls it back (twice), then both pods, then a "
                   "leg flip. Every move starts from folded legs; after 6 moves it is stuck. If the tail is more than 15 deg off the "
                   "rope line (for example after an end-over-end flip), the reel pulls gently, up to 3 N, until it swings round. To "
                   "anchor, both pods plant, the low pod levels the roll to under 2.5 deg, then the drills run and the pods go to "
                   "the 50 deg stops at 4 deg/s; at the stop the worm backs off its stall preload. Then the tow."))
    h2("6.2  Results per landing preset")
    rows = [["Configuration", "Belly", "Leaning 40 deg", "Left side", "Right side", "Upside down"]]
    poses = ("belly", "lean", "left", "right", "upside")
    for key, lab in (("tail/v33/loam", "Tail, 270 g, loam"), ("front/v33/loam", "Front, 270 g, loam"), ("tail/v33/sand", "Tail, 270 g, sand"),
                     ("front/v33/sand", "Front, 270 g, sand"), ("tail/brief/loam", "Tail, 355 g, loam"), ("front/brief/loam", "Front, 355 g, loam")):
        cells = [lab]
        for p in poses:
            x = PRE[key][p]
            if not x["righted"]:
                cells.append("stuck")
            else:
                mv = f"; moved {x['movedAtN']} N" if x["movedAtN"] else ""
                cells.append(f"{x['rightT']} s, {x['moves']} moves; held {x['heldN']} N{mv}")
        rows.append(cells)
    table(rows, [30, 28, 28, 28, 28, 28], "Landing presets: time from landing to belly down, moves used, and the tow result "
          "(results.json presets). Worm and stop loads are in Appendix A.", fs=7.2)
    story.append(P(f"At 270 g every preset righted in both layouts, in 15.6 to 29.7 s with 2 to 4 moves. At 355 g only the belly "
                   f"preset righted: from a side or upside down the leg stalls at about 38 deg of pitch. In the tow, the tail layout "
                   f"held the full 55 N in loam from every preset with a pitch change of 5.6 to 6.5 deg; the front layout moved at "
                   f"49.8 to 52.8 N with about 12 deg of pitch change (no foot in this model, so the nose lifts). In sand both "
                   f"layouts held 15 N and moved at about 18 N (two drills at 7 N each plus friction)."))
    figure([F("sim_upside_flip.png"), F("sim_belly_podroll.png")],
           "(a) Upside down at 270 g: the righting leg swings out over the top and pushes the dart end over end (t 7.0 s). "
           "(b) Leaning on a pod: the low-side pod swings down and rolls the dart back onto its belly, then holds it (t 3.2 s).",
           crops=[(0, 40, 900, 730), (0, 40, 900, 730)], maxw=900)
    figure([F("sim_upside_drilling.png"), F("sim_stuck_355g.png")],
           "(a) Drilling in after righting: both pods swing down with the drills running to the 50 deg stops; the leg stays "
           "folded (t 27.0 s). (b) At 355 g: still upside down after 6 moves; the leg stalls at about 38 deg of pitch because "
           "lifting this dart over its nose needs more than the 0.21 N m the worm gives (t 70.9 s).",
           crops=[(0, 40, 900, 730), (0, 40, 900, 730)], maxw=900)
    h2("6.3  Batch of 50 random landings")
    t8 = BS["tail/v33/loam/cg+8"]
    rows = [["Configuration", "Righted", "Median (s)", "Slowest (s)", "Reel turned the tail", "Held 45 / 50 / 55 N", "Runs with worm load, peak (N m)"]]
    rows.append(["Tail, 270 g, loam", f"{tl['righted']}", f"{tl['medianS']}", f"{tl['slowestS']}", f"{tl['tailTurnedByReel']}",
                 f"{tl['heldAtN']['45']} / {tl['heldAtN']['50']} / {tl['heldAtN']['55']}", f"{tl['runsWithWormLoad']}, {tl['wormPeakNm']}"])
    rows.append(["Front, 270 g, loam", f"{fr['righted']}", f"{fr['medianS']}", f"{fr['slowestS']}", f"{BS['front/v33/loam/cg+8']['aligned']} (rerun)",
                 f"{fr['heldAtN']['45']} / {fr['heldAtN']['50']} / {fr['heldAtN']['55']}", f"{fr['runsWithWormLoad']}"])
    rows.append(["Tail, 270 g, sand", f"{ts['righted']}", f"{BS['tail/v33/sand/cg+8']['median']:.1f} (rerun)", f"{BS['tail/v33/sand/cg+8']['slowest']:.1f} (rerun)", f"{BS['tail/v33/sand/cg+8']['aligned']} (rerun)",
                 "0 / 0 / 0 (held 15 N in 50, 20 N in 0)", f"{ts['runsWithWormLoad']}, {ts['wormPeakNm']}"])
    rows.append(["Tail, 355 g, loam", f"{RB['tail/brief/loam']['righted']}", f"{BS['tail/brief/loam/cg+8']['median']:.1f} (rerun)", f"{BS['tail/brief/loam/cg+8']['slowest']:.1f} (rerun)", "", "the 9 righted held 55 N", f"{BS['tail/brief/loam/cg+8']['worm_runs']}, {BS['tail/brief/loam/cg+8']['worm_peak']:.2f} (rerun)"])
    table(rows, [30, 16, 20, 20, 24, 30, 30], "Batches of 50 random landings (results.json batch_50_random; '(rerun)' values from "
          "data/righting_batch_rerun.json). Every anchored tail run put load on a pod worm.", fs=7.2)
    story.append(P(f"The published batch keeps only the median and the slowest time, so the batch was run again headless with the "
                   f"page's own physics code and its batch seeds 1000 to 1049 (kedge/reports/righting_batch_rerun.mjs, Node 22, Rapier "
                   f"0.14.0). The rerun reproduces the published tail batch exactly (50 righted, median {t8['median']:.1f} s, slowest "
                   f"{t8['slowest']:.1f} s, 47 held 55 N, worm peak {t8['worm_peak']:.3f} N m) and the front batch to within one run "
                   f"(45 N: {BS['front/v33/loam/cg+8']['held'][45]} against 49, 50 N: {BS['front/v33/loam/cg+8']['held'][50]} against 29). "
                   f"Figure {fno('hist')} shows the righting times: they bunch at about 15.6 s (one pod roll or flip, then the anchor checks) and "
                   f"21.4 s (a second move). In the tail batch {mv_tail[2]} of 50 runs needed 2 moves, {mv_tail[1]} one, {mv_tail[3]} three and {mv_tail[4]} four."))
    figure(gp["hist"], "Righting time distribution, 50 random landings in moist loam at 270 g with the centre of mass 8 mm above "
           "the axis (rerun of the published batch, seeds 1000 to 1049; the published median for the front layout is 21.3 s).", jpeg=False, maxw=2000, label="hist", width_mm=150)
    h2("6.4  Mass: 270 g against 355 g")
    story.append(P(f"At 355 g the brief's dart righted in {RB['tail/brief/loam']['righted']} of 50 random landings. {RB['tail/brief/loam']['note'][0].upper() + RB['tail/brief/loam']['note'][1:]}. "
                   f"The leg lifts the dart's weight about its pad, so for the same geometry the torque it needs grows with the "
                   f"mass. Scaling the 355 g figure to 270 g as an estimate:"))
    eq(f"T(270 g) = 0.23 N m x 270 / 355 = {est270:.2f} N m, against the 0.21 N m stall: margin {WORM / est270:.2f}")
    story.append(P("That margin is thin: a worm that measures low on the bench (the model's range is 0.155 to 0.259 N m) or a few "
                   "grams more on the top would bring the 270 g dart to the same stall. Bench test B1 matters here."))
    h2("6.5  Centre of mass height")
    rows = [["Configuration", "Righted (of 50)", "Median (s)", "Slowest (s)", "Held 50 N", "Held 55 N"]]
    for k, lab in (("tail/v33/loam/cg+8", "Tail, 270 g, 8 mm above the axis"), ("tail/v33/loam/cg+0", "Tail, 270 g, on the axis"),
                   ("tail/v33/loam/cg-8", "Tail, 270 g, 8 mm below the axis"), ("tail/brief/loam/cg+8", "Tail, 355 g, 8 mm above"),
                   ("tail/brief/loam/cg+0", "Tail, 355 g, on the axis"), ("front/v33/loam/cg+8", "Front, 270 g, 8 mm above"),
                   ("front/v33/loam/cg+0", "Front, 270 g, on the axis")):
        b = BS[k]
        rows.append([lab, f"{b['righted']}" + (f" ({b['stuck']} stuck later)" if b["stuck"] and b["righted"] == b["n"] else ""), f"{b['median']:.1f}", f"{b['slowest']:.1f}", f"{b['held'][50]}", f"{b['held'][55]}"])
    table(rows, [58, 30, 20, 20, 20, 22], "Effect of the centre of mass height (reruns, moist loam, 50 random landings each).")
    figure(gp["cg"], "Centre of mass height against righting: righted runs and times (reruns). Lower is better in every case; "
           "at 355 g it is the difference between 9 and 38 righted.", jpeg=False, maxw=2000, width_mm=155)
    story.append(P(f"With the centre of mass 8 mm below the axis the dart mostly lands near belly down and rights in "
                   f"{BS['tail/v33/loam/cg-8']['median']:.1f} s median ({BS['tail/v33/loam/cg-8']['first']} needed no move at all). A low "
                   f"centre of mass also helps the front layout hold: on the axis it held 50 N in {BS['front/v33/loam/cg+0']['held'][50]} "
                   f"of 50 runs instead of {BS['front/v33/loam/cg+8']['held'][50]}."))
    h2("6.6  Holding per layout, and why the tail layout loads the worms")
    figure(gp["tension"], "Runs that held each tow step, by layout and soil (results.json batch heldAtN; the dashed line is the "
           "rerun with the centre of mass on the axis). The tail layout holds the 55 N cap in loam; sand drags at the 20 N step "
           "in either layout.", jpeg=False, maxw=2000, width_mm=150)
    story.append(P(f"In loam the tail layout held 55 N in {tl['heldAtN']['55']} of 50 runs; the front layout held 45 N in "
                   f"{fr['heldAtN']['45']}, 50 N in {fr['heldAtN']['50']} and 55 N in none. The tail layout's drills sit right under "
                   "the tether, so the pull runs nearly straight into them and the body barely pitches (about 6 deg against 12)."))
    story.append(P(f"The cost is the worm. In all {tl['runsWithWormLoad']} tail runs the tow reached a pod worm, peak "
                   f"{tl['wormPeakNm']} N m, above the 0.21 N m stall (the worm is self-locking, so in the model it holds; a real "
                   f"gearbox holds up to an unknown limit, assumed 0.6 N m). The mechanism (Figure {fno('layout')}): a pod that folds forward "
                   "points its tip toward the nose when folded, and as it deploys the tip swings down and aft. Under tow the soil "
                   "holds the tip while the dart is pulled aft, so the soil pushes the tip forward, the way it came: back toward "
                   "folded. The hard stop is on the other side (it stops the pod going past 50 deg), so only the worm resists. In "
                   "the front layout the pod folds aft, its tip swings forward as it deploys, and the same soil push turns it "
                   "further down into the hard stop: the stops carried 2.4 N m and the worms nothing in every run. v33 is a "
                   "front-layout dart, which is why its physics shows the stops, not the deploy worm, carrying the tow."))
    figure(gp["layout"], "Why the tail layout loads the worms. Grey: pod folded; black: deployed at 50 deg; dotted: the tip's path "
           "as it deploys. The soil's push on the tip under tow is always forward; it turns a forward-folding pod back toward "
           "folded (worm) and an aft-folding pod into its stop.", jpeg=False, maxw=2400, label="layout")
    story.append(P("Proposed fix: a deploy latch. At the 50 deg stop each pod must be held in both directions by the hull, not by "
                   "the worm. To keep to Harry's rule (no springs, no extra motors) the latch can be worked by the last few degrees "
                   "of the same deploy worm: lost motion in the coupler drives a bolt or cam into the pivot block, and reversing the "
                   "worm draws it back before the pod lifts. It must be designed and checked in the CAD audit and in the simulation "
                   "(the page shows the worm and stop torques live), and it would let the tail layout keep its 55 N holding with "
                   "zero worm load."))
    figure([F("sim_tow_tail_55N.png"), F("sim_tow_front_layout.png")],
           "Tow test screenshots. (a) Tail layout at 55 N from the upside-down preset: worm 0.112 N m on the right pod, stops "
           "0.27 N m, pins 19 N, drills 25 N, pitch change 5.7 deg. (b) Front layout: held 45 N and moved at 50 N; worm 0, "
           "stops 2.38 N m, pins 25 N, drills 24.3 and 25.0 N, pitch change 12 deg, nose lift 26.6 mm.",
           crops=[(0, 40, 900, 730), (0, 40, 900, 730)], maxw=900)

    # ============================================================== 7 launch, flight, impact
    h1("7  Launch, flight and impact")
    rows = [["", "v31", "v32c", "v33"]]
    L_ = {v: PH[v]["launch"] for v in V}
    Fl = {v: PH[v]["flight"] for v in V}
    rows += [
        ["Muzzle speed at 34 deg (m/s)"] + [f"{L_[v]['v_muzzle']}" for v in V],
        ["Stroke, time (m, ms)"] + [f"{L_[v]['stroke_m']}, {L_[v]['t_stroke_ms']}" for v in V],
        ["Peak acceleration, above 4 g for (g, ms)"] + [f"{L_[v]['accel_peak_g']}, {L_[v]['ms_above_4g']:.0f}" for v in V],
        ["Peak push on the tail face (N)"] + [f"{L_[v]['tail_push_peak_N']}" for v in V],
        ["Line pull in the stroke (N)"] + [f"{L_[v]['line_pull_in_stroke_N']}" for v in V],
        ["Aerodynamic centre x, CG x (mm)"] + [f"{Fl[v]['cp_top_x']}, {Fl[v]['cg_x']}" for v in V],
        ["Static margin, pitch (calibres)"] + [f"{Fl[v]['static_margin_calibers_top']}" for v in V],
        ["Impact at 34 deg: soft soil, gravel, hard (g)"] + [f"{PH[v]['landing']['impact_34deg']['soft']['g_peak']:.0f}, {PH[v]['landing']['impact_34deg']['gravel']['g_peak']:.0f}, {PH[v]['landing']['impact_34deg']['hard']['g_peak']:.0f}" for v in V],
        ["Nose dig in soft soil (mm)"] + [f"{PH[v]['landing']['impact_34deg']['soft']['depth_mm']}" for v in V],
        ["Coupler margin on landing: soil, gravel, hard"] + [" / ".join(x[3].split("margin ")[1] for x in PH[v]["loads"] if x[0].startswith("coupler")) for v in V],
    ]
    table(rows, [62, 36, 36, 36], "Launch, flight and impact per version (physics.json). Launch from base/physics/launcher.py "
          "with params 1.5.0: 12.06 J from two springs. Impact peaks are rigid-body upper bounds at the centre of mass.")
    story.append(P(f"The launcher gives the 270 g v33 {L_['33']['v_muzzle']} m/s at 34 deg with a {L_['33']['accel_peak_g']} g peak; "
                   "the push acts on the flat tail face and the cells sit on rigid plates, so launch loads are small (cells 4 N "
                   "each aft). In the air the dart is statically unstable on its own:"))
    eq(f"static margin = (x<sub>ac</sub> - x<sub>cg</sub>) / D = ({Fl['33']['cp_top_x']} - {Fl['33']['cg_x']}) mm / 42 mm = "
       f"{(Fl['33']['cp_top_x'] - Fl['33']['cg_x']) / 42:.2f} calibres")
    story.append(P("What keeps it nose first is the trailing tether: the 0.25 N reel brake gives 0.33 to 0.38 N at the tail. With "
                   "the tether it flies nose first at every rail pitch; without it it tumbles (at 34 deg the v33 angle of attack at "
                   f"impact would be {abs(FS['33']['pitch, 0 g ballast, rail 34 deg, no tether']['alpha_impact_deg'])} deg). Ballast of 0, "
                   "8 or 15 g changes nothing, so there is none. A cruciform on the boom would need about 35 mm of span for a "
                   f"positive margin ({CHK['fin_sweep'][-1]['margin_cal']['pitch']} calibre at 35 mm) and would hit the pusher's 14 mm "
                   "hole, so there are no fins. The reel must never let the line go slack in flight."))
    rows = [["Rail pitch (deg)", "Muzzle (m/s)", "Range (m)", "Flight (s)", "Apex (m)", "Impact (m/s)", "Path angle (deg)", "AoA at impact, peak (deg)"]]
    for p in (10, 20, 34, 45):
        x = FS["33"][f"pitch, 0 g ballast, rail {p} deg, tether 0.25 N"]
        rows.append([p, x["v_muzzle"], x["range_m"], x["t_flight_s"], x["apex_m"], x["v_impact"], x["path_angle_deg"], f"{x['alpha_impact_deg']}, {x['alpha_peak_after_0p15s_deg']}"])
    table(rows, [22, 20, 18, 18, 18, 20, 24, 30], "v33 flights with the tether (0.25 N brake), from out33/flight_sim.json. The angle of "
          "attack is the response to a 5 deg plus 1 rad/s muzzle disturbance; it decays slowly.")
    story.append(P(f"On impact the nose digs 2 to 12 mm in soil, the body slaps down and skids roughly 0.2 m (soil) to 1 m (hard "
                   f"ground). The BNO085 saturates at 8 g on every surface (raw peak at the board {PH['33']['landing']['impact_34deg']['soft']['bno085_raw_g']:.0f} g "
                   f"on soft soil). On gravel the PPA-CF couplers are at a margin of about 1 and on hard ground 0.5: do not fire at "
                   "concrete, rock or packed gravel. At rest on hard ground the folded v33 leans about 41 deg on a pod "
                   f"({lean_share:.0f} % of random tumbles) or lies on a side near 113 to 128 deg ({side_share:.0f} %); it never rests "
                   "belly down, because the centre of mass is above the axis."))

    # ============================================================== 8 firmware and jetson
    h1("8  Firmware and Jetson changes this round")
    h2("8.1  Dart firmware v4.1 and v4.2 (firmware/dart)")
    story.extend(bullets([
        "<b>Flight windows</b> for the real launcher (shortest flight 0.37 s): push-over 200 ms, first impact counted from 250 ms after launch (was 400).",
        "<b>Milling deploy</b>: a deploy stall before 85 % of the estimated pod travel is soil, not the stop; the deploy pauses 0.2 s with the drills cutting and pushes again; 5 s with no progress ends the step; 150 s limit, then a 60 s rest before the next /auto ('motors hot').",
        "<b>67 % stall guard</b> (v4.2): when the drill pair, read alone, passes 0.67 of its bench stall current (drillPairStallMa, 700 mA at 8.0 V), pods and drills stop and the anchor ends ANCHORED with the warning depth_limited. It replaced the 85 % relax pause and is the release safety in firmer loam.",
        "<b>Levelling and the orientation gate</b>: from the BNO085 gravity at rest, upright under 10 deg; leaning 10 to 60 deg swings the pods to 16 deg with the drills off to level the hull first; on a side (60 to 120 deg), upside down (over 120) or nose pitch outside -25 to +15 deg, /auto is refused 'not upright' (force=1 overrides). Tilt-only until imuNoseAxis is set.",
        "<b>Battery limits</b> at rest: low under 6.8 V, critical under 6.4 V refuses /auto and switches the video off; /release always runs.",
        "<b>Kick pulsing</b> (0.4 s on, 0.2 s off) while the foot finishes; release rocks the drills up to 5 times and mills the pods up; soil_mA and moving in /status; optional video switch on D10.",
        "<b>Direct drive</b>: DRILL_SIGN per motor; v32 and v33 drive the auger straight from the motor, so DRILL_SIGN = +1 (it was briefly -1 for the planned planetary).",
        "Verified on the PC: 313 logic checks, 15 simulator tests, fuzz 6 seeds x 8000 episodes; compiles on esp32 core 3.3.12 and 2.0.17. Nothing has run on real hardware yet.",
        "Still to do for v33: the deploy end stop moved from 45 to 50 deg; if podsTravelMs is the full travel, scale it to about 1560 ms (dart-cad3 request).",
    ]))
    h2("8.2  Jetson (jetson/)")
    story.extend(bullets([
        "Follows API v4.1: /status spaced 0.2 s, waits for log_frozen before /flightlog, rides WiFi gaps up to 10 s, asks the operator on 'not upright' (reshoot or force), stops the tow at once on 'moving', battery in preflight; anchor timeout 200 s, release 180 s.",
        "Realistic synthetic flights: the real launcher (about 6.9 m/s, 2.6 to 4.8 m, 2 to 3 % speed scatter), dart-physics' flight and landing (nose swings 4 to 16 deg, skid 0.1 to 1 m, about 42 deg lean on hard ground), the Caddx fisheye with an unknown 125 to 150 deg field of view, analog noise, 60 ms capture delay.",
    ]))
    table([
        ["Check", "48 flights at 360 x 240", "24 flights at 720 x 480"],
        ["Range with the dart's flight log", "median 1.2 %, 90 % within 3.2 %, worst 4.2 %", "median 1.1 %, 90 % within 2.2 %, worst 5.5 %"],
        ["Range from video only (pitch known)", "median 4.4 %, 90 % within 18 %, worst 71 % (missed landing)", "median 4.3 %, 90 % within 19 %"],
        ["Landing marker against the true impact", "median 0.07 m, 90 % within 0.15 m", "median 0.04 m, 90 % within 0.14 m"],
        ["Landing verdict as expected", "90 %", "88 %"],
        ["GO given on sand, rock or water (unsafe)", "0 of 24", "0 of 12"],
    ], [56, 57, 57], "Jetson dart vision on realistic synthetic flights (jetson/README.md section 13). Real footage will be worse; "
       "section 12 re-learns the numbers from real shots.")

    # ============================================================== 9 open items
    h1("9  Open items and what Harry must measure")
    table([
        ["Measure", "How", "What it sets"],
        ["Worm N20 stall torque (B1)", "Motor in a vice, 50 mm lever, luggage scale, 2S at full duty, 3 tries (DART_PULLTEST.md)", f"Model {WORM:.3f} N m (0.155 to 0.259): milling time, and the righting margin ({est270:.2f} N m needed at 270 g)"],
        ["Worm holding strength (B2)", "Pull the lever backwards with the motor off until it slips (stop at 1 N m)", "Assumed 0.6 N m: holding wherever a worm is still in the load path (kick in loam, a tail layout without a latch)"],
        ["Drill stall and pair stall current (B3, BENCH_TEST step 4)", "As B1; both drills at 8.0 V", f"Model {STALL:.3f} N m: release margin; drillPairStallMa (700 mA) for the 67 % guard"],
        ["Foot motor stall (B4)", "As B1", "Its currents imply 0.738 N m, 3.5 times the other worms: check it really is the 27 rpm worm"],
        ["Stall and free currents of all four channels, kick swing time", "BENCH_TEST steps 3 to 6 and 5", "Tuning in kedge_logic.h; kickSwingMs (4300 ms) for KICK_JAM and NO_GROUND"],
        ["Bit tests T12, T16, T20", "Section 5.3", "Final drill diameter, pitch and core; whether any gear stage is needed"],
        ["Soil pull test", "DART_PULLTEST.md part B: pull in 5 N steps, 10 s holds, peg at the tail, log force and movement", f"The loam cohesion (Table {tno('coh')} row), the tow limit (5 mm force / 1.5) and the Jetson tow_limit (hand-pull holding minus 10 N, 40 N for the first field tows)"],
        ["IMU axes", "BENCH_TEST step 7: 'b' with the nose tipped down must read pitch negative", "imuNoseAxis (+2 or +1) in the firmware and imu_axes on the Jetson; until then the orientation gate is tilt-only"],
        ["Dart camera field of view", "Checkerboard, 60 pictures, then calibrate dart (jetson/README.md 7b)", "dart_cam_calib (the default guess is 135 deg horizontal)"],
        ["Capture latency", "3 or more low-rail shots, then learn latency", "dart_video_latency_s (default 0.06 s)"],
        ["Bought part masses", "Kitchen scale", "BOUGHT_G in dart_model.py: mass and centre of mass"],
    ], [38, 64, 68], "What to measure, how, and what it decides.", fs=7.3)
    h2("9.1  Open design items")
    story.extend(bullets([
        "Deploy latch for the tail layout, worked by the deploy worm (no spring, no extra motor); a hold at any angle for depth-limited anchors.",
        "Catch at the kick plant so the foot's tow load goes into the nose, not the kick worm.",
        "Two journals and an axial retainer for each drill auger and for the foot auger, so the motor shafts only drive.",
        "Lower the centre of mass to the axis or below (kick and foot motors lower).",
        "Self-righting with the kick leg, and firmware moves in place of the 'not upright' refusal; then rerun the simulation on the real v34 geometry.",
        "Simulation limits: no foot in the model, drill soil limits simplified (7 and 25 N per drill against the pipeline's 2.5 and 17.2 N for v33), a leg and pods of simple shapes. The physics pipeline does not yet model the drill taper.",
        "Firmware: podsTravelMs for the 50 deg deploy; base: tether line model to end at the boom eye, and 22 mm more room behind the carriage if a 50 mm boom is wanted.",
    ]))

    # ============================================================== appendix
    h1("Appendix A  Simulation data")
    rows = [["Configuration, pose", "Righted (s)", "Moves", "Held (N)", "Moved at (N)", "Worm peak (N m)", "Stop peak (N m)", "Pitch change (deg)", "Tail turned"]]
    for key in PRE:
        for p in poses:
            x = PRE[key][p]
            rows.append([f"{key}, {p}", x["rightT"] if x["righted"] else "stuck", x["moves"] if x["moves"] is not None else "-", x["heldN"],
                         x["movedAtN"] if x["movedAtN"] is not None else "-", x["wormPeakNm"], x["stopPeakNm"], x["pitchChangeDeg"], "yes" if x["tailTurned"] else "no"])
    table(rows, [40, 17, 13, 15, 18, 18, 18, 18, 13], "All landing presets (results.json). Key: layout/mass/soil; v33 = 270 g, brief = 355 g.", fs=6.8, keep=False)
    rows = [["Rerun", "Righted", "Median (s)", "Slowest (s)", "Held 15 / 30 / 45 / 50 / 55 N", "Worm runs, peak (N m)", "Reel turned", "Stuck"]]
    for k in sorted(BS):
        b = BS[k]
        rows.append([k, b["righted"], f"{b['median']:.1f}", f"{b['slowest']:.1f}", " / ".join(str(b["held"][L]) for L in (15, 30, 45, 50, 55)),
                     f"{b['worm_runs']}, {b['worm_peak']:.3f}", b["aligned"], b["stuck"]])
    table(rows, [36, 15, 17, 17, 36, 22, 15, 12], "Headless reruns, 50 random landings each, seeds 1000 to 1049 "
          "(data/righting_batch_rerun.json). Key: layout/mass/soil/centre of mass height in mm.", fs=6.8)

    h1("Appendix B  Physics data")
    rows = [["Version, soil", "Each drill (N)", "Foot (N)", "Friction (N)", "Total (N)", "If worms never slip (N)", "Drill spin (N m)", "Unscrew (N m)", "Margin"]]
    for v in V:
        for s in SOILS:
            b = PH[v]["anchoring"]["by_soil"][s]
            u = unscrew(v, s)
            rows.append([f"{VN[v]}, {s}", b["drill_auger_each_N"], b["foot_auger_N"], b["friction_N"], b["total_N"], b["total_if_worms_never_slip_N"],
                         spin(v, s), u["drill_unscrew_torque_Nm"], u["drill_margin"]])
    table(rows, [30, 17, 15, 17, 16, 22, 18, 18, 17], "Anchoring by soil (physics.json anchoring, anchor.json soils), rover 3 m.", fs=6.8)
    mo = AN["33"]["motors_7V4"]
    table([
        ["Motor at 7.4 V", "Stall (N m)", "Range (N m)", "Stall current (A)", "Free speed (rpm)", "Efficiency"],
        *[[k, mo[k]["stall_Nm"], f"{mo[k]['stall_Nm_range'][0]} to {mo[k]['stall_Nm_range'][1]}", mo[k]["stall_A"], mo[k]["free_rpm"], mo[k]["eta"]] for k in ("worm", "drill", "foot")],
    ], [40, 24, 30, 26, 26, 24], "Motor model (anchor.json motors_7V4, from the bench currents).", fs=7.2)
    rows = [["Version", "Rail 10 deg", "Rail 20 deg", "Rail 34 deg", "Rail 45 deg"]]
    for v in V:
        cells = [VN[v]]
        for p in (10, 20, 34, 45):
            x = FS[v][f"pitch, 0 g ballast, rail {p} deg, tether 0.25 N"]
            cells.append(f"{x['range_m']} m, {x['t_flight_s']} s, {x['v_impact']} m/s at {x['path_angle_deg']} deg")
        rows.append(cells)
    table(rows, [18, 38, 38, 38, 38], "Flights with the tether per version: range, flight time, impact speed and path angle (flight_sim.json).", fs=7)
    rows = [["Version", "Rest states on hard ground: share, roll, pitch"]]
    for v in V:
        rows.append([VN[v], "; ".join(f"{r_['share_pct']} % at roll {r_['roll_deg']}, pitch {r_['pitch_deg']}" for r_ in PH[v]["landing"]["rest_states"])])
    table(rows, [18, 152], "Rest states of the folded dart after a random tumble (physics.json landing).", fs=7)

    h1("Appendix C  File locations and commits")
    table([
        ["What", "Where (repository GrappStar, branch dart-v1)"],
        ["This report, its generator, graphs and copied figures", "kedge/reports/KEDGE_dart_findings_report.pdf, make_findings_report.py, fig/"],
        ["Righting batch rerun", "kedge/reports/righting_batch_rerun.mjs, kedge/reports/data/righting_batch_rerun.json"],
        ["Mass rows (dart_model)", "kedge/reports/data/mass_rows.json"],
        ["Geometry notes, rules learned", "kedge/CLAUDE.md sections 5 and 7"],
        ["Physics findings", "kedge/DART_PHYSICS31.md, kedge/DART_PHYSICS32.md (v32b), shared decisions in kedge/DART_V31_NOTES.md"],
        ["Physics data", "kedge/out31, out32, out33: physics.json, mass_props.json, flight_sim.json, phys31/anchor.json, landing.json; out32/phys31/v32_release.json, v32_checks.json"],
        ["Audit and launcher fit", "kedge/out3x/audit.json, launcher_fit.json, audit_sheet.png, sections.png"],
        ["Review renders", "kedge/out31/audit_sheet.png; out32/review/ (before_v32b, after, v32c_final.png); out33/review/ (15 to 27, v33_sheet.png)"],
        ["Generators", "kedge/kedge_v27.py (v31), kedge_v32.py (v32c), kedge_v33.py (v33); pipeline run_all.sh"],
        ["Print packs and zips", "kedge/pack_v31..pack_v33, dist_v31..dist_v33"],
        ["Prototype bits", "kedge/proto/drill_proto.py, kedge/proto/out (STLs, README.md, proto_bits.png); kedge/KEDGE_drill_test_pack.zip"],
        ["Preliminary simulation spec (v32c)", "kedge/proto/DART_v32c_SIM_SPEC.md"],
        ["Self-righting study", "kedge/sim_righting/index.html, results.json; published privately at claude.ai/artifact/TGTtc2um8jqGDkG5ic6hQA"],
        ["Firmware", "firmware/dart (REPORT.md, API.md, BENCH_TEST.md, WIRING.md); kedge/fw is a synced copy"],
        ["Jetson", "jetson/README.md (sections 6, 7b, 12, 13), jetson/REPORT_VISION.md"],
        ["Pull test", "kedge/DART_PULLTEST.md, kedge/dart_pulltest.py"],
    ], [52, 118], "File locations.", fs=7.2)
    table([
        ["Commit", "Date (UTC)", "What"],
        ["e452f0a", "7 Nov 02:34", "Dart v31 outputs: out31 (audit CLEAN, launcher fit FITS, wire room OK, all STEP valid), pack_v31, dist_v31"],
        ["af7ba34", "7 Nov 02:16", "dart-physics: final v31 rerun (d13, core 6, pods y 29, 0 g ballast, horns and kick rest)"],
        ["4e12448, fb10cb8", "7 Nov 02:16, 02:37", "Dart firmware v4.1: milling deploy, orientation gate, levelling, kick pulsing, heat rest"],
        ["d26d43e", "7 Nov 03:37", "dart-physics: v32 sleeve auger sizing; Matlock near-surface cohesion"],
        ["bd855fe", "7 Nov 04:01", "dart-physics: v31 outputs refreshed (spin torque without tip term, 15 N first tow)"],
        ["78366d0, 3d16ce9", "7 Nov 04:26, 04:27", "dart-physics: v32 release by milling out; v32 rerun on the clean out32"],
        ["bf80310, 47e7fc3", "7 Nov 04:37, 05:03", "Dart firmware v4.2: 67 % deploy guard; direct drive, DRILL_SIGN +1"],
        ["980d230, 9d032bb", "7 Nov 04:54, 05:16", "Dart v32 and v32b outputs: out32, pack_v32, dist_v32, pub32"],
        ["640dc17", "7 Nov 06:47", "Dart v32c outputs (audit CLEAN at 1 deg, launcher fit FITS); CLAUDE.md section 5"],
        ["a57a793, 378631b", "7 Nov 07:16, 07:25", "Prototype tapered drill bits and grip, the v32c simulation spec; drill test print pack"],
        ["7e353bf, c9712bd", "7 Nov 08:40, 08:46", "Dart v33: tapered drill augers on the M3 spine, gimlet foot, close-ups"],
        ["a63342e, d3d1c03, f37dd51", "7 Nov 09:08 to 09:11", "Dart v33 outputs (audit CLEAN, launcher fit FITS), pack_v33, dist_v33, review sheet"],
        ["7c55243", "8 Nov 23:08", "KEDGE righting sim: Rapier rigid-body study of landing, self-righting, drilling in and the reel tow"],
    ], [34, 30, 106], "Commits of this round (git log on dart-v1). This report is committed after them.", fs=7.2)

    # ------------------------------------------------------------------------------------------ em dash check and build
    bad = [t for t in _TEXT if "—" in t or "–" in t]
    if bad:
        raise SystemExit("em or en dash in the text: " + bad[0][:120])
    doc = Doc(OUT_PDF)
    doc.multiBuild(story)
    print("wrote", OUT_PDF, "figures", _fig_n[0], "tables", _tab_n[0])


if __name__ == "__main__":
    build()
