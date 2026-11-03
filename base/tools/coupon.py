"""Fit coupon: small PLA test tiles Harry prints FIRST to tune cad/lib/fits.py before the real parts.

python -m tools.coupon OUT

Writes
  OUT/coupon/<part>.stl          one per coupon part, in print orientation, min corner at (0, 0, 0)
  OUT/coupon/coupon_plate_<n>.3mf  the coupon parts packed on the 256 bed (tools.pack helpers)
  OUT/coupon/coupon_plate_<n>.png  top-down preview of that plate
  OUT/coupon/COUPON.md           what to check on each tile and how to update fits.py

Every tile is flat on the bed, under about 60 x 40 x 10 mm, one shell, with raised labels on the top face (a small
stroke font: digits, '.', '-', '+' and a few letters). The first character of each label is the tile letter (A to
H) used in COUPON.md. All geometry comes from cad/lib (bearings, shafts, fasteners, clips, line, gears) with the fit
value swapped per variant; the variants are read from fits.py at run time, so the labels always match fits.py.

    from tools.coupon import coupon_parts
    parts = coupon_parts()       # list of cad.lib.Part (material PLA, print_tf set)
"""
from __future__ import annotations

import argparse
import contextlib
import math
import os
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

from manifold3d import CrossSection  # noqa: E402

from cad.lib import Part, prim, tf, fits  # noqa: E402
from cad.lib import bearings, shafts, fasteners, clips, line, gears  # noqa: E402

LABEL_H = 4.0          # character height (mm)
LABEL_STROKE = 0.8     # stroke width (two 0.4 nozzle lines)
LABEL_RAISE = 0.6      # raised height above the top face (3 layers)
LABEL_SINK = 0.2       # how far the label solid reaches into the tile (clean union)
COLOR = "#e8e2c8"

# ---------------------------------------------------------------------------------------------- stroke font
# Glyphs on a 2 x 4 grid (x 0..2, y 0..4), each a list of line segments. Seven segment digits plus a few letters.
_SEG = dict(a=((0, 4), (2, 4)), b=((2, 4), (2, 2)), c=((2, 2), (2, 0)), d=((0, 0), (2, 0)), e=((0, 0), (0, 2)),
            f=((0, 2), (0, 4)), g=((0, 2), (2, 2)))
_7SEG = {"0": "abcdef", "1": "bc", "2": "abged", "3": "abgcd", "4": "fgbc", "5": "afgcd", "6": "afgedc", "7": "abc",
         "8": "abcdefg", "9": "abcdfg", "-": "g", "C": "afed", "E": "afedg", "F": "afeg",
         "H": "febcg"}
FONT = {k: [_SEG[s] for s in v] for k, v in _7SEG.items()}
FONT["+"] = [_SEG["g"], ((1, 1), (1, 3))]
FONT["A"] = [((0, 0), (1, 4)), ((1, 4), (2, 0)), ((0.5, 2), (1.5, 2))]
FONT["."] = [((0.0, 0.0), (0.0, 0.01))]
FONT["B"] = [((0, 0), (0, 4)), ((0, 4), (1.4, 4)), ((1.4, 4), (2, 3.4)), ((2, 3.4), (2, 2.6)), ((2, 2.6), (1.4, 2)),
             ((0, 2), (1.4, 2)), ((1.4, 2), (2, 1.4)), ((2, 1.4), (2, 0.6)), ((2, 0.6), (1.4, 0)), ((1.4, 0), (0, 0))]
FONT["D"] = [((0, 0), (0, 4)), ((0, 4), (1.2, 4)), ((1.2, 4), (2, 3.2)), ((2, 3.2), (2, 0.8)), ((2, 0.8), (1.2, 0)),
             ((1.2, 0), (0, 0))]
FONT["G"] = [_SEG["a"], _SEG["f"], _SEG["e"], _SEG["d"], _SEG["c"], ((1, 2), (2, 2))]
FONT["M"] = [((0, 0), (0, 4)), ((0, 4), (1, 2)), ((1, 2), (2, 4)), ((2, 4), (2, 0))]
FONT[" "] = []
_ADV = {".": 0.0, " ": 1.0}     # glyph width in grid units (default 2)


def _text_width(text, h=LABEL_H):
    u = h / 4.0
    gap = 0.6 * u + LABEL_STROKE
    return sum(_ADV.get(ch, 2.0) * u for ch in text) + gap * (len(text) - 1)


def text_cs(text, h=LABEL_H, stroke=LABEL_STROKE):
    """CrossSection of a text string in the stroke font, centred on the origin (x and y)."""
    u = h / 4.0
    gap = 0.6 * u + stroke
    caps, x = [], 0.0
    for ch in text:
        if ch not in FONT:
            raise ValueError(f"stroke font has no glyph for {ch!r}")
        for (x0, y0), (x1, y1) in FONT[ch]:
            c0 = CrossSection.circle(stroke / 2, 12).translate((x + x0 * u, y0 * u))
            c1 = CrossSection.circle(stroke / 2, 12).translate((x + x1 * u, y1 * u))
            caps.append(CrossSection.batch_hull([c0, c1]))
        x += _ADV.get(ch, 2.0) * u + gap
    if not caps:
        return CrossSection()
    cs = caps[0]
    for c in caps[1:]:
        cs = cs + c
    w = _text_width(text, h)
    return cs.translate((-w / 2, -h / 2))


def label(text, x, y, z_top, h=LABEL_H):
    """Raised label solid standing on a top face at z_top, centred on (x, y)."""
    return prim.extrude(text_cs(text, h), LABEL_SINK + LABEL_RAISE, z=z_top - LABEL_SINK).translate((x, y, 0))


# ---------------------------------------------------------------------------------------------- helpers
@contextlib.contextmanager
def patched(mod, **vals):
    """Temporarily set module constants (fit values a library function reads from its module)."""
    old = {k: getattr(mod, k) for k in vals}
    try:
        for k, v in vals.items():
            setattr(mod, k, v)
        yield
    finally:
        for k, v in old.items():
            setattr(mod, k, v)


def tile(sx, sy, t, cuts=(), labels=(), r=2.0):
    """Rounded tile sx x sy, z 0..t, centred on the origin. cuts: (cutter, x, y) with the cutter mouth at z = 0 cutting
    into -Z (cad/lib convention), placed at the top face. labels: (text, x, y) raised on the top face."""
    body = prim.rbox(sx, sy, t, r)
    if cuts:
        body = body - prim.union([prim.place(c, tf.T(x, y, t)) for c, x, y in cuts])
    if labels:
        body = body + prim.union([label(s, x, y, t) for s, x, y in labels])
    return body


def fmt(v, nd=2):
    """Short label number with nd decimals: 0.15 -> .15, -0.05 -> -.05, 3.8 -> 3.8, 4 -> 4.0 (nd 1)."""
    s = f"{v:.{nd}f}"
    if s.startswith("0."):
        s = s[1:]
    elif s.startswith("-0."):
        s = "-" + s[2:]
    return s


def _part(name, solid, notes, qty=1, print_tf=None):
    return Part(name, solid, "PLA", qty=qty, print_tf=tf.I4 if print_tf is None else print_tf, color=COLOR,
                subsystem="coupon", notes=notes)


# ---------------------------------------------------------------------------------------------- variants
STEP_RIB = 0.05
STEP_BORE = 0.10


def variants():
    """The fit values each coupon tests (read from fits.py now)."""
    return dict(
        rib=[round(fits.BRG_RIB_INTERF + d, 3) for d in (-STEP_RIB, 0.0, STEP_RIB)],
        bore=[round(fits.SHAFT_IN_BORE + d, 3) for d in (-STEP_BORE, 0.0, STEP_BORE)],
        insert=[3.8, 4.0, 4.2],
        nut=[5.6, 5.8, 6.0],
        holes=[2.5, 2.6, 2.7, 3.2, 3.4],
        pin=[fits.HOLE_SLIDE, fits.HOLE_CLEAR],
        eye=[3.0, 5.0],
    )


# ---------------------------------------------------------------------------------------------- the coupons
BRG_T = 9.0            # 608 tile: 7 seat + 2 lip
KEY_T = 8.0            # keyed bore tile thickness (engaged length)
KEY_STUB_L = 16.0
PIN_D = 6.0
PIN_GRIP = 4.0         # host plate thickness for the d 6 pins
GEAR_M, GEAR_Z1, GEAR_Z2, GEAR_FACE = 1.5, 12, 24, 5.0
GEAR_PIN_D = 5.0
GEAR_PLATE_T = 3.0


def coupon_a():
    """A: 608 crush-rib seat at three rib interferences."""
    out = []
    for i, v in enumerate(variants()["rib"], 1):
        seat = bearings.ribbed_seat(bearings.B608_OD, bearings.B608_W, lip_d=bearings.B608_LIP_D, interf=v)
        s = tile(34, 40, BRG_T, [(seat, 0, 4)], [("A " + fmt(v), 0, -15.5)])
        out.append(_part(f"coupon_a{i}_608_rib", s, f"608 crush-rib seat, rib interference {v:.2f} mm radial"))
    return out


def coupon_b():
    """B: 20 mm keyed bore at three clearances plus a printed keyed shaft stub."""
    out = []
    for i, v in enumerate(variants()["bore"], 1):
        bore = shafts.keyed_bore(KEY_T, d=20.0, clear=v)
        s = tile(36, 40, KEY_T, [(bore, -1.5, 3)], [("B " + fmt(v), 0, -15.5)])
        out.append(_part(f"coupon_b{i}_key_bore", s, f"20 mm keyed bore, clearance {v:.2f} mm diametral"))
    stub = shafts.keyed_shaft(KEY_STUB_L, d=20.0)
    out.append(_part("coupon_b_key_shaft", stub, f"Keyed shaft stub d 20, {KEY_STUB_L:g} long, prints upright"))
    return out


def coupon_c():
    """C: M3 heat-set insert holes at 3.8 / 4.0 / 4.2."""
    vs = variants()["insert"]
    cuts, labs = [], [("C", -21, 0)]
    for k, d in enumerate(vs):
        x = -8 + 12 * k
        with patched(fasteners, M3_INSERT_D=d):
            cuts.append((fasteners.m3_insert(), x, 3.5))
        labs.append((fmt(d, 1), x, -6))
    s = tile(50, 22, 8.0, cuts, labs)
    return [_part("coupon_c_m3_insert", s, "M3 heat-set insert holes " + " / ".join(f"{d:.1f}" for d in vs))]


def coupon_d():
    """D: M3 nut traps at AF 5.6 / 5.8 / 6.0."""
    vs = variants()["nut"]
    cuts, labs = [], [("D", -21, 0)]
    for k, af in enumerate(vs):
        x = -8 + 13 * k
        cuts.append((fasteners.nut_trap(af, fits.M3_NUT_H, bolt_len=8.0), x, 3.5))
        labs.append((fmt(af, 1), x, -6.5))
    s = tile(52, 22, 6.0, cuts, labs)
    return [_part("coupon_d_m3_nut", s, "M3 nut traps AF " + " / ".join(f"{a:.1f}" for a in vs))]


def coupon_e():
    """E: M3 tap and clearance holes 2.5 / 2.6 / 2.7 / 3.2 / 3.4."""
    vs = variants()["holes"]
    cuts, labs = [], [("E", -25.5, 0)]
    for k, d in enumerate(vs):
        x = -17 + 10.5 * k
        cuts.append((fasteners.round_hole(d, 7.0), x, 3.5))
        labs.append((fmt(d, 1), x, -5.5))
    s = tile(58, 20, 6.0, cuts, labs)
    return [_part("coupon_e_m3_holes", s, "M3 tap and clearance holes " + " / ".join(f"{d:.1f}" for d in vs))]


def _notched(pin, head_d, head_h, n):
    """Cut n small notches into the rim of a pin head (n = which fit). The head sits in z = [-head_h, 0]."""
    cuts = []
    for k in range(n):
        a = 90.0 + (k - (n - 1) / 2) * 30.0
        nb = prim.box(head_d / 2 - 0.9, head_d / 2 + 1, -0.5, 0.5, -head_h - 1, 0.5)
        cuts.append(prim.place(nb, tf.Rz(a)))
    return pin - prim.union(cuts)


def coupon_f():
    """F: d 6 snap pin and clip pin with a C clip, each at two fits (HOLE_SLIDE, HOLE_CLEAR), and a host plate."""
    vs = variants()["pin"]
    hd = clips.HEAD_D_RATIO * PIN_D
    hh = max(clips.HEAD_H_MIN, clips.HEAD_H_RATIO * PIN_D)
    hole = clips.pin_hole_cutter(PIN_D, PIN_GRIP).translate((0, 0, -PIN_GRIP))   # mouth at z = 0, into -Z
    cuts, labs = [], [("F", -15, 13)]
    for y in (6.5, -8.5):                       # snap row, clip row
        for k, x in enumerate((-5.0, 8.0)):
            cuts.append((hole, x, y))
            labs.append(("1" if k == 0 else "2", x - 6.5, y))
    host = tile(36, 36, PIN_GRIP, cuts, labs)
    out = [_part("coupon_f_pin_host", host, "Host plate for the d 6 pins: snap row on top, clip row below")]
    up = tf.T(0, 0, hh)
    for k, f in enumerate(vs, 1):
        tag = f"fit {f:.2f} ({k} notch{'es' if k > 1 else ''})"
        sp = _notched(clips.snap_pin(PIN_D, PIN_GRIP, fit=f), hd, hh, k)
        out.append(_part(f"coupon_f_snap_pin_{k}", sp, f"Snap pin d 6, {tag}", print_tf=up))
        cp = _notched(clips.clip_pin(PIN_D, PIN_GRIP, fit=f), hd, hh, k)
        out.append(_part(f"coupon_f_clip_pin_{k}", cp, f"Clip pin d 6, {tag}", print_tf=up))
    out.append(_part("coupon_f_c_clip", clips.c_clip(PIN_D), "C clip for the d 6 clip pin", qty=2))
    return out


def coupon_g():
    """G: Dyneema eye in a 3 mm and a 5 mm plate (one stepped tile)."""
    t3, t5 = variants()["eye"]
    body = prim.rbox(44, 22, t3, 2.0) + prim.box(0, 22 - 2.0, -11, 11, 0, t5)
    body = body + prim.place(prim.rbox(4.0, 22, t5, 2.0), tf.T(20, 0, 0))       # round the thick end
    body = body - prim.place(line.eye_cutter(t3), tf.T(-9, 3, 0)) - prim.place(line.eye_cutter(t5), tf.T(11, 3, 0))
    body = body + label("G " + fmt(t3, 0), -9, -6, t3) + label(fmt(t5, 0), 11, -6, t5)
    return [_part("coupon_g_eye", body, f"Dyneema eye (v7) in {t3:g} mm and {t5:g} mm plates")]


def gear_cd():
    return gears.center_distance(GEAR_M, GEAR_Z1, GEAR_Z2)


def coupon_h():
    """H: m1.5 gear pair z12 / z24 on printed pins at gears.center_distance, default backlash."""
    cd = gear_cd()
    rp = GEAR_PIN_D / 2
    ph = GEAR_FACE + 1.0
    pin = prim.revolve_pts([(0, 0), (rp, 0), (rp, ph - 0.5), (rp - 0.5, ph), (0, ph)])
    plate = prim.box(-13, cd + 10, -18, 12, 0, GEAR_PLATE_T)
    plate = plate + prim.place(pin, tf.T(0, 0, GEAR_PLATE_T)) + prim.place(pin, tf.T(cd, 0, GEAR_PLATE_T))
    plate = plate + label("H 1.5", -6, -14.5, GEAR_PLATE_T)
    bore = GEAR_PIN_D + fits.SHAFT_IN_BORE
    out = [_part("coupon_h_gear_plate", plate, f"Gear plate: pins d {GEAR_PIN_D:g} at centre distance {cd:.2f}")]
    for z in (GEAR_Z1, GEAR_Z2):
        g = gears.spur_gear(GEAR_M, z, GEAR_FACE, bore=bore)
        out.append(_part(f"coupon_h_gear_z{z}", g, f"Spur gear m{GEAR_M:g} z{z}, backlash {fits.GEAR_BACKLASH:g}"))
    return out


def coupon_parts(P=None) -> list:
    """Every coupon part (cad.lib.Part, PLA, print_tf set). P is unused (kept for the module style)."""
    out = []
    for fn in (coupon_a, coupon_b, coupon_c, coupon_d, coupon_e, coupon_f, coupon_g, coupon_h):
        out += fn()
    return out


# ---------------------------------------------------------------------------------------------- COUPON.md
def coupon_md(parts) -> str:
    v = variants()
    rib = " / ".join(fmt(x) for x in v["rib"])
    bore = " / ".join(fmt(x) for x in v["bore"])
    sl, cl = fits.HOLE_SLIDE, fits.HOLE_CLEAR
    cd = gear_cd()
    L = ["# Fit coupon", "",
         "Print this set FIRST, before any real part. Each tile tells you one number in `base/cad/lib/fits.py`.",
         "Print everything in PLA with your normal profile (0.4 nozzle, 0.2 layers, 4 walls), flat as exported, no",
         "supports. Let the parts cool fully (10 minutes) before testing. The first character of every label is the",
         "tile letter below; the rest is the value that tile was made with.", "",
         "How to change a fit: open `base/cad/lib/fits.py`, change the one number named in the step, save, then rerun",
         "`python -m tools.coupon out` (from `base/`) and the audit of your subsystem. Change one number at a time.",
         "If none of the variants is right, pick the closest one, move the number one more step in that direction",
         "and print only that tile again.", "",
         "## Parts", "", "| part | qty | print bbox mm | what |", "|---|---|---|---|"]
    for p in parts:
        lo, hi = prim.bbox(prim.place(p.solid, p.print_tf))
        dims = " x ".join(f"{hi[i] - lo[i]:.1f}" for i in range(3))
        L.append(f"| {p.name} | {p.qty} | {dims} | {p.notes} |")
    L += ["",
          f"## A. 608 bearing seat (labels A {rib}: rib interference, mm)", "",
          "You need: three 608 bearings (or one, pressed in and out).",
          "1. Press a 608 into each tile with your thumb or a vice. Use a flat block, keep it square.",
          "2. Good: it goes in with a firm push, sits flat on the lip, and does not fall out when you turn the tile",
          "   over and tap it. The outer race must not be pinched: spin the bearing, it should spin as freely as",
          "   before.",
          "3. Too tight (needs a hammer, tile cracks, bearing feels rough): pick a smaller value.",
          "   Too loose (falls out, wobbles): pick a bigger value.",
          f"4. In fits.py set `BRG_RIB_INTERF` to the value printed on the best tile (now {fits.BRG_RIB_INTERF}).", "",
          f"## B. 20 mm keyed bore (labels B {bore}: bore clearance, mm diametral)", "",
          "1. Push the printed keyed shaft stub (coupon_b_key_shaft) into each tile, key in the keyway.",
          "2. Good for a running fit: it slides in by hand with no force, turns by the key only, and has no visible",
          "   wobble when you rock it.",
          "3. Too tight: needs force or files: bigger value. Too loose: rocks or clicks: smaller value.",
          f"4. In fits.py set `SHAFT_IN_BORE` to the best value (now {fits.SHAFT_IN_BORE}).",
          f"5. If the key binds but the round part is fine, raise `KEY_CLEAR` by 0.05 (now {fits.KEY_CLEAR}); if the",
          "   key rattles while the bore is good, lower it by 0.05.", "",
          "## C. M3 heat-set inserts (labels 3.8 / 4.0 / 4.2: hole diameter)", "",
          "You need: three M3x4x5 heat-set inserts, soldering iron with an insert tip.",
          "1. Set the iron to about 220 C. Press an insert into each hole, straight, until flush.",
          "2. Good: it goes in with light pressure, a small ring of plastic rises round it, and an M3 screw tightened",
          "   firmly does not spin the insert or pull it out.",
          "3. Spins or pulls out: smaller hole. Needs a lot of force, or plastic bulges a lot: bigger hole.",
          f"4. In fits.py set `M3_INSERT_D` to the best hole (now {fits.M3_INSERT_D}).", "",
          "## D. M3 nut traps (labels 5.6 / 5.8 / 6.0: across flats)", "",
          "1. Drop an M3 nut into each hex pocket.",
          "2. Good: it presses in with a finger, stays put when you turn the tile over, and does not turn when you",
          "   drive a screw in from below.",
          "3. Will not go in: bigger. Falls out: smaller (a nut that just drops in is fine if a screw holds it).",
          f"4. In fits.py set `M3_NUT_AF` to the best value (now {fits.M3_NUT_AF:.2f}).", "",
          "## E. M3 holes (labels 2.5 / 2.6 / 2.7 / 3.2 / 3.4: hole diameter)", "",
          "1. Drive an M3 screw into each of 2.5, 2.6, 2.7 (self tapping into plastic) and push one through 3.2",
          "   and 3.4.",
          "2. Tap hole good: the screw cuts its thread with moderate force and holds when you pull hard, the plastic",
          f"   does not split. Set `M3_TAP_D` to the best one (now {fits.M3_TAP_D:.1f}).",
          "3. Close hole good: the screw passes with a little drag, no play.",
          f"   Set `M3_CLOSE_D` to it (now {fits.M3_CLOSE_D:.1f}).",
          "4. Clearance hole good: the screw drops through freely.",
          f"   Set `M3_CLEAR_D` to it (now {fits.M3_CLEAR_D:.1f}). If 3.2 is already loose, it can come down too.",
          "5. These holes print vertical. Holes printed lying down come out smaller: that extra is",
          f"   `HOLE_HORIZ_EXTRA` (now {fits.HOLE_HORIZ_EXTRA}), leave it unless a horizontal hole is tight.", "",
          f"## F. d 6 snap pin and clip pin (1 notch = fit {fmt(sl)}, 2 notches = fit {fmt(cl)})", "",
          "The fit is taken off the pin shank: shank d = 6 minus the fit.",
          "The host plate has 6 mm holes: top row for the snap pins, bottom row for the clip pins (the labels 1 and 2",
          "only say which pin to try there). Print two C clips.",
          "1. Push each snap pin through a top hole until the barbs click past the far face.",
          "2. Put each clip pin through a bottom hole and push a C clip onto the groove from the side.",
          "3. Good: the pin turns freely in the hole with very little side play, the snap pin needs a firm push and",
          "   does not fall out, the clip snaps on with a click and stays.",
          f"4. If the 1 notch pins (fit {fmt(sl)}) are best for sliding, keep `HOLE_SLIDE` = {sl}. If they bind and",
          f"   the 2 notch pins (fit {fmt(cl)}) are right,",
          f"   set `HOLE_SLIDE` to about {fmt((sl + cl) / 2)} to {fmt(cl)}.",
          f"   If the 2 notch pins rattle, `HOLE_CLEAR` (now {cl}) can come down by 0.05 to 0.1.",
          "5. If a prong snaps off or the clip cracks, tell the CAD side (clips.py proportions), not fits.py.", "",
          "## G. Dyneema eye (labels G 3 and 5: plate thickness, mm)", "",
          "1. Thread 1 mm Dyneema through both eyes and saw it back and forth under load for a minute.",
          "2. Good: the line runs smoothly, no fuzz on the line, no sharp edge you can feel with a fingernail at",
          "   either face.",
          f"3. If the eye is rough, sand the cones; if the throat is too tight to thread, raise `line.EYE_THROAT_D`",
          f"   (now {line.EYE_THROAT_D}); this one is in line.py, not fits.py.", "",
          f"## H. Gear mesh m1.5 z12 / z24 (label H 1.5), centre distance {cd:.2f} mm", "",
          "1. Drop both gears onto the pins of the gear plate and turn the big gear by hand through several turns.",
          "2. Good: smooth, no tight spot, a tiny bit of play you can just feel when you rock one gear while holding",
          "   the other.",
          f"3. Tight or notchy: raise `GEAR_CENTER_EXTRA` (now {fits.GEAR_CENTER_EXTRA}) by 0.05 or `GEAR_BACKLASH`",
          f"   (now {fits.GEAR_BACKLASH}) by 0.05. Lots of play (clacks when reversing): lower one of them by 0.05.",
          f"4. If a gear is tight on its pin but the mesh is fine, raise `SHAFT_IN_BORE` (now {fits.SHAFT_IN_BORE});",
          "   check tile B first, it tests the same number.", "",
          "## Record", "",
          "Write the numbers you chose and the date at the top of fits.py as a comment, for example",
          "`# coupon 2026-11-10: BRG_RIB_INTERF 0.15, SHAFT_IN_BORE 0.30, ...`, then commit.", ""]
    return "\n".join(L)


# ---------------------------------------------------------------------------------------------- main
def write(out_dir: str, log=print) -> dict:
    """Write the STLs, 3MF plates and COUPON.md into out_dir/coupon. Returns a small report dict."""
    import glob
    from tools import pack
    d = os.path.join(out_dir, "coupon")
    os.makedirs(d, exist_ok=True)
    for f in glob.glob(os.path.join(d, "*")):
        if os.path.isfile(f):
            os.remove(f)
    parts = coupon_parts()
    meshes, items, rep = {}, [], dict(parts=[], plates=[], errors=[])
    for p in parts:
        m = prim.to_trimesh(prim.place(p.solid, p.print_tf))
        m.apply_translation(-m.bounds[0])
        m.export(os.path.join(d, f"{p.name}.stl"))
        meshes[p.name] = m
        dims = [float(x) for x in m.extents]
        items.append(dict(name=p.name, material=p.material, dims=dims, qty=p.qty))
        rep["parts"].append(dict(name=p.name, dims=[round(x, 1) for x in dims], qty=p.qty))
    with open(os.path.join(d, "COUPON.md"), "w") as fh:
        fh.write(coupon_md(parts))
    plates, errs = pack.pack_plates(items, pack.BED, 5.0, 5.0)
    rep["errors"] += errs
    qty = {p.name: p.qty for p in parts}
    for i, pl in enumerate(plates, 1):
        objs = []
        for it in pl["items"]:
            on = it["name"] if qty[it["name"]] == 1 else f"{it['name']}_c{it['copy']}"
            objs.append((on, pack.place_on_plate(meshes[it["name"]], it)))
        fn = os.path.join(d, f"coupon_plate_{i}.3mf")
        pack.write_3mf(fn, objs)
        n = pack.count_3mf_objects(fn)
        if n != len(objs):
            rep["errors"].append(f"coupon_plate_{i}.3mf reopens with {n} objects, expected {len(objs)}")
        pack.plate_png(os.path.join(d, f"coupon_plate_{i}.png"), f"fit coupon plate {i} ({len(objs)} objects)",
                       objs, pack.BED)
        rep["plates"].append(dict(file=os.path.basename(fn), objects=len(objs)))
    log(f"coupon: {len(parts)} parts, {len(rep['plates'])} plate(s) -> {d}")
    for r in rep["parts"]:
        q = f"  qty {r['qty']}" if r["qty"] > 1 else ""
        log(f"  {r['name']:26s} {' x '.join(f'{x:.1f}' for x in r['dims'])}{q}")
    for pl in rep["plates"]:
        log(f"  {pl['file']}: {pl['objects']} objects")
    for e in rep["errors"]:
        log(f"  ERROR {e}")
    return rep


def main(argv=None):
    ap = argparse.ArgumentParser(description="Write the fit coupon (STL, 3MF, COUPON.md) into OUT/coupon")
    ap.add_argument("out")
    a = ap.parse_args(argv)
    rep = write(a.out)
    return 2 if rep["errors"] else 0


if __name__ == "__main__":
    sys.exit(main())
