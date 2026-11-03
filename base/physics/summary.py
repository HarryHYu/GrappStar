"""One-page summary of every physics number: base/physics/SUMMARY.md and base/out/physics/physics_summary.pdf
(plain black Liberation Sans, no name). Runs every model on the current params (spec overlay included).

python summary.py [--fast]
"""
from __future__ import annotations

import argparse
import json
import os

from common import load_params

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "out", "physics")
BALL = os.path.join(HERE, "..", "out", "ballistics.json")
FONT = "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"
FONT_B = "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"


def collect(p=None, fast=False):
    import capstan
    import drivetrain
    from flight import simulate
    from launcher import launch
    from payout import reel_props, spin_up_in_stroke
    p = p or load_params()
    sp = p.spring
    lim = sp.limits()
    su = sp.surge()
    sec = []
    sec.append(("Spring (each, A228, SF 1.2 on Shigley static limits)", [
        ("Rate used / by formula", f"{sp.k / 1000:.3f} / {sp.rate_formula:.3f} N/mm"),
        ("Initial tension used", f"{sp.f_init:.1f} N"),
        ("Body coils, mass", f"{sp.n_body:.0f}, {sp.mass * 1000:.0f} g"),
        ("Safe force (governed by)", f"{lim['f_safe_n']:.1f} N ({lim['governed_by']})"),
        ("Safe extension (wire) / spec cap", f"{lim['ext_safe_mm']:.0f} / {p.launch.get('ext_cap', float('nan')) * 1000:.0f} mm"),
        ("Surge (fixed-fixed)", f"{su['f_fixed_fixed_hz']:.0f} Hz"),
    ]))
    L = launch(p)
    pitch_best = 36.0
    if os.path.exists(BALL):
        pitch_best = max(json.load(open(BALL))["table"], key=lambda r: r["range_m"][1])["pitch_deg"]
    sec.append(("Launch (chain spring model)", [
        ("Stroke, ratio, pitch", f"{L['stroke_m'] * 1000:.0f} mm, {L['pulley_ratio']:.1f}:1, {L['pitch_deg']:.0f} deg"),
        ("Spring extension full / rest", f"{L['ext_full_mm']:.0f} / {L['preload_ext_mm']:.0f} mm"),
        ("Stored energy (2 springs)", f"{L['energy_stored_j']:.1f} J"),
        ("Carriage force at full draw", f"{L['f_carriage_full_n']:.0f} N"),
        ("Muzzle speed", f"{L['v_muzzle']:.2f} m/s"),
        ("Peak acceleration, stroke time", f"{L['a_peak_g']:.0f} g, {L['t_stroke_ms']:.0f} ms"),
        ("Efficiency (dart KE / stored)", f"{L['efficiency'] * 100:.0f} %"),
        ("Carriage KE at stop, pad peak", f"{L['ke_carriage_j']:.2f} J, {L['f_stop_peak_n']:.0f} N ({p.launch['stop_pad_travel'] * 1000:.0f} mm pad)"),
        ("Spring KE at stop, coil clash", f"{L['ke_spring_j']:.2f} J, {L['coil_clash_v']:.1f} m/s"),
    ]))
    v = L["v_muzzle"]
    fl = [simulate(p, v, pitch_best, brake_n=b, reel_model=True) for b in (0.5, 0.75, 1.0)]
    nt = simulate(p, v, pitch_best, tether=False)
    rows = [("Range at 0.5 / 0.75 / 1.0 N brake", " / ".join(f"{r['range_m']:.2f}" for r in fl) + f" m (pitch {pitch_best:.0f})"),
            ("Same shot, no tether", f"{nt['range_m']:.2f} m"),
            ("Flight time, apex (0.75 N)", f"{fl[1]['t_flight_s']:.2f} s, {fl[1]['apex_m']:.2f} m"),
            ("Impact speed, angle (0.75 N)", f"{fl[1]['v_impact']:.1f} m/s, {fl[1]['impact_angle_deg']:.0f} deg down")]
    if os.path.exists(BALL):
        b = json.load(open(BALL))
        best = max(b["table"], key=lambda r: r["range_m"][1])
        rows.append(("Ballistics table", f"{b['pitch_min']:.0f} to {b['pitch_max']:.0f} deg, best {best['pitch_deg']:.1f} deg: {best['range_m'][1]:.2f} m"))
    sec.append(("Flight (bead-chain tether + reel model, level ground)", rows))
    rp = reel_props(p)
    s2 = spin_up_in_stroke(p, v, L["stroke_m"], p.flight["brake_n"])
    pr = simulate(p, v, pitch_best, brake_n=p.flight["brake_n"], reel_model=True)
    sec.append((f"Storage reel payout (LIGHT {p.flight['brake_n']} N at full reel)", [
        ("Reel empty mass, inertia full", f"{rp['mass_empty'] * 1000:.0f} g, {rp['inertia_full'] * 1e6:.1f} kg mm2"),
        ("Line radius full / core", f"{rp['r_full'] * 1000:.1f} / {rp['r_core'] * 1000:.1f} mm (brake +{(rp['r_full'] / rp['r_core'] - 1) * 100:.0f} % at core)"),
        ("Line pull in the stroke (spin-up)", f"{s2['t_line_in_stroke_n']:.2f} N"),
        ("Reel rpm max", f"{pr['reel_rpm_max']:.0f} rpm"),
        ("Overrun after landing", f"{pr['overrun_after_landing_m']:.2f} m in {pr['reel_stop_after_landing_s']:.2f} s"),
    ]))
    c = capstan.run(p)
    _a, _b = capstan.tow_point(20, p, p.battery.v_nom), capstan.tow_point(60, p, p.battery.v_nom)
    n_per_a = 40.0 / (_b["current"] - _a["current"])
    f_off = 20 - n_per_a * (_a["current"] - p.wiper.i_free)
    sec.append(("Capstan and tow", [
        ("Wraps for tow limit (mu 0.15)", f"{c['wraps_needed']['raw']:.1f} raw, {c['wraps_needed']['design']:.0f} design"),
        ("Take-up needed (design wraps)", f"{c['takeup_needed_n']:.2f} N"),
        ("Tow at limit: torque, speed, current", f"{c['at_tow_nom']['t_wiper']:.2f} N m, {c['at_tow_nom']['v_line']:.3f} m/s, {c['at_tow_nom']['current']:.2f} A"),
        (f"Line pull {c['stall']['full']['label']} (12.6 V)", f"{c['stall']['full']['f_line']:.0f} N (break {p.line.breaking:.0f} N)"),
        ("Line pull, motor stall (no limiter, 12.6 V)", f"{c['motor_stall']['full']['f_line']:.0f} N"),
            ("Tow sensing: line N per wiper amp above free", f"{n_per_a:.1f} N/A (line = {n_per_a:.1f} x (I - I_free) - {-f_off:.1f} N)"),
]))
    d = drivetrain.run(p)
    rows = []
    for k, f in d["functions"].items():
        rows.append((f"{k}: peak torque, current, time", f"{f['t_peak']:.2f} N m, {f['i_peak']:.1f} A, {f['time_s']:.0f} s"))
    e = d["electrical"]
    rows.append((f"Wiper {e['stall_label']}, 12.6 V (with sag)", f"{e['stall_full']['torque']:.1f} N m, {e['stall_full']['current']:.1f} A"))
    rows.append(("Wiper motor stall (no limiter), 12.6 V (with sag)", f"{e['motor_stall_full']['torque']:.1f} N m, {e['motor_stall_full']['current']:.1f} A"))
    sec.append(("Drivetrain (wiper at 11.1 V with sag)", rows))
    measure = [
        "Spring hang test: hang 1, 2, 3 kg from one spring, measure the length each time. Slope = rate, intercept = "
        "initial tension. Every range number scales with the stored energy.",
        "Wiper: free speed (turns in 30 s at 12.0 V), free current, stall torque (arm + luggage scale, 1 s) and stall "
        "current. Edit common.Wiper or spec WIPER.",
        "Dart mass on a kitchen scale with ballast and tether knot (spec 0.279 kg).",
        "Slip clutch: pull the line off the full reel with a luggage scale; set 0.4 to 0.5 N (FINDINGS F5).",
        "Rail friction: tilt the rail with the dart on it until it slides (mu = tan angle).",
    ]
    return dict(source=p.source, sections=sec, measure=measure)


def write_md(s, path):
    lines = ["# Base station physics: summary", "", f"Params: {s['source']}. Generated by base/physics/summary.py.", ""]
    for title, rows in s["sections"]:
        lines += [f"## {title}", "", "| Item | Value |", "|---|---|"]
        lines += [f"| {a} | {b} |" for a, b in rows]
        lines.append("")
    lines += ["## Harry must measure", ""] + [f"- {m}" for m in s["measure"]] + [""]
    txt = "\n".join(lines)
    assert "—" not in txt
    open(path, "w").write(txt)


def write_pdf(s, path):
    from reportlab.lib.pagesizes import A4
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont
    from reportlab.pdfgen import canvas
    pdfmetrics.registerFont(TTFont("LS", FONT))
    pdfmetrics.registerFont(TTFont("LSB", FONT_B))
    W, H = A4
    cv = canvas.Canvas(path, pagesize=A4, initialFontName="LS", initialFontSize=8)
    cv.setTitle("Base station physics summary")
    cv.setAuthor("")
    m = 36
    y = H - m
    cv.setFont("LSB", 12)
    cv.drawString(m, y, "Base station physics: summary")
    y -= 13
    cv.setFont("LS", 7.5)
    cv.drawString(m, y, f"Params: {s['source']}")
    y -= 12
    colw = (W - 2 * m - 14) / 2
    col_x = [m, m + colw + 14]
    col = 0
    y_top = y
    fs = 7.6
    lh = 9.4
    for title, rows in s["sections"]:
        need = lh * (len(rows) + 1.6)
        if col == 0 and (y - need < m + 120 or title.startswith("Storage reel")):
            col = 1
            y = y_top
        x = col_x[col]
        cv.setFont("LSB", 8.4)
        cv.drawString(x, y, title)
        y -= lh + 1
        for a, b in rows:
            cv.setFont("LS", fs)
            cv.drawString(x, y, a)
            cv.drawRightString(x + colw, y, b)
            y -= lh
        y -= 5
    y = min(y, m + 112) if col == 1 else y
    y = m + 110
    cv.setFont("LSB", 8.4)
    cv.drawString(m, y, "Harry must measure (then edit the parameter block and rerun: python summary.py)")
    y -= lh + 1
    from reportlab.lib.utils import simpleSplit
    for t in s["measure"]:
        for i, ln in enumerate(simpleSplit(t, "LS", fs, W - 2 * m - 10)):
            cv.setFont("LS", fs)
            cv.drawString(m + (0 if i == 0 else 8), y, ("- " if i == 0 else "") + ln)
            y -= lh
    cv.showPage()
    cv.save()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--fast", action="store_true")
    a = ap.parse_args(argv)
    s = collect(fast=a.fast)
    os.makedirs(OUT_DIR, exist_ok=True)
    write_md(s, os.path.join(HERE, "SUMMARY.md"))
    write_pdf(s, os.path.join(OUT_DIR, "physics_summary.pdf"))
    json.dump(s, open(os.path.join(OUT_DIR, "summary.json"), "w"), indent=1)
    for title, rows in s["sections"]:
        print(title)
        for r in rows:
            print(f"  {r[0]:40s} {r[1]}")
    print("wrote SUMMARY.md and", os.path.join(OUT_DIR, "physics_summary.pdf"))


if __name__ == "__main__":
    main()
