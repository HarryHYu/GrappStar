"""One-page summary of every physics number: base/physics/SUMMARY.md and base/out/physics/physics_summary.pdf
(plain black Liberation Sans, no name). Runs every model on the current params (spec overlay included).

python summary.py [--fast]
"""
from __future__ import annotations

import argparse
import json
import math
import os

from common import load_params

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "out", "physics")
BALL = os.path.join(HERE, "..", "out", "ballistics.json")
FONT = "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"
FONT_B = "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"
MEASURE_TITLE = "To measure (then edit base/spec/params.py and run ./run_all.sh)"
STALL = "STALL: cannot {} with this wiper"


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
    from launcher import effective_muzzle, line_slack
    em = effective_muzzle(p, pitch_best)
    ls = line_slack(p)
    k_v = float(p.launch.get("v_scale", 1.0))     # test-shot factor (spec LAUNCH v_scale), as ballistics.py applies it
    v = em["v"] * k_v
    fl = [simulate(p, v, pitch_best, brake_n=b, reel_model=True) for b in (0.5, 0.75, 1.0)]
    nt = simulate(p, em["v_muzzle"] * k_v, pitch_best, tether=False)   # no line at all: no load in the stroke, no snatch
    rows = [("Line path at the shot", (f"slack {ls['slack_at_release_m'] * 1000:.0f} mm: snatch, flight starts at {v:.2f} m/s"
                                       if em["snatch"] else "taut (path only lengthens)")),
            ("Range at 0.5 / 0.75 / 1.0 N brake", " / ".join(f"{r['range_m']:.2f}" for r in fl) + f" m (pitch {pitch_best:.0f})"),
            ("Same shot, no tether", f"{nt['range_m']:.2f} m"),
            ("Flight time, apex (0.75 N)", f"{fl[1]['t_flight_s']:.2f} s, {fl[1]['apex_m']:.2f} m"),
            ("Impact speed, angle (0.75 N)", f"{fl[1]['v_impact']:.1f} m/s, {fl[1]['impact_angle_deg']:.0f} deg down")]
    if k_v != 1.0:
        rows.insert(0, ("Muzzle speed factor from test shots", f"x{k_v:.3f} (spec LAUNCH v_scale)"))
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
    d = drivetrain.run(p)
    e = d["electrical"]
    tow_ok = not (_a["stalled"] or _b["stalled"]) and _b["current"] > _a["current"]
    rows = [(f"Wraps for tow limit at the capstan ({c['inputs']['tow_capstan']:.0f} N, mu 0.15)",
             f"{c['wraps_needed']['raw']:.1f} raw, {c['wraps_needed']['design']:.0f} design"),
            (f"Take-up needed for the tow at the actual wraps ({c['inputs']['wraps']})", f"{c['takeup_needed_n']:.2f} N")]
    tn = c["at_tow_nom"]
    if tn.get("stalled") or tn["v_line"] <= 0 or tn["current"] > capstan.trip_a(p, "tow"):
        rows.append(("Tow at limit", STALL.format("tow")))
    else:
        rows.append(("Tow at limit: torque, speed, current", f"{tn['t_wiper']:.2f} N m, {tn['v_line']:.3f} m/s, {tn['current']:.2f} A"))
    rows += [(f"Wiper {e['stall_label']}, 12.6 V: torque, current, line pull",
              f"{e['stall_full']['torque']:.1f} N m, {e['stall_full']['current']:.1f} A, {c['stall']['full']['f_line']:.0f} N "
              f"(break {p.line.breaking:.0f} N)")]
    if e["motor_stall_full"]["torque"] > e["stall_full"]["torque"] + 1e-9:     # only when a limiter caps the train
        rows.append(("Wiper motor stall (no limiter), 12.6 V: torque, current, line pull",
                     f"{e['motor_stall_full']['torque']:.1f} N m, {e['motor_stall_full']['current']:.1f} A, "
                     f"{c['motor_stall']['full']['f_line']:.0f} N"))
    if tow_ok:
        n_per_a = 40.0 / (_b["current"] - _a["current"])
        f_off = 20 - n_per_a * (_a["current"] - p.wiper.i_free)
        g = capstan.path_gain(p)                    # capstan-side tension = dart pull x path_gain
        rows.append(("Tow sensing at the dart: N per wiper amp above free",
                     f"dart = {n_per_a:.1f} x (I - I_free) - {-f_off:.1f} N (capstan side x{g:g})"))
    else:
        rows.append(("Tow sensing", STALL.format("tow")))
    sec.append(("Capstan and tow", rows))
    rows = []
    trips = dict(cock="cock", reel="reel_in", tow="tow", aim="aim")
    for k, f in d["functions"].items():
        trip = capstan.trip_a(p, trips.get(k, "reel_in"))
        stalled = (not math.isfinite(f["time_s"]) or f["time_s"] <= 0 or f.get("rpm_mean", 1.0) <= 0
                   or not math.isfinite(f["i_peak"]) or f["i_peak"] > trip)
        if stalled:
            rows.append((f"{k}", STALL.format(k)))
        else:
            rows.append((f"{k}: peak torque, current, time", f"{f['t_peak']:.2f} N m, {f['i_peak']:.1f} A, {f['time_s']:.0f} s"))
    sec.append(("Drivetrain (wiper at 11.1 V with sag)", rows))
    measure = [
        "Spring hang test, each spring: hang 1, 2, 3 kg and measure the length each time (hooks included). "
        "python calib.py spring FREE_MM 1:MM 2:MM 3:MM, then SPRING k_n_per_mm and F0_n (mean of the two); "
        "the stronger spring decides the anchor holes (MEASURE_LIST B4). Every range number scales with the stored energy.",
        "Wiper (MEASURE_LIST A6): free speed and current from a 40 % bench jog in telemetry, then WIPER "
        "n0_rpm = jog rpm x 2.5 (never the raw jog rpm) and free_a = amps. The stall cannot be read (jogs trip at "
        "2.5 to 3 A, A7): keep stall_nm and stall_a.",
        f"Dart mass on a kitchen scale with ballast and tether knot (spec {p.dart.mass:.3f} kg), then DART mass_kg.",
        f"Payout brake (MEASURE_LIST G4, hanging weights: a luggage scale cannot read this low): in LAUNCH 40 g on the "
        f"line must not pull line off and 50 g must (LIGHT {p.flight['brake_n']:g} N); in NEUTRAL the FIRM clutch "
        f"({p.capstan['takeup_n']:g} N) lets line off at about {p.capstan['takeup_n'] / 9.81:.1f} kg on the luggage scale.",
        "First shots on level ground: python calib.py range pitch:brake:range ..., then LAUNCH v_scale = FACTOR.",
    ]
    return dict(source=p.source, sections=sec, measure=measure)


def write_md(s, path):
    lines = ["# Base station physics: summary", "", f"Params: {s['source']}. Generated by base/physics/summary.py.", ""]
    for title, rows in s["sections"]:
        lines += [f"## {title}", "", "| Item | Value |", "|---|---|"]
        lines += [f"| {a} | {b} |" for a, b in rows]
        lines.append("")
    lines += [f"## {MEASURE_TITLE}", ""] + [f"{i}. {m}" for i, m in enumerate(s["measure"], 1)] + [""]
    txt = "\n".join(lines)
    assert "\u2014" not in txt
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
    cv.drawString(m, y, MEASURE_TITLE)
    y -= lh + 1
    from reportlab.lib.utils import simpleSplit
    for n, t in enumerate(s["measure"], 1):
        for i, ln in enumerate(simpleSplit(t, "LS", fs, W - 2 * m - 18)):
            cv.setFont("LS", fs)
            if i == 0:
                cv.drawString(m, y, f"{n}.")
            cv.drawString(m + 10, y, ln)
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
