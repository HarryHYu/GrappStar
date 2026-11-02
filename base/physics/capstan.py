"""Capstan (Euler-Eytelwein) and the wiper tow model through the capstan train.

Line path in REEL mode: dart -> capstan (n wraps) -> storage reel. Dart side tension F, reel side (hold) tension T_hold
from the storage reel slip clutch. Grip holds while F <= T_hold * exp(2 pi mu n).

Drive: wiper output -> capstan ratio (wiper turns per drum turn) at efficiency eta -> sprag #2 -> drum radius r.
Drum torque = (F - T_hold) r. The same train drives the storage reel take-up through the slip clutch, geared to
overspeed the line by `overspeed` at the reel core, so the clutch always slips and costs T_hold x overspeed of line
pull at the drum.

Wrap margin: design wraps = ceil(1.5 x raw wraps), at mu low. 1.5x on wraps is the same as mu dropping by a third
(wet or dusty line).

python capstan.py [--tow 50] [--mu 0.15]   -> table + base/out/physics/capstan.json
"""
from __future__ import annotations

import argparse
import json
import math
import os

from common import load_params, Params, PLA
import gears

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "out", "physics")

WRAP_MARGIN = 1.5
DART_HOLD = dict(sand=7.0, loam=45.0, clay=112.0)   # N, Broms short pile (kedge CLAUDE.md)


# ------------------------------------------------------------------ Euler-Eytelwein
def ratio(mu: float, wraps: float) -> float:
    return math.exp(2 * math.pi * mu * wraps)


def wraps_needed(f_tow: float, t_hold: float, mu: float, margin: float = WRAP_MARGIN) -> dict:
    raw = math.log(f_tow / t_hold) / (2 * math.pi * mu) if f_tow > t_hold else 0.0
    return dict(raw=raw, margin=margin, design=math.ceil(raw * margin))


def max_hold(wraps: float, t_hold: float, mu: float) -> float:
    """Largest dart side tension the wraps hold before the line slips on the drum."""
    return t_hold * ratio(mu, wraps)


def takeup_needed(f_tow: float, wraps: float, mu: float) -> float:
    return f_tow / ratio(mu, wraps)


def slips(f_tow: float, wraps: float, t_hold: float, mu: float) -> bool:
    return f_tow > max_hold(wraps, t_hold, mu)


# ------------------------------------------------------------------ drive train
def _cap(p: Params):
    c = p.capstan
    return c["d"] / 2, c["ratio"], c["eta"], c["takeup_n"]


def wiper_torque_for(f_line: float, p: Params, overspeed: float = 1.2) -> float:
    """Wiper output torque to pull the dart side at f_line, incl. the take-up drive (clutch slipping)."""
    r, k, eta, th = _cap(p)
    return r * (max(f_line - th, 0.0) + overspeed * th) / (k * eta)


def line_from_wiper(t_wiper: float, p: Params, overspeed: float = 1.2) -> float:
    r, k, eta, th = _cap(p)
    return t_wiper * k * eta / r + th - overspeed * th


def stall_torque_sag(p: Params, v_oc: float) -> float:
    w, b = p.wiper, p.battery
    i_s = v_oc / (w.r_winding + b.r_source)
    return w.kt * (i_s - w.i_free), i_s


def tow_point(f_line: float, p: Params, v_oc: float, overspeed: float = 1.2) -> dict:
    """Steady tow at dart side tension f_line, battery at v_oc with sag."""
    r, k, eta, th = _cap(p)
    t_w = wiper_torque_for(f_line, p, overspeed)
    op = p.wiper.operate(t_w, p.battery, v_oc)
    w_drum = op["speed"] / k
    return dict(f_line=f_line, t_wiper=t_w, t_drum=t_w * k * eta, rpm_wiper=op["rpm"], rpm_drum=op["rpm"] / k,
                v_line=w_drum * r, current=op["current"], i_batt=op["i_batt"], v_batt=op["v_batt"],
                power_in=op["power_in"], stalled=op["stalled"])


def current_limit_for(f_limit: float, p: Params, overspeed: float = 1.2) -> dict:
    """Motor current (what the BTS7960 IS pin reads, independent of PWM duty) at the tow limit."""
    t_w = wiper_torque_for(f_limit, p, overspeed)
    i = p.wiper.current_at(t_w)
    return dict(f_limit=f_limit, t_wiper=t_w, current=i, i_free=p.wiper.i_free,
                delta_over_free=i - p.wiper.i_free)


# ------------------------------------------------------------------ printed parts
def part_stresses(f_line: float, p: Params, wraps: int, rpm_drum: float, drum_wall_m: float = 0.010,
                  line_pitch_m: float = 1.05e-3, p_allow: float = PLA.sigma_lewis, bearing_split: float = 0.5,
                  wrap_angle_deg: float = 180.0, key_len_mm: float = 14.0) -> dict:
    """drum_wall_m: drum shell between the line and the 20 mm bore (40 mm drum: 10 mm). p_allow: line bearing and
    shell hoop allowable on printed PLA (sigma_lewis, conservative). bearing_split: share of the resultant on the most
    loaded 608 (0.5 = drum centred between two). wrap_angle_deg: angle between the dart line and the reel line leaving
    the drum (180 = parallel, worst case: resultant F + T_hold)."""
    r, k, eta, th = _cap(p)
    t_drum = max(f_line - th, 0.0) * r
    # line bearing pressure on the drum under the loaded wrap: T / (r d_line)
    p_line = f_line / (r * p.line.d)
    # shell hoop compression under the wraps: ring with p = T/(r pitch) -> sigma = p r / t = T / (pitch t)
    hoop = f_line / (line_pitch_m * drum_wall_m)
    phi = math.radians(wrap_angle_deg)
    resultant = math.sqrt(f_line ** 2 + th ** 2 - 2 * f_line * th * math.cos(phi))
    return dict(
        t_drum=t_drum,
        shaft=gears.shaft_torsion(t_drum, p.shaft),
        key=gears.key(t_drum, key_len_mm, p.shaft),
        sprag=gears.sprag(t_drum, rpm_drum, p.sprag),
        line_pressure_mpa=p_line / 1e6, hoop_mpa=hoop / 1e6, p_allow_mpa=p_allow / 1e6,
        drum_pass=max(p_line, hoop) <= p_allow,
        radial_n=resultant, bearing608=gears.bearing608(resultant * bearing_split, rpm_drum, p.bearing),
        wraps=wraps,
    )


# ------------------------------------------------------------------ full run
def run(p: Params | None = None, tow: float | None = None, mu: float | None = None, light_n: float = 3.0,
        reel_len: float = 20.0, overspeed: float = 1.2) -> dict:
    p = p or load_params()
    c = p.capstan
    tow = c["tow_limit_n"] if tow is None else tow
    mu_lo, mu_hi = PLA.mu_dyneema
    mu = mu_lo if mu is None else mu
    th, wraps = c["takeup_n"], c["wraps"]
    b = p.battery

    wn = wraps_needed(tow, th, mu)
    loads = [5, 10, 20, 30, 45, 50, 75, 100, 112]
    table = {}
    for name, v in (("full", b.v_full), ("nom", b.v_nom)):
        table[name] = [tow_point(f, p, v, overspeed) for f in loads]

    stall = {}
    for name, v in (("full", b.v_full), ("nom", b.v_nom)):
        t_s, i_s = stall_torque_sag(p, v)
        f_s = line_from_wiper(t_s, p, overspeed)
        stall[name] = dict(v_oc=v, t_wiper=t_s, current=i_s, f_line=f_s,
                           exceeds_dart={k: f_s > hv for k, hv in DART_HOLD.items()},
                           exceeds_break=f_s > p.line.breaking, line_breaking=p.line.breaking,
                           wraps_slip_first=slips(f_s, wraps, th, mu_hi),
                           holds_max=max_hold(wraps, th, mu_hi))

    light = tow_point(light_n, p, b.v_nom, overspeed)
    t_reel = reel_len / light["v_line"] if light["v_line"] > 0 else math.inf
    at_tow = tow_point(tow, p, b.v_nom, overspeed)
    t_stall_full = stall["full"]

    res = dict(
        inputs=dict(tow=tow, mu=mu, mu_range=(mu_lo, mu_hi), takeup_n=th, wraps=wraps, drum_d=c["d"],
                    ratio=c["ratio"], eta=c["eta"], overspeed=overspeed, light_n=light_n, source=p.source),
        wraps_needed=wn,
        capstan_ratio=ratio(mu, wraps),
        max_hold_n=dict(mu_low=max_hold(wraps, th, mu_lo), mu_high=max_hold(wraps, th, mu_hi)),
        takeup_needed_n=takeup_needed(tow, wraps, mu),
        slips_at_tow=slips(tow, wraps, th, mu),
        tow_table=table,
        at_tow_nom=at_tow,
        current_limit=current_limit_for(tow, p, overspeed),
        current_limit_dart=dict({k: current_limit_for(v, p, overspeed)["current"] for k, v in DART_HOLD.items()}),
        stall=stall,
        reel=dict(length=reel_len, light=light, time_s=t_reel),
        parts_at_tow=part_stresses(tow, p, wraps, at_tow["rpm_drum"]),
        parts_at_stall=part_stresses(t_stall_full["f_line"], p, wraps, 0.0),
    )
    res["warnings"] = warnings(res)
    return res


def warnings(r: dict) -> list:
    w = []
    s = r["stall"]["full"]
    over = [k for k, v in s["exceeds_dart"].items() if v]
    if over:
        w.append(f"wiper stall pulls {s['f_line']:.0f} N on the line: above dart holding in {', '.join(over)}")
    if s["exceeds_break"]:
        w.append(f"wiper stall line tension {s['f_line']:.0f} N exceeds line breaking {s['line_breaking']:.0f} N")
    if not s["wraps_slip_first"]:
        w.append(f"wraps hold {s['holds_max']:.0f} N at mu high: the capstan is no torque limiter, firmware current "
                 "limit (or a slip clutch) must protect the dart and line")
    if r["slips_at_tow"]:
        w.append("line slips on the drum at the tow limit")
    if r["inputs"]["wraps"] < r["wraps_needed"]["design"]:
        w.append(f"wraps {r['inputs']['wraps']} < design {r['wraps_needed']['design']} (1.5x margin)")
    cl = r["current_limit"]
    if cl["delta_over_free"] < 2.0:
        w.append(f"tow limit is only {cl['delta_over_free']:.2f} A above free current: current sensing of tension is "
                 "coarse, calibrate on the bench or use the load cell")
    for tag in ("parts_at_tow", "parts_at_stall"):
        ps = r[tag]
        for k in ("shaft", "key", "sprag", "bearing608"):
            if not ps[k]["pass"]:
                w.append(f"{tag}: {k} fails")
        if not ps["drum_pass"]:
            w.append(f"{tag}: drum line pressure {ps['line_pressure_mpa']:.1f} / hoop {ps['hoop_mpa']:.1f} MPa over "
                     f"{ps['p_allow_mpa']:.0f} MPa")
    return w


def write_json(res: dict, name: str) -> str:
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, name)
    with open(path, "w") as f:
        json.dump(res, f, indent=1, default=float)
    return os.path.normpath(path)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--tow", type=float, default=None)
    ap.add_argument("--mu", type=float, default=None)
    a = ap.parse_args(argv)
    r = run(tow=a.tow, mu=a.mu)
    i = r["inputs"]
    print(f"capstan d {i['drum_d']*1000:.0f} mm, ratio {i['ratio']}, eta {i['eta']}, wraps {i['wraps']}, "
          f"take-up {i['takeup_n']} N, mu {i['mu']}, tow {i['tow']} N  [{i['source']}]")
    wn = r["wraps_needed"]
    print(f"wraps needed: raw {wn['raw']:.2f}, design {wn['design']} (x{wn['margin']})   "
          f"e^(2 pi mu n) = {r['capstan_ratio']:.1f}")
    print(f"max hold with {i['wraps']} wraps: {r['max_hold_n']['mu_low']:.0f} N (mu low), "
          f"{r['max_hold_n']['mu_high']:.0f} N (mu high); take-up needed for tow: {r['takeup_needed_n']:.2f} N")
    print(f"\n{'F N':>6} {'T_w Nm':>7} {'rpm':>6} {'v m/s':>6} {'I A':>6} {'Vbat':>6}   (11.1 V nominal, sag)")
    for row in r["tow_table"]["nom"]:
        print(f"{row['f_line']:6.0f} {row['t_wiper']:7.2f} {row['rpm_wiper']:6.1f} {row['v_line']:6.3f} "
              f"{row['current']:6.2f} {row['v_batt']:6.2f}{'  STALL' if row['stalled'] else ''}")
    for k, s in r["stall"].items():
        print(f"stall at {s['v_oc']} V: {s['t_wiper']:.2f} N m, {s['current']:.1f} A, line {s['f_line']:.0f} N")
    cl = r["current_limit"]
    print(f"firmware current limit for {cl['f_limit']:.0f} N: {cl['current']:.2f} A motor current "
          f"({cl['t_wiper']:.2f} N m wiper)")
    print("limits for dart holding:", {k: round(v, 2) for k, v in r["current_limit_dart"].items()}, "A")
    print(f"reel {r['reel']['length']:.0f} m at {i['light_n']} N: {r['reel']['light']['v_line']:.3f} m/s, "
          f"{r['reel']['time_s']:.0f} s")
    pt = r["parts_at_tow"]
    print(f"parts at tow: drum T {pt['t_drum']:.2f} N m, line p {pt['line_pressure_mpa']:.1f} MPa, hoop "
          f"{pt['hoop_mpa']:.1f} MPa, 608 radial {pt['bearing608']['radial_n']:.0f} N (s0 {pt['bearing608']['s0']:.1f})")
    ps = r["parts_at_stall"]
    print(f"parts at stall: drum T {ps['t_drum']:.2f} N m (shaft {ps['shaft']['pass']}, key {ps['key']['pass']}, "
          f"sprag {ps['sprag']['pass']}), line p {ps['line_pressure_mpa']:.1f} MPa")
    print("\nWARNINGS:")
    for w in r["warnings"]:
        print(" -", w)
    print("wrote", write_json(r, "capstan.json"))
    return r


if __name__ == "__main__":
    main()
