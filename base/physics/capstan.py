"""Capstan (Euler-Eytelwein) and the wiper tow model through the capstan train.

Line path in REEL mode: dart -> capstan (n wraps) -> storage reel. Dart side tension F, reel side (hold) tension T_hold
from the storage reel slip clutch. Grip holds while F <= T_hold * exp(2 pi mu n).

Take-up: the storage reel slip clutch slips at a constant TORQUE Tc, set as p.capstan["takeup_n"] of line tension at
clutch_r (default the reel core radius). The hold tension at the capstan is Tc / r_line, and r_line grows as line
winds on (payout.line_radius), so T_hold falls from takeup_n (empty reel) to Tc / r_full (20 m on the reel).
The take-up is geared from the capstan train to overspeed the line by `overspeed` at the core, so the clutch always
slips; at a larger line radius the overspeed is overspeed x r_line / r_core. Its drive costs Tc x overspeed / r_core of
line pull at the drum, constant.

Drive: wiper output -> capstan ratio (wiper turns per drum turn) at efficiency eta -> sprag #2 -> drum radius r.
Drum keyed shaft torque = wiper torque x ratio x eta (it carries the take-up branch as well).

Wrap margin: design wraps = ceil(1.5 x raw wraps), at mu low and the lowest hold tension. 1.5x on wraps is the same
as mu dropping by a third (wet or dusty line).

python capstan.py [--tow 50] [--mu 0.15]   -> table + base/out/physics/capstan.json
"""
from __future__ import annotations

import argparse
import json
import math
import os

from common import load_params, Params, PLA, PPA_CF
import gears
import payout

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "out", "physics")

WRAP_MARGIN = 1.5
DART_HOLD = dict(sand=7.0, loam=45.0, clay=112.0)   # N, Broms short pile (kedge CLAUDE.md)
CAP_GEAR = dict(module=2.0, face=12.0, z_driver=20)  # ASSUMED wiper to capstan mesh, PPA-CF

# recommended firmware current trip (all modes)
TRIP_BLANK_MS = 50       # ignore current after start (inrush)
TRIP_HARD_A = 5.0        # hard trip, motor current (aim, reel, tow)
TRIP_COCK_A = 6.5        # hard trip while cocking (launch mode forward): cock peak is ~4.6 A
TRIP_HARD_MS = 30
TRIP_TOW_AVG_MS = 200
CLUTCH_REEL_NM = 3.0     # recommended mechanical slip clutch on the REEL output gear


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


# ------------------------------------------------------------------ take-up (slip clutch, constant torque)
def _core_r(p: Params) -> float:
    return p.reel["core_d"] / 2


def clutch_torque(p: Params, clutch_r: float | None = None) -> float:
    return p.capstan["takeup_n"] * (_core_r(p) if clutch_r is None else clutch_r)


def takeup_tension(p: Params, line_left: float, clutch_r: float | None = None) -> float:
    return clutch_torque(p, clutch_r) / payout.line_radius(p, line_left)


def takeup_drive_force(p: Params, overspeed: float = 1.2, clutch_r: float | None = None) -> float:
    """Line pull equivalent at the drum spent driving the slipping take-up."""
    return clutch_torque(p, clutch_r) * overspeed / _core_r(p)


def takeup_range(p: Params, overspeed: float = 1.2, clutch_r: float | None = None, l_max: float | None = None) -> dict:
    l_max = p.reel["line_stored"] if l_max is None else l_max
    r0, r1 = payout.line_radius(p, 0.0), payout.line_radius(p, l_max)
    tc = clutch_torque(p, clutch_r)
    return dict(clutch_torque=tc, clutch_r=_core_r(p) if clutch_r is None else clutch_r, r_empty=r0, r_full=r1,
                l_max=l_max, t_hold_max=tc / r0, t_hold_min=tc / r1,
                overspeed_min=overspeed * r0 / _core_r(p), overspeed_max=overspeed * r1 / _core_r(p))


# ------------------------------------------------------------------ drive train
def _cap(p: Params):
    c = p.capstan
    return c["d"] / 2, c["ratio"], c["eta"]


def _full(p: Params) -> float:
    return p.reel["line_stored"]


def wiper_torque_for(f_line: float, p: Params, overspeed: float = 1.2, line_left: float | None = None,
                     clutch_r: float | None = None) -> float:
    """Wiper output torque to pull the dart side at f_line. Default line_left = full reel (lowest hold tension, so
    the most torque for a given f_line)."""
    r, k, eta = _cap(p)
    L = _full(p) if line_left is None else line_left
    th = takeup_tension(p, L, clutch_r)
    return r * (max(f_line - th, 0.0) + takeup_drive_force(p, overspeed, clutch_r)) / (k * eta)


def line_from_wiper(t_wiper: float, p: Params, overspeed: float = 1.2, line_left: float = 0.0,
                    clutch_r: float | None = None) -> float:
    """Dart side tension for a wiper torque. Default line_left = 0 (empty reel, highest hold tension, so the most
    line tension for a given torque)."""
    r, k, eta = _cap(p)
    return (t_wiper * k * eta / r + takeup_tension(p, line_left, clutch_r)
            - takeup_drive_force(p, overspeed, clutch_r))


def drum_shaft_torque(t_wiper: float, p: Params) -> float:
    r, k, eta = _cap(p)
    return t_wiper * k * eta


def stall_torque_sag(p: Params, v_oc: float):
    w, b = p.wiper, p.battery
    i_s = v_oc / (w.r_winding + b.r_source)
    return w.kt * (i_s - w.i_free), i_s


def tow_point(f_line: float, p: Params, v_oc: float, overspeed: float = 1.2, line_left: float | None = None) -> dict:
    """Steady tow at dart side tension f_line, battery at v_oc with sag."""
    r, k, eta = _cap(p)
    t_w = wiper_torque_for(f_line, p, overspeed, line_left)
    op = p.wiper.operate(t_w, p.battery, v_oc)
    w_drum = op["speed"] / k
    return dict(f_line=f_line, t_wiper=t_w, t_drum=drum_shaft_torque(t_w, p), rpm_wiper=op["rpm"],
                rpm_drum=op["rpm"] / k, v_line=w_drum * r, current=op["current"], i_batt=op["i_batt"],
                v_batt=op["v_batt"], power_in=op["power_in"], stalled=op["stalled"])


def current_limit_for(f_limit: float, p: Params, overspeed: float = 1.2) -> dict:
    """Motor current at the tow limit. Taken at the empty reel (highest hold tension, so the least torque reaches
    f_limit): a trip at this current keeps the line under f_limit at any line left.
    BTS7960 IS pin: it senses the HIGH side only, so it reads zero in the PWM off-time. Sample during the on-time,
    or scale the averaged reading by 1/duty, to get motor current."""
    t_w = wiper_torque_for(f_limit, p, overspeed, line_left=0.0)
    t_w_full = wiper_torque_for(f_limit, p, overspeed)
    i = p.wiper.current_at(t_w)
    return dict(f_limit=f_limit, t_wiper=t_w, t_wiper_full_reel=t_w_full, current=i, i_free=p.wiper.i_free,
                delta_over_free=i - p.wiper.i_free)


def fw_trip(p: Params, tow_limit: float | None = None, overspeed: float = 1.2, gear: dict | None = None) -> dict:
    """Recommended firmware current trip and what the hard trip means mechanically."""
    tow_limit = p.capstan["tow_limit_n"] if tow_limit is None else tow_limit
    g = CAP_GEAR if gear is None else gear
    t_hard = p.wiper.kt * (TRIP_HARD_A - p.wiper.i_free)
    m = gears.mesh(t_hard, p.capstan["ratio"], g["module"], g["z_driver"], g["face"], PPA_CF, 0.0)
    cl = current_limit_for(tow_limit, p, overspeed)
    return dict(
        blank_ms=TRIP_BLANK_MS, hard_a=TRIP_HARD_A, hard_ms=TRIP_HARD_MS, hard_torque=t_hard,
        hard_gear_sf=m["sf"], hard_gear_pass=m["pass"], hard_drum_torque=drum_shaft_torque(t_hard, p),
        hard_line_n=line_from_wiper(t_hard, p, overspeed),
        tow_trip_delta_a=cl["delta_over_free"], tow_trip_avg_ms=TRIP_TOW_AVG_MS,
        tow_trip_rule=f"measured free current + {cl['delta_over_free']:.2f} A, averaged over {TRIP_TOW_AVG_MS} ms",
        notes=[
            f"ignore current for {TRIP_BLANK_MS} ms after start; hard trip at {TRIP_HARD_A} A for {TRIP_HARD_MS} ms in "
            f"aim, reel and tow ({t_hard:.2f} N m at the wiper), {TRIP_COCK_A} A while cocking "
            f"({p.wiper.kt * (TRIP_COCK_A - p.wiper.i_free):.2f} N m)",
            "sprag #2 (capstan backstop) plus the self-locking worm lock in the stall tension after a trip: add a "
            f"mechanical slip clutch of about {CLUTCH_REEL_NM:.0f} N m on the REEL output gear",
            "the 20 A fuse holds a 17 A stall indefinitely: it is wiring protection only",
            "BTS7960 IS reads the high side only (zero in the PWM off-time): sample in the on-time or scale by duty",
        ])


# ------------------------------------------------------------------ printed parts
def part_stresses(f_line: float, t_wiper: float, p: Params, wraps: int, rpm_drum: float, t_hold: float,
                  drum_wall_m: float = 0.010, line_pitch_m: float = 1.05e-3, p_allow: float = PLA.sigma_lewis,
                  bearing_split: float = 0.5, wrap_angle_deg: float = 180.0, key_len_mm: float = 14.0) -> dict:
    """drum_wall_m: drum shell between the line and the bore (40 mm drum on the 20 mm shaft: 10 mm). p_allow: line
    bearing and shell hoop allowable on printed PLA (sigma_lewis, conservative). bearing_split: share of the resultant
    on the most loaded 608 (0.5 = drum centred between two). wrap_angle_deg: angle between the dart line and the reel
    line leaving the drum (180 = parallel, worst case: resultant F + T_hold)."""
    r = p.capstan["d"] / 2
    t_drum = drum_shaft_torque(t_wiper, p)
    # line bearing pressure on the drum under the loaded wrap: T / (r d_line)
    p_line = f_line / (r * p.line.d)
    # external pressure from the loaded wrap, p = T / (r pitch); thick wall (Lame) hoop compression at the bore:
    # sigma = 2 p ro^2 / (ro^2 - ri^2)
    ro, ri = r, r - drum_wall_m
    p_ext = f_line / (r * line_pitch_m)
    hoop = 2 * p_ext * ro ** 2 / (ro ** 2 - ri ** 2)
    phi = math.radians(wrap_angle_deg)
    resultant = math.sqrt(f_line ** 2 + t_hold ** 2 - 2 * f_line * t_hold * math.cos(phi))
    return dict(
        f_line=f_line, t_drum=t_drum,
        shaft=gears.shaft_torsion(t_drum, p.shaft),
        key=gears.key(t_drum, key_len_mm, p.shaft, hub_r_mm=r * 1e3),
        sprag=gears.sprag(t_drum, rpm_drum, p.sprag),
        line_pressure_mpa=p_line / 1e6, p_ext_mpa=p_ext / 1e6, hoop_mpa=hoop / 1e6, p_allow_mpa=p_allow / 1e6,
        drum_pass=max(p_line, hoop) <= p_allow,
        radial_n=resultant, bearing608=gears.bearing608(resultant * bearing_split, rpm_drum, p.bearing),
        wraps=wraps,
    )


# ------------------------------------------------------------------ full run
def run(p: Params | None = None, tow: float | None = None, mu: float | None = None, light_n: float = 3.0,
        reel_len: float = 20.0, overspeed: float = 1.2, clutch_r: float | None = None) -> dict:
    p = p or load_params()
    c = p.capstan
    tow = c["tow_limit_n"] if tow is None else tow
    mu_lo, mu_hi = PLA.mu_dyneema
    mu = mu_lo if mu is None else mu
    wraps = c["wraps"]
    b = p.battery
    tu = takeup_range(p, overspeed, clutch_r)
    th_min, th_max = tu["t_hold_min"], tu["t_hold_max"]

    wn = wraps_needed(tow, th_min, mu)
    loads = [5, 10, 20, 30, 45, 50, 75, 100, 112]
    table = {}
    for name, v in (("full", b.v_full), ("nom", b.v_nom)):
        table[name] = [tow_point(f, p, v, overspeed) for f in loads]

    stall = {}
    for name, v in (("full", b.v_full), ("nom", b.v_nom)):
        t_s, i_s = stall_torque_sag(p, v)
        f_s = line_from_wiper(t_s, p, overspeed)      # empty reel: highest
        stall[name] = dict(v_oc=v, t_wiper=t_s, current=i_s, f_line=f_s,
                           exceeds_dart={k: f_s > hv for k, hv in DART_HOLD.items()},
                           exceeds_break=f_s > p.line.breaking, line_breaking=p.line.breaking,
                           wraps_slip_first=slips(f_s, wraps, th_max, mu_hi),
                           holds_max=max_hold(wraps, th_max, mu_hi))

    light = tow_point(light_n, p, b.v_nom, overspeed)
    t_reel = reel_len / light["v_line"] if light["v_line"] > 0 else math.inf
    at_tow = tow_point(tow, p, b.v_nom, overspeed)
    s_full = stall["full"]

    res = dict(
        inputs=dict(tow=tow, mu=mu, mu_range=(mu_lo, mu_hi), takeup_n=c["takeup_n"], wraps=wraps, drum_d=c["d"],
                    ratio=c["ratio"], eta=c["eta"], overspeed=overspeed, light_n=light_n, source=p.source),
        takeup=tu,
        wraps_needed=wn,
        capstan_ratio=ratio(mu, wraps),
        max_hold_n=dict(mu_low_worst=max_hold(wraps, th_min, mu_lo), mu_low_best=max_hold(wraps, th_max, mu_lo),
                        mu_high=max_hold(wraps, th_max, mu_hi)),
        takeup_needed_n=takeup_needed(tow, wraps, mu),
        slips_at_tow=slips(tow, wraps, th_min, mu),
        tow_table=table,
        at_tow_nom=at_tow,
        current_limit=current_limit_for(tow, p, overspeed),
        current_limit_dart=dict({k: current_limit_for(v, p, overspeed)["current"] for k, v in DART_HOLD.items()}),
        fw_trip=fw_trip(p, tow, overspeed),
        stall=stall,
        reel=dict(length=reel_len, light=light, time_s=t_reel),
        parts_at_tow=part_stresses(tow, at_tow["t_wiper"], p, wraps, at_tow["rpm_drum"], th_min),
        parts_at_stall=part_stresses(s_full["f_line"], s_full["t_wiper"], p, wraps, 0.0, th_max),
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
        w.append("line slips on the drum at the tow limit (mu low, full reel)")
    if r["inputs"]["wraps"] < r["wraps_needed"]["design"]:
        w.append(f"wraps {r['inputs']['wraps']} < design {r['wraps_needed']['design']} (1.5x margin, full reel)")
    cl = r["current_limit"]
    if cl["delta_over_free"] < 2.0:
        w.append(f"tow limit is only {cl['delta_over_free']:.2f} A above free current: current sensing of tension is "
                 "coarse, calibrate on the bench or use the load cell")
    ft = r["fw_trip"]
    if not ft["hard_gear_pass"]:
        w.append(f"capstan gear fails at the {ft['hard_a']} A hard trip (sf {ft['hard_gear_sf']:.2f})")
    over = [k for k, hv in DART_HOLD.items() if ft["hard_line_n"] > hv]
    if over:
        w.append(f"{ft['hard_a']} A hard trip still allows {ft['hard_line_n']:.0f} N on the line (above {', '.join(over)}):"
                 " the tow trip must act first")
    for tag in ("parts_at_tow", "parts_at_stall"):
        ps = r[tag]
        for k in ("shaft", "key", "sprag", "bearing608"):
            if not ps[k]["pass"]:
                w.append(f"{tag}: {k} fails")
        if ps["shaft"]["pass"] and not ps["shaft"]["strict_pass"]:
            w.append(f"{tag}: shaft {ps['t_drum']:.1f} N m over the Kt 2 limit {ps['shaft']['t_allow_kt']:.1f} N m")
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
    tu = r["takeup"]
    print(f"take-up clutch {tu['clutch_torque']*1000:.1f} N mm: hold {tu['t_hold_min']:.2f} to {tu['t_hold_max']:.2f} N, "
          f"overspeed {tu['overspeed_min']:.2f} to {tu['overspeed_max']:.2f} over 0 to {tu['l_max']:.0f} m on the reel")
    mh = r["max_hold_n"]
    print(f"max hold with {i['wraps']} wraps: {mh['mu_low_worst']:.0f} N (mu low, full reel), {mh['mu_low_best']:.0f} N "
          f"(mu low, empty), {mh['mu_high']:.0f} N (mu high); take-up needed for tow: {r['takeup_needed_n']:.2f} N")
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
    ft = r["fw_trip"]
    print(f"\nFIRMWARE TRIP: {ft['notes'][0]}; capstan gear sf at that torque {ft['hard_gear_sf']:.2f}, "
          f"line {ft['hard_line_n']:.0f} N; tow trip = {ft['tow_trip_rule']}")
    for n in ft["notes"][1:]:
        print(" note:", n)
    print("\nWARNINGS:")
    for w in r["warnings"]:
        print(" -", w)
    print("wrote", write_json(r, "capstan.json"))
    return r


if __name__ == "__main__":
    main()
