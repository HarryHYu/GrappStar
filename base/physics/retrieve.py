"""Coordinator checks (3 Nov): line slack at the shot, wrap loads on the cup and drum N20, final approach onto the cradle.

1. Snatch: with the take-up bus the line is taut at the shot (the reel spins up with the dart during the stroke).
   Without it, 245 mm of slack lies loose and the line snatches tight just after the muzzle: the dart then shares its
   momentum with the reel (plastic limit) and the line sees an elastic spike.
2. Wrap: the wrap cup ring orbits the capstan and draws line off the storage reel against the LIGHT clutch (tension at
   the reel exit = clutch torque / line radius), through the guide eyes. Cup torque -> drum torque -> drum N20 torque.
3. Final approach: reeling the dart up the rail into the cradle at 5 to 10 deg pitch. Available pull from the FIRM
   clutch (take-up only, capstan unwrapped) vs gravity + friction + the seated contact.

python retrieve.py
"""
from __future__ import annotations

import json
import math
import os

from common import G, load_params
from launcher import launch, reel_line_load
from payout import line_radius, reel_props

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "out", "physics")

WRAP = dict(                # cad-drum / cad-reel numbers from the coordinator message; ASSUMED where marked
    cup_turns=7.0,          # wraps laid
    drum_sector_deg=240.0,  # drum rotation that lays them
    cup_per_drum=10.5,      # cup turns per drum turn
    ring_r=0.030,           # m wrap ring radius about the capstan axis (ASSUMED: capstan r 20 + line + clearance)
    guide_wrap_rad=math.pi, # total line wrap over guide eyes between reel and ring (ASSUMED 180 deg)
    mu_guide=0.25,          # Dyneema on PLA, high end
    eta_wrap=0.7,           # segment gear + cup bearing (printed)
    n20_ratio=9.0,          # N20 10 T -> drum 90 T (drum_NOTES)
    eta_n20_mesh=0.9,
    n20_torque_nm=0.20,     # N20 worm output torque you can count on at 9 V (ASSUMED: 1 to 4 kg cm class; MEASURE)
    n20_rpm=27.0,           # at 12 V (spec N20_WORM); about 20 at 9 V
)
APPROACH = dict(
    pitch_deg=(5.0, 10.0),
    mu_cradle=(0.30, 0.40), # dart belly on the PLA cradle
    seat_force=1.0,         # N to close SW_SEATED (printed flex tongue; ASSUMED)
    eye_gain=1.3,           # exit eye + guides between the reel and the dart (line pulled in through them)
)


def snatch(p, pitch=36.0, slack=0.245, seg_len=0.5):
    f_line, m_eff = reel_line_load(p)
    taut = launch(p, pitch)                                   # reel spun up in the stroke (current model)
    loose = launch(p, pitch, line_load=(0.0, 0.0))            # no line load in the stroke
    v0 = loose["v_muzzle"]
    md = p.dart.mass
    v1 = v0 * md / (md + m_eff)                               # momentum shared with the reel at the snatch
    mu = md * m_eff / (md + m_eff)
    k = p.line.ea / (seg_len + slack)                         # line out at the snatch
    t_peak = v0 * math.sqrt(k * mu)                           # undamped elastic spike (upper bound)
    return dict(slack_m=slack, reel_mass_eff_kg=m_eff, v_taut=taut["v_muzzle"], v_no_line_in_stroke=v0,
                v_after_snatch=v1, speed_loss_vs_taut=1 - v1 / taut["v_muzzle"],
                ke_loss_snatch_j=0.5 * md * v0 ** 2 - 0.5 * (md + m_eff) * v1 ** 2,
                line_spike_n_upper=t_peak, line_break_n=p.line.breaking)


def wrap_loads(p, w=WRAP, light_n=None):
    light = p.flight["brake_n"] if light_n is None else light_n
    rp = reel_props(p)
    torque = light * rp["r_full"]                             # light clutch set at the full reel
    rows = []
    for left in (p.reel["line_stored"], 5.0, 0.5):            # line on the reel when wrapping (dart out)
        t_reel = torque / line_radius(p, left)
        t_ring = t_reel * math.exp(w["mu_guide"] * w["guide_wrap_rad"])
        t_cup = t_ring * w["ring_r"]
        t_drum = t_cup * w["cup_per_drum"] / w["eta_wrap"]
        t_n20 = t_drum / (w["n20_ratio"] * w["eta_n20_mesh"])
        rows.append(dict(line_on_reel_m=left, t_reel_n=t_reel, t_ring_n=t_ring, t_cup_nm=t_cup, t_drum_nm=t_drum,
                         t_n20_nm=t_n20, n20_frac=t_n20 / w["n20_torque_nm"]))
    drum_turns = w["drum_sector_deg"] / 360
    t_wrap_s = drum_turns * w["n20_ratio"] / (20.0 / 60)      # N20 about 20 rpm at 9 V
    cup_turns_check = drum_turns * w["cup_per_drum"]
    line_drawn = w["cup_turns"] * 2 * math.pi * (p.capstan["d"] / 2 + p.line.d / 2)
    return dict(params=w, light_n=light, rows=rows, wrap_time_s=t_wrap_s, cup_turns_in_sector=cup_turns_check,
                line_drawn_m=line_drawn)


def approach(p, a=APPROACH):
    rp = reel_props(p)
    torque = p.capstan["takeup_n"] * rp["r_full"]             # FIRM clutch: line tension with the reel full
    r_end = rp["r_full"]                                       # reel nearly full at the end of retrieve
    avail_reel = torque / r_end
    avail_dart = avail_reel / a["eye_gain"]
    rows = []
    for th in a["pitch_deg"]:
        for mu in a["mu_cradle"]:
            t = math.radians(th)
            need = p.dart.mass * G * (math.sin(t) + mu * math.cos(t)) + a["seat_force"]
            rows.append(dict(pitch_deg=th, mu=mu, need_n=need, avail_n=avail_dart, margin=avail_dart / need))
    t_for_margin = max(r["need_n"] for r in rows) * 1.5 * a["eye_gain"]
    return dict(params=a, clutch_torque_nm=torque, avail_at_reel_n=avail_reel, avail_at_dart_n=avail_dart, rows=rows,
                firm_setting_for_1p5_margin_n_full_reel=t_for_margin)


def main(argv=None):
    p = load_params()
    out = dict(params_source=p.source, snatch=snatch(p), wrap=wrap_loads(p), approach=approach(p))
    s = out["snatch"]
    print(f"params: {p.source}")
    print(f"1. snatch: taut line {s['v_taut']:.2f} m/s; with {s['slack_m'] * 1000:.0f} mm slack: {s['v_no_line_in_stroke']:.2f} m/s "
          f"at the muzzle, {s['v_after_snatch']:.2f} after the snatch ({s['speed_loss_vs_taut'] * 100:.1f} % slower than taut, "
          f"{s['ke_loss_snatch_j']:.2f} J lost), line spike up to {s['line_spike_n_upper']:.0f} N (undamped bound)")
    w = out["wrap"]
    print(f"2. wrap ({w['cup_turns_in_sector']:.1f} cup turns in {w['params']['drum_sector_deg']:.0f} deg of drum, light clutch {w['light_n']} N, "
          f"{w['line_drawn_m']:.2f} m drawn, ~{w['wrap_time_s']:.0f} s):")
    for r in w["rows"]:
        print(f"   {r['line_on_reel_m']:4.1f} m on reel: ring pull {r['t_ring_n']:.2f} N, cup {r['t_cup_nm'] * 1000:.0f} N mm, "
              f"drum {r['t_drum_nm']:.3f} N m, N20 {r['t_n20_nm'] * 1000:.0f} N mm ({r['n20_frac'] * 100:.0f} % of {w['params']['n20_torque_nm']} N m)")
    a = out["approach"]
    print(f"3. final approach on the FIRM clutch ({p.capstan['takeup_n']} N with the reel full): {a['avail_at_dart_n']:.2f} N at the dart")
    for r in a["rows"]:
        print(f"   pitch {r['pitch_deg']:.0f}, mu {r['mu']}: need {r['need_n']:.2f} N, margin {r['margin']:.2f}")
    print(f"   FIRM setting for a 1.5 margin: {a['firm_setting_for_1p5_margin_n_full_reel']:.1f} N with the reel full")
    os.makedirs(OUT_DIR, exist_ok=True)
    json.dump(out, open(os.path.join(OUT_DIR, "retrieve.json"), "w"), indent=1)
    return out


if __name__ == "__main__":
    main()
