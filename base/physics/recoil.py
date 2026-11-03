"""Recoil and tip-over on the rover deck at the shot, and the 2-spring vs 4-spring comparison.

Forces on the base (frame + rail) during a shot:
  stroke  the springs pull the carriage forward and the anchors backward: the frame feels the rate of change of the
          moving parts' momentum, backward along the rail (peak = carriage force at full draw, minus nothing: friction
          is internal). It acts along the spring line, `spring_w` above the rail pivot.
  stop    the carriage (and the spring surge through the cord) hits the compliant stop: a short forward pull.
Moments are about the deck point under the rail pivot, then about the rover's rear and front wheel lines.

python recoil.py [--pitch 36]
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os

from common import G, load_params
from flight import simulate
from launch import launch

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "out", "physics")

ROVER = dict(               # ASSUMED: Harry to confirm the rover (deck size, wheelbase, mass)
    mass=4.0,               # kg rover + base + battery
    pivot_h=0.30,           # m rail pivot above the ground (deck ~0.15 + main axis ~0.15 above the deck)
    spring_w=0.060,         # m spring line above the pivot, perpendicular to the rail (cad-launch NOTES w 60)
    rear=0.15,              # m from the pivot back to the rear wheel line
    front=0.15,             # m from the pivot forward to the front wheel line
    cg_x=0.0,               # m rover CG ahead of the pivot
    cg_h=0.12,              # m rover CG height above the ground
)


def tip_dynamic(rover, net_impulse_nms, lever):
    """Short pulses: the static moment comparison is far too pessimistic. Angular impulse about the wheel line ->
    rotational KE -> how high the CG rises vs the energy to roll over the wheel line."""
    m, h = rover["mass"], rover["cg_h"]
    I = m * (lever ** 2 + h ** 2)
    if net_impulse_nms <= 0:
        return dict(omega=0.0, ke_j=0.0, tip_energy_j=m * G * (math.hypot(lever, h) - h), tips=False, lift_deg=0.0)
    w = net_impulse_nms / I
    ke = 0.5 * I * w * w
    e_tip = m * G * (math.hypot(lever, h) - h)
    # lift angle: CG rises by ke / (m g) (small angles)
    r = math.hypot(lever, h)
    a0 = math.atan2(h, lever)
    lift = 0.0
    if ke < e_tip:
        target = min(1.0, (h + ke / (m * G)) / r)
        lift = math.degrees(math.asin(target) - a0)
    return dict(omega=w, ke_j=ke, tip_energy_j=e_tip, tips=ke >= e_tip, lift_deg=lift)


def shot_loads(p, pitch: float, rover=ROVER):
    r = launch(p, pitch, record=True)
    th = math.radians(pitch)
    F = r["f_carriage_full_n"]                 # backward along the rail at the spring line
    # line of action: through the spring line point (u, w = spring_w) with direction (-cos, -sin) in world x, z.
    # moment about the deck point under the pivot (world origin at ground level, pivot at height pivot_h):
    # point on the line: x0 = -spring_w sin th, z0 = pivot_h + spring_w cos th; force (Fx, Fz) = F (-cos, -sin)
    x0, z0 = -rover["spring_w"] * math.sin(th), rover["pivot_h"] + rover["spring_w"] * math.cos(th)
    Fx, Fz = -F * math.cos(th), -F * math.sin(th)

    def moment_about(xp):
        # 2D r x F about the ground point (xp, 0), x forward, z up: + = nose up (front wheels lift)
        return (x0 - xp) * Fz - z0 * Fx

    # recoil pushes backward and down: tips the rover backward about the rear wheels (front wheels lift)
    m_rear = moment_about(-rover["rear"])
    w = rover["mass"] * G
    resist_rear = w * (rover["rear"] + rover["cg_x"])
    # stop: forward pull, peak with a linear pad of the given travel, along the same line
    ke = r["ke_carriage_j"] + r["ke_spring_j"]
    travel = p.launch["stop_pad_travel"]
    f_stop_lin = 2 * ke / travel
    f_stop_const = ke / travel
    m_front = moment_about(rover["front"]) * f_stop_lin / F   # stop force is the reverse: nose down about the front wheels
    resist_front = w * (rover["front"] - rover["cg_x"])
    impulse = p.dart.mass * r["v_muzzle"]
    f_mean = 0.5 * (F + p.n_springs * p.spring.force(p.launch["preload_ext"]) / p.launch["pulley_ratio"])
    t_s = r["t_stroke_ms"] / 1000
    back = tip_dynamic(rover, (m_rear * f_mean / F - resist_rear) * t_s, rover["rear"])
    # stop: momentum of the carriage + spring surge stopped over 2 x travel / v; moment arm as the static case
    t_stop = 2 * travel / r["v_muzzle"]
    front = tip_dynamic(rover, (m_front * 0.5 - resist_front) * t_stop, rover["front"])
    return dict(pitch_deg=pitch, v_muzzle=r["v_muzzle"], f_recoil_peak_n=F, f_recoil_x_n=-Fx, f_recoil_down_n=-Fz,
                t_stroke_ms=r["t_stroke_ms"], impulse_dart_ns=impulse,
                moment_back_tip_nm=m_rear, resist_back_tip_nm=resist_rear, back_tip_margin=resist_rear / m_rear if m_rear > 0 else math.inf,
                stop_ke_j=ke, f_stop_linear_pad_n=f_stop_lin, f_stop_constant_force_n=f_stop_const,
                travel_for_300n_linear_mm=2 * ke / 300 * 1000, moment_front_tip_nm=m_front, resist_front_tip_nm=resist_front,
                front_tip_margin=resist_front / m_front if m_front > 0 else math.inf, rover=rover,
                back_dynamic=back, front_dynamic=front)


def spring_count_case(p, n: int, pitch: float):
    q = copy.deepcopy(p)
    q.n_springs = n
    r = launch(q, pitch)
    rr = [simulate(q, r["v_muzzle"], pitch, brake_n=b, reel_model=True)["range_m"] for b in (0.5, 0.75, 1.0)]
    return dict(n_springs=n, energy_j=r["energy_stored_j"], v_muzzle=r["v_muzzle"], efficiency=r["efficiency"],
                f_carriage_full_n=r["f_carriage_full_n"], range_m=rr, ke_stop_j=r["ke_carriage_j"] + r["ke_spring_j"],
                a_peak_g=r["a_peak_g"])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--pitch", type=float, default=36.0)
    a = ap.parse_args(argv)
    p = load_params()
    out = dict(params_source=p.source)
    rows = []
    for th in (6.0, 20.0, 36.0, 47.0):
        s = shot_loads(p, th)
        rows.append(s)
    out["recoil"] = rows
    print(f"params: {p.source}")
    print("pitch  F peak  back-tip moment / resisting (margin)   stop KE  pad peak lin/const  front-tip margin")
    for s in rows:
        print(f"{s['pitch_deg']:5.0f} {s['f_recoil_peak_n']:6.1f} N  {s['moment_back_tip_nm']:5.2f} / {s['resist_back_tip_nm']:5.2f} N m "
              f"({s['back_tip_margin']:.1f})   {s['stop_ke_j']:.2f} J  {s['f_stop_linear_pad_n']:.0f} / {s['f_stop_constant_force_n']:.0f} N  "
              f"{s['front_tip_margin']:.2f}")
    for s in rows:
        b, f_ = s["back_dynamic"], s["front_dynamic"]
        print(f"  pitch {s['pitch_deg']:.0f}: pulses (dynamic): back rock {b['lift_deg']:.1f} deg (tips {b['tips']}), "
              f"front rock {f_['lift_deg']:.1f} deg (tips {f_['tips']})")
    s = rows[2]
    print(f"stop travel for a 300 N peak with a linear pad: {s['travel_for_300n_linear_mm']:.0f} mm; dart impulse {s['impulse_dart_ns']:.2f} N s")
    out["springs"] = [spring_count_case(p, n, a.pitch) for n in (2, 4)]
    for c in out["springs"]:
        print(f"{c['n_springs']} springs: {c['energy_j']:.1f} J, v {c['v_muzzle']:.2f} m/s, eff {c['efficiency']:.2f}, "
              f"carriage force {c['f_carriage_full_n']:.0f} N, {c['a_peak_g']:.0f} g, stop KE {c['ke_stop_j']:.2f} J, "
              f"range 0.5/0.75/1.0 N: " + " / ".join(f"{x:.2f}" for x in c["range_m"]))
    os.makedirs(OUT_DIR, exist_ok=True)
    json.dump(out, open(os.path.join(OUT_DIR, "recoil.json"), "w"), indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    return out


if __name__ == "__main__":
    main()
