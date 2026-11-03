"""Storage reel payout: reel on 608s, printed slip-clutch brake, optional centrifugal shoes.

Reel geometry -> inertia and line radius vs line left. The flight itself (with the reel coupled to a bead-chain line)
runs in flight.py; this module adds the reel-only numbers: spin-up during the launch stroke, rpm, overrun after
landing, slack, bird's nest risk, and a brake / shoe comparison.

python payout.py [--brake 0.75] [--pitch 40] [--shoes]
"""
from __future__ import annotations

import argparse
import json
import math
import os

from common import Params, load_params

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "out", "physics")


def line_radius(p: Params, line_left: float) -> float:
    """Outer radius of the wound line with `line_left` metres on the reel (level wind, square packing at line_pitch)."""
    R = p.reel
    rc = R["core_d"] / 2
    return math.sqrt(rc ** 2 + max(line_left, 0.0) * R["line_pitch"] ** 2 / (math.pi * R["width"]))


def reel_props(p: Params, line_left: float | None = None) -> dict:
    """Mass and inertia of the spinning reel (printed core + flanges + parts that spin with it + the wound line)."""
    R = p.reel
    rho = 1240.0                      # PLA
    rc, rf = R["core_d"] / 2, R["flange_d"] / 2
    w, tw, tf = R["width"], R["wall"], R["flange_t"]
    ri = rc - tw
    # thin printed walls come out solid; infill only matters for thick sections (none here)
    m_core = rho * math.pi * (rc ** 2 - ri ** 2) * w
    I_core = 0.5 * m_core * (rc ** 2 + ri ** 2)
    hub_r = 0.011                     # 608 seat: outer race + clutch face at about r 11 mm
    m_fl = rho * math.pi * (rf ** 2 - hub_r ** 2) * tf
    I_fl = 0.5 * m_fl * (rf ** 2 + hub_r ** 2)
    # hub web joining the core to the 608 seats (two discs, 3 mm, from r 11 to the core)
    m_web = 2 * rho * math.pi * (ri ** 2 - hub_r ** 2) * 0.003
    I_web = 0.5 * m_web * (ri ** 2 + hub_r ** 2)
    m_hub = R["hub_mass"]
    I_hub = m_hub * hub_r ** 2
    I_empty = I_core + 2 * I_fl + I_web + I_hub
    m_empty = m_core + 2 * m_fl + m_web + m_hub
    L = R["line_stored"] if line_left is None else line_left
    r = line_radius(p, L)
    m_line = p.line.lam * L
    I_line = 0.5 * m_line * (rc ** 2 + r ** 2)
    cap = math.pi * w * (rf ** 2 - rc ** 2) / R["line_pitch"] ** 2
    r_full = line_radius(p, R["line_stored"])
    return dict(mass_empty=m_empty, inertia_empty=I_empty, inertia=I_empty + I_line, inertia_full=I_empty + 0.5 * p.line.lam * R["line_stored"] * (rc ** 2 + r_full ** 2),
                r=r, r_full=r_full, r_core=rc, capacity_m=cap, line_left=L, flange_margin_mm=(rf - r_full) * 1000)


class Reel:
    """Reel state for the coupled flight sim. Brake: constant slip torque set so that the line tension at the reel
    exit is brake_n at the FULL radius (the tension then rises as the reel empties: T = torque / r).
    Centrifugal shoes (baitcaster): torque = shoe_k * omega^2 once omega > shoe_on (rad/s)."""

    def __init__(self, p: Params, brake_n: float, shoes: bool = False, shoe_k: float = 0.0, shoe_on: float = 0.0):
        self.p = p
        rp = reel_props(p)
        self.rc = rp["r_core"]
        self.I_empty = rp["inertia_empty"]
        self.r_full = rp["r_full"]
        self.L_stored = p.reel["line_stored"]
        self.brake_torque = brake_n * self.r_full
        self.bearing = 2 * p.bearing.drag_torque
        self.shoes = shoes
        self.shoe_k = shoe_k
        self.shoe_on = shoe_on
        self.omega = 0.0
        self.paid = 0.0

    def radius(self):
        return line_radius(self.p, self.L_stored - self.paid)

    def inertia(self):
        L = self.L_stored - self.paid
        r = self.radius()
        return self.I_empty + 0.5 * self.p.line.lam * L * (self.rc ** 2 + r ** 2)

    def resist_torque(self):
        t = self.brake_torque + self.bearing
        if self.shoes and self.omega > self.shoe_on:
            t += self.shoe_k * self.omega ** 2
        return t

    def step(self, t_line: float, dt: float):
        """Advance with line tension t_line at the reel (after the guide eye). Returns line paid this step (m)."""
        if self.paid >= self.L_stored:
            self.omega = 0.0
            return 0.0
        r = self.radius()
        drive = t_line * r
        res = self.resist_torque()
        if self.omega <= 0.0 and drive <= res:
            self.omega = 0.0
            return 0.0
        a = (drive - res) / self.inertia()
        self.omega = max(0.0, self.omega + a * dt)
        dl = self.omega * r * dt
        self.paid += dl
        return dl


def spin_up_in_stroke(p: Params, v_muzzle: float, stroke: float, brake_n: float) -> dict:
    """Line tension needed to spin the reel up with the dart during the launch stroke (uniform acceleration estimate)."""
    rp = reel_props(p)
    a = v_muzzle ** 2 / (2 * stroke)
    m_eff = rp["inertia_full"] / rp["r_full"] ** 2
    t_acc = m_eff * a
    t_tot = (t_acc + brake_n) * p.flight["eye_gain"]
    rpm = v_muzzle / rp["r_full"] * 60 / (2 * math.pi)
    return dict(accel=a, reel_mass_eff_g=m_eff * 1000, t_spinup_n=t_acc, t_line_in_stroke_n=t_tot, rpm_at_muzzle=rpm)


def shoe_k_for(p: Params, brake_extra_n: float, at_speed: float) -> float:
    """Shoe coefficient that adds `brake_extra_n` of line tension at line speed `at_speed` (m/s), full reel."""
    rp = reel_props(p)
    w = at_speed / rp["r_full"]
    return brake_extra_n * rp["r_full"] / w ** 2


def analyse(p: Params, brake_n: float = 0.75, pitch_deg: float = 40.0, v_muzzle: float | None = None, shoes: bool = False,
            shoe_extra_n: float = 0.5):
    from flight import simulate
    from launch import launch as do_launch
    if v_muzzle is None:
        v_muzzle = do_launch(p, pitch_deg)["v_muzzle"]
    kw = {}
    if shoes:
        kw = dict(shoes=True, shoe_k=shoe_k_for(p, shoe_extra_n, v_muzzle), shoe_on=0.0)
    fr = simulate(p, v_muzzle, pitch_deg, brake_n=brake_n, reel_model=True, record=True, **kw)
    su = spin_up_in_stroke(p, v_muzzle, p.launch["stroke"], brake_n)
    rp = reel_props(p)
    risk = "low"
    if fr["overrun_after_landing_m"] > 1.0 or fr["slack_max_m"] > 0.5:
        risk = "high"
    elif fr["overrun_after_landing_m"] > 0.3 or fr["slack_max_m"] > 0.15:
        risk = "medium"
    return dict(brake_n=brake_n, pitch_deg=pitch_deg, v_muzzle=v_muzzle, shoes=shoes, reel=rp, spin_up=su,
                rpm_max=fr["reel_rpm_max"], bearing_ok=bool(fr["reel_rpm_max"] < p.bearing.n_max_rpm_2rs),
                t_line_max_n=fr["t_dart_max_n"], t_reel_max_n=fr["t_reel_max_n"],
                slack_in_flight_m=fr["slack_max_m"], overrun_after_landing_m=fr["overrun_after_landing_m"],
                stop_time_after_landing_s=fr["reel_stop_after_landing_s"], range_m=fr["range_m"],
                birds_nest_risk=risk, history=fr.get("history_reel", []))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--brake", type=float, default=0.75)
    ap.add_argument("--pitch", type=float, default=40.0)
    ap.add_argument("--v", type=float, default=None, help="muzzle speed (default: from launch.py)")
    ap.add_argument("--shoes", action="store_true")
    ap.add_argument("--json", default=os.path.join(OUT_DIR, "payout.json"))
    a = ap.parse_args(argv)
    p = load_params()
    rp = reel_props(p)
    print(f"params: {p.source}")
    print(f"reel: empty {rp['mass_empty'] * 1000:.1f} g, I empty {rp['inertia_empty'] * 1e6:.2f} kg mm2, "
          f"full {rp['inertia_full'] * 1e6:.2f} kg mm2, r full {rp['r_full'] * 1000:.1f} mm, "
          f"capacity {rp['capacity_m']:.1f} m, flange margin {rp['flange_margin_mm']:.1f} mm")
    rows = []
    for shoes in (False, True):
        res = analyse(p, a.brake, a.pitch, a.v, shoes=shoes)
        rows.append(res)
        print(f"shoes={shoes}: v0 {res['v_muzzle']:.2f} m/s, range {res['range_m']:.2f} m, rpm max {res['rpm_max']:.0f}, "
              f"line at dart max {res['t_line_max_n']:.2f} N, spin-up in stroke {res['spin_up']['t_line_in_stroke_n']:.2f} N, "
              f"slack in flight {res['slack_in_flight_m']:.2f} m, overrun after landing {res['overrun_after_landing_m']:.2f} m "
              f"in {res['stop_time_after_landing_s']:.2f} s, bird's nest risk {res['birds_nest_risk']}")
    out = dict(params_source=p.source, reel=rp, runs=[{k: v for k, v in r.items() if k != "history"} for r in rows],
               history=rows[0]["history"])
    os.makedirs(os.path.dirname(a.json), exist_ok=True)
    json.dump(out, open(a.json, "w"), indent=1, default=float)
    return out


if __name__ == "__main__":
    main()
