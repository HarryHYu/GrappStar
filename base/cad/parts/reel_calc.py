"""GrappStar base, reel system: design numbers and kinematics (no CAD, numpy only).

Owner: worker cad-reel. Everything the reel CAD, the firmware and the physics worker need from the reel lives here:
capstan wraps (Euler-Eytelwein), storage reel pack, spin-up and overrun at launch, slip clutch preload, gear train
speeds and torques, wrap and unwrap schedules, cup holding torque from Harry's v7 line geometry.

Units: mm, N, N m, s, rad unless a name says otherwise. Run `python3 reel_calc.py` for the summary table.
Shared numbers come from params (P.REEL, P.LINE, P.WIPER, ...) when the spec defines them; DEFAULTS below are the
cad-reel recommendations written into reel_NOTES.md.
"""
from __future__ import annotations

import math

DEFAULTS = dict(
    # line
    line_d=1.0, line_len_m=20.0, line_lam_g_m=0.60, line_break_n=667.0,
    # capstan (Harry v7: dia 40 x 40 body, base flange cone 23 -> 20 over 3)
    cap_r=20.0, cap_len=40.0, cap_flange_r=23.0, cap_flange_h=3.0,
    wraps=7, mu_min=0.10, mu_lo=0.15, mu_hi=0.25,
    # D49 spool on the rail head: core d40, flange d62, pack 20 wide (params REEL.storage)
    core_r=20.0, flange_r=31.0, width=20.0, flange_t=2.0, pack_fill=0.80,
    # clutch line tensions at the design pack radius r 27 (D49): LIGHT 0.25 N fixed, FIRM 5.0 N
    clutch_r=27.0, brake_launch_n=0.25, brake_mid_n=0.25, brake_reel_n=5.0, mu_face=0.30, face_r_in=18.0,
    face_r_out=26.0, brake_stroke=4.0,
    # clutch lever (pivot parallel to the rail): upper arm to the cam follower, lower arm = PPA-CF leaf to the FIRM
    # screw on the thrust ring; leaf section b x t (b = 8 along the rail)
    lever_a=53.0, lever_l=56.0, leaf_b=8.0, leaf_t=4.4, leaf_e=5000.0, cam_gap=0.3,
    # gears (D49): REEL 50T m2 -> C1 49T -> B1 24:12 -> G_a 26T / G_b 32T / G_c 23T m1.5: capstan 2.307 x wiper;
    # take-up bus 50T -> c1' 46T -> swing idler 30T -> T1 20T (2.5 x wiper) -> B2, B3 1:1; cup gear 28T from W 49T
    m_cap=2.0, z_main=50, z_c1=49, b1_ratio=2.0, z_ga=26, z_gb=32, z_gc=23, m_take=1.5, z_bus=50, z_c1p=46,
    z_idler=30, z_reel=20, m_cup=1.5, z_cup=28, z_wrap_idler=49, wrap_idler_turns=4.0,
    # wiper (linear model, physics worker block); firmware tow limit
    wiper_n0_rpm=45.0, wiper_stall_nm=15.0, wiper_i_stall=18.0, wiper_i_free=1.5, eta_train=0.84,
    tow_limit_n=100.0,
    # launch (physics defaults until base/physics/FINDINGS.md): muzzle speed, stroke, flight decel
    v_launch=7.1, stroke_m=0.241, flight_decel=5.0,
    # v7 line geometry relative to the capstan: D49 axial eyelet at r 26 (the dart line enters along the cup axis)
    ring_r=26.0, eye_dx=0.0, eye_dz=1.0, groove_wrap_deg=57.9, groove_r=23.5,
)


def cap_ratio(d):
    """Capstan turns per main shaft turn (D49 train): 50/49 x 2 x 26/23 = 2.307."""
    return d["z_main"] / d["z_c1"] * d["b1_ratio"] * d["z_ga"] / d["z_gc"]


def get(P=None):
    """Design numbers: DEFAULTS overlaid with P.REEL (dict) when the spec defines it."""
    d = dict(DEFAULTS)
    reel = getattr(P, "REEL", None) if P is not None else None
    if isinstance(reel, dict):
        for k, v in reel.items():
            if k in d and v is not None:
                d[k] = v
    line = getattr(P, "LINE", None) if P is not None else None
    if isinstance(line, dict):
        d["line_d"] = line.get("d", d["line_d"])
        d["line_len_m"] = line.get("length_m", d["line_len_m"])
        d["line_break_n"] = line.get("break_n", d["line_break_n"])
    w = getattr(P, "WIPER", None) if P is not None else None
    if isinstance(w, dict):
        d["wiper_n0_rpm"] = w.get("n0_rpm", d["wiper_n0_rpm"])
        d["wiper_stall_nm"] = w.get("stall_nm", d["wiper_stall_nm"])
        d["wiper_i_stall"] = w.get("stall_a", d["wiper_i_stall"])
        d["wiper_i_free"] = w.get("free_a", d["wiper_i_free"])
    return d


# ------------------------------------------------------------------------------------------------ capstan
def capstan_ratio(mu, wraps, extra_deg=0.0):
    """Euler-Eytelwein: T_load / T_hold for `wraps` full turns plus extra wrap in degrees."""
    return math.exp(mu * (2 * math.pi * wraps + math.radians(extra_deg)))


def wraps_needed(t_load, t_tail, mu, margin=2.0):
    """Smallest whole number of wraps that holds margin * t_load with t_tail behind."""
    n = math.log(margin * t_load / t_tail) / (mu * 2 * math.pi)
    return max(1, math.ceil(n - 1e-9))


def line_r_cap(d):
    return d["cap_r"] + d["line_d"] / 2


def wrap_line_mm(d, wraps=None):
    """Line laid on the capstan by `wraps` turns."""
    return (wraps if wraps is not None else d["wraps"]) * 2 * math.pi * line_r_cap(d)


# ------------------------------------------------------------------------------------------------ storage reel
def pack_r(d, stored_m):
    """Outer pack radius with stored_m metres of line on the reel (layer fill factor pack_fill)."""
    vol = stored_m * 1000.0 * d["line_d"] ** 2 / d["pack_fill"]
    return math.sqrt(d["core_r"] ** 2 + vol / (math.pi * d["width"]))


def reel_inertia(d, stored_m=None, structure_kgm2=9.0e-6):
    """Spinning inertia (kg m2): printed reel structure (from the CAD, default estimate) + clutch-side nothing
    (the clutch disk does not spin in payout) + 608 outer races + the stored line as a thick ring."""
    stored = d["line_len_m"] if stored_m is None else stored_m
    r0, r1 = d["core_r"] / 1000, pack_r(d, stored) / 1000
    m_line = stored * d["line_lam_g_m"] / 1000
    i_line = 0.5 * m_line * (r0 ** 2 + r1 ** 2)
    i_races = 2 * 0.004 * 0.5 * (0.011 ** 2 + 0.0095 ** 2)
    return structure_kgm2 + i_line + i_races


def spinup(d, I=None):
    """Line pull and energy to spin the reel up during the launch stroke (constant acceleration)."""
    I = reel_inertia(d) if I is None else I
    R = pack_r(d, d["line_len_m"]) / 1000
    a = d["v_launch"] ** 2 / (2 * d["stroke_m"])
    f_inertia = I * a / R ** 2
    omega = d["v_launch"] / R
    return dict(accel=a, pull_n=f_inertia + d["brake_launch_n"], omega=omega, rpm=omega * 60 / (2 * math.pi),
                energy_j=0.5 * I * omega ** 2)


def overrun(d, brake_n, v_land, I=None, stored_m=None):
    """Line thrown off the reel after the dart stops dead, braked only by brake_n at the line (m, s)."""
    stored = d["line_len_m"] - 8.0 if stored_m is None else stored_m
    I = reel_inertia(d, stored) if I is None else I
    R = pack_r(d, stored) / 1000
    omega = v_land / R
    tau = brake_n * R
    t = I * omega / tau
    return dict(t_s=t, slack_m=0.5 * omega * t * R)


def min_brake_no_overrun(d, I=None):
    """Least line brake that keeps the reel from outrunning a decelerating dart in flight."""
    stored = d["line_len_m"] - 4.0
    I = reel_inertia(d, stored) if I is None else I
    R = pack_r(d, stored) / 1000
    return I * d["flight_decel"] / R ** 2


# ------------------------------------------------------------------------------------------------ slip clutch
def clutch(d):
    """Friction torque and axial preload for both settings (one annular face: the spool's outboard face on the clutch
    disk of the shaft; uniform wear). Convention (D49): line tension at the design pack radius clutch_r (27)."""
    r_ref = d["clutch_r"] / 1000
    r_eff = 0.5 * (d["face_r_in"] + d["face_r_out"]) / 1000
    out = {}
    for k in ("launch", "mid", "reel"):
        t = d[f"brake_{k}_n"] * r_ref
        f = t / (d["mu_face"] * r_eff)
        out[k] = dict(torque_nm=t, preload_n=f, push_n=f,
                      line_n_empty=t / (d["core_r"] / 1000), line_n_full=t / (pack_r(d, d["line_len_m"]) / 1000))
    out["r_ref_mm"] = d["clutch_r"]
    out["lever"] = d["lever_l"] / d["lever_a"]
    return out


def lever_leaf(d):
    """Clutch lever: the cam lift (brake_stroke less the gap) turns the lever; its lower arm (a PPA-CF leaf, overhung
    beam of length lever_l) bends by the tip travel (the thrust ring barely moves). Returns the tip travel, the leaf
    rate, the force added from LIGHT to FIRM, the force FIRM needs over LIGHT and the peak bending stress."""
    cl = clutch(d)
    travel = (d["brake_stroke"] - d["cam_gap"]) * d["lever_l"] / d["lever_a"]
    I = d["leaf_b"] * d["leaf_t"] ** 3 / 12
    k = 3 * d["leaf_e"] * I / d["lever_l"] ** 3
    df = k * travel
    need = cl["reel"]["preload_n"] - cl["launch"]["preload_n"]
    sigma = cl["reel"]["preload_n"] * d["lever_l"] * (d["leaf_t"] / 2) / I
    return dict(travel=travel, rate=k, added_n=df, need_n=need, light_n=cl["launch"]["preload_n"],
                firm_n=cl["launch"]["preload_n"] + df, sigma_max=sigma)


# ------------------------------------------------------------------------------------------------ gear train
def train(d):
    """Speeds, torques and line speed for the D49 reel train. Main = the REEL output gear on the main shaft."""
    up = cap_ratio(d)
    take = (d["z_bus"] / d["z_reel"]) / up                 # clutch disk rpm / capstan rpm in REEL
    rl = line_r_cap(d) / 1000
    out = dict(cap_per_main=up, reel_per_cap=take,
               c_main_c1=0.5 * d["m_cap"] * (d["z_main"] + d["z_c1"]),
               c_ga_gb=0.5 * d["m_take"] * (d["z_ga"] + d["z_gb"]), c_gb_gc=0.5 * d["m_take"] * (d["z_gb"] + d["z_gc"]),
               c_bus=0.5 * d["m_take"] * (d["z_bus"] + d["z_c1p"]),
               c_c1p_idler=0.5 * d["m_take"] * (d["z_c1p"] + d["z_idler"]),
               c_idler_t1=0.5 * d["m_take"] * (d["z_idler"] + d["z_reel"]),
               wraps_from_drum=d["wrap_idler_turns"] * d["z_wrap_idler"] / d["z_cup"],
               reel_per_wiper=d["z_bus"] / d["z_reel"])
    out["takeup_margin"] = take * d["core_r"] / line_r_cap(d)
    out["line_n_at_stall"] = d["wiper_stall_nm"] * d["eta_train"] / up / rl

    def at_tension(T):
        tau_c = T * rl
        tau_main = tau_c * up / d["eta_train"]
        frac = tau_main / d["wiper_stall_nm"]
        rpm_main = d["wiper_n0_rpm"] * max(0.0, 1 - frac)
        amps = d["wiper_i_free"] + frac * (d["wiper_i_stall"] - d["wiper_i_free"])
        v = rpm_main * up * 2 * math.pi / 60 * rl
        return dict(tau_cap=tau_c, tau_main=tau_main, rpm_main=rpm_main, rpm_cap=rpm_main * up, amps=amps, v_m_s=v)
    out["at"] = {T: at_tension(T) for T in (5.0, 30.0, 50.0, 100.0)}
    out["at_fn"] = at_tension
    return out


def tension_from_amps(d, amps):
    """Firmware estimate of the capstan-side line tension from wiper current (no load cell). Linear motor model;
    take-up drag and gear losses lumped in eta_train. Calibrate on the bench with a luggage scale."""
    up = cap_ratio(d)
    frac = max(0.0, (amps - d["wiper_i_free"]) / (d["wiper_i_stall"] - d["wiper_i_free"]))
    return frac * d["wiper_stall_nm"] * d["eta_train"] / up / (line_r_cap(d) / 1000)


def amps_for_tension(d, T):
    up = cap_ratio(d)
    frac = T * (line_r_cap(d) / 1000) * up / d["eta_train"] / d["wiper_stall_nm"]
    return d["wiper_i_free"] + frac * (d["wiper_i_stall"] - d["wiper_i_free"])


# ------------------------------------------------------------------------------------------------ cup (wrap arm)
def cup_torque(d, T, mu=0.20, lever=None):
    """Holding torque on the wrap cup during tow (N m, + = wind-in sense = unwrap sense).
    The dart line enters the ring (radius ring_r) with a tangential share eye_dx / hypot(eye_dx, eye_dz) of its pull
    (D49: the axial eyelet takes the line along the cup axis, so no tangential share at home). The line runs groove_wrap_deg round the groove and leaves tangent to the capstan
    line circle (lever 17.8 .. 20.5 mm with the segment's inclination) in the wind-in sense. In active tow the
    capstan is the pulling side, so the tension RISES through the groove: T exp(+mu wrap) (independent review)."""
    L = math.hypot(d["eye_dx"], d["eye_dz"])
    tang_in = d["eye_dx"] / L
    lev = (d["cap_r"] + d["line_d"] / 2) if lever is None else lever
    t_out = T * math.exp(mu * math.radians(d["groove_wrap_deg"]))
    return (-T * tang_in * d["ring_r"] + t_out * lev) / 1000


def cup_torque_range(d, T):
    """(min, max) cup torque over mu 0.10 .. 0.30 and the 17.8 .. 20.5 mm lever."""
    v = [cup_torque(d, T, mu, lev) for mu in (0.10, 0.20, 0.30) for lev in (17.8, 20.5)]
    return min(v), max(v)


def unwrap_schedule(d, step_turns=0.5, feed=0.8):
    """Interleaved unwrap after the dart is seated: each step the cup unwraps step_turns (releasing slack to the
    dart side), then the capstan winds `feed` of that slack so the line never goes tight on the seated dart and never
    lies loose by more than one step. Returns the list of (cup_turns_delta, capstan_deg_delta, main_deg_delta)."""
    up = cap_ratio(d)
    steps = []
    left = float(d["wraps"])
    while left > 1e-9:
        st = min(step_turns, left)
        left -= st
        slack = st * 2 * math.pi * line_r_cap(d)
        cap_rad = feed * slack / line_r_cap(d)
        steps.append((st, math.degrees(cap_rad), math.degrees(cap_rad) / up))
    return steps


def summary(P=None):
    d = get(P)
    rows = []
    add = lambda k, v, u="": rows.append((k, v, u))
    add("wraps (design)", d["wraps"])
    for mu in (d["mu_min"], d["mu_lo"], d["mu_hi"]):
        add(f"capstan gain e^(mu theta), mu {mu}", round(capstan_ratio(mu, d["wraps"]), 1))
    cl = clutch(d)
    for mu in (d["mu_min"], d["mu_lo"]):
        add(f"holds with REEL tail {cl['reel']['line_n_full']:.1f} N, mu {mu}",
            round(cl["reel"]["line_n_full"] * capstan_ratio(mu, d["wraps"]), 0), "N")
    add("wraps needed: 100 N, tail 2.6 N, mu 0.10, margin 2", wraps_needed(100, 2.6, 0.10))
    add("wrap line laid", round(wrap_line_mm(d), 1), "mm")
    add("pack radius full (20 m)", round(pack_r(d, d["line_len_m"]), 2), "mm")
    add("flange freeboard", round(d["flange_r"] - pack_r(d, d["line_len_m"]), 2), "mm")
    add("reel inertia (full, estimate)", f"{reel_inertia(d):.3e}", "kg m2")
    s = spinup(d)
    add("launch accel", round(s["accel"], 0), "m/s2")
    add("line pull in the stroke (inertia + brake)", round(s["pull_n"], 2), "N")
    add("reel speed at launch", round(s["rpm"], 0), "rpm")
    add("spin energy", round(s["energy_j"], 2), "J")
    add("least brake against overrun in flight", round(min_brake_no_overrun(d), 3), "N")
    for b in (d["brake_launch_n"], 1.0):
        o = overrun(d, b, 7.0)
        add(f"overrun after a dead stop at 7 m/s, brake {b} N", round(o["slack_m"], 2), "m")
    o = overrun(d, 6.0, 7.0)
    add("overrun with the slack-line shoe (6 N)", round(o["slack_m"], 3), "m")
    for k in ("launch", "reel"):
        add(f"clutch {k}: torque", round(cl[k]["torque_nm"], 4), "N m")
        add(f"clutch {k}: axial preload", round(cl[k]["preload_n"], 1), "N")
        add(f"clutch {k}: line N empty..full", f"{cl[k]['line_n_empty']:.2f}..{cl[k]['line_n_full']:.2f}", "N")
    add("clutch mid (wrapping): preload", round(cl["mid"]["preload_n"], 1), "N")
    lv = lever_leaf(d)
    add("clutch lever tip travel LIGHT -> FIRM", round(lv["travel"], 2), "mm")
    add("leaf rate / force added at FIRM / needed", f"{lv['rate']:.2f} N/mm / {lv['added_n']:.1f} N / {lv['need_n']:.1f} N")
    add("leaf peak bending stress at FIRM", round(lv["sigma_max"], 1), "MPa")
    tr = train(d)
    add("capstan rpm / main rpm", round(tr["cap_per_main"], 3))
    add("reel gear rpm / capstan rpm", round(tr["reel_per_cap"], 3))
    add("take-up margin at the empty core", round(tr["takeup_margin"], 3))
    add("centre REEL gear - C1 (no shift)", tr["c_main_c1"], "mm")
    add("centre c1' - swing idler", tr["c_c1p_idler"], "mm")
    add("wraps from the drum (idler turns x idler teeth / ring)", tr["wraps_from_drum"])
    add("clutch disk rpm / wiper rpm (bus)", tr["reel_per_wiper"])
    add("centre swing idler - T1", tr["c_idler_t1"], "mm")
    add("line tension at wiper stall", round(tr["line_n_at_stall"], 0), "N")
    for T, a in tr["at"].items():
        add(f"at {T:.0f} N: main torque / amps / line speed",
            f"{a['tau_main']:.2f} N m / {a['amps']:.1f} A / {a['v_m_s']:.3f} m/s")
    for T in (50.0, 100.0):
        lo, hi = cup_torque_range(d, T)
        add(f"cup holding torque at {T:.0f} N tow (mu 0.1..0.3, lever 17.8..20.5)", f"{lo:+.2f}..{hi:+.2f}", "N m")
    return rows


if __name__ == "__main__":
    for k, v, u in summary():
        print(f"{k:58s} {v} {u}")
