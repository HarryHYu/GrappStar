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
    # storage reel
    core_r=20.0, flange_r=32.0, width=20.0, flange_t=2.0, pack_fill=0.80,
    # slip clutch line tension targets at the mid pack radius
    brake_launch_n=0.75, brake_reel_n=3.0, mu_face=0.30, face_r_in=9.0, face_r_out=17.0, brake_stroke=4.0,
    # gears (module, teeth): main reel output (cad-drum) -> capstan gear; capstan gear -> idler -> reel gear
    m_cap=2.0, z_main=40, z_cap=24, z_idler=24, z_reel=15,
    m_cup=1.5, z_cup=30,
    # wiper (linear model, physics worker block); firmware tow limit
    wiper_n0_rpm=45.0, wiper_stall_nm=15.0, wiper_i_stall=18.0, wiper_i_free=1.5, eta_train=0.85,
    tow_limit_n=100.0,
    # launch (physics defaults until base/physics/FINDINGS.md): muzzle speed, stroke, flight decel
    v_launch=11.0, stroke_m=0.30, flight_decel=5.0,
    # v7 line geometry relative to the capstan (reel frame, see reel_NOTES.md)
    ring_r=24.5, eye_dx=70.0, eye_dz=31.5, groove_wrap_deg=58.0, groove_r=23.5,
)


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
    """Friction torque and axial preload for both settings (annular face, uniform wear)."""
    r_mid = 0.5 * (d["core_r"] + pack_r(d, d["line_len_m"])) / 1000
    r_eff = 0.5 * (d["face_r_in"] + d["face_r_out"]) / 1000
    out = {}
    for k in ("launch", "reel"):
        t = d[f"brake_{k}_n"] * r_mid
        f = t / (d["mu_face"] * r_eff)
        out[k] = dict(torque_nm=t, preload_n=f,
                      line_n_empty=t / (d["core_r"] / 1000), line_n_full=t / (pack_r(d, d["line_len_m"]) / 1000))
    k_spring = (out["reel"]["preload_n"] - out["launch"]["preload_n"]) / d["brake_stroke"]
    out["spring_rate_n_mm"] = k_spring
    out["precompress_mm"] = out["launch"]["preload_n"] / k_spring
    out["r_mid_mm"] = r_mid * 1000
    return out


# ------------------------------------------------------------------------------------------------ gear train
def train(d):
    """Speeds, torques and line speed for the reel train. Main = drum worker's reel output gear on the main shaft."""
    up = d["z_main"] / d["z_cap"]                         # capstan rpm / main rpm
    take = d["z_cap"] / d["z_reel"]                       # reel gear rpm / capstan rpm (idler only reverses twice)
    rl = line_r_cap(d) / 1000
    out = dict(cap_per_main=up, reel_per_cap=take,
               c_main_cap=0.5 * d["m_cap"] * (d["z_main"] + d["z_cap"]),
               c_cap_idler=0.5 * d["m_cap"] * (d["z_cap"] + d["z_idler"]),
               c_idler_reel=0.5 * d["m_cap"] * (d["z_idler"] + d["z_reel"]))
    # take-up must outrun the capstan at the smallest pack radius (most line out)
    out["takeup_margin"] = take * d["core_r"] / line_r_cap(d)
    # tension from motor torque: T = tau_main * eta / up / r
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
    return out


def tension_from_amps(d, amps):
    """Firmware estimate of the dart-side line tension from wiper current (no load cell). Linear motor model;
    take-up drag and gear losses lumped in eta_train. Calibrate on the bench with a luggage scale."""
    up = d["z_main"] / d["z_cap"]
    frac = max(0.0, (amps - d["wiper_i_free"]) / (d["wiper_i_stall"] - d["wiper_i_free"]))
    return frac * d["wiper_stall_nm"] * d["eta_train"] / up / (line_r_cap(d) / 1000)


def amps_for_tension(d, T):
    up = d["z_main"] / d["z_cap"]
    frac = T * (line_r_cap(d) / 1000) * up / d["eta_train"] / d["wiper_stall_nm"]
    return d["wiper_i_free"] + frac * (d["wiper_i_stall"] - d["wiper_i_free"])


# ------------------------------------------------------------------------------------------------ cup (wrap arm)
def cup_torque(d, T, mu=0.20):
    """Holding torque on the wrap cup during tow (N m, + = wind-in sense = unwrap sense).
    Dart line enters the ring (top, radius ring_r) from the exit eye (eye_dx forward, eye_dz axial); the line runs
    groove_wrap_deg round the groove and leaves the rim tangentially onto the capstan in the wind-in sense.
    Tension falls through the groove by exp(-mu * wrap) while the line slides toward the capstan."""
    L = math.hypot(d["eye_dx"], d["eye_dz"])
    tang_in = d["eye_dx"] / L                      # component of the dart pull along the tangent at the ring
    t_out = T * math.exp(-mu * math.radians(d["groove_wrap_deg"]))
    # dart pull: along -wind-in tangent at the ring (pulls the ring forward, the wraps run backward over the top)
    return (-T * tang_in * d["ring_r"] + t_out * d["groove_r"]) / 1000


def unwrap_schedule(d, step_turns=0.5, feed=0.8):
    """Interleaved unwrap after the dart is seated: each step the cup unwraps step_turns (releasing slack to the
    dart side), then the capstan winds `feed` of that slack so the line never goes tight on the seated dart and never
    lies loose by more than one step. Returns the list of (cup_turns_delta, capstan_deg_delta, main_deg_delta)."""
    up = d["z_main"] / d["z_cap"]
    steps = []
    n = int(round(d["wraps"] / step_turns))
    for _ in range(n):
        slack = step_turns * 2 * math.pi * line_r_cap(d)
        cap_rad = feed * slack / line_r_cap(d)
        steps.append((step_turns, math.degrees(cap_rad), math.degrees(cap_rad) / up))
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
        o = overrun(d, b, 8.0)
        add(f"overrun after a dead stop at 8 m/s, brake {b} N", round(o["slack_m"], 2), "m")
    o = overrun(d, 6.0, 8.0)
    add("overrun with the slack-line shoe (6 N)", round(o["slack_m"], 3), "m")
    for k in ("launch", "reel"):
        add(f"clutch {k}: torque", round(cl[k]["torque_nm"], 4), "N m")
        add(f"clutch {k}: axial preload", round(cl[k]["preload_n"], 1), "N")
        add(f"clutch {k}: line N empty..full", f"{cl[k]['line_n_empty']:.2f}..{cl[k]['line_n_full']:.2f}", "N")
    add("clutch spring rate", round(cl["spring_rate_n_mm"], 2), "N/mm")
    add("clutch spring precompression at LAUNCH", round(cl["precompress_mm"], 2), "mm")
    tr = train(d)
    add("capstan rpm / main rpm", round(tr["cap_per_main"], 3))
    add("reel gear rpm / capstan rpm", round(tr["reel_per_cap"], 3))
    add("take-up margin at the empty core", round(tr["takeup_margin"], 3))
    add("centre main-capstan", tr["c_main_cap"], "mm")
    add("centre capstan-idler", tr["c_cap_idler"], "mm")
    add("centre idler-reel", tr["c_idler_reel"], "mm")
    add("line tension at wiper stall", round(tr["line_n_at_stall"], 0), "N")
    for T, a in tr["at"].items():
        add(f"at {T:.0f} N: main torque / amps / line speed",
            f"{a['tau_main']:.2f} N m / {a['amps']:.1f} A / {a['v_m_s']:.3f} m/s")
    for T in (50.0, 100.0):
        add(f"cup holding torque at {T:.0f} N tow (mu 0.1..0.3)",
            f"{cup_torque(d, T, 0.1):+.2f}..{cup_torque(d, T, 0.3):+.2f}", "N m")
    return rows


if __name__ == "__main__":
    for k, v, u in summary():
        print(f"{k:58s} {v} {u}")
