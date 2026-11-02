"""Wiper torque, speed, current, time and energy for every base station function through its train.

Functions (selector drum position in brackets):
  aim   [LAUNCH, one direction]  rail weight moment -> lever -> printed self-locking lead screw -> step-up mesh -> wiper
  cock  [LAUNCH, other direction] 2 springs -> pulley ratio -> winch drum -> cock ratio -> sprag #1 -> wiper
  reel  [REEL]  20 m at light line tension through the capstan, storage reel take-up through its slip clutch
  tow   [REEL]  5 m at the tow limit (capstan.py)

Driver limits: BTS7960 module 43 A peak (datasheet current limit) but heat limited; 15 A continuous without a heatsink
is used as the conservative number. Fuse: 20 A blade on lead #2. A 20 A blade carries 18 A for minutes, so a wiper
stall (about 17 A at 12.6 V with sag) will NOT blow it; it holds a 17 A stall indefinitely. The firmware current trip
(capstan.fw_trip: 50 ms blanking, 5 A hard trip for 30 ms in all modes, tow trip at free + tow current over 200 ms)
is the only stall protection, and a mechanical slip clutch of about 3 N m on the REEL output gear is recommended.
Shaft verdicts at stall: over the agreed 12 N m is a fail, over the Kt 2 value (9.4 N m) a warning.

Times use 11.1 V nominal with sag (slow case); stall currents use 12.6 V full (worst case).
Geometry not yet in the spec is keyword arguments with defaults below (marked ASSUMED).

python drivetrain.py   -> table + base/out/physics/drivetrain.json
"""
from __future__ import annotations

import math

from common import load_params, Params, PPA_CF, G
import capstan
import gears

STALL_FRAC_MAX = 0.6
BTS_PEAK_A = 43.0
BTS_CONT_A = 15.0

# ASSUMED until base/spec/params.py lands
AIM_DEFAULTS = dict(
    rail_mass=0.90,       # kg rail beam, cheeks, springs, pulleys, everything that pitches (excl. dart + carriage)
    cg_dist=0.15,         # m pivot to CG of the pitching mass
    lever=0.06,           # m pivot to the lead screw nut pin (nut force taken perpendicular to the lever)
    screw_dm=0.010,       # m printed screw mean thread diameter
    screw_lead=0.004,     # m lead (single start)
    screw_mu=0.25,        # PLA/PPA on PLA thread friction
    thread_half_angle=15.0,   # deg, trapezoidal
    ratio=1.0 / 3.0,      # wiper turns per screw turn (step-up 3:1, screw 3x wiper speed)
    eta_gear=0.95,
    gear=dict(module=1.5, face=10.0, z_driver=45),   # wiper gear 45T drives screw gear 15T
)
COCK_DEFAULTS = dict(
    drum_r=0.015,         # m winch drum radius (cord to the carriage)
    ratio=1.0,            # wiper turns per drum turn
    eta=0.90,             # mesh + drum bearings
    gear=dict(module=2.0, face=12.0, z_driver=20),
)
CAP_GEAR = capstan.CAP_GEAR


def stall(p: Params, v_oc: float):
    return capstan.stall_torque_sag(p, v_oc)


def _point(p: Params, t_w: float, v_oc: float) -> dict:
    op = p.wiper.operate(t_w, p.battery, v_oc)
    return dict(t=t_w, rpm=op["rpm"], speed=op["speed"], current=op["current"], i_batt=op["i_batt"],
                v_batt=op["v_batt"], power_in=op["power_in"], stalled=op["stalled"])


# ------------------------------------------------------------------ aim
def screw_eff(dm, lead, mu, half_deg):
    mu_e = mu / math.cos(math.radians(half_deg))
    tl = lead / (math.pi * dm)
    eta = tl * (1 - mu_e * tl) / (tl + mu_e)
    return eta, mu_e, tl


def aim(p: Params, **kw) -> dict:
    a = dict(AIM_DEFAULTS, **kw)
    m = a["rail_mass"] + p.dart.mass + p.launch["carriage_mass"]
    lo, hi = math.radians(p.flight["pitch_min"]), math.radians(p.flight["pitch_max"])
    eta_s, mu_e, tl = screw_eff(a["screw_dm"], a["screw_lead"], a["screw_mu"], a["thread_half_angle"])
    r = a["screw_dm"] / 2
    th = [lo + (hi - lo) * i / 40 for i in range(41)]
    t_raise, t_lower = [], []
    for x in th:
        f = m * G * a["cg_dist"] * math.cos(x) / a["lever"]
        t_raise.append(f * a["screw_lead"] / (2 * math.pi * eta_s))
        t_lower.append(f * r * (mu_e - tl) / (1 + mu_e * tl))      # self-locking: lowering also needs torque
    k = 1.0 / (a["ratio"] * a["eta_gear"])
    w_peak = max(t_raise) * k
    w_mean = sum(t_raise) / len(t_raise) * k
    travel = a["lever"] * (hi - lo)
    screw_turns = travel / a["screw_lead"]
    wiper_turns = screw_turns * a["ratio"]
    return dict(params=a, mass=m, screw_eff=eta_s, self_locking=tl < mu_e, t_screw_peak=max(t_raise),
                t_screw_lower_peak=max(t_lower), t_peak=w_peak, t_mean=w_mean, wiper_turns=wiper_turns,
                shaft_torque=max(t_raise), shaft_k=a["ratio"] * a["eta_gear"], gear=a["gear"], gear_ratio=a["ratio"], train_k=k,
                sprag_torque=None)


# ------------------------------------------------------------------ cock
def cock(p: Params, **kw) -> dict:
    c = dict(COCK_DEFAULTS, **kw)
    L = p.launch
    pr = L["pulley_ratio"]
    ext0 = L["preload_ext"]
    ext1 = ext0 + L["stroke"] / pr
    eta_p = L["eta_pulley"] ** L["n_pulley_turns"]
    pitch = math.radians(p.flight["pitch_min"])
    fric = L["mu_rail"] * (L["carriage_mass"] + p.dart.mass) * G * math.cos(pitch)   # gravity help ignored

    def f_carriage(x):           # x = carriage travel from rest
        return p.n_springs * p.spring.force(ext0 + x / pr) / pr / eta_p + fric
    n = 60
    fs = [f_carriage(L["stroke"] * i / n) for i in range(n + 1)]
    k = c["drum_r"] / (c["ratio"] * c["eta"])
    drum_turns = L["stroke"] / (2 * math.pi * c["drum_r"])
    lim = p.spring.limits()
    return dict(params=c, ext_full_mm=ext1 * 1000, f_spring_full=p.n_springs * p.spring.force(ext1),
                f_peak=max(fs), f_mean=sum(fs) / len(fs), t_peak=max(fs) * k, t_mean=sum(fs) / len(fs) * k,
                wiper_turns=drum_turns * c["ratio"], shaft_torque=max(fs) * c["drum_r"], shaft_k=c["ratio"] * c["eta"],
                sprag_torque=max(fs) * c["drum_r"], gear=c["gear"], gear_ratio=c["ratio"], train_k=k,
                spring_safe_ext_mm=lim["ext_safe_mm"], spring_ok=ext1 * 1000 <= lim["ext_safe_mm"],
                profile=fs)


# ------------------------------------------------------------------ reel and tow (capstan train)
def cap_fn(p: Params, f_line: float, length: float, overspeed: float = 1.2) -> dict:
    c = p.capstan
    t_w = capstan.wiper_torque_for(f_line, p, overspeed)
    r = c["d"] / 2
    drum_turns = length / (2 * math.pi * r)
    t_drum = capstan.drum_shaft_torque(t_w, p)     # same treatment as capstan.py (take-up branch included)
    return dict(f_line=f_line, length=length, t_peak=t_w, t_mean=t_w, wiper_turns=drum_turns * c["ratio"],
                shaft_torque=t_drum, sprag_torque=t_drum, shaft_k=c["ratio"] * c["eta"], gear=CAP_GEAR, gear_ratio=c["ratio"],
                train_k=r / (c["ratio"] * c["eta"]))


# ------------------------------------------------------------------ evaluate one function
def evaluate(p: Params, name: str, fn: dict, v_oc: float) -> dict:
    t_s, i_s = stall(p, v_oc)
    t_s_full, i_s_full = stall(p, p.battery.v_full)
    pk = _point(p, fn["t_peak"], v_oc)
    mn = _point(p, fn["t_mean"], v_oc)
    rpm_mean = mn["rpm"]
    time = fn["wiper_turns"] / (rpm_mean / 60) if rpm_mean > 0 else math.inf
    energy_wh = mn["power_in"] * time / 3600
    g = fn["gear"]
    mesh_pk = gears.mesh(fn["t_peak"], fn["gear_ratio"], g["module"], g["z_driver"], g["face"], PPA_CF, pk["rpm"])
    mesh_st = gears.mesh(t_s_full, fn["gear_ratio"], g["module"], g["z_driver"], g["face"], PPA_CF, 0.0)
    shaft_pk = gears.shaft_torsion(fn["shaft_torque"], p.shaft)
    # shaft torque at wiper stall and at the hard trip, through the same train
    shaft_st = fn["shaft_k"] * t_s_full
    st_chk = gears.shaft_torsion(shaft_st, p.shaft)
    t_trip = p.wiper.kt * (capstan.TRIP_HARD_A - p.wiper.i_free)
    mesh_tr = gears.mesh(t_trip, fn["gear_ratio"], g["module"], g["z_driver"], g["face"], PPA_CF, 0.0)
    shaft_tr = fn["shaft_k"] * t_trip
    sp = None
    if fn.get("sprag_torque") is not None:
        rpm_out = pk["rpm"] / fn["gear_ratio"]
        sp = gears.sprag(fn["sprag_torque"], rpm_out, p.sprag)
    frac = fn["t_peak"] / t_s
    flags, stall_warn = [], []
    if frac > STALL_FRAC_MAX:
        flags.append(f"peak torque {frac:.2f} of stall > {STALL_FRAC_MAX}")
    if not shaft_pk["pass"]:
        flags.append(f"keyed shaft {fn['shaft_torque']:.1f} N m > {p.shaft.t_allow} N m")
    elif not shaft_pk["strict_pass"]:
        flags.append(f"keyed shaft {fn['shaft_torque']:.1f} N m over the Kt 2 limit {shaft_pk['t_allow_kt']:.1f} N m")
    if not mesh_pk["pass"]:
        flags.append(f"gear (Lewis) fails at peak, sf {mesh_pk['sf']:.2f}")
    if sp is not None and not sp["pass"]:
        flags.append("sprag over rating")
    if pk["current"] > BTS_CONT_A:
        flags.append(f"peak current {pk['current']:.1f} A > BTS7960 {BTS_CONT_A} A continuous")
    if not mesh_tr["pass"]:
        flags.append(f"gear (Lewis) fails at the {capstan.TRIP_HARD_A} A hard trip, sf {mesh_tr['sf']:.2f}")
    if not st_chk["pass"]:
        flags.append(f"(at stall) keyed shaft {shaft_st:.1f} N m > {p.shaft.t_allow} N m")
    elif not st_chk["strict_pass"]:
        stall_warn.append(f"keyed shaft {shaft_st:.1f} N m over the Kt 2 limit {st_chk['t_allow_kt']:.1f} N m")
    if not mesh_st["pass"]:
        stall_warn.append(f"gear fails at wiper stall (sf {mesh_st['sf']:.2f}); relies on the trip / slip clutch")
    return dict(name=name, t_peak=fn["t_peak"], t_mean=fn["t_mean"], rpm_peak=pk["rpm"], rpm_mean=rpm_mean,
                i_peak=pk["current"], i_mean=mn["current"], v_batt_peak=pk["v_batt"], frac_stall=frac,
                stall_torque=t_s, time_s=time, energy_wh=energy_wh, shaft_torque=fn["shaft_torque"],
                shaft_torque_at_stall=shaft_st, shaft_stall=st_chk, t_trip=t_trip,
                gear_sf_trip=mesh_tr["sf"], shaft_torque_at_trip=shaft_tr, gear_sf_peak=mesh_pk["sf"], gear_sf_stall=mesh_st["sf"],
                sprag=sp, flags=flags, stall_warnings=stall_warn)


def run(p: Params | None = None, aim_kw=None, cock_kw=None, light_n=3.0, reel_len=20.0, tow_len=5.0) -> dict:
    p = p or load_params()
    b = p.battery
    tow_n = p.capstan["tow_limit_n"]
    a = aim(p, **(aim_kw or {}))
    c = cock(p, **(cock_kw or {}))
    fns = dict(aim=a, cock=c, reel=cap_fn(p, light_n, reel_len), tow=cap_fn(p, tow_n, tow_len))
    rows = {k: evaluate(p, k, v, b.v_nom) for k, v in fns.items()}
    t_s_full, i_s_full = stall(p, b.v_full)
    t_s_nom, i_s_nom = stall(p, b.v_nom)
    elec = dict(stall_full=dict(torque=t_s_full, current=i_s_full), stall_nom=dict(torque=t_s_nom, current=i_s_nom),
                free_rpm_full=p.wiper.operate(0.0, b, b.v_full)["rpm"],
                bts_peak_a=BTS_PEAK_A, bts_cont_a=BTS_CONT_A, fuse_a=b.fuse_a,
                fuse_blows_on_stall=i_s_full > b.fuse_a,
                note=("20 A blade survives 18 A for minutes and holds a %.1f A stall indefinitely: the firmware "
                      "current trip is the only stall protection (motor and BTS heating)" % i_s_full))
    general = []
    if not elec["fuse_blows_on_stall"]:
        general.append(elec["note"])
    if i_s_full > BTS_CONT_A:
        general.append(f"stall current {i_s_full:.1f} A > BTS7960 {BTS_CONT_A} A continuous (no heatsink)")
    if not c["spring_ok"]:
        general.append(f"spring extension {c['ext_full_mm']:.0f} mm > safe {c['spring_safe_ext_mm']:.0f} mm")
    if not a["self_locking"]:
        general.append("aim screw is not self-locking")
    cap = capstan.run(p, light_n=light_n, reel_len=reel_len)
    return dict(source=p.source, aim=dict({k: v for k, v in a.items()}), cock={k: v for k, v in c.items()
                if k != "profile"}, cock_profile=c["profile"], functions=rows, electrical=elec,
                general_flags=general, capstan_warnings=cap["warnings"], capstan_current_limit=cap["current_limit"],
                fw_trip=cap["fw_trip"], takeup=cap["takeup"],
                total_energy_wh=sum(r["energy_wh"] for r in rows.values()))


def main():
    r = run()
    e = r["electrical"]
    print(f"[{r['source']}]  wiper stall {e['stall_full']['torque']:.2f} N m / {e['stall_full']['current']:.1f} A "
          f"at 12.6 V, {e['stall_nom']['torque']:.2f} N m / {e['stall_nom']['current']:.1f} A at 11.1 V; "
          f"free {e['free_rpm_full']:.1f} rpm at 12.6 V")
    hdr = f"{'fn':5} {'Tpk':>6} {'Tmean':>6} {'rpm':>5} {'Ipk':>5} {'Imean':>5} {'%stall':>6} {'time s':>7} " \
          f"{'Wh':>6} {'shaftNm':>7} {'gear sf':>7}"
    print(hdr)
    for k, f in r["functions"].items():
        print(f"{k:5} {f['t_peak']:6.2f} {f['t_mean']:6.2f} {f['rpm_mean']:5.1f} {f['i_peak']:5.2f} {f['i_mean']:5.2f} "
              f"{100*f['frac_stall']:6.1f} {f['time_s']:7.1f} {f['energy_wh']:6.3f} {f['shaft_torque']:7.2f} "
              f"{f['gear_sf_peak']:7.2f}")
    a, c = r["aim"], r["cock"]
    print(f"aim: mass {a['mass']:.2f} kg, screw eff {a['screw_eff']:.2f}, self-locking {a['self_locking']}, "
          f"screw torque {a['t_screw_peak']:.3f} N m raise / {a['t_screw_lower_peak']:.3f} lower")
    print(f"cock: spring ext {c['ext_full_mm']:.0f} mm (safe {c['spring_safe_ext_mm']:.0f}), "
          f"springs {c['f_spring_full']:.1f} N, carriage peak {c['f_peak']:.1f} N")
    print(f"total energy {r['total_energy_wh']:.3f} Wh")
    ft = r["fw_trip"]
    print(f"\nFIRMWARE TRIP: {ft['notes'][0]}; tow trip = {ft['tow_trip_rule']}")
    print("  gear sf at the hard trip: " + ", ".join(f"{k} {f['gear_sf_trip']:.2f}" for k, f in r["functions"].items())
          + "; shaft at trip: " + ", ".join(f"{k} {f['shaft_torque_at_trip']:.2f}" for k, f in r["functions"].items())
          + " N m")
    for n in ft["notes"][1:]:
        print("  note:", n)
    print("\nFLAGS:")
    for k, f in r["functions"].items():
        for x in f["flags"]:
            print(f" - {k}: {x}")
        for x in f["stall_warnings"]:
            print(f" - {k} (at stall): {x}")
    for x in r["general_flags"] + r["capstan_warnings"]:
        print(" -", x)
    print("wrote", capstan.write_json(r, "drivetrain.json"))
    return r


if __name__ == "__main__":
    main()
