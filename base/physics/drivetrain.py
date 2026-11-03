"""Wiper torque, speed, current, time and energy for every base station function through its train.

Functions (selector drum position in brackets):
  aim   [LAUNCH, one direction]  rail weight moment -> lever -> printed self-locking lead screw -> step-up mesh -> wiper
  cock  [LAUNCH, other direction] 2 springs -> pulley ratio -> winch drum -> cock ratio -> sprag #1 -> wiper
  reel  [REEL]  20 m at light line tension through the capstan, storage reel take-up through its slip clutch
  tow   [REEL]  5 m at the tow limit (capstan.py)

Driver limits: BTS7960 module 43 A peak (datasheet current limit) but heat limited; 15 A continuous without a heatsink
is used as the conservative number. Fuse: 20 A blade on lead #2. A 20 A blade carries 18 A for minutes, so a wiper
stall (about 17 A at 12.6 V with sag) will NOT blow it; it holds a 17 A stall indefinitely. The firmware current trip
(capstan.fw_trip: 50 ms blanking, 5 A hard trip for 30 ms (6.5 A while cocking), tow trip at free + tow current over 200 ms)
is the only stall protection for the motor and driver. The printed torque limiter in the wiper coupler
(p.wiper.limiter_nm, 5 N m) caps the torque every train can see; while it slips the motor draws about 7 A.
"(at limiter (5 N m))" checks use the limiter torque when it is set; the raw motor stall (no limiter) is reported as
info. Without a limiter the same checks run "(motor stall (no limiter))".
frac_stall = peak torque / raw motor stall torque (11.1 V, sag); frac_limiter = peak torque / limiter, flagged above
LIMITER_FRAC_MAX (the limiter would slip in normal use).
Shaft verdicts at stall/limiter: over the agreed 12 N m is a fail, over the Kt 2 value (9.4 N m) a warning.

Times use 11.1 V nominal with sag (slow case); stall currents use 12.6 V full (worst case).
Geometry not yet in the spec is keyword arguments with defaults below (marked ASSUMED).

python drivetrain.py   -> table + base/out/physics/drivetrain.json
"""
from __future__ import annotations

import math

from common import load_params, Params, PPA_CF, G
import capstan
import printed_gears as gears

STALL_FRAC_MAX = 0.6
LIMITER_FRAC_MAX = capstan.LIMITER_FRAC_MAX
BTS_PEAK_A = 43.0
BTS_CONT_A = 15.0

# Aim train from cad-launch NOTES (3 Nov): backward wiper -> swing-arm splitter -> 2:1 bevel -> single-start worm
# (m 2, pitch d 24, lead angle 4.8 deg) -> worm wheel 20 T = crank -> conrod -> rail lug. 10 wiper turns per crank turn.
# Pitch vs crank angle: the v10 table scaled (6.0 to 47.1 deg), from launch_NOTES.md section 7.
AIM_TABLE = [(0, 6.01), (10, 6.24), (20, 6.93), (30, 8.12), (40, 9.83), (50, 12.10), (60, 14.93), (70, 18.32),
             (80, 22.22), (90, 26.50), (100, 30.98), (110, 35.39), (120, 39.42), (130, 42.76), (140, 45.19),
             (150, 46.62), (160, 47.06), (170, 46.59), (180, 45.37), (190, 43.52), (200, 41.20), (210, 38.54),
             (220, 35.64), (230, 32.59), (240, 29.49), (250, 26.39), (260, 23.36), (270, 20.46), (280, 17.72),
             (290, 15.19), (300, 12.91), (310, 10.91), (320, 9.21), (330, 7.85), (340, 6.85), (350, 6.23), (360, 6.01)]
AIM_DEFAULTS = dict(
    rail_mass=0.90,       # kg rail beam, cheeks, springs, aim lug, everything that pitches (excl. dart + carriage). ASSUMED
    cg_dist=0.15,         # m pivot to CG of the pitching mass. ASSUMED
    worm_pd=0.024,        # m worm pitch diameter
    lead_deg=4.8,         # worm lead angle
    wheel_pd=0.040,       # m worm wheel pitch diameter (m 2 x 20 T)
    wheel_z=20,
    mu=0.25,              # printed worm on printed wheel, dry/greased PLA-PPA
    phi_n_deg=20.0,       # normal pressure angle
    bevel=2.0,            # worm turns per wiper turn
    eta_bevel=0.95,       # bevel + splitter idler
    gear=dict(module=2.0, face=10.0, z_driver=30),   # splitter / bevel stage (m 2 needed: m 1.5 fails at the 5 A trip)
)
COCK_DEFAULTS = dict(
    drum_r=0.050,         # m big gear pitch radius on the rack (cad-launch NOTES: m 2.5, 40 T, pitch d 100)
    ratio=40 / 26,        # wiper turns per big gear turn (drive gear 26 T on the main axis)
    eta=0.90,             # drive gear + rack meshes (printed spur, about 0.95 each)
    gear=dict(module=2.5, face=12.0, z_driver=26),
)
CAP_GEAR = capstan.CAP_GEAR


def stall(p: Params, v_oc: float):
    """Torque the train can see (limiter when set) and the motor current then."""
    return capstan.stall_torque_sag(p, v_oc)


def motor_stall(p: Params, v_oc: float):
    """Raw motor stall, no limiter."""
    return capstan.motor_stall_sag(p, v_oc)


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
    """Wiper torque over one crank turn. Raising: gravity moment through dtheta/dphi, worm efficiency. Lowering: the
    worm is self-locking, so the wiper must still drive it (screw lowering formula on the worm)."""
    a = dict(AIM_DEFAULTS, **kw)
    m = a["rail_mass"] + p.dart.mass + p.launch["carriage_mass"]
    lam = math.radians(a["lead_deg"])
    mu_e = a["mu"] / math.cos(math.radians(a["phi_n_deg"]))
    tl = math.tan(lam)
    eta_w = tl * (1 - mu_e * tl) / (tl + mu_e)
    r_w, r_wh = a["worm_pd"] / 2, a["wheel_pd"] / 2
    k_turns = a["wheel_z"] / a["bevel"]                      # wiper turns per crank turn
    rows = []
    for (p0, t0), (p1, t1) in zip(AIM_TABLE[:-1], AIM_TABLE[1:]):
        th = math.radians((t0 + t1) / 2)
        dth_dphi = (t1 - t0) / (p1 - p0)                     # rail deg per crank deg
        t_crank = m * G * a["cg_dist"] * math.cos(th) * dth_dphi   # + needs drive (raising), - gravity assists
        F = abs(t_crank) / r_wh                              # worm axial force
        if t_crank >= 0:
            t_worm = F * r_w * (tl + mu_e) / (1 - mu_e * tl)
        else:
            t_worm = F * r_w * (mu_e - tl) / (1 + mu_e * tl)
        rows.append((p1, t_crank, t_worm * a["bevel"] / a["eta_bevel"]))   # worm turns bevel x faster than the wiper
    t_w = [r[2] for r in rows]
    wiper_deg_rise = 160 * k_turns                           # crank 0 -> 160 deg raises 6 -> 47
    return dict(params=a, mass=m, screw_eff=eta_w, self_locking=tl * math.cos(math.radians(a["phi_n_deg"])) < a["mu"],
                t_screw_peak=max(r[1] for r in rows), t_screw_lower_peak=-min(r[1] for r in rows),
                t_peak=max(t_w), t_mean=sum(t_w) / len(t_w), wiper_turns=wiper_deg_rise / 360,
                wiper_turns_full_cycle=k_turns, profile=rows,
                shaft_torque=max(r[1] for r in rows), shaft_k=k_turns * eta_w * a["eta_bevel"], gear=a["gear"],
                gear_ratio=1.0 / a["bevel"], train_k=a["bevel"] / a["eta_bevel"], sprag_torque=None)


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
                wiper_turns=c["ratio"],     # one big gear turn per shot: cock, fire, reset (stroke alone is ~78 %)
                wiper_turns_stroke=drum_turns * c["ratio"], shaft_torque=max(fs) * c["drum_r"], shaft_k=c["ratio"] * c["eta"],
                sprag_torque=max(fs) * k, sprag_ratio=1.0,   # sprag #1 is in the drive gear on the wiper axis
                gear=c["gear"], gear_ratio=c["ratio"], train_k=k,
                spring_safe_ext_mm=lim["ext_safe_mm"], spring_ok=ext1 * 1000 <= lim["ext_safe_mm"],
                profile=fs)


# ------------------------------------------------------------------ reel and tow (capstan train)
def cap_fn(p: Params, f_line: float, length: float, overspeed: float | None = None) -> dict:
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
    t_s, i_s = motor_stall(p, v_oc)                      # raw motor stall (frac_stall)
    t_s_full, i_s_full = stall(p, p.battery.v_full)      # what the train sees: at the limiter when set
    t_m_full, _ = motor_stall(p, p.battery.v_full)       # raw, for info
    lab = capstan.stall_label(p)
    lim = capstan.limiter_nm(p)
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
    trip_a = capstan.TRIP_COCK_A if name == "cock" else capstan.TRIP_HARD_A
    t_trip = p.wiper.kt * (trip_a - p.wiper.i_free)
    mesh_tr = gears.mesh(t_trip, fn["gear_ratio"], g["module"], g["z_driver"], g["face"], PPA_CF, 0.0)
    shaft_tr = fn["shaft_k"] * t_trip
    sp = None
    if fn.get("sprag_torque") is not None:
        rpm_out = pk["rpm"] / fn.get("sprag_ratio", fn["gear_ratio"])
        sp = gears.sprag(fn["sprag_torque"], rpm_out, p.sprag)
    frac = fn["t_peak"] / t_s
    frac_lim = fn["t_peak"] / lim if lim is not None else None
    flags, stall_warn, info = [], [], []
    if frac > STALL_FRAC_MAX:
        flags.append(f"peak torque {frac:.2f} of motor stall > {STALL_FRAC_MAX}")
    if frac_lim is not None and frac_lim > LIMITER_FRAC_MAX:
        flags.append(f"peak torque {fn['t_peak']:.2f} N m is {frac_lim:.2f} of the {lim:g} N m limiter > "
                     f"{LIMITER_FRAC_MAX}: the limiter would slip in normal use")
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
        flags.append(f"gear (Lewis) fails at the {trip_a} A hard trip, sf {mesh_tr['sf']:.2f}")
    if not st_chk["pass"]:
        flags.append(f"({lab}) keyed shaft {shaft_st:.1f} N m > {p.shaft.t_allow} N m")
    elif not st_chk["strict_pass"]:
        stall_warn.append(f"keyed shaft {shaft_st:.1f} N m over the Kt 2 limit {st_chk['t_allow_kt']:.1f} N m")
    if not mesh_st["pass"]:
        stall_warn.append(f"gear fails (sf {mesh_st['sf']:.2f}); relies on the trip")
    # raw motor stall (no limiter): informational when the limiter caps the train
    shaft_mst = fn["shaft_k"] * t_m_full
    mesh_mst = gears.mesh(t_m_full, fn["gear_ratio"], g["module"], g["z_driver"], g["face"], PPA_CF, 0.0)
    if t_m_full > t_s_full:
        m_chk = gears.shaft_torsion(shaft_mst, p.shaft)
        verdict = "fail" if not m_chk["pass"] else ("over Kt 2" if not m_chk["strict_pass"] else "ok")
        info.append(f"motor stall (no limiter) {t_m_full:.2f} N m: shaft {shaft_mst:.1f} N m ({verdict}), gear sf "
                    f"{mesh_mst['sf']:.2f}; only if the limiter seizes")
    return dict(name=name, t_peak=fn["t_peak"], t_mean=fn["t_mean"], rpm_peak=pk["rpm"], rpm_mean=rpm_mean,
                i_peak=pk["current"], i_mean=mn["current"], v_batt_peak=pk["v_batt"], frac_stall=frac,
                frac_limiter=frac_lim, limiter_nm=lim, stall_label=lab, stall_torque=t_s,
                torque_at_limiter=t_s_full, time_s=time, energy_wh=energy_wh, shaft_torque=fn["shaft_torque"],
                shaft_torque_at_stall=shaft_st, shaft_stall=st_chk, shaft_torque_at_motor_stall=shaft_mst,
                gear_sf_motor_stall=mesh_mst["sf"], t_trip=t_trip,
                gear_sf_trip=mesh_tr["sf"], shaft_torque_at_trip=shaft_tr, gear_sf_peak=mesh_pk["sf"], gear_sf_stall=mesh_st["sf"],
                sprag=sp, flags=flags, stall_warnings=stall_warn, info=info)


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
    t_m_full, i_m_full = motor_stall(p, b.v_full)
    t_m_nom, i_m_nom = motor_stall(p, b.v_nom)
    lab = capstan.stall_label(p)
    elec = dict(stall_full=dict(torque=t_s_full, current=i_s_full), stall_nom=dict(torque=t_s_nom, current=i_s_nom),
                stall_label=lab, limiter_nm=capstan.limiter_nm(p),
                motor_stall_full=dict(torque=t_m_full, current=i_m_full),
                motor_stall_nom=dict(torque=t_m_nom, current=i_m_nom),
                free_rpm_full=p.wiper.operate(0.0, b, b.v_full)["rpm"],
                bts_peak_a=BTS_PEAK_A, bts_cont_a=BTS_CONT_A, fuse_a=b.fuse_a,
                fuse_blows_on_stall=i_m_full > b.fuse_a,
                note=("20 A blade survives 18 A for minutes and holds a %.1f A motor stall (no limiter) indefinitely: "
                      "the firmware current trip is the only stall protection (motor and BTS heating)" % i_m_full))
    general, info = [], []
    if not elec["fuse_blows_on_stall"]:
        general.append(elec["note"])
    if i_s_full > BTS_CONT_A:
        general.append(f"{lab}: current {i_s_full:.1f} A > BTS7960 {BTS_CONT_A} A continuous (no heatsink)")
    if t_m_full > t_s_full:
        info.append(f"{lab}: motor draws {i_s_full:.1f} A while the limiter slips (12.6 V); motor stall (no limiter) "
                    f"{t_m_full:.2f} N m / {i_m_full:.1f} A"
                    + (f" > BTS7960 {BTS_CONT_A} A continuous" if i_m_full > BTS_CONT_A else ""))
    if not c["spring_ok"]:
        general.append(f"spring extension {c['ext_full_mm']:.0f} mm > safe {c['spring_safe_ext_mm']:.0f} mm")
    if not a["self_locking"]:
        general.append("aim screw is not self-locking")
    cap = capstan.run(p, light_n=light_n, reel_len=reel_len)
    return dict(source=p.source, aim=dict({k: v for k, v in a.items()}), cock={k: v for k, v in c.items()
                if k != "profile"}, cock_profile=c["profile"], functions=rows, electrical=elec,
                general_flags=general, info=info + cap["info"], capstan_warnings=cap["warnings"],
                capstan_limiter=cap["limiter"], capstan_current_limit=cap["current_limit"],
                fw_trip=cap["fw_trip"], takeup=cap["takeup"],
                total_energy_wh=sum(r["energy_wh"] for r in rows.values()))


def main(argv=None):
    import argparse
    argparse.ArgumentParser(description=__doc__.splitlines()[0]).parse_args(argv)
    r = run()
    e = r["electrical"]
    print(f"[{r['source']}]  motor stall (no limiter) {e['motor_stall_full']['torque']:.2f} N m / "
          f"{e['motor_stall_full']['current']:.1f} A at 12.6 V, {e['motor_stall_nom']['torque']:.2f} N m / "
          f"{e['motor_stall_nom']['current']:.1f} A at 11.1 V; free {e['free_rpm_full']:.1f} rpm at 12.6 V")
    if e["limiter_nm"] is not None:
        print(f"  {e['stall_label']}: {e['stall_full']['torque']:.2f} N m, {e['stall_full']['current']:.1f} A while "
              "slipping (this is what every train sees)")
    hdr = f"{'fn':5} {'Tpk':>6} {'Tmean':>6} {'rpm':>5} {'Ipk':>5} {'Imean':>5} {'%stall':>6} {'%lim':>5} " \
          f"{'time s':>7} {'Wh':>6} {'shaftNm':>7} {'gear sf':>7}"
    print(hdr)
    for k, f in r["functions"].items():
        fl = f"{100*f['frac_limiter']:5.1f}" if f["frac_limiter"] is not None else f"{'-':>5}"
        print(f"{k:5} {f['t_peak']:6.2f} {f['t_mean']:6.2f} {f['rpm_mean']:5.1f} {f['i_peak']:5.2f} {f['i_mean']:5.2f} "
              f"{100*f['frac_stall']:6.1f} {fl} {f['time_s']:7.1f} {f['energy_wh']:6.3f} {f['shaft_torque']:7.2f} "
              f"{f['gear_sf_peak']:7.2f}")
    print(f"  %stall = peak / motor stall (no limiter, 11.1 V); %lim = peak / limiter (flag above "
          f"{100*LIMITER_FRAC_MAX:.0f} %)")
    lm = r["capstan_limiter"]
    if lm is not None:
        tw = r["functions"]["tow"]
        print(f"tow {r['functions']['tow']['t_peak']:.2f} N m at 50 N: {tw['i_peak']:.2f} A, {tw['rpm_mean']:.1f} wiper rpm; "
              f"limiter slip line force {lm['f_line_empty_reel']:.0f} N (empty reel) / {lm['f_line_full_reel']:.0f} N "
              f"(full reel) vs dart holding " + ", ".join(f"{k} {v:.0f}" for k, v in lm["dart_hold"].items())
              + f" N, line break {lm['line_breaking']:.0f} N")
    a, c = r["aim"], r["cock"]
    print(f"aim: mass {a['mass']:.2f} kg, screw eff {a['screw_eff']:.2f}, self-locking {a['self_locking']}, "
          f"crank torque {a['t_screw_peak']:.3f} N m raising / {a['t_screw_lower_peak']:.3f} gravity-assisted, worm turns per crank turn {a['params']['wheel_z']}")
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
            print(f" - {k} ({f['stall_label']}): {x}")
    for x in r["general_flags"] + r["capstan_warnings"]:
        print(" -", x)
    print("\nINFO:")
    for k, f in r["functions"].items():
        for x in f["info"]:
            print(f" - {k}: {x}")
    for x in r["info"]:
        print(" -", x)
    print("wrote", capstan.write_json(r, "drivetrain.json"))
    return r


if __name__ == "__main__":
    main()
