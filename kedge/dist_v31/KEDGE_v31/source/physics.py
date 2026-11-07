"""KEDGE system physics (v31): mass/CG/inertia, aerodynamic centre, launch loads at the REAL launcher (base/physics),
landing loads, anchoring, tip-over, kick/deploy torque, battery, part strength.

  python3 physics.py OUT      -> OUT/physics.json (keys kept for report.py / variants.py) + OUT/phys31/*.json

The detailed models live in dart_model.py (mass, motors, soils), dart_anchor.py (anchoring and holding),
dart_landing.py (impact, rest orientation) and flight_sim.py (flight check). Every assumption is a named constant there.
"""
import json, math, os, sys
import numpy as np
import dart_model as dm

G = dm.G
PLA = dm.PLA
COUPLER_SIGMA = 80e6     # dart-cad v31: couplers in PPA-CF printed lying on the D flat (XY tensile about 100 MPa; 80 used)


def aero_centre(d, plane, alpha_deg=10.0, U=7.0):
    """Centre of the normal force (mm) at a small angle, from the same strip model as flight_sim.py."""
    import flight_sim as fs
    b = fs.Body(d, d.ballast_g, plane)
    a = math.radians(alpha_deg)
    vn = -U * math.sin(a) * np.ones_like(b.s)
    F = -0.5 * dm.RHO_AIR * fs.CDC * fs.ETA * b.w * np.abs(vn) * vn * 0.001 - dm.RHO_AIR * U * np.maximum(b.dS, 0) * vn * 0.001
    return float((F * b.xs).sum() / F.sum() * 1000)


def launch_loads(d):
    """Shot at 34 deg with this dart's mass: acceleration trace (base chain-spring launcher model)."""
    M = d.mp["M"] / 1000
    p, r, tr, (f_line, m_line) = dm.launch_trace(34.0, M)
    t, v = tr[:, 0], tr[:, 2]
    a = np.gradient(v, t)
    stroke = t <= r["t_stroke_ms"] / 1000 + 1e-9
    a_s = a[stroke]
    th = math.radians(34.0); mu = p.launch["mu_rail"]
    push = M * a_s + M * G * (math.sin(th) + mu * math.cos(th)) + f_line + m_line * a_s
    from payout import spin_up_in_stroke
    su = spin_up_in_stroke(p, float(r["v_muzzle"]), p.launch["stroke"], p.flight["brake_n"])
    t4 = float((a_s > 4 * G).sum() * np.median(np.diff(t)) * 1000)
    return dict(source="base/physics/launcher.py chain model, params " + p.source, pitch_deg=34.0, dart_mass_kg=round(M, 4),
                v_muzzle=round(float(r["v_muzzle"]), 2), stroke_m=p.launch["stroke"], t_stroke_ms=round(r["t_stroke_ms"], 1),
                energy_J=round(r["energy_stored_j"], 2), accel_peak_g=round(float(a_s.max()) / G, 1), accel_at_release_g=round(float(a_s[-6:-3].mean()) / G, 1),
                tail_push_peak_N=round(float(push.max()), 1), line_pull_in_stroke_N=round(f_line + m_line * float(a_s.max()), 2),
                line_spin_up_estimate_N=round(su["t_line_in_stroke_n"], 2), ms_above_4g=round(t4, 0),
                cradle_normal_N=round(M * G * math.cos(th), 2),
                pitch_couple_Nm=round(M * float(a_s.max()) * d.mp["cg"][2] / 1000, 3),
                note="the push acts on the flat tail face (pusher face covers z +-21 within y +-11): it can move up to the CG height "
                     "(z +%.1f mm), so the CG offset makes no nose lift in the cradle" % d.mp["cg"][2])


def run(out):
    d = dm.Dart(out)
    mp = d.mp; M = mp["M"]; cg = mp["cg"]
    R = dict(source=d.out, version="v31 physics (dart-physics, 7 Nov 2026)")
    R["mass_g"] = round(M, 1); R["cg_mm"] = [round(float(c), 1) for c in cg]
    R["printed_g"] = round(mp["printed"], 1); R["bought_g"] = round(M - mp["printed"] - d.ballast_g, 1); R["ballast_g"] = d.ballast_g
    R["inertia_kg_mm2"] = [[round(float(x) * 1e-3, 2) for x in row] for row in mp["I"]]
    R["heaviest"] = [(n, round(m, 1), [round(float(c), 1) for c in p]) for n, m, p, _ in sorted(mp["rows"], key=lambda r: -r[1])[:8]]
    D = 2 * d.P["R_OUT"]
    cp_top = aero_centre(d, "pitch"); cp_side = aero_centre(d, "yaw")
    R["flight"] = dict(direction="nose first (toward -x)", cg_x=round(float(cg[0]), 1), cp_side_x=round(cp_side, 1), cp_top_x=round(cp_top, 1),
                       static_margin_calibers_side=round((cp_side - cg[0]) / D, 2), static_margin_calibers_top=round((cp_top - cg[0]) / D, 2),
                       note="aerodynamic centre at 10 deg and 7 m/s from the strip model (cross-flow + slender-body nose term); "
                            "top = pitch plane (top-view width), side = yaw. At 7 m/s the air moments are small: the tether (0.33 N "
                            "at the tail) is what keeps it nose first (flight_sim.json).")
    R["launch"] = launch_loads(d)
    La = R["launch"]; gL = La["accel_peak_g"]
    La.update(v_needed_m_s=[La["v_muzzle"], La["v_muzzle"]], accel_g=[gL, gL], spring_energy_J=La["energy_J"])   # report.py keys
    # landing and anchoring (detailed files in OUT/phys31)
    import dart_landing, dart_anchor, flight_sim
    os.makedirs(os.path.join(d.out, "phys31"), exist_ok=True)
    FS = flight_sim.main(d.out, os.path.join(d.out, "phys31"))
    cases = []
    for pitch in (10, 34, 45):
        k = f"pitch, {d.ballast_g:g} g ballast, rail {pitch} deg, tether 0.25 N"
        if k in FS:
            cases.append(dict(pitch=pitch, v=FS[k]["v_impact"], path=FS[k]["path_angle_deg"]))
    LD = dart_landing.run(d.out, dict(impact_cases=cases))
    AN = dart_anchor.run(d.out)
    imp34 = next(r for r in LD["impact"] if r["rail_pitch"] == 34)
    soft = imp34["soft soil (loam, tilled)"]; grav = imp34["gravel (10 to 20 mm)"]; hard = imp34["hard ground (dry compacted, concrete)"]
    gI = soft["g_peak"]
    La.update(impact_g=gI, impact_stop_m=round(soft["depth_mm"] / 1000, 4))
    loads = []
    loads.append(["cells pushed aft at launch (each)", round(22 / 1000 * gL * G, 1), "N", "onto the rigid plates on the rear cell-bay disc: contact never breaks"])
    loads.append(["tail cap push at launch (dart + line)", La["tail_push_peak_N"], "N", "4 radial M3 into the spigot ring, hull wall in compression: fine"])
    for sn, r_ in (("soft soil", soft), ("gravel", grav), ("hard ground", hard)):
        loads.append([f"cells forward at landing, {sn} ({r_['g_peak']:.0f} g at the CG, rigid-body upper bound)", round(2 * r_["loads"]["cell_1"]["N"], 0), "N",
                      "onto the 3 mm front cell-bay disc and its springs"])
    pod_m = sum(d.part_mass(n, d.mesh(n)) for n in d.fam("drill_leg_L"))
    Wc = math.pi * (d.P["CPL_D"] / 1000) ** 3 / 32
    for sn, r_ in (("soft soil", soft), ("gravel", grav), ("hard ground", hard)):
        gp = r_["loads"]["drill_leg_L_motor"]["g"]
        sig = pod_m / 1000 * gp * G * 0.009 / Wc
        loads.append([f"coupler bending at landing, {sn} (pod {pod_m:.0f} g at {gp:.0f} g)", round(sig / 1e6, 1), "MPa", "PPA-CF 80 MPa: margin %.1f" % (COUPLER_SIGMA / sig)])
    kick_m = sum(d.part_mass(n, d.mesh(n)) for n in d.fam("kick_leg"))
    loads.append(["kick leg at landing, soft soil", soft["loads"]["kick_leg"]["N"], "N", "d3 steel shaft shear + ear bearing"])
    R["loads"] = loads
    R["landing"] = dict(impact_34deg=dict(soft=soft, gravel=grav, hard=hard), rest_states=LD["rest_states_folded"][:6], roll_slice=LD["roll_slice"],
                        nose_bumper=LD["nose_bumper_5mm_hard_ground"])
    # anchoring summary in the v29 key layout (design = planar limit analysis, rover 3 m)
    g = AN["geometry"]
    by = {}
    for sn, s in AN["soils"].items():
        lim3 = next(r for r in AN["holding_limit"][sn] if r["rover_m"] == 3.0)
        by[sn] = dict(drill_auger_each_N=s["drill_each"]["H_translation_N"], foot_auger_N=s["foot"]["H_translation_N"],
                      friction_N=round(M / 1000 * G * dm.SOIL[sn]["mu"], 1), total_N=lim3["F_design_N"],
                      total_if_worms_never_slip_N=lim3["F_if_worms_never_slip_N"], governing=lim3["governing"])
    R["anchoring"] = dict(depth_mm=dict(drill=g["drill"]["depth_mm"], foot=g["foot"]["depth_mm"]),
                          embedded_length_mm=dict(drill=g["drill"]["embedded_mm"], foot=g["foot"]["embedded_mm"]), by_soil=by,
                          note="total_N = max tether pull at the dart, rover 3 m, planar limit analysis (dart_anchor.py): soil on 10 "
                               "segments per auger, worm holding %.2f N m, ground contacts. Each-auger values = full lateral resistance." % dart_anchor.WORM_HOLD_NM)
    lo = next(r for r in AN["holding_limit"]["moist loam"] if r["rover_m"] == 3.0)
    lo1 = next(r for r in AN["holding_limit"]["moist loam"] if r["rover_m"] == 1.0)
    R["tip_over_foot_uplift_capacity_loam_N"] = AN["soils"]["moist loam"]["foot"]["Q_pull_N"]
    R["tip_over"] = dict(tether_height_mm=21, pull_N=lo["F_design_N"], moment_Nm=round(lo["F_design_N"] * 0.021, 3), foot_lever_mm=round(g["drill"]["entry_xz_mm"][0] - g["foot"]["entry_xz_mm"][0]),
                         foot_hold_down_needed_N=round(lo["F_design_N"] * 0.021 / ((g["drill"]["entry_xz_mm"][0] - g["foot"]["entry_xz_mm"][0]) / 1000), 2),
                         dart_weight_N=round(M / 1000 * G, 2), line_angle_deg_rover_1_3_5m=[r["beta_deg"] for r in AN["holding_limit"]["moist loam"] if r["rover_m"] in (1.0, 3.0, 5.0)],
                         without_foot_N_loam_3m=lo["F_without_foot_N"], loam_rover_1m_N=lo1["F_design_N"],
                         note="the planar limit analysis includes nose lift (hull belly contacts compression only) and the line angle")
    R["kick"] = dict(leg_mass_g=AN["kick"]["leg_mass_g"], cg_radius_mm=None, max_gravity_torque_kgcm=round(AN["kick"]["gravity_torque_max_Nm"] / G * 100, 3),
                     stall_Nm=AN["kick"]["stall_Nm"], margin_swing=AN["kick"]["margin_swing"], crowd_force_N=AN["kick"]["crowd_force_at_worm_stall_N"],
                     plant_passive_Nm=AN["kick_plant_passive_moment_Nm"], profile_kgcm=[],
                     note="worm N20 stall torque from the bench currents (dart_model.motor): %.2f N m at 7.4 V" % AN["kick"]["stall_Nm"])
    R["deploy"] = dict(pod_mass_g=round(pod_m, 1), gravity_torque_both_kgcm=None,
                       soil_moment_both_Nm={sn: s["deploy"]["passive_moment_max_Nm"] for sn, s in AN["soils"].items()},
                       time_to_deploy_s={sn: s["deploy"]["time_to_deploy_s"] for sn, s in AN["soils"].items()},
                       soil_cut_note="radial augers sweep sideways: they go in only as a mill with the drills spinning and the deploy worm at stall")
    E = AN["energy"]
    R["battery"] = dict(capacity_Wh=round(dart_anchor.CELL_MAH / 1000 * 7.4, 2),
                        idle_W_video_on=E["avionics_W"], hours_video_on=E["video_hours"], anchor_sequence_mAh=E["anchor_mAh"], worst_current_A=1.2,
                        release_mAh=E["release_mAh"])
    # strength
    km_ = d.mesh("kick_leg")
    try:
        sec = km_.section(plane_origin=[d.P["KICK_HUB_D"] / 2 + d.P["KICK_POLE_L"] / 2, 0, 0], plane_normal=[1, 0, 0])
        pl, _ = sec.to_2D(); poly = pl.polygons_full[0]
        from shapely.geometry import Polygon
        c = np.array(poly.centroid.coords[0]); pts = np.array(poly.exterior.coords) - c
        x, y = pts[:, 0], pts[:, 1]; cr = x[:-1] * y[1:] - x[1:] * y[:-1]
        Ix = abs(np.sum((y[:-1] ** 2 + y[:-1] * y[1:] + y[1:] ** 2) * cr) / 12); Iy = abs(np.sum((x[:-1] ** 2 + x[:-1] * x[1:] + x[1:] ** 2) * cr) / 12)
        Wp = min(Ix / np.abs(y).max(), Iy / np.abs(x).max()) * 1e-9
    except Exception:
        Wp = 44.8e-9
    Lp = d.P["KICK_POLE_L"] / 1000
    R["strength"] = dict(kick_pole_section_mm3=round(Wp * 1e9, 1), kick_pole_stress_MPa_at_20N_side_load=round(20 * Lp / Wp / 1e6, 1),
                         kick_pole_side_load_limit_N=round(PLA["sigma_t"] / 2 * Wp / Lp, 1),
                         coupler_torque_capacity_Nm=round(PLA["tau"] * math.pi * (d.P["CPL_D"] / 1000) ** 3 / 16 * 0.6, 2),
                         deploy_worm_moment_at_holding_limit_Nm=lo["deploy_worm_Nm"],
                         note="coupler capacity derated 0.6 for the D flat and layer lines; under tow the pods put up to the worm holding moment through the couplers")
    json.dump(R, open(os.path.join(d.out, "physics.json"), "w"), indent=1, default=float)
    return R


if __name__ == "__main__":
    R = run(sys.argv[1] if len(sys.argv) > 1 else None)
    print(json.dumps({k: R[k] for k in ("mass_g", "cg_mm", "flight", "launch", "anchoring", "battery")}, indent=1, default=float))
