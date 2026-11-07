"""v32 early sizing: hollow auger sleeve round each drill motor (motor inside, the sleeve turns), radial on the pod as in v31.
What flight OD / pitch / flight thickness can the drill gearmotor turn and release in sand and loam, and what does it hold?

  python dart_v32_sizing.py [OUT_GEOM]   -> kedge/out31/phys31/v32_sizing.json (uses the v31 hull, pivots and stops)

Spin torque of an auger turning in place (radial pod: the swing gives the depth, the screw gives none), two mechanisms:
  B (design): the soil in the flight channel turns with the auger and shears on the flight-tip cylinder,
              T_B = int tau_s pi D_f (D_f / 2) ds, tau_s = 0.8 c + sigma_v tan(phi)                (dart_anchor.py rule)
  A (lower bound): the channel soil stays, the core and both flight faces slide on it (interface 0.5 c, tan 2/3 phi),
              T_A = int tau_i [pi D_c^2 / 2 + 2 (pi/4)(D_f^2 - D_c^2) / p x r_m] ds
Release needs 1.3 x the spin torque (set-up) plus the milling share; margin = motor stall / that.
Flight bending: cantilever h = (D_f - D_c) / 2 under the limit lateral pressure, sigma = 3 p h^2 / t^2.
"""
import json, math, os, sys
import numpy as np
import dart_model as dm
import dart_anchor as da

G = dm.G
CORE_OD = 19.0           # mm: 12 x 10 motor (diagonal 15.6) + 0.4 clearance + 1.5 wall
R0_SLEEVE = 10.0         # mm from the pod pivot to the sleeve start (motor end cap + bearing next to the pivot head)
TIP = 10.0               # mm cone at the sleeve end
MOTORS_ALT = {"100 rpm (in hand)": 100.0, "50 rpm (same body)": 50.0, "30 rpm (same body)": 30.0}


def spin_torques(emb, soil, Df, Dc, p, t):
    g, phi, c = soil["gamma"] * 1e3, math.radians(soil["phi"]), soil["c"] * 1e3
    s, z = emb["s"], emb["z"]
    sv = g * np.maximum(z, 0)
    tau_s = 0.8 * c + sv * math.tan(phi)
    tau_i = 0.5 * c + sv * math.tan(2 / 3 * phi)
    TB = float(np.trapezoid(tau_s * math.pi * Df * Df / 2, s))
    rm = (Df + Dc) / 4
    TA = float(np.trapezoid(tau_i * (math.pi * Dc ** 2 / 2 + 2 * math.pi / 4 * (Df ** 2 - Dc ** 2) / p * rm), s))
    p_lat = da.soil_fns(soil, Df)[0](z) / Df            # Pa, limit lateral pressure
    h = (Df - Dc) / 2
    sig = 3 * float(p_lat.max()) * h ** 2 / t ** 2 if h > 0 else 0.0
    return TB, TA, sig


def run(out=None):
    d = dm.Dart(out)
    P = d.P
    res = dict(source=d.out, assumptions=dict(core_od_mm=CORE_OD, sleeve_start_from_pivot_mm=R0_SLEEVE, tip_mm=TIP,
                                              deploy_deg=P["DEPLOY_DEG"], pivot_z=P["DRILL_PIV_Z"], soils=dm.SOIL, horns="v31 deploy horns + kick rest"),
               rows=[])
    mot = {k: (dm.MOTORS.__setitem__("drill", dict(dm.MOTORS["drill"], rpm12=v)) or dm.motor("drill", 7.4)) for k, v in MOTORS_ALT.items()}
    dm.MOTORS["drill"]["rpm12"] = 100.0
    wst = dm.motor("worm", 7.4)["stall_Nm"]
    for L in (95.0, 110.0):
        for Df in (21.0, 23.0, 25.0, 28.0):
            A = da.Anchor(d, auger_d_mm=Df)
            A.r0, A.r1 = R0_SLEEVE, R0_SLEEVE + L
            A.d.P = dict(P, AUGER_TIP=TIP)
            root, tip, a = A.drill_axis()
            emb = A.embedded(root, tip, TIP)
            th_touch = math.degrees(math.asin(min(1.0, (A.piv[2] - A.zg) / A.r1)))
            row = dict(L_mm=L, Df_mm=Df, depth_mm=round(emb["depth"] * 1000, 1), embedded_mm=round(emb["L"] * 1000, 1))
            for sn in ("dry sand", "moist loam"):
                s = dm.SOIL[sn]
                out_s = {}
                for p in (0.6 * Df, 0.8 * Df):
                    TB, TA, sig = spin_torques(emb, s, Df / 1000, CORE_OD / 1000, p / 1000, 1.2e-3)
                    out_s[f"pitch {p:.0f}"] = dict(T_spin_B_Nm=round(TB, 3), T_spin_A_Nm=round(TA, 3))
                TB = out_s[f"pitch {0.8 * Df:.0f}"]["T_spin_B_Nm"]
                Mmax = max(2 * A.deploy_moment(s, th) for th in np.arange(th_touch, A.dep_deg + 0.01, 1.0))
                fD = min(1.0, wst / Mmax) if Mmax > 0 else 1.0
                T_mill = float(np.trapezoid(da.soil_fns(s, Df / 1000)[0](emb["z"]) * fD, emb["s"])) * Df / 2000
                need = 1.3 * TB + T_mill
                lim = A.limit(s, 3.0); lim_nf = A.limit(s, 3.0, foot=False)
                sweep = A.r1 / 1000 * math.radians(A.dep_deg - th_touch)
                out_s.update(release_need_Nm=round(need, 3),
                             margin={k: round(m["stall_Nm"] / need, 2) for k, m in mot.items()},
                             deploy_passive_both_Nm=round(Mmax, 2),
                             deploy_s={k: round(sweep / (fD * Df / 1000 * m["free_rad_s"] / (2 * math.pi) * 0.8), 0) for k, m in mot.items()},
                             hold_3m_N=round(lim["F"], 1), hold_3m_no_foot_N=round(lim_nf["F"], 1), horn_Nm=lim["horn_moment_Nm"],
                             flight_bending_MPa_t1p2=round(spin_torques(emb, s, Df / 1000, CORE_OD / 1000, 0.8 * Df / 1000, 1.2e-3)[2] / 1e6, 2))
                # largest loam cohesion the in-hand motor still releases at margin 1.25
                if sn == "moist loam":
                    def need_c(cc):
                        s2 = dict(s, c=cc)
                        return 1.3 * spin_torques(emb, s2, Df / 1000, CORE_OD / 1000, 0.8 * Df / 1000, 1.2e-3)[0]
                    cmax = {}
                    for k, m in mot.items():
                        lo_, hi_ = 0.0, 60.0
                        for _ in range(40):
                            mid = (lo_ + hi_) / 2
                            (lo_, hi_) = (mid, hi_) if need_c(mid) * 1.25 <= m["stall_Nm"] else (lo_, mid)
                        cmax[k] = round(lo_, 1)
                    out_s["loam_c_max_kPa_at_margin_1p25"] = cmax
                row[sn] = out_s
            res["rows"].append(row)
    res["motors"] = mot
    os.makedirs(os.path.join(d.out, "phys31"), exist_ok=True)
    json.dump(res, open(os.path.join(d.out, "phys31", "v32_sizing.json"), "w"), indent=1, default=float)
    return res


if __name__ == "__main__":
    R = run(sys.argv[1] if len(sys.argv) > 1 else None)
    for r in R["rows"]:
        print(r["L_mm"], r["Df_mm"], "depth", r["depth_mm"], "emb", r["embedded_mm"])
        for sn in ("dry sand", "moist loam"):
            s = r[sn]
            print("   ", sn, {k: v for k, v in s.items() if k.startswith("pitch")}, "need", s["release_need_Nm"], "margin", s["margin"],
                  "hold", s["hold_3m_N"], "nofoot", s["hold_3m_no_foot_N"], "horn", s["horn_Nm"], "dep", s["deploy_passive_both_Nm"], s["deploy_s"],
                  "flight MPa", s["flight_bending_MPa_t1p2"], s.get("loam_c_max_kPa_at_margin_1p25", ""))
