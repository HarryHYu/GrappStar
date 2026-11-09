"""v32 release by milling out: can the in-hand 100 rpm drills free the sleeve augers in moist loam at c 4 to 10 kPa, and the fixes.

  python dart_v32_release.py [OUT]   -> OUT/phys31/v32_release.json   (OUT defaults to out32)

Release sequence (firmware v4.1): rock the drills up to 5 times, then mill the pods UP with the drills spinning.
- To mill at all the drill must first turn the embedded sleeve: breakaway = spin torque (flight-tip cylinder shear,
  dart_anchor T_spin) x SETUP (1.3 after standing; 1.0 straight after anchoring). Once the pod moves up, the sleeve leaves
  a gap on its trailing side, so the running torque is HALF_CONTACT x spin + the milling share (same feed rule as the deploy).
- If the breakaway exceeds the stall, nothing turns and the deploy worm (0.21 N m) cannot drag the augers up sideways
  (passive 3.8 N m in loam): the dart stays anchored. Rocking only remoulds soil the motor can already strain, so no credit.
- Direction: the spin torque is the same either way (cylinder shear); the install direction keeps the flights pulling the
  sleeve along its own axis toward the tip (no help for a radial auger) and the release direction pushes it out (also no
  help, the pod holds the radius): whichever turns is fine; release direction is used so the sleeve also backs out of its thread.
Fixes compared: (1) printed planetary in the tip hub, sun on the M3 shaft, ring in the sleeve, carrier held to the motor:
ratio R/S, efficiency 0.85; (2) smaller flight OD (19 = plain sleeve, 20); (3) firmware: stop the deploy when the drill
reaches DEPTH_LIMIT_FRAC (0.67) of its stall (the release then always has breakaway margin), holding at that partial depth.
"""
import json, math, os, sys
import numpy as np
import dart_model as dm
import dart_anchor as da

SETUP = (1.0, 1.3)
HALF_CONTACT = 0.5
ETA_PLANET = 0.85
DEPTH_LIMIT_FRAC = 0.67      # 1 / (1.3 x 1.15): release after standing keeps a 1.15 breakaway margin


def spin(A, soil, th=None):
    root, tip, a = A.drill_axis(th)
    e = A.embedded(root, tip, A.d.P["AUGER_TIP"])
    return A.pile(e, soil, A.D)["T_spin_Nm"], e


def run(out=None):
    d = dm.Dart(out or os.path.join(dm.HERE, "out32"))
    P = d.P
    V = 7.4
    m100 = dm.motor("drill", V)
    wst = dm.motor("worm", V)["stall_Nm"]
    R = dict(source=d.out, assumptions=dict(setup=SETUP, half_contact=HALF_CONTACT, eta_planet=ETA_PLANET, depth_limit_frac=DEPTH_LIMIT_FRAC,
                                            drill_stall_Nm_7V4=m100["stall_Nm"], drill_free_rpm=m100["free_rpm"]), cases=[])
    drives = [("100 rpm direct", 1.0, 1.0), ("2.33:1 planetary 28/12/8", 28 / 12, ETA_PLANET), ("3:1 planetary", 3.0, ETA_PLANET)]
    for c in (4.0, 5.0, 7.5, 10.0):
        s = dict(dm.SOIL["moist loam"], c=c)
        for Df in (22.0, 20.0, 19.0):
            A = da.Anchor(d, auger_d_mm=Df)
            T, e = spin(A, s)
            th_touch = math.degrees(math.asin((A.piv[2] - A.zg) / A.r1))
            Mmax = max(2 * A.deploy_moment(s, th) for th in np.arange(th_touch, A.dep_deg + 0.01, 1.0))
            sweep = A.r1 / 1000 * math.radians(A.dep_deg - th_touch)
            row = dict(c_kPa=c, flight_od_mm=Df, spin_Nm=round(T, 3), hold_full_N=round(A.limit(s, 3.0)["F"], 1), drives={})
            for name, ratio, eta in drives:
                Tst = m100["stall_Nm"] * ratio * eta
                n_rev = m100["free_rad_s"] / ratio / (2 * math.pi) * 0.8
                fD = min(1.0, wst / Mmax)
                T_mill = float(np.trapezoid(da.soil_fns(s, A.D)[0](e["z"]) * fD, e["s"])) * A.D / 2
                run_T = HALF_CONTACT * T + T_mill
                row["drives"][name] = dict(stall_Nm=round(Tst, 3), breakaway_margin_fresh=round(Tst / (SETUP[0] * T), 2),
                                           breakaway_margin_after_standing=round(Tst / (SETUP[1] * T), 2),
                                           installs=bool(Tst >= T + T_mill), releases_fresh=bool(Tst >= SETUP[0] * T), releases_after_standing=bool(Tst >= SETUP[1] * T),
                                           running_margin=round(Tst / run_T, 2), mill_out_s=round(sweep / (fD * A.D * n_rev), 0))
                if name == "100 rpm direct":
                    # firmware depth limit: deepest pod angle with spin <= DEPTH_LIMIT_FRAC x stall
                    th_ok = th_touch
                    for th in np.arange(th_touch, A.dep_deg + 0.01, 0.5):
                        if spin(A, s, th)[0] <= DEPTH_LIMIT_FRAC * Tst:
                            th_ok = th
                    Ap = da.Anchor(d, dep_deg=th_ok, auger_d_mm=Df)
                    row["depth_limited_100rpm"] = dict(pod_deg=round(float(th_ok), 1), depth_mm=round(spin(A, s, th_ok)[1]["depth"] * 1000, 1),
                                                       hold_N=round(Ap.limit(s, 3.0)["F"], 1))
            R["cases"].append(row)
    os.makedirs(os.path.join(d.out, "phys31"), exist_ok=True)
    json.dump(R, open(os.path.join(d.out, "phys31", "v32_release.json"), "w"), indent=1, default=float)
    return R


if __name__ == "__main__":
    R = run(sys.argv[1] if len(sys.argv) > 1 else None)
    for r in R["cases"]:
        dr = r["drives"]
        print(r["c_kPa"], r["flight_od_mm"], "spin", r["spin_Nm"], "hold", r["hold_full_N"],
              {k: (v["installs"], v["breakaway_margin_fresh"], v["breakaway_margin_after_standing"], v["running_margin"], v["mill_out_s"]) for k, v in dr.items()},
              r.get("depth_limited_100rpm"))
