"""KEDGE v31 flight sanity check at the REAL launcher numbers (about 7 m/s, base/physics): does the folded dart fly nose
first and land nose first? Rigid body in the vertical plane (pitch) and, separately, the horizontal plane (yaw).

  python flight_sim.py OUT PK    -> OUT/flight_sim.json, PK/flight_sim.png   (PK may equal OUT/phys31)

Model: mass, CG and inertia from dart_model (all parts, own inertia included, ballast as set); muzzle speed from
base/physics/launcher.py at each rail pitch with this dart's mass; launch point and range origin as base flight.py
(rail pivot geometry). Aero per 1 mm station: viscous cross-flow (Cdc 1.2 x eta 0.65) on the TOP-view width for pitch
(SIDE-view height for yaw), the slender-body potential term rho U S'(x) w(x) where the section area grows (nose,
leg roots), axial drag CdA 2.6e-3 m2 (base Dart). Tether: tension at the dart from base flight.py (bead-chain line,
0.25 N LIGHT brake x payout gain 1.32 + line drag) applied at the tail knot along the flight path.
Disturbance: 5 deg nose-up at the muzzle plus 1 rad/s pitch (or yaw) rate (tip-off off the cradle about 0.4 rad/s).
"""
import sys, json, math, os
import numpy as np
import dart_model as dm

CDC, ETA = 1.2, 0.65
RHO = dm.RHO_AIR


class Body:
    def __init__(self, d, ballast_g=15.0, plane="pitch"):
        d.ballast_g = ballast_g
        mp = d.mass_props(d.fold)
        self.M = mp["M"] / 1000; self.cg = mp["cg"] / 1000
        self.I = mp["I"][1, 1] * 1e-9 if plane == "pitch" else mp["I"][2, 2] * 1e-9
        pr = d.profiles()
        self.xs = pr["xs"] / 1000
        self.w = (pr["top"] if plane == "pitch" else pr["side"]) / 1000
        S = pr["S"] * 1e-6
        self.dS = np.gradient(S, self.xs)
        self.s = self.cg[0] - self.xs              # + toward the nose
        self.s_tail = self.cg[0] - 0.152
        self.cda = 2.6e-3


def run(b, v0, elev, T_of_t=None, dth=5.0, w0=1.0, tmax=3.0, dt=2e-4, h0=0.41):
    """Plane flight. th = body axis (nose direction) above horizontal; returns rows t, x, z, alpha, theta, speed."""
    th = math.radians(elev + dth); w = w0
    vx, vz = v0 * math.cos(math.radians(elev)), v0 * math.sin(math.radians(elev))
    x, z, t = 0.0, h0, 0.0
    out = []
    ds = 0.001
    while t < tmax and z > 0.0:
        e = np.array([math.cos(th), math.sin(th)]); n = np.array([-math.sin(th), math.cos(th)])
        v = np.array([vx, vz]); U = v @ e; vn0 = v @ n
        vn = vn0 + w * b.s                                         # local normal velocity (+ along n)
        F_cf = -0.5 * RHO * CDC * ETA * b.w * np.abs(vn) * vn * ds
        dSp = np.maximum(b.dS, 0) if U >= 0 else np.maximum(-b.dS, 0)
        F_pot = -RHO * abs(U) * dSp * vn * ds
        Fn = (F_cf + F_pot).sum(); Mn = ((F_cf + F_pot) * b.s).sum()
        Fa = -0.5 * RHO * b.cda * abs(U) * U
        F = Fa * e + Fn * n + np.array([0, -dm.G * b.M])
        if T_of_t is not None:
            T = T_of_t(t); sp = np.linalg.norm(v); u = -v / max(sp, 1e-6)
            F = F + T * u
            r = b.s_tail * e
            Mn += r[0] * T * u[1] - r[1] * T * u[0]
        vx += F[0] / b.M * dt; vz += F[1] / b.M * dt; x += vx * dt; z += vz * dt
        w += Mn / b.I * dt; th += w * dt; t += dt
        alpha = math.degrees(math.atan2(vn0, U))
        if not out or t - out[-1][0] >= 0.005:
            out.append((t, x, z, alpha, math.degrees(th), math.hypot(vx, vz)))
    return np.array(out)


def summarize(o):
    late = o[o[:, 0] > 0.15]
    a_imp = float(o[-1, 3]); path = math.degrees(math.atan2(-(o[-1, 2] - o[-2, 2]), o[-1, 1] - o[-2, 1]))
    return dict(range_m=round(float(o[-1, 1]), 2), t_flight_s=round(float(o[-1, 0]), 3), apex_m=round(float(o[:, 2].max()), 2),
                v_impact=round(float(o[-1, 5]), 2), path_angle_deg=round(path, 1), alpha_impact_deg=round(a_imp, 1),
                alpha_peak_after_0p15s_deg=round(float(np.abs(late[:, 3]).max()) if len(late) else 0.0, 1),
                nose_first=bool(np.abs(o[:, 3]).max() < 45), nose_pitch_at_impact_deg=round(float(o[-1, 4]), 1))


def main(OUT, PK):
    d = dm.Dart(OUT)
    p = dm.base_params()
    from launcher import launch
    from flight import simulate
    L = p.launch
    R = dict(source=d.out, model=__doc__.split("\n")[3:9])
    figdat = {}
    for plane in ("pitch", "yaw"):
        for bal in (0.0, 8.0, 15.0):
            b = Body(d, bal, plane)
            p.dart.mass = b.M
            for pitch in ((10, 20, 34, 45) if plane == "pitch" and bal == 15.0 else (34,)):
                v0 = float(launch(p, pitch)["v_muzzle"])
                th = math.radians(pitch)
                mz = (L["release_u"] * math.cos(th) - L["dart_w"] * math.sin(th), L["pivot_h"] + L["release_u"] * math.sin(th) + L["dart_w"] * math.cos(th))
                fr = simulate(p, v0, pitch, record=True)
                hist = np.array(fr["history"])
                Tf = (lambda tt, h=hist: float(np.interp(tt, h[:, 0], h[:, 5]))) if len(hist) else (lambda tt: 0.33)
                for teth in (True, False):
                    elev = pitch if plane == "pitch" else 0.0
                    o = run(b, v0, pitch, Tf if teth else None, h0=mz[1]) if plane == "pitch" else run_yaw(b, v0, Tf if teth else None, fr["t_flight_s"])
                    key = f"{plane}, {bal:g} g ballast, rail {pitch} deg, {'tether 0.25 N' if teth else 'no tether'}"
                    sm = summarize(o) if plane == "pitch" else yaw_summary(o)
                    sm.update(v_muzzle=round(v0, 2), muzzle_xz_m=[round(mz[0], 3), round(mz[1], 3)], base_range_m=round(fr["range_m"], 2),
                              base_t_flight_s=round(fr["t_flight_s"], 3), max_line_pull_N=round(fr["t_dart_max_n"], 2))
                    R[key] = sm
                    figdat[key] = o
    R["inputs"] = dict(mass_kg=round(Body(d, 15.0).M, 4), cg_x_mm=round(Body(d, 15.0).cg[0] * 1000, 1), cross_flow_cd=CDC, eta=ETA, axial_cda=2.6e-3)
    json.dump(R, open(os.path.join(OUT, "flight_sim.json"), "w"), indent=1)
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "Liberation Sans", "font.size": 8})
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    ls = {0.0: ":", 8.0: "--", 15.0: "-"}
    for k, o in figdat.items():
        if k.startswith("pitch") and "rail 34" in k:
            bal = float(k.split(", ")[1].split(" ")[0])
            ax[0].plot(o[:, 0], o[:, 3], "k", ls=ls[bal], lw=1.4 if "tether 0.25" in k else 0.6, label=k.replace("pitch, ", ""))
        if k.startswith("pitch") and "15 g" in k and "tether 0.25" in k:
            ax[1].plot(o[:, 1], o[:, 2], "k-", lw=0.8)
            ax[1].text(o[-1, 1], 0.03, k.split("rail ")[1].split(",")[0], fontsize=7, ha="center")
    ax[0].set_xlabel("time (s)"); ax[0].set_ylabel("angle of attack (deg)"); ax[0].grid(lw=0.3); ax[0].legend(fontsize=6)
    ax[1].set_xlabel("x from the rail pivot (m)"); ax[1].set_ylabel("height (m)"); ax[1].grid(lw=0.3); ax[1].set_aspect("equal")
    fig.suptitle("Folded dart at the real launcher speed, 5 deg + 1 rad/s nose-up disturbance at the muzzle", fontsize=9)
    fig.tight_layout(); fig.savefig(os.path.join(PK, "flight_sim.png"), dpi=150)
    return R


def run_yaw(b, v0, T_of_t, tf, dt=2e-4):
    """Horizontal plane: same equations without gravity, 5 deg + 1 rad/s yaw disturbance, for the flight time."""
    psi = math.radians(5.0); w = 1.0; v = np.array([v0, 0.0]); t = 0.0; out = []; ds = 0.001
    while t < tf:
        e = np.array([math.cos(psi), math.sin(psi)]); n = np.array([-math.sin(psi), math.cos(psi)])
        U = v @ e; vn0 = v @ n; vn = vn0 + w * b.s
        F_cf = -0.5 * RHO * CDC * ETA * b.w * np.abs(vn) * vn * ds
        dSp = np.maximum(b.dS, 0) if U >= 0 else np.maximum(-b.dS, 0)
        F_pot = -RHO * abs(U) * dSp * vn * ds
        Fn = (F_cf + F_pot).sum(); Mn = ((F_cf + F_pot) * b.s).sum()
        F = -0.5 * RHO * b.cda * abs(U) * U * e + Fn * n
        if T_of_t is not None:
            T = T_of_t(t); u = -v / np.linalg.norm(v); F = F + T * u; r = b.s_tail * e; Mn += r[0] * T * u[1] - r[1] * T * u[0]
        v = v + F / b.M * dt; w += Mn / b.I * dt; psi += w * dt; t += dt
        if not out or t - out[-1][0] >= 0.005:
            out.append((t, 0, 0, math.degrees(math.atan2(vn0, U)), math.degrees(psi), np.linalg.norm(v)))
    return np.array(out)


def yaw_summary(o):
    late = o[o[:, 0] > 0.15]
    return dict(sideslip_end_deg=round(float(o[-1, 3]), 1), sideslip_peak_after_0p15s_deg=round(float(np.abs(late[:, 3]).max()), 1),
                nose_first=bool(np.abs(o[:, 3]).max() < 45))


if __name__ == "__main__":
    OUT = sys.argv[1] if len(sys.argv) > 1 else dm.out_dir()
    PK = sys.argv[2] if len(sys.argv) > 2 else os.path.join(OUT, "phys31")
    os.makedirs(PK, exist_ok=True)
    R = main(OUT, PK)
    for k, v in R.items():
        if isinstance(v, dict):
            print(k, v)
