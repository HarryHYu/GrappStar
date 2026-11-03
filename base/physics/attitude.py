"""Pitch-plane attitude of the KEDGE v30 dart launched by the base station (3-DOF rigid body: x, z, theta).

Same physics as kedge/flight_sim.py: slender-body cross-flow normal force from the real side-view width profile,
axial drag, gravity, tether tension at the tail along the reverse flight path (reel brake + a share of line drag).
Mass, CG, Iyy and the profile come from data/dart_v30_profile.json (extracted once from kedge/out30 with the
kedge/physics.py logic). numpy only at runtime; matplotlib only for the CLI plot.

Angles: theta = body pitch above horizontal, gamma = flight path angle, alpha = theta - gamma (+ = nose above the path).
Runs are batched: every case integrates in one vectorised loop.

  python attitude.py   -> table, base/out/physics/attitude.json, base/out/physics/attitude.png
"""
import itertools
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "out", "physics")
D = json.load(open(os.path.join(HERE, "data", "dart_v30_profile.json")))

G = 9.81
RHO, CDC, CDAX = 1.2, 1.2, 0.9                    # as kedge/flight_sim.py
M0, CGX0, I0 = D["mass_kg"], D["cg_x_m"], D["Iyy_kg_m2"]
XS = np.array(D["xs_m"])
W = np.array(D["side_width_m"])
DS = D["station_dx_m"]
A_FRONT = math.pi * D["R_OUT_m"] ** 2 * 1.9       # folded legs roughly double the frontal area
X_TAIL = D["L_m"]
X_NOSE = 0.0
BALLAST_X = 0.010                                  # nose cavity
LINE_D, LINE_CD, LINE_SHARE = 0.001, 1.2, 0.3      # line drag share felt at the dart (kedge)
MUZZLE_Z = 0.35
TUMBLE_DEG = 90.0


def mass_props(ballast_g):
    """Mass, CG x (from nose tip, m) and Iyy about the CG with a point ballast at the nose."""
    mb = ballast_g / 1000.0
    m = M0 + mb
    cgx = (M0 * CGX0 + mb * BALLAST_X) / m
    iyy = I0 + M0 * (CGX0 - cgx) ** 2 + mb * (BALLAST_X - cgx) ** 2
    return m, cgx, iyy


def _col(v, n):
    return np.broadcast_to(np.asarray(v, float), (n,)).copy()


def simulate(v0, pitch_deg, tether_N, d_theta_deg=0.0, d_rate=0.0, ballast_g=15.0, z0=MUZZLE_Z,
             tmax=4.0, dt=2e-4, aero=True, tether_on=None, rec_dt=0.005, ground_ends=True):
    """Batched run. Scalars or equal-length arrays. tether_N 0 means no tether unless tether_on says otherwise.
    Stops each case when the nose or tail point reaches z = 0 (ground_ends) or the CG passes z = -0.01 (kedge rule). Returns a dict of per-case arrays plus traces."""
    args = [np.atleast_1d(np.asarray(a, float)) for a in (v0, pitch_deg, tether_N, d_theta_deg, d_rate, ballast_g)]
    n = max(len(a) for a in args)
    v0, pitch, T0, dth, drate, bal = (_col(a, n) for a in args)
    teth = T0 > 0 if tether_on is None else _col(tether_on, n).astype(bool)

    m, cgx, iyy = mass_props(bal)
    s_st = cgx[:, None] - XS[None, :]               # station distance from CG, + toward the nose
    s_tail, s_nose = cgx - X_TAIL, cgx - X_NOSE
    wds = W[None, :] * DS

    g0 = np.radians(pitch)
    th = g0 + np.radians(dth)
    vx, vz = v0 * np.cos(g0), v0 * np.sin(g0)
    x = np.zeros(n)
    z = np.full(n, float(z0))
    w = drate.copy()
    L_out = np.zeros(n)
    live = np.ones(n, bool)
    t = 0.0
    tumbled = np.zeros(n, bool)
    max_a_late = np.zeros(n)
    imp = {k: np.full(n, np.nan) for k in ("t", "x", "theta", "alpha", "v", "gamma")}
    nose_first = np.zeros(n, bool)
    rec_t, rec_a, rec_th = [], [], []
    next_rec = 0.0

    def alpha_of(th_, vx_, vz_):
        return (np.degrees(th_ - np.arctan2(vz_, vx_)) + 180.0) % 360.0 - 180.0

    while t < tmax and live.any():
        c, s = np.cos(th), np.sin(th)
        va = vx * c + vz * s
        vn0 = -vx * s + vz * c
        Fx = np.zeros(n)
        Fz = -G * m
        Mom = np.zeros(n)
        if aero:
            vn = vn0[:, None] + w[:, None] * s_st      # local normal velocity per station
            dFn = -0.5 * RHO * CDC * wds * np.abs(vn) * vn
            Fn = dFn.sum(1)
            Mom = (dFn * s_st).sum(1)                   # + = nose up
            Fa = -0.5 * RHO * CDAX * A_FRONT * np.abs(va) * va
            Fx = Fx + Fa * c - Fn * s
            Fz = Fz + Fa * s + Fn * c
        sp = np.hypot(vx, vz)
        L_out = L_out + np.where(teth, sp * dt, 0.0)
        T = np.where(teth, T0 + 0.5 * RHO * LINE_CD * LINE_D * L_out * sp ** 2 * LINE_SHARE, 0.0)
        ux, uz = -vx / np.maximum(sp, 1e-6), -vz / np.maximum(sp, 1e-6)
        Fx = Fx + T * ux
        Fz = Fz + T * uz
        rx, rz = s_tail * c, s_tail * s                 # tail point relative to CG
        Mom = Mom + (rx * T * uz - rz * T * ux)         # r x F, + = nose up (as kedge)

        # semi-implicit Euler like kedge, frozen once landed
        k = live.astype(float)
        vx = vx + k * Fx / m * dt
        vz = vz + k * Fz / m * dt
        x = x + k * vx * dt
        z = z + k * vz * dt
        w = w + k * Mom / iyy * dt
        th = th + k * w * dt
        t += dt

        a = alpha_of(th, vx, vz)
        tumbled |= live & (np.abs(a) > TUMBLE_DEG)
        if t > 0.3:
            max_a_late = np.where(live, np.maximum(max_a_late, np.abs(a)), max_a_late)

        zn = z + s_nose * np.sin(th)
        zt = z + s_tail * np.sin(th)
        hit = live & ((np.minimum(zn, zt) <= 0.0) if ground_ends else (z < -0.01))
        if hit.any():
            imp["t"][hit] = t
            imp["x"][hit] = x[hit]
            imp["theta"][hit] = np.degrees(th[hit])
            imp["alpha"][hit] = a[hit]
            imp["v"][hit] = sp[hit]
            imp["gamma"][hit] = np.degrees(np.arctan2(vz[hit], vx[hit]))
            nose_first[hit] = zn[hit] <= zt[hit]
            live &= ~hit
        if t >= next_rec:
            rec_t.append(t)
            rec_a.append(np.where(live | hit, a, np.nan))
            rec_th.append(np.degrees(th))
            next_rec += rec_dt

    return dict(n=n, v0=v0, pitch=pitch, tether_N=np.where(teth, T0, 0.0), d_theta=dth, d_rate=drate, ballast_g=bal,
                tumbled=tumbled, max_alpha_after_0p3s=max_a_late, landed=~np.isnan(imp["t"]),
                impact=imp, nose_first=nose_first, theta_end=np.degrees(th), w_end=w,
                trace_t=np.array(rec_t), trace_alpha=np.array(rec_a), trace_theta=np.array(rec_th))


# ---------------- matrix
V0S = (6.0, 7.0, 8.0)
PITCHES = (10.0, 25.0, 36.0, 47.0)
TETHERS = (0.0, 0.5, 0.75, 1.0)
DIST = [(a, r) for a in (0.0, 10.0, -10.0) for r in (0.0, 5.0, -5.0)]   # (deg, rad/s)
BALLASTS = (15.0, 0.0)


def run_matrix():
    combos = list(itertools.product(BALLASTS, V0S, PITCHES, TETHERS, DIST))
    b, v, p, te, d = zip(*combos)
    da, dr = zip(*d)
    return simulate(v, p, te, da, dr, b)


def rows_of(r):
    out = []
    for i in range(r["n"]):
        out.append(dict(ballast_g=float(r["ballast_g"][i]), v0=float(r["v0"][i]), pitch_deg=float(r["pitch"][i]),
                        tether_N=float(r["tether_N"][i]), d_theta_deg=float(r["d_theta"][i]), d_rate=float(r["d_rate"][i]),
                        tumbled=bool(r["tumbled"][i]), max_alpha_after_0p3s=round(float(r["max_alpha_after_0p3s"][i]), 1),
                        t_impact=round(float(r["impact"]["t"][i]), 3), range_m=round(float(r["impact"]["x"][i]), 2),
                        theta_impact_deg=round(float(r["impact"]["theta"][i]), 1),
                        alpha_impact_deg=round(float(r["impact"]["alpha"][i]), 1),
                        v_impact=round(float(r["impact"]["v"][i]), 2), nose_first=bool(r["nose_first"][i])))
    return out


def summarise(rows):
    """Group over the 9 disturbances: one line per (ballast, v0, pitch, tether)."""
    grp = {}
    for q in rows:
        grp.setdefault((q["ballast_g"], q["v0"], q["pitch_deg"], q["tether_N"]), []).append(q)
    out = []
    for (b, v, p, te), qs in grp.items():
        th = [q["theta_impact_deg"] for q in qs]
        al = [q["alpha_impact_deg"] for q in qs]
        out.append(dict(ballast_g=b, v0=v, pitch_deg=p, tether_N=te, n=len(qs),
                        tumbles=sum(q["tumbled"] for q in qs), nose_first=sum(q["nose_first"] for q in qs),
                        max_alpha_after_0p3s=max(q["max_alpha_after_0p3s"] for q in qs),
                        theta_impact=[min(th), max(th)], alpha_impact=[min(al), max(al)],
                        t_flight=[min(q["t_impact"] for q in qs), max(q["t_impact"] for q in qs)],
                        range_m=[min(q["range_m"] for q in qs), max(q["range_m"] for q in qs)]))
    return out


def plot(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cols = {0.0: "#2a78d6", 0.5: "#eb6834", 1.0: "#1baf7a"}
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.8), sharey=True)
    for ax, bal in zip(axs, (15.0, 0.0)):
        for te, cl in cols.items():
            for dth, dr, ls in ((10.0, 5.0, "-"), (-10.0, -5.0, "--")):
                r = simulate(7.0, 36.0, te, dth, dr, bal)
                lab = "no tether" if te == 0 else "%.1f N brake" % te
                ax.plot(r["trace_t"], r["trace_alpha"][:, 0], color=cl, lw=1.6, ls=ls,
                        label="%s, %+d deg %+d rad/s" % (lab, dth, dr))
        ax.axhline(0, color="#52514e", lw=0.6)
        for yy in (90, -90):
            ax.axhline(yy, color="#52514e", lw=0.5, ls=":")
        ax.set_title("%d g nose ballast" % bal if bal else "no ballast", fontsize=9, color="#0b0b0b")
        ax.set_xlabel("time s", color="#52514e")
        ax.grid(lw=0.3, color="#e0dfdb")
        for sp_ in ax.spines.values():
            sp_.set_color("#c3c2b7")
    axs[0].set_ylabel("angle of attack deg (+ nose above path)", color="#52514e")
    axs[0].set_ylim(-185, 185)
    axs[1].legend(fontsize=6.5, frameon=False, loc="lower right")
    fig.suptitle("KEDGE v30, 7 m/s at 36 deg from 0.35 m, to ground contact", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    r = run_matrix()
    rows = rows_of(r)
    summ = summarise(rows)
    print("bal  v0  pitch  T_N  tumb  nose1st  max|a|>0.3s  theta_imp      alpha_imp      range_m")
    for s in summ:
        print("%3d %4.0f %5.0f %5.2f %3d/%d %4d/%d %9.1f   %6.1f..%-6.1f %6.1f..%-6.1f %4.1f..%.1f" % (
            s["ballast_g"], s["v0"], s["pitch_deg"], s["tether_N"], s["tumbles"], s["n"], s["nose_first"], s["n"],
            s["max_alpha_after_0p3s"], s["theta_impact"][0], s["theta_impact"][1], s["alpha_impact"][0],
            s["alpha_impact"][1], s["range_m"][0], s["range_m"][1]))
    verdict = {}
    for bal in BALLASTS:
        for tag, sel in (("tethered_0p5_to_1N", lambda q: q["tether_N"] > 0), ("no_tether", lambda q: q["tether_N"] == 0)):
            qs = [q for q in rows if q["ballast_g"] == bal and sel(q)]
            nf = [q for q in qs if q["nose_first"] and not q["tumbled"]]
            verdict["%dg_%s" % (bal, tag)] = dict(
                cases=len(qs), tumbles=sum(q["tumbled"] for q in qs), nose_first=sum(q["nose_first"] for q in qs),
                theta_impact_nose_first=[min(q["theta_impact_deg"] for q in nf), max(q["theta_impact_deg"] for q in nf)] if nf else None,
                alpha_impact_nose_first=[min(q["alpha_impact_deg"] for q in nf), max(q["alpha_impact_deg"] for q in nf)] if nf else None)
    print(json.dumps(verdict, indent=1))
    m15, c15, i15 = mass_props(15.0)
    res = dict(inputs=dict(mass_kg_no_ballast=round(M0, 4), cg_x_mm_no_ballast=round(CGX0 * 1000, 1),
                           Iyy_no_ballast=float("%.4g" % I0), mass_kg_15g=round(m15, 4), cg_x_mm_15g=round(c15 * 1000, 1),
                           Iyy_15g=float("%.4g" % i15), cross_flow_cd=CDC, axial_cd=CDAX, muzzle_z_m=MUZZLE_Z,
                           alpha_sign="theta - gamma, + nose above path", tumble="any |alpha| > 90 deg before ground contact",
                           impact="first of nose tip or tail point reaching z = 0 (body as a line on its axis)"),
               verdict=verdict, summary=summ, cases=rows)
    json.dump(res, open(os.path.join(OUT_DIR, "attitude.json"), "w"), indent=1)
    plot(os.path.join(OUT_DIR, "attitude.png"))
    print("wrote", os.path.normpath(os.path.join(OUT_DIR, "attitude.json")), "and attitude.png")


if __name__ == "__main__":
    main()
