"""Pitch-plane flight simulation of the folded dart: rigid body, slender-body cross-flow normal force from the real side
profile, axial drag, gravity, and the tether tension at the tail. Launch at 45 deg with a nose-up disturbance; compare
with and without the tether.  python3 flight_sim.py out29 pack_v29"""
import sys, json, math, io, contextlib
import numpy as np
OUT, PK = sys.argv[1], sys.argv[2]
sys.argv = ["physics.py", OUT]
with contextlib.redirect_stdout(io.StringIO()):
    import physics as ph                      # reuses the mass model, the folded assembly and the side profile
M = ph.M / 1000.0; cgx = ph.cg[0] / 1000.0; Iyy = ph.I[1, 1] * 1e-9   # kg, m, kg m^2  (I in g mm^2 -> kg m^2)
xs = ph.xs / 1000.0; width = ph.side / 1000.0                          # side-view width per 1 mm station (m)
s_st = cgx - xs                                                         # station distance from CG, + toward the nose
rho, CDC, CDAX = 1.2, 1.2, 0.9
A_front = math.pi * (ph.P["R_OUT"] / 1000) ** 2 * 1.9                   # folded legs roughly double the frontal area
x_tail = ph.P["L"] / 1000.0; s_tail = cgx - x_tail
DS = 0.001
def run(tether, v0=18.0, elev=45.0, dtheta=10.0, T0=1.0, tmax=4.0, dt=2e-4, ballast_g=0.0):
    global M, cgx, Iyy, s_st, s_tail
    M0, c0, I0 = ph.M / 1000.0, ph.cg[0] / 1000.0, ph.I[1, 1] * 1e-9
    mb = ballast_g / 1000.0; xb = 0.010                                  # ballast in the nose cavity at x = 10 mm
    M = M0 + mb; cgx = (M0 * c0 + mb * xb) / M; Iyy = I0 + M0 * (c0 - cgx) ** 2 + mb * (xb - cgx) ** 2
    s_st = cgx - xs; s_tail = cgx - x_tail
    th = math.radians(elev + dtheta); vx, vz = v0 * math.cos(math.radians(elev)), v0 * math.sin(math.radians(elev))
    x = z = 0.0; w = 0.0; L_out = 0.0; out = []
    t = 0.0
    while t < tmax and z >= -0.01:
        e = np.array([math.cos(th), math.sin(th)]); n = np.array([-math.sin(th), math.cos(th)])
        v = np.array([vx, vz]); va = v @ e; vn0 = v @ n
        vn = vn0 + w * s_st                                              # local normal velocity at each station
        dFn = -0.5 * rho * CDC * width * np.abs(vn) * vn * DS            # cross-flow normal force per station
        Fn = dFn.sum(); Mn = (dFn * s_st).sum()                          # moment about CG (+ = nose up, about +y out of plane)
        Fa = -0.5 * rho * CDAX * A_front * abs(va) * va
        F = Fa * e + Fn * n + np.array([0, -9.81 * M]); Mom = Mn
        if tether:
            sp = np.linalg.norm(v); L_out += sp * dt
            T = T0 + 0.5 * rho * 1.2 * 0.001 * L_out * sp ** 2 * 0.3       # reel brake + a share of the line's own drag
            u = -v / max(sp, 1e-6)                                        # line trails behind along the flight path
            F = F + T * u
            r = s_tail * e                                                # tail point relative to CG
            Mom += r[0] * (T * u[1]) - r[1] * (T * u[0])
        vx += F[0] / M * dt; vz += F[1] / M * dt; x += vx * dt; z += vz * dt
        w += Mom / Iyy * dt; th += w * dt; t += dt
        alpha = math.degrees(math.atan2(vn0, va))
        if len(out) == 0 or t - out[-1][0] >= 0.005: out.append((t, x, z, alpha, math.degrees(th)))
    return np.array(out)
R = {}
fig_data = {}
for tag, teth, T0, bal in (("no tether", False, 0.0, 0), ("no tether, 8 g nose ballast", False, 0.0, 8), ("no tether, 15 g nose ballast", False, 0.0, 15), ("tether, 0.5 N reel brake", True, 0.5, 0),
                           ("tether, 1 N reel brake", True, 1.0, 0), ("tether, 3 N reel brake", True, 3.0, 0), ("tether 1 N + 15 g ballast (recommended)", True, 1.0, 15)):
    o = run(teth, T0=T0, ballast_g=bal); fig_data[tag] = o
    late = o[o[:, 0] > 0.6]
    R[tag] = dict(max_alpha_after_0p6s_deg=round(float(np.abs(late[:, 3]).max()) if len(late) else None, 1), final_alpha_deg=round(float(o[-1, 3]), 1),
                  range_m=round(float(o[-1, 1]), 1), tumbled=bool((np.abs(o[:, 3]) > 90).any()))
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
for tag, o in fig_data.items():
    ax[0].plot(o[:, 0], np.clip(o[:, 3], -180, 180), label=tag); ax[1].plot(o[:, 1], o[:, 2], label=tag)
ax[0].set_xlabel("time s"); ax[0].set_ylabel("angle of attack deg"); ax[0].axhline(0, color="k", lw=0.5); ax[0].grid(lw=0.3); ax[0].legend(fontsize=7)
ax[1].set_xlabel("x m"); ax[1].set_ylabel("height m"); ax[1].grid(lw=0.3); ax[1].set_aspect("equal")
fig.suptitle("Folded dart, launch 18 m/s at 45 deg with a 10 deg nose-up kick, to landing", fontsize=9)
fig.tight_layout(); fig.savefig(PK + "/flight_sim.png", dpi=150)
R["inputs"] = dict(mass_kg=round(M, 3), cg_x_mm=round(cgx * 1000, 1), Iyy_kg_m2=float(f"{Iyy:.3e}"), cross_flow_cd=CDC, axial_cd=CDAX)
json.dump(R, open(OUT + "/flight_sim.json", "w"), indent=1); print(json.dumps(R, indent=1))
