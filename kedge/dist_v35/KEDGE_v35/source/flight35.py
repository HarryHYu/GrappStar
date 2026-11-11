"""KEDGE v35 flight check (11 Nov 2026): flight34.py's model unchanged, run on the folded out35 geometry (pods hinged at x 50, folded
aft along the flanks) with the line tied in the tail face (x 152, z -2.4, on the hull), against the final v34 (out34, same tail tie).
  .venv/bin/python flight35.py            -> phys35/flight35.json

The flight34.py notes follow (v34 check with the bridle, dart-physics, 9 Nov 2026). Planar rigid-body flight (pitch plane with gravity, yaw plane
without), the strip aero model of flight_sim.py unchanged (viscous cross-flow Cdc 1.2 x eta 0.65 on the top-view width for pitch /
side-view height for yaw, slender-body term rho U S'(x) where the section grows, axial CdA 2.6e-3), applied to the folded out34
geometry (dart_model profiles; dart_model now reads parts.json "joint" / "sgn"). Mass, CG and inertia from mass34.py
(out34/mass_props.json; with / without putty, bought masses +-10 %).

Tether: payout drag T (0.25 N) along the trailing line (-v). Tow models:
  bridle   v34: legs L (78 mm) from the eye lugs (128.1, +-31.65, 2.7 folded) to a sliding loop. The loop sits on the spheroid with the
           eyes as foci where its normal points along the line, and the pull acts through the loop along the line. Pitch plane: the
           section is a circle round the eye midpoint, so the line of action always passes through the eye axis (x 128.1, z 2.7).
           Yaw plane: an ellipse (semi-axes L across, sqrt(L^2 - 31.65^2) along), so small yaw turns pull through its centre of
           curvature, 14 mm in front of the eye midpoint.
  knot     the same legs tied to a FIXED apex knot: in pitch the triangle hinges about the eye axis (same as the loop); in yaw the apex
           is a fixed tow point 71 mm behind the eyes while both legs stay taut (line within +-23.9 deg), else the pull goes
           through the one taut leg's eye.
  wrapped  the straight legs run about 1 mm into the arm heads and the pivot blocks' aft corners, so the taut legs leave the body at
           those corners (WRAP): the same two models with the eyes moved there; the real bridle is between the two.
  point    a single eye at (x, z) on the body (v31-style tail knot x 152, v32c boom eye x 179.5).
Launch: v0 (6.8 m/s) at the rail pitch, from the base launcher's release point; at the muzzle either flight_sim.py's 5 deg + 1 rad/s
or the cradle tip-off alone (0.4 rad/s), nose up or down (yaw: left or right). The yaw plane runs at v0 for the pitch flight's time.
Check: on out32 with no tether it reproduces out32/flight_sim.json's v32c pitch case to the digit.

"""
import json, math, os, sys
import numpy as np
import dart_model as dm
import mass34

HERE = os.path.dirname(os.path.abspath(__file__))
CDC, ETA, RHO, CDA = 1.2, 0.65, dm.RHO_AIR, 2.6e-3
EYE = dict(x=128.1, y=31.65, z=2.7)        # REVIEW_v34 section 3: P.DRILL_X - P.EYE_FWD, lug centre, pivot z 9.5 - P.EYE_DROP
LEG = 78.0
# Folded, the straight legs run 1 mm into the arm head's rear (x 130-133) and the pivot block's aft corner (hull_top, x 137.4-140.5):
# the taut leg leaves the body at that corner, so for small turns it pulls as if tied there (free leg 78 - 12.5 = 65.5 mm).
WRAP = dict(x=139.5, y=26.6, z=2.7, leg=65.5)
DIST = {"5 deg + 1 rad/s nose up": (5.0, 1.0), "5 deg + 1 rad/s nose down": (-5.0, -1.0),     # flight_sim.py's disturbance
        "tip-off 0.4 rad/s nose up": (0.0, 0.4), "tip-off 0.4 rad/s nose down": (0.0, -0.4)}  # cradle tip-off only


class Air:
    """Strip-model arrays of a folded dart (per 1 mm station)."""
    def __init__(self, out):
        d = dm.Dart(out)
        pr = d.profiles()
        self.xs = pr["xs"] / 1000; self.top = pr["top"] / 1000; self.side = pr["side"] / 1000
        S = pr["S"] * 1e-6
        self.dS = np.gradient(S, self.xs); self.S = S
        self.D = 2 * d.P["R_OUT"] / 1000; self.out = out; self.d = d


class Body:
    def __init__(self, air, M_kg, cg_mm, I_kgm2, plane):
        self.M = M_kg; self.cgx = cg_mm[0] / 1000; self.cgz = cg_mm[2] / 1000; self.I = I_kgm2
        self.w = air.top if plane == "pitch" else air.side
        self.s = self.cgx - air.xs; self.dS = air.dS; self.plane = plane


def tow_r(tow, b, u, e, n):
    """World 2D vector from the CG to a point on the line of action of the tow (which acts along u)."""
    if tow is None:
        return None
    k = tow["kind"]
    if k == "point":
        return (b.cgx - tow["x"] / 1000) * e + ((tow["z"] / 1000 - b.cgz) if b.plane == "pitch" else 0.0) * n
    ey = tow.get("eye", EYE)
    smid = b.cgx - ey["x"] / 1000
    hmid = (ey["z"] / 1000 - b.cgz) if b.plane == "pitch" else 0.0
    L = tow.get("leg", LEG) / 1000; c = ey["y"] / 1000; bb = math.sqrt(L * L - c * c)
    if b.plane == "pitch" or k == "bridle":
        aN = bb if b.plane == "pitch" else L                # semi-axis across the body axis in this plane
        ue, un = u @ e, u @ n
        off = (bb * bb * ue * e + aN * aN * un * n) / math.sqrt(bb * bb * ue * ue + aN * aN * un * un)
        return smid * e + hmid * n + off
    # fixed knot, yaw plane: apex fixed while the line is inside the leg angle, else the taut leg's eye
    half = math.atan2(c, bb)
    ang = math.atan2(u @ n, -(u @ e))
    if abs(ang) <= half:
        return (smid - bb) * e
    return smid * e + math.copysign(c, ang) * n


def forces(b, v, w, th):
    e = np.array([math.cos(th), math.sin(th)]); n = np.array([-math.sin(th), math.cos(th)])
    U = v @ e; vn0 = v @ n; vn = vn0 + w * b.s
    F_cf = -0.5 * RHO * CDC * ETA * b.w * np.abs(vn) * vn * 0.001
    dSp = np.maximum(b.dS, 0) if U >= 0 else np.maximum(-b.dS, 0)
    F_pot = -RHO * abs(U) * dSp * vn * 0.001
    Fn = (F_cf + F_pot).sum(); Mn = ((F_cf + F_pot) * b.s).sum()
    F = -0.5 * RHO * CDA * abs(U) * U * e + Fn * n
    return F, Mn, e, n, U, vn0


def fly(b, v0, elev, T, tow, dth=5.0, w0=1.0, h0=0.42, tmax=3.0, dt=2e-4, tf=None):
    """Pitch plane (gravity, until z = 0) or yaw plane (no gravity, for tf). th = body axis angle (pitch: above horizontal)."""
    pitch = b.plane == "pitch"
    th = math.radians((elev if pitch else 0.0) + dth); w = w0
    v = np.array([v0 * math.cos(math.radians(elev)), v0 * math.sin(math.radians(elev))]) if pitch else np.array([v0, 0.0])
    x, z, t, out = 0.0, h0, 0.0, []
    while (t < tmax and z > 0.0) if pitch else (t < tf):
        F, Mn, e, n, U, vn0 = forces(b, v, w, th)
        if pitch:
            F = F + np.array([0.0, -dm.G * b.M])
        if T and tow is not None:
            u = -v / max(np.linalg.norm(v), 1e-9)
            r = tow_r(tow, b, u, e, n)
            F = F + T * u; Mn += r[0] * T * u[1] - r[1] * T * u[0]
        v = v + F / b.M * dt; x += v[0] * dt; z += v[1] * dt if pitch else 0.0
        w += Mn / b.I * dt; th += w * dt; t += dt
        if not out or t - out[-1][0] >= 0.005:
            out.append((t, x, z, math.degrees(math.atan2(vn0, U)), math.degrees(th), float(np.linalg.norm(v)), w))
    return np.array(out)


def summary_pitch(o):
    late = o[o[:, 0] > 0.15]
    path = math.degrees(math.atan2(-(o[-1, 2] - o[-2, 2]), o[-1, 1] - o[-2, 1]))
    return dict(range_m=round(float(o[-1, 1]), 2), t_flight_s=round(float(o[-1, 0]), 3), v_impact=round(float(o[-1, 5]), 2),
                path_deg_down=round(path, 1), body_pitch_at_impact_deg=round(float(o[-1, 4]), 1), aoa_impact_deg=round(float(o[-1, 3]), 1),
                aoa_peak_after_0p15s_deg=round(float(np.abs(late[:, 3]).max()), 1) if len(late) else None,
                aoa_max_deg=round(float(np.abs(o[:, 3]).max()), 1), nose_first=bool(np.abs(o[:, 3]).max() < 45))


def summary_yaw(o):
    late = o[o[:, 0] > 0.15]
    return dict(sideslip_impact_deg=round(float(o[-1, 3]), 1), sideslip_peak_after_0p15s_deg=round(float(np.abs(late[:, 3]).max()), 1),
                sideslip_max_deg=round(float(np.abs(o[:, 3]).max()), 1), nose_first=bool(np.abs(o[:, 3]).max() < 45))


def linear(air, b, tow_arm_m, T, U=7.0, alpha_deg=10.0):
    """Aero centre (alpha -> 0: slender-body term only; at 10 deg as physics.py), planform centroid, neutral speed with the tow."""
    s = b.s; dSp = np.maximum(b.dS, 0)
    K = RHO * (dSp * s).sum() * 0.001                          # dM/dalpha per U^2 (nose-up positive = destabilizing)
    Kf = RHO * dSp.sum() * 0.001                               # dFn/dalpha per U^2
    ac0 = b.cgx - K / Kf
    a = math.radians(alpha_deg); vn = -U * math.sin(a)
    Fs = -0.5 * RHO * CDC * ETA * b.w * abs(vn) * vn * 0.001 - RHO * U * dSp * vn * 0.001
    ac10 = float((Fs * air.xs).sum() / Fs.sum())
    plan = float((b.w * air.xs).sum() / b.w.sum())
    Ustar = math.sqrt(T * tow_arm_m / K) if K > 0 and tow_arm_m > 0 else None
    ac0 = float(ac0)
    return dict(ac_alpha0_x_mm=round(ac0 * 1000, 1), ac_10deg_x_mm=round(ac10 * 1000, 1), planform_centroid_x_mm=round(plan * 1000, 1),
                static_margin_10deg_cal=round(float((ac10 - b.cgx) / air.D), 2), aero_dMdalpha_at_7ms_Nm_per_rad=round(float(K * 49), 5),
                tow_dMdalpha_Nm_per_rad=round(float(-T * tow_arm_m), 5), tow_arm_mm=round(float(tow_arm_m * 1000), 1),
                neutral_speed_m_s=round(Ustar, 2) if Ustar else None)


def launch_h(elev):
    p = dm.base_params(); L = p.launch; th = math.radians(elev)
    return L["pivot_h"] + L["release_u"] * math.sin(th) + L["dart_w"] * math.cos(th)


def main(res):
    R = dict(what=__doc__.split("\n\n")[0], v0=6.8, tether_N=0.25, elevations=[6, 10, 15, 20, 25, 30, 34, 40, 47],
             conventions="aoa / sideslip = angle of the velocity from the body axis, + when the nose is below (pitch) the flight path; "
                         "body_pitch_at_impact = nose direction below (-) the horizon; path_deg_down = flight path below the horizon at impact; "
                         "nose_first = |aoa| (or |sideslip|) under 45 deg for the whole flight (flight_sim.py's test)")
    builds = {}
    for tag, out in (("v35", "out35"), ("v34", "out34")):
        rows = mass34.load(os.path.join(HERE, out))[0]
        air = Air(os.path.join(HERE, out))
        C = mass34.cases(rows)
        keys = ("putty, bought x1.0", "no putty, bought x1.0", "putty, bought x fwd", "putty, bought x aft", "no putty, bought z up") if tag == "v35" else ("putty, bought x1.0",)
        for k in keys:
            cc = C[k]; builds[f"{tag} {k}"] = (air, cc["mass_g"] / 1000, cc["cg_mm"], cc["I_gmm2"] * 1e-9)
    TIE = dict(kind="point", x=152.0, z=-2.4)          # the two holes in the tail face (P.TETHER "tail"), unchanged from v34
    tows = {"tail tie x 152 z -2.4": TIE}
    plan = [(k, "tail tie x 152 z -2.4") for k in builds]
    R["inputs"] = {k: dict(source=v[0].out, mass_g=round(v[1] * 1000, 1), cg_mm=[round(float(x), 2) for x in v[2]],
                           Iyy_kgm2=float("%.4g" % v[3][1, 1]), Izz_kgm2=float("%.4g" % v[3][2, 2])) for k, v in builds.items()}
    R["cases"] = {}
    for bk, tk in plan + [("v35 putty, bought x1.0", None)]:
        air, M, cg, I = builds[bk]; tow = tows[tk] if tk else None
        key = f"{bk} | {tk or 'no tether'}"
        E = {}
        for plane in ("pitch", "yaw"):
            b = Body(air, M, cg, I[1, 1] if plane == "pitch" else I[2, 2], plane)
            if tow is None: arm = 0.0
            elif tow["kind"] == "point": arm = tow["x"] / 1000 - b.cgx
            else:
                ey = tow.get("eye", EYE); Lw = tow.get("leg", LEG) / 1000; cw = ey["y"] / 1000; bw = math.sqrt(Lw * Lw - cw * cw)
                if plane == "pitch": arm = ey["x"] / 1000 - b.cgx
                elif tow["kind"] == "bridle": arm = ey["x"] / 1000 - b.cgx - cw * cw / bw
                else: arm = ey["x"] / 1000 - b.cgx + bw
            E[plane] = dict(linear=linear(air, b, arm, 0.25))
        bp = Body(air, M, cg, I[1, 1], "pitch"); by = Body(air, M, cg, I[2, 2], "yaw")
        for elev in R["elevations"]:
            for dn, (dth, w0) in DIST.items():
                o = fly(bp, 6.8, elev, 0.25, tow, dth=dth, w0=w0, h0=launch_h(elev))
                sp = summary_pitch(o)
                oy = fly(by, 6.8, elev, 0.25, tow, dth=dth, w0=w0, tf=float(o[-1, 0]))
                sp.update({"yaw_" + k: v for k, v in summary_yaw(oy).items()})
                E[f"rail {elev} deg, {dn}"] = sp
        R["cases"][key] = E
    os.makedirs(res, exist_ok=True)
    json.dump(R, open(os.path.join(res, "flight35.json"), "w"), indent=1)
    return R


if __name__ == "__main__":
    R = main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "phys35"))
    for k, E in R["cases"].items():
        print(k, "| pitch", E["pitch"]["linear"], "| yaw", E["yaw"]["linear"])
        for e in (6, 20, 34, 47):
            for dd in DIST:
                s = E[f"rail {e} deg, {dd}"]
                print(f"   rail {e:2d} {dd:28s} nose_first {s['nose_first']!s:5s} yaw_nf {s['yaw_nose_first']!s:5s} aoa_peak {s['aoa_peak_after_0p15s_deg']} "
                      f"aoa_imp {s['aoa_impact_deg']} pitch_imp {s['body_pitch_at_impact_deg']} path {s['path_deg_down']} v {s['v_impact']} range {s['range_m']} "
                      f"yaw_peak {s['yaw_sideslip_peak_after_0p15s_deg']} yaw_imp {s['yaw_sideslip_impact_deg']}")
