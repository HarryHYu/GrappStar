"""KEDGE v31 landing: impact loads by surface, rest orientation on flat ground, can the legs recover a bad landing, and
what the BNO085 sees.  python dart_landing.py [OUT] -> OUT/phys31/landing.json

Rest states (quasi-static, flat hard ground): for a body-frame "down" direction d the CG height above the ground is
h(d) = max_p (p . d) - cg . d over the convex hull points p. Stable rest states are local minima of h on the sphere
(Fibonacci grid, steepest descent over the 12 nearest neighbours); the basin of each minimum is the share of
directions that descend into it = its probability for a random final tumble. Recovery: the same descent, re-run while
a leg moves (the support set changes with the leg pose), starting from the rest state.
Impact (nose first, soil): the nose pushes into the surface at the dynamic bearing pressure q over the nose area at
that depth (frustum d22 -> d42 over 20 mm, then the hull); energy of the normal velocity component (effective mass at
the nose contact) sets depth, peak force and peak g. Hard ground: the PLA nose rim crushes at 55 MPa over the
crushed area (sliced from the nose mesh).
"""
import json, math, os, sys
import numpy as np
import trimesh
import dart_model as dm

G = dm.G


def fib_sphere(n=6000):
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n); th = math.pi * (1 + 5 ** 0.5) * i
    return np.c_[np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)]


class Rest:
    def __init__(self, d: dm.Dart, n=4000):
        self.d = d
        self.S = fib_sphere(n)
        from scipy.spatial import cKDTree
        self.nb = cKDTree(self.S).query(self.S, k=13)[1][:, 1:]

    def h(self, pts, cg):
        return (self.S @ pts.T).max(1) - self.S @ cg

    def descend(self, H, i):
        while True:
            j = self.nb[i][np.argmin(H[self.nb[i]])]
            if H[j] >= H[i] - 1e-12:
                return i
            i = j

    def states(self, pts, cg):
        H = self.h(pts, cg)
        lab = np.array([self.descend(H, i) for i in range(len(self.S))])
        out = []
        for m in np.unique(lab):
            dn = self.S[m]
            out.append(dict(idx=int(m), share=float((lab == m).mean()), h_mm=float(H[m]), **attitude(dn)))
        return sorted(out, key=lambda r: -r["share"]), H, lab


def attitude(dn):
    """down direction in the body frame -> roll (0 = belly down) and nose pitch (deg, + = nose up)."""
    up = -dn
    pitch = math.degrees(math.asin(np.clip(-up[0], -1, 1)))     # body -x is forward: nose up when up has a -x part
    roll = math.degrees(math.atan2(up[1], up[2]))               # + = rolled onto the right (-y) side... see note
    return dict(roll_deg=round(roll, 1), pitch_deg=round(pitch, 1))


def roll_slice(pts, cg):
    """CG height vs roll when lying along the axis (d = (0, sin r, -cos r)): minima = rest rolls, maxima = basin edges."""
    r = np.radians(np.arange(-180, 181, 1.0))
    D = np.c_[np.zeros_like(r), np.sin(r), -np.cos(r)]
    h = (D @ pts.T).max(1) - D @ cg
    return np.degrees(r), h


def nose_impact(d, v, path_deg, q_pa, m_eff):
    """Penetration of the nose into a surface of bearing q (Pa), normal speed v sin(path)."""
    vn = v * math.sin(math.radians(path_deg))
    E = 0.5 * m_eff * vn ** 2
    nm = d.mesh("nose")
    # area of the nose cross-section facing the surface vs depth along the flight path (axis at the path angle)
    dep = np.linspace(0, 0.060, 601)
    R0, R1, Ln = d.P["NOSE_R0"] / 1000, d.P["R_OUT"] / 1000, d.P["NOSE_L"] / 1000
    rad = np.where(dep < Ln, R0 + (R1 - R0) * dep / Ln, R1)
    A = math.pi * rad ** 2
    W = np.concatenate([[0], np.cumsum(q_pa * 0.5 * (A[1:] + A[:-1]) * np.diff(dep))])
    k = np.searchsorted(W, E)
    if k >= len(dep):
        return dict(depth_mm=60.0, F_peak_N=float(q_pa * A[-1]), g_peak=float(q_pa * A[-1] / m_eff / G))
    F = q_pa * A[k]
    return dict(vn=round(vn, 2), depth_mm=round(dep[k] * 1000, 1), F_peak_N=round(F, 1), g_peak=round(F / m_eff / G, 1),
                duration_ms=round(2 * dep[k] / max(vn, 1e-6) * 1000, 1))


def hard_impact(d, v, path_deg, m_eff, sigma=55e6):
    """Nose rim crushing on rigid ground: crushed area from the nose mesh slice, energy balance."""
    vn = v * math.sin(math.radians(path_deg))
    E = 0.5 * m_eff * vn ** 2
    nm = d.mesh("nose")
    th = math.radians(path_deg)
    up = np.array([math.sin(th), 0.0, math.cos(th)])       # world up in the body frame (nose first along the path)
    bottom = (nm.vertices @ up).min()
    dep = np.linspace(0.05, 8.0, 160)                     # mm crushed
    A = []
    for dd in dep:
        try:
            s = nm.section(plane_origin=up * (bottom + dd), plane_normal=up)
            if s is None:
                A.append(0.0); continue
            pl = s.to_2D()[0]
            # crushed footprint = the region of the plane inside the nose solid below it: use the polygon fill area of
            # the material cut (outer minus inner for the hollow nose)
            A.append(sum(p.area for p in pl.polygons_full))
        except Exception:
            A.append(0.0)
    A = np.array(A) * 1e-6
    W = np.concatenate([[0], np.cumsum(sigma * 0.5 * (A[1:] + A[:-1]) * np.diff(dep) / 1000)])
    k = min(np.searchsorted(W, E), len(dep) - 1)
    F = sigma * A[k]
    return dict(vn=round(vn, 2), crush_mm=round(dep[k], 2), F_peak_N=round(F, 0), g_peak=round(F / m_eff / G, 0))


def run(out=None, flight=None):
    d = dm.Dart(out)
    mp = d.mp; M = mp["M"] / 1000; cg = mp["cg"]; I = mp["I"] * 1e-9
    R = dict(source=d.out, mass_g=round(mp["M"], 1), cg_mm=[round(float(c), 1) for c in cg])
    pts, _ = d.hull_points(0, 0)
    rs = Rest(d)
    st, H, lab = rs.states(pts, cg)
    R["rest_states_folded"] = [dict(share_pct=round(s["share"] * 100, 1), roll_deg=s["roll_deg"], pitch_deg=s["pitch_deg"], cg_height_mm=round(s["h_mm"], 1)) for s in st if s["share"] > 0.005]
    rr, hh = roll_slice(pts, cg)
    mins = [(round(float(rr[i]), 0), round(float(hh[i]), 1)) for i in range(1, len(rr) - 1) if hh[i] <= hh[i - 1] and hh[i] < hh[i + 1]]
    maxs = [(round(float(rr[i]), 0), round(float(hh[i]), 1)) for i in range(1, len(rr) - 1) if hh[i] >= hh[i - 1] and hh[i] > hh[i + 1]]
    R["roll_slice"] = dict(minima_deg_h=mins, maxima_deg_h=maxs, note="lying along the axis: a landing roll between the two maxima either side of 0 settles belly down")
    # recovery attempts from each non-belly rest state: kick swing out and back, pods down and up
    rec = []
    seq = [("kick", k) for k in range(0, 205, 10)] + [("kick", k) for k in range(200, -5, -10)] + \
          [("dep", a) for a in range(0, 44, 6)] + [("dep", a) for a in range(42, -4, -6)]
    cache = {}
    for kind, ang in seq:
        kd, dd = (ang, 0) if kind == "kick" else (0, ang)
        if (kd, dd) not in cache:
            p2, _ = d.hull_points(kd, dd)
            cache[(kd, dd)] = rs.h(p2, d.mass_props(d.assembly(kd, dd))["cg"])
    for s in st:
        if s["share"] < 0.005 or abs(s["roll_deg"]) < 25:
            continue
        i = s["idx"]; path = []
        for kind, ang in seq:
            kd, dd = (ang, 0) if kind == "kick" else (0, ang)
            i = rs.descend(cache[(kd, dd)], i)
            path.append((kind, ang, attitude(rs.S[i])["roll_deg"]))
        fin = attitude(rs.S[i])
        best = min(path, key=lambda q: abs(q[2]))
        rec.append(dict(start_roll=s["roll_deg"], start_pitch=s["pitch_deg"], share_pct=round(s["share"] * 100, 1),
                        end_roll=fin["roll_deg"], end_pitch=fin["pitch_deg"], belly_down_at_end=abs(fin["roll_deg"]) < 25,
                        closest_to_belly=dict(step=best[0], angle=best[1], roll=best[2])))
    R["recovery_quasistatic"] = rec
    # anchor tolerance to roll: depth of each drill auger tip below ground when the deployed dart rests rolled by phi on its hull
    import dart_anchor as da
    A = da.Anchor(d)
    tol = []
    for phi in (0, 5, 10, 15, 20, 25, 30):
        c, s_ = math.cos(math.radians(phi)), math.sin(math.radians(phi))
        Rx = np.array([[1, 0, 0], [0, c, -s_], [0, s_, c]])
        depths = []
        for side in (1, -1):
            root, tip, a = A.drill_axis()
            tip = tip * np.array([1, side, 1])
            tw = Rx @ tip
            depths.append(round(-21.0 - tw[2], 1))
        tol.append(dict(roll_deg=phi, auger_tip_depths_mm=depths))
    R["anchor_vs_roll"] = tol
    # impact
    fl = flight or {}
    cases = fl.get("impact_cases") or [dict(pitch=34, v=6.6, path=52.0), dict(pitch=10, v=6.6, path=27.0), dict(pitch=45, v=6.5, path=58.0)]
    r_n = (np.array([0.0, 0.0, -d.P['NOSE_R0']]) - cg) / 1000
    imp = []
    for c in cases:
        th = math.radians(c["path"])
        n = np.array([math.sin(th), 0, math.cos(th)])     # contact normal in the body frame at the nose tip
        rxn = np.cross(r_n, n)
        m_eff = 1.0 / (1.0 / M + rxn @ np.linalg.solve(I, rxn))
        row = dict(rail_pitch=c["pitch"], v=c["v"], path_deg=c["path"], m_eff_kg=round(m_eff, 3))
        for sn, sf in dm.SURFACE.items():
            if sf["q"] > 1e4:
                r_ = hard_impact(d, c["v"], c["path"], m_eff)
            else:
                r_ = nose_impact(d, c["v"], c["path"], sf["q"] * 1e3, m_eff)
            a_cg = r_["F_peak_N"] / M
            r_["g_peak"] = round(a_cg / G, 0)                    # at the CG (F / M; the nose point itself sees F / m_eff)
            alpha = np.linalg.solve(I, np.cross(r_n, n * r_["F_peak_N"]))
            loads = {}
            for part in ("camera", "cell_1", "xiao", "bno085", "vtx", "n20_kick", "drill_leg_L_motor", "kick_leg", "drill_leg_L_auger"):
                m = d.mesh(part)
                pm = d.part_mass(part, m)
                pc = (np.array(m.center_mass if m.is_watertight else m.bounds.mean(0)) - cg) / 1000
                if part in ("drill_leg_L_auger",):
                    pc = (np.array(d.PP["drill_leg_L"]["pos"]) + np.array(m.bounds.mean(0)) - cg) / 1000
                if part == "kick_leg":
                    pc = (np.array(d.PP["kick_leg"]["pos"]) + np.array(m.bounds.mean(0)) - cg) / 1000
                a_p = n * a_cg + np.cross(alpha, pc)
                loads[part] = dict(g=round(float(np.linalg.norm(a_p)) / G, 0), N=round(pm / 1000 * float(np.linalg.norm(a_p)), 1))
            bno = loads["bno085"]["g"]
            row[sn] = dict(**r_, loads=loads, bno085_reads_g=min(bno, 8.0), bno085_raw_g=bno)
        imp.append(row)
    R["impact"] = imp
    # a 5 mm crushable nose face (TPU 85A lattice or EVA, plateau about 1.2 MPa over the d22 face) on hard ground
    c = max(cases, key=lambda q: q["v"] * math.sin(math.radians(q["path"]))); th = math.radians(c["path"]); n = np.array([math.sin(th), 0, math.cos(th)])
    rxn = np.cross(r_n, n); m_eff = 1.0 / (1.0 / M + rxn @ np.linalg.solve(I, rxn))
    E = 0.5 * m_eff * (c["v"] * math.sin(th)) ** 2
    R["nose_bumper_5mm_hard_ground"] = dict(energy_J=round(E, 2), plateau_force_N=round(E / 0.005, 0), g_cg=round(E / 0.005 / M / G, 0),
                                            plateau_stress_MPa_on_d22=round(E / 0.005 / (math.pi * 0.011 ** 2) / 1e6, 2))
    # strength checks at the worst soil-ish case (gravel) and hard ground
    os.makedirs(os.path.join(d.out, "phys31"), exist_ok=True)
    json.dump(R, open(os.path.join(d.out, "phys31", "landing.json"), "w"), indent=1, default=float)
    return R


if __name__ == "__main__":
    R = run(sys.argv[1] if len(sys.argv) > 1 else None)
    print(json.dumps({k: v for k, v in R.items() if k != "impact"}, indent=1, default=float))
    for row in R["impact"]:
        print(row["rail_pitch"], row["path_deg"], row["m_eff_kg"])
        for sn in dm.SURFACE:
            r = row[sn]; print("   ", sn, {k: v for k, v in r.items() if k != "loads"}, {k: v["N"] for k, v in r["loads"].items()})
