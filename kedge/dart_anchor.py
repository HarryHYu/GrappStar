"""KEDGE v31 anchoring physics: kick swing and foot planting, drill deploy torque and milling time, auger installation
and release torque against the real N20 / drill gearmotor torques, holding by soil (reviewed Broms + cylindrical shear),
nose-lift / tip-over under the real tether angle (planar limit analysis, scipy linprog), pull-out loads on the worms.

  python dart_anchor.py [OUT]      -> OUT/phys31/anchor.json (+ plots), OUT defaults to out31 (else out30)

Method notes (all assumptions are named in dart_model.py or below):
- Augers are short rigid piles. Lateral soil resistance per unit length: sand 3 Kp gamma z D (Broms), cohesion Matlock
  (3 + gamma z / c + 0.5 z / D) c D capped at 9 c D, c-phi loam the sum (without the double-counted gamma z / c). Two lateral capacities are given: "free head" (pile rotates, load at the
  ground entry; Broms short pile, solved for any p(s)) = design value, and "translation" (whole length at the limit
  pressure) = upper bound. Inclined augers: depth z = s sin(theta) along the axis.
- Axial: cylindrical shear through the flight tips, tau = alpha c + K sigma_v tan(phi) (alpha 0.8, K 1.0), plus tip
  bearing (9 c + gamma z Nq) on push-in. Lateral and axial interact on a linear diamond |h|/H + |a|/Q <= 1.
- Installation torque: T = Q_cyl x D/2 (shear on the flight-tip cylinder; equivalent Kt = 2/D) and the Perko field
  correlation Kt = 1433 / d_shaft^0.92 (mm, 1/m) as the low bound: both are reported.
- Radial drill augers cut sideways while the pod swings (their axis passes through the pod pivot, so screwing gives no
  depth). Not spinning: full passive pressure (Broms p) on the sweeping length. Spinning: a mill; the sideways
  (feed) force per unit length is p x f / D for a feed f per auger turn (chip-thickness scaling, f << D). Under the
  worm's stall torque the feed self-limits: f = D x T_worm / M_passive, which sets the time to deploy.
- Planar limit analysis (vertical plane through the tether): three rigid bodies (hull, pod pair, kick leg) joined at
  the two worm pivots (moment capacity = worm static holding strength, ASSUMED), soil forces on the augers inside their
  interaction diamonds, ground contacts compression-only with Coulomb friction. Maximise the tether pull.
"""
import json, math, os, sys
import numpy as np
from scipy.optimize import linprog, brentq
import dart_model as dm

G = dm.G
# ------------------------------------------------------------------------------------------------ assumptions
WORM_HOLD_NM = dm.bench().get("worm_hold_Nm", {}).get("Nm", 0.60)          # static strength of an N20 worm gearbox driven backwards (worm self-locks; limit = wheel teeth /
                             # D shaft / set screw). ASSUMED from listings (5 to 7 kg cm "max load"): MEASURE (pulltest step B3)
ALPHA_C, K_SAND = 0.8, 1.0   # adhesion factor on c, lateral earth pressure coefficient along the flight cylinder
CONTACT_AREA_MM2 = 100.0     # pod nose and kick pole footprint on the ground (bearing limit = q_dyn x area)
TETHER_EXIT_H_M = 0.407      # line leaves the rover at the carriage pusher at HOME, rail at the 6 deg tow pitch:
                             # pivot_h 0.315 (deck 0.15 ASSUMED + 0.165) + 0.0756 sin 6 + 0.085 cos 6 (base launch_NOTES)
TETHER_EXIT_X_M = 0.066      # ahead of the rail pivot (rover distance is measured from this point)
TOW_CAP_N = 55.0             # base/spec REEL tow_limit_max_n (at the dart)
FIRST_TOW_N = 40.0           # first-tow advice
TAIL_PT = (152.0, 0.0)       # tether knot / exit on the axis, (x, z) mm
MILL_FEED_FRAC_MAX = 1.0     # f / D cannot exceed 1 (that is the passive, not spinning, case)


def soil_fns(s, D):
    g, phi, c = s["gamma"] * 1e3, math.radians(s["phi"]), s["c"] * 1e3
    Kp = math.tan(math.pi / 4 + phi / 2) ** 2
    Nq = math.exp(math.pi * math.tan(phi)) * Kp if s["phi"] > 0 else 1.0

    def p_lat(z):            # N/m at depth z (m)
        z = np.maximum(z, 0.0)
        # cohesion: Matlock (1970) near-surface limit pressure (3 + gamma z / c + J z / D) c D, J 0.5, capped at 9 c D
        # (continuous from 3 c D at the surface; Broms' "nothing above 1.5 D" made the capacity fall as D grew on these
        # shallow augers). In a c-phi soil the overburden term is already in the 3 Kp gamma z D sand term.
        if c > 0:
            fc = 3 + 0.5 * z / D + (g * z / c if s["phi"] == 0 else 0.0)
            pc = np.minimum(fc, 9.0) * c * D
        else:
            pc = 0.0 * z
        return 3 * Kp * g * z * D * (s["phi"] > 0) + pc

    def tau(z):              # Pa on the flight-tip cylinder
        return ALPHA_C * c + K_SAND * g * np.maximum(z, 0) * math.tan(phi)

    def q_tip(z):
        return 9 * c + g * max(z, 0) * Nq
    return p_lat, tau, q_tip, Kp, Nq


def broms_free(p, s):
    """Free-head rigid pile, load at the head (s = 0), lateral pressure p(s) N/m on stations s (m). Returns H, rotation point."""
    ds = np.gradient(s)
    def mom(sr):
        a = s <= sr
        return (p * s * ds)[a].sum() - (p * s * ds)[~a].sum()
    if mom(s[-1]) <= 0:
        return 0.0, s[-1]
    sr = brentq(mom, s[0] + 1e-9, s[-1])
    a = s <= sr
    return float((p * ds)[a].sum() - (p * ds)[~a].sum()), float(sr)


class Anchor:
    def __init__(self, d: dm.Dart, kick_deg=None, dep_deg=None, auger_d_mm=None):
        self.d = d; P = d.P
        self.kick_deg = P["KICK_DEG"] if kick_deg is None else kick_deg
        self.dep_deg = P["DEPLOY_DEG"] if dep_deg is None else dep_deg
        self.zg = d.ground
        self.D = (auger_d_mm or P["AUGER_D"]) / 1000; self.Dc = P["AUGER_CORE"] / 1000; self.pitch = P["AUGER_PITCH"] / 1000
        self.Df = None
        a = d.mesh("drill_leg_L_auger").bounds
        self.r0, self.r1 = float(a[0, 0]), float(a[1, 0])            # auger along the pod radius (mm)
        self.piv = np.array(d.PP["drill_leg_L"]["pos"], float)
        fb = d.mesh("foot_auger").bounds
        self.foot_local = (np.array([fb[:, 0].mean(), 0, fb[0, 2]]), np.array([fb[:, 0].mean(), 0, fb[1, 2]]))  # root, tip
        self.Df = (fb[1, 1] - fb[0, 1]) / 1000
        self.kpiv = np.array(d.PP["kick_leg"]["pos"], float)

    # ---------------------------------------------------------------- geometry
    def drill_axis(self, th_deg=None):
        th = math.radians(self.dep_deg if th_deg is None else th_deg)
        a = np.array([math.cos(th), 0, -math.sin(th)])
        return self.piv + self.r0 * a, self.piv + self.r1 * a, a

    def foot_axis(self, k_deg=None):
        T = self.d.T("kick_leg", self.kick_deg if k_deg is None else k_deg)
        r, t = [(T @ np.r_[p, 1])[:3] for p in self.foot_local]
        return r, t, (t - r) / np.linalg.norm(t - r)

    def embedded(self, root, tip, tip_len_mm):
        """Embedded part of an auger axis below the ground plane: entry point, length (m, cone counted half), depths."""
        zr, zt = root[2], tip[2]
        if zt >= self.zg:
            return dict(L=0.0, depth=0.0, entry=tip, s=np.zeros(2), z=np.zeros(2), angle=0.0)
        f = 0.0 if zr <= self.zg else (zr - self.zg) / (zr - zt)
        entry = root + f * (tip - root)
        Lg = np.linalg.norm(tip - entry) / 1000
        L = max(0.0, Lg - 0.5 * tip_len_mm / 1000)
        s = np.linspace(0, L, 200)
        ang = math.asin((entry[2] - tip[2]) / 1000 / Lg)
        z = (self.zg - entry[2]) / 1000 + s * math.sin(ang)
        return dict(L=L, L_geom=Lg, depth=(self.zg - tip[2]) / 1000, entry=entry, s=s, z=z, angle=math.degrees(ang))

    # ---------------------------------------------------------------- capacities
    def pile(self, emb, soil, D):
        p_lat, tau, q_tip, Kp, Nq = soil_fns(soil, D)
        s, z = emb["s"], emb["z"]
        if emb["L"] <= 1e-4:
            return dict(H_free=0.0, H_trans=0.0, Q_pull=0.0, Q_push=0.0, T_install_Nm=0.0, T_perko_Nm=0.0, x_res=0.0)
        p = p_lat(z)
        Hf, sr = broms_free(p, s)
        Ht = float(np.trapezoid(p, s))
        x_res = float(np.trapezoid(p * s, s) / max(Ht, 1e-9))
        Qs = float(np.trapezoid(tau(z) * math.pi * D, s))
        Qtip = q_tip(float(z[-1])) * math.pi * D ** 2 / 4
        Kt_perko = 1433.0 / (self.Dc * 1000) ** 0.92
        return dict(H_free=Hf, H_trans=Ht, s_rot=sr, x_res=x_res, Q_pull=Qs, Q_push=Qs + Qtip,
                    T_install_Nm=Qs * D / 2 + Qtip * D / 4, T_perko_Nm=(Qs + Qtip) / Kt_perko)

    # ---------------------------------------------------------------- deploy (radial augers sweep sideways)
    def deploy_moment(self, soil, th_deg, side_both=True):
        """Passive moment (N m) about the pod pivot to sweep ONE auger sideways at pod angle th (not spinning)."""
        root, tip, a = self.drill_axis(th_deg)
        D = self.D
        p_lat = soil_fns(soil, D)[0]
        r = np.linspace(self.r0, self.r1 - 0.5 * self.d.P["AUGER_TIP"], 300) / 1000
        z = (self.zg / 1000 - (self.piv[2] / 1000 - r * math.sin(math.radians(th_deg))))
        p = p_lat(z) * (z > 0)
        return float(np.trapezoid(p * r, r))

    def kick_plant_moment(self, soil, k_deg):
        """Passive moment (N m) about the kick pivot from the part of the foot motion that is NOT along its own axis
        (the foot moves on an arc; a screw only advances along its axis). Not spinning: full Broms pressure on it."""
        r_, t_, a = self.foot_axis(k_deg)
        emb = self.embedded(r_, t_, self.d.P["FOOT_TIP"])
        if emb["L"] <= 0:
            return 0.0
        p_lat = soil_fns(soil, self.Df)[0]
        M = 0.0
        ey = np.array([0, 1.0, 0])
        for s_, z_ in zip(emb["s"], emb["z"]):
            pt = emb["entry"] + a * s_ * 1000
            rv = (pt - self.kpiv) / 1000
            v = np.cross(ey, rv)                     # point velocity per unit kick rate (direction only matters)
            vperp = v - a * (v @ a)
            M += float(p_lat(np.array([z_]))[0]) * np.linalg.norm(vperp) * (emb["s"][1] - emb["s"][0])
        return M

    # ---------------------------------------------------------------- limit analysis
    def masses(self):
        d = self.d
        dep = d.assembly(self.kick_deg, self.dep_deg)
        fams = dict(pods=set(d.fam("drill_leg_L") + d.fam("drill_leg_R") + d.fam("coupler_L") + d.fam("coupler_R")),
                    kick=set(d.fam("kick_leg")))
        out = {}
        for k, names in fams.items():
            out[k] = d.mass_props([(n, m) for n, m in dep if n in names])
        allm = d.mass_props(dep)
        Mh = allm["M"] - out["pods"]["M"] - out["kick"]["M"]
        cgh = (allm["M"] * allm["cg"] - out["pods"]["M"] * out["pods"]["cg"] - out["kick"]["M"] * out["kick"]["cg"]) / Mh
        out["hull"] = dict(M=Mh, cg=cgh)
        out["all"] = allm
        return out

    def limit(self, soil, rover_m, cap="diamond", worm_hold=WORM_HOLD_NM, foot=True, drills=True, verbose=False):
        """Max tether pull (N) at the dart for a rover line exit rover_m (m) behind the tail (horizontal)."""
        mm = self.masses()
        Dr = self.drill_axis(); Ft = self.foot_axis()
        ed = self.embedded(Dr[0], Dr[1], self.d.P["AUGER_TIP"])
        ef = self.embedded(Ft[0], Ft[1], self.d.P["FOOT_TIP"])
        mu = soil["mu"]; qb = soil["q_dyn"] * 1e3 * CONTACT_AREA_MM2 * 1e-6
        tpz = getattr(self.d, "tether_pt", TAIL_PT)[1]
        beta = math.atan2(TETHER_EXIT_H_M - (tpz - self.zg) / 1000, rover_m)
        ut = np.array([math.cos(beta), math.sin(beta)])
        X = lambda p: np.array([p[0], p[2]]) / 1000 if len(p) == 3 else np.array(p) / 1000
        cols, names, bounds = [], [], []
        B = {"hull": 0, "pods": 1, "kick": 2}

        def add(name, body_forces, lb, ub):
            """body_forces: list of (body, point (x, z) m, direction (fx, fz)) or (body, None, moment)."""
            c = np.zeros(9)
            for b, pt, f in body_forces:
                i = 3 * B[b]
                if pt is None:
                    c[i + 2] += f
                else:
                    c[i] += f[0]; c[i + 1] += f[1]; c[i + 2] += pt[1] * f[0] - pt[0] * f[1]
            cols.append(c); names.append(name); bounds.append((lb, ub))
        tail = X(getattr(self.d, "tether_pt", TAIL_PT))
        add("tether", [("hull", tail, ut)], 0, None)
        pv = X(self.piv); kp = X(self.kpiv)
        for k, e in (("x", (1, 0)), ("z", (0, 1))):
            add("pod pivot R" + k, [("pods", pv, e), ("hull", pv, (-e[0], -e[1]))], None, None)
            add("kick pivot R" + k, [("kick", kp, e), ("hull", kp, (-e[0], -e[1]))], None, None)
        # v31 hard stops: the deploy horn takes the moment that turns the pods further down (negative moment from the hull
        # on the pods, deploy is + about y); the kick rest takes the moment that turns the leg further in its kick direction
        # (kick is - about y, so the rest gives a + moment). The worm only holds the other direction.
        horn = self.horn if hasattr(self, "horn") else ("HORN_GAP" in self.d.P)
        rest = self.rest if hasattr(self, "rest") else ("KICK_REST_DEG" in self.d.P)
        add("deploy worm moment", [("pods", None, 1.0), ("hull", None, -1.0)], 0.0 if horn else -worm_hold, worm_hold)
        add("kick worm moment", [("kick", None, 1.0), ("hull", None, -1.0)], -worm_hold, 0.0 if rest else worm_hold)
        if horn:
            add("deploy horn (hard stop)", [("pods", None, -1.0), ("hull", None, 1.0)], 0, None)
        if rest:
            add("kick rest (hard stop)", [("kick", None, 1.0), ("hull", None, -1.0)], 0, None)
        diam = []
        NSEG = 10
        for tag, body, emb, Dau, mult in (("drill augers", "pods", ed, self.D, 2 * drills), ("foot auger", "kick", ef, self.Df, 1 * foot)):
            a3 = (Dr if tag.startswith("drill") else Ft)[2]
            a2 = np.array([a3[0], a3[2]]); n2 = np.array([a2[1], -a2[0]])   # n: perpendicular in the plane
            ent = X(emb["entry"])
            if emb["L"] <= 1e-4 or mult == 0:
                continue
            p_lat, tau, q_tip, _, _ = soil_fns(soil, Dau)
            edges = np.linspace(0, emb["L"], NSEG + 1); ds = edges[1] - edges[0]
            ang = math.radians(emb["angle"])
            z_ent = 0.0
            for k in range(NSEG):
                sm = 0.5 * (edges[k] + edges[k + 1]); zk = z_ent + sm * math.sin(ang)
                Hk = float(p_lat(np.array([zk]))[0]) * ds * mult
                Ak = float(tau(np.array([zk]))[0]) * math.pi * Dau * ds * mult
                pt = ent + a2 * sm
                idx = len(cols)
                add(f"{tag} seg{k} lateral +", [(body, pt, n2)], 0, None)
                add(f"{tag} seg{k} lateral -", [(body, pt, -n2)], 0, None)
                add(f"{tag} seg{k} shear (resists pull-out)", [(body, pt, a2)], 0, None)
                add(f"{tag} seg{k} shear (resists push-in)", [(body, pt, -a2)], 0, None)
                diam.append((tag, idx, Hk, Ak))
            ztip = z_ent + emb["L"] * math.sin(ang)
            add(f"{tag} tip bearing", [(body, ent + a2 * emb["L"], -a2)], 0, q_tip(ztip) * math.pi * Dau ** 2 / 4 * mult)
        fr = []
        contacts = [("hull belly front (x 20)", "hull", (20.0, self.zg), None), ("hull belly rear (x 141)", "hull", (141.5, self.zg), None),
                    ("pod nose on ground", "pods", (80.7, self.zg), qb), ("kick pole tip on ground", "kick", (-70.3, self.zg), qb)]
        for nm, body, pt, cap_n in contacts:
            i = len(cols)
            add(nm + " normal", [(body, X(pt), (0, 1))], 0, cap_n)
            add(nm + " friction +x", [(body, X(pt), (1, 0))], 0, None)
            add(nm + " friction -x", [(body, X(pt), (-1, 0))], 0, None)
            fr.append((nm, i))
        A_eq = np.array(cols).T
        # equilibrium: sum(cols * var) + weights = 0  ->  A var = -weights
        b_eq = np.zeros(9)
        for k in ("hull", "pods", "kick"):
            W = mm[k]["M"] / 1000 * G; c = X(mm[k]["cg"]); i = 3 * B[k]
            wf = np.array([0.0, -W]); b_eq[i] -= wf[0]; b_eq[i + 1] -= wf[1]; b_eq[i + 2] -= (c[1] * wf[0] - c[0] * wf[1])
        A_ub, b_ub, ubn = [], [], []
        n = len(cols)
        for tag, i, Hk, Ak in diam:
            if cap == "box":          # lateral and shear independent (upper bound)
                for j, c_ in ((0, Hk), (1, Hk), (2, Ak), (3, Ak)):
                    r = np.zeros(n); r[i + j] = 1; A_ub.append(r); b_ub.append(c_); ubn.append(tag + " soil capacity")
                continue
            r = np.zeros(n)
            if Hk > 1e-9:
                r[i] = r[i + 1] = 1 / Hk
            else:
                for j in (0, 1):
                    rr = np.zeros(n); rr[i + j] = 1; A_ub.append(rr); b_ub.append(0.0); ubn.append(tag + " soil capacity")
            r[i + 2] = r[i + 3] = 1 / max(Ak, 1e-9)
            A_ub.append(r); b_ub.append(1.0); ubn.append(tag + " soil capacity")
        for nm, i in fr:
            r = np.zeros(n); r[i + 1] = r[i + 2] = 1; r[i] = -mu
            A_ub.append(r); b_ub.append(0.0); ubn.append(nm + " friction cone")
        cobj = np.zeros(n); cobj[0] = -1
        res = linprog(cobj, A_ub=np.array(A_ub), b_ub=np.array(b_ub), A_eq=A_eq, b_eq=b_eq, bounds=bounds, method="highs")
        if res.status != 0:
            return dict(F=0.0, status=res.message, beta_deg=math.degrees(beta))
        x = res.x
        act = []
        if res.ineqlin is not None:
            for k, mrg in enumerate(res.ineqlin.marginals):
                if abs(mrg) > 1e-9:
                    act.append(ubn[k])
        for j, (lb, ub) in enumerate(bounds):
            if j == 0:
                continue
            if ub is not None and abs(x[j] - ub) < 1e-6 and abs(res.upper.marginals[j]) > 1e-9:
                act.append(names[j] + " at its limit")
            if lb is not None and lb != 0 and abs(x[j] - lb) < 1e-6 and abs(res.lower.marginals[j]) > 1e-9:
                act.append(names[j] + " at its limit")
        nose_N = x[names.index("hull belly front (x 20) normal")]
        vals = {nm: round(float(v), 2) for nm, v in zip(names, x) if abs(v) > 1e-6}
        hm = x[names.index("deploy horn (hard stop)")] if "deploy horn (hard stop)" in names else 0.0
        km_ = x[names.index("kick rest (hard stop)")] if "kick rest (hard stop)" in names else 0.0
        return dict(F=float(x[0]), beta_deg=round(math.degrees(beta), 1), governing=sorted(set(act)), nose_contact_N=round(float(nose_N), 2),
                    horn_moment_Nm=round(float(hm), 3), kick_rest_moment_Nm=round(float(km_), 3),
                    deploy_worm_Nm=round(float(x[names.index("deploy worm moment")]), 3), kick_worm_Nm=round(float(x[names.index("kick worm moment")]), 3),
                    forces=vals if verbose else None)


def run(out=None):
    d = dm.Dart(out)
    A = Anchor(d)
    P = d.P
    R = dict(source=d.out, assumptions=dict(worm_hold_Nm=WORM_HOLD_NM, alpha_c=ALPHA_C, K_sand=K_SAND, tether_exit_h_m=TETHER_EXIT_H_M,
                                            contact_area_mm2=CONTACT_AREA_MM2, soils=dm.SOIL, motors=dm.MOTORS, tow_cap_N=TOW_CAP_N, first_tow_N=FIRST_TOW_N))
    V = 7.4
    mot = {k: dm.motor(k, V) for k in ("worm", "drill", "foot")}
    mot_lo = {k: dm.motor(k, 6.4) for k in ("worm", "drill", "foot")}     # sagging pack near empty, under load
    R["motors_7V4"] = mot; R["motors_6V4"] = mot_lo
    # ---------------- geometry
    Dr = A.drill_axis(); Ft = A.foot_axis()
    ed = A.embedded(Dr[0], Dr[1], P["AUGER_TIP"]); ef = A.embedded(Ft[0], Ft[1], P["FOOT_TIP"])
    th_touch = math.degrees(math.asin((A.piv[2] - A.zg) / A.r1))
    R["geometry"] = dict(drill=dict(D_mm=P["AUGER_D"], core_mm=P["AUGER_CORE"], pitch_mm=P["AUGER_PITCH"], r_from_pivot_mm=[round(A.r0, 1), round(A.r1, 1)],
                                    angle_deg=round(ed["angle"], 1), depth_mm=round(ed["depth"] * 1000, 1), embedded_mm=round(ed["L_geom"] * 1000, 1),
                                    embedded_effective_mm=round(ed["L"] * 1000, 1), entry_xz_mm=[round(ed["entry"][0], 1), round(ed["entry"][2], 1)],
                                    tip_touches_ground_at_deg=round(th_touch, 1), swing_in_soil_deg=round(A.dep_deg - th_touch, 1)),
                         foot=dict(D_mm=round(A.Df * 1000, 1), angle_deg=round(ef["angle"], 1), depth_mm=round(ef["depth"] * 1000, 1),
                                   embedded_mm=round(ef["L_geom"] * 1000, 1), entry_xz_mm=[round(ef["entry"][0], 1), round(ef["entry"][2], 1)]))
    # ---------------- kick swing in air (gravity) and foot planting crowd force
    kf = d.fam("kick_leg")
    km = d.mass_props([(n, d.mesh(n)) for n in kf]); kc = km["cg"]
    tq = []
    for deg in range(0, int(P["KICK_DEG"]) + 1, 5):
        c = (d.T("kick_leg", deg) @ np.r_[kc, 1])[:3] - A.kpiv
        tq.append(km["M"] / 1000 * G * abs(c[0]) / 1000)
    fr_, ft_, fa = A.foot_axis()
    lever = abs(np.cross(fr_ - A.kpiv, fa)[1]) / 1000          # moment arm of a force along the foot axis about the kick pivot
    crowd = mot["worm"]["stall_Nm"] / lever
    R["kick"] = dict(leg_mass_g=round(km["M"], 1), gravity_torque_max_Nm=round(max(tq), 4), stall_Nm=mot["worm"]["stall_Nm"],
                     margin_swing=round(mot["worm"]["stall_Nm"] / max(tq), 1), foot_axis_lever_mm=round(lever * 1000, 1),
                     crowd_force_at_worm_stall_N=round(crowd, 2),
                     swing_time_free_s=round(math.radians(P["KICK_DEG"]) / mot["worm"]["free_rad_s"], 1))
    # ---------------- per soil: capacities, install / unscrew torque, deploy torque and time
    soils = {}
    for sn, s in dm.SOIL.items():
        cd = A.pile(ed, s, A.D); cf = A.pile(ef, s, A.Df)
        Mfull = A.deploy_moment(s, A.dep_deg)
        prof = [(th, round(2 * A.deploy_moment(s, th), 3)) for th in np.arange(10, A.dep_deg + 0.01, 2.0)]
        Mmax2 = max(v for _, v in prof)
        Tw = mot["worm"]["stall_Nm"]
        f_over_D = min(MILL_FEED_FRAC_MAX, Tw / Mmax2) if Mmax2 > 0 else MILL_FEED_FRAC_MAX
        n_rev = mot["drill"]["free_rad_s"] / (2 * math.pi) * 0.8         # turns/s while cutting (about 80 percent of free)
        sweep = A.r1 / 1000 * math.radians(A.dep_deg - th_touch)          # tip arc through the soil (m)
        t_mill = sweep / (f_over_D * A.D * n_rev) if Mmax2 > Tw else sweep / (A.r1 / 1000 * mot["worm"]["free_rad_s"])
        # drill torque while milling at that feed (cutting force per length = p f/D, tangential ~ feed force) + install
        p_lat = soil_fns(s, A.D)[0]
        T_mill = float(np.trapezoid(p_lat(ed["z"]) * f_over_D, ed["s"])) * A.D / 2
        T_drill = cd["T_install_Nm"] + T_mill
        # foot: crowd needed to start a screw ~ tip bearing on the cone tip area (D/3 dia) x 0.3; the screw then self-feeds
        crowd_need = soil_fns(s, A.Df)[2](0.005) * math.pi * (A.Df / 3) ** 2 / 4 * 0.3
        soils[sn] = dict(
            drill_each=dict(H_free_N=round(cd["H_free"], 1), H_translation_N=round(cd["H_trans"], 1), Q_pull_N=round(cd["Q_pull"], 1), Q_push_N=round(cd["Q_push"], 1),
                            install_torque_Nm=round(cd["T_install_Nm"], 3), install_torque_perko_Nm=round(cd["T_perko_Nm"], 3)),
            foot=dict(H_free_N=round(cf["H_free"], 1), H_translation_N=round(cf["H_trans"], 1), Q_pull_N=round(cf["Q_pull"], 1), Q_push_N=round(cf["Q_push"], 1),
                      install_torque_Nm=round(cf["T_install_Nm"], 3), install_torque_perko_Nm=round(cf["T_perko_Nm"], 3),
                      foot_motor_stall_Nm=mot["foot"]["stall_Nm"], kick_worm_as_foot_motor_stall_Nm=mot["worm"]["stall_Nm"],
                      crowd_to_start_N=round(crowd_need, 2), crowd_available_N=round(crowd, 2)),
            deploy=dict(passive_moment_both_pods_at_full_Nm=round(2 * Mfull, 3), passive_moment_max_Nm=round(Mmax2, 3), profile_deg_Nm=prof,
                        worm_stall_Nm=Tw, ratio_needed_over_available=round(Mmax2 / Tw, 1),
                        milling_feed_over_D=round(f_over_D, 4), time_to_deploy_s=round(t_mill, 1), tip_arc_in_soil_mm=round(sweep * 1000, 1),
                        drill_torque_while_milling_Nm=round(T_drill, 3), drill_stall_Nm=mot["drill"]["stall_Nm"],
                        drill_margin=round(mot["drill"]["stall_Nm"] / max(T_drill, 1e-6), 1)),
            unscrew=dict(drill_unscrew_torque_Nm=round(1.3 * cd["T_install_Nm"], 3), drill_margin=round(mot["drill"]["stall_Nm"] / max(1.3 * cd["T_install_Nm"], 1e-6), 1),
                         foot_unscrew_torque_Nm=round(1.3 * cf["T_install_Nm"], 3),
                         note="unscrew torque taken 1.3 x install (soil set-up, flights packed); the pods then mill back out the same way they went in"),
        )
    R["soils"] = soils
    # ---------------- limit analysis: max tether pull vs rover distance
    lim = {}
    for sn, s in dm.SOIL.items():
        rows = []
        for rov in (1.0, 2.0, 3.0, 4.0, 5.0):
            r_free = A.limit(s, rov, "diamond")
            r_tr = A.limit(s, rov, "box")
            r_nf = A.limit(s, rov, foot=False)
            r_strong = A.limit(s, rov, worm_hold=10.0)
            rows.append(dict(rover_m=rov, beta_deg=r_free["beta_deg"], F_design_N=round(r_free["F"], 1), governing=r_free["governing"],
                             deploy_worm_Nm=r_free["deploy_worm_Nm"], kick_worm_Nm=r_free["kick_worm_Nm"],
                             horn_moment_Nm=r_free["horn_moment_Nm"], kick_rest_moment_Nm=r_free["kick_rest_moment_Nm"],
                             F_upper_translation_N=round(r_tr["F"], 1), F_without_foot_N=round(r_nf["F"], 1),
                             F_if_worms_never_slip_N=round(r_strong["F"], 1)))
        lim[sn] = rows
    R["holding_limit"] = lim
    # partial deploy: holding if the deploy stalls early
    part = {}
    for sn, s in dm.SOIL.items():
        rr = []
        for th in (20, 25, 30, 35, 40, A.dep_deg):
            Ap = Anchor(d, dep_deg=th)
            rr.append((round(float(th), 2), round(Ap.limit(s, 3.0)["F"], 1)))
        part[sn] = rr
    R["holding_vs_deploy_angle_rover3m"] = part
    # kick planting: sideways part of the foot motion over the last degrees of the swing
    kp = {}
    for sn, s in dm.SOIL.items():
        kp[sn] = [(k, round(A.kick_plant_moment(s, k), 3)) for k in np.arange(P["KICK_DEG"] - 20, P["KICK_DEG"] + 0.01, 4.0)]
    R["kick_plant_passive_moment_Nm"] = kp
    # options: auger diameter, drill motor speed, deploy worm holding strength (B2 measures it), with this build's hard stops
    imp = []
    Dnow = float(P["AUGER_D"])
    for wh in (0.3, 0.6, 1.2):
        for dmm in sorted({Dnow, 11.0, 15.0, 17.0}):
            for drpm in (100.0, 50.0):
                Ai = Anchor(d, auger_d_mm=dmm)
                row = dict(worm_hold_Nm=wh, auger_d_mm=dmm, drill_rpm12=drpm, current=bool(dmm == Dnow and drpm == 100.0 and wh == WORM_HOLD_NM))
                dm.MOTORS["drill"]["rpm12"] = drpm
                md = dm.motor("drill", V); mw = dm.motor("worm", V)
                dm.MOTORS["drill"]["rpm12"] = 100.0
                for sn in ("dry sand", "moist loam", "firm clay"):
                    s = dm.SOIL[sn]
                    edi = Ai.embedded(*Ai.drill_axis()[:2], P["AUGER_TIP"])
                    cp = Ai.pile(edi, s, Ai.D)
                    Mmax = max(2 * Ai.deploy_moment(s, th) for th in np.arange(16, Ai.dep_deg + 0.01, 1.0))
                    Tw = mw["stall_Nm"]
                    fD = min(1.0, Tw / Mmax) if Mmax > 0 else 1.0
                    nrev = md["free_rad_s"] / (2 * math.pi) * 0.8
                    sweep = Ai.r1 / 1000 * math.radians(Ai.dep_deg - th_touch)
                    t_dep = sweep / (fD * Ai.D * nrev)
                    T_mill = float(np.trapezoid(soil_fns(s, Ai.D)[0](edi["z"]) * fD, edi["s"])) * Ai.D / 2
                    Treq = max(cp["T_install_Nm"] + T_mill, 1.3 * cp["T_install_Nm"])
                    lr = Ai.limit(s, 3.0, worm_hold=wh)
                    row[sn] = dict(F_hold_N=round(lr["F"], 1), horn_Nm=lr["horn_moment_Nm"], deploy_s=round(t_dep, 0),
                                   drill_margin=round(md["stall_Nm"] / max(Treq, 1e-6), 2), installs=bool(md["stall_Nm"] >= 1.25 * Treq))
                imp.append(row)
    R["options"] = imp
    # loam cohesion sensitivity (the pull test calibrates this)
    sens = []
    for c in (2.0, 5.0, 10.0, 20.0):
        s = dict(dm.SOIL["moist loam"], c=c)
        lr = A.limit(s, 3.0)
        sens.append(dict(c_kPa=c, F_N=round(lr["F"], 1), horn_Nm=lr["horn_moment_Nm"], F_soil_only_N=round(A.limit(s, 3.0, worm_hold=50.0)["F"], 1),
                         drill_install_Nm=round(A.pile(ed, s, A.D)["T_install_Nm"], 3)))
    R["loam_cohesion_sensitivity_rover3m"] = sens
    # motor heating while milling in loam (N20 winding: R_th about 40 K/W, time constant about 90 s, ASSUMED typical)
    RTH, TAU = 40.0, 90.0
    th_ = {}
    for tag, drpm in (("100 rpm drills", 100.0), ("50 rpm drills", 50.0)):
        row = next(r for r in imp if r["worm_hold_Nm"] == 0.6 and r["auger_d_mm"] == Dnow and r["drill_rpm12"] == drpm)
        t = row["moist loam"]["deploy_s"]
        dm.MOTORS["drill"]["rpm12"] = drpm; mdr = dm.motor("drill", V)
        Ti = A.pile(ed, dm.SOIL["moist loam"], A.D)["T_install_Nm"]
        Id = min(dm.motor_current_for("drill", Ti), mdr["stall_A"]); dm.MOTORS["drill"]["rpm12"] = 100.0
        Pd = Id ** 2 * mdr["R_ohm"]; Pw = mot["worm"]["stall_A"] ** 2 * mot["worm"]["R_ohm"]
        f = 1 - math.exp(-t / TAU)
        th_[tag] = dict(deploy_s=t, drill_A=round(Id, 3), drill_winding_W=round(Pd, 2), drill_rise_K=round(Pd * RTH * f, 0),
                        worm_winding_W=round(Pw, 2), worm_rise_K=round(Pw * RTH * f, 0))
    R["motor_heating_loam"] = dict(R_th_K_per_W=RTH, tau_s=TAU, cases=th_, note="PLA softens near 60 C: keep the pod motors under about 30 K rise")
    # energy
    R["energy"] = energy(R, mot)
    os.makedirs(os.path.join(d.out, "phys31"), exist_ok=True)
    json.dump(R, open(os.path.join(d.out, "phys31", "anchor.json"), "w"), indent=1, default=float)
    plots(A, R, os.path.join(d.out, "phys31"))
    return R


# ------------------------------------------------------------------------------------------------ energy
CELL_MAH = 700.0             # HIBATT IMR 18350 (listing 700 to 800 mAh): 700 used
CELL_USABLE = 0.90           # to about 3.2 V per cell under load
BUCK_EFF = 0.85              # MP1584 at 5 V, 0.3 to 0.5 A from 7.4 V
LOADS_5V_A = dict(xiao_wifi_ap=0.11, vtx_25mw=0.20, camera=0.10, bno085_ina219=0.015, drv8833_awake_x2=0.004)


def energy(R, mot):
    Vb = 7.4
    P5 = 5.0 * sum(LOADS_5V_A.values()) / BUCK_EFF
    I5 = P5 / Vb
    cap_Ah = CELL_MAH / 1000 * CELL_USABLE
    seq = []
    s = R["soils"]["moist loam"]
    t_dep = s["deploy"]["time_to_deploy_s"]
    seq.append(("kick swing", R["kick"]["swing_time_free_s"], 0.06))
    seq.append(("kick + foot planting (kick at stall, foot working)", 15.0, mot["worm"]["stall_A"] + dm.motor_current_for("foot", s["foot"]["install_torque_Nm"])))
    seq.append(("deploy worm at stall while both drills mill (loam)", t_dep, mot["worm"]["stall_A"] + 2 * dm.motor_current_for("drill", s["deploy"]["drill_torque_while_milling_Nm"])))
    seq.append(("drills finishing", 10.0, 2 * dm.motor_current_for("drill", s["drill_each"]["install_torque_Nm"])))
    mAh = sum(t * i for _, t, i in seq) / 3.6
    rel = sum(t * i for _, t, i in seq) / 3.6 * 1.3
    return dict(cap_usable_mAh=round(cap_Ah * 1000), avionics_W=round(P5, 2), avionics_A_at_7V4=round(I5, 3),
                video_hours=round(cap_Ah / I5, 2), anchor_sequence_loam=[(n, round(t, 1), round(i, 3)) for n, t, i in seq],
                anchor_mAh=round(mAh, 1), release_mAh=round(rel, 1),
                anchor_plus_release_percent=round((mAh + rel) / (cap_Ah * 1000) * 100, 1),
                note="5 V loads are datasheet typicals (ESP32-S3 AP 80 to 150 mA, Zeus nano 25 mW about 1 W, Caddx Ant about 0.5 W)")


def plots(A, R, outd):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "Liberation Sans", "font.size": 8})
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    for i, (sn, rows) in enumerate(R["holding_limit"].items()):
        x = [r["rover_m"] for r in rows]
        ax[0].plot(x, [r["F_design_N"] for r in rows], "k-", marker="os^"[i], label=sn + " (design)")
        ax[0].plot(x, [r["F_upper_translation_N"] for r in rows], "k:", marker="os^"[i], mfc="none", label=sn + " (upper)")
    xr = [1.0, 2.0, 3.0, 4.0, 5.0]
    ax[0].plot(xr, [A.limit(dm.SOIL["moist loam"], r, worm_hold=0.3)["F"] for r in xr], "k--", marker="s", mfc="w", label="moist loam, worm holds only 0.3 N m")
    ax[0].axhline(TOW_CAP_N, color="k", lw=0.6, ls="--"); ax[0].text(1.05, TOW_CAP_N + 1, "tow cap 55 N")
    ax[0].axhline(FIRST_TOW_N, color="k", lw=0.6, ls="-."); ax[0].text(1.05, FIRST_TOW_N + 1, "first tow 40 N")
    ax[0].set_xlabel("rover line exit behind the dart tail (m)"); ax[0].set_ylabel("max tether pull at the dart (N)"); ax[0].grid(lw=0.3)
    ax[0].legend(fontsize=6, loc="center right"); ax[0].set_title("Holding (planar limit analysis; clay only if the drills could install)")
    for i, sn in enumerate(R["soils"]):
        pr = R["soils"][sn]["deploy"]["profile_deg_Nm"]
        ax[1].semilogy([p[0] for p in pr], [max(p[1], 1e-3) for p in pr], "k-", marker="os^"[i], label=sn)
    ax[1].axhline(R["motors_7V4"]["worm"]["stall_Nm"], color="k", ls="--", lw=0.6); ax[1].text(10.5, R["motors_7V4"]["worm"]["stall_Nm"] * 1.1, "deploy worm stall")
    ax[1].set_xlabel("pod angle (deg)"); ax[1].set_ylabel("moment to sweep both augers, not spinning (N m)"); ax[1].grid(lw=0.3, which="both")
    ax[1].legend(fontsize=6); ax[1].set_title("Drill deploy torque")
    fig.tight_layout(); fig.savefig(os.path.join(outd, "anchor.png"), dpi=150)


if __name__ == "__main__":
    R = run(sys.argv[1] if len(sys.argv) > 1 else None)
    print(json.dumps({k: R[k] for k in ("geometry", "kick", "energy")}, indent=1, default=float))
    for sn, s in R["soils"].items():
        print(sn, json.dumps(s, default=float))
    for sn, rows in R["holding_limit"].items():
        for r in rows:
            print(sn, r)
    print(R["holding_vs_deploy_angle_rover3m"])
