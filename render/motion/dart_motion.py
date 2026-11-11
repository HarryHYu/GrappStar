"""KEDGE v34 dart motion for the render: launch on the base carriage, free flight (kedge/flight34.py physics), and the
landing / righting / anchoring sim trace (kedge/sim_righting via sim_dump.mjs), all as the pose of dart:root (the dart
frame: nose tip origin, +x to the tail, +z up; metres) in the world (X launch direction, Z up), plus joint angles.

Frames used here:
  dart frame   kedge/out34 parts.json (mm in the files, m here)
  flight plane x forward (world X), z up; th = body axis angle above the horizon (flight34.fly)
  sim world    x launch direction, y up, z right; sim body x nose, y top, z right, origin on the hull axis at the CG
               station (cgX from the nose); C maps sim world -> world, M maps the dart frame -> sim body.
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
KEDGE = os.path.join(ROOT, "kedge")
if KEDGE not in sys.path:
    sys.path.insert(0, KEDGE)

def kedge_out():
    """the dart build every module uses: $KEDGE_OUT, else kedge/out35 once kedge/out35/READY exists, else kedge/out34"""
    env = os.environ.get("KEDGE_OUT")
    if env:
        return env if os.path.isabs(env) else os.path.join(ROOT, env)
    o35 = os.path.join(KEDGE, "out35")
    return o35 if os.path.exists(os.path.join(o35, "READY")) else os.path.join(KEDGE, "out34")


OUT = kedge_out()

C_SIM = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])           # sim world (x, y up, z right) -> world (X, Y, Z up)
M_D2S = np.array([[-1.0, 0, 0], [0, 0, 1.0], [0, 1.0, 0]])            # dart frame axes -> sim body axes (proper rotation)


# ------------------------------------------------------------------ small rotation helpers
def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def q2R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def R2q(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def slerp(q0, q1, u):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = float(q0 @ q1)
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - u) * th) * q0 + math.sin(u * th) * q1) / math.sin(th)


def smooth(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)


def smoother(u):
    u = min(1.0, max(0.0, u))
    return u * u * u * (u * (6 * u - 15) + 10)


def R_flight(th, phi, psi=0.0):
    """dart:root rotation for nose pitch th above the horizon, roll phi about the dart's own +x, heading psi"""
    return Rz(psi) @ Ry(-th) @ Rz(math.pi) @ Rx(phi)


def decompose(R):
    """inverse of R_flight: (th, phi, psi)"""
    A = R @ Rx(0.0)
    nose = -A[:, 0]
    psi = math.atan2(nose[1], nose[0])
    th = math.atan2(nose[2], math.hypot(nose[0], nose[1]))
    Rr = (Rz(psi) @ Ry(-th) @ Rz(math.pi)).T @ R
    phi = math.atan2(Rr[2, 1], Rr[1, 1])
    return th, phi, psi


# ------------------------------------------------------------------ parts.json joints
def dart_parts():
    J = json.load(open(os.path.join(OUT, "parts.json")))
    return J


# ------------------------------------------------------------------ free flight (flight34 physics)
_FI = {}


def _flight_inputs():
    """flight34 module, the v34 mass case 'putty, bought x1.0' (mass34) and the out34 strip arrays, loaded once"""
    if not _FI:
        import mass34
        import flight34 as F
        rows = mass34.load(OUT)[0]
        _FI["v"] = (F, mass34.cases(rows)["putty, bought x1.0"], F.Air(OUT))
    return _FI["v"]


class Flight:
    """Pitch-plane rigid-body flight with the flight34 strip aero, axial drag and the 0.25 N payout drag on the tail tie
    (flight34 'tail eye x 152' point tow), gravity; integrated at 0.2 ms like flight34.fly, kept at every step."""

    def __init__(self, v0=6.8, T=0.25, w0=-0.4, dth=0.0):
        F, c, air = _flight_inputs()
        self.F = F
        self.mass = c["mass_g"] / 1000
        self.cg = np.asarray(c["cg_mm"], float) / 1000               # dart frame, m
        I = c["I_gmm2"] * 1e-9
        self.I = I
        self.c = c
        self.air = air
        self.body = F.Body(self.air, self.mass, c["cg_mm"], I[1, 1], "pitch")
        self.tow = dict(kind="point", x=152.0, z=0.0)
        self.v0, self.T, self.w0, self.dth = v0, T, w0, dth

    def run(self, p0, elev_deg, z_stop, dt=2e-4, tmax=2.0):
        """p0 = CG (x, z) at separation (world m). Returns rows (t, x, z, th, vx, vz, w) until the CG reaches z_stop."""
        F, b = self.F, self.body
        th = math.radians(elev_deg + self.dth)
        w = self.w0
        v = np.array([self.v0 * math.cos(math.radians(elev_deg)), self.v0 * math.sin(math.radians(elev_deg))])
        x, z, t = float(p0[0]), float(p0[1]), 0.0
        out = [(t, x, z, th, v[0], v[1], w)]
        while t < tmax and z > z_stop:
            Fv, Mn, e, n, U, vn0 = F.forces(b, v, w, th)
            Fv = Fv + np.array([0.0, -9.81 * b.M])
            u = -v / max(np.linalg.norm(v), 1e-9)
            r = F.tow_r(self.tow, b, u, e, n)
            Fv = Fv + self.T * u
            Mn += r[0] * self.T * u[1] - r[1] * self.T * u[0]
            v = v + Fv / b.M * dt
            x += v[0] * dt
            z += v[1] * dt
            w += Mn / b.I * dt
            th += w * dt
            t += dt
            out.append((t, x, z, th, v[0], v[1], w))
        return np.array(out)


class Flight3D(Flight):
    """flight34's two planes run together as one rigid body in 3D: the strip aero of the pitch plane (top-view widths,
    normal force along the body's up axis e3) and of the yaw plane (side-view widths, along its left axis e2) on the
    AIR-relative velocity (a steady cross-wind along world Y), one axial drag, gravity, and the 0.25 N payout drag on the
    tail tie along the trailing line (-v ground). Attitude by heading psi and pitch th; the dart has no roll mechanism in
    this model (axisymmetric strip aero, line on the axis), so roll stays 0. Body axes: e1 nose, e2 left, e3 = e1 x e2.
    With r0 = 0 and no wind it is flight34's pitch plane exactly."""

    def __init__(self, v0=6.8, T=0.25, q0=-0.4, r0=0.0, dth=0.0, dpsi=0.0, wind_y=0.0):
        super().__init__(v0=v0, T=T, w0=q0, dth=dth)
        self.by = self.F.Body(self.air, self.mass, self.c["cg_mm"], self.I[2, 2], "yaw")
        self.r0, self.dpsi, self.wind = r0, dpsi, np.array([0.0, wind_y, 0.0])

    @staticmethod
    def axes(th, psi):
        e1 = np.array([math.cos(th) * math.cos(psi), math.cos(th) * math.sin(psi), math.sin(th)])
        e2 = np.array([-math.sin(psi), math.cos(psi), 0.0])
        return e1, e2, np.cross(e1, e2)

    def run3(self, p0, elev_deg, stop, dt=2e-4, tmax=2.0):
        """p0 = CG (x, y, z) world at separation; stop(t, pos, th, psi) -> True ends the flight.
        Rows: t, x, y, z, th, psi, vx, vy, vz, q (pitch rate, nose up +), r (body yaw rate, nose to e2 +)."""
        F, bp, by = self.F, self.body, self.by
        th = math.radians(elev_deg + self.dth)
        psi = math.radians(self.dpsi)
        q, r = self.w0, self.r0
        el = math.radians(elev_deg)
        v = np.array([self.v0 * math.cos(el), 0.0, self.v0 * math.sin(el)])
        x = np.array(p0, float)
        t = 0.0
        g = np.array([0.0, 0.0, -9.81])
        tow_x, tow_z = self.tow["x"] / 1000, self.tow["z"] / 1000
        out = [(t, *x, th, psi, *v, q, r)]
        while t < tmax and not stop(t, x, th, psi):
            e1, e2, e3 = self.axes(th, psi)
            va = v - self.wind
            U = va @ e1
            Fp, Mp, *_ = F.forces(bp, np.array([U, va @ e3]), q, 0.0)
            Fy, My, *_ = F.forces(by, np.array([U, va @ e2]), r, 0.0)
            Ftot = Fp[0] * e1 + Fp[1] * e3 + Fy[1] * e2 + bp.M * g
            u = -v / max(np.linalg.norm(v), 1e-9)
            rr = (bp.cgx - tow_x) * e1 + (tow_z - bp.cgz) * e3
            Ft = self.T * u
            Mt = np.cross(rr, Ft)
            Ftot = Ftot + Ft
            Mp_t = Mp + Mt @ (-e2)
            My_t = My + Mt @ e3
            v = v + Ftot / bp.M * dt
            x = x + v * dt
            q += Mp_t / bp.I * dt
            r += My_t / by.I * dt
            th += q * dt
            psi += r / max(math.cos(th), 0.05) * dt
            t += dt
            out.append((t, *x, th, psi, *v, q, r))
        return np.array(out)

    @staticmethod
    def R_of(th, psi):
        return R_flight(th, 0.0, psi)

    @staticmethod
    def omega(th, psi, q, r):
        e1, e2, e3 = Flight3D.axes(th, psi)
        return -q * e2 + r * e3


def dart_cloud(J=None, step=6):
    """folded dart surface points (dart frame, m): hull, nose, pods, leg (for ground clearance at the handover)"""
    J = J or dart_parts()
    PP = {p["name"]: p for p in J["parts"]}
    pts = []
    for n in ("hull_top", "hull_bottom", "nose", "drill_leg_L", "drill_leg_R", "drill_leg_L_auger", "drill_leg_R_auger",
              "righting_leg", "coupler_L", "coupler_R", "pod_cap_L", "pod_cap_R"):
        if n not in PP:
            continue
        v = np.asarray(np.load(os.path.join(OUT, n + ".npz"))["v"], float)[::step] / 1000.0
        q = PP[n]
        off = np.zeros(3)
        while True:
            off += np.array(q["pos"], float) / 1000.0
            if not q.get("parent"):
                break
            q = PP[q["parent"]]
        pts.append(v + off)
    return np.vstack(pts)


# ------------------------------------------------------------------ the sim trace
class SimTrace:
    def __init__(self, path):
        d = json.load(open(path))
        self.meta, self.summary, self.cols = d["meta"], d["summary"], d["cols"]
        rows = d["rows"]
        ci = {c: i for i, c in enumerate(self.cols)}
        self.ci = ci
        self.phase = [r[ci["phase"]] for r in rows]
        num = [c for c in self.cols if c != "phase"]
        self.A = np.array([[r[ci[c]] for c in num] for r in rows], float)
        self.k = {c: i for i, c in enumerate(num)}
        self.t = self.A[:, self.k["t"]]
        self.cgX = self.meta["hull"]["cgX"] / 1000.0

    def col(self, c):
        return self.A[:, self.k[c]]

    def at(self, t):
        """interpolated (p body origin sim world, q sim body (w,x,y,z), leg, podL, podR, row index)"""
        t = min(max(t, self.t[0]), self.t[-1])
        i = int(np.clip(np.searchsorted(self.t, t) - 1, 0, len(self.t) - 2))
        u = (t - self.t[i]) / max(1e-12, self.t[i + 1] - self.t[i])
        a, b = self.A[i], self.A[i + 1]
        k = self.k
        p = (1 - u) * a[[k["px"], k["py"], k["pz"]]] + u * b[[k["px"], k["py"], k["pz"]]]
        q = slerp(a[[k["qw"], k["qx"], k["qy"], k["qz"]]], b[[k["qw"], k["qx"], k["qy"], k["qz"]]], u)
        lin = lambda c: (1 - u) * a[k[c]] + u * b[k[c]]
        return p, q, lin("leg_deg"), lin("podL_deg"), lin("podR_deg"), i

    def phase_times(self):
        out, last = [], None
        for t, p in zip(self.t, self.phase):
            if p != last:
                out.append((float(t), p))
                last = p
        return out


class SimMap:
    """sim world -> world: X = Rz(yaw) C (p_sim) + off. dart:root = world pose of the dart frame for a sim body pose."""

    def __init__(self, yaw, off):
        self.Rw = Rz(yaw) @ C_SIM
        self.off = np.asarray(off, float)

    def root(self, p_sim, q_sim, cgX):
        Rb = q2R(q_sim)
        R = self.Rw @ Rb @ M_D2S
        nose_sim = p_sim + Rb @ np.array([cgX, 0.0, 0.0])          # dart frame origin (nose tip) = body x +cgX
        return self.Rw @ nose_sim + self.off, R

    def point(self, p_sim):
        return self.Rw @ np.asarray(p_sim, float) + self.off


# ------------------------------------------------------------------ sim configuration from the dart build
V34_JOB = {"cfg": {"axle": "rigid", "tie": True, "legLen": 3},
           "D": {"hull": {"mass": 0.1905, "cgX": 83.3},
                 "pod": {"mass": 0.026, "pivotX": 132, "pivotY": 9.5, "stopDeg": 46.75, "lat": 31.15},
                 "leg": {"mass": 0.0055, "len": 113, "padW": 37, "pad": 3.5, "hingeX": 33, "hingeY": 14.5, "stopDeg": 178}},
           "LAYOUTS": {"tail": {"pivotX": 132}}, "cgUp": -2.6, "soil": "loam", "layout": "tail"}   # v34_tail_tie_worm.json T_rigid_tie


def sim_job(out=None):
    """righting sim configuration for a dart build: out34 = the validated v34 tail tie job; any other build (v35) from
    its parts.json (pod pivot, flank offset, fold direction, deploy stop, length; leg hinge and swing; tail tie) and
    its mass34 masses (hull = total less pods and leg)"""
    out = out or OUT
    name = os.path.basename(os.path.normpath(out))
    if name == "out34":
        return json.loads(json.dumps(V34_JOB))
    vj = os.path.join(KEDGE, "sim_righting", "v35_check", "F_rigid_tie.job")
    if name == "out35" and os.path.exists(vj):                      # the dart team's validated v35 righting / tow job
        J = json.load(open(vj))
        return {"cfg": J["cfg"], "D": J["D"], "LAYOUTS": J["LAYOUTS"], "cgUp": J["cgUp"], "soil": "loam",
                "layout": J.get("layout", "front"), "source": "kedge/sim_righting/v35_check/F_rigid_tie.job"}
    import mass34
    J = json.load(open(os.path.join(out, "parts.json")))
    P, PP = J["P"], {p["name"]: p for p in J["parts"]}
    rows = mass34.load(out)[0]
    g = mass34.case(rows)["groups"]                                   # hull / deploy (both pods) / leg masses and CGs
    pl = PP["drill_leg_L"]
    piv = np.array(pl["pos"], float)
    au = np.asarray(np.load(os.path.join(out, "drill_leg_L_auger.npz"))["v"], float)
    tip = au[np.argmax(np.linalg.norm(au, axis=1))]
    fold = 1 if tip[0] < 0 else -1                                   # folded tip toward the nose (+1, v34) or the tail (-1, v35)
    lg = PP["righting_leg"]
    layout = "tail" if fold == 1 else "front"
    # the sim puts each pod's mass at half its length from the pivot along the fold and the leg's 75 mm aft of its hinge:
    # place the hull's own CG so the whole dart's CG is the real one (as the validated v34 job does: cgX 83.3)
    tot = mass34.case(rows)
    Mt, ct = tot["mass_g"], np.asarray(tot["cg_mm"], float)
    mp, ml = g["deploy"]["mass_g"] / 2, g["leg"]["mass_g"]
    mh = Mt - 2 * mp - ml
    L = float(np.linalg.norm(tip))
    x_pod = piv[0] - fold * L / 2
    x_leg = float(lg["pos"][0]) + 75.0
    cgX = (Mt * ct[0] - 2 * mp * x_pod - ml * x_leg) / mh
    cgZ = (Mt * ct[2] - 2 * mp * piv[2] - ml * float(lg["pos"][2])) / mh
    job = {"cfg": {"axle": "rigid", "tie": True, "legLen": 3, "tieX": float(P["TIE"]["holes"][0][0]),
                   "tieY": float(P["TIE"]["holes"][0][2])},
           "D": {"hull": {"mass": round(mh / 1000, 4), "cgX": round(float(cgX), 2)},
                 "pod": {"mass": round(g["deploy"]["mass_g"] / 2000, 4), "pivotX": float(piv[0]), "pivotY": float(piv[2]),
                         "stopDeg": float(P["DEPLOY_DEG"]), "lat": abs(float(piv[1])), "len": round(float(np.linalg.norm(tip)), 2)},
                 "leg": {"mass": round(g["leg"]["mass_g"] / 1000, 4), "hingeX": float(lg["pos"][0]), "hingeY": float(lg["pos"][2]),
                         "stopDeg": float(P.get("LEG_DEG", 178.0)), "len": 113, "padW": 37, "pad": 3.5}},
           "LAYOUTS": {layout: {"pivotX": float(piv[0]), "fold": fold}}, "cgUp": round(float(cgZ), 2),
           "soil": "loam", "layout": layout}
    return job


def joint_map(out=None):
    """sim joint angle -> parts.json joint deg, and the auger spin axes (local, toward the tip) with their drill-in
    signs (from the flight helix: right-hand turns + about the tip direction)"""
    out = out or OUT
    J = json.load(open(os.path.join(out, "parts.json")))
    PP = {p["name"]: p for p in J["parts"]}
    au = np.asarray(np.load(os.path.join(out, "drill_leg_L_auger.npz"))["v"], float)
    f = 1 if au[np.argmax(np.linalg.norm(au, axis=1))][0] < 0 else -1      # pods fold toward the nose (v34) or the tail (v35)
    ax_pod = M_D2S.T @ np.array([0.0, 0.0, -f])
    ax_leg = M_D2S.T @ np.array([0.0, 0.0, -1.0])
    k_pod = float(np.sign(ax_pod[1])) * PP["drill_leg_L"].get("sgn", 1)
    k_leg = float(np.sign(ax_leg[1])) * PP["righting_leg"].get("sgn", 1)
    aug = {}
    for side in ("L", "R"):
        v = np.asarray(np.load(os.path.join(out, f"drill_leg_{side}_auger.npz"))["v"], float)
        c = v.mean(0)
        u, s_, vt = np.linalg.svd(v - c, full_matrices=False)
        a = vt[0] if vt[0] @ c > 0 else -vt[0]
        e1 = np.cross(a, [0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.cross(a, [0.0, 1.0, 0.0])
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(a, e1)
        s = v @ a
        r = np.linalg.norm(v - np.outer(s, a), axis=1)
        th = np.arctan2(v @ e2, v @ e1)
        lo, hi = s.min() + 0.45 * np.ptp(s), s.min() + 0.75 * np.ptp(s)       # flight zone (v34 D_HOLD 45..79 of 101)
        band = (s > lo) & (s < hi)
        m = band & (r > 0.85 * r[band].max())
        bins = np.linspace(lo, hi, 40)
        pts = []
        for b0, b1 in zip(bins[:-1], bins[1:]):
            k = m & (s >= b0) & (s < b1)
            if k.sum() > 3:
                pts.append((0.5 * (b0 + b1), np.angle(np.mean(np.exp(1j * th[k])))))
        pts = np.array(pts)
        slope = np.polyfit(pts[:, 0], np.unwrap(pts[:, 1]), 1)[0]
        a = np.where(np.abs(a) < 0.02, 0.0, a)
        a = a / np.linalg.norm(a)
        aug[side] = dict(axis=[round(float(x), 6) for x in a], drill_in_sign=1.0 if slope > 0 else -1.0)
    return dict(k_pod=k_pod, k_leg=k_leg, augers=aug)
