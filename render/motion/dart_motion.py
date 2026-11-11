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
    J = json.load(open(os.path.join(KEDGE, "out34", "parts.json")))
    return J


# ------------------------------------------------------------------ free flight (flight34 physics)
class Flight:
    """Pitch-plane rigid-body flight with the flight34 strip aero, axial drag and the 0.25 N payout drag on the tail tie
    (flight34 'tail eye x 152' point tow), gravity; integrated at 0.2 ms like flight34.fly, kept at every step."""

    def __init__(self, v0=6.8, T=0.25, w0=-0.4, dth=0.0):
        import mass34
        import flight34 as F
        self.F = F
        rows = mass34.load(os.path.join(KEDGE, "out34"))[0]
        c = mass34.cases(rows)["putty, bought x1.0"]
        self.mass = c["mass_g"] / 1000
        self.cg = np.asarray(c["cg_mm"], float) / 1000               # dart frame, m
        I = c["I_gmm2"] * 1e-9
        self.air = F.Air(os.path.join(KEDGE, "out34"))
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
