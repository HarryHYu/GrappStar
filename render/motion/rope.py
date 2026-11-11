"""Tether rope for the render: position-based (verlet) rope of 1 mm Dyneema from the dart's tail knot (node 0, pinned)
to the base exit (last node, pinned: the pusher hole or the muzzle lip roller tangent). Gravity, quadratic air drag
(terminal speed about 2.9 m/s for 0.6 g/m, d 1 mm), ground contact with friction at z = GROUND_Z. The paid-out free
length L is set by the caller each step (payout, spool overrun, take-up); line enters / leaves at the BASE end: the
segment next to the exit (the feed segment) takes the length change, and nodes are inserted or removed there, so line
that is already out stays where it lies."""
from __future__ import annotations

import numpy as np

SEG = 0.05            # m, rest length of the interior segments
LINE_R = 0.0006       # m, line radius
GROUND_Z = LINE_R + 0.0015
G = np.array([0.0, 0.0, -9.81])
K_DRAG = 9.81 / 2.9 ** 2      # quadratic drag coefficient (1/m)
MU_GROUND = 0.6
TIGHT = 0.97          # end-to-end distance over the free length above which every segment is held at length


def project(x, r, iters=6, bilateral=False):
    """Fast projection (Goldenthal et al. 2007) of a chain onto its inextensibility constraints, tension only: the
    stretched segments are the active set, the Lagrange multipliers come from one tridiagonal solve per Newton step, and
    a segment that would push (slack) leaves the set. bilateral: every segment held at its rest length (a line that is
    nearly tight, where the slack form converges too slowly). Both end nodes are pinned (inverse mass 0)."""
    from scipy.linalg import solve_banded
    n = len(x)
    m = n - 1
    if m < 1:
        return
    w = np.ones(n)
    w[0] = w[-1] = 0.0
    active = np.ones(m, bool)
    for _ in range(iters):
        d = x[1:] - x[:-1]
        ln = np.maximum(np.linalg.norm(d, axis=1), 1e-12)
        u = d / ln[:, None]
        C = ln - r
        act = np.ones(m, bool) if bilateral else (active & (C > 1e-7))
        if not act.any():
            return
        diag = w[:-1] + w[1:]
        off = -w[1:-1] * np.einsum("ij,ij->i", u[:-1], u[1:])          # between segment i and i+1 (shared node i+1)
        diag = np.where(act, diag, 1.0)
        diag = np.where(diag < 1e-12, 1.0, diag)
        offa = np.where(act[:-1] & act[1:], off, 0.0)
        ab = np.zeros((3, m))
        ab[0, 1:] = offa
        ab[1] = diag
        ab[2, :-1] = offa
        rhs = np.where(act, -C, 0.0)
        lam = solve_banded((1, 1), ab, rhs)
        push = act & (lam > 1e-12) if not bilateral else np.zeros(m, bool)
        if push.any():
            active = active & ~push
            lam = np.where(push, 0.0, lam)
        lam = np.where(act, lam, 0.0)
        dx = np.zeros_like(x)
        dx[:-1] += -u * lam[:, None]
        dx[1:] += u * lam[:, None]
        x += w[:, None] * dx
        if np.abs((np.linalg.norm(x[1:] - x[:-1], axis=1) - r)[act]).max() < 1e-6:
            return


class Rope:
    def __init__(self, knot, exit_pt):
        knot, exit_pt = np.asarray(knot, float), np.asarray(exit_pt, float)
        self.x = np.array([knot, exit_pt])
        self.xp = self.x.copy()
        self.L = float(np.linalg.norm(exit_pt - knot))

    def n(self):
        return len(self.x)

    def rest(self):
        n = len(self.x)
        r = np.full(n - 1, SEG)
        r[-1] = max(1e-6, self.L - SEG * (n - 2))
        return r

    def _regrid(self):
        """insert / remove nodes at the exit end so the feed segment stays within 0.5..1.5 SEG"""
        while True:
            n = len(self.x)
            feed = self.L - SEG * (n - 2)
            if feed > 1.5 * SEG:
                a, b = self.x[-2], self.x[-1]
                d = b - a
                ln = np.linalg.norm(d)
                p = b - d / ln * min(0.5 * ln, SEG) if ln > 1e-9 else b.copy()
                vel = self.x[-2] - self.xp[-2]                    # the new node starts with its neighbour's motion
                self.x = np.vstack([self.x[:-1], p, self.x[-1:]])
                self.xp = np.vstack([self.xp[:-1], p - vel, self.xp[-1:]])
                continue
            if feed < 0.5 * SEG and n > 2:
                self.x = np.vstack([self.x[:-2], self.x[-1:]])
                self.xp = np.vstack([self.xp[:-2], self.xp[-1:]])
                continue
            break

    def step(self, dt, knot, exit_pt, L, iters=6, ground=True):
        knot, exit_pt = np.asarray(knot, float), np.asarray(exit_pt, float)
        self.L = float(max(L, 1e-4))
        self._regrid()
        x, xp = self.x, self.xp
        v = (x - xp) / max(dt, 1e-9)
        sp = np.linalg.norm(v, axis=1, keepdims=True)
        a = G - K_DRAG * sp * v
        xn = x + v * dt + a * dt * dt
        if ground:
            on = xn[:, 2] <= GROUND_Z + 1e-4
            # ground friction: kill most of the tangential velocity of nodes resting on the ground
            if on.any():
                vt = (xn[on] - x[on])
                vt[:, 2] = 0.0
                xn[on, :2] = x[on, :2] + vt[:, :2] * (1.0 - MU_GROUND)
        xn[0] = knot
        xn[-1] = exit_pt
        r = self.rest()
        tight = float(np.linalg.norm(exit_pt - knot)) > TIGHT * self.L
        for _ in range(3):
            project(xn, r, iters=iters, bilateral=tight)
            if ground:
                np.maximum(xn[:, 2], GROUND_Z, out=xn[:, 2])
            xn[0] = knot
            xn[-1] = exit_pt
        self.xp = x
        self.x = xn

    def length(self):
        return float(np.linalg.norm(np.diff(self.x, axis=0), axis=1).sum())

    def stretch(self):
        """polyline length / rest length - 1 (positive = the solver could not hold it: the line is pulled tight)"""
        return self.length() / max(self.L, 1e-9) - 1.0

    def points(self, k):
        """k points from the exit to the knot, evenly by arc length"""
        p = self.x[::-1]
        s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
        if s[-1] < 1e-9:
            return np.repeat(p[:1], k, axis=0)
        q = np.linspace(0, s[-1], k)
        return np.stack([np.interp(q, s, p[:, j]) for j in range(3)], axis=1)
