"""GrappStar base: carriage stop as a printed PLA flexure (D6/D26, no TPU). Rail frame (u, y, w), joint launch_rail.

Concept (per side, left shown, right mirrored in y):
- SPRING (one print): a stack of closed LOOPS lying flat in the band over the main spring (w 69.5..83.5, |y| 25..76.5).
  Each loop = two leaves spanning y, joined by end blocks at both ends; consecutive loops are joined by a centre
  block at y_mid. The load runs along the stack axis through the centre blocks, so every leaf half is a
  fixed-guided beam (inflection at its middle, no rotation, no lateral force). Leaves are constant-stress tapered
  (thickness ~ sqrt of the moment, 1.2 mm minimum at the inflections). The rear leaf of loop 1 is a rigid bar R
  with a contact pad at y_mid (the carriage arm hits only this pad); the front end is a rigid anchor block A.
- FRAME (one print): outboard bar with the outboard preload ledge, front wall (anchor seat), legs and two feet
  (rear foot bolted with 2 x M3 to the rail wall outer face under the spring, front foot with 2 x M3 down into the
  rail front plate).
- HARD STOP: the stack itself; every gap is cut to the closure at TRAVEL, so the stack goes solid there.
- RAIL ADD: inboard preload ledge (under the arm, behind R's inboard end).
- PRELOAD: R rests on both ledges (symmetric about y_mid) at u_stop; the anchor A is printed DELTA0 short of the
  front wall and two M3 draw screws pull it home, preloading the stack to F0. The carriage therefore settles exactly
  on the ledges at u_stop (F0 per side > half the main spring pull) and the stack is unloaded from the arm's view
  until the arm force exceeds F0.
Two prints per side (spring + frame) are needed: a preloaded spring cannot be printed in one piece with its cage.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from cad.lib import Part, Allow, prim, tf   # noqa: E402
import launch as LA                          # noqa: E402

E_PLA = 3500.0
SIGMA_MAX = 35.0

# ---------------------------------------------------------------------------------------------- design numbers
W0, W1 = 69.5, 83.5           # stack band (b = 14): over the springs (top w 68) and the D5 band (w <= 69), under the dart (84)
Y_IN, Y_OUT = 25.0, 76.5      # stack inboard / outboard faces (|y|); cradle |y| <= 24.4
E_BLK = 3.0                   # end block width (y)
C_HALF = 1.0                  # centre block half width (y)
Y_MID = 0.5 * (Y_IN + Y_OUT)  # 50.75: load line (arm pad, centre blocks, anchor)
A_HALF = 0.5 * ((Y_OUT - E_BLK) - (Y_IN + E_BLK)) - C_HALF   # free half span between blocks (21.75)
T0 = 3.15                     # leaf thickness at the blocks (moment peaks)
T_MIN = 1.0                   # leaf thickness at the inflections (shear about 8 MPa at peak)
N_LOOPS = 16                  # loop 1 = rigid R + one leaf, loops 2..16 = two leaves -> 31 flexible leaves
T_R, T_A_MIN = 6.0, 8.0       # rigid rear bar R / minimum anchor A thickness (u)
G_MIN = 0.6                   # printed gap where the closure is small
F0_TARGET = 32.0              # preload per side (64 N total > main spring pull at HOME, about 20 N)
TRAVEL = 23.8                 # travel to SOLID STACK (the hard stop): gaps are sized to close exactly here
U_FW = 240.15                 # frame front wall rear face (fixed: the spring guards are cut around the frame)
PAD_HALF = 4.0                # contact pad half width (y) around Y_MID, w 72..80
FRAME_T = 8.0                 # front wall thickness (u)


def _sg(side):
    return 1.0 if side == "l" else -1.0


def _bx(u0, u1, ya, yb, w0, w1, sg):
    y0, y1 = sorted((sg * ya, sg * yb))
    return prim.box(u0, u1, y0, y1, w0, w1)


# ---------------------------------------------------------------------------------------------- beam formulas
def _leaf_profile(n=2001):
    """Fixed-guided half span (length A_HALF, load P = F/2): returns x (from the end block), thickness, and the
    deflection v(x) per unit F (v = 0 at the end block, max at the centre block)."""
    a = A_HALF
    x = np.linspace(0.0, a, n)
    t = np.maximum(T_MIN, T0 * np.sqrt(np.abs(2.0 * x / a - 1.0)))
    M = 0.5 * (a / 2.0 - x)
    kap = M / (E_PLA * (W1 - W0) * t ** 3 / 12.0)
    th = np.concatenate([[0.0], np.cumsum(0.5 * (kap[1:] + kap[:-1]) * np.diff(x))])
    v = np.concatenate([[0.0], np.cumsum(0.5 * (th[1:] + th[:-1]) * np.diff(x))])
    return x, t, v


def n_leaves():
    return 2 * N_LOOPS - 1


def stop_calc(P=None):
    """Beam-formula numbers. Forces / stiffness / energy for BOTH units (left + right), stress and volume per unit.
    Travel is measured from u_stop (carriage on the ledges); the stack is already compressed DELTA0 there."""
    b = W1 - W0
    x, t, v = _leaf_profile()
    c_leaf = v[-1]                                   # mm/N, one leaf (both halves in parallel carry F/2 each)
    k_side = 1.0 / (c_leaf * n_leaves())
    delta0 = round(F0_TARGET / k_side, 2)
    f0 = k_side * delta0
    f1 = k_side * (delta0 + TRAVEL)
    e_side = 0.5 * (f0 + f1) * TRAVEL / 1000.0
    M = 0.5 * f1 * np.abs(A_HALF / 2.0 - x)
    sigma = float(np.max(6.0 * M / (b * t ** 2)))
    tau = 1.5 * (0.5 * f1) / (b * T_MIN)
    vol_leaves = n_leaves() * 2.0 * float(np.trapezoid(t, x) if hasattr(np, "trapezoid") else np.trapz(t, x)) * b / 1000.0
    d_leaf_full = c_leaf * f1
    return dict(k_n_per_mm=2.0 * k_side, travel_mm=TRAVEL, energy_j_at_travel=2.0 * e_side, peak_force_n=2.0 * f1,
                peak_stress_mpa=sigma, volume_cm3=vol_leaves,
                # extras
                k_side_n_per_mm=k_side, preload_mm=delta0, preload_force_n=2.0 * f0, f_side_peak_n=f1,
                n_leaves=n_leaves(), leaf_defl_full_mm=d_leaf_full, shear_mpa=tau,
                energy_stored_side_j=0.5 * f1 * (delta0 + TRAVEL) / 1000.0)


# ---------------------------------------------------------------------------------------------- stack geometry
def _stack_layout(n):
    """Sampled faces of the stack along y (|y|). Returns dict(ys, voids=[(u_lo, u_hi, ymask_kind)], u_front)."""
    c = stop_calc()
    d_full = c["leaf_defl_full_mm"] * 1.0
    x, t_x, v_x = _leaf_profile()
    phi_x = v_x / v_x[-1]
    ys = np.unique(np.concatenate([np.linspace(Y_IN, Y_OUT, 97),
                                   [Y_IN + E_BLK, Y_OUT - E_BLK, Y_MID - C_HALF, Y_MID + C_HALF]]))
    # half-span coordinate from the end block for each y
    xi = np.where(ys < Y_MID, ys - (Y_IN + E_BLK), (Y_OUT - E_BLK) - ys)
    xi_c = np.clip(xi, 0.0, A_HALF)
    phi = np.interp(xi_c, x, phi_x)
    t = np.interp(xi_c, x, t_x)
    t = np.where((xi <= 0.0) | (xi >= A_HALF), T0, t)
    u0 = n["u_stop"] + 0.8                          # R rear face (pad proud by 0.8 at y_mid)
    G, d = G_MIN, d_full
    gi = np.maximum(G, 2.0 * d * phi)                # intra-loop gap = closure at TRAVEL (solid stack there)
    ge = np.maximum(G, 2.0 * d * (1.0 - phi))        # inter-loop gap
    # leaf centrelines: R (rigid, flat), F1, then loops k = 2..N with mid planes LAM apart; each loop's two leaves
    # sit +-D1/2 about the mid plane, so all loops are congruent and the stack stays straight
    D1 = t + gi                                      # intra-loop centreline separation
    lam = float(np.max(2.0 * t + gi + ge))
    cF1 = u0 + T_R + np.maximum(G, d * phi) + t / 2.0
    m2 = float(np.max(cF1 + t + ge + D1 / 2.0))
    cl = [cF1]
    for k in range(2, N_LOOPS + 1):
        m = m2 + (k - 2) * lam
        cl += [m - D1 / 2.0, m + D1 / 2.0]
    a_rear = cl[-1] + t / 2.0 + np.maximum(G, d * (1.0 - phi))
    u_front = U_FW - c["preload_mm"]                 # anchor printed DELTA0 short of the front wall
    assert u_front - float(np.max(a_rear)) >= T_A_MIN, "stack too long for the frame"
    faces = [(np.full_like(ys, u0 + T_R), None)] + [(c - t / 2.0, c + t / 2.0) for c in cl]
    voids = []
    for j in range(len(cl)):
        lo = faces[j][0] if j == 0 else faces[j][1]
        hi = faces[j + 1][0]
        voids.append((lo, hi, "closed" if j % 2 == 0 else "open"))
    voids.append((faces[-1][1], a_rear, "open"))
    return dict(ys=ys, voids=voids, u0=u0, u_front=u_front, phi=phi, t=t)


def _void_solid(ys, lo, hi, kind, sg):
    if kind == "closed":
        m = (ys >= Y_IN + E_BLK - 1e-9) & (ys <= Y_OUT - E_BLK + 1e-9)
        segs = [m]
    else:
        segs = [ys <= Y_MID - C_HALF + 1e-9, ys >= Y_MID + C_HALF - 1e-9]
    out = []
    for m in segs:
        yy, a, b = ys[m], lo[m], hi[m]
        if kind == "open":                           # run out through the stack side faces
            if yy[0] <= Y_IN + 1e-9:
                yy, a, b = np.concatenate([[Y_IN - 1.0], yy]), np.concatenate([[a[0]], a]), np.concatenate([[b[0]], b])
            if yy[-1] >= Y_OUT - 1e-9:
                yy, a, b = np.concatenate([yy, [Y_OUT + 1.0]]), np.concatenate([a, [a[-1]]]), np.concatenate([b, [b[-1]]])
        pts = [(a[i], sg * yy[i]) for i in range(len(yy))] + [(b[i], sg * yy[i]) for i in range(len(yy) - 1, -1, -1)]
        out.append(prim.extrude(prim.poly_cs(pts), (W1 - W0) + 2.0, z=W0 - 1.0))
    return out


def spring_solid(n, side="l"):
    sg = _sg(side)
    lay = _stack_layout(n)
    us = n["u_stop"]
    s = _bx(lay["u0"], lay["u_front"], Y_IN, Y_OUT, W0, W1, sg)
    s = s - prim.union([v for (lo, hi, kd) in lay["voids"] for v in _void_solid(lay["ys"], lo, hi, kd, sg)])
    # contact pad (the only face the arm touches); R's inboard end sits on the rail ledge at w 69.5..71.5
    s = s + _bx(us, lay["u0"] + 0.01, Y_MID - PAD_HALF, Y_MID + PAD_HALF, 74.0, 80.0, sg)
    # draw screw pilots in the anchor (M3 self tap, 2.6)
    for dy in (-6.0, 6.0):
        s = s - LA.cyl_u(1.3, lay["u_front"] - 9.0, lay["u_front"] + 1.0, sg * (Y_MID + dy), 76.5)
    return s


def frame_geom(n):
    c = stop_calc()
    lay = _stack_layout(n)
    uF = U_FW                                        # front wall rear face (anchor printed DELTA0 short of it)
    return dict(uF=uF, u_front=lay["u_front"], u_r0=n["u_stop"] + 3.0, u_r1=n["u_stop"] + 15.0)


REAR_BOLTS = ((6.0, 44.0), (12.0, 38.5))             # (u - u_stop, w): M3 along y into the rail wall / shelf
FRONT_BOLTS_Y = (27.0, 42.0)                         # M3 down into the rail front plate (w 30..40)


def frame_solid(n, side="l"):
    sg = _sg(side)
    g = frame_geom(n)
    us, uF, r0, r1 = n["u_stop"], g["uF"], g["u_r0"], g["u_r1"]
    s = []
    s.append(_bx(us - 5.0, uF + 0.01, 77.0, 80.0, W0, W1, sg))                       # outboard bar
    s.append(_bx(us - 5.0, us + 0.8, Y_OUT - 3.5, 77.01, W0, W1, sg))                # outboard preload ledge
    s.append(_bx(uF, uF + FRAME_T, 24.9, 80.0, W0, W1, sg))                          # front wall (anchor seat)
    s.append(_bx(uF, uF + FRAME_T, 24.9, 29.5, 40.0, W0 + 0.01, sg))                 # front legs
    s.append(_bx(uF, uF + FRAME_T, 66.0, 80.0, 40.0, W0 + 0.01, sg))
    s.append(_bx(uF, uF + FRAME_T, 24.9, 80.0, 40.0, 50.5, sg))                      # front foot on the rail plate
    s.append(_bx(r0, r1, 72.0, 80.0, 30.5, 69.0, sg))                                # rear leg (stack slides over it)
    s.append(_bx(r0, r1, 77.0, 80.0, 68.9, W0 + 0.01, sg))
    s.append(_bx(r0, r1, 28.0, 34.0, 30.5, 48.0, sg))                                # rear foot plate (under the peg keeper, w 49)
    s.append(_bx(r0, r1, 28.0, 80.0, 30.5, 36.0, sg))                                # rear foot web
    f = prim.union(s)
    for du, w in REAR_BOLTS:
        f = f - LA.cyl_y(1.7, sg * 27.0, sg * 35.0, us + du, w)                      # clearance
        f = f - LA.cyl_y(3.5, sg * 71.0, sg * 81.0, us + du, w)                      # hex key access through the leg
    for y in FRONT_BOLTS_Y:
        f = f - LA.cyl_w(1.7, 39.0, 51.0, uF + FRAME_T / 2, sg * y)
        f = f - LA.cyl_w(3.1, 47.5, 51.0, uF + FRAME_T / 2, sg * y)                  # head counterbore (under the springs)
    for dy in (-6.0, 6.0):
        f = f - LA.cyl_u(1.7, uF - 1.0, uF + FRAME_T + 1.0, sg * (Y_MID + dy), 76.5)  # draw screws
    return f


# ---------------------------------------------------------------------------------------------- rail interface
def stop_rail_add(n):
    """Union into rail_solid (mid piece): inboard preload ledge (behind R's inboard end, under the arm), both sides.
    No separate hard stop: the stack goes solid at TRAVEL."""
    us = n["u_stop"]
    s = []
    for sg in (1.0, -1.0):
        s.append(_bx(us - 5.0, us + 12.0, 24.9, 28.4, n["lip_w1"] - 0.01, 69.0, sg))   # fin under the stack
        s.append(_bx(us - 5.0, us + 0.8, 24.9, 28.4, 68.9, 71.5, sg))                  # ledge behind R
    return prim.union(s)


def stop_rail_cut(n, P=None):
    """Cut from rail_solid: M3 self tap pilots (2.6) for the frame feet."""
    us = n["u_stop"]
    uF = frame_geom(n)["uF"]
    s = []
    for sg in (1.0, -1.0):
        for du, w in REAR_BOLTS:
            s.append(LA.cyl_y(1.3, sg * 18.0, sg * 28.5, us + du, w))
        for y in FRONT_BOLTS_Y:
            s.append(LA.cyl_w(1.3, n["w_floor0"] + 0.8, n["w_floor0"] + 10.5, uF + FRAME_T / 2, sg * y))
    return prim.union(s)


# carriage arm change (carriage LOCAL frame: u from the carriage rear face, same y, w). Boxes per side (y > 0 shown,
# mirror for the right arm), to UNION into carriage_solid in place of / in addition to the current arm box.
ARM_BOXES = (
    (0.0, 8.0, 12.0, 61.0, 72.0, 80.0),     # arm lengthened |y| 43 -> 61 (as for the D5 pegs at |y| 56)
    (-6.0, 8.0, 20.0, 61.0, 80.0, 83.0),    # taller to w 83 (dart flank features start at w 84 for |y| > 21.5)
    (-6.0, 0.0, 20.0, 50.0, 72.0, 80.0),    # 6 mm rearward stiffener (root moment 145 N x 33 mm -> about 13 MPa)
)
arm_change = dict(
    boxes_local=ARM_BOXES,
    contact="only the pad on R at |y| %.1f +- %.1f, w 72..80, at u_stop (front face of the arm)" % (Y_MID, PAD_HALF),
    hard_stop="none on the arm: the stack goes solid at u_stop + TRAVEL (gaps sized to the closure there)",
    note="union B(u0,u1,y0,y1,w0,w1) and its y mirror for each box; the front face stays at local u 8 (= u_stop at "
         "HOME); pegs must not stand proud of local u 8 (current u_peg_c 4, r 3: ok). Added mass about 4 g per arm.",
)


def arm_solid_local():
    return prim.union([LA.B(*b[:2], b[2], b[3], *b[4:]) + LA.B(*b[:2], -b[3], -b[2], *b[4:]) for b in ARM_BOXES])


# ---------------------------------------------------------------------------------------------- parts
def stop_parts(P, n=None):
    n = n or LA.L(P)
    c = stop_calc(P)
    out = []
    for side in ("l", "r"):
        sp = spring_solid(n, side)
        out.append(Part(f"launch_stop_spring_{side}", sp, "PLA", joint="launch_rail", place=tf.I4,
                        print_tf=LA.bed(sp, tf.Rx(180)), color="#e0b040", infill=1.0,
                        notes=f"Carriage stop flexure: {N_LOOPS} loops ({n_leaves()} tapered leaves t {T0}/{T_MIN} x 14), "
                              f"print upside down on the flat w 83.5 face, 100% infill / all perimeters; preload {c['preload_mm']:.1f} mm by 2 x "
                              f"M3x20 draw screws through the frame front wall"))
        fr = frame_solid(n, side)
        out.append(Part(f"launch_stop_frame_{side}", fr, "PLA", joint="launch_rail", place=tf.I4,
                        print_tf=LA.bed(fr, tf.Rx(180)), color="#7a8a99",
                        notes="Stop frame: outboard preload ledge, anchor seat, rear foot (2 x M3x12 into the rail wall), "
                              "front foot (2 x M3x16 into the rail front plate)"))
    return out


def stop_allowed():
    """Joints are face contacts (zero volume). One modelling artefact: in the overshoot states the stop spring is
    static (its compression is not modelled), so the SW_SEATED plunger foot (carriage, |y| <= 28, w 71.6..73.2,
    9.3 mm behind R's rear face) appears to enter R / the inboard end blocks. In reality R moves with the arm by the
    same d, so the foot stays 9.3 mm behind it (checked in test_launch_stop.py)."""
    return [Allow("launch_stop_spring_*", "launch_seat_plunger",
                  "overshoot states only: R moves with the arm, the plunger foot stays 9.3 mm behind it")]


# ---------------------------------------------------------------------------------------------- overshoot audit
def stop_audit_states(P):
    """Carriage overshoot past HOME (launch_travel negative = forward of HOME), 0..TRAVEL."""
    return [dict(aim_deg=25.0, launch_travel=-float(d), phase="launch_stop_overshoot") for d in np.linspace(0, TRAVEL, 7)]


def stop_overshoot_solids(P, n=None, d=0.0, side=None):
    """Solids (rail frame) to audit against the carriage at overshoot d (carriage at u_c_home + d).
    role 'contact': R moved by d, the arm may touch it ONLY on the pad face (plane u = u_stop + d);
    role 'keepout': the swept envelope of the compressed stack (R front face + d .. anchor), the frame, the rail
    ledges. No 'hardstop' role: the stack goes solid at TRAVEL."""
    n = n or LA.L(P)
    lay = _stack_layout(n)
    out = []
    for sd in ((side,) if side else ("l", "r")):
        sg = _sg(sd)
        R = _bx(n["u_stop"], lay["u0"] + T_R, Y_IN, Y_OUT, W0, W1, sg) ^ spring_solid(n, sd)
        out.append((f"stop_R_{sd}", R.translate((d, 0, 0)), "contact"))
        out.append((f"stop_stack_env_{sd}", _bx(lay["u0"] + T_R + d, lay["u_front"], Y_IN, Y_OUT, W0, W1, sg), "keepout"))
        out.append((f"stop_frame_{sd}", frame_solid(n, sd), "keepout"))
        us = n["u_stop"]
        out.append((f"stop_ledge_{sd}", _bx(us - 5.0, us + 12.0, 24.9, 28.4, n["lip_w1"], 69.0, sg)
                    + _bx(us - 5.0, us + 0.8, 24.9, 28.4, 68.9, 71.5, sg), "keepout"))
    return out
