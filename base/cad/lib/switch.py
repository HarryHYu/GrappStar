"""Printed contact switch: a printed cantilever lever carries an M3 SHCS (head down) whose head face touches the face of
an M3 nut held in a fixed trap. Two wires: one ring-lugged under the jam nut on the screw, one soldered to the nut
(side slot) or ring-lugged under an optional M3 screw from below. Gap set by turning the screw (0.5 mm per turn),
locked by the jam nut. A hard stop under the lever tip limits the bend.

Frame (switch frame): X along the lever, x = 0 at the clamped root of the flexing beam, tip at x = lever_len;
Z up, base bottom at z = 0; Y across. Actuation pushes the lever tip down (-Z) at x_act (info).
The lever's flexible length runs from x = 0 to the screw boss; boss and tip are thick (taken as rigid).
Print the base flat; print the lever on its side (Y up) so layer lines run along the beam.
"""
import math

import numpy as np
from manifold3d import CrossSection

from . import fits, prim, tf

PLA_E = 3500.0            # N/mm2
STRAIN_LIMIT = 0.015      # repeated bending limit for printed PLA
M3_HEAD_DK = 5.5          # SHCS head (bought model)
M3_HEAD_K = 3.0
M3_HEAD_CHAMFER = 0.3     # edge chamfer on the head top (the contact face)
M3_NUT_S = 5.5            # nut across flats (model)
M3_NUT_M = 2.4            # nut height (model)
NUT_PRESS = 0.1           # trap across flats = nut + this (light press, add a drop of glue)
NUT_PROUD = 0.4           # nut face stands above the pillar top
WIRE_D = 1.6              # insulated wire OD the channel is sized for (26 AWG silicone ~1.4)
SCREW_LENGTHS = (6, 8, 10, 12, 16, 20)


def _nut(z0, hole=True):
    cs = CrossSection.circle(M3_NUT_S / math.sqrt(3), 6)
    n = prim.extrude(cs, M3_NUT_M, z=z0)
    return n - prim.cyl(1.5, M3_NUT_M + 2, z=z0 - 1) if hole else n


def _beam(Lf, xa, xs, EI):
    """Linear lever model: returns f(P, R) -> (delta_f, theta_f, M_root) for tip load P at xa (down) and contact
    reaction R at xs (up). Flexible from 0 to Lf, rigid beyond."""
    def f(P, R):
        V = P - R
        M = P * (xa - Lf) - R * (xs - Lf)
        df = V * Lf ** 3 / (3 * EI) + M * Lf ** 2 / (2 * EI)
        th = V * Lf ** 2 / (2 * EI) + M * Lf / EI
        return df, th, P * xa - R * xs
    return f


def contact_switch(lever_len, travel=0.8, overtravel=0.6, t=1.6, b=8.0, tip_off=9.0, root_len=14.0,
                   base_t=3.0, adj=1.0, boss_h=5.0, E=PLA_E, wire_d=WIRE_D):
    """Printed contact switch. lever_len: root to tip (mm, >= tip_off + 8). travel: deflection at the screw axis at
    which the head first touches the nut. overtravel: extra actuator movement after contact before the hard stop.
    t, b: lever beam thickness (bending direction) and width. adj: head clearance below the lever (set by screwing).
    Returns dict: base, lever (printed, switch frame, rest pose), screw, nut, jam_nut (bought models), info.
    info: gap (rest head to nut face, on the axis), pretravel (at the actuator), overtravel, force_close_N,
    force_stop_N, contact_force_N, strain_close, strain_max, x_screw, x_act, z_lever, stop_z, screw_len, ..."""
    xs = lever_len - tip_off
    boss_half = 4.0
    Lf = xs - boss_half
    if Lf < 6.0:
        raise ValueError("lever_len too short for tip_off: flexible length under 6 mm")
    xa = lever_len - 1.5
    I = b * t ** 3 / 12.0
    EI = E * I
    beam = _beam(Lf, xa, xs, EI)

    # heights
    pocket_af = M3_NUT_S + NUT_PRESS
    pocket_r = pocket_af / math.sqrt(3)
    pocket_depth = M3_NUT_M - NUT_PROUD
    z_pt = base_t + pocket_depth + 1.2                 # pillar top (1.2 floor under the nut)
    z_n = z_pt + NUT_PROUD                             # nut face
    rc = M3_HEAD_DK / 2 - M3_HEAD_CHAMFER              # contact rim radius on the head face
    z_rel_face = -(adj + M3_HEAD_K)                    # head face relative to the lever bottom

    def pose(P, R=0.0, z_lb=0.0):
        df, th, _ = beam(P, R)
        return tf.T(0, 0, -df) @ tf.R((0, 1, 0), math.degrees(th), origin=(Lf, 0, z_lb + t / 2))

    # closing load: axis point at xs drops by travel
    c_s = (lambda d: d[0] + d[1] * (xs - Lf))(beam(1.0, 0.0))
    P_close = travel / c_s
    rim = np.array([[xs + rc, 0.0, z_rel_face]])
    gap = float(rim[0, 2] - tf.apply(pose(P_close), rim)[0, 2])
    z_f = z_n + gap
    z_lb = z_f - z_rel_face

    # actuator pretravel and overtravel state (contact holds the axis point, actuator moves overtravel more)
    def dx(P, R, x):
        df, th, _ = beam(P, R)
        return df + th * (x - Lf)
    pre = dx(P_close, 0.0, xa)
    A = np.array([[dx(1, 0, xs), dx(0, 1, xs)], [dx(1, 0, xa), dx(0, 1, xa)]])
    P_ot, R_ot = np.linalg.solve(A, [travel, pre + overtravel])
    x_st1 = lever_len - 0.5
    pillar_half = pocket_r + 1.6
    x_st0 = min(xs + pillar_half + 1.0, x_st1 - 1.5)
    d_stop = dx(P_ot, R_ot, x_st1)
    stop_z = z_lb - d_stop
    P_free_stop = d_stop / dx(1.0, 0.0, x_st1)
    strain = lambda M: abs(M) * (t / 2) / EI
    s_close = strain(beam(P_close, 0)[2])
    s_stop = strain(beam(P_ot, R_ot)[2])
    s_free = strain(beam(P_free_stop, 0)[2])

    # base
    base_w = max(b, 2 * pillar_half) + 12.0
    x0, x1 = -root_len - 2.0, lever_len + 2.0
    base = prim.box(x0, x1, -base_w / 2, base_w / 2, 0, base_t)
    pad = prim.box(-root_len, 0, -b / 2, b / 2, base_t - 0.01, z_lb)
    pillar = prim.box(xs - pillar_half, xs + pillar_half, -pillar_half, pillar_half, base_t - 0.01, z_pt)
    stop = prim.box(x_st0, x_st1, -b / 2, b / 2, base_t - 0.01, stop_z)
    base = prim.union([base, pad, pillar, stop])
    cw, cd = 2 * wire_d + 0.4, wire_d + 0.4
    if cd > base_t - 0.8:
        raise ValueError("base_t too thin for the wire channel")
    cut = [
        prim.extrude(CrossSection.circle(pocket_r, 6), pocket_depth + 1, z=z_pt - pocket_depth),     # nut trap
        prim.cyl(fits.M3_CLEAR_D / 2, z_pt + 2, x=xs, z=-1),                                       # through hole
        prim.box(x0 - 1, xs - pillar_half + 0.5, -cw / 2, cw / 2, base_t - cd, base_t + 0.01),        # channel
        prim.box(xs - pillar_half - 0.01 - cd, xs, -(wire_d + 0.2) / 2, (wire_d + 0.2) / 2,
                 base_t - cd, z_pt + 1),                                                             # nut wire slot
    ]
    cut[0] = cut[0].translate((xs, 0, 0))
    hx = (-root_len * 0.25, -root_len * 0.75)
    for x in hx:                                       # lever clamp screws, self tap into the pad
        cut.append(prim.cyl(fits.M3_TAP_D / 2, z_lb - base_t - 0.8 + 1, x=x, z=base_t + 0.8))
    for x in (-root_len / 2, Lf / 2):                  # base mounting holes
        for y in (-1, 1):
            cut.append(prim.cyl(fits.M3_CLEAR_D / 2, base_t + 2, x=x, y=y * (base_w / 2 - 3.5), z=-1))
    base = prim.diff(base, cut)

    # lever
    root_h = 4.0
    lever = prim.union([
        prim.box(-root_len, 0.01, -b / 2, b / 2, z_lb, z_lb + root_h),
        prim.box(0, Lf + 0.01, -b / 2, b / 2, z_lb, z_lb + t),
        prim.box(Lf, lever_len, -b / 2, b / 2, z_lb, z_lb + boss_h),
    ])
    lcut = [prim.cyl(fits.M3_TAP_D / 2, boss_h + 2, x=xs, z=z_lb - 1)]
    for x in hx:
        lcut.append(prim.cyl(fits.M3_CLEAR_D / 2, root_h + 2, x=x, z=z_lb - 1))
    lever = prim.diff(lever, lcut)

    # bought models
    need = adj + boss_h + M3_NUT_M + 1.0
    screw_len = next((L for L in SCREW_LENGTHS if L >= need), SCREW_LENGTHS[-1])
    head = prim.revolve_pts([(0, 0), (rc, 0), (M3_HEAD_DK / 2, M3_HEAD_CHAMFER), (M3_HEAD_DK / 2, M3_HEAD_K),
                             (0, M3_HEAD_K)], seg=48)
    screw = (head + prim.cyl(1.5, screw_len, z=M3_HEAD_K - 0.01, seg=24)).translate((xs, 0, z_f))
    nut = _nut(z_n - M3_NUT_M).translate((xs, 0, 0))
    jam = _nut(z_lb + boss_h).translate((xs, 0, 0))

    info = {
        "travel": travel, "gap": gap, "pretravel": pre, "overtravel": overtravel,
        "force_close_N": P_close, "force_stop_N": P_ot, "contact_force_N": R_ot,
        "force_free_stop_N": P_free_stop, "strain_close": s_close, "strain_max": max(s_stop, s_free),
        "strain_ok": max(s_stop, s_free) <= STRAIN_LIMIT, "x_screw": xs, "x_act": xa, "x_flex": Lf,
        "z_lever": z_lb, "z_nut_face": z_n, "z_head_face": z_f, "stop_z": stop_z, "stop_x": (x_st0, x_st1),
        "stop_defl": d_stop, "screw_len": screw_len, "screw": f"M3x{screw_len} SHCS ISO 4762 + M3 jam nut",
        "nut_screw_max_len": z_n - 0.3, "turns_per_mm": 2.0, "E": E, "t": t, "b": b,
        "notes": "Beam theory, flexible 0..x_flex, boss and tip rigid. Rim contact first (lever tilts). "
                 "Optional M3 screw from below into the nut for a ring lug: length <= nut_screw_max_len.",
    }
    return {"base": base, "lever": lever, "screw": screw, "nut": nut, "jam_nut": jam, "info": info}


def lever_pose(info, defl):
    """4x4 rigid pose (switch frame) of the lever boss, screw and jam nut when the screw axis has moved down by defl
    (mm, no contact). The thin beam part bends and is not rigid; use this for the boss end only."""
    t, Lf, xs, xa = info["t"], info["x_flex"], info["x_screw"], info["x_act"]
    EI = info["E"] * info["b"] * t ** 3 / 12.0
    beam = _beam(Lf, xa, xs, EI)
    df1, th1, _ = beam(1.0, 0.0)
    P = defl / (df1 + th1 * (xs - Lf))
    df, th, _ = beam(P, 0.0)
    return tf.T(0, 0, -df) @ tf.R((0, 1, 0), math.degrees(th), origin=(Lf, 0, info["z_lever"] + t / 2))
