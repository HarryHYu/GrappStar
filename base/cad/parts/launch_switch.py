"""Printed contact switch (no microswitch): a flex leaf carries an M3 screw whose head touches a captured M3 nut.

Local frame (all in mm):
- base plate z 0..BASE_T on the mounting surface, x from x0 to x1, y -W/2..W/2, two M3 holes (self tap into the
  mount: 2.6 pilot in the mount, 3.4 clear in the base);
- leaf: thin plate x 0..LEAF_T, rising from the base to z_top; the M3 screw is threaded through it along x at z_s;
- head on the +x side, nut captured in a post at +x; the actuator pushes the screw TIP (on the -x side) towards +x;
- modelled PRESSED (head face on the nut face). Free, the leaf springs back TRAVEL towards -x.
Wiring: one wire under the nut (nut trap has a slot for a stripped wire end, clamped by the nut), one wire on the
screw (ring terminal or wrap under a second nut on the screw tip side is not needed: wrap the wire round the leaf screw
before tightening). ESP32 pin with internal pull-up, firmware debounce.
"""
from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from cad.lib import prim  # noqa: E402

BASE_T = 4.0
LEAF_T = 1.2
W = 10.0
HEAD_D, HEAD_H = 5.5, 3.0
NUT_AF, NUT_T = 5.5, 2.4
TRAVEL = 1.2


def info(screw_len=6.0):
    return dict(tip_x=LEAF_T - screw_len - 0.0, head_x0=LEAF_T, head_x1=LEAF_T + HEAD_H, nut_x0=LEAF_T + HEAD_H)


def body(z_s=9.0, screw_len=6.0, x0=-3.0, holes=True, width=None, top=5.0):
    """Base + leaf + nut post (one printed PLA part). Returns the solid."""
    z_top = z_s + top
    Wd = width or W
    xn = LEAF_T + HEAD_H                     # nut face
    x1 = xn + NUT_T + 6.0 + 14.0             # post + room for two mounting holes
    base = prim.box(x0, x1, -Wd / 2, Wd / 2, 0, BASE_T)
    leaf = prim.box(0, LEAF_T, -Wd / 2 + 1, Wd / 2 - 1, BASE_T - 0.01, z_top)
    post = prim.box(xn, xn + NUT_T + 6.0, -Wd / 2, Wd / 2, BASE_T - 0.01, z_top)
    s = base + leaf + post
    # screw tap hole in the leaf, clearance in the post behind the nut, nut trap open at the top (drop the nut in)
    s = s - prim.cylX(1.3, -1, LEAF_T + 1, y=0, z=z_s)
    s = s - prim.cylX(1.8, xn - 0.5, xn + NUT_T + 7.0, y=0, z=z_s)
    hexr = NUT_AF / 2 / 0.8660 + 0.15
    nut_cut = prim.place(prim.cyl(hexr, NUT_T + 0.3, seg=6), _rot_y90())
    nut_cut = nut_cut.translate((xn, 0, z_s))
    s = s - nut_cut - prim.box(xn, xn + NUT_T + 0.3, -NUT_AF / 2 - 0.15, NUT_AF / 2 + 0.15, z_s, z_top + 1)
    # wire slot beside the nut
    s = s - prim.box(xn - 0.01, xn + NUT_T + 0.3, -1.0, 1.0, z_s - hexr - 2.0, z_s)
    if holes:
        for hx in mount_holes(x0, z_s):
            s = s - prim.cyl(1.7, BASE_T + 2, x=hx, z=-1)
    return s


def _rot_y90():
    import numpy as np
    from cad.lib import tf
    return tf.Ry(90)


def screw(screw_len=6.0, z_s=9.0):
    """M3 SHCS, head on +x, pressed position."""
    h = prim.cylX(HEAD_D / 2, LEAF_T, LEAF_T + HEAD_H, y=0, z=z_s)
    sh = prim.cylX(1.5, LEAF_T - screw_len, LEAF_T + 0.01, y=0, z=z_s)
    return h + sh


def nut(z_s=9.0):
    xn = LEAF_T + HEAD_H
    hexr = NUT_AF / 2 / 0.8660
    n = prim.place(prim.cyl(hexr, NUT_T, seg=6), _rot_y90()).translate((xn, 0, z_s))
    return n - prim.cylX(1.5, xn - 1, xn + NUT_T + 1, y=0, z=z_s)


def mount_holes(x0=-3.0, z_s=9.0):
    """x positions (local) of the base mounting holes."""
    xn = LEAF_T + HEAD_H
    x1 = xn + NUT_T + 6.0 + 14.0
    return [x1 - 3.5, x1 - 10.0]


# ---------------------------------------------------------------------------------------------------------------------
# Lip lever contact (SW_HOME on the right rail lip, SW_SEATED on the left): sized with cad/lib/switch.py's beam model.
#
# Lever frame (all mm): x along the lever, x = 0 at the clamped root of the flexing beam (front face of the root block),
# z = 0 at the lever bottom, z up (the rail w). Root block x -LIP_ROOT_LEN..0 sits on the lip (one M3 self tap),
# thin beam (t) x 0..x_flex, rigid boss x_flex..lever_len (M3 tapped through at x_screw), a raised PAD on the boss tip
# at x_act (the actuator foot presses only the pad). M3 x 6 SHCS head DOWN under the boss: its head face is the contact,
# on the face of an M3 nut held in the lip below. Modelled and printed at REST (unpressed): the head face stands the
# contact gap above the nut face. Pressing the pad by the pretravel closes the contact; the actuator goes on by the
# overtravel (the beam bends round the head contact). The actuators are position bound (SW_HOME: rigid yoke foot;
# SW_SEATED: plunger foot, its travel bounded by the pushrod collar and the pusher face), so the foot bottom is the hard
# stop: the strain is also checked with the screw backed right off (pad pressed by pretravel + overtravel, no contact).
# ---------------------------------------------------------------------------------------------------------------------
from cad.lib import switch as SWL  # noqa: E402

LIP_X_SCREW = 25.0        # flexible root to the screw axis
LIP_TIP_OFF = 11.5        # screw axis to the lever tip (pad centre at tip - 1.5, as switch.py's x_act)
LIP_T = 0.8               # beam thickness: switch.py T_MIN (lowest actuator force; the strain is checked below)
LIP_B = 6.0               # lever width (fits on the 6 mm wall top)
LIP_ROOT_LEN = 5.0        # root block length (M3 self tap at its middle)
LIP_ROOT_H = 3.0          # root block height above the lever bottom
LIP_BOSS_H = 5.0          # boss height (M3 x 6 with the head ADJ below the lever: tip flush with the boss top)
LIP_ADJ = 1.0             # head top below the lever bottom (nominal; the gap is set by turning the screw)
LIP_PAD_W, LIP_PAD_H = 1.5, 1.2   # actuator pad (along x, height above the boss top)
LIP_SCREW_LEN = 6.0       # M3 x 6 SHCS
LIP_TRAVEL = 0.43         # axis travel to contact (switch.py `travel`): rim gap 0.50 = one turn of the M3 screw
LIP_OVERTRAVEL = 0.25     # actuator travel after contact (switch.py `overtravel`), at the pad
LIP_CLEAR = 0.003         # modelling clearance at the head/nut and pad/foot contacts in the pressed pose


def lip_lever_info(travel=LIP_TRAVEL, overtravel=LIP_OVERTRAVEL, xs=LIP_X_SCREW, tip_off=LIP_TIP_OFF, t=LIP_T, b=LIP_B,
                   E=SWL.PLA_E):
    """Lever sizing (lever frame) with cad/lib/switch.py's linear beam model (flexible 0..x_flex, boss rigid).
    Returns dict: t, x_flex, x_screw, x_act, lever_len, gap (rest head face to nut face at the first contact point, the
    rim), pretravel (pad travel to contact), overtravel, forces, strain_close / strain_ot / strain_free / strain_max,
    strain_ok (STRAIN_LIMIT 0.008), z_face (head face at rest), pad_top, and the pressed rigid pose (pivot, angle)."""
    import math
    import numpy as np
    from cad.lib import tf
    lever_len = xs + tip_off
    Lf = xs - 4.0
    xa = lever_len - 1.5
    if t is None:
        t = SWL.auto_thickness(lever_len, travel, overtravel, tip_off)
    EI = E * b * t ** 3 / 12.0
    beam = SWL._beam(Lf, xa, xs, EI)

    def dx(P, R, x):
        df, th, _ = beam(P, R)
        return df + th * (x - Lf)

    def pose(P, R=0.0):
        df, th, _ = beam(P, R)
        return tf.T(0, 0, -df) @ tf.R((0, 1, 0), math.degrees(th), origin=(Lf, 0, t / 2))

    rc = HEAD_D / 2
    z_face = -(LIP_ADJ + HEAD_H)
    P_close = travel / dx(1.0, 0.0, xs)
    rim = np.array([[xs + rc, 0.0, z_face]])
    gap = float(rim[0, 2] - tf.apply(pose(P_close), rim)[0, 2])
    pre = dx(P_close, 0.0, xa)
    A = np.array([[dx(1, 0, xs), dx(0, 1, xs)], [dx(1, 0, xa), dx(0, 1, xa)]])
    P_ot, R_ot = (float(v) for v in np.linalg.solve(A, [travel, pre + overtravel]))
    P_free = (pre + overtravel) / dx(1.0, 0.0, xa)
    strain = lambda M: abs(M) * (t / 2) / EI      # noqa: E731
    s_close = strain(beam(P_close, 0.0)[2])
    s_ot = strain(beam(P_ot, R_ot)[2])
    s_free = strain(beam(P_free, 0.0)[2])
    s_lib = float(SWL._strain_max(lever_len, travel, overtravel, t, tip_off))
    s_max = max(s_ot, s_free, s_lib)

    # pressed rigid stand-in for the kinematics: one rotation (about +y, tip down) about a pivot on the beam mid plane
    # that puts the head rim LIP_CLEAR above the nut face and drops the pad's highest (rear) top corner by pretravel +
    # overtravel. It is the boss pose of the beam model at full overtravel; the thin beam and the root block follow it
    # rigidly in the model (the root lifts off the lip in the pressed pose: a modelling stand-in for the beam bend).
    boss_top = LIP_BOSS_H
    pad_top = boss_top + LIP_PAD_H
    zp = t / 2

    def drop(x, z, xp, phi):
        s, c = math.sin(phi), math.cos(phi)
        return z - (zp - (x - xp) * s + (z - zp) * c)

    pw = LIP_PAD_W / 2
    xp, phi = xs - travel / max(1e-9, (pre + overtravel - travel) / (xa - xs)), (pre + overtravel - travel) / (xa - xs)
    for _ in range(50):
        f = np.array([drop(xs + rc, z_face, xp, phi) - (gap - LIP_CLEAR),
                      drop(xa - pw, pad_top, xp, phi) - (pre + overtravel)])
        if np.abs(f).max() < 1e-10:
            break
        J = np.zeros((2, 2))
        for k, (dxp, dph) in enumerate(((1e-6, 0.0), (0.0, 1e-8))):
            f2 = np.array([drop(xs + rc, z_face, xp + dxp, phi + dph) - (gap - LIP_CLEAR),
                           drop(xa - pw, pad_top, xp + dxp, phi + dph) - (pre + overtravel)])
            J[:, k] = (f2 - f) / (dxp or dph)
        xp, phi = np.array([xp, phi]) - np.linalg.solve(J, f)
    pad_drop = min(drop(xa - pw, pad_top, xp, phi), drop(xa + pw, pad_top, xp, phi))   # highest pad corner
    pad_dx = (xa - pw - xp) * (math.cos(phi) - 1.0) + (pad_top - zp) * math.sin(phi)   # its shift along x (pressed)
    return dict(t=t, b=b, E=E, x_flex=Lf, x_screw=xs, x_act=xa, lever_len=lever_len, travel=travel, gap=gap,
                pretravel=pre, overtravel=overtravel, force_close_N=P_close, force_ot_N=P_ot, contact_force_N=R_ot,
                force_free_N=P_free, strain_close=s_close, strain_ot=s_ot, strain_free=s_free, strain_lib=s_lib,
                strain_max=s_max, strain_limit=SWL.STRAIN_LIMIT, strain_ok=s_max <= SWL.STRAIN_LIMIT,
                z_face=z_face, boss_top=boss_top, pad_top=pad_top, pivot=(float(xp), float(zp)),
                pressed_deg=math.degrees(phi), pad_drop=float(pad_drop), pad_dx=float(pad_dx),
                turns=gap * 2.0, screw=f"M3x{LIP_SCREW_LEN:g} SHCS")


def lip_lever_solid(info, z_root0):
    """Printed lever (PLA) in the lever frame at rest. z_root0: lever-frame z of the lip top (root block bottom)."""
    t, Lf, xs, xa, L = info["t"], info["x_flex"], info["x_screw"], info["x_act"], info["lever_len"]
    hb = LIP_B / 2
    root = prim.box(-LIP_ROOT_LEN, 0.01, -hb, hb, z_root0, LIP_ROOT_H)
    beam = prim.box(0, Lf + 0.01, -hb, hb, 0, t)
    boss = prim.box(Lf, L, -hb, hb, 0, LIP_BOSS_H)
    pad = prim.box(xa - LIP_PAD_W / 2, xa + LIP_PAD_W / 2, -hb, hb, LIP_BOSS_H - 0.01, info["pad_top"])
    s = prim.union([root, beam, boss, pad])
    s = s - prim.cyl(1.3, LIP_BOSS_H + 2, x=xs, z=-1)                                    # M3 tap (2.6)
    s = s - prim.cyl(1.7, LIP_ROOT_H - z_root0 + 2, x=-LIP_ROOT_LEN / 2, z=z_root0 - 1)    # root screw clearance
    return s


def lip_screw_solid(info):
    """M3 x 6 SHCS (bought) in the lever frame at rest: head down, face info['z_face'], tip flush with the boss top."""
    xs, zf = info["x_screw"], info["z_face"]
    head = prim.cyl(HEAD_D / 2, HEAD_H, x=xs, z=zf)
    shank = prim.cyl(1.5, LIP_SCREW_LEN + 0.01, x=xs, z=zf + HEAD_H - 0.01)
    return head + shank
