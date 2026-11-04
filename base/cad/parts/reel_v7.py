"""Harry's v7 capstan drum and wrap cup (originals/grappstar_reel_full_v7/full_v7.py) ported to manifold3d.

Helper of reel.py (worker cad-reel). Geometry follows v7 exactly where the line touches it; only the bores change:
the drum is keyed on the 20 mm capstan tube C (integral key rib) instead of the 8 mm D axle, and the cup is keyed
(D flat, rear half only, hidden) on the 10 mm cup shaft K instead of turning free. The lock-pin boss is gone (the
drum train holds the cup during tow, reel_NOTES.md section 7).

Frames (both: axis +Z):
  drum: base flange back face at z = 0, body to z = 3 + DL.
  cup:  rim plane z = 0, face z = CUP_D, eyelet ring at +Y, groove running +theta (CCW about +Z) from the ring
        down to the rim (theta from +X toward +Y).
D49 (the one change to Harry's cup): the ring hole is turned AXIAL, parallel to the cup axis at radius 26.0 (v7: tangential
at 24.5), on a lug in front of the face (z CUP_D .. CUP_D + 4); the groove starts at the face edge under the ring's rear
mouth. The line runs straight through the ring along the cup axis direction at radius 26, 2 mm outside the cup wall.
"""
from __future__ import annotations

import math

import numpy as np
from manifold3d import Manifold

from cad.lib import prim, tf

# v7 sizes (full_v7.py names)
LINE = 1.0
DR, DL = 20.0, 40.0                 # capstan root radius, body length
FLANGE_R, FLANGE_H = 23.0, 3.0      # base flange cone r 23 -> 20 over 3
CUP_IN, CUP_OUT, CUP_D = 20.6, 24.0, 14.0
CUP_FACE_T = 2.0                    # front wall
ER = 23.5                           # line centre radius in the groove (groove floor 23.0)
EZP = 14.5                          # ring axis height above the rim plane (0.5 in front of the face)
RING_L, RING_R = 2.0, 3.4           # ring half length (now axial: lug z CUP_D .. CUP_D + 2 RING_L), outside radius
RING_HOLE_R = 1.4                   # dia 2.8 line hole through the ring
RING_MOUTH_R = 2.0                  # dia 4.0 coned mouths
RING_AT = 26.0                      # D49: ring hole axis radius (axial hole), 2.0 outside the cup wall (r 24)
ALPHA = 30.0                        # groove helix angle from the rim plane
GW = 1.2                            # groove width across the line
G_FLOOR = 23.0                      # groove floor radius
PITCH = 2 * math.pi * CUP_OUT * math.tan(math.radians(ALPHA))   # 87.1 mm per turn
DPM = 360.0 / PITCH                                             # deg of wrap per mm of rise
TH_TOP = 90.0                                                   # D49: groove start under the axial ring's rear mouth
TH_RIM = TH_TOP + DPM * CUP_D                                   # groove crosses the rim here
# changes for the base
HUB_R = 7.5                         # cup hub (v7 7.0): holds the 10 mm K bore
RECESS_R = HUB_R + 0.6              # drum nose recess round the cup hub
RECESS_D = 7.3                      # recess depth: the cup hub stops 0.3 short of the floor


def _revolve(pts, seg=None):
    return prim.revolve_pts(pts, seg=seg or 160)


def drum(c_bore_d=20.3, key_w=6.4, key_depth=2.8, k_hole_r=5.4, web_t=2.4, mouth=None):
    """Capstan drum (PLA). Base flange cone r 23 -> 20 over 3, body r 20 to z 43, 1.2 round on the nose edge, 0.8
    chamfer at the flange back. Keyed bore for C (key on +X) from the back to the web, web with a hole for K,
    nose recess for the cup hub."""
    zt = FLANGE_H + DL
    c = 0.8                          # back chamfer
    f = 1.2                          # nose edge round (approximated by 4 facets)
    prof = [(0, 0), (FLANGE_R - c, 0), (FLANGE_R, c), (DR, FLANGE_H)]
    for i in range(5):
        a = math.radians(90 * i / 4)
        prof.append((DR - f + f * math.cos(a), zt - f + f * math.sin(a)))
    prof.append((0, zt))
    body = _revolve(prof)
    z_web0 = zt - RECESS_D - web_t
    bore = prim.cyl(c_bore_d / 2, z_web0 + 1, z=-1)
    key = prim.box(0, c_bore_d / 2 + key_depth, -key_w / 2, key_w / 2, -1, z_web0)
    if mouth is None:                                                   # v7 0.8 chamfer
        mouth = prim.cyl(c_bore_d / 2 + 0.8, 0.8, z=-0.01, r2=c_bore_d / 2)
    else:                                                               # (r at the back face, depth) 45 deg cone
        mouth = prim.cyl(mouth[0], mouth[1], z=-0.01, r2=mouth[0] - mouth[1])
    recess = prim.cyl(RECESS_R, RECESS_D + 1, z=zt - RECESS_D)
    rmouth = prim.cyl(RECESS_R, 0.81, z=zt - 0.8, r2=RECESS_R + 0.8)
    khole = prim.cyl(k_hole_r, web_t + 2, z=z_web0 - 1)
    return body - prim.union([bore, key, mouth, recess, rmouth, khole])


def drum_info():
    zt = FLANGE_H + DL
    return dict(z_top=zt, z_web0=zt - RECESS_D - 2.4, z_recess=zt - RECESS_D)


def _sphere(r, p, seg=16):
    return Manifold.sphere(r, seg).translate(tuple(p))


def _groove_cutter(n=56):
    """U groove: spheres of r GW/2 swept along the 30 deg helix, centres from r G_FLOOR + GW/2 out to r 25.6
    (open past the cup surface). theta(z) = TH_TOP + DPM (CUP_D - z), z from CUP_D + 0.5 down to -3."""
    rs = GW / 2
    r0, r1 = G_FLOOR + rs, 25.6
    zs = np.linspace(CUP_D + 0.5, -3.0, n)
    caps = []
    for z in zs:
        th = math.radians(TH_TOP + DPM * (CUP_D - z))
        c, s = math.cos(th), math.sin(th)
        caps.append([(r0 * c, r0 * s, z), (r1 * c, r1 * s, z)])
    hulls = []
    for a, b in zip(caps[:-1], caps[1:]):
        pts = [_sphere(rs, p, 12) for p in a + b]
        hulls.append(Manifold.batch_hull(pts))
    return prim.union(hulls)


def _helix_pt_dir(th_deg, z, r=23.6):
    th = math.radians(th_deg)
    ca, sa = math.cos(math.radians(ALPHA)), math.sin(math.radians(ALPHA))
    return np.array((r * math.cos(th), r * math.sin(th), z)), np.array((-math.sin(th) * ca, math.cos(th) * ca, -sa))


def _cone_along(r0, r1, p0, p1):
    v = np.asarray(p1, float) - np.asarray(p0, float)
    L = float(np.linalg.norm(v))
    M = tf.frame(tuple(p0), tuple(v / L))
    return prim.place(prim.cyl(r0, L, r2=r1), M)


def cup(k_d=10.0, bore_clear=0.3, flat=1.2):
    """Wrap cup (PLA), v7 geometry. Keyed D bore for K in the rear half (z 0 .. CUP_D/2), plain round bore at the
    front (Harry: visible holes plain round, D flats hidden). Print open end down, ring up."""
    rf = 1.0                                     # face edge round
    prof = [(0, 0), (CUP_OUT, 0)]
    for i in range(5):
        a = math.radians(90 * i / 4)
        prof.append((CUP_OUT - rf + rf * math.cos(a), CUP_D - rf + rf * math.sin(a)))
    prof.append((0, CUP_D))
    body = _revolve(prof)
    # D49 axial eyelet lug in front of the face (prints on top of the face, open end down); its root reaches into the
    # face to r 22.6 so it is one body with the cup
    ring = prim.cyl(RING_R, 2 * RING_L + 1.0, y=RING_AT, z=CUP_D - 1.0)
    body = prim.union([body, ring])
    # cavity over the drum (rim chamfer 1.0 on both edges) and the hub
    cav = prim.cyl(CUP_IN, CUP_D - CUP_FACE_T + 1, z=-1) - prim.cyl(HUB_R, CUP_D + 2, z=-1)
    cham_o = _revolve([(CUP_OUT - 1.0, -0.01), (CUP_OUT + 1, -0.01), (CUP_OUT + 1, 2.0)])
    cham_i = _revolve([(CUP_IN - 0.01, -0.01), (CUP_IN + 1.0, -0.01), (CUP_IN - 0.01, 1.0)])
    body = body - cav - cham_o - cham_i
    # bores: round front half, D rear half (flat on +X), coned mouths
    rb = (k_d + bore_clear) / 2
    front = prim.cyl(rb, CUP_D / 2 + 1, z=CUP_D / 2)
    rear = prim.cyl(rb, CUP_D / 2 + 1.01, z=-1) - prim.box(rb - flat, rb + 2, -rb - 2, rb + 2, -2, CUP_D / 2 + 0.01)
    m0 = prim.cyl(rb + 0.7, 0.71, z=-0.01, r2=rb)
    m1 = prim.cyl(rb, 0.71, z=CUP_D - 0.7, r2=rb + 0.7)
    body = body - prim.union([front, rear, m0, m1])
    # axial line hole through the lug with coned mouths (d 4.0); the rear mouth opens below the face into the free
    # space outside the cup wall, where the line comes from the spool
    z0, z1 = CUP_D, CUP_D + 2 * RING_L
    hole = prim.cyl(RING_HOLE_R, z1 - z0 + 8.0, y=RING_AT, z=z0 - 6.0)
    c0 = prim.cyl(RING_MOUTH_R, 0.61, y=RING_AT, z=z0 - 0.01, r2=RING_HOLE_R)
    c1 = prim.cyl(RING_HOLE_R, 0.61, y=RING_AT, z=z1 - 0.6, r2=RING_MOUTH_R)
    body = body - prim.union([hole, c0, c1])
    # funnels at both groove ends, then the groove LAST (Harry's rule)
    fun = []
    for th_deg, z, r0, r1 in ((TH_TOP, CUP_D, 1.6, 0.6), (TH_RIM, 0.0, 0.6, 1.8)):
        p, d = _helix_pt_dir(th_deg, z)
        fun.append(_cone_along(r0, r1, p - 3.0 * d, p + 3.0 * d))
    body = body - prim.union(fun)
    body = body - (_groove_cutter() ^ prim.box(-30, 30, -30, 30, -3, CUP_D + 0.5))
    return body


def cup_line_points():
    """Line centre points on the cup (cup frame): ring rear mouth, groove top (face edge), groove at the rim."""
    p_top, _ = _helix_pt_dir(TH_TOP, CUP_D, ER)
    p_rim, _ = _helix_pt_dir(TH_RIM, 0.0, ER)
    return dict(ring_front=np.array((0.0, RING_AT, CUP_D + 2 * RING_L)), ring_rear=np.array((0.0, RING_AT, CUP_D)),
                groove_top=p_top, groove_rim=p_rim)
