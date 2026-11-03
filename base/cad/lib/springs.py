"""Extension spring anchors (Century spring, OD 16, hook ends), a spring envelope model and a force helper.

Century spring wire not measured: CENTURY_WIRE = 2.0 assumed, so the hook ID is about 16 - 2 x 2 = 12.
Hooks are modelled as full loops over the centre in the plane XZ (the post through a hook runs along Y).
"""
import math

import numpy as np

from . import fits, prim, tf

CENTURY_OD = 16.0
CENTURY_WIRE = 2.0            # assumed, caliper it
POST_CLEAR = 0.8              # post OD under the hook ID (diametral)
POST_GROOVE_DEPTH = 1.0       # radial
POST_GROOVE_PLAY = 0.6        # groove width over the wire d
PLA_ALLOW_MPA = 20.0          # bending stress allowed in a printed PLA post (static, about 1/3 of strength)


def _post_dims(hook_id, wire, clear, groove_depth, lip_h, height):
    pd = hook_id - clear
    core = pd - 2 * groove_depth
    gw = wire + POST_GROOVE_PLAY
    z1 = height - lip_h               # groove top
    z0 = z1 - gw
    return pd, core, gw, z0, z1


def anchor_post(height=12.0, hook_id=None, wire=CENTURY_WIRE, clear=POST_CLEAR, groove_depth=POST_GROOVE_DEPTH,
                lip_h=1.6, base="screw", base_d=None, base_t=3.0, n_screws=3):
    """Spring anchor post. Frame: axis +Z, base bottom at z = 0 (the host face), post top at z = base_t + height
    (base=None: post alone from z = 0 to height). Post OD = hook_id - clear (hook drops over it), groove (core
    OD - 2 groove_depth, width wire + 0.6) just under a lip of height lip_h so the loaded hook cannot ride off;
    top chamfer. base: "screw" = round flange d base_d with n_screws M3 counterbored clearance holes; "insert" =
    flange plus an M3 heat-set hole up the post axis from the bottom (screw from under the host); None = post only
    with an M3 heat-set hole in the bottom. hook_id default CENTURY_OD - 2 wire."""
    hook_id = CENTURY_OD - 2 * wire if hook_id is None else hook_id
    pd, core, gw, z0, z1 = _post_dims(hook_id, wire, clear, groove_depth, lip_h, height)
    zb = 0.0 if base is None else base_t
    c = min(0.8, lip_h * 0.5)
    pts = [(0, 0), (pd / 2, 0), (pd / 2, z0), (core / 2, z0 + groove_depth * 0.3), (core / 2, z1 - groove_depth * 0.3),
           (pd / 2, z1), (pd / 2, height - c), (pd / 2 - c, height), (0, height)]
    post = prim.revolve_pts(pts, seg=prim.segs(pd / 2)).translate((0, 0, zb - (0.01 if base else 0)))
    if base is None:
        return post - prim.cyl(fits.M3_INSERT_D / 2, fits.M3_INSERT_L + 0.5, z=-1, seg=24)
    bd = pd + 2 * (fits.M3_HEAD_D + 2.0) if base_d is None else base_d
    flange = prim.cyl(bd / 2, base_t) - prim.chamfer_ring(bd / 2, base_t, 0.6)
    s = prim.union([flange, post])
    cuts = []
    if base == "screw":
        rpc = (pd / 2 + bd / 2) / 2
        for i in range(n_screws):
            a = 2 * math.pi * i / n_screws
            x, y = rpc * math.cos(a), rpc * math.sin(a)
            cuts.append(prim.cyl(fits.M3_CLEAR_D / 2, base_t + 2, x=x, y=y, z=-1, seg=24))
    elif base == "insert":
        cuts.append(prim.cyl(fits.M3_INSERT_D / 2, fits.M3_INSERT_L + 1, z=-1, seg=24))
    else:
        raise ValueError("base must be 'screw', 'insert' or None")
    return prim.diff(s, cuts)


def anchor_post_info(height=12.0, hook_id=None, wire=CENTURY_WIRE, clear=POST_CLEAR,
                     groove_depth=POST_GROOVE_DEPTH, lip_h=1.6, base="screw", base_t=3.0, force=None):
    """Derived numbers: post_d, core_d, groove z range (z0, z1 in the post frame), z of the hook plane (groove
    centre), and if force (N) is given the root bending stress (MPa) of the post loaded at the groove centre."""
    hook_id = CENTURY_OD - 2 * wire if hook_id is None else hook_id
    pd, core, gw, z0, z1 = _post_dims(hook_id, wire, clear, groove_depth, lip_h, height)
    zb = 0.0 if base is None else base_t
    out = {"post_d": pd, "core_d": core, "groove_w": gw, "groove_z": (zb + z0, zb + z1),
           "hook_z": zb + (z0 + z1) / 2, "hook_id": hook_id, "lip_over_core": (pd - core) / 2}
    if force is not None:
        arm = (z0 + z1) / 2
        sigma = 32 * force * arm / (math.pi * pd ** 3)
        out.update(stress_mpa=sigma, stress_ok=sigma <= PLA_ALLOW_MPA)
    return out


def extension_spring_model(od=CENTURY_OD, wire=CENTURY_WIRE, free_len=60.0, extended_len=None, seg=32):
    """Extension spring envelope for renders and audit (bought). Frame: spring axis +Z, hook loops in the XZ plane
    (posts through the hooks run along Y). Length L = extended_len or free_len, measured inside hooks: the inner
    edge of the bottom hook at z = 0 and of the top hook at z = L. Body: solid cylinder od; hooks: full loops (wire d,
    mean dia od - wire) over the centre."""
    L = free_len if extended_len is None else extended_len
    rm = (od - wire) / 2
    ri = rm - wire / 2
    body_len = L - 2 * (rm + ri)
    if body_len <= 0:
        raise ValueError("spring too short for its hooks")
    zb0 = ri + rm
    body = prim.cyl(od / 2, body_len, z=zb0, seg=seg)
    torus = prim.revolve(prim.circle_cs(wire / 2, rm, 0, seg=12), seg=seg)       # in XY about Z
    loop = prim.place(torus, tf.Rx(90))                                          # into XZ, about Y
    lo = loop.translate((0, 0, ri))
    hi = loop.translate((0, 0, L - ri))
    return prim.union([body, lo, hi])


def spring_info(od=CENTURY_OD, wire=CENTURY_WIRE, free_len=60.0):
    """Hook geometry: hook_id, loop centre offset from the inner edge (ri), body length at free length."""
    rm = (od - wire) / 2
    ri = rm - wire / 2
    return {"hook_id": 2 * ri, "loop_r_mean": rm, "loop_center_from_edge": ri, "body_len": free_len - 2 * (rm + ri)}


def spring_force(k, x, initial_tension=0.0):
    """Extension spring force (N): initial_tension + k x for extension x >= 0 (mm, k in N/mm), 0 when slack
    (x < 0). Works on numpy arrays."""
    x = np.asarray(x, float)
    f = np.where(x >= 0, initial_tension + k * x, 0.0)
    return float(f) if f.ndim == 0 else f
