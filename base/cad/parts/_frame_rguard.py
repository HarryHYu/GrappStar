"""Frame-owned guards that ride on cad-launch's rail (helper of frame.py). The guards hang on the frame joint
`frame_rail_guard`, a copy of cad-launch's `launch_rail` joint whose value comes from launch.kin, so a frame-only
audit does not need the launch subsystem loaded.

Spring guards: a C-channel over each launch spring (top plate above the carriage spring arms, outer wall beside the
spring, bottom flange against the rail wall's outer face). The inner side stays open for the carriage arm; the dart
covers the middle. Split at the rail piece splits so each piece prints standing up (profile on the bed, no overhang).
Fixing: 2 x M3 per piece through the flange into the rail wall (cad-launch: an M3 insert hole at each point listed by
`fix_points(P)`, horizontal into the wall outer face).
"""
from __future__ import annotations

import math

import numpy as np

from cad.lib import Part, Allow, Clearance, prim, tf
from . import _frame_util as U

T = 2.4            # guard wall
GAP = 3.0          # clearance to the spring and the carriage arms


def _n(P):
    try:
        from . import launch
        return launch.L(P)
    except Exception:
        return None


def profile(n):
    """C profile in the rail (y, w) plane for the +y side: list of boxes (y0, y1, w0, w1)."""
    yw1 = n["y_wall1"]                                   # rail wall outer face
    ys, ws, ro = n["y_spring"], n["w_spring"], n["spring_od"] / 2
    w_top = max(n["lip_w1"] + 14.0, ws + ro) + GAP       # above the carriage arms and the coils
    y_out = ys + ro + GAP                                 # outer wall inner face
    w_fl = n["w_floor0"] + 4.0                            # bottom flange, below the coils
    return [(yw1 - 0.0, y_out + T, w_top, w_top + T),     # top
            (y_out, y_out + T, w_fl, w_top + T),          # outer wall
            (yw1, y_out + T, w_fl, w_fl + T)], dict(w_top=w_top, y_out=y_out, w_fl=w_fl)


def pieces_u(n):
    u0 = n["u_track0"] + 4.0
    u1 = n["u_anchor"] - 12.0                             # stop short of the anchor pin
    cuts = [c for c in n["split"] if u0 + 30 < c < u1 - 30]
    edges = [u0] + list(cuts) + [u1]
    return [(edges[i] + (0.2 if i else 0.0), edges[i + 1] - (0.2 if i < len(edges) - 2 else 0.0)) for i in range(len(edges) - 1)]


def fix_points(P):
    """(u, y, w) in the launch_rail frame of every guard screw (2 per piece per side)."""
    n = _n(P)
    if n is None:
        return []
    prof, d = profile(n)
    pts = []
    for (ua, ub) in pieces_u(n):
        for u in (ua + 15.0, ub - 15.0):
            for sg in (1, -1):
                pts.append((u, sg * n["y_wall1"], d["w_fl"] + 6.0))
    return pts


_CACHE = {}
SKIP = ("launch_spring", "launch_dart")      # coils and dart run free under the guard (collision audit checks them)
SKIP_EXACT = {"launch_carriage"}               # the carriage body passes under the guard top (its arms and yoke are carved)


def _cutters(P):
    """Launch parts in the launch_rail frame: exact solids of parts fixed to the rail, grown swept hulls of parts
    that move relative to it (aim housing, aim lug, gears)."""
    key = id(P)
    if key in _CACHE:
        return _CACHE[key]
    from manifold3d import Manifold
    from tools.assembly import Assembly
    from tools.timeline import timeline
    asm = Assembly.load(P, ["launch"])
    states = timeline(P, 0.5)
    states = [states[i] for i in np.linspace(0, len(states) - 1, min(40, len(states))).astype(int)] + list(asm.extra_states)
    exact, hulls = [], []
    rel = {}
    for S in states:
        JW = asm.joint_world(S)
        Rinv = tf.inv(JW["launch_rail"])
        W = asm.world(S)
        for nme, p in asm.parts.items():
            if nme.startswith(SKIP) or nme in SKIP_EXACT:
                continue
            rel.setdefault(nme, []).append(Rinv @ W[nme])
    offs = np.array([(3, 0, 0), (-3, 0, 0), (0, 3, 0), (0, -3, 0), (0, 0, 3), (0, 0, -3)], float)
    for nme, Ms in rel.items():
        p = asm.parts[nme]
        if all(np.allclose(M, Ms[0], atol=1e-6) for M in Ms):
            # grown by 0.05 mm (4 shifted copies) so no guard face ends up coplanar with a rail face (watertight STL)
            exact.append(prim.union([prim.place(p.solid, tf.T(*d) @ Ms[0]) for d in
                                     ((0, 0.05, 0), (0, -0.05, 0), (0, 0, 0.05), (0, 0, -0.05))]))
        else:
            v = prim.mesh_of(p.solid.hull())[0]
            v = v[:: max(1, len(v) // 150)]
            if all(np.allclose(M[:3, :3], Ms[0][:3, :3], atol=1e-6) for M in Ms):
                pts = np.vstack([tf.apply(M, v) for M in Ms])       # pure slide (carriage family): one hull = exact sweep
                hulls.append(Manifold.hull_points(np.vstack([pts + o for o in offs])))
                continue
            for k in range(0, len(Ms), 4):
                pts = np.vstack([tf.apply(M, v) for M in Ms[k:k + 5]])
                hulls.append(Manifold.hull_points(np.vstack([pts + o for o in offs])))
    out = prim.union(exact + hulls)
    _CACHE[key] = out
    return out


def parts(P):
    n = _n(P)
    if n is None:
        return []
    prof, d = profile(n)
    try:
        cut = _cutters(P)
    except Exception:
        cut = None
    out = []
    for k, (ua, ub) in enumerate(pieces_u(n)):
        for sg, side in ((1, "l"), (-1, "r")):
            boxes = prof + [(n["y_wall1"], n["y_wall1"] + 6.0, d["w_fl"], d["w_fl"] + 12.0)]   # screw strip (full length)
            sol = prim.union([prim.box(ua, ub, sg * y0, sg * y1, w0, w1) for (y0, y1, w0, w1) in boxes])
            for u in (ua + 15.0, ub - 15.0):
                sol = sol - prim.cylY(U.CLR_D / 2, min(sg * n["y_wall1"], sg * (d["y_out"] + T)) - 1,
                                      max(sg * n["y_wall1"], sg * (d["y_out"] + T)) + 1, u, d["w_fl"] + 6.0)
            if cut is not None and not cut.is_empty():
                sol = sol - cut
                comps = sorted(sol.decompose(), key=lambda m: -m.volume())
                sol = comps[0]
            nm = f"frame_spring_guard_{side}_{k + 1}"
            out.append(Part(nm, sol, "PLA", joint="frame_rail_guard", print_tf=U.print_flat(sol, tf.Ry(-90)), color="#d0d6dc",
                            notes="spring guard (C-channel over the launch spring), 2 x M3 x 10 into the rail wall"))
    return out


def joints(P):
    from cad.lib import Joint
    try:
        from . import launch
        j = next(j for j in launch.joints(P) if j.name == "launch_rail")
    except Exception:
        return []
    return [Joint("frame_rail_guard", None, j.kind, origin=j.origin, axis=j.axis, base=j.base, limits=j.limits)]


def kin(P, S):
    try:
        from . import launch
        v = launch.kin(P, S).get("launch_rail")
    except Exception:
        v = None
    return {} if v is None else {"frame_rail_guard": float(v)}


def allowed(P):
    return [Allow("frame_spring_guard_*", "launch_rail*", "guard flange and lugs bear on the rail wall outer face", max_vol=1.0),
            Allow("frame_spring_guard_*", "frame_spring_guard_*", "pieces butt at the rail splits", max_vol=1.0)]


def clearances(P):
    return []          # guard vs springs and carriage: the whole-base collision audit covers them (a Clearance naming
                       # launch parts would fail in a frame-only audit)


def steps(P):
    names = [p.name for p in parts(P)]
    if not names:
        return []
    return [dict(title="Spring guards",
                 text="After the springs are hooked on: lay each guard over its spring, flange against the rail wall, "
                      "M3 x 10 through each lug into the rail wall insert. Cock by hand once with the wiper unplugged: "
                      "the carriage arms must pass under the guard tops without touching.",
                 parts=names)]
