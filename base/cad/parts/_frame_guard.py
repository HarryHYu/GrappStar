"""Guards (frame subsystem, private helper).

Cheek band: a 2.4 mm skin that closes the box between the two cheeks along the cheek outline (top, front, back), so
no finger reaches the gears and trains inside. It is split along the perimeter into segments that print standing up
(the band width, along Y, is the print height: no overhangs). Each segment carries two screw lugs per end: M3 through
the cheek from outside into an insert in the lug.

Openings: every part of the other subsystems that crosses the band in any audited state (rail, line, levers) is cut
out of the band with a margin (convex hull per state, grown by `margin` in every direction). The openings are where
the mechanism must pass; anything that can pinch there is the moving part's own guard (spring tube cover etc).
"""
from __future__ import annotations

import math

import numpy as np
from shapely.geometry import Polygon, LineString, Point, box as sbox
from shapely.ops import unary_union, substring

from cad.lib import Part, prim, tf
from . import _frame_util as U

T_BAND = 2.4
LUG = (12.0, 9.0, 9.0)      # along the outline, inward, along Y


def band_pieces(C, outline, z_min=8.0, max_len=230.0):
    """2D band segments (shapely polygons in (x, z)) along the cheek outline above z_min, and the lug centres
    (x, z, inward normal) per segment."""
    inner = outline.buffer(-T_BAND, join_style=1)
    ring2d = outline.difference(inner).intersection(sbox(-1e4, z_min, 1e4, 1e4))
    # perimeter curve of the outline above z_min, as an open line from the back bottom to the front bottom
    ext = LineString(list(outline.exterior.coords))
    L = ext.length
    pts = [ext.interpolate(i * L / 800.0) for i in range(800)]
    keep = [p.y >= z_min for p in pts]
    # rotate so the run starts at the first kept point after a gap
    if all(keep):
        start = 0
    else:
        start = next(i for i in range(800) if keep[i] and not keep[i - 1])
    run = []
    for k in range(800):
        i = (start + k) % 800
        if not keep[i]:
            break
        run.append((pts[i].x, pts[i].y))
    curve = LineString(run)
    n = max(1, int(math.ceil(curve.length / max_len)))
    segs, lugs = [], []
    for j in range(n):
        a, b = j * curve.length / n, (j + 1) * curve.length / n
        sub = substring(curve, a, b)
        # wedge region along this piece of curve: buffer the sub-curve wide, intersect with the ring
        reg = sub.buffer(T_BAND * 4, cap_style=2)
        piece = ring2d.intersection(reg)
        if j > 0:
            piece = piece.difference(Point(curve.interpolate(a)).buffer(0.2))
        if piece.geom_type != "Polygon":
            piece = max(piece.geoms, key=lambda g: g.area) if hasattr(piece, "geoms") else piece
        segs.append(piece)
        for d in (a + 10.0, b - 10.0):
            p = curve.interpolate(d)
            q = curve.interpolate(d + 0.5)
            tx, tz = q.x - p.x, q.y - p.y
            nl = math.hypot(tx, tz) or 1.0
            nx, nz = -tz / nl, tx / nl                       # left normal
            if not outline.contains(Point(p.x + nx * 5, p.y + nz * 5)):
                nx, nz = -nx, -nz                            # point inward
            lugs.append((j, p.x, p.y, nx, nz, tx / nl, tz / nl))
    return segs, lugs


def band_parts(C, outline, cutters=None):
    """Band segment Parts (world-in-deck-frame solids) and the cheek screw holes [(x, z)]."""
    y0, y1 = C["y_in"][-1] + 0.2, C["y_in"][1] - 0.2
    segs, lugs = band_pieces(C, outline)
    out, holes = [], []
    for j, pg in enumerate(segs):
        cs = prim.shapely_cs(pg)
        sol = prim.place(prim.extrude(cs, y1 - y0, z=0.0), tf.T(0, y1, 0) @ tf.Rx(90))   # (x, z) plane, y1 -> y0
        for (jj, x, z, nx, nz, tx, tz) in lugs:
            if jj != j:
                continue
            for yy, d in ((y0, 1), (y1, -1)):
                # lug block in a local frame: u along the tangent, v inward, w along y
                lx, lv, lw = LUG
                blk = prim.box(-lx / 2, lx / 2, 0.0, lv, 0.0, lw)
                M = np.eye(4)
                M[:3, 0] = (tx, 0, tz)
                M[:3, 1] = (nx, 0, nz)
                M[:3, 2] = (0, d, 0)
                M[:3, 3] = (x, yy, z)
                blk = prim.place(blk, M)
                cx, cz = x + nx * (lv / 2 + 1.0), z + nz * (lv / 2 + 1.0)
                ins = prim.place(U.insert_hole(0, 0, 0.0), tf.T(cx, yy, cz) @ (tf.Rx(90) if d > 0 else tf.Rx(-90)))
                sol = (sol + blk) - ins
                holes.append((cx, cz))
        if cutters is not None and not cutters.is_empty():
            sol = sol - cutters
            parts_ = sorted(sol.decompose(), key=lambda m: -m.volume())
            sol = parts_[0]
        out.append(sol)
    return out, holes


def swept_cutters(P, C, others=("launch", "reel", "drum"), margin=4.0, n_states=48):
    """Union of convex hulls (grown by margin) of every part of the other subsystems over the audit timeline, in the
    deck frame. Empty Manifold when the other subsystems are not there yet."""
    from manifold3d import Manifold
    from . import _frame_others as OTH
    try:
        from tools.timeline import timeline
        if others is None or set(others) >= {"launch", "reel", "drum"}:
            asm = OTH.load_others(P)
        else:
            from tools.assembly import Assembly, subsystem_names
            names = [n for n in subsystem_names() if n in others]
            asm = Assembly.load(P, names) if names else None
        if asm is None:
            return Manifold()
        states = timeline(P, 0.25)
    except Exception:
        return Manifold()
    if len(states) > n_states:
        idx = np.linspace(0, len(states) - 1, n_states).astype(int)
        states = [states[i] for i in idx]
    shift = tf.T(0, 0, -C["deck_z"])
    hulls = []
    offs = [(margin, 0, 0), (-margin, 0, 0), (0, margin, 0), (0, -margin, 0), (0, 0, margin), (0, 0, -margin)]
    hv = {}
    for n, p in asm.parts.items():                  # hull vertices once per part, thinned
        v = prim.mesh_of(p.solid.hull())[0]
        hv[n] = v[:: max(1, len(v) // 200)]
    Ws = [asm.world(S) for S in states]
    clouds = []
    chunk = 6                                       # one hull per part per 6 consecutive states (swept, conservative)
    for n in asm.parts:
        for k in range(0, len(Ws), chunk):
            pts = np.vstack([tf.apply(shift @ W[n], hv[n]) for W in Ws[k:k + chunk + 1]])
            grown = np.vstack([pts + np.asarray(o) for o in offs])
            clouds.append(grown)
            hulls.append(Manifold.hull_points(grown))
    LAST_CLOUDS[:] = clouds
    return prim.union(hulls)


LAST_CLOUDS = []          # point clouds of the last swept_cutters call (deck frame), for shadow()


def shadow(z0, z1, clouds=None):
    """XY shadow (shapely) of the swept hulls that reach between z0 and z1 (each hull projected whole: conservative)."""
    from shapely.geometry import MultiPoint
    from shapely.ops import unary_union
    cl = LAST_CLOUDS if clouds is None else clouds
    polys = []
    for c in cl:
        if c[:, 2].min() > z1 or c[:, 2].max() < z0:
            continue
        polys.append(MultiPoint(c[:, :2]).convex_hull)
    return unary_union(polys) if polys else None
