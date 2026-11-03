"""What the other subsystems put on the deck (frame subsystem, private helper).

The deck must cover every foot and give every foot screw a heat-set insert, whoever owns the foot. Rather than a
hand-kept list, the frame looks at the other subsystems' parts in the rest state:
- footprint: the slice of every part that reaches the deck top, 0.5 mm above it;
- screw holes: round holes of 3.0 to 4.0 mm in that slice (M3 clearance) get an insert in the deck below.
A subsystem may also define deck_holes(P) -> [(x, y), ...] (world) to add holes explicitly.

Everything is in the frame's deck frame (world shifted so the deck top is z = 0).
"""
from __future__ import annotations

import importlib
import math

import numpy as np
from shapely.geometry import Polygon

from cad.lib import prim, tf

_CACHE = {}
OTHERS_SKIP = {"frame", "demo"}


def load_others(P):
    """Assembly of every other subsystem module that defines parts/joints/kin (others are skipped), or None."""
    key = id(P)
    if key in _CACHE:
        return _CACHE[key]
    asm = None
    try:
        from tools.assembly import Assembly, subsystem_names
        mods = {}
        for n in subsystem_names():
            if n in OTHERS_SKIP or n.startswith("frame"):
                continue
            try:
                m = importlib.import_module(f"cad.parts.{n}")
            except Exception:
                continue
            if all(hasattr(m, f) for f in ("parts", "joints", "kin")):
                mods[n] = m
        if mods:
            asm = Assembly(P, mods)
    except Exception:
        asm = None
    _CACHE[key] = asm
    return asm


def classify_slice(solid, z):
    """Slice a solid at z: (footprint polygons, centres of M3 clearance-size round holes)."""
    polys, holes = [], []
    for r in solid.slice(z).to_polygons():
        r = np.asarray(r)
        if len(r) < 3:
            continue
        pg = Polygon(r).buffer(0)
        a = pg.area
        per = pg.length
        circ = 4 * math.pi * a / max(per * per, 1e-9)
        d = 2 * math.sqrt(a / math.pi)
        if circ > 0.85 and 2.9 <= d <= 4.1:
            holes.append((float(pg.centroid.x), float(pg.centroid.y)))
        elif a > 1.0:
            polys.append(pg)
    return polys, holes


def deck_contacts(P, deck_z, tol=0.6):
    """(footprints, holes, owners): shapely polygons and (x, y) hole centres of other subsystems' parts standing on
    the deck top (world z = deck_z), in deck-frame coordinates."""
    asm = load_others(P)
    polys, holes, owners = [], [], []
    if asm is None:
        return polys, holes, owners
    W = asm.world({})
    for n, p in asm.parts.items():
        s = prim.place(p.solid, W[n])
        lo, hi = prim.bbox(s)
        if not np.all(np.isfinite(lo)) or abs(lo[2] - deck_z) > tol:
            continue
        pp, hh = classify_slice(s, deck_z + 0.5)
        polys += pp
        owners += [n] * len(pp)
        holes += [(x, y, n) for x, y in hh]
    # keep only holes that sit inside some footprint (a pin would be its own polygon, not inside another)
    from shapely.geometry import Point
    from shapely.ops import unary_union
    U = unary_union(polys) if polys else None
    hl = []
    for (x, y, n) in holes:
        if U is not None and U.buffer(0.1).contains(Point(x, y)):
            hl.append((x, y))
    for mn, m in asm.modules.items():
        if hasattr(m, "deck_holes"):
            try:
                hl += [(float(x), float(y)) for x, y in m.deck_holes(P)]
            except Exception:
                pass
    # merge duplicates
    out = []
    for h in hl:
        if all(math.hypot(h[0] - o[0], h[1] - o[1]) > 1.0 for o in out):
            out.append(h)
    return polys, out, owners


def shadow(solid, z0, z1):
    """XY projection (shapely) of the part of a deck-frame solid between z0 and z1."""
    from shapely.ops import unary_union
    if solid is None or solid.is_empty():
        return None
    c = solid ^ prim.box(-1e4, 1e4, -1e4, 1e4, z0, z1)
    if c.is_empty():
        return None
    v, f = prim.mesh_of(c)
    tris = [Polygon(v[t][:, :2]) for t in f]
    tris = [t for t in tris if t.area > 1e-3]
    return unary_union(tris).buffer(0.5) if tris else None
