"""Frame helpers: fastener cutters, bearing seats, flat-plate splitting with keyed joints, print transforms.

Private module of the frame subsystem (leading underscore: the assembly does not load it as a subsystem).
All lengths mm. Fasteners: M3 everywhere (M3 x 4 x 5 heat-set inserts), M8 axles for 608s.
"""
from __future__ import annotations

import math

import numpy as np
from manifold3d import Manifold

from cad.lib import prim, tf, fits

# ------------------------------------------------------------------------------------------------ fasteners
INS_D = fits.M3_INSERT_D          # 4.0 hole for an M3 x 4 x 5 insert
INS_L = fits.M3_INSERT_L + 1.0    # 6.0 deep: insert 5 + 1 for melt and screw tip
CLR_D = fits.M3_CLEAR_D           # 3.4
HEAD_D = fits.M3_HEAD_D           # 6.1 counterbore
HEAD_H = fits.M3_HEAD_H + 0.2     # 3.2
NUT_AF = fits.M3_NUT_AF           # 5.8
NUT_H = fits.M3_NUT_H             # 2.7
INS_WALL = 3.0                    # minimum plastic round an insert (radial)


def insert_hole(x, y, z_top, depth=INS_L):
    """Blind hole for an M3 heat-set insert entering at z_top going -Z, with a 0.3 mm lead-in chamfer.
    Rotate the result for other directions."""
    h = prim.cyl(INS_D / 2, depth + 0.01, x, y, z_top - depth, seg=24)
    lead = prim.cyl(INS_D / 2, 0.61, x, y, z_top - 0.6, r2=INS_D / 2 + 0.3, seg=24)
    return h.__add__(lead).__add__(prim.cyl(INS_D / 2 + 0.3, 0.5, x, y, z_top - 0.001, seg=24))


def clear_hole(x, y, z0, z1, head_top=None, head_h=HEAD_H):
    """M3 clearance hole along Z from z0 to z1, optional counterbore for an SHCS head ending at head_top."""
    c = prim.cyl(CLR_D / 2, z1 - z0 + 0.02, x, y, z0 - 0.01, seg=24)
    if head_top is not None:
        c = c.__add__(prim.cyl(HEAD_D / 2, head_h + 0.01, x, y, head_top - head_h, seg=32))
    return c


def nut_trap(x, y, z0, h=NUT_H, rot=0.0):
    """Hex pocket for an M3 nut, from z0 up h (flats across NUT_AF)."""
    r = NUT_AF / 2 / math.cos(math.pi / 6)
    return Manifold.cylinder(h, r, r, 6).rotate((0, 0, 30 + rot)).translate((x, y, z0))


def slot_cs(x0, y0, x1, y1, w):
    """Stadium (slot) cross-section from (x0, y0) to (x1, y1), width w."""
    from shapely.geometry import LineString
    return prim.shapely_cs(LineString([(x0, y0), (x1, y1)]).buffer(w / 2, 24))


# ------------------------------------------------------------------------------------------------ bearing seats
def brg_dims(P, kind):
    """(od, w, bore) for a bearing kind: '608', 'sprag' or a dict(od, w, bore)."""
    if isinstance(kind, dict):
        return kind["od"], kind["w"], kind.get("bore", 8.0)
    if kind == "608":
        b = getattr(P, "B608", dict(bore=8.0, od=22.0, w=7.0))
        return b["od"], b["w"], b["bore"]
    if kind == "sprag":
        s = getattr(P, "SPRAG", dict(od=47.0, w=14.0, bore=20.0))
        return s["od"], s["w"], s["bore"]
    if kind == "693":
        b = getattr(P, "F693ZZ", dict(bore=3.0, od=8.0, w=4.0))
        return b["od"], b["w"], b["bore"]
    raise ValueError(f"unknown bearing kind {kind!r}")


def lip_for(od, shaft_d=0.0):
    """Lip (shoulder) hole: the library default that leaves the inner race free, never smaller than shaft + 1."""
    from cad.lib import bearings as LB
    base = {LB.B608_OD: LB.B608_LIP_D, LB.CSK20_OD: LB.CSK20_LIP_D, LB.B693_OD: LB.B693_LIP_D}.get(od, od - 6.0)
    return max(base, shaft_d + 1.0)


def bearing_seat_cutter(od, w, z0, ribs=8, lip_id=None, depth_extra=0.0):
    """Cutter for a crush-rib bearing pocket opening at z0 and going +Z (depth w + depth_extra), with a lip
    (shoulder) hole of lip_id beyond. Uses the shared library (cad.lib.bearings): seat_608 for od 22, seat_csk20
    (with its anti-rotation key) for od 47, ribbed_seat otherwise."""
    from cad.lib import bearings as LB
    depth = w + depth_extra
    if abs(od - LB.B608_OD) < 1e-6:
        c = LB.seat_608(depth=depth, lip_d=lip_id or LB.B608_LIP_D, below=60.0)
    elif abs(od - LB.CSK20_OD) < 1e-6:
        c = LB.seat_csk20(depth=depth, lip_d=lip_id or LB.CSK20_LIP_D, below=60.0)
    else:
        c = LB.ribbed_seat(od, depth, n_ribs=ribs, lip_d=lip_id or (od - 6.0), below=60.0)
    return prim.place(c, tf.T(0, 0, z0) @ tf.Rx(180))


# ------------------------------------------------------------------------------------------------ keyed plate split
def dovetail_seam_cs(axis, c, lo, hi, n, depth=8.0, w_root=10.0, w_tip=14.0, gap=0.0, avoid=()):
    """2D region (CrossSection) on the '+' side of a seam at axis value c (axis 'x' or 'y'), running from lo to hi
    along the other axis, with n dovetail tails sticking from the '+' side into the '-' side. gap shrinks the region
    (fit clearance, applied by the caller as an offset)."""
    from shapely.geometry import Polygon, box as sbox
    from shapely.ops import unary_union
    big = 1e4
    if axis == "x":
        base = sbox(c, lo - big, c + big, hi + big)
    else:
        base = sbox(lo - big, c, hi + big, c + big)
    tails = []
    for i in range(n):
        s = lo + (i + 0.5) * (hi - lo) / n
        for a in avoid:                    # keep tails 20 mm clear of crossing seams
            if abs(s - a) < 20.0:
                s = a + (20.0 if s >= a else -20.0)
        pts = [(c + 0.01, s - w_root / 2), (c - depth, s - w_tip / 2), (c - depth, s + w_tip / 2), (c + 0.01, s + w_root / 2)]
        if axis == "y":
            pts = [(q, p) for p, q in pts]
        tails.append(Polygon(pts))
    return unary_union([base] + tails)


def split_plate(outline, t, seams, tol=0.15):
    """Split a flat plate (shapely polygon outline in XY, thickness t from z = 0 to t) at seams.
    seams: list of dict(axis='x'|'y', c=value, n=tails). Returns list of shapely polygons (pieces) whose union
    (plus the fit gaps) is the outline. Each seam gets dovetail tails (keyed in-plane, slides in from above)."""
    from shapely.geometry import box as sbox
    pieces = [outline]
    for sm in seams:
        minx, miny, maxx, maxy = outline.bounds
        lo, hi = (miny, maxy) if sm["axis"] == "x" else (minx, maxx)
        avoid = [o["c"] for o in seams if o["axis"] != sm["axis"]] + list(sm.get("avoid", ()))
        plus = dovetail_seam_cs(sm["axis"], sm["c"], lo + 15, hi - 15, sm.get("n", 2), avoid=avoid)
        new = []
        for pc in pieces:
            a = pc.intersection(plus).buffer(-tol / 2, join_style=2)
            b = pc.difference(plus).buffer(-tol / 2, join_style=2)
            for g in (a, b):
                if g.is_empty:
                    continue
                if g.geom_type == "MultiPolygon":
                    new += [q for q in g.geoms if q.area > 50.0]
                elif g.area > 50.0:
                    new.append(g)
        pieces = new
    return pieces


# ------------------------------------------------------------------------------------------------ print transforms
def print_flat(solid, M_rot=None):
    """print_tf that rotates by M_rot (default identity) then drops the part onto the bed at z = 0 and centres it."""
    M = np.eye(4) if M_rot is None else np.asarray(M_rot, float)
    s = prim.place(solid, M)
    (x0, y0, z0), (x1, y1, z1) = prim.bbox(s)
    return tf.T(-(x0 + x1) / 2 + 128.0, -(y0 + y1) / 2 + 128.0, -z0) @ M


def bbox_size(solid, M=None):
    s = solid if M is None else prim.place(solid, M)
    lo, hi = prim.bbox(s)
    return tuple(h - l for l, h in zip(lo, hi))


def stl_ok(solid):
    """True if the mesh stays watertight after merging coincident vertices (what a slicer sees)."""
    tm = prim.to_trimesh(solid)
    tm.merge_vertices()
    return bool(tm.is_watertight)


def carve_healed(solid, cut, keep_largest=True):
    """solid - cut, keeping the largest piece; if the result is not watertight as an STL (two faces meeting along an
    edge where a cut feature lands exactly on a solid feature), nudge the solid by a few hundredths before the
    subtraction and shift the result back (cheap: one boolean per try)."""
    def run(sol):
        r = sol - cut
        if keep_largest:
            comps = sorted(r.decompose(), key=lambda m: -m.volume())
            r = comps[0] if comps else r
        return r
    out = run(solid)
    if stl_ok(out):
        return out
    for e in ((0.013, 0.007, 0.011), (-0.021, 0.017, -0.009), (0.05, -0.03, 0.04), (-0.11, 0.07, 0.09)):
        o2 = run(solid.translate(e)).translate(tuple(-v for v in e))
        if stl_ok(o2):
            return o2
    return out
