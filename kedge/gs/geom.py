"""Geometry helpers for the ground station: manifold3d solids, involute gears, cam envelopes."""
import math
import numpy as np
import manifold3d as m3
from manifold3d import Manifold, CrossSection
from shapely.geometry import Polygon, Point, MultiPolygon
from shapely.ops import unary_union

SEG = 96  # circle segments for printed parts


def cyl(r, h, x=0, y=0, z=0, seg=SEG, r2=None):
    """Cylinder along +Z from z to z+h."""
    c = Manifold.cylinder(h, r, r if r2 is None else r2, seg)
    return c.translate((x, y, z))


def cylY(r, y0, y1, x=0, z=0, seg=SEG):
    """Cylinder along Y from y0 to y1 centred on (x, z)."""
    h = y1 - y0
    return Manifold.cylinder(h, r, r, seg).rotate((-90, 0, 0)).translate((x, y0, z))


def cylX(r, x0, x1, y=0, z=0, seg=SEG):
    h = x1 - x0
    return Manifold.cylinder(h, r, r, seg).rotate((0, 90, 0)).translate((x0, y, z))


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = min(x0, x1), max(x0, x1); y0, y1 = min(y0, y1), max(y0, y1); z0, z1 = min(z0, z1), max(z0, z1)
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def union(lst):
    lst = [s for s in lst if s is not None]
    out = lst[0]
    for s in lst[1:]:
        out = out + s
    return out


def poly_to_cs(poly):
    """shapely Polygon/MultiPolygon -> CrossSection."""
    polys = poly.geoms if isinstance(poly, MultiPolygon) else [poly]
    rings = []
    for p in polys:
        rings.append(np.asarray(p.exterior.coords)[:-1][::1].tolist())
        for h in p.interiors:
            rings.append(np.asarray(h.coords)[:-1].tolist())
    return CrossSection(rings, m3.FillRule.EvenOdd)


def extrude_xz(poly, y0, y1):
    """Extrude a 2D polygon given in (x, z) along Y from y0 to y1."""
    cs = poly_to_cs(poly)
    s = Manifold.extrude(cs, y1 - y0)            # along +Z, profile in (X, Y)
    # map profile (X,Y)->(x,z), extrusion Z -> y: rotate +90 about X sends Y->Z, Z->-Y
    s = s.rotate((90, 0, 0))                     # now profile in (x, z), extrusion along -y
    return s.translate((0, y1, 0))


def extrude_z(poly, z0, z1):
    cs = poly_to_cs(poly)
    return Manifold.extrude(cs, z1 - z0).translate((0, 0, z0))


# ---------------------------------------------------------------- involute gears
def involute_gear_2d(m, z, pa=20.0, clearance=0.25, backlash=0.15, n_inv=10, present=None, root_fill=True):
    """2D spur gear outline (shapely Polygon) centred at the origin.
    present: list of tooth indices to keep (None = all). Missing teeth are cut to the root circle."""
    rp = m * z / 2
    rb = rp * math.cos(math.radians(pa))
    ra = rp + m
    rf = max(rp - 1.25 * m - clearance * 0 , rb * 0.0 + rp - 1.25 * m)
    # tooth half angle at pitch circle (with backlash)
    inv = lambda a: math.tan(a) - a
    a_p = math.radians(pa)
    half_pitch_ang = math.pi / (2 * z) - (backlash / 2) / rp
    pts = []
    def inv_pt(r):
        if r < rb:
            return None
        a = math.acos(rb / r)
        return inv(a)
    th_p = inv(a_p)
    for k in range(z):
        c = 2 * math.pi * k / z
        keep = present is None or k in present
        if keep:
            # flank 1 (left), from root to tip
            rs = np.linspace(max(rb, rf), ra, n_inv)
            left = []
            right = []
            for r in rs:
                d = inv_pt(r) - th_p
                ang = half_pitch_ang - d
                left.append((r * math.cos(c - ang), r * math.sin(c - ang)))
                right.append((r * math.cos(c + ang), r * math.sin(c + ang)))
            # root points
            ang_root = half_pitch_ang - ((inv_pt(max(rb, rf)) or 0) - th_p)
            pts.append((rf * math.cos(c - ang_root - 0.02), rf * math.sin(c - ang_root - 0.02)))
            pts.extend(left)
            # tip arc
            a0 = c - (half_pitch_ang - (inv_pt(ra) - th_p))
            a1 = c + (half_pitch_ang - (inv_pt(ra) - th_p))
            for t in np.linspace(a0, a1, 4)[1:-1]:
                pts.append((ra * math.cos(t), ra * math.sin(t)))
            pts.extend(right[::-1])
            pts.append((rf * math.cos(c + ang_root + 0.02), rf * math.sin(c + ang_root + 0.02)))
        # root arc to next tooth
        c2 = 2 * math.pi * (k + 1) / z
        ar = half_pitch_ang - ((inv_pt(max(rb, rf)) or 0) - th_p) + 0.02
        a_start = c + ar
        a_end = c2 - ar
        if not keep:
            a_start = c - ar
        for t in np.linspace(a_start, a_end, 6)[1:-1] if keep else np.linspace(a_start, a_end, 8)[:-1]:
            pts.append((rf * math.cos(t), rf * math.sin(t)))
    poly = Polygon(pts).buffer(0)
    return poly


def rack_2d(m, n_teeth, pa=20.0, backlash=0.15, body=10.0, x0=0.0):
    """Rack along +x, pitch line at y=0, teeth pointing -y, body above (y from +m to body)."""
    p = math.pi * m
    tan = math.tan(math.radians(pa))
    pts = [(x0 - p / 2, body)]
    for i in range(n_teeth):
        xc = x0 + i * p
        hw_p = p / 4 - backlash / 4          # half tooth width at pitch line
        # tooth: tip at y=-m, root at y=+1.25m
        tip_hw = hw_p - m * tan
        root_hw = hw_p + 1.25 * m * tan
        pts += [(xc - p / 2, 1.25 * m), (xc - root_hw, 1.25 * m), (xc - tip_hw, -m), (xc + tip_hw, -m), (xc + root_hw, 1.25 * m)]
    pts += [(x0 + (n_teeth - 1) * p + p / 2, 1.25 * m), (x0 + (n_teeth - 1) * p + p / 2, body)]
    return Polygon(pts).buffer(0)


def circle(r, x=0, y=0, n=SEG):
    return Point(x, y).buffer(r, resolution=max(8, n // 4))


def rot2(x, y, a):
    c, s = math.cos(a), math.sin(a)
    return c * x - s * y, s * x + c * y
