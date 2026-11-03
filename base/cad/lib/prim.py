"""Primitive solids and 2D helpers on manifold3d. All lengths mm. Axis-aligned helpers build along +Z unless named."""
import math

import numpy as np
import manifold3d as m3
from manifold3d import Manifold, CrossSection

from . import tf

SEG = 96          # default circle segments for printed parts
SEG_SMALL = 32    # holes under 6 mm


def segs(r, seg=None):
    """Circle segment count: fine for big circles, cheaper for small ones (min 24)."""
    if seg:
        return seg
    return int(max(24, min(SEG * 2, round(2 * math.pi * r / 0.6))))


def cyl(r, h, x=0.0, y=0.0, z=0.0, r2=None, seg=None, center=False):
    """Cylinder along +Z from z to z + h (or centred on z if center). Negative h builds downward from z."""
    if h < 0:
        z, h = z + h, -h
    c = Manifold.cylinder(h, r, r if r2 is None else r2, segs(max(r, r2 or 0), seg), center)
    return c.translate((x, y, z))


def cylX(r, x0, x1, y=0.0, z=0.0, seg=None):
    """Cylinder along X from x0 to x1 centred on (y, z) (order of x0, x1 free)."""
    x0, x1 = sorted((x0, x1))
    return Manifold.cylinder(x1 - x0, r, r, segs(r, seg)).rotate((0, 90, 0)).translate((x0, y, z))


def cylY(r, y0, y1, x=0.0, z=0.0, seg=None):
    """Cylinder along Y from y0 to y1 centred on (x, z) (order of y0, y1 free)."""
    y0, y1 = sorted((y0, y1))
    return Manifold.cylinder(y1 - y0, r, r, segs(r, seg)).rotate((-90, 0, 0)).translate((x, y0, z))


def tube(ro, ri, h, z=0.0, seg=None):
    """Hollow cylinder along +Z."""
    return cyl(ro, h, z=z, seg=seg) - cyl(ri, h + 2, z=z - 1, seg=seg)


def box(x0, x1, y0, y1, z0, z1):
    """Axis-aligned box from corner ranges (order of each pair free)."""
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def cbox(sx, sy, sz, x=0.0, y=0.0, z=0.0):
    """Box centred in x and y, from z to z + sz."""
    return box(x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2, z, z + sz)


def rbox(sx, sy, sz, r, z=0.0):
    """Box centred in x, y with vertical edges rounded to radius r, from z to z + sz."""
    cs = rrect_cs(sx, sy, r)
    return Manifold.extrude(cs, sz).translate((0, 0, z))


def rrect_cs(sx, sy, r, seg=None):
    r = min(r, sx / 2 - 1e-3, sy / 2 - 1e-3)
    if r <= 0:
        return CrossSection.square((sx, sy), True)
    return CrossSection.square((sx - 2 * r, sy - 2 * r), True).offset(r, m3.JoinType.Round, 2.0, segs(r, seg))


def circle_cs(r, x=0.0, y=0.0, seg=None):
    return CrossSection.circle(r, segs(r, seg)).translate((x, y))


def poly_cs(pts):
    """CrossSection from one ring of (x, y) points (orientation fixed automatically)."""
    return CrossSection([np.asarray(pts, float).tolist()], m3.FillRule.Positive) if _ccw(pts) else \
        CrossSection([np.asarray(pts, float)[::-1].tolist()], m3.FillRule.Positive)


def _ccw(pts):
    p = np.asarray(pts, float)
    x, y = p[:, 0], p[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y) > 0


def shapely_cs(poly):
    """shapely Polygon / MultiPolygon -> CrossSection."""
    from shapely.geometry import MultiPolygon
    polys = poly.geoms if isinstance(poly, MultiPolygon) else [poly]
    rings = []
    for p in polys:
        rings.append(np.asarray(p.exterior.coords)[:-1].tolist())
        for h in p.interiors:
            rings.append(np.asarray(h.coords)[:-1].tolist())
    return CrossSection(rings, m3.FillRule.EvenOdd)


def extrude(cs, h, z=0.0, twist_deg=0.0, n_div=0, scale_top=None):
    """Extrude a CrossSection along +Z from z to z + h."""
    kw = {}
    if twist_deg:
        kw["twist_degrees"] = twist_deg
        kw["n_divisions"] = n_div or max(8, int(abs(twist_deg) / 3))
    if scale_top is not None:
        kw["scale_top"] = scale_top if isinstance(scale_top, (tuple, list)) else (scale_top, scale_top)
    return Manifold.extrude(cs, h, **kw).translate((0, 0, z))


def revolve(cs_rz, seg=None, deg=360.0):
    """Revolve a profile given in (r, z) (x = r >= 0, y = z) about +Z."""
    return Manifold.revolve(cs_rz, seg or SEG, deg)


def revolve_pts(pts_rz, seg=None, deg=360.0):
    """Revolve a closed (r, z) polygon about +Z."""
    return revolve(poly_cs(pts_rz), seg, deg)


def union(lst):
    lst = [s for s in lst if s is not None]
    if not lst:
        return Manifold()
    return Manifold.batch_boolean(lst, m3.OpType.Add) if len(lst) > 1 else lst[0]


def diff(a, cutters):
    cutters = [c for c in cutters if c is not None]
    if not cutters:
        return a
    return a - union(cutters)


def place(solid, M):
    """Apply a 4x4 transform to a Manifold."""
    return solid.transform(tf.to_m3(M))


def polar(solid, n, deg=360.0, axis="z"):
    """n copies rotated evenly about an axis through the origin (union)."""
    step = deg / n if deg >= 360 else deg / max(1, n - 1)
    rot = {"z": lambda a: (0, 0, a), "x": lambda a: (a, 0, 0), "y": lambda a: (0, a, 0)}[axis]
    return union([solid.rotate(rot(i * step)) for i in range(n)])


def chamfer_ring(r_out, h, c, z=0.0, top=True):
    """Ring-shaped cutter that chamfers the outer edge of a cylinder of radius r_out at z (top or bottom face)."""
    if top:
        pts = [(r_out - c - 1, z + h + 1), (r_out + 2, z + h + 1), (r_out + 2, z + h - c - 2)]
    else:
        pts = [(r_out - c - 1, z - 1), (r_out + 2, z - 1), (r_out + 2, z + c + 2)]
    return revolve_pts(pts)


def clean(solid, tol=1e-4):
    """Merge vertices closer than tol (mm) so the mesh stays watertight after an STL loader merges coincident
    vertices (booleans can leave near-duplicate vertices 1e-6 apart). Changes the volume by far less than 0.001 mm3."""
    return solid.simplify(tol)


def mesh_of(solid):
    """Manifold -> (vertices (N, 3) float64, faces (M, 3) int64)."""
    m = solid.to_mesh()
    return np.asarray(m.vert_properties[:, :3], np.float64), np.asarray(m.tri_verts, np.int64)


def from_mesh(v, f):
    """(vertices, faces) -> Manifold (raises if not manifold)."""
    mm = m3.Mesh(vert_properties=np.asarray(v, np.float32), tri_verts=np.asarray(f, np.uint32))
    s = Manifold(mm)
    if s.status() != m3.Error.NoError:
        raise ValueError(f"mesh is not a valid manifold: {s.status()}")
    return s


def from_trimesh(tm):
    tm = tm.copy()
    tm.merge_vertices()
    return from_mesh(tm.vertices, tm.faces)


def to_trimesh(solid):
    import trimesh
    v, f = mesh_of(solid)
    return trimesh.Trimesh(v, f, process=False)


def bbox(solid):
    """((xmin, ymin, zmin), (xmax, ymax, zmax))."""
    b = solid.bounding_box()
    return tuple(b[:3]), tuple(b[3:])


def shells(solid):
    """Number of disconnected bodies."""
    return len(solid.decompose())
