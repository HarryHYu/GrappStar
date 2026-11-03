"""Dyneema line handling: eyes (Harry's v7 rule), a bell-mouthed fairlead, and a swept tube for showing or checking
the tether path.

v7 rule (originals/reel-handover.md): every eye is a plain round 4.4 hole coned out to 6.4 at both faces, no rings
with lips, no flares breaking out of posts. Line: 1 mm Dyneema.
Frames: axis +Z, host from z = 0 to z = thickness.
"""
import math

import numpy as np
from manifold3d import Manifold

from . import prim

EYE_THROAT_D = 4.4
EYE_FACE_D = 6.4
LINE_D = 1.0              # 1 mm Dyneema


def _eye_flare(thickness, d_throat, d_face, flare):
    f = (d_face - d_throat) / 2 if flare is None else flare      # 45 deg countersink by default
    return min(f, thickness / 2)


def eye_cutter(thickness, d_throat=EYE_THROAT_D, d_face=EYE_FACE_D, flare=None, ext=1.0):
    """Eye cutter: straight throat d_throat with conical flares to exactly d_face at z = 0 and z = thickness.
    flare: cone depth per face (default 45 deg: (d_face - d_throat) / 2, clamped to thickness / 2). The cones
    continue ext past each face so the cut is clean. Subtract from the host."""
    f = _eye_flare(thickness, d_throat, d_face, flare)
    rt, rf = d_throat / 2, d_face / 2
    k = (rf - rt) / f                                           # radius growth per mm of depth
    pts = [(0, -ext), (rf + k * ext, -ext), (rt, f), (rt, thickness - f), (rf + k * ext, thickness + ext),
           (0, thickness + ext)]
    return prim.revolve_pts(pts, seg=prim.segs(rf + k * ext))


def eye_info(thickness, d_throat=EYE_THROAT_D, d_face=EYE_FACE_D, flare=None):
    """Throat length, flare depth, cone half angle (deg)."""
    f = _eye_flare(thickness, d_throat, d_face, flare)
    return {"throat_d": d_throat, "face_d": d_face, "flare": f, "throat_len": thickness - 2 * f,
            "half_angle_deg": math.degrees(math.atan2((d_face - d_throat) / 2, f))}


def eye_boss(thickness, d_throat=EYE_THROAT_D, d_face=EYE_FACE_D, wall=2.0, flange=0.0, flange_t=1.2, flare=None):
    """Printed eye boss / press insert: cylinder OD d_face + 2 wall, z 0..thickness, with the v7 eye through it.
    flange > 0 adds a collar (radial width flange, height flange_t) at z = 0 so it can be pushed into a host hole
    of the boss OD from one side. Print flange down."""
    ro = d_face / 2 + wall
    b = prim.cyl(ro, thickness)
    if flange > 0:
        b = b + prim.cyl(ro + flange, flange_t)
    return b - eye_cutter(thickness, d_throat, d_face, flare)


def fairlead_cutter(thickness, d_throat=EYE_THROAT_D, flare_r=3.0, ext=1.0):
    """Bell-mouth fairlead cutter: throat d_throat, both faces flared by a quarter round of radius flare_r (smooth,
    no edge for the line to saw on). Face d = d_throat + 2 flare_r (clamped to thickness / 2). Subtract from host."""
    fr = min(flare_r, thickness / 2 - 0.05)
    rt = d_throat / 2
    n = 12
    lo = [(rt + fr - fr * math.sin(math.pi / 2 * i / n), fr - fr * math.cos(math.pi / 2 * i / n)) for i in range(n + 1)]
    # lo runs from (rt + fr, 0) at the face to (rt, fr) inside
    hi = [(r, thickness - z) for (r, z) in reversed(lo)]
    pts = [(0, -ext), (rt + fr + ext, -ext), (rt + fr + ext, 0)] + lo[1:] + hi[:-1] + \
        [(rt + fr + ext, thickness), (rt + fr + ext, thickness + ext), (0, thickness + ext)]
    return prim.revolve_pts(pts, seg=prim.segs(rt + fr + ext))


def fairlead(thickness=6.0, d_throat=EYE_THROAT_D, flare_r=3.0, size=(26.0, 14.0), m3_spacing=18.0):
    """Printed fairlead block: plate size[0] (X) x size[1] (Y) x thickness, bell-mouth eye on the axis, two M3
    clearance holes at x = +-m3_spacing / 2. Frame: eye axis +Z through the origin, z 0..thickness."""
    from . import fits
    b = prim.rbox(size[0], size[1], thickness, 3.0)
    cut = [fairlead_cutter(thickness, d_throat, flare_r)]
    for s in (-1, 1):
        cut.append(prim.cyl(fits.M3_CLEAR_D / 2, thickness + 2, x=s * m3_spacing / 2, z=-1, seg=24))
    return prim.diff(b, cut)


def line_tube(points, d=LINE_D, seg=12):
    """Thin swept tube along a polyline (N x 3 points, mm): union of capsules (hull of two spheres) per segment.
    For renders and clearance checks of the tether; not a printed part."""
    p = np.asarray(points, float)
    if len(p) < 2:
        raise ValueError("need at least two points")
    r = d / 2
    sph = Manifold.sphere(r, seg)
    caps = []
    for a, b in zip(p[:-1], p[1:]):
        if np.linalg.norm(b - a) < 1e-9:
            continue
        caps.append(Manifold.batch_hull([sph.translate(tuple(a)), sph.translate(tuple(b))]))
    return prim.union(caps)


def line_length(points):
    """Polyline length (mm)."""
    p = np.asarray(points, float)
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def line_hits(points, solids, d=LINE_D):
    """Overlap volume (mm3) of the line tube with each solid (dict name -> Manifold or list). 0 = clear."""
    tube = line_tube(points, d)
    items = solids.items() if isinstance(solids, dict) else enumerate(solids)
    return {k: (tube ^ s).volume() for k, s in items}
