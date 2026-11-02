"""Involute spur gears for the drum drive, built as manifold3d solids.

Units: mm and degrees. Solids are manifold3d.Manifold, axis +Z, gear centered
at the origin, bottom face at z=0. Tooth 0 is centered on the +X axis and
tooth k sits at k*360/z degrees.

Profile model (simplified, trochoid free):
  * involute flank from max(r_b, r_f) up to the tip circle r_a,
  * below the base circle a radial line down to the root circle,
  * optional small circular fillet between the radial line and the root arc,
  * root arc across the gap.
Backlash is split evenly per gear: each tooth is thinned by backlash/2,
measured as arc length on the pitch circle.
"""

from __future__ import annotations

import math

import manifold3d as m3d
import numpy as np

__all__ = [
    "inv",
    "inv_inverse",
    "gear_info",
    "center_distance",
    "involute_profile",
    "gear",
    "mesh_phase",
]


def inv(a_rad: float) -> float:
    """Involute function tan(a) - a (radians)."""
    return math.tan(a_rad) - a_rad


def inv_inverse(v: float) -> float:
    """Solve inv(a) = v for a (radians) by Newton iteration."""
    if v <= 0.0:
        return 0.0
    a = (3.0 * v) ** (1.0 / 3.0)  # small angle start: inv(a) ~ a^3/3
    for _ in range(60):
        f = math.tan(a) - a - v
        df = math.tan(a) ** 2
        step = f / df
        a -= step
        if abs(step) < 1e-15:
            break
    return a


def gear_info(z: int, m: float, pa: float = 20.0, x: float = 0.0,
              clearance: float = 0.25, tip_relief: float = 0.0) -> dict:
    """Basic radii of an external spur gear.

    Returns r_p (pitch), r_b (base), r_a (tip), r_f (root), pitch (circular
    pitch on the pitch circle), p_b (base pitch), pa, z, m, x.
    """
    a = math.radians(pa)
    r_p = m * z / 2.0
    return {
        "z": z,
        "m": m,
        "x": x,
        "pa": pa,
        "r_p": r_p,
        "r_b": r_p * math.cos(a),
        "r_a": m * (z / 2.0 + 1.0 + x) - tip_relief,
        "r_f": m * (z / 2.0 - 1.0 - clearance + x),
        "pitch": math.pi * m,
        "p_b": math.pi * m * math.cos(a),
    }


def center_distance(z1: int, z2: int, m: float, x1: float = 0.0,
                    x2: float = 0.0, pa: float = 20.0) -> float:
    """Zero-backlash center distance of an external pair.

    With x1 + x2 != 0 the working pressure angle a_w is solved from
    inv(a_w) = 2 tan(pa) (x1 + x2) / (z1 + z2) + inv(pa), and
    a = m (z1 + z2) / 2 * cos(pa) / cos(a_w).
    """
    a0 = m * (z1 + z2) / 2.0
    if abs(x1 + x2) < 1e-12:
        return a0
    p = math.radians(pa)
    a_w = inv_inverse(2.0 * math.tan(p) * (x1 + x2) / (z1 + z2) + inv(p))
    return a0 * math.cos(p) / math.cos(a_w)


def _arc(r: float, a0: float, a1: float, max_step: float) -> list[tuple[float, float]]:
    """Points on a circle of radius r from angle a0 to a1 (radians), a0 and a1
    included."""
    n = max(1, int(math.ceil(abs(a1 - a0) / max_step)))
    return [(r * math.cos(a0 + (a1 - a0) * i / n),
             r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]


def _half_tooth(z, m, pa, x, backlash, clearance, n_inv, root_fillet,
                tip_relief):
    """Upper flank of tooth 0 (the +angle side), from the root up to the tip,
    as a list of (r, polar_angle_rad). The flank is symmetric about +X.

    Returns (flank, info) where flank[0] is on the root circle and flank[-1]
    on the tip circle.
    """
    gi = gear_info(z, m, pa, x, clearance, tip_relief)
    p = math.radians(pa)
    r_p, r_b, r_a, r_f = gi["r_p"], gi["r_b"], gi["r_a"], gi["r_f"]
    if r_f <= 0:
        raise ValueError("root radius is not positive")
    # tooth thickness on the pitch circle (arc), thinned by backlash/2
    s_p = m * (math.pi / 2.0 + 2.0 * x * math.tan(p)) - backlash / 2.0
    psi_p = s_p / (2.0 * r_p)  # half angular thickness at the pitch circle

    def psi(r):
        a_r = math.acos(min(1.0, r_b / r))
        return psi_p + inv(p) - inv(a_r)

    # Do not let the tooth become pointed: shrink tip until a small land.
    min_land = 0.05 * m
    while psi(r_a) * r_a < min_land / 2.0 and r_a > r_b:
        r_a -= 0.005 * m
    half_pitch = math.pi / z
    if psi(max(r_b, r_f)) >= half_pitch:
        raise ValueError("teeth overlap at the root; check z, x, backlash")

    flank: list[tuple[float, float]] = []
    r_start = max(r_b, r_f)
    psi_b = psi(r_start)
    if r_f < r_b:
        # radial segment below the base circle, with optional root fillet
        rho = 0.0
        if root_fillet:
            rho = min(0.2 * m, 0.45 * (r_b - r_f))
        if rho > 1e-9:
            rc = r_f + rho
            dphi = math.asin(rho / rc)
            phi_c = psi_b + dphi
            if phi_c < half_pitch:
                # fillet circle center (rc, phi_c); tangent to root circle at
                # angle phi_c and to the radial line at radius rc*cos(dphi)
                cx, cy = rc * math.cos(phi_c), rc * math.sin(phi_c)
                # start: point on root circle (direction from center = -radial)
                a_start = phi_c + math.pi
                # end: foot on the radial line at angle psi_b; direction from
                # center to foot is perpendicular to the line, toward -angle
                a_end = psi_b - math.pi / 2.0
                # take the short arc (the corner side, span pi/2 - dphi)
                while a_end - a_start > math.pi:
                    a_end -= 2 * math.pi
                while a_end - a_start <= -math.pi:
                    a_end += 2 * math.pi
                nf = 5
                for i in range(nf + 1):
                    t = a_start + (a_end - a_start) * i / nf
                    px, py = cx + rho * math.cos(t), cy + rho * math.sin(t)
                    flank.append((math.hypot(px, py), math.atan2(py, px)))
            else:
                flank.append((r_f, psi_b))
        else:
            flank.append((r_f, psi_b))
    # involute part, uniform in roll angle t (r = r_b sqrt(1 + t^2))
    t0 = math.sqrt(max(0.0, (r_start / r_b) ** 2 - 1.0))
    t1 = math.sqrt((r_a / r_b) ** 2 - 1.0)
    for i in range(n_inv + 1):
        t = t0 + (t1 - t0) * i / n_inv
        r = r_b * math.sqrt(1.0 + t * t)
        if i == n_inv:
            r = r_a
        flank.append((r, psi(r)))
    info = dict(gi)
    info["r_a"] = r_a
    info["psi_p"] = psi_p
    return flank, info


def _profile(z, m, pa, x, backlash, clearance, n_inv, root_fillet, tip_relief,
             tooth_mask):
    flank, info = _half_tooth(z, m, pa, x, backlash, clearance, n_inv,
                              root_fillet, tip_relief)
    r_f, r_a = info["r_f"], info["r_a"]
    pitch_ang = 2.0 * math.pi / z
    step = math.radians(3.0)
    # tooth 0 outline (CCW): lower flank up, tip arc, upper flank down
    lower = [(r, -a) for (r, a) in flank]          # root -> tip, angle -a
    upper = [(r, a) for (r, a) in reversed(flank)]  # tip -> root
    tip = _arc(r_a, lower[-1][1], upper[0][1], step)[1:-1]
    tooth_pts = ([(r * math.cos(a), r * math.sin(a)) for r, a in lower] + tip +
                 [(r * math.cos(a), r * math.sin(a)) for r, a in upper])
    tooth_pts = np.asarray(tooth_pts)
    root_lo = lower[0][1]   # angle where tooth 0 leaves the root circle
    root_hi = upper[-1][1]
    pts: list[np.ndarray] = []
    for k in range(z):
        c = k * pitch_ang
        g0 = c - pitch_ang / 2.0  # gap center before tooth k
        g1 = c + pitch_ang / 2.0
        if tooth_mask[k]:
            seg_a = _arc(r_f, g0, c + root_lo, step)[:-1]
            ca, sa = math.cos(c), math.sin(c)
            rot = tooth_pts @ np.array([[ca, sa], [-sa, ca]])
            seg_b = _arc(r_f, c + root_hi, g1, step)[1:-1]
            pts.append(np.asarray(seg_a))
            pts.append(rot)
            if seg_b:
                pts.append(np.asarray(seg_b))
        else:
            pts.append(np.asarray(_arc(r_f, g0, g1, step)[:-1]))
    out = np.vstack(pts)
    return [(float(px), float(py)) for px, py in out]


def involute_profile(z: int, m: float, pa: float = 20.0, x: float = 0.0,
                     backlash: float = 0.15, clearance: float = 0.25,
                     n_inv: int = 12, root_fillet: bool = True,
                     tip_relief: float = 0.0) -> list[tuple[float, float]]:
    """Closed CCW outline of a full external spur gear.

    z: teeth, m: module, pa: pressure angle (deg), x: profile shift
    coefficient, backlash: total backlash (mm, pitch-circle arc) of the pair,
    of which this gear takes half, clearance: root clearance coefficient
    (r_f = m (z/2 - 1 - clearance + x)), n_inv: involute segments per flank,
    tip_relief: mm subtracted from r_a. Tooth 0 is centered on +X.
    """
    return _profile(z, m, pa, x, backlash, clearance, n_inv, root_fillet,
                    tip_relief, [True] * z)


def _in_sectors(ang_deg: float, keep) -> bool:
    for s, e in keep:
        span = (e - s) % 360.0
        if span == 0.0 and e != s:
            return True
        if (ang_deg - s) % 360.0 <= span + 1e-9:
            return True
    return False


def gear(z: int, m: float, width: float, pa: float = 20.0, x: float = 0.0,
         bore: float | None = None, backlash: float = 0.15, keep=None,
         clearance: float = 0.25, n_inv: int = 12, root_fillet: bool = True,
         tip_relief: float = 0.0, bore_segments: int = 64) -> m3d.Manifold:
    """Extruded spur gear, bottom at z=0, axis +Z.

    bore: through-hole diameter (mm) or None.
    keep: optional list of (start_deg, end_deg) sectors (CCW from start to
    end). Teeth whose center angle k*360/z lies outside every sector are
    removed and replaced by the root circle (mutilated / segment gear).
    """
    if keep is None:
        mask = [True] * z
    else:
        mask = [_in_sectors(k * 360.0 / z, keep) for k in range(z)]
    pts = _profile(z, m, pa, x, backlash, clearance, n_inv, root_fillet,
                   tip_relief, mask)
    cs = m3d.CrossSection([np.asarray(pts)], m3d.FillRule.Positive)
    solid = m3d.Manifold.extrude(cs, width)
    if bore:
        hole = m3d.Manifold.cylinder(width + 2.0, bore / 2.0, bore / 2.0,
                                     bore_segments).translate((0, 0, -1.0))
        solid = solid - hole
    return solid


def mesh_phase(z1: int, z2: int, theta1_deg: float, phase1: float = 0.0) -> float:
    """Rotation (deg) of gear 2 that meshes with gear 1 at rotation theta1.

    Gear 1 is at the origin, gear 2 at the center distance in direction
    phase1 (deg, default 0 = on +X). Both gears follow the tooth-0-on-+X
    convention before rotation. External mesh: opposite rotation, ratio
    z1/z2.

    When gear 1 tooth 0 points at gear 2 (theta1 = phase1) gear 2 must show a
    gap center toward gear 1, i.e. at angle phase1 + 180. Gear 2 gaps lie at
    theta2 + (k + 1/2) 360/z2, so theta2 = phase1 + 180 - 180/z2 (mod 360/z2,
    the same as phase1 + 180 + 180/z2). Rolling adds -(theta1 - phase1) z1/z2.
    """
    offset = phase1 + 180.0 + 180.0 / z2
    # Not wrapped modulo p2 so the result is continuous in theta1.
    return offset - (theta1_deg - phase1) * z1 / z2
