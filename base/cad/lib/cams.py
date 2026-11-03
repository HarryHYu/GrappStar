"""Barrel (drum) cams for a selector / shift drum: a groove in a cylinder followed by a radial cylindrical pin.

Units mm, degrees. Drum frame: axis +Z, z = 0..length, drum angle measured from +X towards +Y. A track gives the
groove centre height z (mm, in the drum frame) as a function of drum angle deg. The follower pin axis is radial;
the groove is the swept volume of the pin (union of convex hulls of consecutive radial pin positions), so its walls
are right for a cylindrical pin of diameter track_width at any slope.
"""
from __future__ import annotations

import math

import numpy as np
from manifold3d import Manifold

from . import prim, tf


def track_fn(waypoints, closed=True):
    """Track z(deg) from (drum_deg, z) waypoints, blended with cycloidal (zero velocity and acceleration at each
    waypoint) moves between them. Equal z on consecutive waypoints gives a dwell. closed: the track repeats every
    360 deg (the last waypoint blends back to the first + 360). Open tracks hold the end values outside the range."""
    wp = sorted((float(d), float(z)) for d, z in waypoints)
    if closed:
        d0 = wp[0][0]
        if wp[-1][0] - d0 >= 360.0 - 1e-9:
            wp = [w for w in wp if w[0] - d0 < 360.0 - 1e-9]
        wp = wp + [(wp[0][0] + 360.0, wp[0][1])]
    ds = np.array([w[0] for w in wp])
    zs = np.array([w[1] for w in wp])

    def f(deg):
        d = np.asarray(deg, float)
        if closed:
            d = ds[0] + np.mod(d - ds[0], 360.0)
        d = np.clip(d, ds[0], ds[-1])
        i = np.clip(np.searchsorted(ds, d, side="right") - 1, 0, len(ds) - 2)
        span = ds[i + 1] - ds[i]
        u = np.where(span > 0, (d - ds[i]) / np.where(span > 0, span, 1), 0.0)
        s = u - np.sin(2 * math.pi * u) / (2 * math.pi)
        out = zs[i] + (zs[i + 1] - zs[i]) * s
        return float(out) if np.ndim(out) == 0 else out

    f.closed = closed
    f.span = (ds[0], ds[-1])
    return f


def track_z(track, deg):
    """z (mm) of the groove centre at drum angle deg. track is a callable or a (deg, z) waypoint list (closed)."""
    f = track if callable(track) else track_fn(track)
    return f(deg)


def barrel_info(r_out, track, track_depth, n=2000, deg_range=None):
    """Track numbers at the pitch radius r_out - track_depth / 2: max pressure angle (deg, atan(dz / ds)), the drum
    angle where it occurs, z range, and the minimum radius of curvature of the track centre line (mm)."""
    f = track if callable(track) else track_fn(track)
    lo, hi = deg_range or (getattr(f, "span", (0.0, 360.0)) if not getattr(f, "closed", True) else (0.0, 360.0))
    d = np.linspace(lo, hi, n + 1)
    z = np.asarray(f(d), float)
    rp = r_out - track_depth / 2
    s = rp * np.radians(d)
    dz = np.gradient(z, s)
    ang = np.degrees(np.arctan(np.abs(dz)))
    k = int(np.argmax(ang))
    d2 = np.gradient(dz, s)
    curv = np.abs(d2) / (1 + dz ** 2) ** 1.5
    return {"r_pitch": rp, "max_pressure_angle": float(ang[k]), "at_deg": float(d[k]), "z_min": float(z.min()),
            "z_max": float(z.max()), "min_curvature_radius": float(1 / max(curv.max(), 1e-12))}


def _pin(r_in, r_top, w, seg):
    """Radial pin (cylinder of diameter w along +X from r_in to r_top), as a convex Manifold."""
    return prim.cylX(w / 2, r_in, r_top, seg=seg)


def groove_cutter(r_out, track, track_width, track_depth, n_samples=180, deg_range=None, seg=24):
    """Solid swept by a radial pin of diameter track_width whose tip sits at r_out - track_depth, following the
    track (drum frame). Open tracks get round ends."""
    f = track if callable(track) else track_fn(track)
    closed = getattr(f, "closed", True)
    lo, hi = deg_range or ((0.0, 360.0) if closed else f.span)
    d = np.linspace(lo, hi, n_samples + 1)
    if closed and deg_range is None:
        d = d[:-1]
    z = np.asarray(f(d), float)
    pin = _pin(r_out - track_depth, r_out + 1.0, track_width, seg)
    pos = [pin.rotate((0, 0, float(a))).translate((0, 0, float(zz))) for a, zz in zip(d, z)]
    m = len(pos)
    pairs = range(m) if closed and deg_range is None else range(m - 1)
    hulls = [Manifold.batch_hull([pos[i], pos[(i + 1) % m]]) for i in pairs]
    return prim.union(hulls)


def barrel_cam(r_out, length, track, track_width, track_depth, bore=None, n_samples=180, bore_kind="round",
               bore_opts=None, deg_range=None):
    """Barrel cam drum. Frame: axis +Z, z = 0..length, outer radius r_out, groove of width track_width (pin
    diameter + clearance) and depth track_depth following track (callable deg -> z, or (deg, z) waypoints,
    see track_fn). n_samples positions over the track (keep the angle step under ~3 deg). Optional bore."""
    from .gears import cut_bore
    if bore and bore / 2 > r_out - track_depth - 1.0:
        raise ValueError("bore leaves less than 1 mm under the groove")
    body = prim.cyl(r_out, length)
    body = body - groove_cutter(r_out, track, track_width, track_depth, n_samples, deg_range)
    if bore:
        body = cut_bore(body, bore, bore_kind, -1, length + 1, **(bore_opts or {}))
    return body


def follower_pin(r_out, track_depth, pin_d, deg, z, extra_out=5.0, tip_gap=0.2):
    """Radial follower pin solid (drum frame) at drum angle deg and height z: diameter pin_d, tip tip_gap above the
    groove bottom, reaching extra_out beyond r_out. For checks and visuals."""
    r_in = r_out - track_depth + tip_gap
    return _pin(r_in, r_out + extra_out, pin_d, 32).rotate((0, 0, deg)).translate((0, 0, z))
