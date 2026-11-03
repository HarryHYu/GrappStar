"""Pedestal supports for shafts that are not along Y (frame subsystem, private helper).

Horizontal axis (not Y): an upright pillow plate per bearing, normal to the axis, standing on the deck with a foot
(2 x M3 down into deck inserts each side). Vertical axis: a C bracket (back plate standing on the deck beside the
shaft, one arm per bearing) with the seats in the arms. Bearings sit at the axial offsets sh["supports"] (mm from
the origin along the axis), default +-span/2.
"""
from __future__ import annotations

import math

import numpy as np

from cad.lib import Part, prim, tf, fits
from . import _frame_util as U


def _offsets(sh):
    if sh.get("supports"):
        return [float(s) for s in sh["supports"]]
    sp = float(sh.get("span", 40.0))
    return [-sp / 2, sp / 2]


def _seat(P, sh):
    if sh["brg"] == "bush":
        return sh["d"] + 8.0, 10.0, None
    od, w, bore = U.brg_dims(P, sh["brg"])
    return od, w, bore


def pedestals(P, n, sh, fw):
    """Returns (parts, deck_holes, warnings) for one shaft entry."""
    a = np.asarray(sh["axis"], float); a /= np.linalg.norm(a)
    o = np.asarray(sh["origin"], float)
    od, w, bore = _seat(P, sh)
    parts, holes, warns = [], [], []
    vertical = abs(a[2]) > 0.95
    if abs(a[2]) > 0.05 and not vertical:
        return [], [], [f"shaft {n}: inclined axis {tuple(a)} not supported by a pedestal"]
    for k, s in enumerate(_offsets(sh)):
        c = o + a * s
        if not vertical:
            t = w + 3.0
            half = od / 2 + 8.0
            ztop = c[2] + half
            if c[2] - od / 2 < 6.0:
                warns.append(f"shaft {n}: bearing at z {c[2]:.1f} too low for a pedestal")
                continue
            # local: z = up, x across (in the plate), y along the shaft axis (plate thickness), bearing centre at
            # (0, 0, c_z); plate from y -t/2..t/2
            plate = prim.box(-half, half, -t / 2, t / 2, 0, c[2]) + prim.cylY(half, -t / 2, t / 2, 0.0, c[2])
            foot = prim.box(-half - 14, half + 14, -t / 2 - 8, t / 2 + 8, 0, 6)
            sol = plate + foot
            cuts = []
            # seat along local Y
            seat = U.bearing_seat_cutter(od, w, -t / 2 - 0.0, lip_id=U.lip_for(od, sh["d"])) if sh["brg"] != "bush" \
                else prim.cyl((sh["d"] + fits.SHAFT_IN_BORE) / 2, 200, z=-100)
            cuts.append(prim.place(seat, tf.T(0, 0, c[2]) @ tf.Rx(-90)))
            fh = [(-(half + 7), 0.0), (half + 7, 0.0)]
            for (fx, fy) in fh:
                cuts.append(U.clear_hole(fx, fy, -1, 7, head_top=6.01))
            sol = prim.diff(sol, cuts)
            yaw = math.degrees(math.atan2(a[1], a[0])) - 90.0      # local y -> axis direction
            M = tf.T(c[0], c[1], 0.0) @ tf.Rz(yaw)
            nm = f"frame_ped_{n}_{k + 1}"
            p = Part(nm, prim.place(sol, M), "PLA", color="#b8c2cc",
                     notes=f"pedestal for the {n} shaft ({sh['brg']}), 2 x M3 into deck inserts")
            p.print_tf = U.print_flat(p.solid, tf.Rz(-yaw) @ tf.Rx(90))   # plate face down
            parts.append(p)
            holes += [tuple(tf.apply(M, [(fx, fy, 0)])[0][:2]) for fx, fy in fh]
    if vertical and _offsets(sh):
        offs = _offsets(sh)
        zs = sorted(float(o[2] + a[2] * s) for s in offs)
        t = w + 3.0
        half = od / 2 + 7.0
        back_gap = od / 2 + 10.0           # back plate this far from the axis (towards -x local)
        zmax = zs[-1] + t / 2
        sol = prim.box(-back_gap - 8, -back_gap, -half, half, 0, zmax)
        sol = sol + prim.box(-back_gap - 24, -back_gap + 12, -half - 10, half + 10, 0, 6)
        cuts = []
        for z in zs:
            arm = prim.box(-back_gap - 0.01, half, -half, half, z - t / 2, z + t / 2) + prim.cyl(half, t, 0, 0, z - t / 2) \
                if z - t / 2 > 0 else None
            if arm is None:
                warns.append(f"shaft {n}: vertical bearing at z {z:.1f} below the deck")
                continue
            sol = sol + arm
            seat = U.bearing_seat_cutter(od, w, z - t / 2, lip_id=U.lip_for(od, sh["d"])) if sh["brg"] != "bush" \
                else prim.cyl((sh["d"] + fits.SHAFT_IN_BORE) / 2, 200, z=z - 100)
            cuts.append(seat)
        fh = [(-back_gap - 16, -half - 4), (-back_gap - 16, half + 4)]
        for fx, fy in fh:
            cuts.append(U.clear_hole(fx, fy, -1, 7, head_top=6.01))
        sol = prim.diff(sol, cuts)
        yaw = float(sh.get("ped_yaw", 0.0))
        M = tf.T(o[0], o[1], 0.0) @ tf.Rz(yaw)
        nm = f"frame_ped_{n}"
        p = Part(nm, prim.place(sol, M), "PLA", color="#b8c2cc",
                 notes=f"C bracket for the vertical {n} shaft ({sh['brg']}), 2 x M3 into deck inserts")
        p.print_tf = U.print_flat(p.solid, tf.Ry(90) @ tf.Rz(-yaw))     # back plate face down
        parts.append(p)
        holes += [tuple(tf.apply(M, [(fx, fy, 0)])[0][:2]) for fx, fy in fh]
    return parts, holes, warns
