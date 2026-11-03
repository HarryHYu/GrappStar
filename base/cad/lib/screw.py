"""Trapezoidal (ISO 2904 style, 30 deg) lead screws and nuts, plus the shared helical thread builder.

Units mm, degrees. Frame: screw / nut axis +Z, from z = 0 to z = length, centred on the origin. Threads are right
hand. A thread is built as its cross-section perpendicular to the axis (thread_cs) extruded with twist
360 * length / lead, which is exact for a helicoid and always watertight. The thread phase is fixed to the frame
(lobe of start 0 on +X at z = 0), so a nut placed at z = k * pitch (no rotation) mates with the screw; at any other
offset t rotate it by 360 * t / lead about Z.
"""
from __future__ import annotations

import math

import numpy as np

from . import prim

# shared constants (new here)
MU_PLA = 0.25        # sliding friction PLA on PLA (dry, conservative) used for self-locking checks
SCREW_CLEAR = 0.30   # mm, total normal gap between screw and nut flanks (each part takes half)
TR_FLANK = 15.0      # deg, half angle of the trapezoidal thread (30 deg included)


def iso_crest_clearance(pitch):
    """ISO 2904 crest clearance ac (mm, radial) for a pitch."""
    if pitch <= 1.5:
        return 0.15
    if pitch <= 5:
        return 0.25
    if pitch <= 12:
        return 0.5
    return 1.0


def thread_cs(r_root, r_crest, width_fn, lead, starts=1, n_flank=16, root_step_deg=3.0, crest_step_deg=3.0):
    """Cross-section (CrossSection) perpendicular to the axis of a helical thread at z = 0.
    width_fn(r) is the axial width (mm) of the thread tooth at radius r (decreasing with r), lead the advance per
    turn (mm), starts the number of starts (lobe j centred at 360 j / starts deg). Right hand: the section turns
    by +360 / lead deg per mm of z."""
    pitch = lead / starts
    if width_fn(r_root) >= 0.995 * pitch:
        lo, hi = r_root, r_crest
        for _ in range(60):
            mid = (lo + hi) / 2
            if width_fn(mid) >= 0.995 * pitch:
                lo = mid
            else:
                hi = mid
        r_root = hi
    if width_fn(r_crest) <= 0:
        raise ValueError("thread width is not positive at the crest")

    def th(r):
        return math.pi * width_fn(r) / lead          # half angle (rad) of the lobe at radius r

    rs = np.linspace(r_root, r_crest, n_flank)
    pts = []
    for j in range(starts):
        c = 2 * math.pi * j / starts
        for r in rs:                                   # rising flank
            t = c - th(r)
            pts.append((r * math.cos(t), r * math.sin(t)))
        tc = th(r_crest)
        nc = max(1, int(math.degrees(2 * tc) / crest_step_deg))
        for t in np.linspace(c - tc, c + tc, nc + 1)[1:-1]:
            pts.append((r_crest * math.cos(t), r_crest * math.sin(t)))
        for r in rs[::-1]:                             # falling flank
            t = c + th(r)
            pts.append((r * math.cos(t), r * math.sin(t)))
        t0, t1 = c + th(r_root), c + 2 * math.pi / starts - th(r_root)
        nr = max(1, int(math.degrees(t1 - t0) / root_step_deg))
        for t in np.linspace(t0, t1, nr + 1)[1:-1]:
            pts.append((r_root * math.cos(t), r_root * math.sin(t)))
    return prim.poly_cs(pts)


def helix_extrude(cs, length, lead, z0=0.0, step_deg=4.0, hand=1):
    """Extrude a thread cross-section (given at z = 0) from z0 to z0 + length with the matching helical twist.
    The phase stays tied to z = 0 (the section is pre-rotated for z0 != 0). hand = +1 right, -1 left."""
    k = hand * 360.0 / lead
    tw = k * length
    if z0:
        cs = cs.rotate(k * z0)
    return prim.extrude(cs, length, z=z0, twist_deg=tw, n_div=max(4, int(abs(tw) / step_deg)))


def _tr_width(d, pitch, delta):
    """Axial tooth width (mm) at radius r of a basic Tr profile offset normally by delta (outwards > 0)."""
    r2 = (d - pitch / 2) / 2
    ta = math.tan(math.radians(TR_FLANK))
    off = 2 * delta / math.cos(math.radians(TR_FLANK))
    return lambda r: pitch / 2 - 2 * (r - r2) * ta + off


def screw_info(d, pitch, starts=1, mu=MU_PLA):
    """Tr d x pitch numbers: d2 pitch diameter, d3 minor, lead, lead angle (deg), friction angle (deg,
    atan(mu / cos 15)), self_locking (lead angle < friction angle), efficiency (raising), torque_per_n (Nm per N of
    axial load, raising)."""
    ac = iso_crest_clearance(pitch)
    d2 = d - pitch / 2
    lead = pitch * starts
    lam = math.atan(lead / (math.pi * d2))
    phi = math.atan(mu / math.cos(math.radians(TR_FLANK)))
    eff = math.tan(lam) / math.tan(lam + phi)
    return {"d": d, "pitch": pitch, "starts": starts, "lead": lead, "d2": d2, "d3": d - pitch - 2 * ac,
            "D1": d - pitch, "D4": d + 2 * ac, "ac": ac, "lead_angle": math.degrees(lam),
            "friction_angle": math.degrees(phi), "self_locking": bool(lam < phi), "efficiency": eff,
            "torque_per_n": d2 / 2 * math.tan(lam + phi) / 1000.0}


def screw(d, pitch, length, starts=1, clearance=SCREW_CLEAR, chamfer=None, bore=None, bore_kind="round",
          bore_opts=None):
    """Trapezoidal lead screw Tr d x (starts * pitch) P pitch, right hand. Frame: axis +Z, z = 0..length.
    Flanks and crest are offset inwards by clearance / 2 from the basic profile; root at d3 - clearance / 2
    (ISO crest clearance ac). chamfer (mm, default half the thread depth) bevels the crest at both ends.
    Optional bore (hollow screw, see gears.bore_cs)."""
    from .gears import cut_bore, tip_chamfer
    inf = screw_info(d, pitch, starts)
    dl = clearance / 2
    rc = d / 2 - dl
    rr = inf["d3"] / 2 - dl
    cs = thread_cs(rr, rc, _tr_width(d, pitch, -dl), inf["lead"], starts)
    body = helix_extrude(cs, length, inf["lead"])
    c = pitch / 4 if chamfer is None else chamfer
    body = tip_chamfer(body, rc, length, c)
    if bore:
        body = cut_bore(body, bore, bore_kind, -1, length + 1, **(bore_opts or {}))
    return body


def nut_hole(d, pitch, length, starts=1, clearance=SCREW_CLEAR, z0=None, z1=None):
    """Solid of the threaded hole of a nut (subtract it from your own body). Same frame and phase as screw().
    Extends from z0 (default -lead) to z1 (default length + lead)."""
    inf = screw_info(d, pitch, starts)
    dl = clearance / 2
    lead = inf["lead"]
    z0 = -lead if z0 is None else z0
    z1 = length + lead if z1 is None else z1
    rr = inf["D1"] / 2 + dl
    rc = inf["D4"] / 2 + dl
    cs = thread_cs(rr, rc, _tr_width(d, pitch, dl), lead, starts)
    return helix_extrude(cs, z1 - z0, lead, z0=z0)


def nut(d, pitch, length, od=None, hex_af=None, starts=1, clearance=SCREW_CLEAR, chamfer=True):
    """Trapezoidal nut matching screw(d, pitch, ...). Frame: axis +Z, z = 0..length. Outer shape: hex prism with
    hex_af across flats (flats normal to X), else a cylinder of diameter od (default 2 d). chamfer adds a 45 deg
    lead-in countersink at both ends."""
    if hex_af:
        rc = hex_af / 2 / math.cos(math.pi / 6)
        body = prim.extrude(prim.poly_cs([(rc * math.cos(math.radians(30 + 60 * i)),
                                           rc * math.sin(math.radians(30 + 60 * i))) for i in range(6)]), length)
        r_out = hex_af / 2
    else:
        od = 2 * d if od is None else od
        body = prim.cyl(od / 2, length)
        r_out = od / 2
    inf = screw_info(d, pitch, starts)
    if r_out < inf["D4"] / 2 + 1.0:
        raise ValueError("nut outer size too small for the thread")
    body = body - nut_hole(d, pitch, length, starts, clearance)
    if chamfer:
        r0 = inf["D4"] / 2 + clearance / 2
        c = pitch / 2
        cone = prim.revolve_pts([(0, -1), (r0 + 1, -1), (r0 - c, c), (0, c)])
        body = body - cone - cone.mirror((0, 0, 1)).translate((0, 0, length))
    return body
