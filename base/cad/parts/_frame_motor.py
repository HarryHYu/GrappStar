"""Wiper motor model and its separately printed adapter plate + saddle (frame subsystem, private helper).

The motor is NOT measured yet. Every motor number lives in WIPER_DEFAULT below and is overridden by any non-None key
of params.WIPER, so Harry edits only params.py and reprints frame_motor_adapter (and frame_motor_saddle).

Motor mount frame ("M frame"): origin on the output shaft axis in the plane of the mount boss faces, +Z along the
output shaft (out of the gearbox), +X along the gearbox long axis towards the motor can, +Y = Z x X.
The bosses touch the adapter plate at z = 0; the gearbox is behind (z < 0).
"""
from __future__ import annotations

import math

import numpy as np
from shapely.geometry import Point, MultiPoint
from shapely.ops import unary_union

from cad.lib import prim, tf, fits
from . import _frame_util as U

# Typical 12 V car wiper motor (assumed until Harry measures; see frame_NOTES.md, "what to measure").
WIPER_DEFAULT = dict(
    shaft_d=10.0,            # output shaft diameter
    shaft_flat=8.0,          # across the two flats
    shaft_len=20.0,          # from the boss face plane to the end of the flats (start of the thread)
    thread_d=8.0,            # M8 end thread
    thread_len=12.0,
    hub_d=24.0,              # gearbox output bushing boss round the shaft
    hub_h=4.0,               # how far that boss stands proud of the boss face plane (+z)
    mount_holes=[(-24.0, 38.0, 6.5), (-24.0, -38.0, 6.5), (72.0, 0.0, 6.5)],   # (x, y, dia) in the M frame
    boss_d=16.0,             # mount boss diameter
    boss_h=8.0,              # boss height above the gearbox face (gearbox face at z = -boss_h)
    body_box=(-38.0, 84.0, -40.0, 40.0, 56.0),   # gearbox x0, x1, y0, y1, depth behind the gearbox face
    can_d=58.0,              # motor can diameter, axis along +X from x = can_x0
    can_len=95.0,
    can_x0=84.0,
    rot_deg=0.0,             # rotation of the whole motor about its shaft in the adapter (0: can points to -X world)
    bolt_d=6.0,              # boss bolt (M6), head on the cheek side of the adapter wing
    bolt_head_d=10.0,        # M6 SHCS head (hex head AF 10 fits the same slot)
    bolt_nut_af=10.0,        # nut behind a boss ear if the boss is not threaded
    bolt_nut_h=5.0,
    slot=4.0,                # slot half-length either way: every hole can sit +-4 mm off its nominal place
    mass_kg=1.3,
)

# Fixed frame interface (on the motor-side cheek): never changes when the motor is measured.
RING_R = 29.0                 # 6 x M3 heat-set inserts on this radius round the main axis
RING_N = 6
CHEEK_HOLE_D = 44.0           # cheek through hole round the main axis (coupler room)
SPIGOT_H = 3.0                # adapter spigot into the cheek hole (keys the plate radially)
PLATE_T = 10.0                # adapter plate thickness (PPA-CF)
HEAD_SEAT = 5.0               # plastic under the M6 head in the slot (head counterbore from the cheek-side face)


def wiper_cfg(P):
    """WIPER_DEFAULT updated with every non-None key of P.WIPER (and P.FRAMEWORK['wiper'] if present)."""
    w = dict(WIPER_DEFAULT)
    src = dict(getattr(P, "WIPER", {}) or {})
    fw = getattr(P, "FRAMEWORK", None) or {}
    src.update(fw.get("wiper", {}) if isinstance(fw, dict) else {})
    for k, v in src.items():
        if v is None or k in ("measured", "wires", "shaft_thread"):
            continue
        if k == "body_box" and isinstance(v, dict):
            v = (v["x0"], v["x1"], v["y0"], v["y1"], v["depth"])
        w[k] = v
    w["mount_holes"] = [tuple(map(float, h)) for h in w["mount_holes"]]
    return w


# ------------------------------------------------------------------------------------------------ motor model
def motor_body(w):
    """Gearbox + can + bosses + hub, in the M frame (bought part envelope)."""
    x0, x1, y0, y1, d = w["body_box"]
    zf = -w["boss_h"]
    gb = prim.box(x0, x1, y0, y1, zf - d, zf)
    can = prim.cylX(w["can_d"] / 2, w["can_x0"] - 1.0, w["can_x0"] + w["can_len"], 0.0, zf - d / 2)
    parts = [gb, can]
    for (hx, hy, hd) in w["mount_holes"]:
        parts.append(prim.cyl(w["boss_d"] / 2, w["boss_h"] + 0.5, hx, hy, zf - 0.5) - prim.cyl(hd / 2, 30, hx, hy, zf - 15))
    hub = prim.cyl(w["hub_d"] / 2, w["hub_h"] + w["boss_h"] + 0.5, 0, 0, zf - 0.5) - prim.cyl(w["shaft_d"] / 2 + 0.6, 40, z=zf - 20)
    parts.append(hub)
    return prim.union(parts)


def motor_shaft(w):
    """Output shaft stub (rotates): flats from the hub to shaft_len, then the M8 thread. M frame, axis +Z."""
    z0 = w["hub_h"] - 0.0
    r = w["shaft_d"] / 2
    sh = prim.cyl(r, w["shaft_len"] - z0, z=z0)
    f = w["shaft_flat"] / 2
    sh = sh - prim.box(f, r + 1, -r - 1, r + 1, z0 + 2.0, w["shaft_len"] + 1) - prim.box(-r - 1, -f, -r - 1, r + 1, z0 + 2.0, w["shaft_len"] + 1)
    th = prim.cyl(w["thread_d"] / 2, w["thread_len"], z=w["shaft_len"] - 0.01)
    # stub inside the hub so the shaft visibly enters the gearbox (overlap with the body is an allowed pair)
    inner = prim.cyl(r, z0 + w["boss_h"] + 0.01, z=-w["boss_h"])
    return prim.union([sh, th, inner])


# ------------------------------------------------------------------------------------------------ adapter plate
def holes_rot(w):
    """Motor boss holes in the adapter frame: the motor's own pattern turned by rot_deg (the cheek ring stays fixed)."""
    a = math.radians(w["rot_deg"])
    c, s_ = math.cos(a), math.sin(a)
    return [(c * x - s_ * y, s_ * x + c * y, d) for (x, y, d) in w["mount_holes"]]


def adapter_outline(w):
    pts = [Point(0, 0).buffer(RING_R + 8.0, 64)]
    for (hx, hy, hd) in holes_rot(w):
        pts.append(Point(hx, hy).buffer(w["slot"] + w["bolt_nut_af"] / 2 + 5.5, 48))
    return unary_union(pts).convex_hull


def ring_xy(i):
    a = math.radians(30.0 + i * 360.0 / RING_N)
    return RING_R * math.cos(a), RING_R * math.sin(a)


def adapter_plate(w, standoff=0.0):
    """Adapter plate in the adapter frame (M frame of the motor at rot_deg = 0): wings z 0..PLATE_T (motor bosses on
    z = 0), hub z 0..PLATE_T + standoff (cheek face), spigot beyond it into the cheek hole.

    Assembly (bench first): the M6 bolts go in from the cheek side of the wings, heads in the slot channels, through
    the slots into the motor bosses (threaded bosses, or a nut behind a boss ear). Slide to centre, tighten. Then the
    motor + adapter go onto the cheek: 6 x M3 from INSIDE the cheek box through the cheek into heat-set inserts in the
    hub face. Nothing is screwed behind the gearbox."""
    out = adapter_outline(w)
    plate = prim.extrude(prim.shapely_cs(out.buffer(0)), PLATE_T)
    tz = PLATE_T + standoff                    # cheek face (the hub stands the wings off over the block flanges)
    hub = prim.cyl(RING_R + 6.0, tz, z=0.0)
    spig = prim.tube(CHEEK_HOLE_D / 2 - 0.25, CHEEK_HOLE_D / 2 - 4.0, SPIGOT_H, z=tz - 0.01)
    plate = plate + hub + spig
    cut = []
    cut.append(prim.cyl(w["hub_d"] / 2 + 1.5, tz + SPIGOT_H + 2, z=-1))           # hub and shaft pass through
    for (hx, hy, hd) in holes_rot(w):
        r = math.hypot(hx, hy)
        ux, uy = (hx / r, hy / r) if r > 1e-6 else (1.0, 0.0)
        s = w["slot"]
        a0 = (hx - ux * s, hy - uy * s); a1 = (hx + ux * s, hy + uy * s)
        bolt = prim.extrude(U.slot_cs(*a0, *a1, w["bolt_d"] + fits.HOLE_CLEAR + 0.2), PLATE_T + 2, z=-1)
        head_w = w["bolt_head_d"] + 0.6
        chan = prim.extrude(U.slot_cs(*a0, *a1, head_w), PLATE_T - HEAD_SEAT + 0.01, z=HEAD_SEAT)   # head counterbore slot
        cut += [bolt, chan]
    for i in range(RING_N):
        x, y = ring_xy(i)
        cut.append(prim.place(U.insert_hole(0, 0, 0.0), tf.T(x, y, tz)))          # inserts in the hub face
    return prim.diff(plate, cut)


def ring_inserts_cutter():
    """Insert holes for the fixed 6 x M3 ring, in the M frame shifted so the cheek face is at z = PLATE_T (insert
    holes go +Z into the cheek from its outer face)."""
    cs = []
    for i in range(RING_N):
        x, y = ring_xy(i)
        cs.append(U.insert_hole(x, y, 0.0).transform(tf.to_m3(tf.Rx(180))).translate((0, 0, PLATE_T)))
    return prim.union(cs)


def adapter_checks(w):
    """Interference of the slotted motor holes with the fixed ring screws, plate fits the bed."""
    fails = []
    for (hx, hy, hd) in holes_rot(w):
        for i in range(RING_N):
            x, y = ring_xy(i)
            r = math.hypot(hx, hy)
            # closest distance from ring screw to the slot segment
            ux, uy = (hx / r, hy / r) if r > 1e-6 else (1.0, 0.0)
            best = 1e9
            for s in np.linspace(-w["slot"], w["slot"], 21):
                best = min(best, math.hypot(hx + ux * s - x, hy + uy * s - y))
            if best < (w["bolt_head_d"] + 0.6) / 2 + U.INS_D / 2 + 2.0:
                fails.append(f"motor hole ({hx:.1f}, {hy:.1f}) slot comes within {best:.1f} mm of ring screw {i}")
        if math.hypot(hx, hy) < RING_R + 6.0 + (w["bolt_head_d"] + 0.6) / 2 + 1.0 and math.hypot(hx, hy) > 1e-6:
            fails.append(f"motor hole ({hx:.1f}, {hy:.1f}) inside the adapter hub (head slot would cut the ring)")
        if math.hypot(hx, hy) < w["hub_d"] / 2 + w["bolt_nut_af"] / 2 + 3:
            fails.append(f"motor hole ({hx:.1f}, {hy:.1f}) too close to the output hub")
    b = adapter_outline(w).bounds
    if max(b[2] - b[0], b[3] - b[1]) > 250:
        fails.append("adapter plate larger than the bed")
    return fails


# ------------------------------------------------------------------------------------------------ saddle
def saddle(w, M_world, deck_top=0.0):
    """Motor saddle standing on the deck under the can end (world frame solid), or None if the can is not
    roughly horizontal. A cable tie (or hose clamp) through the two slots straps the can down."""
    x0, x1, y0, y1, d = w["body_box"]
    zf = -w["boss_h"]
    xc = w["can_x0"] + w["can_len"] - 22.0     # near the can end, clear of the cheek feet
    c0 = tf.apply(M_world, [(xc, 0.0, zf - d / 2)])[0]
    ax = M_world[:3, :3] @ np.array([1.0, 0, 0])
    if abs(ax[2]) > 0.2:
        return None, None
    r = w["can_d"] / 2
    zc = c0[2]
    if zc - r - deck_top < 3.0:
        return None, None
    width = 22.0
    yaw = math.degrees(math.atan2(ax[1], ax[0]))
    # in a local frame: x along the can axis, centred at the support point
    hgt = zc - deck_top
    blk = prim.box(-width / 2, width / 2, -(r + 8), r + 8, 0, hgt - r * 0.35)
    blk = blk - prim.cylX(r + 0.6, -width, width, 0.0, hgt)
    blk = blk.__add__(prim.box(-width / 2 - 10, width / 2 + 10, -(r + 8), r + 8, 0, 6))     # foot with screw lugs
    cuts = []
    for sy in (-1, 1):
        cuts.append(prim.box(-width / 2 - 1, width / 2 + 1, sy * (r + 2) - 2.5, sy * (r + 2) + 2.5, 8, 14))   # strap slot
    feet = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            fx, fy = sx * (width / 2 + 5), sy * (r + 2)
            cuts.append(U.clear_hole(fx, fy, -1, 7, head_top=6.01))
            feet.append((fx, fy))
    blk = prim.diff(blk, cuts)
    M = tf.T(c0[0], c0[1], deck_top) @ tf.Rz(yaw)
    holes = [tuple(tf.apply(M, [(fx, fy, 0)])[0][:2]) for fx, fy in feet]
    return prim.place(blk, M), holes
