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
    boss_face_y=-10.0,       # D48: world y of the boss face = the motor plate's rear face (params WIPER boss_face_y);
                             # cad-drum's coupler hub reads it too (drum.G), so the shaft and the hub move together
)

# Fixed frame interface (on the motor-side cheek): never changes when the motor is measured.
COUPLER_D = 67.0              # clear bore round the main axis (cad-drum coupler with the shear-pin limiter: r 32, y 0.5..46)
RING_R = 43.0                 # 6 x M3 heat-set inserts on this radius round the main axis
RING_N = 12                   # 12 clearance holes in the plate (30 deg); the adapter uses those clear of its motor slots
RING_PHASE = 0.0              # first ring position (deg) in the adapter frame
RING_MIN = 4                  # at least this many ring screws, no gap over 150 deg
HUB_R = 49.0                  # adapter hub (standoff ring) outer radius
CHEEK_HOLE_D = COUPLER_D      # plate through hole round the main axis
SPIGOT_H = 2.0                # the hub enters a 2 mm counterbore (r HUB_R + 0.3) in the plate face: keyed radially
PLATE_T = 10.0                # adapter plate thickness (PPA-CF)
HEAD_SEAT = 5.0               # plastic under the M6 head in the slot (head counterbore from the cheek-side face)


def wiper_cfg(P):
    """WIPER_DEFAULT updated with every non-None key of P.WIPER (and P.FRAMEWORK['wiper'] if present)."""
    w = dict(WIPER_DEFAULT)
    src = dict(getattr(P, "WIPER", {}) or {})
    fw = getattr(P, "FRAMEWORK", None) or {}
    src.update(fw.get("wiper", {}) if isinstance(fw, dict) else {})
    if src.get("orient_deg") is not None and src.get("rot_deg") is None:
        src["rot_deg"] = float(src["orient_deg"])      # params: motor +X to world +Z at 90 (same sense as rot_deg)
    for k, v in src.items():
        if v is None or k in ("measured", "wires", "shaft_thread", "orient_deg"):
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
    pts = [Point(0, 0).buffer(HUB_R + 2.0, 64)]
    for (hx, hy, hd) in holes_rot(w):
        pts.append(Point(hx, hy).buffer(w["slot"] + w["bolt_nut_af"] / 2 + 5.5, 48))
    return unary_union(pts).convex_hull


def ring_xy(i):
    a = math.radians(RING_PHASE + i * 360.0 / RING_N)
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
    hub = prim.cyl(HUB_R, tz + SPIGOT_H, z=0.0)            # standoff ring, its last 2 mm enter the plate counterbore
    plate = plate + hub
    cut = []
    cut.append(prim.cyl(COUPLER_D / 2, tz + SPIGOT_H + 2, z=-1))                   # the coupler turns inside
    for (hx, hy, hd) in holes_rot(w):
        r = math.hypot(hx, hy)
        ux, uy = (hx / r, hy / r) if r > 1e-6 else (1.0, 0.0)
        s = w["slot"]
        a0 = (hx - ux * s, hy - uy * s); a1 = (hx + ux * s, hy + uy * s)
        bolt = prim.extrude(U.slot_cs(*a0, *a1, w["bolt_d"] + fits.HOLE_CLEAR + 0.2), PLATE_T + 2, z=-1)
        head_w = w["bolt_head_d"] + 0.6
        chan = prim.extrude(U.slot_cs(*a0, *a1, head_w), tz + SPIGOT_H + 1 - HEAD_SEAT, z=HEAD_SEAT)   # head slot (open through the hub)
        cut += [bolt, chan]
    for i in ring_used(w):
        x, y = ring_xy(i)
        cut.append(prim.place(U.insert_hole(0, 0, 0.0), tf.T(x, y, tz + SPIGOT_H)))   # inserts in the hub face
    return prim.diff(plate, cut)


def ring_inserts_cutter():
    """Insert holes for the fixed 6 x M3 ring, in the M frame shifted so the cheek face is at z = PLATE_T (insert
    holes go +Z into the cheek from its outer face)."""
    cs = []
    for i in range(RING_N):
        x, y = ring_xy(i)
        cs.append(U.insert_hole(x, y, 0.0).transform(tf.to_m3(tf.Rx(180))).translate((0, 0, PLATE_T)))
    return prim.union(cs)


def ring_clear(w, i):
    """Smallest distance from ring position i to any motor head slot (centre line)."""
    x, y = ring_xy(i)
    best = 1e9
    for (hx, hy, hd) in holes_rot(w):
        r = math.hypot(hx, hy)
        ux, uy = (hx / r, hy / r) if r > 1e-6 else (1.0, 0.0)
        for s_ in np.linspace(-w["slot"], w["slot"], 21):
            best = min(best, math.hypot(hx + ux * s_ - x, hy + uy * s_ - y))
    return best


def ring_used(w):
    """Ring positions the adapter takes (an insert each): clear of every head slot."""
    need = (w["bolt_head_d"] + 0.6) / 2 + U.INS_D / 2 + 2.0
    return [i for i in range(RING_N) if ring_clear(w, i) >= need]


def adapter_checks(w):
    """Interference of the slotted motor holes with the fixed ring screws, plate fits the bed."""
    fails = []
    used = ring_used(w)
    if len(used) < RING_MIN:
        fails.append(f"only {len(used)} ring screws clear the motor slots (need {RING_MIN})")
    else:
        angs = sorted(i * 360.0 / RING_N for i in used)
        gaps = [(angs[(k + 1) % len(angs)] - angs[k]) % 360.0 for k in range(len(angs))]
        if max(gaps) > 150.0:
            fails.append(f"ring screws bunched: a {max(gaps):.0f} deg gap")
    for (hx, hy, hd) in holes_rot(w):
        if math.hypot(hx, hy) - w["slot"] - (w["bolt_head_d"] + 0.6) / 2 < COUPLER_D / 2 + 1.5:
            fails.append(f"motor hole ({hx:.1f}, {hy:.1f}): head slot breaks into the coupler bore")
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
            cuts.append(U.clear_hole(fx, fy, -1, 7))        # M3 x 10, head on the foot
            feet.append((fx, fy))
    blk = prim.diff(blk, cuts)
    M = tf.T(c0[0], c0[1], deck_top) @ tf.Rz(yaw)
    holes = [tuple(tf.apply(M, [(fx, fy, 0)])[0][:2]) for fx, fy in feet]
    return prim.place(blk, M), holes


def can_stay(w, M_motor, plate_y_face, side, z_plate_top, frac=0.6):
    """Can stay for a motor whose can points up: a post bolted to the motor plate's motor-side face (2 x M3 along y
    into plate inserts), an arm over to the can and a half-ring cradle on the can's -x side with two strap slots (cable
    tie or hose clamp). Deck-frame solid, the plate insert points [(x, z)], or (None, []) if the can is not vertical.
    side = motor_side (-1: the motor is at lower y than the plate)."""
    x0, x1, y0, y1, d = w["body_box"]
    zf = -w["boss_h"]
    ca = tf.apply(M_motor, [(w["can_x0"], 0.0, zf - d / 2), (w["can_x0"] + w["can_len"], 0.0, zf - d / 2)])
    ax = ca[1] - ca[0]
    if abs(ax[2]) < 0.9 * np.linalg.norm(ax):
        return None, []
    c = ca[0] + frac * ax                                # strap height on the can axis
    r = w["can_d"] / 2
    cx, cy, cz = c
    py0, py1 = sorted((plate_y_face, plate_y_face + side * 10.0))    # post: 10 mm off the plate face towards the motor
    px0, px1 = cx - r - 26.0, cx - r - 8.0               # beside the can on its -x side
    pz0 = z_plate_top - 26.0                             # clear of the adapter hub ring (r 49)
    post = prim.box(px0, px1, py0, py1, pz0, cz + 8.0)
    arm = prim.box(px0, cx - r - 2.0, min(py0, cy - 8.0), max(py1, cy + 8.0), cz - 8.0, cz + 8.0)
    ring = prim.cyl(r + 6.0, 16.0, cx, cy, cz - 8.0) - prim.cyl(r + 0.5, 20.0, cx, cy, cz - 10.0) \
        - prim.box(cx, cx + r + 10, cy - r - 10, cy + r + 10, cz - 9, cz + 9)        # half ring on the -x side
    sol = prim.union([post, arm, ring])
    for sy in (-1, 1):                                   # strap slots through the ring ends
        sol = sol - prim.box(cx - 3.0, cx + 1.0, cy + sy * (r + 3.0) - 1.6, cy + sy * (r + 3.0) + 1.6, cz - 10, cz + 10)
    pts = [((px0 + px1) / 2, pz0 + 6.0), ((px0 + px1) / 2, pz0 + 20.0)]
    yface = plate_y_face
    for (x, z) in pts:
        sol = sol - prim.cylY(U.CLR_D / 2, py0 - 1, py1 + 1, x, z) \
            - prim.cylY(U.HEAD_D / 2, (py0 - 1) if side < 0 else (py1 - U.HEAD_H), (py0 + U.HEAD_H) if side < 0 else (py1 + 1), x, z)
    return sol, pts


# ------------------------------------------------------------------------------------------------ v1.x puck adapter
# BASE_SPEC section 2: world y = 0 is the adapter plate FRONT face; the motor sits behind it (-y), its boss face on the
# puck's rear face (y -10 = WIPER boss_face_y, D48), and cad-drum's coupler hub reaches into the puck bore for its
# shaft. The adapter is a PUCK that fills a window in the frame's motor plate (both y -10..0),
# keyed by a lip behind the plate and held by M3 from the front at r RIM_R (outside the drum and reel zone, r < 51).
PUCK_R = 88.0                 # puck radius = plate window radius - 0.25
LIP_R = 97.0                  # rear lip outer radius (behind the plate)
LIP_T = 6.0                   # lip thickness (y -16..-10): clear of the gearbox face at y -18
RIM_R = 92.5                  # 8 x M3 from the plate front face into lip inserts
RIM_N = 8
BORE_R = 15.0                 # output boss r 12 + 3


def rim_xy(i):
    a = math.radians(22.5 + i * 360.0 / RIM_N)
    return RIM_R * math.cos(a), RIM_R * math.sin(a)


def puck(w, extra_inserts=()):
    """Puck adapter in the M frame (origin on the boss face plane, +Z to the front): body z 0..PLATE_T (front face =
    world y 0), lip z -LIP_T..0 from r PUCK_R - 6 to LIP_R. Motor slots with the M6 head counterbores open to the
    front (bench: bolt the motor on, heads from the front, into threaded bosses or nuts behind boss ears). Rim inserts
    in the lip; extra_inserts: (x, y) in the M frame on the front face (cad-drum's SW_INDEX block)."""
    t = PLATE_T
    body = prim.cyl(PUCK_R, t, z=0.0) + prim.tube(LIP_R, PUCK_R - 6.0, LIP_T, z=-LIP_T)
    cut = [prim.cyl(BORE_R, t + LIP_T + 2, z=-LIP_T - 1)]
    for (hx, hy, hd) in holes_rot(w):
        r = math.hypot(hx, hy)
        ux, uy = (hx / r, hy / r) if r > 1e-6 else (1.0, 0.0)
        s = w["slot"]
        a0 = (hx - ux * s, hy - uy * s); a1 = (hx + ux * s, hy + uy * s)
        cut.append(prim.extrude(U.slot_cs(*a0, *a1, w["bolt_d"] + fits.HOLE_CLEAR + 0.2), t + 2, z=-1))
        cut.append(prim.extrude(U.slot_cs(*a0, *a1, w["bolt_head_d"] + 0.6), t - HEAD_SEAT + 0.01, z=HEAD_SEAT))
    for i in range(RIM_N):
        x, y = rim_xy(i)
        cut.append(prim.place(U.insert_hole(0, 0, 0.0, depth=LIP_T - 0.6), tf.T(x, y, 0.0)))
    for (x, y) in extra_inserts:
        cut.append(prim.place(U.insert_hole(0, 0, 0.0), tf.T(x, y, t)))
    return prim.diff(body, cut)


def puck_checks(w, extra_inserts=()):
    fails = []
    head = (w["bolt_head_d"] + 0.6) / 2
    for (hx, hy, hd) in holes_rot(w):
        r = math.hypot(hx, hy)
        if r + w["slot"] + head + 3.0 > PUCK_R:
            fails.append(f"motor hole ({hx:.0f}, {hy:.0f}): slot reaches the puck rim (r {r + w['slot'] + head:.0f} > {PUCK_R - 8:.0f})")
        if r - w["slot"] - head < BORE_R + 2.0:
            fails.append(f"motor hole ({hx:.0f}, {hy:.0f}): head slot breaks into the bore")
        for (x, y) in extra_inserts:
            ux, uy = (hx / r, hy / r) if r > 1e-6 else (1.0, 0.0)
            d = min(math.hypot(hx + ux * s_ - x, hy + uy * s_ - y) for s_ in np.linspace(-w["slot"], w["slot"], 21))
            if d < head + U.INS_D / 2 + 2.0:
                fails.append(f"SW_INDEX insert ({x:.1f}, {y:.1f}) within {d:.1f} mm of the motor slot ({hx:.0f}, {hy:.0f})")
    return fails
