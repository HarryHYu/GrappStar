"""GrappStar base: frame subsystem (deck, side cheeks, bearing blocks, cheek ties, wiper motor adapter, motor saddle,
electronics bay, AS5600 bracket, guards). API: base/cad/README.md (parts, joints, kin, allowed, clearances, checks,
steps). Every number comes from params (LAYOUT, FRAMEWORK, WIPER, BOARDS, B608, SPRAG) with the defaults below, which
only stand in until BASE_SPEC.md fills them.

Structure
- Deck: flat plate on the rover, its own drill template (rover holes match the deck holes). Split into dovetailed
  tiles only when it exceeds the bed. Heat-set M3 inserts take every frame part foot.
- Cheeks: two vertical plates (XZ planes) at y = +-(gap/2 .. gap/2 + t), printed flat, bolted down through a foot
  flange. Every shaft whose axis is along Y gets a hole in each cheek it passes through and a bearing block bolted to
  the cheek's outer face (a spigot into the hole keys the block). Blocks carry the bearing seat (608 with crush ribs,
  the CSK20PP sprag OD, or a plain printed bushing for printed shafts).
- Ties: printed spacers between the cheeks (M3 into inserts at both ends) make the cheeks a box.
- Wiper motor: fixed 6 x M3 ring + 44 mm hole on the motor-side cheek. The adapter plate (PPA-CF, separately printed,
  every motor number from params.WIPER) bolts to that ring; the motor bolts to the adapter through slotted holes.
- Electronics bay and AS5600 bracket: see _frame_ebay.py.
"""
from __future__ import annotations

import math

import numpy as np
from shapely.geometry import Point, Polygon, LineString, box as sbox, MultiPoint
from shapely.ops import unary_union

from cad.lib import Part, Joint, Allow, Clearance, prim, tf, fits
from . import _frame_util as U
from . import _frame_motor as MOT

# ------------------------------------------------------------------------------------------------ defaults
# Placeholder shaft layout (deck coordinates, main axis 165 above the deck) until params.LAYOUT lands (world: x forward, y left, z up, origin on the deck top under
# the wiper axis). Keys per shaft: origin, axis, d (shaft diameter), brg ('608', 'sprag', 'bush', dict(od, w)),
# cheeks (which cheeks support it: (-1, 1), (1,), (-1,), ()).
LAYOUT_DEFAULT = {
    "main":    dict(origin=(0.0, 0.0, 165.0), axis=(0, 1, 0), d=20.0, brg="bush", cheeks=(1,)),
    "drum":    dict(origin=(-58.0, 0.0, 135.0), axis=(0, 1, 0), d=8.0, brg="608", cheeks=(-1, 1)),
    "launch":  dict(origin=(62.0, 0.0, 197.0), axis=(0, 1, 0), d=20.0, brg="bush", cheeks=(-1, 1)),
    "aim":     dict(origin=(10.0, 0.0, 245.0), axis=(0, 1, 0), d=8.0, brg="608", cheeks=(-1, 1)),
    "capstan": dict(origin=(-55.0, 0.0, 210.0), axis=(0, 1, 0), d=20.0, brg="bush", cheeks=(-1, 1)),
    "reel":    dict(origin=(-100.0, 0.0, 160.0), axis=(0, 1, 0), d=8.0, brg="608", cheeks=(-1, 1)),
    "aim_worm": dict(origin=(40.0, 135.0, 145.0), axis=(0, 0, 1), d=8.0, brg="608", supports=(-15.0, 25.0)),
    "aim_wheel": dict(origin=(135.0, 0.0, 155.0), axis=(1, 0, 0), d=8.0, brg="608", span=40.0),
}

FRAMEWORK_DEFAULT = dict(
    cheek_gap=124.0,        # inner distance between the cheeks
    cheek_t=10.0,           # cheek thickness
    cheek_margin=8.0,       # plastic round the outermost block screws
    deck_t=8.0,             # deck plate thickness (z from -deck_t to 0)
    deck_margin=6.0,        # deck edge beyond the outermost footprint
    deck_corner_r=8.0,
    deck_pocket=36.0,       # top-side lightening pocket size (square, 6 mm ribs)
    deck_pocket_depth=0.0,  # pocket depth (0 = solid deck: with 40 % infill pockets save almost nothing, they add walls)
    rover_bolt="M3",        # rover fixing: M3 x 25 SHCS + nyloc (Harry drills 3.4 mm through the rover plate)
    rover_hole_pitch=60.0,  # perimeter hole pitch (max), corners always get one
    motor_side=-1,          # cheek that carries the wiper motor (-1 = right, looking forward)
    foot_w=18.0,            # cheek foot flange width (outboard)
    foot_t=6.0,
    foot_pitch=45.0,        # foot screw pitch (max)
    block_t=6.0,            # bearing block flange thickness (min)
    ties=None,              # list of (x, z) cheek tie centres; None = automatic (top corners)
    tie_d=16.0,
    seat_d=20.0,            # v1.0: launch sleeve journal diameter in the far plate (cad-launch: d20 journal)
    seat_lip=2.0,           # thrust lip at the outboard end of the seat (LAUNCH engage pushes the sleeve outboard)
    spigot_clear_d=14.0,    # hole in the lip for the d12 spigot
    deck_z=-165.0,          # world z of the deck top (world z = 0 is the wiper axis; coordinator 11:00: axis 165 up)
    adapter_standoff=12.0,  # adapter hub height over the cheek (clears block flanges, M6 nuts reachable)
    cheek_y_in=None,        # (right, left) inner cheek face y; None = from the motor face (y = 0) and cheek_gap
    ebay=None,              # electronics bay placement override, see _frame_ebay.py
    motor_support=dict(plate_x=(-104.0, 111.0), plate_z=(-100.0, 120.0), leg_x=(78.0, 108.0), leg_y=(-55.0, -30.0),
                       bridge_z=(-22.0, 22.0)),   # v1.x motor plate, bridge and leg (relative to the main axis, y world)
    plate_back=75.0,        # v1.0 plates: start as a rectangle this far behind the main axis ...
    plate_front=105.0,      # ... and this far in front, deck to above the axis, then carved round the others
    guards=dict(hood_x=(-60.0, 62.0), hood_len=60.0, hood_z=56.0, front_x=104.0, front_len=106.0, front_top=-10.0),
    adapter_carve=True,     # carve the puck round the others (reel cup and capstan pass behind the plate below the axis)
    band=True,              # guard band round the cheek box
    band_cut=True,          # cut openings where the other subsystems pass (swept parts + 4 mm)
)

# printed bushing for printed shafts in a block (PPA-CF shaft in a PLA bush, see fits.SHAFT_IN_BORE)
BUSH_WALL = 4.0


def _get(P, name, default):
    v = getattr(P, name, None)
    return default if v is None else v


def cfg(P):
    """Resolved frame configuration: dict(fw=..., layout=..., wiper=..., derived numbers)."""
    fw = dict(FRAMEWORK_DEFAULT)
    fw["_raw"] = {}
    src = _get(P, "FRAMEWORK", {})
    if isinstance(src, dict):
        fw.update({k: v for k, v in src.items() if v is not None and k in FRAMEWORK_DEFAULT})
        fw["_raw"] = dict(src)
    lay_src = _get(P, "LAYOUT", None)
    c = dict(fw=fw, wiper=MOT.wiper_cfg(P))
    # cheek planes. World y = 0 is the wiper boss face plane (BASE_SPEC section 2): the motor-side cheek outer face
    # sits PLATE_T + adapter_standoff from it, towards the frame.
    ms, g, t = fw["motor_side"], fw["cheek_gap"], fw["cheek_t"]
    if fw.get("cheek_y_in"):
        yr, yl = fw["cheek_y_in"]
        y_in = {-1: float(yr), 1: float(yl)}
    else:
        yi = -ms * (MOT.PLATE_T + fw["adapter_standoff"] + t)
        y_in = {ms: yi, -ms: yi - ms * g}
    tt = {-1: t, 1: t}
    v1 = isinstance(lay_src, dict) and "axes" in lay_src
    c["v1"] = v1
    seat_e = None
    if v1:                                  # BASE_SPEC v1.0: the frame carries the main shaft only
        seat_e = next((e["y"] for e in lay_src.get("stack", []) if e.get("owner") == "cad-frame" and "outboard" in e.get("part", "")),
                      None)
        seat = seat_e or (160.0, 172.0)
        yi_m = 0.0                          # v1.x: the motor plate front face IS the spec's y = 0 (adapter plate face)
        if ms < 0:
            y_in = {-1: yi_m, 1: float(seat[0])}
            tt = {-1: t, 1: float(seat[1] - seat[0])}
        else:
            y_in = {1: yi_m, -1: float(seat[1])}
            tt = {1: t, -1: float(seat[1] - seat[0])}
    c["t"] = tt
    c["sides"] = (ms, -ms) if (not v1 or seat_e) else (ms,)      # v1.1 (D23): only the motor plate
    stack = lay_src.get("stack", []) if v1 else []
    c["as5600_frame"] = not any("AS5600 board" in e.get("part", "") and e.get("owner") != "cad-frame" for e in stack)
    c["y_in"] = y_in
    c["y_out"] = {s_: y_in[s_] + s_ * tt[s_] for s_ in (-1, 1)}
    yc = (y_in[-1] + y_in[1]) / 2
    layout = {}
    if v1:
        deck_z = float(lay_src.get("deck_top_z", fw["deck_z"]))
        for n, a_ in lay_src["axes"].items():
            ow = tuple(map(float, a_["origin"]))
            e = dict(origin=(ow[0], ow[1], ow[2] - deck_z), origin_w=ow, axis=tuple(map(float, a_["axis"])),
                     d=20.0, brg="seat", cheeks=(), frame=False, owner=a_.get("owner", ""))
            if n == "main":
                e.update(cheeks=((-ms,) if seat_e else ()), frame=True, brg="seat", d=float(fw.get("seat_d", 20.0)))
            layout[n] = e
    elif lay_src:
        deck_z = float(fw["deck_z"])
        for n, s_ in lay_src.items():
            ow = tuple(map(float, s_["origin"]))
            e = dict(origin=(ow[0], ow[1], ow[2] - deck_z), origin_w=ow, axis=tuple(map(float, s_["axis"])))
            e["d"] = float(s_.get("d", s_.get("shaft_d", 8.0)))
            e["brg"] = s_.get("brg", s_.get("bearing", "608" if e["d"] <= 8.0 else "bush"))
            e["cheeks"] = tuple(s_.get("cheeks", (-1, 1)))
            for k in ("supports", "span", "frame", "ped_yaw"):
                if k in s_:
                    e[k] = s_[k]
            layout[n] = e
    else:                                   # placeholder: built in deck coordinates, main axis at world z = 0
        deck_z = -LAYOUT_DEFAULT["main"]["origin"][2] if not fw["_raw"].get("deck_z") else float(fw["deck_z"])
        for n, v in LAYOUT_DEFAULT.items():
            e = dict(v)
            o = v["origin"]
            e["origin"] = (o[0], o[1] + yc, o[2])
            e["origin_w"] = (o[0], o[1] + yc, o[2] + deck_z)
            layout[n] = e
    c["layout"] = layout
    c["deck_z"] = deck_z
    c["to_world"] = tf.T(0, 0, deck_z)
    c["main"] = "main" if "main" in layout else next(iter(layout))
    return c


# ------------------------------------------------------------------------------------------------ geometry helpers
def _ymap(d):
    """Rotation taking a +Z-built cutter (entry at local z = 0, going -Z) so that it goes along d * Y from its entry."""
    return tf.Rx(90) if d > 0 else tf.Rx(-90)


def yfeat(solid, x, z, y_face, d):
    return prim.place(solid, tf.T(x, y_face, z) @ _ymap(d))


def is_y(axis):
    a = np.asarray(axis, float)
    return abs(abs(a[1]) - 1.0) < 1e-6


def block_geom(P, sh, fw):
    """Bearing block numbers for a shaft entry: seat od / w, spigot d, flange half size, screw radius."""
    if sh["brg"] == "seat":                     # integral plate seat (no separate block)
        return dict(od=sh["d"], w=0.0, spig=sh["d"], rs=0.0, half=sh["d"] / 2 + 12.0, tb=0.0, hole=sh["d"] + 0.3,
                    cb=U.HEAD_H, seat=True)
    if sh["brg"] == "bush":
        od, w = sh["d"] + 2 * BUSH_WALL, fw["cheek_t"] + fw["block_t"]
    else:
        od, w, _ = U.brg_dims(P, sh["brg"])
    spig = od + 6.0
    rs = spig / 2 + U.INS_D / 2 + 3.5           # insert centre radius on the cheek (3.3 mm wall to the hole)
    half = rs / math.sqrt(2) + U.INS_D / 2 + 3.0
    half = max(half, spig / 2 + 2.0)
    tb = max(fw["block_t"], w - (fw["cheek_t"] - 0.5) + 3.0)
    cb = max(U.HEAD_H, tb - 4.0)                 # counterbore: 2.8 to 4 mm under the head, so every block takes M3 x 8
    return dict(od=od, w=w, spig=spig, rs=rs, half=half, tb=tb, hole=spig + 0.4, cb=cb)


def block_solid(P, sh, fw):
    """Bearing block in its local frame: +Z outward along the shaft, z = 0 on the cheek outer face. Flange z 0..tb,
    spigot z -(cheek_t - 0.5)..0 into the cheek hole, bearing pocket opening at the spigot end (bearings ride on the
    shaft and the block slides on over them), lip at the outer end."""
    b = block_geom(P, sh, fw)
    tc = fw["cheek_t"]
    zsp = -(tc - 0.5)
    fl = prim.extrude(prim.rrect_cs(2 * b["half"], 2 * b["half"], 4.0), b["tb"])
    sp = prim.cyl(b["spig"] / 2, -zsp + 0.01, z=zsp)
    blk = fl.__add__(sp)
    cuts = []
    if sh["brg"] == "bush":
        cuts.append(prim.cyl((sh["d"] + fits.SHAFT_IN_BORE) / 2, 100, z=-50))
    else:
        od, w, bore = U.brg_dims(P, sh["brg"])
        cuts.append(U.bearing_seat_cutter(od, w, zsp, lip_id=U.lip_for(od, sh["d"])))
    for i in range(4):
        a = math.radians(45 + 90 * i)
        x, y = b["rs"] * math.cos(a), b["rs"] * math.sin(a)
        cuts.append(U.clear_hole(x, y, -1, b["tb"] + 1, head_top=b["tb"] + 0.01, head_h=b["cb"] + 0.01))
    return prim.diff(blk, cuts), b


def block_screws(b):
    return [(b["rs"] * math.cos(math.radians(45 + 90 * i)), b["rs"] * math.sin(math.radians(45 + 90 * i))) for i in range(4)]


def contact_intervals(outline, min_len=12.0):
    """x intervals where a plate outline stands on the deck (z = 0)."""
    strip = outline.intersection(sbox(-1e4, -0.01, 1e4, 0.5))
    geoms = getattr(strip, "geoms", [strip])
    out = sorted((g.bounds[0], g.bounds[2]) for g in geoms if not g.is_empty and g.bounds[2] - g.bounds[0] >= min_len)
    return out


def index_inserts(P, C, mo, ms):
    """cad-drum's SW_INDEX block screws (params DRUM.sw_index_mount, world (x, z) on the adapter front face) in the
    adapter frame (x -> world -x, y -> world z for the motor on -y)."""
    import re
    txt = str((getattr(P, "DRUM", {}) or {}).get("sw_index_mount", ""))
    pts = re.findall(r"\(x\s*([-+]?\d+(?:\.\d+)?),\s*z\s*([-+]?\d+(?:\.\d+)?)\)", txt)
    out = []
    for xs, zs in pts:
        x, z = float(xs), float(zs)
        zw = mo[2] + C["deck_z"]                 # main axis world z
        out.append((-(x - mo[0]), (z - zw) if ms < 0 else -(z - zw)))
    return out


def _swept_no_axis(C, axis_xz, margin=3.0):
    """3D union of the other subsystems' swept hulls (deck frame) except those that contain the main axis (shafts,
    sleeves, coupler: the frame bores take them)."""
    from manifold3d import Manifold
    from shapely.geometry import MultiPoint
    from . import _frame_guard as G
    ax = Point(*axis_xz)
    hs = []
    for cl in G.LAST_CLOUDS:
        if MultiPoint(cl[:, [0, 2]]).convex_hull.buffer(-1.0).contains(ax):
            continue
        hs.append(Manifold.hull_points(cl))
    return prim.union(hs)


def motor_support_v1(P, C, info, mo):
    """BASE_SPEC v1.x motor end. The motor plate occupies y -10..0 (front face = world y 0, the spec's adapter plate
    face); a window r PUCK_R takes the puck adapter, keyed by the puck's rear lip and held by 8 x M3 from the front at
    r RIM_R. A bridge behind the lip (y -30..-18, plus a connector beyond the lip radius) carries the plate to a leg in
    the free corner between the reel cup and the bay; the leg stands on a two-sided gusseted foot (4 x M3).
    Everything is carved in 3D round the other subsystems' sweeps; the plate keeps the piece round the window.
    Returns dict(plate=solid, parts=[...])."""
    fw = C["fw"]
    ms = fw["motor_side"]
    s = ms
    yf, yr = C["y_in"][s], C["y_out"][s]                 # front (frame side, y 0) and rear (motor side, y -10)
    y0, y1 = sorted((yf, yr))
    mx, mz = mo[0], mo[2]
    sp = fw["motor_support"]
    plate = prim.box(mx + sp["plate_x"][0], mx + sp["plate_x"][1], y0, y1, mz + sp["plate_z"][0], mz + sp["plate_z"][1])
    lx0, lx1 = mx + sp["leg_x"][0], mx + sp["leg_x"][1]
    ly0, ly1 = sp["leg_y"]
    by1 = yr - s * 0.0                                  # bridge meets the plate's rear face
    lip_back = yr + s * MOT.LIP_T                        # y of the lip's back face (-16)
    bz0, bz1 = mz + sp["bridge_z"][0], mz + sp["bridge_z"][1]
    bridge = prim.box(lx0, lx1, ly1, min(lip_back, yr) - 2.0, bz0, bz1)          # behind the lip, y -30..-18
    xc0 = mx + MOT.LIP_R + 2.0                                                   # connector beyond the lip radius
    conn = prim.box(xc0, mx + sp["plate_x"][1], ly1, yr + 0.01, bz0, bz1)
    plate = prim.union([plate, bridge, conn])
    cuts = [prim.cylY(MOT.PUCK_R + 0.25, y0 - 1, y1 + 1, mx, mz)]
    for i in range(MOT.RIM_N):
        rx, rz = MOT.rim_xy(i)
        wx, wz = mx - rx, mz + (rz if ms < 0 else -rz)
        cuts.append(prim.cylY(U.CLR_D / 2, y0 - 1, y1 + 1, wx, wz))
        cuts.append(yfeat(prim.cyl(U.HEAD_D / 2, U.HEAD_H + 0.01, z=-U.HEAD_H), wx, wz, yf, s))   # head on the front face
    # leg: column, two-sided foot with gussets, 2 x M3 along y into the bridge (long counterbore: 8 mm grip)
    ztop = bz1
    leg = prim.box(lx0, lx1, ly0, ly1, 0.0, ztop)
    fy0, fy1 = ly0 - 22.0, ly1 + 22.0
    leg = leg + prim.box(lx0, lx1, fy0, fy1, 0.0, 6.0)
    for xg in (lx0 + 2.0, lx1 - 6.0):
        for (ya, yb) in ((fy0, ly0), (ly1, fy1)):
            g_ = prim.extrude(prim.poly_cs([(0, 0), (abs(yb - ya), 0), (0, 40.0)]), 4.0)
            # local x along -y or +y away from the column face, local y up; extrude along world x
            if ya < ly0:       # -y side: from the column face at ly0 outwards (occupies x xg..xg+4)
                M = tf.T(xg + 4.0, ly0, 0) @ tf.Rz(-90) @ tf.Rx(90)
            else:              # +y side (occupies x xg..xg+4)
                M = tf.T(xg, ly1, 0) @ tf.Rz(90) @ tf.Rx(90)
            leg = leg + prim.place(g_, M)
    jz = [bz0 + 10.0, bz1 - 10.0]
    xc = (lx0 + lx1) / 2
    for z in jz:
        leg = leg - prim.cylY(U.CLR_D / 2, ly0 - 1, ly1 + 1, xc, z) - prim.cylY(U.HEAD_D / 2, ly0 - 1, ly1 - 8.0, xc, z)
        cuts.append(yfeat(U.insert_hole(0, 0, 0.0), xc, z, ly1, 1))
    feet = [(lx0 + 8.0, fy0 + 8.0), (lx1 - 8.0, fy0 + 8.0), (lx0 + 8.0, fy1 - 8.0), (lx1 - 8.0, fy1 - 8.0)]
    for fx, fy in feet:
        leg = leg - U.clear_hole(fx, fy, -1, 7)
    plate = prim.diff(plate, cuts)
    carve = _swept_no_axis(C, (mx, mz))
    if not carve.is_empty():
        plate = U.carve_healed(plate, carve, keep_largest=False)
        leg_c = U.carve_healed(leg, carve)
        if abs(leg_c.volume() - leg.volume()) > 1.0:
            info["warnings"].append(f"motor leg cut by {leg.volume() - leg_c.volume():.0f} mm3 of another subsystem's sweep")
        leg = leg_c
    comps = plate.decompose()
    keep = [c for c in comps if not (c ^ prim.box(mx - MOT.PUCK_R - 8, mx + MOT.PUCK_R + 8, y0 - 1, y1 + 1, mz - 12, mz + 12)).is_empty()]
    plate = max(keep or comps, key=lambda c: c.volume())
    if (plate ^ bridge).volume() < 0.5 * bridge.volume():
        info["warnings"].append("motor plate: the carve separated the plate from its bridge")
    rim_ok = 0
    for i in range(MOT.RIM_N):
        rx, rz = MOT.rim_xy(i)
        wx, wz = mx - rx, mz + (rz if ms < 0 else -rz)
        ring_ = prim.cylY(4.5, y0 + 0.5, y1 - 0.5, wx, wz) - prim.cylY(2.0, y0, y1, wx, wz)
        if (plate ^ ring_).volume() > 0.8 * ring_.volume():
            rim_ok += 1
            info.setdefault("plate_rim_ok", []).append(i)
    info["rim_screws_ok"] = rim_ok
    if rim_ok < 5:
        info["warnings"].append(f"only {rim_ok} of {MOT.RIM_N} puck rim screws have plate behind them")
    info["deck_holes"] += feet
    info["motor_leg_feet"] = feet
    parts = [Part("frame_motor_leg", leg, "PLA", print_tf=U.print_flat(leg), color="#aab4be",
                  notes="motor support leg (prints upright): 4 x M3 x 10 into the deck, 2 x M3 x 12 along y into the "
                        "bridge behind the motor plate")]
    return dict(plate=plate, parts=parts)


def carve_plate(poly, C, s, info, axis_xz, margin=3.0):
    """Remove from a plate outline (x, z, deck frame) the XZ shadow of every other subsystem's swept hull that reaches
    the plate's y slab, plus the bay. Hulls that contain the main axis (shafts and sleeves through the plate) are left
    to the bores. Keeps the piece that holds the main axis."""
    from shapely.geometry import MultiPoint
    from . import _frame_guard as G
    y0, y1 = sorted((C["y_in"][s], C["y_out"][s]))
    y0 -= 2.0
    y1 += 2.0
    if s == C["fw"]["motor_side"]:
        y0 -= C["fw"]["foot_w"] + 1.0          # the foot flange sits outboard of the motor plate
    else:
        y1 += C["fw"]["foot_w"] + 1.0
    cut = []
    ax = Point(*axis_xz)
    for cl in G.LAST_CLOUDS:
        if cl[:, 1].max() < y0 or cl[:, 1].min() > y1:
            continue
        h = MultiPoint(cl[:, [0, 2]]).convex_hull
        if h.buffer(-1.0).contains(ax):
            continue
        cut.append(h.buffer(margin - 4.0 + 0.01) if margin < 4.0 else h)
    eb = C["fw"].get("_raw", {}).get("ebay")
    if isinstance(eb, dict) and "x" in eb:
        if not (eb["y"][1] < y0 or eb["y"][0] > y1):
            cut.append(sbox(eb["x"][0] - 3, -1, eb["x"][1] + 3, 75.0))
    if cut:
        poly = poly.difference(unary_union(cut))
    geoms = getattr(poly, "geoms", [poly])
    keep = [g for g in geoms if g.buffer(1.0).contains(ax) or g.distance(ax) < 1.0]
    if not keep:
        info["warnings"].append(f"plate {s:+d}: the carve removed the main axis region")
        keep = [max(geoms, key=lambda g: g.area)]
    out = keep[0].buffer(-1.5).buffer(1.5)
    if out.geom_type != "Polygon":
        out = max(out.geoms, key=lambda g: g.area)
    info.setdefault("plate_carved", {})[s] = len(cut)
    return out


def split_cheek(sol, outline, sups, P, fw, info, limit=246.0, tol=0.15):
    """Split a cheek solid that does not fit the bed into pieces with dovetail seams in the cheek plane (x, z).
    Seams go where no bearing block or motor ring is. Returns a list of solids (one if it fits)."""
    from shapely.geometry import box as sb
    x0, z0, x1, z1 = outline.bounds
    seams = []
    for axis, lo, hi in (("x", x0, x1), ("y", z0, z1)):        # 2D axes: x = world x, y = world z
        n = int(math.ceil((hi - lo) / limit))
        for i in range(1, n):
            c0 = lo + i * (hi - lo) / n
            best = None
            for c in np.arange(c0 - 80, c0 + 80.1, 2.0):
                if not (hi - c <= limit and c - lo <= limit * (n - i)) and n == 2:
                    continue
                ok = True
                for nme, sh in sups:
                    b = block_geom(P, sh, fw)
                    q = sh["origin"][0] if axis == "x" else sh["origin"][2]
                    if abs(q - c) < b["half"] + 14:
                        ok = False
                        break
                if ok and (best is None or abs(c - c0) < abs(best - c0)):
                    best = c
            if best is None:
                info["warnings"].append(f"cheek split: no free seam near {axis} = {c0:.0f}")
                best = c0
            seams.append(dict(axis=axis, c=float(best), n=2))
    if not seams:
        return [sol]
    pieces2d = U.split_plate(outline.buffer(40.0), 0, seams, tol=tol)     # oversize: the foot and gussets are outside
    out = []
    for pc in pieces2d:
        prism = prim.place(prim.extrude(prim.shapely_cs(pc), 400.0, z=-200.0), tf.Rx(90))   # 2D (x, z) -> world, along y
        piece = sol ^ prism
        if not piece.is_empty() and piece.volume() > 100.0:
            out += sorted(piece.decompose(), key=lambda m: -m.volume())[:1]
    info.setdefault("cheek_seams", []).append(seams)
    return out


# ------------------------------------------------------------------------------------------------ build
_CACHE = {}


def build(P):
    """Build every frame solid once per params module (cached). Returns dict with parts list and metadata."""
    key = repr([getattr(P, k, None) for k in ("LAYOUT", "FRAMEWORK", "WIPER", "B608", "SPRAG", "BED", "DRUM")])
    if key in _CACHE:
        return _CACHE[key]
    C = cfg(P)
    fw, lay, w = C["fw"], C["layout"], C["wiper"]
    tc, g = fw["cheek_t"], fw["cheek_gap"]
    ms = fw["motor_side"]
    parts, info = [], dict(cfg=C, supports=[], deck_holes=[], rover_holes=[], feet=[], warnings=[])
    # ---- the other subsystems: swept volumes (band openings, bay keep-out), feet and screw holes on the deck
    from . import _frame_guard as G
    from . import _frame_others as OTH
    info["swept"] = G.swept_cutters(P, C)
    opolys, oholes, oown = OTH.deck_contacts(P, C["deck_z"])
    info["others_foot"], info["others_holes"] = opolys, oholes
    info["deck_holes"] += oholes
    info["keepout"] = unary_union([g for g in opolys + [G.shadow(0.0, 80.0)] if g is not None]) \
        if (opolys or not info["swept"].is_empty()) else None


    # ---- supports per cheek
    supports = {-1: [], 1: []}
    for n, sh in lay.items():
        if not is_y(sh["axis"]):
            continue                           # pedestals, built below
        for s in sh["cheeks"]:
            if n == C["main"] and s == ms:
                continue                       # the motor adapter takes this end
            supports[s].append((n, sh))
    # ---- cheek outlines (x, z) from the supports + motor ring
    feats = {-1: [], 1: []}
    for s in (-1, 1):
        for n, sh in supports[s]:
            b = block_geom(P, sh, fw)
            feats[s].append(Point(sh["origin"][0], sh["origin"][2]).buffer(b["half"] * math.sqrt(2) + fw["cheek_margin"], 64))
        if C["main"] in lay:
            mo = lay[C["main"]]["origin"]
            r_ring = MOT.HUB_R + fw["cheek_margin"] + 2
            if s == ms:
                feats[s].append(Point(mo[0], mo[2]).buffer(r_ring, 64))
            # AS5600 bracket ring on the far cheek: the main block is already there
    xs = [p.bounds[0] for s in (-1, 1) for p in feats[s]] + [p.bounds[2] for s in (-1, 1) for p in feats[s]]
    x0c, x1c = min(xs), max(xs)
    outlines = {}
    for s in C["sides"]:
        base = sbox(x0c, 0.0, x1c, 24.0)
        poly = unary_union(feats[s] + [base]).convex_hull
        poly = poly.intersection(sbox(-1e4, 0.0, 1e4, 1e4))
        if C["v1"] and C["main"] in lay:      # a plate from the deck up past the main axis, carved round the others
            mo = lay[C["main"]]["origin"]
            top = mo[2] + (MOT.HUB_R + 10.0 if s == ms else fw["seat_d"] / 2 + 22.0)
            poly = unary_union([poly, sbox(mo[0] - fw["plate_back"], 0.0, mo[0] + fw["plate_front"], top)])
            poly = carve_plate(poly, C, s, info, (mo[0], mo[2]))
        outlines[s] = poly
    xs = [outlines[s].bounds[0] for s in C["sides"]] + [outlines[s].bounds[2] for s in C["sides"]]
    x0c, x1c = min(xs), max(xs)
    info["cheek_x"] = (x0c, x1c)

    # ---- cheek solids
    cheek_world = {}
    v1_motor = None
    if C["v1"] and C["main"] in lay:
        v1_motor = motor_support_v1(P, C, info, lay[C["main"]]["origin"])
        parts += v1_motor["parts"]
    for s in C["sides"]:
        if v1_motor is not None and s == ms:
            cheek_world[s] = v1_motor["plate"]
            continue
        cs = prim.shapely_cs(outlines[s])
        # extrude along local Z then Rx(90): local (X, Y, Z) -> world (X, -Z, Y); y from y_ref to y_ref - t
        y_ref = C["y_in"][s] if s < 0 else C["y_out"][s]
        sol = prim.place(prim.extrude(cs, C["t"][s]), tf.T(0, y_ref, 0) @ tf.Rx(90))
        yo = C["y_out"][s]
        # foot flange (outboard) along every stretch of the bottom edge that stands on the deck
        ivals = contact_intervals(outlines[s])
        if not ivals:
            info["warnings"].append(f"cheek {s:+d}: no edge reaches the deck (the carve cut it off)")
            ivals = [(x0c + 2, x1c - 2)]
        fx0, fx1 = ivals[0]
        foot = prim.union([prim.box(a + 2, b - 2, yo, yo + s * fw["foot_w"], 0.0, fw["foot_t"]) for a, b in ivals])
        # gussets: evenly spread, skipping x ranges where a block, the motor adapter or the AS5600 bracket comes low
        blocked = []
        for n, sh in supports[s]:
            b = block_geom(P, sh, fw)
            if sh["origin"][2] - b["half"] < 34.0:
                blocked.append((sh["origin"][0] - b["half"] - 4, sh["origin"][0] + b["half"] + 4))
        if C["main"] in lay:
            mo_ = lay[C["main"]]["origin"]
            if s == ms:
                ob = MOT.adapter_outline(w).bounds
                r_ad = max(abs(ob[0]), abs(ob[1]), abs(ob[2]), abs(ob[3]))
                blocked.append((mo_[0] - r_ad - 4, mo_[0] + r_ad + 4))
            else:
                blocked.append((mo_[0] - 20, mo_[0] + 20))
        gx = []
        for (ia, ib) in ivals:
            for xg in np.linspace(ia + 8, ib - 8, max(2, int((ib - ia) / 30) + 1)):
                if all(not (a <= xg <= b) for a, b in blocked) and all(abs(xg - q) > 40 for q in gx) and ib - ia > 30:
                    gx.append(float(xg))
        gus = []
        for xg in gx:
            tri = prim.poly_cs([(0, 0), (fw["foot_w"] - 2, 0), (0, 30)])
            gsol = prim.place(prim.extrude(tri, 4.0, z=-2.0), tf.T(xg, yo, 0) @ tf.Rz(90 if s > 0 else -90) @ tf.Rx(90))
            gus.append(gsol)
        sol = prim.union([sol, foot] + gus)
        cuts = []
        # foot screws: M3 down into deck inserts
        feet = []
        xfs = []
        for (ia, ib) in ivals:
            if ib - ia < 18:
                continue
            nf = max(1 if ib - ia < 40 else 2, int(math.ceil((ib - ia - 16) / fw["foot_pitch"])) + 1)
            xfs += [(ia + ib) / 2] if nf == 1 else [ia + 8 + i * (ib - ia - 16) / (nf - 1) for i in range(nf)]
        for xf in xfs:
            for _ in range(4):                                  # dodge the gussets (away from the nearest one)
                if not gx:
                    break
                q = min(gx, key=lambda g: abs(xf - g))
                if abs(xf - q) >= 8.0:
                    break
                xf = q + (8.0 if xf >= q else -8.0)
            yf = yo + s * (fw["foot_w"] / 2 + 1.0)
            feet.append((xf, yf))
            cuts.append(U.clear_hole(xf, yf, -1, fw["foot_t"] + 1))     # M3 x 10, head on the foot top
        info["deck_holes"] += feet
        # cable-tie points between foot screws: two slots through the foot and a 2 mm tunnel under it joining them
        ties_ = []
        for (xa, _), (xb, _) in zip(feet[:-1], feet[1:]):
            xm = (xa + xb) / 2
            if (not gx or min(abs(xm - q) for q in gx) > 8.0) and all(abs(xm - q[0]) > 7.0 for q in feet):
                yc_ = yo + s * (fw["foot_w"] / 2 + 1.0)
                for sy_ in (-1, 1):
                    cuts.append(prim.box(xm - 2.5, xm + 2.5, yc_ + sy_ * 5.5 - 1.2, yc_ + sy_ * 5.5 + 1.2, -1, fw["foot_t"] + 1))
                cuts.append(prim.box(xm - 2.5, xm + 2.5, yc_ - 6.7, yc_ + 6.7, -1, 2.0))
                ties_.append((xm, yc_))
        info.setdefault("cable_ties", {})[s] = ties_
        info["feet"].append(dict(x0=min(a for a, b in ivals), x1=max(b for a, b in ivals), y0=min(yo, yo + s * fw["foot_w"]) - 0.5, y1=max(yo, yo + s * fw["foot_w"]) + 0.5))
        # shaft holes + block inserts
        for n, sh in supports[s]:
            b = block_geom(P, sh, fw)
            ox, _, oz = sh["origin"]
            if b.get("seat"):
                # journal bore through the plate up to the thrust lip at the outboard face, spigot hole in the lip
                yi_, lip = C["y_in"][s], fw["seat_lip"]
                ylip = yo - s * lip
                cuts.append(prim.cylY(b["hole"] / 2, min(yi_, ylip) - 1, max(yi_, ylip), ox, oz))
                cuts.append(prim.cylY(fw["spigot_clear_d"] / 2, min(yo, ylip) - 1, max(yo, ylip) + 1, ox, oz))
                cuts.append(prim.cylY(b["hole"] / 2 + 0.6, min(yi_, yi_ + s * 0.6) - 0.01, max(yi_, yi_ + s * 0.6) + 0.01, ox, oz))
                info["supports"].append(dict(shaft=n, side=s, origin=sh["origin"], brg="seat", block=b, part=None,
                                             thrust_face_y=ylip))
                continue
            cuts.append(prim.cylY(b["hole"] / 2, min(yo, C["y_in"][s]) - 1, max(yo, C["y_in"][s]) + 1, ox, oz))
            for (lx, ly) in block_screws(b):
                cuts.append(yfeat(U.insert_hole(0, 0, 0.0), ox + lx, oz + ly, yo, -s))
            info["supports"].append(dict(shaft=n, side=s, origin=sh["origin"], brg=sh["brg"], block=b))
        # through holes for shafts that pass this cheek without a block (main on the motor side)
        if s == ms and C["main"] in lay:
            mo = lay[C["main"]]["origin"]
            cuts.append(prim.cylY(MOT.CHEEK_HOLE_D / 2, min(yo, C["y_in"][s]) - 1, max(yo, C["y_in"][s]) + 1, mo[0], mo[2]))
            cuts.append(prim.cylY(MOT.HUB_R + 0.3, min(yo, yo - s * MOT.SPIGOT_H) - 0.01, max(yo, yo - s * MOT.SPIGOT_H) + 0.01,
                                  mo[0], mo[2]))       # counterbore that keys the adapter hub
            for i in range(MOT.RING_N):
                rx, rz = MOT.ring_xy(i)
                # adapter frame x -> world -x (x_hint), y -> world z (for motor_side -1); mirror for +1
                wx, wz = mo[0] - rx, mo[2] + (rz if ms < 0 else -rz)
                yin = C["y_in"][s]
                cuts.append(prim.cylY(U.CLR_D / 2, min(yo, yin) - 1, max(yo, yin) + 1, wx, wz))
                cuts.append(yfeat(prim.cyl(U.HEAD_D / 2, U.HEAD_H + 0.01, z=-U.HEAD_H), wx, wz, yin, s))   # head on the inner face
        cheek_world[s] = prim.diff(sol, cuts)

    # ---- ties between the cheeks (auto: near the top of the overlap of both outlines, clear of every block)
    ties = fw["ties"]
    if ties is None and C["v1"]:
        ties = []                              # v1.0: the plates are tied by the deck (everything between them moves)
    if ties is None:
        both = outlines[-1].intersection(outlines[1]).buffer(-14.0)
        blocked = unary_union([Point(sh["origin"][0], sh["origin"][2]).buffer(block_geom(P, sh, fw)["half"] * 1.5 + 4)
                               for n, sh in lay.items() if is_y(sh["axis"])])
        free = both.difference(blocked)
        ties = []
        if not free.is_empty:
            cands = []
            for xq in np.linspace(x0c, x1c, 60):
                for zq in np.linspace(10, 400, 160):
                    if free.contains(Point(xq, zq)):
                        cands.append((xq, zq))
            if cands:
                cands = np.array(cands)
                # front-most and rear-most free points in the upper half of the free region (widest stance)
                zmed = np.median(cands[:, 1])
                up = cands[cands[:, 1] >= zmed]
                ties.append(tuple(up[np.argmax(up[:, 0] + 0.05 * up[:, 1])]))
                ties.append(tuple(up[np.argmin(up[:, 0] - 0.05 * up[:, 1])]))
                low = cands[cands[:, 1] < zmed]
                if len(low):
                    far = low[np.argmax(np.min([np.hypot(low[:, 0] - tx, low[:, 1] - tz) for tx, tz in ties], axis=0))]
                    ties.append(tuple(far))
                ties = [(round(float(a), 1), round(float(b), 1)) for a, b in ties]
    info["ties"] = ties
    for i, (tx, tz) in enumerate(ties):
        r = fw["tie_d"] / 2
        tie = prim.cylY(r, C["y_in"][-1] + 0.0, C["y_in"][1], tx, tz)
        # M3 inserts at both ends, screws through the cheeks from outside
        for s in (-1, 1):
            tie = tie - yfeat(U.insert_hole(0, 0, 0.0), tx, tz, C["y_in"][s], -s)
            cheek_world[s] = cheek_world[s] - prim.cylY(U.CLR_D / 2, min(C["y_in"][s], C["y_out"][s]) - 1,
                                                        max(C["y_in"][s], C["y_out"][s]) + 1, tx, tz) \
                - yfeat(prim.cyl(U.HEAD_D / 2, U.HEAD_H + 0.01, z=-U.HEAD_H), tx, tz, C["y_out"][s], -s)
        # a flat on the bottom so it prints lying down without support
        tie = tie - prim.box(tx - r - 1, tx + r + 1, C["y_in"][-1] - 1, C["y_in"][1] + 1, tz - r - 1, tz - r + 1.5)
        parts.append(Part(f"frame_tie_{i + 1}", tie, "PLA", print_tf=U.print_flat(tie), color="#9aa4ae",
                          notes="cheek tie: spacer between the cheeks, M3 x 12 through each cheek into an insert at each end"))

    # ---- guard band round the cheek box (openings where other subsystems pass)
    if fw["band"] and not C["v1"]:
        from . import _frame_guard as G
        both = outlines[-1].intersection(outlines[1])
        cut = info["swept"] if fw["band_cut"] else None
        bands, bholes = G.band_parts(C, both, cut)
        info["band_holes"] = bholes
        for i, sol in enumerate(bands):
            parts.append(Part(f"frame_guard_band_{i + 1}", sol, "PLA", color="#d0d6dc",
                              print_tf=U.print_flat(sol, tf.Rx(90)),
                              notes="guard band: closes the cheek box (top, front, back); M3 x 12 through each cheek into its lugs"))
        for s in (-1, 1):
            for (hx, hz) in bholes:
                cheek_world[s] = cheek_world[s] - prim.cylY(U.CLR_D / 2, min(C["y_in"][s], C["y_out"][s]) - 1,
                                                            max(C["y_in"][s], C["y_out"][s]) + 1, hx, hz) \
                    - yfeat(prim.cyl(U.HEAD_D / 2, U.HEAD_H + 0.01, z=-U.HEAD_H), hx, hz, C["y_out"][s], -s)

    for s in C["sides"]:
        nm = "frame_cheek_r" if s < 0 else "frame_cheek_l"
        M = tf.Rx(-90) if s < 0 else tf.Rx(90)          # print flat: the cheek's outer face down
        note = (("right" if s < 0 else "left") + " cheek (looking forward): carries the bearing blocks"
                + (" and the wiper motor adapter" if s == ms else "") + "; foot flange bolts to deck inserts")
        pieces = split_cheek(cheek_world[s], outlines[s], supports[s], P, fw, info)
        for k, sol in enumerate(pieces):
            nmk = nm if len(pieces) == 1 else f"{nm}_{k + 1}"
            parts.append(Part(nmk, sol, "PLA", print_tf=U.print_flat(sol, M), color="#aab4be",
                              notes=note + ("" if len(pieces) == 1 else "; piece %d of %d, dovetail seam" % (k + 1, len(pieces)))))

    # ---- bearing blocks
    for sp in info["supports"]:
        if sp["brg"] == "seat":
            continue
        n, s = sp["shaft"], sp["side"]
        sh = lay[n]
        sol, b = block_solid(P, sh, fw)
        ox, _, oz = sh["origin"]
        # block local +Z = outward = s * Y
        M = tf.T(ox, C["y_out"][s], oz) @ (tf.Rx(-90) if s > 0 else tf.Rx(90))
        mat = "PPA-CF" if sh["brg"] == "sprag" else "PLA"      # bushings stay PLA (PPA-CF shaft in a PLA bush)
        nm = f"frame_blk_{n}_{'l' if s > 0 else 'r'}"
        parts.append(Part(nm, prim.place(sol, M), mat, print_tf=None, color="#c9b27a" if mat == "PPA-CF" else "#b8c2cc",
                          notes=f"bearing block, {n} shaft ({sh['brg']}), 4 x M3 x 8 into cheek inserts"))
        p = parts[-1]
        p.print_tf = U.print_flat(p.solid, tf.Rx(-90) if s > 0 else tf.Rx(90))  # flange outer face down, spigot up
        sp["part"] = nm

    # ---- pedestals for shafts not along Y
    from . import _frame_ped as PED
    for n, sh in lay.items():
        if is_y(sh["axis"]) or not sh.get("frame", True):
            continue
        pp, hh, ww = PED.pedestals(P, n, sh, fw)
        parts += pp
        info["deck_holes"] += hh
        info["warnings"] += ww
        info.setdefault("pedestals", []).append(dict(shaft=n, parts=[p.name for p in pp]))

    # ---- wiper motor, shaft, adapter
    if C["main"] in lay:
        mo = lay[C["main"]]["origin"]
        yo = C["y_out"][ms]
        tbs = [sp["block"]["tb"] for sp in info["supports"] if sp["side"] == ms]
        standoff = fw["adapter_standoff"]
        if tbs and standoff < max(tbs) + 1.0:
            info["warnings"].append(f"adapter_standoff {standoff} < thickest motor-side block flange {max(tbs):.1f} + 1")
        info["adapter_standoff"] = standoff
        yo = yo + ms * standoff
        # M frame: origin at the boss face plane (adapter face away from the cheek), +Z towards the frame (= -ms * Y)
        zdir = np.array([0.0, -ms, 0.0])
        xh = np.array([-1.0, 0.0, 0.0])
        y_boss = (C["y_out"][ms]) if C["v1"] else (yo + ms * MOT.PLATE_T)   # v1: boss face on the plate rear (y -10)
        Mm = tf.frame((mo[0], y_boss, mo[2]), zdir, xh) @ tf.Rz(w["rot_deg"])
        info["M_motor"] = Mm
        body = MOT.motor_body(w)
        parts.append(Part("frame_wiper_motor", body, "bought", place=Mm, color="#3c4248",
                          bought_ref="12 V car wiper motor (self-locking worm), 2-wire rebuild",
                          notes="wiper motor ENVELOPE (assumed, not measured): edit params.WIPER and reprint the adapter"))
        Mad = tf.frame((mo[0], y_boss, mo[2]), zdir, xh)    # the ring is fixed; rot only moves the slots
        if C["v1"]:
            extra = index_inserts(P, C, mo, ms)
            info["index_inserts"] = extra
            ad = MOT.puck(w, extra)
            info["warnings"] += MOT.puck_checks(w, extra)
        else:
            ad = MOT.adapter_plate(w, standoff)
        ad_rot = ad                                   # slots already turned by rot_deg; the ring stays fixed
        if C["v1"] and fw["adapter_carve"]:
            # carve the adapter wings round the other subsystems (drum index block, reel sprag hub) in the adapter
            # frame; the slots and the ring inserts must survive (checked)
            carve = _swept_no_axis(C, (mo[0], mo[2]))
            if not carve.is_empty():
                Minv = tf.inv(Mad)
                cut_local = prim.place(carve, Minv)
                ad_c = ad_rot - cut_local
                lost = ad_rot.volume() - ad_c.volume()
                if lost > 1.0:
                    info.setdefault("adapter_carved_mm3", lost)
                    for (hx, hy, hd) in MOT.holes_rot(w):
                        probe = prim.cyl(w["bolt_head_d"] / 2 + 2.0, MOT.PLATE_T, hx, hy, 0.0)
                        if (probe ^ cut_local).volume() > 1.0:
                            info["warnings"].append(f"adapter carve reaches the motor hole at ({hx:.0f}, {hy:.0f})")
                    if not C["v1"]:
                        for i in MOT.ring_used(w):
                            rx, ry = MOT.ring_xy(i)
                            probe = prim.cyl(U.INS_D / 2 + 2.0, 10.0, rx, ry, MOT.PLATE_T)
                            if (probe ^ cut_local).volume() > 1.0:
                                info["warnings"].append(f"adapter carve reaches ring insert {i}")
                    else:
                        ok = []
                        for i in range(MOT.RIM_N):
                            rx, ry = MOT.rim_xy(i)
                            probe = prim.cyl(U.INS_D / 2 + 2.0, MOT.LIP_T, rx, ry, -MOT.LIP_T)
                            if (probe ^ cut_local).volume() < 0.5:
                                ok.append(i)
                        info["puck_rim_ok"] = ok
                        for (x, y) in info.get("index_inserts", []):
                            probe = prim.cyl(U.INS_D / 2 + 2.0, 6.0, x, y, MOT.PLATE_T - 6.0)
                            if (probe ^ cut_local).volume() > 0.5:
                                info["warnings"].append(f"adapter carve reaches the SW_INDEX insert at ({x:.0f}, {y:.0f})")
                    ad_rot = U.carve_healed(ad_rot, cut_local)
        parts.append(Part("frame_motor_adapter", ad_rot, "PPA-CF", place=Mad, print_tf=U.print_flat(ad_rot),
                          color="#c9b27a",
                          notes="wiper adapter plate: slotted M6 holes (+-%.0f mm) for the motor bosses, 6 x M3 to the cheek ring; reprint when the motor is measured" % w["slot"]))
        info["M_adapter"] = Mad
        if C["v1"]:
            sp_ = fw["motor_support"]
            stay, stay_pts = MOT.can_stay(w, Mm, C["y_out"][ms], ms, mo[2] + sp_["plate_z"][1])
            if stay is not None:
                parts.append(Part("frame_can_stay", stay, "PLA", print_tf=U.print_flat(stay, tf.Rx(-90)), color="#aab4be",
                                  notes="can stay: post on the motor plate (2 x M3 x 12 into plate inserts), cable tie or "
                                        "hose clamp round the can through the cradle slots; reprint with the adapter"))
                info["can_stay_pts"] = stay_pts
        sad, sad_holes = MOT.saddle(w, Mm)
        if sad is not None:
            parts.append(Part("frame_motor_saddle", sad, "PLA", print_tf=U.print_flat(sad), color="#9aa4ae",
                              notes="motor can saddle: cable tie or hose clamp round the can through the slots, 4 x M3 into deck inserts"))
            info["deck_holes"] += sad_holes
            info["saddle"] = True

    if info.get("can_stay_pts"):
        for p in parts:
            if p.name == "frame_cheek_r" or p.name.startswith("frame_cheek_r_"):
                for (x, z) in info["can_stay_pts"]:
                    p.solid = p.solid - yfeat(U.insert_hole(0, 0, 0.0), x, z, C["y_out"][ms], -ms)
    # ---- electronics bay + AS5600 (separate helper)
    if True:
        from . import _frame_ebay as EB
        eparts, einfo = EB.build(P, C, info, parts)
        parts += eparts
        info["ebay"] = einfo
        info["deck_holes"] += einfo.get("deck_holes", [])
        a5 = einfo.get("as5600", {})
        if a5.get("cheek_inserts"):
            s5 = a5["side"]
            nm5 = "frame_cheek_r" if s5 < 0 else "frame_cheek_l"
            for p in parts:
                if p.name.startswith(nm5):
                    for (hx, hz) in a5["cheek_inserts"]:
                        p.solid = p.solid - yfeat(U.insert_hole(0, 0, 0.0), hx, hz, C["y_out"][s5], -s5)

    # ---- gear guards (v1.x)
    if C["v1"] and C["main"] in lay:
        gparts, gholes, gpins = guards_v1(P, C, info, lay[C["main"]]["origin"])
        parts += gparts
        info["deck_holes"] += gholes
        for p in parts:
            if p.name == "frame_cheek_r":
                for (x, z) in gpins:
                    p.solid = p.solid - yfeat(U.insert_hole(0, 0, 0.0), x, z, C["y_in"][ms], ms)
        info["guard_pins"] = gpins

    # ---- rover-side kit (D18): camera mount, RC832 holder, cable anchors
    if C["v1"]:
        from . import _frame_kit as KIT
        kparts, kholes = KIT.build(P, C, info)
        parts += kparts
        info["deck_holes"] += kholes

    # ---- deck: footprint = hull of everything standing on it, plus margin, as a rounded rectangle
    foot_pts = []
    for p in parts:
        if p.material == "bought" and p.name != "frame_wiper_motor":
            continue
        W = p.place
        (x0, y0, z0), (x1, y1, z1) = prim.bbox(prim.place(p.solid, W))
        if z0 < 25.0 or p.name == "frame_wiper_motor":
            foot_pts += [(x0, y0), (x1, y1)]
    for pg in info["others_foot"]:
        bx0, by0, bx1, by1 = pg.bounds
        foot_pts += [(bx0, by0), (bx1, by1)]
    lsrc = getattr(P, "LAYOUT", None)
    if isinstance(lsrc, dict) and isinstance(lsrc.get("footprint"), dict):   # the coordinator's target footprint
        fpx, fpy = lsrc["footprint"]["x"], lsrc["footprint"]["y"]
        m_ = fw["deck_margin"]
        foot_pts += [(fpx[0] + m_, fpy[0] + m_), (fpx[1] - m_, fpy[1] - m_)]
    fp = np.array([q for q in foot_pts if np.all(np.isfinite(q))])
    foot_polys = []                     # exact contact footprints: slice every part 0.5 mm above the deck top
    for p in parts:
        if p.joint is not None:
            continue
        sp_ = prim.place(p.solid, p.place)
        if prim.bbox(sp_)[0][2] > 1.0:
            continue
        for ring in sp_.slice(0.5).to_polygons():
            if len(ring) > 2:
                pg = Polygon(np.asarray(ring)).buffer(0)
                if pg.area > 1.0:
                    foot_polys.append(pg)
    m = fw["deck_margin"]
    dx0, dy0 = fp.min(0) - m
    dx1, dy1 = fp.max(0) + m
    info["deck_rect"] = (dx0, dx1, dy0, dy1)
    outline = Polygon([(dx0, dy0), (dx1, dy0), (dx1, dy1), (dx0, dy1)]).buffer(-fw["deck_corner_r"]).buffer(fw["deck_corner_r"], 16)
    seams = []
    lx, ly = dx1 - dx0, dy1 - dy0
    nx, ny = int(math.ceil(lx / 248.0)), int(math.ceil(ly / 248.0))
    def seam_c(c0, k, lo, hi, n):
        """seam near c0 as far as possible from every deck hole (dovetail tails reach 8 mm), tiles under 248."""
        best, bestd = c0, -1.0
        for d in np.arange(-60, 60.1, 0.5):
            c = c0 + d
            if c - lo > 248.0 or hi - c > 248.0 * (n - 1):
                continue
            dmin = min([abs((h[0] if k == "x" else h[1]) - c) for h in info["deck_holes"]] or [1e9])
            if dmin > bestd + 1e-9 or (abs(dmin - bestd) < 1e-9 and abs(d) < abs(best - c0)):
                best, bestd = c, dmin
        if bestd < 11.0:                       # dovetail tails 8 deep + 3 mm round the insert
            info["warnings"].append(f"deck seam {k} = {best:.1f} only {bestd:.1f} mm from a deck hole")
        return float(best)
    for i in range(1, nx):
        seams.append(dict(axis="x", c=seam_c(dx0 + i * lx / nx, "x", dx0, dx1, nx), n=3))
    for i in range(1, ny):
        seams.append(dict(axis="y", c=seam_c(dy0 + i * ly / ny, "y", dy0, dy1, ny), n=3))
    pieces = U.split_plate(outline, fw["deck_t"], seams) if seams else [outline]
    info["deck_seams"] = seams
    # rover holes: perimeter of every piece, pitch <= rover_hole_pitch, 7 mm in from the edge, away from deck inserts
    t = fw["deck_t"]
    rover = []
    for k, pc in enumerate(pieces):
        ring = LineString(pc.buffer(-7.0).exterior.coords)
        L = ring.length
        n = max(4, int(math.ceil(L / fw["rover_hole_pitch"])))
        for i in range(n):
            q = ring.interpolate(i * L / n)
            under = any(pg.buffer(4.0).contains(q) for pg in info["others_foot"] + foot_polys)
            if not under and all(math.hypot(q.x - hx, q.y - hy) > 9.0 for hx, hy in info["deck_holes"]):
                rover.append((q.x, q.y, k))
    info["rover_holes"] = [(round(x, 1), round(y, 1)) for x, y, _ in rover]
    for k in range(len(pieces)):
        if sum(1 for r in rover if r[2] == k) < 3:
            info["warnings"].append(f"deck tile {k + 1} has fewer than 3 rover holes (feet cover its edge)")
    for k, pc in enumerate(pieces):
        sol = prim.extrude(prim.shapely_cs(pc), t, z=-t)
        cuts = []
        for (hx, hy) in info["deck_holes"]:
            if pc.buffer(-3.0).contains(Point(hx, hy)):
                info.setdefault("_ins_ok", set()).add((round(hx, 2), round(hy, 2)))
                cuts.append(U.insert_hole(hx, hy, 0.0))
        for (rx, ry, kk) in rover:
            if kk == k:
                # M3 SHCS from the top, head counterbored flush
                cuts.append(U.clear_hole(rx, ry, -t - 1, 1, head_top=0.01))
        # lightening: pockets on the TOP (the underside stays flat on the rover), deck_pocket_depth deep, on a grid,
        # never under a foot, an insert, a rover bolt, a seam or the edge
        if fw["deck_pocket_depth"] > 0:
            keep = [Point(hx, hy).buffer(7.0) for hx, hy in info["deck_holes"]]
            keep += [Point(rx, ry).buffer(7.5) for rx, ry, _ in rover]
            keep += [f.buffer(3.0) for f in foot_polys]
            for sm in seams:
                keep.append(sbox(sm["c"] - 14, -1e4, sm["c"] + 14, 1e4) if sm["axis"] == "x" else sbox(-1e4, sm["c"] - 14, 1e4, sm["c"] + 14))
            free = pc.buffer(-10.0).difference(unary_union(keep))
            cell, rib = fw["deck_pocket"], 6.0
            bx0, by0, bx1, by1 = pc.bounds
            for gx in np.arange(bx0 + 10, bx1 - 10, cell + rib):
                for gy in np.arange(by0 + 10, by1 - 10, cell + rib):
                    c_ = sbox(gx, gy, gx + cell, gy + cell)
                    if free.contains(c_):
                        cs_ = prim.rrect_cs(cell, cell, 4.0).translate((gx + cell / 2, gy + cell / 2))
                        cuts.append(prim.extrude(cs_, fw["deck_pocket_depth"] + 1, z=-fw["deck_pocket_depth"]))
        sol = prim.diff(sol, cuts)
        nm = "frame_deck" if len(pieces) == 1 else f"frame_deck_{k + 1}"
        parts.append(Part(nm, sol, "PLA", print_tf=U.print_flat(sol), color="#80888f", infill=0.2,
                          notes="deck: M3 inserts for every frame foot; %d x M3 rover holes (it is its own drill template)" % sum(1 for r in rover if r[2] == k)))
    info["deck_pieces"] = len(pieces)
    for (hx, hy) in info["deck_holes"]:
        if (round(hx, 2), round(hy, 2)) not in info.get("_ins_ok", set()):
            info["warnings"].append(f"deck hole ({hx:.1f}, {hy:.1f}) gets no insert (on a seam or off the deck)")

    # deck frame -> world (z shift only)
    for p in parts:
        if p.joint is None:
            p.place = C["to_world"] @ p.place
    out = dict(parts=parts, info=info)
    _CACHE[key] = out
    return out


# ------------------------------------------------------------------------------------------------ API
def parts(P):
    return build(P)["parts"]


def joints(P):
    C = cfg(P)
    js = []

    if C["main"] in C["layout"]:
        ms = C["fw"]["motor_side"]
        mo = C["layout"][C["main"]]["origin_w"]
        js.append(Joint("frame_wiper", None, "rev", origin=mo, axis=(0, -ms, 0)))
    return js


def kin(P, S):
    # wiper_deg: + = motor forward. The motor shaft points into the frame along -motor_side * Y.
    out = {"frame_wiper": float(S.get("wiper_deg", 0.0))}

    return out


def _shaft_part(P):
    C = cfg(P)
    w = C["wiper"]
    ms = C["fw"]["motor_side"]
    # shaft in the M frame; joint frame = world axes at the main origin, joint axis -ms*Y.
    # place: joint frame -> M frame translation along the axis to the boss plane
    Mm = build(P)["info"]["M_motor"]
    mo = C["layout"][C["main"]]["origin"]
    place = tf.T(-np.asarray(mo)) @ Mm
    return Part("frame_wiper_shaft", MOT.motor_shaft(w), "bought", joint="frame_wiper", place=place, color="#8a9096",
                bought_ref="(part of the wiper motor)", notes="wiper output shaft: 10 mm with flats, M8 end thread")


_parts_plain = parts


def parts(P):  # noqa: F811  (adds the moving shaft stub to the static list)
    lst = list(_parts_plain(P))
    if cfg(P)["main"] in cfg(P)["layout"] and not any(p.name == "frame_wiper_shaft" for p in lst):
        lst.append(_shaft_part(P))

    return lst


def allowed(P):
    a = [
        Allow("frame_wiper_shaft", "frame_wiper_motor", "shaft runs in the gearbox"),
        Allow("frame_blk_*", "frame_cheek_*", "block spigot sits in the cheek hole (clearance fit, touching faces)", max_vol=1.0),
        Allow("frame_tie_*", "frame_cheek_*", "tie ends bear on the cheek inner faces", max_vol=1.0),
        Allow("frame_cheek_*", "frame_deck*", "cheek foot sits on the deck", max_vol=1.0),
        Allow("frame_motor_adapter", "frame_cheek_*", "adapter spigot in the cheek ring hole", max_vol=1.0),
        Allow("frame_motor_adapter", "frame_wiper_motor", "motor bosses bolt to the adapter face", max_vol=1.0),
        Allow("frame_motor_saddle", "frame_wiper_motor", "can rests in the saddle", max_vol=5.0),
        Allow("frame_motor_saddle", "frame_deck*", "saddle stands on the deck", max_vol=1.0),
        Allow("frame_deck*", "frame_deck*", "dovetail seam", max_vol=1.0),
        Allow("frame_cheek_*", "frame_cheek_*", "cheek dovetail seam", max_vol=1.0),
        Allow("frame_rc832", "frame_rc832_holder", "receiver in its slot", max_vol=1.0),
        Allow("frame_motor_leg", "frame_cheek_r", "leg bolted to the bridge face", max_vol=1.0),
        Allow("frame_can_stay", "frame_cheek_r", "stay bolted to the plate face", max_vol=1.0),
        Allow("frame_guard_hood", "frame_cheek_r", "hood lugs bolted to the plate face", max_vol=1.0),
        Allow("frame_can_stay", "frame_wiper_motor", "cradle round the can (envelope)", max_vol=20.0),
        Allow("frame_kiyo_x", "frame_camera_mount", "camera clip on the edge plate", max_vol=50.0),
        Allow("frame_guard_band_*", "frame_cheek_*", "band lugs bear on the cheek inner faces", max_vol=1.0),
        Allow("frame_guard_band_*", "frame_guard_band_*", "band segments butt", max_vol=1.0),
    ]
    from . import _frame_ebay as EB
    a += EB.allowed(P)

    return a


def grounded(P):
    """The deck sits on the rover: everything in the base must connect to it through touching parts."""
    return ["frame_deck*"]


def clearances(P):
    return [Clearance("frame_wiper_shaft", "frame_motor_adapter", 0.5, "shaft turns free in the adapter"),
            Clearance("frame_wiper_shaft", "frame_cheek_*", 0.5, "shaft turns free in the cheek hole")]


def checks(P, asm=None):
    B = build(P)
    info = B["info"]
    C = info["cfg"]
    fails = list(MOT.adapter_checks(C["wiper"]))
    bed = getattr(P, "BED", (256.0, 256.0, 260.0))
    for p in B["parts"]:
        if p.print_tf is not None:
            sx, sy, sz = U.bbox_size(p.solid, p.print_tf)
            if sx > bed[0] - 2 or sy > bed[1] - 2 or sz > bed[2]:
                fails.append(f"{p.name}: {sx:.0f} x {sy:.0f} x {sz:.0f} does not fit the bed")
    for p in B["parts"]:
        if p.printed and not U.stl_ok(p.solid):
            fails.append(f"{p.name}: not watertight as an STL after vertex merge")
    if info["cfg"]["v1"]:
        both = set(info.get("plate_rim_ok", [])) & set(info.get("puck_rim_ok", range(MOT.RIM_N)))
        if len(both) < 4:
            fails.append(f"only {len(both)} puck rim screws have both plate and lip material (need 4)")
    # blocks on the same cheek must not overlap
    sup = info["supports"]
    for i in range(len(sup)):
        for j in range(i + 1, len(sup)):
            a, b = sup[i], sup[j]
            if a["side"] != b["side"]:
                continue
            d = max(abs(a["origin"][0] - b["origin"][0]), abs(a["origin"][2] - b["origin"][2]))
            if d < a["block"]["half"] + b["block"]["half"] + 0.5:
                fails.append(f"bearing blocks {a['shaft']} and {b['shaft']} overlap on cheek {a['side']:+d}")
    fails += [f"layout: {w}" for w in info["warnings"]]
    return fails


def steps_v1(P):
    """Assembly steps for the BASE_SPEC v1.x frame, in the whole-base order."""
    B = build(P)
    names = [p.name for p in parts(P)]
    info = B["info"]

    def has(prefix):
        return [n for n in names if n.startswith(prefix)]

    deck = has("frame_deck")
    both = sorted(set(info.get("plate_rim_ok", range(MOT.RIM_N))) & set(info.get("puck_rim_ok", range(MOT.RIM_N))))
    nrim = len(both)
    st = [
        dict(title="Heat-set inserts",
             text="Soldering iron with an insert tip, about 220 C for PLA and about 280 C for the PPA-CF puck. Press every "
                  "M3 x 4 x 5 insert square into its 4.0 mm hole until flush: deck (%d), motor plate (2 for the leg bridge, "
                  "2 for the can stay, %d for the gear hood), puck adapter (%d in the rear lip, %d on the front face for "
                  "cad-drum's SW_INDEX block), bay (corner bosses, standoffs, clamp posts, perfboard bosses). Let each "
                  "cool before the next one nearby."
                  % (len(info["deck_holes"]), len(info.get("guard_pins", [])), MOT.RIM_N, len(info.get("index_inserts", []))),
             parts=deck + has("frame_cheek") + ["frame_motor_adapter", "frame_ebay"]),
        dict(title="Drill the rover",
             text="Lay the deck on the rover top plate where it will live (launch direction forward). Mark through the "
                  "%d perimeter holes with a 3 mm drill or a scriber (or tape down the 1:1 PDF template from "
                  "cad/parts/_frame_tests/drill_template.py), take the deck off, drill 3.4 mm. Deburr." % len(info["rover_holes"]),
             parts=deck),
        dict(title="Deck on the rover",
             text="Slide the tiles together on the dovetails (from above). M3 x 25 SHCS down through every perimeter "
                  "hole, washer and nyloc under the rover plate. Snug, not crushed. Do this before the other subsystems "
                  "put their feet down: some rover bolts sit under their feet.", parts=deck),
        dict(title="Motor leg",
             text="Stand the leg on the deck in the corner between the reel and the bay, foot along y. 4 x M3 x 10 down "
                  "through the foot into the deck inserts.", parts=has("frame_motor_leg")),
        dict(title="Motor plate",
             text="Hold the motor plate upright (front face, the side without the bridge, towards +y) with its bridge "
                  "against the top of the leg. 2 x M3 x 12 along y through the leg's long counterbores into the bridge "
                  "inserts.", parts=has("frame_cheek")),
        dict(title="Wiper motor on the puck (bench)",
             text="Battery unplugged. Put the M6 bolts through the puck slots from the front (heads sit in the slot "
                  "counterbores), into the motor bosses (threaded bosses, or a nut behind each boss ear). Slide the motor "
                  "so its shaft is centred in the bore (a 10 mm drill shank in the coupler bore helps), then tighten.",
             parts=["frame_motor_adapter", "frame_wiper_motor", "frame_wiper_shaft"]),
        dict(title="Puck into the plate",
             text="Feed the puck into the plate window from behind (the motor side) until its lip sits on the plate's rear "
                  "face. From the front, M3 x 12 through the plate into the lip inserts (%d of the 8 rim holes have both "
                  "plate and lip; use those)." % nrim,
             parts=["frame_motor_adapter"]),
    ]
    if has("frame_can_stay"):
        st.append(dict(title="Can stay",
                       text="Post against the plate's rear face, 2 x M3 x 12 into the plate inserts. Cable tie or hose clamp "
                            "round the motor can through the two cradle slots, firm but not crushing.",
                       parts=has("frame_can_stay")))
    st.append(dict(title="Shafts and trains (other subsystems)",
                   text="Now cad-drum (coupler on the wiper shaft, SW_INDEX block on the puck front face: 2 x M3 into the "
                        "puck inserts), cad-reel and cad-launch install their trains and aim housing. Follow their steps, "
                        "then come back.", parts=[]))
    if has("frame_guard_hood") or has("frame_guard_front"):
        st.append(dict(title="Gear guards",
                       text="After every train has been turned by hand: the hood goes over the gears in front of the motor "
                            "plate, M3 x 10 through each lug into the plate inserts; the front guard stands on the deck "
                            "between the drum gears and the bay, M3 x 10 into the deck inserts. Turn the wiper by hand once: "
                            "nothing rubs.", parts=has("frame_guard_hood") + has("frame_guard_front")))
    old = steps_legacy(P)
    keep = ("Electronics bay", "Set the buck", "Camera, receiver", "Wiring and lid")
    st += [x for x in old if x["title"].startswith(keep)]
    return [x for x in st if x["parts"] or "Shafts" in x["title"]]


def steps(P):
    return steps_v1(P) if cfg(P)["v1"] else steps_legacy(P)


def steps_legacy(P):
    """Frame assembly steps, in the whole-base order (other subsystems slot in where named)."""
    B = build(P)
    names = [p.name for p in parts(P)]
    info = B["info"]

    def has(prefix):
        return [n for n in names if n.startswith(prefix)]

    deck = has("frame_deck")
    st = [
        dict(title="Heat-set inserts",
             text="Soldering iron at about 220 C (PLA) with an insert tip. Press every M3 x 4 x 5 insert square into its "
                  "4.0 mm hole until flush: deck (%d), cheeks (block rings, ties, motor ring, sensor bracket), bay "
                  "corners, standoffs and clamp posts, cheek ties (both ends). Let each cool before the next one nearby."
                  % len(info["deck_holes"]), parts=deck + has("frame_cheek") + has("frame_tie") + ["frame_ebay"]),
        dict(title="Drill the rover",
             text="Lay the deck on the rover top plate where it will live (launch direction forward). Mark through the "
                  "%d perimeter holes with a 3 mm drill or a scriber (or tape down the 1:1 PDF template from cad/parts/_frame_tests/drill_template.py), take the deck off, drill 3.4 mm. Deburr. "
                  "If the deck is in tiles, slide the dovetails together first and mark them as one." % len(info["rover_holes"]),
             parts=deck),
        dict(title="Deck on the rover",
             text="Slide the tiles together on the dovetails (from above). M3 x 25 SHCS down through every perimeter "
                  "hole, washer and nyloc under the rover plate. Snug, not crushed.", parts=deck),
        dict(title="Motor-side cheek",
             text="Stand the motor-side cheek on the deck, foot outboard. M3 x 10 SHCS through the foot into the deck "
                  "inserts. The 44 mm hole with the 6-insert ring is the wiper axis.",
             parts=[n for n in has("frame_cheek_r" if info["cfg"]["fw"]["motor_side"] < 0 else "frame_cheek_l")]),
        dict(title="Motor-side bearing blocks",
             text="Push each block's spigot into its cheek hole from outside (flange outside), 4 x M3 x 8 into the "
                  "cheek inserts. The blocks on this side go on now; the shaft bearings drop into their pockets from "
                  "inside when the shafts go in.",
             parts=[sp["part"] for sp in info["supports"] if sp["side"] == info["cfg"]["fw"]["motor_side"] and sp.get("part")]),
    ]
    if "frame_motor_adapter" in names:
        st.append(dict(title="Wiper motor on its adapter (bench)",
                       text="Battery unplugged. Put the M6 bolts through the adapter slots from the cheek side (heads sit "
                            "in the slot counterbores), into the motor bosses (threaded bosses, or a nut behind each boss "
                            "ear). Slide the motor so its shaft is centred in the hub hole (a 10 mm drill shank in the "
                            "coupler bore helps), then tighten.",
                       parts=["frame_motor_adapter", "frame_wiper_motor", "frame_wiper_shaft"]))
        st.append(dict(title="Motor onto the cheek",
                       text="Before any shaft goes in: offer the motor and adapter to the motor-side cheek, spigot into "
                            "the plate counterbore (stand the motor leg on the deck first: 2 x M3 x 10 down into the deck, "
                            "2 x M3 x 16 along y through the leg into the plate bridge). From the frame side of the plate, M3 x 10 through the plate into every "
                            "insert in the adapter hub (%d of the 12 holes line up with inserts; the others stay empty)."
                            % len(MOT.ring_used(info["cfg"]["wiper"]))
                            + (" Strap the can into the saddle (cable tie or hose clamp), saddle 4 x M3 x 10 into the deck."
                               if has("frame_motor_saddle") else ""),
                       parts=["frame_motor_adapter"] + has("frame_motor_saddle") + has("frame_motor_leg")))
    st += [
        dict(title="Shafts and trains (other subsystems)",
             text="Now the drum, launch and reel subsystems put their shaft assemblies in (bearings already pressed on "
                  "the shafts), each bearing into its block pocket on this cheek. Follow their steps, then come back.",
             parts=[]),
        dict(title="Second-side bearing blocks",
             text="Slide each block over its bearing from outside, 4 x M3 x 8. Turn every shaft by hand: it must spin "
                  "free with no tight spot. If one binds, loosen its two blocks, turn the shaft, retighten.",
             parts=[sp["part"] for sp in info["supports"] if sp["side"] != info["cfg"]["fw"]["motor_side"] and sp.get("part")]),
    ]
    if len(info["cfg"]["sides"]) == 2:
        st.insert(len(st) - 1, dict(title="Outboard plate" if info["cfg"]["v1"] else "Second cheek",
             text=("Slide the outboard plate onto the far end of the launch sleeve: the d20 journal goes into the plate "
                   "bore until the sleeve shoulder meets the thrust lip, the d12 spigot pokes through the lip. Foot "
                   "outboard, M3 x 10 into the deck inserts. Turn the main shaft by hand: free, no tight spot."
                   if info["cfg"]["v1"] else
                   "Lower the other cheek over the shaft ends (each bearing enters its hole), foot outboard, M3 x 10 "
                   "into the deck. Then the cheek ties: M3 x 12 through each cheek into the tie inserts."),
             parts=[n for n in has("frame_cheek_l" if info["cfg"]["fw"]["motor_side"] < 0 else "frame_cheek_r")] + has("frame_tie")))
    if has("frame_guard_band"):
        st.append(dict(title="Guard band",
                       text="When every shaft turns free and the trains are tested by hand: slide the band segments in "
                            "between the cheeks along the cheek edges (front, top, back), M3 x 12 through each cheek into "
                            "the band lugs. The openings are where the rail and the line pass.",
                       parts=has("frame_guard_band")))
    if has("frame_ped"):
        st.append(dict(title="Pedestals",
                       text="Press the 608s into the pedestal seats (vice with soft jaws, push on the outer race only). "
                            "Thread the shaft through, stand the pedestals on the deck, M3 x 10 into the inserts.",
                       parts=has("frame_ped")))
    st += [
        dict(title="Electronics bay",
             text="Battery unplugged. Bay on the deck, 4 x M3 x 10 through the tabs. Boards in: BTS7960 on its 4 standoffs "
                  "(M3 x 6), heatsink down; ESP32 DevKitC pins down into the cradle, USB end into the wall slot, clamp "
                  "with 2 x M3 x 8; DRV8833, MP1584 and HX711 into their pockets with a cable tie through the floor slots "
                  "or a drop of hot glue. Fuse holder in its cradle with a cable tie.",
             parts=has("frame_ebay") + has("frame_pcb") + has("frame_fuse")),
        dict(title="Set the buck BEFORE wiring the DRV8833",
             text="Battery unplugged. Wire only the buck input to the fuse output and ground. Plug the battery in, "
                  "meter on the buck output, turn the trimpot until it reads 9.0 V. Unplug the battery. Only then wire "
                  "buck output to DRV8833 VIN (the DRV8833 dies above 10.8 V).",
             parts=["frame_pcb_mp1584", "frame_pcb_drv8833"]),
        dict(title="Wiring and lid",
             text="Battery unplugged. Follow firmware/base/WIRING.md. Every wire leaving the bay goes through its wall "
                  "exit and is cable-tied to the anchor beside it (strain relief). 14 AWG for battery, fuse, BTS7960 "
                  "and motor. Lid on, 4 x M3 x 6. USB cable from the ESP32 native USB port to the Jetson.",
             parts=["frame_ebay_lid"]),
    ]
    if has("frame_guard_hood") or has("frame_guard_front"):
        st.insert(len(st) - 1, dict(
            title="Gear guards",
            text="Last, after every train has been turned by hand: the hood goes over the gears behind the motor plate, "
                 "3 x M3 x 12 through its lugs into the plate inserts; the front guard stands on the deck between the "
                 "drum gears and the bay, M3 x 10 into the deck inserts. Turn the wiper by hand once: nothing rubs.",
            parts=has("frame_guard_hood") + has("frame_guard_front")))
    if has("frame_camera_mount") or has("frame_rc832_holder"):
        st.insert(len(st) - 1, dict(
            title="Camera, receiver and cable anchors",
            text="Camera column on the deck, 4 x M3 x 10 into the inserts. Clip the Kiyo X onto the 16 mm edge plate like "
                 "onto a monitor (or use a 1/4-20 tripod screw up through the plate hole); it looks forward along the "
                 "launch line. RC832: stand it antenna-end up in its holder slot, leads out of the low notch, holder 2 x "
                 "M3 x 10. Cable anchors 1 x M3 x 12 each; tie the camera USB, the receiver AV lead and the capture "
                 "dongle lead to them so no plug carries the cable weight.",
            parts=has("frame_camera_mount") + has("frame_rc832") + has("frame_kiyo_x") + has("frame_cable_anchor")))
    if "frame_as5600_bracket" in names:
        st.insert(len(st) - 1, dict(title="AS5600 board",
             text="Battery unplugged, USB unplugged. Board on the bracket pegs (chip side towards the magnet), melt each "
                  "peg tip flat with the soldering iron (heat stake). Bracket to the cheek: 2 x M3 x 10. The chip is "
                  "1 mm from the magnet face: check with a 1 mm feeler (or a folded business card). STEMMA QT cable to "
                  "the bay sensor window.",
             parts=["frame_as5600_bracket", "frame_pcb_as5600"]))
    return [x for x in st if x["parts"] or "Shafts" in x["title"]]


def guards_v1(P, C, info, mo):
    """Gear guards (v1.x): a hood roof over the gears between the motor plate and the rail, hung from the motor plate
    (lugs with M3 along y into plate inserts), and a front panel between the drum gears and the bay standing on the
    deck. Both are carved round every other subsystem's swept volume (3 mm) and keep their largest piece.
    Returns (parts, deck_holes, plate_inserts [(x, z)])."""
    fw = C["fw"]
    g = fw["guards"]
    if not g:
        return [], [], []
    ms = fw["motor_side"]
    yi = C["y_in"][ms]                                   # plate face on the frame side
    s_in = -ms                                           # direction from the plate into the frame
    mx, mz = mo[0], mo[2]
    carve = _swept_no_axis(C, (mx, mz)) + _swept_axis(C, (mx, mz))
    parts, holes, pins = [], [], []
    # hood roof
    hx0, hx1 = mx + g["hood_x"][0], mx + g["hood_x"][1]
    hy0, hy1 = sorted((yi, yi + s_in * g["hood_len"]))
    hz = mz + g["hood_z"]
    roof = prim.box(hx0, hx1, hy0, hy1, hz, hz + 2.4)
    lugs = []
    for x in np.linspace(hx0 + 12, hx1 - 12, 3):
        lug = prim.box(x - 7, x + 7, min(yi, yi + s_in * 8.0), max(yi, yi + s_in * 8.0), hz - 12.0, hz + 2.4)
        lug = lug - prim.cylY(U.CLR_D / 2, min(yi, yi + s_in * 8.0) - 1, max(yi, yi + s_in * 8.0) + 1, x, hz - 5.0) \
            - prim.cylY(U.HEAD_D / 2, (yi + s_in * 8.0 - U.HEAD_H) if s_in > 0 else (yi + s_in * 8.0 - 0.01),
                        (yi + s_in * 8.0 + 0.01) if s_in > 0 else (yi + s_in * 8.0 + U.HEAD_H), x, hz - 5.0)
        lugs.append(lug)
        pins.append((float(x), float(hz - 5.0)))
    roof = U.carve_healed(prim.union([roof] + lugs), carve)
    comps = [roof]
    if comps and comps[0].volume() > 2000:
        roof = comps[0]
        parts.append(Part("frame_guard_hood", roof, "PLA", print_tf=U.print_flat(roof), color="#d0d6dc",
                          notes="gear hood over the reel and bus gears, M3 x 10 through its lugs into the motor plate"))
        # keep only the lug pins that survived the carve
        pins = [(x, z) for (x, z) in pins if not (roof ^ prim.cylY(3.0, min(yi, yi + s_in * 8) + 0.5,
                                                                     max(yi, yi + s_in * 8) - 0.5, x, z)).is_empty()]
    else:
        pins = []
    # front panel
    fx = mx + g["front_x"]
    fy0, fy1 = sorted((yi + s_in * 2.0, yi + s_in * g["front_len"]))
    panel = prim.box(fx, fx + 2.4, fy0, fy1, 0.0, mz + g["front_top"])
    foot = prim.box(fx - 14.0, fx + 2.4, fy0, fy1, 0.0, 5.0)
    fh = [(fx - 7.0, fy0 + 12.0), (fx - 7.0, fy1 - 12.0), (fx - 7.0, (fy0 + fy1) / 2)]
    panel = panel + foot
    for x, y in fh:
        panel = panel - U.clear_hole(x, y, -1, 6)
    panel = U.carve_healed(panel, carve)
    comps = [panel]
    if comps and comps[0].volume() > 2000:
        panel = comps[0]
        keep = [(x, y) for (x, y) in fh if not (panel ^ prim.cyl(4.5, 4.0, x, y, 0.5) - prim.cyl(2.0, 6, x, y, -1)).is_empty()]
        parts.append(Part("frame_guard_front", panel, "PLA", print_tf=U.print_flat(panel, tf.Ry(90)), color="#d0d6dc",
                          notes="front gear guard between the drum gears and the bay, M3 x 10 into the deck"))
        holes += keep
    return parts, holes, pins


def _swept_axis(C, axis_xz):
    """Swept hulls that contain the main axis (shafts, sleeves, coupler, gears on the main shaft)."""
    from manifold3d import Manifold
    from shapely.geometry import MultiPoint
    from . import _frame_guard as G
    ax = Point(*axis_xz)
    hs = [Manifold.hull_points(cl) for cl in G.LAST_CLOUDS if MultiPoint(cl[:, [0, 2]]).convex_hull.buffer(-1.0).contains(ax)]
    return prim.union(hs)
