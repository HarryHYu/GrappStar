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
# Placeholder shaft layout until params.LAYOUT lands (world: x forward, y left, z up, origin on the deck top under
# the wiper axis). Keys per shaft: origin, axis, d (shaft diameter), brg ('608', 'sprag', 'bush', dict(od, w)),
# cheeks (which cheeks support it: (-1, 1), (1,), (-1,), ()).
LAYOUT_DEFAULT = {
    "main":    dict(origin=(0.0, 0.0, 60.0), axis=(0, 1, 0), d=20.0, brg="bush", cheeks=(1,)),
    "drum":    dict(origin=(-58.0, 0.0, 30.0), axis=(0, 1, 0), d=8.0, brg="608", cheeks=(-1, 1)),
    "launch":  dict(origin=(62.0, 0.0, 92.0), axis=(0, 1, 0), d=20.0, brg="bush", cheeks=(-1, 1)),
    "aim":     dict(origin=(10.0, 0.0, 140.0), axis=(0, 1, 0), d=8.0, brg="608", cheeks=(-1, 1)),
    "capstan": dict(origin=(-55.0, 0.0, 105.0), axis=(0, 1, 0), d=20.0, brg="bush", cheeks=(-1, 1)),
    "reel":    dict(origin=(-100.0, 0.0, 55.0), axis=(0, 1, 0), d=8.0, brg="608", cheeks=(-1, 1)),
    "aim_worm": dict(origin=(40.0, 135.0, 40.0), axis=(0, 0, 1), d=8.0, brg="608", supports=(-15.0, 25.0)),
    "aim_wheel": dict(origin=(135.0, 0.0, 50.0), axis=(1, 0, 0), d=8.0, brg="608", span=40.0),
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
    deck_z=-60.0,           # world z of the deck top (world z = 0 is the wiper axis)
    adapter_standoff=12.0,  # adapter hub height over the cheek (clears block flanges, M6 nuts reachable)
    cheek_y_in=None,        # (right, left) inner cheek face y; None = from the motor face (y = 0) and cheek_gap
    ebay=None,              # electronics bay placement override, see _frame_ebay.py
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
    c["y_in"] = y_in
    c["y_out"] = {s_: y_in[s_] + s_ * t for s_ in (-1, 1)}
    yc = (y_in[-1] + y_in[1]) / 2
    layout = {}
    if lay_src:
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
        deck_z = -LAYOUT_DEFAULT["main"]["origin"][2]
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
    if sh["brg"] == "bush":
        od, w = sh["d"] + 2 * BUSH_WALL, fw["cheek_t"] + fw["block_t"]
    else:
        od, w, _ = U.brg_dims(P, sh["brg"])
    spig = od + 6.0
    rs = spig / 2 + U.INS_D / 2 + 2.5           # insert centre radius on the cheek
    half = rs / math.sqrt(2) + U.INS_D / 2 + 3.0
    half = max(half, spig / 2 + 2.0)
    tb = max(fw["block_t"], w - (fw["cheek_t"] - 0.5) + 3.0)
    return dict(od=od, w=w, spig=spig, rs=rs, half=half, tb=tb, hole=spig + 0.4)


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
        cuts.append(U.clear_hole(x, y, -1, b["tb"] + 1, head_top=b["tb"] + 0.01))
    return prim.diff(blk, cuts), b


def block_screws(b):
    return [(b["rs"] * math.cos(math.radians(45 + 90 * i)), b["rs"] * math.sin(math.radians(45 + 90 * i))) for i in range(4)]


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
    key = id(P)
    if key in _CACHE:
        return _CACHE[key]
    C = cfg(P)
    fw, lay, w = C["fw"], C["layout"], C["wiper"]
    tc, g = fw["cheek_t"], fw["cheek_gap"]
    ms = fw["motor_side"]
    parts, info = [], dict(cfg=C, supports=[], deck_holes=[], rover_holes=[], feet=[], warnings=[])

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
            r_ring = MOT.RING_R + U.INS_D / 2 + fw["cheek_margin"] + 2
            if s == ms:
                feats[s].append(Point(mo[0], mo[2]).buffer(r_ring, 64))
            # AS5600 bracket ring on the far cheek: the main block is already there
    xs = [p.bounds[0] for s in (-1, 1) for p in feats[s]] + [p.bounds[2] for s in (-1, 1) for p in feats[s]]
    x0c, x1c = min(xs), max(xs)
    outlines = {}
    for s in (-1, 1):
        base = sbox(x0c, 0.0, x1c, 24.0)
        poly = unary_union(feats[s] + [base]).convex_hull
        poly = poly.intersection(sbox(-1e4, 0.0, 1e4, 1e4))
        outlines[s] = poly
    info["cheek_x"] = (x0c, x1c)

    # ---- cheek solids
    cheek_world = {}
    for s in (-1, 1):
        cs = prim.shapely_cs(outlines[s])
        # extrude along local Z then Rx(90): local (X, Y, Z) -> world (X, -Z, Y); y from y_ref to y_ref - t
        y_ref = C["y_in"][s] if s < 0 else C["y_out"][s]
        sol = prim.place(prim.extrude(cs, tc), tf.T(0, y_ref, 0) @ tf.Rx(90))
        yo = C["y_out"][s]
        # foot flange (outboard) along the bottom
        fx0, fx1 = x0c + 2, x1c - 2
        foot = prim.box(fx0, fx1, yo, yo + s * fw["foot_w"], 0.0, fw["foot_t"])
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
        for xg in np.linspace(fx0 + 6, fx1 - 6, max(3, int((fx1 - fx0) / 30) + 1)):
            if all(not (a <= xg <= b) for a, b in blocked) and all(abs(xg - q) > 40 for q in gx):
                gx.append(float(xg))
        gus = []
        for xg in gx:
            tri = prim.poly_cs([(0, 0), (fw["foot_w"] - 2, 0), (0, 30)])
            gsol = prim.place(prim.extrude(tri, 4.0, z=-2.0), tf.T(xg, yo, 0) @ tf.Rz(90 if s > 0 else -90) @ tf.Rx(90))
            gus.append(gsol)
        sol = prim.union([sol, foot] + gus)
        cuts = []
        # foot screws: M3 down into deck inserts
        nf = max(2, int(math.ceil((fx1 - fx0 - 16) / fw["foot_pitch"])) + 1)
        feet = []
        for i in range(nf):
            xf = fx0 + 8 + i * (fx1 - fx0 - 16) / (nf - 1)
            if gx and min(abs(xf - q) for q in gx) < 6.0:      # dodge the gussets
                xf += 7.0 if xf < (fx0 + fx1) / 2 else -7.0
            yf = yo + s * (fw["foot_w"] / 2 + 1.0)
            feet.append((xf, yf))
            cuts.append(U.clear_hole(xf, yf, -1, fw["foot_t"] + 1, head_top=fw["foot_t"] + 0.01))
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
        info["feet"].append(dict(x0=fx0, x1=fx1, y0=min(yo, yo + s * fw["foot_w"]) - 0.5, y1=max(yo, yo + s * fw["foot_w"]) + 0.5))
        # shaft holes + block inserts
        for n, sh in supports[s]:
            b = block_geom(P, sh, fw)
            ox, _, oz = sh["origin"]
            cuts.append(prim.cylY(b["hole"] / 2, min(yo, C["y_in"][s]) - 1, max(yo, C["y_in"][s]) + 1, ox, oz))
            for (lx, ly) in block_screws(b):
                cuts.append(yfeat(U.insert_hole(0, 0, 0.0), ox + lx, oz + ly, yo, -s))
            info["supports"].append(dict(shaft=n, side=s, origin=sh["origin"], brg=sh["brg"], block=b))
        # through holes for shafts that pass this cheek without a block (main on the motor side)
        if s == ms and C["main"] in lay:
            mo = lay[C["main"]]["origin"]
            cuts.append(prim.cylY(MOT.CHEEK_HOLE_D / 2, min(yo, C["y_in"][s]) - 1, max(yo, C["y_in"][s]) + 1, mo[0], mo[2]))
            for i in range(MOT.RING_N):
                rx, rz = MOT.ring_xy(i)
                cuts.append(yfeat(U.insert_hole(0, 0, 0.0), mo[0] + rx, mo[2] + rz, yo, -s))
        cheek_world[s] = prim.diff(sol, cuts)

    # ---- ties between the cheeks (auto: near the top of the overlap of both outlines, clear of every block)
    ties = fw["ties"]
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
    if fw["band"]:
        from . import _frame_guard as G
        both = outlines[-1].intersection(outlines[1])
        cut = G.swept_cutters(P, C) if fw["band_cut"] else None
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

    for s in (-1, 1):
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
        n, s = sp["shaft"], sp["side"]
        sh = lay[n]
        sol, b = block_solid(P, sh, fw)
        ox, _, oz = sh["origin"]
        # block local +Z = outward = s * Y
        M = tf.T(ox, C["y_out"][s], oz) @ (tf.Rx(-90) if s > 0 else tf.Rx(90))
        mat = "PPA-CF" if n in ("capstan", "launch", "main") else "PLA"
        nm = f"frame_blk_{n}_{'l' if s > 0 else 'r'}"
        parts.append(Part(nm, prim.place(sol, M), mat, print_tf=None, color="#c9b27a" if mat == "PPA-CF" else "#b8c2cc",
                          notes=f"bearing block, {n} shaft ({sh['brg']}), 4 x M3 x 16 into cheek inserts"))
        p = parts[-1]
        p.print_tf = U.print_flat(p.solid, tf.Rx(90) if s > 0 else tf.Rx(-90))  # flange face down
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
        Mm = tf.frame((mo[0], yo + ms * MOT.PLATE_T, mo[2]), zdir, xh) @ tf.Rz(w["rot_deg"])
        info["M_motor"] = Mm
        body = MOT.motor_body(w)
        parts.append(Part("frame_wiper_motor", body, "bought", place=Mm, color="#3c4248",
                          bought_ref="12 V car wiper motor (self-locking worm), 2-wire rebuild",
                          notes="wiper motor ENVELOPE (assumed, not measured): edit params.WIPER and reprint the adapter"))
        ad = MOT.adapter_plate(w, standoff)
        Mad = tf.frame((mo[0], yo + ms * MOT.PLATE_T, mo[2]), zdir, xh)    # the ring is fixed; rot only moves the slots
        ad_rot = prim.place(ad, tf.Rz(w["rot_deg"]))
        parts.append(Part("frame_motor_adapter", ad_rot, "PPA-CF", place=Mad, print_tf=U.print_flat(ad_rot),
                          color="#c9b27a",
                          notes="wiper adapter plate: slotted M6 holes (+-%.0f mm) for the motor bosses, 6 x M3 to the cheek ring; reprint when the motor is measured" % w["slot"]))
        info["M_adapter"] = Mad
        sad, sad_holes = MOT.saddle(w, Mm)
        if sad is not None:
            parts.append(Part("frame_motor_saddle", sad, "PLA", print_tf=U.print_flat(sad), color="#9aa4ae",
                              notes="motor can saddle: cable tie or hose clamp round the can through the slots, 4 x M3 into deck inserts"))
            info["deck_holes"] += sad_holes
            info["saddle"] = True

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

    # ---- deck: footprint = hull of everything standing on it, plus margin, as a rounded rectangle
    foot_pts = []
    for p in parts:
        if p.material == "bought" and p.name != "frame_wiper_motor":
            continue
        W = p.place
        (x0, y0, z0), (x1, y1, z1) = prim.bbox(prim.place(p.solid, W))
        if z0 < 25.0 or p.name == "frame_wiper_motor":
            foot_pts += [(x0, y0), (x1, y1)]
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
    for i in range(1, nx):
        seams.append(dict(axis="x", c=dx0 + i * lx / nx, n=3))
    for i in range(1, ny):
        seams.append(dict(axis="y", c=dy0 + i * ly / ny, n=3))
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
            if all(math.hypot(q.x - hx, q.y - hy) > 9.0 for hx, hy in info["deck_holes"]):
                rover.append((q.x, q.y, k))
    info["rover_holes"] = [(round(x, 1), round(y, 1)) for x, y, _ in rover]
    for k, pc in enumerate(pieces):
        sol = prim.extrude(prim.shapely_cs(pc), t, z=-t)
        cuts = []
        for (hx, hy) in info["deck_holes"]:
            if pc.buffer(-3.0).contains(Point(hx, hy)):
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
    return {"frame_wiper": float(S.get("wiper_deg", 0.0))}


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
        Allow("frame_guard_band_*", "frame_cheek_*", "band lugs bear on the cheek inner faces", max_vol=1.0),
        Allow("frame_guard_band_*", "frame_guard_band_*", "band segments butt", max_vol=1.0),
    ]
    from . import _frame_ebay as EB
    a += EB.allowed(P)
    return a


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


def steps(P):
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
                  "%d perimeter holes with a 3 mm drill or a scriber, take the deck off, drill 3.4 mm. Deburr. "
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
             text="Push each block's spigot into its cheek hole from outside (flange outside), 4 x M3 x 16 into the "
                  "cheek inserts. The blocks on this side go on now; the shaft bearings drop into their pockets from "
                  "inside when the shafts go in.",
             parts=[sp["part"] for sp in info["supports"] if sp["side"] == info["cfg"]["fw"]["motor_side"]]),
        dict(title="Shafts and trains (other subsystems)",
             text="Now the drum, launch and reel subsystems put their shaft assemblies in (bearings already pressed on "
                  "the shafts), each bearing into its block pocket on this cheek. Follow their steps, then come back.",
             parts=[]),
        dict(title="Second cheek",
             text="Lower the other cheek over the shaft ends (each bearing enters its hole), foot outboard, M3 x 10 "
                  "into the deck. Then the cheek ties: M3 x 12 through each cheek into the tie inserts.",
             parts=[n for n in has("frame_cheek_l" if info["cfg"]["fw"]["motor_side"] < 0 else "frame_cheek_r")] + has("frame_tie")),
        dict(title="Second-side bearing blocks",
             text="Slide each block over its bearing from outside, 4 x M3 x 16. Turn every shaft by hand: it must spin "
                  "free with no tight spot. If one binds, loosen its two blocks, turn the shaft, retighten.",
             parts=[sp["part"] for sp in info["supports"] if sp["side"] != info["cfg"]["fw"]["motor_side"]]),
    ]
    if has("frame_ped"):
        st.append(dict(title="Pedestals",
                       text="Press the 608s into the pedestal seats (vice with soft jaws, push on the outer race only). "
                            "Thread the shaft through, stand the pedestals on the deck, M3 x 10 into the inserts.",
                       parts=has("frame_ped")))
    if "frame_motor_adapter" in names:
        st.append(dict(title="Wiper motor",
                       text="Battery unplugged. Drop an M6 nut into each slot channel on the cheek side of the adapter. "
                            "Bolt the adapter to the cheek ring: 6 x M3 x %d from the motor face into the ring inserts. "
                            "Offer the motor up (its shaft through the hub hole into the coupler from cad-drum), M6 bolts "
                            "through the bosses into the nuts, slide each to centre the shaft, tighten. Strap the can "
                            "into the saddle (cable tie or hose clamp), saddle 4 x M3 x 10 into the deck."
                            % int(round(MOT.PLATE_T + info.get("adapter_standoff", 0) - U.HEAD_H + 5)),
                       parts=["frame_motor_adapter", "frame_wiper_motor", "frame_wiper_shaft"] + has("frame_motor_saddle")))
    st += [
        dict(title="Electronics bay",
             text="Battery unplugged. Bay on the deck, 4 x M3 x 10 through the tabs. Boards in: BTS7960 on its 4 standoffs "
                  "(M3 x 6), heatsink down; ESP32 DevKitC pins down into the cradle, USB end into the wall slot, clamp "
                  "with 2 x M3 x 10; DRV8833, MP1584 and HX711 into their pockets with a cable tie through the floor slots "
                  "or a drop of hot glue. Fuse holder in its cradle with a cable tie.",
             parts=has("frame_ebay") + has("frame_pcb") + has("frame_fuse")),
        dict(title="Set the buck BEFORE wiring the DRV8833",
             text="Battery unplugged. Wire only the buck input to the fuse output and ground. Plug the battery in, "
                  "meter on the buck output, turn the trimpot until it reads 9.0 V. Unplug the battery. Only then wire "
                  "buck output to DRV8833 VIN (the DRV8833 dies above 10.8 V).",
             parts=["frame_pcb_mp1584", "frame_pcb_drv8833"]),
        dict(title="AS5600 board",
             text="Battery unplugged, USB unplugged. Board on the bracket pegs (chip side towards the magnet), melt each "
                  "peg tip flat with the soldering iron (heat stake). Bracket to the cheek: 2 x M3 x 10. The chip is "
                  "1 mm from the magnet face: check with a 1 mm feeler (or a folded business card). STEMMA QT cable to "
                  "the bay sensor window.",
             parts=["frame_as5600_bracket", "frame_pcb_as5600"]),
        dict(title="Wiring and lid",
             text="Battery unplugged. Follow firmware/base/WIRING.md. Every wire leaving the bay goes through its wall "
                  "exit and is cable-tied to the anchor beside it (strain relief). 14 AWG for battery, fuse, BTS7960 "
                  "and motor. Lid on, 4 x M3 x 8. USB cable from the ESP32 native USB port to the Jetson.",
             parts=["frame_ebay_lid"]),
    ]
    return st
