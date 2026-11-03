"""Snap pins and printed C clips (style of Harry's v10 desk launcher).

v10 reference, measured from originals/grappstar_desk_v10_handtest_stl (pin d 12):
head 18 x 2.0, groove core 10.2 (0.85 d) x 1.95 wide, 1.0 land past the groove; clip 12_clip.stl:
ID 10.35 (core + 0.15), OD 16.2, 1.8 thick (groove - 0.15), straight parallel opening 8.56 = 0.84 x core.

Frames (all axis +Z): pins have the host's near face at z = 0, the head below it (z < 0) and the shank going up
through the host to z = grip. Print pins standing on the head. Clips lie flat at z = 0..t, centred on the axis,
opening toward -Y.
"""
import math


from . import fits, prim

# v10 proportions (ratios of pin d unless named)
HEAD_D_RATIO = 1.5          # head d = 1.5 d (18 on 12)
HEAD_H_RATIO = 1.0 / 6.0    # head height = d / 6 (2.0 on 12), min HEAD_H_MIN
HEAD_H_MIN = 1.6
GROOVE_CORE_RATIO = 0.85    # groove core d = 0.85 d (10.2 on 12)
GROOVE_DEPTH_MIN = 0.5      # radial, so small pins still hold a clip
GROOVE_W_RATIO = 0.1625     # groove width = 0.1625 d (1.95 on 12), min GROOVE_W_MIN
GROOVE_W_MIN = 1.6
GROOVE_LAND_MIN = 1.0       # material past the groove
CLIP_ID_CLEAR = 0.15        # clip ID over the groove core (diametral)
CLIP_AX_CLEAR = 0.15        # groove width minus clip thickness
CLIP_OPEN_FRAC = 0.84       # straight opening width / groove core (measured v10)
CLIP_RING_RATIO = 0.244     # clip radial width / d (2.93 on 12), min CLIP_RING_MIN
CLIP_RING_MIN = 1.6
PIN_PLAY = 0.2              # axial end float between host face and clip / barb
PLA_E = 3500.0              # N/mm2
PLA_STRAIN_OK = 0.02        # one-off snap strain limit for PLA (yield about 3 %)


def groove_dims(d):
    """Groove for a printed pin of nominal d: dict core_d, width, depth, land (mm)."""
    core = min(GROOVE_CORE_RATIO * d, d - 2 * GROOVE_DEPTH_MIN)
    w = max(GROOVE_W_MIN, GROOVE_W_RATIO * d)
    return {"core_d": core, "width": w, "depth": (d - core) / 2, "land": max(GROOVE_LAND_MIN, 0.08 * d)}


def clip_groove_cutter(d, z=0.0, width=None, core_d=None):
    """Ring cutter for a clip groove on a printed shaft of diameter d. Axis +Z, groove from z to z + width.
    Subtract from the shaft."""
    g = groove_dims(d)
    w = g["width"] if width is None else width
    core = g["core_d"] if core_d is None else core_d
    return prim.tube(d / 2 + 1.0, core / 2, w, z=z)


def c_clip(d, t=None, core_d=None, ring=None, open_frac=CLIP_OPEN_FRAC, lead=0.0):
    """Printed C clip for the groove of a pin of diameter d (v10 style). Flat at z 0..t, centre on the axis,
    straight opening toward -Y of width open_frac x core_d. lead (mm) widens the opening mouth at the OD as a
    push-on lead-in (0 = v10). Returns Manifold (one shell)."""
    g = groove_dims(d)
    core = g["core_d"] if core_d is None else core_d
    t = g["width"] - CLIP_AX_CLEAR if t is None else t
    ri = (core + CLIP_ID_CLEAR) / 2
    ro = ri + (max(CLIP_RING_MIN, CLIP_RING_RATIO * d) if ring is None else ring)
    w = open_frac * core
    body = prim.tube(ro, ri, t)
    y_in = -math.sqrt(max(ri * ri - (w / 2) ** 2, 0.0))
    pts = [(-w / 2, 0.0), (w / 2, 0.0), (w / 2, y_in), (w / 2 + lead, -ro - 1), (-w / 2 - lead, -ro - 1),
           (-w / 2, y_in)]
    slot = prim.extrude(prim.poly_cs(pts), t + 2, z=-1)
    return body - slot


def clip_info(d, **kw):
    """Derived clip numbers: core_d, id, od, t, opening, snap (opening interference, mm), strain estimate.
    Strain: split ring opened by snap at the gap, eps = snap * h / (3 pi R^2) (h ring width, R mean radius,
    peak opposite the gap), a rough guide."""
    g = groove_dims(d)
    core = kw.get("core_d") or g["core_d"]
    t = kw.get("t") or g["width"] - CLIP_AX_CLEAR
    ri = (core + CLIP_ID_CLEAR) / 2
    h = kw.get("ring") or max(CLIP_RING_MIN, CLIP_RING_RATIO * d)
    w = kw.get("open_frac", CLIP_OPEN_FRAC) * core
    rm = ri + h / 2
    snap = core - w
    eps = snap * h / (3 * math.pi * rm * rm)
    return {"core_d": core, "id": 2 * ri, "od": 2 * (ri + h), "t": t, "opening": w, "snap": snap,
            "strain": eps, "strain_ok": eps <= PLA_STRAIN_OK, "groove": g}


def clip_pin(d, grip, fit=fits.HOLE_SLIDE, head_d=None, head_h=None, play=PIN_PLAY, chamfer=0.4):
    """Headed pin with a clip groove (v10 style) for a hole of diameter d through grip length grip.
    Shank d - fit. Frame: axis +Z, host near face z = 0, head z in [-head_h, 0], groove from z = grip + play
    (clip seats there), land and end chamfer past it. Print standing on the head."""
    g = groove_dims(d)
    rs = (d - fit) / 2
    rc = g["core_d"] / 2
    hd = HEAD_D_RATIO * d if head_d is None else head_d
    hh = max(HEAD_H_MIN, HEAD_H_RATIO * d) if head_h is None else head_h
    zg = grip + play
    ze = zg + g["width"] + g["land"]
    c = min(chamfer, g["land"] * 0.5)
    pts = [(0, -hh), (hd / 2 - 0.3, -hh), (hd / 2, -hh + 0.3), (hd / 2, 0), (rs, 0), (rs, zg), (rc, zg),
           (rc, zg + g["width"]), (rs, zg + g["width"]), (rs, ze - c), (rs - c, ze), (0, ze)]
    return prim.revolve_pts(pts, seg=prim.segs(hd / 2))


def clip_pin_clip_z(d, grip, play=PIN_PLAY):
    """z of the clip's lower face on clip_pin(d, grip): place c_clip(d) with prim.place(clip, tf.T(0, 0, z))."""
    return grip + play


def pin_hole_cutter(d, L, ext=1.0, chamfer=0.0):
    """Plain hole for a pin: diameter d from z = -ext to L + ext (host from z 0 to L). Optional 45 deg entry
    chamfer (mm) at z = 0."""
    h = prim.cyl(d / 2, L + 2 * ext, z=-ext)
    if chamfer > 0:
        h = h + prim.cyl(d / 2 + chamfer + ext, chamfer + ext, z=-ext, r2=d / 2)
    return h


def _snap_dims(d, L, fit, slot, barb, barb_len, play):
    s = max(1.0, 0.3 * d) if slot is None else slot
    bh = min(max(0.3, 0.08 * d), 0.45 * s) if barb is None else barb
    bl = max(1.5, 0.5 * d) if barb_len is None else barb_len
    rs = (d - fit) / 2
    rb = d / 2 + bh
    zc = L + play
    land = 0.4
    ztip = zc + land + bl
    slot_len = max(0.75 * L, 2 * d) + land + bl
    slot_len = min(slot_len, ztip + 0.0)         # never cut into the head
    return dict(s=s, bh=bh, bl=bl, rs=rs, rb=rb, zc=zc, land=land, ztip=ztip, slot_len=slot_len,
                rt=max(rs - bh, 0.5 * rs))


def snap_pin(d, L, fit=fits.HOLE_SLIDE, slot=None, barb=None, barb_len=None, head_d=None, head_h=None,
             play=PIN_PLAY):
    """Split compliant snap pin with barbs for a hole of diameter d through total grip length L.
    Head on one end (v10 proportions), shank d - fit, two prongs split by a slot of width slot (normal X), barb
    ring (radial height barb over the hole) whose flat catch face sits at z = L + play past the far face, then a
    lead-in cone to the tip. Frame: axis +Z, host near face z = 0, head z in [-head_h, 0]. Print standing on the head
    (slot vertical). Returns Manifold (one shell)."""
    k = _snap_dims(d, L, fit, slot, barb, barb_len, play)
    hd = HEAD_D_RATIO * d if head_d is None else head_d
    hh = max(HEAD_H_MIN, HEAD_H_RATIO * d) if head_h is None else head_h
    pts = [(0, -hh), (hd / 2 - 0.3, -hh), (hd / 2, -hh + 0.3), (hd / 2, 0), (k["rs"], 0), (k["rs"], k["zc"]),
           (k["rb"], k["zc"]), (k["rb"], k["zc"] + k["land"]), (k["rt"], k["ztip"]), (0, k["ztip"])]
    pin = prim.revolve_pts(pts, seg=prim.segs(hd / 2))
    z0 = k["ztip"] - k["slot_len"]
    s = k["s"]
    cut = prim.box(-s / 2, s / 2, -d, d, z0 + s / 2, k["ztip"] + 1) + prim.cylY(s / 2, -d, d, z=z0 + s / 2)
    return pin - cut


def snap_pin_info(d, L, fit=fits.HOLE_SLIDE, slot=None, barb=None, barb_len=None, play=PIN_PLAY):
    """Derived numbers: slot, barb height, barb z range (barb_z0, barb_z1), prong length, prong root strain and
    insertion force estimate (each prong a half-disc cantilever deflected by the barb height, PLA E 3.5 GPa)."""
    k = _snap_dims(d, L, fit, slot, barb, barb_len, play)
    r = k["rs"]
    Ihalf = (math.pi / 8 - 8 / (9 * math.pi)) * r ** 4
    ymax = r - 4 * r / (3 * math.pi)
    lp = k["slot_len"] - k["s"] / 2
    delta = k["bh"]
    eps = 3 * delta * ymax / lp ** 2
    f_prong = 3 * PLA_E * Ihalf * delta / lp ** 3
    return {"slot": k["s"], "barb": k["bh"], "barb_od": 2 * k["rb"], "barb_z0": k["zc"],
            "barb_z1": k["ztip"], "tip_z": k["ztip"], "prong_len": lp, "deflection": delta, "strain": eps,
            "strain_ok": eps <= PLA_STRAIN_OK, "prong_force_N": f_prong,
            "closes": 2 * delta <= k["s"]}
