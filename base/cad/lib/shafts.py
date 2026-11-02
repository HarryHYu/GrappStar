"""Printed shafts and matching bore cutters. Units mm, torque N m.

Shaft frame: axis +Z, main body from z = 0 to z = length. End steps (journals) extend below z = 0 (end_lo) and
above z = length (end_hi). A key or flat faces +X.

Bore frame (all *_bore cutters, seat convention): axis +Z, mouth at z = 0, the bore goes down to z = -length, and
the cutter overshoots both ends by `extra` (default 1.0) so it opens both faces of a host of that thickness.
To slide a shaft built at z = [0, L] into a bore of length L, translate the shaft by (0, 0, -L).

Keyed 20 mm shaft (DIN 6885 for d 17 to 22: key 6 x 6, shaft keyway 3.5 deep, hub keyway 2.8 deep). The CSK20PP
inner keyway is 6 wide x 2.8 deep. A printed shaft carries an integral key ridge instead of a loose key:
width 6 - 2 * KEY_CLEAR = 5.6, crest 2.6 above the 20 mm circle (hub depth 2.8 minus 0.2 radial clearance).
"""
import math

from . import prim, tf
from .fits import KEY_CLEAR, SHAFT_IN_BORE, HOLE_SLIDE

KEY20_W = 6.0            # nominal key width for a 20 mm shaft (DIN 6885)
KEY20_HUB_D = 2.8        # hub keyway depth from the bore (DIN 6885, CSK20PP inner keyway)
KEY_RCLEAR = 0.2         # radial clearance key crest to keyway floor
KEY20_H = KEY20_HUB_D - KEY_RCLEAR   # 2.6 key ridge height above the shaft circle
TAU_PPA_CF = 8.0         # MPa, allowable torsional shear of a printed PPA-CF shaft (conservative)
P_KEY_ALLOW = 15.0       # MPa, allowable bearing pressure on a printed key flank (conservative)
JOURNAL_608_D = 8.0      # printed journal for the 608 inner race (prints come out about +0.05; sand to fit)


def _round_profile(segments, chamfer):
    """(r, z) polygon for a stepped round shaft. segments: list of (d, z0, z1) bottom to top, contiguous."""
    pts = [(0.0, segments[0][1])]
    n = len(segments)
    for i, (d, z0, z1) in enumerate(segments):
        r = d / 2
        c = min(chamfer, r * 0.4, (z1 - z0) * 0.4)
        prev_r = segments[i - 1][0] / 2 if i > 0 else 0.0
        next_r = segments[i + 1][0] / 2 if i < n - 1 else 0.0
        if prev_r < r:       # this step is bigger than the one below: chamfer its lower edge
            pts += [(r - c, z0), (r, z0 + c)]
        else:
            pts += [(r, z0)]
        if next_r < r:
            pts += [(r, z1 - c), (r - c, z1)]
        else:
            pts += [(r, z1)]
    pts.append((0.0, segments[-1][2]))
    # drop consecutive duplicates
    out = [pts[0]]
    for p in pts[1:]:
        if abs(p[0] - out[-1][0]) > 1e-9 or abs(p[1] - out[-1][1]) > 1e-9:
            out.append(p)
    return out


def stepped_shaft(segments, chamfer=0.6):
    """Round stepped shaft from segments [(d, z0, z1), ...] bottom to top (contiguous), outer edges chamfered."""
    r_max = max(s[0] for s in segments) / 2
    return prim.revolve_pts(_round_profile(segments, chamfer), seg=prim.segs(r_max))


def _segments(length, d, end_lo, end_hi):
    lo = _steps(end_lo)                      # outward from the body, i.e. going down
    seg = []
    z = -sum(l for _, l in lo)
    for dj, lj in reversed(lo):
        seg.append((dj, z, z + lj))
        z += lj
    seg.append((d, 0.0, length))
    z = length
    for dj, lj in _steps(end_hi):
        seg.append((dj, z, z + lj))
        z += lj
    return seg


def _steps(spec):
    """end spec: (d, len) or a list of (d, len) going outward from the main body."""
    if spec is None:
        return []
    if isinstance(spec[0], (int, float)):
        return [tuple(spec)]
    return [tuple(s) for s in spec]


def keyed_shaft(length, d=20.0, key_w=KEY20_W, key_h=KEY20_H, key_z=None, end_lo=None, end_hi=None,
                chamfer=0.8, key_deg=0.0):
    """Printed shaft with an integral key ridge.

    Frame: axis +Z, main body d from z = 0 to z = length; key ridge on angle key_deg (default +X), width
    key_w - 2 * KEY_CLEAR, crest at r = d/2 + key_h, running z = key_z (default the main body less the chamfers),
    with 45 deg ends. end_lo / end_hi: (d, len) or [(d, len), ...] journal steps outward from the main body,
    e.g. end_lo=(8, 9) for a 608 journal below z = 0 (z = [-9, 0]).
    Print lying on its side with the key up, or upright for short shafts.
    """
    body = stepped_shaft(_segments(length, d, end_lo, end_hi), chamfer)
    w = key_w - 2 * KEY_CLEAR
    r_top = d / 2 + key_h
    z0, z1 = key_z if key_z is not None else (chamfer, length - chamfer)
    key = prim.box(0, r_top, -w / 2, w / 2, z0, z1)
    # 45 deg chamfered key ends: intersect with a double cone envelope
    c = key_h + 0.5
    env = prim.revolve_pts([(0, z0), (r_top - c, z0), (r_top + 0.01, z0 + c + 0.01), (r_top + 0.01, z1 - c - 0.01),
                            (r_top - c, z1), (0, z1)], seg=prim.segs(r_top))
    key = key ^ env
    return body + prim.place(key, tf.Rz(key_deg))


def keyed_bore(length, d=20.0, key_w=KEY20_W, hub_depth=KEY20_HUB_D, clear=SHAFT_IN_BORE, extra=1.0,
               chamfer=0.5, key_deg=0.0):
    """Keyed bore cutter for a printed hub on keyed_shaft (or a steel shaft with a DIN key).

    Frame: axis +Z, mouth z = 0, bore to z = -length, overshoot `extra` at both ends. Bore d + clear (default
    SHAFT_IN_BORE), keyway on angle key_deg: width key_w (the shaft ridge is key_w - 2 * KEY_CLEAR), floor at
    r = d/2 + hub_depth. 45 deg chamfer of `chamfer` on both bore edges.
    """
    R = (d + clear) / 2
    bore = prim.cyl(R, length + 2 * extra, z=-length - extra)
    kw = prim.box(0, d / 2 + hub_depth, -key_w / 2, key_w / 2, -length - extra, extra)
    cut = bore + prim.place(kw, tf.Rz(key_deg))
    if chamfer > 0:
        rc = R + chamfer
        top = prim.revolve_pts([(0, extra), (rc + extra, extra), (rc, 0), (R - 0.01, -chamfer - 0.01),
                                (0, -chamfer - 0.01)])
        bot = prim.place(top, tf.T(0, 0, -length) @ tf.Rx(180))
        cut = cut + top + bot
    return cut


def d_shaft(length, d, flat, chamfer=0.4, flat_deg=0.0):
    """D-flat shaft. Frame: axis +Z, z = [0, length], flat plane at x = d/2 - flat facing angle flat_deg."""
    s = stepped_shaft([(d, 0.0, length)], chamfer)
    cut = prim.box(d / 2 - flat, d, -d, d, -1, length + 1)
    return s - prim.place(cut, tf.Rz(flat_deg))


def d_bore(length, d, flat, clear=HOLE_SLIDE, extra=1.0, flat_deg=0.0):
    """D bore cutter for d_shaft (or a steel D shaft, e.g. a motor output). Frame: mouth z = 0, down to -length,
    overshoot extra. Diameter d + clear, flat plane at x = d/2 - flat + clear/2."""
    R = (d + clear) / 2
    b = prim.cyl(R, length + 2 * extra, z=-length - extra)
    cut = prim.box(d / 2 - flat + clear / 2, R + 1, -R - 1, R + 1, -length - extra - 1, extra + 1)
    return b - prim.place(cut, tf.Rz(flat_deg))


def journal(length, d=JOURNAL_608_D, chamfer=0.4):
    """Plain round journal, default 8 mm for a 608 inner race. Frame: axis +Z, z = [0, length]."""
    return stepped_shaft([(d, 0.0, length)], chamfer)


def journal_608(length=9.0, chamfer=0.4):
    """8 mm journal for a 608 inner race (7 wide plus 2 for a shoulder or clip). Frame: axis +Z, z = [0, length]."""
    return journal(length, JOURNAL_608_D, chamfer)


def _hex_cs(af):
    r = af / math.sqrt(3)   # circumradius; vertices on +X, flats at y = +-af/2
    return prim.poly_cs([(r * math.cos(math.radians(60 * k)), r * math.sin(math.radians(60 * k)))
                         for k in range(6)])


def hex_shaft(length, af, chamfer=0.5):
    """Hex shaft across flats af. Frame: axis +Z, z = [0, length], vertices on +-X, flats at y = +-af/2.
    Ends chamfered by intersecting with a chamfered round of the circumradius."""
    s = prim.extrude(_hex_cs(af), length)
    r = af / math.sqrt(3)
    env = stepped_shaft([(2 * r, 0.0, length)], chamfer + (r - af / 2))
    return s ^ env


def hex_bore(length, af, clear=SHAFT_IN_BORE, extra=1.0):
    """Hex bore cutter for hex_shaft: across flats af + clear. Frame: mouth z = 0, down to -length, overshoot extra."""
    return prim.extrude(_hex_cs(af + clear), length + 2 * extra, z=-length - extra)


def shaft_torque_capacity(d, tau_allow=TAU_PPA_CF, d_in=0.0):
    """Torsion capacity in N m of a round shaft of diameter d (mm), optional bore d_in, at shear tau_allow (MPa).
    T = tau * pi * (d^4 - d_in^4) / (16 d). 20 mm PPA-CF at 8 MPa: 12.6 N m. The integral key adds a little."""
    return tau_allow * math.pi * (d ** 4 - d_in ** 4) / (16.0 * d) / 1000.0


def key_torque_capacity(d, engaged_len, key_h=KEY20_H, p_allow=P_KEY_ALLOW):
    """Key flank bearing capacity in N m: p * h * L * (d/2 + h/2). h is the engaged flank height (mm)."""
    return p_allow * key_h * engaged_len * (d / 2 + key_h / 2) / 1000.0
