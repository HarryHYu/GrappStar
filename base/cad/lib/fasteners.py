"""Fastener holes (cutters) and simple fastener models. Units mm.

Cutter frame: axis +Z, hole mouth at z = 0 (the host face the screw enters), the hole goes down into -Z, and the
cutter reaches `extra` (default 1.0) above z = 0 to open the face cleanly. Depths are measured from the mouth.

horizontal=True (hole printed with its axis lying flat): every diameter (and nut across-flats) grows by
fits.HOLE_HORIZ_EXTRA. teardrop=True adds a 45 deg teardrop point on the round holes, pointing at angle
teardrop_deg in the local XY plane (default 90 = +Y): orient the part so local +Y is up on the print bed.

Hex pockets have vertices on +-X and flats at y = +-AF/2, so a side insertion slot along +X is exactly AF wide.

Model frames: screws and bolts have the head underside at z = 0, head in +Z, shank down to z = -L. Nuts and
inserts have their top face at z = 0, body down into -Z.
"""
import math

import numpy as np
from manifold3d import CrossSection

from . import prim, tf
from .fits import (HOLE_HORIZ_EXTRA, M3_CLEAR_D, M3_CLOSE_D, M3_TAP_D, M3_INSERT_D, M3_INSERT_L, M3_HEAD_D,
                   M3_HEAD_H, M3_NUT_AF, M3_NUT_H, M3_NYLOC_H, M8_CLEAR_D, M8_NUT_AF, M8_NUT_H)

M3_CSK_D = 6.72 + 0.4    # ISO 10642 M3 countersunk head 6.72, plus clearance; 90 deg
M3_INSERT_EXTRA = 0.5    # extra depth below a heat-set insert for displaced melt
M3_INSERT_CHAMFER = 0.4  # entry chamfer on the insert hole (radial)
M8_HEAD_AF, M8_HEAD_H = 13.0, 5.3   # ISO 4017 M8 hex head
M8_NUT_AF_NOM, M8_NUT_H_NOM = 13.0, 6.5
M3_NUT_AF_NOM, M3_NUT_H_NOM, M3_NYLOC_H_NOM = 5.5, 2.4, 4.0
M3_SHCS_HEAD_D, M3_SHCS_HEAD_H = 5.5, 3.0


# ---------------------------------------------------------------- helpers

def _hx(horizontal):
    return HOLE_HORIZ_EXTRA if horizontal else 0.0


def _round_cs(d, teardrop=False, teardrop_deg=90.0):
    r = d / 2
    if not teardrop:
        return prim.circle_cs(r)
    n = prim.segs(r)
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    pts = np.c_[r * np.cos(a), r * np.sin(a)]
    t = math.radians(teardrop_deg)
    apex = np.array([[r * math.sqrt(2) * math.cos(t), r * math.sqrt(2) * math.sin(t)]])
    return CrossSection.hull_points(np.vstack([pts, apex]).tolist())


def _round(d, z0, z1, horizontal=False, teardrop=False, teardrop_deg=90.0):
    d = d + _hx(horizontal)
    return prim.extrude(_round_cs(d, horizontal and teardrop, teardrop_deg), z1 - z0, z=z0)


def _hex_cs(af):
    r = af / math.sqrt(3)
    return prim.poly_cs([(r * math.cos(math.radians(60 * k)), r * math.sin(math.radians(60 * k))) for k in range(6)])


def _hex(af, z0, z1):
    return prim.extrude(_hex_cs(af), z1 - z0, z=z0)


def round_hole(d, depth, horizontal=False, teardrop=False, teardrop_deg=90.0, extra=1.0):
    """Plain round hole cutter of diameter d (plus HOLE_HORIZ_EXTRA if horizontal), z = [-depth, extra]."""
    return _round(d, -depth, extra, horizontal, teardrop, teardrop_deg)


# ---------------------------------------------------------------- M3 cutters

def m3_clear(depth, horizontal=False, teardrop=False, teardrop_deg=90.0, extra=1.0):
    """M3 clearance hole, d M3_CLEAR_D = 3.4, z = [-depth, extra]."""
    return round_hole(M3_CLEAR_D, depth, horizontal, teardrop, teardrop_deg, extra)


def m3_close(depth, horizontal=False, teardrop=False, teardrop_deg=90.0, extra=1.0):
    """M3 close-fit hole, d M3_CLOSE_D = 3.2, z = [-depth, extra]."""
    return round_hole(M3_CLOSE_D, depth, horizontal, teardrop, teardrop_deg, extra)


def m3_tap(depth, horizontal=False, teardrop=False, teardrop_deg=90.0, extra=1.0):
    """M3 self-tapping pilot in PLA, d M3_TAP_D = 2.6, z = [-depth, extra]."""
    return round_hole(M3_TAP_D, depth, horizontal, teardrop, teardrop_deg, extra)


def m3_counterbore(depth, head_depth=M3_HEAD_H + 0.2, horizontal=False, teardrop=False, teardrop_deg=90.0,
                   extra=1.0):
    """M3 SHCS counterbored hole: head pocket d M3_HEAD_D = 6.1 from z = -head_depth up, clearance 3.4 down to
    z = -depth (depth from the mouth, so the screw grips depth - head_depth of host). Place m3_shcs at
    z = -head_depth to seat it."""
    head = _round(M3_HEAD_D, -head_depth, extra, horizontal, teardrop, teardrop_deg)
    return head + _round(M3_CLEAR_D, -depth, extra, horizontal, teardrop, teardrop_deg)


def m3_countersunk(depth, horizontal=False, teardrop=False, teardrop_deg=90.0, extra=1.0):
    """M3 countersunk (ISO 10642, 90 deg) hole: cone from d M3_CSK_D = 7.12 at z = 0 down to the 3.4 clearance,
    clearance down to z = -depth, plus a cylinder of M3_CSK_D above z = 0 up to extra."""
    hx = _hx(horizontal)
    D = M3_CSK_D + hx
    dc = M3_CLEAR_D + hx
    h = (D - dc) / 2
    cone = prim.cyl(dc / 2, h, z=-h, r2=D / 2)
    top = prim.cyl(D / 2, extra, z=0)
    return cone + top + _round(M3_CLEAR_D, -depth, extra, horizontal, teardrop, teardrop_deg)


def m3_insert(depth=M3_INSERT_L + M3_INSERT_EXTRA, chamfer=M3_INSERT_CHAMFER, screw_below=0.0, horizontal=False,
              teardrop=False, teardrop_deg=90.0, extra=1.0):
    """Heat-set insert hole for an M3x4x5 insert: d M3_INSERT_D = 4.0 straight, depth 5 + 0.5 extra (default),
    45 deg entry chamfer `chamfer` (radial) at the mouth. screw_below > 0 adds an M3 clearance hole that long
    below the insert hole (for a screw longer than the insert). Place m3_insert_model at the identity."""
    hx = _hx(horizontal)
    r = (M3_INSERT_D + hx) / 2
    c = prim.revolve_pts([(0, extra), (r + chamfer, extra), (r + chamfer, 0), (r - 0.01, -chamfer - 0.01),
                          (0, -chamfer - 0.01)])
    s = _round(M3_INSERT_D, -depth, extra, horizontal, teardrop, teardrop_deg) + c
    if screw_below > 0:
        s = s + _round(M3_CLEAR_D, -depth - screw_below, -depth + 0.5, horizontal, teardrop, teardrop_deg)
    return s


def nut_trap(af, h, nut_z=0.0, bolt_d=M3_CLEAR_D, bolt_len=10.0, slot=0.0, slot_deg=0.0, horizontal=False,
             teardrop=False, teardrop_deg=90.0, extra=1.0):
    """Generic hex nut trap cutter.

    Hex pocket across flats af (plus HOLE_HORIZ_EXTRA if horizontal), height h, top at z = -nut_z (nut_z = 0: the
    pocket opens at the mouth). Bolt clearance hole bolt_d from z = extra down to z = -bolt_len. slot > 0 adds a
    side insertion slot of width af and height h running `slot` mm from the axis along angle slot_deg (default +X)
    so the nut slides in from the side; with nut_z = 0 and slot = 0 it is a plain pocket open at the mouth.
    """
    a = af + _hx(horizontal)
    z1 = -nut_z if nut_z > 0 else extra
    pocket = _hex(a, -nut_z - h, z1)
    parts = [pocket, _round(bolt_d, -bolt_len, extra, horizontal, teardrop, teardrop_deg)]
    if slot > 0:
        sl = prim.box(0, slot, -a / 2, a / 2, -nut_z - h, z1)
        parts.append(prim.place(sl, tf.Rz(slot_deg)))
    return prim.union(parts)


def m3_nut_trap(nut_z=0.0, bolt_len=10.0, slot=0.0, slot_deg=0.0, horizontal=False, teardrop=False,
                teardrop_deg=90.0, extra=1.0):
    """M3 nut trap: hex AF M3_NUT_AF = 5.8, height M3_NUT_H = 2.7, top of pocket at z = -nut_z, M3 clearance hole
    to z = -bolt_len, optional side slot (see nut_trap). Place m3_nut at z = -nut_z - 0.15 to centre it."""
    return nut_trap(M3_NUT_AF, M3_NUT_H, nut_z, M3_CLEAR_D, bolt_len, slot, slot_deg, horizontal, teardrop,
                    teardrop_deg, extra)


def m3_nyloc_trap(nut_z=0.0, bolt_len=10.0, slot=0.0, slot_deg=0.0, horizontal=False, teardrop=False,
                  teardrop_deg=90.0, extra=1.0):
    """M3 nyloc nut trap: like m3_nut_trap with height M3_NYLOC_H = 4.3."""
    return nut_trap(M3_NUT_AF, M3_NYLOC_H, nut_z, M3_CLEAR_D, bolt_len, slot, slot_deg, horizontal, teardrop,
                    teardrop_deg, extra)


# ---------------------------------------------------------------- M8 cutters

def m8_clear(depth, horizontal=False, teardrop=False, teardrop_deg=90.0, extra=1.0):
    """M8 clearance hole, d M8_CLEAR_D = 8.5, z = [-depth, extra]."""
    return round_hole(M8_CLEAR_D, depth, horizontal, teardrop, teardrop_deg, extra)


def m8_nut_trap(nut_z=0.0, bolt_len=20.0, slot=0.0, slot_deg=0.0, horizontal=False, teardrop=False,
                teardrop_deg=90.0, extra=1.0):
    """M8 nut trap: hex AF M8_NUT_AF = 13.4, height M8_NUT_H = 6.8, M8 clearance hole to z = -bolt_len.
    Place m8_nut at z = -nut_z - 0.15."""
    return nut_trap(M8_NUT_AF, M8_NUT_H, nut_z, M8_CLEAR_D, bolt_len, slot, slot_deg, horizontal, teardrop,
                    teardrop_deg, extra)


# ---------------------------------------------------------------- models

def m3_shcs(L):
    """M3 x L socket head cap screw (ISO 4762): head d 5.5 x 3 in z = [0, 3] with a 2.5 hex socket 1.5 deep,
    shank d 3 in z = [-L, 0]."""
    head = prim.cyl(M3_SHCS_HEAD_D / 2, M3_SHCS_HEAD_H) - _hex(2.5, M3_SHCS_HEAD_H - 1.5, M3_SHCS_HEAD_H + 1)
    return head + prim.cyl(1.5, L + 0.01, z=-L)


def m3_nut(nyloc=False):
    """M3 hex nut (ISO 4032) AF 5.5 x 2.4 (nyloc: 4.0 high), hole 3.0, z = [-h, 0], vertices on +-X."""
    h = M3_NYLOC_H_NOM if nyloc else M3_NUT_H_NOM
    return _hex(M3_NUT_AF_NOM, -h, 0) - prim.cyl(1.5, h + 2, z=-h - 1)


def m3_insert_model():
    """M3x4x5 heat-set insert as installed: envelope d 3.95 (the knurl, about 4.6, melts into the hole wall),
    length 5, M3 bore 3.0, z = [-5, 0]."""
    return prim.cyl(3.95 / 2, M3_INSERT_L, z=-M3_INSERT_L) - prim.cyl(1.5, M3_INSERT_L + 2, z=-M3_INSERT_L - 1)


def m8_bolt(L):
    """M8 x L hex bolt (ISO 4017): head AF 13 x 5.3 in z = [0, 5.3], shank d 8 in z = [-L, 0]."""
    return _hex(M8_HEAD_AF, 0, M8_HEAD_H) + prim.cyl(4.0, L + 0.01, z=-L)


def m8_nut():
    """M8 hex nut (ISO 4032) AF 13 x 6.5, hole 8.0, z = [-6.5, 0], vertices on +-X."""
    return _hex(M8_NUT_AF_NOM, -M8_NUT_H_NOM, 0) - prim.cyl(4.0, M8_NUT_H_NOM + 2, z=-M8_NUT_H_NOM - 1)
