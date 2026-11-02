"""Bearing seats (cutters), bosses and bearing models. Units mm.

Seat frame (all seats): axis +Z, seat mouth at z = 0, the bore goes down into -Z. Every cutter also reaches
`extra` (default 1.0) above z = 0 so it cleanly opens the host face. Subtract the cutter from the host.

Model frame (all bearing models): axis +Z, bearing top face at z = 0, body in z = [-B, 0]. A model placed at the
identity drops straight into the matching seat (fully seated, top flush with the mouth).

Press fit with crush ribs: the bore is OD + fits.BRG_RIB_BORE (the race never touches the wall), and n half-round
ribs (radius rib_r) stand out of the wall so their crests sit fits.BRG_RIB_INTERF inside the nominal OD radius. The
ribs crush when the bearing is pressed in. A lead-in cone at the mouth starts the ribs gently.

Bearing numbers (d x D x B):
  608     8 x 22 x 7. Model races: inner race OD 12.4, outer race ID 18.6 (shields between, 0.25 recess).
  CSK20PP 20 x 47 x 14 sprag one-way bearing, 50 N m. Inner keyway 6 wide x 2.8 deep (DIN 6885 hub depth for a
          20 mm shaft). Outer keyway 6 wide x CSK20_OKEY_D deep (1.5 assumed from the catalog; measure the real
          part and edit CSK20_OKEY_D). Model races: inner OD 30, outer ID 38 (approximate).
  693ZZ   3 x 8 x 4.
  F693ZZ  3 x 8 x 4 with flange 9.5 x 0.6 (flange is part of the 4 mm width, at the top in the model).
"""
import math

from . import prim, tf
from .fits import BRG_RIB_BORE, BRG_RIB_INTERF, KEY_CLEAR, HOLE_CLEAR

# 608
B608_D, B608_OD, B608_W = 8.0, 22.0, 7.0
B608_INNER_OD = 12.4      # model inner race outer diameter
B608_OUTER_ID = 18.6      # model outer race inner diameter
B608_LIP_D = 14.0         # default seat lip bore: outer race seats on the lip, inner race free

# CSK20PP sprag bearing
CSK20_D, CSK20_OD, CSK20_W = 20.0, 47.0, 14.0
CSK20_IKEY_W, CSK20_IKEY_D = 6.0, 2.8     # inner keyway width, depth from the bore
CSK20_OKEY_W, CSK20_OKEY_D = 6.0, 1.5     # outer keyway width, depth from the OD (assumed, measure)
CSK20_INNER_OD = 30.0
CSK20_OUTER_ID = 38.0
CSK20_LIP_D = 34.0
CSK20_KEY_RCLEAR = 0.2    # radial clearance between the printed key crest and the outer keyway floor

# 693 / F693
B693_D, B693_OD, B693_W = 3.0, 8.0, 4.0
F693_FL_D, F693_FL_W = 9.5, 0.6
B693_INNER_OD = 4.6
B693_OUTER_ID = 6.6
B693_LIP_D = 5.6

SHIELD_RECESS = 0.25      # model only: shield face sits this far below the race faces


# ---------------------------------------------------------------- generic helpers

def _ring(ri, ro, z0, z1):
    return prim.cyl(ro, z1 - z0, z=z0) - prim.cyl(ri, z1 - z0 + 2, z=z0 - 1)


def _bearing_body(d, D, B, inner_od, outer_id):
    """Ring d x D x B in z = [-B, 0] with shallow shield recesses on both faces (one shell)."""
    s = SHIELD_RECESS
    pts = [(d / 2, -B), (inner_od / 2, -B), (inner_od / 2, -B + s), (outer_id / 2, -B + s), (outer_id / 2, -B),
           (D / 2, -B), (D / 2, 0), (outer_id / 2, 0), (outer_id / 2, -s), (inner_od / 2, -s), (inner_od / 2, 0),
           (d / 2, 0)]
    return prim.revolve_pts(pts, seg=prim.segs(D / 2))


def ribbed_seat(od, depth, n_ribs=8, rib_r=0.6, lead=0.6, bottom="lip", lip_d=None, below=10.0, extra=1.0,
                rib_phase_deg=None, bore_allow=BRG_RIB_BORE, interf=BRG_RIB_INTERF):
    """Generic crush-rib press-fit seat cutter for a bearing of outer diameter od (mm).

    Frame: axis +Z, mouth at z = 0, seat floor at z = -depth, cutter reaches z = +extra.
    bore = od + bore_allow; n_ribs half-round ribs (radius rib_r) with crests at od/2 - interf.
    Ribs sit at angles (k + 0.5) * 360 / n_ribs + rib_phase_deg (default phase 0, so no rib on +X).
    lead: radial size of the 45 deg lead-in cone at the mouth (it also tapers the rib tops).
    bottom: "lip"     a shoulder at z = -depth with a hole of lip_d going `below` further down (outer race seats,
                      inner race free),
            "through" the ribbed bore continues `below` further down,
            "blind"   flat floor at z = -depth with a 0.6 deep relief of lip_d so the inner race stays free.
    """
    if bottom not in ("lip", "through", "blind"):
        raise ValueError("bottom must be lip, through or blind")
    R = (od + bore_allow) / 2
    r_crest = od / 2 - interf
    rc = r_crest + rib_r
    z_bot = -depth - (below if bottom == "through" else 0.0)
    h = extra - z_bot
    bore = prim.cyl(R, h, z=z_bot)
    if n_ribs:
        ph = 0.0 if rib_phase_deg is None else rib_phase_deg
        ribs = []
        for k in range(n_ribs):
            a = math.radians((k + 0.5) * 360.0 / n_ribs + ph)
            ribs.append(prim.cyl(rib_r, h + 2, x=rc * math.cos(a), y=rc * math.sin(a), z=z_bot - 1, seg=24))
        bore = bore - prim.union(ribs)
    r_top = R + lead
    r_bot = r_crest - 0.05
    zc = -(r_top - r_bot)
    cone = prim.revolve_pts([(0, extra), (r_top, extra), (r_top, 0), (r_bot, zc), (0, zc)], seg=prim.segs(r_top))
    parts = [bore, cone]
    if bottom == "lip":
        if lip_d is None:
            raise ValueError("lip_d needed for bottom='lip'")
        parts.append(prim.cyl(lip_d / 2, below + 1.0, z=-depth - below))
    elif bottom == "blind" and lip_d:
        parts.append(prim.cyl(lip_d / 2, 0.6 + 0.5, z=-depth - 0.6))
    return prim.union(parts)


def seat_boss(od, depth, wall=3.0, floor=2.0, bore_allow=BRG_RIB_BORE):
    """Positive cylinder of material around a seat: radius (od + bore_allow) / 2 + wall, z = [-(depth + floor), 0]."""
    r = (od + bore_allow) / 2 + wall
    return prim.cyl(r, depth + floor, z=-(depth + floor))


# ---------------------------------------------------------------- 608

def seat_608(depth=B608_W, bottom="lip", lip_d=B608_LIP_D, below=10.0, n_ribs=8, rib_r=0.6, lead=0.6, extra=1.0):
    """608 (8 x 22 x 7) PLA crush-rib press-fit seat cutter.

    Frame: axis +Z, mouth z = 0, seat floor z = -depth (default 7, bearing flush). Bore 22 + BRG_RIB_BORE = 22.30,
    8 ribs r 0.6 with crests 0.15 inside the 22 mm OD, 0.6 lead-in. Default bottom "lip": 14 mm hole below the floor
    so the outer race seats and the inner race is free. See ribbed_seat for bottom, below, extra.
    """
    return ribbed_seat(B608_OD, depth, n_ribs, rib_r, lead, bottom, lip_d, below, extra)


def boss_608(depth=B608_W, wall=3.0, floor=2.0):
    """Material around a 608 seat: cylinder r 11.15 + wall, z = [-(depth + floor), 0]."""
    return seat_boss(B608_OD, depth, wall, floor)


def bearing_608():
    """608 model: ring 8 x 22 x 7, z = [-7, 0], shield recesses 0.25 between r 6.2 and r 9.3."""
    return _bearing_body(B608_D, B608_OD, B608_W, B608_INNER_OD, B608_OUTER_ID)


def bearing_608_inner_race():
    """Inner race envelope only (r 4 to 6.2, z = [-7, 0]): use it to check that a host leaves the inner race free."""
    return _ring(B608_D / 2, B608_INNER_OD / 2, -B608_W, 0.0)


# ---------------------------------------------------------------- CSK20PP

def seat_csk20(depth=CSK20_W, bottom="lip", lip_d=CSK20_LIP_D, below=10.0, key_deg=0.0, n_ribs=10, rib_r=0.6,
               lead=0.8, extra=1.0):
    """CSK20PP (20 x 47 x 14) seat cutter with crush ribs and a printed anti-rotation key.

    Frame: axis +Z, mouth z = 0, floor z = -depth (default 14). Bore 47 + BRG_RIB_BORE = 47.30, 10 ribs placed
    between keys. The host keeps a key rib on angle key_deg (default +X): width 6 - 2 * KEY_CLEAR = 5.6, crest at
    r = 23.5 - (CSK20_OKEY_D - CSK20_KEY_RCLEAR) = 22.2, running the full seat depth, that engages the 6 mm outer
    keyway of the bearing so the outer race cannot turn. Default bottom "lip" with a 34 mm hole (inner race free).
    """
    cut = ribbed_seat(CSK20_OD, depth, n_ribs, rib_r, lead, bottom, lip_d, below, extra, rib_phase_deg=key_deg)
    R = (CSK20_OD + BRG_RIB_BORE) / 2
    w = CSK20_OKEY_W - 2 * KEY_CLEAR
    r_in = CSK20_OD / 2 - (CSK20_OKEY_D - CSK20_KEY_RCLEAR)
    zb = -depth - (below if bottom == "through" else 0.0) - 0.01
    key = prim.box(r_in, R + 1.0, -w / 2, w / 2, zb - 1, 0.0)
    key = prim.place(key, tf.Rz(key_deg))
    return cut - key


def boss_csk20(depth=CSK20_W, wall=4.0, floor=2.0):
    """Material around a CSK20PP seat: cylinder r 23.65 + wall, z = [-(depth + floor), 0]."""
    return seat_boss(CSK20_OD, depth, wall, floor)


def bearing_csk20(okey_deg=0.0, ikey_deg=0.0):
    """CSK20PP model: 20 x 47 x 14, z = [-14, 0], outer keyway 6 x CSK20_OKEY_D on angle okey_deg, inner keyway
    6 x 2.8 (from the 20 mm bore) on angle ikey_deg. Real parts have both keyways; their relative angle is free."""
    b = _bearing_body(CSK20_D, CSK20_OD, CSK20_W, CSK20_INNER_OD, CSK20_OUTER_ID)
    ok = prim.box(CSK20_OD / 2 - CSK20_OKEY_D, CSK20_OD / 2 + 1, -CSK20_OKEY_W / 2, CSK20_OKEY_W / 2,
                  -CSK20_W - 1, 1)
    ik = prim.box(0, CSK20_D / 2 + CSK20_IKEY_D, -CSK20_IKEY_W / 2, CSK20_IKEY_W / 2, -CSK20_W - 1, 1)
    return b - prim.place(ok, tf.Rz(okey_deg)) - prim.place(ik, tf.Rz(ikey_deg))


def bearing_csk20_inner_race():
    """Inner race envelope (r 10 to 15, z = [-14, 0]) for free-race checks."""
    return _ring(CSK20_D / 2, CSK20_INNER_OD / 2, -CSK20_W, 0.0)


# ---------------------------------------------------------------- 693 / F693

def seat_693(depth=B693_W, bottom="lip", lip_d=B693_LIP_D, below=6.0, n_ribs=6, rib_r=0.4, lead=0.4, extra=1.0):
    """693ZZ (3 x 8 x 4) crush-rib seat cutter. Frame: axis +Z, mouth z = 0, floor z = -depth (default 4).
    Bore 8.30, 6 ribs r 0.4 crests 0.15 inside the OD, lip hole 5.6 below by default."""
    return ribbed_seat(B693_OD, depth, n_ribs, rib_r, lead, bottom, lip_d, below, extra)


def seat_f693(flange_recess=True, bottom="lip", lip_d=B693_LIP_D, below=6.0, n_ribs=6, rib_r=0.4, lead=0.3,
              extra=1.0):
    """F693ZZ (3 x 8 x 4, flange 9.5 x 0.6) seat cutter. Frame: axis +Z, mouth z = 0.

    flange_recess=True: a 9.5 + HOLE_CLEAR counterbore 0.6 + 0.05 deep takes the flange, so the bearing top is
    flush with z = 0 (place bearing_f693() at the identity); the ribbed 8 mm bore reaches z = -4.
    flange_recess=False: the flange sits on the host face; the ribbed bore is 3.4 deep (place the model at z = +0.6).
    """
    if flange_recess:
        cut = ribbed_seat(B693_OD, B693_W, n_ribs, rib_r, lead, bottom, lip_d, below, extra)
        cb = prim.cyl((F693_FL_D + HOLE_CLEAR) / 2, F693_FL_W + 0.05 + extra, z=-(F693_FL_W + 0.05))
        return cut + cb
    return ribbed_seat(B693_OD, B693_W - F693_FL_W, n_ribs, rib_r, lead, bottom, lip_d, below, extra)


def boss_693(depth=B693_W, wall=2.5, floor=1.5):
    """Material around a 693 / F693 seat: cylinder r 4.15 + wall, z = [-(depth + floor), 0]."""
    return seat_boss(B693_OD, depth, wall, floor)


def bearing_693():
    """693ZZ model: 3 x 8 x 4, z = [-4, 0]."""
    return _bearing_body(B693_D, B693_OD, B693_W, B693_INNER_OD, B693_OUTER_ID)


def bearing_f693():
    """F693ZZ model: 3 x 8 x 4 body z = [-4, 0], flange 9.5 x 0.6 at the top, z = [-0.6, 0]."""
    body = _bearing_body(B693_D, B693_OD, B693_W, B693_INNER_OD, B693_OUTER_ID)
    fl = _ring(B693_OUTER_ID / 2 + 0.3, F693_FL_D / 2, -F693_FL_W, 0.0)
    return body + fl
