"""Gear geometry for the GrappStar base-station launcher (manifold3d solids, printable on a Bambu X2D).

Units mm / deg. Every solid has its axis on +Z through the origin, gear face from z=0 to z=face.
2D shapes are manifold3d CrossSection objects (or plain point lists where noted).

Conventions
- m: module, z: tooth count, pa: pressure angle (deg).
- addendum / dedendum are in multiples of m (1.0 / 1.25 standard, so 0.25 m tip clearance).
- backlash: total circumferential backlash (mm) at the pitch circle for a PAIR. Each part removes
  backlash/2 from its own tooth thickness, so two parts made with the same value mesh with `backlash`.
- Tooth index 0 of a spur gear is centred on the +X axis (angle 0). A gap (tooth space) is centred at
  angle 180/z.
- Phasing for meshing: if the mesh point is at angle `a` (deg) in a gear's own frame, rotate that gear by
  `mesh_phase(z, a)` so a tooth space faces the mate, or by `a` to put a tooth there (the mate then needs
  the space).  Typical: pinion tooth at angle 0 facing a wheel whose mesh point is at 180 -> rotate the
  wheel by mesh_phase(z_wheel, 180).
- Root fillets: concave corners at the tooth root are rounded with radius root_fillet*m (default 0.2 m),
  which stays below the deepest point reached by a standard mating tip (r - m).
"""
from __future__ import annotations

import math

import numpy as np
from manifold3d import CrossSection, JoinType, Manifold

__all__ = [
    "pitch_r", "base_r", "tip_r", "root_r", "center_distance", "worm_center_distance", "mesh_phase",
    "involute_profile_2d", "spur_2d", "spur", "sector_gear_2d", "sector_gear", "rack_2d", "rack",
    "worm_2d", "worm", "worm_lead_angle", "worm_wheel", "bevel_pair", "bevel_place_second",
    "gears_overlap_2d", "overlap_volume",
]

SEG = 128  # circular segments for discs / bores


# ------------------------------------------------------------------------------------------- basic helpers
def pitch_r(m: float, z: float) -> float:
    return m * z / 2.0


def base_r(m: float, z: float, pa: float = 20.0) -> float:
    return pitch_r(m, z) * math.cos(math.radians(pa))


def tip_r(m: float, z: float, addendum: float = 1.0) -> float:
    return pitch_r(m, z) + addendum * m


def root_r(m: float, z: float, dedendum: float = 1.25) -> float:
    return pitch_r(m, z) - dedendum * m


def center_distance(m: float, z1: float, z2: float, extra: float = 0.1) -> float:
    """Spur centre distance plus `extra` (mm, radial play on top of the tooth thinning backlash)."""
    return pitch_r(m, z1) + pitch_r(m, z2) + extra


def worm_center_distance(m: float, z_wheel: int, worm_pd: float, extra: float = 0.1) -> float:
    return pitch_r(m, z_wheel) + worm_pd / 2.0 + extra


def mesh_phase(z: int, mesh_angle: float) -> float:
    """Rotation (deg) that puts the centre of a tooth space at `mesh_angle` for a gear built by this module."""
    return mesh_angle + 180.0 / z


def _inv(a: float) -> float:
    return math.tan(a) - a


def _fillet(cs: CrossSection, rho: float) -> CrossSection:
    """Round concave corners with radius rho (morphological closing); convex corners stay sharp."""
    if rho <= 0:
        return cs
    return cs.offset(rho, JoinType.Round, circular_segments=96).offset(-rho, JoinType.Round,
                                                                         circular_segments=96)


# ------------------------------------------------------------------------------------------- tooth outline
class _Tooth:
    """Involute tooth of a (possibly virtual, non-integer zf) spur gear, centred on angle 0."""

    def __init__(self, m, zf, pa=20.0, backlash=0.15, addendum=1.0, dedendum=1.25):
        self.m, self.zf = m, zf
        self.pa = math.radians(pa)
        self.r = m * zf / 2.0
        self.rb = self.r * math.cos(self.pa)
        self.ra = self.r + addendum * m
        self.rf = self.r - dedendum * m
        self.half_pitch = math.pi / zf
        t = math.pi * m / 2.0 - backlash / 2.0          # tooth thickness at pitch circle
        self.psi = t / (2.0 * self.r)                    # half angle at pitch circle

    def theta(self, R: float) -> float:
        """Half angular thickness of the tooth at radius R (radial flank below the base circle)."""
        if R <= self.rb:
            return self.psi + _inv(self.pa)
        return self.psi + _inv(self.pa) - _inv(math.acos(self.rb / R))

    def r_for_width(self, w_min: float, r_cap: float) -> float:
        """Largest radius <= r_cap where the chordal tooth width is still >= w_min."""
        f = lambda R: 2.0 * R * math.sin(self.theta(R)) - w_min
        if f(r_cap) >= 0:
            return r_cap
        lo, hi = max(self.rb, self.rf), r_cap
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if f(mid) >= 0 else (lo, mid)
        return lo

    def points(self, kind: str = "normal", n_pts: int = 12, r_top: float | None = None):
        """Polar points (R, ang) for angles [-half_pitch, +half_pitch), CCW. kind: normal | gap | top."""
        hp = self.half_pitch
        arc = lambda R, a0, a1, step=math.radians(2.0): [
            (R, a) for a in np.linspace(a0, a1, max(2, int(abs(a1 - a0) / step) + 2))]
        if kind == "gap":
            return arc(self.rf, -hp, hp)[:-1]
        if r_top is None:
            r_top = self.r_for_width(0.05 * self.m, self.ra)
        th_root = self.theta(self.rf)
        flank = []                                       # right flank (negative angle), root -> tip
        if self.rf < self.rb and r_top > self.rb:
            flank.append((self.rf, -th_root))
            r0 = self.rb
        else:
            r0 = self.rf
        r0 = min(r0, r_top)
        # cluster samples towards the base circle where the involute curves most
        for s in np.linspace(0.0, 1.0, n_pts):
            R = r0 + (r_top - r0) * s ** 1.5
            flank.append((R, -self.theta(R)))
        th_top = self.theta(r_top)
        pts = arc(self.rf, -hp, -th_root)[:-1]
        pts += flank
        pts += arc(r_top, -th_top, th_top)[1:-1]
        pts += [(R, -a) for R, a in reversed(flank)]
        pts += arc(self.rf, th_root, hp)[1:-1]
        return pts


def _assemble(polar_teeth, rho=None) -> CrossSection:
    """polar_teeth: list of (centre_angle_rad, [(R, ang)...]); returns one closed outline."""
    pts = []
    for c, tp in polar_teeth:
        pts += [(R * math.cos(c + a), R * math.sin(c + a)) for R, a in tp]
    cs = CrossSection([np.asarray(pts, dtype=float)])
    return _fillet(cs, rho) if rho else cs


def involute_profile_2d(m, z, pa=20.0, backlash=0.15, addendum=1.0, dedendum=1.25, n_pts=12,
                        root_fillet=0.2):
    """Full spur gear outline as a CCW list of (x, y). Tooth 0 centred on +X."""
    return _cs_outline(spur_2d(m, z, pa, backlash, addendum, dedendum, n_pts, root_fillet))


def _cs_outline(cs: CrossSection):
    polys = cs.to_polygons()
    big = max(polys, key=lambda p: abs(_area(p)))
    return [tuple(map(float, p)) for p in big]


def _area(p):
    p = np.asarray(p)
    return 0.5 * float(np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1]))


def spur_2d(m, z, pa=20.0, backlash=0.15, addendum=1.0, dedendum=1.25, n_pts=12, root_fillet=0.2):
    """Spur gear outline as a CrossSection."""
    t = _Tooth(m, z, pa, backlash, addendum, dedendum)
    tp = t.points("normal", n_pts)
    return _assemble([(2 * math.pi * k / z, tp) for k in range(z)], root_fillet * m)


def _bore_cut(solid: Manifold, height: float, bore, key) -> Manifold:
    if not bore:
        return solid
    cut = Manifold.cylinder(height + 2, bore / 2.0, circular_segments=SEG).translate((0, 0, -1))
    if key:
        w, d = key
        cut = cut + Manifold.cube((bore / 2.0 + d, w, height + 2)).translate((0, -w / 2.0, -1))
    return solid - cut


def _finish(cs: CrossSection, face, bore=None, key=None, hub=None, twist=0.0, n_div=0) -> Manifold:
    g = Manifold.extrude(cs, face, n_divisions=n_div, twist_degrees=twist)
    h_tot = face
    if hub:
        d, h = hub
        g = g + Manifold.cylinder(h + 0.01, d / 2.0, circular_segments=SEG).translate((0, 0, face - 0.01))
        h_tot = face + h
    return _bore_cut(g, h_tot, bore, key)


def spur(m, z, face, bore=None, key=None, hub=None, pa=20.0, backlash=0.15, root_fillet=0.2) -> Manifold:
    """Spur gear. bore: through-hole dia; key: (width, depth beyond the bore wall) keyway on +X;
    hub: (dia, height) boss on top of the face (z = face .. face+height)."""
    return _finish(spur_2d(m, z, pa, backlash, root_fillet=root_fillet), face, bore, key, hub)


# ------------------------------------------------------------------------------------------- sector gear
def sector_gear_2d(m, z, teeth_present, first_tooth_angle=0.0, last_relief=True, tall_factor=1.30,
                   backlash=0.15, pa=20.0, addendum=1.0, dedendum=1.25, root_fillet=0.2) -> CrossSection:
    """Mutilated (sector) gear on a z-tooth pitch: teeth_present consecutive teeth, tooth i centred at
    first_tooth_angle + i*360/z (deg, laid out CCW). Missing teeth leave the bare root circle.

    Rotation sense: the teeth enter mesh in index order 0, 1, 2, ... when the gear turns CW (negative
    angle, seen from +Z). Reason: with the mate at a fixed angle, tooth 0 has the smallest angle of the
    run, so turning CW brings it to the mate first, and tooth i+1 (360/z further CCW) follows. So while
    cocking the sector gear turns CW and the gap sits at the mate when it starts / after release.

    last_relief: the last tooth (index teeth_present-1) is cut down to the pitch circle so the rack is
    let go cleanly at the pitch line, and the tooth before it (index teeth_present-2) is made tall: its
    involute flanks continue to r + addendum*tall_factor*m, clipped so its tip stays >= 0.25 m wide.
    The tall tooth carries the load deeper into the release so the rack does not slip early. Note a
    tall tooth needs root clearance on the mate (rack root at 1.25 m only gives 0.25 m spare, so the
    tall tip digs (tall_factor - 1.25)*m into a standard mate's root if tall_factor > 1.25; give the
    mating rack / gear a deeper root there, e.g. dedendum >= tall_factor*addendum + 0.1).
    """
    if not 1 <= teeth_present <= z:
        raise ValueError("teeth_present must be in 1..z")
    t = _Tooth(m, z, pa, backlash, addendum, dedendum)
    normal = t.points("normal")
    gap = t.points("gap")
    pitch_tooth = t.points("top", r_top=t.r)
    r_tall = t.r_for_width(0.25 * m, t.r + addendum * tall_factor * m)
    tall_tooth = t.points("top", r_top=r_tall)
    a0 = math.radians(first_tooth_angle)
    teeth = []
    for k in range(z):
        if k >= teeth_present:
            tp = gap
        elif last_relief and k == teeth_present - 1:
            tp = pitch_tooth
        elif last_relief and k == teeth_present - 2:
            tp = tall_tooth
        else:
            tp = normal
        teeth.append((a0 + 2 * math.pi * k / z, tp))
    return _assemble(teeth, root_fillet * m)


def sector_gear(m, z, teeth_present, face, bore=None, first_tooth_angle=0.0, last_relief=True,
                tall_factor=1.30, backlash=0.15, key=None, hub=None, pa=20.0) -> Manifold:
    cs = sector_gear_2d(m, z, teeth_present, first_tooth_angle, last_relief, tall_factor, backlash, pa)
    return _finish(cs, face, bore, key, hub)


# ------------------------------------------------------------------------------------------- rack
def rack_2d(m, n_teeth, backlash=0.15, back=6.0, pa=20.0, addendum=1.0, dedendum=1.25,
            root_fillet=0.2) -> CrossSection:
    """Straight rack: pitch line y=0, teeth point -y, tooth i centred at x=i*pi*m, root at y=+dedendum*m,
    body from y=+dedendum*m to +dedendum*m+back, x from -p/2 to (n-1)p + p/2."""
    p = math.pi * m
    tn = math.tan(math.radians(pa))
    hw0 = (p / 2.0 - backlash / 2.0) / 2.0               # half thickness on the pitch line
    y_tip, y_root = -addendum * m, dedendum * m
    w_tip = max(hw0 - addendum * m * tn, 0.02 * m)
    w_root = hw0 + dedendum * m * tn
    pts = [(-p / 2.0, y_root + back)]
    pts.append((-p / 2.0, y_root))
    for i in range(n_teeth):
        x = i * p
        pts += [(x - w_root, y_root), (x - w_tip, y_tip), (x + w_tip, y_tip), (x + w_root, y_root)]
    pts += [((n_teeth - 1) * p + p / 2.0, y_root), ((n_teeth - 1) * p + p / 2.0, y_root + back)]
    if _area(pts) < 0:
        pts = pts[::-1]  # CrossSection uses the Positive fill rule: outline must be CCW
    return _fillet(CrossSection([np.asarray(pts, dtype=float)]), root_fillet * m)


def rack(m, n_teeth, face, backlash=0.15, back=6.0) -> Manifold:
    return Manifold.extrude(rack_2d(m, n_teeth, backlash, back), face)


# ------------------------------------------------------------------------------------------- worm + wheel
def worm_lead_angle(m, pd, starts=1) -> float:
    """Lead angle (deg) at the worm pitch diameter."""
    return math.degrees(math.atan(math.pi * m * starts / (math.pi * pd)))


def worm_2d(m, pd, starts=1, backlash=0.15, pa=20.0, addendum=1.0, dedendum=1.25, n_r=24) -> CrossSection:
    """Exact transverse section (z=0 plane) of a right-hand trapezoidal-thread worm (axial rack profile,
    pa flank). Twisting it by 360 deg per lead gives the helicoid exactly (up to facet interpolation)."""
    lead = math.pi * m * starts
    rp = pd / 2.0
    rf, ra = rp - dedendum * m, rp + addendum * m
    tn = math.tan(math.radians(pa))
    hw0 = (math.pi * m / 2.0 - backlash / 2.0) / 2.0
    w = lambda R: hw0 + (rp - R) * tn                     # axial half width of the thread at R
    radii = np.linspace(rf - 0.3 * m, ra, n_r)
    cs = CrossSection.circle(rf, SEG * 2)
    for s in range(starts):
        c = 2 * math.pi * s / starts
        right = [(R, c - 2 * math.pi * w(R) / lead) for R in radii]
        a_tip = 2 * math.pi * w(ra) / lead
        tip = [(ra, c + a) for a in np.linspace(-a_tip, a_tip, 8)[1:-1]]
        left = [(R, c + 2 * math.pi * w(R) / lead) for R in radii[::-1]]
        pts = [(R * math.cos(a), R * math.sin(a)) for R, a in right + tip + left]
        cs = cs + CrossSection([np.asarray(pts)])
    return cs


def worm(m, pd, length, starts=1, bore=None, backlash=0.15, pa=20.0, key=None) -> Manifold:
    """Right-hand worm along +Z, z = 0..length. Lead L = pi*m*starts. Thread ends are cut flat.
    Thread (start 0) centreline is at angle 0 at z=0 and at angle 360*z/L at height z."""
    lead = math.pi * m * starts
    twist = 360.0 * length / lead
    g = Manifold.extrude(worm_2d(m, pd, starts, backlash, pa), length,
                         n_divisions=int(math.ceil(twist / 3.0)), twist_degrees=twist)
    return _bore_cut(g, length, bore, key)


def worm_wheel(m, z, face, worm_pd, bore=None, key=None, hub=None, starts=1, backlash=0.15, pa=20.0,
               throat=True) -> Manifold:
    """Worm wheel approximated as a helical gear (transverse module m, transverse pa) with helix angle =
    worm lead angle, right hand to suit worm(). Optional throat: subtract the torus swept by the worm root
    cylinder + 0.25 m clearance (minor radius worm_pd/2 - m) about the wheel axis, at the default
    worm_center_distance, centred on the face mid-plane. Tooth 0 is centred on +X at the face MID-PLANE
    (z = face/2); the helix is pre-rotated by half the twist to achieve that.

    Meshing with worm(): worm axis parallel to X at (y = worm_center_distance, z = face/2), wheel mesh
    point at angle 90 (wheel rotated by mesh_phase(z, 90)). The worm thread crosses its own angle
    alpha at local height L*(alpha - phi)/360 (phi = worm rotation about its own +Z, L = lead), so the
    thread passes the mesh plane x=0 when phi is chosen accordingly (see the tests). Wheel turns
    +phi*starts/z (CCW about +Z) for worm rotation +phi with this layout."""
    gamma = math.radians(worm_lead_angle(m, worm_pd, starts))
    r = pitch_r(m, z)
    twist = math.degrees(face * math.tan(gamma) / r)
    g = Manifold.extrude(spur_2d(m, z, pa, backlash).rotate(-twist / 2.0), face, n_divisions=max(4, int(abs(twist) / 0.5)),
                         twist_degrees=twist)
    if throat:
        a = worm_center_distance(m, z, worm_pd)
        rt = worm_pd / 2.0 - m
        tor = Manifold.revolve(CrossSection.circle(rt, 96).translate((a, 0)), SEG * 2)
        g = g - tor.translate((0, 0, face / 2.0))
    h_tot = face
    if hub:
        d, h = hub
        g = g + Manifold.cylinder(h + 0.01, d / 2.0, circular_segments=SEG).translate((0, 0, face - 0.01))
        h_tot += h
    return _bore_cut(g, h_tot, bore, key)


# ------------------------------------------------------------------------------------------- bevel
def _bevel_2d(m, z, delta, pa, backlash, root_fillet=0.2) -> CrossSection:
    """Back face outline from Tredgold's back-cone development: tooth shape of the virtual spur gear
    zv = z/cos(delta) mapped onto the real pitch circle (radial offsets scaled by cos(delta))."""
    zv = z / math.cos(delta)
    t = _Tooth(m, zv, pa, backlash)
    r = pitch_r(m, z)
    H = r / math.tan(delta)
    # back-cone point (r + d cos, d sin) projected along its generator (through the apex) onto z = 0
    tp = [((r + (R - t.r) * math.cos(delta)) * H / (H - (R - t.r) * math.sin(delta)), a * zv / z)
          for R, a in t.points("normal")]
    return _assemble([(2 * math.pi * k / z, tp) for k in range(z)], root_fillet * m * math.cos(delta))


def bevel_pair(m, z1, z2, face, bore1=None, bore2=None, pa=20.0, backlash=0.15):
    """Straight bevel pair, 90 deg shafts. Each gear: axis +Z, back face (outer pitch circle, radius
    m*z/2) at z=0, teeth extruded with scale_top towards the cone apex at z = apex_z (> 0).
    face is measured along the cone generatrix.

    Placing them meshing: keep gear 1 as built; gear 2 via bevel_place_second(g2, info) (rotate about Y
    by -90 deg so its axis points to -X, then translate so both apexes coincide at (0, 0, apex_z1)).
    Mesh point is at angle 0 on gear 1 and at angle 180 on gear 2 (own frames). Phase: gear 2 rotated by
    mesh_phase(z2, 180) about its own axis before placing. Ratio: gear 2 turns -a1*z1/z2 about its own
    +Z for gear 1 turning +a1 about its own +Z (bevel_place_second(g2, info, -a1*z1/z2)).
    The back face is the flat z=0 plane (not a true back cone): the profile is the back-cone tooth
    projected along its generators onto z=0, so the flanks are exact straight-bevel surfaces."""
    d1 = math.atan2(z1, z2)
    d2 = math.pi / 2.0 - d1
    A = math.hypot(pitch_r(m, z1), pitch_r(m, z2))
    if face >= A:
        raise ValueError("face must be shorter than the cone distance")
    out = []
    for z, d, bore in ((z1, d1, bore1), (z2, d2, bore2)):
        H = pitch_r(m, z) / math.tan(d)
        h = face * math.cos(d)
        s = 1.0 - h / H
        g = Manifold.extrude(_bevel_2d(m, z, d, pa, backlash), h, n_divisions=4, scale_top=(s, s))
        out.append((_bore_cut(g, h, bore, None), H, h))
    info = dict(
        pitch_cone_deg=(math.degrees(d1), math.degrees(d2)),
        cone_distance=A,
        apex_z=(out[0][1], out[1][1]),               # apex height above the back face, own frame
        mounting_distance=(-out[0][1], -out[1][1]),  # z of back-face pitch plane relative to apex
        axial_height=(out[0][2], out[1][2]),
        pitch_r=(pitch_r(m, z1), pitch_r(m, z2)),
        z=(z1, z2),
    )
    return out[0][0], out[1][0], info


def bevel_place_second(g2: Manifold, info: dict, angle2: float = 0.0, phase: bool = True) -> Manifold:
    """Rotate gear 2 by angle2 (+ mesh phase) about its own axis, then place it meshing gear 1."""
    z2 = info["z"][1]
    a = angle2 + (mesh_phase(z2, 180.0) if phase else 0.0)
    return g2.rotate((0, 0, a)).rotate((0, -90, 0)).translate((info["apex_z"][1], 0, info["apex_z"][0]))


# ------------------------------------------------------------------------------------------- checks
def gears_overlap_2d(cs_a: CrossSection, pose_a, cs_b: CrossSection, pose_b) -> float:
    """Intersection area (mm2) of two 2D shapes, each posed as (x, y, deg): rotate about the origin then
    translate."""
    a = cs_a.rotate(pose_a[2]).translate((pose_a[0], pose_a[1]))
    b = cs_b.rotate(pose_b[2]).translate((pose_b[0], pose_b[1]))
    return float((a ^ b).area())


def overlap_volume(a: Manifold, b: Manifold) -> float:
    return float((a ^ b).volume())
