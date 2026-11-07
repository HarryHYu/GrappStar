"""Prototype drill bits for bench and soil tests (KEDGE dart), plus a hand grip for the drill gearmotor.

    .venv/bin/python proto/drill_proto.py [OUTDIR]        (default proto/out)

Each bit screws straight onto the drill gearmotor's M3 threaded shaft (2.84 x 56 past the face), which runs up the middle
as a steel spine. The bit is tapered to a sharp point, with an anchor (buttress) flight: the face that takes the pull-out load
(toward the motor) is 45 deg so it prints without supports, tip up; the leading face (toward the tip) is 30 deg so it cuts in.
Lock in both directions: an M3 nut pressed into the hex pocket in the base, then a second M3 nut on the shaft jammed up
against the base. Right-hand flight: turning clockwise seen from the motor drives it in (reverse the motor leads if not).
"""
import math
import os
import sys

import numpy as np
import manifold3d as m3
import trimesh

SEG = 72              # facets around
BORE_D = 2.6          # self-taps the M3 shaft into PLA
SPINE = 54.0          # bore depth (the shaft is 56 past the face; the jam nut takes 2.4 of it)
HEX_AF, HEX_H = 5.65, 2.6


def frustum(z0, z1, r0, r1):
    return m3.Manifold.cylinder(z1 - z0, r0, r1, SEG).translate([0, 0, z0])


def flight(z0, z1, pitch, r_core, r_out, lead_deg=30.0, load_deg=45.0, land=0.6, lefthand=False, steps_per_turn=96):
    """Helical buttress flight between z0 and z1 (crest centre line). r_core(z), r_out(z) are callables. Built as a swept
    quad profile (r, z) along the helix; depth fades in over the first and out over the last quarter turn."""
    turns = (z1 - z0) / pitch
    n = max(8, int(turns * steps_per_turn))
    rings = []
    for i in range(n + 1):
        t = i / n
        th = 2 * math.pi * turns * t * (-1 if lefthand else 1)
        zc = z0 + (z1 - z0) * t
        rc, ro = r_core(zc) - 0.4, r_out(zc)
        fade = min(1.0, (turns * t) / 0.25, (turns * (1 - t)) / 0.25)
        ro = rc + 0.3 + max(0.0, (ro - rc - 0.3)) * fade
        d = ro - rc
        za = zc - land / 2 - d * math.tan(math.radians(load_deg))   # root, motor side (load face)
        zb = zc + land / 2 + d * math.tan(math.radians(lead_deg))   # root, tip side (leading face)
        prof = [(rc, za), (ro, zc - land / 2), (ro, zc + land / 2), (rc, zb)]
        c, s = math.cos(th), math.sin(th)
        rings.append([(r * c, r * s, z) for r, z in prof])
    V = np.array([p for ring in rings for p in ring], dtype=np.float64)
    F = []
    for i in range(n):
        a, b = 4 * i, 4 * (i + 1)
        for k in range(4):
            k2 = (k + 1) % 4
            F += [[a + k, b + k, b + k2], [a + k, b + k2, a + k2]]
    F += [[0, 2, 1], [0, 3, 2]]
    e = 4 * n
    F += [[e, e + 1, e + 2], [e, e + 2, e + 3]]
    tm = trimesh.Trimesh(V, np.array(F), process=True)
    trimesh.repair.fix_winding(tm)
    trimesh.repair.fix_normals(tm)
    if tm.volume < 0:
        tm.invert()
    assert tm.is_watertight and tm.is_winding_consistent, "flight mesh not closed"
    mesh = m3.Mesh(vert_properties=np.asarray(tm.vertices, np.float32), tri_verts=np.asarray(tm.faces, np.uint32))
    return m3.Manifold(mesh)


def bit(name, od, core, pitch, length=72.0, taper=26.0, hub_d=11.0, hub_h=4.0, lefthand=False):
    """A tapered anchor bit. od: flight diameter; core: core diameter over the spine; taper: length of the pointed end."""
    zt = length - taper                       # flights and core start to taper here
    rc0, ro0 = core / 2, od / 2
    tip_r = 0.5

    def r_core(z):
        if z <= zt:
            return rc0
        return rc0 + (tip_r - rc0) * min(1.0, (z - zt) / taper)

    def r_out(z):
        if z <= zt:
            return ro0
        return ro0 + (r_core(length) + 0.6 - ro0) * min(1.0, (z - zt) / (taper * 0.95))

    body = frustum(0, hub_h, hub_d / 2, hub_d / 2)
    body += frustum(hub_h, hub_h + 3, hub_d / 2, rc0)                    # 45-ish shoulder from the hub to the core
    body += frustum(hub_h + 2.9, zt, rc0, rc0)
    body += frustum(zt - 0.1, length, rc0, tip_r)
    body += flight(hub_h + 3 + pitch * 0.15, length - 2.5, pitch, r_core, r_out, lefthand=lefthand)
    body -= frustum(-1, SPINE, BORE_D / 2, BORE_D / 2)
    hexp = m3.CrossSection.circle(HEX_AF / math.sqrt(3), 6).extrude(HEX_H + 1).translate([0, 0, -1])
    body -= hexp
    st = body.status()
    assert str(st).endswith("NoError"), (name, st)
    return name, body


def grip():
    """Hand grip for the drill gearmotor: pocket 12.3 x 10.3 x 25 for the gearbox and can (front face flush), wire slot
    at the back, T-bar handle. Prints standing on the T-bar, no supports."""
    blk = m3.Manifold.cube([22, 20, 34], True).translate([0, 0, 17])
    blk += m3.Manifold.cube([110, 16, 12], True).translate([0, 0, 40])          # T-bar
    blk -= m3.Manifold.cube([12.3, 10.3, 25.5], True).translate([0, 0, 34 - 12.75 + 0.01]).rotate([180, 0, 0]).translate([0, 0, 34 + 34 - 25.5 + 0.0])
    return "drill_grip", blk


def grip2():
    """Same grip, built directly (pocket open at z 0 = the drilling end)."""
    body = m3.Manifold.cube([22, 20, 32], True).translate([0, 0, 16])
    body += m3.Manifold.cube([110, 16, 12], True).translate([0, 0, 38])
    body -= m3.Manifold.cube([12.3, 10.3, 25.0], True).translate([0, 0, 12.5 - 0.01])
    body -= m3.Manifold.cube([4.0, 3.0, 40], True).translate([0, 0, 20])           # wire channel to the top
    body -= m3.Manifold.cube([4.0, 30, 3.0], True).translate([0, 0, 42.6])          # wire exit along the T-bar
    return "drill_grip", body


def export(name, man, out):
    mesh = man.to_mesh()
    tm = trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=True)
    assert tm.is_watertight, name
    path = os.path.join(out, name + ".stl")
    tm.export(path)
    return dict(name=name, volume_cm3=round(tm.volume / 1000, 2), bbox_mm=[round(float(x), 1) for x in tm.extents],
                watertight=bool(tm.is_watertight), bodies=len(tm.split(only_watertight=False)))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "out")
    os.makedirs(out, exist_ok=True)
    parts = [
        bit("bit_T16_RH", od=16, core=7.0, pitch=12),                     # main candidate
        bit("bit_T16_LH", od=16, core=7.0, pitch=12, lefthand=True),      # its mirror (the other pod)
        bit("bit_T12_RH", od=12, core=6.8, pitch=9),                      # slim: least torque
        bit("bit_T20_RH", od=20, core=7.5, pitch=15),                     # wide: most holding
        grip2(),
    ]
    rep = [export(n, m, out) for n, m in parts]
    for r in rep:
        print(r)


if __name__ == "__main__":
    main()
