"""Whole-base checks for two cross-owner interfaces the collision audit cannot see (a short engagement or a line
through a part leaves a gap or no part at all):
- cad_power-1 / D48: cad-drum's coupler hub on cad-frame's wiper shaft, both posed from the assembly: at least 12 mm
  of the hub's D bore on the shaft flats and the whole M8 nut on the thread, in several wiper angles.
- cad_support-1: no frame part sits across the tether (the 1 mm line grown by 1 mm) in any timeline phase.
Loads the whole assembly once (about 2 minutes: the frame build loads every subsystem for its carves)."""
import numpy as np
import pytest
from manifold3d import Manifold
from shapely.geometry import Point, Polygon

from cad.lib import prim, tf, line as LN
from tools.assembly import Assembly
from tools.params import load_params
from tools.timeline import timeline

NUT_H = 6.5            # M8 nut height (DIN 934)
NUT_AF = 13.0          # M8 nut across flats: the pocket must be wider than this to take the nut
MIN_FLATS = 12.0       # mm of D bore on the flats (torque path for the 5 N m shear pin and the 15 N m stall)
STEP = 0.25            # section spacing along the main axis


@pytest.fixture(scope="module")
def asm():
    return Assembly.load(load_params())


def _bore(cs):
    """Bounds (x0, y0, x1, y1) of the innermost ring round the axis (the bore), or None when the section is solid there."""
    rings = [Polygon(np.asarray(r)) for r in cs.to_polygons() if len(r) > 2]
    inner = [g for g in rings if g.buffer(0).contains(Point(0.0, 0.0))]
    if len(inner) < 2:                      # only the outer ring: no bore at this plane
        return None
    return min(inner, key=lambda g: abs(g.area)).bounds


def _hub_profile(asm, wiper_deg):
    """Per plane z (hub frame: z along the main axis, y normal to the D bore flats): the shaft section bounds and the
    bore bounds, both parts posed by the assembly with the wiper at wiper_deg."""
    W = asm.world(dict(wiper_deg=wiper_deg))
    Mh = W["drum_coupler_hub"]
    hub = asm.parts["drum_coupler_hub"].solid
    shaft = prim.place(asm.parts["frame_wiper_shaft"].solid, tf.inv(Mh) @ W["frame_wiper_shaft"])
    (_, _, z0), (_, _, z1) = prim.bbox(hub)
    rows = []
    for z in np.arange(z0 + STEP / 2, z1, STEP):
        s = shaft.slice(z)
        sb = s.bounds() if s.area() > 1e-6 else None
        rows.append((float(z), sb, _bore(hub.slice(z))))
    return rows


@pytest.mark.parametrize("wiper_deg", [0.0, 37.0, 200.0])
def test_coupler_hub_on_wiper_flats_and_nut_on_thread(asm, wiper_deg):
    P = asm.P
    W = getattr(P, "WIPER", {}) or {}
    d = float(W.get("shaft_d", 10.0))
    th = 8.0                                    # M8 thread major diameter
    rows = _hub_profile(asm, wiper_deg)

    def w(b):
        return (b[2] - b[0], b[3] - b[1])       # (x extent, y extent)

    flats = 0.0
    thread = []
    seat = None
    for z, sb, bb in rows:
        if sb is not None:
            sx, sy = w(sb)
            if sy < d - 1.0 and sx > (d + th) / 2 and bb is not None:
                bx, by = w(bb)
                if by < d - 1.0 and bx > (d + th) / 2:      # D bore flats on the shaft flats
                    flats += STEP
            if abs(sx - th) < 0.5 and abs(sy - th) < 0.5:
                thread.append(z)
        if seat is None and bb is not None and min(w(bb)) > NUT_AF:
            seat = z - STEP / 2                              # floor of the nut pocket
    assert flats >= MIN_FLATS, f"only {flats:.2f} mm of the hub's D bore on the wiper flats (need {MIN_FLATS})"
    assert seat is not None, "no nut pocket in the hub"
    assert thread, "no M8 thread inside the hub"
    t0, t1 = min(thread) - STEP / 2, max(thread) + STEP / 2
    assert t0 <= seat + 1e-6 and seat + NUT_H <= t1 + 1e-6, \
        f"M8 nut at {seat:.2f}..{seat + NUT_H:.2f} is not all on the thread ({t0:.2f}..{t1:.2f})"
    pocket = [min(w(bb)) > NUT_AF for z, sb, bb in rows if seat < z < seat + NUT_H and bb is not None]
    assert pocket and all(pocket), "the nut does not fit its pocket"


def test_coupler_hub_clear_of_motor_puck_and_shaft(asm):
    for wdeg in (0.0, 37.0, 200.0):
        W = asm.world(dict(wiper_deg=wdeg))
        hub = asm.posed("drum_coupler_hub", {}, W)
        for n in ("frame_wiper_shaft", "frame_wiper_motor", "frame_motor_adapter"):
            v = (hub ^ asm.posed(n, {}, W)).volume()
            assert v < 1e-3, (n, wdeg, v)


def test_tether_clear_of_every_frame_part(asm):
    P = asm.P
    states = timeline(P, 0.5) + asm.extra_states
    r = LN.LINE_D / 2 + 1.0                     # the 1 mm line with 1 mm to spare
    sph = Manifold.sphere(r, 12)
    names = [n for n, p in asm.parts.items() if p.subsystem == "frame"]
    moving = [n for n in names if asm.parts[n].joint is not None]
    W0 = asm.world(states[0])
    fixed = {n: asm.posed(n, {}, W0) for n in names if n not in moving}
    boxes = {n: tuple(map(np.asarray, prim.bbox(s))) for n, s in fixed.items()}
    seen, hits, phases = {}, [], set()
    for S in states:
        L = np.asarray(asm.line(S), float)
        assert len(L) > 2, S.get("phase")
        phases.add(S.get("phase"))
        solids = dict(fixed)
        bx = dict(boxes)
        if moving:
            W = asm.world(S)
            for n in moving:
                solids[n] = asm.posed(n, S, W)
                bx[n] = tuple(map(np.asarray, prim.bbox(solids[n])))
        for a, b in zip(L[:-1], L[1:]):
            if np.linalg.norm(b - a) < 1e-9:
                continue
            key = (tuple(np.round(a, 2)), tuple(np.round(b, 2)))
            lo, hi = np.minimum(a, b) - r, np.maximum(a, b) + r
            for n, (plo, phi) in bx.items():
                if np.any(hi < plo) or np.any(lo > phi):
                    continue
                k2 = key + (n,)
                if n in moving or k2 not in seen:
                    cap = Manifold.batch_hull([sph.translate(tuple(a)), sph.translate(tuple(b))])
                    seen[k2] = (cap ^ solids[n]).volume()
                if seen[k2] > 1e-3:
                    hits.append((S.get("phase"), n, np.round(a, 1).tolist(), np.round(b, 1).tolist(), round(seen[k2], 3)))
    assert {"load", "flight", "wrap", "wrapped", "tow", "reel_in", "unwrap", "approach"} <= phases
    assert not hits, f"{len(hits)} line spans run through frame parts, e.g. {hits[:4]}"
