"""The fast audit (chunked parts, vectorised broad phase, pose grouping, branch and bound clearances) must report
what whole-part booleans at every state report. A small moving assembly is audited with tiny chunks (so every gear
is split into many pieces) and compared with a brute-force reference; plus a speed guard."""
import time
import types

import numpy as np
import pytest

from cad.lib import Part, Joint, Allow, Clearance, prim, tf, gears
from tools.assembly import Assembly
from tools import audit
from tools.params import load_params

Q = dict(log=lambda *a: None)


def _asm():
    m, z1, z2, face = 1.5, 18, 27, 8.0
    cd = gears.center_distance(m, z1, z2)
    g1 = gears.spur_gear(m, z1, face, bore=6)
    g2 = gears.spur_gear(m, z2, face, bore=6)
    plate = prim.cbox(120, 70, 5, x=cd / 2, z=-5)
    pin = prim.cyl(1.5, 30, z=-2, seg=24)
    ps = [Part("plate", plate, "steel"),
          Part("post_1", prim.cyl(3, 14, z=0), "steel"),
          Part("post_2", prim.cyl(3, 14, x=cd, z=0), "steel"),
          Part("gear_1", g1, "PPA-CF", joint="g1", place=tf.T(0, 0, 1), print_tf=tf.I4),
          Part("gear_2", g2, "PPA-CF", joint="g2", place=tf.T(0, 0, 1), print_tf=tf.I4),
          # a pin on a slide that dips into gear 2's teeth for part of the travel (real, varying collisions)
          Part("pin", pin, "steel", joint="pin"),
          # a block inside gear 1's loose box but outside its swept cylinder (cleared by the motion envelope)
          Part("block", prim.cbox(4, 4, 8, x=-13.2, y=-13.2, z=0), "steel"),
          # an arm swinging over the gears, clear of them
          Part("arm", prim.cbox(6, 50, 3, y=25, z=0), "PLA", joint="arm", print_tf=tf.I4)]
    off = 180.0 / z2 if z2 % 2 == 0 else 0.0
    js = [Joint("g1", None, "rev", (0, 0, 0), (0, 0, 1)),
          Joint("g2", None, "rev", (cd, 0, 0), (0, 0, 1), gear=("g1", -z1 / z2, off)),
          Joint("pin", None, "slide", (cd + z2 * m / 2 + 4, 0, 0), (-1, 0, 0)),
          Joint("arm", None, "rev", (0, -10, face + 6), (0, 0, 1))]

    def kin(P, S):
        return {"g1": S["q"], "pin": S["d"], "arm": S["q"] * 0.5}

    mod = types.SimpleNamespace(
        parts=lambda P: ps, joints=lambda P: js, kin=kin,
        allowed=lambda P: [Allow("post_*", "plate", "screwed"), Allow("gear_*", "post_*", "shaft in bore")],
        clearances=lambda P: [Clearance("gear_1", "gear_2", 0.05, "backlash"), Clearance("arm", "gear_*", 1.0, "arm")],
        grounded=lambda P: ["plate"])
    return Assembly(load_params(), {"t": mod})


def _states(n):
    return [dict(q=7.3 * k, d=0.6 * k, phase="p") for k in range(n)]


def _brute(asm, states):
    """Whole-part booleans and min_gap at every state, no caching."""
    names = list(asm.parts)
    hits, gaps = {}, {}
    for i, S in enumerate(states):
        Wd = asm.world(S)
        posed = {n: prim.place(asm.parts[n].solid, Wd[n]) for n in names}
        for ia, a in enumerate(names):
            for b in names[ia + 1:]:
                v = (posed[a] ^ posed[b]).volume()
                if v > audit.HIT_VOL:
                    hits.setdefault((a, b), []).append((i, v))
        for c in asm.clearances:
            for ia, a in enumerate(names):
                for b in names[ia + 1:]:
                    if c.match(a, b):
                        g = posed[a].min_gap(posed[b], max(5.0, 3 * c.min_mm))
                        gaps.setdefault(c.a + "/" + c.b, []).append(g)
    return hits, {k: min(v) for k, v in gaps.items()}


@pytest.mark.parametrize("jobs", [1, 2])
def test_fast_audit_matches_brute_force(monkeypatch, jobs):
    monkeypatch.setattr(audit, "CHUNK_TRI", 400)
    monkeypatch.setattr(audit, "SPLIT_MIN", 800)
    asm = _asm()
    states = _states(30)
    rep = audit.run(asm, states, jobs=jobs, **Q)
    st = rep["stats"]
    assert st["chunked_parts"] >= 2 and st["chunks"] > 20 and st["envelope_cleared_pairs"] >= 1 and st["cropped_pairs"] >= 2
    hits, gaps = _brute(asm, states)
    got = {(e["a"], e["b"]): e for e in rep["collisions"] + rep["allowed_contacts"]}
    assert set(got) == set(hits), (set(got) ^ set(hits))
    assert ("gear_2", "pin") in got or ("pin", "gear_2") in got
    for k, lst in hits.items():
        e = got[k]
        assert e["n_states"] == len(lst)
        assert e["max_vol_mm3"] == pytest.approx(round(max(v for _, v in lst), 3), abs=2e-3)
    for c in rep["clearances"]:
        assert c["gap"] == pytest.approx(round(gaps[c["rule"].replace(" / ", "/")], 3), abs=1.5e-3)
    assert rep["floating"] == []


def test_floating_and_static_match_brute_force(monkeypatch):
    monkeypatch.setattr(audit, "CHUNK_TRI", 400)
    monkeypatch.setattr(audit, "SPLIT_MIN", 800)
    asm = _asm()
    rep = audit.run(asm, _states(1), jobs=1, **Q)
    names = list(asm.parts)
    Wd = asm.world(_states(1)[0])
    posed = {n: prim.place(asm.parts[n].solid, Wd[n]) for n in names}
    touch = {n: set() for n in names}
    for ia, a in enumerate(names):
        for b in names[ia + 1:]:
            if (posed[a] ^ posed[b]).volume() > audit.HIT_VOL or posed[a].min_gap(posed[b], 1.0) <= audit.TOUCH_GAP:
                touch[a].add(b); touch[b].add(a)
    assert rep["contact_graph"] == {n: sorted(v) for n, v in touch.items()}
    assert [r["name"] for r in rep["static"]] == names
    assert rep["static"][3] == audit.static_part(asm.parts["gear_1"])


def test_chunks_partition_the_solid(monkeypatch):
    monkeypatch.setattr(audit, "CHUNK_TRI", 500)
    monkeypatch.setattr(audit, "SPLIT_MIN", 1000)
    g = gears.spur_gear(1.0, 40, 10, bore=8)
    ch = audit._chunks(g)
    assert len(ch) > 8 and all(c.num_tri() <= 3000 for c in ch)
    assert sum(c.volume() for c in ch) == pytest.approx(g.volume(), rel=1e-9)
    other = prim.place(g, tf.T(gears.center_distance(1.0, 40, 40) - 1.0, 0, 0))
    v = sum((c ^ other).volume() for c in ch)
    assert v == pytest.approx((g ^ other).volume(), rel=1e-7)


def test_speed_many_states():
    """A gear pair with 1200 distinct poses plus 40 static parts: well under the old per-pair Python loop."""
    asm = _asm()
    frame = [Part(f"fr_{i}", prim.cbox(4, 4, 4, x=-60 + 6 * (i % 20), y=40 + 6 * (i // 20), z=0), "steel")
             for i in range(40)]
    asm.parts.update({p.name: p for p in frame})
    states = [dict(q=0.9 * k, d=0.0, phase="p") for k in range(1200)]
    t = time.time()
    rep = audit.run(asm, states, jobs=2, **Q)
    assert time.time() - t < 40, rep["timing"]
    assert rep["n_states"] == 1200
