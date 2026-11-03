"""Exact-mesh audit of the assembly. Prints CLEAN or NOT CLEAN and writes OUT/audit.json.

python -m tools.audit OUT [--only demo] [--dt 0.1] [--jobs 4]

Checks
1. Static, per part: valid manifold (watertight, 2-manifold), shell count (printed parts must be one body), print
   orientation (print_tf puts the part on the bed at z = 0, inside 256 x 256 x 260, with a real flat face), overhang
   area (downward faces steeper than 45 deg that are not on the bed), volume and mass.
2. Joint limits over every sampled state.
3. Collisions: every pair of parts over the mission timeline plus the subsystems' audit_states, AABB broad phase,
   exact manifold intersection volume. A pair may overlap only if an Allow covers it (and within its max_vol).
   Pairs whose relative pose does not change are computed once.
4. Clearances: min gap of every Clearance pair over all states, fails below min_mm.
5. Floating: in the rest state (first state) every part must touch (gap <= 0.2 mm) or overlap (allowed) another
   part; if a subsystem declares grounded(P), every part must connect to a grounded part.
"""
from __future__ import annotations

import argparse
import json
import math
import multiprocessing as mp
import os
import sys
import time
from fnmatch import fnmatchcase

import numpy as np
import manifold3d as m3

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

from cad.lib import prim, tf  # noqa: E402
from tools.assembly import Assembly  # noqa: E402
from tools.params import load_params  # noqa: E402
from tools.timeline import timeline  # noqa: E402

HIT_VOL = 1e-3          # mm3: below this an intersection counts as touching
TOUCH_GAP = 0.2         # mm: floating check
OVERHANG_DEG = 45.0
FLAT_MIN_FRAC = 0.05    # bed contact area at least this fraction of the footprint (or FLAT_MIN_MM2)
FLAT_MIN_MM2 = 20.0

_ASM = None             # set before forking workers


# ---------------------------------------------------------------------------------------------- static checks
def static_part(p, bed=(256.0, 256.0, 260.0)) -> dict:
    s = p.solid
    r = dict(name=p.name, subsystem=p.subsystem, material=p.material, printed=p.printed, fails=[], warns=[])
    st = s.status()
    r["status"] = str(st).split(".")[-1]
    if st != m3.Error.NoError or s.is_empty():
        r["fails"].append(f"invalid manifold ({r['status']})")
        return r
    r["volume_mm3"] = round(s.volume(), 2)
    r["area_mm2"] = round(s.surface_area(), 2)
    r["mass_g"] = None if p.mass_g() is None else round(p.mass_g(), 2)
    r["shells"] = prim.shells(s)
    r["genus"] = s.genus()
    if p.printed and r["shells"] != 1:
        r["fails"].append(f"{r['shells']} shells (printed parts must be one body)")
    elif r["shells"] != 1:
        r["warns"].append(f"{r['shells']} shells")
    if r["volume_mm3"] <= 0:
        r["fails"].append("non-positive volume")
    if p.printed:
        if p.print_tf is None:
            r["fails"].append("printed part without print_tf")
        else:
            r.update(print_check(s, p.print_tf, bed))
            r["fails"] += r.pop("_fails")
    return r


def print_check(solid, M, bed) -> dict:
    out = {"_fails": []}
    s = prim.place(solid, M)
    lo, hi = prim.bbox(s)
    size = np.array(hi) - np.array(lo)
    out["print_bbox"] = [round(float(x), 2) for x in size]
    if abs(lo[2]) > 1e-3:
        out["_fails"].append(f"print_tf min z = {lo[2]:.3f} (must be 0)")
    if size[2] > bed[2] + 1e-6 or not ((size[0] <= bed[0] and size[1] <= bed[1]) or (size[1] <= bed[0] and size[0] <= bed[1])):
        out["_fails"].append(f"does not fit the bed: {size.round(1).tolist()} vs {list(bed)}")
    v, f = prim.mesh_of(s)
    tri = v[f]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    a2 = np.linalg.norm(n, axis=1)
    ok = a2 > 1e-12
    nz = np.zeros(len(n)); nz[ok] = n[ok, 2] / a2[ok]
    area = a2 / 2
    zmin = tri[:, :, 2].min(axis=1); zmax = tri[:, :, 2].max(axis=1)
    on_bed = (zmax < lo[2] + 0.01) & (nz < -0.999)
    bed_area = float(area[on_bed].sum())
    foot = float(size[0] * size[1])
    out["bed_contact_mm2"] = round(bed_area, 1)
    if bed_area < min(FLAT_MIN_MM2, FLAT_MIN_FRAC * foot) and bed_area < FLAT_MIN_FRAC * foot:
        out["_fails"].append(f"no flat face on the bed ({bed_area:.1f} mm2)")
    lim = -math.cos(math.radians(OVERHANG_DEG))
    over = (nz < lim) & ~on_bed & (zmin > lo[2] + 0.3)
    out["overhang_mm2"] = round(float(area[over].sum()), 1)
    out["overhang_frac"] = round(float(area[over].sum() / max(1e-9, area.sum())), 4)
    return out


# ---------------------------------------------------------------------------------------------- pair work
def _aabb(solid, M):
    lo, hi = prim.bbox(solid)
    c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    w = tf.apply(M, c)
    return w.min(axis=0), w.max(axis=0)


def _overlap(a, b, pad=0.0):
    return bool(np.all(a[0] <= b[1] + pad) and np.all(b[0] <= a[1] + pad))


def _rel_key(Ma, Mb):
    return tuple(np.round(tf.inv(Ma) @ Mb, 4)[:3].reshape(-1))


def _pair_volume(task):
    a, b, Ma, Mb = task
    P = _ASM.parts
    sa = prim.place(P[a].solid, Ma)
    sb = prim.place(P[b].solid, Mb)
    return (sa ^ sb).volume()


def _pair_gap(task):
    a, b, Ma, Mb, search = task
    P = _ASM.parts
    sa = prim.place(P[a].solid, Ma)
    sb = prim.place(P[b].solid, Mb)
    return sa.min_gap(sb, search)


def _map(fn, tasks, jobs):
    if jobs <= 1 or len(tasks) < 8:
        return [fn(t) for t in tasks]
    ctx = mp.get_context("fork")
    with ctx.Pool(jobs) as pool:
        return pool.map(fn, tasks, chunksize=max(1, len(tasks) // (jobs * 8)))


# ---------------------------------------------------------------------------------------------- main audit
def run(asm: Assembly, states: list[dict], jobs: int = 4, log=print) -> dict:
    global _ASM
    _ASM = asm
    t_start = time.time()
    bed = tuple(getattr(asm.P, "BED", (256.0, 256.0, 260.0)))
    rep = dict(n_parts=len(asm.parts), n_states=len(states), errors=list(asm.errors), static=[], limits=[],
               collisions=[], allowed_contacts=[], clearances=[], floating=[], checks=[])
    names = list(asm.parts)

    # 1 static
    for n in names:
        rep["static"].append(static_part(asm.parts[n], bed))
    # 2 world transforms per state, joint limits
    Ws = []
    for i, S in enumerate(states):
        vals = asm.values(S)
        for jn, j in asm.joints.items():
            if j.limits is not None and not (j.limits[0] - 1e-6 <= vals[jn] <= j.limits[1] + 1e-6):
                rep["limits"].append(dict(joint=jn, value=vals[jn], limits=list(j.limits), state=i, phase=S.get("phase")))
        JW = asm.joint_world(S, vals)
        Ws.append({n: JW.get(p.joint, np.eye(4)) @ p.place for n, p in asm.parts.items()})
    # 3 collisions with caching on the relative pose
    allow_of = {}
    for ia, a in enumerate(names):
        for b in names[ia + 1:]:
            al = [x for x in asm.allowed if x.match(a, b)]
            allow_of[(a, b)] = al[0] if al else None
    seen = {}
    tasks, task_keys = [], []
    hits = {}           # (a, b) -> list of (state, vol)
    for i, W in enumerate(Ws):
        boxes = {n: _aabb(asm.parts[n].solid, W[n]) for n in names}
        for ia, a in enumerate(names):
            for b in names[ia + 1:]:
                if not _overlap(boxes[a], boxes[b]):
                    continue
                k = (a, b, _rel_key(W[a], W[b]))
                if k in seen:
                    seen[k].append(i)
                    continue
                seen[k] = [i]
                tasks.append((a, b, W[a], W[b])); task_keys.append(k)
    log(f"audit: {len(names)} parts, {len(states)} states, {len(tasks)} exact pair checks")
    vols = _map(_pair_volume, tasks, jobs)
    for k, v in zip(task_keys, vols):
        if v > HIT_VOL:
            for i in seen[k]:
                hits.setdefault((k[0], k[1]), []).append((i, v))
    for (a, b), lst in sorted(hits.items()):
        vmax = max(v for _, v in lst)
        imax = max(lst, key=lambda x: x[1])[0]
        al = allow_of[(a, b)]
        entry = dict(a=a, b=b, max_vol_mm3=round(vmax, 3), n_states=len(lst), worst_state=imax,
                     phase=states[imax].get("phase"), t=states[imax].get("t"))
        if al is not None and (al.max_vol is None or vmax <= al.max_vol):
            entry["why"] = al.why
            rep["allowed_contacts"].append(entry)
        else:
            if al is not None:
                entry["note"] = f"allowed up to {al.max_vol} mm3 ({al.why})"
            rep["collisions"].append(entry)
    # 4 clearances
    for c in asm.clearances:
        pairs = [(a, b) for ia, a in enumerate(names) for b in names[ia + 1:] if c.match(a, b)]
        if not pairs:
            rep["clearances"].append(dict(a=c.a, b=c.b, min_mm=c.min_mm, gap=None, ok=False, why=c.why,
                                          note="no parts match this clearance"))
            continue
        search = max(5.0, 3 * c.min_mm)
        ctasks, cmeta, cseen = [], [], set()
        for a, b in pairs:
            for i, W in enumerate(Ws):
                key = (a, b, _rel_key(W[a], W[b]))
                if key in cseen:
                    continue
                cseen.add(key)
                ctasks.append((a, b, W[a], W[b], search)); cmeta.append((a, b, i))
        gaps = _map(_pair_gap, ctasks, jobs)
        g, (a, b, i) = min(zip(gaps, cmeta), key=lambda x: x[0])
        if (a, b) in hits or (b, a) in hits:
            g = 0.0
        rep["clearances"].append(dict(a=a, b=b, rule=f"{c.a} / {c.b}", min_mm=c.min_mm, gap=round(g, 3),
                                      ok=g >= c.min_mm, worst_state=i, phase=states[i].get("phase"), why=c.why))
    # 5 floating (rest state = first state)
    W0 = Ws[0]
    boxes = {n: _aabb(asm.parts[n].solid, W0[n]) for n in names}
    adj = {n: set() for n in names}
    ftasks, fmeta = [], []
    for ia, a in enumerate(names):
        for b in names[ia + 1:]:
            if not _overlap(boxes[a], boxes[b], TOUCH_GAP):
                continue
            if any(i == 0 for i, _ in hits.get((a, b), [])):
                adj[a].add(b); adj[b].add(a)
                continue
            ftasks.append((a, b, W0[a], W0[b], 1.0)); fmeta.append((a, b))
    for g, (a, b) in zip(_map(_pair_gap, ftasks, jobs), fmeta):
        if g <= TOUCH_GAP:
            adj[a].add(b); adj[b].add(a)
    grounded = set()
    for sub, mod in asm.modules.items():
        if hasattr(mod, "grounded"):
            for pat in mod.grounded(asm.P):
                grounded |= {n for n in names if fnmatchcase(n, pat)}
    for n in names:
        if not adj[n] and len(names) > 1:
            rep["floating"].append(dict(part=n, why="touches nothing in the rest state"))
    if grounded:
        reach, todo = set(grounded), list(grounded)
        while todo:
            x = todo.pop()
            for y in adj[x] - reach:
                reach.add(y); todo.append(y)
        for n in names:
            if n not in reach and adj[n]:
                rep["floating"].append(dict(part=n, why="not connected to a grounded part"))
    rep["contact_graph"] = {n: sorted(adj[n]) for n in names}
    # 6 subsystem checks
    for sub, mod in asm.modules.items():
        if hasattr(mod, "checks"):
            for msg in mod.checks(asm.P, asm) or []:
                rep["checks"].append(dict(subsystem=sub, fail=msg))
    # verdict
    static_fails = [dict(part=r["name"], fails=r["fails"]) for r in rep["static"] if r["fails"]]
    rep["static_fails"] = static_fails
    bad = (rep["errors"] or static_fails or rep["limits"] or rep["collisions"] or rep["floating"] or rep["checks"]
           or [c for c in rep["clearances"] if not c["ok"]])
    rep["clean"] = not bad
    rep["seconds"] = round(time.time() - t_start, 1)
    return rep


def summary(rep: dict) -> str:
    L = []
    L.append(f"parts {rep['n_parts']}, states {rep['n_states']}, {rep['seconds']} s")
    for e in rep["errors"]:
        L.append(f"  ERROR {e}")
    for r in rep["static_fails"]:
        L.append(f"  STATIC {r['part']}: {'; '.join(r['fails'])}")
    for x in rep["limits"][:20]:
        L.append(f"  LIMIT {x['joint']} = {x['value']:.3f} outside {x['limits']} (state {x['state']}, {x['phase']})")
    for c in rep["collisions"]:
        L.append(f"  COLLISION {c['a']} x {c['b']}: {c['max_vol_mm3']} mm3 in {c['n_states']} states, worst state "
                 f"{c['worst_state']} ({c['phase']}, t {c['t']}) {c.get('note', '')}")
    for c in rep["clearances"]:
        tag = "ok  " if c["ok"] else "FAIL"
        L.append(f"  CLEARANCE {tag} {c['a']} / {c['b']}: min gap {c['gap']} mm (need {c['min_mm']}) {c.get('note', '')}")
    for f in rep["floating"]:
        L.append(f"  FLOATING {f['part']}: {f['why']}")
    for c in rep["checks"]:
        L.append(f"  CHECK {c['subsystem']}: {c['fail']}")
    L.append(f"  allowed contacts: {len(rep['allowed_contacts'])}")
    L.append("CLEAN" if rep["clean"] else "NOT CLEAN")
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--only", default="")
    ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    a = ap.parse_args(argv)
    P = load_params()
    asm = Assembly.load(P, [s for s in a.only.split(",") if s] or None)
    states = timeline(P, a.dt) + asm.extra_states
    rep = run(asm, states, a.jobs)
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "audit.json"), "w") as fh:
        json.dump(rep, fh, indent=1, default=str)
    s = summary(rep)
    with open(os.path.join(a.out, "audit.txt"), "w") as fh:
        fh.write(s + "\n")
    print(s)
    return 0 if rep["clean"] else 1


if __name__ == "__main__":
    sys.exit(main())
