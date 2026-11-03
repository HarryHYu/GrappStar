"""Faceted STEP export: one STEP per part (local frame) and one assembly STEP with every part placed at a state.

python -m tools.step_export OUT [--only demo] [--state '{"t": 0}'] [--parts a,b] [--jobs 4] [--no-assembly]

Each part mesh becomes planar faces (OCP): small coplanar groups of triangles are merged into one polygon face, the
rest stay triangles; vertices and edges are shared, so the shell is closed without sewing (sewing like
kedge/meshstep.py is the fallback). Writes OUT/step/<name>.step, OUT/step/assembly_rest.step (state = first
timeline state, the rest state, unless --state is given) and OUT/step/step_report.json. Every file is read back
with build123d and checked (valid shape, solid count, volume against the mesh). An invalid file is reported, never
fatal: the exit code is 0 unless an export crashed.
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import sys
import time

import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

from tools.pack import only_list  # noqa: E402

VOL_TOL = 0.01          # relative volume difference allowed on reimport
FACET_MAX_EDGES = 48    # merge coplanar triangles only while the polygon stays small (big polygons slow STEP I/O)

_JOB = {}               # per process: set by _init


# ---------------------------------------------------------------------------------------------- mesh -> solid
def _clean(v, f, tol=1e-4):
    """Merge vertices closer than tol and drop the triangles that collapse."""
    v = np.asarray(v, float)
    q = np.round(v / tol).astype(np.int64)
    _, idx, inv = np.unique(q, axis=0, return_index=True, return_inverse=True)
    f = inv.reshape(-1)[np.asarray(f, np.int64)]
    ok = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    return v[idx], f[ok]


def _facet_loops(v, f, fc, normal):
    """Boundary loops of a coplanar triangle group, outer first, or None when it is not a simple polygon (+ holes)."""
    de = set()
    for a, b, c in f[fc]:
        de |= {(a, b), (b, c), (c, a)}
    bnd = [(a, b) for a, b in de if (b, a) not in de]
    if len(bnd) > FACET_MAX_EDGES:
        return None
    nxt = {}
    for a, b in bnd:
        if a in nxt:
            return None                          # pinched vertex
        nxt[a] = b
    loops = []
    while nxt:
        a0, b = nxt.popitem()
        loop = [a0]
        while b != a0:
            if b not in nxt:
                return None
            loop.append(b)
            b = nxt.pop(b)
        loops.append(loop)
    ax = int(np.argmax(np.abs(normal)))
    i, j = [k for k in range(3) if k != ax]
    sgn = np.sign(normal[ax])

    def area(lp):
        p = v[lp]
        return 0.5 * sgn * np.sum(p[:, i] * np.roll(p[:, j], -1) - np.roll(p[:, i], -1) * p[:, j])

    ar = [area(lp) for lp in loops]
    outer = [k for k, a in enumerate(ar) if a > 0]
    if len(outer) != 1:
        return None
    return [loops[outer[0]]] + [lp for k, lp in enumerate(loops) if k != outer[0]]


def _build(v, f, merge=True):
    """Shared vertices and edges, planar faces, shell closed without sewing. Returns (TopoDS_Solid, face count)."""
    import trimesh
    from OCP.BRep import BRep_Builder
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeSolid,
                                    BRepBuilderAPI_MakeVertex, BRepBuilderAPI_MakeWire)
    from OCP.gp import gp_Pnt
    from OCP.TopoDS import TopoDS, TopoDS_Shell
    V = [BRepBuilderAPI_MakeVertex(gp_Pnt(*map(float, p))).Vertex() for p in v]
    E = {}

    def edge(a, b):
        k = (a, b) if a < b else (b, a)
        e = E.get(k)
        if e is None:
            e = E[k] = BRepBuilderAPI_MakeEdge(V[k[0]], V[k[1]]).Edge()
        return e if a < b else TopoDS.Edge(e.Reversed())

    def wire(loop):
        mw = BRepBuilderAPI_MakeWire()
        for a, b in zip(loop, loop[1:] + loop[:1]):
            mw.Add(edge(a, b))
        return mw.Wire()

    B = BRep_Builder()
    sh = TopoDS_Shell()
    B.MakeShell(sh)
    done = np.zeros(len(f), bool)
    n_faces = 0
    if merge:
        tm = trimesh.Trimesh(v, f, process=False)
        for fc, nrm in zip(tm.facets, tm.facets_normal):
            loops = _facet_loops(v, f, fc, nrm)
            if loops is None:
                continue
            mf = BRepBuilderAPI_MakeFace(wire(loops[0]), True)
            if not mf.IsDone():
                continue
            for lp in loops[1:]:
                mf.Add(wire(lp))
            B.Add(sh, mf.Face())
            done[fc] = True
            n_faces += 1
    for a, b, c in f[~done]:
        B.Add(sh, BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge(a, b), edge(b, c), edge(c, a)).Wire(), True).Face())
        n_faces += 1
    sh.Closed(True)
    return BRepBuilderAPI_MakeSolid(sh).Solid(), n_faces


def _sewn(v, f, tol=1e-3):
    """Fallback: independent triangles sewn together (like kedge/meshstep.py), slower but tolerant."""
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeSolid,
                                    BRepBuilderAPI_Sewing)
    from OCP.gp import gp_Pnt
    from OCP.ShapeFix import ShapeFix_Solid
    from OCP.TopAbs import TopAbs_SHELL
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS
    sew = BRepBuilderAPI_Sewing(tol)
    pts = [gp_Pnt(*map(float, p)) for p in v]
    for a, b, c in f:
        poly = BRepBuilderAPI_MakePolygon(pts[a], pts[b], pts[c], True)
        if poly.IsDone():
            fc = BRepBuilderAPI_MakeFace(poly.Wire(), True)
            if fc.IsDone():
                sew.Add(fc.Face())
    sew.Perform()
    ex = TopExp_Explorer(sew.SewedShape(), TopAbs_SHELL)
    ms = BRepBuilderAPI_MakeSolid()
    n = 0
    while ex.More():
        ms.Add(TopoDS.Shell(ex.Current()))
        n += 1
        ex.Next()
    if n == 0:
        raise RuntimeError("sewing produced no shell")
    fix = ShapeFix_Solid(ms.Solid())
    fix.Perform()
    return fix.Solid(), len(f)


def mesh_to_solid(v, f):
    """(vertices, faces) -> (build123d Solid of planar faces, method, face count). Tries merged facets, then plain
    triangles, then sewing; the first valid one wins."""
    from build123d import Solid
    v, f = _clean(v, f)
    last = None
    for method in ("facets", "triangles", "sewn"):
        try:
            raw, n = _sewn(v, f) if method == "sewn" else _build(v, f, merge=(method == "facets"))
            s = Solid(raw)
            if s.volume < 0:
                s = Solid(raw.Reversed())
            if s.is_valid or method == "sewn":
                return s, method, n
        except Exception as e:                   # noqa: BLE001
            last = e
    raise RuntimeError(f"no valid solid ({last})")


def location(M):
    from build123d import Location
    from OCP.gp import gp_Trsf
    M = np.asarray(M, float)
    t = gp_Trsf()
    t.SetValues(*[float(M[i, j]) for i in range(3) for j in range(4)])
    return Location(t)


def check_step(fn, ref_volume, n_solids=None, full=True):
    """Reimport and validate. Returns dict(ok, valid, n_solids, volume, rel_err, error)."""
    from build123d import import_step
    r = dict(file=os.path.basename(fn), ok=False)
    try:
        shp = import_step(fn)
        solids = shp.solids()
        r["n_solids"] = len(solids)
        r["valid"] = bool(shp.is_valid) if full else None
        vol = float(sum(abs(s.volume) for s in solids))
        r["volume"] = round(vol, 2)
        r["rel_err"] = round(abs(vol - ref_volume) / max(ref_volume, 1e-9), 5)
        r["ok"] = r["valid"] is not False and len(solids) > 0 and r["rel_err"] < VOL_TOL
        if n_solids is not None and len(solids) != n_solids:
            r["ok"] = False
            r["error"] = f"{len(solids)} solids, expected {n_solids}"
    except Exception as e:                       # noqa: BLE001  report, never fail the pipeline
        r["error"] = f"{type(e).__name__}: {e}"
    return r


# ---------------------------------------------------------------------------------------------- jobs
def _part_job(name):
    from build123d import export_step
    from cad.lib import prim
    p = _JOB["asm"].parts[name]
    t0 = time.time()
    fn = os.path.join(_JOB["sdir"], f"{name}.step")
    v, f = prim.mesh_of(p.solid)
    r = dict(file=os.path.basename(fn), ok=False)
    try:
        sol, method, n = mesh_to_solid(v, f)
        sol.label = name
        export_step(sol, fn)
        r = check_step(fn, p.solid.volume())
        r.update(method=method, step_faces=n)
    except Exception as e:                       # noqa: BLE001
        r["error"] = f"{type(e).__name__}: {e}"
        r["crashed"] = True
    r.update(part=name, triangles=int(len(f)), seconds=round(time.time() - t0, 2))
    return r


def _assembly_job(names):
    from build123d import Compound, export_step
    from cad.lib import prim
    asm, W = _JOB["asm"], _JOB["W"]
    t0 = time.time()
    fn = os.path.join(_JOB["sdir"], "assembly_rest.step")
    r = dict(file=os.path.basename(fn), ok=False)
    try:
        placed = []
        for n in names:
            sol, _, _ = mesh_to_solid(*prim.mesh_of(asm.parts[n].solid))
            s2 = sol.moved(location(W[n]))
            s2.label = n
            placed.append(s2)
        comp = Compound(children=placed)
        comp.label = "assembly_rest"
        export_step(comp, fn)
        r = check_step(fn, sum(asm.parts[n].solid.volume() for n in names), n_solids=len(names), full=False)
    except Exception as e:                       # noqa: BLE001
        r["error"] = f"{type(e).__name__}: {e}"
        r["crashed"] = True
    r.update(n_parts=len(names), seconds=round(time.time() - t0, 2))
    return r


def _dispatch(task):
    kind, arg = task
    return _part_job(arg) if kind == "part" else _assembly_job(arg)


def _init(only, state, sdir):
    """Worker set-up (also used in the parent): the assembly, the world poses at the state, the output folder."""
    from tools.assembly import Assembly
    from tools.params import load_params
    from tools.timeline import timeline
    P = load_params()
    asm = Assembly.load(P, only_list(only))
    S = state if state is not None else timeline(P, 0.5)[0]
    _JOB.update(asm=asm, S=S, W=asm.world(S), sdir=sdir)


def run(out, only=None, state=None, part_names=None, jobs=None, assembly=True, log=print) -> dict:
    sdir = os.path.join(out, "step")
    os.makedirs(sdir, exist_ok=True)
    _init(only, state, sdir)
    asm, S = _JOB["asm"], _JOB["S"]
    names = [n for n in asm.parts if not part_names or n in part_names]
    # longest first: the assembly, then the parts by triangle count
    size = {n: len(asm.parts[n].solid.to_mesh().tri_verts) for n in names}
    tasks = ([("asm", names)] if assembly else []) + [("part", n) for n in sorted(names, key=lambda n: -size[n])]
    jobs = max(1, min(jobs or os.cpu_count() or 1, len(tasks)))
    t0 = time.time()
    if jobs > 1:
        # spawn, not fork: manifold and OCP may run threads, and a forked child can deadlock on their locks
        with mp.get_context("spawn").Pool(jobs, initializer=_init, initargs=(only, state, sdir)) as pool:
            res = pool.map(_dispatch, tasks, chunksize=1)
    else:
        res = [_dispatch(t) for t in tasks]
    rep = dict(created=time.strftime("%Y-%m-%d %H:%M"), state={k: v for k, v in S.items() if isinstance(v, (int, float, str))},
               parts=sorted([r for r in res if "part" in r], key=lambda r: names.index(r["part"])),
               assembly=res[0] if assembly else None, seconds=round(time.time() - t0, 1))
    rep["crashed"] = [r.get("part", "assembly") for r in res if r.get("crashed")]
    for r in rep["parts"]:
        err = f"  vol err {100 * r['rel_err']:.3f} %" if "rel_err" in r else ""
        log(f"  {r['part']:28s} {r['triangles']:7d} tri -> {r.get('step_faces', 0):6d} faces ({r.get('method', '-')}) "
            f"{r['seconds']:6.1f} s  {'ok' if r['ok'] else 'INVALID ' + r.get('error', '')}{err}")
    a = rep["assembly"] or dict(n_parts=0, seconds=0, ok=True, error="")
    if rep["assembly"]:
        log(f"  assembly_rest.step: {a['n_parts']} parts, {a['seconds']} s, {'ok' if a['ok'] else 'INVALID ' + a.get('error', '')}")
    with open(os.path.join(sdir, "step_report.json"), "w") as fh:
        json.dump(rep, fh, indent=1)
    bad = [r["part"] for r in rep["parts"] if not r["ok"]]
    log(f"step: {len(rep['parts'])} parts, {len(bad)} invalid{': ' + ', '.join(bad) if bad else ''}, "
        f"{rep['seconds']} s -> {sdir}")
    return rep


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--only", default="")
    ap.add_argument("--state", default="", help="JSON dict of the master state for the assembly STEP")
    ap.add_argument("--parts", default="", help="comma list: only these parts")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--no-assembly", action="store_true", help="skip the assembly STEP")
    a = ap.parse_args(argv)
    rep = run(a.out, a.only, json.loads(a.state) if a.state else None, only_list(a.parts), a.jobs, not a.no_assembly)
    return 1 if rep["crashed"] else 0


if __name__ == "__main__":
    sys.exit(main())
