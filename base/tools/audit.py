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
from fractions import Fraction

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
FLAT_CAP_MM2 = 150.0    # never ask for more bed contact than this
OVERHANG_WARN_MM2 = 100.0
MAX_STEP_DEG = 1.0      # timeline refinement: no rev joint moves more than this between audited states
MAX_STEP_MM = 0.5       # ... and no slide joint more than this

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
    if p.printed:                      # what an STL loader / slicer sees after merging coincident vertices
        tm = prim.to_trimesh(s)
        tm.merge_vertices()
        r["stl_watertight"] = bool(tm.is_watertight)
        if not tm.is_watertight:
            r["fails"].append("not watertight as STL (self-touching edges after vertex merge)")
    if p.printed:
        if p.print_tf is None:
            r["fails"].append("printed part without print_tf")
        else:
            r.update(print_check(s, p.print_tf, bed))
            r["fails"] += r.pop("_fails")
            r["warns"] += r.pop("_warns")
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
    need = min(max(FLAT_MIN_MM2, FLAT_MIN_FRAC * foot), FLAT_CAP_MM2)
    if bed_area < need:
        out["_fails"].append(f"no real flat face on the bed ({bed_area:.1f} mm2, need {need:.0f})")
    lim = -math.cos(math.radians(OVERHANG_DEG))
    over = (nz < lim) & ~on_bed & (zmin > lo[2] + 0.3)
    out["overhang_mm2"] = round(float(area[over].sum()), 1)
    out["overhang_frac"] = round(float(area[over].sum() / max(1e-9, area.sum())), 4)
    out["_warns"] = []
    if out["overhang_mm2"] > OVERHANG_WARN_MM2:
        out["_warns"].append(f"overhang {out['overhang_mm2']} mm2 steeper than {OVERHANG_DEG:g} deg: needs supports "
                             f"or a better print_tf")
    return out


# ---------------------------------------------------------------------------------------------- pair work
# Speed without weaker checks (the verdict and the numbers are those of whole-part booleans at every state):
# - World transforms are kept per joint (states x joints x 4 x 4); part transforms are derived when needed.
# - Broad phase: loose world AABBs of every part at every state (numpy), compared state-vectorised per pair.
#   Candidate states of a pair are grouped by the rounded relative pose (as before): one exact check per group.
# - Parts above SPLIT_MIN triangles are split once into closed chunks (split_by_plane). The chunks partition the
#   solid, so vol(A ^ B) is the sum of vol(Ai ^ Bj) over chunk pairs and min_gap(A, B) the minimum over chunk pairs.
#   A chunk pair whose boxes are apart in A's frame or in B's frame contributes nothing and is skipped: exact.
# - Clearance minima: branch and bound on those box distances (a pose whose lower bound exceeds the best gap found
#   so far cannot change the minimum, and the min_gap search length shrinks to the best gap).
# - Floating: a pair touches when some chunk pair is within TOUCH_GAP; the search stops at the first such pair.
# - One fork pool per run, each worker pinned to one core (manifold3d runs its own threads, which only contend when
#   several workers run), small task batches streamed with imap_unordered, workers recycled (maxtasksperchild).
CHUNK_TRI = 6000        # target triangles per chunk
SPLIT_MIN = 12000       # parts with more triangles are chunked
BOX_PAD = 1e-6          # mm: box tests are padded by this (float safety, boxes are already conservative)
BATCH_COST = 400_000    # triangles per worker task (sum over its chunk pairs)
PROGRESS_S = 20.0       # seconds between progress lines

_CTX = {}               # worker context, filled before the pool forks: chunks and boxes per part index


def _bisect(s):
    """Split a manifold in two along its longest extent at the median triangle centroid (None if not possible)."""
    v, f = prim.mesh_of(s)
    if len(f) == 0:
        return None
    c = v[f].mean(axis=1)
    ext = c.max(axis=0) - c.min(axis=0)
    for ax in np.argsort(-ext):
        if ext[ax] <= 1e-6:
            break
        nrm = [0.0, 0.0, 0.0]
        nrm[int(ax)] = 1.0
        a, b = s.split_by_plane(nrm, float(np.median(c[:, ax])) + 1.2345e-4)
        na, nb = a.num_tri(), b.num_tri()
        if na and nb and na < len(f) and nb < len(f):
            return [a, b]
    return None


def _chunks(solid):
    """Closed pieces that partition the solid (the solid itself when small)."""
    if solid.is_empty() or solid.num_tri() <= SPLIT_MIN:
        return [solid]
    out, todo = [], [solid]
    while todo:
        s = todo.pop()
        sub = _bisect(s) if s.num_tri() > CHUNK_TRI else None
        if sub is None:
            out.append(s)
        else:
            todo += sub
    return out


def _bounds(solid):
    b = np.asarray(solid.bounding_box(), float).reshape(2, 3)
    if not np.all(np.isfinite(b)) or np.any(b[1] < b[0]):          # empty solid: a box that meets nothing
        b = np.array([[np.inf] * 3, [-np.inf] * 3])
    return b


def _boxdist(lo1, hi1, lo2, hi2):
    """Distance between axis-aligned boxes (0 when they overlap), broadcasting."""
    d = np.maximum(np.maximum(lo1 - hi2, lo2 - hi1), 0.0)
    return np.sqrt(np.einsum("...i,...i->...", d, d))


def _xbox(box, R, t):
    """Boxes (C, 2, 3) under transforms R (K, 3, 3), t (K, 3) -> conservative AABBs lo, hi (K, C, 3)."""
    c = (box[:, 0] + box[:, 1]) / 2
    e = (box[:, 1] - box[:, 0]) / 2
    cc = np.einsum("kij,cj->kci", R, c) + t[:, None, :]
    ee = np.einsum("kij,cj->kci", np.abs(R), e)
    return cc - ee, cc + ee


def _rel(Wa, Wb):
    """inv(Wa) @ Wb for stacks of rigid transforms (K, 4, 4)."""
    Ra = np.transpose(Wa[:, :3, :3], (0, 2, 1))
    out = np.zeros_like(Wb)
    out[:, :3, :3] = Ra @ Wb[:, :3, :3]
    out[:, :3, 3] = np.einsum("kij,kj->ki", Ra, Wb[:, :3, 3] - Wa[:, :3, 3])
    out[:, 3, 3] = 1.0
    return out


def _cull(a, b, rels, pad):
    """Chunk pairs of parts a, b that may be within pad of each other, for each relative pose rels (K, 4, 4) mapping
    b's frame to a's. Returns arrays (k, i, j, lower bound of the chunk pair distance)."""
    ca, cb = _CTX["cb"][a], _CTX["cb"][b]
    ba, bb = _CTX["box"][a], _CTX["box"][b]
    K = len(rels)
    R, t = rels[:, :3, :3], rels[:, :3, 3]
    Ri = np.transpose(R, (0, 2, 1))
    ti = -np.einsum("kij,kj->ki", Ri, t)
    out = []
    step = max(1, int(2_000_000 // max(1, len(ca) * len(cb))))
    for k0 in range(0, K, step):
        sl = slice(k0, min(K, k0 + step))
        blo, bhi = _xbox(cb, R[sl], t[sl])             # b chunks in a's frame
        alo, ahi = _xbox(ca, Ri[sl], ti[sl])           # a chunks in b's frame
        mb = _boxdist(blo, bhi, ba[0], ba[1]) <= pad
        ma = _boxdist(alo, ahi, bb[0], bb[1]) <= pad
        k, i, j = np.nonzero(ma[:, :, None] & mb[:, None, :])
        if not len(k):
            continue
        lb = np.maximum(_boxdist(ca[i, 0], ca[i, 1], blo[k, j], bhi[k, j]),
                        _boxdist(alo[k, i], ahi[k, i], cb[j, 0], cb[j, 1]))
        keep = lb <= pad
        out.append((k[keep] + k0, i[keep], j[keep], lb[keep]))
    if not out:
        e = np.zeros(0, int)
        return e, e, e, np.zeros(0)
    return tuple(np.concatenate(x) for x in zip(*out))


def _w_init(counter, cpus):
    if cpus:
        with counter.get_lock():
            k = counter.value
            counter.value += 1
        try:
            os.sched_setaffinity(0, {cpus[k % len(cpus)]})
        except (AttributeError, OSError):
            pass


def _w_static(batch):
    return [(k, static_part(_ASM.parts[n], _CTX["bed"])) for k, n in batch]


def _w_volume(batch):
    """[(item, a, b, rel, pairs)] -> [(item, sum of chunk pair intersection volumes)]"""
    out = []
    ch = _CTX["ch"]
    for iid, a, b, rel, pairs in batch:
        A, B, posed, v = ch[a], ch[b], {}, 0.0
        for i, j in pairs:
            sb = posed.get(j)
            if sb is None:
                sb = posed[j] = prim.place(B[j], rel)
            v += (A[i] ^ sb).volume()
        out.append((iid, v))
    return out


def _w_gap(batch):
    """[(item, a, b, rel, pairs sorted by lower bound, lower bounds, search, stop)] -> [(item, gap)]: min_gap over the
    chunk pairs, capped at search; stops early once the gap is <= stop."""
    out = []
    ch = _CTX["ch"]
    for iid, a, b, rel, pairs, lbs, search, stop in batch:
        A, B, posed, g = ch[a], ch[b], {}, search
        for (i, j), lb in zip(pairs, lbs):
            if lb > g or g <= stop:
                break
            sb = posed.get(j)
            if sb is None:
                sb = posed[j] = prim.place(B[j], rel)
            g = min(g, A[i].min_gap(sb, g))
        out.append((iid, g))
    return out


class _Pool:
    """Lazy fork pool over _CTX (serial when jobs <= 1)."""

    def __init__(self, jobs):
        self.jobs = max(1, int(jobs))
        self.pool = None

    def imap(self, fn, batches):
        if self.jobs <= 1 or len(batches) < 2:
            for b in batches:
                yield fn(b)
            return
        if self.pool is None:
            ctx = mp.get_context("fork")
            try:
                cpus = sorted(os.sched_getaffinity(0))
            except AttributeError:
                cpus = []
            cpus = cpus[-self.jobs:] if len(cpus) > self.jobs else cpus
            self.pool = ctx.Pool(self.jobs, initializer=_w_init, initargs=(ctx.Value("i", 0), cpus),
                                 maxtasksperchild=400)
        yield from self.pool.imap_unordered(fn, batches)

    def close(self):
        if self.pool is not None:
            self.pool.terminate()
            self.pool.join()
            self.pool = None


def _batches(items, cost):
    """Group items into batches of about BATCH_COST triangles (cost(item)) and at most 256 items."""
    out, cur, c = [], [], 0
    for it in items:
        cur.append(it)
        c += cost(it)
        if c >= BATCH_COST or len(cur) >= 256:
            out.append(cur)
            cur, c = [], 0
    if cur:
        out.append(cur)
    return out


class _Progress:
    def __init__(self, log, what, total):
        self.log, self.what, self.total, self.t0, self.last = log, what, total, time.time(), time.time()

    def step(self, done):
        if time.time() - self.last >= PROGRESS_S:
            self.last = time.time()
            self.log(f"audit: {self.what} {done}/{self.total} ({time.time() - self.t0:.0f} s)")


def _matcher(rules, names):
    """For Allow / Clearance rules: per rule the sets of names matching its a and b patterns (fnmatchcase)."""
    return [({n for n in names if fnmatchcase(n, r.a)}, {n for n in names if fnmatchcase(n, r.b)}) for r in rules]


ENV_SEG = 96            # sides of the prism that encloses a rotating part's swept cylinder
ENV_PAD = 1e-3          # mm added to every motion envelope
CROP_MIN_POSES = 8      # pairs with at least this many distinct poses are cropped to their envelopes


def _verts(solid):
    return prim.mesh_of(solid)[0] if not solid.is_empty() else np.zeros((0, 3))


def _env_solid(kind, u, travel=(0.0, 0.0)):
    """Solid in the motion frame (z = joint axis) enclosing every pose of points u (N, 3) of that frame:
    rev: a prism around the whole swept cylinder; slide: the box swept along z by travel; 'zprism': a box in x, y
    unbounded along z (invariant under the slide)."""
    if not len(u):
        return None
    if kind == "rev":
        r = float(np.sqrt((u[:, 0] ** 2 + u[:, 1] ** 2).max())) + ENV_PAD
        z0, z1 = float(u[:, 2].min()) - ENV_PAD, float(u[:, 2].max()) + ENV_PAD
        return prim.cyl(r / math.cos(math.pi / ENV_SEG), z1 - z0, z=z0, seg=ENV_SEG)
    lo, hi = u.min(axis=0) - ENV_PAD, u.max(axis=0) + ENV_PAD
    if kind == "slide":
        return prim.box(lo[0], hi[0], lo[1], hi[1], lo[2] + travel[0], hi[2] + travel[1])
    return prim.box(lo[0], hi[0], lo[1], hi[1], lo[2] - 1e5, hi[2] + 1e5)


# ---------------------------------------------------------------------------------------------- main audit
def run(asm: Assembly, states: list[dict], jobs: int = 4, log=print, pair_refine: int = 0, max_mm: float = MAX_STEP_MM,
        cap: int = 100000) -> dict:
    """Audit asm over states. pair_refine = n: states[:n] are a timeline whose consecutive states each pair refines
    for itself (sub-states where its relative pose moves a point of either part by more than max_mm, at most cap per
    interval); the other states are audited as given. 0 audits exactly the given states."""
    global _ASM, _CTX
    _ASM = asm
    t_start = time.time()
    timing = {}
    tick = [time.time()]

    def lap(name):
        now = time.time()
        timing[name] = round(timing.get(name, 0.0) + now - tick[0], 2)
        tick[0] = now

    bed = tuple(getattr(asm.P, "BED", (256.0, 256.0, 260.0)))
    rep = dict(n_parts=len(asm.parts), n_states=len(states), errors=list(asm.errors), static=[], limits=[],
               collisions=[], allowed_contacts=[], clearances=[], floating=[], checks=[], warns=[])
    names = list(asm.parts)
    NP, NS = len(names), len(states)
    pool = _Pool(jobs)
    try:
        _run(asm, states, rep, names, NP, NS, bed, pool, log, lap, min(int(pair_refine), NS), max_mm, cap)
    finally:
        pool.close()
        _CTX = {}
    # gear meshes across owners (declared Mesh pairs): centre distance, parallel axes, face overlap, ratio
    from tools import meshcheck
    t0 = time.time()
    mres = meshcheck.run(asm, states)
    rep["meshes"] = mres["meshes"]
    rep["mesh_fails"] = mres["fails"]
    rep["warns"] += mres["warns"]
    timing["meshes"] = round(time.time() - t0, 2)
    rep["timing"] = timing
    # verdict
    static_fails = [dict(part=r["name"], fails=r["fails"]) for r in rep["static"] if r["fails"]]
    rep["static_fails"] = static_fails
    bad = (rep["errors"] or static_fails or rep["limits"] or rep["collisions"] or rep["floating"] or rep["checks"]
           or [c for c in rep["clearances"] if not c["ok"]] or rep["mesh_fails"])
    rep["clean"] = not bad
    rep["seconds"] = round(time.time() - t_start, 1)
    return rep


def _discrete_change(A, B):
    """True when a non-numeric value, a bool or a 0/1 flag differs between states A and B (phase and t ignored)."""
    for key in set(A) | set(B):
        if key in ("phase", "t", "_joints"):
            continue
        x, y = A.get(key), B.get(key)
        if x == y:
            continue
        num = lambda v: isinstance(v, (int, float)) and not isinstance(v, bool)
        if not (num(x) and num(y)):
            return True
        if isinstance(x, int) and isinstance(y, int) and {x, y} <= {0, 1}:
            return True
    return False


def _interp(A, B, u):
    """State between A and B at fraction u: numeric (not bool) values linear, everything else from A."""
    S = dict(A)
    for key, x in B.items():
        y = A.get(key)
        if isinstance(x, (int, float)) and not isinstance(x, bool) and isinstance(y, (int, float)) \
                and not isinstance(y, bool):
            S[key] = y + (x - y) * u
    return S


def _inv(M):
    """Inverse of a stack of rigid transforms (K, 4, 4)."""
    out = np.zeros_like(M)
    Rt = np.transpose(M[:, :3, :3], (0, 2, 1))
    out[:, :3, :3] = Rt
    out[:, :3, 3] = -np.einsum("kij,kj->ki", Rt, M[:, :3, 3])
    out[:, 3, 3] = 1.0
    return out


def _move(r1, r2, c, rho):
    """Upper bound of how far a point within rho of c (frame of the moving body, posed by r1) moves from pose r1 to
    pose r2 (stacks (K, 4, 4))."""
    D = r2 @ _inv(r1)
    cosphi = np.clip((np.trace(D[:, :3, :3], axis1=1, axis2=2) - 1) / 2, -1.0, 1.0)
    q = np.einsum("kij,j->ki", r1[:, :3, :3], c) + r1[:, :3, 3]
    dq = np.einsum("kij,kj->ki", D[:, :3, :3], q) + D[:, :3, 3] - q
    return np.sqrt(np.maximum(0.0, 2 - 2 * cosphi)) * rho + np.linalg.norm(dq, axis=1)


class _States:
    """Audited states: the given ones (rows 0..NB-1) plus sub-states inserted between consecutive timeline states for
    the pairs that need them (rows appended; ord gives the time order). JW holds the joint world transforms."""

    def __init__(self, asm, states, jcol, rep):
        self.asm, self.jcol, self.rep = asm, jcol, rep
        self.S = list(states)
        self.ord = [float(k) for k in range(len(states))]
        self.idx = {}
        self.JW = np.empty((max(256, 2 * len(states)), len(jcol) + 1, 4, 4))
        self.JW[:, 0] = np.eye(4)
        self.V = np.zeros((len(states), len(jcol)))          # joint values at the given states
        for k, S in enumerate(states):
            self._fill(k, S)

    def _fill(self, row, S):
        asm = self.asm
        vals = asm.values(S)
        for jn, j in asm.joints.items():
            if j.limits is not None and not (j.limits[0] - 1e-6 <= vals[jn] <= j.limits[1] + 1e-6):
                self.rep["limits"].append(dict(joint=jn, value=vals[jn], limits=list(j.limits), state=row,
                                               phase=S.get("phase")))
        jw = asm.joint_world(S, vals)
        for jn, k in self.jcol.items():
            self.JW[row, k] = jw[jn]
            if row < len(self.V):
                self.V[row, k - 1] = vals[jn]

    def sub(self, k, fr):
        """Row of the state at fraction fr (Fraction in (0, 1)) between states k and k + 1."""
        r = self.idx.get((k, fr))
        if r is None:
            r = len(self.S)
            if r >= len(self.JW):
                grow = np.empty((len(self.JW), self.JW.shape[1], 4, 4))
                grow[:, 0] = np.eye(4)
                self.JW = np.concatenate([self.JW, grow])
            u = float(fr)
            A, B = self.S[k], self.S[k + 1]
            S = _interp(A, B, u)
            if _discrete_change(A, B):
                # a string / bool / 0-1 flag changes in this interval: kin of the blended state would jump at the
                # end, so the joints are interpolated directly and sweep evenly instead
                vals = self.V[k] * (1 - u) + self.V[k + 1] * u
                S["_joints"] = {jn: float(vals[c - 1]) for jn, c in self.jcol.items()}
            # otherwise kin is evaluated on the blended state (exact for joints that follow others nonlinearly)
            self.S.append(S)
            self.ord.append(k + float(fr))
            self.idx[(k, fr)] = r
            self._fill(r, S)
        return r


def _run(asm, states, rep, names, NP, NB, bed, pool, log, lap, T, max_mm, cap):
    stats = rep["stats"] = {}
    # 2 world transforms per state (per joint), joint limits
    jnames = list(asm.joints)
    jcol = {j: k + 1 for k, j in enumerate(jnames)}
    SS = _States(asm, states, jcol, rep)
    body = np.array([jcol.get(asm.parts[n].joint, 0) for n in names], int)
    place = np.array([np.asarray(asm.parts[n].place, float) for n in names]).reshape(NP, 4, 4)

    def W(p, sel=slice(None)):
        return SS.JW[sel, body[p]] @ place[p]

    lap("world")
    # chunks and boxes (local frames)
    _CTX = {"bed": bed, "ch": [], "cb": [], "box": []}
    globals()["_CTX"] = _CTX
    for n in names:
        s = asm.parts[n].solid
        lst = _chunks(s) if s.status() == m3.Error.NoError else [s]
        _CTX["ch"].append(lst)
        _CTX["cb"].append(np.array([_bounds(x) for x in lst]).reshape(-1, 2, 3))
        _CTX["box"].append(_bounds(s))
    ntri = [np.array([x.num_tri() for x in lst]) for lst in _CTX["ch"]]
    stats["chunked_parts"] = sum(1 for lst in _CTX["ch"] if len(lst) > 1)
    stats["chunks"] = sum(len(lst) for lst in _CTX["ch"])
    stats["triangles"] = int(sum(t.sum() for t in ntri))
    cen = [(b[0] + b[1]) / 2 if np.all(np.isfinite(b)) else np.zeros(3) for b in _CTX["box"]]
    rho = [float(np.linalg.norm(b[1] - b[0]) / 2) if np.all(np.isfinite(b)) else 0.0 for b in _CTX["box"]]
    lap("chunks")
    # world AABBs of every part at every given state (loose: local box under the world transform)
    base = np.arange(NB)

    def boxes(p, rows):
        Wp = W(p, rows)
        lo, hi = _xbox(_CTX["box"][p][None], Wp[:, :3, :3], Wp[:, :3, 3])
        return lo[:, 0], hi[:, 0]

    LO = np.empty((NP, NB, 3))
    HI = np.empty((NP, NB, 3))
    CEN = np.empty((NP, NB, 3))
    for p in range(NP):
        LO[p], HI[p] = boxes(p, base)
        Wp = W(p, base)
        CEN[p] = np.einsum("kij,j->ki", Wp[:, :3, :3], cen[p]) + Wp[:, :3, 3]
    # 3 collisions: candidates by AABB, grouped by the rounded relative pose, chunk pairs culled by boxes
    allow_sets = _matcher(asm.allowed, names)
    allow_of = {}
    for ia, a in enumerate(names):
        for b in names[ia + 1:]:
            al = [x for x, (A, B) in zip(asm.allowed, allow_sets) if (a in A and b in B) or (b in A and a in B)]
            # the most restrictive matching Allow decides (None = unlimited), independent of module order
            allow_of[(a, b)] = min(al, key=lambda x: math.inf if x.max_vol is None else x.max_vol) if al else None

    def cr(p):
        """Bounding sphere (centre, radius) of part p (or of a cropped stand-in) in its local frame."""
        while len(cen) <= p:
            bx = _CTX["box"][len(cen)]
            ok = np.all(np.isfinite(bx))
            cen.append((bx[0] + bx[1]) / 2 if ok else np.zeros(3))
            rho.append(float(np.linalg.norm(bx[1] - bx[0]) / 2) if ok else 0.0)
        return cen[p], rho[p]

    # joints that move each part (its joint and that joint's parents) and their step limits
    lim = np.array([MAX_STEP_DEG if asm.joints[j].kind == "rev" else MAX_STEP_MM if asm.joints[j].kind == "slide"
                    else np.inf for j in jnames])

    def chain(p):
        out, j, seen = [], asm.parts[names[p]].joint, set()
        while j in asm.joints and j not in seen:
            seen.add(j)
            out.append(jcol[j] - 1)
            j = asm.joints[j].parent
        return out

    def joint_steps(a, b):
        """Per timeline interval: splits needed so no joint moving a or b steps more than 1 deg / 0.5 mm."""
        js = sorted(set(chain(a) + chain(b)))
        if not js or T < 2:
            return np.ones(max(0, T - 1))
        d = np.abs(np.diff(SS.V[:T, js], axis=0)) / lim[js]
        return np.maximum(1.0, d.max(axis=1))

    def pair_scan(a, b, pad, use=None):
        """Given states where the boxes of a and b are within pad, and the timeline intervals in which their relative
        pose moves some point of a or b by more than max_mm while the boxes are within pad + that motion."""
        d0 = _boxdist(LO[a], HI[a], LO[b], HI[b])
        rows = np.nonzero(d0 <= pad)[0]
        if T < 2:
            return rows, None, None
        rel = _rel(W(a, base[:T]), W(b, base[:T]))
        (ca, ra), (cb, rb) = cr(use[0] if use else a), cr(use[1] if use else b)     # a crop moves less
        dk = np.minimum(_move(rel[:-1], rel[1:], cb, rb), _move(_inv(rel[:-1]), _inv(rel[1:]), ca, ra))
        # never finer than the old per-joint rule (1 deg / 0.5 mm for the joints that move a or b): sub-states are
        # needed where a point moves more than max_mm AND a joint of the pair steps more than its limit
        dk = np.minimum(dk, joint_steps(a, b) * max_mm)
        # bounding spheres: a part turning about its centre stays in its sphere, so the gap between the spheres can
        # close only by the motion of the centres
        cmove = np.minimum(_move(rel[:-1], rel[1:], cb, 0.0), _move(_inv(rel[:-1]), _inv(rel[1:]), ca, 0.0))
        gap = np.linalg.norm(CEN[a, :T] - CEN[b, :T], axis=1) - rho[a] - rho[b]
        reach = np.maximum(np.minimum(d0[:T - 1], d0[1:T]), np.minimum(gap[:-1], gap[1:]))
        need = np.nonzero((dk > max_mm) & (reach <= pad + cmove))[0]
        return rows, need, dk

    # the shared grid: interval k splits into nglob[k] steps by the old global rule (no joint more than 1 deg /
    # 0.5 mm); a pair uses every stride-th grid point, so pairs share sub-states and never exceed the global count
    nraw = np.ceil(np.abs(np.diff(SS.V[:T], axis=0)) / lim).max(axis=1, initial=1.0) \
        if T >= 2 and len(lim) else np.ones(max(0, T - 1))
    for k in np.nonzero(nraw > cap)[0]:
        rep["errors"].append(f"refinement capped: a joint moves {int(nraw[k])} steps between states {k} and {k + 1} "
                             f"(phase {states[k].get('phase')}); raise cap or split the interval")
    nglob = np.minimum(cap, nraw).astype(int)

    def substates(a, b, pad, need, dk, use=None):
        """Rows of the sub-states pair (a, b) needs (no point of a or b moves more than max_mm relative to the other
        per step, on the shared grid) where its boxes are within pad."""
        out = []
        for k in need:
            ng = max(1, int(nglob[k]))
            n = min(ng, int(math.ceil(dk[k] / max_mm)))
            if n <= 1:
                continue
            stride = max(1, ng // n)
            out += [SS.sub(k, Fraction(i, ng)) for i in range(stride, ng, stride)]
        if not out:
            return np.zeros(0, int)
        out = np.unique(np.array(out, int))
        lo_a, hi_a = boxes(a, out)
        lo_b, hi_b = boxes(b, out)
        return out[_boxdist(lo_a, hi_a, lo_b, hi_b) <= pad]

    def timeorder(rows):
        o = np.asarray(SS.ord)[rows]
        return rows[np.argsort(o, kind="stable")]

    def groups(a, b, rows):
        """Distinct rounded relative poses of a, b over the given rows (time order): (rep rows, rel matrices, row
        lists)."""
        rel = _rel(W(a, rows), W(b, rows))
        key = np.round(rel[:, :3, :].reshape(len(rel), 12), 4) + 0.0
        _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
        inv = inv.reshape(-1)
        order = np.argsort(first)                  # groups in order of first occurrence
        rank = np.empty_like(order)
        rank[order] = np.arange(len(order))
        g = rank[inv]
        srt = np.argsort(g, kind="stable")
        bounds = np.searchsorted(g[srt], np.arange(len(order) + 1))
        lists = [rows[srt[bounds[k]:bounds[k + 1]]] for k in range(len(order))]
        return rows[first[order]], rel[first[order]], lists

    # motion envelopes: a part is static (same world pose at every state), or moves on a rev / slide joint whose
    # frame before the joint motion is the same at every state (then its every pose lies in a known prism), or None
    def const(M):
        return bool(np.abs(M - M[:1]).max() <= 1e-9)

    envc = {}

    def pre_frame(p, rows):
        j = asm.joints.get(asm.parts[names[p]].joint)
        return SS.JW[rows, jcol.get(j.parent, 0)] @ j.local(0.0)

    def envelope(p):
        if p in envc:
            return envc[p]
        out = None
        Wp = W(p, base)
        if const(Wp):
            out = ("static", Wp[0], None, None)
        else:
            j = asm.joints.get(asm.parts[names[p]].joint)
            if j is not None and j.kind in ("rev", "slide"):
                Fpre = pre_frame(p, base)
                if const(Fpre):
                    Q = tf.frame((0, 0, 0), j.axis)
                    F = Fpre[0] @ Q
                    G = tf.inv(Q) @ place[p]                  # part frame -> motion frame at joint value 0
                    u = tf.apply(G, _verts(asm.parts[names[p]].solid))
                    travel = (0.0, 0.0)
                    if j.kind == "slide":
                        d = np.einsum("i,ki->k", np.asarray(j.axis), (tf.inv(Fpre[0]) @ SS.JW[base, body[p]])[:, :3, 3])
                        travel = (float(d.min()), float(d.max()))
                    E = _env_solid(j.kind, u, travel)
                    if E is not None:
                        out = (j.kind, F, G, E, travel)
        envc[p] = out
        return out

    def pieces(p, M, E):
        """Chunks of part p cut by solid E given in p's frame through M (only chunks whose boxes meet E's)."""
        Ep = prim.place(E, M)
        eb = _bounds(Ep)
        cb = _CTX["cb"][p]
        sel = np.nonzero(np.all(cb[:, 0] <= eb[1] + BOX_PAD, axis=1) & np.all(eb[0] <= cb[:, 1] + BOX_PAD, axis=1))[0]
        out = []
        for i in sel:
            x = _CTX["ch"][p][i] ^ Ep
            if not x.is_empty() and x.num_tri():
                out.append(x)
        return out

    def add_virtual(lst):
        _CTX["ch"].append(lst)
        _CTX["cb"].append(np.array([_bounds(x) for x in lst]).reshape(-1, 2, 3))
        _CTX["box"].append(np.array([np.min(_CTX["cb"][-1][:, 0], axis=0), np.max(_CTX["cb"][-1][:, 1], axis=0)]))
        return len(_CTX["ch"]) - 1

    n_proved = n_crop = 0
    env_used = set()

    def envelope_stage(a, b):
        """'clear' when the motion envelopes prove vol <= HIT_VOL at every state, else the (possibly cropped) part
        indices to use for a and b."""
        nonlocal n_proved, n_crop
        ea, eb = envelope(a), envelope(b)
        if ea is None or eb is None or (ea[0] == "static" and eb[0] == "static"):
            return a, b
        env_used.update((a, b))
        if ea[0] != "static" and eb[0] != "static":
            Ea, Eb = prim.place(ea[3], ea[1]), prim.place(eb[3], eb[1])
            if (Ea ^ Eb).volume() <= HIT_VOL:
                n_proved += 1
                return "clear"
            return a, b
        flip = ea[0] != "static"
        s_, m_ = (b, a) if flip else (a, b)                 # static part, moving part
        es, em = envelope(s_), envelope(m_)
        Ms = tf.inv(es[1]) @ em[1]                          # motion frame -> static part frame
        sp = pieces(s_, Ms, em[3])                          # static part cut to the moving part's envelope
        if sum(x.volume() for x in sp) <= HIT_VOL:
            n_proved += 1
            return "clear"
        # moving part cut to an envelope of the cut static part that its own motion leaves invariant
        u = np.concatenate([tf.apply(tf.inv(Ms), _verts(x)) for x in sp])
        C = _env_solid("rev" if em[0] == "rev" else "zprism", u)
        mp = pieces(m_, tf.inv(em[2]), C)
        n_crop += 1
        if not mp:
            n_proved += 1
            return "clear"
        vs, vm = add_virtual(sp), add_virtual(mp)
        return (vm, vs) if flip else (vs, vm)

    # pass 1: candidate pairs, envelope proofs and crops, per-pair sub-states (before the pool forks)
    todo = []
    n_cand = 0
    for a in range(NP - 1):
        bs = np.arange(a + 1, NP)
        dmin = np.array([float(_boxdist(LO[a], HI[a], LO[b], HI[b]).min()) for b in bs]) if T >= 2 else None
        if T < 2:
            m = np.all(LO[a][None] <= HI[bs], axis=2) & np.all(LO[bs] <= HI[a][None], axis=2)
            near = bs[m.any(axis=1)]
        else:
            near = bs                        # moving pairs may need sub-states even when apart at every given state
        for b in near:
            if T >= 2 and dmin[b - a - 1] > 0 and body[a] == body[b]:
                continue                      # same body, apart at every state
            rows, need, dk = pair_scan(a, b, BOX_PAD)
            if not len(rows) and (need is None or not len(need)):
                continue
            use = envelope_stage(a, b) if (len(rows) > 1 or (need is not None and len(need))) else (a, b)
            if use == "clear":
                continue
            if use != (a, b) and need is not None and len(need):
                rows, need, dk = pair_scan(a, b, BOX_PAD, use)
            if need is not None and len(need):
                rows = np.concatenate([rows, substates(a, b, BOX_PAD, need, dk, use)])
            if len(rows):
                rows = timeorder(rows)
                n_cand += len(rows)
                todo.append((a, b, use, rows))
    stats.update(envelope_cleared_pairs=n_proved, cropped_pairs=n_crop, substates=len(SS.S) - NB)
    log(f"audit: {NP} parts, {NB} states + {len(SS.S) - NB} pair sub-states, {len(todo)} pairs to check exactly")
    lap("broad")
    ntri += [np.array([x.num_tri() for x in lst]) for lst in _CTX["ch"][len(ntri):]]
    # 1 static (in the pool; same order as names)
    res = {}
    for out in pool.imap(_w_static, _batches(list(enumerate(names)), lambda kn: max(2000, int(ntri[kn[0]].sum())))):
        res.update(out)
    rep["static"] = [res[k] for k in range(NP)]
    lap("static")

    def vcost(it):
        a, b, pairs = it[1], it[2], it[4]
        return int(ntri[a][pairs[:, 0]].sum() + ntri[b][pairs[:, 1]].sum()) + 500 * len(pairs)

    # pass 2: exact checks, streamed in rounds so memory stays bounded
    hits = {}           # (a, b) -> list of (row, vol)
    n_pose = n_items = n_cp = 0
    per = {}
    prog = _Progress(log, "exact pairs", len(todo))
    items = []

    def flush():
        work = [(n, u[0], u[1], rel, pairs) for n, (a, b, _, rel, pairs, _, u) in enumerate(items)]
        for out in pool.imap(_w_volume, _batches(work, vcost)):
            for iid, v in out:
                if v > HIT_VOL:
                    a, b, lst = items[iid][0], items[iid][1], items[iid][5]
                    hits.setdefault((a, b), []).extend((int(i), v) for i in lst)
        items.clear()

    for t_i, (a, b, use, rows) in enumerate(todo):
        reps, rels, lists = groups(a, b, rows)
        n_pose += len(reps)
        k, i, j, _ = _cull(use[0], use[1], rels, BOX_PAD)
        if len(k):
            srt = np.argsort(k, kind="stable")
            k, i, j = k[srt], i[srt], j[srt]
            cuts = np.searchsorted(k, np.arange(len(reps) + 1))
            x = per.setdefault((names[a], names[b]), [0, 0])
            for g in range(len(reps)):
                if cuts[g + 1] > cuts[g]:
                    pairs = np.stack([i[cuts[g]:cuts[g + 1]], j[cuts[g]:cuts[g + 1]]], axis=1)
                    items.append((a, b, int(reps[g]), rels[g], pairs, lists[g], use))
                    n_items += 1
                    n_cp += len(pairs)
                    x[0] += 1
                    x[1] += len(pairs)
        if len(items) >= 20000:
            flush()
        prog.step(t_i + 1)
    if items:
        flush()
    stats.update(candidate_pair_states=int(n_cand), distinct_poses=int(n_pose), exact_items=n_items,
                 exact_chunk_pairs=n_cp)
    stats["busiest_pairs"] = [dict(a=a, b=b, poses=x[0], chunk_pairs=x[1])
                              for (a, b), x in sorted(per.items(), key=lambda kv: -kv[1][1])[:12]]
    hits = {(names[a], names[b]): lst for (a, b), lst in hits.items()}
    lap("exact")
    # 4 clearances: exact minimum over all states by branch and bound
    idx = {n: k for k, n in enumerate(names)}
    n_gap = 0
    clear_rows = {}
    for c, (CA, CB) in zip(asm.clearances, _matcher(asm.clearances, names)):
        pairs = [(a, b) for ia, a in enumerate(names) for b in names[ia + 1:] if (a in CA and b in CB) or (b in CA and a in CB)]
        if not pairs:
            rep["clearances"].append(dict(a=c.a, b=c.b, min_mm=c.min_mm, gap=None, ok=False, why=c.why,
                                          note="no parts match this clearance"))
            continue
        search = max(5.0, 3 * c.min_mm)
        cand = []          # (lower bound, order, a, b, rel, chunk pairs, lower bounds)
        for pi, (an, bn) in enumerate(pairs):
            a, b = idx[an], idx[bn]
            st, need, dk = pair_scan(a, b, search)
            if need is not None and len(need):
                st = np.concatenate([st, substates(a, b, search, need, dk)])
            if not len(st):
                continue           # farther than search at every state: min_gap would return search
            reps, rels, _ = groups(a, b, timeorder(st))
            k, i, j, lb = _cull(a, b, rels, search)
            srt = np.lexsort((lb, k))
            k, i, j, lb = k[srt], i[srt], j[srt], lb[srt]
            cuts = np.searchsorted(k, np.arange(len(reps) + 1))
            for g in range(len(reps)):
                s0, s1 = cuts[g], cuts[g + 1]
                if s1 > s0:
                    cand.append((float(lb[s0:s1].min()), (pi, SS.ord[int(reps[g])], int(reps[g])), a, b, rels[g],
                                 np.stack([i[s0:s1], j[s0:s1]], axis=1), lb[s0:s1]))
        # lower bound first (ties within 1e-6 by state order, so equal gaps report the earliest state)
        cand.sort(key=lambda x: (round(x[0], 6), x[1]))
        best_g, best_o = search, (0, 0.0, 0)       # what min_gap returns for pairs farther than search
        pos, step = 0, max(4, 4 * pool.jobs)
        prog = _Progress(log, f"clearance {c.a} / {c.b}", len(cand))
        while pos < len(cand) and round(cand[pos][0], 6) - 2e-6 < best_g:      # later bounds are all >= best_g
            batch = []
            while pos < len(cand) and round(cand[pos][0], 6) - 2e-6 < best_g and len(batch) < step:
                lbm, o, a, b, rel, cp, lbs = cand[pos]
                if lbm < best_g - 1e-9:            # otherwise its gap cannot be below the best one
                    batch.append((pos, a, b, rel, cp, lbs, best_g + 1e-9, -1.0))
                pos += 1
            if not batch:
                continue
            n_gap += len(batch)
            per_ = max(1, len(batch) // max(1, pool.jobs))
            for out in pool.imap(_w_gap, [batch[x:x + per_] for x in range(0, len(batch), per_)]):
                for q, g in out:
                    o = cand[q][1]
                    if g < best_g - 1e-12 or (g <= best_g + 1e-12 and o < best_o):
                        best_g, best_o = g, o
            prog.step(pos)
        g, (pi, _, i) = best_g, best_o
        a, b = pairs[pi]
        hit = [(x, y) for x, y in pairs if (x, y) in hits or (y, x) in hits]
        if hit:
            g, (a, b) = 0.0, hit[0]
        rep["clearances"].append(dict(a=a, b=b, rule=f"{c.a} / {c.b}", min_mm=c.min_mm, gap=round(g, 3),
                                      ok=(g >= c.min_mm and not hit), worst_state=i, phase=SS.S[i].get("phase"),
                                      why=c.why, note="parts intersect" if hit else ""))
    stats["clearance_evals"] = n_gap
    lap("clearance")
    # every audited state in time order; rows -> report indices
    order = np.argsort(np.asarray(SS.ord), kind="stable")
    ix = np.empty(len(order), int)
    ix[order] = np.arange(len(order))
    rep["n_states"] = len(SS.S)
    stats["given_states"] = NB
    for e in rep["limits"]:
        e["state"] = int(ix[e["state"]])
    rep["limits"].sort(key=lambda e: e["state"])
    for e in rep["clearances"]:
        if e.get("worst_state") is not None:
            e["worst_state"] = int(ix[e["worst_state"]])
    for key in hits:
        hits[key] = sorted(((int(ix[r]), v) for r, v in hits[key]), key=lambda x: x[0])
    # the envelope proofs assumed a constant joint frame over the given states: confirm at the sub-states
    if len(SS.S) > NB:
        sub = np.arange(NB, len(SS.S))
        for p in sorted(env_used):
            e = envc.get(p)
            if e is None:
                continue
            bad = (not np.allclose(W(p, sub), e[1], atol=1e-6)) if e[0] == "static" else \
                (not np.allclose(pre_frame(p, sub) @ tf.frame((0, 0, 0), asm.joints[asm.parts[names[p]].joint].axis),
                                 e[1], atol=1e-6))
            if not bad and e[0] == "slide":
                j = asm.joints[asm.parts[names[p]].joint]
                d = np.einsum("i,ki->k", np.asarray(j.axis), (tf.inv(e[1] @ tf.inv(tf.frame((0, 0, 0), j.axis)))
                                                               @ SS.JW[sub, body[p]])[:, :3, 3])
                bad = d.min() < e[4][0] - 1e-6 or d.max() > e[4][1] + 1e-6
            if bad:
                rep["errors"].append(f"audit: motion envelope of {names[p]} does not hold at interpolated states; "
                                     f"rerun with --global-refine")
    for (a, b), lst in sorted(hits.items()):
        vmax = max(v for _, v in lst)
        imax = max(lst, key=lambda x: x[1])[0]
        al = allow_of[(a, b)]
        S_ = SS.S[int(order[imax])]
        entry = dict(a=a, b=b, max_vol_mm3=round(vmax, 3), n_states=len(lst), worst_state=imax,
                     phase=S_.get("phase"), t=S_.get("t"))
        if al is not None and (al.max_vol is None or vmax <= al.max_vol):
            entry["why"] = al.why
            rep["allowed_contacts"].append(entry)
        else:
            if al is not None:
                entry["note"] = f"allowed up to {al.max_vol} mm3 ({al.why})"
            rep["collisions"].append(entry)
    used = {id(allow_of[(e["a"], e["b"])]) for e in rep["allowed_contacts"] + rep["collisions"] if allow_of.get((e["a"], e["b"]))}
    for al in asm.allowed:
        if id(al) not in used:
            rep["warns"].append(f"Allow({al.a!r}, {al.b!r}) never matched an overlap: remove it or narrow it")
    # 5 floating (rest state = first state)
    adj = {n: set() for n in names}
    fwork, fmeta = [], []
    W0 = [W(p, slice(0, 1)) for p in range(NP)]
    for a in range(NP - 1):
        for b in range(a + 1, NP):
            if not (np.all(LO[a, 0] <= HI[b, 0] + TOUCH_GAP) and np.all(LO[b, 0] <= HI[a, 0] + TOUCH_GAP)):
                continue
            an, bn = names[a], names[b]
            if any(i == 0 for i, _ in hits.get((an, bn), [])):
                adj[an].add(bn); adj[bn].add(an)
                continue
            rel = _rel(W0[a], W0[b])
            _, i, j, lb = _cull(a, b, rel, TOUCH_GAP)
            if not len(i):
                continue
            srt = np.argsort(lb, kind="stable")
            fwork.append((len(fmeta), a, b, rel[0], np.stack([i[srt], j[srt]], axis=1), lb[srt], 1.0, TOUCH_GAP))
            fmeta.append((an, bn))
    for out in pool.imap(_w_gap, _batches(fwork, lambda it: vcost((None, it[1], it[2], None, it[4][:4])))):
        for q, g in out:
            if g <= TOUCH_GAP:
                an, bn = fmeta[q]
                adj[an].add(bn); adj[bn].add(an)
    grounded, declared = set(), False
    for sub, mod in asm.modules.items():
        if hasattr(mod, "grounded"):
            declared = True
            for pat in mod.grounded(asm.P):
                m = {n for n in names if fnmatchcase(n, pat)}
                if not m and any(fnmatchcase(d, pat) for d in getattr(asm, "dropped", ())):
                    continue                        # show build: the grounded part was left out (drop hook)
                if not m:
                    rep["errors"].append(f"{sub}.grounded pattern {pat!r} matches no part")
                grounded |= m
    if not declared:
        rep["warns"].append("no subsystem declares grounded(P): connection to the ground is not checked")
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
            if n not in reach and adj[n]:          # parts touching nothing are already listed
                rep["floating"].append(dict(part=n, why="not connected to a grounded part"))
    rep["contact_graph"] = {n: sorted(adj[n]) for n in names}
    lap("floating")
    # 6 subsystem checks
    for sub, mod in asm.modules.items():
        if hasattr(mod, "checks"):
            for msg in mod.checks(asm.P, asm) or []:
                rep["checks"].append(dict(subsystem=sub, fail=msg))
    lap("checks")


def refine(asm: Assembly, states: list[dict], max_deg=MAX_STEP_DEG, max_mm=MAX_STEP_MM, cap=100000) -> list[dict]:
    """Insert interpolated states so that no joint moves more than max_deg (rev) or max_mm (slide) between
    consecutive states. Joint values are interpolated directly (S["_joints"]); numeric S values linearly, others
    from the earlier state (for display). Raises ValueError if an interval needs more than cap steps."""
    if len(states) < 2:
        return list(states)
    vals = [asm.values(S) for S in states]
    out = [states[0]]
    for k in range(1, len(states)):
        A, B, va, vb = states[k - 1], states[k], vals[k - 1], vals[k]
        n = 1
        for jn, j in asm.joints.items():
            d = abs(vb[jn] - va[jn])
            lim = max_deg if j.kind == "rev" else max_mm if j.kind == "slide" else math.inf
            n = max(n, int(math.ceil(d / lim)))
        if n > cap:
            raise ValueError(f"refine: {n} steps needed between states {k - 1} and {k} (cap {cap})")
        for i in range(1, n):
            u = i / n
            S = _interp(A, B, u)
            if _discrete_change(A, B):
                S["_joints"] = {jn: va[jn] + (vb[jn] - va[jn]) * u for jn in asm.joints}
            out.append(S)
        out.append(B)
    return out


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
    for f in rep.get("mesh_fails", []):
        L.append(f"  MESH {f}")
    if rep.get("meshes"):
        L.append(f"  gear meshes checked: {len(rep['meshes'])}, failing: {sum(not m['ok'] for m in rep['meshes'])}")
    for w in rep.get("warns", []):
        L.append(f"  warn {w}")
    for r in rep["static"]:
        for w in r.get("warns", []):
            L.append(f"  warn {r['name']}: {w}")
    L.append(f"  allowed contacts: {len(rep['allowed_contacts'])}")
    L.append("CLEAN" if rep["clean"] else "NOT CLEAN")
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--only", default="")
    ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--no-refine", action="store_true", help="audit the raw timeline samples only")
    ap.add_argument("--max-mm", type=float, default=MAX_STEP_MM,
                    help="per-pair refinement: largest point motion between audited states (mm)")
    ap.add_argument("--global-refine", action="store_true",
                    help="refine the whole timeline (no joint more than 1 deg / 0.5 mm a step) instead of per pair")
    a = ap.parse_args(argv)
    P = load_params()
    asm = Assembly.load(P, [s for s in a.only.split(",") if s] or None)
    states = timeline(P, a.dt)
    n_tl = 0
    if a.global_refine and not a.no_refine:
        states = refine(asm, states)
    elif not a.no_refine:
        n_tl = len(states)          # each pair refines the timeline for itself (sub-states only where it moves)
    states += asm.extra_states
    rep = run(asm, states, a.jobs, pair_refine=n_tl, max_mm=a.max_mm)
    rep["settings"] = dict(dt=a.dt, max_mm=a.max_mm, jobs=a.jobs, refine="global" if a.global_refine else
                           "none" if a.no_refine else "pair")
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
