"""GrappStar mission animation: ENVIRONMENT module.

build_env(scene) builds Harry's paddock around the world origin (centre of the base station's board top, launch
toward +X, landing patch x 3.5..6 m): rolling uneven terrain with flat-ish zones for the physics, bare dirt, a worn
horse track, hoof-poached mud at the gate and trough, geometry-nodes grass (mixed blade lengths, clumps, seed heads,
dry blades, clover, plantain, dandelion, buttercups) at full density near the action and thinning with distance,
weathered post-and-rail fencing with a timber field gate, a cross-country log jump, fallen logs, a stump, a round
hay bale and a galvanised trough, a CC0 Poly Haven HDRI sky ("Evening Meadow": late afternoon, sun 21 deg,
hedges and fruit trees all round) with a matching sun lamp (physical sky fallback), and render settings via
set_render(scene, engine, preset).

Assets: render/assets/fetch_env.py downloads them to render/assets/cache (gitignored). Everything falls back to
procedural materials if the cache is missing. Run `render/.venv/bin/python render/env.py` for the preview stills
render/out/preview_env.png, preview_env_low.png, preview_env_wide.png.

Other modules can use terrain_height(x, y) (numpy, metres) to sit things on the ground, sun_direction(), and
build_env(scene, quality, clear_zones=[(x, y, r)], board_clear=True, board_half=(hx, hy)) to keep grass out of
the board footprint and from wherever the dart comes to rest. Grass is one object, env:grass (a vertex cloud whose
GN modifier instances the clump sources in collection ENV_SRC, excluded from the view layer); hide it or switch
GRASS_QUALITY ("draft", "preview", "final", "hero") to trade quality for speed.
"""
import math
import os
import sys
import time

import numpy as np
import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.environ.get("GRAPPSTAR_ENV_CACHE", os.path.join(HERE, "assets", "cache"))   # override for tests
OUT = os.path.join(HERE, "out")

SEED = 1907

# ---------------------------------------------------------------------------------------------------------------
# Quality presets. clumps_per_m2: grass clump instances per m2 inside r_near of GRASS_CENTER (a clump is 10 to
# 60 blades); density eases to mid x that by r_full, then to far_min x at r_max where the grass stops (textured
# ground beyond);
# far_min: density factor reached at r_max; hero: extra density multiplier in HERO_ZONE (base + flight line +
# landing patch, where the close-ups are); clumps use LOD1 (60 % of the blades, wider) beyond r_near and LOD2
# (a third of the blades, twice as wide) beyond lod_r;
# blade_mult: scales blades per clump. "final" is meant for EEVEE on a 16 GB M3 (about 0.2 M instances).
GRASS_PRESETS = {
    "draft":   dict(clumps_per_m2=30, hero=2.0, r_near=5.0, mid=0.7, r_full=9.0, r_max=24.0, far_min=0.15,
                    lod_r=7.0, blade_mult=0.5),
    "preview": dict(clumps_per_m2=60, hero=3.0, r_near=7.0, mid=0.75, r_full=14.0, r_max=36.0, far_min=0.08,
                    lod_r=11.0, blade_mult=0.75),
    "final":   dict(clumps_per_m2=90, hero=3.2, r_near=9.0, mid=0.75, r_full=18.0, r_max=46.0, far_min=0.06,
                    lod_r=14.0, blade_mult=1.0),
    "hero":    dict(clumps_per_m2=120, hero=4.5, r_near=11.0, mid=0.8, r_full=22.0, r_max=55.0, far_min=0.07,
                    lod_r=17.0, blade_mult=1.15),
}
GRASS_QUALITY = "final"
GRASS_DENSITY = GRASS_PRESETS[GRASS_QUALITY]      # alias for the active preset
GRASS_CENTER = (2.5, 0.0)                          # action centre: between the base and the landing patch
HERO_ZONE = (-2.6, 7.8, 2.4)                       # x0, x1, half width y: extra-dense grass for close-ups

# ---------------------------------------------------------------------------------------------------------------
# Layout (world metres). Flat zones per SPEC: base and landing patch within +-2 cm of z 0.
FLAT_CORRIDOR = (-1.6, 6.8, 1.35)       # x0, x1, half width in y: kept within +-1.5 cm (flight line + landing)
BOARD_HALF = (0.65, 0.50)               # grass-free pad under the base board (half sizes, m)
BOARD_THICKNESS = 0.015                 # ground under the board sits this much below z 0
BOARD_CLEAR = True                      # keep grass from poking through the board (build_env(board_clear=...))
TERRAIN_CENTER = (5.0, 0.0)
TERRAIN_DENSE_HALF = 24.0               # dense mesh square half size (0.18 m grid), coarser out to TERRAIN_FAR
TERRAIN_STEP = 0.18
TERRAIN_FAR = 420.0

FENCE_A_Y = 11.5                        # back fence along x
FENCE_A_X = (-17.0, 30.0)
FENCE_B_X = -17.0                       # side fence along y
FENCE_B_Y = (-26.0, 11.5)
GATE_X = (7.0, 10.7)                    # field gate in fence A (hanging post, latch post)
POST_SPACING = 2.4
RAIL_HEIGHTS = (0.40, 0.79, 1.17)
TROUGH_XY = (-6.0, 10.35)
BALE_XY = (-12.6, 7.4)
LOG_JUMP = dict(xy=(14.6, 1.9), yaw=78.0, length=4.6, dia=0.58, sink=0.07)
FALLEN_LOGS = [dict(xy=(-7.6, -6.2), yaw=22.0, length=3.4, dia=0.36, sink=0.05, stub=True),
               dict(xy=(9.6, -7.9), yaw=-38.0, length=2.3, dia=0.27, sink=0.07, stub=False)]
STUMP = dict(xy=(-14.9, -9.6), dia=0.56, h=0.42)

# ---------------------------------------------------------------------------------------------------------------
# Sky / sun. Poly Haven "Evening Meadow" (CC0): low sun (20.8 deg), partly cloudy, fruit trees, hedges and
# distant tree lines all round. Values measured from the 4K file (brightest pixel; integral of the sun region
# above luminance 50 = sun irradiance at world strength 1).
HDRI_NAME = "evening_meadow"
HDRI_SUN_U, HDRI_SUN_V = 0.60046, 0.61548
HDRI_SUN_IRR = 1.35
HDRI_SUN_RGB = (1.0, 0.88, 0.74)        # late-afternoon warm (the HDRI's sun pixels are clipped white)
HDRI_CLAMP = 60.0                       # removes the sun disc from the HDRI; the sun lamp replaces it
SUN_AZIMUTH_DEG = 158.0                 # direction TO the sun, CCW from +X: behind the launcher, a bit to +Y,
#                                         so a side view from -Y gets light from behind-left and the HDRI's fruit
#                                         trees and far tree line sit behind the paddock
SUN_STRENGTH = 4.5                      # sun lamp W/m2 (measured HDRI sun is 1.35 x world strength; boosted
#                                         because the HDRI sun sits in thin cloud and reads flat)
SUN_ANGLE_DEG = 1.5
WORLD_STRENGTH = 0.65
EXPOSURE = 0.0

TEX_SIZE = {"forrest_ground_01": 2.0, "brown_mud_02": 1.3, "brown_mud_03": 1.3, "rough_wood": 0.5,
            "bark_brown_02": 1.0, "Grass004": 1.4}


# =============================================================================================================
# noise (numpy, deterministic)
class _Noise:
    _G = np.array([[1, 1], [-1, 1], [1, -1], [-1, -1], [1.41, 0], [-1.41, 0], [0, 1.41], [0, -1.41]]) / 1.41

    def __init__(self, seed):
        p = np.random.RandomState(seed).permutation(256)
        self.p = np.concatenate([p, p])

    def perlin(self, x, y):
        x = np.asarray(x, float)
        y = np.asarray(y, float)
        xi = np.floor(x).astype(np.int64)
        yi = np.floor(y).astype(np.int64)
        xf, yf = x - xi, y - yi
        xi &= 255
        yi &= 255
        p = self.p

        def g(ix, iy, dx, dy):
            h = p[p[ix] + iy] & 7
            G = self._G[h]
            return G[..., 0] * dx + G[..., 1] * dy

        u = xf * xf * xf * (xf * (xf * 6 - 15) + 10)
        v = yf * yf * yf * (yf * (yf * 6 - 15) + 10)
        n00 = g(xi, yi, xf, yf)
        n10 = g(xi + 1, yi, xf - 1, yf)
        n01 = g(xi, yi + 1, xf, yf - 1)
        n11 = g(xi + 1, yi + 1, xf - 1, yf - 1)
        a = n00 + u * (n10 - n00)
        b = n01 + u * (n11 - n01)
        return (a + v * (b - a)) * 1.4

    def fbm(self, x, y, octaves=3, lac=2.03, gain=0.5):
        tot = 0.0
        amp = 1.0
        norm = 0.0
        for o in range(octaves):
            tot = tot + amp * self.perlin(x * lac ** o + 17.3 * o, y * lac ** o - 9.1 * o)
            norm += amp
            amp *= gain
        return tot / norm


_NZ = {}


def _nz(k):
    if k not in _NZ:
        _NZ[k] = _Noise(SEED * 31 + k)
    return _NZ[k]


def _sstep(e0, e1, x):
    t = np.clip((np.asarray(x, float) - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3 - 2 * t)


def _box_sdf(x, y, x0, x1, hy):
    cx, hx = 0.5 * (x0 + x1), 0.5 * (x1 - x0)
    dx = np.abs(x - cx) - hx
    dy = np.abs(y) - hy
    return np.hypot(np.maximum(dx, 0), np.maximum(dy, 0)) + np.minimum(np.maximum(dx, dy), 0)


def _poly_dist(x, y, pts):
    d = np.full(np.shape(x), 1e9)
    for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
        vx, vy = bx - ax, by - ay
        t = np.clip(((x - ax) * vx + (y - ay) * vy) / (vx * vx + vy * vy), 0, 1)
        d = np.minimum(d, np.hypot(x - ax - t * vx, y - ay - t * vy))
    return d


def _ellipse(x, y, cx, cy, rx, ry, rough=0.25, k=40):
    e = np.hypot((x - cx) / rx, (y - cy) / ry) + rough * _nz(k).fbm(x / 1.1, y / 1.1, 2)
    return 1.0 - _sstep(0.72, 1.05, e)


TRACKS = [
    [(FENCE_B_X + 1.7, FENCE_A_Y - 1.5), (-6.0, FENCE_A_Y - 1.2), (4.0, FENCE_A_Y - 1.5), (16.0, FENCE_A_Y - 1.25),
     (FENCE_A_X[1], FENCE_A_Y - 1.4)],
    [(8.8, FENCE_A_Y - 0.4), (10.2, 6.6), (12.4, 1.6), (15.6, -5.0), (18.8, -14.0), (22.0, -27.0)],
    [(FENCE_B_X + 1.4, FENCE_A_Y - 1.7), (FENCE_B_X + 1.6, -4.0), (FENCE_B_X + 1.3, FENCE_B_Y[0])],
]


def _fields(x, y):
    """All terrain / vegetation fields at world (x, y). Returns dict of arrays."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    cx, cy = GRASS_CENTER
    x0, x1, hy = FLAT_CORRIDOR
    sd = _box_sdf(x, y, x0, x1, hy)
    flat = 1.0 - _sstep(0.0, 2.2, sd)                    # 1 in the corridor
    flat_w = 1.0 - _sstep(1.5, 10.0, sd)                 # rolling relief fades in over ~10 m
    pad = np.clip(1.0 - _sstep(0.0, 0.35, _box_sdf(x, y, -BOARD_HALF[0], BOARD_HALF[0], BOARD_HALF[1])), 0, 1)
    tdx = np.maximum(np.abs(x - TERRAIN_CENTER[0]), np.abs(y - TERRAIN_CENTER[1]))
    detail = 1.0 - _sstep(TERRAIN_DENSE_HALF - 4.0, TERRAIN_DENSE_HALF + 1.0, tdx)

    def roll_fn(px, py):
        return 0.85 * _nz(1).fbm(px / 42.0 + 3.1, py / 42.0 - 7.7, 3) + 0.22 * _nz(2).fbm(px / 13.0, py / 13.0, 2)

    roll = roll_fn(x, y) - roll_fn(np.array(cx), np.array(cy))
    d = np.hypot(x - cx, y - cy)
    rise = 0.0045 * np.maximum(d - 55.0, 0.0) ** 1.08
    hum = (0.048 * _nz(3).fbm(x / 2.3, y / 2.3, 3) + 0.011 * _nz(4).fbm(x / 0.62, y / 0.62, 2))

    # worn horse tracks
    td = np.full(x.shape, 1e9)
    for pts in TRACKS:
        td = np.minimum(td, _poly_dist(x, y, pts))
    td = td + 0.10 * _nz(5).fbm(x / 0.9, y / 0.9, 2)
    patchy = _sstep(-0.25, 0.25, _nz(12).fbm(x / 2.2, y / 2.2, 2))            # worn bare in places only
    track = (1.0 - _sstep(0.08, 0.30, td)) * (0.15 + 0.7 * patchy)           # bare core
    track_edge = 1.0 - _sstep(0.30, 0.85, td)            # trampled, sparse edge
    # hoof-poached mud: gate, trough; trampled feeding area at the bale
    mud = np.maximum(_ellipse(x, y, 8.85, FENCE_A_Y - 1.6, 2.9, 1.9, k=41),
                     _ellipse(x, y, TROUGH_XY[0] + 0.2, TROUGH_XY[1] - 0.8, 2.2, 1.4, k=42))
    feed = _ellipse(x, y, BALE_XY[0] + 0.7, BALE_XY[1] - 0.5, 2.7, 2.1, k=43)
    # bare rolling patches, kept out of the flat corridor
    bare = _sstep(0.40, 0.56, _nz(6).fbm(x / 3.4 + 40, y / 3.4, 3)) * (1 - flat) * (1 - _sstep(30, 40, d))
    scuff = _ellipse(x, y, 5.15, -0.42, 0.34, 0.24, rough=0.3, k=44) * 0.75   # hoof scuff by the landing patch
    soil = np.clip(np.maximum.reduce([track, 0.8 * feed, bare, scuff]), 0, 1)

    poach = mud * (0.035 * _nz(7).fbm(x / 0.33, y / 0.33, 2) - 0.02)
    z = (roll * (1 - flat_w) + rise + hum * detail * (1 - 0.8 * flat) * (1 - pad)
         - (0.035 * track + 0.015 * track_edge) * detail + poach * detail - BOARD_THICKNESS * pad)

    # vegetation fields
    fence_d = np.minimum(np.abs(y - FENCE_A_Y) + 1e3 * ((x < FENCE_A_X[0]) | (x > FENCE_A_X[1])),
                         np.abs(x - FENCE_B_X) + 1e3 * ((y < FENCE_B_Y[0]) | (y > FENCE_B_Y[1])))
    near_fence = 1.0 - _sstep(0.35, 0.9, fence_d)
    log_d = np.full(x.shape, 1e9)
    for L in [LOG_JUMP] + FALLEN_LOGS:
        a = math.radians(L["yaw"])
        h = 0.5 * L["length"]
        ex, ey = math.cos(a) * h, math.sin(a) * h
        lx, ly = L["xy"]
        log_d = np.minimum(log_d, _poly_dist(x, y, [(lx - ex, ly - ey), (lx + ex, ly + ey)]) - 0.5 * L["dia"])
    near_log = 1.0 - _sstep(0.05, 0.5, log_d)
    rough = _sstep(0.08, 0.42, _nz(8).fbm(x / 6.5 + 11, y / 6.5, 3))     # tall "roughs" vs grazed lawns
    gh = 0.18 + 0.55 * rough + 0.15 * (_nz(9).fbm(x / 1.7, y / 1.7, 2) + 0.5)
    gh = np.maximum(gh, np.maximum(near_fence, 0.8 * near_log))
    # grazed a little shorter along the flight line and landing patch, organic soft edge (no visible box)
    graze = 1.0 - _sstep(-0.5, 3.0, sd + 1.4 * _nz(14).fbm(x / 2.5, y / 2.5, 2))
    gh = gh + graze * (np.minimum(gh, 0.55) - gh)
    gh = gh * (1 - 0.5 * track_edge) * (1 - 0.6 * mud)
    # grass-free only just inside the board footprint, soft ragged edge (hidden under the board)
    padc = 0.0
    if BOARD_CLEAR:
        bsd = _box_sdf(x, y, -BOARD_HALF[0] + 0.05, BOARD_HALF[0] - 0.05, BOARD_HALF[1] - 0.05)
        padc = 1.0 - _sstep(-0.06, 0.08, bsd + 0.04 * _nz(13).fbm(x / 0.15, y / 0.15, 2))
    dens = (1 - 0.97 * soil) * (1 - 0.92 * mud) * (1 - 0.55 * track_edge) * (1 - 0.6 * feed) * (1 - padc)
    dens = dens * (1 - 0.35 * scuff)
    dry = np.clip(0.18 + 0.35 * _nz(10).fbm(x / 9.0, y / 9.0, 2) + 0.25 * rough + 0.25 * track_edge, 0, 1)
    clover = _sstep(0.12, 0.45, _nz(11).fbm(x / 2.6 + 5, y / 2.6, 2)) * (1 - rough)
    return dict(z=z, soil=soil, mud=mud, track=track_edge, feed=feed, gh=np.clip(gh, 0, 1),
                dens=np.clip(dens, 0, 1), dry=dry, clover=clover, flat=flat, graze=graze, near_fence=near_fence,
                pad=pad, board=1.0 - _sstep(0.0, 0.4, _box_sdf(x, y, -BOARD_HALF[0], BOARD_HALF[0], BOARD_HALF[1])))


def terrain_height(x, y):
    """Ground height (m) at world x, y (scalars or numpy arrays). Matches the terrain mesh within ~2 mm near
    the action (the mesh is a 0.18 m triangulation of this function)."""
    z = _fields(np.atleast_1d(x), np.atleast_1d(y))["z"]
    return float(z[0]) if np.ndim(x) == 0 else z


# =============================================================================================================
# mesh building helpers
class MB:
    """Accumulates vertices, polygons, per-loop UVs, per-vertex float attributes and material indices."""

    def __init__(self):
        self.v, self.f, self.mi, self.uv, self.flat = [], [], [], [], []
        self.n = 0
        self.attr = {}

    def verts(self, arr, **attrs):
        arr = np.asarray(arr, float).reshape(-1, 3)
        base = self.n
        self.v.append(arr)
        self.n += len(arr)
        for k, val in attrs.items():
            self.attr.setdefault(k, []).append((base, np.broadcast_to(np.asarray(val, float), (len(arr),))))
        return base

    def face(self, idx, mat=0, uv=None, flat=False):
        self.f.append(list(idx))
        self.flat.append(flat)
        self.mi.append(mat)
        self.uv.append(uv)

    def build(self, name, mats=(), smooth=False):
        me = bpy.data.meshes.new(name)
        if not self.v:
            return me
        V = np.concatenate(self.v)
        me.vertices.add(len(V))
        me.vertices.foreach_set("co", V.astype(np.float32).ravel())
        sizes = np.array([len(f) for f in self.f])
        loops = np.concatenate([np.asarray(f, np.int64) for f in self.f])
        me.loops.add(len(loops))
        me.loops.foreach_set("vertex_index", loops.astype(np.int32))
        me.polygons.add(len(self.f))
        me.polygons.foreach_set("loop_start", np.concatenate([[0], np.cumsum(sizes)[:-1]]).astype(np.int32))
        me.polygons.foreach_set("material_index", np.asarray(self.mi, np.int32))
        if any(u is not None for u in self.uv):
            uvs = np.zeros((len(loops), 2), np.float32)
            s = 0
            for f, u in zip(self.f, self.uv):
                if u is not None:
                    uvs[s:s + len(f)] = u
                s += len(f)
            lay = me.uv_layers.new(name="UVMap")
            lay.data.foreach_set("uv", uvs.ravel())
        for k, chunks in self.attr.items():
            a = np.zeros(len(V), np.float32)
            for base, val in chunks:
                a[base:base + len(val)] = val
            me.attributes.new(k, "FLOAT", "POINT").data.foreach_set("value", a)
        if smooth:
            me.polygons.foreach_set("use_smooth", ~np.asarray(self.flat, bool))
        me.update(calc_edges=True)
        for m in mats:
            me.materials.append(m)
        return me


def _obj(name, me, coll, loc=(0, 0, 0)):
    ob = bpy.data.objects.new(name, me)
    ob.location = loc
    coll.objects.link(ob)
    return ob


def _frame_from_axis(axis, up_hint=(0, 0, 1)):
    """Rotation matrix whose local X is `axis`."""
    ax = np.asarray(axis, float)
    ax = ax / np.linalg.norm(ax)
    up = np.asarray(up_hint, float)
    if abs(np.dot(up, ax)) > 0.95:
        up = np.array([1.0, 0, 0]) if abs(ax[0]) < 0.9 else np.array([0, 1.0, 0])
    ay = np.cross(up, ax)
    ay /= np.linalg.norm(ay)
    az = np.cross(ax, ay)
    return np.stack([ax, ay, az], axis=1)


def _beam(mb, p0, p1, w, h, rng, mat=0, tex=0.5, chamfer=0.006, segs=None, sag=0.0, top_slope=0.0,
          up=(0, 0, 1), prand=None, extra=None):
    """Sawn timber from p0 to p1, cross-section w (local y) x h (local z), chamfered corners, grain along the
    length (texture v). sag bows the middle down (rails), top_slope cuts the p1 end at an angle (post tops)."""
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    R = _frame_from_axis(p1 - p0, up)
    c = min(chamfer, 0.3 * min(w, h))
    hw, hh = w / 2, h / 2
    sec = np.array([[hw, -hh + c], [hw, hh - c], [hw - c, hh], [-hw + c, hh], [-hw, hh - c], [-hw, -hh + c],
                    [-hw + c, -hh], [hw - c, -hh]])
    ns = len(sec)
    perim = np.concatenate([[0], np.cumsum(np.hypot(*(np.roll(sec, -1, 0) - sec).T))])
    nseg = segs or max(1, int(L / 0.35))
    t = np.linspace(0, 1, nseg + 1)
    pr = rng.random() if prand is None else prand
    uo, vo = rng.random(), rng.random()
    ring = []
    for ti in t:
        xx = ti * L
        off_z = -sag * math.sin(math.pi * ti)
        pts = np.stack([np.full(ns, xx), sec[:, 0], sec[:, 1] + off_z], 1)
        if ti == 1.0 and top_slope:
            pts[:, 0] += pts[:, 2] * top_slope
        ring.append(pts)
    P = np.concatenate(ring) @ R.T + p0
    attrs = dict(prand=pr)
    if extra:
        attrs.update(extra)
    b = mb.verts(P, **attrs)
    for i in range(nseg):
        for j in range(ns):
            j2 = (j + 1) % ns
            a0, a1 = b + i * ns + j, b + i * ns + j2
            c0, c1 = b + (i + 1) * ns + j, b + (i + 1) * ns + j2
            u0, u1 = perim[j] / tex + uo, perim[j + 1] / tex + uo
            v0, v1 = t[i] * L / tex + vo, t[i + 1] * L / tex + vo
            mb.face([a0, a1, c1, c0], mat, [(u0, v0), (u1, v0), (u1, v1), (u0, v1)])
    for end, rev in ((0, True), (nseg, False)):
        idx = [b + end * ns + j for j in range(ns)]
        uv = [(sec[j, 0] / tex + uo, sec[j, 1] / tex + vo) for j in range(ns)]
        if rev:
            idx, uv = idx[::-1], uv[::-1]
        mb.face(idx, mat, uv)
    return L


def _cyl(mb, p0, p1, r0, r1, rng, mat=0, cap_mat=None, tex=1.0, nseg=None, nring=16, bumps=0.0, k=60,
         prand=None, flare=0.0, caps=(True, True), up=(0, 0, 1), bow=0.0):
    """Irregular round timber / log from p0 to p1 (radius r0 -> r1), bark grain along the length."""
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    R = _frame_from_axis(p1 - p0, up)
    nseg = nseg or max(2, int(L / 0.09))
    t = np.linspace(0, 1, nseg + 1)
    th = np.linspace(0, 2 * math.pi, nring, endpoint=False)
    T, TH = np.meshgrid(t, th, indexing="ij")
    rr = r0 + (r1 - r0) * T
    if bumps:
        nz = _nz(k)
        off = rng.random() * 50
        rr = rr * (1 + bumps * nz.fbm(np.cos(TH) * 1.3 + off, T * L * 1.6 + np.sin(TH) * 1.3, 3)
                   + 0.5 * bumps * np.sin(3 * TH + rng.random() * 6))
    if flare:
        rr = rr * (1 + flare * np.exp(-T * L / 0.12))
    bw = bow * np.sin(np.pi * T)
    P = np.stack([T * L, rr * np.cos(TH) + bw, rr * np.sin(TH) + 0.25 * bw], -1).reshape(-1, 3) @ R.T + p0
    pr = rng.random() if prand is None else prand
    b = mb.verts(P, prand=pr)
    circ = 2 * math.pi * 0.5 * (r0 + r1)
    uo, vo = rng.random(), rng.random()
    for i in range(nseg):
        for j in range(nring):
            j2 = (j + 1) % nring
            u0, u1 = j / nring * circ / tex + uo, (j + 1) / nring * circ / tex + uo
            v0, v1 = t[i] * L / tex + vo, t[i + 1] * L / tex + vo
            mb.face([b + i * nring + j, b + i * nring + j2, b + (i + 1) * nring + j2, b + (i + 1) * nring + j],
                    mat, [(u0, v0), (u1, v0), (u1, v1), (u0, v1)])
    cm = mat if cap_mat is None else cap_mat
    for end, rev, on in ((0, True, caps[0]), (nseg, False, caps[1])):
        if not on:
            continue
        idx = [b + end * nring + j for j in range(nring)]
        uv = [(math.cos(a) * 0.5 + 0.5, math.sin(a) * 0.5 + 0.5) for a in th]
        if rev:
            idx, uv = idx[::-1], uv[::-1]
        mb.face(idx, cm, uv, flat=True)
    return R


# =============================================================================================================
# shader helpers
def _sk(sockets, ident):
    for s in sockets:
        if s.identifier == ident:
            return s
    return sockets[ident]


class NT:
    """Small wrapper to build node trees tersely."""

    def __init__(self, tree):
        self.t = tree
        self.nodes = tree.nodes
        self.x = 0

    def node(self, typ, **props):
        n = self.nodes.new(typ)
        self.x += 1
        n.location = ((self.x % 12) * 220 - 1400, -(self.x // 12) * 300)
        for k, v in props.items():
            setattr(n, k, v)
        return n

    def set(self, sock, val):
        if isinstance(val, bpy.types.NodeSocket):
            self.t.links.new(val, sock)
        elif val is None:
            return
        else:
            if sock.type == "RGBA" and len(val) == 3:
                val = tuple(val) + (1.0,)
            sock.default_value = val

    def math(self, op, a, b=None, c=None, clamp=False):
        n = self.node("ShaderNodeMath", operation=op, use_clamp=clamp)
        self.set(n.inputs[0], a)
        if b is not None:
            self.set(n.inputs[1], b)
        if c is not None:
            self.set(n.inputs[2], c)
        return n.outputs[0]

    def vmath(self, op, a, b=None):
        n = self.node("ShaderNodeVectorMath", operation=op)
        self.set(n.inputs[0], a)
        if b is not None:
            self.set(n.inputs[1], b)
        return n.outputs["Value"] if op in ("DOT_PRODUCT", "LENGTH", "DISTANCE") else n.outputs[0]

    def mix(self, fac, a, b, blend="MIX"):
        n = self.node("ShaderNodeMix", data_type="RGBA", blend_type=blend, clamp_result=True)
        self.set(_sk(n.inputs, "Factor_Float"), fac)
        self.set(_sk(n.inputs, "A_Color"), a)
        self.set(_sk(n.inputs, "B_Color"), b)
        return _sk(n.outputs, "Result_Color")

    def mixf(self, fac, a, b):
        n = self.node("ShaderNodeMix", data_type="FLOAT")
        self.set(_sk(n.inputs, "Factor_Float"), fac)
        self.set(_sk(n.inputs, "A_Float"), a)
        self.set(_sk(n.inputs, "B_Float"), b)
        return _sk(n.outputs, "Result_Float")

    def ss(self, val, a, b, lo=0.0, hi=1.0):
        n = self.node("ShaderNodeMapRange", interpolation_type="SMOOTHSTEP")
        self.set(n.inputs["Value"], val)
        n.inputs["From Min"].default_value = a
        n.inputs["From Max"].default_value = b
        n.inputs["To Min"].default_value = lo
        n.inputs["To Max"].default_value = hi
        return n.outputs["Result"]

    def attr(self, name, kind="GEOMETRY"):
        n = self.node("ShaderNodeAttribute", attribute_name=name, attribute_type=kind)
        return n

    def noise(self, vec, scale, detail=3.0, rough=0.5, dist=0.0):
        n = self.node("ShaderNodeTexNoise")
        self.set(n.inputs["Vector"], vec)
        n.inputs["Scale"].default_value = scale
        n.inputs["Detail"].default_value = detail
        n.inputs["Roughness"].default_value = rough
        n.inputs["Distortion"].default_value = dist
        return n

    def ramp(self, fac, stops):
        n = self.node("ShaderNodeValToRGB")
        self.set(n.inputs["Fac"], fac)
        el = n.color_ramp.elements
        while len(el) < len(stops):
            el.new(0.5)
        for e, (p, c) in zip(el, stops):
            e.position = p
            e.color = tuple(c) + (1.0,) if len(c) == 3 else c
        return n.outputs["Color"]

    def hsv(self, col, h=0.5, s=1.0, v=1.0):
        n = self.node("ShaderNodeHueSaturation")
        self.set(n.inputs["Color"], col)
        self.set(n.inputs["Hue"], h)
        self.set(n.inputs["Saturation"], s)
        self.set(n.inputs["Value"], v)
        return n.outputs["Color"]

    def principled(self, col, rough, normal=None, spec=0.5, metal=0.0):
        n = self.node("ShaderNodeBsdfPrincipled")
        self.set(n.inputs["Base Color"], col)
        self.set(n.inputs["Roughness"], rough)
        self.set(n.inputs["Specular IOR Level"], spec)
        self.set(n.inputs["Metallic"], metal)
        if normal is not None:
            self.set(n.inputs["Normal"], normal)
        return n.outputs["BSDF"]

    def mixsh(self, fac, a, b):
        n = self.node("ShaderNodeMixShader")
        self.set(n.inputs[0], fac)
        self.t.links.new(a, n.inputs[1])
        self.t.links.new(b, n.inputs[2])
        return n.outputs[0]

    def output(self, shader, kind="ShaderNodeOutputMaterial"):
        out = self.node(kind)
        self.t.links.new(shader, out.inputs[0])
        return out


def _new_mat(name):
    m = bpy.data.materials.new(name)
    if m.node_tree is None:          # older Blender: node trees are opt-in
        m.use_nodes = True
    for n in list(m.node_tree.nodes):
        m.node_tree.nodes.remove(n)
    return m, NT(m.node_tree)


_IMG = {}


def _tex_path(tex, key):
    return os.path.join(CACHE, "tex", tex, "%s_%s_2k.jpg" % (tex, key))


def _image(tex, key):
    p = _tex_path(tex, key)
    if not os.path.exists(p):
        p1 = os.path.join(CACHE, "tex", tex, "%s_%s_1k.jpg" % (tex, key))
        if not os.path.exists(p1):
            return None
        p = p1
    if p not in _IMG or _IMG[p].name not in bpy.data.images:
        img = bpy.data.images.load(p, check_existing=True)
        img.colorspace_settings.name = "sRGB" if key == "diff" else "Non-Color"
        _IMG[p] = img
    return _IMG[p]


def _pbr(nt, tex, vec, fallback):
    """Sample a Poly Haven texture set at vector `vec` (already scaled to texture space).
    Returns (color, roughness, normal_color) sockets/values; falls back to procedural values."""
    d, a, n = _image(tex, "diff"), _image(tex, "arm"), _image(tex, "nor_gl")
    rmap = _image(tex, "rough") if a is None else None
    if d is None:
        fb_col, fb_rough = fallback
        nz = nt.noise(vec, 6.0, 6.0, 0.6)
        col = nt.ramp(nz.outputs["Fac"], [(0.3, [c * 0.6 for c in fb_col]), (0.7, [min(1, c * 1.4) for c in fb_col])])
        return col, fb_rough, None, nz.outputs["Fac"]

    def img(im):
        t = nt.node("ShaderNodeTexImage", image=im, interpolation="Linear")
        nt.set(t.inputs["Vector"], vec)
        return t

    td = img(d)
    rough = fb_rough = fallback[1]
    if a is not None:
        sep = nt.node("ShaderNodeSeparateColor")
        nt.set(sep.inputs["Color"], img(a).outputs["Color"])
        rough = sep.outputs["Green"]
    elif rmap is not None:
        rough = img(rmap).outputs["Color"]
    nor = img(n).outputs["Color"] if n is not None else None
    return td.outputs["Color"], rough, nor, None


def _uvvec(nt, uv_sock, size, rot=0.0, off=(0.0, 0.0)):
    v = nt.vmath("SCALE", uv_sock)
    v.node.inputs["Scale"].default_value = 1.0 / size
    if rot:
        r = nt.node("ShaderNodeVectorRotate", rotation_type="Z_AXIS")
        nt.set(r.inputs["Vector"], v)
        r.inputs["Angle"].default_value = rot
        v = r.outputs["Vector"]
    if off != (0.0, 0.0):
        v = nt.vmath("ADD", v, (off[0], off[1], 0.0))
    return v


# =============================================================================================================
# HDRI / world
def _hdri_path():
    for res in ("4k", "2k", "1k"):
        for ext in ("hdr", "exr"):
            p = os.path.join(CACHE, "hdri", "%s_%s.%s" % (HDRI_NAME, res, ext))
            if os.path.exists(p):
                return p
    return None


def _hdri_rotation():
    """World Z rotation (rad) for the HDRI so its sun sits at SUN_AZIMUTH_DEG."""
    phi_tex = (0.5 - HDRI_SUN_U) * 2 * math.pi
    return phi_tex - math.radians(SUN_AZIMUTH_DEG)


def sun_direction():
    el = (HDRI_SUN_V - 0.5) * math.pi
    az = math.radians(SUN_AZIMUTH_DEG)
    return (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))


def _env_lookup(nt, vec, img):
    mp = nt.node("ShaderNodeMapping", vector_type="POINT")
    nt.set(mp.inputs["Vector"], vec)
    mp.inputs["Rotation"].default_value = (0.0, 0.0, _hdri_rotation())
    env = nt.node("ShaderNodeTexEnvironment", image=img, interpolation="Linear")
    nt.t.links.new(mp.outputs["Vector"], env.inputs["Vector"])
    clamp = nt.vmath("MINIMUM", env.outputs["Color"], (HDRI_CLAMP,) * 3)
    return clamp


def _build_world(scene):
    w = bpy.data.worlds.get("env:world") or bpy.data.worlds.new("env:world")
    if w.node_tree is None:
        w.use_nodes = True
    nt = NT(w.node_tree)
    for n in list(w.node_tree.nodes):
        w.node_tree.nodes.remove(n)
    path = _hdri_path()
    img = None
    if path:
        img = bpy.data.images.load(path, check_existing=True)
        tc = nt.node("ShaderNodeTexCoord")
        col = _env_lookup(nt, tc.outputs["Generated"], img)
        bg = nt.node("ShaderNodeBackground")
        nt.set(bg.inputs["Color"], col)
        bg.inputs["Strength"].default_value = WORLD_STRENGTH
        nt.output(bg.outputs[0], "ShaderNodeOutputWorld")
    else:
        sky = nt.node("ShaderNodeTexSky")
        for st in ("MULTIPLE_SCATTERING", "NISHITA", "SINGLE_SCATTERING", "HOSEK_WILKIE"):
            try:
                sky.sky_type = st
                break
            except Exception:
                continue
        d = sun_direction()
        try:
            sky.sun_disc = False
            sky.sun_elevation = math.asin(d[2])
            sky.sun_rotation = math.atan2(d[1], d[0]) - math.pi / 2 + math.pi
            sky.altitude = 120.0
            sky.air_density = 1.0
            sky.aerosol_density = 2.0
        except Exception:
            pass
        bg = nt.node("ShaderNodeBackground")
        nt.t.links.new(sky.outputs["Color"], bg.inputs["Color"])
        bg.inputs["Strength"].default_value = 0.12 * WORLD_STRENGTH
        nt.output(bg.outputs[0], "ShaderNodeOutputWorld")
    # EEVEE: no automatic sun extraction (the sun lamp provides the sun); world probe resolution
    for k, v in (("sun_threshold", 1e5), ("probe_resolution", "1024")):
        try:
            setattr(w, k, v)
        except Exception:
            pass
    scene.world = w
    return w, img


def _build_sun(coll):
    L = bpy.data.lights.new("env:sun", "SUN")
    strength = SUN_STRENGTH
    L.energy = strength
    L.color = HDRI_SUN_RGB
    L.angle = math.radians(SUN_ANGLE_DEG)
    for k, v in (("use_shadow", True), ("shadow_maximum_resolution", 0.001), ("use_shadow_jitter", True),
                 ("shadow_jitter_overblur", 10.0), ("shadow_filter_radius", 1.0)):
        try:
            setattr(L, k, v)
        except Exception:
            pass
    ob = bpy.data.objects.new("env:sun", L)
    from mathutils import Vector
    d = Vector(sun_direction())
    ob.rotation_euler = d.to_track_quat("Z", "Y").to_euler()
    ob.location = d * 30.0
    coll.objects.link(ob)
    return ob


# =============================================================================================================
# materials
def _mat_ground(hdri_img):
    m, nt = _new_mat("env:ground")
    uv = nt.node("ShaderNodeUVMap", uv_map="UVMap").outputs["UV"]
    geo = nt.node("ShaderNodeNewGeometry")
    pos = geo.outputs["Position"]
    soil_a = nt.attr("soil").outputs["Fac"]
    mud_a = nt.attr("mud").outputs["Fac"]
    trk_a = nt.attr("track").outputs["Fac"]
    feed_a = nt.attr("feed").outputs["Fac"]
    brk = nt.noise(pos, 1.7, 4.0, 0.6).outputs["Fac"]
    brk2 = nt.noise(pos, 7.0, 2.0, 0.5).outputs["Fac"]
    macro = nt.noise(pos, 0.07, 3.0, 0.55).outputs["Fac"]
    macro2 = nt.noise(pos, 0.31, 2.0, 0.5).outputs["Fac"]

    sw_c, sw_r, sw_n, _ = _pbr(nt, "Grass004", _uvvec(nt, uv, TEX_SIZE["Grass004"], 0.2),
                               ((0.09, 0.13, 0.03), 0.8))
    sw2_c, _, _, _ = _pbr(nt, "Grass004", _uvvec(nt, uv, 3.7, 1.9, (0.41, 0.13)), ((0.09, 0.13, 0.03), 0.8))
    th_c, th_r, th_n, _ = _pbr(nt, "forrest_ground_01", _uvvec(nt, uv, TEX_SIZE["forrest_ground_01"]),
                               ((0.10, 0.11, 0.04), 0.9))
    so_c, so_r, so_n, _ = _pbr(nt, "brown_mud_02", _uvvec(nt, uv, TEX_SIZE["brown_mud_02"], 0.4),
                               ((0.10, 0.07, 0.045), 0.85))
    mu_c, mu_r, mu_n, _ = _pbr(nt, "brown_mud_03", _uvvec(nt, uv, TEX_SIZE["brown_mud_03"], 1.3),
                               ((0.06, 0.045, 0.03), 0.5))
    # sward (dense short grass) mixed with dead thatch, two scales to hide tiling, macro colour drift
    sw = nt.mix(nt.ss(brk, 0.35, 0.65), sw_c, sw2_c)
    thm = nt.ss(nt.math("ADD", brk2, nt.math("MULTIPLY", macro2, 0.6)), 0.55, 0.85)
    th = nt.mix(nt.math("MULTIPLY", thm, 0.75), sw, nt.mix(0.4, th_c, (0.4, 0.5, 0.25), "MULTIPLY"))
    th = nt.hsv(th, nt.mixf(macro2, 0.49, 0.512), 1.12, nt.mixf(macro, 0.48, 0.72))
    th_r = nt.mixf(thm, sw_r, th_r)
    if sw_n is not None and th_n is not None:
        th_n = nt.mix(thm, sw_n, th_n)
    # far: denser-looking meadow colour where the grass instances thin out
    cam = nt.node("ShaderNodeCameraData")
    dist = cam.outputs["View Distance"]
    far_col = nt.mix(nt.ss(macro2, 0.25, 0.75), (0.030, 0.062, 0.012), (0.062, 0.085, 0.020))
    far_col = nt.mix(nt.ss(macro, 0.3, 0.8), nt.mix(0.5, far_col, (0.035, 0.05, 0.015)), (0.09, 0.095, 0.032))
    far_col = nt.mix(nt.math("MULTIPLY", nt.ss(brk2, 0.3, 0.8), 0.5), far_col,
                     nt.mix(1.0, far_col, (0.7, 0.75, 0.6), "MULTIPLY"))
    farf = nt.ss(dist, 10.0, 55.0, 0.0, 0.85)
    col = nt.mix(farf, th, far_col)
    # soil / mud with broken edges
    edge = nt.math("ADD", nt.math("MULTIPLY", nt.math("SUBTRACT", brk, 0.5), 0.6),
                   nt.math("MULTIPLY", nt.math("SUBTRACT", brk2, 0.5), 0.45))
    soil_m = nt.ss(nt.math("ADD", soil_a, edge), 0.4, 0.6)
    mud_m = nt.ss(nt.math("ADD", mud_a, nt.math("MULTIPLY", edge, 1.3)), 0.42, 0.62)
    soil_col = nt.mix(nt.ss(trk_a, 0.2, 0.9), nt.mix(1.0, so_c, (1.15, 1.0, 0.85), "MULTIPLY"),
                      nt.hsv(nt.mix(1.0, so_c, (1.3, 1.12, 0.85), "MULTIPLY"), 0.5, 0.85, 1.55))  # tracks: dusty
    hay = nt.mix(nt.math("MULTIPLY", feed_a, nt.ss(brk2, 0.55, 0.7)), soil_col, (0.42, 0.33, 0.15))
    col = nt.mix(soil_m, col, hay)
    col = nt.mix(mud_m, col, nt.mix(1.0, mu_c, (1.25, 1.02, 0.78), "MULTIPLY"))
    bare_m = nt.math("MAXIMUM", soil_m, mud_m)
    rough = nt.mixf(soil_m, nt.math("MAXIMUM", th_r, 0.6), so_r)
    rough = nt.mixf(mud_m, rough, nt.math("MAXIMUM", nt.math("MULTIPLY", mu_r, 0.85), 0.42))
    normal = None
    if th_n is not None:
        ncol = nt.mix(soil_m, th_n, so_n if so_n is not None else th_n)
        ncol = nt.mix(mud_m, ncol, mu_n if mu_n is not None else ncol)
        nm = nt.node("ShaderNodeNormalMap", uv_map="UVMap")
        nt.set(nm.inputs["Strength"], nt.mixf(bare_m, 0.6, 1.4))
        nt.set(nm.inputs["Color"], ncol)
        normal = nm.outputs["Normal"]
    bsdf = nt.principled(col, rough, normal, spec=nt.mixf(mud_m, 0.15, 0.3))
    shader = bsdf
    if hdri_img is not None:
        # beyond ~150 m blend into the HDRI's own ground so the horizon is seamless
        inc = nt.vmath("SCALE", geo.outputs["Incoming"])
        inc.node.inputs["Scale"].default_value = -1.0
        env = _env_lookup(nt, inc, hdri_img)
        em = nt.node("ShaderNodeEmission")
        nt.set(em.inputs["Color"], env)
        lp = nt.node("ShaderNodeLightPath")
        fac = nt.math("MULTIPLY", nt.ss(dist, 200.0, 400.0), lp.outputs["Is Camera Ray"])
        shader = nt.mixsh(fac, bsdf, em.outputs[0])
    nt.output(shader)
    return m


def _mat_grass():
    m, nt = _new_mat("env:grass_blade")
    h = nt.attr("h01").outputs["Fac"]
    br = nt.attr("brnd").outputs["Fac"]
    dry = nt.attr("dry").outputs["Fac"]
    tint = nt.attr("tint", "INSTANCER").outputs["Fac"]
    oi = nt.node("ShaderNodeObjectInfo")
    green = nt.ramp(br, [(0.0, (0.030, 0.115, 0.016)), (0.45, (0.062, 0.150, 0.020)),
                         (0.8, (0.105, 0.165, 0.026)), (1.0, (0.135, 0.170, 0.035))])
    # lush (blue-green) <-> pale/yellowed patches
    green = nt.mix(nt.ss(tint, 0.0, 1.0), nt.mix(0.45, green, (0.035, 0.12, 0.022), "MIX"),
                   nt.mix(0.45, green, (0.16, 0.175, 0.045), "MIX"))
    green = nt.hsv(green, nt.mixf(oi.outputs["Random"], 0.485, 0.515), 1.0,
                   nt.mixf(oi.outputs["Random"], 0.85, 1.12))
    dcol = nt.ramp(br, [(0.0, (0.15, 0.10, 0.045)), (0.5, (0.32, 0.25, 0.12)), (1.0, (0.45, 0.37, 0.19))])
    col = nt.mix(dry, green, dcol)
    shade = nt.ramp(h, [(0.0, (0.22, 0.22, 0.18)), (0.2, (0.62, 0.62, 0.55)), (0.55, (1, 1, 1)),
                        (1.0, (1.18, 1.16, 1.0))])
    col = nt.mix(1.0, col, shade, "MULTIPLY")
    rough = nt.mixf(dry, 0.55, 0.7)
    bsdf = nt.principled(col, rough, spec=0.22)
    tr = nt.node("ShaderNodeBsdfTranslucent")
    nt.set(tr.inputs["Color"], nt.mix(0.35, col, (0.25, 0.32, 0.03), "MIX"))
    sh = nt.mixsh(nt.mixf(dry, 0.32, 0.12), bsdf, tr.outputs[0])
    nt.output(sh)
    _thin(m)
    return m


def _thin(m):
    for k, v in (("thickness_mode", "SLAB"), ("use_backface_culling", False)):
        try:
            setattr(m, k, v)
        except Exception:
            pass


def _mat_simple_leaf(name, cols, rough=0.45, trans=0.25, spec=0.45):
    m, nt = _new_mat(name)
    br = nt.attr("brnd").outputs["Fac"]
    h = nt.attr("h01").outputs["Fac"]
    oi = nt.node("ShaderNodeObjectInfo")
    col = nt.ramp(br, cols)
    col = nt.hsv(col, nt.mixf(oi.outputs["Random"], 0.49, 0.51), 1.0, nt.mixf(oi.outputs["Random"], 0.85, 1.1))
    col = nt.mix(1.0, col, nt.ramp(h, [(0.0, (0.45, 0.45, 0.4)), (0.4, (1, 1, 1))]), "MULTIPLY")
    bsdf = nt.principled(col, rough, spec=spec)
    if trans > 0:
        tr = nt.node("ShaderNodeBsdfTranslucent")
        nt.set(tr.inputs["Color"], col)
        bsdf = nt.mixsh(trans, bsdf, tr.outputs[0])
    nt.output(bsdf)
    _thin(m)
    return m


def _mat_wood(name="env:timber"):
    """Weathered grey timber (fence, gate): rough_wood + per-piece variation, ground-line staining, lichen."""
    m, nt = _new_mat(name)
    uv = nt.node("ShaderNodeUVMap", uv_map="UVMap").outputs["UV"]
    geo = nt.node("ShaderNodeNewGeometry")
    pr = nt.attr("prand").outputs["Fac"]
    new = nt.attr("newwood").outputs["Fac"]
    c, r, n, _ = _pbr(nt, "rough_wood", uv, ((0.33, 0.30, 0.26), 0.8))
    if _image("rough_wood", "diff") is None:
        wave = nt.node("ShaderNodeTexWave", wave_type="BANDS", bands_direction="Y")
        nt.set(wave.inputs["Vector"], uv)
        wave.inputs["Scale"].default_value = 6.0
        wave.inputs["Distortion"].default_value = 6.0
        c = nt.mix(nt.ss(wave.outputs["Fac"], 0.2, 0.9), (0.20, 0.18, 0.155), (0.40, 0.37, 0.33))
    c = nt.hsv(c, nt.mixf(pr, 0.48, 0.53), nt.mixf(pr, 0.7, 1.1), nt.mixf(pr, 0.72, 1.15))
    c = nt.mix(nt.math("MULTIPLY", new, 0.7), c, nt.mix(1.0, c, (1.9, 1.45, 0.85), "MULTIPLY"))
    sep = nt.node("ShaderNodeSeparateXYZ")
    nt.set(sep.inputs["Vector"], geo.outputs["Position"])
    zfac = nt.ss(sep.outputs["Z"], 0.32, 0.0)                     # wet soil / algae stain at the ground line
    c = nt.mix(nt.math("MULTIPLY", zfac, 0.75), c, nt.mix(1.0, c, (0.42, 0.45, 0.32), "MULTIPLY"))
    nsep = nt.node("ShaderNodeSeparateXYZ")
    nt.set(nsep.inputs["Vector"], geo.outputs["Normal"])
    lich = nt.math("MULTIPLY", nt.ss(nsep.outputs["Z"], 0.5, 0.9),
                   nt.ss(nt.noise(geo.outputs["Position"], 9.0, 4.0, 0.65).outputs["Fac"], 0.58, 0.68))
    c = nt.mix(nt.math("MULTIPLY", lich, 0.8), c, (0.30, 0.33, 0.22))
    normal = None
    if n is not None:
        nm = nt.node("ShaderNodeNormalMap", uv_map="UVMap")
        nt.set(nm.inputs["Color"], n)
        nm.inputs["Strength"].default_value = 1.3
        normal = nm.outputs["Normal"]
    nt.output(nt.principled(c, r, normal, spec=0.3))
    return m


def _mat_bark():
    m, nt = _new_mat("env:bark")
    uv = nt.node("ShaderNodeUVMap", uv_map="UVMap").outputs["UV"]
    pr = nt.attr("prand").outputs["Fac"]
    c, r, n, _ = _pbr(nt, "bark_brown_02", uv, ((0.12, 0.10, 0.08), 0.85))
    c = nt.hsv(c, 0.5, nt.mixf(pr, 0.75, 1.0), nt.mixf(pr, 0.7, 0.95))
    normal = None
    if n is not None:
        nm = nt.node("ShaderNodeNormalMap", uv_map="UVMap")
        nt.set(nm.inputs["Color"], n)
        nm.inputs["Strength"].default_value = 1.6
        normal = nm.outputs["Normal"]
    nt.output(nt.principled(c, r, normal, spec=0.3))
    return m


def _mat_endgrain():
    """Sawn end grain: growth rings and radial checks around the cap centre (cap UVs are a unit disc)."""
    m, nt = _new_mat("env:endgrain")
    uv = nt.node("ShaderNodeUVMap", uv_map="UVMap").outputs["UV"]
    c = nt.vmath("SUBTRACT", uv, (0.5, 0.5, 0.0))
    nz = nt.noise(c, 9.0, 5.0, 0.65)
    jit = nt.vmath("SCALE", nt.vmath("SUBTRACT", nz.outputs["Color"], (0.5, 0.5, 0.5)))
    jit.node.inputs["Scale"].default_value = 0.035
    cd = nt.vmath("ADD", c, jit)
    wave = nt.node("ShaderNodeTexWave", wave_type="RINGS", rings_direction="SPHERICAL")
    nt.set(wave.inputs["Vector"], cd)
    wave.inputs["Scale"].default_value = 5.5
    wave.inputs["Distortion"].default_value = 1.5
    wave.inputs["Detail"].default_value = 2.0
    r = nt.vmath("LENGTH", c)
    ang = nt.node("ShaderNodeSeparateXYZ")
    nt.set(ang.inputs["Vector"], c)
    theta = nt.math("ARCTAN2", ang.outputs["Y"], ang.outputs["X"])
    chk = nt.node("ShaderNodeTexNoise", noise_dimensions="2D")
    cvec = nt.node("ShaderNodeCombineXYZ")
    nt.set(cvec.inputs["X"], nt.math("MULTIPLY", theta, 3.0))
    nt.set(cvec.inputs["Y"], nt.math("MULTIPLY", r, 1.5))
    nt.t.links.new(cvec.outputs[0], chk.inputs["Vector"])
    chk.inputs["Scale"].default_value = 2.5
    chk.inputs["Detail"].default_value = 2.0
    cracks = nt.math("MULTIPLY", nt.ss(chk.outputs["Fac"], 0.66, 0.72), nt.ss(r, 0.08, 0.3))
    base = nt.mix(nt.ss(wave.outputs["Fac"], 0.2, 0.9), (0.22, 0.17, 0.11), (0.42, 0.35, 0.25))
    base = nt.mix(nt.ss(r, 0.38, 0.5), base, (0.16, 0.12, 0.08))          # darker sapwood rim
    col = nt.mix(0.3, base, (0.36, 0.34, 0.31))                            # weathered grey
    col = nt.mix(cracks, col, (0.03, 0.025, 0.02))
    bump = nt.node("ShaderNodeBump")
    nt.set(bump.inputs["Height"], nt.math("SUBTRACT", wave.outputs["Fac"], nt.math("MULTIPLY", cracks, 3.0)))
    bump.inputs["Strength"].default_value = 0.4
    bump.inputs["Distance"].default_value = 0.003
    nt.output(nt.principled(col, 0.85, bump.outputs["Normal"], spec=0.25))
    return m


def _mat_straw():
    m, nt = _new_mat("env:hay")
    tc = nt.node("ShaderNodeTexCoord")
    obj = tc.outputs["Object"]
    sc = nt.node("ShaderNodeMapping", vector_type="POINT")
    nt.set(sc.inputs["Vector"], obj)
    sc.inputs["Scale"].default_value = (40.0, 2.0, 2.0)           # fibres run around the bale (local x axis)
    fib = nt.noise(sc.outputs["Vector"], 3.0, 8.0, 0.7, 0.4)
    coarse = nt.noise(obj, 3.5, 3.0, 0.5)
    col = nt.mix(nt.ss(fib.outputs["Fac"], 0.25, 0.8), (0.08, 0.065, 0.035), (0.26, 0.20, 0.10))
    col = nt.mix(nt.ss(coarse.outputs["Fac"], 0.35, 0.75), nt.mix(0.65, col, (0.13, 0.12, 0.10)), col)  # weathered
    bump = nt.node("ShaderNodeBump")
    nt.set(bump.inputs["Height"], fib.outputs["Fac"])
    bump.inputs["Strength"].default_value = 0.9
    bump.inputs["Distance"].default_value = 0.02
    nt.output(nt.principled(col, 0.85, bump.outputs["Normal"], spec=0.25))
    return m


def _mat_galv():
    m, nt = _new_mat("env:galvanised")
    tc = nt.node("ShaderNodeTexCoord")
    sp = nt.node("ShaderNodeTexVoronoi")
    nt.set(sp.inputs["Vector"], tc.outputs["Object"])
    sp.inputs["Scale"].default_value = 28.0
    grime = nt.noise(tc.outputs["Object"], 4.0, 5.0, 0.6)
    col = nt.mix(nt.ss(sp.outputs["Distance"], 0.1, 0.6), (0.34, 0.35, 0.36), (0.48, 0.49, 0.50))
    geo = nt.node("ShaderNodeNewGeometry")
    sep = nt.node("ShaderNodeSeparateXYZ")
    nt.set(sep.inputs["Vector"], geo.outputs["Position"])
    col = nt.mix(nt.math("MULTIPLY", nt.ss(sep.outputs["Z"], 0.25, 0.0), 0.8), col, (0.14, 0.11, 0.08))
    col = nt.mix(nt.math("MULTIPLY", nt.ss(grime.outputs["Fac"], 0.5, 0.75), 0.5), col, (0.33, 0.31, 0.28))
    rough = nt.mixf(sp.outputs["Distance"], 0.32, 0.5)
    nt.output(nt.principled(col, rough, spec=0.5, metal=0.85))
    return m


def _mat_water():
    m, nt = _new_mat("env:water")
    tc = nt.node("ShaderNodeTexCoord")
    nz = nt.noise(tc.outputs["Object"], 6.0, 3.0, 0.5)
    bump = nt.node("ShaderNodeBump")
    nt.set(bump.inputs["Height"], nz.outputs["Fac"])
    bump.inputs["Strength"].default_value = 0.08
    nt.output(nt.principled((0.02, 0.025, 0.018), 0.05, bump.outputs["Normal"], spec=0.5))
    return m


def _mat_metal_dark():
    m, nt = _new_mat("env:iron")
    nt.output(nt.principled((0.08, 0.075, 0.07), 0.55, spec=0.5, metal=0.8))
    return m


# =============================================================================================================
# grass source clumps
def _strip(mb, rng, base, L, W, segs, lean, bend, phi, brnd, dry, width_fn=None, twist=0.0, mat=0, fold=0.0,
           ret_cl=False):
    """One blade / leaf: centreline leaning `lean` from vertical toward azimuth phi, bending a further `bend`
    toward the tip; width W*width_fn(t); tip collapses to a point. fold > 0 adds a midrib (V section)."""
    t = np.linspace(0.0, 1.0, segs + 1)
    tm = (t[:-1] + t[1:]) * 0.5
    ang = lean + bend * tm ** 1.5
    ds = L / segs
    d = np.array([math.cos(phi), math.sin(phi), 0.0])
    steps = ds * (np.outer(np.sin(ang), d) + np.outer(np.cos(ang), [0, 0, 1.0]))
    cl = np.vstack([np.zeros(3), np.cumsum(steps, 0)]) + base
    wf = width_fn(t) if width_fn else (1 - t) ** 0.75 * (0.75 + 0.25 * np.minimum(1, t * 5))
    rows = []
    for k in range(segs):
        a = phi + math.pi / 2 + twist * t[k]
        side = np.array([math.cos(a), math.sin(a), 0.0]) * W * wf[k] * 0.5
        if fold:
            tang = cl[k + 1] - cl[k]
            nrm = np.cross(side, tang)
            nrm = nrm / (np.linalg.norm(nrm) + 1e-12) * W * wf[k] * fold
            rows.append([cl[k] - side, cl[k] - nrm, cl[k] + side])
        else:
            rows.append([cl[k] - side, cl[k] + side])
    nr = len(rows[0])
    P = np.array(rows).reshape(-1, 3)
    hv = np.repeat(t[:-1], nr)
    b = mb.verts(np.vstack([P, cl[-1]]), h01=np.append(hv, 1.0), brnd=brnd, dry=dry)
    tip = b + segs * nr
    for k in range(segs - 1):
        for j in range(nr - 1):
            mb.face([b + k * nr + j, b + k * nr + j + 1, b + (k + 1) * nr + j + 1, b + (k + 1) * nr + j], mat)
    for j in range(nr - 1):
        mb.face([b + (segs - 1) * nr + j, b + (segs - 1) * nr + j + 1, tip], mat)
    return cl if ret_cl else cl[-1]


def _ribbon(mb, p0, p1, W, side, brnd, mat, h01=1.0, dry=0.0):
    """Small pointed leaf-like quad strip from p0 to p1 (spikelets, seed heads)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    m = 0.5 * (p0 + p1)
    sv = np.asarray(side, float)
    sv = sv / (np.linalg.norm(sv) + 1e-12) * W * 0.5
    b = mb.verts([p0, m - sv, p1, m + sv], h01=h01, brnd=brnd, dry=dry)
    mb.face([b, b + 1, b + 2, b + 3], mat)


def _fan(mb, center, radius_fn, n, up, brnd, mat, h01=0.6, dry=0.0):
    """Flat-ish polygon fan around `center` with rim radius radius_fn(angle) in the plane with normal `up`."""
    up = np.asarray(up, float)
    up /= np.linalg.norm(up)
    e1 = np.cross(up, [0.3, 0.1, 0.95])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(up, e1)
    ang = np.linspace(0, 2 * math.pi, n, endpoint=False)
    rim = [center + radius_fn(a) * (math.cos(a) * e1 + math.sin(a) * e2) for a in ang]
    b = mb.verts(np.vstack([center, rim]), h01=h01, brnd=brnd, dry=dry)
    for i in range(n):
        mb.face([b, b + 1 + i, b + 1 + (i + 1) % n], mat)


def _ball(mb, center, r, brnd, mat, zscale=1.0):
    """Low-poly sphere (clover / dandelion flower head)."""
    rows = []
    for i in range(1, 5):
        el = -math.pi / 2 + i * math.pi / 5
        rows.append([center + r * np.array([math.cos(el) * math.cos(a), math.cos(el) * math.sin(a),
                                            math.sin(el) * zscale])
                     for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)])
    P = np.array(rows).reshape(-1, 3)
    b = mb.verts(np.vstack([P, center - [0, 0, r * zscale], center + [0, 0, r * zscale]]), h01=1.0, brnd=brnd,
                 dry=0.0)
    bot, top = b + 32, b + 33
    for i in range(3):
        for j in range(8):
            j2 = (j + 1) % 8
            mb.face([b + i * 8 + j, b + i * 8 + j2, b + (i + 1) * 8 + j2, b + (i + 1) * 8 + j], mat)
    for j in range(8):
        j2 = (j + 1) % 8
        mb.face([bot, b + j2, b + j], mat)
        mb.face([top, b + 24 + j, b + 24 + j2], mat)


GRASS_KINDS = ["short", "medium", "tall"]


def _make_clump(kind, rng, mult=1.0, lod=0):
    mb = MB()
    U = rng.uniform
    spec = {
        "short": dict(n=62, L=(0.03, 0.095), W=(0.003, 0.0052), R=0.065, segs=3, lean=(0.2, 1.15),
                      bend=(0.4, 1.5), pdry=0.06),
        "medium": dict(n=48, L=(0.06, 0.19), W=(0.0035, 0.006), R=0.07, segs=4, lean=(0.15, 1.0),
                       bend=(0.4, 1.6), pdry=0.08),
        "tall": dict(n=34, L=(0.12, 0.31), W=(0.004, 0.0078), R=0.075, segs=5, lean=(0.08, 0.75), bend=(0.5, 1.7),
                     pdry=0.12),
        "dry": dict(n=28, L=(0.06, 0.20), W=(0.003, 0.005), R=0.065, segs=4, lean=(0.3, 1.3), bend=(0.4, 1.5),
                    pdry=0.75),
    }
    if kind in spec:
        s = spec[kind]
        n = max(5, int(round(s["n"] * mult * (1.0, 0.6, 0.33)[lod])))
        segs = min(s["segs"], (9, 4, 3)[lod])
        wmul = (1.0, 1.3, 2.0)[lod]
        for i in range(n):
            r = s["R"] * math.sqrt(rng.random())
            a = rng.random() * 2 * math.pi
            base = np.array([r * math.cos(a), r * math.sin(a), -0.012])
            phi = a + rng.normal(0, 0.7)
            Lb = U(*s["L"])
            if rng.random() < 0.15:
                Lb *= 0.5
            dry = 1.0 if rng.random() < s["pdry"] else (U(0.25, 0.55) if rng.random() < 0.1 else 0.0)
            _strip(mb, rng, base, Lb, U(*s["W"]) * wmul, segs, U(*s["lean"]), U(*s["bend"]), phi, rng.random(),
                   dry, twist=rng.normal(0, 0.6))
        if not lod:
            for i in range(rng.integers(2, 6)):      # flattened dead blades in the thatch
                a = rng.random() * 2 * math.pi
                _strip(mb, rng, np.array([0, 0, -0.005]), U(0.06, 0.16), U(0.003, 0.005), 3, U(1.25, 1.45),
                       U(0.05, 0.2), a, rng.random(), 1.0)
    elif kind in ("seed", "seed_b"):
        for i in range(int(10 * mult)):
            a = rng.random() * 2 * math.pi
            _strip(mb, rng, np.array([0.03 * math.cos(a), 0.03 * math.sin(a), -0.012]), U(0.10, 0.22),
                   U(0.0035, 0.006), 5, U(0.1, 0.5), U(0.4, 1.2), a, rng.random(), 0.0)
        for i in range(rng.integers(3, 6)):
            a = rng.random() * 2 * math.pi
            H = U(0.30, 0.58)
            phi = rng.random() * 2 * math.pi
            lean = U(0.02, 0.22)
            cl = _strip(mb, rng, np.array([0.02 * math.cos(a), 0.02 * math.sin(a), -0.01]), H, 0.0022, 8, lean,
                        U(0.05, 0.3), phi, rng.random(), 0.45, width_fn=lambda t: np.ones_like(t) * 0.9,
                        ret_cl=True)
            top = cl[-1]
            ax = cl[-1] - cl[-3]
            ax = ax / np.linalg.norm(ax)
            s1 = np.cross(ax, [0.3, 0.7, 0.1])
            s1 /= np.linalg.norm(s1)
            s2 = np.cross(ax, s1)
            br = rng.random()
            if kind == "seed":          # timothy-like cylindrical spike: three crossed tapered ribbons
                hl = U(0.045, 0.09)
                for k in range(3):
                    ang = k * math.pi / 3
                    sv = s1 * math.cos(ang) + s2 * math.sin(ang)
                    for q in range(3):
                        a0, a1 = q / 3.0, (q + 1) / 3.0
                        wq = 0.0055 * math.sin(math.pi * (q + 0.5) / 3.0) ** 0.5
                        _ribbon(mb, top - ax * hl * (1 - a0), top - ax * hl * (1 - a1) + ax * 0.004, wq, sv, br, 1)
            else:                       # ryegrass-like: flat spikelets alternating along the top of the stalk
                ns = int(rng.integers(7, 12))
                for q in range(ns):
                    base = top - ax * (0.012 * q + 0.004)
                    sgn = 1 if q % 2 else -1
                    tipv = ax * 0.011 + s1 * sgn * 0.0045
                    _ribbon(mb, base, base + tipv, 0.0032, np.cross(tipv, s2), br, 1)
    elif kind == "clover":
        for i in range(int(rng.integers(10, 17) * max(mult, 0.6))):
            r = 0.09 * math.sqrt(rng.random())
            a = rng.random() * 2 * math.pi
            base = np.array([r * math.cos(a), r * math.sin(a), -0.01])
            H = U(0.025, 0.085)
            top = _strip(mb, rng, base, H, 0.0016, 3, U(0.0, 0.35), U(0.0, 0.4), rng.random() * 6.28,
                         rng.random(), 0.0, mat=2, width_fn=lambda t: np.ones_like(t))
            rot = rng.random() * 2 * math.pi
            br = rng.random()
            for k in range(3):
                aa = rot + k * 2 * math.pi / 3
                dvec = np.array([math.cos(aa), math.sin(aa), 0.0])
                lr = U(0.0055, 0.0085)
                c = top + dvec * lr * 0.95
                up = np.array([0, 0, 1.0]) + dvec * U(-0.1, 0.35)
                _fan(mb, c, lambda t, lr=lr: lr * (0.9 + 0.12 * math.cos(t)), 8, up, br, 2, h01=0.8)
        for i in range(rng.integers(0, 3)):
            a = rng.random() * 2 * math.pi
            top = _strip(mb, rng, np.array([0.05 * math.cos(a), 0.05 * math.sin(a), -0.01]), U(0.08, 0.14),
                         0.0018, 4, U(0.0, 0.25), 0.2, a, rng.random(), 0.0, mat=2,
                         width_fn=lambda t: np.ones_like(t))
            if rng.random() < 0.5:
                _ball(mb, top + np.array([0, 0, 0.004]), 0.0055, rng.random(), 5)
    elif kind in ("plantain", "dandelion"):
        nl = rng.integers(6, 10) if kind == "plantain" else rng.integers(8, 13)
        for i in range(nl):
            a = rng.random() * 2 * math.pi
            if kind == "plantain":
                L, W = U(0.09, 0.20), U(0.016, 0.026)
                wf = lambda t: np.sin(np.pi * np.clip(t, 0, 1)) ** 0.7 + 0.08
                lean = U(0.7, 1.15)
            else:
                L, W = U(0.08, 0.17), U(0.020, 0.030)
                wf = lambda t: (np.clip(t * 1.4, 0, 1) ** 0.6 * (1 - t) ** 0.5 * 1.6
                                * (1 - 0.45 * np.abs(np.sin(5.5 * np.pi * t))) + 0.06)
                lean = U(1.0, 1.35)
            _strip(mb, rng, np.array([0, 0, -0.005]), L, W, 6, lean, U(0.1, 0.35), a, rng.random(), 0.0,
                   width_fn=wf, mat=3, fold=0.18)
        if kind == "dandelion" and rng.random() < 0.8:
            for i in range(rng.integers(1, 3)):
                top = _strip(mb, rng, np.array([0, 0, 0]), U(0.10, 0.22), 0.0035, 5, U(0.05, 0.3), 0.15,
                             rng.random() * 6.28, rng.random(), 0.0, mat=3, width_fn=lambda t: np.ones_like(t))
                _ball(mb, top + np.array([0, 0, 0.004]), 0.0135, rng.random(), 4, zscale=0.5)
        if kind == "plantain" and rng.random() < 0.7:
            for i in range(rng.integers(1, 4)):
                top = _strip(mb, rng, np.array([0, 0, 0]), U(0.18, 0.32), 0.0022, 6, U(0.05, 0.3), 0.15,
                             rng.random() * 6.28, rng.random(), 0.3, mat=3, width_fn=lambda t: np.ones_like(t))
                for k in range(2):
                    _strip(mb, rng, top - np.array([0, 0, 0.02]), 0.025, 0.009, 3, 0.0, 0.0, k * math.pi / 2,
                           rng.random(), 0.0, mat=1, width_fn=lambda t: np.sin(np.pi * np.clip(t, 0.05, 1)) ** 0.5)
    elif kind == "buttercup":
        for i in range(rng.integers(3, 6)):
            a = rng.random() * 2 * math.pi
            c = np.array([0.04 * math.cos(a), 0.04 * math.sin(a), U(0.02, 0.05)])
            _fan(mb, c, lambda t: 0.015 * (0.35 + 0.65 * abs(math.cos(1.5 * t)) ** 0.5), 18,
                 (math.cos(a) * 0.3, math.sin(a) * 0.3, 1), rng.random(), 3, h01=0.7)
        for i in range(rng.integers(1, 3)):
            top = _strip(mb, rng, np.array([0, 0, 0]), U(0.12, 0.26), 0.0028, 6, U(0.03, 0.3), U(0.0, 0.3),
                         rng.random() * 6.28, rng.random(), 0.0, mat=3, width_fn=lambda t: np.ones_like(t))
            for p in range(5):
                pa = p * 2 * math.pi / 5 + rng.random() * 0.2
                dv = np.array([math.cos(pa), math.sin(pa), 0.0])
                _fan(mb, top + dv * 0.006 + np.array([0, 0, 0.003]), lambda t: 0.0058, 7,
                     dv * -1.4 + np.array([0, 0, 1.0]), rng.random(), 4, h01=1.0)
            _fan(mb, top + np.array([0, 0, 0.002]), lambda t: 0.003, 6, (0, 0, 1), 0.5, 4, h01=1.0)
    return mb


# variant table: (kind, lod, count)
_VARIANTS = ([("short", 0, 4), ("medium", 0, 4), ("tall", 0, 3), ("seed", 0, 2), ("seed_b", 0, 1), ("dry", 0, 2),
              ("clover", 0, 2), ("plantain", 0, 1), ("dandelion", 0, 1), ("buttercup", 0, 1),
              ("short", 1, 3), ("medium", 1, 3), ("tall", 1, 2), ("short", 2, 2), ("medium", 2, 2), ("tall", 2, 2)])


def _build_grass_sources(coll, mats, mult):
    rng = np.random.default_rng(SEED + 5)
    index = {}
    objs = []
    tris = []
    i = 0
    for kind, lod, cnt in _VARIANTS:
        for c in range(cnt):
            mb = _make_clump(kind, rng, mult, lod)
            name = "env:gsrc_%02d_%s%s_%d" % (i, kind, "_lod%d" % lod if lod else "", c)
            me = mb.build(name, mats, smooth=True)
            ob = _obj(name, me, coll)
            index.setdefault((kind, lod), []).append(i)
            objs.append(ob)
            tris.append(sum(len(f) - 2 for f in mb.f))
            i += 1
    return objs, index, np.array(tris)


def _scatter_points(preset, clear_zones=()):
    """Stratified (jittered grid) clump positions with density falloff, plus per-point instance choice."""
    rng = np.random.default_rng(SEED + 11)
    dmax = preset["clumps_per_m2"]
    cell = 1.0 / math.sqrt(dmax)
    cx, cy = GRASS_CENTER
    R = preset["r_max"]
    n = int(2 * R / cell) + 1
    g = (np.arange(n) * cell - R)
    X, Y = np.meshgrid(g + cx, g + cy, indexing="ij")
    X = X.ravel() + rng.random(X.size) * cell
    Y = Y.ravel() + rng.random(Y.size) * cell
    d = np.hypot(X - cx, Y - cy)
    keep = d < R
    X, Y, d = X[keep], Y[keep], d[keep]
    fall = (1.0 - (1.0 - preset["mid"]) * _sstep(preset["r_near"], preset["r_full"], d)) * \
        (1.0 - (1.0 - preset["far_min"] / preset["mid"]) * _sstep(preset["r_full"], R, d))
    # extra candidates in the hero zone (density multiplier preset["hero"])
    hx0, hx1, hhy = HERO_ZONE
    if preset["hero"] > 1.0:
        hc = cell / math.sqrt(preset["hero"] - 1.0)
        gx = np.arange(hx0 - 1.0, hx1 + 1.0, hc)
        gy = np.arange(-hhy - 1.0, hhy + 1.0, hc)
        HX, HY = np.meshgrid(gx, gy, indexing="ij")
        HX = HX.ravel() + rng.random(HX.size) * hc
        HY = HY.ravel() + rng.random(HY.size) * hc
        hm = 1.0 - _sstep(-1.5, 3.0, _box_sdf(HX, HY, hx0, hx1, hhy) + 1.5 * _nz(15).fbm(HX / 2.0, HY / 2.0, 2))
        hk = rng.random(len(HX)) < hm
        X = np.concatenate([X, HX[hk]])
        Y = np.concatenate([Y, HY[hk]])
        d = np.hypot(X - cx, Y - cy)
        fall = np.concatenate([fall, np.ones(hk.sum())])
    F = _fields(X, Y)
    clus = np.clip(0.80 + 0.55 * _nz(21).fbm(X / 0.8, Y / 0.8, 2), 0.35, 1.0)    # tufty clustering
    p = F["dens"] * fall * clus
    for (zx, zy, zr) in clear_zones:
        p = p * (np.hypot(X - zx, Y - zy) > zr)
    keep = rng.random(len(X)) < p
    X, Y, d = X[keep], Y[keep], d[keep]
    F = {k: v[keep] for k, v in F.items()}
    return X, Y, d, F, rng


def _choose_instances(X, Y, d, F, rng, index, preset):
    N = len(X)
    inst = np.zeros(N, np.int32)
    hzone = _box_sdf(X, Y, *HERO_ZONE) < 0.5
    lod = np.where(hzone | (d <= preset["r_near"]), 0, np.where(d <= preset["lod_r"], 1, 2))
    t = F["gh"] + rng.normal(0, 0.16, N)
    cat = np.where(t < 0.36, 0, np.where(t < 0.70, 1, 2))
    for ci, kind in enumerate(GRASS_KINDS):
        for lv in (0, 1, 2):
            m = (cat == ci) & (lod == lv)
            ids = np.array(index[(kind, lv)])
            inst[m] = ids[rng.integers(0, len(ids), m.sum())]
    lod = lod == 2
    tallm = (cat == 2) & ~lod
    seedp = 0.18 + 0.45 * F["near_fence"]
    sm = tallm & (rng.random(N) < seedp)
    seed_ids = np.array(index[("seed", 0)] + index[("seed_b", 0)])
    inst[sm] = seed_ids[rng.integers(0, len(seed_ids), sm.sum())]
    drym = (rng.random(N) < 0.02 + 0.10 * F["dry"]) & ~lod
    inst[drym] = np.array(index[("dry", 0)])[rng.integers(0, 2, drym.sum())]
    near = d < preset["lod_r"] * 1.6
    w = rng.random(N)
    clov = near & (w < 0.02 + 0.16 * F["clover"])
    inst[clov] = np.array(index[("clover", 0)])[rng.integers(0, 2, clov.sum())]
    w2 = rng.random(N)
    weeds = near & ~clov & (F["gh"] < 0.6)
    inst[weeds & (w2 < 0.006)] = index[("plantain", 0)][0]
    inst[weeds & (w2 >= 0.006) & (w2 < 0.011)] = index[("dandelion", 0)][0]
    inst[weeds & (w2 >= 0.011) & (w2 < 0.0145)] = index[("buttercup", 0)][0]
    return inst


def _build_grass(coll, src_coll, terrain_axes, Zgrid, preset, mats, clear_zones=()):
    objs, index, tris = _build_grass_sources(src_coll, mats, preset["blade_mult"])
    X, Y, d, F, rng = _scatter_points(preset, clear_zones)
    inst = _choose_instances(X, Y, d, F, rng, index, preset)
    Z = _interp_grid(terrain_axes, Zgrid, X, Y)
    N = len(X)
    # scale: per-clump random, shorter in the grazed flat zones, larger with distance to keep coverage
    s = rng.uniform(0.78, 1.25, N) * (1.0 - 0.1 * F["graze"]) * (1.0 - 0.35 * F["board"]) * \
        (1.0 + 0.7 * _sstep(preset["r_full"], preset["r_max"], d))
    sz = s * rng.uniform(0.85, 1.15, N)
    scl = np.stack([s, s, sz], 1)
    rot = np.stack([rng.normal(0, 0.08, N), rng.normal(0, 0.08, N), rng.random(N) * 2 * math.pi], 1)
    tint = np.clip(0.5 + 0.55 * _nz(20).fbm(X / 4.0, Y / 4.0, 3) + 0.35 * (F["dry"] - 0.3), 0, 1)
    me = bpy.data.meshes.new("env:grass")
    me.vertices.add(N)
    me.vertices.foreach_set("co", np.stack([X, Y, Z], 1).astype(np.float32).ravel())
    me.attributes.new("inst", "INT", "POINT").data.foreach_set("value", inst)
    me.attributes.new("rot", "FLOAT_VECTOR", "POINT").data.foreach_set("vector", rot.astype(np.float32).ravel())
    me.attributes.new("scl", "FLOAT_VECTOR", "POINT").data.foreach_set("vector", scl.astype(np.float32).ravel())
    me.attributes.new("tint", "FLOAT", "POINT").data.foreach_set("value", tint.astype(np.float32))
    me.update()
    ob = _obj("env:grass", me, coll)
    ng = _grass_nodes(src_coll)
    mod = ob.modifiers.new("scatter", "NODES")
    mod.node_group = ng
    stats = dict(instances=int(N), triangles=int(tris[inst].sum()), sources=len(objs),
                 per_kind={k: int(np.isin(inst, v).sum()) for k, v in
                           (("grass", sum((index[(g, l)] for g in GRASS_KINDS for l in (0, 1, 2)), [])),
                            ("seed", index[("seed", 0)] + index[("seed_b", 0)]),
                            ("dry", index[("dry", 0)]), ("clover", index[("clover", 0)]),
                            ("weeds", index[("plantain", 0)] + index[("dandelion", 0)]
                             + index[("buttercup", 0)]))})
    return ob, stats


def _grass_nodes(src_coll):
    ng = bpy.data.node_groups.new("env:grass_scatter", "GeometryNodeTree")
    ng.interface.new_socket("Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    ng.interface.new_socket("Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    N = ng.nodes
    gi = N.new("NodeGroupInput")
    go = N.new("NodeGroupOutput")
    ci = N.new("GeometryNodeCollectionInfo")
    ci.transform_space = "ORIGINAL"
    ci.inputs["Collection"].default_value = src_coll
    ci.inputs["Separate Children"].default_value = True
    ci.inputs["Reset Children"].default_value = True
    iop = N.new("GeometryNodeInstanceOnPoints")
    iop.inputs["Pick Instance"].default_value = True

    def named(name, dt):
        n = N.new("GeometryNodeInputNamedAttribute")
        n.data_type = dt
        n.inputs["Name"].default_value = name
        return n.outputs["Attribute"]

    e2r = N.new("FunctionNodeEulerToRotation")
    L = ng.links
    L.new(gi.outputs[0], iop.inputs["Points"])
    L.new(ci.outputs["Instances"], iop.inputs["Instance"])
    L.new(named("inst", "INT"), iop.inputs["Instance Index"])
    L.new(named("rot", "FLOAT_VECTOR"), e2r.inputs[0])
    L.new(e2r.outputs[0], iop.inputs["Rotation"])
    L.new(named("scl", "FLOAT_VECTOR"), iop.inputs["Scale"])
    L.new(iop.outputs["Instances"], go.inputs[0])
    for i, n in enumerate((gi, ci, iop, go)):
        n.location = (i * 250, 0)
    return ng


# =============================================================================================================
# terrain
def _axis(c, half, step, far, growth=1.075):
    inner = np.arange(-half, half + 1e-9, step)
    outer = []
    s, x = step, inner[-1]
    while x < far:
        s = min(s * growth, 30.0)
        x += s
        outer.append(x)
    outer = np.array(outer)
    return c + np.concatenate([-outer[::-1], inner, outer])


def _interp_grid(axes, Z, px, py):
    """Exact height on the triangulated grid (cell split along the (i,j)-(i+1,j+1) diagonal)."""
    ax, ay = axes
    i = np.clip(np.searchsorted(ax, px) - 1, 0, len(ax) - 2)
    j = np.clip(np.searchsorted(ay, py) - 1, 0, len(ay) - 2)
    fx = (px - ax[i]) / (ax[i + 1] - ax[i])
    fy = (py - ay[j]) / (ay[j + 1] - ay[j])
    za, zb, zc, zd = Z[i, j], Z[i + 1, j], Z[i + 1, j + 1], Z[i, j + 1]
    return np.where(fx >= fy, za + fx * (zb - za) + fy * (zc - zb), za + fy * (zd - za) + fx * (zc - zd))


def _build_terrain(coll, mat):
    ax = _axis(TERRAIN_CENTER[0], TERRAIN_DENSE_HALF, TERRAIN_STEP, TERRAIN_FAR)
    ay = _axis(TERRAIN_CENTER[1], TERRAIN_DENSE_HALF, TERRAIN_STEP, TERRAIN_FAR)
    X, Y = np.meshgrid(ax, ay, indexing="ij")
    F = _fields(X.ravel(), Y.ravel())
    nx, ny = X.shape
    Z = F["z"].reshape(nx, ny)
    V = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
    I = np.arange(nx * ny).reshape(nx, ny)
    a, b, c, d = I[:-1, :-1].ravel(), I[1:, :-1].ravel(), I[1:, 1:].ravel(), I[:-1, 1:].ravel()
    tris = np.concatenate([np.stack([a, b, c], 1), np.stack([a, c, d], 1)])
    me = bpy.data.meshes.new("env:terrain")
    me.vertices.add(len(V))
    me.vertices.foreach_set("co", V.astype(np.float32).ravel())
    me.loops.add(tris.size)
    me.loops.foreach_set("vertex_index", tris.astype(np.int32).ravel())
    me.polygons.add(len(tris))
    me.polygons.foreach_set("loop_start", (np.arange(len(tris)) * 3).astype(np.int32))
    me.polygons.foreach_set("use_smooth", np.ones(len(tris), bool))
    uv = me.uv_layers.new(name="UVMap")
    uv.data.foreach_set("uv", V[tris.ravel(), :2].astype(np.float32).ravel())
    for k in ("soil", "mud", "track", "feed"):
        me.attributes.new(k, "FLOAT", "POINT").data.foreach_set("value", F[k].astype(np.float32))
    me.update(calc_edges=True)
    me.materials.append(mat)
    ob = _obj("env:terrain", me, coll)
    return ob, (ax, ay), Z, len(tris)


def check_flat_zones(n=4000):
    """Max |z| (m) in the SPEC flat zones: landing patch x 3.5..6, |y| < 1, and the base (r < 1.5 m)."""
    rng = np.random.default_rng(1)
    lx, ly = rng.uniform(3.5, 6.0, n), rng.uniform(-1.0, 1.0, n)
    a, r = rng.uniform(0, 2 * math.pi, n), 1.5 * np.sqrt(rng.random(n))
    bx, by = r * np.cos(a), r * np.sin(a)
    return float(np.abs(terrain_height(lx, ly)).max()), float(np.abs(terrain_height(bx, by)).max())


# =============================================================================================================
# wood features
def _post_axis(rng, lean_deg):
    a = rng.random() * 2 * math.pi
    t = math.radians(lean_deg)
    return np.array([math.sin(t) * math.cos(a), math.sin(t) * math.sin(a), math.cos(t)])


def _fence_line(mb, rng, p0, p1, inward, gate=None, post_h=1.32):
    """Post-and-rail fence from p0 to p1 (xy). Rails on the `inward` (paddock) side. gate=(s0, s1) leaves a
    gap along the line (distance from p0) for the field gate."""
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    dirv = (p1 - p0) / L
    inward = np.asarray(inward, float)
    nb = int(round(L / POST_SPACING))
    ss = np.linspace(0, L, nb + 1)
    if gate:
        ss = np.array([s for s in ss if s < gate[0] - 0.6 or s > gate[1] + 0.6] + [gate[0] - 0.12, gate[1] + 0.12])
        ss.sort()
    posts = []
    for s in ss:
        xy = p0 + dirv * (s + rng.normal(0, 0.05))
        z0 = terrain_height(xy[0], xy[1])
        lean = rng.choice([rng.uniform(0.3, 2.0), rng.uniform(2.5, 5.5)], p=[0.85, 0.15])
        axis = _post_axis(rng, lean)
        base = np.array([xy[0], xy[1], z0 - 0.35])
        top = np.array([xy[0], xy[1], z0]) + axis * (post_h + rng.normal(0, 0.025))
        is_gate = gate is not None and any(abs(s - g) < 0.2 for g in (gate[0] - 0.12, gate[1] + 0.12))
        if is_gate:
            _cyl(mb, base, top + axis * 0.15, 0.105, 0.095, rng, 0, cap_mat=2, tex=TEX_SIZE["rough_wood"],
                 nring=14, bumps=0.03, k=61)
        else:
            _beam(mb, base, top, 0.075, 0.125, rng, 0, TEX_SIZE["rough_wood"], chamfer=0.01, segs=3,
                  top_slope=0.5, up=np.append(dirv, 0))
        posts.append((s, np.array([xy[0], xy[1], z0]), axis, is_gate))
    off = 0.0375 + 0.019 + 0.002
    for (s0, g0, a0, gate0), (s1, g1, a1, gate1) in zip(posts[:-1], posts[1:]):
        if gate and s0 < gate[1] and s1 > gate[0]:
            continue
        broken = rng.random() < 0.03
        for k, h in enumerate(RAIL_HEIGHTS):
            if broken and k == 1:
                continue
            hh = h + rng.normal(0, 0.012)
            e0 = g0 + a0 * (hh + rng.normal(0, 0.01)) + np.append(inward, 0) * off + np.append(dirv, 0) * 0.004
            e1 = g1 + a1 * (hh + rng.normal(0, 0.01)) + np.append(inward, 0) * off - np.append(dirv, 0) * 0.004
            new = 1.0 if rng.random() < 0.04 else 0.0
            _beam(mb, e0, e1, 0.038, 0.087, rng, 0, TEX_SIZE["rough_wood"], chamfer=0.004, segs=6,
                  sag=rng.uniform(0.0, 0.015), extra=dict(newwood=new))
    return posts


def _gate(mb, rng, hinge_xy, latch_xy, inward):
    """Five-bar timber field gate (closed) between the gate posts, with a diagonal brace and iron hinges."""
    h0 = np.asarray(hinge_xy, float)
    h1 = np.asarray(latch_xy, float)
    dirv = (h1 - h0) / np.linalg.norm(h1 - h0)
    W = float(np.linalg.norm(h1 - h0)) - 0.24
    z0 = terrain_height(*(0.5 * (h0 + h1)))
    o = np.append(h0 + dirv * 0.12, z0) + np.append(inward, 0) * 0.0
    D = np.append(dirv, 0)
    Z = np.array([0, 0, 1.0])
    T = TEX_SIZE["rough_wood"]
    bot, H = 0.13, 1.13
    up = np.append(inward, 0)
    _beam(mb, o + Z * bot, o + Z * (bot + H + 0.05), 0.10, 0.12, rng, 0, T, segs=3, up=D)           # hanging stile
    _beam(mb, o + D * W + Z * bot, o + D * W + Z * (bot + H), 0.085, 0.09, rng, 0, T, segs=3, up=D)  # closing stile
    _beam(mb, o + Z * (bot + H), o + D * W + Z * (bot + H - 0.01), 0.085, 0.12, rng, 0, T, segs=6, up=up)
    for k in range(4):
        zz = bot + 0.06 + k * (H - 0.20) / 4.0 * (0.85 + 0.05 * k)
        _beam(mb, o + Z * zz, o + D * W + Z * zz, 0.042, 0.095, rng, 0, T, segs=6, up=up)
    _beam(mb, o + Z * (bot + 0.08) + D * 0.06, o + D * (W - 0.06) + Z * (bot + H - 0.06), 0.042, 0.095, rng, 0,
          T, segs=6, up=up)
    for zz in (bot + 0.12, bot + H - 0.04):          # hinge straps and pins
        _beam(mb, o + Z * zz + up * 0.03 - D * 0.08, o + Z * zz + up * 0.03 + D * 0.55, 0.006, 0.04, rng, 1, 1.0,
              segs=1, up=up, chamfer=0.001)
        _cyl(mb, o + Z * (zz - 0.05) - D * 0.1, o + Z * (zz + 0.05) - D * 0.1, 0.012, 0.012, rng, 1, nring=8,
             nseg=1)
    _beam(mb, o + D * (W + 0.02) + Z * (bot + H - 0.15) + up * 0.04,
          o + D * (W + 0.16) + Z * (bot + H - 0.15) + up * 0.04, 0.012, 0.03, rng, 1, 1.0, segs=1, up=up,
          chamfer=0.001)


def _log(mb, rng, cfg, k=70):
    """Fallen log / log jump lying on the terrain, pitched to follow it, with sawn end caps."""
    lx, ly = cfg["xy"]
    a = math.radians(cfg["yaw"])
    dv = np.array([math.cos(a), math.sin(a)])
    L, R = cfg["length"], 0.5 * cfg["dia"]
    s = np.linspace(-L / 2, L / 2, 9)
    gz = terrain_height(lx + dv[0] * s, ly + dv[1] * s)
    slope, icpt = np.polyfit(s, gz, 1)
    p0 = np.array([lx - dv[0] * L / 2, ly - dv[1] * L / 2, icpt - slope * L / 2 + R - cfg.get("sink", 0.05)])
    p1 = np.array([lx + dv[0] * L / 2, ly + dv[1] * L / 2, icpt + slope * L / 2 + R * 0.88 - cfg.get("sink", 0.05)])
    Rf = _cyl(mb, p0, p1, R, R * 0.88, rng, 0, cap_mat=1, tex=TEX_SIZE["bark_brown_02"], nring=26, bumps=0.09, k=k,
              bow=rng.uniform(-0.02, 0.02) * L)
    if cfg.get("stub"):
        for frac, ang, ln in ((0.35, 1.9, 0.38), (0.72, -2.3, 0.22)):
            c = p0 + (p1 - p0) * frac
            dirn = Rf @ np.array([0.25, math.cos(ang), math.sin(ang)])
            _cyl(mb, c + dirn * R * 0.6, c + dirn * (R + ln), R * 0.28, R * 0.18, rng, 0, cap_mat=1,
                 tex=TEX_SIZE["bark_brown_02"], nring=10, bumps=0.05, k=k + 1)


def _bale(mb, rng, xy, yaw):
    """Round hay bale on its side, slightly slumped."""
    a = math.radians(yaw)
    dv = np.array([math.cos(a), math.sin(a), 0.0])
    z0 = terrain_height(*xy)
    c = np.array([xy[0], xy[1], z0])
    R, W = 0.74, 1.2
    nring, nseg = 36, 8
    th = np.linspace(0, 2 * math.pi, nring, endpoint=False)
    t = np.linspace(-0.5, 0.5, nseg + 1)
    Rm = _frame_from_axis(dv)
    verts = []
    nz = _nz(90)
    for ti in t:
        bulge = (1.0 + 0.03 * (1 - (2 * ti) ** 2)) * (1 - 0.05 * (2 * abs(ti)) ** 6)
        for tt in th:
            rj = 1.0 + 0.025 * nz.fbm(math.cos(tt) * 2.0 + ti * 3.0, math.sin(tt) * 2.0, 2)
            y, z = math.cos(tt) * R * bulge * rj, math.sin(tt) * R * bulge * rj
            z = max(z, -R * 0.86)
            verts.append([ti * W, y, z + R * 0.86 - 0.03])
    P = np.array(verts) @ Rm.T + c
    b = mb.verts(P)
    for i in range(nseg):
        for j in range(nring):
            j2 = (j + 1) % nring
            mb.face([b + i * nring + j, b + i * nring + j2, b + (i + 1) * nring + j2, b + (i + 1) * nring + j], 0)
    for end, rev in ((0, True), (nseg, False)):
        idx = [b + end * nring + j for j in range(nring)]
        mb.face(idx[::-1] if rev else idx, 1)


def _trough(mb, rng, xy):
    """Round corrugated galvanised water trough with water."""
    z0 = terrain_height(*xy) - 0.03
    R, H = 0.72, 0.56
    nring = 40
    th = np.linspace(0, 2 * math.pi, nring, endpoint=False)
    zs = np.linspace(0, H, 17)
    for rr, flip in ((R, False), (R - 0.006, True)):
        verts = [[xy[0] + (rr + 0.006 * math.sin(2 * math.pi * z / 0.068)) * math.cos(a),
                  xy[1] + (rr + 0.006 * math.sin(2 * math.pi * z / 0.068)) * math.sin(a), z0 + z]
                 for z in zs for a in th]
        b = mb.verts(verts)
        for i in range(len(zs) - 1):
            for j in range(nring):
                j2 = (j + 1) % nring
                f = [b + i * nring + j, b + i * nring + j2, b + (i + 1) * nring + j2, b + (i + 1) * nring + j]
                mb.face(f[::-1] if flip else f, 0)
    # rolled rim
    tor = []
    for a in th:
        for p in np.linspace(0, 2 * math.pi, 8, endpoint=False):
            rr = R + 0.012 * math.cos(p)
            tor.append([xy[0] + rr * math.cos(a), xy[1] + rr * math.sin(a), z0 + H + 0.012 * math.sin(p)])
    b = mb.verts(tor)
    for i in range(nring):
        for j in range(8):
            i2, j2 = (i + 1) % nring, (j + 1) % 8
            mb.face([b + i * 8 + j, b + i2 * 8 + j, b + i2 * 8 + j2, b + i * 8 + j2], 0)
    wv = [[xy[0] + (R - 0.01) * math.cos(a), xy[1] + (R - 0.01) * math.sin(a), z0 + H - 0.09] for a in th]
    b = mb.verts(wv)
    mb.face([b + j for j in range(nring)], 1)


def _build_wood(coll, mats):
    rng = np.random.default_rng(SEED + 3)
    timber, iron, endg, bark, hay, galv, water = (mats[k] for k in
                                                   ("timber", "iron", "endgrain", "bark", "hay", "galv", "water"))
    # fence A (back, along x) with the gate; fence B (side, along y). Corner post shared.
    mb = MB()
    _fence_line(mb, rng, (FENCE_A_X[0], FENCE_A_Y), (FENCE_A_X[1], FENCE_A_Y), (0, -1),
                gate=(GATE_X[0] - FENCE_A_X[0], GATE_X[1] - FENCE_A_X[0]))
    _gate(mb, rng, (GATE_X[0], FENCE_A_Y), (GATE_X[1], FENCE_A_Y), (0, -1))
    fa = _obj("env:fence_back", mb.build("env:fence_back", (timber, iron, endg)), coll)
    mb = MB()
    _fence_line(mb, rng, (FENCE_B_X, FENCE_B_Y[1] - POST_SPACING), (FENCE_B_X, FENCE_B_Y[0]), (1, 0))
    fb = _obj("env:fence_side", mb.build("env:fence_side", (timber, iron, endg)), coll)
    out = [fa, fb]
    mb = MB()
    _log(mb, rng, LOG_JUMP, k=70)
    out.append(_obj("env:log_jump", mb.build("env:log_jump", (bark, endg), smooth=True), coll))
    for i, cfg in enumerate(FALLEN_LOGS):
        mb = MB()
        _log(mb, rng, cfg, k=71 + 2 * i)
        out.append(_obj("env:log_%d" % (i + 1), mb.build("env:log_%d" % (i + 1), (bark, endg), smooth=True), coll))
    mb = MB()
    sx, sy = STUMP["xy"]
    z0 = terrain_height(sx, sy)
    _cyl(mb, (sx, sy, z0 - 0.1), (sx, sy, z0 + STUMP["h"]), STUMP["dia"] / 2, STUMP["dia"] / 2 * 0.93, rng, 0,
         cap_mat=1, tex=TEX_SIZE["bark_brown_02"], nring=24, bumps=0.07, k=80, flare=0.35, caps=(False, True))
    out.append(_obj("env:stump", mb.build("env:stump", (bark, endg), smooth=True), coll))
    mb = MB()
    _bale(mb, rng, BALE_XY, 18.0)
    out.append(_obj("env:hay_bale", mb.build("env:hay_bale", (hay, hay), smooth=True), coll))
    mb = MB()
    _trough(mb, rng, TROUGH_XY)
    out.append(_obj("env:water_trough", mb.build("env:water_trough", (galv, water), smooth=True), coll))
    return out


# =============================================================================================================
def _collection(scene, name, parent=None):
    c = bpy.data.collections.get(name)
    if c is None:
        c = bpy.data.collections.new(name)
        (parent or scene.collection).children.link(c)
    return c


def _clear_env():
    for name in ("ENV_SRC", "ENV"):
        c = bpy.data.collections.get(name)
        if c:
            for ob in list(c.objects):
                bpy.data.objects.remove(ob, do_unlink=True)
    for ng in [g for g in bpy.data.node_groups if g.name.startswith("env:")]:
        bpy.data.node_groups.remove(ng)


def build_env(scene, quality=None, clear_zones=(), board_clear=True, board_half=None):
    """Build the paddock environment into collection ENV of `scene`.

    quality: key of GRASS_PRESETS (default GRASS_QUALITY). clear_zones: [(x, y, r), ...] discs kept free of
    grass (e.g. where the dart comes to rest). board_clear: no grass inside the base board footprint
    (board_half = (hx, hy) half sizes in m, default BOARD_HALF), so blades do not poke through the board; the
    edge is soft and ragged and sits just inside the footprint. Returns a dict with the main objects and stats."""
    global BOARD_CLEAR, BOARD_HALF
    BOARD_CLEAR = bool(board_clear)
    if board_half:
        BOARD_HALF = tuple(board_half)
    t0 = time.time()
    preset = GRASS_PRESETS[quality or GRASS_QUALITY]
    _clear_env()
    env = _collection(scene, "ENV")
    src = _collection(scene, "ENV_SRC", env)
    for lc in scene.view_layers[0].layer_collection.children:
        for sub in lc.children:
            if sub.collection == src:
                sub.exclude = True
    src.hide_render = False
    world, hdri_img = _build_world(scene)
    sun = _build_sun(env)
    mats = dict(ground=_mat_ground(hdri_img), grass=_mat_grass(), timber=_mat_wood(), bark=_mat_bark(),
                endgrain=_mat_endgrain(), hay=_mat_straw(), galv=_mat_galv(), water=_mat_water(),
                iron=_mat_metal_dark())
    mats["seedhead"] = _mat_simple_leaf("env:seedhead", [(0.0, (0.085, 0.09, 0.040)), (0.5, (0.12, 0.085, 0.07)),
                                                         (1.0, (0.20, 0.17, 0.085))], rough=0.65, trans=0.12,
                                        spec=0.25)
    mats["clover"] = _mat_simple_leaf("env:clover_leaf", [(0.0, (0.045, 0.110, 0.022)),
                                                          (1.0, (0.075, 0.140, 0.032))], rough=0.55, trans=0.25,
                                      spec=0.25)
    mats["weed"] = _mat_simple_leaf("env:weed_leaf", [(0.0, (0.030, 0.075, 0.014)), (1.0, (0.055, 0.105, 0.022))],
                                    rough=0.5, trans=0.2, spec=0.28)
    mats["yellow"] = _mat_simple_leaf("env:flower_yellow", [(0.0, (0.75, 0.45, 0.0)), (1.0, (0.9, 0.62, 0.02))],
                                      rough=0.25, trans=0.1, spec=0.7)
    mats["white"] = _mat_simple_leaf("env:flower_white", [(0.0, (0.42, 0.36, 0.32)), (1.0, (0.62, 0.58, 0.52))],
                                     rough=0.7, trans=0.2, spec=0.2)
    t1 = time.time()
    terrain, axes, Zgrid, ntris = _build_terrain(env, mats["ground"])
    t2 = time.time()
    gmats = (mats["grass"], mats["seedhead"], mats["clover"], mats["weed"], mats["yellow"], mats["white"])
    grass, gstats = _build_grass(env, src, axes, Zgrid, preset, gmats, clear_zones)
    t3 = time.time()
    wood = _build_wood(env, mats)
    t4 = time.time()
    flat = check_flat_zones()
    stats = dict(grass=gstats, terrain_tris=ntris, flat_landing_max_m=flat[0], flat_base_max_m=flat[1],
                 build_s=round(t4 - t0, 1), t_mats=round(t1 - t0, 1), t_terrain=round(t2 - t1, 1),
                 t_grass=round(t3 - t2, 1), t_wood=round(t4 - t3, 1), hdri=_hdri_path() is not None,
                 quality=quality or GRASS_QUALITY)
    return dict(terrain=terrain, grass=grass, sun=sun, world=world, wood=wood, stats=stats)


# =============================================================================================================
def set_render(scene, engine="EEVEE", preset="preview", motion_blur=False, samples=None):
    """Render settings. engine "EEVEE" (full animation) or "CYCLES" (hero stills); preset "preview" (960x540,
    low samples) or "final" (1920x1080). motion_blur: hook for build.py (shutter 0.5 frame)."""
    r = scene.render
    final = preset == "final"
    r.resolution_x, r.resolution_y = (1920, 1080) if final else (960, 540)
    r.resolution_percentage = 100
    r.fps, r.fps_base = 30, 1.0
    r.use_motion_blur = bool(motion_blur)
    r.motion_blur_shutter = 0.5
    r.film_transparent = False
    vs = scene.view_settings
    vs.view_transform = "AgX"
    vs.look = "AgX - Medium High Contrast"
    vs.exposure = EXPOSURE
    vs.gamma = 1.0
    r.image_settings.file_format = "PNG"
    r.image_settings.color_depth = "8"
    if engine.upper().startswith("EEVEE"):
        r.engine = "BLENDER_EEVEE"
        e = scene.eevee
        e.taa_render_samples = samples or (128 if final else 32)
        e.use_raytracing = True
        e.ray_tracing_method = "SCREEN"
        opt = e.ray_tracing_options
        for k, v in (("resolution_scale", "1" if final else "2"), ("use_denoise", True), ("trace_max_roughness", 0.5),
                     ("screen_trace_quality", 0.5 if final else 0.25), ("screen_trace_thickness", 0.2)):
            try:
                setattr(opt, k, v)
            except Exception:
                pass
        for k, v in (("use_shadows", True), ("shadow_ray_count", 2 if final else 1), ("shadow_step_count", 8 if final
                     else 6), ("shadow_resolution_scale", 1.0 if final else 0.5), ("use_fast_gi", True),
                     ("fast_gi_method", "GLOBAL_ILLUMINATION"), ("fast_gi_distance", 0.4), ("fast_gi_resolution",
                     "2" if final else "4"), ("fast_gi_ray_count", 2), ("fast_gi_step_count", 8),
                     ("use_volumetric_shadows", False), ("volumetric_tile_size", "8"),
                     ("clamp_surface_indirect", 10.0), ("motion_blur_steps", 1)):
            try:
                setattr(e, k, v)
            except Exception:
                pass
    else:
        r.engine = "CYCLES"
        c = scene.cycles
        c.device = "CPU"
        try:
            prefs = bpy.context.preferences.addons["cycles"].preferences
            for dev_type in ("METAL", "OPTIX", "CUDA", "HIP", "ONEAPI"):
                try:
                    prefs.compute_device_type = dev_type
                    prefs.get_devices()
                    gpus = [d for d in prefs.devices if d.type != "CPU"]
                    if gpus:
                        for d in prefs.devices:
                            d.use = True
                        c.device = "GPU"
                        break
                except Exception:
                    continue
        except Exception:
            pass
        c.samples = samples or (384 if final else 48)
        c.use_adaptive_sampling = True
        c.adaptive_threshold = 0.01 if final else 0.03
        c.adaptive_min_samples = 0
        c.use_denoising = True
        c.denoiser = "OPENIMAGEDENOISE"
        try:
            c.denoising_input_passes = "RGB_ALBEDO_NORMAL"
            c.denoising_prefilter = "ACCURATE"
        except Exception:
            pass
        c.max_bounces = 8
        c.diffuse_bounces = 3
        c.glossy_bounces = 3
        c.transmission_bounces = 4
        c.transparent_max_bounces = 8
        c.volume_bounces = 0
        c.caustics_reflective = False
        c.caustics_refractive = False
        c.sample_clamp_indirect = 8.0
        c.use_light_tree = True
        r.use_persistent_data = True
    return scene


# =============================================================================================================
def _add_cam(scene, name, loc, target, lens):
    from mathutils import Vector
    cd = bpy.data.cameras.new(name)
    cd.lens = lens
    cd.clip_start = 0.02
    cd.clip_end = 1000.0
    cd.sensor_width = 36.0
    ob = bpy.data.objects.new(name, cd)
    coll = _collection(scene, "CAMERAS")
    coll.objects.link(ob)
    ob.location = loc
    ob.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    return ob


PREVIEW_CAMS = {
    "": dict(loc=(-6.2, -9.2, 1.65), target=(4.0, 1.6, 0.35), lens=26.0),
    "_low": dict(loc=(3.05, -1.55, 0.17), target=(5.1, 0.35, 0.05), lens=30.0),
    "_wide": dict(loc=(-13.5, -22.0, 6.8), target=(4.5, 3.5, 0.0), lens=22.0),
    "_wood": dict(loc=(10.5, 4.5, 1.4), target=(12.5, 9.5, 0.6), lens=24.0),
    "_logs": dict(loc=(10.8, -2.2, 1.1), target=(14.3, 1.6, 0.25), lens=28.0),
}


def _peak_rss_mb():
    try:
        import resource
        return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
    except Exception:
        return -1.0


if __name__ == "__main__":
    import argparse
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    ap = argparse.ArgumentParser(description="Build the env test scene and render previews")
    ap.add_argument("--quality", default=GRASS_QUALITY, choices=list(GRASS_PRESETS))
    ap.add_argument("--engine", default="CYCLES", choices=["CYCLES", "EEVEE"])
    ap.add_argument("--samples", type=int, default=None)
    ap.add_argument("--views", default="main,low,wide")
    ap.add_argument("--percent", type=int, default=100)
    ap.add_argument("--suffix", default="")
    ap.add_argument("--save", default="", help="optional .blend path to save the test scene")
    ap.add_argument("--board", action="store_true", help="clear grass under the (absent) base board")
    a = ap.parse_args(argv)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.name = "env_test"
    t0 = time.time()
    res = build_env(scene, a.quality, board_clear=a.board)
    st = res["stats"]
    print("build %.1fs (mats %.1f terrain %.1f grass %.1f wood %.1f) hdri=%s" % (
        time.time() - t0, st["t_mats"], st["t_terrain"], st["t_grass"], st["t_wood"], st["hdri"]))
    print("grass:", st["grass"], "terrain tris:", st["terrain_tris"])
    print("flat zones max |z|: landing %.4f m, base %.4f m" % (st["flat_landing_max_m"], st["flat_base_max_m"]))
    set_render(scene, a.engine, "preview", samples=a.samples)
    scene.render.resolution_percentage = a.percent
    os.makedirs(OUT, exist_ok=True)
    names = {"main": "", "low": "_low", "wide": "_wide", "wood": "_wood", "logs": "_logs"}
    for v in a.views.split(","):
        key = names[v]
        c = PREVIEW_CAMS[key]
        cam = _add_cam(scene, "cam:env_preview%s" % key, c["loc"], c["target"], c["lens"])
        scene.camera = cam
        scene.render.filepath = os.path.join(OUT, "preview_env%s%s.png" % (key, a.suffix))
        t1 = time.time()
        bpy.ops.render.render(write_still=True)
        print("rendered %s in %.1fs, peak RSS %.0f MB" % (scene.render.filepath, time.time() - t1, _peak_rss_mb()))
    if a.save:
        bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(a.save))
    print("total %.1fs, peak RSS %.0f MB" % (time.time() - t0, _peak_rss_mb()))
