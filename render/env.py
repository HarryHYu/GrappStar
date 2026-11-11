"""GrappStar mission animation: ENVIRONMENT module (v2: Australian summer paddock).

build_env(scene) builds Harry's paddock around the world origin (centre of the base station's board top, launch
toward +X, landing patch x 3.5..6 m), as a rural Australian equestrian paddock in summer:
- gently undulating terrain, flat within +-2 cm at the base and the landing patch (SPEC), dusty hoof-trodden bare
  dirt patches, a worn track along the fence and to the trough, a bare shade camp under the paddock gum;
- geometry-nodes grass: mostly dry straw-coloured blades with some green at the base, grazed short sward, tussocks,
  wild-oat / barley-grass / ryegrass seed heads, three LODs, full density near the action, wind sway (animated);
- ground litter: gum leaves (sickle shaped, grey-green to red-brown), bark strips, twigs, sticks, small rocks and
  dried horse manure piles;
- eucalyptus gum trees (pale mottled trunks, sparse pendulous grey-green canopy, leaves sway), weathered timber
  post + plain wire fencing with steel star pickets (white safety caps) and a white electric sighter tape, a
  galvanised tube gate and trough, a fallen gum limb, a far corrugated-iron shed and round bales;
- Poly Haven "Qwantani Noon" HDRI (dry Highveld: clear deep-blue sky, high harsh sun at 50 deg, dusty haze on
  the horizon) with a matched sun lamp and aerial haze on distant objects; physical-sky fallback.
add_cockatoos(scene, fire_frame) adds sulphur-crested cockatoos: a flock foraging far off that takes off at the
fire event, plus a pair crossing the sky. set_render(scene, engine, preset) sets EEVEE / Cycles output.

Assets: render/assets/fetch_env.py downloads them to render/assets/cache (gitignored); without them the module
still builds (procedural fallbacks). Run `render/.venv/bin/python render/env.py` for previews from about where
cam:A and cam:B sit: render/out/preview_env.png (A) and preview_env_b.png (B); with
`--views A --suffix _uhdcrop --uhd-crop 0.40,0.22,0.65,0.47` a crop of cam:A at true 4K pixel density
(render/out/preview_env_uhdcrop.png) shows the grass as the 4K film will.

API for other modules: terrain_height(x, y) (numpy, m), sun_direction(), build_env(scene, quality,
clear_zones=[(x, y, r)], board_clear=True, board_half=(hx, hy)), add_cockatoos(scene, fire_frame, ...),
set_render(scene, "EEVEE"|"CYCLES", "preview"|"final"|"uhd", motion_blur=...). GRASS_QUALITY / GRASS_PRESETS trade
quality for speed.
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
# Grass quality presets. clumps_per_m2: clump instances per m2 inside r_near of GRASS_CENTER (a clump is 10 to 90
# blades); density eases to mid x that by r_full, then to far_min x at r_max where the grass stops (textured
# ground beyond); hero: extra density multiplier in HERO_ZONE (base, flight line, landing patch); clumps use LOD1
# (60 % of the blades, wider) beyond r_near and LOD2 (a third, twice as wide) beyond lod_r. litter: multiplier on
# the ground-litter density. "final" is meant for 4K EEVEE on a 16 GB M3.
GRASS_PRESETS = {
    "draft":   dict(clumps_per_m2=22, hero=1.5, r_near=5.0, mid=0.7, r_full=10.0, r_max=26.0, far_min=0.15,
                    lod_r=7.0, blade_mult=0.5, litter=0.3),
    "preview": dict(clumps_per_m2=45, hero=2.5, r_near=8.0, mid=0.75, r_full=16.0, r_max=40.0, far_min=0.08,
                    lod_r=12.0, blade_mult=0.75, litter=0.6),
    "final":   dict(clumps_per_m2=70, hero=3.4, r_near=12.0, mid=0.75, r_full=22.0, r_max=50.0, far_min=0.06,
                    lod_r=17.0, blade_mult=1.0, litter=1.0),
    "hero":    dict(clumps_per_m2=95, hero=3.5, r_near=14.0, mid=0.8, r_full=26.0, r_max=60.0, far_min=0.07,
                    lod_r=20.0, blade_mult=1.15, litter=1.3),
}
GRASS_QUALITY = "final"
GRASS_DENSITY = GRASS_PRESETS[GRASS_QUALITY]      # alias for the active preset
GRASS_CENTER = (2.5, 2.0)                          # between the base, the landing patch and the cameras' view
HERO_ZONE = (-3.5, 8.5, -8.0, 3.0)                 # x0, x1, y0, y1: extra-dense grass (action + the cameras'
#                                                    foreground, which look from -Y)


def _hero_sdf(x, y):
    x0, x1, y0, y1 = HERO_ZONE
    return _box_sdf(x, y - 0.5 * (y0 + y1), x0, x1, 0.5 * (y1 - y0))

# Wind (radians of instance tilt, noise scale 1/m, drift m/s). Animated through the scene time.
WIND = dict(grass_amp=0.05, grass_scale=0.22, leaf_amp=0.05, leaf_scale=0.08, speed=1.6, gust_scale=0.03)

# ---------------------------------------------------------------------------------------------------------------
# Layout (world metres). Cameras look roughly toward +Y, so the fence, trees and birds are on the +Y side.
FLAT_CORRIDOR = (-1.6, 6.8, 1.35)       # x0, x1, half width in y: kept within +-1.5 cm (flight line + landing)
BOARD_HALF = (0.65, 0.50)               # grass-free pad under the base board (half sizes, m)
BOARD_THICKNESS = 0.015                 # ground under the board sits this much below z 0
BOARD_CLEAR = True                      # keep grass from poking through the board (build_env(board_clear=...))
TERRAIN_CENTER = (4.0, 6.0)
TERRAIN_DENSE_HALF = 27.0               # dense mesh square half size (0.2 m grid), coarser out to TERRAIN_FAR
TERRAIN_STEP = 0.2
TERRAIN_FAR = 420.0

FENCE_BACK_Y = 13.0                     # timber post + wire fence behind the paddock, along x
FENCE_BACK_X = (-34.0, 46.0)
FENCE_SIDE_X = -18.0                    # side fence along y (left of the cameras)
FENCE_SIDE_Y = (-34.0, 13.0)
GATE_X = (9.2, 12.9)                    # galvanised tube gate in the back fence
POST_SPACING = 12.0                     # timber posts; star pickets every PICKET_SPACING between them
PICKET_SPACING = 3.0
WIRE_HEIGHTS = (0.30, 0.55, 0.80, 1.05)
TAPE_HEIGHT = 1.24                      # white electric sighter tape on top
TROUGH_XY = (14.4, 11.6)
# gum trees: (x, y, height m, seed). One shade tree inside the paddock, the rest beyond the back fence.
GUM_TREES = [(-7.5, 9.0, 15.0, 1), (5.5, 22.5, 14.0, 2), (-9.5, 19.5, 12.5, 3), (16.5, 18.5, 11.5, 4),
             (-1.5, 30.0, 16.0, 5), (11.0, 33.0, 14.5, 6), (-16.0, 27.0, 14.0, 7), (24.0, 27.0, 15.0, 8),
             (30.0, 40.0, 16.0, 9), (-24.0, 38.0, 15.0, 10), (3.0, 48.0, 17.0, 11), (-10.0, 55.0, 16.0, 12),
             (18.0, 58.0, 17.0, 13), (-30.0, 22.0, 13.0, 14)]
FALLEN_LIMBS = [dict(xy=(-10.5, 4.2), yaw=24.0, length=4.2, dia=0.34),
                dict(xy=(-4.6, 6.9), yaw=-52.0, length=3.2, dia=0.20)]
SHED = dict(xy=(-52.0, 98.0), yaw=8.0, size=(14.0, 9.0, 4.2))
BALES = [(22.0, 58.0, 10.0), (24.0, 59.5, 40.0), (20.2, 60.5, -20.0)]
MANURE = [(-9.5, -4.0), (-8.6, -3.2), (9.0, -6.5), (14.5, 4.0), (-3.0, 7.6), (12.0, 9.0), (-12.5, 9.5),
          (17.5, -3.0), (-14.0, -10.0), (3.0, -8.5), (-1.5, 10.8), (19.0, 8.0), (-6.0, -9.0), (7.5, 9.6)]

# ---------------------------------------------------------------------------------------------------------------
# Sky / sun. Poly Haven "Qwantani Noon" (CC0, dry Highveld grassland): clear deep-blue sky, high harsh sun
# (49.8 deg), dusty haze on a flat dry horizon. Values measured from the 4K file (brightest pixel; integral of the
# sun region above luminance 50 = sun irradiance at world strength 1; sky horizontal irradiance is 0.53, so the
# sun / sky ratio is about 10: crisp hard shadows).
HDRI_NAME = "qwantani_noon"
HDRI_SUN_U, HDRI_SUN_V = 0.60046, 0.77661
HDRI_SUN_IRR = 7.28
HDRI_SUN_RGB = (1.0, 0.93, 0.82)        # warm midday sun (the HDRI's sun pixels are clipped white)
HDRI_CLAMP = 20.0                       # removes the sun disc from the HDRI; the sun lamp replaces it
SUN_AZIMUTH_DEG = -25.0                 # direction TO the sun, CCW from +X: to the right of cameras looking +Y and
#                                         a little behind them: side light, so grass, posts and trunks throw crisp
#                                         shadows across the ground toward the left
SUN_STRENGTH = 9.5                      # W/m2: the HDRI's measured sun (7.3) pushed for a harsh Australian key
SUN_ANGLE_DEG = 0.55
WORLD_STRENGTH = 1.25                   # sky fill (shadows stay crisp: sun / sky about 10:1)
SKY_CAMERA_GAIN = 1.35                  # the sky as the camera sees it: brighter ...
SKY_CAMERA_SAT = 1.45                   # ... and bluer than the dusty HDRI
EXPOSURE = -0.6
HAZE_COLOR = (0.18, 0.20, 0.235)        # sky just above the horizon behind the paddock (measured)
HAZE_DIST = 1100.0                      # m: aerial perspective 1 - exp(-d / HAZE_DIST): only far things

TEX_SIZE = {"dry_ground_rocks": 4.0, "baseball_playground": 3.0, "park_dirt": 3.0, "bark_bluegum": 1.82,
            "rough_wood": 0.5}


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
    [(FENCE_SIDE_X + 1.6, FENCE_BACK_Y - 1.3), (-6.0, FENCE_BACK_Y - 1.0), (4.0, FENCE_BACK_Y - 1.4),
     (TROUGH_XY[0] - 1.0, FENCE_BACK_Y - 1.5), (FENCE_BACK_X[1], FENCE_BACK_Y - 1.2)],
    [(11.0, FENCE_BACK_Y - 0.4), (12.6, 8.0), (14.6, 1.5), (17.4, -6.0), (20.5, -16.0), (23.0, -30.0)],
    [(FENCE_SIDE_X + 1.3, FENCE_BACK_Y - 1.6), (FENCE_SIDE_X + 1.6, -6.0), (FENCE_SIDE_X + 1.2, FENCE_SIDE_Y[0])],
]
SHADE_TREE = GUM_TREES[0]               # horses camp under it: bare dusty ground


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
        return 0.55 * _nz(1).fbm(px / 48.0 + 3.1, py / 48.0 - 7.7, 3) + 0.15 * _nz(2).fbm(px / 15.0, py / 15.0, 2)

    roll = roll_fn(x, y) - roll_fn(np.array(0.0), np.array(0.0))
    d = np.hypot(x - cx, y - cy)
    rise = 0.004 * np.maximum(d - 60.0, 0.0) ** 1.06
    hum = (0.035 * _nz(3).fbm(x / 2.1, y / 2.1, 3) + 0.010 * _nz(4).fbm(x / 0.55, y / 0.55, 2))

    # worn horse tracks (bare only in places)
    td = np.full(x.shape, 1e9)
    for pts in TRACKS:
        td = np.minimum(td, _poly_dist(x, y, pts))
    td = td + 0.10 * _nz(5).fbm(x / 0.9, y / 0.9, 2)
    patchy = _sstep(-0.25, 0.25, _nz(12).fbm(x / 2.2, y / 2.2, 2))
    track = (1.0 - _sstep(0.10, 0.38, td)) * (0.45 + 0.55 * patchy)
    track_edge = 1.0 - _sstep(0.30, 0.95, td)
    # trodden bare ground: gateway, trough, shade camp under the paddock gum
    sx, sy = SHADE_TREE[0], SHADE_TREE[1]
    trod = np.maximum.reduce([
        _ellipse(x, y, 11.1, FENCE_BACK_Y - 1.7, 3.2, 2.1, rough=0.35, k=41),
        _ellipse(x, y, TROUGH_XY[0], TROUGH_XY[1] - 0.9, 2.6, 1.8, rough=0.35, k=42),
        _ellipse(x, y, sx + 0.8, sy - 0.6, 5.6, 4.3, rough=0.45, k=43)])
    # patchy bare dirt over the paddock (grazed out, rolled in), kept out of the flat corridor
    bare_n = _nz(6).fbm(x / 3.0 + 40, y / 3.0, 3) + 0.35 * _nz(16).fbm(x / 0.9, y / 0.9, 2)
    bare = _sstep(0.26, 0.52, bare_n) * (1 - 0.45 * flat) * (1 - _sstep(35, 48, d))
    scuff = _ellipse(x, y, 5.15, -0.42, 0.40, 0.28, rough=0.3, k=44) * 0.55    # hoof scuff by the landing patch
    soil = np.clip(np.maximum.reduce([track, trod, bare, scuff]), 0, 1)

    z = (roll * (1 - flat_w) + rise + hum * detail * (1 - 0.8 * flat) * (1 - pad)
         - (0.025 * track + 0.01 * track_edge) * detail - BOARD_THICKNESS * pad)

    # vegetation fields
    fence_d = np.minimum(np.abs(y - FENCE_BACK_Y) + 1e3 * ((x < FENCE_BACK_X[0]) | (x > FENCE_BACK_X[1])),
                         np.abs(x - FENCE_SIDE_X) + 1e3 * ((y < FENCE_SIDE_Y[0]) | (y > FENCE_SIDE_Y[1])))
    near_fence = 1.0 - _sstep(0.3, 1.1, fence_d)
    outside = _sstep(FENCE_BACK_Y + 0.5, FENCE_BACK_Y + 2.0, y)          # ungrazed beyond the back fence
    man_d = np.full(x.shape, 1e9)
    for mx, my in MANURE:
        man_d = np.minimum(man_d, np.hypot(x - mx, y - my))
    near_man = 1.0 - _sstep(0.4, 1.6, man_d)                          # horses leave tussocks round dung
    rough = _sstep(0.10, 0.45, _nz(8).fbm(x / 7.0 + 11, y / 7.0, 3))  # tall "roughs" vs grazed sward
    gh = 0.26 + 0.5 * rough + 0.2 * (_nz(9).fbm(x / 1.6, y / 1.6, 2) + 0.5)
    gh = np.maximum.reduce([gh, 0.95 * near_fence, 0.9 * near_man, outside * (0.62 + 0.3 * rough)])
    graze = 1.0 - _sstep(-0.5, 3.0, sd + 1.4 * _nz(14).fbm(x / 2.5, y / 2.5, 2))
    gh = gh + graze * (np.minimum(gh, 0.5) - gh)
    gh = gh * (1 - 0.5 * track_edge)
    padc = 0.0
    if BOARD_CLEAR:
        bsd = _box_sdf(x, y, -BOARD_HALF[0] + 0.05, BOARD_HALF[0] - 0.05, BOARD_HALF[1] - 0.05)
        padc = 1.0 - _sstep(-0.06, 0.08, bsd + 0.04 * _nz(13).fbm(x / 0.15, y / 0.15, 2))
    dens = (1 - 0.85 * soil) * (1 - 0.45 * track_edge) * (1 - padc) * (1 - 0.35 * scuff)
    # green: some in low spots, round the trough and in the shade; mostly dry everywhere
    green = np.clip(0.25 + 0.6 * _nz(10).fbm(x / 11.0, y / 11.0, 2) - 4.0 * z
                    + 0.6 * _ellipse(x, y, TROUGH_XY[0], TROUGH_XY[1] - 2.5, 4.0, 3.0, k=45), 0, 1)
    # litter density near the gums (crown radius ~ 0.35 x height)
    lit = np.zeros(x.shape)
    for gx, gy, gh_t, _s in GUM_TREES:
        rr = 0.38 * gh_t
        lit = np.maximum(lit, np.exp(-np.maximum(np.hypot(x - gx, y - gy) - 0.5 * rr, 0.0) / (0.55 * rr)))
    return dict(z=z, soil=soil, track=track_edge, trod=trod, gh=np.clip(gh, 0, 1), dens=np.clip(dens, 0, 1),
                green=green, flat=flat, graze=graze, near_fence=near_fence, pad=pad, lit=lit, outside=outside,
                board=1.0 - _sstep(0.0, 0.4, _box_sdf(x, y, -BOARD_HALF[0], BOARD_HALF[0], BOARD_HALF[1])))


def terrain_height(x, y):
    """Ground height (m) at world x, y (scalars or numpy arrays). Matches the terrain mesh within ~2 mm near
    the action (the mesh is a 0.2 m triangulation of this function)."""
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
         prand=None, flare=0.0, caps=(True, True), up=(0, 0, 1), bow=0.0, extra=None):
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
    b = mb.verts(P, prand=pr, **(extra or {}))
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
    for res in ("8k", "4k", "2k", "1k"):
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
        # what the camera sees of the sky: brighter and a deeper blue (the HDRI's dusty high-altitude sky reads
        # grey next to its own sun); lighting keeps the measured HDRI
        bgc = nt.node("ShaderNodeBackground")
        nt.set(bgc.inputs["Color"], nt.hsv(col, 0.5, SKY_CAMERA_SAT, 1.0))
        bgc.inputs["Strength"].default_value = WORLD_STRENGTH * SKY_CAMERA_GAIN
        lp = nt.node("ShaderNodeLightPath")
        nt.output(nt.mixsh(lp.outputs["Is Camera Ray"], bg.outputs[0], bgc.outputs[0]), "ShaderNodeOutputWorld")
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
            sky.aerosol_density = 3.0
        except Exception:
            pass
        bg = nt.node("ShaderNodeBackground")
        nt.t.links.new(sky.outputs["Color"], bg.inputs["Color"])
        bg.inputs["Strength"].default_value = 0.3 * WORLD_STRENGTH
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
    strength = SUN_STRENGTH if _hdri_path() else 5.0
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


def _hazed(nt, shader, scale=1.0):
    """Aerial perspective: mix toward the horizon sky colour by camera distance (camera rays only)."""
    cam = nt.node("ShaderNodeCameraData")
    d = nt.math("DIVIDE", cam.outputs["View Distance"], -HAZE_DIST / scale)
    fac = nt.math("SUBTRACT", 1.0, nt.math("EXPONENT", d))
    lp = nt.node("ShaderNodeLightPath")
    fac = nt.math("MULTIPLY", fac, lp.outputs["Is Camera Ray"])
    em = nt.node("ShaderNodeEmission")
    em.inputs["Color"].default_value = tuple(c * WORLD_STRENGTH * SKY_CAMERA_GAIN for c in HAZE_COLOR) + (1.0,)
    return nt.mixsh(fac, shader, em.outputs[0])


# =============================================================================================================
# materials
def _thin(m):
    for k, v in (("thickness_mode", "SLAB"), ("use_backface_culling", False)):
        try:
            setattr(m, k, v)
        except Exception:
            pass


def _nmap(nt, col, strength):
    if col is None:
        return None
    nm = nt.node("ShaderNodeNormalMap", uv_map="UVMap")
    nt.set(nm.inputs["Color"], col)
    nt.set(nm.inputs["Strength"], strength)
    return nm.outputs["Normal"]


def _mat_ground(hdri_img):
    """Dry dusty Australian paddock ground: pebbly soil, trodden dust with hoof prints, dead-grass thatch tone
    where the grass is, dry-paddock colour in the distance, haze and a seamless blend into the HDRI horizon."""
    m, nt = _new_mat("env:ground")
    uv = nt.node("ShaderNodeUVMap", uv_map="UVMap").outputs["UV"]
    geo = nt.node("ShaderNodeNewGeometry")
    pos = geo.outputs["Position"]
    soil_a = nt.attr("soil").outputs["Fac"]
    cov_a = nt.attr("cov").outputs["Fac"]
    trod_a = nt.attr("trod").outputs["Fac"]
    brk = nt.noise(pos, 1.5, 4.0, 0.6).outputs["Fac"]
    brk2 = nt.noise(pos, 6.0, 2.0, 0.5).outputs["Fac"]
    macro = nt.noise(pos, 0.06, 3.0, 0.55).outputs["Fac"]
    macro2 = nt.noise(pos, 0.25, 2.0, 0.5).outputs["Fac"]

    g_c, g_r, g_n, _ = _pbr(nt, "dry_ground_rocks", _uvvec(nt, uv, TEX_SIZE["dry_ground_rocks"], 0.3),
                           ((0.30, 0.20, 0.11), 0.9))
    p_c, p_r, p_n, _ = _pbr(nt, "park_dirt", _uvvec(nt, uv, TEX_SIZE["park_dirt"], 1.2, (0.3, 0.6)),
                           ((0.24, 0.18, 0.09), 0.9))
    t_c, t_r, t_n, _ = _pbr(nt, "baseball_playground", _uvvec(nt, uv, 1.9, 2.1), ((0.30, 0.23, 0.15), 0.9))
    mixgp = nt.ss(nt.math("ADD", brk, nt.math("MULTIPLY", macro2, 0.5)), 0.45, 0.8)
    soil = nt.mix(mixgp, g_c, p_c)
    soil = nt.hsv(soil, nt.mixf(macro, 0.49, 0.51), nt.mixf(macro2, 0.9, 1.15), nt.mixf(macro, 0.85, 1.12))
    rough = nt.mixf(mixgp, g_r, p_r)
    ncol = nt.mix(mixgp, g_n, p_n) if (g_n is not None and p_n is not None) else g_n
    # dead grass thatch: straw-tone the soil under the grass, broken up
    thatch = nt.mix(nt.ss(brk2, 0.3, 0.8), (0.16, 0.135, 0.085), (0.30, 0.26, 0.17))
    thm = nt.math("MULTIPLY", nt.ss(cov_a, 0.15, 0.8), nt.mixf(nt.ss(brk, 0.3, 0.7), 0.6, 0.95))
    col = nt.mix(thm, soil, nt.mix(0.45, thatch, soil, "MULTIPLY"))
    col = nt.mix(nt.math("MULTIPLY", thm, 0.6), col, thatch)
    # trodden dust (hoof prints) on bare ground
    edge = nt.math("ADD", nt.math("MULTIPLY", nt.math("SUBTRACT", brk, 0.5), 0.6),
                   nt.math("MULTIPLY", nt.math("SUBTRACT", brk2, 0.5), 0.4))
    soil_m = nt.ss(nt.math("ADD", soil_a, edge), 0.35, 0.65)
    dust = nt.hsv(nt.mix(1.0, t_c, (1.0, 0.97, 0.92), "MULTIPLY"), 0.5, 0.6, nt.mixf(trod_a, 0.82, 0.95))
    dust = nt.mix(nt.math("MULTIPLY", nt.ss(brk2, 0.45, 0.75), 0.55), dust, g_c)          # pebbly grit
    col = nt.mix(nt.math("MULTIPLY", soil_m, 0.85), col, dust)
    rough = nt.mixf(soil_m, rough, t_r)
    if ncol is not None and t_n is not None:
        ncol = nt.mix(soil_m, ncol, t_n)
    # distance: dry-paddock colour where the grass instances thin out
    cam = nt.node("ShaderNodeCameraData")
    dist = cam.outputs["View Distance"]
    far_col = nt.mix(nt.ss(macro2, 0.25, 0.75), (0.36, 0.31, 0.19), (0.48, 0.42, 0.27))
    far_col = nt.mix(nt.ss(macro, 0.3, 0.8), nt.mix(0.35, far_col, (0.20, 0.17, 0.10)), far_col)
    far_col = nt.mix(nt.math("MULTIPLY", nt.ss(brk2, 0.3, 0.8), 0.4), far_col,
                     nt.mix(1.0, far_col, (0.75, 0.75, 0.7), "MULTIPLY"))
    farf = nt.math("MULTIPLY", nt.ss(dist, 18.0, 70.0, 0.0, 0.9), nt.math("SUBTRACT", 1.0, soil_m))
    col = nt.mix(farf, col, far_col)
    normal = _nmap(nt, ncol, nt.mixf(soil_m, 0.45, 1.1))
    vor = nt.node("ShaderNodeTexVoronoi")
    nt.set(vor.inputs["Vector"], pos)
    vor.inputs["Scale"].default_value = 6.5
    vor.inputs["Randomness"].default_value = 1.0
    dent = nt.math("MULTIPLY", nt.ss(vor.outputs["Distance"], 0.05, 0.42), soil_m)
    bump = nt.node("ShaderNodeBump")
    nt.set(bump.inputs["Height"], nt.math("ADD", dent, nt.math("MULTIPLY", brk2, 0.3)))
    bump.inputs["Strength"].default_value = 0.55
    bump.inputs["Distance"].default_value = 0.02
    if normal is not None:
        nt.t.links.new(normal, bump.inputs["Normal"])
    normal = bump.outputs["Normal"]
    bsdf = nt.principled(col, nt.math("MAXIMUM", rough, 0.65), normal, spec=0.25)
    shader = _hazed(nt, bsdf)
    # far ground fades into the horizon haze (heat haze) so the terrain edge never shows
    em = nt.node("ShaderNodeEmission")
    em.inputs["Color"].default_value = tuple(0.85 * WORLD_STRENGTH * SKY_CAMERA_GAIN * (0.5 * c + 0.5 * g) for c, g
                                             in zip(HAZE_COLOR, (0.21, 0.16, 0.12))) + (1,)    # dusty ground haze
    lp = nt.node("ShaderNodeLightPath")
    fac = nt.math("MULTIPLY", nt.ss(dist, 120.0, 400.0, 0.0, 0.85), lp.outputs["Is Camera Ray"])
    shader = nt.mixsh(fac, shader, em.outputs[0])
    nt.output(shader)
    return m


def _mat_grass():
    """Dry summer grass: straw / golden / weathered grey blades, green toward the base on some (attribute grn =
    green fraction from the base), darker at the root, warm translucency."""
    m, nt = _new_mat("env:grass_blade")
    h = nt.attr("h01").outputs["Fac"]
    br = nt.attr("brnd").outputs["Fac"]
    dry = nt.attr("dry").outputs["Fac"]
    grn = nt.attr("grn").outputs["Fac"]
    tint = nt.attr("tint", "INSTANCER").outputs["Fac"]
    igrn = nt.attr("igrn", "INSTANCER").outputs["Fac"]
    oi = nt.node("ShaderNodeObjectInfo")
    straw = nt.ramp(br, [(0.0, (0.50, 0.43, 0.26)), (0.3, (0.56, 0.49, 0.31)), (0.55, (0.50, 0.40, 0.20)),
                         (0.8, (0.44, 0.41, 0.33)), (1.0, (0.38, 0.30, 0.17))])
    straw = nt.mix(dry, straw, nt.mix(0.5, straw, (0.36, 0.33, 0.27)))            # grey dead / weathered
    straw = nt.mix(nt.ss(tint, 0.0, 1.0), nt.mix(0.25, straw, (0.40, 0.31, 0.16)),
                   nt.mix(0.3, straw, (0.62, 0.57, 0.42)))                        # warmer / bleached patches
    straw = nt.hsv(straw, nt.mixf(oi.outputs["Random"], 0.49, 0.51), nt.mixf(oi.outputs["Random"], 0.85, 1.1),
                   nt.mixf(oi.outputs["Random"], 0.85, 1.1))
    green = nt.ramp(br, [(0.0, (0.05, 0.10, 0.025)), (0.6, (0.10, 0.14, 0.035)), (1.0, (0.17, 0.18, 0.06))])
    gfrac = nt.math("ADD", grn, nt.math("MULTIPLY", igrn, 0.2))
    gness = nt.math("SUBTRACT", 1.0, nt.ss(nt.math("SUBTRACT", h, gfrac), -0.12, 0.06))
    gness = nt.math("MULTIPLY", gness, nt.ss(gfrac, 0.0, 0.08))
    col = nt.mix(gness, straw, green)
    shade = nt.ramp(h, [(0.0, (0.25, 0.24, 0.20)), (0.18, (0.62, 0.60, 0.55)), (0.5, (1, 1, 1)),
                        (1.0, (1.08, 1.06, 1.0))])
    col = nt.mix(1.0, col, shade, "MULTIPLY")
    bsdf = nt.principled(col, nt.mixf(gness, 0.5, 0.45), spec=0.35)
    tr = nt.node("ShaderNodeBsdfTranslucent")
    nt.set(tr.inputs["Color"], nt.mix(0.4, col, (0.60, 0.48, 0.16), "MIX"))
    nt.output(_hazed(nt, nt.mixsh(0.28, bsdf, tr.outputs[0])))
    _thin(m)
    return m


def _mat_leafy(name, cols, rough=0.45, trans=0.25, spec=0.4, attr="brnd", haze=True):
    m, nt = _new_mat(name)
    br = nt.attr(attr).outputs["Fac"]
    oi = nt.node("ShaderNodeObjectInfo")
    col = nt.ramp(br, cols)
    col = nt.hsv(col, nt.mixf(oi.outputs["Random"], 0.49, 0.51), 1.0, nt.mixf(oi.outputs["Random"], 0.85, 1.12))
    bsdf = nt.principled(col, rough, spec=spec)
    if trans > 0:
        tr = nt.node("ShaderNodeBsdfTranslucent")
        nt.set(tr.inputs["Color"], col)
        bsdf = nt.mixsh(trans, bsdf, tr.outputs[0])
    nt.output(_hazed(nt, bsdf) if haze else bsdf)
    _thin(m)
    return m


def _mat_gum_canopy():
    """Eucalyptus leaves on the tree: grey-green, waxy, a few yellow / red older leaves, translucent."""
    return _mat_leafy("env:gum_leaf", [(0.0, (0.10, 0.14, 0.085)), (0.35, (0.14, 0.17, 0.11)),
                                       (0.7, (0.18, 0.20, 0.13)), (0.9, (0.22, 0.21, 0.12)),
                                       (1.0, (0.30, 0.16, 0.08))], rough=0.42, trans=0.3, spec=0.5, attr="lcol")


def _mat_leaf_litter():
    """Fallen gum leaves: fresh grey-green, yellow, orange-brown, dark brown, weathered grey."""
    return _mat_leafy("env:gum_leaf_litter", [(0.0, (0.15, 0.16, 0.10)), (0.2, (0.30, 0.26, 0.12)),
                                              (0.45, (0.32, 0.17, 0.07)), (0.7, (0.17, 0.10, 0.05)),
                                              (1.0, (0.30, 0.27, 0.22))], rough=0.6, trans=0.15, spec=0.3,
                      attr="lcol", haze=False)


def _mat_gum_bark():
    """Pale smooth gum trunk (spotted / manna gum look): blue gum relief, re-toned cream-grey with grey and
    salmon patches and a rougher darker stocking at the base."""
    m, nt = _new_mat("env:gum_bark")
    uv = nt.node("ShaderNodeUVMap", uv_map="UVMap").outputs["UV"]
    geo = nt.node("ShaderNodeNewGeometry")
    pr = nt.attr("prand").outputs["Fac"]
    c, r, n, _ = _pbr(nt, "bark_bluegum", _uvvec(nt, uv, 1.0), ((0.4, 0.38, 0.33), 0.6))
    sep = nt.node("ShaderNodeSeparateColor")
    nt.set(sep.inputs["Color"], c)
    lum = nt.math("MULTIPLY", nt.math("ADD", nt.math("ADD", sep.outputs["Red"], sep.outputs["Green"]),
                                      sep.outputs["Blue"]), 0.33)
    pos = geo.outputs["Position"]
    patch = nt.noise(pos, 1.3, 3.0, 0.55, 0.6).outputs["Fac"]
    patch2 = nt.noise(pos, 4.0, 2.0, 0.5).outputs["Fac"]
    base = nt.mix(nt.ss(patch, 0.38, 0.62), (0.62, 0.60, 0.55), (0.47, 0.46, 0.43))
    base = nt.mix(nt.math("MULTIPLY", nt.ss(patch2, 0.6, 0.72), 0.7), base, (0.62, 0.44, 0.32))   # salmon
    base = nt.mix(nt.math("MULTIPLY", nt.ss(patch2, 0.25, 0.32), 0.5), base, (0.36, 0.38, 0.30))  # grey-green
    base = nt.hsv(base, 0.5, nt.mixf(pr, 0.8, 1.15), nt.mixf(pr, 0.85, 1.05))
    col = nt.mix(1.0, base, nt.ramp(lum, [(0.0, (0.45, 0.45, 0.45)), (0.35, (1.0, 1.0, 1.0)),
                                          (1.0, (1.25, 1.25, 1.25))]), "MULTIPLY")
    sepz = nt.node("ShaderNodeSeparateXYZ")
    nt.set(sepz.inputs["Vector"], pos)
    stock = nt.math("MULTIPLY", nt.attr("stock").outputs["Fac"], nt.ss(patch2, 0.2, 0.5))
    col = nt.mix(stock, col, nt.mix(1.0, c, (0.9, 0.85, 0.8), "MULTIPLY"))
    normal = _nmap(nt, n, nt.mixf(stock, 0.8, 1.6))
    nt.output(_hazed(nt, nt.principled(col, nt.math("MAXIMUM", r, 0.55), normal, spec=0.3)))
    return m


def _mat_bark_strip():
    """Shed bark ribbons: weathered grey-tan outside, salmon / orange-brown inside."""
    m, nt = _new_mat("env:bark_strip")
    geo = nt.node("ShaderNodeNewGeometry")
    br = nt.attr("lcol").outputs["Fac"]
    out_c = nt.ramp(br, [(0.0, (0.30, 0.27, 0.22)), (1.0, (0.19, 0.16, 0.12))])
    in_c = nt.ramp(br, [(0.0, (0.36, 0.24, 0.15)), (1.0, (0.25, 0.15, 0.09))])
    fib = nt.noise(geo.outputs["Position"], 40.0, 3.0, 0.6)
    col = nt.mix(geo.outputs["Backfacing"], out_c, in_c)
    col = nt.mix(nt.math("MULTIPLY", nt.ss(fib.outputs["Fac"], 0.4, 0.7), 0.5), col,
                 nt.mix(1.0, col, (0.7, 0.68, 0.65), "MULTIPLY"))
    nt.output(nt.principled(col, 0.75, spec=0.25))
    _thin(m)
    return m


def _mat_deadwood(name="env:deadwood"):
    """Weathered grey eucalyptus wood: sticks, twigs, fallen limbs (bark mostly shed)."""
    m, nt = _new_mat(name)
    uv = nt.node("ShaderNodeUVMap", uv_map="UVMap").outputs["UV"]
    geo = nt.node("ShaderNodeNewGeometry")
    pr = nt.attr("prand").outputs["Fac"]
    c, r, n, _ = _pbr(nt, "bark_bluegum", _uvvec(nt, uv, 0.7), ((0.36, 0.33, 0.29), 0.7))
    sep = nt.node("ShaderNodeSeparateColor")
    nt.set(sep.inputs["Color"], c)
    lum = nt.math("MULTIPLY", nt.math("ADD", sep.outputs["Red"], sep.outputs["Green"]), 0.5)
    base = nt.mix(pr, (0.40, 0.37, 0.32), (0.30, 0.25, 0.19))
    patch = nt.noise(geo.outputs["Position"], 3.0, 3.0, 0.5).outputs["Fac"]
    base = nt.mix(nt.math("MULTIPLY", nt.ss(patch, 0.6, 0.7), 0.6), base, (0.42, 0.30, 0.20))
    col = nt.mix(1.0, base, nt.ramp(lum, [(0.0, (0.4, 0.4, 0.4)), (0.4, (1, 1, 1)), (1.0, (1.3, 1.3, 1.3))]),
                 "MULTIPLY")
    nt.output(_hazed(nt, nt.principled(col, 0.8, _nmap(nt, n, 1.2), spec=0.25)))
    return m


def _mat_rock():
    m, nt = _new_mat("env:rock")
    geo = nt.node("ShaderNodeNewGeometry")
    br = nt.attr("lcol").outputs["Fac"]
    col = nt.ramp(br, [(0.0, (0.30, 0.16, 0.08)), (0.4, (0.24, 0.18, 0.13)), (0.75, (0.36, 0.33, 0.30)),
                       (1.0, (0.60, 0.57, 0.52))])           # ironstone, brown, grey, quartz
    nz = nt.noise(geo.outputs["Position"], 30.0, 5.0, 0.65)
    col = nt.mix(nt.math("MULTIPLY", nt.ss(nz.outputs["Fac"], 0.35, 0.7), 0.6),
                 nt.mix(1.0, col, (0.6, 0.6, 0.6), "MULTIPLY"), col)
    dust = nt.ss(nt.vmath("DOT_PRODUCT", geo.outputs["Normal"], (0, 0, 1)), 0.4, 0.95)
    col = nt.mix(nt.math("MULTIPLY", dust, 0.45), col, (0.40, 0.31, 0.20))
    bump = nt.node("ShaderNodeBump")
    nt.set(bump.inputs["Height"], nz.outputs["Fac"])
    bump.inputs["Strength"].default_value = 0.5
    bump.inputs["Distance"].default_value = 0.004
    nt.output(nt.principled(col, 0.75, bump.outputs["Normal"], spec=0.35))
    return m


def _mat_manure():
    m, nt = _new_mat("env:manure")
    geo = nt.node("ShaderNodeNewGeometry")
    br = nt.attr("lcol").outputs["Fac"]
    fib = nt.noise(geo.outputs["Position"], 120.0, 4.0, 0.7)
    col = nt.ramp(br, [(0.0, (0.20, 0.17, 0.10)), (0.5, (0.28, 0.24, 0.15)), (1.0, (0.36, 0.33, 0.25))])
    col = nt.mix(nt.math("MULTIPLY", nt.ss(fib.outputs["Fac"], 0.35, 0.75), 0.6), col,
                 nt.mix(0.6, col, (0.42, 0.36, 0.22)))
    bump = nt.node("ShaderNodeBump")
    nt.set(bump.inputs["Height"], fib.outputs["Fac"])
    bump.inputs["Strength"].default_value = 0.8
    bump.inputs["Distance"].default_value = 0.003
    nt.output(nt.principled(col, 0.9, bump.outputs["Normal"], spec=0.2))
    return m


def _mat_timber():
    """Weathered grey round timber posts (rough_wood), darker stained at the ground line."""
    m, nt = _new_mat("env:timber")
    uv = nt.node("ShaderNodeUVMap", uv_map="UVMap").outputs["UV"]
    geo = nt.node("ShaderNodeNewGeometry")
    pr = nt.attr("prand").outputs["Fac"]
    c, r, n, _ = _pbr(nt, "rough_wood", uv, ((0.33, 0.30, 0.26), 0.8))
    c = nt.hsv(c, nt.mixf(pr, 0.49, 0.53), nt.mixf(pr, 0.7, 1.0), nt.mixf(pr, 0.85, 1.25))
    c = nt.mix(0.25, c, (0.46, 0.40, 0.32), "MIX")
    sep = nt.node("ShaderNodeSeparateXYZ")
    nt.set(sep.inputs["Vector"], geo.outputs["Position"])
    gz = nt.attr("gz").outputs["Fac"]
    zfac = nt.ss(nt.math("SUBTRACT", sep.outputs["Z"], gz), 0.25, 0.0)
    c = nt.mix(nt.math("MULTIPLY", zfac, 0.6), c, nt.mix(1.0, c, (0.55, 0.48, 0.40), "MULTIPLY"))
    nt.output(_hazed(nt, nt.principled(c, r, _nmap(nt, n, 1.3), spec=0.3)))
    return m


def _mat_metal(name, col, rough, metal=1.0, spangle=False):
    m, nt = _new_mat(name)
    c = col
    rr = rough
    if spangle:
        tc = nt.node("ShaderNodeTexCoord")
        sp = nt.node("ShaderNodeTexVoronoi")
        nt.set(sp.inputs["Vector"], tc.outputs["Object"])
        sp.inputs["Scale"].default_value = 30.0
        c = nt.mix(nt.ss(sp.outputs["Distance"], 0.1, 0.6), [x * 0.82 for x in col], col)
        rr = nt.mixf(sp.outputs["Distance"], rough * 0.8, rough * 1.25)
    nt.output(_hazed(nt, nt.principled(c, rr, spec=0.5, metal=metal)))
    return m


def _mat_plain(name, col, rough, spec=0.4, trans=0.0):
    m, nt = _new_mat(name)
    sh = nt.principled(col, rough, spec=spec)
    if trans:
        tr = nt.node("ShaderNodeBsdfTranslucent")
        tr.inputs["Color"].default_value = tuple(col) + (1.0,)
        sh = nt.mixsh(trans, sh, tr.outputs[0])
    nt.output(_hazed(nt, sh))
    return m


def _mat_zincalume():
    m, nt = _new_mat("env:zincalume")
    tc = nt.node("ShaderNodeTexCoord")
    wave = nt.node("ShaderNodeTexWave", wave_type="BANDS", bands_direction="X")
    nt.set(wave.inputs["Vector"], tc.outputs["Object"])
    wave.inputs["Scale"].default_value = 6.0
    bump = nt.node("ShaderNodeBump")
    nt.set(bump.inputs["Height"], wave.outputs["Fac"])
    bump.inputs["Strength"].default_value = 0.6
    bump.inputs["Distance"].default_value = 0.01
    grime = nt.noise(tc.outputs["Object"], 0.4, 3.0, 0.5).outputs["Fac"]
    col = nt.mix(nt.ss(grime, 0.4, 0.7), (0.22, 0.23, 0.22), (0.17, 0.165, 0.15))
    nt.output(_hazed(nt, nt.principled(col, 0.55, bump.outputs["Normal"], spec=0.4, metal=0.7)))
    return m


def _mat_straw():
    m, nt = _new_mat("env:hay")
    tc = nt.node("ShaderNodeTexCoord")
    obj = tc.outputs["Object"]
    fib = nt.noise(obj, 60.0, 8.0, 0.7, 0.4)
    coarse = nt.noise(obj, 3.5, 3.0, 0.5)
    col = nt.mix(nt.ss(fib.outputs["Fac"], 0.25, 0.8), (0.22, 0.17, 0.08), (0.48, 0.38, 0.18))
    col = nt.mix(nt.ss(coarse.outputs["Fac"], 0.35, 0.75), nt.mix(0.5, col, (0.30, 0.27, 0.20)), col)
    bump = nt.node("ShaderNodeBump")
    nt.set(bump.inputs["Height"], fib.outputs["Fac"])
    bump.inputs["Strength"].default_value = 0.9
    bump.inputs["Distance"].default_value = 0.02
    nt.output(_hazed(nt, nt.principled(col, 0.85, bump.outputs["Normal"], spec=0.25)))
    return m


def _mat_water():
    m, nt = _new_mat("env:water")
    tc = nt.node("ShaderNodeTexCoord")
    nz = nt.noise(tc.outputs["Object"], 6.0, 3.0, 0.5)
    bump = nt.node("ShaderNodeBump")
    nt.set(bump.inputs["Height"], nz.outputs["Fac"])
    bump.inputs["Strength"].default_value = 0.08
    nt.output(nt.principled((0.03, 0.035, 0.02), 0.05, bump.outputs["Normal"], spec=0.5))
    return m


def _mat_feather():
    """Cockatoo plumage: white, slightly warm, pale sulphur wash under the wings (attribute 'under')."""
    m, nt = _new_mat("env:cockatoo_white")
    und = nt.attr("under").outputs["Fac"]
    col = nt.mix(und, (0.78, 0.78, 0.75), (0.80, 0.72, 0.42))
    sh = nt.principled(col, 0.6, spec=0.3)
    tr = nt.node("ShaderNodeBsdfTranslucent")
    tr.inputs["Color"].default_value = (0.8, 0.78, 0.7, 1.0)
    nt.output(_hazed(nt, nt.mixsh(0.12, sh, tr.outputs[0])))
    return m


def _strip(mb, rng, base, L, W, segs, lean, bend, phi, brnd, dry, width_fn=None, twist=0.0, mat=0, fold=0.0,
           ret_cl=False, grn=-0.2):
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
    b = mb.verts(np.vstack([P, cl[-1]]), h01=np.append(hv, 1.0), brnd=brnd, dry=dry, grn=grn)
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


def _stalk_frame(cl):
    ax = cl[-1] - cl[-3]
    ax = ax / np.linalg.norm(ax)
    s1 = np.cross(ax, [0.3, 0.7, 0.1])
    s1 /= np.linalg.norm(s1)
    return ax, s1, np.cross(ax, s1)


def _seed_stalk(mb, rng, base, H, kind):
    """Seed stalk (culm) with a head: 'oat' (wild oat panicle, pendulous spikelets), 'barley' (barley grass:
    bristly cylindrical spike with awns) or 'rye' (ryegrass: flat alternating spikelets)."""
    U = rng.uniform
    phi = rng.random() * 2 * math.pi
    lean = U(0.03, 0.3)
    cl = _strip(mb, rng, base, H, 0.0022, 8, lean, U(0.05, 0.35), phi, rng.random(), U(0.3, 1.0),
                width_fn=lambda t: np.ones_like(t) * (1.0 - 0.4 * t), ret_cl=True)
    top = cl[-1]
    ax, s1, s2 = _stalk_frame(cl)
    br = rng.random()
    if kind == "oat":
        nn = int(rng.integers(4, 7))
        for q in range(nn):
            node = top - ax * (0.035 * q)
            for k in range(int(rng.integers(1, 3))):
                a = rng.random() * 2 * math.pi
                out = s1 * math.cos(a) + s2 * math.sin(a)
                pl = U(0.025, 0.05)
                pe = node + out * pl * 0.8 - ax * pl * 0.2 + np.array([0, 0, -0.01])
                _ribbon(mb, node, pe, 0.0012, np.cross(out, [0, 0, 1.0]) + 1e-6, br, 1)
                sl = U(0.018, 0.026)
                tip = pe + np.array([out[0] * 0.25, out[1] * 0.25, -1.0]) * sl
                for kk in range(2):
                    sv = np.cross(tip - pe, [math.cos(a + kk * 1.57), math.sin(a + kk * 1.57), 0.0])
                    _ribbon(mb, pe, tip, 0.0042, sv + 1e-6, br, 1)
    elif kind == "barley":
        hl = U(0.04, 0.07)
        for k in range(3):
            ang = k * math.pi / 3
            sv = s1 * math.cos(ang) + s2 * math.sin(ang)
            _ribbon(mb, top - ax * hl, top + ax * 0.004, 0.007, sv, br, 1)
        for k in range(int(rng.integers(8, 14))):
            a = rng.random() * 2 * math.pi
            p = top - ax * hl * rng.random()
            out = s1 * math.cos(a) + s2 * math.sin(a)
            _ribbon(mb, p, p + (ax * 0.9 + out * 0.35) * U(0.04, 0.07), 0.0008, np.cross(ax, out) + 1e-6, br, 1)
    else:
        ns = int(rng.integers(7, 12))
        for q in range(ns):
            b = top - ax * (0.013 * q + 0.004)
            sgn = 1 if q % 2 else -1
            tv = ax * 0.012 + s1 * sgn * 0.005
            _ribbon(mb, b, b + tv, 0.0034, np.cross(tv, s2), br, 1)
    return top


# kind -> blade spec. grnp: probability a blade has a green base; grn: green fraction range for those.
GRASS_SPEC = {
    "short": dict(n=60, L=(0.025, 0.09), W=(0.0024, 0.0042), R=0.075, segs=3, lean=(0.3, 1.35), bend=(0.3, 1.6),
                  pdry=0.22, grnp=0.55, grn=(0.2, 0.7), fold=0.0),
    "mid": dict(n=44, L=(0.07, 0.25), W=(0.0024, 0.0044), R=0.07, segs=4, lean=(0.15, 1.1), bend=(0.5, 1.9),
                pdry=0.25, grnp=0.45, grn=(0.15, 0.45), fold=0.0),
    "tussock": dict(n=78, L=(0.16, 0.55), W=(0.002, 0.0042), R=0.05, segs=6, lean=(0.04, 0.62),
                    bend=(0.6, 2.2), pdry=0.3, grnp=0.45, grn=(0.1, 0.38), fold=0.22),
    "green": dict(n=50, L=(0.03, 0.12), W=(0.003, 0.0055), R=0.08, segs=3, lean=(0.2, 1.2), bend=(0.3, 1.4),
                  pdry=0.05, grnp=0.95, grn=(0.5, 1.1), fold=0.0),
}
GRASS_KINDS = ["short", "mid", "tussock"]


def _make_clump(kind, rng, mult=1.0, lod=0):
    mb = MB()
    U = rng.uniform
    if kind in GRASS_SPEC:
        s = GRASS_SPEC[kind]
        n = max(5, int(round(s["n"] * mult * (1.0, 0.6, 0.33)[lod])))
        segs = min(s["segs"], (9, 4, 3)[lod])
        wmul = (1.0, 1.35, 2.0)[lod]
        fold = s["fold"] if lod == 0 else 0.0
        for i in range(n):
            r = s["R"] * math.sqrt(rng.random())
            a = rng.random() * 2 * math.pi
            base = np.array([r * math.cos(a), r * math.sin(a), -0.012])
            phi = a + rng.normal(0, 0.7)
            Lb = U(*s["L"])
            lean = U(*s["lean"])
            if kind == "tussock":           # longest blades arch out most, core stays upright
                Lb = s["L"][0] + (s["L"][1] - s["L"][0]) * rng.random() ** 0.8
                lean = lean * (0.5 + Lb / s["L"][1])
            if rng.random() < 0.12:
                Lb *= 0.45                  # grazed / broken
            dry = 1.0 if rng.random() < s["pdry"] else 0.0
            grn = U(*s["grn"]) if rng.random() < s["grnp"] else -0.2
            if kind == "tussock" and lean < 0.25:
                grn = max(grn, U(0.1, 0.4))
            _strip(mb, rng, base, Lb, U(*s["W"]) * wmul, segs, lean, U(*s["bend"]), phi, rng.random(), dry,
                   twist=rng.normal(0, 0.9), fold=fold, grn=grn)
        if lod == 0 and kind != "green":
            for i in range(rng.integers(3, 8)):      # flattened dead blades in the thatch
                a = rng.random() * 2 * math.pi
                _strip(mb, rng, np.array([0, 0, -0.004]), U(0.06, 0.2), U(0.0025, 0.0045), 3, U(1.3, 1.5),
                       U(0.0, 0.15), a, rng.random(), 1.0)
        if kind == "mid" and lod == 0 and rng.random() < 0.5:
            for i in range(rng.integers(1, 3)):
                _seed_stalk(mb, rng, np.array([0.02, 0.0, -0.01]), U(0.22, 0.42), rng.choice(["rye", "barley"]))
        if kind == "tussock" and lod == 0:
            for i in range(rng.integers(2, 6)):
                _seed_stalk(mb, rng, np.array([0.015 * rng.normal(), 0.015 * rng.normal(), -0.01]),
                            U(0.5, 0.85), rng.choice(["oat", "rye"]))
    elif kind in ("oat", "barley", "rye"):
        for i in range(int(12 * mult)):
            a = rng.random() * 2 * math.pi
            _strip(mb, rng, np.array([0.03 * math.cos(a), 0.03 * math.sin(a), -0.012]), U(0.08, 0.22),
                   U(0.0025, 0.004), 4, U(0.2, 1.0), U(0.5, 1.6), a, rng.random(), float(rng.random() < 0.3),
                   grn=U(0.1, 0.3) if rng.random() < 0.3 else -0.2)
        H = {"oat": (0.45, 0.9), "barley": (0.18, 0.42), "rye": (0.3, 0.6)}[kind]
        for i in range(rng.integers(4, 9)):
            a = rng.random() * 2 * math.pi
            _seed_stalk(mb, rng, np.array([0.025 * math.cos(a), 0.025 * math.sin(a), -0.01]), U(*H), kind)
    return mb


# variant table: (kind, lod, count)
_VARIANTS = ([("short", 0, 4), ("mid", 0, 4), ("tussock", 0, 3), ("oat", 0, 2), ("barley", 0, 2), ("rye", 0, 1),
              ("green", 0, 2), ("short", 1, 3), ("mid", 1, 3), ("tussock", 1, 2), ("short", 2, 2), ("mid", 2, 2),
              ("tussock", 2, 2)])


def _build_sources(coll, prefix, variants, maker, mats, mult=1.0, seed=5):
    rng = np.random.default_rng(SEED + seed)
    index, objs, tris = {}, [], []
    i = 0
    for kind, lod, cnt in variants:
        for c in range(cnt):
            mb = maker(kind, rng, mult, lod)
            name = "env:%s_%02d_%s%s_%d" % (prefix, i, kind, "_lod%d" % lod if lod else "", c)
            me = mb.build(name, mats, smooth=True)
            objs.append(_obj(name, me, coll))
            index.setdefault((kind, lod), []).append(i)
            tris.append(sum(len(f) - 2 for f in mb.f))
            i += 1
    return objs, index, np.array(tris)


def _choose_instances(X, Y, d, F, rng, index, preset):
    N = len(X)
    inst = np.zeros(N, np.int32)
    hzone = _hero_sdf(X, Y) < 0.5
    lod = np.where(hzone | (d <= preset["r_near"]), 0, np.where(d <= preset["lod_r"], 1, 2))
    t = F["gh"] + rng.normal(0, 0.15, N)
    cat = np.where(t < 0.36, 0, np.where(t < 0.68, 1, 2))
    for ci, kind in enumerate(GRASS_KINDS):
        for lv in (0, 1, 2):
            m = (cat == ci) & (lod == lv)
            ids = np.array(index[(kind, lv)])
            inst[m] = ids[rng.integers(0, len(ids), m.sum())]
    near = lod < 2
    seedp = (0.10 + 0.22 * F["near_fence"] + 0.25 * F["outside"]) * (cat > 0)
    sm = near & (rng.random(N) < seedp)
    pick = rng.random(N)
    for kind, lo, hi in (("oat", 0.0, 0.4), ("barley", 0.4, 0.75), ("rye", 0.75, 1.01)):
        m = sm & (pick >= lo) & (pick < hi)
        ids = np.array(index[(kind, 0)])
        inst[m] = ids[rng.integers(0, len(ids), m.sum())]
    gm = near & (cat == 0) & (rng.random(N) < 0.35 * F["green"] ** 2)
    inst[gm] = np.array(index[("green", 0)])[rng.integers(0, 2, gm.sum())]
    return inst


def _build_grass(coll, src_coll, terrain_axes, Zgrid, preset, mats, clear_zones=()):
    objs, index, tris = _build_sources(src_coll, "gsrc", _VARIANTS, _make_clump, mats, preset["blade_mult"])
    X, Y, d, F, rng = _scatter_points(preset, clear_zones)
    inst = _choose_instances(X, Y, d, F, rng, index, preset)
    Z = _interp_grid(terrain_axes, Zgrid, X, Y)
    N = len(X)
    s = rng.uniform(0.78, 1.25, N) * (1.0 - 0.1 * F["graze"]) * (1.0 - 0.35 * F["board"]) * \
        (1.0 + 0.7 * _sstep(preset["r_full"], preset["r_max"], d))
    scl = np.stack([s, s, s * rng.uniform(0.85, 1.15, N)], 1)
    rot = np.stack([rng.normal(0, 0.07, N), rng.normal(0, 0.07, N), rng.random(N) * 2 * math.pi], 1)
    tint = np.clip(0.5 + 0.55 * _nz(20).fbm(X / 5.0, Y / 5.0, 3), 0, 1)
    ob = _points_object("env:grass", coll, np.stack([X, Y, Z], 1), inst, rot, scl,
                        dict(tint=tint, igrn=F["green"]))
    ng = _instance_nodes("env:grass_scatter", src_coll, WIND["grass_amp"], WIND["grass_scale"])
    ob.modifiers.new("scatter", "NODES").node_group = ng
    stats = dict(instances=int(N), triangles=int(tris[inst].sum()), sources=len(objs))
    return ob, stats


def _points_object(name, coll, P, inst, rot, scl, floats):
    me = bpy.data.meshes.new(name)
    me.vertices.add(len(P))
    me.vertices.foreach_set("co", np.asarray(P, np.float32).ravel())
    me.attributes.new("inst", "INT", "POINT").data.foreach_set("value", np.asarray(inst, np.int32))
    me.attributes.new("rot", "FLOAT_VECTOR", "POINT").data.foreach_set("vector", np.asarray(rot, np.float32).ravel())
    me.attributes.new("scl", "FLOAT_VECTOR", "POINT").data.foreach_set("vector", np.asarray(scl, np.float32).ravel())
    for k, v in floats.items():
        me.attributes.new(k, "FLOAT", "POINT").data.foreach_set("value", np.asarray(v, np.float32))
    me.update()
    return _obj(name, me, coll)


def _instance_nodes(name, src_coll, wind_amp=0.0, wind_scale=0.2):
    """GN: instance the children of src_coll on the points (attributes inst / rot / scl), then sway them in a
    wind field (noise of world position drifting with the scene time, gusts) by rotating each instance about
    its own origin (the clump base / twig attachment)."""
    ng = bpy.data.node_groups.new(name, "GeometryNodeTree")
    ng.interface.new_socket("Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    ng.interface.new_socket("Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    N = ng.nodes
    L = ng.links
    gi = N.new("NodeGroupInput")
    go = N.new("NodeGroupOutput")
    ci = N.new("GeometryNodeCollectionInfo")
    ci.transform_space = "ORIGINAL"
    ci.inputs["Collection"].default_value = src_coll
    ci.inputs["Separate Children"].default_value = True
    ci.inputs["Reset Children"].default_value = True
    iop = N.new("GeometryNodeInstanceOnPoints")
    iop.inputs["Pick Instance"].default_value = True

    def named(nm, dt):
        n = N.new("GeometryNodeInputNamedAttribute")
        n.data_type = dt
        n.inputs["Name"].default_value = nm
        return n.outputs["Attribute"]

    e2r = N.new("FunctionNodeEulerToRotation")
    L.new(gi.outputs[0], iop.inputs["Points"])
    L.new(ci.outputs["Instances"], iop.inputs["Instance"])
    L.new(named("inst", "INT"), iop.inputs["Instance Index"])
    L.new(named("rot", "FLOAT_VECTOR"), e2r.inputs[0])
    L.new(e2r.outputs[0], iop.inputs["Rotation"])
    L.new(named("scl", "FLOAT_VECTOR"), iop.inputs["Scale"])
    last = iop.outputs["Instances"]
    if wind_amp > 0:
        st = N.new("GeometryNodeInputSceneTime")
        pos = N.new("GeometryNodeInputPosition")
        spd = WIND["speed"]
        drift = N.new("ShaderNodeCombineXYZ")
        m1 = N.new("ShaderNodeMath")
        m1.operation = "MULTIPLY"
        m1.inputs[1].default_value = spd * 0.8
        L.new(st.outputs["Seconds"], m1.inputs[0])
        m2 = N.new("ShaderNodeMath")
        m2.operation = "MULTIPLY"
        m2.inputs[1].default_value = spd * 0.6
        L.new(st.outputs["Seconds"], m2.inputs[0])
        L.new(m1.outputs[0], drift.inputs["X"])
        L.new(m2.outputs[0], drift.inputs["Y"])
        fine = N.new("ShaderNodeVectorMath")
        fine.operation = "MULTIPLY_ADD"
        L.new(pos.outputs["Position"], fine.inputs[0])
        fine.inputs[1].default_value = (wind_scale,) * 3
        sdr = N.new("ShaderNodeVectorMath")
        sdr.operation = "SCALE"
        L.new(drift.outputs[0], sdr.inputs[0])
        sdr.inputs["Scale"].default_value = wind_scale
        L.new(sdr.outputs[0], fine.inputs[2])
        nz = N.new("ShaderNodeTexNoise")
        nz.inputs["Scale"].default_value = 1.0
        nz.inputs["Detail"].default_value = 1.5
        L.new(fine.outputs[0], nz.inputs["Vector"])
        gust_v = N.new("ShaderNodeVectorMath")
        gust_v.operation = "MULTIPLY_ADD"
        L.new(pos.outputs["Position"], gust_v.inputs[0])
        gust_v.inputs[1].default_value = (WIND["gust_scale"],) * 3
        gsd = N.new("ShaderNodeVectorMath")
        gsd.operation = "SCALE"
        L.new(drift.outputs[0], gsd.inputs[0])
        gsd.inputs["Scale"].default_value = WIND["gust_scale"]
        L.new(gsd.outputs[0], gust_v.inputs[2])
        gn = N.new("ShaderNodeTexNoise")
        gn.inputs["Scale"].default_value = 1.0
        gn.inputs["Detail"].default_value = 0.0
        L.new(gust_v.outputs[0], gn.inputs["Vector"])
        sep = N.new("ShaderNodeSeparateXYZ")
        L.new(nz.outputs["Color"], sep.inputs[0])
        # tilt about world X (sway along the wind, +Y) and a little about Y; amplitude rides on the gusts
        amp = N.new("ShaderNodeMapRange")
        L.new(gn.outputs["Fac"], amp.inputs["Value"])
        amp.inputs["From Min"].default_value = 0.3
        amp.inputs["From Max"].default_value = 0.7
        amp.inputs["To Min"].default_value = wind_amp * 0.35
        amp.inputs["To Max"].default_value = wind_amp * 1.4
        comb = N.new("ShaderNodeCombineXYZ")
        for axis, w, off in (("X", 1.0, 0.42), ("Y", 0.45, 0.5)):
            sub = N.new("ShaderNodeMath")
            sub.operation = "SUBTRACT"
            L.new(sep.outputs[axis], sub.inputs[0])
            sub.inputs[1].default_value = off
            mul = N.new("ShaderNodeMath")
            mul.operation = "MULTIPLY"
            L.new(sub.outputs[0], mul.inputs[0])
            L.new(amp.outputs["Result"], mul.inputs[1])
            mul2 = N.new("ShaderNodeMath")
            mul2.operation = "MULTIPLY"
            L.new(mul.outputs[0], mul2.inputs[0])
            mul2.inputs[1].default_value = 4.0 * w
            L.new(mul2.outputs[0], comb.inputs[axis])
        e2 = N.new("FunctionNodeEulerToRotation")
        L.new(comb.outputs[0], e2.inputs[0])
        ri = N.new("GeometryNodeRotateInstances")
        L.new(last, ri.inputs["Instances"])
        L.new(e2.outputs[0], ri.inputs["Rotation"])
        ri.inputs["Local Space"].default_value = False
        last = ri.outputs["Instances"]
    L.new(last, go.inputs[0])
    return ng


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
    hx0, hx1, hy0, hy1 = HERO_ZONE
    if preset["hero"] > 1.0:
        hc = cell / math.sqrt(preset["hero"] - 1.0)
        gx = np.arange(hx0 - 4.0, hx1 + 4.0, hc)
        gy = np.arange(hy0 - 4.0, hy1 + 4.0, hc)
        HX, HY = np.meshgrid(gx, gy, indexing="ij")
        HX = HX.ravel() + rng.random(HX.size) * hc
        HY = HY.ravel() + rng.random(HY.size) * hc
        hm = 1.0 - _sstep(-1.5, 3.0, _hero_sdf(HX, HY) + 1.5 * _nz(15).fbm(HX / 2.0, HY / 2.0, 2))
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


# =============================================================================================================
# ground litter: gum leaves, bark strips, twigs, sticks, rocks, dried manure
def _leaf_mesh(mb, M, t0, L, W, curve, curl, arch, lcol, mat, segs=6):
    """Eucalyptus leaf (falcate lanceolate, sickle curved) in local frame: x along the leaf, y across, z normal;
    placed with rotation M (3x3) and translation t0. Three vertices per row give a midrib crease / curl."""
    u = np.linspace(0, 1, segs + 1)
    yc = curve * L * u ** 2
    w = W * np.sin(np.pi * np.clip(u, 0, 1) ** 0.75) ** 0.8
    w[0] = W * 0.08
    rows = []
    for k in range(segs + 1):
        z0 = arch * L * math.sin(math.pi * u[k])
        hw = 0.5 * w[k]
        rows.append([[u[k] * L, yc[k] - hw, z0 + curl * hw], [u[k] * L, yc[k], z0], [u[k] * L, yc[k] + hw,
                                                                                     z0 + curl * hw]])
    P = np.array(rows).reshape(-1, 3) @ np.asarray(M).T + t0
    b = mb.verts(P, lcol=lcol, h01=np.repeat(u, 3))
    for k in range(segs):
        for j in range(2):
            mb.face([b + k * 3 + j, b + k * 3 + j + 1, b + (k + 1) * 3 + j + 1, b + (k + 1) * 3 + j], mat)


def _rot_mat(yaw, pitch=0.0, roll=0.0):
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _blob(mb, rng, center, radii, nlon=10, nlat=6, rough=0.18, mat=0, k=50, **attrs):
    """Irregular low-poly ellipsoid (rocks, dung balls)."""
    nz = _nz(k)
    off = rng.random() * 100
    P = []
    for i in range(1, nlat):
        el = -math.pi / 2 + i * math.pi / nlat
        for j in range(nlon):
            a = j * 2 * math.pi / nlon
            dvec = np.array([math.cos(el) * math.cos(a), math.cos(el) * math.sin(a), math.sin(el)])
            f = 1 + rough * nz.fbm(dvec[0] * 1.7 + off, dvec[1] * 1.7 + dvec[2] * 1.3, 2)
            P.append(center + dvec * radii * f)
    P += [center - [0, 0, radii[2]], center + [0, 0, radii[2] * (1 + rough * 0.3)]]
    b = mb.verts(np.array(P), **attrs)
    nr = nlat - 1
    for i in range(nr - 1):
        for j in range(nlon):
            j2 = (j + 1) % nlon
            mb.face([b + i * nlon + j, b + i * nlon + j2, b + (i + 1) * nlon + j2, b + (i + 1) * nlon + j], mat)
    bot, top = b + nr * nlon, b + nr * nlon + 1
    for j in range(nlon):
        j2 = (j + 1) % nlon
        mb.face([bot, b + j2, b + j], mat)
        mb.face([top, b + (nr - 1) * nlon + j, b + (nr - 1) * nlon + j2], mat)


def _make_litter(kind, rng, mult=1.0, lod=0):
    mb = MB()
    U = rng.uniform
    if kind == "leaf":                       # a few fallen gum leaves lying together
        for i in range(rng.integers(1, 4)):
            M = _rot_mat(rng.random() * 6.283, U(-0.1, 0.1), U(-0.25, 0.25) + (math.pi if rng.random() < 0.4
                                                                               else 0.0))
            L = U(0.10, 0.19)
            _leaf_mesh(mb, M, np.array([U(-0.08, 0.08), U(-0.08, 0.08), U(0.003, 0.012)]) - M @ [L / 2, 0, 0],
                       L, L * U(0.12, 0.2), U(0.08, 0.22) * rng.choice([-1, 1]), U(0.1, 0.9), U(0.0, 0.06),
                       rng.random(), 0)
    elif kind == "bark":                     # shed bark ribbon, curled
        L = U(0.15, 0.6)
        Wd = U(0.02, 0.06)
        span = U(2.4, 4.6)
        r = Wd / span
        segs = max(4, int(L / 0.06))
        twist = U(-1.2, 1.2)
        rows = []
        for k in range(segs + 1):
            u = k / segs
            tw = twist * u
            for q in range(5):
                a = -span / 2 + q * span / 4
                yy, zz = r * math.sin(a), r * (1 - math.cos(a))
                c, s = math.cos(tw), math.sin(tw)
                rows.append([u * L - L / 2, yy * c - zz * s, yy * s + zz * c + 0.006 + 0.02 * math.sin(math.pi * u)
                             * U(0, 1)])
        b = mb.verts(np.array(rows), lcol=rng.random())
        for k in range(segs):
            for q in range(4):
                mb.face([b + k * 5 + q, b + k * 5 + q + 1, b + (k + 1) * 5 + q + 1, b + (k + 1) * 5 + q], 1)
    elif kind in ("twig", "stick"):
        L = U(0.15, 0.5) if kind == "twig" else U(0.45, 1.3)
        r0 = U(0.003, 0.006) if kind == "twig" else U(0.009, 0.02)
        z = r0 * 0.7
        p0 = np.array([-L / 2, 0, z])
        p1 = np.array([L / 2, U(-0.05, 0.05) * L, z + U(0, 0.02)])
        _cyl(mb, p0, p1, r0, r0 * 0.6, rng, 2, nring=5 if kind == "twig" else 7, bumps=0.08, k=55,
             tex=TEX_SIZE["bark_bluegum"] * 0.5)
        for i in range(rng.integers(1, 4)):
            f = U(0.2, 0.8)
            c = p0 + (p1 - p0) * f
            a = U(0.4, 1.1) * rng.choice([-1, 1])
            dvec = np.array([math.cos(a), math.sin(a), U(0.0, 0.3)])
            _cyl(mb, c, c + dvec * L * U(0.15, 0.4), r0 * 0.6, r0 * 0.3, rng, 2, nring=4, nseg=2,
                 tex=TEX_SIZE["bark_bluegum"] * 0.5)
    elif kind == "rock":
        r = U(0.02, 0.07) if rng.random() < 0.8 else U(0.08, 0.16)
        rad = np.array([r, r * U(0.65, 1.0), r * U(0.35, 0.65)])
        _blob(mb, rng, np.array([0, 0, rad[2] * 0.45]), rad, 10, 6, 0.22, 3, k=56, lcol=rng.random())
    elif kind == "grit":                     # tiny pebbles, clods and dung crumbs (under 1.5 cm high)
        for i in range(rng.integers(6, 14)):
            a = rng.random() * 6.283
            rr = 0.12 * math.sqrt(rng.random())
            rb = U(0.004, 0.012)
            rad = np.array([rb, rb * U(0.6, 1.0), rb * U(0.4, 0.7)])
            _blob(mb, rng, np.array([rr * math.cos(a), rr * math.sin(a), rad[2] * 0.3]), rad, 6, 4, 0.25,
                  3 if rng.random() < 0.7 else 4, k=59, lcol=rng.random())
        for i in range(rng.integers(2, 6)):  # flattened dead grass blades
            a = rng.random() * 6.283
            _strip(mb, rng, np.array([U(-0.1, 0.1), U(-0.1, 0.1), 0.0]), U(0.05, 0.16), U(0.002, 0.004), 3,
                   U(1.4, 1.55), 0.0, a, rng.random(), 1.0, mat=2)
    elif kind == "manure":                   # dried horse dung pile
        nb = int(rng.integers(8, 15))
        lc = rng.random()
        for i in range(nb):
            a = rng.random() * 6.283
            rr = 0.12 * math.sqrt(rng.random())
            rb = U(0.028, 0.038)
            zz = rb * 0.6 + (0.035 if rr < 0.05 and i % 3 == 0 else 0.0)
            _blob(mb, rng, np.array([rr * math.cos(a), rr * math.sin(a), zz]), np.array([rb, rb * 0.9, rb * 0.75]),
                  8, 5, 0.15, 4, k=57, lcol=float(np.clip(lc + rng.normal(0, 0.15), 0, 1)))
        for i in range(rng.integers(3, 8)):  # crumbled bits
            a = rng.random() * 6.283
            rr = U(0.1, 0.25)
            rb = U(0.008, 0.016)
            _blob(mb, rng, np.array([rr * math.cos(a), rr * math.sin(a), rb * 0.5]), np.array([rb, rb, rb * 0.6]),
                  6, 4, 0.2, 4, k=58, lcol=lc)
    return mb


_LITTER_VARIANTS = [("leaf", 0, 8), ("bark", 0, 5), ("twig", 0, 4), ("stick", 0, 4), ("rock", 0, 8),
                    ("manure", 0, 3), ("grit", 0, 4)]


def _build_litter(coll, src_coll, axes, Zgrid, preset, mats, clear_zones=()):
    objs, index, tris = _build_sources(src_coll, "lsrc", _LITTER_VARIANTS, _make_litter, mats, seed=9)
    rng = np.random.default_rng(SEED + 13)
    lm = preset["litter"]
    cx, cy = 4.0, 14.0
    R = 46.0
    # candidate grid at the max density of the densest kind, per-kind acceptance
    rates = dict(leaf=lambda F: 1.3 + 14.0 * F["lit"] ** 1.5, bark=lambda F: 0.04 + 0.8 * F["lit"] ** 2,
                 twig=lambda F: 0.15 + 1.5 * F["lit"] ** 2, stick=lambda F: 0.012 + 0.22 * F["lit"] ** 2,
                 rock=lambda F: 0.06 + 1.2 * F["soil"], grit=lambda F: 0.3 + 5.0 * F["soil"])
    Ps, I, S = [], [], []
    for kind, fn in rates.items():
        dmax = {"leaf": 15.3, "bark": 0.81, "twig": 1.63, "stick": 0.23, "rock": 1.26, "grit": 5.3}[kind] * lm
        cell = 1.0 / math.sqrt(dmax)
        g = np.arange(-R, R, cell)
        X, Y = np.meshgrid(g + cx, g + cy, indexing="ij")
        X = X.ravel() + rng.random(X.size) * cell
        Y = Y.ravel() + rng.random(Y.size) * cell
        F = _fields(X, Y)
        p = fn(F) * lm / dmax * (np.hypot(X - cx, Y - cy) < R) * (1 - F["board"])
        if kind in ("rock", "stick", "twig"):
            p = p * (1 - F["flat"])                       # keep the physics' flat ground clear
        for (zx, zy, zr) in clear_zones:
            p = p * (np.hypot(X - zx, Y - zy) > zr)
        k = rng.random(len(X)) < p
        ids = np.array(index[(kind, 0)])
        Ps.append(np.stack([X[k], Y[k]], 1))
        I.append(ids[rng.integers(0, len(ids), k.sum())])
        sc = rng.uniform(0.8, 1.25, k.sum())
        S.append(sc)
    # manure piles where the horses dung, plus a few scattered single droppings
    mp = np.array(MANURE, float)
    Ps.append(mp + rng.normal(0, 0.1, mp.shape))
    I.append(np.array(index[("manure", 0)])[rng.integers(0, 3, len(mp))])
    S.append(rng.uniform(0.85, 1.2, len(mp)))
    P = np.concatenate(Ps)
    inst = np.concatenate(I)
    s = np.concatenate(S)
    Z = _interp_grid(axes, Zgrid, P[:, 0], P[:, 1])
    N = len(P)
    rot = np.stack([rng.normal(0, 0.04, N), rng.normal(0, 0.04, N), rng.random(N) * 2 * math.pi], 1)
    ob = _points_object("env:litter", coll, np.column_stack([P, Z]), inst, rot, np.stack([s, s, s], 1), {})
    ob.modifiers.new("scatter", "NODES").node_group = _instance_nodes("env:litter_scatter", src_coll)
    return ob, dict(instances=int(N), triangles=int(tris[inst].sum()))


# =============================================================================================================
# eucalyptus gum trees
def _tube(mb, pts, radii, nring, mat, tex, attrs, uo=0.0):
    """Tube along a polyline (parallel-transported rings), UV u around, v along the length (bark grain)."""
    pts = np.asarray(pts, float)
    n = len(pts)
    T = np.gradient(pts, axis=0)
    T /= np.linalg.norm(T, axis=1, keepdims=True) + 1e-12
    ref = np.array([0.0, 0.0, 1.0]) if abs(T[0][2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    Nv = np.cross(T[0], ref)
    Nv /= np.linalg.norm(Nv)
    th = np.linspace(0, 2 * math.pi, nring, endpoint=False)
    P, V = [], []
    s_acc = 0.0
    for i in range(n):
        if i > 0:
            b = np.cross(T[i - 1], T[i])
            if np.linalg.norm(b) > 1e-8:
                b /= np.linalg.norm(b)
                ang = math.acos(np.clip(np.dot(T[i - 1], T[i]), -1, 1))
                Nv = Nv * math.cos(ang) + np.cross(b, Nv) * math.sin(ang) + b * np.dot(b, Nv) * (1 - math.cos(ang))
            s_acc += np.linalg.norm(pts[i] - pts[i - 1])
        Bv = np.cross(T[i], Nv)
        for a in th:
            P.append(pts[i] + radii[i] * (math.cos(a) * Nv + math.sin(a) * Bv))
        V.append(s_acc)
    base = mb.verts(np.array(P), **{k: (np.repeat(v, nring) if np.ndim(v) else v) for k, v in attrs.items()})
    circ = 2 * math.pi * float(np.mean(radii))
    for i in range(n - 1):
        for j in range(nring):
            j2 = (j + 1) % nring
            u0, u1 = j / nring * circ / tex + uo, (j + 1) / nring * circ / tex + uo
            v0, v1 = V[i] / tex, V[i + 1] / tex
            mb.face([base + i * nring + j, base + i * nring + j2, base + (i + 1) * nring + j2,
                     base + (i + 1) * nring + j], mat, [(u0, v0), (u1, v0), (u1, v1), (u0, v1)])


def _grow(rng, p0, d0, L, nseg, wobble, up_w, droop_w):
    pts = [np.asarray(p0, float)]
    d = np.asarray(d0, float) / np.linalg.norm(d0)
    for i in range(nseg):
        t = (i + 0.5) / nseg
        d = d + rng.normal(0, wobble, 3) + np.array([0, 0, up_w - droop_w * t])
        d /= np.linalg.norm(d)
        pts.append(pts[-1] + d * L / nseg)
    return np.array(pts)


def _perp_rotate(d, angle, az):
    d = d / np.linalg.norm(d)
    a = np.cross(d, [0, 0, 1.0])
    if np.linalg.norm(a) < 1e-6:
        a = np.array([1.0, 0, 0])
    a /= np.linalg.norm(a)
    b = np.cross(d, a)
    perp = a * math.cos(az) + b * math.sin(az)
    return d * math.cos(angle) + perp * math.sin(angle)


def _gum_tree(mb, rng, base, H, leaf_pts):
    """One eucalypt: sinuous pale trunk forking into a few heavy limbs, branches, drooping twigs; leaf cluster
    attachment points (position, scale) appended to leaf_pts; a few hanging bark ribbons at the forks."""
    U = rng.uniform
    pr = rng.random()
    tex = TEX_SIZE["bark_bluegum"]
    rt = 0.019 * H * U(0.85, 1.2)
    fork = H * U(0.2, 0.36)
    lean = np.array([U(-0.12, 0.12), U(-0.12, 0.12), 1.0])
    trunk = _grow(rng, base + np.array([0, 0, -0.3]), lean, fork + 0.3, 9, 0.05, 0.12, 0.0)
    rr = np.linspace(rt, rt * 0.78, len(trunk))
    rr[:2] *= (1.45, 1.15)
    stock_h = U(0.8, 2.6) if rng.random() < 0.6 else 0.0
    stock = (trunk[:, 2] - base[2] < stock_h).astype(float)
    _tube(mb, trunk, rr, 12, 0, tex, dict(prand=pr, stock=stock))
    nl = int(rng.integers(2, 5))
    az0 = rng.random() * 6.283
    for i in range(nl):
        az = az0 + i * 6.283 / nl + U(-0.4, 0.4)
        ang = U(0.45, 0.95)
        d = np.array([math.sin(ang) * math.cos(az), math.sin(ang) * math.sin(az), math.cos(ang)])
        _gum_branch(mb, rng, trunk[-1], d, (H - fork) * U(0.8, 1.1), rt * 0.78 * U(0.55, 0.78), 1, leaf_pts, pr)
    for i in range(rng.integers(2, 6)):                 # hanging bark ribbons near the forks
        a = rng.random() * 6.283
        top = trunk[-1] + np.array([math.cos(a) * rt * 0.9, math.sin(a) * rt * 0.9, U(-1.5, 0.2)])
        Lr = U(0.5, 1.6)
        rows = []
        for k in range(7):
            u = k / 6
            sway = 0.08 * math.sin(u * 3 + a)
            for q in range(3):
                off = (q - 1) * U(0.02, 0.035)
                rows.append(top + np.array([math.cos(a) * (0.02 + sway) - math.sin(a) * off,
                                            math.sin(a) * (0.02 + sway) + math.cos(a) * off, -u * Lr]))
        b = mb.verts(np.array(rows), lcol=rng.random())
        for k in range(6):
            for q in range(2):
                mb.face([b + k * 3 + q, b + k * 3 + q + 1, b + (k + 1) * 3 + q + 1, b + (k + 1) * 3 + q], 1)


def _gum_branch(mb, rng, p0, d0, L, r0, depth, leaf_pts, pr):
    U = rng.uniform
    nseg = max(3, int(L / (0.7 if depth < 3 else 0.3)))
    pts = _grow(rng, p0, d0, L, nseg, (0.06, 0.1, 0.14, 0.18, 0.2)[depth], (0.1, 0.08, 0.03, -0.01, -0.03)[depth],
                (0.0, 0.05, 0.25, 0.5, 0.7)[depth])
    r1 = max(r0 * 0.35, 0.005)
    radii = np.linspace(r0, r1, len(pts))
    _tube(mb, pts, radii, (12, 8, 6, 4, 3)[depth], 0, TEX_SIZE["bark_bluegum"], dict(prand=pr, stock=0.0))
    seglen = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0], np.cumsum(seglen)])

    def at(t):
        s = t * cum[-1]
        i = min(int(np.searchsorted(cum, s)) - 1, len(pts) - 2)
        i = max(i, 0)
        f = (s - cum[i]) / max(seglen[i], 1e-9)
        return pts[i] + (pts[i + 1] - pts[i]) * f, (pts[i + 1] - pts[i]) / max(seglen[i], 1e-9), \
            radii[i] + (radii[i + 1] - radii[i]) * f

    if depth >= 4 or r0 < 0.008:
        t = U(0.15, 0.3)
        while t <= 1.0:
            p, d, _r = at(t)
            leaf_pts.append((p, U(0.8, 1.25)))
            t += U(0.14, 0.24)
        return
    if depth == 3:                                    # leafy outer half of the small branches too
        t = 0.55
        while t <= 1.0:
            p, d, _r = at(t)
            leaf_pts.append((p, U(0.9, 1.3)))
            t += U(0.25, 0.4)
    nch = int(rng.integers(*((4, 8), (4, 7), (3, 6))[depth - 1]))
    for k in range(nch):
        t = U(0.25, 0.98) if depth > 1 else U(0.35, 0.98)
        p, d, r = at(t)
        ang = U(0.55, 1.1)
        dch = _perp_rotate(d, ang, rng.random() * 6.283)
        if depth == 1:
            dch = dch + np.array([0, 0, 0.25])
        _gum_branch(mb, rng, p, dch, L * U(0.38, 0.62), max(r * U(0.5, 0.72), 0.005), depth + 1, leaf_pts, pr)
    p, d, _r = at(1.0)
    leaf_pts.append((p, U(0.9, 1.3)))


def _leaf_flat(mb, M, t0, L, W, curve, lcol, segs=3):
    """Cheap canopy leaf: flat sickle blade, two vertices per row, pointed tip (5 triangles)."""
    u = np.linspace(0, 1, segs + 1)
    yc = curve * L * u ** 2
    w = W * np.sin(np.pi * np.clip(u, 0.06, 1) ** 0.75) ** 0.8
    P = []
    for k in range(segs):
        P += [[u[k] * L, yc[k] - 0.5 * w[k], 0.0], [u[k] * L, yc[k] + 0.5 * w[k], 0.0]]
    P.append([L, yc[-1], 0.0])
    P = np.array(P) @ np.asarray(M).T + t0
    b = mb.verts(P, lcol=lcol)
    for k in range(segs - 1):
        mb.face([b + 2 * k, b + 2 * k + 1, b + 2 * k + 3, b + 2 * k + 2], 0)
    mb.face([b + 2 * (segs - 1), b + 2 * (segs - 1) + 1, b + 2 * segs], 0)


def _make_leaf_cluster(kind, rng, mult=1.0, lod=0):
    """Clump of pendulous adult gum leaves (blades hang edge-on, sickle curved)."""
    mb = MB()
    U = rng.uniform
    n = int(rng.integers(34, 52))
    for i in range(n):
        p = np.array([rng.normal(0, 0.2), rng.normal(0, 0.2), rng.normal(0, 0.13) - 0.06])
        theta = U(0.15, 1.2)                       # from straight down
        a = rng.random() * 6.283
        x_ax = np.array([math.sin(theta) * math.cos(a), math.sin(theta) * math.sin(a), -math.cos(theta)])
        y_ax = np.cross([0, 0, 1.0], x_ax)
        y_ax /= np.linalg.norm(y_ax) + 1e-9
        roll = U(-0.5, 0.5)
        z_ax = np.cross(x_ax, y_ax)
        y2 = y_ax * math.cos(roll) + z_ax * math.sin(roll)
        z2 = np.cross(x_ax, y2)
        M = np.stack([x_ax, y2, z2], 1)
        L = U(0.09, 0.17)
        lc = rng.random() ** 1.6 if rng.random() > 0.08 else U(0.85, 1.0)
        _leaf_flat(mb, M, p, L, L * U(0.13, 0.2), U(0.08, 0.25) * rng.choice([-1, 1]), lc)
    return mb


def _build_trees(coll, src_coll, axes, Zgrid, mats):
    trees, leaf_pts = [], []
    for (x, y, H, sd) in GUM_TREES:
        rng = np.random.default_rng(SEED * 7 + sd)
        mb = MB()
        z = float(_interp_grid(axes, Zgrid, np.array([x]), np.array([y]))[0])
        _gum_tree(mb, rng, np.array([x, y, z]), H, leaf_pts)
        trees.append(_obj("env:gum_%02d" % sd, mb.build("env:gum_%02d" % sd, (mats["gum_bark"], mats["bark_strip"]),
                                                          smooth=True), coll))
    objs, index, tris = _build_sources(src_coll, "leafsrc", [("cluster", 0, 6)], _make_leaf_cluster,
                                       (mats["gum_leaf"],), seed=21)
    rng = np.random.default_rng(SEED + 23)
    P = np.array([p for p, s in leaf_pts])
    s = np.array([s for p, s in leaf_pts])
    N = len(P)
    inst = np.array(index[("cluster", 0)])[rng.integers(0, 6, N)]
    rot = np.stack([rng.normal(0, 0.15, N), rng.normal(0, 0.15, N), rng.random(N) * 6.283], 1)
    ob = _points_object("env:gum_leaves", coll, P, inst, rot, np.stack([s, s, s], 1), {})
    ob.modifiers.new("scatter", "NODES").node_group = _instance_nodes("env:gum_leaf_scatter", src_coll,
                                                                     WIND["leaf_amp"], WIND["leaf_scale"])
    return trees + [ob], dict(trees=len(trees), clusters=int(N), leaf_tris=int(tris[inst].sum()))


# =============================================================================================================
# fencing, gate, trough, fallen limbs, shed, bales
def _post_axis(rng, lean_deg):
    a = rng.random() * 2 * math.pi
    t = math.radians(lean_deg)
    return np.array([math.sin(t) * math.cos(a), math.sin(t) * math.sin(a), math.cos(t)])


def _star_picket(mb, rng, xy, z0, axis, h=1.2):
    """Steel star picket (Y section, black bitumen paint) with a white safety cap."""
    yaw = rng.random() * 2 * math.pi
    prof = []
    for k in range(3):
        a = yaw + k * 2 * math.pi / 3
        da = np.array([math.cos(a), math.sin(a)])
        nv = np.array([-da[1], da[0]]) * 0.0016
        prof += [nv * 1.0 + da * 0.004, nv + da * 0.030, -nv + da * 0.030, -nv + da * 0.004]
    prof = np.array(prof)
    bot = np.array([xy[0], xy[1], z0 - 0.45])
    top = np.array([xy[0], xy[1], z0]) + axis * h
    ring0 = np.column_stack([prof, np.zeros(len(prof))]) + bot
    ring1 = np.column_stack([prof, np.zeros(len(prof))]) + top
    b = mb.verts(np.vstack([ring0, ring1]))
    n = len(prof)
    for j in range(n):
        j2 = (j + 1) % n
        mb.face([b + j, b + j2, b + n + j2, b + n + j], 3)
    mb.face([b + n + j for j in range(n)], 3)
    _cyl(mb, top - axis * 0.04, top + axis * 0.035, 0.026, 0.022, rng, 4, nring=10, nseg=1)
    return top


def _wire(mb, pts, r, mat, nring=4):
    pts = np.asarray(pts, float)
    _tube(mb, pts, np.full(len(pts), r), nring, mat, 1.0, {})


def _tape(mb, pts, w, mat, rng):
    """Electric sighter tape: vertical ribbon through the points with a slight twist."""
    rows = []
    for i, p in enumerate(pts):
        tw = 0.25 * math.sin(i * 0.7 + rng.random())
        rows.append([p + np.array([0, 0, -w / 2]) + np.array([tw * 0.01, 0, 0]),
                     p + np.array([0, 0, w / 2]) - np.array([tw * 0.01, 0, 0])])
    b = mb.verts(np.array(rows).reshape(-1, 3))
    for i in range(len(pts) - 1):
        mb.face([b + 2 * i, b + 2 * i + 2, b + 2 * i + 3, b + 2 * i + 1], mat)


def _wire_fence(mb, rng, p0, p1, gate=None):
    """Timber posts (round, weathered) every POST_SPACING, star pickets every PICKET_SPACING, strainer posts with
    stays at the ends and the gate, plain wires and a white sighter tape on top. gate=(s0, s1) along the line."""
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    dirv = (p1 - p0) / L
    D3 = np.append(dirv, 0.0)
    T = TEX_SIZE["rough_wood"]
    supports = []                                      # (s, kind, ground point, axis)
    s = 0.0
    while s <= L + 1e-6:
        kind = "post" if (abs(s) < 1e-6 or abs(s - L) < 1e-6 or (s % POST_SPACING) < 1e-6) else "picket"
        supports.append([s, kind])
        s += PICKET_SPACING
    if gate:
        supports = [sp for sp in supports if sp[0] < gate[0] - 0.5 or sp[0] > gate[1] + 0.5]
        supports += [[gate[0], "strainer"], [gate[1], "strainer"]]
        supports.sort(key=lambda v: v[0])
    supports[0][1] = supports[-1][1] = "strainer"
    out = []
    for s, kind in supports:
        xy = p0 + dirv * (s + rng.normal(0, 0.04))
        z0 = terrain_height(xy[0], xy[1])
        if kind == "picket":
            axis = _post_axis(rng, rng.uniform(0.5, 4.0))
            _star_picket(mb, rng, xy, z0, axis)
        else:
            big = kind == "strainer"
            axis = _post_axis(rng, rng.uniform(0.2, 1.5) if big else rng.uniform(0.3, 3.0))
            h = 1.38 if big else 1.3
            r = rng.uniform(0.09, 0.11) if big else rng.uniform(0.055, 0.075)
            gz = z0
            _cyl(mb, np.array([xy[0], xy[1], z0 - 0.4]), np.array([xy[0], xy[1], z0]) + axis * h, r * 1.05, r * 0.95,
                 rng, 0, cap_mat=0, tex=T, nring=12, bumps=0.06, k=61, extra=dict(gz=gz))
            if big:                                    # diagonal stay to the next span
                sgn = 1.0 if s < L / 2 else -1.0
                if gate and abs(s - gate[0]) < 0.1:
                    sgn = -1.0
                if gate and abs(s - gate[1]) < 0.1:
                    sgn = 1.0
                foot = np.array([xy[0], xy[1], 0]) + D3 * sgn * 2.2
                foot[2] = terrain_height(foot[0], foot[1]) - 0.05
                _cyl(mb, np.array([xy[0], xy[1], z0]) + axis * 0.95, foot, 0.05, 0.05, rng, 0, cap_mat=0, tex=T,
                     nring=8, bumps=0.05, k=62, extra=dict(gz=gz))
        out.append((s, kind, np.array([xy[0], xy[1], z0]), axis))
    # wires and tape between consecutive supports, interrupted at the gate
    spans = []
    cur = []
    for sp in out:
        cur.append(sp)
        if gate and sp[1] == "strainer" and abs(sp[0] - gate[0]) < 0.1:
            spans.append(cur)
            cur = []
        elif gate and sp[1] == "strainer" and abs(sp[0] - gate[1]) < 0.1:
            cur = [sp]
    spans.append(cur)
    for span in spans:
        if len(span) < 2:
            continue
        for h in WIRE_HEIGHTS:
            pts = [g + ax * (h + rng.normal(0, 0.004)) for (_s, _k, g, ax) in span]
            _wire(mb, pts, 0.00125, 2)
        tp = []
        posts = [sp for sp in span if sp[1] != "picket"]
        for a, b in zip(posts[:-1], posts[1:]):
            A = a[2] + a[3] * TAPE_HEIGHT
            B = b[2] + b[3] * TAPE_HEIGHT
            n = max(2, int(np.linalg.norm(B - A) / 0.6))
            for i in range(n):
                u = i / n
                tp.append(A + (B - A) * u - np.array([0, 0, 0.05 * math.sin(math.pi * u)]))
        tp.append(posts[-1][2] + posts[-1][3] * TAPE_HEIGHT)
        _tape(mb, tp, 0.04, 5, rng)
    return out


def _tube_gate(mb, rng, a_xy, b_xy):
    """Galvanised tube farm gate (closed) between the strainer posts."""
    a = np.asarray(a_xy, float)
    b = np.asarray(b_xy, float)
    dv = (b - a) / np.linalg.norm(b - a)
    W = float(np.linalg.norm(b - a)) - 0.26
    z0 = terrain_height(*(0.5 * (a + b)))
    o = np.append(a + dv * 0.13, z0 + 0.1)
    D = np.append(dv, 0.0)
    Z = np.array([0, 0, 1.0])
    H = 1.15
    tubes = [(o, o + Z * H, 0.021), (o + D * W, o + D * W + Z * H, 0.021), (o + Z * H, o + D * W + Z * H, 0.021),
             (o, o + D * W, 0.017)]
    for k in range(1, 4):
        tubes.append((o + Z * (H * k / 4), o + D * W + Z * (H * k / 4), 0.0135))
    tubes.append((o + Z * 0.05, o + D * (W * 0.5) + Z * (H - 0.03), 0.0135))
    tubes.append((o + D * (W * 0.5) + Z * (H - 0.03), o + D * W + Z * 0.05, 0.0135))
    for p0, p1, r in tubes:
        _cyl(mb, p0, p1, r, r, rng, 1, nring=8, nseg=1)


def _deadwood_limb(mb, rng, cfg):
    """Fallen gum limb: grey weathered wood with a few side branches, resting on the ground."""
    lx, ly = cfg["xy"]
    a = math.radians(cfg["yaw"])
    dv = np.array([math.cos(a), math.sin(a)])
    L, R = cfg["length"], 0.5 * cfg["dia"]
    s = np.linspace(-L / 2, L / 2, 9)
    gz = terrain_height(lx + dv[0] * s, ly + dv[1] * s)
    slope, icpt = np.polyfit(s, gz, 1)
    p0 = np.array([lx - dv[0] * L / 2, ly - dv[1] * L / 2, icpt - slope * L / 2 + R * 0.8])
    p1 = np.array([lx + dv[0] * L / 2, ly + dv[1] * L / 2, icpt + slope * L / 2 + R * 0.5])
    Rf = _cyl(mb, p0, p1, R, R * 0.6, rng, 0, cap_mat=0, tex=TEX_SIZE["bark_bluegum"], nring=14, bumps=0.1, k=70,
              bow=rng.uniform(-0.03, 0.03) * L)
    for i in range(rng.integers(3, 6)):
        f = rng.uniform(0.25, 0.95)
        c = p0 + (p1 - p0) * f
        ang = rng.uniform(0.3, 1.2) * rng.choice([-1, 1])
        dirn = Rf @ np.array([math.cos(ang), math.sin(ang), rng.uniform(-0.05, 0.5)])
        _cyl(mb, c, c + dirn * rng.uniform(0.4, 1.4) * R / 0.17, R * 0.35, R * 0.12, rng, 0, cap_mat=0,
             tex=TEX_SIZE["bark_bluegum"], nring=7, bumps=0.08, k=71)


def _shed(mb, cfg):
    """Far corrugated-iron shed with a low gable roof."""
    x, y = cfg["xy"]
    Lx, Wy, Hw = cfg["size"]
    a = math.radians(cfg["yaw"])
    R = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
    z0 = terrain_height(x, y) - 0.2
    ridge = Hw + 0.5 * Wy * math.tan(math.radians(11))
    pts = np.array([[-Lx / 2, -Wy / 2, 0], [Lx / 2, -Wy / 2, 0], [Lx / 2, Wy / 2, 0], [-Lx / 2, Wy / 2, 0],
                    [-Lx / 2, -Wy / 2, Hw], [Lx / 2, -Wy / 2, Hw], [Lx / 2, Wy / 2, Hw], [-Lx / 2, Wy / 2, Hw],
                    [-Lx / 2, 0, ridge], [Lx / 2, 0, ridge]]) @ R.T + [x, y, z0]
    b = mb.verts(pts)
    for f in ([0, 1, 5, 4], [2, 3, 7, 6], [4, 5, 9, 8], [6, 7, 8, 9], [0, 4, 8, 7, 3], [1, 2, 6, 9, 5]):
        mb.face([b + i for i in f], 0)
    door = np.array([[-2.0, -Wy / 2 - 0.02, 0], [2.0, -Wy / 2 - 0.02, 0], [2.0, -Wy / 2 - 0.02, 3.4],
                     [-2.0, -Wy / 2 - 0.02, 3.4]]) @ R.T + [x, y, z0]
    b = mb.verts(door)
    mb.face([b, b + 1, b + 2, b + 3], 1)


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


# =============================================================================================================
# sulphur-crested cockatoos
_INTERP = {"CONSTANT": 0, "LINEAR": 1, "BEZIER": 2}


def _key_curve(idb, path, index, frames, values, interp="LINEAR"):
    """Bulk keys on one F-curve (Blender 5 layered actions)."""
    ad = idb.animation_data or idb.animation_data_create()
    if ad.action is None:
        ad.action = bpy.data.actions.new("act:" + idb.name)
    fc = ad.action.fcurve_ensure_for_datablock(idb, path, index=index)
    kp = fc.keyframe_points
    if len(kp):
        kp.clear()
    n = len(frames)
    kp.add(n)
    co = np.empty(2 * n, np.float32)
    co[0::2] = np.asarray(frames, np.float32)
    co[1::2] = np.asarray(values, np.float32)
    kp.foreach_set("co", co)
    kp.foreach_set("interpolation", np.full(n, _INTERP[interp], np.int32))
    fc.update()
    return fc


def _cockatoo_meshes(mats):
    """Body (pose frame: origin at the body centre, +X forward, +Z up), folded wings, legs (root frame, feet at
    the origin) and the two flight wings (shoulder at the origin, span along +Y / -Y)."""
    rng = np.random.default_rng(SEED + 31)
    body = MB()
    _blob(body, rng, np.array([0.0, 0, 0]), np.array([0.12, 0.068, 0.072]), 12, 8, 0.03, 0, k=81)
    _blob(body, rng, np.array([0.115, 0, 0.068]), np.array([0.052, 0.046, 0.05]), 10, 7, 0.02, 0, k=82)
    for side in (-1, 1):
        _blob(body, rng, np.array([0.142, side * 0.033, 0.082]), np.array([0.006, 0.006, 0.006]), 6, 4, 0.0, 3,
              k=83)
        _blob(body, rng, np.array([0.15, side * 0.03, 0.066]), np.array([0.008, 0.004, 0.006]), 6, 4, 0.0, 0,
              k=84)                                                    # pale eye-ring / cheek
    # beak: hooked dark cone
    tip = np.array([0.188, 0, 0.038])
    ring = [np.array([0.155, 0.018 * math.cos(a), 0.06 + 0.016 * math.sin(a)]) for a in np.linspace(0, 6.283, 8,
                                                                                                     endpoint=False)]
    b = body.verts(np.vstack([ring, tip]))
    for j in range(8):
        body.face([b + j, b + (j + 1) % 8, b + 8], 2)
    # crest: yellow feathers sweeping back and up from the crown
    for k in range(7):
        a = (k - 3) * 0.09
        p0 = np.array([0.12 - 0.012 * abs(k - 3), a * 0.12, 0.112])
        L = 0.055 + 0.03 * (1 - abs(k - 3) / 3)
        _ribbon(body, p0, p0 + np.array([-0.75 * L, a * 0.25, 0.55 * L]), 0.012, np.array([0, 1.0, 0.2]),
                rng.random(), 1)
    # tail fan
    b = body.verts([[-0.09, -0.02, -0.005], [-0.09, 0.02, -0.005], [-0.25, 0.045, -0.04], [-0.26, 0.0, -0.045],
                    [-0.25, -0.045, -0.04]])
    body.face([b, b + 1, b + 2, b + 3, b + 4], 0)
    fold = MB()
    for side in (-1, 1):
        _blob(fold, rng, np.array([-0.04, side * 0.058, 0.012]), np.array([0.14, 0.022, 0.05]), 10, 6, 0.03, 0,
              k=85)
    legs = MB()
    for side in (-1, 1):
        _cyl(legs, np.array([0.0, side * 0.025, 0.0]), np.array([0.01, side * 0.028, 0.12]), 0.006, 0.007, rng, 4,
             nring=5, nseg=1)
        for a in (-0.5, 0.0, 0.5, 3.1):
            _cyl(legs, np.array([0.0, side * 0.025, 0.004]),
                 np.array([0.03 * math.cos(a), side * 0.025 + 0.03 * math.sin(a), 0.002]), 0.003, 0.002, rng, 4,
                 nring=4, nseg=1)
    wings = []
    for side in (1, -1):
        w = MB()
        ns, nc = 7, 3
        P = []
        for i in range(ns + 1):
            s = i / ns
            le = 0.05 - 0.035 * s
            te = -0.14 + 0.03 * s + 0.07 * s ** 4
            for j in range(nc + 1):
                c = j / nc
                x = le + (te - le) * c
                P.append([x, side * (0.02 + 0.47 * s), 0.012 * math.sin(math.pi * c) * (1 - s)])
        b = w.verts(np.array(P))
        for i in range(ns):
            for j in range(nc):
                f = [b + i * (nc + 1) + j, b + i * (nc + 1) + j + 1, b + (i + 1) * (nc + 1) + j + 1,
                     b + (i + 1) * (nc + 1) + j]
                w.face(f if side > 0 else f[::-1], 0)
        # primary feather "fingers" at the tip
        for k in range(5):
            x0 = 0.02 - 0.03 * k
            p0 = np.array([x0, side * 0.47, 0.0])
            _ribbon(w, p0, p0 + np.array([-0.02 * k, side * 0.07, 0.0]), 0.025, np.array([1.0, 0, 0.0]), 0.5, 0)
        wings.append(w)
    feather = (mats["feather"], mats["crest"], mats["beak"], mats["eye"], mats["feet"])
    return (body.build("env:cockatoo_body", feather, smooth=True), fold.build("env:cockatoo_fold", feather,
                                                                                smooth=True),
            legs.build("env:cockatoo_legs", feather), wings[0].build("env:cockatoo_wing_l", feather, smooth=True),
            wings[1].build("env:cockatoo_wing_r", feather, smooth=True))


def _bird_rig(coll, meshes, name):
    body_me, fold_me, legs_me, wl_me, wr_me = meshes
    root = bpy.data.objects.new(name, None)
    root.empty_display_size = 0.2
    root.rotation_mode = "ZYX"
    coll.objects.link(root)
    pose = bpy.data.objects.new(name + "_pose", None)
    pose.parent = root
    pose.location = (0, 0, 0.17)
    coll.objects.link(pose)
    parts = {}
    for key, me, par, loc in (("body", body_me, pose, (0, 0, 0)), ("fold", fold_me, pose, (0, 0, 0)),
                              ("legs", legs_me, root, (0, 0, 0)), ("wl", wl_me, pose, (0.02, 0.045, 0.035)),
                              ("wr", wr_me, pose, (0.02, -0.045, 0.035))):
        ob = bpy.data.objects.new("%s_%s" % (name, key), me)
        ob.parent = par
        ob.location = loc
        coll.objects.link(ob)
        parts[key] = ob
    parts["root"] = root
    parts["pose"] = pose
    return parts


def _flap(t, freq, amp, bias=0.1):
    return amp * np.sin(2 * math.pi * freq * t) + bias


def _flight_attitude(P, fps):
    V = np.gradient(P, axis=0) * fps
    yaw = np.unwrap(np.arctan2(V[:, 1], V[:, 0]))
    sp = np.linalg.norm(V[:, :2], axis=1) + 1e-6
    pitch = -np.arctan2(V[:, 2], sp) * 0.6
    bank = np.clip(-np.gradient(yaw) * fps * 0.25, -0.6, 0.6)
    return yaw, pitch, bank


def _key_bird(parts, frames, P, yaw, pitch, bank, wing):
    root, pose = parts["root"], parts["pose"]
    for i in range(3):
        _key_curve(root, "location", i, frames, P[:, i])
    _key_curve(root, "rotation_euler", 2, frames, yaw)
    _key_curve(root, "rotation_euler", 0, frames, bank)
    _key_curve(pose, "rotation_euler", 1, frames, pitch)
    _key_curve(parts["wl"], "rotation_euler", 0, frames, wing)
    _key_curve(parts["wr"], "rotation_euler", 0, frames, -np.asarray(wing))


def _vis(ob, frames, visible):
    _key_curve(ob, "hide_render", 0, frames, [0.0 if v else 1.0 for v in visible], "CONSTANT")
    _key_curve(ob, "hide_viewport", 0, frames, [0.0 if v else 1.0 for v in visible], "CONSTANT")


def add_cockatoos(scene, fire_frame, frame_start=None, frame_end=None, n_ground=7, pair_frame=None,
                  ground_center=(-4.0, 8.2), seed=0):
    """Sulphur-crested cockatoos for the background (collection ENV, objects env:cockatoo_*).

    n_ground birds forage on the ground around ground_center (the bare shade camp under the paddock gum, about
    20 m from cam:A),
    pecking and shuffling; each takes off 0.2 to 0.8 s after fire_frame and flies away from the base, climbing
    over the trees. A pair crosses the sky from left to right starting at pair_frame (default 9 s after the fire).
    Wings flap 4 to 7 Hz. Returns the list of bird root empties."""
    fps = scene.render.fps / scene.render.fps_base
    f0 = scene.frame_start if frame_start is None else frame_start
    f1 = scene.frame_end if frame_end is None else frame_end
    env = _collection(scene, "ENV")
    for ob in [o for o in env.objects if o.name.startswith("env:cockatoo_")]:
        bpy.data.objects.remove(ob, do_unlink=True)
    mats = dict(feather=_mat_feather(), crest=_mat_plain("env:cockatoo_crest", (0.80, 0.55, 0.03), 0.5, 0.3, 0.2),
                beak=_mat_plain("env:cockatoo_beak", (0.035, 0.035, 0.04), 0.35, 0.5),
                eye=_mat_plain("env:cockatoo_eye", (0.01, 0.01, 0.01), 0.2, 0.6),
                feet=_mat_plain("env:cockatoo_feet", (0.12, 0.11, 0.11), 0.6, 0.3))
    meshes = _cockatoo_meshes(mats)
    rng = np.random.default_rng(SEED + 41 + seed)
    roots = []
    gx, gy = ground_center
    for i in range(n_ground):
        parts = _bird_rig(env, meshes, "env:cockatoo_%02d" % i)
        p0 = np.array([gx + rng.uniform(-3.8, 3.8), gy + rng.uniform(-1.8, 1.8), 0.0])
        p0[2] = terrain_height(p0[0], p0[1])
        yaw0 = rng.random() * 6.283
        f_to = int(fire_frame + rng.uniform(0.2, 0.8) * fps)
        # on the ground: pecking (pose pitch down), the odd shuffle and turn
        fr = np.arange(f0, f_to + 1, 3)
        t = fr / fps
        peck = np.clip(np.sin(t * rng.uniform(1.2, 2.2) + rng.random() * 10) * 3.0 - 1.8, 0, 1)
        g_pitch = -0.35 + 0.95 * peck
        sh = np.cumsum(rng.normal(0, 0.004, len(fr)))
        g_yaw = yaw0 + np.cumsum(rng.normal(0, 0.02, len(fr)))
        gP = np.column_stack([p0[0] + sh * math.cos(yaw0), p0[1] + sh * math.sin(yaw0), np.full(len(fr), p0[2])])
        # take-off (0.2 to 0.8 s after the fire) and flight away from the base, climbing over the trees
        ff = np.arange(fr[-1] + 1, min(fr[-1] + 1 + int(7.0 * fps), f1 + 1))
        tt = (ff - fr[-1]) / fps
        away = gP[-1, :2] / (np.linalg.norm(gP[-1, :2]) + 1e-6) + rng.normal(0, 0.35, 2)
        away /= np.linalg.norm(away)
        dist = np.cumsum(1.5 + 8.0 * (1 - np.exp(-tt / 1.0))) / fps
        curve = rng.uniform(-0.15, 0.15) * tt
        dirs = np.stack([away[0] * np.cos(curve) - away[1] * np.sin(curve),
                         away[0] * np.sin(curve) + away[1] * np.cos(curve)], 1)
        fP = np.column_stack([gP[-1, :2] + dirs * dist[:, None],
                              p0[2] + 0.6 * (1 - np.exp(-tt / 0.25)) + rng.uniform(1.8, 3.0) * tt])
        f_yaw, f_pitch, f_bank = _flight_attitude(np.vstack([gP[-1:], fP]), fps)
        f_yaw = f_yaw[1:] + 2 * math.pi * np.round((g_yaw[-1] - f_yaw[1]) / (2 * math.pi))
        flap = _flap(tt, 6.5 - 1.5 * np.clip(tt / 2, 0, 1), 1.0 - 0.3 * np.clip(tt / 2, 0, 1))
        flap = np.where((np.sin(tt * 0.9 + rng.random() * 6) > 0.85) & (tt > 2), 0.12, flap)
        frames = np.concatenate([fr, ff])
        _key_bird(parts, frames, np.vstack([gP, fP]), np.concatenate([g_yaw, f_yaw]),
                  np.concatenate([g_pitch, f_pitch[1:]]), np.concatenate([np.zeros(len(fr)), f_bank[1:]]),
                  np.concatenate([np.full(len(fr), -0.3), flap]))
        f_to = int(fr[-1])
        on_ground = [True, False]
        kf = [f0, f_to + 2]
        _vis(parts["fold"], kf, on_ground)
        _vis(parts["legs"], kf, on_ground)
        _vis(parts["wl"], kf, [False, True])
        _vis(parts["wr"], kf, [False, True])
        roots.append(parts["root"])
    # a pair crossing the sky, left to right, beyond the trees
    pf = int(fire_frame + 9 * fps) if pair_frame is None else int(pair_frame)
    for j in range(2):
        parts = _bird_rig(env, meshes, "env:cockatoo_pair_%d" % j)
        n = int(16 * fps)
        ff = np.arange(pf, pf + n)
        tt = (ff - pf) / fps
        y = 48.0 + 2.5 * j + 1.5 * np.sin(tt * 0.4 + j)
        x = -75.0 + 9.5 * tt - 3.0 * j
        z = 21.0 + 1.5 * j + 1.2 * np.sin(tt * 0.6 + j * 2)
        P = np.column_stack([x, y, z])
        yaw, pitch, bank = _flight_attitude(P, fps)
        flap = np.where(np.sin(tt * 0.8 + j * 1.3) > 0.55, 0.12, _flap(tt + 0.13 * j, 4.5, 0.8))
        _key_bird(parts, ff, P, yaw, pitch, bank, flap)
        _vis(parts["fold"], [f0], [False])
        _vis(parts["legs"], [f0], [False])
        for key in ("body", "wl", "wr"):
            _vis(parts[key], [min(f0, pf - 1), pf, pf + n], [False, True, False])
        roots.append(parts["root"])
    return roots


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
    F["cov"] = F["dens"] * (1 - F["outside"] * 0.0)
    for k in ("soil", "track", "trod", "cov"):
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
def _collection(scene, name, parent=None):
    c = bpy.data.collections.get(name)
    if c is None:
        c = bpy.data.collections.new(name)
        (parent or scene.collection).children.link(c)
    return c


_SRC_COLLS = ("ENV_SRC_GRASS", "ENV_SRC_LITTER", "ENV_SRC_LEAF")


def _clear_env():
    for name in _SRC_COLLS + ("ENV_SRC", "ENV"):
        c = bpy.data.collections.get(name)
        if c:
            for ob in list(c.objects):
                bpy.data.objects.remove(ob, do_unlink=True)
    for ng in [g for g in bpy.data.node_groups if g.name.startswith("env:")]:
        bpy.data.node_groups.remove(ng)


def _exclude(scene, coll):
    def walk(lc):
        for sub in lc.children:
            if sub.collection == coll:
                sub.exclude = True
                return True
            if walk(sub):
                return True
        return False
    for vl in scene.view_layers:
        walk(vl.layer_collection)


def build_env(scene, quality=None, clear_zones=(), board_clear=True, board_half=None):
    """Build the Australian paddock into collection ENV of `scene`.

    quality: key of GRASS_PRESETS (default GRASS_QUALITY). clear_zones: [(x, y, r), ...] discs kept free of
    grass and litter (e.g. where the dart comes to rest). board_clear: no grass inside the base board footprint
    (board_half = (hx, hy) half sizes in m, default BOARD_HALF), so blades do not poke through the board; the
    edge is soft and ragged and sits just inside the footprint. Returns a dict with the main objects and stats.
    Cockatoos are separate: add_cockatoos(scene, fire_frame)."""
    global BOARD_CLEAR, BOARD_HALF
    BOARD_CLEAR = bool(board_clear)
    if board_half:
        BOARD_HALF = tuple(board_half)
    t0 = time.time()
    preset = GRASS_PRESETS[quality or GRASS_QUALITY]
    _clear_env()
    env = _collection(scene, "ENV")
    src = _collection(scene, "ENV_SRC", env)
    srcs = {k: _collection(scene, k, src) for k in _SRC_COLLS}
    _exclude(scene, src)
    world, hdri_img = _build_world(scene)
    sun = _build_sun(env)
    mats = dict(ground=_mat_ground(hdri_img), grass=_mat_grass(), gum_bark=_mat_gum_bark(),
                gum_leaf=_mat_gum_canopy(), leaf_litter=_mat_leaf_litter(), bark_strip=_mat_bark_strip(),
                deadwood=_mat_deadwood(), rock=_mat_rock(), manure=_mat_manure(), timber=_mat_timber(),
                galv=_mat_metal("env:galvanised", (0.55, 0.56, 0.57), 0.35, 0.9, spangle=True),
                wire=_mat_metal("env:wire", (0.6, 0.6, 0.6), 0.3, 1.0),
                picket=_mat_plain("env:picket_black", (0.025, 0.025, 0.025), 0.5, 0.5),
                cap=_mat_plain("env:picket_cap", (0.75, 0.75, 0.72), 0.45, 0.4),
                tape=_mat_plain("env:sighter_tape", (0.72, 0.72, 0.68), 0.6, 0.3, trans=0.25),
                water=_mat_water(), zinc=_mat_zincalume(), hay=_mat_straw(),
                shed_door=_mat_plain("env:shed_door", (0.08, 0.075, 0.07), 0.6, 0.3))
    mats["seedhead"] = _mat_leafy("env:seedhead", [(0.0, (0.48, 0.40, 0.22)), (0.5, (0.42, 0.33, 0.17)),
                                                   (0.8, (0.34, 0.27, 0.18)), (1.0, (0.55, 0.47, 0.30))],
                                  rough=0.6, trans=0.3, spec=0.3)
    t1 = time.time()
    terrain, axes, Zgrid, ntris = _build_terrain(env, mats["ground"])
    t2 = time.time()
    grass, gstats = _build_grass(env, srcs["ENV_SRC_GRASS"], axes, Zgrid, preset, (mats["grass"], mats["seedhead"]),
                                 clear_zones)
    litter, lstats = _build_litter(env, srcs["ENV_SRC_LITTER"], axes, Zgrid, preset,
                                   (mats["leaf_litter"], mats["bark_strip"], mats["deadwood"], mats["rock"],
                                    mats["manure"]), clear_zones)
    t3 = time.time()
    trees, tstats = _build_trees(env, srcs["ENV_SRC_LEAF"], axes, Zgrid, mats)
    t4 = time.time()
    props = []
    rng = np.random.default_rng(SEED + 3)
    fmats = (mats["timber"], mats["galv"], mats["wire"], mats["picket"], mats["cap"], mats["tape"])
    mb = MB()
    _wire_fence(mb, rng, (FENCE_BACK_X[0], FENCE_BACK_Y), (FENCE_BACK_X[1], FENCE_BACK_Y),
                gate=(GATE_X[0] - FENCE_BACK_X[0], GATE_X[1] - FENCE_BACK_X[0]))
    _tube_gate(mb, rng, (GATE_X[0], FENCE_BACK_Y), (GATE_X[1], FENCE_BACK_Y))
    props.append(_obj("env:fence_back", mb.build("env:fence_back", fmats, smooth=True), env))
    mb = MB()
    _wire_fence(mb, rng, (FENCE_SIDE_X, FENCE_SIDE_Y[1]), (FENCE_SIDE_X, FENCE_SIDE_Y[0]))
    props.append(_obj("env:fence_side", mb.build("env:fence_side", fmats, smooth=True), env))
    mb = MB()
    _trough(mb, rng, TROUGH_XY)
    props.append(_obj("env:water_trough", mb.build("env:water_trough", (mats["galv"], mats["water"]), smooth=True),
                      env))
    for i, cfg in enumerate(FALLEN_LIMBS):
        mb = MB()
        _deadwood_limb(mb, rng, cfg)
        props.append(_obj("env:fallen_limb_%d" % (i + 1), mb.build("env:fallen_limb_%d" % (i + 1),
                                                                    (mats["deadwood"],), smooth=True), env))
    mb = MB()
    _shed(mb, SHED)
    props.append(_obj("env:shed", mb.build("env:shed", (mats["zinc"], mats["shed_door"])), env))
    for i, (bx, by, yaw) in enumerate(BALES):
        mb = MB()
        _bale(mb, rng, (bx, by), yaw)
        props.append(_obj("env:round_bale_%d" % (i + 1), mb.build("env:round_bale_%d" % (i + 1),
                                                                   (mats["hay"], mats["hay"]), smooth=True), env))
    t5 = time.time()
    flat = check_flat_zones()
    stats = dict(grass=gstats, litter=lstats, trees=tstats, terrain_tris=ntris, flat_landing_max_m=flat[0],
                 flat_base_max_m=flat[1], build_s=round(t5 - t0, 1), t_mats=round(t1 - t0, 1),
                 t_terrain=round(t2 - t1, 1), t_grass_litter=round(t3 - t2, 1), t_trees=round(t4 - t3, 1),
                 t_props=round(t5 - t4, 1), hdri=_hdri_path() is not None, quality=quality or GRASS_QUALITY)
    return dict(terrain=terrain, grass=grass, litter=litter, trees=trees, sun=sun, world=world, props=props,
                stats=stats)


def set_render(scene, engine="EEVEE", preset="preview", motion_blur=False, samples=None):
    """Render settings. engine "EEVEE" (full animation) or "CYCLES" (hero stills); preset "preview" (960x540,
    low samples), "final" (1920x1080) or "uhd" (3840x2160). motion_blur: 180 deg shutter (0.5 frame)."""
    r = scene.render
    final = preset in ("final", "uhd")
    r.resolution_x, r.resolution_y = {"uhd": (3840, 2160), "final": (1920, 1080)}.get(preset, (960, 540))
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


PREVIEW_CAMS = {          # about where build.py's two locked-off tripod cameras sit (SPEC v2)
    "": dict(loc=(2.4, -10.6, 1.55), target=(2.6, 1.0, 0.35), lens=40.0),      # cam:A wide, 10.6 m out
    "_b": dict(loc=(2.3, -4.1, 0.85), target=(2.4, 0.6, 0.2), lens=24.0),      # cam:B side-on, a few m off
    "_trees": dict(loc=(2.4, -6.0, 1.6), target=(0.0, 25.0, 6.0), lens=28.0),
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
    ap.add_argument("--views", default="A,B")
    ap.add_argument("--percent", type=int, default=100)
    ap.add_argument("--suffix", default="")
    ap.add_argument("--frame", type=int, default=60, help="frame to render (wind, birds)")
    ap.add_argument("--fire", type=int, default=90, help="fire frame for the test cockatoos")
    ap.add_argument("--save", default="", help="optional .blend path to save the test scene")
    ap.add_argument("--board", action="store_true", help="clear grass under the (absent) base board")
    ap.add_argument("--uhd-crop", default="", help="render a 3840x2160 frame cropped to 'x0,y0,x1,y1' (0..1)")
    a = ap.parse_args(argv)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.name = "env_test"
    scene.frame_start, scene.frame_end = 0, 600
    t0 = time.time()
    res = build_env(scene, a.quality, board_clear=a.board)
    add_cockatoos(scene, a.fire)
    st = res["stats"]
    print("build %.1fs (mats %.1f terrain %.1f grass+litter %.1f trees %.1f props %.1f) hdri=%s" % (
        time.time() - t0, st["t_mats"], st["t_terrain"], st["t_grass_litter"], st["t_trees"], st["t_props"],
        st["hdri"]))
    print("grass:", st["grass"], "litter:", st["litter"], "trees:", st["trees"], "terrain tris:", st["terrain_tris"])
    print("flat zones max |z|: landing %.4f m, base %.4f m" % (st["flat_landing_max_m"], st["flat_base_max_m"]))
    set_render(scene, a.engine, "preview", samples=a.samples)
    scene.render.resolution_percentage = a.percent
    if a.uhd_crop:
        x0, y0, x1, y1 = (float(v) for v in a.uhd_crop.split(","))
        r = scene.render
        r.resolution_x, r.resolution_y = 3840, 2160
        r.use_border, r.use_crop_to_border = True, True
        r.border_min_x, r.border_min_y, r.border_max_x, r.border_max_y = x0, y0, x1, y1
    scene.frame_set(a.frame)
    os.makedirs(OUT, exist_ok=True)
    names = {"A": "", "B": "_b", "trees": "_trees"}
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


