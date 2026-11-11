"""GrappStar mission animation: ENVIRONMENT module (v3: Harry's yard).

build_env(scene) builds Harry's actual yard (after his photos) around the world origin (centre of the base station's
board top, launch toward +X, landing patch x 3.5..6 m):
- a mown lawn (3 to 6 cm, cut blades, faint mower stripes) that is mostly green with yellow-brown dry patches and a
  few small bare spots, rising gently over ~20 m to a hump and a fence line on the crest; the base and landing patch
  stay flat within +-2 cm (SPEC);
- along the crest a weathered grey post-and-rail fence (3 rails) with a galvanised mesh gate, in front of it an
  electric tape line on thin tread-in posts and star pickets, a big old stump, orange sandstone boulders and a row
  of white sacks;
- behind the fence an open-sided shelter (dark steel roof on timber posts) and a mixed treeline: dark green
  eucalypts, big dense bushy trees, a Norfolk Island pine, dead grey and bare twiggy trees, understorey shrubs
  (all procedural, leaves sway in the wind with the grass);
- Poly Haven "Pretoria Gardens" HDRI (sunny, scattered cumulus, sun 52 deg, green trees round) with a matched sun
  lamp from the side; physical-sky fallback.
add_cockatoos(scene, fire_frame) adds ONE sulphur-crested cockatoo near the fence: it walks (leg swing, bob,
waddle), pecks, looks round with its crest flicking up, startles at the fire and flies off. set_render(scene,
engine, preset) sets EEVEE / Cycles output ("preview" 960x540, "final" 1920x1080, "uhd" 3840x2160).

Assets: render/assets/fetch_env.py downloads them to render/assets/cache (gitignored); without them the module
still builds (procedural fallbacks). Run `render/.venv/bin/python render/env.py` for previews: render/out/
preview_env.png (about cam:A), preview_env_b.png (about cam:B) and, with --views photo, preview_env_photo.png from
about where Harry took his photos.

API for other modules (unchanged): terrain_height(x, y) (numpy, m), sun_direction(), build_env(scene, quality,
clear_zones=[(x, y, r)], board_clear=True, board_half=(hx, hy)), add_cockatoos(scene, fire_frame, ...),
set_render(scene, "EEVEE"|"CYCLES", "preview"|"final"|"uhd", motion_blur=...).
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
# Lawn quality presets. clumps_per_m2: grass clump instances per m2 inside r_near of GRASS_CENTER (a clump is 25 to
# 70 short mown blades); density eases to mid x that by r_full, then to far_min x at r_max where the 3D grass stops
# (the lawn texture carries on); hero: extra density multiplier in HERO_ZONE (action + the cameras' foreground);
# clumps use LOD1 (60 % of the blades, wider) beyond r_near and LOD2 (a third, twice as wide) beyond lod_r.
# litter: multiplier on the (sparse) ground litter. "final" is meant for 4K EEVEE on a 16 GB M3.
GRASS_PRESETS = {
    "draft":   dict(clumps_per_m2=30, hero=1.0, r_near=6.0, mid=0.7, r_full=12.0, r_max=30.0, far_min=0.15,
                    lod_r=9.0, blade_mult=0.5, litter=0.3),
    "preview": dict(clumps_per_m2=60, hero=1.0, r_near=9.0, mid=0.75, r_full=18.0, r_max=40.0, far_min=0.1,
                    lod_r=14.0, blade_mult=0.75, litter=0.6),
    "final":   dict(clumps_per_m2=70, hero=1.0, r_near=13.0, mid=0.7, r_full=20.0, r_max=32.0, far_min=0.12,
                    lod_r=19.0, blade_mult=1.0, litter=1.0),
    "hero":    dict(clumps_per_m2=100, hero=1.0, r_near=15.0, mid=0.75, r_full=24.0, r_max=40.0, far_min=0.1,
                    lod_r=21.0, blade_mult=1.1, litter=1.2),
}
GRASS_QUALITY = "final"
GRASS_DENSITY = GRASS_PRESETS[GRASS_QUALITY]      # alias for the active preset
GRASS_CENTER = (2.5, 6.0)                          # between the action and the fence rise the cameras look at
HERO_ZONE = (-3.5, 8.5, -8.0, 3.0)                 # x0, x1, y0, y1: extra-dense grass (action + the cameras'
#                                                    foreground, which look from -Y)


def _hero_sdf(x, y):
    x0, x1, y0, y1 = HERO_ZONE
    return _box_sdf(x, y - 0.5 * (y0 + y1), x0, x1, 0.5 * (y1 - y0))


# Wind (radians of instance tilt, noise scale 1/m, drift m/s). Animated through the scene time.
WIND = dict(grass_amp=0.0, grass_scale=0.22, leaf_amp=0.0, leaf_scale=0.08, speed=1.6, gust_scale=0.03)  # still
MOW_STRIPE = dict(width=0.95, angle_deg=12.0, lean=0.0)    # no visible mower pattern
SCAN_GRASS = os.path.join(CACHE, "models", "grass_bermuda_01", "grass_bermuda_01_1k.blend")   # Poly Haven scan
SCAN_DENSITY = dict(near=420.0, r_near=11.0, mid=140.0, r_full=20.0, r_max=30.0)   # scanned clumps per m2   # mower stripes: band width (m), direction, blade lean

# ---------------------------------------------------------------------------------------------------------------
# Layout (world metres), after Harry's yard photos: a mown lawn rising gently to a hump and a fence line on the
# crest about 28 m beyond the base, seen from where cam:A stands (looking +Y).
FLAT_CORRIDOR = (-1.6, 6.8, 1.35)       # x0, x1, half width in y: kept within +-1.5 cm (flight line + landing)
BOARD_HALF = (0.65, 0.50)               # grass-free pad under the base board (half sizes, m)
BOARD_THICKNESS = 0.015                 # ground under the board sits this much below z 0
BOARD_CLEAR = True                      # keep grass from poking through the board (build_env(board_clear=...))
TERRAIN_CENTER = (4.0, 10.0)
TERRAIN_DENSE_HALF = 30.0               # dense mesh square half size (0.2 m grid), coarser out to TERRAIN_FAR
TERRAIN_STEP = 0.2
TERRAIN_FAR = 420.0
RISE = dict(y0=6.0, y1=27.0, h=2.3)     # the lawn rises h metres between y0 and y1 to the fence crest
HUMP = dict(xy=(-3.0, 23.0), r=(10.0, 3.5), h=0.6)   # mound below the fence (left of centre in the photos)

FENCE_Y = 28.0                          # weathered post-and-rail fence (3 rails) along the crest
FENCE_X = (-40.0, 46.0)
GATE_X = (22.5, 26.1)                   # galvanised mesh gate in the fence (right of centre in the photos)
POST_SPACING = 2.4
RAIL_HEIGHTS = (0.38, 0.76, 1.14)
TAPE_Y = 25.6                           # electric tape line in front of the rails: thin posts + star pickets
TAPE_HEIGHTS = (0.42, 0.88)
TAPE_POST_SPACING = 6.0
STUMP = dict(xy=(3.6, 28.8), dia=0.95, h=1.35)
BOULDERS = [(8.4, 27.3, 0.6), (11.8, 27.4, 0.55), (-22.0, 26.9, 0.45)]
BAGS = dict(x=(4.6, 7.6), y=27.0, n=9)   # row of white sacks along the fence
SHELTER = dict(xy=(-2.5, 32.6), size=(6.5, 4.2), h=(3.2, 2.6))   # open-sided, dark steel skillion on timber posts
# treeline behind the fence: (x, y, height, seed, species); species gum (dark green eucalypt), bush (big dense dark
# tree), pine (Norfolk Island pine), dead (bare grey gum), bare (twiggy pale deciduous), shrub (understorey)
TREES = [(-27.0, 40.0, 12.0, 1, "dead"), (-24.0, 44.0, 10.5, 2, "gum"), (-18.5, 46.5, 10.0, 3, "gum"),
         (-30.5, 47.0, 10.0, 4, "bush"), (-21.0, 37.5, 6.5, 5, "shrub"), (-17.0, 43.0, 14.0, 6, "pine"),
         (-21.0, 41.0, 11.0, 7, "dead"), (-5.5, 40.0, 16.0, 8, "bush"), (-10.0, 42.5, 11.5, 9, "bush"),
         (1.5, 36.0, 6.0, 10, "shrub"), (4.5, 38.5, 8.0, 11, "gum"), (7.5, 35.0, 5.0, 12, "shrub"),
         (9.5, 44.0, 10.5, 13, "bare"), (13.0, 45.5, 10.0, 14, "bare"), (17.0, 42.0, 9.5, 15, "gum"),
         (21.0, 44.0, 10.5, 16, "gum"), (25.0, 41.0, 9.5, 17, "bush"), (19.5, 47.5, 11.0, 18, "gum"),
         (28.5, 45.0, 10.0, 19, "gum"), (33.0, 40.0, 11.5, 20, "gum"), (37.5, 46.0, 10.5, 21, "bush"),
         (-12.0, 52.0, 12.0, 22, "gum"), (14.0, 55.0, 12.0, 23, "gum"), (2.0, 56.0, 11.0, 24, "bush"),
         (-35.0, 55.0, 12.0, 25, "gum"), (41.0, 55.0, 12.0, 26, "gum"), (-38.0, 41.0, 10.5, 27, "gum")]
MANURE = []                             # mown yard: no dung piles

# ---------------------------------------------------------------------------------------------------------------
# Sky / sun. Poly Haven "Pretoria Gardens" (CC0): sunny with scattered cumulus, sun at 52 deg, green trees and lawn
# all round. Values measured from the 4K file (brightest pixel; integral of the sun region above luminance 50 = sun
# irradiance at world strength 1; sky horizontal irradiance about 1.8: real shadows, not desert harsh).
HDRI_NAME = "pretoria_gardens"
HDRI_SUN_U, HDRI_SUN_V = 0.60022, 0.78931
HDRI_SUN_IRR = 5.07
HDRI_SUN_RGB = (1.0, 0.95, 0.88)
HDRI_CLAMP = 25.0                       # removes the sun disc from the HDRI; the sun lamp replaces it
SUN_AZIMUTH_DEG = 78.0                  # direction TO the sun, CCW from +X: AHEAD of the cameras (they look +Y), a
SUN_ELEVATION_DEG = 26.0                #   little to the right and just above frame: backlit, glowing, sparkling
#                                         grass, rim light, long shadows toward the camera. build.py can re-aim it
#                                         with set_sun(scene, azimuth_deg, elevation_deg) once the cameras are set.
SUN_STRENGTH = 7.5                      # W/m2: strong low sun
SUN_ANGLE_DEG = 0.5
SUN_COLOR = (1.0, 0.85, 0.66)           # about 5000 K, a touch golden
SUN_GLOW = (2500.0, 6.0, 0.9)           # camera-visible sun: disc, inner halo, wide glow (the sky blows out by it)
WORLD_STRENGTH = 1.0                    # sky fill: shadows readable but clearly darker
SKY_CAMERA_GAIN = 1.0                   # the sky as the camera sees it (brightness / saturation)
SKY_CAMERA_SAT = 1.1
EXPOSURE = -0.1
CLOUD_SHADOWS = dict(height=120.0, size=1200.0, cover=0.25, scale=0.008, density=0.45, drift=(1.6, 0.5))
HAZE_COLOR = (0.55, 0.68, 0.85)         # sky just above the horizon
HAZE_DIST = 1400.0                      # m: aerial perspective 1 - exp(-d / HAZE_DIST): only far things

TEX_SIZE = {"dry_ground_rocks": 4.0, "bark_bluegum": 1.82,
            "rough_wood": 0.5, "Grass004": 1.4}


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


def _fields(x, y):
    """All terrain / lawn fields at world (x, y). Returns dict of arrays."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    x0, x1, hy = FLAT_CORRIDOR
    sd = _box_sdf(x, y, x0, x1, hy)
    flat = 1.0 - _sstep(0.0, 2.2, sd)                    # 1 in the corridor
    flat_w = 1.0 - _sstep(1.5, 9.0, sd)                  # relief fades in over ~9 m
    pad = np.clip(1.0 - _sstep(0.0, 0.35, _box_sdf(x, y, -BOARD_HALF[0], BOARD_HALF[0], BOARD_HALF[1])), 0, 1)
    tdx = np.maximum(np.abs(x - TERRAIN_CENTER[0]), np.abs(y - TERRAIN_CENTER[1]))
    detail = 1.0 - _sstep(TERRAIN_DENSE_HALF - 4.0, TERRAIN_DENSE_HALF + 1.0, tdx)

    def roll_fn(px, py):
        return 0.35 * _nz(1).fbm(px / 26.0 + 3.1, py / 26.0 - 7.7, 3) + 0.12 * _nz(2).fbm(px / 9.0, py / 9.0, 2)

    roll = roll_fn(x, y) - roll_fn(np.array(0.0), np.array(0.0))
    # the rise to the fence crest (wavy crest line), the hump below it, gentle fall-away behind
    crest = RISE["y1"] + 1.2 * np.sin(x / 9.0) + 0.8 * _nz(17).fbm(x / 14.0, 0.5, 2)
    rise = RISE["h"] * _sstep(RISE["y0"], crest, y) * (1.0 + 0.12 * np.sin(x / 13.0 + 1.0))
    rise = rise - 0.35 * _sstep(crest + 6.0, crest + 30.0, y)
    hx, hyy = HUMP["xy"]
    hump = HUMP["h"] * np.exp(-((x - hx) / HUMP["r"][0]) ** 2 - ((y - hyy) / HUMP["r"][1]) ** 2)
    hump += 0.3 * np.exp(-((x - 13.0) / 6.0) ** 2 - ((y - 23.5) / 2.5) ** 2)
    hum = (0.03 * _nz(3).fbm(x / 2.0, y / 2.0, 3) + 0.008 * _nz(4).fbm(x / 0.5, y / 0.5, 2))
    z = ((roll + rise + hump) * (1 - flat_w) + hum * detail * (1 - 0.8 * flat) * (1 - pad)
         - BOARD_THICKNESS * pad)

    # lawn: patchy dry (yellow-brown) areas, mostly on the hump and the crest; a few small bare spots
    dry_n = _nz(10).fbm(x / 5.5, y / 5.5, 3) + 0.45 * _nz(18).fbm(x / 1.6, y / 1.6, 2)
    dry = _sstep(-0.05, 0.5, dry_n + 1.1 * _sstep(14.0, 24.0, y) * (hump / HUMP["h"] + 0.35))
    dry = np.clip(dry * (1 - 0.6 * flat) + 0.25 * _sstep(-0.3, 0.6, _nz(19).fbm(x / 1.1, y / 1.1, 2)), 0, 1)
    bare_n = _nz(6).fbm(x / 1.8 + 40, y / 1.8, 3) + 0.4 * _nz(16).fbm(x / 0.5, y / 0.5, 2)
    soil = _sstep(0.5, 0.68, bare_n) * (1 - 0.7 * flat) * (0.5 + 0.5 * dry)
    near_fence = 1.0 - _sstep(0.4, 1.6, np.abs(y - FENCE_Y) + 1e3 * ((x < FENCE_X[0]) | (x > FENCE_X[1])))
    under_fence = 1.0 - _sstep(0.2, 0.6, np.abs(y - FENCE_Y))       # the mower misses it: longer grass
    beyond = _sstep(FENCE_Y + 0.3, FENCE_Y + 1.5, y)                 # unmown behind the fence
    gh = np.clip(0.3 + 0.08 * _nz(9).fbm(x / 3.0, y / 3.0, 2) + 0.6 * under_fence + 0.7 * beyond, 0, 1)
    padc = 0.0
    if BOARD_CLEAR:
        bsd = _box_sdf(x, y, -BOARD_HALF[0] + 0.05, BOARD_HALF[0] - 0.05, BOARD_HALF[1] - 0.05)
        padc = 1.0 - _sstep(-0.06, 0.08, bsd + 0.04 * _nz(13).fbm(x / 0.15, y / 0.15, 2))
    dens = (1 - 0.85 * _sstep(0.38, 0.62, soil)) * (1 - padc)     # thin out only where bare soil shows
    lit = np.zeros(x.shape)
    for tx, ty, th, _s, sp in TREES:
        if sp == "shrub":
            continue
        rr = 0.3 * th
        lit = np.maximum(lit, np.exp(-np.maximum(np.hypot(x - tx, y - ty) - 0.5 * rr, 0.0) / (0.5 * rr)))
    a = math.radians(MOW_STRIPE["angle_deg"])
    stripe = np.floor((x * math.cos(a) + y * math.sin(a)) / MOW_STRIPE["width"]) % 2
    return dict(z=z, soil=soil, dry=dry, gh=gh, dens=np.clip(dens, 0, 1), flat=flat, near_fence=near_fence,
                beyond=beyond, pad=pad, lit=lit, stripe=stripe, track=np.zeros(x.shape), trod=np.zeros(x.shape),
                green=1.0 - dry, outside=beyond, graze=np.zeros(x.shape),
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
    el = math.radians(SUN_ELEVATION_DEG)
    az = math.radians(SUN_AZIMUTH_DEG)
    return (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))


def _env_lookup(nt, vec, img):
    mp = nt.node("ShaderNodeMapping", vector_type="POINT")
    nt.set(mp.inputs["Vector"], vec)
    mp.inputs["Rotation"].default_value = (0.0, 0.0, _hdri_rotation())
    mp.name = mp.label = "env:hdri_map"
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
        nt.set(bgc.inputs["Color"], nt.mix(1.0, nt.hsv(col, 0.5, SKY_CAMERA_SAT, 1.0), _sun_glow(nt, tc), "ADD"))
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
    L.color = SUN_COLOR
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


def _sun_glow(nt, tc):
    """Camera-visible sun: a bright disc plus halo lobes around the sun lamp direction (node env:sun_dir), so the
    sky near the sun blows out whatever the HDRI shows there (its own sun is clamped away)."""
    dirn = nt.vmath("NORMALIZE", tc.outputs["Generated"])
    sd = nt.node("ShaderNodeCombineXYZ")
    sd.name = sd.label = "env:sun_dir"
    for k, v in zip("XYZ", sun_direction()):
        sd.inputs[k].default_value = v
    cosang = nt.vmath("DOT_PRODUCT", dirn, sd.outputs[0])
    x = nt.math("SUBTRACT", cosang, 1.0)
    glow = None
    for strength, k in zip(SUN_GLOW, (60000.0, 220.0, 9.0)):
        lobe = nt.math("MULTIPLY", nt.math("EXPONENT", nt.math("MULTIPLY", x, k)), strength / WORLD_STRENGTH)
        glow = lobe if glow is None else nt.math("ADD", glow, lobe)
    c = nt.node("ShaderNodeCombineColor")
    for k, w in zip(("Red", "Green", "Blue"), SUN_COLOR):
        nt.set(c.inputs[k], nt.math("MULTIPLY", glow, w))
    return c.outputs[0]


def _cloud_plane_center(h):
    d = sun_direction()
    t = h / max(d[2], 0.05)
    return (GRASS_CENTER[0] + d[0] * t, GRASS_CENTER[1] + d[1] * t, h)


def _build_cloud_shadows(coll, scene):
    """Soft cloud shadows drifting across the paddock: a big plane high toward the sun, invisible to the camera,
    with a noise-masked transparent material that only dims the sunlight passing through it."""
    cs = CLOUD_SHADOWS
    m, nt = _new_mat("env:cloud_shadow")
    tc = nt.node("ShaderNodeTexCoord")
    nz = nt.noise(tc.outputs["Object"], cs["scale"], 4.0, 0.55)        # object space is metres: ~1/scale m
    mask = nt.ss(nz.outputs["Fac"], 0.5 + 0.12 * (1 - 2 * cs["cover"]), 0.62 + 0.12 * (1 - 2 * cs["cover"]))
    trans = nt.node("ShaderNodeBsdfTransparent")
    nt.set(trans.inputs["Color"], nt.mix(mask, (1, 1, 1), [1 - cs["density"]] * 3))
    nt.output(trans.outputs[0])
    for k, v in (("surface_render_method", "DITHERED"), ("use_transparent_shadow", True)):
        try:
            setattr(m, k, v)
        except Exception:
            pass
    me = bpy.data.meshes.new("env:cloud_shadows")
    hs = cs["size"] / 2
    me.from_pydata([(-hs, -hs, 0), (hs, -hs, 0), (hs, hs, 0), (-hs, hs, 0)], [], [(0, 1, 2, 3)])
    me.materials.append(m)
    ob = _obj("env:cloud_shadows", me, coll, _cloud_plane_center(cs["height"]))
    for k in ("visible_camera", "visible_diffuse", "visible_glossy", "visible_transmission", "visible_volume_scatter"):
        try:
            setattr(ob, k, False)
        except Exception:
            pass
    ob.visible_shadow = True
    _key_cloud_drift(ob, scene)
    return ob


def _key_cloud_drift(ob, scene):
    fps = scene.render.fps / scene.render.fps_base
    c = np.array(ob.location[:])
    dx, dy = CLOUD_SHADOWS["drift"]
    f1 = 20000
    for i, v in enumerate((dx, dy)):
        _key_curve(ob, "location", i, [0, f1], [c[i], c[i] + v * f1 / fps])


def set_sun(scene, azimuth_deg, elevation_deg):
    """Aim the sun: moves the sun lamp, rotates the HDRI so its own sun azimuth lines up with the lamp, re-aims
    the camera-visible sun glow and moves the cloud-shadow plane so its shadows still fall on the paddock.
    azimuth_deg: direction TO the sun, CCW from +X (cameras look +Y, so ~90 is straight ahead); elevation_deg
    above the horizon."""
    global SUN_AZIMUTH_DEG, SUN_ELEVATION_DEG
    SUN_AZIMUTH_DEG, SUN_ELEVATION_DEG = float(azimuth_deg), float(elevation_deg)
    from mathutils import Vector
    d = Vector(sun_direction())
    ob = bpy.data.objects.get("env:sun")
    if ob is not None:
        ob.rotation_euler = d.to_track_quat("Z", "Y").to_euler()
        ob.location = d * 30.0
    trees = [scene.world.node_tree] if scene.world and scene.world.node_tree else []
    for nt in trees:
        for n in nt.nodes:
            if n.name.startswith("env:hdri_map"):
                n.inputs["Rotation"].default_value = (0.0, 0.0, _hdri_rotation())
            if n.name.startswith("env:sun_dir"):
                for k, v in zip("XYZ", d):
                    n.inputs[k].default_value = v
    cl = bpy.data.objects.get("env:cloud_shadows")
    if cl is not None:
        ad = cl.animation_data
        if ad and ad.action:
            bpy.data.actions.remove(ad.action)
        cl.location = _cloud_plane_center(CLOUD_SHADOWS["height"])
        _key_cloud_drift(cl, scene)
    return d


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
    """Mown lawn ground: dense short sward (ambientCG Grass004) under the 3D blades, yellow-brown dry patches,
    faint mower stripes, a few small bare soil spots, haze in the distance."""
    m, nt = _new_mat("env:ground")
    uv = nt.node("ShaderNodeUVMap", uv_map="UVMap").outputs["UV"]
    geo = nt.node("ShaderNodeNewGeometry")
    pos = geo.outputs["Position"]
    soil_a = nt.attr("soil").outputs["Fac"]
    dry_a = nt.attr("dry").outputs["Fac"]
    stripe_a = nt.attr("stripe").outputs["Fac"]
    brk = nt.noise(pos, 1.5, 4.0, 0.6).outputs["Fac"]
    brk2 = nt.noise(pos, 6.0, 2.0, 0.5).outputs["Fac"]
    macro = nt.noise(pos, 0.08, 3.0, 0.55).outputs["Fac"]
    sw_c, sw_r, sw_n, _ = _pbr(nt, "Grass004", _uvvec(nt, uv, TEX_SIZE["Grass004"], 0.2), ((0.07, 0.13, 0.03), 0.8))
    sw2_c, _, _, _ = _pbr(nt, "Grass004", _uvvec(nt, uv, 3.3, 1.9, (0.41, 0.13)), ((0.07, 0.13, 0.03), 0.8))
    g_c, g_r, g_n, _ = _pbr(nt, "dry_ground_rocks", _uvvec(nt, uv, TEX_SIZE["dry_ground_rocks"], 0.3),
                           ((0.20, 0.15, 0.10), 0.9))
    sw = nt.mix(nt.ss(brk, 0.35, 0.65), sw_c, sw2_c)
    sw = nt.hsv(sw, nt.mixf(macro, 0.495, 0.505), 0.95, nt.mixf(macro, 0.84, 0.94))    # even lawn tone
    dryc = nt.mix(nt.ss(brk2, 0.3, 0.8), (0.20, 0.17, 0.08), (0.15, 0.11, 0.06))
    drym = nt.ss(nt.math("ADD", dry_a, nt.math("MULTIPLY", nt.math("SUBTRACT", brk2, 0.5), 0.5)), 0.3, 0.7)
    col = nt.mix(nt.math("MULTIPLY", drym, 0.0), sw, nt.mix(0.6, sw, dryc))           # even lawn: no dry blobs
    col = nt.mix(nt.math("MULTIPLY", stripe_a, 0.0), col, col)
    edge = nt.math("MULTIPLY", nt.math("SUBTRACT", brk, 0.5), 0.6)
    soil_m = nt.ss(nt.math("ADD", soil_a, edge), 0.4, 0.65)
    col = nt.mix(soil_m, col, nt.hsv(g_c, 0.5, 0.8, 0.8))
    rough = nt.mixf(soil_m, sw_r, g_r)
    ncol = nt.mix(soil_m, sw_n, g_n) if (sw_n is not None and g_n is not None) else sw_n
    cam = nt.node("ShaderNodeCameraData")
    dist = cam.outputs["View Distance"]
    normal = _nmap(nt, ncol, nt.mixf(soil_m, 0.7, 1.0))
    bsdf = nt.principled(col, nt.math("MAXIMUM", rough, 0.9), normal, spec=0.02)    # rough, matte
    shader = _hazed(nt, bsdf)
    em = nt.node("ShaderNodeEmission")
    em.inputs["Color"].default_value = tuple(0.8 * WORLD_STRENGTH * SKY_CAMERA_GAIN * c for c in HAZE_COLOR) + (1,)
    lp = nt.node("ShaderNodeLightPath")
    fac = nt.math("MULTIPLY", nt.ss(dist, 150.0, 400.0, 0.0, 0.8), lp.outputs["Is Camera Ray"])
    nt.output(nt.mixsh(fac, shader, em.outputs[0]))
    return m


def _mat_grass():
    """Mown lawn blades: greens with yellow-green and straw-gold dry patches (instance attribute idry, mottled
    blade by blade), darker at the root. Waxy: specular with per-blade random roughness and a per-blade random
    normal tilt, so highlights pop on some blades and not others; strong translucency so backlit blades glow."""
    m, nt = _new_mat("env:grass_blade")
    h = nt.attr("h01").outputs["Fac"]
    br = nt.attr("brnd").outputs["Fac"]
    dry = nt.attr("dry").outputs["Fac"]
    idry = nt.attr("idry", "INSTANCER").outputs["Fac"]
    tint = nt.attr("tint", "INSTANCER").outputs["Fac"]
    oi = nt.node("ShaderNodeObjectInfo")
    wn = nt.node("ShaderNodeTexWhiteNoise", noise_dimensions="1D")
    nt.set(wn.inputs["W"], nt.math("ADD", br, nt.math("MULTIPLY", oi.outputs["Random"], 7.31)))
    rnd = wn.outputs["Value"]
    green = nt.ramp(br, [(0.0, (0.07, 0.13, 0.03)), (0.4, (0.11, 0.165, 0.04)), (0.75, (0.16, 0.19, 0.055)),
                         (1.0, (0.20, 0.21, 0.075))])
    green = nt.mix(nt.ss(tint, 0.0, 1.0), nt.mix(0.4, green, (0.05, 0.12, 0.03)),
                   nt.mix(0.45, green, (0.20, 0.20, 0.07)))
    green = nt.hsv(green, nt.mixf(oi.outputs["Random"], 0.49, 0.51), 1.0, nt.mixf(oi.outputs["Random"], 0.85, 1.12))
    yel = nt.ramp(br, [(0.0, (0.20, 0.19, 0.055)), (0.5, (0.27, 0.22, 0.07)), (1.0, (0.22, 0.16, 0.06))])
    gold = nt.ramp(br, [(0.0, (0.36, 0.28, 0.12)), (1.0, (0.24, 0.17, 0.08))])
    dfac = nt.math("MAXIMUM", nt.math("MULTIPLY", idry, nt.mixf(rnd, 0.15, 1.1)), dry)    # mottled per blade
    col = nt.mix(nt.ss(dfac, 0.15, 0.7), green, yel)
    col = nt.mix(nt.ss(dfac, 0.65, 1.0), col, gold)
    shade = nt.ramp(h, [(0.0, (0.3, 0.3, 0.26)), (0.35, (0.75, 0.75, 0.7)), (0.7, (1, 1, 1)),
                        (1.0, (1.1, 1.08, 1.0))])
    col = nt.mix(1.0, col, shade, "MULTIPLY")
    # per-blade normal tilt
    geo = nt.node("ShaderNodeNewGeometry")
    wv = nt.node("ShaderNodeTexWhiteNoise", noise_dimensions="1D")
    nt.set(wv.inputs["W"], nt.math("ADD", br, 3.7))
    tilt = nt.vmath("SCALE", nt.vmath("SUBTRACT", wv.outputs["Color"], (0.5, 0.5, 0.5)))
    tilt.node.inputs["Scale"].default_value = 0.4
    nrm = nt.vmath("NORMALIZE", nt.vmath("ADD", geo.outputs["Normal"], tilt))
    rough = nt.mixf(rnd, 0.58, 0.75)                   # matte: only the odd blade glints
    rough = nt.mixf(nt.ss(dfac, 0.4, 1.0), rough, 0.55)
    bsdf = nt.principled(col, rough, nrm, spec=nt.mixf(nt.ss(rnd, 0.85, 1.0), 0.18, 0.45))
    tr = nt.node("ShaderNodeBsdfTranslucent")
    nt.set(tr.inputs["Color"], nt.mix(0.45, nt.mix(1.0, col, (1.6, 1.6, 1.2), "MULTIPLY"), (0.42, 0.50, 0.10)))
    nt.output(_hazed(nt, nt.mixsh(nt.mixf(dfac, 0.42, 0.3), bsdf, tr.outputs[0])))
    _thin(m)
    return m


def _mat_sandstone():
    m, nt = _new_mat("env:sandstone")
    tc = nt.node("ShaderNodeTexCoord")
    obj = tc.outputs["Object"]
    wave = nt.node("ShaderNodeTexWave", wave_type="BANDS", bands_direction="Z")
    nt.set(wave.inputs["Vector"], obj)
    wave.inputs["Scale"].default_value = 3.0
    wave.inputs["Distortion"].default_value = 8.0
    wave.inputs["Detail"].default_value = 3.0
    nz = nt.noise(obj, 6.0, 6.0, 0.65)
    grain = nt.noise(obj, 60.0, 3.0, 0.6)
    col = nt.mix(nt.ss(wave.outputs["Fac"], 0.2, 0.9), (0.42, 0.18, 0.07), (0.55, 0.30, 0.13))
    col = nt.mix(nt.math("MULTIPLY", nt.ss(nz.outputs["Fac"], 0.5, 0.7), 0.6), col, (0.30, 0.22, 0.15))
    geo = nt.node("ShaderNodeNewGeometry")
    top = nt.ss(nt.vmath("DOT_PRODUCT", geo.outputs["Normal"], (0, 0, 1)), 0.6, 0.95)
    col = nt.mix(nt.math("MULTIPLY", top, 0.45), col, (0.22, 0.20, 0.16))          # weathered grey tops
    bump = nt.node("ShaderNodeBump")
    nt.set(bump.inputs["Height"], nt.math("ADD", nz.outputs["Fac"], nt.math("MULTIPLY", grain.outputs["Fac"], 0.3)))
    bump.inputs["Strength"].default_value = 0.6
    bump.inputs["Distance"].default_value = 0.02
    nt.output(_hazed(nt, nt.principled(col, 0.95, bump.outputs["Normal"], spec=0.15)))
    return m


def _mat_sack():
    """White woven polypropylene sacks, a little dirty."""
    m, nt = _new_mat("env:sack")
    tc = nt.node("ShaderNodeTexCoord")
    obj = tc.outputs["Object"]
    wv = nt.node("ShaderNodeTexWave", wave_type="BANDS", bands_direction="X")
    nt.set(wv.inputs["Vector"], obj)
    wv.inputs["Scale"].default_value = 180.0
    dirt = nt.noise(obj, 3.0, 4.0, 0.6).outputs["Fac"]
    col = nt.mix(nt.math("MULTIPLY", nt.ss(dirt, 0.55, 0.75), 0.5), (0.74, 0.74, 0.71), (0.45, 0.40, 0.32))
    bump = nt.node("ShaderNodeBump")
    nt.set(bump.inputs["Height"], wv.outputs["Fac"])
    bump.inputs["Strength"].default_value = 0.15
    nt.output(_hazed(nt, nt.principled(col, 0.92, bump.outputs["Normal"], spec=0.12)))
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
    nt.output(nt.principled(col, 0.92, bump.outputs["Normal"], spec=0.15))
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
    c = nt.mix(0.35, c, (0.22, 0.20, 0.17), "MIX")             # weathered grey-brown, darker
    sep = nt.node("ShaderNodeSeparateXYZ")
    nt.set(sep.inputs["Vector"], geo.outputs["Position"])
    gz = nt.attr("gz").outputs["Fac"]
    zfac = nt.ss(nt.math("SUBTRACT", sep.outputs["Z"], gz), 0.25, 0.0)
    c = nt.mix(nt.math("MULTIPLY", zfac, 0.6), c, nt.mix(1.0, c, (0.55, 0.48, 0.40), "MULTIPLY"))
    nt.output(_hazed(nt, nt.principled(c, nt.math("MAXIMUM", r, 0.85), _nmap(nt, n, 2.0), spec=0.15)))
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


# kind -> blade spec for the mown lawn. Blades are cut (blunt tips), stiff and short.
GRASS_SPEC = {
    "lawn": dict(n=64, L=(0.03, 0.065), W=(0.0045, 0.0075), R=0.06, segs=3, lean=(0.05, 0.65), bend=(0.0, 0.5),
                 pdry=0.06),
    "lawn_long": dict(n=48, L=(0.05, 0.15), W=(0.003, 0.0055), R=0.065, segs=4, lean=(0.1, 0.9), bend=(0.2, 1.2),
                      pdry=0.12),
}
GRASS_KINDS = ["lawn", "lawn_long"]


def _cut_width(t):
    return (1.0 - 0.35 * t) * (0.8 + 0.2 * np.minimum(1, t * 4))


def _make_clump(kind, rng, mult=1.0, lod=0):
    mb = MB()
    U = rng.uniform
    if kind == "clover":                                 # white clover patch among the blades
        for i in range(int(rng.integers(10, 18))):
            r = 0.08 * math.sqrt(rng.random())
            a = rng.random() * 2 * math.pi
            top = _strip(mb, rng, np.array([r * math.cos(a), r * math.sin(a), -0.008]), U(0.015, 0.04), 0.0015, 2,
                         U(0.0, 0.3), 0.0, rng.random() * 6.283, 0.2 + 0.2 * rng.random(), 0.0,
                         width_fn=lambda t: np.ones_like(t))
            rot = rng.random() * 2 * math.pi
            br = 0.15 * rng.random()
            for k in range(3):
                aa = rot + k * 2 * math.pi / 3
                dv = np.array([math.cos(aa), math.sin(aa), 0.0])
                lr = U(0.005, 0.008)
                _fan(mb, top + dv * lr * 0.95, lambda t, lr=lr: lr * (0.9 + 0.12 * math.cos(t)), 7,
                     np.array([0, 0, 1.0]) + dv * U(-0.1, 0.35), br, 0, h01=0.85)
        return mb
    s = GRASS_SPEC[kind]
    n = max(5, int(round(s["n"] * mult * (1.0, 0.6, 0.33)[lod])))
    segs = min(s["segs"], (9, 3, 2)[lod])
    wmul = (1.0, 1.35, 2.2)[lod]                        # close to the coverage of LOD0, not tuftier
    for i in range(n):
        r = s["R"] * math.sqrt(rng.random())
        a = rng.random() * 2 * math.pi
        base = np.array([r * math.cos(a), r * math.sin(a), -0.008])
        dry = 1.0 if rng.random() < s["pdry"] else 0.0
        _strip(mb, rng, base, U(*s["L"]), U(*s["W"]) * wmul, segs, U(*s["lean"]), U(*s["bend"]),
               a + rng.normal(0, 0.8), rng.random(), dry, width_fn=_cut_width if kind == "lawn" else None,
               twist=rng.normal(0, 0.5))
    if lod == 0:
        for i in range(rng.integers(2, 6)):          # clippings / flattened blades in the thatch
            a = rng.random() * 2 * math.pi
            _strip(mb, rng, np.array([U(-0.05, 0.05), U(-0.05, 0.05), -0.004]), U(0.02, 0.06), U(0.003, 0.005), 2,
                   U(1.35, 1.5), 0.0, a, rng.random(), float(rng.random() < 0.5))
    return mb


_VARIANTS = [("lawn", 0, 5), ("lawn_long", 0, 3), ("lawn", 1, 3), ("lawn_long", 1, 2), ("lawn", 2, 2),
             ("lawn_long", 2, 2), ("clover", 0, 2)]


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
    dj = d + rng.normal(0, 1.5, N)                     # dithered LOD boundaries (no visible rings)
    lod = np.where(hzone | (dj <= preset["r_near"]), 0, np.where(dj <= preset["lod_r"], 1, 2))
    cat = (F["gh"] + rng.normal(0, 0.12, N) > 0.62).astype(int)
    for ci, kind in enumerate(GRASS_KINDS):
        for lv in (0, 1, 2):
            m = (cat == ci) & (lod == lv)
            ids = np.array(index[(kind, lv)])
            inst[m] = ids[rng.integers(0, len(ids), m.sum())]
    clov = (lod < 2) & (cat == 0) & (rng.random(N) < 0.05 * (0.4 + _sstep(-0.2, 0.4, _nz(22).fbm(X / 2.0, Y / 2.0, 2))))
    inst[clov] = np.array(index[("clover", 0)])[rng.integers(0, 2, clov.sum())]
    return inst


def _mat_scan_grass():
    """Poly Haven grass_bermuda_01 scan: photo colour, alpha cut-out, normal and roughness maps; matte, a little
    translucent; colour evened out to a natural lawn green."""
    m, nt = _new_mat("env:scan_grass")
    d = os.path.dirname(SCAN_GRASS)
    tx = nt.node("ShaderNodeTexCoord").outputs["UV"]

    def img(name, cs):
        pth = os.path.join(d, "textures", name)
        if not os.path.exists(pth):
            return None
        im = bpy.data.images.load(pth, check_existing=True)
        im.colorspace_settings.name = cs
        t = nt.node("ShaderNodeTexImage", image=im, interpolation="Linear")
        nt.set(t.inputs["Vector"], tx)
        return t
    dif = img("grass_bermuda_01_diff_1k.jpg", "sRGB")
    alp = img("grass_bermuda_01_alpha_1k.png", "Non-Color")
    nor = img("grass_bermuda_01_nor_gl_1k.exr", "Non-Color")
    rou = img("grass_bermuda_01_rough_1k.exr", "Non-Color")
    oi = nt.node("ShaderNodeObjectInfo")
    col = dif.outputs["Color"] if dif else (0.08, 0.15, 0.04)
    col = nt.hsv(col, nt.mixf(oi.outputs["Random"], 0.497, 0.505), 1.0, nt.mixf(oi.outputs["Random"], 0.9, 1.06))
    rough = nt.math("MAXIMUM", rou.outputs["Color"], 0.6) if rou else 0.65
    bsdf = nt.principled(col, rough, _nmap(nt, nor.outputs["Color"] if nor else None, 1.0), spec=0.25)
    tr = nt.node("ShaderNodeBsdfTranslucent")
    nt.set(tr.inputs["Color"], col)
    sh = nt.mixsh(0.25, bsdf, tr.outputs[0])
    if alp is not None:
        tp = nt.node("ShaderNodeBsdfTransparent")
        a = nt.ss(alp.outputs["Color"], 0.45, 0.55)
        sh = nt.mixsh(a, tp.outputs[0], sh)
    nt.output(_hazed(nt, sh))
    for k, v in (("surface_render_method", "DITHERED"), ("use_backface_culling", False),
                 ("use_transparent_shadow", True)):
        try:
            setattr(m, k, v)
        except Exception:
            pass
    return m


def _load_scan_grass(src_coll):
    """Link the scanned lawn clumps (small / medium / seedling cards) into src_coll as instance sources."""
    if not os.path.exists(SCAN_GRASS):
        return []
    with bpy.data.libraries.load(SCAN_GRASS) as (src, dst):
        dst.objects = [n for n in src.objects if n.startswith("grass_bermuda_01_") and "geometry" not in n
                       and "single" not in n]
    mat = _mat_scan_grass()
    objs = []
    for i, o in enumerate(sorted([o for o in dst.objects if o and o.type == "MESH"], key=lambda o: o.name)):
        o.name = "env:gsrc_%02d_scan_%s" % (i, o.name.replace("grass_bermuda_01_", ""))
        o.location = (0, 0, 0)
        o.data.materials.clear()
        o.data.materials.append(mat)
        src_coll.objects.link(o)
        objs.append(o)
    return objs


def _build_scan_grass(coll, src_coll, axes, Zgrid, objs, clear_zones=()):
    """Dense, even lawn of scanned clumps: jittered grid, density easing off with distance from the action and
    the cameras' foreground, random rotation / scale, no wind."""
    rng = np.random.default_rng(SEED + 17)
    sd = SCAN_DENSITY
    cx, cy = GRASS_CENTER[0], GRASS_CENTER[1] - 3.0
    Ps = []
    for r0, r1, dens in ((0.0, sd["r_near"], sd["near"]), (sd["r_near"], sd["r_full"], sd["mid"]),
                         (sd["r_full"], sd["r_max"], sd["mid"] * 0.45)):
        cell = 1.0 / math.sqrt(dens)
        g = np.arange(-r1, r1, cell)
        X, Y = np.meshgrid(g + cx, g + cy, indexing="ij")
        X = X.ravel() + rng.random(X.size) * cell
        Y = Y.ravel() + rng.random(Y.size) * cell
        d = np.hypot(X - cx, Y - cy)
        k = (d >= r0) & (d < r1)
        Ps.append(np.stack([X[k], Y[k]], 1))
    P = np.concatenate(Ps)
    F = _fields(P[:, 0], P[:, 1])
    keep = rng.random(len(P)) < F["dens"]
    for (zx, zy, zr) in clear_zones:
        keep &= np.hypot(P[:, 0] - zx, P[:, 1] - zy) > zr
    P = P[keep]
    N = len(P)
    Z = _interp_grid(axes, Zgrid, P[:, 0], P[:, 1]) - 0.004
    inst = rng.integers(0, len(objs), N).astype(np.int32)
    s = rng.uniform(0.75, 1.1, N)
    sz = s * rng.uniform(0.62, 0.8, N)                 # mown: clumps kept 5 to 8 cm tall
    rot = np.stack([rng.normal(0, 0.06, N), rng.normal(0, 0.06, N), rng.random(N) * 2 * math.pi], 1)
    ob = _points_object("env:grass", coll, np.column_stack([P, Z]), inst, rot, np.stack([s, s, sz], 1), {})
    ob.modifiers.new("scatter", "NODES").node_group = _instance_nodes("env:grass_scatter", src_coll)
    tris = np.array([sum(len(pp.vertices) - 2 for pp in o.data.polygons) for o in objs])
    return ob, dict(instances=int(N), triangles=int(tris[inst].sum()), sources=len(objs), scanned=True)


def _build_grass(coll, src_coll, terrain_axes, Zgrid, preset, mats, clear_zones=()):
    scan = _load_scan_grass(src_coll)
    if scan:
        return _build_scan_grass(coll, src_coll, terrain_axes, Zgrid, scan, clear_zones)
    objs, index, tris = _build_sources(src_coll, "gsrc", _VARIANTS, _make_clump, mats, preset["blade_mult"])
    X, Y, d, F, rng = _scatter_points(preset, clear_zones)
    inst = _choose_instances(X, Y, d, F, rng, index, preset)
    Z = _interp_grid(terrain_axes, Zgrid, X, Y)
    N = len(X)
    s = rng.uniform(0.82, 1.2, N) * (1.0 - 0.35 * F["board"]) * (1.0 + 0.6 * _sstep(preset["r_full"],
                                                                                      preset["r_max"], d))
    scl = np.stack([s, s, s * rng.uniform(0.85, 1.15, N)], 1)
    # mower stripes: blades in alternate bands lean the two ways along the stripe normal (world frame), so the
    # bands catch the light differently; Euler XYZ of Rx(tilt) * Rz(spin)
    tilt = (F["stripe"] * 2 - 1) * MOW_STRIPE["lean"] * (1 - 0.6 * F["beyond"]) + rng.normal(0, 0.07, N)
    spin = rng.random(N) * 2 * math.pi
    ct, st, cs, ss_ = np.cos(tilt), np.sin(tilt), np.cos(spin), np.sin(spin)
    ex = np.arctan2(st * cs, ct)
    ey = -np.arcsin(np.clip(st * ss_, -1, 1))
    ez = np.arctan2(ct * ss_, cs)
    rot = np.stack([ex, ey, ez], 1)
    tint = np.clip(0.5 + 0.55 * _nz(20).fbm(X / 4.0, Y / 4.0, 3), 0, 1)
    ob = _points_object("env:grass", coll, np.stack([X, Y, Z], 1), inst, rot, scl,
                        dict(tint=tint, idry=F["dry"]))
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
    clus = np.clip(0.88 + 0.35 * _nz(21).fbm(X / 0.8, Y / 0.8, 2), 0.65, 1.0)    # mild clustering
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
    cx, cy = 4.0, 18.0
    R = 46.0
    # candidate grid at the max density of the densest kind, per-kind acceptance
    rates = dict(leaf=lambda F: 0.04 + 6.0 * F["lit"] ** 1.5, bark=lambda F: 0.01 + 0.3 * F["lit"] ** 2,
                 twig=lambda F: 0.03 + 0.7 * F["lit"] ** 2, stick=lambda F: 0.004 + 0.12 * F["lit"] ** 2,
                 rock=lambda F: 0.02 + 0.5 * F["soil"], grit=lambda F: 0.15 + 2.5 * F["soil"])
    Ps, I, S = [], [], []
    for kind, fn in rates.items():
        dmax = {"leaf": 6.04, "bark": 0.31, "twig": 0.73, "stick": 0.124, "rock": 0.52, "grit": 2.65}[kind] * lm
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
    if MANURE:
        mp = np.array(MANURE, float).reshape(-1, 2)
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


SPECIES = {
    # fork: first fork (fraction of height); limbs: count; lang: limb angle from vertical (rad); step: leaf
    # cluster spacing along twigs (m, None = leafless); mat: tree mesh material slot (0 gum bark, 2 grey deadwood)
    "gum": dict(fork=(0.3, 0.45), limbs=(3, 5), lang=(0.4, 0.9), step=(0.2, 0.3), mat=0, leaf="gum", rt=0.021,
                kids=((4, 8), (4, 7), (3, 6))),
    "bush": dict(fork=(0.12, 0.22), limbs=(4, 7), lang=(0.6, 1.15), step=(0.15, 0.22), mat=0, leaf="bush", rt=0.023,
                 kids=((5, 9), (4, 8), (4, 7))),
    "dead": dict(fork=(0.3, 0.5), limbs=(2, 4), lang=(0.3, 0.7), step=None, mat=2, leaf=None, rt=0.017,
                 kids=((2, 5), (2, 4), (1, 3))),
    "bare": dict(fork=(0.2, 0.35), limbs=(3, 6), lang=(0.25, 0.65), step=None, mat=2, leaf=None, rt=0.015,
                 kids=((5, 9), (5, 9), (4, 8))),
    "shrub": dict(fork=(0.0, 0.05), limbs=(4, 7), lang=(0.25, 0.7), step=(0.12, 0.18), mat=0, leaf="bush",
                  rt=0.012, kids=((4, 7), (3, 6), (3, 5))),
}
_CAM_A = np.array([2.4, -10.6])


def _gum_tree(mb, rng, base, H, leaf_pts, species="gum"):
    """Tree from the branching generator: trunk forking into limbs, branches and drooping twigs; leaf cluster
    attachment points (position, scale, leaf kind) appended to leaf_pts. Far trees get sparser, bigger clusters."""
    sp = SPECIES[species]
    U = rng.uniform
    pr = rng.random()
    tex = TEX_SIZE["bark_bluegum"]
    far = float(np.clip((np.hypot(*(base[:2] - _CAM_A)) - 40.0) / 30.0, 0.0, 1.0))
    ctx = dict(sp=sp, pr=pr, far=far)
    rt = sp["rt"] * H * U(0.85, 1.2)
    fork = H * U(*sp["fork"])
    if species == "shrub":                               # multi-stemmed from the ground
        top = base + np.array([0, 0, -0.1])
    else:
        lean = np.array([U(-0.1, 0.1), U(-0.1, 0.1), 1.0])
        trunk = _grow(rng, base + np.array([0, 0, -0.3]), lean, fork + 0.3, 9, 0.05, 0.12, 0.0)
        rr = np.linspace(rt, rt * 0.78, len(trunk))
        rr[:2] *= (1.45, 1.15)
        stock_h = U(0.8, 2.6) if (rng.random() < 0.5 and species == "gum") else 0.0
        stock = (trunk[:, 2] - base[2] < stock_h).astype(float)
        _tube(mb, trunk, rr, 12, sp["mat"], tex, dict(prand=pr, stock=stock))
        top = trunk[-1]
    nl = int(rng.integers(*sp["limbs"]))
    az0 = rng.random() * 6.283
    for i in range(nl):
        az = az0 + i * 6.283 / nl + U(-0.4, 0.4)
        ang = U(*sp["lang"])
        d = np.array([math.sin(ang) * math.cos(az), math.sin(ang) * math.sin(az), math.cos(ang)])
        _gum_branch(mb, rng, top, d, (H - fork) * U(0.8, 1.1), max(rt * 0.78 * U(0.55, 0.78), 0.012), 1, leaf_pts,
                    ctx)


def _gum_branch(mb, rng, p0, d0, L, r0, depth, leaf_pts, ctx):
    sp, pr, far = ctx["sp"], ctx["pr"], ctx["far"]
    U = rng.uniform
    nseg = max(3, int(L / (0.7 if depth < 3 else 0.3)))
    pts = _grow(rng, p0, d0, L, nseg, (0.06, 0.1, 0.14, 0.18, 0.2)[depth], (0.1, 0.08, 0.03, -0.01, -0.03)[depth],
                (0.0, 0.05, 0.25, 0.5, 0.7)[depth] * (0.4 if sp["step"] is None else 1.0))
    r1 = max(r0 * 0.35, 0.004)
    radii = np.linspace(r0, r1, len(pts))
    _tube(mb, pts, radii, (12, 8, 6, 4, 3)[depth], sp["mat"], TEX_SIZE["bark_bluegum"], dict(prand=pr, stock=0.0))
    seglen = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0], np.cumsum(seglen)])

    def at(t):
        s = t * cum[-1]
        i = max(min(int(np.searchsorted(cum, s)) - 1, len(pts) - 2), 0)
        f = (s - cum[i]) / max(seglen[i], 1e-9)
        return pts[i] + (pts[i + 1] - pts[i]) * f, (pts[i + 1] - pts[i]) / max(seglen[i], 1e-9), \
            radii[i] + (radii[i + 1] - radii[i]) * f

    def leaves(t0, step):
        if sp["step"] is None:
            return
        t = t0
        while t <= 1.0:
            p, d, _r = at(t)
            leaf_pts.append((p, U(0.85, 1.25) * (1 + 0.5 * far), sp["leaf"], None))
            t += step * (1 + 0.8 * far) / max(cum[-1], 0.3)

    if depth >= 4 or r0 < 0.007:
        if sp["step"] is not None:
            leaves(U(0.1, 0.25), U(*sp["step"]))
        return
    if depth == 3 and sp["step"] is not None:
        leaves(0.5, U(*sp["step"]) * 1.6)
    nch = int(rng.integers(*sp["kids"][depth - 1]))
    for k in range(nch):
        t = U(0.25, 0.98) if depth > 1 else U(0.3, 0.98)
        p, d, r = at(t)
        dch = _perp_rotate(d, U(0.55, 1.1), rng.random() * 6.283)
        if depth == 1:
            dch = dch + np.array([0, 0, 0.25])
        _gum_branch(mb, rng, p, dch, L * U(0.38, 0.62), max(r * U(0.5, 0.72), 0.004), depth + 1, leaf_pts, ctx)
    if sp["step"] is not None:
        p, d, _r = at(1.0)
        leaf_pts.append((p, U(0.9, 1.3) * (1 + 0.5 * far), sp["leaf"], None))


def _pine_tree(mb, rng, base, H, leaf_pts):
    """Norfolk Island pine: straight trunk, tiers of near-horizontal branches shortening upward (conical), foliage
    sprays along every branch."""
    U = rng.uniform
    pr = rng.random()
    rt = 0.014 * H
    trunk = np.array([base + np.array([0, 0, -0.3]), base + np.array([0.05, 0.03, H])])
    trunk = np.array([trunk[0] + (trunk[1] - trunk[0]) * t for t in np.linspace(0, 1, 10)])
    _tube(mb, trunk, np.linspace(rt * 1.3, 0.02, len(trunk)), 10, 2, 1.0, dict(prand=pr, stock=0.0))
    z = H * 0.18
    tier = 0
    while z < H - 0.4:
        u = (z - H * 0.18) / (H * 0.82)
        Lb = 0.32 * H * (1 - u) ** 0.9 + 0.35
        nb = int(rng.integers(5, 8))
        az0 = rng.random() * 6.283
        c = base + np.array([0.05 * z / H, 0.03 * z / H, z])
        for k in range(nb):
            az = az0 + k * 6.283 / nb + U(-0.15, 0.15)
            d = np.array([math.cos(az), math.sin(az), U(-0.05, 0.12)])
            pts = _grow(rng, c, d, Lb, max(3, int(Lb / 0.5)), 0.04, 0.02, -0.04)
            _tube(mb, pts, np.linspace(0.035 * (1 - u) + 0.01, 0.008, len(pts)), 4, 2, 1.0, dict(prand=pr))
            s = 0.25
            while s < Lb:
                p = pts[0] + (pts[-1] - pts[0]) * (s / Lb)
                leaf_pts.append((p, U(0.8, 1.15) * (0.7 + 0.5 * (1 - u)), "pine", az))
                s += U(0.28, 0.4)
        z += U(0.8, 1.2)
        tier += 1
    for k in range(4):
        leaf_pts.append((base + np.array([0.05, 0.03, H - 0.3 * k]), 0.6, "pine", k * 1.6))


def _leaf_flat(mb, M, t0, L, W, curve, lcol, segs=3, mat=0):
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
        mb.face([b + 2 * k, b + 2 * k + 1, b + 2 * k + 3, b + 2 * k + 2], mat)
    mb.face([b + 2 * (segs - 1), b + 2 * (segs - 1) + 1, b + 2 * segs], mat)


def _hang_frame(rng, theta_rng):
    U = rng.uniform
    theta = U(*theta_rng)
    a = rng.random() * 6.283
    x_ax = np.array([math.sin(theta) * math.cos(a), math.sin(theta) * math.sin(a), -math.cos(theta)])
    y_ax = np.cross([0, 0, 1.0], x_ax)
    y_ax /= np.linalg.norm(y_ax) + 1e-9
    roll = U(-0.6, 0.6)
    z_ax = np.cross(x_ax, y_ax)
    y2 = y_ax * math.cos(roll) + z_ax * math.sin(roll)
    return np.stack([x_ax, y2, np.cross(x_ax, y2)], 1)


def _make_leaf_cluster(kind, rng, mult=1.0, lod=0):
    """Foliage clump: 'gum' pendulous eucalypt leaves, 'bush' a dense mass of small leaves, 'pine' a fan of
    Norfolk-pine branchlet tails (crossed ribbons curving up at the tips)."""
    mb = MB()
    U = rng.uniform
    if kind == "gum":
        for i in range(int(rng.integers(36, 54))):
            p = np.array([rng.normal(0, 0.2), rng.normal(0, 0.2), rng.normal(0, 0.13) - 0.06])
            L = U(0.09, 0.16)
            lc = rng.random() ** 1.6 if rng.random() > 0.06 else U(0.85, 1.0)
            _leaf_flat(mb, _hang_frame(rng, (0.2, 1.3)), p, L, L * U(0.14, 0.21), U(0.08, 0.22) * rng.choice([-1, 1]),
                       lc, mat=0)
    elif kind == "bush":
        for i in range(int(rng.integers(60, 85))):
            p = np.array([rng.normal(0, 0.24), rng.normal(0, 0.24), rng.normal(0, 0.17)])
            L = U(0.05, 0.09)
            _leaf_flat(mb, _hang_frame(rng, (0.6, 2.4)), p, L, L * U(0.3, 0.45), U(-0.08, 0.08), rng.random(),
                       mat=1)
    else:
        for i in range(int(rng.integers(7, 11))):
            a = U(-1.0, 1.0)
            d = np.array([math.cos(a), math.sin(a), U(-0.25, 0.0)])
            L = U(0.3, 0.5)
            p0 = np.array([0.0, 0.0, 0.0])
            pts = [p0, p0 + d * L * 0.5, p0 + d * L * 0.85 + np.array([0, 0, 0.05]),
                   p0 + d * L + np.array([0, 0, 0.12])]
            br = rng.random()
            for q in range(3):
                for k in range(2):
                    sv = np.cross(pts[q + 1] - pts[q], [math.cos(k * 1.57), 0.0, math.sin(k * 1.57)]) + 1e-6
                    _ribbon(mb, pts[q], pts[q + 1], 0.028 * (1 - 0.25 * q), sv, br, 2)
    return mb


def _build_trees(coll, src_coll, axes, Zgrid, mats):
    trees, leaf_pts = [], []
    tmats = (mats["gum_bark"], mats["bark_strip"], mats["deadwood"])
    for (x, y, H, sd, species) in TREES:
        rng = np.random.default_rng(SEED * 7 + sd)
        mb = MB()
        z = float(_interp_grid(axes, Zgrid, np.array([x]), np.array([y]))[0])
        if species == "pine":
            _pine_tree(mb, rng, np.array([x, y, z]), H, leaf_pts)
        else:
            _gum_tree(mb, rng, np.array([x, y, z]), H, leaf_pts, species)
        nm = "env:tree_%02d_%s" % (sd, species)
        trees.append(_obj(nm, mb.build(nm, tmats, smooth=True), coll))
    objs, index, tris = _build_sources(src_coll, "leafsrc", [("gum", 0, 5), ("bush", 0, 4), ("pine", 0, 3)],
                                       _make_leaf_cluster, (mats["leaf_gum"], mats["leaf_bush"], mats["leaf_pine"]),
                                       seed=21)
    rng = np.random.default_rng(SEED + 23)
    P = np.array([e[0] for e in leaf_pts])
    s = np.array([e[1] for e in leaf_pts])
    kinds = [e[2] for e in leaf_pts]
    yaws = [e[3] for e in leaf_pts]
    N = len(P)
    inst = np.array([index[(k, 0)][rng.integers(0, len(index[(k, 0)]))] for k in kinds], np.int32)
    s = s * np.where(np.array([k != "pine" for k in kinds]), 1.7, 1.0)       # few big clumps: full crowns, light
    rot = np.stack([rng.normal(0, 0.15, N), rng.normal(0, 0.15, N), rng.random(N) * 6.283], 1)
    for i, yw in enumerate(yaws):
        if yw is not None:
            rot[i] = (rot[i, 0] * 0.3, rot[i, 1] * 0.3, yw + rng.normal(0, 0.25))
    ob = _points_object("env:tree_leaves", coll, P, inst, rot, np.stack([s, s, s], 1), {})
    ob.modifiers.new("scatter", "NODES").node_group = _instance_nodes("env:tree_leaf_scatter", src_coll,
                                                                     WIND["leaf_amp"], WIND["leaf_scale"])
    return trees + [ob], dict(trees=len(trees), clusters=int(N), leaf_tris=int(tris[inst].sum()))


# =============================================================================================================
# fence line on the crest: post-and-rail, electric tape line, mesh gate, stump, boulders, sacks, shelter
def _post_axis(rng, lean_deg):
    a = rng.random() * 2 * math.pi
    t = math.radians(lean_deg)
    return np.array([math.sin(t) * math.cos(a), math.sin(t) * math.sin(a), math.cos(t)])


def _star_picket(mb, rng, xy, z0, axis, h=1.2, mat=3, cap=4):
    """Steel star picket (Y section) with a plastic cap."""
    yaw = rng.random() * 2 * math.pi
    prof = []
    for k in range(3):
        a = yaw + k * 2 * math.pi / 3
        da = np.array([math.cos(a), math.sin(a)])
        nv = np.array([-da[1], da[0]]) * 0.0016
        prof += [nv + da * 0.004, nv + da * 0.030, -nv + da * 0.030, -nv + da * 0.004]
    prof = np.array(prof)
    bot = np.array([xy[0], xy[1], z0 - 0.45])
    top = np.array([xy[0], xy[1], z0]) + axis * h
    ring0 = np.column_stack([prof, np.zeros(len(prof))]) + bot
    ring1 = np.column_stack([prof, np.zeros(len(prof))]) + top
    b = mb.verts(np.vstack([ring0, ring1]))
    n = len(prof)
    for j in range(n):
        mb.face([b + j, b + (j + 1) % n, b + n + (j + 1) % n, b + n + j], mat)
    mb.face([b + n + j for j in range(n)], mat)
    _cyl(mb, top - axis * 0.04, top + axis * 0.035, 0.024, 0.02, rng, cap, nring=10, nseg=1)
    return top


def _tape(mb, pts, w, mat):
    """Electric tape: vertical ribbon through the points."""
    rows = []
    for p in pts:
        rows.append([p + np.array([0, 0, -w / 2]), p + np.array([0, 0, w / 2])])
    b = mb.verts(np.array(rows).reshape(-1, 3))
    for i in range(len(pts) - 1):
        mb.face([b + 2 * i, b + 2 * i + 2, b + 2 * i + 3, b + 2 * i + 1], mat)


def _rail_fence(mb, rng, p0, p1, gate=None, post_h=1.3):
    """Weathered post-and-rail fence (3 rails, rails on the paddock side, a few leaning posts). gate=(s0, s1)
    leaves a gap (distance along the line) for the mesh gate, hung on round gate posts."""
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    dirv = (p1 - p0) / L
    inward = np.array([-dirv[1], dirv[0]]) * -1.0          # paddock side (-Y for a fence along +X)
    ss = np.linspace(0, L, int(round(L / POST_SPACING)) + 1)
    if gate:
        ss = np.array([s for s in ss if s < gate[0] - 0.6 or s > gate[1] + 0.6] + [gate[0] - 0.12, gate[1] + 0.12])
        ss.sort()
    T = TEX_SIZE["rough_wood"]
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
            _cyl(mb, base, top + axis * 0.15, 0.1, 0.09, rng, 0, cap_mat=0, tex=T, nring=14, bumps=0.03, k=61,
                 extra=dict(gz=z0))
        else:
            _beam(mb, base, top, 0.14, 0.15, rng, 0, T, chamfer=0.015, segs=3, top_slope=0.4,
                  up=np.append(dirv, 0), extra=dict(gz=z0))
        posts.append((s, np.array([xy[0], xy[1], z0]), axis))
    off = 0.07 + 0.025 + 0.002
    for (s0, g0, a0), (s1, g1, a1) in zip(posts[:-1], posts[1:]):
        if gate and s0 < gate[1] and s1 > gate[0]:
            continue
        for k, h in enumerate(RAIL_HEIGHTS):
            hh = h + rng.normal(0, 0.012)
            e0 = g0 + a0 * hh + np.append(inward, 0) * off + np.append(dirv, 0) * 0.004
            e1 = g1 + a1 * hh + np.append(inward, 0) * off - np.append(dirv, 0) * 0.004
            _beam(mb, e0, e1, 0.05, 0.145, rng, 0, T, chamfer=0.006, segs=6, sag=rng.uniform(0.0, 0.02),
                  extra=dict(newwood=1.0 if rng.random() < 0.03 else 0.0, gz=min(g0[2], g1[2])))
    return posts


def _tape_line(mb, rng, y, x0, x1):
    """Electric fence in front of the rails: thin white tread-in posts and star pickets carrying two white
    tapes that sag a little between posts."""
    xs = np.arange(x0, x1 + 0.01, TAPE_POST_SPACING)
    sup = []
    for i, x in enumerate(xs):
        xy = np.array([x + rng.normal(0, 0.2), y + rng.normal(0, 0.15) + 0.6 * math.sin(x / 11.0)])
        z0 = terrain_height(xy[0], xy[1])
        axis = _post_axis(rng, rng.uniform(0.5, 5.0))
        if i % 3 == 0:
            top = _star_picket(mb, rng, xy, z0, axis, 1.05, 3, 4)
        else:
            top = np.array([xy[0], xy[1], z0]) + axis * 1.0
            _cyl(mb, np.array([xy[0], xy[1], z0 - 0.15]), top, 0.007, 0.006, rng, 4, nring=6, nseg=1)
        sup.append((np.array([xy[0], xy[1], z0]), axis))
    for h in TAPE_HEIGHTS:
        pts = []
        for (g0, a0), (g1, a1) in zip(sup[:-1], sup[1:]):
            A, B = g0 + a0 * h, g1 + a1 * h
            for k in range(8):
                u = k / 8
                pts.append(A + (B - A) * u - np.array([0, 0, 0.06 * math.sin(math.pi * u)]))
        pts.append(sup[-1][0] + sup[-1][1] * h)
        _tape(mb, pts, 0.012 if h < 0.6 else 0.02, 5)


def _mesh_gate(mb, rng, a_xy, b_xy):
    """Galvanised tube-frame gate with welded mesh (100 mm squares), closed."""
    a = np.asarray(a_xy, float)
    b = np.asarray(b_xy, float)
    dv = (b - a) / np.linalg.norm(b - a)
    W = float(np.linalg.norm(b - a)) - 0.26
    z0 = terrain_height(*(0.5 * (a + b)))
    o = np.append(a + dv * 0.13, z0 + 0.08)
    D = np.append(dv, 0.0)
    Z = np.array([0, 0, 1.0])
    H = 1.15
    for p0, p1, r in ((o, o + Z * H, 0.02), (o + D * W, o + D * W + Z * H, 0.02), (o + Z * H, o + D * W + Z * H, 0.02),
                      (o, o + D * W, 0.018), (o + Z * (H / 2), o + D * W + Z * (H / 2), 0.012)):
        _cyl(mb, p0, p1, r, r, rng, 1, nring=8, nseg=1)
    for k in range(1, int(W / 0.1)):
        _wire_seg(mb, o + D * (k * 0.1), o + D * (k * 0.1) + Z * H)
    for k in range(1, int(H / 0.1)):
        _wire_seg(mb, o + Z * (k * 0.1), o + D * W + Z * (k * 0.1))


def _wire_seg(mb, p0, p1, r=0.002, mat=1):
    _tube(mb, np.array([p0, p1]), np.array([r, r]), 4, mat, 1.0, {})


def _shelter(mb, rng, cfg):
    """Open-sided field shelter: timber posts and a dark steel skillion roof."""
    x, y = cfg["xy"]
    Lx, Wy = cfg["size"]
    hf, hb = cfg["h"]
    T = TEX_SIZE["rough_wood"]
    z = terrain_height(x, y)
    for i, px in enumerate(np.linspace(x - Lx / 2 + 0.2, x + Lx / 2 - 0.2, 4)):
        for py, hh in ((y - Wy / 2 + 0.2, hf), (y + Wy / 2 - 0.2, hb)):
            zg = terrain_height(px, py)
            _beam(mb, (px, py, zg - 0.4), (px, py, z + hh - 0.05), 0.11, 0.11, rng, 0, T, segs=2,
                  extra=dict(gz=zg))
    for py, hh in ((y - Wy / 2 + 0.2, hf), (y + Wy / 2 - 0.2, hb)):
        _beam(mb, (x - Lx / 2, py, z + hh - 0.12), (x + Lx / 2, py, z + hh - 0.12), 0.06, 0.15, rng, 0, T, segs=4,
              extra=dict(gz=z))
    yy = np.linspace(y - Wy / 2 - 0.3, y + Wy / 2 + 0.3, 9)
    xx = np.linspace(x - Lx / 2 - 0.3, x + Lx / 2 + 0.3, 2)
    rows = []
    for py in yy:
        f = (py - (y - Wy / 2)) / Wy
        hz = z + hf + (hb - hf) * f
        for px in xx:
            rows.append([px, py, hz])
    b = mb.verts(np.array(rows))
    for i in range(len(yy) - 1):
        mb.face([b + 2 * i, b + 2 * i + 1, b + 2 * i + 3, b + 2 * i + 2], 6)


def _boulder(mb, rng, xy, r, k):
    z = terrain_height(*xy)
    _blob(mb, rng, np.array([xy[0], xy[1], z + r * 0.35]), np.array([r * 1.3, r, r * 0.75]), 14, 9, 0.18, 0, k=k)


def _sacks(mb, rng, cfg):
    """Row of filled white sacks along the fence, slumped and touching."""
    xs = np.linspace(cfg["x"][0], cfg["x"][1], cfg["n"])
    for i, x in enumerate(xs):
        px, py = x + rng.normal(0, 0.06), cfg["y"] + rng.normal(0, 0.12)
        z = terrain_height(px, py)
        rad = np.array([rng.uniform(0.26, 0.32), rng.uniform(0.2, 0.26), rng.uniform(0.11, 0.15)])
        _blob(mb, rng, np.array([px, py, z + rad[2] * 0.7]), rad, 12, 7, 0.1, 0, k=95 + i % 3)


# =============================================================================================================
# one sulphur-crested cockatoo, walking and pecking near the fence, flying off at the fire event
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


def _bird_parts(mats):
    """Meshes in their own pivot frames (+X forward, +Z up): body (origin at the hips), head (origin at the neck
    base), crest (origin at the crown, feathers laid back), legs (origin at the hip, foot 0.12 below), inner wing
    (origin at the shoulder, span +Y) and outer wing (origin at the wrist)."""
    rng = np.random.default_rng(SEED + 31)
    body = MB()
    _blob(body, rng, np.array([0.02, 0, 0.05]), np.array([0.12, 0.066, 0.07]), 12, 8, 0.03, 0, k=81)
    for side in (-1, 1):                                               # folded wings along the flanks
        _blob(body, rng, np.array([-0.03, side * 0.056, 0.065]), np.array([0.135, 0.02, 0.048]), 10, 6, 0.03, 0,
              k=85)
    b = body.verts([[-0.08, -0.022, 0.05], [-0.08, 0.022, 0.05], [-0.24, 0.042, 0.02], [-0.25, 0.0, 0.015],
                    [-0.24, -0.042, 0.02]])
    body.face([b, b + 1, b + 2, b + 3, b + 4], 0)                      # tail
    head = MB()
    _blob(head, rng, np.array([0.035, 0, 0.035]), np.array([0.05, 0.044, 0.048]), 10, 7, 0.02, 0, k=82)
    for side in (-1, 1):
        _blob(head, rng, np.array([0.06, side * 0.032, 0.048]), np.array([0.006, 0.006, 0.006]), 6, 4, 0.0, 3, k=83)
        _blob(head, rng, np.array([0.068, side * 0.029, 0.033]), np.array([0.008, 0.004, 0.006]), 6, 4, 0.0, 0,
              k=84)
    ring = [np.array([0.074, 0.017 * math.cos(a), 0.028 + 0.015 * math.sin(a)]) for a in
            np.linspace(0, 6.283, 8, endpoint=False)]
    bb = head.verts(np.vstack([ring, [0.105, 0, 0.005]]))
    for j in range(8):
        head.face([bb + j, bb + (j + 1) % 8, bb + 8], 2)               # hooked dark beak
    crest = MB()
    for k in range(7):
        a = (k - 3) * 0.09
        L = 0.055 + 0.03 * (1 - abs(k - 3) / 3)
        p0 = np.array([0.0, a * 0.1, 0.0])
        _ribbon(crest, p0, p0 + np.array([-0.85 * L, a * 0.2, 0.32 * L]), 0.012, np.array([0, 1.0, 0.25]),
                rng.random(), 1)
    legs = []
    for side in (-1, 1):
        lg = MB()
        _cyl(lg, np.array([0.0, 0, 0.0]), np.array([0.01, 0, -0.12]), 0.007, 0.006, rng, 4, nring=5, nseg=1)
        for a in (-0.5, 0.0, 0.5, 3.1):
            _cyl(lg, np.array([0.01, 0, -0.118]), np.array([0.01 + 0.03 * math.cos(a), 0.03 * math.sin(a), -0.12]),
                 0.003, 0.002, rng, 4, nring=4, nseg=1)
        legs.append(lg)
    wings = []
    for side in (1, -1):
        for part, (s0, s1) in (("in", (0.0, 0.22)), ("out", (0.0, 0.27))):
            w = MB()
            ns, nc = 4, 3
            P = []
            for i in range(ns + 1):
                s = i / ns
                if part == "in":
                    le, te = 0.05 - 0.01 * s, -0.15 + 0.01 * s
                else:
                    le, te = 0.04 - 0.035 * s, -0.14 + 0.04 * s + 0.07 * s ** 4
                for j in range(nc + 1):
                    c = j / nc
                    P.append([le + (te - le) * c, side * (s0 + (s1 - s0) * s), 0.01 * math.sin(math.pi * c) * (1 - s)])
            bw = w.verts(np.array(P))
            for i in range(ns):
                for j in range(nc):
                    f = [bw + i * (nc + 1) + j, bw + i * (nc + 1) + j + 1, bw + (i + 1) * (nc + 1) + j + 1,
                         bw + (i + 1) * (nc + 1) + j]
                    w.face(f if side > 0 else f[::-1], 0)
            if part == "out":
                for k in range(5):
                    x0 = 0.02 - 0.03 * k
                    p0 = np.array([x0, side * 0.27, 0.0])
                    _ribbon(w, p0, p0 + np.array([-0.02 * k, side * 0.07, 0.0]), 0.025, np.array([1.0, 0, 0.0]), 0.5, 0)
            wings.append(w)
    m = (mats["feather"], mats["crest"], mats["beak"], mats["eye"], mats["feet"])
    names = ("body", "head", "crest", "leg_l", "leg_r", "wing_l_in", "wing_l_out", "wing_r_in", "wing_r_out")
    meshes = [body, head, crest, legs[1], legs[0]] + wings
    return {n: mm.build("env:cockatoo_" + n, m, smooth=True) for n, mm in zip(names, meshes)}


def _bird_rig(coll, me):
    def obj(name, mesh, parent, loc):
        ob = bpy.data.objects.new(name, mesh)
        if parent is not None:
            ob.parent = parent
        ob.location = loc
        ob.rotation_mode = "XYZ"
        coll.objects.link(ob)
        return ob
    root = obj("env:cockatoo", None, None, (0, 0, 0))
    root.rotation_mode = "ZYX"
    root.empty_display_size = 0.25
    body = obj("env:cockatoo_body", me["body"], root, (0, 0, 0.125))     # hips 12.5 cm above the feet
    parts = dict(root=root, body=body)
    parts["head"] = obj("env:cockatoo_head", me["head"], body, (0.11, 0, 0.1))
    parts["crest"] = obj("env:cockatoo_crest", me["crest"], parts["head"], (0.03, 0, 0.08))
    parts["leg_l"] = obj("env:cockatoo_leg_l", me["leg_l"], body, (0.0, 0.025, 0.0))
    parts["leg_r"] = obj("env:cockatoo_leg_r", me["leg_r"], body, (0.0, -0.025, 0.0))
    for side, sy in (("l", 0.045), ("r", -0.045)):
        win = obj("env:cockatoo_wing_%s_in" % side, me["wing_%s_in" % side], body, (0.02, sy, 0.09))
        parts["wing_%s_in" % side] = win
        parts["wing_%s_out" % side] = obj("env:cockatoo_wing_%s_out" % side, me["wing_%s_out" % side], win,
                                          (0.0, 0.22 if side == "l" else -0.22, 0.0))
    return parts


def _smooth_pulse(t, t0, up, hold, down):
    """0 -> 1 -> 0 envelope: rise over `up` s from t0, hold, fall over `down` s (smoothstep edges)."""
    a = _sstep(t0, t0 + up, t)
    b = 1.0 - _sstep(t0 + up + hold, t0 + up + hold + down, t)
    return a * b


def add_cockatoos(scene, fire_frame, frame_start=None, frame_end=None, start_xy=(6.0, 24.6), seed=0):
    """One sulphur-crested cockatoo in the background near the fence (about 35 m from cam:A).

    Before the fire it forages: short walking bouts (alternating leg swing, body bob and waddle, head nodding with
    each step), pecks at the grass (body tips, head dips, quick double pecks), pauses to look round with the crest
    flicking up now and then. At the fire event it startles (crest up, head up), crouches and flies off: a few
    hard, fast wingbeats with the wrists folding on the upstroke, legs tucked, climbing away over the treeline.
    Entry point kept from v2: add_cockatoos(scene, fire_frame). Returns [root empty]."""
    fps = scene.render.fps / scene.render.fps_base
    f0 = scene.frame_start if frame_start is None else frame_start
    f1 = scene.frame_end if frame_end is None else frame_end
    env = _collection(scene, "ENV")
    for ob in [o for o in env.objects if o.name.startswith("env:cockatoo")]:
        bpy.data.objects.remove(ob, do_unlink=True)
    mats = dict(feather=_mat_feather(), crest=_mat_plain("env:cockatoo_crest", (0.80, 0.55, 0.03), 0.5, 0.3, 0.2),
                beak=_mat_plain("env:cockatoo_beak", (0.035, 0.035, 0.04), 0.35, 0.5),
                eye=_mat_plain("env:cockatoo_eye", (0.01, 0.01, 0.01), 0.2, 0.6),
                feet=_mat_plain("env:cockatoo_feet", (0.12, 0.11, 0.11), 0.6, 0.3))
    P = _bird_rig(env, _bird_parts(mats))
    rng = np.random.default_rng(SEED + 41 + seed)
    t_fire = (fire_frame - f0) / fps
    t_go = t_fire + 0.25                                # startle reaction
    t_to = t_go + 0.18                                  # leaves the ground
    frames = np.arange(f0, f1 + 1)
    t = (frames - f0) / fps
    n = len(t)
    # --- foraging schedule: alternate walk bouts (1 to 3 s) and peck / look bouts (1.5 to 4 s)
    walk = np.zeros(n)
    peck = np.zeros(n)
    look = np.zeros(n)
    crest_up = np.zeros(n)
    tt = 0.0
    while tt < t_go:
        dw = rng.uniform(0.8, 2.6)
        walk = np.maximum(walk, _smooth_pulse(t, tt, 0.15, dw, 0.15))
        tt += dw + 0.3
        dp = rng.uniform(1.5, 4.0)
        for k in range(int(dp / 0.7)):                  # pecks: fast dip, short hold, rise
            tp = tt + k * rng.uniform(0.55, 0.85)
            peck = np.maximum(peck, _smooth_pulse(t, tp, 0.09, 0.08, 0.16))
            if rng.random() < 0.4:                      # double peck
                peck = np.maximum(peck, _smooth_pulse(t, tp + 0.36, 0.07, 0.05, 0.14))
        tt += dp
        if rng.random() < 0.6:
            look = np.maximum(look, _smooth_pulse(t, tt, 0.2, rng.uniform(0.6, 1.4), 0.25))
            if rng.random() < 0.6:
                crest_up = np.maximum(crest_up, _smooth_pulse(t, tt + 0.1, 0.12, rng.uniform(0.4, 1.0), 0.4))
            tt += 1.2
    pre = t < t_go
    walk *= pre
    peck *= pre
    look *= pre
    # --- walking: steps at 2.6 Hz, 5 cm stride, heading wanders and turns between bouts
    step_hz = 2.6
    phase = np.cumsum(walk * step_hz / fps) * 2 * math.pi
    heading = rng.uniform(-math.pi, math.pi) + np.cumsum(rng.normal(0, 0.012, n) + walk * 0.006 * np.sin(t * 0.7))
    speed = walk * 0.13                                  # m/s
    x = start_xy[0] + np.cumsum(speed * np.cos(heading)) / fps
    y = start_xy[1] + np.cumsum(speed * np.sin(heading)) / fps
    y = np.clip(y, start_xy[1] - 1.5, FENCE_Y - 0.6)
    z = terrain_height(x, y)
    bob = 0.006 * walk * np.abs(np.sin(phase))
    waddle = 0.07 * walk * np.sin(phase)
    leg_l = 0.45 * walk * np.sin(phase)
    leg_r = -leg_l
    # --- body pitch (nose up stance, tips down to peck) and head (nods with steps, dips to peck, looks round)
    body_pitch = -0.32 + 0.55 * peck - 0.12 * look
    head_pitch = 0.15 + 0.9 * peck - 0.25 * look + 0.12 * walk * np.sin(2 * phase)
    head_yaw = 0.6 * look * np.sin(t * 2.3 + 1.0)
    crest = 1.3 * crest_up
    # --- startle and take-off
    st = _sstep(t_fire + 0.12, t_go, t) * (t < t_to + 0.4)
    crest = np.maximum(crest, 1.35 * st)
    head_pitch = head_pitch * (1 - st) - 0.3 * st
    crouch = 0.025 * _smooth_pulse(t, t_go, 0.1, 0.05, 0.12)
    fly = t >= t_to
    tf = np.clip(t - t_to, 0, None)
    away = np.array([0.35, 1.0]) / np.hypot(0.35, 1.0)
    vel = 1.0 + 8.5 * (1 - np.exp(-tf / 0.9))
    dist = np.cumsum(vel * fly) / fps
    climb = (0.5 * (1 - np.exp(-tf / 0.2)) + 3.6 * tf) * fly
    x0f, y0f = x[~fly][-1] if (~fly).any() else x[0], y[~fly][-1] if (~fly).any() else y[0]
    zf0 = z[~fly][-1] if (~fly).any() else z[0]
    x = np.where(fly, x0f + away[0] * dist, x)
    y = np.where(fly, y0f + away[1] * dist, y)
    z = np.where(fly, zf0 + climb, z - crouch)
    heading = np.where(fly, math.atan2(away[1], away[0]), heading)
    turn_in = _sstep(t_go, t_to + 0.25, t)              # swing round to the flight direction
    heading = np.where(pre, heading, heading[pre][-1] * (1 - turn_in) + math.atan2(away[1], away[0]) * turn_in
                       if pre.any() else heading)
    body_pitch = np.where(fly, -0.55 * np.exp(-tf / 0.4) + 0.05, body_pitch)
    leg_tuck = 1.3 * _sstep(t_to, t_to + 0.35, t)
    # wingbeats: 7 Hz at take-off easing to 4.5 Hz, downstroke 55 %, wrists fold on the upstroke; glides later
    f_hz = 7.0 - 2.5 * _sstep(0.0, 2.0, tf)
    wph = np.cumsum(f_hz * fly / fps) * 2 * math.pi
    down = np.sin(wph - 0.35 * np.sin(wph))            # skewed: quicker downstroke
    amp = 1.05 - 0.25 * _sstep(0.0, 2.0, tf)
    glide = (np.sin(tf * 0.9) > 0.8) & (tf > 2.5)
    w_in = np.where(fly, np.where(glide, 0.1, amp * down + 0.15), -0.1)
    w_out = np.where(fly, np.where(glide, 0.05, 0.55 * np.clip(-np.cos(wph), 0, 1) * amp - 0.1 * down), 0.0)
    # opening wings during the crouch
    pre_open = _sstep(t_go, t_to, t) * ~fly
    w_in = np.where(pre_open > 0, 0.9 * pre_open, w_in)
    # --- keys
    root, body = P["root"], P["body"]
    for i, v in enumerate((x, y, z)):
        _key_curve(root, "location", i, frames, v)
    _key_curve(root, "rotation_euler", 2, frames, np.unwrap(heading))
    _key_curve(body, "location", 2, frames, 0.125 + bob)
    _key_curve(body, "rotation_euler", 1, frames, body_pitch)
    _key_curve(body, "rotation_euler", 0, frames, waddle)
    _key_curve(P["head"], "rotation_euler", 1, frames, head_pitch)
    _key_curve(P["head"], "rotation_euler", 2, frames, head_yaw)
    _key_curve(P["crest"], "rotation_euler", 1, frames, crest)
    _key_curve(P["leg_l"], "rotation_euler", 1, frames, leg_l + leg_tuck)
    _key_curve(P["leg_r"], "rotation_euler", 1, frames, leg_r + leg_tuck)
    for side, sg in (("l", 1.0), ("r", -1.0)):
        _key_curve(P["wing_%s_in" % side], "rotation_euler", 0, frames, sg * w_in)
        _key_curve(P["wing_%s_out" % side], "rotation_euler", 0, frames, sg * w_out)
    vis_f = [f0, int(fire_frame + (t_go - t_fire) * fps)]
    for side in ("l", "r"):
        for part in ("in", "out"):
            ob = P["wing_%s_%s" % (side, part)]
            _key_curve(ob, "hide_render", 0, vis_f, [1.0, 0.0], "CONSTANT")
            _key_curve(ob, "hide_viewport", 0, vis_f, [1.0, 0.0], "CONSTANT")
    return [root]


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
                galv=_mat_metal("env:galvanised", (0.42, 0.43, 0.43), 0.62, 0.7, spangle=True),
                wire=_mat_metal("env:wire", (0.5, 0.5, 0.5), 0.55, 0.8),
                picket=_mat_plain("env:picket_black", (0.025, 0.025, 0.025), 0.5, 0.5),
                cap=_mat_plain("env:tread_in_post", (0.72, 0.72, 0.69), 0.45, 0.4),
                tape=_mat_plain("env:sighter_tape", (0.74, 0.74, 0.70), 0.6, 0.3, trans=0.25),
                roof=_mat_metal("env:roof_steel", (0.10, 0.10, 0.10), 0.65, 0.6), sandstone=_mat_sandstone(),
                sack=_mat_sack(), endgrain=_mat_endgrain())
    mats["leaf_gum"] = _mat_leafy("env:leaf_gum", [(0.0, (0.045, 0.075, 0.035)), (0.5, (0.065, 0.095, 0.045)),
                                                   (0.85, (0.085, 0.11, 0.05)), (1.0, (0.16, 0.13, 0.06))],
                                  rough=0.45, trans=0.25, spec=0.45, attr="lcol")
    mats["leaf_bush"] = _mat_leafy("env:leaf_bush", [(0.0, (0.03, 0.055, 0.022)), (1.0, (0.055, 0.085, 0.035))],
                                   rough=0.45, trans=0.2, spec=0.4, attr="lcol")
    mats["leaf_pine"] = _mat_leafy("env:leaf_pine", [(0.0, (0.025, 0.05, 0.022)), (1.0, (0.04, 0.07, 0.032))],
                                   rough=0.55, trans=0.15, spec=0.3)
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
    fmats = (mats["timber"], mats["galv"], mats["wire"], mats["picket"], mats["cap"], mats["tape"], mats["roof"])
    mb = MB()
    _rail_fence(mb, rng, (FENCE_X[0], FENCE_Y), (FENCE_X[1], FENCE_Y),
                gate=(GATE_X[0] - FENCE_X[0], GATE_X[1] - FENCE_X[0]))
    _mesh_gate(mb, rng, (GATE_X[0], FENCE_Y), (GATE_X[1], FENCE_Y))
    props.append(_obj("env:fence_rails", mb.build("env:fence_rails", fmats, smooth=True), env))
    mb = MB()
    _tape_line(mb, rng, TAPE_Y, FENCE_X[0] + 2.0, FENCE_X[1] - 2.0)
    props.append(_obj("env:fence_tape", mb.build("env:fence_tape", fmats, smooth=True), env))
    mb = MB()
    _shelter(mb, rng, SHELTER)
    props.append(_obj("env:shelter", mb.build("env:shelter", fmats), env))
    mb = MB()
    sx, sy = STUMP["xy"]
    z0 = terrain_height(sx, sy)
    _cyl(mb, (sx, sy, z0 - 0.15), (sx, sy, z0 + STUMP["h"]), STUMP["dia"] / 2, STUMP["dia"] / 2 * 0.9, rng, 0,
         cap_mat=1, tex=TEX_SIZE["bark_bluegum"], nring=24, bumps=0.08, k=80, flare=0.3, caps=(False, True))
    props.append(_obj("env:stump", mb.build("env:stump", (mats["deadwood"], mats["endgrain"]), smooth=True), env))
    for i, (bx, by, br) in enumerate(BOULDERS):
        mb = MB()
        _boulder(mb, rng, (bx, by), br, 90 + i)
        props.append(_obj("env:boulder_%d" % (i + 1), mb.build("env:boulder_%d" % (i + 1), (mats["sandstone"],),
                                                                smooth=True), env))
    mb = MB()
    _sacks(mb, rng, BAGS)
    props.append(_obj("env:sacks", mb.build("env:sacks", (mats["sack"],), smooth=True), env))
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


PREVIEW_CAMS = {          # about where build.py's two locked-off tripod cameras sit (SPEC v2), and Harry's photos
    "": dict(loc=(2.4, -10.6, 1.55), target=(2.6, 1.0, 0.35), lens=40.0),
    "_b": dict(loc=(2.3, -4.1, 0.85), target=(2.4, 0.6, 0.2), lens=24.0),
    "_photo": dict(loc=(4.0, -8.5, 1.6), target=(4.0, 28.0, 4.2), lens=26.0, res=(960, 720)),
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
    names = {"A": "", "B": "_b", "photo": "_photo"}
    for v in a.views.split(","):
        key = names[v]
        c = PREVIEW_CAMS[key]
        if not a.uhd_crop:
            scene.render.resolution_x, scene.render.resolution_y = c.get("res", (960, 540))
        cam = _add_cam(scene, "cam:env_preview%s" % key, c["loc"], c["target"], c["lens"])
        scene.camera = cam
        scene.render.filepath = os.path.join(OUT, "preview_env%s%s.png" % (key, a.suffix))
        t1 = time.time()
        bpy.ops.render.render(write_still=True)
        print("rendered %s in %.1fs, peak RSS %.0f MB" % (scene.render.filepath, time.time() - t1, _peak_rss_mb()))
    if a.save:
        bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(a.save))
    print("total %.1fs, peak RSS %.0f MB" % (time.time() - t0, _peak_rss_mb()))


