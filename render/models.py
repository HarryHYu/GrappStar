"""GrappStar render: MODELS module (see render/SPEC.md).

    import models
    roots = models.build_models(scene)      # base:root, dart:root, elec:root (+ board) with materials, rest pose

    render/.venv/bin/python render/models.py            # builds the models with studio lighting and renders
                                                        # render/out/preview_models_{base,dart,elec}.png

What is built (all metres, Z up, world origin = centre of the plywood board top, launch toward +X):
- BASE (collection BASE): every part of the full D50 build base/out/base/asm.json as its own mesh object
  `base:<part>` (mesh in the part's local frame, the CAD npz meshes, mm -> m), parented to the empty `base:root`
  whose matrix is BASE_TO_WORLD. Each part object's local matrix is its asm world transform at the rest state
  (timeline state_at(P, 0): LAUNCH, rail at the 6 deg low stop, wiper 0) = JW(joint) @ place, computed here from
  asm.json "joints" with BASE_REST_JOINTS. So a motion world transform for a part is simply
  BASE_TO_WORLD @ asm.world(S)[part] (translation mm -> m), set as matrix_world. The old dart (launch_dart) is not
  built (the v34 dart replaces it). The five coil segments per launch spring (launch_spring_{l,r}_{0..4}) are
  replaced by one stretchable coil object per spring, `base:launch_spring_l` / `base:launch_spring_r` (geometry
  nodes; see SPRINGS, set_spring(), key_springs()); the hooks keep their asm names (procedural wire loops).
  Bought envelopes are replaced by detailed procedural models in the same local frame: the bay boards
  (frame_pcb_esp32, _bts7960, _drv8833, _mp1584, _hx711, frame_perfboard, frame_fuse_20a), frame_rc832 (with its
  antenna and AV lead), frame_kiyo_x; the wiper motor and the N20s keep their CAD envelopes with split materials.
- the plywood board `env:board` is built here too (birch ply, 1.0 x 0.8 x 0.018 m, top at z 0), with cable ties.
- DART (collection DART): every kedge/out34 part as `dart:<part>` in the dart frame (nose tip origin, +x to the
  tail, +z up) under the empty `dart:root`. The moving parts (righting leg, both drill pods, both augers) are
  their own pivots: origin on the joint axis, everything riding on them parented below (DART_JOINTS, the same
  names / parents / pivots / axes as motion.json "joints"); pose_dart() / apply_dart_joint() set them. At rest
  dart:root sits folded on the launcher cradle (asm launch_dart transform).
- materials: PBR detail on everything (_detail: roughness variation, CC0 roughness / height maps box-projected,
  fingerprints, cavity grime, worn edges, dust, micro bump); mat_rope(colour) for the Dyneema tether.
- ELEC (collection ELEC): Harry's kit on the board beside the base: `elec:jetson` (Jetson Orin Nano developer kit with its
  heatsink and fan), `elec:digitnow` (USB capture dongle on a Jetson USB port), `elec:lipo_3s` (3S 2200 with XT60
  and balance lead), XT60 Y harness, and the cables (`elec:cable_*`) to the base's electronics bay, RC832, Kiyo X.
  No openly licensed login-free 3D model of these was found (render/assets/CREDITS.md), so all are procedural
  from datasheet dimensions and Harry's caliper photos (originals/measurements.pdf).
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

import numpy as np

import bpy
import bmesh
from mathutils import Matrix, Vector, Euler

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
BASE_DIR = os.path.join(REPO, "base", "out", "base")
DART_DIR = os.path.join(REPO, "kedge", "out34")
MOTION_JSON = os.path.join(HERE, "motion", "motion.json")
OUT_DIR = os.path.join(HERE, "out")
MM = 0.001

# ---------------------------------------------------------------------------------------------- frames
# base frame (base/spec/BASE_SPEC.md section 2): origin on the wiper shaft axis, +X launch azimuth, deck top z -165,
# deck underside z -173. BASE_TO_WORLD puts the deck underside on the board top (world z 0) with the deck footprint
# (x -410.5..269.4, y -137.8..234.8 mm incl. the D50 extension tile) centred on the world origin (the same matrix
# render/motion/base_motion.py uses). If render/motion/motion.json carries "base_to_world", that one wins.
BASE_TO_WORLD_DEFAULT = [[1.0, 0.0, 0.0, 0.0705],
                         [0.0, 1.0, 0.0, -0.0485],
                         [0.0, 0.0, 1.0, 0.173],
                         [0.0, 0.0, 0.0, 1.0]]


def _load_base_to_world():
    try:
        with open(MOTION_JSON) as fh:
            M = np.asarray(json.load(fh)["base_to_world"], float).reshape(4, 4)
        if np.abs(M[:3, 3]).max() > 5.0:            # given in mm
            M[:3, 3] *= MM
        return M.tolist()
    except Exception:
        return [list(r) for r in BASE_TO_WORLD_DEFAULT]


BASE_TO_WORLD = _load_base_to_world()

BOARD_SIZE = (1.00, 0.80, 0.018)          # plywood board (x, y, thickness), top face at z 0, centred on the origin

# rest state of the base: tools.timeline.state_at(P, 0.0) = load phase: t 0, wiper 0, drum LAUNCH, aim 6 deg, cock 0.
# Joint values of every asm.json joint at that state (Assembly.values(S0), computed with the base tools 11 Nov 2026;
# every joint not listed is 0). JW(joint) @ place reproduces Assembly.world(S0) for all 271 parts to < 0.01 mm.
BASE_REST_JOINTS = {
    "drum_collar_slide": 7.0, "drum_fork": 7.0, "drum_idler": -1376.76, "drum_n20": -322.0, "drum_sw0": 4.775,
    "launch_rail": 6.024, "launch_big_gear": -2.0, "launch_drive": 3.976, "launch_sw_seated_lever": 3.77,
    "launch_sw_home_lever": 3.77, "launch_aim_crank": -243.3, "launch_aim_worm": 4866.0, "launch_aim_cshaft": 2433.0,
    "launch_aim_conrod": 179.96, "launch_aim_arm": 96.213, "launch_aim_idler": 350.48, "reel_bus": 3.78,
    "reel_idler": 447.541, "reel_shaft": -1172.34,
}

# ---------------------------------------------------------------------------------------------- dart joints
# The moving parts are their own pivots: each moving object sits with its origin ON its joint axis (identity rest
# rotation) under the parent named here, and everything that moves with it hangs below it. This is exactly the
# form render/motion/motion.json "joints" uses (object name, parent, pivot in the parent frame, axis, deg): set the
# object's location = pivot and rotate it by deg about axis (pose_dart() / apply_dart_joint() do that). Axes
# include parts.json "sgn" (-1), so +deg is the physical direction: leg 0 folded .. 178 righting, pods 0 folded ..
# 46.75 deployed (parts.json DEPLOY_DEG); augers: right-hand about the axis toward the tip (L auger right-hand, +deg
# drills in; R auger left-hand, -deg drills in: drill_in_sign).
DART_JOINTS = {
    "dart:righting_leg": dict(joint="leg", parent="dart:root", pivot=(0.033, 0.0, 0.0145), axis=(0.0, -1.0, 0.0),
                              range_deg=(0.0, 178.0),
                              carries=["dart:leg_coupler_L", "dart:leg_coupler_R", "dart:leg_shaft"]),
    "dart:drill_leg_L": dict(joint="deploy", parent="dart:root", pivot=(0.132, 0.03115, 0.0095),
                             axis=(0.0, -1.0, 0.0), range_deg=(0.0, 46.75),
                             carries=["dart:drill_leg_L_motor", "dart:coupler_L", "dart:pod_cap_L",
                                      "dart:coupler_L_screw", "dart:deploy_shaft", "dart:drill_leg_L_auger"]),
    "dart:drill_leg_R": dict(joint="deploy", parent="dart:root", pivot=(0.132, -0.03115, 0.0095),
                             axis=(0.0, -1.0, 0.0), range_deg=(0.0, 46.75),
                             carries=["dart:drill_leg_R_motor", "dart:coupler_R", "dart:pod_cap_R",
                                      "dart:coupler_R_screw", "dart:drill_leg_R_auger"]),
    "dart:drill_leg_L_auger": dict(joint="auger_L", parent="dart:drill_leg_L", pivot=(0.0, 0.0, 0.0),
                                   axis=(-1.0, 0.0, 0.0), drill_in_sign=1,
                                   carries=["dart:drill_leg_L_shaft", "dart:drill_leg_L_nut"]),
    "dart:drill_leg_R_auger": dict(joint="auger_R", parent="dart:drill_leg_R", pivot=(0.0, 0.0, 0.0),
                                   axis=(-1.0, 0.0, 0.0), drill_in_sign=-1,
                                   carries=["dart:drill_leg_R_shaft", "dart:drill_leg_R_nut"]),
}
DART_JOINT_ALIASES = {"dart:pivot_leg": "dart:righting_leg", "dart:pivot_drill_L": "dart:drill_leg_L",
                      "dart:pivot_drill_R": "dart:drill_leg_R", "dart:pivot_auger_L": "dart:drill_leg_L_auger",
                      "dart:pivot_auger_R": "dart:drill_leg_R_auger"}

# ---------------------------------------------------------------------------------------------- springs
# Century C-243 (base/spec/params.py SPRING): OD 16.0, wire 1.37 (0.054 in), free length 165.1 over the loops,
# 103 active coils (close wound at free length: (165.1 - 2 x 12) / 1.37). The coil body runs between the two hook
# objects' coil ends (12 mm inside each pin centre, launch.HOOK_L); length is a geometry-nodes input, so the wire
# keeps its round section however far it is stretched.
SPRINGS = {
    "base:launch_spring_l": dict(hook_c="base:launch_spring_l_hook_c", hook_a="base:launch_spring_l_hook_a"),
    "base:launch_spring_r": dict(hook_c="base:launch_spring_r_hook_c", hook_a="base:launch_spring_r_hook_a"),
}
SPRING_SPEC = dict(od=16.0 * MM, wire=1.37 * MM, coils=103, hook_l=12.0 * MM, pin_d=8.0 * MM, res=12)

COLLECTIONS = ("BASE", "DART", "ELEC", "ENV")


# ============================================================================================== small helpers
def srgb(hexcol, alpha=1.0):
    """'#rrggbb' or '#rgb' -> linear RGBA"""
    h = hexcol.lstrip("#")
    if len(h) == 3:
        h = "".join(c * 2 for c in h)
    c = [int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]
    lin = [x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c]
    return (lin[0], lin[1], lin[2], alpha)


def get_collection(name, scene=None):
    scene = scene or bpy.context.scene
    col = bpy.data.collections.get(name)
    if col is None:
        col = bpy.data.collections.new(name)
    if col.name not in scene.collection.children:
        scene.collection.children.link(col)
    return col


def link(obj, col):
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    col.objects.link(obj)
    return obj


def empty(name, col, parent=None, matrix=None, size=0.02):
    ob = bpy.data.objects.get(name)
    if ob is None:
        ob = bpy.data.objects.new(name, None)
    ob.empty_display_type = "ARROWS"
    ob.empty_display_size = size
    link(ob, col)
    ob.parent = parent
    ob.matrix_parent_inverse = Matrix.Identity(4)
    if matrix is not None:
        ob.matrix_basis = Matrix(matrix)
    return ob


def mesh_np(name, v, f, mat_idx=None, smooth_angle=30.0):
    """Mesh datablock from numpy verts (m) and faces (triangles (n, 3) or a list of index lists)."""
    me = bpy.data.meshes.new(name)
    v = np.asarray(v, np.float32).reshape(-1, 3)
    if isinstance(f, np.ndarray) and f.ndim == 2:
        sizes = np.full(len(f), f.shape[1], np.int32)
        loops = f.astype(np.int32).ravel()
    else:
        sizes = np.array([len(p) for p in f], np.int32)
        loops = np.fromiter((i for p in f for i in p), np.int32, count=int(sizes.sum()))
    me.vertices.add(len(v))
    me.vertices.foreach_set("co", v.ravel())
    me.loops.add(len(loops))
    me.loops.foreach_set("vertex_index", loops)
    me.polygons.add(len(sizes))
    starts = np.zeros(len(sizes), np.int32)
    if len(sizes) > 1:
        starts[1:] = np.cumsum(sizes)[:-1]
    me.polygons.foreach_set("loop_start", starts)
    me.update(calc_edges=True)
    if mat_idx is not None:
        me.polygons.foreach_set("material_index", np.asarray(mat_idx, np.int32))
    if smooth_angle:
        me.shade_smooth()
        try:
            me.set_sharp_from_angle(angle=math.radians(smooth_angle))
        except Exception:
            pass
    return me


def mesh_object(name, me, col, parent=None, matrix=None, mats=()):
    ob = bpy.data.objects.get(name)
    if ob is not None:
        bpy.data.objects.remove(ob, do_unlink=True)
    ob = bpy.data.objects.new(name, me)
    col.objects.link(ob)
    for m in mats:
        me.materials.append(m)
    ob.parent = parent
    ob.matrix_parent_inverse = Matrix.Identity(4)
    if matrix is not None:
        ob.matrix_basis = Matrix(matrix)
    return ob


def set_attr(me, name, values):
    a = me.attributes.get(name) or me.attributes.new(name, "FLOAT", "POINT")
    a.data.foreach_set("value", np.asarray(values, np.float32))


def mm_matrix(M):
    """4x4 in mm -> 4x4 in m (translation scaled)"""
    M = np.array(M, float)
    M[:3, 3] *= MM
    return M


# ============================================================================================== materials
_MATS = {}


def _new_mat(key):
    m = bpy.data.materials.get(key)
    if m is not None:
        bpy.data.materials.remove(m)
    m = bpy.data.materials.new(key)
    if bpy.app.version < (5, 0, 0):
        m.use_nodes = True
    nt = m.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    out.location = (600, 0)
    bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.location = (300, 0)
    nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    _MATS[key] = m
    return m, nt, bsdf


class _N:
    """terse node builder for a material node tree"""

    def __init__(self, nt):
        self.nt = nt
        self._obj = None
        self._geo = None

    def node(self, kind, **props):
        n = self.nt.nodes.new(kind)
        for k, v in props.items():
            setattr(n, k, v)
        return n

    def link(self, a, b):
        self.nt.links.new(a, b)

    def _in(self, sock, x):
        if isinstance(x, (int, float, tuple, list)):
            sock.default_value = x
        else:
            self.link(x, sock)

    def math(self, op, a, b=None, clamp=False):
        n = self.node("ShaderNodeMath", operation=op, use_clamp=clamp)
        for i, x in enumerate((a, b)):
            if x is not None:
                self._in(n.inputs[i], x)
        return n.outputs[0]

    def mix(self, fac, a, b, kind="RGBA", blend="MIX"):
        n = self.node("ShaderNodeMix", data_type=kind)
        if kind == "RGBA":
            n.blend_type = blend
        ins = {"RGBA": ("A_Color", "B_Color", "Result_Color"), "FLOAT": ("A_Float", "B_Float", "Result_Float")}[kind]
        fi = [s for s in n.inputs if s.identifier == "Factor_Float"][0]
        ai = [s for s in n.inputs if s.identifier == ins[0]][0]
        bi = [s for s in n.inputs if s.identifier == ins[1]][0]
        for sock, x in ((fi, fac), (ai, a), (bi, b)):
            self._in(sock, x)
        return [s for s in n.outputs if s.identifier == ins[2]][0]

    def vscale(self, vec, s):
        n = self.node("ShaderNodeVectorMath", operation="SCALE")
        self._in(n.inputs[0], vec)
        self._in(n.inputs["Scale"], s)
        return n.outputs["Vector"]

    def vmath(self, op, a, b):
        n = self.node("ShaderNodeVectorMath", operation=op)
        self._in(n.inputs[0], a)
        self._in(n.inputs[1], b)
        return n.outputs["Value"] if op in ("DOT_PRODUCT", "LENGTH", "DISTANCE") else n.outputs["Vector"]

    def rgb(self, col):
        n = self.node("ShaderNodeRGB")
        n.outputs[0].default_value = col
        return n.outputs[0]

    def noise(self, scale, detail=3.0, rough=0.55, vec=None, dim="3D"):
        n = self.node("ShaderNodeTexNoise", noise_dimensions=dim)
        n.inputs["Scale"].default_value = scale
        n.inputs["Detail"].default_value = detail
        n.inputs["Roughness"].default_value = rough
        if vec is not None:
            self.link(vec, n.inputs["Vector"])
        return n

    def coord(self, which="Object"):
        if which == "Object":
            if self._obj is None:
                self._obj = self.node("ShaderNodeTexCoord").outputs["Object"]
            return self._obj
        return self.node("ShaderNodeTexCoord").outputs[which]

    def geo(self):
        if self._geo is None:
            self._geo = self.node("ShaderNodeNewGeometry")
        return self._geo

    def attr(self, name):
        n = self.node("ShaderNodeAttribute", attribute_name=name)
        return n.outputs["Fac"]

    def ramp(self, fac, stops):
        n = self.node("ShaderNodeValToRGB")
        self.link(fac, n.inputs["Fac"])
        cr = n.color_ramp
        while len(cr.elements) > len(stops):
            cr.elements.remove(cr.elements[-1])
        while len(cr.elements) < len(stops):
            cr.elements.new(0.5)
        for el, (pos, col) in zip(cr.elements, stops):
            el.position = pos
            el.color = col
        return n.outputs["Color"]

    def maprange(self, x, a, b, c=0.0, d=1.0):
        mr = self.node("ShaderNodeMapRange", clamp=True)
        self._in(mr.inputs["Value"], x)
        mr.inputs["From Min"].default_value = a
        mr.inputs["From Max"].default_value = b
        mr.inputs["To Min"].default_value = c
        mr.inputs["To Max"].default_value = d
        return mr.outputs["Result"]

    def sep(self, vec):
        n = self.node("ShaderNodeSeparateXYZ")
        self.link(vec, n.inputs[0])
        return n.outputs

    def bump(self, height, strength, distance, normal=None):
        n = self.node("ShaderNodeBump")
        n.inputs["Strength"].default_value = strength
        n.inputs["Distance"].default_value = distance
        self.link(height, n.inputs["Height"])
        if normal is not None:
            self.link(normal, n.inputs["Normal"])
        return n.outputs["Normal"]

    def bevel(self, radius):
        n = self.node("ShaderNodeBevel")
        n.inputs["Radius"].default_value = radius
        n.samples = 4
        return n.outputs["Normal"]

    def ao(self, distance):
        n = self.node("ShaderNodeAmbientOcclusion")
        n.inputs["Distance"].default_value = distance
        n.only_local = True
        n.samples = 8
        return n.outputs["AO"]

    def img(self, image, tile_m, vec=None, blend=0.3):
        """box-projected (triplanar, no UVs needed) image texture, one tile per tile_m metres of object space"""
        n = self.node("ShaderNodeTexImage")
        n.image = image
        n.projection = "BOX"
        n.projection_blend = blend
        self.link(self.vscale(vec if vec is not None else self.coord("Object"), 1.0 / tile_m), n.inputs["Vector"])
        return n

    def fade(self, near=0.25, far=1.6):
        """1 close to the camera, 0 far (kills sub-pixel bump aliasing)"""
        cam = self.node("ShaderNodeCameraData")
        return self.maprange(cam.outputs["View Distance"], near, far, 1.0, 0.0)


def _set(bsdf, **kw):
    names = {"base": "Base Color", "metal": "Metallic", "rough": "Roughness", "ior": "IOR", "spec": "Specular IOR Level",
             "coat": "Coat Weight", "coat_rough": "Coat Roughness", "trans": "Transmission Weight",
             "aniso": "Anisotropic", "sss": "Subsurface Weight", "sss_radius": "Subsurface Radius",
             "emit": "Emission Color", "emit_s": "Emission Strength", "sheen": "Sheen Weight",
             "coat_tint": "Coat Tint", "alpha": "Alpha"}
    for k, v in kw.items():
        sock = bsdf.inputs[names[k]]
        if isinstance(v, (int, float, tuple, list)):
            sock.default_value = v
        else:
            bsdf.id_data.links.new(v, sock)


# ---------------------------------------------------------------------------------------------- texture sets
# CC0 sets (render/assets/fetch_models.py downloads them into render/assets/cache/tex_models; CREDITS.md). Every
# material falls back to procedural detail when a file is missing.
TEX_DIR = os.path.join(HERE, "assets", "cache", "tex_models")
TEXSETS = {
    "scratch": "Metal009/Metal009_1K-JPG_Roughness.jpg",            # brushed steel with scratches (roughness)
    "scratch_h": "Metal009/Metal009_1K-JPG_Displacement.jpg",
    "prints": "Fingerprints002/Fingerprints002_1K-JPG_Roughness.jpg",  # fingerprints / smudges
    "plastic": "Plastic006/Plastic006_1K-JPG_Roughness.jpg",         # moulded plastic micro texture
    "plastic_h": "Plastic006/Plastic006_1K-JPG_Displacement.jpg",
    "rubber": "Rubber004/Rubber004_1K-JPG_Roughness.jpg",
    "rubber_col": "Rubber004/Rubber004_1K-JPG_Color.jpg",
    "ply_col": "plywood/plywood_diff_2k.jpg",                          # Poly Haven plywood face
    "ply_rough": "plywood/plywood_rough_2k.jpg",
    "ply_nrm": "plywood/plywood_nor_gl_2k.jpg",
    "plyside_col": "Wood088/Wood088_1K-JPG_Color.jpg",                # ambientCG plywood edge
    "birch_col": "Wood090A/Wood090A_1K-JPG_Color.jpg",
}
_IMGS = {}


def tex(key, color=False):
    """bpy image for a texture set entry, or None if not downloaded"""
    if key in _IMGS:
        return _IMGS[key]
    path = os.path.join(TEX_DIR, TEXSETS[key])
    im = None
    if os.path.exists(path):
        im = bpy.data.images.load(path, check_existing=True)
        im.colorspace_settings.name = "sRGB" if color else "Non-Color"
    _IMGS[key] = im
    return im


def _detail(N, bsdf, col, rough, metal=0.0, var=0.05, rvar=0.12, grime=0.45, grime_dist=0.004, edge=0.3,
            edge_col=None, edge_metal=False, edge_rough=None, dust=0.2, prints=0.0, scratch=0.0, texkey=None,
            tex_tile=0.08, micro=0.12, micro_scale=2500.0, height=None, bevel=0.0004, bump_dist=0.00005):
    """Physically motivated surface detail wired into a Principled BSDF:
    colour variation, roughness variation (noise + optional CC0 roughness texture, box projected), fingerprints,
    cavity grime (AO, darker and rougher in crevices), edge wear (bevel normal vs shading normal: lighter worn
    edges, or bare metal for paint / anodising; Cycles only, EEVEE skips it), top-facing dust, micro bump."""
    obj = N.coord("Object")
    geo = N.geo()
    c = N.rgb(col) if isinstance(col, tuple) else col
    low = N.noise(18.0, 3.0, 0.55, obj).outputs["Fac"]
    c = N.mix(low, N.vscale(c, 1.0 - var), N.vscale(c, 1.0 + var))
    rn = N.noise(260.0, 4.0, 0.6, obj).outputs["Fac"]
    r = N.math("ADD", N.math("MULTIPLY", N.math("SUBTRACT", rn, 0.5), rvar), rough)
    h = N.math("MULTIPLY", N.noise(micro_scale, 2.0, 0.6, obj).outputs["Fac"], micro)
    if texkey and tex(texkey) is not None:
        t = N.img(tex(texkey), tex_tile).outputs["Color"]
        tv = N.sep(t)[0]
        r = N.math("ADD", r, N.math("MULTIPLY", N.math("SUBTRACT", tv, 0.5), scratch))
        hk = texkey + "_h"
        if hk in TEXSETS and tex(hk) is not None:
            th = N.sep(N.img(tex(hk), tex_tile).outputs["Color"])[0]
            h = N.math("ADD", h, N.math("MULTIPLY", th, 0.5))
    if prints and tex("prints") is not None:
        pv = N.sep(N.img(tex("prints"), 0.11).outputs["Color"])[0]
        pm = N.math("MULTIPLY", N.maprange(pv, 0.25, 0.8), prints)
        r = N.math("ADD", r, N.math("MULTIPLY", pm, 0.16))
        c = N.mix(N.math("MULTIPLY", pm, 0.2), c, N.vscale(c, 1.8))
    if grime:
        ao = N.ao(grime_dist)
        gn = N.noise(140.0, 3.0, 0.6, obj).outputs["Fac"]
        gm = N.math("MULTIPLY", N.math("MULTIPLY", N.math("SUBTRACT", 1.0, ao), grime),
                    N.maprange(gn, 0.3, 0.7, 0.55, 1.0))
        c = N.mix(gm, c, N.mix(0.5, N.vscale(c, 0.3), (0.035, 0.03, 0.024, 1.0)))
        r = N.math("ADD", r, N.math("MULTIPLY", gm, 0.25))
    nb = N.bevel(bevel) if bevel else None
    met = metal
    if edge and nb is not None:
        d = N.vmath("DOT_PRODUCT", nb, geo.outputs["Normal"])
        en = N.noise(90.0, 4.0, 0.65, obj).outputs["Fac"]
        em = N.math("MULTIPLY", N.math("MULTIPLY", N.maprange(N.math("SUBTRACT", 1.0, d), 0.0015, 0.05), edge),
                    N.maprange(en, 0.35, 0.65))
        if edge_col is None:
            ecol = N.mix(0.35, N.vscale(c, 1.35), (0.8, 0.8, 0.8, 1.0))
        else:
            ecol = N.rgb(edge_col) if isinstance(edge_col, tuple) else edge_col
        c = N.mix(em, c, ecol)
        if edge_metal:
            met = N.mix(em, metal, 1.0, kind="FLOAT")
        if edge_rough is not None:
            r = N.mix(em, r, edge_rough, kind="FLOAT")
    if dust:
        nz = N.sep(geo.outputs["Normal"])[2]
        dn = N.noise(45.0, 4.0, 0.65, obj).outputs["Fac"]
        dm = N.math("MULTIPLY", N.math("MULTIPLY", N.maprange(nz, 0.5, 0.95), N.maprange(dn, 0.35, 0.75)), dust)
        c = N.mix(N.math("MULTIPLY", dm, 0.3), c, (0.30, 0.27, 0.23, 1.0))
        r = N.math("ADD", r, N.math("MULTIPLY", dm, 0.35))
    if height is not None:
        h = N.math("ADD", h, height)
    nrm = N.bump(h, 1.0, bump_dist, nb)
    r = N.math("ADD", r, 0.0, clamp=True)
    _set(bsdf, base=c, rough=r, metal=met)
    N.link(nrm, bsdf.inputs["Normal"])


def mat_printed(hexcol, kind="PLA"):
    """FDM print: CAD colour, layer lines from the per-vertex attribute 'layer' (print-bed z in mm, 0.2 mm layers;
    faded with distance), satin PLA with a little subsurface softness, worn lighter edges, grime in the corners,
    dust on top faces. kind PPA-CF: matte dark carbon-filled nylon with a fine fibrous speckle (CAD colours of
    PPA-CF parts are ignored)."""
    key = f"{kind}_{hexcol.lstrip('#')}" if kind == "PLA" else "PPA-CF"
    if key in _MATS:
        return _MATS[key]
    m, nt, bsdf = _new_mat(key)
    N = _N(nt)
    obj = N.coord("Object")
    lay = N.attr("layer")
    f = N.math("FRACT", N.math("DIVIDE", lay, 0.2))
    prof = N.math("POWER", N.math("ABSOLUTE", N.math("SUBTRACT", f, 0.5)), 1.6)
    wob = N.noise(700.0, 2.0, 0.5, obj).outputs["Fac"]                               # layer-to-layer wobble
    lines = N.math("MULTIPLY", N.math("ADD", prof, N.math("MULTIPLY", wob, 0.08)), N.fade(0.15, 1.2))
    if kind == "PLA":
        col = srgb(hexcol)
        _detail(N, bsdf, col, 0.42, var=0.05, rvar=0.14, grime=0.5, edge=0.55, edge_rough=0.3, dust=0.25,
                micro=0.08, height=N.math("MULTIPLY", lines, -2.2), bump_dist=0.00012, bevel=0.0004)
        _set(bsdf, spec=0.5, sss=0.06, sss_radius=(0.0015, 0.0015, 0.0015), sheen=0.08)
    else:
        fib = N.noise(1600.0, 6.0, 0.8, obj).outputs["Fac"]
        spk = N.math("GREATER_THAN", N.noise(5200.0, 1.0, 0.5, obj).outputs["Fac"], 0.68)
        base = N.mix(spk, N.ramp(fib, [(0.35, srgb("#1a1b1d")), (0.65, srgb("#2a2b2e"))]), srgb("#3c3e42"))
        _detail(N, bsdf, base, 0.72, var=0.04, rvar=0.1, grime=0.4, edge=0.45, edge_col=srgb("#55585d"),
                edge_rough=0.55, dust=0.3, micro=0.0, height=N.math("ADD", N.math("MULTIPLY", lines, -1.4),
                                                                   N.math("MULTIPLY", fib, 0.8)),
                bump_dist=0.0001, bevel=0.0003)
        _set(bsdf, spec=0.35)
    return m


def mat_pbr(key, base, rough=0.5, metal=0.0, detail=None, **bsdf_kw):
    """principled material + _detail(); detail = dict of _detail options"""
    if key in _MATS:
        return _MATS[key]
    m, nt, bsdf = _new_mat(key)
    N = _N(nt)
    if isinstance(base, str):
        base = srgb(base)
    _set(bsdf, **bsdf_kw)
    _detail(N, bsdf, base, rough, metal, **(detail or {}))
    return m


def mat_simple(key, base, metal=0.0, rough=0.5, spec=0.5, noise=0.0, noise_scale=400.0, coat=0.0, aniso=0.0,
               bevel=0.0, plain=False, **kw):
    """generic material with the standard plastic-like detail (plain=True: no detail, e.g. LEDs, holes)"""
    if key in _MATS:
        return _MATS[key]
    m, nt, bsdf = _new_mat(key)
    N = _N(nt)
    if isinstance(base, str):
        base = srgb(base)
    _set(bsdf, spec=spec, coat=coat, aniso=aniso, **kw)
    if plain or kw.get("emit_s"):
        _set(bsdf, base=base, metal=metal, rough=rough)
        return m
    _detail(N, bsdf, base, rough, metal, rvar=max(noise, 0.08), grime=0.4, edge=0.25, dust=0.18,
            bevel=bevel or 0.0003)
    return m


_STEEL_D = dict(texkey="scratch", scratch=0.35, tex_tile=0.06, grime=0.4, edge=0.25, edge_col=(0.8, 0.8, 0.82, 1),
                dust=0.12, prints=0.15, micro=0.05, rvar=0.08)


def M(name):
    """named stock materials"""
    if name in _MATS:
        return _MATS[name]
    t = {
        "steel": lambda: mat_pbr("steel", (0.60, 0.60, 0.62, 1), 0.28, 1.0, _STEEL_D, aniso=0.45),
        "steel_bright": lambda: mat_pbr("steel_bright", (0.74, 0.74, 0.76, 1), 0.18, 1.0,
                                        dict(_STEEL_D, scratch=0.1, tex_tile=0.03, prints=0.15, dust=0.04)),
        "spring_steel": lambda: mat_pbr("spring_steel", (0.56, 0.57, 0.59, 1), 0.26, 1.0,
                                        dict(_STEEL_D, scratch=0.25, edge=0.0, prints=0.0, grime=0.5), aniso=0.3),
        "zinc": lambda: _mat_zinc(),
        "black_oxide": lambda: mat_pbr("black_oxide", (0.045, 0.045, 0.05, 1), 0.42, 0.85,
                                       dict(_STEEL_D, edge=0.6, edge_col=(0.45, 0.45, 0.47, 1), edge_metal=True,
                                            edge_rough=0.25)),
        "brass": lambda: mat_pbr("brass", (0.80, 0.56, 0.26, 1), 0.28, 1.0,
                                 dict(_STEEL_D, grime=0.6, edge_col=(0.95, 0.75, 0.42, 1))),
        "copper": lambda: mat_pbr("copper", (0.93, 0.52, 0.36, 1), 0.3, 1.0, dict(_STEEL_D, grime=0.6)),
        "gold": lambda: mat_pbr("gold", (1.0, 0.74, 0.32, 1), 0.22, 1.0,
                                dict(rvar=0.08, grime=0.3, edge=0.0, dust=0.05, micro=0.05, bevel=0.0001)),
        "tin": lambda: mat_pbr("tin", (0.76, 0.76, 0.78, 1), 0.32, 1.0,
                               dict(rvar=0.15, grime=0.3, edge=0.0, dust=0.05, micro=0.15, bevel=0.0001)),
        "alu": lambda: mat_pbr("alu", (0.80, 0.81, 0.83, 1), 0.3, 1.0,
                               dict(_STEEL_D, scratch=0.3, tex_tile=0.05), aniso=0.6),
        "alu_cast": lambda: mat_pbr("alu_cast", (0.34, 0.35, 0.36, 1), 0.5, 1.0,
                                    dict(var=0.12, rvar=0.2, grime=0.7, grime_dist=0.008, edge=0.5,
                                         edge_col=(0.62, 0.63, 0.65, 1), edge_rough=0.3, dust=0.4, micro=0.6,
                                         micro_scale=700.0, bump_dist=0.0002, bevel=0.0008)),
        "alu_black": lambda: mat_pbr("alu_black", (0.016, 0.016, 0.018, 1), 0.5, 0.5,
                                     dict(var=0.06, rvar=0.1, grime=0.4, edge=0.35, edge_col=(0.55, 0.56, 0.58, 1),
                                          edge_metal=True, edge_rough=0.25, dust=0.06, prints=0.2, micro=0.08,
                                          bevel=0.0003)),
        "paint_black": lambda: mat_pbr("paint_black", (0.018, 0.018, 0.02, 1), 0.34, 0.15,
                                       dict(var=0.06, rvar=0.12, grime=0.6, grime_dist=0.008, edge=0.6,
                                            edge_col=(0.42, 0.42, 0.44, 1), edge_metal=True, edge_rough=0.3,
                                            dust=0.45, prints=0.4, micro=0.25, micro_scale=500.0,
                                            bump_dist=0.0001, bevel=0.0008), coat=0.35, coat_rough=0.2),
        "rubber": lambda: mat_pbr("rubber", (0.024, 0.024, 0.026, 1), 0.82, 0.0,
                                  dict(texkey="rubber", scratch=0.3, tex_tile=0.04, grime=0.4, edge=0.0,
                                       dust=0.15, micro=0.3, rvar=0.12), spec=0.3, sheen=0.2),
        "plastic_black": lambda: mat_pbr("plastic_black", (0.011, 0.011, 0.012, 1), 0.5, 0.0,
                                         dict(texkey="plastic", scratch=0.2, tex_tile=0.05, prints=0.15, grime=0.5,
                                              edge=0.3, edge_col=(0.07, 0.07, 0.075, 1), edge_rough=0.3, dust=0.06,
                                              micro=0.05, bevel=0.0003)),
        "plastic_gloss_black": lambda: mat_pbr("plastic_gloss_black", (0.012, 0.012, 0.014, 1), 0.08, 0.0,
                                               dict(rvar=0.04, prints=0.45, grime=0.4, edge=0.25,
                                                    edge_col=(0.06, 0.06, 0.065, 1), edge_rough=0.2, dust=0.05,
                                                    micro=0.02, bevel=0.0005), coat=0.7, coat_rough=0.05),
        "plastic_white": lambda: mat_pbr("plastic_white", (0.78, 0.76, 0.70, 1), 0.48, 0.0,
                                         dict(texkey="plastic", scratch=0.2, prints=0.15, grime=0.6, edge=0.2,
                                              dust=0.25), sss=0.15, sss_radius=(0.002, 0.002, 0.002)),
        "plastic_blue": lambda: mat_pbr("plastic_blue", srgb("#1f5fbf"), 0.4, 0.0, dict(grime=0.5, edge=0.25)),
        "plastic_green": lambda: mat_pbr("plastic_green", srgb("#2f9a3c"), 0.45, 0.0,
                                         dict(texkey="plastic", scratch=0.2, grime=0.6, edge=0.3, dust=0.25)),
        "nylon_yellow": lambda: mat_pbr("nylon_yellow", srgb("#e0b21a"), 0.45, 0.0,
                                        dict(texkey="plastic", scratch=0.2, grime=0.6, edge=0.25, dust=0.3,
                                             prints=0.2), sss=0.12, sss_radius=(0.002, 0.002, 0.001)),
        "nylon_tie": lambda: mat_pbr("nylon_tie", srgb("#1a1a1a"), 0.42, 0.0, dict(grime=0.3, edge=0.1)),
        "ic": lambda: _mat_ic(),
        "ceramic": lambda: mat_pbr("ceramic", srgb("#8f7a55"), 0.42, 0.0,
                                   dict(var=0.1, grime=0.3, edge=0.0, dust=0.0, bevel=0.0001)),
        "inductor": lambda: mat_pbr("inductor", (0.035, 0.035, 0.037, 1), 0.62, 0.0,
                                    dict(var=0.1, rvar=0.2, micro=0.5, micro_scale=3000.0, grime=0.3, edge=0.4,
                                         edge_col=(0.25, 0.25, 0.26, 1), bevel=0.0002)),
        "lens_glass": lambda: _mat_lens(),
        "dyneema": lambda: mat_dyneema(),
        "putty": lambda: mat_pbr("putty", srgb("#55504a"), 0.85, 0.0, dict(var=0.15, micro=0.8,
                                                                         micro_scale=900.0, bump_dist=0.0002)),
        "fpc": lambda: mat_pbr("fpc", srgb("#c88a1a"), 0.35, 0.0, dict(grime=0.2, edge=0.0), coat=0.4, sss=0.2,
                               sss_radius=(0.001, 0.0006, 0.0002)),
        "led_red": lambda: mat_simple("led_red", srgb("#ff2a1a"), rough=0.2, emit=srgb("#ff2a1a"), emit_s=2.0),
        "led_blue": lambda: mat_simple("led_blue", srgb("#3a7aff"), rough=0.2, emit=srgb("#3a7aff"), emit_s=1.5),
        "led_green": lambda: mat_simple("led_green", srgb("#30ff60"), rough=0.2, emit=srgb("#30ff60"), emit_s=1.0),
        "ply": lambda: mat_ply(),
        "cell_wrap": lambda: _mat_cell(),
    }
    if name in t:
        return t[name]()
    if name.startswith("pcb_"):
        return mat_pcb(name[4:])
    if name.startswith("wire_"):
        return mat_wire(name[5:])
    raise KeyError(name)


def _mat_zinc():
    m, nt, bsdf = _new_mat("zinc")
    N = _N(nt)
    nz = N.noise(160.0, 4.0, 0.6, N.coord("Object"))
    base = N.ramp(nz.outputs["Fac"], [(0.35, (0.64, 0.66, 0.71, 1)), (0.5, (0.73, 0.73, 0.73, 1)),
                                      (0.65, (0.72, 0.69, 0.62, 1))])
    _detail(N, bsdf, base, 0.3, 1.0, **dict(_STEEL_D, scratch=0.25, grime=0.55))
    return m


def _mat_ic():
    """black epoxy package: slight sheen, fine stipple, laser marking on top faces"""
    m, nt, bsdf = _new_mat("ic")
    N = _N(nt)
    obj = N.coord("Object")
    br = N.node("ShaderNodeTexBrick")
    N.link(N.vscale(obj, 1.0), br.inputs["Vector"])
    br.inputs["Scale"].default_value = 1400.0
    br.inputs["Mortar Size"].default_value = 0.35
    br.inputs["Color1"].default_value = (1, 1, 1, 1)
    br.inputs["Color2"].default_value = (0.8, 0.8, 0.8, 1)
    br.inputs["Mortar"].default_value = (0, 0, 0, 1)
    nz = N.sep(N.geo().outputs["Normal"])[2]
    mark = N.math("MULTIPLY", N.math("MULTIPLY", N.math("LESS_THAN", br.outputs["Fac"], 0.5),
                                     N.math("GREATER_THAN", nz, 0.9)),
                  N.math("GREATER_THAN", N.noise(500.0, 1.0, 0.5, obj).outputs["Fac"], 0.5))
    col = N.mix(N.math("MULTIPLY", mark, 0.6), (0.022, 0.022, 0.024, 1.0), (0.32, 0.32, 0.33, 1.0))
    _detail(N, bsdf, col, 0.36, 0.0, var=0.05, rvar=0.1, grime=0.2, edge=0.25, edge_col=(0.12, 0.12, 0.13, 1),
            dust=0.05, micro=0.5, micro_scale=9000.0, bump_dist=0.00002, bevel=0.0001)
    return m


def _mat_lens():
    m, nt, bsdf = _new_mat("lens_glass")
    _set(bsdf, base=(0.85, 0.9, 1.0, 1), rough=0.02, trans=1.0, ior=1.52, coat=1.0, coat_rough=0.03,
         coat_tint=(0.75, 0.62, 1.0, 1))
    return m


def _mat_cell():
    """18350 TrustFire as Harry's photo: black wrap, orange end (aft face, +x normal), gloss with prints"""
    m, nt, bsdf = _new_mat("cell_wrap")
    N = _N(nt)
    nx = N.sep(N.geo().outputs["Normal"])[0]
    end = N.math("GREATER_THAN", N.math("ABSOLUTE", nx), 0.7)
    col = N.mix(end, srgb("#121314"), srgb("#e8774a"))
    _detail(N, bsdf, col, 0.25, 0.0, prints=0.6, grime=0.3, edge=0.3, dust=0.1)
    _set(bsdf, coat=0.5, coat_rough=0.1)
    return m


def mat_pcb(color):
    """solder mask: satin colour mask over copper (traces and planes show lighter and raised), clear coat sheen,
    white silkscreen outlines and text blocks, a little grime and dust"""
    key = "pcb_" + color
    m, nt, bsdf = _new_mat(key)
    N = _N(nt)
    masks = {"green": "#1c5a2e", "black": "#08090a", "blue": "#1d4f9e", "red": "#a01818", "darkgreen": "#163d26",
             "purple": "#3a1f5c", "fr4": "#b89a5a"}
    base = srgb(masks.get(color, color))
    obj = N.coord("Object")

    def bricks(scale, mortar, vec=None):
        b = N.node("ShaderNodeTexBrick")
        N.link(vec if vec is not None else obj, b.inputs["Vector"])
        b.inputs["Scale"].default_value = scale
        b.inputs["Mortar Size"].default_value = mortar
        b.inputs["Color1"].default_value = (0, 0, 0, 1)
        b.inputs["Color2"].default_value = (0, 0, 0, 1)
        b.inputs["Mortar"].default_value = (1, 1, 1, 1)
        return b.outputs["Fac"]
    rot = N.node("ShaderNodeVectorRotate", rotation_type="Z_AXIS")
    N.link(obj, rot.inputs["Vector"])
    rot.inputs["Angle"].default_value = math.radians(45)
    tr1 = N.math("MULTIPLY", bricks(300.0, 0.03), N.math("GREATER_THAN", N.noise(90.0, 2, 0.5, obj).outputs["Fac"], 0.5))
    tr2 = N.math("MULTIPLY", bricks(220.0, 0.025, rot.outputs["Vector"]),
                 N.math("GREATER_THAN", N.noise(70.0, 2, 0.5, obj).outputs["Fac"], 0.58))
    plane = N.math("GREATER_THAN", N.noise(25.0, 2, 0.5, obj).outputs["Fac"], 0.62)
    copper = N.math("MAXIMUM", N.math("MAXIMUM", tr1, tr2), N.math("MULTIPLY", plane, 0.6))
    silk_o = N.math("MULTIPLY", bricks(420.0, 0.025), N.math("GREATER_THAN", N.noise(70.0, 2, 0.5, obj).outputs["Fac"], 0.72))
    txt = N.math("MULTIPLY", N.math("LESS_THAN", bricks(2600.0, 0.45), 0.5),
                 N.math("GREATER_THAN", N.noise(55.0, 2, 0.5, obj).outputs["Fac"], 0.78))
    silk = N.math("MAXIMUM", silk_o, txt)
    cu_col = tuple(min(1.0, c * 1.6 + 0.012) for c in base[:3]) + (1,)
    if color == "fr4":
        cu_col = srgb("#c8823a")
    col = N.mix(copper, base, cu_col)
    col = N.mix(silk, col, (0.86, 0.86, 0.83, 1))
    _detail(N, bsdf, col, 0.34, 0.0, var=0.04, rvar=0.08, grime=0.35, grime_dist=0.002, edge=0.3,
            edge_col=srgb("#c9b88a"), edge_rough=0.6, dust=0.04, prints=0.1, micro=0.05,
            height=N.math("ADD", N.math("MULTIPLY", copper, 0.6), N.math("MULTIPLY", silk, 0.3)),
            bump_dist=0.00002, bevel=0.00015)
    _set(bsdf, spec=0.6, coat=0.0 if color == "fr4" else 0.45, coat_rough=0.12)
    return m


def mat_wire(color):
    """PVC insulation: slight gloss and sheen"""
    key = "wire_" + color
    cols = {"red": "#c81414", "black": "#141414", "yellow": "#e0c010", "white": "#e6e6e0", "blue": "#1a4ad0",
            "green": "#1a9a3a", "orange": "#ff7a10", "brown": "#6a3a1a", "grey": "#8a8a8a", "purple": "#7a2ab0",
            "usb": "#1a1a1c"}
    return mat_pbr(key, srgb(cols.get(color, color)), 0.38, 0.0,
                   dict(rvar=0.12, grime=0.3, edge=0.0, dust=0.25, prints=0.15, micro=0.05, bevel=0.0),
                   coat=0.2, coat_rough=0.25, sheen=0.25)


def mat_dyneema():
    """1 mm Dyneema braid: yellow / white carriers in a diagonal braid pattern"""
    m, nt, bsdf = _new_mat("dyneema")
    N = _N(nt)
    obj = N.coord("Object")
    xyz = N.sep(obj)
    s = N.math("ADD", N.math("ADD", xyz[0], xyz[1]), xyz[2])
    d = N.math("SUBTRACT", N.math("ADD", xyz[0], xyz[1]), xyz[2])
    a = N.math("SINE", N.math("MULTIPLY", s, 4200.0))
    b = N.math("SINE", N.math("MULTIPLY", d, 4200.0))
    pat = N.math("GREATER_THAN", N.math("MULTIPLY", a, b), 0.0)
    col = N.mix(pat, srgb("#f2ecd8"), srgb("#f0d020"))
    col = N.mix(N.math("MULTIPLY", N.noise(30.0, 3, 0.6, obj).outputs["Fac"], 0.25), col, srgb("#8a8270"))
    _set(bsdf, base=col, rough=0.62, sheen=0.5)
    N.link(N.bump(N.math("ABSOLUTE", N.math("MULTIPLY", a, b)), 0.5, 0.0001), bsdf.inputs["Normal"])
    return m


def mat_rope(color="#f0cf1e", name=None):
    """1 mm 12-strand Dyneema braid for the tether (build.py's line:tether): 6 + 6 carriers crossing in a 2-over-2
    diagonal weave (object-space helices, no UVs needed), each strand's fibre lay as fine streaks, slight fuzz
    (sheen + fibre noise), soft satin highlight. color: '#rrggbb' (default the yellow braid)."""
    key = name or "rope_" + color.lstrip("#")
    if key in _MATS:
        return _MATS[key]
    m, nt, bsdf = _new_mat(key)
    N = _N(nt)
    obj = N.coord("Object")
    x, y, zz = N.sep(obj)
    k = 2 * math.pi / 0.0011                                   # one carrier pitch about 1.1 mm along the rope
    s1 = N.math("ADD", N.math("ADD", x, y), zz)
    s2 = N.math("SUBTRACT", N.math("ADD", x, y), zz)
    a = N.math("SINE", N.math("MULTIPLY", s1, k))
    b = N.math("SINE", N.math("MULTIPLY", s2, k))
    over = N.math("GREATER_THAN", N.math("SINE", N.math("MULTIPLY", N.math("ADD", s1, s2), k * 0.5)), 0.0)
    ridge = N.mix(over, N.math("ABSOLUTE", a), N.math("ABSOLUTE", b), kind="FLOAT")
    fib = N.noise(14000.0, 2.0, 0.6, obj).outputs["Fac"]
    base = srgb(color)
    c = N.mix(over, N.vscale(N.rgb(base), 0.86), N.rgb(base))
    c = N.mix(N.math("MULTIPLY", N.math("SUBTRACT", 1.0, ridge), 0.5), c, N.vscale(N.rgb(base), 0.55))
    c = N.mix(N.math("MULTIPLY", fib, 0.25), c, (0.9, 0.88, 0.8, 1.0))
    _set(bsdf, base=c, rough=N.math("ADD", N.math("MULTIPLY", fib, 0.15), 0.52), sheen=0.7, spec=0.4,
         coat=0.0)
    h = N.math("ADD", ridge, N.math("MULTIPLY", fib, 0.35))
    N.link(N.bump(h, 0.8, 0.00012), bsdf.inputs["Normal"])
    return m


def mat_ply():
    """birch plywood: the Poly Haven plywood face (CC0, box projected, re-toned to pale birch) on the faces,
    alternating plies with dark glue lines on the edges (plies 1.5 mm along object z) with the ambientCG edge
    texture as grain, sanded edge wear, dirt in the grain"""
    m, nt, bsdf = _new_mat("ply")
    N = _N(nt)
    obj = N.coord("Object")
    geo = N.geo()
    nz = N.sep(geo.outputs["Normal"])[2]
    face = N.math("GREATER_THAN", N.math("ABSOLUTE", nz), 0.5)
    xyz = N.sep(obj)
    fine = N.noise(600.0, 3, 0.6, obj).outputs["Fac"]
    blot = N.noise(4.0, 4, 0.6, obj).outputs["Fac"]
    if tex("ply_col", True) is not None:
        tc = N.img(tex("ply_col", True), 0.55, blend=0.05).outputs["Color"]
        bw = N.node("ShaderNodeRGBToBW")
        N.link(tc, bw.inputs[0])
        g = N.maprange(bw.outputs[0], 0.05, 0.35)
        face_col = N.ramp(g, [(0.0, srgb("#b99467")), (0.45, srgb("#d6b98c")), (1.0, srgb("#ead6b0"))])
        fr = N.sep(N.img(tex("ply_rough"), 0.55, blend=0.05).outputs["Color"])[0]
        face_rough = N.math("ADD", N.math("MULTIPLY", fr, 0.35), 0.42)
        fh = N.math("MULTIPLY", N.math("SUBTRACT", 1.0, g), 0.6)
    else:
        wv = N.node("ShaderNodeTexWave", wave_type="BANDS", bands_direction="Y")
        N.link(obj, wv.inputs["Vector"])
        wv.inputs["Scale"].default_value = 22.0
        wv.inputs["Distortion"].default_value = 3.0
        g = N.math("ADD", N.math("MULTIPLY", wv.outputs["Fac"], 0.6), N.math("MULTIPLY", fine, 0.4))
        face_col = N.ramp(g, [(0.25, srgb("#cfae80")), (0.5, srgb("#dcc196")), (0.75, srgb("#e6d1aa"))])
        face_rough = 0.6
        fh = g
    face_col = N.mix(N.math("MULTIPLY", blot, 0.3), face_col, srgb("#bf9a68"))
    ply = N.math("FRACT", N.math("DIVIDE", xyz[2], 0.0015))
    glue = N.math("LESS_THAN", ply, 0.1)
    alt = N.math("FRACT", N.math("DIVIDE", xyz[2], 0.003))
    edge_col = N.mix(N.math("GREATER_THAN", alt, 0.5), srgb("#d1ab79"), srgb("#bb9260"))
    if tex("plyside_col", True) is not None:
        sc = N.img(tex("plyside_col", True), 0.15, blend=0.05).outputs["Color"]
        edge_col = N.mix(0.45, edge_col, sc, blend="OVERLAY")
    edge_col = N.mix(glue, edge_col, srgb("#4a3420"))
    col = N.mix(face, edge_col, face_col)
    rough = N.mix(face, N.math("ADD", N.math("MULTIPLY", fine, 0.2), 0.62), face_rough, kind="FLOAT")
    h = N.mix(face, N.math("ADD", N.math("MULTIPLY", fine, 0.5), N.math("MULTIPLY", glue, -0.8)), fh, kind="FLOAT")
    _detail(N, bsdf, col, 0.0, 0.0, var=0.04, rvar=0.06, grime=0.5, grime_dist=0.01, edge=0.6,
            edge_col=srgb("#e8d4b0"), dust=0.2, micro=0.0, height=h, bump_dist=0.00025, bevel=0.0009)
    rr = bsdf.inputs["Roughness"].links[0].from_socket
    N.link(N.math("ADD", rr, rough, clamp=True), bsdf.inputs["Roughness"])
    _set(bsdf, spec=0.35)
    return m


def mat_lipo_wrap():
    """3S pack shrink wrap: gloss black with a yellow label band and fake print, fingerprints"""
    key = "lipo_wrap"
    if key in _MATS:
        return _MATS[key]
    m, nt, bsdf = _new_mat(key)
    N = _N(nt)
    obj = N.coord("Object")
    xyz = N.sep(obj)
    band = N.math("LESS_THAN", N.math("ABSOLUTE", N.math("SUBTRACT", xyz[0], 0.010)), 0.019)
    txt = N.node("ShaderNodeTexBrick")
    N.link(obj, txt.inputs["Vector"])
    txt.inputs["Scale"].default_value = 160.0
    txt.inputs["Mortar Size"].default_value = 0.25
    txt.inputs["Color1"].default_value = (1, 1, 1, 1)
    txt.inputs["Color2"].default_value = (0.6, 0.6, 0.6, 1)
    txt.inputs["Mortar"].default_value = (0, 0, 0, 1)
    printed = N.math("LESS_THAN", txt.outputs["Fac"], 0.5)
    label = N.mix(printed, srgb("#ffbf00"), srgb("#101010"))
    col = N.mix(band, srgb("#0d0e10"), label)
    stripe = N.math("LESS_THAN", N.math("ABSOLUTE", N.math("SUBTRACT", xyz[0], -0.03)), 0.004)
    col = N.mix(stripe, col, srgb("#d8d8d8"))
    _detail(N, bsdf, col, 0.2, 0.0, var=0.03, rvar=0.08, prints=0.5, grime=0.3, edge=0.25, edge_rough=0.3,
            dust=0.04, micro=0.15, micro_scale=600.0, bump_dist=0.00008, bevel=0.002)
    _set(bsdf, coat=0.5, coat_rough=0.15)
    return m


# ============================================================================================== mesh builder
class MB:
    """accumulates primitives (verts in m, polygons, material slot index) and turns them into one mesh object"""

    def __init__(self):
        self.v, self.f, self.mi = [], [], []
        self.n = 0
        self.mats = []

    def _slot(self, mat):
        if isinstance(mat, str):
            mat = M(mat)
        if mat not in self.mats:
            self.mats.append(mat)
        return self.mats.index(mat)

    def add(self, v, faces, mat, T=None):
        v = np.asarray(v, float).reshape(-1, 3)
        if T is not None:
            T = np.asarray(T, float)
            v = v @ T[:3, :3].T + T[:3, 3]
        s = self._slot(mat)
        self.v.append(v)
        for p in faces:
            self.f.append([i + self.n for i in p])
            self.mi.append(s)
        self.n += len(v)
        return self

    def add_bm(self, bm, mat, T=None):
        bm.verts.ensure_lookup_table()
        v = np.array([x.co[:] for x in bm.verts])
        faces = [[x.index for x in fc.verts] for fc in bm.faces]
        bm.free()
        return self.add(v, faces, mat, T)

    def box(self, lo, hi, mat, bevel=0.0, seg=2, T=None):
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=1.0)
        if bevel > 0:
            r = min(bevel, float((hi - lo).min()) * 0.49)
            sc = hi - lo
            for vv in bm.verts:
                vv.co = Vector([vv.co[i] * sc[i] for i in range(3)])
            bmesh.ops.bevel(bm, geom=list(bm.edges), offset=r, segments=seg, affect="EDGES", profile=0.5)
            for vv in bm.verts:
                vv.co = Vector([vv.co[i] / sc[i] for i in range(3)])
        for vv in bm.verts:
            vv.co = Vector([lo[i] + (vv.co[i] + 0.5) * (hi[i] - lo[i]) for i in range(3)])
        return self.add_bm(bm, mat, T)

    def cyl(self, p0, p1, r, mat, seg=24, r2=None, cap=True, T=None):
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        d = p1 - p0
        L = float(np.linalg.norm(d))
        bm = bmesh.new()
        bmesh.ops.create_cone(bm, cap_ends=cap, cap_tris=False, segments=seg, radius1=r,
                              radius2=r if r2 is None else r2, depth=L)
        R = _rot_z_to(d / max(L, 1e-12))
        A = np.eye(4)
        A[:3, :3] = R
        A[:3, 3] = (p0 + p1) / 2
        if T is not None:
            A = np.asarray(T, float) @ A
        return self.add_bm(bm, mat, A)

    def tube(self, pts, r, mat, seg=8, closed=False, T=None, cap=True):
        """swept circle along a polyline (parallel-transport frames)"""
        pts = np.asarray(pts, float)
        n = len(pts)
        tang = np.gradient(pts, axis=0)
        if closed:
            tang = np.roll(pts, -1, 0) - np.roll(pts, 1, 0)
        tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-12
        ref = np.array([0, 0, 1.0]) if abs(tang[0][2]) < 0.9 else np.array([1.0, 0, 0])
        nrm = np.cross(tang[0], ref)
        nrm /= np.linalg.norm(nrm)
        verts = []
        for i in range(n):
            t = tang[i]
            nrm = nrm - t * np.dot(nrm, t)
            nrm /= np.linalg.norm(nrm) + 1e-12
            bi = np.cross(t, nrm)
            for k in range(seg):
                a = 2 * math.pi * k / seg
                verts.append(pts[i] + r * (math.cos(a) * nrm + math.sin(a) * bi))
        faces = []
        rng = n if closed else n - 1
        for i in range(rng):
            j = (i + 1) % n
            for k in range(seg):
                k2 = (k + 1) % seg
                faces.append([i * seg + k, i * seg + k2, j * seg + k2, j * seg + k])
        if cap and not closed:
            faces.append(list(range(seg))[::-1])
            faces.append([(n - 1) * seg + k for k in range(seg)])
        return self.add(np.array(verts), faces, mat, T)

    def torus(self, c, R, r, mat, seg=24, rseg=8, axis=(0, 0, 1), T=None):
        a = np.linspace(0, 2 * math.pi, seg, endpoint=False)
        pts = np.c_[R * np.cos(a), R * np.sin(a), np.zeros(seg)]
        A = np.eye(4)
        A[:3, :3] = _rot_z_to(np.asarray(axis, float))
        A[:3, 3] = c
        if T is not None:
            A = np.asarray(T, float) @ A
        return self.tube(pts, r, mat, rseg, closed=True, T=A)

    def obj(self, name, col, parent=None, matrix=None, smooth=30.0):
        if not self.v:
            v, f, mi = np.zeros((0, 3)), [], []
        else:
            v, f, mi = np.vstack(self.v), self.f, self.mi
        me = mesh_np(name, v, f, mi, smooth)
        return mesh_object(name, me, col, parent, matrix, self.mats)


def plate_hole(mb, x0, x1, y0, y1, z0, z1, cx, cy, r, mat, seg=48):
    """rectangular plate (x0..x1, y0..y1, z0..z1) with a round through hole (centre cx, cy, radius r)"""
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    angs = set(np.round(np.linspace(0, 2 * math.pi, seg, endpoint=False), 9))
    angs |= {round(math.atan2(y - cy, x - cx) % (2 * math.pi), 9) for x, y in corners}
    angs = sorted(angs)
    v = []
    for a in angs:
        c, s_ = math.cos(a), math.sin(a)
        ts = [t for t in ((x0 - cx) / c if abs(c) > 1e-12 else None, (x1 - cx) / c if abs(c) > 1e-12 else None,
                          (y0 - cy) / s_ if abs(s_) > 1e-12 else None, (y1 - cy) / s_ if abs(s_) > 1e-12 else None)
              if t is not None and t > 0]
        t = min(ts)
        ix, iy, ox, oy = cx + r * c, cy + r * s_, cx + t * c, cy + t * s_
        v += [(ix, iy, z0), (ix, iy, z1), (ox, oy, z0), (ox, oy, z1)]
    n = len(angs)
    f = []
    for i in range(n):
        j = (i + 1) % n
        a, b = 4 * i, 4 * j
        f += [[a + 1, a + 3, b + 3, b + 1], [a, b, b + 2, a + 2], [a, a + 1, b + 1, b], [a + 2, b + 2, b + 3, a + 3]]
    return mb.add(np.array(v), f, mat)


def _rot_z_to(d):
    """rotation matrix taking +Z to unit vector d"""
    d = np.asarray(d, float)
    d = d / (np.linalg.norm(d) + 1e-12)
    z = np.array([0, 0, 1.0])
    c = float(np.dot(z, d))
    if c > 1 - 1e-9:
        return np.eye(3)
    if c < -1 + 1e-9:
        return np.diag([1.0, -1.0, -1.0])
    ax = np.cross(z, d)
    s = np.linalg.norm(ax)
    ax /= s
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + s * K + (1 - c) * K @ K


def T_(x=0.0, y=0.0, z=0.0):
    A = np.eye(4)
    A[:3, 3] = (x, y, z)
    return A


def R_(axis, deg):
    A = np.eye(4)
    A[:3, :3] = np.array(Matrix.Rotation(math.radians(deg), 3, axis))
    return A


# ============================================================================================== base
def _rot_deg(axis, deg):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    x, y, z = a
    C = 1 - c
    R = np.eye(4)
    R[:3, :3] = [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                 [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                 [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]
    return R


def base_joint_world(asm, values=None):
    """asm.json joints -> {joint: 4x4 base frame (mm)} for joint values (default: the rest state)"""
    vals = BASE_REST_JOINTS if values is None else values
    J = {j["name"]: j for j in asm["joints"]}
    out = {}

    def local(j, v):
        Mx = T_(*j["origin"])
        if j["base"] is not None:
            Mx = Mx @ np.asarray(j["base"], float)
        if j["kind"] == "rev":
            Mx = Mx @ _rot_deg(j["axis"], v)
        elif j["kind"] == "slide":
            Mx = Mx @ T_(*(np.asarray(j["axis"], float) * v))
        return Mx

    def get(n):
        if n not in out:
            j = J[n]
            out[n] = np.eye(4)
            Mp = np.eye(4) if j["parent"] not in J else get(j["parent"])
            out[n] = Mp @ local(j, float(vals.get(n, 0.0)))
        return out[n]

    for n in J:
        get(n)
    return out


def base_rest_world(asm=None):
    """{part: 4x4 base frame (mm)} at the rest state"""
    asm = asm or json.load(open(os.path.join(BASE_DIR, "asm.json")))
    JW = base_joint_world(asm)
    return {p["name"]: (JW[p["joint"]] if p["joint"] else np.eye(4)) @ np.asarray(p["place"], float)
            for p in asm["parts"]}


SKIP_BASE = ("launch_dart",)                    # replaced by the v34 dart


def _is_spring_seg(n):
    return n.startswith("launch_spring_") and n.rsplit("_", 1)[-1].isdigit()


def _base_material(p):
    n, mat, col = p["name"], p["material"], p["color"]
    if mat == "PLA":
        return mat_printed(col, "PLA")
    if mat == "PPA-CF":
        return mat_printed(col, "PPA-CF")
    if mat == "steel":
        return M("steel")
    if n.startswith("launch_line"):
        return M("dyneema")
    if "_brg" in n or n.startswith("reel_brg") or n.startswith("launch_aim_brg"):
        return M("steel_bright")
    if n in ("launch_sprag", "reel_sprag"):
        return M("steel")
    if n.startswith("launch_spring"):
        return M("spring_steel")
    if any(k in n for k in ("screw", "_nut", "_jam", "bolt", "_hw", "_pin", "rocker_pin")):
        return M("zinc")
    if "axle" in n or n == "frame_wiper_shaft":
        return M("steel")
    if n == "sensors_as5600":
        return M("pcb_black")
    return M("plastic_black")


def _split_by(v, f, axis, thr, below_mat, above_mat):
    """per-face material index: face centroid coordinate < thr -> 0 else 1"""
    c = v[f].mean(1)[:, axis]
    return [below_mat, above_mat], (c >= thr).astype(np.int32)


def _decimate(ob, target):
    n = len(ob.data.polygons)
    if n <= target:
        return
    md = ob.modifiers.new("dec", "DECIMATE")
    md.ratio = target / n
    md.use_collapse_triangulate = True
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    old = ob.data
    ob.modifiers.remove(md)
    ob.data = me
    for m in old.materials:
        if m.name not in [x.name for x in me.materials if x]:
            me.materials.append(m)
    bpy.data.meshes.remove(old)


def build_base(scene, col=None):
    """every base part at its rest pose under base:root; returns base:root"""
    col = col or get_collection("BASE", scene)
    asm = json.load(open(os.path.join(BASE_DIR, "asm.json")))
    W = base_rest_world(asm)
    root = empty("base:root", col, matrix=BASE_TO_WORLD, size=0.1)
    proc = _procedural_base_parts()
    for p in asm["parts"]:
        n = p["name"]
        if n.startswith(SKIP_BASE) or _is_spring_seg(n):
            continue
        name = "base:" + n
        Mw = mm_matrix(W[n])
        if n in proc:
            mb = proc[n]()
            ob = mb.obj(name, col, root, Mw)
        else:
            z = np.load(os.path.join(BASE_DIR, "meshes", n + ".npz"))
            v = z["v"].astype(float)
            f = z["f"].astype(np.int64)
            mats, mi = [_base_material(p)], None
            if n == "frame_wiper_motor":               # gearbox (cast alu) + motor can (black paint), local x > 50
                mats, mi = _split_by(v, f, 0, 50.0, M("alu_cast"), M("paint_black"))
            elif n == "drum_n20":                      # N20: brass gearbox at the pinion end, nickel can
                mats, mi = _split_by(v, f, 2, 16.0, M("brass"), M("steel_bright"))
            me = mesh_np(name, v * MM, f, mi)
            if p["material"] in ("PLA", "PPA-CF"):
                pt = np.asarray(p["print_tf"] if p.get("print_tf") else np.eye(4), float)
                set_attr(me, "layer", v @ pt[2, :3] + pt[2, 3])
            ob = mesh_object(name, me, col, root, Mw, mats)
            if len(f) > 60000:
                _decimate(ob, 45000)
        ob["asm_part"] = n
        ob["subsystem"] = p["subsystem"]
        ob["joint"] = p["joint"] or ""
        if n.startswith("launch_line"):
            ob["is_line"] = True                       # the in-base tether pieces (build.py may hide them)
    _build_springs(scene, col, root)
    _build_cable_ties(col, root, W)
    _build_spool_line(col)
    return root


def _build_spool_line(col):
    """the 20 m of 1 mm Dyneema wound on the storage spool (core d40 between the flanges at local z 130..150; pack
    OD about 51 mm), a child of base:reel_spool so it turns with it"""
    sp = bpy.data.objects.get("base:reel_spool")
    if sp is None:
        return
    mb = MB()
    seg = 72
    a = np.linspace(0, 2 * math.pi, seg, endpoint=False)
    ro, ri, z0, z1 = 25.5, 20.0, 130.3, 149.7
    v = []
    for z in (z0, z1):
        v += [(ro * math.cos(t), ro * math.sin(t), z) for t in a]
    for z in (z0, z1):
        v += [(ri * math.cos(t), ri * math.sin(t), z) for t in a]
    f = []
    for i in range(seg):
        j = (i + 1) % seg
        f.append([i, j, seg + j, seg + i])                                   # outer
        f.append([2 * seg + j, 2 * seg + i, i, j])                          # bottom annulus
        f.append([seg + i, seg + j, 3 * seg + j, 3 * seg + i])              # top annulus
    mb.add(np.array(v), f, mat_line_pack())
    ob = _mm(mb).obj("base:reel_spool_line", col, sp, np.eye(4))
    ob["note"] = "Dyneema pack on the storage spool (20 m, 1 mm)"


def mat_line_pack():
    """wound 1 mm braid: yellow / white wraps, ridges every 1 mm along the spool axis (object z)"""
    key = "dyneema_pack"
    if key in _MATS:
        return _MATS[key]
    m, nt, bsdf = _new_mat(key)
    N = _N(nt)
    obj = N.coord("Object")
    xyz = N.sep(obj)
    ang = N.math("ARCTAN2", xyz[1], xyz[0])
    ph = N.math("ADD", N.math("DIVIDE", xyz[2], 0.001), N.math("DIVIDE", ang, 2 * math.pi))
    ridge = N.math("ABSOLUTE", N.math("SUBTRACT", N.math("FRACT", ph), 0.5))
    nz = N.noise(800.0, 3, 0.6, obj).outputs["Fac"]
    col = N.mix(N.math("GREATER_THAN", N.math("SINE", N.math("MULTIPLY", ang, 9.0)), 0.3), srgb("#f2ecd8"),
                srgb("#eed22a"))
    _set(bsdf, base=col, rough=0.6, sheen=0.4)
    N.link(N.bump(N.math("ADD", ridge, N.math("MULTIPLY", nz, 0.3)), 0.6, 0.0003), bsdf.inputs["Normal"])
    return m


# ---------------------------------------------------------------------------------------------- springs
def _spring_nodegroup():
    name = "GS_extension_spring"
    ng = bpy.data.node_groups.get(name)
    if ng is not None:
        return ng
    ng = bpy.data.node_groups.new(name, "GeometryNodeTree")
    itf = ng.interface
    itf.new_socket("Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    itf.new_socket("Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    s_len = itf.new_socket("Length", in_out="INPUT", socket_type="NodeSocketFloat")
    s_coil = itf.new_socket("Coils", in_out="INPUT", socket_type="NodeSocketFloat")
    s_r = itf.new_socket("Coil Radius", in_out="INPUT", socket_type="NodeSocketFloat")
    s_w = itf.new_socket("Wire Radius", in_out="INPUT", socket_type="NodeSocketFloat")
    s_res = itf.new_socket("Resolution", in_out="INPUT", socket_type="NodeSocketInt")
    s_len.default_value = 0.165
    s_coil.default_value = SPRING_SPEC["coils"]
    s_r.default_value = (SPRING_SPEC["od"] - SPRING_SPEC["wire"]) / 2
    s_w.default_value = SPRING_SPEC["wire"] / 2
    s_res.default_value = SPRING_SPEC["res"]
    N = ng.nodes
    gi = N.new("NodeGroupInput")
    go = N.new("NodeGroupOutput")
    sp = N.new("GeometryNodeCurveSpiral")
    L = ng.links
    L.new(gi.outputs["Resolution"], sp.inputs["Resolution"])
    L.new(gi.outputs["Coils"], sp.inputs["Rotations"])
    L.new(gi.outputs["Coil Radius"], sp.inputs["Start Radius"])
    L.new(gi.outputs["Coil Radius"], sp.inputs["End Radius"])
    L.new(gi.outputs["Length"], sp.inputs["Height"])
    circ = N.new("GeometryNodeCurvePrimitiveCircle")
    circ.mode = "RADIUS"
    circ.inputs["Resolution"].default_value = 8
    L.new(gi.outputs["Wire Radius"], circ.inputs["Radius"])
    c2m = N.new("GeometryNodeCurveToMesh")
    L.new(sp.outputs["Curve"], c2m.inputs["Curve"])
    L.new(circ.outputs["Curve"], c2m.inputs["Profile Curve"])
    xf = N.new("GeometryNodeTransform")
    L.new(c2m.outputs["Mesh"], xf.inputs["Geometry"])
    xf.inputs["Rotation"].default_value = (0.0, math.pi / 2, 0.0)       # spiral +Z -> spring +X
    sm = N.new("GeometryNodeSetShadeSmooth")
    L.new(xf.outputs["Geometry"], sm.inputs["Geometry"])
    smat = N.new("GeometryNodeSetMaterial")
    L.new(sm.outputs["Geometry"], smat.inputs["Geometry"])
    smat.inputs["Material"].default_value = M("spring_steel")
    L.new(smat.outputs["Geometry"], go.inputs["Geometry"])
    return ng


def _socket_id(ng, name):
    for it in ng.interface.items_tree:
        if getattr(it, "name", None) == name and getattr(it, "in_out", "") == "INPUT":
            return it.identifier
    raise KeyError(name)


def _hook_mesh(sign):
    """C-243 machine loop on the 8 mm pin (pin axis = local z through the origin) and the turn into the coil,
    which starts at local (12, 0, -7.3) mm (spring frame of set_spring); sign -1 mirrors (hook_a)."""
    R = (SPRING_SPEC["od"] - SPRING_SPEC["wire"]) / 2 / MM          # 7.3 mean radius
    r = SPRING_SPEC["wire"] / 2 / MM
    lc = R - (SPRING_SPEC["pin_d"] / 2 / MM + r)                      # loop bears on the pin's far side
    a = np.linspace(0, 2 * math.pi, 40)
    loop = np.c_[lc + R * np.cos(a), R * np.sin(a), np.zeros_like(a)]
    turn = np.array([[lc + R, 0, 0], [lc + R + 0.9, 0, -1.2], [11.4, 0, -3.6], [12.0, 0, -5.6], [12.2, 0, -R]])
    pts = np.vstack([loop[:-1], turn])
    mb = MB()
    mb.tube(pts * MM, r * MM, "spring_steel", 8)
    if sign < 0:
        mb.v = [v * np.array([-1.0, -1.0, 1.0]) for v in mb.v]
    return mb


def set_spring(obj, p0, p1, up=(0.0, 0.0, 1.0), frame=None):
    """Place / stretch a spring coil object between world points p0 (hook_c coil end) and p1 (hook_a coil end);
    keyframes location, rotation and the coil length when frame is given."""
    if isinstance(obj, str):
        obj = bpy.data.objects[obj]
    p0, p1 = Vector(p0), Vector(p1)
    d = p1 - p0
    L = d.length
    x = d.normalized()
    z = Vector(up)
    z = (z - x * z.dot(x)).normalized()
    y = z.cross(x)
    Mw = Matrix(((x[0], y[0], z[0], p0[0]), (x[1], y[1], z[1], p0[1]), (x[2], y[2], z[2], p0[2]), (0, 0, 0, 1)))
    obj.matrix_world = Mw
    path = _gn_set(obj, "Spring", "Length", float(L))
    obj.update_tag()
    if frame is not None:
        obj.keyframe_insert("location", frame=frame)
        obj.keyframe_insert("rotation_euler", frame=frame)
        obj.keyframe_insert(path, frame=frame)
    return L


def _gn_set(obj, mod, socket_name, value):
    """set a geometry-nodes modifier input (Blender 5.x interface API, 4.x id-property fallback); returns the
    data path for keyframing"""
    md = obj.modifiers[mod]
    sid = _socket_id(md.node_group, socket_name)
    try:
        getattr(md.properties.inputs, sid).value = value
        return f'modifiers["{mod}"].properties.inputs.{sid}.value'
    except AttributeError:
        md[sid] = value
        return f'modifiers["{mod}"]["{sid}"]'


def set_spring_pins(obj, pin_c, pin_a, up=(0.0, 0.0, 1.0), frame=None):
    """as set_spring, from the two pin centres (carriage peg, rail anchor pin; world m, e.g. from
    render/motion/base_motion.py BaseKin.spring_ends): the coil ends sit HOOK_L (12 mm) inside each pin"""
    a, b = Vector(pin_c), Vector(pin_a)
    d = (b - a).normalized() * SPRING_SPEC["hook_l"]
    return set_spring(obj, a + d, b - d, up, frame)


def spring_ends(name):
    """world coil end points (p0 at hook_c, p1 at hook_a) and the up vector from the current hook objects"""
    s = SPRINGS[name]
    hc, ha = bpy.data.objects[s["hook_c"]], bpy.data.objects[s["hook_a"]]
    k = SPRING_SPEC["hook_l"]
    p0 = hc.matrix_world @ Vector((k, 0, 0))
    p1 = ha.matrix_world @ Vector((-k, 0, 0))
    up = (hc.matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized()
    return p0, p1, up


def update_springs(frame=None):
    """re-fit both coils to their hooks (call after the hooks moved; keyframes if frame is given)"""
    for nm in SPRINGS:
        if nm in bpy.data.objects:
            p0, p1, up = spring_ends(nm)
            set_spring(nm, p0, p1, up, frame)


def key_springs(scene, frames):
    """keyframe both coils on the given frames from the (already animated) hook objects"""
    for f in frames:
        scene.frame_set(f)
        update_springs(frame=f)


def key_springs_from_motion(motion=None, step=1):
    """keyframe both coils from render/motion/motion.json "springs" (anchor_pin / carriage_pin world positions per
    frame); up vector = the rail's +z (pins are vertical in the rail frame)"""
    if motion is None:
        with open(MOTION_JSON) as fh:
            motion = json.load(fh)
    for side in ("l", "r"):
        sp = motion.get("springs", {}).get(f"launch_spring_{side}")
        obn = f"base:launch_spring_{side}"
        if not sp or obn not in bpy.data.objects:
            continue
        for i in range(0, len(sp["frames"]), step):
            f = sp["frames"][i]
            a, c = Vector(sp["anchor_pin"][i]), Vector(sp["carriage_pin"][i])
            d = (a - c).normalized()
            up = Vector((-d.z, 0.0, d.x)) if abs(d.y) < 0.9 else Vector((0, 0, 1))
            if up.z < 0:
                up = -up
            set_spring_pins(obn, c, a, up, frame=f)


def _build_springs(scene, col, root):
    ng = _spring_nodegroup()
    for nm in SPRINGS:
        me = bpy.data.meshes.new(nm)
        ob = mesh_object(nm, me, col, root)
        md = ob.modifiers.new("Spring", "NODES")
        md.node_group = ng
        me.materials.append(M("spring_steel"))
        ob["spring"] = "Century C-243"
    bpy.context.view_layer.update()
    update_springs()


def _build_cable_ties(col, root, W):
    """black nylon ties round the cable bundles at the two deck anchors (base frame, rest)"""
    for i in (1, 2):
        n = f"frame_cable_anchor_{i}"
        if n not in W:
            continue
        lo = np.array([187.0, -108.0, -165.0]) if i == 1 else np.array([203.0, 122.0, -165.0])
        c = lo + (np.array([16.0, 24.0, 10.0]) if i == 1 else np.array([24.0, 16.0, 10.0])) / 2
        mb = MB()
        a = np.linspace(0, 2 * math.pi, 28)
        rx, rz = 6.5, 5.0
        pts = np.c_[np.zeros_like(a), rx * np.cos(a), 4.0 + rz * np.sin(a)]
        if i == 2:
            pts = pts[:, [1, 0, 2]]
        mb.tube((pts + c) * MM, 0.7 * MM, "nylon_tie", 6, closed=True)
        hc = c + np.array([0, 0, 4.0 + rz + 1.2])
        mb.box((hc - [2.6, 2.6, 1.6]) * MM, (hc + [2.6, 2.6, 1.6]) * MM, "nylon_tie", bevel=0.4 * MM)
        mb.box((hc + [-1.0, 2.4, -0.4]) * MM, (hc + [1.0, 9.0, 0.4]) * MM, "nylon_tie")
        mb.obj(f"base:cable_tie_{i}", col, root)


# ============================================================================================== dart
# parts.json root parts that ride on a moving part (same deploy axis) -> the moving object they hang under
DART_RIDES_ON = {"coupler_L": "drill_leg_L", "deploy_shaft": "drill_leg_L", "coupler_R": "drill_leg_R"}
# children re-parented to the spinning auger (parts.json hangs them on the arm; they turn with the auger)
DART_ON_AUGER = {"drill_leg_L_shaft": "drill_leg_L_auger", "drill_leg_L_nut": "drill_leg_L_auger",
                 "drill_leg_R_shaft": "drill_leg_R_auger", "drill_leg_R_nut": "drill_leg_R_auger"}


def dart_rest_matrix():
    """world matrix of dart:root at rest: folded on the launcher cradle (asm launch_dart transform)"""
    asm = json.load(open(os.path.join(BASE_DIR, "asm.json")))
    W = base_rest_world(asm)
    return np.asarray(BASE_TO_WORLD) @ mm_matrix(W["launch_dart"])


def _dart_material(p, v, f):
    n, note = p["name"], p.get("note", "")
    if p.get("printable"):
        if "(PPA-CF)" in note or n.startswith("coupler") or n.startswith("leg_coupler"):
            return [mat_printed(p["color"], "PPA-CF")], None
        return [mat_printed(p["color"], "PLA")], None
    if n.startswith("cell_"):
        return [M("cell_wrap")], None
    if n in ("n20_leg", "n20_deploy"):                   # worm box up top (brass), nickel can pointing down
        top = v[:, 2].max()
        return _split_by(v, f, 2, top - 11.93, M("steel_bright"), M("brass"))
    if n.endswith("_motor"):                             # drill gearmotor: gearbox at the shaft end (x min)
        return _split_by(v, f, 0, v[:, 0].min() + 9.0, M("brass"), M("steel_bright"))
    if n.endswith("_shaft"):
        return ([mat_thread()] if "drill" in n else [M("steel")]), None
    if n.endswith("_nut") or n == "screws_radial":
        return [M("zinc")], None
    if n.endswith("_screw"):
        return [M("black_oxide")], None
    if n.startswith("contacts"):
        return [M("steel_bright")], None
    if n.startswith("jst"):
        return [M("plastic_white")], None
    if n == "antenna":
        return [M("plastic_black")], None
    if n == "xiao_antenna":
        return [M("fpc")], None
    if n == "capacitor":
        return [mat_simple("cap_blue", srgb("#24408c"), rough=0.35, coat=0.3)], None
    if n == "ballast":
        return [M("putty")], None
    return [M("plastic_black")], None


def mat_thread():
    """M3 threaded steel rod: helical ridges as bump (pitch 0.5 mm along object x)"""
    key = "steel_thread"
    if key in _MATS:
        return _MATS[key]
    m, nt, bsdf = _new_mat(key)
    N = _N(nt)
    xyz = N.sep(N.coord("Object"))
    ang = N.math("ARCTAN2", xyz[2], xyz[1])
    ph = N.math("ADD", N.math("DIVIDE", xyz[0], 0.0005), N.math("DIVIDE", ang, 2 * math.pi))
    h = N.math("ABSOLUTE", N.math("SUBTRACT", N.math("FRACT", ph), 0.5))
    _set(bsdf, base=(0.66, 0.66, 0.67, 1), metal=1.0, rough=0.3)
    N.link(N.bump(h, 0.8, 0.00015), bsdf.inputs["Normal"])
    return m


def build_dart(scene, col=None, matrix=None):
    """all kedge/out34 parts under dart:root (rest: folded on the cradle); returns dart:root"""
    col = col or get_collection("DART", scene)
    J = json.load(open(os.path.join(DART_DIR, "parts.json")))
    PP = {p["name"]: p for p in J["parts"]}
    root = empty("dart:root", col, matrix=dart_rest_matrix() if matrix is None else matrix, size=0.05)
    detail = _procedural_dart_parts()
    made = {"dart:root": root}

    def place(n):
        """(parent object name, local matrix) for part n"""
        p = PP[n]
        on = "dart:" + n
        if on in DART_JOINTS:
            d = DART_JOINTS[on]
            return d["parent"], T_(*d["pivot"])
        if n in DART_ON_AUGER:
            return "dart:" + DART_ON_AUGER[n], np.eye(4)
        if p.get("parent"):
            return "dart:" + p["parent"], T_(*(np.asarray(p["pos"], float) * MM))
        pos = np.asarray(p["pos"], float) * MM
        if n in DART_RIDES_ON:
            host = "dart:" + DART_RIDES_ON[n]
            return host, T_(*(pos - np.asarray(DART_JOINTS[host]["pivot"])))
        return "dart:root", T_(*pos)

    todo = list(PP)
    while todo:
        rest = []
        for n in todo:
            par, local = place(n)
            if par not in made:
                rest.append(n)
                continue
            name = "dart:" + n
            p = PP[n]
            if n in detail:
                ob = detail[n]().obj(name, col, made[par], local)
            else:
                z = np.load(os.path.join(DART_DIR, n + ".npz"))
                v = z["v"].astype(float)
                f = z["f"].astype(np.int64)
                mats, mi = _dart_material(p, v, f)
                me = mesh_np(name, v * MM, f, mi)
                if p.get("printable"):
                    set_attr(me, "layer", v[:, 2])
                ob = mesh_object(name, me, col, made[par], local, mats)
            ob["kedge_part"] = n
            if name in DART_JOINTS:
                ob["joint_axis"] = list(DART_JOINTS[name]["axis"])
                ob["joint"] = DART_JOINTS[name]["joint"]
                ob.rotation_mode = "QUATERNION"
            made[name] = ob
        if len(rest) == len(todo):
            raise RuntimeError(f"dart parts with unresolved parents: {rest}")
        todo = rest
    return root


def apply_dart_joint(name, deg, frame=None):
    """rotate a moving dart object (DART_JOINTS name or alias) to deg about its axis; keyframe when frame given"""
    name = DART_JOINT_ALIASES.get(name, name)
    ob = bpy.data.objects.get(name)
    if ob is None:
        return None
    d = DART_JOINTS[name]
    ob.rotation_mode = "QUATERNION"
    ob.location = d["pivot"]
    ob.rotation_quaternion = Matrix.Rotation(math.radians(deg), 4, Vector(d["axis"])).to_quaternion()
    if frame is not None:
        ob.keyframe_insert("rotation_quaternion", frame=frame)
    return ob


def pose_dart(leg_deg=0.0, deploy_L=0.0, deploy_R=None, auger_L=0.0, auger_R=0.0, frame=None):
    """set all dart joints (degrees as DART_JOINTS); keyframes rotation when frame is given"""
    deploy_R = deploy_L if deploy_R is None else deploy_R
    for nm, deg in (("dart:righting_leg", leg_deg), ("dart:drill_leg_L", deploy_L), ("dart:drill_leg_R", deploy_R),
                    ("dart:drill_leg_L_auger", auger_L), ("dart:drill_leg_R_auger", auger_R)):
        apply_dart_joint(nm, deg, frame)


# ============================================================================================== procedural parts
def _mm(mb):
    """builders below work in mm: scale to metres"""
    mb.v = [v * MM for v in mb.v]
    return mb


def bezier(p0, t0, p1, t1, n=16):
    """cubic Bezier from p0 (tangent t0, length-scaled) to p1 (arriving along t1)"""
    p0, p1, t0, t1 = (np.asarray(a, float) for a in (p0, p1, t0, t1))
    u = np.linspace(0, 1, n)[:, None]
    c1, c2 = p0 + t0, p1 - t1
    return (1 - u) ** 3 * p0 + 3 * (1 - u) ** 2 * u * c1 + 3 * (1 - u) * u ** 2 * c2 + u ** 3 * p1


def _smd_scatter(mb, x0, x1, y0, y1, z, n, seed, avoid=()):
    rng = np.random.default_rng(seed)
    for _ in range(n):
        for _try in range(10):
            x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
            if not any(a[0] <= x <= a[2] and a[1] <= y <= a[3] for a in avoid):
                break
        big = rng.random() < 0.3
        w, h = (2.0, 1.25) if big else (1.6, 0.8)
        if rng.random() < 0.5:
            w, h = h, w
        mat = ["ceramic", "ic", "ceramic", "tin"][rng.integers(0, 4)]
        mb.box((x, y, z), (x + w, y + h, z + (0.6 if big else 0.45)), mat)
        mb.box((x, y, z), (x + min(w, h) * 0.25 if w < h else x + w * 0.2, y + h, z + 0.5), "tin") if mat != "tin" else None


def _header(mb, x, y0, n, z_base, up=True, pitch=2.54, along="y", pin_len=6.0, base_h=2.5):
    """single row pin header: black base + gold square pins"""
    L = n * pitch
    if along == "y":
        lo, hi = (x - 1.27, y0 - 1.27), (x + 1.27, y0 - 1.27 + L)
    else:
        lo, hi = (y0 - 1.27, x - 1.27), (y0 - 1.27 + L, x + 1.27)
    zb0, zb1 = (z_base, z_base + base_h) if up else (z_base - base_h, z_base)
    mb.box((lo[0], lo[1], zb0), (hi[0], hi[1], zb1), "plastic_black", bevel=0.2)
    for k in range(n):
        c = (x, y0 + k * pitch) if along == "y" else (y0 + k * pitch, x)
        za, zb = (z_base - 3.0, z_base + base_h + pin_len) if up else (z_base - base_h - pin_len, z_base + 1.0)
        mb.box((c[0] - 0.32, c[1] - 0.32, za), (c[0] + 0.32, c[1] + 0.32, zb), "gold")


def _pads(mb, pts, z, r=0.9):
    for (x, y) in pts:
        mb.cyl((x, y, z), (x, y, z + 0.05), r, "gold", 12)
        mb.cyl((x, y, z + 0.05), (x, y, z + 0.45), r * 0.75, "tin", 10, r2=r * 0.3)


def _ic(mb, x0, y0, x1, y1, z, h, pins_along="x", npins=4, pitch=1.27):
    mb.box((x0, y0, z + 0.1), (x1, y1, z + h), "ic", bevel=0.15)
    if pins_along == "x":
        for side, yy in ((-1, y0), (1, y1)):
            for k in range(npins):
                xx = (x0 + x1) / 2 + (k - (npins - 1) / 2) * pitch
                mb.box((xx - 0.2, min(yy, yy + side * 0.9), z), (xx + 0.2, max(yy, yy + side * 0.9), z + 0.35), "tin")
    else:
        for side, xx in ((-1, x0), (1, x1)):
            for k in range(npins):
                yy = (y0 + y1) / 2 + (k - (npins - 1) / 2) * pitch
                mb.box((min(xx, xx + side * 0.9), yy - 0.2, z), (max(xx, xx + side * 0.9), yy + 0.2, z + 0.35), "tin")


# --- electronics bay boards (bay frame, base/cad/parts/_frame_ebay.py BOARD / PLACE numbers, mm)
_BAY = dict(FLOOR=3.0, WALL=2.4, ESP_Z=30.0, BTS_Z=27.0)


def _p_esp32():
    """Espressif ESP32-S3-DevKitC-1-N8 (Harry's: black PCB, two USB-C), pins down into DuPont housings"""
    mb = MB()
    ex, ey = 13.0, 3.0
    bw, bl, bt = 25.40, 62.87, 1.6
    z0 = _BAY["ESP_Z"]
    zt = z0 + bt
    mb.box((ex, ey, z0), (ex + bw, ey + bl, zt), "pcb_black", bevel=0.3)
    my1 = ey + bl + 6.33
    my0 = my1 - 25.5
    mb.box((ex + 3.7, my0, zt), (ex + 21.7, my1, zt + 0.8), "pcb_black", bevel=0.2)            # WROOM-1 module
    mb.box((ex + 4.6, my0 + 0.6, zt + 0.8), (ex + 20.8, my0 + 18.2, zt + 3.1), "tin", bevel=0.35)
    for k in range(6):                                                                       # PCB antenna
        mb.box((ex + 5.0 + k * 2.6, my0 + 19.2, zt + 0.8), (ex + 5.5 + k * 2.6, my0 + 24.6, zt + 0.84), "gold")
    mb.box((ex + 5.0, my0 + 24.1, zt + 0.8), (ex + 18.5, my0 + 24.6, zt + 0.84), "gold")
    for ux in (6.0, 19.4):                                                                    # USB-C x 2
        mb.box((ex + ux - 4.47, ey - 0.6, zt), (ex + ux + 4.47, ey + 6.8, zt + 3.26), "steel_bright", bevel=1.4, seg=3)
        mb.box((ex + ux - 3.7, ey - 0.7, zt + 0.55), (ex + ux + 3.7, ey - 0.2, zt + 2.7), "plastic_black", bevel=1.0)
    for bx_ in (1.2, 20.6):                                                                   # BOOT / RST
        mb.box((ex + bx_, ey + 8.0, zt), (ex + bx_ + 3.6, ey + 11.6, zt + 1.1), "steel_bright", bevel=0.2)
        mb.cyl((ex + bx_ + 1.8, ey + 9.8, zt + 1.1), (ex + bx_ + 1.8, ey + 9.8, zt + 1.7), 1.0, "plastic_black", 12)
    mb.box((ex + 8.0, ey + 15.5, zt), (ex + 14.5, ey + 19.0, zt + 1.6), "ic", bevel=0.2)       # LDO SOT-223
    mb.box((ex + 9.5, ey + 19.0, zt), (ex + 13.0, ey + 20.6, zt + 0.3), "tin")
    mb.box((ex + 16.0, ey + 24.0, zt), (ex + 19.0, ey + 27.0, zt + 0.9), "plastic_white", bevel=0.2)   # RGB LED
    _ic(mb, ex + 4.5, ey + 26.0, ex + 9.5, ey + 31.0, zt, 0.9, "x", 4, 1.0)
    mb.box((ex + 3.0, ey + 13.0, zt), (ex + 4.6, ey + 13.8, zt + 0.5), "led_red")
    _smd_scatter(mb, ex + 3, ex + bw - 4, ey + 12, my0 - 2, zt, 22, 3,
                 avoid=[(ex + 8, ey + 15, ex + 15, ey + 21), (ex + 4, ey + 25, ex + 10, ey + 32)])
    pins_y = [ey + 7.96 + k * 2.54 for k in range(22)]
    rng = np.random.default_rng(11)
    cols = ["red", "black", "yellow", "green", "blue", "orange", "white", "purple", "brown", "grey"]
    for px in (1.27, 24.13):
        x = ex + px
        mb.box((x - 1.27, pins_y[0] - 1.27, z0 - 2.5), (x + 1.27, pins_y[-1] + 1.27, z0), "plastic_black")
        for k, y in enumerate(pins_y):
            mb.box((x - 0.32, y - 0.32, z0 - 8.5), (x + 0.32, y + 0.32, zt + 1.0), "gold")
            mb.cyl((x, y, zt), (x, y, zt + 0.6), 0.75, "tin", 8, r2=0.35)
            if k % 3 == 1 and rng.random() < 0.8:                                             # DuPont + wire
                mb.box((x - 1.25, y - 1.25, z0 - 21.0), (x + 1.25, y + 1.25, z0 - 6.5), "plastic_black", bevel=0.2)
                side = -1 if px < 10 else 1
                p0 = np.array([x, y, z0 - 21.0])
                p1 = np.array([x + side * rng.uniform(6, 14), y + rng.uniform(-10, 10), _BAY["FLOOR"] + 1.0])
                pts = bezier(p0, (0, 0, -8), p1, (side * 6, 0, 0), 10)
                mb.tube(pts, 0.65, "wire_" + cols[rng.integers(0, len(cols))], 6)
    return _mm(mb)


def _p_bts7960():
    """BTS7960 IBT-2 (Harry's is the blue board): two BTS7960 half bridges, 74HC244, 4-way terminal, cap, 8-pin
    header; the finned heatsink under the PCB"""
    mb = MB()
    bx, by = 60.0, 62.0
    sx, sy, st = 49.66, 49.40, 1.6
    z0 = _BAY["BTS_Z"]
    zt = z0 + st
    mb.box((bx, by, z0), (bx + sx, by + sy, zt), "pcb_blue", bevel=0.4)
    for (hx, hy) in [(4.83, 4.7), (44.83, 4.7), (4.83, 44.7), (44.83, 44.7)]:
        mb.cyl((bx + hx, by + hy, zt), (bx + hx, by + hy, zt + 0.05), 2.8, "gold", 16)
        mb.cyl((bx + hx, by + hy, zt + 0.05), (bx + hx, by + hy, zt + 2.0), 2.7, "zinc", 16)
        mb.cyl((bx + hx, by + hy, zt + 2.0), (bx + hx, by + hy, zt + 2.05), 1.2, "plastic_black", 6)
    cx, cy = bx + sx / 2, by + sy / 2
    hsx, hsy, hsz = 36.0, 30.0, 22.0
    mb.box((cx - hsx / 2, cy - hsy / 2, z0 - 4.0), (cx + hsx / 2, cy + hsy / 2, z0 - 0.3), "alu", bevel=0.4)
    nf = 10
    for k in range(nf):
        yy = cy - hsy / 2 + 0.8 + k * (hsy - 1.6) / (nf - 1)
        mb.box((cx - hsx / 2, yy - 0.8, z0 - hsz), (cx + hsx / 2, yy + 0.8, z0 - 4.0), "alu", bevel=0.3)
    mb.box((cx - hsx / 2 + 2, cy - hsy / 2 + 2, z0 - 0.3), (cx + hsx / 2 - 2, cy + hsy / 2 - 2, z0), "ceramic")
    for yy in (by + 10.0, by + 25.0):                                                        # BTS7960 TO-263-7
        mb.box((bx + 18.0, yy, zt), (bx + 28.2, yy + 9.2, zt + 4.4), "ic", bevel=0.4)
        mb.box((bx + 28.2, yy + 0.2, zt), (bx + 30.0, yy + 9.0, zt + 1.3), "tin")
        for k in range(7):
            ly = yy + 0.9 + k * 1.27
            mb.box((bx + 14.8, ly - 0.3, zt), (bx + 18.0, ly + 0.3, zt + 0.5), "tin")
    _ic(mb, bx + 5.5, by + 37.0, bx + 18.3, by + 44.5, zt, 2.4, "x", 10, 1.27)                # 74HC244
    tx0 = bx + sx - 10.0                                                                      # terminal block
    mb.box((tx0, by + 12.0, zt), (bx + sx, by + 34.0, zt + 11.0), "plastic_green", bevel=0.5)
    for k in range(4):
        yy = by + 14.75 + k * 5.5
        mb.cyl((tx0 + 4.5, yy, zt + 11.0), (tx0 + 4.5, yy, zt + 11.3), 1.9, "steel_bright", 16)
        mb.box((tx0 + 3.0, yy - 0.3, zt + 11.25), (tx0 + 6.0, yy + 0.3, zt + 11.35), "plastic_black")
        mb.box((bx + sx - 0.3, yy - 1.8, zt + 2.0), (bx + sx + 0.05, yy + 1.8, zt + 5.6), "plastic_black")
    mb.cyl((bx + sx - 6.0, by + 6.0, zt), (bx + sx - 6.0, by + 6.0, zt + 16.4), 5.0, mat_simple(
        "cap_sleeve", srgb("#1b1b22"), rough=0.35, coat=0.3), 28)                            # 220 uF can
    mb.cyl((bx + sx - 6.0, by + 6.0, zt + 16.4), (bx + sx - 6.0, by + 6.0, zt + 16.9), 4.7, "alu", 28)
    _header(mb, bx + 2.5, by + 15.0, 8, zt, True)
    for k, m in enumerate(("led_red", "led_green")):
        mb.box((bx + 8.0 + k * 3.0, by + 5.0, zt), (bx + 9.6 + k * 3.0, by + 5.8, zt + 0.5), m)
    _smd_scatter(mb, bx + 6, bx + sx - 14, by + 3, by + sy - 3, zt, 18, 7,
                 avoid=[(bx + 14, by + 9, bx + 31, by + 36), (bx + 5, by + 36, bx + 19, by + 45),
                        (bx + 1, by + 13, bx + 4, by + 36)])
    for k, c in enumerate(("red", "black", "red", "black")):                                   # 14 AWG motor / supply
        yy = by + 14.75 + k * 5.5
        p0 = np.array([bx + sx - 1.0, yy, zt + 3.8])
        p1 = np.array([bx + sx + 9.0, yy + (k - 1.5) * 4.0, _BAY["FLOOR"] + 2.0])
        mb.tube(bezier(p0, (6, 0, 0), p1, (0, 0, -6), 10), 1.5, "wire_" + c, 8)
    return _mm(mb)


def _p_small_board(name):
    mb = MB()
    zf = _BAY["FLOOR"]
    if name == "mp1584":
        x, y = 46.0, 40.0
        sx, sy, st = 22.26, 16.96, 1.2
        zt = zf + st
        mb.box((x, y, zf), (x + sx, y + sy, zt), "pcb_green", bevel=0.2)
        mb.box((x + 8.0, y + 5.0, zt), (x + 14.8, y + 11.8, zt + 3.0), "inductor", bevel=0.9)
        _ic(mb, x + 3.0, y + 6.0, x + 7.9, y + 10.0, zt, 1.5, "x", 4, 1.27)
        mb.box((x + 3.0, y + 11.6, zt), (x + 7.5, y + 16.2, zt + 2.6), "plastic_blue", bevel=0.3)
        mb.cyl((x + 5.25, y + 13.9, zt + 2.6), (x + 5.25, y + 13.9, zt + 3.1), 1.3, "brass", 12)
        for (cx_, cy_) in ((16.6, 2.5), (16.6, 12.3), (2.2, 2.0)):
            mb.box((x + cx_, y + cy_, zt), (x + cx_ + 3.2, y + cy_ + 1.6, zt + 1.2), "ceramic")
        pads = [(x + 1.3, y + 1.4), (x + 1.3, y + sy - 1.4), (x + sx - 1.3, y + 1.4), (x + sx - 1.3, y + sy - 1.4)]
        _pads(mb, pads, zt, 1.1)
        wires = [("red", pads[0], (-8, -4)), ("black", pads[1], (-8, 4)), ("red", pads[2], (6, -6)),
                 ("black", pads[3], (6, 6))]
    elif name == "drv8833":
        x, y = 74.0, 40.0
        sx, sy, st = 20.32, 12.70, 1.6
        zt = zf + st
        mb.box((x, y, zf), (x + sx, y + sy, zt), "pcb_green", bevel=0.2)
        _ic(mb, x + 7.6, y + 4.2, x + 12.6, y + 8.6, zt, 1.1, "x", 8, 0.65)
        for (cx_, cy_) in ((4.0, 5.0), (15.0, 5.0), (4.0, 7.2)):
            mb.box((x + cx_, y + cy_, zt), (x + cx_ + 1.6, y + cy_ + 0.8, zt + 0.5), "ceramic")
        pads = [(x + 1.27 + k * 2.54, y + 1.27) for k in range(8)] + [(x + 1.27 + k * 2.54, y + sy - 1.27) for k in range(8)]
        _pads(mb, pads, zt, 0.8)
        wires = [("red", pads[0], (-4, -8)), ("black", pads[1], (-2, -8)), ("blue", pads[3], (0, -8)),
                 ("yellow", pads[4], (2, -8)), ("orange", pads[9], (0, 8)), ("green", pads[10], (2, 8))]
    else:                                                                                      # hx711
        x, y = 16.0, 86.0
        sx, sy, st = 34.0, 23.0, 1.6
        zt = zf + st
        mb.box((x, y, zf), (x + sx, y + sy, zt), "pcb_green", bevel=0.3)
        _ic(mb, x + 12.0, y + 7.0, x + 22.0, y + 11.0, zt, 1.5, "x", 8, 1.27)
        _header(mb, x + 2.0, y + 5.0, 4, zt, True, along="y")
        _smd_scatter(mb, x + 5, x + sx - 3, y + 2, y + sy - 2, zt, 12, 5, avoid=[(x + 11, y + 6, x + 23, y + 12)])
        pads = [(x + sx - 2.0, y + 4.0 + k * 2.54) for k in range(6)]
        _pads(mb, pads, zt, 0.8)
        wires = [(c, pads[k], (10 + k * 2, 18)) for k, c in enumerate(("red", "black", "white", "green"))]
    for c, (px, py), (dx, dy) in wires:
        p0 = np.array([px, py, zt + 0.5])
        p1 = p0 + np.array([dx, dy, 2.0])
        mb.tube(bezier(p0, (0, 0, 3), p1, (dx * 0.3, dy * 0.3, 0), 8), 0.55, "wire_" + c, 6)
    return _mm(mb)


def _p_perfboard():
    mb = MB()
    y0, y1 = 2.4 + 6.0, 2.4 + 6.0 + 1.6
    mb.box((44.0, y0, 24.5), (99.0, y1, 55.5), "pcb_fr4", bevel=0.2)
    for hx in (51.0, 92.0):
        mb.cyl((hx, y1, 40.0), (hx, y1 + 2.2, 40.0), 2.75, "zinc", 16)
    rng = np.random.default_rng(21)
    bands = ["#8a5a1a", "#101010", "#d01818", "#ff8a10", "#d0b010"]
    for k in range(7):
        x = 56.0 + k * 4.5
        z = 30.0 + (k % 2) * 14.0
        mb.cyl((x, y1 + 1.6, z), (x, y1 + 1.6, z + 6.5), 1.1, mat_simple("res_body", srgb("#c8b48a"), rough=0.5), 10)
        for b in range(3):
            mb.cyl((x, y1 + 1.6, z + 1.5 + b * 1.3), (x, y1 + 1.6, z + 2.2 + b * 1.3), 1.15,
                   mat_simple("band_" + bands[(k + b) % 5][1:], srgb(bands[(k + b) % 5]), rough=0.5), 10)
        mb.tube([(x, y1, z - 2.0), (x, y1 + 1.6, z - 1.0), (x, y1 + 1.6, z)], 0.25, "tin", 5)
        mb.tube([(x, y1 + 1.6, z + 6.5), (x, y1 + 1.6, z + 7.5), (x, y1, z + 8.5)], 0.25, "tin", 5)
    for k in range(5):
        c = ["red", "black", "yellow", "white", "blue"][k]
        p0 = np.array([47.0 + k * 3.0, y1, 50.0 + rng.uniform(-3, 3)])
        p1 = np.array([40.0 + k * 9.0, y1 + 25.0, 36.0])
        mb.tube(bezier(p0, (0, 6, 0), p1, (0, 5, -5), 10), 0.6, "wire_" + c, 6)
    return _mm(mb)


def _p_fuse():
    """inline ATO fuse holder (14 AWG red leads), 20 A"""
    mb = MB()
    fx, fy = 46.0, 6.0
    mb.box((fx + 9.0, fy + 3.0, 6.0), (fx + 46.0, fy + 23.0, 19.5), "plastic_black", bevel=3.0, seg=3)
    mb.box((fx + 12.0, fy + 2.8, 12.3), (fx + 43.0, fy + 23.2, 12.8), mat_simple("seam", (0.004, 0.004, 0.004, 1),
                                                                                    rough=0.6))
    for sgn, xa in ((-1, fx + 9.0), (1, fx + 46.0)):
        mb.cyl((xa, fy + 13.0, 12.7), (xa + sgn * 6.0, fy + 13.0, 12.7), 4.5, "rubber", 16, r2=2.6)
        p0 = np.array([xa + sgn * 6.0, fy + 13.0, 12.7])
        p1 = p0 + np.array([sgn * 10.0, 4.0 * sgn, -6.0])
        mb.tube(bezier(p0, (sgn * 5, 0, 0), p1, (sgn * 4, 0, -2), 10), 1.6, "wire_red", 8)
    return _mm(mb)


def _p_rc832():
    """Lockmaner / Boscam RC832 5.8 GHz receiver (80 x 65 x 15) standing antenna-end up in its holder (local frame
    of the CAD envelope x 182.5..247.5, y 152.5..167.5, z 4..84 mm): black anodised case, label, channel LEDs,
    button, SMA with the rubber duck antenna, AV / DC plug below"""
    mb = MB()
    x0, x1, y0, y1, z0, z1 = 182.5, 247.5, 152.5, 167.5, 4.0, 84.0
    mb.box((x0, y0, z0), (x1, y1, z1), "alu_black", bevel=2.2, seg=3)
    lab = mat_label()
    for yy in (y0 - 0.12, y1):
        mb.box((x0 + 12, yy, z0 + 16), (x1 - 12, yy + 0.12, z0 + 44), lab)
    for k in range(8):                                                                       # channel / band LEDs
        xx = x0 + 14 + k * 4.6
        mb.box((xx, y0 - 0.5, z1 - 13), (xx + 2.0, y0, z1 - 11), "led_blue" if k == 3 else "plastic_gloss_black")
    mb.cyl(((x0 + x1) / 2 + 18, y0 + 7.5, z1), ((x0 + x1) / 2 + 18, y0 + 7.5, z1 + 1.6), 2.6, "plastic_black", 16)
    cx, cy = (x0 + x1) / 2 - 8.0, (y0 + y1) / 2
    mb.cyl((cx, cy, z1), (cx, cy, z1 + 2.2), 4.6, "gold", 6)                                   # SMA bulkhead nut
    mb.cyl((cx, cy, z1 + 2.2), (cx, cy, z1 + 8.5), 3.15, "gold", 20)
    mb.cyl((cx, cy, z1 + 8.0), (cx, cy, z1 + 14.0), 4.1, "gold", 6)                            # antenna SMA nut
    mb.cyl((cx, cy, z1 + 14.0), (cx, cy, z1 + 22.0), 4.4, "rubber", 20, r2=4.8)
    mb.cyl((cx, cy, z1 + 22.0), (cx, cy, z1 + 98.0), 4.8, "rubber", 20, r2=4.3)
    mb.cyl((cx, cy, z1 + 98.0), (cx, cy, z1 + 101.0), 4.3, "rubber", 20, r2=2.0)
    mb.box((x1 - 22, cy - 3.5, z0 - 1.0), (x1 - 8, cy + 3.5, z0 + 0.2), "plastic_black", bevel=0.8)
    mb.box((x1 - 9, cy - 3.0, z0 - 0.8), (x1 + 4, cy + 3.0, z0 + 5.2), "plastic_black", bevel=1.2)   # AV / DC plug
    return _mm(mb)


def mat_label():
    """printed white / silver label with dark print (procedural)"""
    key = "label"
    if key in _MATS:
        return _MATS[key]
    m, nt, bsdf = _new_mat(key)
    N = _N(nt)
    obj = N.coord("Object")
    br = N.node("ShaderNodeTexBrick")
    N.link(obj, br.inputs["Vector"])
    br.inputs["Scale"].default_value = 420.0
    br.inputs["Mortar Size"].default_value = 0.3
    br.inputs["Color1"].default_value = (1, 1, 1, 1)
    br.inputs["Color2"].default_value = (0.7, 0.7, 0.7, 1)
    br.inputs["Mortar"].default_value = (0, 0, 0, 1)
    nz = N.noise(150.0, 2, 0.5, obj).outputs["Fac"]
    ink = N.math("MULTIPLY", N.math("LESS_THAN", br.outputs["Fac"], 0.5), N.math("GREATER_THAN", nz, 0.45))
    col = N.mix(ink, (0.82, 0.83, 0.84, 1), (0.02, 0.02, 0.025, 1))
    _set(bsdf, base=col, rough=0.3, metal=0.3)
    return m


def _p_kiyo():
    """Razer Kiyo X (USB webcam) on its clip, looking along +X (local frame of the CAD envelope x 126.4..183.7,
    y -131..-61, z 230.9..296.3 mm)"""
    mb = MB()
    mb.box((146.0, -131.0, 257.0), (182.0, -61.0, 296.3), "plastic_black", bevel=8.0, seg=4)
    mb.box((181.0, -128.0, 260.0), (183.0, -64.0, 293.3), "plastic_gloss_black", bevel=1.0)
    cy, cz = -96.0, 276.6
    mb.cyl((182.8, cy, cz), (184.6, cy, cz), 14.0, "plastic_gloss_black", 40)
    mb.cyl((184.6, cy, cz), (185.0, cy, cz), 11.5, "alu", 40, r2=10.5)
    mb.cyl((183.6, cy, cz), (184.9, cy, cz), 9.6, "plastic_black", 32)
    mb.cyl((184.4, cy, cz), (184.95, cy, cz), 8.2, "lens_glass", 32)
    mb.cyl((183.0, -72.0, 289.0), (183.4, -72.0, 289.0), 1.2, "led_green", 10)
    mb.box((150.0, -110.0, 247.0), (176.0, -82.0, 258.0), "plastic_black", bevel=2.0)          # hinge + clip
    mb.box((170.0, -116.0, 231.0), (176.0, -76.0, 249.0), "plastic_black", bevel=1.5)
    mb.box((128.0, -114.0, 236.0), (151.0, -78.0, 249.0), "plastic_black", bevel=2.0)
    mb.box((169.6, -112.0, 233.0), (170.2, -80.0, 246.0), "rubber")
    p0 = np.array([146.0, -96.0, 272.0])
    mb.cyl(p0, p0 + (-8.0, 0, 0), 3.2, "plastic_black", 16, r2=2.2)
    return _mm(mb)


def _procedural_base_parts():
    return {
        "frame_pcb_esp32": _p_esp32,
        "frame_pcb_bts7960": _p_bts7960,
        "frame_pcb_mp1584": lambda: _p_small_board("mp1584"),
        "frame_pcb_drv8833": lambda: _p_small_board("drv8833"),
        "frame_pcb_hx711": lambda: _p_small_board("hx711"),
        "frame_perfboard": _p_perfboard,
        "frame_fuse_20a": _p_fuse,
        "frame_rc832": _p_rc832,
        "frame_kiyo_x": _p_kiyo,
        "launch_spring_l_hook_c": lambda: _hook_mesh(1),
        "launch_spring_r_hook_c": lambda: _hook_mesh(1),
        "launch_spring_l_hook_a": lambda: _hook_mesh(-1),
        "launch_spring_r_hook_a": lambda: _hook_mesh(-1),
    }


# --- dart electronics (dart frame, kedge/out34 envelopes, Harry's caliper photos)
def _d_camera():
    """Caddx Ant: black housing, M8 lens barrel with glass, black PCB, white JST at the back"""
    mb = MB()
    zc = -2.4
    mb.box((4.0, -7.0, -9.7), (15.6, 7.0, 4.9), "plastic_black", bevel=0.8)
    mb.box((15.6, -7.0, -9.7), (17.2, 7.0, 4.9), "pcb_black")
    mb.box((17.2, -3.0, -6.0), (19.5, 3.0, -1.0), "plastic_white", bevel=0.3)
    mb.cyl((1.2, 0, zc), (4.0, 0, zc), 4.0, "plastic_black", 28)
    mb.cyl((1.25, 0, zc), (1.45, 0, zc), 3.0, "lens_glass", 28)
    mb.cyl((1.15, 0, zc), (1.3, 0, zc), 4.0, "alu_black", 28, r2=3.1)
    return _mm(mb)


def _d_board(name):
    mb = MB()
    if name == "xiao":                                    # XIAO ESP32S3: dark PCB, RF shield, USB-C to -y
        mb.box((43.0, -19.0, 1.0), (60.7, 2.5, 2.2), "pcb_black", bevel=0.4)
        mb.box((45.0, -12.5, 2.2), (58.7, 1.0, 4.6), "tin", bevel=0.3)
        mb.box((47.4, -20.3, 2.2), (56.3, -13.0, 5.4), "steel_bright", bevel=1.4, seg=3)
        _pads(mb, [(43.6, -16.0 + k * 2.54) for k in range(7)] + [(60.1, -16.0 + k * 2.54) for k in range(7)], 2.2, 0.7)
        mb.box((44.5, -18.0, 2.2), (46.0, -16.5, 2.8), "plastic_white")
    elif name.startswith("drv8833"):                     # standing boards, components toward the centre line
        s = 1 if name.endswith("1") else -1
        yo = 11.1 if s > 0 else -11.1
        mb.box((125.7, min(yo, yo - s * 1.6), -14.6), (138.4, max(yo, yo - s * 1.6), 5.7), "pcb_green", bevel=0.2)
        yf = yo - s * 1.6
        mb.box((129.6, min(yf, yf - s * 1.1), -6.5), (134.6, max(yf, yf - s * 1.1), -2.0), "ic")
        for zz in (-11.0, -9.0, 1.5):
            mb.box((127.0, min(yf, yf - s * 0.5), zz), (128.6, max(yf, yf - s * 0.5), zz + 0.8), "ceramic")
        for k in range(8):
            for xx in (126.5, 137.6):
                mb.box((xx - 0.6, min(yo, yf) - 0.05, -13.5 + k * 2.54), (xx + 0.6, max(yo, yf) + 0.05,
                                                                       -12.7 + k * 2.54), "gold")
    elif name == "ina219":
        mb.box((61.7, -17.6, 1.0), (87.4, 2.7, 2.6), "pcb_black", bevel=0.4)
        mb.box((70.0, -10.0, 2.6), (76.4, -6.8, 3.4), "ic")
        mb.box((66.0, -4.0, 2.6), (72.4, -0.8, 3.2), mat_simple("shunt", srgb("#202020"), rough=0.4))
        for xx in (63.0, 82.0):
            mb.box((xx, -15.5, 2.6), (xx + 3.5, -10.0, 5.6), "plastic_white", bevel=0.2)
        _pads(mb, [(64.0 + k * 2.54, 1.3) for k in range(6)], 2.6, 0.75)
    elif name == "bno085":
        mb.box((89.8, -12.9, 1.0), (112.6, 12.9, 2.6), "pcb_black", bevel=0.5)
        mb.box((99.4, -2.6, 2.6), (103.2, 2.6, 3.6), "ic")
        for yy in (-11.0, 7.5):
            mb.box((91.0, yy, 2.6), (95.0, yy + 3.5, 5.8), "plastic_white", bevel=0.2)
        _pads(mb, [(91.5 + k * 2.54, -11.6) for k in range(8)] + [(91.5 + k * 2.54, 11.6) for k in range(8)], 2.6, 0.75)
        _smd_scatter(mb, 96, 110, -9, 9, 2.6, 10, 9, avoid=[(99, -3, 104, 3)])
    elif name == "mp1584":
        mb.box((92.2, -8.5, 7.2), (114.5, 8.5, 8.4), "pcb_green", bevel=0.2)
        mb.box((100.0, -3.4, 8.4), (106.8, 3.4, 11.2), "inductor", bevel=0.9)
        _ic(mb, 94.5, -1.9, 98.5, 2.1, 8.4, 1.4, "y", 4, 1.27)
        mb.box((108.0, 2.0, 8.4), (111.2, 3.6, 9.6), "ceramic")
        mb.box((108.0, -3.6, 8.4), (111.2, -2.0, 9.6), "ceramic")
        _pads(mb, [(93.4, -7.3), (93.4, 7.3), (113.3, -7.3), (113.3, 7.3)], 8.4, 1.0)
    elif name == "vtx":                                   # Zeus nano VTX: red PCB, shield with label, u.FL
        mb.box((49.0, -10.0, 7.2), (91.2, 10.0, 8.2), "pcb_red", bevel=1.5)
        mb.box((56.0, -7.5, 8.2), (76.0, 7.5, 11.2), "tin", bevel=0.4)
        mb.box((57.5, -6.0, 11.2), (74.5, 6.0, 11.25), mat_simple("zeus_label", srgb("#d42a2a"), rough=0.35))
        mb.cyl((84.0, 5.0, 8.2), (84.0, 5.0, 9.4), 1.3, "gold", 12)
        mb.box((80.0, -6.0, 8.2), (83.0, -3.0, 9.6), "plastic_black")
        mb.box((86.0, -3.0, 8.2), (87.6, -2.2, 8.8), "led_red")
        _pads(mb, [(51.0, -8.0), (51.0, 8.0), (89.2, -8.0), (89.2, 8.0)], 8.2, 1.4)
        _smd_scatter(mb, 77, 89, -8, 8, 8.2, 8, 13, avoid=[(79.5, -6.5, 85.5, 6.5)])
    elif name == "bms_2s":
        mb.box((46.0, 3.2, 1.0), (86.8, 11.2, 2.0), "pcb_blue", bevel=0.3)
        for xx in (58.0, 66.0):
            mb.box((xx, 4.6, 2.0), (xx + 5.0, 9.8, 3.1), "ic", bevel=0.2)
        _ic(mb, 74.0, 5.5, 77.0, 8.9, 2.0, 1.0, "x", 3, 0.95)
        for xx in (47.5, 85.0):
            mb.box((xx - 1.2, 4.0, 2.0), (xx + 1.2, 10.4, 2.1), "gold")
    return _mm(mb)


def _procedural_dart_parts():
    d = {"camera": _d_camera}
    for n in ("xiao", "drv8833_1", "drv8833_2", "ina219", "bno085", "mp1584", "vtx", "bms_2s"):
        d[n] = (lambda nn: (lambda: _d_board(nn)))(n)
    return d


# ============================================================================================== board
def build_board(scene, col=None):
    """birch plywood board, top face at z 0, centred on the world origin"""
    col = col or get_collection("BASE", scene)
    sx, sy, t = BOARD_SIZE
    mb = MB()
    mb.box((-sx / 2, -sy / 2, -t), (sx / 2, sy / 2, 0.0), "ply", bevel=0.0012, seg=2)
    ob = mb.obj("base:plywood_board", col, smooth=40.0)
    ob["note"] = "birch plywood 1000 x 800 x 18 mm, top at z 0"
    return ob


# ============================================================================================== top level
def build_models(scene=None):
    """build base (+ board), dart and electronics at rest; returns {'base': base:root, 'dart': dart:root,
    'elec': elec:root, 'board': board object}"""
    scene = scene or bpy.context.scene
    t0 = time.time()
    out = {"board": build_board(scene, get_collection("BASE", scene))}
    out["base"] = build_base(scene, get_collection("BASE", scene))
    out["dart"] = build_dart(scene, get_collection("DART", scene))
    out["elec"] = build_elec(scene, get_collection("ELEC", scene))
    bpy.context.view_layer.update()
    print(f"models built in {time.time() - t0:.1f} s: {stats()}")
    return out


def stats():
    """triangle counts per collection (evaluated, so the spring coils count)"""
    dg = bpy.context.evaluated_depsgraph_get()
    res = {}
    for cn in ("BASE", "DART", "ELEC"):
        col = bpy.data.collections.get(cn)
        if col is None:
            continue
        tri = 0
        for ob in col.all_objects:
            if ob.type not in ("MESH", "CURVE"):
                continue
            ev = ob.evaluated_get(dg)
            try:
                me = ev.to_mesh()
            except Exception:
                continue
            if me is not None:
                me.calc_loop_triangles()
                tri += len(me.loop_triangles)
                ev.to_mesh_clear()
        res[cn] = tri
    return res


# ============================================================================================== electronics on the board
# placements (world, m): the Jetson on the board's -y strip with its I/O edge facing the base (+y); the 3S pack on
# the +y strip; cables to the bay (ESP32 USB on the bay's front wall, battery lead through its +y wall), the RC832
# (in its holder on the deck) and the Kiyo X (on its mount).
ELEC_PLACE = {
    "elec:jetson": (0.085, -0.31, 0.0, 0.0),            # x, y, z, yaw deg (board origin = carrier centre on its feet)
    "elec:lipo_3s": (-0.02, 0.29, 0.0, 0.0),
}
JET_Z = 5.0                 # carrier underside above the board (tray + feet), mm
JET_T = 6.6                 # carrier top, mm


JETSON_NPZ = os.path.join(HERE, "assets", "models", "jetson_orin_nano_devkit.npz")
# kit frame of NVIDIA's CAD (carrier x -4..96, y -17..62, board top z 1.6, I/O edge at y -17) -> elec:jetson local
# frame (carrier centre, I/O edge toward +y, tray underside on z 0): rotate 180 deg about z, shift
JET_KIT_C = (46.0, 22.5, -4.9)


def _kit2local(p):
    return (JET_KIT_C[0] - p[0], JET_KIT_C[1] - p[1], p[2] - JET_KIT_C[2])


def _usb_a_ports():
    """Jetson local (mm): port face centres on the I/O edge (+y), {name: (x, y, z)}"""
    if os.path.exists(JETSON_NPZ):                    # NVIDIA CAD positions (kit frame), see _e_jetson_cad
        k = {"usb1_low": (38.6, -18.7, 6.3), "usb1_up": (38.6, -18.7, 14.1), "usb2_low": (55.6, -18.7, 6.3),
             "usb2_up": (55.6, -18.7, 14.1), "dc": (2.5, -18.5, 7.6), "rj45": (73.0, -18.7, 7.5),
             "usbc": (88.0, -18.7, 3.2)}
        return {n: _kit2local(q) for n, q in k.items()}
    return {"usb1_low": (-6.75, 41.0, JET_T + 4.6), "usb1_up": (-6.75, 41.0, JET_T + 12.4),
            "usb2_low": (11.25, 41.0, JET_T + 4.6), "usb2_up": (11.25, 41.0, JET_T + 12.4),
            "dc": (-41.5, 41.0, JET_T + 6.3), "rj45": (29.0, 41.0, JET_T + 6.8), "usbc": (44.5, 41.0, JET_T + 1.7)}


def _jet_part_material(name):
    """material key + optional face split rule for a leaf of NVIDIA's dev kit CAD (by part number / name)"""
    n = name
    if n.startswith("375-0137"):
        return "plastic_black", None                         # tray
    if n.startswith("600-13768") or n.startswith("GROGU_MODULE"):
        return "pcb_black", None                             # carrier / module boards
    if n.startswith("095-0180"):
        return "alu_black", None                             # finned heatsink
    if n.startswith("3514"):
        return "plastic_black", None                         # fan frame + impeller
    if n.startswith("WIRE_FAN") or n.startswith("COAXIAL"):
        return "wire_black", None
    if n.startswith("LEAFSPRING") or n.startswith("155-") or n.startswith("M1_6"):
        return "black_oxide", None                           # spring plate, standoffs, screws
    if n.startswith(("065-0738", "064-0404", "080-1898", "080-1890", "080-1263", "320-1004")):
        return "steel_bright", None                          # USB A stacks, RJ45, DP, USB-C, u.FL
    if n.startswith(("074-0221", "074-0239")):
        return "plastic_black", ("z_above", 4.3, "gold")     # headers: pins above the housing
    if n.startswith(("080-1656", "080-1243", "080-1483", "080-2191", "064-0437")):
        return "plastic_black", None                         # SO-DIMM, M.2 sockets, FPC, DC jack
    if n.startswith("080-1655"):
        return "gold", None                                  # module edge fingers
    if n.startswith(("WIFI_ANTENNA", "142-13526")):
        return "plastic_black", None
    if n.startswith("330-0266"):
        return "pcb_green", None                             # Wi-Fi card
    if n.startswith(("LOGO_LABEL", "LL_VENDOR")):
        return "label", None
    if n.startswith("LOGO_SILK"):
        return "plastic_white", None
    if n.startswith(("FID_", "VSB_", "VTB_", "VST_", "LTPT")):
        return "gold", None
    if n.startswith(("0402_", "0603_", "0805_", "1206_", "R0201", "C0201")):
        cap = n.startswith("C0201") or "H060" in n or "H100" in n or "H140" in n or "H150" in n or "H160" in n
        return ("ceramic" if cap else "ic"), ("ends", 0.22, "tin")
    if n.startswith(("032-", "130-1404")):
        return "inductor", None
    if n.startswith("064-0267"):
        return "plastic_white", None
    return "ic", None


def _e_jetson_cad():
    """elec:jetson from NVIDIA's Orin Nano dev kit CAD (render/assets/models/jetson_orin_nano_devkit.npz):
    returns (verts m, tris, mat index, mats) in the local frame, or None when the npz is missing"""
    if not os.path.exists(JETSON_NPZ):
        return None
    z = np.load(JETSON_NPZ)
    names, v, f, vo, fo = z["names"], z["v"].astype(np.float64), z["f"].astype(np.int64), z["voff"], z["foff"]
    if "global_idx" not in z.files:                          # per-leaf vertex indices -> global
        f += np.repeat(vo[:-1], np.diff(fo))[:, None]
    keys, mi = [], np.zeros(len(f), np.int32)

    def slot(k):
        if k not in keys:
            keys.append(k)
        return keys.index(k)
    for i, n in enumerate(names):
        n = str(n)
        if n.startswith("MECH_RECT"):
            mi[fo[i]:fo[i + 1]] = -1
            continue
        key, rule = _jet_part_material(n)
        mi[fo[i]:fo[i + 1]] = slot(key)
        if rule is None:
            continue
        ff = f[fo[i]:fo[i + 1]]
        cen = v[ff].mean(1)
        if rule[0] == "z_above":
            sel = (cen[:, 2] > rule[1]) | (cen[:, 2] < -0.2)
        else:                                                # passive terminals: both ends of the long axis
            vv = v[vo[i]:vo[i + 1]]
            lo, hi = vv.min(0), vv.max(0)
            ax = int(np.argmax((hi - lo)[:2]))
            u = (cen[:, ax] - lo[ax]) / max(hi[ax] - lo[ax], 1e-9)
            sel = (u < rule[1]) | (u > 1 - rule[1])
        sub = mi[fo[i]:fo[i + 1]]
        sub[sel] = slot(rule[2])
    keep = mi >= 0
    f, mi = f[keep], mi[keep]
    loc = np.c_[JET_KIT_C[0] - v[:, 0], JET_KIT_C[1] - v[:, 1], v[:, 2] - JET_KIT_C[2]] * MM
    mats = [mat_label() if k == "label" else M(k) for k in keys]
    return loc, f, mi, mats


def _e_jetson():
    """Jetson Orin Nano developer kit (Harry's photos): matte black carrier (100 x 79) on its black plastic tray
    with corner feet and an open underside (M.2 Wi-Fi card + antenna leads), black module in the SO-DIMM socket
    under a black finned heatsink (fins open toward the FPC edge) with a 40 mm black fan, NVIDIA logo on the side;
    I/O edge (+y): DC barrel, DisplayPort, 2 x 2 USB A (silver), RJ45, USB-C; 40-pin header along the +x edge, two
    FPC camera connectors on the -x edge. Local frame: carrier centre, z = board top (the plywood)."""
    mb = MB()
    zt = JET_T
    zb = JET_Z
    # tray: rim frame, corner bosses with feet, open underside
    X, Y, w = 51.5, 41.0, 2.4
    for (x0, x1, y0, y1) in ((-X, X, -Y, -Y + w), (-X, X, Y - w, Y), (-X, -X + w, -Y, Y), (X - w, X, -Y, Y)):
        mb.box((x0, y0, 1.2), (x1, y1, zb + 0.8), "plastic_black", bevel=0.6)
    mb.box((-X + w, -6.0, 1.2), (X - w, -3.0, 2.6), "plastic_black")
    for sx_ in (-1, 1):
        for sy_ in (-1, 1):
            x, y = sx_ * 45.5, sy_ * 35.0
            mb.cyl((x, y, 1.2), (x, y, zb), 3.6, "plastic_black", 20)
            mb.cyl((sx_ * 48.0, sy_ * 37.5, 0.0), (sx_ * 48.0, sy_ * 37.5, 1.3), 3.0, "rubber", 16)
            mb.cyl((x, y, zt), (x, y, zt + 1.4), 2.6, "black_oxide", 16)
            mb.box((x - 1.6, y - 0.3, zt + 1.3), (x + 1.6, y + 0.3, zt + 1.45), "plastic_black")
    mb.box((-50, -39.5, zb), (50, 39.5, zt), "pcb_black", bevel=0.5)
    # underside: Wi-Fi card and its two antenna leads
    mb.box((-12.0, -30.0, zb - 2.6), (10.0, 0.0, zb - 1.0), "pcb_green")
    mb.box((-10.0, -24.0, zb - 3.6), (8.0, -6.0, zb - 2.6), "tin")
    for k in range(2):
        mb.tube(bezier((-6.0 + 8 * k, -2.0, zb - 2.2), (0, 6, -0.5), (-30.0 + 6 * k, 30.0, zb - 1.0), (-6, 4, 0), 12),
                0.55, "wire_black", 5)
    # module in the SO-DIMM socket
    mx0, mx1, my0, my1 = -38.0, 31.6, -36.5, 8.5
    mb.box((mx0 - 1.5, my1 - 2.5, zt), (mx1 + 1.5, my1 + 4.0, zt + 4.6), "plastic_black", bevel=0.3)
    mb.box((mx0, my0, zt + 3.4), (mx1, my1, zt + 4.4), "pcb_black")
    for (x0, y0, x1, y1) in ((14.0, -10.0, 22.0, -2.0), (24.0, -30.0, 30.0, -20.0), (14.0, -32.0, 21.0, -26.0)):
        mb.box((x0, y0, zt + 4.4), (x1, y1, zt + 5.4), "ic", bevel=0.2)
    _smd_scatter(mb, 13.0, 30.0, -34.0, 6.0, zt + 4.4, 26, 41, avoid=[(13.5, -10.5, 22.5, -1.5)])
    hx0, hx1, hy0, hy1 = -38.5, 11.5, -37.0, 5.5
    hz0 = zt + 4.4
    mb.box((hx0, hy0, hz0), (hx1, hy1, hz0 + 3.2), "alu_black", bevel=0.5)
    nf = 15
    for k in range(nf):
        y = hy0 + 1.0 + k * (hy1 - hy0 - 2.0) / (nf - 1)
        mb.box((hx0, y - 0.6, hz0 + 3.2), (hx1, y + 0.6, hz0 + 11.5), "alu_black", bevel=0.25)
    mb.box((hx0 + 9.0, hy1 - 0.2, hz0 + 7.5), (hx0 + 23.0, hy1 + 0.05, hz0 + 10.5), mat_simple(
        "logo_grey", (0.28, 0.29, 0.3, 1), rough=0.4, metal=0.5))                                   # NVIDIA logo
    fz = hz0 + 11.5
    fcx, fcy = (hx0 + hx1) / 2, (hy0 + hy1) / 2
    plate_hole(mb, fcx - 20.0, fcx + 20.0, fcy - 20.0, fcy + 20.0, fz, fz + 1.2, fcx, fcy, 19.0, "plastic_black")
    plate_hole(mb, fcx - 20.0, fcx + 20.0, fcy - 20.0, fcy + 20.0, fz + 8.8, fz + 10.0, fcx, fcy, 19.0, "plastic_black")
    for sx_ in (-1, 1):                                                           # fan frame corner pillars
        for sy_ in (-1, 1):
            mb.box((fcx + sx_ * 20.0 - (3.0 if sx_ > 0 else 0), fcy + sy_ * 20.0 - (3.0 if sy_ > 0 else 0), fz),
                   (fcx + sx_ * 20.0 + (3.0 if sx_ < 0 else 0), fcy + sy_ * 20.0 + (3.0 if sy_ < 0 else 0), fz + 10.0),
                   "plastic_black", bevel=0.4)
            mb.cyl((fcx + sx_ * 16.0, fcy + sy_ * 16.0, fz + 10.0), (fcx + sx_ * 16.0, fcy + sy_ * 16.0, fz + 10.1),
                   1.6, mat_simple("hole", (0.003,) * 3 + (1,), rough=0.9, plain=True), 12)
    a = np.linspace(0, 2 * math.pi, 48)
    ring = np.c_[fcx + 19.4 * np.cos(a), fcy + 19.4 * np.sin(a), np.full_like(a, fz + 5.0)]
    for dz in (-3.5, 0.0, 3.5):
        mb.tube(ring + (0, 0, dz), 0.8, "plastic_black", 6, closed=True)
    mb.cyl((fcx, fcy, fz + 1.0), (fcx, fcy, fz + 9.6), 10.5, "plastic_black", 32)                   # hub
    for k in range(9):                                                                             # blades
        ang = 2 * math.pi * k / 9
        blade = np.array([[10.0, -3.5, -2.5], [18.8, -6.5, -1.8], [18.8, 4.5, 2.6], [10.0, 3.5, 2.0]])
        R = np.array([[math.cos(ang), -math.sin(ang), 0], [math.sin(ang), math.cos(ang), 0], [0, 0, 1]])
        v = blade @ R.T + (fcx, fcy, fz + 5.5)
        mb.add(np.vstack([v, v + (0, 0, 0.7)]), [[0, 1, 2, 3], [7, 6, 5, 4], [0, 4, 5, 1], [1, 5, 6, 2],
                                                 [2, 6, 7, 3], [3, 7, 4, 0]], "plastic_black")
    mb.tube(bezier((fcx + 20.0, fcy - 14.0, fz + 3.0), (6, 0, 0), (36.0, -14.0, zt + 1.0), (0, 0, -6), 10), 0.5,
            "wire_black", 5)
    mb.box((34.0, -16.0, zt), (38.0, -12.0, zt + 3.0), "plastic_white", bevel=0.2)
    # I/O edge (+y)
    P = _usb_a_ports()
    hole = mat_simple("hole", (0.003,) * 3 + (1,), rough=0.9, plain=True)
    x = P["dc"][0]
    mb.box((x - 4.5, 27.0, zt), (x + 4.5, 40.5, zt + 11.0), "plastic_black", bevel=0.6)
    mb.cyl((x, 40.5, P["dc"][2]), (x, 40.6, P["dc"][2]), 3.2, hole, 20)
    mb.cyl((x, 40.0, P["dc"][2]), (x, 40.65, P["dc"][2]), 1.0, "steel_bright", 12)
    mb.box((-34.0, 26.0, zt), (-18.0, 41.0, zt + 6.6), "steel_bright", bevel=0.8)                   # DP
    mb.box((-32.5, 40.9, zt + 1.2), (-19.5, 41.1, zt + 5.4), "plastic_black", bevel=0.8)
    for xc in (P["usb1_up"][0], P["usb2_up"][0]):                                                  # USB A stacks
        mb.box((xc - 7.4, 24.0, zt), (xc + 7.4, 41.0, zt + 16.0), "steel_bright", bevel=0.5)
        for zc in (zt + 4.6, zt + 12.4):
            mb.box((xc - 6.2, 40.9, zc - 2.7), (xc + 6.2, 41.1, zc + 2.7), hole)
            mb.box((xc - 5.6, 38.0, zc - 0.4), (xc + 5.6, 41.05, zc + 1.4), "plastic_black")
    x = P["rj45"][0]
    mb.box((x - 8.0, 20.0, zt), (x + 8.0, 41.5, zt + 13.6), "steel_bright", bevel=0.5)
    mb.box((x - 6.0, 41.4, zt + 2.0), (x + 6.0, 41.6, zt + 10.0), hole)
    mb.box((x - 7.4, 41.45, zt + 10.6), (x - 4.8, 41.7, zt + 12.6), "led_green")
    mb.box((x + 4.8, 41.45, zt + 10.6), (x + 7.4, 41.7, zt + 12.6), mat_simple("led_amber", srgb("#ffb020"),
                                                                                emit=srgb("#ffb020"), emit_s=1.0))
    x = P["usbc"][0]
    mb.box((x - 4.5, 33.5, zt), (x + 4.5, 41.0, zt + 3.3), "steel_bright", bevel=1.5, seg=3)
    # 40-pin header along the +x edge, button header, FPC camera connectors on the -x edge
    for row in (0, 1):
        _header(mb, 45.5 + row * 2.54, -26.0, 20, zt, True, along="y")
    _header(mb, 38.0, -34.0, 6, zt, True, along="x")
    for yy in (-26.0, -8.0):
        mb.box((-49.0, yy, zt), (-44.0, yy + 15.0, zt + 2.2), "plastic_black", bevel=0.2)
        mb.box((-49.2, yy + 0.8, zt + 1.4), (-47.0, yy + 14.2, zt + 2.4), mat_simple("fpc_lock", srgb("#3a3a3a"),
                                                                                     rough=0.5))
    _smd_scatter(mb, -46, 46, 12, 22, zt, 46, 31, avoid=[(-47, 19, 48, 42), (-40, 5, 34, 14)])
    _smd_scatter(mb, -43, -36, -36, 10, zt, 18, 33)
    _smd_scatter(mb, 33, 43, -24, 8, zt, 14, 32)
    for (x0, y0, x1, y1) in ((-26, 14, -16, 21), (37, 12, 44, 20), (2, 15, 10, 22)):
        _ic(mb, x0, y0, x1, y1, zt, 1.0, "x", 5, 1.6)
    return _mm(mb)


def _e_usb_a_plug(overmold="usb", length=18.0):
    """USB A plug: metal shell (inserted 9 mm) + black overmold toward +y; local origin at the port face centre"""
    mb = MB()
    mb.box((-6.0, -9.0, -2.25), (6.0, 0.2, 2.25), "steel_bright", bevel=0.3)
    mb.box((-7.6, 0.0, -4.0), (7.6, length, 4.0), "plastic_black", bevel=2.0, seg=3)
    mb.cyl((0, length, 0), (0, length + 7.0, 0), 3.2, "plastic_black", 16, r2=2.3)
    return _mm(mb)


def _e_digitnow():
    """DIGITNOW USB 2.0 video capture stick: gloss black body on a USB A plug (local origin = port face centre,
    body toward +y), strain relief and cable stub"""
    mb = MB()
    mb.box((-6.0, -9.0, -2.25), (6.0, 0.2, 2.25), "steel_bright", bevel=0.3)
    mb.box((-12.0, 0.0, -5.5), (12.0, 58.0, 5.5), "plastic_gloss_black", bevel=3.0, seg=3)
    mb.box((-9.0, 18.0, 5.45), (9.0, 46.0, 5.55), mat_label())
    mb.box((7.0, 52.0, 5.4), (9.0, 54.0, 5.7), "led_blue")
    mb.cyl((0, 58.0, 0), (0, 68.0, 0), 3.6, "plastic_black", 16, r2=2.4)
    return _mm(mb)


def _e_av_breakout():
    """the DIGITNOW's AV fan-out: three RCA jacks (yellow / white / red) and the S-video jack on short leads; local
    origin at the cable entry, jacks toward +x"""
    mb = MB()
    mb.box((-4.0, -5.0, 0.0), (8.0, 5.0, 8.0), "plastic_black", bevel=2.0, seg=3)
    cols = (("yellow", srgb("#e8c010")), ("white", srgb("#e8e8e4")), ("red", srgb("#c81818")))
    for k, (nm, c) in enumerate(cols):
        y = (k - 1.5) * 11.0
        p0 = np.array([8.0, (k - 1.5) * 2.0, 4.0])
        p1 = np.array([34.0, y, 4.5])
        mb.tube(bezier(p0, (8, 0, 0), p1, (8, 0, 0), 10), 1.6, "wire_" + nm, 8)
        mb.cyl(p1, p1 + (14.0, 0, 0), 4.6, mat_simple("rca_" + nm, c, rough=0.4), 20)
        mb.cyl(p1 + (14.0, 0, 0), p1 + (22.0, 0, 0), 4.2, "steel_bright", 20)
        mb.cyl(p1 + (21.9, 0, 0), p1 + (22.05, 0, 0), 1.6, mat_simple("hole", (0.003,) * 3 + (1,), rough=0.9, plain=True), 12)
    p0 = np.array([8.0, 3.5, 4.0])
    p1 = np.array([30.0, 22.0, 4.5])
    mb.tube(bezier(p0, (8, 0, 0), p1, (8, 0, 0), 10), 1.8, "wire_black", 8)
    mb.cyl(p1, p1 + (16.0, 0, 0), 5.0, "plastic_black", 20)
    return _mm(mb)


def _e_xt60(male=True):
    """XT60 (yellow nylon) along +x from the origin: male = pins sticking out at +x; leads leave at -x"""
    mb = MB()
    mb.box((0.0, -7.85, -4.05), (16.0, 7.85, 4.05), "nylon_yellow", bevel=1.0)
    mb.box((14.0, -7.85, 2.0), (16.0, -4.0, 4.05), "nylon_yellow")
    for y in (-3.6, 3.6):
        if male:
            mb.cyl((16.0, y, 0), (22.0, y, 0), 1.75, "gold", 16)
        else:
            mb.cyl((15.95, y, 0), (16.05, y, 0), 2.4, mat_simple("hole", (0.003,) * 3 + (1,), rough=0.9, plain=True), 16)
        mb.cyl((-6.0, y, 0), (0.0, y, 0), 2.6, mat_simple("shrink_" + ("red" if y > 0 else "black"),
                                                            srgb("#b01010" if y > 0 else "#101010"), rough=0.5), 16)
    return _mm(mb)


def _e_lipo():
    """3S 2200 mAh LiPo (106 x 34 x 25), XT60 lead (12 AWG silicone) from the +x end, balance lead + JST-XH 4P"""
    mb = MB()
    mb.box((-53.0, -17.0, 0.0), (53.0, 17.0, 25.0), mat_lipo_wrap(), bevel=3.0, seg=3)
    mb.box((53.0, -12.0, 4.0), (55.5, 12.0, 21.0), mat_simple("tape", srgb("#d0d0c8"), rough=0.6), bevel=0.8)
    for k, (y, c) in enumerate(((5.0, "red"), (-1.0, "black"))):
        p0 = np.array([55.5, y, 14.0])
        p1 = np.array([118.0, 3.6 if c == "red" else -3.6, 4.05])
        mb.tube(bezier(p0, (25, 0, 0), p1, (25, 0, 0), 18), 1.8, mat_wire_si(c), 10)
    for k, c in enumerate(("black", "red", "red", "red")):
        p0 = np.array([55.5, -8.0 + k * 1.4, 8.0])
        p1 = np.array([82.0, -26.0 + k * 1.3, 1.0])
        mb.tube(bezier(p0, (10, 0, 0), p1, (12, -4, 0), 14), 0.6, mat_wire_si(c), 6)
    mb.box((82.0, -27.0, 0.0), (88.0, -20.0, 5.6), "plastic_white", bevel=0.4)
    return _mm(mb)


def mat_wire_si(color):
    """soft silicone wire insulation (LiPo leads)"""
    return mat_simple("si_" + color, srgb({"red": "#c01010", "black": "#121212"}.get(color, color)), rough=0.62,
                      spec=0.4, sheen=0.3)


def _cable(name, pts, r, mat, col, res=10):
    """smooth cable (Bezier curve through world points, auto handles, round bevel)"""
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth = r
    cu.bevel_resolution = 3
    cu.resolution_u = res
    cu.use_fill_caps = True
    sp = cu.splines.new("BEZIER")
    pts = [Vector(p) for p in pts]
    sp.bezier_points.add(len(pts) - 1)
    for bp, p in zip(sp.bezier_points, pts):
        bp.co = p
        bp.handle_left_type = bp.handle_right_type = "AUTO"
    if isinstance(mat, str):
        mat = M(mat)
    cu.materials.append(mat)
    ob = bpy.data.objects.get(name)
    if ob is not None:
        bpy.data.objects.remove(ob, do_unlink=True)
    ob = bpy.data.objects.new(name, cu)
    col.objects.link(ob)
    return ob


def _pair(name, pts, r, cols, col, gap=None):
    """two-wire lead (red / black) side by side: offset the path sideways in the horizontal plane"""
    pts = np.asarray(pts, float)
    gap = gap or 2.1 * r
    t = np.gradient(pts, axis=0)
    side = np.cross(t, [0, 0, 1.0])
    side /= np.linalg.norm(side, axis=1, keepdims=True) + 1e-12
    return [_cable(f"{name}_{c}", pts + s * side * gap / 2, r, w, col)
            for s, (c, w) in zip((1, -1), ((cols[0], mat_wire_si(cols[0])), (cols[1], mat_wire_si(cols[1]))))]


def _b2w(p):
    """base frame mm -> world m"""
    B = np.asarray(BASE_TO_WORLD)
    return (B[:3, :3] @ (np.asarray(p, float) * MM) + B[:3, 3]).tolist()


def build_elec(scene, col=None):
    """Harry's kit on the board and the cabling; returns elec:root (an empty at the origin)"""
    col = col or get_collection("ELEC", scene)
    root = empty("elec:root", col, size=0.05)
    jx, jy, jz, jyaw = ELEC_PLACE["elec:jetson"]
    JET = T_(jx, jy, jz) @ R_("Z", jyaw)
    cad = _e_jetson_cad()
    if cad is not None:
        v, f, mi, mats = cad
        me = mesh_np("elec:jetson", v, f, mi, smooth_angle=35.0)
        ob = mesh_object("elec:jetson", me, col, root, JET, mats)
        ob["source"] = "NVIDIA Orin Nano dev kit CAD via Printables 555369 (CC0), render/assets/models"
    else:
        _e_jetson().obj("elec:jetson", col, root, JET)
    P = {k: (JET @ np.r_[np.asarray(v) * MM, 1.0])[:3] for k, v in _usb_a_ports().items()}
    _e_digitnow().obj("elec:digitnow", col, root, T_(*P["usb2_up"]) @ R_("Z", jyaw))
    _e_usb_a_plug().obj("elec:plug_esp32_usb", col, root, T_(*P["usb1_low"]) @ R_("Z", jyaw))
    _e_usb_a_plug().obj("elec:plug_kiyo_usb", col, root, T_(*P["usb1_up"]) @ R_("Z", jyaw))
    mb = MB()                                                                                   # DC barrel plug
    mb.cyl((0, -9.5, 0), (0, 0.1, 0), 2.7, "steel_bright", 16)
    mb.cyl((0, 0, 0), (0, 14.0, 0), 4.6, "plastic_black", 20)
    mb.cyl((0, 14.0, 0), (0, 22.0, 0), 4.0, "plastic_black", 16, r2=2.6)
    _mm(mb).obj("elec:plug_dc", col, root, T_(*P["dc"]) @ R_("Z", jyaw))
    lx, ly, lz, lyaw = ELEC_PLACE["elec:lipo_3s"]
    LIPO = T_(lx, ly, lz) @ R_("Z", lyaw)
    _e_lipo().obj("elec:lipo_3s", col, root, LIPO)
    xt_m = (LIPO @ np.r_[0.118, 0.0, 0.00405, 1.0])[:3]
    _e_xt60(True).obj("elec:lipo_xt60", col, root, T_(*xt_m) @ R_("Z", lyaw))
    xt_f = xt_m + np.array([0.0381, 0.0, 0.0])
    _e_xt60(False).obj("elec:harness_xt60", col, root, T_(*xt_f) @ R_("Z", 180 + lyaw))
    # AV fan-out of the capture stick, on the board in front-right of the base
    av = np.array([0.25, -0.235, 0.0])
    _e_av_breakout().obj("elec:digitnow_av", col, root, T_(*av) @ R_("Z", 35.0))
    zr = 0.0024
    # --- cables (world points)
    tip = (JET @ np.r_[np.asarray(_usb_a_ports()["usb2_up"]) * MM + [0, 0.068, 0], 1.0])[:3]
    _cable("elec:cable_digitnow", [tip, tip + [0.01, 0.025, -0.012], [0.16, -0.20, zr], [0.22, -0.225, zr],
                                   av + [-0.004, -0.002, 0.004]], 0.0022, "wire_usb", col)
    rc_out = np.array(_b2w((251.0, 160.0, -163.0)))
    av_y = (av + np.array(R_("Z", 35.0))[:3, :3] @ np.array([0.056, -0.0165, 0.0045]))
    _cable("elec:cable_rc832_av", [rc_out, rc_out + [0.03, 0.0, -0.005], _b2w((300.0, 150.0, -173.0 + 2.0)),
                                   [0.385, 0.06, zr], [0.40, -0.12, zr], [0.34, -0.215, zr],
                                   av_y + [0.04, -0.005, 0.0], av_y + [0.015, 0, 0]], 0.0018, "wire_yellow", col)
    _pair("elec:cable_rc832_power", [rc_out + [0, 0.004, 0], rc_out + [0.03, 0.006, -0.005],
                                      _b2w((305.0, 175.0, -171.0)), [0.30, 0.2, zr * 0.5], [0.16, 0.27, zr * 0.5],
                                      xt_f + [0.028, 0.012, 0.0]], 0.0009, ("red", "black"), col)
    esp_port = np.array(_b2w((237.0 - 2.4 - 0.6 + 0.6, -67.0 + 13.0 + 19.4, -165.0 + 30.0 + 1.6 + 1.6)))
    ep = esp_port + [0.012, 0, 0]
    tie1 = np.array(_b2w((195.0, -96.0, -165.0 + 4.0 + 1.5)))
    kiyo_back = np.array(_b2w((131.0, -96.0, 272.0 - 165.0)))
    p_esp = (JET @ np.r_[np.asarray(_usb_a_ports()["usb1_low"]) * MM + [0, 0.025, 0], 1.0])[:3]
    p_kiyo = (JET @ np.r_[np.asarray(_usb_a_ports()["usb1_up"]) * MM + [0, 0.025, 0], 1.0])[:3]
    _cable("elec:cable_esp32_usb", [esp_port, ep, ep + [0.02, 0.0, -0.025], np.array(_b2w((262.0, -60.0, -163.0))),
                                    np.array(_b2w((240.0, -92.0, -163.0))), tie1 + [0.006, 0, 0], tie1,
                                    tie1 + [-0.012, -0.004, 0], np.array(_b2w((176.0, -134.0, -162.0))),
                                    np.array(_b2w((170.0, -150.0, -171.0))), [0.11, -0.215, zr], p_esp + [0, 0.02, 0],
                                    p_esp], 0.0021, "wire_usb", col)
    _cable("elec:cable_kiyo_usb", [kiyo_back, kiyo_back + [-0.02, 0, -0.01], np.array(_b2w((118.0, -120.0, 60.0))),
                                   np.array(_b2w((120.0, -150.0, -90.0))), np.array(_b2w((110.0, -152.0, -171.0))),
                                   [0.07, -0.215, zr], p_kiyo + [0, 0.02, 0.0], p_kiyo], 0.0018, "wire_usb", col)
    bat_exit = np.array(_b2w((237.0 - 24.0, 57.0 + 2.0, -165.0 + 10.0)))
    tie2 = np.array(_b2w((215.0, 130.0, -165.0 + 4.0 + 1.5)))
    lead2 = [bat_exit, bat_exit + [0, 0.02, -0.004], tie2 + [0, -0.008, 0], tie2, tie2 + [0.01, 0.01, 0],
             np.array(_b2w((268.0, 140.0, -163.0))), np.array(_b2w((285.0, 142.0, -172.0))), [0.37, 0.16, zr],
             [0.30, 0.25, zr], [0.20, 0.30, zr], xt_f + [0.05, 0.004, 0.0], xt_f + [0.016, 0.0, 0.0]]
    _pair("elec:cable_battery_lead2", lead2, 0.0017, ("red", "black"), col)
    p_dc = (JET @ np.r_[np.asarray(_usb_a_ports()["dc"]) * MM + [0, 0.022, 0], 1.0])[:3]
    lead1 = [xt_f + [0.016, -0.006, 0.0], xt_f + [0.05, -0.02, 0.0], [0.24, 0.21, zr], [0.43, 0.10, zr],
             [0.44, -0.12, zr], [0.38, -0.30, zr], [0.20, -0.35, zr], [0.045, -0.24, zr], p_dc + [0, 0.015, 0],
             p_dc]
    _cable("elec:cable_jetson_dc", lead1, 0.0021, "wire_black", col)
    # wiper motor: 2-pin connector boot on the can end cap, 14 AWG pair down to the bay's motor window
    mb = MB()
    mb.box((-6.0, -9.0, 0.0), (6.0, 9.0, 9.0), "plastic_black", bevel=1.5)
    mb.box((-4.0, -7.0, 9.0), (4.0, 7.0, 15.0), "rubber", bevel=1.5)
    _mm(mb).obj("elec:wiper_plug", col, root, np.asarray(BASE_TO_WORLD) @ T_(8.0 * MM, -46.0 * MM, 179.5 * MM))
    wp = [_b2w((8.0, -46.0, 194.0)), _b2w((14.0, -50.0, 197.0)), _b2w((30.0, -78.0, 182.0)),
          _b2w((70.0, -95.0, 60.0)), _b2w((95.0, -60.0, -120.0)), _b2w((105.0, -12.0, -152.0)),
          _b2w((116.0, -11.0, -152.0))]
    _pair("elec:cable_wiper", wp, 0.0016, ("red", "black"), col)
    return root


# ============================================================================================== previews
def _look_at(ob, target):
    ob.rotation_euler = (Vector(target) - ob.location).to_track_quat("-Z", "Y").to_euler()


def studio(scene, floor_z=-BOARD_SIZE[2] - 0.0005):
    """simple studio: soft gradient world, grey floor, key / fill / rim area lights (preview only)"""
    w = bpy.data.worlds.get("studio") or bpy.data.worlds.new("studio")
    scene.world = w
    nt = w.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    N = _N(nt)
    out = N.node("ShaderNodeOutputWorld")
    bg = N.node("ShaderNodeBackground")
    z = N.sep(N.coord("Generated"))[2]
    col = N.ramp(N.math("ADD", N.math("MULTIPLY", z, 0.5), 0.5), [(0.35, (0.03, 0.03, 0.033, 1)), (0.55, (0.16, 0.165, 0.175, 1)),
                                                        (1.0, (0.5, 0.52, 0.55, 1))])
    N.link(col, bg.inputs["Color"])
    bg.inputs["Strength"].default_value = 0.3
    N.link(bg.outputs["Background"], out.inputs["Surface"])
    col_l = get_collection("STUDIO", scene)
    if "studio:floor" not in bpy.data.objects:
        mb = MB()
        mb.box((-12, -12, floor_z - 0.01), (12, 12, floor_z), mat_simple("studio_floor", (0.09, 0.09, 0.095, 1), plain=True,
                                                                         rough=0.55, spec=0.3))
        mb.obj("studio:floor", col_l, smooth=0)
    for nm, (loc, size, energy, colr) in {
            "key": ((1.3, -1.5, 1.9), 1.4, 230.0, (1.0, 0.95, 0.88)),
            "fill": ((-1.6, -1.0, 1.1), 3.0, 70.0, (0.88, 0.94, 1.0)),
            "rim": ((-0.9, 1.6, 1.4), 1.0, 160.0, (1.0, 1.0, 1.0)),
            "top": ((0.0, 0.0, 2.6), 2.5, 50.0, (1.0, 1.0, 1.0))}.items():
        ob = bpy.data.objects.get("studio:" + nm)
        if ob is None:
            ld = bpy.data.lights.new("studio:" + nm, "AREA")
            ob = bpy.data.objects.new("studio:" + nm, ld)
            col_l.objects.link(ob)
        ob.data.energy, ob.data.size, ob.data.color = energy, size, colr
        ob.location = loc
        _look_at(ob, (0, 0, 0.1))
    cam = bpy.data.objects.get("studio:cam")
    if cam is None:
        cam = bpy.data.objects.new("studio:cam", bpy.data.cameras.new("studio:cam"))
        col_l.objects.link(cam)
    scene.camera = cam
    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.use_denoising = True
    scene.cycles.max_bounces = 6
    scene.view_settings.view_transform = "AgX"
    try:
        scene.view_settings.look = "AgX - Medium High Contrast"
    except Exception:
        pass
    return cam


def render_grid(scene, views, path, grid=(2, 2), size=(960, 540), samples=48):
    """render each view (dict: loc, target, lens, before=callable, after=callable) as a tile and save the grid"""
    cam = scene.camera
    cols, rows = grid
    tw, th = size[0] // cols, size[1] // rows
    scene.render.resolution_x, scene.render.resolution_y = tw, th
    scene.render.resolution_percentage = 100
    scene.cycles.samples = samples
    canvas = np.zeros((size[1], size[0], 4), np.float32)
    tmp = os.path.join(OUT_DIR, "_tile.png")
    for i, v in enumerate(views):
        if v.get("before"):
            v["before"]()
        cam.location = v["loc"]
        _look_at(cam, v["target"])
        cam.data.lens = v.get("lens", 50.0)
        cam.data.clip_start = 0.005
        cam.data.dof.use_dof = False
        scene.render.filepath = tmp
        t0 = time.time()
        bpy.ops.render.render(write_still=True)
        img = bpy.data.images.load(tmp, check_existing=False)
        px = np.array(img.pixels[:], np.float32).reshape(th, tw, 4)
        bpy.data.images.remove(img)
        r, c = divmod(i, cols)
        y0 = (rows - 1 - r) * th
        canvas[y0:y0 + th, c * tw:(c + 1) * tw] = px
        print(f"  view {i}: {time.time() - t0:.1f} s")
        if v.get("after"):
            v["after"]()
    out = bpy.data.images.new("grid", size[0], size[1], alpha=False)
    out.pixels.foreach_set(canvas.ravel())
    out.filepath_raw = path
    out.file_format = "PNG"
    out.save()
    bpy.data.images.remove(out)
    try:
        os.remove(tmp)
    except OSError:
        pass
    print("saved", path)


def _hide(names, flag=True):
    for n in names:
        ob = bpy.data.objects.get(n)
        if ob is not None:
            ob.hide_render = flag
            ob.hide_viewport = flag


def _hide_col(name, flag=True):
    col = bpy.data.collections.get(name)
    if col is not None:
        col.hide_render = flag


def _dart_to_floor(floor_z):
    """drop dart:root so the lowest dart vertex touches the floor (preview poses)"""
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    zmin = 1e9
    for ob in bpy.data.collections["DART"].all_objects:
        if ob.type != "MESH":
            continue
        Mw = np.array(ob.matrix_world)
        v = np.empty(len(ob.data.vertices) * 3, np.float32)
        ob.data.vertices.foreach_get("co", v)
        v = v.reshape(-1, 3) @ Mw[:3, :3].T + Mw[:3, 3]
        zmin = min(zmin, float(v[:, 2].min())) if len(v) else zmin
    r = bpy.data.objects["dart:root"]
    r.location.z += floor_z - zmin
    bpy.context.view_layer.update()


def previews(scene, only=("base", "elec", "dart")):
    os.makedirs(OUT_DIR, exist_ok=True)
    floor_z = -BOARD_SIZE[2] - 0.0005
    studio(scene, floor_z)
    B = lambda p: Vector(_b2w(p))                                    # noqa: E731
    # --- base (with the board, the dart on the cradle and the kit)
    if "base" in only:
        render_grid(scene, [
            dict(loc=(1.05, -0.95, 0.62), target=(0.0, 0.0, 0.10), lens=38),
            dict(loc=(-1.0, 0.95, 0.70), target=(-0.05, 0.02, 0.10), lens=38),
            dict(loc=B((470.0, 380.0, 300.0)), target=B((170.0, 130.0, 60.0)), lens=35),
            dict(loc=B((470.0, -330.0, 90.0)), target=B((110.0, 10.0, -110.0)), lens=30),
        ], os.path.join(OUT_DIR, "preview_models_base.png"))
    # --- elec
    lid = ["base:frame_ebay_lid"]
    if "elec" in only:
        render_grid(scene, [
            dict(loc=(0.33, -0.55, 0.22), target=(0.10, -0.29, 0.02), lens=45),
            dict(loc=(0.35, 0.62, 0.28), target=(0.12, 0.25, 0.02), lens=40),
            dict(loc=B((250.0, -40.0, 120.0)), target=B((175.0, -5.0, -150.0)), lens=40,
                 before=lambda: _hide(lid), after=lambda: _hide(lid, False)),
            dict(loc=(0.95, -0.75, 0.55), target=(0.18, -0.12, 0.02), lens=40),
        ], os.path.join(OUT_DIR, "preview_models_elec.png"))
    if "dart" not in only:
        return
    # --- dart alone on the floor
    _hide_col("BASE")
    _hide_col("ELEC")
    root = bpy.data.objects["dart:root"]
    rest = root.matrix_world.copy()
    root.matrix_world = Matrix.Identity(4)
    pose_dart(0, 0, 0)
    _dart_to_floor(floor_z)
    shell = ["dart:hull_top", "dart:nose", "dart:righting_leg", "dart:leg_plug_L", "dart:leg_plug_R",
             "dart:screws_radial"]

    def deployed():
        root.matrix_world = Matrix.Identity(4)
        pose_dart(leg_deg=95.0, deploy_L=46.75, deploy_R=46.75, auger_L=30.0, auger_R=-30.0)
        _dart_to_floor(floor_z)
    c = Vector((0.085, 0.0, floor_z + 0.03))
    render_grid(scene, [
        dict(loc=c + Vector((-0.20, -0.24, 0.16)), target=c, lens=55),
        dict(loc=c + Vector((0.05, -0.14, 0.26)), target=c + Vector((0, 0, -0.005)), lens=55,
             before=lambda: _hide(shell), after=lambda: _hide(shell, False)),
        dict(loc=c + Vector((0.34, -0.22, 0.14)), target=c + Vector((0.0, 0, 0.01)), lens=50, before=deployed),
        dict(loc=c + Vector((-0.12, 0.30, 0.05)), target=c + Vector((0.02, 0.0, 0.0)), lens=50),
    ], os.path.join(OUT_DIR, "preview_models_dart.png"))
    pose_dart(0, 0, 0)
    root.matrix_world = rest
    _hide_col("BASE", False)
    _hide_col("ELEC", False)


if __name__ == "__main__":
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    build_models(sc)
    if "--no-render" not in sys.argv:
        sel = [a for a in ("base", "elec", "dart") if a in sys.argv[1:]] or ("base", "elec", "dart")
        previews(sc, sel)
