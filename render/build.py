"""GrappStar mission animation: BUILD (integration). See render/SPEC.md and render/BUILD.md.

    render/.venv/bin/python render/build.py [--quality preview|final] [--no-save]
        [--shots all|1,4,...] [--motion-check] [--check-only]

Builds the whole scene in a fresh Blender session and saves render/out/grappstar_mission.blend (textures packed,
relative paths):
  env      render/env.py build_env(): paddock, grass, HDRI sky, sun (grass kept off the board and the dart's rest
           spot; a grass "Density" input and a dart-pressed-grass deformer are added to the env:grass node tree here)
  models   render/models.py build_models(): base, dart, electronics at rest
  motion   render/motion/motion.json: every moving object keyed from its world transforms (local to its parent),
           dart joints (righting leg, pods, auger spin) as axis-angle keys, springs stretched per key
  line     line:tether and line:wraps: geometry-nodes curves that read a per-key point cache stored in their own
           mesh (no Python handlers, no drivers, renders headless anywhere); draped on the ground and the board
  fx       soil crumbs (geometry-nodes instances with ballistic paths computed in the node tree) at the impact and
           at both drills, soil mound rings round the drills
  cameras  10 shots (cam:NN_name), keyed every frame, cut by timeline markers bound to the cameras, DOF keyed
  render   env.set_render(EEVEE, final 1920x1080, 30 fps, motion blur), frames 0..1730
Everything is rebuilt from the current env.py, models.py and motion.json on each run.
--shots renders Cycles CPU 640x360 stills (render/out/shot_NN_name.png + shots_sheet.png), --motion-check renders a
320x180 EEVEE preview of every 2nd frame to render/out/motion_check.mp4 (frames cached in render/out/frames/mc/,
resumable). --check-only skips the build and loads the saved .blend for the checks.
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import time

import numpy as np
import bpy
from mathutils import Matrix, Quaternion, Vector
from mathutils.bvhtree import BVHTree

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
OUT = os.path.join(HERE, "out")
MOTION_JSON = os.path.join(HERE, "motion", "motion.json")
BLEND = os.path.join(OUT, "grappstar_mission.blend")
FRAMES_DIR = os.path.join(OUT, "frames")

ROPE_R = 0.0005                     # 1 mm Dyneema
ROPE_REST_Z = 0.002                 # rope centre height when lying on the (flat) sim ground in motion.json
ROPE_DRAPE = 0.001                  # rope centre above the real ground (terrain, board, deck)
DART_CG_X = 0.0833                  # dart frame CG station (motion.json dart.cg_station_m)
G = 9.81

INTERP = {"CONSTANT": 0, "LINEAR": 1, "BEZIER": 2}
T0 = time.time()


def say(*a):
    print("[build %6.1fs]" % (time.time() - T0), *a, flush=True)


# =============================================================================================================
# small helpers
def coll(scene, name, parent=None, exclude=False):
    c = bpy.data.collections.get(name) or bpy.data.collections.new(name)
    par = parent or scene.collection
    if c.name not in par.children:
        par.children.link(c)
    if exclude:
        def walk(lc):
            for sub in lc.children:
                if sub.collection == c:
                    sub.exclude = True
                walk(sub)
        walk(scene.view_layers[0].layer_collection)
    return c


def ensure_action(idb):
    ad = idb.animation_data or idb.animation_data_create()
    if ad.action is None:
        ad.action = bpy.data.actions.new("act:" + idb.name)
    return ad.action


def key_curve(idb, path, index, frames, values, interp="LINEAR"):
    """bulk keys: one F-curve, all keys at once (keyframe_points.add + foreach_set)"""
    act = ensure_action(idb)
    fc = act.fcurve_ensure_for_datablock(idb, path, index=index)
    kp = fc.keyframe_points
    if len(kp):
        kp.clear()
    n = len(frames)
    kp.add(n)
    co = np.empty(2 * n, np.float32)
    co[0::2] = np.asarray(frames, np.float32)
    co[1::2] = np.asarray(values, np.float32)
    kp.foreach_set("co", co)
    kp.foreach_set("interpolation", np.full(n, INTERP[interp], np.int32))
    fc.update()
    return fc


def all_fcurves(idb):
    ad = idb.animation_data
    if not ad or not ad.action:
        return []
    out = []
    for layer in ad.action.layers:
        for strip in layer.strips:
            for cb in strip.channelbags:
                out.extend(cb.fcurves)
    return out


def set_linear(idb):
    for fc in all_fcurves(idb):
        for k in fc.keyframe_points:
            k.interpolation = "LINEAR"


def quat_fix(Q):
    """sign continuity for a (K, 4) quaternion track"""
    Q = np.array(Q, float)
    for i in range(1, len(Q)):
        if np.dot(Q[i], Q[i - 1]) < 0:
            Q[i] = -Q[i]
    return Q


def qmul(a, b):
    """quaternion product, (..., 4) wxyz"""
    aw, ax, ay, az = np.moveaxis(a, -1, 0)
    bw, bx, by, bz = np.moveaxis(b, -1, 0)
    return np.stack([aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw], -1)


def qrot(q, v):
    """rotate vectors v (..., 3) by unit quaternions q (..., 4)"""
    w = q[..., :1]
    u = q[..., 1:]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def sample(frames, vals, fq):
    """linear interpolation of a key track (motion.json keys are sparse, LINEAR)"""
    fr = np.asarray(frames, float)
    V = np.asarray(vals, float)
    if V.ndim == 1:
        return np.interp(fq, fr, V)
    return np.stack([np.interp(fq, fr, V[:, d]) for d in range(V.shape[1])], -1)


def sample_quat(frames, Q, fq):
    q = sample(frames, quat_fix(Q), fq)
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def gauss_smooth(A, sigma):
    if sigma <= 0:
        return A.copy()
    r = int(3 * sigma) + 1
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    k /= k.sum()
    P = np.pad(A, ((r, r), (0, 0)), mode="edge")
    return np.stack([np.convolve(P[:, d], k, mode="valid") for d in range(A.shape[1])], -1)


def smoother(u):
    u = np.clip(u, 0.0, 1.0)
    return u * u * u * (u * (6 * u - 15) + 10)


def ease(f, f0, f1):
    return float(smoother((f - f0) / max(f1 - f0, 1e-9)))


def lerp(a, b, t):
    return np.asarray(a, float) + (np.asarray(b, float) - np.asarray(a, float)) * t


# =============================================================================================================
# node helpers
class NG:
    def __init__(self, name, ng=None):
        if ng is None:
            old = bpy.data.node_groups.get(name)
            if old:
                bpy.data.node_groups.remove(old)
            ng = bpy.data.node_groups.new(name, "GeometryNodeTree")
            ng.interface.new_socket("Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
            ng.interface.new_socket("Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
            self.gi = ng.nodes.new("NodeGroupInput")
            self.go = ng.nodes.new("NodeGroupOutput")
        else:
            self.gi = [n for n in ng.nodes if n.bl_idname == "NodeGroupInput"][0]
            self.go = [n for n in ng.nodes if n.bl_idname == "NodeGroupOutput"][0]
        self.ng = ng
        self.k = 0

    def n(self, t, **kw):
        node = self.ng.nodes.new(t)
        for k, v in kw.items():
            setattr(node, k, v)
        node.location = (200 * (self.k % 12), -180 * (self.k // 12))
        self.k += 1
        return node

    def put(self, sock, v):
        if isinstance(v, bpy.types.NodeSocket):
            self.ng.links.new(v, sock)
        elif v is not None:
            sock.default_value = v

    def math(self, op, a, b=None, c=None):
        m = self.n("ShaderNodeMath", operation=op)
        for i, v in enumerate((a, b, c)):
            self.put(m.inputs[i], v)
        return m.outputs[0]

    def vmath(self, op, a, b=None, scale=None):
        m = self.n("ShaderNodeVectorMath", operation=op)
        self.put(m.inputs[0], a)
        if b is not None:
            self.put(m.inputs[1], b)
        if scale is not None:
            self.put(m.inputs[3], scale)
        return m.outputs["Value"] if op in ("LENGTH", "DOT_PRODUCT", "DISTANCE") else m.outputs["Vector"]

    def attr(self, name, dt="FLOAT"):
        a = self.n("GeometryNodeInputNamedAttribute", data_type=dt)
        a.inputs["Name"].default_value = name
        return a.outputs["Attribute"]

    def sample(self, geo, value, index, dt="FLOAT"):
        s = self.n("GeometryNodeSampleIndex", data_type=dt, domain="POINT")
        self.put(s.inputs["Geometry"], geo)
        self.put(s.inputs["Value"], value)
        self.put(s.inputs["Index"], index)
        return s.outputs["Value"]

    def xyz(self, x, y, z):
        c = self.n("ShaderNodeCombineXYZ")
        for i, v in enumerate((x, y, z)):
            self.put(c.inputs[i], v)
        return c.outputs[0]

    def sep(self, v):
        s = self.n("ShaderNodeSeparateXYZ")
        self.put(s.inputs[0], v)
        return s.outputs

    def maprange(self, v, a, b, c=0.0, d=1.0, smooth=True):
        m = self.n("ShaderNodeMapRange", interpolation_type="SMOOTHSTEP" if smooth else "LINEAR", clamp=True)
        for i, x in enumerate((v, a, b, c, d)):
            self.put(m.inputs[i], x)
        return m.outputs["Result"]


# =============================================================================================================
# motion
class Motion:
    def __init__(self, path):
        with open(path) as fh:
            self.M = json.load(fh)
        M = self.M
        self.nf = int(M["frames"])
        self.fps = int(M.get("fps", 30))
        self.F = np.arange(self.nf, dtype=float)
        self.ev = {}
        for e in M.get("events", []):
            self.ev.setdefault(e["name"], int(e["frame"]))
        d = M["objects"]["dart:root"]
        self.dart_loc = sample(d["frames"], d["loc"], self.F)
        self.dart_q = sample_quat(d["frames"], d["quat"], self.F)
        self.dart_cg = self.dart_loc + qrot(self.dart_q, np.array([DART_CG_X, 0.0, 0.0]))
        self.cg_s = gauss_smooth(self.dart_cg, 4.0)

    def e(self, name, default):
        return self.ev.get(name, default)

    def dart_point(self, p_dart, f=None):
        """world position of a dart-frame point for all frames (or frame f)"""
        p = np.asarray(p_dart, float)
        if f is None:
            return self.dart_loc + qrot(self.dart_q, p)
        i = int(np.clip(round(f), 0, self.nf - 1))
        return self.dart_loc[i] + qrot(self.dart_q[i], p)

    def joint_deg(self, name, fq):
        J = self.M["joints"].get(name)
        if J is None:
            return np.zeros_like(np.asarray(fq, float))
        return sample(J["frames"], J["deg"], fq)


def apply_objects(mo, report):
    objs = mo.M["objects"]
    missing, keyed, static = [], 0, 0
    for name, d in objs.items():
        ob = bpy.data.objects.get(name)
        if ob is None:
            missing.append(name)
            continue
        fr = np.asarray(d["frames"], float)
        L = np.asarray(d["loc"], float)
        Q = quat_fix(d["quat"])
        par = ob.parent
        if par is not None and par.name in objs:
            pd = objs[par.name]
            PL = sample(pd["frames"], pd["loc"], fr)
            PQ = sample_quat(pd["frames"], pd["quat"], fr)
            PI = np.asarray(ob.matrix_parent_inverse)
            if np.abs(PI - np.eye(4)).max() > 1e-9:
                raise RuntimeError("parent inverse not identity on animated child " + name)
            PQi = PQ * np.array([1, -1, -1, -1])
            Ll = qrot(PQi, L - PL)
            Ql = qmul(PQi, Q)
        else:
            Pw = (par.matrix_world @ ob.matrix_parent_inverse) if par is not None else Matrix.Identity(4)
            Pinv = Pw.inverted()
            pl, pq, ps = Pinv.decompose()
            if abs(ps.x - 1) + abs(ps.y - 1) + abs(ps.z - 1) > 1e-6:
                raise RuntimeError("scaled parent for " + name)
            pq = np.array(pq)
            Ll = qrot(np.broadcast_to(pq, Q.shape), L) + np.array(pl)
            Ql = qmul(np.broadcast_to(pq, Q.shape), Q)
        Ql = quat_fix(Ql)
        ob.rotation_mode = "QUATERNION"
        if len(fr) == 1:
            ob.location = Ll[0]
            ob.rotation_quaternion = Ql[0]
            static += 1
            continue
        for i in range(3):
            key_curve(ob, "location", i, fr, Ll[:, i])
        for i in range(4):
            key_curve(ob, "rotation_quaternion", i, fr, Ql[:, i])
        keyed += 1
    report["objects_keyed"] = keyed
    report["objects_static_pose"] = static
    report["objects_missing"] = missing
    say("objects keyed %d, posed %d, not in scene %d %s" % (keyed, static, len(missing), missing[:6]))


def apply_joints(mo, models, report):
    aliases = getattr(models, "DART_JOINT_ALIASES", {})
    DJ = getattr(models, "DART_JOINTS", {})
    done = []
    for jn, J in mo.M.get("joints", {}).items():
        inv = [k for k, v in aliases.items() if v == jn]
        cands = [c for c in [jn, aliases.get(jn)] + inv if c and bpy.data.objects.get(c) is not None]
        cands.sort(key=lambda c: c not in DJ)
        ob = bpy.data.objects.get(cands[0]) if cands else None
        if ob is None:
            say("joint", jn, "has no pivot object")
            continue
        axis = Vector(J["axis"]).normalized()
        if ob.name in DJ:
            ma = Vector(DJ[ob.name]["axis"]).normalized()
            if (ma - axis).length > 1e-4:
                say("WARNING joint axis differs from models.DART_JOINTS", jn, tuple(axis), tuple(ma))
        ob.rotation_mode = "AXIS_ANGLE"
        ob.rotation_axis_angle = (0.0, axis.x, axis.y, axis.z)
        key_curve(ob, "rotation_axis_angle", 0, J["frames"], np.radians(np.asarray(J["deg"], float)))
        done.append(ob.name)
    report["joints"] = done
    say("joints keyed:", done)


def apply_springs(mo, models, report):
    out = []
    objs = mo.M["objects"]
    fn = getattr(models, "set_spring_pins", None)
    for key, S in mo.M.get("springs", {}).items():
        name = key if key.startswith("base:") else "base:" + key
        ob = bpy.data.objects.get(name)
        if ob is None or fn is None:
            say("spring", name, "not keyed (no object or no models.set_spring_pins)")
            continue
        fr = np.asarray(S["frames"], float)
        hk = objs.get(name + "_hook_c")
        if hk is not None:
            HQ = sample_quat(hk["frames"], hk["quat"], fr)
            UP = qrot(HQ, np.array([0.0, 0.0, 1.0]))
        else:
            UP = np.tile([0.0, 0.0, 1.0], (len(fr), 1))
        for f, pc, pa, up in zip(fr, S["carriage_pin"], S["anchor_pin"], UP):
            fn(ob, pc, pa, up=tuple(up), frame=float(f))
        set_linear(ob)
        out.append(name)
    report["springs"] = out
    say("springs keyed:", out)


def hide_standins(report):
    hid = []
    for ob in bpy.data.objects:
        if ob.name.startswith("base:launch_line_"):
            ob.hide_render = True
            ob.hide_viewport = True
            hid.append(ob.name)
    report["hidden_line_standins"] = len(hid)


# =============================================================================================================
# ground for draping: terrain + the static low surfaces of the board and the base deck
class Ground:
    def __init__(self, env):
        self.env = env
        verts, polys = [], []
        dg = bpy.context.evaluated_depsgraph_get()

        def animated(ob):
            while ob is not None:
                if ob.animation_data and ob.animation_data.action:
                    return True
                ob = ob.parent
            return False

        for ob in bpy.data.objects:
            if ob.type != "MESH" or not (ob.name.startswith("base:") or ob.name.startswith("elec:")):
                continue
            if animated(ob) or ob.hide_render:
                continue
            bb = [ob.matrix_world @ Vector(c) for c in ob.bound_box]
            if min(v.z for v in bb) > 0.04:
                continue
            ev = ob.evaluated_get(dg)
            me = ev.to_mesh()
            mw = ob.matrix_world
            base = len(verts)
            co = np.empty(len(me.vertices) * 3, np.float32)
            me.vertices.foreach_get("co", co)
            co = co.reshape(-1, 3) @ np.asarray(mw.to_3x3()).T + np.asarray(mw.translation)
            verts.extend(map(tuple, co))
            polys.extend([tuple(base + i for i in p.vertices) for p in me.polygons])
            ev.to_mesh_clear()
        self.bvh = BVHTree.FromPolygons(verts, polys) if polys else None
        say("drape surfaces: %d verts" % len(verts))

    def height(self, x, y, z_top=0.06):
        """ground height under (x, y) arrays"""
        g = np.asarray(self.env.terrain_height(np.asarray(x, float), np.asarray(y, float)), float)
        if self.bvh is not None:
            for i in range(len(g)):
                hit = self.bvh.ray_cast(Vector((x[i], y[i], z_top)), Vector((0, 0, -1)), 0.2)
                if hit[0] is not None:
                    g[i] = max(g[i], hit[0].z)
        return g


def hermite_resample(P, m):
    """(K, n, 3) polylines -> (K, (n-1)*m+1, 3) through the same points, chord-length Catmull-Rom tangents"""
    K, n, _ = P.shape
    d = np.linalg.norm(np.diff(P, axis=1), axis=2) + 1e-9            # (K, n-1)
    T = np.zeros((K, n, 3))
    T[:, 1:-1] = (P[:, 2:] - P[:, :-2]) / (d[:, 1:] + d[:, :-1])[..., None]
    T[:, 0] = (P[:, 1] - P[:, 0]) / d[:, :1]
    T[:, -1] = (P[:, -1] - P[:, -2]) / d[:, -1:]
    out = []
    for s in range(n - 1):
        p0, p1 = P[:, s], P[:, s + 1]
        m0, m1 = T[:, s] * d[:, s:s + 1], T[:, s + 1] * d[:, s:s + 1]
        for j in range(m):
            t = j / m
            h00, h10, h01, h11 = 2 * t**3 - 3 * t**2 + 1, t**3 - 2 * t**2 + t, -2 * t**3 + 3 * t**2, t**3 - t**2
            out.append(h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1)
    out.append(P[:, -1])
    return np.stack(out, 1)


def drape(P, ground, k_fixed):
    """rope points lying on the flat sim ground (z ~ 2 mm) -> onto the real ground (+1 mm); points higher up
    keep their height (blend over 6 cm); points before k_fixed (inside the base) are untouched"""
    K, n, _ = P.shape
    Q = P.copy()
    free = Q[:, k_fixed:].reshape(-1, 3)
    low = free[:, 2] < 0.08
    idx = np.where(low)[0]
    if len(idx):
        x, y, z = free[idx, 0], free[idx, 1], free[idx, 2]
        g = ground.height(x, y)
        w = np.clip(1.0 - (z - 0.004) / 0.056, 0.0, 1.0)
        zn = z + w * (g + ROPE_DRAPE - ROPE_REST_Z)
        zn = np.maximum(zn, g + ROPE_R * 1.3)
        free[idx, 2] = zn
    Q[:, k_fixed:] = free.reshape(K, n - k_fixed, 3)
    return Q


def cache_curve_nodes(name, N, P, K, mat, res=6, prof=6, hide_before=False):
    """GN tree that turns a per-key point cache (N points per key, attribute kf = key frame) into the curve
    for the current (sub)frame, interpolating linearly between keys. The first P points (base path) are a
    poly spline, the rest Catmull-Rom; P = 0: all Catmull-Rom."""
    g = NG(name)
    geo = g.gi.outputs[0]
    t = g.n("GeometryNodeInputSceneTime").outputs["Frame"]
    kf = g.attr("kf")
    lt = g.math("LESS_THAN", kf, g.math("ADD", t, 1e-3))
    st = g.n("GeometryNodeAttributeStatistic", data_type="FLOAT", domain="POINT")
    g.put(st.inputs["Geometry"], geo)
    g.put(st.inputs["Attribute"], lt)
    k = g.math("SUBTRACT", g.math("FLOOR", g.math("DIVIDE", st.outputs["Sum"], float(N))), 1.0)
    k = g.math("MINIMUM", g.math("MAXIMUM", k, 0.0), float(max(K - 2, 0)))
    i0 = g.math("MULTIPLY", k, float(N))
    i1 = g.math("ADD", i0, float(N))
    f0 = g.sample(geo, kf, i0)
    f1 = g.sample(geo, kf, i1)
    w = g.math("DIVIDE", g.math("SUBTRACT", t, f0), g.math("MAXIMUM", g.math("SUBTRACT", f1, f0), 1e-6))
    w = g.math("MINIMUM", g.math("MAXIMUM", w, 0.0), 1.0)
    idx = g.n("GeometryNodeInputIndex").outputs[0]
    if P > 0:
        after = g.math("GREATER_THAN", idx, P - 0.5)
        src = g.math("SUBTRACT", idx, after)
        count = N + 1
    else:
        after = None
        src = idx
        count = N
    pos_attr = g.n("GeometryNodeInputPosition").outputs[0]
    p0 = g.sample(geo, pos_attr, g.math("ADD", i0, src), "FLOAT_VECTOR")
    p1 = g.sample(geo, pos_attr, g.math("ADD", i1, src), "FLOAT_VECTOR")
    pos = g.vmath("ADD", p0, g.vmath("SCALE", g.vmath("SUBTRACT", p1, p0), scale=w))
    pts = g.n("GeometryNodePoints")
    pts.inputs["Count"].default_value = count
    g.put(pts.inputs["Position"], pos)
    p2c = g.n("GeometryNodePointsToCurves")
    g.put(p2c.inputs["Points"], pts.outputs[0])
    if after is not None:
        g.put(p2c.inputs["Curve Group ID"], after)
    g.put(p2c.inputs["Weight"], idx)
    sty = g.n("GeometryNodeCurveSplineType", spline_type="CATMULL_ROM")
    g.put(sty.inputs["Curve"], p2c.outputs[0])
    if P > 0:
        cidx = g.n("GeometryNodeInputIndex").outputs[0]
        g.put(sty.inputs["Selection"], g.math("GREATER_THAN", cidx, 0.5))
    rs = g.n("GeometryNodeSetSplineResolution")
    g.put(rs.inputs["Curve"], sty.outputs[0])
    rs.inputs["Resolution"].default_value = res
    circ = g.n("GeometryNodeCurvePrimitiveCircle", mode="RADIUS")
    circ.inputs["Resolution"].default_value = prof
    circ.inputs["Radius"].default_value = ROPE_R
    c2m = g.n("GeometryNodeCurveToMesh")
    g.put(c2m.inputs["Curve"], rs.outputs[0])
    g.put(c2m.inputs["Profile Curve"], circ.outputs["Curve"])
    c2m.inputs["Fill Caps"].default_value = True
    sm = g.n("GeometryNodeSetShadeSmooth")
    g.put(sm.inputs["Mesh"], c2m.outputs[0])
    sem = g.n("GeometryNodeSetMaterial")
    g.put(sem.inputs["Geometry"], sm.outputs[0])
    sem.inputs["Material"].default_value = mat
    outg = sem.outputs[0]
    if hide_before:
        fa = g.sample(geo, kf, 0)
        vis = g.n("FunctionNodeCompare", data_type="FLOAT", operation="GREATER_EQUAL")
        g.put(vis.inputs["A"], t)
        g.put(vis.inputs["B"], g.math("SUBTRACT", fa, 0.5))
        sw = g.n("GeometryNodeSwitch", input_type="GEOMETRY")
        g.put(sw.inputs["Switch"], vis.outputs[0])
        g.put(sw.inputs["True"], outg)
        outg = sw.outputs[0]
    g.ng.links.new(outg, g.go.inputs[0])
    return g.ng


def cache_object(name, col, frames, P3, ng):
    K, N, _ = P3.shape
    me = bpy.data.meshes.new(name)
    me.vertices.add(K * N)
    me.vertices.foreach_set("co", P3.reshape(-1).astype(np.float32))
    a = me.attributes.new("kf", "FLOAT", "POINT")
    a.data.foreach_set("value", np.repeat(np.asarray(frames, np.float32), N))
    me.update()
    ob = bpy.data.objects.new(name, me)
    col.objects.link(ob)
    md = ob.modifiers.new("line", "NODES")
    md.node_group = ng
    ob["note"] = "geometry nodes curve from a per-key point cache (%d keys x %d points) in this mesh" % (K, N)
    return ob


def rope_material(models):
    for fn in ("rope_material", "mat_rope", "mat_dyneema_rope", "mat_dyneema"):
        f = getattr(models, fn, None)
        if callable(f):
            try:
                return f()
            except Exception as ex:
                say("rope material", fn, "failed:", ex)
    m = bpy.data.materials.new("line:dyneema")
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = (0.85, 0.65, 0.02, 1)
    b.inputs["Roughness"].default_value = 0.6
    return m


def build_line(scene, mo, models, env, report):
    col = coll(scene, "LINE")
    L = mo.M["line"]
    fr = np.asarray(L["frames"], float)
    P = np.asarray(L["points"], float)
    kb = int(L.get("k_base", 12))
    ground = Ground(env)
    free = hermite_resample(P[:, kb - 1:], 3)              # exit point + free rope, 3x denser, same points
    PP = np.concatenate([P[:, :kb - 1], free], 1)
    PP = drape(PP, ground, kb)
    mat = rope_material(models)
    K, N, _ = PP.shape
    ng = cache_curve_nodes("GS_line_tether", N, kb, K, mat)
    ob = cache_object("line:tether", col, fr, PP, ng)
    report["line"] = dict(keys=K, points=N, base_points=kb)
    W = mo.M.get("line_wraps")
    if W and len(W.get("frames", [])) > 1:
        WP = np.asarray(W["points"], float)
        ngw = cache_curve_nodes("GS_line_wraps", WP.shape[1], 0, WP.shape[0], mat, res=4, hide_before=True)
        cache_object("line:wraps", col, W["frames"], WP, ngw)
        report["line_wraps"] = dict(keys=WP.shape[0], points=WP.shape[1], first=int(W["frames"][0]))
    say("line built:", report.get("line"), report.get("line_wraps"))
    return ob, ground


# =============================================================================================================
# grass: density control and the dart pressing / parting the grass
def grass_mods(scene, mo, rest_press):
    grass = bpy.data.objects.get("env:grass")
    if grass is None:
        return
    md = [m for m in grass.modifiers if m.type == "NODES"][0]
    g = NG(None, md.node_group)
    ng = g.ng
    iop = [n for n in ng.nodes if n.bl_idname == "GeometryNodeInstanceOnPoints"][0]
    # Density (0..1): random thinning of the clumps (EEVEE speed knob)
    s = ng.interface.new_socket("Density", in_out="INPUT", socket_type="NodeSocketFloat")
    s.default_value, s.min_value, s.max_value = 1.0, 0.0, 1.0
    src = iop.inputs["Points"].links[0].from_socket
    rnd = g.n("FunctionNodeRandomValue", data_type="FLOAT")
    rnd.inputs["Seed"].default_value = 7
    dele = g.n("GeometryNodeDeleteGeometry", domain="POINT")
    g.put(dele.inputs["Geometry"], src)
    gt = g.math("GREATER_THAN", rnd.outputs["Value"], g.gi.outputs["Density"])
    g.put(dele.inputs["Selection"], gt)
    ng.links.new(dele.outputs[0], iop.inputs["Points"])
    # presses: empties (FX) whose xy position flattens and splays the clumps round them
    fx = coll(scene, "FX")
    root = bpy.data.objects["dart:root"]
    presses = []

    def press(name, parent=None, loc=(0, 0, 0), r=0.07, live=True):
        e = bpy.data.objects.new(name, None)
        fx.objects.link(e)
        e.empty_display_type = "CIRCLE"
        e.empty_display_size = 1.0
        e.parent = parent
        e.location = loc
        e.scale = (r, r, 1.0)
        e["live"] = live
        presses.append((e, live))
        return e

    press("fx:press_nose", root, (0.035, 0.0, 0.0), 0.075)
    press("fx:press_tail", root, (0.125, 0.0, 0.0), 0.08)
    for (name, xy, r, f_on, w) in rest_press:
        e = press(name, None, (xy[0], xy[1], 0.0), r, live=False)
        e.scale = (r, r, 0.0)
        key_curve(e, "scale", 2, [f_on - 2, f_on + 3], [0.0, w])
    pos = g.n("GeometryNodeInputPosition").outputs[0]
    fsum, dsum = None, None
    for e, live in presses:
        oi = g.n("GeometryNodeObjectInfo", transform_space="ORIGINAL")
        oi.inputs["Object"].default_value = e
        loc, sc = oi.outputs["Location"], oi.outputs["Scale"]
        lx, ly, lz = g.sep(loc)
        r = g.sep(sc)[0]
        px, py, _ = g.sep(pos)
        d2 = g.xyz(g.math("SUBTRACT", px, lx), g.math("SUBTRACT", py, ly), 0.0)
        dist = g.vmath("LENGTH", d2)
        f = g.maprange(dist, r, g.math("MULTIPLY", r, 0.4))
        if live:
            wgt = g.maprange(lz, 0.10, 0.045)
        else:
            wgt = g.sep(sc)[2]
        f = g.math("MULTIPLY", f, wgt)
        dvec = g.vmath("SCALE", g.vmath("NORMALIZE", d2), scale=f)
        fsum = f if fsum is None else g.math("MAXIMUM", fsum, f)
        dsum = dvec if dsum is None else g.vmath("ADD", dsum, dvec)
    dx, dy, _ = g.sep(dsum)
    axis = g.vmath("NORMALIZE", g.xyz(g.math("MULTIPLY", dy, -1.0), dx, 0.0))
    out_link = iop.outputs["Instances"].links[0]
    dst = out_link.to_socket
    sci = g.n("GeometryNodeScaleInstances")
    g.put(sci.inputs["Instances"], iop.outputs["Instances"])
    g.put(sci.inputs["Scale"], g.xyz(g.math("ADD", 1.0, g.math("MULTIPLY", fsum, 0.35)),
                                     g.math("ADD", 1.0, g.math("MULTIPLY", fsum, 0.35)),
                                     g.math("SUBTRACT", 1.0, g.math("MULTIPLY", fsum, 0.8))))
    sci.inputs["Local Space"].default_value = True
    aa = g.n("FunctionNodeAxisAngleToRotation")
    g.put(aa.inputs["Axis"], axis)
    g.put(aa.inputs["Angle"], g.math("MULTIPLY", fsum, 1.15))
    roi = g.n("GeometryNodeRotateInstances")
    g.put(roi.inputs["Instances"], sci.outputs[0])
    g.put(roi.inputs["Rotation"], aa.outputs[0])
    g.put(roi.inputs["Pivot Point"], pos)
    roi.inputs["Local Space"].default_value = False
    sel = g.n("FunctionNodeCompare", data_type="FLOAT", operation="GREATER_THAN")
    g.put(sel.inputs["A"], fsum)
    sel.inputs["B"].default_value = 1e-4
    g.put(sci.inputs["Selection"], sel.outputs[0])
    g.put(roi.inputs["Selection"], sel.outputs[0])
    ng.links.new(roi.outputs[0], dst)
    say("grass: Density input + %d presses" % len(presses))


# =============================================================================================================
# soil fx: crumbs (GN, ballistic in the node tree) and mound rings at the drills
def mat_soil():
    m = bpy.data.materials.get("fx:soil") or bpy.data.materials.new("fx:soil")
    nt = m.node_tree
    b = nt.nodes.get("Principled BSDF")
    noise = nt.nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 900.0
    noise.inputs["Detail"].default_value = 4.0
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].color = (0.035, 0.022, 0.013, 1)
    ramp.color_ramp.elements[1].color = (0.11, 0.075, 0.045, 1)
    nt.links.new(noise.outputs["Fac"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], b.inputs["Base Color"])
    b.inputs["Roughness"].default_value = 0.85
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Distance"].default_value = 0.0006
    nt.links.new(noise.outputs["Fac"], bump.inputs["Height"])
    nt.links.new(bump.outputs["Normal"], b.inputs["Normal"])
    return m


def crumb_sources(scene, mat):
    src = coll(scene, "FX_SRC", coll(scene, "FX"), exclude=True)
    rng = np.random.default_rng(5)
    import bmesh
    for i in range(6):
        bm = bmesh.new()
        bmesh.ops.create_icosphere(bm, subdivisions=1, radius=0.5)
        for v in bm.verts:
            v.co *= rng.uniform(0.7, 1.15)
        bmesh.ops.scale(bm, vec=(1.0, rng.uniform(0.6, 1.0), rng.uniform(0.45, 0.8)), verts=bm.verts)
        me = bpy.data.meshes.new("fx:crumb_%d" % i)
        bm.to_mesh(me)
        bm.free()
        me.materials.append(mat)
        ob = bpy.data.objects.new("fx:crumb_%d" % i, me)
        src.objects.link(ob)
    return src


def crumb_nodes(src):
    g = NG("GS_soil_crumbs")
    geo = g.gi.outputs[0]
    t = g.n("GeometryNodeInputSceneTime").outputs["Frame"]
    t0, tl, gg = g.attr("t0"), g.attr("tl"), g.attr("g")
    dt = g.math("MINIMUM", g.math("MAXIMUM", g.math("SUBTRACT", t, t0), 0.0), tl)
    v0 = g.attr("v0", "FLOAT_VECTOR")
    drop = g.math("MULTIPLY", g.math("MULTIPLY", g.math("MULTIPLY", dt, dt), gg), -0.5)
    off = g.vmath("ADD", g.vmath("SCALE", v0, scale=dt), g.xyz(0.0, 0.0, drop))
    sp = g.n("GeometryNodeSetPosition")
    g.put(sp.inputs["Geometry"], geo)
    g.put(sp.inputs["Offset"], off)
    dele = g.n("GeometryNodeDeleteGeometry", domain="POINT")
    g.put(dele.inputs["Geometry"], sp.outputs[0])
    g.put(dele.inputs["Selection"], g.math("LESS_THAN", t, t0))
    ci = g.n("GeometryNodeCollectionInfo", transform_space="ORIGINAL")
    ci.inputs["Collection"].default_value = src
    ci.inputs["Separate Children"].default_value = True
    ci.inputs["Reset Children"].default_value = True
    iop = g.n("GeometryNodeInstanceOnPoints")
    g.put(iop.inputs["Points"], dele.outputs[0])
    g.put(iop.inputs["Instance"], ci.outputs[0])
    iop.inputs["Pick Instance"].default_value = True
    g.put(iop.inputs["Instance Index"], g.attr("pick", "INT"))
    rot = g.vmath("ADD", g.attr("rot0", "FLOAT_VECTOR"), g.vmath("SCALE", g.attr("spin", "FLOAT_VECTOR"), scale=dt))
    e2r = g.n("FunctionNodeEulerToRotation")
    g.put(e2r.inputs[0], rot)
    g.put(iop.inputs["Rotation"], e2r.outputs[0])
    size = g.attr("size")
    g.put(iop.inputs["Scale"], g.xyz(size, size, size))
    g.ng.links.new(iop.outputs[0], g.go.inputs[0])
    return g.ng


def drill_axes(mo, models, fq):
    """world origin and direction (toward the tip) of both auger axes for frames fq"""
    DJ = getattr(models, "DART_JOINTS", {})
    out = {}
    fi = np.clip(np.round(fq).astype(int), 0, mo.nf - 1)
    for side in ("L", "R"):
        jn = "dart:drill_leg_" + side
        J = mo.M["joints"].get(jn)
        piv = np.asarray(J["pivot"] if J else DJ["dart:pivot_drill_" + side]["pivot"], float)
        ax = np.asarray(J["axis"] if J else (0, -1, 0), float)
        deg = mo.joint_deg(jn, fq)
        dirs = []
        for d in np.radians(deg):
            R = Matrix.Rotation(float(d), 3, Vector(ax))
            dirs.append(R @ Vector((-1.0, 0.0, 0.0)))
        dirs = np.array(dirs)
        q = mo.dart_q[fi]
        org = mo.dart_loc[fi] + qrot(q, piv)
        out[side] = (org, qrot(q, dirs))
    return out


def auger_radius():
    ob = bpy.data.objects.get("dart:drill_leg_L_auger")
    if ob is None or ob.type != "MESH":
        return 0.0055
    v = np.array([vv.co[:] for vv in ob.data.vertices])
    return float(np.percentile(np.hypot(v[:, 1], v[:, 2]), 98))


def build_fx(scene, mo, models, env, report):
    fx = coll(scene, "FX")
    mat = mat_soil()
    src = crumb_sources(scene, mat)
    rng = np.random.default_rng(1021)
    rows = []                                          # p0, v0, t0, tl, g, size, rot0, spin, pick

    def add(p0, v0, t0, gfr, size, ground_z):
        vz = v0[2]
        z0 = p0[2] - ground_z - size * 0.35
        tl = (vz + math.sqrt(max(vz * vz + 2 * gfr * max(z0, 0.0), 0.0))) / gfr
        rows.append((p0, v0, t0, tl, gfr, size, rng.uniform(0, 6.3, 3), rng.normal(0, 0.35, 3),
                     int(rng.integers(0, 6))))

    # impact (slow motion about 6x at the impact): film frame units
    f_imp = mo.e("impact", 717)
    nose = mo.dart_point((0.0, 0.0, 0.0), f_imp)
    s = 6.0 * mo.fps
    gfr = G / s ** 2
    vdir = mo.dart_cg[f_imp] - mo.dart_cg[f_imp - 3]
    head = math.atan2(vdir[1], vdir[0])
    for _ in range(46):
        a = head + rng.normal(0, 0.9)
        sp = rng.uniform(0.35, 1.5)
        el = rng.uniform(0.5, 1.2)
        v = np.array([math.cos(a) * math.cos(el), math.sin(a) * math.cos(el), math.sin(el)]) * sp / s
        gz = env.terrain_height(nose[0], nose[1])
        p0 = np.array([nose[0], nose[1], gz]) + np.r_[rng.normal(0, 0.008, 2), 0.004]
        add(p0, v, f_imp + rng.uniform(0, 3), gfr, rng.uniform(0.0015, 0.004), gz)
    # drills: crumbs spill out of both holes while the augers screw in (real time)
    f_d0, f_d1 = mo.e("drills_start", 1102), mo.e("anchored", 1393)
    fq = np.arange(f_d0, f_d1 + 1, dtype=float)
    axes = drill_axes(mo, models, fq)
    gfr = G / mo.fps ** 2
    entry = {}
    rad = auger_radius()
    for side, (org, dirs) in axes.items():
        E = []
        for o, d in zip(org, dirs):
            gz = env.terrain_height(o[0], o[1])
            sdist = (gz - o[2]) / d[2] if d[2] < -1e-6 else 0.0
            E.append(o + d * max(sdist, 0.0))
        E = np.array(E)
        entry[side] = E
        cg = mo.dart_cg[f_d1]
        away = math.atan2(E[-1][1] - cg[1], E[-1][0] - cg[0])
        n = 150
        tt = np.sort(f_d0 + (f_d1 - 40 - f_d0) * rng.random(n) ** 1.4)
        for t0 in tt:
            i = int(np.clip(t0 - f_d0, 0, len(E) - 1))
            a = away + rng.uniform(-1.9, 1.9)
            r0 = rad * rng.uniform(0.9, 1.4)
            gz = env.terrain_height(E[i][0], E[i][1])
            p0 = np.array([E[i][0] + r0 * math.cos(a), E[i][1] + r0 * math.sin(a), gz + 0.004])
            sp = rng.uniform(0.03, 0.28)
            v = np.array([math.cos(a) * sp, math.sin(a) * sp, rng.uniform(0.05, 0.32)]) / mo.fps
            add(p0, v, float(t0), gfr, rng.uniform(0.0012, 0.0035), gz)
    me = bpy.data.meshes.new("fx:soil_crumbs")
    me.vertices.add(len(rows))
    me.vertices.foreach_set("co", np.array([r[0] for r in rows], np.float32).ravel())
    for nm, col_i, dt in (("v0", 1, "FLOAT_VECTOR"), ("t0", 2, "FLOAT"), ("tl", 3, "FLOAT"), ("g", 4, "FLOAT"),
                          ("size", 5, "FLOAT"), ("rot0", 6, "FLOAT_VECTOR"), ("spin", 7, "FLOAT_VECTOR"),
                          ("pick", 8, "INT")):
        a = me.attributes.new(nm, dt, "POINT")
        vals = np.array([r[col_i] for r in rows])
        if dt == "FLOAT_VECTOR":
            a.data.foreach_set("vector", vals.astype(np.float32).ravel())
        elif dt == "INT":
            a.data.foreach_set("value", vals.astype(np.int32))
        else:
            a.data.foreach_set("value", vals.astype(np.float32))
    me.update()
    ob = bpy.data.objects.new("fx:soil_crumbs", me)
    fx.objects.link(ob)
    md = ob.modifiers.new("crumbs", "NODES")
    md.node_group = crumb_nodes(src)
    # mound rings round each hole, growing while the auger screws in
    import bmesh
    for side, E in entry.items():
        bm = bmesh.new()
        nseg, nring = 36, 6
        r_in, r_out, h = rad + 0.0012, rad + 0.016, 0.0055
        rows_v = []
        for j in range(nring + 1):
            u = j / nring
            rr = r_in + (r_out - r_in) * u
            zz = h * math.sin(math.pi * min(1.0, 0.15 + 0.85 * u)) * (1 - u) ** 0.3 + (-0.0025 if j == nring else 0)
            ring = []
            for i in range(nseg):
                a = 2 * math.pi * i / nseg
                jit = 1 + 0.18 * math.sin(3 * a + 1.3 * j) * rng.uniform(0.6, 1.0)
                ring.append(bm.verts.new((rr * jit * math.cos(a), rr * jit * math.sin(a),
                                          zz * rng.uniform(0.75, 1.15))))
            rows_v.append(ring)
        for j in range(nring):
            for i in range(nseg):
                i2 = (i + 1) % nseg
                bm.faces.new((rows_v[j][i], rows_v[j][i2], rows_v[j + 1][i2], rows_v[j + 1][i]))
        me = bpy.data.meshes.new("fx:mound_" + side)
        bm.to_mesh(me)
        bm.free()
        me.materials.append(mat)
        for p in me.polygons:
            p.use_smooth = True
        mo_ob = bpy.data.objects.new("fx:mound_" + side, me)
        fx.objects.link(mo_ob)
        e = E[-1]
        mo_ob.location = (e[0], e[1], env.terrain_height(e[0], e[1]) - 0.0008)
        fk = [f_d0 - 1, f_d0 + 25, f_d0 + 110, f_d1]
        sc = [(0.25, 0.25, 0.0), (0.5, 0.5, 0.35), (0.85, 0.85, 0.8), (1.0, 1.0, 1.0)]
        for i in range(3):
            key_curve(mo_ob, "scale", i, fk, [v[i] for v in sc])
        for fc in all_fcurves(mo_ob):
            for k in fc.keyframe_points:
                k.interpolation = "BEZIER"
            fc.update()
    report["fx"] = dict(crumbs=len(rows), drill_entry={k: np.round(v[-1], 4).tolist() for k, v in entry.items()},
                        auger_radius=round(rad, 4))
    say("fx:", report["fx"])
    return entry


# =============================================================================================================
# cameras
class Shot:
    def __init__(self, n, name, f0, f1, fn, still, fstop=4.0, sensor=36.0):
        self.n, self.name, self.f0, self.f1, self.fn, self.still, self.fstop = n, name, f0, f1, fn, still, fstop
        self.sensor = sensor
        self.cam = None

    @property
    def label(self):
        return "%02d_%s" % (self.n, self.name)


def make_shots(mo):
    E = mo.e
    f_aim, f_cock, f_fire = E("aim_start", 120), E("cock_start", 285), E("fire", 513)
    f_cocked, f_exit, f_imp = E("cocked", 435), E("muzzle_exit", 566), E("impact", 717)
    f_flip, f_right, f_drill = E("righting_legFlip", 866), E("righted", 1082), E("drills_start", 1102)
    f_anch, f_taut, f_tow = E("anchored", 1393), E("reel_taut", 1458), E("tow_start", 1565)
    f_hold, f_end = E("tension_hold", 1655), mo.nf - 1
    cg, cgs = mo.dart_cg, mo.cg_s

    def P(f):
        return cg[int(np.clip(round(f), 0, mo.nf - 1))]

    def PS(f):
        return cgs[int(np.clip(round(f), 0, mo.nf - 1))]

    rest = cg[f_end]
    # shot boundaries
    b = [0, f_aim, f_cock, f_fire - 18, f_exit + 34, f_imp - 6, f_flip - 8, f_right + 6, f_anch + 10,
         f_hold - 5, f_end]

    def wide(f):
        u = ease(f, b[0], b[1] + 10)
        return dict(pos=lerp((-2.45, -2.75, 1.05), (-2.05, -2.30, 0.92), u),
                    target=lerp((1.6, 0.55, 0.05), (1.3, 0.45, 0.1), u), lens=24.0, focus=(0.0, 0.0, 0.2))

    def aim(f):
        u = ease(f, b[1], b[2])
        return dict(pos=lerp((0.78, 0.86, 0.36), (0.62, 0.78, 0.44), u),
                    target=lerp((0.10, 0.08, 0.20), (0.08, 0.08, 0.25), u), lens=50.0, focus=(0.08, 0.10, 0.22))

    def cock(f):
        u = ease(f, b[2], f_cocked + 20)
        tgt = lerp((0.16, 0.10, 0.22), (0.08, 0.10, 0.20), u)
        return dict(pos=lerp((0.42, 0.55, 0.40), (0.34, 0.52, 0.38), u), target=tgt, lens=55.0,
                    focus=lerp((0.16, 0.10, 0.20), (0.10, 0.11, 0.19), u))

    def fire(f):
        u = ease(f, f_fire, b[4])
        tx = PS(max(f, f_fire))
        tgt = lerp((0.10, 0.09, 0.26), tx, smoother((f - f_fire) / 30.0))
        pos = lerp((0.18, 1.10, 0.36), (0.55, 1.12, 0.55), u)
        return dict(pos=pos, target=tgt, lens=40.0, focus=P(max(f, f_fire - 1)))

    def flight(f):
        u = ease(f, b[4], b[5])
        off = lerp((-0.75, -1.25, 0.30), (-0.70, -0.85, 0.12), u)
        tgt = PS(f + 2)
        return dict(pos=PS(f - 6) + off, target=tgt, lens=35.0, focus=P(f))

    def impact(f):
        u = ease(f, b[5], b[6])
        return dict(pos=lerp((4.80, -0.62, 0.075), (4.84, -0.58, 0.08), u),
                    target=lerp(PS(f), (4.90, 0.12, 0.03), 0.65 if f < f_imp else 0.75), lens=40.0, focus=P(f))

    def righting(f):
        u = ease(f, b[6], b[7])
        c = PS(f)
        return dict(pos=lerp((5.30, -0.36, 0.17), (5.24, -0.44, 0.15), u),
                    target=c + np.array([0.0, 0.0, 0.01]), lens=55.0, focus=P(f))

    def drilling(f):
        u = ease(f, b[7], b[8])
        tgt = lerp(rest + np.array([0.0, 0.0, -0.01]), rest + np.array([0.0, 0.0, -0.015]), u)
        return dict(pos=lerp(rest + np.array([0.30, -0.22, 0.09]), rest + np.array([0.20, -0.26, 0.06]), u),
                    target=tgt, lens=60.0, focus=tgt)

    def reel(f):
        u = ease(f, b[8], f_taut + 40)
        v = ease(f, f_taut, b[9])
        pos = lerp((-0.55, 0.42, 0.46), (-0.95, 0.20, 0.42), v)
        tgt = lerp((-0.18, 0.09, 0.24), (1.6, 0.07, 0.12), v)
        return dict(pos=pos + np.array([0.04, 0.0, 0.0]) * (1 - u), target=tgt, lens=lerp(40.0, 35.0, v),
                    focus=lerp((-0.18, 0.09, 0.24), (0.9, 0.08, 0.2), v))

    def endwide(f):
        u = ease(f, b[9], b[10] + 20)
        return dict(pos=lerp((-1.6, -2.4, 0.75), (-2.4, -3.3, 1.55), u),
                    target=lerp((2.0, 0.2, 0.05), (2.3, 0.3, 0.0), u), lens=24.0, focus=(1.5, 0.1, 0.1))

    return [
        Shot(1, "establishing", b[0], b[1] - 1, wide, 60, fstop=8.0),
        Shot(2, "aim", b[1], b[2] - 1, aim, 220, fstop=4.0),
        Shot(3, "cock", b[2], b[3] - 1, cock, 400, fstop=4.0),
        Shot(4, "fire", b[3], b[4] - 1, fire, 545, fstop=4.0),
        Shot(5, "flight", b[4], b[5] - 1, flight, 660, fstop=5.6),
        Shot(6, "impact", b[5], b[6] - 1, impact, f_imp + 6, fstop=2.8),
        Shot(7, "righting", b[6], b[7] - 1, righting, 1003, fstop=4.0),
        Shot(8, "drilling", b[7], b[8] - 1, drilling, 1300, fstop=5.6),
        Shot(9, "reel_taut", b[8], b[9] - 1, reel, 1520, fstop=5.6),
        Shot(10, "end_wide", b[9], b[10], endwide, 1715, fstop=8.0),
    ]


def build_cameras(scene, mo, report):
    col = coll(scene, "CAMERAS")
    shots = make_shots(mo)
    for m in list(scene.timeline_markers):
        scene.timeline_markers.remove(m)
    info = []
    for sh in shots:
        cd = bpy.data.cameras.new("cam:" + sh.label)
        cd.sensor_width = sh.sensor
        cd.clip_start = 0.005
        cd.clip_end = 1500.0
        cd.dof.use_dof = True
        cd.dof.aperture_fstop = sh.fstop
        cd.dof.aperture_blades = 7
        ob = bpy.data.objects.new("cam:" + sh.label, cd)
        col.objects.link(ob)
        ob.rotation_mode = "QUATERNION"
        fr = np.arange(sh.f0, sh.f1 + 1, dtype=float)
        LOC, ROT, LENS, FOC = [], [], [], []
        prev = None
        for f in fr:
            d = sh.fn(f)
            pos = Vector(np.asarray(d["pos"], float))
            tgt = Vector(np.asarray(d["target"], float))
            q = (tgt - pos).to_track_quat("-Z", "Y")
            q = np.array(q)
            if prev is not None and np.dot(q, prev) < 0:
                q = -q
            prev = q
            LOC.append(pos[:])
            ROT.append(q)
            LENS.append(float(d.get("lens", 35.0)))
            fp = Vector(np.asarray(d.get("focus", d["target"]), float))
            fwd = (tgt - pos).normalized()
            FOC.append(max(0.02, (fp - pos).dot(fwd)))
        LOC, ROT = np.array(LOC), np.array(ROT)
        for i in range(3):
            key_curve(ob, "location", i, fr, LOC[:, i])
        for i in range(4):
            key_curve(ob, "rotation_quaternion", i, fr, ROT[:, i])
        key_curve(cd, "lens", 0, fr, LENS)
        key_curve(cd, "dof.focus_distance", 0, fr, FOC)
        mk = scene.timeline_markers.new(sh.label, frame=sh.f0)
        mk.camera = ob
        ob["still"] = sh.still
        sh.cam = ob
        info.append(dict(shot=sh.label, frames=[sh.f0, sh.f1], still=sh.still))
    scene.camera = shots[0].cam
    report["shots"] = info
    say("cameras:", [(i["shot"], i["frames"]) for i in info])
    return shots


# =============================================================================================================
def build(args, report):
    import env
    import models
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.name = "grappstar_mission"
    mo = Motion(MOTION_JSON)
    say("motion.json: %d frames, %d objects, %d joints, line %d keys" % (
        mo.nf, len(mo.M["objects"]), len(mo.M.get("joints", {})), len(mo.M["line"]["frames"])))
    # where the dart comes to rest (grass kept clear just under it; the presses flatten the rest)
    rest = mo.dart_cg[-1]
    f_imp = mo.e("impact", 717)
    lie = mo.dart_cg[max(mo.e("righting_legFlip", 866) - 5, f_imp + 1)]       # where it lies before righting
    clear = [(float(rest[0]), float(rest[1]), 0.10)]
    board_half = tuple(0.5 * s for s in getattr(models, "BOARD_SIZE", (1.0, 0.8))[:2])
    res = env.build_env(scene, args.quality, clear_zones=clear, board_clear=True, board_half=board_half)
    report["env"] = res["stats"]
    say("env built", res["stats"]["grass"]["instances"], "grass clumps; clear zones", clear)
    models.build_models(scene)
    say("models built")
    scene.frame_start, scene.frame_end = 0, mo.nf - 1
    scene.render.fps, scene.render.fps_base = mo.fps, 1.0
    apply_objects(mo, report)
    apply_joints(mo, models, report)
    apply_springs(mo, models, report)
    hide_standins(report)
    nose_imp = mo.dart_point((0.0, 0.0, 0.0), f_imp)
    rest_press = [("fx:press_impact", nose_imp[:2], 0.07, f_imp, 0.75),
                  ("fx:press_lie", lie[:2], 0.10, f_imp + 25, 0.8)]
    grass_mods(scene, mo, rest_press)
    build_line(scene, mo, models, env, report)
    build_fx(scene, mo, models, env, report)
    shots = build_cameras(scene, mo, report)
    for e in mo.M.get("events", []):
        scene.timeline_markers.new("ev:" + e["name"], frame=int(e["frame"]))
    env.set_render(scene, "EEVEE", "final" if args.quality in ("final", "hero") else "preview", motion_blur=True)
    scene.render.motion_blur_shutter = 0.5
    scene.frame_set(0)
    scene["grappstar_build"] = json.dumps(dict(quality=args.quality, motion_frames=mo.nf,
                                               built=time.strftime("%Y-%m-%d %H:%M")))
    return scene, mo, shots


def save(scene, report):
    os.makedirs(OUT, exist_ok=True)
    try:
        bpy.ops.file.pack_all()
    except Exception as ex:
        say("pack_all failed:", ex)
    unpacked = [i.name for i in bpy.data.images if i.source == "FILE" and not i.packed_file]
    bpy.ops.wm.save_as_mainfile(filepath=BLEND, compress=False, relative_remap=True)
    try:
        bpy.ops.file.make_paths_relative()
    except Exception as ex:
        say("make_paths_relative:", ex)
    bpy.ops.wm.save_as_mainfile(filepath=BLEND, compress=True, relative_remap=True)
    mb = os.path.getsize(BLEND) / 1e6
    report["blend"] = dict(path=BLEND, mb=round(mb, 1), unpacked_images=unpacked)
    say("saved %s (%.1f MB), unpacked images: %s" % (BLEND, mb, unpacked))


# =============================================================================================================
# checks
def find_shots(scene):
    out = []
    mk = sorted([m for m in scene.timeline_markers if m.camera], key=lambda m: m.frame)
    for i, m in enumerate(mk):
        f1 = mk[i + 1].frame - 1 if i + 1 < len(mk) else scene.frame_end
        out.append((m.name, m.frame, f1, m.camera))
    return out


def render_shots(scene, which, stills, samples=48):
    import env
    shots = find_shots(scene)
    keep = dict(engine=scene.render.engine, x=scene.render.resolution_x, y=scene.render.resolution_y,
                fp=scene.render.filepath)
    env.set_render(scene, "CYCLES", "preview", motion_blur=True, samples=samples)
    scene.cycles.device = "CPU"
    scene.render.resolution_x, scene.render.resolution_y = 640, 360
    scene.render.resolution_percentage = 100
    scene.render.use_stamp = False
    files = []
    for i, (name, f0, f1, cam) in enumerate(shots):
        if which != "all" and str(i + 1) not in which.split(","):
            continue
        f = int(stills.get(name, (f0 + f1) // 2))
        scene.frame_set(f)
        scene.camera = cam
        path = os.path.join(OUT, "shot_%s.png" % name)
        scene.render.filepath = path
        t = time.time()
        bpy.ops.render.render(write_still=True)
        say("rendered %s frame %d in %.0fs" % (path, f, time.time() - t))
        files.append((path, name, f))
    contact_sheet()
    scene.render.engine = keep["engine"]
    return files


def contact_sheet():
    files = sorted(f for f in os.listdir(OUT) if f.startswith("shot_") and f.endswith(".png"))
    if not files:
        return
    inputs, filt = [], []
    for i, f in enumerate(files):
        inputs += ["-i", os.path.join(OUT, f)]
        lab = f[5:-4].replace("_", " ")
        filt.append("[%d:v]scale=480:270,drawtext=text='%s':x=8:y=8:fontsize=18:fontcolor=white:box=1:"
                    "boxcolor=black@0.5[v%d]" % (i, lab, i))
    cols = 3
    rows = (len(files) + cols - 1) // cols
    while len(files) < rows * cols:
        inputs += ["-f", "lavfi", "-i", "color=c=black:s=480x270"]
        filt.append("[%d:v]null[v%d]" % (len(files), len(files)))
        files.append("pad")
    layout = "|".join("%d_%d" % (480 * (i % cols), 270 * (i // cols)) for i in range(len(files)))
    filt.append("".join("[v%d]" % i for i in range(len(files))) + "xstack=inputs=%d:layout=%s[out]" % (len(files),
                                                                                                        layout))
    cmd = ["ffmpeg", "-y", "-loglevel", "error"] + inputs + ["-filter_complex", ";".join(filt), "-map", "[out]",
                                                             "-frames:v", "1", os.path.join(OUT, "shots_sheet.png")]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:                                   # no fonts: plain tiles
        say("contact sheet labels failed:", r.stderr[-300:])
        cmd = [re.sub(r",drawtext=[^\[]*", "", c) for c in cmd]
        r = subprocess.run(cmd, capture_output=True, text=True)
    say("contact sheet", os.path.join(OUT, "shots_sheet.png"), "ok" if r.returncode == 0 else r.stderr[-300:])


def motion_check(scene, step=2, density=0.5, samples=8, engine="EEVEE"):
    import env
    d = os.path.join(FRAMES_DIR, "mc")
    os.makedirs(d, exist_ok=True)
    g = bpy.data.objects.get("env:grass")
    if g is not None:
        md = [m for m in g.modifiers if m.type == "NODES"][0]
        try:
            sid = [it.identifier for it in md.node_group.interface.items_tree
                   if getattr(it, "name", "") == "Density"][0]
            try:
                getattr(md.properties.inputs, sid).value = density
            except AttributeError:
                md[sid] = density
        except IndexError:
            pass
    env.set_render(scene, engine, "preview", motion_blur=True, samples=samples)
    r = scene.render
    r.resolution_x, r.resolution_y, r.resolution_percentage = 320, 180, 100
    if engine == "CYCLES":
        scene.cycles.device = "CPU"
        scene.cycles.samples = samples
    r.use_stamp = True
    for k in ("use_stamp_date", "use_stamp_time", "use_stamp_render_time", "use_stamp_memory", "use_stamp_hostname",
              "use_stamp_lens", "use_stamp_scene", "use_stamp_filename", "use_stamp_frame_range",
              "use_stamp_sequencer_strip", "use_stamp_note", "use_stamp_camera"):
        if hasattr(r, k):
            setattr(r, k, False)
    r.use_stamp_frame = True
    r.use_stamp_marker = True
    r.stamp_font_size = 9
    r.image_settings.file_format = "JPEG"
    r.image_settings.quality = 88
    r.use_overwrite = False
    r.use_placeholder = True
    scene.frame_step = step
    r.filepath = os.path.join(d, "f_")
    t = time.time()
    bpy.ops.render.render(animation=True)
    say("motion check frames in %.0fs" % (time.time() - t))
    encode_motion_check(d, step)


def encode_motion_check(d, step=2):
    out = os.path.join(OUT, "motion_check.mp4")
    lst = os.path.join(d, "list.txt")
    files = sorted(f for f in os.listdir(d) if f.startswith("f_") and f.endswith(".jpg"))
    with open(lst, "w") as fh:
        for f in files:
            fh.write("file '%s'\nduration %.6f\n" % (os.path.join(d, f), step / 30.0))
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", lst, "-vf",
           "fps=30,format=yuv420p", "-c:v", "libx264", "-crf", "26", "-preset", "slow", out]
    subprocess.run(cmd, check=True)
    say("motion check video %s (%.1f MB, %d frames)" % (out, os.path.getsize(out) / 1e6, len(files)))


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--quality", default="final", choices=["draft", "preview", "final", "hero"])
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--shots", default="", help="'all' or comma list of shot numbers: Cycles 640x360 stills")
    ap.add_argument("--still", default="", help="override still frames, e.g. 4:540,6:725")
    ap.add_argument("--samples", type=int, default=48)
    ap.add_argument("--motion-check", action="store_true")
    ap.add_argument("--mc-engine", default="EEVEE", choices=["EEVEE", "CYCLES"])
    ap.add_argument("--mc-step", type=int, default=2)
    ap.add_argument("--check-only", action="store_true", help="load the saved .blend instead of building")
    a = ap.parse_args(argv)
    report = {}
    if a.check_only:
        bpy.ops.wm.open_mainfile(filepath=BLEND)
        scene = bpy.context.scene
        shots = None
    else:
        scene, mo, shots = build(a, report)
        if not a.no_save:
            save(scene, report)
        with open(os.path.join(OUT, "build_report.json"), "w") as fh:
            json.dump(report, fh, indent=1, default=str)
    stills = {}
    if shots:
        stills = {sh.label: sh.still for sh in shots}
    else:
        for name, f0, f1, cam in find_shots(scene):
            stills[name] = cam.get("still", (f0 + f1) // 2)
    if a.still:
        names = [s[0] for s in find_shots(scene)]
        for kv in a.still.split(","):
            k, v = kv.split(":")
            stills[names[int(k) - 1]] = int(v)
    if a.shots:
        render_shots(scene, a.shots, stills, a.samples)
    if a.motion_check:
        motion_check(scene, step=a.mc_step, engine=a.mc_engine)
    say("done")


if __name__ == "__main__":
    main()
