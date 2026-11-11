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
STILL_FRAMES = {}
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


def gn_input(md, name, value=None):
    """get / set a geometry-nodes modifier input by socket name (Blender 5.x API with 4.x fallback)"""
    sid = [it.identifier for it in md.node_group.interface.items_tree
           if getattr(it, "name", "") == name and getattr(it, "in_out", "") == "INPUT"][0]
    try:
        holder = getattr(md.properties.inputs, sid)
        if value is not None:
            holder.value = value
        return holder.value
    except AttributeError:
        if value is not None:
            md[sid] = value
        return md[sid]


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
        global DART_CG_X
        DART_CG_X = float(M.get("dart", {}).get("cg_station_m", DART_CG_X))
        d = M["objects"]["dart:root"]
        self.dart_loc = sample(d["frames"], d["loc"], self.F)
        self.dart_q = sample_quat(d["frames"], d["quat"], self.F)
        self.dart_cg = self.dart_loc + qrot(self.dart_q, np.array([DART_CG_X, 0.0, 0.0]))
        self.cg_s = gauss_smooth(self.dart_cg, 4.0)
        self.cg_s2 = gauss_smooth(self.dart_cg, 10.0)          # calmer target for the ground shots

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


def resample_by_length(Q, n):
    L = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(Q, axis=0), axis=1))]
    t = np.linspace(0.0, L[-1], n)
    return np.stack([np.interp(t, L, Q[:, d]) for d in range(3)], 1)


def reroute_lip(mo, P, fr, kf):
    """motion.json lets the free rope fall straight from the rail exit (pusher hole) once the dart is out, i.e.
    through the rail and the base. Real line runs along the rail and over the lip roller at the muzzle, then
    hangs. For every key whose free rope dips below the exit -> roller-top line behind the roller, rebuild the
    start of the free rope: exit -> along the rail -> over the roller top -> smooth hang into the rope's first
    point in front of the roller; blended in with the dip depth so nothing pops. P (K, n, 3), kf = exit index."""
    d = mo.M["objects"].get("base:launch_muzzle_roller")
    ob = bpy.data.objects.get("base:launch_muzzle_roller")
    if d is None or ob is None:
        return P, 0
    cl = np.mean([v[:] for v in ob.bound_box], axis=0)
    rr = 0.5 * float(np.max(np.ptp(np.array([v[:] for v in ob.bound_box]), axis=0)))     # roller radius
    C = sample(d["frames"], d["loc"], fr) + qrot(sample_quat(d["frames"], d["quat"], fr), cl)
    out = P.copy()
    fixed = 0
    for i in range(len(fr)):
        free = P[i, kf:]
        M = len(free)
        ex = free[0]
        dv = C[i] - ex
        if dv[0] <= 0.02:
            continue
        dv = dv / np.linalg.norm(dv)
        nrm = np.array([0.0, 0.0, 1.0]) - dv * dv[2]
        nrm = nrm / np.linalg.norm(nrm)
        top = C[i] + nrm * (rr + ROPE_R)
        dl = top - ex
        beh = free[:, 0] < top[0]
        zl = ex[2] + (free[:, 0] - ex[0]) * dl[2] / dl[0]
        depth = float(np.max((zl - free[:, 2])[beh], initial=0.0))
        w = float(smoother((depth - 0.004) / 0.016))
        if w <= 0.0:
            continue
        ahead = np.where(free[:, 0] > top[0] + 0.02)[0]
        if not len(ahead):
            continue
        j = int(ahead[0])
        pj = free[j]
        tj = free[min(j + 1, M - 1)] - pj
        tj = tj / (np.linalg.norm(tj) + 1e-9)
        dist = np.linalg.norm(pj - top)
        c1 = top + dv * 0.3 * dist * np.array([1.0, 1.0, 0.0]) / max(np.linalg.norm(dv[:2]), 1e-6)
        c2 = pj - tj * 0.3 * dist
        u = np.linspace(0, 1, 14)[1:-1, None]
        bez = (1 - u) ** 3 * top + 3 * (1 - u) ** 2 * u * c1 + 3 * (1 - u) * u ** 2 * c2 + u ** 3 * pj
        tail = resample_by_length(np.vstack([top, bez, free[j:]]), M - 2)
        new = np.vstack([ex, 0.5 * (ex + top), tail])
        out[i, kf:] = free * (1 - w) + new * w
        fixed += 1
    return out, fixed


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
    PP, nfix = reroute_lip(mo, PP, fr, kb - 1)
    PP = drape(PP, ground, kb)
    mat = rope_material(models)
    K, N, _ = PP.shape
    ng = cache_curve_nodes("GS_line_tether", N, kb, K, mat)
    ob = cache_object("line:tether", col, fr, PP, ng)
    report["line"] = dict(keys=K, points=N, base_points=kb, keys_rerouted_over_lip=nfix)
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
def dart_length(root):
    """x extent (dart frame) of the dart's meshes as built (folded)"""
    Ri = root.matrix_world.inverted()
    xs = []
    for ob in bpy.data.objects:
        o, inside = ob, False
        while o is not None:
            if o == root:
                inside = True
                break
            o = o.parent
        if not inside or ob.type != "MESH":
            continue
        M = Ri @ ob.matrix_world
        xs += [(M @ Vector(c)).x for c in ob.bound_box]
    if not xs:
        return 0.0, 0.16
    return float(np.percentile(xs, 2)), float(np.percentile(xs, 98))


MOW = 0.42                          # grass height factor in the mown patches (env:grass modifier input "Mow")


def grass_mods(scene, mo, rest_press, mow_zones):
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
    gn_input(md, "Density", 1.0)
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

    x0, x1 = dart_length(root)
    for i, x in enumerate(np.linspace(x0 + 0.005, x1 - 0.005, 5)):        # along the body, nose to tail
        press("fx:press_%d" % i, root, (float(x), 0.0, 0.0), 0.10)
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
    # mown / grazed-short patches round the landing spot and the base (soft, noisy edge): "Mow" scales blade
    # height there so the 16 cm dart and the board read in the grass
    s2 = ng.interface.new_socket("Mow", in_out="INPUT", socket_type="NodeSocketFloat")
    s2.default_value, s2.min_value, s2.max_value = MOW, 0.1, 1.0
    gn_input(md, "Mow", MOW)
    nz = g.n("ShaderNodeTexNoise")
    nz.inputs["Scale"].default_value = 0.9
    g.put(nz.inputs["Vector"], pos)
    wmax = None
    mowf = None
    for (cx, cy, r1, r2, strength) in mow_zones:
        px, py, _ = g.sep(pos)
        dd = g.vmath("LENGTH", g.xyz(g.math("SUBTRACT", px, cx), g.math("SUBTRACT", py, cy), 0.0))
        dd = g.math("ADD", dd, g.math("MULTIPLY", g.math("SUBTRACT", nz.outputs["Fac"], 0.5), min(0.9, r2 * 0.5)))
        w = g.maprange(dd, r2, r1)
        lvl = g.gi.outputs["Mow"] if strength is None else strength
        fz = g.math("ADD", 1.0, g.math("MULTIPLY", w, g.math("SUBTRACT", lvl, 1.0)))
        mowf = fz if mowf is None else g.math("MINIMUM", mowf, fz)
    dirv = g.vmath("NORMALIZE", dsum)
    dst = iop.outputs["Instances"].links[0].to_socket
    sci = g.n("GeometryNodeScaleInstances")
    g.put(sci.inputs["Instances"], iop.outputs["Instances"])
    g.put(sci.inputs["Scale"], g.xyz(g.math("ADD", 1.0, g.math("MULTIPLY", fsum, 0.2)),
                                     g.math("ADD", 1.0, g.math("MULTIPLY", fsum, 0.2)),
                                     g.math("MULTIPLY", mowf, g.math("SUBTRACT", 1.0, g.math("MULTIPLY", fsum, 0.85)))))
    sci.inputs["Local Space"].default_value = True
    # pressed clumps become flat mats pushed a little out from under the dart (no tilt: a tilted mat would lift
    # its inner blades into the body)
    tri = g.n("GeometryNodeTranslateInstances")
    g.put(tri.inputs["Instances"], sci.outputs[0])
    g.put(tri.inputs["Translation"], g.vmath("SCALE", dirv, scale=g.math("MULTIPLY", fsum, 0.015)))
    tri.inputs["Local Space"].default_value = False
    sel = g.n("FunctionNodeCompare", data_type="FLOAT", operation="GREATER_THAN")
    g.put(sel.inputs["A"], fsum)
    sel.inputs["B"].default_value = 1e-4
    g.put(tri.inputs["Selection"], sel.outputs[0])
    ng.links.new(tri.outputs[0], dst)
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
    ramp.color_ramp.elements[0].color = (0.085, 0.058, 0.036, 1)
    ramp.color_ramp.elements[1].color = (0.23, 0.16, 0.10, 1)
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
    grow = g.math("MINIMUM", g.math("MAXIMUM", g.math("DIVIDE", g.math("SUBTRACT", t, t0),
                                                    g.math("MAXIMUM", g.attr("grow"), 0.01)), 0.0), 1.0)
    size = g.math("MULTIPLY", g.attr("size"), grow)
    g.put(iop.inputs["Scale"], g.xyz(size, size, g.math("MULTIPLY", size, g.attr("flat"))))
    g.ng.links.new(iop.outputs[0], g.go.inputs[0])
    return g.ng


def auger_joints(mo):
    """motion.json joints that spin an auger -> {side: (joint name, joint dict)}; side = the L/R token in the
    name (else an index), so a dart with other pod names or hinge positions drops in"""
    out = {}
    for jn, J in mo.M.get("joints", {}).items():
        if "auger" in jn.lower() or "drill" in jn.lower() and "spin" in jn.lower():
            m = re.search(r"_([LR])(?:_|$)", jn)
            out[m.group(1) if m else str(len(out))] = (jn, J)
    return out


def drill_axes(mo, models, fq):
    """world origin (on the auger axis, at its joint origin) and direction toward the tip of every auger
    for frames fq, through its chain of parent joints (pod hinge) from motion.json"""
    out = {}
    fq = np.atleast_1d(np.asarray(fq, float))
    fi = np.clip(np.round(fq).astype(int), 0, mo.nf - 1)
    JS = mo.M.get("joints", {})
    for side, (jn, J) in auger_joints(mo).items():
        ax = np.asarray(J["axis"], float)
        ax = ax / np.linalg.norm(ax)
        org = np.tile(np.asarray(J.get("pivot", (0.0, 0.0, 0.0)), float), (len(fq), 1))
        dirs = np.tile(ax, (len(fq), 1))
        par = J.get("parent")
        while par in JS:                                   # innermost parent joint first
            P = JS[par]
            R = [Matrix.Rotation(float(d), 3, Vector(P["axis"])) for d in np.radians(mo.joint_deg(par, fq))]
            org = np.array([R[i] @ Vector(org[i]) for i in range(len(fq))]) + np.asarray(P["pivot"], float)
            dirs = np.array([R[i] @ Vector(dirs[i]) for i in range(len(fq))])
            par = P.get("parent")
        q = mo.dart_q[fi]
        out[side] = (mo.dart_loc[fi] + qrot(q, org), qrot(q, dirs))
    return out


def auger_dims(mo):
    """(radius, tip distance along the axis) of the auger meshes, from the first auger joint's object"""
    for side, (jn, J) in auger_joints(mo).items():
        ob = bpy.data.objects.get(jn)
        if ob is None or ob.type != "MESH" or not len(ob.data.vertices):
            continue
        v = np.array([vv.co[:] for vv in ob.data.vertices])
        ax = np.asarray(J["axis"], float)
        ax = ax / np.linalg.norm(ax)
        along = v @ ax
        radial = np.linalg.norm(v - along[:, None] * ax, axis=1)
        return float(np.percentile(radial, 98)), float(along.max())
    return 0.0093, 0.10


def build_fx(scene, mo, models, env, report):
    """soil: a crumb burst at the impact, crumbs spilling from both drills while they screw in, and the
    disturbed soil heaped along each drill's cut (the auger crosses the surface along a short track while its
    pod swings down): all one geometry-nodes point cloud (fx:soil_crumbs)"""
    fx = coll(scene, "FX")
    mat = mat_soil()
    src = crumb_sources(scene, mat)
    rng = np.random.default_rng(1021)
    rows = []                                          # p0, v0, t0, tl, g, size, rot0, spin, pick, grow, flat

    def add(p0, v0, t0, gfr, size, ground_z, grow=2.0, flat=1.0):
        vz = v0[2]
        z0 = p0[2] - ground_z - size * 0.35
        tl = (vz + math.sqrt(max(vz * vz + 2 * gfr * max(z0, 0.0), 0.0))) / gfr if gfr > 0 else 0.0
        rows.append((p0, v0, t0, tl, gfr, size, rng.uniform(0, 6.3, 3), rng.normal(0, 0.35, 3) * (gfr > 0),
                     int(rng.integers(0, 6)), grow, flat))

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
    # drills (real time)
    f_p0 = mo.e("pods_deploy_start", mo.e("drills_start", 1102) - 20)
    f_d1 = mo.e("anchored", 1393)
    fq = np.arange(f_p0, f_d1 + 1, dtype=float)
    axes = drill_axes(mo, models, fq)
    gfr = G / mo.fps ** 2
    rad, tip = auger_dims(mo)
    entry, contact = {}, {}
    for side, (org, dirs) in axes.items():
        E, S = [], []
        for o, d in zip(org, dirs):
            gz = env.terrain_height(o[0], o[1])
            sd = (gz - o[2]) / d[2] if d[2] < -1e-6 else 1e3
            E.append(o + d * min(max(sd, 0.0), tip))
            S.append(sd)
        E, S = np.array(E), np.array(S)
        inside = np.where(S < tip - 0.002)[0]
        if not len(inside):
            continue
        i0 = int(inside[0])
        f_c = float(fq[i0])
        entry[side], contact[side] = E, f_c
        cg = mo.dart_cg[int(f_d1)]
        away = math.atan2(E[-1][1] - cg[1], E[-1][0] - cg[0])
        # crumbs thrown out by the auger flights at the current surface crossing
        n = 260
        tt = np.sort(f_c + (f_d1 - 25 - f_c) * rng.random(n) ** 1.3)
        for t0 in tt:
            i = int(np.clip(t0 - f_p0, 0, len(E) - 1))
            a = away + rng.uniform(-2.0, 2.0)
            r0 = rad * rng.uniform(0.9, 1.3)
            gz = env.terrain_height(E[i][0], E[i][1])
            p0 = np.array([E[i][0] + r0 * math.cos(a), E[i][1] + r0 * math.sin(a), gz + 0.003])
            sp = rng.uniform(0.03, 0.25)
            v = np.array([math.cos(a) * sp, math.sin(a) * sp, rng.uniform(0.05, 0.30)]) / mo.fps
            add(p0, v, float(t0), gfr, rng.uniform(0.0012, 0.0032), gz, grow=2.0)
        # heaped soil along both sides of the cut and round the final hole, growing as the cut passes
        trk = E[i0:]
        tdir = trk[-1][:2] - trk[0][:2]
        tdir = tdir / (np.linalg.norm(tdir) + 1e-9)
        perp = np.array([-tdir[1], tdir[0]])
        L = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(trk[:, :2], axis=0), axis=1))]
        for l in np.arange(0.0, L[-1] + 1e-6, 0.0035):
            j = int(np.searchsorted(L, l))
            j = min(j, len(trk) - 1)
            c = trk[j]
            for sgn in (-1, 1):
                off = rad + rng.uniform(0.002, 0.007)
                q = c[:2] + perp * sgn * off + rng.normal(0, 0.0015, 2)
                gz = env.terrain_height(q[0], q[1])
                sz = rng.uniform(0.0035, 0.0075)
                add(np.array([q[0], q[1], gz - sz * 0.12]), np.zeros(3), float(fq[i0 + j]) + 2, 0.0, sz, gz,
                    grow=30.0, flat=rng.uniform(0.35, 0.6))
        for k in range(26):
            a = rng.uniform(0, 2 * math.pi)
            q = E[-1][:2] + (rad + rng.uniform(0.002, 0.012)) * np.array([math.cos(a), math.sin(a)])
            gz = env.terrain_height(q[0], q[1])
            sz = rng.uniform(0.003, 0.007)
            add(np.array([q[0], q[1], gz - sz * 0.12]), np.zeros(3), float(f_d1) - rng.uniform(20, 120), 0.0, sz,
                gz, grow=40.0, flat=rng.uniform(0.35, 0.6))
    me = bpy.data.meshes.new("fx:soil_crumbs")
    me.vertices.add(len(rows))
    me.vertices.foreach_set("co", np.array([r[0] for r in rows], np.float32).ravel())
    for nm, col_i, dt in (("v0", 1, "FLOAT_VECTOR"), ("t0", 2, "FLOAT"), ("tl", 3, "FLOAT"), ("g", 4, "FLOAT"),
                          ("size", 5, "FLOAT"), ("rot0", 6, "FLOAT_VECTOR"), ("spin", 7, "FLOAT_VECTOR"),
                          ("pick", 8, "INT"), ("grow", 9, "FLOAT"), ("flat", 10, "FLOAT")):
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
    report["fx"] = dict(crumbs=len(rows), drill_entry={k: np.round(v[-1], 4).tolist() for k, v in entry.items()},
                        drill_contact_frame=contact, auger_radius=round(rad, 4), auger_tip=round(tip, 4))
    say("fx:", report["fx"])
    return entry


# =============================================================================================================
# cameras
# two locked-off tripod cameras (SPEC v2): cam:A wide 3/4 side, cut at the motion.json "cut" event to cam:B
# side-on to the line. No moves, no zoom; aim and focal length are FITTED to the action from motion.json, so a
# different dart / landing spot / timing reframes itself on a rerun.
CAM_A = dict(dist=9.5, height=1.5, side=0.92, back=-0.40, fill=0.80, fstop=5.6)    # back < 0: behind the base side
CAM_B = dict(dist=2.7, height=1.2, along=1.0, fill=0.86, fstop=5.6)    # abreast of 1 m down the line from the base:
                                    # side-on but a little oblique, so the 1 mm line is about 1 px wide at 4K near
                                    # the base and still reads out to the dart
TRIPOD_SHAKE = 0.00022              # rad, F-curve noise on the camera rotation (about 1 px at 4K): a tripod in a
                                    # light breeze, not handheld; 0 disables
LOOK = dict(distortion=0.012, dispersion=0.003, vignette=0.24, grain=0.045)     # compositor "real camera" look


class Shot:
    def __init__(self, name, f0, f1, pos, target, lens, focus, fstop, stills):
        self.name, self.f0, self.f1 = name, f0, f1
        self.pos, self.target, self.lens, self.focus, self.fstop = pos, target, lens, focus, fstop
        self.stills = stills
        self.cam = None

    @property
    def label(self):
        return self.name


def fit_camera(pos, pts, fill, aspect=16 / 9, sensor=36.0, lens_range=(14.0, 200.0)):
    """aim (target point) and focal length for a camera at pos so that pts fill `fill` of the frame width or
    height (whichever binds), centred"""
    pos = np.asarray(pos, float)
    d = np.asarray(pts, float) - pos
    fwd = np.mean(d / np.linalg.norm(d, axis=1, keepdims=True), axis=0)
    fwd /= np.linalg.norm(fwd)
    for _ in range(6):
        right = np.cross(fwd, (0.0, 0.0, 1.0))
        right /= np.linalg.norm(right)
        up = np.cross(right, fwd)
        z = d @ fwd
        x, y = (d @ right) / z, (d @ up) / z
        cx, cy = 0.5 * (x.max() + x.min()), 0.5 * (y.max() + y.min())
        fwd = fwd + right * cx + up * cy
        fwd /= np.linalg.norm(fwd)
    hw, hh = 0.5 * (x.max() - x.min()), 0.5 * (y.max() - y.min())
    tan_half = max(hw, hh * aspect) / fill
    lens = float(np.clip(0.5 * sensor / tan_half, *lens_range))
    return pos + fwd * float(np.median(z)), lens


def make_shots(mo, entry=None):
    E = mo.e
    nf = mo.nf
    f_aim, f_cock, f_fire = E("aim_start", 120), E("cock_start", 285), E("fire", 513)
    f_imp = E("impact", 717)
    f_flip, f_right = E("righting_legFlip", f_imp + 150), E("righted", f_imp + 320)
    f_drill, f_taut = E("drills_start", f_right + 50), E("reel_taut", nf - 290)
    f_tow, f_end = E("tow_start", nf - 160), nf - 1
    f_cut = E("cut", E("tidy_start", f_taut - 30))
    f_cut = int(np.clip(f_cut, f_imp + 30, f_end - 30))
    cg = mo.dart_cg
    rest = cg[f_end]
    base = np.array([0.0, 0.0, 0.0])
    L = rest[:2] - base[:2]
    L = np.r_[L / np.linalg.norm(L), 0.0]
    n = np.array([-L[1], L[0], 0.0])
    if n[1] < 0:
        n = -n                                             # cameras on the +y side (sun side, aim crank side)
    board = np.array([[sx * 0.5, sy * 0.4, 0.0] for sx in (-1, 1) for sy in (-1, 1)])
    # cam:A: wide 3/4 side, sees aim .. drills start
    mid = 0.5 * (base + rest)
    mid[2] = 0.0
    uA = n * CAM_A["side"] + L * CAM_A["back"]
    uA /= np.linalg.norm(uA)
    posA = mid + uA * CAM_A["dist"] + np.array([0.0, 0.0, CAM_A["height"]])
    fl = cg[f_fire:f_imp + 1:3]
    ptsA = np.vstack([board, [[0.0, 0.0, 0.40]], fl, rest + 0.35 * L, rest - 0.35 * L, rest + 0.3 * n,
                      rest - 0.3 * n])
    tgtA, lensA = fit_camera(posA, ptsA, CAM_A["fill"])
    focA = mid + np.array([0.0, 0.0, 0.3])
    # cam:B: roughly side-on to the line, base reel to the dart
    Lk = mo.M["line"]
    P_end = np.asarray(Lk["points"][-1], float)
    reel = P_end[0]
    knot = P_end[-1]
    midB = 0.5 * (reel + knot)
    midB[2] = 0.0
    posB = base * np.array([1, 1, 0]) + L * CAM_B["along"] + n * CAM_B["dist"] + np.array([0.0, 0.0, CAM_B["height"]])
    ptsB = np.vstack([board, [[0.0, 0.0, 0.40]], P_end, rest + 0.15 * L, rest - 0.15 * L,
                      rest + np.array([0.0, 0.0, 0.08])])
    tgtB, lensB = fit_camera(posB, ptsB, CAM_B["fill"])
    focB = midB + np.array([0.0, 0.0, 0.15])
    i_ap = f_fire + int(np.argmax(cg[f_fire:f_imp + 1, 2]))
    stA = dict(aim=f_aim + (f_cock - f_aim) * 3 // 4, cock=(f_cock + f_fire) // 2, launch=f_fire + 1,
               flight=i_ap, impact=f_imp + 4, righting=(f_flip + f_right) // 2, drills=min(f_drill + 60, f_cut - 2))
    stB = dict(after_cut=f_cut + 5, taut=min(f_taut + 15, f_end), tow=min(f_tow + 40, f_end), end=f_end - 3)
    stA = {k: int(np.clip(v, 0, f_cut - 1)) for k, v in stA.items()}
    stB = {k: int(np.clip(v, f_cut, f_end)) for k, v in stB.items()}
    return [Shot("A_wide", 0, f_cut - 1, posA, tgtA, lensA, focA, CAM_A["fstop"], stA),
            Shot("B_line", f_cut, f_end, posB, tgtB, lensB, focB, CAM_B["fstop"], stB)]


def build_cameras(scene, mo, report, shots):
    col = coll(scene, "CAMERAS")
    for m in list(scene.timeline_markers):
        scene.timeline_markers.remove(m)
    rng = np.random.default_rng(77)
    info = []
    for sh in shots:
        nm = "cam:" + sh.name.split("_")[0]
        cd = bpy.data.cameras.new(nm)
        cd.sensor_width = 36.0
        cd.sensor_fit = "HORIZONTAL"
        cd.lens = sh.lens
        cd.clip_start = 0.05
        cd.clip_end = 2000.0
        cd.dof.use_dof = True
        cd.dof.aperture_fstop = sh.fstop
        cd.dof.aperture_blades = 7
        ob = bpy.data.objects.new(nm, cd)
        col.objects.link(ob)
        pos, tgt = Vector(sh.pos), Vector(sh.target)
        ob.location = pos
        ob.rotation_mode = "XYZ"
        ob.rotation_euler = (tgt - pos).to_track_quat("-Z", "Y").to_euler()
        cd.dof.focus_distance = max(0.1, (Vector(sh.focus) - pos).dot((tgt - pos).normalized()))
        if TRIPOD_SHAKE > 0:
            for i in range(3):
                fc = key_curve(ob, "rotation_euler", i, [0.0], [ob.rotation_euler[i]])
                md = fc.modifiers.new("NOISE")
                md.scale = float(rng.uniform(9.0, 16.0))
                md.strength = TRIPOD_SHAKE * (0.35 if i == 1 else 1.0)       # little roll
                md.phase = float(rng.uniform(0, 100))
                md.depth = 1
        ob["stills"] = json.dumps(sh.stills)
        mk = scene.timeline_markers.new(sh.name, frame=sh.f0)
        mk.camera = ob
        sh.cam = ob
        info.append(dict(shot=sh.name, camera=nm, frames=[sh.f0, sh.f1], lens_mm=round(sh.lens, 1),
                         pos=np.round(sh.pos, 2).tolist(), target=np.round(sh.target, 2).tolist(), stills=sh.stills))
    scene.camera = shots[0].cam
    report["shots"] = info
    say("cameras:", [(i["camera"], i["frames"], i["lens_mm"], i["pos"]) for i in info])
    return shots


def camera_look(scene):
    """compositor: mild barrel distortion with a touch of lateral CA, vignette, light per-frame film grain"""
    if not hasattr(scene, "compositing_node_group"):
        return
    ng = bpy.data.node_groups.new("GS_camera_look", "CompositorNodeTree")
    ng.interface.new_socket("Image", in_out="OUTPUT", socket_type="NodeSocketColor")
    N, Lk = ng.nodes, ng.links

    def math(op, a, b=None):
        m = N.new("ShaderNodeMath")
        m.operation = op
        for i, v in enumerate((a, b)):
            if v is None:
                continue
            if isinstance(v, bpy.types.NodeSocket):
                Lk.new(v, m.inputs[i])
            else:
                m.inputs[i].default_value = v
        return m.outputs[0]

    rl = N.new("CompositorNodeRLayers")
    rl.scene = scene
    ld = N.new("CompositorNodeLensdist")
    Lk.new(rl.outputs["Image"], ld.inputs["Image"])
    ld.inputs["Distortion"].default_value = LOOK["distortion"]
    ld.inputs["Dispersion"].default_value = LOOK["dispersion"]
    ld.inputs["Fit"].default_value = True
    ic = N.new("CompositorNodeImageCoordinates")
    Lk.new(ld.outputs["Image"], ic.inputs["Image"])
    sp = N.new("ShaderNodeSeparateXYZ")
    Lk.new(ic.outputs["Normalized"], sp.inputs[0])
    dx = math("MULTIPLY", math("SUBTRACT", sp.outputs[0], 0.5), 16 / 9)
    dy = math("SUBTRACT", sp.outputs[1], 0.5)
    r2 = math("DIVIDE", math("ADD", math("MULTIPLY", dx, dx), math("MULTIPLY", dy, dy)), 1.04)
    vig = math("SUBTRACT", 1.0, math("MULTIPLY", math("POWER", r2, 1.15), LOOK["vignette"]))
    wn = N.new("ShaderNodeTexWhiteNoise")
    wn.noise_dimensions = "4D"
    Lk.new(ic.outputs["Pixel"], wn.inputs["Vector"])
    st = N.new("CompositorNodeSceneTime")
    Lk.new(st.outputs["Frame"], wn.inputs["W"])
    gr = math("ADD", 1.0, math("MULTIPLY", math("SUBTRACT", wn.outputs["Value"], 0.5), LOOK["grain"]))
    k = math("MULTIPLY", vig, gr)
    mx = N.new("ShaderNodeMixRGB")
    mx.blend_type = "MULTIPLY"
    mx.inputs[0].default_value = 1.0
    Lk.new(ld.outputs["Image"], mx.inputs[1])
    cmb = N.new("ShaderNodeCombineXYZ")
    for i in range(3):
        Lk.new(k, cmb.inputs[i])
    Lk.new(cmb.outputs[0], mx.inputs[2])
    go = N.new("NodeGroupOutput")
    Lk.new(mx.outputs[0], go.inputs[0])
    scene.compositing_node_group = ng
    scene.render.use_compositing = True
    say("compositor: lens distortion %.3f, CA %.3f, vignette %.2f, grain %.3f" % (
        LOOK["distortion"], LOOK["dispersion"], LOOK["vignette"], LOOK["grain"]))


def build(args, report):
    import importlib
    import env
    models = importlib.import_module(os.environ.get("GRAPPSTAR_MODELS", "models"))   # test hook
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
    clear = [(float(rest[0]), float(rest[1]), 0.05)]
    f_d1 = mo.e("anchored", 1393)
    axes = drill_axes(mo, models, np.array([float(f_d1)]))
    entry0 = {}
    for side, (org, dirs) in axes.items():
        o, d = org[0], dirs[0]
        gz = float(env.terrain_height(o[0], o[1]))
        entry0[side] = [o + d * max((gz - o[2]) / d[2], 0.0) if d[2] < -1e-6 else o]
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
    rest_press = [("fx:press_impact", nose_imp[:2], 0.045, f_imp, 0.6),
                  ("fx:press_lie", lie[:2], 0.075, f_imp + 25, 0.7)]
    mow = [(float(rest[0]), float(rest[1]), 0.45, 1.1, 0.22), (float(rest[0]), float(rest[1]), 1.1, 2.6, None),
           (0.0, 0.0, 0.75, 1.6, None)]
    for side, ee in entry0.items():            # trampled round the drill holes (the drilling close-up)
        e = ee[0]
        mow.append((float(e[0]), float(e[1]), 0.05, 0.16, 0.18))
    try:
        grass_mods(scene, mo, rest_press, mow)
    except Exception as ex:                    # env.py grass tree changed shape: render without the extras
        say("WARNING grass presses / mow not added:", repr(ex))
        report["grass_mods_error"] = repr(ex)
    build_line(scene, mo, models, env, report)
    entry = build_fx(scene, mo, models, env, report)
    shots = build_cameras(scene, mo, report, make_shots(mo, entry))
    for e in mo.M.get("events", []):
        scene.timeline_markers.new("ev:" + e["name"], frame=int(e["frame"]))
    birds = getattr(env, "add_cockatoos", None)
    if callable(birds):
        try:
            birds(scene, mo.e("fire", 513))
            report["cockatoos"] = True
            say("cockatoos added (take off at the fire frame %d)" % mo.e("fire", 513))
        except Exception as ex:
            say("WARNING env.add_cockatoos failed:", repr(ex))
            report["cockatoos"] = repr(ex)
    env.set_render(scene, "EEVEE", "final" if args.quality in ("final", "hero") else "preview", motion_blur=True)
    scene.render.motion_blur_shutter = 0.5                 # 180 deg shutter
    if args.quality in ("final", "hero"):
        scene.render.resolution_x, scene.render.resolution_y = 3840, 2160      # uhd default (mac preset)
    camera_look(scene)
    scene.frame_set(0)
    scene["grappstar_build"] = json.dumps(dict(quality=args.quality, motion_frames=mo.nf,
                                               built=time.strftime("%Y-%m-%d %H:%M")))
    return scene, mo, shots


def save(scene, report):
    os.makedirs(OUT, exist_ok=True)
    try:
        bpy.context.preferences.filepaths.save_version = 0      # no .blend1 backups (disk is tight)
    except Exception:
        pass
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
    for bak in (BLEND + "1", BLEND + "2"):
        if os.path.exists(bak):
            os.remove(bak)
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


def render_stills(scene, which="all", samples=32, size=(960, 540)):
    """Cycles CPU stills at the key moments of each camera (camera custom property "stills") ->
    render/out/shot_<cam>_<moment>.png, then the contact sheet"""
    import env
    env.set_render(scene, "CYCLES", "preview", motion_blur=True, samples=samples)
    scene.render.motion_blur_shutter = 0.5
    scene.cycles.device = "CPU"
    scene.render.resolution_x, scene.render.resolution_y = size
    scene.render.resolution_percentage = 100
    scene.render.use_stamp = False
    files = []
    for name, f0, f1, cam in find_shots(scene):
        letter = name.split("_")[0]
        st = json.loads(cam.get("stills", "{}")) or {"mid": (f0 + f1) // 2}
        for idx, (lab, f) in enumerate(st.items()):
            key = "%s%d_%s" % (letter, idx + 1, lab)
            sel = which.split(",")
            if which != "all" and letter not in sel and key not in sel and "%s_%s" % (letter, lab) not in sel:
                continue
            STILL_FRAMES[key] = "f%d" % int(f)
            scene.frame_set(int(f))
            scene.camera = cam
            path = os.path.join(OUT, "shot_%s.png" % key)
            scene.render.filepath = path
            t = time.time()
            bpy.ops.render.render(write_still=True)
            say("rendered %s frame %d in %.0fs" % (path, f, time.time() - t))
            files.append(path)
    contact_sheet()
    return files


def render_frames(scene, frames, samples=32, cam_spec="", size=(640, 360), tag="check"):
    """debug stills: frames with their marker camera (or a custom camera 'x,y,z:tx,ty,tz:lens') ->
    render/out/frames/<tag>_FFFF.png"""
    import env
    env.set_render(scene, "CYCLES", "preview", motion_blur=True, samples=samples)
    scene.cycles.device = "CPU"
    scene.render.resolution_x, scene.render.resolution_y = size
    scene.render.resolution_percentage = 100
    os.makedirs(FRAMES_DIR, exist_ok=True)
    cam = None
    if cam_spec:
        p, t, lens = cam_spec.split(":")
        cd = bpy.data.cameras.new("cam:debug")
        cd.lens = float(lens)
        cd.clip_start = 0.003
        cam = bpy.data.objects.new("cam:debug", cd)
        scene.collection.objects.link(cam)
        pv, tv = Vector([float(v) for v in p.split(",")]), Vector([float(v) for v in t.split(",")])
        cam.location = pv
        cam.rotation_euler = (tv - pv).to_track_quat("-Z", "Y").to_euler()
        cd.dof.use_dof = False
    shots = find_shots(scene)
    if cam is not None:                                # markers bound to cameras would switch it back
        for m in scene.timeline_markers:
            m.camera = None
    out = []
    for f in frames:
        scene.frame_set(f)
        if cam is not None:
            scene.camera = cam
        else:
            for name, f0, f1, c in shots:
                if f0 <= f <= f1:
                    scene.camera = c
        path = os.path.join(FRAMES_DIR, "%s_%04d.png" % (tag, f))
        scene.render.filepath = path
        t0 = time.time()
        bpy.ops.render.render(write_still=True)
        say("rendered %s in %.0fs" % (path, time.time() - t0))
        out.append(path)
    return out


def contact_sheet():
    files = sorted(f for f in os.listdir(OUT) if f.startswith("shot_") and f.endswith(".png"))
    if not files:
        return
    inputs, filt = [], []
    for i, f in enumerate(files):
        inputs += ["-i", os.path.join(OUT, f)]
        lab = f[5:-4].replace("_", " ") + (" " + STILL_FRAMES.get(f[5:-4], "") if STILL_FRAMES else "")
        filt.append("[%d:v]scale=480:270,drawtext=text='%s':x=8:y=8:fontsize=18:fontcolor=white:box=1:"
                    "boxcolor=black@0.5[v%d]" % (i, lab, i))
    cols = 4
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


def motion_check(scene, step=2, density=0.5, samples=8, engine="CYCLES"):
    import env
    d = os.path.join(FRAMES_DIR, "mc")
    os.makedirs(d, exist_ok=True)
    g = bpy.data.objects.get("env:grass")
    if g is not None:
        try:
            gn_input([m for m in g.modifiers if m.type == "NODES"][0], "Density", density)
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
    ap.add_argument("--shots", default="", help="'all', camera letters 'A,B' or stills 'A_launch,B_taut': Cycles "
                                                "960x540 stills at the cameras' key moments")
    ap.add_argument("--samples", type=int, default=48)
    ap.add_argument("--motion-check", action="store_true")
    ap.add_argument("--mc-engine", default="CYCLES", choices=["EEVEE", "CYCLES"])
    ap.add_argument("--mc-step", type=int, default=2)
    ap.add_argument("--check-only", action="store_true", help="load the saved .blend instead of building")
    ap.add_argument("--frames", default="", help="debug stills of these frames (comma list) -> out/frames/")
    ap.add_argument("--cam", default="", help="custom debug camera 'x,y,z:tx,ty,tz:lens' for --frames")
    ap.add_argument("--tag", default="check")
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
    if a.shots:
        render_stills(scene, a.shots, a.samples)
    if a.frames:
        render_frames(scene, [int(v) for v in a.frames.split(",")], a.samples, a.cam, tag=a.tag)
    if a.motion_check:
        motion_check(scene, step=a.mc_step, engine=a.mc_engine)
    say("done")


if __name__ == "__main__":
    main()
