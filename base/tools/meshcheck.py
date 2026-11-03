"""Gear mesh check across owners: declared Mesh pairs must sit at the right centre distance, with parallel axes,
overlapping faces and a joint ratio that matches the teeth. Called by tools.audit (a failure makes it NOT CLEAN).

For every Mesh(gear_a, gear_b, m, z_a, z_b, ...) from the subsystems' optional meshes(P), in every given state where
Mesh.when(S) is true (default: every state):
- axes parallel within 0.5 deg (gear axis = part-local +Z through frame_a / frame_b),
- centre distance = gears.center_distance (spur), internal_center_distance (internal) or the rack pitch line
  distance (rack), within Mesh.tol,
- face overlap along the axis >= 80 % of the narrower face,
- if the two gears ride on joints linked by Joint.gear (directly, either way), the ratio matches the teeth.
Joints with a gear link but no Mesh between their parts are listed as warnings (idler chains, coaxial links).
"""
from __future__ import annotations

import math

import numpy as np

from cad.lib import tf, gears

AXIS_TOL_DEG = 0.5
FACE_MIN = 0.8
RATIO_TOL = 1e-3


def _frame(asm, W, name, local):
    M = W[name] @ (np.eye(4) if local is None else np.asarray(local, float))
    return M[:3, 3], M[:3, 2] / np.linalg.norm(M[:3, 2]), M


def _face(asm, name, local, face):
    """Axial interval (z0, z1) of the gear in its gear frame."""
    if face is not None:
        return 0.0, float(face)
    from cad.lib import prim
    s = asm.parts[name].solid
    if local is not None:
        s = prim.place(s, tf.inv(np.asarray(local, float)))
    lo, hi = prim.bbox(s)
    return lo[2], hi[2]


def expected_distance(ms) -> float:
    if ms.kind == "internal":
        return gears.internal_center_distance(ms.m, ms.z_b, ms.z_a, pa=ms.pa, x_ring=ms.x_b, x_pinion=ms.x_a)
    if ms.kind == "rack":
        return gears.rack_center_distance(ms.m, ms.z_a, ms.x_a)
    return gears.center_distance(ms.m, ms.z_a, ms.z_b, ms.x_a, ms.x_b, ms.pa)


def check_one(asm, ms, S, W=None) -> list[str]:
    """Failure strings for one mesh in one state (empty = pass)."""
    for g in (ms.gear_a, ms.gear_b):
        if g not in asm.parts:
            return [f"mesh {ms.gear_a} / {ms.gear_b}: unknown part {g}"]
    W = W or asm.world(S)
    pa_, za, Ma = _frame(asm, W, ms.gear_a, ms.frame_a)
    pb_, zb, Mb = _frame(asm, W, ms.gear_b, ms.frame_b)
    fails = []
    tag = f"mesh {ms.gear_a} / {ms.gear_b} (m{ms.m:g} z{ms.z_a}/{ms.z_b} {ms.kind})"
    want = expected_distance(ms)
    if ms.kind == "rack":
        # gear axis must be along the rack's Z, at want from the rack pitch line (rack y = 0)
        ang = math.degrees(math.acos(min(1.0, abs(float(np.dot(za, zb))))))
        p_in_rack = tf.apply(tf.inv(Mb), [pa_])[0]
        dist = abs(p_in_rack[1])
        axial_a = (0.0, 0.0)
    else:
        ang = math.degrees(math.acos(min(1.0, abs(float(np.dot(za, zb))))))
        d = pb_ - pa_
        dist = float(np.linalg.norm(d - np.dot(d, za) * za))
    if ang > AXIS_TOL_DEG:
        fails.append(f"{tag}: axes not parallel ({ang:.2f} deg)")
    if abs(dist - want) > ms.tol:
        fails.append(f"{tag}: centre distance {dist:.3f}, expected {want:.3f} (tol {ms.tol})")
    # face overlap along gear a's axis
    a0, a1 = _face(asm, ms.gear_a, ms.frame_a, ms.face_a)
    b0, b1 = _face(asm, ms.gear_b, ms.frame_b, ms.face_b)
    sgn = 1.0 if float(np.dot(za, zb)) >= 0 else -1.0
    off = float(np.dot(pb_ - pa_, za))
    bb = sorted((off + sgn * b0, off + sgn * b1))
    ov = max(0.0, min(a1, bb[1]) - max(a0, bb[0]))
    narrow = min(a1 - a0, b1 - b0)
    if narrow <= 0 or ov < FACE_MIN * narrow:
        fails.append(f"{tag}: face overlap {ov:.2f} of {narrow:.2f} mm (need {FACE_MIN:.0%})")
    return fails


def check_ratio(asm, ms) -> list[str]:
    ja, jb = asm.parts[ms.gear_a].joint, asm.parts[ms.gear_b].joint
    if ja is None or jb is None or ja == jb:
        return []
    Ja, Jb = asm.joints[ja], asm.joints[jb]
    link = None
    if Jb.gear is not None and Jb.gear[0] == ja:
        link = ("b_from_a", float(Jb.gear[1]))
    elif Ja.gear is not None and Ja.gear[0] == jb:
        link = ("a_from_b", float(Ja.gear[1]))
    if link is None:
        return []
    W = asm.world({})
    za = (W[ms.gear_a] @ (np.eye(4) if ms.frame_a is None else ms.frame_a))[:3, 2]
    zb = (W[ms.gear_b] @ (np.eye(4) if ms.frame_b is None else ms.frame_b))[:3, 2]
    # joint axes in world (rev joints rotate about their axis; compare with the gear axes)
    JW = asm.joint_world({})
    axa = JW[ja][:3, :3] @ np.asarray(Ja.axis)
    axb = JW[jb][:3, :3] @ np.asarray(Jb.axis)
    tag = f"mesh {ms.gear_a} / {ms.gear_b}"
    if ms.kind == "rack":
        if Ja.kind != "rev" or Jb.kind != "slide":
            return []
        mm_per_deg = math.pi * ms.m * ms.z_a / 360.0
        want = mm_per_deg if link[0] == "b_from_a" else 1.0 / mm_per_deg
        got = abs(link[1])
        return [] if abs(got - want) <= RATIO_TOL * want else [f"{tag}: rack ratio {got:.5f}, teeth give {want:.5f}"]
    if Ja.kind != "rev" or Jb.kind != "rev":
        return []
    # speed of b about ITS gear axis per unit speed of a about ITS gear axis
    sa = float(np.sign(np.dot(axa, za)) or 1.0)
    sb = float(np.sign(np.dot(axb, zb)) or 1.0)
    par = float(np.sign(np.dot(za, zb)) or 1.0)
    want = (ms.z_a / ms.z_b) * (1.0 if ms.kind == "internal" else -1.0) * par   # in b-axis terms, for a about a-axis
    r = link[1]
    got = r * sb * sa if link[0] == "b_from_a" else (1.0 / r) * sb * sa if r else math.inf
    return [] if abs(got - want) <= RATIO_TOL * abs(want) else [
        f"{tag}: joint ratio {got:+.5f} (gear axis terms), teeth give {want:+.5f}"]


def run(asm, states) -> dict:
    res = dict(meshes=[], fails=[], warns=[])
    Ws = [asm.world(S) for S in states]
    for ms in asm.meshes:
        n, f_all = 0, []
        when = getattr(ms, "when", None)
        for S, W in zip(states, Ws):
            if when is not None and not when(S):
                continue
            n += 1
            for f in check_one(asm, ms, S, W):
                if f not in f_all:
                    f_all.append(f + f" (first at phase {S.get('phase')}, t {S.get('t')})")
        try:
            f_all += check_ratio(asm, ms) if ms.gear_a in asm.parts and ms.gear_b in asm.parts else []
        except Exception as e:                  # never let a ratio check crash the audit
            f_all.append(f"mesh {ms.gear_a} / {ms.gear_b}: ratio check error {e}")
        res["meshes"].append(dict(a=ms.gear_a, b=ms.gear_b, m=ms.m, z=[ms.z_a, ms.z_b], kind=ms.kind,
                                  expected=round(expected_distance(ms), 3), states=n, ok=not f_all, fails=f_all))
        res["fails"] += f_all
    # gear-linked joints without any declared mesh between their parts
    on = {}
    for p in asm.parts.values():
        on.setdefault(p.joint, set()).add(p.name)
    meshed = {frozenset((asm.parts[m.gear_a].joint, asm.parts[m.gear_b].joint)) for m in asm.meshes
              if m.gear_a in asm.parts and m.gear_b in asm.parts}
    for j in asm.joints.values():
        if j.gear is not None and abs(abs(j.gear[1]) - 1.0) > 1e-9 and frozenset((j.name, j.gear[0])) not in meshed:
            res["warns"].append(f"joint {j.name} is geared to {j.gear[0]} (ratio {j.gear[1]:g}) but no Mesh is declared "
                                f"between their parts: declare meshes(P) so the centre distance is checked")
    return res
