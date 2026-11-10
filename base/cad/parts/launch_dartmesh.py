"""Build launch_dart_v34_solid.npz (one manifold) and launch_dart_v34.json from kedge/out34 (KEDGE v34, folded).

    python cad/parts/launch_dartmesh.py [KEDGE_OUT]      (default ../../kedge/out34 from this folder)

Folded pose = every joint at 0 deg. Each part mesh is in its own local frame: a root part sits at world =
T(pos) * RotY(sgn * deg) * local, a child at parent * T(pos) * local (kedge/launcher_fit.py _posed()); at 0 deg that is
the sum of pos up the parent chain. Components that are not closed manifolds are skipped; launch.dart_solid() adds the
hull and the nose as solids (R_OUT, NOSE_L, L, NOSE_R0 from parts.json P, kept in the json here).
Dart frame (= base's launch_dart frame): nose tip x = 0, tail face x = 152, y left, z up, belly z = -21.
"""
import json
import math
import os
import sys

import numpy as np
import trimesh
import manifold3d as m3

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(HERE, "..", "..", "..", "kedge", "out34"))


def _T(p):
    M = np.eye(4)
    M[:3, 3] = p
    return M


def posed(out, deg=0.0):
    J = json.load(open(os.path.join(out, "parts.json")))
    PP = {p["name"]: p for p in J["parts"]}

    def M(name):
        p = PP[name]
        if p.get("parent"):
            return M(p["parent"]) @ _T(p["pos"])
        R = _T(p["pos"])
        if p.get("axis"):
            a = math.radians(p.get("sgn", 1) * deg)
            c, s = math.cos(a), math.sin(a)
            R = R @ np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])
        return R
    return J, PP, M


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else SRC
    J, PP, M = posed(out)
    ms, names = [], []
    for n in PP:
        z = np.load(os.path.join(out, n + ".npz"))
        v = np.asarray(z["v"], float)
        v = (np.c_[v, np.ones(len(v))] @ M(n).T)[:, :3]
        tm = trimesh.Trimesh(v, np.asarray(z["f"], np.int64), process=True)
        for c in tm.split(only_watertight=False):
            try:
                S = m3.Manifold(m3.Mesh(vert_properties=np.asarray(c.vertices, np.float32),
                                        tri_verts=np.asarray(c.faces, np.uint32)))
            except Exception:
                continue
            if S.status() == m3.Error.NoError and S.volume() > 0:
                ms.append(S)
        names.append(n)
    U = m3.Manifold.batch_boolean(ms, m3.OpType.Add)
    mm = U.to_mesh()
    np.savez_compressed(os.path.join(HERE, "launch_dart_v34_solid.npz"), v=mm.vert_properties[:, :3].astype(np.float32),
                        f=mm.tri_verts.astype(np.uint32))
    P = J["P"]
    lo, hi = U.bounding_box()[:3], U.bounding_box()[3:]
    info = dict(source="kedge/out34 parts.json + per-part npz, folded (every joint 0 deg)",
                frame="nose tip x=0, x to the tail, y left, z up, hull axis y=z=0, belly z=-21, tail face x=152",
                R_OUT=P["R_OUT"], NOSE_L=P["NOSE_L"], L=P["L"], NOSE_R0=P["NOSE_R0"], TIE=P.get("TIE"),
                bbox_min=[round(float(x), 2) for x in lo], bbox_max=[round(float(x), 2) for x in hi],
                components=len(ms), parts=names)
    json.dump(info, open(os.path.join(HERE, "launch_dart_v34.json"), "w"), indent=1)
    print(len(ms), "components, volume", round(U.volume()), "bbox", info["bbox_min"], info["bbox_max"])


if __name__ == "__main__":
    main()
