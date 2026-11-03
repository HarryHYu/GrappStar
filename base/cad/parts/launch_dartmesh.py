"""Build launch_dart_v30_solid.npz (one manifold) from launch_dart_v30.npz (all KEDGE v30 out30 parts, folded).
Components that are not closed manifolds are skipped; launch.dart_solid() adds the hull and nose as solids."""
import os
import numpy as np
import trimesh
import manifold3d as m3

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    d = np.load(os.path.join(HERE, "launch_dart_v30.npz"))
    tm = trimesh.Trimesh(d["v"], d["f"], process=True)
    ok = []
    for c in tm.split(only_watertight=False):
        try:
            M = m3.Manifold(m3.Mesh(vert_properties=np.asarray(c.vertices, np.float32),
                                    tri_verts=np.asarray(c.faces, np.uint32)))
        except Exception:
            continue
        if M.status() == m3.Error.NoError and M.volume() > 0:
            ok.append(M)
    U = m3.Manifold.batch_boolean(ok, m3.OpType.Add)
    mm = U.to_mesh()
    np.savez_compressed(os.path.join(HERE, "launch_dart_v30_solid.npz"), v=mm.vert_properties[:, :3].astype(np.float32),
                        f=mm.tri_verts.astype(np.uint32))
    print(len(ok), "components", U.volume())


if __name__ == "__main__":
    main()
