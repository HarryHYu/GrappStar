import trimesh, sys, numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
PK = sys.argv[1]
import glob, os
PL = sorted(os.path.basename(f)[:-4] for f in glob.glob(f"{PK}/plate_*.3mf"))
fig = plt.figure(figsize=(7 * len(PL), 6))
for i, pl in enumerate(PL):
    sc = trimesh.load(f"{PK}/{pl}.3mf"); ax = fig.add_subplot(1, len(PL), i + 1, projection="3d")
    for name, g in sc.geometry.items():
        m = g.copy(); 
        try: m = m.simplify_quadric_decimation(face_count=4000)
        except Exception: pass
        tri = m.vertices[m.faces]; nz = m.face_normals[:, 2]
        shade = 0.45 + 0.5 * np.clip(m.face_normals @ np.array([0.3, -0.5, 0.8]), 0, 1)
        pc = Poly3DCollection(tri, facecolors=np.c_[shade, shade, shade * 0.95, np.ones_like(shade)], linewidths=0); ax.add_collection3d(pc)
        c = m.bounds.mean(0); ax.text(c[0], c[1], m.bounds[1][2] + 4, name, fontsize=6)
    ax.set_xlim(-120, 120); ax.set_ylim(-120, 120); ax.set_zlim(0, 80); ax.set_box_aspect((240, 240, 80)); ax.view_init(35, -60); ax.set_title(pl, fontsize=9)
fig.tight_layout(); fig.savefig(f"{PK}/plates.png", dpi=110); print("ok")
