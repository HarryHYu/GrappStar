"""Quick shaded render of STL meshes (matplotlib, no GPU) for design iteration.
python3 qrender.py out.png "elev,azim[,zoom]" file1.stl[:color] file2.stl ...   (several views: "20,-60;10,30")"""
import sys, numpy as np, trimesh, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def render(out, views, items, size=7.0):
    ms = []
    for it in items:
        f, _, c = it.partition(":")
        m = trimesh.load(f, process=False)
        ms.append((m, c or "#9aa5b1"))
    allv = np.vstack([m.vertices for m, _ in ms]); lo, hi = allv.min(0), allv.max(0); ctr = (lo + hi) / 2; span = (hi - lo).max()
    V = [tuple(float(x) for x in v.split(",")) for v in views.split(";")]
    fig = plt.figure(figsize=(size * len(V), size))
    light = np.array([0.4, -0.5, 0.8]); light /= np.linalg.norm(light)
    for k, v in enumerate(V):
        el, az = v[0], v[1]; zoom = v[2] if len(v) > 2 else 1.0
        ax = fig.add_subplot(1, len(V), k + 1, projection="3d")
        for m, c in ms:
            tri = m.vertices[m.faces]; n = m.face_normals
            sh = 0.35 + 0.65 * np.clip(n @ light, 0, 1)
            base = np.array(matplotlib.colors.to_rgb(c))
            col = np.clip(base[None, :] * sh[:, None], 0, 1)
            pc = Poly3DCollection(tri, facecolors=col, edgecolors="none", linewidths=0)
            ax.add_collection3d(pc)
        h = span / 2 / zoom
        ax.set_xlim(ctr[0] - h, ctr[0] + h); ax.set_ylim(ctr[1] - h, ctr[1] + h); ax.set_zlim(ctr[2] - h, ctr[2] + h)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(el, az); ax.set_axis_off()
    plt.tight_layout(); plt.savefig(out, dpi=90); plt.close()

if __name__ == "__main__":
    render(sys.argv[1], sys.argv[2], sys.argv[3:])
