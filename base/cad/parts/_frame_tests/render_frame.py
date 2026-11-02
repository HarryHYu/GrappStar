"""Quick look renders of the frame (matplotlib, no browser): 6 views + 2 section cuts -> OUT/frame_views.png.
python cad/parts/_frame_tests/render_frame.py OUT [--only frame,launch]   (run from base/)"""
import os, sys, argparse
BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, BASE)
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.colors import to_rgb
from cad.lib import prim
from tools.assembly import Assembly
from tools.params import load_params


def shade(v, f, col, light=np.array([0.4, -0.5, 0.8])):
    tri = v[f]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    k = 0.45 + 0.55 * np.clip(n @ (light / np.linalg.norm(light)), 0, 1)
    return np.clip(np.outer(k, to_rgb(col)), 0, 1)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("out"); ap.add_argument("--only", default="frame")
    ap.add_argument("--state", default="")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    asm = Assembly.load(load_params(), [s for s in a.only.split(",") if s])
    S = {}
    W = asm.world(S)
    meshes = []
    for n, p in asm.parts.items():
        s = prim.place(p.solid, W[n]).simplify(0.05) if hasattr(prim.place(p.solid, W[n]), "simplify") else prim.place(p.solid, W[n])
        v, f = prim.mesh_of(s)
        meshes.append((n, v, f, p.color))
    allv = np.vstack([m[1] for m in meshes]); lo, hi = allv.min(0), allv.max(0); c = (lo + hi) / 2; r = (hi - lo).max() / 2
    views = [(25, -60), (25, 30), (25, 120), (25, 210), (90, -90), (0, -90)]
    fig = plt.figure(figsize=(24, 16))
    for i, (el, az) in enumerate(views):
        ax = fig.add_subplot(2, 4, i + 1, projection="3d")
        for n, v, f, col in meshes:
            pc = Poly3DCollection(v[f], facecolors=shade(v, f, col), edgecolor="none")
            ax.add_collection3d(pc)
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.view_init(el, az); ax.set_box_aspect((1, 1, 1)); ax.set_title(f"el {el} az {az}")
        ax.set_xlabel("x"); ax.set_ylabel("y")
    # sections: y = 0 plane (XZ) and x = main plane (YZ)
    for k, (axis, val) in enumerate((("y", float(os.environ.get("SEC_Y", 0.0))), ("x", float(os.environ.get("SEC_X", 0.0))))):
        ax = fig.add_subplot(2, 4, 7 + k)
        for n, p in asm.parts.items():
            s = prim.place(p.solid, W[n])
            if axis == "y":
                s2 = s.rotate((90, 0, 0))   # world y -> -z ; slice at z = -val
                cs = s2.slice(-val)
            else:
                s2 = s.rotate((0, -90, 0))
                cs = s2.slice(val)
            for poly in cs.to_polygons():
                pts = np.asarray(poly)
                if len(pts) > 2:
                    hx, vz = (pts[:, 0], -pts[:, 1]) if axis == "y" else (pts[:, 1], -pts[:, 0])
                    ax.fill(hx, vz, color=p.color, alpha=0.8, lw=0.3, ec="k")
        ax.set_aspect("equal"); ax.set_title(f"section {axis} = {val}"); ax.grid(True, lw=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, "frame_views.png"), dpi=60)
    print("wrote", os.path.join(a.out, "frame_views.png"))


if __name__ == "__main__":
    main()
