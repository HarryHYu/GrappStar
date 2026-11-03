"""Quick look renders of the drum subsystem (matplotlib, no browser): 3D views and section cuts in the L frame.

Run from base/:  python -m cad.parts._drum_views OUTDIR [drum_deg ...]
"""
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from tools.params import load_params
from tools.assembly import Assembly
from cad.lib import prim, tf


def posed_L(asm, S):
    """Every drum part posed in the drum_base frame L (z along the main axis, y down)."""
    W = asm.world(S)
    JW = asm.joint_world(S)
    inv = np.linalg.inv(JW["drum_base"])
    out = {}
    for n, p in asm.parts.items():
        out[n] = (prim.place(p.solid, inv @ W[n]), p.color)
    return out


def view3d(posed, ax, elev, azim):
    for n, (m, col) in posed.items():
        mm = m.to_mesh()
        v = np.asarray(mm.vert_properties)[:, :3]
        f = np.asarray(mm.tri_verts)
        if len(f) > 6000:
            f = f[:: max(1, len(f) // 6000)]
        tri = v[f]
        # L -> plot: x forward, z axis along shaft, up = -y
        tri = tri[:, :, [0, 2, 1]] * np.array([1, 1, -1])
        nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
        shade = 0.45 + 0.55 * np.clip(nrm @ np.array([0.3, -0.5, 0.8]), 0, 1)
        rgb = np.array(matplotlib.colors.to_rgb(col))
        pc = Poly3DCollection(tri, facecolors=np.clip(rgb[None, :] * shade[:, None], 0, 1), edgecolors="none")
        ax.add_collection3d(pc)
    ax.set_xlim(-60, 140); ax.set_ylim(-10, 190); ax.set_zlim(-170, 30)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev, azim)
    ax.set_xlabel("x fwd"); ax.set_ylabel("s (main axis)"); ax.set_zlabel("up")


def section_z(posed, ax, s, title):
    for n, (m, col) in posed.items():
        cs = m.slice(s)
        for poly in cs.to_polygons():
            p = np.asarray(poly)
            if len(p) < 3:
                continue
            ax.fill(p[:, 0], -p[:, 1], color=col, alpha=0.85, lw=0.3, ec="k")
    ax.set_aspect("equal"); ax.set_title(title, fontsize=8); ax.grid(alpha=0.3)


def section_x(posed, ax, x, title):
    """Cut by the plane x = const (longitudinal), plot s horizontally, up vertically."""
    R = tf.Ry(-90)            # maps x -> z
    for n, (m, col) in posed.items():
        mm = prim.place(m, R).translate((0, 0, 0))
        cs = mm.slice(x)
        for poly in cs.to_polygons():
            p = np.asarray(poly)
            if len(p) < 3:
                continue
            # after Ry(-90): new x = z(s), new y = y
            ax.fill(p[:, 0], -p[:, 1], color=col, alpha=0.85, lw=0.3, ec="k")
    ax.set_aspect("equal"); ax.set_title(title, fontsize=8); ax.grid(alpha=0.3)


def main(argv):
    out = argv[0] if argv else "/tmp/drum_views"
    degs = [float(a) for a in argv[1:]] or [0.0, 40.0, 80.0, 200.0, 338.0]
    os.makedirs(out, exist_ok=True)
    P = load_params()
    asm = Assembly.load(P, ["drum"])
    for d in degs:
        posed = posed_L(asm, dict(drum_deg=d, wiper_deg=0.0))
        fig = plt.figure(figsize=(16, 11))
        ax = fig.add_subplot(2, 3, 1, projection="3d"); view3d(posed, ax, 20, -60)
        ax = fig.add_subplot(2, 3, 2, projection="3d"); view3d(posed, ax, 10, -150)
        section_x(posed, fig.add_subplot(2, 3, 3), 0.0, f"section x=0 (drum {d:g})")
        section_z(posed, fig.add_subplot(2, 3, 4), 91.8, "section s=91.8 (fork pin plane)")
        section_z(posed, fig.add_subplot(2, 3, 5), 63.0, "section s=63 (lock layer)")
        section_z(posed, fig.add_subplot(2, 3, 6), 72.0, "section s=72 (idler gear)")
        fig.tight_layout()
        fn = os.path.join(out, f"drum_{int(d):03d}.png")
        fig.savefig(fn, dpi=80)
        plt.close(fig)
        print(fn)


if __name__ == "__main__":
    main(sys.argv[1:])
