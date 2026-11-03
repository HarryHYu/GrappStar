"""Quick look views of the launcher (worker cad-launch): side silhouettes and section cuts, PNG via matplotlib.

    cd base && python cad/parts/launch_view.py OUTDIR [--only launch,...]

Needs the cad-infra tools (base/tools). Views:
  side_<state>.png   silhouettes projected on the x-z plane (world), one colour per part
  secy_<state>.png   section in the plane y = rail centre (world)
  secu_<state>.png   sections across the rail at several u (rail frame)
"""
from __future__ import annotations

import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                               # noqa: E402
from matplotlib.collections import PolyCollection             # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(os.path.dirname(HERE))
for p in (BASE, HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

from tools.params import load_params                          # noqa: E402
from tools.assembly import Assembly                           # noqa: E402
from cad.lib import prim, tf                                  # noqa: E402

STATES = {
    "rest_p6": dict(aim_deg=6.02, cock=0.0),
    "cocked_p6": dict(aim_deg=6.02, cock=1.0),
    "cocked_p47": dict(aim_deg=47.0, cock=1.0),
    "rest_p47": dict(aim_deg=47.0, cock=0.0),
}


def _polys(v, f, ax0, ax1):
    return v[f][:, :, [ax0, ax1]]


def side(asm, S, path, title):
    W = asm.world(S)
    fig, ax = plt.subplots(figsize=(14, 8))
    for name, part in asm.parts.items():
        v, f = prim.mesh_of(prim.place(part.solid, W[name]))
        pc = PolyCollection(_polys(v, f, 0, 2), facecolor=part.color, edgecolor="none", alpha=0.35)
        ax.add_collection(pc)
    ax.autoscale()
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.set_title(title)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("z (mm)")
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)


def _draw_cs(ax, cs, color, label=None):
    for poly in cs.to_polygons():
        poly = np.asarray(poly)
        if len(poly) < 3:
            continue
        ax.fill(poly[:, 0], poly[:, 1], color=color, alpha=0.6, lw=0.3, ec="k")


def sec_y(asm, S, y, path, title):
    W = asm.world(S)
    M = tf.Rx(-90)                                   # z' = -y, section coords (x, z)
    fig, ax = plt.subplots(figsize=(14, 8))
    for name, part in asm.parts.items():
        s = prim.place(part.solid, tf.mul(M, W[name]))
        cs = s.slice(-y)
        if cs.area() > 0:
            _draw_cs(ax, cs, part.color)
    ax.set_aspect("equal")
    ax.autoscale()
    ax.grid(True, lw=0.3)
    ax.set_title(title)
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)


def sec_u(asm, S, us, path, title, rail_joint="launch_rail"):
    W = asm.world(S)
    JW = asm.joint_world(S)
    Minv = np.linalg.inv(JW[rail_joint])
    R = tf.Ry(-90)                                   # z' = u, section coords (-w, y)
    fig, axs = plt.subplots(1, len(us), figsize=(4.2 * len(us), 5.5))
    for ax, u in zip(axs, us):
        for name, part in asm.parts.items():
            s = prim.place(part.solid, tf.mul(R, Minv, W[name]))
            cs = s.slice(u)
            if cs.area() > 0:
                pts = [np.asarray(p) for p in cs.to_polygons()]
                for p in pts:
                    if len(p) >= 3:
                        ax.fill(p[:, 1], -p[:, 0], color=part.color, alpha=0.6, lw=0.3, ec="k")
        ax.set_aspect("equal")
        ax.autoscale()
        ax.grid(True, lw=0.3)
        ax.set_title(f"u = {u:g}")
        ax.set_xlabel("y rel")
    fig.suptitle(title)
    fig.savefig(path, dpi=100, bbox_inches="tight")
    plt.close(fig)


def main(argv=None):
    argv = argv or sys.argv[1:]
    out = argv[0] if argv else "out_launch_view"
    only = ["launch"]
    if "--only" in argv:
        only = argv[argv.index("--only") + 1].split(",")
    os.makedirs(out, exist_ok=True)
    P = load_params()
    asm = Assembly.load(P, only)
    import launch as LL
    n = LL.L(P)
    for k, S in STATES.items():
        side(asm, S, os.path.join(out, f"side_{k}.png"), f"side {k}")
        sec_y(asm, S, n["Y0"], os.path.join(out, f"secy_{k}.png"), f"section y = {n['Y0']:g} {k}")
    sec_u(asm, STATES["rest_p6"], [-150, 0, 40, 82.6, 150, 245], os.path.join(out, "secu_rest.png"), "rest, sections across the rail")
    sec_u(asm, STATES["cocked_p6"], [-150, 0, 40, 82.6, 150, 245], os.path.join(out, "secu_cocked.png"), "cocked")
    print("views in", out)


if __name__ == "__main__":
    main()
