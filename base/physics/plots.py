"""Plain plots for the guide and site (black and grey, Liberation Sans): range vs pitch from base/out/ballistics.json,
and dart trajectories at a few pitches. Writes base/out/physics/range_vs_pitch.png and trajectories.png.

python plots.py
"""
from __future__ import annotations

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

from common import load_params  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "out", "physics")
BALL = os.path.join(HERE, "..", "out", "ballistics.json")
FONT = "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"

STYLES = [dict(color="#000000", ls="-"), dict(color="#555555", ls="--"), dict(color="#999999", ls=":")]


def _style():
    if os.path.exists(FONT):
        font_manager.fontManager.addfont(FONT)
        plt.rcParams["font.family"] = "Liberation Sans"
    plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                         "axes.edgecolor": "#444444", "axes.grid": True, "grid.color": "#dddddd", "grid.linewidth": 0.6,
                         "lines.linewidth": 2.0})


def range_plot(path):
    b = json.load(open(BALL))
    th = [r["pitch_deg"] for r in b["table"]]
    fig, ax = plt.subplots(figsize=(6.0, 3.4))
    for j, d in enumerate(b["drag_n"]):
        y = [r["range_m"][j] for r in b["table"]]
        ax.plot(th, y, **STYLES[j % 3], label=f"payout brake {d} N")
        k = max(range(len(y)), key=lambda i: y[i])
        ax.annotate(f"{y[k]:.1f} m", (th[k], y[k]), textcoords="offset points", xytext=(0, 5), ha="center", fontsize=8)
    ax.set_xlabel("rail pitch (deg)")
    ax.set_ylabel("range on level ground (m)")
    ax.set_title("Range vs pitch (from the rail pivot's ground point)", fontsize=10, loc="left")
    ax.legend(frameon=False, fontsize=8, loc="lower center")
    ax.set_ylim(bottom=0)
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def trajectory_plot(path, pitches=(10.0, 25.0, 36.0, 47.0)):
    from flight import simulate
    from launcher import launch
    p = load_params()
    fig, ax = plt.subplots(figsize=(6.0, 3.0))
    greys = ["#000000", "#444444", "#777777", "#aaaaaa"]
    for g, th in zip(greys, pitches):
        v = launch(p, th)["v_muzzle"]
        r = simulate(p, v, th, reel_model=True, record=True)
        xs = [h[1] for h in r["history"]]
        zs = [h[2] for h in r["history"]]
        ax.plot(xs, zs, color=g, lw=1.8)
        ax.annotate(f"{th:.0f} deg", (xs[len(xs) // 2], zs[len(zs) // 2]), textcoords="offset points", xytext=(0, 4),
                    ha="center", fontsize=8)
    ax.set_xlabel("distance from the rail pivot (m)")
    ax.set_ylabel("height (m)")
    ax.set_title(f"Dart path, payout brake {p.flight['brake_n']} N", fontsize=10, loc="left")
    ax.set_aspect("equal")
    ax.set_ylim(bottom=0)
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def main():
    _style()
    os.makedirs(OUT_DIR, exist_ok=True)
    range_plot(os.path.join(OUT_DIR, "range_vs_pitch.png"))
    trajectory_plot(os.path.join(OUT_DIR, "trajectories.png"))
    print("wrote range_vs_pitch.png, trajectories.png in", os.path.abspath(OUT_DIR))


if __name__ == "__main__":
    main()
