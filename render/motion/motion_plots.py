"""Quick check plots of the key motion curves (matplotlib, PNG, light surface). One measure per axis; small multiples."""
from __future__ import annotations

import math
import os

import numpy as np

SER = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
SURF, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"


def _style(plt):
    plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "axes.edgecolor": GRID, "axes.labelcolor": INK2,
                         "xtick.color": INK2, "ytick.color": INK2, "text.color": INK, "axes.grid": True, "grid.color": GRID,
                         "grid.linewidth": 0.6, "lines.linewidth": 1.6, "font.size": 8.5, "axes.titlesize": 9.5,
                         "axes.titleweight": "bold", "legend.frameon": False, "axes.spines.top": False, "axes.spines.right": False})


def _events(ax, ev, fps, names=None, ymax=None):
    for f, nm in ev:
        if names and nm not in names:
            continue
        ax.axvline(f / fps, color=INK2, lw=0.6, ls=":", alpha=0.7)


def run(ctx, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _style(plt)
    os.makedirs(out, exist_ok=True)
    fps = 30
    tf = ctx["tf"]
    ev = ctx["ev"]
    gear = ctx["gear"]
    frames = ctx["frames"]
    key = {"aim_start", "cock_start", "fire", "impact", "righted", "cut", "reel_taut", "tension_hold"}

    # 1) film clock: real time 1:1 with one cut
    fig, ax = plt.subplots(figsize=(8, 3.2))
    rt = np.array([ctx["t_of_frame"](f) for f in range(len(tf))])
    ax.plot(tf, rt, color=SER[0])
    ax.set_ylabel("real time (s)")
    ax.set_xlabel("film time (s)")
    ax.set_title("Real time 1:1; one hard cut (cam:A -> cam:B) skips the long drilling and wrap")
    _events(ax, ev, fps, key | {"cut"})
    fig.tight_layout()
    fig.savefig(os.path.join(out, "01_film_clock.png"), dpi=110)
    plt.close(fig)

    # 2) base mechanism
    S = [fr["S"] for fr in frames]
    wiper = np.array([s["wiper_deg"] for s in S])
    drum = np.array([s.get("drum_deg", 0.0) for s in S])
    lo = np.array([s.get("line_out_m", 0.0) for s in S])
    sl = np.array([np.linalg.norm(a - c) for a, c in ctx["springs"]["l"]])
    fig, axs = plt.subplots(3, 2, figsize=(11, 7.5), sharex=True)
    panels = [(axs[0, 0], gear[:, 4], "rail pitch (deg)", "Rail pitch: crank-rocker to 34, rail_low over the 47 top to 6"),
              (axs[0, 1], gear[:, 1], "carriage travel (mm)", "Carriage: drawn back 241 mm, 59 ms stroke, stop overshoot"),
              (axs[1, 0], wiper, "wiper (deg)", "Wiper / main shaft (forward +)"),
              (axs[1, 1], gear[:, 2], "big gear (deg)", "Big gear (one turn per shot, -360 at rest)"),
              (axs[2, 0], drum, "drum (deg)", "Selector drum: LAUNCH 0, NEUTRAL 41, wrap 62..302, park 314, REEL 347"),
              (axs[2, 1], sl * 1000, "spring pin distance (mm)", "Launch spring (l): 189 at HOME, 430 at full cock")]
    for ax, y, yl, tt in panels:
        ax.plot(tf, y, color=SER[0])
        ax.set_ylabel(yl)
        ax.set_title(tt)
        _events(ax, ev, fps, key)
    for a in axs[-1]:
        a.set_xlabel("film time (s)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "02_base.png"), dpi=110)
    plt.close(fig)

    # 3) shot detail vs real time
    fl = ctx["fl"]
    td = np.array([fr["t"] for fr in frames])
    T0 = ctx["T_FIRE"]
    m = (td > T0 - 0.01) & (td < T0 + 0.6)
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.6))
    axs[0].plot((td[m] - T0) * 1000, gear[m, 1], color=SER[0], marker="o", ms=2.5, lw=1.2)
    axs[0].set_xlabel("ms after full cock (real)")
    axs[0].set_ylabel("carriage travel (mm)")
    axs[0].set_title("The shot, one dot per film frame (real time)")
    axs[1].plot((td[m] - T0) * 1000, gear[m, 2], color=SER[1], marker="o", ms=2.5, lw=1.2)
    axs[1].set_xlabel("ms after full cock (real)")
    axs[1].set_ylabel("big gear (deg)")
    axs[1].set_title("Big gear: release at -281.25, reset to -360 in its gap")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "03_shot.png"), dpi=110)
    plt.close(fig)

    # 4) dart trajectory side view + attitude
    loc = ctx["dart_loc"]
    R = ctx["dart_R"]
    nose_dir = np.array([-Ri[:, 0] for Ri in R])
    pitch = np.degrees(np.arcsin(np.clip(nose_dir[:, 2], -1, 1)))
    knot = np.array([fr["knot"] for fr in frames])
    fig, axs = plt.subplots(2, 1, figsize=(10, 6.5))
    axs[0].plot(loc[:, 0], loc[:, 2], color=SER[0], label="nose tip (dart:root)")
    axs[0].plot(knot[:, 0], knot[:, 2], color=SER[1], label="tail knot")
    axs[0].axhline(0, color=INK2, lw=0.8)
    axs[0].set_aspect("equal")
    axs[0].set_xlabel("world X (m)")
    axs[0].set_ylabel("Z (m)")
    axs[0].set_title("Side view: launch at 34 deg, flight, impact near X 4.7")
    axs[0].legend(loc="upper right")
    axs[1].plot(tf, pitch, color=SER[0])
    axs[1].set_ylabel("nose pitch (deg)")
    axs[1].set_xlabel("film time (s)")
    axs[1].set_title("Nose pitch above the horizon (nose first in flight; tumble, flip, belly down)")
    _events(axs[1], ev, fps, key)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "04_dart_path.png"), dpi=110)
    plt.close(fig)

    # 5) flight attitude vs flight path
    fig, ax = plt.subplots(figsize=(7, 3.4))
    path = np.degrees(np.arctan2(fl[:, 8], np.hypot(fl[:, 6], fl[:, 7])))
    ax.plot(fl[:, 0], np.degrees(fl[:, 4]), color=SER[0], label="body pitch")
    ax.plot(fl[:, 0], path, color=SER[1], label="flight path")
    ax.plot(fl[:, 0], np.degrees(fl[:, 5]), color=SER[2], label="heading (yaw)")
    ax.set_xlabel("flight time (s, real)")
    ax.set_ylabel("deg above horizon")
    ax.set_title("Free flight (flight34 physics in 3D, tail tie, picked launch inputs)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out, "05_flight_attitude.png"), dpi=110)
    plt.close(fig)

    # 6) dart joints
    fig, axs = plt.subplots(3, 1, figsize=(9, 6.5), sharex=True)
    axs[0].plot(tf, ctx["leg"], color=SER[0])
    axs[0].set_ylabel("leg (deg)")
    axs[0].set_title("Righting leg (sim joint 'leg')")
    axs[1].plot(tf, ctx["pod"], color=SER[1])
    axs[1].set_ylabel("pods (deg)")
    axs[1].set_title("Drill pods (one deploy worm, stop 46.75)")
    axs[2].plot(tf, ctx["aug"] / 360.0, color=SER[2])
    axs[2].set_ylabel("auger turns")
    axs[2].set_title("Auger spin (L +, R - about the tip direction)")
    axs[2].set_xlabel("film time (s)")
    for a in axs:
        _events(a, ev, fps, key)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "06_dart_joints.png"), dpi=110)
    plt.close(fig)

    # 7) tether
    L = np.array(ctx["lens"])
    fig, axs = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)
    axs[0].plot(tf, L[:, 1], color=SER[0], label="free line paid out")
    axs[0].plot(tf, L[:, 2], color=SER[1], label="straight exit to knot")
    axs[0].set_ylabel("m")
    axs[0].set_title("Tether: payout, spool overrun slack, take-up, tight")
    axs[0].legend(loc="lower right")
    axs[1].plot(tf, lo, color=SER[2])
    axs[1].set_ylabel("line_out_m (spool)")
    axs[1].set_xlabel("film time (s)")
    axs[1].set_title("Paid off the storage spool (drives the spool angle in reel kin)")
    for a in axs:
        _events(a, ev, fps, key)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "07_tether.png"), dpi=110)
    plt.close(fig)

    # 8) line snapshots (side view)
    LF = ctx["LF"]
    evd = dict((nm, f) for f, nm in ev)
    snaps = [("muzzle_exit", 0), ("impact", 0), ("righted", 0), ("cut", 0), ("reel_taut", 0), ("end", 0)]
    fig, ax = plt.subplots(figsize=(11, 3.4))
    for k, (nm, _) in enumerate(snaps):
        f = evd.get(nm)
        if f is None:
            continue
        P = LF[min(f, len(LF) - 1)]
        ax.plot(P[:, 0], P[:, 2], color=SER[k % len(SER)], lw=1.2, label=f"{nm} (f {f})")
    ax.axhline(0, color=INK2, lw=0.8)
    ax.set_xlabel("world X (m)")
    ax.set_ylabel("Z (m)")
    ax.set_title("Tether polyline (base path + free rope), side view")
    ax.legend(ncol=3, fontsize=7.5)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "08_line_snapshots.png"), dpi=110)
    plt.close(fig)

    # 9) sim tension
    sim = ctx["sim"]
    fig, ax = plt.subplots(figsize=(8, 3))
    ax.plot(sim.t, sim.col("T"), color=SER[0])
    ax.set_xlabel("sim time (s)")
    ax.set_ylabel("line tension (N)")
    ax.set_title("Sim line tension (0.25 N drag; tow test ramp, film shows the first 6 s of the tow)")
    ax.set_xlim(0, sim.t[-1])
    fig.tight_layout()
    fig.savefig(os.path.join(out, "09_sim_tension.png"), dpi=110)
    plt.close(fig)


def scene(ctx, out, frames_sel=None):
    """Side and top views of the posed base meshes, the posed dart and the line at a few events (sanity of the posing)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import motion_checks as MC
    _style(plt)
    ROOT = MC.ROOT
    mdir = os.path.join(ROOT, "base", "out", "base", "meshes")
    tr = ctx["base_tracks"]
    verts = {}
    for nm in tr:
        fp = os.path.join(mdir, nm + ".npz")
        if os.path.exists(fp):
            v = np.asarray(np.load(fp)["v"], float) / 1000.0
            verts[nm] = v[:: max(1, len(v) // 600)]
    J = ctx["J34"]
    dm = MC.dart_meshes(J)
    evd = dict((nm, f) for f, nm in ctx["ev"])
    sel = frames_sel or ["aim_start", "cocked", "carriage_home", "muzzle_exit", "cut", "end"]
    fig, axs = plt.subplots(2, len(sel), figsize=(4.2 * len(sel), 7.5))
    for k, nm in enumerate(sel):
        f = evd[nm]
        fr = ctx["frames"][f]
        pts = []
        for pn, v in verts.items():
            M = tr[pn][f]
            pts.append(v @ M[:3, :3].T + M[:3, 3])
        P = np.vstack(pts)
        posed = MC.pose_dart(J, dm, fr["dart"][1], fr["dart"][2], ctx["leg"][f], ctx["pod"][f])
        D = np.vstack(list(posed.values()))
        Lp = ctx["LF"][f]
        for row, (i, j, lab) in enumerate(((0, 2, "Z"), (0, 1, "Y"))):
            ax = axs[row, k]
            ax.scatter(P[:, i], P[:, j], s=0.15, color=SER[0], alpha=0.35, linewidths=0)
            ax.scatter(D[:, i], D[:, j], s=0.15, color=SER[1], alpha=0.6, linewidths=0)
            ax.plot(Lp[:, i], Lp[:, j], color=SER[7], lw=0.9)
            ax.set_aspect("equal")
            if nm in ("cut", "end"):
                c = fr["dart"][1]
                ax.set_xlim(c[0] - 0.22, c[0] + 0.06)
                ax.set_ylim((c[j] - 0.1, c[j] + 0.1) if row else (-0.06, 0.08))
            else:
                ax.set_xlim(-0.5, 0.45)
                ax.set_ylim((-0.05, 0.62) if row == 0 else (-0.2, 0.3))
            if row == 0:
                ax.axhline(0, color=INK2, lw=0.7)
                ax.set_title(f"{nm} (frame {f})")
            ax.set_xlabel("X (m)")
            ax.set_ylabel(lab + " (m)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "10_scene_check.png"), dpi=110)
    plt.close(fig)
