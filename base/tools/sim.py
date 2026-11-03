"""HD mission sim page: python -m tools.sim OUT [--only demo] [--dt 0.05] [--shots]

Writes OUT/sim.html: one self-contained offline file (three.js, data and app inline) that plays the per-part world tracks
from asm.tracks(timeline(P, dt)) with the phase markers and texts of tools.timeline.phases, plus OUT/sim_body.html,
the body fragment for publishing (three.js r128 from cdnjs, the rest inline). --shots also captures a few headless screenshots into OUT/shots/ (sim_*.png) to look at.
"""
from __future__ import annotations

import argparse
import os
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

from tools.assembly import Assembly  # noqa: E402
from tools.params import load_params  # noqa: E402
from tools import timeline as TL  # noqa: E402
from tools.render import build_data, write_pages, run_shots  # noqa: E402


def build(out: str, names=None, dt: float = 0.05, P=None) -> str:
    P = P or load_params()
    asm = Assembly.load(P, names)
    os.makedirs(out, exist_ok=True)
    asm.export(out)
    states = TL.timeline(P, dt)
    data = build_data(asm, out, states)
    data["phases"] = TL.phases(P)
    dst, _ = write_pages(out, "sim", "sim_template.html", data, ["viewer_core.js", "sim_app.js"], data["title"] + " mission sim")
    return dst


def shots(out: str, times=None) -> list[str]:
    """Headless screenshots of OUT/sim.html at a few mission times (light, one section cut, dark and phone width)."""
    sd = os.path.join(out, "shots")
    os.makedirs(sd, exist_ok=True)
    url = "file://" + os.path.abspath(os.path.join(out, "sim.html"))
    P = load_params()
    if times is None:
        ph = TL.phases(P)
        times = [round((p["t0"] + p["t1"]) / 2, 2) for p in ph[1::3]][:4]
    files, job = [], []
    for t in times:
        f = os.path.join(sd, f"sim_t{t:06.2f}.png")
        job.append(dict(file=f, ops=[["SIM_PAUSE"], ["SIM_SEEK", t]]))
        files.append(f)
    f = os.path.join(sd, "sim_cut_y.png")
    job.append(dict(file=f, ops=[["SIM_PAUSE"], ["SIM_SEEK", times[0]], ["SIM_CUT", "y", 0.5, True]]))
    files.append(f)
    # the whole mission track (wide view) and, during the flight, the camera following the dart
    fl = [p for p in TL.phases(P) if "flight" in p["name"]]
    tf = round((fl[0]["t0"] + fl[0]["t1"]) / 2, 2) if fl else times[0]
    for name, ops in (("sim_mission.png", [["SIM_FOLLOW", ""], ["SIM_VIEW", "mission"]]),
                      ("sim_follow_dart.png", [["SIM_VIEW", "iso"], ["SIM_FOLLOW", "dart"]])):
        f = os.path.join(sd, name)
        job.append(dict(file=f, ops=[["SIM_PAUSE"], ["SIM_CUT", None], ["SIM_SEEK", tf]] + ops))
        files.append(f)
    res = run_shots(dict(url=url, width=1500, height=900, ready="window.READY === true", shots=job))
    errs = list(res.get("errors", []))
    f = os.path.join(sd, "sim_dark_phone.png")
    res = run_shots(dict(url=url, width=400, height=820, scale=2, color_scheme="dark", ready="window.READY === true",
                         shots=[dict(file=f, ops=[["SIM_PAUSE"], ["SIM_SEEK", times[-1]]])]))
    files.append(f)
    errs += res.get("errors", [])
    for e in errs:
        print("page:", e)
    return files


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out")
    ap.add_argument("--only", default="")
    ap.add_argument("--dt", type=float, default=0.05)
    ap.add_argument("--shots", action="store_true", help="also write headless screenshots to OUT/shots/")
    a = ap.parse_args(argv)
    names = [s for s in a.only.split(",") if s] or None
    dst = build(a.out, names, a.dt)
    body = os.path.join(a.out, "sim_body.html")
    print(f"{dst} {os.path.getsize(dst) / 1e6:.2f} MB, {body} {os.path.getsize(body) / 1e6:.2f} MB")
    if a.shots:
        for f in shots(a.out):
            print(f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
