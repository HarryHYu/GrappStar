"""Show build renders: the whole base with the KEDGE v34 dart LOADED, COCKED and FIRED (rail at 34 deg), plus the hand
controls (crank, aim knob, drum lever) and the dart tail on the pusher.

python -m tools.show_render OUT RENDERS      (OUT: scratch export folder; RENDERS: where the PNGs go)

Writes RENDERS/show_<state>_<view>.png and RENDERS/show_states.png (one sheet). Uses tools.render's helpers.
"""
from __future__ import annotations

import os
import shutil
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

from tools import render as R            # noqa: E402
from tools.assembly import Assembly      # noqa: E402
from tools.params import load_params     # noqa: E402

PITCH = 34.0


def states():
    return [
        ("loaded", dict(aim_deg=PITCH, cock=0.0, drum_deg=0.0, phase="launch_sweep")),
        ("cocked", dict(aim_deg=PITCH, cock=1.0, drum_deg=0.0, phase="launch_sweep")),
        ("fired", dict(aim_deg=PITCH, launch_travel=0.0, fired=True, launch_dart_fly=300.0, drum_deg=0.0,
                       phase="launch_flight")),
    ]


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    out, dst = argv[0], argv[1]
    os.makedirs(out, exist_ok=True)
    os.makedirs(dst, exist_ok=True)
    P = load_params()
    asm = Assembly.load(P)
    asm.export(out)
    st = states()
    data = R.build_data(asm, out, [S for _, S in st], title="GrappStar base, show build, KEDGE v34")
    srv, url = R.serve({"data.js": R.data_js(data).encode(), "index.html": os.path.join(R.WEB, "render.html")})
    views = os.path.join(out, "views")
    if os.path.isdir(views):
        shutil.rmtree(views)
    os.makedirs(views)
    W, H = 1200, 860
    shots, items = [], []
    for k, (nm, _) in enumerate(st):
        for vn, ops in (("iso", [["view", 55, 24, 0.95]]),
                        ("iso_motor_side", [["view", -55, 24, 0.95]]),
                        ("side", [["view", 90, 4, 0.95]]),
                        ("launcher", [["cam", [70, 140, 110], 820, 60, 24]])):
            f = os.path.join(views, f"show_{nm}_{vn}.png")
            shots.append(dict(file=f, ops=[["sample", k], ["cut", None], ["ground", True]] + ops))
            items.append((f, f"{nm}: {vn}"))
    # hand controls (loaded state): crank, aim knob, drum lever seen from the outboard (+y) side; dart tail and knot
    for vn, ops in (("controls", [["cam", [40, 215, -40], 650, 35, 18]]),
                    ("tail_knot", [["cam", [18, 137, 114], 260, 120, 55]]),
                    ("tail_knot_b", [["cam", [18, 137, 114], 260, 60, 60]])):
        f = os.path.join(views, f"show_loaded_{vn}.png")
        shots.append(dict(file=f, ops=[["sample", 0], ["cut", None], ["ground", True]] + ops))
        items.append((f, f"loaded: {vn}"))
    try:
        res = R.run_shots(dict(url=url + "index.html", width=W, height=H, shots=shots))
        for e in res.get("errors", []):
            print("page:", e)
    finally:
        srv.shutdown()
    for f, _ in items:
        shutil.copy(f, os.path.join(dst, os.path.basename(f)))
    sheet = [(os.path.join(dst, os.path.basename(f)), lab) for f, lab in items if f.endswith("_iso.png") or f.endswith("_launcher.png")]
    size = R.grid_sheet(sheet, 2, os.path.join(dst, "show_states.png"), "GrappStar base, show build, KEDGE v34",
                        f"rail at {PITCH:g} deg: loaded, cocked, fired")
    print("show_states.png", size, len(items), "views ->", dst)
    return 0


if __name__ == "__main__":
    sys.exit(main())
