"""Frame hand calculations: fastener and plate loads for the tow, the wiper-stall fault pull, the carriage stop and the
motor reaction; mass, CG and the tip margin on the deck (frame subsystem, private helper).

python cad/parts/_frame_calc.py   (from base/) prints the table used in frame_REPORT.md.

Strength values (conservative, PLA printed, heat-set inserts):
- M3 x 4 x 5 brass insert in PLA: pull-out about 1000 N (CNC Kitchen tests 2020: 1000 to 1500 N), we use 600 N
  working; shear on the insert taken the same.
- M3 8.8 screw shear about 2,000 N; never the limit here.
- PLA printed, in-layer tension about 40 MPa, across layers about 15 MPa; allowable 10 MPa (in-layer) / 4 MPa (across).
"""
from __future__ import annotations

import math
import os
import sys

if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

INSERT_N = 600.0          # working load per M3 insert (pull-out or shear)
PLA_IN = 10.0             # MPa allowable, in-layer
PLA_ACROSS = 4.0          # MPa allowable, across layers

# bought masses (g) not in the CAD (bought parts have no density)
BOUGHT_G = {
    "frame_wiper_motor": 1300.0, "frame_wiper_shaft": 0.0, "frame_pcb_esp32": 10.0, "frame_pcb_bts7960": 66.0,
    "frame_pcb_mp1584": 3.0, "frame_pcb_drv8833": 2.0, "frame_pcb_hx711": 3.0, "frame_fuse_20a": 25.0,
    "frame_pcb_as5600": 2.0,
}


def loads(P, info):
    """Return a list of (case, item, value, limit, ok) rows."""
    C = info["cfg"]
    fw = C["fw"]
    lay = C["layout"]
    rows = []
    phys_tow = 100.0
    stall_pull = 597.0          # physics summary: line pull at wiper stall (12.6 V), break 667 N
    pad_peak = 762.0            # physics summary: carriage stop pad peak force (4 mm pad)
    stall_nm = float((getattr(P, "WIPER", {}) or {}).get("stall_nm", 15.0))

    # capstan blocks: the line wraps the capstan, in and out roughly the same way: shaft load 2 T, two blocks
    cap = next((sp for sp in info["supports"] if sp["shaft"] == "capstan"), None)
    h_cap = lay["capstan"]["origin"][2] if "capstan" in lay else 100.0
    for case, T in (("tow 100 N", phys_tow), ("FAULT stall pull 597 N", stall_pull)):
        F = 2.0 * T
        per_screw = F / 2 / 4
        rows.append((case, "capstan block, shear per M3 insert", per_screw, INSERT_N, per_screw <= INSERT_N))
        # cheek root bending, line pull sideways (worst, out of plane), one cheek takes half
        L = info["cheek_x"][1] - info["cheek_x"][0]
        t = fw["cheek_t"]
        Z = L * t * t / 6.0
        M = F / 2 * h_cap
        sig = M / Z
        rows.append((case, f"cheek root bending, sideways pull at z {h_cap:.0f} (in-layer)", sig, PLA_IN, sig <= PLA_IN))
        # cheek feet: the moment above is taken by the foot screws on one side in tension, lever = foot_w + t
        ft = info["feet"][0]
        nfeet = max(1, len([1 for (x, y) in info["deck_holes"] if ft["x0"] <= x <= ft["x1"] and ft["y0"] <= y <= ft["y1"]]))
        lever = fw["foot_w"] / 2 + t
        Tn = M / lever / nfeet
        rows.append((case, f"cheek foot screw pull ({nfeet} per cheek)", Tn, INSERT_N, Tn <= INSERT_N))
        # deck to rover: tow forward at line height, overturning about the front edge, rear bolt row in tension
        x0, x1, y0, y1 = info["deck_rect"]
        lx = x1 - x0
        nrear = max(2, len([1 for (x, y) in info["rover_holes"] if x < x0 + 20]))
        Tr = T * h_cap / lx / nrear
        rows.append((case, f"rover bolt pull, rear row ({nrear} bolts)", Tr, INSERT_N, Tr <= INSERT_N))

    # carriage stop: if the whole pad peak reached the main shaft block (worst case, the rail is the main shaft)
    main_sup = [sp for sp in info["supports"] if sp["shaft"] == C["main"]]
    nblk = max(1, len(main_sup)) + 1                     # + the motor side (adapter ring, 6 screws)
    per = pad_peak / nblk / 4
    rows.append(("carriage stop 762 N (worst)", "main shaft supports, shear per M3", per, INSERT_N, per <= INSERT_N))

    # motor reaction at stall through the adapter ring (6 x M3 at R ring) and the 3 M6 bosses
    from cad.parts import _frame_motor as MOT
    ring = stall_nm * 1000.0 / (MOT.RING_N * MOT.RING_R)
    rows.append((f"wiper stall {stall_nm:.0f} N m", "adapter ring, shear per M3 insert", ring, INSERT_N, ring <= INSERT_N))
    w = C["wiper"]
    rm = min(math.hypot(x, y) for x, y, _ in w["mount_holes"])
    boss = stall_nm * 1000.0 / (len(w["mount_holes"]) * rm)
    # bearing stress of an M6 on the PPA-CF plate (10 thick, nut side): F / (d * t)
    bs = boss / (w["bolt_d"] * MOT.PLATE_T)
    rows.append((f"wiper stall {stall_nm:.0f} N m", "adapter boss bolt bearing stress (MPa)", bs, 20.0, bs <= 20.0))
    return rows


def mass_cg(asm, S=None):
    """Mass (g) and CG (world mm) of every part in an assembly (printed by density x infill, bought from BOUGHT_G or
    the part notes). Returns (total_g, cg, per_subsystem)."""
    import numpy as np
    from cad.lib import prim
    S = S or {}
    W = asm.world(S)
    tot, mom, sub = 0.0, np.zeros(3), {}
    for n, p in asm.parts.items():
        m = p.mass_g()
        if m is None:
            m = BOUGHT_G.get(n, 0.0)
        if m <= 0:
            continue
        s = prim.place(p.solid, W[n])
        lo, hi = prim.bbox(s)
        c = (np.asarray(lo) + np.asarray(hi)) / 2          # bbox centre (good enough for the tip margin)
        tot += m
        mom += m * c
        sub[p.subsystem] = sub.get(p.subsystem, 0.0) + m
    return tot, mom / max(tot, 1e-9), sub


def tip_margin(total_g, cg, deck_rect, deck_z, impulse_ns=2.0, F_side=100.0, h_side=100.0):
    """Static tip check of the base alone on its deck (as if NOT bolted): distance from the CG ground projection to the
    nearest deck edge, and the side force at line height that would tip it. Bolted to the rover the base cannot tip;
    this number tells how much the bolts work."""
    x0, x1, y0, y1 = deck_rect
    d = min(cg[0] - x0, x1 - cg[0], cg[1] - y0, y1 - cg[1])
    g = 9.81
    m = total_g / 1000.0
    h = cg[2] - deck_z
    F_tip = m * g * d / max(h_side, 1.0)
    return dict(edge_mm=d, cg_height_mm=h, F_tip_n=F_tip, tips_unbolted=F_side > F_tip)


if __name__ == "__main__":
    from tools.params import load_params
    from cad.parts import frame
    P = load_params()
    B = frame.build(P)
    print(f"{'case':30s} {'item':58s} {'value':>8s} {'limit':>7s}")
    for case, item, v, lim, ok in loads(P, B["info"]):
        print(f"{case:30s} {item:58s} {v:8.1f} {lim:7.1f} {'ok' if ok else 'OVER'}")
    from tools.assembly import Assembly
    asm = Assembly.load(P, ["frame"])
    tot, cg, sub = mass_cg(asm)
    print(f"frame mass {tot:.0f} g, CG {cg.round(1)}")
    print(tip_margin(tot, cg, B["info"]["deck_rect"], B["info"]["cfg"]["deck_z"]))
