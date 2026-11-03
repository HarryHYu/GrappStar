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
    from cad.parts import _frame_motor as MOT
    w = C["wiper"]
    stall_nm = float((getattr(P, "WIPER", {}) or {}).get("stall_nm", 15.0))
    limiter_nm = float((getattr(P, "DRUM", {}) or {}).get("torque_limiter_nm", 5.0))   # D8 shear pin in the coupler
    tow = float((getattr(P, "REEL", {}) or {}).get("tow_limit_n", 50.0))
    h_line = 95.0                                              # capstan axis above the deck (BASE_SPEC 5)

    # motor reaction: the frame sees the coupler torque (limiter) in service, the full stall torque only if the shear
    # pin is replaced by something stronger
    used = (sorted(set(info.get("plate_rim_ok", [])) & set(info.get("puck_rim_ok", range(MOT.RIM_N))))
            if C.get("v1") else MOT.ring_used(w))
    ring_r = MOT.RIM_R if C.get("v1") else MOT.RING_R
    for case, Tq in ((f"limiter {limiter_nm:.0f} N m (D8)", limiter_nm), (f"FAULT stall {stall_nm:.0f} N m", stall_nm)):
        ring = Tq * 1000.0 / (max(1, len(used)) * ring_r)
        rows.append((case, f"adapter rim screws, shear per M3 insert ({len(used)} used)", ring, INSERT_N, ring <= INSERT_N))
        rm = min(math.hypot(x, y) for x, y, _ in w["mount_holes"])
        boss = Tq * 1000.0 / (len(w["mount_holes"]) * rm)
        bs = boss / (w["bolt_d"] * MOT.PLATE_T)
        rows.append((case, "adapter boss bolt bearing stress (MPa)", bs, 20.0, bs <= 20.0))
        feet = info.get("motor_leg_feet") or []
        if len(feet) >= 2:
            span = max(abs(a[0] - b[0]) for a in feet for b in feet)
            # the motor torque about y reaches the leg foot as a couple across x; 2 screws per side; plus the weight
            # of motor + plate + puck + stay (about 1.6 kg) acting about 90 mm from the leg in x
            M = Tq * 1000.0 + 1.6 * 9.81 * 90.0
            F = M / max(span, 1.0) / 2.0
            rows.append((case, f"motor leg foot screw pull ({len(feet)} screws, span {span:.0f} mm)", F, INSERT_N, F <= INSERT_N))
            # foot flange bending under that screw pull, 8 thick, 30 wide, lever 7 mm to the column face (across layers
            # when printed upright? no: the foot is the first layers, bending is in-layer)
            sig = F * 7.0 / (30.0 * 8.0 ** 2 / 6.0)
            rows.append((case, "motor leg foot flange bending (in-layer, MPa)", sig, PLA_IN, sig <= PLA_IN))
            sp = fw["motor_support"]
            bx = sp["leg_x"][1] - sp["leg_x"][0]
            by = sp["leg_y"][1] - sp["leg_y"][0]
            sig = Tq * 1000.0 / (by * bx * bx / 6.0)
            rows.append((case, "motor leg bending at the root (in-layer, MPa)", sig, PLA_IN, sig <= PLA_IN))

    # tow: the line pull at the capstan goes into cad-reel's wall and its deck feet, then the deck and the rover bolts
    x0, x1, y0, y1 = info["deck_rect"]
    for case, T in ((f"tow {tow:.0f} N (firmware limit)", tow), ("FAULT line at wiper stall 373 N (D3)", 373.0)):
        nrear = max(2, len([1 for (x, y) in info["rover_holes"] if x < x0 + 20]))
        Tr = T * h_line / (x1 - x0) / nrear
        rows.append((case, f"rover bolt pull, rear row ({nrear} bolts)", Tr, INSERT_N, Tr <= INSERT_N))
        sh = T / max(1, len(info["rover_holes"]))
        rows.append((case, f"rover bolt shear ({len(info['rover_holes'])} bolts)", sh, INSERT_N, sh <= INSERT_N))
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


def shot_tip(m_total_kg, h_cg_mm, h_rail_mm, p_dart=0.279 * 7.13, pitch_deg=35.0):
    """Rover + base as one rigid body on its wheels at the moment of the shot. The dart leaves with momentum p_dart
    along the rail; the base gets -p_dart through the rail pivot at h_rail above the ground. Returns the smallest
    wheel base half-length (rear wheel to CG, horizontal) that keeps both front wheels down, from energy:
    rotation about the rear wheel contact must not lift the CG over it. Conservative (ignores wheel compliance)."""
    g = 9.81
    m = m_total_kg
    px = p_dart * math.cos(math.radians(pitch_deg))
    pz = p_dart * math.sin(math.radians(pitch_deg))
    # angular impulse about a rear contact point at distance a behind the CG, ground level
    best = None
    for a_mm in range(20, 400):
        a = a_mm / 1000.0
        h = h_rail_mm / 1000.0
        L = px * h - pz * a                       # backward push high up tips nose up; downward share helps
        hc = h_cg_mm / 1000.0
        I = m * (a * a + hc * hc)                 # point mass about the contact
        E = max(L, 0.0) ** 2 / (2 * I)
        dh = math.hypot(a, hc) - hc               # CG rise to tip over the rear contact
        if E < m * g * dh:
            best = a_mm
            break
    return dict(angular_impulse_nms=round(px * h_rail_mm / 1000.0, 3), min_cg_to_rear_wheel_mm=best)


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
    for m_r in (3.0, 5.0):
        print(f"rover {m_r} kg + base {tot / 1000:.1f} kg:", shot_tip(m_r + tot / 1000.0, 150.0, 120.0 + 165.0 + 8.0))
