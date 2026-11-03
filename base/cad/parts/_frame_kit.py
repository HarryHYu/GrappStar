"""Rover-side kit on the base deck (D18): Kiyo X camera mount, RC832 receiver holder, cable anchors (frame helper).

Kiyo X (Razer RZ19-04170): body about 70 x 58 x 48 (retail), 80.85 x 85.05 x 64.5 with the clip folded (Razer spec
sheet), monitor clip plus a 1/4"-20 tripod thread (Razer support "At a Glance"). The mount is a printed MONITOR
EDGE: a 16 mm thick plate the camera's own clip grips (no screws, nothing to buy), with a 1/4" (6.6) hole through the
top so a tripod screw also works. The plate top is tilted to the params pitch so the camera looks along +X.
RC832 receiver: 80 x 65 x 15 (datasheet), SMA antenna on a short end. It stands on its long edge in a slotted holder,
antenna up, clear of the metal wiper motor.
"""
from __future__ import annotations

import math

import numpy as np

from cad.lib import Part, prim, tf
from . import _frame_util as U

KIYO = dict(body=(70.0, 48.0, 58.0), clip_folded=(80.85, 64.5, 85.05), edge_t=16.0, optic_above_edge=30.0)
RC832 = dict(body=(80.0, 15.0, 65.0))      # length (stands vertical), thickness, width
COL = 30.0                                  # camera column section


def camera_mount(cam_world, pitch_deg, deck_z):
    """Column from the deck up to the monitor-edge plate (deck frame solids). cam_world = optical centre (world)."""
    x, y, z = cam_world
    zc = z - deck_z                                     # optical centre above the deck
    ze = zc - KIYO["optic_above_edge"]                  # top of the edge plate
    h_col = ze - 30.0
    col = prim.box(x - COL / 2, x + COL / 2, y - COL / 2, y + COL / 2, 0.0, h_col)      # solid (print at 15 % infill)
    foot = prim.box(x - COL / 2 - 14, x + COL / 2 + 14, y - COL / 2 - 14, y + COL / 2 + 14, 0, 6)
    holes = [(x + sx * (COL / 2 + 7), y + sy * (COL / 2 + 7)) for sx in (-1, 1) for sy in (-1, 1)]
    # edge plate: 70 wide (y), 30 tall, 16 thick (x), its top face tilted by the pitch about y
    t = KIYO["edge_t"]
    plate = prim.box(-t / 2, t / 2, -36.0, 36.0, -30.0, 0.0)
    plate = plate - prim.cyl(3.3, 20, 0, 0, -10)        # 1/4" tripod screw hole from the top
    plate = prim.place(plate, tf.T(x, y, ze))           # level: the Kiyo X clip hinge sets the -10 deg pitch
    neck = prim.box(x - COL / 2, x + COL / 2, y - COL / 2, y + COL / 2, h_col - 3.0, ze - 22.0)
    sol = prim.union([col, foot, plate, neck])
    for hx, hy in holes:
        sol = sol - U.clear_hole(hx, hy, -1, 7)
    return sol, holes


def rc832_holder(x, y):
    """Slotted holder: the receiver stands on a short end, antenna end up; the slot grips its 15 mm thickness along
    the 65 mm width. Deck frame, centred at (x, y)."""
    L_, T_, W_ = RC832["body"]
    h = 40.0
    wall = 3.0
    g = T_ + 0.6
    outer = prim.box(x - W_ / 2 - wall, x + W_ / 2 + wall, y - g / 2 - wall, y + g / 2 + wall, 0, h)
    slot = prim.box(x - W_ / 2 - 0.3, x + W_ / 2 + 0.3, y - g / 2, y + g / 2, 4.0, h + 1)
    window = prim.box(x - W_ / 2 + 8, x + W_ / 2 - 8, y - g / 2 - wall - 1, y + g / 2 + wall + 1, 10.0, h + 1)
    cable = prim.box(x - 12, x + 12, y - g / 2 - wall - 1, y + g / 2 + 1, 4.0, 14.0)   # AV and power leads out low
    foot = prim.box(x - W_ / 2 - 16, x + W_ / 2 + 16, y - g / 2 - wall, y + g / 2 + wall, 0, 6)
    sol = prim.union([outer, foot]) - slot - window - cable
    holes = [(x - W_ / 2 - 9, y), (x + W_ / 2 + 9, y)]
    for hx, hy in holes:
        sol = sol - U.clear_hole(hx, hy, -1, 7)
    return sol, holes


def cable_anchor(x, y, yaw=0.0):
    """Cable-tie anchor block on the deck (strain relief): two tunnels for ties, 1 x M3 into the deck."""
    b = prim.box(-12, 12, -8, 8, 0, 10)
    for sx in (-6.0, 6.0):
        b = b - prim.box(sx - 2.2, sx + 2.2, -9, 9, 3.0, 6.0)
    b = b - U.clear_hole(0, 0, -1, 11)
    b = b - prim.cyl(U.HEAD_D / 2, 6.0, 0, 0, 5.0)
    M = tf.T(x, y, 0) @ tf.Rz(yaw)
    return prim.place(b, M), [(x, y)]


def build(P, C, info):
    """Kit parts (deck frame) and their deck holes."""
    fwr = C["fw"].get("_raw", {}) or {}
    cam = fwr.get("camera", {}) or {}
    parts, holes = [], []
    pos = list(cam.get("pos", (150.0, -50.0, 100.0)))
    eb = fwr.get("ebay")
    if isinstance(eb, dict) and "y" in eb and eb["x"][0] - 25 < pos[0] < eb["x"][1] + 25 and eb["y"][0] - 25 < pos[1] < eb["y"][1] + 25:
        pos[1] = eb["y"][0] - 26.0                       # keep the column off the bay (cad-frame owns the final pose)
        info["warnings_soft"] = info.get("warnings_soft", []) + [f"camera column moved to y {pos[1]:.0f} (off the bay)"]
    info["camera_pos"] = tuple(pos)
    sol, h = camera_mount(pos, float(cam.get("pitch_deg", -10.0)), C["deck_z"])
    parts.append(Part("frame_camera_mount", sol, "PLA", print_tf=U.print_flat(sol), color="#9aa4ae",
                      notes="Kiyo X mount: its own clip grips the 16 mm edge plate (or a 1/4-20 screw), 4 x M3 x 10 into the deck"))
    holes += h
    rx, ry = fwr.get("rc832", {}).get("xy", (215.0, 160.0)) if isinstance(fwr.get("rc832"), dict) else (215.0, 160.0)
    sol, h = rc832_holder(rx, ry)
    parts.append(Part("frame_rc832_holder", sol, "PLA", print_tf=U.print_flat(sol), color="#9aa4ae",
                      notes="RC832 holder: receiver stands antenna-end up in the slot, leads out low, 2 x M3 x 10"))
    holes += h
    info["rc832_xy"] = (rx, ry)
    # bought envelopes (for the audit and the sim)
    L_, T_, W_ = RC832["body"]
    parts.append(Part("frame_rc832", prim.box(rx - W_ / 2, rx + W_ / 2, ry - T_ / 2, ry + T_ / 2, 4.0, 4.0 + L_), "bought",
                      color="#30343a", bought_ref="RC832 5.8 GHz receiver (80 x 65 x 15)", notes="video receiver, antenna up"))
    bx_, by_, bz_ = KIYO["body"]
    x, y, z = pos
    edge_top = z - C["deck_z"] - KIYO["optic_above_edge"]
    # body envelope sitting on the edge plate (the clip arm hangs behind the plate; envelope only)
    cam_sol = prim.box(x - by_ / 2, x + by_ / 2, y - bx_ / 2, y + bx_ / 2, edge_top + 0.05, edge_top + 0.05 + bz_)
    parts.append(Part("frame_kiyo_x", cam_sol, "bought", color="#202020", bought_ref="Razer Kiyo X webcam",
                      notes="rover camera (body envelope), looks along +X"))
    # cable anchors: Jetson USB from the bay, capture dongle and receiver leads
    for i, (axy, yaw) in enumerate(cable_points(C, info)):
        sol, h = cable_anchor(axy[0], axy[1], yaw)
        parts.append(Part(f"frame_cable_anchor_{i + 1}", sol, "PLA", print_tf=U.print_flat(sol), color="#9aa4ae",
                          notes="cable-tie anchor (strain relief), 1 x M3 x 12 into the deck"))
        holes += h
    return parts, holes


def cable_points(C, info):
    pts = []
    x, y, z = info["camera_pos"]
    pts.append(((x + COL / 2 + 30.0, y), 90.0))          # camera USB lead, beside the column
    rx, ry = info["rc832_xy"]
    pts.append(((rx, ry - 30.0), 0.0))                     # receiver AV lead to the capture dongle
    return pts
