"""SHOW build of the base station (dart-v1, from the D50 design at tag base-parked-d50): hand operated, no logic.

Harry: "the launcher from before but this is just for show. Basically full function except forget the one way
bearings, just have everything hanging, have a place for the wiper motor but I'm not gonna actually link it up. Make
everything manually operable."

What this module does (the owners' modules carry the small geometry edits, each marked "show build"):
- adds the HAND CRANK on the free end of the main shaft (the d12 spigot now runs on to y 255, past the aim housing):
  a printed crank with a long hub (it also holds the shaft endwise against the launch sleeve end, the old magnet cap's
  job), D bore on the spigot flat, M3 set screw, and a printed handle peg;
- adds the hand AIM KNOB on the aim countershaft end (now out of the housing to y 244): 10 knob turns per full rail
  cycle, the self-locking worm holds the pitch;
- drop(P): leaves out the one-way and sensing parts and the electronics (tools.assembly drop hook).
Changed elsewhere: sprag #1 -> printed fixed hub (launch_drive_hub); sprag #2 -> plain bore idler on a plain axle
(reel); the aim splitter and the take-up swing idler are gone (the aim is by the knob, the spool by hand); the
wiper is not linked (no shear pin: the coupler turns free on the hub, which only carries the shaft end on the motor
shaft); the N20 pinion is left out (the drum turns by its hand lever); the dart is KEDGE v34.
Frame: world, mm; the crank and handle are built in the drum_main frame (local z = world y), the knob in the aim
countershaft frame (local z from its face, along world +y).
"""
from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from cad.lib import Part, Allow, Clearance, prim, tf, fits   # noqa: E402

CRANK_S = (195.0, 254.0)      # crank hub (world y); the old magnet cap started at 195 (2 mm off the sleeve end)
ARM_S = (246.0, 254.0)        # crank arm plane
ARM_R = 50.0                  # handle radius
HUB_R = 11.0
SET_S = 250.0                 # M3 set screw onto the spigot flat
HANDLE_L = 30.0
KNOB_Y = (234.0, 242.0)       # aim knob on the countershaft (housing ends at y 233)
KNOB_R = 14.0

DROP = [
    # one-way elements and their logic
    "reel_sprag",                                   # CSK20PP sprag #2 (capstan backstop)
    "reel_swing_arm", "reel_gi",                    # forward-only take-up swing idler
    "launch_aim_sun", "launch_aim_arm", "launch_aim_drag_pad", "launch_aim_drag_shim",   # aim direction splitter
    "launch_aim_idler", "launch_aim_idler_clip",
    # the wiper and N20 links (mounts stay)
    "drum_shear_pin", "drum_n20_pinion",
    # sensing
    "drum_magnet_cap", "drum_index_block", "drum_index_lever", "drum_switch_block", "drum_sw_lever_*", "drum_dog_post",
    "launch_sw_seated*", "launch_sw_home*", "launch_sw_gap*", "launch_seat_plunger", "launch_seat_pushrod",
    "launch_seat_clip",
    "reel_sw_bracket", "reel_sw_base", "reel_sw_lever", "reel_sw_hw",
    # electronics bay, camera and video receiver, cable anchors
    "frame_ebay", "frame_ebay_*", "frame_pcb_*", "frame_perfboard", "frame_fuse_20a",
    "frame_camera_mount", "frame_kiyo_x", "frame_rc832", "frame_rc832_holder", "frame_cable_anchor_*",
]


def drop(P):
    return list(DROP)


def _drum(P):
    import drum as D
    return D, D.G(P)


def crank_solid(P):
    """Hand crank in the drum_main frame (z = world y): hub r 11 over y 195..254 with a D bore on the spigot flat
    (flat on local +x), arm to -x in the plane 246..254, handle hole d 6.2 at r 50, M3 tapped hole onto the flat."""
    D, g = _drum(P)
    r_sp = g["spig_d"] / 2
    s0, s1 = CRANK_S
    a0, a1 = ARM_S
    body = prim.union([prim.cyl(HUB_R, s1 - s0, z=s0),
                       prim.box(-ARM_R, 0.0, -7.0, 7.0, a0, a1),
                       prim.cyl(9.0, a1 - a0, x=-ARM_R, z=a0)])
    bore = prim.cyl(r_sp + fits.HOLE_SLIDE / 2, s1 - s0 + 2, z=s0 - 1) - prim.box(r_sp - 1.0 + 0.1, 20, -20, 20, s0 - 2, s1 + 2)
    cuts = [bore,
            prim.cyl(3.1, a1 - a0 + 2, x=-ARM_R, z=a0 - 1),
            prim.place(prim.cyl(1.25, 14.0, z=3.0), tf.mul(tf.T(0, 0, SET_S), tf.Ry(90)))]
    # 45 deg chamfer on the hub's inner end (prints arm face down, hub up)
    return body - prim.union(cuts)


def handle_solid(P):
    """Handle peg in the drum_main frame: d 6 pin in the arm hole (246.2..254), grip d 14 from the arm face outboard."""
    a0, a1 = ARM_S
    pin = prim.cyl(3.0, a1 - a0 - 0.2 + 0.01, x=-ARM_R, z=a0 + 0.2)
    grip = prim.cyl(7.0, HANDLE_L, x=-ARM_R, z=a1)
    return pin + grip


def knob_solid(P):
    """Aim knob (local z 0..8 along world +y from KNOB_Y[0]): r 14 with 10 grip flutes, D bore on the countershaft flat
    (flat on local +x at 3.2 from the axis, d8 shaft)."""
    import math
    h = KNOB_Y[1] - KNOB_Y[0]
    k = prim.cyl(KNOB_R, h)
    fl = [prim.cyl(2.2, h + 2, x=(KNOB_R + 0.8) * math.cos(math.radians(36 * i)),
                   y=(KNOB_R + 0.8) * math.sin(math.radians(36 * i)), z=-1) for i in range(10)]
    bore = prim.cyl(4.0 + fits.HOLE_SLIDE / 2, h + 2, z=-1) - prim.box(3.2 + 0.1, 10, -10, 10, -2, h + 2)
    return k - prim.union(fl + [bore])


def _bed(solid, R=None):
    R = tf.I4 if R is None else R
    m = prim.place(solid, R)
    lo, hi = prim.bbox(m)
    return tf.mul(tf.T(-0.5 * (lo[0] + hi[0]), -0.5 * (lo[1] + hi[1]), -lo[2]), R)


def parts(P):
    import launch_aim as AIM
    cr, hd, kb = crank_solid(P), handle_solid(P), knob_solid(P)
    return [
        Part("show_hand_crank", cr, "PLA", joint="drum_main", place=tf.I4, print_tf=_bed(cr, tf.Rx(180)),
             color="#d97a00", bought_ref="1 x M3x6 set screw or SHCS",
             notes="Show build: hand crank on the main shaft spigot (D bore, M3 onto the flat; its hub holds the shaft "
                   "against the launch sleeve end). Turns the whole train. Prints arm face down"),
        Part("show_crank_handle", hd, "PLA", joint="drum_main", place=tf.I4, print_tf=_bed(hd, tf.Rx(180)),
             color="#3a3f45", notes="Show build: crank handle peg, pressed (or glued) into the arm hole. Prints grip end down"),
        Part("show_aim_knob", kb, "PLA", joint="launch_aim_cshaft", place=AIM._ygear(kb, KNOB_Y[0]), print_tf=_bed(kb),
             color="#d97a00",
             notes="Show build: hand aim knob on the aim countershaft end (D bore, a drop of glue): 10 turns per rail "
                   "cycle 6 -> 47 -> 6 deg; the self-locking worm holds the pitch"),
    ]


def joints(P):
    return []


def kin(P, S):
    return {}


def allowed(P):
    return [
        Allow("show_hand_crank", "drum_shaft", "D bore on the spigot flat (slide fit)", max_vol=1.0),
        Allow("show_crank_handle", "show_hand_crank", "handle pin in the arm hole", max_vol=1.0),
        Allow("show_aim_knob", "launch_aim_output", "knob on the countershaft D flat", max_vol=1.0),
    ]


def clearances(P):
    return [
        Clearance("show_hand_crank", "launch_aim_housing", 0.3, "crank hub turns clear of the aim housing"),
        Clearance("show_hand_crank", "show_aim_knob", 1.0, "crank arm passes outboard of the aim knob"),
        Clearance("show_crank_handle", "show_aim_knob", 1.0, "handle passes the aim knob"),
    ]


def steps(P):
    return [
        dict(title="Show build: hand crank", stage=990,
             text="After the launcher: slide the hand crank onto the main shaft spigot end, hub first, flat to flat, "
                  "until the hub is about 2 mm from the launch sleeve end, and tighten the M3 x 6 onto the flat. Press "
                  "the handle peg into the arm hole (a drop of glue if loose). Turning the crank turns the main shaft "
                  "and whatever the drum has selected: LAUNCH cocks the launcher, REEL winds the capstan.",
             parts=["show_hand_crank", "show_crank_handle"]),
        dict(title="Show build: aim knob", stage=991,
             text="Push the aim knob onto the aim countershaft end outside the housing, flat to flat (a drop of glue). "
                  "Turning it raises or lowers the rail through the bevels, the worm and the crank-rocker; the worm "
                  "holds the angle.",
             parts=["show_aim_knob"]),
    ]
