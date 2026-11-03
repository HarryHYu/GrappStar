"""Electronics bay, board models, AS5600 bracket and magnet holder (frame subsystem, private helper).

Board data (mm). Sources (looked up 3 Nov 2026, see frame_NOTES.md section 2):
- ESP32-S3-DevKitC-1 v1.0/v1.1: Espressif user guide + DXF (dl.espressif.com DXF_ESP32-S3-DevKitC-1_V1.1_20220429).
  PCB 25.40 x 62.87, no mounting holes, module antenna 6.3 past the top edge (overall 69.2), headers 2 x 22 at
  2.54, rows 22.86 apart, first pin 7.96 from the USB edge. USB edge: UART port centre x 6.0, native USB x 19.4.
  Official boards have Micro-USB; clones use USB-C (the slot fits either).
- BTS7960 IBT-2: Handsontec datasheet drawing. PCB 49.66 x 49.40, overall height 43 incl. heatsink, 4 x M3 holes on
  a 40 x 40 square. Heatsink under the PCB, height not published (22 assumed: MEASURE).
- Adafruit AS5600 STEMMA QT (PID 6357): Adafruit Eagle .brd. 25.4 x 17.78, 4 x 2.5 holes at 2.54 from each edge
  (20.32 x 12.70), sensor at the board centre (12.7, 8.89). NO magnet included: a 6 x 2.5 mm DIAMETRIC magnet is needed.
- Pololu DRV8833 (2130): 20.32 x 12.70, no holes; fitted without headers, wires soldered to the pads.
- MP1584EN buck: 22.26 x 16.96 x 3.98 (Harry's calipers), no holes, IN on one short end, OUT on the other.
- HX711: generic green board about 34 x 21 x 3 (seller data); SparkFun 13879 is 30.48 x 22.86. Pocket fits both.
- Fuse: inline ATO holder envelope 55 x 26 x 16 (typical; the 14 AWG holder is Littelfuse FHAC0003ZXJB).
- XT60: body 15.7 x 8.1, panel cutout 16.5 x 9.

Bay local frame: x along the long side, y across, z up; origin at the outer corner on the deck (z = 0 = deck top).
"""
from __future__ import annotations

import math

import numpy as np
from shapely.geometry import Polygon, box as sbox
from shapely.ops import unary_union

from cad.lib import Part, Allow, prim, tf, fits
from . import _frame_util as U

BOARD = dict(
    esp32=dict(pcb=(25.40, 62.87, 1.6), ant=6.33, usb_x=(6.0, 19.4), top_h=3.3, pin_down=8.5,
               src="Espressif ESP32-S3-DevKitC-1 user guide v1.1 + DXF"),
    bts7960=dict(pcb=(49.66, 49.40, 1.6), holes=[(3.0, 4.8), (43.0, 4.8), (3.0, 44.6), (43.0, 44.6)], hole_d=3.2,
                 hs=(36.0, 30.0, 22.0), top_h=19.4, src="Handsontec BTS7960 datasheet drawing"),
    as5600=dict(pcb=(25.4, 17.78, 1.6), holes=[(2.54, 2.54), (22.86, 2.54), (2.54, 15.24), (22.86, 15.24)],
                hole_d=2.5, chip=(12.7, 8.89), src="Adafruit-AS5600-Magnetic-Angle-Sensor-PCB (Eagle)"),
    drv8833=dict(pcb=(20.32, 12.70, 1.6), top_h=1.5, src="Pololu 2130 product page"),
    mp1584=dict(pcb=(22.26, 16.96, 1.2), top_h=2.8, src="Harry's calipers (22.26 x 16.96 x 3.98)"),
    hx711=dict(pcb=(34.0, 23.0, 1.6), top_h=1.5, src="seller data (SparkFun 13879: 30.48 x 22.86)"),
    fuse=dict(body=(55.0, 26.0, 16.0), src="typical inline ATO holder"),
    xt60=dict(body=(15.7, 8.1, 16.0), cut=(16.5, 9.0)),
)
MAGNET = dict(d=6.0, h=2.5, gap=1.0)          # diametric NdFeB, air gap to the chip face (datasheet 0.5 to 3)

# bay box
L, W, H = 124.0, 124.0, 58.0       # fits the params region (130 x 130) with the deck screws inside
WALL, FLOOR = 2.4, 3.0
TAB_W, TAB_T = 0.0, 6.0             # no outside tabs: 4 floor pads inside take the deck screws
PADS = [(110.0, 20.0), (110.0, 50.0), (8.0, 82.0), (53.0, 116.0)]
ESP_Z = FLOOR + 27.0            # ESP32 PCB underside: room for DuPont housings on the downward pins
BTS_Z = FLOOR + 24.0            # BTS7960 PCB underside (heatsink below it)
ESP_CLAMP_Z = ESP_Z + 1.6 + 3.1  # top of the WROOM module = clamp underside
USB_OPEN = (12.5, 9.5)          # native USB opening (fits a Micro-USB or USB-C plug overmold)
EXIT_MOTOR = (33.0, 45.0)       # x range of the 14 AWG wiper pair window in the y = W wall
EXIT_SENS = (14.0, 30.0)        # x range of the sensor window in the y = W wall

# component placement (bay frame, board corner, rotation of the board about z in deg)
PLACE = dict(
    esp32=(13.0, WALL + 0.6, 0.0),        # USB edge against the y = 0 wall
    mp1584=(46.0, 40.0, 0.0),
    drv8833=(74.0, 40.0, 0.0),
    hx711=(16.0, 86.0, 0.0),
    bts7960=(60.0, 62.0, 0.0),           # terminal edge (+x) towards the x = L wall, heatsink beside two walls
    fuse=(46.0, 6.0, 0.0),
)


def _rect_pocket(x, y, sx, sy, depth, relief=True):
    """Shallow pocket (open top) at the floor top for a small board, with two solder relief channels under its
    short ends and two cable-tie slots through the floor across it."""
    z0 = FLOOR
    c = [prim.box(x - 0.3, x + sx + 0.3, y - 0.3, y + sy + 0.3, z0 - 0.01, z0 + depth + 5)]
    if relief:
        c.append(prim.box(x + 1.0, x + sx - 1.0, y + 1.0, y + sy - 1.0, z0 - 1.0, z0 + 0.1))
    return c


def _tie_pair(xc, ya, yb):
    """Cable-tie hold-down: two slots through the floor at ya and yb and a 1.2 mm tunnel under the floor joining
    them (the tie goes down one slot, under, up the other, over the part)."""
    c = [prim.box(xc - 2.2, xc + 2.2, yy - 1.2, yy + 1.2, -1, FLOOR + 3) for yy in (ya, yb)]
    c.append(prim.box(xc - 2.2, xc + 2.2, ya - 1.2, yb + 1.2, -1, 1.2))
    return c


def bay_box():
    """Open-top bay with walls, tabs, cradles, pockets, vents, wire exits and cable-tie anchors (bay frame)."""
    outer = prim.box(0, L, 0, W, 0, H)
    inner = prim.box(WALL, L - WALL, WALL, W - WALL, FLOOR, H + 1)
    box = outer - inner
    adds, cuts = [], []
    # corner bosses for the lid inserts
    for (x, y) in ((WALL, WALL), (L - WALL - 9, WALL), (WALL, W - WALL - 9), (L - WALL - 9, W - WALL - 9)):
        adds.append(prim.box(x, x + 9, y, y + 9, FLOOR - 0.01, H))
        cuts.append(U.insert_hole(x + 4.5, y + 4.5, H))
    # floor pads inside: M3 x 10 down into the deck inserts, head on the 6 mm pad
    tabs = []
    for (px_, py_) in PADS:
        adds.append(prim.box(px_ - 6, px_ + 6, py_ - 6, py_ + 6, FLOOR - 0.01, TAB_T))
        cuts.append(U.clear_hole(px_, py_, -1, TAB_T + 1))
        tabs.append((px_, py_))
    # ESP32 cradle: pillar under the USB end between the two connectors, pillar + stop under the module overhang,
    # two insert posts for the hold-down clamp
    ex, ey, _ = PLACE["esp32"]
    bw, bl, bt = BOARD["esp32"]["pcb"]
    adds.append(prim.box(ex + 9.5, ex + 15.5, ey + 0.6, ey + 5.6, FLOOR - 0.01, ESP_Z))
    adds.append(prim.box(ex + 4.5, ex + 21.0, ey + bl + 0.6, ey + bl + 5.8, FLOOR - 0.01, ESP_Z + bt))
    adds.append(prim.box(ex + 4.5, ex + 21.0, ey + bl + 6.6, ey + bl + 9.0, FLOOR - 0.01, ESP_CLAMP_Z))
    for px in (ex - 5.0, ex + bw + 5.0):
        adds.append(prim.box(px - 4, px + 4, ey + bl + 0.0, ey + bl + 8.0, FLOOR - 0.01, ESP_CLAMP_Z))
        cuts.append(U.insert_hole(px, ey + bl + 4.0, ESP_CLAMP_Z))
    # USB opening for the native "USB" port only (the Jetson cable; it also flashes the board). The PCB edge either
    # side of it bears on the wall, so a cable pull cannot drag the board out. The UART port stays inside.
    ux = ex + BOARD["esp32"]["usb_x"][1]
    cuts.append(prim.box(ux - USB_OPEN[0] / 2, ux + USB_OPEN[0] / 2, -1, WALL + 1, ESP_Z + bt + 1.6 - USB_OPEN[1] / 2,
                         ESP_Z + bt + 1.6 + USB_OPEN[1] / 2))
    # BTS7960 standoffs with inserts
    bx, by, _ = PLACE["bts7960"]
    for (hx, hy) in BOARD["bts7960"]["holes"]:
        adds.append(prim.cyl(4.0, BTS_Z - FLOOR + 0.01, bx + hx, by + hy, FLOOR - 0.01))
        cuts.append(U.insert_hole(bx + hx, by + hy, BTS_Z))
    # heatsink vents: slots in both long walls beside the heatsink, plus the lid grille
    for i in range(6):
        cuts.append(prim.box(bx + 4 + i * 7.5, bx + 8 + i * 7.5, W - WALL - 1, W + 1, FLOOR + 4.0, BTS_Z - 1.0))
        cuts.append(prim.box(L - WALL - 1, L + 1, by + 4 + i * 7.5, by + 8 + i * 7.5, FLOOR + 4.0, BTS_Z - 1.0))
    # small board pockets
    for nm in ("mp1584", "drv8833", "hx711"):
        x, y, _ = PLACE[nm]
        sx, sy, _ = BOARD[nm]["pcb"]
        adds.append(prim.box(x - 2.5, x + sx + 2.5, y - 2.5, y + sy + 2.5, FLOOR - 0.01, FLOOR + 1.8))
        cuts += _rect_pocket(x, y, sx, sy, 1.8)
        cuts += _tie_pair(x + sx / 2, y - 4.0, y + sy + 4.0)
    # fuse holder cradle: two low walls and a tie slot
    fx, fy, _ = PLACE["fuse"]
    fl, fw_, fh = BOARD["fuse"]["body"]
    for xx in (fx + 8, fx + fl - 12):
        adds.append(prim.box(xx, xx + 4, fy - 2, fy + fw_ + 2, FLOOR - 0.01, FLOOR + 6) - prim.box(xx - 1, xx + 5, fy, fy + fw_, FLOOR + 2, FLOOR + 7))
    cuts += _tie_pair(fx + fl / 2, fy - 4.5, fy + fw_ + 4.5)
    # wire exits: 14 AWG motor pair and XT60 (x = L wall), sensor bundle (x = 0 wall), each with a tie anchor inside
    # wire exits on the y = W wall (towards the motor and the sensors): 14 AWG wiper pair and the sensor bundle;
    # XT60 battery lead on the x = L wall; each with a cable-tie anchor inside
    cuts.append(prim.box(EXIT_MOTOR[0], EXIT_MOTOR[1], W - WALL - 1, W + 1, FLOOR + 2.0, FLOOR + 10.0))
    cuts.append(prim.box(EXIT_SENS[0], EXIT_SENS[1], W - WALL - 1, W + 1, FLOOR + 2.0, FLOOR + 12.0))
    xc = BOARD["xt60"]["cut"]
    cuts.append(prim.box(L - WALL - 1, L + 1, 38.0 - xc[0] / 2, 38.0 + xc[0] / 2, FLOOR + 6.0, FLOOR + 6.0 + xc[1]))
    anchors = [((EXIT_MOTOR[0] + EXIT_MOTOR[1]) / 2, W - WALL - 7.0), ((EXIT_SENS[0] + EXIT_SENS[1]) / 2, W - WALL - 7.0),
               (L - WALL - 6.0, 26.0), (ex + bw / 2, ey + 14.0)]
    for (ax_, ay_) in anchors:
        a = prim.box(ax_ - 3, ax_ + 3, ay_ - 5, ay_ + 5, FLOOR - 0.01, FLOOR + 6.0)
        a = a - prim.box(ax_ - 4, ax_ + 4, ay_ - 2.0, ay_ + 2.0, FLOOR - 0.5, FLOOR + 3.0)   # tie tunnel
        adds.append(a)
    box = prim.union([box] + [a for a in adds if not a.is_empty()])
    box = prim.diff(box, cuts)
    return box, tabs


def bay_lid():
    """Lid: 4 mm plate with a lip, grille over the BTS7960 heatsink and board, 4 x M3 x 10 into the corner bosses."""
    t = 4.0
    lid = prim.box(0, L, 0, W, H, H + t)
    lip = prim.box(WALL + 0.3, L - WALL - 0.3, WALL + 0.3, W - WALL - 0.3, H - 3.0, H + 0.01) \
        - prim.box(WALL + 2.3, L - WALL - 2.3, WALL + 2.3, W - WALL - 2.3, H - 4, H + 1)
    for (x, y) in ((WALL, WALL), (L - WALL - 9, WALL), (WALL, W - WALL - 9), (L - WALL - 9, W - WALL - 9)):
        lip = lip - prim.box(x - 0.5, x + 9.5, y - 0.5, y + 9.5, H - 4, H + 1)
    lid = lid + lip
    cuts = []
    for (x, y) in ((WALL + 4.5, WALL + 4.5), (L - WALL - 4.5, WALL + 4.5), (WALL + 4.5, W - WALL - 4.5), (L - WALL - 4.5, W - WALL - 4.5)):
        cuts.append(U.clear_hole(x, y, H - 1, H + t + 1, head_top=H + t + 0.01, head_h=2.0))
    bx, by, _ = PLACE["bts7960"]
    for i in range(8):
        cuts.append(prim.box(bx + 4, bx + 46, by + 2 + i * 6.0, by + 5.0 + i * 6.0, H - 1, H + t + 1))
    return prim.diff(lid, cuts)


def esp_clamp():
    ex, ey, _ = PLACE["esp32"]
    bw, bl, _ = BOARD["esp32"]["pcb"]
    zc = ESP_CLAMP_Z                      # clamp underside = module top: presses the module end down
    c = prim.box(ex - 9, ex + bw + 9, ey + bl + 0.0, ey + bl + 8.0, zc, zc + 4.0)
    for px in (ex - 5.0, ex + bw + 5.0):
        c = c - U.clear_hole(px, ey + bl + 4.0, zc - 1, zc + 5, head_top=zc + 4.01, head_h=1.5)
    return c


# ------------------------------------------------------------------------------------------------ board models
def board_models():
    """Bought board envelopes in the bay frame: name -> solid."""
    out = {}
    ex, ey, _ = PLACE["esp32"]
    bw, bl, bt = BOARD["esp32"]["pcb"]
    pcb = prim.box(ex, ex + bw, ey, ey + bl, ESP_Z, ESP_Z + bt)
    mod = prim.box(ex + 3.75, ex + 21.75, ey + 38.0, ey + bl + BOARD["esp32"]["ant"], ESP_Z + bt, ESP_Z + bt + 3.1)
    usb = [prim.box(ex + ux - 4.0, ex + ux + 4.0, ey - 0.6, ey + 5.5, ESP_Z + bt, ESP_Z + bt + 3.0) for ux in BOARD["esp32"]["usb_x"]]
    pins = [prim.box(ex + px - 1.27, ex + px + 1.27, ey + 7.96 - 1.27, ey + 61.3 + 1.27, ESP_Z - 8.5, ESP_Z + 0.01) for px in (1.27, 24.13)]
    out["frame_pcb_esp32"] = prim.union([pcb, mod] + usb + pins)
    bx, by, _ = PLACE["bts7960"]
    sx, sy, st = BOARD["bts7960"]["pcb"]
    hsx, hsy, hsz = BOARD["bts7960"]["hs"]
    pcb = prim.box(bx, bx + sx, by, by + sy, BTS_Z, BTS_Z + st)
    for (hx, hy) in BOARD["bts7960"]["holes"]:
        pcb = pcb - prim.cyl(1.6, 10, bx + hx, by + hy, BTS_Z - 2)
    hs = prim.box(bx + (sx - hsx) / 2, bx + (sx + hsx) / 2, by + (sy - hsy) / 2, by + (sy + hsy) / 2, BTS_Z - hsz, BTS_Z + 0.01)
    term = prim.box(bx + sx - 10.0, bx + sx, by + 12.0, by + 34.0, BTS_Z + st, BTS_Z + st + 12.0)
    cap = prim.cyl(5.0, 17.0, bx + sx - 6.0, by + 6.0, BTS_Z + st)
    out["frame_pcb_bts7960"] = prim.union([pcb, hs, term, cap])
    for nm in ("mp1584", "drv8833", "hx711"):
        x, y, _ = PLACE[nm]
        sx, sy, st = BOARD[nm]["pcb"]
        out[f"frame_pcb_{nm}"] = prim.box(x, x + sx, y, y + sy, FLOOR, FLOOR + st) + \
            prim.box(x + 3, x + sx - 3, y + 3, y + sy - 3, FLOOR + st - 0.01, FLOOR + st + BOARD[nm]["top_h"])
    fx, fy, _ = PLACE["fuse"]
    fl, fw_, fh = BOARD["fuse"]["body"]
    out["frame_fuse_20a"] = prim.box(fx, fx + fl, fy, fy + fw_, FLOOR + 2.0, FLOOR + 2.0 + fh)
    return out


# ------------------------------------------------------------------------------------------------ AS5600 bracket
def as5600_parts(P, C, info):
    """AS5600 board and its bracket on the far (non-motor) cheek, chip on the main axis facing the drum's magnet. Returns
    (list of (name, solid_world or local, kwargs)), info."""
    lay = C["layout"]
    if C["main"] not in lay:
        return [], {}
    ms = C["fw"]["motor_side"]
    s = -ms
    sup = [sp for sp in info["supports"] if sp["shaft"] == C["main"] and sp["side"] == s]
    mo = np.asarray(lay[C["main"]]["origin"], float)
    yb = C["y_out"][s] + s * (sup[0]["block"]["tb"] if sup else 0.0)        # outer face of the main block
    # The magnet holder on the shaft end belongs to cad-drum (BASE_SPEC section 3). Its magnet face y comes from
    # params (DRUM["magnet_face_y"] or FRAMEWORK["as5600_magnet_face_y"], world); until then assume the drum's cap ends
    # 21 mm beyond the main block face (8 mm shaft end + 13 mm cap).
    yf = None
    for src in (getattr(P, "DRUM", None) or {}, C["fw"].get("_raw", {}) or {}):
        for k in ("magnet_face_y", "as5600_magnet_face_y"):
            if isinstance(src, dict) and src.get(k) is not None:
                yf = float(src[k])
    if yf is None:
        yf = yb + s * 21.0
    y_chip = yf + s * MAGNET["gap"]
    a = BOARD["as5600"]
    bw, bh, bt = a["pcb"]
    pcb = prim.box(-a["chip"][0], bw - a["chip"][0], -a["chip"][1], bh - a["chip"][1], 1.0, 1.0 + bt)   # chip 1 mm tall
    pcb = pcb + prim.box(-2.5, 2.5, -2.5, 2.5, 0.0, 1.01)
    for (hx, hy) in a["holes"]:
        pcb = pcb - prim.cyl(a["hole_d"] / 2, 10, hx - a["chip"][0], hy - a["chip"][1], -2)
    Mpcb = tf.T(mo[0], y_chip, mo[2]) @ (tf.Rx(-90) if s > 0 else tf.Rx(90))
    # bracket: plate behind the PCB with 4 heat-stake pegs through the 2.5 holes, two legs back to the cheek with
    # M3 into two inserts above and below the main block
    blk_half = sup[0]["block"]["half"] if sup else 20.0
    plate_z0 = 1.0 + bt                       # bracket plate starts behind the PCB (local frame of the PCB)
    pt = 4.0
    leg_off = blk_half + 7.0                   # legs above / below the block flange (local y = world z)
    reach = abs(y_chip - C["y_out"][s]) + plate_z0 + pt
    br = prim.box(-bw / 2 - 2, bw / 2 + 2, -leg_off - 5, leg_off + 5, plate_z0, plate_z0 + pt)
    for sy in (-1, 1):
        br = br + prim.box(-6, 6, sy * leg_off - 5, sy * leg_off + 5, plate_z0 + pt - reach, plate_z0 + pt)
    for (hx, hy) in a["holes"]:
        br = br + prim.cyl(1.15, plate_z0 - 0.0 + 0.01, hx - a["chip"][0], hy - a["chip"][1], 0.0, seg=16)   # pegs
    # cable window over the STEMMA QT side
    holes = []
    for sy in (-1, 1):
        zz = plate_z0 + pt - reach
        # long counterbore down the leg: 6 mm of leg under the head, so an M3 x 10 reaches the cheek insert
        br = br - U.clear_hole(0, sy * leg_off, zz - 1, plate_z0 + pt + 1, head_top=plate_z0 + pt + 0.01,
                               head_h=reach - 6.0)
        holes.append((mo[0], mo[2] + sy * leg_off))
    # chip clearance: hole in the plate is not needed (PCB sits on the pegs' shoulders = plate face)
    Mbr = Mpcb
    parts = [
        ("frame_pcb_as5600", prim.place(pcb, Mpcb), dict(material="bought", bought_ref="Adafruit AS5600 STEMMA QT (PID 6357)",
            notes="angle sensor, chip on the main axis, 1 mm air gap to the magnet")),
        ("frame_as5600_bracket", prim.place(br, Mbr), dict(material="PLA",
            notes="AS5600 bracket: 4 pegs through the board holes (heat-stake the tips), 2 x M3 into the cheek")),
    ]
    return parts, dict(cheek_inserts=holes, side=s, y_chip=y_chip)


# ------------------------------------------------------------------------------------------------ placement + build
def _footprints(parts):
    polys = []
    for p in parts:
        (x0, y0, z0), (x1, y1, z1) = prim.bbox(prim.place(p.solid, p.place))
        if z0 < 70.0 and np.all(np.isfinite([x0, y0, x1, y1])):
            polys.append(sbox(x0, y0, x1, y1))
    return unary_union(polys)


def place_bay(C, parts, override=None, info=None):
    """Choose the bay pose on the deck. Hard rules: clear of every footprint; 45 mm free in front of the native USB
    opening (the Jetson plug and its bend). Score: deck rectangle area + 3 x (motor lead + sensor lead lengths).
    Returns 4x4 bay -> deck frame."""
    if override and isinstance(override.get("x"), (tuple, list)):
        # params give a region (x0, x1), (y0, y1): centre the bay in it, long side along the longer edge
        (rx0, rx1), (ry0, ry1) = override["x"], override["y"]
        yaw = float(override.get("yaw", 90.0))     # 90: the USB wall (local -y) faces world +x, the exits face -x
        M0 = tf.Rz(yaw)
        cs = tf.apply(M0, [(0, -TAB_W, 0), (L, -TAB_W, 0), (L, W + TAB_W, 0), (0, W + TAB_W, 0)])
        cx, cy = (cs[:, 0].min() + cs[:, 0].max()) / 2, (cs[:, 1].min() + cs[:, 1].max()) / 2
        if info is not None and (np.ptp(cs[:, 0]) > rx1 - rx0 + 0.5 or np.ptp(cs[:, 1]) > ry1 - ry0 + 0.5):
            info["warnings"].append(f"bay {np.ptp(cs[:, 0]):.0f} x {np.ptp(cs[:, 1]):.0f} does not fit the params region "
                                    f"{rx1 - rx0:.0f} x {ry1 - ry0:.0f}")
        return tf.T((rx0 + rx1) / 2 - cx, (ry0 + ry1) / 2 - cy, 0) @ M0
    if override:
        return tf.T(override["x"], override["y"], 0) @ tf.Rz(override.get("yaw", 0.0))
    fp = _footprints(parts)
    if info is not None and info.get("keepout") is not None:
        fp = fp.union(info["keepout"])               # other subsystems' feet and everything that sweeps low
    fpb = fp.buffer(4.0)
    x0, y0, x1, y1 = fp.bounds
    mo = C["layout"][C["main"]]["origin"] if C["main"] in C["layout"] else (0, 0, 0)
    motor_xy = None
    if info is not None and "M_motor" in info:
        w = C["wiper"]
        motor_xy = tf.apply(info["M_motor"], [(w["body_box"][0], 0.0, -w["boss_h"])])[0][:2]
    sensor_xy = np.array([mo[0], C["y_out"][-C["fw"]["motor_side"]]])
    ux = PLACE["esp32"][0] + BOARD["esp32"]["usb_x"][1]
    usb_zone = [(ux - 12, -45.0), (ux + 12, -45.0), (ux + 12, 0.0), (ux - 12, 0.0)]
    best = None
    for yaw in (0.0, 90.0, 180.0, 270.0):
        M0 = tf.Rz(yaw)
        corners = tf.apply(M0, [(0, -TAB_W, 0), (L, -TAB_W, 0), (L, W + TAB_W, 0), (0, W + TAB_W, 0)])
        cx0, cy0 = corners[:, 0].min(), corners[:, 1].min()
        cx1, cy1 = corners[:, 0].max(), corners[:, 1].max()
        sx, sy = cx1 - cx0, cy1 - cy0
        for gx in np.arange(x0 - sx - 4, x1 + 4, 5.0):
            for gy in np.arange(y0 - sy - 4, y1 + 4, 5.0):
                bb = sbox(gx, gy, gx + sx, gy + sy)
                if bb.intersects(fpb):
                    continue
                M = tf.T(gx - cx0, gy - cy0, 0) @ M0
                uz = Polygon([tuple(tf.apply(M, [(u, v, 0)])[0][:2]) for u, v in usb_zone])
                if uz.intersects(fp):
                    continue
                ux0, uy0 = min(x0, gx), min(y0, gy)
                ux1, uy1 = max(x1, gx + sx), max(y1, gy + sy)
                area = (ux1 - ux0) * (uy1 - uy0)
                mot_exit = tf.apply(M, [(sum(EXIT_MOTOR) / 2, W, 0)])[0][:2]
                sen_exit = tf.apply(M, [(sum(EXIT_SENS) / 2, W, 0)])[0][:2]
                lead = np.abs(sen_exit - sensor_xy).sum()
                if motor_xy is not None:
                    lead += np.abs(mot_exit - motor_xy).sum()
                score = area + 30.0 * lead          # 1 mm of lead counts like 30 mm2 of deck
                if best is None or score < best[0]:
                    best = (score, M)
    return best[1]


def build(P, C, info, parts_so_far):
    fw = C["fw"]
    out = []
    asp, ainfo = as5600_parts(P, C, info)
    extra = [Part(nm, sol, "bought") for nm, sol, kw in asp]
    Mbay = place_bay(C, list(parts_so_far) + extra, fw.get("ebay"), info)
    box, tabs = bay_box()
    deck_holes = [tuple(tf.apply(Mbay, [(x, y, 0)])[0][:2]) for x, y in tabs]
    out.append(Part("frame_ebay", box, "PLA", place=Mbay, print_tf=U.print_flat(box), color="#9aa4ae",
                    notes="electronics bay: ESP32-S3 cradle (USB slot), BTS7960 on standoffs with vents, DRV8833, MP1584, HX711 pockets, fuse cradle, wire exits with tie anchors"))
    lid = bay_lid()
    out.append(Part("frame_ebay_lid", lid, "PLA", place=Mbay, print_tf=U.print_flat(lid, tf.Rx(180)), color="#c9d1d9",
                    notes="bay lid: grille over the BTS7960, 4 x M3 into the corner bosses"))
    cl = esp_clamp()
    out.append(Part("frame_ebay_esp_clamp", cl, "PLA", place=Mbay, print_tf=U.print_flat(cl), color="#c9d1d9",
                    notes="ESP32 hold-down over the module end, 2 x M3 x 8 into the cradle posts"))
    refs = dict(frame_pcb_esp32="Espressif ESP32-S3-DevKitC-1-N8", frame_pcb_bts7960="BTS7960 43 A module (IBT-2)",
                frame_pcb_mp1584="MP1584EN buck module, set to 9.0 V", frame_pcb_drv8833="Pololu DRV8833 2130 (headers off)",
                frame_pcb_hx711="HX711 load cell amplifier (optional)", frame_fuse_20a="inline ATO fuse holder + 20 A fuse")
    for nm, sol in board_models().items():
        out.append(Part(nm, sol, "bought", place=Mbay, color="#2b6e3f" if "pcb" in nm else "#c03030", bought_ref=refs[nm],
                        notes=refs[nm]))
    from cad.lib import Part as _P
    for nm, sol, kw in asp:
        j = kw.pop("joint", False)
        mat = kw.pop("material")
        p = _P(nm, sol, mat, color="#e0a030" if "magnet" in nm else "#9aa4ae", **kw)
        if j:
            p.joint = "frame_wiper"
            p.place = tf.T(-np.asarray(C["layout"][C["main"]]["origin"], float))
        if p.printed:
            p.print_tf = U.print_flat(sol, tf.Rx(-90) if ainfo["side"] > 0 else tf.Rx(90))   # plate outer face down
        out.append(p)
    return out, dict(M_bay=Mbay, deck_holes=deck_holes, as5600=ainfo)


def allowed(P):
    return [
        Allow("frame_pcb_*", "frame_ebay*", "boards sit on their cradles, pegs and pockets", max_vol=2.0),
        Allow("frame_fuse_20a", "frame_ebay", "fuse holder in its cradle", max_vol=2.0),
        Allow("frame_ebay*", "frame_ebay*", "lid lip, clamp on its posts", max_vol=1.0),
        Allow("frame_ebay", "frame_deck*", "bay stands on the deck", max_vol=1.0),
        Allow("frame_pcb_as5600", "frame_as5600_bracket", "pegs through the board holes", max_vol=2.0),
        Allow("frame_as5600_bracket", "frame_cheek_*", "bracket legs bolted to the cheek", max_vol=1.0),
    ]
