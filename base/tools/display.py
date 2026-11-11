"""DISPLAY build: the D50 launcher at real size, launcher only, for show on a shelf (dart-v1). Nothing moves.

python -m tools.display OUT          (OUT = base/out/display)

Built from the show assembly (launch, frame, drum, reel modules) posed at one state (rail at PITCH, carriage at
HOME, v34 dart loaded). Stacks are fused into single PLA prints (washers, clips, pegs, bearings, rollers, pulleys);
the reel, the drum, the deck, the guards, the stops, the line and every sensor are left out. New simple parts: a
motor adapter plate with two legs (the real wiper motor bolts to it), the main axle (the real wiper shaft plugs into
its hub), the electronics box (the D50 bay with thinner walls, no lid), and two OPTIONAL hollow dummies (selector
drum, storage reel). The real Century C-243 springs hang on printed anchor pins and the yoke pegs; the carriage is
held at HOME by the big gear's first tooth (the big gear is screwed to the cheek), as at the start of a cock.

Writes OUT/asm.json + meshes (the display assembly: printed parts + the real parts for the fit check), OUT/audit.txt
(interference check at the display pose), and returns the assembly for tools.display_pack.
"""
from __future__ import annotations

import fnmatch
import math
import os
import sys
import types

import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

from cad.lib import Part, Allow, Clearance, prim, tf   # noqa: E402
from tools.assembly import Assembly                     # noqa: E402
from tools.params import load_params                    # noqa: E402

PITCH = 30.02                                  # an audited aim_sweep state of the full base
S0 = dict(aim_deg=PITCH, cock=0.0, drum_deg=0.0, phase="launch_sweep")
DECK_Z = -165.0                                # board top (the old deck top)
CLR = "#d9dde3"

# fused prints: (display name, base part = print frame and print rotation, source patterns, note)
GROUPS = [
    ("disp_rail_rear", "launch_rail_rear", ["launch_rail_rear", "launch_pulley_top", "launch_pulley_top_brg",
                                            "launch_pulley_top_washer_*", "launch_pulley_top_axle"],
     "Rail rear piece with the tether pulley, its bearing, washers and axle fused in"),
    ("disp_rail_mid", "launch_rail_mid", ["launch_rail_mid"],
     "Rail mid piece (cheeks on the sleeve, big gear pin bosses); glue the front joint (no splice straps)"),
    ("disp_rail_front", "launch_rail_front", ["launch_rail_front"],
     "Rail front piece (spring anchor posts, muzzle tie)"),
    ("disp_yoke", "launch_carriage_yoke", ["launch_carriage_yoke", "launch_yoke_peg_*"],
     "Carriage yoke with both spring pegs fused on (the real spring hooks slip onto the peg ends)"),
    ("disp_big_gear", "launch_big_gear", ["launch_big_gear"],
     "Big gear; one M3 screw through the cheek fixes it with its first tooth on the rack"),
    ("disp_gear_pin", "launch_gear_pin", ["launch_gear_pin"], "Big gear pin (no clip: a drop of glue)"),
    ("disp_drive_gear", "launch_drive_gear", ["launch_drive_gear", "launch_drive_hub"],
     "Drive gear 26T with the hub fused in (keyway onto the sleeve rib)"),
    ("disp_sleeve", "launch_sleeve", ["launch_sleeve"], "Launch sleeve: the rail pivot on the main axle"),
    ("disp_anchor_pin_l", "launch_anchor_pin_l", ["launch_anchor_pin_l"], "Spring anchor pin (printed; or 8 mm rod)"),
    ("disp_anchor_pin_r", "launch_anchor_pin_r", ["launch_anchor_pin_r"], "Spring anchor pin (printed; or 8 mm rod)"),
    ("disp_aim_housing", "launch_aim_housing", ["launch_aim_housing"], "Aim housing: stands on the board, carries the sleeve end"),
    ("disp_aim_cap", "launch_aim_cap", ["launch_aim_cap"], "Worm saddle cap"),
    ("disp_aim_worm", "launch_aim_worm", ["launch_aim_worm"], "Worm: self-locking, it holds the crank and so the rail angle"),
    ("disp_aim_crank", "launch_aim_crank", ["launch_aim_crank", "launch_aim_brg_1", "launch_aim_brg_2"],
     "Worm wheel and crank with the two 608 bearings fused in (runs on the printed pin)"),
    ("disp_aim_pin", "launch_aim_bolt", ["launch_aim_bolt"], "Crank axle pin (printed in place of the M8 bolt)"),
    ("disp_aim_conrod", "launch_aim_conrod", ["launch_aim_conrod"], "Conrod"),
    ("disp_aim_lug", "launch_aim_lug", ["launch_aim_lug"], "Conrod lug on the left rail wall (2 x M3 or glue)"),
]
REAL = {   # real parts for the fit check: display name -> (source patterns, bought ref)
    "real_dart": (["launch_dart"], "KEDGE v34 dart (built separately)"),
    "real_wiper_motor": (["frame_wiper_motor"], "12 V wiper motor (Harry's)"),
    "real_wiper_shaft": (["frame_wiper_shaft"], "(part of the wiper motor)"),
}
SPRING_PATS = ["launch_spring_l_*", "launch_spring_r_*"]
BOARDS = {"frame_pcb_esp32": "ESP32-S3-DevKitC-1", "frame_pcb_bts7960": "BTS7960 (IBT-2)",
          "frame_pcb_drv8833": "Pololu DRV8833", "frame_pcb_mp1584": "MP1584EN buck"}


def _rot(M):
    R = np.eye(4)
    R[:3, :3] = np.asarray(M, float)[:3, :3]
    return R


def rebed(solid, R):
    m = prim.place(solid, R)
    lo, hi = prim.bbox(m)
    return tf.mul(tf.T(-0.5 * (lo[0] + hi[0]), -0.5 * (lo[1] + hi[1]), -lo[2]), R)


def best_rot(solid, R0, bed=233.0):
    """Keep R0 when the part has a real flat face on the bed in it; else the axis rotation with the largest contact
    area that fits the 235 safe area."""
    def contact(R):
        m = prim.place(solid, R)
        lo, hi = prim.bbox(m)
        if max(hi[0] - lo[0], hi[1] - lo[1]) > bed or hi[2] - lo[2] > 250:
            return -1.0
        return m.slice(lo[2] + 0.1).area()
    if contact(R0) >= 150.0:
        return R0
    cands = [R0, tf.I4, tf.Rx(90), tf.Rx(-90), tf.Rx(180), tf.Ry(90), tf.Ry(-90)]
    return max(cands, key=contact)


def stand(solid):
    """Upright print rotation for a pin: the larger end down."""
    lo, hi = prim.bbox(solid)
    a_lo = solid.slice(lo[2] + 0.3).area()
    a_hi = solid.slice(hi[2] - 0.3).area()
    return tf.I4 if a_lo >= a_hi else tf.Rx(180)


class Builder:
    def __init__(self, P):
        self.P = P
        self.A = Assembly.load(P, ["launch", "frame", "drum", "reel"])      # no show module: nothing dropped
        self.W = self.A.world(S0)
        self.parts, self.src = [], {}
        self.allowed = []

    def match(self, pats):
        return [n for n in self.A.parts if any(fnmatch.fnmatchcase(n, p) for p in pats)]

    def fuse(self, base, pats):
        Mb = np.linalg.inv(self.W[base])
        names = self.match(pats)
        sol = prim.union([prim.place(self.A.parts[n].solid, Mb @ self.W[n]) for n in names])
        return sol, names

    def add(self, name, solid, place, print_tf, notes, srcs=(), material=None, color=CLR, bought=""):
        material = material or "PLA"
        p = Part(name, solid, material, joint=None, place=place, print_tf=print_tf, color=color, notes=notes,
                 bought_ref=bought)
        p.infill = 0.08
        p.subsystem = "optional" if name.startswith("opt_") else "display"
        self.parts.append(p)
        self.src[name] = list(srcs)
        return p

    # ------------------------------------------------------------------------------------------------ launcher
    def launcher(self):
        out = {}
        for name, base, pats, note in GROUPS:
            sol, names = self.fuse(base, pats)
            Mb = np.linalg.inv(self.W[base])
            for c in self.connectors(name):
                sol = sol + prim.place(c, Mb)
            bp = self.A.parts[base]
            R = _rot(bp.print_tf) if bp.print_tf is not None else stand(sol)
            R = best_rot(sol, R)
            col = bp.color if bp.material != "bought" else CLR
            out[name] = self.add(name, sol, self.W[base], rebed(sol, R), note, names, color=col)
        # carriage: split in two (286 long > 235), printed on its side as before
        cp = self.A.parts["launch_carriage"]
        cs, rnames = self.fuse("launch_carriage", ["launch_carriage", "launch_muzzle_roller", "launch_muzzle_roller_brg",
                                                   "launch_muzzle_washer_*", "launch_muzzle_roller_axle"])
        cs = cs + prim.place(self.rod("launch_muzzle_roller_axle", 2.2), np.linalg.inv(self.W["launch_carriage"]))
        lo, hi = prim.bbox(cs)
        um = 0.5 * (lo[0] + hi[0])
        for nm, half in (("disp_carriage_rear", cs ^ prim.box(lo[0] - 1, um, -200, 200, -200, 200)),
                         ("disp_carriage_front", cs ^ prim.box(um, hi[0] + 1, -200, 200, -200, 200))):
            out[nm] = self.add(nm, half, self.W["launch_carriage"], rebed(half, tf.Rx(90)),
                               "Carriage (rack, cradle, pusher, lip roller fused) in two halves: glue at the cut face", rnames,
                               color=cp.color)
        self._gear_screw(out)
        return out

    def rod(self, n, r):
        """World cylinder r along the longest local bbox axis of part n, over its full length (fuses a pin into the
        holes it sits in)."""
        lo, hi = prim.bbox(self.A.parts[n].solid)
        lo, hi = np.array(lo), np.array(hi)
        k = int(np.argmax(hi - lo))
        c = 0.5 * (lo + hi)
        cyl = prim.cyl(r, hi[k] - lo[k], z=lo[k])
        R = {0: tf.Ry(90), 1: tf.Rx(-90), 2: tf.I4}[k]
        if k == 0:
            M = tf.mul(tf.T(0, c[1], c[2]), tf.Ry(90))
            cyl = prim.cyl(r, hi[0] - lo[0], z=-hi[0])
        elif k == 1:
            M = tf.mul(tf.T(c[0], 0, c[2]), tf.Rx(-90))
        else:
            M = tf.T(c[0], c[1], 0)
        return prim.place(cyl, self.W[n] @ M)

    def connectors(self, name):
        W = self.W
        if name == "disp_yoke":
            return [self.rod(n, 4.15) for n in self.match(["launch_yoke_peg_*"])]
        if name == "disp_drive_gear":
            return [prim.place(prim.tube(23.75, 23.2, 14.0, z=-7.0), W["launch_drive_hub"])]
        if name == "disp_aim_crank":
            return [prim.place(prim.tube(11.15, 10.0, 7.0, z=-7.0), W[n]) for n in ("launch_aim_brg_1", "launch_aim_brg_2")]
        return []

    def _gear_screw(self, out):
        """M3 hole that fixes the big gear to a rail cheek: clearance through the cheek, pilot 10 deep in the gear."""
        bg, rm = out["disp_big_gear"], out["disp_rail_mid"]
        Wg = bg.place
        g_lo, g_hi = prim.bbox(self.A.parts["launch_big_gear"].solid)
        rail_w = prim.place(rm.solid, rm.place)
        gear_w = prim.place(bg.solid, Wg)
        best = None
        for r in (13.0, 15.0, 17.0, 20.0, 24.0, 28.0):
            for a in range(0, 360, 15):
                x, y = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
                for side in (-1, 1):
                    z0 = g_lo[2] if side < 0 else g_hi[2]
                    cheek = prim.place(prim.cyl(1.6, 14.0, x, y, z0 - 14.0 if side < 0 else z0 + 0.3), Wg)
                    gear = prim.place(prim.cyl(1.3, 10.0, x, y, z0 if side < 0 else z0 - 10.0), Wg)
                    vc = (cheek ^ rail_w).volume() / (math.pi * 1.6 ** 2)
                    vg = (gear ^ gear_w).volume() / (math.pi * 1.3 ** 2)
                    if vg > 9.0 and vc > 4.0 and (best is None or vc + vg > best[0]):
                        best = (vc + vg, x, y, side, z0, vc)
        if best is None:
            self.gear_fix = "glue the big gear to its washer face on the cheek (no screw spot found)"
            return
        _, x, y, side, z0, vc = best
        cut_c = prim.cyl(1.6, 30.0, x, y, z0 - 30.0 if side < 0 else z0)
        cut_g = prim.cyl(1.3, 10.0, x, y, z0 if side < 0 else z0 - 10.0)
        rm.solid = rm.solid - prim.place(cut_c, np.linalg.inv(rm.place) @ Wg)
        bg.solid = bg.solid - cut_g
        self.gear_fix = (f"one M3 x {int(round(vc + 8))} self-tapping screw through the hole in the rail mid cheek into "
                         f"the big gear's pilot hole, with the carriage at HOME resting on the gear's first tooth")

    # ------------------------------------------------------------------------------------------------ frame
    def axle(self):
        """Main axle: the D50 coupler hub (the real wiper shaft plugs into its bore) fused to the main shaft, made
        round r 13 so it prints lying on a 0.6 mm flat (support under the d12 spigot end)."""
        A, W = self.A, self.W
        hub = prim.place(A.parts["drum_coupler_hub"].solid, W["drum_coupler_hub"])
        sh = prim.place(A.parts["drum_shaft"].solid, W["drum_shaft"])
        lo, hi = prim.bbox(sh)
        # round bar r 13 over the hex length (y 28 .. hex end) and the hub bridge
        import drum as D
        g = D.G(self.P)
        h0, h1 = g["hex_s"]
        bar = prim.cylY(12.5, -5.0, h1, x=0, z=0)
        ax = prim.union([hub, sh, bar])
        # the bar fills the hub: open the wiper shaft bore again (round d10.3; the flats need no key for show) and the
        # room for the shaft's M8 thread end
        ax = ax - prim.cylY(5.15, -7.0, g["flat_end_s"] + 0.5) - prim.cylY(4.8, g["flat_end_s"], h0 + 5.0)
        ax = ax - prim.box(-20, 20, -10.0, h1 + 0.5, -40.0, -12.2)          # print flat
        ax = ax ^ prim.box(-50, 50, -20.0, g["spig_end"], -50, 50)          # D50 spigot end (no show crank)
        return self.add("disp_axle", ax, tf.I4, rebed(ax, tf.Rz(90)),
                        "Main axle: D50 coupler hub (the real wiper shaft plugs in) + main shaft, round, d12 spigot "
                        "into the sleeve. Prints lying on its flat, support under the spigot",
                        ["drum_coupler_hub", "drum_shaft"])

    def motor_plate(self):
        """Motor adapter plate (display): world y -10..0 (D48 boss face on its rear face), the three wiper boss holes
        as M6 slots (+-4 mm along x; params WIPER mount_holes, NOT measured), output boss hole r 15, two leg sockets."""
        Wp = getattr(self.P, "WIPER", {}) or {}
        holes = Wp.get("mount_holes", [(-24.0, 38.0, 6.5), (-24.0, -38.0, 6.5), (72.0, 0.0, 6.5)])
        x0, x1, z0, z1 = -52.0, 52.0, -34.0, 80.0
        plate = prim.box(x0, x1, -10.0, 0.0, z0, z1)
        cuts = [prim.cylY(15.0, -11, 1)]
        for (mx, my, d) in holes:
            wx, wz = my, mx                                  # motor X -> world +Z, motor Y -> world +X (orient 90)
            sl = prim.union([prim.cylY(d / 2 + 0.2, -11, 1, x=wx + dx, z=wz) for dx in (-4.0, 4.0)] +
                            [prim.box(wx - 4, wx + 4, -11, 1, wz - d / 2 - 0.2, wz + d / 2 + 0.2)])
            cuts.append(sl)
        # lightening windows where nothing mounts (between the bosses), plate stays a frame
        cuts.append(prim.box(-26.0, 26.0, -11, 1, 22.0, 50.0))
        cuts.append(prim.box(-44.0, -22.0, -11, 1, -10.0, 30.0))
        cuts.append(prim.box(22.0, 44.0, -11, 1, -10.0, 30.0))
        plate = plate - prim.union(cuts)
        self.add("disp_motor_plate", plate, tf.I4, rebed(plate, tf.Rx(90)),
                 "Motor adapter plate: the real wiper motor bolts on from behind with 3 x M6 (bosses on the rear face, "
                 "slots +-4 mm: the D50 pattern is assumed, not measured); its shaft goes through the 30 mm hole into "
                 "the axle hub")
        # one leg under the plate's bottom edge: post with a 10 mm slot for the plate, a cross foot (front strip
        # along x, a tail strip under the motor along -y) and a web; 3 screws into the board
        post = prim.box(-12.0, 12.0, -12.0, 4.0, DECK_Z, z0 + 6.0)       # behind the plate only 2 mm: the motor
        front = prim.box(-36.0, 36.0, -24.0, 4.0, DECK_Z, DECK_Z + 4.0)  # gearbox starts 8 mm behind its face
        tail = prim.box(-7.0, 7.0, -72.0, 4.0, DECK_Z, DECK_Z + 4.0)
        web = prim.box(-2.0, 2.0, -72.0, -12.0, DECK_Z, DECK_Z + 30.0)
        side = prim.box(-36.0, 36.0, -12.0, -8.0, DECK_Z, DECK_Z + 22.0)
        leg = prim.union([post, front, tail, web, side]) - prim.box(-13, 13, -10.2, 0.2, z0 - 0.01, z0 + 7)
        holes = [(-30.0, -18.0), (30.0, -18.0), (0.0, -66.0)]
        leg = leg - prim.union([prim.cyl(1.7, 10, hx, hy, DECK_Z - 1) for hx, hy in holes])
        self.leg_holes = holes
        legs = [self.add("disp_motor_leg", leg, tf.I4, rebed(leg, tf.I4),
                         "Motor plate leg: the plate drops into its 10 mm slot (glue); 3 screws into the board")]
        return legs

    def ebox(self):
        """D50 electronics bay with 1.6 mm walls, no lid, at its D50 place in front of the drum (board level)."""
        from cad.parts import _frame_ebay as EB
        EB.WALL, EB.FLOOR, EB.H = 1.2, 2.0, 50.0          # thinner, lower (BTS7960 top at about 48)
        box, _ = EB.bay_box()
        self.ebox_pads = [tuple((np.asarray(self.A.parts["frame_ebay"].place, float) @ np.array([x, y, 0, 1.0]))[:2])
                          for x, y in EB.PADS]
        M = np.asarray(self.A.parts["frame_ebay"].place, float)
        self.add("disp_ebox", box, M, rebed(box, tf.I4),
                 "Electronics box (D50 bay, thin walls, no lid): ESP32-S3 DevKitC-1 cradle with USB opening, BTS7960 "
                 "on 4 standoffs (M3 inserts or self tap), DRV8833 and MP1584 pockets with cable-tie slots")
        bm = EB.board_models()
        for n, ref in BOARDS.items():
            self.add("real_" + n.replace("frame_pcb_", "pcb_"), bm[n], M, None, f"{ref} (Harry's)",
                     material="bought", color="#2e7d4f", bought=ref)

    def real_parts(self):
        for nm, (pats, ref) in REAL.items():
            n = self.match(pats)[0]
            p = self.A.parts[n]
            self.add(nm, p.solid, self.W[n], None, ref, [n], material="bought", color=p.color, bought=ref)
        for n in self.match(SPRING_PATS):
            p = self.A.parts[n]
            self.add("real_" + n.replace("launch_", ""), p.solid, self.W[n], None, "Century C-243 spring (Harry's)", [n],
                     material="bought", color=p.color, bought="Century C-243" if n.endswith("hook_a") else "")

    # ------------------------------------------------------------------------------------------------ dummies
    def dummies(self):
        """OPTIONAL hollow look-alikes at the real places (thin walls modelled: they print as walls only)."""
        import drum as D
        g, c = D.G(self.P), D.C(self.P)
        dx, dy = D.drum_xy(g)
        M = self.W.get("drum_drum_a")
        # drum axis in world: the drum joint frame z axis
        Jd = self.A.joint_world(S0)["drum_drum"]
        o, ax = Jd[:3, 3], Jd[:3, 2]
        t = 1.2

        drum = prim.cyl(30.0, 128.0, z=8.0) + prim.tube(48.0, 29.9, 3.0, z=8.0)       # barrel + gear flange
        drum = drum - prim.cyl(30.0 - t, 200.0, z=8.0 + t)                              # open top, walls only
        Mdrum = np.eye(4)
        Mdrum[:3, :3] = Jd[:3, :3]
        Mdrum[:3, 3] = o
        self.add("opt_drum_dummy", drum, Mdrum, rebed(drum, tf.I4),
                 "OPTIONAL selector drum look-alike (hollow), gear end down; sits on the drum saddle", color="#e8c27a")
        # saddle under it: a block from the board to the barrel with a cradle
        cw = Mdrum @ np.array([0, 0, 110.0, 1.0])
        zc = float(o[2])
        sad = prim.box(cw[0] - 22, cw[0] + 22, cw[1] - 6, cw[1] + 6, DECK_Z, zc - 20.0)
        sad = sad - prim.cylY(30.12, cw[1] - 7, cw[1] + 7, x=cw[0], z=zc)
        cw2 = Mdrum @ np.array([0, 0, 40.0, 1.0])
        sad2 = prim.box(cw2[0] - 22, cw2[0] + 22, cw2[1] - 6, cw2[1] + 6, DECK_Z, zc - 20.0)
        sad2 = sad2 - prim.cylY(30.12, cw2[1] - 7, cw2[1] + 7, x=cw2[0], z=zc)
        sad = sad + sad2 + prim.box(cw[0] - 22, cw[0] + 22, cw2[1] - 6, cw[1] + 6, DECK_Z, DECK_Z + 4.0)
        self.add("opt_drum_saddle", sad, tf.I4, rebed(sad, tf.I4), "OPTIONAL saddle for the drum look-alike", color="#c8ccd2")
        # storage reel: spool look-alike at the real spool place on a one-sided bracket with a stub axle
        Js = self.W["reel_spool"]
        lo, hi = prim.bbox(prim.place(self.A.parts["reel_spool"].solid, Js))
        cx, cz = 0.5 * (lo[0] + hi[0]), 0.5 * (lo[2] + hi[2])
        y0, y1 = lo[1], hi[1]
        sp = prim.union([prim.cylY(31.0, y0, y0 + 2.0, x=cx, z=cz), prim.cylY(20.0, y0 + 1.9, y1 - 1.9, x=cx, z=cz),
                         prim.cylY(31.0, y1 - 2.0, y1, x=cx, z=cz)])
        sp = sp - prim.cylY(18.8, y0 + 1.2, y1 - 1.2, x=cx, z=cz) - prim.cylY(4.3, y0 - 1, y1 + 1, x=cx, z=cz)
        self.add("opt_reel_spool", sp, tf.I4, rebed(sp, tf.Rx(90)), "OPTIONAL storage reel look-alike (hollow spool)",
                 color="#8e7cc3")
        br = prim.box(cx - 25, cx + 25, y0 - 6.0, y0 - 0.05, DECK_Z, cz + 10.0) + \
            prim.box(cx - 25, cx + 25, y0 - 40.0, y0 - 1.0, DECK_Z, DECK_Z + 4.0)
        br = br - prim.union([prim.cyl(1.7, 10, cx + s * 17, y0 - 25, DECK_Z - 1) for s in (-1, 1)])
        self.add("opt_reel_bracket", br, tf.I4, rebed(br, tf.Ry(90)), "OPTIONAL reel bracket (glue the spool to its face)",
                 color="#c8ccd2")

    # ------------------------------------------------------------------------------------------------ allows
    def allows(self):
        """Original Allow pairs remapped onto the display parts, plus the real-part fits."""
        A = self.A
        owner = {}
        for d, ss in self.src.items():
            for s in ss:
                owner.setdefault(s, set()).add(d)
        seen = set()
        for al in A.allowed:
            da = {d for s in owner if fnmatch.fnmatchcase(s, al.a) for d in owner[s]}
            db = {d for s in owner if fnmatch.fnmatchcase(s, al.b) for d in owner[s]}
            for x in da:
                for y in db:
                    if x != y and (x, y) not in seen and (y, x) not in seen:
                        seen.add((x, y))
                        self.allowed.append(Allow(x, y, "D50: " + al.why))
        self.allowed += [
            Allow("real_spring_*", "real_spring_*", "segmented spring model"),
            Allow("real_spring_*_hook_a", "disp_anchor_pin_*", "spring hook on the anchor pin"),
            Allow("real_spring_*_hook_c", "disp_yoke", "spring hook on the yoke peg"),
            Allow("real_wiper_shaft", "disp_axle", "wiper shaft in the axle hub bore"),
            Allow("real_wiper_shaft", "real_wiper_motor", "shaft in the gearbox"),
            Allow("real_pcb_*", "disp_ebox", "board on its cradle, standoffs or pocket", max_vol=60.0),
            Allow("disp_aim_pin", "disp_aim_housing", "pin in the housing wall bore", max_vol=40.0),
            Allow("disp_aim_pin", "disp_aim_crank", "pin in the crank bore", max_vol=40.0),
        ]


def build(P=None):
    P = P or load_params()
    B = Builder(P)
    B.launcher()
    B.axle()
    B.motor_plate()
    B.ebox()
    B.real_parts()
    B.dummies()
    B.allows()
    mod = types.ModuleType("display")
    mod.parts = lambda P: list(B.parts)
    mod.joints = lambda P: []
    mod.kin = lambda P, S: {}
    mod.allowed = lambda P: list(B.allowed)
    mod.clearances = lambda P: []
    mod.grounded = lambda P: ["disp_aim_housing", "disp_motor_leg", "disp_ebox", "opt_drum_saddle", "opt_reel_bracket"]
    asm = Assembly(P, {"display": mod})
    asm.gear_fix = getattr(B, "gear_fix", "")
    asm.builder = B
    return asm


RATE = 15.0          # mm3/s average extrusion, X2D, PLA, 0.28 mm layers, 2 walls (estimate)
LAYER_S = 3.0        # s per 0.28 mm layer (travel, wipe, cooling), counted once per plate
SLICER = ["PLA, 0.4 mm nozzle, 0.28 mm layers (0.24 for the gears and the carriage if you want finer teeth)",
          "2 walls, 4 top and 3 bottom layers, 8 % infill (gyroid or grid; 5 % is fine for the box, plate and leg)",
          "supports: tree, build plate only, for the rail pieces, the carriage halves and the axle spigot; "
          "none for the rest",
          "brim on the tall parts (aim housing, sleeve, worm)"]


def pack_all(out):
    from tools import pack as PK
    PK.PROFILE["PLA"] = dict(walls=2, top=4, bottom=3, infill=0.08)
    PK.RATE["PLA"] = RATE
    PK.LAYER_S = LAYER_S * 0.2 / 0.28
    PK.export_reason = lambda fn, names: None           # the display manifest is exported by this tool
    core = PK.run(out, os.path.join(out, "pack"), only="display", spacing=4.0, margin=10.5)
    import rectpack
    items = [dict(name=p["name"], material="PLA", dims=p["dims"], qty=1) for p in core["parts"]]
    orig, best = rectpack.MaxRectsBssf, None
    for algo in ("MaxRectsBssf", "MaxRectsBaf", "MaxRectsBlsf", "MaxRectsBl", "GuillotineBssfSas", "GuillotineBafSas",
                 "GuillotineBlsfSas", "SkylineBl", "SkylineMwf", "SkylineMwfl"):
        for sp in (4.0, 3.0):
            rectpack.MaxRectsBssf = getattr(rectpack, algo)
            pl, err = PK.pack_plates(items, (256.0, 256.0, 260.0), sp, 10.5)
            if not err and (best is None or len(pl) < best[0]):
                best = (len(pl), algo, sp)
    rectpack.MaxRectsBssf = getattr(rectpack, best[1])
    print("packing:", best)
    core = PK.run(out, os.path.join(out, "pack"), only="display", spacing=best[2], margin=10.5)
    rectpack.MaxRectsBssf = orig
    opt = PK.run(out, os.path.join(out, "pack_optional"), only="optional", spacing=4.0, margin=10.5)
    for pl in core["plates"]:
        pl["dir"], pl["optional"] = "pack", False
    n = len(core["plates"])
    for i, pl in enumerate(opt["plates"]):
        pl["dir"], pl["optional"] = "pack_optional", True
        pl["name"] = f"plate_{n + i + 1}_optional_PLA.3mf"
    for pl in core["plates"]:
        pl["name"] = pl["file"]
    return dict(plates=core["plates"] + opt["plates"], core=core, opt=opt)


def zip_stls(out, asm, rep):
    import zipfile
    zf = os.path.join(out, "grappstar_launcher_display_stl.zip")
    with zipfile.ZipFile(zf, "w", zipfile.ZIP_DEFLATED) as z:
        for n, p in asm.parts.items():
            if p.printed:
                opt = p.subsystem == "optional"
                z.write(os.path.join(out, "pack_optional" if opt else "pack", "stl", n + ".stl"),
                        f"{'optional' if opt else 'PLA'}/{n}.stl")
        for pl in rep["plates"]:
            z.write(os.path.join(out, pl["dir"], "plates", pl["file"]), "plates/" + _plate_name(pl))
    return zf


def _plate_name(pl):
    return pl["name"]


def render(out, asm):
    import shutil
    from tools import render as R
    data = R.build_data(asm, out, [dict(S0)], title="GrappStar launcher, display build, KEDGE v34")
    srv, url = R.serve({"data.js": R.data_js(data).encode(), "index.html": os.path.join(R.WEB, "render.html")})
    views = os.path.join(out, "views")
    shutil.rmtree(views, ignore_errors=True)
    os.makedirs(views)
    dst = os.path.join(out, "renders")
    os.makedirs(dst, exist_ok=True)
    shots, items = [], []
    for vn, ops in (("iso_left", [["view", 55, 24, 0.95]]), ("iso_right_motor", [["view", -55, 24, 0.95]]),
                    ("side", [["view", 90, 4, 0.95]]), ("top", [["view", 0, 88, 0.95]]),
                    ("launcher", [["cam", [80, 140, 120], 760, 60, 24]]),
                    ("ebox_motor", [["cam", [120, -40, -120], 650, -40, 40]])):
        f = os.path.join(views, f"display_{vn}.png")
        shots.append(dict(file=f, ops=[["sample", 0], ["cut", None], ["ground", True]] + ops))
        items.append((f, vn.replace("_", " ")))
    try:
        R.run_shots(dict(url=url + "index.html", width=1200, height=860, shots=shots))
    finally:
        srv.shutdown()
    for f, _ in items:
        shutil.copy(f, os.path.join(dst, os.path.basename(f)))
    R.grid_sheet([(os.path.join(dst, os.path.basename(f)), lab) for f, lab in items], 2,
                 os.path.join(dst, "display_sheet.png"), "GrappStar launcher, display build",
                 f"real size, rail at {PITCH:.0f} deg, v34 dart loaded, real springs, wiper motor and boards in place")


def note(out, asm, rep, audit_line):
    B = asm.builder
    P = asm.P
    plates = rep["plates"]
    core = [pl for pl in plates if not pl["optional"]]
    opt = [pl for pl in plates if pl["optional"]]

    def ptxt(pl):
        names = sorted({i["name"] for i in pl["items"]})
        return (f"{_plate_name(pl)}: {pl.get('mass_g', 0):.0f} g, about {pl.get('time_h', 0):.1f} h: " + ", ".join(names))
    cm = sum(pl.get("mass_g", 0) for pl in core)
    ct = sum(pl.get("time_h", 0) for pl in core)
    om = sum(pl.get("mass_g", 0) for pl in opt)
    ot = sum(pl.get("time_h", 0) for pl in opt)
    import launch_aim as AIM
    hs = [tuple(round(v) for v in xy) for xy in AIM.deck_holes(P)]
    eb = [tuple(round(v) for v in xy) for xy in B.ebox_pads]
    lg = [(round(x), round(y)) for x, y in B.leg_holes]
    lo, hi = np.array([1e9] * 3), np.array([-1e9] * 3)
    for n, p in asm.parts.items():
        if (p.printed and p.subsystem == "display") or n == "real_wiper_motor":
            a, b = prim.bbox(prim.place(p.solid, p.place))
            lo, hi = np.minimum(lo, a), np.maximum(hi, b)
    L = []
    L.append("GrappStar launcher, DISPLAY build (real size, launcher only), KEDGE v34 dart")
    L.append("")
    L.append(f"The D50 launcher as it looks, at real size, rail fixed at {PITCH:.0f} deg, carriage at HOME, the v34 dart "
             "on the cradle. Nothing moves. Reel and drum are left out (optional hollow look-alikes on a 4th plate), "
             "no deck, no guards, no stops, no sensors. All PLA.")
    L.append("")
    L.append("PRINT (X2D, 235 x 235 safe area)")
    for pl in core:
        L.append("  " + ptxt(pl))
    L.append(f"  Core total: {cm:.0f} g, about {ct:.1f} h on {len(core)} plates.")
    for pl in opt:
        L.append("  Optional " + ptxt(pl))
    if opt:
        L.append(f"  With the optional plate: {cm + om:.0f} g, about {ct + ot:.1f} h.")
    L.append("  Estimate: printed mass from each part's volume and surface (2 walls of 0.42 mm, 8 % infill core); time "
             f"= plastic at {RATE:g} mm3/s plus {LAYER_S:g} s per 0.28 mm layer per plate. Real times vary by about "
             "+-20 %. Next cuts if it runs long: print the ebox at 5 % infill without the lid bosses (about -15 g), "
             "skip the worm and cap (glue the crank instead, -13 g), leave out the drive gear (-21 g, hidden between "
             "the cheeks).")
    L.append("")
    L.append("SLICER")
    for x in SLICER:
        L.append("  " + x)
    L.append("")
    L.append("WHAT GOES WHERE")
    L.append("  Aim housing: stands on the board (4 screws). The sleeve's outer journal sits in its bushing; the crank "
             "with its printed pin sits between the housing walls, the worm in its saddles under the cap (it holds "
             "the crank, so the rail angle); the conrod goes from the crank pin to the lug on the left rail wall.")
    L.append("  Rail: rear, mid and front pieces glued at their joints; the mid piece's cheeks ride on the sleeve. "
             "Drive gear on the sleeve rib between the cheeks; big gear on its pin in the cheek bosses, "
             f"fixed by {asm.gear_fix}.")
    L.append("  Carriage: glue the two halves, slide it into the track from the front, yoke on its back (2 x M3 x 10). "
             "The springs pull it forward onto the big gear's first tooth: that is HOME.")
    L.append("  Main axle: its hub end goes onto the real wiper motor shaft through the motor plate; the d12 spigot "
             "carries the sleeve. Motor plate: drops into the leg's slot (glue), leg screwed to the board.")
    L.append("")
    L.append("HARRY'S REAL PARTS (checked in the model: no clashes)")
    L.append("  Springs: two Century C-243 hang from the printed anchor pins in the rail front posts (or 8 mm rod) to "
             "the pegs under the yoke arms, at their HOME length (slightly stretched, about 10 N each).")
    L.append("  Wiper motor: bolts to the rear face of the motor plate with 3 x M6 through the slots (+-4 mm: the D50 "
             "pattern is assumed, not measured; measure yours and open the slots if needed). Its output boss sits "
             "in the 30 mm hole, its shaft in the axle hub. Not wired.")
    L.append("  Boards in the electronics box (D50 bay, thinner walls, no lid): ESP32-S3 DevKitC-1 on its cradle with "
             "the USB port at the wall opening, BTS7960 on 4 standoffs (M3 self tap or inserts), DRV8833 and MP1584 "
             "in their pockets with cable ties. Box: 4 screws into the board through its floor pads.")
    L.append("")
    L.append("BOARD (any flat board, all coordinates in mm from the main axis point on the board, x forward, y left)")
    L.append(f"  Needs about {hi[0] - lo[0]:.0f} x {hi[1] - lo[1]:.0f} mm (x {lo[0]:.0f} to {hi[0]:.0f}, y {lo[1]:.0f} to "
             f"{hi[1]:.0f}); the launcher is {hi[2] - DECK_Z:.0f} mm tall at {PITCH:.0f} deg.")
    L.append(f"  Aim housing 4 x M3 (or wood screws): {hs}. Motor leg 3 screws: {lg}. Electronics box 4 screws: {eb}.")
    if opt:
        ob = []
        for n in ("opt_drum_saddle", "opt_reel_bracket"):
            a, b = prim.bbox(prim.place(asm.parts[n].solid, asm.parts[n].place))
            ob.append(f"{n.replace('opt_', '').replace('_', ' ')} at x {a[0]:.0f}..{b[0]:.0f}, y {a[1]:.0f}..{b[1]:.0f}")
        L.append("  Optional, 2 screws each (or glue): " + "; ".join(ob) + " (the reel sits behind the launcher, so the "
                 "board then needs to reach back that far).")
    L.append("")
    L.append(f"Interference check at the display pose: {audit_line}. Source: base/tools/display.py (dart-v1).")
    txt = "\n".join(L) + "\n"
    for bad in (chr(0x2014), chr(0x2013)):
        assert bad not in txt
    open(os.path.join(out, "DISPLAY_NOTE.txt"), "w").write(txt)
    from tools.show_pack import _pdf
    _pdf(L, os.path.join(out, "DISPLAY_NOTE.pdf"))
    return txt


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    out = argv[0]
    os.makedirs(out, exist_ok=True)
    P = load_params()
    asm = build(P)
    asm.export(out)
    from tools import audit as AU
    rep = AU.run(asm, [dict(S0)], 3)
    s = AU.summary(rep)
    with open(os.path.join(out, "audit.txt"), "w") as fh:
        fh.write(s + "\n")
    import json
    with open(os.path.join(out, "audit.json"), "w") as fh:
        json.dump(rep, fh, indent=1, default=str)
    print(s)
    lines = s.strip().splitlines()
    audit_line = f"{lines[-1]} ({lines[0]})"
    prep = pack_all(out)
    zf = zip_stls(out, asm, prep)
    import shutil
    pd = os.path.join(out, "plates")
    shutil.rmtree(pd, ignore_errors=True)
    os.makedirs(pd)
    for pl in prep["plates"]:
        shutil.copy(os.path.join(out, pl["dir"], "plates", pl["file"]), os.path.join(pd, _plate_name(pl)))
        shutil.copy(os.path.join(out, pl["dir"], "plates", pl["png"]), os.path.join(pd, _plate_name(pl).replace(".3mf", ".png")))
    print(note(out, asm, prep, audit_line))
    if "--no-render" not in argv:
        render(out, asm)
    print(zf)
    return 0 if rep["clean"] else 1


if __name__ == "__main__":
    sys.exit(main())
