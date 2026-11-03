"""The base station taken apart by mechanism (not by CAD owner): which parts belong to each mechanism, what each one
does, what drives it, and when it works in the mission. Used by tools.sim (the "show" picker in the sim page) and by
scripts/make_site.py (the "Mechanisms one by one" page).

    python -m tools.mechanisms OUT [--shots]     prints the groups and counts; --shots writes OUT/shots/mech_<key>.png

A part may sit in more than one group (the switches also appear under "Switches and sensors"). Every part that no
mechanism names falls into "Frame and electronics", so the groups always cover the whole assembly.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from fnmatch import fnmatchcase

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# key, title, part name patterns, the TIMELINE phase to picture it in, and the plain-language description
MECHS = [
    dict(key="power", title="Motor and torque limiter", phase="cock",
         parts=["frame_wiper_motor", "frame_wiper_shaft", "frame_motor_adapter", "frame_motor_leg", "frame_can_stay",
                "drum_coupler_hub", "drum_coupler", "drum_shear_pin", "drum_shaft", "drum_index_block",
                "drum_index_lever", "drum_magnet_cap", "sensors_as5600*"],
         job="The one actuator. A 12 V car wiper motor (about 45 rpm, self-locking worm gearbox) turns the main shaft "
             "for everything: launch, aim, wrap drive, reel and take-up.",
         drive="12 V from the BTS7960 driver; direction and duty from the ESP32.",
         when="Every phase that moves something.",
         how="The coupler hub clamps on the wiper shaft (D bore on the flats, M8 nut). A printed shear pin passes the "
             "torque to the coupler and breaks at about 5 N m, so a jam breaks a 10 cent pin, not a gear. The coupler "
             "carries the 50T bus gear (always turning with the motor, it feeds the take-up) and drives the hex main "
             "shaft. The AS5600 magnet sensor on the shaft end reads the wiper angle; a 6-bump cam and contact is the "
             "fallback when there is no magnet.",
         note="Everything downstream shares this one motor, which is why the rest exists: each other mechanism is a way "
              "of choosing what this motor turns."),
    dict(key="selector", title="Selector drum (the gear shifter)", phase="reel_engage",
         parts=["drum_drum_a", "drum_drum_b", "drum_axle", "drum_n20", "drum_n20_*", "drum_fork", "drum_rail",
                "drum_collar", "drum_dogring_launch", "drum_dog_post", "drum_gear_reel", "drum_reel_washer",
                "drum_bracket_a", "drum_bracket_b", "drum_switch_block", "drum_sw_lever_*", "drum_hand_lever"],
         job="Chooses what the wiper drives: the launcher (LAUNCH), nothing (NEUTRAL), the wrap cup (WRAP) or the "
             "capstan (REEL), always in that physical order.",
         drive="A small N20 gearmotor (12:1 onto the drum, about 8 deg/s), from the DRV8833.",
         when="Between phases: LAUNCH for aim and cock, NEUTRAL after the shot, WRAP right after landing, REEL for the "
              "tow and reel-in, then back.",
         how="Drum A and drum B are one barrel on an axle. A groove in drum B moves a springy fork, which slides the dog "
             "collar along the hex main shaft: forward into the LAUNCH dog ring (to the launch sleeve), back into the "
             "REEL gear (to the capstan), or parked between them (NEUTRAL). Drum A also carries the segment gear that "
             "drives the wrap cup while the drum passes its WRAP sector, and a cam that pushes the storage reel brake "
             "bar (clutch LIGHT or FIRM). Three contacts read a Gray code off drum B, so the firmware always knows the "
             "drum position, even after a power cut. A hand lever turns it if the N20 fails.",
         note="This is the densest part of the design: one small motor does the work of a shifter, a wrap motor and a "
              "brake actuator, so its timing (dog windows, code edges, cam positions) has to be right."),
    dict(key="wrap", title="Wrap cup", phase="wrap",
         parts=["drum_idler", "drum_idler_gear", "drum_bracket_idler", "drum_bracket_idler2", "reel_k", "reel_cup",
                "reel_cup_clip", "reel_gk", "reel_k_clip", "reel_sw_bracket", "reel_sw_base", "reel_sw_lever",
                "reel_sw_hw"],
         job="Lays 7 turns of the tether round the capstan right after landing, so the capstan can grip it for the tow.",
         drive="The selector drum's segment gear (N20 power), not the wiper.",
         when="WRAP (right after landing, line still slack) and UNWRAP (before the final approach).",
         how="While the drum turns through its WRAP sector, its segment gear spins the wrap idler (10T pinion, 42T "
             "output). That drives the 24T cup gear on the cup shaft, which runs inside the capstan tube. The cup on the "
             "front orbits the capstan 7 times, its ring laying the line on as wraps. A lock arc holds the idler still "
             "outside the sector. SW_ARM rides a track on the cup gear and opens at the cup's home notch, so the "
             "firmware counts the turns.",
         note="This is the step that replaces a person wrapping the line by hand. Without it the capstan cannot pull."),
    dict(key="capstan", title="Capstan winch (tow and reel-in)", phase="tow",
         parts=["reel_wall", "reel_sprag_hub", "reel_hub_screws", "reel_sprag", "reel_c", "reel_c_clip", "reel_gc",
                "reel_drum"],
         job="Pulls the rover toward the anchored dart (the tow), then reels the released dart back in.",
         drive="The wiper, through the REEL dog of the selector (collar in REEL).",
         when="REEL: fly-in, check pull, tow, reel-in.",
         how="The REEL gear (50T) on the main shaft drives the 22T capstan gear (2.27 x wiper speed), keyed on the "
             "capstan tube, which carries the capstan drum (40 mm, Harry's v7). The line holds by friction from the 7 "
             "wraps (capstan effect). Sprag #2 (CSK20PP) lets the capstan turn only in the wind-in sense, so the load "
             "cannot pull line back out; the wiper's self-locking worm holds it too. Tension is measured from the motor "
             "current (about 13 N per amp above free running), with a 55 N cap.",
         note="Mechanically this is the simplest train: one gear pair and a one-way clutch."),
    dict(key="storage", title="Storage reel, clutch and take-up", phase="flight",
         parts=["reel_bus", "reel_swing_arm", "reel_gi", "reel_gr", "reel_s", "reel_s_clip", "reel_spool",
                "reel_spool_flange", "reel_brg_*", "reel_t", "reel_rocker", "reel_rocker_pin", "reel_trim_screws",
                "reel_crank", "reel_crank_washer", "reel_crank_screw", "reel_firm_screw", "reel_bar", "reel_bar_cap_*",
                "reel_bar_screws"],
         job="Holds the 20 m of Dyneema: pays it out in flight against a light drag, and winds the slack back in.",
         drive="Payout: the dart pulls it. Take-up: the wiper's forward turns through the bus gear. Drag level: the "
               "selector drum's cam.",
         when="FLIGHT (payout, LIGHT drag), COCK and TIDY (take-up), final approach (FIRM take-up pulls the dart home).",
         how="The storage reel turns on two 608s. Its rear face is a friction clutch: the rocker, a flexible PPA-CF "
             "lever, presses a thrust ring on it. LIGHT is about 0.5 N at the line (low enough for flight); when the "
             "drum cam pushes the brake bar, the bell crank presses the rocker harder for FIRM (about 5 N, enough to "
             "pull the dart home). The bus gear on the coupler always turns with the wiper. On forward turns only, the "
             "swing arm drags the swing idler into mesh with the reel gear, so the clutch disk winds the reel through "
             "the clutch (it slips once the line is tight). On backward turns the idler swings out.",
         note="The swing idler is a direction-sensitive clutch made of a gear on an arm: forward winds, backward does "
              "nothing."),
    dict(key="launcher", title="Launcher: rail, carriage and springs", phase="cock",
         parts=["launch_rail_*", "launch_carriage", "launch_carriage_yoke", "launch_yoke_peg_*", "launch_anchor_pin_*",
                "launch_spring_*", "launch_stop_*", "launch_guard_*", "launch_dart"],
         job="Throws the dart: two extension springs (12.1 J stored) pull the carriage 241 mm along the rail.",
         drive="Stored spring energy, loaded by the cock train.",
         when="COCK (springs stretched), FIRE (released), then stopped by the flexure stops.",
         how="The rail is printed in three pieces with splice straps. The carriage runs on shoes in the rail, holding "
             "the dart in a cradle with its tail on the pusher. Its yoke holds the two spring pegs (8 mm steel rod). "
             "At the end of the stroke the yoke hits two printed flexure stops (16-loop springs), which take the "
             "carriage's energy so the dart flies on. The dart leaves at about 6.85 m/s for about 4.2 m at 34 deg.",
         note="No trigger mechanism: the release comes from the big gear running out of teeth (next mechanism)."),
    dict(key="cock", title="Cock and release train", phase="cock",
         parts=["launch_sleeve", "launch_drive_gear", "launch_drive_washer", "launch_sprag", "launch_big_gear",
                "launch_gear_washer", "launch_gear_pin", "launch_gear_clip"],
         job="Draws the carriage back against the springs and lets it go at the end, with no separate trigger.",
         drive="The wiper turning FORWARD, through the LAUNCH dog of the selector.",
         when="COCK and FIRE; the reset run after the shot.",
         how="The launch sleeve turns with the main shaft when the collar is in LAUNCH. Sprag #1 inside the 26T drive "
             "gear drives only on forward turns. The drive gear turns the 40T big gear, whose sector layer (27 of 40 "
             "teeth) meshes the rack under the carriage and pulls it back. Past the last tooth the rack is free: the "
             "springs fire. The big gear keeps turning to its toothless gap, ready for the next cock (SW_GAP sees the "
             "gap). On backward turns the sprag freewheels, so aiming does not cock.",
         note="Fire is a geometry event, not an actuator: it happens at a fixed wiper angle, which the firmware tracks."),
    dict(key="aim", title="Aim: splitter, worm and crank-rocker", phase="aim",
         parts=["launch_aim_*"],
         job="Tilts the rail between 6 and 47 deg and holds it there with no power.",
         drive="The wiper turning BACKWARD, through the LAUNCH dog of the selector.",
         when="AIM, and RAIL LOW after the shot (down to 6 deg before the tow).",
         how="The splitter sun is keyed on the launch sleeve. A swing arm on the sleeve carries an idler; a drag pad "
             "in its hub makes it follow the sun's direction, so on backward turns the idler swings into mesh with the "
             "output gear, and on forward turns (cocking) it swings out. The output gear drives a 2:1 bevel pair into a "
             "single-start worm (self-locking), which turns the worm wheel. That wheel is also the crank: crank, conrod "
             "and lug form a crank-rocker that lifts the rail. The worm holds the angle; SW_RAIL_LOW marks the low end.",
         note="This is how one motor direction aims and the other cocks: the sprag and the swing idler split the two "
              "directions."),
    dict(key="tether", title="Tether path", phase="tow",
         parts=["reel_shroud", "reel_exit_post", "reel_pulley1", "reel_p1_*", "reel_p2_*", "reel_pulley2",
                "launch_pulley_*", "launch_muzzle_*", "launch_line_*"],
         job="Guides the line from the storage reel past the capstan, up the rail to the dart, without rubbing.",
         drive="Passive.",
         when="Always: payout, wrap, tow, reel-in.",
         how="From the storage reel the line passes the shroud and exit post, round pulley 1 and the pulley 2 swivel "
             "(which follows the rail angle), under the rail to the pivot pulley, round the bottom and top pulleys "
             "behind the rail, forward along the rail to the pusher, and over the lip roller at the carriage front when "
             "the dart is ahead and below. Every turn under tow load is a pulley on a 693ZZ bearing or an M3 axle.",
         note="The whole-assembly tether check confirms the line clears every part in every phase."),
    dict(key="sensors", title="Switches and sensors", phase="cock",
         parts=["sensors_*", "launch_sw_*", "launch_seat_*", "drum_switch_block", "drum_sw_lever_*", "drum_dog_post",
                "drum_index_block", "drum_index_lever", "drum_magnet_cap", "reel_sw_*"],
         job="Tell the firmware where everything is, so it never has to guess.",
         drive="Passive contacts (printed levers on screw tips) and one AS5600 magnet sensor.",
         when="Always.",
         how="AS5600: wiper angle. Drum code (3 contacts): selector position. Dog contacts: collar fully in. SW_HOME: "
             "carriage at rest. SW_SEATED: dart on the cradle. SW_GAP: big gear in its gap (ready to cock). SW_RAIL_LOW: "
             "rail at the bottom. SW_ARM: wrap cup at home (counts wraps). SW_INDEX: fallback angle source.",
         note="Most of these are printed contacts, not bought microswitches: two screw tips and a printed lever."),
]

FRAME = dict(key="frame", title="Frame and electronics", phase="load", parts=["frame_*"],
             job="Holds everything on the rover and carries the electronics.",
             drive="Passive.", when="Always.",
             how="Four deck tiles bolt to the rover. The motor plate, cheek and legs hold the main axis; the electronics "
                 "bay holds the ESP32, the BTS7960 and DRV8833 drivers, the buck converter and the HX711 option. Guards "
                 "cover the gears and springs; mounts hold the camera and the video receiver.",
             note="")


def groups(names) -> list:
    """[{key, title, phase, parts, job, drive, when, how, note}] for the given part names; the frame group takes every
    part that no mechanism names (so nothing is left out)."""
    names = list(names)
    out, claimed = [], set()
    for m in MECHS:
        ps = [n for n in names if any(fnmatchcase(n, pat) for pat in m["parts"])]
        claimed.update(ps)
        out.append({**m, "parts": ps})
    out.append({**FRAME, "parts": [n for n in names if n not in claimed]})
    return out


def stats(asm_json: dict) -> dict:
    """{key: dict(parts, printed, bought, moving, mass_g)} from an exported asm.json."""
    parts = {p["name"]: p for p in asm_json["parts"]}
    joints = {j["name"]: j for j in asm_json["joints"]}

    def moving(jn):
        while jn:
            j = joints.get(jn)
            if j is None:
                return False
            if j["kind"] != "fixed" or j.get("gear"):
                return True
            jn = j["parent"]
        return False

    res = {}
    for g in groups(parts):
        ps = [parts[n] for n in g["parts"]]
        res[g["key"]] = dict(parts=sum(int(p.get("qty") or 1) for p in ps),
                             printed=sum(int(p.get("qty") or 1) for p in ps if p["material"] != "bought"),
                             bought=sum(int(p.get("qty") or 1) for p in ps if p["material"] == "bought"),
                             moving=sum(1 for p in ps if moving(p.get("joint"))))
    return res


def shots(out: str) -> list:
    """OUT/shots/mech_<key>.png: each mechanism alone in the sim, in the middle of its picture phase."""
    from tools.render import run_shots
    from tools.params import load_params
    from tools import timeline as TL
    ph = {p["name"]: p for p in TL.phases(load_params())}
    sd = os.path.join(out, "shots")
    os.makedirs(sd, exist_ok=True)
    url = "file://" + os.path.abspath(os.path.join(out, "sim.html"))
    job, files = [], []
    for m in MECHS + [FRAME]:
        p = ph.get(m["phase"]) or {}
        t = 0.5 * (p.get("t0", 0.0) + p.get("t1", 0.0))
        f = os.path.join(sd, f"mech_{m['key']}.png")
        job.append(dict(file=f, ops=[["SIM_CLEAN"], ["SIM_PAUSE"], ["SIM_SEEK", t], ["SIM_MECH", m["key"]]]))
        files.append(f)
    run_shots(dict(url=url, width=1200, height=820, ready="window.READY === true", shots=job))
    return files


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out")
    ap.add_argument("--shots", action="store_true")
    a = ap.parse_args(argv)
    asm = json.load(open(os.path.join(a.out, "asm.json")))
    st = stats(asm)
    for g in groups([p["name"] for p in asm["parts"]]):
        s = st[g["key"]]
        print(f"{g['key']:9s} {s['parts']:3d} parts ({s['printed']} printed, {s['bought']} bought), {s['moving']} moving: {g['title']}")
    if a.shots:
        for f in shots(a.out):
            print(f)


if __name__ == "__main__":
    main()
