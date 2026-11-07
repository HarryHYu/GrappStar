"""Re-learn the vision thresholds on Harry's own footage (README section "Re-learn on your own footage").

Everything the dart vision decides was tuned on synthetic flights. Real footage will differ. These commands
measure the real numbers from recorded runs and write them where the software reads them at start:

  python3 -m grappstar learn extract runs/2026*          landing pictures to sort by hand -> labelled/_unsorted
  python3 -m grappstar learn latency runs/2026*          capture delay of the dart video -> prints the setting
  python3 -m grappstar learn flight shots.csv            range checks against the tape -> calib/flight_tuning.json
  python3 -m grappstar learn terrain labelled            terrain classifier on your pictures -> calib/terrain_prototypes.json

Each run folder is what a mission writes (runs/<date-time>/): dart_shotN.avi (+ .frames.jsonl time stamps),
flightlog_shotN.json (with clock_offset_s) and mission.jsonl (the commanded pitch).
shots.csv: one line per shot, `run_folder,shot,range_m` with range_m measured with a tape from the ground under the
launcher pivot to the mark where the dart first hit the ground.
"""
from __future__ import annotations

import csv
import glob
import json
import os
import statistics
import sys
from typing import Dict, List, Optional, Sequence

HERE = os.path.dirname(os.path.abspath(__file__))
JETSON_DIR = os.path.dirname(HERE)
CALIB = os.path.join(JETSON_DIR, "calib")
SETTINGS = os.path.join(JETSON_DIR, "settings.json")


def settings() -> dict:
    try:
        with open(SETTINGS) as f:
            return json.load(f)
    except Exception:
        return {}


def dart_lens_from_settings(size=(720, 480)):
    from .vision.lens import DartLens
    s = settings()
    path = s.get("dart_cam_calib") or os.path.join(CALIB, "dartcam.json")
    if not os.path.isabs(path):
        path = os.path.join(JETSON_DIR, path)
    return DartLens.from_settings(path, s.get("dart_hfov_deg"), None, s.get("dart_aspect"), size)


def latency_setting() -> float:
    return float(settings().get("dart_video_latency_s", 0.06))


def shot_pitch(folder: str) -> Optional[float]:
    """The commanded pitch of the last aim in a run (mission.jsonl 'aimed at X deg')."""
    import re
    p = os.path.join(folder, "mission.jsonl")
    if not os.path.exists(p):
        return None
    from . import jlog
    pitch = None
    for r in jlog.read(p):
        m = re.search(r"aimed at ([-0-9.]+) deg", str(r.get("msg") or ""))
        if m:
            pitch = float(m.group(1))
    return pitch


def shots(folders: Sequence[str]):
    """(folder, shot number, video path, flightlog dict or None) for every recorded shot."""
    from .replay import find_shot
    import re
    for folder in folders:
        for v in sorted(glob.glob(os.path.join(folder, "dart_shot*.avi")) +
                        glob.glob(os.path.join(folder, "dart_shot*_jpg"))):
            m = re.search(r"shot(\d+)", os.path.basename(v))
            if not m:
                continue
            n = int(m.group(1))
            _, flp, _ = find_shot(folder, n)
            fl = None
            if flp:
                with open(flp) as f:
                    fl = json.load(f)
            yield folder, n, v, fl


# ---------------------------------------------------------------- extract

def extract(folders: Sequence[str], out_dir: str = "labelled", n: int = 5) -> int:
    """Save the n sharpest landed frames of every shot (raw, as recorded) into out_dir/_unsorted/ for sorting into
    out_dir/<class>/ by hand (bedrock, hard_ground, loose_soil, sand_dust, vegetation, water_void)."""
    import cv2
    from .replay import load_frames
    from .vision import flight
    from .vision.lens import rectify_frames
    dest = os.path.join(out_dir, "_unsorted")
    os.makedirs(dest, exist_ok=True)
    for c in ("bedrock", "hard_ground", "loose_soil", "sand_dust", "vegetation", "water_void"):
        os.makedirs(os.path.join(out_dir, c), exist_ok=True)
    count = 0
    for folder, k, vid, fl in shots(folders):
        frames = load_frames(vid)
        if not frames:
            continue
        lens = dart_lens_from_settings((frames[0].img.shape[1], frames[0].img.shape[0]))
        rect = rectify_frames(frames, lens)
        res = flight.analyze_flight(rect, fl, video_latency_s=latency_setting())
        land = flight.landing_frames(rect, res, n=15)
        idx = {f.idx for f in sorted(land, key=lambda f: flight.sharpness(f.img), reverse=True)[:n]}
        base = os.path.basename(os.path.normpath(folder))
        for f in frames:
            if f.idx in idx:
                p = os.path.join(dest, f"{base}_shot{k}_{f.idx:05d}.png")
                cv2.imwrite(p, f.img)
                count += 1
    print(f"{count} pictures in {dest}: move each into the folder of its ground type, delete unclear ones")
    return count


# ---------------------------------------------------------------- latency

def latency(folders: Sequence[str]) -> Optional[float]:
    """Capture delay of the dart video: video launch time minus the IMU launch time on the Jetson clock, median over
    the shots. Needs flight logs with clock_offset_s (the mission saves them). The video launch can be seen late on
    a plain sky view, so the median of 3 or more shots is used, and shots with a rail view of mostly ground count
    more (the motion starts clearly there)."""
    from .replay import load_frames
    from .vision import flight
    from .vision.lens import rectify_frames
    vals: List[float] = []
    for folder, k, vid, fl in shots(folders):
        if not fl or fl.get("clock_offset_s") is None:
            print(f"{folder} shot {k}: no flight log with clock offset, skipped")
            continue
        frames = load_frames(vid)
        if not frames:
            continue
        lens = dart_lens_from_settings((frames[0].img.shape[1], frames[0].img.shape[0]))
        rect = rectify_frames(frames, lens)
        res = flight.analyze_flight(rect, None)
        ev = flight.imu_events(fl)
        vl = res.video_times.get("launch")
        if vl is None or ev.get("launch") is None:
            print(f"{folder} shot {k}: launch not found in the video or the log, skipped")
            continue
        d = vl - (ev["launch"] + float(fl["clock_offset_s"]))
        print(f"{folder} shot {k}: video launch {d * 1000:+.0f} ms after the IMU launch")
        if -0.05 <= d <= 0.4:
            vals.append(d)
    if len(vals) < 3:
        print(f"only {len(vals)} usable shots: record at least 3 (fire into soft ground, rail aimed low so the "
              "camera sees ground)")
        return None
    med = statistics.median(vals)
    print(f"capture delay {med * 1000:.0f} ms (spread {min(vals) * 1000:.0f} to {max(vals) * 1000:.0f} ms over "
          f"{len(vals)} shots). Set \"dart_video_latency_s\": {med:.3f} in settings.json")
    return med


# ---------------------------------------------------------------- flight

def _read_tape(path: str) -> List[dict]:
    rows = []
    with open(path) as f:
        for r in csv.reader(f):
            if not r or r[0].strip().startswith("#") or r[0].strip() == "run_folder":
                continue
            rows.append({"folder": r[0].strip(), "shot": int(r[1]), "range_m": float(r[2])})
    return rows


def flight_tune(tape_csv: str, write: bool = True, out: Optional[str] = None) -> dict:
    """Compare the analysis with tape-measured ranges and measure the video timing on real shots.

    Writes jetson/calib/flight_tuning.json (merging with what is there):
    - table_scale: median of tape range / C3 table range at the commanded pitch. The aim solver and the range
      estimate both use it (ballistics.site_table_scale). Also tell dart-physics if it is far from 1.
    - VID_T_LATE: 90th percentile of how late the video impact is against the IMU impact (+10 ms), so the video
      only path allows for the real crash and skid.
    - VID_IMPACT_LATE_TYP: the median of that lateness."""
    from .ballistics import Ballistics
    from .replay import load_frames
    from .vision import flight
    from .vision.lens import rectify_frames
    from .analysis import run_post_flight
    rows = _read_tape(tape_csv)
    bal = Ballistics.load()
    scales, late, report = [], [], []
    for r in rows:
        try:
            vid = next(v for f, k, v, fl in shots([r["folder"]]) if k == r["shot"])
        except StopIteration:
            print(f"{r['folder']} shot {r['shot']}: no video, skipped")
            continue
        fl = next((fl for f, k, v, fl in shots([r["folder"]]) if k == r["shot"]), None)
        frames = load_frames(vid)
        lens = dart_lens_from_settings((frames[0].img.shape[1], frames[0].img.shape[0])) if frames else None
        pitch = shot_pitch(r["folder"])
        row = {"folder": r["folder"], "shot": r["shot"], "tape_m": r["range_m"], "pitch": pitch}
        if pitch is not None:
            tab = bal.at_pitch(pitch).get("range_m")
            if tab:
                row["table_m"] = round(tab / bal.range_scale, 3)
                scales.append(r["range_m"] / row["table_m"])
        for name, use_fl in (("imu", fl), ("video", None)):
            res = run_post_flight(frames, use_fl, None, aim_pitch_deg=pitch, dart_lens=lens,
                                  clock_offset_s=(fl or {}).get("clock_offset_s") if use_fl else None,
                                  video_latency_s=latency_setting(), do_mosaic=False)
            fr = res.get("flight") or {}
            row[name + "_m"] = fr.get("range_m")
            if name == "imu" and fl:
                ev = flight.imu_events(fl)
                rect = rectify_frames(frames, lens)
                vres = flight.analyze_flight(rect, None)
                off = fl.get("clock_offset_s")
                if ev.get("impact") is not None and vres.video_times.get("impact") is not None and off is not None:
                    late.append(vres.video_times["impact"] - (ev["impact"] + float(off) + latency_setting()))
        report.append(row)
        print(json.dumps(row))
    out_d = {}
    if len(scales) >= 3:
        out_d["table_scale"] = round(statistics.median(scales), 3)
    if len(late) >= 3:
        late.sort()
        p90 = late[min(len(late) - 1, int(0.9 * len(late)))]
        out_d["VID_T_LATE"] = round(max(0.05, p90 + 0.01), 3)
        out_d["VID_IMPACT_LATE_TYP"] = round(max(0.0, statistics.median(late)), 3)
    if not out_d:
        print("not enough shots (3 or more with a commanded pitch and a flight log): nothing written")
        return {"rows": report}
    print("learned:", json.dumps(out_d))
    if out_d.get("table_scale") and abs(out_d["table_scale"] - 1) > 0.1:
        print("NOTE: the real ranges differ from the C3 table by more than 10 percent: tell dart-physics / the base "
              "physics model (base/out/ballistics.json), and check the brake and the springs")
    if write:
        path = out or os.path.join(CALIB, "flight_tuning.json")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        old = {}
        if os.path.exists(path):
            try:
                with open(path) as f:
                    old = json.load(f)
            except Exception:
                old = {}
        old.update(out_d)
        with open(path, "w") as f:
            json.dump(old, f, indent=1)
        print(f"written {path} (serve and the analysis read it at start)")
    return {"rows": report, "learned": out_d}


# ---------------------------------------------------------------- terrain

def _real_accuracy(real: Dict[str, list]) -> Optional[float]:
    from .vision import terrain
    import numpy as np
    n = ok = 0
    for c, vs in real.items():
        for v in vs:
            f = dict(zip(terrain.FEATURE_NAMES, v))
            pr = terrain.classify_features(f)["probs"]
            ok += int(max(pr, key=pr.get) == c)
            n += 1
    return ok / n if n else None


def terrain_learn(real_dir: str, write: bool = True, jobs: int = 4, quick: bool = False) -> dict:
    """Re-learn the terrain classifier with Harry's labelled pictures added (counted 3 times each) to the
    synthetic training set. Prints the accuracy on the real pictures before and after. Writes
    jetson/calib/terrain_prototypes.json, which analyze_landing prefers over the built-in constants."""
    from .vision import terrain
    lens = dart_lens_from_settings()
    real = terrain.real_samples(real_dir, lens=lens)
    n = {c: len(v) for c, v in real.items()}
    print("real pictures with usable ground per class:", n)
    if sum(n.values()) < 6:
        print("fewer than 6 usable pictures: nothing learned")
        return {"n": n}
    before = _real_accuracy(real)
    kw = dict(jobs=jobs, verbose=True, real_dir=real_dir, real_lens=lens)
    if quick:
        kw.update(flight_seeds=(), caddx_seeds=())
    protos = terrain.learn_prototypes(**kw)
    terrain._LEARNED = None
    old = terrain._PROTOTYPES
    terrain._PROTOTYPES = protos
    after = _real_accuracy(real)
    terrain._PROTOTYPES = old
    terrain._LEARNED = None
    print(f"accuracy on your pictures: before {before:.0%}, after {after:.0%} (the same pictures: check new ones "
          "with `python3 -m grappstar replay` too)")
    if write:
        path = terrain.site_prototypes_path()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(protos, f)
        print(f"written {path}; delete it to go back to the built-in classifier")
    return {"n": n, "before": before, "after": after}


def main(argv: Optional[Sequence[str]] = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(prog="python3 -m grappstar learn", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="what", required=True)
    e = sub.add_parser("extract", help="landing pictures of recorded runs to sort by hand")
    e.add_argument("runs", nargs="+")
    e.add_argument("--out", default="labelled")
    e.add_argument("--n", type=int, default=5)
    la = sub.add_parser("latency", help="capture delay of the dart video from recorded shots")
    la.add_argument("runs", nargs="+")
    fl = sub.add_parser("flight", help="range against the tape: table scale and video timing")
    fl.add_argument("tape_csv")
    fl.add_argument("--dry", action="store_true", help="print only, write nothing")
    te = sub.add_parser("terrain", help="terrain classifier with your labelled pictures")
    te.add_argument("labelled")
    te.add_argument("--dry", action="store_true")
    te.add_argument("--quick", action="store_true", help="textures and your pictures only (a minute, less robust)")
    te.add_argument("--jobs", type=int, default=4)
    a = ap.parse_args(argv)
    if a.what == "extract":
        return 0 if extract(a.runs, a.out, a.n) else 1
    if a.what == "latency":
        return 0 if latency(a.runs) is not None else 1
    if a.what == "flight":
        return 0 if flight_tune(a.tape_csv, write=not a.dry).get("learned") else 1
    if a.what == "terrain":
        r = terrain_learn(a.labelled, write=not a.dry, jobs=a.jobs, quick=a.quick)
        return 0 if r.get("after") is not None else 1
    return 2


if __name__ == "__main__":
    sys.exit(main())
