"""Replay a recorded run folder: re-run the flight and landing analysis on the saved dart video and flightlog, and
summarise the mission log. Works on a PC with no hardware.

  python3 -m grappstar replay jetson/runs/20261103-153000
"""
from __future__ import annotations

import glob
import json
import os
import re
from typing import Optional

from . import jlog
from .analysis import run_post_flight


def find_shot(folder: str, shot: Optional[int] = None):
    vids = sorted(glob.glob(os.path.join(folder, "dart_shot*.avi")) + glob.glob(os.path.join(folder, "dart_shot*_jpg")))
    if not vids:
        raise FileNotFoundError(f"no dart_shot*.avi in {folder}")
    def num(p):
        m = re.search(r"shot(\d+)", os.path.basename(p))
        return int(m.group(1)) if m else -1
    if shot is None:
        v = max(vids, key=num)
    else:
        c = [p for p in vids if num(p) == shot]
        if not c:
            raise FileNotFoundError(f"no dart_shot{shot} in {folder}")
        v = c[0]
    n = num(v)
    fl = os.path.join(folder, f"flightlog_shot{n}.json")
    return v, (fl if os.path.exists(fl) else None), n


def load_frames(path: str):
    from .video.quality import frame_quality
    from .video.replay import FileSource
    src = FileSource(path)
    frames = []
    try:
        while True:
            f = src.read()
            if f is None:
                break
            if f.quality is None:
                q = frame_quality(f.img)
                f.quality, f.bad = q["score"], bool(q["bad"])
            frames.append(f)
    finally:
        src.close()
    return frames


def timeline(folder: str):
    p = os.path.join(folder, "mission.jsonl")
    if not os.path.exists(p):
        return []
    out = []
    t0 = None
    for r in jlog.read(p):
        if r.get("kind") in ("step", "prompt", "answer", "error", "warn", "result"):
            t0 = t0 or r["t"]
            out.append(f"{r['t'] - t0:7.1f}s {r['kind']:6} {r.get('name') or r.get('msg') or r.get('text') or r.get('choice')}")
    return out


def replay_run(folder: str, out: Optional[str] = None, shot: Optional[int] = None) -> dict:
    vid, flp, n = find_shot(folder, shot)
    frames = load_frames(vid)
    fl = None
    if flp:
        with open(flp) as f:
            fl = json.load(f)
    out = out or os.path.join(folder, f"replay_shot{n}")
    from .learn import dart_lens_from_settings, latency_setting, shot_pitch
    lens = dart_lens_from_settings((frames[0].img.shape[1], frames[0].img.shape[0])) if frames else None
    res = run_post_flight(frames, fl, out, dart_lens=lens, video_latency_s=latency_setting(),
                          aim_pitch_deg=shot_pitch(folder),
                          clock_offset_s=(fl or {}).get("clock_offset_s") if fl else None)
    res["timeline"] = timeline(folder)
    for line in res["timeline"]:
        print(line)
    return res
