"""Find the USB cameras and guess which one is which.

find_cameras() -> list of dicts, one per /dev/videoN node:
    dev      "/dev/video0"
    index    0
    by_id    "/dev/v4l/by-id/usb-Razer_Kiyo_X-video-index0" or None
    name     driver card name (from sysfs, or the by-id link)
    formats  [{"fourcc": "MJPG", "desc": "...", "sizes": [{"w", "h", "fps": [...]}]}]
             (empty if v4l2-ctl is missing or the node is a metadata node)
    capture  True if the node lists any capture formats (or formats are unknown)
    role     "rover" (Razer Kiyo), "dart" (MS2109 capture dongle) or None

The pure parsing functions (parse_list_formats_ext, parse_list_devices, guess_role)
do no I/O, so they are unit tested with sample text.
"""
import glob
import os
import re
import shutil
import subprocess
from typing import Optional

ROVER_KEYS = ("kiyo", "razer")
DART_KEYS = ("digitnow", "usb video", "usb_video", "av to usb", "av_to_usb", "macrosilicon",
             "ms2109", "uvc")


def guess_role(*names) -> Optional[str]:
    """Guess a camera's role from any of its names. Rover keywords win over dart ones
    because "UVC" is generic and could appear in the Kiyo's description too."""
    text = " ".join(n for n in names if n).lower()
    if any(k in text for k in ROVER_KEYS):
        return "rover"
    if any(k in text for k in DART_KEYS):
        return "dart"
    return None


_FMT_NEW = re.compile(r"\[\d+\]:\s*'(\w+)'\s*\((.*)\)")          # [0]: 'YUYV' (YUYV 4:2:2)
_FMT_OLD = re.compile(r"Pixel Format\s*:\s*'(\w+)'(?:\s*\((.*)\))?")
_NAME_OLD = re.compile(r"^\s*Name\s*:\s*(.*)$")
_SIZE = re.compile(r"Size:\s*Discrete\s+(\d+)x(\d+)")
_STEP = re.compile(r"Size:\s*Stepwise\s+(\d+)x(\d+)\s*-\s*(\d+)x(\d+)")
_FPS = re.compile(r"Interval:.*?\(([\d.]+)\s*fps\)")


def parse_list_formats_ext(text: str) -> list:
    """Parse `v4l2-ctl --list-formats-ext` output (old and new layouts)."""
    formats = []
    fmt = None
    size = None
    for line in text.splitlines():
        m = _FMT_NEW.search(line) or _FMT_OLD.search(line)
        if m:
            fmt = {"fourcc": m.group(1), "desc": (m.group(2) or "").strip(), "sizes": []}
            formats.append(fmt)
            size = None
            continue
        if fmt is None:
            continue
        m = _NAME_OLD.match(line)
        if m and not fmt["desc"]:
            fmt["desc"] = m.group(1).strip()
            continue
        m = _SIZE.search(line)
        if m:
            size = {"w": int(m.group(1)), "h": int(m.group(2)), "fps": []}
            fmt["sizes"].append(size)
            continue
        m = _STEP.search(line)
        if m:
            # Stepwise range: record the maximum size, fps unknown.
            size = {"w": int(m.group(3)), "h": int(m.group(4)), "fps": [], "stepwise": True}
            fmt["sizes"].append(size)
            continue
        m = _FPS.search(line)
        if m and size is not None:
            size["fps"].append(float(m.group(1)))
    return formats


def parse_list_devices(text: str) -> dict:
    """Parse `v4l2-ctl --list-devices` into {"/dev/video0": "Card name (usb-...)"}."""
    out = {}
    card = None
    for line in text.splitlines():
        if not line.strip():
            continue
        if not line[0].isspace():
            card = line.strip().rstrip(":")
        elif card is not None:
            dev = line.strip()
            if dev.startswith("/dev/video"):
                out[dev] = card
    return out


def pick_mode(formats: list, prefer=("MJPG", "YUYV"), max_width: Optional[int] = None,
              min_fps: float = 20.0) -> Optional[dict]:
    """Choose a capture mode: preferred fourcc first, then the largest size that still
    runs at min_fps (and is no wider than max_width). Returns {"fourcc","w","h","fps"}."""
    by_cc = {f["fourcc"]: f for f in formats}
    for cc in prefer:
        f = by_cc.get(cc)
        if not f:
            continue
        best = None
        for s in f["sizes"]:
            if max_width and s["w"] > max_width:
                continue
            fps = max(s["fps"]) if s["fps"] else 0.0
            if s["fps"] and fps < min_fps:
                continue
            key = (s["w"] * s["h"], fps)
            if best is None or key > best[0]:
                best = (key, {"fourcc": cc, "w": s["w"], "h": s["h"], "fps": fps or None})
        if best:
            return best[1]
    return None


def _run(cmd: list, timeout: float = 3.0) -> Optional[str]:
    """Run a command and return stdout, or None if it is missing or fails."""
    if shutil.which(cmd[0]) is None:
        return None
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError):
        return None
    return r.stdout if r.returncode == 0 else None


def _sysfs_name(index: int) -> Optional[str]:
    try:
        with open("/sys/class/video4linux/video%d/name" % index) as fh:
            return fh.read().strip()
    except OSError:
        return None


def find_cameras(use_v4l2ctl: bool = True) -> list:
    devs = glob.glob("/dev/video*")
    devs = sorted((d for d in devs if re.fullmatch(r"/dev/video\d+", d)),
                  key=lambda d: int(d[len("/dev/video"):]))
    # by-id links point at ../../videoN; map the real node back to its stable name.
    by_id = {}
    for link in sorted(glob.glob("/dev/v4l/by-id/*")):
        by_id.setdefault(os.path.realpath(link), link)
    cards = {}
    if use_v4l2ctl:
        text = _run(["v4l2-ctl", "--list-devices"])
        if text:
            cards = parse_list_devices(text)
    cams = []
    for dev in devs:
        index = int(dev[len("/dev/video"):])
        link = by_id.get(os.path.realpath(dev))
        name = _sysfs_name(index) or cards.get(dev) or (os.path.basename(link) if link else dev)
        formats = []
        known = False
        if use_v4l2ctl:
            text = _run(["v4l2-ctl", "--list-formats-ext", "-d", dev])
            if text is not None:
                formats = parse_list_formats_ext(text)
                known = True
        cams.append({
            "dev": dev,
            "index": index,
            "by_id": link,
            "name": name,
            "formats": formats,
            "capture": bool(formats) or not known,
            "role": guess_role(name, link, cards.get(dev)),
        })
    return cams


def find_role(role: str, cams: Optional[list] = None) -> Optional[dict]:
    """First capture-capable camera with the given role, or None."""
    for c in (cams if cams is not None else find_cameras()):
        if c["role"] == role and c["capture"]:
            return c
    return None
