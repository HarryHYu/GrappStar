#!/usr/bin/env python3
"""GrappStar environment check. Prints what is installed and what is plugged in.

    python3 setup/check_env.py

Uses only the standard library; numpy, cv2, serial and onnxruntime are optional imports.
Safe on a PC with none of the hardware: it reports and never crashes. Exit code is 0
unless a required Python package (numpy, cv2, serial) is missing, or numpy is 2.x on a Jetson.
"""
import glob
import os
import platform
import shutil
import subprocess
import sys

PROBLEMS = []
ON_JETSON = os.path.exists("/etc/nv_tegra_release")

try:
    import grp
except ImportError:  # not on Linux or macOS
    grp = None


def line(label, value, note=""):
    print("  %-22s %s%s" % (label, value, ("   " + note) if note else ""))


def warn(msg):
    PROBLEMS.append(msg)
    print("  WARNING: " + msg)


def section(title):
    print("\n" + title)


def read_text(path):
    try:
        with open(path, "rb") as f:
            return f.read().decode("utf-8", "replace").strip("\x00\n ")
    except OSError:
        return None


def run(cmd, timeout=5):
    """Run a command, return its stdout or None if it is missing or fails."""
    if not shutil.which(cmd[0]):
        return None
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError):
        return None
    return r.stdout if r.returncode == 0 else None


def check_machine():
    section("Machine")
    model = read_text("/proc/device-tree/model")
    line("model", model or platform.machine())
    tegra = read_text("/etc/nv_tegra_release")
    line("JetPack (L4T)", tegra.splitlines()[0] if tegra else "not a Jetson")
    line("hostname", platform.node())
    uid = os.getuid() if hasattr(os, "getuid") else -1
    line("user", "%s (uid %d)" % (os.environ.get("USER") or os.environ.get("USERNAME") or "?", uid))
    try:
        names = [grp.getgrgid(g).gr_name for g in os.getgroups()] if grp else []
    except (KeyError, OSError, AttributeError):
        names = []
    for g in ("dialout", "video"):
        if g in names or uid == 0:
            line("group " + g, "yes")
        else:
            line("group " + g, "no", "(needed for the base serial port / cameras; run setup/install.sh, then log out and in)")


def check_python():
    section("Python packages")
    v = sys.version_info
    note = "" if (v.major, v.minor) == (3, 10) else "(the Jetson has 3.10; other versions are fine for PC tests)"
    line("python", "%d.%d.%d" % (v.major, v.minor, v.micro), note)

    try:
        import numpy
        nv = numpy.__version__
        line("numpy", nv)
        if int(nv.split(".")[0]) >= 2 and ON_JETSON:
            warn("numpy %s is 2.x. It breaks JetPack system packages. Fix: python3 -m pip install --user 'numpy>=1.21,<2'" % nv)
        elif int(nv.split(".")[0]) >= 2:
            print("  note: numpy is 2.x. Fine on a PC, but the Jetson must stay below 2.")
    except Exception as e:  # noqa: BLE001 (any import failure is just a report)
        line("numpy", "MISSING")
        warn("numpy not importable (%s). Install: sudo apt install python3-numpy" % e.__class__.__name__)

    try:
        import cv2
        line("opencv (cv2)", cv2.__version__, "from " + os.path.dirname(getattr(cv2, "__file__", "") or "?"))
        info = cv2.getBuildInformation()
        for key, label in (("GStreamer", "  GStreamer"), ("v4l/v4l2", "  V4L2"), ("FFMPEG", "  FFMPEG")):
            line(label, build_flag(info, key))
        if build_flag(info, "v4l/v4l2") != "YES":
            warn("OpenCV has no V4L2 support: cameras may not open. On the Jetson use apt OpenCV, not pip opencv-python.")
    except Exception as e:  # noqa: BLE001
        line("opencv (cv2)", "MISSING")
        warn("cv2 not importable (%s). Jetson: sudo apt install python3-opencv. PC: pip install opencv-python-headless" % e.__class__.__name__)

    try:
        import serial
        line("pyserial", getattr(serial, "VERSION", getattr(serial, "__version__", "?")))
    except Exception as e:  # noqa: BLE001
        line("pyserial", "MISSING")
        warn("pyserial not importable (%s). Install: sudo apt install python3-serial" % e.__class__.__name__)

    try:
        import onnxruntime as ort
        line("onnxruntime", ort.__version__, "providers: " + ", ".join(ort.get_available_providers()))
    except Exception:  # noqa: BLE001
        line("onnxruntime", "not installed", "(optional)")

    try:
        import pytest
        line("pytest", pytest.__version__)
    except Exception:  # noqa: BLE001
        line("pytest", "not installed", "(needed only to run the tests)")

    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if os.path.isdir(os.path.join(here, "grappstar")):
        line("grappstar package", "found in " + here)
    else:
        line("grappstar package", "not found next to setup/")


def build_flag(info, key):
    """Find 'key: YES (...)' in cv2.getBuildInformation() and return YES/NO/unknown."""
    for raw in info.splitlines():
        s = raw.strip()
        if s.startswith(key + ":"):
            rest = s[len(key) + 1:].strip()
            return "YES" if rest.upper().startswith("YES") else ("NO" if rest.upper().startswith("NO") else rest[:30])
    return "unknown"


def check_serial():
    section("Serial devices (base station)")
    named = sorted(glob.glob("/dev/grappstar-base*"))
    for p in named:
        line(p, "-> " + os.path.realpath(p), access_note(p))
    if not named:
        line("/dev/grappstar-base", "not present", "(fine with no base plugged in)")
    rules = read_text("/etc/udev/rules.d/99-grappstar.rules")
    if rules is None:
        line("udev rules", "not installed", "(run bash setup/install.sh)")
    elif 'ATTRS{serial}=="REPLACE_WITH_BASE_SERIAL"' in rules:
        line("base serial", "not set", "(README step 4: only the base plugged in, then bash setup/install.sh --no-apt)")
    else:
        line("base serial", "set in the udev rule")
    others = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    for p in others:
        line(p, usb_name(p), access_note(p))
    if not others:
        line("ttyACM/ttyUSB", "none")


def check_video():
    section("Video devices (cameras)")
    for p in sorted(glob.glob("/dev/grappstar-*cam")):
        line(p, "-> " + os.path.realpath(p), access_note(p))
    vids = sorted(glob.glob("/dev/video*"), key=lambda s: (len(s), s))
    if not vids:
        line("/dev/video*", "none", "(fine with no cameras plugged in)")
    for p in vids:
        n = os.path.basename(p)
        name = read_text("/sys/class/video4linux/%s/name" % n) or "?"
        idx = read_text("/sys/class/video4linux/%s/index" % n)
        line(p, name, "index %s %s" % (idx if idx is not None else "?", access_note(p)))
    if vids and shutil.which("v4l2-ctl") is None:
        line("v4l2-ctl", "missing", "(sudo apt install v4l-utils)")


def usb_name(dev):
    n = os.path.basename(dev)
    base = "/sys/class/tty/%s/device" % n
    for up in ("..", "../.."):
        d = os.path.realpath(os.path.join(base, up))
        vid = read_text(os.path.join(d, "idVendor"))
        if vid:
            pid = read_text(os.path.join(d, "idProduct")) or "?"
            prod = read_text(os.path.join(d, "product")) or ""
            return "%s:%s %s" % (vid, pid, prod)
    return "?"


def access_note(p):
    return "" if os.access(p, os.R_OK | os.W_OK) else "(NO ACCESS: check groups)"


def check_network():
    section("Network")
    out = run(["nmcli", "-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device"])
    if out is None:
        line("nmcli", "not available", "(NetworkManager not installed or not running)")
    else:
        for row in out.splitlines():
            parts = row.split(":")
            if len(parts) >= 4 and parts[1] in ("wifi", "ethernet"):
                line(parts[0], "%s %s" % (parts[1], parts[2]), parts[3])
        wifi = run(["nmcli", "-t", "-f", "ACTIVE,SSID,SIGNAL", "device", "wifi", "list", "--rescan", "no"])
        if wifi:
            seen = [r.split(":") for r in wifi.splitlines() if r]
            kedge = [r for r in seen if len(r) >= 3 and r[1] == "KEDGE"]
            if kedge:
                line("dart AP KEDGE", "visible, signal %s%%" % kedge[0][2], "(joined)" if kedge[0][0] == "yes" else "(not joined)")
            else:
                line("dart AP KEDGE", "not seen", "(fine if the dart is off)")
    addrs = run(["ip", "-4", "-o", "addr", "show"])
    if addrs:
        for row in addrs.splitlines():
            f = row.split()
            if len(f) >= 4 and f[1] != "lo":
                line("address " + f[1], f[3].split("/")[0], "(page: http://%s:8080)" % f[3].split("/")[0])


def main():
    print("GrappStar environment check")
    for fn in (check_machine, check_python, check_serial, check_video, check_network):
        try:
            fn()
        except Exception as e:  # noqa: BLE001 (keep going no matter what)
            print("  (check %s failed: %s)" % (fn.__name__, e))
    section("Summary")
    if PROBLEMS:
        for p in PROBLEMS:
            print("  - " + p)
        return 1
    print("  No problems found.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
