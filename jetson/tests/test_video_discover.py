"""video.discover parsing with sample v4l2-ctl output."""
from grappstar.video import discover
from grappstar.video.discover import (parse_list_formats_ext, parse_list_devices, guess_role,
                                      pick_mode)

NEW_FORMAT = """ioctl: VIDIOC_ENUM_FMT
\tType: Video Capture

\t[0]: 'YUYV' (YUYV 4:2:2)
\t\tSize: Discrete 720x480
\t\t\tInterval: Discrete 0.033s (30.000 fps)
\t\tSize: Discrete 720x576
\t\t\tInterval: Discrete 0.040s (25.000 fps)
\t[1]: 'MJPG' (Motion-JPEG, compressed)
\t\tSize: Discrete 1920x1080
\t\t\tInterval: Discrete 0.033s (30.000 fps)
\t\t\tInterval: Discrete 0.017s (60.000 fps)
\t\tSize: Discrete 1280x720
\t\t\tInterval: Discrete 0.017s (60.000 fps)
\t\tSize: Discrete 640x480
\t\t\tInterval: Discrete 0.008s (120.000 fps)
"""

OLD_FORMAT = """ioctl: VIDIOC_ENUM_FMT
\tIndex       : 0
\tType        : Video Capture
\tPixel Format: 'YUYV'
\tName        : YUYV 4:2:2
\t\tSize: Discrete 1920x1080
\t\t\tInterval: Discrete 0.200s (5.000 fps)
\t\tSize: Discrete 640x480
\t\t\tInterval: Discrete 0.033s (30.000 fps)

\tIndex       : 1
\tType        : Video Capture
\tPixel Format: 'MJPG' (compressed)
\tName        : Motion-JPEG
\t\tSize: Stepwise 16x16 - 1920x1080 with step 1/1
"""

METADATA_NODE = """ioctl: VIDIOC_ENUM_FMT
\tType: Video Capture

"""

LIST_DEVICES = """NVIDIA Tegra Video Input Device (platform:tegra-camrtc-ca):
\t/dev/media0

Razer Kiyo X (usb-3610000.xhci-2.1):
\t/dev/video0
\t/dev/video1
\t/dev/media1

USB Video: USB Video (usb-3610000.xhci-2.3):
\t/dev/video2
\t/dev/video3
\t/dev/media2
"""


def test_parse_new_layout():
    f = parse_list_formats_ext(NEW_FORMAT)
    assert [x["fourcc"] for x in f] == ["YUYV", "MJPG"]
    assert f[0]["desc"] == "YUYV 4:2:2"
    assert f[0]["sizes"][0] == {"w": 720, "h": 480, "fps": [30.0]}
    assert f[1]["sizes"][0]["fps"] == [30.0, 60.0]
    assert len(f[1]["sizes"]) == 3


def test_parse_old_layout():
    f = parse_list_formats_ext(OLD_FORMAT)
    assert [x["fourcc"] for x in f] == ["YUYV", "MJPG"]
    assert f[0]["desc"] == "YUYV 4:2:2"
    assert f[1]["desc"] == "compressed"
    assert f[0]["sizes"][1] == {"w": 640, "h": 480, "fps": [30.0]}
    assert f[1]["sizes"][0]["w"] == 1920 and f[1]["sizes"][0]["stepwise"]


def test_parse_empty():
    assert parse_list_formats_ext(METADATA_NODE) == []
    assert parse_list_formats_ext("") == []


def test_parse_list_devices():
    d = parse_list_devices(LIST_DEVICES)
    assert d["/dev/video0"].startswith("Razer Kiyo X")
    assert d["/dev/video3"].startswith("USB Video")
    assert "/dev/media0" not in d


def test_guess_role():
    assert guess_role("Razer Kiyo X") == "rover"
    assert guess_role(None, "/dev/v4l/by-id/usb-Razer_Inc_Razer_Kiyo_X-video-index0") == "rover"
    assert guess_role("USB Video: USB Video") == "dart"
    assert guess_role("/dev/v4l/by-id/usb-MACROSILICON_AV_TO_USB2.0-video-index0") == "dart"
    assert guess_role("DIGITNOW Video Capture") == "dart"
    assert guess_role("UVC Camera (534d:2109)") == "dart"
    assert guess_role("Razer Kiyo X UVC") == "rover"     # rover wins over the generic UVC
    assert guess_role("Logitech C920") is None


def test_pick_mode():
    f = parse_list_formats_ext(NEW_FORMAT)
    assert pick_mode(f) == {"fourcc": "MJPG", "w": 1920, "h": 1080, "fps": 60.0}
    assert pick_mode(f, max_width=1280)["w"] == 1280
    assert pick_mode(f, prefer=("YUYV",)) == {"fourcc": "YUYV", "w": 720, "h": 576, "fps": 25.0}
    assert pick_mode(f, prefer=("H264",)) is None


def test_find_cameras_tolerates_missing_tools(monkeypatch):
    # Pretend there is one device, no sysfs name and no v4l2-ctl.
    monkeypatch.setattr(discover.glob, "glob",
                        lambda p: ["/dev/video0", "/dev/video10"] if p == "/dev/video*" else [])
    monkeypatch.setattr(discover, "_run", lambda cmd, timeout=3.0: None)
    monkeypatch.setattr(discover, "_sysfs_name", lambda i: "Razer Kiyo X" if i == 0 else None)
    cams = discover.find_cameras()
    assert [c["dev"] for c in cams] == ["/dev/video0", "/dev/video10"]
    assert cams[0]["role"] == "rover" and cams[0]["formats"] == [] and cams[0]["capture"]
    assert cams[1]["name"] == "/dev/video10" and cams[1]["role"] is None


def test_find_cameras_with_v4l2ctl(monkeypatch):
    monkeypatch.setattr(discover.glob, "glob",
                        lambda p: ["/dev/video2", "/dev/video3"] if p == "/dev/video*" else [])
    monkeypatch.setattr(discover, "_sysfs_name", lambda i: None)

    def fake_run(cmd, timeout=3.0):
        if "--list-devices" in cmd:
            return LIST_DEVICES
        return NEW_FORMAT if cmd[-1] == "/dev/video2" else METADATA_NODE
    monkeypatch.setattr(discover, "_run", fake_run)
    cams = discover.find_cameras()
    assert cams[0]["role"] == "dart" and cams[0]["capture"]
    assert not cams[1]["capture"]                 # metadata node
    assert discover.find_role("dart", cams)["dev"] == "/dev/video2"
    assert discover.find_role("rover", cams) is None


def test_find_cameras_runs_on_this_machine():
    assert isinstance(discover.find_cameras(), list)
