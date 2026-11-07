"""Targeting from the rover camera at the REAL launcher's reach (2.6 to 4.8 m from the pivot, C3 table).

The camera pose comes from base/spec/params.py (FRAMEWORK.camera: 150 mm ahead of the pivot, 50 mm right of the
world origin, 0.19 m right of the launch line, 10 deg down) on a 0.15 m deck. An orange marker is placed at a known
range and bearing from the pivot ground point and the launch line, rendered by the Kiyo X model with noise and
JPEG compression, found by detect_orange and turned into range and bearing from the pivot (campose). Then the aim
solver picks the pitch, and the table range at that pitch is compared with the true distance: that is the miss the
operator would see from targeting alone.
"""
import math

import cv2
import numpy as np
import pytest

from grappstar.ballistics import Ballistics, REPO_TABLE
from grappstar.campose import camera_model, target_info_pivot
from grappstar.vision.target import detect_orange
from test_target import render_ground

W, H = 1280, 720


def _marker_cam_coords(m, r_pivot, b_pivot):
    """(range, bearing) in the camera ground frame of a point r_pivot, b_pivot (deg, + right) from the pivot."""
    x_right = r_pivot * math.sin(math.radians(b_pivot))
    y_fwd = r_pivot * math.cos(math.radians(b_pivot))
    xc = x_right + m.lateral_offset_m          # camera is -lateral to the right of the line
    yc = y_fwd - m.range_offset_m
    return math.hypot(xc, yc), math.degrees(math.atan2(xc, yc))


@pytest.mark.parametrize("r,b", [(2.8, 0.0), (3.5, 5.0), (4.2, -6.0), (4.75, 2.0)])
def test_rover_targeting_at_real_reach(r, b):
    m = camera_model(None, (W, H), deck_height_m=0.15)
    assert m.lateral_offset_m == pytest.approx(-0.19, abs=1e-6) and m.range_offset_m == pytest.approx(0.15)
    rc, bc = _marker_cam_coords(m, r, b)
    img = render_ground(m, [(rc, bc)], seed=int(r * 10))
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])     # the Kiyo X MJPG stream
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    det = detect_orange(img)
    assert det is not None
    info = target_info_pivot(m, det[0], det[1], image_size=(W, H))
    assert abs(info["range_m"] - r) / r < 0.04, (info, r)
    assert abs(info["bearing_deg"] - b) < 1.0, (info, b)
    # aim from that estimate: the table range at the chosen pitch against the true distance
    bal = Ballistics.load(REPO_TABLE)
    s = bal.solve(info["range_m"])
    if r <= s.range_max_m:
        assert s.reachable
        landed = bal.at_pitch(s.pitch_deg)["range_m"]
        assert abs(landed - r) / r < 0.05, (landed, r, s)


def test_deck_height_error_effect_is_bounded():
    """2 cm of deck height error (Harry measures it with a tape) moves a 4 m target by under 6 percent."""
    m_true = camera_model(None, (W, H), deck_height_m=0.15)
    rc, bc = _marker_cam_coords(m_true, 4.0, 0.0)
    img = render_ground(m_true, [(rc, bc)])
    u, v, _ = detect_orange(img)
    for dh in (-0.02, 0.02):
        m = camera_model(None, (W, H), deck_height_m=0.15 + dh)
        info = target_info_pivot(m, u, v, image_size=(W, H))
        assert abs(info["range_m"] - 4.0) / 4.0 < 0.06, (dh, info["range_m"])
