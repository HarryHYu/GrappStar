import json
import math

import cv2
import numpy as np
import pytest

from grappstar.vision.calibrate import (calibrate_intrinsics, camera_from, default_camera,
                                        kiyo_default_intrinsics, main as calib_main,
                                        mounting_from_ground_board, mounting_from_marker)
from grappstar.vision.target import CameraModel, detect_orange, target_info, turn_advice

W, H = 1280, 720
ORANGE = (0, 110, 255)      # BGR safety orange, OpenCV hue about 13


def _cam(pitch=8.0, height=0.5, roll=0.0, yaw=0.0, dist=None):
    return CameraModel(fx=900.0, fy=900.0, cx=639.5, cy=359.5, height_m=height, pitch_deg=pitch,
                       roll_deg=roll, dist=dist, calib_size=(W, H), yaw_deg=yaw)


def render_ground(cam, markers=(), seed=0):
    """Render a textured flat ground plane plus standing orange markers (range, bearing)."""
    vs, us = np.mgrid[0:H, 0:W].astype(np.float64)
    x = (us - cam.cx) / cam.fx
    y = (vs - cam.cy) / cam.fy
    R = cam.rotation()
    d = R @ np.stack([x.ravel(), y.ravel(), np.ones(x.size)])
    dz = d[2].reshape(H, W)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(dz < -1e-9, cam.height_m / -dz, np.nan)
    X, Y = t * d[0].reshape(H, W), t * d[1].reshape(H, W)
    rng = np.random.default_rng(seed)
    ph = rng.uniform(0, 6, 4)
    g = 110 + 25 * np.sin(X * 3.1 + ph[0]) * np.sin(Y * 2.3 + ph[1]) + 15 * np.sin(X * 11 + Y * 7 + ph[2])
    img = np.zeros((H, W, 3), np.float64)
    img[..., 0] = g * 0.75
    img[..., 1] = g * 0.9
    img[..., 2] = g
    sky = np.isnan(t)
    img[sky] = (230, 200, 170)
    img = np.clip(np.nan_to_num(img), 0, 255).astype(np.uint8)
    for rng_m, brg in markers:
        b = math.radians(brg)
        P = np.array([rng_m * math.sin(b), rng_m * math.cos(b)])
        lat = np.array([math.cos(b), -math.sin(b)]) * 0.06
        corners = []
        for side, z in ((-1, 0.0), (1, 0.0), (1, 0.5), (-1, 0.5)):
            q = P + side * lat
            corners.append(cam.world_to_pixel(q[0], q[1], z))
        poly = np.round(np.array(corners) * 16).astype(np.int32)
        cv2.fillConvexPoly(img, poly, ORANGE, lineType=cv2.LINE_8, shift=4)
    img = img.astype(np.float32) + rng.normal(0, 3, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def test_geometry_round_trip_and_horizon():
    for cam in (_cam(), _cam(pitch=20, roll=3, yaw=-2), _cam(dist=[-0.05, 0.01, 0, 0, 0])):
        for r, b in ((2.0, 0.0), (5.0, 10.0), (8.0, -15.0)):
            uv = cam.ground_to_pixel(r, b)
            assert uv is not None
            rr, bb = cam.pixel_to_ground(*uv)
            assert abs(rr - r) < 1e-3 * r and abs(bb - b) < 1e-3
    cam = _cam(pitch=8)
    assert cam.pixel_to_ground(640, 0) is None             # above the horizon
    hv = cam.horizon_v()
    assert abs(hv - (359.5 - 900 * math.tan(math.radians(8)))) < 1e-6
    r, b = cam.pixel_to_ground(cam.cx, cam.cy)             # optical axis hits the ground
    assert abs(r - 0.5 / math.tan(math.radians(8))) < 1e-6 and abs(b) < 1e-9
    # A point right of centre has positive bearing.
    assert cam.pixel_to_ground(900, 600)[1] > 0


def test_scaled_intrinsics():
    cam = _cam(pitch=12)
    full = cam.pixel_to_ground(800, 500)
    half = cam.pixel_to_ground((800 + 0.5) / 2 - 0.5, (500 + 0.5) / 2 - 0.5, image_size=(640, 360))
    assert abs(full[0] - half[0]) < 1e-6 and abs(full[1] - half[1]) < 1e-6
    s = cam.for_size(1920, 1080)
    assert abs(s.fx - 1350) < 1e-9 and s.calib_size == (1920, 1080)


def test_save_load(tmp_path):
    cam = _cam(pitch=11, roll=1.5, dist=[0.01, 0, 0, 0, 0])
    p = tmp_path / "cam.json"
    cam.save(str(p))
    cam2 = CameraModel.load(str(p))
    assert cam2 == cam
    json.loads(p.read_text())


@pytest.mark.parametrize("rng_m,brg", [(3.0, 0.0), (6.0, 12.0), (10.0, -8.0), (4.5, -20.0)])
def test_orange_marker_range_bearing(rng_m, brg):
    cam = _cam(pitch=8.0, height=0.5)
    img = render_ground(cam, [(rng_m, brg)])
    det = detect_orange(img)
    assert det is not None
    u, v, area = det
    r, b = cam.pixel_to_ground(u, v)
    assert abs(r - rng_m) / rng_m < 0.05, (r, rng_m)
    assert abs(b - brg) < 1.0, (b, brg)
    unc = cam.range_uncertainty(u, v)
    assert unc["lo"] <= r <= unc["hi"] and unc["plus_m"] >= 0 and unc["minus_m"] >= 0
    info = target_info(cam, u, v)
    assert info["ok"] and json.dumps(info)


def test_detect_orange_none_and_small():
    img = np.full((240, 320, 3), 120, np.uint8)
    assert detect_orange(img) is None
    cv2.circle(img, (100, 100), 2, ORANGE, -1)
    assert detect_orange(img) is None                     # too small
    cv2.rectangle(img, (200, 50), (220, 120), ORANGE, -1)
    u, v, a = detect_orange(img)
    assert abs(u - 210) < 1 and abs(v - 120.5) < 1 and a > 1000


def test_turn_advice():
    assert turn_advice(12.3) == "Turn the rover 12 degrees right"
    assert turn_advice(-1.2) == "Turn the rover 1 degree left"
    assert turn_advice(0.4) == "Target is straight ahead"


def test_range_uncertainty_grows_with_range():
    cam = _cam(pitch=8)
    near = cam.range_uncertainty(*cam.ground_to_pixel(3, 0))
    far = cam.range_uncertainty(*cam.ground_to_pixel(12, 0))
    assert (far["hi"] - far["lo"]) / 12 > (near["hi"] - near["lo"]) / 3


def test_default_kiyo_intrinsics():
    d = kiyo_default_intrinsics()
    assert d["uncalibrated"] and "NOT calibrated" in d["warning"]
    diag = math.hypot(1280, 720)
    fov = 2 * math.degrees(math.atan(diag / 2 / d["fx"]))
    assert abs(fov - 82.0) < 1e-6 and 800 < d["fx"] < 900
    cam = default_camera(0.4, 10)
    assert cam.uncalibrated and cam.notes
    assert target_info(cam, 640, 500)["uncalibrated"]


# ------------------------------------------------------------------ checkerboard rendering

def board_texture(pattern=(9, 6), px=40, margin=1):
    cols, rows = pattern[0] + 1, pattern[1] + 1
    tex = np.full(((rows + 2 * margin) * px, (cols + 2 * margin) * px), 255, np.uint8)
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                y, x = (r + margin) * px, (c + margin) * px
                tex[y:y + px, x:x + px] = 0
    return tex


def render_board(K, rvec_board_to_cam, t, square_m, size=(W, H), pattern=(9, 6), px=40, seed=0):
    """Render the board (texture px -> metres -> camera) with homography K [r1 r2 t] A."""
    R, _ = cv2.Rodrigues(np.asarray(rvec_board_to_cam, np.float64))
    A = np.array([[square_m / px, 0, -square_m], [0, square_m / px, -square_m], [0, 0, 1]])
    Hm = K @ np.column_stack([R[:, 0], R[:, 1], np.asarray(t, np.float64)]) @ A
    tex = board_texture(pattern, px)
    img = cv2.warpPerspective(tex, Hm, size, flags=cv2.INTER_LINEAR, borderValue=150)
    img = cv2.GaussianBlur(img, (3, 3), 0.6).astype(np.float32)
    img += np.random.default_rng(seed).normal(0, 2, img.shape)
    return cv2.cvtColor(np.clip(img, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)


def test_checkerboard_intrinsics_round_trip(tmp_path):
    K = np.array([[920.0, 0, 652.0], [0, 915.0, 351.0], [0, 0, 1]])
    rng = np.random.default_rng(1)
    imgs = []
    for i in range(12):
        rv = np.array([rng.uniform(-0.45, 0.45), rng.uniform(-0.45, 0.45), rng.uniform(-0.2, 0.2)])
        t = np.array([rng.uniform(-0.15, 0.05), rng.uniform(-0.1, 0.0), rng.uniform(0.55, 0.85)])
        imgs.append(render_board(K, rv, t, 0.025, seed=i))
    res = calibrate_intrinsics(imgs)
    assert res["n_used"] >= 8
    for k, ref in (("fx", 920), ("fy", 915), ("cx", 652), ("cy", 351)):
        assert abs(res[k] - ref) / ref < 0.03, (k, res[k])
    # CLI writes JSON
    for i, im in enumerate(imgs[:6]):
        cv2.imwrite(str(tmp_path / ("c%02d.png" % i)), im)
    out = tmp_path / "intr.json"
    assert calib_main(["intrinsics", "--images", str(tmp_path / "c*.png"), "--out", str(out)]) == 0
    assert json.loads(out.read_text())["n_used"] >= 3


def _ground_board_image(cam, origin_xy, square_m, pattern=(9, 6)):
    """Board lying flat on the ground; board x along world X, board y along world Y."""
    R = cam.rotation()
    C = np.array([0.0, 0.0, cam.height_m])
    O = np.array([origin_xy[0], origin_xy[1], 0.0])
    Rc = R.T
    M = np.column_stack([Rc @ np.array([1.0, 0, 0]), Rc @ np.array([0, 1.0, 0]), Rc @ (O - C)])
    rvec, _ = cv2.Rodrigues(np.column_stack([M[:, 0], M[:, 1], np.cross(M[:, 0], M[:, 1])]))
    return render_board(cam.K(), rvec.ravel(), M[:, 2], square_m, pattern=pattern)


@pytest.mark.parametrize("height,pitch,roll", [(0.35, 25.0, 0.0), (0.5, 15.0, 2.0)])
def test_ground_checkerboard_mounting(height, pitch, roll):
    cam = _cam(pitch=pitch, height=height, roll=roll)
    sq = 0.05
    img = _ground_board_image(cam, (-0.25, 0.6 if pitch > 20 else 1.0), sq)
    intr = {"fx": cam.fx, "fy": cam.fy, "cx": cam.cx, "cy": cam.cy, "dist": None,
            "calib_size": [W, H]}
    m = mounting_from_ground_board(img, intr, square_m=sq)
    assert abs(m["height_m"] - height) < 0.01, m
    assert abs(m["pitch_deg"] - pitch) < 0.5, m
    assert abs(m["roll_deg"] - roll) < 0.5, m
    rebuilt = camera_from(intr, m)
    r, b = rebuilt.pixel_to_ground(640, 600)
    r0, b0 = cam.pixel_to_ground(640, 600)
    assert abs(r - r0) / r0 < 0.03


def test_marker_pitch_calibration():
    truth = _cam(pitch=9.0, height=0.45)
    img = render_ground(truth, [(5.0, 0.0)])
    intr = {"fx": truth.fx, "fy": truth.fy, "cx": truth.cx, "cy": truth.cy, "calib_size": [W, H]}
    m = mounting_from_marker(img, 5.0, 0.45, intr)
    assert abs(m["pitch_deg"] - 9.0) < 0.3, m
    assert abs(m["yaw_deg"]) < 0.5
