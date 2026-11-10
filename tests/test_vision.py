import math

import cv2
import numpy as np
import pytest

from vision import (AprilTagPose, BrightBlobDetector, CameraModel, KalmanCV, LockInDetector, parse_yolo_output,
                    range_from_tag_pixels, refine_centroid, tag_pose)

FPS, F = 120.0, 13.0


def frames(n, beacon=(100, 60), blink=True, window=(30, 30), mains=(150, 90), shape=(120, 200), seed=0):
    rng = np.random.default_rng(seed)
    for i in range(n):
        t = i / FPS
        img = rng.integers(0, 4, size=shape).astype(np.float32)
        img[window[1] - 2:window[1] + 3, window[0] - 2:window[0] + 3] += 220            # steady lamp
        img[mains[1] - 2:mains[1] + 3, mains[0] - 2:mains[0] + 3] += 160 * (0.5 + 0.5 * math.cos(2 * math.pi * 100 * t))
        if beacon:
            b = 0.4 + 0.6 * (math.sin(2 * math.pi * F * t) >= 0) if blink else 1.0
            img[beacon[1] - 1:beacon[1] + 2, beacon[0] - 1:beacon[0] + 2] += 150 * b
        yield np.clip(img, 0, 255).astype(np.uint8), t


def run_lockin(**kw):
    d = LockInDetector(F, FPS, periods=4, min_amplitude=25.0)
    for f, t in frames(80, **kw):
        d.push(f, t)
    return d


class TestLockIn:
    def test_finds_the_13hz_beacon_and_ignores_steady_and_mains_lights(self):
        b = run_lockin().detect()
        assert b is not None and (b.x, b.y) == pytest.approx((100, 60), abs=0.5)

    def test_steady_light_alone_is_not_a_beacon(self):
        assert run_lockin(beacon=None).detect() is None

    def test_steady_bright_light_in_the_beacon_place_is_not_a_beacon(self):
        assert run_lockin(beacon=(100, 60), blink=False).detect() is None

    def test_not_ready_before_a_full_window(self):
        d = LockInDetector(F, FPS, periods=4)
        for f, t in frames(5):
            d.push(f, t)
        assert not d.ready and d.detect() is None

    def test_roi_restricts_the_search(self):
        d = run_lockin()
        assert d.detect((0, 0, 50, 50)) is None
        assert d.detect((80, 40, 120, 80)) is not None

    def test_amplitude_tracks_brightness(self):
        full, half = run_lockin(), LockInDetector(F, FPS, periods=4, min_amplitude=1.0)
        for f, t in frames(80):
            half.push((f.astype(np.float32) * 0.5).astype(np.uint8), t)
        assert half.detect().amplitude == pytest.approx(0.5 * full.detect().amplitude, rel=0.15)

    def test_reset(self):
        d = run_lockin()
        d.reset()
        assert not d.ready and d.detect() is None


def test_refine_centroid_is_subpixel():
    img = np.zeros((60, 60), np.uint8)
    img[30, 30] = 200
    img[30, 31] = 100
    cx, cy = refine_centroid(img, 29, 31)
    assert cx == pytest.approx(30 + 100 / 300, abs=0.05) and cy == pytest.approx(30, abs=0.05)


class TestKalman:
    def test_tracks_a_moving_point_and_estimates_velocity(self):
        kf = KalmanCV(accel_sigma=20.0, meas_sigma=0.3)
        rng = np.random.default_rng(0)
        kf.init(0, 0)
        for i in range(1, 200):
            t = i / 120
            kf.predict(1 / 120)
            kf.update(50 * t + rng.normal(0, 0.3), 20 * t + rng.normal(0, 0.3))
        assert kf.velocity == pytest.approx((50, 20), abs=6)

    def test_coasting_follows_the_prediction(self):
        kf = KalmanCV()
        kf.init(0, 0)
        for i in range(1, 120):
            kf.predict(1 / 120)
            kf.update(100 * i / 120, 0)
        x0 = kf.position[0]
        for _ in range(12):
            kf.predict(1 / 120)
        assert kf.position[0] == pytest.approx(x0 + 10, abs=3)


class TestPose:
    cam = CameraModel(1280, 720, 70.0)

    def project(self, tilt_deg, dist=3.0, size=0.09):
        h = size / 2
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)
        rvec = np.array([math.radians(tilt_deg), 0.0, 0.0]).reshape(3, 1)       # tilt about the x axis
        pts, _ = cv2.projectPoints(obj, rvec, np.array([0.0, 0.0, dist]).reshape(3, 1), self.cam.matrix, None)
        return pts.reshape(4, 2)

    @pytest.mark.parametrize("tilt", [0, 5, 20, 40])
    def test_lean_and_range_from_corners(self, tilt):
        p = tag_pose(self.project(tilt), 0.09, self.cam.matrix)
        assert p.lean_deg == pytest.approx(tilt, abs=1.0) and p.range_m == pytest.approx(3.0, rel=0.02)

    def test_range_from_tag_pixels(self):
        px = self.cam.focal_px * 0.09 / 3.0
        assert range_from_tag_pixels(self.cam.focal_px, 0.09, px) == pytest.approx(3.0)

    def test_apriltag_detector_end_to_end(self):
        marker = cv2.aruco.generateImageMarker(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 7, 200)
        frame = np.full((720, 1280), 255, np.uint8)
        side = int(self.cam.focal_px * 0.5 / 3.0)               # a 0.5 m tag at 3 m, seen frontally
        frame[300:300 + side, 500:500 + side] = cv2.resize(marker, (side, side), interpolation=cv2.INTER_NEAREST)
        pose = AprilTagPose(self.cam, tag_size_m=0.5)(frame)
        assert pose is not None and pose.range_m == pytest.approx(3.0, rel=0.1) and pose.lean_deg < 5

    def test_no_tag_no_pose(self):
        assert AprilTagPose(self.cam)(np.zeros((720, 1280), np.uint8)) is None


class TestSearchDetectors:
    def test_blob_detector_boxes_the_drone_not_a_small_light(self):
        img = np.zeros((100, 200), np.uint8)
        img[40:60, 100:140] = 90
        img[10:13, 10:13] = 255
        box = BrightBlobDetector()(img)
        assert box.cx == pytest.approx(119.5, abs=1) and box.cy == pytest.approx(49.5, abs=1)

    def test_blob_detector_none_in_empty_view(self):
        assert BrightBlobDetector()(np.zeros((50, 50), np.uint8)) is None

    def test_parse_yolov8_output(self):
        out = np.zeros((1, 5, 100), np.float32)          # [1, 4 + 1 class, N]
        out[0, :, 3] = [320, 160, 64, 32, 0.9]           # cx, cy, w, h, score in 640 px space
        out[0, :, 7] = [100, 100, 10, 10, 0.2]           # below threshold
        box = parse_yolo_output(out, 640, 1280, 720)
        assert (box.cx, box.cy, box.w, box.h) == pytest.approx((640, 180, 128, 36), abs=0.5)

    def test_parse_yolov5_output_and_nothing_found(self):
        out = np.zeros((1, 100, 6), np.float32)          # [1, N, 5 + 1 class]
        assert parse_yolo_output(out, 640, 640, 640) is None
        out[0, 5] = [100, 200, 20, 40, 0.9, 0.9]
        box = parse_yolo_output(out, 640, 640, 640)
        assert (box.cx, box.cy) == pytest.approx((100, 200), abs=0.5)

    def test_camera_model(self):
        c = CameraModel(1280, 720, 70.0)
        assert c.mrad_per_px == pytest.approx(0.95, abs=0.01)     # design doc: ~0.1 px -> ~0.1 mrad
        assert c.px_to_mrad(640, 360) == (0, 0)
