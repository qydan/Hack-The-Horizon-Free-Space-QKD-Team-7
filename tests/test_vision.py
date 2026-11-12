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
        assert c.mrad_per_px == pytest.approx(1.094, abs=0.005)   # 1 / focal_px at the centre
        assert c.px_to_mrad(640, 360) == (0, 0)

    def test_camera_model_wide_lens_is_pinhole(self):
        c = CameraModel(1920, 1080, 104.0)
        # the centre scale is 1/f, not hfov/width (which would be ~30% smaller at 104 deg)
        assert c.mrad_per_px == pytest.approx(1000.0 / c.focal_px)
        assert c.mrad_per_px > math.radians(104.0) * 1000.0 / 1920 * 1.3
        # the frame edge is hfov/2 off axis
        assert c.px_to_mrad(1920, 540)[0] == pytest.approx(math.radians(52.0) * 1000.0, rel=1e-6)
        # round trip
        x, y = c.mrad_to_px(*c.px_to_mrad(123.4, 777.7))
        assert (x, y) == pytest.approx((123.4, 777.7))


# ---------------------------------------------------------------- steady (non-blinking) spot detector

def _bgr(shape=(120, 200), ambient=60, seed=0):
    rng = np.random.default_rng(seed)
    return (ambient + rng.integers(0, 4, size=shape + (3,))).astype(np.float32)


def _spot(img, x, y, peak, bgr=(1.0, 1.0, 1.0), sigma=1.6):
    yy, xx = np.mgrid[0:img.shape[0], 0:img.shape[1]]
    g = peak * np.exp(-((xx - x) ** 2 + (yy - y) ** 2) / (2 * sigma ** 2))
    img += g[..., None] * np.asarray(bgr, np.float32) if img.ndim == 3 else g


def _u8(img):
    return np.clip(img, 0, 255).astype(np.uint8)


class TestSteadySpot:
    def test_red_mode_ignores_a_brighter_white_lamp(self):
        img = _bgr()
        _spot(img, 60, 50, 400, sigma=3.0)                       # white lamp, clips
        _spot(img, 140.3, 70.6, 150, bgr=(0.15, 0.2, 1.0))       # red LED
        from vision import SteadySpotDetector
        assert SteadySpotDetector(40, color="gray").detect(_u8(img)).x == pytest.approx(60, abs=1)
        b = SteadySpotDetector(40, color="red").detect(_u8(img))
        assert (b.x, b.y) == pytest.approx((140.3, 70.6), abs=0.3)

    def test_clipped_white_led_core_is_found_from_its_red_ring(self):
        img = _bgr()
        _spot(img, 100.0, 60.0, 900, bgr=(0.3, 0.35, 1.0), sigma=2.5)   # core clips to (255, 255, 255)
        u = _u8(img)
        assert (u[60, 100] == 255).all()
        from vision import SteadySpotDetector
        b = SteadySpotDetector(40, color="red", bg_kernel_px=31).detect(u)   # background scale of a 960 px frame
        assert b is not None and (b.x, b.y) == pytest.approx((100, 60), abs=0.5)

    def test_large_bright_area_and_its_corners_are_ignored(self):
        img = np.random.default_rng(0).integers(0, 4, size=(120, 200)).astype(np.float32) + 40
        img[20:80, 20:90] = 255                                  # blown-out window, far brighter than the LED
        _spot(img, 150, 60, 120)
        from vision import SteadySpotDetector
        b = SteadySpotDetector(40, color="gray").detect(_u8(img))
        assert b is not None and (b.x, b.y) == pytest.approx((150, 60), abs=0.5)

    def test_prefer_picks_the_spot_near_the_prediction(self):
        img = np.zeros((120, 200), np.float32)
        _spot(img, 50, 60, 200)
        _spot(img, 150, 60, 120)
        from vision import SteadySpotDetector
        d = SteadySpotDetector(40)
        assert d.detect(_u8(img)).x == pytest.approx(50, abs=0.5)
        assert d.detect(_u8(img), prefer=(145, 62, 15)).x == pytest.approx(150, abs=0.5)

    def test_clutter_map_hides_a_known_lamp_from_the_search(self):
        from vision import ClutterMap, SteadySpotDetector
        d = SteadySpotDetector(40)
        lamp = np.zeros((120, 200), np.float32)
        _spot(lamp, 50, 60, 200)
        cm = ClutterMap(lamp.shape)
        cm.learn(d.candidates(_u8(lamp)))                        # beacon off: only the lamp
        assert cm.count == 1
        scene = lamp.copy()
        _spot(scene, 150, 60, 120)
        assert d.detect(_u8(scene), exclude=cm.excluded).x == pytest.approx(150, abs=0.5)

    def test_noisy_frame_without_a_spot_gives_nothing(self):
        img = np.random.default_rng(1).normal(100, 15, size=(120, 200)).astype(np.float32)
        from vision import SteadySpotDetector
        assert SteadySpotDetector(25).detect(_u8(img)) is None

    def test_edges_and_lines_are_rejected(self):
        img = np.zeros((120, 200), np.float32)
        img[58:61, 20:120] = 200                                  # thin bright line (a lit edge)
        from vision import SteadySpotDetector
        assert SteadySpotDetector(40).detect(_u8(img)) is None


class TestCourtLines:
    """A squash court: red painted lines on a white wall, mid-grey exposure. The detector runs with the
    background scale of a 960 px frame (31 px), as on the rig."""

    LINES = [((0, 135), (399, 135), 4), ((150, 0), (165, 239), 3), ((250, 30), (390, 200), 3)]

    def court(self, led=None, peak=300.0, stub=False, lamp=None):
        rng = np.random.default_rng(0)
        img = (150 + rng.integers(0, 4, size=(240, 400, 3))).astype(np.float32)
        m = np.zeros((240, 400), np.uint8)
        for p, q, w in self.LINES + ([((60, 200), (68, 204), 4)] if stub else []):
            cv2.line(m, p, q, 1, w)
        img[m > 0] = (35, 35, 120)                                   # red paint (BGR)
        if lamp is not None:
            _spot(img, *lamp, 500, sigma=2.5)                        # white lamp, clips
        if led is not None:
            _spot(img, *led, peak, bgr=(0.25, 0.3, 1.0), sigma=1.6)
        return _u8(cv2.GaussianBlur(img, (3, 3), 0))

    def test_lines_alone_give_nothing(self):
        from vision import SteadySpotDetector
        assert SteadySpotDetector(40, bg_kernel_px=31).candidates(self.court()) == []

    @pytest.mark.parametrize("led", [(200, 135), (155, 80), (320, 115), (200, 60)],
                             ids=["on horizontal", "on vertical", "on diagonal", "off the lines"])
    def test_led_is_found_on_and_off_the_lines(self, led):
        from vision import SteadySpotDetector
        b = SteadySpotDetector(40, bg_kernel_px=31).detect(self.court(led))
        assert b is not None and (b.x, b.y) == pytest.approx(led, abs=0.7)

    def test_led_crossing_a_line_edge_is_never_lost(self):
        from vision import SteadySpotDetector
        d = SteadySpotDetector(40, bg_kernel_px=31)
        for x in np.arange(146.0, 164.5, 1.5):                      # the vertical line is at x = 155 here
            b = d.detect(self.court((x, 80)))
            assert b is not None and abs(b.x - x) < 1.5 and abs(b.y - 80) < 0.5, (x, b)

    def test_short_piece_of_line_is_paint_not_a_light(self):
        from vision import SteadySpotDetector
        c = SteadySpotDetector(40, bg_kernel_px=31).candidates(self.court(stub=True), keep_rejected=True)
        assert all(not q.valid for q in c)
        assert any(q.reason == "not a light" and abs(q.x - 64) < 4 for q in c)

    def test_white_lamp_touching_a_red_line_is_not_made_red(self):
        from vision import SteadySpotDetector
        b = SteadySpotDetector(40, bg_kernel_px=31).detect(self.court(led=(200, 60), lamp=(100, 131)))
        assert b is not None and (b.x, b.y) == pytest.approx((200, 60), abs=0.7)
