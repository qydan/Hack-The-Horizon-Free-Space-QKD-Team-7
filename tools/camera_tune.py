"""Live camera view for tuning the LED detector on the real rig (needs a display).

    python tools/camera_tune.py --device 0

Shows the detected centroid, tracker state, and drift/jitter/intensity. Keys:
    q quit    c calibrate (average the next 3 s as boresight / full-signal brightness)
Use the sliders to adjust the HSV thresholds, then copy the values into LEDDetector.
Run it with the Python that owns the camera (e.g. Windows Python, not WSL).
"""
import argparse
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from camera_provider import Calibration, LEDDetector, PATTracker, VideoCaptureSource  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=60)
    ap.add_argument("--exposure", type=float, default=-6)
    a = ap.parse_args()

    src = VideoCaptureSource(a.device, a.width, a.height, a.fps, a.exposure)
    det = LEDDetector()
    tracker = PATTracker(det, Calibration(det.norm_width / 2, det.norm_width * a.height / a.width / 2, 100.0))
    cv2.namedWindow("tune")
    for name, init, mx in (("H low", det.hsv_low[0], 179), ("H high", det.hsv_high[0], 179),
                           ("S low", det.hsv_low[1], 255), ("V low", det.hsv_low[2], 255)):
        cv2.createTrackbar(name, "tune", int(init), mx, lambda v: None)

    cal_until, cal_dets = 0.0, []
    while True:
        item = src.read()
        if item is None:
            break
        frame, t = item
        det.hsv_low[:] = (cv2.getTrackbarPos("H low", "tune"), cv2.getTrackbarPos("S low", "tune"),
                          cv2.getTrackbarPos("V low", "tune"))
        det.hsv_high[0] = cv2.getTrackbarPos("H high", "tune")
        s = tracker.update(frame, t)
        view = frame.copy()
        if tracker._last is not None:
            scale = frame.shape[1] / det.norm_width
            cv2.circle(view, (int(tracker._last.x * scale), int(tracker._last.y * scale)), 14, (0, 255, 255), 2)
        txt = f"{s.state} drift={s.drift:.1f}px jitter={s.jitter:.1f}px/s I={s.intensity:.2f}"
        if tracker.acquisition_time is not None:
            txt += f"  acq={tracker.acquisition_time:.2f}s"
        cv2.putText(view, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("tune", view)

        if cal_until:
            if tracker.state == "TRACK" and tracker._last:
                cal_dets.append(tracker._last)
            if time.monotonic() > cal_until:
                cal = Calibration.from_detections(cal_dets)
                tracker.cal = cal
                cal.save("calibration.json")
                print("saved calibration.json:", cal)
                cal_until, cal_dets = 0.0, []
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k == ord("c"):
            cal_until, cal_dets = time.monotonic() + 3.0, []
            print("calibrating for 3 s...")
    src.release()
    cv2.destroyAllWindows()
    print(f"HSV low={tuple(int(x) for x in det.hsv_low)} high={tuple(int(x) for x in det.hsv_high)}")


if __name__ == "__main__":
    main()
