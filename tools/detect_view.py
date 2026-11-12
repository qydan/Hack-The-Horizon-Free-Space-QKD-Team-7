"""Live debug view of what the steady-LED detector sees, on the real camera.

    python tools/detect_view.py --device 0 --fps 30
    python tools/detect_view.py --zone 0.25,0.1,0.75,0.9 --min-contrast 25

Uses the same SteadySpotDetector (red-excess + local-background top-hat), proc-width downscale and --zone as
`src/tracker.py --beacon steady`, so what is circled here is what the tracker would lock onto. Run on Windows
Python (not WSL).

Overlay colours
  green circle + cross   the spot the tracker would pick
  orange boxes + label   other candidates: valid ones show their contrast, rejected ones why
                         (small / large / elongated / not a light = paint darker than its surroundings,
                         e.g. a short piece of court line / clutter)
  magenta                masked clutter (learned with [c]; the tracker's --clutter-learn does the same)
  red pixels             saturated (>= 250 in all channels) pixels, i.e. blown-out light sources
  cyan rectangle         the search zone (anything outside it is ignored)
HUD: the winning spot's contrast and the detector's min-contrast. Contrast is measured after straight lines (court lines),
     edges and large areas are removed: red-excess units (R - max(G, B)) in red mode, grey levels in gray mode.

Keys: q quit | h toggle feature heat-map (what the detector thresholds) | r toggle red / gray
      +/- change min-contrast | c learn clutter from this frame (beacon OFF) | x clear clutter
      s save the current frame + overlay to detect_view_<n>.png
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
sys.path.insert(0, HERE)
from vision import ClutterMap, SteadySpotDetector  # noqa: E402
from camera_check import open_camera  # noqa: E402


def label(vis, text, org, colour):
    cv2.putText(vis, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
    cv2.putText(vis, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--fps", type=float, default=30)
    ap.add_argument("--backend", choices=["dshow", "msmf", "any"], default="dshow")
    ap.add_argument("--fourcc", default="MJPG")
    ap.add_argument("--exposure", type=float, default=None, help="manual exposure (DirectShow: 2^value s)")
    ap.add_argument("--dialog", action="store_true", help="open the DirectShow driver settings dialog")
    ap.add_argument("--proc-width", type=int, default=960, help="downscale to this width, as the tracker does")
    ap.add_argument("--zone", default=None, help="x0,y0,x1,y1 as 0-1 fractions, as in tracker.py")
    ap.add_argument("--min-contrast", type=float, default=25.0,
                    help="contrast a spot needs above its local background (tracker: --min-amplitude)")
    ap.add_argument("--spot-color", choices=["red", "gray"], default="red", help="as tracker.py --spot-color")
    ap.add_argument("--spot-max-area", type=int, default=None, help="default: as the tracker, scaled to proc width")
    ap.add_argument("--scale", type=float, default=1.0, help="display scale")
    a = ap.parse_args()

    cap = open_camera(a)
    zone = tuple(float(v) for v in a.zone.split(",")) if a.zone else None
    min_contrast, color = a.min_contrast, a.spot_color
    heat, saved = False, 0
    clutter = None
    t_prev, fps_disp = time.monotonic(), 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        if a.proc_width and frame.shape[1] > a.proc_width:
            s = a.proc_width / frame.shape[1]
            frame = cv2.resize(frame, (a.proc_width, int(round(frame.shape[0] * s))), interpolation=cv2.INTER_AREA)
        h, w = frame.shape[:2]
        if clutter is None or clutter.mask.shape != (h, w):
            clutter = ClutterMap((h, w))
        roi = (0, 0, w, h)
        if zone:
            roi = (int(zone[0] * w), int(zone[1] * h), int(zone[2] * w), int(zone[3] * h))

        max_area = a.spot_max_area or max(50, round(600 * (w / 960) ** 2))
        det = SteadySpotDetector(min_contrast, max_area=max_area, color=color)
        t0 = time.perf_counter()
        cands = det.candidates(frame, roi, exclude=clutter.excluded, keep_rejected=True)
        cost_ms = 1000 * (time.perf_counter() - t0)
        best = cands[0] if cands and cands[0].valid else None

        if heat:
            _, th = det.contrast_map(frame)      # what is left after lines, edges and large areas are removed
            c = np.clip(th * (255.0 / max(1.0, min_contrast * 2)), 0, 255).astype(np.uint8)
            vis = cv2.applyColorMap(c, cv2.COLORMAP_INFERNO)
        else:
            vis = frame.copy()
        vis[(frame >= 250).all(axis=2)] = (0, 0, 255)                                   # saturated pixels
        vis[clutter.excluded] = (0.5 * vis[clutter.excluded] + (127, 0, 127)).astype(np.uint8)
        for c in cands[1 if best else 0:]:
            bx, by, bw, bh = c.bbox
            cv2.rectangle(vis, (bx - 3, by - 3), (bx + bw + 3, by + bh + 3), (0, 165, 255), 1)
            label(vis, c.reason or f"+{c.contrast:.0f}", (bx + bw + 5, by + 8), (0, 165, 255))
        if zone:
            cv2.rectangle(vis, (roi[0], roi[1]), (roi[2], roi[3]), (255, 255, 0), 1)
        if best is not None:
            p = (int(round(best.x)), int(round(best.y)))
            cv2.circle(vis, p, 14, (0, 255, 0), 2)
            cv2.drawMarker(vis, p, (0, 255, 0), cv2.MARKER_CROSS, 10, 1)

        now = time.monotonic()
        fps_disp = 0.9 * fps_disp + 0.1 / max(1e-6, now - t_prev) if fps_disp else 1.0 / max(1e-6, now - t_prev)
        t_prev = now
        n_valid = sum(c.valid for c in cands)
        lines = [f"{color} mode  min-contrast {min_contrast:.0f}  detect {cost_ms:4.1f} ms  fps {fps_disp:4.1f}",
                 (f"LOCK at ({best.x:.0f},{best.y:.0f})  contrast +{best.contrast:.0f}  area {best.area}  "
                  f"rivals {n_valid - 1}" if best else
                  ("NO LOCK: no spot above the threshold" if not cands else
                   "NO LOCK: all candidates rejected - see labels")),
                 f"clutter regions {clutter.count}   [h] heat {'ON' if heat else 'off'}  [r] red/gray  "
                 f"[+/-] contrast  [c] learn clutter  [x] clear  [s] save  [q] quit"]
        for i, text in enumerate(lines):
            cv2.putText(vis, text, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(vis, text, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        if a.scale != 1.0:
            vis = cv2.resize(vis, None, fx=a.scale, fy=a.scale)
        cv2.imshow("detect_view", vis)

        k = cv2.waitKey(1) & 0xFF
        if k == ord("q") or k == 27:
            break
        elif k == ord("h"):
            heat = not heat
        elif k == ord("r"):
            color = "gray" if color == "red" else "red"
        elif k in (ord("+"), ord("=")):
            min_contrast += 5
        elif k in (ord("-"), ord("_")):
            min_contrast = max(5.0, min_contrast - 5)
        elif k == ord("c"):
            clutter.learn([c for c in cands if c.valid])
            print(f"clutter: {clutter.count} region(s) masked")
        elif k == ord("x"):
            clutter.clear()
        elif k == ord("s"):
            name = f"detect_view_{saved}.png"
            cv2.imwrite(name, vis)
            print("saved", name)
            saved += 1

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
