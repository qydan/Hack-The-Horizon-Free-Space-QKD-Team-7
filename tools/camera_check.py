"""Camera bring-up check: is the camera good enough for the 13 Hz lock-in tracker?

    python tools/camera_check.py --device 0 --width 1920 --height 1080 --fps 60

Run on Windows Python (not WSL). Three phases:
  1. FRAME RATE   real fps, dropped frames, and whether brightness drifts (auto exposure).
  2. NOISE FLOOR  beacon OFF/covered: the lock-in amplitude of pure noise -> min_amplitude.
  3. BEACON       beacon ON, held still: lock-in amplitude, SNR, saturation, spot position.
Prints a verdict and the TrackerConfig / CameraModel values to use.
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from vision import LockInDetector  # noqa: E402


def open_camera(a):
    backend = {"dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF, "any": cv2.CAP_ANY}[a.backend]
    cap = cv2.VideoCapture(a.device, backend)
    if not cap.isOpened():
        raise SystemExit(f"cannot open camera {a.device} with backend {a.backend}")
    if a.fourcc != "none":
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*a.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    cap.set(cv2.CAP_PROP_FPS, a.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if a.exposure is not None:
        # manual exposure: this also stops the camera trading frame rate for light
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)   # V4L2-style manual
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)      # DirectShow manual
        cap.set(cv2.CAP_PROP_EXPOSURE, a.exposure)
        print(f"exposure set to {a.exposure}, camera reports {cap.get(cv2.CAP_PROP_EXPOSURE)}")
    if a.dialog:
        cap.set(cv2.CAP_PROP_SETTINGS, 1)           # DirectShow driver dialog
        input("Adjust the driver dialog (turn OFF low-light compensation / auto frame rate), then Enter...")
    fc = int(cap.get(cv2.CAP_PROP_FOURCC))
    print(f"driver agreed to: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} "
          f"@ {cap.get(cv2.CAP_PROP_FPS):.0f} fps, format {''.join(chr((fc >> 8 * i) & 255) for i in range(4))!r}, "
          f"backend {a.backend}")
    return cap


def grab(cap):
    ok, f = cap.read()
    return (f, time.monotonic()) if ok else (None, None)


def phase_fps(cap, seconds):
    print(f"\n[1/3] FRAME RATE ({seconds:.0f} s) ...")
    ts, means = [], []
    t_end = time.monotonic() + seconds
    shape = None
    while time.monotonic() < t_end:
        f, t = grab(cap)
        if f is None:
            continue
        shape = f.shape
        ts.append(t)
        means.append(float(f[::8, ::8].mean()))
    ts = np.array(ts)
    if len(ts) < 10:
        raise SystemExit("got almost no frames")
    dt = np.diff(ts)
    fps = 1.0 / float(np.median(dt))
    late = int((dt > 1.5 * np.median(dt)).sum())
    drift = (max(means) - min(means)) / max(1.0, np.mean(means))
    print(f"  frame size        {shape[1]}x{shape[0]}")
    print(f"  measured fps      {fps:.1f}   (median), mean {len(ts) / (ts[-1] - ts[0]):.1f}")
    print(f"  late/dropped      {late} of {len(dt)} frames")
    print(f"  brightness drift  {100 * drift:.1f}%  (high with a static scene = auto exposure/gain still on)")
    return fps, shape, drift, late / len(dt)


def phase_lockin(cap, fps, seconds, a, label):
    det = LockInDetector(a.blink_hz, fps, a.periods, min_amplitude=0.0)
    cost, peaks, sat = [], [], []
    t_end = time.monotonic() + seconds
    pos = None
    while time.monotonic() < t_end:
        f, t = grab(cap)
        if f is None:
            continue
        if a.proc_width and f.shape[1] > a.proc_width:
            s = a.proc_width / f.shape[1]
            f = cv2.resize(f, (a.proc_width, int(f.shape[0] * s)), interpolation=cv2.INTER_AREA)
        t0 = time.perf_counter()
        det.push(f, t)
        cost.append(time.perf_counter() - t0)
        if det.ready:
            m = det.amp_map
            peaks.append(float(m.max()))
            if label == "beacon":
                y, x = np.unravel_index(int(np.argmax(m)), m.shape)
                pos = (x, y, int(f[max(0, y - 3):y + 4, max(0, x - 3):x + 4].max()), m.shape)
                g = f if f.ndim == 2 else cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
                sat.append(float((g >= 250).mean()))
    if not peaks:
        raise SystemExit("lock-in never filled its window (too few frames)")
    return dict(peak=float(np.percentile(peaks, 95)), median=float(np.median(peaks)),
                cost_ms=1000 * float(np.mean(cost)), pos=pos, sat=float(np.mean(sat)) if sat else 0.0,
                window=det.n)


def steady_check(cap, fps, shape, a):
    """Constantly-lit LED: how clearly does it stand out, and is anything else competing?"""
    from vision import SteadySpotDetector
    det = SteadySpotDetector(min_contrast=a.contrast, color=a.spot_color)
    input("\n[2/2] STEADY BEACON: switch the LED ON at its real working distance, keep it still, "
          "press Enter...")
    pts, peaks, sat, rivals, bgs, cost = [], [], [], [], [], []
    t_end = time.monotonic() + a.seconds
    while time.monotonic() < t_end:
        f, t = grab(cap)
        if f is None:
            continue
        if a.proc_width and f.shape[1] > a.proc_width:
            s = a.proc_width / f.shape[1]
            f = cv2.resize(f, (a.proc_width, int(f.shape[0] * s)), interpolation=cv2.INTER_AREA)
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        t0 = time.perf_counter()
        cands = det.candidates(f)
        b = cands[0].as_beacon() if cands else None
        cost.append(time.perf_counter() - t0)
        bgs.append(float(np.median(g[::4, ::4])))
        if b is not None:
            pts.append((b.x, b.y))
            peaks.append(b.peak)
            sat.append(float((g >= 250).mean()))
        rivals.append(len(cands))        # spots the detector accepts (the LED plus anything it could confuse)
    cap.release()
    rate = len(pts) / max(1, len(cost))
    print("\n================ RESULT (steady beacon) ================")
    ok = True

    def verdict(good, msg):
        nonlocal ok
        ok &= good
        print(("  OK    " if good else "  FIX   ") + msg)

    verdict(rate > 0.95, f"LED found in {100 * rate:.0f}% of frames (want > 95%)")
    if pts:
        p = np.array(pts)
        sd = p.std(axis=0)
        verdict(float(np.hypot(*sd)) < 3.0, f"spot position scatter {sd[0]:.2f}, {sd[1]:.2f} px while held still (want < 3)")
        unit = "red-excess levels" if a.spot_color == "red" else "grey levels"
        verdict(float(np.median(peaks)) >= 2 * a.contrast,
                f"LED stands {np.median(peaks):.0f} {unit} above its local background (want >= {2 * a.contrast:.0f})")
        verdict(max(sat) < 0.01, "LED not blown out" if max(sat) < 0.01 else "LED saturates a large area: lower ISO/exposure")
    verdict(float(np.median(rivals)) <= 1.5,
            f"LED-like spots in view: {np.median(rivals):.0f} (want 1: remove or --zone out the others, or mask them "
            f"with tracker --clutter-learn; tools/detect_view.py shows which)")
    if a.spot_color == "gray":
        verdict(np.mean(bgs) < 80, f"background level {np.mean(bgs):.0f} (want dark, < 80; or use --spot-color red)")
    else:
        print(f"  info  background level {np.mean(bgs):.0f} (red mode tolerates a lit room)")
    verdict(1000 * np.mean(cost) < 0.5 * 1000 / fps, f"detector cost {1000 * np.mean(cost):.1f} ms/frame")
    print(f"\n  python src/tracker.py --source camera --device {a.device} --fps {fps:.0f} --beacon steady "
          f"--spot-color {a.spot_color} --min-amplitude {a.contrast:.0f} --proc-width {a.proc_width} --hfov <fov>")
    print("\nOverall:", "READY for tracking tests" if ok else "fix the FIX lines above, then re-run")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steady", action="store_true", help="LED is constantly on (no blink)")
    ap.add_argument("--contrast", type=float, default=40.0,
                    help="steady mode: contrast above the local background (red-excess or grey levels)")
    ap.add_argument("--spot-color", choices=["red", "gray"], default="red",
                    help="steady mode: red = red excess R - max(G, B) (ignores white lights); gray = brightness")
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--fps", type=float, default=60)
    ap.add_argument("--backend", choices=["dshow", "msmf", "any"], default="dshow")
    ap.add_argument("--fourcc", default="MJPG", help="MJPG, YUY2, NV12, H264 or 'none' to leave it to the driver")
    ap.add_argument("--exposure", type=float, default=None,
                    help="manual exposure value (DirectShow: 2^value seconds, e.g. -9 = 1/512 s)")
    ap.add_argument("--dialog", action="store_true", help="open the DirectShow driver settings dialog")
    ap.add_argument("--blink-hz", type=float, default=13.0)
    ap.add_argument("--periods", type=int, default=4)
    ap.add_argument("--proc-width", type=int, default=960, help="downscale before the lock-in (0 = none)")
    ap.add_argument("--seconds", type=float, default=8.0)
    a = ap.parse_args()

    cap = open_camera(a)
    for _ in range(30):          # let the camera settle
        grab(cap)

    fps, shape, drift, late = phase_fps(cap, 5.0)
    if a.steady:
        return steady_check(cap, fps, shape, a)

    input("\n[2/3] NOISE FLOOR: cover or switch OFF the beacon (keep the scene as it will be), "
          "then press Enter...")
    noise = phase_lockin(cap, fps, a.seconds, a, "noise")
    print(f"  noise floor (lock-in amplitude, grey levels): typical {noise['median']:.1f}, worst {noise['peak']:.1f}")

    input("\n[3/3] BEACON: switch the beacon ON at its real working distance, keep it still in view, "
          "press Enter...")
    sig = phase_lockin(cap, fps, a.seconds, a, "beacon")
    cap.release()

    snr = sig["peak"] / max(noise["peak"], 1e-6)
    proc_w = a.proc_width if a.proc_width and shape[1] > a.proc_width else shape[1]
    print("\n================ RESULT ================")
    ok = True

    def verdict(good, msg):
        nonlocal ok
        ok &= good
        print(("  OK    " if good else "  FIX   ") + msg)

    verdict(fps >= 4 * a.blink_hz, f"frame rate {fps:.1f} fps ({fps / a.blink_hz:.1f} samples per blink; want >= 4)")
    verdict(late < 0.02, f"dropped frames {100 * late:.1f}% (want < 2%)")
    verdict(drift < 0.03, f"brightness drift {100 * drift:.1f}% (want < 3%: lock exposure, ISO, white balance)")
    verdict(sig["cost_ms"] < 0.6 * 1000 / fps,
            f"lock-in cost {sig['cost_ms']:.1f} ms/frame vs {1000 / fps:.1f} ms frame period "
            f"(lower --proc-width if FIX)")
    verdict(snr >= 4.0, f"beacon amplitude {sig['peak']:.1f} vs noise {noise['peak']:.1f} -> SNR {snr:.1f} (want >= 4)")
    verdict(sig["sat"] < 0.01 and (sig["pos"] is None or sig["pos"][2] < 255),
            "beacon not clipped" if sig["sat"] < 0.01 else
            "beacon is saturating (255): modulation is clipped, lower ISO/exposure or add an ND filter")
    if sig["pos"]:
        print(f"  spot found at ({sig['pos'][0]}, {sig['pos'][1]}) of {sig['pos'][3][1]}x{sig['pos'][3][0]}, "
              f"peak pixel {sig['pos'][2]}")

    min_amp = max(noise["peak"] * 2.5, 1.0)
    print("\nSuggested settings:")
    print(f"  TrackerConfig(fps={fps:.0f}, blink_hz={a.blink_hz}, lockin_periods={a.periods}, "
          f"min_amplitude={min_amp:.0f})   # window = {sig['window']} frames")
    print(f"  CameraModel(width={proc_w}, height={int(shape[0] * proc_w / shape[1])}, hfov_deg=<lens FOV after dewarp>)")
    print(f"  lock-in amplitude of the beacon at this distance (i_ref guess): {sig['peak']:.0f}")
    print("\nOverall:", "READY for tracking tests" if ok else "fix the FIX lines above, then re-run")


if __name__ == "__main__":
    main()
