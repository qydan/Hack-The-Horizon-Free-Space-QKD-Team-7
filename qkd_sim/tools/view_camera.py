"""Show raw Action 5 Pro frames with sim time and gimbal attitude overlaid. No detection of any kind.

    python tools/view_camera.py --rig rig_fixed               # simulated camera (a scenario must be running)
    python tools/view_camera.py --rig rig_gimbal --save out/  # also save every Nth frame
    python tools/view_camera.py --hardware --device 0         # real camera in webcam mode

Keys: q quits, s saves the current frame, space pauses.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sim_bridge.frame_source import make_frame_source  # noqa: E402
from sim_bridge.gimbal_io import make_gimbal_io  # noqa: E402
from tools.scenario import normalize_rig  # noqa: E402


def main() -> None:
    import cv2
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rig", default="fixed", help="fixed|gimbal (or A|B)")
    ap.add_argument("--fidelity", default=None, choices=["full", "fast"])
    ap.add_argument("--hardware", action="store_true")
    ap.add_argument("--device", default=0)
    ap.add_argument("--scale", type=float, default=0.5, help="display scale")
    ap.add_argument("--save", type=Path, default=None)
    ap.add_argument("--save-every", type=int, default=30)
    ap.add_argument("--no-window", action="store_true", help="headless: only save frames")
    args = ap.parse_args()
    args.rig = normalize_rig(args.rig)

    if args.hardware:
        src = make_frame_source(hardware=True, device=int(args.device) if str(args.device).isdigit() else args.device)
    else:
        src = make_frame_source(rig=args.rig, fidelity=args.fidelity, publish_meta=False)
    try:
        gimbal = make_gimbal_io(args.rig, hardware=args.hardware,
                                world=getattr(src, "world", None))
    except Exception as exc:  # noqa: BLE001
        print(f"gimbal readback unavailable: {exc}")
        gimbal = None
    if args.save:
        args.save.mkdir(parents=True, exist_ok=True)
    paused = False
    last_wall = time.monotonic()
    fps = 0.0
    while True:
        if not paused:
            fr = src.read(timeout_s=5.0)
            if fr is None:
                print("no frame within 5 s (is the scenario running?)")
                continue
            now = time.monotonic()
            fps = 0.9 * fps + 0.1 / max(now - last_wall, 1e-6)
            last_wall = now
            img = fr.image.copy()
            lines = [f"seq {fr.seq}  t_capture {fr.t_capture_sim:9.3f} s  t_delivered {fr.t_delivered_sim:9.3f} s",
                     f"latency {1e3 * (fr.t_delivered_sim - fr.t_capture_sim):6.1f} ms   display {fps:5.1f} Hz (wall)"]
            if gimbal is not None:
                try:
                    g = gimbal.read()
                    lines.append(f"{args.rig}: pan {math.degrees(g.pan_rad):8.3f} deg  tilt {math.degrees(g.tilt_rad):7.3f} deg"
                                 f"  roll {math.degrees(g.roll_rad):6.3f} deg  (t {g.t_sim:9.3f})")
                except Exception as exc:  # noqa: BLE001
                    lines.append(f"gimbal: {exc}")
            for i, text in enumerate(lines):
                cv2.putText(img, text, (20, 40 + 36 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(img, text, (20, 40 + 36 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 1, cv2.LINE_AA)
            if args.save and fr.seq % args.save_every == 0:
                cv2.imwrite(str(args.save / f"frame_{fr.seq:06d}.png"), fr.image)
        if args.no_window:
            continue
        cv2.imshow("Action 5 Pro (raw)", cv2.resize(img, None, fx=args.scale, fy=args.scale))
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s") and args.save:
            cv2.imwrite(str(args.save / f"snap_{fr.seq:06d}.png"), fr.image)
        if key == ord(" "):
            paused = not paused
    src.close()


if __name__ == "__main__":
    main()
