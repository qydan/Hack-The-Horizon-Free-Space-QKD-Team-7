"""Tracker entry point: wires acquisition, tracking, filter, pointing, state machine and telemetry for the
chosen rig.

    python -m tracking.main --rig rig_fixed|rig_gimbal [--hardware] [--duration S]

Uses ``sim_bridge.frame_source.make_frame_source`` and ``sim_bridge.gimbal_io.make_gimbal_io`` so the same code
runs in simulation and on the real hardware. tools/run_scenario.py launches this module (the sandbox listens on
the other end of port 5556; an abort from its gate on 5557 makes the tracker drop its lock and re-acquire).
"""
from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import time
from typing import Any

log = logging.getLogger("tracking")


def run(rig: str, cfg: dict[str, Any], hardware: bool = False, *, source=None, gimbal=None, out=None,
        duration_s: float | None = None, stop=None) -> Any:
    """Run the tracker until interrupted (or for ``duration_s`` of frame time). Returns the Tracker.
    ``source`` / ``gimbal`` / ``out`` default to the real or simulated interfaces (tests inject their own)."""
    from sim_bridge.frame_source import make_frame_source
    from sim_bridge.gimbal_io import make_gimbal_io
    from tracking.telemetry_out import TelemetryOut
    from tracking.tracker import Tracker

    stopped = {"v": False}
    if stop is None:
        def _stop(*_):
            stopped["v"] = True
        for sig in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGBREAK", None)):
            if sig is not None:
                try:
                    signal.signal(sig, _stop)
                except (ValueError, OSError):
                    pass
        stop = lambda: stopped["v"]  # noqa: E731

    source = source or make_frame_source(hardware=hardware, rig=rig)
    gimbal = gimbal or make_gimbal_io(rig, hardware=hardware)
    out = out or TelemetryOut(cfg, rig)
    tr = Tracker(cfg, rig, gimbal)
    log.info("tracker %s: camera %s (%.2f mrad/px at the centre), pointing %s, target %s", rig, tr.cam.source,
             tr.cam.mrad_per_px, tr.pointer.mode, tr.target)
    t0 = None
    last_log = 0.0
    last_state = None
    try:
        while not stop():
            fr = source.read(timeout_s=1.0)
            if fr is None:
                if getattr(source, "finished", False):     # a finite source (replay, test rig) ran out
                    break
                continue                                    # a live source timed out: keep waiting
            t0 = fr.t_capture_sim if t0 is None else t0
            cmd = out.poll_abort()
            if cmd is not None and cmd.get("cmd", "reacquire") == "reacquire":
                log.info("gate abort (%s): re-acquiring", cmd.get("reason", ""))
                tr.reacquire(fr.t_capture_sim)
            rec = out.publish(tr.step(fr))
            if rec["state"] != last_state:
                log.info("t=%.2f state %s", rec["t"] - t0, rec["state"])
                last_state = rec["state"]
            if time.monotonic() - last_log > 5.0:
                last_log = time.monotonic()
                log.info("t=%.1f %s residual %.2f mrad drift %.2f jitter %.2f intensity %.2f gain %.2f | first lock %s, "
                         "lock losses %d", rec["t"] - t0, rec["state"], rec["residual_mrad"], rec["drift_mrad"],
                         rec["jitter_mrad"], rec["intensity"], rec["exposure_gain"],
                         "-" if tr.acquisition_time is None else f"{tr.acquisition_time:.2f}s", tr.lock_losses)
            if duration_s is not None and fr.t_capture_sim - t0 >= duration_s:
                break
    except KeyboardInterrupt:
        pass
    finally:
        close = getattr(source, "close", None)
        if close:
            close()
        if hasattr(out, "close"):
            out.close()
    log.info("tracker done: %d records, first lock %s, lock losses %d, aborts %d", getattr(out, "sent", 0),
             "never" if tr.acquisition_time is None else f"{tr.acquisition_time:.2f}s", tr.lock_losses, tr.aborts)
    return tr


def main() -> None:
    """Parse arguments and call :func:`run`."""
    from tracking.config import load_tracking_cfg
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rig", default=os.environ.get("QKD_RIG", "rig_fixed"), choices=["rig_fixed", "rig_gimbal"])
    ap.add_argument("--hardware", action="store_true", help="real camera / gimbal instead of the simulator")
    ap.add_argument("--duration", type=float, default=None, help="stop after this many seconds of frames")
    ap.add_argument("--target", choices=["red_led", "tag", "auto"], help="override tracking.yaml acquire.target")
    ap.add_argument("--beacon", choices=["blink", "steady"], help="override acquire.beacon (default: beacon.yaml)")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s", stream=sys.stdout)
    cfg = load_tracking_cfg()
    if a.target:
        cfg["acquire"]["target"] = a.target
    if a.beacon:
        cfg["acquire"]["beacon"] = a.beacon
    run(a.rig, cfg, a.hardware, duration_s=a.duration)
    # The simulated frame source holds gz-transport nodes whose C++ teardown at interpreter exit aborts the process
    # ("terminate called without an active exception"). Everything is flushed: leave without running it (as
    # tools/run_scenario.py does).
    sys.stdout.flush()
    logging.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
