"""Camera frame sources: the simulated Action 5 Pro (Gazebo) and the real one (UVC webcam mode).

``FrameSource`` is one of the two sim_bridge modules tracking/ may import. Both implementations return the
same :class:`Frame`: BGR uint8 (OpenCV convention), capture and delivery timestamps, camera id and sequence.

* :class:`GzAction5FrameSource` subscribes to the Gazebo camera topic and runs the Action 5 Pro emulation
  (rolling shutter, auto-exposure, MJPG, latency; see ``image_pipeline.py``). Timestamps are Gazebo sim time.
  Simulator-only metadata (exposure gain, saturation) is NOT exposed on the Frame, because the real camera does
  not report it; it is published on the internal frames_meta port for ``frames_meta.csv`` and evaluation.
* :class:`UvcFrameSource` captures the real camera. Timestamps are host monotonic time (seconds).

Use :func:`make_frame_source` to get the right one.
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Protocol, runtime_checkable

import numpy as np

from sim_bridge import config as cfgmod

log = logging.getLogger("frame_source")


@dataclass(frozen=True)
class Frame:
    image: np.ndarray          # H x W x 3 uint8, BGR
    t_capture_sim: float       # readout start of the first row (sim time; host time on real hardware)
    t_delivered_sim: float     # when the frame became available to the reader
    camera_id: str
    seq: int


@runtime_checkable
class FrameSource(Protocol):
    def read(self, timeout_s: float = 5.0) -> Frame | None:
        """Block until a frame newer than the last one returned has been delivered; return the newest.
        Returns None on timeout."""

    def close(self) -> None: ...


class GzAction5FrameSource:  # pragma: no cover - needs Gazebo
    """Simulated Action 5 Pro in webcam mode, fed by the Gazebo camera sensor."""

    def __init__(self, *, rig: str | None = None, fidelity: str | None = None, world=None,
                 overrides: dict[str, dict[str, Any]] | None = None, publish_meta: bool | None = None):
        from sim_bridge.camera_model import EquidistantLens, rectilinear_to_equidistant_maps, render_sensor_type
        from sim_bridge.gz_io import GzWorld, image_msg_type
        from sim_bridge.image_pipeline import Action5Pipeline

        cfgs = cfgmod.load_all(overrides)
        cam = cfgs["camera_action5"]
        rig = rig or os.environ.get("QKD_RIG") or cfgs["rigs"]["default"]
        fidelity = fidelity or os.environ.get("QKD_FIDELITY") or cam["fidelity"]["default"]
        self.camera_id = cam["camera_id"]
        self.world = world or GzWorld(cfgs["world"]["world_name"])
        pp = cfgs["rigs"][rig]["principal_point_offset_px"]
        remap = None
        if render_sensor_type(cam) == "rectilinear_remap":     # only for the equidistant fisheye option
            lens = EquidistantLens.from_cfg(cam)
            r = cam["render"]
            remap = rectilinear_to_equidistant_maps(lens, int(r["rectilinear_width_px"]),
                                                    int(r["rectilinear_height_px"]),
                                                    np.radians(float(r["rectilinear_hfov_deg"])))
        self.pipeline = Action5Pipeline(cam, fidelity, pp_offset_px=pp, remap=remap)
        self._lock = threading.Condition()
        self._delivered: deque = deque(maxlen=8)
        self._last_seq = -1
        self.renders = 0
        self._meta_sock = None
        if publish_meta if publish_meta is not None else os.environ.get("QKD_FRAMES_META", "1") == "1":
            import zmq
            bus = cfgs["bus"]
            self._meta_sock = zmq.Context.instance().socket(zmq.PUB)
            self._meta_sock.connect(f"tcp://{bus['host']}:{bus['internal_ports']['frames_meta']}")
        self.world.subscribe(image_msg_type(), cam["topics"]["image"], self._on_image)
        self._stop = threading.Event()
        self._deliverer = threading.Thread(target=self._deliver_loop, daemon=True, name="frame-deliver")
        self._deliverer.start()

    def _on_image(self, msg) -> None:
        from sim_bridge.gz_io import stamp_to_s
        t = stamp_to_s(msg.header.stamp)
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self.renders += 1
        with self._lock:
            self.pipeline.push_render(img, t)

    def _deliver_loop(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                nxt = self.pipeline.pending[0].t_delivered_sim if self.pipeline.pending else None
            if nxt is None:
                time.sleep(0.002)
                continue
            self.world.wait_until(nxt, timeout_wall_s=0.5)
            with self._lock:
                for pf in self.pipeline.pop_delivered(self.world.sim_time):
                    self._delivered.append(Frame(pf.image_bgr, pf.t_capture_sim, max(pf.t_delivered_sim,
                                                 self.world.sim_time), self.camera_id, pf.seq))
                    if self._meta_sock is not None:
                        self._meta_sock.send_multipart([b"frames_meta", json.dumps({
                            "seq": pf.seq, "t_capture": pf.t_capture_sim, "t_delivered": pf.t_delivered_sim,
                            "exposure_gain": pf.exposure_gain, "saturated": pf.saturated}).encode()])
                self._lock.notify_all()

    def read(self, timeout_s: float = 5.0) -> Frame | None:
        with self._lock:
            ok = self._lock.wait_for(lambda: self._delivered and self._delivered[-1].seq > self._last_seq,
                                     timeout=timeout_s)
            if not ok:
                return None
            f = self._delivered[-1]
            self._last_seq = f.seq
            return f

    def close(self) -> None:
        self._stop.set()


class UvcFrameSource:  # pragma: no cover - needs the camera
    """Real Action 5 Pro in webcam (UVC) mode.

    MJPG FOURCC is set *before* resolution and fps (otherwise many UVC drivers fall back to YUYV at a low
    frame rate). A capture thread keeps only the newest frame. Timestamps are host monotonic time at grab;
    the capture-to-frame latency is unknown to the host and must be calibrated (calibration/latency.py).
    """

    def __init__(self, device: int | str = 0, *, width: int | None = None, height: int | None = None,
                 fps: float | None = None):
        import cv2
        cam = cfgmod.load("camera_action5")
        self.camera_id = cam["camera_id"]
        api = cv2.CAP_V4L2 if hasattr(cv2, "CAP_V4L2") and os.name != "nt" else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(device, api)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open camera {device!r}")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width or cam["stream"]["width_px"])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height or cam["stream"]["height_px"])
        self.cap.set(cv2.CAP_PROP_FPS, fps or cam["stream"]["fps"])
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._cv = threading.Condition()
        self._latest: Frame | None = None
        self._last_seq = -1
        self._seq = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="uvc-capture")
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            ok, img = self.cap.read()
            t = time.monotonic()
            if not ok:
                time.sleep(0.005)
                continue
            with self._cv:
                self._latest = Frame(img, t, t, self.camera_id, self._seq)
                self._seq += 1
                self._cv.notify_all()

    def read(self, timeout_s: float = 5.0) -> Frame | None:
        with self._cv:
            if not self._cv.wait_for(lambda: self._latest is not None and self._latest.seq > self._last_seq,
                                     timeout=timeout_s):
                return None
            self._last_seq = self._latest.seq
            return self._latest

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.cap.release()


def make_frame_source(*, hardware: bool = False, device: int | str = 0, **kwargs) -> FrameSource:
    """Simulation by default; ``hardware=True`` (or env QKD_HARDWARE=1) opens the real camera."""
    if hardware or os.environ.get("QKD_HARDWARE") == "1":
        return UvcFrameSource(device)
    return GzAction5FrameSource(**kwargs)
