"""Ground truth -> port 5590, EVALUATION ONLY (section 11.3).

Nothing under tracking/ or sandbox/ may import this module or connect to its port (tests B1, B2).

Record (one per tick, default 100 Hz sim time):
  t_sim, rig, fan_level, lighting,
  drone_pos_m, drone_quat_wxyz, drone_vel_mps,
  beacon_pos_m, lens_pos_m,
  los_az_deg, los_el_deg                (from the lens centre, world ENU; az from +x toward +y)
  cam_yaw_deg, cam_pitch_deg, cam_roll_deg   (true camera attitude incl. mount error, SDF convention)
  gimbal_joint_pan_deg, gimbal_joint_tilt_deg, gimbal_joint_roll_deg   (rig_gimbal only, else null)
  boresight_err_mrad, boresight_err_x_mrad, boresight_err_y_mrad   (optical axis vs LOS; x right, y down)
  tx_pointing_err_mrad                  (tx_frame axis vs beacon->lens direction)
  wind_mps                              (wind velocity at the drone)
  lean_deg                              (tilt of the body z axis from vertical)

Attitude convention: R_cam = Rz(yaw) Ry(pitch) Rx(roll), SDF/ENU (pitch positive = nose DOWN). The gimbal
joints are pan (about +z), tilt (about -y, positive = look up), roll (about +x).
"""
from __future__ import annotations

import json
import logging
import math
from typing import Any, Callable

import numpy as np

from sim_bridge.camera_model import rot_zyx

log = logging.getLogger("truth_publisher")
DEG = math.pi / 180.0


def quat_to_rot(q_wxyz) -> np.ndarray:
    w, x, y, z = q_wxyz
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_to_ypr(R: np.ndarray) -> tuple[float, float, float]:
    yaw = math.atan2(R[1, 0], R[0, 0])
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    return yaw, pitch, roll


def camera_rotation(rig: str, rig_cfg: dict[str, Any], joints: tuple[float, float, float] | None) -> np.ndarray:
    """True camera link rotation (x forward). ``joints`` = (pan, tilt, roll) in rad for rig_gimbal."""
    err = rig_cfg["mount_error_deg"]
    R_err = rot_zyx(err["yaw"] * DEG, err["pitch"] * DEG, err["roll"] * DEG)
    if rig == "rig_fixed":
        R_nom = rot_zyx(rig_cfg["nominal_pan_deg"] * DEG, -rig_cfg["nominal_tilt_deg"] * DEG,
                        rig_cfg.get("nominal_roll_deg", 0.0) * DEG)
    else:
        pan, tilt, roll = joints if joints is not None else (rig_cfg["initial_pan_deg"] * DEG,
                                                             rig_cfg["initial_tilt_deg"] * DEG, 0.0)
        R_nom = rot_zyx(pan, -tilt, roll)
    return R_nom @ R_err


def lens_position(rig: str, rig_cfg: dict[str, Any], R_cam: np.ndarray) -> np.ndarray:
    base = np.array([0.0, 0.0, float(rig_cfg["lens_height_m"])])
    if rig == "rig_gimbal":
        base = base + R_cam @ np.array([float(rig_cfg["optical_centre_to_tilt_axis_m"]), 0.0, 0.0])
    return base


def tx_axis_body(station, payload_offset, lens, start_yaw_rad: float) -> np.ndarray:
    """tx_frame axis in body coordinates: the beacon->lens direction at the nominal station and start yaw."""
    beacon = np.asarray(station, float) + rot_zyx(start_yaw_rad, 0, 0) @ np.asarray(payload_offset, float)
    d = np.asarray(lens, float) - beacon
    return rot_zyx(start_yaw_rad, 0, 0).T @ (d / np.linalg.norm(d))


def compute_truth(*, t_sim: float, rig: str, rig_cfg: dict[str, Any], drone_pos, drone_quat, drone_vel,
                  payload_offset, tx_axis_b, joints=None, wind=(0, 0, 0), fan_level="off",
                  lighting="office_50hz") -> dict[str, Any]:
    R_body = quat_to_rot(drone_quat)
    p = np.asarray(drone_pos, float)
    beacon = p + R_body @ np.asarray(payload_offset, float)
    R_cam = camera_rotation(rig, rig_cfg, joints)
    lens = lens_position(rig, rig_cfg, R_cam)
    los = beacon - lens
    rng = float(np.linalg.norm(los))
    u = los / rng
    az = math.atan2(u[1], u[0])
    el = math.asin(u[2])
    d_cam = R_cam.T @ u                      # camera link frame (x fwd, y left, z up)
    bore = math.acos(max(-1.0, min(1.0, d_cam[0])))
    err_x = math.atan2(-d_cam[1], d_cam[0])  # image right
    err_y = math.atan2(-d_cam[2], d_cam[0])  # image down
    tx_axis_w = R_body @ np.asarray(tx_axis_b, float)
    to_lens = -u
    tx_err = math.acos(max(-1.0, min(1.0, float(tx_axis_w @ to_lens))))
    lean = math.acos(max(-1.0, min(1.0, float(R_body[2, 2]))))
    yaw, pitch, roll = rot_to_ypr(R_cam)
    rec = {
        "t_sim": t_sim, "rig": rig, "fan_level": fan_level, "lighting": lighting,
        "drone_pos_m": p.tolist(), "drone_quat_wxyz": list(map(float, drone_quat)),
        "drone_vel_mps": list(map(float, drone_vel)),
        "beacon_pos_m": beacon.tolist(), "lens_pos_m": lens.tolist(), "range_m": rng,
        "los_az_deg": math.degrees(az), "los_el_deg": math.degrees(el),
        "cam_yaw_deg": math.degrees(yaw), "cam_pitch_deg": math.degrees(pitch), "cam_roll_deg": math.degrees(roll),
        "gimbal_joint_pan_deg": None, "gimbal_joint_tilt_deg": None, "gimbal_joint_roll_deg": None,
        "boresight_err_mrad": 1e3 * bore, "boresight_err_x_mrad": 1e3 * err_x, "boresight_err_y_mrad": 1e3 * err_y,
        "tx_pointing_err_mrad": 1e3 * tx_err,
        "wind_mps": list(map(float, wind)),
        "lean_deg": math.degrees(lean),
    }
    if rig == "rig_gimbal" and joints is not None:
        rec["gimbal_joint_pan_deg"], rec["gimbal_joint_tilt_deg"], rec["gimbal_joint_roll_deg"] = (
            math.degrees(joints[0]), math.degrees(joints[1]), math.degrees(joints[2]))
    return rec


class TruthPublisher:  # pragma: no cover - needs Gazebo
    """Publishes truth records on the truth port (evaluation only).

    ``pose_fn()`` returns the tracked target's body pose as an object with ``t_sim``, ``position`` and
    ``orientation_wxyz`` (the drone model pose, or a static / scripted beacon), or None if unknown yet.
    ``payload_offset`` is the beacon position in that body frame.
    """

    def __init__(self, world, cfgs: dict[str, dict[str, Any]], *, rig: str, pose_fn: Callable[[], Any],
                 payload_offset, wind_fn: Callable[[], Any], fan_level_fn: Callable[[], str],
                 lighting_fn: Callable[[], str], rate_hz: float = 100.0,
                 gimbal_model: str = "gimbal_storm32_3axis", station=None):
        import zmq
        from gz.msgs10.model_pb2 import Model  # type: ignore
        self.world = world
        self.rig = rig
        self.rig_cfg = cfgs["rigs"][rig]
        self.pose_fn = pose_fn
        self.payload_offset = list(payload_offset)
        sk = cfgs["drone"]["station_keeping"]
        lens0 = lens_position(rig, self.rig_cfg, camera_rotation(rig, self.rig_cfg, None))
        self.tx_axis_b = tx_axis_body(station if station is not None else cfgs["world"]["hover_station_m"],
                                      self.payload_offset, lens0, float(sk["start_yaw_deg"]) * DEG)
        self.wind_fn, self.fan_level_fn, self.lighting_fn = wind_fn, fan_level_fn, lighting_fn
        self.rate = rate_hz
        bus = cfgs["bus"]
        self.sock = zmq.Context.instance().socket(zmq.PUB)
        self.sock.bind(f"tcp://{bus['host']}:{bus['ports']['truth']}")
        self.topic = bus["topics"]["truth"].encode()
        self._joints: tuple[float, float, float] | None = None
        self._prev: tuple[float, np.ndarray] | None = None
        self.records = 0
        from collections import deque
        self.history: deque = deque(maxlen=600)      # recent records, for the live accuracy overlay
        if rig == "rig_gimbal":
            world.subscribe(Model, f"/world/{world.world}/model/{gimbal_model}/joint_state", self._on_joints)

    def _on_joints(self, msg) -> None:
        pos = {j.name: j.axis1.position for j in msg.joint}
        self._joints = (pos.get("pan_joint", 0.0), pos.get("tilt_joint", 0.0), pos.get("roll_joint", 0.0))

    def run(self, until=None, stop=lambda: False) -> None:
        from sim_bridge.sim_loop import SimTicker
        for t in SimTicker(self.world, self.rate, until=until, stop=stop):
            rec = self.pose_fn()
            if rec is None:
                continue
            p = np.asarray(rec.position, float)
            v = np.zeros(3)
            if self._prev is not None and rec.t_sim > self._prev[0]:
                v = (p - self._prev[1]) / (rec.t_sim - self._prev[0])
            self._prev = (rec.t_sim, p)
            truth = compute_truth(t_sim=t, rig=self.rig, rig_cfg=self.rig_cfg, drone_pos=p,
                                  drone_quat=rec.orientation_wxyz, drone_vel=v, payload_offset=self.payload_offset,
                                  tx_axis_b=self.tx_axis_b, joints=self._joints, wind=self.wind_fn(),
                                  fan_level=self.fan_level_fn(), lighting=self.lighting_fn())
            self.sock.send_multipart([self.topic, json.dumps(truth).encode()])
            self.records += 1
            self.history.append(truth)
