"""Thin wrapper around gz-transport 13 / gz-msgs 10 (Gazebo Harmonic) Python bindings.

All gz imports are confined to this module so the rest of sim_bridge stays importable (and unit-testable)
on machines without Gazebo. Call :func:`gz_available` before using anything here.

Services and topics used (Gazebo Harmonic, gz-sim 8):
  /world/<w>/clock                    gz.msgs.Clock         sim time
  /world/<w>/dynamic_pose/info        gz.msgs.Pose_V        model/link poses
  /world/<w>/pose/info                gz.msgs.Pose_V        all poses (incl. static) at a lower rate
  /world/<w>/scene/info   (service)   Empty -> Scene        visual ids for material updates
  /world/<w>/visual_config (service)  Visual -> Boolean     runtime material change (beacon modulation)
  /world/<w>/light_config  (service)  Light -> Boolean      lighting scenarios and mains flicker
  /world/<w>/set_pose      (service)  Pose -> Boolean       scripted occluder / checkerboard / targets
  /world/<w>/create        (service)  EntityFactory -> Boolean
  /world/<w>/wrench/persistent        gz.msgs.EntityWrench  ApplyLinkWrench system (fan force)
  /world/<w>/wrench/clear             gz.msgs.Entity
  /world/<w>/wrench                   gz.msgs.EntityWrench  one-step wrench (fallback)
  /world/<w>/control       (service)  WorldControl -> Boolean   pause / step
"""
from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Callable

_IMPORT_ERROR: Exception | None = None
try:  # pragma: no cover - only available inside the Gazebo image
    from gz.transport13 import Node, SubscribeOptions  # type: ignore
    from gz.msgs10.boolean_pb2 import Boolean  # type: ignore
    from gz.msgs10.clock_pb2 import Clock  # type: ignore
    from gz.msgs10.double_pb2 import Double  # type: ignore
    from gz.msgs10.empty_pb2 import Empty  # type: ignore
    from gz.msgs10.entity_pb2 import Entity  # type: ignore
    from gz.msgs10.entity_factory_pb2 import EntityFactory  # type: ignore
    from gz.msgs10.entity_wrench_pb2 import EntityWrench  # type: ignore
    from gz.msgs10.image_pb2 import Image  # type: ignore
    from gz.msgs10.light_pb2 import Light  # type: ignore
    from gz.msgs10.pose_pb2 import Pose  # type: ignore
    from gz.msgs10.pose_v_pb2 import Pose_V  # type: ignore
    from gz.msgs10.scene_pb2 import Scene  # type: ignore
    from gz.msgs10.visual_pb2 import Visual  # type: ignore
    from gz.msgs10.world_control_pb2 import WorldControl  # type: ignore
    from gz.msgs10.world_stats_pb2 import WorldStatistics  # type: ignore
except Exception as exc:  # noqa: BLE001
    _IMPORT_ERROR = exc


def gz_available() -> bool:
    return _IMPORT_ERROR is None


def require_gz() -> None:
    if _IMPORT_ERROR is not None:
        raise RuntimeError(
            "Gazebo Harmonic Python bindings (python3-gz-transport13, python3-gz-msgs10) are not importable: "
            f"{_IMPORT_ERROR!r}. Run inside the docker image (docker/run.sh)."
        )


def _throttle(msgs_per_sec: float):
    opts = SubscribeOptions()
    opts.msgs_per_sec = int(msgs_per_sec)
    return opts


def stamp_to_s(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nsec) * 1e-9


@dataclass
class PoseRecord:
    t_sim: float
    position: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float]


class GzWorld:
    """Connection to one running Gazebo world.

    gz-transport delivers each node's messages and service replies on that node's own thread, so one busy
    subscription (the 30 Hz, 6 MB camera images) delays everything else on the node. To keep things
    independent: clock/poses/stats share ``self.node``; every :meth:`subscribe` gets its own node; publishers
    share one node; service requests go through a helper process (sim_bridge/gz_service_proxy.py).
    """

    def __init__(self, world_name: str):
        require_gz()
        self.world = world_name
        self.node = Node()
        self._pub_node = Node()
        self._sub_nodes: list[Any] = []
        self._lock = threading.Lock()
        self._sim_time = 0.0
        self._clock_cv = threading.Condition(self._lock)
        self._poses: dict[str, PoseRecord] = {}
        self._pubs: dict[tuple[str, Any], Any] = {}
        self._rtf = float("nan")
        from sim_bridge import config as _cfg
        rates = _cfg.load("world").get("bridge", {})
        self.node.subscribe(Clock, f"/world/{world_name}/clock", self._on_clock,
                            _throttle(rates.get("clock_rate_hz", 250.0)))
        self.node.subscribe(Pose_V, f"/world/{world_name}/dynamic_pose/info", self._on_poses,
                            _throttle(rates.get("pose_rate_hz", 200.0)))
        self.node.subscribe(WorldStatistics, f"/world/{world_name}/stats", self._on_stats)

    # ------------------------------------------------------------------ time
    def _on_clock(self, msg: "Clock") -> None:
        with self._clock_cv:
            self._sim_time = stamp_to_s(msg.sim)
            self._clock_cv.notify_all()

    def _on_stats(self, msg: "WorldStatistics") -> None:
        self._rtf = float(msg.real_time_factor)

    @property
    def sim_time(self) -> float:
        with self._lock:
            return self._sim_time

    @property
    def real_time_factor(self) -> float:
        return self._rtf

    def wait_until(self, t_sim: float, timeout_wall_s: float = 30.0) -> bool:
        """Block (wall clock) until sim time reaches ``t_sim``."""
        with self._clock_cv:
            return self._clock_cv.wait_for(lambda: self._sim_time >= t_sim, timeout=timeout_wall_s)

    def wait_for_clock(self, timeout_wall_s: float = 60.0) -> bool:
        with self._clock_cv:
            return self._clock_cv.wait_for(lambda: self._sim_time > 0.0, timeout=timeout_wall_s)

    # ----------------------------------------------------------------- poses
    def _on_poses(self, msg: "Pose_V") -> None:
        t = stamp_to_s(msg.header.stamp)
        with self._lock:
            for p in msg.pose:
                self._poses[p.name] = PoseRecord(
                    t, (p.position.x, p.position.y, p.position.z),
                    (p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z))

    def pose(self, name: str) -> PoseRecord | None:
        with self._lock:
            return self._poses.get(name)

    def subscribe(self, msg_type: Any, topic: str, cb: Callable[[Any], None]) -> None:
        node = Node()
        if not node.subscribe(msg_type, topic, cb):
            raise RuntimeError(f"could not subscribe to {topic}")
        self._sub_nodes.append(node)

    # ------------------------------------------------------------- publishing
    def publisher(self, topic: str, msg_type: Any):
        key = (topic, msg_type)
        if key not in self._pubs:
            self._pubs[key] = self._pub_node.advertise(topic, msg_type)
        return self._pubs[key]

    def publish_double(self, topic: str, value: float) -> None:
        msg = Double()
        msg.data = float(value)
        self.publisher(topic, Double).publish(msg)

    def request(self, service: str, req: Any, req_type: Any, rep_type: Any, timeout_ms: int = 1000):
        """Service request, made through :mod:`sim_bridge.gz_service_proxy` (see there for why)."""
        return _proxy_request(service, req, req_type, rep_type, timeout_ms)

    # --------------------------------------------------------------- services
    def set_pose(self, name: str, xyz, quat_wxyz=(1.0, 0.0, 0.0, 0.0), timeout_ms: int = 200) -> bool:
        req = Pose()
        req.name = name
        req.position.x, req.position.y, req.position.z = map(float, xyz)
        req.orientation.w, req.orientation.x, req.orientation.y, req.orientation.z = map(float, quat_wxyz)
        ok, rep = self.request(f"/world/{self.world}/set_pose", req, Pose, Boolean, timeout_ms)
        return bool(ok and rep.data)

    def create_from_sdf(self, sdf: str, name: str | None = None, timeout_ms: int = 5000) -> bool:
        req = EntityFactory()
        req.sdf = sdf
        if name:
            req.name = name
        ok, rep = self.request(f"/world/{self.world}/create", req, EntityFactory, Boolean, timeout_ms)
        return bool(ok and rep.data)

    def scene(self, timeout_ms: int = 5000):
        ok, rep = self.request(f"/world/{self.world}/scene/info", Empty(), Empty, Scene, timeout_ms)
        return rep if ok else None

    def find_visual_id(self, model_name: str, visual_name: str) -> int | None:
        """Find a visual entity id by model name and visual name (searches nested models and links)."""
        scene = self.scene()
        if scene is None:
            return None

        def walk_model(m) -> int | None:
            for link in m.link:
                for v in link.visual:
                    if v.name == visual_name or v.name.endswith("::" + visual_name):
                        return int(v.id)
            for sub in m.model:
                r = walk_model(sub)
                if r is not None:
                    return r
            return None

        for m in scene.model:
            if m.name == model_name:
                return walk_model(m)
        return None

    def set_visual_emissive(self, visual_id: int, rgba, timeout_ms: int = 100) -> bool:
        req = Visual()
        req.id = int(visual_id)
        e = req.material.emissive
        e.r, e.g, e.b, e.a = map(float, rgba)
        ok, rep = self.request(f"/world/{self.world}/visual_config", req, Visual, Boolean, timeout_ms)
        return bool(ok and rep.data)

    def set_light(self, name: str, *, diffuse=None, specular=None, intensity=None, cast_shadows=None,
                  light_type: str = "point", direction=None, pose_xyz=None, attenuation_range=None,
                  timeout_ms: int = 100) -> bool:
        req = Light()
        req.name = name
        req.type = {"point": Light.POINT, "spot": Light.SPOT, "directional": Light.DIRECTIONAL}[light_type]
        if diffuse is not None:
            req.diffuse.r, req.diffuse.g, req.diffuse.b = map(float, diffuse[:3])
            req.diffuse.a = 1.0
        if specular is not None:
            req.specular.r, req.specular.g, req.specular.b = map(float, specular[:3])
            req.specular.a = 1.0
        if intensity is not None:
            req.intensity = float(intensity)
        if cast_shadows is not None:
            req.cast_shadows = bool(cast_shadows)
        if direction is not None:
            req.direction.x, req.direction.y, req.direction.z = map(float, direction)
        if pose_xyz is not None:
            req.pose.position.x, req.pose.position.y, req.pose.position.z = map(float, pose_xyz)
        if attenuation_range is not None:
            req.range = float(attenuation_range)
        ok, rep = self.request(f"/world/{self.world}/light_config", req, Light, Boolean, timeout_ms)
        return bool(ok and rep.data)

    def set_light_spec(self, spec: dict[str, Any], timeout_ms: int = 100) -> bool:
        """Send a complete light (see sim_bridge.lighting.light_specs): light_config replaces every field."""
        req = Light()
        req.name = spec["name"]
        req.type = {"point": Light.POINT, "spot": Light.SPOT, "directional": Light.DIRECTIONAL}[spec["type"]]
        req.diffuse.r, req.diffuse.g, req.diffuse.b = map(float, spec["diffuse"])
        req.diffuse.a = 1.0
        req.specular.r, req.specular.g, req.specular.b = map(float, spec.get("specular", (0, 0, 0)))
        req.specular.a = 1.0
        req.intensity = float(spec["intensity"])
        req.cast_shadows = bool(spec.get("cast_shadows", False))
        if "position" in spec:
            req.pose.position.x, req.pose.position.y, req.pose.position.z = map(float, spec["position"])
            req.pose.orientation.w = 1.0
        if "direction" in spec:
            req.direction.x, req.direction.y, req.direction.z = map(float, spec["direction"])
        if "range" in spec:
            req.range = float(spec["range"])
        att = spec.get("attenuation")
        if att:
            req.attenuation_constant = float(att["constant"])
            req.attenuation_linear = float(att["linear"])
            req.attenuation_quadratic = float(att["quadratic"])
        sp = spec.get("spot")
        if sp:
            req.spot_inner_angle = float(sp["inner"])
            req.spot_outer_angle = float(sp["outer"])
            req.spot_falloff = float(sp["falloff"])
        ok, rep = self.request(f"/world/{self.world}/light_config", req, Light, Boolean, timeout_ms)
        return bool(ok and rep.data)

    # ----------------------------------------------------------------- wrench
    def _link_entity(self, model: str, link: str) -> "Entity":
        ent = Entity()
        ent.name = f"{model}::{link}"
        ent.type = Entity.LINK
        return ent

    def clear_persistent_wrench(self, model: str, link: str) -> None:
        self.publisher(f"/world/{self.world}/wrench/clear", Entity).publish(self._link_entity(model, link))

    def set_persistent_wrench(self, model: str, link: str, force_xyz, torque_xyz=(0.0, 0.0, 0.0)) -> None:
        """Replace the persistent wrench on a link.

        ApplyLinkWrench keeps a *list* of persistent wrenches, so each update first clears the link's
        wrenches and then adds the new one. Both are handled in the same PreUpdate (verified by test A4).
        """
        self.clear_persistent_wrench(model, link)
        msg = EntityWrench()
        msg.entity.CopyFrom(self._link_entity(model, link))
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = map(float, force_xyz)
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = map(float, torque_xyz)
        self.publisher(f"/world/{self.world}/wrench/persistent", EntityWrench).publish(msg)

    def apply_step_wrench(self, model: str, link: str, force_xyz) -> None:
        """One-physics-step wrench (fallback: must be called every step)."""
        msg = EntityWrench()
        msg.entity.CopyFrom(self._link_entity(model, link))
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = map(float, force_xyz)
        self.publisher(f"/world/{self.world}/wrench", EntityWrench).publish(msg)

    # ----------------------------------------------------------------- control
    def pause(self, paused: bool = True) -> bool:
        req = WorldControl()
        req.pause = paused
        ok, rep = self.request(f"/world/{self.world}/control", req, WorldControl, Boolean, 2000)
        return bool(ok and rep.data)


# ------------------------------------------------------------------------------------------ service proxy
_proxy_lock = threading.Lock()
_proxy_procs: list[Any] = []
_proxy_local = threading.local()


def _proxy_port() -> int:
    """One helper process per calling thread, so e.g. the 250 Hz lighting flicker never queues behind the
    beacon modulator or the occluder."""
    port = getattr(_proxy_local, "port", None)
    if port is None:
        import atexit
        import subprocess
        import sys
        proc = subprocess.Popen([sys.executable, "-m", "sim_bridge.gz_service_proxy"], stdout=subprocess.PIPE, text=True)
        line = proc.stdout.readline().strip()
        if not line.startswith("PORT "):
            raise RuntimeError(f"gz service proxy failed to start: {line!r}")
        port = _proxy_local.port = int(line.split()[1])
        with _proxy_lock:
            if not _proxy_procs:
                atexit.register(lambda: [p.kill() for p in _proxy_procs])
            _proxy_procs.append(proc)
    return port


def _proxy_request(service: str, req: Any, req_type: Any, rep_type: Any, timeout_ms: int):
    import zmq
    port = _proxy_port()
    sock = getattr(_proxy_local, "sock", None)
    if sock is None:
        sock = zmq.Context.instance().socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(f"tcp://127.0.0.1:{port}")
        _proxy_local.sock = sock

    def spec(t):                                    # gz/msgs/visual.proto -> gz.msgs10.visual_pb2:Visual
        stem = t.DESCRIPTOR.file.name.rsplit("/", 1)[-1].removesuffix(".proto")
        return f"gz.msgs10.{stem}_pb2:{t.DESCRIPTOR.name}".encode()

    sock.send_multipart([service.encode(), spec(req_type), spec(rep_type), req.SerializeToString(),
                         str(int(timeout_ms)).encode()])
    if not sock.poll(int(timeout_ms) + 2000):
        sock.close()                                # a REQ socket without a reply is stuck; start over
        _proxy_local.sock = None
        return False, rep_type()
    ok, payload = sock.recv_multipart()
    return ok == b"1", (rep_type.FromString(payload) if ok == b"1" else rep_type())


def image_msg_type():
    require_gz()
    return Image


def pixel_format_rgb8() -> int:
    require_gz()
    from gz.msgs10.image_pb2 import PixelFormatType  # type: ignore  # noqa: WPS433
    return PixelFormatType.RGB_INT8
