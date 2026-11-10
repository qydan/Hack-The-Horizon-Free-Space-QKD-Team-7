"""Run one scenario: Gazebo + PX4 SITL + sim bridges + (tracking, sandbox if implemented) + recorders.

    python tools/run_scenario.py t3_turbulence --rig rig_gimbal            # headless (default)
    python tools/run_scenario.py t3_turbulence --rig rig_fixed --gui
    python tools/run_scenario.py t5_latency --duration 10 --fidelity fast  # quick smoke run
    python tools/run_scenario.py t3 --rig gimbal --view --quick            # short names, live view, 20 s
    python tools/run_scenario.py --list

Short names: any unique scenario prefix ("t3" -> t3_turbulence); rigs "fixed"/"A" and "gimbal"/"B".
--view serves the live camera at http://localhost:8080 (works from Docker too). --quick = 20 s, fast fidelity.

Everything runs on Gazebo sim time. Outputs go to runs/<scenario>_<rig>_<timestamp>/:
  world.sdf, scenario.yaml, camera_truth.yaml (copy, evaluation only), summary.json,
  telemetry.jsonl/.csv (I2), abort/keys/eve/photodiode.jsonl, frames_meta.csv, sim_events.jsonl,
  truth.jsonl/.csv (evaluation only), sample frames (frame_*.png), logs/*.log.

Environment:
  PX4_DIR            PX4-Autopilot checkout with px4/install_px4_assets.sh applied (needed when the drone flies)
  QKD_GZ_CMD         Gazebo command (default "gz sim")
"""
from __future__ import annotations

import argparse
import ast
import asyncio
import json
import logging
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sim_bridge import config as cfgmod  # noqa: E402
from tools.scenario import ScenarioError, list_scenarios, load_scenario, normalize_rig  # noqa: E402
from tools.world_builder import build_world, write_camera_truth  # noqa: E402

log = logging.getLogger("run_scenario")
DEG = math.pi / 180.0


# ----------------------------------------------------------------------------------------------- helpers
def is_open_stub(module_file: Path, func: str = "run") -> bool:
    """True if ``func`` in the module is still an OPEN placeholder (body = docstring + raise NotImplementedError)."""
    if not module_file.exists():
        return True
    tree = ast.parse(module_file.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == func:
            body = [n for n in node.body if not (isinstance(n, ast.Expr) and isinstance(getattr(n, "value", None), ast.Constant))]
            return (len(body) == 1 and isinstance(body[0], ast.Raise)
                    and "NotImplementedError" in ast.unparse(body[0]))
    return True


def schedule_value(items: list[dict[str, Any]], t: float, key: str, default=None):
    v = default
    for it in sorted(items or [], key=lambda d: float(d["t_s"])):
        if t >= float(it["t_s"]):
            v = it[key]
    return v


@dataclass
class StaticPose:
    t_sim: float
    position: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


class Proc:
    def __init__(self, name: str, cmd: list[str], log_dir: Path, env: dict[str, str] | None = None,
                 cwd: Path | None = None):
        self.name = name
        self.log_path = log_dir / f"{name}.log"
        self.fh = open(self.log_path, "w", encoding="utf-8")
        log.info("starting %s: %s", name, " ".join(cmd))
        self.p = subprocess.Popen(cmd, stdout=self.fh, stderr=subprocess.STDOUT, env={**os.environ, **(env or {})},
                                  cwd=str(cwd or ROOT), start_new_session=(os.name != "nt"))

    def alive(self) -> bool:
        return self.p.poll() is None

    def stop(self, timeout: float = 10.0) -> None:
        if self.alive():
            try:
                if os.name != "nt":
                    os.killpg(self.p.pid, signal.SIGINT)
                else:
                    self.p.send_signal(signal.CTRL_BREAK_EVENT if hasattr(signal, "CTRL_BREAK_EVENT") else signal.SIGTERM)
                self.p.wait(timeout=timeout)
            except Exception:  # noqa: BLE001
                self.p.kill()
        self.fh.close()


# ----------------------------------------------------------------------------------------------- runner
class ScenarioRun:
    def __init__(self, scn: dict[str, Any], rig: str, args: argparse.Namespace):
        self.scn = scn
        self.rig = rig
        self.args = args
        self.cfgs = cfgmod.load_all(scn.get("overrides"))
        if args.fidelity:
            scn["fidelity"] = args.fidelity
        self.fidelity = scn["fidelity"] or self.cfgs["camera_action5"]["fidelity"]["default"]
        self.duration = float(args.duration or scn["duration_s"])
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.out = Path(args.out) if args.out else ROOT / "runs" / f"{scn['name']}_{rig}_{stamp}"
        (self.out / "logs").mkdir(parents=True, exist_ok=True)
        self.procs: list[Proc] = []
        self.threads: list[threading.Thread] = []
        self.stop_evt = threading.Event()
        self.summary: dict[str, Any] = {"scenario": scn["name"], "rig": rig, "fidelity": self.fidelity,
                                        "duration_s": self.duration, "seed": scn["seed"], "warnings": []}
        self.world = None
        self.info: dict[str, Any] = {}
        self.frame_callback = None          # tests: called as cb(frame, world) for every delivered frame
        self.viewer = None                  # --view: MJPEG server

    # ------------------------------------------------------------------------------------------- setup
    def build(self) -> Path:
        sdf, info = build_world(self.cfgs, {**self.scn, "fidelity": self.fidelity}, self.rig)
        self.info = info
        path = self.out / "world.sdf"
        path.write_text(sdf)
        (self.out / "scenario.yaml").write_text(yaml.safe_dump(self.scn, sort_keys=False))
        truth = write_camera_truth(self.cfgs, self.rig)
        shutil.copy(truth, self.out / "camera_truth.yaml")
        self.summary["world"] = info
        return path

    def env(self) -> dict[str, str]:
        res = os.pathsep.join([str(ROOT / "models"), os.environ.get("GZ_SIM_RESOURCE_PATH", "")])
        return {"GZ_SIM_RESOURCE_PATH": res, "QKD_RIG": self.rig, "QKD_FIDELITY": self.fidelity,
                "PYTHONPATH": os.pathsep.join([str(ROOT), os.environ.get("PYTHONPATH", "")])}

    def start_gazebo(self, world_sdf: Path) -> None:
        cmd = os.environ.get("QKD_GZ_CMD", "gz sim").split()
        if self.args.gui:
            cmd += ["-r", str(world_sdf)]
        else:
            cmd += ["-s", "-r", "--headless-rendering", str(world_sdf)]
        if self.scn["recorders"].get("gz_log"):
            cmd += ["--record", "--record-path", str(self.out / "gz_log")]
        self.procs.append(Proc("gazebo", cmd, self.out / "logs", self.env()))

    def start_px4(self) -> None:
        px4_dir = Path(os.environ.get("PX4_DIR", Path.home() / "PX4-Autopilot"))
        binary = px4_dir / "build" / "px4_sitl_default" / "bin" / "px4"
        if not binary.exists():
            raise RuntimeError(f"PX4 SITL binary not found at {binary}. Build PX4 (make px4_sitl) and run "
                               f"px4/install_px4_assets.sh, or set PX4_DIR.")
        env = {**self.env(), "PX4_GZ_STANDALONE": "1", "PX4_GZ_MODEL_NAME": self.cfgs["drone"]["model_name"],
               "PX4_SYS_AUTOSTART": str(self.cfgs["drone"]["px4"]["airframe_id"]),
               "PX4_GZ_WORLD": self.info["world_name"], "PX4_SIM_MODEL": "gz_" + self.cfgs["drone"]["model_name"],
               "HEADLESS": "1"}
        work = self.out / "px4"
        work.mkdir(exist_ok=True)
        self.procs.append(Proc("px4", [str(binary), "-w", str(work), str(px4_dir / "build" / "px4_sitl_default" / "etc")],
                               self.out / "logs", env, cwd=work))

    def thread(self, name: str, target, *a, **k) -> None:
        def wrap():
            try:
                target(*a, **k)
            except Exception as exc:  # noqa: BLE001
                log.exception("%s failed", name)
                self.summary["warnings"].append(f"{name}: {exc!r}")
        th = threading.Thread(target=wrap, name=name, daemon=True)
        th.start()
        self.threads.append(th)

    # ------------------------------------------------------------------------------------------- bridges
    def start_bridges(self) -> None:  # pragma: no cover - needs Gazebo
        from sim_bridge.beacon_modulator import BeaconModulator
        from sim_bridge.fan_wind import FanWindBridge
        from sim_bridge.gz_io import GzWorld
        from sim_bridge.lighting import LightingController
        from sim_bridge.occluder import OccluderController

        scn, cfgs = self.scn, self.cfgs
        self.world = world = GzWorld(self.info["world_name"])
        if not world.wait_for_clock(timeout_wall_s=120):
            raise RuntimeError("Gazebo clock not received within 120 s (see logs/gazebo.log)")
        stop = self.stop_evt.is_set
        until = self.t0 + self.duration
        seed = int(scn["seed"])
        self.summary["t0_sim"] = self.t0

        # sim events (fan / lighting changes, trials) -> recorder
        import zmq
        self.events = zmq.Context.instance().socket(zmq.PUB)
        self.events.connect(f"tcp://{cfgs['bus']['host']}:{cfgs['bus']['internal_ports']['sim_events']}")
        self._events_lock = threading.Lock()

        # beacons ---------------------------------------------------------------------------------------
        drone_on = (scn["drone"] or {}).get("enabled", True) and "spawn" in self.info
        beacon_event_models = {e["model"] for e in scn.get("beacon_events") or []}
        targets: list[tuple[str, str, str]] = []          # (model, visual, target)
        if drone_on and scn["payload"] == "red_led_beacon":
            targets.append((cfgs["drone"]["model_name"], "led_visual", "red_led"))
        if drone_on and scn["payload"] == "tag":
            targets.append((cfgs["drone"]["model_name"], "tag_visual", "tag"))
        for m in (self.info.get("static_beacons") or {}):
            targets.append((m, "led_visual", "red_led"))
        for m in ("spinning_arm", "slider_beacon", "pendulum_rig"):
            if m in self.info["models"]:
                targets.append((m, "led_visual", "red_led"))
        methods = {}
        for model, vis, tgt in targets:
            if model in beacon_event_models:
                continue
            mod = BeaconModulator(world, model, cfgs["beacon"], visual_name=vis, target=tgt)
            try:
                methods[model] = mod.select_method()
                self.thread(f"beacon:{model}", mod.run, until=until, stop=stop)
            except RuntimeError as exc:
                self.summary["warnings"].append(str(exc))
        self.summary["beacon_methods"] = methods
        (self.out / "beacon_method.txt").write_text(json.dumps(methods, indent=2))
        if beacon_event_models:
            self.thread("beacon_events", self._beacon_events, world, until)

        # fan -------------------------------------------------------------------------------------------
        self.fan = None
        fan_sched = [{"t_s": self.t0 + float(i["t_s"]), "level": i["level"]} for i in scn["fan"]["schedule"]]
        if drone_on:
            self.fan = FanWindBridge(world, cfgs, model=cfgs["drone"]["model_name"], link=cfgs["fan"]["force"]["target_link"],
                                     cd_area=cfgs["drone"]["aero"]["body_cd_area_m2"], schedule=fan_sched, seed=seed + 1)
        elif "pendulum_rig" in self.info["models"]:
            self.fan = FanWindBridge(world, cfgs, model="pendulum_rig", link="pendulum_bob",
                                     cd_area=cfgs["fan"]["force"]["pendulum_beacon_cd_area_m2"], schedule=fan_sched,
                                     seed=seed + 1)
            self.fan._body_state = self._pendulum_state_fn(self.fan)
        if self.fan is not None:
            self.thread("fan_wind", self.fan.run, until=until, stop=stop)

        # lighting --------------------------------------------------------------------------------------
        lsched = [{"t_s": self.t0 + float(i["t_s"]), "scenario": i["scenario"]} for i in scn["lighting"].get("schedule") or []]
        self.lighting = LightingController(world, cfgs["lighting"], schedule=lsched, default=scn["lighting"]["initial"],
                                           shadows=bool(cfgs["world"]["render"]["shadows"]))
        self.thread("lighting", self.lighting.run, until=until, stop=stop)

        # occluder + scripted poses ----------------------------------------------------------------------
        events = [{**e, "t_s": self.t0 + float(e["t_s"])} for e in scn["occluder"]["events"]]
        tracks = {m: [{**it, "t_s": self.t0 + float(it["t_s"])} for it in tr] for m, tr in (scn.get("pose_tracks") or {}).items()}
        lens = [0.0, 0.0, cfgs["rigs"][self.rig]["lens_height_m"]]
        occ = OccluderController(world, events, lens, self.info.get("station", cfgs["world"]["hover_station_m"]),
                                 pose_tracks=tracks)
        self.thread("occluder", occ.run, until=until, stop=stop)

        # gimbal (rig_gimbal): one owner, served to other processes -------------------------------------
        if self.rig == "rig_gimbal":
            from sim_bridge.gimbal_io import Storm32GimbalServer, Storm32GimbalSim
            gcfg = cfgs["gimbal_storm32"]
            gcfg = {**gcfg, "offset_schedule": [{**o, "t_s": self.t0 + float(o["t_s"])} for o in gcfg.get("offset_schedule") or []]}
            self.gimbal = Storm32GimbalSim(gcfg, cfgs["rigs"]["rig_gimbal"], world=world, seed=seed + 2).start()
            server = Storm32GimbalServer(self.gimbal, cfgs["bus"])
            self.gimbal_server = server
            self.thread("gimbal_server", server.run, stop=stop)
            if scn["gimbal"]["command_schedule"] or scn["gimbal"]["active_schedule"]:
                self.thread("gimbal_schedule", self._gimbal_schedule, until)

        # truth -----------------------------------------------------------------------------------------
        from sim_bridge.truth_publisher import TruthPublisher
        pose_fn, offset = self._target_pose_fn()
        wind = (lambda: self.fan.last_wind.tolist()) if self.fan is not None else (lambda: [0.0, 0.0, 0.0])
        level = (lambda: self.fan.level) if self.fan is not None else (lambda: "off")
        self.truth = TruthPublisher(world, cfgs, rig=self.rig, pose_fn=pose_fn, payload_offset=offset, wind_fn=wind,
                                    fan_level_fn=level, lighting_fn=lambda: self.lighting.current or scn["lighting"]["initial"],
                                    station=self.info.get("station"))
        self.thread("truth", self.truth.run, until=until, stop=stop)

        # photodiode (optional) ---------------------------------------------------------------------------
        if scn["photodiode"].get("enabled") or cfgs["photodiode"].get("enabled"):
            self._start_photodiode(world, pose_fn, offset, until, stop, seed + 3)

    def _target_pose_fn(self):
        cfgs, scn = self.cfgs, self.scn
        world = self.world
        if "spawn" in self.info:
            name = cfgs["drone"]["model_name"]
            off = cfgs["drone"]["payload"]["variants"][scn["payload"]]["offset_m"]
            return (lambda: world.pose(name)), off
        statics = self.info.get("static_beacons") or {}
        if statics:
            tracks = scn.get("pose_tracks") or {}

            def fn():
                t = world.sim_time - self.t0
                for m, default in statics.items():
                    xyz = schedule_value(tracks.get(m, []), t, "xyz", default)
                    if xyz is not None and xyz[2] > -1.0:
                        return StaticPose(world.sim_time, tuple(xyz))
                return None
            return fn, [0.0, 0.0, 0.0]
        if "pendulum_rig" in self.info["models"]:
            return self._pendulum_pose, [0.0, 0.0, 0.0]
        if "slider_beacon" in self.info["models"] or "spinning_arm" in self.info["models"]:
            name = "slider_beacon" if "slider_beacon" in self.info["models"] else "spinning_arm"
            link = "carriage" if name == "slider_beacon" else "arm"
            base = cfgs["world"]["props"][name]["position_m"]

            def fn2():
                rec = world.pose(link)
                if rec is None:
                    return None
                p = np.asarray(base) + np.asarray(rec.position)
                if name == "spinning_arm":   # beacon at the arm tip, arm rotates about x
                    from sim_bridge.truth_publisher import quat_to_rot
                    p = np.asarray(base) + quat_to_rot(rec.orientation_wxyz) @ np.array([0, 0, cfgs["world"]["props"]["spinning_arm"]["arm_length_m"]])
                return StaticPose(rec.t_sim, tuple(p))
            return fn2, [0.0, 0.0, 0.0]
        return (lambda: None), [0.0, 0.0, 0.0]

    def _pendulum_pose(self):
        """Bob (beacon) world position: stand pose + link pose of pendulum_bob (relative to the model) +
        the string offset along the rotated link -z axis."""
        from sim_bridge.truth_publisher import quat_to_rot
        rec = self.world.pose("pendulum_bob")
        if rec is None:
            return None
        st = self.cfgs["world"]["hover_station_m"]
        R = quat_to_rot(rec.orientation_wxyz)
        p = np.array([st[0], st[1], 0.0]) + np.asarray(rec.position) + R @ np.array(
            [0, 0, -float(self.cfgs["beacon"]["pendulum"]["string_length_m"])])
        return StaticPose(rec.t_sim, tuple(p), rec.orientation_wxyz)

    def _pendulum_state_fn(self, fan):
        prev = {"v": None}

        def state():
            rec = self._pendulum_pose()
            if rec is None:
                return None
            p = np.asarray(rec.position)
            v = np.zeros(3)
            if prev["v"] is not None and rec.t_sim > prev["v"][0]:
                v = (p - prev["v"][1]) / (rec.t_sim - prev["v"][0])
            prev["v"] = (rec.t_sim, p)
            return p, v
        return state

    def _beacon_events(self, world, until) -> None:  # pragma: no cover
        rgb = self.cfgs["beacon"]["red_led"]["emissive_rgb_full"]
        events = sorted(self.scn["beacon_events"], key=lambda e: float(e["t_s"]))
        vids = {}
        for ev in events:
            t_ev = self.t0 + float(ev["t_s"])
            if t_ev > until or self.stop_evt.is_set():
                return
            world.wait_until(t_ev, timeout_wall_s=600)
            m = ev["model"]
            if m not in vids:
                vids[m] = world.find_visual_id(m, "led_visual")
            lvl = float(ev["level"])
            world.set_visual_emissive(vids[m], (rgb[0] * lvl, rgb[1] * lvl, rgb[2] * lvl, 1.0))
            self._event({"t_sim": world.sim_time, "event": "beacon_level", "model": m, "level": lvl,
                         "t_scheduled": t_ev})

    def _gimbal_schedule(self, until) -> None:  # pragma: no cover
        cmds = [{**c, "kind": "cmd"} for c in self.scn["gimbal"]["command_schedule"]]
        acts = [{**a, "kind": "active"} for a in self.scn["gimbal"]["active_schedule"]]
        for item in sorted(cmds + acts, key=lambda d: float(d["t_s"])):
            t_ev = self.t0 + float(item["t_s"])
            if t_ev > until or self.stop_evt.is_set():
                return
            self.world.wait_until(t_ev, timeout_wall_s=600)
            if item["kind"] == "cmd":
                self.gimbal.set_target(float(item["pan_deg"]) * DEG, float(item["tilt_deg"]) * DEG)
            else:
                with self.gimbal._lock:
                    self.gimbal.dyn.set_active(bool(item["active"]))
            self._event({"t_sim": self.world.sim_time, "event": f"gimbal_{item['kind']}", **item})

    def _start_photodiode(self, world, pose_fn, offset, until, stop, seed) -> None:  # pragma: no cover
        from sim_bridge.photodiode_sim import PhotodiodeSensor
        from sim_bridge.truth_publisher import camera_rotation, lens_position, quat_to_rot
        cfgs = self.cfgs
        pd = cfgs["photodiode"]

        def rx():
            joints = self.truth._joints if self.rig == "rig_gimbal" else None
            R = camera_rotation(self.rig, cfgs["rigs"][self.rig], joints)
            return lens_position(self.rig, cfgs["rigs"][self.rig], R) + R @ np.asarray(pd["receiver"]["mount_offset_m"]), R[:, 0]

        def tx():
            rec = pose_fn()
            if rec is None:
                return None
            R = quat_to_rot(rec.orientation_wxyz)
            return np.asarray(rec.position) + R @ np.asarray(offset), R @ self.truth.tx_axis_b

        sensor = PhotodiodeSensor(world, cfgs, rig=self.rig, seed=seed,
                                  lighting_name=lambda: self.lighting.current or self.scn["lighting"]["initial"],
                                  receiver_pose_fn=rx, tx_pose_fn=tx)
        self.photodiode = sensor
        self.thread("photodiode", sensor.run, until=until, stop=stop)

    def _event(self, rec: dict[str, Any]) -> None:
        with self._events_lock:
            self.events.send_multipart([b"sim_events", json.dumps(rec, default=str).encode()])

    # ------------------------------------------------------------------------------------------- drone
    def start_drone(self) -> None:  # pragma: no cover - needs PX4
        from sim_bridge.hover_station import run_station
        scn, cfgs = self.scn, self.cfgs
        d = scn["drone"]
        rng = np.random.default_rng(int(scn["seed"]) + 10)
        station = list(self.info["station"])
        schedule = []
        trials = scn["trials"]
        if d.get("randomise_station_m"):
            r = float(d["randomise_station_m"])
            tdur = float(trials.get("duration_s") or self.duration / max(1, int(trials["count"])))
            for k in range(int(trials["count"])):
                st = [station[0] + rng.uniform(-r, r), station[1] + rng.uniform(-r, r), station[2]]
                schedule.append((self.t0 + 10.0 + k * tdur, st))
            self.summary["trial_stations"] = [{"t_sim": t, "station": s} for t, s in schedule]
        self.trial_schedule = schedule
        push = None if d.get("slow_push_at_s") is None else self.t0 + float(d["slow_push_at_s"])

        def go():
            asyncio.run(run_station(cfgs["drone"], {**cfgs["world"], "world_name": self.info["world_name"]},
                                    station=station, duration_s=self.duration, wander=bool(d.get("wander", True)),
                                    push_start_s=push, seed=int(scn["seed"]) + 11, gnss=bool(d.get("gnss_wander")),
                                    world=self.world, spawn=self.info["spawn"], station_schedule=schedule,
                                    stop=self.stop_evt.is_set))
        self.thread("hover_station", go)

    # ------------------------------------------------------------------------------------------- OPEN blocks
    def start_open_blocks(self) -> None:
        env = self.env()
        py = sys.executable
        self.tracking_on = not is_open_stub(ROOT / "tracking" / "main.py", "run")
        self.sandbox_on = not is_open_stub(ROOT / "sandbox" / "main.py", "run")
        self.summary["tracking"] = "running" if self.tracking_on else "OPEN placeholder (not started)"
        self.summary["sandbox"] = "running" if self.sandbox_on else "OPEN placeholder (started under port_guard only)"
        if self.tracking_on:
            self.procs.append(Proc("tracking", [py, "-m", "tracking.main", "--rig", self.rig], self.out / "logs", env))
        allow = [str(p) for p in self.cfgs["bus"]["allow_list"]["sandbox"]]
        self.port_log = self.out / "sandbox_ports.jsonl"
        self.procs.append(Proc("sandbox", [py, "-m", "tools.port_guard", "--allow", *allow, "--log", str(self.port_log),
                                           "sandbox.main"], self.out / "logs", env))

    def start_recorders(self) -> None:
        from evaluation.truth_recorder import TruthRecorder
        from telemetry.recorder import Recorder
        stop = self.stop_evt.is_set
        if self.scn["recorders"].get("telemetry", True) or self.scn["recorders"].get("frames_meta", True):
            rec = Recorder(self.out)
            self.recorder = rec
            self.thread("recorder", rec.run, None, stop)
        if self.scn["recorders"].get("truth", True):
            tr = TruthRecorder(self.out)
            self.truth_recorder = tr
            self.thread("truth_recorder", tr.run, None, stop)

    def start_frame_pump(self, record: bool = True) -> None:  # pragma: no cover - needs Gazebo
        """Read frames in the bridge host. When tracking is not running (record=True) this keeps the Action 5
        pipeline running, writes frames_meta.csv and saves a few sample frames. It also feeds --view."""
        from sim_bridge.frame_source import GzAction5FrameSource
        src = GzAction5FrameSource(rig=self.rig, fidelity=self.fidelity, world=self.world,
                                   overrides=self.scn.get("overrides"), publish_meta=record)
        self.frame_source = src
        self.frames = 0

        def pump():
            import cv2
            save_at = {0, 30, 150, 600}
            last = None
            while not self.stop_evt.is_set():
                fr = src.read(timeout_s=2.0)
                if fr is None:
                    continue
                self.frames += 1
                if self.frame_callback is not None:
                    self.frame_callback(fr, self.world)
                if self.viewer is not None:
                    self.viewer.update(fr.image, self._overlay(fr))
                if not record:
                    continue
                if last is not None:
                    self.summary.setdefault("frame_dt", []).append(fr.t_capture_sim - last)
                last = fr.t_capture_sim
                if fr.seq in save_at:
                    cv2.imwrite(str(self.out / f"frame_{fr.seq:05d}.png"), fr.image)
            src.close()
        self.thread("frame_pump", pump)

    def _overlay(self, fr) -> list[str]:
        t = fr.t_capture_sim - getattr(self, "t0", 0.0)
        lines = [f"{self.scn['name']}  {self.rig}  {self.fidelity}   t = {t:6.2f} / {self.duration:.0f} s   frame {fr.seq}",
                 f"latency {1e3 * (fr.t_delivered_sim - fr.t_capture_sim):.0f} ms   RTF {self.world.real_time_factor:.2f}"]
        extra = []
        if getattr(self, "fan", None) is not None:
            extra.append(f"fan {self.fan.level}")
        if getattr(self, "lighting", None) is not None and self.lighting.current:
            extra.append(f"light {self.lighting.current}")
        if getattr(self, "gimbal", None) is not None:
            g = self.gimbal.read()
            extra.append(f"gimbal pan {math.degrees(g.pan_rad):.2f} tilt {math.degrees(g.tilt_rad):.2f} deg")
        if extra:
            lines.append("   ".join(extra))
        return lines

    def start_viewer(self) -> None:
        from tools.mjpeg_server import MjpegServer
        self.viewer = MjpegServer(port=int(self.args.view)).start()
        msg = f"Live camera: http://localhost:{self.viewer.port}"
        log.info(msg)
        print(f"\n  >>> {msg}  <<<\n", flush=True)

    # ------------------------------------------------------------------------------------------- main loop
    def run(self) -> dict[str, Any]:
        world_sdf = self.build()
        if self.args.build_only:
            log.info("world written to %s", world_sdf)
            return self.summary
        try:
            self.start_recorders()
            self.start_gazebo(world_sdf)
            from sim_bridge.gz_io import GzWorld
            probe = GzWorld(self.info["world_name"])
            if not probe.wait_for_clock(timeout_wall_s=120):
                raise RuntimeError("Gazebo did not start (no clock); see logs/gazebo.log")
            self.t0 = probe.sim_time
            self.start_bridges()
            if "spawn" in self.info:
                self.start_px4()
                self.start_drone()
            if getattr(self.args, "view", None):
                self.start_viewer()
            self.start_open_blocks()
            if not self.tracking_on:
                self.start_frame_pump()
            elif self.viewer is not None:
                self.start_frame_pump(record=False)
            t_wall0 = time.monotonic()
            until = self.t0 + self.duration
            last_print = 0.0
            trial_starts = [t for t, _ in getattr(self, "trial_schedule", [])]
            while self.world.sim_time < until:
                time.sleep(0.5)
                if trial_starts and self.world.sim_time >= trial_starts[0]:
                    trial_starts.pop(0)
                    self._event({"t_sim": self.world.sim_time, "event": "trial_start"})
                    if self.tracking_on:            # T2: tracker restarts in SEARCH for every trial
                        old = next(p for p in self.procs if p.name.startswith("tracking"))
                        old.stop()
                        self.procs.remove(old)
                        k = len(self.summary.setdefault("tracker_restarts", []))
                        self.summary["tracker_restarts"].append(self.world.sim_time)
                        self.procs.append(Proc(f"tracking_{k + 1}", [sys.executable, "-m", "tracking.main", "--rig",
                                                                     self.rig], self.out / "logs", self.env()))
                if not self.procs[0].alive():
                    raise RuntimeError("Gazebo exited early; see logs/gazebo.log")
                if time.monotonic() - last_print > 5.0:
                    last_print = time.monotonic()
                    log.info("t_sim=%.1f / %.1f s  RTF=%.3f", self.world.sim_time - self.t0, self.duration,
                             self.world.real_time_factor)
            wall = time.monotonic() - t_wall0
            self.summary["wall_s"] = wall
            self.summary["rtf_mean"] = self.duration / wall if wall > 0 else float("nan")
            self.summary["rtf_last_reported"] = self.world.real_time_factor
        finally:
            self.shutdown()
        return self.summary

    def shutdown(self) -> None:
        self.stop_evt.set()
        for p in reversed(self.procs):
            p.stop()
        for th in self.threads:
            th.join(timeout=5.0)
        if hasattr(self, "gimbal"):
            self.gimbal.stop()
        if self.viewer is not None:
            self.viewer.stop()
        # release every bound port so the next run in this process can bind again
        socks = [getattr(getattr(self, "truth", None), "sock", None), getattr(self, "events", None),
                 getattr(getattr(self, "gimbal_server", None), "pub", None),
                 getattr(getattr(self, "gimbal_server", None), "sub", None),
                 getattr(getattr(self, "frame_source", None), "_meta_sock", None),
                 getattr(getattr(self, "photodiode", None), "sock", None)]
        for sock in socks:
            if sock is not None:
                sock.close(linger=0)
        s = self.summary
        if hasattr(self, "frames"):
            s["frames_read"] = self.frames
            dts = s.pop("frame_dt", [])
            if dts:
                s["fps_sim_mean"] = 1.0 / float(np.mean(dts))
        if hasattr(self, "lighting"):
            s["lighting_update_rate_hz"] = self.lighting.achieved_rate_hz
            (self.out / "lighting_rate.txt").write_text(f"{self.lighting.achieved_rate_hz}\n")
        if hasattr(self, "truth"):
            s["truth_records"] = self.truth.records
        if hasattr(self, "port_log") and self.port_log.exists():
            evs = [json.loads(line) for line in self.port_log.read_text().splitlines() if line.strip()]
            s["sandbox_port_violations"] = [e for e in evs if not e["allowed"]]
        if hasattr(self, "recorder"):
            s["recorded"] = dict(self.recorder.counts)
        (self.out / "summary.json").write_text(json.dumps(s, indent=2, default=str))
        log.info("run directory: %s", self.out)
        print(json.dumps({k: v for k, v in s.items() if k not in ("world",)}, indent=2, default=str))


def run(scenario: str, rig: str, *, duration: float | None = None, fidelity: str | None = None,
        out: str | Path | None = None, patch: dict[str, Any] | None = None, frame_callback=None,
        gui: bool = False) -> tuple[dict[str, Any], Path]:
    """Programmatic entry point (used by tests): returns (summary, run directory)."""
    from tools.scenario import _merge, validate
    scn = load_scenario(scenario)
    if patch:
        scn = _merge(scn, patch)
        validate(scn, cfgmod.load_all(scn.get("overrides")))
    ns = argparse.Namespace(fidelity=fidelity, duration=duration, out=str(out) if out else None, gui=gui,
                            build_only=False, view=None)
    r = ScenarioRun(scn, rig, ns)
    r.frame_callback = frame_callback
    summary = r.run()
    return summary, r.out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", nargs="?")
    ap.add_argument("--rig", default=None,
                    help="fixed|gimbal (or A|B, rig_fixed|rig_gimbal); default: every rig the scenario lists")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--duration", type=float, default=None, help="override duration_s (sim seconds)")
    ap.add_argument("--fidelity", choices=["full", "fast"], default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--build-only", action="store_true", help="only write world.sdf and scenario.yaml")
    ap.add_argument("--view", nargs="?", const=8080, default=None, type=int, metavar="PORT",
                    help="serve the live camera at http://localhost:PORT (default 8080)")
    ap.add_argument("--quick", action="store_true", help="20 s of sim time at fast fidelity (unless overridden)")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()
    if args.quick:
        args.duration = args.duration or 20.0
        args.fidelity = args.fidelity or "fast"
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    if args.list or not args.scenario:
        for p in list_scenarios():
            try:
                s = load_scenario(str(p))
                print(f"{p.stem:24s} {str(s['test']):12s} rigs={','.join(s['rigs']):22s} {s['description']}")
            except ScenarioError as exc:
                print(f"{p.stem:24s} INVALID: {exc}")
        return
    try:
        scn = load_scenario(args.scenario)
        rigs = [normalize_rig(args.rig)] if args.rig else scn["rigs"]
    except ScenarioError as exc:
        print(f"Scenario rejected: {exc}", file=sys.stderr)
        sys.exit(2)
    for rig in rigs:
        if rig not in scn["rigs"]:
            print(f"Scenario {scn['name']} does not support {rig} (rigs: {scn['rigs']})", file=sys.stderr)
            sys.exit(2)
        ScenarioRun(dict(scn), rig, args).run()


if __name__ == "__main__":
    main()
    # The gz-transport Python binding can abort at interpreter shutdown (pybind11 GIL check while tearing
    # down subscriptions). Everything is written by now, so leave without running the teardown.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)
