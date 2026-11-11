"""Generate every model SDF and world SDF from config/*.yaml, so no number is duplicated in an asset.

    python tools/world_builder.py --write-assets            # (re)write models/*/ and worlds/*.sdf defaults
    python tools/world_builder.py --scenario scenarios/t3_turbulence.yaml --rig rig_gimbal --out runs/x/world.sdf

``tools/run_scenario.py`` calls :func:`build_world` for each run (world SDF goes into the run directory),
and :func:`write_camera_truth` writes evaluation/camera_truth.yaml (evaluation only).

Frames: world ENU, origin on the floor under the camera mount, +x toward the drone. SDF poses are
"x y z roll pitch yaw" with R = Rz(yaw) Ry(pitch) Rx(roll); pitch positive = nose DOWN. Camera links look
along their +x axis (Gazebo convention).
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sim_bridge import config as cfgmod  # noqa: E402
from sim_bridge.camera_model import make_lens, render_sensor_type, rot_zyx  # noqa: E402
from sim_bridge.image_pipeline import Action5Pipeline  # noqa: E402
from sim_bridge.truth_publisher import rot_to_ypr  # noqa: E402
from tools.compute_inertia import inertia, rotor_positions  # noqa: E402

DEG = math.pi / 180.0
PARK_Z = -20.0


# ------------------------------------------------------------------------------------------------ helpers
def f(x: float) -> str:
    return f"{float(x):.6g}"


def pose(xyz=(0, 0, 0), rpy=(0, 0, 0)) -> str:
    return f"<pose>{' '.join(f(v) for v in (*xyz, *rpy))}</pose>"


def rgba(c, a: float = 1.0) -> str:
    c = list(c)
    return " ".join(f(v) for v in (c + [a])[:4]) if len(c) == 3 else " ".join(f(v) for v in c)


def material(diffuse=(0.7, 0.7, 0.7), emissive=None, albedo_map=None, emissive_map=None, specular=0.1) -> str:
    s = (f"<material><ambient>{rgba(diffuse)}</ambient><diffuse>{rgba(diffuse)}</diffuse>"
         f"<specular>{rgba((specular,) * 3)}</specular>")
    if emissive is not None:
        s += f"<emissive>{rgba(emissive)}</emissive>"
    if albedo_map or emissive_map:
        s += "<pbr><metal>"
        if albedo_map:
            s += f"<albedo_map>{albedo_map}</albedo_map>"
        if emissive_map:
            s += f"<emissive_map>{emissive_map}</emissive_map>"
        s += "<roughness>0.9</roughness><metalness>0.0</metalness></metal></pbr>"
    return s + "</material>"


def box_geom(size) -> str:
    return f"<geometry><box><size>{' '.join(f(v) for v in size)}</size></box></geometry>"


def cyl_geom(radius, length) -> str:
    return f"<geometry><cylinder><radius>{f(radius)}</radius><length>{f(length)}</length></cylinder></geometry>"


def sph_geom(radius) -> str:
    return f"<geometry><sphere><radius>{f(radius)}</radius></sphere></geometry>"


def visual(name, geom, mat, xyz=(0, 0, 0), rpy=(0, 0, 0), cast_shadows=True) -> str:
    cs = "" if cast_shadows else "<cast_shadows>false</cast_shadows>"
    return f"<visual name='{name}'>{pose(xyz, rpy)}{geom}{mat}{cs}</visual>"


def collision(name, geom, xyz=(0, 0, 0), rpy=(0, 0, 0)) -> str:
    return f"<collision name='{name}'>{pose(xyz, rpy)}{geom}</collision>"


def inertial(mass, ixx, iyy, izz, ixy=0.0, ixz=0.0, iyz=0.0, xyz=(0, 0, 0)) -> str:
    return (f"<inertial>{pose(xyz)}<mass>{f(mass)}</mass><inertia><ixx>{f(ixx)}</ixx><ixy>{f(ixy)}</ixy>"
            f"<ixz>{f(ixz)}</ixz><iyy>{f(iyy)}</iyy><iyz>{f(iyz)}</iyz><izz>{f(izz)}</izz></inertia></inertial>")


def box_inertial(mass, size, xyz=(0, 0, 0)) -> str:
    x, y, z = size
    return inertial(mass, mass / 12 * (y * y + z * z), mass / 12 * (x * x + z * z), mass / 12 * (x * x + y * y), xyz=xyz)


def static_model(name, xyz, links: str, rpy=(0, 0, 0)) -> str:
    return f"<model name='{name}'><static>true</static>{pose(xyz, rpy)}{links}</model>"


def emissive_rgb_level(rgb, level=1.0):
    return [c * level for c in rgb]


# ------------------------------------------------------------------------------------------------ camera
def camera_sensor_sdf(cam: dict[str, Any], fidelity: str) -> str:
    r = cam["render"]
    rate = Action5Pipeline.required_render_rate_hz(cam, fidelity)
    noise = (f"<noise><type>gaussian</type><mean>0</mean><stddev>{f(r['noise_stddev'])}</stddev></noise>")
    clip = f"<clip><near>{f(r['clip_near_m'])}</near><far>{f(r['clip_far_m'])}</far></clip>"
    st = render_sensor_type(cam)
    if st == "camera":                       # rectilinear (pinhole) lens: one ordinary render per frame
        w, h = cam["stream"]["width_px"], cam["stream"]["height_px"]
        body = (f"<horizontal_fov>{f(float(cam['lens']['hfov_deg']) * DEG)}</horizontal_fov>"
                f"<image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>{clip}{noise}")
        stype = "camera"
    elif st == "wideanglecamera":
        w, h = cam["stream"]["width_px"], cam["stream"]["height_px"]
        hfov = float(cam["lens"]["hfov_deg"]) * DEG
        body = (f"<horizontal_fov>{f(hfov)}</horizontal_fov>"
                f"<image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>{clip}{noise}"
                f"<lens><type>equidistant</type><scale_to_hfov>true</scale_to_hfov>"
                f"<cutoff_angle>{f(math.pi / 2)}</cutoff_angle>"
                f"<env_texture_size>{int(r['env_texture_size_px'])}</env_texture_size></lens>")
        stype = "wideanglecamera"
    elif st == "rectilinear_remap":
        body = (f"<horizontal_fov>{f(float(r['rectilinear_hfov_deg']) * DEG)}</horizontal_fov>"
                f"<image><width>{int(r['rectilinear_width_px'])}</width><height>{int(r['rectilinear_height_px'])}"
                f"</height><format>R8G8B8</format></image>{clip}{noise}")
        stype = "camera"
    else:
        raise ValueError(f"unknown render.sensor_type {st!r}")
    return (f"<sensor name='action5_camera' type='{stype}'><always_on>true</always_on>"
            f"<update_rate>{f(rate)}</update_rate><visualize>false</visualize>"
            f"<topic>{cam['topics']['image']}</topic><camera name='action5'>{body}</camera></sensor>")


def camera_link_sdf(cam: dict[str, Any], fidelity: str, xyz, rpy, *, ring_light: dict | None, beacon: dict) -> str:
    """Action 5 Pro body + sensor (+ optional ring light) as a link named camera_link."""
    L, W, H = cam["body"]["size_lwh_m"]
    m = float(cam["body"]["mass_kg"])
    vis = visual("body", box_geom((L, W, H)), material((0.08, 0.08, 0.09)), xyz=(-L / 2, 0, 0))
    vis += visual("lens", cyl_geom(0.011, 0.006), material((0.02, 0.02, 0.03), specular=0.6),
                  xyz=(0.003, 0, 0), rpy=(0, math.pi / 2, 0))
    ring = ""
    if ring_light is not None:
        rl = beacon["ring_light"]
        r_mid = 0.25 * (float(rl["inner_diameter_m"]) + float(rl["outer_diameter_m"]))
        seg = 12
        for k in range(seg):
            a = 2 * math.pi * k / seg
            vis += visual(f"ring_seg_{k}", box_geom((0.004, 0.012, 0.012)),
                          material((0.9, 0.9, 0.9), emissive=(1, 1, 1)), xyz=(0.006, r_mid * math.cos(a), r_mid * math.sin(a)),
                          cast_shadows=False)
        on = 1.0 if rl.get("power_on", True) else 0.0
        ring = (f"<light name='ring_light_spot' type='spot'>{pose((0.01, 0, 0), (0, math.pi / 2, 0))}"
                f"<diffuse>{rgba((on, on, on))}</diffuse><specular>0.2 0.2 0.2 1</specular>"
                f"<intensity>{f(on)}</intensity><direction>0 0 -1</direction>"
                f"<attenuation><range>10</range><linear>0.05</linear><constant>0.5</constant><quadratic>0.01</quadratic></attenuation>"
                f"<spot><inner_angle>0.4</inner_angle><outer_angle>0.8</outer_angle><falloff>1</falloff></spot>"
                f"<cast_shadows>false</cast_shadows></light>")
    return (f"<link name='camera_link'>{pose(xyz, rpy)}{box_inertial(m, (L, W, H))}<gravity>false</gravity>"
            f"{vis}{ring}{camera_sensor_sdf(cam, fidelity)}</link>")


# -------------------------------------------------------------------------------------------------- rigs
def _err_rot(rig_cfg) -> np.ndarray:
    e = rig_cfg["mount_error_deg"]
    return rot_zyx(e["yaw"] * DEG, e["pitch"] * DEG, e["roll"] * DEG)


def tripod_visuals(height: float, spread: float, prefix="tripod") -> str:
    v = ""
    for k in range(3):
        a = 2 * math.pi * k / 3 + math.pi / 3     # legs at 60/180/300 deg: none points into the view
        dx, dy = spread * math.cos(a), spread * math.sin(a)
        length = math.sqrt(dx * dx + dy * dy + height * height)
        tilt = math.atan2(spread, height)
        v += visual(f"{prefix}_leg_{k}", cyl_geom(0.01, length), material((0.15, 0.15, 0.15)),
                    xyz=(dx / 2, dy / 2, height / 2), rpy=(0, -tilt, a + math.pi))
    return v


def rig_fixed_model(cfgs, fidelity: str, *, ring_light: bool) -> str:
    rig = cfgs["rigs"]["rig_fixed"]
    cam = cfgs["camera_action5"]
    h = float(rig["lens_height_m"])
    R = rot_zyx(rig["nominal_pan_deg"] * DEG, -rig["nominal_tilt_deg"] * DEG, rig.get("nominal_roll_deg", 0) * DEG)
    R = R @ _err_rot(rig)
    yaw, pitch, roll = rot_to_ypr(R)
    spread = cfgs["world"]["props"]["tripod"]["leg_spread_m"]
    head_z = h - 0.05
    base = (f"<link name='tripod'>{box_inertial(1.5, (0.1, 0.1, 0.1), (0, 0, 0.5))}"
            f"{tripod_visuals(head_z - 0.03, spread)}"
            f"{visual('ball_head', sph_geom(0.025), material((0.1, 0.1, 0.1)), xyz=(0, 0, head_z))}"
            f"{collision('col', box_geom((0.1, 0.1, head_z)), xyz=(0, 0, head_z / 2))}</link>")
    camlink = camera_link_sdf(cam, fidelity, (0, 0, h), (roll, pitch, yaw), ring_light={} if ring_light else None,
                              beacon=cfgs["beacon"])
    joints = ("<joint name='world_fix' type='fixed'><parent>world</parent><child>tripod</child></joint>"
              "<joint name='camera_fix' type='fixed'><parent>tripod</parent><child>camera_link</child></joint>")
    return f"<model name='mount_fixed'>{pose()}{base}{camlink}{joints}</model>"


def rig_gimbal_model(cfgs, fidelity: str, *, ring_light: bool) -> str:
    rig = cfgs["rigs"]["rig_gimbal"]
    gim = cfgs["gimbal_storm32"]
    cam = cfgs["camera_action5"]
    jh = rig["joint_heights_m"]
    jc = gim["joint_control"]
    spread = cfgs["world"]["props"]["tripod"]["leg_spread_m"]
    m_g = float(gim["mass_kg"])
    pan0, tilt0 = rig["initial_pan_deg"] * DEG, rig["initial_tilt_deg"] * DEG
    pl, tl = gim["limits_deg"]["pan"], gim["limits_deg"]["tilt"]
    yaw_e, pitch_e, roll_e = rot_to_ypr(_err_rot(rig))
    dz_roll = jh["roll_axis"] - jh["pan_base"]
    dz_tilt = jh["tilt_axis"] - jh["roll_axis"]
    tripod = (f"<link name='tripod'>{box_inertial(1.5, (0.1, 0.1, 0.1), (0, 0, 0.4))}"
              f"{tripod_visuals(jh['pan_base'] - 0.02, spread)}"
              f"{collision('col', box_geom((0.1, 0.1, jh['pan_base'])), xyz=(0, 0, jh['pan_base'] / 2))}</link>")
    pan = (f"<link name='pan_link'>{pose((0, 0, jh['pan_base']))}<gravity>false</gravity>"
           f"{box_inertial(m_g * 0.4, (0.05, 0.05, 0.06), (0, 0, 0.03))}"
           f"{visual('pan_motor', cyl_geom(0.022, 0.02), material((0.05, 0.05, 0.05)), xyz=(0, 0, 0.01))}"
           f"{visual('pan_arm', box_geom((0.012, 0.02, dz_roll)), material((0.2, 0.2, 0.22)), xyz=(-0.045, 0, dz_roll / 2))}"
           f"</link>")
    roll = (f"<link name='roll_link'>{pose((0, 0, jh['roll_axis']))}<gravity>false</gravity>"
            f"{box_inertial(m_g * 0.3, (0.04, 0.04, 0.04))}"
            f"{visual('roll_motor', cyl_geom(0.02, 0.018), material((0.05, 0.05, 0.05)), xyz=(-0.045, 0, 0), rpy=(0, math.pi / 2, 0))}"
            f"{visual('roll_arm', box_geom((0.012, 0.09, 0.012)), material((0.2, 0.2, 0.22)), xyz=(-0.04, 0, dz_tilt / 2))}"
            f"</link>")
    tilt = (f"<link name='tilt_link'>{pose((0, 0, jh['tilt_axis']))}<gravity>false</gravity>"
            f"{box_inertial(m_g * 0.3, (0.04, 0.08, 0.02))}"
            f"{visual('tilt_motor', cyl_geom(0.02, 0.018), material((0.05, 0.05, 0.05)), xyz=(0, 0.048, 0), rpy=(math.pi / 2, 0, 0))}"
            f"{visual('plate', box_geom((0.06, 0.08, 0.004)), material((0.25, 0.25, 0.27)), xyz=(-0.01, 0, -0.025))}"
            f"</link>")
    camlink = camera_link_sdf(cam, fidelity, (rig["optical_centre_to_tilt_axis_m"], 0, jh["tilt_axis"]),
                              (roll_e, pitch_e, yaw_e), ring_light={} if ring_light else None, beacon=cfgs["beacon"])

    def rjoint(name, parent, child, axis, lo, hi):
        return (f"<joint name='{name}' type='revolute'><parent>{parent}</parent><child>{child}</child>"
                f"<axis><xyz>{axis}</xyz><limit><lower>{f(lo)}</lower><upper>{f(hi)}</upper></limit>"
                f"<dynamics><damping>0.0</damping></dynamics></axis></joint>")

    def ctrl(joint, topic, init):
        return (f"<plugin filename='gz-sim-joint-position-controller-system' "
                f"name='gz::sim::systems::JointPositionController'><joint_name>{joint}</joint_name>"
                f"<topic>{topic}</topic><p_gain>{f(jc['p_gain'])}</p_gain><i_gain>{f(jc['i_gain'])}</i_gain>"
                f"<d_gain>{f(jc['d_gain'])}</d_gain><cmd_max>{f(jc['cmd_max'])}</cmd_max><cmd_min>{f(-jc['cmd_max'])}</cmd_min>"
                f"<use_velocity_commands>{str(bool(jc['use_velocity_commands'])).lower()}</use_velocity_commands>"
                f"<initial_position>{f(init)}</initial_position></plugin>")

    joints = ("<joint name='world_fix' type='fixed'><parent>world</parent><child>tripod</child></joint>"
              + rjoint("pan_joint", "tripod", "pan_link", "0 0 1", pl[0] * DEG, pl[1] * DEG)
              + rjoint("roll_joint", "pan_link", "roll_link", "1 0 0", -0.5, 0.5)
              + rjoint("tilt_joint", "roll_link", "tilt_link", "0 -1 0", tl[0] * DEG, tl[1] * DEG)
              + "<joint name='camera_fix' type='fixed'><parent>tilt_link</parent><child>camera_link</child></joint>")
    plugins = (ctrl("pan_joint", jc["topics"]["pan"], pan0) + ctrl("roll_joint", jc["topics"]["roll"], 0.0)
               + ctrl("tilt_joint", jc["topics"]["tilt"], tilt0)
               + "<plugin filename='gz-sim-joint-state-publisher-system' name='gz::sim::systems::JointStatePublisher'/>")
    return f"<model name='gimbal_storm32_3axis'>{pose()}{tripod}{pan}{roll}{tilt}{camlink}{joints}{plugins}</model>"


# ------------------------------------------------------------------------------------------------- drone
IMU_NOISE = ("<noise type='gaussian'><mean>0</mean><stddev>0.0003394</stddev><dynamic_bias_stddev>3.8785e-05"
             "</dynamic_bias_stddev><dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time></noise>")
ACC_NOISE = ("<noise type='gaussian'><mean>0</mean><stddev>0.004</stddev><dynamic_bias_stddev>0.006"
             "</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise>")
PX4_SENSORS = (  # copied from PX4 x500_base (sensor parameters unchanged)
    "<sensor name='air_pressure_sensor' type='air_pressure'><always_on>1</always_on><update_rate>50</update_rate>"
    "<air_pressure><pressure><noise type='gaussian'><mean>0</mean><stddev>0.01</stddev></noise></pressure>"
    "</air_pressure></sensor>"
    "<sensor name='magnetometer_sensor' type='magnetometer'><always_on>1</always_on><update_rate>100</update_rate>"
    "<magnetometer><x><noise type='gaussian'><stddev>0.0001</stddev></noise></x>"
    "<y><noise type='gaussian'><stddev>0.0001</stddev></noise></y>"
    "<z><noise type='gaussian'><stddev>0.0001</stddev></noise></z></magnetometer></sensor>"
    "<sensor name='imu_sensor' type='imu'><always_on>1</always_on><update_rate>250</update_rate><imu>"
    f"<angular_velocity><x>{IMU_NOISE}</x><y>{IMU_NOISE}</y><z>{IMU_NOISE}</z></angular_velocity>"
    f"<linear_acceleration><x>{ACC_NOISE}</x><y>{ACC_NOISE}</y><z>{ACC_NOISE}</z></linear_acceleration>"
    "</imu></sensor>"
    "<sensor name='navsat_sensor' type='navsat'><always_on>1</always_on><update_rate>30</update_rate></sensor>")


def payload_visuals(cfgs, payload: str, target_scale: float = 1.0, beacon_method: str = "visual_config") -> tuple[str, str, str]:
    """Return (visuals on base_link, extra links, extra joints/plugins) for the payload variant."""
    b = cfgs["beacon"]
    pv = cfgs["drone"]["payload"]["variants"][payload]
    off = pv["offset_m"]
    top = 0.5 * float(cfgs["drone"]["spec"]["folded_lwh_m"][2])       # top shell height above body centre
    if payload == "red_led_beacon":
        r = 0.5 * float(b["red_led"]["render_diameter_m"]) * target_scale
        em = b["red_led"]["emissive_rgb_full"]
        post = max(off[2] - top - r, 0.001)
        mount = visual("beacon_post", cyl_geom(0.0015, post), material((0.15, 0.15, 0.15)),
                       xyz=(off[0], off[1], top + post / 2))
        if beacon_method != "shell_toggle":
            return (mount + visual("led_visual", sph_geom(r), material(b["red_led"]["diffuse_rgb"], emissive=em),
                                   xyz=off, cast_shadows=False), "", "")
        # fallback 3: two co-located spheres, the inactive one pulled down into an opaque shell
        lo = b["red_led"]["modulation"]["low_level"]
        links, joints = "", ""
        for name, level in (("hi", 1.0), ("lo", lo)):
            links += (f"<link name='beacon_{name}'>{pose(off)}{inertial(1e-4, 1e-8, 1e-8, 1e-8)}"
                      f"{visual('led_visual_' + name, sph_geom(r * (1.0 if name == 'hi' else 0.995)), material(b['red_led']['diffuse_rgb'], emissive=emissive_rgb_level(em, level)), cast_shadows=False)}</link>")
            joints += (f"<joint name='slider_{name}' type='prismatic'><parent>base_link</parent><child>beacon_{name}</child>"
                       f"<axis><xyz>0 0 1</xyz><limit><lower>-0.05</lower><upper>0</upper></limit></axis></joint>"
                       f"<plugin filename='gz-sim-joint-position-controller-system' name='gz::sim::systems::JointPositionController'>"
                       f"<joint_name>slider_{name}</joint_name><topic>/model/{cfgs['drone']['model_name']}/slider_{name}/cmd_pos</topic>"
                       f"<p_gain>1000</p_gain><use_velocity_commands>true</use_velocity_commands><cmd_max>100</cmd_max><cmd_min>-100</cmd_min>"
                       f"<initial_position>{'0' if name == 'hi' else '-0.045'}</initial_position></plugin>")
        shell = visual("beacon_shell", cyl_geom(r * 1.15, 2.2 * r), material((0.1, 0.1, 0.1)),
                       xyz=(off[0], off[1], off[2] - 2.25 * r))
        return mount + shell, links, joints
    if payload == "tag":
        t = b["tag"]
        edge = (float(t["outer_edge_m"]) + 2 * float(t["white_margin_m"])) * target_scale
        tex = f"model://tag_aruco/materials/textures/aruco_{t['dictionary']}_{t['id']}.png"
        w = float(t["emissive_white_full"])
        # plate normal along -x (rear face, toward the camera at start yaw 0)
        return (visual("tag_visual", f"<geometry><plane><normal>-1 0 0</normal><size>{f(edge)} {f(edge)}</size></plane></geometry>",
                       material((1, 1, 1), emissive=(w, w, w), albedo_map=tex, emissive_map=tex), xyz=off,
                       cast_shadows=False), "", "")
    return "", "", ""


def drone_model(cfgs, payload: str, spawn_xyz, *, yaw_enu_deg: float, target_scale: float = 1.0,
                beacon_method: str = "visual_config") -> str:
    d = cfgs["drone"]
    name = d["model_name"]
    L, W, H = d["spec"]["folded_lwh_m"]
    inr = inertia(d, payload)
    I = inr["I"]
    mot = d["motor"]
    g = d["geometry"]
    prop_r = float(g["prop_radius_m"])
    rotors = rotor_positions(d)
    dirs = ["ccw", "ccw", "cw", "cw"]                 # x500 convention
    pv, plinks, pjoints = payload_visuals(cfgs, payload, target_scale, beacon_method)
    vis = visual("body", box_geom((L, W, H)), material((0.55, 0.57, 0.6)))
    vis += visual("camera_pod", box_geom((0.02, 0.03, 0.02)), material((0.1, 0.1, 0.1)), xyz=(L / 2 + 0.01, 0, -0.015))
    for k, (x, y, z) in enumerate(rotors):
        arm_len = math.hypot(x, y)
        vis += visual(f"arm_{k}", box_geom((arm_len, 0.012, 0.01)), material((0.5, 0.52, 0.55)),
                      xyz=(x / 2, y / 2, 0.0), rpy=(0, 0, math.atan2(y, x)))
        vis += visual(f"motor_{k}", cyl_geom(0.009, 0.016), material((0.15, 0.15, 0.15)), xyz=(x, y, z - 0.008))
    px4 = d.get("flight_model", "px4") == "px4"
    # simple flight model: the pose is written by sim_bridge/simple_flight.py, so no gravity, sensors or motors
    nograv = "" if px4 else "<gravity>false</gravity>"
    base = (f"<link name='base_link'>{nograv}{inertial(inr['mass_kg'], I[0][0], I[1][1], I[2][2], I[0][1], I[0][2], I[1][2], xyz=inr['com_m'])}"
            f"{collision('base_collision', box_geom((L, W, H)))}{vis}{pv}{PX4_SENSORS if px4 else ''}</link>")
    links, joints, plugins = "", "", ""
    mr = float(g["rotor_link_mass_kg"])
    for k, (x, y, z) in enumerate(rotors):
        links += (f"<link name='rotor_{k}'>{pose((x, y, z))}{nograv}"
                  f"{inertial(mr, mr * prop_r ** 2 / 12, mr * prop_r ** 2 / 12, mr * prop_r ** 2 / 6)}"
                  f"{visual('prop', cyl_geom(prop_r, g['prop_visual_thickness_m']), material((0.85, 0.85, 0.85)), cast_shadows=False)}"
                  f"</link>")
        if not px4:
            joints += (f"<joint name='rotor_{k}_joint' type='fixed'><parent>base_link</parent><child>rotor_{k}</child></joint>")
            continue
        joints += (f"<joint name='rotor_{k}_joint' type='revolute'><parent>base_link</parent><child>rotor_{k}</child>"
                   f"<axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>")
        plugins += (f"<plugin filename='gz-sim-multicopter-motor-model-system' name='gz::sim::systems::MulticopterMotorModel'>"
                    f"<jointName>rotor_{k}_joint</jointName><linkName>rotor_{k}</linkName>"
                    f"<turningDirection>{dirs[k]}</turningDirection>"
                    f"<timeConstantUp>{f(mot['time_constant_up_s'])}</timeConstantUp>"
                    f"<timeConstantDown>{f(mot['time_constant_down_s'])}</timeConstantDown>"
                    f"<maxRotVelocity>{f(mot['max_rot_velocity'])}</maxRotVelocity>"
                    f"<motorConstant>{f(mot['motor_constant'])}</motorConstant>"
                    f"<momentConstant>{f(mot['moment_constant'])}</momentConstant>"
                    f"<commandSubTopic>command/motor_speed</commandSubTopic><motorNumber>{k}</motorNumber>"
                    f"<rotorDragCoefficient>{f(mot['rotor_drag_coefficient'])}</rotorDragCoefficient>"
                    f"<rollingMomentCoefficient>{f(mot['rolling_moment_coefficient'])}</rollingMomentCoefficient>"
                    f"<rotorVelocitySlowdownSim>{int(mot['rotor_velocity_slowdown_sim'])}</rotorVelocitySlowdownSim>"
                    f"<motorType>velocity</motorType></plugin>")
    if px4:
        plugins += ("<plugin filename='gz-sim-odometry-publisher-system' name='gz::sim::systems::OdometryPublisher'>"
                    "<dimensions>3</dimensions></plugin>")
    return (f"<model name='{name}'>{pose(spawn_xyz, (0, 0, yaw_enu_deg * DEG))}<self_collide>false</self_collide>"
            f"{base}{links}{plinks}{joints}{pjoints}{plugins}</model>")


# ------------------------------------------------------------------------------------------------- props
def beacon_on_stand(cfgs, name: str, xyz, scale: float = 1.0) -> str:
    b = cfgs["beacon"]["red_led"]
    r = 0.5 * float(b["render_diameter_m"]) * scale
    pole = cfgs["world"]["props"]["stand"]["pole_diameter_m"]
    z = float(xyz[2])
    links = (f"<link name='link'>"
             f"{visual('pole', cyl_geom(pole / 2, z - r), material((0.2, 0.2, 0.2)), xyz=(0, 0, -(z - r) / 2 - r))}"
             f"{visual('led_visual', sph_geom(r), material(b['diffuse_rgb'], emissive=b['emissive_rgb_full']), cast_shadows=False)}"
             f"</link>")
    return static_model(name, xyz, links)


def checkerboard_model(cfgs, xyz) -> str:
    c = cfgs["world"]["props"]["checkerboard"]
    nx, ny = c["inner_corners"]
    sq, mg = float(c["square_m"]), float(c["margin_m"])
    w, h = (nx + 1) * sq + 2 * mg, (ny + 1) * sq + 2 * mg
    tex = "model://checkerboard/materials/textures/checkerboard.png"
    links = (f"<link name='link'>{visual('board', f'<geometry><plane><normal>-1 0 0</normal><size>{f(h)} {f(w)}</size></plane></geometry>', material((1, 1, 1), albedo_map=tex))}"
             f"{visual('backing', box_geom((0.005, w, h)), material((0.3, 0.3, 0.3)), xyz=(0.004, 0, 0))}</link>")
    return static_model("checkerboard", xyz, links)


def occluder_model(cfgs) -> str:
    w, h = cfgs["world"]["props"]["occluder"]["size_m"]
    return static_model("occluder", (-3.0, 2.5, -10.0),
                        f"<link name='link'>{visual('visual', box_geom((0.01, w, h)), material((0.75, 0.55, 0.45)))}</link>")


def fan_model(cfgs) -> str:
    wf = cfgs["world"]["fan"]
    D = float(cfgs["fan"]["diameter_m"])
    fv = cfgs["world"]["props"]["desk_fan_visual"]
    ap = cfgs["world"]["appearance"]
    x, y, z = wf["position_m"]
    links = (f"<link name='link'>"
             f"{visual('head', cyl_geom(D / 2, fv['head_depth_m']), material(ap['fan_body_rgb']), xyz=(0, -fv['head_depth_m'] / 2, 0), rpy=(-math.pi / 2, 0, 0))}"
             f"{visual('grille', cyl_geom(D / 2 * 0.95, 0.005), material(ap['fan_grille_rgb']), xyz=(0, 0.002, 0), rpy=(-math.pi / 2, 0, 0))}"
             f"{visual('pole', cyl_geom(0.015, z), material(ap['fan_stand_rgb']), xyz=(0, -fv['head_depth_m'], -z / 2))}"
             f"{visual('base', cyl_geom(fv['base_diameter_m'] / 2, 0.03), material(ap['fan_stand_rgb']), xyz=(0, -fv['head_depth_m'], -z + 0.015))}"
             f"</link>")
    yaw = math.atan2(wf["axis"][1], wf["axis"][0]) - math.pi / 2     # model +y = blowing direction
    return static_model("desk_fan", (x, y, z), links, rpy=(0, 0, yaw))


def distractor_models(cfgs, show: bool) -> str:
    d = cfgs["lighting"]["distractors"]
    out = ""

    def place(p):
        return p if show else (p[0], p[1], PARK_Z)

    s = d["steady_red_lamp"]
    out += static_model("steady_red_lamp", place(s["position_m"]),
                        f"<link name='link'>{visual('visual', sph_geom(s['diameter_m'] / 2), material((0.9, 0.3, 0.3), emissive=s['emissive_rgb']), cast_shadows=False)}</link>")
    bl = d["blinking_red_indicator"]
    out += static_model("blinking_red_indicator", place(bl["position_m"]),
                        f"<link name='link'>{visual('visual', sph_geom(bl['diameter_m'] / 2), material((0.6, 0.1, 0.1), emissive=bl['emissive_rgb']), cast_shadows=False)}</link>")
    for i, g in enumerate(d["glints"]):
        out += static_model(f"glint_{i}", place(g["position_m"]),
                            f"<link name='link'>{visual('visual', box_geom((0.002, *g['size_m'])), material((1, 1, 1), emissive=g['emissive']), cast_shadows=False)}</link>")
    return out


def window_plane_model(cfgs, show: bool) -> str:
    wp = cfgs["lighting"]["scenarios"]["window"]["window_plane"]
    c = wp["centre_m"]
    xyz = c if show else (c[0], c[1], PARK_Z)
    return static_model("window_plane", xyz,
                        f"<link name='link'>{visual('visual', box_geom((0.01, *wp['size_m'])), material((1, 1, 1), emissive=wp['emissive']), cast_shadows=False)}</link>")


def spinning_arm_model(cfgs) -> str:
    a = cfgs["world"]["props"]["spinning_arm"]
    b = cfgs["beacon"]["red_led"]
    L = float(a["arm_length_m"])
    r = 0.5 * float(b["render_diameter_m"])
    hub = f"<link name='hub'>{inertial(1.0, 0.01, 0.01, 0.01)}{visual('stand', cyl_geom(0.015, a['position_m'][2]), material((0.2, 0.2, 0.2)), xyz=(0.05, 0, -a['position_m'][2] / 2))}</link>"
    arm = (f"<link name='arm'><gravity>false</gravity>{inertial(0.05, 1e-3, 1e-3, 1e-3)}"
           f"{visual('bar', box_geom((0.01, 0.01, L)), material((0.15, 0.15, 0.15)), xyz=(0, 0, L / 2))}"
           f"{visual('led_visual', sph_geom(r), material(b['diffuse_rgb'], emissive=b['emissive_rgb_full']), xyz=(0, 0, L), cast_shadows=False)}</link>")
    j = ("<joint name='fix' type='fixed'><parent>world</parent><child>hub</child></joint>"
         "<joint name='arm_joint' type='revolute'><parent>hub</parent><child>arm</child><axis><xyz>1 0 0</xyz></axis></joint>"
         f"<plugin filename='gz-sim-joint-controller-system' name='gz::sim::systems::JointController'>"
         f"<joint_name>arm_joint</joint_name><initial_velocity>{f(a['speed_rad_s'])}</initial_velocity></plugin>")
    return f"<model name='spinning_arm'>{pose(a['position_m'])}{hub}{arm}{j}</model>"


def slider_beacon_model(cfgs) -> str:
    s = cfgs["world"]["props"]["slider_beacon"]
    b = cfgs["beacon"]["red_led"]
    r = 0.5 * float(b["render_diameter_m"])
    rail = f"<link name='rail'>{inertial(1.0, 0.01, 0.01, 0.01)}</link>"
    car = (f"<link name='carriage'><gravity>false</gravity>{inertial(0.05, 1e-4, 1e-4, 1e-4)}"
           f"{visual('led_visual', sph_geom(r), material(b['diffuse_rgb'], emissive=b['emissive_rgb_full']), cast_shadows=False)}</link>")
    j = ("<joint name='fix' type='fixed'><parent>world</parent><child>rail</child></joint>"
         f"<joint name='slide' type='prismatic'><parent>rail</parent><child>carriage</child><axis><xyz>0 1 0</xyz>"
         f"<limit><lower>0</lower><upper>{f(s['travel_m'])}</upper></limit></axis></joint>"
         f"<plugin filename='gz-sim-joint-controller-system' name='gz::sim::systems::JointController'>"
         f"<joint_name>slide</joint_name><initial_velocity>{f(s['speed_mps'])}</initial_velocity></plugin>")
    return f"<model name='slider_beacon'>{pose(s['position_m'])}{rail}{car}{j}</model>"


def pendulum_model(cfgs) -> str:
    p = cfgs["beacon"]["pendulum"]
    b = cfgs["beacon"]["red_led"]
    st = cfgs["world"]["hover_station_m"]
    H = float(p["stand_height_m"])
    Ls = float(p["string_length_m"])
    r = 0.5 * float(b["render_diameter_m"])
    damp = f(p["joint_damping_nms_per_rad"])
    m = float(cfgs["drone"]["payload"]["variants"]["red_led_beacon"]["mass_kg"])
    stand = (f"<link name='stand'>{inertial(5.0, 0.5, 0.5, 0.5)}"
             f"{visual('pole', cyl_geom(0.02, H), material((0.3, 0.3, 0.3)), xyz=(-0.4, 0, H / 2))}"
             f"{visual('boom', box_geom((0.42, 0.03, 0.03)), material((0.3, 0.3, 0.3)), xyz=(-0.2, 0, H))}</link>")
    gimbal = f"<link name='swing_x'>{pose((0, 0, H))}{inertial(1e-4, 1e-8, 1e-8, 1e-8)}</link>"
    i_s = 0.4 * m * r * r                                   # solid sphere about its own centre
    bob = (f"<link name='pendulum_bob'>{pose((0, 0, H))}{inertial(m, i_s, i_s, i_s, xyz=(0, 0, -Ls))}"
           f"{visual('string', cyl_geom(0.0005, Ls), material((0.9, 0.9, 0.9)), xyz=(0, 0, -Ls / 2))}"
           f"{visual('led_visual', sph_geom(r), material(b['diffuse_rgb'], emissive=b['emissive_rgb_full']), xyz=(0, 0, -Ls), cast_shadows=False)}</link>")
    j = ("<joint name='fix' type='fixed'><parent>world</parent><child>stand</child></joint>"
         f"<joint name='swing_x_joint' type='revolute'><parent>stand</parent><child>swing_x</child><axis><xyz>1 0 0</xyz><dynamics><damping>{damp}</damping></dynamics></axis></joint>"
         f"<joint name='swing_y_joint' type='revolute'><parent>swing_x</parent><child>pendulum_bob</child><axis><xyz>0 1 0</xyz><dynamics><damping>{damp}</damping></dynamics></axis></joint>")
    return f"<model name='pendulum_rig'>{pose((st[0], st[1], 0.0))}{stand}{gimbal}{bob}{j}</model>"


# ------------------------------------------------------------------------------------------------ room/world
def room_models(cfgs) -> str:
    r = cfgs["world"]["room"]
    sx, sy, sz = r["size_xyz_m"]
    cx, cy = r["centre_xy_m"]
    ap = cfgs["world"]["appearance"]
    wall = material(ap["walls_rgb"], specular=ap["wall_specular"])
    ceiling = material(ap["ceiling_rgb"], specular=ap["wall_specular"])
    t = 0.05
    floor_tex = "model://qkd_textures/floor_checker_noise.png"
    links = (f"<link name='floor'>{collision('c', f'<geometry><plane><normal>0 0 1</normal><size>{f(sx)} {f(sy)}</size></plane></geometry>')}"
             f"{visual('v', f'<geometry><plane><normal>0 0 1</normal><size>{f(sx)} {f(sy)}</size></plane></geometry>', material((1, 1, 1), albedo_map=floor_tex, specular=0.02))}</link>")
    walls = [("wall_back", (cx + sx / 2, cy, sz / 2), (t, sy, sz)), ("wall_front", (cx - sx / 2, cy, sz / 2), (t, sy, sz)),
             ("wall_left", (cx, cy + sy / 2, sz / 2), (sx, t, sz)), ("wall_right", (cx, cy - sy / 2, sz / 2), (sx, t, sz)),
             ("ceiling", (cx, cy, sz), (sx, sy, t))]
    for name, xyz, size in walls:
        mat = ceiling if name == "ceiling" else wall
        links += f"<link name='{name}'>{pose(xyz)}{collision('c', box_geom(size))}{visual('v', box_geom(size), mat)}</link>"
    return static_model("room", (0, 0, 0), links)


def light_sdf(spec: dict[str, Any]) -> str:
    out = f"<light type='{spec['type']}' name='{spec['name']}'>"
    if "position" in spec:
        out += pose(spec["position"])
    out += (f"<cast_shadows>{str(bool(spec.get('cast_shadows'))).lower()}</cast_shadows>"
            f"<diffuse>{rgba(spec['diffuse'])}</diffuse><specular>{rgba(spec.get('specular', (0, 0, 0)))}</specular>"
            f"<intensity>{f(spec['intensity'])}</intensity>")
    if "direction" in spec:
        out += f"<direction>{' '.join(f(v) for v in spec['direction'])}</direction>"
    if "range" in spec:
        att = spec["attenuation"]
        out += (f"<attenuation><range>{f(spec['range'])}</range><constant>{f(att['constant'])}</constant>"
                f"<linear>{f(att['linear'])}</linear><quadratic>{f(att['quadratic'])}</quadratic></attenuation>")
    if "spot" in spec:
        sp = spec["spot"]
        out += (f"<spot><inner_angle>{f(sp['inner'])}</inner_angle><outer_angle>{f(sp['outer'])}</outer_angle>"
                f"<falloff>{f(sp['falloff'])}</falloff></spot>")
    return out + "</light>"


def lights_sdf(cfgs, scenario_name: str, *, indoor: bool) -> str:
    from sim_bridge.lighting import light_specs
    specs = light_specs(cfgs["lighting"], scenario_name, indoor=indoor,
                        shadows=bool(cfgs["world"]["render"]["shadows"]))
    return "".join(light_sdf(sp) for sp in specs)


def world_header(cfgs, name: str) -> str:
    w = cfgs["world"]
    ph = w["physics"]
    sc = w["spherical_coordinates"]
    from sim_bridge.lighting import SCENE_AMBIENT
    a = SCENE_AMBIENT
    return (f"<?xml version='1.0'?>\n<sdf version='1.9'><world name='{name}'>"
            f"<physics name='1ms' type='ignored'><max_step_size>{f(ph['max_step_size_s'])}</max_step_size>"
            f"<real_time_factor>{f(ph['real_time_factor'])}</real_time_factor>"
            f"<real_time_update_rate>{f(1.0 / ph['max_step_size_s'] * ph['real_time_factor'])}</real_time_update_rate></physics>"
            f"<plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'><engine><filename>"
            f"gz-physics-{ph['engine']}-plugin</filename></engine></plugin>"
            "<plugin filename='gz-sim-user-commands-system' name='gz::sim::systems::UserCommands'/>"
            "<plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>"
            "<plugin filename='gz-sim-contact-system' name='gz::sim::systems::Contact'/>"
            "<plugin filename='gz-sim-imu-system' name='gz::sim::systems::Imu'/>"
            "<plugin filename='gz-sim-air-pressure-system' name='gz::sim::systems::AirPressure'/>"
            "<plugin filename='gz-sim-magnetometer-system' name='gz::sim::systems::Magnetometer'/>"
            "<plugin filename='gz-sim-navsat-system' name='gz::sim::systems::NavSat'/>"
            "<plugin filename='gz-sim-apply-link-wrench-system' name='gz::sim::systems::ApplyLinkWrench'/>"
            "<plugin filename='gz-sim-sensors-system' name='gz::sim::systems::Sensors'><render_engine>ogre2</render_engine></plugin>"
            f"<scene><ambient>{a} {a} {a} 1</ambient><background>0.05 0.05 0.06 1</background><shadows>{str(bool(w['render']['shadows'])).lower()}</shadows>"
            "<grid>false</grid></scene>"
            "<gravity>0 0 -9.81</gravity><magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>"
            "<atmosphere type='adiabatic'/>"
            f"<spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU"
            f"</world_frame_orientation><latitude_deg>{sc['latitude_deg']}</latitude_deg><longitude_deg>"
            f"{sc['longitude_deg']}</longitude_deg><elevation>{sc['elevation_m']}</elevation><heading_deg>0</heading_deg>"
            "</spherical_coordinates>")


def field_ground(cfgs) -> str:
    s = float(cfgs["world"]["mission"]["ground_size_m"])
    g = f"<geometry><plane><normal>0 0 1</normal><size>{f(s)} {f(s)}</size></plane></geometry>"
    return static_model("ground", (0, 0, 0), f"<link name='link'>{collision('c', g)}{visual('v', g, material((0.35, 0.42, 0.28), specular=0.02))}</link>")


# ---------------------------------------------------------------------------------------------- top level
DEFAULT_SCENARIO: dict[str, Any] = {
    "world": "qkd_lab", "rig": "rig_fixed", "payload": "red_led_beacon", "target": "red_led",
    "fidelity": None, "lighting": {"initial": "office_50hz"}, "drone": {"enabled": True},
    "props": {}, "station": None,
}


def build_world(cfgs: dict[str, dict[str, Any]], scenario: dict[str, Any], rig: str, *,
                beacon_method: str = "visual_config") -> tuple[str, dict[str, Any]]:
    """Return (world SDF text, info dict with world_name, spawn, models present)."""
    sc = {**DEFAULT_SCENARIO, **scenario}
    kind = sc["world"]
    fidelity = sc.get("fidelity") or cfgs["camera_action5"]["fidelity"]["default"]
    props = sc.get("props") or {}
    lighting0 = (sc.get("lighting") or {}).get("initial", cfgs["lighting"]["default"])
    indoor = kind in ("qkd_lab", "qkd_pendulum")
    name = {"qkd_lab": cfgs["world"]["world_name"], "qkd_pendulum": cfgs["world"]["pendulum_world"]["world_name"],
            "qkd_field": cfgs["world"]["mission"]["world_name"]}[kind]
    ring = sc.get("target") == "tag"
    parts = [world_header(cfgs, name), lights_sdf(cfgs, lighting0, indoor=indoor)]
    if indoor:
        parts.append(room_models(cfgs))
    else:
        parts.append(field_ground(cfgs))
    parts.append(rig_fixed_model(cfgs, fidelity, ring_light=ring) if rig == "rig_fixed"
                 else rig_gimbal_model(cfgs, fidelity, ring_light=ring))
    info: dict[str, Any] = {"world_name": name, "rig": rig, "fidelity": fidelity, "models": []}
    if kind == "qkd_lab":
        parts.append(fan_model(cfgs))
        parts.append(occluder_model(cfgs))
        cb = cfgs["world"]["checkerboard"]
        parts.append(checkerboard_model(cfgs, cb["shown_position_m"] if props.get("checkerboard") else cb["hidden_position_m"]))
        ldist = any((it.get("scenario") == "distractors") for it in (sc.get("lighting") or {}).get("schedule", [])) \
            or lighting0 == "distractors"
        parts.append(distractor_models(cfgs, show=lighting0 == "distractors"))
        parts.append(window_plane_model(cfgs, show=lighting0 == "window"))
        info["models"] += ["desk_fan", "occluder", "checkerboard", "window_plane", "distractors" if ldist else ""]
        if props.get("static_beacons"):
            names = ["static_beacon_near", "static_beacon_far"]
            for nm, xyz in zip(names, cfgs["world"]["static_beacons_m"]):
                parts.append(beacon_on_stand(cfgs, nm, xyz))
            info["static_beacons"] = dict(zip(names, cfgs["world"]["static_beacons_m"]))
        if props.get("spinning_arm"):
            parts.append(spinning_arm_model(cfgs))
            info["models"].append("spinning_arm")
        if props.get("slider_beacon"):
            parts.append(slider_beacon_model(cfgs))
            info["models"].append("slider_beacon")
    if kind == "qkd_pendulum":
        parts.append(fan_model(cfgs))
        parts.append(pendulum_model(cfgs))
        info["models"].append("pendulum_rig")
    drone_on = (sc.get("drone") or {}).get("enabled", True) and kind != "qkd_pendulum"
    if drone_on:
        sk = cfgs["drone"]["station_keeping"]
        station = (cfgs["world"]["mission"]["hover_station_m"] if kind == "qkd_field"
                   else cfgs["world"]["hover_station_m"])
        spawn = [station[0] + sk["spawn_offset_m"][0], station[1] + sk["spawn_offset_m"][1], sk["spawn_offset_m"][2]]
        scale = float(cfgs["world"]["mission"]["beacon_scale"]) if kind == "qkd_field" else 1.0
        parts.append(drone_model(cfgs, sc["payload"], spawn, yaw_enu_deg=float(sk["start_yaw_deg"]),
                                 target_scale=scale, beacon_method=beacon_method))
        info["spawn"] = spawn
        info["station"] = station
        info["target_scale"] = scale
        info["models"].append(cfgs["drone"]["model_name"])
    parts.append("</world></sdf>\n")
    return "".join(parts), info


# ---------------------------------------------------------------------------------------------- assets
def write_textures(cfgs) -> None:
    import cv2
    from tools.make_tag_texture import make_tag_texture
    make_tag_texture(cfgs["beacon"])
    c = cfgs["world"]["props"]["checkerboard"]
    nx, ny = c["inner_corners"]
    ppsq = int(c["texture_px_per_square"])
    mg = int(round(ppsq * float(c["margin_m"]) / float(c["square_m"])))
    sq = np.indices((ny + 1, nx + 1)).sum(axis=0) % 2
    board = np.kron(sq, np.ones((ppsq, ppsq))).astype(np.uint8) * 255
    board = np.pad(board, mg, constant_values=255)
    out = ROOT / "models" / "checkerboard" / "materials" / "textures" / "checkerboard.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), board)
    ft = cfgs["world"]["floor_texture"]
    sx, sy, _ = cfgs["world"]["room"]["size_xyz_m"]
    nx_t, ny_t = int(round(sx / ft["tile_m"])), int(round(sy / ft["tile_m"]))
    rng = np.random.default_rng(1234)
    ap = cfgs["world"]["appearance"]
    base = np.asarray(ap["floor_base_rgb"], float)
    shade = 1.0 + float(ap["floor_tile_variation"]) * (rng.random((ny_t, nx_t, 1)) - 0.5)
    hue = 1.0 + float(ap["floor_hue_jitter"]) * (rng.random((ny_t, nx_t, 3)) - 0.5)
    tiles = np.clip(base * shade * hue, 0.0, 1.0) * 255.0                      # RGB per tile
    img = np.kron(tiles, np.ones((ft["px_per_tile"], ft["px_per_tile"], 1))).astype(np.uint8)
    out = ROOT / "models" / "qkd_textures" / "floor_checker_noise.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))


def write_model_dir(name: str, model_sdf: str, description: str) -> None:
    d = ROOT / "models" / name
    d.mkdir(parents=True, exist_ok=True)
    (d / "model.config").write_text(
        f"<?xml version='1.0'?>\n<model><name>{name}</name><version>1.0</version><sdf version='1.9'>model.sdf</sdf>"
        f"<author><name>qkd_sim (generated by tools/world_builder.py)</name></author>"
        f"<description>{description}</description></model>\n")
    (d / "model.sdf").write_text(f"<?xml version='1.0'?>\n<sdf version='1.9'>{model_sdf}</sdf>\n")


def write_assets(cfgs) -> list[Path]:
    write_textures(cfgs)
    fid = cfgs["camera_action5"]["fidelity"]["default"]
    b = cfgs["beacon"]["red_led"]
    write_model_dir("mini2se", drone_model(cfgs, cfgs["drone"]["payload"]["variant"], (0, 0, 0), yaw_enu_deg=0.0),
                    "DJI Mini 2 SE stand-in (PX4 x500_base derived), default payload")
    write_model_dir("mount_fixed", rig_fixed_model(cfgs, fid, ring_light=False), "Config A: tripod + ball head + Action 5 Pro")
    write_model_dir("gimbal_storm32_3axis", rig_gimbal_model(cfgs, fid, ring_light=False),
                    "Config B: tripod + STorM32 3-axis gimbal + Action 5 Pro")
    write_model_dir("action5pro", f"<model name='action5pro'>{camera_link_sdf(cfgs['camera_action5'], fid, (0, 0, 0), (0, 0, 0), ring_light=None, beacon=cfgs['beacon'])}</model>",
                    "DJI Osmo Action 5 Pro body + webcam-mode camera sensor")
    write_model_dir("ring_light", f"<model name='ring_light'>{camera_link_sdf(cfgs['camera_action5'], fid, (0, 0, 0), (0, 0, 0), ring_light={}, beacon=cfgs['beacon'])}</model>",
                    "Action 5 Pro with the white LED ring light (tag option)")
    r = 0.5 * float(b["render_diameter_m"])
    write_model_dir("beacon_red_led", f"<model name='beacon_red_led'><static>true</static><link name='link'>"
                    f"{visual('led_visual', sph_geom(r), material(b['diffuse_rgb'], emissive=b['emissive_rgb_full']), cast_shadows=False)}</link></model>",
                    "bare 5 mm red LED (emissive)")
    t = cfgs["beacon"]["tag"]
    tv, _, _ = payload_visuals(cfgs, "tag")
    write_model_dir("tag_aruco", f"<model name='tag_aruco'><static>true</static><link name='link'>{tv}</link></model>",
                    f"ArUco {t['dictionary']} id {t['id']} tag plate")
    write_model_dir("desk_fan", fan_model(cfgs),
                    "Desk fan (visual only; wind from sim_bridge/fan_wind.py)")
    write_model_dir("pendulum_rig", pendulum_model(cfgs), "Fallback: red beacon on a 1 m string")
    write_model_dir("checkerboard", checkerboard_model(cfgs, (0, 0, 0)), "Calibration target 9x6 inner corners, 40 mm")
    write_model_dir("occluder", occluder_model(cfgs), "Hand-sized occluder plate")
    write_model_dir("distractors", f"<model name='distractors'>{distractor_models(cfgs, True)}</model>",
                    "Steady red lamp, blinking red indicator, glint patches")
    write_model_dir("spinning_arm", spinning_arm_model(cfgs), "Beacon on a spinning arm (rolling-shutter calibration)")
    write_model_dir("slider_beacon", slider_beacon_model(cfgs), "Beacon sliding sideways at 1 m/s (test A7)")
    (ROOT / "models" / "qkd_textures" / "model.config").write_text(
        "<?xml version='1.0'?>\n<model><name>qkd_textures</name><version>1.0</version>"
        "<description>Shared textures</description></model>\n")
    written = []
    for kind, scn in (("qkd_lab", {"world": "qkd_lab"}), ("qkd_pendulum", {"world": "qkd_pendulum", "drone": {"enabled": False}}),
                      ("qkd_field", {"world": "qkd_field", "lighting": {"initial": "office_50hz"}})):
        rig = "rig_gimbal" if kind == "qkd_field" else "rig_fixed"
        sdf, _ = build_world(cfgs, scn, rig)
        p = ROOT / "worlds" / f"{kind}.sdf"
        p.write_text(sdf)
        written.append(p)
    return written


def write_camera_truth(cfgs, rig: str, path: Path | None = None) -> Path:
    """EVALUATION ONLY: exact simulated intrinsics, distortion and mount pose."""
    cam = cfgs["camera_action5"]
    rc = cfgs["rigs"][rig]
    lens = make_lens(cam, rc["principal_point_offset_px"])
    truth = lens.truth_dict()
    truth.update({
        "rig": rig,
        "render_sensor_type": render_sensor_type(cam),
        "mount_error_deg": dict(rc["mount_error_deg"]),
        "lens_height_m": rc["lens_height_m"],
        "nominal_pan_deg": rc.get("nominal_pan_deg", rc.get("initial_pan_deg")),
        "nominal_tilt_deg": rc.get("nominal_tilt_deg", rc.get("initial_tilt_deg")),
        "rolling_shutter_readout_s": cam["rolling_shutter"]["readout_s"],
        "latency_s": cam["latency"]["capture_to_frame_s"],
    })
    path = path or (ROOT / "evaluation" / "camera_truth.yaml")
    path.write_text("# EVALUATION ONLY. Never read by calibration/ or tracking/ (test B1).\n"
                    + yaml.safe_dump(truth, sort_keys=False))
    return path


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--write-assets", action="store_true")
    ap.add_argument("--scenario", default=None)
    ap.add_argument("--rig", default="rig_fixed", choices=["rig_fixed", "rig_gimbal"])
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    cfgs = cfgmod.load_all()
    if args.write_assets:
        for p in write_assets(cfgs):
            print("wrote", p.relative_to(ROOT))
        print("wrote", write_camera_truth(cfgs, args.rig).relative_to(ROOT))
    if args.scenario:
        scn = yaml.safe_load(Path(args.scenario).read_text())
        cfgs = cfgmod.load_all(scn.get("overrides"))
        sdf, info = build_world(cfgs, scn, args.rig)
        out = Path(args.out or f"{info['world_name']}_{args.rig}.sdf")
        out.write_text(sdf)
        print("wrote", out, info)


if __name__ == "__main__":
    main()
