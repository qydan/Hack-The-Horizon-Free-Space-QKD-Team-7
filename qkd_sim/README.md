# qkd_sim: Gazebo digital twin for the drone FSO-QKD demo

A Gazebo Harmonic + PX4 SITL simulation of the demo's **air segment** (DJI Mini 2 SE stand-in, 7 Hz red LED
beacon or ArUco tag, desk fan) and **ground segment** (DJI Osmo Action 5 Pro in webcam mode, on a fixed tripod
or a STorM32 3-axis gimbal), with all the plumbing between them. The team's blocks, ported from the repo's
`src/` (tracker, BB84 sandbox) and retuned for this camera, run inside it: **tracking** (lock-in or steady-spot
acquisition, Kalman filter, gimbal / virtual-receiver pointing), the **cyber sandbox** (error budget,
decoy-state channel, Eve, BB84 with Cascade and Toeplitz, the 11% gate) and the **preflight brightness
calibration**. The same code runs on the real hardware. See
[Tracking, sandbox and calibration](#tracking-sandbox-and-calibration).

## Quick start

Everything goes through one command, `qkd` (on Windows type `.\qkd`, on Linux/macOS `./qkd`).

```text
.\qkd doctor                 # what's installed, and the exact next step
.\qkd setup                  # Python packages for the offline tools and tests (creates .venv if needed)
.\qkd test                   # offline tests: no simulator needed, ~1 min

.\qkd setup --docker         # once: builds the simulator image (Gazebo Harmonic + PX4 SITL, ~30-60 min)
.\qkd demo                   # 45 s hover with the fan on, camera live at http://localhost:8080
.\qkd demo gimbal            # the same with the STorM32 gimbal rig
.\qkd run t3 gimbal          # any scenario: a unique prefix is enough; rig = fixed | gimbal (or A | B)
.\qkd run t1 fixed --quick   # 20 s at fast fidelity
.\qkd score                  # score the latest run's tracking against ground truth
.\qkd dashboard              # QBER / motion / key-gate dashboard of the latest run (--live while it runs)
.\qkd replay --eve 0.5       # re-run the key segment on the latest run's telemetry (--pns, --pool, --snr)
.\qkd test --sim             # acceptance tests in Gazebo / PX4
.\qkd list                   # all scenarios
```

- **You don't need to install Gazebo.** If it isn't installed natively, `qkd` runs everything inside the
  Docker image for you: it mounts the repo, passes the GPU through and forwards the camera port.
- **The live camera opens in your browser** at <http://localhost:8080> during every `run` / `demo`. It shows
  raw frames with sim time, latency, real-time factor, fan level, lighting and gimbal angles (`--no-view` turns
  it off). This works from Docker on Windows and macOS, where the Gazebo GUI can't be shown; `--gui` adds the
  Gazebo GUI on Linux or a native install.
- **Prerequisites for the simulator:** Docker Desktop (on Windows with the WSL 2 backend) and, for usable speed,
  an NVIDIA GPU with a current driver. Without a GPU, rendering falls back to software and is very slow.
  `qkd doctor` checks all of this.

## Status

| Part | State |
|---|---|
| Configs, models, worlds, scenarios, bridges, bus, recorders, evaluation, `qkd` launcher | Built |
| Tracking, sandbox, preflight brightness calibration (ported from `src/`) | Built; closed-loop tested offline |
| Calibration: intrinsics, mount alignment, gimbal response, rolling shutter, latency | Still OPEN stubs |
| Offline tests (models, camera pipeline, gimbal, no-bypass, world generation, launcher, tracking, sandbox) | **107 pass** (`qkd test`, ~4 min) |
| End-to-end run in Docker (Gazebo + PX4 + all bridges + browser viewer) | **Works** (`qkd demo`) |
| Simulator acceptance tests (A1-A9, A14) | Written; not yet run (`qkd test --sim`) |

The simulator has been brought up end to end in Docker on Windows (see [Open issues](#open-issues) for what
that settled). The next step is `qkd test --sim`.

<details><summary>Running without the launcher</summary>

```bash
docker/run.sh                                                  # shell in the image (Linux; repo at /qkd_sim)
python3 tools/run_scenario.py t3 --rig gimbal --view --quick   # what `qkd run t3 gimbal --quick` does
python3 tools/view_camera.py --rig gimbal                      # OpenCV window on a running scenario
QKD_RUN_SIM=1 pytest -m sim -s
```

Native install instead of Docker: Gazebo Harmonic with `python3-gz-transport13 python3-gz-msgs10`, PX4 built
with `make px4_sitl` after `px4/install_px4_assets.sh $PX4_DIR`, `pip install -r requirements.txt`, and
`PX4_DIR` set. `qkd` detects a native install and uses it.

</details>

Each run writes `runs/<scenario>_<rig>_<timestamp>/`: `world.sdf`, `scenario.yaml`, `summary.json` (real-time
factor, frame rate, beacon method, lighting update rate, port violations), `telemetry.jsonl/.csv`,
`frames_meta.csv` (seq, t_capture, t_delivered, exposure gain, saturated), `sim_events.jsonl`,
`truth.jsonl/.csv` (evaluation only), sample frames and `logs/`. Score a run with
`qkd score [runs/<dir>]` once tracking exists.

## How it fits together

```
                    Gazebo Harmonic (sim time, 1 ms steps, lockstep with PX4)
   mini2se (PX4 SITL) ── beacon / tag ── desk fan ── room, lights, props ── rig_fixed | rig_gimbal + Action 5 Pro
        ^  MAVSDK offboard                 ^ ApplyLinkWrench      ^ light_config        |  camera topic   ^ joint targets
        |                                  |                      | visual_config       v                 |
  hover_station.py      fan_wind.py ───────┘   lighting.py ──────┘  beacon_modulator  GzAction5FrameSource   Storm32GimbalSim
   (OU wander, push)    (jet + OU gusts)       (flicker, scenes)     (7 Hz, sim time)  (rolling shutter, AE, (server, 50 Hz
                                                                                        MJPG, latency)          readback)
                        truth_publisher.py ──> 5590 (evaluation only)                        │                    │
                                                                                             v                    v
                                                    tracking/ ──────── FrameSource ─────────┘  GimbalIO ─────────┘
                                                        │ 5556 telemetry          ^ 5557 abort
                                                        v                         │
                                                    sandbox/ ──────────> 5559 keys     <── 5560 Eve toggle
```

`tools/run_scenario.py` is the bridge host: it generates the world from the configs, starts `gz sim`, PX4 SITL
(attached to the `mini2se` model already in the world, `PX4_GZ_STANDALONE=1`), all bridges as threads on the
sim clock, the recorders, the tracker (`--target` from the scenario) and the sandbox (always under the port
guard, writing `sandbox_log.jsonl`), and plays a scenario's Eve schedule on 5560.

**Every number lives in `config/*.yaml`.** The SDF models and worlds are generated from them by
`tools/world_builder.py` (`--write-assets` refreshes `models/` and `worlds/`), so no value is duplicated in an
asset. Each value is tagged `# SPEC`, `# ESTIMATE` or `# MEASURE`; a test fails if one is not.

## The two ground-station rigs

| | Config A `rig_fixed` | Config B `rig_gimbal` |
|---|---|---|
| Model | `mount_fixed`: tripod + ball head | `gimbal_storm32_3axis`: tripod + pan / roll / tilt joints |
| Camera | Action 5 Pro, lens at 1.00 m, aimed at the hover station | same camera on the tilt link, optical centre 6 mm from the tilt axis |
| Deliberate errors for calibration | mount yaw +0.5°, pitch -0.3°, roll +0.4°; principal point (+8, -5) px | same, camera-to-tilt-link |
| `GimbalIO` | `FixedMount`: `read()` returns the nominal angles (not the error) with sim time; `set_target()` logs a warning | `Storm32GimbalSim`: 20 ms command latency, 50 Hz command rate, 6 Hz / 0.7 second-order axes, 120°/s rate limit, 0.02° holding jitter (10-30 Hz), IMU readback with 0.05° noise and a 0.2°/min random-walk pan bias, 50 Hz |
| Real hardware | `FixedMount` with host clock | `Storm32Gimbal` stub (MAVLink Gimbal Protocol v2) |

The Storm32 emulator has a single owner, the bridge host, which drives the Gazebo joints. Other processes
(the tracker, `view_camera.py`) get a `Storm32GimbalRemote` from `make_gimbal_io("rig_gimbal")`, which talks to
it over an internal port. `make_gimbal_io(..., local=True)` builds an in-process emulator for tests.

**Pan drift (config B).** The gimbal has no magnetometer, so its pan readback drifts (random walk, 0.2°/min).
The tracker computes "world angle = gimbal angle - pixel offset", so this drift moves its world frame:
it shows up in `drift_mrad` over long runs (about 3.5 mrad/min, 1 sigma), not in the residual, which is measured
in the image.

**Config A consequence.** With nothing to steer, the receiver pointing is either a fixed boresight (residual =
drone offset from the boresight) or a software-steered virtual receiver that follows the tracker with the real
frame latency plus a servo lag (`tracking.yaml: pointing.mode_rig_fixed`, default `virtual_receiver`, as
`src/`'s `--virtual-mount`). Telemetry `gimbal_*` fields stay constant.

## Camera emulation (Action 5 Pro, webcam mode)

1920×1080 at 30 fps through a **rectilinear (pinhole) lens with 104° HFOV**: the Action 5 Pro's dewarped
"standard" mode, as on the real rig (f = 750 px, 1.33 mrad/px at the centre, 71.5° VFOV), rendered by an ordinary
`camera` sensor (Gaussian noise 0.007). The build spec's 140° equidistant fisheye is still available:
`camera_action5.yaml: lens.projection: equidistant, lens.hfov_deg: 140` (rendered by a `wideanglecamera`, about 6x
the rendering cost). `GzAction5FrameSource` then applies, in sim time:

1. **Rolling shutter** (`fidelity: full`): sub-frames rendered at 5 bands / 15 ms = 333 Hz; output band k comes
   from the sub-frame nearest its readout time. `fidelity: fast` renders once per frame.
2. **Auto-exposure**: first-order loop in log-gain toward mean brightness 0.45, τ = 0.5 s, applied in linear
   light, clipped at 255.
3. **MJPG** at quality 80 (encode + decode).
4. **Latency**: frames are delivered at `t_capture + 150 ms` in sim time.
5. BGR uint8 out, the same as a real UVC capture.

`Frame.t_capture_sim` is the readout start of the first row. The exposure gain is **not** on the `Frame`,
because the real camera does not report it; it goes to `frames_meta.csv` for evaluation. Tracking has to
estimate the gain for the telemetry `exposure_gain` field.

**Beacon: a bare red LED, steady by default.** One 5 mm red LED (20 mA, 2.0 V), no diffuser, on a short post,
pointing at the camera; it dims off-axis (half brightness at 30 deg) as the drone leans. It is rendered at its
apparent size on the camera (15 mm, MEASURE), because a 1.3 px LED is not drawn by Gazebo. It is constantly lit
like the real rig; `beacon.yaml: red_led.modulation.enabled: true` switches on the spec's 7 Hz blink.

**No IR anywhere.** The drone carries a red LED (~630 nm) and the ground camera is RGB with an IR-cut filter, so
scenario validation rejects any non-visible beacon, e.g. 850 nm (`test_a10_ir_beacon`). The spec's optional
940 nm IR data link (photodiode sensor, port 5558, telemetry `rx_signal`/`rx_background`) has been removed,
matching the real rig.

**Tag limit.** At 3 m the 8 cm ArUco tag spans about 20 px on this camera, too few for a reliable lean (pose)
estimate. T3 with `t3_turbulence_tag` and truth comparison will show how bad it is. Options: a shorter range
(2 m, about 31 px), or a larger tag if the weight budget allows.

## How the drone flies

By default the drone uses a **built-in flight model** (`drone.yaml: flight_model: simple`,
`sim_bridge/simple_flight.py`), not an autopilot: a PID position hold toward the hover station plus its random
wander, the desk-fan jet as drag, and the attitude tilted to point the thrust, so the drone leans into the wind
(steady lean = atan(drag / weight)) and is pushed sideways before recovering. Nothing has to arm, so it always
takes off (airborne about 4 s after start). `flight_model: px4` switches back to PX4 SITL + MAVSDK (PX4 is still
built in the image; arming and health checks then apply and are slow when the sim runs well below real time).

## Scenarios

Every scenario sets world, rigs, payload, fan schedule, lighting, hover wander, fidelity, seed, duration and
recorders. `tools/run_scenario.py <scenario> [--rig ...]` runs each listed rig in turn.

| Scenario | Test | Rigs | Set-up |
|---|---|---|---|
| `t1_noise_floor` | T1 | A, B | Static modulated beacon at 1 m then 4 m, 60 s each; in B the gimbal is off for the first pass, then active |
| `t2_acquisition` | T2 | A, B | 20 trials of 30 s, station randomised within ±1 m, tracker restarted in SEARCH each trial |
| `t3_turbulence` | T3 | A, B | 2 min each at fan off / low / high, occluder passes at 30, 70, 100 s |
| `t3_turbulence_tag` | T3 | A, B | Same, with the ArUco tag and ring light |
| `t4_lighting` | T4 | A, B | dark, office 50 Hz, office 60 Hz, bright lamp, window, distractors (40 s each) |
| `t5_latency` | T5 | A, B | Fan low; capture, delivery and processing timestamps |
| `t6_causality` | T6 | A, B | Fan off → low → high, slow push from 200 s; B adds 0, 1, 2, 4, 8 mrad gimbal offset steps |
| `t7_eve` | T7 | A, B | Fan low, 3 min hover; Eve (port 5560) intercept-resend 50% at 60-90 s, PNS at 120-150 s |
| `calib_intrinsics` | cal | A, B | Checkerboard scripted through 24 poses across the 140° view |
| `calib_alignment` | cal | A, B | Static beacons at (1, 0, 1) and (4, 0, 1) |
| `calib_gimbal` | cal | B | Step, slew and hold commands |
| `calib_rolling_shutter` | cal | A, B | Beacon on a spinning arm (2 rev/s, 0.3 m) |
| `calib_latency` | cal | A, B | Beacon switched on and off at known sim times |
| `calib_brightness` | cal | A, B | Beacon at a known 4 m range |
| `pendulum` | fallback | A, B | Beacon on a 1 m string in the fan jet, low then high |
| `mission_300m` | D3 | B | `qkd_field`, drone at 300 m / 50 m altitude, GNSS wander, beacon scaled 10× |
| `tracking_sweep` | tracking | A, B | Scripted path, no wander, fan off: holds, 30 cm steps, sideways sweeps at 0.2 → 1.5 m/s, a figure-8, range 2 → 4 m. Score per segment with `python tools/tracker_accuracy.py` |
| `test_a7_slider` | A7 | A | Beacon sliding sideways at 1 m/s at 3 m |
| `test_a10_ir_beacon` | A10 | | Invalid on purpose (850 nm beacon) |

**Mission caveat.** At 300 m a 40 mm beacon subtends about 0.13 mrad, a tenth of a pixel, and won't render
reliably. `mission_300m` scales the beacon 10× (`world.yaml: mission.beacon_scale`). Read its results as a
demonstration that a commercial system needs a narrow-field camera.

## Tracking, sandbox and calibration

The team's blocks, ported from the repo's `src/` into the interfaces the simulator defines. `src/` stays as the
standalone version (pinhole webcam, 120 fps, its own simulated rig); these packages are the versions that run
against the Action 5 Pro emulation and the STorM32, and they must stay self-contained (the Docker image mounts only
`qkd_sim/`).

| Block | Ported from | Inputs | Outputs |
|---|---|---|---|
| `tracking/` | `src/vision.py`, `src/tracker.py`, `src/gimbal.py` | `make_frame_source()`, `make_gimbal_io(rig)`, `config/tracking.yaml`, `config/calibration.yaml`, aborts on 5557 | I2 telemetry on 5556 (validated against `telemetry/schema.json`), `GimbalIO.set_target` in config B |
| `sandbox/` | `src/error_budget.py`, `channel.py`, `decoy.py`, `qkd_core.py`, `key_pool.py`, `sandbox.py` | 5556 telemetry, 5560 Eve toggle, `config/sandbox.yaml` | 5557 aborts, 5559 key-block decisions, `--log` run log |
| `calibration/preflight_brightness` | the brightness half of `src/tracker.py` `calibrate()` | the `calib_brightness` scenario | `config/calibration.yaml: preflight_brightness` |
| `calibration/` (the other five) | nothing in `src/` | the `calib_*` scenarios | still OPEN stubs |

**Tracking** (`tracker.Tracker` wires one frame through all of it):

- **Acquisition.** For a blinking beacon: 7 Hz lock-in on the linear-light red channel, using each frame's capture
  time. Its 20-frame window cancels the 10 Hz and 0 Hz aliases of mains flicker. For a steady beacon (the default,
  `beacon.yaml red_led.modulation.enabled: false`): `src/`'s small-bright-spot detector on redness (R − max(G, B))
  at full resolution. A spot must stay lit for 0.5 s (the 2 Hz red indicator never does), with a core-size limit
  against the red lamp and an optional `search_zone`. ArUco is used for the tag target.
- **States.** SEARCH → ACQUIRE (consecutive consistent hits) → TRACK ↔ COAST.
- **Fine track.** Inside the ROI the lock-in confirms the spot, then a sub-pixel centroid is taken on *this*
  frame's redness at its rolling-shutter row time. World angle = gimbal readback interpolated to that row time minus the pixel
  offset, fed to a gated constant-velocity Kalman filter.
- **Lens.** The active lens of `camera_action5.yaml` (rectilinear 104° by default, or the 140° fisheye) until
  `calibration/intrinsics.py` writes K and D.
- **Pointing.** In rig_gimbal: `set_target` with a velocity lead over the frame latency, plus a PI term on the image
  offset. In rig_fixed: the virtual receiver.
- **Telemetry.** Drift and jitter around a boresight (taken from the first seconds after lock), and intensity =
  lock-in amplitude ÷ estimated exposure gain ÷ reference. The gain is estimated by replaying the AE law on the
  frame means. Lean and range come from the ArUco pose.

**Sandbox:** one `Sandbox` per run streams telemetry records into pulses (1 GHz emulated source):

- The error budget maps residual, lean and drift, intensity and range to a transmittance.
- Click counts per class come from a decoy-state model (signal / decoy / vacuum), with Eve (intercept-resend or
  PNS) inside the channel.
- BB84 then sifts, discloses a 20% sample for QBER, runs Cascade and Toeplitz.
- The gate aborts a block on pooled QBER ≥ 11% or inconsistent decoy statistics; an abort is sent to the tracker
  on 5557.

**Offline closed-loop tests** (`tests/test_tracking_offline.py`, `tests/synthetic_rig.py`):

- Synthetic hover scenes are pushed through the real `Action5Pipeline` (auto-exposure, 150 ms latency,
  principal-point offset) and, for rig_gimbal, the real STorM32 dynamics, with the mount error and the lighting
  distractors. Ground truth scores the result.
- Results:
  - first lock about 0.7 s after the beacon appears (blink; under 0.5 s steady)
  - err_x / err_y within about 0.4 mrad RMS of truth, after the constant bias from the uncalibrated lens and mount
  - the gimbal slews about 11° onto the drone
  - occlusions coast or re-acquire
  - a gate abort re-acquires in under 1 s
  - the exposure-gain estimate stays within 10% through a 2.5× AE step
- `tests/test_sandbox_offline.py` reproduces `src/`'s numbers:
  - about 3% QBER with keys when calm
  - intercept-resend on half the pulses gives about 14.5% and aborts
  - PNS is caught by the decoy check
  - keys stop at about 4% QBER per 1 s block
  - `pool` recovers keys around 3.9% QBER, but not at 4.6% (there the decoy-certified single-photon share no longer
    covers the Cascade leakage)

**Findings to keep in mind:**

- **Residual floor.** The camera's 150 ms latency is ~2.5× `src/`'s. Even with a velocity lead, the residual floor
  in calm hover is about 3 mrad (virtual receiver) and 4 mrad (gimbal), against the 10 mrad virtual receiver
  half-width.
- **Take-off and the reference.** The drone takes off at t = 0 and the tracker locks while it is still on the
  floor. In Gazebo t5, a boresight taken during the climb read as 100–180 mrad of drift. The error budget saw
  the beam pointing about 8° away, and the gate aborted 37 of 44 blocks. The boresight is now fixed only on a
  trend-free hover window, then follows the hover mean (30 s). Re-run on that same recorded telemetry: 1 abort
  and 39 keys.
- **The LED is faint in colour.** A Gazebo render of the 15 mm LED is pale pink (redness about 56 before AE).
  AE in a bright office and MJPG's halved chroma resolution leave about 15. The thresholds in `tracking.yaml` are
  set for that; the original ones (30–40, tuned on a pure-red LED) never locked at all. The test rig now
  reproduces the measured colour and MJPG.
- **Steady beacon.** A steady LED can't be told from a steady lamp by blinking. On the plain red channel the
  detector locked onto a white glint 112 mrad off the LED; on brightness alone it took the 2 Hz red indicator.
  Redness, a 0.5 s persistence test and a core-size limit handle both. The size limit also rejects the LED closer
  than about 1.5 m (T1's 1 m pass), so use `search_zone` there.
- **Exposure gain.** The AE replay must meter exactly like the camera. On real hardware, lock the exposure or
  identify the AE law first.
- **Lean counted twice.** The LED dims off-axis, so with a tag pose a lean costs signal twice: through intensity and
  through the error budget's transmitter factor.

**Commands:**

- `qkd run …` runs both blocks. `qkd dashboard` draws `runs/<id>/sandbox_log.jsonl`, and `qkd dashboard --live`
  follows a run in progress.
- `qkd replay [--eve 0.5 | --pns]` re-runs the key segment on a recorded flight.
- `python -m tools.eve_toggle intercept 0.5` sends the operator Eve toggle. `t7_eve` schedules the same toggles.
- `python tools/tune.py` prints the residual / lean dose-response table.
- `python -m calibration.preflight_brightness --range 4` measures I_cal during `calib_brightness`.

**No-bypass rule.** `tracking/` may import only `sim_bridge.frame_source`, `sim_bridge.gimbal_io` and
`telemetry` from this repo. Neither `tracking/` nor `sandbox/` may import `sim_bridge.truth_publisher`,
`evaluation` or `gz.*`, mention port 5590 or `camera_truth.yaml`, or use `allow_truth`. Test B1 walks the AST to
enforce it. At run time the sandbox runs under a port guard that refuses and logs any connection outside 5556,
5557, 5559 and 5560 (test B2). `calibration/` must not read `evaluation/camera_truth.yaml` either; that file is
for `evaluation/score.py`.

## Open issues

Ground rule 5 says to report anything that might not work as specified, with the proposed alternative,
instead of silently changing the design. Items marked **(verified)** were settled by running the simulator
in Docker on Windows (RTX 4070 laptop); the rest still need the sim acceptance tests.

**First end-to-end run** (`qkd demo`, t5_latency, rig_fixed, fast fidelity, 75 s): PX4 armed and held the
station, camera 29.99 fps (sim) with 150.0 ms latency, beacon modulated, truth at 100 Hz, no warnings,
0.64x real time.

1. **Beacon modulation method (A6). (verified)** Method 1, the `/world/<w>/visual_config` emissive change, works
   (only edges are sent: 14 updates/s). Method 3 (two spheres, shell toggle) remains as a fallback. Method 2
   (C++ plugin) is not shipped. Each run records the method in `beacon_method.txt`.
2. **Wide-angle camera speed (A1). (verified, fast fidelity)** With the GPU, `wideanglecamera` at 1920x1080 and
   30 Hz runs at about 1.0x real time on its own and about 0.65x with PX4 and all bridges. Full fidelity
   (333 renders/s) will be slower; everything is in sim time, so results are unaffected. The fallback
   `rectilinear_remap` (120° render) **cannot fill a 140° output: the outer ~10° on each side stay black**.
3. **ApplyLinkWrench persistent wrenches (A4). Open: check this first.** Each 100 Hz update clears and re-adds
   the link's persistent wrench. In the first run the drone leaned **0.47°** in a 2.2 m/s crosswind, where the
   drag model predicts about **1.3°**. Either part of the force is lost between clear and add, or something else
   is compensating. Test A4 measures it. Fallback: the one-step `/wrench` topic every physics step
   (`FanWindBridge(use_step_wrench=True)`).
4. **Mains flicker rate (A8). (verified)** One `light_config` call takes about 0.4-1.3 ms, so the spec's 1 kHz
   update for 4 panels is out of reach. The office now has **2 ceiling panels** updated at **250 Hz** (231 Hz
   achieved), still above twice the 120 Hz flicker. The camera samples the lights at 30 Hz (fast) or 333 Hz (full).
5. **Scene ambient can't change at run time** in Harmonic. The world fixes it at the `dark` level (0.02), and
   higher ambient levels come from two shadowless directional "fill" lights.
6. **A4 band vs spec numbers.** With 5 m/s at fan high, the drone at 3 D (full outlet speed), C_d·A = 0.02 m² and
   264 g, the quasi-static lean is **about 6.6°, just above A4's 3-6° band**. 4 m/s gives 4.6°. Measuring
   C_d·A and the fan speeds (T3) will settle it. Until then A4 is expected to fail narrowly at "high".
7. **Fan levels.** The spec lists "off / low / high" with three speeds (3 / 4 / 5 m/s). I read that as
   `low` 3.0, `medium` 4.0, `high` 5.0 plus `off` (`fan.yaml`). Please confirm.
8. **Inertia.** The spec's figures (5.9e-4 / 7.6e-4 / 1.26e-3 kg·m²) ignore the 20 mm rotor height. The model
   uses the tensor about the true centre of mass (6.1e-4 / 7.9e-4 / 1.26e-3 without payload,
   `tools/compute_inertia.py`). The `mini2se` model is written from PX4 `x500_base`'s structure and sensor
   parameters, but was not copied from a PX4 checkout.
9. **Hover throttle (A3).** The ESC outputs are set so motor speed is linear in the command (`SIM_GZ_EC_MIN 0`),
   with `THR_MDL_FAC 1`. Hover thrust is then 0.50 without a payload and 0.54 with the beacon. Rate gains are
   PX4 defaults × 0.5 (about twice the x500's torque-to-inertia ratio). Check the step response.
10. **Tag rendering.** Retroreflection is emulated with the tag texture as both albedo and emissive map, so the
    black cells stay dark. This assumes ogre2 multiplies the emissive map by the emissive colour. If the tag
    appears mirrored on the `plane` geometry, set `beacon.yaml: texture.flip_horizontal: true`.
11. **Gamma.** Auto-exposure assumes the rendered 8-bit values are gamma-2.2 encoded. Because the gain is a
    power law, beacon brightness ratios are preserved either way (A6 measures them).
12. **Gimbal pan drift.** I read "random-walk bias 0.2°/min" as a 1-σ growth of 0.2° after 1 minute
    (σ ∝ √t).
13. **Not emulated:** the Bayer mosaic and demosaicing, and electronic stabilisation, which must be **off** on
    the real camera.
14. **PX4 version. (verified)** v1.16.0 builds and flies the `4099_gz_mini2se` airframe. PX4 can refuse the
    first arm right after boot, so `hover_station.py` waits for "armable" and retries.

Fixed while bringing up the simulator (no action needed, noted for whoever maintains this):

- **GPU rendering on Windows.** Docker Desktop's `--gpus all` gives CUDA only; OpenGL silently falls back to the
  CPU (0.2x real time). `qkd` mounts `/usr/lib/wsl` and selects Mesa's `d3d12` driver, which renders on the
  NVIDIA GPU (OpenGL 4.6).
- **Shadows are off** (`world.yaml: render.shadows`): Gazebo 8.15's wide-angle camera segfaults in the ogre2
  shadow pass when lights change at run time. Nothing measured depends on shadows.
- **Service requests go through a helper process** (`sim_bridge/gz_service_proxy.py`). The gz-transport Python
  binding keeps the GIL while waiting for a reply, so in a process that also receives the 1 kHz clock or camera
  images every request timed out. The clock and poses are also subscribed throttled (`world.yaml: bridge`).
- **Lights are always sent complete.** A `light_config` request replaces the whole light, so partial updates
  zeroed the range and attenuation. The world file and the run-time controller now share one definition
  (`sim_bridge/lighting.py: light_specs`).
- **MAVSDK is pinned below 4.0.** Version 4 is a different (native) API.

## Values to replace with measurements

Section 16 of the spec. Until these are measured, label any result that depends on them. The full table is
regenerated by `python3 tools/list_estimates.py` (`--measure-only` for the short list).

<details><summary>All ESTIMATE / MEASURE values (current numbers)</summary>

| File | Key | Value | Tag | Note |
|---|---|---|---|---|
| `world.yaml` | `bridge.clock_rate_hz` | 250.0 | ESTIMATE |  |
| `world.yaml` | `bridge.pose_rate_hz` | 200.0 | ESTIMATE |  |
| `world.yaml` | `render.shadows` | false | ESTIMATE | visual realism only; nothing measured depends on shadows |
| `world.yaml` | `room.centre_xy_m` | [2.5, 0.0] | ESTIMATE | room placed so camera and drone both sit well inside |
| `world.yaml` | `room.wall_rgb` | [0.55, 0.55, 0.52] | ESTIMATE | matte wall albedo |
| `world.yaml` | `camera_mount.lens_height_m` | 1.00 | MEASURE | lens-centre height on the real tripod |
| `world.yaml` | `checkerboard.hidden_position_m` | [-3.5, 2.8, -5.0] | ESTIMATE | off-scene parking spot (below the floor, behind the camera) |
| `world.yaml` | `spherical_coordinates.latitude_deg` | 47.397971 | ESTIMATE | (PX4 default home) |
| `world.yaml` | `spherical_coordinates.longitude_deg` | 8.546164 | ESTIMATE |  |
| `world.yaml` | `spherical_coordinates.elevation_m` | 488.0 | ESTIMATE |  |
| `world.yaml` | `props.checkerboard.margin_m` | 0.02 | ESTIMATE | white border |
| `world.yaml` | `props.checkerboard.texture_px_per_square` | 64 | ESTIMATE |  |
| `world.yaml` | `props.occluder.size_m` | [0.10, 0.18] | ESTIMATE | hand-sized plate (width, height) |
| `world.yaml` | `props.occluder.default_duration_s` | 0.6 | ESTIMATE | sweep time |
| `world.yaml` | `props.occluder.default_sweep_m` | 0.40 | ESTIMATE | sweep length across the line of sight |
| `world.yaml` | `props.spinning_arm.position_m` | [3.0, 0.0, 1.0] | ESTIMATE | hub at the hover-station range |
| `world.yaml` | `props.spinning_arm.arm_length_m` | 0.30 | ESTIMATE |  |
| `world.yaml` | `props.spinning_arm.speed_rad_s` | 12.566 | ESTIMATE | 2 rev/s -> 3.8 m/s tip speed |
| `world.yaml` | `props.slider_beacon.position_m` | [3.0, -1.5, 1.0] | ESTIMATE | start of travel |
| `world.yaml` | `props.slider_beacon.travel_m` | 3.0 | ESTIMATE |  |
| `world.yaml` | `props.stand.pole_diameter_m` | 0.02 | ESTIMATE | beacon tripod pole |
| `world.yaml` | `props.desk_fan_visual.base_diameter_m` | 0.25 | ESTIMATE |  |
| `world.yaml` | `props.desk_fan_visual.head_depth_m` | 0.12 | ESTIMATE |  |
| `world.yaml` | `props.tripod.leg_spread_m` | 0.35 | ESTIMATE | visual only |
| `world.yaml` | `floor_texture.tile_m` | 0.10 | ESTIMATE | random checker tiles for "vision positioning" texture |
| `world.yaml` | `floor_texture.px_per_tile` | 24 | ESTIMATE |  |
| `world.yaml` | `mission.ground_size_m` | 1000.0 | ESTIMATE |  |
| `world.yaml` | `mission.beacon_scale` | 10.0 | ESTIMATE | 40 mm -> 0.4 m sphere so it spans ~1 px at 300 m (documented) |
| `drone.yaml` | `geometry.motor_diagonal_m` | 0.213 | ESTIMATE | Mini 2 family figure; MEASURE with a ruler |
| `drone.yaml` | `geometry.rotor_z_m` | 0.02 | ESTIMATE | rotor plane above body centre |
| `drone.yaml` | `geometry.prop_radius_m` | 0.06 | ESTIMATE | 0.12 m props |
| `drone.yaml` | `geometry.arm_mass_kg` | 0.020 | ESTIMATE | per arm+motor point mass (x4) |
| `drone.yaml` | `geometry.body_mass_kg` | 0.166 | ESTIMATE | body + battery box (0.246 - 4*0.020) |
| `drone.yaml` | `geometry.rotor_link_mass_kg` | 0.0005 | ESTIMATE | per prop link (gz needs a non-zero mass); adds 2 g in total |
| `drone.yaml` | `geometry.prop_visual_thickness_m` | 0.004 | ESTIMATE | visual only |
| `drone.yaml` | `motor.max_rot_velocity` | 1000.0 | ESTIMATE | (x500 convention) |
| `drone.yaml` | `motor.rotor_velocity_slowdown_sim` | 10 | ESTIMATE | (x500 convention) |
| `drone.yaml` | `motor.motor_constant` | 1.2e-6 | ESTIMATE | -> 1.2 N max thrust per motor |
| `drone.yaml` | `motor.moment_constant` | 0.016 | ESTIMATE |  |
| `drone.yaml` | `motor.time_constant_up_s` | 0.0125 | ESTIMATE |  |
| `drone.yaml` | `motor.time_constant_down_s` | 0.025 | ESTIMATE |  |
| `drone.yaml` | `motor.rotor_drag_coefficient` | 1.066e-5 | ESTIMATE | x500 8.06428e-05 * (0.12/0.33)^2 |
| `drone.yaml` | `motor.rolling_moment_coefficient` | 1.32e-7 | ESTIMATE | x500 1e-06 * (0.12/0.33)^2 |
| `drone.yaml` | `aero.body_cd_area_m2` | 0.02 | ESTIMATE | 4 m/s crosswind -> ~4.5 deg lean; MEASURE in T3 |
| `drone.yaml` | `px4.airframe_id` | 4099 | ESTIMATE | pick an unused number; install script checks for clashes |
| `drone.yaml` | `px4.mpc_thr_hover` | 0.50 | ESTIMATE | with THR_MDL_FAC=1 (quadratic thrust model); verify with A3 |
| `drone.yaml` | `px4.thr_mdl_fac` | 1.0 | ESTIMATE | gz motor model gives thrust ~ command^2 |
| `drone.yaml` | `px4.inertia_scale_vs_x500` | 0.0335 | ESTIMATE | Ixx ratio mini2se / x500 (6.2e-4 / 0.0185) used to scale rate gains |
| `drone.yaml` | `payload.variants.tag.offset_m` | [-0.071, 0.0, 0.0] | ESTIMATE | on the rear face (body x = -69 mm) plus 2 mm plate |
| `drone.yaml` | `station_keeping.start_yaw_deg` | 0.0 | ESTIMATE | (ENU) nose toward +x, so the rear face (tag) faces the camera at the origin |
| `drone.yaml` | `station_keeping.spawn_offset_m` | [0.0, 0.0, 0.03] | ESTIMATE | spawn on the floor under the hover station (landing-gear clearance) |
| `drone.yaml` | `station_keeping.setpoint_rate_hz` | 20.0 | ESTIMATE | MAVSDK offboard rate |
| `drone.yaml` | `station_keeping.slow_push.axis` | [0.0, 1.0, 0.0] | ESTIMATE | sideways = +y (image horizontal) |
| `drone.yaml` | `station_keeping.gnss_wander.tau_s` | 10.0 | ESTIMATE |  |
| `drone.yaml` | `mavsdk.system_address` | "udpin://0.0.0.0:14540" | ESTIMATE | PX4 SITL default offboard link |
| `beacon.yaml` | `red_led.emissive_rgb_full` | [1.0, 0.05, 0.02] | ESTIMATE | emissive colour at 100% (tune so the rendered core is just below 255 at gain 1) |
| `beacon.yaml` | `red_led.diffuse_rgb` | [0.9, 0.85, 0.8] | ESTIMATE | ping-pong-ball white plastic |
| `beacon.yaml` | `red_led.modulation.phase_s` | 0.0 | ESTIMATE |  |
| `beacon.yaml` | `red_led.update_rate_hz` | 1000.0 | ESTIMATE | one update per physics step; edges only matter, so 2 updates/period would do |
| `beacon.yaml` | `tag.white_margin_m` | 0.01 | ESTIMATE | matte white quiet zone around the marker |
| `beacon.yaml` | `tag.texture_px` | 512 | ESTIMATE | generated texture size |
| `beacon.yaml` | `tag.emissive_white_full` | 0.8 | ESTIMATE | emulates retroreflection under the ring light |
| `beacon.yaml` | `ring_light.inner_diameter_m` | 0.05 | ESTIMATE |  |
| `beacon.yaml` | `ring_light.outer_diameter_m` | 0.09 | ESTIMATE |  |
| `beacon.yaml` | `pendulum.joint_damping_nms_per_rad` | 0.0005 | ESTIMATE | light damping |
| `beacon.yaml` | `pendulum.stand_height_m` | 2.2 | ESTIMATE | pivot so the beacon hangs at 1.2 m |
| `fan.yaml` | `diameter_m` | 0.40 | MEASURE |  |
| `fan.yaml` | `levels_mps.low` | 3.0 | MEASURE | with a phone anemometer |
| `fan.yaml` | `levels_mps.medium` | 4.0 | MEASURE |  |
| `fan.yaml` | `levels_mps.high` | 5.0 | MEASURE |  |
| `fan.yaml` | `jet.profile` | gaussian_hwhm | ESTIMATE | half-width read as half-width at half maximum |
| `fan.yaml` | `turbulence.intensity` | 0.15 | ESTIMATE | 15% of local speed, each axis |
| `fan.yaml` | `turbulence.tau_s` | 0.3 | ESTIMATE |  |
| `fan.yaml` | `force.pendulum_beacon_cd_area_m2` | 0.00063 | ESTIMATE | sphere Cd 0.5 * pi * 0.02^2 |
| `camera_action5.yaml` | `sensor_spec.passband_nm` | [400, 680] | ESTIMATE | visible band passed by the IR-cut filter |
| `camera_action5.yaml` | `stream.width_px` | 1920 | MEASURE | v4l2-ctl --list-formats-ext |
| `camera_action5.yaml` | `stream.height_px` | 1080 | MEASURE |  |
| `camera_action5.yaml` | `stream.fps` | 30.0 | MEASURE |  |
| `camera_action5.yaml` | `stream.pixel_format` | MJPG | MEASURE |  |
| `camera_action5.yaml` | `lens.projection` | equidistant | ESTIMATE | fisheye model r = f * theta |
| `camera_action5.yaml` | `lens.hfov_deg` | 140.0 | ESTIMATE | ; MEASURE (155 is the max, the 16:9 crop may be narrower) |
| `camera_action5.yaml` | `mount_lens_offset_m` | [0.0, 0.0, 0.0] | ESTIMATE | optical centre relative to the camera link origin |
| `camera_action5.yaml` | `render.rectilinear_width_px` | 1920 | ESTIMATE |  |
| `camera_action5.yaml` | `render.rectilinear_height_px` | 1080 | ESTIMATE |  |
| `camera_action5.yaml` | `render.env_texture_size_px` | 1024 | ESTIMATE | wide-angle cube-map face size |
| `camera_action5.yaml` | `render.noise_stddev` | 0.007 | ESTIMATE | Gaussian, in [0,1] units |
| `camera_action5.yaml` | `render.clip_near_m` | 0.05 | ESTIMATE |  |
| `camera_action5.yaml` | `render.clip_far_m` | 500.0 | ESTIMATE | (mission world needs > 300 m) |
| `camera_action5.yaml` | `rolling_shutter.readout_s` | 0.015 | ESTIMATE | ; MEASURE (spinning line or fast LED) |
| `camera_action5.yaml` | `rolling_shutter.bands` | 5 | ESTIMATE | -> render sub-frames at bands / readout = 333 Hz |
| `camera_action5.yaml` | `exposure.target_mean` | 0.45 | ESTIMATE | mean brightness in [0,1] |
| `camera_action5.yaml` | `exposure.tau_s` | 0.5 | ESTIMATE | slow loop time constant |
| `camera_action5.yaml` | `exposure.gain_min` | 0.125 | ESTIMATE |  |
| `camera_action5.yaml` | `exposure.gain_max` | 16.0 | ESTIMATE |  |
| `camera_action5.yaml` | `exposure.gain_initial` | 1.0 | ESTIMATE |  |
| `camera_action5.yaml` | `exposure.locked_gain` | 1.0 | ESTIMATE | used when mode == locked |
| `camera_action5.yaml` | `exposure.gamma` | 2.2 | ESTIMATE | gain applied in linear light, then re-encoded |
| `camera_action5.yaml` | `compression.mjpg_quality` | 80 | ESTIMATE |  |
| `camera_action5.yaml` | `latency.capture_to_frame_s` | 0.150 | ESTIMATE | ; MEASURE in T5 |
| `camera_action5.yaml` | `topics.image` | /qkd/action5/image | ESTIMATE | explicit <topic> set in the model SDF |
| `rigs.yaml` | `rig_fixed.lens_height_m` | 1.00 | MEASURE |  |
| `rigs.yaml` | `rig_fixed.nominal_pan_deg` | 0.0 | ESTIMATE | derived aim, yaw toward +x |
| `rigs.yaml` | `rig_fixed.nominal_tilt_deg` | 3.81 | ESTIMATE | derived atan(0.2 / 3.0), positive = up |
| `rigs.yaml` | `rig_fixed.nominal_roll_deg` | 0.0 | ESTIMATE |  |
| `rigs.yaml` | `rig_fixed.mount_error_deg` | {yaw: 0.5, pitch: -0.3, roll: 0.4} | ESTIMATE |  |
| `rigs.yaml` | `rig_fixed.principal_point_offset_px` | [8.0, -5.0] | ESTIMATE | (applied in GzAction5FrameSource) |
| `rigs.yaml` | `rig_gimbal.lens_height_m` | 1.00 | MEASURE |  |
| `rigs.yaml` | `rig_gimbal.initial_pan_deg` | 0.0 | ESTIMATE |  |
| `rigs.yaml` | `rig_gimbal.initial_tilt_deg` | 3.81 | ESTIMATE |  |
| `rigs.yaml` | `rig_gimbal.mount_error_deg` | {yaw: 0.5, pitch: -0.3, roll: 0.4} | ESTIMATE |  |
| `rigs.yaml` | `rig_gimbal.principal_point_offset_px` | [8.0, -5.0] | ESTIMATE |  |
| `rigs.yaml` | `rig_gimbal.joint_heights_m.pan_base` | 0.86 | ESTIMATE |  |
| `rigs.yaml` | `rig_gimbal.joint_heights_m.roll_axis` | 0.94 | ESTIMATE |  |
| `rigs.yaml` | `rig_gimbal.joint_heights_m.tilt_axis` | 1.00 | ESTIMATE | equals lens height (camera balanced on the tilt axis) |
| `rigs.yaml` | `rig_gimbal.optical_centre_to_tilt_axis_m` | 0.006 | ESTIMATE | within the 10 mm SPEC balance requirement |
| `gimbal_storm32.yaml` | `mass_kg` | 0.200 | ESTIMATE |  |
| `gimbal_storm32.yaml` | `limits_deg.pan` | [-150.0, 150.0] | MEASURE | (cable-limited on many cheap gimbals) |
| `gimbal_storm32.yaml` | `command.max_rate_hz` | 50.0 | ESTIMATE | setpoints faster than this are dropped (latest wins) |
| `gimbal_storm32.yaml` | `command.latency_s` | 0.020 | ESTIMATE | serial + controller loop |
| `gimbal_storm32.yaml` | `response.natural_frequency_hz` | 6.0 | ESTIMATE | ; MEASURE with step commands |
| `gimbal_storm32.yaml` | `response.damping_ratio` | 0.7 | ESTIMATE |  |
| `gimbal_storm32.yaml` | `response.rate_limit_dps` | 120.0 | ESTIMATE | per axis |
| `gimbal_storm32.yaml` | `response.integration_dt_s` | 0.001 | ESTIMATE | matches the physics step |
| `gimbal_storm32.yaml` | `holding_jitter.rms_deg` | 0.02 | ESTIMATE | ; MEASURE (T1 gimbal active vs off) |
| `gimbal_storm32.yaml` | `holding_jitter.band_hz` | [10.0, 30.0] | ESTIMATE | band-limited |
| `gimbal_storm32.yaml` | `readback.rate_hz` | 50.0 | ESTIMATE |  |
| `gimbal_storm32.yaml` | `readback.noise_rms_deg` | 0.05 | ESTIMATE | IMU-based attitude, no encoders |
| `gimbal_storm32.yaml` | `readback.pan_drift_deg_per_min` | 0.2 | ESTIMATE | random-walk bias (no magnetometer); 1-sigma growth rate per sqrt(min) |
| `gimbal_storm32.yaml` | `joint_control.p_gain` | 1000.0 | ESTIMATE | = 1 / physics step: dead-beat, the joint reaches the target in one step |
| `gimbal_storm32.yaml` | `joint_control.i_gain` | 0.0 | ESTIMATE |  |
| `gimbal_storm32.yaml` | `joint_control.d_gain` | 0.0 | ESTIMATE |  |
| `gimbal_storm32.yaml` | `joint_control.cmd_max` | 100.0 | ESTIMATE | rad/s velocity clamp (well above the 120 deg/s emulated rate limit) |
| `gimbal_storm32.yaml` | `real_hardware.serial_port` | /dev/ttyUSB0 | MEASURE |  |
| `gimbal_storm32.yaml` | `real_hardware.mavlink_system_id` | 255 | ESTIMATE | laptop as a GCS |
| `lighting.yaml` | `mains.update_rate_hz` | 250.0 | ESTIMATE | spec asks 1000 (one per step); a light_config call takes ~0.4 ms and 4 panels x 1 kHz is out of reach. 250 Hz > 2 x 120 Hz flicker |
| `lighting.yaml` | `ceiling_panels.positions_m` | [[1.5, 0.0, 2.9], [4.0, 0.0, 2.9]] | ESTIMATE | 2 panels (not 4): each flicker update is a light_config call |
| `lighting.yaml` | `ceiling_panels.diffuse` | [0.9, 0.9, 0.85] | ESTIMATE |  |
| `lighting.yaml` | `ceiling_panels.intensity` | 1.6 | ESTIMATE | (2 panels carry the light of 4) |
| `lighting.yaml` | `ceiling_panels.attenuation_range_m` | 8.0 | ESTIMATE |  |
| `lighting.yaml` | `ceiling_panels.attenuation` | {constant: 0.5, linear: 0.05, quadratic: 0.01} | ESTIMATE |  |
| `lighting.yaml` | `ceiling_panels.specular` | [0.1, 0.1, 0.1] | ESTIMATE |  |
| `lighting.yaml` | `ambient_fill.directions` | [[0.05, 0.05, -1.0], [1.0, 0.1, -0.2]] | ESTIMATE | down, and from behind the camera |
| `lighting.yaml` | `spot_defaults.range_m` | 20.0 | ESTIMATE |  |
| `lighting.yaml` | `spot_defaults.attenuation` | {constant: 0.2, linear: 0.02, quadratic: 0.005} | ESTIMATE |  |
| `lighting.yaml` | `spot_defaults.falloff` | 1.0 | ESTIMATE |  |
| `lighting.yaml` | `spot_defaults.specular` | [0.5, 0.5, 0.5] | ESTIMATE |  |
| `lighting.yaml` | `scenarios.office_50hz.ambient` | 0.25 | ESTIMATE |  |
| `lighting.yaml` | `scenarios.office_60hz.ambient` | 0.25 | ESTIMATE |  |
| `lighting.yaml` | `scenarios.bright_lamp.ambient` | 0.25 | ESTIMATE |  |
| `lighting.yaml` | `scenarios.bright_lamp.spot.position_m` | [3.0, 0.5, 1.3] | ESTIMATE | beside the drone, aimed at the camera |
| `lighting.yaml` | `scenarios.bright_lamp.spot.aim_at_m` | [0.0, 0.0, 1.0] | ESTIMATE |  |
| `lighting.yaml` | `scenarios.bright_lamp.spot.diffuse` | [1.0, 1.0, 0.95] | ESTIMATE |  |
| `lighting.yaml` | `scenarios.bright_lamp.spot.intensity` | 8.0 | ESTIMATE | washout |
| `lighting.yaml` | `scenarios.bright_lamp.spot.outer_angle_deg` | 30.0 | ESTIMATE |  |
| `lighting.yaml` | `scenarios.bright_lamp.spot.inner_angle_deg` | 15.0 | ESTIMATE |  |
| `lighting.yaml` | `scenarios.window.ambient` | 0.25 | ESTIMATE |  |
| `lighting.yaml` | `scenarios.window.sun.direction` | [-0.6, 0.2, -0.4] | ESTIMATE | through the back-wall window toward the camera |
| `lighting.yaml` | `scenarios.window.sun.diffuse` | [1.0, 0.97, 0.9] | ESTIMATE |  |
| `lighting.yaml` | `scenarios.window.sun.intensity` | 3.0 | ESTIMATE |  |
| `lighting.yaml` | `scenarios.window.window_plane.centre_m` | [6.45, 0.0, 1.6] | ESTIMATE | in the back wall (x = 6.5) |
| `lighting.yaml` | `scenarios.window.window_plane.size_m` | [2.0, 1.2] | ESTIMATE | (width y, height z) |
| `lighting.yaml` | `scenarios.window.window_plane.emissive` | [1.0, 1.0, 0.95] | ESTIMATE | large bright area in the 140 deg view |
| `lighting.yaml` | `scenarios.distractors.ambient` | 0.25 | ESTIMATE |  |
| `lighting.yaml` | `distractors.steady_red_lamp.position_m` | [3.6, 0.9, 1.0] | ESTIMATE |  |
| `lighting.yaml` | `distractors.steady_red_lamp.emissive_rgb` | [1.0, 0.05, 0.02] | ESTIMATE |  |
| `lighting.yaml` | `distractors.steady_red_lamp.diameter_m` | 0.06 | ESTIMATE |  |
| `lighting.yaml` | `distractors.blinking_red_indicator.position_m` | [2.6, -0.7, 0.9] | ESTIMATE |  |
| `lighting.yaml` | `distractors.blinking_red_indicator.emissive_rgb` | [1.0, 0.02, 0.0] | ESTIMATE |  |
| `lighting.yaml` | `distractors.blinking_red_indicator.diameter_m` | 0.01 | ESTIMATE |  |
| `bus.yaml` | `internal_ports.gimbal_state` | 5570 | ESTIMATE | Storm32GimbalSim readback (server in the bridge host -> GimbalIO clients) |
| `bus.yaml` | `internal_ports.gimbal_cmd` | 5573 | ESTIMATE | GimbalIO clients -> Storm32GimbalSim set_target |
| `bus.yaml` | `internal_ports.frames_meta` | 5571 | ESTIMATE | frame-source metadata for frames_meta.csv |
| `bus.yaml` | `internal_ports.sim_events` | 5572 | ESTIMATE | occluder, lighting and fan schedule events (for the recorder) |

</details>

## Repository layout

```
config/        every number (tagged)            models/, worlds/  generated SDF (tools/world_builder.py)
sim_bridge/    everything that touches Gazebo   px4/              airframe 4099_gz_mini2se + installer
telemetry/     schema, ZeroMQ bus, recorder     evaluation/       truth recorder, scorer, camera_truth.yaml
tracking/      acquisition, tracking, pointing  calibration/      preflight brightness (rest still OPEN)
sandbox/       error budget, decoys, Eve, BB84  scenarios/        T1-T7, calib_*, pendulum, mission, test support
tools/         run_scenario, world_builder, view_camera, mjpeg_server, make_tag_texture, compute_inertia,
               port_guard, list_estimates, scenario, make_dashboard, live_dashboard, tune, eve_toggle
qkd.py         the launcher (qkd.cmd on Windows, ./qkd on Linux/macOS)
tests/         offline + simulator acceptance tests
docker/        Dockerfile, run.sh
```
