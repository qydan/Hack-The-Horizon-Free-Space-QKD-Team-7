# Hardware Bill of Materials (BOM) & Physical Setup Guide

The physical hardware-in-the-loop rig that drives the ground tracker. Choices started from the
early design brief (`original_design.docx`, partly out of date); see `docs/hardware_handoff.md` for the
bring-up procedure.

## 1. Bill of Materials

| Component | Purpose | Notes |
| :--- | :--- | :--- |
| **Drone** | Holds station in the fan stream; its motion is the disturbance. | DJI Mini 2 SE, flown by hand and parked in a hover. No SDK or drone telemetry is used. A beacon takes it over 250 g: check the event rules. |
| **Beacon, option A** | One bright point for the ground camera to lock onto. | **Used on the rig:** a red LED, constantly on (`--beacon steady`), under a ping-pong ball diffuser, on its own battery. No radio, no microcontroller. |
| **Beacon, option B** | Same, with nothing powered on the drone. | Retroreflective tape or a retroreflective AprilTag (6-8 cm, about 3 g), lit by a steady ring light around the camera lens. |
| **AprilTag** (optional) | Drone lean and range from the ground. | 36h11, 8-10 cm (`vision.AprilTagPose`). Not used in fixed-camera mode yet. |
| **Ground camera** | Captures the beacon. | **DJI Osmo Action 5 Pro in webcam mode** (USB, 1080p at 30 fps, exposure and white balance locked), with a steady red beacon. Upgrade path: Arducam B0332 (OV9281, mono, global shutter, about 120 fps), which would allow a blinking beacon and lock-in detection. |
| **Gimbal** (optional) | Points the camera and reports its angles. | Pan-tilt gimbal; needs a `gimbal.Gimbal` driver wrapper. Without it the camera is mounted still (`FixedGimbal`). |
| **Turbulence generator** | Pushes the drone to create drift, jitter and lean. | Desk fan with several speeds. |
| **Ground station PC** | Runs the tracker and the cyber sandbox. | Python 3 with `requirements.txt` (NumPy, pyzmq, opencv-contrib-python). |

## 2. Physical Rig Setup

### Distance & Lighting
1. Position the camera about **3 m** from the drone (the calibration range the error budget assumes).
2. Vision positioning on the Mini 2 SE needs a patterned floor, more than 15 lux and 0.5-10 m height; over a plain floor it drifts more.
3. The beacon does not blink, so the tracker finds it as the brightest small spot in view: dim the room, keep lamps, screens and windows out of the picture (or outside `--zone`), and avoid other small bright spots. Keep the light above 15 lux for the drone's vision positioning (dim, not dark). A blinking beacon would tolerate room lights; this one does not.

### Camera
1. Hard-mount the camera on a desk or heavy tripod. In fixed-camera mode **it must not move** after calibration.
2. Aim it so the calm-hover position is at the image centre (the residual is measured from there).
3. Lock exposure, gain and white balance, then run `python tools/camera_check.py` until it reports READY.

### Fan
1. Position the fan so its air stream hits the drone body, not the camera.
2. Find three settings before the demo: off, a low setting where keys keep flowing, and a high setting that makes the gate abort.
