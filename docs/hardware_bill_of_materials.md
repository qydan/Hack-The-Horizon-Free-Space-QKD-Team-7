# Hardware Bill of Materials (BOM) & Physical Setup Guide

This document defines the physical Hardware-in-the-Loop (HIL) rig required to drive the OpenCV telemetry bridge.

## 1. Bill of Materials

| Component | Purpose | Notes |
| :--- | :--- | :--- |
| **Drone Frame & Gimbal** | Physical host for Alice's optical beacon. | Must possess an active 2-axis or 3-axis stabilization gimbal. |
| **Optical Beacon** | Represents the quantum downlink. | 5mm High-Brightness Green LED (520nm) + 3V CR2032 Battery. Taped directly to the drone gimbal lens. |
| **Ground Receiver** | Captures PAT telemetry at high speed. | DJI Action Cam (in UVC Webcam mode) OR PlayStation 3 Eye Camera. **Must support 60 FPS minimum.** |
| **Turbulence Generator** | Injects mechanical jitter into the drone. | 3-Speed oscillating desk fan. |
| **Ground Station PC** | Runs OpenCV and BB84 simulation. | Python 3.x with NumPy and OpenCV installed. |

## 2. Physical Rig Setup & Calibration

### Distance & Lighting
1. Position the Ground Receiver approximately **1.5 to 2.0 meters** away from the drone.
2. Dim ambient room lighting if possible to maximize the contrast of the LED beacon against the background.

### Camera Tuning (OpenCV)
1. Ensure the camera is hard-mounted to a desk or heavy tripod. **The ground camera must not move.**
2. Manually lock the camera exposure in OpenCV to prevent auto-exposure from masking intensity drops:
   `cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)`
   `cap.set(cv2.CAP_PROP_EXPOSURE, -6)` (Adjust based on room lighting)

### Mechanical Tuning
1. Position the desk fan approximately **1 meter** perpendicular to the drone.
2. Ensure the fan's air stream hits the drone's body/gimbal directly, not the ground camera.