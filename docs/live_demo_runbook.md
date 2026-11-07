# Live Demo Presentation Runbook

**Goal:** Prove to the judges that physical environmental disturbances accurately drive cryptographic security thresholds in real-time.

## Pre-Flight Checklist (10 Minutes Before Demo)
- [ ] Drone is powered on, and the gimbal is holding a steady horizon lock.
- [ ] Green LED beacon is powered and clearly visible on the camera feed.
- [ ] Desk fan is plugged in, positioned, but **turned OFF**.
- [ ] Verify `src/demo_runner.py` is configured to use the live OpenCV tracker (not the Mock tracker).

## The Pitch Script

### Step 1: The Baseline (Steady State)
* **Action:** Start `python src/demo_runner.py`. Keep the fan OFF.
* **Speaker:** *"Judges, what you are seeing is the Cyber Sandbox. Our drone is hovering steadily. The ground camera is tracking the beacon at 60 frames per second. Because the physical drift and jitter are low, our simulated Quantum Error Rate (QBER) is sitting comfortably at ~3%. The pipeline is successfully sifting bases and distilling secure AES-256 keys."*
* **Visual:** Point to the terminal showing successful AES key outputs.

### Step 2: The Attack / Environmental Disturbance
* **Action:** Turn the desk fan to HIGH power, aiming directly at the drone.
* **Speaker:** *"We are now introducing extreme mechanical turbulence, simulating high wind or a physical intercept attempt. Notice the drone gimbal fighting to stay locked."*

### Step 3: The Cryptographic Abort
* **Action:** Watch the terminal output.
* **Speaker:** *"Look at our telemetry bridge. The physical jitter has spiked, driving our Quantum Channel Error probability up. Our BB84 parameter estimation has caught the disturbance. The QBER has breached the 11% Shor-Preskill threshold. The system has automatically aborted the key generation to guarantee absolute cryptographic security."*
* **Visual:** Point to the red `[ABORT]` warnings in the terminal. 

### Step 4: Recovery
* **Action:** Turn the fan OFF.
* **Speaker:** *"As the drone stabilizes, the physical jitter drops, the QBER falls back below 11%, and secure key distribution automatically resumes."*

## Contingency Plans
* **If the camera fails/disconnects:** Immediately swap the `hardware` instance in `src/demo_runner.py` back to `MockHardware()` and restart. Explain that you are falling back to the software simulator mode to demonstrate the math.