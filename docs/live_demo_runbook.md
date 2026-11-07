# Live Demo Presentation Runbook

**Goal:** Prove to the judges that physical environmental disturbances accurately drive cryptographic security thresholds in real-time.

## Pre-Flight Checklist (10 Minutes Before Demo)
- [ ] Drone is powered on, and the gimbal is holding a steady horizon lock.
- [ ] Green LED beacon is powered and clearly visible on the camera feed.
- [ ] Desk fan is plugged in, positioned, but **turned OFF**.
- [ ] Run with the live camera: `python src/demo_runner.py --source camera --calibration calibration.json --duration 120 --log run.jsonl --record demo.csv` (calibrate first, see `docs/hardware_handoff.md`).
- [ ] Fallback ready: `--source csv --csv demo.csv` (recorded run) or `--source synthetic`.

## The Pitch Script

### Step 1: The Baseline (Steady State)
* **Action:** Start `python src/demo_runner.py`. Keep the fan OFF.
* **Speaker:** *"Judges, what you are seeing is the Cyber Sandbox. Our drone is hovering steadily. The ground camera is tracking the beacon at 60 frames per second. Because the physical drift and jitter are low, our simulated Quantum Error Rate (QBER) is sitting comfortably at ~3%. The pipeline is successfully sifting bases and distilling secure AES-256 keys."*
* **Visual:** Point to the terminal showing successful AES key outputs.

### Step 2a: Sustained Turbulence (keys keep flowing)
* **Action:** Turn the desk fan to a MODERATE setting. Keep it on for the rest of the 2-minute window.
* **Speaker:** *"Under real turbulence the tracker holds lock and QBER stays below 11%, so keys keep being issued."*

### Step 2b: The Attack / Environmental Disturbance
* **Action:** Turn the desk fan to HIGH power, aiming directly at the drone.
* **Speaker:** *"We are now introducing extreme mechanical turbulence, simulating high wind or a physical intercept attempt. Notice the drone gimbal fighting to stay locked."*

### Step 3: The Cryptographic Abort
* **Action:** Watch the terminal output.
* **Speaker:** *"Look at our telemetry bridge. The physical jitter has spiked, driving our Quantum Channel Error probability up. Our BB84 parameter estimation has caught the disturbance. The QBER has breached the 11% Shor-Preskill threshold. The system has automatically aborted the key generation to guarantee absolute cryptographic security."*
* **Visual:** Point to the red `[ABORT]` warnings in the terminal. 

### Optional: Eavesdropper
* **Action:** Re-run on the recorded CSV with `--eve 1.0` (QBER jumps by about 25 points, every window aborts) and `--decoy --pns` (the PNS attacker passes the QBER check but is rejected by the decoy statistics).

### Step 4: Recovery
* **Action:** Turn the fan OFF.
* **Speaker:** *"As the drone stabilizes, the physical jitter drops, the QBER falls back below 11%, and secure key distribution automatically resumes."*

## Contingency Plans
* **If the camera fails/disconnects:** Restart with `--source csv --csv demo.csv` (the recorded run), or `--source synthetic`. Explain that you are replaying recorded telemetry through the same pipeline.