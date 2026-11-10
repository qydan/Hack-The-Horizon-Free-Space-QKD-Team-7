# Live Demo Presentation Runbook

**Goal:** show the judges that the quality of the quantum key rises and falls with how well the ground station tracks a real, wind-blown drone.

## Pre-Flight Checklist (10 Minutes Before Demo)
- [ ] Drone is powered on and the beacon is blinking and clearly visible on the camera feed.
- [ ] Desk fan is plugged in, positioned, but **turned OFF**.
- [ ] Camera checked and calibrated in calm hover (`docs/hardware_handoff.md`, steps 1-2); `calibration.json` is present.
- [ ] Two terminals ready, sandbox first:
  `python src/demo_runner.py --source zmq --log run.jsonl --record demo.csv --keys-out keys.jsonl`
  then `python src/tracker.py --source camera --fps 60 --duration 120`.
- [ ] Fallback ready: `python src/demo_runner.py --source csv --csv demo.csv` (a recorded run) or `python src/demo_runner.py` (simulated rig).

## The Pitch Script

### Step 1: The Baseline (Steady State)
* **Action:** Start the sandbox, then the tracker. Keep the fan OFF.
* **Speaker:** *"Judges, what you are seeing is the Cyber Sandbox. Our drone is hovering steadily. The ground camera has locked onto the drone's blinking beacon and measures its motion every frame. That measured motion is the only input to the key software. Because the tracking residual is low, our emulated Quantum Bit Error Rate (QBER) sits at about 3%. The pipeline is sifting bases, correcting errors and distilling 256-bit keys."*
* **Visual:** Point to the terminal lines ending in `KEY ...`.

### Step 2a: Sustained Turbulence (keys keep flowing)
* **Action:** Turn the desk fan to a LOW setting.
* **Speaker:** *"Under real turbulence the tracker holds lock. QBER rises with the measured motion but the block average stays below 11%, so keys keep being issued."*
* **Note:** per-block keys get scarce above about 5% QBER (finite-size effect, see README). If the fan setting sits there, say so, or run with `--pool` or a longer `--block`.

### Step 2b: The Environmental Disturbance
* **Action:** Turn the desk fan to HIGH power, aiming directly at the drone.
* **Speaker:** *"We are now introducing strong turbulence. The drone is pushed off the receiver's line of sight faster than the ground station can follow."*

### Step 3: The Cryptographic Abort
* **Action:** Watch the terminal output.
* **Speaker:** *"The measured pointing error has cut the received signal, so noise clicks make up a larger share of the detections. Parameter estimation has caught it: the block's QBER has crossed the 11% threshold. The gate discards the block and tells the tracker to re-acquire. No key is issued from a block we cannot trust."*
* **Visual:** Point to the `ABORT: QBER ...; tracker re-acquires` lines.

### Step 4: Recovery
* **Action:** Turn the fan OFF.
* **Speaker:** *"As the drone settles, the tracker re-acquires the beacon, QBER falls back and key distribution resumes on its own."*

### Optional: Eavesdropper
* **Action:** Re-run on the recorded CSV:
  * `python src/demo_runner.py --source csv --csv demo.csv --eve 0.5`: intercept-resend on half the pulses, QBER about 14.5%, every block aborts.
  * `python src/demo_runner.py --source csv --csv demo.csv --pns`: the PNS attacker keeps QBER near 3% and passes the QBER check, but the decoy statistics reject every block (`DECOY ABORT`).

### Afterwards
* `python tools/make_dashboard.py run.jsonl dashboard.html` gives the QBER trace against the gate, the two error bars (ground residual vs drone lean) and the gate decision per block.
* `python tools/tune.py` prints the dose-response table (test T6).

## What to say honestly
* Photons, detectors and Eve are emulated in software; the tracking telemetry driving them is measured.
* Basis and parity exchange are in-process calls standing in for the authenticated classical channel.
* When a run is a CSV or simulated replay rather than the live camera, say so.
* On the fixed camera, drone lean is not measured, so the drone-lean error bar is empty.

## Contingency Plans
* **If the camera fails or disconnects:** restart with `--source csv --csv demo.csv` (the recorded run), or `python src/demo_runner.py` (simulated rig). Explain that you are replaying recorded telemetry through the same pipeline.
* **If the sandbox exits early with `0 keys in the key store`:** it stops after 5 s without telemetry. Start it again (or pass `--timeout 60`) before the tracker.
* **If the beacon locks onto a lamp or window:** restart the tracker with `--zone x0,y0,x1,y1` around the hover area.
