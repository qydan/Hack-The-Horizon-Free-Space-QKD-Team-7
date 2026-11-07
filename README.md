# QKD Cyber Sandbox: Hardware-in-the-Loop Emulation

This repository contains the software engine for a Free-Space Optical (FSO) Quantum Key Distribution (QKD) prototype. It implements a **Hardware-in-the-Loop (HIL)** architecture, where physical spatial tracking telemetry dynamically drives a software-emulated BB84 quantum channel.

## 🏗️ Architecture Overview

To comply with the "Cyber Sandbox" requirements, this system does not transmit raw cryptographic bits over a physical light beam. Instead, it separates the physical tracking from the cryptographic generation:

1. **The Physical Link (Hardware):** A drone gimbal targets a ground camera. The camera calculates `drift`, `jitter`, and `intensity`.
2. **The Cyber Bridge (Software):** This telemetry is converted into a dynamic Quantum Channel Error Probability (`P_error`).
3. **The Emulation (Software):** A high-speed BB84 protocol runs in memory, with bit-flips injected based on the physical `P_error`.
4. **The Security Gate:** If the desk fan shakes the drone too much and the sampled Quantum Bit Error Rate (QBER) hits **11%**, the pipeline aborts. Otherwise, it distills a secure 256-bit AES key.

## 📁 Repository Structure

* `src/telemetry.py`: The interface contract. Defines the `TelemetryProvider` base class (metric definitions and units are in its docstring) and `CSVReplayProvider`, which replays recorded/synthetic telemetry. The OpenCV/camera provider will inherit from the same base class.
* `src/qkd_core.py`: The cryptographic engine. BB84 transmit/sift, 11% QBER gate, Cascade error correction (with leakage accounting), and Toeplitz privacy amplification to a 256-bit key. Key bits come from `os.urandom`.
* `src/camera_provider.py`: OpenCV telemetry. LED detector, acquisition/tracking state machine (SEARCH → ACQUIRE → TRACK, with reacquire and a time-to-lock timer), calibration, `VideoCaptureSource` for a real camera and `SyntheticLEDSource` for development.
* `src/decoy.py`: Decoy-state analysis and a photon-number-splitting attacker model.
* `src/key_pool.py`: Pools sifted bits across windows so noisier conditions still yield keys.
* `src/demo_runner.py`: The integration script. Sources: `mock`, `csv` (replay), `synthetic` (rendered camera frames), `camera` (real USB camera). Options for the threat model (`--eve`, `--decoy`, `--pns`), pooling (`--pool`), recording (`--record`) and run logs (`--log`).
* `data/fake_telemetry.csv`: Synthetic 120 s telemetry (acquisition, calm, moderate fan, gust, recovery). Regenerate with `python tools/generate_fake_telemetry.py`.

## 🚀 How to Test the Engine (Hardware-Free)

You do not need the drone, gimbal, or cameras to test the cryptography and system logic. The `demo_runner.py` script includes a synthetic mock environment.

### Prerequisites
Use Python 3 and install the project dependencies:
```bash
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
python -m pip install -r requirements.txt
```

Run the hardware-free demo:
```bash
python src/demo_runner.py                      # quick mock run
python src/demo_runner.py --source csv         # replay data/fake_telemetry.csv (120 windows)
python src/demo_runner.py --source csv --csv my_recording.csv --window 1.0
```

More modes:
```bash
python src/demo_runner.py --source synthetic --quiet --log run.jsonl   # camera pipeline on rendered frames (120 s)
python tools/make_dashboard.py run.jsonl dashboard.html                # charts + stats for the run
python src/demo_runner.py --source csv --eve 1.0                       # intercept-resend attacker -> aborts
python src/demo_runner.py --source csv --decoy --pns                   # PNS attacker -> rejected by decoy states
python src/demo_runner.py --source csv --pool                          # pool windows into larger blocks
python src/demo_runner.py --source camera --calibrate 10               # real USB camera (see docs/hardware_handoff.md)
```

Telemetry CSV format: header `timestamp,drift,jitter,intensity`, one row per camera frame (seconds, px, px/s, 0-1).

OpenCV is included for the planned camera integration; the current mock demo does not use it yet.

## 📚 Docs

* `CONTEXT.md`: the hackathon brief (deliverables D1-D3).
* `docs/system_architecture.md`: layered design of the prototype.
* `docs/d3_engineering_design.md`: D3, payload budget, link budget and roadmap to a commercial drone.
* `docs/threat_model.md`: intercept-resend, PNS / decoy states, and what is out of scope.
* `docs/hardware_handoff.md`: what the rig team needs to do (tuning, calibration, recording).
* `docs/hardware_bill_of_materials.md`, `docs/calibration_log.md`, `docs/live_demo_runbook.md`.

## 🧪 Running the Test Suite

Tests use `pytest` and live in `tests/`. They need `numpy`, `pytest` and OpenCV (`opencv-python-headless` is enough); no camera or hardware is used.

### One-time setup
```bash
# On Debian/Ubuntu/WSL, if `python3 -m venv` fails with "ensurepip is not available":
sudo apt update && sudo apt install -y python3-venv python3-pip

python3 -m venv .venv
source .venv/bin/activate        # On Windows: .venv\Scripts\activate
python -m pip install numpy pytest opencv-python-headless
```

### Run the tests
From the repository root, with the virtual environment active:
```bash
pytest tests/ -v                                   # full suite
pytest tests/ -k "ChannelError or EndToEnd" -v     # black-box tests only (public API in, result out)
pytest tests/ -k calibration -v                    # check calm/fan targets from docs/calibration_log.md
pytest tests/ -x                                   # stop at first failure
```

Tests put the engine in seeded test mode (`tests/conftest.py`), so results are reproducible. Real runs are unseeded and use `os.urandom`. A failure in `TestEndToEnd` or `test_obstruction_trips_gate` usually means the calibration constants need tuning, not that the code is broken.

### Tuning the calibration constants
`tools/tune.py` runs a Monte-Carlo sweep for calm, fan-on and lens-blocked scenarios and prints QBER and abort/key rates:
```bash
python tools/tune.py                                   # current constants
python tools/tune.py --gamma 0.02 --beta 0.0008        # try new constants
python tools/tune.py --calm 3,1,0.9 --fan 40,60,0.2    # use your measured telemetry (drift,jitter,intensity)
```
Once the targets are met (calm QBER ~3%, fan/obstruction aborts), copy the constants into `QuantumCryptoEngine.__init__` and record them in `docs/calibration_log.md`.

## 🧭 Keeping the project organized

For a fast hackathon build, keep the core modules in `src/`. Add `hardware/` for camera/sensor adapters and `tests/` as those pieces arrive. If the codebase grows enough to justify packaging, move the modules together into `src/qkd_simulator/` and add a `pyproject.toml` rather than splitting them up prematurely.
