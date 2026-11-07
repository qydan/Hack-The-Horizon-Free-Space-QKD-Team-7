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
* `src/demo_runner.py`: The integration script. Runs on a mock generator (`--source mock`) or replays a telemetry CSV (`--source csv`).
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

Telemetry CSV format: header `timestamp,drift,jitter,intensity`, one row per camera frame (seconds, px, px/s, 0-1).

OpenCV is included for the planned camera integration; the current mock demo does not use it yet.

## 🧪 Running the Test Suite

Tests use `pytest` and live in `tests/`. They only need `numpy` and `pytest` (no hardware or OpenCV).

### One-time setup
```bash
# On Debian/Ubuntu/WSL, if `python3 -m venv` fails with "ensurepip is not available":
sudo apt update && sudo apt install -y python3-venv python3-pip

python3 -m venv .venv
source .venv/bin/activate        # On Windows: .venv\Scripts\activate
python -m pip install numpy pytest
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
