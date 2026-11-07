# QKD Cyber Sandbox: Hardware-in-the-Loop Emulation

This repository contains the software engine for a Free-Space Optical (FSO) Quantum Key Distribution (QKD) prototype. It implements a **Hardware-in-the-Loop (HIL)** architecture, where physical spatial tracking telemetry dynamically drives a software-emulated BB84 quantum channel.

## 🏗️ Architecture Overview

To comply with the "Cyber Sandbox" requirements, this system does not transmit raw cryptographic bits over a physical light beam. Instead, it separates the physical tracking from the cryptographic generation:

1. **The Physical Link (Hardware):** A drone gimbal targets a ground camera. The camera calculates `drift`, `jitter`, and `intensity`.
2. **The Cyber Bridge (Software):** This telemetry is converted into a dynamic Quantum Channel Error Probability (`P_error`).
3. **The Emulation (Software):** A high-speed BB84 protocol runs in memory, with bit-flips injected based on the physical `P_error`.
4. **The Security Gate:** If the desk fan shakes the drone too much and the sampled Quantum Bit Error Rate (QBER) hits **11%**, the pipeline aborts. Otherwise, it distills a secure 256-bit AES key.

## 📁 Repository Structure

* `src/telemetry.py`: The interface contract. Defines the `TelemetryProvider` base class. The hardware team must build their OpenCV/sensor scripts to inherit from this.
* `src/qkd_core.py`: The black-box cryptographic engine. Handles BB84 sifting, the 11% QBER parameter estimation, cascade error correction, and SHA-256 privacy amplification.
* `src/demo_runner.py`: The integration script. Currently uses a `MockHardware` generator to simulate drone flight and turbulence without physical cameras connected.

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
python src/demo_runner.py
```

OpenCV is included for the planned camera integration; the current mock demo does not use it yet.

## 🧭 Keeping the project organized

For a fast hackathon build, keep the core modules in `src/`. Add `hardware/` for camera/sensor adapters and `tests/` as those pieces arrive. If the codebase grows enough to justify packaging, move the modules together into `src/qkd_simulator/` and add a `pyproject.toml` rather than splitting them up prematurely.
