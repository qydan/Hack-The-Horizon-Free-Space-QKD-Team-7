# System Architecture & Deliverable D3 Roadmap

This document outlines the architecture for the QKD Cyber Sandbox prototype, as well as the proposed commercial roadmap (Deliverable D3) for a fully bidirectional optical tracking system.

## 1. Prototype Architecture (One-Way Optical, Two-Way Classical)

To satisfy the hackathon requirements for a Hardware-in-the-Loop (HIL) quantum simulation, the prototype isolates the physical Pointing, Acquisition, and Tracking (PAT) layer from the cryptographic quantum emulation.

### The 4-Layer HIL Model

1. **Layer 1: Physical PAT (Pointing, Acquisition, Tracking)**
   - **Drone:** Carries a continuous green LED beacon mounted to a stabilized gimbal.
   - **Environment:** Subjected to mechanical turbulence via a desk fan.
   - **Ground Station:** A stationary high-framerate USB camera (e.g., DJI Action in UVC mode) capturing at 60 FPS.

2. **Layer 2: OpenCV Telemetry Bridge (`src/telemetry.py`)**
   - Processes camera frames to isolate the beacon centroid.
   - Calculates real-time physical metrics:
     - `Drift`: Spatial offset from the calibrated optical boresight (pixels).
     - `Jitter`: Velocity of the centroid movement between frames (pixels/sec).
     - `Intensity`: Received optical power/brightness.

3. **Layer 3: Cyber Sandbox Transfer Engine (`src/qkd_core.py`)**
   - Maps physical telemetry to a Quantum Channel Error Probability ($P_{error}$) using the transfer function:
     `P_error = e_optical + (alpha * drift) + (beta * jitter) + (gamma / intensity)`

4. **Layer 4: BB84 Cryptographic Pipeline (`src/qkd_core.py`)**
   - Software-emulated quantum states are transmitted and subjected to bit-flips based on $P_{error}$.
   - Executes classical basis sifting over a local socket.
   - **Security Gate:** Destructively samples 20% of the sifted key to estimate QBER. If QBER $\ge 11\%$, the channel aborts.
   - Performs cascade error correction and SHA-256 Privacy Amplification to yield AES-256 symmetric keys.

---

## 2. Deliverable D3: Commercial Deployment Roadmap (Two-Way Optical)

For a real-world, 5+ km Free-Space Optical (FSO) deployment, the one-way ground tracking must be upgraded to a mutual tracking architecture.

### Commercial Hardware Specifications
* **Drone Node (Alice):**
  * **Uplink Sensor:** SWIR Camera tracking a 1550nm ground beacon.
  * **Downlink Emitter:** 850nm single-photon quantum source steered via a Fast Steering Mirror (FSM).
* **Ground Node (Bob):**
  * **Uplink Emitter:** 1550nm wide-divergence acquisition laser.
  * **Downlink Sensor:** Single-Photon Avalanche Diode (SPAD) array behind a motorized tracking telescope.

### Two-Way Error Composite
In the commercial system, the Cyber Sandbox error function expands to include mutual alignment failure. The composite error rate $P_{total}$ ensures that if *either* node loses target lock, the key generation safely aborts:
`P_total = 1.0 - (1.0 - P_downlink) * (1.0 - P_uplink)`