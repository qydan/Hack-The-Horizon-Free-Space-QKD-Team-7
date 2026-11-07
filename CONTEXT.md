# Secure the Last Mile: Drone-to-Ground FSO-QKD

## Overview

Terrestrial fiber networks are rigid, and satellite ground stations are vulnerable to weather and tracking blockages. To build resilient networks, we need a **rapidly deployable, mobile Free-Space Optical Quantum Key Distribution (FSO-QKD) link**.

Teams must design a **scalable drone-to-ground quantum network system** and demonstrate a **functional prototype** using available hardware and software emulation.

---

## Instructions

### 1. System Architecture
Document a **non-military, Commercial-Off-The-Shelf (COTS)** design for a **<5 kg drone payload** capable of receiving and transmitting QKD keys.

### 2. Physical Prototype Setup (The Demonstration)
Deploy a **hardware-in-the-loop mock link** using any available consumer drone and camera/sensor combination, for example:
- OpenCV
- Micro-gimbals
- LEDs / classical lasers

### 3. The Cyber Sandbox
- Real-time spatial tracking accuracy (**jitter, drift, light intensity**) must **directly feed** a local software network simulator.
- **No hardcoded or traditional wireless bypasses** are permitted.
- Quantum keys must be **dynamically generated via this physical tracking telemetry**.

---

## Deliverables

### D1: Live Link Demo
- Demonstrate **autonomous spatial target acquisition in < 60 seconds**.
- Sustain **continuous tracking across a 2-minute flight window** under simulated environmental turbulence (e.g., wind drift from a desk fan).

### D2: Crypto Pipeline Execution
- Run a complete, active software **BB84 pipeline**:
  1. Sifting
  2. Error correction
  3. Privacy amplification
- Keep the calculated **Quantum Bit Error Rate (QBER) strictly below 11%**.

### D3: Engineering Design
- Submit an engineering design detailing a clear **technical roadmap from the hackathon prototype to a commercial QKD distribution drone**.

### Optional: Threat Modeling
- Handle **intercept-resend** attacks.
- Handle **photon-number-splitting (PNS)** attacks via **decoy states** (e.g., a 2nd IR transmitter and camera).

---

## Checklist

| ID | Requirement | Target |
|----|-------------|--------|
| D1 | Target acquisition time | < 60 s |
| D1 | Continuous tracking window | 2 min under turbulence |
| D2 | BB84: sift, error-correct, privacy-amplify | Complete and active |
| D2 | QBER | < 11% |
| D3 | Hackathon-to-commercial roadmap | Submitted |
| Opt | Intercept-resend / PNS (decoy-state) threat model | Bonus |
