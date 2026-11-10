# D3: Engineering Design and Roadmap to a Commercial QKD Drone

**Scope statement.** This is a civilian, non-military design built from commercial off-the-shelf
(COTS) components. Intended uses: secure last-mile links for critical infrastructure,
disaster-recovery networks, temporary links between sites where fiber is unavailable, and
resilient backup when satellite ground stations are blocked by weather or tracking limits.

> All masses, powers and loss figures below are **planning estimates** for the class of
> component, not datasheet values. Verify each against the chosen part before ordering.

## 1. Concept

A multirotor drone carries a QKD terminal and flies a short free-space optical (FSO) link
to a ground terminal. Both ends can transmit and receive:

- **Quantum channel:** polarization-encoded BB84 using decoy states, at 850 nm (silicon SPAD
  detectors, good atmospheric transmission, mature COTS parts).
- **Pointing channel:** a classical beacon in a different band (1550 nm) with a SWIR tracking
  camera at each end, so each terminal tracks the other. The quantum beam is steered by a
  fast steering mirror (FSM) from the same tracking error.
- **Classical channel:** authenticated sifting / error-correction traffic over a conventional
  radio or optical link. It carries no secrets.
- **Role-agnostic payload:** every terminal carries both a transmitter and a receiver, so the
  drone can act as Alice or Bob, and one drone can relay between two ground nodes.

```
 Drone terminal                                           Ground terminal
 ┌─────────────────────────────┐   850 nm quantum (down) ┌───────────────────────────┐
 │ BB84 source + decoy modulator│ ─────────────────────► │ telescope + SPAD receiver │
 │ SPAD receiver (role swap)    │ ◄───────────────────── │ source (role swap)        │
 │ SWIR camera + FSM + gimbal   │   1550 nm beacons       │ SWIR camera + FSM + mount │
 │ compute (tracking + QKD post)│ ◄─────────────────────► │ compute + key management  │
 └─────────────────────────────┘   authenticated classical└───────────────────────────┘
```

## 2. Payload budget (target under 5 kg)

| Subsystem | COTS class | Mass (kg, est.) | Power (W, est.) |
|---|---|---|---|
| 2-axis stabilised gimbal | Camera-class gimbal, payload-rated | 1.0 | 10 |
| Fast steering mirror + driver | Voice-coil / piezo FSM | 0.4 | 5 |
| Transmitter: 850 nm laser diode, polarization modulation, attenuator | Fiber-coupled diode + EOM | 0.5 | 8 |
| Receiver: small telescope + SPAD module (silicon) | 50 mm class optics, Si SPAD | 0.8 | 10 |
| SWIR tracking camera + 1550 nm beacon laser | InGaAs camera, eye-safe beacon | 0.6 | 12 |
| Compute (tracking loop, QKD post-processing, time tagging) | Embedded GPU/FPGA board | 0.3 | 15 |
| Battery (payload only) | about 100 Wh Li-ion | 0.7 | n/a |
| Structure, thermal, cabling | Carbon plate, fasteners | 0.5 | 2 |
| **Total** | | **4.8** | **about 60** |

At about 60 W the 100 Wh pack gives roughly 1.5 hours of terminal operation; endurance is limited
by the airframe battery, not the payload. A tethered power variant removes that limit for
fixed-site links.

## 3. Link budget (illustrative, 850 nm downlink)

Geometric loss dominates: spot diameter = divergence x range, collected fraction ~ (D_rx / spot)^2.

| Term | 1 km | 5 km |
|---|---|---|
| Beam divergence (full angle) | 100 urad | 20 urad |
| Spot diameter at receiver | 0.10 m | 0.10 m |
| Receiver aperture | 0.05 m | 0.05 m |
| Geometric loss | about 6 dB | about 6 dB |
| Atmosphere (clear air, about 0.5 dB/km) | 0.5 dB | 2.5 dB |
| Optics + detector efficiency | about 6 dB | about 6 dB |
| Pointing allowance | 3 dB | 3 dB |
| **Total** | **about 15.5 dB (eta about 0.03)** | **about 17.5 dB (eta about 0.02)** |

The 5 km case only closes if the beam stays narrow (20 urad), and a 20 urad beam needs pointing
error well under 10 urad. That is why mutual tracking (beacon cameras + FSM) is essential, not optional.
The sandbox uses the same structure at room scale (`src/error_budget.py`): geometric loss from
range, a receiver pointing factor from the measured ground residual, a transmitter pointing
factor from drone lean and drift, and beacon brightness. With a 10 degree LED beam and a 25 mm
lens that is about 26 dB at the 3 m demo range. QBER then follows from signal clicks against
noise clicks.

The prototype models the downlink only. In the two-way design, either direction losing lock
aborts key generation.

## 4. Safety, regulatory and security
- **Laser safety:** keep the quantum beam at single-photon-level powers and the beacon within
  eye-safe limits for the exposure geometry (IEC 60825-1 / ANSI Z136 classes). Add an
  automatic shutter when lock is lost.
- **Airspace:** operate under the civil drone rules of the deployment region (for example
  FAA Part 107 or the EU open/specific categories). Laser emission into navigable airspace needs
  local notification or approval.
- **Security engineering:** hardened detectors against blinding / time-shift attacks,
  authenticated classical channel, beacon pattern verification, and key management
  integration (ETSI GS QKD 014 style key-delivery API) for the consuming encryptors.
- **Environment:** thermal control of optics, vibration isolation, and a weather envelope
  (daylight background and rain/fog will limit uptime; the abort logic must fail safe).

## 5. Roadmap

| Phase | Goal | What gets built | Exit criteria |
|---|---|---|---|
| **0. Hackathon (now)** | Prove the pipeline | Blinking beacon + ground camera hardware-in-the-loop (lock-in tracker); telemetry-driven BB84 emulation (error budget, sift, Cascade, privacy amplification), decoy-state and intercept-resend / PNS models | Acquisition under 60 s; 2 min tracking under fan turbulence; QBER below 11% with keys issued |
| **1. Bench QKD** (0-3 mo) | Replace emulated photons with real ones | Attenuated 850 nm source, polarization encoding, SPAD receiver on an optical table or 10 m indoor path | Measured QBER and key rate; check the error budget against real data |
| **2. Outdoor ground link** (3-9 mo) | Tracking over real atmosphere | Ground-to-ground FSO at 100 m to 1 km with SWIR beacons, FSM, daylight filtering | Link held across a day / weather range; measured eta and QBER vs turbulence |
| **3. Drone integration** (9-18 mo) | Fit the payload on an airframe | Under 5 kg terminal on a gimbal; tethered hover tests | Mass and power budget met; lock held in wind; reacquisition tests |
| **4. Free flight** (18-30 mo) | Drone-to-ground keys | Free-flight link at 1 to 5 km, role swap, relay between two ground nodes; ETSI-style key API | Sustained secret key rate at range; integration demo with an off-the-shelf encryptor |
| **5. Pilot** (30+ mo) | Commercialise | Hardening, certifications, safety case, pilot with an infrastructure operator | Pilot SLA; certification path agreed |

Main risks: pointing stability at long range (mitigation: FSM + beacons, tethered tests first),
daylight background (narrow filters, timing gates), atmospheric turbulence (adaptive
aperture, larger receivers), and regulatory approval for laser operation near airspace.

## 6. How the hackathon prototype maps to this design

| Hackathon prototype | Commercial equivalent |
|---|---|
| 13 Hz blinking beacon on the drone (LED or retroreflector) | 1550 nm beacon laser, on a different wavelength from the 850 nm quantum source |
| Ground camera with lock-in detection and Kalman filter (fixed, or on a pan-tilt gimbal) | SWIR tracking camera + FSM closed loop |
| Ground residual (what the ground station failed to remove) | Receiver pointing error after the FSM |
| Drone lean (body-mounted beacon tilts with the drone) | Transmitter pointing error: the case for a drone-side gimbal and FSM |
| `drift`, `jitter`, `intensity`, `range` telemetry | Pointing jitter, received power, link range |
| Error budget (telemetry to transmittance to QBER) | Measured QBER vs telemetry (phase 1-2) |
| Emulated photons in software | Real polarization-encoded photons on SPADs |
| Decoy-state / intercept-resend / PNS models | Same protocol logic, running on real counts |
| Optional IR LED data link (`rx_signal`, `rx_background`) | Classical channel for sifting and error correction |
