# Drone QKD demo: ground tracking drives a software BB84 link

The demo proves one thing: **the quality of the quantum key rises and falls with how well the ground station tracks a real, wind-blown drone.**

A ground camera finds a drone hovering in front of a fan, locks onto its blinking beacon and measures how much it moves. That measured motion becomes photon loss in a software BB84 channel, which decides whether the key is safe to keep. The source of truth for the design is `Drone QKD demo system diagram explained.docx`; `docs/system_architecture.md` maps it onto the code.

```
drone + beacon --light--> camera + gimbal + tracker --telemetry--> error budget -> channel + decoys
                                   ^                                  -> Eve -> BB84 -> QBER/decoy gate -> keys
                                   +------------ abort / re-acquire -------------------------+
```

Two separate Python programs on one laptop, joined only by a ZeroMQ telemetry stream (no hardcoded values, no radio):

* `src/tracker.py`: search, lock-in detection of the 13 Hz beacon, Kalman filter, gimbal control, drift / jitter / residual / lean / range / intensity measurement. Runs on the simulated rig or on a real fixed camera.
* `src/sandbox.py` (driven by `src/demo_runner.py`): error budget, decoy-state channel, Eve emulator, BB84 (sift, Cascade, Toeplitz), QBER + decoy gate, key store.

## Repository structure

| File | What it is |
|---|---|
| `src/telemetry.py` | `TelemetryMessage` schema, ZeroMQ publisher/subscriber (I2) and control channel (I8), CSV replay |
| `src/vision.py` | lock-in detector, Kalman filter, AprilTag pose, search-stage detectors (OpenCV DNN / YOLO), camera capture |
| `src/gimbal.py` | `Gimbal` interface (wrap the real driver here), `SimGimbal`, `FixedGimbal` (camera mounted still) |
| `src/tracker.py` | `GroundTracker` (SEARCH -> TRACK -> COAST), calibration, tracker program (`--source sim | camera`) |
| `src/sim.py` | simulated rig: wind-blown drone, blinking beacon, room lights, camera |
| `src/error_budget.py` | telemetry -> transmittance -> QBER |
| `src/channel.py` | signal / decoy / vacuum pulses, Poisson photons, dark counts, Eve emulator |
| `src/decoy.py` | decoy-state analysis and the PNS attacker model |
| `src/qkd_core.py` | BB84 pipeline: sifting, QBER sample, Cascade, Toeplitz privacy amplification |
| `src/key_pool.py` | optional pooling of accepted blocks |
| `src/sandbox.py` | the key segment end to end, gate, key store |
| `src/demo_runner.py` | runs it: `--source sim | csv | zmq` |
| `tools/` | `generate_fake_telemetry.py`, `tune.py` (dose-response, test T6), `make_dashboard.py`, `camera_check.py` (camera bring-up) |

## Setup

```bash
python -m venv .venv
source .venv/bin/activate            # Windows: .venv\Scripts\activate
python -m pip install -r requirements.txt pytest
```

## Run it (no hardware)

```bash
python src/demo_runner.py                          # simulated rig, 120 s: fan off, low, high, gust, recovery
python src/demo_runner.py --source csv             # replay data/fake_telemetry.csv (fast)
python src/demo_runner.py --source csv --eve 0.5   # intercept-resend on half the pulses -> aborts (QBER about 14.5%)
python src/demo_runner.py --source csv --pns       # PNS attacker -> QBER stays low, decoy check aborts
python src/demo_runner.py --source csv --pool      # pool blocks before distilling
python src/demo_runner.py --log run.jsonl && python tools/make_dashboard.py run.jsonl dashboard.html
python tools/tune.py                               # dose-response table: residual and lean vs QBER
```

Two real programs over ZeroMQ (start the sandbox first):

```bash
python src/demo_runner.py --source zmq
python src/tracker.py --source sim --realtime
```

Useful options: `--block` (seconds per key block), `--pulse-rate`, `--snr` (on-axis signal/noise, sets the locked QBER), `--keys-out keys.jsonl`, `--record telemetry.csv` (replayable), `--timeout` (seconds of telemetry silence before the zmq sandbox stops, default 5).

## Run it on a real camera

Fixed camera, no gimbal (run on Windows Python, not WSL). Full procedure: `docs/hardware_handoff.md`.

```bash
python tools/camera_check.py --device 0 --fps 60                             # is the camera good enough? prints the settings to use
python src/tracker.py --source camera --fps 60 --calibrate 15 --duration 1   # calm hover: saves calibration.json
python src/demo_runner.py --source zmq --log run.jsonl --record demo.csv     # sandbox first
python src/tracker.py --source camera --fps 60                               # then the tracker (120 s)
```

`--beacon steady` tracks a constantly-lit LED (needs a dark scene), `--zone x0,y0,x1,y1` restricts where the beacon is looked for, and `--blink-hz` must match the beacon (13 Hz needs about 60 fps or more; use about 7 Hz at 30 fps). There is no AprilTag pose in this mode yet, so `lean_deg` is 0 and `range_m` is the nominal 3 m.

## Telemetry

One JSON message per frame (about 120 Hz):

```json
{"t": 12.508, "state": "TRACK", "drift_mrad": 3.1, "jitter_mrad": 0.8, "residual_mrad": 0.4,
 "gimbal_lag_mrad": 0.3, "lean_deg": 4.2, "range_m": 3.05, "intensity": 0.92,
 "rx_signal": 0.88, "rx_background": 0.04}
```

`rx_signal` / `rx_background` exist only when the IR data link is built (otherwise `null`). CSV files for replay use the same columns.

## What the demo shows

* **Causality:** fan off, low, high: QBER rises with the measured motion (`tools/tune.py`, dashboard).
* **Ground tracking matters:** the virtual receiver half-width is 10 mrad, so a few mrad of residual visibly costs signal.
* **Two error bars:** ground residual (what the gimbal and tracker failed to remove) next to drone lean (what only a drone-side steering mirror could fix): the D3 roadmap argument.
* **Aborts work:** a gust or Eve pushes the pooled block QBER past 11%, the gate aborts and the tracker re-acquires.

Finite-size note: a 1 s block at 1 GHz gives about 150k sifted bits; keys appear up to roughly 5% QBER per block and get scarce above that (the 11% gate is the asymptotic bound). Use `--block` or `--pool` for larger blocks.

## Operating point

The design says to choose `mu`, `eta_d` and `p_dark` so the on-axis signal is "about 10 times the noise" and also that QBER sits "near 2-3% when locked". The formula cannot do both: 10x noise gives about 6.4%. The code follows the 2-3% statement (and the Eve and PNS figures that depend on it): `--snr 50` gives about 3%. Use `--snr 10` for the other reading.

## Tests

```bash
pytest tests/ -v
```

No camera or hardware is used; the engine runs in seeded test mode (`tests/conftest.py`). The full suite takes several minutes (the tracker tests render simulated camera frames).

## Docs

* `docs/system_architecture.md`: how the design maps onto the code.
* `docs/hardware_handoff.md`, `docs/hardware_bill_of_materials.md`: what the rig needs and how to bring up the camera.
* `docs/live_demo_runbook.md`: the demo script and fallbacks.
* `docs/calibration_log.md`: preflight calibration and operating-point log (templates).
* `docs/threat_model.md`: intercept-resend, PNS / decoy states, what is out of scope.
* `docs/d3_engineering_design.md`: D3 payload design and roadmap.
* `docs/transfer_function_derivation.md`: superseded (the previous linear model), kept for history.
* `CONTEXT.md`: the hackathon brief. `TODO.md`: what is left.
