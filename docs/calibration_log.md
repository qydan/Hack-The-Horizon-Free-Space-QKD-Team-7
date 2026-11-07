# Physical-to-Quantum Calibration Log

To ensure the "Cyber Sandbox" accurately models real-world quantum channel degradation, we must calibrate our transfer function constants (`alpha`, `beta`, `gamma`) against the specific physical characteristics of our hardware rig and desk fan.

## Mathematical Model
`P_error = e_optical + (alpha * drift) + (beta * jitter) + (gamma / intensity)`

*Target Threshold:* We want a QBER of ~3% during calm hover, and >11% when the fan is on HIGH.

---

## Log Entries

### [Template] Date: YYYY-MM-DD | Test: Initial Baseline
* **Conditions:** Fan OFF, Drone powered on, Gimbal active.
* **Observed Telemetry:**
  * Average Drift: `__ px`
  * Average Jitter: `__ px/s`
  * Average Intensity: `__`
* **Constants Used:** `e_optical = 0.015`, `alpha = 0.0015`, `beta = 0.0012`, `gamma = 0.012`
* **Resulting QBER:** `__ %`
* **Notes:** (e.g., "Baseline is too noisy, need to decrease beta slightly so natural gimbal hum doesn't trigger false aborts.")

### [Template] Date: YYYY-MM-DD | Test: High Turbulence (Fan ON)
* **Conditions:** Fan on HIGH, positioned 1m from drone.
* **Observed Telemetry:**
  * Peak Jitter: `__ px/s`
* **Constants Used:** [Insert current constants]
* **Resulting QBER:** `__ %`
* **Notes:** (e.g., "The fan caused the QBER to hit 14%. The 11% abort gate tripped successfully. No tuning required.")

### [Template] Date: YYYY-MM-DD | Test: Intensity/Obstruction Test
* **Conditions:** Hand briefly passed in front of the lens to simulate a bird or dust cloud.
* **Observed Telemetry:** Intensity dropped from `0.95` to `0.20`.
* **Constants Used:** `gamma = __`
* **Resulting QBER:** `__ %`
* **Notes:** (e.g., "Need to increase gamma so that signal loss immediately spikes the error rate above 11%.")