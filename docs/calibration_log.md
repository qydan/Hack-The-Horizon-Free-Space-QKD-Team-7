# Calibration Log

Record of the preflight calibration and the operating point used for each session, so a run can
be explained afterwards. No real entries yet: the blocks below are templates.

## What is calibrated

| Item | How | Stored in |
|---|---|---|
| Camera settings (fps, `min_amplitude`, exposure) | `python tools/camera_check.py` | command line of `tracker.py` |
| Boresight (calm-hover direction) and `i_ref` (full-signal beacon brightness) | `python src/tracker.py --source camera --calibrate 15 --duration 1` | `calibration.json` |
| Operating point (`on_axis_snr`, sets `p_dark` and the locked QBER) | `--snr` on `demo_runner.py`, default 50 (about 3% locked) | command line |

## Model (see `src/error_budget.py`)

`eta = eta_geo(L) * exp(-2 theta_rx^2 / theta_fov^2) * eta_tx(lean + drift) * I / I_cal`

`QBER = (e_opt p_sig + p_noise / 2) / (p_sig + p_noise)`, with `e_opt = 0.02`, `mu = 0.5`,
`eta_d = 0.6`, `theta_fov = 10 mrad`. `python tools/tune.py` prints where the 11% gate sits for
a given `--snr` and `--fov`.

*Targets:* QBER about 3% in calm hover with keys issued, rising with the fan, and at or above
11% (abort) in a gust or when the beacon is blocked.

---

## Log Entries

### [Template] Date: YYYY-MM-DD | Camera check
* **Camera / lens / hfov:** `__`
* **Measured fps, dropped frames:** `__ fps`, `__ %`
* **Blink rate:** `__ Hz`
* **Noise floor / beacon amplitude / SNR:** `__` / `__` / `__`
* **Settings used:** `--fps __ --blink-hz __ --min-amplitude __ --proc-width __ --zone __`
* **Notes:**

### [Template] Date: YYYY-MM-DD | Calm-hover calibration
* **Conditions:** fan OFF, range `__ m`, lighting `__`
* **`calibration.json`:** boresight (`__`, `__`) mrad, `i_ref = __`
* **Calm telemetry (mean):** drift `__ mrad`, jitter `__ mrad`, residual `__ mrad`, intensity `__`
* **`--snr`:** `__`  **Resulting block QBER:** `__ %`  **Keys issued:** `__ of __ blocks`
* **Notes:**

### [Template] Date: YYYY-MM-DD | Fan run
* **Conditions:** fan setting `__`, distance from drone `__ m`
* **Telemetry (mean / peak):** drift `__`, jitter `__`, residual `__` mrad
* **Resulting block QBER:** `__ %`  **Outcomes:** `__ key, __ abort, __ no_key, __ no_link`
* **Lock losses / re-acquisitions:** `__`
* **Notes:**

### [Template] Date: YYYY-MM-DD | Obstruction test
* **Conditions:** hand passed in front of the lens for `__ s`
* **Observed:** state went `TRACK -> COAST -> SEARCH`? `__`  Time to re-lock: `__ s`
* **Outcomes during the obstruction:** `__` (expect `no_link` or `abort`, never `key`)
* **Notes:**
