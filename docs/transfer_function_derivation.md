> **Superseded.** This describes the previous linear model
> (`P_error = e_optical + alpha * drift + beta * jitter + gamma / intensity`), which is no longer
> in the code. So are the tools it names (`tools/camera_tune.py`, `tools/summarize_telemetry.py`)
> and HSV detection. The current model is the error budget in `src/error_budget.py`, described
> in `docs/system_architecture.md`; calibration is in `docs/hardware_handoff.md`. Kept for
> history only.

# Transfer Function: Physical Basis and Calibration Plan

`P_error = e_optical + alpha * drift + beta * jitter + gamma / intensity`  (capped at 0.5)

This is a **model**, not a measured law. It maps the three tracking observables to a channel
error rate. The cause of a bad link (fan, gust, bird, dust) does not matter; only its effect on
drift, jitter and intensity does. We therefore do **not** fit the constants to the fan.

## 1. Where each constant comes from

### `gamma` (derived from signal-to-noise)
Detected clicks are signal plus a fixed background (dark counts, stray light). Background clicks
are random, so half of them are wrong. With signal proportional to `intensity` and background `B`
relative to the full-signal level `S_ref`:

    QBER ~ e_optical + 0.5 * (B / S_ref) / intensity      =>  gamma = 0.5 * B / S_ref

`gamma` is half the background-to-signal ratio at full signal. the old `gamma = 0.012` meant background
is 2.4% of full signal.

**Design requirement:** losing the signal (lens blocked, intensity 0.95 -> 0.20) must cross the
11% gate: `e_optical + gamma / 0.2 > 0.11`, so `gamma >~ 0.019` (with `e_optical = 0.015`; about 0.021 with `e_optical = 0.004`). The old `0.012` failed this
(`test_obstruction_trips_gate` was xfail); `gamma` is now `0.0215`. With `gamma = 0.02`, `tools/tune.py` gives
calm 4.2% QBER (all keys), fan 20.7% (all abort), obstruction 12.0% (96% abort, 4% reconcile
failure, no key either way). Applied: `gamma = 0.0215` with `e_optical = 0.004`: calm 3.3% (all keys), fan 20.2% (all abort), obstruction 11.6% (89% abort, 11% reconcile failure; no key issued in any case). A higher `gamma` (0.025) with `e_optical = 0.015` gave more obstruction margin but raised calm QBER to 4.8%, which broke the decoy-state and pooling tests, so `e_optical` was lowered instead. Obstruction margin over the gate is thin (about 0.6 points); it never issues a key, but re-check on the real rig.

### `e_optical` (assumed floor)
Error of a perfect, perfectly aligned link: detector dark counts and imperfect polarisation
visibility. 0.4% corresponds to roughly 99% visibility, an optimistic but achievable figure for a
well-aligned receiver. It was lowered from 1.5% so that `gamma` could be raised for obstruction
detection without pushing calm QBER too high. This is an assumption, not a measurement of the rig.

### `alpha`, `beta` (anchored, not derived)
Pointing offset and motion cost signal and add noise, but the exact mapping depends on beam
width, aperture and detector, which we cannot know for a camera standing in for a receiver.
Part of the effect is already captured by `intensity` (offset lowers received power), so these
two terms are deliberately small and act as a pointing-quality penalty. We fix them by anchors:

* Calm hover (baseline drift `d_c`, jitter `j_c`) gives about 3-4% total.
* A severely degraded track (drift `d_bad`, jitter `j_bad`, set as a multiple of the calm
  baseline or a fraction of the field of view) gives above 11%.

Given the anchors, `alpha` and `beta` follow by solving the two conditions. The anchors are our
design choice and should be stated as such.

## 2. What is calibrated in person

Calibrate **the measurement chain and the baseline**, not the fan.

| Item | How | Why |
|---|---|---|
| HSV thresholds | `tools/camera_tune.py`, circle only the LED | LED color/brightness depends on their camera and room |
| Exposure/gain lock | Lock manually, check intensity is steady at calm hover | Auto-exposure hides intensity drops |
| Boresight pixel | `c` in `camera_tune.py` during calm hover | Drift is measured from here |
| Full-signal intensity `S_ref` | Same capture | Defines `intensity = 1.0` |
| Calm baseline (`d_c`, `j_c`) | About 10 s calm capture at the start of the demo | Lets constants be relative to their camera, distance and fps |
| Lock-loss check | Cover the lens briefly | Confirms intensity falls toward 0 and tracking returns |
| fps and resolution used | Note them | `jitter` scales with fps; coordinates are normalised to `norm_width` |

If any time is left, record 20-30 s of whatever turbulence they provide and run
`tools/summarize_telemetry.py` to see where the real telemetry sits. Use it to check the anchors
are sane, not to tune the constants to hit a target.

## 3. What is fixed in advance

`e_optical`, `gamma` (from the obstruction requirement) and the anchor definitions for `alpha`
and `beta` (as multiples of the calm baseline). The one code change needed to use baseline-
relative drift and jitter is still on the TODO list.

## 4. Claims we can make

* The constants follow a stated physical model (SNR for `gamma`, assumed visibility for
  `e_optical`) and fixed anchors, not a tuned fit to one fan setting.
* Absolute accuracy against a real quantum link is not claimed. This is a cyber sandbox model.
