# Threat Model (optional challenge item)

Scope: what the software BB84 sandbox demonstrates against an eavesdropper (Eve), and what it
does not. The Eve emulator is in `src/channel.py`, the decoy analysis in `src/decoy.py`, the
gate in `src/sandbox.py` and the BB84 pipeline in `src/qkd_core.py`.

## Assumptions
- The classical channel (sifting, parity exchange) is **authenticated** (e.g. a pre-shared
  seed used for message authentication). It is public but tamper-proof. The sandbox models it
  as in-memory calls; authentication is not implemented.
- Eve controls the optical path but not the endpoints.
- Telemetry (residual, drift, lean, range, intensity) sets the *natural* error rate through the
  error budget (`src/error_budget.py`). Any extra error is attributed to Eve, which is the
  conservative QKD assumption.

## 1. Environmental disturbance (not an attacker)
Turbulence, drone drift, lean or an obstruction lowers the transmittance in the error budget, so
noise clicks make up more of the detections and QBER rises. The protocol cannot tell this from
an attack, so it aborts when the block's pooled QBER reaches 11%. This costs availability, never
secrecy.

## 2. Intercept-resend
Eve measures a fraction `f` of pulses in a random basis and resends them.

- She picks the wrong basis half the time, and then Bob's matched-basis bit is wrong half the time.
- Expected extra QBER on the sifted key: **25% x f**.
- Demo: `python src/demo_runner.py --source csv --eve 0.5` gives about 14.5% QBER and aborts
  every block (`--eve 1.0`: about 26.5%).
- Limit (tested in `tests/test_extensions.py`): `f` around 10% adds only about 2.5% QBER and stays
  under the gate. QBER alone bounds what Eve can learn, and the privacy-amplification step removes
  that bounded information. It does not guarantee detection.

## 3. Photon-number splitting (PNS) and decoy states
Real sources are weak laser pulses, not single photons. Photon number is Poisson distributed,
so some pulses carry two or more photons. A PNS attacker keeps one photon from those pulses and
passes the rest, learning the bit without adding QBER. She also blocks single-photon pulses and
uses a lossless line to Bob to hide the loss.

- **The naive check is blind to it.** The sifted-key QBER stays at the optical error rate, so the
  11% gate passes.
- **Decoy states catch it.** Alice randomly sends signal (mu = 0.5), decoy (nu = 0.1) and vacuum
  pulses. Bob's detection rate per class must follow `Q = Y0 + 1 - exp(-eta * lambda)`. Eve cannot
  see the intensity class, so thinning the multi-photon pulses to match the signal rate leaves
  the decoy rate wrong. The analysis flags the mismatch and certifies zero single-photon yield.
- **Without an attack** the vacuum + weak-decoy bounds (Ma, Qi, Zhao, Lo 2005) give a lower bound
  on the single-photon yield `Y1` and an upper bound on its error rate `e1`. Only that share
  of the key counts as secret: `n * Delta * (1 - h2(e1)) - leakage - 2 log2(1/eps)`.
- **Telemetry coupling:** the transmittance of every pulse comes from the error budget, so the
  expected per-class gains follow the measured tracking. The check compares the observed gains
  with the click-weighted expectation over the block (`BlockCounts.expected_gains`).
- Demo: `python src/demo_runner.py --source csv --pns` rejects every block that passes the QBER
  gate (`DECOY ABORT`). Without `--pns` the same run issues keys in calm blocks.

Decoy states are always on in the sandbox. Cost of the defence: the multi-photon share of the
key is discarded, so the usable QBER range shrinks (per-block keys get scarce above about 5%
QBER; use a longer `--block` or `--pool`).

## 4. Second IR channel (cross-check)
A second wavelength and receiver on the same path gives an independent measurement of alignment
and loss. If the two channels disagree, the main beam is being tapped or blocked. This is a
design for the deployed system only: the demo has no infrared hardware and the cross-check is
**not implemented**.

## Out of scope / not modelled
| Attack | Notes |
|---|---|
| Detector blinding, time-shift, Trojan-horse | Implementation attacks on real hardware. Need hardened detectors, monitors and isolators. |
| Beacon spoofing | The tracker only accepts a spot pulsing at the blink rate (13 Hz lock-in), which rejects ordinary lights but not an attacker who blinks at the same rate. Mitigation: a keyed blink pattern, verified. With `--beacon steady` any small bright spot can lure the tracker. |
| Beacon jamming / blinding the camera | Denial of service. The system aborts, so it fails safe. |
| Classical channel authentication | Assumed; a real system needs it. |
| Composable finite-key security | The decoy and finite-size bounds use conservative Gaussian-style margins. They are not a formal proof. |
