# Threat Model (optional challenge item)

Scope: what the software BB84 sandbox demonstrates against an eavesdropper (Eve), and what it
does not. Everything here runs in `src/qkd_core.py` and `src/decoy.py`.

## Assumptions
- The classical channel (sifting, parity exchange) is **authenticated** (e.g. a pre-shared
  seed used for message authentication). It is public but tamper-proof. The sandbox models it
  as in-memory calls; authentication is not implemented.
- Eve controls the optical path but not the endpoints.
- Telemetry (drift / jitter / intensity) sets the *natural* error rate. Any extra error is
  attributed to Eve, which is the conservative QKD assumption.

## 1. Environmental disturbance (not an attacker)
Turbulence, drone drift or an obstruction raises `P_error` through the transfer function. The
protocol cannot tell this from an attack, so it aborts when QBER reaches 11%. This costs
availability, never secrecy.

## 2. Intercept-resend
Eve measures a fraction `f` of pulses in a random basis and resends them.

- She picks the wrong basis half the time, and then Bob's matched-basis bit is wrong half the time.
- Expected extra QBER on the sifted key: **25% x f**.
- Demo: `python src/demo_runner.py --source csv --eve 1.0` aborts every window.
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
- **Telemetry coupling:** link transmittance `eta = eta0 * intensity` and optical error
  `e_d = P_error`.
- Demo: `python src/demo_runner.py --source csv --decoy --pns` rejects every non-abort window.
  Without `--pns` the same run issues keys in calm windows.

Cost of the defence: the multi-photon share of the key is discarded, so the usable QBER
range shrinks. In decoy mode the 32k-pulse block is marginal even in calm conditions; use a
65k block or `--pool`.

## 4. Second IR channel (cross-check)
A second wavelength and receiver on the same path gives an independent measurement of alignment
and loss. If the two channels disagree, the main beam is being tapped or blocked. The
`TelemetryProvider` interface allows a second provider; comparing them is **not implemented**.

## Out of scope / not modelled
| Attack | Notes |
|---|---|
| Detector blinding, time-shift, Trojan-horse | Implementation attacks on real hardware. Need hardened detectors, monitors and isolators. |
| Beacon spoofing | An attacker with a green LED could lure the tracker. Mitigation: modulate the beacon with a pattern and verify it. |
| Beacon jamming / blinding the camera | Denial of service. The system aborts, so it fails safe. |
| Classical channel authentication | Assumed; a real system needs it. |
| Composable finite-key security | The decoy and finite-size bounds use conservative Gaussian-style margins. They are not a formal proof. |
