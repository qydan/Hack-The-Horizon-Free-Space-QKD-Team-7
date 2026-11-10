"""Optional IR data-link sensor: 940 nm LED on the drone -> PIN photodiode beside the camera (section 10).

This is a *simulated sensor*: like the camera, it may use world state (it is the physics of the link, not an
estimator). It publishes only what a real photodiode front end would measure.

Per 1 ms frame:
  P_rx = P_tx * (A_lens / A_spot(R)) * T * tx_beam_factor(off-axis angle at Tx) * rx_fov_factor(angle at Rx)
  A_spot = pi * (R tan(half_angle))^2      flat-top beam, soft edge over the last 10% of the angle
  S = responsivity * P_rx  (A),  B = responsivity * P_background (A)
  noise: shot  sqrt(2 q (S + B) BW), amplifier i_n sqrt(BW)
  test link: OOK at ``bit_rate_bps``; BER = 0.5 erfc(Q / sqrt 2), Q = S / (sigma_0 + sigma_1)
Publishes {t_sim, S, B, test_bit_errors} (S, B in watts of optical power at the detector) on the photodiode
port (5558).
"""
from __future__ import annotations

import argparse
import json
import logging
import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from sim_bridge import config as cfgmod

log = logging.getLogger("photodiode_sim")


def smooth_edge(angle: float, half_angle: float, soft_fraction: float) -> float:
    """1 inside (1 - soft) * half_angle, cosine roll-off to 0 at half_angle."""
    a = abs(angle)
    inner = (1.0 - soft_fraction) * half_angle
    if a <= inner:
        return 1.0
    if a >= half_angle:
        return 0.0
    x = (a - inner) / (half_angle - inner)
    return 0.5 * (1.0 + math.cos(math.pi * x))


@dataclass
class LinkBudget:
    p_tx: float
    tx_half_angle: float
    soft: float
    lens_area: float
    transmission: float
    rx_half_fov: float
    responsivity: float

    @classmethod
    def from_cfg(cls, c: dict[str, Any]) -> "LinkBudget":
        tx, rx = c["transmitter"], c["receiver"]
        if rx["pinhole"]["enabled"]:
            half_fov = 0.5 * float(rx["pinhole"]["fov_full_angle_mrad"]) * 1e-3
        else:
            half_fov = math.atan(0.5 * float(rx["detector_size_m"]) / float(rx["focal_length_m"]))
        return cls(float(tx["optical_power_w"]), math.radians(0.5 * float(tx["full_angle_deg"])),
                   float(tx["soft_edge_fraction"]), math.pi * (0.5 * float(rx["lens_diameter_m"])) ** 2,
                   float(rx["transmission"]), half_fov, float(rx["responsivity_apw"]))

    def received_power(self, range_m: float, tx_off_axis_rad: float = 0.0, rx_off_axis_rad: float = 0.0) -> float:
        spot_r = range_m * math.tan(self.tx_half_angle)
        geo = min(1.0, self.lens_area / (math.pi * spot_r * spot_r))
        return (self.p_tx * geo * self.transmission
                * smooth_edge(tx_off_axis_rad, self.tx_half_angle, self.soft)
                * smooth_edge(rx_off_axis_rad, self.rx_half_fov, 0.1))


@dataclass
class Detector:
    responsivity: float
    bandwidth: float
    i_amp: float
    q: float
    bit_rate: float
    frame_s: float
    rng: np.random.Generator

    @classmethod
    def from_cfg(cls, c: dict[str, Any], rng) -> "Detector":
        n = c["noise"]
        return cls(float(c["receiver"]["responsivity_apw"]), float(n["bandwidth_hz"]),
                   float(n["amplifier_current_noise_a_per_rthz"]), float(n["electron_charge_c"]),
                   float(c["test_link"]["bit_rate_bps"]), float(c["frame_period_s"]), rng)

    def measure(self, p_signal: float, p_background: float) -> tuple[float, float, int]:
        """Noisy (S, B) in optical watts plus test-pattern bit errors in this frame."""
        i_s, i_b = self.responsivity * p_signal, self.responsivity * p_background
        amp2 = (self.i_amp ** 2) * self.bandwidth
        sig0 = math.sqrt(2 * self.q * i_b * self.bandwidth + amp2)
        sig1 = math.sqrt(2 * self.q * (i_s + i_b) * self.bandwidth + amp2)
        # frame averages: noise reduced by sqrt(samples in frame)
        n_avg = max(1.0, self.bandwidth * 2 * self.frame_s)
        s_meas = (i_s + sig1 / math.sqrt(n_avg) * float(self.rng.standard_normal())) / self.responsivity
        b_meas = (i_b + sig0 / math.sqrt(n_avg) * float(self.rng.standard_normal())) / self.responsivity
        q_factor = i_s / (sig0 + sig1) if (sig0 + sig1) > 0 else math.inf
        ber = 0.5 * math.erfc(q_factor / math.sqrt(2.0))
        n_bits = int(round(self.bit_rate * self.frame_s))
        errors = int(self.rng.binomial(n_bits, min(0.5, ber))) if n_bits > 0 else 0
        return s_meas, b_meas, errors


def angle_between(a, b) -> float:
    a, b = np.asarray(a, float), np.asarray(b, float)
    c = float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.acos(max(-1.0, min(1.0, c)))


class PhotodiodeSensor:  # pragma: no cover - needs Gazebo
    """Reads the drone tx_frame and the receiver pose from Gazebo; publishes on the photodiode port."""

    def __init__(self, world, cfgs: dict[str, dict[str, Any]], *, rig: str, lighting_name, seed: int = 0,
                 receiver_pose_fn=None, tx_pose_fn=None):
        import zmq
        self.world = world
        self.cfg = cfgs["photodiode"]
        self.link = LinkBudget.from_cfg(self.cfg)
        self.det = Detector.from_cfg(self.cfg, np.random.default_rng(seed))
        self.lighting_name = lighting_name          # callable -> current lighting scenario name
        self.receiver_pose_fn = receiver_pose_fn    # callable -> (position, boresight unit vector)
        self.tx_pose_fn = tx_pose_fn                # callable -> (position, tx axis unit vector)
        bus = cfgs["bus"]
        self.sock = zmq.Context.instance().socket(zmq.PUB)
        self.sock.bind(f"tcp://{bus['host']}:{bus['ports']['photodiode']}")
        self.topic = bus["topics"]["photodiode"].encode()
        self.rig = rig

    def run(self, until=None, stop=lambda: False) -> None:
        from sim_bridge.sim_loop import SimTicker
        for t in SimTicker(self.world, 1.0 / float(self.cfg["frame_period_s"]), until=until, stop=stop):
            tx = self.tx_pose_fn()
            rx = self.receiver_pose_fn()
            if tx is None or rx is None:
                continue
            (p_tx, a_tx), (p_rx, a_rx) = tx, rx
            los = np.asarray(p_rx) - np.asarray(p_tx)
            rng_m = float(np.linalg.norm(los))
            p_sig = self.link.received_power(rng_m, angle_between(a_tx, los), angle_between(a_rx, -los))
            p_bg = float(self.cfg["background"].get(self.lighting_name(), 0.0))
            s, b, errs = self.det.measure(p_sig, p_bg)
            self.sock.send_multipart([self.topic, json.dumps(
                {"t_sim": t, "S": s, "B": b, "test_bit_errors": errs}).encode()])


def main() -> None:
    ap = argparse.ArgumentParser(description="Print the on-axis link budget")
    ap.add_argument("--ranges", type=float, nargs="+", default=[3.0, 5.0, 20.0, 300.0])
    args = ap.parse_args()
    link = LinkBudget.from_cfg(cfgmod.load("photodiode"))
    for r in args.ranges:
        print(f"R = {r:7.1f} m   P_rx = {link.received_power(r) * 1e6:10.3f} uW")


if __name__ == "__main__":
    main()
