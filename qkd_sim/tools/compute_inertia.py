"""Inertia tensor of the Mini 2 SE stand-in from its component model (section 5.2), per payload variant.

Components (config/drone.yaml):
  * body + battery: uniform box (folded L x W x H), mass ``geometry.body_mass_kg``, centred at the origin
  * 4 arm+motor point masses at (+-d/2/sqrt2, +-d/2/sqrt2, rotor_z)
  * payload point mass at ``payload.variants.<v>.offset_m``
The tensor is taken about the combined centre of mass (that is what SDF <inertial> wants, with <pose> at the
CoM).

    python tools/compute_inertia.py [--payload red_led_beacon|tag|none|all]
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from sim_bridge import config as cfgmod  # noqa: E402


def rotor_positions(drone: dict) -> list[tuple[float, float, float]]:
    """X layout, PX4/x500 order: 0 front-right, 1 rear-left, 2 front-left, 3 rear-right (FLU body frame)."""
    a = float(drone["geometry"]["motor_diagonal_m"]) / 2.0 / math.sqrt(2.0)
    z = float(drone["geometry"]["rotor_z_m"])
    return [(a, -a, z), (-a, a, z), (a, a, z), (-a, -a, z)]


def inertia(drone: dict, payload: str) -> dict:
    g = drone["geometry"]
    L, W, H = drone["spec"]["folded_lwh_m"]
    mb = float(g["body_mass_kg"])
    ma = float(g["arm_mass_kg"])
    pv = drone["payload"]["variants"][payload]
    mp = float(pv["mass_kg"])
    points = [(ma, np.array(p)) for p in rotor_positions(drone)]
    if mp > 0:
        points.append((mp, np.asarray(pv["offset_m"], float)))
    m_tot = mb + sum(m for m, _ in points)
    com = sum((m * p for m, p in points), np.zeros(3)) / m_tot      # body box is centred at the origin
    I = np.diag([mb / 12 * (W * W + H * H), mb / 12 * (L * L + H * H), mb / 12 * (L * L + W * W)])
    I = I + mb * (np.dot(com, com) * np.eye(3) - np.outer(com, com))      # box about the CoM
    for m, p in points:
        r = p - com
        I = I + m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    return {"mass_kg": m_tot, "com_m": com.tolist(), "I": I.tolist()}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--payload", default="all")
    args = ap.parse_args()
    drone = cfgmod.load("drone")
    variants = list(drone["payload"]["variants"]) if args.payload == "all" else [args.payload]
    for v in variants:
        r = inertia(drone, v)
        I = np.asarray(r["I"])
        print(f"payload={v:15s} mass={r['mass_kg']:.4f} kg  CoM=({r['com_m'][0]*1e3:+.1f}, {r['com_m'][1]*1e3:+.1f}, "
              f"{r['com_m'][2]*1e3:+.1f}) mm")
        print(f"    Ixx={I[0,0]:.3e}  Iyy={I[1,1]:.3e}  Izz={I[2,2]:.3e}  Ixy={I[0,1]:.1e}  Ixz={I[0,2]:.1e}  Iyz={I[1,2]:.1e} kg m^2")


if __name__ == "__main__":
    main()
