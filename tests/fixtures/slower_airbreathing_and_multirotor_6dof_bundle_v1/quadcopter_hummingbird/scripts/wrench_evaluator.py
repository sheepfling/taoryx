from __future__ import annotations

import argparse
import json
import math
from typing import Sequence

import numpy as np

MASS_KG = 0.500
ARM_M = 0.17
K_ETA = 5.57e-6
K_M = 1.36e-7
K_D = 1.19e-4
K_Z = 2.32e-4
K_H = 3.39e-3
C_D = np.diag([0.005, 0.005, 0.010])
ROTOR_DIRECTIONS = np.array([1.0, -1.0, 1.0, -1.0])
D = ARM_M / math.sqrt(2.0)
ROTOR_POSITIONS = np.array([[D,D,0.0],[D,-D,0.0],[-D,-D,0.0],[-D,D,0.0]])


def skew(vector: np.ndarray) -> np.ndarray:
    return np.array([[0.0,-vector[2],vector[1]],[vector[2],0.0,-vector[0]],[-vector[1],vector[0],0.0]])
####


def evaluate(
    velocity: Sequence[float],
    rates: Sequence[float],
    rotor_speeds: Sequence[float],
) -> dict[str, list[float]]:
    v = np.asarray(velocity, dtype=float)
    w = np.asarray(rates, dtype=float)
    omega = np.asarray(rotor_speeds, dtype=float)
    local_v = v[:,None] + skew(w) @ ROTOR_POSITIONS.T
    thrust = np.zeros((3,4), dtype=float)
    thrust[2,:] = K_ETA*omega**2 + K_H*(local_v[0,:]**2 + local_v[1,:]**2)
    rotor_drag = -omega*(np.diag([K_D,K_D,K_Z]) @ local_v)
    body_drag = -np.linalg.norm(v)*C_D @ v
    force_each = thrust + rotor_drag
    force = np.sum(force_each, axis=1) + body_drag
    moment = np.sum(np.cross(ROTOR_POSITIONS, force_each.T), axis=0)
    moment[2] += np.sum(ROTOR_DIRECTIONS*K_M*omega**2)
    return {"force_body_N": force.tolist(), "moment_body_Nm": moment.tolist()}
####


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--velocity", nargs=3, type=float, default=[0.0,0.0,0.0])
    parser.add_argument("--rates", nargs=3, type=float, default=[0.0,0.0,0.0])
    parser.add_argument("--rotors", nargs=4, type=float, required=True)
    args = parser.parse_args()
    print(json.dumps(evaluate(args.velocity, args.rates, args.rotors), indent=2))
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
####
