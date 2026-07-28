"""Run the initial IMU, dead-reckoning, and MEKF integration example.

Install with ``python -m pip install -e '.[sensors]'`` before running this
example. The truth sequence is synthetic and exists to exercise contracts, not
to represent a qualified vehicle navigation solution.
"""

from __future__ import annotations

import argparse

import numpy as np

from taoryx.navigation import DeadReckoningNavigator, MultiplicativeEkf, NavigationState
from taoryx.sensors import ImuErrorModelAdapter, TruthPoint


def _truth(time_s: float, acceleration_eci_mps2: np.ndarray, gravity_eci_mps2: np.ndarray) -> TruthPoint:
    velocity_without_gravity = acceleration_eci_mps2 * time_s
    earth_rate_radps = 7.2921151467e-5
    angle = earth_rate_radps * time_s
    return TruthPoint(
        time_s=time_s,
        position_eci_m=0.5 * (acceleration_eci_mps2 + gravity_eci_mps2) * time_s**2,
        velocity_eci_mps=(acceleration_eci_mps2 + gravity_eci_mps2) * time_s,
        velocity_without_gravity_eci_mps=velocity_without_gravity,
        orientation_eci_from_body=np.array(
            [
                [np.cos(angle), -np.sin(angle), 0.0],
                [np.sin(angle), np.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        ),
        gravity_eci_mps2=gravity_eci_mps2,
        angular_rate_body_radps=np.array([0.0, 0.0, earth_rate_radps]),
        acceleration_eci_mps2=acceleration_eci_mps2 + gravity_eci_mps2,
    )


def main(profile_path: str | None = None) -> None:
    gravity_eci = np.array([0.0, 0.0, -9.81])
    specific_force = np.array([1.0, 0.0, 0.0])
    adapter = ImuErrorModelAdapter.from_profile(profile_path, seed=17) if profile_path is not None else ImuErrorModelAdapter.from_config(seed=17)
    initial = NavigationState(0.0, np.zeros(3), np.zeros(3), np.eye(3))
    dead_reckoning = DeadReckoningNavigator(initial, gravity_eci)
    mekf = MultiplicativeEkf(initial, np.eye(15), gravity_eci)
    for time_s in np.arange(0.0, 10.0 + 1.0e-12, 0.1):
        truth = _truth(float(time_s), specific_force, gravity_eci)
        packet = adapter.sample(truth)
        dead_reckoning.propagate(packet)
        mekf.propagate(packet)
        if packet.valid and int(round(time_s * 10.0)) % 10 == 0:
            mekf.update_position(truth.position_eci_m, np.eye(3) * 0.25)
    print("dead_reckoning_position_eci_m", dead_reckoning.state.position_eci_m)
    print("mekf_position_eci_m", mekf.state.position_eci_m)
    print("mekf_position_sigma_m", np.sqrt(np.diag(mekf.covariance)[0:3]))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", help="validated imu-error-model profile YAML/JSON path")
    main(parser.parse_args().profile)
