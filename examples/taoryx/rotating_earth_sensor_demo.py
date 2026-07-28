"""Generate isolated ECI Earth-rate sensor evidence without changing family baselines."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.runtime import RuntimeProblem, RuntimeState, RuntimeVehicle, SensorScenarioSpec, attach_sensor_scenario  # noqa: E402
from taoryx.runtime.engine import compute_trajectories  # noqa: E402
from taoryx.sensors import TruthPoint  # noqa: E402

EARTH_RATE_RAD_S = 7.2921151467e-5
EARTH_RADIUS_M = 6_378_137.0


class RotatingEarthTransportTruth:
    """Kinematic equatorial Earth-transport fixture for sensor-frame evidence."""

    contract = {
        "mode": "rotating-earth-transport-fixture",
        "frame": "ECI",
        "support_mode": "transport-only",
        "earth_rate": {"mode": "nominal", "omega_rad_s": EARTH_RATE_RAD_S},
        "translation": {"available": True, "source": "kinematic-earth-transport-fixture"},
        "orientation": {"available": True, "source": "earth-fixed-body-transport"},
        "angular_rate": {"available": True, "source": "earth-rate-transport"},
        "acceleration": {"available": True, "source": "kinematic-transport-acceleration"},
        "baseline_policy": "does-not-modify-omega-zero-family-fixtures",
    }

    def __call__(self, state: RuntimeState) -> TruthPoint:
        angle = EARTH_RATE_RAD_S * state.time
        position = EARTH_RADIUS_M * np.array([np.cos(angle), np.sin(angle), 0.0])
        velocity = EARTH_RATE_RAD_S * EARTH_RADIUS_M * np.array([-np.sin(angle), np.cos(angle), 0.0])
        acceleration = -(EARTH_RATE_RAD_S**2) * position
        orientation = np.array(
            [
                [np.cos(angle), -np.sin(angle), 0.0],
                [np.sin(angle), np.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        return TruthPoint(
            time_s=state.time,
            position_eci_m=position,
            velocity_eci_mps=velocity,
            velocity_without_gravity_eci_mps=velocity,
            orientation_eci_from_body=orientation,
            gravity_eci_mps2=np.zeros(3),
            angular_rate_body_radps=np.array([0.0, 0.0, EARTH_RATE_RAD_S]),
            acceleration_eci_mps2=acceleration,
        )


def run(output_dir: Path) -> dict[str, object]:
    truth_provider = RotatingEarthTransportTruth()
    vehicle = RuntimeVehicle(
        "earth-fixed-transport",
        RuntimeState(0.0, (0.0,), value_names=("fixture",)),
        derivative=lambda _: (0.0,),
        step_size=0.1,
        truth_provider=truth_provider,
    )
    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=1.0)
    spec = SensorScenarioSpec(
        sensor_name="imu",
        vehicle_name=vehicle.name,
        scenario_id="rotating_earth_sensor_v1",
        provider="ideal",
        truth_mode="vehicle",
        cadence_s=0.1,
        estimator_modes=("dead-reckoning", "mekf"),
        earth_rate_mode="nominal",
        earth_omega_rad_s=EARTH_RATE_RAD_S,
    )
    runtime = attach_sensor_scenario(problem, spec)
    result = compute_trajectories(problem, max_steps=20)
    runtime.finalize(completed=result.completed, stop_reason=result.stop_reason, max_steps=20)
    output_dir.mkdir(parents=True, exist_ok=True)
    runtime.write_artifacts(output_dir, 1)
    execution = runtime.artifact()
    report = {
        "artifact_id": "taoryx.rotating-earth-sensor.v1",
        "completed": result.completed,
        "stop_reason": result.stop_reason,
        "baseline_policy": "does-not-modify-omega-zero-family-fixtures",
        "truth_contract": truth_provider.contract,
        "sensor_execution": execution,
    }
    (output_dir / "rotating-earth-report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    packets = execution["measurements"]["imu"]
    valid = [packet for packet in packets if packet["valid"]]
    figure, axes = plt.subplots(2, 1, figsize=(8.0, 6.0), sharex=True, layout="constrained")
    times = [float(packet["sampled_at_s"]) for packet in valid]
    rates = [float(packet["payload"]["delta_theta_body_rad"][2]) / 0.1 for packet in valid]
    axes[0].plot(times, rates, marker=".", label="IMU gyro z rate")
    axes[0].axhline(EARTH_RATE_RAD_S, color="#b91c1c", linestyle="--", label="nominal Earth rate")
    axes[0].set_ylabel("rad/s")
    axes[0].legend()
    axes[1].plot(times, [float(packet["payload"]["delta_v_body_mps"][0]) for packet in valid], marker=".")
    axes[1].set_xlabel("accepted sample time (s)")
    axes[1].set_ylabel("delta-v x (m/s)")
    for axis in axes:
        axis.grid(True, alpha=0.25)
    figure.suptitle("Rotating-Earth ECI transport sensor evidence")
    figure.savefig(output_dir / "rotating-earth-sensor.png", dpi=140)
    plt.close(figure)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    arguments = parser.parse_args()
    report = run(arguments.output_dir.resolve())
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["completed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
