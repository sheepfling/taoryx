"""Run a runtime-scheduled ECI IMU stream through two navigation consumers.

The runtime owns accepted boundaries and delivery. The navigation consumers see
only measurement packets, never the runtime truth provider. An optional
``imu-error-model`` profile exercises profile provenance and output scaling.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from taoryx.navigation import DeadReckoningNavigator, MultiplicativeEkf, NavigationState
from taoryx.runtime import RuntimeProblem, RuntimeState, RuntimeVehicle, SensorBinding, SensorBus, SensorClockSpec
from taoryx.runtime.engine import compute_trajectories
from taoryx.sensors import ImuErrorModelAdapter, TruthPoint

EARTH_RATE_RADPS = 7.2921151467e-5


def truth_provider(state: RuntimeState) -> TruthPoint:
    time_s = state.time
    velocity_eci = np.array([state.values[0], 0.0, 0.0])
    angle = EARTH_RATE_RADPS * time_s
    orientation = np.array(
        [
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    return TruthPoint(
        time_s=time_s,
        position_eci_m=np.array([0.5 * time_s * time_s, 0.0, 0.0]),
        velocity_eci_mps=velocity_eci,
        velocity_without_gravity_eci_mps=velocity_eci,
        orientation_eci_from_body=orientation,
        gravity_eci_mps2=np.zeros(3),
        angular_rate_body_radps=np.array([0.0, 0.0, EARTH_RATE_RADPS]),
        acceleration_eci_mps2=np.array([1.0, 0.0, 0.0]),
    )


def _packet_payload(packet: object) -> dict[str, object]:
    payload = getattr(packet, "payload", None)
    result: dict[str, object] = {
        "valid": bool(getattr(packet, "valid")),
        "sampled_at_s": float(getattr(packet, "sampled_at_s")),
        "available_at_s": float(getattr(packet, "available_at_s")),
        "interval_start_s": getattr(packet, "interval_start_s"),
    }
    if payload is None:
        result["payload"] = None
    else:
        result["delta_v_body_mps"] = np.asarray(payload.delta_v_body_mps).tolist()
        result["delta_theta_body_rad"] = np.asarray(payload.delta_theta_body_rad).tolist()
    return result


def run(profile_path: str | None, output_path: Path) -> dict[str, object]:
    adapter = ImuErrorModelAdapter.from_profile(profile_path, seed=41) if profile_path else ImuErrorModelAdapter.from_config(seed=41)
    cadence_s = float(adapter.provenance.get("sample_period_s", 0.1))
    vehicle = RuntimeVehicle(
        "demo",
        RuntimeState(0.0, (0.0,)),
        lambda _: (1.0,),
        step_size=max(0.1, cadence_s * 5.0),
        truth_provider=truth_provider,
    )
    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=1.0)
    clock = SensorClockSpec("imu", "imu", cadence_s=cadence_s)
    bus = SensorBus()
    bus.register(SensorBinding("imu", vehicle.name, clock, adapter, provenance=adapter.provenance))
    bus.attach(problem)
    initial = NavigationState(0.0, np.zeros(3), np.zeros(3), np.eye(3))
    dead_reckoning = DeadReckoningNavigator(initial, np.zeros(3))
    mekf = MultiplicativeEkf(initial, np.eye(15), np.zeros(3))
    dead_history: list[tuple[float, float]] = []
    mekf_history: list[tuple[float, float]] = []

    def consume(packet: object) -> None:
        typed = packet
        dead_reckoning.propagate(typed)
        mekf.propagate(typed)
        if getattr(packet, "valid"):
            dead_history.append((dead_reckoning.state.time_s, float(dead_reckoning.state.position_eci_m[0])))
            mekf_history.append((mekf.state.time_s, float(mekf.state.position_eci_m[0])))

    bus.subscribe("imu", consume)
    result = compute_trajectories(problem)
    bus.release_available(2.0)
    truth = truth_provider(vehicle.state)
    packets = bus.packets("imu")
    artifact: dict[str, object] = {
        "schema_version": 1,
        "artifact_id": "taoryx.runtime-imu-navigation.v1",
        "completed": result.completed,
        "stop_reason": result.stop_reason,
        "frame": "ECI",
        "profile": dict(adapter.provenance),
        "sensor_bus": bus.to_metadata(),
        "truth_final": {
            "time_s": truth.time_s,
            "position_eci_m": truth.position_eci_m.tolist(),
            "velocity_eci_mps": truth.velocity_eci_mps.tolist(),
            "orientation_eci_from_body": truth.orientation_eci_from_body.tolist(),
        },
        "dead_reckoning_final": {
            "position_eci_m": dead_reckoning.state.position_eci_m.tolist(),
            "velocity_eci_mps": dead_reckoning.state.velocity_eci_mps.tolist(),
        },
        "mekf_final": {
            "position_eci_m": mekf.state.position_eci_m.tolist(),
            "velocity_eci_mps": mekf.state.velocity_eci_mps.tolist(),
            "position_sigma_m": np.sqrt(np.diag(mekf.covariance)[0:3]).tolist(),
        },
        "packets": [_packet_payload(packet) for packet in packets],
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    figure, axis = plt.subplots(figsize=(8, 4.5))
    axis.plot([item[0] for item in dead_history], [item[1] for item in dead_history], label="dead reckoning")
    axis.plot([item[0] for item in mekf_history], [item[1] for item in mekf_history], label="MEKF")
    axis.plot(truth.time_s, truth.position_eci_m[0], "o", label="truth endpoint")
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("ECI X position (m)")
    axis.set_title("Accepted-truth IMU navigation example")
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output_path.with_suffix(".png"), dpi=140)
    plt.close(figure)
    return artifact


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", help="validated imu-error-model profile YAML/JSON path")
    parser.add_argument("--output", type=Path, default=Path("artifacts/imu_navigation/runtime_imu_navigation.json"))
    arguments = parser.parse_args()
    run(arguments.profile, arguments.output)
