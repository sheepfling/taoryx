from __future__ import annotations

import numpy as np
import pytest

from taoryx.runtime import (
    InteractiveSession,
    RuntimeProblem,
    RuntimeState,
    RuntimeVehicle,
    SensorBinding,
    SensorBus,
    SensorClockSpec,
)
from taoryx.runtime.sensor_scenario import SensorScenarioSpec
from taoryx.sensors import ImuErrorModelAdapter, TruthPoint


def _truth(state: RuntimeState) -> TruthPoint:
    velocity = np.array([state.values[0], 0.0, 0.0])
    return TruthPoint(
        time_s=state.time,
        position_eci_m=np.zeros(3),
        velocity_eci_mps=velocity,
        velocity_without_gravity_eci_mps=velocity,
        orientation_eci_from_body=np.eye(3),
        gravity_eci_mps2=np.zeros(3),
        angular_rate_body_radps=np.zeros(3),
    )


def test_interactive_session_delivers_measurements_at_accepted_steps() -> None:
    vehicle = RuntimeVehicle(
        "vehicle",
        RuntimeState(0.0, (0.0,)),
        lambda _: (1.0,),
        step_size=0.25,
        truth_provider=_truth,
    )
    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=0.3)
    bus = SensorBus()
    bus.register(
        SensorBinding(
            "imu",
            vehicle.name,
            SensorClockSpec("imu", "imu", cadence_s=0.1),
            ImuErrorModelAdapter.from_config(seed=12),
        )
    )
    bus.attach(problem)
    received: list[float] = []
    bus.subscribe("imu", lambda packet: received.append(packet.sampled_at_s))

    session = InteractiveSession(problem)
    session.step(0.1)
    session.step(0.1)

    assert received == pytest.approx([0.0, 0.1, 0.2])
    assert [snapshot.time_end for snapshot in session.snapshots] == pytest.approx([0.1, 0.2])
    assert all(state.time in {0.0, 0.1, 0.2} for state in vehicle.history)


def test_interactive_session_can_rebind_a_sidecar_and_project_sensor_artifact(tmp_path) -> None:
    vehicle = RuntimeVehicle(
        "vehicle",
        RuntimeState(0.0, (0.0,)),
        lambda _: (1.0,),
        step_size=0.1,
        truth_provider=_truth,
    )
    session = InteractiveSession(RuntimeProblem({vehicle.name: vehicle}, final_time=0.2))
    runtime = session.attach_sensor_scenario(
        SensorScenarioSpec("imu", vehicle.name, provider="ideal", cadence_s=0.1, estimator_modes=("dead-reckoning",))
    )

    session.step(0.1)
    artifact = session.to_run_artifact()

    assert runtime.bus.packets("imu")
    assert artifact.sensor_execution["spec"]["provider"] == "ideal"

    checkpoint = session.save_checkpoint(tmp_path / "sensor.checkpoint.json")
    restored_vehicle = RuntimeVehicle(
        "vehicle",
        RuntimeState(0.0, (0.0,)),
        lambda _: (1.0,),
        step_size=0.1,
        truth_provider=_truth,
    )
    restored = InteractiveSession.load_checkpoint(checkpoint, RuntimeProblem({"vehicle": restored_vehicle}, final_time=0.2))

    assert restored.problem.metadata["checkpoint_sensor_rebind"]["required"] is True
    rebound = restored.attach_sensor_scenario(SensorScenarioSpec("imu", "vehicle", provider="ideal", cadence_s=0.1, estimator_modes=("dead-reckoning",)))
    assert rebound.bus.initialized
