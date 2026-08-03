from __future__ import annotations

import numpy as np
import pytest

from taoryx.integration import rkf45_step
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
from taoryx.simulation.contracts import SimulationState


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


def test_interactive_checkpoint_reconstructs_declared_sensor_and_estimator_sidecar(tmp_path) -> None:
    vehicle = RuntimeVehicle(
        "vehicle",
        RuntimeState(0.0, (0.0,)),
        lambda _: (1.0,),
        step_size=0.1,
        truth_provider=_truth,
    )
    session = InteractiveSession(RuntimeProblem({vehicle.name: vehicle}, final_time=0.3))
    runtime = session.attach_sensor_scenario(
        SensorScenarioSpec(
            "imu",
            vehicle.name,
            provider="ideal",
            cadence_s=0.1,
            delivery_s=0.1,
            drop_every_n=2,
            estimator_modes=("dead-reckoning",),
        )
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
    restored = InteractiveSession.load_checkpoint(checkpoint, RuntimeProblem({"vehicle": restored_vehicle}, final_time=0.3))

    assert restored.problem.metadata["sensor_rebind"] == {
        "required": False,
        "scenario_id": "sensor-scenario-v1",
        "status": "restored-from-checkpoint",
    }
    assert restored.problem.sensor_bus is not None
    assert restored.problem.sensor_bus.initialized

    session.step(0.2)
    restored.step(0.2)

    original_packets = runtime.bus.packets("imu")
    restored_packets = restored.problem.sensor_bus.packets("imu")
    assert [packet.sampled_at_s for packet in restored_packets] == pytest.approx(
        [packet.sampled_at_s for packet in original_packets]
    )
    assert [packet.valid for packet in restored_packets] == [packet.valid for packet in original_packets]
    assert [packet.sampled_at_s for packet in original_packets] == pytest.approx([0.0, 0.2])
    assert restored.problem.sensor_bus.bindings[0].dropped_samples == runtime.bus.bindings[0].dropped_samples == 2


def test_declared_sensor_checkpoint_rejects_unregistered_drop_callback() -> None:
    vehicle = RuntimeVehicle(
        "vehicle",
        RuntimeState(0.0, (0.0,)),
        lambda _: (1.0,),
        step_size=0.1,
        truth_provider=_truth,
    )
    session = InteractiveSession(RuntimeProblem({vehicle.name: vehicle}, final_time=0.1))
    runtime = session.attach_sensor_scenario(
        SensorScenarioSpec(
            "imu",
            vehicle.name,
            provider="ideal",
            cadence_s=0.1,
            estimator_modes=("dead-reckoning",),
        )
    )
    runtime.bus.bindings[0].drop_predicate = lambda _: False

    with pytest.raises(TypeError, match="unregistered drop callback"):
        runtime.checkpoint_payload()


def test_interactive_adaptive_session_preserves_sensor_boundaries_after_rejections() -> None:
    trial = rkf45_step(
        lambda state: (10.0 * state.values[0],),
        SimulationState(0.0, (1.0,), "ECFC"),
        0.05,
        1.0e-10,
        1.0e-10,
    )
    assert trial.rejected_steps > 0
    vehicle = RuntimeVehicle(
        "vehicle",
        RuntimeState(0.0, (1.0,)),
        lambda state: (10.0 * state.values[0],),
        step_size=0.1,
        integrator="rkf45",
        absolute_tolerance=1.0e-10,
        relative_tolerance=1.0e-10,
        truth_provider=_truth,
    )
    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=0.2)
    bus = SensorBus()
    bus.register(
        SensorBinding(
            "imu",
            vehicle.name,
            SensorClockSpec("imu", "imu", cadence_s=0.05),
            ImuErrorModelAdapter.from_config(seed=23),
        )
    )
    bus.attach(problem)
    session = InteractiveSession(problem)

    snapshot = session.step(0.2)

    assert snapshot.time_end == pytest.approx(0.2)
    assert [packet.sampled_at_s for packet in bus.packets("imu")] == pytest.approx([0.0, 0.05, 0.10, 0.15, 0.20])
    assert all(state.time <= 0.2 + 1.0e-12 for state in vehicle.history)
    ####
