from __future__ import annotations

import numpy as np
import pytest

from taoryx.integration import rkf45_step
from taoryx.runtime import RuntimeProblem, RuntimeState, RuntimeVehicle, SensorBinding, SensorBus, SensorClockSpec
from taoryx.runtime.engine import compute_trajectories
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


def test_sensor_bus_samples_only_accepted_boundaries_and_releases_latency() -> None:
    vehicle = RuntimeVehicle("vehicle", RuntimeState(0.0, (0.0,)), lambda _: (1.0,), step_size=0.25, truth_provider=_truth)
    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=0.3)
    clock = SensorClockSpec("imu", "imu", cadence_s=0.1, delivery_s=0.05)
    bus = SensorBus()
    bus.register(SensorBinding("imu", vehicle.name, clock, ImuErrorModelAdapter.from_config(seed=9)))
    bus.attach(problem)
    received: list[float] = []
    bus.subscribe("imu", lambda packet: received.append(packet.sampled_at_s))

    result = compute_trajectories(problem)

    assert result.completed
    assert [packet.sampled_at_s for packet in bus.packets("imu")] == pytest.approx([0.0, 0.1, 0.2])
    assert [packet.sampled_at_s for packet in bus.packets("imu", delivered=False)] == pytest.approx([0.3])
    assert received == pytest.approx([0.0, 0.1, 0.2])
    assert all(packet.interval_start_s is None or packet.interval_start_s < packet.sampled_at_s for packet in bus.packets("imu"))
    assert all(state.time in {0.0, 0.1, 0.2, 0.3} for state in vehicle.history)


def test_interval_sensor_accumulates_accepted_segments() -> None:
    vehicle = RuntimeVehicle("vehicle", RuntimeState(0.0, (0.0,)), lambda _: (1.0,), step_size=0.05, truth_provider=_truth)
    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=0.2)
    clock = SensorClockSpec(
        "imu",
        "imu",
        cadence_s=0.1,
        sample_mode="interval",
        truth_policy="accepted-segment",
        rate_policy="accumulate",
    )
    bus = SensorBus()
    bus.register(SensorBinding("imu", vehicle.name, clock, ImuErrorModelAdapter.from_config(seed=10)))
    bus.attach(problem)

    result = compute_trajectories(problem)

    assert result.completed
    packets = bus.packets("imu")
    assert [packet.sampled_at_s for packet in packets] == pytest.approx([0.1, 0.2])
    valid = [packet for packet in packets if packet.valid]
    assert [packet.interval_start_s for packet in valid] == pytest.approx([0.0, 0.1])


def test_sensor_drop_is_accounted_for_without_delivery() -> None:
    vehicle = RuntimeVehicle("vehicle", RuntimeState(0.0, (0.0,)), lambda _: (1.0,), step_size=0.25, truth_provider=_truth)
    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=0.2)
    binding = SensorBinding(
        "imu",
        vehicle.name,
        SensorClockSpec("imu", "imu", cadence_s=0.1),
        ImuErrorModelAdapter.from_config(seed=11),
        drop_predicate=lambda packet: packet.sampled_at_s > 0.15,
    )
    bus = SensorBus()
    bus.register(binding)
    bus.attach(problem)
    received: list[float] = []
    bus.subscribe("imu", lambda packet: received.append(packet.sampled_at_s))

    result = compute_trajectories(problem)

    assert result.completed
    assert received == pytest.approx([0.0, 0.1])
    assert binding.samples_emitted == 3
    assert binding.dropped_samples == 1
    assert [packet.sampled_at_s for packet in bus.packets("imu")] == pytest.approx([0.0, 0.1])


def test_instantaneous_and_interval_imus_ignore_adaptive_rejected_trial_states() -> None:
    """A forced RKF45 rejection must not manufacture a sensor truth sample."""

    trial = rkf45_step(
        lambda state: (10.0 * state.values[0],),
        SimulationState(0.0, (1.0,), "ECFC"),
        0.05,
        1.0e-10,
        1.0e-10,
    )
    assert trial.rejected_steps > 0

    truth_times: list[float] = []

    def adaptive_truth(state: RuntimeState) -> TruthPoint:
        truth_times.append(state.time)
        return _truth(state)
        ####

    vehicle = RuntimeVehicle(
        "vehicle",
        RuntimeState(0.0, (1.0,)),
        lambda state: (10.0 * state.values[0],),
        step_size=0.1,
        integrator="rkf45",
        absolute_tolerance=1.0e-10,
        relative_tolerance=1.0e-10,
        truth_provider=adaptive_truth,
    )
    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=0.2)
    bus = SensorBus()
    bus.register(
        SensorBinding(
            "instantaneous",
            vehicle.name,
            SensorClockSpec("instantaneous", "imu", cadence_s=0.05),
            ImuErrorModelAdapter.from_config(seed=21),
        )
    )
    bus.register(
        SensorBinding(
            "interval",
            vehicle.name,
            SensorClockSpec(
                "interval",
                "imu",
                cadence_s=0.05,
                sample_mode="interval",
                truth_policy="accepted-segment",
                rate_policy="accumulate",
            ),
            ImuErrorModelAdapter.from_config(seed=22),
        )
    )
    bus.attach(problem)

    result = compute_trajectories(problem, max_steps=10000)

    assert result.completed
    assert [packet.sampled_at_s for packet in bus.packets("instantaneous")] == pytest.approx(
        [0.0, 0.05, 0.10, 0.15, 0.20]
    )
    assert [packet.sampled_at_s for packet in bus.packets("interval")] == pytest.approx([0.05, 0.10, 0.15, 0.20])
    accepted_times = [state.time for state in vehicle.history]
    assert all(any(sampled_at == pytest.approx(accepted) for accepted in accepted_times) for sampled_at in truth_times)
    assert all(
        packet.interval_start_s is not None and packet.interval_start_s < packet.sampled_at_s
        for packet in bus.packets("interval")
    )
    ####
