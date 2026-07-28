from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from pydantic import ValidationError

from taoryx.rigid_body import RIGID_BODY_STATE_NAMES
from taoryx.runtime import RuntimeProblem, RuntimeState, RuntimeVehicle
from taoryx.runtime.engine import compute_trajectories
from taoryx.runtime.sensor_contracts import (
    Pseudo6DofTruthConfig,
    RotorcraftAttitudeConfig,
    VelocityAlignedAttitudeConfig,
    parse_provider_config,
    parse_truth_config,
)
from taoryx.runtime.sensor_scenario import SensorScenarioSpec, attach_sensor_scenario
from taoryx.runtime.truth import (
    AttitudePolicyFactory,
    CompositeTruthProvider,
    Pseudo6DofPolicy,
    Pseudo6DofTruthProvider,
    RotationTruth,
    RotorcraftAttitudePolicy,
    TranslationTruthProvider,
)
from taoryx.sensors import TruthPoint

ROOT = Path(__file__).resolve().parents[2]


def _truth(state: RuntimeState) -> TruthPoint:
    return TruthPoint(
        time_s=state.time,
        position_eci_m=np.zeros(3),
        velocity_eci_mps=np.zeros(3),
        velocity_without_gravity_eci_mps=np.zeros(3),
        orientation_eci_from_body=np.eye(3),
        gravity_eci_mps2=np.array([0.0, 0.0, -9.81]),
        angular_rate_body_radps=np.zeros(3),
        acceleration_eci_mps2=np.zeros(3),
    )


def test_sensor_sidecar_loads_provider_and_earth_contract() -> None:
    spec = SensorScenarioSpec.from_file(ROOT / "examples/sensors/hummingbird_sensorized_hover_v1.yaml")

    assert spec.scenario_id == "hummingbird_sensorized_hover_v1"
    assert spec.provider == "imu-error-model"
    assert spec.earth_rate_mode == "source"
    metadata = spec.to_metadata()
    assert metadata["profile_path"]
    assert metadata["provider_config"] == {"kind": "imu-error-model"}
    assert metadata["truth_config"] == {"mode": "vehicle"}


def test_translation_and_pseudo_sidecars_declare_distinct_truth_contracts() -> None:
    translation = SensorScenarioSpec.from_file(ROOT / "examples/sensors/hummingbird_sensorized_translation_v1.yaml")
    pseudo = SensorScenarioSpec.from_file(ROOT / "examples/sensors/hummingbird_sensorized_pseudo6dof_v1.yaml")

    assert translation.truth_mode == "translation-only"
    assert translation.provider == "translation-acceleration"
    assert translation.estimator_modes == ("translation-dead-reckoning",)
    assert pseudo.truth_mode == "pseudo-6dof"
    assert pseudo.provider == "ideal"
    assert pseudo.to_metadata()["truth_config"]["orientation"]["kind"] == "velocity-aligned"


def test_sensor_contracts_discriminate_provider_and_attitude_variants() -> None:
    velocity = parse_truth_config({"mode": "pseudo-6dof", "orientation": {"alignment": "velocity"}})
    rotorcraft = parse_truth_config(
        {
            "mode": "pseudo-6dof",
            "orientation": {"kind": "rotorcraft", "vehicle_type": "tilt-rotor", "forward_source": "body-axis"},
        }
    )

    assert isinstance(velocity, Pseudo6DofTruthConfig)
    assert isinstance(velocity.orientation, VelocityAlignedAttitudeConfig)
    assert isinstance(rotorcraft, Pseudo6DofTruthConfig)
    assert isinstance(rotorcraft.orientation, RotorcraftAttitudeConfig)
    assert parse_provider_config("ideal").kind == "ideal"
    with pytest.raises(ValidationError):
        parse_provider_config("not-a-provider")
    with pytest.raises(ValidationError):
        parse_truth_config({"mode": "pseudo-6dof", "orientation": {"kind": "rotorcraft", "vehicle_type": "helicopter"}})


def test_rotorcraft_body_axis_policy_uses_quaternion_and_fails_closed_without_it() -> None:
    policy = AttitudePolicyFactory.create(
        {"kind": "rotorcraft", "vehicle_type": "quadcopter", "forward_source": "body-axis"}
    )
    assert isinstance(policy, RotorcraftAttitudePolicy)
    base = TranslationTruthProvider(3.986004418e14, 0.0)
    provider = Pseudo6DofTruthProvider(base, policy)
    state = RuntimeState(
        0.0,
        (0.0, 0.0, 6_378_137.0, 0.0, 100.0, 0.0, 1.0, 0.0, 0.0, 0.0),
        frame="ecic",
        value_names=("x", "y", "z", "xdt", "ydt", "zdt", "qw", "qx", "qy", "qz"),
    )
    truth = provider(state)

    assert truth.orientation_eci_from_body is not None
    assert truth.orientation_eci_from_body[:, 0] == pytest.approx([1.0, 0.0, 0.0])
    assert provider.contract["orientation"]["policy"] == "rotorcraft"

    missing_quaternion = RuntimeState(
        0.0,
        (0.0, 0.0, 6_378_137.0, 0.0, 100.0, 0.0),
        frame="ecic",
        value_names=("x", "y", "z", "xdt", "ydt", "zdt"),
    )
    with pytest.raises(ValueError, match="requires qw"):
        provider(missing_quaternion)


def test_pseudo6dof_policy_is_continuous_through_vertical_crossing() -> None:
    base = TranslationTruthProvider(3.986004418e14, 0.0)
    provider = Pseudo6DofTruthProvider(base, Pseudo6DofPolicy(speed_threshold_mps=1.0))
    states = (
        RuntimeState(0.0, (0.0, 0.0, 6_378_137.0, 0.0, 0.0, 0.0), frame="ecic", value_names=("x", "y", "z", "xdt", "ydt", "zdt")),
        RuntimeState(0.1, (0.0, 0.0, 6_378_137.0, 0.0, 0.0, 2.0), frame="ecic", value_names=("x", "y", "z", "xdt", "ydt", "zdt")),
        RuntimeState(0.2, (0.0, 0.0, 6_378_137.0, 0.0, 0.0, -2.0), frame="ecic", value_names=("x", "y", "z", "xdt", "ydt", "zdt")),
    )
    truths = [provider(state) for state in states]

    for truth in truths:
        assert truth.orientation_eci_from_body is not None
        assert np.allclose(truth.orientation_eci_from_body.T @ truth.orientation_eci_from_body, np.eye(3), atol=1.0e-8)
        assert np.isclose(np.linalg.det(truth.orientation_eci_from_body), 1.0, atol=1.0e-8)
        assert truth.angular_rate_body_radps is not None
        assert np.all(np.isfinite(truth.angular_rate_body_radps))


def test_hybrid_truth_combines_substituted_translation_with_external_rotation() -> None:
    base = TranslationTruthProvider(3.986004418e14, 0.0)
    rate = 0.25

    def table_rotation(state: RuntimeState) -> RotationTruth:
        angle = rate * state.time
        orientation = np.array(
            [
                [np.cos(angle), -np.sin(angle), 0.0],
                [np.sin(angle), np.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        return RotationTruth(state.time, orientation, np.array([0.0, 0.0, rate]))

    provider = CompositeTruthProvider(base, table_rotation)
    state = RuntimeState(2.0, (10.0, 20.0, 6_378_137.0, 1.0, 2.0, 3.0), frame="ecic", value_names=("x", "y", "z", "xdt", "ydt", "zdt"))
    truth = provider(state)

    assert truth.position_eci_m == pytest.approx([10.0, 20.0, 6_378_137.0])
    assert truth.orientation_eci_from_body is not None
    assert truth.angular_rate_body_radps == pytest.approx([0.0, 0.0, rate])
    assert provider.contract["mode"] == "hybrid-6dof"
    assert provider.contract["translation"]["substitution_allowed"] is True
    assert provider.contract["channel_sources"]["rotation"] == "external-rotational-provider"


def test_hybrid_sensor_scenario_attaches_external_rotation_source() -> None:
    vehicle = RuntimeVehicle("vehicle", RuntimeState(0.0, (0.0,)), lambda _: (1.0,), step_size=0.1, truth_provider=_truth)
    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=0.2)

    def table_rotation(state: RuntimeState) -> RotationTruth:
        return RotationTruth(state.time, np.eye(3), np.zeros(3))

    spec = SensorScenarioSpec(
        "imu",
        vehicle.name,
        provider="ideal",
        truth_mode="hybrid-6dof",
        cadence_s=0.1,
        estimator_modes=("dead-reckoning",),
    )
    runtime = attach_sensor_scenario(problem, spec, rotational_truth_provider=table_rotation)
    result = compute_trajectories(problem, max_steps=3)

    assert result.completed
    assert runtime.truth_contract["mode"] == "hybrid-6dof"
    assert runtime.truth_contract["channel_sources"]["rotation"] == "external-rotational-provider"
    assert runtime.histories["dead_reckoning"]


def test_rotation_only_sensor_scenario_uses_attitude_dead_reckoning() -> None:
    vehicle = RuntimeVehicle("vehicle", RuntimeState(0.0, (0.0,)), lambda _: (1.0,), step_size=0.1, truth_provider=_truth)
    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=0.2)
    spec = SensorScenarioSpec(
        "gyro",
        vehicle.name,
        provider="ideal-gyroscope",
        truth_mode="rotation-only",
        cadence_s=0.1,
        estimator_modes=("attitude-dead-reckoning",),
    )
    runtime = attach_sensor_scenario(problem, spec)
    result = compute_trajectories(problem, max_steps=3)

    assert result.completed
    assert runtime.histories["attitude_dead_reckoning"]
    packet = runtime.bus.packets("gyro")[1]
    assert packet.payload is not None
    assert runtime.bus.bindings[0].provenance["translation_usage"] == "not-consumed"


def test_ideal_sensor_drop_and_timeout_are_artifact_visible() -> None:
    vehicle = RuntimeVehicle("vehicle", RuntimeState(0.0, (0.0,)), lambda _: (1.0,), step_size=0.1, truth_provider=_truth)
    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=0.3)
    spec = SensorScenarioSpec(
        "imu",
        vehicle.name,
        scenario_id="unit-ideal-drop",
        provider="ideal",
        cadence_s=0.1,
        drop_every_n=2,
        estimator_modes=("dead-reckoning",),
    )
    runtime = attach_sensor_scenario(problem, spec)
    result = compute_trajectories(problem, max_steps=2)
    runtime.finalize(completed=result.completed, stop_reason=result.stop_reason, max_steps=2)
    execution = runtime.artifact()

    assert not result.completed
    summary = execution["measurement_summary"]
    assert isinstance(summary, dict)
    assert summary["timeout"] is True
    assert summary["dropped"] == 1
    assert summary["queued"] >= 0
    assert execution["rerun"]["recommended_max_steps"] == 4
    assert execution["reproducibility"]["measurement_sha256"]


def test_mekf_feedback_is_delivered_only_and_does_not_mutate_plant_truth() -> None:
    initial_values = (
        6_378_137.0,
        0.0,
        0.0,
        0.0,
        100.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        10.0,
        5.0,
        0.0,
        0.0,
    )

    def rigid_truth(state: RuntimeState) -> TruthPoint:
        return TruthPoint(
            state.time,
            np.asarray(state.values[0:3]),
            np.asarray(state.values[3:6]),
            np.asarray(state.values[3:6]),
            np.eye(3),
            np.zeros(3),
            np.zeros(3),
        )

    vehicle = RuntimeVehicle(
        "vehicle",
        RuntimeState(0.0, initial_values, frame="ecic", value_names=RIGID_BODY_STATE_NAMES),
        lambda _: (0.0,) * len(RIGID_BODY_STATE_NAMES),
        step_size=0.1,
        truth_provider=rigid_truth,
    )
    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=0.3)
    spec = SensorScenarioSpec(
        "imu",
        vehicle.name,
        provider="ideal",
        cadence_s=0.1,
        delivery_s=0.02,
        estimator_modes=("mekf",),
        feedback={"source": "mekf", "availability": "delivered", "stale_policy": "hold"},
    )

    runtime = attach_sensor_scenario(problem, spec)
    result = compute_trajectories(problem, max_steps=10)

    assert result.completed
    assert runtime.feedback_selected_count >= 2
    assert runtime.feedback_last_available_s == pytest.approx(0.22)
    assert vehicle.controller_state is not None
    assert tuple(vehicle.history[-1].values) == pytest.approx(initial_values)
    assert runtime.artifact()["feedback"]["startup_fallback"] == "plant-truth-until-first-delivered-estimate"


def test_mekf_feedback_fail_policy_reports_missing_delivered_estimate() -> None:
    vehicle = RuntimeVehicle(
        "vehicle",
        RuntimeState(0.0, (6_378_137.0, 0.0, 0.0, 0.0, 100.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 5.0, 0.0, 0.0), frame="ecic", value_names=RIGID_BODY_STATE_NAMES),
        lambda _: (0.0,) * len(RIGID_BODY_STATE_NAMES),
        step_size=0.1,
        truth_provider=lambda state: _truth(state),
    )
    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=0.2)
    spec = SensorScenarioSpec(
        "imu",
        vehicle.name,
        provider="ideal",
        cadence_s=0.1,
        drop_every_n=1,
        estimator_modes=("mekf",),
        feedback={"source": "mekf", "stale_policy": "fail", "max_age_s": 0.01},
    )

    attach_sensor_scenario(problem, spec)
    with pytest.raises(RuntimeError, match="is stale"):
        compute_trajectories(problem, max_steps=10)
