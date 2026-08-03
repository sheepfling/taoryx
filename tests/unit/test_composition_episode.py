"""Regression coverage for composition-owned interactive episodes."""

from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.composition_episode import ActionFrame, open_vehicle_composition_episode
from taoryx.vehicle_composition import VehicleCompositionRequest, compile_vehicle_composition, load_vehicle_composition_request

ROOT = Path(__file__).resolve().parents[2]


def _composition(name: str):
    return compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / name))
    ####


def test_x8_composition_episode_reuses_the_language_backed_interactive_kernel(tmp_path: Path) -> None:
    episode = open_vehicle_composition_episode(_composition("x8_racetrack_capability_3dof_compose.yaml"))

    assert {channel.name for channel in episode.action_schema} == {
        "throttle",
        "collective-elevon-deg",
        "differential-elevon-deg",
    }
    assert episode.observe().time_s == pytest.approx(0.0)
    first = episode.step({"throttle": 0.6}, 0.1)

    assert first.time_start_s == pytest.approx(0.0)
    assert first.time_end_s == pytest.approx(0.1)
    assert first.observation.time_s == pytest.approx(0.1)
    checkpoint = episode.save_checkpoint(tmp_path / "x8.checkpoint.json")
    episode.reset()
    assert episode.observe().time_s == pytest.approx(0.0)
    restored = episode.load_checkpoint(checkpoint)
    assert restored.time_s == pytest.approx(0.1)
    ####


def test_hummingbird_pseudo_episode_uses_declared_aggregate_thrust_response_and_checkpoint(tmp_path: Path) -> None:
    composition = _composition("hummingbird_hover_yaw_sensor_episode_pseudo6dof_compose.yaml")
    episode = open_vehicle_composition_episode(composition, seed=7, integration_step_s=0.02)

    initial = episode.observe()
    assert initial.values["position_ned_m"] == [0.0, 0.0, -2.0]
    assert initial.values["physical_motor_allocation"] is False
    response = episode.step({"yaw_rad": 1.0, "thrust_ratio": 2.0}, 0.1)

    assert response.time_end_s == pytest.approx(0.1)
    assert response.applied_action["thrust_ratio"] == pytest.approx(1.0)
    assert response.observation.values["attitude_rad"][2] > 0.0
    checkpoint = episode.save_checkpoint(tmp_path / "hummingbird.checkpoint.json")
    episode.reset(seed=7)
    assert episode.observe().time_s == pytest.approx(0.0)
    restored = episode.load_checkpoint(checkpoint)
    assert restored.time_s == pytest.approx(0.1)
    assert restored.values["physical_motor_allocation"] is False
    ####


def test_x8_episode_accepts_the_resolved_semantic_action_frame() -> None:
    episode = open_vehicle_composition_episode(_composition("x8_racetrack_capability_3dof_compose.yaml"))
    contract = episode.interface_contract
    frame = ActionFrame(
        contract.id,
        contract.fingerprint,
        "native_control_bridge",
        {"propulsion.command.fraction": 0.6},
        0.1,
    )

    result = episode.step_frame(frame)

    assert result.applied_action["throttle"] == pytest.approx(0.6)
    assert result.applied_semantic_action is not None
    assert result.applied_semantic_action["propulsion.command.fraction"] == pytest.approx(0.6)
    assert result.action_frame == frame
    assert result.observation_frame is not None
    assert result.status_frame is not None
    assert result.status_frame.values["execution.time"] == pytest.approx(0.1)
    assert result.status_frame.values["position.altitude"] == pytest.approx(178.0, abs=0.1)
    ####


def test_x8_declared_sensor_uses_committed_cadence_and_restores_from_checkpoint(tmp_path: Path) -> None:
    episode = open_vehicle_composition_episode(_composition("x8_racetrack_sensor_episode_3dof_compose.yaml"))

    initial = episode.observe_frame("declared_sensor")
    assert initial.source_time_s is None
    assert not any(initial.valid.values())
    episode.step({"throttle": 0.6}, 0.01)
    before_release = episode.observe_frame("declared_sensor")
    assert before_release.source_time_s is None
    episode.step({"throttle": 0.6}, 0.05)
    released = episode.observe_frame("declared_sensor")
    assert released.time_s == pytest.approx(0.06)
    assert released.source_time_s == pytest.approx(0.0)
    assert all(released.valid.values())
    assert released.values["execution.time"] == pytest.approx(0.0)

    checkpoint = episode.save_checkpoint(tmp_path / "x8-sensor.checkpoint.json")
    episode.reset()
    assert episode.observe_frame("declared_sensor").source_time_s is None
    episode.load_checkpoint(checkpoint)
    restored = episode.observe_frame("declared_sensor")
    assert restored.source_time_s == pytest.approx(0.0)
    assert restored.values == released.values
    ####


def test_x8_declared_sensor_error_is_seeded_and_never_changes_truth_status(tmp_path: Path) -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_sensor_episode_3dof_compose.yaml")
    payload = request.model_dump(mode="json", by_alias=True)
    payload["observation"]["declared_sensor"]["channel_errors"] = {
        "execution.time": {"bias": 0.25, "gaussian_stddev": 0.0, "quantization_step": 0.05},
    }
    composition = compile_vehicle_composition(VehicleCompositionRequest.model_validate(payload))
    episode = open_vehicle_composition_episode(composition, seed=31)

    episode.step({"throttle": 0.6}, 0.06)
    measured = episode.observe_frame("declared_sensor")
    truth = episode.status_frame()
    assert measured.source_time_s == pytest.approx(0.0)
    assert measured.values["execution.time"] == pytest.approx(0.25)
    assert truth.values["execution.time"] == pytest.approx(0.06)

    checkpoint = episode.save_checkpoint(tmp_path / "x8-noisy-sensor.checkpoint.json")
    episode.reset(seed=31)
    episode.load_checkpoint(checkpoint)
    assert episode.observe_frame("declared_sensor").values == measured.values
    ####


def test_language_backed_reset_rebuilds_the_declared_sensor_at_the_requested_seed() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_sensor_episode_3dof_compose.yaml")
    payload = request.model_dump(mode="json", by_alias=True)
    payload["observation"]["declared_sensor"]["channel_errors"] = {
        "execution.time": {"bias": 0.0, "gaussian_stddev": 0.1},
    }
    composition = compile_vehicle_composition(VehicleCompositionRequest.model_validate(payload))
    episode = open_vehicle_composition_episode(composition, seed=3)
    episode.step({"throttle": 0.6}, 0.06)

    episode.reset(seed=11)
    episode.step({"throttle": 0.6}, 0.06)
    reset_observation = episode.observe_frame("declared_sensor")

    fresh = open_vehicle_composition_episode(composition, seed=11)
    fresh.step({"throttle": 0.6}, 0.06)
    fresh_observation = fresh.observe_frame("declared_sensor")

    assert reset_observation.as_dict() == fresh_observation.as_dict()
    assert episode.status_frame().values["execution.time"] == pytest.approx(0.06)
    ####


def test_hummingbird_episode_maps_semantic_controls_and_status_without_motor_promotion() -> None:
    episode = open_vehicle_composition_episode(_composition("hummingbird_hover_yaw_sensor_episode_pseudo6dof_compose.yaml"))
    contract = episode.interface_contract
    frame = ActionFrame(
        contract.id,
        contract.fingerprint,
        "body_motion_response",
        {
            "attitude.yaw.command": 0.5,
            "propulsion.command.fraction": 2.0,
        },
        0.1,
    )

    result = episode.step_frame(frame)

    assert result.applied_action["yaw_rad"] == pytest.approx(0.5)
    assert result.applied_action["thrust_ratio"] == pytest.approx(1.0)
    assert result.applied_semantic_action is not None
    assert result.applied_semantic_action["propulsion.command.fraction"] == pytest.approx(1.0)
    assert result.status_frame is not None
    assert result.status_frame.values["propulsion.output.thrust.aggregate"] > 0.0
    assert result.status_frame.values["control.physical_motor_allocation"] is False
    assert result.observation_frame is not None
    assert result.observation_frame.valid["resources.battery.fraction_remaining"] is True
    assert result.observation_frame.observation_profile_id == "declared_sensor"
    assert result.observation_frame.source_time_s == pytest.approx(0.08)
    ####


def test_x15_local_direct_wrench_episode_exposes_bounded_bridge_not_effectors(tmp_path: Path) -> None:
    """The direct-wrench screen must be stepwise without an actuator claim."""

    episode = open_vehicle_composition_episode(_composition("x15_local_direct_wrench_screen_compose.yaml"))
    contract = episode.interface_contract

    assert contract.authority_profile("direct_wrench").availability == "available"
    assert episode.claim_boundary.startswith("This episode applies an explicit bounded direct wrench")
    initial = episode.status_frame()
    assert initial.values["control.realization"] == "direct_wrench_screen"
    assert initial.values["control.wrench.saturated"] is False

    frame = ActionFrame(
        contract.id,
        contract.fingerprint,
        "direct_wrench",
        {
            "wrench.force.command": [1.0e9, 0.0, 0.0],
            "wrench.moment.command": [0.0, 0.0, 0.0],
        },
        0.004,
    )
    result = episode.step_frame(frame)

    assert result.applied_action["force_body_n"][0] == pytest.approx(4.0e5)
    assert result.applied_semantic_action == {
        "wrench.force.command": [4.0e5, 0.0, 0.0],
        "wrench.moment.command": [0.0, 0.0, 0.0],
    }
    assert result.status_frame is not None
    assert result.status_frame.values["control.wrench.saturated"] is True
    assert result.status_frame.values["control.wrench.achieved.force"][0] == pytest.approx(4.0e5)
    assert result.status_frame.values["control.physical_effector_allocation"] is False

    checkpoint = episode.save_checkpoint(tmp_path / "x15-direct-wrench.checkpoint.json")
    expected = episode.observe().as_dict()
    episode.reset()
    episode.load_checkpoint(checkpoint)
    assert episode.observe().as_dict() == expected
    ####


def test_episode_creation_fails_closed_without_a_registered_adapter() -> None:
    composition = _composition("a320_racetrack_capability_3dof_compose.yaml")

    with pytest.raises(ValueError, match="no composition episode adapter"):
        open_vehicle_composition_episode(composition)
    ####
