"""Regression coverage for composition-owned interactive episodes."""

from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.composition_episode import (
    ActionFrame,
    EpisodeChannel,
    HummingbirdPseudoCompositionEpisode,
    ReducedFixedWingCompositionEpisode,
    open_vehicle_composition_episode,
    validate_vehicle_composition_episode_contract,
)
from taoryx.value_space import finite_set, validate_value_space_value
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
        "guidance-override-enabled",
        "guidance-speed-mps",
        "guidance-flight-path-angle-deg",
        "guidance-heading-deg",
    }
    assert all(channel.value_space is not None for channel in episode.action_schema)
    action_spaces = {channel.name: channel.value_space for channel in episode.action_schema}
    assert action_spaces["throttle"] is not None
    assert action_spaces["throttle"].topology == "unit_interval"
    assert action_spaces["collective-elevon-deg"] is not None
    assert action_spaces["collective-elevon-deg"].topology == "bounded_interval"
    assert all(channel.value_space is not None for channel in episode.observation_schema)
    observations = {channel.name: channel.value_space for channel in episode.observation_schema}
    assert observations["1.psi"] is not None
    assert observations["1.psi"].topology == "periodic_circle"
    assert observations["1.psi"].period == pytest.approx(360.0)
    assert observations["1.vel"] is not None
    assert observations["1.vel"].topology == "positive_half_line"
    assert observations["1.throttle"] is not None
    assert observations["1.throttle"].topology == "unit_interval"
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


@pytest.mark.parametrize(
    "composition_name",
    (
        "x8_racetrack_capability_3dof_compose.yaml",
        "x8_racetrack_capability_compose.yaml",
        "b747_racetrack_capability_pseudo6dof_compose.yaml",
        "hummingbird_hover_yaw_sensor_episode_pseudo6dof_compose.yaml",
        "a320_racetrack_capability_pseudo6dof_compose.yaml",
        "f16_racetrack_capability_pseudo6dof_compose.yaml",
        "x15_local_direct_wrench_screen_compose.yaml",
        "hl20_local_direct_wrench_screen_compose.yaml",
    ),
)
def test_runnable_episode_exposes_only_contract_bound_native_channels(composition_name: str) -> None:
    """Legacy native schemas must be a complete projection of the semantic API."""

    episode = open_vehicle_composition_episode(_composition(composition_name))
    report = validate_vehicle_composition_episode_contract(episode)

    assert report["status"] == "pass", report["findings"]
    graph = report["mission_graph"]
    assert graph["status"] == "bound"
    assert graph["execution_observation_status"] == "not_emitted_by_episode"
    assert graph["instance_ids"]
    assert graph["execution_contract"]["status"] in {
        "template_success_sequence_only",
        "family_extension_declared",
    }
    assert report["semantic_action_channel_count"] == report["native_action_channel_count"]
    assert report["available_observation_profiles"]
    episode.close()
    ####


def test_episode_contract_gate_rejects_an_unbound_native_action(monkeypatch: pytest.MonkeyPatch) -> None:
    """An adapter cannot expose a legacy control that bypasses its semantic authority."""

    original = HummingbirdPseudoCompositionEpisode.action_schema

    def action_schema_with_bypass(self: HummingbirdPseudoCompositionEpisode) -> tuple[EpisodeChannel, ...]:
        return (*original.__get__(self, HummingbirdPseudoCompositionEpisode), EpisodeChannel("hidden_native_bypass", None, description="invalid test bypass"))
        ####

    monkeypatch.setattr(
        HummingbirdPseudoCompositionEpisode,
        "action_schema",
        property(action_schema_with_bypass),
    )
    episode = open_vehicle_composition_episode(_composition("hummingbird_hover_yaw_sensor_episode_pseudo6dof_compose.yaml"))

    report = validate_vehicle_composition_episode_contract(episode)

    assert report["status"] == "fail"
    assert "hidden_native_bypass" in str(report["findings"])
    episode.close()
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
    action_spaces = {channel.name: channel.value_space for channel in episode.action_schema}
    assert action_spaces["yaw_rad"] is not None
    assert action_spaces["yaw_rad"].topology == "periodic_circle"
    assert action_spaces["thrust_ratio"] is not None
    assert action_spaces["thrust_ratio"].topology == "unit_interval"
    observation_spaces = {channel.name: channel.value_space for channel in episode.observation_schema}
    assert observation_spaces["aggregate_thrust_n"] is not None
    assert observation_spaces["aggregate_thrust_n"].topology == "positive_half_line"
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

    with pytest.raises(ValueError, match="exceeds its declared upper bound"):
        episode.step_frame(frame)

    result = episode.step_frame(
        ActionFrame(
            contract.id,
            contract.fingerprint,
            "body_motion_response",
            {
                "attitude.yaw.command": 0.5,
                "propulsion.command.fraction": 0.8,
            },
            0.1,
        )
    )

    assert result.applied_action["yaw_rad"] == pytest.approx(0.5)
    assert result.applied_action["thrust_ratio"] == pytest.approx(0.8)
    assert result.applied_semantic_action is not None
    assert result.applied_semantic_action["propulsion.command.fraction"] == pytest.approx(0.8)
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
    action_spaces = {channel.name: channel.value_space for channel in episode.action_schema}
    assert action_spaces["force_body_n"] is not None
    assert action_spaces["force_body_n"].topology == "euclidean"
    observation_spaces = {channel.name: channel.value_space for channel in episode.observation_schema}
    assert observation_spaces["wrench_saturated"] is not None
    assert observation_spaces["wrench_saturated"].topology == "boolean"

    assert contract.authority_profile("direct_wrench").availability == "available"
    assert episode.claim_boundary.startswith("This episode applies an explicit bounded direct wrench")
    initial = episode.status_frame()
    assert initial.values["control.realization"] == "direct_wrench_screen"
    assert initial.values["control.wrench.saturated"] is False
    assert initial.values["resources.mass.total"] == pytest.approx(episode.observe().values["mass_kg"])
    assert initial.values["resources.mass.total"] > 0.0

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


def test_hl20_local_direct_wrench_episode_reuses_the_bridge_without_promoting_the_glide_mission() -> None:
    episode = open_vehicle_composition_episode(_composition("hl20_local_direct_wrench_screen_compose.yaml"))
    contract = episode.interface_contract

    assert contract.authority_profile("direct_wrench").availability == "available"
    initial = episode.status_frame()
    assert initial.values["resources.mass.total"] == pytest.approx(episode.observe().values["mass_kg"])
    assert initial.values["resources.mass.total"] > 0.0
    result = episode.step_frame(
        ActionFrame(
            contract.id,
            contract.fingerprint,
            "direct_wrench",
            {
                "wrench.force.command": [0.0, 0.0, 0.0],
                "wrench.moment.command": [0.0, 0.0, 0.0],
            },
            0.004,
        )
    )
    assert result.status_frame is not None
    assert result.status_frame.values["control.realization"] == "direct_wrench_screen"
    assert result.status_frame.values["control.physical_effector_allocation"] is False
    assert "not physical effector allocation" in episode.claim_boundary
    episode.close()
    ####


def test_a320_reduced_episode_steps_the_declared_kinematic_guidance_state(tmp_path: Path) -> None:
    composition = _composition("a320_racetrack_capability_pseudo6dof_compose.yaml")
    episode = open_vehicle_composition_episode(composition)
    contract = episode.interface_contract

    assert {channel.name for channel in episode.action_schema} == {
        "speed_m_s",
        "flight_path_angle_deg",
        "heading_deg",
        "bank_angle_deg",
    }
    frame = ActionFrame(
        contract.id,
        contract.fingerprint,
        "kinematic_guidance",
        {
            "guidance.speed.command": 230.0,
            "guidance.flight_path_angle.command": 2.0,
            "guidance.heading.command": 0.0,
            "guidance.bank.command": 8.0,
        },
        1.0,
    )
    result = episode.step_frame(frame)

    assert result.time_end_s == pytest.approx(1.0)
    assert result.applied_semantic_action == frame.values
    assert result.status_frame is not None
    assert result.status_frame.values["position.east"] > 0.0
    assert result.status_frame.values["attitude.euler"][0] != 0.0
    assert result.status_frame.values["control.realization"] == "response_law"

    checkpoint = episode.save_checkpoint(tmp_path / "a320-reduced.checkpoint.json")
    expected = episode.observe().as_dict()
    episode.reset()
    episode.load_checkpoint(checkpoint)
    assert episode.observe().as_dict() == expected
    ####


def test_f16_reduced_episode_uses_the_shared_kinematic_guidance_contract(tmp_path: Path) -> None:
    composition = _composition("f16_racetrack_capability_pseudo6dof_compose.yaml")
    episode = open_vehicle_composition_episode(composition)
    contract = episode.interface_contract

    assert isinstance(episode, ReducedFixedWingCompositionEpisode)

    frame = ActionFrame(
        contract.id,
        contract.fingerprint,
        "kinematic_guidance",
        {
            "guidance.speed.command": 155.0,
            "guidance.flight_path_angle.command": 2.0,
            "guidance.heading.command": 0.0,
            "guidance.bank.command": 8.0,
        },
        1.0,
    )
    result = episode.step_frame(frame)

    assert result.time_end_s == pytest.approx(1.0)
    assert result.applied_semantic_action == frame.values
    assert result.status_frame is not None
    assert result.status_frame.values["position.east"] > 0.0
    assert result.status_frame.values["attitude.euler"][0] != 0.0
    assert result.status_frame.values["control.realization"] == "response_law"

    checkpoint = episode.save_checkpoint(tmp_path / "f16-reduced.checkpoint.json")
    expected = episode.observe().as_dict()
    episode.reset()
    episode.load_checkpoint(checkpoint)
    assert episode.observe().as_dict() == expected
    ####


@pytest.mark.parametrize(
    "composition_name",
    [
        "a320_racetrack_capability_pseudo6dof_compose.yaml",
        "f16_racetrack_capability_pseudo6dof_compose.yaml",
    ],
)
def test_reduced_fixed_wing_episode_conformance_contract(composition_name: str) -> None:
    """New reduced fixed-wing members retain one public episode vocabulary."""

    episode = open_vehicle_composition_episode(_composition(composition_name))
    contract = episode.interface_contract

    assert isinstance(episode, ReducedFixedWingCompositionEpisode)
    assert tuple(channel.name for channel in episode.action_schema) == (
        "speed_m_s",
        "flight_path_angle_deg",
        "heading_deg",
        "bank_angle_deg",
    )
    frame = ActionFrame(
        contract.id,
        contract.fingerprint,
        "kinematic_guidance",
        {
            "guidance.speed.command": 155.0,
            "guidance.flight_path_angle.command": 0.0,
            "guidance.heading.command": 0.0,
            "guidance.bank.command": 5.0,
        },
        0.2,
    )
    result = episode.step_frame(frame)

    assert result.applied_semantic_action == frame.values
    assert result.status_frame is not None
    assert result.status_frame.values["control.realization"] == "response_law"
    assert result.status_frame.values["execution.time"] == pytest.approx(0.2)
    ####


def test_native_source_state_codes_are_discrete_without_becoming_boolean_or_continuous() -> None:
    specification = finite_set(representation="scalar source state code")

    validate_value_space_value(specification, 1.0, context="source state")
    with pytest.raises(ValueError, match="finite numeric"):
        validate_value_space_value(specification, "1", context="source state")
    ####
