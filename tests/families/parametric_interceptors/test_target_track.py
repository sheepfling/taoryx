"""Vertical witnesses for the selectable moving-target mission surface."""

from __future__ import annotations

from pathlib import Path

import pytest
from taoryx_parametric_interceptors import (
    TARGET_TRACK_MISSION_TEMPLATE_ID,
    InterceptorSensorSuite,
    ParametricInterceptorMissionCompositionProvider,
    interceptor,
)

from taoryx.runtime.sensor_contracts import SensorProviderConfig
from taoryx.trajectory.configuration_contract import ConfigurationContractError
from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunRequest,
    MissionCompositionTrajectoryResponse,
)
from taoryx.trajectory.session_contract import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionManager,
    MissionCompositionSessionStepRequest,
)

_TARGET_ACTION_IDS = (
    "navigation.target.position.north.command",
    "navigation.target.position.east.command",
    "navigation.target.position.altitude.command",
    "navigation.target.velocity.north.command",
    "navigation.target.velocity.east.command",
    "navigation.target.velocity.vertical.command",
    "navigation.target.capture_radius.command",
)


def _provider() -> ParametricInterceptorMissionCompositionProvider:
    return ParametricInterceptorMissionCompositionProvider.from_profiles(
        interceptor(
            "moving-target-sam",
            launch_mass_kg=420.0,
            propellant_fraction=0.42,
            length_m=5.2,
            body_diameter_m=0.36,
            guidance_family="active_radar",
            guidance_archetype="proportional_navigation",
            navigation_constant=4.0,
        )
    )
    ####


def _provider_with_target_sensor(
    suite_id: str,
    target_config: dict[str, object],
) -> ParametricInterceptorMissionCompositionProvider:
    suite = InterceptorSensorSuite(
        id=suite_id,
        version="1.0.0",
        target_track_provider=SensorProviderConfig(
            kind="relative-state-track",
            config={"target_id": "guidance-target", **target_config},
        ),
        provenance="focused target-track sensor witness",
        claim_boundary="Synthetic relative-state measurement witness only.",
    )
    return ParametricInterceptorMissionCompositionProvider.from_profiles(
        interceptor(
            "moving-target-sam",
            guidance_archetype="proportional_navigation",
            navigation_constant=4.0,
        ),
        sensor_suites={suite.id: suite},
        default_sensor_suite_id=suite.id,
    )
    ####


def _configuration(
    provider: ParametricInterceptorMissionCompositionProvider,
    fidelity: str,
    *,
    startup_authority_profile_id: str | None = None,
):
    return provider.configuration(
        "moving-target-sam",
        configuration_id=f"moving-target-{fidelity}",
        fidelity=fidelity,
        mission_template_id=TARGET_TRACK_MISSION_TEMPLATE_ID,
        startup_authority_profile_id=startup_authority_profile_id,
        launch_altitude_m=100.0,
        launch_speed_mps=200.0,
        launch_heading_deg=0.0,
        launch_flight_path_deg=0.0,
        navigation_target_position_north_command=1_000.0,
        navigation_target_position_east_command=500.0,
        navigation_target_position_altitude_command=100.0,
        navigation_target_velocity_north_command=25.0,
        navigation_target_velocity_east_command=50.0,
        navigation_target_velocity_vertical_command=0.0,
        navigation_target_capture_radius_command=10.0,
        runtime_duration_s=0.3,
        runtime_time_step_s=0.1,
    )
    ####


def test_target_track_mission_advertises_exact_controls_feedback_and_operations() -> None:
    provider = _provider()
    model = provider.model("moving-target-sam")
    realization = model.realizations[0]
    properties = {item.id: item for item in model.presentation.properties}

    assert {item.id for item in model.mission_templates} == {
        "fixed_waypoint_intercept",
        TARGET_TRACK_MISSION_TEMPLATE_ID,
        "direct_lateral_acceleration_control",
    }
    mission = next(item for item in model.mission_templates if item.id == TARGET_TRACK_MISSION_TEMPLATE_ID)
    assert mission.segment_sequence == ("boost", "target_track_guidance")
    assert all("target_track" in item.execution_mode for item in mission.operations if item.operation != "validate")
    authorities = {item.id: item for item in realization.controls.authorities}
    assert authorities["target_track_guidance"].channel_ids == _TARGET_ACTION_IDS
    assert authorities["live_target_track_guidance"].channel_ids == _TARGET_ACTION_IDS
    assert authorities["live_target_track_guidance"].scheme_layer == "mission"
    controls = {item.id: item for item in realization.controls.channels}
    assert controls["navigation.target.position.north.command"].canonical_unit == "m"
    assert controls["navigation.target.velocity.north.command"].canonical_unit == "m/s"
    assert controls["navigation.target.capture_radius.command"].interval.minimum.value == 0.1
    assert controls["navigation.target.velocity.east.command"].provider_binding["feedback_channel_id"] == "guidance.target.velocity.east.accepted"
    outputs = {item.id: item for item in model.output_schema.telemetry_channels}
    assert outputs["guidance.target.position.north"].canonical_unit == "m"
    assert outputs["guidance.objective.range"].canonical_unit == "m"
    assert outputs["guidance.objective.captured"].data_type == "boolean"
    assert outputs["guidance.relative.velocity.north"].canonical_unit == "m/s"
    assert outputs["guidance.relative.time_to_closest_approach"].canonical_unit == "s"
    assert outputs["guidance.relative.predicted_miss_distance"].canonical_unit == "m"
    assert "not a terminal-hit prediction" in outputs["guidance.relative.predicted_miss_distance"].description
    assert outputs["sensor.target_track.range"].canonical_unit == "m"
    assert outputs["sensor.target_track.range"].frame == "sensor"
    assert outputs["sensor.target_track.line_of_sight_rate.sensor.z"].canonical_unit == "rad/s"
    assert outputs["sensor.target_track.valid"].data_type == "boolean"
    assert outputs["sensor.target_track.schema_id"].data_type == "string"
    assert "seeker" in mission.claim_boundary
    assert "target dynamics" in mission.claim_boundary
    assert properties["live_target_track_session_status"].value == "available_point_mass_and_pseudo6"
    assert properties["default_target_track_sensor_provider"].value == "relative-state-track"
    assert properties["capture_event_detection_contract"].value == "taoryx.parametric-interceptors.semi-implicit-step-events/v1"
    ####


@pytest.mark.parametrize("fidelity", ("point_mass_3dof", "attitude_response_pseudo_6dof"))
def test_target_track_batch_propagates_shared_relative_state(fidelity: str) -> None:
    provider = _provider()
    prepared = provider.validate_configuration(_configuration(provider, fidelity))
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id=f"moving-target-batch-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )

    assert isinstance(response, MissionCompositionTrajectoryResponse)
    samples = response.result.objects[0].samples
    initial = samples[0].values
    terminal = samples[-1].values
    assert initial["guidance.objective.kind"] == "constant_velocity_target"
    assert initial["sensor.target_track.applicable"] is True
    assert initial["sensor.target_track.valid"] is True
    assert initial["sensor.target_track.delivery_fresh"] is True
    assert initial["sensor.target_track.sequence"] == 0
    assert initial["sensor.target_track.schema_id"] == "taoryx.tracking.relative-state/v1"
    assert initial["sensor.target_track.target_id"] == "guidance-target"
    assert initial["sensor.target_track.range"] == pytest.approx((1_000.0**2 + 500.0**2) ** 0.5)
    assert initial["guidance.target.position.north"] == 1_000.0
    assert initial["guidance.target.position.east"] == 500.0
    assert initial["guidance.relative.velocity.north"] == -175.0
    assert initial["guidance.relative.velocity.east"] == 50.0
    assert float(initial["guidance.relative.time_to_closest_approach"]) > 0.0
    assert float(initial["guidance.relative.predicted_miss_distance"]) > 0.0
    assert terminal["guidance.target.position.north"] == pytest.approx(1_007.5)
    assert terminal["guidance.target.position.east"] == pytest.approx(515.0)
    assert response.result.objects[0].terminal_disposition == "duration"
    assert response.result.diagnostics[0].details["target_track_sensor_provider_kind"] == "relative-state-track"
    ####


@pytest.mark.parametrize("fidelity", ("point_mass_3dof", "attitude_response_pseudo_6dof"))
def test_invalid_native_target_track_disables_guidance_without_hiding_truth_range(fidelity: str) -> None:
    provider = _provider_with_target_sensor(
        "test.sensors.short-range-target-track",
        {"maximum_range_m": 100.0},
    )
    prepared = provider.validate_configuration(_configuration(provider, fidelity))
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id=f"unavailable-target-track-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )

    assert isinstance(response, MissionCompositionTrajectoryResponse)
    initial = response.result.objects[0].samples[0].values
    assert initial["sensor.target_track.applicable"] is True
    assert initial["sensor.target_track.valid"] is False
    assert initial["sensor.target_track.invalid_reason"] == "outside-range"
    assert initial["guidance.available"] is False
    assert initial["guidance.law.mode"] == "target_track_unavailable"
    assert initial["guidance.lateral_acceleration.commanded"] == 0.0
    assert all(
        initial[identifier] == 0.0
        for identifier in (
            "guidance.lateral_acceleration.commanded.local.north",
            "guidance.lateral_acceleration.commanded.local.east",
            "guidance.lateral_acceleration.commanded.local.vertical",
            "guidance.lateral_acceleration.achieved.local.north",
            "guidance.lateral_acceleration.achieved.local.east",
            "guidance.lateral_acceleration.achieved.local.vertical",
        )
    )
    assert initial["guidance.lateral_acceleration.achievement_fraction"] == 1.0
    assert initial["guidance.lateral_acceleration.direction_error.valid"] is False
    assert initial["guidance.lateral_acceleration.direction_error"] == 0.0
    assert float(initial["guidance.objective.range"]) > 1_000.0
    assert initial["guidance.objective.captured"] is False
    ####


def test_target_sensor_bias_changes_guidance_instead_of_only_telemetry() -> None:
    ideal = _provider()
    biased = _provider_with_target_sensor(
        "test.sensors.biased-target-track",
        {"azimuth_bias_rad": 0.2},
    )

    def initial_values(provider: ParametricInterceptorMissionCompositionProvider) -> dict[str, object]:
        prepared = provider.validate_configuration(
            provider.configuration(
                "moving-target-sam",
                fidelity="point_mass_3dof",
                mission_template_id=TARGET_TRACK_MISSION_TEMPLATE_ID,
                launch_altitude_m=100.0,
                launch_speed_mps=200.0,
                launch_flight_path_deg=0.0,
                navigation_target_position_north_command=1_000.0,
                navigation_target_position_east_command=0.0,
                navigation_target_position_altitude_command=100.0,
                navigation_target_capture_radius_command=10.0,
                runtime_duration_s=0.1,
                runtime_time_step_s=0.1,
            )
        )
        response = provider.build_runner().run(
            MissionCompositionRunRequest(
                request_id=f"target-bias-{provider.resolved_profile('moving-target-sam').fingerprint[:8]}",
                provider_id=provider.metadata.id,
                provider_version=provider.metadata.version,
                prepared_configuration=prepared,
                output=MissionCompositionOutputSelection(mode="all"),
            )
        )
        assert isinstance(response, MissionCompositionTrajectoryResponse)
        return dict(response.result.objects[0].samples[0].values)

    ideal_values = initial_values(ideal)
    biased_values = initial_values(biased)
    assert ideal_values["sensor.target_track.azimuth"] == pytest.approx(0.0)
    assert biased_values["sensor.target_track.azimuth"] == pytest.approx(0.2)
    assert ideal_values["guidance.lateral_acceleration.commanded"] == pytest.approx(0.0)
    biased_command = biased_values["guidance.lateral_acceleration.commanded"]
    assert isinstance(biased_command, int | float)
    assert biased_command > 0.0
    ####


@pytest.mark.parametrize("fidelity", ("point_mass_3dof", "attitude_response_pseudo_6dof"))
def test_noisy_target_track_checkpoint_replays_rng_and_does_not_double_sample(
    fidelity: str,
    tmp_path: Path,
) -> None:
    provider = _provider_with_target_sensor(
        "test.sensors.noisy-target-track",
        {
            "angular_noise_stddev_rad": 0.01,
            "relative_velocity_noise_stddev_mps": 0.5,
        },
    )
    prepared = provider.validate_configuration(_configuration(provider, fidelity))
    episode = provider.open_session_episode(prepared, seed=31, integration_step_s=0.1)

    first = episode.step({}, 0.2)
    assert first.observation.values["sensor.target_track.sequence"] == 2
    checkpoint = episode.save_checkpoint(tmp_path / f"noisy-target-{fidelity}.json")
    advanced = episode.step({}, 0.1)
    assert advanced.observation.values["sensor.target_track.sequence"] == 3
    episode.load_checkpoint(checkpoint)
    replayed = episode.step({}, 0.1)
    assert replayed.observation.values == advanced.observation.values
    ####


def test_fast_fly_through_is_localized_in_batch_and_latched_in_live_sessions() -> None:
    provider = _provider()
    localized_times: list[float] = []
    for fidelity in ("point_mass_3dof", "attitude_response_pseudo_6dof"):
        prepared = provider.validate_configuration(
            provider.configuration(
                "moving-target-sam",
                configuration_id=f"fly-through-{fidelity}",
                fidelity=fidelity,
                mission_template_id=TARGET_TRACK_MISSION_TEMPLATE_ID,
                launch_altitude_m=100.0,
                launch_speed_mps=200.0,
                launch_heading_deg=0.0,
                launch_flight_path_deg=0.0,
                navigation_target_position_north_command=10.0,
                navigation_target_position_east_command=0.0,
                navigation_target_position_altitude_command=100.0,
                navigation_target_velocity_north_command=0.0,
                navigation_target_velocity_east_command=0.0,
                navigation_target_velocity_vertical_command=0.0,
                navigation_target_capture_radius_command=0.5,
                runtime_duration_s=0.1,
                runtime_time_step_s=0.1,
            )
        )
        response = provider.build_runner().run(
            MissionCompositionRunRequest(
                request_id=f"fly-through-{fidelity}",
                provider_id=provider.metadata.id,
                provider_version=provider.metadata.version,
                prepared_configuration=prepared,
                output=MissionCompositionOutputSelection(mode="all"),
            )
        )
        assert isinstance(response, MissionCompositionTrajectoryResponse)
        vehicle = response.result.objects[0]
        terminal = vehicle.samples[-1]
        localized_times.append(terminal.time_s)
        assert vehicle.terminal_disposition == "target_intercept"
        assert 0.0 < terminal.time_s < 0.1
        assert terminal.values["guidance.objective.range"] == pytest.approx(0.5)
        assert terminal.values["guidance.objective.captured"] is True
        assert terminal.values["guidance.objective.capture_occurred"] is True

        manager = MissionCompositionSessionManager(provider)
        descriptor = manager.open(
            MissionCompositionOpenSessionRequest(
                session_id=f"fly-through-live-{fidelity}",
                provider_id=provider.metadata.id,
                provider_version=provider.metadata.version,
                prepared_configuration=prepared,
                integration_step_s=0.1,
            )
        )
        fly_through = manager.step(
            MissionCompositionSessionStepRequest(
                session_id=descriptor.session_id,
                action={},
                duration_s=0.1,
                expected_sequence=0,
            )
        )
        assert fly_through.events == ("target_intercept",)
        assert fly_through.observation.values["guidance.objective.captured"] is False
        assert fly_through.observation.values["guidance.objective.capture_occurred"] is True
        event_times = fly_through.lowering_evidence["objective_capture_event_times_s"]
        assert event_times == pytest.approx([terminal.time_s])

        retargeted = manager.step(
            MissionCompositionSessionStepRequest(
                session_id=descriptor.session_id,
                action={"navigation.target.position.north.command": 1_000.0},
                duration_s=0.1,
                expected_sequence=1,
            )
        )
        assert retargeted.events == ("target_track_updated",)
        assert retargeted.observation.values["guidance.objective.capture_occurred"] is False

    assert localized_times[0] == pytest.approx(localized_times[1])
    ####


@pytest.mark.parametrize("fidelity", ("point_mass_3dof", "attitude_response_pseudo_6dof"))
def test_target_intercept_is_an_explicit_outcome_and_live_track_can_be_reassigned(fidelity: str) -> None:
    provider = _provider()
    prepared = provider.validate_configuration(
        provider.configuration(
            "moving-target-sam",
            configuration_id=f"target-intercept-{fidelity}",
            fidelity=fidelity,
            mission_template_id=TARGET_TRACK_MISSION_TEMPLATE_ID,
            launch_altitude_m=100.0,
            navigation_target_position_north_command=0.0,
            navigation_target_position_east_command=0.0,
            navigation_target_position_altitude_command=100.0,
            navigation_target_capture_radius_command=10.0,
            runtime_duration_s=0.1,
            runtime_time_step_s=0.1,
        )
    )
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id=f"target-intercept-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )
    assert isinstance(response, MissionCompositionTrajectoryResponse)
    initial = response.result.objects[0].samples[0].values
    assert response.result.objects[0].terminal_disposition == "target_intercept"
    assert initial["phase.id"] == "target_intercept"
    assert initial["guidance.objective.captured"] is True
    assert initial["guidance.objective.range"] == 0.0

    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id=f"target-intercept-live-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.1,
        )
    )
    assert descriptor.initial_observation.values["guidance.objective.captured"] is True
    reassigned = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={"navigation.target.position.north.command": 1_000.0},
            duration_s=0.1,
            expected_sequence=0,
        )
    )
    assert reassigned.events == ("target_track_updated",)
    assert reassigned.observation.values["guidance.objective.captured"] is False
    assert reassigned.observation.values["guidance.objective.capture_occurred"] is False
    assert reassigned.observation.control_authority is not None
    assert reassigned.observation.control_authority.runtime_availability == "available"
    ####


@pytest.mark.parametrize("fidelity", ("point_mass_3dof", "attitude_response_pseudo_6dof"))
def test_target_track_session_selects_narrow_authority_and_rebases_partial_update(
    fidelity: str,
    tmp_path: Path,
) -> None:
    provider = _provider()
    prepared = provider.validate_configuration(_configuration(provider, fidelity))
    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id=f"moving-target-session-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.1,
        )
    )

    assert descriptor.active_authority_profile_id == "live_target_track_guidance"
    assert tuple(item.id for item in descriptor.action_schema) == _TARGET_ACTION_IDS
    assert all("waypoint" not in item.id for item in descriptor.action_schema)
    first = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={},
            duration_s=0.2,
            expected_sequence=0,
        )
    )
    assert first.observation.values["guidance.target.position.north"] == pytest.approx(1_005.0)
    assert first.observation.values["guidance.target.position.east"] == pytest.approx(510.0)
    batch = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id=f"moving-target-session-parity-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )
    assert isinstance(batch, MissionCompositionTrajectoryResponse)
    batch_at_two_tenths = batch.result.objects[0].samples[2].values
    for identifier in (
        "position.local.north",
        "position.local.east",
        "velocity.local.north",
        "velocity.local.east",
        "guidance.target.position.north",
        "guidance.target.position.east",
        "guidance.relative.velocity.north",
        "guidance.relative.predicted_miss_distance",
    ):
        assert first.observation.values[identifier] == pytest.approx(batch_at_two_tenths[identifier]), identifier
    second = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={"navigation.target.velocity.east.command": 80.0},
            duration_s=0.1,
            expected_sequence=1,
        )
    )
    assert second.events == ("target_track_updated",)
    assert second.observation.values["guidance.target.reference_position.north.accepted"] == pytest.approx(1_005.0)
    assert second.observation.values["guidance.target.reference_position.east.accepted"] == pytest.approx(510.0)
    assert second.observation.values["guidance.target.position.north"] == pytest.approx(1_007.5)
    assert second.observation.values["guidance.target.position.east"] == pytest.approx(518.0)
    assert second.lowering_evidence["accepted_guidance_objective"]["reference_time_s"] == pytest.approx(0.2)
    assert "constant_velocity_target_propagation" in second.lowering_evidence["lowering_chain"]
    assert second.lowering_evidence["target_track_sensor_provider_kind"] == "relative-state-track"
    assert second.lowering_evidence["target_track_sensor_schema_id"] == "taoryx.tracking.relative-state/v1"
    assert "taoryx_registered_relative_state_track" in second.lowering_evidence["lowering_chain"]
    feedback = {item.channel_id: item for item in second.control_feedback}
    assert feedback["navigation.target.velocity.east.command"].disposition == "applied_as_requested"
    assert feedback["navigation.target.position.north.command"].disposition == "held"

    episode = provider.open_session_episode(prepared, integration_step_s=0.1)
    episode.step({}, 0.2)
    checkpoint = episode.save_checkpoint(tmp_path / f"moving-target-{fidelity}.json")
    action = {"navigation.target.velocity.east.command": 80.0}
    advanced = episode.step(action, 0.1)
    episode.load_checkpoint(checkpoint)
    replayed = episode.step(action, 0.1)
    assert replayed.observation.values == advanced.observation.values
    assert replayed.events == advanced.events
    ####


def test_target_track_mission_rejects_waypoint_authority_and_fixed_waypoint_remains_stationary() -> None:
    provider = _provider()
    with pytest.raises(ConfigurationContractError, match="unknown-control-authority"):
        provider.validate_configuration(
            _configuration(
                provider,
                "point_mass_3dof",
                startup_authority_profile_id="live_waypoint_guidance",
            )
        )

    prepared = provider.validate_configuration(
        provider.configuration(
            "moving-target-sam",
            runtime_duration_s=0.1,
            runtime_time_step_s=0.1,
        )
    )
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id="stationary-waypoint-regression",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )
    assert isinstance(response, MissionCompositionTrajectoryResponse)
    samples = response.result.objects[0].samples
    assert all(item.values["guidance.objective.kind"] == "fixed_waypoint" for item in samples)
    assert all(item.values["guidance.target.position.north"] == 10_000.0 for item in samples)
    assert all(item.values["guidance.target.velocity.north.accepted"] == 0.0 for item in samples)
    assert all(item.values["sensor.target_track.applicable"] is False for item in samples)
    assert all(item.values["sensor.target_track.valid"] is False for item in samples)
    ####
