"""Vertical direct lateral-acceleration control witnesses."""

from __future__ import annotations

import pytest
from taoryx_parametric_interceptors import (
    DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
    ParametricInterceptorMissionCompositionProvider,
    PointMassMission,
    evaluate_direct_lateral_acceleration,
    interceptor,
    resolve_interceptor,
    run_point_mass_interceptor,
    run_pseudo6_interceptor,
)

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

_DIRECT_ACTION_IDS = (
    "control.lateral_acceleration.local.north.command",
    "control.lateral_acceleration.local.east.command",
    "control.lateral_acceleration.local.vertical.command",
)
_ACCEPTED_FEEDBACK_IDS = (
    "control.lateral_acceleration.accepted.local.north",
    "control.lateral_acceleration.accepted.local.east",
    "control.lateral_acceleration.accepted.local.vertical",
)


def _resolved_profile():
    return resolve_interceptor(
        interceptor(
            "direct-acceleration-sam",
            launch_mass_kg=100.0,
            burnout_mass_kg=60.0,
            body_diameter_m=0.2,
            burn_time_s=4.0,
            nominal_thrust_n=2_000.0,
            maneuverability_class="high",
            control_bandwidth_class="fast",
        )
    )
    ####


def _provider() -> ParametricInterceptorMissionCompositionProvider:
    return ParametricInterceptorMissionCompositionProvider.from_profiles(
        interceptor(
            "direct-acceleration-sam",
            launch_mass_kg=100.0,
            burnout_mass_kg=60.0,
            body_diameter_m=0.2,
            burn_time_s=4.0,
            nominal_thrust_n=2_000.0,
            maneuverability_class="high",
            control_bandwidth_class="fast",
        )
    )
    ####


def _mission(**updates: object) -> PointMassMission:
    values: dict[str, object] = {
        "launch_altitude_m": 1_000.0,
        "launch_speed_mps": 100.0,
        "launch_heading_deg": 0.0,
        "launch_flight_path_deg": 0.0,
        "direct_lateral_acceleration_north_mps2": 5.0,
        "direct_lateral_acceleration_east_mps2": 10.0,
        "direct_lateral_acceleration_vertical_mps2": 0.0,
        "objective_kind": "direct_lateral_acceleration",
        "duration_s": 0.2,
        "time_step_s": 0.05,
    }
    values.update(updates)
    return PointMassMission(**values)  # type: ignore[arg-type]
    ####


def test_direct_command_is_projected_onto_current_lateral_plane() -> None:
    evaluation = evaluate_direct_lateral_acceleration(
        (5.0, 10.0, -2.0),
        reference_direction=(1.0, 0.0, 0.0),
    )

    assert evaluation.archetype == "external_lateral_acceleration"
    assert evaluation.mode == "direct_lateral_acceleration"
    assert evaluation.acceleration_mps2 == (0.0, 10.0, -2.0)
    assert evaluation.commanded_acceleration_mps2 == pytest.approx(104.0**0.5)
    assert evaluation.navigation_constant == 0.0
    ####


def test_both_tiers_execute_direct_control_without_fake_capture_or_target_sensor() -> None:
    profile = _resolved_profile()
    mission = _mission()

    point = run_point_mass_interceptor(profile, mission)
    pseudo = run_pseudo6_interceptor(profile, mission)

    for run in (point, pseudo):
        initial = run.samples[0]
        assert run.termination == "duration"
        assert initial.phase_id == "direct_lateral_acceleration_control"
        assert initial.guidance_objective_kind == "direct_lateral_acceleration"
        assert initial.guidance_archetype == "external_lateral_acceleration"
        assert initial.guidance_mode == "direct_lateral_acceleration"
        assert initial.direct_lateral_acceleration_accepted_vector_mps2 == (5.0, 10.0, 0.0)
        assert initial.lateral_acceleration_command_vector_mps2 == (0.0, 10.0, 0.0)
        assert initial.guidance_available
        assert not initial.target_track_applicable
        assert not initial.target_track_valid
        assert all(sample.standard_ecef.frame_id == "ecfc" for sample in run.samples)
        assert all(
            len(sample.standard_ecef.position_ecef_m) == 3
            and len(sample.standard_ecef.velocity_ecef_mps) == 3
            and len(sample.standard_ecef.acceleration_ecef_mps2) == 3
            and len(sample.standard_ecef.angular_velocity_body_radps) == 3
            and len(sample.standard_ecef.ecef_from_body_wxyz) == 4
            for sample in run.samples
        )

    assert point.samples[0].lateral_acceleration_achieved_vector_mps2[1] > 0.0
    assert point.samples[0].standard_ecef.orientation_kind == "kinematic_velocity_aligned"
    assert pseudo.samples[0].lateral_acceleration_achieved_vector_mps2 == (0.0, 0.0, 0.0)
    assert pseudo.samples[0].standard_ecef.orientation_kind == "native_ned_from_body"
    assert pseudo.samples[0].standard_ecef.angular_velocity_kind == "native_body_rate"
    assert pseudo.samples[-1].yaw_command_rad > 0.0
    assert pseudo.samples[-1].east_velocity_mps > 0.0
    ####


def test_direct_command_reports_shared_force_saturation() -> None:
    run = run_point_mass_interceptor(
        _resolved_profile(),
        _mission(
            direct_lateral_acceleration_north_mps2=0.0,
            direct_lateral_acceleration_east_mps2=10_000.0,
        ),
    )
    initial = run.samples[0]

    assert initial.control_limited
    assert "lateral_acceleration_command_saturation" in initial.control_limit_reason
    assert initial.lateral_acceleration_achieved_mps2 <= initial.lateral_acceleration_limit_mps2
    assert initial.lateral_acceleration_achievement_fraction < 1.0
    ####


def test_composition_advertises_direct_control_as_a_distinct_mission_and_authority() -> None:
    provider = _provider()
    model = provider.model("direct-acceleration-sam")

    assert DIRECT_ACCELERATION_MISSION_TEMPLATE_ID in {item.id for item in model.mission_templates}
    mission = next(item for item in model.mission_templates if item.id == DIRECT_ACCELERATION_MISSION_TEMPLATE_ID)
    assert mission.segment_sequence == ("direct_lateral_acceleration_control",)
    assert {(item.fidelity, item.operation) for item in mission.operations} >= {
        ("point_mass_3dof", "batch"),
        ("point_mass_3dof", "step"),
        ("attitude_response_pseudo_6dof", "batch"),
        ("attitude_response_pseudo_6dof", "step"),
    }
    for realization in model.realizations:
        controls = {item.id: item for item in realization.controls.channels}
        authorities = {item.id: item for item in realization.controls.authorities}
        assert authorities["direct_lateral_acceleration"].channel_ids == _DIRECT_ACTION_IDS
        assert authorities["live_direct_lateral_acceleration"].channel_ids == _DIRECT_ACTION_IDS
        assert authorities["direct_lateral_acceleration"].scheme_layer == "kinematic"
        for identifier, accepted_feedback_id in zip(
            _DIRECT_ACTION_IDS,
            _ACCEPTED_FEEDBACK_IDS,
            strict=True,
        ):
            assert controls[identifier].canonical_unit == "m/s^2"
            assert controls[identifier].frame == "local_neu"
            feedback_id = controls[identifier].provider_binding["feedback_channel_id"]
            assert feedback_id == accepted_feedback_id
        outputs = {item.id: item for item in model.output_schema.telemetry_channels}
        for accepted_feedback_id in _ACCEPTED_FEEDBACK_IDS:
            assert outputs[accepted_feedback_id].quantity == "acceleration"
            assert outputs[accepted_feedback_id].canonical_unit == "m/s^2"
            assert outputs[accepted_feedback_id].frame == "local_neu"
    ####


def test_direct_control_bounds_are_model_specific_and_agent_normalization_ready() -> None:
    provider = _provider()
    model = provider.model("direct-acceleration-sam")
    limit = provider.resolved_profile(model.id).number("max_lateral_acceleration_mps2")
    parameters = {item.id: item for item in provider.get_model_schema(model.id).root.children}
    direct_parameter = parameters["control.lateral_acceleration.local.east.command"]

    assert direct_parameter.interval is not None
    assert direct_parameter.interval.minimum is not None
    assert direct_parameter.interval.maximum is not None
    assert direct_parameter.interval.minimum.value == -limit
    assert direct_parameter.interval.maximum.value == limit
    controls = {item.id: item for item in model.realizations[0].controls.channels}
    for identifier in _DIRECT_ACTION_IDS:
        channel = controls[identifier]
        assert channel.interval == direct_parameter.interval
        assert channel.native_binding is not None
        assert channel.native_binding.interval == direct_parameter.interval
        assert channel.value_space.normalization_rule == "affine to [-1, 1]"
        assert channel.semantics.agent_normalization == "affine"
        assert channel.semantics.agent_clip is True
        assert "per-axis structural envelope" in channel.claim_boundary

    prepared = provider.validate_configuration(
        provider.configuration(
            model.id,
            mission_template_id=DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
            startup_authority_profile_id="live_direct_lateral_acceleration",
            launch_altitude_m=1_000.0,
            launch_speed_mps=100.0,
            launch_flight_path_deg=0.0,
        )
    )
    descriptor = MissionCompositionSessionManager(provider).open(
        MissionCompositionOpenSessionRequest(
            session_id="direct-normalization-metadata",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.05,
        )
    )
    assert descriptor.agent_action_space is not None
    assert descriptor.agent_action_space.requires_external_statistics is False
    for channel in descriptor.action_schema:
        assert channel.minimum == -limit
        assert channel.maximum == limit
        assert channel.agent_normalization == "affine"
        assert channel.agent_clip is True
    for channel in descriptor.agent_action_space.channels:
        assert channel.normalization == "affine"
        assert channel.native_minimum == -limit
        assert channel.native_maximum == limit
        assert channel.agent_minimum == -1.0
        assert channel.agent_maximum == 1.0

    with pytest.raises(ConfigurationContractError) as captured:
        provider.validate_configuration(
            provider.configuration(
                model.id,
                mission_template_id=DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
                control_lateral_acceleration_local_east_command=limit + 1.0,
            )
        )
    assert captured.value.code == "out-of-bounds"
    assert captured.value.path.endswith("control.lateral_acceleration.local.east.command")
    ####


def test_direct_control_envelope_varies_with_resolved_model_authority() -> None:
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(
        interceptor("low-authority-direct-sam", maneuverability_class="low"),
        interceptor("high-authority-direct-sam", maneuverability_class="high"),
    )
    maxima: dict[str, float] = {}
    for model_id in ("low-authority-direct-sam", "high-authority-direct-sam"):
        parameters = {item.id: item for item in provider.get_model_schema(model_id).root.children}
        interval = parameters["control.lateral_acceleration.local.east.command"].interval
        assert interval is not None
        assert interval.maximum is not None
        assert interval.maximum.value is not None
        maxima[model_id] = interval.maximum.value
        assert maxima[model_id] == provider.resolved_profile(model_id).number("max_lateral_acceleration_mps2")

    assert maxima["low-authority-direct-sam"] < maxima["high-authority-direct-sam"]
    ####


def test_component_valid_direct_vector_still_reports_vector_saturation() -> None:
    provider = _provider()
    model_id = "direct-acceleration-sam"
    limit = provider.resolved_profile(model_id).number("max_lateral_acceleration_mps2")
    prepared = provider.validate_configuration(
        provider.configuration(
            model_id,
            configuration_id="direct-vector-saturation",
            mission_template_id=DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
            launch_altitude_m=1_000.0,
            launch_speed_mps=100.0,
            launch_flight_path_deg=0.0,
            control_lateral_acceleration_local_east_command=limit,
            control_lateral_acceleration_local_vertical_command=limit,
            runtime_duration_s=0.05,
            runtime_time_step_s=0.05,
        )
    )
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id="direct-vector-saturation",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )

    assert isinstance(response, MissionCompositionTrajectoryResponse)
    initial = response.result.objects[0].samples[0].values
    assert initial["control.limited"] is True
    assert "lateral_acceleration_command_saturation" in initial["control.limit.reason"]
    assert initial["guidance.lateral_acceleration.commanded"] == pytest.approx(2.0**0.5 * limit)
    assert initial["guidance.lateral_acceleration.achieved"] <= limit
    ####


@pytest.mark.parametrize("fidelity", ("point_mass_3dof", "attitude_response_pseudo_6dof"))
def test_composition_batch_executes_direct_acceleration_with_explicit_readback(fidelity: str) -> None:
    provider = _provider()
    prepared = provider.validate_configuration(
        provider.configuration(
            "direct-acceleration-sam",
            configuration_id=f"direct-batch-{fidelity}",
            fidelity=fidelity,
            mission_template_id=DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
            startup_authority_profile_id="direct_lateral_acceleration",
            launch_altitude_m=1_000.0,
            launch_speed_mps=100.0,
            launch_heading_deg=0.0,
            launch_flight_path_deg=0.0,
            control_lateral_acceleration_local_north_command=5.0,
            control_lateral_acceleration_local_east_command=10.0,
            runtime_duration_s=0.2,
            runtime_time_step_s=0.05,
        )
    )
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id=f"direct-batch-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )

    assert isinstance(response, MissionCompositionTrajectoryResponse)
    initial = response.result.objects[0].samples[0].values
    assert response.result.objects[0].terminal_disposition == "duration"
    assert initial["phase.id"] == "direct_lateral_acceleration_control"
    assert initial["guidance.objective.kind"] == "direct_lateral_acceleration"
    assert initial["guidance.law.id"] == "external_lateral_acceleration"
    assert initial["guidance.law.mode"] == "direct_lateral_acceleration"
    assert initial["control.lateral_acceleration.accepted.local.north"] == pytest.approx(5.0)
    assert initial["control.lateral_acceleration.accepted.local.east"] == pytest.approx(10.0)
    assert initial["control.lateral_acceleration.accepted.local.vertical"] == pytest.approx(0.0)
    assert initial["guidance.lateral_acceleration.commanded.local.north"] == pytest.approx(0.0)
    assert initial["guidance.lateral_acceleration.commanded.local.east"] == pytest.approx(10.0)
    assert initial["guidance.lateral_acceleration.achieved.local.north"] == pytest.approx(0.0)
    assert 0.0 <= initial["guidance.lateral_acceleration.achieved.local.east"] <= 10.0
    assert initial["guidance.objective.captured"] is False
    assert initial["sensor.target_track.applicable"] is False
    assert initial["sensor.target_track.valid"] is False
    ####


@pytest.mark.parametrize("fidelity", ("point_mass_3dof", "attitude_response_pseudo_6dof"))
def test_live_direct_acceleration_holds_partial_updates_and_reports_realization(fidelity: str) -> None:
    provider = _provider()
    prepared = provider.validate_configuration(
        provider.configuration(
            "direct-acceleration-sam",
            configuration_id=f"direct-session-{fidelity}",
            fidelity=fidelity,
            mission_template_id=DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
            startup_authority_profile_id="live_direct_lateral_acceleration",
            launch_altitude_m=1_000.0,
            launch_speed_mps=100.0,
            launch_heading_deg=0.0,
            launch_flight_path_deg=0.0,
            runtime_duration_s=1.0,
            runtime_time_step_s=0.05,
        )
    )
    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id=f"direct-session-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.05,
        )
    )

    assert descriptor.active_authority_profile_id == "live_direct_lateral_acceleration"
    assert tuple(item.id for item in descriptor.action_schema) == _DIRECT_ACTION_IDS
    first = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={
                "control.lateral_acceleration.local.north.command": 5.0,
                "control.lateral_acceleration.local.east.command": 10.0,
            },
            duration_s=0.1,
            expected_sequence=0,
        )
    )
    assert first.events == ("direct_lateral_acceleration_updated",)
    assert first.applied_action == {
        "control.lateral_acceleration.local.north.command": 5.0,
        "control.lateral_acceleration.local.east.command": 10.0,
        "control.lateral_acceleration.local.vertical.command": 0.0,
    }
    assert first.lowering_evidence["lowering_chain"][:3] == [
        "live_local_neu_acceleration_hold",
        "velocity_transverse_projection",
        "shared_force_authority_allocation",
    ]
    assert first.observation.values["guidance.objective.kind"] == "direct_lateral_acceleration"
    assert first.observation.values["guidance.objective.capture_occurred"] is False
    assert first.observation.values["guidance.law.id"] == "external_lateral_acceleration"
    assert first.observation.values["sensor.target_track.applicable"] is False
    assert first.observation.values["control.lateral_acceleration.accepted.local.north"] == pytest.approx(5.0)
    assert first.observation.values["control.lateral_acceleration.accepted.local.east"] == pytest.approx(10.0)
    standard = first.observation.standard_ecef
    assert standard.frame_id == "ecfc"
    assert len(standard.position_ecef_m) == 3
    assert len(standard.velocity_ecef_mps) == 3
    assert len(standard.acceleration_ecef_mps2) == 3
    assert len(standard.angular_velocity_body_radps) == 3
    assert len(standard.ecef_from_body_wxyz) == 4
    if fidelity == "point_mass_3dof":
        assert standard.angular_velocity_kind == "orientation_finite_difference"
        assert any(abs(value) > 1.0e-6 for value in standard.angular_velocity_body_radps)
    feedback = {item.channel_id: item for item in first.control_feedback}
    assert feedback["control.lateral_acceleration.local.east.command"].achievement_status == "observed"
    assert feedback["control.lateral_acceleration.local.east.command"].achieved_value == pytest.approx(
        first.observation.values["control.lateral_acceleration.accepted.local.east"]
    )
    assert feedback["control.lateral_acceleration.local.north.command"].achieved_value == pytest.approx(
        first.observation.values["control.lateral_acceleration.accepted.local.north"]
    )
    assert feedback["control.lateral_acceleration.local.north.command"].disposition == "applied_as_requested"

    held = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={"control.lateral_acceleration.local.vertical.command": 3.0},
            duration_s=0.1,
            expected_sequence=1,
        )
    )
    held_feedback = {item.channel_id: item for item in held.control_feedback}
    assert held.applied_action["control.lateral_acceleration.local.east.command"] == 10.0
    assert held.applied_action["control.lateral_acceleration.local.north.command"] == 5.0
    assert held.applied_action["control.lateral_acceleration.local.vertical.command"] == 3.0
    assert held.observation.values["control.lateral_acceleration.accepted.local.north"] == pytest.approx(5.0)
    assert held.observation.values["control.lateral_acceleration.accepted.local.vertical"] == pytest.approx(3.0)
    assert held_feedback["control.lateral_acceleration.local.east.command"].disposition == "held"
    assert held_feedback["control.lateral_acceleration.local.vertical.command"].disposition == "applied_as_requested"
    ####
