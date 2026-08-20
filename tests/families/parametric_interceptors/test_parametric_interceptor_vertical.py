"""One focused discovery-to-execution witness for the new plug-in."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import pytest
from taoryx_parametric_interceptors import (
    DEFAULT_ENVIRONMENT_MODEL_ID,
    DEFAULT_GRAVITY_MODEL_ID,
    DEFAULT_SENSOR_SUITE_ID,
    ParametricInterceptorMissionCompositionProvider,
    interceptor,
)
from taoryx_parametric_interceptors.plugin import PLUGIN

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.model_overview import build_model_overview_catalog, render_model_overview_markdown
from taoryx.plugins import DeferredMissionCompositionProvider, discover_plugins
from taoryx.runtime.environment_runtime import EnvironmentSample, StaticEnvironmentProvider
from taoryx.trajectory.execution_contract import (
    MissionCompositionExecutionError,
    MissionCompositionOutputSelection,
    MissionCompositionRunRequest,
    MissionCompositionTrajectoryResponse,
    audit_provider_advertisement,
)
from taoryx.trajectory.session_contract import (
    MissionCompositionCloseSessionRequest,
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionManager,
    MissionCompositionSessionStepRequest,
)


@dataclass(frozen=True)
class _EntryPoint:
    name: str
    value: str
    target: object

    def load(self) -> object:
        return self.target
        ####

    ####


def _provider() -> ParametricInterceptorMissionCompositionProvider:
    return ParametricInterceptorMissionCompositionProvider.from_profiles(
        interceptor(
            "vertical-sam",
            launch_mass_kg=420.0,
            propellant_fraction=0.42,
            length_m=5.2,
            body_diameter_m=0.36,
            maneuverability_class="high",
            guidance_family="sarh",
        )
    )
    ####


def test_model_advertises_profile_version_waypoint_controls_and_feedback() -> None:
    provider = _provider()
    model = provider.list_models()[0]
    realization = model.realizations[0]

    assert provider.metadata.version == "0.1.0a32"
    assert PLUGIN.metadata.version == "0.1.0a32"
    assert model.id == "vertical-sam"
    assert model.version == "0.1.0"
    assert model.fidelities[0].id == "point_mass_3dof"
    assert {item.id for item in model.fidelities} == {
        "point_mass_3dof",
        "attitude_response_pseudo_6dof",
    }
    assert {(item.from_fidelity, item.to_fidelity) for item in model.fidelity_transitions} == {
        ("point_mass_3dof", "attitude_response_pseudo_6dof"),
        ("attitude_response_pseudo_6dof", "point_mass_3dof"),
    }
    assert len(model.fidelities[0].profile_id or "") == 64
    properties = {item.id: item for item in model.presentation.properties}
    assert set(provider.resolved_profile(model.id).parameters) <= set(properties)
    assert "origin=simulation_assumption" in properties["launch_mass_kg"].provenance
    assert "origin=archetype_assumption" in properties["drag_coefficient"].provenance
    assert properties["calibration_screen_status"].value == "available_via_provider_api"
    assert properties["calibration_qualification"].value == "unqualified_surrogate_screen"
    assert "peak_speed_mps" in str(properties["calibration_observables"].value)
    assert properties["parameter_fit_status"].value == "available_via_authoring_api"
    assert "thrust_scale" in str(properties["parameter_fit_variables"].value)
    assert properties["parameter_fit_receipt_contract"].value == "taoryx.parametric-interceptors.fit-receipt/v1"
    assert properties["guidance_family"].value == "sarh"
    assert properties["guidance_archetype"].value == "waypoint_pursuit"
    assert properties["navigation_constant"].canonical_unit == "1"
    assert properties["control_allocation_policy"].value == "aerodynamic_first"
    assert properties["applicability.contract"].value == "taoryx.parametric-interceptors.applicability/v1"
    assert properties["applicability.enforcement"].value == "advisory"
    assert properties["applicability.declared"].value is False
    assert properties["applicability.bounds"].value == "none"
    assert properties["pseudo6.response_authority_coupling"].value == "current_lateral_command_support_fraction"
    assert properties["target_track.truth_fallback"].value == "disabled_for_guidance_and_attitude_commands"
    assert properties["pseudo6.flow_angle_contract"].value == "air_relative_velocity_projected_into_forward_right_down_response_body_frame"
    assert properties["maneuver_drag_factor"].value == 0.1
    assert "origin=archetype_assumption" in properties["maneuver_drag_factor"].provenance
    assert realization.controls.default_authority_id == "live_waypoint_guidance"
    assert realization.controls.authorities[0].scheme_id == "mission.waypoint"
    assert {item.id for item in realization.controls.authorities} == {
        "waypoint_guidance",
        "live_waypoint_guidance",
        "target_track_guidance",
        "live_target_track_guidance",
        "direct_lateral_acceleration",
        "live_direct_lateral_acceleration",
    }
    assert realization.operations == ("validate", "batch", "step")
    assert model.realizations[1].operations == ("validate", "batch", "step")
    assert model.realizations[1].controls.default_authority_id == "live_waypoint_guidance"
    assert {item.id for item in realization.controls.channels} == {
        "navigation.waypoint.north.command",
        "navigation.waypoint.east.command",
        "navigation.waypoint.altitude.command",
        "navigation.waypoint.capture_radius.command",
        "navigation.target.position.north.command",
        "navigation.target.position.east.command",
        "navigation.target.position.altitude.command",
        "navigation.target.velocity.north.command",
        "navigation.target.velocity.east.command",
        "navigation.target.velocity.vertical.command",
        "navigation.target.capture_radius.command",
        "control.lateral_acceleration.local.north.command",
        "control.lateral_acceleration.local.east.command",
        "control.lateral_acceleration.local.vertical.command",
    }
    assert {item.id for item in model.output_schema.telemetry_channels} >= {
        "guidance.lateral_acceleration.commanded",
        "guidance.lateral_acceleration.achieved",
        "guidance.lateral_acceleration.limit",
        "guidance.lateral_acceleration.utilization",
        "control.authority.configuration",
        "control.authority.aerodynamic",
        "control.authority.thrust_vector",
        "control.authority.combined_unclipped",
        "control.authority.available",
        "control.authority.utilization",
        "control.authority.structural_limit_active",
        "control.allocation.policy",
        "control.allocation.aerodynamic_achieved",
        "control.allocation.thrust_vector_achieved",
        "control.allocation.thrust_vector_angle",
        "control.attitude_response.authority_available",
        "control.attitude_response.command_support_fraction",
        "control.attitude_response.authority_limited",
        "aerodynamics.flow_angles.valid",
        "aerodynamics.relative_velocity.body.x",
        "aerodynamics.relative_velocity.body.y",
        "aerodynamics.relative_velocity.body.z",
        "aerodynamics.angle_of_attack",
        "aerodynamics.sideslip_angle",
        "propulsion.thrust.axial",
        "model.applicability.declared",
        "model.applicability.status",
        "model.applicability.reason",
        "guidance.law.id",
        "guidance.law.mode",
        "guidance.closing_speed",
        "guidance.line_of_sight_rate",
        "guidance.navigation_constant",
        "guidance.available",
        "control.limited",
        "control.limit.reason",
        "environment.density",
        "environment.pressure",
        "environment.temperature",
        "environment.speed_of_sound",
        "environment.wind.ecfc.x",
        "environment.wind.ecfc.y",
        "environment.wind.ecfc.z",
        "aerodynamics.airspeed",
        "aerodynamics.mach",
        "aerodynamics.drag_coefficient",
        "aerodynamics.maneuver.normal_force_coefficient",
        "aerodynamics.maneuver.drag_factor",
        "aerodynamics.maneuver.drag_coefficient",
        "aerodynamics.drag_coefficient.total",
        "aerodynamics.drag.base",
        "aerodynamics.drag.maneuver",
        "aerodynamics.drag",
        "guidance.waypoint.north.accepted",
        "guidance.waypoint.capture_radius.accepted",
        "guidance.objective.kind",
        "guidance.target.position.north",
        "guidance.target.velocity.north.accepted",
        "guidance.relative.time_to_closest_approach",
        "guidance.relative.predicted_miss_distance",
        "sensor.translation_acceleration.valid",
        "sensor.translation_acceleration.interval",
        "sensor.translation_acceleration.delta_velocity.eci.x",
    }
    control_by_id = {item.id: item for item in realization.controls.channels}
    assert control_by_id["navigation.waypoint.north.command"].provider_binding["feedback_channel_id"] == "guidance.waypoint.north.accepted"
    outputs = {item.id: item for item in model.output_schema.telemetry_channels}
    assert outputs["guidance.lateral_acceleration.limit"].canonical_unit == "m/s^2"
    assert outputs["guidance.lateral_acceleration.commanded"].frame is None
    assert outputs["guidance.lateral_acceleration.achieved"].frame is None
    assert outputs["guidance.lateral_acceleration.limit"].frame is None
    assert outputs["guidance.lateral_acceleration.utilization"].canonical_unit == "1"
    assert "[0, 1]" in outputs["guidance.lateral_acceleration.utilization"].description
    assert outputs["control.authority.available"].canonical_unit == "m/s^2"
    assert outputs["control.authority.utilization"].canonical_unit == "1"
    assert outputs["control.attitude_response.authority_available"].compatible_fidelities == ("attitude_response_pseudo_6dof",)
    assert outputs["control.attitude_response.command_support_fraction"].canonical_unit == "1"
    assert "zero demand reports 1" in outputs["control.attitude_response.command_support_fraction"].description
    assert outputs["control.attitude_response.authority_limited"].compatible_realizations == ("parametric_attitude_response_pseudo6",)
    assert outputs["aerodynamics.flow_angles.valid"].data_type == "boolean"
    assert outputs["aerodynamics.flow_angles.valid"].frame is None
    assert outputs["aerodynamics.angle_of_attack"].canonical_unit == "rad"
    assert outputs["aerodynamics.angle_of_attack"].frame == "body"
    assert outputs["aerodynamics.angle_of_attack"].compatible_fidelities == ("attitude_response_pseudo_6dof",)
    assert "atan2" in outputs["aerodynamics.sideslip_angle"].description
    assert "'none'" in outputs["control.limit.reason"].description
    assert "distinct" in outputs["guidance.law.id"].description
    assert outputs["guidance.line_of_sight_rate"].canonical_unit == "rad/s"
    assert outputs["environment.wind.ecfc.x"].frame == "ecfc"
    assert outputs["aerodynamics.airspeed"].canonical_unit == "m/s"
    assert outputs["aerodynamics.mach"].canonical_unit == "1"
    schema_parameters = {item.id: item for item in provider.get_model_schema(model.id).root.children}
    assert schema_parameters["runtime.environment_model_id"].choices == (DEFAULT_ENVIRONMENT_MODEL_ID,)
    assert schema_parameters["runtime.gravity_model_id"].choices == (DEFAULT_GRAVITY_MODEL_ID,)
    assert schema_parameters["runtime.environment_model_id"].default == DEFAULT_ENVIRONMENT_MODEL_ID
    assert schema_parameters["runtime.gravity_model_id"].default == DEFAULT_GRAVITY_MODEL_ID
    assert schema_parameters["runtime.sensor_suite_id"].choices == (DEFAULT_SENSOR_SUITE_ID,)
    assert schema_parameters["runtime.sensor_suite_id"].default == DEFAULT_SENSOR_SUITE_ID
    assert {item.id for item in model.reference_frames} >= {"local_ned", "body", "eci", "ecfc"}
    audit = audit_provider_advertisement(provider, provider.build_runner())
    assert audit.status == "pass", audit.diagnostics
    ####


@pytest.mark.parametrize("fidelity", ("point_mass_3dof", "attitude_response_pseudo_6dof"))
def test_proportional_navigation_is_shared_and_reports_live_capture_fallback(fidelity: str) -> None:
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(
        interceptor(
            "pn-guidance-sam",
            guidance_family="active_radar",
            guidance_archetype="proportional_navigation",
            navigation_constant=4.0,
        )
    )
    prepared = provider.validate_configuration(
        provider.configuration(
            "pn-guidance-sam",
            configuration_id=f"pn-guidance-{fidelity}",
            fidelity=fidelity,
            startup_authority_profile_id="live_waypoint_guidance",
            launch_altitude_m=100.0,
            launch_speed_mps=100.0,
            launch_heading_deg=0.0,
            launch_flight_path_deg=0.0,
            navigation_waypoint_north_command=10_000.0,
            navigation_waypoint_east_command=1_000.0,
            navigation_waypoint_altitude_command=100.0,
            runtime_duration_s=0.1,
            runtime_time_step_s=0.1,
        )
    )
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id=f"pn-guidance-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )
    assert isinstance(response, MissionCompositionTrajectoryResponse)
    initial = response.result.objects[0].samples[0].values
    assert initial["guidance.law.id"] == "proportional_navigation"
    assert initial["guidance.law.mode"] == "proportional_navigation"
    assert initial["guidance.navigation_constant"] == 4.0
    assert float(initial["guidance.closing_speed"]) > 0.0
    assert float(initial["guidance.line_of_sight_rate"]) > 0.0
    assert float(initial["guidance.lateral_acceleration.commanded"]) > 0.0

    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id=f"pn-guidance-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.1,
        )
    )
    assert descriptor.initial_observation.values["guidance.law.id"] == "proportional_navigation"
    retargeted = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={
                "navigation.waypoint.north.command": -10_000.0,
                "navigation.waypoint.east.command": 1_000.0,
            },
            duration_s=0.1,
            expected_sequence=0,
        )
    )
    assert retargeted.observation.values["guidance.law.id"] == "proportional_navigation"
    assert retargeted.observation.values["guidance.law.mode"] == "capture_fallback"
    assert float(retargeted.observation.values["guidance.closing_speed"]) < 0.0
    assert retargeted.lowering_evidence["guidance_archetype"] == "proportional_navigation"
    assert retargeted.lowering_evidence["guidance_mode"] == "capture_fallback"
    assert retargeted.lowering_evidence["guidance_navigation_constant"] == 4.0
    ####


def test_convenience_configuration_runs_through_common_batch_contract() -> None:
    provider = _provider()
    configuration = provider.configuration(
        "vertical-sam",
        configuration_id="vertical-sam-short-run",
        navigation_waypoint_north_command=5_000.0,
        navigation_waypoint_altitude_command=1_500.0,
        runtime_duration_s=2.0,
        runtime_time_step_s=0.1,
    )
    prepared = provider.validate_configuration(configuration)
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id="vertical-sam-request",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all", cadence_s=0.2),
        )
    )

    assert isinstance(response, MissionCompositionTrajectoryResponse)
    result = response.result
    vehicle = result.objects[0]
    assert result.primary_model_id == "vertical-sam"
    assert vehicle.fidelity == "point_mass_3dof"
    assert vehicle.samples[-1].time_s == 2.0
    assert vehicle.samples[-1].values["mass.total"] < vehicle.samples[0].values["mass.total"]
    assert vehicle.samples[-1].values["velocity.speed"] > vehicle.samples[0].values["velocity.speed"]
    assert vehicle.samples[0].values["sensor.translation_acceleration.valid"] is False
    assert all(sample.values["sensor.translation_acceleration.valid"] is True for sample in vehicle.samples[1:])
    assert vehicle.samples[1].values["sensor.translation_acceleration.interval"] == pytest.approx(0.1)
    assert vehicle.samples[0].values["aerodynamics.airspeed"] == pytest.approx(vehicle.samples[0].values["velocity.speed"])
    assert vehicle.samples[0].values["aerodynamics.mach"] == pytest.approx(
        float(vehicle.samples[0].values["aerodynamics.airspeed"]) / float(vehicle.samples[0].values["environment.speed_of_sound"])
    )
    for sample in vehicle.samples:
        limit = provider.resolved_profile("vertical-sam").number("max_lateral_acceleration_mps2")
        assert sample.values["guidance.lateral_acceleration.achieved"] <= limit + 1.0e-9
        assert sample.values["guidance.lateral_acceleration.limit"] == limit
        assert sample.values["guidance.lateral_acceleration.utilization"] == pytest.approx(
            float(sample.values["guidance.lateral_acceleration.achieved"]) / limit
        )
        assert (sample.values["control.limit.reason"] != "none") is sample.values["control.limited"]
        assert "sensor.imu.valid" not in sample.values
        assert "aerodynamics.angle_of_attack" not in sample.values
    ####


@pytest.mark.parametrize("fidelity", ("point_mass_3dof", "attitude_response_pseudo_6dof"))
def test_composition_selects_registered_environment_and_gravity_with_air_relative_feedback(
    fidelity: str,
    tmp_path: Path,
) -> None:
    environment_model_id = "test.environment.north-tailwind-v1"
    gravity_model_id = "test.gravity.zero-v1"
    environment = StaticEnvironmentProvider(
        EnvironmentSample(
            density=1.0,
            pressure=90_000.0,
            temperature=250.0,
            speed_of_sound=250.0,
            wind=FrameVector3(Vector3(0.0, 0.0, 20.0), Frame.ECFC),
        )
    )
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(
        interceptor(
            "environment-selectable-sam",
            launch_mass_kg=420.0,
            propellant_fraction=0.42,
            length_m=5.2,
            body_diameter_m=0.36,
        ),
        environment_models={environment_model_id: environment},
        gravity_models={gravity_model_id: lambda _: 0.0},
        default_environment_model_id=environment_model_id,
        default_gravity_model_id=gravity_model_id,
    )
    model = provider.model("environment-selectable-sam")
    schema_parameters = {item.id: item for item in provider.get_model_schema(model.id).root.children}
    assert schema_parameters["runtime.environment_model_id"].choices == (
        DEFAULT_ENVIRONMENT_MODEL_ID,
        environment_model_id,
    )
    assert schema_parameters["runtime.gravity_model_id"].choices == (
        DEFAULT_GRAVITY_MODEL_ID,
        gravity_model_id,
    )
    configuration = provider.configuration(
        model.id,
        configuration_id=f"environment-selection-{fidelity}",
        fidelity=fidelity,
        startup_authority_profile_id="live_waypoint_guidance",
        launch_altitude_m=100.0,
        launch_speed_mps=100.0,
        launch_heading_deg=0.0,
        launch_flight_path_deg=0.0,
        navigation_waypoint_north_command=10_000.0,
        navigation_waypoint_east_command=0.0,
        navigation_waypoint_altitude_command=100.0,
        runtime_duration_s=0.1,
        runtime_time_step_s=0.1,
    )
    prepared = provider.validate_configuration(configuration)
    assert prepared.resolved["runtime.environment_model_id"] == environment_model_id
    assert prepared.resolved["runtime.gravity_model_id"] == gravity_model_id
    built_in_dependencies = provider.validate_configuration(
        provider.configuration(
            model.id,
            configuration_id=configuration.configuration_id,
            fidelity=fidelity,
            startup_authority_profile_id="live_waypoint_guidance",
            launch_altitude_m=100.0,
            launch_speed_mps=100.0,
            launch_heading_deg=0.0,
            launch_flight_path_deg=0.0,
            navigation_waypoint_north_command=10_000.0,
            navigation_waypoint_altitude_command=100.0,
            runtime_duration_s=0.1,
            runtime_time_step_s=0.1,
            runtime_environment_model_id=DEFAULT_ENVIRONMENT_MODEL_ID,
            runtime_gravity_model_id=DEFAULT_GRAVITY_MODEL_ID,
        )
    )
    assert built_in_dependencies.fingerprint != prepared.fingerprint

    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id=f"environment-selection-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )
    assert isinstance(response, MissionCompositionTrajectoryResponse)
    samples = response.result.objects[0].samples
    assert samples[0].values["environment.pressure"] == 90_000.0
    assert samples[0].values["environment.temperature"] == 250.0
    assert samples[0].values["environment.speed_of_sound"] == 250.0
    assert samples[0].values["environment.wind.ecfc.z"] == 20.0
    assert samples[0].values["aerodynamics.airspeed"] == pytest.approx(80.0)
    assert samples[0].values["aerodynamics.mach"] == pytest.approx(0.32)
    assert samples[0].values["aerodynamics.dynamic_pressure"] == pytest.approx(3_200.0)
    if fidelity == "attitude_response_pseudo_6dof":
        assert samples[0].values["aerodynamics.flow_angles.valid"] is True
        assert samples[0].values["aerodynamics.relative_velocity.body.x"] == pytest.approx(80.0)
        assert samples[0].values["aerodynamics.relative_velocity.body.y"] == pytest.approx(0.0)
        assert samples[0].values["aerodynamics.relative_velocity.body.z"] == pytest.approx(0.0)
        assert samples[0].values["aerodynamics.angle_of_attack"] == pytest.approx(0.0)
        assert samples[0].values["aerodynamics.sideslip_angle"] == pytest.approx(0.0)
    else:
        assert "aerodynamics.flow_angles.valid" not in samples[0].values
    assert samples[-1].values["velocity.local.vertical"] == pytest.approx(0.0, abs=1.0e-12)
    dependency_details = response.result.diagnostics[0].details
    assert dependency_details["environment_model_id"] == environment_model_id
    assert dependency_details["gravity_model_id"] == gravity_model_id

    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id=f"environment-selection-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.1,
        )
    )
    assert descriptor.initial_observation.values["aerodynamics.airspeed"] == pytest.approx(80.0)
    assert ("aerodynamics.flow_angles.valid" in descriptor.initial_observation.values) is (fidelity == "attitude_response_pseudo_6dof")
    step = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={},
            duration_s=0.1,
            expected_sequence=0,
        )
    )
    assert step.lowering_evidence["environment_model_id"] == environment_model_id
    assert step.lowering_evidence["gravity_model_id"] == gravity_model_id
    assert step.observation.values["velocity.local.vertical"] == pytest.approx(0.0, abs=1.0e-12)
    assert step.observation.values["aerodynamics.airspeed"] == pytest.approx(samples[-1].values["aerodynamics.airspeed"])

    episode = provider.open_session_episode(prepared, integration_step_s=0.1)
    episode.step({}, 0.1)
    checkpoint = episode.save_checkpoint(tmp_path / f"{fidelity}-custom-runtime.json")
    action = {"navigation.waypoint.east.command": 250.0}
    advanced = episode.step(action, 0.1)
    episode.load_checkpoint(checkpoint)
    replayed = episode.step(action, 0.1)
    for identifier, expected in advanced.observation.values.items():
        actual = replayed.observation.values[identifier]
        if isinstance(expected, float):
            assert actual == pytest.approx(expected), identifier
        else:
            assert actual == expected, identifier
    ####


def test_pseudo6_fidelity_exposes_attitude_response_and_standard_imu_measurements() -> None:
    provider = _provider()
    configuration = provider.configuration(
        "vertical-sam",
        configuration_id="vertical-sam-pseudo6",
        fidelity="attitude_response_pseudo_6dof",
        navigation_waypoint_north_command=4_000.0,
        navigation_waypoint_east_command=2_000.0,
        navigation_waypoint_altitude_command=1_500.0,
        runtime_duration_s=1.0,
        runtime_time_step_s=0.1,
    )
    prepared = provider.validate_configuration(configuration)
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id="vertical-sam-pseudo6-request",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )

    assert isinstance(response, MissionCompositionTrajectoryResponse)
    vehicle = response.result.objects[0]
    profile = provider.resolved_profile("vertical-sam")
    assert vehicle.fidelity == "attitude_response_pseudo_6dof"
    assert vehicle.realization_id == "parametric_attitude_response_pseudo6"
    assert vehicle.samples[0].values["sensor.imu.valid"] is False
    assert "sensor.translation_acceleration.valid" not in vehicle.samples[0].values
    assert vehicle.samples[0].values["aerodynamics.flow_angles.valid"] is True
    assert all(sample.values["sensor.imu.valid"] is True for sample in vehicle.samples[1:])
    assert vehicle.samples[1].values["sensor.imu.interval"] == 0.1
    assert vehicle.samples[-1].values["attitude.yaw"] > vehicle.samples[0].values["attitude.yaw"]
    assert any(abs(float(sample.values["sensor.imu.delta_angle.body.z"])) > 0.0 for sample in vehicle.samples[1:])
    for sample in vehicle.samples:
        assert 0.0 <= float(sample.values["guidance.lateral_acceleration.utilization"]) <= 1.0
        assert isinstance(sample.values["control.attitude_response.authority_available"], bool)
        assert 0.0 <= float(sample.values["control.attitude_response.command_support_fraction"]) <= 1.0
        assert isinstance(sample.values["control.attitude_response.authority_limited"], bool)
        body_air_speed = math.sqrt(
            sum(
                float(sample.values[channel]) ** 2
                for channel in (
                    "aerodynamics.relative_velocity.body.x",
                    "aerodynamics.relative_velocity.body.y",
                    "aerodynamics.relative_velocity.body.z",
                )
            )
        )
        assert body_air_speed == pytest.approx(float(sample.values["aerodynamics.airspeed"]))
        assert (sample.values["control.limit.reason"] != "none") is sample.values["control.limited"]
        assert abs(float(sample.values["angular_velocity.body.p"])) <= profile.number("max_body_rate_rad_s") + 1.0e-9
        assert abs(float(sample.values["angular_velocity.body.q"])) <= profile.number("max_body_rate_rad_s") + 1.0e-9
        assert abs(float(sample.values["angular_velocity.body.r"])) <= profile.number("max_body_rate_rad_s") + 1.0e-9
        for channel in (
            "angular_acceleration.body.p",
            "angular_acceleration.body.q",
            "angular_acceleration.body.r",
        ):
            assert abs(float(sample.values[channel])) <= profile.number("max_body_acceleration_rad_s2") + 1.0e-9
    ####


@pytest.mark.parametrize("fidelity", ("point_mass_3dof", "attitude_response_pseudo_6dof"))
def test_control_authority_readback_distinguishes_unlimited_and_saturated_commands(fidelity: str) -> None:
    provider = _provider()
    runner = provider.build_runner()

    def first_sample(*, configuration_id: str, speed_mps: float, north_m: float, east_m: float) -> dict[str, object]:
        prepared = provider.validate_configuration(
            provider.configuration(
                "vertical-sam",
                configuration_id=configuration_id,
                fidelity=fidelity,
                launch_speed_mps=speed_mps,
                launch_flight_path_deg=0.0,
                navigation_waypoint_north_command=north_m,
                navigation_waypoint_east_command=east_m,
                navigation_waypoint_altitude_command=0.0,
                runtime_duration_s=0.1,
                runtime_time_step_s=0.1,
            )
        )
        response = runner.run(
            MissionCompositionRunRequest(
                request_id=configuration_id,
                provider_id=provider.metadata.id,
                provider_version=provider.metadata.version,
                prepared_configuration=prepared,
                output=MissionCompositionOutputSelection(mode="all"),
            )
        )
        assert isinstance(response, MissionCompositionTrajectoryResponse)
        return dict(response.result.objects[0].samples[0].values)
        ####

    aligned = first_sample(
        configuration_id=f"{fidelity}-aligned",
        speed_mps=20.0,
        north_m=10_000.0,
        east_m=0.0,
    )
    assert aligned["control.limited"] is False
    assert aligned["control.limit.reason"] == "none"
    assert aligned["guidance.lateral_acceleration.utilization"] == pytest.approx(0.0)

    cross_range = first_sample(
        configuration_id=f"{fidelity}-cross-range",
        speed_mps=1_000.0,
        north_m=0.0,
        east_m=10_000.0,
    )
    limit = provider.resolved_profile("vertical-sam").number("max_lateral_acceleration_mps2")
    assert cross_range["guidance.lateral_acceleration.limit"] == limit
    assert cross_range["control.limited"] is True
    reasons = str(cross_range["control.limit.reason"]).split("+")
    assert reasons[0] == "lateral_acceleration_command_saturation"
    assert len(reasons) == len(set(reasons))
    if fidelity == "point_mass_3dof":
        assert reasons == ["lateral_acceleration_command_saturation"]
        assert cross_range["guidance.lateral_acceleration.utilization"] == pytest.approx(1.0)
    else:
        assert "bank_angle_saturation" in reasons
        assert "attitude_acceleration_saturation" in reasons
        assert cross_range["guidance.lateral_acceleration.utilization"] == pytest.approx(0.0)
    ####


def test_point_mass_session_retargets_held_waypoint_with_standard_feedback() -> None:
    provider = _provider()
    prepared = provider.validate_configuration(
        provider.configuration(
            "vertical-sam",
            configuration_id="vertical-sam-live-waypoint",
            startup_authority_profile_id="live_waypoint_guidance",
            runtime_duration_s=2.0,
            runtime_time_step_s=0.1,
        )
    )
    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id="vertical-sam-live-waypoint",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.1,
        )
    )

    assert descriptor.active_authority_profile_id == "live_waypoint_guidance"
    assert all(not item.id.startswith(("attitude.", "sensor.imu.")) for item in descriptor.observation_schema)
    assert descriptor.initial_observation.values["sensor.translation_acceleration.valid"] is False
    assert {item.id for item in descriptor.observation_schema} >= {
        "sensor.translation_acceleration.valid",
        "sensor.translation_acceleration.interval",
        "sensor.translation_acceleration.delta_velocity.eci.x",
    }
    assert [item.id for item in descriptor.action_schema] == [
        "navigation.waypoint.north.command",
        "navigation.waypoint.east.command",
        "navigation.waypoint.altitude.command",
        "navigation.waypoint.capture_radius.command",
    ]
    first = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={
                "navigation.waypoint.north.command": 2_000.0,
                "navigation.waypoint.east.command": 500.0,
                "navigation.waypoint.altitude.command": 1_200.0,
                "navigation.waypoint.capture_radius.command": 20.0,
            },
            duration_s=0.2,
            expected_sequence=0,
        )
    )

    assert first.events == ("waypoint_retargeted",)
    assert first.observation.values["guidance.waypoint.north.accepted"] == 2_000.0
    assert first.lowering_evidence["native_transition"] == "PointMassKernel.evaluate+advance+sample"
    assert first.lowering_evidence["sensor_suite_id"] == DEFAULT_SENSOR_SUITE_ID
    assert first.lowering_evidence["sensor_provider_kind"] == "translation-acceleration"
    assert first.lowering_evidence["lateral_acceleration_limit_mps2"] == first.observation.values["guidance.lateral_acceleration.limit"]
    assert first.lowering_evidence["lateral_acceleration_utilization"] == first.observation.values["guidance.lateral_acceleration.utilization"]
    assert first.lowering_evidence["lateral_acceleration_command_vector_mps2"] == {
        "north": first.observation.values["guidance.lateral_acceleration.commanded.local.north"],
        "east": first.observation.values["guidance.lateral_acceleration.commanded.local.east"],
        "vertical": first.observation.values["guidance.lateral_acceleration.commanded.local.vertical"],
    }
    assert first.lowering_evidence["lateral_acceleration_achieved_vector_mps2"] == {
        "north": first.observation.values["guidance.lateral_acceleration.achieved.local.north"],
        "east": first.observation.values["guidance.lateral_acceleration.achieved.local.east"],
        "vertical": first.observation.values["guidance.lateral_acceleration.achieved.local.vertical"],
    }
    assert (
        first.lowering_evidence["lateral_acceleration_achievement_fraction"] == first.observation.values["guidance.lateral_acceleration.achievement_fraction"]
    )
    assert (
        first.lowering_evidence["lateral_acceleration_direction_error_valid"] == first.observation.values["guidance.lateral_acceleration.direction_error.valid"]
    )
    assert first.lowering_evidence["lateral_acceleration_direction_error_rad"] == first.observation.values["guidance.lateral_acceleration.direction_error"]
    assert first.lowering_evidence["control_limit_reason"] == first.observation.values["control.limit.reason"]
    assert first.observation.values["sensor.translation_acceleration.valid"] is True
    assert first.observation.values["sensor.translation_acceleration.interval"] == pytest.approx(0.1)
    feedback = {item.channel_id: item for item in first.control_feedback}
    assert feedback["navigation.waypoint.north.command"].disposition == "applied_as_requested"
    assert feedback["navigation.waypoint.north.command"].achievement_status == "observed"
    assert feedback["navigation.waypoint.north.command"].achieved_value == 2_000.0

    held = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={"navigation.waypoint.east.command": -500.0},
            duration_s=0.2,
            expected_sequence=1,
        )
    )
    held_feedback = {item.channel_id: item for item in held.control_feedback}
    assert held.observation.values["guidance.waypoint.north.accepted"] == 2_000.0
    assert held.observation.values["guidance.waypoint.east.accepted"] == -500.0
    assert held_feedback["navigation.waypoint.north.command"].disposition == "held"
    assert held_feedback["navigation.waypoint.east.command"].disposition == "applied_as_requested"
    assert manager.close(MissionCompositionCloseSessionRequest(session_id=descriptor.session_id)).lifecycle == "closed"
    ####


def test_pseudo6_session_retargets_with_attitude_and_standard_imu_feedback() -> None:
    provider = _provider()
    prepared = provider.validate_configuration(
        provider.configuration(
            "vertical-sam",
            configuration_id="vertical-sam-pseudo6-live-waypoint",
            fidelity="attitude_response_pseudo_6dof",
            startup_authority_profile_id="live_waypoint_guidance",
            runtime_duration_s=1.0,
            runtime_time_step_s=0.1,
        )
    )
    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id="vertical-sam-pseudo6-live-waypoint",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.1,
        )
    )

    assert descriptor.active_authority_profile_id == "live_waypoint_guidance"
    assert descriptor.initial_observation.values["sensor.imu.valid"] is False
    assert {item.id for item in descriptor.observation_schema} >= {
        "attitude.roll.commanded",
        "attitude.roll",
        "angular_velocity.body.p",
        "aerodynamics.angle_of_attack",
        "aerodynamics.sideslip_angle",
        "sensor.imu.valid",
        "sensor.imu.interval",
    }
    assert all(not item.id.startswith("sensor.translation_acceleration.") for item in descriptor.observation_schema)
    result = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={
                "navigation.waypoint.east.command": 1_000.0,
                "navigation.waypoint.altitude.command": 2_000.0,
            },
            duration_s=0.2,
            expected_sequence=0,
        )
    )

    assert result.events == ("waypoint_retargeted",)
    assert result.observation.values["sensor.imu.valid"] is True
    assert result.observation.values["sensor.imu.interval"] == pytest.approx(0.1)
    assert float(result.observation.values["attitude.yaw"]) > 0.0
    assert result.lowering_evidence["sensor_suite_id"] == DEFAULT_SENSOR_SUITE_ID
    assert result.lowering_evidence["sensor_provider_kind"] == "ideal"
    assert result.lowering_evidence["native_transition"] == "Pseudo6Kernel.evaluate+advance+sample"
    assert result.lowering_evidence["lateral_acceleration_limit_mps2"] == result.observation.values["guidance.lateral_acceleration.limit"]
    assert result.lowering_evidence["lateral_acceleration_utilization"] == result.observation.values["guidance.lateral_acceleration.utilization"]
    assert result.lowering_evidence["lateral_acceleration_command_vector_mps2"] == {
        "north": result.observation.values["guidance.lateral_acceleration.commanded.local.north"],
        "east": result.observation.values["guidance.lateral_acceleration.commanded.local.east"],
        "vertical": result.observation.values["guidance.lateral_acceleration.commanded.local.vertical"],
    }
    assert result.lowering_evidence["lateral_acceleration_achieved_vector_mps2"] == {
        "north": result.observation.values["guidance.lateral_acceleration.achieved.local.north"],
        "east": result.observation.values["guidance.lateral_acceleration.achieved.local.east"],
        "vertical": result.observation.values["guidance.lateral_acceleration.achieved.local.vertical"],
    }
    assert (
        result.lowering_evidence["lateral_acceleration_achievement_fraction"] == result.observation.values["guidance.lateral_acceleration.achievement_fraction"]
    )
    assert (
        result.lowering_evidence["lateral_acceleration_direction_error_valid"]
        == result.observation.values["guidance.lateral_acceleration.direction_error.valid"]
    )
    assert result.lowering_evidence["lateral_acceleration_direction_error_rad"] == result.observation.values["guidance.lateral_acceleration.direction_error"]
    assert result.lowering_evidence["control_limit_reason"] == result.observation.values["control.limit.reason"]
    assert result.lowering_evidence["attitude_response_authority_available"] == result.observation.values["control.attitude_response.authority_available"]
    assert (
        result.lowering_evidence["attitude_response_command_support_fraction"]
        == result.observation.values["control.attitude_response.command_support_fraction"]
    )
    assert result.lowering_evidence["attitude_response_authority_limited"] == result.observation.values["control.attitude_response.authority_limited"]
    assert result.lowering_evidence["flow_angles_valid"] == result.observation.values["aerodynamics.flow_angles.valid"]
    assert result.lowering_evidence["air_relative_velocity_body_mps"] == {
        "x": result.observation.values["aerodynamics.relative_velocity.body.x"],
        "y": result.observation.values["aerodynamics.relative_velocity.body.y"],
        "z": result.observation.values["aerodynamics.relative_velocity.body.z"],
    }
    assert result.lowering_evidence["angle_of_attack_rad"] == result.observation.values["aerodynamics.angle_of_attack"]
    assert result.lowering_evidence["sideslip_angle_rad"] == result.observation.values["aerodynamics.sideslip_angle"]
    feedback = {item.channel_id: item for item in result.control_feedback}
    assert feedback["navigation.waypoint.east.command"].disposition == "applied_as_requested"
    assert feedback["navigation.waypoint.north.command"].disposition == "held"
    assert feedback["navigation.waypoint.east.command"].achieved_value == 1_000.0
    ####


def test_pseudo6_session_matches_batch_and_restores_imu_checkpoint(tmp_path: Path) -> None:
    provider = _provider()
    configuration = provider.configuration(
        "vertical-sam",
        configuration_id="vertical-sam-pseudo6-session-parity",
        fidelity="attitude_response_pseudo_6dof",
        runtime_duration_s=0.3,
        runtime_time_step_s=0.1,
    )
    prepared = provider.validate_configuration(configuration)
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id="vertical-sam-pseudo6-session-parity-batch",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )
    assert isinstance(response, MissionCompositionTrajectoryResponse)
    batch_values = response.result.objects[0].samples[-1].values

    episode = provider.open_session_episode(prepared, integration_step_s=0.1)
    assert episode.observe().values["sensor.imu.valid"] is False
    assert episode.observe().values["sensor.imu.valid"] is False
    streamed = episode.step({}, 0.3)
    for identifier, expected in batch_values.items():
        actual = streamed.observation.values[identifier]
        if isinstance(expected, float):
            assert actual == pytest.approx(expected), identifier
        else:
            assert actual == expected, identifier

    checkpoint = episode.save_checkpoint(tmp_path / "pseudo6-session.json")
    retarget = {"navigation.waypoint.east.command": 750.0}
    advanced = episode.step(retarget, 0.2)
    episode.load_checkpoint(checkpoint)
    replayed = episode.step(retarget, 0.2)
    for identifier, expected in advanced.observation.values.items():
        actual = replayed.observation.values[identifier]
        if isinstance(expected, float):
            assert actual == pytest.approx(expected), identifier
        else:
            assert actual == expected, identifier
    assert replayed.events == advanced.events
    assert replayed.observation.values["sensor.imu.valid"] is True
    assert replayed.observation.values["sensor.imu.interval"] == pytest.approx(0.1)
    ####


def test_pseudo6_session_rejects_unstable_integration_step() -> None:
    provider = _provider()
    prepared = provider.validate_configuration(
        provider.configuration(
            "vertical-sam",
            fidelity="attitude_response_pseudo_6dof",
            runtime_time_step_s=0.1,
        )
    )

    with pytest.raises(ValueError, match="spectral radius"):
        provider.open_session_episode(prepared, integration_step_s=5.0)
    ####


@pytest.mark.parametrize("fidelity", ("point_mass_3dof", "attitude_response_pseudo_6dof"))
def test_waypoint_capture_phase_keeps_live_retargeting_available(fidelity: str) -> None:
    provider = _provider()
    prepared = provider.validate_configuration(
        provider.configuration(
            "vertical-sam",
            fidelity=fidelity,
            startup_authority_profile_id="live_waypoint_guidance",
            navigation_waypoint_north_command=0.0,
            navigation_waypoint_east_command=0.0,
            navigation_waypoint_altitude_command=0.0,
            navigation_waypoint_capture_radius_command=100.0,
            runtime_duration_s=0.2,
            runtime_time_step_s=0.1,
        )
    )
    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id=f"vertical-sam-capture-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.1,
        )
    )

    initial = descriptor.initial_observation
    authority = initial.control_authority
    assert initial.values["phase.id"] == "waypoint_capture"
    assert initial.values["guidance.available"] is False
    assert initial.values["vehicle.operational"] is True
    assert authority is not None
    assert authority.phase_id == "waypoint_capture"
    assert authority.runtime_availability == "available"
    assert authority.available_action_ids

    resumed = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={
                "navigation.waypoint.north.command": 2_000.0,
                "navigation.waypoint.altitude.command": 1_000.0,
            },
            duration_s=0.1,
            expected_sequence=0,
        )
    )
    assert resumed.events == ("waypoint_retargeted",)
    assert resumed.observation.values["phase.id"] == "boost"
    assert resumed.observation.values["guidance.available"] is True
    assert resumed.observation.control_authority is not None
    assert resumed.observation.control_authority.runtime_availability == "available"
    ####


@pytest.mark.parametrize("fidelity", ("point_mass_3dof", "attitude_response_pseudo_6dof"))
def test_ground_impact_completes_session_and_masks_live_authority(fidelity: str) -> None:
    provider = _provider()
    prepared = provider.validate_configuration(
        provider.configuration(
            "vertical-sam",
            fidelity=fidelity,
            startup_authority_profile_id="live_waypoint_guidance",
            launch_altitude_m=1.0,
            launch_speed_mps=50.0,
            launch_flight_path_deg=-89.0,
            navigation_waypoint_north_command=2_000.0,
            navigation_waypoint_altitude_command=0.0,
            runtime_duration_s=0.2,
            runtime_time_step_s=0.05,
        )
    )
    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id=f"vertical-sam-impact-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.05,
        )
    )
    impact = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={},
            duration_s=0.1,
            expected_sequence=0,
        )
    )

    authority = impact.observation.control_authority
    assert impact.observation.lifecycle == "completed"
    assert impact.observation.values["phase.id"] == "ground_impact"
    assert impact.observation.values["vehicle.operational"] is False
    assert impact.observation.values["guidance.available"] is False
    assert authority is not None
    assert authority.phase_id == "ground_impact"
    assert authority.runtime_availability == "temporarily_unavailable"
    assert authority.availability_reason_codes == ("episode_terminal",)
    assert authority.available_action_ids == ()

    with pytest.raises(MissionCompositionExecutionError) as unavailable:
        manager.step(
            MissionCompositionSessionStepRequest(
                session_id=descriptor.session_id,
                action={"navigation.waypoint.north.command": 3_000.0},
                duration_s=0.1,
                expected_sequence=1,
            )
        )
    assert unavailable.value.diagnostic.code == "action-temporarily-unavailable"
    assert unavailable.value.diagnostic.details["reason_codes"] == ["episode_terminal"]
    ####


@pytest.mark.parametrize("fidelity", ("point_mass_3dof", "attitude_response_pseudo_6dof"))
def test_batch_ground_impact_takes_precedence_at_duration_boundary(fidelity: str) -> None:
    provider = _provider()
    prepared = provider.validate_configuration(
        provider.configuration(
            "vertical-sam",
            fidelity=fidelity,
            launch_altitude_m=1.0,
            launch_speed_mps=50.0,
            launch_flight_path_deg=-89.0,
            navigation_waypoint_north_command=2_000.0,
            navigation_waypoint_altitude_command=0.0,
            runtime_duration_s=0.1,
            runtime_time_step_s=0.1,
        )
    )
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id=f"vertical-sam-impact-boundary-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )

    assert isinstance(response, MissionCompositionTrajectoryResponse)
    vehicle = response.result.objects[0]
    assert vehicle.terminal_disposition == "ground_impact"
    assert vehicle.samples[-1].values["phase.id"] == "ground_impact"
    assert vehicle.samples[-1].values["vehicle.operational"] is False
    ####


@pytest.mark.parametrize("fidelity", ("point_mass_3dof", "attitude_response_pseudo_6dof"))
def test_propulsion_burnout_preserves_coasting_waypoint_authority(fidelity: str) -> None:
    provider = _provider()
    prepared = provider.validate_configuration(
        provider.configuration(
            "vertical-sam",
            fidelity=fidelity,
            startup_authority_profile_id="live_waypoint_guidance",
            launch_altitude_m=100_000.0,
            launch_speed_mps=100.0,
            launch_flight_path_deg=0.0,
            navigation_waypoint_north_command=200_000.0,
            navigation_waypoint_altitude_command=100_000.0,
            runtime_duration_s=8.1,
            runtime_time_step_s=0.1,
        )
    )
    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id=f"vertical-sam-burnout-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.1,
        )
    )
    coast = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={},
            duration_s=8.1,
            expected_sequence=0,
        )
    )

    authority = coast.observation.control_authority
    assert coast.observation.values["propulsion.available"] is False
    assert coast.observation.values["propulsion.phase"] == "burnout"
    assert coast.observation.values["phase.id"] == "waypoint_guidance"
    assert coast.observation.values["guidance.available"] is True
    assert authority is not None
    assert authority.runtime_availability == "available"
    assert authority.available_action_ids
    ####


def test_point_mass_session_matches_batch_and_checkpoint_replay(tmp_path: Path) -> None:
    provider = _provider()
    configuration = provider.configuration(
        "vertical-sam",
        configuration_id="vertical-sam-session-parity",
        runtime_duration_s=0.3,
        runtime_time_step_s=0.1,
    )
    prepared = provider.validate_configuration(configuration)
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id="vertical-sam-session-parity-batch",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )
    assert isinstance(response, MissionCompositionTrajectoryResponse)
    batch_values = response.result.objects[0].samples[-1].values

    episode = provider.open_session_episode(prepared, integration_step_s=0.1)
    assert episode.observe().values["sensor.translation_acceleration.valid"] is False
    assert episode.observe().values["sensor.translation_acceleration.valid"] is False
    streamed = episode.step({}, 0.3)
    for identifier, expected in batch_values.items():
        actual = streamed.observation.values[identifier]
        if isinstance(expected, float):
            assert actual == pytest.approx(expected), identifier
        else:
            assert actual == expected, identifier

    checkpoint = episode.save_checkpoint(tmp_path / "point-mass-session.json")
    retarget = {"navigation.waypoint.east.command": 750.0}
    advanced = episode.step(retarget, 0.2)
    episode.load_checkpoint(checkpoint)
    replayed = episode.step(retarget, 0.2)
    for identifier, expected in advanced.observation.values.items():
        actual = replayed.observation.values[identifier]
        if isinstance(expected, float):
            assert actual == pytest.approx(expected), identifier
        else:
            assert actual == expected, identifier
    assert replayed.events == advanced.events
    assert replayed.observation.values["sensor.translation_acceleration.valid"] is True
    assert replayed.observation.values["sensor.translation_acceleration.interval"] == pytest.approx(0.1)
    ####


def test_entry_point_discovery_stays_deferred_and_package_scoped() -> None:
    catalog = discover_plugins(
        include_builtin=False,
        entry_points=(
            _EntryPoint(
                name="taoryx.parametric-interceptors",
                value="taoryx_parametric_interceptors.plugin:PLUGIN",
                target=PLUGIN,
            ),
        ),
    )

    assert tuple(item.id for item in catalog.plugins) == ("taoryx.parametric-interceptors",)
    contribution = catalog.contribution("mission_composition_provider", "taoryx.parametric-interceptors.mission-composition")
    assert contribution.plugin.package == "taoryx-parametric-interceptors"
    provider = catalog.build_mission_composition_provider_registry().provider("taoryx.parametric-interceptors.mission-composition")
    assert isinstance(provider, DeferredMissionCompositionProvider)
    assert {item.id for item in provider.list_models()} == {
        "generic-medium-sam",
        "aim9x-block2",
        "aim120-amraam-c5-c7",
    }
    plan = build_model_authoring_plan(
        catalog.build_mission_composition_provider_registry(),
        catalog.build_controller_tuning_campaign_registry(),
        "taoryx.parametric-interceptors.mission-composition",
        "aim9x-block2",
        fidelity="attitude_response_pseudo_6dof",
        mission_template_id="direct_lateral_acceleration_control",
    )
    assert plan["status"] == "ready_to_author"
    assert plan["selection"]["fidelity"] == "attitude_response_pseudo_6dof"
    assert plan["selection"]["mission_template_id"] == "direct_lateral_acceleration_control"
    ####


def test_model_card_advertises_developer_scope_and_extension_guides() -> None:
    """The installed card points developers to the profile authoring routes."""

    catalog = discover_plugins(
        include_builtin=False,
        entry_points=(
            _EntryPoint(
                name="taoryx.parametric-interceptors",
                value="taoryx_parametric_interceptors.plugin:PLUGIN",
                target=PLUGIN,
            ),
        ),
    )
    providers = catalog.build_mission_composition_provider_registry()
    overview = build_model_overview_catalog(
        catalog,
        providers,
        catalog.build_controller_tuning_campaign_registry(),
        provider_id="taoryx.parametric-interceptors.mission-composition",
        model_id="aim9x-block2",
    )

    markdown = render_model_overview_markdown(overview)

    assert "#### Provider scope" in markdown
    assert "Create and integrate provenance-preserving, low-fidelity SAM/interceptor surrogates" in markdown
    assert "[Parametric interceptor developer guide](packages/taoryx-parametric-interceptors/docs/model-architecture.md)" in markdown
    assert "[Profile quickstart and examples](packages/taoryx-parametric-interceptors/README.md)" in markdown
    assert "Add a new model from Python, flat YAML, or a provenance-bearing catalogue record" in markdown
    ####
