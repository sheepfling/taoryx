"""Focused vertical proof for the source-bound CADAC GHAME6 composition package."""

from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ghame6_mission_composition import (
    GHAME6_FIDELITY_ID,
    GHAME6_MISSION_ID,
    GHAME6_MODEL_ID,
    GHAME6_MODEL_VERSION,
    GHAME6_RADAR_MODEL_ID,
    GHAME6_REALIZATION_ID,
    GHAME6_SATELLITE_MODEL_ID,
)
from taoryx.families.cadac.mission_composition_plugin import (
    CadacMissionCompositionProvider,
    CadacSourceCaseBindings,
    build_default_cadac_configuration,
)
from test_ghame6 import _write_ghame6_case

from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistry
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.trajectory.configuration_contract import (
    ConfigurableTrajectoryProviderRegistry,
    ConfigurationGroupValue,
    ConfigurationNodeValue,
    ConfigurationParameterValue,
    TrajectoryConfigurationInstance,
)
from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
    audit_provider_advertisement,
)
from taoryx.trajectory.mission_composition import MissionCompositionOpenSessionRequest


def _bound_ghame6_provider(tmp_path: Path) -> CadacMissionCompositionProvider:
    """Bind only the synthetic GHAME6 package into one selected catalog scope."""

    return CadacSourceCaseBindings(
        ghame6_case_path=_write_ghame6_case(tmp_path),
    ).build_provider(selected_model_ids=(GHAME6_MODEL_ID,))
    ####


def _ghame6_configuration(provider: CadacMissionCompositionProvider) -> TrajectoryConfigurationInstance:
    """Prepare a short, command-bearing full source phase program through the catalog API."""

    configuration = build_default_cadac_configuration(provider, GHAME6_MODEL_ID)
    if not isinstance(configuration.root, ConfigurationGroupValue):
        raise TypeError("GHAME6 configuration root must be a group")
    ####
    root_values: dict[str, ConfigurationNodeValue] = dict(configuration.root.values)
    root_values["commands"] = ConfigurationGroupValue(
        values={
            "aileron_command_deg": ConfigurationParameterValue(value=1.0, unit="deg"),
            "elevator_command_deg": ConfigurationParameterValue(value=2.0, unit="deg"),
            "rudder_command_deg": ConfigurationParameterValue(value=-0.5, unit="deg"),
            "thrust_vector_unit_body": ConfigurationParameterValue(value=(1.0, 0.01, -0.01)),
            "roll_command_deg": ConfigurationParameterValue(value=1.0, unit="deg"),
            "pitch_command_deg": ConfigurationParameterValue(value=2.0, unit="deg"),
            "yaw_command_deg": ConfigurationParameterValue(value=-3.0, unit="deg"),
            "alpha_command_deg": ConfigurationParameterValue(value=0.3, unit="deg"),
            "beta_command_deg": ConfigurationParameterValue(value=-0.1, unit="deg"),
            "lateral_acceleration_command_g": ConfigurationParameterValue(value=0.2, unit="g"),
            "normal_acceleration_command_g": ConfigurationParameterValue(value=0.4, unit="g"),
        }
    )
    root_values["runtime"] = ConfigurationGroupValue(
        values={
            "enable_boost_cutoff": ConfigurationParameterValue(value=True),
            "boost_cutoff_time_s": ConfigurationParameterValue(value=0.65, unit="s"),
            "enable_terminal_lock": ConfigurationParameterValue(value=True),
            "terminal_lock_time_s": ConfigurationParameterValue(value=0.85, unit="s"),
            "end_time_s": ConfigurationParameterValue(value=1.2, unit="s"),
            "sample_step_s": ConfigurationParameterValue(value=0.05, unit="s"),
            "random_seed": ConfigurationParameterValue(value=7),
        }
    )
    return configuration.model_copy(update={"root": ConfigurationGroupValue(values=root_values)})
    ####


def test_bound_ghame6_preserves_one_package_scope_and_native_sensor_boundary(
    tmp_path: Path,
) -> None:
    """GHAME6 keeps three source roots, caller controls, and no fabricated session."""

    provider = _bound_ghame6_provider(tmp_path)
    runner = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(runner)

    assert provider.metadata.model_count == 1
    assert tuple(item.id for item in provider.list_models()) == (GHAME6_MODEL_ID,)
    assert runner.registrations() == ((provider.metadata.id, GHAME6_MODEL_ID),)
    with pytest.raises(KeyError, match="unknown CADAC trajectory plug-in"):
        provider.get_model_schema("cadac.rocket6g.vehicle")

    model = provider.list_models()[0]
    controls = model.realizations[0].controls
    integration = provider.get_model_integration_contract(GHAME6_MODEL_ID)
    audit = audit_provider_advertisement(provider, runner)

    assert model.version == GHAME6_MODEL_VERSION
    assert model.model_kind == "mission_composition"
    assert model.common_runner_operations == ("batch", "step")
    assert tuple(item.id for item in model.fidelities) == (GHAME6_FIDELITY_ID,)
    assert model.fidelities[0].dynamics_fidelity == "rigid_body_6dof"
    assert model.output_schema.entity_output.supports_multiple_entities
    assert not model.output_schema.entity_output.supports_dynamic_spawning
    assert model.capabilities.supports_staging
    assert controls.status == "available"
    assert tuple(channel.id for channel in controls.channels) == (
        "actuator.aileron.deflection",
        "actuator.elevator.deflection",
        "actuator.rudder.deflection",
        "rcs.thrust_vector.direction",
        "rcs.roll.attitude_command",
        "rcs.pitch.attitude_command",
        "rcs.yaw.attitude_command",
        "rcs.alpha.command",
        "rcs.beta.command",
        "rcs.lateral_acceleration.command",
        "rcs.normal_acceleration.command",
    )
    assert all(channel.operations == ("batch",) for channel in controls.channels)
    assert controls.authorities[0].id == "direct_surface_and_rcs_commands"
    assert controls.authorities[0].command_owner == "caller"
    assert audit.status == "pass", audit.model_dump(mode="json")

    assert integration.step.status == "available"
    assert integration.step.state_semantics == "core_batch_replay"
    assert integration.step.action_semantics == "read_only_replay"
    assert integration.environment.execution_profile == "cadac_compat"
    assert integration.controller_analysis.ownership == "external_at_source_boundary"
    assert integration.controller_analysis.command_output_channel_ids == (
        "requested_control_deg",
        "requested_thrust_vector_unit_body",
        "requested_rcs_attitude_deg",
        "requested_rcs_incidence_deg",
        "requested_rcs_acceleration_g",
    )
    assert integration.controller_analysis.response_output_channel_ids == (
        "achieved_control_deg",
        "rcs_force_body_n",
        "rcs_moment_body_nm",
        "alpha_deg",
        "beta_deg",
    )
    assert integration.controller_analysis.time_domain_analysis == "available"
    assert integration.controller_analysis.controller_comparison == "available"
    assert integration.controller_analysis.local_linear_stability == "blocked"
    assert integration.sensor_integration.status == "available"
    assert integration.sensor_integration.native_sensor_provider == "relative-state-track"
    assert integration.sensor_integration.execution == "native_projection_in_source_module"
    assert integration.sensor_integration.sensor_bus_status == "blocked"
    assert any("RADAR0" in item for item in integration.sensor_integration.source_specialised_state)

    prepared = provider.validate_configuration(_ghame6_configuration(provider))
    with pytest.raises(ValueError, match="no installed native persistent session binding"):
        provider.open_session(
            MissionCompositionOpenSessionRequest(
                session_id="cadac-ghame6-must-not-fabricate-session",
                provider_id=provider.metadata.id,
                provider_version=provider.metadata.version,
                prepared_configuration=prepared,
                integration_step_s=0.05,
            )
        )
    with pytest.raises(ValueError, match="individual phases remain runtime-reported"):
        build_default_cadac_configuration(provider, GHAME6_MODEL_ID, phase_id="transfer_vector_rcs")
    with pytest.raises(ValueError, match="omits explicit source bindings"):
        CadacSourceCaseBindings(
            ghame6_case_path=_write_ghame6_case(tmp_path / "omitted-scope"),
        ).build_provider(selected_model_ids=("cadac.ghame3.hypersonic_vehicle",))
    ####


def test_bound_ghame6_runs_full_package_with_standard_radar_track_events(
    tmp_path: Path,
) -> None:
    """The selected batch path keeps roots, controls, source tracks, and native tracks distinct."""

    provider = _bound_ghame6_provider(tmp_path)
    prepared = provider.validate_configuration(_ghame6_configuration(provider))
    registry = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(registry)

    plan = build_model_authoring_plan(
        ConfigurableTrajectoryProviderRegistry((provider,)),
        ControllerTuningCampaignRegistry(),
        provider.metadata.id,
        GHAME6_MODEL_ID,
        fidelity=GHAME6_FIDELITY_ID,
        realization_id=GHAME6_REALIZATION_ID,
        mission_template_id=GHAME6_MISSION_ID,
    )
    controller = plan["controller_automation"]
    assert plan["status"] == "ready_to_author"
    assert isinstance(controller, dict)
    assert controller["status"] == "campaign_registration_required"
    assert controller["control_status"] == "available"
    assert [channel["id"] for channel in controller["channels"]] == [
        "actuator.aileron.deflection",
        "actuator.elevator.deflection",
        "actuator.rudder.deflection",
        "rcs.thrust_vector.direction",
        "rcs.roll.attitude_command",
        "rcs.pitch.attitude_command",
        "rcs.yaw.attitude_command",
        "rcs.alpha.command",
        "rcs.beta.command",
        "rcs.lateral_acceleration.command",
        "rcs.normal_acceleration.command",
    ]
    assert controller["authorities"][0]["command_owner"] == "caller"
    assert controller["authorities"][0]["operations"] == ["batch"]

    response = registry.run(
        MissionCompositionRunRequest(
            request_id="cadac-ghame6-package-vertical-batch",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    assert response.result.status == "completed"
    assert response.result.provider_version == provider.metadata.version
    assert [item.model_id for item in response.result.objects] == [
        GHAME6_MODEL_ID,
        GHAME6_SATELLITE_MODEL_ID,
        GHAME6_RADAR_MODEL_ID,
    ]
    assert [item.parent_object_id for item in response.result.objects] == [None, None, None]
    assert response.result.relationships == ()

    hyper = response.result.objects[0]
    assert hyper.fidelity == GHAME6_FIDELITY_ID
    assert {
        "position_inertial_m",
        "velocity_inertial_mps",
        "source_phase",
        "runtime_fidelity",
        "control_realization",
        "rcs_force_mode",
        "requested_control_deg",
        "achieved_control_deg",
        "requested_thrust_vector_unit_body",
        "rcs_force_body_n",
        "rcs_moment_body_nm",
    } <= {channel.id for channel in hyper.channels}
    values = [sample.values for sample in hyper.samples]
    assert {sample["source_phase"] for sample in values} == {
        "atmospheric_surfaces",
        "transfer_angle_rcs",
        "transfer_vector_rcs",
        "interceptor_glideslope_rcs",
        "interceptor_terminal_rcs",
    }
    assert {sample["runtime_fidelity"] for sample in values} == {
        "rigid_body_6dof_surface_allocated",
        "rigid_body_6dof_direct_wrench",
    }
    assert any(tuple(sample["requested_control_deg"]) == pytest.approx((1.0, 2.0, -0.5)) for sample in values)
    assert any(any(abs(float(value)) > 0.0 for value in sample["achieved_control_deg"]) for sample in values)
    assert any(any(abs(float(value)) > 0.0 for value in sample["rcs_moment_body_nm"]) for sample in values)

    source_track = next(event for event in response.result.events if event.kind == "radar_track_update")
    native_track = next(event for event in response.result.events if event.kind == "native_relative_state_track")
    assert "native_relative_state_packet" not in source_track.data
    assert native_track.object_id == "ghame6-radar-1"
    assert native_track.time_s == source_track.time_s
    assert native_track.data["sensor_id"] == "ghame6-radar0-native-relative-state"
    assert native_track.data["schema_id"] == "taoryx.tracking.relative-state/v1"
    assert native_track.data["port"] == "track"
    assert native_track.data["valid"] is True
    assert native_track.data["sequence"] == source_track.data["update_sequence"] - 1
    payload = native_track.data["payload"]
    assert payload["target_id"] == "ghame6-satellite-1"
    assert payload["frame_id"] == "cadac.ghame6.radar0.eci"
    assert payload["range_m"] == pytest.approx(source_track.data["true_range_m"])
    assert native_track.data["payload_contract"]["measurement"] == "relative-state-track"
    assert native_track.data["provenance"]["source_actor"] == "RADAR0"
    assert native_track.data["provenance"]["target_actor"] == "SAT3"
    assert any(item.code == "cadac-ghame6-native-radar-track" for item in response.result.diagnostics)
    ####
