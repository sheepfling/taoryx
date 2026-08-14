"""Focused vertical proof for the source-bound CADAC ROCKET6G plug-in slice."""

from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.mission_composition_plugin import (
    CadacMissionCompositionProvider,
    CadacSourceCaseBindings,
    build_default_cadac_configuration,
)
from taoryx.families.cadac.rocket6g_mission_composition import (
    ROCKET6G_FIDELITY_ID,
    ROCKET6G_MISSION_ID,
    ROCKET6G_MODEL_ID,
    ROCKET6G_MODEL_VERSION,
    ROCKET6G_REALIZATION_ID,
)
from test_rocket6g import _write_rocket_case

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


def _bound_rocket6g_provider(tmp_path: Path) -> CadacMissionCompositionProvider:
    """Bind only the synthetic ROCKET6G case into one selected catalog scope."""

    return CadacSourceCaseBindings(
        rocket6g_case_path=_write_rocket_case(tmp_path),
    ).build_provider(selected_model_ids=(ROCKET6G_MODEL_ID,))
    ####


def _rocket6g_configuration(provider: CadacMissionCompositionProvider) -> TrajectoryConfigurationInstance:
    """Prepare one short, command-bearing source phase program through the catalog API."""

    configuration = build_default_cadac_configuration(provider, ROCKET6G_MODEL_ID)
    if not isinstance(configuration.root, ConfigurationGroupValue):
        raise TypeError("ROCKET6G configuration root must be a group")
    ####
    root_values: dict[str, ConfigurationNodeValue] = dict(configuration.root.values)
    root_values["commands"] = ConfigurationGroupValue(
        values={
            "tvc_pitch_command_deg": ConfigurationParameterValue(value=1.0, unit="deg"),
            "tvc_yaw_command_deg": ConfigurationParameterValue(value=-0.5, unit="deg"),
            "thrust_vector_unit_body": ConfigurationParameterValue(value=(1.0, 0.01, -0.01)),
            "roll_command_deg": ConfigurationParameterValue(value=1.0, unit="deg"),
            "pitch_command_deg": ConfigurationParameterValue(value=2.0, unit="deg"),
            "yaw_command_deg": ConfigurationParameterValue(value=-3.0, unit="deg"),
        }
    )
    root_values["runtime"] = ConfigurationGroupValue(
        values={
            "enable_boost_cutoff": ConfigurationParameterValue(value=True),
            "boost_cutoff_time_s": ConfigurationParameterValue(value=4.2, unit="s"),
            "end_time_s": ConfigurationParameterValue(value=4.8, unit="s"),
            "sample_step_s": ConfigurationParameterValue(value=0.1, unit="s"),
        }
    )
    return configuration.model_copy(update={"root": ConfigurationGroupValue(values=root_values)})
    ####


def test_bound_rocket6g_preserves_one_model_scope_and_control_analysis_boundary(
    tmp_path: Path,
) -> None:
    """The selected launch vehicle retains caller-owned batch controls and readback."""

    provider = _bound_rocket6g_provider(tmp_path)
    runner = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(runner)

    assert provider.metadata.model_count == 1
    assert tuple(item.id for item in provider.list_models()) == (ROCKET6G_MODEL_ID,)
    assert runner.registrations() == ((provider.metadata.id, ROCKET6G_MODEL_ID),)
    with pytest.raises(KeyError, match="unknown CADAC trajectory plug-in"):
        provider.get_model_schema("cadac.ghame3.hypersonic_vehicle")

    model = provider.list_models()[0]
    controls = model.realizations[0].controls
    integration = provider.get_model_integration_contract(ROCKET6G_MODEL_ID)
    audit = audit_provider_advertisement(provider, runner)

    assert model.version == ROCKET6G_MODEL_VERSION
    assert model.common_runner_operations == ("batch",)
    assert tuple(item.id for item in model.fidelities) == (ROCKET6G_FIDELITY_ID,)
    assert model.fidelities[0].dynamics_fidelity == "rigid_body_6dof"
    assert model.fidelities[0].input_realization == "actuator_allocated"
    assert model.capabilities.supports_staging
    assert controls.status == "available"
    assert tuple(channel.id for channel in controls.channels) == (
        "tvc.pitch.deflection",
        "tvc.yaw.deflection",
        "rcs.thrust_vector.direction",
        "rcs.roll.attitude_command",
        "rcs.pitch.attitude_command",
        "rcs.yaw.attitude_command",
    )
    assert all(channel.operations == ("batch",) for channel in controls.channels)
    assert controls.authorities[0].id == "direct_tvc_and_rcs_commands"
    assert controls.authorities[0].command_owner == "caller"
    assert audit.status == "pass", audit.model_dump(mode="json")

    assert integration.step.status == "blocked"
    assert integration.step.state_semantics == "batch_only"
    assert integration.step.action_semantics == "configuration_fixed"
    assert integration.controller_analysis.ownership == "external_at_source_boundary"
    assert integration.controller_analysis.command_output_channel_ids == (
        "requested_tvc_control_deg",
        "requested_thrust_vector_unit_body",
        "requested_rcs_attitude_deg",
    )
    assert integration.controller_analysis.response_output_channel_ids == (
        "achieved_nozzle_deg",
        "rcs_force_body_n",
        "rcs_moment_body_nm",
    )
    assert integration.controller_analysis.time_domain_analysis == "available"
    assert integration.controller_analysis.controller_comparison == "available"
    assert integration.controller_analysis.local_linear_stability == "blocked"
    assert integration.sensor_integration.sensor_bus_status == "not_applicable"

    prepared = provider.validate_configuration(_rocket6g_configuration(provider))
    with pytest.raises(ValueError, match="no installed persistent session binding"):
        provider.open_session(
            MissionCompositionOpenSessionRequest(
                session_id="cadac-rocket6g-must-not-fabricate-session",
                provider_id=provider.metadata.id,
                provider_version=provider.metadata.version,
                prepared_configuration=prepared,
                integration_step_s=0.1,
            )
        )

    with pytest.raises(ValueError, match="omits explicit source bindings"):
        omitted_scope = tmp_path / "omitted-scope"
        omitted_scope.mkdir()
        CadacSourceCaseBindings(
            rocket6g_case_path=_write_rocket_case(omitted_scope),
        ).build_provider(selected_model_ids=("cadac.ghame3.hypersonic_vehicle",))
    ####


def test_bound_rocket6g_runs_through_catalog_batch_api_with_control_readback(
    tmp_path: Path,
) -> None:
    """The common wrapper returns phase labels and requested/achieved control evidence."""

    provider = _bound_rocket6g_provider(tmp_path)
    prepared = provider.validate_configuration(_rocket6g_configuration(provider))
    registry = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(registry)

    plan = build_model_authoring_plan(
        ConfigurableTrajectoryProviderRegistry((provider,)),
        ControllerTuningCampaignRegistry(),
        provider.metadata.id,
        ROCKET6G_MODEL_ID,
        fidelity=ROCKET6G_FIDELITY_ID,
        realization_id=ROCKET6G_REALIZATION_ID,
        mission_template_id=ROCKET6G_MISSION_ID,
    )
    controller = plan["controller_automation"]
    assert plan["status"] == "ready_to_author"
    assert isinstance(controller, dict)
    assert controller["status"] == "campaign_registration_required"
    assert controller["control_status"] == "available"
    assert [channel["id"] for channel in controller["channels"]] == [
        "tvc.pitch.deflection",
        "tvc.yaw.deflection",
        "rcs.thrust_vector.direction",
        "rcs.roll.attitude_command",
        "rcs.pitch.attitude_command",
        "rcs.yaw.attitude_command",
    ]
    assert controller["authorities"][0]["command_owner"] == "caller"
    assert controller["authorities"][0]["operations"] == ["batch"]

    response = registry.run(
        MissionCompositionRunRequest(
            request_id="cadac-rocket6g-vertical-batch",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    assert response.result.status == "completed"
    assert response.result.provider_version == provider.metadata.version
    assert [item.model_id for item in response.result.objects] == [ROCKET6G_MODEL_ID]
    vehicle = response.result.objects[0]
    assert vehicle.fidelity == ROCKET6G_FIDELITY_ID
    assert {
        "position_inertial_m",
        "velocity_inertial_mps",
        "quaternion_wxyz",
        "body_rates_inertial_rad_s",
        "active_stage",
        "source_phase",
        "runtime_fidelity",
        "rcs_force_mode",
        "requested_tvc_control_deg",
        "achieved_nozzle_deg",
        "requested_thrust_vector_unit_body",
        "rcs_force_body_n",
        "rcs_moment_body_nm",
    } <= {channel.id for channel in vehicle.channels}
    values = [sample.values for sample in vehicle.samples]
    assert {sample["active_stage"] for sample in values} == {1, 2, 3}
    assert {sample["source_phase"] for sample in values} >= {"aggregate_rcs", "mixed_tvc_rcs"}
    assert {sample["runtime_fidelity"] for sample in values} >= {
        "rigid_body_6dof_direct_wrench",
        "rigid_body_6dof_surface_allocated",
    }
    assert any(tuple(sample["requested_tvc_control_deg"]) == pytest.approx((1.0, -0.5)) for sample in values)
    assert any(any(abs(float(value)) > 0.0 for value in sample["achieved_nozzle_deg"]) for sample in values)
    assert all(sample["rcs_force_mode"] == 0 for sample in values)
    assert all(not any(abs(float(value)) > 0.0 for value in sample["rcs_force_body_n"]) for sample in values)
    assert any(any(abs(float(value)) > 0.0 for value in sample["rcs_moment_body_nm"]) for sample in values)
    assert tuple(event.kind for event in response.result.events) == tuple(f"source_event_{index}" for index in range(5))
    ####
