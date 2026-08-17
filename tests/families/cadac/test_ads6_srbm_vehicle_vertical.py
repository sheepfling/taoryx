"""Focused vertical proof for the source-bound CADAC ADS6 SRBM plug-in slice."""

from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ads6_srbm_mission_composition import (
    ADS6_SRBM_FIDELITY_ID,
    ADS6_SRBM_MISSION_ID,
    ADS6_SRBM_MODEL_ID,
    ADS6_SRBM_MODEL_VERSION,
    ADS6_SRBM_REALIZATION_ID,
)
from taoryx.families.cadac.mission_composition_plugin import (
    CadacMissionCompositionProvider,
    CadacSourceCaseBindings,
    build_default_cadac_configuration,
)
from test_ads6_srbm import _write_ads6_srbm_case

from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistry
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.trajectory.configuration_contract import ConfigurableTrajectoryProviderRegistry
from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
    audit_provider_advertisement,
)
from taoryx.trajectory.mission_composition import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionStepRequest,
)

CADAC_PROVIDER_ID = "cadac"


def _provider(tmp_path: Path) -> CadacMissionCompositionProvider:
    """Bind one test-only guided ROCKET5 case without any catalog fallback."""

    return CadacSourceCaseBindings(
        ads6_srbm_case_path=_write_ads6_srbm_case(
            tmp_path,
            include_sensor=True,
            seeker_mode=1,
            guidance_mode=11,
        ),
    ).build_provider(selected_model_ids=(ADS6_SRBM_MODEL_ID,))
    ####


def test_bound_ads6_srbm_preserves_catalog_identity_controls_and_sensor_api(tmp_path: Path) -> None:
    """One source-bound SRBM reaches the normal API with source-owned control readback."""

    provider = _provider(tmp_path)
    model = next(item for item in provider.list_models() if item.id == ADS6_SRBM_MODEL_ID)
    configuration = build_default_cadac_configuration(provider, ADS6_SRBM_MODEL_ID)
    prepared = provider.validate_configuration(configuration)
    runner = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(runner)
    audit = audit_provider_advertisement(provider, runner)

    assert tuple(item.id for item in provider.list_models()) == (ADS6_SRBM_MODEL_ID,)
    with pytest.raises(KeyError, match="unknown CADAC trajectory plug-in"):
        provider.get_model_schema("cadac.aim5.missile")
    assert runner.registrations() == ((CADAC_PROVIDER_ID, ADS6_SRBM_MODEL_ID),)
    assert model.version == ADS6_SRBM_MODEL_VERSION
    assert model.common_runner_operations == ("batch", "step")
    assert model.realizations[0].id == ADS6_SRBM_REALIZATION_ID
    assert model.realizations[0].controls.status == "internally_generated"
    assert model.realizations[0].controls.default_authority_id == "source_program_control"
    assert audit.status == "pass", audit.model_dump(mode="json")

    outputs = {channel.id: channel for channel in model.output_schema.channels}
    assert outputs["normal_command_g"].operations == ("batch", "step")
    assert outputs["lateral_command_g"].operations == ("batch", "step")
    assert outputs["normal_acceleration_g"].canonical_unit == "g"
    assert outputs["lateral_acceleration_g"].canonical_unit == "g"

    plan = build_model_authoring_plan(
        ConfigurableTrajectoryProviderRegistry((provider,)),
        ControllerTuningCampaignRegistry(),
        CADAC_PROVIDER_ID,
        ADS6_SRBM_MODEL_ID,
        fidelity=ADS6_SRBM_FIDELITY_ID,
        realization_id=ADS6_SRBM_REALIZATION_ID,
        mission_template_id=ADS6_SRBM_MISSION_ID,
    )
    controller = plan["controller_automation"]
    assert plan["status"] == "ready_to_author"
    assert isinstance(controller, dict)
    assert controller["status"] == "provider_managed"
    assert controller["control_status"] == "internally_generated"
    assert controller["channels"] == []
    assert controller["default_authority_id"] == "source_program_control"

    response = runner.run(
        MissionCompositionRunRequest(
            request_id="cadac-ads6-srbm-vertical-batch",
            provider_id=CADAC_PROVIDER_ID,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=3),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json")
    assert response.result.provider_version == provider.metadata.version
    assert [item.model_id for item in response.result.objects] == [ADS6_SRBM_MODEL_ID]
    batch_values = response.result.objects[0].samples[-1].values
    assert isinstance(batch_values["normal_command_g"], float)
    assert isinstance(batch_values["lateral_command_g"], float)
    assert isinstance(batch_values["normal_acceleration_g"], float)
    assert isinstance(batch_values["lateral_acceleration_g"], float)

    descriptor = provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="cadac-ads6-srbm-vertical-session",
            provider_id=CADAC_PROVIDER_ID,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.01,
        )
    )
    step = provider.step_session(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            duration_s=0.01,
        )
    )
    integration = provider.get_model_integration_contract(ADS6_SRBM_MODEL_ID)

    assert descriptor.action_schema == ()
    assert descriptor.command_source_id == "cadac_source_program"
    assert descriptor.initial_observation.values["source_sensor_state"]["source_seeker_enabled"] is True
    assert step.observation.values["native_relative_state_track"]["valid"]
    assert isinstance(step.observation.values["normal_command_g"], float)
    assert isinstance(step.observation.values["normal_acceleration_g"], float)
    assert [packet.sequence for packet in provider.session_sensor_packets(descriptor.session_id)] == [0, 1]
    assert integration.step.state_semantics == "persistent_native_state"
    assert integration.sensor_integration.sensor_bus_status == "available"
    assert integration.controller_analysis.ownership == "source_owned"
    assert integration.controller_analysis.time_domain_analysis == "available"
    assert integration.controller_analysis.controller_comparison == "available"
    assert integration.controller_analysis.local_linear_stability == "blocked"
    assert set(integration.controller_analysis.command_output_channel_ids) >= {"normal_command_g", "lateral_command_g"}
    assert set(integration.controller_analysis.response_output_channel_ids) >= {
        "normal_acceleration_g",
        "lateral_acceleration_g",
    }
    assert integration.environment.atmosphere_owner == "cadac_compatibility_runtime"
    assert integration.environment.gravity_owner == "cadac_compatibility_runtime"
    assert integration.environment.host_environment_status == "blocked"
    provider.close_session(descriptor.session_id)
    ####
