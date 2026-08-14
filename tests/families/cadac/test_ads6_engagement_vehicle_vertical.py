"""Focused vertical proof for the source-bound CADAC ADS6 engagement plug-in slice."""

from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ads6_engagement_mission_composition import ADS6_ENGAGEMENT_MODEL_ID
from taoryx.families.cadac.mission_composition_plugin import (
    CadacMissionCompositionProvider,
    CadacSourceCaseBindings,
    build_default_cadac_configuration,
)
from test_ads6_engagement import _write_source_controller_aircraft_engagement

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
    """Bind one source-controller ADS6 package without catalog fallback."""

    return CadacSourceCaseBindings(
        ads6_engagement_case_path=_write_source_controller_aircraft_engagement(tmp_path),
    ).build_provider(selected_model_ids=(ADS6_ENGAGEMENT_MODEL_ID,))
    ####


def test_bound_ads6_engagement_preserves_catalog_control_and_sensor_api(tmp_path: Path) -> None:
    """The three-root source package stays one exact source-owned endpoint."""

    provider = _provider(tmp_path)
    assert tuple(item.id for item in provider.list_models()) == (ADS6_ENGAGEMENT_MODEL_ID,)
    with pytest.raises(KeyError, match="unknown CADAC trajectory plug-in"):
        provider.get_model_schema("cadac.aim5.missile")
    model = provider.list_models()[0]
    configuration = build_default_cadac_configuration(provider, ADS6_ENGAGEMENT_MODEL_ID)
    prepared = provider.validate_configuration(configuration)
    runner = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(runner)
    audit = audit_provider_advertisement(provider, runner)

    assert prepared.resolved["package"]["command_law"] == "source_controller"
    assert runner.registrations() == ((CADAC_PROVIDER_ID, ADS6_ENGAGEMENT_MODEL_ID),)
    assert model.common_runner_operations == ("batch", "step")
    assert model.fidelities[0].input_realization == "actuator_allocated"
    assert model.realizations[0].controls.status == "internally_generated"
    authority = model.realizations[0].controls.authorities[0]
    assert authority.id == "source_program_control"
    assert authority.operations == ("batch", "step")
    assert audit.status == "pass", audit.model_dump(mode="json")

    outputs = {channel.id: channel for channel in model.output_schema.channels}
    for channel_id in ("requested_control_deg", "achieved_control_deg", "requested_fins_deg", "achieved_fins_deg"):
        assert outputs[channel_id].operations == ("batch", "step")
        assert outputs[channel_id].canonical_unit == "deg"
    ####
    assert outputs["requested_fins_deg"].shape == (4,)
    assert outputs["achieved_fins_deg"].shape == (4,)

    plan = build_model_authoring_plan(
        ConfigurableTrajectoryProviderRegistry((provider,)),
        ControllerTuningCampaignRegistry(),
        CADAC_PROVIDER_ID,
        ADS6_ENGAGEMENT_MODEL_ID,
        fidelity=model.fidelities[0].id,
        realization_id=model.realizations[0].id,
        mission_template_id=model.mission_templates[0].id,
    )
    controller = plan["controller_automation"]
    assert plan["status"] == "ready_to_author"
    assert isinstance(controller, dict)
    assert controller["status"] == "provider_managed"
    assert controller["input_realization"] == "actuator_allocated"
    assert controller["control_status"] == "internally_generated"
    assert controller["channels"] == []
    assert controller["authorities"][0]["operations"] == ["batch", "step"]

    response = runner.run(
        MissionCompositionRunRequest(
            request_id="cadac-ads6-engagement-vertical-batch",
            provider_id=CADAC_PROVIDER_ID,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=3),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json")
    assert response.result.provider_version == provider.metadata.version
    assert [item.object_id for item in response.result.objects] == ["m1", "a1", "f1"]
    missile_values = response.result.objects[0].samples[-1].values
    assert len(missile_values["requested_fins_deg"]) == 4
    assert len(missile_values["achieved_fins_deg"]) == 4
    assert isinstance(missile_values["normal_command_g"], float)
    assert isinstance(missile_values["achieved_normal_acceleration_g"], float)
    assert missile_values["telemetry"]["controller_mode"] == 3

    descriptor = provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="cadac-ads6-engagement-vertical-session",
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
    integration = provider.get_model_integration_contract(ADS6_ENGAGEMENT_MODEL_ID)

    assert descriptor.action_schema == ()
    assert descriptor.command_source_id == "cadac_source_program"
    assert step.observation.values["source_controller_state"]["source_schedule"] == "sam_then_target_then_radar"
    tracks = step.observation.values["native_relative_state_tracks"]
    assert tracks["ads6-engagement-m1-native-relative-state"]["valid"]
    assert len(step.observation.values["requested_fins_deg"]) == 4
    assert len(step.observation.values["achieved_fins_deg"]) == 4
    assert [packet.sequence for packet in provider.session_sensor_packets(descriptor.session_id)] == [0, 1]
    assert integration.step.state_semantics == "persistent_native_state"
    assert integration.sensor_integration.sensor_bus_status == "available"
    assert integration.controller_analysis.ownership == "source_owned"
    assert integration.controller_analysis.time_domain_analysis == "available"
    assert integration.controller_analysis.controller_comparison == "available"
    assert integration.controller_analysis.local_linear_stability == "blocked"
    assert set(integration.controller_analysis.command_output_channel_ids) >= {"normal_command_g", "lateral_command_g"}
    assert set(integration.controller_analysis.response_output_channel_ids) >= {
        "achieved_lateral_acceleration_g",
        "achieved_normal_acceleration_g",
    }
    assert integration.environment.atmosphere_owner == "cadac_compatibility_runtime"
    assert integration.environment.gravity_owner == "cadac_compatibility_runtime"
    assert integration.environment.host_environment_status == "blocked"
    provider.close_session(descriptor.session_id)
    ####
