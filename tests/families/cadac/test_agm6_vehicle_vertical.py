"""Focused vertical proof for the source-bound CADAC AGM6 plug-in slice."""

from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.agm6_mission_composition import (
    AGM6_AIRCRAFT_MODEL_ID,
    AGM6_FIDELITY_ID,
    AGM6_MISSION_ID,
    AGM6_MODEL_ID,
    AGM6_REALIZATION_ID,
    AGM6_TARGET_MODEL_ID,
)
from taoryx.families.cadac.mission_composition_plugin import (
    CadacMissionCompositionProvider,
    CadacSourceCaseBindings,
    build_default_cadac_configuration,
)
from test_agm6 import _write_agm6_case

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
    """Bind one test-only three-actor AGM6 case without catalog fallback."""

    return CadacSourceCaseBindings(
        agm6_case_path=_write_agm6_case(tmp_path),
    ).build_provider(selected_model_ids=(AGM6_MODEL_ID,))
    ####


def test_bound_agm6_preserves_catalog_identity_control_readback_and_sensor_api(tmp_path: Path) -> None:
    """The physical-fin composition remains one exact, source-owned endpoint."""

    provider = _provider(tmp_path)
    assert tuple(item.id for item in provider.list_models()) == (AGM6_MODEL_ID,)
    with pytest.raises(KeyError, match="unknown CADAC trajectory plug-in"):
        provider.get_model_schema("cadac.aim5.missile")
    model = provider.list_models()[0]
    configuration = build_default_cadac_configuration(provider, AGM6_MODEL_ID)
    prepared = provider.validate_configuration(configuration)
    runner = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(runner)
    audit = audit_provider_advertisement(provider, runner)

    assert runner.registrations() == ((CADAC_PROVIDER_ID, AGM6_MODEL_ID),)
    assert model.common_runner_operations == ("batch", "step")
    assert model.fidelities[0].id == AGM6_FIDELITY_ID
    assert model.fidelities[0].input_realization == "actuator_allocated"
    assert model.realizations[0].id == AGM6_REALIZATION_ID
    assert model.realizations[0].controls.status == "internally_generated"
    authority = model.realizations[0].controls.authorities[0]
    assert authority.id == "source_program_control"
    assert authority.operations == ("batch", "step")
    assert audit.status == "pass", audit.model_dump(mode="json")

    outputs = {channel.id: channel for channel in model.output_schema.channels}
    for channel_id in ("requested_control_deg", "requested_fins_deg", "achieved_fins_deg", "achieved_control_deg"):
        assert outputs[channel_id].operations == ("batch", "step")
        assert outputs[channel_id].canonical_unit == "deg"
    ####
    assert outputs["requested_fins_deg"].shape == (4,)
    assert outputs["achieved_fins_deg"].shape == (4,)

    plan = build_model_authoring_plan(
        ConfigurableTrajectoryProviderRegistry((provider,)),
        ControllerTuningCampaignRegistry(),
        CADAC_PROVIDER_ID,
        AGM6_MODEL_ID,
        fidelity=AGM6_FIDELITY_ID,
        realization_id=AGM6_REALIZATION_ID,
        mission_template_id=AGM6_MISSION_ID,
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
            request_id="cadac-agm6-vertical-batch",
            provider_id=CADAC_PROVIDER_ID,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=3),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json")
    assert response.result.provider_version == provider.metadata.version
    assert [item.model_id for item in response.result.objects] == [
        AGM6_MODEL_ID,
        AGM6_TARGET_MODEL_ID,
        AGM6_AIRCRAFT_MODEL_ID,
    ]
    missile_values = response.result.objects[0].samples[-1].values
    assert len(missile_values["requested_fins_deg"]) == 4
    assert len(missile_values["achieved_fins_deg"]) == 4
    assert isinstance(missile_values["normal_command_g"], float)
    assert isinstance(missile_values["normal_acceleration_g"], float)

    descriptor = provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="cadac-agm6-vertical-session",
            provider_id=CADAC_PROVIDER_ID,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.001,
        )
    )
    step = provider.step_session(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            duration_s=0.002,
        )
    )
    integration = provider.get_model_integration_contract(AGM6_MODEL_ID)

    assert descriptor.action_schema == ()
    assert descriptor.command_source_id == "cadac_source_program"
    assert step.observation.values["source_sensor_and_datalink_state"]["source_schedule"] == "missile_then_target_then_tracking_aircraft"
    assert step.observation.values["native_relative_state_track"]["valid"]
    assert len(step.observation.values["requested_fins_deg"]) == 4
    assert len(step.observation.values["achieved_fins_deg"]) == 4
    assert [packet.sequence for packet in provider.session_sensor_packets(descriptor.session_id)] == [0, 1, 2]
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
