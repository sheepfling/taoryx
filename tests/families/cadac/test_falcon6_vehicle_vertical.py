"""Focused vertical proof for the standalone CADAC FALCON6 physical-plant slice."""

from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.falcon6_mission_composition import (
    FALCON6_FIDELITY_ID,
    FALCON6_MISSION_ID,
    FALCON6_MODEL_ID,
    FALCON6_REALIZATION_ID,
)
from taoryx.families.cadac.mission_composition_plugin import (
    CadacMissionCompositionProvider,
    CadacSourceCaseBindings,
    build_default_cadac_configuration,
)
from test_falcon6 import _write_falcon_case

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
from taoryx.trajectory.session_contract import MissionCompositionSessionManager, MissionCompositionSessionStepRequest

CADAC_PROVIDER_ID = "cadac"


def _provider(tmp_path: Path) -> CadacMissionCompositionProvider:
    """Bind exactly one FALCON6 source plant without a catalog fallback."""

    return CadacSourceCaseBindings(
        falcon6_case_path=_write_falcon_case(tmp_path),
    ).build_provider(selected_model_ids=(FALCON6_MODEL_ID,))
    ####


def _configuration(
    provider: CadacMissionCompositionProvider,
    *,
    aileron_deg: float,
    elevator_deg: float,
    rudder_deg: float,
) -> TrajectoryConfigurationInstance:
    """Create one catalog configuration with fixed external surface commands."""

    configuration = build_default_cadac_configuration(provider, FALCON6_MODEL_ID)
    if not isinstance(configuration.root, ConfigurationGroupValue):
        raise TypeError("FALCON6 configuration root must be a group")
    ####
    root_values: dict[str, ConfigurationNodeValue] = dict(configuration.root.values)
    root_values["physical_controls"] = ConfigurationGroupValue(
        values={
            "aileron_command_deg": ConfigurationParameterValue(value=aileron_deg, unit="deg"),
            "elevator_command_deg": ConfigurationParameterValue(value=elevator_deg, unit="deg"),
            "rudder_command_deg": ConfigurationParameterValue(value=rudder_deg, unit="deg"),
        }
    )
    root_values["runtime"] = ConfigurationGroupValue(
        values={
            "end_time_s": ConfigurationParameterValue(value=0.1, unit="s"),
            "sample_step_s": ConfigurationParameterValue(value=0.01, unit="s"),
        }
    )
    return configuration.model_copy(update={"root": ConfigurationGroupValue(values=root_values)})
    ####


def test_bound_falcon6_preserves_physical_control_readback_and_limit_feedback(tmp_path: Path) -> None:
    """The direct-surface plug-in advertises batch-only control and true actuator feedback."""

    provider = _provider(tmp_path)
    assert tuple(item.id for item in provider.list_models()) == (FALCON6_MODEL_ID,)
    with pytest.raises(KeyError, match="unknown CADAC trajectory plug-in"):
        provider.get_model_schema("cadac.aim5.missile")
    model = provider.list_models()[0]
    configuration = _configuration(provider, aileron_deg=30.0, elevator_deg=5.0, rudder_deg=-25.0)
    prepared = provider.validate_configuration(configuration)
    runner = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(runner)
    audit = audit_provider_advertisement(provider, runner)

    assert runner.registrations() == ((CADAC_PROVIDER_ID, FALCON6_MODEL_ID),)
    assert model.common_runner_operations == ("batch", "step")
    assert model.fidelities[0].id == FALCON6_FIDELITY_ID
    assert model.fidelities[0].input_realization == "actuator_allocated"
    assert model.realizations[0].id == FALCON6_REALIZATION_ID
    controls = model.realizations[0].controls
    assert controls.status == "available"
    assert tuple(channel.id for channel in controls.channels) == (
        "actuator.aileron.deflection",
        "actuator.elevator.deflection",
        "actuator.rudder.deflection",
    )
    assert all(channel.operations == ("batch",) for channel in controls.channels)
    assert all(channel.canonical_unit == "deg" for channel in controls.channels)
    authority = controls.authorities[0]
    assert authority.id == "physical_surface_commands"
    assert authority.command_owner == "caller"
    assert authority.operations == ("batch",)
    assert audit.status == "pass", audit.model_dump(mode="json")

    outputs = {channel.id: channel for channel in model.output_schema.channels}
    for channel_id in ("requested_surfaces_deg", "achieved_surfaces_deg", "aero_surfaces_deg"):
        assert outputs[channel_id].operations == ("batch",)
        assert outputs[channel_id].canonical_unit == "deg"
        assert outputs[channel_id].shape == (3,)
    ####
    for channel_id in ("surface_position_limited", "surface_rate_limited"):
        assert outputs[channel_id].operations == ("batch",)
        assert outputs[channel_id].data_type == "boolean"
        assert outputs[channel_id].shape == (3,)

    plan = build_model_authoring_plan(
        ConfigurableTrajectoryProviderRegistry((provider,)),
        ControllerTuningCampaignRegistry(),
        CADAC_PROVIDER_ID,
        FALCON6_MODEL_ID,
        fidelity=FALCON6_FIDELITY_ID,
        realization_id=FALCON6_REALIZATION_ID,
        mission_template_id=FALCON6_MISSION_ID,
    )
    controller = plan["controller_automation"]
    assert plan["status"] == "ready_to_author"
    assert isinstance(controller, dict)
    assert controller["status"] == "campaign_registration_required"
    assert controller["control_status"] == "available"
    assert [channel["id"] for channel in controller["channels"]] == [channel.id for channel in controls.channels]
    assert all(channel["operations"] == ["batch"] for channel in controller["channels"])
    assert controller["authorities"][0]["command_owner"] == "caller"

    response = runner.run(
        MissionCompositionRunRequest(
            request_id="cadac-falcon6-vertical-batch",
            provider_id=CADAC_PROVIDER_ID,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=20),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json")
    assert response.result.provider_version == provider.metadata.version
    assert [(item.model_id, item.fidelity, item.realization_id) for item in response.result.objects] == [
        (FALCON6_MODEL_ID, FALCON6_FIDELITY_ID, FALCON6_REALIZATION_ID),
    ]
    samples = [sample.values for sample in response.result.objects[0].samples]
    final = samples[-1]
    assert tuple(final["requested_surfaces_deg"]) == pytest.approx((30.0, 5.0, -25.0))
    assert any(abs(float(value)) > 0.0 for value in final["achieved_surfaces_deg"])
    assert max(abs(float(value)) for value in final["achieved_surfaces_deg"]) <= 20.1
    assert any(any(bool(flag) for flag in sample["surface_position_limited"]) for sample in samples)
    assert any(any(bool(flag) for flag in sample["surface_rate_limited"]) for sample in samples)
    assert len(final["aero_surfaces_deg"]) == 3
    assert len(final["quaternion_wxyz"]) == 4
    assert len(final["body_rates_rad_s"]) == 3

    integration = provider.get_model_integration_contract(FALCON6_MODEL_ID)
    assert integration.step.status == "available"
    assert integration.step.state_semantics == "core_batch_replay"
    assert integration.step.action_semantics == "read_only_replay"
    assert integration.sensor_integration.status == "not_applicable"
    assert integration.sensor_integration.sensor_bus_status == "not_applicable"
    assert integration.controller_analysis.ownership == "external_at_source_boundary"
    assert integration.controller_analysis.time_domain_analysis == "available"
    assert integration.controller_analysis.controller_comparison == "available"
    assert integration.controller_analysis.local_linear_stability == "blocked"
    assert integration.controller_analysis.command_output_channel_ids == ("requested_surfaces_deg",)
    assert integration.controller_analysis.response_output_channel_ids == ("achieved_surfaces_deg",)
    assert integration.environment.atmosphere_owner == "cadac_compatibility_runtime"
    assert integration.environment.gravity_owner == "cadac_compatibility_runtime"

    with pytest.raises(ValueError, match="no installed native persistent session binding"):
        provider.open_session(
            MissionCompositionOpenSessionRequest(
                session_id="cadac-falcon6-must-not-fabricate-session",
                provider_id=CADAC_PROVIDER_ID,
                provider_version=provider.metadata.version,
                prepared_configuration=prepared,
                integration_step_s=0.001,
            )
        )

    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id="cadac-falcon6-core-replay-session",
            provider_id=CADAC_PROVIDER_ID,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.1,
        )
    )
    assert descriptor.action_schema == ()
    assert descriptor.state_owner == "core_batch_replay_session"
    assert descriptor.timestep_semantics == "caller_duration_advanced_to_next_replay_sample"
    replay_step = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            duration_s=0.1,
        )
    )
    assert replay_step.time_end_s > replay_step.time_start_s
    assert len(replay_step.observation.standard_ecef.ecef_from_body_wxyz) == 4
    ####
