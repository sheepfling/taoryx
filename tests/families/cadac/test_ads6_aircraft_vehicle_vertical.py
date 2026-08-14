"""Focused vertical proof for the standalone CADAC ADS6 AIRCRAFT3 plug-in slice."""

from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ads6_aircraft_mission_composition import (
    ADS6_AIRCRAFT_FIDELITY_ID,
    ADS6_AIRCRAFT_MISSION_ID,
    ADS6_AIRCRAFT_MODEL_ID,
    ADS6_AIRCRAFT_REALIZATION_ID,
)
from taoryx.families.cadac.mission_composition_plugin import (
    CadacMissionCompositionProvider,
    CadacSourceCaseBindings,
    build_default_cadac_configuration,
)
from test_ads6_aircraft import _write_ads6_aircraft_case

from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistry
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.trajectory.configuration_contract import ConfigurableTrajectoryProviderRegistry
from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
    audit_provider_advertisement,
)
from taoryx.trajectory.mission_composition import MissionCompositionOpenSessionRequest

CADAC_PROVIDER_ID = "cadac"


def _provider(tmp_path: Path) -> CadacMissionCompositionProvider:
    """Bind one g-turn AIRCRAFT3 source case without catalog fallback."""

    return CadacSourceCaseBindings(
        ads6_aircraft_case_path=_write_ads6_aircraft_case(
            tmp_path,
            guidance_option=1,
            turn_load_g=1.5,
            maneuver_start_s=0.02,
            maneuver_stop_s=0.05,
            bank_time_constant_s=0.02,
            load_factor_time_constant_s=0.02,
            end_time_s=0.08,
        ),
    ).build_provider(selected_model_ids=(ADS6_AIRCRAFT_MODEL_ID,))
    ####


def test_bound_ads6_aircraft_preserves_source_control_and_point_mass_truth_boundary(tmp_path: Path) -> None:
    """The exact target exposes source-program telemetry without pseudo-6DoF promotion."""

    provider = _provider(tmp_path)
    model = next(item for item in provider.list_models() if item.id == ADS6_AIRCRAFT_MODEL_ID)
    configuration = build_default_cadac_configuration(provider, ADS6_AIRCRAFT_MODEL_ID)
    prepared = provider.validate_configuration(configuration)
    runner = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(runner)
    audit = audit_provider_advertisement(provider, runner)

    assert tuple(item.id for item in provider.list_models()) == (ADS6_AIRCRAFT_MODEL_ID,)
    with pytest.raises(KeyError, match="unknown CADAC trajectory plug-in"):
        provider.get_model_schema("cadac.aim5.missile")
    assert runner.registrations() == ((CADAC_PROVIDER_ID, ADS6_AIRCRAFT_MODEL_ID),)
    assert model.common_runner_operations == ("batch",)
    assert model.fidelities[0].id == ADS6_AIRCRAFT_FIDELITY_ID
    assert model.fidelities[0].input_realization == "provider_defined"
    assert model.fidelities[0].control_realization == "force_model"
    assert model.realizations[0].id == ADS6_AIRCRAFT_REALIZATION_ID
    assert model.realizations[0].controls.status == "internally_generated"
    assert model.realizations[0].controls.channels == ()
    authority = model.realizations[0].controls.authorities[0]
    assert authority.id == "source_program_control"
    assert authority.command_owner == "source_program"
    assert authority.operations == ("batch",)
    assert audit.status == "pass", audit.model_dump(mode="json")

    resolved = prepared.resolved
    assert resolved["maneuver"]["guidance_option"] == 1
    assert resolved["maneuver"]["turn_load_g"] == pytest.approx(1.5)
    outputs = {channel.id: channel for channel in model.output_schema.channels}
    for channel_id in (
        "commanded_acceleration_ned_mps2",
        "commanded_bank_deg",
        "bank_state_deg",
        "bank_deg",
        "commanded_load_factor_g",
        "normal_load_factor_g",
        "specific_force_body_mps2",
    ):
        assert outputs[channel_id].operations == ("batch",)
    ####
    assert outputs["commanded_acceleration_ned_mps2"].shape == (3,)
    assert outputs["specific_force_body_mps2"].shape == (3,)

    plan = build_model_authoring_plan(
        ConfigurableTrajectoryProviderRegistry((provider,)),
        ControllerTuningCampaignRegistry(),
        CADAC_PROVIDER_ID,
        ADS6_AIRCRAFT_MODEL_ID,
        fidelity=ADS6_AIRCRAFT_FIDELITY_ID,
        realization_id=ADS6_AIRCRAFT_REALIZATION_ID,
        mission_template_id=ADS6_AIRCRAFT_MISSION_ID,
    )
    controller = plan["controller_automation"]
    assert plan["status"] == "ready_to_author"
    assert isinstance(controller, dict)
    assert controller["status"] == "provider_managed"
    assert controller["control_status"] == "internally_generated"
    assert controller["channels"] == []
    assert controller["authorities"][0]["operations"] == ["batch"]
    assert controller["control_scheme_support"][0]["scheme_id"] == "provider.program"

    response = runner.run(
        MissionCompositionRunRequest(
            request_id="cadac-ads6-aircraft-vertical-batch",
            provider_id=CADAC_PROVIDER_ID,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=20, include_events=True),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json")
    assert response.result.provider_version == provider.metadata.version
    assert [(item.model_id, item.fidelity, item.realization_id) for item in response.result.objects] == [
        (ADS6_AIRCRAFT_MODEL_ID, ADS6_AIRCRAFT_FIDELITY_ID, ADS6_AIRCRAFT_REALIZATION_ID),
    ]
    samples = [sample.values for sample in response.result.objects[0].samples]
    active = [sample for sample in samples if sample["maneuver_active"]]
    assert active
    assert {sample["mode"] for sample in active} == {"g_turn"}
    assert max(float(sample["commanded_bank_deg"]) for sample in active) > 0.0
    assert max(float(sample["bank_deg"]) for sample in active) > 0.0
    assert max(float(sample["commanded_load_factor_g"]) for sample in active) > 1.0
    assert max(float(sample["normal_load_factor_g"]) for sample in active) > 1.0
    assert all(len(sample["specific_force_body_mps2"]) == 3 for sample in active)
    assert "quaternion_wxyz" not in active[0]
    assert "body_rates_rad_s" not in active[0]
    assert [(event.kind, event.data["active"]) for event in response.result.events] == [
        ("cadac-aircraft-maneuver-window", True),
        ("cadac-aircraft-maneuver-window", False),
    ]

    integration = provider.get_model_integration_contract(ADS6_AIRCRAFT_MODEL_ID)
    assert integration.step.status == "blocked"
    assert integration.step.state_semantics == "batch_only"
    assert integration.step.action_semantics == "provider_internal"
    assert integration.sensor_integration.status == "not_applicable"
    assert integration.sensor_integration.sensor_bus_status == "not_applicable"
    assert integration.controller_analysis.ownership == "source_owned"
    assert integration.controller_analysis.time_domain_analysis == "available"
    assert integration.controller_analysis.controller_comparison == "available"
    assert integration.controller_analysis.local_linear_stability == "blocked"
    assert set(integration.controller_analysis.command_output_channel_ids) >= {
        "commanded_acceleration_ned_mps2",
        "commanded_bank_deg",
        "commanded_load_factor_g",
    }
    assert set(integration.controller_analysis.response_output_channel_ids) >= {
        "bank_state_deg",
        "normal_load_factor_g",
    }
    assert integration.environment.atmosphere_owner == "cadac_compatibility_runtime"
    assert integration.environment.gravity_owner == "cadac_compatibility_runtime"
    assert integration.environment.host_environment_status == "blocked"

    with pytest.raises(ValueError, match="no installed persistent session binding"):
        provider.open_session(
            MissionCompositionOpenSessionRequest(
                session_id="cadac-ads6-aircraft-must-not-fabricate-session",
                provider_id=CADAC_PROVIDER_ID,
                provider_version=provider.metadata.version,
                prepared_configuration=prepared,
                integration_step_s=0.01,
            )
        )
    ####
