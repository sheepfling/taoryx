"""Focused vertical proof for the source-bound CADAC GHAME3 plug-in slice."""

from __future__ import annotations

from pathlib import Path
from typing import cast

import pytest
from taoryx.families.cadac.ghame3_mission_composition import (
    GHAME3_FIDELITY_ID,
    GHAME3_MISSION_ID,
    GHAME3_MODEL_ID,
    GHAME3_MODEL_VERSION,
    GHAME3_REALIZATION_ID,
)
from taoryx.families.cadac.mission_composition_plugin import (
    CadacMissionCompositionProvider,
    CadacSourceCaseBindings,
    build_default_cadac_configuration,
)
from test_ghame3 import _case

from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistry
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.trajectory.configuration_contract import (
    ConfigurableTrajectoryProviderRegistry,
    ConfigurationGroupValue,
    ConfigurationParameterValue,
    TrajectoryConfigurationInstance,
)
from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
    audit_provider_advertisement,
)


def _bound_ghame3_provider(tmp_path: Path) -> CadacMissionCompositionProvider:
    """Bind only the synthetic GHAME3 case into one selected catalog scope."""

    return CadacSourceCaseBindings(
        ghame3_case_path=_case(tmp_path),
    ).build_provider(selected_model_ids=(GHAME3_MODEL_ID,))
    ####


def _short_ghame3_configuration(provider: CadacMissionCompositionProvider) -> TrajectoryConfigurationInstance:
    """Build a brief source-model run without changing source-case defaults."""

    configuration = build_default_cadac_configuration(provider, GHAME3_MODEL_ID)
    assert isinstance(configuration.root, ConfigurationGroupValue)
    root = configuration.root.model_copy(
        update={
            "values": {
                **configuration.root.values,
                "runtime": ConfigurationGroupValue(
                    values={
                        "end_time_s": ConfigurationParameterValue(value=0.05, unit="s"),
                        "sample_step_s": ConfigurationParameterValue(value=0.01, unit="s"),
                    }
                ),
            }
        }
    )
    return configuration.model_copy(update={"root": root})
    ####


def test_bound_ghame3_preserves_one_model_scope_and_fixed_source_control(
    tmp_path: Path,
) -> None:
    """GHAME3 exposes one batch-only 3-DoF source-program boundary."""

    provider = _bound_ghame3_provider(tmp_path)
    assert provider.metadata.model_count == 1
    assert tuple(item.id for item in provider.list_models()) == (GHAME3_MODEL_ID,)
    with pytest.raises(KeyError, match="unknown CADAC trajectory plug-in"):
        provider.get_model_schema("cadac.magsix.vehicle")

    model = provider.list_models()[0]
    controls = model.realizations[0].controls
    integration = provider.get_model_integration_contract(GHAME3_MODEL_ID)
    audit = audit_provider_advertisement(provider)

    assert model.version == GHAME3_MODEL_VERSION
    assert model.common_runner_operations == ("batch",)
    assert tuple(item.id for item in model.fidelities) == (GHAME3_FIDELITY_ID,)
    assert model.fidelities[0].dynamics_fidelity == "point_mass_3dof"
    assert controls.status == "internally_generated"
    assert controls.default_authority_id == "source_program_control"
    assert controls.channels == ()
    assert audit.status == "pass", audit.model_dump(mode="json")
    assert integration.step.status == "blocked"
    assert integration.step.state_semantics == "batch_only"
    assert integration.controller_analysis.ownership == "fixed_source_program"
    assert integration.controller_analysis.command_output_channel_ids == ()
    assert integration.controller_analysis.response_output_channel_ids == ()
    assert integration.controller_analysis.time_domain_analysis == "not_applicable"
    assert integration.controller_analysis.local_linear_stability == "blocked"
    assert integration.sensor_integration.sensor_bus_status == "not_applicable"

    with pytest.raises(ValueError, match="omits explicit source bindings"):
        CadacSourceCaseBindings(
            ghame3_case_path=_case(tmp_path / "omitted-scope"),
        ).build_provider(selected_model_ids=("cadac.magsix.vehicle",))
    ####


def test_bound_ghame3_runs_through_catalog_batch_api_with_labeled_outputs(
    tmp_path: Path,
) -> None:
    """The selected catalog wrapper retains source events and point-mass truth."""

    provider = _bound_ghame3_provider(tmp_path)
    prepared = provider.validate_configuration(_short_ghame3_configuration(provider))
    registry = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(registry)

    plan = build_model_authoring_plan(
        ConfigurableTrajectoryProviderRegistry((provider,)),
        ControllerTuningCampaignRegistry(),
        provider.metadata.id,
        GHAME3_MODEL_ID,
        fidelity=GHAME3_FIDELITY_ID,
        realization_id=GHAME3_REALIZATION_ID,
        mission_template_id=GHAME3_MISSION_ID,
    )
    controller = cast(dict[str, object], plan["controller_automation"])
    assert plan["status"] == "ready_to_author"
    assert controller["status"] == "provider_managed"
    assert controller["control_status"] == "internally_generated"
    assert controller["channels"] == []
    assert controller["default_authority_id"] == "source_program_control"

    response = registry.run(
        MissionCompositionRunRequest(
            request_id="cadac-ghame3-vertical-batch",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=6),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    assert response.result.status == "completed"
    assert response.result.provider_version == provider.metadata.version
    assert [item.model_id for item in response.result.objects] == [GHAME3_MODEL_ID]
    vehicle = response.result.objects[0]
    assert vehicle.fidelity == GHAME3_FIDELITY_ID
    assert len(vehicle.samples) > 1
    assert {
        "longitude_deg",
        "latitude_deg",
        "altitude_m",
        "velocity_geographic_mps",
        "mach",
        "alpha_deg",
        "bank_deg",
        "throttle",
        "thrust_n",
        "fuel_mass_kg",
    } <= {channel.id for channel in vehicle.channels}
    assert len(response.result.events) == 1
    assert response.result.events[0].kind == "cadac-source-event"
    assert response.result.events[0].time_s == pytest.approx(0.03)
    assert vehicle.samples[-1].values["altitude_m"] > 0.0
    assert isinstance(vehicle.samples[-1].values["thrust_n"], float)
    ####
