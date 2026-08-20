"""Focused vertical proof for the source-bound CADAC MAGSIX plug-in slice."""

from __future__ import annotations

from pathlib import Path
from typing import cast

import pytest
from taoryx.families.cadac.magsix_mission_composition import (
    MAGSIX_ATTITUDE_FIDELITY_ID,
    MAGSIX_ATTITUDE_PHASE_ID,
    MAGSIX_MISSION_ID,
    MAGSIX_MODEL_ID,
    MAGSIX_MODEL_VERSION,
    MAGSIX_TRAJECTORY_FIDELITY_ID,
    MAGSIX_TRAJECTORY_REALIZATION_ID,
)
from taoryx.families.cadac.mission_composition_plugin import (
    CadacMissionCompositionProvider,
    CadacSourceCaseBindings,
    build_default_cadac_configuration,
)
from test_magsix import _case

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


def _bound_magsix_provider(tmp_path: Path) -> CadacMissionCompositionProvider:
    """Bind only the synthetic MAGSIX case into one selected catalog scope."""

    return CadacSourceCaseBindings(
        magsix_case_path=_case(tmp_path),
    ).build_provider(selected_model_ids=(MAGSIX_MODEL_ID,))
    ####


def _short_magsix_configuration(provider: CadacMissionCompositionProvider) -> TrajectoryConfigurationInstance:
    """Build a brief trajectory-only run without changing source-case defaults."""

    configuration = build_default_cadac_configuration(provider, MAGSIX_MODEL_ID)
    assert isinstance(configuration.root, ConfigurationGroupValue)
    root = configuration.root.model_copy(
        update={
            "values": {
                **configuration.root.values,
                "runtime": ConfigurationGroupValue(
                    values={
                        "end_time_dnt": ConfigurationParameterValue(value=0.05),
                        "sample_step_dnt": ConfigurationParameterValue(value=0.01),
                    }
                ),
            }
        }
    )
    return configuration.model_copy(update={"root": root})
    ####


def test_bound_magsix_preserves_one_model_scope_and_fixed_source_control(
    tmp_path: Path,
) -> None:
    """MAGSIX publishes one batch-only force-model boundary with no fake action."""

    provider = _bound_magsix_provider(tmp_path)
    assert provider.metadata.model_count == 1
    assert tuple(item.id for item in provider.list_models()) == (MAGSIX_MODEL_ID,)
    with pytest.raises(KeyError, match="unknown CADAC trajectory plug-in"):
        provider.get_model_schema("cadac.cruise5.cruise_vehicle")

    model = provider.list_models()[0]
    controls = model.realizations[0].controls
    integration = provider.get_model_integration_contract(MAGSIX_MODEL_ID)
    audit = audit_provider_advertisement(provider)

    assert model.version == MAGSIX_MODEL_VERSION
    assert model.common_runner_operations == ("batch", "step")
    assert tuple(item.id for item in model.fidelities) == (
        MAGSIX_TRAJECTORY_FIDELITY_ID,
        MAGSIX_ATTITUDE_FIDELITY_ID,
    )
    assert controls.status == "internally_generated"
    assert controls.default_authority_id == "source_program_control"
    assert controls.channels == ()
    assert audit.status == "pass", audit.model_dump(mode="json")
    assert integration.step.status == "available"
    assert integration.step.state_semantics == "core_batch_replay"
    assert integration.controller_analysis.ownership == "fixed_source_program"
    assert integration.controller_analysis.time_domain_analysis == "not_applicable"
    assert integration.controller_analysis.local_linear_stability == "blocked"
    assert integration.sensor_integration.sensor_bus_status == "not_applicable"

    blocked_configuration = build_default_cadac_configuration(
        provider,
        MAGSIX_MODEL_ID,
        phase_id=MAGSIX_ATTITUDE_PHASE_ID,
    )
    blocked_prepared = provider.validate_configuration(blocked_configuration)
    assert blocked_prepared.configuration.fidelity == MAGSIX_ATTITUDE_FIDELITY_ID
    with pytest.raises(ValueError, match="validation-only"):
        provider.execute_batch(
            MissionCompositionRunRequest(
                request_id="cadac-magsix-attitude-blocked",
                provider_id=provider.metadata.id,
                provider_version=provider.metadata.version,
                prepared_configuration=blocked_prepared,
            )
        )
    ####


def test_bound_magsix_runs_through_catalog_batch_api_with_labeled_outputs(
    tmp_path: Path,
) -> None:
    """The selected catalog wrapper keeps its revision and trajectory truth."""

    provider = _bound_magsix_provider(tmp_path)
    prepared = provider.validate_configuration(_short_magsix_configuration(provider))
    registry = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(registry)

    plan = build_model_authoring_plan(
        ConfigurableTrajectoryProviderRegistry((provider,)),
        ControllerTuningCampaignRegistry(),
        provider.metadata.id,
        MAGSIX_MODEL_ID,
        fidelity=MAGSIX_TRAJECTORY_FIDELITY_ID,
        realization_id=MAGSIX_TRAJECTORY_REALIZATION_ID,
        mission_template_id=MAGSIX_MISSION_ID,
    )
    controller = cast(dict[str, object], plan["controller_automation"])
    assert plan["status"] == "ready_to_author"
    assert controller["status"] == "provider_managed"
    assert controller["control_status"] == "internally_generated"
    assert controller["channels"] == []
    assert controller["default_authority_id"] == "source_program_control"

    response = registry.run(
        MissionCompositionRunRequest(
            request_id="cadac-magsix-vertical-batch",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=6),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    assert response.result.status == "completed"
    assert response.result.provider_version == provider.metadata.version
    assert [item.model_id for item in response.result.objects] == [MAGSIX_MODEL_ID]
    rotor = response.result.objects[0]
    assert rotor.fidelity == MAGSIX_TRAJECTORY_FIDELITY_ID
    assert len(rotor.samples) > 1
    assert {
        "position_ned_m",
        "velocity_ned_mps",
        "altitude_m",
        "speed_mps",
        "spin_rpm",
        "source_time_dnt",
    } <= {channel.id for channel in rotor.channels}
    assert rotor.samples[-1].values["source_time_dnt"] > rotor.samples[0].values["source_time_dnt"]
    assert isinstance(rotor.samples[-1].values["spin_rpm"], float)
    ####
