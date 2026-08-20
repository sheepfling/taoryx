"""Focused vertical proof for the source-bound CADAC CRUISE5 plug-in slice."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import cast

import pytest
from taoryx.families.cadac.cruise5_mission_composition import (
    CRUISE5_FIDELITY_ID,
    CRUISE5_MISSION_ID,
    CRUISE5_MODEL_ID,
    CRUISE5_MODEL_VERSION,
    CRUISE5_REALIZATION_ID,
)
from taoryx.families.cadac.mission_composition_plugin import (
    CadacMissionCompositionProvider,
    CadacSourceCaseBindings,
    build_default_cadac_configuration,
)
from taoryx_cadac.plugin import PLUGIN
from test_cruise5 import _case

from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistry
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import DeferredMissionCompositionProvider, PluginCatalog, PluginEntryPoint, discover_plugins
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

CADAC_PACKAGE_ID = "taoryx.cadac"
CADAC_PROVIDER_ID = "cadac"


@dataclass(frozen=True)
class _EntryPoint:
    """Minimal deterministic entry-point double for isolated discovery."""

    name: str
    value: str
    target: object

    def load(self) -> object:
        """Return the package-owned plug-in definition."""

        return self.target
        ####

    ####


def _discovered_catalog() -> PluginCatalog:
    """Load exactly the CADAC entry point without built-in or peer fallbacks."""

    return discover_plugins(
        include_builtin=False,
        entry_points=(
            cast(
                PluginEntryPoint,
                _EntryPoint(
                    name=CADAC_PACKAGE_ID,
                    value="taoryx_cadac.plugin:PLUGIN",
                    target=PLUGIN,
                ),
            ),
        ),
    )
    ####


def _bound_cruise5_provider(tmp_path: Path) -> CadacMissionCompositionProvider:
    """Bind only the synthetic CRUISE5 case into one selected catalog scope."""

    return CadacSourceCaseBindings(
        cruise5_case_path=_case(tmp_path),
    ).build_provider(selected_model_ids=(CRUISE5_MODEL_ID,))
    ####


def _short_cruise5_configuration(provider: CadacMissionCompositionProvider) -> TrajectoryConfigurationInstance:
    """Build a brief source-model run without changing source-case defaults."""

    configuration = build_default_cadac_configuration(provider, CRUISE5_MODEL_ID)
    assert isinstance(configuration.root, ConfigurationGroupValue)
    root = configuration.root.model_copy(
        update={
            "values": {
                **configuration.root.values,
                "runtime": ConfigurationGroupValue(
                    values={
                        "end_time_s": ConfigurationParameterValue(value=1.0, unit="s"),
                        "sample_step_s": ConfigurationParameterValue(value=0.5, unit="s"),
                    }
                ),
            }
        }
    )
    return configuration.model_copy(update={"root": root})
    ####


def test_cadac_catalog_discovery_is_source_independent_and_versioned() -> None:
    """The package metadata exposes its versioned provider identity without schema construction."""

    catalog = _discovered_catalog()
    deferred_provider = catalog.build_mission_composition_provider_registry().provider(CADAC_PROVIDER_ID)
    revision = catalog.plugin_revision(CADAC_PACKAGE_ID)

    assert tuple(item.id for item in catalog.plugins) == (CADAC_PACKAGE_ID,)
    assert isinstance(deferred_provider, DeferredMissionCompositionProvider)
    assert deferred_provider.__taoryx_provider_id__ == CADAC_PROVIDER_ID
    assert revision.package == "taoryx-cadac"
    assert revision.version == "0.1.0a0"
    assert revision.api_version == "1"
    assert len(revision.fingerprint) == 64
    ####


def test_bound_cruise5_preserves_one_model_scope_and_source_managed_controls(
    tmp_path: Path,
) -> None:
    """One source-bound Cruise5 case retains its batch-only control boundary."""

    provider = _bound_cruise5_provider(tmp_path)
    assert provider.metadata.model_count == 1
    assert tuple(item.id for item in provider.list_models()) == (CRUISE5_MODEL_ID,)
    with pytest.raises(KeyError, match="unknown CADAC trajectory plug-in"):
        provider.get_model_schema("cadac.aim5.missile")

    model = provider.list_models()[0]
    audit = audit_provider_advertisement(provider)
    controls = model.realizations[0].controls
    integration = provider.get_model_integration_contract(CRUISE5_MODEL_ID)
    schema = provider.get_model_schema(CRUISE5_MODEL_ID)
    output = provider.get_model_output_schema(CRUISE5_MODEL_ID)

    assert model.version == CRUISE5_MODEL_VERSION
    assert schema.model_version == model.version
    assert output.model_version == model.version
    assert len(model.metadata_fingerprint) == 64
    assert len(model.configuration_schema_fingerprint) == 64
    assert len(model.output_schema_fingerprint) == 64
    assert model.common_runner_operations == ("batch", "step")
    assert model.fidelities[0].id == CRUISE5_FIDELITY_ID
    assert controls.status == "internally_generated"
    assert controls.default_authority_id == "source_program_control"
    assert controls.channels == ()
    assert audit.status == "pass", audit.model_dump(mode="json")
    assert integration.step.status == "available"
    assert integration.step.state_semantics == "core_batch_replay"
    assert integration.controller_analysis.ownership == "source_owned"
    assert integration.controller_analysis.command_output_channel_ids == (
        "bank_command_deg",
        "load_command_g",
        "lateral_command_g",
    )
    assert integration.controller_analysis.response_output_channel_ids == ()
    assert integration.controller_analysis.time_domain_analysis == "blocked"
    assert integration.controller_analysis.local_linear_stability == "blocked"
    assert integration.sensor_integration.sensor_bus_status == "not_applicable"

    with pytest.raises(ValueError, match="omits explicit source bindings"):
        CadacSourceCaseBindings(
            cruise5_case_path=_case(tmp_path),
        ).build_provider(selected_model_ids=("cadac.aim5.missile",))
    ####


def test_selected_scope_rejects_unselected_eager_source_before_loading_it(tmp_path: Path) -> None:
    """A foreign eager deck cannot add parsing or Pydantic work to a focused scope."""

    with pytest.raises(ValueError, match="omits explicit source bindings"):
        CadacSourceCaseBindings(
            rocket6g_case_path=tmp_path / "unselected-rocket6g.asc",
        ).build_provider(selected_model_ids=(CRUISE5_MODEL_ID,))
    ####


def test_bound_cruise5_runs_through_catalog_batch_api_with_labeled_outputs(
    tmp_path: Path,
) -> None:
    """The selected catalog wrapper keeps its revision and complete readback."""

    provider = _bound_cruise5_provider(tmp_path)
    model = provider.list_models()[0]
    configuration = _short_cruise5_configuration(provider)
    prepared = provider.validate_configuration(configuration)
    registry = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(registry)

    plan = build_model_authoring_plan(
        ConfigurableTrajectoryProviderRegistry((provider,)),
        ControllerTuningCampaignRegistry(),
        CADAC_PROVIDER_ID,
        CRUISE5_MODEL_ID,
        fidelity=CRUISE5_FIDELITY_ID,
        realization_id=CRUISE5_REALIZATION_ID,
        mission_template_id=CRUISE5_MISSION_ID,
    )
    controller = cast(dict[str, object], plan["controller_automation"])
    selection = cast(dict[str, object], plan["selection"])
    assert plan["status"] == "ready_to_author"
    assert selection["model_version"] == model.version
    assert selection["model_metadata_fingerprint"] == model.metadata_fingerprint
    assert controller["status"] == "provider_managed"
    assert controller["control_status"] == "internally_generated"
    assert controller["channels"] == []
    assert controller["default_authority_id"] == "source_program_control"

    response = registry.run(
        MissionCompositionRunRequest(
            request_id="cadac-cruise5-vertical-batch",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=3),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    assert response.result.status == "completed"
    assert response.result.provider_version == provider.metadata.version
    assert [item.model_id for item in response.result.objects] == [CRUISE5_MODEL_ID]
    vehicle = response.result.objects[0]
    assert vehicle.fidelity == CRUISE5_FIDELITY_ID
    assert [sample.time_s for sample in vehicle.samples] == pytest.approx([0.0, 0.5, 1.0])
    assert {
        "longitude_deg",
        "latitude_deg",
        "altitude_m",
        "velocity_geographic_mps",
        "alpha_deg",
        "bank_deg",
        "bank_command_deg",
        "waypoint_ground_range_m",
    } <= {channel.id for channel in vehicle.channels}
    assert vehicle.samples[-1].values["waypoint_ground_range_m"] > 0.0
    assert isinstance(vehicle.samples[-1].values["bank_command_deg"], float)
    assert isinstance(vehicle.samples[-1].values["bank_deg"], float)
    ####
