"""Focused vertical proof for the source-bound CADAC AIM5 plug-in slice."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import cast

import pytest
from taoryx.families.cadac.mission_composition_plugin import (
    CadacSourceCaseBindings,
    build_default_cadac_configuration,
)
from taoryx_cadac.plugin import PLUGIN
from test_aim5 import AERO, INPUT, PROP

from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistry
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import DeferredMissionCompositionProvider, PluginCatalog, PluginEntryPoint, discover_plugins
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
AIM5_MODEL_ID = "cadac.aim5.missile"
AIM5_REALIZATION_ID = "cadac-source-compatibility"
AIM5_MISSION_ID = "air_intercept"
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


def _write_synthetic_aim5_case(tmp_path: Path) -> Path:
    """Materialize non-upstream AIM5 inputs owned only by this test boundary."""

    case_path = tmp_path / "input.asc"
    case_path.write_text(INPUT, encoding="utf-8")
    (tmp_path / "aero.asc").write_text(AERO, encoding="utf-8")
    (tmp_path / "prop.asc").write_text(PROP, encoding="utf-8")
    return case_path
    ####


def _discovered_catalog() -> PluginCatalog:
    """Load exactly the CADAC entry point without built-in or peer fallbacks."""

    return discover_plugins(
        include_builtin=False,
        entry_points=(
            cast(
                PluginEntryPoint,
                _EntryPoint(
                    name="taoryx.cadac",
                    value="taoryx_cadac.plugin:PLUGIN",
                    target=PLUGIN,
                ),
            ),
        ),
    )
    ####


def test_cadac_catalog_discovery_is_source_independent_and_revisioned() -> None:
    """The package advertises its versioned provider identity without catalog construction."""

    catalog = _discovered_catalog()
    provider = catalog.build_mission_composition_provider_registry().provider(CADAC_PROVIDER_ID)
    revision = catalog.plugin_revision("taoryx.cadac")

    assert tuple(item.id for item in catalog.plugins) == ("taoryx.cadac",)
    assert isinstance(provider, DeferredMissionCompositionProvider)
    assert provider.__taoryx_provider_id__ == CADAC_PROVIDER_ID
    assert revision.package == "taoryx-cadac"
    assert revision.version == "0.1.0a0"
    assert len(revision.fingerprint) == 64
    ####


def test_bound_aim5_preserves_catalog_identity_controls_and_sensor_api(tmp_path: Path) -> None:
    """One caller-owned AIM5 case reaches the normal API with no actor fallback."""

    provider = CadacSourceCaseBindings(
        aim5_case_path=_write_synthetic_aim5_case(tmp_path),
    ).build_provider(selected_model_ids=(AIM5_MODEL_ID,))
    model = next(item for item in provider.list_models() if item.id == AIM5_MODEL_ID)
    configuration = build_default_cadac_configuration(provider, AIM5_MODEL_ID)
    prepared = provider.validate_configuration(configuration)
    runner = MissionCompositionRunnerRegistry()
    provider.register_runnable_models(runner)
    audit = audit_provider_advertisement(provider, runner)
    schema = provider.get_model_schema(AIM5_MODEL_ID)
    output = provider.get_model_output_schema(AIM5_MODEL_ID)

    assert tuple(item.id for item in provider.list_models()) == (AIM5_MODEL_ID,)
    with pytest.raises(KeyError, match="unknown CADAC trajectory plug-in"):
        provider.get_model_schema("cadac.cruise5.cruise_vehicle")
    assert runner.registrations() == ((CADAC_PROVIDER_ID, AIM5_MODEL_ID),)
    assert model.version
    assert schema.model_version == model.version
    assert output.model_version == model.version
    assert len(model.metadata_fingerprint) == 64
    assert len(model.configuration_schema_fingerprint) == 64
    assert len(model.output_schema_fingerprint) == 64
    assert model.common_runner_operations == ("batch", "step")
    assert model.realizations[0].controls.status == "internally_generated"
    assert model.realizations[0].controls.default_authority_id == "source_program_control"
    assert audit.status == "pass", audit.model_dump(mode="json")

    plan = build_model_authoring_plan(
        ConfigurableTrajectoryProviderRegistry((provider,)),
        ControllerTuningCampaignRegistry(),
        CADAC_PROVIDER_ID,
        AIM5_MODEL_ID,
        fidelity="pseudo_6dof",
        realization_id=AIM5_REALIZATION_ID,
        mission_template_id=AIM5_MISSION_ID,
    )
    controller = plan["controller_automation"]
    selection = cast(dict[str, object], plan["selection"])
    assert plan["status"] == "ready_to_author"
    assert isinstance(controller, dict)
    assert selection["model_version"] == model.version
    assert selection["model_metadata_fingerprint"] == model.metadata_fingerprint
    assert controller["status"] == "provider_managed"
    assert controller["control_status"] == "internally_generated"
    assert controller["channels"] == []
    assert controller["default_authority_id"] == "source_program_control"

    response = runner.run(
        MissionCompositionRunRequest(
            request_id="cadac-aim5-vertical-batch",
            provider_id=CADAC_PROVIDER_ID,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="core", maximum_samples_per_object=3),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json")
    assert response.result.provider_version == provider.metadata.version
    assert [item.model_id for item in response.result.objects] == [AIM5_MODEL_ID, "cadac.aim5.target"]
    assert all(item.samples for item in response.result.objects)

    descriptor = provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="cadac-aim5-vertical-session",
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
    integration = provider.get_model_integration_contract(AIM5_MODEL_ID)

    assert descriptor.action_schema == ()
    assert descriptor.command_source_id == "cadac_source_program"
    assert step.observation.values["native_relative_state_track"]["valid"]
    assert len(provider.session_sensor_packets(descriptor.session_id)) == 2
    assert integration.step.state_semantics == "persistent_native_state"
    assert integration.sensor_integration.sensor_bus_status == "available"
    assert integration.controller_analysis.ownership == "source_owned"
    assert integration.controller_analysis.local_linear_stability == "blocked"
    assert integration.environment.atmosphere_owner == "cadac_compatibility_runtime"
    assert integration.environment.gravity_owner == "cadac_compatibility_runtime"
    assert integration.environment.host_environment_status == "blocked"
    provider.close_session(descriptor.session_id)
    ####
