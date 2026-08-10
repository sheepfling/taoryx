"""Focused Composition proof for the Simple Aero fixed-L/D workflow."""

from __future__ import annotations

import math
from typing import Any, cast

import pytest
from taoryx.trajectory import build_simple_aero_template_configuration
from taoryx.trajectory.native_mission_composition import build_registry_mission_composition_runner
from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider
from taoryx.trajectory.session_contract import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionManager,
)
from taoryx.trajectory.simple_aero_mission_composition import (
    build_simple_aero_example_configuration,
    build_simple_aero_prepared_configuration,
)

from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.trajectory.execution_contract import (
    MissionCompositionExecutionError,
    MissionCompositionOutputSelection,
    MissionCompositionRunRequest,
)

PROVIDER_ID = "taoryx.registry.mission-composition"
MODEL_ID = "simple_aero"
MISSION_ID = "fixed_ld_baseline"
FIDELITY = "point_mass_3dof"
REALIZATION_ID = "fixed_ld_point_mass"


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover the installed distributions once for the focused workflow slice."""

    return discover_plugins(include_external=False)
    ####


def _prepared(provider: RegistryMissionCompositionProvider):
    """Build the checked fixed-L/D witness through the published schema."""

    return provider.validate_configuration(
        build_simple_aero_example_configuration(provider.get_model_schema(MODEL_ID))
    )
    ####


def _prepared_template(provider: RegistryMissionCompositionProvider, mission_template_id: str):
    """Build one documented source-shaped recipe through the public helper."""

    return provider.validate_configuration(
        build_simple_aero_template_configuration(
            mission_template_id,
            schema=provider.get_model_schema(MODEL_ID),
        )
    )
    ####


def test_simple_aero_advertisement_builds_a_complete_batch_authoring_plan(
    plugins: PluginCatalog,
) -> None:
    """The workflow publishes generated controls without inventing a plant or tuner."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity=FIDELITY,
        realization_id=REALIZATION_ID,
        mission_template_id=MISSION_ID,
    )

    assert plan["status"] == "ready_to_author"
    selection = cast(dict[str, object], plan["selection"])
    assert selection["physical_family"] is None
    data_contract = cast(dict[str, object], plan["data_contract"])
    assert data_contract["model_kind"] == "trajectory_workflow"
    assert data_contract["source_refs"]
    assert data_contract["core_output_channels"]
    controller = cast(dict[str, object], plan["controller_automation"])
    assert controller["status"] == "provider_managed"
    assert controller["control_status"] == "internally_generated"
    channels = cast(list[dict[str, Any]], controller["channels"])
    assert {item["id"] for item in channels} == {"command.bank", "command.throttle"}
    assert all(item["availability"] == "available_in_batch" for item in channels)
    assert controller["campaigns"] == []
    assert cast(dict[str, object], plan["segment_automation"])["instances"]
    ####


def test_simple_aero_fixed_ld_baseline_executes_through_the_public_common_runner() -> None:
    """The advertised workflow yields typed status, resource, control, and aero output."""

    provider = RegistryMissionCompositionProvider()
    prepared = _prepared(provider)
    request = MissionCompositionRunRequest(
        request_id="simple-aero-vertical-fixed-ld",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=17),
    )
    response = build_registry_mission_composition_runner(provider).run(request)

    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    assert response.result.status == "completed"
    assert response.result.primary_model_id == MODEL_ID
    assert len(response.result.objects) == 1
    primary = response.result.objects[0]
    assert len(primary.samples) == 17
    assert {item.id for item in primary.channels} == {
        item.id for item in provider.get_model_output_schema(MODEL_ID).channels
    }
    assert all(set(item.values) == {channel.id for channel in primary.channels} for item in primary.samples)
    assert all(math.isfinite(float(item.values["mass.total"])) for item in primary.samples)
    assert any(float(item.values["propulsion.thrust"]) > 0.0 for item in primary.samples)
    assert all(0.0 <= float(item.values["propulsion.throttle_command"]) <= 1.0 for item in primary.samples)
    ####


def test_simple_aero_published_templates_have_exact_fixed_ld_lowerings() -> None:
    """Every advertised recipe is buildable, not merely schema-valid fixture text."""

    provider = RegistryMissionCompositionProvider()
    model = provider.model(MODEL_ID)
    template_ids = tuple(item.id for item in model.mission_templates)

    assert template_ids == (
        "fixed_ld_baseline",
        "ballistic",
        "cbcr",
        "crossrange",
        "marv",
        "phugoid",
        "range_extension",
        "skip",
        "slalom",
        "weave",
    )
    for template_id in template_ids:
        prepared = _prepared_template(provider, template_id)
        build = build_simple_aero_prepared_configuration(prepared)
        operation = next(
            item
            for mission in model.mission_templates
            if mission.id == template_id
            for item in mission.operations
            if item.operation == "batch"
        )
        assert build.derived.total_duration_s > 0.0
        assert f"source Simple Aero segment {prepared.resolved['segments'][0]['selected']}" in build.problem_text
        assert operation.status == "available"
        assert operation.common_runner_status == "registered"
    ####


def test_simple_aero_phugoid_template_executes_through_the_same_public_runner() -> None:
    """The alpha-profile branch reaches the real batch runner with normalized output."""

    provider = RegistryMissionCompositionProvider()
    prepared = _prepared_template(provider, "phugoid")
    response = build_registry_mission_composition_runner(provider).run(
        MissionCompositionRunRequest(
            request_id="simple-aero-phugoid-vertical",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=7),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    assert response.result.status == "completed"
    primary = response.result.objects[0]
    assert primary.realization_id == REALIZATION_ID
    assert primary.fidelity == FIDELITY
    assert len(primary.samples) == 7
    assert {item.id for item in primary.channels} == {
        item.id for item in provider.get_model_output_schema(MODEL_ID).channels
    }
    assert all(math.isfinite(float(item.values["aerodynamics.angle_of_attack"])) for item in primary.samples)
    ####


def test_simple_aero_advertises_and_enforces_its_batch_only_boundary() -> None:
    """An unavailable interactive route returns the published, actionable blocker."""

    provider = RegistryMissionCompositionProvider()
    model = provider.model(MODEL_ID)
    baseline = next(item for item in model.mission_templates if item.id == MISSION_ID)
    operations = {item.operation: item for item in baseline.operations}
    batch = operations["batch"]
    step = operations["step"]

    assert batch.status == "available"
    assert batch.common_runner_status == "registered"
    assert step.status == "blocked"
    assert step.common_runner_status == "not_available"
    assert step.blockers == ("no interactive Simple Aero Mission Composition session binding is registered",)

    with pytest.raises(MissionCompositionExecutionError) as error:
        MissionCompositionSessionManager(provider).open(
            MissionCompositionOpenSessionRequest(
                session_id="simple-aero-workflow-boundary",
                provider_id=provider.metadata.id,
                provider_version=provider.metadata.version,
                prepared_configuration=_prepared(provider),
            )
        )

    assert error.value.diagnostic.code == "interactive-operation-not-available"
    assert error.value.diagnostic.phase == "preflight"
    assert error.value.diagnostic.details["blockers"] == list(step.blockers)
    ####
